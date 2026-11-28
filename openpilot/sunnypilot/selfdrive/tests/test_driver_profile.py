"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import dataclasses
import datetime
import json
import os
import tempfile
from types import SimpleNamespace
from typing import cast

from openpilot.cereal import custom, log
from opendbc.car import structs
from opendbc.car.structs import car
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, Events
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.sunnypilot.selfdrive import driver_profile as dp
from openpilot.sunnypilot.selfdrive.car.genesis_mode import GenesisModeButton
from openpilot.sunnypilot.selfdrive.car.hyundai_takeover import steer_fault_alert_window
from openpilot.sunnypilot.selfdrive.car.speed_limit_follow import SpeedLimitFollow
from openpilot.sunnypilot.selfdrive.controls.lib import e2e_alerts_helper, gas_resume, over_speed_alert, red_light_alert, stop_cue
from openpilot.sunnypilot.selfdrive.controls.lib.gas_resume import GasResume
from openpilot.sunnypilot.selfdrive.controls.lib.red_light_alert import red_ahead_not_slowing
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import vote
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

EventName = log.OnroadEvent.EventName
SETTINGS_UI = os.path.join(os.path.dirname(__file__), "..", "..", "sunnylink", "settings_ui.json")


def profile_with(**sections) -> dp.DriverProfile:
  """DEFAULT_PROFILE with some keys replaced: profile_with(gas_resume={"enabled": False})."""
  return dataclasses.replace(dp.DEFAULT_PROFILE, **{s: dataclasses.replace(getattr(dp.DEFAULT_PROFILE, s), **kv) for s, kv in sections.items()})


class TestDefaultsAreTheForkConstants(OpenpilotTestCase):
  """The cutover guard: a missing DriverProfile drives exactly as the constants did at 25615e941."""

  def test_values(self):
    p = dp.DEFAULT_PROFILE
    assert (p.wheel_button.long_press, p.wheel_button.hold_s) == ("genesis_mode", 1.0)
    assert p.main_button.long_press == "speed_limit_follow"
    assert dataclasses.astuple(p.red_light) == (10.0, -0.5, 80.0, 10.0, 0.3, 5.0, 3.0, 6.0, 2.0, 1.3)
    assert dataclasses.astuple(p.stop_cue) == (True, 4.5, 25.0, -2.5)
    assert dataclasses.astuple(p.green_light) == (1.0, 1.0)
    assert dataclasses.astuple(p.cues) == (True, True)
    assert dataclasses.astuple(p.alerts) == (True, True)
    assert dataclasses.astuple(p.speed_limit) == ("America/Los_Angeles", (0, 1, 2, 3, 4), ("07:00", "17:00"), 25, 35, 55, 0.5)
    assert dataclasses.astuple(p.over_speed) == (2.0, 5.0)
    assert dataclasses.astuple(p.gas_resume) == (True, 0.5, 2.0, -0.5, 2.5, -0.8)
    assert dataclasses.astuple(p.soft_stop) == (1.15, 5.0)

  def test_module_names_keep_their_old_values(self):
    assert red_light_alert.MIN_SPEED == 10 * CV.MPH_TO_MS
    assert (red_light_alert.MIN_ACCEL, red_light_alert.STOP_DIST_MAX, red_light_alert.LEAD_STOP_MARGIN) == (-0.5, 80.0, 10.0)
    assert (red_light_alert.HOLD_TIME, red_light_alert.REARM_CLEAR_TIME, red_light_alert.BRAKE_RECENT_TIME) == (0.3, 5.0, 3.0)
    assert (red_light_alert.GAS_LIFT_TIME, red_light_alert.LATE_DECEL, red_light_alert.REACTION_TIME) == (6.0, 2.0, 1.3)
    assert (stop_cue.STOP_SIGN_MIN_SPEED, stop_cue.BRAKE_NOW_MIN_SPEED, stop_cue.BRAKE_NOW_ACCEL) == (4.5, 25 * CV.MPH_TO_MS, -2.5)
    assert (e2e_alerts_helper.GO_WAIT, e2e_alerts_helper.BRAKE_GO_LOOKBACK) == (1.0, 1.0)
    assert (over_speed_alert.OVER_SPEED_HOLD_TIME, over_speed_alert.OVER_SPEED_REARM_CLEAR_TIME) == (2.0, 5.0)
    assert (gas_resume.GAS_RESUME_HOLD, gas_resume.GAS_RESUME_DECEL_TIME, gas_resume.GAS_RESUME_MAX_DECEL) == (0.5, 2.0, -0.5)
    assert (gas_resume.LEAD_EASE_TIME, gas_resume.LEAD_EASE_MAX_DECEL) == (2.5, -0.8)
    assert vote.LOCAL_TZ.key == "America/Los_Angeles"
    assert (vote.MPH.school_max, vote.MPH.school_base_min) == (25, 35)

  def test_every_default_is_inside_its_range(self):
    for name, cls in dp.SECTIONS.items():
      for f in dataclasses.fields(cls):
        if "range" in f.metadata and not isinstance(f.default, tuple):
          lo, hi = f.metadata["range"]
          assert lo <= f.default <= hi, f"{name}.{f.name}"
        if "choices" in f.metadata:
          assert f.default in f.metadata["choices"], f"{name}.{f.name}"

  def test_numeric_keys_carry_a_unit_suffix(self):
    unitless = {"low_speed_decel_gain", "school_days", "school_hours"}
    for cls in dp.SECTIONS.values():
      for f in dataclasses.fields(cls):
        if "range" in f.metadata and f.name not in unitless:
          assert f.name.endswith(("_s", "_mps", "_mps2", "_mph", "_m")), f.name

  def test_round_trip(self):
    assert dp.parse(json.loads(json.dumps(dp.to_dict(dp.DEFAULT_PROFILE)))) == (dp.DEFAULT_PROFILE, [])


class TestParse(OpenpilotTestCase):
  def test_missing_and_empty_are_defaults(self):
    assert dp.parse(None) == (dp.DEFAULT_PROFILE, [])
    assert dp.parse({}) == (dp.DEFAULT_PROFILE, [])
    assert dp.parse({"schema_version": 1, "red_light": {}}) == (dp.DEFAULT_PROFILE, [])

  def test_out_of_range_is_clamped_per_key(self):
    p, problems = dp.parse({"soft_stop": {"low_speed_decel_gain": 0.8}, "red_light": {"hold_s": 9.0, "min_speed_mph": 12}})
    assert p.soft_stop.low_speed_decel_gain == 1.0  # never softer
    assert p.red_light.hold_s == 1.0 and p.red_light.min_speed_mph == 12.0
    assert len(problems) == 2 and all("clamped" in m for m in problems)

  def test_strict_refuses_instead_of_clamping(self):
    p, problems = dp.parse({"soft_stop": {"low_speed_decel_gain": 0.8}}, strict=True)
    assert p.soft_stop.low_speed_decel_gain == 1.15 and "outside" in problems[0]

  def test_wrong_types_take_the_default(self):
    p, problems = dp.parse({"gas_resume": {"enabled": "yes", "hold_s": "1"}, "speed_limit": {"freeway_min_mph": 60.5},
                            "wheel_button": {"long_press": "toggle"}, "cues": {"bookmark": 1}})
    assert p.gas_resume == dp.DEFAULT_PROFILE.gas_resume
    assert p.speed_limit.freeway_min_mph == 55 and p.wheel_button.long_press == "genesis_mode" and p.cues.bookmark
    assert len(problems) == 5

  def test_unknown_keys_and_sections_are_ignored_and_reported(self):
    p, problems = dp.parse({"red_light": {"min_speed": 20}, "lasers": {"on": True}})
    assert p == dp.DEFAULT_PROFILE and len(problems) == 2

  def test_limit_ahead_decel_is_off_or_at_least_its_minimum(self):
    for raw, want in ((0, 0.), (0.0, 0.), (0.05, 0.2), (0.2, 0.2), (1.0, 1.0), (9, 1.5), (-1, 0.)):
      p, problems = dp.parse({"speed_limit": {"limit_ahead_decel_mps2": raw}})
      assert p.speed_limit.limit_ahead_decel_mps2 == want and len(problems) == (raw not in (0, 0.2, 1.0)), raw
    p, problems = dp.parse({"speed_limit": {"limit_ahead_decel_mps2": 0.05}}, strict=True)
    assert p.speed_limit.limit_ahead_decel_mps2 == 0.5 and "between 0 (off) and 0.2" in problems[0]

  def test_unknown_schema_version_is_all_defaults(self):
    p, problems = dp.parse({"schema_version": 2, "soft_stop": {"low_speed_decel_gain": 1.3}})
    assert p == dp.DEFAULT_PROFILE and len(problems) == 1

  def test_not_an_object(self):
    assert dp.parse([1, 2])[0] == dp.DEFAULT_PROFILE and dp.parse("x")[1]

  def test_school_rule_group_fallbacks(self):
    p, problems = dp.parse({"speed_limit": {"school_hours": ["17:00", "07:00"], "school_days": [0, 7], "time_zone": "Mars/Olympus"}})
    assert p.speed_limit.school_hours == ("07:00", "17:00") and p.speed_limit.school_days == (0, 1, 2, 3, 4)
    assert p.speed_limit.time_zone == "America/Los_Angeles" and len(problems) == 3  # a bad zone keeps the default zone

  def test_bad_time_zones_keep_the_default_and_empty_opts_out(self):
    for tz in (None, 5, ["UTC"], "Mars/Olympus", "america/los angeles", "../etc/passwd", "UTC\x00", "x" * 300):
      p, problems = dp.parse({"speed_limit": {"time_zone": tz}})
      assert p.speed_limit.time_zone == "America/Los_Angeles" and len(problems) == 1, tz
    assert dp.parse({"speed_limit": {"time_zone": ""}}) == (profile_with(speed_limit={"time_zone": ""}), [])
    assert dp.parse({"speed_limit": {"time_zone": "Europe/Berlin"}})[1] == []

  def test_school_hours_accept_only_ascii_hh_mm(self):
    for bad in (" 7:00", "+7:00", "07: 0", "-0:00", "\uff10\uff17:\uff10\uff10", "7:00", "07:0", "07-00", "24:00", "07:60", "07:00 ", 700, None):
      assert dp.parse_hhmm(bad) is None, bad
      p, problems = dp.parse({"speed_limit": {"school_hours": [bad, "17:00"]}})
      assert p.speed_limit.school_hours == ("07:00", "17:00") and len(problems) == 1, bad
    assert dp.parse_hhmm("00:00") == datetime.time(0, 0) and dp.parse_hhmm("23:59") == datetime.time(23, 59)
    assert dp.parse({"speed_limit": {"school_hours": ["06:45", "15:30"]}})[0].speed_limit.school_hours == ("06:45", "15:30")

  def test_nan_is_a_wrong_type(self):
    p, problems = dp.parse({"gas_resume": {"max_decel_mps2": float("nan")}})
    assert p.gas_resume.max_decel_mps2 == -0.5 and problems

  def test_huge_ints(self):
    p, problems = dp.parse({"gas_resume": {"max_decel_mps2": 10 ** 400}, "speed_limit": {"freeway_min_mph": 10 ** 5000}})
    assert p.gas_resume.max_decel_mps2 == -0.5  # not a float: the default
    assert p.speed_limit.freeway_min_mph == 75  # an int: clamped
    assert len(problems) == 2 and all(len(m) < 200 for m in problems)
    assert len(dp.check_params({"HyundaiTakeoverTorque": 10 ** 5000, "HyundaiTakeoverReleaseTorque": 10 ** 5001})) == 3


ADVERSARIAL = [None, True, False, 0, 1, -1, 7, 2 ** 63, 10 ** 400, -10 ** 400, 10 ** 5000, 0.5, -0.5, 1e308, -1e308, float("nan"), float("inf"),
               "", " 7:00", "+7:00", "07: 0", "-0:00", "\uff10\uff17:\uff10\uff10", "07:00", "17:00", "23:59", "24:00", "UTC",
               "America/Los_Angeles", "Mars/Olympus", "../etc/passwd", "UTC\x00", "genesis_mode", "none", "plain", "speed_limit_follow",
               [], [0], [6, 0], [0, 7], [0, 0], [True], ["07:00", "17:00"], ["17:00", "07:00"], ["07:00", " 8:00"], ["00:00", "23:59"], {},
               {"a": 1}, "x" * 10000]


def build_consumers(p: dp.DriverProfile) -> None:
  """Everything that takes a profile section (planner helpers, vote, card and selfdrived parts), one step each where cheap."""
  now = datetime.datetime(2026, 11, 24, 16, 30, tzinfo=datetime.UTC)
  v = vote.SpeedLimitVote(0.05, p.speed_limit)
  v.update(11.0, 11.0, 15.6, True, 10.0, False, now)
  vote.in_school_hours(now, v.school_rule)
  red_light_alert.RedLightAlertHelper(True, p.red_light)
  red_ahead_not_slowing(15.0, 0.1, False, 40.0, 2.0, 3.0, False, p.red_light)
  stop_cue.StopCueHelper(True, p)
  e2e_alerts_helper.E2EAlertsHelper(p.green_light)
  over_speed_alert.OverSpeedAlertHelper(p.over_speed).alert.step(30.0, 20.0, 10, False)
  g = GasResume(0.05, p.gas_resume)
  g.update(False, 30.0, 25.0, -1.0)
  g.ease_lead(-1.0, (SimpleNamespace(present=True, dRel=40.0, vRel=-3.0, aLeadK=0.0),), 30.0, 1.45)
  GenesisModeButton(SimpleNamespace(carFingerprint="GENESIS_GV70_2ND_GEN"), SimpleNamespace(), p.wheel_button)
  params = cast(Params, SimpleNamespace(get=lambda k, return_default=False: {"SpeedLimitMode": Mode.warning}.get(k, 5)))
  SpeedLimitFollow(structs.CarParams(brand="hyundai", pcmCruise=False), structs.CarParamsSP(), params, p)
  Events(p.alerts.quiet_silent_steer_fault, p.alerts.quiet_brake_disengage)


def gv70_planner(p: dp.DriverProfile) -> LongitudinalPlanner:
  CP = CarInterface.get_non_essential_params(CAR.GENESIS_GV70_2ND_GEN)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.GENESIS_GV70_2ND_GEN)
  CP.openpilotLongitudinalControl = True
  return LongitudinalPlanner(CP, CP_SP, profile=p)


class TestEveryAcceptedValueIsUsable(OpenpilotTestCase):
  """Property: for every key and every adversarial value, parse() never raises, the result is canonical (its JSON parses back strictly to
  itself) and every consumer can be built from it, so no value can stop a process at onroad start."""

  def test_every_key_and_value(self):
    for name, cls in dp.SECTIONS.items():
      for f in dataclasses.fields(cls):
        last_accepted = None
        for raw in ADVERSARIAL:
          for strict in (False, True):
            p, problems = dp.parse({"schema_version": 1, name: {f.name: raw}}, strict=strict)
            assert dp.parse(json.loads(json.dumps(dp.to_dict(p))), strict=True) == (p, []), (name, f.name, raw)
            build_consumers(p)
            if not problems:
              last_accepted = p
        if last_accepted is not None:
          gv70_planner(last_accepted)

  def test_top_level_garbage(self):
    for raw in ADVERSARIAL:
      for doc in (raw, {"schema_version": raw}, {"red_light": raw}, {raw if isinstance(raw, str) else "k": raw}):
        build_consumers(dp.parse(doc)[0])


NON_DEFAULT = profile_with(
  wheel_button={"long_press": "none", "hold_s": 2.0}, main_button={"long_press": "plain"},
  red_light={"min_speed_mph": 20.0, "hold_s": 0.5}, stop_cue={"enabled": False, "brake_now_accel_mps2": -3.0},
  green_light={"go_wait_s": 2.0, "brake_release_lookback_s": 0.5}, cues={"bookmark": False, "in_dashcam_mode": False},
  alerts={"quiet_silent_steer_fault": False, "quiet_brake_disengage": False},
  speed_limit={"time_zone": "Europe/Berlin", "school_zone_max_mph": 20, "school_street_min_mph": 40, "freeway_min_mph": 60},
  over_speed={"hold_s": 4.0, "rearm_clear_s": 10.0}, gas_resume={"enabled": False, "lead_ease_s": 1.0},
  soft_stop={"low_speed_decel_gain": 1.25, "low_speed_decel_speed_mps": 6.0})


class TestProcessWiring(OpenpilotTestCase):
  """Each process passes its loaded profile's sections to the right feature."""

  def test_planner(self):
    lp = gv70_planner(NON_DEFAULT)
    p = NON_DEFAULT
    assert lp.red_light_alert_helper.cfg is p.red_light
    assert lp.stop_cue_helper.cfg is p.stop_cue and lp.stop_cue_helper.red_light is p.red_light and not lp.stop_cue_helper.enabled
    assert lp.e2e_alerts_helper.cfg is p.green_light
    assert lp.over_speed_alert_helper.alert.hold_frames == round(4.0 / DT_MDL)
    assert lp.over_speed_alert_helper.alert.rearm_frames == round(10.0 / DT_MDL)
    assert lp.resolver._vote.school_rule.tz is not None and lp.resolver._vote.school_rule.tz.key == "Europe/Berlin"
    assert (lp.resolver._vote.mph.school_max, lp.resolver._vote.mph.school_base_min) == (20, 40)
    assert lp.gas_resume.cfg is p.gas_resume
    assert lp.soft_stop is p.soft_stop and lp._low_speed_decel_gain(3.0) == 1.25

  def test_selfdrived(self):
    Params().put(dp.PARAM_KEY, dp.to_dict(NON_DEFAULT), block=True)
    CP = car.CarParams.new_message(brand="hyundai", openpilotLongitudinalControl=True)
    sd = SelfdriveD(CP=CP, CP_SP=custom.CarParamsSP.new_message())
    assert sd.profile == NON_DEFAULT
    assert sd.events.get_events_mapping()[EventName.pedalPressed][ET.USER_DISABLE].audible_alert == log.SelfdriveState.AudibleAlert.disengage

  def test_selfdrived_other_brand_never_quiet(self):
    sd = SelfdriveD(CP=car.CarParams.new_message(brand="toyota"), CP_SP=custom.CarParamsSP.new_message())
    assert sd.profile == dp.DEFAULT_PROFILE and sd.events.get_events_mapping() is not EVENTS
    hy = SelfdriveD(CP=car.CarParams.new_message(brand="hyundai"), CP_SP=custom.CarParamsSP.new_message())
    assert hy.events.get_events_mapping() is EVENTS


class TestLoad(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params(tempfile.mkdtemp())

  def alert(self):
    return self.params.get(dp.OFFROAD_ALERT)

  def test_missing_param_is_defaults_without_alert(self):
    assert dp.load(self.params) == dp.DEFAULT_PROFILE
    assert self.alert() is None

  def test_invalid_json_is_defaults_with_alert(self):
    os.makedirs(os.path.dirname(self.params.get_param_path(dp.PARAM_KEY)), exist_ok=True)
    with open(self.params.get_param_path(dp.PARAM_KEY), "w") as f:
      f.write("{not json")
    assert dp.load(self.params) == dp.DEFAULT_PROFILE
    assert self.alert()["extra"].startswith("1: ")

  def test_clamped_value_is_used_and_reported_then_cleared(self):
    self.params.put(dp.PARAM_KEY, {"schema_version": 1, "gas_resume": {"lead_max_decel_mps2": -5.0}}, block=True)
    assert dp.load(self.params).gas_resume.lead_max_decel_mps2 == -2.0
    assert "lead_max_decel_mps2" in self.alert()["extra"]
    self.params.put(dp.PARAM_KEY, dp.to_dict(dp.DEFAULT_PROFILE), block=True)
    assert dp.load(self.params) == dp.DEFAULT_PROFILE
    assert self.alert() is None

  def test_read_only_load_never_writes_the_alert(self):
    self.params.put(dp.PARAM_KEY, {"lasers": 1}, block=True)
    assert dp.load(self.params, report=False) == dp.DEFAULT_PROFILE
    assert self.alert() is None


class TestParamSchema(OpenpilotTestCase):
  def test_ranges_equal_settings_ui(self):
    with open(SETTINGS_UI) as f:
      ui = json.load(f)
    widgets = {}

    def walk(node):
      if isinstance(node, dict):
        if "key" in node and "widget" in node:
          widgets[node["key"]] = node
        for v in node.values():
          walk(v)
      elif isinstance(node, list):
        for v in node:
          walk(v)
    walk(ui)
    for key, spec in dp.PARAM_SCHEMA.items():
      w = widgets.get(key)
      if w is None:
        continue  # SpeedLimitFollowOffset*: no widget, the range is speed_limit_follow.OFFSET_RANGE
      if spec.kind is bool:
        assert w["widget"] == "toggle", key
      elif w["widget"] == "option":
        assert (w["min"], w["max"], w.get("step", 1)) == (spec.lo, spec.hi, spec.step), key
      else:
        assert [o["value"] for o in w["options"]] == list(range(spec.lo, spec.hi + 1)), key

  def test_follow_offset_range_matches_the_feature(self):
    from openpilot.sunnypilot.selfdrive.car.speed_limit_follow import OFFSET_RANGE
    for key in ("SpeedLimitFollowOffsetStreet", "SpeedLimitFollowOffsetFreeway"):
      assert (dp.PARAM_SCHEMA[key].lo, dp.PARAM_SCHEMA[key].hi) == OFFSET_RANGE

  def test_check_params(self):
    assert dp.check_params({"HyundaiTakeoverTorque": 150, "HyundaiTakeoverReleaseTorque": 100, "RedLightAlert": True}) == []
    assert len(dp.check_params({"HyundaiResumeAngleRate": 23})) == 1
    assert len(dp.check_params({"HyundaiTakeoverTorque": 155})) == 1  # step 10
    assert len(dp.check_params({"HyundaiTakeoverTorque": 100, "HyundaiTakeoverReleaseTorque": 150})) == 1
    assert dp.check_params({"HyundaiTakeoverTorque": 0, "HyundaiTakeoverReleaseTorque": 150}) == []  # filter off
    assert len(dp.check_params({"RedLightAlert": 1, "Foo": 3})) == 2


class TestProfileReachesTheFeatures(OpenpilotTestCase):
  def test_gas_resume_off_is_the_upstream_planner(self):
    g = GasResume(0.05, profile_with(gas_resume={"enabled": False}).gas_resume)
    for _ in range(20):
      g.update(True, 30., 25., -2.0)
    assert g.update(False, 30., 25., -2.0) == -2.0
    assert g.ease_lead(-1.5, (), 30., 1.45) == -1.5

  def test_low_speed_decel_gain(self):
    CP = CarInterface.get_non_essential_params(CAR.GENESIS_GV70_2ND_GEN)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.GENESIS_GV70_2ND_GEN)
    CP.openpilotLongitudinalControl = True
    assert LongitudinalPlanner(CP, CP_SP)._low_speed_decel_gain(3.0) == 1.15
    assert LongitudinalPlanner(CP, CP_SP, profile=profile_with(soft_stop={"low_speed_decel_gain": 1.0}))._low_speed_decel_gain(3.0) == 1.0

  def test_red_light_threshold(self):
    cfg = profile_with(red_light={"min_speed_mph": 30.0}).red_light
    assert red_ahead_not_slowing(20 * CV.MPH_TO_MS, 0.0, False, 40.0)
    assert not red_ahead_not_slowing(20 * CV.MPH_TO_MS, 0.0, False, 40.0, cfg=cfg)

  def test_empty_time_zone_turns_the_school_rule_off(self):
    weekday_school = datetime.datetime(2026, 11, 24, 8, 30, tzinfo=vote.LOCAL_TZ)
    v = vote.SpeedLimitVote(0.05, profile_with(speed_limit={"time_zone": ""}).speed_limit)
    assert not vote.in_school_hours(weekday_school, v.school_rule)
    assert vote.in_school_hours(weekday_school)

  def test_wheel_button_none_never_switches(self):
    params = SimpleNamespace(store={}, remove=lambda k: None, get_bool=lambda k: True, put_bool=lambda k, v, block=False: None)
    btn = GenesisModeButton(SimpleNamespace(carFingerprint="GENESIS_GV70_2ND_GEN"), params, profile_with(wheel_button={"long_press": "none"}).wheel_button)
    cs = SimpleNamespace(gearShifter=structs.CarState.GearShifter.park, vEgo=0.0)
    assert not any(btn.update(p, cs, False) for p in [False] * 100 + [True] * 300)

  def test_main_button_plain_turns_the_follow_off(self):
    params = cast(Params, SimpleNamespace(get=lambda k, return_default=False: {"SpeedLimitMode": Mode.warning}.get(k, 5)))
    CP = structs.CarParams(brand="hyundai", pcmCruise=False)
    CP_SP = structs.CarParamsSP(safetyParam=int(HyundaiSafetyFlagsSP.MAIN_BUTTON_ENGAGE))
    assert SpeedLimitFollow(CP, CP_SP, params).available
    assert not SpeedLimitFollow(CP, CP_SP, params, profile_with(main_button={"long_press": "plain"})).available

  def test_alert_quieting_off_is_upstream(self):
    quiet, upstream = Events(), Events(False, False)
    assert quiet.get_events_mapping() is EVENTS
    steer = upstream.get_events_mapping()[EventName.steerTempUnavailableSilent][ET.WARNING]
    assert steer.audible_alert == log.SelfdriveState.AudibleAlert.prompt
    disengage = upstream.get_events_mapping()[EventName.pedalPressed][ET.USER_DISABLE]
    assert disengage.audible_alert == log.SelfdriveState.AudibleAlert.disengage
    assert EVENTS[EventName.steerTempUnavailableSilent][ET.WARNING].audible_alert == log.SelfdriveState.AudibleAlert.none  # not mutated

  def test_steer_fault_window_is_upstream_without_the_takeover_filter(self):
    def window(brand, torque):
      CP, CP_SP = structs.CarParams(brand=brand), custom.CarParamsSP(hyundaiTakeoverTorque=torque)
      return steer_fault_alert_window(CP, CP_SP), CarEvents(CP, CP_SP).steer_fault_window_frames
    assert window("hyundai", 150) == (2.0, int(2.0 / DT_CTRL))
    assert window("hyundai", 0) == (1.5, int(1.5 / DT_CTRL))
    assert window("toyota", 150) == (1.5, int(1.5 / DT_CTRL))


class TestSafety(OpenpilotTestCase):
  def test_no_safety_bit_depends_on_the_settings(self):
    """The [param] keys the profile writes and DriverProfile itself, through opendbc's params path: safetyParam and the flags that feed it stay."""
    def build(params_list):
      CP = CarInterface.get_non_essential_params(CAR.GENESIS_GV70_2ND_GEN)
      CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.GENESIS_GV70_2ND_GEN)
      setup_interfaces(CarInterface, CP, CP_SP, params_list)
      return CP, CP_SP
    driver = {"HyundaiSteerFeel": "3", "HyundaiCurveHold": "1", "HyundaiTakeoverTorque": "150", "HyundaiTakeoverReleaseTorque": "100",
              "HyundaiTakeoverHold": "500", "HyundaiResumeAngleRate": "22", dp.PARAM_KEY: json.dumps(dp.to_dict(dp.DEFAULT_PROFILE))}
    (cp0, sp0), (cp1, sp1) = build([]), build([driver])
    assert sp0.safetyParam == sp1.safetyParam
    assert [c.safetyParam for c in cp0.safetyConfigs] == [c.safetyParam for c in cp1.safetyConfigs]
