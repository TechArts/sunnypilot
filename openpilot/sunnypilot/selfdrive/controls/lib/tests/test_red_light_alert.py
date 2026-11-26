"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import red_light_alert
from openpilot.sunnypilot.selfdrive.controls.lib.red_light_alert import (
  RedLightAlertHelper, RedLightStates, model_plans_stop, red_ahead_not_slowing, HOLD_TIME, REARM_CLEAR_TIME, MIN_SPEED, BRAKE_RECENT_TIME,
  GAS_LIFT_TIME,
)

IDX_N = 33
V_CRUISING = 20.0
FRAMES_HOLD = round(HOLD_TIME / DT_MDL)
FRAMES_REARM = round(REARM_CLEAR_TIME / DT_MDL)


class FakeParams:
  def get_bool(self, key):
    return True


@pytest.fixture
def helper(monkeypatch):
  monkeypatch.setattr(red_light_alert, "Params", FakeParams)
  return RedLightAlertHelper(openpilot_longitudinal=True)


def path(stop_dist=None, v_end=0.0):
  """Model path: cruising, or ending at v_end after stop_dist metres."""
  if stop_dist is None:
    return [V_CRUISING * i for i in range(IDX_N)], [V_CRUISING] * IDX_N
  return [stop_dist * i / (IDX_N - 1) for i in range(IDX_N)], [V_CRUISING * (1 - i / (IDX_N - 1)) + v_end for i in range(IDX_N)]


def run(helper, condition, frames, v_ego=V_CRUISING):
  return [helper.step(condition, v_ego) for _ in range(frames)]


class TestModelPlansStop:
  def test_stop_ahead(self):
    x, v = path(stop_dist=60.0)
    assert model_plans_stop(x, v, V_CRUISING, False, 0.0)

  def test_cruising_model(self):
    x, v = path()
    assert not model_plans_stop(x, v, V_CRUISING, False, 0.0)

  def test_stop_too_far(self):
    x, v = path(stop_dist=150.0)
    assert not model_plans_stop(x, v, V_CRUISING, False, 0.0)

  def test_model_not_stopping_fully(self):
    x, v = path(stop_dist=60.0, v_end=6.0)
    assert not model_plans_stop(x, v, V_CRUISING, False, 0.0)

  def test_lead_explains_stop(self):
    x, v = path(stop_dist=60.0)
    assert not model_plans_stop(x, v, V_CRUISING, True, 55.0)
    assert model_plans_stop(x, v, V_CRUISING, True, 120.0)

  def test_empty_model(self):
    assert not model_plans_stop([], [], V_CRUISING, False, 0.0)


STOP_FAR = 70.0   # m, the model's stop point
V_CITY = 12.0     # m/s (27 mph): v^2 / 2d = 1.03 m/s^2 needed at 70 m, below LATE_DECEL


class TestRedAheadNotSlowing:
  def test_baseline(self):
    assert red_ahead_not_slowing(V_CITY, 0.0, False, STOP_FAR)

  def test_each_gate(self):
    assert not red_ahead_not_slowing(MIN_SPEED - 0.1, 0.0, False, STOP_FAR)  # too slow
    assert not red_ahead_not_slowing(V_CITY, -0.6, False, STOP_FAR)          # already decelerating
    assert not red_ahead_not_slowing(V_CITY, 0.0, True, STOP_FAR)            # braking
    assert not red_ahead_not_slowing(V_CITY, 0.0, False, None)               # no stop planned

  def test_fires_at_12_mph(self):
    # 10 mph threshold: a red light approached at 12 mph without slowing is cued (was silent under 15 mph)
    assert red_ahead_not_slowing(12 * CV.MPH_TO_MS, 0.0, False, STOP_FAR)

  def test_silent_while_easing_in(self):
    # driver 2026-11-25 (a3 9:32.8, a4 5:19.4): cued while already stopping by brake taps or a coast after lifting the gas
    assert not red_ahead_not_slowing(V_CITY, -0.3, False, STOP_FAR, since_brake=BRAKE_RECENT_TIME - 0.5)
    assert not red_ahead_not_slowing(V_CITY, -0.15, False, STOP_FAR, since_gas=GAS_LIFT_TIME - 2.0)

  def test_coasting_long_after_the_gas_still_cues(self):
    assert red_ahead_not_slowing(V_CITY, -0.15, False, STOP_FAR, since_gas=GAS_LIFT_TIME + 1.0)

  def test_gas_lift_with_car_not_slowing_still_cues(self):
    # gas just lifted but the car not decelerating (a6 1:21.4 pattern: on the gas toward the light): still cued
    assert red_ahead_not_slowing(V_CITY, 0.1, False, STOP_FAR, since_gas=0.5)

  def test_silent_while_openpilot_controls_speed(self):
    assert not red_ahead_not_slowing(V_CITY, -0.3, False, STOP_FAR, long_active=True)

  def test_late_stop_threshold_while_openpilot_drives(self):
    # slowing (no reaction margin): 12 m/s needs 144 / 70 = 2.06 m/s^2 at 35 m (cue), 1.95 at 37 m (silent)
    assert red_ahead_not_slowing(V_CITY, -0.1, False, 35.0, long_active=True)
    assert not red_ahead_not_slowing(V_CITY, -0.1, False, 37.0, long_active=True)

  def test_late_stop_counts_the_reaction_when_not_slowing(self):
    # not slowing: 1.3 s at 12 m/s (15.6 m) is gone before braking starts, so the cue comes at 51 m, not 35 m
    assert red_ahead_not_slowing(V_CITY, 0.0, False, 51.0, long_active=True)
    assert not red_ahead_not_slowing(V_CITY, 0.0, False, 53.0, long_active=True)

  def test_stop_at_zero_distance_cues(self):
    assert red_ahead_not_slowing(V_CITY, -0.1, False, 0.0, since_brake=1.0, long_active=True)

  def test_gas_still_held_cues(self):
    # R1 1: since_gas is 0 while the gas is pressed; a slight decel as the throttle builds is not a coast
    assert red_ahead_not_slowing(V_CITY, -0.1, False, STOP_FAR, since_gas=0.0)

  def test_gas_after_a_brake_tap_cues(self):
    # R1 1: a brake tap then back on the gas toward the light: the brake no longer counts
    assert red_ahead_not_slowing(V_CITY, 0.5, False, STOP_FAR, since_brake=1.0, since_gas=0.0)
    assert red_ahead_not_slowing(V_CITY, 0.5, False, STOP_FAR, since_brake=1.0, since_gas=0.2)


class TestRedLightDebounce:
  def test_fires_after_hold_time_only(self, helper):
    assert not any(run(helper, True, FRAMES_HOLD - 1))
    assert helper.step(True, V_CRUISING)

  def test_flicker_resets_hold_timer(self, helper):
    for _ in range(5):
      assert not any(run(helper, True, FRAMES_HOLD - 1))
      assert not helper.step(False, V_CRUISING)

  def test_one_alert_per_stop_event(self, helper):
    assert sum(run(helper, True, FRAMES_HOLD * 20)) == 1
    assert helper.state == RedLightStates.FIRED

  def test_brief_clear_does_not_rearm(self, helper):
    run(helper, True, FRAMES_HOLD)
    run(helper, False, FRAMES_REARM - 1)
    assert helper.state == RedLightStates.FIRED
    assert not any(run(helper, True, FRAMES_HOLD * 2))

  def test_clear_resets_rearm_timer(self, helper):
    run(helper, True, FRAMES_HOLD)
    run(helper, False, FRAMES_REARM - 1)
    run(helper, True, 1)
    run(helper, False, FRAMES_REARM - 1)
    assert helper.state == RedLightStates.FIRED

  def test_rearms_after_condition_clears_for_5s(self, helper):
    run(helper, True, FRAMES_HOLD)
    run(helper, False, FRAMES_REARM)
    assert helper.state == RedLightStates.ARMED
    assert sum(run(helper, True, FRAMES_HOLD)) == 1

  def test_rearms_when_car_stops(self, helper):
    run(helper, True, FRAMES_HOLD)
    helper.step(True, 0.0)
    assert helper.state == RedLightStates.ARMED
    assert sum(run(helper, True, FRAMES_HOLD)) == 1

  def test_disabled_never_fires(self, helper):
    helper.enabled = False
    assert not any(run(helper, True, FRAMES_HOLD * 5))
    assert helper.state == RedLightStates.ARMED


class TestUpdateReadsRealRadarSchema:
  """RadarState.LeadData has 'present' (no 'status'): a stand-in that only has the real fields makes any other lead access fail."""

  @staticmethod
  def sm(lead, blinker=False, long_active=False, gas=False, brake=False):
    model = SimpleNamespace(position=SimpleNamespace(x=path(stop_dist=60.0)[0]), velocity=SimpleNamespace(x=path(stop_dist=60.0)[1]))
    car_state = SimpleNamespace(vEgo=V_CITY, aEgo=0.0, brakePressed=brake, gasPressed=gas, leftBlinker=blinker, rightBlinker=False)
    return {'carState': car_state, 'modelV2': model, 'radarState': SimpleNamespace(leadOne=lead),
            'carControl': SimpleNamespace(longActive=long_active)}

  def test_read_condition_with_no_lead(self, helper):
    assert helper.read_condition(self.sm(SimpleNamespace(present=False, dRel=0.0)))

  def test_blinker_does_not_silence_it(self, helper):
    # a red light in a turn lane, blinker on, not slowing: still cued (driver's request 2026-11-25)
    assert helper.read_condition(self.sm(SimpleNamespace(present=False, dRel=0.0), blinker=True))

  def test_read_condition_with_lead_at_the_stop(self, helper):
    assert not helper.read_condition(self.sm(SimpleNamespace(present=True, dRel=55.0)))

  def test_openpilot_controlling_speed_silences_it(self, helper):
    assert not helper.read_condition(self.sm(SimpleNamespace(present=False, dRel=0.0), long_active=True))

  def test_a_brake_tap_silences_it_for_a_while(self, helper):
    none = SimpleNamespace(present=False, dRel=0.0)
    helper.read_condition(self.sm(none, brake=True))
    assert not helper.read_condition(self.sm(none))  # released, but tapped 0.05 s ago
    for _ in range(round(BRAKE_RECENT_TIME / DT_MDL)):
      helper.read_condition(self.sm(none))
    assert helper.read_condition(self.sm(none))

  def test_update_fires_with_events(self, helper):
    events = SimpleNamespace(names=[], add=lambda name: events.names.append(name))
    sm = self.sm(SimpleNamespace(present=False, dRel=0.0))
    for _ in range(FRAMES_HOLD):
      helper.update(sm, events)
    assert helper.red_light_alert
    assert len(events.names) == 1

  def test_same_light_cues_once_while_coasting(self, helper):
    # R1 2: cue, then gas released and coasting past GAS_LIFT_TIME with the same stop still ahead: one cue, not two
    events = SimpleNamespace(names=[], add=lambda name: events.names.append(name))
    none = SimpleNamespace(present=False, dRel=0.0)
    for _ in range(FRAMES_HOLD):
      helper.update(self.sm(none), events)
    helper.update(self.sm(none, gas=True), events)
    coast = self.sm(none)
    coast['carState'].aEgo = -0.15
    for _ in range(round((GAS_LIFT_TIME + 1.0) / DT_MDL)):
      helper.update(coast, events)
    assert len(events.names) == 1

  def test_stock_scc_does_not_silence_it(self, monkeypatch):
    # R1 3: longActive is also true while stock SCC drives (ICBM, no openpilot longitudinal); SCC does not stop for lights
    monkeypatch.setattr(red_light_alert, "Params", FakeParams)
    stock = RedLightAlertHelper(openpilot_longitudinal=False)
    assert stock.read_condition(self.sm(SimpleNamespace(present=False, dRel=0.0), long_active=True))
