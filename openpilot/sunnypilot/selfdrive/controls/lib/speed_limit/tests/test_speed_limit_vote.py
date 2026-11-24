"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The speed-limit vote on the cases seen on the GV70's routes 74-8d (research/speed-limit-modes-2026-11-24.md in the driver's notes).
"""
import datetime

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.vote import MPH, SpeedLimitVote, vote, in_school_hours, LOCAL_TZ, BIG_DROP_HOLD

WEEKDAY_SCHOOL = datetime.datetime(2026, 11, 24, 8, 30, tzinfo=LOCAL_TZ)  # Tuesday 08:30
SUNDAY = datetime.datetime(2026, 11, 22, 8, 40, tzinfo=LOCAL_TZ)
EVENING = datetime.datetime(2026, 11, 24, 19, 40, tzinfo=LOCAL_TZ)


def ms(mph):
  return mph * CV.MPH_TO_MS if mph else 0.


def run(v: SpeedLimitVote, seconds: float, dash, nav, osm, speed, sign=False, now=WEEKDAY_SCHOOL):
  r = None
  for _ in range(max(1, round(seconds / DT_MDL))):
    r = v.update(ms(dash), ms(nav), ms(osm), sign, ms(speed), False, now)
  return round(r.limit * CV.MS_TO_MPH), r.school


class TestVoteFrame:
  def test_truck_sign_on_a_freeway_is_ignored(self):
    # route 79, San Diego Fwy: the camera's 55 (TRUCKS 55), nav and OSM 65, car at 72
    assert vote(55, 65, 65, False, 72, MPH)[0] == 65

  def test_camera_sign_confirmed_by_osm_counts(self):
    assert vote(35, 25, 35, False, 30, MPH)[0] == 35

  def test_nav_on_a_ramp_at_speed_loses_to_osm(self):
    # route 84 71.9: nav 40, OSM 65, car at 62
    assert vote(40, 40, 65, False, 62, MPH)[:2] == (65, True)

  def test_osm_wrong_on_a_highway_keeps_the_car(self):
    # route 84 CA 138 / Chino Valley Fwy: car 65, OSM 55
    assert vote(65, 65, 55, False, 67, MPH)[0] == 65

  def test_low_car_value_at_city_speed_is_kept(self):
    assert vote(25, 25, 35, False, 20, MPH)[0] == 25

  def test_no_car_value_uses_osm(self):
    assert vote(None, None, 45, False, 40, MPH)[:2] == (45, True)

  def test_school_candidate(self):
    assert vote(25, 25, 40, False, 45, MPH)[2]  # nav 25, OSM 40 (route 79 Nordhoff)
    assert vote(20, 25, None, True, 52, MPH)[2]  # School Crossing sign, no OSM (route 8b Sheep Creek Rd)
    assert not vote(25, 25, None, False, 25, MPH)[2]  # a plain 25 street


class TestSchoolHours:
  def test_hours(self):
    assert in_school_hours(WEEKDAY_SCHOOL)
    assert not in_school_hours(SUNDAY)
    assert not in_school_hours(EVENING)
    assert not in_school_hours(datetime.datetime(2026, 11, 24, 6, 59, tzinfo=LOCAL_TZ))
    assert in_school_hours(datetime.datetime(2026, 11, 24, 15, 0, tzinfo=datetime.UTC))  # 07:00 in Los Angeles


class TestVotePersistence:
  def test_first_limit_applies_at_once(self):
    assert run(SpeedLimitVote(DT_MDL), DT_MDL, 45, 45, 45, 40) == (45, False)

  def test_increase_waits_two_seconds(self):
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 45, 45, 45, 40)
    assert run(v, 1.5, 55, 55, 55, 45)[0] == 45
    assert run(v, 0.6, 55, 55, 55, 45)[0] == 55

  def test_short_ramp_snap_never_applies(self):
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 65, 65, None, 62)  # OSM missing: the vote has only the car
    assert run(v, 3, 40, 40, None, 62)[0] == 65  # a 25 mph drop at 62 mph with one source must hold 10 s
    assert run(v, 1, 65, 65, None, 62)[0] == 65

  def test_big_drop_applies_after_its_hold_or_when_both_maps_agree(self):
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 65, 65, None, 62)
    assert run(v, BIG_DROP_HOLD + 0.1, 40, 40, None, 50)[0] == 40
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 65, 65, 65, 62)
    assert run(v, 1.1, 40, 40, 40, 55)[0] == 40  # leaving the freeway: the maps agree

  def test_school_zone_in_school_hours(self):
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 40, 40, 40, 40)
    assert run(v, 1.1, 25, 25, 40, 38, now=WEEKDAY_SCHOOL) == (25, True)

  def test_school_zone_out_of_school_hours_keeps_the_street_limit(self):
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 40, 40, 40, 45, now=SUNDAY)
    assert run(v, 5, 25, 25, 40, 45, now=SUNDAY) == (40, False)

  def test_school_sign_without_osm_on_sunday_keeps_the_previous_limit(self):
    # route 8b 10.65 min, Sheep Creek Rd, Sunday 19:40: the dash 20, nav 25, School Crossing, no OSM, car at 52
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 40, 40, None, 46, now=EVENING)
    assert run(v, 3, 20, 25, None, 52, sign=True, now=EVENING) == (40, False)

  def test_metric_thresholds(self):
    # a ramp snap in km/h: nav 60 at 100 km/h under OSM 100 -> OSM; the same at 50 km/h (city) keeps the car
    v = SpeedLimitVote(DT_MDL)
    r = v.update(60 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 100 * CV.KPH_TO_MS, False, 100 * CV.KPH_TO_MS, True, WEEKDAY_SCHOOL)
    assert round(r.limit * CV.MS_TO_KPH) == 100 and r.from_map
    v = SpeedLimitVote(DT_MDL)
    r = v.update(60 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 100 * CV.KPH_TO_MS, False, 50 * CV.KPH_TO_MS, True, WEEKDAY_SCHOOL)
    assert round(r.limit * CV.MS_TO_KPH) == 60

  def test_school_first_out_of_hours_without_osm_does_not_crash(self):
    # review: the first candidate of a drive a school one, out of hours, no OSM yet: there is no street limit to fall back to
    v = SpeedLimitVote(DT_MDL)
    assert run(v, 1, 20, 25, None, 30, sign=True, now=EVENING) == (0, False)

  def test_school_hours_ending_inside_the_zone_clears_it(self):
    v = SpeedLimitVote(DT_MDL)
    assert run(v, 1, 20, 25, None, 20, sign=True, now=WEEKDAY_SCHOOL) == (25, True)
    assert run(v, 1, 20, 25, None, 20, sign=True, now=EVENING) == (0, False)

  def test_low_nav_at_freeway_speed_is_a_ramp_not_a_school(self):
    # review: nav snapped to a <= 25 ramp at 62 mph under OSM 65 must not become a school zone
    assert vote(25, 25, 65, False, 62, MPH) == (65, True, False)
