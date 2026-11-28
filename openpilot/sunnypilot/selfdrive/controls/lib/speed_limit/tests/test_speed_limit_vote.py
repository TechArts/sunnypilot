"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The speed-limit vote on the cases seen on the GV70's routes 74-8d (research/speed-limit-modes-2026-11-24.md in the driver's notes), and OSM's
lower limit ahead on routes 83-a8 (research/speed-limit-ahead-2026-11-27.md).
"""
import dataclasses
import datetime

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.vote import (MPH, SpeedLimitVote, vote, in_school_hours, LOCAL_TZ, BIG_DROP_HOLD,
                                                                         DROP_HOLD, LIMIT_AHEAD_CONFIRM, LIMIT_AHEAD_LAG)
from openpilot.sunnypilot.selfdrive.driver_profile import DEFAULT_PROFILE

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


def drive(v: SpeedLimitVote, seconds: float, car, osm, speed, ahead=None, dist=0., now=WEEKDAY_SCHOOL, closing=True):
  """Frames at a constant speed (mph): the distance to OSM's limit ahead shrinks with the travel (grows: closing=False), as the resolver passes
  it (never below 0). Returns the frames' (limit mph, result distance m, distance to the start m) and the distance left."""
  out = []
  for _ in range(round(seconds / DT_MDL)):
    r = v.update(ms(car), ms(car), ms(osm), False, ms(speed), False, now, ms(ahead), max(0., dist))
    out.append((round(r.limit * CV.MS_TO_MPH), r.distance, dist))
    dist += ms(speed) * DT_MDL * (-1. if closing else 1.)
  return out, dist


def first_at(frames, limit):
  return next((f for f in frames if f[0] == limit), None)


class TestLimitAhead:
  """OSM's lower limit ahead (research/speed-limit-ahead-2026-11-27.md): route 91 Pearblossom Hwy 55 -> 50 at 79 mph, the car's nav 112 m late."""
  START_79 = ms(79) * LIMIT_AHEAD_LAG + (ms(79) ** 2 - ms(50) ** 2) / (2 * DEFAULT_PROFILE.speed_limit.limit_ahead_decel_mps2)  # 819 m

  def approach(self, v, car=55, osm=55, ahead=50, speed=79, dist=1200.):
    run(v, 1, car, car, osm, speed)
    return drive(v, (dist + 5.) / ms(speed), car, osm, speed, ahead, dist)

  def test_lower_limit_ahead_is_taken_where_openpilot_needs_to_start(self):
    frames, _ = self.approach(SpeedLimitVote(DT_MDL))
    assert all(f[0] == 55 for f in frames if f[2] > self.START_79)
    first = first_at(frames, 50)
    assert first is not None and self.START_79 - ms(79) * (DROP_HOLD + 0.1) <= first[2] <= self.START_79
    assert abs(first[1] - first[2]) < 1.  # not in force yet: the resolver's distance to its start
    assert all(f[0] == 50 for f in frames if f[2] < first[2])
    assert frames[-1][1] == 0.  # reached: in force

  def test_kept_while_the_car_lags_then_the_car_takes_over(self):
    v = SpeedLimitVote(DT_MDL)
    self.approach(v)
    frames, _ = drive(v, 100. / ms(79), 55, 50, 79)  # OSM switched, the nav still 55 for 100 m
    assert all(f[:2] == (50, 0.) for f in frames)
    assert run(v, 1, 50, 50, 50, 70)[0] == 50 and v.ahead is None

  def test_never_confirmed_by_the_car_goes_back(self):
    v = SpeedLimitVote(DT_MDL)
    self.approach(v)
    frames, _ = drive(v, (LIMIT_AHEAD_CONFIRM + 200.) / ms(79), 55, 50, 79)
    assert frames[0][0] == 50 and frames[-1][0] == 55  # the vote's own rule then: the car's 55 over OSM's 50

  def test_never_raised_early(self):
    frames, _ = self.approach(SpeedLimitVote(DT_MDL), car=45, osm=45, ahead=55, speed=60, dist=300.)
    assert all(f[0] == 45 for f in frames)

  def test_car_source_contradicting_osm(self):
    # route 91 46.9: nav 50, OSM 55 with 45 ahead: the two maps disagree on the current limit, nothing early
    frames, _ = self.approach(SpeedLimitVote(DT_MDL), car=50, osm=55, ahead=45, speed=67, dist=1220.)
    assert all(f[0] == 50 for f in frames)
    # the car's limit moving elsewhere during the approach lets it go
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 55, 55, 55, 79)
    drive(v, 3, 55, 55, 79, 50, self.START_79 - 10.)
    frames, _ = drive(v, 3, 60, 55, 79, 50, self.START_79 - 120.)
    assert frames[-1][0] == 60 and v.ahead is None

  def test_stale_or_missing_osm_lets_it_go(self):
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 55, 55, 55, 79)
    drive(v, 3, 55, 55, 79, 50, 500.)
    assert v.ahead is not None
    frames, _ = drive(v, 3, 55, None, 79)  # the resolver passes no OSM limit (and no ahead) when the map data is older than its limit
    assert v.ahead is None and frames[-1][0] == 55

  def test_not_for_a_school_zone_limit_a_big_drop_or_an_unposted_value(self):
    for car, ahead in ((35, 25), (65, 45), (60, 37)):  # route 93 Oxnard St 35 -> 25, a 20 mph drop, route 91 OSM "37"
      frames, _ = self.approach(SpeedLimitVote(DT_MDL), car=car, osm=car, ahead=ahead, speed=car + 5, dist=600.)
      assert all(f[0] == car for f in frames), (car, ahead)

  def test_school_rule_untouched(self):
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 40, 40, 40, 40, now=SUNDAY)
    drive(v, 2, 40, 40, 40, 35, 60., now=SUNDAY)
    assert run(v, 5, 25, 25, 40, 38, now=SUNDAY) == (40, False)  # a school zone out of hours still keeps the street's 40
    # in school hours, OSM's 25 ahead is not taken early; the car's 25 at the zone is the school-zone limit
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 40, 40, 40, 40)
    frames, _ = drive(v, 3, 40, 40, 40, 25, 100.)
    assert all(f[:2] == (40, 0.) for f in frames) and v.ahead is None
    assert run(v, 1.1, 25, 25, 40, 38) == (25, True)

  def test_cross_street_start_is_let_go(self):
    # route 92 De Soto Ave: OSM's 30 ahead, the distance growing again well before it is reached
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 40, 40, 40, 40)
    drive(v, 2, 40, 40, 40, 35, 120.)
    assert v.ahead is not None
    drive(v, 2, 40, 40, 40, 35, 84., closing=False)  # 120 m and growing: not on the car's way
    assert v.ahead is None and v.ahead_dropped == (35, 40)
    frames, _ = drive(v, 4, 40, 40, 40, 35, 100.)  # the same pair again, the distance shrinking: not taken again
    first_40 = next(i for i, f in enumerate(frames) if f[0] == 40)  # back to 40 after the raise hold, and stays there
    assert v.ahead is None and all(f[0] == 40 for f in frames[first_40:])

  def test_passing_the_start_before_osm_switches(self):
    # route 84 Chino Valley Fwy: OSM's distance bottoms out at 3 m, grows again and OSM's own limit switches 73 m after its start
    v = SpeedLimitVote(DT_MDL)
    frames, _ = self.approach(v, car=65, osm=65, ahead=55, speed=60, dist=400.)
    assert frames[-1][0] == 55
    frames, _ = drive(v, 70. / ms(60), 65, 65, 60, 55, 3., closing=False)  # past the start, OSM still 65 for 70 m
    frames2, _ = drive(v, 200. / ms(60), 65, 55, 60, 65, 2300.)  # OSM 55 now, 65 again 2.3 km on; the nav still 65
    assert all(f[0] == 55 for f in frames + frames2)

  def test_short_lower_stretch_is_held_until_osm_leaves_it(self):
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 40, 40, 40, 30)
    drive(v, 3, 40, 40, 30, 35, 40.)
    frames, _ = drive(v, 3, 40, 35, 30, 40, 30.)  # OSM 35, 40 again in 30 m, the car still 40: kept for the stretch
    assert all(f[0] == 35 for f in frames) and v.ahead is not None
    frames, _ = drive(v, 3, 40, 40, 30)  # OSM's stretch is over
    assert v.ahead is None and frames[-1][0] == 40

  def test_lower_stretch_then_higher_while_the_nav_lags(self):
    # 45 road, a 40 zone from 1000 to 1250 m, the nav 150 m late at both ends: one drop, one raise, never back up inside the zone
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 45, 45, 45, 45)
    out = drive(v, 1005. / ms(45), 45, 45, 45, 40, 1000.)[0]  # to the start
    out += drive(v, 150. / ms(45), 45, 40, 45, 45, 250.)[0]  # OSM 40, the nav still 45
    out += drive(v, 100. / ms(45), 40, 40, 45, 45, 100.)[0]  # the nav 40 too
    out += drive(v, 150. / ms(45), 40, 45, 45)[0]  # OSM 45 at the zone's end, the nav still 40
    out += drive(v, 3, 45, 45, 45)[0]
    limits = [f[0] for f in out]
    changes = [b for a, b in zip(limits, limits[1:], strict=False) if a != b]
    assert changes == [40, 45], changes

  def test_frozen_gps_distance_is_let_go(self):
    # no fix: mapd repeats the distance from the last position, the car keeps going; let go after the start plus LIMIT_AHEAD_CONFIRM of travel
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 55, 55, 55, 79)
    frames = [v.update(ms(55), ms(55), ms(55), False, ms(79), False, WEEKDAY_SCHOOL, ms(50), 300.) for _ in range(round(30. / DT_MDL))]
    limits = [round(r.limit * CV.MS_TO_MPH) for r in frames]
    taken_for = sum(lim == 50 for lim in limits) * DT_MDL * ms(79)
    assert limits[-1] == 55 and v.ahead is None and 300. + LIMIT_AHEAD_CONFIRM - 50. < taken_for < 300. + LIMIT_AHEAD_CONFIRM + 50.

  def test_limit_in_force_until_the_start(self):
    v = SpeedLimitVote(DT_MDL)
    run(v, 1, 55, 55, 55, 79)
    r = None
    for _ in range(round(2. / DT_MDL)):
      r = v.update(ms(55), ms(55), ms(55), False, ms(79), False, WEEKDAY_SCHOOL, ms(50), 600.)
    assert r is not None and round(r.limit * CV.MS_TO_MPH) == 50 and r.distance == 600.
    assert round(r.in_force * CV.MS_TO_MPH) == 55  # the over-speed alert's limit until the start
    r = v.update(ms(55), ms(55), ms(50), False, ms(79), False, WEEKDAY_SCHOOL)
    assert r.distance == 0. and round(r.in_force * CV.MS_TO_MPH) == 50

  def test_profile_zero_turns_it_off(self):
    p = dataclasses.replace(DEFAULT_PROFILE.speed_limit, limit_ahead_decel_mps2=0.)
    frames, _ = self.approach(SpeedLimitVote(DT_MDL, p))
    assert all(f[0] == 55 for f in frames)
