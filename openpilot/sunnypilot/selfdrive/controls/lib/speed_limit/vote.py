"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Speed-limit policy "vote" (Hyundai CCNC, GV70): the car's dash value (the nav map limit, replaced by the camera when it reads a sign), the head
unit's nav limit, and OSM vote, with guards against the wrong-low values seen on 28 routes (research/speed-limit-modes-2026-11-24.md in the
driver's notes): a camera reading counts only when OSM confirms it (truck "55" and ramp advisories on freeways), the nav snapping to a ramp at
speed loses to OSM, a big drop at speed must persist, and a school zone's conditional limit applies only in weekday school hours.

Lower limit ahead (research/speed-limit-ahead-2026-11-27.md): when the car and OSM agree on the current limit and OSM has a lower one ahead, the
vote takes the lower one so early that openpilot, slowing at the profile's limit_ahead_decel_mps2, is at it where OSM's own limit changes. The
car's nav changed up to 228 m after OSM's change point on the port routes, so the lower limit is kept a while past it while the car still shows
the old one. Never for a school-zone-sized limit, a drop the vote would make wait (big_drop), or a value that is not a posted one.
"""
import dataclasses
import datetime
from dataclasses import dataclass
from zoneinfo import ZoneInfo

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.driver_profile import DEFAULT_PROFILE, SpeedLimit, load_zone, school_hours_times


@dataclass(frozen=True)
class SchoolRule:
  """When a school zone's conditional limit is in force: weekdays and local hours in the car's home time zone (tz None: never)."""
  tz: ZoneInfo | None
  days: frozenset[int]
  start: datetime.time
  end: datetime.time

  @classmethod
  def from_profile(cls, cfg: SpeedLimit) -> "SchoolRule":
    """DriverProfile.speed_limit with the loader's own parsers: time_zone "" turns the rule off; hours or a zone they reject (a profile not
    built by the loader) fall back to the defaults, so this never raises."""
    default = DEFAULT_PROFILE.speed_limit
    times = school_hours_times(cfg.school_hours) or school_hours_times(default.school_hours)
    assert times is not None
    tz = None if cfg.time_zone == "" else (load_zone(cfg.time_zone) or load_zone(default.time_zone))
    return cls(tz, frozenset(cfg.school_days), times[0], times[1])


DEFAULT_SCHOOL_RULE = SchoolRule.from_profile(DEFAULT_PROFILE.speed_limit)
LOCAL_TZ = DEFAULT_SCHOOL_RULE.tz  # the default home time zone (America/Los_Angeles): school hours are local
SCHOOL_SIGN_MEMORY = 30.  # s a "School Crossing" sign keeps a school-zone candidate alive

RAISE_HOLD = 2.  # s an increase must persist
DROP_HOLD = 1.  # s a decrease must persist
BIG_DROP_HOLD = 10.  # s a big decrease at speed must persist unless both maps agree

LIMIT_AHEAD_LAG = 2.  # s at speed before openpilot slows: the vote's DROP_HOLD and the cruise easing into its decel
LIMIT_STEP = 5  # posted limits are multiples of 5 (mph and km/h); OSM's "37" (a 60 km/h tag read as mph, route 91) is not one
LIMIT_AHEAD_NEAR = 25.  # m: the start OSM reported is reached; before it the distance may grow back this much (more: a cross street, route 92)
LIMIT_AHEAD_OSM_LATE = 150.  # m past the reported start OSM's own limit may still be the old one (it switched 15-73 m past it on the port routes)
LIMIT_AHEAD_CONFIRM = 400.  # m past the start the car's own limit may still be the old one (route 84: OSM 73 m late, the nav 228 m later); counted
                            # from the car's own travel since the limit was taken, so a frozen GPS position (mapd repeating its distance) ends it too


@dataclass(frozen=True)
class Units:
  """Thresholds in the displayed unit (the limits are whole mph or km/h)."""
  to_ms: float
  disagree: int  # the car this far below OSM at speed = a ramp / frontage-road snap
  fast: int  # "at speed"
  big_drop: int
  school_max: int  # a school-zone limit is at most this
  school_base_min: int  # OSM at least this means the street's own limit is higher than the school zone's


MPH = Units(CV.MPH_TO_MS, 10, 45, 15, DEFAULT_PROFILE.speed_limit.school_zone_max_mph, DEFAULT_PROFILE.speed_limit.school_street_min_mph)
KPH = Units(CV.KPH_TO_MS, 15, 70, 25, 30, 50)  # the profile's _mph thresholds do not apply to a km/h display


@dataclass
class VoteResult:
  limit: float  # m/s, 0 = none
  from_map: bool
  school: bool
  distance: float = 0.  # m to the start of the limit when it is a lower one ahead taken early (not in force yet), else 0
  in_force: float = 0.  # m/s, the limit in force now: the one before that lower limit until its start, else the limit


def _units(v: float, u: Units) -> int | None:
  return int(round(v / u.to_ms)) if v > 0. else None


def in_school_hours(now: datetime.datetime, rule: SchoolRule = DEFAULT_SCHOOL_RULE) -> bool:
  if rule.tz is None:
    return False
  local = now.astimezone(rule.tz)
  return local.weekday() in rule.days and rule.start <= local.time() < rule.end


def vote(dash: int | None, nav: int | None, osm: int | None, school_sign: bool, v_ego: int, u: Units) -> tuple[int | None, bool, bool]:
  """One frame's (limit, from_map, school_candidate) in display units, before persistence. school_candidate: the car's value is a school zone's
  conditional limit (the caller decides by the time of day whether it applies); the limit returned for it is the car's."""
  car = nav if nav is not None else dash
  if car is None:
    return osm, osm is not None, False
  camera_confirmed = dash is not None and nav is not None and dash != nav and osm == dash
  # school zone: the car <= 25 with a School Crossing sign, or below a much higher OSM at city speed (at freeway speed that is a ramp snap)
  low = car <= u.school_max
  if low and (school_sign or (not camera_confirmed and osm is not None and osm >= u.school_base_min and v_ego <= u.fast)):
    return car, False, True
  if camera_confirmed:
    return dash, False, False  # the camera's sign, confirmed by OSM
  if osm is None or car == osm:
    return car, False, False
  if car <= osm - u.disagree and v_ego > u.fast:
    return osm, True, False  # the nav snapped to a ramp or a frontage road
  return car, False, False


class SpeedLimitVote:
  """vote() per frame plus persistence and the school-zone time rule. update() returns the limit to use."""
  def __init__(self, dt: float, cfg: SpeedLimit = DEFAULT_PROFILE.speed_limit):
    self.dt = dt
    self.school_rule = SchoolRule.from_profile(cfg)
    self.mph = dataclasses.replace(MPH, school_max=cfg.school_zone_max_mph, school_base_min=cfg.school_street_min_mph)
    self.applied: int | None = None
    self.applied_from_map = False
    self.applied_school = False
    self.base: int | None = None  # the last applied non-school limit, kept through a school zone out of school hours
    self.base_from_map = False
    self.pending: tuple[int, bool] | None = None  # (limit, school) waiting to persist
    self.pending_time = 0.
    self.school_sign_age = SCHOOL_SIGN_MEMORY + 1.
    self.ahead_decel = cfg.limit_ahead_decel_mps2
    self.ahead: tuple[int, int] | None = None  # (OSM's lower limit ahead, the limit before it) being slowed for
    self.ahead_distance = 0.  # m to its start, 0 once reached
    self.ahead_closest = 0.  # m, the shortest distance to its start so far
    self.ahead_past = 0.  # m since its start was reached (by mapd's distance)
    self.ahead_start = 0.  # m to its start when it was taken
    self.ahead_travel = 0.  # m the car has gone since it was taken (its own speed, whatever mapd reports)
    self.ahead_osm_low = False  # OSM's own limit has been the lower one
    self.ahead_dropped: tuple[int, int] | None = None  # the (ahead, current) pair last let go: not taken again while OSM reports the same

  def _start_distance(self, v_ego: float, limit: float) -> float:
    """How far before a lower limit (m/s) openpilot has to start, at v_ego, to be at it there."""
    return v_ego * LIMIT_AHEAD_LAG + max(0., v_ego ** 2 - limit ** 2) / (2. * self.ahead_decel)

  def _limit_ahead(self, car: int | None, o: int | None, a: int | None, ahead_dist: float, v_ego: float, u: Units) -> int | None:
    """OSM's lower limit ahead to use now (display units), or None."""
    if self.ahead is not None:
      low, before = self.ahead
      grew = False
      if o == before and a == low:
        grew = ahead_dist > self.ahead_closest + LIMIT_AHEAD_NEAR
        self.ahead_closest = min(self.ahead_closest, ahead_dist)
      self.ahead_osm_low = self.ahead_osm_low or o == low
      reached = self.ahead_osm_low or self.ahead_closest <= LIMIT_AHEAD_NEAR
      if reached:
        self.ahead_past += v_ego * self.dt
      self.ahead_travel += v_ego * self.dt
      self.ahead_distance = 0. if reached else ahead_dist
      if car not in (before, low) or o not in (before, low):
        keep = False  # the car's limit or OSM's went elsewhere (or OSM is stale / missing)
      elif self.ahead_travel > self.ahead_start + LIMIT_AHEAD_CONFIRM:
        keep = False  # the car never confirmed it, or mapd's distance is frozen (no GPS fix)
      elif o == before:
        # OSM's lower stretch is over; before the start: OSM still has it ahead (not after a turn onto another road) and the car is heading for
        # it (not a cross street); past it: OSM's own limit has a little while to follow
        keep = not self.ahead_osm_low and (self.ahead_past <= LIMIT_AHEAD_OSM_LATE if reached else (a == low and not grew))
      else:
        keep = car != low  # OSM's own limit is the lower one: until the car has it too (the plain vote takes it then)
      if not keep:
        self.ahead, self.ahead_dropped = None, self.ahead
    if self.ahead_dropped != (a, o):
      self.ahead_dropped = None
    if (self.ahead is None and self.ahead_dropped is None and self.ahead_decel > 0. and a is not None and o is not None and car == o
        and u.school_max < a < o and o - a <= u.big_drop and a % LIMIT_STEP == 0 and 0. < ahead_dist <= self._start_distance(v_ego, a * u.to_ms)):
      self.ahead, self.ahead_distance, self.ahead_closest, self.ahead_past = (a, o), ahead_dist, ahead_dist, 0.
      self.ahead_start, self.ahead_travel, self.ahead_osm_low = ahead_dist, 0., False
    return self.ahead[0] if self.ahead is not None else None

  def update(self, dash: float, nav: float, osm: float, school_sign: bool, v_ego: float, is_metric: bool,
             now: datetime.datetime | None = None, ahead: float = 0., ahead_dist: float = 0.) -> VoteResult:
    """ahead, ahead_dist: OSM's limit ahead (m/s, 0 = none) and the distance to its start now (m); only with a fresh, valid OSM limit."""
    u = KPH if is_metric else self.mph
    self.school_sign_age = 0. if school_sign else self.school_sign_age + self.dt
    d, n, o, v = _units(dash, u), _units(nav, u), _units(osm, u), int(round(v_ego / u.to_ms))
    cand, from_map, school = vote(d, n, o, self.school_sign_age <= SCHOOL_SIGN_MEMORY, v, u)
    low = self._limit_ahead(n if n is not None else d, o, _units(ahead, u), ahead_dist, v_ego, u)

    if school and not in_school_hours(now or datetime.datetime.now(datetime.UTC), self.school_rule):
      # not in force: the street's own limit (OSM's, else the one before the zone)
      cand, from_map, school = (o, True, False) if o is not None else (self.base, self.base_from_map, False)
      if cand is None and self.applied_school:  # a school zone applied in school hours, nothing to go back to: no limit
        self.applied, self.applied_school = None, False

    if low is not None and not school and cand is not None and cand > low:
      cand, from_map = low, True

    if cand is None or (cand == self.applied and school == self.applied_school):
      self.pending, self.pending_time = None, 0.
    else:
      if (cand, school) != self.pending:
        self.pending, self.pending_time = (cand, school), 0.
      self.pending_time += self.dt
      if self.applied is None:
        hold = 0.
      elif cand > self.applied:
        hold = RAISE_HOLD
      elif self.applied - cand > u.big_drop and v > u.fast and not (o == cand and (n if n is not None else d) == cand):
        hold = BIG_DROP_HOLD
      else:
        hold = DROP_HOLD
      if self.pending_time >= hold:
        self.applied, self.applied_from_map, self.applied_school = cand, from_map, school
        if not school:
          self.base, self.base_from_map = cand, from_map
        self.pending, self.pending_time = None, 0.

    limit = self.applied * u.to_ms if self.applied else 0.
    distance = self.ahead_distance if self.ahead is not None and self.applied == self.ahead[0] and not self.applied_school else 0.
    in_force = self.ahead[1] * u.to_ms if distance > 0. and self.ahead is not None else limit
    return VoteResult(limit, self.applied_from_map, self.applied_school, distance, in_force)
