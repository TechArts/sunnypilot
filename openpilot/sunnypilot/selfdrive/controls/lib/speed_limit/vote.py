"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Speed-limit policy "vote" (Hyundai CCNC, GV70): the car's dash value (the nav map limit, replaced by the camera when it reads a sign), the head
unit's nav limit, and OSM vote, with guards against the wrong-low values seen on 28 routes (research/speed-limit-modes-2026-11-24.md in the
driver's notes): a camera reading counts only when OSM confirms it (truck "55" and ramp advisories on freeways), the nav snapping to a ramp at
speed loses to OSM, a big drop at speed must persist, and a school zone's conditional limit applies only in weekday school hours.
"""
import dataclasses
import datetime
from dataclasses import dataclass
from zoneinfo import ZoneInfo

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.driver_profile import DEFAULT_PROFILE, SpeedLimit


@dataclass(frozen=True)
class SchoolRule:
  """When a school zone's conditional limit is in force: weekdays and local hours in the car's home time zone (tz None: never)."""
  tz: ZoneInfo | None
  days: frozenset[int]
  start: datetime.time
  end: datetime.time

  @classmethod
  def from_profile(cls, cfg: SpeedLimit) -> "SchoolRule":
    """DriverProfile.speed_limit: time_zone "" turns the rule off (the loader already replaced an unknown zone with "")."""
    start, end = (datetime.time.fromisoformat(h) for h in cfg.school_hours)
    return cls(ZoneInfo(cfg.time_zone) if cfg.time_zone else None, frozenset(cfg.school_days), start, end)


DEFAULT_SCHOOL_RULE = SchoolRule.from_profile(DEFAULT_PROFILE.speed_limit)
LOCAL_TZ = DEFAULT_SCHOOL_RULE.tz  # the default home time zone (America/Los_Angeles): school hours are local
SCHOOL_SIGN_MEMORY = 30.  # s a "School Crossing" sign keeps a school-zone candidate alive

RAISE_HOLD = 2.  # s an increase must persist
DROP_HOLD = 1.  # s a decrease must persist
BIG_DROP_HOLD = 10.  # s a big decrease at speed must persist unless both maps agree


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

  def update(self, dash: float, nav: float, osm: float, school_sign: bool, v_ego: float, is_metric: bool,
             now: datetime.datetime | None = None) -> VoteResult:
    u = KPH if is_metric else self.mph
    self.school_sign_age = 0. if school_sign else self.school_sign_age + self.dt
    d, n, o, v = _units(dash, u), _units(nav, u), _units(osm, u), int(round(v_ego / u.to_ms))
    cand, from_map, school = vote(d, n, o, self.school_sign_age <= SCHOOL_SIGN_MEMORY, v, u)

    if school and not in_school_hours(now or datetime.datetime.now(datetime.UTC), self.school_rule):
      # not in force: the street's own limit (OSM's, else the one before the zone)
      cand, from_map, school = (o, True, False) if o is not None else (self.base, self.base_from_map, False)
      if cand is None and self.applied_school:  # a school zone applied in school hours, nothing to go back to: no limit
        self.applied, self.applied_school = None, False

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

    return VoteResult(self.applied * u.to_ms if self.applied else 0., self.applied_from_map, self.applied_school)
