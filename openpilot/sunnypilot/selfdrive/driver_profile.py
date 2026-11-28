"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Driver profile (GV70 fork): the driver's own values for fork features, stored as one JSON param "DriverProfile". The upstream-able knobs stay
individual Params (PARAM_SCHEMA lists their ranges for the notes tool tools/device/profile.py). Every default is the fork's current behaviour, so
a missing or empty DriverProfile drives exactly as the constants did.

- Read once per drive: card, selfdrived, plannerd and the UI's onroad start call load(); nothing re-reads it while onroad.
- The device loader never raises and works per key: wrong type -> the default, out of range -> clamped, unknown key -> ignored, an unknown
  schema_version -> all defaults. Each of these logs one warning and the offroad alert Offroad_DriverProfileInvalid lists them.
- parse(strict=True) is the notes tool's check: the same rules, but every problem is an error and nothing is clamped.
- Nothing here reaches the panda or opendbc: no key sets, clears or depends on a safetyParam bit. Hard limits stay constants next to their
  feature (LEAD_URGENT_DECEL and lead_needs_more(), the ACCEL_MIN clip, Genesis mode's Park / standstill / not-engaged conditions, the EPS
  2.2 deg/frame cap); the ranges below sit inside them. Longitudinal keys can only ease braking inside the gas-resume window (where
  lead_needs_more() still forces full braking) or make braking firmer (low_speed_decel_gain >= 1.0).

This module imports only the standard library: the notes tool loads it straight from the fork's file.
"""
import dataclasses
import datetime
import math
from dataclasses import dataclass, field
from typing import Any
from zoneinfo import ZoneInfo

PARAM_KEY = "DriverProfile"
OFFROAD_ALERT = "Offroad_DriverProfileInvalid"
SCHEMA_VERSION = 1


def _num(default: float, lo: float, hi: float):
  return field(default=default, metadata={"range": (lo, hi)})


def _num_or_off(default: float, lo: float, hi: float):
  """0 (off) or lo..hi: a value between 0 and lo is raised to lo."""
  return field(default=default, metadata={"range": (0., hi), "min_on": lo})


def _choice(default: str, *choices: str):
  return field(default=default, metadata={"choices": choices})


@dataclass(frozen=True)
class WheelButton:
  """The programmable steering-wheel button (GV70 only: SWRC_01 Audio_CustomSw3Sta)."""
  long_press: str = _choice("genesis_mode", "none", "genesis_mode")
  hold_s: float = _num(1.0, 0.5, 3.0)


@dataclass(frozen=True)
class MainButton:
  """Main-button engage cars: what a long MAIN from cruise off does (plain = engages like a short one)."""
  long_press: str = _choice("speed_limit_follow", "plain", "speed_limit_follow")


@dataclass(frozen=True)
class RedLight:
  """Red-light cue thresholds (active with the RedLightAlert param)."""
  min_speed_mph: float = _num(10.0, 5.0, 30.0)
  not_slowing_accel_mps2: float = _num(-0.5, -1.5, 0.0)
  stop_dist_max_m: float = _num(80.0, 30.0, 150.0)
  lead_stop_margin_m: float = _num(10.0, 0.0, 30.0)
  hold_s: float = _num(0.3, 0.1, 1.0)
  rearm_clear_s: float = _num(5.0, 1.0, 15.0)
  brake_recent_s: float = _num(3.0, 0.0, 10.0)
  gas_lift_s: float = _num(6.0, 0.0, 10.0)
  late_decel_mps2: float = _num(2.0, 1.0, 4.0)
  reaction_s: float = _num(1.3, 0.5, 2.5)


@dataclass(frozen=True)
class StopCue:
  """Camera stop-sign cue and the BRAKE! cue (also need the RedLightAlert param)."""
  enabled: bool = True
  stop_sign_min_speed_mps: float = _num(4.5, 2.0, 13.5)
  brake_now_min_speed_mph: float = _num(25.0, 15.0, 50.0)
  brake_now_accel_mps2: float = _num(-2.5, -4.0, -1.5)


@dataclass(frozen=True)
class GreenLight:
  """Green-light / lead-depart chime (GreenLightAlert, LeadDepartAlert params)."""
  go_wait_s: float = _num(1.0, 0.0, 3.0)
  brake_release_lookback_s: float = _num(1.0, 0.0, 3.0)


@dataclass(frozen=True)
class Cues:
  bookmark: bool = True
  in_dashcam_mode: bool = True


@dataclass(frozen=True)
class Alerts:
  """Hyundai only; other brands keep upstream's alerts."""
  quiet_silent_steer_fault: bool = True
  quiet_brake_disengage: bool = True


@dataclass(frozen=True)
class SpeedLimit:
  """Speed-limit vote (school zones) and follow. The _mph thresholds apply with an imperial display; km/h keeps the code's values."""
  time_zone: str = "America/Los_Angeles"  # IANA name; "" turns the school-zone time rule off (a school limit never applies)
  school_days: tuple[int, ...] = field(default=(0, 1, 2, 3, 4), metadata={"range": (0, 6)})  # 0 = Monday
  school_hours: tuple[str, str] = ("07:00", "17:00")  # local "HH:MM", start < end
  school_zone_max_mph: int = _num(25, 15, 30)
  school_street_min_mph: int = _num(35, 25, 45)
  freeway_min_mph: int = _num(55, 40, 75)
  # the vote takes OSM's lower limit ahead so early that this decel gets there; 0 = off (0.2 m/s^2 at least: 0.05 would start ~12 km out at 79 mph)
  limit_ahead_decel_mps2: float = _num_or_off(0.5, 0.2, 1.5)


@dataclass(frozen=True)
class OverSpeed:
  """Over-speed chime timing (the margin is the SpeedLimitOverSpeedAlert param)."""
  hold_s: float = _num(2.0, 0.5, 10.0)
  rearm_clear_s: float = _num(5.0, 1.0, 30.0)


@dataclass(frozen=True)
class GasResume:
  """After a gas override above the set speed (openpilot longitudinal)."""
  enabled: bool = True
  hold_s: float = _num(0.5, 0.0, 2.0)
  decel_time_s: float = _num(2.0, 0.0, 5.0)
  max_decel_mps2: float = _num(-0.5, -2.0, 0.0)
  lead_ease_s: float = _num(2.5, 0.0, 3.0)
  lead_max_decel_mps2: float = _num(-0.8, -2.0, -0.3)


@dataclass(frozen=True)
class SoftStop:
  """Low-speed decel feed-forward (part B), GV70 with openpilot longitudinal."""
  low_speed_decel_gain: float = _num(1.15, 1.0, 1.3)
  low_speed_decel_speed_mps: float = _num(5.0, 2.0, 8.0)


@dataclass(frozen=True)
class DriverProfile:
  wheel_button: WheelButton = WheelButton()
  main_button: MainButton = MainButton()
  red_light: RedLight = RedLight()
  stop_cue: StopCue = StopCue()
  green_light: GreenLight = GreenLight()
  cues: Cues = Cues()
  alerts: Alerts = Alerts()
  speed_limit: SpeedLimit = SpeedLimit()
  over_speed: OverSpeed = OverSpeed()
  gas_resume: GasResume = GasResume()
  soft_stop: SoftStop = SoftStop()


DEFAULT_PROFILE = DriverProfile()
SECTIONS: dict[str, Any] = {f.name: f.type for f in dataclasses.fields(DriverProfile)}  # section name -> its dataclass


@dataclass(frozen=True)
class ParamSpec:
  """An individual Param the notes tool writes ([param] in the TOML). Ranges equal settings_ui.json where the key has a widget."""
  kind: type
  lo: int = 0
  hi: int = 0
  step: int = 1


PARAM_SCHEMA: dict[str, ParamSpec] = {
  "HyundaiSteerFeel": ParamSpec(int, 0, 3),
  "HyundaiCurveHold": ParamSpec(int, 0, 1),
  "HyundaiTakeoverTorque": ParamSpec(int, 0, 500, 10),
  "HyundaiTakeoverReleaseTorque": ParamSpec(int, 0, 500, 10),
  "HyundaiTakeoverHold": ParamSpec(int, 0, 2000, 50),
  "HyundaiResumeAngleRate": ParamSpec(int, 1, 22),  # 0.1 deg/frame; the LFA_ALT cap is 2.2 deg/frame
  "RedLightAlert": ParamSpec(bool),
  "GreenLightAlert": ParamSpec(bool),
  "LeadDepartAlert": ParamSpec(bool),
  "SpeedLimitOverSpeedAlert": ParamSpec(int, 0, 15, 5),
  "SpeedLimitFollowOffsetStreet": ParamSpec(int, -10, 25),
  "SpeedLimitFollowOffsetFreeway": ParamSpec(int, -10, 25),
}


def parse_hhmm(v: Any) -> datetime.time | None:
  """Exactly "HH:MM" with ASCII digits, 00:00-23:59; anything else (" 7:00", "+7:00", "07: 0", full-width digits) is None."""
  if not isinstance(v, str) or len(v) != 5 or v[2] != ":":
    return None
  digits = v[:2] + v[3:]
  if not (digits.isascii() and digits.isdigit()):
    return None
  hour, minute = int(v[:2]), int(v[3:])
  return datetime.time(hour, minute) if hour < 24 and minute < 60 else None


def school_hours_times(v: Any) -> tuple[datetime.time, datetime.time] | None:
  """["HH:MM", "HH:MM"] with start < end -> the two times, else None. The vote's school rule builds its hours with this too."""
  if not isinstance(v, (list, tuple)) or len(v) != 2:
    return None
  start, end = parse_hhmm(v[0]), parse_hhmm(v[1])
  return (start, end) if start is not None and end is not None and start < end else None


def load_zone(name: Any) -> ZoneInfo | None:
  """The ZoneInfo for an IANA name, or None if it is not a loadable zone (wrong type, typo, missing from tzdata). Never raises."""
  if not isinstance(name, str) or name == "":
    return None
  try:
    return ZoneInfo(name)
  except Exception:  # ZoneInfoNotFoundError (a KeyError), ValueError for malformed keys, OSError, TypeError
    return None


def _show(v: Any) -> str:
  """repr for a problem message, short and safe (repr of an int over 4300 digits raises)."""
  try:
    s = repr(v)
  except ValueError:
    return f"<{type(v).__name__} too large to show>"
  return s if len(s) <= 60 else s[:57] + "..."


def _value(section: str, f: dataclasses.Field, raw: Any, strict: bool, problems: list[str]) -> Any:
  """One key: the value to use, or the default after recording a problem."""
  key = f"{section}.{f.name}"
  default = f.default
  rng: tuple[float, float] = f.metadata.get("range", (0, 0))
  choices = f.metadata.get("choices")

  if f.name == "school_days":
    lo, hi = rng
    if (isinstance(raw, (list, tuple)) and all(isinstance(d, int) and not isinstance(d, bool) and lo <= d <= hi for d in raw)
        and len(set(raw)) == len(raw)):
      return tuple(sorted(raw))
    problems.append(f"{key}: {_show(raw)} is not a list of distinct days {lo}-{hi}")
    return default
  if f.name == "school_hours":
    times = school_hours_times(raw)
    if times is not None:
      return (times[0].strftime("%H:%M"), times[1].strftime("%H:%M"))
    problems.append(f"{key}: {_show(raw)} is not [\"HH:MM\", \"HH:MM\"] with start < end")
    return default
  if f.name == "time_zone":
    if raw == "" or load_zone(raw) is not None:
      return raw
    problems.append(f"{key}: {_show(raw)} is not an IANA time zone this build knows, {default!r} used")
    return default
  if choices is not None:
    if raw in choices:
      return raw
    problems.append(f"{key}: {_show(raw)} is not one of {', '.join(choices)}")
    return default
  if isinstance(default, bool):
    if isinstance(raw, bool):
      return raw
    problems.append(f"{key}: {_show(raw)} is not true/false")
    return default

  if isinstance(default, int):
    ok = isinstance(raw, int) and not isinstance(raw, bool)
    expected = "an integer"
  else:
    try:
      ok = isinstance(raw, (int, float)) and not isinstance(raw, bool) and math.isfinite(raw)
    except OverflowError:  # an int too large for a float
      ok = False
    expected = "a number"
  if not ok:
    problems.append(f"{key}: {_show(raw)} is not {expected}")
    return default
  value: float = int(raw) if isinstance(default, int) else float(raw)
  lo, hi = rng
  min_on = f.metadata.get("min_on")
  in_range = lo <= value <= hi
  if in_range and not (min_on is not None and 0. < value < min_on):
    return value
  if strict:
    problems.append(f"{key}: {_show(raw)} is outside {lo}..{hi}" if not in_range else f"{key}: {_show(raw)} is between 0 (off) and {min_on}")
    return default
  clamped = type(value)(min(max(value, lo), hi))
  if min_on is not None and 0. < clamped < min_on:
    clamped = type(value)(min_on)
  problems.append(f"{key}: {_show(raw)} clamped to {clamped}")
  return clamped


def parse(raw: Any, strict: bool = False) -> tuple[DriverProfile, list[str]]:
  """The DriverProfile dict (schema_version + sections) -> (profile, problems). Never raises. strict: out of range is an error, not clamped."""
  problems: list[str] = []
  if raw is None:
    return DEFAULT_PROFILE, problems
  if not isinstance(raw, dict):
    return DEFAULT_PROFILE, [f"{PARAM_KEY} is not a JSON object: all defaults"]

  version = raw.get("schema_version", SCHEMA_VERSION)
  if version != SCHEMA_VERSION or isinstance(version, bool):
    return DEFAULT_PROFILE, [f"schema_version {_show(version)} is not {SCHEMA_VERSION}: all defaults"]

  sections = {}
  for name, raw_section in raw.items():
    if name == "schema_version":
      continue
    if name not in SECTIONS:
      problems.append(f"{name}: unknown section, ignored")
    elif not isinstance(raw_section, dict):
      problems.append(f"{name}: {_show(raw_section)} is not a table, defaults")
  for name, cls in SECTIONS.items():
    raw_section = raw.get(name)
    if not isinstance(raw_section, dict):
      sections[name] = getattr(DEFAULT_PROFILE, name)
      continue
    fields = {f.name: f for f in dataclasses.fields(cls)}
    for key in raw_section:
      if key not in fields:
        problems.append(f"{name}.{key}: unknown key, ignored")
    values = {key: _value(name, f, raw_section[key], strict, problems) for key, f in fields.items() if key in raw_section}
    sections[name] = cls(**values)
  return DriverProfile(**sections), problems


def to_dict(profile: DriverProfile) -> dict[str, Any]:
  """The JSON object stored in the param (tuples as lists), schema_version first."""
  out: dict[str, Any] = {"schema_version": SCHEMA_VERSION}
  for name in SECTIONS:
    out[name] = {k: list(v) if isinstance(v, tuple) else v for k, v in dataclasses.asdict(getattr(profile, name)).items()}
  return out


def check_params(values: dict[str, Any]) -> list[str]:
  """The notes tool's check of the [param] keys: unknown keys, types, ranges, steps, release <= takeover torque."""
  problems = []
  for key, v in values.items():
    spec = PARAM_SCHEMA.get(key)
    if spec is None:
      problems.append(f"param.{key}: not a key this tool writes")
      continue
    if spec.kind is bool:
      if not isinstance(v, bool):
        problems.append(f"param.{key}: {_show(v)} is not true/false")
      continue
    if not isinstance(v, int) or isinstance(v, bool):
      problems.append(f"param.{key}: {_show(v)} is not an integer")
    elif not spec.lo <= v <= spec.hi:
      problems.append(f"param.{key}: {_show(v)} is outside {spec.lo}..{spec.hi}")
    elif (v - spec.lo) % spec.step:
      problems.append(f"param.{key}: {v} is not a multiple of {spec.step} from {spec.lo}")
  takeover, release = values.get("HyundaiTakeoverTorque"), values.get("HyundaiTakeoverReleaseTorque")
  if isinstance(takeover, int) and isinstance(release, int) and takeover > 0 and release > takeover:
    problems.append(f"param.HyundaiTakeoverReleaseTorque: {_show(release)} is above HyundaiTakeoverTorque {_show(takeover)}")
  return problems


def load(params=None, report: bool = True) -> DriverProfile:
  """Device: read DriverProfile once (missing -> defaults). report: log each problem and set or clear the offroad alert (the UI only reads).
  Never raises."""
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog

  params = params or Params()
  try:
    raw = params.get(PARAM_KEY)
    if raw is None:  # missing, or a value json.loads rejected (Params.get returns None for both)
      try:
        with open(params.get_param_path(PARAM_KEY), "rb") as f:
          raw = "invalid JSON" if f.read().strip() else None
      except OSError:
        pass
    profile, problems = parse(raw)
  except Exception as e:  # a broken param store must not stop the process
    profile, problems = DEFAULT_PROFILE, [f"{PARAM_KEY} unreadable ({e}): all defaults"]

  if report:
    for p in problems:
      cloudlog.warning(f"driver profile: {p}")
    try:
      if problems:
        from openpilot.selfdrive.selfdrived.alertmanager import OFFROAD_ALERTS
        alert = dict(OFFROAD_ALERTS[OFFROAD_ALERT], extra=f"{len(problems)}: " + "; ".join(problems)[:400])
        params.put(OFFROAD_ALERT, alert, block=True)
      else:
        params.remove(OFFROAD_ALERT)
    except Exception as e:
      cloudlog.warning(f"driver profile: offroad alert failed: {e}")
  return profile
