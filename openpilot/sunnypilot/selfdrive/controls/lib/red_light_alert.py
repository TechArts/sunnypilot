"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Sequence

from openpilot.cereal import messaging, custom

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.driver_profile import DEFAULT_PROFILE, RedLight
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

# The tunable thresholds are DriverProfile.red_light (driver_profile.py); the names below are their defaults in this module's units.
_D = DEFAULT_PROFILE.red_light
MIN_SPEED = _D.min_speed_mph * CV.MPH_TO_MS  # m/s, no cue below this (driver's request 2026-11-25: same as the stop-sign cue's 4.5 m/s)
MIN_ACCEL = _D.not_slowing_accel_mps2   # m/s^2, driver is already slowing down at or below this
STOP_SPEED_THRESHOLD = 1.0              # m/s, the model's planned speed at its horizon counts as stopped below this
STOP_SPEED_RATIO = 0.25                 # ... and it must also be well below the current speed
STOP_DIST_MAX = _D.stop_dist_max_m      # m, the model's planned stop point (end of its path) must be nearer than this
LEAD_STOP_MARGIN = _D.lead_stop_margin_m  # m, a lead this close to the planned stop point explains the stop, so no cue
HOLD_TIME = _D.hold_s                   # s, condition must hold this long before the cue fires
REARM_CLEAR_TIME = _D.rearm_clear_s     # s, condition must stay clear this long to re-arm after a cue
STOPPED_SPEED = 0.5                     # m/s, the car being below this re-arms the cue
# "Already stopping" (driver 2026-11-25, routes a3-a6: 3 of 5 cues came while the driver was easing in): a brake press in the last
# brake_recent_s with no gas since, the gas released in the last gas_lift_s with the car coasting, or openpilot controlling the speed
# (openpilot longitudinal only: stock SCC does not stop for lights) silence the cue, unless stopping at the model's stop point, after
# reaction_s at the current speed when not slowing, needs more than late_decel_mps2 (then the cue fires as before).
BRAKE_RECENT_TIME = _D.brake_recent_s   # s (a3 9:32.8: brake released between taps 0.5 s before the cue)
GAS_LIFT_TIME = _D.gas_lift_s           # s (a4 5:19.4: gas lifted 4.2 s before, coasting at -0.1 to -0.2 m/s^2)
LATE_DECEL = _D.late_decel_mps2         # m/s^2, v^2 / (2 * stop distance left after the reaction)
REACTION_TIME = _D.reaction_s           # s, counted only while the car is not slowing (a_ego >= 0)


class RedLightStates:
  ARMED = 0
  FIRED = 1


def model_stop_distance(model_x: Sequence[float], model_v: Sequence[float], v_ego: float, lead_status: bool, lead_d_rel: float,
                        cfg: RedLight = _D) -> float | None:
  """The model's path ends (10 s horizon) at near-zero speed close ahead, and no lead vehicle explains that stop: the stop distance, else None."""
  if len(model_x) == 0 or len(model_x) != len(model_v):
    return None

  stop_dist = model_x[-1]
  stops = model_v[-1] < min(STOP_SPEED_THRESHOLD, STOP_SPEED_RATIO * v_ego)
  if not stops or stop_dist > cfg.stop_dist_max_m:
    return None

  return None if lead_status and lead_d_rel < stop_dist + cfg.lead_stop_margin_m else stop_dist


def model_plans_stop(model_x: Sequence[float], model_v: Sequence[float], v_ego: float, lead_status: bool, lead_d_rel: float,
                     cfg: RedLight = _D) -> bool:
  return model_stop_distance(model_x, model_v, v_ego, lead_status, lead_d_rel, cfg) is not None


def red_ahead_not_slowing(v_ego: float, a_ego: float, brake_pressed: bool, stop_dist: float | None, since_brake: float = 1e3,
                          since_gas: float = 1e3, long_active: bool = False, cfg: RedLight = _D) -> bool:
  """No blinker gate (driver's request 2026-11-25): a red light in a turn lane still needs the driver to slow down. since_brake / since_gas: s since
  the last frame with the brake / gas pressed (0 while pressed); long_active: openpilot controls the speed (its planner brakes for the stop)."""
  if stop_dist is None or v_ego <= cfg.min_speed_mph * CV.MPH_TO_MS or a_ego <= cfg.not_slowing_accel_mps2 or brake_pressed:
    return False
  d_left = stop_dist - (v_ego * cfg.reaction_s if a_ego >= 0.0 else 0.0)
  if v_ego ** 2 / (2.0 * max(d_left, 1.0)) > cfg.late_decel_mps2:
    return True
  easing_in = since_brake < min(since_gas, cfg.brake_recent_s) or (0.0 < since_gas < cfg.gas_lift_s and a_ego < 0.0)
  return not (long_active or easing_in)


class RedLightAlertHelper:
  def __init__(self, openpilot_longitudinal: bool = False, cfg: RedLight = _D):
    self.openpilot_longitudinal = openpilot_longitudinal
    self.cfg = cfg
    self._params = Params()
    self.frame = -1
    self.enabled = self._params.get_bool("RedLightAlert")

    self.state = RedLightStates.ARMED
    self.hold_timer = 0
    self.clear_timer = 0
    self.red_light_alert = False
    self.since_brake = 1e3  # s
    self.since_gas = 1e3  # s
    self.stop_ahead = False

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self._params.get_bool("RedLightAlert")

  def step(self, condition: bool, v_ego: float, present: bool | None = None) -> bool:
    """Advance the debounce / one-shot state machine one model frame. Returns True on the frame the cue fires. present: a stop is still
    ahead (default: the condition); after a cue only its absence for rearm_clear_s re-arms, so a gate that silences the condition while
    the same light is ahead cannot re-arm it."""
    if not self.enabled:
      self.state = RedLightStates.ARMED
      self.hold_timer = 0
      self.clear_timer = 0
      return False

    if self.state == RedLightStates.ARMED:
      self.hold_timer = self.hold_timer + 1 if condition else 0
      if self.hold_timer * DT_MDL >= self.cfg.hold_s:
        self.state = RedLightStates.FIRED
        self.hold_timer = 0
        self.clear_timer = 0
        return True
    else:
      self.clear_timer = 0 if (condition if present is None else present) else self.clear_timer + 1
      if v_ego < STOPPED_SPEED or self.clear_timer * DT_MDL >= self.cfg.rearm_clear_s:
        self.state = RedLightStates.ARMED
        self.hold_timer = 0
        self.clear_timer = 0

    return False

  def read_condition(self, sm: messaging.SubMaster) -> bool:
    CS = sm['carState']
    model = sm['modelV2']
    lead = sm['radarState'].leadOne
    self.since_brake = 0.0 if CS.brakePressed else self.since_brake + DT_MDL
    self.since_gas = 0.0 if CS.gasPressed else self.since_gas + DT_MDL

    stop_dist = model_stop_distance(model.position.x, model.velocity.x, CS.vEgo, lead.present, lead.dRel, self.cfg)
    self.stop_ahead = stop_dist is not None
    long_active = self.openpilot_longitudinal and sm['carControl'].longActive
    return red_ahead_not_slowing(CS.vEgo, CS.aEgo, CS.brakePressed, stop_dist, self.since_brake, self.since_gas, long_active, self.cfg)

  def update(self, sm: messaging.SubMaster, events_sp: EventsSP) -> None:
    self._read_params()

    condition = self.read_condition(sm)
    self.red_light_alert = self.step(condition, sm['carState'].vEgo, self.stop_ahead)
    if self.red_light_alert:
      events_sp.add(custom.OnroadEventSP.EventName.redLightAlert)

    self.frame += 1
