"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import messaging, custom

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.red_light_alert import MIN_ACCEL, STOPPED_SPEED, model_plans_stop
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

# Stop sign cue: the Hyundai camera flags a stop sign (carStateSP.cameraStopSign) and the car is not slowing down for it
STOP_SIGN_MIN_SPEED = 4.5               # m/s, no cue below this
STOP_SIGN_HOLD_TIME = 0.5               # s, the car must keep not slowing this long before the cue fires
STOP_SIGN_REARM_CLEAR_TIME = 5.0        # s, the sign flag must stay off this long to re-arm (the flag stays on ~4-9 s per sign)

# Brake now: the model asks for a hard stop that the driver has to start (a late red light)
BRAKE_NOW_MIN_SPEED = 25 * CV.MPH_TO_MS  # m/s, no cue below this
BRAKE_NOW_ACCEL = -2.5                   # m/s^2, the model's requested accel must be below this
BRAKE_NOW_HOLD_TIME = 0.3                # s, condition must hold this long before the cue fires
BRAKE_NOW_REARM_CLEAR_TIME = 5.0         # s, condition must stay clear this long to re-arm after a cue


def stop_sign_not_slowing(long_active: bool, sign: bool, v_ego: float, a_ego: float, brake_pressed: bool) -> bool:
  """The camera flags a stop sign, openpilot drives, and nothing shows the car is slowing down (same semantics as red_ahead_not_slowing)."""
  return long_active and sign and v_ego > STOP_SIGN_MIN_SPEED and a_ego > MIN_ACCEL and not brake_pressed


def brake_now_needed(long_active: bool, v_ego: float, model_accel: float, brake_pressed: bool, model_should_stop: bool, stop_planned: bool) -> bool:
  """The model requests a hard decel at speed for a stop it plans itself (e2e shouldStop, or a path ending at standstill with no lead explaining it).
  Hard braking without that stop source is the MPC following a lead, which is not for this cue."""
  return (long_active and v_ego > BRAKE_NOW_MIN_SPEED and model_accel < BRAKE_NOW_ACCEL and not brake_pressed and
          (model_should_stop or stop_planned))


class OneShotCue:
  """Fires once when the condition has held for hold_time, then stays silent until `present` was off for rearm_time or the car stopped."""

  def __init__(self, hold_time: float, rearm_time: float):
    self.hold_frames = round(hold_time / DT_MDL)
    self.rearm_frames = round(rearm_time / DT_MDL)
    self.fired = False
    self.hold_timer = 0
    self.clear_timer = 0

  def reset(self) -> None:
    self.fired = False
    self.hold_timer = 0
    self.clear_timer = 0

  def step(self, condition: bool, present: bool, v_ego: float) -> bool:
    """Advance one model frame. Returns True on the frame the cue fires."""
    if not self.fired:
      self.hold_timer = self.hold_timer + 1 if condition else 0
      if self.hold_timer >= self.hold_frames:
        self.fired = True
        self.hold_timer = 0
        self.clear_timer = 0
        return True
    else:
      self.clear_timer = 0 if present else self.clear_timer + 1
      if v_ego < STOPPED_SPEED or self.clear_timer >= self.rearm_frames:
        self.reset()

    return False


class StopCueHelper:
  """Stop sign cue and brake-now cue. Alerts only: no acceleration is changed."""

  def __init__(self, openpilot_longitudinal: bool):
    self._params = Params()
    self.frame = -1
    self.enabled = self._params.get_bool("RedLightAlert")
    self.openpilot_longitudinal = openpilot_longitudinal

    self.stop_sign = OneShotCue(STOP_SIGN_HOLD_TIME, STOP_SIGN_REARM_CLEAR_TIME)
    self.brake_now = OneShotCue(BRAKE_NOW_HOLD_TIME, BRAKE_NOW_REARM_CLEAR_TIME)
    self.stop_sign_alert = False
    self.brake_now_alert = False

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self._params.get_bool("RedLightAlert")

  def update(self, sm: messaging.SubMaster, events_sp: EventsSP) -> None:
    self._read_params()
    self.frame += 1

    CS = sm['carState']
    long_active = self.openpilot_longitudinal and sm['carControl'].enabled
    if not self.enabled:
      self.stop_sign.reset()
      self.brake_now.reset()
      self.stop_sign_alert = self.brake_now_alert = False
      return

    sign = sm['carStateSP'].cameraStopSign
    slowing_cond = stop_sign_not_slowing(long_active, sign, CS.vEgo, CS.aEgo, CS.brakePressed)
    self.stop_sign_alert = self.stop_sign.step(slowing_cond, sign, CS.vEgo)

    model = sm['modelV2']
    lead = sm['radarState'].leadOne
    stop_planned = model_plans_stop(model.position.x, model.velocity.x, CS.vEgo, lead.present, lead.dRel)
    brake_cond = brake_now_needed(long_active, CS.vEgo, model.action.desiredAcceleration, CS.brakePressed, model.action.shouldStop, stop_planned)
    self.brake_now_alert = self.brake_now.step(brake_cond, brake_cond, CS.vEgo)

    if self.stop_sign_alert:
      events_sp.add(custom.OnroadEventSP.EventName.stopSignAlert)
    if self.brake_now_alert:
      events_sp.add(custom.OnroadEventSP.EventName.brakeNowAlert)
