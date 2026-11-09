"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Sequence

from cereal import messaging, custom

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

MIN_SPEED = 15 * CV.MPH_TO_MS           # m/s, no cue below this
MIN_ACCEL = -0.5                        # m/s^2, driver is already slowing down at or below this
STOP_SPEED_THRESHOLD = 1.0              # m/s, the model's planned speed at its horizon counts as stopped below this
STOP_SPEED_RATIO = 0.25                 # ... and it must also be well below the current speed
STOP_DIST_MAX = 80.0                    # m, the model's planned stop point (end of its path) must be nearer than this
LEAD_STOP_MARGIN = 10.0                 # m, a lead this close to the planned stop point explains the stop, so no cue
HOLD_TIME = 0.3                         # s, condition must hold this long before the cue fires
REARM_CLEAR_TIME = 5.0                  # s, condition must stay clear this long to re-arm after a cue
STOPPED_SPEED = 0.5                     # m/s, the car being below this re-arms the cue


class RedLightStates:
  ARMED = 0
  FIRED = 1


def model_plans_stop(model_x: Sequence[float], model_v: Sequence[float], v_ego: float, lead_status: bool, lead_d_rel: float) -> bool:
  """The model's path ends (10 s horizon) at near-zero speed close ahead, and no lead vehicle explains that stop."""
  if len(model_x) == 0 or len(model_x) != len(model_v):
    return False

  stop_dist = model_x[-1]
  stops = model_v[-1] < min(STOP_SPEED_THRESHOLD, STOP_SPEED_RATIO * v_ego)
  if not stops or stop_dist > STOP_DIST_MAX:
    return False

  return not (lead_status and lead_d_rel < stop_dist + LEAD_STOP_MARGIN)


def red_ahead_not_slowing(v_ego: float, a_ego: float, brake_pressed: bool, blinker: bool, stop_planned: bool) -> bool:
  return stop_planned and v_ego > MIN_SPEED and a_ego > MIN_ACCEL and not brake_pressed and not blinker


class RedLightAlertHelper:
  def __init__(self):
    self._params = Params()
    self.frame = -1
    self.enabled = self._params.get_bool("RedLightAlert")

    self.state = RedLightStates.ARMED
    self.hold_timer = 0
    self.clear_timer = 0
    self.red_light_alert = False

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self._params.get_bool("RedLightAlert")

  def step(self, condition: bool, v_ego: float) -> bool:
    """Advance the debounce / one-shot state machine one model frame. Returns True on the frame the cue fires."""
    if not self.enabled:
      self.state = RedLightStates.ARMED
      self.hold_timer = 0
      self.clear_timer = 0
      return False

    if self.state == RedLightStates.ARMED:
      self.hold_timer = self.hold_timer + 1 if condition else 0
      if self.hold_timer * DT_MDL >= HOLD_TIME:
        self.state = RedLightStates.FIRED
        self.hold_timer = 0
        self.clear_timer = 0
        return True
    else:
      self.clear_timer = 0 if condition else self.clear_timer + 1
      if v_ego < STOPPED_SPEED or self.clear_timer * DT_MDL >= REARM_CLEAR_TIME:
        self.state = RedLightStates.ARMED
        self.hold_timer = 0
        self.clear_timer = 0

    return False

  @staticmethod
  def read_condition(sm: messaging.SubMaster) -> bool:
    CS = sm['carState']
    model = sm['modelV2']
    lead = sm['radarState'].leadOne

    stop_planned = model_plans_stop(model.position.x, model.velocity.x, CS.vEgo, lead.status, lead.dRel)
    return red_ahead_not_slowing(CS.vEgo, CS.aEgo, CS.brakePressed, CS.leftBlinker or CS.rightBlinker, stop_planned)

  def update(self, sm: messaging.SubMaster, events_sp: EventsSP) -> None:
    self._read_params()

    self.red_light_alert = self.step(self.read_condition(sm), sm['carState'].vEgo)
    if self.red_light_alert:
      events_sp.add(custom.OnroadEventSP.EventName.redLightAlert)

    self.frame += 1
