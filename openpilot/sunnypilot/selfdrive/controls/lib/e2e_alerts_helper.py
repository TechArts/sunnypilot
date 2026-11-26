"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom
from opendbc.car import structs

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.driver_profile import DEFAULT_PROFILE, GreenLight
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

GREEN_LIGHT_X_THRESHOLD = 30
LEAD_DEPART_DIST_THRESHOLD = 1.0
TRIGGER_TIMER_THRESHOLD = 0.3
# DriverProfile.green_light (driver_profile.py); the names below are its defaults
GO_WAIT = DEFAULT_PROFILE.green_light.go_wait_s  # s, after a green / lead-depart trigger the chime waits this long for the car to start moving by itself
BRAKE_GO_LOOKBACK = DEFAULT_PROFILE.green_light.brake_release_lookback_s  # s, a brake release this long before a trigger (or during its wait): going

GearShifter = structs.CarState.GearShifter
NOT_DRIVING_GEARS = (GearShifter.park, GearShifter.reverse, GearShifter.neutral, GearShifter.unknown)


class E2EStates:
  INACTIVE = 0
  ARMED = 1
  CONSUMED = 2


class E2EAlertsHelper:
  def __init__(self, cfg: GreenLight = DEFAULT_PROFILE.green_light):
    self.cfg = cfg
    self._params = Params()
    self.frame = -1
    self.green_light_state = E2EStates.INACTIVE
    self.prev_green_light_state = E2EStates.INACTIVE
    self.lead_depart_state = E2EStates.INACTIVE
    self.prev_lead_depart_state = E2EStates.INACTIVE

    self.green_light_alert = False
    self.green_light_alert_enabled = self._params.get_bool("GreenLightAlert")
    self.lead_depart_alert = False
    self.lead_depart_alert_enabled = self._params.get_bool("LeadDepartAlert")

    self.green_light_trigger_timer = 0
    self.lead_depart_trigger_timer = 0
    self.last_lead_distance = -1
    self.last_moving_frame = -1

    self.allowed = False
    self.last_allowed = False
    self.has_lead = False

    self.lead_depart_arm_timer = 0
    self.lead_depart_confirmed_lead = False
    self.lead_depart_armed = False

    self.moving = False
    self.pending_frame: int | None = None  # frame of a trigger that waits go_wait_s before it chimes
    self.pending_green = False
    self.stop_chimed = False  # one chime per stop (green and lead-depart share it), cleared when the car moves
    self.brake_pressed = False
    self.last_brake_release_frame: int | None = None

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.green_light_alert_enabled = self._params.get_bool("GreenLightAlert")
      self.lead_depart_alert_enabled = self._params.get_bool("LeadDepartAlert")

  def update_alert_trigger(self, sm: messaging.SubMaster):
    CS = sm['carState']

    model_x = sm['modelV2'].position.x
    max_idx = len(model_x) - 1
    self.has_lead = sm['radarState'].leadOne.present
    lead_dRel = sm['radarState'].leadOne.dRel

    standstill = CS.standstill
    moving = not standstill and CS.vEgo > 0.1

    if self.brake_pressed and not CS.brakePressed:
      self.last_brake_release_frame = self.frame
    self.brake_pressed = CS.brakePressed

    if moving:
      self.last_moving_frame = self.frame
    recent_moving = self.last_moving_frame == -1 or (self.frame - self.last_moving_frame) * DT_MDL < 2.0

    # only while stopped in a forward gear: in park or reverse (parking, backing away from a car) a 'lead departing' or a long model path
    # is not a cue to drive off
    in_drive = CS.gearShifter not in NOT_DRIVING_GEARS
    # universal: openpilot cruise on or off (the chime waits for the car to start moving by itself, see update())
    self.allowed = in_drive and not moving and not CS.gasPressed and not recent_moving
    self.moving = moving

    # Green Light Alert
    green_light_trigger = False
    if self.green_light_state == E2EStates.ARMED:
      if model_x[max_idx] > GREEN_LIGHT_X_THRESHOLD:
        self.green_light_trigger_timer += 1
      else:
        self.green_light_trigger_timer = 0

      if self.green_light_trigger_timer * DT_MDL > TRIGGER_TIMER_THRESHOLD:
        green_light_trigger = True
    elif self.green_light_state != E2EStates.ARMED:
      self.green_light_trigger_timer = 0

    # Lead Departure Alert
    close_lead_valid = self.has_lead and lead_dRel < 8.0
    if self.allowed and not self.last_allowed and close_lead_valid:
      self.lead_depart_confirmed_lead = True
    elif not self.allowed:
      self.lead_depart_confirmed_lead = False

    if self.allowed and self.lead_depart_confirmed_lead and close_lead_valid:
      self.lead_depart_arm_timer += 1

      if self.lead_depart_arm_timer * DT_MDL >= 1.0:
        self.lead_depart_armed = True
    else:
      self.lead_depart_arm_timer = 0
      self.lead_depart_armed = False

    lead_depart_trigger = False
    if self.lead_depart_state == E2EStates.ARMED:
      if self.last_lead_distance == -1 or lead_dRel < self.last_lead_distance:
        self.last_lead_distance = lead_dRel

      if self.last_lead_distance != -1 and (lead_dRel - self.last_lead_distance > LEAD_DEPART_DIST_THRESHOLD):
        self.lead_depart_trigger_timer += 1
      else:
        self.lead_depart_trigger_timer = 0

      if self.lead_depart_trigger_timer * DT_MDL > TRIGGER_TIMER_THRESHOLD:
        lead_depart_trigger = True
    elif self.lead_depart_state != E2EStates.ARMED:
      self.last_lead_distance = -1
      self.lead_depart_trigger_timer = 0

    self.last_allowed = self.allowed

    return green_light_trigger, lead_depart_trigger

  def _driver_released_brake(self, since_frame: int) -> bool:
    """The driver let go of the brake at most brake_release_lookback_s before since_frame and has not pressed it again: already going.
    A release edge, not 'brake up': with the car holding the stop (Auto Hold, openpilot cruise) the foot is already off the brake."""
    return (not self.brake_pressed and self.last_brake_release_frame is not None and
            self.last_brake_release_frame >= since_frame - round(self.cfg.brake_release_lookback_s / DT_MDL))

  @staticmethod
  def update_state_machine(state: int, enabled: bool, allowed: bool, triggered: bool) -> tuple[int, bool]:
    if state != E2EStates.INACTIVE:
      if not allowed or not enabled:
        state = E2EStates.INACTIVE

      else:
        if state == E2EStates.ARMED:
          if triggered:
            state = E2EStates.CONSUMED

        elif state == E2EStates.CONSUMED:
          pass

    elif state == E2EStates.INACTIVE:
      if allowed and enabled:
        state = E2EStates.ARMED

    return state, triggered

  def update(self, sm: messaging.SubMaster, events_sp: EventsSP) -> None:
    self._read_params()

    green_light_trigger, lead_depart_trigger = self.update_alert_trigger(sm)

    self.prev_green_light_state = self.green_light_state
    self.prev_lead_depart_state = self.lead_depart_state

    self.green_light_state, self.green_light_alert = self.update_state_machine(
      self.green_light_state,
      self.green_light_alert_enabled,
      self.allowed and not self.has_lead,
      green_light_trigger
    )

    self.lead_depart_state, self.lead_depart_alert = self.update_state_machine(
      self.lead_depart_state,
      self.lead_depart_alert_enabled,
      self.allowed and self.lead_depart_armed,
      lead_depart_trigger
    )

    # The chime sounds only if the car is still stopped go_wait_s after the trigger with no gas and no brake release: a car that starts moving by itself
    # (openpilot or the car's cruise, the driver) or a driver who already let go of the brake needs no cue. Once per stop. The published flags mark
    # the chime frame (bookmark, cluster cue).
    triggered = self.green_light_alert or self.lead_depart_alert
    if self.moving:
      self.stop_chimed = False
    if not self.allowed:
      self.pending_frame = None
    elif triggered and self.pending_frame is None and not self.stop_chimed:
      self.pending_frame, self.pending_green = self.frame, self.green_light_alert
    if self.pending_frame is not None and not (self.green_light_alert_enabled if self.pending_green else self.lead_depart_alert_enabled):
      self.pending_frame = None  # its alert was turned off during the wait
    due = self.pending_frame is not None and (self.frame - self.pending_frame) * DT_MDL >= self.cfg.go_wait_s
    if due and self._driver_released_brake(self.pending_frame):
      self.pending_frame = None  # decided at the deadline: the driver let go of the brake and has not pressed it again, already going
    chime = due and self.pending_frame is not None
    self.green_light_alert = chime and self.pending_green
    self.lead_depart_alert = chime and not self.pending_green
    if chime:
      events_sp.add(custom.OnroadEventSP.EventName.e2eChime)
      self.stop_chimed, self.pending_frame = True, None

    self.frame += 1
