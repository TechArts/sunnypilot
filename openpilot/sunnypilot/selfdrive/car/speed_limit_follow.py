"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Speed-limit follow (Genesis GV70, main-button engage): MAIN held >= 0.5 s from cruise off engages openpilot cruise as a short MAIN does, then
sets the speed to the speed limit + the driver's offset and keeps following every accepted limit change. The camera never sees the long press:
the car controller passes a held MAIN on as a short one (opendbc ccnc.py MAIN_FWD_MAX_FRAMES), so it does not enter its own limiter mode.
- RES+/SET- while following change the offset by the step they make on the set speed, kept per class (street / freeway) in params. Only a
  set-speed change in the button's own direction while it is held (or on its release) counts, and none while the gas is pressed.
- A school-zone limit (resolver policy vote, weekday school hours) below the set speed waits for a SET- tap; the limit itself, no offset.
- No limit known: the set speed stays.
- Leaving: as any cruise (brake, pause key, CANCEL, MAIN). RES+ or the pause key resume following; SET- or a short MAIN start a plain cruise.
- Off while Speed Limit Assist (SpeedLimitMode assist) is on: both would write the set speed.
"""
from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.cruise import IMPERIAL_INCREMENT, V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.car.main_button_engage import main_button_engage
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

ButtonType = car.CarState.ButtonEvent.Type

MAIN_LONG_FRAMES = round(0.5 / DT_CTRL)
FREEWAY_LIMIT = {False: 55, True: 90}  # mph / km/h: a limit at or above it uses the freeway offset
OFFSET_RANGE = (-10, 25)  # displayed units
OFFSET_PARAMS = {False: "SpeedLimitFollowOffsetStreet", True: "SpeedLimitFollowOffsetFreeway"}
RESUME_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise)
PLAIN_BUTTONS = (ButtonType.decelCruise, ButtonType.setCruise, ButtonType.mainCruise)
SPEED_BUTTONS = {ButtonType.accelCruise: 1, ButtonType.resumeCruise: 1, ButtonType.decelCruise: -1, ButtonType.setCruise: -1}


class SpeedLimitFollow:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params | None = None):
    self.params = params or Params()
    self.available = (main_button_engage(CP, CP_SP) and not CP.pcmCruise and
                      self.params.get("SpeedLimitMode", return_default=True) != Mode.assist)
    self.offsets = {fw: int(self.params.get(OFFSET_PARAMS[fw], return_default=True)) for fw in (False, True)}
    self.follow = False  # following now (cruise engaged in this mode)
    self.resume_follow = False  # the last session followed: RES+ / pause key resume it
    self.main_held_frames = 0  # 0 = MAIN up
    self.main_from_off = False
    self.main_long_used = False
    self.last_engage_button = None
    self.speed_button_dir = 0  # +1 / -1 while RES+ / SET- is held, and on the frame it is released (its short-press step lands there)
    self._speed_released = False
    self.enabled_prev = False
    self.applied_kph: float | None = None
    self.school_pending = False
    self.school_accepted = False
    self.offset_in_use = 0

  def _buttons(self, CS: car.CarState) -> None:
    held_dir = self.speed_button_dir if self.speed_button_dir and not self._speed_released else 0
    self._speed_released = False
    if self.main_held_frames:
      self.main_held_frames += 1
    for b in CS.buttonEvents:
      if b.type == ButtonType.mainCruise:
        if b.pressed:
          self.main_held_frames, self.main_long_used = 1, False
          self.main_from_off = not CS.cruiseState.available
          self.last_engage_button = ButtonType.mainCruise  # MAIN engages on the press (the camera's main rises before the release)
          if CS.cruiseState.available:  # MAIN turns cruise off
            self.follow = self.resume_follow = False
        else:
          self.main_held_frames = 0
      elif b.type.raw in SPEED_BUTTONS:  # .raw: a capnp enum does not hash like its int (as VCruiseHelper's button_timers)
        held_dir = SPEED_BUTTONS[b.type.raw]
        if not b.pressed:
          self.last_engage_button = b.type
          self._speed_released = True  # its short-press step lands on this frame
    self.speed_button_dir = held_dir

  def _set_offset(self, freeway: bool, value: int) -> None:
    value = max(OFFSET_RANGE[0], min(OFFSET_RANGE[1], value))
    if value != self.offsets[freeway]:
      self.offsets[freeway] = value
      self.params.put(OFFSET_PARAMS[freeway], value)  # block=False: queued, card's loop does not wait for the disk

  def update(self, CS: car.CarState, enabled: bool, resolver, v_cruise_helper, is_metric: bool) -> None:
    """After VCruiseHelper.update_v_cruise / initialize_v_cruise: overrides its set speed while following. resolver: longitudinalPlanSP
    .speedLimit.resolver."""
    if not self.available:
      return
    self._buttons(CS)

    if not CS.cruiseState.available:
      self.follow = self.resume_follow = False
    if enabled and not self.enabled_prev:
      if self.resume_follow and self.last_engage_button in RESUME_BUTTONS:
        self.follow = True
      elif self.last_engage_button in PLAIN_BUTTONS:
        self.resume_follow = False
      self.applied_kph = None
    if enabled and not self.follow and not self.main_long_used and self.main_from_off and self.main_held_frames >= MAIN_LONG_FRAMES:
      self.follow = self.resume_follow = self.main_long_used = True
      self.applied_kph = None
    if not enabled:
      if self.follow:
        self.resume_follow = True
      self.follow = False
      self.school_pending = self.school_accepted = False
    self.enabled_prev = enabled
    if not self.follow:
      return

    # set speeds on VCruiseHelper's own grid (whole km/h, or IMPERIAL_INCREMENT steps per mph), so its long-press rounding moves whole units
    unit_kph = 1. if is_metric else IMPERIAL_INCREMENT
    limit_u = round(resolver.speedLimitLast * (CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH)) if resolver.speedLimitLastValid else 0
    freeway = limit_u >= FREEWAY_LIMIT[is_metric]

    # the driver's RES+ / SET- since our last set speed: an offset change, or the school-zone acceptance
    if self.applied_kph is not None and self.speed_button_dir and not CS.gasPressed:
      delta_u = round((v_cruise_helper.v_cruise_kph - self.applied_kph) / unit_kph)
      if delta_u * self.speed_button_dir > 0:
        if self.school_pending and delta_u < 0:
          self.school_accepted = True
        elif limit_u > 0 and not resolver.schoolZone:
          self._set_offset(freeway, self.offsets[freeway] + delta_u)

    if limit_u <= 0:
      self.applied_kph = v_cruise_helper.v_cruise_kph  # no limit known: the set speed stays (the driver's buttons work as usual)
      return

    current_u = round(v_cruise_helper.v_cruise_kph / unit_kph) if self.applied_kph is not None else None
    if resolver.schoolZone:
      target_u = limit_u
      self.school_pending = current_u is not None and target_u < current_u and not self.school_accepted
      if self.school_pending:
        target_u = current_u
    else:
      self.school_pending = self.school_accepted = False
      target_u = limit_u + self.offsets[freeway]
    self.offset_in_use = target_u - limit_u

    target_kph = max(v_cruise_helper.v_cruise_min, min(V_CRUISE_MAX, round(target_u * unit_kph, 1)))
    v_cruise_helper.v_cruise_kph = v_cruise_helper.v_cruise_cluster_kph = target_kph
    self.applied_kph = target_kph

  def publish(self, cs_sp) -> None:
    """Fill the card-owned carStateSP fields (custom.capnp CarStateSP)."""
    cs_sp.speedLimitFollow = self.follow
    cs_sp.speedLimitFollowOffset = float(self.offset_in_use)
    cs_sp.speedLimitSchoolPending = self.school_pending
