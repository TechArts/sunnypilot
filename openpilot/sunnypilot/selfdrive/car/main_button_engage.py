"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import structs
from opendbc.car.structs import car
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.cruise_ext import V_CRUISE_MAX

ButtonType = car.CarState.ButtonEvent.Type

MAIN_ENGAGE_MIN_SET_SPEED = 30  # kph, or 20 mph, like the stock cruise
ENGAGE_BUTTONS = (ButtonType.mainCruise, ButtonType.setCruise, ButtonType.decelCruise, ButtonType.accelCruise)


def main_button_engage(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  """openpilot engages on the cruise main button (Hyundai CCNC LFA_ALT): the SP safety flag the car interface and panda safety also use."""
  return CP.brand == "hyundai" and bool(CP_SP.safetyParam & HyundaiSafetyFlagsSP.MAIN_BUTTON_ENGAGE)


def resume_blocked(CP: structs.CarParams, CP_SP: structs.CarParamsSP, CS: car.CarState) -> bool:
  """A resume/up press with no set speed yet (carState.vCruise unset) is blocked ('Press Set to Engage') on non-PCM cars. Not with main-button engage:
  there pause and up/SET start at the current speed (MainButtonCruise.initial_speed), which is also what the panda allows (a SET/RES/pause release
  with main on grants controls), so the NO_ENTRY must not reject them."""
  resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
  return not CP.pcmCruise and CS.vCruise > 250 and resume_pressed and not main_button_engage(CP, CP_SP)


class MainButtonCruise:
  """Set speed on a main-button engage, like the stock cruise: the pause key (resumeCruise, see PauseResumeKey) resumes the last set speed; MAIN,
  SET and up set the current speed, at least 30 kph / 20 mph (no 40 kph floor)."""
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.active = main_button_engage(CP, CP_SP)
    self.resume_requested = False  # a pause-key release (resumeCruise) is waiting for its enable, kept across card iterations

  def update_resume_request(self, CS: car.CarState) -> None:
    """selfdrived/controlsd enable some card iterations after the pause-key release, so initial_speed cannot rely on that release being in the
    carState it is given. The newest engage button decides: a pause-key release asks for the remembered speed, MAIN/SET/up for the current one;
    main off drops the request."""
    if not self.active:
      return
    if not CS.cruiseState.available:
      self.resume_requested = False
    for b in CS.buttonEvents:
      if b.type == ButtonType.resumeCruise:
        if not b.pressed:
          self.resume_requested = True
      elif b.type in (*ENGAGE_BUTTONS, ButtonType.cancel):
        self.resume_requested = False

  def initial_speed(self, CS: car.CarState, v_cruise_kph_last: float, initialized: bool, is_metric: bool) -> float | None:
    """The set speed (kph) for the enable that is being initialized, or None when this is not a main-button engage car."""
    if not self.active:
      return None
    resume = self.resume_requested or any(b.type == ButtonType.resumeCruise for b in CS.buttonEvents)
    self.resume_requested = False  # consumed with the enable
    if resume and initialized:
      return v_cruise_kph_last
    min_speed = MAIN_ENGAGE_MIN_SET_SPEED if is_metric else 20 * CV.MPH_TO_KPH
    return int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, min_speed, V_CRUISE_MAX)))
