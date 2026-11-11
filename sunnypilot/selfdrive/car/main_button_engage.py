"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import car
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiSafetyFlags


def main_button_engage(CP: structs.CarParams) -> bool:
  """openpilot engages on the cruise main button (Hyundai CCNC LFA_ALT): the flag the car interface and panda safety also use."""
  return CP.brand == "hyundai" and bool(CP.safetyConfigs) and \
         bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE)


def resume_blocked(CP, CS) -> bool:
  """A resume/up press with no set speed yet (carState.vCruise unset) is blocked ('Press Set to Engage') on non-PCM cars. Not with main-button engage:
  there pause and up/SET start at the current speed (VCruiseHelper.initialize_v_cruise), which is also what the panda allows (a SET/RES/pause release
  with main on grants controls), so the NO_ENTRY must not reject them."""
  resume_pressed = any(be.type in (car.CarState.ButtonEvent.Type.accelCruise, car.CarState.ButtonEvent.Type.resumeCruise) for be in CS.buttonEvents)
  return not CP.pcmCruise and CS.vCruise > 250 and resume_pressed and not main_button_engage(CP)
