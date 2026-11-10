"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.car.main_button_engage import main_button_engage

ButtonType = structs.CarState.ButtonEvent.Type


class PauseResumeKey:
  """Genesis (main-button engage): the pause key (CANCEL) is a pause/resume toggle like the stock cruise. A press that starts while openpilot
  cruise is off with main on resumes it on release, like RES: its press and release become resumeCruise events and the release enables. A press
  that starts while engaged stays a cancel. Mirrors the panda (hyundai_common_cruise_buttons_check), which also blocks it on a held brake."""
  def __init__(self, CP: structs.CarParams):
    self.active = main_button_engage(CP)
    self.resume = False

  def update(self, CS: structs.CarState, engaged: bool) -> None:
    if not self.active:
      return
    for be in CS.buttonEvents:
      if be.type != ButtonType.cancel:
        continue
      if be.pressed:
        self.resume = not engaged and CS.cruiseState.available
      if self.resume:
        be.type = ButtonType.resumeCruise
        if not be.pressed:
          CS.buttonEnable = CS.buttonEnable or (CS.cruiseState.available and not CS.brakePressed)
          self.resume = False
