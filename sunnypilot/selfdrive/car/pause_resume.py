"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.car.main_button_engage import main_button_engage

ButtonType = structs.CarState.ButtonEvent.Type


def panda_fault(pandaStates) -> bool:
  """The panda state that cancels a pending pause-key resume, the same contract as the panda's (hyundai_common_cancel_resume_fault_check): rx checks
  invalid. A lost heartbeat re-initializes the panda safety, which drops the panda's request too. The heartbeat engaged-mismatch counts are not
  visible to openpilot and not part of the contract."""
  return any(ps.heartbeatLost or ps.safetyRxChecksInvalid for ps in pandaStates)


class PauseResumeKey:
  """Genesis (main-button engage): the pause key (CANCEL) is a pause/resume toggle like the stock cruise. A press that starts while openpilot
  cruise is off with main on resumes it on release, like RES: its press and release become resumeCruise events and the release enables. A press
  that starts while engaged stays a cancel. Mirrors the panda (hyundai_common_cruise_buttons_check), which also blocks it on a held brake.

  The resume lives for one uninterrupted hold, like the panda's: a brake press (also released again before the key), a main change (off, or off and
  on again) or a CAN/panda fault (panda_fault, CAN invalid) while the key is down cancels it; the release is then a plain
  cancel and the driver has to start a new press."""
  def __init__(self, CP: structs.CarParams):
    self.active = main_button_engage(CP)
    self.resume = False
    self.main_prev = False

  def update(self, CS: structs.CarState, engaged: bool, fault: bool = False) -> None:
    if not self.active:
      return
    available = CS.cruiseState.available
    if self.resume and (CS.brakePressed or fault or not CS.canValid or CS.canTimeout or available != self.main_prev):
      self.resume = False
    self.main_prev = available

    for be in CS.buttonEvents:
      if be.type != ButtonType.cancel:
        continue
      if be.pressed:
        self.resume = not engaged and available and not fault
      if self.resume:
        be.type = ButtonType.resumeCruise
        if not be.pressed:
          CS.buttonEnable = CS.buttonEnable or (available and not CS.brakePressed)
          self.resume = False
          break
