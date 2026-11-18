"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import DT_CTRL
from opendbc.car.structs import car

ButtonType = car.CarState.ButtonEvent.Type

# the engage of openpilot from the same press that started the car's ACC enables within 0.05 s and must not chime
STOCK_ACC_DELAY = 0.2  # s


class StockAccMonitor:
  """The car's own ACC is engaged on the camera (carStateSP.stockAccEngaged) while openpilot is not: say so at once and audibly.
  The pause key held to resume: the camera resumes while the key is down, openpilot only on its release, so the key hold is not reported."""
  def __init__(self) -> None:
    self.counter = 0
    self.resume_key_held = False

  def update(self, stock_acc_engaged: bool, enabled: bool, main_available: bool, button_events) -> bool:
    for be in button_events:
      if be.type == ButtonType.resumeCruise:
        self.resume_key_held = be.pressed
    if enabled or not main_available:
      self.resume_key_held = False  # a lost release frame must not mute the alert for good

    stock_acc = stock_acc_engaged and not enabled and not self.resume_key_held
    self.counter = self.counter + 1 if stock_acc else 0
    return self.counter > int(STOCK_ACC_DELAY / DT_CTRL)
