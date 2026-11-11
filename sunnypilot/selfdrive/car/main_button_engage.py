"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiSafetyFlags


def main_button_engage(CP: structs.CarParams) -> bool:
  """openpilot engages on the cruise main button (Hyundai CCNC LFA_ALT): the flag the car interface and panda safety also use."""
  return CP.brand == "hyundai" and bool(CP.safetyConfigs) and \
         bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE)
