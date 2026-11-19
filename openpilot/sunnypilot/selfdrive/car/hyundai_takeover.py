"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs

TAKEOVER_ALERT_WINDOW = 2.0  # s, after a takeover a steer fault or an undershooting turn is the driver's doing


def hyundai_takeover(CP: structs.CarParams, CP_SP: structs.CarParamsSP, CS: structs.CarState, release: bool = False) -> bool:
  """A takeover is the wheel pressed or, on Hyundai with the opendbc takeover filter on (HyundaiTakeoverTorque > 0), a pull at the filter's
  torque (release=False: the car pauses steering then) or at its release torque (release=True: the filter keeps steering paused, then resumes
  it blended, until the torque has stayed below it for the hold time)."""
  if CS.steeringPressed:
    return True
  if CP.brand != "hyundai" or CP_SP.hyundaiTakeoverTorque <= 0:
    return False
  threshold = CP_SP.hyundaiTakeoverReleaseTorque if release else CP_SP.hyundaiTakeoverTorque
  return abs(CS.steeringTorque) >= threshold
