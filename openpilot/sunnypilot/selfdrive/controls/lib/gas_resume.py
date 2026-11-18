"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
GAS_RESUME_HOLD = 0.5  # s, no braking at all
GAS_RESUME_DECEL_TIME = 2.0  # s, then a gentle decel limit
GAS_RESUME_MAX_DECEL = -0.5  # m/s^2


class GasResume:
  """After the driver releases the gas above the set speed: no braking for GAS_RESUME_HOLD, then decel limited to GAS_RESUME_MAX_DECEL for
  GAS_RESUME_DECEL_TIME, so the car coasts down instead of snapping back to the set speed. Not with a lead and not when slowing for a stop."""
  def __init__(self, dt: float) -> None:
    self.dt = dt
    self.since_gas = 1e3  # s

  def update(self, gas_pressed: bool, v_ego: float, v_cruise: float, has_lead: bool, slowing_for_stop: bool, a_target: float) -> float:
    self.since_gas = 0.0 if gas_pressed else self.since_gas + self.dt
    if v_ego > v_cruise and self.since_gas < GAS_RESUME_HOLD + GAS_RESUME_DECEL_TIME and not (has_lead or slowing_for_stop):
      return max(a_target, 0.0 if self.since_gas < GAS_RESUME_HOLD else GAS_RESUME_MAX_DECEL)
    return a_target
