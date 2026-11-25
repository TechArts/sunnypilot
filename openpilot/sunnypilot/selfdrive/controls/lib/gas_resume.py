"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
GAS_RESUME_HOLD = 0.5  # s, no braking at all
GAS_RESUME_DECEL_TIME = 2.0  # s, then a gentle decel limit
GAS_RESUME_MAX_DECEL = -0.5  # m/s^2

# lead braking right after a gas override (routes 84-95: 21 of 50 releases braked harder than -0.5 m/s^2 within 3 s, down to -2.5, most of
# them for the car being passed or a car 1-2 s ahead closing slowly)
LEAD_EASE_TIME = 2.5  # s after the release
LEAD_EASE_MAX_DECEL = -0.8  # m/s^2
LEAD_EASE_STOP_DISTANCE = 6.0  # m, the MPC's STOP_DISTANCE: the follow distance is v_ego * T_FOLLOW + this
LEAD_URGENT_DECEL = -2.0  # m/s^2, the lead itself brakes this hard: full braking


class GasResume:
  """After the driver releases the gas above the set speed: the cruise-speed accel candidate is raised to 0 for GAS_RESUME_HOLD, then limited to
  GAS_RESUME_MAX_DECEL for GAS_RESUME_DECEL_TIME, so the car coasts down instead of snapping back to the set speed. Only that candidate is
  eased: the planner still takes the minimum with the MPC (lead) and model candidates, so their braking is never replaced."""
  def __init__(self, dt: float) -> None:
    self.dt = dt
    self.since_gas = 1e3  # s

  def update(self, gas_pressed: bool, v_ego: float, v_cruise: float, a_cruise: float, suppress: bool = False) -> float:
    self.since_gas = 0.0 if gas_pressed else self.since_gas + self.dt
    if v_ego > v_cruise and self.since_gas < GAS_RESUME_HOLD + GAS_RESUME_DECEL_TIME and not suppress:
      return max(a_cruise, 0.0 if self.since_gas < GAS_RESUME_HOLD else GAS_RESUME_MAX_DECEL)
    return a_cruise

  @staticmethod
  def lead_needs_more(lead, v_ego: float, t_follow: float) -> bool:
    """True when LEAD_EASE_MAX_DECEL would not match the lead's speed before the follow distance (or the lead brakes hard). Route 91 seg 28,
    83 mph: at the release a car 68 m ahead closing 4.5 m/s needed 0.56 m/s^2 (eased); 1.4 s later a cut-in 43 m ahead closing 7.3 m/s was
    already inside the follow distance (full braking at once: a time-to-collision test called it 6 s away and held the braking back)."""
    if not lead.present:
      return False
    if lead.aLeadK < LEAD_URGENT_DECEL:
      return True
    margin = lead.dRel - (v_ego * t_follow + LEAD_EASE_STOP_DISTANCE)
    if margin <= 0.0:
      return True
    closing = -lead.vRel
    return closing > 0.0 and closing ** 2 / (2.0 * margin) > -LEAD_EASE_MAX_DECEL

  def ease_lead(self, a_mpc: float, lead, v_ego: float, t_follow: float, suppress: bool = False) -> float:
    """The MPC (lead) candidate right after a gas override: its braking is limited to LEAD_EASE_MAX_DECEL for LEAD_EASE_TIME, so the car does
    not brake hard the moment the driver lifts off after passing or closing in, but only while that limit is enough (lead_needs_more) and decel
    is not forced. The model candidate (stops, lights) is never eased. Call after update() in the same frame."""
    if self.since_gas < LEAD_EASE_TIME and not suppress and not self.lead_needs_more(lead, v_ego, t_follow):
      return max(a_mpc, LEAD_EASE_MAX_DECEL)
    return a_mpc
