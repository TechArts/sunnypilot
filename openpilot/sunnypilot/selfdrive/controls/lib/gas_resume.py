"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.sunnypilot.selfdrive.driver_profile import DEFAULT_PROFILE, GasResume as GasResumeProfile

# The tunable times and decels are DriverProfile.gas_resume (driver_profile.py); the names below are their defaults.
_D = DEFAULT_PROFILE.gas_resume
GAS_RESUME_HOLD = _D.hold_s  # s, no braking at all
GAS_RESUME_DECEL_TIME = _D.decel_time_s  # s, then a gentle decel limit
GAS_RESUME_MAX_DECEL = _D.max_decel_mps2  # m/s^2

# lead braking right after a gas override (routes 84-95: 21 of 50 releases braked harder than -0.5 m/s^2 within 3 s, down to -2.5, most of
# them for the car being passed or a car 1-2 s ahead closing slowly)
LEAD_EASE_TIME = _D.lead_ease_s  # s after the release
LEAD_EASE_MAX_DECEL = _D.lead_max_decel_mps2  # m/s^2
# hard limits, not in the profile
LEAD_EASE_STOP_DISTANCE = 6.0  # m, the MPC's STOP_DISTANCE: the follow distance is v_ego * T_FOLLOW + this
LEAD_URGENT_DECEL = -2.0  # m/s^2, the lead itself brakes this hard: full braking


class GasResume:
  """After the driver releases the gas above the set speed: the cruise-speed accel candidate is raised to 0 for hold_s, then limited to
  max_decel_mps2 for decel_time_s, so the car coasts down instead of snapping back to the set speed. Only that candidate is eased: the planner
  still takes the minimum with the MPC (lead) and model candidates, so their braking is never replaced. enabled false: upstream's planner."""
  def __init__(self, dt: float, cfg: GasResumeProfile = _D) -> None:
    self.dt = dt
    self.cfg = cfg
    self.since_gas = 1e3  # s

  def update(self, gas_pressed: bool, v_ego: float, v_cruise: float, a_cruise: float, suppress: bool = False) -> float:
    self.since_gas = 0.0 if gas_pressed else self.since_gas + self.dt
    cfg = self.cfg
    if cfg.enabled and v_ego > v_cruise and self.since_gas < cfg.hold_s + cfg.decel_time_s and not suppress:
      return max(a_cruise, 0.0 if self.since_gas < cfg.hold_s else cfg.max_decel_mps2)
    return a_cruise

  @staticmethod
  def lead_needs_more(lead, v_ego: float, t_follow: float, max_decel: float = LEAD_EASE_MAX_DECEL) -> bool:
    """True when max_decel would not match the lead's speed before the follow distance (or the lead brakes hard). Route 91 seg 28,
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
    return closing > 0.0 and closing ** 2 / (2.0 * margin) > -max_decel

  def ease_lead(self, a_mpc: float, leads, v_ego: float, t_follow: float, suppress: bool = False) -> float:
    """The MPC (lead) candidate right after a gas override: its braking is limited to lead_max_decel_mps2 for lead_ease_s, so the car does
    not brake hard the moment the driver lifts off after passing or closing in, but only while that limit is enough for every lead the MPC
    brakes for (radarState leadOne and leadTwo, the model's car expected ahead, e.g. a cut-in) and decel is not forced. The model candidate
    (stops, lights) is never eased. Call after update() in the same frame."""
    cfg = self.cfg
    if (cfg.enabled and self.since_gas < cfg.lead_ease_s and not suppress and
        not any(self.lead_needs_more(lead, v_ego, t_follow, cfg.lead_max_decel_mps2) for lead in leads)):
      return max(a_mpc, cfg.lead_max_decel_mps2)
    return a_mpc
