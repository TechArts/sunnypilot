from types import SimpleNamespace

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.gas_resume import GasResume, GAS_RESUME_HOLD, GAS_RESUME_DECEL_TIME, GAS_RESUME_MAX_DECEL, \
  LEAD_EASE_TIME, LEAD_EASE_MAX_DECEL

DT = 0.05


def run(g, seconds, gas=False, v_ego=30., v_cruise=25., suppress=False, a_cruise=-2.0):
  out = a_cruise
  for _ in range(round(seconds / DT)):
    out = g.update(gas, v_ego, v_cruise, a_cruise, suppress)
  return out


class TestGasResume(OpenpilotTestCase):
  def test_no_braking_during_the_hold_then_gentle_decel_then_normal(self):
    g = GasResume(DT)
    run(g, 1., gas=True)
    assert run(g, GAS_RESUME_HOLD - 2 * DT) == 0.0
    assert run(g, 2 * DT + DT) == GAS_RESUME_MAX_DECEL  # past the hold
    assert run(g, GAS_RESUME_DECEL_TIME) == -2.0  # past the decel window

  def test_gas_pressed_itself_is_not_limited_below_zero(self):
    g = GasResume(DT)
    assert run(g, 1., gas=True) == 0.0

  def test_below_the_set_speed_is_untouched(self):
    g = GasResume(DT)
    run(g, 1., gas=True, v_ego=20.)
    assert run(g, 0.1, v_ego=20.) == -2.0

  def test_suppressed_is_untouched(self):
    g = GasResume(DT)
    run(g, 1., gas=True, suppress=True)
    assert run(g, 0.1, suppress=True) == -2.0

  def test_a_cruise_above_the_limit_is_kept(self):
    g = GasResume(DT)
    run(g, 1., gas=True, a_cruise=0.8)
    assert run(g, 1., a_cruise=0.8) == 0.8
    assert run(g, 1., a_cruise=-0.2) == -0.2

  def test_never_pressed_gas_is_untouched(self):
    g = GasResume(DT)
    assert run(g, 3., a_cruise=-1.5) == -1.5

  def test_mpc_and_model_braking_still_win_the_planner_min(self):
    # the planner takes min(mpc, eased cruise, model): during the easing window a lead / model brake request is not replaced by the eased 0
    g = GasResume(DT)
    run(g, 1., gas=True)
    cruise = run(g, 0.1, a_cruise=-0.4)
    assert cruise == 0.0
    for mpc, model in ((-1.5, 0.3), (0.2, -2.0), (-0.8, -0.9)):
      assert min(mpc, cruise, model) == min(mpc, model)


def lead(d_rel, v_rel, a_lead=0.0, present=True):
  return SimpleNamespace(present=present, dRel=d_rel, vRel=v_rel, aLeadK=a_lead)


NO_LEAD = lead(0., 0., present=False)


def leads(lead_one, lead_two=NO_LEAD):
  return (lead_one, lead_two)


class TestLeadEase(OpenpilotTestCase):
  V = 35.0  # m/s, 78 mph
  T = 1.25  # s, aggressive (the driver's personality on routes 85-95): follow distance at V = 49.75 m

  def released(self, seconds=0.5):
    g = GasResume(DT)
    run(g, 1., gas=True, v_ego=self.V)
    run(g, seconds, v_ego=self.V)
    return g

  def test_route_91_release_is_eased(self):
    # route 91 seg 28 at the release: lead 68 m closing 4.5 m/s needs 0.56 m/s^2 before the follow distance; the planner asked -2.5
    g = self.released()
    assert g.ease_lead(-2.5, leads(lead(68., -4.5)), self.V, self.T) == LEAD_EASE_MAX_DECEL

  def test_route_91_cut_in_brakes_fully(self):
    # 1.4 s later a cut-in 43 m ahead closing 7.3 m/s: inside the follow distance, no easing (a time-to-collision rule held it back to -0.8
    # for the rest of the window, then the plan stepped to -3.1)
    g = self.released()
    assert g.ease_lead(-1.8, leads(lead(43., -7.3)), 35.8, self.T) == -1.8

  def test_predicted_cut_in_brakes_fully(self):
    # no leadOne, but the model's car expected ahead (leadTwo) 30 m away closing: the MPC brakes for it, so no easing (review R1 2026-11-25)
    g = self.released()
    assert g.ease_lead(-2.0, leads(NO_LEAD, lead(30., -3.0)), self.V, self.T) == -2.0
    assert g.ease_lead(-2.0, leads(lead(68., -4.5), lead(30., -3.0)), self.V, self.T) == -2.0

  def test_more_than_the_limit_needed_brakes_fully(self):
    g = self.released()
    assert g.ease_lead(-2.5, leads(lead(60., -6.0)), self.V, self.T) == -2.5     # needs 1.75 m/s^2
    assert g.ease_lead(-2.5, leads(lead(45., 0.0)), self.V, self.T) == -2.5      # inside the follow distance, not closing
    assert g.ease_lead(-2.5, leads(lead(80., -1.0, -3.0)), self.V, self.T) == -2.5  # lead braking hard
    assert g.ease_lead(-2.5, leads(lead(68., -4.5)), self.V, 1.75) == -2.5      # relaxed: follow distance 67 m

  def test_forced_decel_is_not_eased(self):
    g = self.released()
    assert g.ease_lead(-2.5, leads(lead(68., -4.5)), self.V, self.T, suppress=True) == -2.5

  def test_only_right_after_the_gas(self):
    g = self.released(LEAD_EASE_TIME + DT)
    assert g.ease_lead(-2.5, leads(lead(68., -4.5)), self.V, self.T) == -2.5
    assert GasResume(DT).ease_lead(-2.5, leads(lead(68., -4.5)), self.V, self.T) == -2.5  # never pressed

  def test_milder_braking_and_accel_untouched(self):
    g = self.released()
    assert g.ease_lead(-0.3, leads(lead(68., -4.5)), self.V, self.T) == -0.3
    assert g.ease_lead(0.4, leads(NO_LEAD), self.V, self.T) == 0.4
