from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.gas_resume import GasResume, GAS_RESUME_HOLD, GAS_RESUME_DECEL_TIME, GAS_RESUME_MAX_DECEL

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
