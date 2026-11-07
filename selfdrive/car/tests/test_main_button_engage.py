"""
Cruise main button engage (Hyundai CCNC LFA_ALT openpilot longitudinal): set speed on engage.
Runs on the device / with the opendbc submodule at a revision that has HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE.
"""
import pytest

from cereal import car, custom
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_INITIAL, V_CRUISE_MAX, MAIN_ENGAGE_WINDOW

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type

SAFETY_PARAM = 7180 | HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE


def make_helper(brand="hyundai", safety_param=SAFETY_PARAM):
  CP = car.CarParams(pcmCruise=False, brand=brand, safetyConfigs=[{"safetyParam": int(safety_param)}])
  return VCruiseHelper(CP, custom.CarParamsSP(pcmCruiseSpeed=True))


def cs(v_ego_kph=0., events=(), available=True):
  return car.CarState(vEgo=v_ego_kph * CV.KPH_TO_MS, cruiseState={"available": available},
                      buttonEvents=[ButtonEvent(type=t, pressed=p) for t, p in events])


def press_main(helper, v_ego_kph, is_metric=True):
  """main press turns main on -> carState with the rising edge -> a few frames later carControl.enabled -> initialize"""
  helper.update_v_cruise(cs(v_ego_kph, [(ButtonType.mainCruise, True)]), enabled=False, is_metric=is_metric)
  for _ in range(5):
    helper.update_v_cruise(cs(v_ego_kph), enabled=False, is_metric=is_metric)
  helper.initialize_v_cruise(cs(v_ego_kph), False, False, is_metric)


class TestMainButtonEngage:
  def test_set_speed_is_current_speed(self):
    for kph in (35, 50.4, 88.6, 120):
      helper = make_helper()
      press_main(helper, kph)
      assert helper.v_cruise_kph == int(round(kph))
      assert helper.v_cruise_cluster_kph == helper.v_cruise_kph

  def test_below_minimum_uses_minimum_not_set_floor(self):
    # SET floors at V_CRUISE_INITIAL (40 kph); main engage uses the stock minimum, 30 kph / 20 mph
    assert V_CRUISE_INITIAL == 40
    for v_ego in (0., 5., 20.):
      helper = make_helper()
      press_main(helper, v_ego, is_metric=True)
      assert helper.v_cruise_kph == 30
      helper = make_helper()
      press_main(helper, v_ego, is_metric=False)
      assert helper.v_cruise_kph == int(round(20 * CV.MPH_TO_KPH))

  def test_clipped_to_max(self):
    helper = make_helper()
    press_main(helper, 200)
    assert helper.v_cruise_kph == V_CRUISE_MAX

  def test_set_press_unchanged_after_main_engage(self):
    # engage with main, disengage, then SET: still the SET floor
    helper = make_helper()
    press_main(helper, 10)
    assert helper.v_cruise_kph == 30
    helper.initialize_v_cruise(cs(10), False, False)
    assert helper.v_cruise_kph == V_CRUISE_INITIAL  # the main press is consumed by the first initialize

  def test_press_expires(self):
    helper = make_helper()
    helper.update_v_cruise(cs(10, [(ButtonType.mainCruise, True)]), enabled=False, is_metric=True)
    for _ in range(MAIN_ENGAGE_WINDOW + 1):
      helper.update_v_cruise(cs(10), enabled=False, is_metric=True)
    helper.initialize_v_cruise(cs(10), False, False)
    assert helper.v_cruise_kph == V_CRUISE_INITIAL

  def test_main_off_press_is_not_an_engage_request(self):
    # the press that turns main off has cruiseState.available False
    helper = make_helper()
    helper.update_v_cruise(cs(10, [(ButtonType.mainCruise, True)], available=False), enabled=False, is_metric=True)
    helper.initialize_v_cruise(cs(10), False, False)
    assert helper.v_cruise_kph == V_CRUISE_INITIAL

  @pytest.mark.parametrize("brand,param", [("hyundai", 7180), ("hyundai", 7176), ("toyota", SAFETY_PARAM)])
  def test_gate(self, brand, param):
    # without the safety flag (or on another brand) the main press has no effect on the set speed
    helper = make_helper(brand, param)
    press_main(helper, 10)
    assert helper.v_cruise_kph == V_CRUISE_INITIAL

  def test_resume_restores_previous_speed_unchanged(self):
    helper = make_helper()
    press_main(helper, 90)
    helper.v_cruise_kph_last = helper.v_cruise_kph
    helper.initialize_v_cruise(cs(10, [(ButtonType.accelCruise, False)]), False, False)
    assert helper.v_cruise_kph == 90
