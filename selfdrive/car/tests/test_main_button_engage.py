"""
Cruise main button engage (Hyundai CCNC LFA_ALT openpilot longitudinal, Genesis buttons): set speed on engage, like the stock cruise.
Runs on the device / with the opendbc submodule at a revision that has HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE.
"""
import pytest

from cereal import car, custom
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_INITIAL, V_CRUISE_MAX

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type

SAFETY_PARAM = 7180 | HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE
MIN_MPH_KPH = int(round(20 * CV.MPH_TO_KPH))


def make_helper(brand="hyundai", safety_param=SAFETY_PARAM):
  CP = car.CarParams(pcmCruise=False, brand=brand, safetyConfigs=[{"safetyParam": int(safety_param)}])
  return VCruiseHelper(CP, custom.CarParamsSP(pcmCruiseSpeed=True))


def cs(v_ego_kph=0., events=(), available=True):
  return car.CarState(vEgo=v_ego_kph * CV.KPH_TO_MS, cruiseState={"available": available},
                      buttonEvents=[ButtonEvent(type=t, pressed=p) for t, p in events])


def engage(helper, v_ego_kph, button, is_metric=True):
  """the carState of the frame selfdrived enables on goes to initialize_v_cruise"""
  helper.initialize_v_cruise(cs(v_ego_kph, [(button, False)]), False, False, is_metric)


ENGAGE_BUTTONS = (ButtonType.mainCruise, ButtonType.setCruise, ButtonType.decelCruise, ButtonType.accelCruise)


class TestMainButtonEngage:
  @pytest.mark.parametrize("button", ENGAGE_BUTTONS)
  def test_set_speed_is_current_speed(self, button):
    for kph in (35, 50.4, 88.6, 120):
      helper = make_helper()
      engage(helper, kph, button)
      assert helper.v_cruise_kph == int(round(kph))
      assert helper.v_cruise_cluster_kph == helper.v_cruise_kph

  @pytest.mark.parametrize("button", ENGAGE_BUTTONS)
  def test_below_minimum_uses_stock_minimum(self, button):
    # the stock cruise sets at least 20 mph, not openpilot's 40 kph SET floor
    for v_ego in (0., 5., 20.):
      helper = make_helper()
      engage(helper, v_ego, button, is_metric=True)
      assert helper.v_cruise_kph == 30
      helper = make_helper()
      engage(helper, v_ego, button, is_metric=False)
      assert helper.v_cruise_kph == MIN_MPH_KPH

  def test_clipped_to_max(self):
    helper = make_helper()
    engage(helper, 200, ButtonType.mainCruise)
    assert helper.v_cruise_kph == V_CRUISE_MAX

  def test_up_in_standby_sets_current_speed_not_the_last(self):
    helper = make_helper()
    engage(helper, 90, ButtonType.mainCruise)
    helper.v_cruise_kph_last = helper.v_cruise_kph
    engage(helper, 60, ButtonType.accelCruise)
    assert helper.v_cruise_kph == 60

  def test_pause_key_resumes_the_last_set_speed(self):
    helper = make_helper()
    engage(helper, 90, ButtonType.mainCruise)
    helper.v_cruise_kph_last = helper.v_cruise_kph
    engage(helper, 60, ButtonType.resumeCruise)
    assert helper.v_cruise_kph == 90

  def test_pause_key_without_a_set_speed_sets_current_speed(self):
    helper = make_helper()
    engage(helper, 60, ButtonType.resumeCruise)
    assert helper.v_cruise_kph == 60

  @pytest.mark.parametrize("brand,param", [("hyundai", 7180), ("hyundai", 7176), ("toyota", SAFETY_PARAM)])
  def test_gate(self, brand, param):
    # without the safety flag (or on another brand): openpilot's own rules, SET floor and accel resumes
    helper = make_helper(brand, param)
    engage(helper, 10, ButtonType.mainCruise)
    assert helper.v_cruise_kph == V_CRUISE_INITIAL
    helper.v_cruise_kph = 90
    helper.v_cruise_kph_last = 90
    engage(helper, 60, ButtonType.accelCruise)
    assert helper.v_cruise_kph == 90
