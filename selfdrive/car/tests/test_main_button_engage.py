"""
Cruise main button engage (Hyundai CCNC LFA_ALT openpilot longitudinal, Genesis buttons): set speed on engage, like the stock cruise.
Runs on the device / with the opendbc submodule at a revision that has HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE.
"""
import pytest

from cereal import car, custom, log
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_INITIAL, V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.selfdrived.events import Events, EventName
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.selfdrive.car.main_button_engage import resume_blocked
from openpilot.sunnypilot.selfdrive.car.pause_resume import PauseResumeKey

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


SP = structs.CarState.ButtonEvent.Type
State = log.SelfdriveState.OpenpilotState


class Drive:
  """card -> selfdrived -> card for the Genesis buttons: PauseResumeKey, buttonEnable / buttonCancel / resumeBlocked events, the state machine,
  then the carControl.enabled that card sees `enable_delay` iterations after selfdrived's state changed; card initializes the set speed on it."""

  def __init__(self, enable_delay=0, safety_param=SAFETY_PARAM):
    self.CP = car.CarParams(pcmCruise=False, brand="hyundai", safetyConfigs=[{"safetyParam": int(safety_param)}])
    self.helper = VCruiseHelper(self.CP, custom.CarParamsSP(pcmCruiseSpeed=True))
    self.key = PauseResumeKey(self.CP)
    self.sm = StateMachine()
    self.enable_delay = enable_delay
    self.cc_enabled = False                # carControl.enabled as card sees it
    self.history = []                      # selfdrived's enabled state per iteration
    self.prev = None                       # card's CS_prev
    self.vCruise = float(V_CRUISE_UNSET)   # carState.vCruise as selfdrived sees it
    self.blocked = False

  def frame(self, v_kph=0., buttons=(), available=True, brake=False):
    # card
    visible = self.history[-1 - self.enable_delay] if len(self.history) > self.enable_delay else False
    was, self.cc_enabled = self.cc_enabled, visible
    CS = structs.CarState(vEgo=v_kph * CV.KPH_TO_MS, brakePressed=brake, canValid=True, standstill=v_kph == 0)
    CS.cruiseState.available = available
    CS.buttonEvents = [structs.CarState.ButtonEvent(type=t, pressed=p) for t, p in buttons]
    # opendbc update_button_enable: SET/RES (decel/accel) release enables with the camera's main on
    CS.buttonEnable = available and any(be.type in (SP.accelCruise, SP.decelCruise) and not be.pressed for be in CS.buttonEvents)
    self.key.update(CS, self.cc_enabled)
    cs = car.CarState.new_message(vEgo=CS.vEgo, brakePressed=brake, canValid=True, buttonEnable=CS.buttonEnable, vCruise=self.vCruise,
                                  cruiseState={"available": available},
                                  buttonEvents=[{"type": str(be.type), "pressed": be.pressed} for be in CS.buttonEvents]).as_reader()
    self.helper.update_v_cruise(cs, self.cc_enabled, True)
    if self.cc_enabled and not was:
      self.helper.initialize_v_cruise(self.prev or cs, False, False, True)
    self.vCruise = float(self.helper.v_cruise_kph)
    cs = car.CarState.new_message(**{**cs.to_dict(), "vCruise": self.vCruise}).as_reader()
    # selfdrived
    events = Events()
    if cs.buttonEnable:
      events.add(EventName.buttonEnable)
    if any(be.type == car.CarState.ButtonEvent.Type.cancel and be.pressed for be in cs.buttonEvents):
      events.add(EventName.buttonCancel)
    self.blocked = resume_blocked(self.CP, cs)
    if self.blocked:
      events.add(EventName.resumeBlocked)
    self.sm.update(events)
    self.history.append(self.sm.state in (State.enabled, State.preEnabled, State.overriding))
    self.prev = cs
    return self

  def run(self, n, **kw):
    for _ in range(n):
      self.frame(**kw)
    return self


class TestEnginePipeline:
  def tap(self, d, t, v, frames=3, **kw):
    d.frame(v, [(t, True)], **kw)
    d.run(frames, v_kph=v)
    d.frame(v, [(t, False)], **kw)
    return d

  @pytest.mark.parametrize("button", (SP.decelCruise, SP.accelCruise))
  def test_set_and_up_with_no_speed_engage_at_the_current_speed(self, button):
    # vCruise is unset (255): not blocked, the panda grants controls on the same release
    d = Drive()
    d.run(3, v_kph=60)
    self.tap(d, button, 60)
    assert not d.blocked
    d.run(2, v_kph=60)
    assert d.cc_enabled and d.helper.v_cruise_kph == 60

  def test_pause_with_no_remembered_speed_engages_at_the_current_speed(self):
    d = Drive()
    d.run(3, v_kph=70)
    self.tap(d, SP.cancel, 70)
    assert not d.blocked
    d.run(2, v_kph=70)
    assert d.cc_enabled and d.helper.v_cruise_kph == 70

  def test_set_below_the_minimum_uses_the_stock_minimum(self):
    d = Drive()
    d.run(2, v_kph=10)
    self.tap(d, SP.decelCruise, 10)
    d.run(2, v_kph=10)
    assert d.cc_enabled and d.helper.v_cruise_kph == 30

  def test_without_main_nothing_enables(self):
    d = Drive()
    d.run(2, v_kph=60, available=False)
    self.tap(d, SP.decelCruise, 60, available=False)
    d.run(3, v_kph=60, available=False)
    assert not d.cc_enabled

  def test_other_cars_keep_resume_blocked(self):
    d = Drive(safety_param=7180)
    d.run(2, v_kph=60)
    d.frame(60, [(SP.accelCruise, True)])
    assert d.blocked

  @pytest.mark.parametrize("delay", (0, 1, 3, 8))
  def test_delayed_enable_pause_restores_the_remembered_speed(self, delay):
    d = Drive(enable_delay=delay)
    d.run(2, v_kph=90)
    self.tap(d, SP.decelCruise, 90)
    d.run(delay + 3, v_kph=90)
    assert d.cc_enabled and d.helper.v_cruise_kph == 90
    self.tap(d, SP.cancel, 90)  # pause
    d.run(delay + 3, v_kph=90)
    assert not d.cc_enabled
    self.tap(d, SP.cancel, 50)  # resume while rolling at 50: remembered 90, not the current speed
    d.run(delay + 3, v_kph=50)
    assert d.cc_enabled and d.helper.v_cruise_kph == 90

  @pytest.mark.parametrize("delay", (0, 1, 3, 8))
  @pytest.mark.parametrize("button", (SP.decelCruise, SP.accelCruise))
  def test_delayed_enable_set_up_main_use_the_current_speed(self, delay, button):
    d = Drive(enable_delay=delay)
    d.run(2, v_kph=90)
    self.tap(d, SP.decelCruise, 90)
    d.run(delay + 3, v_kph=90)
    self.tap(d, SP.cancel, 90)
    d.run(delay + 3, v_kph=90)
    assert not d.cc_enabled
    self.tap(d, button, 50)
    d.run(delay + 3, v_kph=50)
    assert d.cc_enabled and d.helper.v_cruise_kph == 50
