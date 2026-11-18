"""
Genesis (main-button engage, SP safety flag MAIN_BUTTON_ENGAGE): MAIN, SET and up engage openpilot at the current speed, at least 30 kph / 20 mph,
like the stock cruise. Drives the card -> selfdrived -> card pipeline, including the enable that card sees a few iterations late.
"""
from openpilot.cereal import custom, log
from opendbc.car.structs import car
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from openpilot.common.constants import CV
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_INITIAL, V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.selfdrived.events import Events, EventName
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.selfdrive.car.main_button_engage import resume_blocked

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
State = log.SelfdriveState.OpenpilotState

MAIN_BUTTON_ENGAGE = int(HyundaiSafetyFlagsSP.MAIN_BUTTON_ENGAGE)
MIN_MPH_KPH = int(round(20 * CV.MPH_TO_KPH))
ENGAGE_BUTTONS = (ButtonType.mainCruise, ButtonType.setCruise, ButtonType.decelCruise, ButtonType.accelCruise)


def make_cp(brand="hyundai", sp_param=MAIN_BUTTON_ENGAGE):
  return car.CarParams(pcmCruise=False, brand=brand), custom.CarParamsSP(pcmCruiseSpeed=True, safetyParam=int(sp_param))


def make_helper(brand="hyundai", sp_param=MAIN_BUTTON_ENGAGE):
  return VCruiseHelper(*make_cp(brand, sp_param))


def cs(v_ego_kph=0., events=(), available=True):
  return car.CarState(vEgo=v_ego_kph * CV.KPH_TO_MS, cruiseState={"available": available},
                      buttonEvents=[ButtonEvent(type=t, pressed=p) for t, p in events])


def engage(helper, v_ego_kph, button, is_metric=True):
  """the carState of the frame selfdrived enables on goes to initialize_v_cruise"""
  helper.initialize_v_cruise(cs(v_ego_kph, [(button, False)]), 0, is_metric)


class TestMainButtonEngage(OpenpilotTestCase):
  def test_set_speed_is_current_speed(self):
    for button in ENGAGE_BUTTONS:
      for kph in (35, 50.4, 88.6, 120):
        helper = make_helper()
        engage(helper, kph, button)
        assert helper.v_cruise_kph == int(round(kph)), (button, kph)
        assert helper.v_cruise_cluster_kph == helper.v_cruise_kph

  def test_below_minimum_uses_stock_minimum(self):
    # the stock cruise sets at least 20 mph, not openpilot's 40 kph SET floor
    for button in ENGAGE_BUTTONS:
      for v_ego in (0., 5., 20.):
        helper = make_helper()
        engage(helper, v_ego, button, is_metric=True)
        assert helper.v_cruise_kph == 30, (button, v_ego)
        helper = make_helper()
        engage(helper, v_ego, button, is_metric=False)
        assert helper.v_cruise_kph == MIN_MPH_KPH, (button, v_ego)

  def test_initial_cruise_speed_param_does_not_apply(self):
    helper = make_helper()
    helper.initialize_v_cruise(cs(60, [(ButtonType.mainCruise, False)]), 100, True)
    assert helper.v_cruise_kph == 60

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

  def test_gate(self):
    # without the safety flag (or on another brand): openpilot's own rules, SET floor and accel resumes
    for brand, sp_param in (("hyundai", 0), ("hyundai", int(HyundaiSafetyFlagsSP.CCNC)), ("toyota", MAIN_BUTTON_ENGAGE)):
      helper = make_helper(brand, sp_param)
      helper.initialize_v_cruise(cs(10, [(ButtonType.mainCruise, False)]), 0)
      assert helper.v_cruise_kph == V_CRUISE_INITIAL, (brand, sp_param)
      helper.v_cruise_kph = 90
      helper.v_cruise_kph_last = 90
      engage(helper, 60, ButtonType.accelCruise)
      assert helper.v_cruise_kph == 90, (brand, sp_param)


class Drive:
  """card -> selfdrived -> card for the Genesis buttons: buttonEnable / buttonCancel / resumeBlocked events, the state machine, then the
  carControl.enabled that card sees `enable_delay` iterations after selfdrived's state changed; card initializes the set speed on it."""

  def __init__(self, enable_delay=0, sp_param=MAIN_BUTTON_ENGAGE):
    self.CP, self.CP_SP = make_cp(sp_param=sp_param)
    self.helper = VCruiseHelper(self.CP, self.CP_SP)
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
    # opendbc update_button_enable: SET/RES (decel/accel) release enables with the camera's main on
    button_enable = available and any(t in (ButtonType.accelCruise, ButtonType.decelCruise) and not p for t, p in buttons)
    cs = car.CarState(vEgo=v_kph * CV.KPH_TO_MS, brakePressed=brake, canValid=True, buttonEnable=button_enable, vCruise=self.vCruise,
                      cruiseState={"available": available}, buttonEvents=[ButtonEvent(type=t, pressed=p) for t, p in buttons])
    self.helper.update_v_cruise(cs, self.cc_enabled, True)
    if self.cc_enabled and not was:
      self.helper.initialize_v_cruise(self.prev or cs, 0, True)
    self.vCruise = float(self.helper.v_cruise_kph)
    cs.vCruise = self.vCruise
    # selfdrived
    events = Events()
    if cs.buttonEnable:
      events.add(EventName.buttonEnable)
    if any(be.type == ButtonType.cancel and be.pressed for be in cs.buttonEvents):
      events.add(EventName.buttonCancel)
    self.blocked = resume_blocked(self.CP, self.CP_SP, cs)
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


def tap(d, t, v, frames=3, **kw):
  d.frame(v, [(t, True)], **kw)
  d.run(frames, v_kph=v)
  d.frame(v, [(t, False)], **kw)
  return d


class TestEnginePipeline(OpenpilotTestCase):
  def test_set_and_up_with_no_speed_engage_at_the_current_speed(self):
    for button in (ButtonType.decelCruise, ButtonType.accelCruise):
      # vCruise is unset (255): not blocked, the panda grants controls on the same release
      d = Drive()
      d.run(3, v_kph=60)
      tap(d, button, 60)
      assert not d.blocked
      d.run(2, v_kph=60)
      assert d.cc_enabled and d.helper.v_cruise_kph == 60

  def test_set_below_the_minimum_uses_the_stock_minimum(self):
    d = Drive()
    d.run(2, v_kph=10)
    tap(d, ButtonType.decelCruise, 10)
    d.run(2, v_kph=10)
    assert d.cc_enabled and d.helper.v_cruise_kph == 30

  def test_without_main_nothing_enables(self):
    d = Drive()
    d.run(2, v_kph=60, available=False)
    tap(d, ButtonType.decelCruise, 60, available=False)
    d.run(3, v_kph=60, available=False)
    assert not d.cc_enabled

  def test_other_cars_keep_resume_blocked(self):
    d = Drive(sp_param=0)
    d.run(2, v_kph=60)
    d.frame(60, [(ButtonType.accelCruise, True)])
    assert d.blocked

  def test_delayed_enable_set_and_up_use_the_current_speed(self):
    for delay in (0, 1, 3, 8):
      for button in (ButtonType.decelCruise, ButtonType.accelCruise):
        d = Drive(enable_delay=delay)
        d.run(2, v_kph=50)
        tap(d, button, 50)
        d.run(delay + 3, v_kph=50)
        assert d.cc_enabled and d.helper.v_cruise_kph == 50, (delay, button)
