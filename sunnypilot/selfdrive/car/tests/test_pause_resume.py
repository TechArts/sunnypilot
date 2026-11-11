"""Genesis pause key: a pause/resume toggle like the stock cruise. Runs with the opendbc submodule that has LONG_MAIN_BUTTON_ENGAGE."""
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from openpilot.sunnypilot.selfdrive.car.pause_resume import PauseResumeKey

ButtonType = structs.CarState.ButtonEvent.Type


def make_key(safety_param=7180 | HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE, brand="hyundai"):
  CP = structs.CarParams(brand=brand)
  CP.safetyConfigs = [structs.CarParams.SafetyConfig(safetyParam=int(safety_param))]
  return PauseResumeKey(CP)


def step(key, pressed, engaged, available=True, brake=False, can_valid=True, fault=False):
  CS = structs.CarState(brakePressed=brake, canValid=can_valid)
  CS.cruiseState.available = available
  CS.buttonEvents = [structs.CarState.ButtonEvent(type=ButtonType.cancel, pressed=pressed)]
  key.update(CS, engaged, fault)
  return CS.buttonEvents[0].type, CS.buttonEnable


def frame(key, engaged=False, available=True, brake=False, can_valid=True, fault=False):
  """an ordinary carState frame while the key is held: no button event"""
  CS = structs.CarState(brakePressed=brake, canValid=can_valid)
  CS.cruiseState.available = available
  key.update(CS, engaged, fault)


class TestPauseResumeKey:
  def test_resumes_from_standby_on_release(self):
    key = make_key()
    assert step(key, True, engaged=False) == (ButtonType.resumeCruise, False)
    assert step(key, False, engaged=False) == (ButtonType.resumeCruise, True)

  def test_pauses_when_engaged_and_the_release_does_not_resume(self):
    key = make_key()
    assert step(key, True, engaged=True) == (ButtonType.cancel, False)
    assert step(key, False, engaged=False) == (ButtonType.cancel, False)  # decided at the press

  def test_main_off_or_brake_does_not_resume(self):
    key = make_key()
    assert step(key, True, engaged=False, available=False) == (ButtonType.cancel, False)
    assert step(key, False, engaged=False, available=False) == (ButtonType.cancel, False)
    step(key, True, engaged=False)
    assert step(key, False, engaged=False, brake=True) == (ButtonType.cancel, False)  # held at the release

  def test_other_cars_keep_cancel(self):
    for key in (make_key(7180), make_key(brand="toyota")):
      assert step(key, True, engaged=False) == (ButtonType.cancel, False)
      assert step(key, False, engaged=False) == (ButtonType.cancel, False)

  def _hold(self, key, interrupt):
    assert step(key, True, engaged=False) == (ButtonType.resumeCruise, False)
    interrupt()
    return step(key, False, engaged=False)

  def test_brake_pressed_and_released_during_the_hold_cancels_the_resume(self):
    key = make_key()
    assert self._hold(key, lambda: (frame(key, brake=True), frame(key, brake=False))) == (ButtonType.cancel, False)
    assert step(key, True, engaged=False) == (ButtonType.resumeCruise, False)  # a fresh press starts clean
    assert step(key, False, engaged=False) == (ButtonType.resumeCruise, True)

  def test_main_off_or_off_and_on_during_the_hold_cancels_the_resume(self):
    for seq in ((False,), (False, True)):
      key = make_key()
      def interrupt(key=key, seq=seq):
        for available in seq:
          frame(key, available=available)
      assert self._hold(key, interrupt) == (ButtonType.cancel, False)

  def test_fault_during_the_hold_cancels_the_resume(self):
    for kw in ({"fault": True}, {"can_valid": False}):
      key = make_key()
      assert self._hold(key, lambda key=key, kw=kw: frame(key, **kw)) == (ButtonType.cancel, False)

  def test_uninterrupted_hold_still_resumes(self):
    key = make_key()
    assert self._hold(key, lambda: frame(key)) == (ButtonType.resumeCruise, True)
