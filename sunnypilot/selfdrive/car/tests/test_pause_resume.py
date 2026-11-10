"""Genesis pause key: a pause/resume toggle like the stock cruise. Runs with the opendbc submodule that has LONG_MAIN_BUTTON_ENGAGE."""
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from openpilot.sunnypilot.selfdrive.car.pause_resume import PauseResumeKey

ButtonType = structs.CarState.ButtonEvent.Type


def make_key(safety_param=7180 | HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE, brand="hyundai"):
  CP = structs.CarParams(brand=brand)
  CP.safetyConfigs = [structs.CarParams.SafetyConfig(safetyParam=int(safety_param))]
  return PauseResumeKey(CP)


def step(key, pressed, engaged, available=True, brake=False):
  CS = structs.CarState(brakePressed=brake)
  CS.cruiseState.available = available
  CS.buttonEvents = [structs.CarState.ButtonEvent(type=ButtonType.cancel, pressed=pressed)]
  key.update(CS, engaged)
  return CS.buttonEvents[0].type, CS.buttonEnable


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
    assert step(key, False, engaged=False, brake=True) == (ButtonType.resumeCruise, False)

  def test_other_cars_keep_cancel(self):
    for key in (make_key(7180), make_key(brand="toyota")):
      assert step(key, True, engaged=False) == (ButtonType.cancel, False)
      assert step(key, False, engaged=False) == (ButtonType.cancel, False)
