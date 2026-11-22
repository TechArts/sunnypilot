"""Genesis pause key: a pause/resume toggle like the stock cruise (SP safety flag MAIN_BUTTON_ENGAGE)."""
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.car.pause_resume import PauseResumeKey, panda_fault

ButtonType = structs.CarState.ButtonEvent.Type

MAIN_BUTTON_ENGAGE = int(HyundaiSafetyFlagsSP.MAIN_BUTTON_ENGAGE)


def make_key(sp_param=MAIN_BUTTON_ENGAGE, brand="hyundai"):
  return PauseResumeKey(structs.CarParams(brand=brand), structs.CarParamsSP(safetyParam=int(sp_param)))


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


class TestPauseResumeKey(OpenpilotTestCase):
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
    for key in (make_key(0), make_key(brand="toyota")):
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
    key = make_key()
    assert self._hold(key, lambda: frame(key, fault=True)) == (ButtonType.cancel, False)

  def test_can_invalid_or_timeout_during_the_hold_does_not_diverge_from_the_panda(self):
    # the panda only drops the request on brake, main change, rx checks invalid and reset; a non-rx-checked message timing out is none of them
    key = make_key()
    assert self._hold(key, lambda: frame(key, can_valid=False)) == (ButtonType.resumeCruise, True)
    key = make_key()
    assert step(key, True, engaged=False) == (ButtonType.resumeCruise, False)
    CS = structs.CarState(canValid=False, canTimeout=True)
    CS.cruiseState.available = True
    key.update(CS, False)
    assert step(key, False, engaged=False, can_valid=False) == (ButtonType.resumeCruise, True)

  def test_uninterrupted_hold_still_resumes(self):
    key = make_key()
    assert self._hold(key, lambda: frame(key)) == (ButtonType.resumeCruise, True)

  def test_card_panda_fault_mapping_matches_the_panda_contract(self):
    def ps(**kw):
      return SimpleNamespace(**{"heartbeatLost": False, "safetyRxChecksInvalid": False, "heartbeatEngagedMismatches": 0, **kw})
    assert not panda_fault([ps()])
    assert not panda_fault([])
    assert panda_fault([ps(safetyRxChecksInvalid=True)])
    assert panda_fault([ps(), ps(heartbeatLost=True)])
    # a heartbeat engaged-mismatch (longitudinal or MADS) is not a fault the panda gate uses, so it must not disarm in Python either
    assert not panda_fault([ps(heartbeatEngagedMismatches=2)])

  def test_fault_from_the_card_mapping_cancels_the_resume(self):
    key = make_key()
    fault = panda_fault([SimpleNamespace(heartbeatLost=False, safetyRxChecksInvalid=True)])
    assert self._hold(key, lambda: frame(key, fault=fault)) == (ButtonType.cancel, False)
