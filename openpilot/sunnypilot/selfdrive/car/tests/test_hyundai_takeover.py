from openpilot.cereal import custom, log
from opendbc.car import DT_CTRL, structs
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.sunnypilot.selfdrive.car.hyundai_takeover import hyundai_takeover, TAKEOVER_ALERT_WINDOW

EventName = log.OnroadEvent.EventName


def make(brand="hyundai", torque=150, release=100):
  CP = structs.CarParams(brand=brand, carFingerprint="HYUNDAI_IONIQ_5")  # opendbc dataclass: the app side reads the capnp CarParamsSP
  CP_SP = custom.CarParamsSP(hyundaiTakeoverTorque=torque, hyundaiTakeoverReleaseTorque=release)
  return CP, CP_SP


class TestHyundaiTakeover(OpenpilotTestCase):
  def test_wheel_press_is_always_a_takeover(self):
    for brand in ("hyundai", "toyota"):
      CP, CP_SP = make(brand, torque=0)
      assert hyundai_takeover(CP, CP_SP, structs.CarState(steeringPressed=True))

  def test_pull_at_the_filter_torque(self):
    CP, CP_SP = make()
    assert not hyundai_takeover(CP, CP_SP, structs.CarState(steeringTorque=149))
    assert hyundai_takeover(CP, CP_SP, structs.CarState(steeringTorque=150))
    assert hyundai_takeover(CP, CP_SP, structs.CarState(steeringTorque=-200))

  def test_release_torque_for_the_resume_window(self):
    CP, CP_SP = make()
    assert not hyundai_takeover(CP, CP_SP, structs.CarState(steeringTorque=99), release=True)
    assert hyundai_takeover(CP, CP_SP, structs.CarState(steeringTorque=100), release=True)

  def test_filter_off_or_other_brand_ignores_torque(self):
    for CP, CP_SP in (make(torque=0), make(brand="toyota")):
      assert not hyundai_takeover(CP, CP_SP, structs.CarState(steeringTorque=400))
      assert not hyundai_takeover(CP, CP_SP, structs.CarState(steeringTorque=400), release=True)

  def test_user_settings_move_the_thresholds(self):
    CP, CP_SP = make(torque=300, release=250)
    assert not hyundai_takeover(CP, CP_SP, structs.CarState(steeringTorque=250))
    assert hyundai_takeover(CP, CP_SP, structs.CarState(steeringTorque=250), release=True)


class TestSteerFaultAlertWindow(OpenpilotTestCase):
  def run_events(self, torque_before_fault):
    """a temporary steer fault after `torque_before_fault` frames of a hard pull, then a gentle hand: which event is raised"""
    CP, CP_SP = make()
    ce = CarEvents(CP, CP_SP)
    CC = structs.CarControl()
    CS_prev = structs.CarState()
    for _ in range(torque_before_fault):
      ce.update(structs.CarState(steeringTorque=200), CS_prev, CC)
    # let the window run: a fault starts `window` seconds after the pull ended
    return ce

  def fault_event(self, ce, frames_after_pull):
    CC = structs.CarControl()
    CS_prev = structs.CarState()
    for _ in range(frames_after_pull):
      ce.update(structs.CarState(steeringTorque=0), CS_prev, CC)
    return ce.update(structs.CarState(steeringTorque=0, steerFaultTemporary=True), CS_prev, CC)

  def test_fault_within_the_window_after_a_pull_is_silent(self):
    ce = self.run_events(10)
    events = self.fault_event(ce, int(TAKEOVER_ALERT_WINDOW / DT_CTRL) - 10)
    assert events.has(EventName.steerTempUnavailableSilent)
    assert not events.has(EventName.steerTempUnavailable)

  def test_fault_after_the_window_is_the_loud_alert(self):
    ce = self.run_events(10)
    events = self.fault_event(ce, int(TAKEOVER_ALERT_WINDOW / DT_CTRL) + 5)
    assert events.has(EventName.steerTempUnavailable)
