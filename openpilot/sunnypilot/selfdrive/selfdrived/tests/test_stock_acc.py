from types import SimpleNamespace

from opendbc.car import DT_CTRL
from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.selfdrived.events import stock_acc_alert
from openpilot.sunnypilot.selfdrive.selfdrived.stock_acc import StockAccMonitor, STOCK_ACC_DELAY

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type

DELAY_FRAMES = int(STOCK_ACC_DELAY / DT_CTRL)


def run(monitor, frames, stock=True, enabled=False, main=True, buttons=()):
  out = False
  for _ in range(frames):
    out = monitor.update(stock, enabled, main, buttons)
  return out


class TestStockAccMonitor(OpenpilotTestCase):
  def test_reports_after_the_delay_only(self):
    m = StockAccMonitor()
    assert not run(m, DELAY_FRAMES)
    assert run(m, 1)

  def test_enable_from_the_same_press_does_not_report(self):
    m = StockAccMonitor()
    assert not run(m, 5)
    assert not run(m, 1, enabled=True)
    assert not run(m, DELAY_FRAMES + 5, stock=False, enabled=True)

  def test_not_engaged_resets_the_counter(self):
    m = StockAccMonitor()
    run(m, DELAY_FRAMES)
    assert not run(m, 1, stock=False)
    assert not run(m, DELAY_FRAMES)

  def test_pause_key_hold_is_not_reported_until_release(self):
    m = StockAccMonitor()
    assert not run(m, 1, buttons=[ButtonEvent(type=ButtonType.resumeCruise, pressed=True)])
    assert not run(m, 3 * DELAY_FRAMES)
    assert not run(m, 1, buttons=[ButtonEvent(type=ButtonType.resumeCruise, pressed=False)])
    assert run(m, DELAY_FRAMES + 1)

  def test_lost_release_frame_does_not_mute_the_alert_for_good(self):
    m = StockAccMonitor()
    run(m, 1, buttons=[ButtonEvent(type=ButtonType.resumeCruise, pressed=True)])
    run(m, 1, main=False)  # main off clears the hold
    assert run(m, DELAY_FRAMES + 1)

  def test_enabled_clears_the_hold(self):
    m = StockAccMonitor()
    run(m, 1, buttons=[ButtonEvent(type=ButtonType.resumeCruise, pressed=True)])
    run(m, 1, enabled=True)
    assert run(m, DELAY_FRAMES + 1)


class TestStockAccAlert(OpenpilotTestCase):
  def alert(self, controls_allowed):
    sm = {'pandaStates': [SimpleNamespace(controlsAllowed=controls_allowed)]}
    return stock_acc_alert(None, None, sm, True, 0, None)

  def test_locked_out_camera_says_cruise_blocked(self):
    assert self.alert(False).alert_text_1 == "Cruise blocked"

  def test_camera_in_control_says_so(self):
    assert self.alert(True).alert_text_1 == "Car's Adaptive Cruise in control"
