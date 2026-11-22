from types import SimpleNamespace

from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import EVENTS, ET

EventName = log.OnroadEvent.EventName
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
VisualAlert = car.CarControl.HUDControl.VisualAlert


class TestQuietAlerts(OpenpilotTestCase):
  def test_silent_steer_fault_is_text_only(self):
    alert = EVENTS[EventName.steerTempUnavailableSilent][ET.WARNING]
    assert alert.audible_alert == AudibleAlert.none
    assert alert.visual_alert == VisualAlert.none
    assert alert.alert_text_1 == "Steering Assist Temporarily Unavailable"

  def disengage_alert(self, brake):
    callback = EVENTS[EventName.pedalPressed][ET.USER_DISABLE]
    return callback(None, SimpleNamespace(brakePressed=brake), None, True, 0, None)

  def test_brake_disengage_has_no_chime_but_still_disengages(self):
    assert self.disengage_alert(brake=True).audible_alert == AudibleAlert.none
    assert ET.NO_ENTRY in EVENTS[EventName.pedalPressed]

  def test_accelerator_or_regen_disengage_keeps_the_chime(self):
    assert self.disengage_alert(brake=False).audible_alert == AudibleAlert.disengage
