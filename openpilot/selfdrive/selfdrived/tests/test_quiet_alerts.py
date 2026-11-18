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

  def test_brake_disengage_has_no_chime_but_still_disengages(self):
    events = EVENTS[EventName.pedalPressed]
    assert events[ET.USER_DISABLE].audible_alert == AudibleAlert.none
    assert ET.NO_ENTRY in events
