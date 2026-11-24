"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The speed-limit follow alerts read the limit (m/s) and the set speed (carState.vCruiseCluster, km/h): both must show in the driver's unit.
"""
from openpilot.cereal import messaging
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.selfdrived.events import speed_limit_follow_alert, speed_limit_school_zone_alert


def _inputs(limit_mph, set_mph):
  lp = messaging.new_message('longitudinalPlanSP')
  lp.longitudinalPlanSP.speedLimit.resolver.speedLimitLast = limit_mph * CV.MPH_TO_MS
  cs = messaging.new_message('carState')
  cs.carState.vCruiseCluster = set_mph * CV.MPH_TO_KPH
  return cs.carState.as_reader(), {'longitudinalPlanSP': lp.longitudinalPlanSP.as_reader()}


def test_follow_alert_shows_the_offset_in_mph():
  cs, sm = _inputs(40, 45)
  assert speed_limit_follow_alert(None, cs, sm, False, 0, None).alert_text_1 == "Following the speed limit +5 mph"


def test_follow_alert_in_kph():
  cs, sm = _inputs(40, 45)
  assert speed_limit_follow_alert(None, cs, sm, True, 0, None).alert_text_1 == "Following the speed limit +8 km/h"


def test_follow_alert_without_a_limit():
  cs, sm = _inputs(0, 45)
  assert speed_limit_follow_alert(None, cs, sm, False, 0, None).alert_text_1 == "Following the speed limit (none known yet)"


def test_school_zone_alert():
  cs, sm = _inputs(25, 45)
  a = speed_limit_school_zone_alert(None, cs, sm, False, 0, None)
  assert (a.alert_text_1, a.alert_text_2) == ("School zone 25 mph", "Press SET- to slow down")
