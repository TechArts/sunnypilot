"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from openpilot.common.parameterized import parameterized
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper, GREEN_LIGHT_X_THRESHOLD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

GearShifter = structs.CarState.GearShifter

# Hyundai's drivable gears besides drive are sport and manumatic; low raises wrongGear
FORWARD_GEARS = [(GearShifter.drive,), (GearShifter.sport,), (GearShifter.manumatic,)]
NOT_FORWARD_GEARS = [(GearShifter.park,), (GearShifter.reverse,), (GearShifter.neutral,), (GearShifter.unknown,), (GearShifter.low,)]


def make_sm(gear, v_ego=0.0, path_end=5.0, lead_d=None):
  cs = SimpleNamespace(standstill=v_ego < 0.1, vEgo=v_ego, gearShifter=gear, gasPressed=False)
  lead = SimpleNamespace(present=lead_d is not None, dRel=lead_d if lead_d is not None else 0.0)
  return {
    'carState': cs,
    'carControl': SimpleNamespace(enabled=False),
    'modelV2': SimpleNamespace(position=SimpleNamespace(x=[0.0, path_end])),
    'radarState': SimpleNamespace(leadOne=lead),
  }


class TestE2EAlertsHelper(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    params = Params()
    params.put_bool("GreenLightAlert", True)
    params.put_bool("LeadDepartAlert", True)
    CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_IONIQ_5)
    self.helper = E2EAlertsHelper(CP)

  def chimes(self, gear, seconds, **kw):
    events, count = EventsSP(), 0
    for _ in range(round(seconds / DT_MDL)):
      self.helper.update(make_sm(gear, **kw), events)
      count += len(events.names)
      events.clear()
    return count

  def stop(self, gear, lead_d=None):
    # the alerts are allowed 2 s after the car stops; a lead departure arms after 1 s more with the lead close
    self.chimes(gear, 1.0, v_ego=10.0, lead_d=lead_d)
    assert self.chimes(gear, 4.0, lead_d=lead_d) == 0

  @parameterized.expand(FORWARD_GEARS)
  def test_green_light_chimes_in_a_forward_gear(self, gear):
    self.stop(gear)
    assert self.chimes(gear, 2.0, path_end=GREEN_LIGHT_X_THRESHOLD + 20.0) == 1

  @parameterized.expand(FORWARD_GEARS)
  def test_lead_depart_chimes_in_a_forward_gear(self, gear):
    self.stop(gear, lead_d=4.0)
    assert self.chimes(gear, 2.0, lead_d=6.0) == 1  # the lead pulls away (still inside the 8 m arming range)

  @parameterized.expand(NOT_FORWARD_GEARS)
  def test_no_chimes_when_not_in_a_forward_gear(self, gear):
    self.stop(gear)
    assert self.chimes(gear, 2.0, path_end=GREEN_LIGHT_X_THRESHOLD + 20.0) == 0

    self.stop(gear, lead_d=4.0)
    assert self.chimes(gear, 2.0, lead_d=6.0) == 0
