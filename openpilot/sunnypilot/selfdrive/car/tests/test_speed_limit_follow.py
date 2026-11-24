"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Speed-limit follow driven like card drives it, with the real VCruiseHelper (its button steps, long-press rounding and gas clip) under it.
"""
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import VCruiseHelper, CRUISE_LONG_PRESS
from openpilot.sunnypilot.selfdrive.car.speed_limit_follow import SpeedLimitFollow, MAIN_LONG_FRAMES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

ButtonType = structs.CarState.ButtonEvent.Type


class FakeParams:
  def __init__(self, **extra):
    self.d = {"SpeedLimitFollowOffsetStreet": 5, "SpeedLimitFollowOffsetFreeway": 10, "SpeedLimitMode": Mode.warning, **extra}

  def get(self, k, return_default=False):
    return self.d[k]

  def put(self, k, v, block=False):
    self.d[k] = v


class Car:
  """One card frame per step: VCruiseHelper.update_v_cruise (+ initialize_v_cruise on the enable edge), then the follow mode."""
  def __init__(self, **params):
    CP = structs.CarParams(brand="hyundai", pcmCruise=False)
    CP_SP = structs.CarParamsSP(safetyParam=int(HyundaiSafetyFlagsSP.MAIN_BUTTON_ENGAGE))
    self.params = FakeParams(**params)
    self.f = SpeedLimitFollow(CP, CP_SP, self.params)
    self.vch = VCruiseHelper(structs.CarParams(pcmCruise=False), custom.CarParamsSP())
    self.limit_mph, self.school = 40, False
    self.available, self.enabled, self.v_mph, self.gas = False, False, 38, False
    self.cs_prev = structs.CarState()
    self.enabled_prev = False

  def step(self, events=(), n=1):
    for i in range(n):
      cs = structs.CarState(buttonEvents=[structs.CarState.ButtonEvent(type=t, pressed=p) for t, p in (events if i == 0 else ())],
                            vEgo=self.v_mph * CV.MPH_TO_MS, gasPressed=self.gas)
      cs.cruiseState.available = self.available
      self.vch.update_v_cruise(cs, self.enabled, False)
      if self.enabled and not self.enabled_prev:
        self.vch.initialize_v_cruise(self.cs_prev, 40, False)
      r = SimpleNamespace(speedLimitLast=self.limit_mph * CV.MPH_TO_MS, speedLimitLastValid=self.limit_mph > 0, schoolZone=self.school)
      self.f.update(cs, self.enabled, r, self.vch, False)
      self.cs_prev, self.enabled_prev = cs, self.enabled

  def set_mph(self):
    return round(self.vch.v_cruise_kph * CV.KPH_TO_MPH)

  def main_press(self, frames):
    """MAIN from cruise off: the camera's main rises and openpilot engages 5 frames into the press."""
    self.step([(ButtonType.mainCruise, True)])
    self.available = True
    self.step(n=4)
    self.enabled = True
    self.step(n=max(0, frames - 5))
    self.step([(ButtonType.mainCruise, False)])

  def tap(self, button, n=1):
    for _ in range(n):
      self.step([(button, True)])
      self.step([(button, False)])

  def hold(self, button, frames):
    self.step([(button, True)])
    self.step(n=frames)
    self.step([(button, False)])


class TestSpeedLimitFollow(OpenpilotTestCase):
  def test_short_main_is_a_plain_cruise(self):
    c = Car()
    c.main_press(12)
    c.step(n=100)
    assert not c.f.follow and c.set_mph() == 38

  def test_long_main_follows_the_limit_plus_the_street_offset(self):
    c = Car()
    c.main_press(MAIN_LONG_FRAMES + 5)
    assert c.f.follow and c.set_mph() == 45

  def test_freeway_offset_and_limit_changes(self):
    c = Car()
    c.main_press(MAIN_LONG_FRAMES + 5)
    c.limit_mph = 65
    c.step()
    assert c.set_mph() == 75
    c.limit_mph = 55
    c.step()
    assert c.set_mph() == 65

  def test_res_plus_tap_changes_and_keeps_the_offset(self):
    c = Car()
    c.main_press(MAIN_LONG_FRAMES + 5)
    c.tap(ButtonType.accelCruise, 3)  # stock 1 mph per tap
    assert c.set_mph() == 48 and c.params.d["SpeedLimitFollowOffsetStreet"] == 8
    c.limit_mph = 35
    c.step()
    assert c.set_mph() == 43

  def test_long_press_steps_whole_units_both_ways(self):
    # review: off the 1.6 km/h grid, a long SET- rounded down by a fraction of a mph and never lowered the speed
    c = Car()
    c.main_press(MAIN_LONG_FRAMES + 5)
    c.hold(ButtonType.decelCruise, CRUISE_LONG_PRESS + 1)
    assert c.set_mph() == 40 and c.params.d["SpeedLimitFollowOffsetStreet"] == 0
    c.hold(ButtonType.accelCruise, CRUISE_LONG_PRESS + 1)
    assert c.set_mph() == 45 and c.params.d["SpeedLimitFollowOffsetStreet"] == 5

  def test_set_minus_on_the_gas_does_not_raise_the_offset(self):
    # review: VCruiseHelper clips SET- on the gas up to vEgo; that is no offset change
    c = Car()
    c.main_press(MAIN_LONG_FRAMES + 5)
    c.limit_mph = 65
    c.step()
    c.gas, c.v_mph = True, 80
    c.tap(ButtonType.decelCruise)
    c.gas = False
    c.step()
    assert c.params.d["SpeedLimitFollowOffsetFreeway"] == 10 and c.set_mph() == 75

  def test_speed_limit_assist_mode_disables_it(self):
    # review: SLA writes the set speed too; follow would learn those writes as offset changes
    c = Car(SpeedLimitMode=Mode.assist)
    c.main_press(MAIN_LONG_FRAMES + 5)
    assert not c.f.follow

  def test_no_limit_keeps_the_set_speed(self):
    c = Car()
    c.limit_mph = 0
    c.main_press(MAIN_LONG_FRAMES + 5)
    c.step(n=10)
    assert c.f.follow and c.set_mph() == 38

  def test_school_zone_waits_for_set_minus(self):
    c = Car()
    c.main_press(MAIN_LONG_FRAMES + 5)
    c.limit_mph, c.school = 25, True
    c.step(n=50)
    assert c.f.school_pending and c.set_mph() == 45
    c.tap(ButtonType.decelCruise)
    assert not c.f.school_pending and c.set_mph() == 25
    assert c.params.d["SpeedLimitFollowOffsetStreet"] == 5  # the acceptance is not an offset change
    c.limit_mph, c.school = 40, False
    c.step()
    assert c.set_mph() == 45

  def test_resume_follows_again_set_minus_does_not(self):
    c = Car()
    c.main_press(MAIN_LONG_FRAMES + 5)
    c.enabled = False  # brake
    c.step(n=10)
    c.step([(ButtonType.accelCruise, True)])
    c.step([(ButtonType.accelCruise, False)])
    c.enabled = True
    c.step()
    assert c.f.follow and c.set_mph() == 45
    c.enabled = False
    c.step(n=10)
    c.step([(ButtonType.decelCruise, True)])
    c.step([(ButtonType.decelCruise, False)])
    c.enabled = True
    c.step()
    assert not c.f.follow and c.set_mph() == 38

  def test_main_while_on_ends_it(self):
    c = Car()
    c.main_press(MAIN_LONG_FRAMES + 5)
    c.step([(ButtonType.mainCruise, True)])
    c.available, c.enabled = False, False
    c.step(n=MAIN_LONG_FRAMES + 5)
    c.step([(ButtonType.mainCruise, False)])
    assert not c.f.follow and not c.f.resume_follow

  def test_other_cars_never_follow(self):
    c = Car()
    c.f.available = False
    c.main_press(MAIN_LONG_FRAMES + 5)
    assert not c.f.follow and c.set_mph() == 38
