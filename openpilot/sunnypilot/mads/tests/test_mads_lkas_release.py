from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from opendbc.car import structs
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem, LKAS_LONG_PRESS_FRAMES
from openpilot.sunnypilot.mads.tests.test_mads_steering_mode import make_car_state, make_mads
from openpilot.common.test import OpenpilotTestCase

ButtonType = structs.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName


def build(mocker, fingerprint=CAR.GENESIS_GV70_2ND_GEN, ccnc=True):
  _, sd = make_mads(mocker, MadsSteeringModeOnBrake.REMAIN_ACTIVE)
  sd.CP.carFingerprint = fingerprint
  if ccnc:
    sd.CP_SP.flags = int(HyundaiFlagsSP.CCNC)
  mads = ModularAssistiveDrivingSystem(sd)
  mads.enabled_toggle = True
  return mads, sd


def lkas(pressed):
  cs = make_car_state()
  be = structs.CarState.ButtonEvent()
  be.type = ButtonType.lkas
  be.pressed = pressed
  cs.buttonEvents = [be]
  return cs


def step(mads, sd, cs):
  mads.update(cs)
  toggled = sd.events_sp.contains(EventNameSP.lkasEnable) or sd.events_sp.contains(EventNameSP.lkasDisable)
  sd.CS_prev = cs
  sd.events.clear()
  sd.events_sp.clear()
  return toggled


def hold(mads, sd, frames):
  """Press LFA, release it `frames` cycles later; return the cycles (0 = press-down) whose events toggled MADS."""
  return [i for i, cs in enumerate([lkas(True)] + [make_car_state()] * (frames - 1) + [lkas(False)]) if step(mads, sd, cs)]


class TestLkasRelease(OpenpilotTestCase):
  @parameterized.expand([(10,), (85,), (188,)])  # 0.1 s, 0.85 s and 1.88 s holds: the GV70 camera toggled its LFA on release
  def test_gv70_short_press_toggles_on_release(self, mocker, frames):
    mads, sd = build(mocker)
    assert hold(mads, sd, frames) == [frames]

  @parameterized.expand([(210,), (321,)])  # 2.10 s and 3.21 s holds: the camera ignored them
  def test_gv70_long_press_is_ignored(self, mocker, frames):
    mads, sd = build(mocker)
    assert hold(mads, sd, frames) == []

  def test_gv70_boundary(self, mocker):
    mads, sd = build(mocker)
    assert hold(mads, sd, LKAS_LONG_PRESS_FRAMES - 1) == [LKAS_LONG_PRESS_FRAMES - 1]
    assert hold(mads, sd, LKAS_LONG_PRESS_FRAMES) == []

  def test_gv70_long_press_then_short_press_still_toggles(self, mocker):
    mads, sd = build(mocker)
    assert hold(mads, sd, 240) == []
    assert hold(mads, sd, 12) == [12]

  def test_gv70_release_without_press_is_ignored(self, mocker):
    mads, sd = build(mocker)
    assert not step(mads, sd, lkas(False))

  def test_gv70_second_press_restarts_the_timer(self, mocker):
    # a missing release: the next press-down restarts the hold, so a stale start cannot turn a short press into a long one
    mads, sd = build(mocker)
    assert not step(mads, sd, lkas(True))
    for _ in range(300):
      step(mads, sd, make_car_state())
    assert hold(mads, sd, 15) == [15]

  @parameterized.expand([(10,), (240,)])
  def test_other_ccnc_cars_keep_press_down(self, mocker, frames):
    mads, sd = build(mocker, fingerprint=CAR.HYUNDAI_IONIQ_5, ccnc=True)
    assert hold(mads, sd, frames) == [0]
