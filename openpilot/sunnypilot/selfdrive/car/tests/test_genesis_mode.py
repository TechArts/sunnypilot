from types import SimpleNamespace

from opendbc.car import structs
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.genesis_mode import GenesisModeButton, HOLD_TIME, RELEASE_TIME

GearShifter = structs.CarState.GearShifter
HOLD_FRAMES = round(HOLD_TIME / DT_CTRL)
RELEASE_FRAMES = round(RELEASE_TIME / DT_CTRL)
GV70 = SimpleNamespace(carFingerprint="GENESIS_GV70_2ND_GEN")


class FakeParams:
  def __init__(self, enabled=True):
    self.store = {"OpenpilotEnabledToggle": enabled, "OnroadCycleRequested": False, "ControlsReady": True}

  def get_bool(self, k):
    return self.store[k]

  def put_bool(self, k, v, block=False):
    self.store[k] = v

  def remove(self, k):
    self.store.pop(k, None)


def cs(gear=GearShifter.park, v=0.0):
  return SimpleNamespace(gearShifter=gear, vEgo=v)


def hold(btn, frames, state=None, engaged=False):
  return [btn.update(True, state or cs(), engaged) for _ in range(frames)]


def release(btn, frames=RELEASE_FRAMES):
  for _ in range(frames):
    btn.update(False, cs(), False)


def armed(params, CP=GV70):
  btn = GenesisModeButton(CP, params)
  release(btn)
  return btn


class TestGenesisModeButton:
  def setup_method(self):
    self.params = FakeParams()
    self.btn = armed(self.params)

  def test_long_press_in_park_switches_once(self):
    res = hold(self.btn, HOLD_FRAMES + 200)
    assert res.count(True) == 1 and res.index(True) == HOLD_FRAMES - 1
    # ControlsReady removed so pandad cannot re-apply the old CarParams (relay closed) after the onroad cycle
    assert self.params.store == {"OpenpilotEnabledToggle": False, "OnroadCycleRequested": True}

  def test_short_press_does_nothing(self):
    hold(self.btn, HOLD_FRAMES - 2)
    release(self.btn)
    hold(self.btn, HOLD_FRAMES - 2)
    assert self.params.store["OpenpilotEnabledToggle"] and not self.params.store["OnroadCycleRequested"]

  def test_second_hold_switches_back(self):
    hold(self.btn, HOLD_FRAMES)
    release(self.btn)
    hold(self.btn, HOLD_FRAMES)
    assert self.params.store["OpenpilotEnabledToggle"]

  def test_held_through_restart_does_not_switch_back(self):
    """A new card (after the onroad cycle) must not count a button still held from the switch."""
    hold(self.btn, HOLD_FRAMES)
    new = GenesisModeButton(GV70, self.params)
    release(new, 20)  # SWRC_01 not arrived yet / one stale gap: reads released briefly
    assert not any(hold(new, HOLD_FRAMES * 2))
    release(new)
    assert any(hold(new, HOLD_FRAMES))

  def test_not_in_park_moving_or_engaged(self):
    for state, engaged in ((cs(GearShifter.drive), False), (cs(v=0.5), False), (cs(), True)):
      btn = armed(self.params)
      assert not any(hold(btn, HOLD_FRAMES * 2, state, engaged))
    assert self.params.store["OpenpilotEnabledToggle"]

  def test_leaving_park_during_hold_blocks_until_release(self):
    hold(self.btn, HOLD_FRAMES // 2)
    self.btn.update(True, cs(GearShifter.drive), False)
    assert not any(hold(self.btn, HOLD_FRAMES * 2))  # back in Park, same hold
    release(self.btn)
    assert any(hold(self.btn, HOLD_FRAMES))

  def test_other_cars_inactive(self):
    btn = armed(self.params, SimpleNamespace(carFingerprint="HYUNDAI_IONIQ_5"))
    assert not any(hold(btn, HOLD_FRAMES * 2))
