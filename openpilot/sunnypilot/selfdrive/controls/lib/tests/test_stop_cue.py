"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import stop_cue
from openpilot.sunnypilot.selfdrive.controls.lib.stop_cue import (
  OneShotCue, StopCueHelper, stop_sign_not_slowing, brake_now_needed, STOP_SIGN_MIN_SPEED, STOP_SIGN_HOLD_TIME,
  STOP_SIGN_REARM_CLEAR_TIME, BRAKE_NOW_MIN_SPEED, BRAKE_NOW_ACCEL, BRAKE_NOW_HOLD_TIME, BRAKE_NOW_REARM_CLEAR_TIME,
)

V = 12.0
IDX_N = 33


class FakeParams:
  def get_bool(self, key):
    return True


@pytest.fixture
def helper(monkeypatch):
  monkeypatch.setattr(stop_cue, "Params", FakeParams)
  return StopCueHelper(True)


def frames(t):
  return round(t / DT_MDL)


class TestStopSignNotSlowing:
  def test_baseline(self):
    assert stop_sign_not_slowing(True, True, V, 0.0, False)

  def test_each_gate(self):
    assert not stop_sign_not_slowing(False, True, V, 0.0, False)                      # openpilot longitudinal not active
    assert not stop_sign_not_slowing(True, False, V, 0.0, False)                      # no sign
    assert not stop_sign_not_slowing(True, True, STOP_SIGN_MIN_SPEED - 0.1, 0.0, False)  # too slow
    assert not stop_sign_not_slowing(True, True, V, -0.6, False)                      # already slowing
    assert not stop_sign_not_slowing(True, True, V, 0.0, True)                        # braking

  def test_thresholds(self):
    assert STOP_SIGN_MIN_SPEED == pytest.approx(4.5)
    assert STOP_SIGN_HOLD_TIME == pytest.approx(0.5)


class TestStopSignLatch:
  def test_fires_after_hold_time_only(self):
    cue = OneShotCue(STOP_SIGN_HOLD_TIME, STOP_SIGN_REARM_CLEAR_TIME)
    assert not any(cue.step(True, True, V) for _ in range(frames(STOP_SIGN_HOLD_TIME) - 1))
    assert cue.step(True, True, V)

  def test_once_per_sign(self):
    cue = OneShotCue(STOP_SIGN_HOLD_TIME, STOP_SIGN_REARM_CLEAR_TIME)
    assert sum(cue.step(True, True, V) for _ in range(frames(9.0))) == 1

  def test_flag_flicker_does_not_refire(self):
    cue = OneShotCue(STOP_SIGN_HOLD_TIME, STOP_SIGN_REARM_CLEAR_TIME)
    for _ in range(frames(STOP_SIGN_HOLD_TIME)):
      cue.step(True, True, V)
    # the flag drops for 1 s and returns: still the same sign
    assert not any(cue.step(False, False, V) for _ in range(frames(1.0)))
    assert not any(cue.step(True, True, V) for _ in range(frames(2.0)))

  def test_rearms_after_flag_clear(self):
    cue = OneShotCue(STOP_SIGN_HOLD_TIME, STOP_SIGN_REARM_CLEAR_TIME)
    for _ in range(frames(STOP_SIGN_HOLD_TIME)):
      cue.step(True, True, V)
    for _ in range(frames(STOP_SIGN_REARM_CLEAR_TIME)):
      cue.step(False, False, V)
    assert sum(cue.step(True, True, V) for _ in range(frames(STOP_SIGN_HOLD_TIME))) == 1

  def test_rearms_when_car_stops(self):
    cue = OneShotCue(STOP_SIGN_HOLD_TIME, STOP_SIGN_REARM_CLEAR_TIME)
    for _ in range(frames(STOP_SIGN_HOLD_TIME)):
      cue.step(True, True, V)
    cue.step(True, True, 0.0)
    assert not cue.fired

  def test_driver_slowing_resets_hold(self):
    cue = OneShotCue(STOP_SIGN_HOLD_TIME, STOP_SIGN_REARM_CLEAR_TIME)
    for _ in range(5):
      assert not any(cue.step(True, True, V) for _ in range(frames(STOP_SIGN_HOLD_TIME) - 1))
      assert not cue.step(False, True, V)


class TestBrakeNowNeeded:
  def args(self, **kw):
    a = dict(long_active=True, v_ego=BRAKE_NOW_MIN_SPEED + 1.0, model_accel=BRAKE_NOW_ACCEL - 0.5, brake_pressed=False,
             model_should_stop=True, stop_planned=False)
    a.update(kw)
    return a

  def test_baseline(self):
    assert brake_now_needed(**self.args())

  def test_stop_planned_is_enough(self):
    assert brake_now_needed(**self.args(model_should_stop=False, stop_planned=True))

  def test_hard_braking_without_stop_source_is_lead_following(self):
    assert not brake_now_needed(**self.args(model_should_stop=False, stop_planned=False))

  def test_each_gate(self):
    assert not brake_now_needed(**self.args(long_active=False))
    assert not brake_now_needed(**self.args(v_ego=BRAKE_NOW_MIN_SPEED - 0.1))
    assert not brake_now_needed(**self.args(model_accel=BRAKE_NOW_ACCEL + 0.1))
    assert not brake_now_needed(**self.args(brake_pressed=True))

  def test_thresholds(self):
    assert BRAKE_NOW_MIN_SPEED == pytest.approx(25 * CV.MPH_TO_MS)
    assert BRAKE_NOW_ACCEL == -2.5
    assert BRAKE_NOW_HOLD_TIME == pytest.approx(0.3)

  def test_debounce_and_once(self):
    cue = OneShotCue(BRAKE_NOW_HOLD_TIME, BRAKE_NOW_REARM_CLEAR_TIME)
    assert not any(cue.step(True, True, 15.0) for _ in range(frames(BRAKE_NOW_HOLD_TIME) - 1))
    assert cue.step(True, True, 15.0)
    assert not any(cue.step(True, True, 15.0) for _ in range(frames(5.0)))


class TestUpdateReadsRealSchema:
  @staticmethod
  def sm(sign=True, accel=0.0, long_enabled=True, desired=0.0, should_stop=False, v_ego=V, stop_dist=None):
    x = [0.0] * IDX_N if stop_dist is None else [stop_dist * i / (IDX_N - 1) for i in range(IDX_N)]
    v = [v_ego] * IDX_N if stop_dist is None else [v_ego * (1 - i / (IDX_N - 1)) for i in range(IDX_N)]
    model = SimpleNamespace(position=SimpleNamespace(x=x), velocity=SimpleNamespace(x=v),
                            action=SimpleNamespace(desiredAcceleration=desired, shouldStop=should_stop))
    return {'carState': SimpleNamespace(vEgo=v_ego, aEgo=accel, brakePressed=False),
            'carControl': SimpleNamespace(enabled=long_enabled),
            'carStateSP': SimpleNamespace(cameraStopSign=sign),
            'modelV2': model,
            'radarState': SimpleNamespace(leadOne=SimpleNamespace(present=False, dRel=0.0))}

  @staticmethod
  def events():
    ev = SimpleNamespace(names=[])
    ev.add = lambda name: ev.names.append(name)
    return ev

  def test_stop_sign_cue_fires_once(self, helper):
    ev = self.events()
    sm = self.sm()
    for _ in range(frames(STOP_SIGN_HOLD_TIME) * 10):
      helper.update(sm, ev)
    assert len(ev.names) == 1

  def test_no_cue_when_openpilot_longitudinal_off(self, helper):
    ev = self.events()
    sm = self.sm(long_enabled=False)
    for _ in range(frames(2.0)):
      helper.update(sm, ev)
    assert ev.names == []

  def test_brake_now_fires_for_model_stop(self, helper):
    ev = self.events()
    sm = self.sm(sign=False, desired=-3.4, should_stop=True, v_ego=18.0)
    for _ in range(frames(BRAKE_NOW_HOLD_TIME)):
      helper.update(sm, ev)
    assert helper.brake_now_alert
    assert len(ev.names) == 1

  def test_brake_now_not_for_lead_braking(self, helper):
    ev = self.events()
    sm = self.sm(sign=False, desired=-3.4, should_stop=False, v_ego=18.0)
    for _ in range(frames(2.0)):
      helper.update(sm, ev)
    assert ev.names == []
