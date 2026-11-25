"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import red_light_alert
from openpilot.sunnypilot.selfdrive.controls.lib.red_light_alert import (
  RedLightAlertHelper, RedLightStates, model_plans_stop, red_ahead_not_slowing, HOLD_TIME, REARM_CLEAR_TIME, MIN_SPEED,
)

IDX_N = 33
V_CRUISING = 20.0
FRAMES_HOLD = round(HOLD_TIME / DT_MDL)
FRAMES_REARM = round(REARM_CLEAR_TIME / DT_MDL)


class FakeParams:
  def get_bool(self, key):
    return True


@pytest.fixture
def helper(monkeypatch):
  monkeypatch.setattr(red_light_alert, "Params", FakeParams)
  return RedLightAlertHelper()


def path(stop_dist=None, v_end=0.0):
  """Model path: cruising, or ending at v_end after stop_dist metres."""
  if stop_dist is None:
    return [V_CRUISING * i for i in range(IDX_N)], [V_CRUISING] * IDX_N
  return [stop_dist * i / (IDX_N - 1) for i in range(IDX_N)], [V_CRUISING * (1 - i / (IDX_N - 1)) + v_end for i in range(IDX_N)]


def run(helper, condition, frames, v_ego=V_CRUISING):
  return [helper.step(condition, v_ego) for _ in range(frames)]


class TestModelPlansStop:
  def test_stop_ahead(self):
    x, v = path(stop_dist=60.0)
    assert model_plans_stop(x, v, V_CRUISING, False, 0.0)

  def test_cruising_model(self):
    x, v = path()
    assert not model_plans_stop(x, v, V_CRUISING, False, 0.0)

  def test_stop_too_far(self):
    x, v = path(stop_dist=150.0)
    assert not model_plans_stop(x, v, V_CRUISING, False, 0.0)

  def test_model_not_stopping_fully(self):
    x, v = path(stop_dist=60.0, v_end=6.0)
    assert not model_plans_stop(x, v, V_CRUISING, False, 0.0)

  def test_lead_explains_stop(self):
    x, v = path(stop_dist=60.0)
    assert not model_plans_stop(x, v, V_CRUISING, True, 55.0)
    assert model_plans_stop(x, v, V_CRUISING, True, 120.0)

  def test_empty_model(self):
    assert not model_plans_stop([], [], V_CRUISING, False, 0.0)


class TestRedAheadNotSlowing:
  def test_baseline(self):
    assert red_ahead_not_slowing(V_CRUISING, 0.0, False, True)

  def test_each_gate(self):
    assert not red_ahead_not_slowing(MIN_SPEED - 0.1, 0.0, False, True)  # too slow
    assert not red_ahead_not_slowing(V_CRUISING, -0.6, False, True)      # already decelerating
    assert not red_ahead_not_slowing(V_CRUISING, 0.0, True, True)        # braking
    assert not red_ahead_not_slowing(V_CRUISING, 0.0, False, False)      # no stop planned

  def test_fires_at_12_mph(self):
    # 10 mph threshold: a red light approached at 12 mph without slowing is cued (was silent under 15 mph)
    assert red_ahead_not_slowing(12 * CV.MPH_TO_MS, 0.0, False, True)


class TestRedLightDebounce:
  def test_fires_after_hold_time_only(self, helper):
    assert not any(run(helper, True, FRAMES_HOLD - 1))
    assert helper.step(True, V_CRUISING)

  def test_flicker_resets_hold_timer(self, helper):
    for _ in range(5):
      assert not any(run(helper, True, FRAMES_HOLD - 1))
      assert not helper.step(False, V_CRUISING)

  def test_one_alert_per_stop_event(self, helper):
    assert sum(run(helper, True, FRAMES_HOLD * 20)) == 1
    assert helper.state == RedLightStates.FIRED

  def test_brief_clear_does_not_rearm(self, helper):
    run(helper, True, FRAMES_HOLD)
    run(helper, False, FRAMES_REARM - 1)
    assert helper.state == RedLightStates.FIRED
    assert not any(run(helper, True, FRAMES_HOLD * 2))

  def test_clear_resets_rearm_timer(self, helper):
    run(helper, True, FRAMES_HOLD)
    run(helper, False, FRAMES_REARM - 1)
    run(helper, True, 1)
    run(helper, False, FRAMES_REARM - 1)
    assert helper.state == RedLightStates.FIRED

  def test_rearms_after_condition_clears_for_5s(self, helper):
    run(helper, True, FRAMES_HOLD)
    run(helper, False, FRAMES_REARM)
    assert helper.state == RedLightStates.ARMED
    assert sum(run(helper, True, FRAMES_HOLD)) == 1

  def test_rearms_when_car_stops(self, helper):
    run(helper, True, FRAMES_HOLD)
    helper.step(True, 0.0)
    assert helper.state == RedLightStates.ARMED
    assert sum(run(helper, True, FRAMES_HOLD)) == 1

  def test_disabled_never_fires(self, helper):
    helper.enabled = False
    assert not any(run(helper, True, FRAMES_HOLD * 5))
    assert helper.state == RedLightStates.ARMED


class TestUpdateReadsRealRadarSchema:
  """RadarState.LeadData has 'present' (no 'status'): a stand-in that only has the real fields makes any other lead access fail."""

  @staticmethod
  def sm(lead, blinker=False):
    model = SimpleNamespace(position=SimpleNamespace(x=path(stop_dist=60.0)[0]), velocity=SimpleNamespace(x=path(stop_dist=60.0)[1]))
    car_state = SimpleNamespace(vEgo=V_CRUISING, aEgo=0.0, brakePressed=False, leftBlinker=blinker, rightBlinker=False)
    return {'carState': car_state, 'modelV2': model, 'radarState': SimpleNamespace(leadOne=lead)}

  def test_read_condition_with_no_lead(self):
    assert RedLightAlertHelper.read_condition(self.sm(SimpleNamespace(present=False, dRel=0.0)))

  def test_blinker_does_not_silence_it(self):
    # a red light in a turn lane, blinker on, not slowing: still cued (driver's request 2026-11-25)
    assert RedLightAlertHelper.read_condition(self.sm(SimpleNamespace(present=False, dRel=0.0), blinker=True))

  def test_read_condition_with_lead_at_the_stop(self):
    assert not RedLightAlertHelper.read_condition(self.sm(SimpleNamespace(present=True, dRel=55.0)))

  def test_update_fires_with_events(self, helper):
    events = SimpleNamespace(names=[], add=lambda name: events.names.append(name))
    sm = self.sm(SimpleNamespace(present=False, dRel=0.0))
    for _ in range(FRAMES_HOLD):
      helper.update(sm, events)
    assert helper.red_light_alert
    assert len(events.names) == 1
