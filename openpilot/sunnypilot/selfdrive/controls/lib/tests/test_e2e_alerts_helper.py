"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import e2e_alerts_helper
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper, GO_WAIT, GREEN_LIGHT_X_THRESHOLD

GEAR_DRIVE = structs.CarState.GearShifter.drive
GO_WAIT_FRAMES = round(GO_WAIT / DT_MDL)


class FakeParams:
  values: dict[str, bool] = {}

  def get_bool(self, key):
    return self.values.get(key, True)


class FakeEvents:
  def __init__(self):
    self.count = 0

  def add(self, name):
    self.count += 1


@pytest.fixture
def helper(monkeypatch):
  monkeypatch.setattr(e2e_alerts_helper, "Params", FakeParams)
  FakeParams.values = {}
  return E2EAlertsHelper()


def disable(helper, key):
  """Turn an alert off in the settings, as the next params refresh would."""
  FakeParams.values[key] = False
  helper.green_light_alert_enabled = FakeParams().get_bool("GreenLightAlert")
  helper.lead_depart_alert_enabled = FakeParams().get_bool("LeadDepartAlert")


def sm(v_ego=0.0, gas=False, path_end=5.0, lead_d=None):
  cs = SimpleNamespace(standstill=v_ego < 0.1, vEgo=v_ego, gearShifter=GEAR_DRIVE, gasPressed=gas)
  lead = SimpleNamespace(present=lead_d is not None, dRel=lead_d if lead_d is not None else 0.0)
  return {'carState': cs, 'modelV2': SimpleNamespace(position=SimpleNamespace(x=[0.0, path_end])), 'radarState': SimpleNamespace(leadOne=lead)}


def run(helper, events, frames, **kw):
  """Run frames model frames with the same inputs; returns the frames (index) that chimed."""
  chimed = []
  for i in range(frames):
    before = events.count
    helper.update(sm(**kw), events)
    if events.count > before:
      chimed.append(i)
  return chimed


def stop(helper, events, lead_d=None):
  """Drive, then stand still long enough for the cue to be allowed (2 s after moving), with a short model path (red light)."""
  run(helper, events, 20, v_ego=10.0, lead_d=lead_d)
  assert run(helper, events, round(3.0 / DT_MDL), lead_d=lead_d) == []


class TestGreenChimeUniversal:
  def test_chimes_once_if_the_car_is_still_stopped_after_the_wait(self, helper):
    events = FakeEvents()
    stop(helper, events)
    chimed = run(helper, events, round(5.0 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0)
    assert len(chimed) == 1
    assert chimed[0] >= GO_WAIT_FRAMES  # not before the wait (trigger 0.3 s + GO_WAIT)

  def test_no_chime_when_the_car_starts_moving_by_itself(self, helper):
    """Cruise (openpilot or the car's) or the driver pulls away within the wait: no cue."""
    events = FakeEvents()
    stop(helper, events)
    run(helper, events, round(0.5 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0)
    assert run(helper, events, round(5.0 / DT_MDL), v_ego=2.0, path_end=GREEN_LIGHT_X_THRESHOLD + 20.0) == []

  def test_no_chime_when_the_driver_presses_the_gas_during_the_wait(self, helper):
    events = FakeEvents()
    stop(helper, events)
    run(helper, events, round(0.5 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0)
    assert run(helper, events, round(1.0 / DT_MDL), gas=True, path_end=GREEN_LIGHT_X_THRESHOLD + 20.0) == []
    assert helper.pending_frame is None

  def test_one_chime_per_stop_even_if_the_path_flickers(self, helper):
    events = FakeEvents()
    stop(helper, events)
    assert len(run(helper, events, round(3.0 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0)) == 1
    run(helper, events, round(1.0 / DT_MDL), gas=True)  # re-arms the state machine (not allowed), the car does not move
    assert run(helper, events, round(5.0 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0) == []

  def test_next_stop_chimes_again(self, helper):
    events = FakeEvents()
    stop(helper, events)
    assert len(run(helper, events, round(3.0 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0)) == 1
    stop(helper, events)
    assert len(run(helper, events, round(3.0 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0)) == 1

  def test_published_flag_marks_the_chime_frame(self, helper):
    events = FakeEvents()
    stop(helper, events)
    flags = []
    for _ in range(round(3.0 / DT_MDL)):
      helper.update(sm(path_end=GREEN_LIGHT_X_THRESHOLD + 20.0), events)
      flags.append(helper.green_light_alert)
    assert sum(flags) == 1 and events.count == 1

  def test_lead_departure_then_green_is_one_chime(self, helper):
    """A car ahead pulls away (lead-depart cue), then the path clears (green cue) at the same stop: one chime."""
    events = FakeEvents()
    stop(helper, events, lead_d=4.0)
    chimed = run(helper, events, round(3.0 / DT_MDL), lead_d=7.5)  # still a close lead (< 8 m), 3.5 m further away: lead-depart cue
    chimed += run(helper, events, round(3.0 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0)
    assert len(chimed) == 1

  def test_green_turned_off_during_the_wait_does_not_chime(self, helper):
    events = FakeEvents()
    stop(helper, events)
    run(helper, events, round(0.5 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0)
    assert helper.pending_frame is not None
    disable(helper, "GreenLightAlert")
    assert run(helper, events, round(3.0 / DT_MDL), path_end=GREEN_LIGHT_X_THRESHOLD + 20.0) == []

  def test_lead_depart_turned_off_during_the_wait_does_not_chime(self, helper):
    events = FakeEvents()
    stop(helper, events, lead_d=4.0)
    run(helper, events, round(0.6 / DT_MDL), lead_d=7.5)
    assert helper.pending_frame is not None and not helper.pending_green
    disable(helper, "LeadDepartAlert")
    assert run(helper, events, round(3.0 / DT_MDL), lead_d=7.5) == []
