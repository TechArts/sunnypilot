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
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper, BRAKE_GO_LOOKBACK, GO_WAIT, GREEN_LIGHT_X_THRESHOLD

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


def sm(v_ego=0.0, gas=False, path_end=5.0, lead_d=None, brake=False):
  cs = SimpleNamespace(standstill=v_ego < 0.1, vEgo=v_ego, gearShifter=GEAR_DRIVE, gasPressed=gas, brakePressed=brake)
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


def stop(helper, events, lead_d=None, brake=False):
  """Drive, then stand still long enough for the cue to be allowed (2 s after moving), with a short model path (red light)."""
  run(helper, events, 20, v_ego=10.0, lead_d=lead_d)
  assert run(helper, events, round(3.0 / DT_MDL), lead_d=lead_d, brake=brake) == []


def frames(seconds):
  return round(seconds / DT_MDL)


def green_until_trigger(helper, events, brake=True):
  """Clear the path (green) with the brake in the given state until the trigger starts the wait; no chime before it."""
  for _ in range(frames(2.0)):
    assert run(helper, events, 1, path_end=GREEN_LIGHT_X_THRESHOLD + 20.0, brake=brake) == []
    if helper.pending_frame is not None:
      return
  raise AssertionError("no trigger")


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


class TestBrakeReleaseIsGoing:
  """F32: the driver letting go of the brake means they are already going; the chime is for a driver who has not noticed."""
  GREEN = GREEN_LIGHT_X_THRESHOLD + 20.0

  @pytest.mark.parametrize("release_after", [0.2, 0.5, 0.6, 0.8])  # routes 5d/5e: released 0.2-0.8 s after the trigger, the chime came after
  def test_release_during_the_wait_does_not_chime(self, helper, release_after):
    events = FakeEvents()
    stop(helper, events, brake=True)
    green_until_trigger(helper, events)
    assert run(helper, events, frames(release_after), path_end=self.GREEN, brake=True) == []
    assert run(helper, events, frames(3.0), path_end=self.GREEN) == []

  def test_release_after_the_chime_keeps_the_chime(self, helper):
    """Route 5d 3:25.6: the chime came 0.4 s before the driver let go."""
    events = FakeEvents()
    stop(helper, events, brake=True)
    green_until_trigger(helper, events)
    assert len(run(helper, events, frames(GO_WAIT + 0.2), path_end=self.GREEN, brake=True)) == 1
    assert run(helper, events, frames(3.0), path_end=self.GREEN) == []

  def test_brake_held_through_the_wait_chimes(self, helper):
    events = FakeEvents()
    stop(helper, events, brake=True)
    green_until_trigger(helper, events)
    assert len(run(helper, events, frames(3.0), path_end=self.GREEN, brake=True)) == 1

  def test_release_just_before_the_trigger_does_not_chime(self, helper):
    """The driver saw the green before the model's path cleared."""
    events = FakeEvents()
    stop(helper, events, brake=True)
    run(helper, events, frames(BRAKE_GO_LOOKBACK / 2), brake=False)
    assert run(helper, events, frames(5.0), path_end=self.GREEN) == []

  def test_foot_off_long_before_the_green_chimes(self, helper):
    """The car holds the stop (Auto Hold, openpilot cruise): no release near the green, so the chime stays."""
    events = FakeEvents()
    stop(helper, events, brake=True)
    run(helper, events, frames(BRAKE_GO_LOOKBACK + 2.0), brake=False)
    assert len(run(helper, events, frames(5.0), path_end=self.GREEN)) == 1

  def test_no_brake_at_all_chimes(self, helper):
    """openpilot cruise holding the stop: the brake is never pressed (a press ends cruise)."""
    events = FakeEvents()
    stop(helper, events)
    assert len(run(helper, events, frames(5.0), path_end=self.GREEN)) == 1

  def test_release_then_brake_again_chimes(self, helper):
    """A release followed by a new press before the trigger: the driver is not going."""
    events = FakeEvents()
    stop(helper, events, brake=True)
    run(helper, events, frames(0.2), brake=False)
    green_until_trigger(helper, events, brake=True)
    assert len(run(helper, events, frames(3.0), path_end=self.GREEN, brake=True)) == 1

  def test_release_then_brake_again_during_the_wait_chimes(self, helper):
    """Released 0.2 s after the trigger, pressed again at 0.4 s, the car never moved: the driver is not going."""
    events = FakeEvents()
    stop(helper, events, brake=True)
    green_until_trigger(helper, events)
    assert run(helper, events, frames(0.2), path_end=self.GREEN, brake=True) == []
    assert run(helper, events, frames(0.2), path_end=self.GREEN) == []
    assert len(run(helper, events, frames(3.0), path_end=self.GREEN, brake=True)) == 1

  def test_lead_departure_release_during_the_wait_does_not_chime(self, helper):
    """Route 5d 3:07.4: the car ahead pulled away, the driver let go of the brake before the chime."""
    events = FakeEvents()
    stop(helper, events, lead_d=4.0, brake=True)
    for _ in range(frames(2.0)):
      run(helper, events, 1, lead_d=7.5, brake=True)
      if helper.pending_frame is not None:
        break
    assert helper.pending_frame is not None and not helper.pending_green
    run(helper, events, frames(0.2), lead_d=7.5, brake=True)
    assert run(helper, events, frames(3.0), lead_d=7.5) == []
