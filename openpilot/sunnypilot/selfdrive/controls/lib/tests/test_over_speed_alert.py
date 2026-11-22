"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest
from types import SimpleNamespace
from unittest import mock

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import over_speed_alert
from openpilot.sunnypilot.selfdrive.controls.lib.over_speed_alert import (
  OverSpeedAlert, OverSpeedAlertHelper, current_speed_limit, over_speed, over_speed_margin_ms, OVER_SPEED_HOLD_TIME, OVER_SPEED_REARM_CLEAR_TIME)

LIMIT = 35 * CV.MPH_TO_MS
HOLD = round(OVER_SPEED_HOLD_TIME / DT_MDL)
REARM = round(OVER_SPEED_REARM_CLEAR_TIME / DT_MDL)


def run(alert, frames, v_mph, limit=LIMIT, margin=10, metric=False):
  return [alert.step(v_mph * CV.MPH_TO_MS, limit, margin, metric) for _ in range(frames)]


class TestOverSpeedAlert(unittest.TestCase):
  def test_margin_units(self):
    self.assertAlmostEqual(over_speed_margin_ms(10, False), 10 * CV.MPH_TO_MS)
    self.assertAlmostEqual(over_speed_margin_ms(10, True), 10 * CV.KPH_TO_MS)

  def test_threshold_is_strict(self):
    m = over_speed_margin_ms(10, False)
    self.assertFalse(over_speed(LIMIT + m, LIMIT, m))
    self.assertTrue(over_speed(LIMIT + m + 0.01, LIMIT, m))

  def test_fires_once_after_hold(self):
    a = OverSpeedAlert()
    out = run(a, HOLD + 100, 46)
    self.assertEqual(out.index(True), HOLD - 1)
    self.assertEqual(sum(out), 1)

  def test_at_margin_never_fires(self):
    self.assertFalse(any(run(OverSpeedAlert(), 3 * HOLD, 45)))

  def test_hold_resets_when_dip_below(self):
    a = OverSpeedAlert()
    self.assertFalse(any(run(a, HOLD - 1, 46)))
    self.assertFalse(any(run(a, 1, 40)))
    self.assertFalse(any(run(a, HOLD - 1, 46)))
    self.assertTrue(any(run(a, 1, 46)))

  def test_rearm_needs_clear_time(self):
    a = OverSpeedAlert()
    self.assertTrue(any(run(a, HOLD, 46)))
    self.assertFalse(any(run(a, REARM - 1, 40)))  # just short of re-arming
    self.assertFalse(any(run(a, HOLD + 10, 46)))  # still disarmed
    self.assertFalse(any(run(a, REARM - 1, 40)))
    run(a, 1, 40)  # re-armed
    self.assertTrue(any(run(a, HOLD, 46)))

  def test_over_speed_resets_rearm_clear_timer(self):
    a = OverSpeedAlert()
    run(a, HOLD, 46)
    run(a, REARM - 1, 40)
    run(a, 1, 46)  # back over: the clear stretch restarts
    self.assertFalse(any(run(a, REARM - 1, 40)))
    self.assertFalse(any(run(a, HOLD + 10, 46)))

  def test_no_limit(self):
    a = OverSpeedAlert()
    self.assertFalse(any(run(a, 3 * HOLD, 90, limit=0.)))

  def test_limit_lost_counts_as_clear(self):
    a = OverSpeedAlert()
    self.assertFalse(any(run(a, HOLD - 1, 46)))
    run(a, 1, 46, limit=0.)
    self.assertFalse(any(run(a, HOLD - 1, 46)))

  def test_off(self):
    a = OverSpeedAlert()
    self.assertFalse(any(run(a, 3 * HOLD, 90, margin=0)))

  def test_off_resets_state(self):
    a = OverSpeedAlert()
    self.assertTrue(any(run(a, HOLD, 46)))
    run(a, 1, 46, margin=0)
    self.assertTrue(any(run(a, HOLD, 46)))

  def test_metric_margin(self):
    a = OverSpeedAlert()
    limit = 50 * CV.KPH_TO_MS
    self.assertFalse(any(a.step(59 * CV.KPH_TO_MS, limit, 10, True) for _ in range(2 * HOLD)))
    self.assertTrue(any(a.step(61 * CV.KPH_TO_MS, limit, 10, True) for _ in range(HOLD)))

  def test_limit_drop_starts_fresh_hold(self):
    a = OverSpeedAlert()
    lim40, lim25 = 40 * CV.MPH_TO_MS, 25 * CV.MPH_TO_MS
    self.assertFalse(any(run(a, 3 * HOLD, 40, limit=lim40)))  # at the limit: fine
    out = run(a, HOLD + 5, 40, limit=lim25)  # 40 -> 25 mph at 40 mph
    self.assertFalse(any(out[:HOLD - 1]))
    self.assertTrue(out[HOLD - 1])

  def test_limit_change_after_firing_rearms_with_fresh_hold(self):
    a = OverSpeedAlert()
    self.assertTrue(any(run(a, HOLD, 46)))
    self.assertFalse(any(run(a, HOLD + 5, 46)))  # latched
    out = run(a, HOLD, 46, limit=35.5 * CV.MPH_TO_MS)  # the limit moves: a new road
    self.assertFalse(any(out[:HOLD - 1]))
    self.assertTrue(out[HOLD - 1])

  def test_outage_then_recovery_while_speeding_chimes_again(self):
    a = OverSpeedAlert()
    self.assertTrue(any(run(a, HOLD, 46)))
    run(a, 3, 46, limit=0.)  # brief outage
    out = run(a, HOLD, 46)
    self.assertFalse(any(out[:HOLD - 1]))
    self.assertTrue(out[HOLD - 1])

  def test_upcoming_limit_ahead_is_ignored(self):
    self.assertEqual(current_speed_limit(25 * CV.MPH_TO_MS, 120.), 0.)
    self.assertEqual(current_speed_limit(25 * CV.MPH_TO_MS, 0.), 25 * CV.MPH_TO_MS)

  def test_helper_no_chime_until_upcoming_limit_applies(self):
    class FakeParams:
      def get(self, key, return_default=False):
        return 10 if key == "SpeedLimitOverSpeedAlert" else 0

      def get_bool(self, key):
        return False

    with mock.patch.object(over_speed_alert, "Params", FakeParams):
      helper = OverSpeedAlertHelper()
    events = mock.Mock()
    sm = {'carState': SimpleNamespace(vEgo=40 * CV.MPH_TO_MS)}
    lim40, lim25 = 40 * CV.MPH_TO_MS, 25 * CV.MPH_TO_MS
    for _ in range(3 * HOLD):  # 25 mph limit ahead (anticipatory, distance > 0), still in a 40 zone
      helper.update(sm, lim25, 100., events)
    events.add.assert_not_called()
    for _ in range(HOLD):  # it applies now
      helper.update(sm, lim25, 0., events)
    events.add.assert_called_once()


if __name__ == "__main__":
  unittest.main()
