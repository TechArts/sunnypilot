"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.over_speed_alert import (
  OverSpeedAlert, over_speed, over_speed_margin_ms, OVER_SPEED_HOLD_TIME, OVER_SPEED_REARM_CLEAR_TIME)

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


if __name__ == "__main__":
  unittest.main()
