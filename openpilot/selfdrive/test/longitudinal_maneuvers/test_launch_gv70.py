import unittest

from openpilot.selfdrive.test.longitudinal_maneuvers.study_launch_gv70 import ROWS, SCENARIOS, run

MIN_CREEP_GAP = 4.0  # m: behind a car that creeps forward and stops again (route 7d: the old build closed to 1.45 m)
AWAY_GAP_LOSS = 0.5  # m: a car driving away may not be followed closer than its start gap minus this
MAX_AWAY_T3 = 8.0  # s from standstill to 3 m/s behind a car driving away


class TestLaunchGV70(unittest.TestCase):
  def test_synthetic_launch_gaps(self):
    for delivery, lag in ROWS:
      for name, (gap0, _, _) in SCENARIOS.items():
        with self.subTest(delivery=delivery, lag=lag, scenario=name):
          r = run(name, delivery, lag)
          if name.startswith('creep'):
            self.assertGreaterEqual(r['min'], min(gap0, MIN_CREEP_GAP))
            self.assertGreaterEqual(r['end'], MIN_CREEP_GAP)
          else:
            self.assertGreaterEqual(r['min'], gap0 - AWAY_GAP_LOSS)
            self.assertLessEqual(r['t3'], MAX_AWAY_T3)


if __name__ == "__main__":
  unittest.main()
