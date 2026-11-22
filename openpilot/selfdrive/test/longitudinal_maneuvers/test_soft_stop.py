import unittest

from openpilot.selfdrive.test.longitudinal_maneuvers.study_soft_stop import run

# (actuator delivery, lag s): ideal car, a good car, the GV70 as fitted from logged drives
REALISTIC_ROWS = [(1.0, 0.0), (0.97, 0.2), (0.88, 0.6)]
MIN_STOP_GAP = 4.5  # m, behind a stopped lead (the lead MPC aims for 6.0 m)


class TestSoftStop(unittest.TestCase):
  def test_stopped_lead_gap(self):
    for delivery, lag in REALISTIC_ROWS:
      for v0 in (5., 10., 15.):
        with self.subTest(delivery=delivery, lag=lag, v0=v0):
          r = run(v0, delivery, lag)
          self.assertTrue(r['t1'] == r['t1'], "did not come to a standstill")  # t1 is NaN without a stop
          self.assertGreaterEqual(r['min'], MIN_STOP_GAP)
          self.assertGreaterEqual(r['minreq'], -2.5 if v0 > 5. else -2.0)
          self.assertLessEqual(r['jerk'], 2.0)
          self.assertLessEqual(r['drift'], 0.1)


if __name__ == "__main__":
  unittest.main()
