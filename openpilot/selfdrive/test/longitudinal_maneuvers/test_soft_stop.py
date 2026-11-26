from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized_class

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver

# (actuator delivery, lag s): an ideal car, a good car, and a car that delivers 88 % of small decel requests 0.6 s late
ACTUATOR_ROWS = [(1.0, 0.0), (0.97, 0.2), (0.88, 0.6)]
# without the planner speed bound the last row stops 2.5-2.8 m behind the lead, with it 4.4 m or more
MIN_STOP_GAP = STOP_DISTANCE - 2.0


def create_maneuvers(platform, delivery, lag):
  return [
    Maneuver(
      f'approach stopped car at {v0}m/s, actuator {delivery} x decel, {lag}s lag',
      duration=40.,
      initial_speed=v0,
      lead_relevancy=True,
      initial_distance_lead=STOP_DISTANCE + 10. * v0 + 20.,
      speed_lead_values=[0., 0.],
      actuator_delivery=delivery,
      actuator_lag=lag,
      use_long_control=True,
      platform=platform,
    ) for v0 in (5., 10., 15.)
  ]


@parameterized_class(("platform", "delivery", "lag"), [(p, d, lag) for p in ("HYUNDAI_IONIQ_5", "TOYOTA_RAV4_TSS2") for d, lag in ACTUATOR_ROWS])
class TestSoftStop(OpenpilotTestCase):
  platform: str
  delivery: float
  lag: float

  def test_stopped_lead_gap(self, subtests):
    for maneuver in create_maneuvers(self.platform, self.delivery, self.lag):
      with subtests.test(title=maneuver.title, platform=self.platform):
        valid, logs = maneuver.evaluate()
        gaps = logs[:, 2] - logs[:, 1]
        assert valid
        assert logs[-1, 3] == 0.0, "did not come to a standstill"
        assert gaps.min() >= MIN_STOP_GAP, f"stopped {gaps.min():.2f} m behind the lead"
