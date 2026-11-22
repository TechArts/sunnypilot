"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import messaging, custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

# Audible over-speed alert: one chime when the car is faster than the resolved speed limit plus a margin.
# Independent of cruise state; alert only.
OVER_SPEED_HOLD_TIME = 2.0  # s, the car must stay over limit + margin this long before the chime
OVER_SPEED_REARM_CLEAR_TIME = 5.0  # s, it must stay at or below limit + margin this long to re-arm


def over_speed_margin_ms(margin: int, is_metric: bool) -> float:
  """Param value as the driver sees it (mph, or km/h when the display is metric; 0 = off) in m/s."""
  return margin * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)


def over_speed(v_ego: float, speed_limit: float, margin_ms: float) -> bool:
  """Over limit + margin. No valid limit: never."""
  return speed_limit > 0. and v_ego > speed_limit + margin_ms


class OverSpeedAlert:
  """Fires once when over speed has held for hold_time, then stays silent until a clear stretch of rearm_time."""

  def __init__(self, hold_time: float = OVER_SPEED_HOLD_TIME, rearm_time: float = OVER_SPEED_REARM_CLEAR_TIME, dt: float = DT_MDL):
    self.hold_frames = round(hold_time / dt)
    self.rearm_frames = round(rearm_time / dt)
    self.reset()

  def reset(self) -> None:
    self.fired = False
    self.hold_timer = 0
    self.clear_timer = 0

  def step(self, v_ego: float, speed_limit: float, margin: int, is_metric: bool) -> bool:
    """Advance one frame (margin 0 = off). Returns True on the frame the alert fires."""
    if margin <= 0:
      self.reset()
      return False

    over = over_speed(v_ego, speed_limit, over_speed_margin_ms(margin, is_metric))
    if not self.fired:
      self.hold_timer = self.hold_timer + 1 if over else 0
      if self.hold_timer >= self.hold_frames:
        self.fired = True
        self.hold_timer = 0
        self.clear_timer = 0
        return True
    else:
      self.clear_timer = 0 if over else self.clear_timer + 1
      if self.clear_timer >= self.rearm_frames:
        self.reset()
    return False


class OverSpeedAlertHelper:
  def __init__(self):
    self._params = Params()
    self.frame = -1
    self.alert = OverSpeedAlert()
    self.margin = 0
    self.is_metric = False
    self.over_speed_alert = False
    self._read_params()

  def _read_params(self) -> None:
    self.margin = int(self._params.get("SpeedLimitOverSpeedAlert", return_default=True))
    self.is_metric = self._params.get_bool("IsMetric")

  def update(self, sm: messaging.SubMaster, speed_limit: float, events_sp: EventsSP) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self._read_params()
    self.frame += 1

    self.over_speed_alert = self.alert.step(sm['carState'].vEgo, speed_limit, self.margin, self.is_metric)
    if self.over_speed_alert:
      events_sp.add(custom.OnroadEventSP.EventName.overSpeedAlert)
