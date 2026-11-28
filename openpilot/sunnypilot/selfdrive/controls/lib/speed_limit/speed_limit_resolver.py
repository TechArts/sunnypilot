"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD, get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy, OffsetType
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.vote import SpeedLimitVote
from openpilot.sunnypilot.selfdrive.driver_profile import DEFAULT_PROFILE, SpeedLimit

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source

ALL_SOURCES = tuple(SpeedLimitSource.schema.enumerants.values())


class SpeedLimitResolver:
  limit_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  distance_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  v_ego: float
  speed_limit: float
  speed_limit_last: float
  speed_limit_final: float
  speed_limit_final_last: float
  distance: float
  source: custom.LongitudinalPlanSP.SpeedLimit.Source
  speed_limit_offset: float

  def __init__(self, cfg: SpeedLimit = DEFAULT_PROFILE.speed_limit):
    self.params = Params()
    self.frame = -1

    self._gps_location_service = get_gps_location_service(self.params)
    self.limit_solutions = {}  # Store for speed limit solutions from different sources
    self.distance_solutions = {}  # Store for distance to current speed limit start for different sources

    self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
    self.policy = get_sanitize_int_param(
      "SpeedLimitPolicy",
      Policy.min().value,
      Policy.max().value,
      self.params
    )
    self._policy_to_sources_map = {
      Policy.car_state_only: [SpeedLimitSource.car],
      Policy.map_data_only: [SpeedLimitSource.map],
      Policy.car_state_priority: [SpeedLimitSource.car, SpeedLimitSource.map],
      Policy.map_data_priority: [SpeedLimitSource.map, SpeedLimitSource.car],
      Policy.vote: [SpeedLimitSource.car, SpeedLimitSource.map],
      Policy.combined: [SpeedLimitSource.car, SpeedLimitSource.map],
    }
    self.source = SpeedLimitSource.none
    for source in ALL_SOURCES:
      self._reset_limit_sources(source)

    self.is_metric = self.params.get_bool("IsMetric")
    self.offset_type = get_sanitize_int_param(
      "SpeedLimitOffsetType",
      OffsetType.min().value,
      OffsetType.max().value,
      self.params
    )
    self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

    self.speed_limit = 0.
    self.distance = 0.  # read by the over-speed alert on the planner's first cycle, before the first update()
    self.speed_limit_last = 0.
    self.speed_limit_final = 0.
    self.speed_limit_final_last = 0.
    self.speed_limit_offset = 0.
    self.school_zone = False
    self._vote = SpeedLimitVote(DT_MDL, cfg)

  def update_speed_limit_states(self) -> None:
    self.speed_limit_final = self.speed_limit + self.speed_limit_offset

    if self.speed_limit > 0.:
      self.speed_limit_last = self.speed_limit
      self.speed_limit_final_last = self.speed_limit_final

  @property
  def speed_limit_valid(self) -> bool:
    return self.speed_limit > 0.

  @property
  def speed_limit_last_valid(self) -> bool:
    return self.speed_limit_last > 0.

  def update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
      self.is_metric = self.params.get_bool("IsMetric")
      self.offset_type = self.params.get("SpeedLimitOffsetType", return_default=True)
      self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

  def _get_speed_limit_offset(self) -> float:
    if self.offset_type == OffsetType.off:
      return 0
    elif self.offset_type == OffsetType.fixed:
      return float(self.offset_value * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS))
    elif self.offset_type == OffsetType.percentage:
      return float(self.offset_value * 0.01 * self.speed_limit)
    else:
      raise NotImplementedError("Offset not supported")

  def _reset_limit_sources(self, source: custom.LongitudinalPlanSP.SpeedLimit.Source) -> None:
    self.limit_solutions[source] = 0.
    self.distance_solutions[source] = 0.

  def _get_from_car_state(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.car)
    self.limit_solutions[SpeedLimitSource.car] = sm['carStateSP'].speedLimit
    self.distance_solutions[SpeedLimitSource.car] = 0.

  def _get_from_map_data(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.map)
    self._process_map_data(sm)

    # the car's navigation limit stands in for the map only when the map has no valid limit (OSM wins when valid)
    # (only while carStateSP is alive: a frozen producer must not keep a stale limit)
    nav_speed_limit = sm['carStateSP'].navSpeedLimit if sm.alive['carStateSP'] else 0.
    if self.limit_solutions[SpeedLimitSource.map] <= 0. and nav_speed_limit > 0.:
      self.limit_solutions[SpeedLimitSource.map] = nav_speed_limit
      self.distance_solutions[SpeedLimitSource.map] = 0.

  def _map_data_age(self, sm: messaging.SubMaster) -> float:
    """Seconds since the planner received the last GPS and map messages, the older of the two, on the planner's own clock (time.monotonic(),
    as SubMaster.recv_time). The GPS message's unixTimestampMillis is UTC wall time: against time.monotonic() it came out about -1.8e9 s,
    so no map data was ever too old and an ahead limit was never within reach."""
    now = time.monotonic()
    return max(now - sm.recv_time[self._gps_location_service], now - sm.recv_time['liveMapDataSP'])

  def _map_distance_ahead(self, sm: messaging.SubMaster) -> float:
    """The distance to the start of the limit ahead now: mapd measured it from the position it had when it published."""
    elapsed = time.monotonic() - sm.recv_time['liveMapDataSP']
    return max(0., sm['liveMapDataSP'].speedLimitAheadDistance - self.v_ego * elapsed)

  def _process_map_data(self, sm: messaging.SubMaster) -> None:
    """Policies 0-4: OSM's current limit, distance 0. Upstream's anticipation of the limit ahead is gone: it never ran (the age was wrong by
    -1.8e9 s), and with a right age it re-decided from the current speed every frame, flipping back to the current limit as the car slowed."""
    map_data = sm['liveMapDataSP']
    if self._map_data_age(sm) > LIMIT_MAX_MAP_DATA_AGE:
      return
    self.limit_solutions[SpeedLimitSource.map] = map_data.speedLimit if map_data.speedLimitValid else 0.
    self.distance_solutions[SpeedLimitSource.map] = 0.

  def _get_source_solution_according_to_policy(self) -> custom.LongitudinalPlanSP.SpeedLimit.Source:
    sources_for_policy = self._policy_to_sources_map[Policy(self.policy)]

    if Policy(self.policy) != Policy.combined:
      # They are ordered in the order of preference, so we pick the first that's non-zero
      for source in sources_for_policy:
        if self.limit_solutions[source] > 0.:
          return source
      return SpeedLimitSource.none

    sources_with_limits = [(s, limit) for s, limit in [(s, self.limit_solutions[s]) for s in sources_for_policy] if limit > 0.]
    if sources_with_limits:
      return min(sources_with_limits, key=lambda x: x[1])[0]

    return SpeedLimitSource.none

  def _resolve_by_vote(self, sm: messaging.SubMaster) -> tuple[float, float, custom.LongitudinalPlanSP.SpeedLimit.Source]:
    """Policy vote: the car's dash value, its nav value and OSM's current limit (without the nav stand-in or the upstream anticipation)
    plus OSM's limit ahead, see speed_limit/vote.py. The distance is the one to a lower limit ahead the vote took early, else 0."""
    self._reset_limit_sources(SpeedLimitSource.map)
    map_data = sm['liveMapDataSP']
    ahead = ahead_dist = 0.
    if map_data.speedLimitValid and self._map_data_age(sm) <= LIMIT_MAX_MAP_DATA_AGE:
      self.limit_solutions[SpeedLimitSource.map] = map_data.speedLimit
      if map_data.speedLimitAheadValid and map_data.speedLimitAheadDistance > 0.:
        ahead, ahead_dist = map_data.speedLimitAhead, self._map_distance_ahead(sm)
    car_sp = sm['carStateSP']
    alive = sm.alive['carStateSP']
    dash = car_sp.speedLimit if alive else 0.
    nav = car_sp.navSpeedLimit if alive else 0.
    self.limit_solutions[SpeedLimitSource.car] = dash
    result = self._vote.update(dash, nav, self.limit_solutions[SpeedLimitSource.map], alive and car_sp.speedLimitSchoolSign,
                               self.v_ego, self.is_metric, ahead=ahead, ahead_dist=ahead_dist)
    self.school_zone = result.school
    if result.limit <= 0.:
      return 0., 0., SpeedLimitSource.none
    return result.limit, result.distance, SpeedLimitSource.map if result.from_map else SpeedLimitSource.car

  def _resolve_limit_sources(self, sm: messaging.SubMaster) -> tuple[float, float, custom.LongitudinalPlanSP.SpeedLimit.Source]:
    """Get limit solutions from each data source"""
    if Policy(self.policy) == Policy.vote:
      return self._resolve_by_vote(sm)
    self.school_zone = False
    self._get_from_car_state(sm)
    self._get_from_map_data(sm)

    source = self._get_source_solution_according_to_policy()
    speed_limit = self.limit_solutions[source] if source else 0.
    distance = self.distance_solutions[source] if source else 0.

    return speed_limit, distance, source

  def update(self, v_ego: float, sm: messaging.SubMaster) -> None:
    self.v_ego = v_ego
    self.update_params()

    self.speed_limit, self.distance, self.source = self._resolve_limit_sources(sm)
    self.speed_limit_offset = self._get_speed_limit_offset()

    self.update_speed_limit_states()

    self.frame += 1
