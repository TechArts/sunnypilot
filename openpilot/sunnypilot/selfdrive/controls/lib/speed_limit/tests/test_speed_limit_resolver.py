"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import time

from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver, ALL_SOURCES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.common.test import OpenpilotTestCase

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def create_mock(properties, mocker):
  mock = mocker.MagicMock()
  for _property, value in properties.items():
    setattr(mock, _property, value)
  return mock


def setup_sm_mock(mocker):
  cruise_speed_limit = random.uniform(0, 120)
  live_map_data_limit = random.uniform(0, 120)

  car_state = create_mock({
    'gasPressed': False,
    'brakePressed': False,
    'standstill': False,
  }, mocker)
  car_state_sp = create_mock({
    'speedLimit': cruise_speed_limit,
    'navSpeedLimit': 0.,
  }, mocker)
  live_map_data = create_mock({
    'speedLimit': live_map_data_limit,
    'speedLimitValid': True,
    'speedLimitAhead': 0.,
    'speedLimitAheadValid': 0.,
    'speedLimitAheadDistance': 0.,
  }, mocker)
  gps_data = create_mock({
    'unixTimestampMillis': time.monotonic() * 1e3,
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carState': car_state,
    'liveMapDataSP': live_map_data,
    'carStateSP': car_state_sp,
    'gpsLocation': gps_data,
  }[key]
  return sm_mock


parametrized_policies = parameterized.expand(
  [
    (Policy.car_state_only, 'carStateSP', SpeedLimitSource.car),
    (Policy.car_state_priority, 'carStateSP', SpeedLimitSource.car),
    (Policy.map_data_only, 'liveMapDataSP', SpeedLimitSource.map),
    (Policy.map_data_priority, 'liveMapDataSP', SpeedLimitSource.map),
  ],
  names=["policy", "sm_key", "function_key"]
)


def resolver_class():
  return SpeedLimitResolver


class TestSpeedLimitResolverValidation(OpenpilotTestCase):

  @parameterized.expand(list(Policy), names=["policy"])
  def test_initial_state(self, resolver_class, policy):
    resolver = resolver_class()
    resolver.policy = policy
    for source in ALL_SOURCES:
      if source in resolver.limit_solutions:
        assert resolver.limit_solutions[source] == 0.
        assert resolver.distance_solutions[source] == 0.

  @parametrized_policies
  def test_resolver(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the resolver
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.speed_limit == source_speed_limit
    assert resolver.source == ALL_SOURCES[function_key]

  def test_resolver_combined(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    sm_mock = setup_sm_mock(mocker)
    socket_to_source = {'carStateSP': SpeedLimitSource.car, 'liveMapDataSP': SpeedLimitSource.map}
    minimum_key, minimum_speed_limit = min(
      ((key, sm_mock[key].speedLimit) for key in
       socket_to_source.keys()), key=lambda x: x[1])

    # Assert the resolver
    resolver.update(minimum_speed_limit, sm_mock)
    assert resolver.speed_limit == minimum_speed_limit
    assert resolver.source == socket_to_source[minimum_key]

  @parametrized_policies
  def test_parser(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the parsing
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.limit_solutions[ALL_SOURCES[function_key]] == source_speed_limit
    assert resolver.distance_solutions[ALL_SOURCES[function_key]] == 0.

  @parameterized.expand(list(Policy), names=["policy"])
  def test_resolve_interaction_in_update(self, resolver_class, policy, mocker):
    v_ego = 50
    resolver = resolver_class()
    resolver.policy = policy

    sm_mock = setup_sm_mock(mocker)
    resolver.update(v_ego, sm_mock)

    # After resolution
    assert resolver.speed_limit is not None
    assert resolver.distance is not None
    assert resolver.source is not None

  @parameterized.expand(list(Policy), names=["policy"])
  def test_old_map_data_ignored(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = mocker.MagicMock()
    sm_mock['gpsLocation'].unixTimestampMillis = (time.monotonic() - 2 * LIMIT_MAX_MAP_DATA_AGE) * 1e3
    sm_mock['carStateSP'].navSpeedLimit = 0.
    sm_mock.alive = {'carStateSP': True}
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.

  def _nav_sm(self, mocker, nav, map_valid=True):
    sm_mock = setup_sm_mock(mocker)
    sm_mock['carStateSP'].navSpeedLimit = nav
    sm_mock.alive = {'carStateSP': True}
    sm_mock['liveMapDataSP'].speedLimitValid = map_valid
    return sm_mock

  @parameterized.expand([(Policy.map_data_only,), (Policy.map_data_priority,), (Policy.combined,)], names=["policy"])
  def test_nav_limit_fills_map_source_without_map_limit(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = self._nav_sm(mocker, 15.5, map_valid=False)
    resolver.update(15.5, sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 15.5
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.
    if policy != Policy.combined:
      assert resolver.speed_limit == 15.5 and resolver.source == SpeedLimitSource.map

  def test_nav_limit_does_not_override_valid_map_limit(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    sm_mock = self._nav_sm(mocker, 15.5, map_valid=True)
    map_limit = sm_mock['liveMapDataSP'].speedLimit
    resolver.update(15.5, sm_mock)
    assert resolver.speed_limit == map_limit

  def test_car_state_only_ignores_nav_limit(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.car_state_only
    sm_mock = self._nav_sm(mocker, 15.5, map_valid=False)
    resolver.update(15.5, sm_mock)
    assert resolver.speed_limit == sm_mock['carStateSP'].speedLimit
    assert resolver.source == SpeedLimitSource.car

  def test_car_priority_over_nav_limit(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.car_state_priority
    sm_mock = self._nav_sm(mocker, 15.5, map_valid=False)
    resolver.update(15.5, sm_mock)
    assert resolver.source == SpeedLimitSource.car

  def test_map_priority_falls_back_to_nav_before_car(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_priority
    sm_mock = self._nav_sm(mocker, 15.5, map_valid=False)
    resolver.update(15.5, sm_mock)
    assert resolver.speed_limit == 15.5 and resolver.source == SpeedLimitSource.map

  @parameterized.expand([(Policy.map_data_only,), (Policy.map_data_priority,), (Policy.combined,)], names=["policy"])
  def test_nav_limit_ignored_when_car_state_sp_not_alive(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = self._nav_sm(mocker, 15.5, map_valid=False)
    sm_mock.alive = {'carStateSP': False}
    resolver.update(15.5, sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.source != SpeedLimitSource.map or resolver.speed_limit != 15.5

  def test_no_nav_limit_no_change(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    sm_mock = self._nav_sm(mocker, 0., map_valid=False)
    resolver.update(15.5, sm_mock)
    assert resolver.speed_limit == 0.
    assert resolver.source == SpeedLimitSource.none

  def _vote_sm(self, mocker, dash, nav, osm, school_sign=False):
    """GV70 routes, mph: the dash value, the head unit's nav limit, OSM (0 = none)."""
    sm_mock = setup_sm_mock(mocker)
    sm_mock['carStateSP'].speedLimit = dash * CV.MPH_TO_MS
    sm_mock['carStateSP'].navSpeedLimit = nav * CV.MPH_TO_MS
    sm_mock['carStateSP'].speedLimitSchoolSign = school_sign
    sm_mock['liveMapDataSP'].speedLimit = osm * CV.MPH_TO_MS
    sm_mock['liveMapDataSP'].speedLimitValid = osm > 0
    sm_mock.alive = {'carStateSP': True}
    return sm_mock

  @parameterized.expand([
    ("truck_sign_on_the_405", 55, 65, 65, 72, 65, SpeedLimitSource.car),
    ("nav_on_a_ramp_at_speed", 40, 40, 65, 62, 65, SpeedLimitSource.map),
    ("osm_wrong_on_ca_138", 65, 65, 55, 67, 65, SpeedLimitSource.car),
    ("no_car_value", 0, 0, 45, 40, 45, SpeedLimitSource.map),
  ], names=["case", "dash", "nav", "osm", "v_ego_mph", "expected_mph", "expected_source"])
  def test_vote(self, resolver_class, case, dash, nav, osm, v_ego_mph, expected_mph, expected_source, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.vote
    sm_mock = self._vote_sm(mocker, dash, nav, osm)
    resolver.update(v_ego_mph * CV.MPH_TO_MS, sm_mock)
    assert round(resolver.speed_limit * CV.MS_TO_MPH) == expected_mph
    assert resolver.source == expected_source
    assert not resolver.school_zone

  def test_vote_ignores_car_when_car_state_sp_not_alive(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.vote
    sm_mock = self._vote_sm(mocker, 55, 55, 65)
    sm_mock.alive = {'carStateSP': False}
    resolver.update(30 * CV.MPH_TO_MS, sm_mock)
    assert round(resolver.speed_limit * CV.MS_TO_MPH) == 65 and resolver.source == SpeedLimitSource.map
