"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import time

from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized
from openpilot.common.realtime import DT_MDL

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
    'unixTimestampMillis': 1795578296299,  # UTC wall time as the GPS sends it (route 94), unrelated to time.monotonic()
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carState': car_state,
    'liveMapDataSP': live_map_data,
    'carStateSP': car_state_sp,
    'gpsLocation': gps_data,
  }[key]
  now = time.monotonic()
  sm_mock.recv_time = {'gpsLocation': now, 'liveMapDataSP': now, 'carStateSP': now}
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

  @parameterized.expand([(p, svc) for p in Policy for svc in ('gpsLocation', 'liveMapDataSP')], names=["policy", "old"])
  def test_old_map_data_ignored(self, resolver_class, policy, old, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = self._nav_sm(mocker, 0.)
    sm_mock.recv_time[old] = time.monotonic() - 2 * LIMIT_MAX_MAP_DATA_AGE
    resolver.v_ego = 20.
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.
    if policy == Policy.vote:
      resolver.update(20., sm_mock)
      assert resolver.limit_solutions[SpeedLimitSource.map] == 0.

  def test_map_data_age_is_on_the_planner_clock(self, resolver_class, mocker):
    # the GPS message's unixTimestampMillis is UTC wall time (~1.8e12 ms on the port routes, against a monotonic clock of ~2e4 s): fresh
    # messages are fresh whatever it says
    resolver = resolver_class()
    sm_mock = self._nav_sm(mocker, 0.)
    sm_mock['gpsLocation'].unixTimestampMillis = 1795578296299
    assert 0. <= resolver._map_data_age(sm_mock) < 1.
    sm_mock.recv_time['liveMapDataSP'] = time.monotonic() - 3.
    assert 3. <= resolver._map_data_age(sm_mock) < 4.

  def _ahead_sm(self, mocker, current_mph, ahead_mph, distance, map_age=0.):
    sm_mock = self._nav_sm(mocker, 0.)
    live_map = sm_mock['liveMapDataSP']
    live_map.speedLimit, live_map.speedLimitValid = current_mph * CV.MPH_TO_MS, True
    live_map.speedLimitAhead, live_map.speedLimitAheadValid, live_map.speedLimitAheadDistance = ahead_mph * CV.MPH_TO_MS, True, distance
    sm_mock.recv_time['liveMapDataSP'] = time.monotonic() - map_age
    return sm_mock

  @parameterized.expand([(p, ahead) for p in (Policy.map_data_only, Policy.map_data_priority, Policy.combined) for ahead in (50, 65)],
                        names=["policy", "ahead_mph"])
  def test_upstream_policies_keep_the_current_osm_limit(self, resolver_class, policy, ahead_mph, mocker):
    # policies 0-4 with the age fixed: OSM's current 55 and distance 0 all the way to a lower (or higher) limit ahead, frame after frame
    resolver = resolver_class()
    resolver.policy = policy
    mocker.patch.object(resolver, 'update_params')  # keep the policy: the periodic refresh reads SpeedLimitPolicy from Params
    for distance in range(300, 0, -5):
      sm_mock = self._ahead_sm(mocker, 55, ahead_mph, float(distance))
      sm_mock['carStateSP'].speedLimit = 55 * CV.MPH_TO_MS
      resolver.update(60 * CV.MPH_TO_MS, sm_mock)
      assert round(resolver.speed_limit * CV.MS_TO_MPH) == 55 and resolver.distance == 0., distance
    assert resolver.policy == policy

  def test_vote_reads_the_current_osm_limit(self, resolver_class, mocker):
    # the upstream anticipation must not reach the vote as OSM's limit: car and OSM 65, OSM 55 ahead in 50 m
    resolver = resolver_class()
    resolver.policy = Policy.vote
    sm_mock = self._ahead_sm(mocker, 65, 55, 50.)
    sm_mock['carStateSP'].speedLimit = sm_mock['carStateSP'].navSpeedLimit = 65 * CV.MPH_TO_MS
    resolver.update(73 * CV.MPH_TO_MS, sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 65 * CV.MPH_TO_MS

  def _vote_ahead(self, mocker, distance, map_age=0.5, seconds=1.2):
    """Route 91 Pearblossom Hwy: dash, nav and OSM 55, OSM 50 ahead, the car at 79 mph; the vote's frames for `seconds`."""
    resolver = SpeedLimitResolver()
    resolver.policy = Policy.vote
    mocker.patch.object(resolver, 'update_params')  # keep the policy: the periodic refresh reads SpeedLimitPolicy from Params
    sm_mock = self._ahead_sm(mocker, 55, 50, distance, map_age)
    sm_mock['carStateSP'].speedLimit = sm_mock['carStateSP'].navSpeedLimit = 55 * CV.MPH_TO_MS
    sm_mock['carStateSP'].speedLimitSchoolSign = False
    for _ in range(round(seconds / DT_MDL)):
      resolver.update(79 * CV.MPH_TO_MS, sm_mock)
    return resolver

  def test_vote_takes_a_lower_limit_ahead_early_with_its_distance(self, resolver_class, mocker):
    resolver = self._vote_ahead(mocker, 600.)  # inside the 819 m openpilot needs from 79 mph at 0.5 m/s^2
    assert round(resolver.speed_limit * CV.MS_TO_MPH) == 50 and resolver.source == SpeedLimitSource.map
    assert 600. - 79 * CV.MPH_TO_MS * 0.6 < resolver.distance < 600. - 79 * CV.MPH_TO_MS * 0.4  # the travel since the map message
    resolver = self._vote_ahead(mocker, 1000.)
    assert round(resolver.speed_limit * CV.MS_TO_MPH) == 55 and resolver.distance == 0.

  def test_vote_ignores_a_stale_or_unplaced_limit_ahead(self, resolver_class, mocker):
    resolver = self._vote_ahead(mocker, 600., map_age=2 * LIMIT_MAX_MAP_DATA_AGE)
    assert round(resolver.speed_limit * CV.MS_TO_MPH) == 55 and resolver.limit_solutions[SpeedLimitSource.map] == 0.
    resolver = self._vote_ahead(mocker, 0.)  # mapd sends 0 m when the limit ahead has no coordinates
    assert round(resolver.speed_limit * CV.MS_TO_MPH) == 55

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
