"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from cereal import custom
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

ClusterAlert = custom.CarControlSP.ClusterAlert


class FakeSM:
  def __init__(self, updated=True, red=False, green=False, lead_depart=False):
    self.updated = {'longitudinalPlanSP': updated}
    self._plan = SimpleNamespace(e2eAlerts=SimpleNamespace(redLightAlert=red, greenLightAlert=green, leadDepartAlert=lead_depart))

  def __getitem__(self, key):
    return self._plan


class TestClusterAlertRequest:
  def test_red(self):
    assert ControlsExt.get_cluster_alert(FakeSM(red=True)) == ClusterAlert.red

  def test_green_from_either_e2e_alert(self):
    assert ControlsExt.get_cluster_alert(FakeSM(green=True)) == ClusterAlert.green
    assert ControlsExt.get_cluster_alert(FakeSM(lead_depart=True)) == ClusterAlert.green

  def test_none(self):
    assert ControlsExt.get_cluster_alert(FakeSM()) == ClusterAlert.none

  def test_one_shot_only_on_new_plan_message(self):
    assert ControlsExt.get_cluster_alert(FakeSM(updated=False, red=True)) == ClusterAlert.none

  def test_red_wins_over_green(self):
    assert ControlsExt.get_cluster_alert(FakeSM(red=True, green=True)) == ClusterAlert.red
