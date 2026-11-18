"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import messaging


def e2e_alert_fired(e2e_alerts) -> bool:
  return e2e_alerts.redLightAlert or e2e_alerts.greenLightAlert or e2e_alerts.leadDepartAlert


def publish_alert_bookmark(sm, pm: messaging.PubMaster) -> bool:
  """Mark the moment a red/green/lead-depart alert fires so the clip can be labelled later: loggerd keeps the segment of a userBookmark and
  selfdrived shows the bookmark alert, exactly like the bookmark button. msgq allows one userBookmark publisher and the UI layouts own it,
  so the alert is forwarded through the layout's PubMaster."""
  if not (sm.updated['longitudinalPlanSP'] and e2e_alert_fired(sm['longitudinalPlanSP'].e2eAlerts)):
    return False
  pm.send('userBookmark', messaging.new_message('userBookmark', valid=True))
  return True
