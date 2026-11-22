from types import SimpleNamespace

from openpilot.selfdrive.ui.sunnypilot.alert_bookmark import publish_alert_bookmark


class FakePM:
  def __init__(self):
    self.sent = []

  def send(self, service, msg):
    self.sent.append(service)


class SM:
  def __init__(self, **kw):
    e2e = SimpleNamespace(redLightAlert=kw.get('red', False), greenLightAlert=kw.get('green', False), leadDepartAlert=kw.get('lead_depart', False))
    self.updated = {'longitudinalPlanSP': kw.get('updated', True)}
    self._plan = SimpleNamespace(e2eAlerts=e2e)

  def __getitem__(self, key):
    return self._plan


class TestAlertBookmark:
  def test_each_alert_sends_user_bookmark_through_the_given_publisher(self):
    for kw in ({'red': True}, {'green': True}, {'lead_depart': True}):
      pm = FakePM()
      assert publish_alert_bookmark(SM(**kw), pm)
      assert pm.sent == ['userBookmark']

  def test_no_alert_no_bookmark(self):
    pm = FakePM()
    assert not publish_alert_bookmark(SM(), pm)
    assert pm.sent == []

  def test_alert_on_a_stale_message_is_not_bookmarked_twice(self):
    pm = FakePM()
    assert not publish_alert_bookmark(SM(updated=False, red=True), pm)
    assert pm.sent == []

  def test_button_bookmark_path_is_independent(self):
    """The bookmark button keeps its own send in the layouts' _on_bookmark_clicked on the same publisher: the alert path adds no second publisher."""
    pm = FakePM()
    for service in ('bookmarkButton', 'userBookmark'):  # as in MainLayout._on_bookmark_clicked
      pm.send(service, None)
    publish_alert_bookmark(SM(red=True), pm)
    assert pm.sent == ['bookmarkButton', 'userBookmark', 'userBookmark']
