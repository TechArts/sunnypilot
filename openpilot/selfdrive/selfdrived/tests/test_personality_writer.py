from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD

KEY = "LongitudinalPersonality"


class AsyncPersonalityParams:
  """Params whose non-blocking put of LongitudinalPersonality lands only when land() is called, like Params.put's writer thread."""

  def __init__(self, params, stored):
    self.params = params
    self.stored = stored
    self.pending: list[int] = []

  def __getattr__(self, name):
    return getattr(self.params, name)

  def get(self, key, *args, **kwargs):
    return self.stored if key == KEY else self.params.get(key, *args, **kwargs)

  def put(self, key, value, block=False):
    if key != KEY:
      return self.params.put(key, value)
    if block:
      self.stored = value
    else:
      self.pending.append(value)

  def land(self):
    if self.pending:
      self.stored = self.pending.pop(0)


class OneIteration:
  def __init__(self):
    self.calls = 0

  def is_set(self):
    self.calls += 1
    return self.calls > 1


class TestPersonalityWriter(OpenpilotTestCase):
  def setUp(self):
    CP = car.CarParams.new_message(brand="hyundai", openpilotLongitudinalControl=True)
    self.sd = SelfdriveD(CP=CP, CP_SP=custom.CarParamsSP.new_message())
    self.sd.initialized = True
    self.sd.mads.read_params = lambda: None
    self.params = AsyncPersonalityParams(self.sd.params, self.sd.personality)
    self.sd.params = self.params

  def press(self):
    CS = car.CarState.new_message(canValid=False, buttonEvents=[{"type": "gapAdjustCruise", "pressed": False}])
    self.sd.update_events(CS)

  def params_read(self):
    self.sd.params_thread(OneIteration())

  def test_press_survives_a_read_taken_before_the_write_lands(self):
    # GV70 route 6b: the 10 Hz read returned the old value and put it back (1 -> 0 -> 1)
    assert self.sd.personality == 1
    self.press()
    assert self.sd.personality == 0
    self.params_read()
    assert self.sd.personality == 0
    self.params.land()
    self.params_read()
    assert self.sd.personality == 0
    assert self.params.stored == 0

  def test_presses_survive_a_write_backlog(self):
    # three presses before any write lands: the file still holds the start value, which equals the new personality (1 -> 0 -> 2 -> 1)
    for expected in (0, 2, 1):
      self.press()
      assert self.sd.personality == expected
    for _ in range(4):
      self.params_read()
      assert self.sd.personality == 1
      self.params.land()
    self.params_read()
    assert self.sd.personality == 1
    assert self.params.stored == 1

  def test_ui_change_is_adopted(self):
    self.params.stored = 2
    self.params_read()
    assert self.sd.personality == 2
    self.press()
    self.params_read()
    self.params.land()
    self.params_read()
    assert self.sd.personality == 1
    assert self.params.stored == 1
