"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.driver_profile import DEFAULT_PROFILE, WheelButton

GearShifter = structs.CarState.GearShifter

HOLD_TIME = DEFAULT_PROFILE.wheel_button.hold_s  # s, default hold (DriverProfile.wheel_button.hold_s); a short press keeps the button's own function
RELEASE_TIME = 0.5  # s released before a hold counts: > 2 SWRC_01 periods (~5 Hz), so a button held through card's restart never re-arms
STANDSTILL_SPEED = 0.1  # m/s
GENESIS_MODE_CARS = ("GENESIS_GV70_2ND_GEN",)  # the button meaning (SWRC_01 Audio_CustomSw3Sta) is only verified on this car


class GenesisModeButton:
  """GV70: a long press of the programmable steering-wheel button (carStateSP.customButton) in Park switches between openpilot and 'Genesis mode',
  openpilot's dashcam mode: card starts passive and never sets ControlsReady, so pandad stays in ELM327 with the harness relay released and the
  car's own SCC, LFA, HDA and FCA run while the comma records. It flips OpenpilotEnabledToggle and requests an onroad cycle (the settings toggle's
  own path: hardwared drops onroad for ~1 s and card restarts with the new value).

  One switch per hold: the press must start and reach the hold time in Park, at standstill, with openpilot not engaged; leaving Park, moving or
  engaging during the hold cancels it until the button is released. A new card (after the restart) only arms once the button has been released
  for RELEASE_TIME. Only with DriverProfile.wheel_button.long_press "genesis_mode"; the Park / standstill / not-engaged conditions are fixed."""
  def __init__(self, CP: structs.CarParams, params, cfg: WheelButton = DEFAULT_PROFILE.wheel_button):
    self.active = CP.carFingerprint in GENESIS_MODE_CARS and cfg.long_press == "genesis_mode"
    self.hold_time = cfg.hold_s
    self.params = params
    self.held_frames = 0
    self.released_frames = 0
    self.blocked = True  # this hold can no longer switch (done, a condition broke during it, or not yet released since card started)

  def update(self, pressed: bool, CS: structs.CarState, engaged: bool) -> bool:
    """Returns True on the update that switched the mode."""
    if not self.active:
      return False
    if not pressed:
      self.held_frames = 0
      self.released_frames += 1
      if self.released_frames * DT_CTRL >= RELEASE_TIME:
        self.blocked = False
      return False
    self.released_frames = 0

    if CS.gearShifter != GearShifter.park or CS.vEgo > STANDSTILL_SPEED or engaged:
      self.blocked = True
    if self.blocked:
      return False

    self.held_frames += 1
    if self.held_frames * DT_CTRL < self.hold_time:
      return False

    self.blocked = True
    # pilot -> Genesis: the old ControlsReady and CarParams survive the 1 s offroad window, and pandad could apply that CarParams (Hyundai safety,
    # relay closed) before manager clears it at the onroad transition. Without ControlsReady pandad waits for the new card, which never sets it
    # when passive. Harmless now: pandad is already configured for this session.
    self.params.remove("ControlsReady")
    self.params.put_bool("OpenpilotEnabledToggle", not self.params.get_bool("OpenpilotEnabledToggle"), block=True)
    self.params.put_bool("OnroadCycleRequested", True, block=True)
    return True
