#!/usr/bin/env python3
"""Closed-loop study: GV70 launch from a stop behind a car that creeps forward and stops again (route 7d 20:48-20:57), and normal
launches behind a car that drives away, with the real planner, LongControl and a lossy actuator.

Two parts, both printed:
- synthetic: the maneuver plant (fake model) with scripted lead speed profiles;
- logged: every risky logged launch (LOGGED), run closed loop: the logged model, lead speed and set speed, but the car's own speed and
  the gap come from the simulation (the lead moves at its logged speed from the logged gap at the window start).
Prints the smallest gap, the gap at the end, the highest request and speed, and the time to 3 m/s. Result 2026-11-25: the port's
planner never closes below the start gap (synthetic) and keeps 3.8-9.1 m on the logged launches the old build closed to 1.45-2.8 m,
so a dedicated launch guard was not needed (designed, simulated, inert in every case).
"""
import copy

import numpy as np

from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant

ROWS = [(1.0, 0.0), (0.88, 0.6), (0.80, 0.7)]  # (decel delivery, lag s): ideal, the GV70's measured worst, the soft-stop stress row


def creep(gap, v_peak, a_up=1.2, hold=0.5, a_down=-1.5, t_start=2.0):
  """The car ahead starts at t_start, accelerates to v_peak, holds it, then brakes to a stop (7d: 4.25 m, 2.6 m/s, -1.5 m/s^2)."""
  t1 = t_start + v_peak / a_up
  t2 = t1 + hold
  t3 = t2 + v_peak / -a_down
  return gap, [0.0, t_start, t1, t2, t3, 60.0], [0.0, 0.0, v_peak, v_peak, 0.0, 0.0]


def drive_away(gap, a_up=1.5, v_end=12.0, t_start=2.0):
  t1 = t_start + v_end / a_up
  return gap, [0.0, t_start, t1, 60.0], [0.0, 0.0, v_end, v_end]


SCENARIOS = {
  'creep 4.3 m, 2.6 m/s, -1.5 (7d)': creep(4.25, 2.6),
  'creep 4.3 m, 4.0 m/s, -1.5': creep(4.25, 4.0),
  'creep 4.3 m, 2.6 m/s, -2.5': creep(4.25, 2.6, a_down=-2.5),
  'creep 3.5 m, 1.5 m/s, -1.5': creep(3.5, 1.5),
  'creep 6.0 m, 4.0 m/s, -2.5': creep(6.0, 4.0, a_down=-2.5),
  'away 4.3 m, 1.5 m/s2': drive_away(4.25),
  'away 6.0 m, 1.0 m/s2': drive_away(6.0, a_up=1.0),
  'away 8.0 m, 2.0 m/s2': drive_away(8.0, a_up=2.0),
}

# name -> (route, start, end): every logged launch under openpilot speed from a stop with a car < 10 m ahead that closed to under 3.5 m
# or ended with the driver braking (drive_feel.py launches, routes 0a-9f); the sim starts stopped at the start time
LOGGED = {
  '4a 5:51 (driver braked)': ('0000004a--60b9fbafcb', 351.4, 366.0),
  '66 7:07 (2.8 m)': ('00000066--f75fba8f17', 427.5, 442.0),
  '7d 19:44 (1.55 m)': ('0000007d--77bcff9869', 1184.3, 1199.0),
  '7d 20:48 (1.45 m, driver braked)': ('0000007d--77bcff9869', 1248.1, 1262.0),
  '7d 21:53 (2.25 m)': ('0000007d--77bcff9869', 1312.7, 1327.0),
}


def summary(gaps, vs, reqs, ts_):
  gaps, vs, reqs, ts_ = map(np.array, (gaps, vs, reqs, ts_))
  fast = np.where(vs >= 3.0)[0]
  return dict(min=float(gaps.min()), end=float(gaps[-1]), maxreq=float(reqs.max()), v_max=float(vs.max()),
              t3=float(ts_[fast[0]] - ts_[0]) if len(fast) else float('nan'))


def run(scenario, delivery, lag, duration=25.0, v_cruise=36.0):
  gap0, bps, vls = SCENARIOS[scenario]
  plant = Plant(lead_relevancy=True, speed=0.0, distance_lead=gap0, actuator_delivery=delivery, actuator_lag=lag,
                car='gv70', use_long_control=True)
  gaps, vs, reqs, ts_ = [], [], [], []
  while plant.current_time < duration:
    log = plant.step(float(np.interp(plant.current_time, bps, vls)), 1.0, v_cruise)
    gaps.append(log['distance_lead'] - log['distance'])
    vs.append(log['speed'])
    reqs.append(log['requested_acceleration'])
    ts_.append(plant.current_time)
  return summary(gaps, vs, reqs, ts_)


def run_logged(route, ts, t_end, delivery, lag):
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.values import CAR
  from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
  import openpilot.cereal.messaging as messaging
  from openpilot.common.realtime import DT_CTRL
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.test.longitudinal_maneuvers import replay_planner_gv70 as rp
  window = f"study_{route[:8]}_{int(ts)}"
  rp.WINDOWS[window] = (route, ts, t_end)
  st, planner, lc = {}, None, None
  x = v = a = 0.0
  x_lead = t_prev = None
  gaps, vs, reqs, ts_ = [], [], [], []
  for t, w, d, mono in rp.load_window(window):
    st[w] = d
    if w != "modelV2" or not all(s in st for s in rp.SERVICES) or t < ts or t > t_end:
      continue
    lead = st["radarState"]["leadOne"]
    if planner is None:
      if not lead["status"]:
        continue
      planner, CP = rp.make_planner(0.0, 0.0)
      lc = LongControl(CP, CarInterface.get_non_essential_params_sp(CP, CAR.GENESIS_GV70_2ND_GEN))
      x_lead, t_prev = lead["dRel"], t
    dt = t - t_prev
    if dt <= 0.0:
      continue
    t_prev = t
    x_lead += max(lead["vLead"], 0.0) * dt
    s = copy.deepcopy(st)
    s["carState"].update(vEgo=float(v), aEgo=float(a), standstill=bool(v < 0.01), gasPressed=False, brakePressed=False)
    s["carControl"].update(enabled=True, override=False)
    s["selfdriveState"]["enabled"] = True
    s["controlsState"]["longControlState"] = "pid"
    for k in ("leadOne", "leadTwo"):
      s["radarState"][k].update(dRel=float(x_lead - x), vRel=float(lead["vLead"] - v), status=True)
    planner.update(rp.FakeSM(rp.build_msgs(s), rp.collections.defaultdict(lambda: mono)))
    CS = messaging.new_message('carState').carState
    CS.vEgo, CS.aEgo, CS.standstill = float(v), float(a), bool(v < 0.01)
    req = 0.0
    for _ in range(max(round(dt / DT_CTRL), 1)):
      req = lc.update(True, CS, planner.output_a_target, planner.output_should_stop, (ACCEL_MIN, ACCEL_MAX))
    a += (req * (delivery if req < 0 else 1.0) - a) * dt / (lag + dt)
    v = max(v + a * dt, 0.0)
    x += v * dt
    gaps.append(x_lead - x)
    vs.append(v)
    reqs.append(req)
    ts_.append(t)
  return summary(gaps, vs, reqs, ts_)


def study(logged=False):
  out = {}
  print(f"== {'logged windows, closed loop' if logged else 'synthetic lead profiles'}")
  print(f"{'deliv':>5} {'lag':>4} {'scenario':<34} {'min':>6} {'end':>6} {'maxreq':>7} {'vmax':>5} {'t 3m/s':>6}")
  for delivery, lag in ROWS:
    for name in (LOGGED if logged else SCENARIOS):
      r = run_logged(*LOGGED[name], delivery, lag) if logged else run(name, delivery, lag)
      out[(delivery, lag, name)] = r
      print(f"{delivery:5.2f} {lag:4.1f} {name:<34} {r['min']:6.2f} {r['end']:6.2f} {r['maxreq']:7.2f} {r['v_max']:5.2f} {r['t3']:6.1f}")
  return out


if __name__ == "__main__":
  study()
  study(logged=True)
