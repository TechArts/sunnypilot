#!/usr/bin/env python3
"""Closed-loop study: GV70 approach to a stopped lead with the real planner, LongControl and a lossy actuator.

Prints, per (delivery, lag) row and approach speed: final gap, min gap, max (real - planner speed) over the last
8 m, min request and max |jerk| below 5 m/s, and the time from 1 m/s to standstill.
"""
import dataclasses

import numpy as np

from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant
from openpilot.sunnypilot.selfdrive.driver_profile import DEFAULT_PROFILE

ROWS = [(1.0, 0.0), (0.97, 0.2), (0.88, 0.6), (0.80, 0.7)]
SPEEDS = [5., 10., 15.]
LEAD_DISTANCE = 6.0 + 10.0 * 15.0 + 20.0  # far enough for the planner to settle at 15 m/s


def run(speed, delivery, lag, duration=60.0, profile=DEFAULT_PROFILE):
  kw = dict(car='gv70', use_long_control=True)
  d0 = LEAD_DISTANCE if speed >= 10 else 80.0
  plant = Plant(lead_relevancy=True, speed=speed, distance_lead=d0, actuator_delivery=delivery, actuator_lag=lag, profile=profile, **kw)
  gaps, vs, vps, reqs, ts_ = [], [], [], [], []
  while plant.current_time < duration:
    log = plant.step(0.0, 1.0, 50.)
    gaps.append(log['distance_lead'] - log['distance'])
    vs.append(log['speed'])
    vps.append(log['v_planner'])
    reqs.append(log['requested_acceleration'])
    ts_.append(plant.current_time)
    if log['speed'] == 0.0 and plant.current_time > 2.0:
      break
  gaps, vs, vps, reqs, ts_ = map(np.array, (gaps, vs, vps, reqs, ts_))
  dt = plant.ts
  last8 = gaps <= 8.0
  low = vs < 5.0
  jerk = np.abs(np.diff(reqs)) / dt
  below1 = np.where(vs < 1.0)[0]
  stopped = np.where(vs == 0.0)[0]
  t_stop = ts_[stopped[0]] - ts_[below1[0]] if len(stopped) and len(below1) else float('nan')
  return dict(final=gaps[-1], min=gaps.min(),
              drift=float(np.max(vs[last8] - vps[last8])) if last8.any() else float('nan'),
              minreq=float(reqs[low].min()) if low.any() else float('nan'),
              jerk=float(jerk[low[1:]].max()) if low[1:].any() else float('nan'), t1=t_stop)


VARIANTS = {  # (V_DESIRED_BOUND_SPEED, low_speed_decel_gain); None = the shipped value (the A+B row)
  'baseline': (0.0, 1.0),
  'A': (None, 1.0),
  'A+B': (None, None),
}


def study(variant='A+B'):
  import openpilot.selfdrive.controls.lib.longitudinal_planner as lp
  shipped_bound = lp.V_DESIRED_BOUND_SPEED
  bound, gain = VARIANTS[variant]
  lp.V_DESIRED_BOUND_SPEED = shipped_bound if bound is None else bound
  soft_stop = DEFAULT_PROFILE.soft_stop if gain is None else dataclasses.replace(DEFAULT_PROFILE.soft_stop, low_speed_decel_gain=gain)
  profile = dataclasses.replace(DEFAULT_PROFILE, soft_stop=soft_stop)
  out = {}
  try:
    print(f"== {variant}: bound speed {lp.V_DESIRED_BOUND_SPEED}, decel gain {soft_stop.low_speed_decel_gain}")
    print(f"{'deliv':>5} {'lag':>4} {'v0':>3} {'final':>6} {'min':>6} {'drift':>6} {'minreq':>7} {'jerk':>6} {'t1->0':>6}")
    for delivery, lag in ROWS:
      for v in SPEEDS:
        r = run(v, delivery, lag, profile=profile)
        out[(delivery, lag, v)] = r
        print(f"{delivery:5.2f} {lag:4.1f} {v:3.0f} {r['final']:6.2f} {r['min']:6.2f} {r['drift']:6.2f} "
              f"{r['minreq']:7.2f} {r['jerk']:6.2f} {r['t1']:6.1f}")
  finally:
    lp.V_DESIRED_BOUND_SPEED = shipped_bound
  return out


if __name__ == "__main__":
  import sys
  for name in (sys.argv[1:] or list(VARIANTS)):
    study(name)
