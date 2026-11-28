#!/usr/bin/env python3
"""Open-loop replay of logged GV70 drives through the PORT's LongitudinalPlanner.

Study script. For every logged modelV2 frame the inputs the planner reads are copied (field by field, explicit map)
from a log written by the installed build into port message builders, the port planner is stepped, and its
aTarget / source / shouldStop are compared with the logged longitudinalPlan of the installed build.

OPEN LOOP: the car's v/a are the logged ones (driven by the installed build / the driver), not what the port would
have commanded. Differences after the point where the plans diverge are therefore "what the port would ask for given
that state", not a closed-loop outcome.

Run:  python replay_planner_gv70.py [window ...]    (windows: stop, red, launch1, launch2; default all)
Old logs are read with xseg from RedLightAlert/tools/analysis (installed-build schema); run with an interpreter that
has capnp + zstandard.
"""
import os
import sys
import collections

import numpy as np

XSEG_DIR = os.path.expanduser("~/Library/Mobile Documents/com~apple~CloudDocs/Personal/Notes/Personal/RedLightAlert/tools/analysis")
ROUTES = os.path.expanduser("~/workspace/comma/routes")
# The installed-build schema and the port schema cannot be loaded into one process (duplicate capnp ids), so logs are
# extracted to plain dicts by a subprocess run with the old-schema interpreter and cached as pickles.
OLD_PY = os.path.expanduser("~/workspace/comma/opendbc/.venv/bin/python")
CACHE = "/tmp/replay_planner_gv70"
if "--extract" in sys.argv:
  sys.path.insert(0, XSEG_DIR)
  import xseg  # noqa: E402
else:
  import pickle
  import subprocess
  import openpilot.cereal.messaging as messaging  # noqa: E402
  from opendbc.car.hyundai.values import CAR  # noqa: E402
  from opendbc.car.hyundai.interface import CarInterface  # noqa: E402
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner  # noqa: E402

MPH = 2.23694
SEG_LEN = 60.0
WARMUP = 10.0

# name -> (route dir prefix, start s, end s)
WINDOWS = {
  "stop": ("0000006f--668aed956d", 130.0, 145.0),
  "red": ("00000070--56c11c9d5a", 365.0, 377.0),
  "launch1": ("00000076--d430fb4042", 1203.0, 1210.0),
  "launch2": ("0000006f--668aed956d", 236.0, 243.0),
  "launch3": ("0000007d--77bcff9869", 1244.0, 1258.0),
}

SERVICES = ("carState", "carControl", "carStateSP", "controlsState", "selfdriveState", "radarState", "modelV2", "longitudinalPlan")


def seg_path(route, seg):
  return f"{ROUTES}/{route}--{seg}/rlog.zst"


def route_t0(route):
  for e in xseg.events(seg_path(route, 0)):
    if e.which() == "carState":
      return e.logMonoTime / 1e9
  raise RuntimeError("no carState in seg 0")


def lead_dict(l):
  # the shared xseg schema (tools/analysis/cereal, the port's) names the lead flag `present` and has no fcw; kept as "status"
  d = {k: getattr(l, k) for k in ("dRel", "yRel", "vRel", "vLead", "vLeadK", "aLeadK", "aLeadTau", "modelProb", "radar")}
  d["status"] = l.present
  return d


def extract(w, e):
  """Logged event -> plain dict of exactly the fields the port planner (and SP helpers) read."""
  if w == "carState":
    c = e.carState
    return dict(vEgo=c.vEgo, aEgo=c.aEgo, vCruise=c.vCruise, vCruiseCluster=c.vCruiseCluster, standstill=c.standstill,
                gasPressed=c.gasPressed, brakePressed=c.brakePressed)
  if w == "carControl":
    c = e.carControl
    return dict(enabled=c.enabled, override=c.cruiseControl.override, longActive=c.longActive)
  if w == "carStateSP":
    return dict(speedLimit=e.carStateSP.speedLimit)
  if w == "controlsState":
    c = e.controlsState
    return dict(forceDecel=c.forceDecel, longControlState=str(c.longControlState), curvature=c.curvature)
  if w == "selfdriveState":
    s = e.selfdriveState
    # the shared xseg schema (tools/analysis/cereal) keeps experimentalMode in the deprecated group
    return dict(enabled=s.enabled, personality=str(s.personality), experimentalMode=s.deprecated.experimentalMode)
  if w == "radarState":
    r = e.radarState
    return dict(leadOne=lead_dict(r.leadOne), leadTwo=lead_dict(r.leadTwo))
  if w == "modelV2":
    m = e.modelV2
    return dict(px=list(m.position.x), py=list(m.position.y), pz=list(m.position.z), vx=list(m.velocity.x),
                ax=list(m.acceleration.x), orz=list(m.orientationRate.z), des_a=m.action.desiredAcceleration,
                should_stop=m.action.shouldStop, des_curv=m.action.desiredCurvature)
  if w == "longitudinalPlan":
    p = e.longitudinalPlan
    return dict(aTarget=p.aTarget, source=str(p.longitudinalPlanSource), shouldStop=p.shouldStop, allowThrottle=p.allowThrottle,
                modelMonoTime=p.modelMonoTime)
  return None


def load_window(name):
  """Windows added at run time (study scripts) pass their bounds to the extractor."""
  p = f"{CACHE}_{name}.pkl"
  if not os.path.exists(p):
    route, ts, te = WINDOWS[name]
    subprocess.run([OLD_PY, os.path.abspath(__file__), "--extract", name, route, str(ts), str(te)], check=True)
  with open(p, "rb") as f:
    return pickle.load(f)


def extract_window(route, t_start, t_end):
  """Time-ordered list of (t_route, service, dict) covering [t_start-WARMUP, t_end]."""
  t0 = route_t0(route)
  lo = t_start - WARMUP
  out = []
  for seg in range(max(0, int(lo // SEG_LEN) - 1), int(t_end // SEG_LEN) + 2):
    p = seg_path(route, seg)
    if not os.path.exists(p):
      continue
    for e in xseg.events(p):
      w = e.which()
      if w not in SERVICES:
        continue
      t = e.logMonoTime / 1e9 - t0
      if t < lo or t > t_end:
        continue
      out.append((t, w, extract(w, e), e.logMonoTime))
  out.sort(key=lambda x: x[0])
  return out


class FakeSM:
  """Just enough of SubMaster for the planner: item access, updated/valid/alive/logMonoTime, all_checks."""
  def __init__(self, msgs, mono):
    self.msgs = msgs
    self.logMonoTime = mono
    self.recv_time = collections.defaultdict(float)  # 0: the speed-limit resolver takes the (empty) map data as stale
    self.updated = collections.defaultdict(lambda: True)
    self.valid = collections.defaultdict(lambda: True)
    self.alive = collections.defaultdict(lambda: True)
    self.frame = 0

  def __getitem__(self, k):
    return self.msgs[k]

  def all_checks(self, *a, **k):
    return True


def fill_lead(dst, d):
  for k, v in d.items():
    if k == "status":
      dst.present = v
    else:
      setattr(dst, k, v)


def build_msgs(st):
  """Port message builders from the latest logged dicts (explicit field map)."""
  m = {}
  cs = messaging.new_message("carState").carState
  c = st["carState"]
  cs.vEgo, cs.aEgo, cs.vCruise, cs.vCruiseCluster = c["vEgo"], c["aEgo"], c["vCruise"], c["vCruiseCluster"]
  cs.standstill, cs.gasPressed, cs.brakePressed = c["standstill"], c["gasPressed"], c["brakePressed"]
  m["carState"] = cs

  cc = messaging.new_message("carControl").carControl
  cc.enabled = st["carControl"]["enabled"]
  cc.cruiseControl.override = st["carControl"]["override"]
  m["carControl"] = cc

  sp = messaging.new_message("carStateSP").carStateSP
  sp.speedLimit = st["carStateSP"]["speedLimit"]
  # cameraStopSign does not exist in the installed schema: left False (only drives an alert, not aTarget)
  m["carStateSP"] = sp

  ct = messaging.new_message("controlsState").controlsState
  ct.forceDecel = st["controlsState"]["forceDecel"]
  ct.longControlState = st["controlsState"]["longControlState"]
  ct.curvature = st["controlsState"]["curvature"]
  m["controlsState"] = ct

  sd = messaging.new_message("selfdriveState").selfdriveState
  sd.enabled = st["selfdriveState"]["enabled"]
  sd.personality = st["selfdriveState"]["personality"]
  m["selfdriveState"] = sd

  rs = messaging.new_message("radarState").radarState
  fill_lead(rs.leadOne, st["radarState"]["leadOne"])
  fill_lead(rs.leadTwo, st["radarState"]["leadTwo"])
  m["radarState"] = rs

  mv = messaging.new_message("modelV2").modelV2
  g = st["modelV2"]
  mv.position.x, mv.position.y, mv.position.z = g["px"], g["py"], g["pz"]
  mv.velocity.x = g["vx"]
  mv.acceleration.x = g["ax"]
  mv.orientationRate.z = g["orz"]
  mv.action.desiredAcceleration = g["des_a"]
  mv.action.shouldStop = g["should_stop"]
  mv.action.desiredCurvature = g["des_curv"]
  m["modelV2"] = mv

  m["liveMapDataSP"] = messaging.new_message("liveMapDataSP").liveMapDataSP
  m["gpsLocation"] = messaging.new_message("gpsLocation").gpsLocation
  m["gpsLocationExternal"] = m["gpsLocation"]
  return m


def make_planner(v0, a0):
  CP = CarInterface.get_non_essential_params(CAR.GENESIS_GV70_2ND_GEN)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.GENESIS_GV70_2ND_GEN)
  CP.openpilotLongitudinalControl = True
  return LongitudinalPlanner(CP, CP_SP, init_v=v0, init_a=a0), CP


def replay(name):
  route, ts, te = WINDOWS[name]
  ev = load_window(name)
  st = {}
  planner = None
  rows = []
  for t, w, d, mono in ev:
    if w != "modelV2":
      st[w] = d
      continue
    st[w] = d
    if not all(s in st for s in SERVICES):
      continue
    msgs = build_msgs(st)
    if planner is None:
      planner, _ = make_planner(st["carState"]["vEgo"], st["carState"]["aEgo"])
    sm = FakeSM(msgs, collections.defaultdict(lambda: mono))
    planner.update(sm)
    lg = st["longitudinalPlan"]
    lead = st["radarState"]["leadOne"]
    rows.append(dict(t=t, v=st["carState"]["vEgo"], a=st["carState"]["aEgo"], brake=st["carState"]["brakePressed"],
                     gas=st["carState"]["gasPressed"], en=st["carControl"]["enabled"], des=d["des_a"], ss=d["should_stop"],
                     lead=lead["dRel"] if lead["status"] else None, vl=lead["vLead"] if lead["status"] else None,
                     log_a=lg["aTarget"], log_src=lg["source"].replace("LongitudinalPlanSource.", ""), log_ss=lg["shouldStop"],
                     port_a=float(planner.output_a_target), port_src=SRC_NAMES.get(int(planner.mpc.source), str(planner.mpc.source)),
                     port_ss=bool(planner.output_should_stop), sp_src=int(planner.source)))
  return [r for r in rows if r["t"] >= ts], ts, te


SRC_NAMES = {0: "cruise", 1: "lead0", 2: "lead1", 3: "lead2", 4: "e2e"}  # log.LongitudinalPlan.LongitudinalPlanSource


def mmss(t):
  return f"{int(t // 60)}:{t % 60:04.1f}"


def report(name):
  rows, ts, te = replay(name)
  print(f"\n=== {name}  {WINDOWS[name][0]}  {mmss(ts)}-{mmss(te)}  (open loop: v/a/brake are the logged car) ===")
  print(f"{'t':>7} {'v m/s':>6} {'mph':>5} {'a_ego':>6} {'en':>2} {'brk':>3} {'lead m':>7} {'vLead':>6} {'modelA':>6} {'mss':>3} | {'log_aT':>6} {'log_src':>8} {'lss':>3} | {'port_aT':>7} {'port_src':>8} {'pss':>3}")
  nxt = ts
  for r in rows:
    if r["t"] + 1e-6 < nxt:
      continue
    nxt += 0.5
    ld = f"{r['lead']:7.1f}" if r["lead"] is not None else "      -"
    vl = f"{r['vl']:6.1f}" if r["vl"] is not None else "     -"
    print(f"{mmss(r['t']):>7} {r['v']:6.2f} {r['v'] * MPH:5.1f} {r['a']:6.2f} {int(r['en']):>2} {int(r['brake']):>3} {ld} {vl} {r['des']:6.2f} {int(r['ss']):>3} | "
          f"{r['log_a']:6.2f} {r['log_src']:>8} {int(r['log_ss']):>3} | {r['port_a']:7.2f} {r['port_src']:>8} {int(r['port_ss']):>3}")
  # stats only while the installed build was engaged: once carControl.enabled drops (driver braked) the planner resets
  # to the car's actual accel, which says nothing about what the plan would have been
  eng = [r for r in rows if r["en"]]
  la = np.array([r["log_a"] for r in eng])
  pa = np.array([r["port_a"] for r in eng])
  tt = np.array([r["t"] for r in eng])
  print(f"engaged until {mmss(tt[-1])} (rows after that are driver-controlled, not comparable)")
  print(f"min aTarget: logged {la.min():.2f} @ {mmss(tt[la.argmin()])}   port {pa.min():.2f} @ {mmss(tt[pa.argmin()])}")
  print(f"max aTarget: logged {la.max():.2f} @ {mmss(tt[la.argmax()])}   port {pa.max():.2f} @ {mmss(tt[pa.argmax()])}")
  for thr in (-0.5, -1.0, -2.0):
    f = lambda arr: mmss(tt[np.argmax(arr < thr)]) if (arr < thr).any() else "never"
    print(f"first aTarget < {thr:+.1f}: logged {f(la)}   port {f(pa)}")
  for thr in (0.5, 1.0):
    f = lambda arr: mmss(tt[np.argmax(arr > thr)]) if (arr > thr).any() else "never"
    print(f"first aTarget > {thr:+.1f}: logged {f(la)}   port {f(pa)}")
  lss = [r["t"] for r in eng if r["log_ss"]]
  pss = [r["t"] for r in eng if r["port_ss"]]
  print(f"shouldStop first: logged {mmss(lss[0]) if lss else 'never'}   port {mmss(pss[0]) if pss else 'never'}")
  print(f"mean(port-logged) engaged: {np.mean(pa - la):+.2f}   mean|diff| {np.mean(np.abs(pa - la)):.2f}")


if __name__ == "__main__":
  if "--extract" in sys.argv:
    import pickle
    i = sys.argv.index("--extract")
    n, route, ts, te = sys.argv[i + 1], sys.argv[i + 2], float(sys.argv[i + 3]), float(sys.argv[i + 4])
    with open(f"{CACHE}_{n}.pkl", "wb") as f:
      pickle.dump(extract_window(route, ts, te), f)
  else:
    for n in (sys.argv[1:] or list(WINDOWS)):
      report(n)
