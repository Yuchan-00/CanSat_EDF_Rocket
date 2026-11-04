"""Edit decisions (pure): flight events from a log, the automatic shot list, and the video-time -> sim-time
remapping with eased slow-motion windows.

A shot is a contiguous piece of sim time [t0, t1] filmed by one camera rig ('wide', 'spectator',
'orbit', 'crane', 'closeup', 'turntable').  Time inside a shot is continuous (the footage a real camera
would have recorded); between shots it may jump (an edit cut).  A slow-motion window plays the sim time
inside [a, b] at ``factor`` x real time (e.g. 0.25 = a 120 fps camera played at 30 fps), ramped in and out
with a smoothstep over ``ramp_s`` of sim time so the speed never jumps.

Invariant: everything here is a deterministic function of the logged arrays and the explicit options.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from typing import Any, Sequence

import numpy as np

CAM_KINDS = ("wide", "spectator", "orbit", "crane", "closeup", "turntable")
STATE_POWERED, STATE_ABORT = 2, 5
PHASE_HOLD, PHASE_LAND, PHASE_LANDED = 3, 4, 5

KIND_KO = {"start": "시작", "arm": "시동 준비", "motor_on": "모터 가동", "liftoff": "이륙", "hover": "호버링",
           "cmd": "지상 명령", "gust": "돌풍", "push": "밀기", "land": "착륙 시작", "touchdown": "착지",
           "abort": "비상정지", "end": "종료"}


@dataclass
class Event:
    t: float
    kind: str
    label: str = ""
    value: float = float("nan")
    t_end: float = float("nan")


@dataclass
class Slow:
    a: float
    b: float
    factor: float


@dataclass
class Shot:
    t0: float
    t1: float
    cam: str
    slow: list[Slow] = field(default_factory=list)
    note: str = ""
    params: dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> dict[str, Any]:
        d = asdict(self)
        return d


def shot_list_display(obj: Any) -> dict[str, str]:
    """Display texts of a shot-list JSON in its dict form ({"title_ko", "subtitle_ko", "shots": [...]}); {} for the
    plain list form.  The renderer shows these instead of the log's meta title (art review round 2: the synthetic log's
    own title said the pushes were by hand while every shot shows the pole)."""
    if isinstance(obj, dict):
        return {k: str(obj[k]) for k in ("title_ko", "subtitle_ko") if obj.get(k)}
    return {}


def shots_from_json(obj: Any) -> list[Shot]:
    """A shot list from JSON: [{"t0":.., "t1":.., "cam":.., "slow":[{"a":..,"b":..,"factor":..}], ...}] or the dict
    form {"title_ko": .., "subtitle_ko": .., "shots": [...]}."""
    if isinstance(obj, dict):
        obj = obj.get("shots", [])
    out = []
    for s in obj:
        if s.get("cam") not in CAM_KINDS:
            raise ValueError(f"unknown camera {s.get('cam')!r} (known: {CAM_KINDS})")
        out.append(Shot(float(s["t0"]), float(s["t1"]), str(s["cam"]),
                        [Slow(float(w["a"]), float(w["b"]), float(w["factor"])) for w in s.get("slow", [])],
                        str(s.get("note", "")), dict(s.get("params", {}))))
    return out


# --------------------------------------------------------------------------- #
# events
# --------------------------------------------------------------------------- #
def _rising(t: np.ndarray, flag: np.ndarray) -> list[tuple[float, float, int, int]]:
    """(t_start, t_end, i_start, i_end_exclusive) of every run of True."""
    a = np.asarray(flag, dtype=bool).astype(int)
    e = np.diff(np.concatenate([[0], a, [0]]))
    s, f = np.flatnonzero(e == 1), np.flatnonzero(e == -1)
    return [(float(t[i]), float(t[min(j, t.size - 1)]), int(i), int(j)) for i, j in zip(s, f)]


def detect_events(log: Any) -> list[Event]:
    """Flight events from logged arrays only (state / phase codes, commands, gust / push flags, meta times)."""
    t = log.t
    ev: list[Event] = [Event(float(t[0]), "start", KIND_KO["start"])]
    st = np.asarray(log.state, dtype=int)
    ph = np.asarray(log.phase, dtype=int)
    i_arm = np.flatnonzero(st >= 1)
    if i_arm.size:
        ev.append(Event(float(t[i_arm[0]]), "arm", KIND_KO["arm"]))
    i_pw = np.flatnonzero(st == STATE_POWERED)
    t_pw = float(t[i_pw[0]]) if i_pw.size else None
    if t_pw is not None:
        ev.append(Event(t_pw, "motor_on", KIND_KO["motor_on"]))
    z0 = float(log.pos[0, 2])
    t_lift = log.meta.get("t_liftoff")
    if t_lift is None and t_pw is not None:
        up = np.flatnonzero((t >= t_pw) & (log.pos[:, 2] - z0 > 0.03))
        t_lift = float(t[up[0]]) if up.size else None
    if t_lift is None and float(np.max(log.pos[:, 2]) - z0) > 0.3 and z0 < 0.05:
        up = np.flatnonzero(log.pos[:, 2] - z0 > 0.03)
        t_lift = float(t[up[0]]) if up.size else None
    if t_lift is not None:
        ev.append(Event(float(t_lift), "liftoff", KIND_KO["liftoff"]))
    i_h = np.flatnonzero(ph == PHASE_HOLD)
    if i_h.size:
        ev.append(Event(float(t[i_h[0]]), "hover", KIND_KO["hover"], value=float(log.pos[i_h[0], 2])))
    for tc, name in zip(log.cmd_t, log.cmd_name):
        ev.append(Event(float(tc), "cmd", str(name)))
    for (a, b, i, j) in _rising(t, log.gust_active):
        w = np.linalg.norm(log.wind[i:max(j, i + 1)], axis=1)
        ev.append(Event(a, "gust", KIND_KO["gust"], value=float(np.max(w)) if w.size else float("nan"), t_end=b))
    if log.push_active is not None and log.push_force is not None:
        for (a, b, i, j) in _rising(t, log.push_active):
            f = np.linalg.norm(log.push_force[i:max(j, i + 1)], axis=1)
            ev.append(Event(a, "push", KIND_KO["push"], value=float(np.max(f)), t_end=b))
    i_l = np.flatnonzero(ph == PHASE_LAND)
    if i_l.size:
        ev.append(Event(float(t[i_l[0]]), "land", KIND_KO["land"]))
    t_td = log.meta.get("t_touchdown")
    if t_td is None:
        i_td = np.flatnonzero(ph == PHASE_LANDED)
        if i_td.size:
            t_td = float(t[i_td[0]])
        elif t_lift is not None:
            after = np.flatnonzero((t > t_lift + 1.0) & (st != STATE_POWERED))
            t_td = float(t[after[0]]) if after.size else None
    if t_td is not None:
        ev.append(Event(float(t_td), "touchdown", KIND_KO["touchdown"]))
    i_ab = np.flatnonzero(st == STATE_ABORT)
    if i_ab.size:
        ev.append(Event(float(t[i_ab[0]]), "abort", KIND_KO["abort"]))
    ev.append(Event(float(t[-1]), "end", KIND_KO["end"]))
    ev.sort(key=lambda e: (e.t, e.kind))
    return ev


def first(events: Sequence[Event], kind: str) -> Event | None:
    for e in events:
        if e.kind == kind:
            return e
    return None


# --------------------------------------------------------------------------- #
# automatic shot list
# --------------------------------------------------------------------------- #
def auto_shots(events: Sequence[Event], t_first: float, t_last: float, indoor: bool = False,
               max_disturbances: int = 4) -> list[Shot]:
    """Pad -> spool -> lift-off -> climb -> hover -> disturbances (gust / push) -> correction -> landing -> end.

    Each beat is a sim-time window around its event with a priority (disturbances and lift-off / landing first,
    then the hover establishing shot, then pad / climb / command shots).  ``place`` puts beats into the free sim
    time in priority order, trimming a beat to the free piece around its key time and dropping it when less than
    1.2 s remain, so the edit never runs time backwards and an event never loses its shot to a filler.
    Disturbances closer than 2.5 s merge; at most ``max_disturbances`` (the strongest) get shots.
    """
    lift = first(events, "liftoff")
    td = first(events, "touchdown")
    hov = first(events, "hover")
    abort = first(events, "abort")
    dist = [e for e in events if e.kind in ("gust", "push")]
    beats: list[tuple[int, float, Shot]] = []           # (priority: lower first, key time, shot)
    if lift is not None:
        tl = lift.t
        pre = tl - 0.6 - t_first
        if pre >= 6.0:
            beats.append((3, tl - 5.0, Shot(tl - 7.5, tl - 4.0, "wide", note="pad")))
            beats.append((2, tl - 2.0, Shot(tl - 4.0, tl - 0.6, "closeup", note="spool", params={"mode": "pad"})))
        elif pre >= 1.2:
            mid = t_first + 0.45 * pre
            if mid - t_first >= 1.2:
                beats.append((3, t_first, Shot(t_first, mid, "wide", note="pad")))
            beats.append((2, tl - 1.0, Shot(mid if mid - t_first >= 1.2 else t_first, tl - 0.6, "closeup", note="spool",
                                            params={"mode": "pad"})))
        beats.append((0, tl, Shot(tl - 0.6, tl + 3.2, "spectator", [Slow(tl - 0.25, tl + 0.9, 0.4)], note="liftoff")))
        t_hov = hov.t if hov is not None else tl + 4.0
        beats.append((3, t_hov, Shot(max(tl + 3.2, t_hov - 4.0), t_hov + 1.0, "crane" if not indoor else "orbit", note="climb")))
        beats.append((1, t_hov + 2.0, Shot(t_hov + 1.0, t_hov + 4.5, "wide", note="hover")))
    else:
        beats.append((1, t_first, Shot(t_first, min(t_last, t_first + 4.0), "wide", note="opening")))
    groups: list[list[Event]] = []
    for e in dist:
        if groups and e.t - max(x.t_end if np.isfinite(x.t_end) else x.t for x in groups[-1]) < 2.5:
            groups[-1].append(e)
        else:
            groups.append([e])
    if len(groups) > max_disturbances:
        groups = sorted(sorted(groups, key=lambda g: -max(x.value if np.isfinite(x.value) else 0.0 for x in g))
                        [:max_disturbances], key=lambda g: g[0].t)
    rot = ("closeup", "spectator", "orbit") if (indoor or (groups and groups[0][0].kind == "push")) else ("spectator", "closeup", "orbit")
    per_kind: dict[str, int] = {}
    for k, g in enumerate(groups):
        per_kind[g[0].kind] = per_kind.get(g[0].kind, 0) + 1
        a = g[0].t
        b = max(x.t_end if np.isfinite(x.t_end) else x.t + 0.5 for x in g)
        slow_b = a + min(1.4, (b - a) + 0.8)
        tail = b + 3.2
        if k + 1 < len(groups):                       # leave the next disturbance its 1.3 s pre-roll
            tail = max(b + 1.5, min(tail, groups[k + 1][0].t - 1.3))
        beats.append((0, a, Shot(a - 1.3, tail, rot[k % 3], [Slow(a - 0.15, slow_b, 0.25)], note=f"{g[0].kind}#{per_kind[g[0].kind]}",
                                 params={"event_t": a, "event_end": b, "kind": g[0].kind})))
    for e in events:
        if e.kind == "cmd" and e.label in ("UP", "DOWN") and hov is not None and e.t > hov.t:
            beats.append((4, e.t, Shot(e.t - 0.5, e.t + 4.5, "orbit" if indoor else "crane", note=f"cmd {e.label}")))
            break
    if abort is not None:
        beats.append((0, abort.t, Shot(abort.t - 1.5, min(t_last, abort.t + 4.0), "spectator",
                                       [Slow(abort.t - 0.2, abort.t + 1.2, 0.5)], note="abort")))
    elif td is not None:
        beats.append((0, td.t, Shot(td.t - 5.0, td.t + 2.2, "spectator", [Slow(td.t - 0.5, td.t + 0.5, 0.5)], note="landing",
                                    params={"locked": True})))
        beats.append((2, td.t + 3.0, Shot(td.t + 2.2, min(t_last, td.t + 5.5), "wide", note="end")))
    return place(beats, t_first, t_last)


ALT_CAM = {"wide": "spectator", "spectator": "orbit", "orbit": "spectator", "crane": "spectator", "closeup": "spectator"}


def place(beats: Sequence[tuple[int, float, Shot]], t_first: float, t_last: float, min_len: float = 1.2) -> list[Shot]:
    """Priority placement into free sim time (see ``auto_shots``); adjacent contiguous shots never share a rig."""
    free: list[tuple[float, float]] = [(t_first, t_last)]
    out: list[Shot] = []
    for _, key, s in sorted(beats, key=lambda b: (b[0], b[1])):
        a, b = max(s.t0, t_first), min(s.t1, t_last)
        pieces = [(max(a, f0), min(b, f1)) for f0, f1 in free if min(b, f1) - max(a, f0) > 1e-9]
        if not pieces:
            continue
        inside = [p for p in pieces if p[0] - 1e-9 <= key <= p[1] + 1e-9]
        p0, p1 = inside[0] if inside else max(pieces, key=lambda p: p[1] - p[0])
        if p1 - p0 < min_len:
            continue
        slow = [Slow(max(w.a, p0), min(w.b, p1), w.factor) for w in s.slow if min(w.b, p1) > max(w.a, p0)]
        out.append(Shot(p0, p1, s.cam, slow, s.note, dict(s.params)))
        nf = []
        for f0, f1 in free:
            if f1 <= p0 or f0 >= p1:
                nf.append((f0, f1))
                continue
            if f0 < p0:
                nf.append((f0, p0))
            if p1 < f1:
                nf.append((p1, f1))
        free = nf
    out.sort(key=lambda s: s.t0)
    for i in range(1, len(out)):
        if out[i].cam == out[i - 1].cam and abs(out[i].t0 - out[i - 1].t1) < 1e-6:
            out[i].cam = ALT_CAM.get(out[i].cam, "spectator")
    return out


def normalise(beats: list[Shot], t_first: float, t_last: float, min_len: float = 1.2) -> list[Shot]:
    """Clip a user shot list to the log and drop shots shorter than ``min_len`` (no reordering)."""
    out = []
    for s in beats:
        t0, t1 = max(s.t0, t_first), min(s.t1, t_last)
        if t1 - t0 >= min_len:
            slow = [Slow(max(w.a, t0), min(w.b, t1), w.factor) for w in s.slow if min(w.b, t1) > max(w.a, t0)]
            out.append(Shot(t0, t1, s.cam, slow, s.note, dict(s.params)))
    return out


# --------------------------------------------------------------------------- #
# time remapping
# --------------------------------------------------------------------------- #
def smoothstep(x: np.ndarray | float) -> np.ndarray | float:
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def speed_at(t: float, slow: Sequence[Slow], ramp_s: float = 0.25) -> float:
    """Playback speed (sim seconds per video second) at sim time t: 1 outside the windows, ``factor`` inside,
    smoothstep ramps of ``ramp_s`` sim seconds on both sides (inside the window)."""
    s = 1.0
    for w in slow:
        if t <= w.a or t >= w.b:
            continue
        r = min(ramp_s, 0.5 * (w.b - w.a))
        u = min(float(smoothstep((t - w.a) / r)) if r > 0 else 1.0, float(smoothstep((w.b - t) / r)) if r > 0 else 1.0)
        s = min(s, 1.0 + (w.factor - 1.0) * u)
    return s


def shot_times(shot: Shot, fps: float, ramp_s: float = 0.25) -> np.ndarray:
    """Sim times of the video frames of one shot: t_{k+1} = t_k + speed(t_k) / fps, t_0 = shot.t0, t_k < shot.t1."""
    if fps <= 0:
        raise ValueError("fps must be > 0")
    ts = [shot.t0]
    while True:
        t = ts[-1]
        nt = t + speed_at(t, shot.slow, ramp_s) / fps
        if nt >= shot.t1 - 1e-9:
            break
        ts.append(nt)
    return np.array(ts)


def timeline(shots: Sequence[Shot], fps: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(sim time per frame, shot index per frame, playback speed per frame) of the whole edit."""
    ts, seg, sp = [], [], []
    for i, s in enumerate(shots):
        tt = shot_times(s, fps)
        ts.append(tt)
        seg.append(np.full(tt.size, i, dtype=int))
        sp.append(np.array([speed_at(float(x), s.slow) for x in tt]))
    if not ts:
        return np.zeros(0), np.zeros(0, dtype=int), np.zeros(0)
    return np.concatenate(ts), np.concatenate(seg), np.concatenate(sp)


def shots_json(shots: Sequence[Shot]) -> str:
    return json.dumps([s.to_json() for s in shots], indent=1)


def video_duration_s(shots: Sequence[Shot], fps: float) -> float:
    return sum(shot_times(s, fps).size for s in shots) / fps


def describe(shots: Sequence[Shot], fps: float) -> list[str]:
    out = []
    for i, s in enumerate(shots):
        n = shot_times(s, fps).size
        sl = ", ".join(f"{w.factor:g}x {w.a:.2f}-{w.b:.2f}" for w in s.slow)
        out.append(f"#{i:02d} {s.cam:9s} sim {s.t0:7.2f}-{s.t1:7.2f}  {n:4d} fr ({n / fps:5.2f} s)  {s.note} {sl}")
    return out


def vane_work(delta: np.ndarray, thetas: Sequence[float]) -> np.ndarray:
    """|sum_i delta_i e_theta_i| per sample (rad): the vanes' net side-force command (pitch / yaw correction), without
    the common-mode roll trim (tools/sim_video.py poster metric, e_theta = (-sin theta, cos theta))."""
    E = np.array([[-math.sin(th), math.cos(th)] for th in thetas])
    d = np.asarray(delta, float)[:, :E.shape[0]]
    return np.linalg.norm(d @ E, axis=1)


def vane_peak_time(log: Any, a: float, b: float, thetas: Sequence[float]) -> float:
    """Sim time in [a, b] where the vanes work hardest while powered (``vane_work``) - the frame a 'correction'
    close-up should show (art review round 2: a fixed time showed only the roll trim)."""
    t = np.asarray(log.t, float)
    sel = (t >= a) & (t <= b)
    if hasattr(log, "state") and log.state is not None:
        pw = sel & (np.asarray(log.state) == STATE_POWERED)
        sel = pw if np.any(pw) else sel
    if not np.any(sel):
        raise ValueError(f"no log samples in [{a}, {b}] s")
    idx = np.flatnonzero(sel)
    w = vane_work(np.asarray(log.delta)[idx], thetas)
    return float(t[idx[int(np.argmax(w))]])


_ = math
