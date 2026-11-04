"""Tests for tools/sim_video.py: the pure parts (log loading, resampling, slerp, the vane transform sign,
the camera plan, HUD text, plot cursor), the vane sense against a real ``sim.sim6dof`` attitude
correction, the projection against VTK's own, and a small smoke render (MP4 + poster).

The synthetic logs below are kinematic TEST PATTERNS for the renderer (labelled so in their meta); they
are not simulations and nothing here claims flight behaviour from them.

Run: ``.venv/Scripts/python.exe -m pytest tools/test_sim_video.py -q``
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import sim_video as sv  # noqa: E402

try:
    import importlib.util
    HAVE_PV = importlib.util.find_spec("pyvista") is not None and importlib.util.find_spec("imageio_ffmpeg") is not None
except ImportError:  # pragma: no cover
    HAVE_PV = False

SYNTH_TITLE = "SYNTHETIC TEST PATTERN (kinematic, not a simulation)"


# --------------------------------------------------------------------------- #
# synthetic logs (renderer test patterns)
# --------------------------------------------------------------------------- #
def _axis_angle_q(axis: np.ndarray, ang: np.ndarray) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis, axis=-1, keepdims=True)
    h = 0.5 * np.asarray(ang, dtype=float)
    return np.column_stack([np.cos(h), axis * np.sin(h)[:, None]]) if axis.ndim == 2 else \
        np.column_stack([np.cos(h), np.outer(np.sin(h), axis)])


def synthetic_vtol(path: str, hz: float = 100.0, t_end: float = 30.0, extra: dict | None = None,
                   drop: tuple[str, ...] = ()) -> str:
    """VTOL-shaped kinematic pattern: SAFE, ON -> SPOOL, CLIMB to 5 m, HOLD (gust + tilt bump, UP, DOWN),
    OFF -> LAND beside the pad on the ground (ground_z = -0.11), LANDED, SAFE."""
    t = np.round(np.arange(0.0, t_end + 1e-9, 1.0 / hz), 9)
    n = t.size
    state = np.zeros(n, dtype=np.int8)
    phase = np.zeros(n, dtype=np.int8)
    state[(t >= 1.0) & (t < 3.0)] = 1
    phase[(t >= 1.0) & (t < 3.0)] = 1
    state[(t >= 3.0) & (t < 27.0)] = 2
    phase[(t >= 3.0) & (t < 8.0)] = 2
    phase[(t >= 8.0) & (t < 20.0)] = 3
    phase[(t >= 20.0) & (t < 27.0)] = 4
    state[(t >= 27.0) & (t < 29.0)] = 3
    phase[(t >= 27.0) & (t < 29.0)] = 5
    tgt = np.full(n, np.nan)
    tgt[t >= 3.0] = 5.0
    tgt[t >= 12.0] = 6.0
    tgt[t >= 16.0] = 5.0
    tgt[t >= 20.0] = -0.11
    tgt[t >= 27.0] = np.nan
    ref = np.full(n, np.nan)
    h = 0.0
    zs = np.zeros(n)
    for i in range(n):
        if 3.0 <= t[i] < 27.0:
            goal = tgt[i]
            rate = 1.0 if goal > h else (0.5 if h < 1.0 else 1.0)
            h = h + np.clip(goal - h, -rate / hz, rate / hz)
            ref[i] = h
        zs[i] = h
    x = np.where(t >= 3.0, 0.12 * (np.minimum(t, 27.0) - 3.0), 0.0)
    z = zs.copy()
    z[t >= 27.0] = -0.11
    pos = np.column_stack([x, 0.02 * x, z])
    lean = 4.0 * np.exp(-((t - 10.6) / 0.5) ** 2)                  # deg, a bump toward +x during the gust
    ang = np.radians(lean)
    q = _axis_angle_q(np.array([0.0, 1.0, 0.0]), ang)              # rotation about +y: nose toward +x
    vel = np.gradient(pos, t, axis=0)
    thr = np.where(state == 2, 0.92 + 0.03 * np.sin(2 * t), 0.0)
    thr[(t >= 10.2) & (t < 10.9)] = 1.0                             # 'saturated' stretch for the overlay
    thrust = 30.3 * thr ** 1.0
    # vanes: the SPEC 8.2 convention, net side force on the tail toward the lean (+x): vane 2 (theta 90 deg,
    # e_theta = -x) negative, vane 4 (theta 270 deg, e_theta = +x) positive
    d = np.radians(lean * 2.0)
    delta = np.column_stack([np.zeros(n), -d, np.zeros(n), d])
    wind = np.zeros((n, 3))
    wind[:, 0] = 2.0
    gust = (t >= 10.0) & (t < 11.0)
    wind[gust, 0] += 4.0
    meta = {"scenario": "synthetic_vtol", "title": SYNTH_TITLE, "seed": 0, "mode": "vtol", "params_sha256": "n/a",
            "notes": "Renderer test pattern written by tools/test_sim_video.py: kinematic curves, no physics.",
            "limits": {"u_max": 1.0, "tilt_abort_deg": 45.0, "hover_throttle": 0.92}}
    data = {"t": t, "pos": pos, "vel": vel, "q": q, "delta": delta, "throttle": thr, "thrust": thrust,
            "state": state, "phase": phase, "h_target": tgt, "h_ref": ref, "tilt_deg": lean, "wind": wind,
            "gust_active": gust, "ground_z": np.array(-0.11),
            "event_t": np.array([1.0, 3.0, 8.0, 20.0, 27.0, 29.0]),
            "event_text": np.array(["SAFE -> ARMED (SPOOL)", "ARMED -> POWERED (CLIMB)", "HOLD at 5.0 m",
                                    "LAND (OFF)", "touchdown confirmed -> UNPOWERED (LANDED)", "LANDED -> SAFE"]),
            "cmd_t": np.array([0.9, 12.0, 16.0, 19.9]), "cmd_name": np.array(["ON", "UP", "DOWN", "OFF"]),
            "meta": np.array(json.dumps(meta))}
    if extra:
        data.update(extra)
    for k in drop:
        data.pop(k, None)
    np.savez(path, **data)
    return path


PUSHES = ((9.0, 0.30, 12.0, 30.0, "axis"), (14.0, 0.25, 8.0, 160.0, "surface"), (19.0, 0.40, 15.0, 280.0, "axis"))
SYNTH_PUSH_TITLE = "SYNTHETIC TEST PATTERN: demo hover 1.5 m + 3 hand pushes (kinematic, not a simulation)"
SYNTH_PUSH_TITLE_KO = "합성 테스트 패턴: 1.5 m 호버링 + 손으로 3번 밀기 (운동학 곡선, 시뮬레이션 아님)"


def synthetic_push(path: str, hz: float = 100.0, t_end: float = 30.0, pushes=PUSHES, drop: tuple[str, ...] = ()) -> str:
    """Demo-shaped kinematic pattern: ON, SPOOL, CLIMB to 1.5 m at 0.5 m/s, HOLD with three pushes (start, duration,
    force N, direction deg toward; contact on the body axis at the CG or on the surface), OFF -> LAND, LANDED.
    Response curves are made up for the renderer (tilt bump 0.5 deg/N, displacement 0.03 m/N); NOT physics."""
    t = np.round(np.arange(0.0, t_end + 1e-9, 1.0 / hz), 9)
    n = t.size
    state = np.zeros(n, dtype=np.int8)
    phase = np.zeros(n, dtype=np.int8)
    state[(t >= 1.0) & (t < 3.0)] = 1
    phase[(t >= 1.0) & (t < 3.0)] = 1
    state[(t >= 3.0) & (t < 27.5)] = 2
    phase[(t >= 3.0) & (t < 6.0)] = 2
    phase[(t >= 6.0) & (t < 24.0)] = 3
    phase[(t >= 24.0) & (t < 27.5)] = 4
    state[(t >= 27.5) & (t < 29.5)] = 3
    phase[(t >= 27.5) & (t < 29.5)] = 5
    z = np.interp(t, [0.0, 3.0, 6.0, 24.0, 27.4, 30.0], [0.0, 0.0, 1.5, 1.5, -0.11, -0.11])
    tgt = np.where((t >= 3.0) & (t < 24.0), 1.5, np.nan)
    tgt[(t >= 24.0) & (t < 27.5)] = -0.11
    ref = np.where((t >= 3.0) & (t < 27.5), z, np.nan)
    xy = np.zeros((n, 2))
    tilt = np.zeros(n)
    lean_axis = np.zeros((n, 2))
    pf = np.zeros((n, 3))
    act = np.zeros(n, dtype=bool)
    z_cg = 0.39
    for (t0, dur, F, ddeg, _where) in pushes:
        u = np.array([math.cos(math.radians(ddeg)), math.sin(math.radians(ddeg))])
        on = (t >= t0) & (t < t0 + dur)
        act |= on
        pf[on, :2] = F * u
        s = np.clip(t - t0, 0.0, None)
        xy += (0.03 * F * (1.0 - np.exp(-s / 0.8)))[:, None] * u[None, :]
        bump = 0.5 * F * (s / 0.35) * np.exp(1.0 - s / 0.35) * (t >= t0)
        tilt += bump
        lean_axis += bump[:, None] * u[None, :]
    nrm = np.linalg.norm(lean_axis, axis=1)
    d_hat = np.where(nrm[:, None] > 1e-9, lean_axis / np.maximum(nrm, 1e-9)[:, None], np.array([1.0, 0.0]))
    rot_axis = np.column_stack([-d_hat[:, 1], d_hat[:, 0], np.zeros(n)])     # z x d: tips the nose toward d
    q = _axis_angle_q(rot_axis, np.radians(tilt))
    pos = np.column_stack([xy, z])
    R = sv.quat_to_matrix(q)
    cg = pos + R[:, :, 2] * z_cg
    pp = cg.copy()
    for (t0, dur, F, ddeg, where) in pushes:
        on = (t >= t0) & (t < t0 + dur)
        if where == "surface":
            u = np.array([math.cos(math.radians(ddeg)), math.sin(math.radians(ddeg)), 0.0])
            pp[on] = cg[on] - 0.06 * u
    vel = np.gradient(pos, t, axis=0)
    thr = np.where(state == 2, 0.91 + 0.01 * tilt, 0.0)
    thrust = 30.3 * thr ** 2
    d = np.radians(np.clip(2.0 * tilt, 0.0, 20.0))
    # SPEC 8.2 (as in synthetic_vtol): net side force on the tail toward the lean d_hat
    delta = np.column_stack([d * d_hat[:, 1], -d * d_hat[:, 0], -d * d_hat[:, 1], d * d_hat[:, 0]])
    meta = {"scenario": "synthetic_push", "title": SYNTH_PUSH_TITLE, "title_ko": SYNTH_PUSH_TITLE_KO, "seed": 0,
            "mode": "vtol", "params_sha256": "n/a", "profile": "demo",
            "notes": "Renderer test pattern written by tools/test_sim_video.py: kinematic curves, no physics. "
                     "No ground-effect model, no tip-over model (statements of the real sim, repeated for the cards).",
            "limits": {"u_max": 1.0, "tilt_abort_deg": 45.0, "hover_throttle": 0.91, "ceiling_m": 5.0, "geofence_m": 8.0},
            "criteria": {"gust_band_deg": 2.0}}
    data = {"t": t, "pos": pos, "vel": vel, "q": q, "delta": delta, "throttle": thr, "thrust": thrust, "state": state,
            "phase": phase, "h_target": tgt, "h_ref": ref, "tilt_deg": tilt, "wind": np.zeros((n, 3)),
            "gust_active": np.zeros(n, dtype=bool), "ground_z": np.array(-0.11),
            "push_force": pf, "push_point": pp, "push_active": act,
            "event_t": np.array([1.0, 3.0, 6.0, 24.0, 27.5, 29.5]),
            "event_text": np.array(["SAFE -> ARMED: ON (seq 1)", "ARMED -> POWERED: spool warning complete",
                                    "POWERED: phase CLIMB -> HOLD: reference at the target 1.5 m",
                                    "POWERED: phase HOLD -> LAND: OFF (seq 2)",
                                    "POWERED -> UNPOWERED: touchdown confirmed: motor off (LANDED)",
                                    "UNPOWERED -> SAFE: landed for 500 ticks"]),
            "cmd_t": np.array([0.9, 23.9]), "cmd_name": np.array(["ON", "OFF"]), "meta": np.array(json.dumps(meta))}
    for k in drop:
        data.pop(k, None)
    np.savez(path, **data)
    return path


def synthetic_wind_swing(path: str, hz: float = 50.0, t_end: float = 40.0, amp_deg: float = 60.0, period_s: float = 16.0,
                         mean_dir_deg: float = 30.0, speed: float = 3.0, with_mean: bool = True) -> str:
    """Field-shaped kinematic pattern: hover at 5 m while the wind direction swings +-amp_deg about mean_dir_deg
    (sinusoid of period_s) at ``speed`` with a little made-up turbulence and one gust; drift follows the wind."""
    t = np.round(np.arange(0.0, t_end + 1e-9, 1.0 / hz), 9)
    n = t.size
    ang = np.radians(mean_dir_deg + amp_deg * np.sin(2.0 * math.pi * t / period_s))
    wm = np.column_stack([speed * np.cos(ang), speed * np.sin(ang), np.zeros(n)])
    rng = np.random.default_rng(5)
    turb = np.zeros((n, 3))                                     # made-up first-order (mean-reverting) noise, ~0.5 m/s
    e = rng.normal(0.0, 0.15, size=(n, 2))
    for i in range(1, n):
        turb[i, :2] = 0.95 * turb[i - 1, :2] + e[i]
    wind = wm + turb
    gust = (t >= 20.0) & (t < 21.0)
    wind[gust] += 3.0 * np.array([math.cos(ang[gust][0]), math.sin(ang[gust][0]), 0.0])
    state = np.where((t >= 2.0) & (t < 38.0), 2, 0).astype(np.int8)
    phase = np.where(state == 2, 3, 0).astype(np.int8)
    phase[(t >= 2.0) & (t < 7.0)] = 2
    phase[(t >= 33.0) & (t < 38.0)] = 4
    z = np.interp(t, [0, 2, 7, 33, 38, 40], [0, 0, 5, 5, -0.11, -0.11])
    xy = np.cumsum(0.4 * wm[:, :2], axis=0) / hz * (state == 2)[:, None]
    pos = np.column_stack([xy, z])
    q = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    meta = {"scenario": "synthetic_wind_swing", "title": "SYNTHETIC TEST PATTERN: swinging wind (kinematic)",
            "title_ko": "합성 테스트 패턴: 방향이 흔들리는 바람 (운동학 곡선, 시뮬레이션 아님)", "seed": 0, "mode": "vtol",
            "notes": "Renderer test pattern: kinematic curves, no physics.", "limits": {"u_max": 1.0}}
    data = {"t": t, "pos": pos, "q": q, "wind": wind, "gust_active": gust, "state": state, "phase": phase,
            "throttle": np.where(state == 2, 0.92, 0.0), "thrust": np.where(state == 2, 25.6, 0.0),
            "ground_z": np.array(-0.11), "meta": np.array(json.dumps(meta))}
    if with_mean:
        data["wind_mean"] = wm
    np.savez(path, **data)
    return path


# --------------------------------------------------------------------------- #
# loading
# --------------------------------------------------------------------------- #
def test_load_log_full(tmp_path):
    p = synthetic_vtol(str(tmp_path / "a.npz"))
    log = sv.load_log(p)
    assert log.n == 3001
    assert log.warnings == []
    assert log.mode == "vtol" and log.meta["title"] == SYNTH_TITLE
    assert log.ground_z == pytest.approx(-0.11)
    assert log.cmd_name == ["ON", "UP", "DOWN", "OFF"]
    assert log.has_phase and log.phase_name(3) == "HOLD" and log.state_name(5) == "ABORT"
    assert np.allclose(np.linalg.norm(log.q, axis=1), 1.0)
    assert len(log.sha256) == 64


def test_load_log_defaults_and_warnings(tmp_path):
    p = synthetic_vtol(str(tmp_path / "b.npz"), drop=("vel", "delta", "wind", "gust_active", "ground_z", "tilt_deg",
                                                     "phase", "h_target", "h_ref", "meta", "event_t", "event_text"))
    log = sv.load_log(p)
    joined = " ".join(log.warnings)
    for key in ("vel", "delta", "wind", "gust_active", "ground_z", "tilt_deg", "no meta"):
        assert key in joined
    assert np.all(log.delta == 0.0) and not log.has_phase
    assert np.all(np.isnan(log.h_target))
    assert np.allclose(log.tilt_deg, sv.tilt_deg_of(log.q))
    assert log.ground_z < 0.0                                  # the pad's foot level from params.json
    # vel from the gradient of pos
    i = 1500
    assert log.vel[i, 0] == pytest.approx((log.pos[i + 1, 0] - log.pos[i - 1, 0]) / (log.t[i + 1] - log.t[i - 1]))


def test_load_log_rejects_bad(tmp_path):
    p = str(tmp_path / "c.npz")
    np.savez(p, t=np.arange(5.0), pos=np.zeros((5, 3)))
    with pytest.raises(ValueError, match="'q'"):
        sv.load_log(p)
    np.savez(p, t=np.array([0.0, 1.0, 1.0]), pos=np.zeros((3, 3)), q=np.tile([1.0, 0, 0, 0], (3, 1)))
    with pytest.raises(ValueError, match="strictly increasing"):
        sv.load_log(p)
    np.savez(p, t=np.arange(4.0), pos=np.zeros((3, 3)), q=np.tile([1.0, 0, 0, 0], (4, 1)))
    with pytest.raises(ValueError, match="do not match"):
        sv.load_log(p)


def test_load_log_truncates_nonfinite_and_cg_ref(tmp_path):
    n = 50
    t = np.linspace(0.0, 1.0, n)
    q = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1)) * 2.0          # not unit -> normalised + warned
    pos = np.zeros((n, 3))
    pos[:, 2] = 0.4                                           # CG of an upright vehicle on the pad
    pos[30:, 0] = np.nan
    meta = {"pos_ref": "cg", "z_cg_m": 0.4}
    p = str(tmp_path / "d.npz")
    np.savez(p, t=t, pos=pos, q=q, meta=np.array(json.dumps(meta)))
    log = sv.load_log(p)
    assert log.n == 30
    assert any("non-finite" in w for w in log.warnings) and any("not unit" in w for w in log.warnings)
    assert np.allclose(log.pos[:, 2], 0.0)                     # exit plane = CG - R (0, 0, z_cg)


# --------------------------------------------------------------------------- #
# resampling / slerp
# --------------------------------------------------------------------------- #
def test_frame_times():
    ft = sv.frame_times(1.0, 3.0, 30.0, 1.0)
    assert ft.size == 61 and ft[0] == 1.0 and ft[-1] == pytest.approx(3.0)
    ft2 = sv.frame_times(0.0, 10.0, 30.0, 2.0)
    assert ft2.size == 151 and np.allclose(np.diff(ft2), 2.0 / 30.0)
    with pytest.raises(ValueError):
        sv.frame_times(0.0, 1.0, 0.0)


def test_slerp_properties():
    rng = np.random.default_rng(3)
    for _ in range(20):
        q0 = sv.quat_normalize(rng.normal(size=4))
        q1 = sv.quat_normalize(rng.normal(size=4))
        assert np.allclose(np.abs(np.dot(sv.slerp(q0, q1, 0.0), q0)), 1.0)
        assert np.allclose(np.abs(np.dot(sv.slerp(q0, q1, 1.0), q1)), 1.0)
        s = rng.uniform(size=7)
        qs = sv.slerp(np.tile(q0, (7, 1)), np.tile(q1, (7, 1)), s)
        assert np.allclose(np.linalg.norm(qs, axis=1), 1.0)
    # constant rate: about z from 0 to 90 deg, the midpoint is 45 deg
    qa = np.array([1.0, 0.0, 0.0, 0.0])
    qb = np.array([math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)])
    qm = sv.slerp(qa, qb, 0.5)
    assert math.degrees(2.0 * math.acos(qm[0])) == pytest.approx(45.0, abs=1e-9)
    # q and -q are the same rotation: the short arc is taken
    qm2 = sv.slerp(qa, -qb, 0.5)
    assert np.allclose(sv.quat_to_matrix(qm2), sv.quat_to_matrix(qm))


def test_resample_linear_hold_and_gust_split(tmp_path):
    log = sv.load_log(synthetic_vtol(str(tmp_path / "e.npz")))
    tq = np.array([2.995, 10.5, 11.995, 12.005, 28.0])
    fr = sv.resample(log, tq)
    i = 299                                                   # t = 2.99
    assert fr.pos[0, 0] == pytest.approx(0.5 * (log.pos[i, 0] + log.pos[i + 1, 0]))
    assert fr.state[0] == 1 and fr.phase[0] == 1              # zero-order hold
    assert fr.h_target[2] == 5.0 and fr.h_target[3] == 6.0    # a step command is held, not interpolated
    assert np.isnan(fr.h_target[4])
    assert fr.gust_active[1]
    assert np.allclose(fr.wind_steady[1], [2.0, 0.0, 0.0]) and np.allclose(fr.wind[1], [6.0, 0.0, 0.0])
    assert np.allclose(np.linalg.norm(fr.q, axis=1), 1.0)
    assert fr.R.shape == (5, 3, 3)


def test_last_command_and_events():
    ct = np.array([1.0, 2.0, 5.0])
    names = ["ON", "UP", "OFF"]
    assert sv.last_command(ct, names, 0.5) is None
    assert sv.last_command(ct, names, 2.0) == ("UP", 2.0)
    assert sv.last_command(ct, names, 9.0) == ("OFF", 5.0)
    ev = sv.recent_events(np.array([0.0, 1.0, 2.0, 3.0]), ["a", "b", "c", "d"], 2.5, max_n=2)
    assert [e for _, e in ev] == ["b", "c"]


# --------------------------------------------------------------------------- #
# vane transform sign (SPEC 8.2)
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("theta_deg", [0.0, 90.0, 180.0, 270.0, 33.0])
def test_vane_transform_sign(theta_deg):
    """delta > 0: the trailing edge (below the hinge) moves toward -e_theta, the leading edge toward
    +e_theta, points on the hinge axis stay; the STL lies along +x (theta_stl = 0)."""
    hz = 0.037
    th = math.radians(theta_deg)
    te = np.array([0.028, 0.0, 0.0146])                       # trailing edge, vane STL frame (theta 0)
    le = np.array([0.028, 0.0, 0.0466])
    hinge = np.array([0.028, 0.0, hz])
    M0 = sv.vane_local_matrix(th, 0.0, hz)
    M = sv.vane_local_matrix(th, math.radians(10.0), hz)
    e = sv.e_theta(th)
    d_te = sv.apply_matrix(M, te) - sv.apply_matrix(M0, te)
    d_le = sv.apply_matrix(M, le) - sv.apply_matrix(M0, le)
    assert float(np.dot(d_te, e)) == pytest.approx(-(hz - te[2]) * math.sin(math.radians(10.0)), rel=1e-9)
    assert float(np.dot(d_le, e)) > 0.0
    assert np.allclose(sv.apply_matrix(M, hinge), sv.apply_matrix(M0, hinge))
    # the neutral vane sits at theta
    p0 = sv.apply_matrix(M0, te)
    assert math.degrees(math.atan2(p0[1], p0[0])) % 360.0 == pytest.approx(theta_deg % 360.0, abs=1e-9)
    # an STL exported at another angle is first brought onto +x
    Ms = sv.vane_local_matrix(th, 0.0, hz, theta_stl_rad=math.radians(90.0))
    p_s = sv.apply_matrix(Ms, np.array([0.0, 0.028, 0.0146]))
    assert np.allclose(p_s, p0)


def test_vane_forces_world_net_direction():
    """delta_x < 0 (vanes 1 / 3) -> net side force -y; vanes 2 / 4 opposite signs -> net +x."""
    th = (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)
    F = sv.vane_forces_world(np.eye(3), np.radians([-5.0, 0.0, 5.0, 0.0]), th, 1.0, math.radians(20.0))
    net = F.sum(axis=0)
    assert net[1] < 0 and abs(net[0]) < 1e-12
    F = sv.vane_forces_world(np.eye(3), np.radians([0.0, -5.0, 0.0, 5.0]), th, 1.0, math.radians(20.0))
    assert F.sum(axis=0)[0] > 0
    # equal deflections: a pure roll couple, no net side force
    F = sv.vane_forces_world(np.eye(3), np.radians([5.0, 5.0, 5.0, 5.0]), th, 1.0, math.radians(20.0))
    assert np.allclose(F.sum(axis=0), 0.0, atol=1e-12)


def _sim_available() -> bool:
    try:
        import importlib
        importlib.import_module("sim.sim6dof")
        importlib.import_module("sim.vehicle")
        return True
    except Exception:  # pragma: no cover
        return False


@pytest.mark.skipif(not _sim_available(), reason="sim package not importable")
def test_vane_sense_against_s1_correction(tmp_path):
    """A real sim.sim6dof run (S1: 10 deg initial tilt, hover thrust): while the tilt is being removed,
    (a) the moment of the logged vane angles under the SPEC 8.2 force convention points along the
    correcting rotation, and (b) the net side force the renderer draws pushes the tail toward the side the
    nose leans to (which is what straightens the vehicle)."""
    p = str(tmp_path / "s1.npz")
    sv.export_dev_log("S1", p, t_end=0.5)
    log = sv.load_log(p)
    g = sv.VehicleGeom.from_params()
    z_cg = float(log.meta["z_cg_m"])
    tilt_dd = np.gradient(np.gradient(log.tilt_deg, log.t), log.t)

    def signs(i: int) -> tuple[float, float]:
        R = sv.quat_to_matrix(log.q[i])
        # (a) body moment of the vane forces (SPEC 8.3 / 8.4 geometry), F_i = k delta_i e_theta_i, k > 0
        M = np.zeros(3)
        for d, th in zip(log.delta[i], g.vane_thetas_rad):
            r = np.array([g.vane_force_r_m * math.cos(th), g.vane_force_r_m * math.sin(th), g.hinge_z_m - z_cg])
            M += np.cross(r, d * sv.e_theta(th))
        z_b = R[:, 2]
        corr_axis_body = R.T @ np.cross(z_b, [0.0, 0.0, 1.0])     # the rotation that brings z_b back up
        # (b) the net side force the renderer draws, against the direction the nose leans
        F = sv.vane_forces_world(R, log.delta[i], g.vane_thetas_rad, 1.0, g.delta_cmd_max_rad).sum(axis=0)
        return float(np.dot(M, corr_axis_body)), float(np.dot(F[:2], z_b[:2]))

    # 0.04-0.20 s: the tilt falls ever faster (the vanes accelerate the correction) -> both positive;
    # 0.26-0.40 s: the fall slows down (the rate loop brakes) -> both negative.  Same sign rule both ways.
    acc = np.flatnonzero((log.t >= 0.04) & (log.t <= 0.20))
    brk = np.flatnonzero((log.t >= 0.26) & (log.t <= 0.40))
    assert np.all(np.diff(log.tilt_deg[acc[0]:brk[-1] + 1]) < 0.0)
    assert np.all(tilt_dd[acc] < 0.0) and np.all(tilt_dd[brk] > 0.0)
    for i in acc:
        m, f = signs(i)
        assert m > 0.0 and f > 0.0, (log.t[i], m, f)
    for i in brk:
        m, f = signs(i)
        assert m < 0.0 and f < 0.0, (log.t[i], m, f)
    assert acc.size >= 35 and brk.size >= 30


# --------------------------------------------------------------------------- #
# HUD text
# --------------------------------------------------------------------------- #
def _sample(**kw):
    s = {"t": 12.5, "speed": 1.0, "mode": "vtol", "phase": "HOLD", "state": "POWERED", "alt": 5.02,
         "h_target": 6.0, "h_ref": 5.4, "vz": 0.31, "tilt": 1.2, "throttle": 0.93, "u_max": 1.0,
         "wind": np.array([2.0, 0.0, 0.0]), "gust": np.zeros(3), "gust_active": False, "last_cmd": ("UP", 12.0)}
    s.update(kw)
    return s


def test_hud_lines_content():
    rows = sv.hud_lines(_sample())
    d = {lab: (val, tone) for lab, val, tone in rows}
    assert d["PHASE"][0] == "HOLD" and d["STATE"] == ("POWERED", "ok")
    assert "target 6.0" in d["ALT"][0] and "ref 5.40" in d["ALT"][0]
    assert d["CMD"][0].startswith("UP") and "0.5 s ago" in d["CMD"][0]
    assert d["THR"][1] == "normal" and "GUST" not in d
    assert "toward   0 deg" in d["WIND"][0]


def test_hud_lines_flags():
    rows = sv.hud_lines(_sample(throttle=1.0, gust_active=True, gust=np.array([0.0, 4.0, 0.0]), state="ABORT",
                                h_target=float("nan"), h_ref=float("nan"), last_cmd=None, phase=None, tilt=35.0,
                                speed=2.0))
    d = {lab: (val, tone) for lab, val, tone in rows}
    assert "SATURATED" in d["THR"][0] and d["THR"][1] == "warn"
    assert d["GUST"] == ("ACTIVE  +4.0 m/s", "alert")
    assert d["STATE"][1] == "alert" and d["TILT"][1] == "alert"
    assert "target" not in d["ALT"][0] and "PHASE" not in d
    assert d["CMD"] == ("none yet", "muted")
    assert "(x2)" in d["TIME"][0]


# --------------------------------------------------------------------------- #
# camera
# --------------------------------------------------------------------------- #
def test_camera_keeps_vehicle_in_view(tmp_path):
    log = sv.load_log(synthetic_vtol(str(tmp_path / "f.npz")))
    fps = 30.0
    tq = sv.frame_times(log.t[0], log.t[-1], fps)
    fr = sv.resample(log, tq)
    W, H = 1280, 1080
    plan = sv.plan_camera(fr, log.ground_z, fps, W / H, 0.86, "all", azimuth_deg=90.0)
    vp = sv.vehicle_points(fr, 0.86)
    pad_seen = 0
    for k in range(fr.n):
        cam = plan.view(k)
        px, z = sv.project(vp[k], cam, W, H)
        assert np.all(z > 0)
        assert np.all((px[:, 0] > 0.04 * W) & (px[:, 0] < 0.96 * W) & (px[:, 1] > 0.04 * H) & (px[:, 1] < 0.96 * H)), k
        pp, zp = sv.project(np.array([[0.0, 0.0, 0.0]]), cam, W, H)
        if zp[0] > 0 and 0 <= pp[0, 0] <= W and 0 <= pp[0, 1] <= H:
            pad_seen += 1
    assert pad_seen == fr.n                                    # a 6 m hover with 2.6 m drift keeps the pad in view
    assert np.all(plan.framing == 2)
    # smooth: no frame-to-frame jump in distance above 3 %
    assert np.max(np.abs(np.diff(plan.dist)) / plan.dist[:-1]) < 0.03


def test_camera_drops_pad_when_far():
    n = 200
    t = np.linspace(0.0, 20.0, n)
    pos = np.column_stack([np.linspace(0.0, 300.0, n), np.zeros(n), np.linspace(0.0, 150.0, n)])
    q = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    fr = sv.Frames(t, pos, np.zeros((n, 3)), q, sv.quat_to_matrix(q), np.zeros((n, 4)), np.zeros(n), np.zeros(n),
                   np.zeros(n, dtype=int), np.zeros(n, dtype=int), np.full(n, np.nan), np.full(n, np.nan), np.zeros(n),
                   np.zeros((n, 3)), np.zeros((n, 3)), np.zeros(n, dtype=bool))
    plan = sv.plan_camera(fr, -0.11, 10.0, 1280 / 1080, 0.86, "all")
    assert plan.framing[0] == 2 and plan.framing[-1] == 0
    for k in range(n):
        px, z = sv.project(sv.vehicle_points(fr, 0.86)[k], plan.view(k), 1280, 1080)
        assert np.all(z > 0) and np.all((px >= 0) & (px <= [1280, 1080]))


def test_camera_tracks_ballistic_arc():
    """Vehicle-only framing through an apogee (-g curvature): the vehicle stays near the framing-box
    centre (the focal point follows the lightly smoothed path, not a 0.6 s average of it)."""
    fps = 30.0
    t = np.arange(0.0, 6.0, 1.0 / fps)
    z = 140.0 + 15.0 * (t - 3.0) - 0.5 * 9.81 * (t - 3.0) ** 2
    n = t.size
    pos = np.column_stack([3.0 * t, np.zeros(n), z])
    q = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    fr = sv.Frames(t, pos, np.zeros((n, 3)), q, sv.quat_to_matrix(q), np.zeros((n, 4)), np.zeros(n), np.zeros(n),
                   np.zeros(n, dtype=int), np.zeros(n, dtype=int), np.full(n, np.nan), np.full(n, np.nan), np.zeros(n),
                   np.zeros((n, 3)), np.zeros((n, 3)), np.zeros(n, dtype=bool))
    aspect = 1280 / 1080
    plan = sv.plan_camera(fr, -0.11, fps, aspect, 0.86, "vehicle")
    bx0, bx1, by0, by1 = sv.FRAME_BOX
    for k in range(n):
        ndc, _ = sv._ndc(sv.vehicle_points(fr, 0.86)[k][1:2], plan.view(k), aspect)
        assert abs(ndc[0, 0] - 0.5 * (bx0 + bx1)) < 0.12 and abs(ndc[0, 1] - 0.5 * (by0 + by1)) < 0.12, (t[k], ndc)


def test_camera_avoids_overlay_rects(tmp_path):
    log = sv.load_log(synthetic_vtol(str(tmp_path / "av.npz")))
    fps = 15.0
    fr = sv.resample(log, sv.frame_times(log.t[0], log.t[-1], fps))
    aspect = 1280 / 1080
    hud = (-0.98, -0.3, 0.4, 0.92)                            # a big box at the upper left
    plan = sv.plan_camera(fr, log.ground_z, fps, aspect, 0.86, "all", azimuth_deg=90.0, avoid=[hud])
    vp = sv.vehicle_points(fr, 0.86)
    for k in range(fr.n):
        ndc, _ = sv._ndc(vp[k], plan.view(k), aspect)
        inside = (ndc[:, 0] > hud[0]) & (ndc[:, 0] < hud[1]) & (ndc[:, 1] > hud[2]) & (ndc[:, 1] < hud[3])
        assert not np.any(inside), k


def test_auto_azimuth_crosses_drift(tmp_path):
    log = sv.load_log(synthetic_vtol(str(tmp_path / "g.npz")))
    az = sv.auto_azimuth(log)
    f, r, _ = sv.camera_basis(az, 12.0)
    d = log.pos[-1, :2] - log.pos[0, :2]
    assert float(np.dot(r[:2], d)) > 0.99 * float(np.linalg.norm(d))   # drift runs left -> right


def test_nice_step_and_wrap():
    assert [sv.nice_step(x) for x in (0.7, 1.0, 1.3, 2.2, 4.0, 7.0, 130.0)] == [1.0, 1.0, 2.0, 2.5, 5.0, 10.0, 200.0]
    assert sv.wrap("a b c", 3) == ["a b", "c"]


# --------------------------------------------------------------------------- #
# plot panel (matplotlib only)
# --------------------------------------------------------------------------- #
def test_plot_panel_cursor(tmp_path):
    log = sv.load_log(synthetic_vtol(str(tmp_path / "h.npz")))
    panel = sv.PlotPanel(log, 0.0, 30.0, 640, 1080, 1.0)
    assert panel.bg.shape == (1080, 640, 3)
    xs = [panel.x_px(t) for t in (0.0, 10.0, 20.0, 30.0)]
    assert all(b > a for a, b in zip(xs, xs[1:]))
    img = panel.frame(15.0)
    x = int(round(panel.x_px(15.0)))
    _, _, r0, r1, _, _ = panel.boxes[1]
    col = img[(r0 + r1) // 2, x]
    assert np.all(col < 40)                                    # the cursor line is ink
    # the future is dimmed toward white
    x_f = int(round(panel.x_px(25.0)))
    assert img[(r0 + r1) // 2, x_f].mean() >= panel.bg[(r0 + r1) // 2, x_f].mean()


# --------------------------------------------------------------------------- #
# v2: language
# --------------------------------------------------------------------------- #
def test_tr_and_command_labels():
    assert sv.tr("sim_tag", "ko") == "시뮬레이션 - 실제 비행 영상 아님"
    assert sv.tr("sim_tag", "en") == "SIMULATION - not flight footage"
    assert sv.tr("push_label", "ko", f=12.04) == "밀기 12 N"
    assert sv.tr("no_such_key", "ko") == "no_such_key"
    assert sv.cmd_label("UP", "ko") == "UP (고도 +)" and sv.cmd_label("UP", "en") == "UP"
    assert sv.cmd_label("WEIRD", "ko") == "WEIRD"
    # every Korean string is really Korean where the English has letters (no forgotten copies)
    for k, d in sv.TXT.items():
        if "ko" in d and any(c.isalpha() for c in d["en"].replace("u_max", "")) and k not in ("toward",):
            assert sv.has_hangul(d["ko"]) or d["ko"] == d["en"], k


def test_phase_state_names_korean(tmp_path):
    log = sv.load_log(synthetic_vtol(str(tmp_path / "k.npz")))
    assert [log.phase_name(c, "ko") for c in range(6)] == ["대기", "시동", "상승", "호버링", "착륙", "착지"]
    assert log.state_name(5, "ko") == "비상정지" and log.state_name(2, "ko") == "동력 비행"
    assert log.phase_name(3) == "HOLD"                                   # English default unchanged


def test_event_gloss_ko():
    g = sv.event_gloss_ko
    assert g("POWERED -> UNPOWERED: touchdown confirmed: motor off (LANDED)") == "동력 비행 → 무동력 (착지 확인)"
    assert g("POWERED: phase HOLD -> LAND: OFF (seq 6)") == "단계 호버링 → 착륙"
    assert g("GS TX UP seq 2") == "지상국 송신: UP (고도 +)"
    assert g("GS ACK UP seq 3 rejected (cause ...)").endswith("(거부)")
    assert g("GS target 5.0 -> 6.0 m") == "지상국: 목표 고도 5.0 → 6.0 m"
    assert g("POWERED -> ABORT: KILL (seq 5)") == "동력 비행 → 비상정지"
    assert g("touchdown #2 on the ground") == "착지 #2" and g("liftoff (T = 25.4 N)") == "이륙"
    assert g("something unknown") == ""


def test_hud_lines_korean_and_push_row():
    rows = sv.hud_lines(_sample(phase="호버링", state="동력 비행", state_code=2, push=np.array([0.0, 12.0, 0.0])), "ko")
    d = {lab: (val, tone) for lab, val, tone in rows}
    assert d["상태"] == ("동력 비행", "ok") and d["비행 단계"][0] == "호버링"
    assert "목표 6.0" in d["고도"][0] and "기준 5.40" in d["고도"][0]
    assert d["밀기"][0].strip() == "12.0 N  →  90°" and d["밀기"][1] == "warn"
    assert d["명령"][0].startswith("UP (고도 +)") and "0.5 s 전" in d["명령"][0]
    assert d["기울기"][0].endswith("°") and d["모드"][0] == "수직이착륙 (VTOL)"
    rows = sv.hud_lines(_sample(state="비상정지", state_code=5, wind=np.zeros(3)), "ko")
    d = {lab: (val, tone) for lab, val, tone in rows}
    assert d["상태"][1] == "alert" and d["바람"][0] == "무풍" and "밀기" not in d


def test_korean_font_renders_hangul_glyphs():
    """The Korean font maps Hangul to real glyphs (two different syllables give different bitmaps; a font without
    them would draw the same .notdef box for both)."""
    p = sv.korean_font_path()
    if p is None:
        pytest.skip("no Korean font on this machine")
    f = sv.font(40, lang="ko")
    a, b = np.asarray(f.getmask("가")), np.asarray(f.getmask("밀"))
    assert a.size and b.size and (a.shape != b.shape or np.any(a != b))
    assert sv.font(20, lang="ko").getlength("0") == sv.font(20, lang="ko").getlength("8")   # tabular digits
    fam = sv.mpl_korean_family()
    assert fam


# --------------------------------------------------------------------------- #
# v2: pushes, wind direction, person
# --------------------------------------------------------------------------- #
def test_load_push_keys_and_defaults(tmp_path):
    log = sv.load_log(synthetic_push(str(tmp_path / "p.npz")))
    assert log.warnings == [] and log.has_push
    assert log.push_force.shape == (log.n, 3) and log.push_active.dtype == bool
    assert log.wind_mean_source == "smoothed"                       # no wind_mean in this pattern
    log2 = sv.load_log(synthetic_push(str(tmp_path / "p2.npz"), drop=("push_active", "push_point")))
    assert any("push_active missing" in w for w in log2.warnings) and any("push_point missing" in w for w in log2.warnings)
    assert np.array_equal(log2.push_active, np.linalg.norm(log2.push_force, axis=1) > sv.PUSH_EPS_N)
    assert np.count_nonzero(log2.push_active) == np.count_nonzero(log.push_active)
    log3 = sv.load_log(synthetic_vtol(str(tmp_path / "np.npz")))     # v1 log: no push keys, no warning
    assert log3.push_force is None and not log3.has_push and log3.warnings == []


def test_find_pushes_numbers(tmp_path):
    log = sv.load_log(synthetic_push(str(tmp_path / "p.npz")))
    ps = sv.find_pushes(log)
    assert [p.i for p in ps] == [1, 2, 3]
    for p, (t0, dur, F, ddeg, _) in zip(ps, PUSHES):
        assert p.t0 == pytest.approx(t0) and p.duration_s == pytest.approx(dur, abs=0.011)
        assert p.f_peak_n == pytest.approx(F) and p.dir_deg == pytest.approx(ddeg % 360.0)
        assert p.impulse_ns == pytest.approx(F * dur, rel=0.05)
        assert p.peak_tilt_deg == pytest.approx(0.5 * F, rel=0.02)          # the pattern's bump: 0.5 deg / N
        assert p.baseline_tilt_deg == pytest.approx(0.0, abs=0.05)
    # recovery = first time after the push END from which the tilt stays <= 2 deg (the gust_recovery rule)
    tl, t = log.tilt_deg, log.t
    for p in ps:
        after = (t >= p.t1) & (t <= p.window_end)
        out = np.flatnonzero(tl[after] > 2.0)
        assert out.size and p.recovery_s == pytest.approx(t[after][out[-1] + 1] - p.t1)
    # the window is cut at the next push
    assert ps[0].window_end == pytest.approx(min(ps[0].t1 + sv.PUSH_WINDOW_S, ps[1].t0))


def test_find_pushes_recovery_none_and_zero():
    n = 601
    t = np.linspace(0.0, 6.0, n)
    act = (t >= 1.0) & (t < 1.2)
    F = np.zeros((n, 3))
    F[act, 0] = 10.0
    base = dict(t=t, pos=np.zeros((n, 3)), vel=np.zeros((n, 3)), q=np.tile([1.0, 0, 0, 0], (n, 1)), delta=np.zeros((n, 4)),
                throttle=np.zeros(n), thrust=np.zeros(n), state=np.full(n, 2), phase=np.full(n, 3), h_target=np.zeros(n),
                h_ref=np.zeros(n), h_est=np.zeros(n), wind=np.zeros((n, 3)), gust_active=np.zeros(n, dtype=bool),
                ground_z=-0.11, event_t=np.zeros(0), event_text=[], cmd_t=np.zeros(0), cmd_name=[], meta={},
                push_force=F, push_point=np.zeros((n, 3)), push_active=act)
    stays = sv.SimLog(tilt_deg=np.where(t >= 1.0, 3.0, 0.0), **base)       # never back below 2 deg
    assert sv.find_pushes(stays)[0].recovery_s is None
    calm = sv.SimLog(tilt_deg=np.where((t >= 1.0) & (t < 1.2), 3.0, 0.5), **base)   # back inside when the push ends
    assert sv.find_pushes(calm)[0].recovery_s == 0.0
    assert sv.push_row_text(sv.find_pushes(calm)[0], "ko").count("기준 이내 유지") == 1
    assert "구간 내 미복귀" in sv.push_row_text(sv.find_pushes(stays)[0], "ko")
    assert "복귀 중" in sv.push_row_text(sv.find_pushes(stays)[0], "ko", t_now=1.5)


def test_push_window_resample_keeps_short_pushes():
    """A 0.04 s push between two frame times of a x4 video (0.133 s apart) is still drawn on a frame."""
    t = np.round(np.arange(0.0, 2.0, 0.004), 9)
    act = (t >= 1.08) & (t < 1.12)                                       # between the frames at 1.067 and 1.2 s
    F = np.zeros((t.size, 3))
    F[act] = [5.0, 0.0, 0.0]
    F[(t >= 1.10) & (t < 1.11)] = [9.0, 0.0, 0.0]
    P = np.zeros((t.size, 3))
    P[:, 2] = t
    tq = np.arange(0.0, 2.0, 4.0 / 30.0)
    a, f, p = sv.push_window_resample(t, act, F, P, tq, 4.0 / 30.0)
    assert np.count_nonzero(a) >= 1
    k = int(np.flatnonzero(a)[0])
    assert f[k, 0] == pytest.approx(9.0) and 1.10 <= p[k, 2] < 1.11      # the strongest sample of the window
    assert np.all(f[~a] == 0.0)
    a0, _, _ = sv.push_window_resample(t, act, F, P, tq, 0.0)             # a zero window would miss it
    assert not np.any(a0)


def test_wind_dir_varies_and_smoothed_wind(tmp_path):
    sw = sv.load_log(synthetic_wind_swing(str(tmp_path / "w.npz")))
    assert sw.wind_mean_source == "log" and sv.wind_dir_varies(sw)
    d = sv.wind_dir_deg_series(sw.wind_mean)
    assert d.max() - d.min() == pytest.approx(120.0, abs=0.5)
    const = sv.load_log(synthetic_wind_swing(str(tmp_path / "c.npz"), amp_deg=0.0, with_mean=False))
    assert const.wind_mean_source == "smoothed" and not sv.wind_dir_varies(const)
    calm = sv.load_log(synthetic_push(str(tmp_path / "p.npz")))
    assert not sv.wind_dir_varies(calm)
    # the smoothed default holds the last calm wind through a gust
    t = np.arange(0.0, 10.0, 0.01)
    w = np.zeros((t.size, 3))
    w[:, 0] = 3.0
    g = (t >= 5.0) & (t < 6.0)
    w[g, 0] = 9.0
    sm = sv.smoothed_wind(t, w, g)
    assert np.allclose(sm[:, 0], 3.0)
    # the direction unwraps through 0/360
    wv = np.column_stack([np.cos(np.radians([350.0, 355.0, 5.0, 10.0])), np.sin(np.radians([350.0, 355.0, 5.0, 10.0])),
                          np.zeros(4)])
    assert np.allclose(np.diff(sv.wind_dir_deg_series(wv)), [5.0, 10.0, 5.0])


def test_push_azimuths_and_track():
    az = sv.push_azimuths([30.0, 160.0, 280.0])
    assert az[0] == pytest.approx(120.0)                                  # dir + 90: the push runs left -> right
    for a, d in zip(az, (30.0, 160.0, 280.0)):
        assert abs(sv.ang_diff_deg(a, d)) == pytest.approx(90.0)           # always across the screen
    assert abs(az[1] - az[0]) <= 90.0 + 1e-9 and abs(az[2] - az[1]) <= 90.0 + 1e-9   # the smaller turn each time
    t = np.arange(0.0, 30.0, 1.0 / 30.0)
    tr_ = sv.azimuth_track(t, [(9.0, 9.3, 30.0), (14.0, 14.25, 160.0), (19.0, 19.4, 280.0)])
    for (t0, t1, _), a in zip([(9.0, 9.3, 0), (14.0, 14.25, 0), (19.0, 19.4, 0)], az):
        sel = (t >= t0 - 0.4) & (t <= t1)
        assert np.allclose(tr_[sel], a)                                    # held during each push
    assert np.max(np.abs(np.diff(tr_))) * 30.0 < 120.0                    # deg/s: a smooth turn, no cut


def test_figure_track_positions():
    n = 301
    t = np.linspace(0.0, 10.0, n)
    veh = np.zeros((n, 3))
    veh[:, 2] = 1.5
    act = ((t >= 2.0) & (t < 2.5)) | ((t >= 7.0) & (t < 7.3))
    F = np.zeros((n, 3))
    F[(t >= 2.0) & (t < 2.5)] = [10.0, 0.0, 0.0]
    F[(t >= 7.0) & (t < 7.3)] = [0.0, -8.0, 0.0]
    C = veh + np.array([0.0, 0.0, 0.39])
    ft = sv.figure_track(t, veh, act, F, C, -0.11, (5.0, 5.0))
    reach = sv.POLE_REACH_M + sv.FIGURE_HANDS[0]
    k1 = int(np.flatnonzero(act)[0])
    assert np.allclose(ft.feet[k1], [-reach, 0.0, -0.11]) and ft.yaw[k1] == pytest.approx(0.0)
    k2 = int(np.flatnonzero(t >= 7.0)[0])
    assert np.allclose(ft.feet[k2, :2], [0.0, reach]) and ft.yaw[k2] == pytest.approx(-0.5 * math.pi)
    assert np.allclose(ft.feet[0, :2], [-reach, 0.0])                      # waits where the first push comes from
    k_mid = int(np.flatnonzero(t >= 4.0)[0])
    assert np.allclose(ft.feet[k_mid, :2], [-reach, 0.0])                  # holds until it walks (2 s before #2)
    k_w = int(np.flatnonzero(t >= 6.0)[0])
    assert 0.0 < np.hypot(*(ft.feet[k_w, :2] - [-reach, 0.0])) < np.hypot(reach, reach)
    assert np.allclose(ft.hands[k1], ft.feet[k1] + [sv.FIGURE_HANDS[0], 0.0, sv.FIGURE_HANDS[2]])
    pole = np.linalg.norm(C[k1] - ft.hands[k1])
    assert pole >= 1.0                                                     # a pole of 1 m or more
    idle = sv.figure_track(t, veh, None, None, None, -0.11, (1.0, 2.0))
    assert np.allclose(idle.feet[:, :2], [1.0, 2.0]) and not np.any(idle.pushing)
    assert idle.yaw[0] == pytest.approx(math.atan2(-2.0, -1.0))           # faces the vehicle


def test_figure_occludes():
    cam = sv.CamView(np.array([0.0, -8.0, 1.5]), np.array([0.0, 0.0, 1.5]), np.array([0.0, 0.0, 1.0]), 30.0)
    veh = np.array([[0.0, 0.0, 1.5], [0.0, 0.0, 1.9], [0.0, 0.0, 2.36]])

    def person_at(x, y):
        return sv.figure_points(sv.FigureTrack(np.array([[x, y, -0.11]]), np.array([0.0]), np.array([False]),
                                               np.zeros((1, 3))))[0]
    assert sv.figure_occludes(person_at(0.0, -3.0), veh, cam, 1280, 1080)          # in front, overlapping
    assert not sv.figure_occludes(person_at(0.0, 3.0), veh, cam, 1280, 1080)       # behind the vehicle
    assert not sv.figure_occludes(person_at(2.0, -1.0), veh, cam, 1280, 1080)      # beside it


def test_visual_contact_moves_axis_point_to_surface():
    R = np.eye(3)
    pos = np.array([1.0, 2.0, 1.5])
    on_axis = pos + [0.0, 0.0, 0.39]
    c = sv.visual_contact(on_axis, np.array([10.0, 0.0, 0.0]), pos, R, 0.06)
    assert np.allclose(c, on_axis + [-0.06, 0.0, 0.0])                     # on the pusher's side of the body
    surf = on_axis + [-0.06, 0.0, 0.0]
    assert np.allclose(sv.visual_contact(surf, np.array([10.0, 0.0, 0.0]), pos, R, 0.06), surf)


def test_align_z():
    for v in ([1.0, 0, 0], [0, 0, -1.0], [0.3, -0.4, 0.5], [0, 0, 2.0]):
        M = sv._align_z(np.array(v))
        assert np.allclose(M @ [0, 0, 1.0], np.array(v) / np.linalg.norm(v)) and np.allclose(M @ M.T, np.eye(3))


def test_plan_camera_demo_keeps_person_and_vehicle(tmp_path):
    log = sv.load_log(synthetic_push(str(tmp_path / "p.npz")))
    fps = 10.0
    fr = sv.resample(log, sv.frame_times(log.t[0], log.t[-1], fps), window=0.1)
    pushes = sv.find_pushes(log)
    az = sv.azimuth_track(fr.t, [(p.t0, p.t1, p.dir_deg) for p in pushes])
    ft = sv.figure_track(fr.t, fr.pos, fr.push_active, fr.push_force, fr.push_point, log.ground_z, (-1.6, 0.6))
    W, H = 1280, 1080
    kw = dict(sv.DEMO_CAMERA)
    el = kw.pop("elevation_deg")
    plan = sv.plan_camera(fr, log.ground_z, fps, W / H, 0.86, "demo", az, el, extra_pts=sv.figure_points(ft), **kw)
    vp = sv.vehicle_points(fr, 0.86)
    fp = sv.figure_points(ft)
    big = 0
    for k in range(fr.n):
        cam = plan.view(k)
        for pts in (vp[k], fp[k]):
            px, z = sv.project(pts, cam, W, H)
            assert np.all(z > 0) and np.all((px >= 0) & (px <= [W, H])), k
        px, _ = sv.project(vp[k], cam, W, H)
        big += float(np.hypot(*(px[2] - px[0]))) > 150.0                  # the 0.86 m vehicle is large on screen
    assert big == fr.n and np.all(plan.framing == 3)


# --------------------------------------------------------------------------- #
# rendering (PyVista off-screen)
# --------------------------------------------------------------------------- #
@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_project_matches_vtk(tmp_path):
    """sim_video.project (used for the screen markers and inset labels) equals VTK's world -> display."""
    import pyvista as pv
    pv.OFF_SCREEN = True
    W, H = 640, 400
    pl = pv.Plotter(off_screen=True, window_size=(W, H))
    pl.add_mesh(pv.Sphere(radius=0.1))
    cam = sv.CamView(np.array([4.0, -9.0, 3.0]), np.array([0.5, 0.2, 1.0]), np.array([0.0, 0.0, 1.0]), 30.0)
    pl.camera.position = tuple(cam.position)
    pl.camera.focal_point = tuple(cam.focal)
    pl.camera.up = tuple(cam.up)
    pl.camera.view_angle = 30.0
    pl.render()
    ren = pl.renderer
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 1.5], [-1.0, 0.5, 2.5]])
    mine, _ = sv.project(pts, cam, W, H)
    for p, m in zip(pts, mine):
        ren.SetWorldPoint(float(p[0]), float(p[1]), float(p[2]), 1.0)
        ren.WorldToDisplay()
        dx, dy, _ = ren.GetDisplayPoint()
        assert m[0] == pytest.approx(dx, abs=0.6)
        assert m[1] == pytest.approx(H - dy, abs=0.6)          # VTK display y is bottom-up
    pl.close()


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_smoke_render_mp4_and_poster(tmp_path):
    import imageio_ffmpeg
    p = synthetic_vtol(str(tmp_path / "s.npz"), t_end=12.0, hz=50.0)
    out = str(tmp_path / "v" / "smoke.mp4")
    info = sv.render(p, out, fps=10.0, size=(480, 270), speed=4.0, start=0.0, end=12.0, title_s=0.3, end_s=0.3,
                     quiet=True)
    assert os.path.getsize(out) > 1000
    n, secs = imageio_ffmpeg.count_frames_and_secs(out)
    assert n == info["frames"] == 3 + 31 + 3
    gen = imageio_ffmpeg.read_frames(out)
    meta = next(gen)
    assert tuple(meta["size"]) == (480, 270)
    gen.close()
    assert os.path.exists(os.path.join(tmp_path, "v", "smoke_poster.png"))
    with open(os.path.join(tmp_path, "v", "smoke.json"), encoding="utf-8") as f:
        side = json.load(f)
    assert side["s_per_frame"] > 0 and side["log_sha256"] == sv.load_log(p).sha256
    assert side["summary"]["throttle_saturated_s"] > 0.0      # the pattern's u = 1 stretch is reported


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_cli_preview_and_missing_stl_standins(tmp_path, monkeypatch, capsys):
    """--preview through main(); with no CAD exports the renderer draws labelled stand-ins and says so."""
    p = synthetic_vtol(str(tmp_path / "cli.npz"), t_end=8.0, hz=50.0)
    monkeypatch.setattr(sv, "STL_DIR", str(tmp_path / "no_stl_here"))
    rc = sv.main(["--log", p, "--out", str(tmp_path / "cli.mp4"), "--size", "480x270", "--preview", "2"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "not found: drawn as a simple stand-in" in out
    with open(str(tmp_path / "cli_frames.json"), encoding="utf-8") as f:
        side = json.load(f)
    assert side["frames"] == 2 and len(side["files"]) == 2
    assert all(os.path.isabs(x) and os.path.exists(x) for x in side["files"])


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_push_demo_frames_korean_and_english(tmp_path):
    """--framing demo on the push pattern: the person, pole and arrow appear while a push is active (orange arrow
    pixels in the 3-D view), the sidecar carries the push table, Korean is the default and --lang en works."""
    p = synthetic_push(str(tmp_path / "pd.npz"), hz=50.0)
    info = sv.render(p, str(tmp_path / "pd.mp4"), size=(640, 360), at=[5.0, 9.15], framing="demo", quiet=True)
    assert info["lang"] == "ko" and info["figure"] is True and info["camera_elevation_deg"] == pytest.approx(8.0)
    assert [q["i"] for q in info["summary"]["pushes"]] == [1, 2, 3]
    assert info["summary"]["pushes"][0]["recovery_s"] == pytest.approx(0.86, abs=0.03)
    assert all(os.path.exists(f) for f in info["files"])
    log = sv.load_log(p)
    rnd = sv.Renderer(log, 0.0, float(log.t[-1]), 30.0, 1.0, (640, 360), "demo", tq=np.array([5.0, 9.15, 9.8]))
    try:
        vis = []
        for k in range(3):
            rnd.frame(k)
            sc = rnd.scene
            vis.append((bool(sc.pole.visibility), bool(sc.push_actor.visibility), bool(sc.push_label.GetVisibility()),
                        bool(sc.glow.visibility), bool(sc.fig_push.visibility), bool(sc.fig_idle.visibility)))
        assert vis[0] == (False, False, False, False, False, True)       # before: idle person, no pole
        assert vis[1] == (True, True, True, False, True, False)          # pushing: pole, arrow, label, push pose
        assert vis[2] == (False, False, False, True, False, True)        # 0.5 s after: the fading marker
        assert "밀기 12 N" == sc.push_label.GetInput() or rnd.lang != "ko"
    finally:
        rnd.close()
    info_en = sv.render(p, str(tmp_path / "pe.mp4"), size=(640, 360), at=[9.15], framing="demo", lang="en", quiet=True)
    assert info_en["lang"] == "en"


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_preview_frames_differ(tmp_path):
    """Off-screen frames really change (a PyVista screenshot without render() returns the previous image)."""
    p = synthetic_vtol(str(tmp_path / "pv.npz"), t_end=12.0, hz=50.0)
    info = sv.render(p, str(tmp_path / "pv.mp4"), size=(480, 270), at=[2.0, 6.0, 10.6], quiet=True)
    from PIL import Image
    imgs = [np.asarray(Image.open(f)).astype(int) for f in info["files"]]
    assert len(imgs) == 3 and imgs[0].shape == (270, 480, 3)
    W_main = sv.Layout.for_size(480, 270).main_w
    for a, b in zip(imgs, imgs[1:]):
        assert np.abs(a[:, :W_main] - b[:, :W_main]).mean() > 1.0


# --------------------------------------------------------------------------- #
# realistic style (tools/render_scene.py): pure parts + the style / HUD / camera switches
# --------------------------------------------------------------------------- #
def _rs():
    return sv._render_scene()


def test_texture_generators_deterministic_and_tileable():
    RS = _rs()
    for make in (lambda s: RS.grass_texture(256, 16.0, s), lambda s: RS.concrete_texture(128, s),
                 lambda s: RS.wood_floor_texture(256, 4.0, 0.10, s), lambda s: RS.ceiling_texture(128, 4.0, s)):
        a, b, c = make(5), make(5), make(6)
        assert a.dtype == np.uint8 and a.ndim == 3 and a.shape[2] == 3
        assert np.array_equal(a, b)                       # same seed: byte-identical
        assert not np.array_equal(a, c)
    g = RS.grass_texture(256, 16.0, 11).astype(float)
    seam = np.abs(g[:, 0] - g[:, -1]).mean()              # wrap-around neighbours
    inner = np.abs(g[:, 100] - g[:, 101]).mean()          # ordinary neighbours
    assert seam < 2.5 * inner + 2.0
    n = RS.fft_noise((64, 64), 2.0, 3)
    assert abs(float(n.mean())) < 1e-9 and float(n.std()) == pytest.approx(1.0)


def test_sky_deterministic_sun_where_asked_and_cached(tmp_path):
    RS = _rs()
    spec = RS.SkySpec("field", sun_az_deg=90.0, sun_el_deg=40.0, width=512)
    a, b = RS.sky_field(spec), RS.sky_field(spec)
    assert a.shape == (256, 512, 3) and np.array_equal(a, b)
    lum = a.astype(float).sum(axis=2)
    r, c = np.unravel_index(int(np.argmax(lum)), lum.shape)
    az = (c + 0.5) / 512 * 360.0                          # equirect: column = azimuth from +x toward +y
    el = 90.0 - (r + 0.5) / 256 * 180.0                   # row 0 = zenith
    assert abs(az - 90.0) < 3.0 and abs(el - 40.0) < 3.0
    assert a[200:, :, 1].mean() > a[200:, :, 2].mean()   # below the horizon: the (green) far field
    img, path = RS.cached_image(str(tmp_path), "sky", spec.key(), lambda: RS.sky_image(spec))

    def boom():
        raise AssertionError("cache miss")
    img2, path2 = RS.cached_image(str(tmp_path), "sky", spec.key(), boom)
    assert path == path2 and os.path.exists(path) and np.array_equal(img, img2)
    assert RS.sky_hall(RS.SkySpec("hall", width=256)).shape == (128, 256, 3)


def test_two_bone_ik_reaches_and_keeps_lengths():
    RS = _rs()
    root = np.array([0.1, -0.2, 1.4])
    for tgt in ([0.5, 0.0, 1.2], [0.2, -0.1, 0.95], [0.3, -0.5, 1.6]):
        j, e = RS.two_bone_ik(root, np.array(tgt), 0.30, 0.25, np.array([0.0, -1.0, -1.0]))
        assert np.allclose(e, tgt, atol=1e-9)
        assert np.linalg.norm(j - root) == pytest.approx(0.30) and np.linalg.norm(e - j) == pytest.approx(0.25)
        assert float((j - 0.5 * (root + e)) @ np.array([0.0, -1.0, -1.0])) > 0.0     # bends toward the pole
    far = np.array([1.5, 0.0, 1.4])
    j, e = RS.two_bone_ik(root, far, 0.30, 0.25, np.array([0.0, 0.0, -1.0]))
    assert np.linalg.norm(e - root) == pytest.approx(0.55, abs=1e-6)                 # straight toward it
    assert np.allclose(np.cross(e - root, far - root), 0.0, atol=1e-6)


def test_mannequin_hands_on_the_pole_and_proportions():
    """The pushing pose: both wrists reach their grips on the pole (POLE_GRIP_M from the held end FIGURE_HANDS,
    pole toward the vehicle along +x) with the forward lean; the relaxed figure is 1.70 m tall, feet on the ground."""
    RS = _rs()
    w = np.array([1.0, 0.0, 0.0])
    grips = [sv.FIGURE_HANDS + w * sv.POLE_GRIP_M[0], sv.FIGURE_HANDS + w * sv.POLE_GRIP_M[1]]
    J = RS.mannequin_joints(grips, lean_deg=10.0, stride=0.30)
    for side, g in (("r", grips[0]), ("l", grips[1])):
        assert np.linalg.norm(g - J[f"shoulder_{side}"]) <= RS.UPPER_ARM + RS.FOREARM
        assert np.allclose(J[f"wrist_{side}"], g, atol=1e-9)
        assert np.linalg.norm(J[f"elbow_{side}"] - J[f"shoulder_{side}"]) == pytest.approx(RS.UPPER_ARM)
    R0 = RS.mannequin_joints()
    assert R0["head_top"][2] == pytest.approx(RS.MANNEQUIN_H)
    assert min(R0["toe_r"][2], R0["heel_r"][2]) >= 0.0 and R0["ankle_l"][2] < 0.1
    assert R0["wrist_r"][2] < R0["elbow_r"][2] < R0["shoulder_r"][2]          # arms hang
    M = RS.segment_world_matrices(J, np.array([2.0, 1.0, -0.11]), 0.5)
    assert set(M) == {s[0] for s in RS.SEGMENTS}


def test_ground_effect_and_shadow_raster():
    RS = _rs()
    assert RS.ground_effect_strength(1.0, 3.0) == 0.0 and RS.ground_effect_strength(0.0, 0.2) == 0.0
    vals = [RS.ground_effect_strength(0.9, h) for h in (0.0, 0.5, 1.0, 2.0)]
    assert vals[0] == pytest.approx(0.9) and all(a > b for a, b in zip(vals, vals[1:]))
    tri = np.array([[[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]])
    m = RS.shadow_raster(tri, -0.5, -0.5, 2.0 / 255, 256, 0.0)
    area = float(m.sum()) * (2.0 / 255) ** 2
    assert area == pytest.approx(0.5, rel=0.05)
    d = np.array([0.3, 0.0, -1.0])
    p = RS.project_to_plane(np.array([[0.0, 0.0, 2.0]]), d, 0.0)
    assert np.allclose(p, [[0.6, 0.0, 0.0]])


def test_scene_profile_and_switch_validation(tmp_path):
    assert sv.scene_profile({"profile": "demo"}) == "hall"
    assert sv.scene_profile({"scenario": "V_nominal"}) == "field"
    assert sv.scene_profile({"scenario": "demo_push"}) == "hall"
    assert sv.scene_profile({"profile": "demo"}, "field") == "field"
    with pytest.raises(ValueError):
        sv.scene_profile({}, "moon")
    p = synthetic_vtol(str(tmp_path / "v.npz"), t_end=6.0, hz=50.0)
    for kw in ({"style": "cartoon"}, {"hud": "some"}, {"camera": "drone"}):
        with pytest.raises(ValueError):
            sv.render(p, str(tmp_path / "x.mp4"), size=(480, 270), at=[2.0], quiet=True, **kw)
    lay = sv.Layout.for_size(1920, 1080, full=True)
    assert lay.main_w == 1920 and sv.Layout.for_size(1920, 1080).main_w == 1280


def test_plan_spectator_eye_height_and_follow(tmp_path):
    p = synthetic_vtol(str(tmp_path / "sp.npz"), t_end=20.0, hz=50.0)
    log = sv.load_log(p)
    fr = sv.resample(log, sv.frame_times(0.0, 20.0, 10.0))
    plan = sv.plan_spectator(fr, log.ground_z, 10.0, 0.86, 90.0)
    assert np.allclose(plan.position[:, 2], log.ground_z + 1.6)
    assert np.all((plan.view_angle >= 5.0 - 1e-9) & (plan.view_angle <= 45.0 + 1e-9))
    c = fr.pos + np.einsum("nij,j->ni", fr.R, np.array([0.0, 0.0, 0.43]))
    for k in range(0, fr.n, 7):
        px, z = sv.project(c[k:k + 1], plan.view(k), 1920, 1080)
        assert z[0] > 0 and abs(px[0, 0] - 960) < 0.2 * 1920 and abs(px[0, 1] - 540) < 0.25 * 1080
    plan2 = sv.plan_spectator(fr, log.ground_z, 10.0, 0.86, 90.0)
    assert np.array_equal(plan.focal, plan2.focal)        # deterministic shake


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_realistic_style_frames_hud_modes_and_schematic(tmp_path):
    """--style realistic is the default, differs from --style schematic, keeps the sim tag (red box top right) in
    every HUD mode, and --hud minimal / none fill the frame with the 3-D view (no side panel)."""
    p = synthetic_vtol(str(tmp_path / "rs.npz"), t_end=12.0, hz=50.0)
    from PIL import Image
    out = {}
    for name, kw in (("real", {}), ("schem", {"style": "schematic"}), ("none", {"hud": "none"}),
                     ("mini", {"hud": "minimal", "camera": "spectator", "motion_blur": 2})):
        info = sv.render(p, str(tmp_path / f"{name}.mp4"), size=(640, 360), at=[6.0], quiet=True, **kw)
        assert info["style"] == kw.get("style", "realistic")
        out[name] = np.asarray(Image.open(info["files"][0])).astype(int)
    W_main = sv.Layout.for_size(640, 360).main_w
    assert np.abs(out["real"][:, :W_main] - out["schem"][:, :W_main]).mean() > 8.0
    for name, img in out.items():
        x0 = (W_main if name in ("real", "schem") else 640) - 60
        tag = img[5:12, x0:x0 + 40]
        assert tag[..., 0].mean() > 150 and tag[..., 1].mean() < 120, name           # the red simulation tag
    panel = out["real"][:, W_main + 20:]
    assert panel.mean() > 200                                # the light side panel (full HUD)
    assert out["none"][:, W_main + 20:].mean() < 200         # 3-D scenery there instead


# --------------------------------------------------------------------------- #
# v0.8: GPS position hold (hold point, GPS status, tilt setpoint, the antenna pod)
# --------------------------------------------------------------------------- #
SYNTH_GPS_TITLE = "SYNTHETIC TEST PATTERN: GPS hold hover 1.5 m (kinematic, not a simulation)"
GPS_OUT = (14.0, 16.0)          # the pattern's GPS outage (gps_ok False, 0 satellites)
GPS_TILT_DEG = 3.0              # constant lean toward +x in HOLD (moves the CG 0.39 sin 3 deg = 2 cm off the exit plane)


def synthetic_gps(path: str, hz: float = 50.0, t_end: float = 30.0, hold_shape: str = "point", drop: tuple = (),
                  tilt_sp_2d: bool = False) -> str:
    """v0.8-shaped kinematic pattern (NOT a simulation): ON, CLIMB to 1.5 m 3-6 s while drifting 0.1 m/s along +y,
    HOLD 6-24 s circling the hold point (radius 0.5 m, period 8 s) leaning 3 deg toward +x, GPS outage 14-16 s, LAND,
    LANDED.  hold_point = the CG at the first HOLD sample ([3], or the [N,3] series with ``hold_shape='series'``)."""
    t = np.round(np.arange(0.0, t_end + 1e-9, 1.0 / hz), 9)
    n = t.size
    state = np.zeros(n, dtype=np.int8)
    phase = np.zeros(n, dtype=np.int8)
    state[(t >= 1.0) & (t < 3.0)], phase[(t >= 1.0) & (t < 3.0)] = 1, 1
    state[(t >= 3.0) & (t < 27.0)] = 2
    phase[(t >= 3.0) & (t < 6.0)] = 2
    phase[(t >= 6.0) & (t < 24.0)] = 3
    phase[(t >= 24.0) & (t < 27.0)] = 4
    state[(t >= 27.0) & (t < 29.0)], phase[(t >= 27.0) & (t < 29.0)] = 3, 5
    z = np.interp(t, [0.0, 3.0, 6.0, 24.0, 27.0, 40.0], [0.0, 0.0, 1.5, 1.5, -0.11, -0.11])
    y = 0.1 * np.clip(t - 3.0, 0.0, 3.0)
    x = np.zeros(n)
    hold = (t >= 6.0) & (t < 24.0)
    ang = 2.0 * math.pi * (t - 6.0) / 8.0
    x = np.where(hold, 0.5 * np.sin(ang), x)
    y = np.where(hold, 0.3 + 0.5 * (1.0 - np.cos(ang)), y)
    y = np.where(t >= 24.0, 0.3, y)
    pos = np.column_stack([x, y, z])
    lean = np.where(hold, GPS_TILT_DEG, 0.0)
    q = _axis_angle_q(np.array([0.0, 1.0, 0.0]), np.radians(lean))
    z_cg = 0.39
    cg = pos + sv.quat_to_matrix(q)[:, :, 2] * z_cg
    i0 = int(np.flatnonzero(phase == 3)[0])
    hp = np.array([cg[i0, 0], cg[i0, 1], -0.11])
    ok = (state == 2) & ~((t >= GPS_OUT[0]) & (t < GPS_OUT[1]))
    ok |= t < 3.0
    sats = np.where(ok, 9, 0).astype(np.int16)
    tsp = np.where(hold, 2.0 + np.sin(ang), 0.0)
    meta = {"scenario": "synthetic_gps", "title": SYNTH_GPS_TITLE, "seed": 0, "mode": "vtol", "params_sha256": "n/a",
            "profile": "demo", "z_cg_m": z_cg, "gps_on": True,
            "notes": "Renderer test pattern written by tools/test_sim_video.py: kinematic curves, no physics.",
            "limits": {"u_max": 1.0, "tilt_abort_deg": 45.0, "hover_throttle": 0.92},
            "criteria": {"station_keeping_max_m": {"field": 3.0, "demo": 2.5}}}
    data = {"t": t, "pos": pos, "vel": np.gradient(pos, t, axis=0), "q": q, "delta": np.zeros((n, 4)),
            "throttle": np.where(state == 2, 0.92, 0.0),
            "thrust": np.where(state == 2, 25.0, 0.0), "state": state, "phase": phase,
            "h_target": np.where(state == 2, 1.5, np.nan), "h_ref": np.where(state == 2, z, np.nan), "tilt_deg": lean,
            "wind": np.zeros((n, 3)), "gust_active": np.zeros(n, dtype=bool), "ground_z": np.array(-0.11),
            "event_t": np.array([1.0, 3.0, 6.0, 24.0, 26.9]),
            "event_text": np.array(["SAFE -> ARMED: ON (seq 1)", "ARMED -> POWERED: spool warning complete",
                                    "POWERED: phase CLIMB -> HOLD: reference at the target 1.5 m",
                                    "POWERED: phase HOLD -> LAND: OFF (seq 2)",
                                    "touchdown #1 on the ground (z -0.11 m) at v_z -0.30 m/s, horizontal 0.10 m/s"]),
            "cmd_t": np.array([0.9, 23.9]), "cmd_name": np.array(["ON", "OFF"]), "meta": np.array(json.dumps(meta)),
            "hold_point": hp if hold_shape == "point" else np.where((t >= t[i0])[:, None], hp[None, :], np.nan),
            "gps_ok": ok, "gps_sats": sats, "gps_hacc_m": np.where(ok, 2.5, np.nan),
            "gps_fix": np.where(ok, 3, 0).astype(np.int8), "pos_est": cg[:, :2] + 0.2,
            "tilt_sp_deg": np.column_stack([tsp, np.zeros(n)]) if tilt_sp_2d else tsp}
    for k in drop:
        data.pop(k, None)
    np.savez(path, **data)
    return path


def test_load_v08_keys_hold_point_and_derived_distance(tmp_path):
    log = sv.load_log(synthetic_gps(str(tmp_path / "g.npz")))
    assert log.has_gps and log.has_hold and log.warnings == []
    t = log.t
    assert np.all(np.isnan(log.hold_xy[t < 6.0, 0])) and np.all(np.isfinite(log.hold_xy[t >= 6.0, 0]))
    # CG = pos + R e_z z_cg: the 3 deg lean toward +x moves it 0.39 sin(3 deg) along +x
    k = int(np.searchsorted(t, 10.0))
    cg = log.cg_xy()
    assert cg[k, 0] - log.pos[k, 0] == pytest.approx(0.39 * math.sin(math.radians(GPS_TILT_DEG)), abs=1e-9)
    hd = log.hold_dist()
    assert np.all(np.isnan(hd[t < 6.0]))
    assert hd[int(np.searchsorted(t, 6.0))] == pytest.approx(0.0, abs=1e-9)
    # half a circle (4 s) after the HOLD start the vehicle is the diameter 1.0 m away (lean unchanged in HOLD)
    assert hd[int(np.searchsorted(t, 10.0))] == pytest.approx(1.0, abs=2e-3)
    assert log.station_keeping_limit() == 2.5                                  # the demo entry of the dict
    assert log.tilt_sp_deg.shape == (log.n,) and log.pos_est.shape == (log.n, 3) and np.all(np.isnan(log.pos_est[:, 2]))
    # [N,3] series and [N,2] tilt components load the same
    log2 = sv.load_log(synthetic_gps(str(tmp_path / "g2.npz"), hold_shape="series", tilt_sp_2d=True))
    np.testing.assert_allclose(log2.hold_xy, log.hold_xy, equal_nan=True)
    np.testing.assert_allclose(log2.tilt_sp_deg, log.tilt_sp_deg)


def test_load_v08_partial_and_bad_keys(tmp_path):
    p = synthetic_gps(str(tmp_path / "p.npz"), drop=("hold_point", "pos_est", "tilt_sp_deg", "gps_hacc_m", "gps_fix"))
    log = sv.load_log(p)
    assert log.has_gps and not log.has_hold and log.hold_dist() is None and log.gps_hacc is None
    p2 = synthetic_gps(str(tmp_path / "q.npz"), drop=("hold_point", "pos_est", "tilt_sp_deg", "gps_hacc_m", "gps_fix",
                                                      "gps_ok", "gps_sats"))
    with np.load(p2) as d:
        data = {k: d[k] for k in d.files}
    meta = json.loads(str(data["meta"]))
    meta.pop("gps_on")
    data["meta"] = np.array(json.dumps(meta))
    np.savez(p2, **data)
    assert not sv.load_log(p2).has_gps                          # a v0.7.1-style log: no GPS, no pod
    for bad in ({"hold_point": np.zeros(4)}, {"tilt_sp_deg": np.zeros((5, 2))}, {"pos_est": np.zeros((3, 3))}):
        with np.load(p) as d:
            data = {k: d[k] for k in d.files}
        data.update(bad)
        np.savez(str(tmp_path / "bad.npz"), **data)
        with pytest.raises(ValueError):
            sv.load_log(str(tmp_path / "bad.npz"))


def test_criterion_value_and_hold_series():
    assert sv.criterion_value({"field": 3.0, "demo": 2.5}, "demo") == 2.5
    assert sv.criterion_value({"field": 3.0, "demo": 2.5}, None) == 3.0
    assert sv.criterion_value({"demo": 2.5}, "field") == 2.5                  # the only entry
    assert sv.criterion_value("x") is None and sv.criterion_value(None) is None and sv.criterion_value(4) == 4.0
    ph = np.array([0, 2, 2, 3, 3, 4])
    h = sv.hold_xy_series(np.array([1.0, 2.0, 0.0]), ph, 6)
    assert np.all(np.isnan(h[:3])) and np.allclose(h[3:], [1.0, 2.0])
    assert np.allclose(sv.hold_xy_series(np.array([1.0, 2.0, 0.0]), np.zeros(6), 6), [1.0, 2.0])    # no HOLD: all
    with pytest.raises(ValueError):
        sv.hold_xy_series(np.zeros((4, 3)), ph, 6)


def test_station_keeping_numbers(tmp_path):
    log = sv.load_log(synthetic_gps(str(tmp_path / "g.npz")))
    sk = sv.station_keeping(log)
    hold = log.phase == 3
    hd = log.hold_dist()
    assert sk["hold_rms_m"] == pytest.approx(float(np.sqrt(np.mean(hd[hold] ** 2))))
    assert sk["hold_max_m"] == pytest.approx(1.0, abs=2e-3)                  # the circle's diameter
    assert sk["hold_s"] == pytest.approx(18.0, abs=0.05) and sk["criterion_max_m"] == 2.5
    assert sk["gps_lost_s_powered"] == pytest.approx(2.0, abs=0.05)          # the 14-16 s outage
    assert sk["gps_ok_frac_powered"] == pytest.approx(1.0 - 2.0 / 24.0, abs=0.01)
    assert sk["nav_pos_err_rms_m"] == pytest.approx(math.hypot(0.2, 0.2), abs=1e-6)     # the pattern's +0.2 m estimate
    # a constant estimate offset is common to the hold point: relative to it the vehicle knows where it is exactly
    er = log.est_rel_xy()
    np.testing.assert_allclose(er[hold], log.cg_xy()[hold], atol=1e-12)
    assert np.all(np.isnan(er[log.t < 6.0])) and sk["nav_err_drift_rms_m"] == pytest.approx(0.0, abs=1e-12)
    assert sk["touchdown_from_hold_m"] is not None
    assert sv.station_keeping(sv.load_log(synthetic_vtol(str(tmp_path / "v.npz")))) is None


def test_gps_hud_rows_korean_and_tones():
    txt, tone = sv.gps_text({"ok": True, "sats": 9, "hacc": 2.5, "fix": 3}, "ko")
    assert txt == "3D 고정 · 위성 9 · 정확도 2.5 m" and tone == "ok"
    txt, tone = sv.gps_text({"ok": False, "sats": 0, "hacc": float("nan"), "fix": 0}, "ko")
    assert txt.startswith("신호 없음") and "자세만 유지" in txt and tone == "alert"
    assert sv.gps_text({"ok": True, "sats": None, "hacc": None, "fix": None}, "en") == ("usable", "ok")
    rows = sv.hud_lines(_sample(hold=0.62, hold_lim=2.5, gps={"ok": True, "sats": 9, "hacc": 2.5, "fix": 3}), "ko")
    d = {lab: (val, tone) for lab, val, tone in rows}
    assert (d["유지점"][0].strip(), d["유지점"][1]) == ("0.62 m 떨어짐", "accent") and d["GPS"][1] == "ok"
    labs = [r[0] for r in rows]
    assert labs.index("유지점") == labs.index("스로틀") + 1 and labs.index("GPS") == labs.index("유지점") + 1
    d = {lab: (val, tone) for lab, val, tone in sv.hud_lines(_sample(hold=3.1, hold_lim=2.5), "ko")}
    assert d["유지점"][1] == "warn"
    d = {lab: (val, tone) for lab, val, tone in sv.hud_lines(_sample(hold=False), "en")}
    assert d["HOLD PT"] == ("not set yet (end of the climb)", "muted")
    assert all(r[0] not in ("HOLD PT", "GPS") for r in sv.hud_lines(_sample()))        # a v0.7.1 sample: no new rows


def test_scene_profile_gps_is_outdoor():
    assert sv.scene_profile({"profile": "demo"}, gps=True) == "field"
    assert sv.scene_profile({"profile": "demo"}, "hall", gps=True) == "hall"
    assert sv.scene_profile({"profile": "demo"}) == "hall"


def test_gps_pod_paths(tmp_path, monkeypatch):
    d = tmp_path / "stl"
    d.mkdir()
    assert sv.gps_pod_paths(str(d)) == []
    (d / "nose_cone.stl").write_bytes(b"")
    (d / "x_gps_box.stl").write_bytes(b"")
    assert [os.path.basename(p) for p in sv.gps_pod_paths(str(d))] == ["x_gps_box.stl"]
    (d / "gps_pod.stl").write_bytes(b"")
    (d / "gps_pod_lid.stl").write_bytes(b"")
    assert [os.path.basename(p) for p in sv.gps_pod_paths(str(d))] == ["gps_pod.stl", "gps_pod_lid.stl"]
    monkeypatch.setattr(sv, "STL_DIR", str(d))
    assert sv.gps_pod_paths() == sv.gps_pod_paths(str(d))


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_hold_marker_and_standin_pod_geometry():
    m = sv.hold_marker_mesh()
    r = np.hypot(m.points[:, 0], m.points[:, 1])
    assert r.max() <= sv.HOLD_CROSS_M * 1.01 + 0.02 and np.allclose(m.points[:, 2], 0.0)
    # the ticks never overlap the ring (a translucent mark has no double-blended spots)
    faces = np.asarray(m.faces).reshape(-1, 5)[:, 1:]
    ring_n = 96
    for f in faces[ring_n:]:
        rr = np.hypot(m.points[f, 0], m.points[f, 1])
        assert rr.max() < sv.HOLD_RING_M - 0.011 or rr.min() > sv.HOLD_RING_M + 0.011
    pod = sv.gps_pod_standin()
    c = np.asarray(pod.center)
    assert math.degrees(math.atan2(c[1], c[0])) % 360.0 == pytest.approx(192.0, abs=0.5)
    assert 0.679 < pod.bounds[4] and pod.bounds[5] < 0.701                   # under the nose skirt (z 701.3 mm)


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_v08_render_hold_marker_pod_and_hud(tmp_path, monkeypatch):
    """A GPS log: the pod is drawn (stand-in + its own tag when the STL is missing), the hold mark and the error line
    appear from the start of HOLD (not before; hidden with --hud none), the HUD carries the new rows, the field is
    chosen although the profile is demo, the tilt plot has the setpoint and a hold-distance plot is added."""
    p = synthetic_gps(str(tmp_path / "g.npz"))
    monkeypatch.setattr(sv, "STL_DIR", str(tmp_path / "only_body"))           # no STLs at all: stand-ins everywhere
    log = sv.load_log(p)
    rnd = sv.Renderer(log, 0.0, float(log.t[-1]), 30.0, 1.0, (640, 360), "demo", tq=np.array([4.0, 10.0, 15.0]),
                      style="schematic")
    try:
        assert rnd.profile == "field" and rnd.with_gps_pod and rnd.n_gps_rows == 2
        assert len(rnd.scene.pod_actors) == 1 and rnd.scene.pod_source.startswith("stand-in")
        assert any(w.startswith("GPS pod STL not found") for w in rnd.warnings)
        assert rnd.standin_tag() == sv.tr("standin", "ko")                     # body STLs missing too: the general tag
        vis = []
        for k in range(3):
            img = rnd.frame(k)
            assert img.shape == (360, 640, 3)
            vis.append((bool(rnd.scene.hold_actor.visibility), bool(rnd.scene.hold_line_actor.visibility)))
        assert vis == [(False, False), (True, True), (True, True)]
        h = rnd.overlay_rects_px()["hud"]
        assert h[3] - h[1] == int((280 + 44) * (360 / 1080))                   # two v0.8 rows: the box grows
        assert len(rnd.panel.boxes) == 5                                       # + the hold-distance plot
        sk = rnd.summary["station_keeping"]
        assert sk["hold_max_m"] == pytest.approx(1.0, abs=2e-3) and rnd.summary["gps"] is True
    finally:
        rnd.close()
    info = sv.render(p, str(tmp_path / "n.mp4"), size=(640, 360), at=[10.0], hud="none", style="schematic", quiet=True)
    assert info["gps_pod"] is not None and info["scene"] == "field"
    off = sv.render(p, str(tmp_path / "o.mp4"), size=(640, 360), at=[10.0], gps_pod="off", style="schematic", quiet=True)
    assert off["gps_pod"] is None


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_v08_pod_tag_only_pod_missing_and_ghost(tmp_path, monkeypatch):
    """With the body STLs present and only the pod missing, the on-screen tag names the pod; a GhostVehicle follows its
    own frames (index k) and is drawn translucent."""
    if not os.path.exists(os.path.join(sv.STL_DIR, "nose_cone.stl")):
        pytest.skip("CAD STL exports not present")
    d = tmp_path / "stl"
    d.mkdir()
    import shutil
    for name, _ in sv.BODY_PARTS + (sv.PAD_PART,):
        shutil.copy(os.path.join(sv.STL_DIR, f"{name}.stl"), d)
    shutil.copy(os.path.join(sv.STL_DIR, "vane.stl"), d)
    monkeypatch.setattr(sv, "STL_DIR", str(d))
    p = synthetic_gps(str(tmp_path / "g.npz"))
    log = sv.load_log(p)
    rnd = sv.Renderer(log, 0.0, float(log.t[-1]), 30.0, 1.0, (640, 360), tq=np.array([10.0]), style="schematic")
    try:
        assert rnd.standin_tag() == sv.tr("standin_pod", "ko")
        other = sv.resample(sv.load_log(synthetic_vtol(str(tmp_path / "v.npz"), t_end=12.0, hz=50.0)), np.array([10.0]))
        g = rnd.scene.add_ghost(other, label="변경 전")
        rnd.frame(0)
        assert np.allclose(g.body[0].user_matrix[:3, 3], other.pos[0])
        assert g.body[0].prop.opacity == pytest.approx(0.42) and g.label.GetVisibility()
        assert np.allclose(g.points(0)[0], other.pos[0])
    finally:
        rnd.close()


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_v071_log_unchanged_by_v08_features(tmp_path):
    """A log without the v0.8 keys gets none of the v0.8 drawing: no pod, no hold mark, the HUD box of v0.7."""
    p = synthetic_vtol(str(tmp_path / "v.npz"), t_end=12.0, hz=50.0)
    log = sv.load_log(p)
    rnd = sv.Renderer(log, 0.0, 12.0, 30.0, 1.0, (640, 360), tq=np.array([6.0]), style="schematic")
    try:
        assert not rnd.with_gps_pod and rnd.scene.pod_actors == [] and rnd.scene.hold_actor is None
        assert rnd.n_gps_rows == 0 and len(rnd.panel.boxes) == 4
        h = rnd.overlay_rects_px()["hud"]
        assert h[3] - h[1] == int(280 * (360 / 1080))
        assert rnd.summary["station_keeping"] is None and rnd.summary["gps"] is False
    finally:
        rnd.close()


# --------------------------------------------------------------------------- #
# v6 (v0.9 hardware): the vehicle the run flew, the tether, the power rows
# --------------------------------------------------------------------------- #
SYNTH_V09_TITLE = "SYNTHETIC TEST PATTERN: v0.9 keys on the demo hover (kinematic + made-up tether values)"


def synthetic_v09(path: str, hw: str | None = "vtol", power: str | None = "battery", tether_keys: bool = True,
                  disconnect_t: float | None = None, ground_z: float = -0.11, meta_keys: bool = True) -> str:
    """``synthetic_push`` with the v6 keys: hardware_config / power_source (meta, or npz keys with meta_keys False), for a
    tether / hybrid power the tether keys (anchor 3.8 m over the pad, eye at 0.4953 m, tension 1 + 0.5 sin, connected until
    ``disconnect_t``) and supply / buffer series; battery: v_cell.  The values are made up for the renderer."""
    synthetic_push(path)
    d = dict(np.load(path, allow_pickle=False))
    n, t = d["t"].size, d["t"]
    meta = json.loads(str(d["meta"]))
    meta["title"] = SYNTH_V09_TITLE
    keys = {k: v for k, v in (("hardware_config", hw), ("power_source", power)) if v is not None}
    if meta_keys:
        meta.update(keys)
    else:
        d.update({k: np.array(v) for k, v in keys.items()})
    d["meta"] = np.array(json.dumps(meta))
    d["ground_z"] = np.array(ground_z)
    if power in ("tether", "hybrid"):
        con = np.ones(n, dtype=bool) if disconnect_t is None else t < disconnect_t
        if tether_keys:
            d.update(tether_anchor_m=np.array([0.0, 0.0, 3.8]), tether_attach_body_m=np.array([0.0, 0.0, 0.4953]),
                     tether_tension_N=np.where(con, 1.0 + 0.5 * np.sin(0.7 * t), 0.0), tether_connected=con)
        d.update(supply_v_V=np.where(d["state"] == 2, 23.0, 25.2), supply_i_A=np.where(d["state"] == 2, 34.0, 0.3),
                 supply_state=np.where((t > 12.0) & (t < 13.0), 1, 0))
        if power == "hybrid":
            d.update(buffer_v_V=np.full(n, 23.4), buffer_i_A=np.where(t > 20.0, 12.0, 0.0))
    elif hw is not None:
        d["v_cell"] = np.linspace(4.15, 3.85, n)
    np.savez(path, **d)
    return path


def test_v6_hardware_of_keys_aliases_and_inference():
    H = sv.hardware_of
    assert H({}) == sv.Hardware() and H({}).legacy and sv.Hardware().label() == ""
    assert H({"hardware_config": "VTOL"}) == sv.Hardware("vtol", "battery", "meta")
    assert H({}, {"hardware_config": np.array("rocket")}) == sv.Hardware("rocket", "battery", "npz")
    assert H({"hardware_config": "vtol", "power_source": "wired_buffer"}).power == "hybrid"
    assert H({"hardware_config": "vtol", "power_source": "wired"}).tethered
    w: list[str] = []
    assert H({"hardware_config": "rocket", "power_source": "tether"}, warnings=w).power == "battery" and w
    w = []
    assert H({"power_source": "hybrid"}, warnings=w) == sv.Hardware("vtol", "hybrid", "inferred") and w
    w = []
    assert H({"hardware_config": "glider"}, warnings=w).legacy and w
    assert H({"power_source": "battery"}).legacy                      # a battery key alone says nothing new
    assert "VTOL" in H({"hardware_config": "vtol", "power_source": "tether"}).label("en")


def test_v6_load_tether_power_keys_and_cg_ref(tmp_path):
    p = synthetic_v09(str(tmp_path / "t.npz"), power="hybrid", disconnect_t=22.0, meta_keys=False)
    log = sv.load_log(p)
    assert log.hw == sv.Hardware("vtol", "hybrid", "npz")
    assert np.allclose(log.tether_anchor, [0.0, 0.0, 3.8]) and np.allclose(log.tether_attach_b, [0.0, 0.0, 0.4953])
    assert log.tether_connected.dtype == bool and not log.tether_connected[-1] and log.tether_connected[0]
    assert set(log.power) == {"supply_v", "supply_i", "supply_state", "buffer_v", "buffer_i"}
    fr = sv.resample(log, np.array([12.5, 25.0]))
    assert fr.power["supply_state"][0] == 1 and not fr.tether_connected[1]
    # pos_ref 'cg': the attach point is given from the CG and moved to the exit-plane origin
    d = dict(np.load(p, allow_pickle=False))
    m = json.loads(str(d["meta"]))
    m.update(pos_ref="cg", z_cg_m=0.25)
    d["meta"] = np.array(json.dumps(m))
    d["tether_attach_body_m"] = np.array([0.0, 0.0, 0.2453])
    q = str(tmp_path / "cg.npz")
    np.savez(q, **d)
    assert np.allclose(sv.load_log(q).tether_attach_b, [0.0, 0.0, 0.4953])
    # tether keys alone (no hardware key): the VTOL tether configuration, with a warning
    d = dict(np.load(p, allow_pickle=False))
    for k in ("hardware_config", "power_source"):
        d.pop(k)
    np.savez(q, **d)
    lg = sv.load_log(q)
    assert lg.hw.tethered and any("tether keys without" in w for w in lg.warnings)
    with pytest.raises(ValueError):
        d["tether_anchor_m"] = np.array([0.0, 3.8])
        np.savez(q, **d)
        sv.load_log(q)


def test_v6_power_hud_rows_and_tones():
    rows = sv.power_hud_rows({"tether": {"connected": True, "tension": 1.23},
                              "supply": {"v": 23.0, "i": 34.0, "state": 1.0},
                              "buffer": {"v": 23.4, "i": 12.0, "soc": 0.8}, "batt": {"v_cell": 3.9}}, "ko")
    assert [r[0] for r in rows] == ["테더", "전원", "버퍼", "배터리"]
    assert "장력  1.2 N" in rows[0][1] and rows[0][2] == "accent"
    assert "전류 제한" in rows[1][1] and rows[1][2] == "warn"
    assert "셀당 3.90 V" in rows[2][1] and "방전 중" in rows[2][1] and rows[2][2] == "warn"
    off = sv.power_hud_rows({"tether": {"connected": False, "tension": None}, "supply": {"v": None, "i": None, "state": 2},
                             "buffer": {"v": None, "i": -3.0, "soc": None}}, "en")
    assert off[0] == ("TETHER", "DISCONNECTED", "alert") and off[1][2] == "alert" and off[2][2] == "alert"
    assert sv.power_hud_rows({}, "ko") == []


def test_v6_catenary_tension_shape_and_straight_cases():
    w = sv.TETHER_W_N_PER_M
    for D, h, T in ((0.5, 2.3, 1.0), (1.2, 2.0, 3.0), (0.3, 2.5, 6.0), (2.0, 0.5, 5.0)):
        a = sv.catenary_param(D, h, T, w)
        x0 = float(sv._catenary_x0(np.array([a]), D, h)[0])
        T_end = w * a * math.cosh(x0 / a)                                  # tension at the vehicle end (x = 0)
        aa = np.geomspace(1e-3, 1e3, 400)
        T_min = float(np.min(w * aa * np.cosh(np.minimum(np.abs(sv._catenary_x0(aa, D, h) / aa), 300.0))))
        assert T_end == pytest.approx(max(T, T_min), rel=0.01)
        P, A = np.array([0.0, 0.0, 1.5]), np.array([D, 0.0, 1.5 + h])
        c = sv.tether_curve(P, A, T, w, 48)
        assert np.allclose(c[0], P) and np.allclose(c[-1], A) and c.shape == (48, 3)
        seg = np.linalg.norm(np.diff(c, axis=0), axis=1)
        assert seg.max() / seg.min() < 1.05                                # even in arc length
    # vertical and very taut: straight lines
    c = sv.tether_curve([0, 0, 1.0], [0, 0, 3.8], 1.0)
    assert np.allclose(c[:, :2], 0.0)
    assert sv.catenary_param(1.0, 1.0, 1e6, w) is None
    # a low tension sags below the chord, a high one less
    lo = sv.tether_curve([0, 0, 1.0], [1.5, 0, 3.0], 0.5)
    hi = sv.tether_curve([0, 0, 1.0], [1.5, 0, 3.0], 20.0)
    chord = lambda p: 1.0 + 2.0 * p[:, 0] / 1.5                            # noqa: E731
    assert np.max(chord(lo) - lo[:, 2]) > np.max(chord(hi) - hi[:, 2]) > 0.0


def test_v6_free_tether_falls_and_hangs():
    A = np.array([0.0, 0.0, 3.0])
    shape = np.array([A, [1.5, 0.0, 3.0]])                                 # horizontal, released from rest
    ft = sv.simulate_free_tether(shape, np.zeros(3), 0.0, 6.0, ground_z=0.0)
    end = ft.at(6.0)
    assert np.allclose(end[0], A)
    L0 = sv.polyline_length(sv.resample_polyline(shape, end.shape[0]))
    assert sv.polyline_length(end) == pytest.approx(L0, rel=0.03)          # inextensible
    assert end[-1, 2] == pytest.approx(3.0 - 1.5, abs=0.08) and abs(end[-1, 0]) < 0.15   # hangs below the anchor
    assert np.all(end[:, 2] >= 0.0)
    long = sv.simulate_free_tether(np.array([A, [0.0, 4.0, 3.0]]), np.zeros(3), 0.0, 6.0, ground_z=0.0)
    assert np.all(long.at(6.0)[:, 2] >= 0.0) and long.at(6.0)[-1, 2] < 0.05   # the excess lies on the ground
    assert np.allclose(ft.at(-1.0), ft.nodes[0])


def test_v6_choose_stand_dir_avoids_person_and_prefers_behind_right():
    u, L = sv.choose_stand_dir([0.0, 0.0], [0.0])                          # camera looks along +x
    assert u[0] > 0.6 and u[1] < -0.4 and abs(np.linalg.norm(u) - 1.0) < 1e-9   # behind (+x), screen-right (-y)
    assert L == sv.BOOM_LEN_M
    person = np.array([[sv.BOOM_LEN_M * u[0], sv.BOOM_LEN_M * u[1]]])
    u2, L2 = sv.choose_stand_dir([0.0, 0.0], [0.0], person_xy=person)
    assert float(np.hypot(*(L2 * u2[:2] - person[0]))) >= 1.4 and L2 == sv.BOOM_LEN_M
    # a person all round the vehicle at 1.2 m: even at 2.5 m the gap is 1.3 m < 1.4 -> the farthest place (longest boom)
    ring = np.array([[1.2 * math.cos(a), 1.2 * math.sin(a)] for a in np.radians(np.arange(0.0, 360.0, 20.0))])
    u3, L3 = sv.choose_stand_dir([0.0, 0.0], [0.0], person_xy=ring)
    assert L3 == sv.BOOM_LENS_M[-1]
    # people on the +y half at 1.2 m only: a clear place on the -y side within 2.0 m
    half = np.array([[1.2 * math.cos(a), 1.2 * math.sin(a)] for a in np.radians(np.arange(0.0, 181.0, 15.0))])
    u4, L4 = sv.choose_stand_dir([0.0, 0.0], [0.0], person_xy=half)
    assert L4 <= 2.0 and float(np.min(np.hypot(*(half - L4 * u4[:2]).T))) >= 1.4


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_v6_legacy_log_draws_the_pre_v09_vehicle(tmp_path):
    """No hardware key: exactly the BODY_PARTS set from STL_DIR, no servo covers / top items / interior / tether, the HUD
    box of v0.7, no 'hardware' in the sidecar."""
    p = synthetic_vtol(str(tmp_path / "v.npz"), t_end=12.0, hz=50.0)
    log = sv.load_log(p)
    assert log.hardware is None and log.hw.legacy and log.power == {}
    rnd = sv.Renderer(log, 0.0, 12.0, 30.0, 1.0, (640, 360), tq=np.array([6.0]), style="schematic")
    try:
        sc = rnd.scene
        assert len(sc.body_actors) == len(sv.BODY_PARTS) and sc.hw_actors == [] and sc.tether is None
        assert sc.stl_dir == sv.STL_DIR and sc.pad_mesh is not None and rnd.n_power_rows == 0
    finally:
        rnd.close()
    info = sv.render(p, str(tmp_path / "o.mp4"), size=(640, 360), at=[6.0], hud="none", style="schematic", quiet=True)
    assert "hardware" not in info and "tether" not in info


def _legacy_inputs_fingerprint() -> str:
    """What the pinned legacy digests depend on: the legacy STLs, the params VehicleGeom reads, the VTK version."""
    import hashlib
    import vtk
    h = hashlib.sha256(vtk.vtkVersion.GetVTKVersion().encode())
    for name in [n for n, _ in sv.BODY_PARTS] + [sv.VANE_PART, sv.PAD_PART[0]]:
        p = os.path.join(sv.STL_DIR, f"{name}.stl")
        h.update(open(p, "rb").read() if os.path.exists(p) else b"missing")
    h.update(repr(sv.VehicleGeom.from_params()).encode())
    return h.hexdigest()


# frames of synthetic_vtol (t_end 12, 50 Hz) at 6.0 / 10.6 s, 640 x 360, --hud none, rendered 2026-10-28 on the reference
# PC (Intel UHD 750, VTK 9.6.2) by the PRE-v0.9 sim_video.py (frozen copy) and, identically, by this v6 sim_video.py
# (scratchpad v09sim/renderer/pin_digests.py orig|new)
LEGACY_PIN = {
    "inputs": "fd7b055da1a96fb04436489b270a950092721b65a9895382ddff2b4cc40041ac",
    "schematic": ["d3889417cfa0a7e7f1cc7bd5eb091219c83b3fbb74e9f3cfd80f084a956e6eae",
                  "9674b8e99b7bbc6271b89359a64fcc7fe8304564cb5f0aae833788ae28713f05"],
    "realistic": ["d0e25dfdf66dc95211d1c26fc6449de18cdc336342f1036e1ad4b50943ab2ca2",
                  "bbe129c57f33ea4dfefef196967906ae85b046dbc4fb6d60a4777880443d27b8"],
}


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_v6_legacy_frames_bit_identical(tmp_path):
    """A legacy log renders byte-identically to the pre-v0.9 renderer (pinned frame digests, both styles).  The pin holds
    only for the same inputs (legacy STLs, vane / pad params, VTK version): when they change the test skips - re-pin it
    from the frozen pre-v0.9 copy with the reason, never to hide a difference."""
    import hashlib
    from PIL import Image
    if _legacy_inputs_fingerprint() != LEGACY_PIN["inputs"]:
        pytest.skip("legacy-render inputs (STLs / vane+pad params / VTK) differ from the pinned ones: re-pin needed")
    p = synthetic_vtol(str(tmp_path / "v.npz"), t_end=12.0, hz=50.0)
    for style in ("schematic", "realistic"):
        info = sv.render(p, str(tmp_path / f"{style}.mp4"), size=(640, 360), at=[6.0, 10.6], hud="none", style=style,
                         quiet=True)
        dig = [hashlib.sha256(np.asarray(Image.open(f).convert("RGB")).tobytes()).hexdigest() for f in info["files"]]
        assert dig == LEGACY_PIN[style], style


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_v6_vtol_mesh_set_top_items_interior_and_pad(tmp_path):
    if not os.path.isdir(os.path.join(os.path.dirname(sv.STL_DIR), "stl_vtol")):
        pytest.skip("VTOL STL exports not present")
    p = synthetic_v09(str(tmp_path / "b.npz"), power="battery")
    log = sv.load_log(p)
    rnd = sv.Renderer(log, 0.0, 30.0, 30.0, 1.0, (640, 360), tq=np.array([9.1]), style="schematic", figure="off")
    try:
        sc = rnd.scene
        assert os.path.basename(sc.stl_dir) == "stl_vtol" and sc.hw.vtol and sc.pod_actors == []
        # 4 modules + groove cover + tray + the four servo covers as one mesh
        assert len(sc.body_actors) == 7
        assert sc.length_m == pytest.approx(0.4753, abs=2e-3) and sc.top_z_m == pytest.approx(0.4753, abs=2e-3)
        keys = [k for k, _ in sc.hw_meshes]
        assert {"copper", "patch", "pcb"} <= set(keys) and "boot" not in keys       # battery: no eye bolt / boot
        assert {"edf", "lipo", "esc", "pcb_g"} <= {k for k, _ in sc.interior}
        fins = sc.body_actors[-1].mapper.dataset
        th = np.degrees(np.arctan2(fins.points[:, 1], fins.points[:, 0])) % 360.0
        for a in (0.0, 90.0, 180.0, 270.0):                                         # a cover at every fin
            assert np.any(np.abs(((th - a + 180.0) % 360.0) - 180.0) < 12.0)
        assert sc.pad_mesh is not None and rnd.n_power_rows == 1                   # pad (ground -0.11), battery row
        img = rnd.frame(0)
        assert img.shape == (360, 640, 3)
    finally:
        rnd.close()
    q = synthetic_v09(str(tmp_path / "g.npz"), power="battery", ground_z=0.0)
    rnd = sv.Renderer(sv.load_log(q), 0.0, 30.0, 30.0, 1.0, (640, 360), tq=np.array([9.1]), style="schematic", figure="off")
    try:
        assert rnd.scene.pad_mesh is None                                           # VTOL on the ground: no pad
    finally:
        rnd.close()


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_v6_tether_scene_hud_disconnect_and_sidecar(tmp_path):
    if not os.path.isdir(os.path.join(os.path.dirname(sv.STL_DIR), "stl_vtol_hybrid")):
        pytest.skip("VTOL STL exports not present")
    p = synthetic_v09(str(tmp_path / "h.npz"), power="hybrid", disconnect_t=22.0)
    info = sv.render(p, str(tmp_path / "h.mp4"), size=(640, 360), at=[9.1, 22.5], style="realistic", quiet=True)
    assert info["hardware"]["config"] == "vtol" and info["hardware"]["stl_dir"].endswith("stl_vtol_hybrid")
    assert "boot" in info["hardware"]["top_items"]
    te = info["tether"]
    assert te["anchor_m"] == [0.0, 0.0, 3.8] and te["assumed"] == [] and len(te["disconnects"]) == 1
    assert te["disconnects"][0]["t0"] == pytest.approx(21.99, abs=0.02)
    log = sv.load_log(p)
    rnd = sv.Renderer(log, 0.0, 30.0, 30.0, 1.0, (640, 360), tq=np.array([9.1, 22.5]), style="schematic")
    try:
        tv = rnd.scene.tether
        assert tv is not None and rnd.n_power_rows == 3                             # tether, supply, buffer
        M = sv.body_matrix(rnd.fr.pos[0], rnd.fr.q[0])
        pts, con, T = tv.shape_at(0, M)
        assert con and np.allclose(pts[-1], [0.0, 0.0, 3.8]) and np.allclose(pts[0], M[:3, :3] @ tv.attach_b + M[:3, 3])
        pts2, con2, _ = tv.shape_at(1, sv.body_matrix(rnd.fr.pos[1], rnd.fr.q[1]))
        assert not con2 and np.allclose(pts2[-1], [0.0, 0.0, 3.8])                  # free cable still on the pulley
        S = tv.anchor[:2] + tv.boom_len * tv.u[:2]
        assert float(np.min(np.hypot(*(rnd.fig_grid.feet[:, :2] - S).T))) >= 1.4    # the stand clear of the person
        rows = sv.power_hud_rows(sv.frame_power_sample(rnd.fr, 1, log), "ko")
        assert rows[0][2] == "alert" and "분리됨" in rows[0][1]
        assert rnd.frame(1).shape == (360, 640, 3)
    finally:
        rnd.close()
    # tether / hybrid without tether keys: drawn from the ASSUMED defaults, with a warning
    q = synthetic_v09(str(tmp_path / "n.npz"), power="tether", tether_keys=False)
    info = sv.render(q, str(tmp_path / "n.mp4"), size=(640, 360), at=[9.1], style="schematic", quiet=True)
    assert len(info["tether"]["assumed"]) == 3 and any("tether keys missing" in w for w in info["warnings"])


@pytest.mark.skipif(not HAVE_PV, reason="pyvista not installed")
def test_v6_ghost_draws_its_own_hardware(tmp_path):
    """A ghost whose log flew another configuration loads its own mesh set (else it copies the scene's, as before)."""
    if not os.path.isdir(os.path.join(os.path.dirname(sv.STL_DIR), "stl_vtol")):
        pytest.skip("VTOL STL exports not present")
    p = synthetic_vtol(str(tmp_path / "v.npz"), t_end=12.0, hz=50.0)
    rnd = sv.Renderer(sv.load_log(p), 0.0, 12.0, 30.0, 1.0, (640, 360), tq=np.array([6.0]), style="schematic")
    try:
        other = sv.resample(sv.load_log(synthetic_v09(str(tmp_path / "b.npz"), power="battery")), np.array([6.0]))
        same = rnd.scene.add_ghost(other)
        assert len(same.body) == len(sv.BODY_PARTS)
        g = rnd.scene.add_ghost(other, hardware=sv.Hardware("vtol", "battery", "meta"))
        assert len(g.body) == 7 and g.length_m == pytest.approx(0.4753, abs=2e-3)
        rnd.frame(0)
    finally:
        rnd.close()
