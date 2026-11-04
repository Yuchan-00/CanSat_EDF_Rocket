"""Tests of the rigged-human runtime (tools/human_figure.py) and the person timeline helpers in tools/sim_video.py.

Pure numpy (no VTK): the npz loads and is consistent, LBS is the identity at rest, the IK reaches the pole grips
within 1 cm (or says it cannot), feet stay on the floor, the head looks at its target, everything is deterministic.
Skipped when render/human/human_lbs.npz has not been built (python render/human/build_human.py).
"""
from __future__ import annotations

import math
import os
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import human_figure as HF  # noqa: E402

HAVE_NPZ = os.path.exists(HF.HUMAN_NPZ)
pytestmark = pytest.mark.skipif(not HAVE_NPZ, reason="render/human/human_lbs.npz not built")

HANDS = np.array([0.45, 0.0, 1.25])           # sim_video.FIGURE_HANDS
GRIP_M = (-0.12, 0.10)                        # sim_video.POLE_GRIP_M


@pytest.fixture(scope="module")
def rig() -> HF.Rig:
    return HF.Rig()


@pytest.fixture(scope="module")
def solver(rig: HF.Rig) -> HF.Solver:
    return HF.Solver(rig)


def push_input(feet, yaw, tip_fig, w=1.0, look=None) -> HF.PersonInput:
    """A push like sim_video's: held end at FIGURE_HANDS in the figure frame, pole to ``tip_fig``."""
    M = HF.figure_matrix(feet, yaw)
    wd = HF.unit(np.asarray(tip_fig, dtype=float) - HANDS)
    g = [(M @ np.append(HANDS + wd * m, 1.0))[:3] for m in GRIP_M]
    tip_w = (M @ np.append(tip_fig, 1.0))[:3]
    return HF.PersonInput(feet=np.asarray(feet, dtype=float), yaw=yaw, look_at=tip_w if look is None else look,
                          push_w=w, grips=(g[0], g[1]), pole_dir=M[:3, :3] @ wd)


def test_npz_loads_and_is_consistent(rig: HF.Rig):
    assert rig.B == len(rig.names) == 55 and set(rig.presets) >= {"rest", "relaxed_hands", "grip", "idle", "push_ref"}
    assert all(p < b for b, p in enumerate(rig.parents) if p >= 0) and rig.parents[0] == -1     # topological order
    for w, wi, n in ((rig.w, rig.wi, rig.verts.shape[0]), (rig.s_w, rig.s_wi, rig.s_verts.shape[0])):
        assert w.shape == wi.shape == (n, 4)
        assert np.allclose(w.sum(axis=1), 1.0, atol=1e-5) and np.all(w >= 0.0)
        assert wi.min() >= 0 and wi.max() < rig.B
    assert 30_000 <= rig.faces.shape[0] <= 60_000 and 1_000 <= rig.s_faces.shape[0] <= 4_000
    z = rig.verts[:, 2]
    assert z.min() == pytest.approx(0.0, abs=0.005) and z.max() == pytest.approx(1.75, abs=0.01)   # 1.75 m tall
    assert np.allclose(np.linalg.norm(rig.normals, axis=1), 1.0, atol=1e-4)
    for s in "LR":
        c, a = rig.grip[s]
        assert np.linalg.norm(a) == pytest.approx(1.0) and 0.03 < np.linalg.norm(c) < 0.15     # cavity in the palm
    assert rig.meta["limits_deg"]["look_yaw"] == HF.LIMITS["look_yaw"]


def test_lbs_is_identity_at_rest(rig: HF.Rig):
    G, L = rig.fk(None)
    assert np.allclose(G, rig.rest, atol=1e-12) and np.allclose(L, np.eye(3))
    v, n = rig.skin(G)
    assert np.abs(v - rig.verts).max() < 1e-5 and np.abs(n - rig.normals).max() < 1e-4
    M = HF.figure_matrix([1.0, -2.0, 0.3], 0.8)
    vw, _ = rig.skin(G, M)
    assert np.abs(vw - (rig.verts @ M[:3, :3].T + M[:3, 3])).max() < 1e-5      # rigid placement


def test_two_bone_keeps_lengths_and_reports_reach():
    S = np.array([0.0, 0.2, 1.4])
    for tgt in ([0.4, 0.0, 1.2], [0.1, 0.3, 0.95], [0.2, -0.1, 1.6]):
        E, W, ok = HF.two_bone(S, np.array(tgt), 0.30, 0.24, np.array([-1.0, 0.3, -0.5]))
        assert ok and np.allclose(W, tgt, atol=1e-9)
        assert np.linalg.norm(E - S) == pytest.approx(0.30) and np.linalg.norm(W - E) == pytest.approx(0.24)
        assert float((E - 0.5 * (S + W)) @ np.array([-1.0, 0.3, -0.5])) > 0.0      # bends toward the pole
    E, W, ok = HF.two_bone(S, S + np.array([2.0, 0.0, 0.0]), 0.30, 0.24, np.array([0.0, 0.0, -1.0]))
    assert not ok and np.linalg.norm(W - S) == pytest.approx(0.54, abs=1e-6)


@pytest.mark.parametrize("yaw,tip", [(0.0, (1.55, 0.0, 2.0)), (0.9, (1.50, 0.25, 1.70)), (-2.4, (1.45, -0.2, 2.25)),
                                     (2.0, (1.55, 0.1, 1.40))])
def test_push_ik_puts_both_grip_cavities_on_the_pole(rig: HF.Rig, solver: HF.Solver, yaw, tip):
    feet = np.array([0.7, -1.1, -0.11])
    inp = push_input(feet, yaw, np.array(tip))
    G, _ = solver.solve(HF.person_params(rig, inp))
    M = HF.figure_matrix(feet, yaw)
    for s, g in (("R", inp.grips[0]), ("L", inp.grips[1])):
        assert solver.last[s].reached
        c_h, a_h = rig.grip[s]
        Gh = M @ G[rig.ix[f"hand_{s}"]]
        centre = (Gh @ np.append(c_h, 1.0))[:3]
        assert np.linalg.norm(centre - g) < 0.01                                        # within 1 cm (exact here)
        axis = Gh[:3, :3] @ a_h
        assert abs(float(axis @ inp.pole_dir)) > 0.999                                  # the pole runs through the fist
    # the fingers wrap the pole without cutting into it: hand skin stays outside radius - 3 mm near the grips
    v, _ = rig.skin(G, M)
    hand_ids = [rig.ix[f"hand_{s}"] for s in "LR"] + [rig.ix[n] for s in "LR" for n in HF.finger_bones(s)]
    hv = v[np.isin(rig.wi[:, 0], hand_ids) & (rig.w[:, 0] > 0.5)]
    p0, d = inp.grips[0], inp.pole_dir
    rel = hv - p0
    t = rel @ d
    near = (t > -0.08) & (t < 0.30)
    dist = np.linalg.norm(rel[near] - np.outer(t[near], d), axis=1)
    assert dist.min() > rig.grip_radius - 0.003


def test_unreachable_grip_is_reported_not_faked(rig: HF.Rig, solver: HF.Solver):
    inp = push_input(np.zeros(3), 0.0, np.array([1.55, 0.0, 2.0]))
    far = (inp.grips[0] + np.array([1.5, 0.0, 0.0]), inp.grips[1] + np.array([1.5, 0.0, 0.0]))
    inp.grips = far
    G, _ = solver.solve(HF.person_params(rig, inp))
    assert not solver.last["R"].reached and not solver.last["L"].reached
    assert np.all(np.isfinite(G))
    for s in "LR":                                     # bone lengths kept even when the target is out of reach
        a = G[rig.ix[f"upperarm_{s}"], :3, 3]
        b = G[rig.ix[f"forearm_{s}"], :3, 3]
        assert np.linalg.norm(b - a) == pytest.approx(solver.arm_l[s][0], abs=1e-5)   # float32 rest data


@pytest.mark.parametrize("which", ["idle", "push"] + [f"walk{k}" for k in range(8)])
def test_feet_on_the_floor(rig: HF.Rig, solver: HF.Solver, which):
    if which == "idle":
        inp = HF.PersonInput(feet=np.zeros(3), yaw=0.0, look_at=np.array([2.0, 1.0, 1.5]), t=3.3)
    elif which == "push":
        inp = push_input(np.zeros(3), 0.0, np.array([1.55, 0.0, 2.0]))
    else:
        inp = HF.PersonInput(feet=np.zeros(3), yaw=0.0, walk_w=1.0, walk_phase=int(which[4:]) * math.pi / 4,
                             walk_dir=np.array([1.0, 0.0]))
    G, _ = solver.solve(HF.person_params(rig, inp))
    v, _ = rig.skin(G)
    for s in "LR":
        ids = [rig.ix[f"foot_{s}"], rig.ix[f"toe_{s}"]]
        fv = v[np.isin(rig.wi[:, 0], ids) & (rig.w[:, 0] > 0.6)]
        assert fv[:, 2].min() > -0.012                                   # never through the floor
        if not which.startswith("walk"):
            assert fv[:, 2].min() < 0.012                                # standing feet touch it
    # the standing figure's head stays near its height (no crouch / stretch from the IK)
    top = G[rig.ix["head"]] @ np.array([0.0, rig.lengths[rig.ix["head"]], 0.0, 1.0])
    assert 1.55 < top[2] < 1.77


def test_head_looks_at_the_target(rig: HF.Rig, solver: HF.Solver):
    for tgt in (np.array([3.0, 1.2, 2.0]), np.array([2.0, -1.0, 1.2])):
        G, _ = solver.solve(HF.person_params(rig, HF.PersonInput(feet=np.zeros(3), yaw=0.0, look_at=tgt)))
        H = G[rig.ix["head"]]
        eye = H[:3, 3] + H[:3, :3] @ (rig.rest[rig.ix["head"], :3, :3].T @ solver.eye_off)
        face = H[:3, 2]                                              # the head bone's Z = forward (roll hint)
        ang = math.degrees(math.acos(float(face @ HF.unit(tgt - eye))))
        assert ang < 12.0
    G, _ = solver.solve(HF.person_params(rig, HF.PersonInput(feet=np.zeros(3), yaw=0.0,
                                                               look_at=np.array([-3.0, 0.1, 1.6]))))  # behind: clamped
    yaw = math.degrees(math.atan2(G[rig.ix["head"], 1, 2], G[rig.ix["head"], 0, 2]))
    assert abs(yaw) <= HF.LIMITS["look_yaw"] + 15.0


def test_deterministic_and_history_free(rig: HF.Rig, solver: HF.Solver):
    a = push_input(np.array([0.3, 0.2, 0.0]), 0.4, np.array([1.5, 0.1, 1.9]), w=0.6)
    b = HF.PersonInput(feet=np.zeros(3), yaw=1.0, walk_w=0.8, walk_phase=2.0, walk_dir=np.array([0.6, 0.8]))
    G1, _ = solver.solve(HF.person_params(rig, a))
    v1, n1 = rig.skin(G1)
    solver.solve(HF.person_params(rig, b))
    G2, _ = HF.Solver(rig).solve(HF.person_params(rig, a))
    v2, n2 = rig.skin(G2)
    assert np.array_equal(G1, G2) and np.array_equal(v1, v2) and np.array_equal(n1, n2)


def test_blend_is_continuous(rig: HF.Rig, solver: HF.Solver):
    """Push weight 0 -> 1 in small steps moves every vertex by at most a few cm per step (no pops)."""
    prev = None
    for w in np.linspace(0.0, 1.0, 11):
        G, _ = solver.solve(HF.person_params(rig, push_input(np.zeros(3), 0.0, np.array([1.55, 0.0, 2.0]), w=w)))
        v = rig.skin_shadow(G).reshape(-1, 3)
        if prev is not None:
            assert np.abs(v - prev).max() < 0.12
        prev = v


def test_pose_and_skin_cost(rig: HF.Rig, solver: HF.Solver):
    inp = push_input(np.array([1.0, 2.0, -0.11]), 0.7, np.array([1.55, 0.1, 2.0]))
    M = HF.figure_matrix(inp.feet, inp.yaw)
    ts = []
    for _ in range(12):
        t0 = time.perf_counter()
        G, _ = solver.solve(HF.person_params(rig, inp))
        rig.skin(G, M)
        rig.skin_shadow(G, M)
        ts.append(time.perf_counter() - t0)
    assert float(np.median(ts)) < 0.060          # ~13 ms on the dev PC; generous for a loaded CI machine


def test_person_timeline_helpers():
    import sim_video as sv
    pushes = [(9.0, 9.3, 900, 930), (14.0, 14.24, 1400, 1424)]
    assert sv.person_push_weight(pushes, 5.0) == (0.0, None)
    w, j = sv.person_push_weight(pushes, 9.1)
    assert w == 1.0 and j == 900
    w1, j1 = sv.person_push_weight(pushes, 8.85)
    w2, _ = sv.person_push_weight(pushes, 8.95)
    assert 0.0 < w1 < w2 < 1.0 and j1 == 900                        # rising before the push, pole of its start
    w3, j3 = sv.person_push_weight(pushes, 9.45)
    assert 0.0 < w3 < 1.0 and j3 == 930                             # falling after it, pole of its end
    assert sv.person_push_weight(pushes, 9.61)[0] == 0.0
