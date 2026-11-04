"""Frame plan (pure numpy): everything Blender needs per video frame, computed from a log + a shot list.

The plan is the only channel from the log to the 3-D scene: Blender scripts read ``plan.npz`` and key
what it says; they never read the log.  Per frame k (Blender frame k + 1):

  vehicle   pos (exit-plane centre, world m), q (w,x,y,z body->world), vane angles delta (rad), thrust (N)
  rotor     spin 0..1 (blades -> blur disc), angle (rad; cosmetic, only seen at spin ~ 0)
  jet       impact point on the ground along the body axis, centreline speed there (m/s), half-width (m),
            wall-jet constant K = sqrt(T / rho) (m^2/s; radial wall-jet speed u(r) = min(u_ground, K / r)),
            dust 0..1 - from the free-jet / wall-jet model in ``jet_state`` (formulas there).  No heat haze: an EDF
            exhaust is only a few kelvin above ambient (the ``haze`` array stays in the contract, all zeros)
  wind      mean wind (m/s, the air's velocity) and gust part; windsock yaw + per-segment droop (rad)
  person    feet (planted through every push), yaw, push weight, walking, the rigid pole (held end, direction, pad
            snapped tangent to the body while pushing), grips, look-at point; indoors also the safety officer at the
            KILL switch (meta 'kill_operator')
  camera    one rig per shot: position, look-at target, roll, lens (mm), focus distance (m), f-stop, exposure offset
            (EV, ``shot_ev``: the matte black body exposed to read black in every shot), lens shift (level cameras keep
            verticals vertical: push two-shots, the telephoto establishing shot, composed locked-off stills)
  scene     windsock visibility per frame (never a bare mast in frame), indoors the hall fitted to the WHOLE log (one
            hall for every shot list), the stage key spot and the white balance (meta)

Conventions: world z up, origin = pad top = exit plane at lift-off (tools/sim_video.py contract), so a
logged ``pos`` maps straight onto the vehicle's root.  Rendered effects that the simulation does not
model (dust, grass bending, heat haze, windsock flutter, the person's pose) are *illustrations* driven by
logged values through the stated formulas; they are labelled as such in the sidecar.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

from .logio import quat_to_matrix
from .shots import Shot, smoothstep, timeline

SENSOR_W_MM = 36.0
G = 9.81
RHO_AIR = 1.2
Z_CG_DEFAULT = 0.376
VEH_LEN = 0.856
BODY_R = 0.060


# --------------------------------------------------------------------------- #
# small maths
# --------------------------------------------------------------------------- #
def interp(t: np.ndarray, y: np.ndarray, tq: np.ndarray) -> np.ndarray:
    y = np.asarray(y, dtype=float)
    if y.ndim == 1:
        return np.interp(tq, t, y)
    return np.stack([np.interp(tq, t, y[:, j]) for j in range(y.shape[1])], axis=1)


def hold(t: np.ndarray, y: np.ndarray, tq: np.ndarray) -> np.ndarray:
    i = np.clip(np.searchsorted(t, tq, side="right") - 1, 0, t.size - 1)
    return np.asarray(y)[i]


def qnorm(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return q / np.maximum(np.linalg.norm(q, axis=-1, keepdims=True), 1e-12)


def slerp(q0: np.ndarray, q1: np.ndarray, s: np.ndarray) -> np.ndarray:
    q0, q1 = qnorm(q0), qnorm(q1)
    s = np.asarray(s, dtype=float)[..., None]
    d = np.sum(q0 * q1, axis=-1, keepdims=True)
    q1 = np.where(d < 0, -q1, q1)
    d = np.abs(d)
    th = np.arccos(np.clip(d, -1, 1))
    st = np.sin(th)
    near = d > 0.9995
    st = np.where(near, 1.0, st)
    a = np.where(near, 1 - s, np.sin((1 - s) * th) / st)
    b = np.where(near, s, np.sin(s * th) / st)
    return qnorm(a * q0 + b * q1)


def slerp_resample(t: np.ndarray, q: np.ndarray, tq: np.ndarray) -> np.ndarray:
    i = np.clip(np.searchsorted(t, tq, side="right") - 1, 0, t.size - 2)
    s = np.clip((tq - t[i]) / (t[i + 1] - t[i]), 0.0, 1.0)
    return slerp(q[i], q[i + 1], s)


def hemisphere(q: np.ndarray) -> np.ndarray:
    q = np.array(q, dtype=float)
    for i in range(1, q.shape[0]):
        if float(np.dot(q[i], q[i - 1])) < 0:
            q[i] = -q[i]
    return q


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 rotation -> (w, x, y, z)."""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    return qnorm(np.array(q))


def ema(x: np.ndarray, t: np.ndarray, tau: float) -> np.ndarray:
    """Causal first-order low-pass over (possibly irregular) times t, applied twice (critically damped feel)."""
    x = np.asarray(x, dtype=float)
    if x.shape[0] == 0 or tau <= 0:
        return x.copy()
    y = x.copy()
    for _ in range(2):
        z = y.copy()
        for k in range(1, y.shape[0]):
            a = 1.0 - math.exp(-max(float(t[k] - t[k - 1]), 0.0) / tau)
            z[k] = z[k - 1] + a * (y[k] - z[k - 1])
        y = z
    return y


def gauss_smooth(x: np.ndarray, sigma: float) -> np.ndarray:
    """Zero-phase Gaussian smoothing along axis 0 (sigma in samples), edge-padded."""
    x = np.asarray(x, dtype=float)
    if sigma <= 0.3 or x.shape[0] < 3:
        return x.copy()
    r = int(math.ceil(3 * sigma))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    pad = np.concatenate([np.repeat(x[:1], r, axis=0), x, np.repeat(x[-1:], r, axis=0)])
    if x.ndim == 1:
        return np.convolve(pad, k, mode="valid")
    return np.stack([np.convolve(pad[:, j], k, mode="valid") for j in range(x.shape[1])], axis=1)


# --------------------------------------------------------------------------- #
# vanes (same convention as tools/sim_video.vane_local_matrix)
# --------------------------------------------------------------------------- #
def vane_euler(theta: float, delta: float) -> tuple[float, float, float]:
    """Blender Euler XYZ of the hinge empty of a vane at ``theta`` deflected by ``delta`` (rad):
    R = Rz(theta) Rx(-delta) (XYZ order applies X first), i.e. (-delta, 0, theta)."""
    return (-float(delta), 0.0, float(theta))


def euler_xyz_matrix(e: Sequence[float]) -> np.ndarray:
    a, b, c = e
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    Rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def vane_world_matrix(theta: float, delta: float, hinge_z: float, theta_stl: float = 0.0) -> np.ndarray:
    """4x4 of the Blender hierarchy (hinge empty at (0,0,hinge_z) with ``vane_euler``, mesh baked as
    T(0,0,-hinge_z) Rz(-theta_stl)) - must equal sim_video.vane_local_matrix (checked in the tests)."""
    H = np.eye(4)
    H[:3, :3] = euler_xyz_matrix(vane_euler(theta, delta))
    H[:3, 3] = (0.0, 0.0, hinge_z)
    B = np.eye(4)
    c, s = math.cos(-theta_stl), math.sin(-theta_stl)
    B[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    B[2, 3] = -hinge_z
    return H @ B


# --------------------------------------------------------------------------- #
# exhaust jet (illustration model)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class JetParams:
    duct_r: float = 0.045
    hub_frac: float = 0.45
    core_d: float = 6.2           # potential-core length in exit diameters (round free jet)
    spread: float = 0.10          # jet half-width growth per metre of axial distance
    dust_lo: float = 6.0          # ground centreline speed (m/s) where dust starts ...
    dust_hi: float = 22.0         # ... and where it is at full strength


def jet_state(pos: np.ndarray, R: np.ndarray, thrust: np.ndarray, ground_z: float, jp: JetParams = JetParams(),
              max_len: float = 12.0) -> dict[str, np.ndarray]:
    """Per frame: exit speed v0 = sqrt(T / (rho A)), A = pi r^2 (1 - hub^2) (momentum flux = thrust); axial distance
    x_g from the exit plane to the ground along the exhaust axis -R[:,2]; centreline speed u = v0 inside the
    potential core (x < 6.2 D) else v0 * 6.2 D / x (round free-jet decay); half-width b = r + 0.1 x.  The ground is
    reached only when the axis points downward (axis_z < -0.2)."""
    pos = np.asarray(pos, float)
    T = np.maximum(np.asarray(thrust, float), 0.0)
    A = math.pi * jp.duct_r ** 2 * (1.0 - jp.hub_frac ** 2)
    v0 = np.sqrt(T / (RHO_AIR * A))
    axis = -R[:, :, 2]
    D = 2.0 * jp.duct_r
    down = axis[:, 2] < -0.2
    xg = np.where(down, (ground_z - pos[:, 2]) / np.where(down, axis[:, 2], -1.0), np.inf)
    xg = np.where(xg < 0, np.inf, xg)
    x_eff = np.minimum(xg, max_len)
    u = np.where(x_eff < jp.core_d * D, v0, v0 * jp.core_d * D / np.maximum(x_eff, 1e-6))
    hit = np.isfinite(xg) & (xg < max_len)
    u_g = np.where(hit, u, 0.0)
    impact = pos + axis * np.where(hit, xg, 0.0)[:, None]
    impact[:, 2] = np.where(hit, ground_z, impact[:, 2])
    b = jp.duct_r + jp.spread * np.where(hit, xg, max_len)
    dust = np.asarray(smoothstep((u_g - jp.dust_lo) / (jp.dust_hi - jp.dust_lo)))
    # radial wall jet after impingement: the jet's momentum flux (= thrust) is conserved, the wall jet's peak speed
    # falls ~1/r: u(r) = K / r with K = sqrt(T_n / rho) (T_n = thrust x the axis' downward cosine).  Check against the
    # impinging-jet correlation u_m / U_j = 1.03 (r / D)^-1.12 (Poreh et al. 1967): 64 m/s, D 90 mm, r 0.78 m ->
    # 5.9 m/s; K / r = sqrt(25 / 1.2) / 0.78 = 5.9 m/s.
    K = np.where(hit, np.sqrt(T * np.clip(-axis[:, 2], 0.0, 1.0) / RHO_AIR), 0.0)
    return {"v0": v0, "axis": axis, "x_ground": np.where(hit, xg, np.nan), "u_ground": u_g, "impact": impact,
            "half_width": b, "hit": hit, "dust": dust, "length": np.where(hit, xg, np.minimum(max_len, 0.25 * v0)),
            "wall_k": K}


def wall_jet_speed(r: np.ndarray, u_ground: np.ndarray, b: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Peak radial wall-jet speed (m/s) at distance r (m) from the impact point: min(u_ground, K / max(r, b))
    (``jet_state``); 0 where the jet does not reach the ground (K = 0)."""
    r = np.asarray(r, float)
    return np.minimum(np.asarray(u_ground, float), np.asarray(K, float) / np.maximum(r, np.asarray(b, float)))


TAPE_HALF = 0.80                      # indoor keep-out tape square half-size (render/blender/env.TAPE_HALF)
TAPE_SEGS, TAPE_SEG_M = 6, 0.04       # the peeled-up free end of the hazard tape: 6 x 4 cm


def tape_flutter(t: np.ndarray, u: np.ndarray, phase0: float = 0.3) -> np.ndarray:
    """(N, TAPE_SEGS) bend angles (rad, + = up) of the loose hazard-tape end in the local wall-jet speed u (m/s):
    the root segment lifts 55 deg (1 - exp(-u / 4.5)), the rest curl back by 18 % of that, and every segment flutters
    +-(10 + 5 i) deg (1 - exp(-u / 5)) at 6 + 0.5 u Hz (<= 14 Hz) as a wave running to the tip.  u = 0: flat."""
    u = np.clip(np.asarray(u, float), 0.0, None)
    lift = np.radians(55.0) * (1.0 - np.exp(-u / 4.5))
    f = np.minimum(6.0 + 0.5 * u, 14.0)
    ph = 2 * math.pi * np.cumsum(np.concatenate([[0.0], np.diff(t) * f[1:]]))
    out = np.zeros((t.size, TAPE_SEGS))
    for i in range(TAPE_SEGS):
        amp = np.radians(10.0 + 5.0 * i) * (1.0 - np.exp(-u / 5.0))
        out[:, i] = (lift if i == 0 else -0.18 * lift) + amp * np.sin(ph - 1.1 * i + phase0)
    return out


def tape_end_site(feet_xy: np.ndarray | None, half: float = TAPE_HALF) -> tuple[np.ndarray, np.ndarray]:
    """(root xy, unit direction) of the loose tape end: the tape-square corner farthest from every operator position,
    the free end pointing diagonally outward (the wall jet blows it away from the pad)."""
    corners = np.array([[half, half], [-half, half], [-half, -half], [half, -half]])
    if feet_xy is None or len(feet_xy) == 0:
        c = corners[0]
    else:
        fx = np.asarray(feet_xy, float).reshape(-1, 2)
        c = max(corners, key=lambda p: float(np.min(np.hypot(*(fx - p).T))))
    return c.copy(), c / np.linalg.norm(c)


# --------------------------------------------------------------------------- #
# windsock (illustration model)
# --------------------------------------------------------------------------- #
N_SOCK = 5
SOCK_MAST_H = 6.0             # airfield-type 6 m mast: the sock flies near the hover height, so a telephoto
#                               establishing shot frames both at a useful size (render/blender/props.build_windsock)
SOCK_FULL_MPS = 7.7           # a standard windsock is fully extended at ~15 kt


def windsock(t: np.ndarray, wind: np.ndarray, phase0: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """(yaw (N,), droop (N, N_SOCK)) of the windsock: it points where the air goes (atan2 of the low-passed
    horizontal wind, 0.6 s), extension s = clip(|w| / 7.7 m/s, 0, 1); root droop below horizontal
    75 deg (1 - s)^1.5 + 4 deg, each further segment +9 deg (1 - s) + 1.5 deg, flutter 1 + 3 s deg at 2.2 Hz travelling
    down the sock.  Calm air: hangs ~79 deg."""
    w = np.asarray(wind, float)
    wl = ema(w[:, :2], t, 0.6)
    sp = np.hypot(wl[:, 0], wl[:, 1])
    yaw = np.unwrap(np.arctan2(wl[:, 1], wl[:, 0]))
    # keep the last direction when the air is calm (no spinning sock)
    calm = sp < 0.15
    if np.any(~calm):
        last = yaw[np.flatnonzero(~calm)[0]]
        for k in range(yaw.size):
            if calm[k]:
                yaw[k] = last
            else:
                last = yaw[k]
    else:
        yaw[:] = phase0
    s = np.clip(sp / SOCK_FULL_MPS, 0.0, 1.0)
    droop = np.zeros((t.size, N_SOCK))
    root = np.radians(75.0) * (1.0 - s) ** 1.5 + np.radians(4.0)
    for i in range(N_SOCK):
        amp = np.radians(1.0 + 3.0 * s) * (s > 0.05)
        fl = amp * np.sin(2 * math.pi * 2.2 * t - 0.6 * i + phase0)
        # each further segment bends down 9 deg (1 - s) + 1.5 deg of fabric sag: a curved sock, not a rigid cone
        droop[:, i] = (root if i == 0 else 0.0) + (np.radians(9.0) * (1.0 - s) + np.radians(1.5) if i else 0.0) + fl
    return yaw, droop


# --------------------------------------------------------------------------- #
# person with a padded pole (USER DECISION 2: the CC0 human of render/human; pose is an illustration)
# --------------------------------------------------------------------------- #
# Pole geometry (art review round 2).  The pole is held at CHEST height, the rear (right) fist at the lower ribs and the
# front (left) fist ahead of the chest (grip centres ~1.13 / 1.30 m above the floor), and pitches up 20-25 deg to the
# contact (a 1.5 m hover's contact sits ~2.0 m above the floor).  Round 1 shouldered it at 1.42 m within 15 deg of
# horizontal: the rear fist sat at the chin and the pole crossed the mouth on screen.  A pitched pole is fine for a
# horizontal push: the pad meets the cylindrical body with a horizontal normal and a rubber / foam pad with mu >= 0.5
# carries the pole's axial vertical component (tan 25 deg = 0.47).  It is also closer to tools/sim_video.py's hands at
# 1.25 m / 34 deg pole, so the PyVista and Blender videos agree.  Grip reach, elbows, knees and the head / face
# clearances were probed with tools/human_figure + render.pose (render scratch grip_probe_r2: both grips reached, rear
# elbow ~115 deg, front ~30-45 deg, pole 0.35 m clear of the skull, fists 0.40 m from the face).
FIGURE_HANDS = np.array([0.40, -0.12, 1.22])  # figure frame (x forward, y left, z up): the pole's held end at push start
POLE_GRIP_M = (-0.24, 0.20)                  # rear (right) / front (left) grip along the pole from the held end
POLE_BUTT_M = 0.33                           # pole behind the held end (2.33 m butt -> pad centre)
POLE_PAD_R = 0.045                           # foam pad (ball) radius; the pad centre sits this far from the contact
POLE_HOLD_TO_PAD = 2.00                      # held end -> pad centre: the pole is RIGID (2.33 m butt -> pad centre)
POLE_LEN = POLE_BUTT_M + POLE_HOLD_TO_PAD
POLE_R = 0.012                               # 24 mm tube: the human's grip cavities are fitted to it
POLE_SLEEVE = (0.62, 2.22)                   # foam sleeve on the tube (m from the butt, ahead of the front grip to the
#                                              pad): the padded pole of the demo rules (params.json mission.vtol demo)
POLE_MAX_PITCH_DEG = 25.0                    # the pole's elevation is clamped to +-25 deg (pad friction carries the rest)
PUSH_BLEND_S = 0.5                           # arms bring the pole up / down over this time around a push
CARRY_HOLD = np.array([0.32, -0.14, 1.18])   # carry between pushes: the pole held upright at the ready ...
CARRY_DIR = np.array([0.2, -0.3, 0.93])      # ... pad ~2.9 m up beside the right shoulder, clear of the vehicle
MIN_STANDOFF_M = 1.6                         # the operator's feet never come closer to the vehicle axis than this
WALK_S = 2.0                                 # the operator walks to the next push position in the 2 s before its blend
FLOOR_DZ = {True: -0.018, False: -0.008}     # floor below the log's ground_z: indoor stage (18 mm plywood board under
#                                              the pad), outdoor soil (pavers 8 mm proud) - render/blender/env.py


@dataclass
class PersonTrack:
    feet: np.ndarray            # (N, 3) between the feet, on the floor
    yaw: np.ndarray             # (N,) facing (figure +x), rad
    push_w: np.ndarray          # (N,) 0 relaxed carry .. 1 pushing (smoothstep over PUSH_BLEND_S)
    walk_w: np.ndarray          # (N,) 0..1
    walk_phase: np.ndarray      # (N,) rad, 2 pi per 1.15 m travelled (tools/human_figure.WALK_STRIDE_M)
    walk_dir: np.ndarray        # (N, 2) walking direction in the FIGURE frame
    hold: np.ndarray            # (N, 3) the pole's held end (world)
    pole_dir: np.ndarray        # (N, 3) unit, butt -> pad
    pad: np.ndarray             # (N, 3) pad centre
    butt: np.ndarray            # (N, 3)
    grip_r: np.ndarray          # (N, 3) rear / right grip centre
    grip_l: np.ndarray          # (N, 3) front / left grip centre
    look: np.ndarray            # (N, 3) head look-at (the vehicle CG)
    notes: list[str] = field(default_factory=list)


def visual_contact(contact: np.ndarray, force: np.ndarray, pos: np.ndarray, R: np.ndarray, body_r: float = BODY_R) -> np.ndarray:
    """The logged contact point moved out to the body surface (against the force, perpendicular to the body axis)
    when it lies inside the body (same rule as sim_video.visual_contact)."""
    ax = R[:, 2]
    rel = contact - pos
    radial = rel - ax * float(rel @ ax)
    if float(np.linalg.norm(radial)) >= 0.9 * body_r:
        return np.asarray(contact, float)
    fp = force - ax * float(force @ ax)
    nf = float(np.linalg.norm(fp))
    if nf < 1e-9:
        return np.asarray(contact, float)
    return contact - radial - fp / nf * body_r


def _rot2(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _unit(v: np.ndarray) -> np.ndarray:
    return np.asarray(v, float) / max(float(np.linalg.norm(v)), 1e-12)


def push_weight(t: np.ndarray, act: np.ndarray, blend_s: float = PUSH_BLEND_S) -> tuple[np.ndarray, np.ndarray]:
    """(weight, reference sample) per sample: 1 while pushing, smoothstep up over ``blend_s`` before a push and down
    after it (sim_video.person_push_weight with a longer blend); the reference is the push's first / last sample
    (whose contact the pad approaches / leaves), -1 when the weight is 0."""
    n = t.size
    w = np.zeros(n)
    ref = np.full(n, -1, int)
    if not np.any(act):
        return w, ref
    e = np.diff(np.concatenate([[0], act.astype(int), [0]]))
    runs = list(zip(np.flatnonzero(e == 1), np.flatnonzero(e == -1) - 1))
    for a, b in runs:
        ta, tb = t[a], t[b]
        for k in range(n):
            if a <= k <= b:
                w[k], ref[k] = 1.0, k
                continue
            if ta - blend_s < t[k] < ta:
                x = float(smoothstep((t[k] - (ta - blend_s)) / blend_s))
                j = a
            elif tb < t[k] < tb + blend_s:
                x = float(smoothstep(1.0 - (t[k] - tb) / blend_s))
                j = b
            else:
                continue
            if x > w[k]:
                w[k], ref[k] = x, j
    return w, ref


def push_runs(act: np.ndarray) -> list[tuple[int, int]]:
    """(first, last) sample index of every run of True."""
    a = np.asarray(act, bool)
    if not np.any(a):
        return []
    e = np.diff(np.concatenate([[0], a.astype(int), [0]]))
    return list(zip(np.flatnonzero(e == 1), np.flatnonzero(e == -1) - 1))


def _yaw_to(src: np.ndarray, dst: np.ndarray) -> float:
    d = np.asarray(dst, float)[:2] - np.asarray(src, float)[:2]
    return math.atan2(float(d[1]), float(d[0]))


def person_track(t: np.ndarray, pos: np.ndarray, R: np.ndarray, floor_z: float, push_active: np.ndarray | None,
                 push_force: np.ndarray | None, push_point: np.ndarray | None,
                 idle_xy: Sequence[float] | None = None, walk_s: float = WALK_S) -> PersonTrack:
    """Pure.  The operator (feet on the floor at ``floor_z``) and the rigid 2.33 m pole.

    Push positions: for every logged push the feet are PLANTED at one anchor for the whole push window (its 0.5 s
    blend in, the push, the blend out): the place from which the nominal held end (``FIGURE_HANDS``, chest height,
    0.12 m right of the centre line) reaches the push's first contact with the pole on the line of the force and its
    pitch within +-25 deg: anchor = pad_xy - u (2.0 cos(pitch)) - u 0.44 - left (-0.12), u = horizontal force direction,
    pad = contact - u 45 mm; never closer than MIN_STANDOFF_M to the vehicle axis.  While the vehicle gives way the
    feet stay put and the arms follow the contact (round 1 slid the feet 0.13 m with the contact and mixed a walk cycle
    into the push: foot skating, tiptoe legs).  Between pushes the operator waits at the last anchor and walks to the
    next one in the ``walk_s`` before its blend starts (eased in / out, never overlapping a push window); before the
    first push it waits at the first anchor; no push at all: it stands at ``idle_xy`` facing the vehicle.

    Facing: the vehicle, blended to the force direction by the push weight.  Walking: weight from the feet's speed
    (0 below 0.15 m/s, 1 from 0.6 m/s, smoothed 0.3 s) and forced to 0 wherever the push weight exceeds 0.05; the gait
    phase advances with the distance walked (2 pi per 1.15 m), never with time.

    Pole: carried upright at the ready (CARRY_HOLD / CARRY_DIR) and blended over 0.5 s into the push geometry: pad =
    contact - u 45 mm (u = the force direction; the pad follows the contact while the push lasts), the pole from the
    nominal hands toward the pad with its pitch clamped to +-25 deg, held end = pad - dir 2.0 m; grips at POLE_GRIP_M
    along the pole from the held end."""
    n = t.size
    cg = pos + R[:, :, 2] * Z_CG_DEFAULT
    act = np.zeros(n, bool) if push_active is None else np.asarray(push_active, bool)
    has = bool(np.any(act)) and push_force is not None and push_point is not None
    if idle_xy is None:
        idle_xy = (float(pos[0, 0]) - 2.2, float(pos[0, 1]) + 1.0)
    mp = math.radians(POLE_MAX_PITCH_DEG)
    contact = np.zeros((n, 3))
    if has:
        for k in np.flatnonzero(act):
            contact[k] = visual_contact(np.asarray(push_point[k], float), np.asarray(push_force[k], float), pos[k], R[k])
    pw, ref = push_weight(t, act) if has else (np.zeros(n), np.full(n, -1, int))
    runs = push_runs(act) if has else []
    # ---- one planted anchor per push ----
    anchors, u_run = [], []
    for a, b in runs:
        fh = np.asarray(push_force[a, :2], float)
        if float(np.hypot(*fh)) < 1e-6:
            fh = contact[a, :2] - np.asarray(idle_xy, float)
        u = fh / max(float(np.hypot(*fh)), 1e-9)
        F = np.asarray(push_force[a], float)
        u3 = _unit(F) if float(np.linalg.norm(F)) > 1e-9 else np.array([u[0], u[1], 0.0])
        pad_a = contact[a] - u3 * POLE_PAD_R
        dz = float(pad_a[2]) - (floor_z + FIGURE_HANDS[2])
        pitch = math.asin(float(np.clip(dz / POLE_HOLD_TO_PAD, -math.sin(mp), math.sin(mp))))
        left = np.array([-u[1], u[0]])
        fxy = pad_a[:2] - u * (POLE_HOLD_TO_PAD * math.cos(pitch) + FIGURE_HANDS[0]) - left * FIGURE_HANDS[1]
        d = fxy - cg[a, :2]
        r = float(np.hypot(*d))
        if r < MIN_STANDOFF_M:
            fxy = cg[a, :2] + d / max(r, 1e-9) * MIN_STANDOFF_M
        anchors.append(fxy)
        u_run.append(u)
    feet = np.zeros((n, 3))
    feet[:, 2] = floor_z
    yaw = np.zeros(n)
    if not runs:
        feet[:, :2] = np.asarray(idle_xy, float)
        for k in range(n):
            yaw[k] = _yaw_to(feet[k], cg[k])
    else:
        # push windows (planted) in time
        win = [(float(t[a]) - PUSH_BLEND_S, float(t[b]) + PUSH_BLEND_S) for a, b in runs]
        for k in range(n):
            tk = float(t[k])
            j = int(np.searchsorted([w[0] for w in win], tk, side="right")) - 1   # last window started
            if j < 0:
                feet[k, :2] = anchors[0]
            elif tk <= win[j][1] or j == len(win) - 1:
                feet[k, :2] = anchors[j]
            else:
                # between window j and j + 1: wait, then walk in the walk_s before window j + 1 (eased)
                t1 = win[j + 1][0]
                t0 = max(win[j][1], t1 - walk_s)
                s = float(np.clip((tk - t0) / max(t1 - t0, 1e-9), 0.0, 1.0))
                s = s * s * (3.0 - 2.0 * s)
                feet[k, :2] = (1.0 - s) * anchors[j] + s * anchors[j + 1]
                # walking around the vehicle, never through its stand-off circle
                d = feet[k, :2] - cg[k, :2]
                r = float(np.hypot(*d))
                if r < MIN_STANDOFF_M:
                    feet[k, :2] = cg[k, :2] + d / max(r, 1e-9) * MIN_STANDOFF_M
        yv = np.unwrap(np.array([_yaw_to(feet[k], cg[k]) for k in range(n)]))
        for k in range(n):
            y = float(yv[k])
            if pw[k] > 0.0 and ref[k] >= 0:
                jr = int(np.searchsorted([a for a, _ in runs], int(ref[k]), side="right")) - 1
                yu = math.atan2(float(u_run[jr][1]), float(u_run[jr][0]))
                y = y + float(pw[k]) * ang_diff(yu, y)
            yaw[k] = y
        yaw = np.unwrap(yaw)
    # ---- walking (from the planted / walking feet track) ----
    xy = feet[:, :2]
    v = np.gradient(xy, t, axis=0) if n > 2 else np.zeros_like(xy)
    sp = np.hypot(v[:, 0], v[:, 1])
    raw = np.clip((sp - 0.15) / 0.45, 0.0, 1.0)
    raw = raw * raw * (3.0 - 2.0 * raw)
    dt = float(np.median(np.diff(t))) if n > 1 else 1.0
    half = max(1, int(round(0.5 * 0.3 / max(dt, 1e-6))))
    ker = np.ones(2 * half + 1) / (2 * half + 1)
    walk_w = np.convolve(np.pad(raw, half, mode="edge"), ker, mode="valid") if n > 2 else np.zeros(n)
    walk_w = np.where(pw > 0.05, 0.0, walk_w)
    dist = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]) if n > 1 else np.zeros(n)
    walk_phase = 2.0 * math.pi * dist / 1.15
    wdir = np.zeros((n, 2))
    last = np.array([1.0, 0.0])
    for k in range(n):
        if sp[k] > 0.05:
            last = v[k] / sp[k]
        c, s_ = math.cos(float(yaw[k])), math.sin(float(yaw[k]))
        wdir[k] = (c * last[0] + s_ * last[1], -s_ * last[0] + c * last[1])
    # ---- the pole ----
    hold_ = np.zeros((n, 3))
    pdir = np.zeros((n, 3))
    for k in range(n):
        Rz = _rot2(float(yaw[k]))
        hc = feet[k] + Rz @ CARRY_HOLD
        dc = Rz @ _unit(CARRY_DIR)
        if pw[k] > 0.0 and ref[k] >= 0:
            j = int(ref[k])
            F = np.asarray(push_force[j], float)
            hands = feet[k] + Rz @ FIGURE_HANDS
            u = _unit(F) if float(np.linalg.norm(F)) > 1e-9 else _unit(contact[j] - hands)
            pad = contact[j] - u * POLE_PAD_R
            dp = _unit(pad - hands)
            hzn = float(np.hypot(dp[0], dp[1]))
            pitch = float(np.clip(math.atan2(dp[2], max(hzn, 1e-9)), -mp, mp))
            dp = np.array([dp[0] / max(hzn, 1e-9) * math.cos(pitch), dp[1] / max(hzn, 1e-9) * math.cos(pitch), math.sin(pitch)])
            hp = pad - dp * POLE_HOLD_TO_PAD
            w = float(pw[k])
            hold_[k] = (1.0 - w) * hc + w * hp
            pdir[k] = _unit((1.0 - w) * dc + w * dp)
        else:
            hold_[k], pdir[k] = hc, dc
    pad = hold_ + pdir * POLE_HOLD_TO_PAD
    butt = hold_ - pdir * POLE_BUTT_M
    notes = []
    if has:
        d = np.hypot(*(feet[:, :2] - cg[:, :2]).T)
        notes.append(f"person: {len(runs)} pushes, feet planted per push, closest feet-vehicle horizontal distance "
                     f"{float(np.min(d)):.2f} m")
    return PersonTrack(feet, yaw, pw, walk_w, walk_phase, wdir, hold_, pdir, pad, butt, hold_ + pdir * POLE_GRIP_M[0],
                       hold_ + pdir * POLE_GRIP_M[1], cg, notes)


def snap_pad(ptrack: dict[str, np.ndarray], pa: np.ndarray, pf: np.ndarray, pp: np.ndarray, pos: np.ndarray,
             R: np.ndarray) -> float:
    """Frame-level contact: in every push-active frame move the whole pole (rigidly, by a few mm) so the pad sphere is
    tangent to the body at the logged contact of THAT frame (art review round 2: up to 9.8 mm clearance at the push
    ramps, where the 50 Hz pole track and the frame-rate vehicle pose disagree).  Returns the largest shift (m)."""
    worst = 0.0
    for k in np.flatnonzero(np.asarray(pa, bool)):
        F = np.asarray(pf[k], float)
        if float(np.linalg.norm(F)) < 1e-12:
            # (the frame straddles the push's last log sample: the interpolated force is ~0 - push along the pole)
            d0 = np.asarray(ptrack["pole_dir"][k], float)
            F = np.array([d0[0], d0[1], 0.0])
            if float(np.linalg.norm(F)) < 1e-9:
                continue
        c = visual_contact(np.asarray(pp[k], float), F, pos[k], R[k])
        target = c - _unit(F) * POLE_PAD_R
        d = target - ptrack["pad"][k]
        worst = max(worst, float(np.linalg.norm(d)))
        for key in ("hold", "pad", "butt", "grip_r", "grip_l"):
            ptrack[key][k] = ptrack[key][k] + d
    return worst


# --------------------------------------------------------------------------- #
# cameras
# --------------------------------------------------------------------------- #
@dataclass
class CamTrack:
    pos: np.ndarray        # (n, 3)
    tgt: np.ndarray        # (n, 3)
    roll: np.ndarray       # (n,) rad
    lens: np.ndarray       # (n,) mm
    focus: np.ndarray      # (n,) m
    fstop: np.ndarray      # (n,)
    fill: np.ndarray | None = None   # (n,) 0..1 camera-mounted fill light (vane close-ups)
    shift: np.ndarray | None = None  # (n, 2) Blender lens shift (x, y) in units of the sensor width (level cameras)


@dataclass
class SceneCfg:
    indoor: bool = False
    sun_az_deg: float = 135.0          # where the sun light comes FROM (azimuth from +x toward +y)
    windsock_xy: tuple[float, float] = (7.0, 5.0)
    windsock_auto: bool = True           # place it for the establishing shot (build_plan); False = windsock_xy
    room_half: tuple[float, float] = (11.0, 8.0)  # indoor: keep cameras inside the hall
    seed: int = 7
    width: int = 1920
    height: int = 1080
    assets_dir: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "polyhaven")


def look_quat(pos: np.ndarray, tgt: np.ndarray, roll: float = 0.0) -> np.ndarray:
    """Blender camera quaternion (w,x,y,z): local -Z toward tgt, +Y up (world z), rolled about the view axis."""
    f = np.asarray(tgt, float) - np.asarray(pos, float)
    f /= max(float(np.linalg.norm(f)), 1e-12)
    up = np.array([0.0, 0.0, 1.0]) if abs(f[2]) < 0.999 else np.array([0.0, 1.0, 0.0])
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    if roll:
        c, s = math.cos(roll), math.sin(roll)
        r, u = c * r + s * u, -s * r + c * u
    return mat_to_quat(np.column_stack([r, u, -f]))


def project(P: np.ndarray, pos: np.ndarray, tgt: np.ndarray, roll: float, lens_mm: float, W: int, H: int,
            shift: Sequence[float] = (0.0, 0.0)) -> tuple[float, float, float]:
    """Pixel (x right, y down) and depth of world point P for the camera (Blender sensor fit: 36 mm horizontal; lens
    shift in units of the sensor width moves the image by shift x W pixels)."""
    q = look_quat(pos, tgt, roll)
    w, x, y, z = q
    Rm = quat_to_matrix(np.array([w, x, y, z]))
    d = np.asarray(P, float) - np.asarray(pos, float)
    cx, cy, cz = Rm.T @ d                          # camera coords (looking along -z)
    depth = -cz
    if depth <= 1e-6:
        return float("nan"), float("nan"), depth
    k = lens_mm / SENSOR_W_MM * W
    return W / 2 + k * cx / depth - float(shift[0]) * W, H / 2 - k * cy / depth + float(shift[1]) * W, depth


# level two-shot framing (art review round 2): headroom above the top of the subjects, the band kept clear for the
# lower HUD row, the widest / longest lens allowed
FRAME_TOP = 0.12
FRAME_BOTTOM = 0.85
FRAME_SIDE = 0.44


def level_frame(cam: np.ndarray, fwd_h: np.ndarray, pts: np.ndarray, W: int, H: int, lens_lo: float = 18.0,
                lens_hi: float = 50.0, side_margin_m: float = 0.30, lens: float | None = None) -> tuple[float, float]:
    """(lens mm, shift_y) of a LEVEL camera at ``cam`` looking along the horizontal unit ``fwd_h`` so that every point
    of ``pts`` lies between FRAME_TOP and FRAME_BOTTOM of the frame height and within FRAME_SIDE of the half width
    (verticals stay vertical: a tilted 23 mm lens made a 5.3 deg lean read as ~12 deg)."""
    f = np.array([fwd_h[0], fwd_h[1], 0.0])
    f /= max(float(np.linalg.norm(f)), 1e-9)
    r = np.cross(f, [0.0, 0.0, 1.0])
    xs, ys = [], []
    for p in np.asarray(pts, float).reshape(-1, 3):
        v = p - cam
        z = max(float(v @ f), 0.3)
        xs.append((abs(float(v @ r)) + side_margin_m) / z)
        ys.append(float(v[2]) / z)
    top, bot = max(ys), min(ys)
    k_v = (FRAME_BOTTOM - FRAME_TOP) * H / max(top - bot, 1e-6)          # px per tan unit
    k_h = FRAME_SIDE * W / max(max(xs), 1e-6)
    k = min(k_v, k_h)
    if lens is None:
        lens = float(np.clip(k * SENSOR_W_MM / W, lens_lo, lens_hi))
    k = lens / SENSOR_W_MM * W
    # centre the content between the top / bottom limits
    y_mid = 0.5 * (FRAME_TOP + FRAME_BOTTOM) * H
    t_mid = 0.5 * (top + bot)
    shift_y = (y_mid - H / 2 + k * t_mid) / W
    return lens, float(shift_y)


def lens_for(height_m: float, dist_m: float, frac: float, H: int = 1080, W: int = 1920, lo: float = 18.0, hi: float = 200.0) -> float:
    """Focal length (mm) at which an object ``height_m`` tall at ``dist_m`` fills ``frac`` of the frame height."""
    sensor_h = SENSOR_W_MM * H / W
    ang = 2 * math.atan2(0.5 * height_m, max(dist_m, 1e-3))
    fov = ang / max(frac, 1e-3)
    return float(np.clip(0.5 * sensor_h / math.tan(0.5 * min(fov, math.radians(150))), lo, hi))


def _shake(ts: np.ndarray, seed: int, amp_deg: float, amp_m: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Deterministic handheld shake: sums of sines (0.3 - 3 Hz) with seeded phases; (yaw, pitch, roll) rad + pos (n,3)."""
    rng = np.random.default_rng(seed)
    freqs = np.array([0.31, 0.73, 1.37, 2.9])
    wts = np.array([1.0, 0.6, 0.35, 0.15])
    out = []
    for _ in range(6):
        ph = rng.uniform(0, 2 * math.pi, size=freqs.size)
        out.append(np.sum(wts[None, :] * np.sin(2 * math.pi * freqs[None, :] * ts[:, None] + ph[None, :]), axis=1) / wts.sum())
    rot = np.radians(amp_deg) * np.stack(out[:3], axis=1)
    rot[:, 2] *= 0.5
    trans = amp_m * np.stack(out[3:], axis=1)
    return rot[:, 0], rot[:, 1], rot[:, 2], trans


def _azimuth_for(cfg: SceneCfg, shot_i: int, kind: str) -> float:
    """Camera azimuth (from the subject toward the camera, deg): the sun 35-80 deg to the side of the camera's back
    (modelling light on the vehicle, no backlit haze), varied per shot."""
    base = cfg.sun_az_deg if not cfg.indoor else 200.0          # camera on the sun's side: front / side light
    # the sun 60-90 deg to the side of the lens: a matte black body needs a lit side and a shadow side to read as a
    # shape (front light flattens it into mid grey: round 1 measured 77 sRGB on the pad shot)
    offs = {"spectator": (-70.0, 65.0, -80.0), "closeup": (-80.0, 70.0, -20.0), "wide": (-60.0, 55.0),
            "orbit": (70.0, 160.0, -90.0), "crane": (-70.0, 60.0)}.get(kind, (0.0,))
    return base + offs[shot_i % len(offs)]


def ang_diff(a: float, b: float) -> float:
    """Signed smallest difference a - b (rad), in (-pi, pi]."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


# Camera azimuth relative to the operator (deg from the vehicle -> person bearing), one side of the action line for
# the whole edit (the 180-degree rule).  |rel| >= 90 always: the person faces the vehicle, so a camera there sees him
# in profile or front three-quarter, never from behind.
REL_AZ = {"push": 95.0, "spectator": 100.0, "closeup": 160.0, "orbit": 110.0, "crane": 115.0, "wide": 130.0}
# (close-ups from below sit nearly opposite the operator: from the person's side the pole runs between the lens and
#  the duct exit and reads as a rod holding the vehicle)
PERSON_MAX_FRAC = 0.5          # the person never takes more than half the frame height (spectator / wide)


def person_bearing(feet: np.ndarray, veh: np.ndarray) -> float:
    """Mean bearing (rad) from the vehicle to the person over a shot."""
    d = np.mean(np.asarray(feet, float)[:, :2] - np.asarray(veh, float)[:, :2], axis=0)
    return math.atan2(float(d[1]), float(d[0]))


def side_azimuth(kind: str, push: bool, bearing: float, side: int) -> float:
    """Camera azimuth (deg) on the chosen side (+1 / -1) of the vehicle-person line (``REL_AZ``)."""
    rel = REL_AZ["push"] if push else REL_AZ.get(kind, 110.0)
    return math.degrees(bearing) + side * rel


def person_safe_azimuth(default_deg: float, feet: np.ndarray, veh: np.ndarray, push: bool,
                        min_sep_deg: float = 55.0) -> float:
    """Camera azimuth (deg, from the vehicle toward the camera) that never puts the person between the camera and the
    vehicle: for a push shot the profile view (person bearing +-90 deg, the side nearer the default); otherwise the
    default unless it is within ``min_sep_deg`` of the person's bearing, then the nearest bearing +-min_sep_deg."""
    d = np.mean(np.asarray(feet, float)[:, :2] - np.asarray(veh, float)[:, :2], axis=0)
    bp = math.atan2(float(d[1]), float(d[0]))
    a0 = math.radians(default_deg)
    if push:
        c1, c2 = bp + math.pi / 2, bp - math.pi / 2
        return math.degrees(c1 if abs(ang_diff(c1, a0)) <= abs(ang_diff(c2, a0)) else c2)
    dd = ang_diff(a0, bp)
    sep = math.radians(min_sep_deg)
    if abs(dd) >= sep:
        return default_deg
    return math.degrees(bp + math.copysign(sep, dd if dd != 0 else 1.0))


def _clamp_track(hall: Any, subject: np.ndarray, cpos: np.ndarray) -> np.ndarray:
    """Indoor: keep every camera position on the stage of the hall (render.hall.clamp_xy, pulled toward the subject)."""
    if hall is None:
        return cpos
    from . import hall as HL
    out = cpos.copy()
    for k in range(cpos.shape[0]):
        out[k, :2] = HL.clamp_xy(hall, subject[k], cpos[k])
    return out


def cfg_fps(ts: np.ndarray) -> float:
    """Samples per second of a frame-time array (slow motion included)."""
    d = np.diff(np.asarray(ts, float))
    return 1.0 / max(float(np.median(d)), 1e-6) if d.size else 30.0


def _push_two_shot(ts: np.ndarray, c: np.ndarray, cg: np.ndarray, feet: np.ndarray, pole_mid: np.ndarray,
                   ground_z: float, cfg: SceneCfg, az0: float, w: float, params: dict[str, Any], hall: Any) -> CamTrack:
    """Push two-shot (art review round 2: the round-1 orbit jammed the contact into the top-left corner with a tilted
    23 mm lens).  A LEVEL camera at chest height (``cam_h``, 1.30 m) on the chosen side, orbiting slowly (``w``)
    around the middle of the pole (between the hands and the pad) and looking at it horizontally; one lens for the shot,
    ~32 mm (``lens``) by choosing the orbit radius (indoors the stage may pull the camera in: then wider), and a vertical
    lens shift that puts the vehicle, the pole and the operator between 12 % headroom and the band of the lower HUD row
    (``level_frame``)."""
    n = ts.size
    W, H = cfg.width, cfg.height
    # the middle of the action: halfway between the operator's hands (chest height) and the vehicle - the pole's middle
    # while pushing, and steady while the pole is carried upright (a causal smoothing of the pole's own middle lagged
    # toward the carried pole: round 2 probe framed the operator, not the action).  Zero-phase smoothing.
    sig = max(0.4 * cfg_fps(ts), 1.0)
    feet_s = gauss_smooth(feet, sig)
    ctr = gauss_smooth(0.5 * (feet + np.array([0.0, 0.0, 1.25]) + cg), sig)
    cam_z = ground_z + float(params.get("cam_h", 1.30))
    a = az0 + w * (ts - ts[0])
    want = float(params.get("lens", 32.0))
    Rr = float(params.get("radius", 4.6))
    up = np.array([0.0, 0.0, 1.0])

    def content(k: int) -> np.ndarray:
        # the vehicle's nose / exit, the operator's head and both feet (a staggered stance reaches ~0.3 m either side)
        return np.array([c[k] + 0.50 * up, c[k] - 0.42 * up, feet_s[k] + 1.80 * up, feet_s[k],
                         np.array([0.0, 0.0, ground_z - 0.02]),          # the pad stand under the hover (height cue)
                         feet_s[k] + np.array([0.3, 0.0, 0.0]), feet_s[k] - np.array([0.3, 0.0, 0.0]),
                         feet_s[k] + np.array([0.0, 0.3, 0.0]), feet_s[k] - np.array([0.0, 0.3, 0.0])])
    cpos = None
    lens_fit = want
    for _ in range(4):
        cpos = np.stack([ctr[:, 0] + Rr * np.cos(a), ctr[:, 1] + Rr * np.sin(a), np.full(n, cam_z)], axis=1)
        cpos = _clamp_track(hall, ctr, cpos)
        fits = []
        for k in range(n):
            fh = ctr[k] - cpos[k]
            fits.append(level_frame(cpos[k], fh, content(k), W, H, 14.0, 80.0)[0])
        lens_fit = float(min(fits))
        if "radius" in params or abs(lens_fit - want) < 1.5:
            break
        Rr = float(np.clip(Rr * want / lens_fit, 2.5, 9.0))
    lens_c = float(np.clip(lens_fit, 18.0, 50.0))
    shift = np.zeros((n, 2))
    for k in range(n):
        shift[k, 1] = level_frame(cpos[k], ctr[k] - cpos[k], content(k), W, H, lens=lens_c)[1]
    shift[:, 1] = gauss_smooth(shift[:, 1], sig)
    fh = ctr - cpos
    fh[:, 2] = 0.0
    fh /= np.maximum(np.linalg.norm(fh, axis=1, keepdims=True), 1e-9)
    tgt = cpos + fh * 5.0
    focus = np.linalg.norm(ctr - cpos, axis=1)
    return CamTrack(cpos, tgt, np.zeros(n), np.full(n, lens_c), focus, np.full(n, float(params.get("fstop", 4.0))),
                    None, shift)


def camera_for_shot(kind: str, ts: np.ndarray, pos: np.ndarray, R: np.ndarray, ground_z: float, cfg: SceneCfg,
                    shot_i: int, params: dict[str, Any] | None = None, extra_pts: np.ndarray | None = None,
                    hall: Any = None, person_feet: np.ndarray | None = None,
                    pole_mid: np.ndarray | None = None) -> CamTrack:
    """One rig over the frames of a shot.  ``pos``/``R`` are the vehicle samples at the frame times ``ts``; ``hall``
    (indoor) keeps the camera on the stage; ``person_feet`` limits the person's size in the frame (spectator);
    ``pole_mid`` (the pole's middle between the hands and the pad) centres a push two-shot."""
    params = params or {}
    n = ts.size
    cg = pos + R[:, :, 2] * Z_CG_DEFAULT
    exitp = pos.copy()
    W, H = cfg.width, cfg.height
    az = math.radians(float(params.get("azimuth_deg", _azimuth_for(cfg, shot_i, kind))))
    dirv = np.array([math.cos(az), math.sin(az), 0.0])
    roll = np.zeros(n)
    fstop = np.full(n, 4.0)
    if kind == "spectator" and bool(params.get("locked", False)) and "compose_t" in params:
        # locked-off telephoto composed for one instant (a still: art review round 2 - the landing vehicle small and
        # centred over 45 % empty grass): level tripod ``dist`` m away, ``lens`` (75 mm), the vehicle's CG at that
        # instant on a lower-third intersection with lead room toward its screen motion (the drift)
        D = float(params.get("dist", 18.0))
        lens_c = float(params.get("lens", 75.0))
        k_s = int(np.argmin(np.abs(ts - float(params["compose_t"]))))
        P0 = cg[k_s]
        cam = np.array([P0[0] + D * dirv[0], P0[1] + D * dirv[1], ground_z + float(params.get("cam_h", 1.5))])
        kpx = lens_c / SENSOR_W_MM * W
        # screen motion: the CG's horizontal velocity across the view
        k0, k1 = max(k_s - 6, 0), min(k_s + 6, n - 1)
        vel = cg[k1] - cg[k0]
        bearing = math.atan2(float(P0[1] - cam[1]), float(P0[0] - cam[0]))
        rgt = np.array([math.sin(bearing), -math.cos(bearing), 0.0])
        x_frac = 1.0 / 3.0 if float(vel @ rgt) >= 0.0 else 2.0 / 3.0
        y_frac = float(params.get("y_frac", 2.0 / 3.0))
        phi = math.atan((x_frac - 0.5) * W / kpx)
        yaw_v = bearing + phi
        fh = np.array([math.cos(yaw_v), math.sin(yaw_v), 0.0])
        zd = float((P0 - cam) @ fh)
        t_v = float(P0[2] - cam[2]) / max(zd, 1e-6)
        sy = (y_frac * H - H / 2 + kpx * t_v) / W
        cpos = np.repeat(cam[None, :], n, axis=0)
        tgt = cpos + fh[None, :] * 10.0
        shift = np.zeros((n, 2))
        shift[:, 1] = sy
        focus = np.linalg.norm(cg - cpos, axis=1)
        fstop[:] = float(params.get("fstop", 5.6))
        return CamTrack(cpos, tgt, roll, np.full(n, lens_c), focus, fstop, None, shift)
    if kind == "spectator":
        hmax = float(np.max(cg[:, 2] - ground_z))
        D = float(params.get("dist", (3.6 if cfg.indoor else 5.5) + 0.6 * max(0.0, hmax - 2.0)))
        locked = bool(params.get("locked", False))
        anchor = cg[0, :2] if not locked else cg[-1, :2]
        cam0 = np.array([anchor[0] + D * dirv[0], anchor[1] + D * dirv[1], ground_z + float(params.get("cam_h", 1.62))])
        if hall is not None:
            cam0[:2] = _clamp_track(hall, cg[:1], cam0[None, :])[0, :2]
        # locked-off (landing): a tripod with a slow pan, no handheld shake / roll, 180-degree shutter
        if locked:
            # tripod, no pan at all (a pan over the drifting touchdown smeared the turf in round 1): aim at where
            # the vehicle is in the second half of the shot; the lens below keeps that whole stretch in frame
            tgt = np.repeat(np.mean(cg[n // 2:], axis=0)[None, :], n, axis=0)
        else:
            tgt = ema(cg, ts, 0.30)
        if person_feet is not None:
            # rule of thirds with the operator: aim 30 % of the way toward his chest, he stays in the frame
            chest = ema(np.asarray(person_feet, float) + np.array([0.0, 0.0, 1.25]), ts, 0.6)
            tgt = tgt + 0.30 * (chest - tgt) * np.array([1.0, 1.0, 0.0])
        amp = (0.0, 0.0) if locked else (0.22, 0.004)
        yaw_s, pitch_s, roll_s, tr = _shake(ts, cfg.seed * 101 + shot_i, amp[0], amp[1])
        cpos = cam0[None, :] + tr
        # apply small angular shake by moving the target in the image plane
        dist = np.linalg.norm(tgt - cpos, axis=1)
        right = np.cross(tgt - cpos, [0, 0, 1])
        right /= np.maximum(np.linalg.norm(right, axis=1, keepdims=True), 1e-9)
        tgt = tgt + right * (yaw_s * dist)[:, None] + np.array([0, 0, 1.0]) * (pitch_s * dist)[:, None]
        roll = roll_s
        lens = np.array([lens_for(VEH_LEN, float(d), 0.30, H, W, 24, 160) for d in dist])
        if locked:
            f0 = tgt[0] - cpos[0]
            f0 /= np.linalg.norm(f0)
            dev = [math.acos(float(np.clip((v / np.linalg.norm(v)) @ f0, -1, 1))) for v in (cg[n // 2:] - cpos[0])]
            l_fit = 0.5 * SENSOR_W_MM / math.tan(max(max(dev) / 0.75, math.radians(4.0)))
            lens = np.full(n, float(np.clip(min(float(np.median(lens)), l_fit), 24.0, 160.0)))
        if person_feet is not None:
            # the person at most PERSON_MAX_FRAC of the frame height (mid-ground, never a dominating figure)
            dp = np.linalg.norm(np.asarray(person_feet, float) + np.array([0.0, 0.0, 0.9]) - cpos, axis=1)
            lens = np.minimum(lens, [lens_for(1.80, float(d), PERSON_MAX_FRAC, H, W, 18, 160) for d in dp])
        lens = ema(lens, ts, 0.8)
        focus = np.linalg.norm(cg - cpos, axis=1)
        fstop[:] = float(params.get("fstop", 4.0))
        return CamTrack(cpos, tgt, roll, lens, focus, fstop)
    if kind == "orbit":
        c = ema(cg, ts, float(params.get("follow_s", 0.25)))
        Rr = float(params.get("radius", 2.6 if cfg.indoor else 3.4))
        w = math.radians(float(params.get("deg_per_s", 11.0)))
        a = az + w * (ts - ts[0])
        h = float(params.get("height", 0.35))
        two_shot = person_feet is not None and str(params.get("kind", "")) == "push"
        if two_shot and pole_mid is not None:
            return _push_two_shot(ts, c, cg, np.asarray(person_feet, float), np.asarray(pole_mid, float), ground_z, cfg,
                                  az, w, params, hall)
        if two_shot and "radius" not in params:
            Rr = 4.2                                  # a two-shot needs room: operator + 2.4 m pole + vehicle
        if two_shot:
            # push: a two-shot of the operator and the vehicle - orbit around their midpoint, aim between them
            chest = ema(np.asarray(person_feet, float) + np.array([0.0, 0.0, 1.25]), ts, 0.6)
            ctr = c.copy()
            ctr[:, :2] = 0.5 * (c[:, :2] + chest[:, :2])
        else:
            ctr = c
        cpos = np.stack([ctr[:, 0] + Rr * np.cos(a), ctr[:, 1] + Rr * np.sin(a), np.maximum(c[:, 2] + h, ground_z + 0.6)], axis=1)
        cpos = _clamp_track(hall, ctr, cpos)
        lens = np.full(n, float(params.get("lens", 40.0)))
        if hall is not None and not two_shot:
            # a camera pulled in by the walls keeps the framing with a wider lens
            dd = np.linalg.norm(cpos[:, :2] - c[:, :2], axis=1)
            lens = np.maximum(lens * dd / max(Rr, 1e-6), 18.0)
            lens = ema(lens, ts, 0.5)
        tgt = c + np.array([0.0, 0.0, float(params.get("tgt_dz", 0.0))])
        if two_shot:
            # horizontally between the two; vertically from below the feet (room for the lower HUD row) to above
            # the vehicle's nose
            tgt = 0.5 * (c + chest)
            tgt[:, 2] = 0.5 * ((ground_z - 0.30) + (c[:, 2] + 0.60))
            fit = []
            hs_v = 0.5 * SENSOR_W_MM * H / W
            for k in range(n):
                f_ = tgt[k] - cpos[k]
                f_ /= np.linalg.norm(f_)
                r_ = np.cross(f_, [0.0, 0.0, 1.0])
                r_ /= max(np.linalg.norm(r_), 1e-9)
                u_ = np.cross(r_, f_)
                feet_k = chest[k] - np.array([0.0, 0.0, 1.25])
                need_x, need_top, need_bot = 0.0, 0.0, 0.0
                for pt in (c[k] + np.array([0.0, 0.0, 0.60]), c[k] - np.array([0.0, 0.0, 0.42]), chest[k] + np.array([0.0, 0.0, 0.55]), feet_k):
                    v = pt - cpos[k]
                    z = float(v @ f_)
                    need_x = max(need_x, (abs(float(v @ r_)) + 0.35) / max(z, 0.3))
                    yy = float(v @ u_) / max(z, 0.3)
                    need_top, need_bot = max(need_top, yy), max(need_bot, -yy)
                # vertical: the top within 92 % of the half-height, the feet above the lower HUD row (75 %)
                tan_v = max(need_top / 0.92, need_bot / 0.75)
                l_h = 0.5 * SENSOR_W_MM / max(need_x / 0.92, 1e-6)
                l_v = hs_v / max(tan_v, 1e-6)
                fit.append(max(min(l_h, l_v), 18.0))     # never wider than 18 mm (distortion)
            lens = np.minimum(lens, ema(np.array(fit), ts, 0.5))
        focus = np.linalg.norm(cg - cpos, axis=1)
        fstop[:] = float(params.get("fstop", 2.8))
        return CamTrack(cpos, tgt, roll, lens, focus, fstop)
    if kind == "crane":
        c = ema(cg, ts, 0.5)
        s = smoothstep((ts - ts[0]) / max(ts[-1] - ts[0], 1e-6))
        D = (1 - s) * 5.2 + s * 3.8
        z = (1 - s) * (ground_z + 0.45) + s * (c[:, 2] + 1.3)
        a = az - math.radians(25.0) * s
        cpos = np.stack([c[:, 0] + D * np.cos(a), c[:, 1] + D * np.sin(a), z], axis=1)
        lens = np.full(n, 35.0)
        focus = np.linalg.norm(cg - cpos, axis=1)
        fstop[:] = 3.5
        return CamTrack(cpos, c, roll, lens, focus, fstop)
    if kind == "closeup":
        # the camera rides with the vehicle (light 0.05 s smoothing: a drifting vehicle stays in the frame)
        e = ema(exitp, ts, 0.05)
        hgt = e[:, 2] - ground_z
        mode = str(params.get("mode", "auto"))
        if mode == "auto":
            mode = "pad" if float(np.max(hgt)) < 0.4 else "vanes"
        yaw_s, pitch_s, roll_s, tr = _shake(ts, cfg.seed * 211 + shot_i, 0.10, 0.0012)
        if mode == "pad":
            # high three-quarter view into the intake windows: the EDF lip, spinner and rotor (blades -> blur disc)
            d_h, z_up = float(params.get("dist", 0.46)), float(params.get("height", 0.66))
            cpos = np.stack([e[:, 0] + d_h * dirv[0], e[:, 1] + d_h * dirv[1], e[:, 2] + z_up], axis=1) + tr
            tgt = exitp + np.array([0.0, 0.0, 0.22])
            lens = np.full(n, float(params.get("lens", 42.0)))
            fstop[:] = float(params.get("fstop", 5.6))
        else:
            # low close-up from below-side: the duct exit with the four vanes deflecting.  Snap the azimuth to 20 deg
            # off a vane plane (= between two fins, fins at 45 + k 90 deg body): a fin pointing at the lens is seen
            # edge-on and reads as a spike / rod hanging under the duct
            if bool(params.get("fin_avoid", True)):
                yaw = math.atan2(float(np.mean(R[:, 1, 0])), float(np.mean(R[:, 0, 0])))
                cands = [yaw + math.radians(90.0 * kk + sg * 20.0) for kk in range(4) for sg in (-1.0, 1.0)]
                az = min(cands, key=lambda c_: abs(ang_diff(c_, az)))
                dirv = np.array([math.cos(az), math.sin(az), 0.0])
            # ~13 deg off the body axis, 0.45 m from the exit (art review round 2: from 31-40 deg the vanes hid in
            # the black bore; near the axis the lens looks straight into the exit and the four vanes read as a
            # cross silhouetted against the softly lit tail cone, the floor / sky bounce on their edges)
            d_h, below = float(params.get("dist", 0.10)), float(params.get("below", 0.44))
            cz = np.maximum(e[:, 2] - below, ground_z + 0.25)
            cpos = np.stack([e[:, 0] + d_h * dirv[0], e[:, 1] + d_h * dirv[1], cz], axis=1) + tr
            tgt = exitp + R[:, :, 2] * 0.03
            lens = np.full(n, float(params.get("lens", 40.0)))
            fstop[:] = float(params.get("fstop", 8.0))
        cpos = _clamp_track(hall, e, cpos)
        roll = roll_s
        focus = np.linalg.norm(tgt - cpos, axis=1)
        fill = np.full(n, 1.0 if mode == "vanes" else 0.0)
        shift = np.zeros((n, 2))
        if mode == "vanes":
            shift[:, 1] = float(params.get("shift_y", -0.035))     # the duct above the lower HUD row
        return CamTrack(cpos, tgt, roll, lens, focus, fstop, fill, shift)
    if kind == "wide" and not cfg.indoor and str(params.get("mode", "tele")) == "tele":
        # Telephoto ESTABLISHING shot (art review round 2: the round-1 still framed a soft silhouette against empty
        # sky).  From ``dist`` (26 m) at eye height on the sun's side (the sun 25 deg off the lens' back axis: the
        # vehicle shows a lit side), level camera, one lens that keeps the pad / field (``with_pad``) AND the vehicle
        # between 12 % headroom and the lower HUD band (level_frame with lens shift: the horizon and the mountains sit
        # in the upper part, verticals vertical), 85-135 mm when the geometry allows it (a climbing vehicle: an 8 m
        # hover above the pad does not fit a telephoto frame with the pad - then the lens widens down to 50 mm).
        # Focus on the vehicle.  The windsock is placed in the background of this shot (build_plan).
        D = float(params.get("dist", 26.0))
        az_w = math.radians(float(params.get("azimuth_deg", cfg.sun_az_deg + 25.0)))
        V = np.mean(cg, axis=0)
        cam = np.array([V[0] + D * math.cos(az_w), V[1] + D * math.sin(az_w), ground_z + float(params.get("cam_h", 1.6))])
        fh = V - cam
        fh[2] = 0.0
        up = np.array([0.0, 0.0, 1.0])
        pts = [cg + R[:, :, 2] * 0.50, pos - up * 0.02]
        if bool(params.get("with_pad", True)):
            pts.append(np.array([[0.0, 0.0, ground_z - 0.05], [0.0, 0.0, ground_z + 0.10]]))
        pts = np.concatenate(pts, axis=0)
        lens_c, sy = level_frame(cam, fh, pts, W, H, 50.0, 135.0, side_margin_m=1.0)
        if "lens" in params:
            lens_c = float(params["lens"])
            sy = level_frame(cam, fh, pts, W, H, lens=lens_c, side_margin_m=1.0)[1]
        cpos = np.repeat(cam[None, :], n, axis=0)
        tgt = cpos + (fh / max(float(np.linalg.norm(fh)), 1e-9))[None, :] * 10.0
        shift = np.zeros((n, 2))
        shift[:, 1] = sy
        focus = np.linalg.norm(cg - cpos, axis=1)
        fstop[:] = float(params.get("fstop", 5.6))
        return CamTrack(cpos, tgt, roll, np.full(n, lens_c), focus, fstop, None, shift)
    if kind == "wide":
        D = float(params.get("dist", 9.5 if cfg.indoor else 24.0))
        pts = [cg, pos, np.array([[0.0, 0.0, 0.0]])]
        if extra_pts is not None:
            pts.append(np.asarray(extra_pts, float).reshape(-1, 3))
        allp = np.concatenate(pts, axis=0)
        mid = 0.5 * (allp.min(axis=0) + allp.max(axis=0))
        anchor = np.array([mid[0] + D * dirv[0], mid[1] + D * dirv[1], ground_z + 1.5])
        if hall is not None:
            anchor[:2] = _clamp_track(hall, mid[None, :], anchor[None, :])[0, :2]
        elif cfg.indoor:
            anchor[0] = np.clip(anchor[0], -cfg.room_half[0] + 0.8, cfg.room_half[0] - 0.8)
            anchor[1] = np.clip(anchor[1], -cfg.room_half[1] + 0.8, cfg.room_half[1] - 0.8)
        cpos = np.repeat(anchor[None, :], n, axis=0)
        # pan to keep the vehicle and the pad in frame (smooth)
        tgt_raw = 0.5 * (cg + np.array([0.0, 0.0, ground_z + 0.6]))
        if extra_pts is not None:
            # aim between the vehicle / pad and the extra points (the windsock) so all of them stay in frame
            tgt_raw[:, :2] = 0.5 * (tgt_raw[:, :2] + mid[None, :2])
        tgt = ema(tgt_raw, ts, 1.2)
        span = np.max(np.linalg.norm(allp[:, None, :2] - allp[None, :, :2], axis=2)) if allp.shape[0] < 400 else 6.0
        dist = np.linalg.norm(tgt - cpos, axis=1)
        need_h = np.maximum(np.max(cg[:, 2]) - ground_z + 1.0, 2.5)
        lens_h = np.array([lens_for(need_h, float(d), 0.75, H, W, 18, 120) for d in dist])
        lens_w = np.array([lens_for(max(span, 3.0) * H / W, float(d), 0.75, H, W, 18, 120) for d in dist])
        lens = ema(np.minimum(lens_h, lens_w), ts, 1.5)
        focus = np.linalg.norm(cg - cpos, axis=1)
        fstop[:] = 8.0
        return CamTrack(cpos, tgt, roll, lens, focus, fstop)
    if kind == "turntable":
        n_ = ts.size
        s = (ts - ts[0]) / max(ts[-1] - ts[0], 1e-6)
        a = az + math.radians(25.0) * s
        D = 2.75 - 0.40 * smoothstep(s)
        z = ground_z + 0.45 + 0.30 * smoothstep(s)
        c = np.array([0.0, 0.0, 0.40])
        cpos = np.stack([c[0] + D * np.cos(a), c[1] + D * np.sin(a), z], axis=1)
        tgt = np.repeat(c[None, :], n_, axis=0)
        lens = np.full(n_, 50.0)
        focus = np.linalg.norm(tgt - cpos, axis=1)
        fstop[:] = 5.6
        return CamTrack(cpos, tgt, roll, lens, focus, fstop)
    raise ValueError(f"unknown camera kind {kind!r}")


# Per-shot exposure offset (EV, added to the scene exposure).  The matte black body must read black in every shot
# (USER DECISION 1); seen from below in the bright hall its 4 % albedo + 3.4 % specular still mirror the sunlit parquet
# and the stage lights: measured sRGB ~(100, 80, 65) on the fins in the round-1b probe - beige.  A close-up from below
# is exposed for the dark body against the ceiling instead (-0.8 EV: ~(70, 55, 45)), as a camera operator would.
# Outdoors the sunlit close-up from below read mid grey (body 78, a fin 111 sRGB in round 1): -0.7 EV there too.
SHOT_EV = {("closeup", True): -0.8, ("closeup", False): -0.7}


# indoor stage key spot (render/blender/env.stage_key_spot) and the per-scene white balance (Blender view transform
# white balance, K of the light to neutralise: the school-hall photo is lit by warm tungsten + sun through amber
# blinds; round 1 graded the stage floor R/B 3.6 and the black PLA brown)
KEY_SPOT = {"height_m": 4.2, "off_vertical_deg": 8.0, "az_from_cam_deg": 50.0, "power_w": 3000.0, "radius_m": 0.06,
            "spot_deg": 60.0, "blend": 0.45}
WHITE_BALANCE_K = {"indoor": 5200.0, "outdoor": None}   # probe r2: 4300 K -> body |R-B| 16 (blue), 5000-5600 K -> 1-6


def shot_ev(kind: str, indoor: bool, params: dict[str, Any] | None = None) -> float:
    """Exposure offset (EV) of a shot: params['ev'] when given, else SHOT_EV[(kind, indoor)], else 0."""
    if params and "ev" in params:
        return float(params["ev"])
    return float(SHOT_EV.get((kind, bool(indoor)), 0.0))


def _refine_backplate(s: Shot, params: dict[str, Any], ts: np.ndarray, pos: np.ndarray, R: np.ndarray, gz: float,
                      cfg: SceneCfg, i: int, hall: Any, emap: Any, feet: np.ndarray, side: int = 1,
                      lmap: Any = None, pole_mid: np.ndarray | None = None) -> float:
    """Azimuth (deg) near params['azimuth_deg'] for a push shot whose backplate has no bright vertical structure
    behind / below the vehicle at the shot's key frame (``event_t``): score = min(pillar, 5) (render.hall.
    backplate_pillar: luminance profile across the vehicle) + 0.5 x vertical-edge energy (backplate_edges) + 40 x the
    blown-out fraction of a 25 x 13 ray grid (backplate_glare: round 2, the sunlit doorway behind the operator) +
    0.003 / deg of change.  Searched up to 51 deg toward the operator's front (``side`` x da >= 0: the person stays in
    profile / front three-quarter, never seen from behind) and 8 deg the other way.  (Round 1b: the edge map alone within
    +-12 deg kept a white pillar running down from the vehicle - it read as a support rod.)"""
    from . import hall as HL
    a0 = float(params["azimuth_deg"])
    best = (float("inf"), 0.0, a0)
    ev = float(params.get("event_t", 0.5 * (ts[0] + ts[-1])))
    k = int(np.argmin(np.abs(ts - ev)))
    for da in [0.0, -4.0 * side, -8.0 * side] + [side * float(x) for x in range(3, 52, 3)]:
        p = dict(params, azimuth_deg=a0 + da)
        ct = camera_for_shot(s.cam, ts, pos, R, gz, cfg, i, p, hall=hall, person_feet=feet, pole_mid=pole_mid)
        e = HL.backplate_edges(hall, emap, ct.pos[k], pos[k], float(ct.lens[k]))
        pil = HL.backplate_pillar(hall, lmap, ct.pos[k], ct.tgt[k], pos[k] + np.array([0.0, 0.0, 0.2]), float(ct.lens[k]))
        gl = HL.backplate_glare(hall, lmap, ct.pos[k], ct.tgt[k], float(ct.lens[k]), nx=25, ny=13)
        cand = (round(min(pil, 5.0) + 0.5 * e + 40.0 * gl + 0.003 * abs(da), 4), abs(da), a0 + da)
        if cand < best:
            best = cand
    return best[2]


KILL_RADII = (2.6, 3.0, 3.4, 3.8)  # the safety officer at the kill switch: distance from the pad (m)


def place_kill_operator(cam: dict[str, np.ndarray], kinds: list[str], seg: np.ndarray, op_feet: np.ndarray,
                        veh_xy: np.ndarray, cfg: SceneCfg, floor_z: float, hall: Any = None,
                        op_feet_frames: np.ndarray | None = None) -> dict[str, Any] | None:
    """Where the second person (the safety officer holding the KILL switch, params.json mission.vtol demo rule: one
    person at KILL) stands: on a ring 2.6-3.8 m from the pad, >= 2.0 m from every operator position of the edit, >= 1.5 m from the
    vehicle path and from every camera, on the stage (indoors).  Among those, the spot seen whole in the most
    spectator / wide frames and never beside the vehicle or the operator on screen in the hero shots (he may stand in
    their background).  None when nothing qualifies."""
    W, H = cfg.width, cfg.height
    best = None
    sub = np.arange(0, cam["pos"].shape[0], 3)
    n_ = cam["pos"].shape[0]

    def veh_xy_k(k: int) -> np.ndarray:
        return veh_xy[min(k, veh_xy.shape[0] - 1)]

    def op_xy_k(k: int) -> np.ndarray:
        j = int(round(k * (op_frames.shape[0] - 1) / max(n_ - 1, 1)))
        return op_frames[j, :2]
    op_frames = op_feet_frames if op_feet_frames is not None else np.asarray(op_feet, float)
    for r in KILL_RADII:
        for a_deg in np.arange(0.0, 360.0, 15.0):
            a = math.radians(float(a_deg))
            p = np.array([r * math.cos(a), r * math.sin(a)])
            if op_feet.size and float(np.min(np.hypot(*(op_feet[:, :2] - p).T))) < 2.0:
                continue
            if float(np.min(np.hypot(*(veh_xy - p).T))) < 1.5:
                continue
            if float(np.min(np.hypot(*(cam["pos"][:, :2] - p).T))) < 1.5:
                continue
            if hall is not None:
                from . import hall as HL
                if float(HL.stage_margin(hall.to_img(p))[0]) < 0.5:
                    continue
            vis_w, vis_h = 0, 0
            for k in sub:
                kind = kinds[int(seg[k])]
                pts = [np.array([p[0], p[1], floor_z + z]) for z in (0.0, 1.75)]
                inside = 0
                for P_ in pts:
                    x, y, d = project(P_, cam["pos"][k], cam["tgt"][k], float(cam["roll"][k]), float(cam["lens"][k]), W, H,
                                      cam["shift"][k])
                    inside += int(np.isfinite(x) and d > 0 and 0.03 * W < x < 0.97 * W and 0 < y < H)
                seen = inside == 2
                partly = inside == 1
                if kind in ("spectator", "wide", "crane"):
                    vis_w += int(seen) - 2 * int(partly)
                elif seen or partly:
                    # in a hero shot he may stand in the background, but never in front of the action (a large figure in
                    # the foreground) and never next to / behind the vehicle or the operator on screen
                    xk, _, dk = project(pts[0], cam["pos"][k], cam["tgt"][k], float(cam["roll"][k]), float(cam["lens"][k]),
                                        W, H, cam["shift"][k])
                    dv = float(np.linalg.norm(np.array([*veh_xy_k(k), floor_z]) - cam["pos"][k]))
                    if dk < dv + 0.5:
                        vis_h += 3
                    subj = [veh_xy_k(k), op_xy_k(k)]
                    for sp_ in subj:
                        xs_ = project(np.array([sp_[0], sp_[1], floor_z + 1.0]), cam["pos"][k], cam["tgt"][k],
                                      float(cam["roll"][k]), float(cam["lens"][k]), W, H, cam["shift"][k])[0]
                        if np.isfinite(xs_) and abs(xk - xs_) < 0.14 * W:
                            vis_h += 1
                            break
                    vis_h += int(partly)
            score = vis_w - 3 * vis_h - 0.2 * r
            if best is None or score > best[0]:
                best = (score, p, a)
    if best is None:
        return None
    _, p, a = best
    v = np.mean(veh_xy, axis=0) - p
    return {"xy": [float(p[0]), float(p[1])], "yaw": math.atan2(float(v[1]), float(v[0]))}


def place_windsock_background(ct: CamTrack, cfg: SceneCfg, behind_m: float = 42.0, x_frac: float = 0.27
                              ) -> tuple[float, float]:
    """The windsock in the background of the telephoto establishing shot: ``behind_m`` beyond the vehicle along the
    view, ``x_frac`` of the frame width to the side (soft, the focus is on the vehicle; a 6 m mast ~70 m away stands
    whole inside a 85-135 mm frame)."""
    cam = ct.pos[0]
    f = ct.tgt[0] - cam
    f[2] = 0.0
    f /= max(float(np.linalg.norm(f)), 1e-9)
    rgt = np.array([f[1], -f[0], 0.0])
    d = float(ct.focus[0]) + behind_m
    # far enough that the sock at the mast top sits below 8 % of the frame height (the whole windsock in frame)
    W, H = cfg.width, cfg.height
    kpx = float(ct.lens[0]) / SENSOR_W_MM * W
    sy = float(ct.shift[0, 1]) if ct.shift is not None else 0.0
    t_top = (0.42 * H + sy * W) / kpx
    gz = float(cam[2]) - 1.6
    if t_top > 1e-3:
        d = max(d, (gz + SOCK_MAST_H + 0.2 - float(cam[2])) / t_top)
    lat = x_frac * SENSOR_W_MM / float(ct.lens[0]) * d
    p = cam + f * d + rgt * lat
    return float(p[0]), float(p[1])


def windsock_visibility(sock_xy: Sequence[float], gz: float, cam: dict[str, np.ndarray], cfg: SceneCfg,
                        mast_h: float = SOCK_MAST_H, seg: np.ndarray | None = None) -> np.ndarray:
    """(N,) 1 = show the windsock, 0 = hide it: hidden when its mast is in the frame but the sock at the top is not
    (art review round 2: a bare mast standing in the lift-off still read as an unexplained pole).  With ``seg`` the
    decision holds for a whole shot (a mast that pops in or out mid-shot is worse than none)."""
    W, H = cfg.width, cfg.height
    n = cam["pos"].shape[0]
    out = np.ones(n)
    base = np.array([sock_xy[0], sock_xy[1], gz])
    for k in range(n):
        sh = cam["shift"][k] if "shift" in cam else (0.0, 0.0)

        def inside(P: np.ndarray, m: float = 0.0) -> bool:
            x, y, dep = project(P, cam["pos"][k], cam["tgt"][k], float(cam["roll"][k]), float(cam["lens"][k]), W, H, sh)
            return bool(np.isfinite(x) and dep > 0 and m * W <= x <= (1 - m) * W and m * H <= y <= (1 - m) * H)
        mast = any(inside(base + np.array([0.0, 0.0, z])) for z in (0.3, 0.5 * mast_h, 0.85 * mast_h))
        head = inside(base + np.array([0.0, 0.0, mast_h - 0.12]), 0.03)
        if mast and not head:
            out[k] = 0.0
    if seg is not None:
        for i in np.unique(seg):
            m_ = np.asarray(seg) == i
            if np.any(out[m_] < 0.5):
                out[m_] = 0.0
    return out


# --------------------------------------------------------------------------- #
# the plan
# --------------------------------------------------------------------------- #
def build_plan(log: Any, shots: Sequence[Shot], fps: float, cfg: SceneCfg, person: str = "auto",
               geom: dict[str, Any] | None = None) -> dict[str, Any]:
    """All per-frame arrays (see the module docstring) + 'meta' (JSON-able)."""
    geom = geom or {}
    t_all, seg, speed = timeline(shots, fps)
    if t_all.size == 0:
        raise ValueError("empty shot list")
    t = log.t
    pos = interp(t, log.pos, t_all)
    q = hemisphere(slerp_resample(t, log.q, t_all))
    R = quat_to_matrix(q)
    delta = interp(t, log.delta, t_all)
    thrust = interp(t, log.thrust, t_all)
    throttle = interp(t, log.throttle, t_all)
    tref = float(log.t_ref_n)
    rpm = 26000.0 * np.sqrt(np.clip(thrust / max(tref, 1e-6), 0.0, 1.5))
    spin = np.clip(rpm / 2500.0, 0.0, 1.0)
    rotor_angle = np.cumsum(np.concatenate([[0.0], np.diff(t_all) * rpm[1:] / 60.0 * 2 * math.pi]))
    gz = float(log.ground_z)
    jet = jet_state(pos, R, thrust, gz, JetParams(duct_r=float(geom.get("duct_r", 0.045))))
    haze = np.zeros(t_all.size)            # no heat haze (a cold-air jet): kept in the contract, always 0
    wind = interp(t, log.wind, t_all)
    wmean = interp(t, log.wind_mean, t_all) if getattr(log, "wind_mean", None) is not None else wind
    gust = wind - wmean
    sock_yaw, sock_droop = windsock(t_all, wind)
    # person
    has_push = getattr(log, "has_push", False)
    want_person = person == "on" or (person == "auto" and (has_push or cfg.indoor))
    pa = pf = pp = None
    if has_push:
        # per-frame push state: active when any log sample within +-half a frame is active
        pa_i = log.push_active.astype(bool)
        cs = np.concatenate([[0], np.cumsum(pa_i)])
        half = 0.5 * np.maximum(speed / fps, 1e-4)
        lo = np.searchsorted(t, t_all - half, side="left")
        hi = np.searchsorted(t, t_all + half, side="right")
        hold_i = np.clip(np.searchsorted(t, t_all, side="right") - 1, 0, t.size - 1)
        hi = np.maximum(hi, hold_i + 1)
        lo = np.minimum(lo, hold_i)
        pa = (cs[hi] - cs[lo]) > 0
        pf = interp(t, log.push_force, t_all) * pa[:, None]
        pp = interp(t, log.push_point, t_all)
    ptrack = None
    pt = None
    floor_z = gz + FLOOR_DZ[bool(cfg.indoor)]
    if want_person:
        # the person needs the continuous log-rate history (walks between pushes span cut-out time): compute on
        # a uniform 50 Hz grid over the log, then sample at the frame times
        tg = np.arange(t[0], t[-1], 0.02)
        pos_g = interp(t, log.pos, tg)
        R_g = quat_to_matrix(slerp_resample(t, log.q, tg))
        if has_push:
            pa_g = hold(t, log.push_active.astype(bool), tg)
            pf_g = interp(t, log.push_force, tg)
            pp_g = interp(t, log.push_point, tg)
        else:
            pa_g = pf_g = pp_g = None
        pt = person_track(tg, pos_g, R_g, floor_z, pa_g, pf_g, pp_g)
        keys = ("feet", "yaw", "push_w", "walk_w", "walk_phase", "walk_dir", "hold", "pole_dir", "pad", "butt", "grip_r",
                "grip_l", "look")
        ptrack = {k: interp(tg, getattr(pt, k), t_all) for k in keys}
        pdv = ptrack["pole_dir"]
        ptrack["pole_dir"] = pdv / np.maximum(np.linalg.norm(pdv, axis=1, keepdims=True), 1e-12)
        # never a walk cycle inside a push (the interpolation must not leak one in either)
        ptrack["walk_w"] = np.where(ptrack["push_w"] > 0.05, 0.0, ptrack["walk_w"])
        ptrack["notes"] = list(pt.notes)
        if pa is not None:
            sh = snap_pad(ptrack, pa, interp(t, log.push_force, t_all), pp, pos, R)
            ptrack["notes"].append(f"person: pad snapped tangent to the body in every push-active frame (max shift "
                                   f"{sh * 1000:.1f} mm)")
    # ---- windsock placement (outdoor scene dressing): 6.5 m from where the vehicle meets the first gust, on the side
    # perpendicular to that wind, so the telephoto establishing shot from beyond the sock sees it streaming side-on
    # (not toward the lens), lit from the side ----
    sock_xy = tuple(cfg.windsock_xy)
    if not cfg.indoor and cfg.windsock_auto:
        ga = np.asarray(log.gust_active, bool)
        wsel = ga if np.any(ga) else (np.linalg.norm(np.asarray(log.wind)[:, :2], axis=1) > 0.5)
        if np.any(wsel):
            i0 = int(np.flatnonzero(wsel)[0])
            j1 = i0 + int(np.argmax(~wsel[i0:])) if np.any(~wsel[i0:]) else t.size
            w = np.mean(np.asarray(log.wind)[i0:max(j1, i0 + 1), :2], axis=0)
            if float(np.hypot(*w)) > 0.5:
                wu = w / float(np.hypot(*w))
                sun = np.array([math.cos(math.radians(cfg.sun_az_deg)), math.sin(math.radians(cfg.sun_az_deg))])
                # view direction = -p: prefer the sun ~70 deg off the lens axis on the far side (dot ~ +0.35): side /
                # back light gives the matte-black body a shape (front light flattened it to 77 sRGB grey) and the sun
                # stays far out of an 18-50 deg telephoto frame
                p = min((np.array([-wu[1], wu[0]]), np.array([wu[1], -wu[0]])), key=lambda pp: abs(float(-pp @ sun) - 0.35))
                anchor = np.asarray(log.pos[i0, :2], float)
                sock_xy = (float(anchor[0] + 6.5 * p[0]), float(anchor[1] + 6.5 * p[1]))
    # ---- camera side of the action line (180-degree rule) and, indoors, the room around the demo ----
    cg_all = pos + R[:, :, 2] * Z_CG_DEFAULT
    side, hall, notes_cam, emap, lmap = 1, None, [], None, None
    is_push = [str(s.params.get("kind", "")) == "push" or str(s.note).startswith("push") for s in shots]

    def hero_views(sd: int) -> list[tuple[np.ndarray, float]]:
        """Hero camera views from the LOG, not from the shot list (art review round 2: the stills and the clip of one
        demo were fitted into differently rotated halls): every push (the push two-shot azimuth at its middle) and the
        lift-off (the spectator azimuth)."""
        out: list[tuple[np.ndarray, float]] = []
        if pt is None:
            return out
        cg_g = pos_g + R_g[:, :, 2] * Z_CG_DEFAULT
        for a, b in (push_runs(pa_g) if pa_g is not None else []):
            k = (int(a) + int(b)) // 2
            bp = math.atan2(float(pt.feet[k, 1] - pos_g[k, 1]), float(pt.feet[k, 0] - pos_g[k, 0]))
            out.append((cg_g[k], side_azimuth("orbit", True, bp, sd)))
        up = np.flatnonzero(pos_g[:, 2] > gz + 0.2)
        k = int(up[0]) if up.size else 0
        bp = math.atan2(float(pt.feet[k, 1] - pos_g[k, 1]), float(pt.feet[k, 0] - pos_g[k, 0]))
        out.append((cg_g[k], side_azimuth("spectator", False, bp, sd)))
        return out
    if cfg.indoor:
        from . import hall as HL
        hdr = os.path.join(cfg.assets_dir, "school_hall", "school_hall_4k.hdr")
        emap = HL.edge_map(hdr, os.path.join(os.path.dirname(cfg.assets_dir), "cache"))
        lmap = HL.lum_map(hdr, os.path.join(os.path.dirname(cfg.assets_dir), "cache"))
        # the room is fitted to the WHOLE log (vehicle path, every operator position, the hero views): one hall per
        # log, the same for every shot list (round 2 review: the stills and the clip of one demo sat in two halls)
        if pt is not None:
            veh_xy = (pos_g + R_g[:, :, 2] * Z_CG_DEFAULT)[::10, :2]
            feet_xy = pt.feet[::10, :2]
        else:
            veh_xy = (log.pos + quat_to_matrix(log.q)[:, :, 2] * Z_CG_DEFAULT)[::5, :2]
            feet_xy = np.zeros((0, 2))
        fits = {sd: HL.fit(veh_xy, feet_xy, hero_views(sd), gz - 0.018, emap, lmap=lmap, cam_dist=5.6)
                for sd in (1, -1)}
        side = max((1, -1), key=lambda sd: (fits[sd].feasible, fits[sd].score))
        hall = fits[side]
        notes_cam += hall.notes
    elif ptrack is not None:
        hv = {sd: hero_views(sd) for sd in (1, -1)}
        side = min((1, -1), key=lambda sd: sum(abs(ang_diff(math.radians(a), math.radians(cfg.sun_az_deg))) for _, a in hv[sd]))
    # cameras per shot
    cam = {k: np.zeros((t_all.size, 3)) for k in ("pos", "tgt")}
    for k in ("roll", "lens", "focus", "fstop", "fill", "ev"):
        cam[k] = np.zeros(t_all.size)
    cam["quat"] = np.zeros((t_all.size, 4))
    cam["shift"] = np.zeros((t_all.size, 2))
    kinds = []
    tele_shot = None

    def sock_pts(sel_: np.ndarray) -> np.ndarray:
        """The windsock as the rendered model shows it in a shot: pole top + the sock's tail (yaw / root droop of
        render.plan.windsock at the shot's middle frame; render/blender/props.py swivel 2.88 m, sock 1.2 m)."""
        k = int(np.flatnonzero(sel_)[len(np.flatnonzero(sel_)) // 2])
        yw, dr = float(sock_yaw[k]), float(sock_droop[k, 0])
        L_ = 1.3
        zs = gz + SOCK_MAST_H - 0.12
        tail = [sock_xy[0] + L_ * math.cos(dr) * math.cos(yw), sock_xy[1] + L_ * math.cos(dr) * math.sin(yw),
                zs - L_ * math.sin(dr) - 0.12]
        return np.array([[sock_xy[0], sock_xy[1], zs + 0.14], tail,
                         [sock_xy[0] + 0.25 * math.cos(yw), sock_xy[1] + 0.25 * math.sin(yw), zs - 0.18]])
    for i, s in enumerate(shots):
        sel = seg == i
        params = dict(s.params)
        if ptrack is not None and "azimuth_deg" not in params and not (s.cam == "wide" and not cfg.indoor):
            bp = person_bearing(ptrack["feet"][sel], pos[sel])
            params["azimuth_deg"] = side_azimuth(s.cam, is_push[i], bp, side) + float(params.get("az_offset_deg", 0.0))
            if s.cam == "orbit":
                # turn away from the line, never across - also when the shot list leaves the rate at its default
                # (round 1b: an auto push orbit on side -1 swung 55 deg toward the operator's back)
                params["deg_per_s"] = abs(float(params.get("deg_per_s", 11.0))) * side
            if hall is not None and is_push[i] and s.cam == "orbit" and "az_offset_deg" not in params:
                # nudge the push camera (same side, +-12 deg) to the azimuth whose backplate below the vehicle has the
                # least vertical-edge energy (the white pillars between the blinds read as a rod holding the vehicle)
                params["azimuth_deg"] = _refine_backplate(s, params, t_all[sel], pos[sel], R[sel], gz, cfg, i, hall,
                                                          emap, ptrack["feet"][sel], side, lmap,
                                                          0.5 * (ptrack["hold"][sel] + ptrack["pad"][sel]))
        tele = s.cam == "wide" and not cfg.indoor and str(params.get("mode", "tele")) == "tele"
        ct = camera_for_shot(s.cam, t_all[sel], pos[sel], R[sel], gz, cfg, i, params,
                             extra_pts=sock_pts(sel) if s.cam == "wide" and not cfg.indoor and not tele else None,
                             hall=hall, person_feet=ptrack["feet"][sel] if ptrack is not None else None,
                             pole_mid=0.5 * (ptrack["hold"][sel] + ptrack["pad"][sel]) if ptrack is not None else None)
        for k in ("pos", "tgt", "roll", "lens", "focus", "fstop"):
            cam[k][sel] = getattr(ct, k)
        if ct.fill is not None:
            cam["fill"][sel] = ct.fill
        if ct.shift is not None:
            cam["shift"][sel] = ct.shift
        if tele and tele_shot is None:
            tele_shot = (sel.copy(), ct)
        cam["ev"][sel] = shot_ev(s.cam, cfg.indoor, params)
        qs = np.array([look_quat(p_, t_, r_) for p_, t_, r_ in zip(ct.pos, ct.tgt, ct.roll)])
        cam["quat"][sel] = hemisphere(qs)
        kinds.append(s.cam)
    if tele_shot is not None and cfg.windsock_auto:
        sock_xy = place_windsock_background(tele_shot[1], cfg)
    sock_vis = windsock_visibility(sock_xy, gz, cam, cfg, seg=seg) if not cfg.indoor else np.zeros(t_all.size)
    cut_after = np.zeros(t_all.size, bool)
    cut_after[:-1] = seg[1:] != seg[:-1]
    cut_after[-1] = True
    plan: dict[str, Any] = {
        "t": t_all, "seg": seg, "speed": speed, "cut_after": cut_after,
        "pos": pos, "q": q, "delta": delta, "thrust": thrust, "throttle": throttle, "spin": spin,
        "rotor_angle": rotor_angle, "tilt_deg": interp(t, log.tilt_deg, t_all),
        "jet_impact": jet["impact"], "jet_u": jet["u_ground"], "jet_b": jet["half_width"], "jet_len": jet["length"],
        "jet_hit": jet["hit"], "dust": jet["dust"], "haze": haze, "jet_k": jet["wall_k"],
        "wind": wind, "wind_mean": wmean, "gust": gust, "sock_yaw": sock_yaw, "sock_droop": sock_droop,
        "cam_pos": cam["pos"], "cam_tgt": cam["tgt"], "cam_roll": cam["roll"], "cam_lens": cam["lens"],
        "cam_focus": cam["focus"], "cam_fstop": cam["fstop"], "cam_quat": cam["quat"], "cam_fill": cam["fill"],
        "cam_ev": cam["ev"], "cam_shift": cam["shift"], "sock_vis": sock_vis,
        "state": hold(t, log.state, t_all), "phase": hold(t, log.phase, t_all),
        "h_target": hold(t, log.h_target, t_all),
    }
    tape_meta = None
    if cfg.indoor:
        root_xy, tdir = tape_end_site(ptrack["feet"][:, :2] if ptrack is not None else None)
        r_t = np.hypot(*(jet["impact"][:, :2] - root_xy[None, :]).T)
        u_t = wall_jet_speed(r_t, jet["u_ground"], jet["half_width"], jet["wall_k"])
        plan["tape_lift"] = tape_flutter(t_all, np.where(jet["hit"], u_t, 0.0))
        tape_meta = {"root": [float(x) for x in root_xy], "dir": [float(x) for x in tdir], "seg_m": TAPE_SEG_M,
                     "half": TAPE_HALF}
    if pa is not None:
        plan.update({"push_active": pa, "push_force": pf, "push_point": pp})
    if ptrack is not None:
        for k, v in ptrack.items():
            if k != "notes":
                plan["person_" + k] = v
    kill_op = None
    if cfg.indoor and ptrack is not None:
        # (the operator positions of THIS edit: with every push of the log the stage was too crowded and the officer
        #  ended up out of every shot - round 2 probe)
        kill_op = place_kill_operator(cam, kinds, seg, ptrack["feet"], cg_all[:, :2], cfg, floor_z, hall,
                                      op_feet_frames=ptrack["feet"])
    key_light = None
    if cfg.indoor:
        # (a theatre spot has a small source: 6 cm radius; a 0.25 m source blurred the 12 cm wide body's shadow from 2 m
        #  up into a 0.4 m smudge - 3 % darker in the round-2 probe)
        # a motivated stage spot over the demo square (art review round 2: the 70 W stage boxes 6.2 m up cast nothing -
        # the hovering vehicle read as pasted in): 4.2 m above the floor, 20 deg off vertical, from 50 deg round from
        # the hero push camera (a three-quarter front-top light: the shadows fall 0.7 m beyond the vehicle, on the board /
        # floor in view; round-2 probe: from 140 deg they fell out of the frame toward the lens)
        hv = hero_views(side)
        az_c = math.radians(hv[0][1]) if hv else math.radians(_azimuth_for(cfg, 0, "spectator"))
        az_k = az_c + math.radians(KEY_SPOT["az_from_cam_deg"])
        hk = KEY_SPOT["height_m"]
        rk = hk * math.tan(math.radians(KEY_SPOT["off_vertical_deg"]))
        key_light = {"pos": [rk * math.cos(az_k), rk * math.sin(az_k), floor_z + hk], "aim": [0.0, 0.0, floor_z + 0.9],
                     "power_w": KEY_SPOT["power_w"], "radius_m": KEY_SPOT["radius_m"], "spot_deg": KEY_SPOT["spot_deg"],
                     "blend": KEY_SPOT["blend"]}
    plan["meta"] = {
        "key_light": key_light, "white_balance_k": WHITE_BALANCE_K.get("indoor" if cfg.indoor else "outdoor"),
        "kill_operator": kill_op,
        "floor_z": floor_z,
        "scene": "indoor" if cfg.indoor else "outdoor", "exposure": -0.15 if cfg.indoor else 0.0,
        "fps": fps, "frames": int(t_all.size), "ground_z": gz, "t_ref_n": tref, "shots": [s.to_json() for s in shots],
        "cam_kinds": kinds, "person": bool(ptrack is not None), "indoor": cfg.indoor, "sun_az_deg": cfg.sun_az_deg,
        "windsock_xy": list(sock_xy), "windsock_h": SOCK_MAST_H, "notes": (ptrack or {}).get("notes", []) + notes_cam,
        "cam_side": side, "hall": hall.to_json() if hall is not None else None, "tape_end": tape_meta,
        "pole": {"len_m": POLE_LEN, "hold_to_pad_m": POLE_HOLD_TO_PAD, "butt_m": POLE_BUTT_M, "pad_r_m": POLE_PAD_R,
                 "r_m": POLE_R, "max_pitch_deg": POLE_MAX_PITCH_DEG, "sleeve_m": list(POLE_SLEEVE)},
        "illustrations": ["grass bending, grass clippings / grit and the dust over the slab from the free-jet + wall-jet "
                          "model (render.plan.jet_state, wall_jet_speed: u(r) = min(u_ground, sqrt(T/rho)/r))",
                          "windsock droop / flutter (render.plan.windsock)",
                          "indoor: the loose hazard-tape end fluttering in the wall jet",
                          "person pose (tools/human_figure solver), pole carry / push and walking "
                          "(render.plan.person_track; pole <= 15 deg from horizontal)", "EDF rotor blur from thrust"],
    }
    return plan


def vane_wiggle(t: np.ndarray, t0: float = 1.8, dt: float = 1.1, amp_deg: float = 15.0, n_vanes: int = 4) -> np.ndarray:
    """Reveal choreography (no log): vane i swings +amp -> -amp -> 0 in a smooth 1.1 s pulse starting at
    t0 + i * dt (like a pre-flight servo check), then all four together once."""
    out = np.zeros((t.size, n_vanes))
    a = math.radians(amp_deg)
    for i in range(n_vanes + 1):
        s = (t - (t0 + i * dt)) / dt
        w = np.where((s > 0) & (s < 1), np.sin(2 * math.pi * np.clip(s, 0, 1)) * np.sin(math.pi * np.clip(s, 0, 1)), 0.0)
        if i < n_vanes:
            out[:, i] += a * w
        else:
            out += a * 0.8 * w[:, None]
    return out


def turntable_plan(fps: float = 30.0, seconds: float = 9.0, ground_z: float = -0.11, cfg: SceneCfg | None = None,
                   rot_deg: float = 110.0) -> dict[str, Any]:
    """The 'vehicle reveal' (independent of any log): vehicle on its pad on a studio turntable turning ``rot_deg``
    with an eased start / stop, a slow camera crane (render.plan.camera_for_shot 'turntable'), and a servo-check
    vane choreography (``vane_wiggle``).  Same array contract as ``build_plan`` (+ 'tt_angle')."""
    cfg = cfg or SceneCfg(sun_az_deg=-60.0)
    n = int(round(seconds * fps))
    t = np.arange(n) / fps
    s = np.asarray(smoothstep(t / max(seconds, 1e-6)))
    tt = np.radians(-35.0 + rot_deg * (0.15 * t / seconds + 0.85 * s))
    pos = np.zeros((n, 3))
    q = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    R = quat_to_matrix(q)
    ct = camera_for_shot("turntable", t, pos, R, ground_z, cfg, 0, {"azimuth_deg": -60.0})
    # reveal beat (art review round 2: 8 s of identical rotation): the rim strips come up first (0.2-1.1 s: a
    # silhouette in the dark), then the key / fill / top (0.9-2.3 s), while the camera pushes in 0.5 m (0-2.4 s)
    rim = np.asarray(smoothstep((t - 0.2) / 0.9))
    key_up = np.asarray(smoothstep((t - 0.9) / 1.4))
    push = 0.5 * (1.0 - np.asarray(smoothstep(t / 2.4)))
    c0 = np.array([0.0, 0.0, 0.40])
    dvec = ct.pos - c0
    dvec[:, 2] = 0.0
    dn = dvec / np.maximum(np.linalg.norm(dvec, axis=1, keepdims=True), 1e-9)
    ct.pos = ct.pos + dn * push[:, None]
    # frame the whole vehicle + pad: 40 mm, aim a little lower
    ct.lens[:] = 40.0
    ct.tgt[:, 2] = 0.36
    ct.focus = np.linalg.norm(ct.tgt - ct.pos, axis=1)
    quat = hemisphere(np.array([look_quat(p_, t_, r_) for p_, t_, r_ in zip(ct.pos, ct.tgt, ct.roll)]))
    cut_after = np.zeros(n, bool)
    cut_after[-1] = True
    z3 = np.zeros((n, 3))
    plan: dict[str, Any] = {
        "t": t, "seg": np.zeros(n, int), "speed": np.ones(n), "cut_after": cut_after, "pos": pos, "q": q,
        "delta": vane_wiggle(t), "thrust": np.zeros(n), "throttle": np.zeros(n), "spin": np.zeros(n),
        "rotor_angle": np.zeros(n), "tilt_deg": np.zeros(n), "jet_impact": z3.copy(), "jet_u": np.zeros(n),
        "jet_b": np.full(n, 0.05), "jet_len": np.zeros(n), "jet_hit": np.zeros(n, bool), "dust": np.zeros(n),
        "haze": np.zeros(n), "jet_k": np.zeros(n), "wind": z3.copy(), "wind_mean": z3.copy(), "gust": z3.copy(), "sock_yaw": np.zeros(n),
        "sock_droop": np.zeros((n, N_SOCK)), "cam_pos": ct.pos, "cam_tgt": ct.tgt, "cam_roll": ct.roll,
        "cam_lens": ct.lens, "cam_focus": ct.focus, "cam_fstop": ct.fstop, "cam_quat": quat, "cam_fill": np.zeros(n),
        "cam_ev": np.zeros(n), "cam_shift": np.zeros((n, 2)), "sock_vis": np.zeros(n),
        "state": np.zeros(n, int), "phase": np.zeros(n, int), "h_target": np.full(n, np.nan), "tt_angle": tt,
        "studio_rim": rim, "studio_key": key_up,
    }
    plan["meta"] = {"scene": "studio", "fps": fps, "frames": n, "ground_z": ground_z, "t_ref_n": 1.0,
                    "shots": [{"t0": 0.0, "t1": seconds, "cam": "turntable", "slow": [], "note": "reveal", "params": {}}],
                    "cam_kinds": ["turntable"], "person": False, "indoor": False, "sun_az_deg": cfg.sun_az_deg,
                    "windsock_xy": [0.0, 0.0], "notes": [], "turntable": True,
                    "illustrations": ["vane servo-check choreography (render.plan.vane_wiggle), not a log",
                                      "studio light-up reveal (rims, then key) and a 0.5 m push-in over the first 2.4 s"]}
    return plan


def save_plan(plan: dict[str, Any], path: str) -> None:
    import json
    arrays = {k: np.asarray(v) for k, v in plan.items() if k != "meta"}
    np.savez_compressed(path, meta=np.array(json.dumps(plan["meta"])), **arrays)
