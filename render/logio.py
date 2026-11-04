"""Flight-log access for the Blender pipeline: the renderer npz contract of tools/sim_video.py.

``load_log`` uses ``tools.sim_video.load_log`` (read-only import: the contract's reference loader, which
also converts ``meta.pos_ref = 'cg'`` logs to the exit-plane origin).  Should that module fail to import
(it is owned and edited by another workflow), a compact fallback loader with the same contract subset is
used and a warning says so.  Either way the result is a ``sim_video.SimLog``-like object whose ``pos`` is
the duct exit-plane centre in world metres (z up, origin = pad top).
"""
from __future__ import annotations

import hashlib
import json
import os
import sys
from dataclasses import dataclass, field
from typing import Any

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """R(q) body -> world for q = (w, x, y, z), (..., 4) -> (..., 3, 3) (same as sim.quat / sim_video)."""
    q = np.asarray(q, dtype=float)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = w * w + x * x - y * y - z * z
    R[..., 0, 1] = 2.0 * (x * y - w * z)
    R[..., 0, 2] = 2.0 * (x * z + w * y)
    R[..., 1, 0] = 2.0 * (x * y + w * z)
    R[..., 1, 1] = w * w - x * x + y * y - z * z
    R[..., 1, 2] = 2.0 * (y * z - w * x)
    R[..., 2, 0] = 2.0 * (x * z - w * y)
    R[..., 2, 1] = 2.0 * (y * z + w * x)
    R[..., 2, 2] = w * w - x * x - y * y + z * z
    return R


@dataclass
class Log:
    """The fields the pipeline uses (a subset of sim_video.SimLog, same meaning and units)."""

    t: np.ndarray
    pos: np.ndarray
    q: np.ndarray
    vel: np.ndarray
    delta: np.ndarray
    throttle: np.ndarray
    thrust: np.ndarray
    state: np.ndarray
    phase: np.ndarray
    h_target: np.ndarray
    tilt_deg: np.ndarray
    wind: np.ndarray
    gust_active: np.ndarray
    ground_z: float
    event_t: np.ndarray
    event_text: list[str]
    cmd_t: np.ndarray
    cmd_name: list[str]
    meta: dict[str, Any]
    sha256: str
    source: str
    push_force: np.ndarray | None = None
    push_point: np.ndarray | None = None
    push_active: np.ndarray | None = None
    wind_mean: np.ndarray | None = None
    warnings: list[str] = field(default_factory=list)
    loader: str = "sim_video"

    @property
    def n(self) -> int:
        return int(self.t.shape[0])

    @property
    def has_push(self) -> bool:
        return self.push_active is not None and bool(np.any(self.push_active))

    @property
    def t_ref_n(self) -> float:
        tm = self.meta.get("t_max_n")
        if tm:
            return float(tm)
        m = float(np.nanmax(self.thrust)) if self.n else 0.0
        return m if m > 1e-6 else 1.0

    @property
    def profile(self) -> str:
        """'demo' (indoor / low hover with pushes) or 'field' (meta.profile, else guessed from the log)."""
        p = str(self.meta.get("profile", "")).lower()
        if p in ("demo", "field"):
            return p
        hmax = float(np.max(self.pos[:, 2] - self.pos[0, 2])) if self.n else 0.0
        return "demo" if (self.has_push and hmax < 3.0) else "field"


def _from_simlog(s: Any) -> Log:
    return Log(t=s.t, pos=s.pos, q=s.q, vel=s.vel, delta=s.delta, throttle=s.throttle, thrust=s.thrust, state=s.state,
               phase=s.phase, h_target=s.h_target, tilt_deg=s.tilt_deg, wind=s.wind, gust_active=s.gust_active,
               ground_z=float(s.ground_z), event_t=s.event_t, event_text=list(s.event_text), cmd_t=s.cmd_t,
               cmd_name=list(s.cmd_name), meta=dict(s.meta), sha256=s.sha256, source=s.source,
               push_force=s.push_force, push_point=s.push_point, push_active=s.push_active, wind_mean=s.wind_mean,
               warnings=list(s.warnings), loader="sim_video")


def load_log(path: str) -> Log:
    try:
        from tools import sim_video as sv           # read-only use of the reference loader
        return _from_simlog(sv.load_log(path))
    except (ImportError, SyntaxError, AttributeError, NameError) as e:
        log = _fallback_load(path)
        log.warnings.append(f"tools.sim_video unavailable ({type(e).__name__}: {e}); fallback loader used")
        log.loader = "fallback"
        return log


def _fallback_load(path: str) -> Log:
    with open(path, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    with np.load(path, allow_pickle=False) as d:
        data = {k: d[k] for k in d.files}
    for k in ("t", "pos", "q"):
        if k not in data:
            raise ValueError(f"log {path!r} lacks {k!r}")
    t = np.asarray(data["t"], float).reshape(-1)
    n = t.size
    if n < 2 or np.any(np.diff(t) <= 0):
        raise ValueError("t must have >= 2 strictly increasing samples")
    pos = np.asarray(data["pos"], float).reshape(n, 3)
    q = np.asarray(data["q"], float).reshape(n, 4)
    q = q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)
    for i in range(1, n):
        if float(np.dot(q[i], q[i - 1])) < 0.0:
            q[i] = -q[i]
    warnings: list[str] = []
    try:
        meta = json.loads(str(np.asarray(data.get("meta", "{}")).reshape(-1)[0]))
    except (ValueError, TypeError):
        meta, _ = {}, warnings.append("meta not JSON")

    def g(k: str, shape: tuple[int, ...], default: Any, dtype: Any = float) -> np.ndarray:
        if k not in data:
            return default
        return np.asarray(data[k]).astype(dtype).reshape(shape)
    if str(meta.get("pos_ref", "exit_plane")) == "cg":
        pos = pos - quat_to_matrix(q) @ np.array([0.0, 0.0, float(meta["z_cg_m"])])
    tilt = g("tilt_deg", (n,), None)
    if tilt is None:
        c = 1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2)
        tilt = np.degrees(np.arccos(np.clip(c, -1, 1)))
    pf = g("push_force", (n, 3), None)
    pa = g("push_active", (n,), None, bool)
    if pf is not None and pa is None:
        pa = np.linalg.norm(pf, axis=1) > 0.05
    pp = g("push_point", (n, 3), None)
    if pf is not None and pp is None:
        pp = pos + quat_to_matrix(q) @ np.array([0.0, 0.0, float(meta.get("z_cg_m", 0.39))])
    return Log(t=t, pos=pos, q=q, vel=g("vel", (n, 3), np.gradient(pos, t, axis=0)), delta=g("delta", (n, 4), np.zeros((n, 4))),
               throttle=g("throttle", (n,), np.zeros(n)), thrust=g("thrust", (n,), np.zeros(n)),
               state=g("state", (n,), np.zeros(n, int), int), phase=g("phase", (n,), np.zeros(n, int), int),
               h_target=g("h_target", (n,), np.full(n, np.nan)), tilt_deg=tilt, wind=g("wind", (n, 3), np.zeros((n, 3))),
               gust_active=g("gust_active", (n,), np.zeros(n, bool), bool),
               ground_z=float(np.asarray(data.get("ground_z", -0.11)).reshape(-1)[0]),
               event_t=np.asarray(data.get("event_t", np.zeros(0)), float).reshape(-1),
               event_text=[str(x) for x in np.atleast_1d(data.get("event_text", np.zeros(0, dtype=str))).tolist()],
               cmd_t=np.asarray(data.get("cmd_t", np.zeros(0)), float).reshape(-1),
               cmd_name=[str(x) for x in np.atleast_1d(data.get("cmd_name", np.zeros(0, dtype=str))).tolist()],
               meta=meta, sha256=sha, source=os.path.abspath(path), push_force=pf, push_point=pp, push_active=pa,
               wind_mean=g("wind_mean", (n, 3), None), warnings=warnings)
