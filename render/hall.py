"""Indoor demo hall (pure numpy): the school_hall HDRI's real room, measured, and where the demo sits in it.

The Poly Haven ``school_hall`` HDRI is shot ON THE STAGE of a school auditorium.  Measured from the image
(render/hdr_io + the floor / wall junction elevation per azimuth, capture height 1.6 m assumed): the stage is a
rectangle around the capture point, in the image's own frame (x' toward image azimuth 0, y' toward +90 deg):

    x' -3.9 .. +4.75 m (curtained wings), y' -2.05 m (the stage front edge, audience side) .. +4.3 m (back curtains)

with the auditorium (chairs, windowed side walls, back wall) beyond the stage edge, ~1.0 m lower.  The previous
round projected the photo onto a 24 x 18 m box: everything outside the true stage smeared (curtain bottoms
sheared over the floor, the stage front streaked, a 'ghost ramp').  Now the proxy geometry follows the real room
(``proxy_boxes``), the stage floor is entirely CG (no projected floor on the stage), and every indoor camera stays on
the stage (``clamp_xy``).

The room is placed around the demo by ``fit``: a rotation (world azimuth = image azimuth - rot) and the vehicle's
position on the stage (image frame) chosen so that the vehicle path and every operator position fit on the stage
with margins, the push cameras keep their distance, and the backdrop behind the vehicle in the push shots is the bright
auditorium without strong vertical edges (the white pillars between the window blinds read as a rod holding the
vehicle) - ``backplate_edges`` samples a vertical-edge map of the HDRI along the camera rays below the vehicle.

Invariant: deterministic; the only file input is the HDRI (optional: without it the edge term is 0).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

STAGE = (-3.9, 4.75, -2.05, 4.3)      # x'0, x'1, y'0 (stage edge), y'1 (back curtains), metres from the capture point
CAP_H = 1.6                           # capture height above the stage floor (assumed; distances scale with it)
AUDIENCE_DROP = 1.0                   # auditorium floor below the stage floor
AUDIENCE = (-8.0, 8.0, -28.0)         # auditorium side walls x' = +-8 m, back wall y' = -28 m
STAGE_TOP = 7.5                       # stage ceiling / curtain top above the stage floor
HALL_TOP = 8.5                        # auditorium ceiling above the stage floor
AUDIENCE_SECTOR = (-150.0, -30.0)     # image azimuths (deg) looking out into the auditorium
PIANO_SECTOR = (-180.0, -140.0)       # a grand piano + lectern near the stage edge: flattened by any proxy, avoid


@dataclass
class Hall:
    rot: float                         # rad: world azimuth = image azimuth - rot
    veh_img: np.ndarray                # (2,) the world origin (the pad) in the image frame (m from the capture point)
    floor_z: float                     # stage floor (world z)
    score: float = 0.0
    feasible: bool = True
    notes: list[str] = field(default_factory=list)

    @property
    def cap_xy(self) -> np.ndarray:
        """World xy of the capture point."""
        return -rot2(-self.rot) @ self.veh_img

    def to_world(self, p_img: np.ndarray) -> np.ndarray:
        """Image-frame xy (relative to the capture point) -> world xy."""
        p = np.asarray(p_img, float)
        return (rot2(-self.rot) @ (p - self.veh_img).T).T if p.ndim == 2 else rot2(-self.rot) @ (p - self.veh_img)

    def to_img(self, p_w: np.ndarray) -> np.ndarray:
        p = np.asarray(p_w, float)
        return (rot2(self.rot) @ p.T).T + self.veh_img if p.ndim == 2 else rot2(self.rot) @ p + self.veh_img

    def to_json(self) -> dict[str, Any]:
        return {"rot_rad": float(self.rot), "veh_img": [float(x) for x in self.veh_img], "floor_z": float(self.floor_z),
                "cap_xy": [float(x) for x in self.cap_xy], "cap_h": CAP_H, "stage": list(STAGE),
                "audience_drop": AUDIENCE_DROP, "audience": list(AUDIENCE), "stage_top": STAGE_TOP, "hall_top": HALL_TOP,
                "score": float(self.score), "feasible": bool(self.feasible), "notes": list(self.notes)}


def rot2(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def stage_margin(p_img: np.ndarray) -> np.ndarray:
    """Signed distance (m) of image-frame points inside the stage rectangle (positive inside)."""
    p = np.atleast_2d(np.asarray(p_img, float))
    x0, x1, y0, y1 = STAGE
    return np.minimum.reduce([p[:, 0] - x0, x1 - p[:, 0], p[:, 1] - y0, y1 - p[:, 1]])


def clamp_xy(hall: Hall, subject_w: np.ndarray, cam_w: np.ndarray, margin: float = 0.35) -> np.ndarray:
    """Pull a camera (world xy) toward its subject until it is ``margin`` inside the stage (the room's walls / the
    stage edge); returns the clamped world xy."""
    s = hall.to_img(np.asarray(subject_w, float)[:2])
    c = hall.to_img(np.asarray(cam_w, float)[:2])
    if float(stage_margin(c)[0]) >= margin:
        return np.asarray(cam_w, float)[:2].copy()
    lo, hi = 0.0, 1.0                                  # fraction of the way subject -> camera
    if float(stage_margin(s)[0]) < margin:
        return np.asarray(subject_w, float)[:2].copy()
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if float(stage_margin(s + mid * (c - s))[0]) >= margin:
            lo = mid
        else:
            hi = mid
    return hall.to_world(s + lo * (c - s))


# --------------------------------------------------------------------------- #
# backdrop analysis (vertical-edge map of the HDRI)
# --------------------------------------------------------------------------- #
_EDGE_CACHE: dict[str, np.ndarray] = {}


def edge_map(hdr_path: str, cache_dir: str) -> np.ndarray | None:
    """(H, W) vertical-edge energy of the HDRI (|d log L / du|, blurred), image rows top -> bottom; None without it."""
    if hdr_path in _EDGE_CACHE:
        return _EDGE_CACHE[hdr_path]
    if not os.path.exists(hdr_path):
        return None
    from .hdr_io import cached_preview
    img = cached_preview(hdr_path, cache_dir, step=4)
    L = np.log(np.maximum(img.mean(axis=2), 1e-4))
    gx = np.abs(np.roll(L, -1, axis=1) - np.roll(L, 1, axis=1))
    gy = np.abs(np.roll(L, -1, axis=0) - np.roll(L, 1, axis=0))
    e = np.clip(gx - 0.5 * gy, 0.0, None)              # vertical edges only (a horizontal edge is harmless)
    k = np.array([1, 2, 3, 2, 1], float)
    k /= k.sum()
    for ax in (0, 1):
        e = np.apply_along_axis(lambda v: np.convolve(np.concatenate([v[-2:], v, v[:2]]), k, mode="valid"), ax, e)
    _EDGE_CACHE[hdr_path] = e
    return e


def sample_dirs(emap: np.ndarray, dirs_img: np.ndarray) -> np.ndarray:
    from .hdr_io import dir_to_uv
    u, v = dir_to_uv(dirs_img)
    h, w = emap.shape
    return emap[np.clip((v * h).astype(int), 0, h - 1), np.clip((u * w).astype(int), 0, w - 1)]


def backplate_rays(cam: np.ndarray, veh_exit: np.ndarray, lens_mm: float, n: int = 24, H: int = 1080,
                   W: int = 1920, cols: int = 5) -> np.ndarray:
    """World directions of camera rays through a screen strip directly BELOW the vehicle (+-3 % of the frame width,
    from the duct exit to 40 % further down the frame): where a bright vertical column in the backplate reads as a
    support rod."""
    f = np.asarray(veh_exit, float) - np.asarray(cam, float)
    f /= np.linalg.norm(f)
    r = np.cross(f, [0.0, 0.0, 1.0])
    r /= max(np.linalg.norm(r), 1e-9)
    u = np.cross(r, f)
    k = lens_mm / 36.0 * W
    ys = np.linspace(0.0, 0.40 * H, n)
    xs = np.linspace(-0.03, 0.03, cols) * W
    out = []
    for y in ys:
        for x in xs:
            d = f + r * (x / k) - u * (y / k)
            out.append(d / np.linalg.norm(d))
    return np.array(out)


def _planes() -> list[tuple[int, float, tuple[float, float], tuple[float, float], str]]:
    """Proxy planes in the image frame (axis 0 = x', 1 = y', 2 = z above the stage floor): (axis, value, bounds of the
    first other axis, bounds of the second, 'cg' | 'proj')."""
    x0, x1, y0, y1 = STAGE
    ax0, ax1, ay1 = AUDIENCE
    za, zt, zh = -AUDIENCE_DROP, STAGE_TOP, HALL_TOP
    return [(2, 0.0, (x0, x1), (y0, y1), "cg"),                      # stage floor
            (1, y0, (x0, x1), (za, 0.0), "cg"),                      # stage-front apron
            (1, y1, (x0, x1), (0.0, zt), "proj"), (0, x0, (y0, y1), (0.0, zt), "proj"), (0, x1, (y0, y1), (0.0, zt), "proj"),
            (2, zt, (x0, x1), (y0, y1), "proj"),
            (2, za, (ax0, ax1), (ay1, y0), "proj"), (0, ax0, (ay1, y0), (za, zh), "proj"), (0, ax1, (ay1, y0), (za, zh), "proj"),
            (1, ay1, (ax0, ax1), (za, zh), "proj"), (2, zh, (ax0, ax1), (ay1, y0), "proj"),
            (1, y0, (ax0, x0), (za, zh), "proj"), (1, y0, (x1, ax1), (za, zh), "proj"), (1, y0, (x0, x1), (zt, zh), "proj")]


def proxy_hit(hall: Hall, origin_w: np.ndarray, dir_w: np.ndarray) -> tuple[str, np.ndarray | None]:
    """First proxy surface a world ray meets: ('cg' | 'proj' | 'none', image-frame hit point (x', y', z above the stage
    floor))."""
    o = np.array([*hall.to_img(np.asarray(origin_w, float)[:2]), float(origin_w[2]) - hall.floor_z])
    d = np.array([*(rot2(hall.rot) @ np.asarray(dir_w, float)[:2]), float(dir_w[2])])
    best_t, best = float("inf"), ("none", None)
    for ax, val, b1, b2, kind in _planes():
        if abs(d[ax]) < 1e-9:
            continue
        t = (val - o[ax]) / d[ax]
        if t <= 1e-6 or t >= best_t:
            continue
        p = o + t * d
        oth = [a for a in (0, 1, 2) if a != ax]
        if b1[0] - 1e-6 <= p[oth[0]] <= b1[1] + 1e-6 and b2[0] - 1e-6 <= p[oth[1]] <= b2[1] + 1e-6:
            best_t, best = t, (kind, p)
    return best


def backplate_edges(hall: Hall, emap: np.ndarray | None, cam: np.ndarray, veh_exit: np.ndarray, lens_mm: float,
                    fast: bool = False) -> float:
    """Mean vertical-edge energy of the backplate below the vehicle: each camera ray is traced to the proxy room
    (``proxy_hit``) and the HDRI is sampled in the direction from the CAPTURE POINT to the hit (what the projection
    actually shows there); rays ending on CG surfaces (the stage floor, the apron) count 0."""
    if emap is None:
        return 0.0
    d = backplate_rays(cam, veh_exit, lens_mm, n=8 if fast else 24, cols=3 if fast else 5)
    vals = []
    for di in d:
        kind, p = proxy_hit(hall, cam, di)
        if kind != "proj" or p is None:
            vals.append(0.0)
            continue
        dimg = p - np.array([0.0, 0.0, CAP_H])
        vals.append(float(sample_dirs(emap, dimg[None, :])[0]))
    return float(np.mean(vals))


_LUM_CACHE: dict[str, np.ndarray] = {}


def lum_map(hdr_path: str, cache_dir: str) -> np.ndarray | None:
    """(H, W) luminance of the HDRI (linear, step-4 preview); None without it."""
    if hdr_path in _LUM_CACHE:
        return _LUM_CACHE[hdr_path]
    if not os.path.exists(hdr_path):
        return None
    from .hdr_io import cached_preview
    _LUM_CACHE[hdr_path] = cached_preview(hdr_path, cache_dir, step=4).mean(axis=2)
    return _LUM_CACHE[hdr_path]


GLARE_L = 5.0                 # linear radiance ~40x the hall's median: the sunlit doorway, bare lamps


def backplate_glare(hall: Hall, lmap: np.ndarray | None, cam: np.ndarray, tgt: np.ndarray, lens_mm: float, nx: int = 9, ny: int = 5,
                    W: int = 1920, H: int = 1080) -> float:
    """Fraction of a 9 x 5 ray grid over the frame whose projected backplate is blown out (> GLARE_L): the open
    daylight doorway of the hall turns into a white hole that out-shines the black vehicle."""
    if lmap is None:
        return 0.0
    f = np.asarray(tgt, float) - np.asarray(cam, float)
    f /= np.linalg.norm(f)
    r = np.cross(f, [0.0, 0.0, 1.0])
    r /= max(np.linalg.norm(r), 1e-9)
    u = np.cross(r, f)
    k = lens_mm / 36.0 * W
    hits = 0
    tot = 0
    for y in np.linspace(-0.42, 0.42, ny) * H:
        for x in np.linspace(-0.45, 0.45, nx) * W:
            d = f + r * (x / k) + u * (y / k)
            d /= np.linalg.norm(d)
            kind, p = proxy_hit(hall, cam, d)
            tot += 1
            if kind == "proj" and p is not None:
                hits += int(float(sample_dirs(lmap, (p - np.array([0.0, 0.0, CAP_H]))[None, :])[0]) > GLARE_L)
    return hits / max(tot, 1)


def backplate_pillar(hall: Hall, lmap: np.ndarray | None, cam: np.ndarray, tgt: np.ndarray, veh: np.ndarray,
                     lens_mm: float, W: int = 1920, H: int = 1080) -> float:
    """Bright vertical structure directly behind / below the vehicle (the white pillars between the window blinds,
    the sunlit doorway): the projected backplate's luminance averaged down 8 screen rows (0 - 35 % of the frame
    height below the vehicle) in 25 columns across +-6 % of the frame width; returns the brightest of the 7 central
    columns (+-1.7 %) minus the median of all 25 (linear radiance; 0 = a uniform backdrop).  Measured round 1b: a
    pillar under the vehicle scores 0.5 - 1.8, the doorway ~49, a plain blind ~0."""
    if lmap is None:
        return 0.0
    from . import plan as PL                     # (lazy: render.plan imports this module lazily too)
    q = PL.look_quat(cam, tgt)
    Rc = PL.quat_to_matrix(q)
    px, py, depth = PL.project(np.asarray(veh, float), cam, tgt, 0.0, lens_mm, W, H)
    if not np.isfinite(px):
        return 0.0
    k_ = lens_mm / 36.0 * W
    prof = []
    for xo in np.linspace(-0.06, 0.06, 25) * W:
        col = []
        for yo in np.linspace(0.0, 0.35, 8) * H:
            d = Rc @ np.array([(px + xo - W / 2) / k_, -(py + yo - H / 2) / k_, -1.0])
            d /= np.linalg.norm(d)
            kind, hp = proxy_hit(hall, cam, d)
            if kind == "proj" and hp is not None:
                col.append(float(sample_dirs(lmap, (hp - np.array([0.0, 0.0, CAP_H]))[None, :])[0]))
        prof.append(float(np.mean(col)) if col else float("nan"))
    a = np.array(prof)
    if not np.any(np.isfinite(a[9:16])):
        return 0.0
    return max(0.0, float(np.nanmax(a[9:16]) - np.nanmedian(a)))


def image_azimuth(hall: Hall, view_dir_w: np.ndarray) -> float:
    """Image-frame azimuth (deg) of a world view direction."""
    d = rot2(hall.rot) @ np.asarray(view_dir_w, float)[:2]
    return math.degrees(math.atan2(d[1], d[0]))


def in_sector(a_deg: float, sector: Sequence[float]) -> bool:
    a = (a_deg + 180.0) % 360.0 - 180.0
    return sector[0] <= a <= sector[1]


# --------------------------------------------------------------------------- #
# fitting the room around the demo
# --------------------------------------------------------------------------- #
def fit(veh_xy: np.ndarray, feet_xy: np.ndarray, push_views: Sequence[tuple[np.ndarray, float]], floor_z: float,
        emap: np.ndarray | None = None, rot_step_deg: float = 10.0, lmap: np.ndarray | None = None,
        cam_dist: float = 4.0) -> Hall:
    """Choose the room rotation and the pad's place on the stage.

    ``veh_xy`` (N,2) vehicle positions of the edit, ``feet_xy`` (M,2) operator positions, ``push_views`` a list of
    (vehicle CG world xyz, desired camera azimuth deg) for the hero shots (push / lift-off).  Feasible = vehicle >=
    0.7 m and feet >= 0.4 m inside the stage; score = sum over views of [camera distance after clamping (<= ``cam_dist``;
    round 2: 5.6 m, the distance a level 32 mm two-shot of the operator and a 1.5 m hover needs) +
    1.5 if the camera looks into the auditorium - 2 in the piano sector - 6 x backplate vertical edges - 25 x the
    blown-out fraction of the frame (the sunlit doorway)] - 0.5 |offset|.
    """
    best: Hall | None = None
    offs = [np.array([x, y]) for x in (-0.6, 0.0, 0.6, 1.2) for y in (-0.3, 0.2, 0.7, 1.2)]
    veh = np.asarray(veh_xy, float).reshape(-1, 2)
    feet = np.asarray(feet_xy, float).reshape(-1, 2) if feet_xy is not None and len(feet_xy) else np.zeros((0, 2))
    for rd in np.arange(0.0, 360.0, rot_step_deg):
        rot = math.radians(float(rd))
        for o in offs:
            h = Hall(rot=rot, veh_img=o, floor_z=floor_z)
            mv = float(np.min(stage_margin(h.to_img(veh)))) if veh.size else 1.0
            mf = float(np.min(stage_margin(h.to_img(feet)))) if feet.size else 1.0
            feas = mv >= 0.7 and mf >= 0.4
            sc = 0.0
            for cg, az_deg in push_views:
                a = math.radians(az_deg)
                cam = np.array([cg[0] + cam_dist * math.cos(a), cg[1] + cam_dist * math.sin(a)])
                c2 = clamp_xy(h, cg[:2], cam)
                dist = float(np.hypot(*(c2 - cg[:2])))
                sc += min(dist, cam_dist)
                camz = np.array([c2[0], c2[1], floor_z + 1.45])
                view = np.asarray(cg, float) - camz
                ia = image_azimuth(h, view)
                sc += 1.5 if in_sector(ia, AUDIENCE_SECTOR) else 0.0
                sc -= 2.0 if in_sector(ia, PIANO_SECTOR) else 0.0
                sc -= 6.0 * backplate_edges(h, emap, camz, np.asarray(cg, float) - np.array([0, 0, 0.376]), 32.0, fast=True)
                sc -= 25.0 * backplate_glare(h, lmap, camz, np.asarray(cg, float) - np.array([0, 0, 0.5]), 28.0)
            sc -= 0.5 * float(np.linalg.norm(o))
            # infeasible layouts rank below every feasible one, by how far they miss
            key = (1 if feas else 0, sc if feas else min(mv - 0.7, mf - 0.4))
            if best is None or key > (1 if best.feasible else 0, best.score):
                h.score, h.feasible = (sc if feas else min(mv - 0.7, mf - 0.4)), feas
                best = h
    assert best is not None
    if not best.feasible:
        best.notes.append("indoor: the vehicle path / operator positions do not fit on the stage of the hall HDRI "
                          f"(worst margin {best.score:.2f} m): the vehicle or the operator leaves the stage proxy")
    return best


# --------------------------------------------------------------------------- #
# proxy geometry (consumed by render/blender/env.py)
# --------------------------------------------------------------------------- #
def proxy_quads(hall: Hall) -> dict[str, list[list[tuple[float, float, float]]]]:
    """World-space quads (4 corners each) of the projection proxies, by role: 'stage_walls' (curtains, ceiling),
    'auditorium' (floor 1 m below the stage, side / back / front walls, ceiling), 'stage_front' (the CG apron face)
    and 'stage_floor' (the CG floor rectangle, 0.15 m under the walls)."""
    x0, x1, y0, y1 = STAGE
    ax0, ax1, ay1 = AUDIENCE
    fz = hall.floor_z
    za, zt, zh = fz - AUDIENCE_DROP, fz + STAGE_TOP, fz + HALL_TOP

    def w(x: float, y: float, z: float) -> tuple[float, float, float]:
        p = hall.to_world(np.array([x, y]))
        return (float(p[0]), float(p[1]), float(z))
    out: dict[str, list] = {"stage_walls": [], "auditorium": [], "stage_front": [], "stage_floor": []}
    # curtains: back (y1), wings (x0, x1); inward-facing order does not matter (emission, both sides)
    out["stage_walls"] += [[w(x0, y1, fz), w(x1, y1, fz), w(x1, y1, zt), w(x0, y1, zt)],
                           [w(x0, y0, fz), w(x0, y1, fz), w(x0, y1, zt), w(x0, y0, zt)],
                           [w(x1, y0, fz), w(x1, y1, fz), w(x1, y1, zt), w(x1, y0, zt)],
                           [w(x0, y0, zt), w(x1, y0, zt), w(x1, y1, zt), w(x0, y1, zt)]]
    out["auditorium"] += [[w(ax0, ay1, za), w(ax1, ay1, za), w(ax1, y0, za), w(ax0, y0, za)],       # floor
                          [w(ax0, ay1, za), w(ax0, y0, za), w(ax0, y0, zh), w(ax0, ay1, zh)],       # side walls
                          [w(ax1, ay1, za), w(ax1, y0, za), w(ax1, y0, zh), w(ax1, ay1, zh)],
                          [w(ax0, ay1, za), w(ax1, ay1, za), w(ax1, ay1, zh), w(ax0, ay1, zh)],     # back wall
                          [w(ax0, ay1, zh), w(ax1, ay1, zh), w(ax1, y0, zh), w(ax0, y0, zh)],       # ceiling
                          [w(ax0, y0, za), w(x0, y0, za), w(x0, y0, zh), w(ax0, y0, zh)],           # proscenium wall L
                          [w(x1, y0, za), w(ax1, y0, za), w(ax1, y0, zh), w(x1, y0, zh)],           # proscenium wall R
                          [w(x0, y0, zt), w(x1, y0, zt), w(x1, y0, zh), w(x0, y0, zh)]]             # above the opening
    out["stage_front"] += [[w(x0, y0, za), w(x1, y0, za), w(x1, y0, fz), w(x0, y0, fz)]]
    e = 0.15
    out["stage_floor"] += [[w(x0 - e, y0, fz), w(x1 + e, y0, fz), w(x1 + e, y1 + e, fz), w(x0 - e, y1 + e, fz)]]
    return out
