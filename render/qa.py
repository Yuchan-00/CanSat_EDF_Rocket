"""Frame QA (pure numpy + PIL, no Blender): does the PLA Matte Black body actually read black?

USER DECISION 1 makes every printed part matte black (linear albedo ~0.04).  Round 1 of the art review measured the
'black' body at sRGB 77-113 in some shots (a camera fill and a hot studio key lit it beige).  ``shell_luminance``
projects the vehicle's printed shell (the CadQuery STLs, posed exactly as in the plan) through the plan's camera,
rasterises it into a mask, erodes it (edges / motion blur / DOF) and returns the median and 90th-percentile sRGB luma
of the raw rendered frame inside it.  The driver warns when the median exceeds ``SHELL_MAX_SRGB`` (the art
director's limit: a 4 % albedo body under a neutral grade stays below ~60 sRGB; target 35-55).  Occluders in front
of the body (the pole, the person) are not removed from the mask: the 3 px erosion and the median absorb small ones.

Round 2 gates (the median alone passed a brown body indoors and a silver sheen in the studio):
  chroma     |median R - median B| of the shell pixels <= ``SHELL_MAX_CHROMA`` (6 sRGB): matte black is neutral
  highlights fewer than ``SHELL_MAX_HI_FRAC`` (10 %) of the shell pixels above ``SHELL_HI_SRGB`` (100 sRGB): rims are
             thin lines, not a sheen over a third of the body
  shadow     (indoors, ``shadow_contrast``) while the vehicle hovers below 1.6 m its shadow from the stage key spot
             is >= 15 % darker than the floor around it (it reads as hovering, not pasted in)

Invariant: read-only (STLs, the plan arrays, the PNG); deterministic.
"""
from __future__ import annotations

import os
import struct
from typing import Any

import numpy as np

SHELL_PARTS = ("propulsion_module", "intake_module", "avionics_module", "payload_module", "nose_cone")
SHELL_MAX_SRGB = 60.0
SHELL_MAX_CHROMA = 6.0
SHELL_HI_SRGB = 100.0
SHELL_MAX_HI_FRAC = 0.10
SHADOW_MIN_DARKEN = 0.15
SENSOR_W_MM = 36.0
_STL_CACHE: dict[str, np.ndarray] = {}


def read_stl_tris(path: str, scale: float = 1e-3) -> np.ndarray:
    """Binary STL -> (F, 3, 3) triangle vertices in metres."""
    if path in _STL_CACHE:
        return _STL_CACHE[path]
    with open(path, "rb") as f:
        raw = f.read()
    n = struct.unpack("<I", raw[80:84])[0]
    a = np.frombuffer(raw[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    tris = a["v"].astype(np.float64) * scale
    _STL_CACHE[path] = tris
    return tris


OUTER_R_MIN = 0.052        # m: triangles closer to the axis are the duct / bore walls (seen through the exit)


def shell_tris(root: str) -> np.ndarray:
    """The printed shell without the propulsion module's duct / bore walls (centroid < OUTER_R_MIN from the axis;
    round 1b: they projected into the exit opening and the bright vanes behind it inflated the median); back faces
    are culled in ``shell_mask``."""
    d = os.path.join(root, "cad", "exports", "cadquery", "stl")
    parts = [read_stl_tris(os.path.join(d, f"{p}.stl")) for p in SHELL_PARTS if os.path.exists(os.path.join(d, f"{p}.stl"))]
    if not parts:
        return np.zeros((0, 3, 3))
    out = []
    for name, t in zip([p for p in SHELL_PARTS if os.path.exists(os.path.join(d, f"{p}.stl"))], parts):
        if name == "propulsion_module":                # the duct / bore: inside r 45-50 mm
            c = t.mean(axis=1)
            t = t[np.hypot(c[:, 0], c[:, 1]) >= OUTER_R_MIN]
        out.append(t)
    return np.concatenate(out, axis=0)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(q, float)
    return np.array([[w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z]])


def shell_mask(tris: np.ndarray, pos: np.ndarray, q: np.ndarray, cam_pos: np.ndarray, cam_quat: np.ndarray,
               lens_mm: float, W: int, H: int, scale: float = 0.5, shift: Any = (0.0, 0.0)) -> np.ndarray:
    """Boolean (H*scale, W*scale) mask of the projected shell (Blender camera: looks along local -Z, +Y up,
    horizontal sensor fit 36 mm)."""
    from PIL import Image, ImageDraw
    Rv = quat_to_matrix(q)
    P = tris.reshape(-1, 3) @ Rv.T + np.asarray(pos, float)
    Rc = quat_to_matrix(cam_quat)
    C = (P - np.asarray(cam_pos, float)) @ Rc                      # camera coords (rows: x, y, z)
    depth = -C[:, 2]
    w, h = int(W * scale), int(H * scale)
    k = lens_mm / SENSOR_W_MM * w
    sx, sy = float(shift[0]), float(shift[1])
    with np.errstate(divide="ignore", invalid="ignore"):
        x = w / 2 + k * C[:, 0] / depth - sx * w
        y = h / 2 - k * C[:, 1] / depth + sy * w
    x, y, depth = x.reshape(-1, 3), y.reshape(-1, 3), depth.reshape(-1, 3)
    ok = np.all(depth > 0.02, axis=1)
    # front faces only (STL winding: counter-clockwise seen from outside): the visible outer skin
    Pt = P.reshape(-1, 3, 3)
    nrm = np.cross(Pt[:, 1] - Pt[:, 0], Pt[:, 2] - Pt[:, 0])
    ok &= np.einsum("ij,ij->i", nrm, np.asarray(cam_pos, float)[None, :] - Pt.mean(axis=1)) > 0
    img = Image.new("1", (w, h), 0)
    d = ImageDraw.Draw(img)
    for tx, ty in zip(x[ok], y[ok]):
        if tx.max() < 0 or ty.max() < 0 or tx.min() > w or ty.min() > h:
            continue
        d.polygon([(tx[0], ty[0]), (tx[1], ty[1]), (tx[2], ty[2])], fill=1)
    # seen from below the exit plane, the open duct (EDF, vanes, bulkhead behind them) is not the shell: cut the
    # exit disc (r 47 mm) out of the mask
    cam_b = Rv.T @ (np.asarray(cam_pos, float) - np.asarray(pos, float))
    if cam_b[2] < 0.0:
        a = np.linspace(0.0, 2 * np.pi, 48, endpoint=False)
        disc = np.column_stack([0.047 * np.cos(a), 0.047 * np.sin(a), np.zeros(a.size)]) @ Rv.T + np.asarray(pos, float)
        Cd = (disc - np.asarray(cam_pos, float)) @ Rc
        dd = -Cd[:, 2]
        if np.all(dd > 0.02):
            d.polygon(list(zip((w / 2 + k * Cd[:, 0] / dd - sx * w).tolist(),
                               (h / 2 - k * Cd[:, 1] / dd + sy * w).tolist())), fill=0)
    return np.asarray(img, dtype=bool)


def shell_luminance(png: str, root: str, plan: dict[str, np.ndarray], frame: int, erode_px: int = 3) -> dict[str, Any]:
    """{'median', 'p90', 'pixels'} sRGB luma (0-255) of the shell in Blender frame ``frame`` (1-based)."""
    from PIL import Image, ImageFilter
    k = frame - 1
    with Image.open(png) as im:
        W, H = im.size
        small = im.convert("RGB").resize((W // 2, H // 2), Image.BILINEAR)
    sh = plan["cam_shift"][k] if "cam_shift" in plan else (0.0, 0.0)
    m = shell_mask(shell_tris(root), plan["pos"][k], plan["q"][k], plan["cam_pos"][k], plan["cam_quat"][k],
                   float(plan["cam_lens"][k]), W, H, 0.5, sh)
    mi = Image.fromarray((m * 255).astype(np.uint8)).filter(ImageFilter.MinFilter(2 * erode_px + 1))
    m = np.asarray(mi) > 127
    if int(m.sum()) < 30:
        return {"median": float("nan"), "p90": float("nan"), "pixels": int(m.sum()), "chroma": float("nan"),
                "hi_frac": float("nan"), "rgb_median": [float("nan")] * 3}
    a = np.asarray(small, dtype=np.float32)
    luma = 0.2126 * a[..., 0] + 0.7152 * a[..., 1] + 0.0722 * a[..., 2]
    v = luma[m]
    rgb = [float(np.median(a[..., i][m])) for i in range(3)]
    return {"median": round(float(np.median(v)), 1), "p90": round(float(np.percentile(v, 90)), 1), "pixels": int(m.sum()),
            "rgb_median": [round(x, 1) for x in rgb], "chroma": round(abs(rgb[0] - rgb[2]), 1),
            "hi_frac": round(float(np.mean(v > SHELL_HI_SRGB)), 3)}


def gate_warnings(fr: int, r: dict[str, Any]) -> list[str]:
    """The round-2 gates on one frame's ``shell_luminance`` result."""
    out = []
    if np.isfinite(r.get("median", float("nan"))) and r["median"] > SHELL_MAX_SRGB:
        out.append(f"frame {fr}: matte-black shell median {r['median']:.0f} sRGB > {SHELL_MAX_SRGB:.0f}")
    if np.isfinite(r.get("chroma", float("nan"))) and r["chroma"] > SHELL_MAX_CHROMA:
        out.append(f"frame {fr}: shell chroma |R-B| {r['chroma']:.0f} sRGB > {SHELL_MAX_CHROMA:.0f} (not neutral black)")
    if np.isfinite(r.get("hi_frac", float("nan"))) and r["hi_frac"] > SHELL_MAX_HI_FRAC:
        out.append(f"frame {fr}: {100 * r['hi_frac']:.0f} % of the shell above {SHELL_HI_SRGB:.0f} sRGB "
                   f"(> {100 * SHELL_MAX_HI_FRAC:.0f} %: a sheen, not rim lines)")
    return out


def shadow_contrast(png: str, plan: dict[str, np.ndarray], frame: int, light_pos: Any, floor_z: float) -> dict[str, Any]:
    """Indoor: luminance of the vehicle's shadow on the floor vs the floor around it.  The shadow centre is where the
    ray light -> CG meets the floor; the 'in' disc has the body radius (0.06 m), the ring 0.25-0.40 m around it (both
    projected through the plan camera).  Returns {'darken': 1 - in / ring} or {} when not applicable (vehicle above 1.6 m,
    the shadow outside the frame)."""
    from PIL import Image
    k = frame - 1
    pos = np.asarray(plan["pos"][k], float)
    if float(pos[2]) > 1.6 or light_pos is None:              # altitude above the pad top (the log z)
        return {}
    Rv = quat_to_matrix(plan["q"][k])
    cg = pos + Rv[:, 2] * 0.376
    L = np.asarray(light_pos, float)
    d = cg - L
    if d[2] >= -1e-6:
        return {}
    s_ = (floor_z - L[2]) / d[2]
    c = L + d * s_
    with Image.open(png) as im:
        W, H = im.size
        a = np.asarray(im.convert("L"), dtype=np.float32)
    Rc = quat_to_matrix(plan["cam_quat"][k])
    sh = plan["cam_shift"][k] if "cam_shift" in plan else (0.0, 0.0)
    kpx = float(plan["cam_lens"][k]) / SENSOR_W_MM * W

    def px(P: np.ndarray) -> tuple[float, float, float]:
        C = (np.asarray(P, float) - plan["cam_pos"][k]) @ Rc
        dep = -C[2]
        return W / 2 + kpx * C[0] / dep - sh[0] * W, H / 2 - kpx * C[1] / dep + sh[1] * W, dep
    ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    vals_in, vals_ring = [], []
    for r, store in ((0.0, vals_in), (0.03, vals_in), (0.06, vals_in), (0.25, vals_ring), (0.32, vals_ring), (0.40, vals_ring)):
        for t in (ang if r > 0 else [0.0]):
            x, y, dep = px(c + np.array([r * np.cos(t), r * np.sin(t), 0.0]))
            if dep > 0 and 0 <= x < W and 0 <= y < H:
                store.append(float(a[int(y), int(x)]))
    if len(vals_in) < 5 or len(vals_ring) < 20:
        return {}
    i_, r_ = float(np.median(vals_in)), float(np.median(vals_ring))
    return {"in": round(i_, 1), "ring": round(r_, 1), "darken": round(1.0 - i_ / max(r_, 1e-6), 3),
            "centre": [round(float(x), 3) for x in c]}


# --------------------------------------------------------------------------- #
# props vs the frame
# --------------------------------------------------------------------------- #
def _qmat(q: Any) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def frame_safe(base_xyz: Any, radius: float, height: float, cams: dict | None,
               side: float = 0.05, bottom: float = 0.85) -> bool:
    """True when a prop (a cylinder ``radius`` x ``height`` standing at ``base_xyz``) is, in EVERY camera frame of
    ``cams`` ({'pos','quat','lens','shift' arrays, 'aspect' H/W}), either fully outside the frame or fully inside the
    safe area (``side`` of the width from the edges, above ``bottom`` of the height: the lower HUD row) - never cut by a
    frame edge or sitting under the HUD (art review round 2: a cone half cut at the edge, cones under the HUD panel)."""
    if cams is None:
        return True
    a = np.linspace(0.0, 2 * np.pi, 8, endpoint=False)
    b = np.asarray(base_xyz, float)
    pts = np.concatenate([np.column_stack([b[0] + radius * np.cos(a), b[1] + radius * np.sin(a), np.full(8, b[2] + z)])
                          for z in (0.0, height)])
    asp = float(cams.get("aspect", 9.0 / 16.0))
    for cp, cq, lens, sh in zip(cams["pos"], cams["quat"], cams["lens"], cams["shift"]):
        Rm = _qmat(cq)
        c = (pts - np.asarray(cp, float)) @ Rm                       # camera coords: x right, y up, -z forward
        dep = -c[:, 2]
        if np.all(dep <= 0.05):
            continue
        if np.any(dep <= 0.05):
            return False                                             # straddles the camera plane: too close
        k = float(lens) / 36.0
        x = 0.5 + k * c[:, 0] / dep - float(sh[0])                   # fractions of the width / height
        y = 0.5 - (k * c[:, 1] / dep) / asp + float(sh[1]) / asp
        out = (x.max() < 0.0) or (x.min() > 1.0) or (y.max() < 0.0) or (y.min() > 1.0)
        if out:
            continue
        safe = x.min() >= side and x.max() <= 1.0 - side and y.min() >= 0.02 and y.max() <= bottom
        if not safe:
            return False
    return True
