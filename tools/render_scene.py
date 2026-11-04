"""Realistic scenery for ``tools/sim_video.py --style realistic``: procedural textures, a procedural sky used both as
the visible background and for image-based lighting, PBR materials, a projected sun shadow, a soft layered exhaust jet
with ground effects, an outdoor field or an indoor sports hall, and a jointed 1.7 m mannequin with two-bone arm IK.

Invariants:
  * Nothing is downloaded and nothing is read except the arguments: every texture / sky image is generated from a fixed
    seed (``np.random.default_rng(seed)``), so the same call gives byte-identical pixels.  Generated images are cached as
    PNG files under ``<cache_dir>`` (default ``sim/out/video/_assets``) keyed by a hash of their parameters; a cache
    miss regenerates the same pixels.
  * Scenery never changes a logged number: the jet length, the ground-effect strength and the shadow follow the
    logged thrust, pose and height by the formulas written next to them; flicker / dust are deterministic functions of
    the frame time (no wall clock, no global RNG).
  * Pure parts (textures, sky, ``two_bone_ik``, ``mannequin_joints``, ``ground_effect_strength``, ``shadow_raster``)
    have no VTK dependency and are unit-tested in ``tools/test_sim_video.py``.

Coordinates: world z up, metres; equirectangular images: column u = azimuth / 360 deg measured from +x toward +y,
row 0 = zenith, last row = nadir (checked against VTK's skybox / environment orientation with up = +z, right = +x).
"""
from __future__ import annotations

import hashlib
import math
import os
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

# Bambu Lab PLA Matte BLACK (params.json printer.material / printer.color; user 2026-10-20: every printed part of the
# vehicle in PLA Matte Black - in Bambu's PLA Matte line the black is sold as "Charcoal").  A matte black print is not
# #000000 under daylight: its diffuse albedo is ~4 %.  VTK's PBR uses the colour value as LINEAR albedo (no sRGB decode),
# so the albedo is written directly: 0x0b / 255 = 0.043 (measured 2026-10-20: #232426 rendered mid-grey outdoors, #0b0b0c
# reads as matte black with the form still shaded by the sky light).
PLA_BLACK = "#0b0b0c"
PLA_MATTE = {
    "propulsion_module": (PLA_BLACK, "PLA Matte Black"),
    "intake_module": (PLA_BLACK, "PLA Matte Black"),
    "avionics_module": (PLA_BLACK, "PLA Matte Black"),
    "payload_module": (PLA_BLACK, "PLA Matte Black"),
    "nose_cone": (PLA_BLACK, "PLA Matte Black"),
    "vane": (PLA_BLACK, "PLA Matte Black (vanes)"),
    "launch_pad": (PLA_BLACK, "PLA Matte Black (pad, same filament)"),
    "gps_pod": (PLA_BLACK, "PLA Matte Black (v0.8 GPS antenna pod + lid, same filament)"),
    # v0.9 (2026-10-28): the VTOL top cap, the direct-drive servo covers and the motor-lead groove cover
    "top_cap": (PLA_BLACK, "PLA Matte Black (v0.9 VTOL top cap)"),
    "servo_cover": (PLA_BLACK, "PLA Matte Black (v0.9 direct-drive servo covers)"),
    "wire_cover": (PLA_BLACK, "PLA Matte Black (v0.9 motor-lead groove cover)"),
}
PLA_ROUGHNESS = 0.9
# matte print: the micro-rough surface scatters the specular lobe - an effective F0 of ~1.2 % (IOR 1.25) instead of the
# smooth-dielectric 4 % (IOR 1.5) keeps a matte black part from mirroring the bright sky as mid-grey
PLA_BASE_IOR = 1.25
SKIN, SHIRT, PANTS, SHOES, HAIR = "#c99a7b", "#2f5d8a", "#3a3f47", "#1f1f22", "#3b2a20"
DEFAULT_SUN_EL_DEG = 38.0
# heights of the flat layers above the ground (m): far enough apart for a 24-bit depth buffer at video distances
DECAL_Z = {"slab": 0.004, "ring": 0.008, "line": 0.006, "shadow": 0.012, "ripple": 0.016}


def hex_rgb01(c: str) -> np.ndarray:
    c = c.lstrip("#")
    return np.array([int(c[i:i + 2], 16) for i in (0, 2, 4)], dtype=float) / 255.0


def to_u8(img: np.ndarray) -> np.ndarray:
    return (np.clip(img, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


# --------------------------------------------------------------------------- #
# noise (tileable by construction: spectral synthesis on a periodic grid)
# --------------------------------------------------------------------------- #
def fft_noise(shape: tuple[int, int], beta: float, seed: int, k_cut: float | None = None,
              aniso: tuple[float, float] = (1.0, 1.0)) -> np.ndarray:
    """Zero-mean, unit-std periodic noise with power spectrum ~ k^-beta (optionally Gaussian-cut above ``k_cut``
    cycles/pixel; ``aniso`` stretches the spectrum: (ax, ay) > 1 shortens the features along that axis)."""
    h, w = int(shape[0]), int(shape[1])
    rng = np.random.default_rng(int(seed))
    f = np.fft.rfft2(rng.standard_normal((h, w)))
    ky = np.fft.fftfreq(h)[:, None] * aniso[1]
    kx = np.fft.rfftfreq(w)[None, :] * aniso[0]
    k = np.sqrt(kx * kx + ky * ky)
    k[0, 0] = 1.0
    amp = k ** (-0.5 * beta)
    if k_cut is not None:
        amp = amp * np.exp(-(k / k_cut) ** 2)
    f = f * amp
    f[0, 0] = 0.0
    out = np.fft.irfft2(f, s=(h, w))
    sd = float(out.std())
    return out / (sd if sd > 0 else 1.0)


def smoothstep(e0: float, e1: float, x: np.ndarray) -> np.ndarray:
    t = np.clip((np.asarray(x, dtype=float) - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# --------------------------------------------------------------------------- #
# textures (uint8 RGB, row 0 = +v edge; all tileable)
# --------------------------------------------------------------------------- #
def grass_texture(px: int = 2048, tile_m: float = 16.0, seed: int = 11) -> np.ndarray:
    """A ``tile_m`` square of mown grass: patchy colour (macro / meso noise), dry tufts, blade speckle and
    2 m mowing stripes along x (a scale cue).  Tileable."""
    n = int(px)
    macro = fft_noise((n, n), 3.4, seed)
    meso = fft_noise((n, n), 2.2, seed + 1, k_cut=0.08)
    dry = fft_noise((n, n), 3.0, seed + 2)
    fine = fft_noise((n, n), 0.6, seed + 3, aniso=(1.0, 1.6))
    dark = np.array([0.16, 0.27, 0.09])
    mid = np.array([0.30, 0.44, 0.15])
    straw = np.array([0.52, 0.52, 0.27])
    a = 1.0 / (1.0 + np.exp(-(0.9 * macro + 0.5 * meso)))
    col = dark + (mid - dark) * a[..., None]
    d = smoothstep(1.1, 2.2, dry)[..., None] * 0.55
    col = col * (1.0 - d) + straw * d
    stripes_per_tile = max(1, int(round(tile_m / 2.0)))
    y = (np.arange(n) + 0.5) / n
    st = np.sign(np.sin(2.0 * math.pi * stripes_per_tile * 0.5 * y))       # one light + one dark stripe = 4 m
    st = np.convolve(st, np.ones(9) / 9.0, mode="same")[:, None, None]
    col = col * (1.0 + 0.07 * st) * (1.0 + 0.13 * np.clip(fine, -2.5, 2.5)[..., None] / 2.5)
    return to_u8(col)


def concrete_texture(px: int = 1024, seed: int = 21) -> np.ndarray:
    """A 2 m concrete slab: grey noise, fine aggregate speckle, a few stains, darker worn edge band."""
    n = int(px)
    a = fft_noise((n, n), 2.6, seed)
    s = fft_noise((n, n), 0.2, seed + 1)
    stain = smoothstep(1.4, 2.4, fft_noise((n, n), 3.2, seed + 2))
    base = np.array([0.58, 0.575, 0.55])
    col = base * (1.0 + 0.06 * a + 0.05 * np.clip(s, -2, 2) / 2.0 - 0.10 * stain)[..., None]
    e = (np.arange(n) + 0.5) / n
    edge = np.minimum(np.minimum(e, 1 - e)[:, None], np.minimum(e, 1 - e)[None, :])
    col = col * (1.0 - 0.18 * (1.0 - smoothstep(0.0, 0.012, edge)))[..., None]
    return to_u8(col)


def wood_floor_texture(px: int = 2048, tile_m: float = 4.0, strip_m: float = 0.10, seed: int = 31) -> np.ndarray:
    """A ``tile_m`` square of sports-hall maple strip flooring: strips along x of ``strip_m``, staggered board ends,
    per-board tone, long grain (anisotropic noise), dark seams.  Tileable (strip count and board pattern wrap)."""
    n = int(px)
    rng = np.random.default_rng(seed)
    n_strips = max(1, int(round(tile_m / strip_m)))
    rows = np.arange(n)
    strip = (rows * n_strips) // n                                   # strip index per pixel row
    grain = fft_noise((n, n), 2.0, seed + 1, aniso=(0.04, 1.0))         # long streaks along x
    fine = fft_noise((n, n), 0.8, seed + 2, aniso=(0.25, 1.0))
    base = np.array([0.80, 0.58, 0.34])
    img = np.empty((n, n, 3))
    xs = np.arange(n)
    for i in range(n_strips):
        r0, r1 = (i * n) // n_strips, ((i + 1) * n) // n_strips
        # board ends along the strip (wrapping), lengths 0.6-1.8 m
        cuts = [int(rng.uniform(0, n))]
        pos = 0.0
        while True:
            L = rng.uniform(0.6, 1.8) / tile_m * n
            pos += L
            if pos > n - 0.3 / tile_m * n:
                break
            cuts.append(int((cuts[0] + pos) % n))
        cuts = sorted(set(cuts))
        board = np.searchsorted(cuts, xs, side="right") % len(cuts)
        tones = 1.0 + 0.10 * rng.standard_normal(len(cuts))
        hue = rng.normal(0.0, 0.03, (len(cuts), 3))
        t = tones[board][None, :, None] * (base + hue[board][None, :, :])
        img[r0:r1] = t
        seam_cols = np.array([c for c in cuts], dtype=int)
        img[r0:r1, seam_cols % n] *= 0.55
        img[r0:r1, (seam_cols + 1) % n] *= 0.75
        img[r0, :] *= 0.6                                             # strip seam
    img *= (1.0 + 0.07 * grain + 0.03 * fine)[..., None]
    _ = strip
    return to_u8(img)


def wall_texture(w_px: int = 512, h_px: int = 1280, wall_h_m: float = 10.0, tile_m: float = 4.0, seed: int = 41) -> np.ndarray:
    """One ``tile_m`` wide strip of a sports-hall wall (row 0 = top): blue-grey crash pads to 2 m with 1 m seams, a
    dark rail, painted block wall with 0.4 x 0.2 m joints, a band of high windows from 7.2 to 8.6 m."""
    h, w = int(h_px), int(w_px)
    z = wall_h_m * (1.0 - (np.arange(h) + 0.5) / h)[:, None] * np.ones((1, w))
    x = tile_m * (np.arange(w) + 0.5)[None, :] / w * np.ones((h, 1))
    noise = fft_noise((h, w), 2.4, seed)
    img = np.empty((h, w, 3))
    paint = np.array([0.80, 0.79, 0.75])
    img[:] = paint
    bx = (x + 0.2 * (np.floor(z / 0.2) % 2)) % 0.4
    joint = (np.minimum(bx, 0.4 - bx) < 0.006) | ((z % 0.2) < 0.006)
    img[joint] *= 0.86
    pad = z < 2.0
    img[pad] = np.array([0.22, 0.33, 0.50])
    seam = pad & (np.minimum(x % 1.0, 1.0 - x % 1.0) < 0.008)
    img[seam] *= 0.6
    rail = (z >= 2.0) & (z < 2.12)
    img[rail] = np.array([0.18, 0.18, 0.2])
    win = (z > 7.2) & (z < 8.6)
    frame = win & ((np.minimum(x % 1.3, 1.3 - x % 1.3) < 0.04) | (np.abs(z - 7.9) < 0.02))
    img[win] = np.array([0.80, 0.88, 0.96])
    img[frame] = np.array([0.35, 0.36, 0.38])
    img *= (1.0 + 0.04 * noise)[..., None]
    return to_u8(img)


def ceiling_texture(px: int = 512, tile_m: float = 4.0, seed: int = 51) -> np.ndarray:
    """A ``tile_m`` square of a hall ceiling: grey acoustic deck with corrugation and a steel truss line."""
    n = int(px)
    x = tile_m * (np.arange(n) + 0.5) / n
    corr = 0.5 + 0.5 * np.sin(2.0 * math.pi * x / 0.2)
    img = np.empty((n, n, 3))
    img[:] = np.array([0.42, 0.43, 0.45])
    img *= (0.9 + 0.1 * corr)[None, :, None]
    truss = np.minimum(np.abs(x - 0.5 * tile_m), 9.0) < 0.06
    img[:, truss] = np.array([0.22, 0.23, 0.25])
    img *= (1.0 + 0.03 * fft_noise((n, n), 2.0, seed))[..., None]
    return to_u8(img)


def ring_texture(n_r: int = 8, n_a: int = 64) -> np.ndarray:
    """(unused placeholder kept small) a radial ramp."""
    return np.linspace(0.0, 1.0, n_r)[:, None] * np.ones((1, n_a))


# --------------------------------------------------------------------------- #
# sky (equirectangular) and the matching fog colour
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SkySpec:
    profile: str = "field"                  # 'field' | 'hall'
    sun_az_deg: float = 225.0               # direction TO the sun, from +x toward +y
    sun_el_deg: float = DEFAULT_SUN_EL_DEG
    width: int = 4096
    seed: int = 7

    def key(self) -> str:
        s = f"sky3|{self.profile}|{self.sun_az_deg:.1f}|{self.sun_el_deg:.1f}|{self.width}|{self.seed}"
        return hashlib.sha256(s.encode()).hexdigest()[:16]


ZENITH = np.array([0.25, 0.46, 0.78])
HORIZON = np.array([0.72, 0.81, 0.89])
HAZE = np.array([0.78, 0.83, 0.87])
FAR_GRASS = np.array([0.36, 0.45, 0.25])


def sky_dirs(width: int) -> tuple[np.ndarray, np.ndarray]:
    h = width // 2
    az = (np.arange(width) + 0.5) / width * 2.0 * math.pi
    el = (0.5 - (np.arange(h) + 0.5) / h) * math.pi
    return np.meshgrid(az, el)


def field_fog_color() -> np.ndarray:
    """The colour the far ground fades to = the skybox just below the horizon (so the ground's edge disappears)."""
    return 0.62 * HAZE + 0.38 * FAR_GRASS


def sky_field(spec: SkySpec) -> np.ndarray:
    """Clear sky with a few soft clouds, the sun disk + glow, horizon haze, distant hills and a tree line (both
    hazed), and a hazy far field below the horizon.  uint8 (H = W / 2, W, 3)."""
    W = int(spec.width)
    A, E = sky_dirs(W)
    el_deg = np.degrees(E)
    s = np.clip(E / (0.5 * math.pi), 0.0, 1.0) ** 0.45
    img = HORIZON * (1.0 - s[..., None]) + ZENITH * s[..., None]
    # clouds: a periodic noise texture projected onto a flat cloud layer
    from scipy.ndimage import map_coordinates
    cn = fft_noise((1024, 1024), 3.0, spec.seed + 3)
    dz = np.maximum(np.sin(E), 0.0)
    k = 1.0 / (dz + 0.12)
    px_ = (np.cos(E) * np.cos(A) * k * 220.0) % 1024
    py_ = (np.cos(E) * np.sin(A) * k * 220.0) % 1024
    c = map_coordinates(cn, [py_.ravel(), px_.ravel()], order=1, mode="grid-wrap").reshape(E.shape)   # bilinear
    cloud = smoothstep(0.9, 2.1, c) * smoothstep(1.0, 9.0, el_deg) * 0.75
    sun = np.array([math.cos(math.radians(spec.sun_el_deg)) * math.cos(math.radians(spec.sun_az_deg)),
                    math.cos(math.radians(spec.sun_el_deg)) * math.sin(math.radians(spec.sun_az_deg)),
                    math.sin(math.radians(spec.sun_el_deg))])
    v = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], axis=-1)
    cosg = np.clip(v @ sun, -1.0, 1.0)
    ang = np.arccos(cosg)
    cloud_col = np.array([0.93, 0.94, 0.95]) * (0.86 + 0.14 * np.clip(cosg, 0, 1))[..., None]
    img = img * (1.0 - cloud[..., None]) + cloud_col * cloud[..., None]
    glow = 0.20 * np.exp(-ang / 0.25) + 0.30 * np.exp(-ang / 0.05)
    img = img + glow[..., None] * np.array([1.0, 0.93, 0.80])
    disk = smoothstep(0.0105, 0.0085, ang)
    img = img * (1.0 - disk[..., None]) + disk[..., None] * np.array([1.0, 0.99, 0.95])
    # horizon haze
    hz = np.exp(-np.abs(el_deg) / 2.5)
    img = img * (1.0 - 0.55 * hz[..., None]) + HAZE * (0.55 * hz[..., None])
    # hills and tree line (elevation of the silhouette as a function of azimuth)
    az = A[0]
    rng = np.random.default_rng(spec.seed + 9)
    hills = 1.2 + sum(rng.uniform(0.3, 1.0) * np.sin(f * az + rng.uniform(0, 2 * math.pi)) for f in (2, 3, 5, 7)) * 0.55
    hills = np.clip(hills, 0.3, 3.6)
    tree_n = fft_noise((1, W), 1.2, spec.seed + 10)[0]
    gaps = smoothstep(-0.2, 0.6, fft_noise((1, W), 3.5, spec.seed + 11)[0])
    trees = (0.55 + 0.30 * np.clip(tree_n, -2, 2) / 2 + 0.25) * gaps + 0.08
    dh = 180.0 / (W // 2)                                    # degrees per row: anti-aliased silhouettes
    above = (el_deg >= -0.05)[..., None]
    cov_h = np.clip((hills[None, :] - el_deg) / dh + 0.5, 0.0, 1.0)[..., None] * above
    cov_t = np.clip((trees[None, :] - el_deg) / dh + 0.5, 0.0, 1.0)[..., None] * above
    hill_col = 0.55 * HAZE + 0.45 * np.array([0.40, 0.50, 0.46])
    tree_col = (0.35 * HAZE + 0.65 * np.array([0.17, 0.25, 0.15])) *         (1.0 + 0.10 * np.clip(fft_noise((W // 2, W), 0.5, spec.seed + 12), -2, 2) / 2)[..., None]
    img = img * (1.0 - cov_h) + hill_col * cov_h
    img = img * (1.0 - cov_t) + tree_col * cov_t
    # below the horizon: the far field fading into haze
    below = el_deg < -0.05
    f = np.exp(-np.abs(el_deg) / 6.0)
    ground = FAR_GRASS * (1.0 - 0.62 * f[..., None]) + HAZE * (0.62 * f[..., None])
    img[below] = ground[below]
    return to_u8(img)


def sky_hall(spec: SkySpec) -> np.ndarray:
    """Environment of a sports hall for image-based lighting: bright ceiling light panels in a grey deck above,
    pale walls with blue pads at the horizon, warm wood floor below.  uint8 (H = W / 2, W, 3)."""
    W = int(spec.width)
    A, E = sky_dirs(W)
    el = np.degrees(E)
    img = np.empty(A.shape + (3,))
    img[:] = np.array([0.40, 0.41, 0.43])
    wall = (el > -2.0) & (el < 35.0)
    img[wall] = np.array([0.78, 0.77, 0.73])
    pads = (el > -2.0) & (el < 4.0)
    img[pads] = np.array([0.24, 0.34, 0.50])
    win = (el > 22.0) & (el < 30.0) & ((np.degrees(A) % 30.0) < 22.0)
    img[win] = np.array([0.85, 0.92, 1.0])
    floor = el <= -2.0
    img[floor] = np.array([0.66, 0.48, 0.29])
    lights = (el > 40.0) & ((np.degrees(A) % 45.0) < 14.0) & ((el % 12.0) < 5.0)
    img[lights] = np.array([1.0, 0.98, 0.92])
    return to_u8(img)


def sky_image(spec: SkySpec) -> np.ndarray:
    return sky_hall(spec) if spec.profile == "hall" else sky_field(spec)


# --------------------------------------------------------------------------- #
# cache
# --------------------------------------------------------------------------- #
def cached_image(cache_dir: str | None, name: str, key: str, make: Any) -> tuple[np.ndarray, str | None]:
    """``make()`` -> uint8 image, cached as ``<cache_dir>/<name>_<key>.png`` (None: no cache).  Returns (image, path)."""
    from PIL import Image
    if cache_dir:
        path = os.path.join(cache_dir, f"{name}_{key}.png")
        if os.path.exists(path):
            try:
                return np.asarray(Image.open(path).convert("RGB")), path
            except OSError:
                pass
        img = make()
        try:
            os.makedirs(cache_dir, exist_ok=True)
            tmp = path + ".tmp.png"
            Image.fromarray(img).save(tmp)
            os.replace(tmp, path)
        except OSError:
            path = None
        return img, path
    return make(), None


def texture_key(*parts: Any) -> str:
    return hashlib.sha256("|".join(str(p) for p in parts).encode()).hexdigest()[:16]


# --------------------------------------------------------------------------- #
# two-bone IK and the mannequin (pure)
# --------------------------------------------------------------------------- #
def two_bone_ik(root: np.ndarray, target: np.ndarray, l1: float, l2: float, pole: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(joint, end) of a two-bone chain from ``root`` toward ``target`` with bone lengths l1, l2, bending toward
    ``pole`` (a direction).  The end reaches the target exactly when |target - root| is within [|l1 - l2|, l1 + l2];
    otherwise the chain points at it, straight (too far) or folded (too near).  Bone lengths are always kept."""
    root = np.asarray(root, dtype=float)
    d = np.asarray(target, dtype=float) - root
    D = float(np.linalg.norm(d))
    u = d / D if D > 1e-9 else np.array([0.0, 0.0, -1.0])
    lo, hi = abs(l1 - l2) + 1e-9, l1 + l2 - 1e-9
    Dc = min(max(D, lo), hi)
    x = (l1 * l1 - l2 * l2 + Dc * Dc) / (2.0 * Dc)
    hgt = math.sqrt(max(l1 * l1 - x * x, 0.0))
    p = np.asarray(pole, dtype=float)
    p = p - u * float(p @ u)
    n = float(np.linalg.norm(p))
    if n < 1e-9:
        p = np.cross(u, [0.0, 0.0, 1.0])
        if float(np.linalg.norm(p)) < 1e-9:
            p = np.cross(u, [1.0, 0.0, 0.0])
        n = float(np.linalg.norm(p))
    p = p / n
    joint = root + u * x + p * hgt
    end = root + u * Dc
    return joint, end


# body proportions for a 1.70 m adult (Drillis & Contini segment ratios x H), figure frame: x forward, y left, z up
MANNEQUIN_H = 1.70
UPPER_ARM, FOREARM, HAND = 0.30, 0.25, 0.17
THIGH, SHANK = 0.425, 0.42
SHOULDER_Y, HIP_Y = 0.185, 0.09
SHOULDER_Z, HIP_Z, ANKLE_Z = 1.40, 0.90, 0.075


TORSO_Z = (0.80, 1.49)                  # torso from below the crotch to the base of the neck (standing)
# (fraction of the torso length, half depth rx, half width ry, colour band) - hips, waist, chest, shoulders
TORSO_PROFILE = ((0.00, 0.050, 0.070), (0.05, 0.095, 0.140), (0.15, 0.115, 0.172), (0.30, 0.102, 0.150),
                 (0.42, 0.100, 0.146), (0.58, 0.118, 0.160), (0.75, 0.120, 0.176), (0.86, 0.105, 0.188),
                 (0.94, 0.080, 0.150), (1.00, 0.045, 0.055))
TORSO_PANTS_FRAC = 0.30                 # below this fraction: trousers (belt line), above: shirt


def torso_points(n_theta: int = 28, rings_per_span: int = 3) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Points, triangle faces and a per-point 'is shirt' flag of the lofted torso (local frame: z up the torso from
    0 to its length, x forward); elliptic-superellipse rings through TORSO_PROFILE, closed at both ends."""
    L = TORSO_Z[1] - TORSO_Z[0]
    prof = np.array(TORSO_PROFILE)
    fr = []
    for i in range(len(prof) - 1):
        for j in range(rings_per_span):
            fr.append(prof[i, 0] + (prof[i + 1, 0] - prof[i, 0]) * j / rings_per_span)
    fr.append(1.0)
    fr = np.array(fr)
    rx = np.interp(fr, prof[:, 0], prof[:, 1])
    ry = np.interp(fr, prof[:, 0], prof[:, 2])
    th = np.linspace(0.0, 2.0 * math.pi, n_theta, endpoint=False)
    c, s_ = np.cos(th), np.sin(th)
    e = 0.8                                                     # superellipse exponent < 1: a boxier cross-section
    cx = np.sign(c) * np.abs(c) ** e
    sy = np.sign(s_) * np.abs(s_) ** e
    pts = [[0.0, 0.0, 0.0]]
    for k in range(fr.size):
        for t in range(n_theta):
            pts.append([rx[k] * cx[t], ry[k] * sy[t], fr[k] * L])
    pts.append([0.0, 0.0, L])
    pts = np.array(pts)
    faces = []
    nr = fr.size
    for t in range(n_theta):
        faces.append((0, 1 + (t + 1) % n_theta, 1 + t))
    for k in range(nr - 1):
        for t in range(n_theta):
            a = 1 + k * n_theta + t
            b = 1 + k * n_theta + (t + 1) % n_theta
            faces.append((a, b, b + n_theta))
            faces.append((a, b + n_theta, a + n_theta))
    top = pts.shape[0] - 1
    for t in range(n_theta):
        faces.append((top, 1 + (nr - 1) * n_theta + t, 1 + (nr - 1) * n_theta + (t + 1) % n_theta))
    shirt = pts[:, 2] >= TORSO_PANTS_FRAC * L
    return pts, np.array(faces, dtype=np.int64), shirt


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def mannequin_joints(hands: Sequence[np.ndarray] | None = None, lean_deg: float = 0.0, stride: float = 0.0,
                     sway: float = 0.0) -> dict[str, np.ndarray]:
    """Joint positions in the figure frame (origin between the feet on the ground).  ``hands`` = (right, left) hand
    targets in the figure frame (None: relaxed arms); ``lean_deg`` tilts the upper body forward about the hips;
    ``stride`` puts the left foot forward and the right foot back by +-stride/2 (m); ``sway`` (m) a small lateral
    weight shift.  Arms reach their targets with ``two_bone_ik`` (elbows bend down and out)."""
    J: dict[str, np.ndarray] = {}
    pel = np.array([0.0, sway, HIP_Z + 0.03])
    J["pelvis"] = pel
    for side, s in (("r", -1.0), ("l", 1.0)):
        foot_x = 0.5 * stride * s
        ankle = np.array([foot_x - 0.02, s * 0.10, ANKLE_Z])
        hip = np.array([0.0, s * HIP_Y + sway, HIP_Z])
        knee, ank = two_bone_ik(hip, ankle, THIGH, SHANK, np.array([1.0, 0.0, 0.0]))
        J[f"hip_{side}"], J[f"knee_{side}"], J[f"ankle_{side}"] = hip, knee, ank
        J[f"toe_{side}"] = ank + np.array([0.20, 0.0, -0.035])
        J[f"heel_{side}"] = ank + np.array([-0.06, 0.0, -0.03])
    Rl = _rot_y(math.radians(lean_deg))                         # positive lean: chest moves +x (forward)

    def up(p: Sequence[float]) -> np.ndarray:
        return pel + Rl @ (np.asarray(p, dtype=float) - pel)
    J["torso_base"] = np.array([0.0, sway, TORSO_Z[0]])
    J["waist"] = up([0.0, sway, 1.06])
    J["chest"] = up([0.0, sway, 1.27])
    J["torso_top"] = up([0.0, sway, TORSO_Z[1]])
    J["neck"] = up([0.0, sway, 1.45])
    J["head"] = up([0.01, sway, 1.585])
    J["head_top"] = up([0.01, sway, MANNEQUIN_H])
    for side, s in (("r", -1.0), ("l", 1.0)):
        sh = up([0.0, s * SHOULDER_Y + sway, SHOULDER_Z])
        J[f"shoulder_{side}"] = sh
        if hands is None:
            tgt = sh + np.array([0.06, s * 0.05, -(UPPER_ARM + FOREARM) * 0.97])
            pole = np.array([-0.3, s * 0.4, 0.0])
        else:
            tgt = np.asarray(hands[0 if side == "r" else 1], dtype=float)
            pole = np.array([-0.2, s * 0.9, -1.0])
        el, wr = two_bone_ik(sh, tgt, UPPER_ARM, FOREARM, pole)
        J[f"elbow_{side}"], J[f"wrist_{side}"] = el, wr
        fwd = wr - el
        J[f"hand_{side}"] = wr + fwd / max(float(np.linalg.norm(fwd)), 1e-9) * HAND
    return J


# segments: (name, joint a, joint b, rx, ry, taper (end radius / start radius), colour, twist reference)
SEGMENTS: tuple[tuple[str, str, str, float, float, float, str, str], ...] = (
    ("torso", "torso_base", "torso_top", 0.0, 0.0, 1.0, SHIRT, "x"),        # lofted, see TORSO_PROFILE
    ("neck", "neck", "head", 0.048, 0.048, 1.0, SKIN, "x"),
    ("head", "head", "head_top", 0.098, 0.078, 0.85, SKIN, "x"),
    ("upper_arm_r", "shoulder_r", "elbow_r", 0.050, 0.050, 0.85, SHIRT, "x"),
    ("upper_arm_l", "shoulder_l", "elbow_l", 0.050, 0.050, 0.85, SHIRT, "x"),
    ("forearm_r", "elbow_r", "wrist_r", 0.040, 0.040, 0.78, SKIN, "x"),
    ("forearm_l", "elbow_l", "wrist_l", 0.040, 0.040, 0.78, SKIN, "x"),
    ("hand_r", "wrist_r", "hand_r", 0.022, 0.040, 0.9, SKIN, "z"),
    ("hand_l", "wrist_l", "hand_l", 0.022, 0.040, 0.9, SKIN, "z"),
    ("thigh_r", "hip_r", "knee_r", 0.080, 0.080, 0.72, PANTS, "x"),
    ("thigh_l", "hip_l", "knee_l", 0.080, 0.080, 0.72, PANTS, "x"),
    ("shank_r", "knee_r", "ankle_r", 0.056, 0.056, 0.70, PANTS, "x"),
    ("shank_l", "knee_l", "ankle_l", 0.056, 0.056, 0.70, PANTS, "x"),
    ("foot_r", "heel_r", "toe_r", 0.045, 0.050, 0.85, SHOES, "z"),
    ("foot_l", "heel_l", "toe_l", 0.045, 0.050, 0.85, SHOES, "z"),
)


def segment_matrix(a: np.ndarray, b: np.ndarray, ref: str = "x") -> np.ndarray:
    """4x4 carrying a segment built along +z from 0 to |b - a| onto a -> b (x axis toward the figure's +x / +z)."""
    a = np.asarray(a, dtype=float)
    z = np.asarray(b, dtype=float) - a
    L = float(np.linalg.norm(z))
    z = z / L if L > 1e-9 else np.array([0.0, 0.0, 1.0])
    r = np.array([1.0, 0.0, 0.0]) if ref == "x" else np.array([0.0, 0.0, 1.0])
    x = r - z * float(r @ z)
    if float(np.linalg.norm(x)) < 1e-6:
        x = np.array([0.0, 0.0, 1.0]) - z * z[2]
        if float(np.linalg.norm(x)) < 1e-6:
            x = np.array([1.0, 0.0, 0.0]) - z * z[0]
    x = x / float(np.linalg.norm(x))
    y = np.cross(z, x)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, a
    return M


def capsule_points(L: float, rx: float, ry: float, taper: float, res: int = 20) -> tuple[np.ndarray, np.ndarray]:
    """Points + triangle faces of a tapered elliptic capsule along +z from 0 to L (pure numpy)."""
    nt, nphi = int(res), max(6, int(res) // 2 * 2)
    th = np.linspace(0.0, 2.0 * math.pi, nt, endpoint=False)
    ph = np.linspace(0.0, math.pi, nphi + 1)                          # 0 = top pole
    re = min(rx, ry, 0.5 * L)
    pts = []
    for p in ph:
        cz, sz = math.cos(p), math.sin(p)
        zc = (L - re) if cz >= 0 else re
        z = zc + re * cz
        f = 1.0 + (taper - 1.0) * (z / max(L, 1e-9))
        for t in th:
            pts.append((rx * f * sz * math.cos(t), ry * f * sz * math.sin(t), z))
    pts = np.array(pts)
    faces = []
    for i in range(nphi):
        for j in range(nt):
            a, b = i * nt + j, i * nt + (j + 1) % nt
            c, d = (i + 1) * nt + (j + 1) % nt, (i + 1) * nt + j
            faces.append((a, d, c))
            faces.append((a, c, b))
    return pts, np.array(faces, dtype=np.int64)


def figure_matrix(feet: np.ndarray, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    M = np.eye(4)
    M[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    M[:3, 3] = np.asarray(feet, dtype=float)
    return M


def segment_world_matrices(J: dict[str, np.ndarray], feet: np.ndarray, yaw: float) -> dict[str, np.ndarray]:
    Mf = figure_matrix(feet, yaw)
    return {name: Mf @ segment_matrix(J[a], J[b], ref) for name, a, b, _, _, _, _, ref in SEGMENTS}


# --------------------------------------------------------------------------- #
# ground effect, shadows (pure)
# --------------------------------------------------------------------------- #
def ground_effect_strength(thrust_frac: float, h_exit_m: float, h_scale_m: float = 2.2) -> float:
    """0..1 strength of the jet's effect on the ground: thrust fraction x (1 - h / h_scale)^1.5 (an impinging
    round jet from a 90 mm duct is felt to ~20-25 D); 0 above h_scale or without thrust."""
    if thrust_frac <= 0.0 or h_exit_m >= h_scale_m:
        return 0.0
    return float(min(1.0, thrust_frac) * (1.0 - max(h_exit_m, 0.0) / h_scale_m) ** 1.5)


def project_to_plane(pts: np.ndarray, light_dir: np.ndarray, z0: float) -> np.ndarray:
    """Points moved along ``light_dir`` (the direction light travels, d_z < 0) onto the plane z = z0."""
    d = np.asarray(light_dir, dtype=float)
    p = np.asarray(pts, dtype=float)
    s = (p[..., 2] - z0) / d[2]
    return p - s[..., None] * d


def shadow_raster(tris_xy: np.ndarray, x0: float, y0: float, texel: float, n: int, blur_px: float,
                  blobs: Sequence[tuple[float, float, float, float]] = ()) -> np.ndarray:
    """(n, n) float mask in [0, 1] of the union of 2-D triangles (m, 3, 2) on a grid with origin (x0, y0) (row 0 =
    the +y edge), blurred by ``blur_px``; ``blobs`` = soft contact blobs (x, y, radius, strength)."""
    from PIL import Image, ImageDraw, ImageFilter
    im = Image.new("L", (n, n), 0)
    d = ImageDraw.Draw(im)
    if len(tris_xy):
        P = np.empty(tris_xy.shape)
        P[..., 0] = (tris_xy[..., 0] - x0) / texel
        P[..., 1] = (n - 1) - (tris_xy[..., 1] - y0) / texel
        for t in P:
            d.polygon([(float(t[0, 0]), float(t[0, 1])), (float(t[1, 0]), float(t[1, 1])), (float(t[2, 0]), float(t[2, 1]))],
                      fill=255)
    if blur_px > 0.3:
        im = im.filter(ImageFilter.GaussianBlur(radius=float(blur_px)))
    m = np.asarray(im, dtype=np.float32) / 255.0
    if blobs:
        yy = (n - 1 - np.arange(n))[:, None] * texel + y0
        xx = np.arange(n)[None, :] * texel + x0
        for bx, by, br, bs in blobs:
            g = np.exp(-((xx - bx) ** 2 + (yy - by) ** 2) / (2.0 * (br * 0.5) ** 2)) * bs
            m = np.maximum(m, g.astype(np.float32))
    return m


# --------------------------------------------------------------------------- #
# VTK / PyVista parts
# --------------------------------------------------------------------------- #
def make_texture(img: np.ndarray, srgb: bool = True, repeat: bool = True) -> Any:
    import pyvista as pv
    tex = pv.Texture(np.ascontiguousarray(img))
    tex.repeat = repeat
    tex.mipmap = True
    tex.interpolate = True
    if srgb:
        tex.UseSRGBColorSpaceOn()
    if hasattr(tex, "SetMaximumAnisotropicFiltering"):
        tex.SetMaximumAnisotropicFiltering(16)
    return tex


def pbr_mesh(pl: Any, mesh: Any, color: str | None = None, roughness: float = PLA_ROUGHNESS, metallic: float = 0.0,
             texture: Any = None, smooth: bool = True, **kw: Any) -> Any:
    a = pl.add_mesh(mesh, color=color or "white", pbr=True, metallic=metallic, roughness=roughness,
                    smooth_shading=smooth, **kw)
    if texture is not None:
        a.prop.SetBaseColorTexture(texture)
    return a


def add_fog(actor: Any, color: np.ndarray, start_m: float, scale_m: float, fmax: float) -> None:
    """Aerial perspective on one actor: mix toward ``color`` with 1 - exp(-(d - start) / scale), at most fmax
    (d = the fragment's distance from the camera).  A fragment-shader replacement after VTK's lighting."""
    sp = actor.GetShaderProperty()
    c = ", ".join(f"{float(v):.5f}" for v in color)
    code = ("//VTK::Light::Impl\n"
            "  {\n"
            f"    float fogD = max(length(vertexVC.xyz) - {start_m:.3f}, 0.0);\n"
            f"    float fogF = min(1.0 - exp(-fogD / {scale_m:.3f}), {fmax:.3f});\n"
            f"    gl_FragData[0].rgb = mix(gl_FragData[0].rgb, vec3({c}), fogF);\n"
            "  }\n")
    sp.AddFragmentShaderReplacement("//VTK::Light::Impl", True, code, False)


class DynamicTexture:
    """An RGBA vtkTexture whose pixels are replaced in place each frame."""

    def __init__(self, n: int):
        import pyvista as pv
        import vtk
        self.n = int(n)
        self.img = pv.ImageData(dimensions=(self.n, self.n, 1))
        self.buf = np.zeros((self.n * self.n, 4), dtype=np.uint8)
        self.img.point_data.set_array(self.buf, "rgba")
        self.img.point_data.active_scalars_name = "rgba"
        self.tex = vtk.vtkTexture()
        self.tex.SetInputData(self.img)
        self.tex.InterpolateOn()
        self.tex.RepeatOff()
        self.tex.EdgeClampOn()

    def set(self, rgba: np.ndarray) -> None:
        """rgba (n, n, 4) uint8, row 0 = the +v edge."""
        arr = self.img.point_data["rgba"]
        arr[:] = np.ascontiguousarray(rgba[::-1]).reshape(-1, 4)
        self.img.point_data["rgba"] = arr
        self.img.Modified()


class ShadowDecal:
    """A ground quad textured with the blurred union of an object's triangles projected along the light."""

    def __init__(self, pl: Any, z: float, color: Sequence[float] = (0.05, 0.07, 0.10), strength: float = 0.55,
                 n: int = 256):
        import pyvista as pv
        self.n = int(n)
        self.z = float(z)
        self.color = np.asarray(color, dtype=float)
        self.strength = float(strength)
        self.dt = DynamicTexture(self.n)
        pts = np.array([[0, 0, z], [1, 0, z], [1, 1, z], [0, 1, z]], dtype=float)
        self.quad = pv.PolyData(pts, faces=np.array([4, 0, 1, 2, 3]))
        self.quad.active_texture_coordinates = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=float)
        self.actor = pl.add_mesh(self.quad, lighting=False, show_scalar_bar=False)
        self.actor.texture = self.dt.tex
        self.actor.prop.color = (1.0, 1.0, 1.0)

    def update(self, tris_world: np.ndarray, light_dir: np.ndarray, blur_m: float,
               blobs: Sequence[tuple[float, float, float, float]] = (), visible: bool = True) -> None:
        if not visible or (len(tris_world) == 0 and not blobs):
            self.actor.visibility = False
            return
        d = np.asarray(light_dir, dtype=float)
        tris = tris_world
        if len(tris):
            nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
            tris = tris[(nrm @ -d) > 0.0]                                    # faces toward the light only
        P = project_to_plane(tris, d, self.z)[..., :2] if len(tris) else np.zeros((0, 3, 2))
        pts = P.reshape(-1, 2)
        for bx, by, br, _ in blobs:
            pts = np.vstack([pts, [[bx - br, by - br], [bx + br, by + br]]])
        if pts.shape[0] == 0:
            self.actor.visibility = False
            return
        margin = 3.0 * blur_m + 0.05
        lo = pts.min(axis=0) - margin
        hi = pts.max(axis=0) + margin
        span = float(max(hi[0] - lo[0], hi[1] - lo[1]))
        texel = span / (self.n - 1)
        c = 0.5 * (lo + hi)
        x0, y0 = c[0] - 0.5 * span, c[1] - 0.5 * span
        m = shadow_raster(P, x0, y0, texel, self.n, blur_m / texel, blobs)
        rgba = np.empty((self.n, self.n, 4), dtype=np.uint8)
        rgba[..., :3] = (self.color * 255.0).astype(np.uint8)
        rgba[..., 3] = np.clip(m * self.strength * 255.0, 0, 255).astype(np.uint8)
        self.dt.set(rgba)
        x1, y1 = x0 + span, y0 + span
        self.quad.points = np.array([[x0, y0, self.z], [x1, y0, self.z], [x1, y1, self.z], [x0, y1, self.z]])
        self.actor.visibility = True


def mesh_triangles(mesh: Any, target_tris: int | None = None) -> np.ndarray:
    """(m, 3, 3) triangle corner points of a PyVista mesh (decimated to about ``target_tris`` when larger)."""
    m = mesh.triangulate()
    if target_tris and m.n_cells > target_tris:
        try:
            m = m.decimate(1.0 - target_tris / m.n_cells)
        except Exception:                                        # noqa: BLE001 - decimation is an optimisation only
            pass
    faces = np.asarray(m.faces).reshape(-1, 4)[:, 1:]
    return np.asarray(m.points, dtype=float)[faces]


class Mannequin:
    """The jointed 1.7 m figure: one PBR actor per segment, posed per frame from ``mannequin_joints``.  Since
    2026-10-21 only the fallback of ``tools/sim_video.py`` when the rigged human (``tools/human_figure.py``,
    ``render/human/human_lbs.npz``) cannot be loaded."""

    def __init__(self, pl: Any, res: int = 20):
        import pyvista as pv
        self.actors: dict[str, Any] = {}
        self.local_tris: dict[str, np.ndarray] = {}
        self.J = mannequin_joints()
        for name, a, b, rx, ry, taper, col, _ in SEGMENTS:
            L = float(np.linalg.norm(self.J[b] - self.J[a]))
            if name == "head":
                L = 0.235
            if name == "torso":
                pts, faces, shirt = torso_points()
                mesh = pv.PolyData(pts, faces=np.hstack([np.full((faces.shape[0], 1), 3), faces]).ravel())
                rgb = np.where(shirt[:, None], hex_rgb01(SHIRT)[None, :], hex_rgb01(PANTS)[None, :])
                mesh.point_data["rgb"] = (rgb * 255).astype(np.uint8)
                mesh = mesh.compute_normals(auto_orient_normals=True, split_vertices=False)
                self.actors[name] = pl.add_mesh(mesh, scalars="rgb", rgb=True, pbr=True, metallic=0.0, roughness=0.85,
                                                smooth_shading=True, show_scalar_bar=False)
                lp, lf, _ = torso_points(12, 1)
                self.local_tris[name] = lp[lf]
                continue
            pts, faces = capsule_points(L, rx, ry, taper, res)
            mesh = pv.PolyData(pts, faces=np.hstack([np.full((faces.shape[0], 1), 3), faces]).ravel())
            mesh = mesh.compute_normals(auto_orient_normals=True, split_vertices=False)
            rough = 0.55 if col == SKIN else (0.6 if col == SHOES else 0.85)
            self.actors[name] = pl.add_mesh(mesh, color=col, pbr=True, metallic=0.0, roughness=rough, smooth_shading=True)
            lp, lf = capsule_points(L, rx, ry, taper, 8)
            self.local_tris[name] = lp[lf]
        self.mats: dict[str, np.ndarray] = {}

    def pose(self, feet: np.ndarray, yaw: float, hands_world: Sequence[np.ndarray] | None, lean_deg: float,
             stride: float, opacity: float = 1.0, visible: bool = True) -> dict[str, np.ndarray]:
        Mf = figure_matrix(feet, yaw)
        hands = None
        if hands_world is not None:
            inv = np.linalg.inv(Mf)
            hands = [inv[:3, :3] @ np.asarray(h, dtype=float) + inv[:3, 3] for h in hands_world]
        J = mannequin_joints(hands, lean_deg, stride)
        if hands is None:
            J["head_top"] = J["head"] + (J["head_top"] - J["head"])
        mats = segment_world_matrices(J, feet, yaw)
        for name, M in mats.items():
            a = self.actors[name]
            a.user_matrix = M
            a.prop.opacity = float(opacity)
            a.visibility = bool(visible)
        self.mats = mats
        self.joints_world = {k: Mf[:3, :3] @ v + Mf[:3, 3] for k, v in J.items()}
        return self.joints_world

    def triangles_world(self) -> np.ndarray:
        out = [self.local_tris[n] @ M[:3, :3].T + M[:3, 3] for n, M in self.mats.items()]
        return np.concatenate(out, axis=0) if out else np.zeros((0, 3, 3))


class Jet:
    """The EDF exhaust as nested translucent cones (alpha falling along the jet and from the core outward, a slight
    deterministic flicker) - nearly invisible in daylight, like a real cold-air jet; its effects are on the ground."""

    LAYERS = ((1.00, 0.10, 0.035), (0.80, 0.075, 0.05), (0.55, 0.05, 0.075))   # (r0 / duct r, spread, peak alpha)

    def __init__(self, pl: Any, duct_r: float, nu: int = 40, nv: int = 18, tint: Sequence[float] = (0.93, 0.95, 0.97)):
        import pyvista as pv
        self.nu, self.nv = int(nu), int(nv)
        self.duct_r = float(duct_r)
        self.tint = np.asarray(tint, dtype=float)
        self.th = np.linspace(0.0, 2.0 * math.pi, self.nu, endpoint=False)
        self.s = np.linspace(0.0, 1.0, self.nv)
        faces = []
        for i in range(self.nv - 1):
            for j in range(self.nu):
                a, b = i * self.nu + j, i * self.nu + (j + 1) % self.nu
                faces += [4, a, b, b + self.nu, a + self.nu]
        self.meshes, self.actors = [], []
        for _ in self.LAYERS:
            m = pv.PolyData(np.zeros((self.nu * self.nv, 3)), faces=np.array(faces))
            m.point_data["rgba"] = np.zeros((self.nu * self.nv, 4), dtype=np.uint8)
            a = pl.add_mesh(m, scalars="rgba", rgba=True, lighting=False, show_scalar_bar=False, smooth_shading=True)
            self.meshes.append(m)
            self.actors.append(a)

    def update(self, M: np.ndarray, L: float, t: float, visible: bool) -> None:
        if not visible or L < 0.02:
            for a in self.actors:
                a.visibility = False
            return
        S, TH = np.meshgrid(self.s, self.th, indexing="ij")
        for (r0f, spread, a0), m, act in zip(self.LAYERS, self.meshes, self.actors):
            r = self.duct_r * r0f + spread * S * L
            # a gently wavering column: flicker travels down the jet, a few streaks around it
            fl = 1.0 + 0.22 * np.sin(2.0 * math.pi * (6.0 * t - 2.2 * S * L)) * np.cos(3.0 * TH + 1.7 * t) \
                + 0.12 * np.sin(5.0 * TH - 4.0 * t + 9.0 * S)
            alpha = a0 * (1.0 - S) ** 1.6 * smoothstep(0.0, 0.06, S) * fl
            pts = np.stack([r * np.cos(TH), r * np.sin(TH), -0.003 - S * L], axis=-1).reshape(-1, 3)
            m.points = pts
            rgba = np.empty((pts.shape[0], 4), dtype=np.uint8)
            rgba[:, :3] = (self.tint * 255).astype(np.uint8)
            rgba[:, 3] = np.clip(alpha.reshape(-1) * 255.0, 0, 255).astype(np.uint8)
            m.point_data["rgba"] = rgba
            act.user_matrix = M
            act.visibility = True


class GroundEffect:
    """Where the jet meets the ground: a ripple disc (outdoor: grass flattened by the wall jet, a lighter sheen with
    radial streaks; indoor: a faint pressure ring) and flying particles (grass bits / dust), all scaled by
    ``ground_effect_strength``; positions and phases come from a fixed-seed RNG and the frame time."""

    def __init__(self, pl: Any, z: float, indoor: bool, n_particles: int = 260, seed: int = 1234):
        import pyvista as pv
        self.z = float(z)
        self.indoor = bool(indoor)
        rng = np.random.default_rng(seed)
        self.np_ = int(n_particles if not indoor else n_particles // 3)
        self.ang = rng.uniform(0.0, 2.0 * math.pi, self.np_)
        self.rate = rng.uniform(0.6, 1.4, self.np_)
        self.phase = rng.uniform(0.0, 1.0, self.np_)
        self.hop = rng.uniform(0.2, 1.0, self.np_)
        self.spd = rng.uniform(0.6, 1.0, self.np_)
        if indoor:
            cols = np.array([[0.80, 0.80, 0.78], [0.70, 0.68, 0.64]])
        else:
            cols = np.array([[0.60, 0.57, 0.50], [0.52, 0.50, 0.46], [0.55, 0.47, 0.36], [0.36, 0.46, 0.20]])
        self.pcol = (cols[rng.integers(0, cols.shape[0], self.np_)] * 255).astype(np.uint8)
        self.pts = pv.PolyData(np.zeros((self.np_, 3)))
        self.pts.point_data["rgba"] = np.zeros((self.np_, 4), dtype=np.uint8)
        self.p_actor = pl.add_mesh(self.pts, scalars="rgba", rgba=True, render_points_as_spheres=True, point_size=4.0,
                                   lighting=False, show_scalar_bar=False)
        nr, na = 14, 72
        self.nr, self.na = nr, na
        r = np.linspace(0.0, 1.0, nr)
        a = np.linspace(0.0, 2.0 * math.pi, na, endpoint=False)
        self.R, self.A = np.meshgrid(r, a, indexing="ij")
        faces = []
        for i in range(nr - 1):
            for j in range(na):
                p, q = i * na + j, i * na + (j + 1) % na
                faces += [4, p, q, q + na, p + na]
        self.disc = pv.PolyData(np.zeros((nr * na, 3)), faces=np.array(faces))
        self.disc.point_data["rgba"] = np.zeros((nr * na, 4), dtype=np.uint8)
        self.d_actor = pl.add_mesh(self.disc, scalars="rgba", rgba=True, lighting=False, show_scalar_bar=False)

    def update(self, center_xy: np.ndarray, strength: float, t: float, px_per_m: float) -> None:
        if strength <= 0.01:
            self.p_actor.visibility = False
            self.d_actor.visibility = False
            return
        I = float(strength)
        Rmax = 0.35 + 1.4 * math.sqrt(I)
        rr = self.R * Rmax
        x = center_xy[0] + rr * np.cos(self.A)
        y = center_xy[1] + rr * np.sin(self.A)
        self.disc.points = np.stack([x, y, np.full(x.shape, self.z + DECAL_Z["ripple"])], axis=-1).reshape(-1, 3)
        s = self.R
        if self.indoor:
            ring = 0.5 + 0.5 * np.sin(2.0 * math.pi * (s * Rmax / 0.35 - 1.6 * t))
            alpha = 0.16 * I * ring * smoothstep(0.05, 0.25, s) * (1.0 - smoothstep(0.6, 1.0, s))
            col = np.array([0.92, 0.94, 0.97])
        else:
            streak = 0.6 + 0.4 * np.cos(11.0 * self.A + 0.8 * np.sin(1.3 * t + 3.0 * s))
            ripple = 0.65 + 0.35 * np.sin(2.0 * math.pi * (s * Rmax / 0.22 - 2.4 * t))
            alpha = 0.32 * I * streak * ripple * smoothstep(0.03, 0.2, s) * (1.0 - smoothstep(0.55, 1.0, s))
            col = np.array([0.74, 0.82, 0.55])
        rgba = np.empty((self.nr * self.na, 4), dtype=np.uint8)
        rgba[:, :3] = (col * 255).astype(np.uint8)
        rgba[:, 3] = np.clip(alpha.reshape(-1) * 255.0, 0, 255).astype(np.uint8)
        self.disc.point_data["rgba"] = rgba
        self.d_actor.visibility = True
        age = (t * self.rate * (0.8 + 0.8 * I) + self.phase) % 1.0
        r = 0.08 + age * Rmax * 1.25 * self.spd
        h = (0.04 + 0.30 * self.hop * I) * np.sin(math.pi * age) * (0.5 if self.indoor else 1.0)
        P = np.column_stack([center_xy[0] + r * np.cos(self.ang + 0.3 * age), center_xy[1] + r * np.sin(self.ang + 0.3 * age),
                             self.z + 0.01 + h])
        self.pts.points = P
        rgba = np.empty((self.np_, 4), dtype=np.uint8)
        rgba[:, :3] = self.pcol
        a = I * smoothstep(0.0, 0.12, age) * (1.0 - smoothstep(0.6, 1.0, age)) * (0.55 if self.indoor else 0.95)
        rgba[:, 3] = np.clip(a * 255.0, 0, 255).astype(np.uint8)
        self.pts.point_data["rgba"] = rgba
        self.p_actor.prop.point_size = float(np.clip(0.012 * px_per_m, 1.5, 7.0))
        self.p_actor.visibility = True


def ring_ribbon(r: float, width: float, z: float, n: int = 240) -> Any:
    """A flat annulus (painted line) of radius r and the given width."""
    import pyvista as pv
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = np.vstack([np.column_stack([(r - 0.5 * width) * np.cos(a), (r - 0.5 * width) * np.sin(a), np.full(n, z)]),
                     np.column_stack([(r + 0.5 * width) * np.cos(a), (r + 0.5 * width) * np.sin(a), np.full(n, z)])])
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [4, i, j, n + j, n + i]
    return pv.PolyData(pts, faces=np.array(faces))


def rect_ribbon(x0: float, y0: float, x1: float, y1: float, width: float, z: float) -> Any:
    """Four painted lines along the edges of a rectangle (court boundary)."""
    import pyvista as pv
    w = 0.5 * width
    out = None
    for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)):
        lo = (min(ax, bx) - w, min(ay, by) - w)
        hi = (max(ax, bx) + w, max(ay, by) + w)
        b = pv.Plane(center=(0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), z), direction=(0, 0, 1),
                     i_size=hi[0] - lo[0], j_size=hi[1] - lo[1], i_resolution=1, j_resolution=1)
        out = b if out is None else out + b
    return out


@dataclass
class WorldInfo:
    profile: str
    sun_dir: np.ndarray        # unit vector toward the sun (field) / the key light (hall)
    light_dir: np.ndarray      # direction the light travels (shadow projection)
    shadow_blur_base_m: float
    shadow_blur_per_m: float
    shadow_strength: float
    sky_path: str | None
    fog: bool


class World:
    """Static scenery + lights + IBL for one profile ('field' or 'hall')."""

    def __init__(self, pl: Any, profile: str, ground_z: float, half_extent_m: float, sun_az_deg: float,
                 cache_dir: str | None, warnings: list[str], ring_radii: Sequence[float] = (), lang: str = "ko"):
        import pyvista as pv
        self.pl = pl
        self.profile = "hall" if profile == "hall" else "field"
        gz = float(ground_z)
        self.gz = gz
        ren = pl.renderer
        ren.SetEnvironmentUp(0.0, 0.0, 1.0)
        ren.SetEnvironmentRight(1.0, 0.0, 0.0)
        spec = SkySpec(self.profile, float(sun_az_deg) % 360.0, DEFAULT_SUN_EL_DEG if self.profile == "field" else 75.0,
                       4096 if self.profile == "field" else 1024)
        sky, path = cached_image(cache_dir, f"sky_{self.profile}", spec.key(), lambda: sky_image(spec))
        stex = make_texture(sky, srgb=True, repeat=False)
        pl.set_environment_texture(stex, is_srgb=True, resample=0.25, show_background=False)
        self.skybox = None
        if self.profile == "field":
            self.skybox = stex.to_skybox(floor_plane=(0, 0, 1, 0), floor_right=(1, 0, 0))
            pl.add_actor(self.skybox)
        else:
            pl.set_background("#3a3b3e")
        sa, se = math.radians(spec.sun_az_deg), math.radians(spec.sun_el_deg)
        sun_dir = np.array([math.cos(se) * math.cos(sa), math.cos(se) * math.sin(sa), math.sin(se)])
        light = pv.Light(light_type="scene light")
        light.positional = False
        light.position = tuple(sun_dir * 100.0)
        light.focal_point = (0.0, 0.0, 0.0)
        if self.profile == "field":
            light.color = (1.0, 0.95, 0.86)
            light.intensity = 2.6
        else:
            light.color = (1.0, 0.98, 0.94)
            light.intensity = 1.5
        pl.add_light(light)
        if self.profile == "hall":
            fill = pv.Light(light_type="scene light")
            fill.positional = False
            fill.position = (-sun_dir[0] * 50, -sun_dir[1] * 50, 60.0)
            fill.focal_point = (0.0, 0.0, 0.0)
            fill.intensity = 0.8
            pl.add_light(fill)
        self.info = WorldInfo(self.profile, sun_dir, -sun_dir,
                              0.006 if self.profile == "field" else 0.02,
                              0.0093 if self.profile == "field" else 0.03,
                              0.55 if self.profile == "field" else 0.7, path, self.profile == "field")
        self.static_tris: list[np.ndarray] = []
        if self.profile == "field":
            self._field(gz, half_extent_m, cache_dir, ring_radii)
        else:
            self._hall(gz, cache_dir, ring_radii)

    # ---- outdoor ------------------------------------------------------------------------
    def _field(self, gz: float, half: float, cache_dir: str | None, ring_radii: Sequence[float]) -> None:
        import pyvista as pv
        pl = self.pl
        tile = 16.0
        img, _ = cached_image(cache_dir, "grass", texture_key("grass1", 2048, tile, 11), lambda: grass_texture(2048, tile, 11))
        H = max(1200.0, 4.0 * float(half))
        # tessellated: two 2.4 km triangles clipped at the near plane lose the depth precision the decals need
        plane = pv.Plane(center=(0, 0, gz), direction=(0, 0, 1), i_size=2 * H, j_size=2 * H, i_resolution=96, j_resolution=96)
        plane.active_texture_coordinates = np.asarray(plane.active_texture_coordinates, dtype=float) * (2.0 * H / tile)
        self.ground = pbr_mesh(pl, plane, roughness=0.95, texture=make_texture(img), smooth=False)
        add_fog(self.ground, field_fog_color(), 25.0, 520.0, 0.93)
        self.ground.UseBoundsOff()
        cimg, _ = cached_image(cache_dir, "concrete", texture_key("concrete1", 1024, 21), lambda: concrete_texture(1024, 21))
        slab = pv.Plane(center=(0, 0, gz + DECAL_Z["slab"]), direction=(0, 0, 1), i_size=2.0, j_size=2.0, i_resolution=1, j_resolution=1)
        pbr_mesh(pl, slab, roughness=0.9, texture=make_texture(cimg, repeat=False), smooth=False)
        for r in ring_radii:
            if r <= 1.05:
                continue                                    # the slab is 2 m: the 1 m ring would cross it
            if r <= 3.5:                                    # chalk lines near the pad (usually near the camera)
                a = pl.add_mesh(ring_ribbon(r, 0.04, gz + DECAL_Z["ring"]), color="#f4f4ee", opacity=0.45, pbr=True,
                                metallic=0.0, roughness=0.9)
                add_fog(a, field_fog_color(), 25.0, 520.0, 0.93)
            else:                                           # farther rings: 1.5 px lines (a thin ribbon aliases)
                a_ = np.linspace(0.0, 2.0 * math.pi, max(180, int(8 * r)) + 1)
                pts = np.column_stack([r * np.cos(a_), r * np.sin(a_), np.full(a_.size, gz + DECAL_Z["ring"])])
                pl.add_mesh(pv.lines_from_points(pts), color="#eef0e8", line_width=1.5, opacity=0.5, lighting=False)

    # ---- indoor ---------------------------------------------------------------------------
    HALL = (-22.0, 22.0, -14.0, 14.0, 10.0)        # x0, x1, y0, y1, height (m) around the pad

    def _hall(self, gz: float, cache_dir: str | None, ring_radii: Sequence[float]) -> None:
        import pyvista as pv
        pl = self.pl
        x0, x1, y0, y1, hh = self.HALL
        tile = 4.0
        fimg, _ = cached_image(cache_dir, "woodfloor", texture_key("wood1", 2048, tile, 0.10, 31),
                               lambda: wood_floor_texture(2048, tile, 0.10, 31))
        floor = pv.Plane(center=(0.5 * (x0 + x1), 0.5 * (y0 + y1), gz), direction=(0, 0, 1), i_size=x1 - x0, j_size=y1 - y0,
                         i_resolution=44, j_resolution=28)
        tc = np.asarray(floor.active_texture_coordinates, dtype=float)
        floor.active_texture_coordinates = tc * np.array([(x1 - x0) / tile, (y1 - y0) / tile])
        self.ground = pbr_mesh(pl, floor, roughness=0.32, texture=make_texture(fimg), smooth=False)
        # court lines: a 28 x 15 m court around the pad, the centre line and the 1.8 m centre circle
        lines = rect_ribbon(-14.0, -7.5, 14.0, 7.5, 0.05, gz + DECAL_Z["line"]) + pv.Plane(
            center=(0.0, 0.0, gz + DECAL_Z["line"]), direction=(0, 0, 1), i_size=0.05, j_size=15.0, i_resolution=1, j_resolution=1)
        pl.add_mesh(lines, color="#f4f1e8", pbr=True, roughness=0.4, metallic=0.0)
        pl.add_mesh(ring_ribbon(1.8, 0.05, gz + DECAL_Z["line"]), color="#f4f1e8", pbr=True, roughness=0.4, metallic=0.0)
        pl.add_mesh(rect_ribbon(-17.0, -10.0, 17.0, 10.0, 0.05, gz + DECAL_Z["line"]), color="#2d6fbf", pbr=True, roughness=0.4)
        for r in ring_radii:
            if r in (1.0, 2.0, 3.0):                       # tape marks (the court lines carry the larger scale)
                pl.add_mesh(ring_ribbon(r, 0.03, gz + DECAL_Z["ring"]), color="#d0551f", opacity=0.75, pbr=True,
                            metallic=0.0, roughness=0.5)
        wimg, _ = cached_image(cache_dir, "hallwall", texture_key("wall1", 512, 1280, hh, tile, 41),
                               lambda: wall_texture(512, 1280, hh, tile, 41))
        wtex = make_texture(wimg)
        walls = []
        for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)):
            L = math.hypot(bx - ax, by - ay)
            # wound so the front face looks into the hall (seen from outside the wall is culled)
            pts = np.array([[bx, by, gz], [ax, ay, gz], [ax, ay, gz + hh], [bx, by, gz + hh]], dtype=float)
            w = pv.PolyData(pts, faces=np.array([4, 0, 1, 2, 3]))
            w.active_texture_coordinates = np.array([[0, 0], [L / tile, 0], [L / tile, 1], [0, 1]], dtype=float)
            a = pbr_mesh(pl, w, roughness=0.85, texture=wtex, smooth=False)
            a.prop.backface_culling = True
            walls.append(a)
        cimg, _ = cached_image(cache_dir, "hallceiling", texture_key("ceil1", 512, tile, 51), lambda: ceiling_texture(512, tile, 51))
        ceil = pv.PolyData(np.array([[x0, y0, gz + hh], [x0, y1, gz + hh], [x1, y1, gz + hh], [x1, y0, gz + hh]], dtype=float),
                           faces=np.array([4, 0, 1, 2, 3]))
        ceil.active_texture_coordinates = np.array([[0, 0], [0, (y1 - y0) / tile], [(x1 - x0) / tile, (y1 - y0) / tile],
                                                    [(x1 - x0) / tile, 0]], dtype=float)
        ca = pbr_mesh(pl, ceil, roughness=0.9, texture=make_texture(cimg), smooth=False)
        ca.prop.backface_culling = True
        panels = None
        for ix in np.arange(-15.0, 15.1, 7.5):
            for iy in (-6.0, 0.0, 6.0):
                p = pv.Plane(center=(ix, iy, gz + hh - 0.05), direction=(0, 0, -1), i_size=2.4, j_size=0.9,
                             i_resolution=1, j_resolution=1)
                panels = p if panels is None else panels + p
        pl.add_mesh(panels, color="#fffaf0", lighting=False)
        for a in walls + [ca, self.ground]:
            a.UseBoundsOff()
