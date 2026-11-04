"""Minimal Radiance .hdr (RGBE) reader in numpy, for planning-time analysis of the Poly Haven HDRIs (no Blender).

``read_hdr(path, step)`` -> float32 (H/step, W/step, 3) linear RGB, every ``step``-th row / column (the decoder
still parses every scanline: new-style RLE has no random access).  Supports the flat and the new run-length
encoded scanline formats of the Radiance spec (the format Poly Haven ships); old-style RLE is rejected.

Invariant: pure function of the file bytes; nothing is cached on disk except by the explicit callers.
"""
from __future__ import annotations

import os

import numpy as np


def _header(buf: bytes) -> tuple[int, int, int, bool]:
    """(offset of pixel data, width, height, y_down) from the text header."""
    pos = 0
    while True:
        e = buf.index(b"\n", pos)
        line = buf[pos:e].strip()
        pos = e + 1
        if not line:
            break
        if line.startswith(b"FORMAT=") and b"32-bit_rle_rgbe" not in line:
            raise ValueError("only 32-bit_rle_rgbe is supported")
    e = buf.index(b"\n", pos)
    res = buf[pos:e].decode("ascii").split()
    pos = e + 1
    if len(res) != 4 or res[0] not in ("-Y", "+Y") or res[2] != "+X":
        raise ValueError(f"unsupported resolution line {res}")
    return pos, int(res[3]), int(res[1]), res[0] == "-Y"


def read_hdr(path: str, step: int = 1) -> np.ndarray:
    with open(path, "rb") as f:
        buf = f.read()
    pos, w, h, ydown = _header(buf)
    data = np.frombuffer(buf, dtype=np.uint8, offset=pos)
    rows = list(range(0, h, step))
    out = np.zeros((len(rows), (w + step - 1) // step, 3), np.float32)
    want = {r: i for i, r in enumerate(rows)}
    p = 0
    line = np.empty((4, w), np.uint8)
    for y in range(h):
        if w >= 8 and w < 32768 and data[p] == 2 and data[p + 1] == 2 and (int(data[p + 2]) << 8 | int(data[p + 3])) == w:
            p += 4
            for c in range(4):
                x = 0
                while x < w:
                    n = int(data[p])
                    p += 1
                    if n > 128:
                        n -= 128
                        line[c, x:x + n] = data[p]
                        p += 1
                    else:
                        line[c, x:x + n] = data[p:p + n]
                        p += n
                    x += n
            px = line
        else:
            px = data[p:p + 4 * w].reshape(w, 4).T
            p += 4 * w
        if y in want:
            q = px[:, ::step].astype(np.float32)
            e = q[3]
            f = np.where(e > 0, np.ldexp(1.0, (e - 136).astype(np.int32)), 0.0).astype(np.float32)
            out[want[y]] = (q[:3] * f[None, :]).T
    return out if ydown else out[::-1]


def equirect_dir(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Unit direction (Blender world, before the world mapping rotation) of equirect image coords u in [0,1)
    (left -> right) and v in [0,1] (top -> bottom): azimuth = (0.5 - u) 2 pi, elevation = (0.5 - v) pi."""
    az = (0.5 - np.asarray(u, float)) * 2 * np.pi
    el = (0.5 - np.asarray(v, float)) * np.pi
    return np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=-1)


def dir_to_uv(d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Inverse of ``equirect_dir`` (u in [0, 1), v in [0, 1], v = 0 at the zenith)."""
    d = np.asarray(d, float)
    d = d / np.maximum(np.linalg.norm(d, axis=-1, keepdims=True), 1e-12)
    az = np.arctan2(d[..., 1], d[..., 0])
    el = np.arcsin(np.clip(d[..., 2], -1, 1))
    u = (0.5 - az / (2 * np.pi)) % 1.0
    v = 0.5 - el / np.pi
    return u, v


def cached_preview(path: str, cache_dir: str, step: int = 8) -> np.ndarray:
    """``read_hdr(path, step)`` cached as .npy in ``cache_dir`` (decoding a 4k HDRI takes a few seconds)."""
    os.makedirs(cache_dir, exist_ok=True)
    cp = os.path.join(cache_dir, os.path.splitext(os.path.basename(path))[0] + f"_s{step}.npy")
    if os.path.exists(cp) and os.path.getmtime(cp) >= os.path.getmtime(path):
        return np.load(cp)
    img = read_hdr(path, step)
    np.save(cp, img)
    return img
