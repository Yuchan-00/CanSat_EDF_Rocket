"""EDF sound bed (pure numpy): a quiet illustration track synthesised from the frame plan - never a recording.

A silent hover reads fake in a live presentation (art review round 1), so every encoded video can carry a soft ducted
fan sound whose every parameter comes from logged values in the plan:

  rotor speed   rpm = 26 000 sqrt(T / T_ref)            (render.plan: the same rotor model that drives the blur disc)
  roar          band-limited noise (300 Hz - 6 kHz), amplitude ~ (T / T_ref)^0.8
  whine         the 12-blade passing tone rpm / 60 x 12 (+ its 2nd harmonic, 1.5 % jitter), amplitude ~ (T / T_ref)
  rumble        low-passed noise (< 160 Hz), amplitude ~ (T / T_ref)
  distance      every component x clamp(1.6 m / d, 0.12, 1), d = camera -> duct exit (the shot's own perspective);
                the jump at a cut is limited to +-4 dB (shot-mean levels, art review round 2)
  slow motion   inside a slowed stretch (plan 'speed' < 1) the pitch drops by speed^0.5 and the top end is muffled,
                the usual film convention for slowed sound (stated, not physics)
  room tone     a low, dark hall / field ambience (pink-ish noise, -40 dB under the bed's level) under EVERYTHING,
                the title and end cards included: no hard digital silence at the card cuts
  loudness      the mix is normalised to -16 LUFS integrated (ITU-R BS.1770-4: K-weighting, 400 ms blocks, -70 LUFS
                absolute and -10 LU relative gates) with a -1 dBTP true-peak ceiling (4x oversampled)

The EDF bed fades in over 0.4 s after the title card and out over 0.45 s into the end card; the room tone fades in
at the very start and out at the very end.  Deterministic (seeded noise).  Stated on the end card as a synthesised
sound.
"""
from __future__ import annotations

import math
import wave
from typing import Any

import numpy as np

SR = 48000
RPM_MAX = 26000.0
BLADES = 12
TARGET_LUFS = -16.0
TRUE_PEAK_DBTP = -1.0
CUT_JUMP_DB = 4.0
CARD_FADE_S = (0.40, 0.45)        # bed fade-in after the title card, fade-out into the end card
ROOM_TONE_DB = -40.0              # room tone RMS relative to the bed's RMS


def _lp(x: np.ndarray, fc: float, sr: int = SR, order: int = 1) -> np.ndarray:
    """``order`` cascaded one-pole low-passes at ``fc`` Hz: FFT convolution with the truncated exponential impulse
    response a (1 - a)^k (k < 9 / a), numpy only."""
    a = 1.0 - math.exp(-2.0 * math.pi * fc / sr)
    L = int(min(max(9.0 / a, 8), x.size))
    h = a * (1.0 - a) ** np.arange(L)
    y = np.asarray(x, float)
    for _ in range(order):
        m = 1 << int(math.ceil(math.log2(y.size + L)))
        y = np.fft.irfft(np.fft.rfft(y, m) * np.fft.rfft(h, m), m)[:x.size]
    return y


def frame_params(P: dict[str, np.ndarray], t_ref: float) -> dict[str, np.ndarray]:
    """Per video frame: rpm, roar / whine / rumble amplitudes (distance-weighted), slow-motion factor."""
    T = np.clip(np.asarray(P["thrust"], float) / max(float(t_ref), 1e-6), 0.0, 1.5)
    rpm = RPM_MAX * np.sqrt(T)
    pos = np.asarray(P["pos"], float)
    d = np.linalg.norm(np.asarray(P["cam_pos"], float) - pos, axis=1)
    g = np.clip(1.6 / np.maximum(d, 1e-3), 0.12, 1.0)
    if "seg" in P:
        g = limit_cut_jumps(g, np.asarray(P["seg"], int))
    speed = np.clip(np.asarray(P.get("speed", np.ones(T.size)), float), 0.05, 1.0)
    return {"rpm": rpm, "roar": g * T ** 0.8, "whine": g * T, "rumble": g * T, "speed": speed, "gain": g}


def limit_cut_jumps(g: np.ndarray, seg: np.ndarray, max_db: float = CUT_JUMP_DB) -> np.ndarray:
    """Distance gains with the level step at every cut limited to +-``max_db``: each shot's mean level (dB) is pulled
    to within max_db of the previous shot's (already limited) level; the perspective inside a shot is unchanged."""
    g = np.asarray(g, float).copy()
    prev = None
    for i in np.unique(seg):
        m = seg == i
        lv = 20.0 * math.log10(max(float(np.mean(g[m])), 1e-9))
        if prev is not None:
            lim = float(np.clip(lv, prev - max_db, prev + max_db))
            g[m] *= 10.0 ** ((lim - lv) / 20.0)
            lv = lim
        prev = lv
    return g


def k_weight(x: np.ndarray, sr: int = SR) -> np.ndarray:
    """ITU-R BS.1770-4 K-weighting (pre-filter high shelf + RLB high-pass), coefficients for any sample rate."""
    from scipy.signal import lfilter
    f0, G, Q = 1681.974450955533, 3.999843853973347, 0.7071752369554196
    K = math.tan(math.pi * f0 / sr)
    Vh, Vb = 10.0 ** (G / 20.0), (10.0 ** (G / 20.0)) ** 0.4996667741545416
    a0 = 1.0 + K / Q + K * K
    b1 = [(Vh + Vb * K / Q + K * K) / a0, 2.0 * (K * K - Vh) / a0, (Vh - Vb * K / Q + K * K) / a0]
    a1 = [1.0, 2.0 * (K * K - 1.0) / a0, (1.0 - K / Q + K * K) / a0]
    f0, Q = 38.13547087602444, 0.5003270373238773
    K = math.tan(math.pi * f0 / sr)
    a0 = 1.0 + K / Q + K * K
    b2 = [1.0, -2.0, 1.0]
    a2 = [1.0, 2.0 * (K * K - 1.0) / a0, (1.0 - K / Q + K * K) / a0]
    return lfilter(b2, a2, lfilter(b1, a1, x, axis=0), axis=0)


def loudness_lufs(x: np.ndarray, sr: int = SR) -> float:
    """Integrated loudness (LUFS) of a (N,) or (N, C) signal, BS.1770-4 gating."""
    y = k_weight(np.asarray(x, float).reshape(len(x), -1), sr)
    blk, hop = int(0.4 * sr), int(0.1 * sr)
    if y.shape[0] < blk:
        return float("-inf")
    starts = np.arange(0, y.shape[0] - blk + 1, hop)
    ms = np.array([np.sum(np.mean(y[a:a + blk] ** 2, axis=0)) for a in starts])
    lk = -0.691 + 10.0 * np.log10(np.maximum(ms, 1e-20))
    sel = lk > -70.0
    if not np.any(sel):
        return float("-inf")
    rel = -0.691 + 10.0 * math.log10(float(np.mean(ms[sel]))) - 10.0
    sel &= lk > rel
    return -0.691 + 10.0 * math.log10(float(np.mean(ms[sel])))


def true_peak_dbtp(x: np.ndarray) -> float:
    """4x-oversampled peak (dBTP)."""
    from scipy.signal import resample_poly
    y = resample_poly(np.asarray(x, float), 4, 1, axis=0)
    return 20.0 * math.log10(max(float(np.max(np.abs(y))), 1e-12))


def synth(P: dict[str, np.ndarray], t_ref: float, fps: float = 30.0, pre_s: float = 0.0, post_s: float = 0.0,
          seed: int = 11) -> np.ndarray:
    """Stereo float32 (N, 2) in [-1, 1]: ``pre_s`` silence + one video frame of sound per plan row + ``post_s`` tail."""
    fp = frame_params(P, t_ref)
    n = fp["rpm"].size
    spf = SR / fps
    ns = int(round(n * spf))
    # per-sample parameters, linear between frame centres
    xs = (np.arange(ns) + 0.5) / spf - 0.5
    fi = np.arange(n, dtype=float)

    def up(v: np.ndarray) -> np.ndarray:
        return np.interp(xs, fi, v)
    rpm, roar, whine, rumble, speed = (up(fp[k]) for k in ("rpm", "roar", "whine", "rumble", "speed"))
    pitch = np.sqrt(speed)
    rng = np.random.default_rng(seed)
    # roar: white noise, high-passed at 300 Hz (minus its 300 Hz low-pass), low-passed at 6 kHz; in slow motion it
    # cross-fades to a muffled (1.2 kHz) version
    w = rng.standard_normal(ns)
    hp = w - _lp(w, 300.0)
    bright = _lp(hp, 6000.0, order=2)
    dull = _lp(hp, 1200.0, order=2)
    bright /= max(float(np.std(bright)), 1e-9)
    dull /= max(float(np.std(dull)), 1e-9)
    slow = np.clip((1.0 - speed) / 0.75, 0.0, 1.0)
    roar_s = (1.0 - slow) * bright + slow * dull
    # whine: blade passing tone with a slow random wander (0.3 %: speed-controller ripple) + 2nd harmonic
    wd = _lp(rng.standard_normal(ns), 2.0)
    wander = 1.0 + 0.003 * wd / max(float(np.std(wd)), 1e-9)
    f_bp = rpm / 60.0 * BLADES * pitch * wander
    ph = 2.0 * math.pi * np.cumsum(f_bp) / SR
    whine_s = np.sin(ph) + 0.35 * np.sin(2.0 * ph + 0.7)
    # rumble: low-passed noise
    r = _lp(rng.standard_normal(ns), 160.0, order=2)
    r /= max(float(np.std(r)), 1e-9)
    mono = 0.55 * roar * roar_s + 0.08 * whine * whine_s + 0.35 * rumble * r
    # the bed: fades in after the title card, and runs 0.45 s into the end card while fading out
    n_pre, n_post = int(round(pre_s * SR)), int(round(post_s * SR))
    f_in = int(CARD_FADE_S[0] * SR) if pre_s > 0 else int(0.3 * SR)
    f_out = int(CARD_FADE_S[1] * SR)
    env = np.ones(ns)
    if ns > f_in + 1:
        env[:f_in] = np.linspace(0.0, 1.0, f_in)
    tail_n = min(f_out, n_post) if n_post > 0 else 0
    if n_post <= 0 and ns > f_out:
        env[-f_out:] *= np.linspace(1.0, 0.0, f_out)
    mono *= env
    roar2 = np.roll(roar_s, int(0.011 * SR))
    right = mono + 0.08 * roar * (roar2 - roar_s) * env
    st = np.stack([mono, right], axis=1)
    if tail_n > 0:
        # the last frame's sound continues into the end card and fades out (no hard cut to silence)
        last = st[-int(0.2 * SR):] if ns > int(0.2 * SR) else st
        reps = int(math.ceil(tail_n / max(last.shape[0], 1)))
        tail = np.concatenate([last] * reps)[:tail_n] * np.linspace(1.0, 0.0, tail_n)[:, None]
    else:
        tail = np.zeros((0, 2))
    body = np.concatenate([np.zeros((n_pre, 2)), st, tail, np.zeros((max(n_post - tail_n, 0), 2))])
    # room tone under everything: low-passed pink-ish noise, 40 dB under the bed
    rt = _lp(rng.standard_normal((body.shape[0])), 900.0, order=2)
    rt2 = _lp(rng.standard_normal((body.shape[0])), 900.0, order=2)
    bed_rms = float(np.sqrt(np.mean(st ** 2))) if st.size else 1e-3
    room = np.stack([rt, rt2], axis=1)
    room *= bed_rms * 10.0 ** (ROOM_TONE_DB / 20.0) / max(float(np.sqrt(np.mean(room ** 2))), 1e-12)
    rf = int(0.3 * SR)
    if room.shape[0] > 2 * rf:
        room[:rf] *= np.linspace(0.0, 1.0, rf)[:, None]
        room[-int(0.6 * SR):] *= np.linspace(1.0, 0.0, int(0.6 * SR))[:, None]
    out = body + room
    # loudness: -16 LUFS integrated, -1 dBTP ceiling
    L = loudness_lufs(out)
    gain_db = (TARGET_LUFS - L) if np.isfinite(L) else 0.0
    tp = true_peak_dbtp(out) + gain_db
    if tp > TRUE_PEAK_DBTP:
        gain_db -= tp - TRUE_PEAK_DBTP
    out *= 10.0 ** (gain_db / 20.0)
    return np.clip(out, -1.0, 1.0).astype(np.float32)


def write_wav(path: str, x: np.ndarray, sr: int = SR) -> None:
    pcm = (np.clip(x, -1.0, 1.0) * 32767.0).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(pcm.shape[1] if pcm.ndim == 2 else 1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm.tobytes())


def track_for(P: dict[str, Any], t_ref: float, path: str, fps: float = 30.0, pre_s: float = 0.0,
              post_s: float = 0.0) -> dict[str, Any]:
    """Synthesise and write the WAV; returns a sidecar summary."""
    x = synth(P, t_ref, fps, pre_s, post_s)
    write_wav(path, x)
    return {"path": path, "seconds": round(x.shape[0] / SR, 3), "sr": SR,
            "lufs": round(loudness_lufs(x), 2), "true_peak_dbtp": round(true_peak_dbtp(x), 2),
            "model": "synthesised EDF sound bed (render.audio): rpm = 26000 sqrt(T/T_ref), roar ~ (T/T_ref)^0.8, "
                     "12-blade passing tone, distance 1.6 m / d; illustration, not a recording"}
