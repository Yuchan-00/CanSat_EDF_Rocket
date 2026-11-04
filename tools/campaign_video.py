"""Campaign dispersion animation: a VTOL campaign's runs in one MP4 (1920 x 1080, 30 fps).

Input: a campaign directory written by ``sim/vtol_campaign.py`` holding ``tracks.npz`` (or the file itself).
Output: an H.264 MP4, a poster PNG (the last body frame) and a JSON sidecar (frames, render speed, the counts and
statistics shown, warnings).  Like ``tools/sim_video.py`` it is a viewer: every number on screen is read from the
tracks file or computed from it by the formula written next to it; it never runs the simulation.

Tracks contract (``tracks.npz``; R runs, T samples of a common time grid; keys after the colon are optional):

  t [T] float64 s              common time grid, strictly increasing (10 Hz recommended), covering the longest run
  pos [R,T,3] float32 world m  exit-plane centre (the sim_video ``pos``); after a run ends: its last value or NaN
  alive [R,T] bool             True while run r is still being simulated at t[j]
  phase [R,T] uint8            VTOL phase (NONE 0 SPOOL 1 CLIMB 2 HOLD 3 LAND 4 LANDED 5)
  state [R,T] uint8            LP_STATE (SAFE 0 ARMED 1 POWERED 2 UNPOWERED 3 RELEASED 4 ABORT 5)
  outcome [R] uint8            index into outcome_names (the run's final verdict class)
  outcome_names [K] str        identifiers; recognised: pass (landed, every criterion met), landed_fail (landed, a
                               criterion failed), abort (ABORT / release), not_landed (no LANDED confirmation, e.g.
                               V6 cutoff), other.  Colours and Korean labels follow the identifier.
  meta () JSON str             title, title_ko, campaign_seed, n, estimator, profile ('field' | 'demo'), params_sha256,
                               criteria {touchdown_vz_max_mps, touchdown_vh_max_mps, touchdown_tilt_max_deg, ...},
                               criteria_text, model_notes, notes_ko, sampling_ASSUMED {name: [lo, hi]}, date, written_by;
                               v0.9 (optional): hardware_config / power_source (sim_video.hardware_of) -> a title-card
                               line naming the vehicle (absent: the card as before)
  : outcome_labels_ko [K] str  Korean labels (default: by identifier)
    failed [R] str             failed criteria per run, ';'-separated ('' = none)
    wind_mps [R], wind_dir_deg [R]  mean wind speed and direction (toward, deg from +x toward +y) per run
    wind_swing_deg [R], wind_swing_period_s [R]  amplitude / period of a swinging wind direction (0 = steady)
    wind_xy [R,T,2] float32    the mean wind vector over time (animates the wind rose; needed for swinging winds)
    touchdown_t [R], touchdown_vz [R] (signed, < 0 down), touchdown_vh [R], touchdown_tilt_deg [R],
    touchdown_distance_m [R], drift_max_m [R]   (NaN where a run has no touchdown)
    seed [R] int64, run_index [R] int32
    v0.8 (GPS position hold): hold_xy [R,2] float  each run's hold point (world x, y; NaN = the run never set one)
    touchdown_vh_ground [R] (horizontal touchdown speed over the ground), sk_rms_m [R] / sk_max_m [R] (station
    keeping in HOLD: the true CG's distance from the hold point), touchdown_from_hold_m [R]; meta.criteria may carry
    station_keeping_max_m (a number or {"field": .., "demo": ..}) and touchdown_ground_speed_mps

Hold-point-relative view (``--origin hold``; ``auto`` = hold when hold_xy is present): every run's track is drawn
relative to its own hold point (x - hold_x, y - hold_y: all hold points on the map centre), so the station keeping of
all runs overlays; a run without a hold point stays relative to the pad (counted on screen); the station-keeping
criterion is a red dashed ring; the closing card shows the station-keeping and touchdown-from-hold-point histograms.

Picture: left, the top view of all R tracks growing over time (colour = final outcome; a dot per run: filled while
flying, a ring once landed, a cross once aborted), distance rings around the pad (square-root radial scale when a few
runs drift far beyond the rest, said on screen); right, the height of every run against time, the counters (waiting /
flying / landed / aborted / ended) with their history, the wind rose (each run's mean wind; with wind_xy, its current
wind and the last 10 s of its tip) and the outcome legend with final counts.  A title card (sampling ranges, model
caveats) opens and a closing card with the histograms (drift, touchdown speeds and tilt) and the outcome counts ends
it.  ``SIMULATION - not flight footage`` ("시뮬레이션 - 실제 비행 영상 아님") is always on screen.

Usage:
  python tools/campaign_video.py --campaign sim/out/vtol/<campaign dir> --out OUT.mp4 [--speed 2] [--lang ko|en]
                                 [--fps 30] [--size 1920x1080] [--at t1,t2] [--title-s 4] [--end-s 8]
                                 [--origin auto|pad|hold]
  python tools/campaign_video.py --synthetic OUT_DIR [--runs 200]   (a labelled kinematic test pattern)
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import sim_video as sv  # noqa: E402

PROJECT_ROOT = os.path.dirname(_HERE)
OUT_DIR = os.path.join(PROJECT_ROOT, "sim", "out", "video")

OUTCOME_DEFAULT_NAMES = ("pass", "landed_fail", "abort", "not_landed", "other")
OUTCOME_COLORS = {"pass": "#1baf7a", "landed_fail": "#eda100", "abort": "#e34948", "not_landed": "#8a63d2",
                  "other": "#7d8590"}
OUTCOME_LABELS = {"ko": {"pass": "착지 - 모든 기준 통과", "landed_fail": "착지 - 기준 미달", "abort": "비상정지",
                         "not_landed": "착지 확인 안 됨", "other": "기타"},
                  "en": {"pass": "landed, all criteria met", "landed_fail": "landed, a criterion failed",
                         "abort": "abort", "not_landed": "no landing confirmed", "other": "other"}}
WAITING, FLYING, LANDED, ABORTED, ENDED = range(5)
STATUS_NAMES = {"ko": ("대기", "비행 중", "착지", "비상정지", "종료(기타)"),
                "en": ("waiting", "flying", "landed", "aborted", "ended")}
STATUS_COLORS = ("#c9cdd3", "#2a78d6", "#1baf7a", "#e34948", "#8a63d2")
PHASE_LANDED = 5
STATE_SAFE, STATE_ARMED, STATE_POWERED, STATE_RELEASED, STATE_ABORT = 0, 1, 2, 4, 5
WIND_TRACE_S = 10.0
SQRT_SCALE_RATIO = 6.0      # sqrt radial scale when the farthest run drifts > 6 x the median run's maximum drift

T_ = {
    "title_top": {"ko": "CanSat EDF 기체 - VTOL 비행 캠페인 (시뮬레이션 분산)", "en": "CanSat EDF vehicle - VTOL campaign (simulated dispersion)"},
    "runs": {"ko": "{n}회 시뮬레이션", "en": "{n} simulated runs"},
    "map_title": {"ko": "위에서 본 궤적  (색 = 최종 결과, 점 = 현재 위치)", "en": "TOP VIEW  (colour = final outcome, dot = now)"},
    "map_sqrt": {"ko": "반경 눈금: 제곱근 척도 (멀리 간 기체까지 보이도록)", "en": "radial scale: square root (to keep far runs in view)"},
    "side_title": {"ko": "높이 - 시간  (모든 시뮬레이션)", "en": "HEIGHT vs TIME  (all runs)"},
    "h_axis": {"ko": "높이 [m]", "en": "height [m]"}, "t_axis": {"ko": "시뮬레이션 시간 [s]", "en": "sim time [s]"},
    "count_title": {"ko": "현재 상태별 기체 수", "en": "RUNS BY STATUS NOW"},
    "rose_title": {"ko": "바람 (각 시뮬레이션의 평균 바람)", "en": "WIND (each run's mean wind)"},
    "rose_now": {"ko": "바람 (각 시뮬레이션의 현재 평균 바람, 자취 10초)", "en": "WIND (each run's mean wind now, 10 s trace)"},
    "legend_title": {"ko": "최종 결과 (전체 {n}회)", "en": "FINAL OUTCOME ({n} runs)"},
    "sim_tag": {"ko": "시뮬레이션 - 실제 비행 영상 아님", "en": "SIMULATION - not flight footage"},
    "time": {"ko": "시뮬레이션 시간 {t:6.1f} s   (x{s:g} 배속)", "en": "sim time {t:6.1f} s   (x{s:g})"},
    "pad": {"ko": "패드", "en": "pad"},
    # v0.8: hold-point-relative tracks
    "map_title_hold": {"ko": "유지점 기준 궤적  (모든 유지점을 가운데에 겹침, 색 = 최종 결과)",
                       "en": "TRACKS RELATIVE TO EACH RUN'S HOLD POINT  (colour = final outcome)"},
    "hold": {"ko": "유지점", "en": "hold point"},
    "no_hold": {"ko": "유지점 없는 시뮬레이션 {n}회: 패드 기준으로 그림", "en": "{n} runs without a hold point: drawn from the pad"},
    "crit_ring": {"ko": "기준 {v:g} m", "en": "criterion {v:g} m"},
}
ORIGINS = ("auto", "pad", "hold")
V08_SCALARS = ("touchdown_vh_ground", "sk_rms_m", "sk_max_m", "touchdown_from_hold_m")


SAMPLING_KO = {
    "wind_mps": "평균 풍속 [m/s]", "wind_dir_deg": "풍향 [°]", "wind_swing_deg": "풍향 흔들림 폭 [°]",
    "wind_swing_period_s": "풍향 흔들림 주기 [s]", "turbulence_sigma_mps": "난류 세기 [m/s]", "n_gusts": "돌풍 횟수",
    "gust_mps": "돌풍 세기 [m/s]", "gust_duration_s": "돌풍 지속 [s]", "gust_t0_s": "돌풍 시작 [s]",
    "gust_rise_s": "돌풍 상승 [s]", "gust_vertical_mps": "돌풍 수직 성분 [m/s]", "packet_loss": "무선 패킷 손실률",
    "gyro_noise_scale": "자이로 잡음 배율", "gyro_bias_sigma_scale": "자이로 바이어스 배율", "accel_bias_g": "가속도계 바이어스 [g]",
    "baro_noise_m": "기압계 잡음 [m]", "baro_thrust_comp_m": "기압계 추력 오차 [m]", "mass_scale": "질량 배율",
    "cg_offset_mm": "무게중심 오프셋 [mm]", "thrust_scale": "추력 배율", "servo_tau_scale": "서보 응답 배율",
    "vane_trim_sigma_deg": "베인 트림 오차 σ [°]", "batt_v_rest_cell": "셀 전압 [V]", "initial_tilt_deg": "초기 기울기 [°]",
    "t_on_s": "ON 시각 [s]", "up_after_s": "UP까지 [s]", "up_gap_s": "UP 간격 [s]", "t_down_s": "DOWN 시각 [s]",
    "off_after_down_s": "OFF 지연 [s]", "push_force_n": "밀기 힘 [N]", "push_duration_s": "밀기 시간 [s]",
    # v0.8 (campaign round: vtol_campaign lists the flown envelope and the receiver ranges first)
    "gps_latency_s": "GPS 실제 지연 [s]", "gps_pos_gm_sigma_m": "GPS 위치 떠돎 σ [m]", "gps_pos_gm_tau_s": "GPS 떠돎 시간상수 [s]",
    "gps_vel_white_mps": "GPS 속도 잡음 [m/s]", "gps_sky_blocked": "하늘 가림 비율", "gps_outage_s": "GPS 끊김 길이 [s]",
    "gps_heading_err_sigma_deg": "패드 방향 오차 σ [°]", "push_z_body_m": "민 높이 [m]", "n_pushes": "밀기 횟수",
    "swirl_scale": "잔류 회전 토크 배율", "thrust_voltage_exp": "추력 전압 지수", "hold_s": "호버링 시간 [s]",
    # v0.9 campaign round: the v0.9 plant / tether keys vtol_campaign now lists first (identifiers showed in Korean videos)
    "t_max_kgf": "최대 추력 (22.2 V) [kgf]", "p_swirl_negative": "잔류 회전 반대 방향 확률",
    "vane_force_scale": "베인 힘 배율", "fan_power_model": "팬 전력 모델 (0 P1 ~ 1 P2)",
    "rotor_inertia_scale": "로터 관성 배율", "lost_motion_deg": "베인 헐거움 [°]", "gear_backlash_deg": "서보 기어 유격 [°]",
    "indoor_wind_mps": "실내 바람 [m/s]", "p_indoor": "실내 비행 확률",
}


def sampling_items(samp: dict[str, Any], lang: str) -> list[str]:
    """'name lo..hi' per sampled variable (Korean names where known; identifiers otherwise)."""
    out = []
    for k, v in samp.items():
        name = SAMPLING_KO.get(k, k) if lang == "ko" else k
        if isinstance(v, (list, tuple)) and len(v) == 2 and all(isinstance(x, (int, float)) for x in v):
            out.append(f"{name} {v[0]:g}~{v[1]:g}")
        else:
            out.append(f"{name} {v}")
    return out


def tt(key: str, lang: str, **fmt: Any) -> str:
    d = T_[key]
    s = d.get(lang, d["en"])
    return s.format(**fmt) if fmt else s


# --------------------------------------------------------------------------- #
# loading
# --------------------------------------------------------------------------- #
@dataclass
class Tracks:
    t: np.ndarray
    pos: np.ndarray
    alive: np.ndarray
    phase: np.ndarray
    state: np.ndarray
    outcome: np.ndarray
    outcome_names: list[str]
    outcome_ko: list[str]
    meta: dict[str, Any]
    opt: dict[str, np.ndarray] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    source: str = ""
    sha256: str = ""

    @property
    def R(self) -> int:
        return int(self.pos.shape[0])

    @property
    def T(self) -> int:
        return int(self.t.shape[0])

    def label(self, code: int, lang: str) -> str:
        name = self.outcome_names[int(code)] if int(code) < len(self.outcome_names) else "other"
        if lang == "ko":
            if int(code) < len(self.outcome_ko) and self.outcome_ko[int(code)]:
                return self.outcome_ko[int(code)]
            return OUTCOME_LABELS["ko"].get(name, name)
        return OUTCOME_LABELS["en"].get(name, name)

    def color(self, code: int) -> str:
        name = self.outcome_names[int(code)] if int(code) < len(self.outcome_names) else "other"
        return OUTCOME_COLORS.get(name, OUTCOME_COLORS["other"])

    def end_index(self) -> np.ndarray:
        """Last alive sample of each run (0 when never alive)."""
        a = self.alive
        last = np.where(a, np.arange(self.T)[None, :], -1).max(axis=1)
        return np.maximum(last, 0)


def _strs(a: Any) -> list[str]:
    return [x.decode("utf-8", "replace") if isinstance(x, bytes) else str(x) for x in np.atleast_1d(np.asarray(a)).tolist()]


def load_tracks(path: str) -> Tracks:
    """Load and validate a tracks file (or the ``tracks.npz`` of a campaign directory)."""
    if os.path.isdir(path):
        path = os.path.join(path, "tracks.npz")
    with open(path, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    with np.load(path, allow_pickle=False) as d:
        data = {k: d[k] for k in d.files}
    for k in ("t", "pos", "alive", "phase", "state", "outcome", "outcome_names", "meta"):
        if k not in data:
            raise ValueError(f"tracks {path!r} lack the required key {k!r}")
    warnings: list[str] = []
    t = np.asarray(data["t"], dtype=float).reshape(-1)
    if t.size < 2 or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError("t must have >= 2 finite, strictly increasing samples")
    pos = np.asarray(data["pos"], dtype=float)
    if pos.ndim != 3 or pos.shape[1:] != (t.size, 3):
        raise ValueError(f"pos {pos.shape} is not [R, {t.size}, 3]")
    R, T = pos.shape[0], t.size
    if R < 1:
        raise ValueError("no runs")

    def shaped(key: str, shape: tuple[int, ...], dtype: Any) -> np.ndarray:
        a = np.asarray(data[key]).astype(dtype)
        if a.shape != shape:
            raise ValueError(f"{key} has shape {a.shape}, expected {shape}")
        return a
    alive = shaped("alive", (R, T), bool)
    phase = shaped("phase", (R, T), np.uint8)
    state = shaped("state", (R, T), np.uint8)
    outcome = shaped("outcome", (R,), np.int64)
    names = _strs(data["outcome_names"])
    if outcome.size and int(outcome.max()) >= len(names):
        raise ValueError(f"outcome code {int(outcome.max())} has no name (outcome_names has {len(names)})")
    for nm in names:
        if nm not in OUTCOME_COLORS:
            warnings.append(f"outcome name {nm!r} not recognised: drawn grey as 'other'")
    ko = _strs(data["outcome_labels_ko"]) if "outcome_labels_ko" in data else []
    raw = np.asarray(data["meta"])
    s = raw.item() if raw.shape == () else raw.reshape(-1)[0]
    try:
        meta = json.loads(s.decode("utf-8") if isinstance(s, bytes) else str(s))
        if not isinstance(meta, dict):
            raise ValueError
    except ValueError:
        warnings.append("meta is not a JSON object")
        meta = {}
    # positions: forward-fill non-finite samples (after a run ends) with the last finite one; a run with no
    # finite sample at all is a contract error
    bad = ~np.all(np.isfinite(pos), axis=2)
    if np.any(bad):
        for r in np.flatnonzero(np.any(bad, axis=1)):
            ok = np.flatnonzero(~bad[r])
            if ok.size == 0:
                raise ValueError(f"run {r} has no finite position")
            idx = np.maximum.accumulate(np.where(~bad[r], np.arange(T), -1))
            idx = np.where(idx < 0, ok[0], idx)
            pos[r] = pos[r, idx]
        if np.any(bad & alive):
            warnings.append(f"non-finite positions while alive in {int(np.count_nonzero(np.any(bad & alive, axis=1)))} "
                            "runs: held at the last finite value")
    opt: dict[str, np.ndarray] = {}
    for k in ("wind_mps", "wind_dir_deg", "wind_swing_deg", "wind_swing_period_s", "touchdown_t", "touchdown_vz",
              "touchdown_vh", "touchdown_tilt_deg", "touchdown_distance_m", "drift_max_m") + V08_SCALARS:
        if k in data:
            opt[k] = shaped(k, (R,), float)
    if "hold_xy" in data:
        opt["hold_xy"] = shaped("hold_xy", (R, 2), float)
    for k in ("seed", "run_index"):
        if k in data:
            opt[k] = shaped(k, (R,), np.int64)
    if "failed" in data:
        opt["failed"] = np.asarray(_strs(data["failed"]))
        if opt["failed"].shape != (R,):
            raise ValueError("failed must be [R]")
    if "wind_xy" in data:
        opt["wind_xy"] = shaped("wind_xy", (R, T, 2), float)
    return Tracks(t=t, pos=pos, alive=alive, phase=phase, state=state, outcome=outcome, outcome_names=names,
                  outcome_ko=ko, meta=meta, opt=opt, warnings=warnings, source=os.path.abspath(path), sha256=sha)


# --------------------------------------------------------------------------- #
# pure computations
# --------------------------------------------------------------------------- #
def run_status(phase: np.ndarray, state: np.ndarray, alive: np.ndarray) -> np.ndarray:
    """[R, T] status per run and sample: WAITING before the first non-SAFE state; FLYING while ARMED / POWERED
    (not in LANDED); LANDED from a LANDED phase until it arms again; ABORTED (latched) from ABORT / RELEASED;
    after a run's end its last status holds, and a run that ends while FLYING becomes ENDED."""
    R, T = phase.shape
    ar = np.arange(T)[None, :]
    started = np.maximum.accumulate(state != STATE_SAFE, axis=1)
    abort = np.maximum.accumulate((state == STATE_ABORT) | (state == STATE_RELEASED), axis=1)
    last_landed = np.maximum.accumulate(np.where(phase == PHASE_LANDED, ar, -1), axis=1)
    flying_now = ((state == STATE_ARMED) | (state == STATE_POWERED)) & (phase != PHASE_LANDED)
    last_fly = np.maximum.accumulate(np.where(flying_now, ar, -1), axis=1)
    st = np.full((R, T), WAITING, dtype=np.uint8)
    st[started] = FLYING
    st[started & (last_landed > last_fly)] = LANDED
    st[abort] = ABORTED
    # after the end: hold the last alive status; FLYING at the end -> ENDED
    last_alive = np.maximum.accumulate(np.where(alive, ar, -1), axis=1)
    never = last_alive < 0
    held = np.take_along_axis(st, np.maximum(last_alive, 0), axis=1)
    ended = ~alive & ~never
    out = np.where(ended, np.where(held == FLYING, ENDED, held), st)
    out[never] = WAITING
    return out.astype(np.uint8)


def status_counts(status: np.ndarray) -> np.ndarray:
    """[T, 5] number of runs in each status at each sample."""
    return np.stack([np.count_nonzero(status == s, axis=0) for s in range(5)], axis=1)


def interp_runs(t: np.ndarray, y: np.ndarray, tq: float, end_idx: np.ndarray) -> np.ndarray:
    """y [R, T, ...] at time tq for every run (linear), each run clamped at its last alive sample."""
    j = int(np.clip(np.searchsorted(t, tq, side="right") - 1, 0, t.size - 1))
    j1 = min(j + 1, t.size - 1)
    s = 0.0 if j1 == j else float(np.clip((tq - t[j]) / (t[j1] - t[j]), 0.0, 1.0))
    a = y[:, j]
    b = y[:, j1]
    val = a + (b - a) * s
    over = end_idx < j1
    if np.any(over):
        val[over] = y[over, np.minimum(end_idx[over], t.size - 1)]
    return val


def max_drift(tracks: Tracks) -> np.ndarray:
    """[R] largest horizontal distance from the pad while alive (``drift_max_m`` when given)."""
    if "drift_max_m" in tracks.opt:
        return tracks.opt["drift_max_m"]
    r = np.hypot(tracks.pos[:, :, 0], tracks.pos[:, :, 1])
    return np.where(tracks.alive, r, 0.0).max(axis=1)


def resolve_origin(tracks: Tracks, origin: str = "auto") -> str:
    """'hold' (tracks relative to each run's hold point) or 'pad'; auto = hold when the tracks carry a finite hold_xy."""
    if origin not in ORIGINS:
        raise ValueError("origin must be " + " | ".join(ORIGINS))
    has = "hold_xy" in tracks.opt and bool(np.any(np.all(np.isfinite(tracks.opt["hold_xy"]), axis=1)))
    if origin == "hold" and not has:
        raise ValueError("--origin hold needs hold_xy [R, 2] in the tracks (v0.8 GPS position hold)")
    if origin == "auto":
        return "hold" if has else "pad"
    return origin


def map_offsets(tracks: Tracks, origin: str) -> np.ndarray:
    """[R, 2] what is subtracted from each run's x, y on the map: its hold point ('hold'; a run without one: 0, i.e.
    from the pad), else 0."""
    off = np.zeros((tracks.R, 2))
    if origin == "hold" and "hold_xy" in tracks.opt:
        h = tracks.opt["hold_xy"]
        ok = np.all(np.isfinite(h), axis=1)
        off[ok] = h[ok]
    return off


def max_dist_from(tracks: Tracks, offsets: np.ndarray) -> np.ndarray:
    """[R] largest horizontal distance from the map origin of each run (pad or its hold point) while alive."""
    r = np.hypot(tracks.pos[:, :, 0] - offsets[:, :1], tracks.pos[:, :, 1] - offsets[:, 1:])
    return np.where(tracks.alive, r, 0.0).max(axis=1)


def radial_mode(drift: np.ndarray) -> tuple[str, float]:
    """('linear' | 'sqrt', the map radius in m): sqrt when the farthest run is > SQRT_SCALE_RATIO x the median."""
    d = np.asarray(drift, dtype=float)
    d = d[np.isfinite(d)]
    mx = float(d.max()) if d.size else 1.0
    med = float(np.median(d)) if d.size else 1.0
    R_map = sv.nice_step(max(2.0, 1.08 * mx))
    return ("sqrt" if mx > SQRT_SCALE_RATIO * max(med, 0.5) else "linear"), R_map


def radial_px(r_m: np.ndarray, R_map: float, radius_px: float, mode: str) -> np.ndarray:
    r = np.clip(np.asarray(r_m, dtype=float), 0.0, None) / R_map
    return radius_px * (np.sqrt(r) if mode == "sqrt" else r)


def ring_values(R_map: float, mode: str) -> list[float]:
    if mode == "sqrt":
        cands = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000]
        return [float(c) for c in cands if c <= R_map + 1e-9][-6:]
    step = sv.nice_step(R_map / 4.0)
    return [float(step * i) for i in range(1, 5) if step * i <= R_map + 1e-9]


def outcome_counts(tracks: Tracks) -> list[tuple[int, int]]:
    return [(c, int(np.count_nonzero(tracks.outcome == c))) for c in range(len(tracks.outcome_names))]


def summary_stats(tracks: Tracks) -> dict[str, Any]:
    """Numbers of the closing card (also in the sidecar)."""
    o = tracks.opt
    out: dict[str, Any] = {"runs": tracks.R, "outcomes": {tracks.outcome_names[c]: n for c, n in outcome_counts(tracks)}}
    dm = max_drift(tracks)
    out["drift_max_m"] = _pcts(dm)
    for k in ("touchdown_distance_m", "touchdown_vh", "touchdown_tilt_deg") + V08_SCALARS:
        if k in o:
            out[k] = _pcts(o[k])
    if "hold_xy" in o:
        out["runs_with_hold_point"] = int(np.count_nonzero(np.all(np.isfinite(o["hold_xy"]), axis=1)))
    if "touchdown_vz" in o:
        out["touchdown_abs_vz"] = _pcts(np.abs(o["touchdown_vz"]))
    if "failed" in o:
        from collections import Counter
        c = Counter(x for f in o["failed"] for x in str(f).split(";") if x)
        out["failed_criteria"] = dict(c.most_common())
    return out


def _pcts(a: np.ndarray) -> dict[str, float | int | None]:
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return {"n": 0, "p50": None, "p95": None, "max": None}
    return {"n": int(a.size), "p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)), "max": float(a.max())}


# --------------------------------------------------------------------------- #
# rendering
# --------------------------------------------------------------------------- #
class CampaignRenderer:
    def __init__(self, tracks: Tracks, fps: float = 30.0, speed: float = 2.0, size: tuple[int, int] = (1920, 1080),
                 lang: str = "ko", origin: str = "auto"):
        import matplotlib
        matplotlib.use("Agg")
        if lang not in ("ko", "en"):
            raise ValueError("lang must be ko | en")
        self.tr, self.fps, self.speed, self.lang = tracks, float(fps), float(speed), lang
        self.W, self.H = int(size[0]) // 2 * 2, int(size[1]) // 2 * 2
        self.s = s = self.H / 1080.0
        self.warnings = list(tracks.warnings)
        if lang == "ko" and sv.korean_font_path() is None:
            self.warnings.append("lang ko: no Korean font found: Hangul will not render")
        self.status = run_status(tracks.phase, tracks.state, tracks.alive)
        self.counts = status_counts(self.status)
        self.end_idx = tracks.end_index()
        self.t_end = float(tracks.t[int(self.end_idx.max())]) if tracks.R else float(tracks.t[-1])
        self.tq = sv.frame_times(float(tracks.t[0]), self.t_end, self.fps, self.speed)
        # v0.8: the map origin - the pad, or each run's own hold point ('hold')
        self.origin = resolve_origin(tracks, origin)
        self.offsets = map_offsets(tracks, self.origin)
        self.n_no_hold = (int(np.count_nonzero(~np.all(np.isfinite(tracks.opt["hold_xy"]), axis=1)))
                          if self.origin == "hold" else 0)
        self.pos_map = tracks.pos.copy()
        self.pos_map[:, :, :2] -= self.offsets[:, None, :]
        self.drift = max_drift(tracks) if self.origin == "pad" else max_dist_from(tracks, self.offsets)
        self.mode, self.R_map = radial_mode(self.drift)
        crit = (tracks.meta.get("criteria") or {}).get("station_keeping_max_m")
        self.sk_crit = sv.criterion_value(crit, tracks.meta.get("profile")) if self.origin == "hold" else None
        # layout
        m = int(16 * s)
        self.banner_h = int(56 * s)
        self.map_box = (m, self.banner_h + int(8 * s), m + int(1040 * s), self.H - m)
        x0 = self.map_box[2] + int(24 * s)
        self.side_box = (x0, self.banner_h + int(8 * s), self.W - m, self.banner_h + int(400 * s))
        self.count_box = (x0, self.side_box[3] + int(12 * s), self.W - m, self.side_box[3] + int(332 * s))
        self.bottom_box = (x0, self.count_box[3] + int(12 * s), self.W - m, self.H - m)
        mx0, my0, mx1, my1 = self.map_box
        self.map_c = np.array([0.5 * (mx0 + mx1), 0.5 * (my0 + my1) + int(14 * s)])
        self.map_r = 0.5 * min(mx1 - mx0, my1 - my0) - int(40 * s)
        rose = int(min(self.bottom_box[3] - self.bottom_box[1], 300 * s))
        self.rose_box = (self.bottom_box[0], self.bottom_box[1], self.bottom_box[0] + rose, self.bottom_box[1] + rose)
        self.alpha = int(np.clip(50 + 6000 / max(tracks.R, 1), 50, 230))
        self._build_static()
        self.reset()

    # ---- static backgrounds ----------------------------------------------------------
    def _f(self, size: float, bold: bool = False, mono: bool = False) -> Any:
        return sv.font(int(size), mono=mono, bold=bold, lang=self.lang)

    def _build_static(self) -> None:
        from PIL import Image, ImageDraw
        s, L, trk = self.s, self.lang, self.tr
        bg = Image.new("RGB", (self.W, self.H), (246, 247, 249))
        d = ImageDraw.Draw(bg)
        # map panel
        mx0, my0, mx1, my1 = self.map_box
        d.rounded_rectangle([mx0, my0, mx1, my1], radius=int(10 * s), fill=(252, 252, 251), outline=(190, 194, 200),
                            width=max(1, int(s)))
        hold_mode = self.origin == "hold"
        d.text((mx0 + int(14 * s), my0 + int(10 * s)), tt("map_title_hold" if hold_mode else "map_title", L), fill=(20, 20, 20),
               font=self._f(17 * s, bold=True))
        y_sub = my0 + int(34 * s)
        if self.mode == "sqrt":
            d.text((mx0 + int(14 * s), y_sub), tt("map_sqrt", L), fill=(150, 90, 20), font=self._f(13 * s))
            y_sub += int(20 * s)
        if hold_mode and self.n_no_hold:
            d.text((mx0 + int(14 * s), y_sub), tt("no_hold", L, n=self.n_no_hold), fill=(150, 90, 20), font=self._f(13 * s))
        cx, cy = self.map_c
        fr = self._f(13 * s)
        for i, rv in enumerate(ring_values(self.R_map, self.mode)):
            rr = float(radial_px(np.array([rv]), self.R_map, self.map_r, self.mode)[0])
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=sv.hex_rgb(sv.RING), width=max(1, int(1.2 * s)))
            lab = f"{rv:g} m"
            d.text((cx + rr * 0.707 + 3 * s, cy - rr * 0.707 - 16 * s), lab, fill=sv.hex_rgb(sv.RING), font=fr)
        if hold_mode and self.sk_crit is not None and self.sk_crit < self.R_map:
            # the station-keeping criterion: a red dashed ring (24 dashes)
            rr = float(radial_px(np.array([self.sk_crit]), self.R_map, self.map_r, self.mode)[0])
            for j in range(24):
                a0, a1 = j * 15.0, j * 15.0 + 9.0
                d.arc([cx - rr, cy - rr, cx + rr, cy + rr], a0, a1, fill=sv.hex_rgb(sv.GUST_C), width=max(1, int(2 * s)))
            lab = tt("crit_ring", L, v=self.sk_crit)
            d.text((cx - rr * 0.707 - d.textlength(lab, font=fr) - 4 * s, cy + rr * 0.707), lab, fill=sv.hex_rgb(sv.GUST_C),
                   font=fr)
        mark_c = sv.hex_rgb(sv.HOLD_C) if hold_mode else (60, 60, 60)
        d.line([cx - 9 * s, cy, cx + 9 * s, cy], fill=mark_c, width=max(1, int(1.5 * s)))
        d.line([cx, cy - 9 * s, cx, cy + 9 * s], fill=mark_c, width=max(1, int(1.5 * s)))
        if hold_mode:
            d.ellipse([cx - 6 * s, cy - 6 * s, cx + 6 * s, cy + 6 * s], outline=mark_c, width=max(1, int(1.5 * s)))
        d.text((cx + 6 * s, cy + 4 * s), tt("hold" if hold_mode else "pad", L), fill=(80, 80, 80), font=fr)
        ax0 = (mx1 - int(70 * s), my1 - int(18 * s))
        d.line([ax0, (ax0[0] + 26 * s, ax0[1])], fill=(70, 70, 70), width=max(1, int(1.5 * s)))
        d.text((ax0[0] + 30 * s, ax0[1] - 10 * s), "x", fill=(70, 70, 70), font=fr)
        d.line([ax0, (ax0[0], ax0[1] - 26 * s)], fill=(70, 70, 70), width=max(1, int(1.5 * s)))
        d.text((ax0[0] - 14 * s, ax0[1] - 34 * s), "y", fill=(70, 70, 70), font=fr)
        # side view + counter history (matplotlib, rendered once)
        self.side_img, self.side_tf = self._mpl_side()
        bg.paste(Image.fromarray(self.side_img), (self.side_box[0], self.side_box[1]))
        self.count_img, self.count_tf = self._mpl_counts()
        # bottom: legend (right of the rose)
        bx0, by0, bx1, by1 = self.bottom_box
        lx = self.rose_box[2] + int(18 * s)
        d.rounded_rectangle([lx - int(8 * s), by0, bx1, by1], radius=int(10 * s), fill=(252, 252, 251), outline=(190, 194, 200),
                            width=max(1, int(s)))
        d.text((lx + int(4 * s), by0 + int(10 * s)), tt("legend_title", L, n=trk.R), fill=(20, 20, 20),
               font=self._f(15 * s, bold=True))
        y = by0 + int(40 * s)
        for c, n in outcome_counts(trk):
            col = sv.hex_rgb(trk.color(c))
            d.rounded_rectangle([lx + int(4 * s), y + int(3 * s), lx + int(22 * s), y + int(19 * s)], radius=int(3 * s), fill=col)
            d.text((lx + int(30 * s), y), f"{trk.label(c, L)}", fill=(30, 30, 30), font=self._f(14 * s))
            txt = f"{n}  ({100.0 * n / max(trk.R, 1):.0f} %)"
            tw = d.textlength(txt, font=self._f(14 * s, bold=True))
            d.text((bx1 - int(12 * s) - tw, y), txt, fill=(30, 30, 30), font=self._f(14 * s, bold=True))
            y += int(26 * s)
        info = self._info_lines()
        for ln in info:
            if y > by1 - int(22 * s):
                break
            d.text((lx + int(4 * s), y + int(4 * s)), ln, fill=(95, 100, 110), font=self._f(12 * s))
            y += int(19 * s)
        self.bg = bg
        self.rose_bg = self._rose_bg()

    def _info_lines(self) -> list[str]:
        m, L = self.tr.meta, self.lang
        out = []
        prof = m.get("profile")
        est = m.get("estimator")
        seed = m.get("campaign_seed")
        if L == "ko":
            out.append(f"프로파일 {prof or '?'}   추정기 {est or '?'}   캠페인 시드 {seed if seed is not None else '?'}")
            crit = m.get("criteria") or {}
            if crit and "touchdown_vh_ground_max_mps" in crit:
                # v0.9 campaign round (bug fix): the VTOL configuration's judged touchdown criteria (the v0.7.1 air-relative
                # horizontal speed / tilt were shown and read as the v0.9 criteria)
                out.append(f"기준: |v_z|≤{crit.get('touchdown_vz_max_mps', '?')}, 지면 수평≤"
                           f"{crit['touchdown_vh_ground_max_mps']:g} m/s, 넘어짐 여유>0, 상승 초과≤"
                           f"{crit.get('climb_overshoot_max_m', '?')} m(유선 {crit.get('climb_overshoot_tether_max_m', '?')})")
            elif crit:
                out.append(f"착지 기준: |v_z| <= {crit.get('touchdown_vz_max_mps', '?')} m/s, 수평 <= "
                           f"{crit.get('touchdown_vh_max_mps', '?')} m/s, 기울기 <= {crit.get('touchdown_tilt_max_deg', '?')}°")
        else:
            out.append(f"profile {prof or '?'}   estimator {est or '?'}   campaign seed {seed if seed is not None else '?'}")
            if m.get("criteria_text"):
                out.extend(sv.wrap(str(m["criteria_text"]), 70)[:3])
        return out

    def _mpl_fig(self, w: int, h: int) -> Any:
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        fig = Figure(figsize=(w / 100.0, h / 100.0), dpi=100.0, facecolor=sv.SURFACE)
        FigureCanvasAgg(fig)
        return fig

    def _rc(self) -> dict[str, Any]:
        if self.lang == "ko":
            fam = sv.mpl_korean_family()
            if fam:
                return {"font.family": [fam, "DejaVu Sans"], "axes.unicode_minus": False}
        return {}

    def _mpl_side(self) -> tuple[np.ndarray, Any]:
        import matplotlib
        x0, y0, x1, y1 = self.side_box
        w, h = x1 - x0, y1 - y0
        s, L, trk = self.s, self.lang, self.tr
        with matplotlib.rc_context(self._rc()):
            fig = self._mpl_fig(w, h)
            ax = fig.add_axes((0.09, 0.15, 0.88, 0.72))
            fs = 11 * s
            z = np.where(trk.alive, trk.pos[:, :, 2], np.nan)
            zmax = float(np.nanmax(z)) if np.any(np.isfinite(z)) else 1.0
            zmin = float(np.nanmin(z)) if np.any(np.isfinite(z)) else 0.0
            ax.set_xlim(float(trk.t[0]), self.t_end)
            ax.set_ylim(min(zmin, 0.0) - 0.05 * max(zmax, 1.0), 1.12 * max(zmax, 1.0))
            lim = trk.meta.get("limits") or {}
            for key in ("ceiling_m", "geofence_m"):
                if key in lim and float(lim[key]) <= 1.6 * max(zmax, 1.0):
                    ax.axhline(float(lim[key]), color=sv.GUST_C, lw=0.9 * s, ls="--")
            ax.axhline(0.0, color=sv.MUTED, lw=0.8 * s, ls=":")
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            ax.grid(True, color=sv.GRID, lw=0.6)
            ax.tick_params(labelsize=fs * 0.85, colors=sv.INK2)
            ax.set_xlabel(tt("t_axis", L), fontsize=fs, color=sv.INK2)
            ax.set_ylabel(tt("h_axis", L), fontsize=fs, color=sv.INK2)
            fig.text(0.015, 0.95, tt("side_title", L), fontsize=fs * 1.25, weight="bold", color=sv.INK, va="top")
            fig.canvas.draw()
            img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
            tf = ax.transData
            hh = img.shape[0]
            bb = ax.get_window_extent()
            box = (int(bb.x0), int(hh - bb.y1), int(bb.x1), int(hh - bb.y0))

        def to_px(tv: np.ndarray, zv: np.ndarray) -> np.ndarray:
            p = tf.transform(np.column_stack([tv, zv]))
            return np.column_stack([p[:, 0] + x0, hh - p[:, 1] + y0])
        self.side_axes_box = (box[0] + x0, box[1] + y0, box[2] + x0, box[3] + y0)
        return img, to_px

    def _mpl_counts(self) -> tuple[np.ndarray, Any]:
        import matplotlib
        x0, y0, x1, y1 = self.count_box
        w, h = x1 - x0, y1 - y0
        s, L = self.s, self.lang
        with matplotlib.rc_context(self._rc()):
            fig = self._mpl_fig(w, h)
            ax = fig.add_axes((0.09, 0.14, 0.88, 0.40))
            fs = 11 * s
            c = self.counts.astype(float)
            ax.stackplot(self.tr.t, c.T, colors=STATUS_COLORS, alpha=0.9, lw=0)
            ax.set_xlim(float(self.tr.t[0]), self.t_end)
            ax.set_ylim(0, self.tr.R)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            ax.tick_params(labelsize=fs * 0.8, colors=sv.INK2)
            ax.set_xlabel(tt("t_axis", L), fontsize=fs * 0.9, color=sv.INK2)
            fig.canvas.draw()
            img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
            hh = img.shape[0]
            bb = ax.get_window_extent()
            box = (int(bb.x0) + x0, int(hh - bb.y1) + y0, int(bb.x1) + x0, int(hh - bb.y0) + y0)
        t0, t1 = float(self.tr.t[0]), self.t_end
        self.count_axes_box = box

        def x_of(t: float) -> float:
            return box[0] + (min(max(t, t0), t1) - t0) / max(t1 - t0, 1e-9) * (box[2] - box[0])
        return img, x_of

    def _rose_bg(self) -> Any:
        from PIL import Image, ImageDraw
        s, L = self.s, self.lang
        x0, y0, x1, y1 = self.rose_box
        w = x1 - x0
        im = Image.new("RGBA", (w, w), (0, 0, 0, 0))
        d = ImageDraw.Draw(im)
        d.rounded_rectangle([0, 0, w - 1, w - 1], radius=int(10 * s), fill=(18, 24, 34, 255))
        has_xy = "wind_xy" in self.tr.opt
        d.text((int(10 * s), int(8 * s)), tt("rose_now" if has_xy else "rose_title", L), fill=(200, 208, 220, 255),
               font=self._f(11.5 * s, bold=True))
        wmax = self._wind_max()
        self.rose_wmax = sv.nice_step(max(1.0, 1.05 * wmax))
        self.rose_c = np.array([w / 2.0, w / 2.0 + 10 * s])
        self.rose_R = w / 2.0 - 30 * s
        cx, cy = self.rose_c
        step = sv.nice_step(self.rose_wmax / 3.0)
        r = step
        f = self._f(10 * s)
        while r <= self.rose_wmax + 1e-9:
            rr = r / self.rose_wmax * self.rose_R
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=(110, 125, 145, 255), width=1)
            d.text((cx + rr * 0.72 + 2 * s, cy + rr * 0.66), f"{r:g} m/s", fill=(150, 165, 185, 255), font=f)
            r += step
        for a, lab in ((0, "0° +x"), (90, "90° +y")):
            ar = math.radians(a)
            p = np.array([cx, cy]) + np.array([math.cos(ar), -math.sin(ar)]) * (self.rose_R + 4 * s)
            d.text((p[0] - (40 * s if a == 0 else -4 * s), p[1] - (16 * s if a == 0 else 2 * s)), lab, fill=(200, 208, 220, 255),
                   font=f)
        return im

    def _wind_max(self) -> float:
        o = self.tr.opt
        if "wind_xy" in o:
            return float(np.nanmax(np.hypot(o["wind_xy"][..., 0], o["wind_xy"][..., 1])))
        if "wind_mps" in o:
            return float(np.nanmax(o["wind_mps"]))
        return 1.0

    # ---- per frame --------------------------------------------------------------------
    def reset(self) -> None:
        """Start the incremental trail layers over (frames must then be drawn in time order)."""
        from PIL import Image
        mx0, my0, mx1, my1 = self.map_box
        self.map_layer = Image.new("RGBA", (mx1 - mx0, my1 - my0), (0, 0, 0, 0))
        sx0, sy0, sx1, sy1 = self.side_axes_box
        self.side_layer = Image.new("RGBA", (sx1 - sx0, sy1 - sy0), (0, 0, 0, 0))
        self.prev_t: float | None = None
        self.prev_xy: np.ndarray | None = None
        self.prev_side: np.ndarray | None = None

    def map_px(self, xy: np.ndarray) -> np.ndarray:
        """World xy [..., 2] -> map pixels (canvas), radial scale per ``self.mode``."""
        xy = np.asarray(xy, dtype=float)
        r = np.hypot(xy[..., 0], xy[..., 1])
        rp = radial_px(r, self.R_map, self.map_r, self.mode)
        with np.errstate(invalid="ignore", divide="ignore"):
            u = np.where(r[..., None] > 1e-9, xy / np.maximum(r, 1e-9)[..., None], 0.0)
        return np.stack([self.map_c[0] + u[..., 0] * rp, self.map_c[1] - u[..., 1] * rp], axis=-1)

    def _advance(self, t: float) -> None:
        """Draw every run's trail from the previous frame time to t into the accumulating layers."""
        from PIL import Image, ImageDraw
        trk = self.tr
        P = interp_runs(trk.t, self.pos_map, t, self.end_idx)
        xy = self.map_px(P[:, :2])
        side = self.side_tf(np.full(trk.R, t), P[:, 2])
        j = int(np.clip(np.searchsorted(trk.t, t, side="right") - 1, 0, trk.T - 1))
        started = self.status[:, j] != WAITING
        if self.prev_t is not None:
            mx0, my0 = self.map_box[0], self.map_box[1]
            sx0, sy0 = self.side_axes_box[0], self.side_axes_box[1]
            lay = Image.new("RGBA", self.map_layer.size, (0, 0, 0, 0))
            sl = Image.new("RGBA", self.side_layer.size, (0, 0, 0, 0))
            dm, ds = ImageDraw.Draw(lay), ImageDraw.Draw(sl)
            jprev = int(np.clip(np.searchsorted(trk.t, self.prev_t, side="right") - 1, 0, trk.T - 1))
            moving = started & (self.end_idx >= jprev)
            wl = max(1, int(round(1.4 * self.s)))
            for r in np.flatnonzero(moving):
                col = sv.hex_rgb(trk.color(int(trk.outcome[r]))) + (self.alpha,)
                a, b = self.prev_xy[r], xy[r]
                dm.line([(a[0] - mx0, a[1] - my0), (b[0] - mx0, b[1] - my0)], fill=col, width=wl)
                a, b = self.prev_side[r], side[r]
                ds.line([(a[0] - sx0, a[1] - sy0), (b[0] - sx0, b[1] - sy0)], fill=col, width=wl)
            self.map_layer = Image.alpha_composite(self.map_layer, lay)
            self.side_layer = Image.alpha_composite(self.side_layer, sl)
        self.prev_t, self.prev_xy, self.prev_side = t, xy, side
        self.cur_P, self.cur_xy, self.cur_side, self.cur_j = P, xy, side, j

    def frame(self, t: float) -> np.ndarray:
        """The frame at sim time t (call in increasing t; ``reset`` to start over)."""
        from PIL import ImageDraw
        if self.prev_t is not None and t < self.prev_t - 1e-9:
            self.reset()
        # a jump (preview frames): draw the history at the video's frame spacing up to t first
        t_from = float(self.tr.t[0]) - 1.0 if self.prev_t is None else self.prev_t
        for tp in self.tq[(self.tq > t_from + 1e-9) & (self.tq < t - 1e-9)]:
            self._advance(float(tp))
        self._advance(t)
        trk, s = self.tr, self.s
        im = self.bg.copy().convert("RGBA")
        im.alpha_composite(self.map_layer, (self.map_box[0], self.map_box[1]))
        im.alpha_composite(self.side_layer, (self.side_axes_box[0], self.side_axes_box[1]))
        d = ImageDraw.Draw(im)
        st = self.status[:, self.cur_j]
        # current positions
        rr = max(2.0, 3.6 * s * (1.0 if trk.R <= 300 else 0.75))
        for r in np.flatnonzero(st != WAITING):
            col = sv.hex_rgb(trk.color(int(trk.outcome[r]))) + (255,)
            x, y = self.cur_xy[r]
            if st[r] == FLYING:
                d.ellipse([x - rr, y - rr, x + rr, y + rr], fill=col, outline=(20, 20, 20, 255), width=1)
            elif st[r] == LANDED:
                d.ellipse([x - rr, y - rr, x + rr, y + rr], outline=col, width=max(1, int(1.6 * s)))
            elif st[r] == ABORTED:
                d.line([(x - rr, y - rr), (x + rr, y + rr)], fill=col, width=max(1, int(2 * s)))
                d.line([(x - rr, y + rr), (x + rr, y - rr)], fill=col, width=max(1, int(2 * s)))
            else:
                d.rectangle([x - rr * 0.8, y - rr * 0.8, x + rr * 0.8, y + rr * 0.8], outline=col, width=1)
        # side view cursor
        sx0, sy0, sx1, sy1 = self.side_axes_box
        xc = float(self.side_tf(np.array([t]), np.array([0.0]))[0, 0])
        d.line([(xc, sy0), (xc, sy1)], fill=(20, 20, 20, 255), width=max(1, int(1.5 * s)))
        for r in np.flatnonzero(st == FLYING):
            x, y = self.cur_side[r]
            d.ellipse([x - 2.2 * s, y - 2.2 * s, x + 2.2 * s, y + 2.2 * s], fill=sv.hex_rgb(trk.color(int(trk.outcome[r]))) + (255,))
        self._draw_counts(im, d, t)
        self._draw_rose(im, t)
        self._draw_banner(d, t)
        return np.asarray(im.convert("RGB"))

    def _draw_counts(self, im: Any, d: Any, t: float) -> None:
        from PIL import Image
        s, L = self.s, self.lang
        x0, y0, x1, y1 = self.count_box
        img = self.count_img.copy()
        ax0, ay0, ax1, ay1 = self.count_axes_box
        xc = int(round(self.count_tf(t)))
        if xc + 1 < ax1:                                   # the future dimmed
            reg = img[ay0 - y0:ay1 - y0, xc + 1 - x0:ax1 - x0].astype(np.float32)
            img[ay0 - y0:ay1 - y0, xc + 1 - x0:ax1 - x0] = (reg * 0.35 + 255.0 * 0.65).astype(np.uint8)
        img[ay0 - y0:ay1 - y0, max(ax0, xc - 1) - x0:min(ax1, xc + 1) + 1 - x0] = sv.hex_rgb(sv.INK)
        im.paste(Image.fromarray(img).convert("RGBA"), (x0, y0))
        d.text((x0 + int(12 * s), y0 + int(8 * s)), tt("count_title", L), fill=(20, 20, 20, 255), font=self._f(15 * s, bold=True))
        j = self.cur_j
        n = self.counts[j]
        bw = (x1 - x0 - int(24 * s)) / 5.0
        for k in range(5):
            bx = x0 + int(12 * s) + k * bw
            by = y0 + int(36 * s)
            col = np.array(sv.hex_rgb(STATUS_COLORS[k]), dtype=float)
            tint = tuple(int(v) for v in 0.16 * col + 0.84 * 255.0)            # drawn opaque: blend by hand
            dark = tuple(int(v) for v in 0.75 * col)
            d.rounded_rectangle([bx + 2 * s, by, bx + bw - 4 * s, by + int(92 * s)], radius=int(8 * s),
                                fill=tint + (255,), outline=tuple(int(v) for v in col) + (255,), width=max(1, int(2 * s)))
            num = f"{int(n[k])}"
            fb = self._f(34 * s, bold=True)
            tw = d.textlength(num, font=fb)
            d.text((bx + 0.5 * bw - tw / 2, by + int(8 * s)), num, fill=dark + (255,), font=fb)
            lab = STATUS_NAMES[L][k]
            fl = self._f(13 * s, bold=True)
            tw = d.textlength(lab, font=fl)
            d.text((bx + 0.5 * bw - tw / 2, by + int(60 * s)), lab, fill=(40, 40, 40, 255), font=fl)

    def _draw_rose(self, im: Any, t: float) -> None:
        from PIL import Image, ImageDraw
        trk, s = self.tr, self.s
        rose = self.rose_bg.copy()
        lay = Image.new("RGBA", rose.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(lay)
        c = self.rose_c
        k = self.rose_R / self.rose_wmax
        a = int(np.clip(60 + 3000 / max(trk.R, 1), 60, 230))
        o = trk.opt
        if "wind_xy" in o:
            W = interp_runs(trk.t, o["wind_xy"], t, self.end_idx)
            i0 = int(np.searchsorted(trk.t, t - WIND_TRACE_S))
            i1 = int(np.searchsorted(trk.t, t, side="right"))
        elif "wind_mps" in o and "wind_dir_deg" in o:
            ang = np.radians(o["wind_dir_deg"])
            W = np.column_stack([o["wind_mps"] * np.cos(ang), o["wind_mps"] * np.sin(ang)])
            i0 = i1 = 0
        else:
            W = None
            i0 = i1 = 0
        if W is not None:
            for r in range(trk.R):
                col = sv.hex_rgb(trk.color(int(trk.outcome[r])))
                if i1 - i0 > 1:
                    tr_ = o["wind_xy"][r, i0:i1]
                    pts = [(c[0] + w[0] * k, c[1] - w[1] * k) for w in tr_[:: max(1, (i1 - i0) // 25)]]
                    if len(pts) > 1:
                        d.line(pts, fill=col + (max(30, a // 3),), width=1)
                d.line([(c[0], c[1]), (c[0] + W[r, 0] * k, c[1] - W[r, 1] * k)], fill=col + (a,), width=max(1, int(1.3 * s)))
        rose = Image.alpha_composite(rose, lay)
        im.alpha_composite(rose, (self.rose_box[0], self.rose_box[1]))

    def _draw_banner(self, d: Any, t: float) -> None:
        s, L, m = self.s, self.lang, self.tr.meta
        title = str((m.get("title_ko") if L == "ko" else None) or m.get("title") or tt("title_top", L))
        txt = f"{title}   |   {tt('runs', L, n=self.tr.R)}"
        f = self._f(20 * s, bold=True)
        tag = tt("sim_tag", L)
        ft = self._f(16 * s, bold=True)
        tw = d.textlength(tag, font=ft)
        max_w = self.W - tw - int(380 * s)
        while d.textlength(txt, font=f) > max_w and len(txt) > 10:
            txt = txt[:-4] + "..."
        d.text((int(18 * s), int(14 * s)), txt, fill=(20, 24, 30, 255), font=f)
        tm = tt("time", L, t=t, s=self.speed)
        fm = self._f(17 * s, mono=True, bold=True)
        xr = self.W - int(16 * s)
        d.rounded_rectangle([xr - tw - int(24 * s), int(10 * s), xr, int(44 * s)], radius=int(6 * s), fill=(200, 40, 40, 220))
        d.text((xr - tw - int(12 * s), int(15 * s)), tag, fill=(255, 255, 255, 255), font=ft)
        tmw = d.textlength(tm, font=fm)
        d.text((xr - tw - int(40 * s) - tmw, int(16 * s)), tm, fill=(30, 60, 120, 255), font=fm)

    # ---- cards ------------------------------------------------------------------------
    def title_card(self) -> np.ndarray:
        from PIL import Image, ImageDraw
        s, L, m, trk = self.s, self.lang, self.tr.meta, self.tr
        im = Image.new("RGB", (self.W, self.H), (16, 24, 38))
        d = ImageDraw.Draw(im)
        x, y = int(self.W * 0.07), int(self.H * 0.09)
        max_w = self.W - 2 * x

        def line(txt: str, size: float, col: tuple[int, int, int], bold: bool = False) -> None:
            nonlocal y
            f = self._f(size * s, bold=bold)
            for ln in sv.wrap_px(d, txt, f, max_w):
                if y > self.H - int(50 * s):
                    return
                d.text((x, y), ln, fill=col, font=f)
                y += int(size * s * 1.45)
        line(tt("title_top", L), 26, (150, 190, 255), True)
        line(str((m.get("title_ko") if L == "ko" else None) or m.get("title") or ""), 38, (255, 255, 255), True)
        line(tt("runs", L, n=trk.R) + (f"   ·   캠페인 시드 {m.get('campaign_seed')}" if L == "ko" else
                                         f"   ·   campaign seed {m.get('campaign_seed')}")
             + f"   ·   params sha256 {str(m.get('params_sha256', '?'))[:12]}", 20, (200, 208, 220))
        hw = sv.hardware_of(m)                  # v0.9: the vehicle the campaign flew (meta.hardware_config / power_source)
        if not hw.legacy:
            line(("기체: " if L == "ko" else "vehicle: ") + hw.label(L), 20, (150, 215, 255))
        y += int(10 * s)
        samp = m.get("sampling_ASSUMED") or m.get("sampling") or {}
        if samp:
            line("시뮬레이션마다 무작위로 바꾼 조건 (가정 범위):" if L == "ko" else "Varied per run (ASSUMED ranges):", 21,
                 (255, 205, 120), True)
            line("    ".join(sampling_items(samp, L)[:30]), 16, (220, 226, 236))
        y += int(10 * s)
        notes = m.get("notes_ko") if L == "ko" else None
        if L == "ko":
            lines = sv.wrap(str(notes), 80) if notes else ["· " + q for q in sv.notes_ko_lines(m.get("model_notes", ""), "vtol")]
            line("모델의 한계와 가정" , 21, (255, 205, 120), True)
            for ln in lines[:8]:
                line(ln, 18, (225, 230, 238))
        elif m.get("model_notes"):
            line(str(m["model_notes"]), 17, (225, 230, 238))
        y += int(12 * s)
        line(("색 = 각 시뮬레이션의 최종 결과.  " if L == "ko" else "colour = each run's final outcome.  ")
             + tt("sim_tag", L), 20, (255, 140, 130), True)
        return np.asarray(im)

    def end_card(self) -> np.ndarray:
        import matplotlib
        from PIL import Image
        s, L, trk = self.s, self.lang, self.tr
        o = trk.opt
        crit = trk.meta.get("criteria") or {}
        # v0.9 campaign round (bug fix): on the VTOL configuration the JUDGED touchdown speed over the ground is
        # criteria.touchdown_vh_ground_max_mps (0.3 m/s, sim.vtol_scenarios.V09_TOUCHDOWN_VH_MAX_MPS); the v0.8
        # touchdown_ground_speed_mps (0.5 m/s) is only reported there - the histogram drew the 0.5 m/s line as the criterion
        gs_key = "touchdown_vh_ground_max_mps" if "touchdown_vh_ground_max_mps" in crit else "touchdown_ground_speed_mps"
        with matplotlib.rc_context(self._rc()):
            fig = self._mpl_fig(self.W, self.H)
            fig.set_facecolor("#10182a")
            fs = 12 * s
            gs = fig.add_gridspec(2, 3, left=0.06, right=0.97, top=0.80, bottom=0.13, hspace=0.62, wspace=0.22)
            panels = []
            if self.origin == "hold":
                # v0.8 station keeping first: touchdown from the hold point, the largest distance from it in HOLD
                prof = trk.meta.get("profile")
                for key, ko_t, en_t, ck, col in (
                        ("touchdown_from_hold_m", "착지 지점 - 유지점 거리 [m]", "touchdown distance from the hold point [m]",
                         "touchdown_from_hold_point_m", "#2a78d6"),
                        ("sk_max_m", "제자리 유지 최대 (호버링) [m]", "station keeping max in HOLD [m]", "station_keeping_max_m",
                         "#4fb8f0"),
                        ("touchdown_vh_ground", "착지 수평 속도 (지면 기준) [m/s]", "touchdown speed over the ground [m/s]",
                         gs_key, "#eda100")):
                    v_ = o.get(key)
                    if v_ is not None and np.any(np.isfinite(v_)):
                        panels.append((v_, ko_t if L == "ko" else en_t, sv.criterion_value(crit.get(ck), prof), col))
            n_v08 = len(panels)
            dist = o.get("touchdown_distance_m")
            if dist is not None and np.any(np.isfinite(dist)):
                panels.append((dist, "착지 지점 - 패드 거리 [m]" if L == "ko" else "touchdown distance from the pad [m]",
                               None, "#2a78d6"))
            panels.append((max_drift(trk), "최대 수평 이동 [m]" if L == "ko" else "largest drift from the pad [m]", None,
                           "#4a3aa7"))
            if "touchdown_vz" in o:
                panels.append((np.abs(o["touchdown_vz"]), "착지 수직 속도 |v_z| [m/s]" if L == "ko" else
                               "touchdown |v_z| [m/s]", crit.get("touchdown_vz_max_mps"), "#1baf7a"))
            gnd = o.get("touchdown_vh_ground")
            if n_v08 == 0 and gnd is not None and np.any(np.isfinite(gnd)) and gs_key in crit:
                # campaign round (bug fix): a v0.8 campaign drawn from the pad (GPS on or off) - the speed over the
                # ground is the v0.8 quantity; touchdown_vh is the v0.7.1 speed RELATIVE TO THE MEAN WIND (vtol_campaign
                # tracks_note), which reads as "the wind speed" for a vehicle that holds its position
                panels.append((gnd, "착지 수평 속도 (지면 기준) [m/s]" if L == "ko" else "touchdown speed over the ground [m/s]",
                               sv.criterion_value(crit.get(gs_key), trk.meta.get("profile")),
                               "#eda100"))
            elif "touchdown_vh" in o:
                panels.append((o["touchdown_vh"], "착지 수평 속도 (평균 바람 기준) [m/s]" if L == "ko"
                               else "touchdown horizontal speed (relative to the mean wind) [m/s]",
                               crit.get("touchdown_vh_max_mps"), "#eda100"))
            if "touchdown_tilt_deg" in o:
                panels.append((o["touchdown_tilt_deg"], "착지 기울기 [°]" if L == "ko" else "touchdown tilt [deg]",
                               crit.get("touchdown_tilt_max_deg"), "#e87ba4"))
            if n_v08:
                # hold view: after the station-keeping panels the touchdown |v_z| and tilt, then the pad-relative ones
                rest = panels[n_v08:]
                pri = [p for p in rest if ("v_z" in p[1] or "기울기" in p[1] or "tilt" in p[1])]
                panels = panels[:n_v08] + pri + [p for p in rest if not any(p is q for q in pri)]
            for i, (vals, title, lim, col) in enumerate(panels[:5]):
                ax = fig.add_subplot(gs[i // 3, i % 3])
                v = np.asarray(vals, dtype=float)
                v = v[np.isfinite(v)]
                ax.set_facecolor("#18223a")
                for side in ("top", "right"):
                    ax.spines[side].set_visible(False)
                for side in ("left", "bottom"):
                    ax.spines[side].set_color("#8090a8")
                ax.tick_params(colors="#c8d0dc", labelsize=fs * 0.85)
                if v.size:
                    ax.hist(v, bins=min(40, max(8, int(math.sqrt(v.size) * 2))), color=col, alpha=0.9)
                    p50, p95 = np.percentile(v, [50, 95])
                    ax.axvline(p50, color="#ffffff", lw=1.0 * s, ls=":")
                    sub = (f"중앙값 {p50:.2f} (흰 점선), 95% {p95:.2f}, 최대 {v.max():.2f}  (n {v.size})" if L == "ko" else
                           f"median {p50:.2f} (white dotted), 95% {p95:.2f}, max {v.max():.2f}  (n {v.size})")
                else:
                    sub = "값 없음" if L == "ko" else "no values"
                if lim is not None:
                    ax.axvline(float(lim), color="#ff6b6b", lw=1.6 * s, ls="--")
                    sub += (f"\n빨간 점선 = 기준 {float(lim):g}" if L == "ko" else f"\nred dashed = criterion {float(lim):g}")
                ax.set_title(title, color="#ffffff", fontsize=fs * 1.15, weight="bold", loc="left")
                ax.text(0.0, -0.13, sub, transform=ax.transAxes, color="#c8d0dc", fontsize=fs * 0.9, va="top",
                        linespacing=1.5)
            ax = fig.add_subplot(gs[1, 2])
            ax.set_facecolor("#18223a")
            cnt = outcome_counts(trk)
            labels = [trk.label(c, L) for c, _ in cnt]
            ax.barh(range(len(cnt)), [n for _, n in cnt], color=[trk.color(c) for c, _ in cnt], height=0.45)
            ax.set_yticks([])
            ax.invert_yaxis()
            for i, (_, n) in enumerate(cnt):                   # label above each bar, count at its end
                ax.text(0, i - 0.30, labels[i], color="#ffffff", va="bottom", fontsize=fs * 0.95)
                ax.text(n, i, f"  {n} ({100.0 * n / max(trk.R, 1):.0f} %)", color="#ffffff", va="center", fontsize=fs * 0.9)
            ax.set_ylim(len(cnt) - 0.5, -0.9)
            ax.set_xlim(0, max(1, max(n for _, n in cnt)) * 1.35)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            ax.tick_params(colors="#c8d0dc", labelsize=fs * 0.85)
            ax.set_title("최종 결과" if L == "ko" else "final outcome", color="#ffffff", fontsize=fs * 1.15, weight="bold",
                         loc="left")
            head = ("캠페인 요약 - " + tt("runs", L, n=trk.R)) if L == "ko" else ("Campaign summary - " + tt("runs", L, n=trk.R))
            fig.text(0.06, 0.95, head, color="#ffffff", fontsize=fs * 2.0, weight="bold", va="top")
            sub = []
            if "failed" in o:
                from collections import Counter
                c = Counter(x for f in o["failed"] for x in str(f).split(";") if x)
                if c:
                    sub.append(("자주 실패한 기준: " if L == "ko" else "most failed criteria: ") +
                               ", ".join(f"{k} {v}" for k, v in c.most_common(4)))
            sub.append(tt("sim_tag", L))
            fig.text(0.06, 0.885, "   ·   ".join(sub), color="#ffb3a8", fontsize=fs * 1.05, va="top")
            fig.canvas.draw()
            img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
        im = Image.fromarray(img)
        return np.asarray(im.convert("RGB"))


# --------------------------------------------------------------------------- #
# output
# --------------------------------------------------------------------------- #
def render_campaign(path: str, out: str, fps: float = 30.0, size: tuple[int, int] = (1920, 1080), speed: float = 2.0,
                    lang: str = "ko", title_s: float = 4.0, end_s: float = 8.0, at: Sequence[float] | None = None,
                    quiet: bool = False, origin: str = "auto") -> dict[str, Any]:
    """Render the campaign video (or PNG frames at ``at``); returns the sidecar dict (also <stem>.json)."""
    from PIL import Image
    t_wall0 = time.perf_counter()
    trk = load_tracks(path)
    rnd = CampaignRenderer(trk, fps, speed, size, lang, origin=origin)
    t_setup = time.perf_counter() - t_wall0
    stem, _ = os.path.splitext(out)
    info: dict[str, Any] = {"tracks": trk.source, "tracks_sha256": trk.sha256, "meta": trk.meta, "fps": fps, "speed": speed,
                            "size": [rnd.W, rnd.H], "lang": lang, "runs": trk.R, "origin": rnd.origin,
                            "runs_without_hold_point": rnd.n_no_hold, "radial_scale": rnd.mode,
                            "map_radius_m": rnd.R_map, "setup_s": round(t_setup, 3), "summary": summary_stats(trk),
                            "final_status_counts": dict(zip(STATUS_NAMES["en"], [int(x) for x in rnd.counts[-1]]))}
    files: list[str] = []
    if at:
        outdir = stem + "_frames"
        os.makedirs(outdir, exist_ok=True)
        t_r = time.perf_counter()
        for k, tq in enumerate(sorted(at)):
            p = os.path.join(outdir, f"frame_{k:03d}_t{tq:08.3f}.png")
            Image.fromarray(rnd.frame(float(tq))).save(p)
            files.append(os.path.abspath(p))
        for name, img in (("title", rnd.title_card()), ("end", rnd.end_card())):
            p = os.path.join(outdir, f"card_{name}.png")
            Image.fromarray(img).save(p)
            files.append(os.path.abspath(p))
        info["render_s"] = round(time.perf_counter() - t_r, 3)
    else:
        n_title, n_end = int(round(title_s * fps)), int(round(end_s * fps))
        timing = {"body_s": 0.0}
        last: dict[str, np.ndarray] = {}

        def gen():
            if n_title:
                tc = rnd.title_card()
                for _ in range(n_title):
                    yield tc
            tb = time.perf_counter()
            rnd.reset()
            for k, tq in enumerate(rnd.tq):
                img = rnd.frame(float(tq))
                if not quiet and k % max(1, int(fps * 5)) == 0:
                    el = time.perf_counter() - tb
                    print(f"  frame {k}/{rnd.tq.size}  t = {tq:.1f} s  {el / max(1, k):.3f} s/frame", flush=True)
                last["img"] = img
                yield img
            timing["body_s"] = time.perf_counter() - tb
            if n_end:
                ec = rnd.end_card()
                last["end"] = ec
                for _ in range(n_end):
                    yield ec
        t_r = time.perf_counter()
        n = sv.write_mp4(out, gen, (rnd.W, rnd.H), fps)
        files.append(os.path.abspath(out))
        pp = stem + "_poster.png"
        Image.fromarray(last["img"]).save(pp)
        files.append(os.path.abspath(pp))
        if "end" in last:
            pe = stem + "_summary.png"
            Image.fromarray(last["end"]).save(pe)
            files.append(os.path.abspath(pe))
        info.update({"frames": n, "body_frames": int(rnd.tq.size), "title_frames": n_title, "end_frames": n_end,
                     "video_s": round(n / fps, 3), "render_s": round(time.perf_counter() - t_r, 3),
                     "s_per_frame": round(timing["body_s"] / max(1, rnd.tq.size), 4)})
    info["warnings"] = rnd.warnings
    info["files"] = files
    info["wall_s"] = round(time.perf_counter() - t_wall0, 3)
    side = stem + ("_frames.json" if at else ".json")
    with open(side, "w", encoding="utf-8") as f:
        json.dump(info, f, indent=1, ensure_ascii=False, default=lambda o: o.tolist() if isinstance(o, np.ndarray) else str(o))
    info["sidecar"] = side
    return info


# --------------------------------------------------------------------------- #
# writing tracks (the contract as code, for the campaign runner)
# --------------------------------------------------------------------------- #
RUN_SCALARS = ("wind_mps", "wind_dir_deg", "wind_swing_deg", "wind_swing_period_s", "touchdown_t", "touchdown_vz",
               "touchdown_vh", "touchdown_tilt_deg", "touchdown_distance_m", "drift_max_m") + V08_SCALARS


def resample_run(t_log: np.ndarray, t_grid: np.ndarray, pos: np.ndarray, phase: np.ndarray, state: np.ndarray,
                 wind_xy: np.ndarray | None = None) -> dict[str, np.ndarray]:
    """One run's log onto the common grid: pos (and wind_xy) linear, phase / state zero-order hold, alive = grid
    times within the log; after the log's end pos holds its last value."""
    t_log = np.asarray(t_log, dtype=float)
    end = float(t_log[-1])
    tq = np.minimum(t_grid, end)
    hi = np.clip(np.searchsorted(t_log, tq, side="right") - 1, 0, t_log.size - 1)
    out = {"pos": np.stack([np.interp(tq, t_log, np.asarray(pos)[:, j]) for j in range(3)], axis=1).astype(np.float32),
           "phase": np.asarray(phase)[hi].astype(np.uint8), "state": np.asarray(state)[hi].astype(np.uint8),
           "alive": (t_grid <= end + 1e-9) & (t_grid >= float(t_log[0]) - 1e-9)}
    if wind_xy is not None:
        out["wind_xy"] = np.stack([np.interp(tq, t_log, np.asarray(wind_xy)[:, j]) for j in range(2)], axis=1).astype(np.float32)
    return out


def write_tracks(path: str, runs: Sequence[dict[str, Any]], meta: dict[str, Any], hz: float = 10.0,
                 outcome_names: Sequence[str] = OUTCOME_DEFAULT_NAMES) -> str:
    """Write ``tracks.npz`` from per-run dicts: required t [N] s, pos [N, 3] (exit plane, world m), phase [N], state [N],
    outcome (index into ``outcome_names``); optional wind_xy [N, 2] (mean wind over time), failed (str) and the
    scalars of ``RUN_SCALARS`` (NaN when absent), seed, run_index, hold_xy ([x, y] or None: v0.8).  The grid runs from
    the earliest start to the latest end at ``hz``."""
    if not runs:
        raise ValueError("no runs")
    t0 = min(float(np.asarray(r["t"])[0]) for r in runs)
    t1 = max(float(np.asarray(r["t"])[-1]) for r in runs)
    grid = np.round(t0 + np.arange(int(math.floor((t1 - t0) * hz + 1e-9)) + 1) / hz, 9)
    rs = [resample_run(r["t"], grid, r["pos"], r["phase"], r["state"], r.get("wind_xy")) for r in runs]
    data: dict[str, Any] = {"t": grid, "pos": np.stack([x["pos"] for x in rs]), "alive": np.stack([x["alive"] for x in rs]),
                            "phase": np.stack([x["phase"] for x in rs]), "state": np.stack([x["state"] for x in rs]),
                            "outcome": np.array([int(r["outcome"]) for r in runs], dtype=np.uint8),
                            "outcome_names": np.array(list(outcome_names)), "meta": np.array(json.dumps(meta))}
    if all("wind_xy" in x for x in rs):
        data["wind_xy"] = np.stack([x["wind_xy"] for x in rs])
    for k in RUN_SCALARS:
        if any(k in r for r in runs):
            data[k] = np.array([np.nan if r.get(k) is None else float(r[k]) for r in runs])
    if any("failed" in r for r in runs):
        data["failed"] = np.array([str(r.get("failed", "")) for r in runs])
    if any("hold_xy" in r for r in runs):
        data["hold_xy"] = np.array([[np.nan, np.nan] if r.get("hold_xy") is None else [float(v) for v in r["hold_xy"][:2]]
                                    for r in runs], dtype=float)
    for k, dt in (("seed", np.int64), ("run_index", np.int32)):
        if all(k in r for r in runs):
            data[k] = np.array([int(r[k]) for r in runs], dtype=dt)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez_compressed(path, **data)
    return path


# --------------------------------------------------------------------------- #
# synthetic tracks (a labelled kinematic test pattern for tests and development)
# --------------------------------------------------------------------------- #
def write_synthetic_tracks(path: str, runs: int = 200, seed: int = 7, hz: float = 10.0, swing_frac: float = 0.3,
                           with_hold: bool = False) -> str:
    """A tracks file of made-up kinematic runs (NOT a simulation): ON at 0.5-3 s, 2 s spool, climb to 5 m at 1 m/s,
    hold, descend at 0.6 m/s from 24-28 s, LANDED, SAFE; horizontal drift = 0.35 x the wind (made-up factor) plus a
    random walk; outcomes by a fixed lottery (65 % pass, 20 % landed_fail, 8 % abort (a fall from the hold), 7 %
    not_landed (motor on at the ground until 60 s)); ``swing_frac`` of the runs have a wind direction swinging
    +-30..80 deg with a 10-25 s period.  ``with_hold`` (v0.8 pattern): from the start of HOLD each run stays near its
    hold point (the position there) with a made-up mean-reverting wander (0.4 m, 3 s) instead of the wind drift, and
    the file carries hold_xy, sk_rms_m, sk_max_m, touchdown_from_hold_m, touchdown_vh_ground (all of the pattern)."""
    rng = np.random.default_rng(seed)
    t = np.round(np.arange(0.0, 62.0 + 1e-9, 1.0 / hz), 9)
    T = t.size
    R = int(runs)
    pos = np.full((R, T, 3), np.nan, dtype=np.float32)
    alive = np.zeros((R, T), dtype=bool)
    phase = np.zeros((R, T), dtype=np.uint8)
    state = np.zeros((R, T), dtype=np.uint8)
    wind_xy = np.zeros((R, T, 2), dtype=np.float32)
    lottery = rng.uniform(size=R)
    outcome = np.where(lottery < 0.65, 0, np.where(lottery < 0.85, 1, np.where(lottery < 0.93, 2, 3))).astype(np.uint8)
    w_mps = rng.uniform(0.0, 6.0, R)
    w_dir = rng.uniform(0.0, 360.0, R)
    swing = np.where(rng.uniform(size=R) < swing_frac, rng.uniform(30.0, 80.0, R), 0.0)
    period = rng.uniform(10.0, 25.0, R)
    td = np.full((R, 6), np.nan)          # touchdown t, vz, vh, tilt, distance, drift_max
    hold_xy = np.full((R, 2), np.nan)
    sk = np.full((R, 4), np.nan)          # sk_rms, sk_max, touchdown_from_hold, touchdown_vh_ground (pattern)
    failed = []
    for r in range(R):
        t_on = rng.uniform(0.5, 3.0)
        t_climb = t_on + 2.0
        t_hold = t_climb + 5.0
        t_down = rng.uniform(24.0, 28.0)
        t_land = t_down + 5.0 / 0.6 + 1.0
        oc = int(outcome[r])
        t_endr = {0: t_land + 2.6, 1: t_land + 2.6, 2: rng.uniform(t_hold + 2.0, t_down), 3: 60.0}[oc]
        a = (t <= t_endr + 1e-9)
        alive[r] = a
        ang = np.radians(w_dir[r] + swing[r] * np.sin(2.0 * math.pi * t / period[r]))
        wx, wy = w_mps[r] * np.cos(ang), w_mps[r] * np.sin(ang)
        wind_xy[r, :, 0], wind_xy[r, :, 1] = wx, wy
        z = np.interp(t, [0.0, t_climb, t_hold, t_down, t_land, 100.0], [0.0, 0.0, 5.0, 5.0, -0.11, -0.11])
        if oc == 2:                                        # a fall after the abort
            z = np.where(t >= t_endr - 0.9, np.maximum(-0.11, z - 0.5 * 9.81 * np.clip(t - (t_endr - 0.9), 0, None) ** 2), z)
        flying = (t >= t_climb) & (t < min(t_land, t_endr))
        dt = 1.0 / hz
        vx = 0.35 * wx * flying + rng.normal(0, 0.15, T) * flying
        vy = 0.35 * wy * flying + rng.normal(0, 0.15, T) * flying
        x, y = np.cumsum(vx) * dt, np.cumsum(vy) * dt
        if with_hold:
            # made-up station keeping: from the start of HOLD a mean-reverting wander about the position there
            kh = int(np.searchsorted(t, t_hold))
            hold_xy[r] = (x[kh], y[kh])
            e = np.zeros((T, 2))
            a_ = math.exp(-dt / 3.0)
            w_ = rng.normal(0.0, 0.4 * math.sqrt(1.0 - a_ * a_), size=(T, 2))
            for j in range(kh + 1, T):
                e[j] = a_ * e[j - 1] + w_[j]
            stay = t >= t_hold
            x = np.where(stay, hold_xy[r, 0] + e[:, 0], x)
            y = np.where(stay, hold_xy[r, 1] + e[:, 1], y)
            hd = np.hypot(x - hold_xy[r, 0], y - hold_xy[r, 1])
            inh = (t >= t_hold) & (t < min(t_down, t_endr))
            if np.any(inh):
                sk[r, 0], sk[r, 1] = float(np.sqrt(np.mean(hd[inh] ** 2))), float(hd[inh].max())
        pos[r, :, 0], pos[r, :, 1], pos[r, :, 2] = x, y, z
        st = np.zeros(T, dtype=np.uint8)
        ph = np.zeros(T, dtype=np.uint8)
        st[(t >= t_on) & (t < t_climb)] = STATE_ARMED
        ph[(t >= t_on) & (t < t_climb)] = 1
        st[(t >= t_climb) & (t < t_land)] = STATE_POWERED
        ph[(t >= t_climb) & (t < t_hold)] = 2
        ph[(t >= t_hold) & (t < t_down)] = 3
        ph[(t >= t_down) & (t < t_land)] = 4
        if oc in (0, 1):
            st[(t >= t_land) & (t < t_land + 2.0)] = 3
            ph[(t >= t_land) & (t < t_land + 2.0)] = PHASE_LANDED
            ph[t >= t_land + 2.0] = 0
            k = int(np.searchsorted(t, t_land))
            dist = float(np.hypot(x[k], y[k]))
            td[r] = [t_land, -rng.uniform(0.2, 0.9 if oc == 0 else 1.4), rng.uniform(0.05, 0.45 if oc == 0 else 2.0),
                     rng.uniform(0.2, 4.5 if oc == 0 else 9.0), dist, float(np.max(np.hypot(x, y)[a]))]
            if with_hold:
                sk[r, 2] = float(np.hypot(x[k] - hold_xy[r, 0], y[k] - hold_xy[r, 1]))
                sk[r, 3] = td[r, 2]
            failed.append("" if oc == 0 else rng.choice(["touchdown_vh", "touchdown_vz", "touchdown_tilt", "gust_recovery"]))
        elif oc == 2:
            st[t >= t_endr - 0.9] = STATE_ABORT
            ph[t >= t_endr - 0.9] = 0
            td[r, 5] = float(np.max(np.hypot(x, y)[a]))
            failed.append("no_interlock;landed_then_safe")
        else:
            st[t >= t_land] = STATE_POWERED                # on the ground, motor on, no LANDED until the cutoff
            ph[t >= t_land] = 4
            td[r, 5] = float(np.max(np.hypot(x, y)[a]))
            failed.append("landed_then_safe")
        state[r], phase[r] = st, ph
        pos[r, ~a] = np.nan
    meta = {"title": "SYNTHETIC TEST PATTERN: campaign tracks (kinematic, not a simulation)",
            "title_ko": "합성 테스트 패턴: 캠페인 궤적 (운동학 곡선, 시뮬레이션 아님)", "campaign_seed": seed, "n": R,
            "estimator": "n/a", "profile": "field", "params_sha256": "n/a",
            "criteria": {"touchdown_vz_max_mps": 1.0, "touchdown_vh_max_mps": 0.5, "touchdown_tilt_max_deg": 5.0},
            "criteria_text": "made-up lottery outcomes for the renderer test pattern",
            "model_notes": "Renderer test pattern written by tools/campaign_video.py: kinematic curves, no physics.",
            "sampling_ASSUMED": {"wind_mps": [0.0, 6.0], "wind_dir_deg": [0.0, 360.0], "wind_swing_deg": [30.0, 80.0]},
            "written_by": "tools/campaign_video.py write_synthetic_tracks"}
    extra: dict[str, Any] = {}
    if with_hold:
        meta["title"] = "SYNTHETIC TEST PATTERN: campaign tracks with hold points (kinematic, not a simulation)"
        meta["title_ko"] = "합성 테스트 패턴: 유지점이 있는 캠페인 궤적 (운동학 곡선, 시뮬레이션 아님)"
        meta["criteria"]["station_keeping_max_m"] = {"field": 3.0, "demo": 2.5}     # params criteria_v08_ASSUMED
        meta["criteria"]["touchdown_ground_speed_mps"] = 0.5
        extra = {"hold_xy": hold_xy, "sk_rms_m": sk[:, 0], "sk_max_m": sk[:, 1], "touchdown_from_hold_m": sk[:, 2],
                 "touchdown_vh_ground": sk[:, 3]}
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez_compressed(path, t=t, pos=pos, alive=alive, phase=phase, state=state, outcome=outcome,
                        outcome_names=np.array(OUTCOME_DEFAULT_NAMES[:4]), failed=np.array(failed), wind_mps=w_mps,
                        wind_dir_deg=w_dir, wind_swing_deg=swing, wind_swing_period_s=period, wind_xy=wind_xy,
                        touchdown_t=td[:, 0], touchdown_vz=td[:, 1], touchdown_vh=td[:, 2], touchdown_tilt_deg=td[:, 3],
                        touchdown_distance_m=td[:, 4], drift_max_m=td[:, 5], seed=np.arange(R, dtype=np.int64) + 1000,
                        run_index=np.arange(R, dtype=np.int32), meta=np.array(json.dumps(meta)), **extra)
    return path


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--campaign", help="campaign directory (with tracks.npz) or a tracks .npz")
    ap.add_argument("--out", help="output .mp4 (poster: <stem>_poster.png, closing card: <stem>_summary.png, sidecar: <stem>.json)")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--speed", type=float, default=2.0, help="sim seconds per video second")
    ap.add_argument("--lang", choices=("ko", "en"), default="ko")
    ap.add_argument("--title-s", type=float, default=4.0)
    ap.add_argument("--end-s", type=float, default=8.0)
    ap.add_argument("--at", default="", help="comma-separated sim times: PNG frames (+ both cards) instead of the video")
    ap.add_argument("--synthetic", default=None, help="write a synthetic tracks.npz (kinematic test pattern) into this directory")
    ap.add_argument("--runs", type=int, default=200)
    ap.add_argument("--synthetic-hold", action="store_true", help="--synthetic: the v0.8 pattern with hold points")
    ap.add_argument("--origin", choices=ORIGINS, default="auto",
                    help="map origin: pad, or each run's hold point (v0.8: hold_xy in the tracks); auto = hold when present")
    a = ap.parse_args(argv)
    if a.synthetic:
        p = write_synthetic_tracks(os.path.join(a.synthetic, "tracks.npz"), a.runs, with_hold=a.synthetic_hold)
        print("synthetic tracks", p)
        if not a.out:
            return 0
        a.campaign = a.campaign or a.synthetic
    if not a.campaign or not a.out:
        ap.error("--campaign and --out are required (or --synthetic DIR)")
    at = [float(x) for x in a.at.split(",") if x.strip()] if a.at else None
    info = render_campaign(a.campaign, a.out, a.fps, sv.parse_size(a.size), a.speed, a.lang, a.title_s, a.end_s, at,
                           origin=a.origin)
    for w in info["warnings"]:
        print("warning:", w)
    print(f"frames {info.get('frames', len(info['files']))}  render {info.get('render_s')} s  "
          f"({info.get('s_per_frame', '-')} s/frame)  setup {info['setup_s']} s")
    for fpath in info["files"]:
        print("  ", fpath)
    print("  sidecar", info["sidecar"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
