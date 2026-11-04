"""Before / after comparison video of a control change: two flight logs side by side in one MP4 (2026-10-21).

Why: every control change of the CanSat EDF VTOL vehicle is kept as a video that says HOW it was changed and lets
the audience compare the flight BEFORE and AFTER it (user request 2026-10-21, for the competition presentation).
Like ``tools/sim_video.py`` - whose log loader, renderer, styles, Korean text helpers and MP4 writer it imports and
uses read-only - it is a viewer: every number on screen is a logged value, a value resampled from a log, or a
quantity derived from logged values by the formula written next to it.  The change description, the reason and the
honesty statement come from a JSON spec written by a person; nothing on screen is invented by this tool.

Picture (1920 x 1080, 30 fps; other sizes scale):
  top banner      the change title in Korean (e.g. "변경 1: 자세 추정기 - 자이로 단독 → 가속도 보정형"), the scenario and
                  the alignment, the comparison tag (분리 비교 / 제어 + 모델 동시 변경 / 미확인) and the
                  "시뮬레이션 - 실제 비행 영상 아님" tag (on every frame and every card).
  two 3-D views   "변경 전" left (grey frame), "변경 후" right (blue frame), each a ``sim_video.Renderer`` 3-D view
                  (realistic style by default) of one log.  Both use ONE camera rule: the same azimuth, elevation and
                  view angle, the same distance rings on the ground and - with ``--camera-sync scale`` (default) -
                  the same camera distance in every frame (the larger of the two ``sim_video.plan_camera`` needs, at
                  most ``D_MAX_M`` = 14 m: near the ground the pad and the ground are framed, higher up the camera
                  closes in on the vehicle so attitude differences are visible), each centred on its own vehicle, so
                  the vehicle has the same size on both sides; ``--camera-sync shared`` uses one identical camera
                  for both (frames both vehicles: drift differences are seen as positions).  Per side: a label, a
                  compact HUD (aligned time + log time, phase, height and target, v_z, true and estimated tilt,
                  distance from the pad, throttle), a top-view inset of the track at ONE scale for both sides (the
                  drift from the pad stays visible when the camera is close), the latest supervisor event (Korean
                  reading of the logged text) and, for 5 s, each touchdown with the logged v_z / horizontal speed /
                  tilt.
  bottom strip    overlaid time plots on the ALIGNED time axis: before = grey dashed, after = blue solid (estimated
                  tilt dotted, target a thin step); channels (``--channels``): alt (height + target), tilt (true +
                  estimated), est_err (attitude-estimate error), dist (horizontal distance from the pad), throttle,
                  vz, phase (phase bands of both logs under every plot), touchdown (touchdown instants); a moving
                  cursor with the future dimmed and a dot per log on its curve.
  opening card    what changed and why (spec), how it is compared (scenario, seed, alignment, logs + sha256), the
                  honesty statement.
  end card        before / after metrics table (spec values, else computed from the logs, else the scenario entry of
                  a ``vtol_metrics.json`` named by the spec - the source of every value is marked), the criterion of
                  each row from the logs' ``meta.criteria`` and the honesty statement.

Alignment (``--align``): ``motor_on`` (default: first sample in POWERED), an event name (on_cmd, liftoff, hold,
land, off_cmd, touchdown, landed, start, ``cmd:NAME``, ``event:TEXT`` or any text found in a logged event) or
``offset:a,b`` (log times a of the before log and b of the after log are both shown as tau = 0).  The aligned time
is tau = t - offset; the video runs from max(both log starts, -2 s) to the later log end (``--start`` / ``--end`` in
tau); a log that has ended is frozen at its last sample with a "종료" overlay ("시작 전" before its first sample).

Honesty: the spec's ``isolation`` is "isolated" (only the control change differs: same simulation model, scenario,
seed), "control+model" (the simulation model changed too - e.g. any v0.7.0 log against a v0.7.1 log: body
cross-flow drag, EDF ram drag, wind-direction swing, hand pushes, baro thrust offset, accelerometer vibration were
added in v0.7.1 - then the differences cannot be credited to the control change alone, and the video says so on
the opening card, the end card and on every frame) or "unknown".  A spec claiming "isolated" for logs whose
``params_sha256`` / seed / scenario differ is reported as a warning (sidecar + console).

Spec (JSON, UTF-8; only ``title`` is required)::

  {"title": "변경 1: 자세 추정기 - 자이로 단독 → 가속도 보정형", "subtitle": "...",
   "what": ["...", ...], "why": ["...", ...], "expected": ["..."],
   "isolation": "isolated" | "control+model" | "unknown", "honesty_note": "(default by isolation)",
   "before": "path.npz", "after": "path.npz",            (relative to the spec file; the CLI overrides them)
   "before_label": "변경 전", "after_label": "변경 후", "before_version": "v0.7.0 추정기", "after_version": "...",
   "align": "motor_on", "channels": ["tilt", "est_err", ...], "metrics": ["td_vz", ...],
   "metric_values": {"before": {"td_vz": -0.3}, "after": {...}},     (override; marked * on the end card)
   "metrics_json": {"before": "vtol_metrics.json#V_nominal", ...},  (fills values the log lacks; marked †)
   "notes": ["end-card footnote", ...], "sources": ["params.json mission.vtol.nav_estimator", ...],
   "camera": {"azimuth": 90, "elevation": 12, "sync": "scale", "framing": "all", "scene": "auto", "d_max": 14},
   "title_s": 8, "end_s": 12, "poster_tau": 20.0,
   "layout": "split" | "shared", "ghost_align": "pad" | "hold"}                       (v0.8)

v0.8 (2026-10-21, GPS position hold; the sim_video log contract's optional keys hold_point / gps_* / pos_est /
tilt_sp_deg):
  channels        hold_dist (the true CG's horizontal distance from the hold point; a log without a logged hold point -
                  e.g. v0.7.1 - gets the v0.8 rule applied to it: the CG position at its first HOLD sample, marked
                  '계산' on screen and on the end card; the criterion line from meta.criteria.station_keeping_max_m),
                  hspeed (horizontal speed over the ground, truth), gps (satellites in use as a step line, red bands where
                  the GPS is not usable while powered).  ``V08_CHANNELS`` is a sensible set for station keeping.
  HUD / top view  rows 유지점 / 수평 속도 / GPS when either log holds position; the hold point (ring + cross) and a line
                  to the vehicle in each top view.
  metrics         sk_rms, sk_max (station keeping over HOLD), td_hold (first touchdown from the hold point), gps_avail
                  (GPS usable, % of the powered time); criteria per profile ({"field": .., "demo": ..}) are read.
  --layout shared ONE 3-D scene (``spec.layout``): the after log's vehicle (GPS pod, hold point, person) and the before
                  vehicle as a translucent grey ghost (``sim_video.GhostVehicle``) at its own aligned times, labelled; a
                  camera that keeps the ghost in view while that needs <= ``D_MAX_SHARED_M`` (spec camera.d_max), else an
                  edge arrow with the ghost's distance from the hold point; beside the view a table of both logs at
                  this instant and ONE top view at the after vehicle's scale (the ghost clipped, an edge arrow with its
                  distance).  ``--ghost-align hold`` (spec ``ghost_align``) moves the before log so its hold point lies
                  on the after one (said on screen), default ``pad`` (both as flown from the same pad).

Timing mode (``--timing spec.json``): an animated bar chart of the FCU tick for the tick-order change - the job
order of one 4 ms tick before / after, and per path (e.g. IMU sample -> vane register) the before / after bars
built from segments, each segment tagged measured / derived / analog / assumed / unmeasured (drawn differently and
named on screen), with the 4 ms tick and the limit lines; a cursor sweeps the time since the IMU sample.

Usage:
  python tools/compare_video.py --before B.npz --after A.npz --spec spec.json --out OUT.mp4
                                [--align motor_on|<event>|offset:a,b] [--channels alt,tilt,dist,vz,phase,touchdown]
                                [--speed 1] [--start TAU] [--end TAU] [--preview N] [--at tau1,tau2]
                                [--fps 30] [--size 1920x1080] [--style realistic|schematic]
                                [--camera-sync scale|shared] [--azimuth DEG] [--elevation DEG] [--framing ...]
                                [--layout split|shared] [--ghost-align pad|hold]
  python tools/compare_video.py --timing timing.json --out OUT.mp4 [--preview N]
Outputs: the MP4, ``<stem>_poster.png`` and ``<stem>.json`` (alignment, window, metrics with their sources,
camera, warnings, measured render speed ``s_per_frame``); ``--preview N`` writes N body frames + both cards as PNG
into ``<stem>_frames/`` instead.

Speed (measured 2026-10-21, Intel UHD 750, 1920 x 1080, realistic style, MSAA 8x, incl. H.264; the sidecar records
``s_per_frame`` and its parts): each side is rendered in its own pass into a disk cache (``<stem>_cache_*.npy``,
~1.6 MB per frame, deleted afterwards), then the frames are composed - alternating the two OpenGL contexts frame by
frame cost 0.74 s per view against 0.175 s in one context.  Measured: 0.13-0.16 s per 936 x 556 view and pass +
0.13 s compose / encode = 0.42 s per body frame (v0.7.0 vs v0.7.1 V_nominal, 1450 frames, 11 min wall incl. 42 s
setup); 0.61 s per frame while another workflow's six 6-DOF workers held the CPU at 100 %.  The timing mode draws
with PIL only: 0.34 s per frame.  A frozen side re-uses its last image while its camera does not move.

Invariant: reads only the two logs (+ the optional metrics JSON and the spec), ``params.json`` / the CAD STLs through
``sim_video``; never runs the simulation and never edits ``sim_video.py`` (its ``Scene`` ring-radius helper is
overridden only while the two scenes are built, so both sides draw the same distance rings).
"""
from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import re
import sys
import time
import types
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)
import sim_video as sv  # noqa: E402

PROJECT_ROOT = sv.PROJECT_ROOT
OUT_DIR = os.path.join(PROJECT_ROOT, "sim", "out", "compare")

# colours: before = neutral grey + dashes (the reference), after = categorical slot 1 (blue); the estimate in slot 2
BEFORE_C, AFTER_C, EST_C, BEFORE_EST_C, TARGET_C = "#7a7f87", "#2a78d6", "#eb6834", "#b4b8be", "#2b2b29"
BG_DARK = (16, 24, 38)
SIM_TAG = sv.TXT["sim_tag"]["ko"]                       # "시뮬레이션 - 실제 비행 영상 아님"
ISOLATIONS = ("isolated", "control+model", "unknown")
ISOLATION_TAG = {"isolated": "분리 비교: 제어 변경 하나만 다름",
                 "control+model": "주의: 제어 + 시뮬레이션 모델 동시 변경",
                 "unknown": "비교 조건 미확인"}
ISOLATION_NOTE = {
    "isolated": "분리 비교입니다. 두 로그는 같은 시뮬레이션 모델, 같은 시나리오, 같은 시드로 계산했고, 바뀐 것은 위의 제어 "
                "변경 하나뿐입니다. 그래도 시뮬레이션 결과이며 실제 비행의 측정값이 아닙니다.",
    "control+model": "제어와 시뮬레이션 모델이 함께 바뀌었습니다 (v0.7.1 모델에 기체 측면 항력, EDF 램 항력, 풍향 변화, "
                     "손 밀기, 기압계 추력 오차, 가속도계 진동이 추가됨). 그래서 이 차이 전부를 제어 변경의 효과로 볼 수 "
                     "없습니다. 제어 변경만의 효과는 분리 비교 영상을 보십시오.",
    "unknown": "비교 조건(시뮬레이션 모델, 시드, 시나리오가 같은지)이 확인되지 않았습니다. 차이의 원인을 단정할 수 없습니다.",
}
ISOLATION_COL = {"isolated": (70, 175, 115), "control+model": (240, 165, 40), "unknown": (150, 150, 150)}

DATA_CHANNELS = ("alt", "tilt", "est_err", "dist", "throttle", "vz", "hold_dist", "hspeed", "gps")
MOD_CHANNELS = ("phase", "touchdown")
CHANNELS = DATA_CHANNELS + MOD_CHANNELS
DEFAULT_CHANNELS = ("alt", "tilt", "dist", "vz", "phase", "touchdown")
# v0.8 station keeping: hold_dist, hspeed, gps (e.g. "channels": ["alt", "hold_dist", "hspeed", "tilt", "gps", "phase"])
V08_CHANNELS = ("alt", "hold_dist", "hspeed", "tilt", "gps", "phase", "touchdown")
CHANNEL_TITLE = {"alt": "높이 [m]  (참값, 목표)", "tilt": "기울기 [°]  (참값, 추정)", "est_err": "자세 추정 오차 [°]",
                 "dist": "패드로부터 수평 거리 [m]", "throttle": "스로틀 명령 u", "vz": "수직 속도 [m/s]",
                 "hold_dist": "유지점으로부터 수평 거리 [m]  (참값, 무게중심)", "hspeed": "수평 속도 [m/s]  (지면 기준, 참값)",
                 "gps": "GPS 위성 수  (붉은 띠 = GPS 사용 불가)"}
MISSING_NOTE = {"est_err": "자세 추정 오차 기록 없음", "hold_dist": "유지점 없음 (호버링 단계 없음)",
                "gps": "GPS 기록 없음 (GPS 없는 빌드)"}
LAYOUTS = ("split", "shared")
GHOST_ALIGNS = ("pad", "hold")
D_MAX_SHARED_M = 12.0           # --layout shared: the camera backs off at most this far to keep the ghost in view
                                # (the wide view still spans ~+-9 m there; farther the 0.86 m vehicle would shrink below
                                # ~40 px at 720 lines - the ghost then leaves the view and an edge marker points at it)
EXTRA_KEYS = ("h_true", "tilt_ctrl_deg", "tilt_est_err_deg", "reason", "vc_td_by", "vz_est")
SYNCS = ("scale", "shared")
D_MAX_M = 14.0                  # comparison camera: at most this far (sim_video chase: 70 m; demo preset 14 m)
PRE, RUN, ENDED = -1, 0, 1
PHASE_HOLD, PHASE_LAND, PHASE_LANDED = 3, 4, 5
LANDED_BY_KO = {1: "정지 감지 (still)", 2: "지지력 감지 (support)", 3: "바닥 감지 (floor)",
                "still": "정지 감지 (still)", "support": "지지력 감지 (support)", "floor": "바닥 감지 (floor)"}
ALIGN_EVENTS = {"motor_on": "모터 시동", "on_cmd": "ON 명령", "liftoff": "이륙", "hold": "호버링 시작",
                "land": "착륙 시작", "off_cmd": "OFF 명령", "touchdown": "첫 착지", "landed": "착지 확인",
                "start": "로그 시작"}
TD_RE = re.compile(r"touchdown #(\d+)\b.*?v_z ([-+]?\d+(?:\.\d+)?) m/s, horizontal (\d+(?:\.\d+)?) m/s"
                   r"(?:.*?tilt (\d+(?:\.\d+)?) deg)?")


# --------------------------------------------------------------------------- #
# spec
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MetricDef:
    label: str
    unit: str
    fmt: str
    better: str | None              # "lower" | "lower_abs" | None (no direction: shown, not judged)
    criterion: str | None = None    # meta.criteria key: a limit on |value|


METRICS: dict[str, MetricDef] = {
    "td_vz": MetricDef("첫 착지 수직 속도", "m/s", "+.2f", "lower_abs", "touchdown_vz_max_mps"),
    "td_vh": MetricDef("첫 착지 수평 속도 (지면 기준)", "m/s", ".2f", "lower", "touchdown_vh_max_mps"),
    "td_tilt": MetricDef("첫 착지 기울기", "°", ".1f", "lower", "touchdown_tilt_max_deg"),
    "td_dist": MetricDef("첫 착지 위치: 패드로부터 거리", "m", ".1f", "lower"),
    "est_err_rms": MetricDef("자세 추정 오차 RMS (동력 비행)", "°", ".2f", "lower"),
    "est_err_max": MetricDef("자세 추정 오차 최대 (동력 비행)", "°", ".2f", "lower"),
    "hold_rms": MetricDef("고도 유지 오차 RMS (HOLD, 참값 − 기준)", "m", ".3f", "lower", "hold_rms_err_max_m"),
    "t_land": MetricDef("착륙 시간 (LAND 시작 → 첫 착지)", "s", ".1f", None),
    "t_flight": MetricDef("모터 시동 → 착지 확인", "s", ".1f", None),
    "drift_max": MetricDef("최대 수평 이동 (출발점 기준)", "m", ".1f", "lower"),
    "max_tilt": MetricDef("최대 기울기 (동력 비행)", "°", ".1f", "lower"),
    "n_touchdowns": MetricDef("접지 횟수 (튀어 오름 포함)", "회", "d", "lower"),
    "landed_by": MetricDef("착지 확인 방식", "", "", None),
    "reasons": MetricDef("안전 규칙 작동 (reason 코드)", "", "", None),
    # v0.8 station keeping (truth CG against the hold point: the logged one, else the v0.8 rule applied to the log)
    "sk_rms": MetricDef("제자리 유지 RMS (호버링, 유지점 기준)", "m", ".2f", "lower"),
    "sk_max": MetricDef("제자리 유지 최대 (호버링, 유지점 기준)", "m", ".2f", "lower", "station_keeping_max_m"),
    "td_hold": MetricDef("첫 착지 위치: 유지점으로부터 거리", "m", ".2f", "lower", "touchdown_from_hold_point_m"),
    "gps_avail": MetricDef("GPS 사용 가능 (동력 비행 시간 중)", "%", ".0f", None),
}
V08_METRICS = ("sk_rms", "sk_max", "td_hold", "td_vh", "td_vz", "td_tilt", "gps_avail", "drift_max", "reasons")
DEFAULT_METRICS = ("td_vz", "td_vh", "td_tilt", "td_dist", "est_err_rms", "hold_rms", "t_land", "drift_max",
                   "landed_by", "reasons")
# vtol_metrics.json (sim.sim6dof.compute_vtol_metrics) key of each metric
JSON_KEYS = {"td_vz": "touchdown_vz", "td_vh": "touchdown_vh", "td_tilt": "touchdown_tilt_deg",
             "td_dist": "touchdown_distance_m", "est_err_rms": "rms_tilt_est_err_deg",
             "est_err_max": "max_tilt_est_err_deg", "hold_rms": "hold_rms_err_m", "drift_max": "drift_max_m",
             "max_tilt": "max_tilt_powered_deg", "n_touchdowns": "n_touchdowns", "landed_by": "landed_by",
             "reasons": "interlocks", "sk_rms": "sk_rms_m", "sk_max": "sk_max_m", "td_hold": "td_dist_hold_m"}
# |computed - metrics JSON| above max(rel * |json|, abs) is reported (two definitions should agree)
CROSSCHECK_TOL = {"rel": 0.03, "abs": {"td_dist": 0.3, "drift_max": 0.3, "t_land": 0.05, "t_flight": 0.05}}


def _str_list(v: Any, what: str) -> list[str]:
    if v is None:
        return []
    if isinstance(v, str):
        return [v]
    if not isinstance(v, (list, tuple)) or not all(isinstance(x, (str, int, float)) for x in v):
        raise ValueError(f"spec.{what} must be a string or a list of strings")
    return [str(x) for x in v]


@dataclass
class CompareSpec:
    title: str
    subtitle: str = ""
    what: list[str] = field(default_factory=list)
    why: list[str] = field(default_factory=list)
    expected: list[str] = field(default_factory=list)
    isolation: str = "unknown"
    honesty_note: str = ""
    before: str | None = None
    after: str | None = None
    before_label: str = "변경 전"
    after_label: str = "변경 후"
    before_version: str = ""
    after_version: str = ""
    align: str = "motor_on"
    channels: tuple[str, ...] = DEFAULT_CHANNELS
    metrics: tuple[str, ...] = DEFAULT_METRICS
    metric_values: dict[str, dict[str, Any]] = field(default_factory=dict)
    metrics_json: dict[str, str] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    sources: list[str] = field(default_factory=list)
    camera: dict[str, Any] = field(default_factory=dict)
    title_s: float = 8.0
    end_s: float = 12.0
    poster_tau: float | None = None
    base_dir: str = "."
    layout: str = "split"           # v0.8: "shared" = both vehicles in ONE scene (the before one as a ghost)
    ghost_align: str = "pad"        # shared layout: "pad" (logs as flown) | "hold" (before moved onto the after hold point)

    @property
    def note(self) -> str:
        return self.honesty_note or ISOLATION_NOTE[self.isolation]

    def resolve(self, p: str | None) -> str | None:
        """A spec path relative to the spec file's directory (absolute paths unchanged)."""
        if p is None:
            return None
        p = os.path.expandvars(os.path.expanduser(str(p)))
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(self.base_dir, p))


SPEC_KEYS = {"title", "subtitle", "what", "why", "expected", "isolation", "honesty_note", "before", "after",
             "before_label", "after_label", "before_version", "after_version", "align", "channels", "metrics",
             "metric_values", "metrics_json", "notes", "sources", "camera", "title_s", "end_s", "poster_tau", "comment",
             "layout", "ghost_align"}


def parse_channels(v: Any) -> tuple[str, ...]:
    items = [x.strip() for x in v.split(",")] if isinstance(v, str) else list(v)
    items = [x for x in items if x]
    bad = [x for x in items if x not in CHANNELS]
    if bad:
        raise ValueError(f"unknown channel(s) {bad}; choose from {', '.join(CHANNELS)}")
    if not any(x in DATA_CHANNELS for x in items):
        raise ValueError(f"channels need at least one of {', '.join(DATA_CHANNELS)}")
    return tuple(dict.fromkeys(items))


def parse_spec(d: dict[str, Any], base_dir: str = ".") -> CompareSpec:
    """Validate a spec dict (see the module docstring); raises ValueError naming the bad key."""
    if not isinstance(d, dict):
        raise ValueError("spec must be a JSON object")
    unknown = sorted(set(d) - SPEC_KEYS)
    if unknown:
        raise ValueError(f"unknown spec key(s) {unknown}")
    title = d.get("title")
    if not isinstance(title, str) or not title.strip():
        raise ValueError("spec.title (a non-empty string) is required")
    iso = str(d.get("isolation", "unknown"))
    if iso not in ISOLATIONS:
        raise ValueError(f"spec.isolation must be one of {ISOLATIONS}")
    mets = tuple(d.get("metrics", DEFAULT_METRICS))
    bad = [m for m in mets if m not in METRICS]
    if bad:
        raise ValueError(f"unknown metric(s) {bad}; choose from {', '.join(METRICS)}")
    mv = d.get("metric_values", {}) or {}
    if not isinstance(mv, dict) or set(mv) - {"before", "after"} or not all(isinstance(x, dict) for x in mv.values()):
        raise ValueError('spec.metric_values must be {"before": {metric: value}, "after": {...}}')
    for side, vals in mv.items():
        bad = [k for k in vals if k not in METRICS]
        if bad:
            raise ValueError(f"spec.metric_values.{side}: unknown metric(s) {bad}")
    mj = d.get("metrics_json", {}) or {}
    if not isinstance(mj, dict) or set(mj) - {"before", "after"} or not all(isinstance(x, str) for x in mj.values()):
        raise ValueError('spec.metrics_json must be {"before": "path#scenario", "after": "..."}')
    cam = d.get("camera", {}) or {}
    if not isinstance(cam, dict) or set(cam) - {"azimuth", "elevation", "sync", "framing", "scene", "d_max"}:
        raise ValueError("spec.camera keys: azimuth, elevation, sync, framing, scene, d_max")
    if "sync" in cam and cam["sync"] not in SYNCS:
        raise ValueError(f"spec.camera.sync must be one of {SYNCS}")
    if "framing" in cam and cam["framing"] not in sv.FRAMINGS:
        raise ValueError(f"spec.camera.framing must be one of {sv.FRAMINGS}")
    align = str(d.get("align", "motor_on"))
    parse_align(align)
    ts, es = float(d.get("title_s", 8.0)), float(d.get("end_s", 12.0))
    if ts < 0 or es < 0:
        raise ValueError("title_s / end_s must be >= 0")
    pt = d.get("poster_tau")
    layout = str(d.get("layout", "split"))
    if layout not in LAYOUTS:
        raise ValueError(f"spec.layout must be one of {LAYOUTS}")
    galign = str(d.get("ghost_align", "pad"))
    if galign not in GHOST_ALIGNS:
        raise ValueError(f"spec.ghost_align must be one of {GHOST_ALIGNS}")
    return CompareSpec(layout=layout, ghost_align=galign,
        title=title.strip(), subtitle=str(d.get("subtitle", "")), what=_str_list(d.get("what"), "what"),
        why=_str_list(d.get("why"), "why"), expected=_str_list(d.get("expected"), "expected"), isolation=iso,
        honesty_note=str(d.get("honesty_note", "")), before=d.get("before"), after=d.get("after"),
        before_label=str(d.get("before_label", "변경 전")), after_label=str(d.get("after_label", "변경 후")),
        before_version=str(d.get("before_version", "")), after_version=str(d.get("after_version", "")),
        align=align, channels=parse_channels(d.get("channels", DEFAULT_CHANNELS)), metrics=mets,
        metric_values={k: dict(v) for k, v in mv.items()}, metrics_json=dict(mj),
        notes=_str_list(d.get("notes"), "notes"), sources=_str_list(d.get("sources"), "sources"), camera=dict(cam),
        title_s=ts, end_s=es, poster_tau=None if pt is None else float(pt), base_dir=os.path.abspath(base_dir))


def load_spec(path: str) -> CompareSpec:
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    return parse_spec(d, os.path.dirname(os.path.abspath(path)))


# --------------------------------------------------------------------------- #
# logs, alignment
# --------------------------------------------------------------------------- #
@dataclass
class Flight:
    """One log: the sim_video ``SimLog`` + the extra keys of the VTOL export (``EXTRA_KEYS``, when present)."""

    log: sv.SimLog
    ex: dict[str, np.ndarray]
    path: str

    @property
    def t(self) -> np.ndarray:
        return self.log.t

    @property
    def height(self) -> np.ndarray:
        """h_true (the CG rise above its pad height: what the altitude loop controls and h_target / h_ref refer to)
        when logged, else the exit-plane z."""
        return self.ex["h_true"] if "h_true" in self.ex else self.log.pos[:, 2]

    @property
    def height_src(self) -> str:
        return "h_true" if "h_true" in self.ex else "pos_z"

    @property
    def hold(self) -> tuple[np.ndarray | None, str]:
        """(hold point per sample (N, 2), source) - see ``hold_reference``."""
        if not hasattr(self, "_hold"):
            self._hold = hold_reference(self.log)
        return self._hold

    def hold_dist(self) -> np.ndarray | None:
        """(N,) |CG_xy - hold_xy| (NaN before the hold point exists); None when there is none."""
        h, _ = self.hold
        return None if h is None else np.hypot(*(self.log.cg_xy() - h).T)


def hold_reference(log: sv.SimLog) -> tuple[np.ndarray | None, str]:
    """The hold point per sample and its source: the logged ``hold_point`` ('log'), else the v0.8 rule applied to the
    log - the CG position at the first HOLD sample, from that sample on ('derived': a v0.7.1 log has no hold point; its
    'distance from the hold point' is then the drift from where its hover began) - else (None, 'none')."""
    if log.has_hold:
        return log.hold_xy, "log"
    ih = np.flatnonzero(log.phase == PHASE_HOLD)
    if not ih.size:
        return None, "none"
    out = np.full((log.n, 2), np.nan)
    out[ih[0]:] = log.cg_xy()[ih[0]]
    return out, "derived"


def first_hold_xy(h: np.ndarray | None) -> np.ndarray | None:
    """The first finite hold point of a per-sample series (None when none)."""
    if h is None:
        return None
    i = np.flatnonzero(np.isfinite(h[:, 0]))
    return None if not i.size else np.asarray(h[i[0]], dtype=float)


def load_flight(path: str) -> Flight:
    log = sv.load_log(path)
    ex: dict[str, np.ndarray] = {}
    with np.load(path, allow_pickle=False) as d:
        for k in EXTRA_KEYS:
            if k in d.files:
                a = np.asarray(d[k])
                if a.ndim >= 1 and a.shape[0] >= log.n:
                    ex[k] = a[:log.n]
    return Flight(log, ex, os.path.abspath(path))


@dataclass(frozen=True)
class Align:
    event: str | None = "motor_on"
    offsets: tuple[float, float] | None = None

    @property
    def label(self) -> str:
        if self.offsets is not None:
            return f"지정 시각 (전 {self.offsets[0]:g} s, 후 {self.offsets[1]:g} s)"
        e = str(self.event)
        if e in ALIGN_EVENTS:
            return ALIGN_EVENTS[e]
        if e.startswith("cmd:"):
            return f"{sv.cmd_label(e[4:].upper(), 'ko')} 명령"
        return f"이벤트 '{e[6:] if e.startswith('event:') else e}'"


def parse_align(s: str) -> Align:
    """``motor_on`` | an event name / text | ``offset:a,b`` (log times shown as tau = 0)."""
    s = str(s).strip()
    if not s:
        raise ValueError("empty --align")
    if s.startswith("offset:"):
        parts = [p.strip() for p in s[7:].split(",")]
        if len(parts) != 2:
            raise ValueError("--align offset:a,b needs two numbers")
        try:
            a, b = float(parts[0]), float(parts[1])
        except ValueError:
            raise ValueError("--align offset:a,b needs two numbers") from None
        if not (math.isfinite(a) and math.isfinite(b)):
            raise ValueError("--align offsets must be finite")
        return Align(None, (a, b))
    return Align(s, None)


def _first_event(log: sv.SimLog, pattern: str, regex: bool = False) -> float | None:
    for t, x in zip(log.event_t, log.event_text):
        if (re.search(pattern, x) if regex else pattern.lower() in x.lower()):
            return float(t)
    return None


def event_time(log: sv.SimLog, name: str) -> float:
    """Log time of the named event (``ALIGN_EVENTS``, ``cmd:NAME``, ``event:TEXT`` or a logged-event substring)."""
    n = str(name).strip()
    t: float | None = None
    if n == "start":
        t = float(log.t[0])
    elif n == "motor_on":
        i = np.flatnonzero(log.state == sv.STATE_POWERED)
        t = float(log.t[i[0]]) if i.size else _first_event(log, "-> POWERED")
    elif n in ("hold", "land", "landed"):
        i = np.flatnonzero(log.phase == {"hold": PHASE_HOLD, "land": PHASE_LAND, "landed": PHASE_LANDED}[n])
        t = float(log.t[i[0]]) if i.size else None
    elif n == "liftoff":
        t = _first_event(log, r"^liftoff\b", regex=True)
    elif n == "touchdown":
        t = _first_event(log, r"^touchdown #1\b", regex=True)
    elif n in ("on_cmd", "off_cmd") or n.startswith("cmd:"):
        cmd = {"on_cmd": "ON", "off_cmd": "OFF"}.get(n, n[4:]).upper()
        ts = [float(ct) for ct, cn in zip(log.cmd_t, log.cmd_name) if str(cn).upper() == cmd]
        t = ts[0] if ts else None
    elif n.startswith("event:"):
        t = _first_event(log, n[6:])
    else:
        t = _first_event(log, n)
    if t is None:
        raise ValueError(f"alignment event {name!r} not found in {os.path.basename(log.source) or 'the log'}")
    return t


def align_offsets(before: sv.SimLog, after: sv.SimLog, al: Align) -> tuple[float, float]:
    """Log time of tau = 0 in each log."""
    if al.offsets is not None:
        return al.offsets
    return event_time(before, str(al.event)), event_time(after, str(al.event))


def tau_window(spans: Sequence[tuple[float, float]], offsets: Sequence[float], start: float | None = None,
               end: float | None = None, pre_s: float = 2.0) -> tuple[float, float]:
    """Aligned video interval: from max(the log starts, -pre_s) (or ``start``) to the later log end (or ``end``)."""
    starts = [a - o for (a, _), o in zip(spans, offsets)]
    ends = [b - o for (_, b), o in zip(spans, offsets)]
    t0 = max(max(starts), -abs(pre_s)) if start is None else float(start)
    t1 = max(ends) if end is None else float(end)
    if not (math.isfinite(t0) and math.isfinite(t1)) or t1 <= t0:
        raise ValueError(f"empty aligned interval [{t0}, {t1}]")
    return t0, t1


def map_side(tau: np.ndarray, offset: float, t_first: float, t_last: float) -> tuple[np.ndarray, np.ndarray]:
    """Log time of each aligned frame time (clamped to the log: a log is frozen at its last sample after its end)
    and the frame status: PRE (-1) before the first sample, RUN (0), ENDED (1) after the last sample."""
    t = np.asarray(tau, dtype=float) + float(offset)
    status = np.where(t < t_first - 1e-9, PRE, np.where(t > t_last + 1e-9, ENDED, RUN)).astype(np.int8)
    return np.clip(t, t_first, t_last), status


# --------------------------------------------------------------------------- #
# plot series (pure)
# --------------------------------------------------------------------------- #
@dataclass
class Series:
    role: str               # "main" | "est" | "target"
    tau: np.ndarray
    y: np.ndarray
    label: str
    step: bool = False


def channel_series(fl: Flight, name: str, offset: float) -> list[Series]:
    """The series of channel ``name`` of one log on the aligned axis tau = t - offset ([] when not logged)."""
    log, ex = fl.log, fl.ex
    tau = log.t - float(offset)
    out: list[Series] = []
    if name == "alt":
        out.append(Series("main", tau, fl.height, "높이 (참값)"))
        if np.any(np.isfinite(log.h_target)):
            out.append(Series("target", tau, log.h_target, "목표 고도", step=True))
    elif name == "tilt":
        out.append(Series("main", tau, log.tilt_deg, "기울기 (참값)"))
        if "tilt_ctrl_deg" in ex:
            out.append(Series("est", tau, ex["tilt_ctrl_deg"].astype(float), "기울기 (추정값: 제어기가 보는 값)"))
    elif name == "est_err":
        if "tilt_est_err_deg" in ex:
            out.append(Series("main", tau, ex["tilt_est_err_deg"].astype(float), "자세 추정 오차"))
    elif name == "dist":
        out.append(Series("main", tau, np.hypot(log.pos[:, 0], log.pos[:, 1]), "패드로부터 수평 거리"))
    elif name == "throttle":
        out.append(Series("main", tau, log.throttle, "스로틀 명령"))
    elif name == "vz":
        out.append(Series("main", tau, log.vel[:, 2], "수직 속도 (참값)"))
    elif name == "hold_dist":
        hd = fl.hold_dist()
        if hd is not None:
            src = fl.hold[1]
            out.append(Series("main", tau, hd, "유지점 거리" + (" (v0.8 규칙으로 계산한 유지점)" if src == "derived" else "")))
    elif name == "hspeed":
        out.append(Series("main", tau, np.hypot(log.vel[:, 0], log.vel[:, 1]), "수평 속도 (참값)"))
    elif name == "gps":
        if log.gps_sats is not None:
            out.append(Series("main", tau, log.gps_sats.astype(float), "GPS 위성 수", step=True))
        elif log.gps_ok is not None:
            out.append(Series("main", tau, log.gps_ok.astype(float), "GPS 사용 가능 (1 / 0)", step=True))
    else:
        raise ValueError(f"not a data channel: {name!r}")
    return out


def value_at(s: Series, tau: float) -> float:
    """The series at tau, clamped to its range (a frozen log keeps its last value); step series hold."""
    if s.tau.size == 0:
        return float("nan")
    tc = min(max(float(tau), float(s.tau[0])), float(s.tau[-1]))
    if s.step:
        i = int(np.clip(np.searchsorted(s.tau, tc, side="right") - 1, 0, s.tau.size - 1))
        return float(s.y[i])
    return float(np.interp(tc, s.tau, s.y))


def phase_spans(phase: np.ndarray, tau: np.ndarray) -> list[tuple[float, float, int]]:
    """(tau_start, tau_end, code) of each run of equal phase codes."""
    if tau.size == 0:
        return []
    edges = np.flatnonzero(np.diff(phase)) + 1
    starts = np.concatenate([[0], edges])
    ends = np.concatenate([edges, [tau.size]])
    return [(float(tau[a]), float(tau[b]) if b < tau.size else float(tau[-1]), int(phase[a])) for a, b in zip(starts, ends)]


def series_limits(values: Sequence[np.ndarray], floor0: bool = False, min_span: float = 1.0) -> tuple[float, float]:
    """y-limits covering every finite value with 6 % padding (``floor0``: from 0)."""
    v = np.concatenate([np.asarray(x, dtype=float)[np.isfinite(x)] for x in values] or [np.zeros(1)])
    if v.size == 0:
        v = np.zeros(1)
    lo, hi = float(v.min()), float(v.max())
    if floor0:
        lo = min(0.0, lo)
    if hi - lo < min_span:
        mid = 0.5 * (hi + lo)
        lo, hi = (min(lo, 0.0), max(hi, min_span)) if floor0 else (mid - 0.5 * min_span, mid + 0.5 * min_span)
    pad = 0.06 * (hi - lo)
    return (lo if floor0 and lo == 0.0 else lo - pad), hi + pad


# --------------------------------------------------------------------------- #
# metrics (pure)
# --------------------------------------------------------------------------- #
def touchdowns(log: sv.SimLog) -> list[dict[str, Any]]:
    """Touchdown events of the log with the values logged in their text (v_z, horizontal speed, tilt)."""
    out = []
    for t, x in zip(log.event_t, log.event_text):
        m = TD_RE.match(str(x))
        if m:
            out.append({"i": int(m.group(1)), "t": float(t), "vz": float(m.group(2)), "vh": float(m.group(3)),
                        "tilt": None if m.group(4) is None else float(m.group(4)), "text": str(x)})
    return out


def _rms(a: np.ndarray) -> float | None:
    a = np.asarray(a, dtype=float)
    return float(np.sqrt(np.mean(np.square(a)))) if a.size else None


def reason_names(codes: Sequence[int]) -> list[str]:
    """Korean names of TELEM reason codes (sim.ground_station_logic.REASON_TEXT_KO, else link_proto.REASON_NAMES,
    else the number)."""
    names: dict[int, str] = {}
    try:
        if PROJECT_ROOT not in sys.path:
            sys.path.insert(0, PROJECT_ROOT)
        from sim import link_proto as lp  # read-only use
        names.update({int(k): str(v) for k, v in lp.REASON_NAMES.items()})
        from sim.ground_station_logic import REASON_TEXT_KO
        for k, v in REASON_TEXT_KO.items():
            code = str(lp.REASON_NAMES.get(k, "")).split(" ")[0]          # "V10", "I2", ... (the rule's name)
            names[int(k)] = f"{code} {v}" if code and code[0] in "IV" else str(v)
    except Exception:  # noqa: BLE001 - the sim package may be mid-edit by another workflow; the codes stay correct
        pass
    return [names.get(int(c), f"reason {int(c)}") for c in codes]


def extract_metrics(fl: Flight) -> dict[str, Any]:
    """Metrics computed from one log (None where the log lacks the data).  Definitions follow
    ``sim.sim6dof.compute_vtol_metrics``: POWERED = state 2, HOLD = phase 3, first touchdown = the first
    'touchdown #1' event (its v_z / horizontal speed / tilt are the logged values of that event), distance = the
    exit-plane centre's horizontal distance from the pad at that time, drift = from the first sample's position."""
    log, ex = fl.log, fl.ex
    t = log.t
    P = log.state == sv.STATE_POWERED
    m: dict[str, Any] = {k: None for k in METRICS}
    tds = touchdowns(log)
    m["n_touchdowns"] = len(tds)
    td = tds[0] if tds else None
    if td is not None:
        m["td_vz"], m["td_vh"], m["td_tilt"] = td["vz"], td["vh"], td["tilt"]
        x = float(np.interp(td["t"], t, log.pos[:, 0]))
        y = float(np.interp(td["t"], t, log.pos[:, 1]))
        m["td_dist"] = math.hypot(x, y)
        if m["td_tilt"] is None:
            m["td_tilt"] = float(np.interp(td["t"], t, log.tilt_deg))
    if "tilt_est_err_deg" in ex and P.any():
        e = ex["tilt_est_err_deg"].astype(float)[P]
        m["est_err_rms"], m["est_err_max"] = _rms(e), float(e.max())
    hold = log.phase == PHASE_HOLD
    if "h_true" in ex and hold.any():
        h = ex["h_true"].astype(float)
        ok = hold & np.isfinite(log.h_ref) & np.isfinite(h)
        m["hold_rms"] = _rms(h[ok] - log.h_ref[ok]) if ok.any() else None
    il = np.flatnonzero(log.phase == PHASE_LAND)
    if td is not None and il.size:
        m["t_land"] = td["t"] - float(t[il[0]])
    ip = np.flatnonzero(P)
    ild = np.flatnonzero(log.phase == PHASE_LANDED)
    if ip.size and ild.size:
        m["t_flight"] = float(t[ild[0]]) - float(t[ip[0]])
    m["drift_max"] = float(np.max(np.hypot(log.pos[:, 0] - log.pos[0, 0], log.pos[:, 1] - log.pos[0, 1])))
    if P.any():
        m["max_tilt"] = float(log.tilt_deg[P].max())
    if "reason" in ex:
        codes = sorted({int(c) for c in np.unique(ex["reason"]) if int(c) != 0})
        m["reasons"] = reason_names(codes)
    if ild.size and "vc_td_by" in ex:
        by = int(ex["vc_td_by"][max(int(ild[0]) - 1, 0)])
        if by:
            m["landed_by"] = LANDED_BY_KO.get(by, str(by))
        elif "reason" in ex and int(ex["reason"][ild[0]]):
            m["landed_by"] = reason_names([int(ex["reason"][ild[0]])])[0]
    # v0.8 station keeping: truth CG against the hold point (logged, else derived by the v0.8 rule: hold_reference)
    hd = fl.hold_dist()
    if hd is not None:
        ok = hold & np.isfinite(hd)
        if ok.any():
            m["sk_rms"], m["sk_max"] = _rms(hd[ok]), float(hd[ok].max())
        if td is not None:
            v = float(np.interp(td["t"], t, np.where(np.isfinite(hd), hd, np.nan)))
            m["td_hold"] = v if math.isfinite(v) else None
    if log.gps_ok is not None and P.any():
        dt = np.diff(np.append(t, t[-1]))
        m["gps_avail"] = 100.0 * float(np.sum(dt[P & log.gps_ok]) / max(np.sum(dt[P]), 1e-12))
    return m


def metrics_from_json(ref: str, base_dir: str = ".") -> dict[str, Any]:
    """``path#scenario`` of a vtol_metrics.json -> this tool's metric keys (missing -> absent)."""
    path, _, scen = str(ref).partition("#")
    if not scen:
        raise ValueError(f"metrics_json reference {ref!r} needs '#scenario'")
    p = path if os.path.isabs(path) else os.path.join(base_dir, path)
    with open(p, "r", encoding="utf-8") as f:
        d = json.load(f)
    entry = (d.get("scenarios") or {}).get(scen)
    if entry is None:
        raise ValueError(f"scenario {scen!r} not in {p}")
    mj = entry.get("metrics", entry)
    out: dict[str, Any] = {}
    for k, jk in JSON_KEYS.items():
        if mj.get(jk) is not None:
            out[k] = mj[jk]
    if mj.get("touchdown_t_s") is not None and mj.get("t_land_s") is not None:
        out["t_land"] = float(mj["touchdown_t_s"]) - float(mj["t_land_s"])
    if mj.get("t_landed_s") is not None and mj.get("t_motor_start_s") is not None:
        out["t_flight"] = float(mj["t_landed_s"]) - float(mj["t_motor_start_s"])
    if "landed_by" in out:
        out["landed_by"] = LANDED_BY_KO.get(out["landed_by"], out["landed_by"])
    if "reasons" in out and isinstance(out["reasons"], list):
        out["reasons"] = [str(x) for x in out["reasons"]]
    return out


def combine_metrics(keys: Sequence[str], computed: dict[str, Any], from_json: dict[str, Any] | None,
                    from_spec: dict[str, Any] | None) -> tuple[dict[str, Any], dict[str, str], list[str]]:
    """Value + source per metric: spec ('spec') > computed from the log ('log') > metrics JSON ('json') > None;
    a computed value that disagrees with the JSON value beyond ``CROSSCHECK_TOL`` gives a warning."""
    vals: dict[str, Any] = {}
    src: dict[str, str] = {}
    warns: list[str] = []
    fj = from_json or {}
    fs = from_spec or {}
    for k in keys:
        c, j = computed.get(k), fj.get(k)
        if isinstance(c, (int, float)) and isinstance(j, (int, float)) and not isinstance(c, bool):
            tol = max(CROSSCHECK_TOL["rel"] * abs(float(j)), CROSSCHECK_TOL["abs"].get(k, 0.01))
            if abs(float(c) - float(j)) > tol:
                warns.append(f"{k}: computed {float(c):.4g} vs metrics JSON {float(j):.4g}")
        if k in fs:
            vals[k], src[k] = fs[k], "spec"
        elif c is not None:
            vals[k], src[k] = c, "log"
        elif j is not None:
            vals[k], src[k] = j, "json"
        else:
            vals[k], src[k] = None, "none"
    return vals, src, warns


def fmt_metric(key: str, v: Any) -> str:
    md = METRICS[key]
    if v is None:
        return "기록 없음"
    if isinstance(v, (list, tuple)):
        return ", ".join(str(x) for x in v) if v else "없음"
    if isinstance(v, str):
        return v
    if md.fmt == "d":
        return f"{int(v)} {md.unit}".strip()
    s = format(float(v), md.fmt)
    return f"{s} {md.unit}".strip() if md.unit not in ("°",) else f"{s}{md.unit}"


def metric_change(key: str, vb: Any, va: Any) -> tuple[str, str]:
    """(text, tone) of after - before: tone 'better' | 'worse' | 'same' | 'na' (no direction or no numbers)."""
    md = METRICS[key]
    num = (int, float)
    if not (isinstance(vb, num) and isinstance(va, num)) or isinstance(vb, bool) or isinstance(va, bool):
        if vb is None or va is None:
            return "비교 불가", "na"
        return ("같음", "same") if vb == va else ("다름", "na")
    d = float(va) - float(vb)
    ref = abs(float(vb))
    if md.fmt == "d":
        txt = f"{int(round(d)):+d} {md.unit}"
    else:
        dfmt = md.fmt.lstrip("+")
        txt = format(d, "+" + dfmt) + ("" if md.unit == "°" else " ") + md.unit
    if md.better == "lower_abs":
        a, b = abs(float(va)), abs(float(vb))
    else:
        a, b = float(va), float(vb)
    if ref > 1e-9 and md.better is not None:
        rel = (a - b) / max(abs(b), 1e-12) * 100.0
        txt += f"  ({rel:+.0f} %)"

    if md.better is None:
        return txt, "na"
    eps = max(1e-9, 0.005 * max(abs(a), abs(b)))
    if a < b - eps:
        return txt, "better"
    if a > b + eps:
        return txt, "worse"
    return txt, "same"


def criterion_limit(key: str, metas: Sequence[dict[str, Any]]) -> float | None:
    """The limit of a metric from the logs' meta.criteria (a number, or {"field": .., "demo": ..} by the log's
    profile - the v0.8 criteria are per profile)."""
    ck = METRICS[key].criterion
    if not ck:
        return None
    for m in metas:
        c = sv.criterion_value((m.get("criteria") or {}).get(ck), m.get("profile"))
        if c is not None:
            return float(c)
    return None


def isolation_warnings(spec_iso: str, metas: Sequence[dict[str, Any]]) -> list[str]:
    """A claimed 'isolated' comparison whose logs differ in params_sha256 / seed / scenario base is suspicious."""
    if spec_iso != "isolated" or len(metas) != 2:
        return []
    a, b = metas
    out = []
    if a.get("params_sha256") != b.get("params_sha256"):
        out.append("spec says isolated, but the logs' params_sha256 differ (parameters differ)")
    if a.get("seed") != b.get("seed"):
        out.append("spec says isolated, but the logs' seeds differ")
    if a.get("profile") != b.get("profile"):
        out.append("spec says isolated, but the logs' profiles differ")
    return out


# --------------------------------------------------------------------------- #
# camera (pure)
# --------------------------------------------------------------------------- #
def shared_distance(dists: Sequence[np.ndarray], fps_eff: float, smooth_s: float = 1.0) -> np.ndarray:
    """One camera distance per frame for both sides: the larger of the two (each already smooth), a 1 s running
    maximum and the zero-phase smoothing ``sim_video.plan_camera`` uses, never below that larger distance (the
    smoothing's end reflection could otherwise cut a side's framing near the first / last frames)."""
    d = np.max(np.vstack([np.asarray(x, dtype=float) for x in dists]), axis=0)
    win = max(1, int(round(1.0 * fps_eff)))
    return np.maximum(sv._smooth(sv._running_max(d, win), smooth_s * fps_eff), d)


def reaim(focal: np.ndarray, d_old: np.ndarray, d_new: np.ndarray, azimuth_deg: np.ndarray, elevation_deg: float,
          view_angle_deg: float, aspect: float, box: sv.NDC) -> np.ndarray:
    """Focal points that keep each frame's framed set at the same place in ``box`` when the distance changes:
    ``plan_camera`` aims c - R bcx d th_h - U bcy d th_v, so focal' = focal + (d - d') (R bcx th_h + U bcy th_v)."""
    th_v = math.tan(math.radians(view_angle_deg) / 2.0)
    th_h = th_v * aspect
    bcx, bcy = 0.5 * (box[0] + box[1]), 0.5 * (box[2] + box[3])
    out = np.array(focal, dtype=float, copy=True)
    for k in range(out.shape[0]):
        _, r, u = sv.camera_basis(float(azimuth_deg[k]), elevation_deg)
        out[k] += (float(d_old[k]) - float(d_new[k])) * (r * bcx * th_h + u * bcy * th_v)
    return out


def pair_azimuth(logs: Sequence[sv.SimLog]) -> float:
    """Camera look direction across the drift of the log that drifts farther (``sim_video.auto_azimuth``)."""
    far = max(logs, key=lambda lg: float(np.max(np.hypot(lg.pos[:, 0], lg.pos[:, 1]))))
    return sv.auto_azimuth(far)


def in_view_fraction(fr: sv.Frames, plan: sv.CameraPlan, length_m: float, aspect: float) -> float:
    vp = sv.vehicle_points(fr, length_m)
    ok = 0
    for k in range(fr.n):
        q, z = sv._ndc(vp[k], plan.view(k), aspect)
        ok += int(float(z.min()) > 0.0 and float(np.max(np.abs(q))) <= 1.0)
    return ok / max(1, fr.n)


class TopView:
    """Top-view inset of one side (PIL): distance rings around the pad at ONE scale for both sides (``r_max_m`` =
    the larger drift of the two logs), this log's track so far in its colour, the current position and its distance
    from the pad, and the main camera's look direction."""

    def __init__(self, r_max_m: float, size_px: int, scale: float, cam_az_deg: float, color: str, label: str = ""):
        from PIL import Image, ImageDraw
        self.sz, self.s = int(size_px), float(scale)
        s = self.s
        self.hh = int(48 * s)
        self.W, self.H = self.sz, self.sz + self.hh
        self.R = sv.nice_step(max(2.0, 1.08 * float(r_max_m)))
        self.step = sv.nice_step(self.R / 4.0)
        self.k = (self.sz / 2.0 - 8 * s) / self.R
        self.c = np.array([self.sz / 2.0, self.hh + self.sz / 2.0])
        self.col = sv.hex_rgb(color)
        bg = Image.new("RGBA", (self.W, self.H), (0, 0, 0, 0))
        d = ImageDraw.Draw(bg)
        d.rounded_rectangle([0, 0, self.W - 1, self.H - 1], radius=int(8 * s), fill=(250, 250, 248, 215),
                            outline=self.col + (255,), width=max(1, int(2 * s)))
        fs = F(12 * s)
        dtext(d, (8 * s, 7 * s), "위에서 본 이동 (같은 축척)", fill=(30, 30, 30, 255), font=F(13 * s, True))
        cx, cy = self.c
        r = self.step
        while r <= self.R + 1e-9:
            rr = r * self.k
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=sv.hex_rgb(sv.RING) + (150,), width=1)
            lab = f"{r:g} m"
            dtext(d, (cx + rr * 0.7071 + 2 * s, cy - rr * 0.7071 - 14 * s), lab, fill=sv.hex_rgb(sv.RING) + (255,), font=fs)
            r += self.step
        ps = 4 * s
        d.rectangle([cx - ps, cy - ps, cx + ps, cy + ps], fill=(110, 105, 98, 255))
        dtext(d, (cx - 10 * s, cy + 6 * s), "패드", fill=(90, 90, 90, 255), font=fs)
        a = math.radians(cam_az_deg)
        e = np.array([cx - math.cos(a) * (self.sz / 2 - 16 * s), cy + math.sin(a) * (self.sz / 2 - 16 * s)])
        tip = e + np.array([math.cos(a), -math.sin(a)]) * 12 * s
        d.line([tuple(e), tuple(tip)], fill=(90, 95, 102, 255), width=max(1, int(2 * s)))
        dtext(d, (e[0] - 12 * s, e[1] + 2 * s), "카메라", fill=(90, 95, 102, 255), font=F(10 * s))
        self.bg = bg

    def px(self, xy: np.ndarray) -> tuple[float, float]:
        return float(self.c[0] + float(xy[0]) * self.k), float(self.c[1] - float(xy[1]) * self.k)

    def _hold_mark(self, d: Any, hold: np.ndarray, col: tuple[int, int, int], to: tuple[float, float] | None) -> None:
        """The hold point (ring + cross, at least 5 px) and a thin line to the vehicle."""
        s = self.s
        hp = self.px(hold)
        hr = max(5 * s, sv.HOLD_RING_M * self.k)
        hc = max(8 * s, sv.HOLD_CROSS_M * self.k)
        if to is not None:
            d.line([hp, to], fill=col + (255,), width=max(1, int(1.4 * s)))
        d.ellipse([hp[0] - hr, hp[1] - hr, hp[0] + hr, hp[1] + hr], outline=col + (255,), width=max(1, int(1.6 * s)))
        d.line([(hp[0] - hc, hp[1]), (hp[0] + hc, hp[1])], fill=col + (255,), width=max(1, int(1.2 * s)))
        d.line([(hp[0], hp[1] - hc), (hp[0], hp[1] + hc)], fill=col + (255,), width=max(1, int(1.2 * s)))

    def _track(self, d: Any, xy_past: np.ndarray, col: tuple[int, int, int], clip: bool = False,
               name: str = "") -> tuple[float, float] | None:
        """The track and the current dot.  ``clip``: only the parts within the map radius; a current position beyond
        it is drawn as an arrow on the map edge with ``name`` and its distance from the pad."""
        s = self.s
        xy = np.asarray(xy_past, dtype=float)
        if xy.shape[0] > 1:
            step = max(1, xy.shape[0] // 500)
            sub = np.vstack([xy[::step], xy[-1:]])
            inside = np.hypot(sub[:, 0], sub[:, 1]) <= 1.04 * self.R if clip else np.ones(sub.shape[0], dtype=bool)
            run: list[tuple[float, float]] = []
            for p, ok in zip(sub, inside):
                if ok:
                    run.append(self.px(p))
                    continue
                if len(run) > 1:
                    d.line(run, fill=col + (255,), width=max(2, int(2.5 * s)))
                run = []
            if len(run) > 1:
                d.line(run, fill=col + (255,), width=max(2, int(2.5 * s)))
        if not xy.shape[0]:
            return None
        cur = xy[-1]
        r_cur = float(np.hypot(*cur))
        if clip and r_cur > 1.04 * self.R:
            u = cur / r_cur
            tip = self.c + np.array([u[0], -u[1]]) * (self.sz / 2.0 - 6 * s)
            v = np.array([u[0], -u[1]])
            nrm = np.array([-v[1], v[0]])
            hl = 11 * s
            d.polygon([tuple(tip), tuple(tip - v * hl + nrm * hl * 0.6), tuple(tip - v * hl - nrm * hl * 0.6)],
                      fill=col + (255,))
            lab = f"{name} {r_cur:.0f} m →" if name else f"{r_cur:.0f} m"
            fl = F(12 * s, True)
            tx = float(np.clip(tip[0] - v[0] * 18 * s - dlen(d, lab, fl) / 2.0, 4 * s, self.W - dlen(d, lab, fl) - 4 * s))
            ty = float(np.clip(tip[1] - v[1] * 18 * s - 8 * s, self.hh + 2 * s, self.H - 18 * s))
            dtext(d, (tx, ty), lab, fill=col + (255,), font=fl, stroke_width=max(1, int(2 * s)), stroke_fill=(255, 255, 255, 230))
            return None
        x, y = self.px(cur)
        r = 5 * s
        d.ellipse([x - r, y - r, x + r, y + r], fill=col + (255,), outline=(255, 255, 255, 255), width=max(1, int(s)))
        return x, y

    def frame(self, xy_past: np.ndarray, hold: np.ndarray | None = None, hold_d: float | None = None,
              others: Sequence[tuple[np.ndarray, tuple[int, int, int], str]] = ()) -> Any:
        """This log's track so far and its distance from the pad; v0.8: ``hold`` (its hold point xy, NaN / None =
        none) with ``hold_d`` (the CG's distance from it) and ``others`` = further (track, colour, name) drawn under
        it, clipped to the map (the shared layout's ghost: beyond the map an edge arrow with its distance)."""
        from PIL import ImageDraw
        im = self.bg.copy()
        d = ImageDraw.Draw(im)
        s = self.s
        for xy_o, col_o, name_o in others:
            self._track(d, xy_o, col_o, clip=True, name=name_o)
        cur = self._track(d, xy_past, self.col)
        if hold is not None and bool(np.all(np.isfinite(hold))):
            self._hold_mark(d, hold, sv.hex_rgb(sv.HOLD_C), cur)
        if cur is not None:
            dist = float(np.hypot(*xy_past[-1]))
            txt = f"{dist:.1f} m"
            ft = F(14 * s, True)
            txt = f"패드에서 {txt}"
            if hold_d is not None and math.isfinite(hold_d):
                txt = f"패드 {dist:.1f} m · 유지점 {hold_d:.2f} m" if hold_d < 10.0 else \
                    f"패드 {dist:.1f} m · 유지점 {hold_d:.1f} m"
                while dlen(d, txt, ft) > self.W - 12 * s and ft.size > 9:
                    ft = F(ft.size - 1, True)
            dtext(d, (8 * s, 25 * s), txt, fill=self.col + (255,), font=ft)
        return im


@contextlib.contextmanager
def shared_rings(radii: Sequence[float]):
    """While building the scenes, ``sim_video.Scene._ring_radii`` returns ``radii`` (identical rings on both sides);
    the original is restored afterwards.  sim_video.py itself is not modified."""
    orig = sv.Scene.__dict__["_ring_radii"]
    sv.Scene._ring_radii = staticmethod(lambda fr: list(radii))
    try:
        yield
    finally:
        sv.Scene._ring_radii = orig


# --------------------------------------------------------------------------- #
# drawing helpers
# --------------------------------------------------------------------------- #
def F(size: float, bold: bool = False, text: str = "") -> Any:
    return sv.font(max(6, int(round(size))), bold=bold, text=text, lang="ko")


# characters Malgun Gothic has no glyph for (measured with fontTools on malgun.ttf / malgunbd.ttf, 2026-10-21)
GLYPH_FALLBACK = {"−": "-", "≈": "~", "✓": "v", "✗": "x"}


def safe(text: Any) -> str:
    """Text as drawn with PIL: characters the Korean font lacks replaced (U+2212 minus -> '-', ...)."""
    return "".join(GLYPH_FALLBACK.get(ch, ch) for ch in str(text))


def dtext(d: Any, xy: tuple[float, float], text: Any, **kw: Any) -> None:
    d.text(xy, safe(text), **kw)


def dlen(d: Any, text: Any, font: Any) -> float:
    return float(d.textlength(safe(text), font=font))


def pill(d: Any, xy: tuple[float, float], text: str, fnt: Any, fill: tuple[int, ...], fg: tuple[int, ...],
         pad: tuple[float, float] = (12, 6), anchor_right: bool = False, radius: float = 8) -> tuple[int, int, int, int]:
    """Rounded label; returns its box.  ``anchor_right``: xy is the top-right corner."""
    tw = dlen(d, text, font=fnt)
    asc, desc = fnt.getmetrics()
    h = asc + desc
    x, y = xy
    if anchor_right:
        x = x - tw - 2 * pad[0]
    box = (int(x), int(y), int(x + tw + 2 * pad[0]), int(y + h + 2 * pad[1]))
    d.rounded_rectangle(box, radius=int(radius), fill=fill)
    dtext(d, (x + pad[0], y + pad[1]), text, fill=fg, font=fnt)
    return box


def draw_sim_tag(d: Any, right: float, top: float, s: float) -> tuple[int, int, int, int]:
    return pill(d, (right, top), SIM_TAG, F(17 * s, True, SIM_TAG), (200, 40, 40, 235), (255, 255, 255, 255),
                pad=(12 * s, 5 * s), anchor_right=True, radius=6 * s)


def draw_wrapped(d: Any, xy: tuple[float, float], text: str, fnt: Any, fill: Any, max_w: float,
                 line_h: float, max_lines: int = 99) -> float:
    """Word-wrapped text; returns the y below it."""
    x, y = xy
    n = 0
    for para in str(text).split("\n"):
        for ln in sv.wrap_px(d, safe(para), fnt, max_w):
            if n >= max_lines:
                return y
            dtext(d, (x, y), ln, fill=fill, font=fnt)
            y += line_h
            n += 1
    return y


# --------------------------------------------------------------------------- #
# the plot strip
# --------------------------------------------------------------------------- #
class CompareStrip:
    """Matplotlib plots drawn once (before grey dashed, after blue; phase bands; touchdown marks); per frame a
    cursor, the future dimmed and a dot per log on its curves."""

    def __init__(self, flights: Sequence[Flight], offsets: Sequence[float], labels: Sequence[str],
                 channels: Sequence[str], tau0: float, tau1: float, width: int, height: int, scale: float,
                 align_label: str):
        import matplotlib
        matplotlib.use("Agg")
        fam = sv.mpl_korean_family()
        rc = {"font.family": [fam, "DejaVu Sans"], "axes.unicode_minus": False} if fam else {}
        with matplotlib.rc_context(rc):
            self._build(flights, offsets, labels, channels, tau0, tau1, width, height, scale, align_label)

    def _build(self, flights, offsets, labels, channels, tau0, tau1, width, height, s, align_label) -> None:
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        import matplotlib.transforms as mtrans
        self.w, self.h, self.s = int(width), int(height), float(s)
        self.tau0, self.tau1 = float(tau0), float(tau1)
        data = [c for c in channels if c in DATA_CHANNELS] or ["alt"]
        show_phase = "phase" in channels and any(f.log.has_phase for f in flights)
        show_td = "touchdown" in channels
        ncols = len(data) if len(data) <= 4 else 3
        nrows = int(math.ceil(len(data) / ncols))
        dpi = 100.0
        fig = Figure(figsize=(self.w / dpi, self.h / dpi), dpi=dpi, facecolor=sv.SURFACE)
        FigureCanvasAgg(fig)
        fs = 12.5 * s
        top = 1.0 - 62.0 * s / self.h
        bottom = 66.0 * s / self.h
        outer = fig.add_gridspec(nrows, ncols, left=0.04, right=0.99, top=top, bottom=bottom, wspace=0.17,
                                 hspace=0.55 if nrows > 1 else 0.3)
        self.axes: list[tuple[Any, str, list[tuple[Series, str, int]]]] = []   # (ax, channel, [(series, colour, side)])
        self.phase_axes: list[Any] = []
        self.ax_notes: list[tuple[Any, Any, str, str]] = []     # (ax, where, text, colour): drawn per frame, not dimmed
        cols = (BEFORE_C, AFTER_C)
        est_cols = (BEFORE_EST_C, EST_C)
        series_all = {c: [channel_series(f, c, o) for f, o in zip(flights, offsets)] for c in data}
        tds = [[td["t"] - o for td in touchdowns(f.log)] for f, o in zip(flights, offsets)]
        self.missing: list[str] = []
        for j, c in enumerate(data):
            r, cc = divmod(j, ncols)
            if show_phase:
                sub = outer[r, cc].subgridspec(2, 1, height_ratios=[1.0, 0.17], hspace=0.06)
                ax = fig.add_subplot(sub[0])
                axp = fig.add_subplot(sub[1], sharex=ax)
                self.phase_axes.append(axp)
            else:
                ax = fig.add_subplot(outer[r, cc])
                axp = None
            dots: list[tuple[Series, str, int]] = []
            ys: list[np.ndarray] = []
            for side in (0, 1):                     # before first, after on top
                ser = series_all[c][side]
                if not ser and c in MISSING_NOTE:
                    self.missing.append(f"{labels[side]}: {MISSING_NOTE[c]}")
                for sr in ser:
                    sel = (sr.tau >= self.tau0 - 1.0) & (sr.tau <= self.tau1 + 1.0)
                    x, y = sr.tau[sel], np.asarray(sr.y, dtype=float)[sel]
                    if x.size > 4000:                  # draw at most ~4000 points per series (250 Hz logs)
                        k = int(math.ceil(x.size / 4000))
                        x, y = x[::k], y[::k]
                    if sr.role == "target":
                        ax.step(x, y, where="post", color=BEFORE_EST_C if side == 0 else TARGET_C, lw=1.1 * s,
                                ls=(0, (2, 2)) if side == 0 else "-", zorder=2 + side)
                        continue
                    col = (est_cols if sr.role == "est" else cols)[side]
                    ls = ((0, (1.2, 1.6)) if sr.role == "est" else ((0, (5, 3)) if side == 0 else "-"))
                    lw = (1.4 if sr.role == "est" else (1.7 if side == 0 else 2.1)) * s
                    if sr.step:                        # a discrete value (GPS satellites): hold, no slope
                        ax.step(x, y, where="post", color=col, lw=lw, ls=ls, zorder=3 + side + 0.5)
                    else:
                        ax.plot(x, y, color=col, lw=lw, ls=ls, zorder=3 + side + (0.5 if sr.role == "main" else 0))
                    ys.append(y)
                    dots.append((sr, col, side))
                if c == "gps" and flights[side].log.gps_ok is not None:
                    # GPS not usable while powered: a light red band on this log's half of the plot (before top)
                    lg = flights[side].log
                    bad = (~lg.gps_ok) & (lg.state == sv.STATE_POWERED)
                    y0b, y1b = (0.5, 1.0) if side == 0 else (0.0, 0.5)
                    for a_, b_, v_ in phase_spans(bad.astype(np.int8), lg.t - offsets[side]):
                        if v_ and b_ >= self.tau0 and a_ <= self.tau1:
                            ax.axvspan(max(a_, self.tau0), min(b_, self.tau1), ymin=y0b, ymax=y1b, color=sv.GUST_C,
                                       alpha=0.18, lw=0, zorder=1)
            lo, hi = series_limits(ys or [np.zeros(1)], floor0=c in ("tilt", "est_err", "dist", "throttle", "hold_dist",
                                                                     "hspeed", "gps"),
                                   min_span={"throttle": 0.2, "vz": 1.0, "gps": 6.0}.get(c, 1.0))
            if c == "throttle":
                lo, hi = min(lo, -0.02), max(hi, 1.05)
            if c == "hold_dist":
                lim = criterion_limit("sk_max", [f.log.meta for f in flights[::-1]])
                # a log that drifts away (v0.7.1: tens of metres) would flatten the station-keeping curve: the axis is
                # cut at the other log's scale and the cut-off log's maximum is written on the plot
                mx = [max([float(np.nanmax(sr.y)) for sr in series_all[c][sd_] if np.any(np.isfinite(sr.y))] or [np.nan])
                      for sd_ in (0, 1)]
                if all(math.isfinite(v) for v in mx):
                    small, big = min(mx), max(mx)
                    if big > 5.0 * max(small, 0.2) and big > 2.0 * (lim or 1.0):
                        hi = max(1.35 * small, 1.25 * lim if lim is not None else 0.0, 2.0)
                        sb = int(np.argmax(mx))
                        # drawn per frame (``frame``): above the future-dimming
                        self.ax_notes.append((ax, "tr", f"{labels[sb]}: 최대 {big:.1f} m (축 위로 벗어남 ↑)", cols[sb]))
                if lim is not None and lim <= 3.0 * max(hi, 0.5):           # not when it would flatten the curves
                    ax.axhline(lim, color=sv.GUST_C, lw=1.0 * s, ls="--", zorder=2)
                    self.ax_notes.append((ax, ("lim", lim), f"기준 {lim:g} m", sv.GUST_C))
                    hi = max(hi, 1.12 * lim)
            ax.set_ylim(lo, hi)
            ax.set_xlim(self.tau0, self.tau1)
            ax.axvline(0.0, color=sv.MUTED, lw=0.8 * s, ls=":", zorder=1)
            if show_td:
                for side in (0, 1):
                    for k_, tt in enumerate(tds[side]):
                        if self.tau0 <= tt <= self.tau1:
                            ax.axvline(tt, color=cols[side], lw=0.9 * s, ls=(0, (3, 2)) if side == 0 else "-",
                                       alpha=0.75, zorder=1)
                            tr_ = mtrans.blended_transform_factory(ax.transData, ax.transAxes)
                            ax.plot([tt], [1.0], marker="v", ms=7 * s, color=cols[side], transform=tr_, clip_on=False,
                                    zorder=6, mec="white", mew=0.6 * s)
            ax.set_title(CHANNEL_TITLE[c], loc="left", fontsize=fs, color=sv.INK, pad=4 * s, weight="bold")
            self._style(ax, fs)
            if axp is not None or r < nrows - 1:
                ax.tick_params(labelbottom=False)
            if axp is not None:
                for side, f in enumerate(flights):
                    ph = f.log.phase
                    tau = f.log.t - offsets[side]
                    y0, y1 = (0.53, 0.97) if side == 0 else (0.03, 0.47)
                    for a, b, code in phase_spans(ph, tau):
                        if b < self.tau0 or a > self.tau1:
                            continue
                        axp.axvspan(max(a, self.tau0), min(b, self.tau1), ymin=y0, ymax=y1,
                                    color=sv.PHASE_COLORS.get(code, "#f3f4f6"), lw=0)
                    axp.axvline(f.t[-1] - offsets[side], ymin=y0, ymax=y1, color=cols[side], lw=1.6 * s)
                axp.set_ylim(0, 1)
                axp.set_yticks([0.75, 0.25])
                axp.set_yticklabels(["전", "후"], fontsize=fs * 0.75)
                for tl, col in zip(axp.get_yticklabels(), cols):
                    tl.set_color(col)
                axp.tick_params(axis="y", length=0, pad=2)
                self._style(axp, fs, grid=False)
                axp.set_xlim(self.tau0, self.tau1)
                if r < nrows - 1:
                    axp.tick_params(labelbottom=False)
            self.axes.append((ax, c, dots))
        # legend (one for the strip, top left)
        handles = [Line2D([], [], color=BEFORE_C, lw=1.8 * s, ls=(0, (5, 3)), label=labels[0]),
                   Line2D([], [], color=AFTER_C, lw=2.2 * s, label=labels[1])]
        if "alt" in data:
            handles.append(Line2D([], [], color=TARGET_C, lw=1.1 * s, label="목표 고도 (계단)"))
        if "tilt" in data and any(sr.role == "est" for side in series_all["tilt"] for sr in side):
            handles.append(Line2D([], [], color=EST_C, lw=1.5 * s, ls=(0, (1.2, 1.6)), label="추정 기울기 (점선)"))
        if show_td and any(tds):
            handles.append(Line2D([], [], color=sv.INK2, marker="v", ls="none", ms=7 * s, label="착지 순간"))
        if show_phase:
            seen = sorted({int(c) for f in flights for c in np.unique(f.log.phase)} - {0})
            handles += [Patch(facecolor=sv.PHASE_COLORS.get(c, "#f3f4f6"), edgecolor=sv.GRID,
                              label=sv.PHASE_NAMES_KO.get(c, str(c))) for c in seen]
        lg = fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(0.035, 1.0 - 4.0 * s / self.h), ncol=len(handles),
                        frameon=False, fontsize=fs * 0.88, handlelength=2.2, columnspacing=1.3, handletextpad=0.5)
        for txt in lg.get_texts():
            txt.set_color(sv.INK2)
        fig.text(0.5, 6.0 * s / self.h, f"정렬 시간 τ [s]   (τ = 0: {align_label})", ha="center", va="bottom",
                 fontsize=fs * 0.95, color=sv.INK2)
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        self.bg = np.full((self.h, self.w, 3), sv.hex_rgb(sv.SURFACE), dtype=np.uint8)
        hh, ww = min(self.h, buf.shape[0]), min(self.w, buf.shape[1])
        self.bg[:hh, :ww] = buf[:hh, :ww]
        self.boxes: list[tuple[int, int, int, int, float, float, list[tuple[Series, str, int]]]] = []
        for ax, _, dots in self.axes:
            self.boxes.append(self._box(ax) + (dots,))
        self.pboxes = [self._box(a) for a in self.phase_axes]
        self.channels = data
        self.notes_px: list[tuple[float, float, str, str, str]] = []     # (x, top y, text, colour, 'r' | 'l' anchor)
        for ax, where, txt, col in self.ax_notes:
            x0, x1, r0, r1, lo, hi = self._box(ax)
            if where == "tr":                                  # top right
                self.notes_px.append((x1 - 4 * s, r0 + 2 * s, txt, col, "r"))
            else:                                              # a limit line: its label above the line, at the left
                v = float(where[1])
                yv = r0 + (hi - v) / (hi - lo) * (r1 - r0)
                self.notes_px.append((x0 + 6 * s, yv - 18 * s, txt, col, "l"))

    def _box(self, ax: Any) -> tuple[int, int, int, int, float, float]:
        bb = ax.get_window_extent()
        y0, y1 = ax.get_ylim()
        return (int(round(bb.x0)), int(round(bb.x1)), int(round(self.h - bb.y1)), int(round(self.h - bb.y0)),
                float(y0), float(y1))

    @staticmethod
    def _style(ax: Any, fs: float, grid: bool = True) -> None:
        ax.set_facecolor(sv.SURFACE)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(sv.GRID)
        if grid:
            ax.grid(True, color=sv.GRID, lw=0.6)
        ax.tick_params(colors=sv.INK2, labelsize=fs * 0.82)

    def x_px(self, box: tuple, tau: float) -> float:
        x0, x1 = box[0], box[1]
        return x0 + (min(max(tau, self.tau0), self.tau1) - self.tau0) / (self.tau1 - self.tau0) * (x1 - x0)

    def frame(self, tau: float) -> np.ndarray:
        from PIL import Image, ImageDraw
        img = self.bg.copy()
        all_boxes = [b[:6] for b in self.boxes] + list(self.pboxes)
        for b in all_boxes:
            x0, x1, r0, r1 = b[0], b[1], b[2], b[3]
            x = int(round(self.x_px(b, tau)))
            if x + 1 < x1:
                reg = img[r0:r1, x + 1:x1].astype(np.float32)
                img[r0:r1, x + 1:x1] = (reg * 0.42 + 255.0 * 0.58).astype(np.uint8)
            img[r0:r1, max(x0, x - 1):min(x1, x + 1) + 1] = sv.hex_rgb(sv.INK)
        im = Image.fromarray(img)
        d = ImageDraw.Draw(im)
        rad = max(3.0, 4.5 * self.s)
        for (x0, x1, r0, r1, lo, hi, dots) in self.boxes:
            for sr, col, _side in dots:
                if sr.tau.size == 0:
                    continue
                tc = min(max(tau, float(sr.tau[0])), float(sr.tau[-1]))
                v = value_at(sr, tc)
                if not math.isfinite(v):
                    continue
                xx = self.x_px((x0, x1), tc)
                yy = r0 + (hi - v) / (hi - lo) * (r1 - r0)
                yy = min(max(yy, r0), r1)
                d.ellipse([xx - rad, yy - rad, xx + rad, yy + rad], fill=col, outline=(255, 255, 255), width=max(1, int(self.s)))
        fn = F(13.5 * self.s, True)
        for xa, yt, nt, col, anchor in getattr(self, "notes_px", ()):
            tw = dlen(d, nt, fn)
            xl = xa - tw if anchor == "r" else xa
            d.rectangle([xl - 3 * self.s, yt, xl + tw + 3 * self.s, yt + 18 * self.s], fill=sv.hex_rgb(sv.SURFACE))
            dtext(d, (xl, yt), nt, fill=sv.hex_rgb(col), font=fn)
        txt = f"τ = {tau:+.2f} s"
        fb = F(19 * self.s, True, txt)
        dtext(d, (self.w - int(16 * self.s) - dlen(d, txt, font=fb), int(8 * self.s)), txt, fill=sv.hex_rgb(sv.INK), font=fb)
        return np.asarray(im)


# --------------------------------------------------------------------------- #
# the comparison renderer
# --------------------------------------------------------------------------- #
@dataclass
class CompareLayout:
    W: int
    H: int
    s: float
    banner_h: int
    view_y: int
    view_w: int
    view_h: int
    view_x: tuple[int, int]
    strip_y: int
    col_x: int = 0                  # shared layout: the right column (comparison table + top view)
    col_w: int = 0

    @classmethod
    def for_size(cls, W: int, H: int, shared: bool = False) -> "CompareLayout":
        """split: two 3-D views side by side; shared: ONE 3-D view (68 % of the width) + a right column."""
        W, H = int(W) // 2 * 2, int(H) // 2 * 2
        s = H / 1080.0
        bh = int(round(96 * s))
        m, g = int(round(16 * s)), int(round(16 * s))
        vh = int(round(556 * s)) // 2 * 2
        vy = bh + int(round(8 * s))
        if shared:
            vw = int(round((W - 2 * m - g) * 0.68)) // 2 * 2
            return cls(W, H, s, bh, vy, vw, vh, (m, m), vy + vh + int(round(8 * s)), m + vw + g, W - m - (m + vw + g))
        vw = (W - 2 * m - g) // 2 // 2 * 2
        return cls(W, H, s, bh, vy, vw, vh, (m, m + vw + g), vy + vh + int(round(8 * s)))


class _SideRenderer(sv.Renderer):
    """``sim_video.Renderer`` whose own camera plan is a placeholder: the comparison plans one camera for both sides
    (``CompareRenderer._build_scenes``) and replaces ``cam``; skipping the discarded plan saves ~5 s per side."""

    def _plan(self, framing: str, extra: np.ndarray | None, cam_kw: dict[str, Any]) -> sv.CameraPlan:
        g = self.grid
        return sv.CameraPlan(g.pos + np.array([0.0, 0.0, 0.4]), np.full(g.n, 10.0), np.asarray(self.az_grid, dtype=float),
                             self.elevation, 30.0, np.zeros(g.n, dtype=int))


class Side:
    """One 3-D view: a ``sim_video.Renderer`` (style as chosen, HUD drawn here) on the aligned frame times."""

    def __init__(self, fl: Flight, tq: np.ndarray, status: np.ndarray, kw: dict[str, Any]):
        self.fl, self.tq, self.status = fl, tq, status
        self.rnd = _SideRenderer(fl.log, float(fl.t[0]), float(fl.t[-1]), tq=tq, **kw)
        self._last_key: tuple | None = None
        self._last_img: np.ndarray | None = None
        self.reused = 0

    def render3d(self, k: int, cam: sv.CamView) -> np.ndarray:
        key = (round(float(self.tq[k]), 6), tuple(np.round(cam.position, 4)), tuple(np.round(cam.focal, 4)))
        if key == self._last_key and self._last_img is not None:
            self.reused += 1
            return self._last_img
        img = self.rnd._render_main(k, cam)
        self._last_key, self._last_img = key, img
        return img

    def close(self) -> None:
        self.rnd.close()


class SharedSide(Side):
    """The shared layout's one 3-D view: the AFTER log's renderer (vehicle, GPS pod, hold point, person, pushes) and
    the BEFORE vehicle as a translucent ``sim_video.GhostVehicle`` posed from the before log at its own aligned times
    (moved horizontally by ``shift_xy`` with ghost_align 'hold', else as flown), each with a label above it."""

    def __init__(self, after: Flight, before: Flight, tq_a: np.ndarray, st_a: np.ndarray, tq_b: np.ndarray,
                 st_b: np.ndarray, kw: dict[str, Any], shift_xy: np.ndarray, labels: Sequence[str]):
        super().__init__(after, tq_a, st_a, kw)
        fr_b = sv.resample(before.log, tq_b, window=self.rnd.frame_dt)
        sh = np.asarray(shift_xy, dtype=float)
        if np.any(sh != 0.0):
            fr_b.pos = fr_b.pos.copy()
            fr_b.pos[:, :2] += sh
            if fr_b.cg_xy is not None:
                fr_b.cg_xy = fr_b.cg_xy + sh
        self.ghost_fr, self.ghost_tq, self.ghost_status, self.shift_xy = fr_b, tq_b, st_b, sh
        sc = self.rnd.scene
        # label size: billboards scale with the window height (556 px here): 24 -> ~17 px text
        # the ghost's label sits higher than the after one: side by side (the start) they stack instead of overlapping
        # v0.9: the ghost draws the vehicle its own log flew (a legacy log names none: the scene's meshes, as before)
        self.ghost = sc.add_ghost(fr_b, label=labels[0], label_size=24, label_dz=0.42, hardware=before.log.hw,
                                  warnings=self.rnd.warnings)
        self.main_label = sc._billboard(labels[1], 24, "#ffffff", bg=AFTER_C, bg_opacity=0.85)

    def render3d(self, k: int, cam: sv.CamView) -> np.ndarray:
        key = (round(float(self.tq[k]), 6), round(float(self.ghost_tq[k]), 6), tuple(np.round(cam.position, 4)),
               tuple(np.round(cam.focal, 4)))
        if key == self._last_key and self._last_img is not None:
            self.reused += 1
            return self._last_img
        fr, sc = self.rnd.fr, self.rnd.scene
        self.main_label.SetPosition(*(fr.pos[k] + fr.R[k] @ np.array([0.0, 0.0, sc.length_m + 0.12])))
        img = self.rnd._render_main(k, cam)
        self._last_key, self._last_img = key, img
        return img


class CompareRenderer:
    def __init__(self, before: Flight, after: Flight, spec: CompareSpec, align: Align, channels: Sequence[str],
                 fps: float = 30.0, speed: float = 1.0, size: tuple[int, int] = (1920, 1080),
                 start: float | None = None, end: float | None = None, style: str = "realistic", scene: str = "auto",
                 framing: str | None = None, azimuth: float | None = None, elevation: float | None = None,
                 sync: str | None = None, aa: str = "msaa", asset_dir: str | None = sv.ASSET_DIR,
                 build_scenes: bool = True, layout: str | None = None, ghost_align: str | None = None):
        self.flights = (before, after)
        self.spec, self.align, self.fps, self.speed = spec, align, float(fps), float(speed)
        self.layout = layout or spec.layout
        if self.layout not in LAYOUTS:
            raise ValueError(f"layout must be one of {LAYOUTS}")
        self.ghost_align = ghost_align or spec.ghost_align
        if self.ghost_align not in GHOST_ALIGNS:
            raise ValueError(f"ghost_align must be one of {GHOST_ALIGNS}")
        self.lay = lay = CompareLayout.for_size(*size, shared=self.layout == "shared")
        self.v08 = any(f.log.has_gps or f.log.has_hold for f in self.flights)
        self.shift_xy = np.zeros(2)
        self.warnings: list[str] = []
        self.offsets = align_offsets(before.log, after.log, align)
        spans = [(float(f.t[0]), float(f.t[-1])) for f in self.flights]
        self.tau0, self.tau1 = tau_window(spans, self.offsets, start, end)
        self.tau = sv.frame_times(self.tau0, self.tau1, fps, speed)
        self.maps = [map_side(self.tau, o, a, b) for o, (a, b) in zip(self.offsets, spans)]
        self.labels = [self._side_label(0), self._side_label(1)]
        cam = spec.camera
        self.sync = sync or cam.get("sync", "scale")
        if self.sync not in SYNCS:
            raise ValueError(f"camera sync must be one of {SYNCS}")
        prof = [str(f.log.meta.get("profile", "")) for f in self.flights]
        self.framing = framing or cam.get("framing") or ("demo" if all(p == "demo" for p in prof) else "all")
        self.scene_name = sv.scene_profile(after.log.meta, scene if scene != "auto" else cam.get("scene", "auto"),
                                           gps=any(f.log.has_gps for f in self.flights))
        self.az = float(azimuth if azimuth is not None else cam.get("azimuth", pair_azimuth([f.log for f in self.flights])))
        cam_kw: dict[str, Any] = dict(sv.DEMO_CAMERA) if self.framing == "demo" else {}
        self.elevation = float(elevation if elevation is not None else cam.get("elevation", cam_kw.pop("elevation_deg", 12.0)))
        cam_kw.pop("elevation_deg", None)
        self.cam_kw = cam_kw
        self.style = style
        self.warnings += isolation_warnings(spec.isolation, [f.log.meta for f in self.flights])
        for f in self.flights:
            self.warnings += [f"{os.path.basename(f.path)}: {w}" for w in f.log.warnings]
        # metrics
        self.metrics: list[dict[str, Any]] = []
        for side, key in enumerate(("before", "after")):
            comp = extract_metrics(self.flights[side])
            fj = None
            if key in spec.metrics_json:
                try:
                    fj = metrics_from_json(spec.metrics_json[key], spec.base_dir)
                except (OSError, ValueError, json.JSONDecodeError) as e:
                    self.warnings.append(f"metrics_json.{key}: {e}")
            vals, src, warns = combine_metrics(spec.metrics, comp, fj, spec.metric_values.get(key))
            self.warnings += [f"{key} {w}" for w in warns]
            self.metrics.append({"values": vals, "sources": src, "computed": comp, "json": fj})
        # v0.8: where a log has no hold point of its own, the station-keeping numbers use the v0.8 rule (said on the cards)
        self.auto_notes: list[str] = []
        uses_hold = ("hold_dist" in channels or any(m in spec.metrics for m in ("sk_rms", "sk_max", "td_hold"))
                     or self.layout == "shared")
        for side, f in enumerate(self.flights):
            if uses_hold and f.hold[1] == "derived":
                self.auto_notes.append(f"{self.labels[side]}: 기록된 유지점이 없어 v0.8 규칙(첫 호버링 순간의 무게중심 위치)으로 "
                                       "계산한 유지점 기준 ('(계산)' 표시)")
        self.strip = CompareStrip(self.flights, self.offsets, self.labels, channels, self.tau0, self.tau1, lay.W,
                                  lay.H - lay.strip_y, lay.s, align.label)
        self.sides: list[Side] = []
        self.plans: list[Any] = []
        self.topviews: list[TopView] = []
        self.camera_info: dict[str, Any] = {}
        if build_scenes:
            self._build_scenes(aa, asset_dir)

    # ---- construction ---------------------------------------------------------------
    def _side_label(self, side: int) -> str:
        sp = self.spec
        lab, ver = (sp.before_label, sp.before_version) if side == 0 else (sp.after_label, sp.after_version)
        return f"{lab} · {ver}" if ver else lab

    TOPVIEW_PX = 232

    def topview_rect(self) -> tuple[int, int, int, int]:
        s, vw, vh = self.lay.s, self.lay.view_w, self.lay.view_h
        w = int(self.TOPVIEW_PX * s)
        h = w + int(48 * s)
        return (vw - int(12 * s) - w, vh - int(12 * s) - h, vw - int(12 * s), vh - int(12 * s))

    def avoid_px(self) -> list[tuple[int, int, int, int]]:
        """Overlay boxes in a 3-D view (view pixels): the label + event pills (top left), the HUD (bottom left), the
        top-view inset (bottom right); shared layout: the legend / event pills (top left) and the touchdown pills (top
        right) only (the table and the top view sit beside the view)."""
        s, vw, vh = self.lay.s, self.lay.view_w, self.lay.view_h
        if self.layout == "shared":
            return [(0, 0, int(760 * s), int(100 * s)), (vw - int(560 * s), 0, vw, int(100 * s))]
        x0, y0, x1, y1 = self.topview_rect()
        n_rows = 7 + (3 if self.v08 else 0)                    # the HUD grows with the v0.8 rows
        hud_h = int((24 * n_rows + 14 + 12) * s)
        return [(0, 0, int(500 * s), int(100 * s)), (0, vh - max(int(206 * s), hud_h), int((430 if self.v08 else 372) * s), vh),
                (x0 - int(6 * s), y0 - int(6 * s), vw, vh)]

    def _build_scenes(self, aa: str, asset_dir: str | None) -> None:
        if self.layout == "shared":
            self._build_shared(aa, asset_dir)
            return
        lay = self.lay
        size = (lay.view_w, lay.view_h)
        aspect = lay.view_w / lay.view_h
        allpos = np.vstack([f.log.pos for f in self.flights])
        radii = sv.Scene._ring_radii(types.SimpleNamespace(pos=allpos, n=allpos.shape[0]))
        kw = dict(fps=self.fps, speed=self.speed, size=size, framing=self.framing, azimuth_deg=self.az,
                  elevation_deg=self.elevation, aa=aa, lang="ko", figure="auto", style=self.style, hud="minimal",
                  camera="chase", scene=self.scene_name, asset_dir=asset_dir)
        with shared_rings(radii):
            for side, f in enumerate(self.flights):
                self.sides.append(Side(f, self.maps[side][0], self.maps[side][1], kw))
        self.warnings += [f"{('before', 'after')[i]} renderer: {w}" for i, sd in enumerate(self.sides)
                          for w in sd.rnd.warnings if w not in sd.fl.log.warnings]
        # the shared camera
        W, H = float(lay.view_w), float(lay.view_h)
        avoid = [(x0 / W * 2.0 - 1.0, x1 / W * 2.0 - 1.0, 1.0 - y1 / H * 2.0, 1.0 - y0 / H * 2.0)
                 for x0, y0, x1, y1 in self.avoid_px()]
        box: sv.NDC = (-0.62, 0.86, -0.56, 0.62)
        kwp = dict(self.cam_kw)
        # the comparison keeps the vehicle larger than sim_video's 70 m chase limit: beyond d_max the pad drops out of
        # the framing (the top-view inset carries the distance from the pad, at one scale for both sides)
        kwp.setdefault("d_max", float(self.spec.camera.get("d_max", D_MAX_M)))
        if self.style == "realistic":
            kwp.setdefault("view_angle_deg", 38.0)
            kwp["follow_sigma_s"] = max(float(kwp.get("follow_sigma_s", 0.12)), 0.3)
            kwp["smooth_s"] = max(float(kwp.get("smooth_s", 0.6)), 1.0)
        fps_eff = self.fps / self.speed
        length = self.sides[0].rnd.scene.length_m
        n = self.tau.size
        az = np.full(n, self.az)
        if self.sync == "scale":
            plans = []
            for sd in self.sides:
                extra = sv.figure_points(sd.rnd.fig) if sd.rnd.with_figure else None
                plans.append(sv.plan_camera(sd.rnd.fr, sd.fl.log.ground_z, fps_eff, aspect, length, self.framing, az,
                                            self.elevation, box=box, avoid=avoid, extra_pts=extra, **kwp))
            d = shared_distance([p.dist for p in plans], fps_eff, float(kwp.get("smooth_s", 0.6)))
            va = plans[0].view_angle_deg
            self.plans = [sv.CameraPlan(reaim(p.focal, p.dist, d, az, self.elevation, va, aspect, box), d.copy(), az,
                                        self.elevation, va, p.framing) for p in plans]
        else:
            frb, fra = self.sides[0].rnd.fr, self.sides[1].rnd.fr
            vpa = sv.vehicle_points(fra, length)
            below = np.column_stack([vpa[:, 0, 0], vpa[:, 0, 1], np.full(n, self.flights[1].log.ground_z)])[:, None, :]
            kwp2 = dict(kwp, d_max=1.0e4)
            p = sv.plan_camera(frb, self.flights[0].log.ground_z, fps_eff, aspect, length, "all", az, self.elevation,
                               box=box, avoid=avoid, extra_pts=np.concatenate([vpa, below], axis=1), **kwp2)
            self.plans = [p, p]
        for sd, p in zip(self.sides, self.plans):
            sd.rnd.cam = p
        fr_in = [in_view_fraction(sd.rnd.fr, p, length, aspect) for sd, p in zip(self.sides, self.plans)]
        for i, fv in enumerate(fr_in):
            if fv < 0.98:
                self.warnings.append(f"{('before', 'after')[i]} vehicle in view in only {100 * fv:.1f} % of the frames")
        self.camera_info = {"sync": self.sync, "azimuth_deg": self.az, "elevation_deg": self.elevation,
                            "view_angle_deg": self.plans[0].view_angle_deg, "framing": self.framing,
                            "scene": self.scene_name, "ring_radii_m": [float(r) for r in self.sides[0].rnd.scene.ring_radii],
                            "dist_m": [round(float(self.plans[0].dist.min()), 2), round(float(self.plans[0].dist.max()), 2)],
                            "in_view_fraction": [round(x, 4) for x in fr_in], "d_max_m": kwp["d_max"]}
        r_all = max(float(np.max(np.hypot(f.log.pos[:, 0], f.log.pos[:, 1]))) for f in self.flights)
        self.topviews = [TopView(r_all, int(self.TOPVIEW_PX * lay.s), lay.s, self.az, BEFORE_C if i == 0 else AFTER_C)
                         for i in (0, 1)]

    def _build_shared(self, aa: str, asset_dir: str | None) -> None:
        """--layout shared: ONE scene of the after log with the before vehicle as a ghost, one camera that frames the
        after vehicle, the ground, the pad and - while it fits within ``d_max`` - the ghost (beyond that an edge
        marker points at it), the comparison table + one top view beside the view."""
        lay = self.lay
        b, a = self.flights
        size = (lay.view_w, lay.view_h)
        aspect = lay.view_w / lay.view_h
        # ghost alignment: as flown (pad), or the before log moved onto the after hold point (hold)
        if self.ghost_align == "hold":
            ha, hb = first_hold_xy(a.hold[0]), first_hold_xy(b.hold[0])
            if ha is None or hb is None:
                self.warnings.append("ghost_align hold: a log has no hold point (no HOLD phase) - drawn as flown")
            else:
                self.shift_xy = ha - hb
        allpos = np.vstack([a.log.pos[:, :2], b.log.pos[:, :2] + self.shift_xy])
        radii = sv.Scene._ring_radii(types.SimpleNamespace(pos=allpos, n=allpos.shape[0]))
        kw = dict(fps=self.fps, speed=self.speed, size=size, framing=self.framing, azimuth_deg=self.az,
                  elevation_deg=self.elevation, aa=aa, lang="ko", figure="auto", style=self.style, hud="minimal",
                  camera="chase", scene=self.scene_name, asset_dir=asset_dir)
        with shared_rings(radii):
            sd = SharedSide(a, b, self.maps[1][0], self.maps[1][1], self.maps[0][0], self.maps[0][1], kw, self.shift_xy,
                            self.labels)
        self.sides = [sd]
        self.warnings += [f"after renderer: {w}" for w in sd.rnd.warnings if w not in a.log.warnings]
        W, H = float(lay.view_w), float(lay.view_h)
        avoid = [(x0 / W * 2.0 - 1.0, x1 / W * 2.0 - 1.0, 1.0 - y1 / H * 2.0, 1.0 - y0 / H * 2.0)
                 for x0, y0, x1, y1 in self.avoid_px()]
        box: sv.NDC = (-0.82, 0.86, -0.80, 0.58)
        kwp = dict(self.cam_kw)
        kwp["d_max"] = float(self.spec.camera.get("d_max", D_MAX_SHARED_M))
        if self.style == "realistic":
            kwp.setdefault("view_angle_deg", 38.0)
            kwp["follow_sigma_s"] = max(float(kwp.get("follow_sigma_s", 0.12)), 0.3)
            kwp["smooth_s"] = max(float(kwp.get("smooth_s", 0.6)), 1.0)
        fps_eff = self.fps / self.speed
        length = sd.rnd.scene.length_m
        n = self.tau.size
        az = np.full(n, self.az)
        gp = np.stack([sd.ghost.points(k) for k in range(n)])                     # (N, 3, 3)
        extra = gp if not sd.rnd.with_figure else np.concatenate([sv.figure_points(sd.rnd.fig), gp], axis=1)
        plan = sv.plan_camera(sd.rnd.fr, a.log.ground_z, fps_eff, aspect, length, self.framing if self.framing != "vehicle"
                              else "all", az, self.elevation, box=box, avoid=avoid, extra_pts=extra, **kwp)
        self.plans = [plan]
        sd.rnd.cam = plan
        fv = in_view_fraction(sd.rnd.fr, plan, length, aspect)
        ghost_in = self.ghost_in_view(np.arange(n))
        if fv < 0.98:
            self.warnings.append(f"after vehicle in view in only {100 * fv:.1f} % of the frames")
        self.camera_info = {"layout": "shared", "ghost_align": self.ghost_align, "ghost_shift_m": self.shift_xy.tolist(),
                            "azimuth_deg": self.az, "elevation_deg": self.elevation, "view_angle_deg": plan.view_angle_deg,
                            "framing": self.framing, "scene": self.scene_name,
                            "ring_radii_m": [float(r) for r in sd.rnd.scene.ring_radii],
                            "dist_m": [round(float(plan.dist.min()), 2), round(float(plan.dist.max()), 2)],
                            "in_view_fraction": [round(float(np.mean(ghost_in)), 4), round(fv, 4)],
                            "d_max_m": kwp["d_max"]}
        # one top view: the scale of the after vehicle and its hold point (the ghost beyond it: an edge marker)
        ha = first_hold_xy(a.hold[0])
        r_a = float(np.max(np.hypot(a.log.pos[:, 0], a.log.pos[:, 1])))
        if ha is not None:
            r_a = max(r_a, float(np.hypot(*ha)) + 1.0)
        tw = min(lay.col_w - int(24 * lay.s), lay.view_h - int(self.TABLE_H * lay.s) - int(8 * lay.s) - int(48 * lay.s))
        self.topviews = [TopView(max(r_a, 2.0), int(tw), lay.s, self.az, AFTER_C)]

    TABLE_H = 236                   # shared layout: the comparison table's height (px at 1080 lines)

    def ghost_in_view(self, ks: np.ndarray) -> np.ndarray:
        """Per frame: is the ghost (its three body points) inside the shared view?"""
        sd = self.sides[0]
        aspect = self.lay.view_w / self.lay.view_h
        out = np.zeros(len(ks), dtype=bool)
        for j, k in enumerate(ks):
            q, z = sv._ndc(sd.ghost.points(int(k)), self.plans[0].view(int(k)), aspect)
            out[j] = float(z.min()) > 0.0 and float(np.max(np.abs(q))) <= 1.0
        return out

    # ---- 3-D passes -----------------------------------------------------------------
    def render_pass(self, side: int, ks: Sequence[int], sink: Callable[[int, np.ndarray], None],
                    progress: Callable[[int, int], None] | None = None) -> float:
        """Render the 3-D view of one side for frames ``ks`` (``sink(j, image)``); returns the wall time.  The two
        sides are rendered in separate passes: alternating two OpenGL contexts frame by frame costs ~4x per view
        (measured 2026-10-21, Intel UHD 750: 0.74 vs 0.175 s per 936 x 556 realistic view)."""
        sd = self.sides[side]
        t0 = time.perf_counter()
        for j, k in enumerate(ks):
            sink(j, sd.render3d(int(k), sd.rnd.cam.view(int(k))))
            if progress is not None:
                progress(j, len(ks))
        return time.perf_counter() - t0

    # ---- one frame ------------------------------------------------------------------
    def frame(self, k: int, views: Sequence[np.ndarray] | None = None) -> np.ndarray:
        """Frame k; ``views`` = the pre-rendered 3-D views (``render_pass``; two, or one in the shared layout), else
        rendered here."""
        from PIL import Image, ImageDraw
        if self.layout == "shared":
            return self._frame_shared(k, views)
        lay, s = self.lay, self.lay.s
        canvas = Image.new("RGB", (lay.W, lay.H), sv.hex_rgb(sv.SURFACE))
        for side, sd in enumerate(self.sides):
            cam = sd.rnd.cam.view(k)
            img = views[side] if views is not None else sd.render3d(k, cam)
            view = Image.fromarray(np.asarray(img)).convert("RGBA")
            st = int(sd.status[k])
            if st != RUN:                        # the log has ended (or not begun): its frozen view is veiled
                view = Image.alpha_composite(view, Image.new("RGBA", view.size, (10, 14, 20, 105)))
            ov = Image.new("RGBA", view.size, (0, 0, 0, 0))
            d = ImageDraw.Draw(ov)
            if st == RUN:
                sd.rnd._draw_marker(d, k, cam)
            self._draw_side(d, ov, side, k, st)
            view = Image.alpha_composite(view, ov)
            canvas.paste(view.convert("RGB"), (lay.view_x[side], lay.view_y))
        dc = ImageDraw.Draw(canvas)
        for side in (0, 1):
            col = sv.hex_rgb(BEFORE_C if side == 0 else AFTER_C)
            x0, y0 = lay.view_x[side], lay.view_y
            dc.rectangle([x0 - 2, y0 - 2, x0 + lay.view_w + 1, y0 + lay.view_h + 1], outline=col, width=max(2, int(3 * s)))
        canvas.paste(Image.fromarray(self.strip.frame(float(self.tau[k]))), (0, lay.strip_y))
        self._draw_banner(canvas)
        return np.asarray(canvas)

    # ---- the shared layout ------------------------------------------------------------
    def _side_time(self, side: int, k: int) -> float:
        """Log time of frame k on one side (clamped to the log: a frozen log keeps its last sample)."""
        return float(self.maps[side][0][k])

    def _xy_past_side(self, side: int, k: int) -> np.ndarray:
        """Horizontal exit-plane track of one log up to frame k (the before log moved by the ghost shift)."""
        f = self.flights[side]
        t = self._side_time(side, k)
        j = int(np.searchsorted(f.t, t, side="right"))
        xy = f.log.pos[:max(j, 1), :2]
        step = max(1, xy.shape[0] // 800)
        cur = np.array([[np.interp(t, f.t, f.log.pos[:, 0]), np.interp(t, f.t, f.log.pos[:, 1])]])
        out = np.vstack([xy[::step], cur])
        return out + (self.shift_xy if side == 0 else 0.0)

    def _frame_shared(self, k: int, views: Sequence[np.ndarray] | None = None) -> np.ndarray:
        from PIL import Image, ImageDraw
        lay, s = self.lay, self.lay.s
        sd = self.sides[0]
        cam = sd.rnd.cam.view(k)
        img = views[0] if views is not None else sd.render3d(k, cam)
        view = Image.fromarray(np.asarray(img)).convert("RGBA")
        st_a, st_b = int(sd.status[k]), int(sd.ghost_status[k])
        if st_a != RUN:
            view = Image.alpha_composite(view, Image.new("RGBA", view.size, (10, 14, 20, 105)))
        ov = Image.new("RGBA", view.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        if st_a == RUN:
            sd.rnd._draw_marker(d, k, cam)
        self._draw_shared_view(d, k, st_a, st_b, cam)
        view = Image.alpha_composite(view, ov)
        canvas = Image.new("RGB", (lay.W, lay.H), sv.hex_rgb(sv.SURFACE))
        canvas.paste(view.convert("RGB"), (lay.view_x[0], lay.view_y))
        dc = ImageDraw.Draw(canvas, "RGBA")
        x0, y0 = lay.view_x[0], lay.view_y
        dc.rectangle([x0 - 2, y0 - 2, x0 + lay.view_w + 1, y0 + lay.view_h + 1], outline=(60, 66, 76),
                     width=max(2, int(3 * s)))
        y_tab = self._draw_shared_table(dc, k)
        # one top view: both tracks (the ghost clipped to the after vehicle's scale)
        a = self.flights[1]
        hold_xy, hold_d = None, None
        h, _ = a.hold
        if h is not None:
            i = int(np.clip(np.searchsorted(a.t, self._side_time(1, k), side="right") - 1, 0, a.log.n - 1))
            hold_xy, hold_d = h[i], float(a.hold_dist()[i])
        tv = self.topviews[0].frame(self._xy_past_side(1, k), hold=hold_xy, hold_d=hold_d,
                                    others=[(self._xy_past_side(0, k), sv.hex_rgb(BEFORE_C), "변경 전")])
        tx = lay.col_x + (lay.col_w - tv.size[0]) // 2
        canvas.paste(tv, (int(tx), int(y_tab + 8 * s)), tv)
        canvas.paste(Image.fromarray(self.strip.frame(float(self.tau[k]))), (0, lay.strip_y))
        self._draw_banner(canvas)
        return np.asarray(canvas)

    def _draw_shared_view(self, d: Any, k: int, st_a: int, st_b: int, cam: sv.CamView) -> None:
        """Overlays of the shared view: which vehicle is which, the latest event, touchdown pills of both logs, the
        ghost's edge marker when it is out of view, and the ended / not-started notes."""
        s = self.lay.s
        vw, vh = self.lay.view_w, self.lay.view_h
        b, a = self.flights
        cb, ca = sv.hex_rgb(BEFORE_C), sv.hex_rgb(AFTER_C)
        fl = F(19 * s, True)
        x = 12 * s
        box = pill(d, (x, 12 * s), f"{self.labels[1]} (진하게)", F(19 * s, True, self.labels[1]), ca + (240,),
                   (255, 255, 255, 255), pad=(12 * s, 5 * s), radius=8 * s)
        gl = f"{self.labels[0]} (회색 반투명)"
        if st_b == ENDED:
            gl += " · 로그 끝: 멈춤"
        elif st_b == PRE:
            gl += " · 시작 전"
        box2 = pill(d, (box[2] + 8 * s, 12 * s), gl, fl, (110, 116, 124, 235), (255, 255, 255, 255), pad=(12 * s, 5 * s),
                    radius=8 * s)
        y = box2[3] + 6 * s
        if self.ghost_align == "hold" and np.any(self.shift_xy != 0.0):
            txt = (f"회색 기체는 유지점을 맞추려고 ({self.shift_xy[0]:+.1f}, {self.shift_xy[1]:+.1f}) m 옮겨 그림")
            box3 = pill(d, (x, y), txt, F(15 * s, False, txt), (14, 20, 30, 190), (235, 238, 242, 255), pad=(10 * s, 4 * s),
                        radius=6 * s)
            y = box3[3] + 4 * s
        t_a = self._side_time(1, k)
        evs = [(et, ex) for et, ex in sv.recent_events(a.log.event_t, a.log.event_text, t_a, 12) if not ex.startswith("GS ")]
        if evs and t_a - evs[-1][0] <= 4.0:
            g = sv.event_gloss_ko(evs[-1][1])
            txt = g if g else evs[-1][1][:60]
            pill(d, (x, y), f"변경 후 이벤트: {txt}", F(15 * s, False, txt), (14, 20, 30, 190), (235, 238, 242, 255),
                 pad=(10 * s, 4 * s), radius=6 * s)
        # touchdown pills (5 s), both logs, top right
        yr = 12 * s
        for side, f in ((1, a), (0, b)):
            t = self._side_time(side, k)
            tds = [td for td in touchdowns(f.log) if td["t"] <= t + 1e-9 and t - td["t"] <= 5.0]
            if tds:
                td = tds[-1]
                txt = (f"{'후' if side == 1 else '전'} 착지 #{td['i']}  v_z {td['vz']:+.2f} · 수평 {td['vh']:.2f} m/s"
                       + (f" · {td['tilt']:.1f}°" if td["tilt"] is not None else ""))
                bx = pill(d, (vw - 12 * s, yr), txt, F(16 * s, True, txt), (250, 250, 250, 235), (ca if side else cb) + (255,),
                          pad=(10 * s, 5 * s), anchor_right=True, radius=8 * s)
                yr = bx[3] + 6 * s
        # the ghost out of view: an arrow on the view edge toward it, with its distance from the after hold point
        sd = self.sides[0]
        mid = sd.ghost.points(k)[1]
        px, z = sv.project(mid[None, :], cam, vw, vh)
        p = px[0].astype(float)
        inside = float(z[0]) > 0 and 0.0 <= p[0] <= vw and 0.0 <= p[1] <= vh
        if not inside:
            c = np.array([vw / 2.0, vh / 2.0])
            v = p - c if float(z[0]) > 0 else c - p
            n_ = float(np.hypot(*v))
            v = v / n_ if n_ > 1e-6 else np.array([1.0, 0.0])
            m = 34 * s
            sx = (vw / 2.0 - m) / max(abs(v[0]), 1e-6)
            sy = (vh / 2.0 - m) / max(abs(v[1]), 1e-6)
            tip = c + v * min(sx, sy)
            nrm = np.array([-v[1], v[0]])
            hl = 22 * s
            d.polygon([tuple(tip), tuple(tip - v * hl + nrm * hl * 0.55), tuple(tip - v * hl - nrm * hl * 0.55)],
                      fill=cb + (240,), outline=(255, 255, 255, 255))
            ha = first_hold_xy(a.hold[0])
            g_xy = sd.ghost_fr.cg_xy[k] if sd.ghost_fr.cg_xy is not None else sd.ghost_fr.pos[k, :2]
            if ha is not None:
                txt = f"변경 전 기체: 유지점에서 {float(np.hypot(*(g_xy - ha))):.1f} m"
            else:
                txt = f"변경 전 기체: 패드에서 {float(np.hypot(*g_xy)):.1f} m"
            ft = F(16 * s, True, txt)
            tw = dlen(d, txt, ft)
            gap = hl + 14 * s                                     # the label sits inward of the arrow, never on it
            if abs(v[0]) >= abs(v[1]):
                lx = tip[0] - gap - tw if v[0] > 0 else tip[0] + gap
                ly = tip[1] - 12 * s
            else:
                lx = tip[0] - tw / 2.0
                ly = tip[1] - gap - 24 * s if v[1] > 0 else tip[1] + gap
            lx = float(np.clip(lx, 12 * s, vw - tw - 12 * s))
            ly = float(np.clip(ly, 8 * s, vh - 30 * s))
            d.rounded_rectangle([lx - 8 * s, ly - 3 * s, lx + tw + 8 * s, ly + 25 * s], radius=int(6 * s),
                                fill=(14, 20, 30, 200))
            dtext(d, (lx, ly), txt, fill=(235, 238, 242, 255), font=ft)
        if st_a != RUN:
            big = "종료" if st_a == ENDED else "시작 전"
            fb = F(44 * s, True, big)
            tw = dlen(d, big, fb)
            x0_, y0_ = vw / 2.0 - tw / 2.0 - 24 * s, 110 * s
            d.rounded_rectangle([x0_, y0_, x0_ + tw + 48 * s, y0_ + 70 * s], radius=int(12 * s), fill=(14, 20, 30, 215),
                                outline=ca + (255,), width=max(2, int(2 * s)))
            dtext(d, (x0_ + 24 * s, y0_ + 6 * s), big, fill=(255, 255, 255, 255), font=fb)

    def _shared_cells(self, side: int, k: int) -> dict[str, str]:
        """The comparison table's values of one log at frame k (log time, clamped)."""
        f = self.flights[side]
        lg = f.log
        t = self._side_time(side, k)
        i = int(np.clip(np.searchsorted(f.t, t, side="right") - 1, 0, lg.n - 1))
        ph = int(lg.phase[i])
        st_name = lg.state_name(int(lg.state[i]), "ko")
        out = {"단계": lg.phase_name(ph, "ko") if ph else st_name,
               "높이": f"{float(np.interp(t, f.t, f.height)):5.2f} m",
               "기울기": f"{float(np.interp(t, f.t, lg.tilt_deg)):4.1f}°",
               "수평 속도": f"{float(np.hypot(np.interp(t, f.t, lg.vel[:, 0]), np.interp(t, f.t, lg.vel[:, 1]))):4.2f} m/s",
               "패드 거리": f"{float(np.hypot(np.interp(t, f.t, lg.pos[:, 0]), np.interp(t, f.t, lg.pos[:, 1]))):5.1f} m"}
        h, src = f.hold
        if h is not None:
            v = float(f.hold_dist()[i])
            out["유지점 거리"] = (f"{v:5.2f} m" + (" (계산)" if src == "derived" else "")) if math.isfinite(v) else "아직 없음"
        else:
            out["유지점 거리"] = "없음"
        if lg.gps_ok is not None or lg.gps_sats is not None or lg.gps_fix is not None:
            ok = bool(lg.gps_ok[i]) if lg.gps_ok is not None else bool(lg.gps_fix is not None and lg.gps_fix[i] >= 3)
            txt, _ = sv.gps_text({"ok": ok, "sats": None if lg.gps_sats is None else lg.gps_sats[i],
                                  "fix": None if lg.gps_fix is None else lg.gps_fix[i]}, "ko")
            out["GPS"] = txt
        else:
            out["GPS"] = "없음" if not lg.has_gps else "기록 없음"
        return out

    SHARED_ROWS = ("유지점 거리", "수평 속도", "패드 거리", "높이", "기울기", "단계", "GPS")

    def _draw_shared_table(self, dc: Any, k: int) -> float:
        """The comparison table (right column, top): the same quantities of both logs at this instant; returns the y
        below it."""
        lay, s = self.lay, self.lay.s
        x0, y0 = lay.col_x, lay.view_y
        w, h = lay.col_w, int(self.TABLE_H * s)
        dc.rounded_rectangle([x0, y0, x0 + w, y0 + h], radius=int(10 * s), fill=(14, 20, 30, 235))
        dtext(dc, (x0 + 14 * s, y0 + 8 * s), f"지금 이 순간 (참값)   τ {float(self.tau[k]):+.2f} s", fill=(200, 208, 220),
              font=F(15 * s, True))
        c0 = x0 + 14 * s
        c1 = x0 + 118 * s
        c2 = x0 + 118 * s + (w - 132 * s) / 2.0
        cw = (w - 132 * s) / 2.0 - 8 * s
        fh = F(15 * s, True)
        for cx, lab, col in ((c1, self.labels[0], BEFORE_C), (c2, self.labels[1], AFTER_C)):
            tt = lab
            while dlen(dc, tt, fh) > cw and len(tt) > 4:
                tt = tt[:-2]
            dtext(dc, (cx, y0 + 34 * s), tt if tt == lab else tt + "…", fill=sv.hex_rgb(col), font=fh)
            dc.rectangle([cx, y0 + 56 * s, cx + cw, y0 + 58 * s], fill=sv.hex_rgb(col))
        cells = [self._shared_cells(0, k), self._shared_cells(1, k)]
        fl, fv = F(14.5 * s, True), F(14.5 * s)
        y = y0 + 64 * s
        for i, key in enumerate(self.SHARED_ROWS):
            if i % 2 == 0:
                dc.rectangle([x0 + 6 * s, y - 2 * s, x0 + w - 6 * s, y + 21 * s], fill=(255, 255, 255, 12))
            dtext(dc, (c0, y), key, fill=sv.TONES["muted"], font=fl)
            for cx, cell in ((c1, cells[0]), (c2, cells[1])):
                v = cell.get(key, "")
                while dlen(dc, v, fv) > cw and len(v) > 4:
                    v = v[:-2]
                col = sv.TONES["alert"] if ("사용 불가" in v or "신호 없음" in v) else sv.TONES["normal"]
                dtext(dc, (cx, y), v, fill=col, font=fv)
            y += 23 * s
        return y0 + h

    def _hud_rows(self, side: int, k: int) -> list[tuple[str, str]]:
        sd, f = self.sides[side], self.flights[side]
        fr, log = sd.rnd.fr, f.log
        t = float(fr.t[k])
        tau = float(self.tau[k])
        ph = int(fr.phase[k])
        st_name = log.state_name(int(fr.state[k]), "ko")
        ph_name = log.phase_name(ph, "ko") if ph else st_name
        h = float(np.interp(t, f.t, f.height))
        tgt = float(fr.h_target[k])
        tilt = float(fr.tilt_deg[k])
        est = f.ex.get("tilt_ctrl_deg")
        tilt_txt = f"{tilt:5.1f}°" + (f"   (추정 {float(np.interp(t, f.t, est)):4.1f}°)" if est is not None else "")
        rows = [("시간", f"τ {tau:+7.2f} s   (로그 {t:6.2f} s)"),
                ("단계", ph_name if ph_name == st_name else f"{ph_name} · {st_name}"),
                ("높이", f"{h:5.2f} m" + (f"   (목표 {tgt:.1f} m)" if math.isfinite(tgt) else "")),
                ("수직 속도", f"{round(float(fr.vel[k, 2]), 2) + 0.0:+5.2f} m/s"),
                ("기울기", tilt_txt),
                ("패드 거리", f"{float(np.hypot(fr.pos[k, 0], fr.pos[k, 1])):5.1f} m"),
                ("스로틀", f"{float(fr.throttle[k]):5.3f}")]
        rows += self._v08_rows(side, k)
        return [(a, b) for a, b in rows]

    def _v08_rows(self, side: int, k: int) -> list[tuple[str, str]]:
        """v0.8 rows when either log holds position with GPS: the distance from the hold point (a derived one marked
        '계산'), the horizontal speed, the GPS status ('없음' for a build without GPS)."""
        if not self.v08:
            return []
        f = self.flights[side]
        fr = self.sides[side].rnd.fr if self.sides else None
        t = float(fr.t[k]) if fr is not None else float(self.maps[side][0][k])
        rows: list[tuple[str, str]] = []
        h, src = f.hold
        if h is not None:
            i = int(np.clip(np.searchsorted(f.t, t, side="right") - 1, 0, f.log.n - 1))
            hd = f.hold_dist()
            v = float(hd[i]) if hd is not None else float("nan")
            rows.append(("유지점", (f"{v:5.2f} m" + ("  (계산)" if src == "derived" else "")) if math.isfinite(v)
                         else "아직 없음 (상승 끝에 설정)"))
        vh = float(np.hypot(*np.array([np.interp(t, f.t, f.log.vel[:, j]) for j in (0, 1)])))
        rows.append(("수평 속도", f"{vh:5.2f} m/s"))
        lg = f.log
        if lg.gps_ok is not None or lg.gps_sats is not None or lg.gps_fix is not None:
            i = int(np.clip(np.searchsorted(f.t, t, side="right") - 1, 0, lg.n - 1))

            def at(a: np.ndarray | None) -> Any:
                return None if a is None else a[i]
            ok = bool(lg.gps_ok[i]) if lg.gps_ok is not None else bool(lg.gps_fix is not None and lg.gps_fix[i] >= 3)
            txt, _ = sv.gps_text({"ok": ok, "sats": at(lg.gps_sats), "hacc": at(lg.gps_hacc), "fix": at(lg.gps_fix)}, "ko")
            rows.append(("GPS", txt))
        else:
            rows.append(("GPS", "없음 (GPS 없는 빌드)" if not lg.has_gps else "기록 없음"))
        return rows

    def _draw_side(self, d: Any, ov: Any, side: int, k: int, st: int) -> None:
        from PIL import ImageDraw
        s = self.lay.s
        vw, vh = self.lay.view_w, self.lay.view_h
        f = self.flights[side]
        col = sv.hex_rgb(BEFORE_C if side == 0 else AFTER_C)
        t = float(self.sides[side].rnd.fr.t[k])
        # label
        lab = self.labels[side]
        fl = F(22 * s, True, lab)
        box = pill(d, (12 * s, 12 * s), lab, fl, col + (240,), (255, 255, 255, 255), pad=(14 * s, 6 * s), radius=8 * s)
        # latest supervisor / sim event (not ground-station lines) within the last 4 s, Korean reading + logged text
        evs = [(et, ex) for et, ex in sv.recent_events(f.log.event_t, f.log.event_text, t, 12) if not ex.startswith("GS ")]
        if evs and t - evs[-1][0] <= 4.0:
            et, ex = evs[-1]
            g = sv.event_gloss_ko(ex)
            txt = f"{g}" if g else ex[:60]
            fe = F(16 * s, False, txt)
            pill(d, (12 * s, box[3] + 6 * s), f"이벤트: {txt}", fe, (14, 20, 30, 190), (235, 238, 242, 255),
                 pad=(10 * s, 4 * s), radius=6 * s)
        # touchdown flash (5 s): the logged values of the touchdown event
        tds = [td for td in touchdowns(f.log) if td["t"] <= t + 1e-9 and t - td["t"] <= 5.0]
        if tds:
            td = tds[-1]
            txt = (f"착지 #{td['i']}   v_z {td['vz']:+.2f} m/s · 수평 {td['vh']:.2f} m/s"
                   + (f" · 기울기 {td['tilt']:.1f}°" if td["tilt"] is not None else ""))
            ft = F(18 * s, True, txt)
            pill(d, (vw - 12 * s, 12 * s), txt, ft, (250, 250, 250, 235), (20, 20, 20, 255), pad=(12 * s, 6 * s),
                 anchor_right=True, radius=8 * s)
        # HUD
        rows = self._hud_rows(side, k)
        fl_, fv = F(15.5 * s, True), F(15.5 * s)
        lh = int(24 * s)
        lab_w = int(max(dlen(d, a, fl_) for a, _ in rows) + 26 * s)
        w = lab_w + int(max(dlen(d, v, font=fv) for _, v in rows)) + int(22 * s)
        w = max(w, int(300 * s))
        h = lh * len(rows) + int(14 * s)
        x0, y0 = int(12 * s), vh - int(12 * s) - h
        d.rounded_rectangle([x0, y0, x0 + w, y0 + h], radius=int(8 * s), fill=(14, 20, 30, 178))
        d.rectangle([x0, y0 + int(8 * s), x0 + max(2, int(4 * s)), y0 + h - int(8 * s)], fill=col + (255,))
        for i, (a, b) in enumerate(rows):
            y = y0 + int(7 * s) + i * lh
            dtext(d, (x0 + int(14 * s), y), a, fill=sv.TONES["muted"] + (255,), font=fl_)
            dtext(d, (x0 + lab_w, y), b, fill=sv.TONES["normal"] + (255,), font=fv)
        # top view (same scale on both sides)
        hold_xy, hold_d = None, None
        h, _src = f.hold
        if self.v08 and h is not None:
            i = int(np.clip(np.searchsorted(f.t, t, side="right") - 1, 0, f.log.n - 1))
            hold_xy, hold_d = h[i], float(f.hold_dist()[i])
        tv = self.topviews[side].frame(self.sides[side].rnd._xy_past(k), hold=hold_xy, hold_d=hold_d)
        tx0, ty0, _, _ = self.topview_rect()
        ov.alpha_composite(tv, (int(tx0), int(ty0)))
        if st != RUN:
            d2 = ImageDraw.Draw(ov)
            big = "종료" if st == ENDED else "시작 전"
            fb = F(44 * s, True, big)
            tw = dlen(d2, big, font=fb)
            sub = (f"로그 끝 (t = {float(f.t[-1]):.2f} s): 마지막 상태로 멈춤" if st == ENDED
                   else f"로그 시작 전 (t = {float(f.t[0]):.2f} s)")
            fs_ = F(17 * s, False, sub)
            sw = dlen(d2, sub, font=fs_)
            bw = max(tw, sw) + 48 * s
            # top centre: the camera centres the (landed, frozen) vehicle; the corners carry the label, HUD, top view
            x0, y0 = vw / 2.0 - bw / 2.0, 62 * s
            x1, y1 = x0 + bw, y0 + 104 * s
            d2.rounded_rectangle([x0, y0, x1, y1], radius=int(12 * s), fill=(14, 20, 30, 215), outline=col + (255,),
                                 width=max(2, int(2 * s)))
            cx = 0.5 * (x0 + x1)
            dtext(d2, (cx - tw / 2, y0 + 6 * s), big, fill=(255, 255, 255, 255), font=fb)
            dtext(d2, (cx - sw / 2, y0 + 66 * s), sub, fill=(220, 226, 235, 255), font=fs_)

    def _draw_banner(self, canvas: Any) -> None:
        from PIL import ImageDraw
        lay, s, sp = self.lay, self.lay.s, self.spec
        d = ImageDraw.Draw(canvas, "RGBA")
        d.rectangle([0, 0, lay.W, lay.banner_h], fill=BG_DARK)
        tag = draw_sim_tag(d, lay.W - 16 * s, 12 * s, s)
        iso = ISOLATION_TAG[sp.isolation]
        fi = F(16 * s, True, iso)
        ic = ISOLATION_COL[sp.isolation]
        pill(d, (lay.W - 16 * s, tag[3] + 8 * s), iso, fi, ic + (255,), (15, 15, 15, 255), pad=(12 * s, 4 * s),
             anchor_right=True, radius=6 * s)
        max_w = tag[0] - 40 * s
        ft = F(31 * s, True, sp.title)
        title = sp.title
        while dlen(d, title, font=ft) > max_w and len(title) > 10:
            title = title[:-2]
        if title != sp.title:
            title = title.rstrip() + "…"
        dtext(d, (24 * s, 10 * s), title, fill=(255, 255, 255), font=ft)
        sc = str(self.flights[1].log.meta.get("scenario") or self.flights[0].log.meta.get("scenario") or "")
        line2 = f"시나리오 {sc}   ·   τ = 0: {self.align.label}   ·   재생 x{self.speed:g}"
        if sp.subtitle:
            line2 = sp.subtitle + "   ·   " + line2
        f2 = F(17 * s, False, line2)
        while dlen(d, line2, font=f2) > max_w and len(line2) > 10:
            line2 = line2[:-2]
        dtext(d, (26 * s, 58 * s), line2, fill=(190, 200, 214), font=f2)

    # ---- cards ----------------------------------------------------------------------
    def title_card(self) -> np.ndarray:
        from PIL import Image, ImageDraw
        lay, s, sp = self.lay, self.lay.s, self.spec
        im = Image.new("RGB", (lay.W, lay.H), BG_DARK)
        d = ImageDraw.Draw(im, "RGBA")
        draw_sim_tag(d, lay.W - 24 * s, 22 * s, s)
        x, mx = 110 * s, lay.W - 220 * s
        y = 52 * s
        dtext(d, (x, y), "CanSat EDF 수직이착륙 기체 · 제어 변경 기록 (변경 전 / 변경 후 비교)", fill=(150, 190, 255),
               font=F(24 * s, False))
        y += 46 * s
        y = draw_wrapped(d, (x, y), sp.title, F(44 * s, True, sp.title), (255, 255, 255), mx, 58 * s, 2)
        if sp.subtitle:
            y = draw_wrapped(d, (x, y + 2 * s), sp.subtitle, F(23 * s), (200, 208, 220), mx, 32 * s, 2)
        y += 22 * s
        colw = (lay.W - 2 * x - 60 * s) / 2.0
        y_cols = y
        yl = y_cols
        sections = [("무엇을 바꿨나", sp.what, x), ("왜 바꿨나", sp.why, x + colw + 60 * s)]
        ys = []
        for head, items, cx in sections:
            yy = y_cols
            dtext(d, (cx, yy), head, fill=(255, 205, 120), font=F(27 * s, True))
            yy += 42 * s
            for it in items or ["(spec에 없음)"]:
                fb = F(21 * s)
                dtext(d, (cx, yy), "•", fill=(255, 205, 120), font=fb)
                yy = draw_wrapped(d, (cx + 24 * s, yy), it, fb, (230, 234, 240), colw - 24 * s, 30 * s, 5) + 6 * s
            ys.append(yy)
        yl = max(ys) + 14 * s
        if sp.expected:
            dtext(d, (x, yl), "기대 효과 (검증 대상)", fill=(150, 220, 180), font=F(23 * s, True))
            yl += 36 * s
            for it in sp.expected:
                yl = draw_wrapped(d, (x + 24 * s, yl), "• " + it, F(20 * s), (220, 228, 238), lay.W - 2 * x - 24 * s, 28 * s, 3)
            yl += 10 * s
        # how it is compared
        dtext(d, (x, yl), "비교 방법", fill=(160, 200, 255), font=F(23 * s, True))
        yl += 36 * s
        b, a = self.flights
        mb, ma = b.log.meta, a.log.meta
        title_ko = str(ma.get("title_ko") or mb.get("title_ko") or ma.get("title") or "")
        if self.layout == "shared":
            first = (f"한 장면 비교: {self.labels[1]} = 진한 기체 (파랑 표시),  {self.labels[0]} = 회색 반투명 기체.  같은 패드, "
                     f"같은 바람 조건" + (", 회색 기체는 유지점을 맞춰 옮겨 그림" if self.ghost_align == "hold" else ", 두 기체 모두 비행한 그대로")
                     + ".  유지점 표시(하늘색 원 + 십자)는 변경 후 기체의 유지점.")
        else:
            first = f"왼쪽 (회색) = {self.labels[0]}      오른쪽 (파랑) = {self.labels[1]}"
        lines = [first,
                 f"시나리오: {ma.get('scenario', '?')} / {mb.get('scenario', '?')}   ·   시드 {mb.get('seed', '?')} / "
                 f"{ma.get('seed', '?')}   ·   {title_ko}",
                 f"시간 정렬: {self.align.label}을(를) 두 영상의 τ = 0으로 맞춤.  두 로그의 길이가 다르면 짧은 쪽은 끝에서 "
                 f"정지 화면 + '종료' 표시.",
                 f"로그: {os.path.basename(b.path)} (sha256 {b.log.sha256[:12]}, params {str(mb.get('params_sha256', '?'))[:12]})"
                 f"  /  {os.path.basename(a.path)} (sha256 {a.log.sha256[:12]}, params {str(ma.get('params_sha256', '?'))[:12]})"]
        if self.v08:
            lines.append("GPS: " + "  ".join(c + "." for c in sv.gps_caveats("ko")))
        for ln in lines:
            yl = draw_wrapped(d, (x + 24 * s, yl), ln, F(18.5 * s), (205, 213, 225), lay.W - 2 * x - 24 * s, 27 * s, 2)
        yl += 14 * s
        self._honesty_box(d, x, yl, lay.W - 2 * x)
        dtext(d, (x, lay.H - 46 * s), "화면의 모든 수치는 시뮬레이션 로그에서 읽은 값입니다 (변경 설명은 설계 기록에서 옮김).",
               fill=(255, 150, 140), font=F(19 * s, True))
        return np.asarray(im)

    def _honesty_box(self, d: Any, x: float, y: float, w: float) -> float:
        s, sp = self.lay.s, self.spec
        col = ISOLATION_COL[sp.isolation]
        fh = F(22 * s, True)
        fb = F(20 * s)
        lines = sv.wrap_px(d, safe(sp.note), fb, w - 48 * s)
        h = 44 * s + 29 * s * len(lines) + 12 * s
        d.rounded_rectangle([x, y, x + w, y + h], radius=int(10 * s), fill=col + (40,), outline=col + (255,),
                            width=max(2, int(2 * s)))
        dtext(d, (x + 22 * s, y + 10 * s), ISOLATION_TAG[sp.isolation], fill=col + (255,), font=fh)
        yy = y + 44 * s
        for ln in lines:
            dtext(d, (x + 22 * s, yy), ln, fill=(235, 238, 242), font=fb)
            yy += 29 * s
        return y + h

    def end_card(self) -> np.ndarray:
        from PIL import Image, ImageDraw
        lay, s, sp = self.lay, self.lay.s, self.spec
        im = Image.new("RGB", (lay.W, lay.H), BG_DARK)
        d = ImageDraw.Draw(im, "RGBA")
        draw_sim_tag(d, lay.W - 24 * s, 22 * s, s)
        x = 90 * s
        dtext(d, (x, 34 * s), "결과 비교 (시뮬레이션)", fill=(150, 190, 255), font=F(24 * s))
        title = sp.title
        ft = F(34 * s, True, title)
        while dlen(d, title, font=ft) > lay.W - 2 * x - 420 * s and len(title) > 10:
            title = title[:-2]
        dtext(d, (x, 68 * s), title if title == sp.title else title + "…", fill=(255, 255, 255), font=ft)
        metas = [self.flights[1].log.meta, self.flights[0].log.meta]
        cols = [x, x + 640 * s, x + 820 * s, x + 1110 * s, x + 1400 * s]
        right = lay.W - x
        y = 130 * s
        rh = 40 * s
        hh_ = 58 * s
        fhd, fhs, fr_, fv = F(19 * s, True), F(15 * s), F(19 * s), F(20 * s, True)
        d.rectangle([x - 10 * s, y, right, y + hh_], fill=(40, 52, 72))
        heads = [("항목", ""), ("기준", ""), (sp.before_label, sp.before_version), (sp.after_label, sp.after_version),
                 ("변화 (후 - 전)", "")]
        for cx, (hd, sub) in zip(cols, heads):
            for txt_, fnt_, yy_, col_ in ((hd, fhd, y + 6 * s, (220, 228, 240)), (sub, fhs, y + 32 * s, (175, 185, 200))):
                tt = txt_
                while dlen(d, tt, font=fnt_) > 275 * s and len(tt) > 6:
                    tt = tt[:-2]
                dtext(d, (cx, yy_), tt if tt == txt_ else tt + "…", fill=col_, font=fnt_)
        d.rectangle([cols[2] - 10 * s, y + hh_ - 4 * s, cols[2] + 260 * s, y + hh_], fill=sv.hex_rgb(BEFORE_C))
        d.rectangle([cols[3] - 10 * s, y + hh_ - 4 * s, cols[3] + 260 * s, y + hh_], fill=sv.hex_rgb(AFTER_C))
        y += hh_
        tone_col = {"better": (110, 220, 160), "worse": (255, 130, 120), "same": (190, 196, 205), "na": (190, 196, 205)}
        tone_word = {"better": "  개선", "worse": "  악화", "same": "", "na": ""}
        marks = {"spec": " *", "json": " †", "log": "", "none": ""}
        used = set()
        for i, key in enumerate(sp.metrics):
            md = METRICS[key]
            if i % 2 == 0:
                d.rectangle([x - 10 * s, y, right, y + rh], fill=(255, 255, 255, 10))
            dtext(d, (cols[0], y + 9 * s), md.label, fill=(235, 238, 242), font=fr_)
            lim = criterion_limit(key, metas)
            if lim is not None:
                unit = md.unit if md.unit == "°" else " " + md.unit
                dtext(d, (cols[1], y + 9 * s), f"|값| ≤ {lim:g}{unit}", fill=(200, 208, 220), font=fr_)
            for side in (0, 1):
                v = self.metrics[side]["values"].get(key)
                src = self.metrics[side]["sources"].get(key, "none")
                used.add(src)
                txt = fmt_metric(key, v) + marks[src]
                c = (245, 247, 250)
                if lim is not None and isinstance(v, (int, float)) and abs(float(v)) > lim + 1e-12:
                    c = (255, 135, 125)
                if v is None:
                    c = (150, 156, 166)
                fvx = fv if v is not None else fr_
                while dlen(d, txt, font=fvx) > 280 * s and len(txt) > 6:
                    txt = txt[:-2]
                dtext(d, (cols[2 + side], y + 8 * s), txt, fill=c, font=fvx)
            ch, tone = metric_change(key, self.metrics[0]["values"].get(key), self.metrics[1]["values"].get(key))
            dtext(d, (cols[4], y + 9 * s), ch + tone_word[tone], fill=tone_col[tone], font=fr_)
            y += rh
        y += 10 * s
        notes = ["빨강 값 = 기준 초과 (기준은 로그 meta.criteria; 판정 대상이 아닌 항목은 기준 칸이 비어 있음).  "
                 "'개선 / 악화'는 작을수록 좋은 항목에서 두 숫자를 비교한 것뿐입니다."]
        src_txt = "값의 출처: 표시 없음 = 로그에서 계산"
        if "json" in used:
            src_txt += ",  † = 시뮬레이션의 vtol_metrics.json (로그에 없는 값)"
        if "spec" in used:
            src_txt += ",  * = 비교 사양(spec)에 적힌 값"
        notes.append(src_txt)
        if self.flights[0].height_src != self.flights[1].height_src:
            notes.append("높이: 두 로그의 높이 기록 방식이 다름 (h_true / 배기구 z)")
        notes += self.auto_notes
        if self.v08:
            notes.append("GPS: " + "  ".join(c + "." for c in sv.gps_caveats("ko")))
        notes += list(sp.notes)
        for nt in notes:
            y = draw_wrapped(d, (x, y), nt, F(17 * s), (190, 198, 210), lay.W - 2 * x, 25 * s, 3)
        y += 8 * s
        y = self._honesty_box(d, x, y, lay.W - 2 * x) + 14 * s
        b, a = self.flights
        src_lines = [f"로그: {os.path.basename(b.path)} (sha256 {b.log.sha256[:12]})  /  {os.path.basename(a.path)} "
                     f"(sha256 {a.log.sha256[:12]})"]
        if sp.sources:
            src_lines.append("변경 기록: " + "  ·  ".join(sp.sources))
        for ln in src_lines:
            y = draw_wrapped(d, (x, y), ln, F(15 * s), (150, 160, 175), lay.W - 2 * x, 22 * s, 2)
        return np.asarray(im)

    def poster_index(self) -> int:
        if self.spec.poster_tau is not None:
            return int(np.clip(np.searchsorted(self.tau, self.spec.poster_tau), 0, self.tau.size - 1))
        a = self.flights[1].log
        hold = np.flatnonzero(a.phase == PHASE_HOLD)
        if hold.size:
            tau_mid = float(0.5 * (a.t[hold[0]] + a.t[hold[-1]])) - self.offsets[1]
            return int(np.clip(np.searchsorted(self.tau, tau_mid), 0, self.tau.size - 1))
        return self.tau.size // 2

    def close(self) -> None:
        for sd in self.sides:
            sd.close()


# --------------------------------------------------------------------------- #
# timing-diagram mode
# --------------------------------------------------------------------------- #
KINDS = ("measured", "derived", "analog", "assumed", "unmeasured")
KIND_KO = {"measured": "측정", "derived": "측정값에서 계산", "analog": "유사 빌드 측정", "assumed": "모델 가정",
           "unmeasured": "미측정"}


@dataclass
class TimingSegment:
    label: str
    us: float | None
    kind: str
    detail: str = ""


@dataclass
class TimingBar:
    segments: list[TimingSegment]
    note: str = ""
    total_kind: str | None = None      # the kind of the TOTAL when it differs from its parts (a measured total split
                                       # into a measured part and a derived remainder)

    @property
    def total_us(self) -> float | None:
        if not self.segments or any(sg.us is None for sg in self.segments):
            return None
        return float(sum(sg.us for sg in self.segments))


@dataclass
class TimingPath:
    name: str
    before: TimingBar
    after: TimingBar
    limit_us: float | None = None
    limit_label: str = ""


@dataclass
class TimingSpec:
    title: str
    subtitle: str
    tick_us: float
    axis_max_us: float
    order_before: list[str]
    order_after: list[str]
    highlight: list[str]
    paths: list[TimingPath]
    limits: list[tuple[float, str]]
    notes: list[str]
    sources: list[str]
    before_label: str = "변경 전"
    after_label: str = "변경 후"
    isolation_note: str = ""
    title_s: float = 6.0
    sweep_s: float = 10.0
    hold_s: float = 10.0


def _parse_bar(d: Any, where: str) -> TimingBar:
    if not isinstance(d, dict):
        raise ValueError(f"{where} must be an object with 'segments'")
    segs = []
    for i, sg in enumerate(d.get("segments", [])):
        if not isinstance(sg, dict) or "label" not in sg:
            raise ValueError(f"{where}.segments[{i}] needs a label")
        kind = str(sg.get("kind", "measured"))
        if kind not in KINDS:
            raise ValueError(f"{where}.segments[{i}].kind must be one of {KINDS}")
        us = sg.get("us")
        if us is None and kind != "unmeasured":
            raise ValueError(f"{where}.segments[{i}]: 'us' is required unless kind is 'unmeasured'")
        if us is not None and (not isinstance(us, (int, float)) or us < 0):
            raise ValueError(f"{where}.segments[{i}].us must be a number >= 0")
        segs.append(TimingSegment(str(sg["label"]), None if us is None else float(us), kind, str(sg.get("detail", ""))))
    tk = d.get("total_kind")
    if tk is not None and tk not in KINDS:
        raise ValueError(f"{where}.total_kind must be one of {KINDS}")
    return TimingBar(segs, str(d.get("note", "")), tk)


def parse_timing_spec(d: dict[str, Any]) -> TimingSpec:
    if not isinstance(d, dict) or not str(d.get("title", "")).strip():
        raise ValueError("timing spec needs a title")
    paths = []
    for i, p in enumerate(d.get("paths", [])):
        if "name" not in p:
            raise ValueError(f"paths[{i}] needs a name")
        lim = p.get("limit_us")
        paths.append(TimingPath(str(p["name"]), _parse_bar(p.get("before", {}), f"paths[{i}].before"),
                                _parse_bar(p.get("after", {}), f"paths[{i}].after"),
                                None if lim is None else float(lim), str(p.get("limit_label", ""))))
    if not paths:
        raise ValueError("timing spec needs at least one path")
    order = d.get("order", {}) or {}
    tick = float(d.get("tick_us", 4000.0))
    longest = max([b.total_us or 0.0 for p in paths for b in (p.before, p.after)] + [tick])
    ax_max = float(d.get("axis_max_us", 1000.0 * math.ceil(1.12 * longest / 1000.0)))
    lims = [(float(x["us"]), str(x.get("label", ""))) for x in d.get("limits", [])]
    return TimingSpec(str(d["title"]), str(d.get("subtitle", "")), tick, ax_max,
                      [str(x) for x in order.get("before", [])], [str(x) for x in order.get("after", [])],
                      [str(x) for x in order.get("highlight", [])], paths, lims,
                      _str_list(d.get("notes"), "notes"), _str_list(d.get("sources"), "sources"),
                      str(d.get("before_label", "변경 전")), str(d.get("after_label", "변경 후")),
                      str(d.get("isolation_note", "")), float(d.get("title_s", 6.0)), float(d.get("sweep_s", 10.0)),
                      float(d.get("hold_s", 10.0)))


def total_qualifier(bar: TimingBar) -> str:
    """How the bar's total is known, in Korean: its ``total_kind`` when given, else the common kind of its segments
    ('혼합' when they differ)."""
    if bar.total_kind:
        return KIND_KO[bar.total_kind]
    kinds = {sg.kind for sg in bar.segments}
    return KIND_KO[kinds.pop()] if len(kinds) == 1 else "혼합"


def sweep_us(frame_k: int, fps: float, sweep_s: float, axis_max_us: float) -> float:
    """The animated cursor (time since the IMU sample, us) of body frame k: linear 0 -> axis max over sweep_s."""
    return float(np.clip(frame_k / max(1e-9, fps * sweep_s), 0.0, 1.0)) * axis_max_us


class TimingRenderer:
    def __init__(self, spec: TimingSpec, fps: float = 30.0, size: tuple[int, int] = (1920, 1080)):
        self.sp, self.fps = spec, float(fps)
        self.W, self.H = int(size[0]) // 2 * 2, int(size[1]) // 2 * 2
        self.s = self.H / 1080.0
        self.n_body = int(round((spec.sweep_s + spec.hold_s) * fps))

    def _x(self, us: float) -> float:
        s = self.s
        x0, x1 = 560 * s, self.W - 400 * s
        return x0 + min(max(us, 0.0), self.sp.axis_max_us) / self.sp.axis_max_us * (x1 - x0)

    def _base(self) -> tuple[Any, Any]:
        from PIL import Image, ImageDraw
        im = Image.new("RGB", (self.W, self.H), sv.hex_rgb(sv.SURFACE))
        d = ImageDraw.Draw(im, "RGBA")
        s = self.s
        d.rectangle([0, 0, self.W, 92 * s], fill=BG_DARK)
        tag = draw_sim_tag(d, self.W - 16 * s, 12 * s, s)
        fnt = F(30 * s, True, self.sp.title)
        title = self.sp.title
        while dlen(d, title, font=fnt) > tag[0] - 50 * s and len(title) > 10:
            title = title[:-2]
        dtext(d, (24 * s, 10 * s), title, fill=(255, 255, 255), font=fnt)
        if self.sp.subtitle:
            dtext(d, (26 * s, 56 * s), self.sp.subtitle, fill=(190, 200, 214), font=F(17 * s))
        return im, d

    def _seg_fill(self, kind: str, side: int) -> tuple[tuple[int, int, int, int], bool]:
        base = sv.hex_rgb(BEFORE_C if side == 0 else AFTER_C)
        if kind == "measured":
            return base + (255,), False
        if kind == "derived":
            return tuple(int(0.55 * c + 0.45 * 255) for c in base) + (255,), False
        if kind == "analog":
            return tuple(int(0.35 * c + 0.65 * 255) for c in base) + (255,), True
        if kind == "assumed":
            return (255, 255, 255, 255), True
        return (255, 255, 255, 0), False

    def frame(self, k: int) -> np.ndarray:
        sp, s = self.sp, self.s
        im, d = self._base()
        cur = sweep_us(k, self.fps, sp.sweep_s, sp.axis_max_us)
        done = k >= int(round(sp.sweep_s * self.fps))
        # job order of one FCU tick
        y = 112 * s
        dtext(d, (40 * s, y), f"FCU 한 틱 ({sp.tick_us / 1000.0:g} ms) 안의 실행 순서", fill=sv.hex_rgb(sv.INK), font=F(23 * s, True))
        y += 40 * s
        n_jobs = max(len(sp.order_before), len(sp.order_after), 1)
        # the job boxes light up one after another over the first 40 % of the sweep: an order, not a time axis
        step = min(1.0, max(0.0, k / max(1.0, 0.4 * sp.sweep_s * self.fps)))
        for side, (lab, order) in enumerate(((sp.before_label, sp.order_before), (sp.after_label, sp.order_after))):
            col = sv.hex_rgb(BEFORE_C if side == 0 else AFTER_C)
            dtext(d, (40 * s, y + 12 * s), lab, fill=col, font=F(20 * s, True, lab))
            x = 330 * s
            bw = min(250 * s, (self.W - x - 40 * s) / n_jobs - 46 * s)
            active = int(min(len(order) - 1, math.floor(step * len(order)))) if order else -1
            for j, job in enumerate(order):
                hi = job in sp.highlight
                fill = col + (255,) if j <= active or done else (235, 237, 240, 255)
                fg = (255, 255, 255) if (j <= active or done) else (60, 60, 60)
                d.rounded_rectangle([x, y, x + bw, y + 50 * s], radius=int(8 * s), fill=fill,
                                    outline=(220, 70, 50, 255) if hi else (150, 150, 150, 255), width=max(2, int((4 if hi else 1) * s)))
                fj = F(18 * s, True, job)
                txt = f"{j + 1}. {job}"
                while dlen(d, txt, font=fj) > bw - 16 * s and len(txt) > 4:
                    txt = txt[:-2]
                dtext(d, (x + 10 * s, y + 13 * s), txt, fill=fg, font=fj)
                if j < len(order) - 1:
                    ax_ = x + bw + 8 * s
                    d.line([ax_, y + 25 * s, ax_ + 26 * s, y + 25 * s], fill=(120, 120, 120), width=max(2, int(2 * s)))
                    d.polygon([(ax_ + 30 * s, y + 25 * s), (ax_ + 20 * s, y + 18 * s), (ax_ + 20 * s, y + 32 * s)],
                              fill=(120, 120, 120))
                x += bw + 46 * s
            y += 66 * s
        if sp.highlight:
            dtext(d, (330 * s, y - 4 * s), "빨간 테두리 = " + ", ".join(sp.highlight) + " (이 변경의 핵심 출력)",
                   fill=(200, 70, 50), font=F(16 * s))
        # bars
        y = 330 * s
        dtext(d, (40 * s, y), "IMU 샘플 기준 경과 시간 [µs]  (막대 = 경로 시간의 구성, 칸의 순서는 합계에 영향 없음)",
               fill=sv.hex_rgb(sv.INK), font=F(21 * s, True))
        y += 44 * s
        axis_y = y
        # axis + ticks
        n_rows = len(sp.paths) * 2
        rows_h = n_rows * 52 * s + len(sp.paths) * 34 * s
        for us in np.arange(0.0, sp.axis_max_us + 1e-6, 500.0):
            xx = self._x(float(us))
            d.line([xx, axis_y, xx, axis_y + rows_h], fill=sv.hex_rgb(sv.GRID) + (255,), width=1)
            if int(us) % 1000 == 0:
                lab = f"{us / 1000.0:g} ms"
                dtext(d, (xx - dlen(d, lab, font=F(14 * s)) / 2, axis_y + rows_h + 6 * s), lab,
                       fill=sv.hex_rgb(sv.INK2), font=F(14 * s))
        # limit lines: the tick + spec limits
        lims = [(sp.tick_us, f"FCU 틱 {sp.tick_us / 1000.0:g} ms")] + list(sp.limits)
        for i, (us, lab) in enumerate(lims):
            xx = self._x(us)
            col = (200, 50, 40) if i == 0 else (150, 80, 160)
            for yy in np.arange(axis_y - 6 * s, axis_y + rows_h, 12 * s):
                d.line([xx, yy, xx, yy + 7 * s], fill=col, width=max(2, int(2 * s)))
            fl = F(15 * s, True, lab)
            tw = dlen(d, lab, font=fl)
            dtext(d, (min(xx - tw / 2, self.W - tw - 10 * s), axis_y - 26 * s - (18 * s if i % 2 else 0)), lab, fill=col, font=fl)
        # cursor (under the bars and labels; gone once the sweep is complete)
        xc = self._x(cur)
        if not done:
            d.line([xc, axis_y - 4 * s, xc, axis_y + rows_h], fill=(20, 20, 20), width=max(2, int(2 * s)))
        yy = axis_y + 8 * s
        for p in sp.paths:
            dtext(d, (40 * s, yy), p.name, fill=sv.hex_rgb(sv.INK), font=F(17 * s, True, p.name))
            yy += 26 * s
            for side, (lab, bar) in enumerate(((sp.before_label, p.before), (sp.after_label, p.after))):
                self._draw_bar(d, side, lab, bar, yy, cur, p.limit_us, done)
                yy += 52 * s
            yy += 8 * s
        lab = f"샘플 후 {cur:4.0f} µs"
        pill(d, (self._x(sp.axis_max_us) + 30 * s, axis_y - 50 * s), lab, F(16 * s, True), (20, 20, 20, 255),
             (255, 255, 255, 255), pad=(10 * s, 4 * s), radius=6 * s)
        # legend + notes
        ly = axis_y + rows_h + 38 * s
        lx = 40 * s
        for kind in KINDS:
            fill, hatch = self._seg_fill(kind, 1)
            d.rectangle([lx, ly, lx + 34 * s, ly + 20 * s], fill=fill, outline=(90, 90, 90),
                        width=max(1, int(s)))
            if hatch:
                self._hatch(d, (lx, ly, lx + 34 * s, ly + 20 * s), (90, 90, 90))
            if kind == "unmeasured":
                self._dashed_rect(d, (lx, ly, lx + 34 * s, ly + 20 * s), (90, 90, 90))
            dtext(d, (lx + 42 * s, ly), KIND_KO[kind], fill=sv.hex_rgb(sv.INK2), font=F(16 * s))
            lx += 42 * s + dlen(d, KIND_KO[kind], font=F(16 * s)) + 34 * s
        ny = ly + 34 * s
        for nt in sp.notes:
            ny = draw_wrapped(d, (40 * s, ny), "• " + nt, F(16 * s), sv.hex_rgb(sv.INK2), self.W - 80 * s, 23 * s, 2)
            if ny > self.H - 30 * s:
                break
        return np.asarray(im)

    @staticmethod
    def _hatch(d: Any, box: tuple[float, float, float, float], col: tuple[int, int, int], gap: float = 9.0) -> None:
        x0, y0, x1, y1 = box
        h = y1 - y0
        x = x0 - h
        while x < x1:
            a = (max(x, x0), y0 + (max(x, x0) - x))
            b = (min(x + h, x1), y0 + (min(x + h, x1) - x))
            if a[0] < b[0]:
                d.line([a, b], fill=col + (200,), width=1)
            x += gap

    @staticmethod
    def _dashed_rect(d: Any, box: tuple[float, float, float, float], col: tuple[int, int, int]) -> None:
        x0, y0, x1, y1 = box
        for xa in np.arange(x0, x1, 10):
            d.line([xa, y0, min(xa + 5, x1), y0], fill=col, width=2)
            d.line([xa, y1, min(xa + 5, x1), y1], fill=col, width=2)
        for ya in np.arange(y0, y1, 10):
            d.line([x0, ya, x0, min(ya + 5, y1)], fill=col, width=2)
            d.line([x1, ya, x1, min(ya + 5, y1)], fill=col, width=2)

    def _draw_bar(self, d: Any, side: int, lab: str, bar: TimingBar, y: float, cur: float, limit: float | None,
                  done: bool) -> None:
        s = self.s
        col = sv.hex_rgb(BEFORE_C if side == 0 else AFTER_C)
        dtext(d, (60 * s, y + 10 * s), lab, fill=col, font=F(17 * s, True, lab))
        h = 40 * s
        x = 0.0
        if not bar.segments or bar.total_us is None and all(sg.us is None for sg in bar.segments):
            note = bar.note or "미측정"
            box = (self._x(0.0), y, self._x(0.0) + 520 * s, y + h)
            self._dashed_rect(d, box, (140, 140, 140))
            dtext(d, (box[0] + 12 * s, y + 9 * s), note, fill=(110, 110, 110), font=F(16 * s, False, note))
            return
        for sg in bar.segments:
            if sg.us is None:
                bx0 = self._x(x)
                box = (bx0, y, bx0 + 200 * s, y + h)
                self._dashed_rect(d, box, (140, 140, 140))
                dtext(d, (bx0 + 8 * s, y + 9 * s), sg.label, fill=(110, 110, 110), font=F(15 * s, False, sg.label))
                continue
            a, b = x, x + sg.us
            x = b
            if cur <= a:
                continue
            bx0, bx1 = self._x(a), self._x(min(b, cur))
            fill, hatch = self._seg_fill(sg.kind, side)
            d.rectangle([bx0, y, bx1, y + h], fill=fill, outline=col + (255,), width=max(1, int(2 * s)))
            if hatch:
                self._hatch(d, (bx0, y, bx1, y + h), col)
            if cur >= b:
                txt = f"{sg.label} {sg.us:.0f} µs"
                ff = F(15 * s, True, txt)
                if dlen(d, txt, font=ff) > bx1 - bx0 - 12 * s:
                    txt = f"{sg.us:.0f}"
                if dlen(d, txt, font=ff) <= bx1 - bx0 - 8 * s:
                    tc = (255, 255, 255) if sg.kind == "measured" else (25, 25, 25)
                    dtext(d, (bx0 + 8 * s, y + 10 * s), txt, fill=tc, font=ff)
        tot = bar.total_us
        if tot is not None and cur >= tot:
            over = limit is not None and tot > limit
            txt = f"합계 {tot:.0f} µs ({total_qualifier(bar)})" + (f"  ▲ {limit:.0f} µs 초과" if over else "")
            ff = F(16 * s, True, txt)
            xt = self._x(tot) + 10 * s
            fn = F(14 * s)
            wl = max(dlen(d, txt, ff), dlen(d, bar.note, fn) if bar.note else 0.0)
            d.rectangle([xt - 4 * s, y, xt + wl + 4 * s, y + h], fill=(252, 252, 251, 235))
            dtext(d, (xt, y + (1 if bar.note else 10) * s), txt, fill=(200, 50, 40) if over else sv.hex_rgb(sv.INK), font=ff)
            if bar.note:
                dtext(d, (xt, y + 21 * s), bar.note, fill=sv.hex_rgb(sv.INK2), font=fn)

    def title_card(self) -> np.ndarray:
        from PIL import Image, ImageDraw
        sp, s = self.sp, self.s
        im = Image.new("RGB", (self.W, self.H), BG_DARK)
        d = ImageDraw.Draw(im, "RGBA")
        draw_sim_tag(d, self.W - 24 * s, 22 * s, s)
        x, mx = 110 * s, self.W - 220 * s
        y = 60 * s
        dtext(d, (x, y), "CanSat EDF 수직이착륙 기체 · 제어 변경 기록 (FCU 타이밍)", fill=(150, 190, 255), font=F(24 * s))
        y = draw_wrapped(d, (x, y + 46 * s), sp.title, F(44 * s, True, sp.title), (255, 255, 255), mx, 58 * s, 2)
        if sp.subtitle:
            y = draw_wrapped(d, (x, y + 4 * s), sp.subtitle, F(23 * s), (200, 208, 220), mx, 32 * s, 2)
        y += 30 * s
        for side, (lab, order) in enumerate(((sp.before_label, sp.order_before), (sp.after_label, sp.order_after))):
            col = sv.hex_rgb(BEFORE_C if side == 0 else AFTER_C)
            dtext(d, (x, y), lab, fill=tuple(min(255, c + 60) for c in col), font=F(26 * s, True, lab))
            y = draw_wrapped(d, (x + 30 * s, y + 40 * s), "  →  ".join(order) or "-", F(24 * s), (235, 238, 242),
                             self.W - 2 * x, 34 * s, 2) + 18 * s
        y += 10 * s
        dtext(d, (x, y), "값의 종류", fill=(255, 205, 120), font=F(23 * s, True))
        y += 36 * s
        y = draw_wrapped(d, (x + 24 * s, y), "측정 = co-simulation / simbench 측정값.  측정값에서 계산 = 두 측정값의 차.  "
                         "유사 빌드 측정 = 같은 경로를 가진 다른 빌드의 측정값.  가정 (모델) = 6-DOF 시뮬레이션이 쓰는 값.  "
                         "미측정 = 아직 측정하지 않음.", F(20 * s), (220, 228, 238), self.W - 2 * x - 24 * s, 29 * s, 4)
        if sp.isolation_note:
            y += 16 * s
            draw_wrapped(d, (x, y), sp.isolation_note, F(20 * s), (255, 205, 120), self.W - 2 * x, 29 * s, 4)
        dtext(d, (x, self.H - 50 * s), "실제 비행 영상이 아닌 타이밍 분석 도식입니다. 모든 수치의 출처는 화면 아래에 적혀 있습니다.",
               fill=(255, 150, 140), font=F(19 * s, True))
        return np.asarray(im)

    def end_card(self) -> np.ndarray:
        from PIL import Image, ImageDraw
        sp, s = self.sp, self.s
        im = Image.new("RGB", (self.W, self.H), BG_DARK)
        d = ImageDraw.Draw(im, "RGBA")
        draw_sim_tag(d, self.W - 24 * s, 22 * s, s)
        x = 100 * s
        dtext(d, (x, 50 * s), "요약 및 출처", fill=(255, 255, 255), font=F(36 * s, True))
        y = 120 * s
        for p in sp.paths:
            tb, ta = p.before.total_us, p.after.total_us
            tb_s = f"{tb:.0f} µs ({total_qualifier(p.before)})" if tb is not None else "미측정"
            ta_s = f"{ta:.0f} µs ({total_qualifier(p.after)})" if ta is not None else "미측정"
            dtext(d, (x, y), p.name, fill=(160, 200, 255), font=F(22 * s, True, p.name))
            y += 34 * s
            dtext(d, (x + 30 * s, y), f"{sp.before_label}: {tb_s}      {sp.after_label}: {ta_s}"
                   + (f"      한계 {p.limit_us:.0f} µs ({p.limit_label})" if p.limit_us is not None else ""),
                   fill=(230, 234, 240), font=F(20 * s))
            y += 42 * s
        y += 10 * s
        dtext(d, (x, y), "출처", fill=(255, 205, 120), font=F(22 * s, True))
        y += 34 * s
        for src in sp.sources:
            y = draw_wrapped(d, (x + 20 * s, y), "• " + src, F(17 * s), (200, 208, 220), self.W - 2 * x - 20 * s, 25 * s, 3)
        return np.asarray(im)


# --------------------------------------------------------------------------- #
# output
# --------------------------------------------------------------------------- #
def _json_default(o: Any) -> Any:
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return str(o)


def render_compare(before_path: str, after_path: str, spec: CompareSpec, out: str, align: str | None = None,
                   channels: Sequence[str] | None = None, fps: float = 30.0, size: tuple[int, int] = (1920, 1080),
                   speed: float = 1.0, start: float | None = None, end: float | None = None, preview: int = 0,
                   at: Sequence[float] | None = None, style: str = "realistic", scene: str = "auto",
                   framing: str | None = None, azimuth: float | None = None, elevation: float | None = None,
                   sync: str | None = None, crf: int = 20, aa: str = "msaa", title_s: float | None = None,
                   end_s: float | None = None, quiet: bool = False, asset_dir: str | None = sv.ASSET_DIR,
                   layout: str | None = None, ghost_align: str | None = None) -> dict[str, Any]:
    t_wall0 = time.perf_counter()
    al = parse_align(align or spec.align)
    chans = parse_channels(channels) if channels else spec.channels
    b, a = load_flight(before_path), load_flight(after_path)
    rnd = CompareRenderer(b, a, spec, al, chans, fps, speed, size, start, end, style, scene, framing, azimuth,
                          elevation, sync, aa, asset_dir, layout=layout, ghost_align=ghost_align)
    t_setup = time.perf_counter() - t_wall0
    stem, _ = os.path.splitext(out)
    info: dict[str, Any] = {
        "tool": "tools/compare_video.py", "spec": {k: v for k, v in spec.__dict__.items() if k != "base_dir"},
        "before": {"path": b.path, "sha256": b.log.sha256, "scenario": b.log.meta.get("scenario"),
                   "params_sha256": b.log.meta.get("params_sha256"), "height_source": b.height_src},
        "after": {"path": a.path, "sha256": a.log.sha256, "scenario": a.log.meta.get("scenario"),
                  "params_sha256": a.log.meta.get("params_sha256"), "height_source": a.height_src},
        "align": {"spec": align or spec.align, "label": al.label, "offsets_s": list(rnd.offsets)},
        "window_tau_s": [rnd.tau0, rnd.tau1], "fps": fps, "speed": speed, "size": [rnd.lay.W, rnd.lay.H],
        "channels": list(chans), "style": style, "layout": rnd.layout, "camera": rnd.camera_info,
        "setup_s": round(t_setup, 3), "hold_reference": {"before": b.hold[1], "after": a.hold[1]},
        "ghost_shift_m": rnd.shift_xy.tolist(),
        "metrics": {side: {"values": m["values"], "sources": m["sources"], "computed_from_log": m["computed"],
                           "metrics_json": m["json"]} for side, m in zip(("before", "after"), rnd.metrics)},
        "frozen_frames": {"before": int(np.sum(rnd.maps[0][1] != RUN)), "after": int(np.sum(rnd.maps[1][1] != RUN))},
    }
    files: list[str] = []
    from PIL import Image
    try:
        if preview or at:
            outdir = stem + "_frames"
            os.makedirs(outdir, exist_ok=True)
            if at:
                ks = sorted({int(np.clip(np.searchsorted(rnd.tau, float(x)), 0, rnd.tau.size - 1)) for x in at})
            else:
                ks = sorted({int(round(x)) for x in np.linspace(0, rnd.tau.size - 1, int(preview))})
            t_r = time.perf_counter()
            n_sides = len(rnd.sides)
            views: list[list[np.ndarray]] = [[] for _ in range(n_sides)]
            t_pass = [rnd.render_pass(side, ks, lambda j, img, side=side: views[side].append(img)) for side in range(n_sides)]
            for j, k in enumerate(ks):
                p = os.path.join(outdir, f"frame_{k:05d}_tau{rnd.tau[k]:+08.3f}.png")
                Image.fromarray(rnd.frame(k, tuple(v[j] for v in views))).save(p)
                files.append(os.path.abspath(p))
            dt = time.perf_counter() - t_r
            for name, fn in (("card_title.png", rnd.title_card), ("card_end.png", rnd.end_card)):
                p = os.path.join(outdir, name)
                Image.fromarray(fn()).save(p)
                files.append(os.path.abspath(p))
            info.update({"frames": len(ks), "render_s": round(dt, 3), "s_per_frame": round(dt / max(1, len(ks)), 4),
                         "pass_s": [round(x, 3) for x in t_pass]})
        else:
            n_title = int(round((spec.title_s if title_s is None else title_s) * fps))
            n_end = int(round((spec.end_s if end_s is None else end_s) * fps))
            n_body = rnd.tau.size
            ks = list(range(n_body))
            vh, vw = rnd.lay.view_h, rnd.lay.view_w
            caches: list[Any] = []
            t_r = time.perf_counter()
            t_pass = []
            pass_names = ("before", "after") if len(rnd.sides) == 2 else ("shared",)
            for side in range(len(rnd.sides)):
                path = f"{stem}_cache_{pass_names[side]}.npy"
                caches.append(path)
                mm = np.lib.format.open_memmap(path, mode="w+", dtype=np.uint8, shape=(n_body, vh, vw, 3))

                def prog(j: int, n: int, side: int = side, t0: float = time.perf_counter()) -> None:
                    if not quiet and j % max(1, int(fps * 10)) == 0:
                        el = time.perf_counter() - t0
                        print(f"  3-D pass {pass_names[side]}: {j}/{n}  {el / max(1, j):.3f} s/view", flush=True)
                # the sink is a temporary: nothing may keep the write map open (Windows cannot delete a mapped file)
                t_pass.append(rnd.render_pass(side, ks, lambda j, img, mm=mm: mm.__setitem__(j, img[:vh, :vw, :3]), prog))
                mm.flush()
                del mm
            mms = [np.load(p, mmap_mode="r") for p in caches]
            timing = {"compose_s": 0.0}
            k_p = rnd.poster_index()
            poster = rnd.frame(k_p, tuple(mm[k_p] for mm in mms))

            def gen():
                if n_title:
                    tc = rnd.title_card()
                    for _ in range(n_title):
                        yield tc
                tb = time.perf_counter()
                for k in range(n_body):
                    img = rnd.frame(k, tuple(mm[k] for mm in mms))
                    if not quiet and k % max(1, int(fps * 10)) == 0:
                        el = time.perf_counter() - tb
                        print(f"  compose {k}/{n_body}  tau = {rnd.tau[k]:+.2f} s  {el / max(1, k):.3f} s/frame", flush=True)
                    yield img
                timing["compose_s"] = time.perf_counter() - tb
                if n_end:
                    ec = rnd.end_card()
                    for _ in range(n_end):
                        yield ec
            try:
                n = sv.write_mp4(out, gen, (rnd.lay.W, rnd.lay.H), fps, crf)
            finally:
                del mms
                import gc
                gc.collect()
                for p in caches:
                    with contextlib.suppress(OSError):
                        os.remove(p)
            dt = time.perf_counter() - t_r
            files.append(os.path.abspath(out))
            pp = stem + "_poster.png"
            Image.fromarray(poster).save(pp)
            files.append(os.path.abspath(pp))
            body = sum(t_pass) + timing["compose_s"]
            info.update({"frames": n, "body_frames": n_body, "title_frames": n_title, "end_frames": n_end,
                         "video_s": round(n / fps, 3), "render_s": round(dt, 3),
                         "s_per_frame": round(body / max(1, n_body), 4),
                         "s_per_frame_parts": {**{f"3d_{nm}": round(tp / max(1, n_body), 4)
                                                  for nm, tp in zip(pass_names, t_pass)},
                                               "compose_encode": round(timing["compose_s"] / max(1, n_body), 4)},
                         "poster_tau": float(rnd.tau[k_p]), "reused_3d_frames": [sd.reused for sd in rnd.sides]})
    finally:
        rnd.close()
    info["warnings"] = rnd.warnings + [f"plot: {m}" for m in rnd.strip.missing]
    info["files"] = files
    info["wall_s"] = round(time.perf_counter() - t_wall0, 3)
    side = stem + ("_frames.json" if (preview or at) else ".json")
    with open(side, "w", encoding="utf-8") as f:
        json.dump(info, f, indent=1, ensure_ascii=False, default=_json_default)
    info["sidecar"] = side
    return info


def render_timing(spec_path: str, out: str, fps: float = 30.0, size: tuple[int, int] = (1920, 1080), preview: int = 0,
                  crf: int = 20) -> dict[str, Any]:
    t0 = time.perf_counter()
    with open(spec_path, "r", encoding="utf-8") as f:
        sp = parse_timing_spec(json.load(f))
    rnd = TimingRenderer(sp, fps, size)
    stem, _ = os.path.splitext(out)
    files: list[str] = []
    from PIL import Image
    info: dict[str, Any] = {"tool": "tools/compare_video.py --timing", "spec": os.path.abspath(spec_path), "fps": fps,
                            "size": [rnd.W, rnd.H]}
    if preview:
        outdir = stem + "_frames"
        os.makedirs(outdir, exist_ok=True)
        ks = sorted({int(round(x)) for x in np.linspace(0, rnd.n_body - 1, int(preview))})
        tr = time.perf_counter()
        for k in ks:
            p = os.path.join(outdir, f"frame_{k:05d}.png")
            Image.fromarray(rnd.frame(k)).save(p)
            files.append(os.path.abspath(p))
        info["s_per_frame"] = round((time.perf_counter() - tr) / max(1, len(ks)), 4)
        for name, fn in (("card_title.png", rnd.title_card), ("card_end.png", rnd.end_card)):
            p = os.path.join(outdir, name)
            Image.fromarray(fn()).save(p)
            files.append(os.path.abspath(p))
    else:
        n_t, n_e = int(round(sp.title_s * fps)), int(round(max(6.0, sp.hold_s * 0.6) * fps))
        tb: dict[str, float] = {}

        def gen():
            tc = rnd.title_card()
            for _ in range(n_t):
                yield tc
            t1 = time.perf_counter()
            for k in range(rnd.n_body):
                yield rnd.frame(k)
            tb["body"] = time.perf_counter() - t1
            ec = rnd.end_card()
            for _ in range(n_e):
                yield ec
        n = sv.write_mp4(out, gen, (rnd.W, rnd.H), fps, crf)
        files.append(os.path.abspath(out))
        pp = stem + "_poster.png"
        Image.fromarray(rnd.frame(rnd.n_body - 1)).save(pp)
        files.append(os.path.abspath(pp))
        info.update({"frames": n, "video_s": round(n / fps, 3), "s_per_frame": round(tb.get("body", 0.0) / max(1, rnd.n_body), 4)})
    info["files"] = files
    info["wall_s"] = round(time.perf_counter() - t0, 3)
    side = stem + ("_frames.json" if preview else ".json")
    with open(side, "w", encoding="utf-8") as f:
        json.dump(info, f, indent=1, ensure_ascii=False, default=_json_default)
    info["sidecar"] = side
    return info


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--before", help="log .npz before the change (default: spec.before)")
    ap.add_argument("--after", help="log .npz after the change (default: spec.after)")
    ap.add_argument("--spec", help="comparison spec JSON (title, what, why, isolation, ...)")
    ap.add_argument("--timing", help="timing-diagram spec JSON (FCU tick order mode) instead of two logs")
    ap.add_argument("--out", required=True, help="output .mp4 (poster <stem>_poster.png, sidecar <stem>.json)")
    ap.add_argument("--align", default=None, help="motor_on (default) | an event name / text | offset:a,b")
    ap.add_argument("--channels", default=None, help=f"comma list of {', '.join(CHANNELS)}")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--speed", type=float, default=1.0, help="sim seconds per video second")
    ap.add_argument("--start", type=float, default=None, help="aligned time tau of the first frame")
    ap.add_argument("--end", type=float, default=None, help="aligned time tau of the last frame")
    ap.add_argument("--preview", type=int, default=0, help="N evenly spaced PNG frames + both cards instead of the video")
    ap.add_argument("--at", default="", help="comma-separated tau values: PNG frames at these aligned times")
    ap.add_argument("--style", choices=sv.STYLES, default="realistic")
    ap.add_argument("--scene", choices=sv.SCENES, default="auto")
    ap.add_argument("--framing", choices=sv.FRAMINGS, default=None)
    ap.add_argument("--camera-sync", choices=SYNCS, default=None,
                    help="scale (default): same direction + distance, each side centred on its vehicle; shared: one camera")
    ap.add_argument("--azimuth", type=float, default=None)
    ap.add_argument("--elevation", type=float, default=None)
    ap.add_argument("--layout", choices=LAYOUTS, default=None,
                    help="split (default: two views side by side) or shared (ONE scene: the after vehicle + the before "
                         "vehicle as a translucent ghost, the after hold point, a comparison table and one top view)")
    ap.add_argument("--ghost-align", choices=GHOST_ALIGNS, default=None,
                    help="shared layout: pad (default, both as flown) or hold (the before log moved so its hold point - "
                         "derived by the v0.8 rule when not logged - lies on the after hold point)")
    ap.add_argument("--title-s", type=float, default=None)
    ap.add_argument("--end-s", type=float, default=None)
    ap.add_argument("--crf", type=int, default=20)
    ap.add_argument("--aa", choices=("msaa", "fxaa", "none"), default="msaa")
    ap.add_argument("--assets", default=sv.ASSET_DIR)
    a = ap.parse_args(argv)
    if a.timing:
        info = render_timing(a.timing, a.out, a.fps, sv.parse_size(a.size), a.preview, a.crf)
        print(f"timing video: {info.get('frames', len(info['files']))} frames, {info.get('s_per_frame')} s/frame")
        for f in info["files"][:4]:
            print("  ", f)
        print("  sidecar", info["sidecar"])
        return 0
    if not a.spec:
        ap.error("--spec is required (or --timing)")
    spec = load_spec(a.spec)
    bp = a.before or spec.resolve(spec.before)
    apath = a.after or spec.resolve(spec.after)
    if not bp or not apath:
        ap.error("--before and --after (or spec.before / spec.after) are required")
    at = [float(x) for x in a.at.split(",") if x.strip()] if a.at else None
    info = render_compare(bp, apath, spec, a.out, a.align, a.channels, a.fps, sv.parse_size(a.size), a.speed, a.start,
                          a.end, a.preview, at, a.style, a.scene, a.framing, a.azimuth, a.elevation, a.camera_sync,
                          a.crf, a.aa, a.title_s, a.end_s, asset_dir=a.assets or None, layout=a.layout,
                          ghost_align=a.ghost_align)
    for w in info["warnings"]:
        print("warning:", w)
    print(f"frames {info.get('frames')}  render {info.get('render_s')} s  ({info.get('s_per_frame')} s/frame)  "
          f"setup {info['setup_s']} s")
    for f in info["files"][:3]:
        print("  ", f)
    if len(info["files"]) > 3:
        print(f"   ... {len(info['files'])} files")
    print("  sidecar", info["sidecar"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
