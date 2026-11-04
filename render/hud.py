"""Broadcast-style overlays for the Blender renders (PIL + Malgun Gothic) and the H.264 encoder.

Two variants of every frame:
  pro    analysis version: one slim lower row (<= 11 % of the frame height: 비행 단계 / 고도 + 목표 / 기울기 / 스로틀 /
         바람 + 화면 기준 방향 화살표 / 밀기 힘; 80 % size in close-ups), a vane-angle indicator (the duct seen from below,
         each vane's deflection and side-force arrow, the net correction and the roll trim - round 2: it replaced the
         unreadable 10 s sparkline),
         the honesty pill (6-DOF logs: "시뮬레이션 - 실제 비행 영상 아님"; kinematic test patterns: "운동학 합성 패턴
         (물리 시뮬레이션 아님) · CG"), a slow-motion badge, event call-outs (돌풍 / 밀기 shown while the logged event
         lasts + 0.3 s; 지상 명령 / 착지 2.6 s), sim-time code; a dark title card BEFORE the footage
         (``title_frames``; the display title / subtitle come from the shot list, the log's own title only without its
         parenthetical disclaimers - one disclaimer line + the corner tag) and a 3 s end card (key numbers incl. the
         pushes THIS clip shows, the whole-flight tilt chart drawing in, credits) after it.
  clean  cinematic version: only a small "시뮬레이션 (CG)" tag ("CG · 합성 테스트 패턴" for a kinematic test pattern,
         "CG 렌더" for the studio turntable, which no simulation drives).

Both variants get the same light vignette (``grade``).  Provenance strings come from the log's meta (``provenance``):
a synthetic / kinematic log is never called a simulation.  Every number on screen is a logged value or the formula
written next to it (``frame_values``); the text builders are pure functions (tested in tools/test_blender_render.py).
Encoding: imageio-ffmpeg's ffmpeg, H.264 High, CRF 18, yuv420p, +faststart.
"""
from __future__ import annotations

import math
import os
import subprocess
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

FONT_REG = r"C:\Windows\Fonts\malgun.ttf"
FONT_BOLD = r"C:\Windows\Fonts\malgunbd.ttf"
TAG_PRO = "시뮬레이션 - 실제 비행 영상 아님"
TAG_CLEAN = "시뮬레이션 (CG)"
TAG_RENDER = "CG 렌더"                 # the studio turntable: no simulation drives it (art review round 2)
PHASE_KO = {0: "대기", 1: "시동", 2: "상승", 3: "호버링", 4: "착륙", 5: "착지"}
STATE_KO = {0: "안전", 1: "준비", 2: "동력 비행", 3: "무동력", 4: "분리", 5: "비상정지"}
CMD_KO = {"ON": "이륙", "OFF": "착륙", "UP": "고도 +", "DOWN": "고도 -", "KILL": "긴급정지"}
PHASE_COLOR = {0: (170, 178, 189), 1: (255, 214, 102), 2: (90, 168, 255), 3: (64, 220, 150), 4: (255, 181, 71),
               5: (190, 198, 208)}
ABORT_COLOR = (255, 84, 84)
ACCENT = (255, 150, 60)
ALT_COLOR = (120, 196, 255)
SPARK_S = 10.0
EVENT_HOLD_S = 2.6            # call-out time of instantaneous events (commands, lift-off, touchdown)
EVENT_TAIL_S = 0.3            # pushes / gusts: shown while logged + this, then faded out
SCENARIO_KO = {"synthetic_push": "1.5 m 호버링 · 막대 밀기 시연", "V_nominal": "현장 비행 · 5 m 이륙, 8 m 호버링, 돌풍 3회",
               "D_demo_nominal": "시연 · 1.5 m 호버링, 막대 밀기 5회", "V_gusty_swing": "현장 비행 · 흔들리는 돌풍",
               "D_push_cg": "시연 · 무게중심 높이 밀기", "D_push_nose": "시연 · 기수 쪽 밀기", "V_kill": "현장 비행 · 긴급정지",
               "V_link_loss": "현장 비행 · 통신 두절", "V_gust_beyond_20": "현장 비행 · 한계 초과 돌풍"}


# --------------------------------------------------------------------------- #
# provenance (what produced the motion): never call a kinematic test pattern a simulation
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Provenance:
    kind: str                 # 'sim6dof' | 'synthetic' | 'log'
    tag_pro: str              # top-right pill of the analysis version
    tag_clean: str            # the cinematic version's small tag
    card_note: str            # title-card line under the subtitle
    credit: str               # end-card 'motion' credit


def provenance(meta: dict[str, Any]) -> Provenance:
    """From the log meta: 'synthetic' when the scenario / title / notes say kinematic, synthetic or test pattern;
    'sim6dof' when the notes name the 6-DOF simulation; otherwise a plain 'log'."""
    txt = " ".join(str(meta.get(k, "")) for k in ("scenario", "title", "notes", "mode")).lower()
    if any(w in txt for w in ("synthetic", "kinematic", "test pattern", "not a simulation")):
        return Provenance("synthetic", "운동학 합성 패턴 (물리 시뮬레이션 아님) · CG", "CG · 합성 테스트 패턴",
                          "운동학 합성 패턴으로 움직인 CG 영상입니다 (물리 시뮬레이션·실제 비행 아님)",
                          "운동: 운동학 합성 테스트 패턴 (물리 시뮬레이션 아님)")
    if any(w in txt for w in ("6-dof", "sim6dof", "6dof")):
        return Provenance("sim6dof", TAG_PRO, TAG_CLEAN, "6자유도 비행 시뮬레이션 로그로 렌더링한 CG 영상입니다",
                          "운동: 6자유도 비행 시뮬레이션 로그")
    return Provenance("log", TAG_PRO, TAG_CLEAN, "비행 로그로 렌더링한 CG 영상입니다 (실제 비행 영상 아님)",
                      "운동: 비행 로그")


# --------------------------------------------------------------------------- #
# data
# --------------------------------------------------------------------------- #
@dataclass
class Telemetry:
    """Log-rate history for the sparkline and the event call-outs (from render.logio.Log)."""
    t: np.ndarray
    alt: np.ndarray
    tilt: np.ndarray
    events: list[tuple[float, str, str, float]] = field(default_factory=list)   # (t, kind, Korean text, t_end)
    has_push: bool = False
    title: str = ""
    subtitle: str = ""
    summary: list[tuple[str, str]] = field(default_factory=list)
    credits: list[str] = field(default_factory=list)
    prov: Provenance = field(default_factory=lambda: provenance({}))
    marks: list[tuple[float, str]] = field(default_factory=list)       # (t, kind) of pushes / gusts for the chart


def telemetry_from_log(log: Any, events: Sequence[Any], title: str = "", subtitle: str = "",
                       person: bool = False, audio: bool = False, edit_t: np.ndarray | None = None) -> Telemetry:
    """``edit_t`` (the sim times of the edit's frames) makes the end card report the pushes THIS clip shows."""
    ev: list[tuple[float, str, str, float]] = []
    for e in events:
        txt = event_text_ko(e.kind, e.label, e.value)
        if txt:
            te = float(e.t_end) if np.isfinite(getattr(e, "t_end", float("nan"))) else float("nan")
            ev.append((float(e.t), e.kind, txt, te))
    prov = provenance(log.meta)
    return Telemetry(t=np.asarray(log.t, float), alt=np.asarray(log.pos[:, 2], float),
                     tilt=np.asarray(log.tilt_deg, float), events=ev, has_push=bool(log.has_push),
                     title=title or default_title(log), subtitle=subtitle or default_subtitle(log),
                     summary=summary_numbers(log, edit_t), credits=credit_lines(person, prov, audio), prov=prov,
                     marks=[(float(e.t), e.kind) for e in events if e.kind in ("push", "gust")])


def event_alpha(ev: tuple[float, str, str, float], t: float) -> float:
    """Opacity 0..1 of a call-out at sim time t: pushes / gusts while logged (+ EVENT_TAIL_S fade), other events
    EVENT_HOLD_S; 0.25 s fade-in."""
    t0, kind, _, te = ev
    age = t - t0
    if age < 0.0:
        return 0.0
    if kind in ("push", "gust") and np.isfinite(te):
        end = te + EVENT_TAIL_S
        if t > end:
            return 0.0
        return float(min(1.0, age / 0.25, (end - t) / EVENT_TAIL_S if t > te else 1.0))
    if age > EVENT_HOLD_S:
        return 0.0
    return float(min(1.0, age / 0.25, (EVENT_HOLD_S - age) / 0.4))


def event_text_ko(kind: str, label: str = "", value: float = float("nan")) -> str:
    if kind == "gust":
        return f"돌풍  {value:.1f} m/s" if np.isfinite(value) else "돌풍"
    if kind == "push":
        return f"밀기  {value:.0f} N" if np.isfinite(value) else "밀기"
    if kind == "cmd":
        g = CMD_KO.get(str(label).upper())
        return f"지상 명령  {label}" + (f" ({g})" if g else "")
    return {"liftoff": "이륙", "hover": "호버링 진입", "land": "착륙 시작", "touchdown": "착지", "abort": "비상정지",
            "motor_on": "모터 가동"}.get(kind, "")


def default_title(log: Any) -> str:
    """The log's Korean title without parenthetical disclaimers (the card's provenance line and the corner tag say
    it once each: round 1 showed the disclaimer three times on one card)."""
    import re
    t = str(log.meta.get("title_ko") or "")
    t = re.sub(r"\s*\([^)]*아님[^)]*\)", "", t).strip()
    if t:
        return t
    return "수직 이착륙 · 자세 제어 시뮬레이션"


def default_subtitle(log: Any) -> str:
    """'CanSat EDF 기체 · <Korean scenario name>' - a raw scenario id is never shown to the audience."""
    sc = str(log.meta.get("scenario", ""))
    ko = SCENARIO_KO.get(sc)
    if ko is None:
        prof = str(log.meta.get("profile", "")).lower()
        ko = "실내 시연" if prof == "demo" else ("현장 비행" if prof == "field" else "")
    return f"CanSat EDF 기체 · {ko}" if ko else "CanSat EDF 기체"


def push_runs_t(log: Any) -> list[tuple[float, float, float]]:
    """(t0, t1, peak force N) of every logged push."""
    a = np.asarray(log.push_active, bool)
    e = np.diff(np.concatenate([[0], a.astype(int), [0]]))
    out = []
    for i0, i1 in zip(np.flatnonzero(e == 1), np.flatnonzero(e == -1) - 1):
        out.append((float(log.t[i0]), float(log.t[i1]), float(np.max(np.linalg.norm(log.push_force[i0:i1 + 1], axis=1)))))
    return out


def summary_numbers(log: Any, edit_t: np.ndarray | None = None) -> list[tuple[str, str]]:
    """End-card key numbers, all from logged arrays: max height (pos z = duct exit above the pad top), max tilt
    while powered, time in HOLD, pushes (count, max force; with ``edit_t`` the pushes this clip shows, as 'push k of
    n'), landing vertical speed (meta.touchdown if logged)."""
    out: list[tuple[str, str]] = []
    out.append(("최고 고도", f"{float(np.max(log.pos[:, 2])):.2f} m"))
    pw = np.asarray(log.state) == 2
    tilt = np.asarray(log.tilt_deg, float)
    out.append(("최대 기울기 (동력 비행 중)", f"{float(np.max(tilt[pw])) if np.any(pw) else float(np.max(tilt)):.1f}°"))
    ph = np.asarray(log.phase)
    if np.any(ph == 3):
        dt = np.diff(np.concatenate([log.t, [log.t[-1]]]))
        out.append(("호버링 시간", f"{float(np.sum(dt[ph == 3])):.1f} s"))
    if log.has_push:
        runs = push_runs_t(log)
        shown = runs
        if edit_t is not None and len(edit_t):
            et = np.asarray(edit_t, float)
            shown = [r for r in runs if np.any((et >= r[0]) & (et <= r[1]))]
        if edit_t is not None and len(shown) < len(runs) and shown:
            idx = [runs.index(r) + 1 for r in shown]
            which = ", ".join(f"{i}" for i in idx)
            out.append((f"이 영상의 밀기 (전체 {len(runs)}회 중 {which}번째)",
                        f"{len(shown)}회 · {max(r[2] for r in shown):.0f} N"))
        else:
            fmax = max(r[2] for r in runs) if runs else 0.0
            out.append(("밀기", f"{len(runs)}회 · 최대 {fmax:.0f} N"))
    gust = np.asarray(log.gust_active, bool)
    if np.any(gust):
        out.append(("최대 돌풍", f"{float(np.max(np.linalg.norm(log.wind[gust], axis=1))):.1f} m/s"))
    td = log.meta.get("touchdown")
    if isinstance(td, dict) and "touchdown_vz" in td:
        out.append(("착지 수직 속도", f"{abs(float(td['touchdown_vz'])):.2f} m/s"))
    return out


AUDIO_CREDIT = "소리: 스로틀 로그로 합성한 EDF 효과음 (실제 녹음 아님)"


def credit_lines(person: bool, prov: Provenance | None = None, audio: bool = False) -> list[str]:
    prov = prov or provenance({"notes": "6-DOF"})
    c = [f"기체 형상: 팀 CAD (CadQuery STL) · {prov.credit} · 렌더: Blender",
         "배경 HDRI · 텍스처 · 소품: Poly Haven (CC0)"]
    if person:
        c.append("인체 모델: Blender Studio Human Base Meshes (CC0)")
    if audio:
        c.append(AUDIO_CREDIT)
    return c


@dataclass
class FrameInfo:
    t: float
    speed: float
    phase: int
    state: int
    alt: float
    h_target: float
    tilt: float
    throttle: float
    wind: np.ndarray
    push_active: bool
    push_n: float
    cam_right: np.ndarray
    cam_fwd: np.ndarray
    frame: int = 0
    slow_factor: float = float("nan")    # the shot's slow-motion window factor (badge), nan outside
    n_frames: int = 1
    delta: np.ndarray | None = None      # (4,) vane deflections (rad), body frame
    kind: str = ""                       # the shot's camera kind (close-ups get a compact HUD)


def frames_from_plan(P: dict[str, np.ndarray], shots: list[dict] | None = None) -> list[FrameInfo]:
    """Per video frame values from the frame plan (render.plan.build_plan); ``shots`` (plan meta) gives the
    slow-motion window factor shown on the badge (the instantaneous speed ramps through other values)."""
    from .logio import quat_to_matrix
    n = int(P["t"].shape[0])
    Rc = quat_to_matrix(P["cam_quat"])
    pa = P.get("push_active")
    pf = P.get("push_force")
    out = []
    for k in range(n):
        out.append(FrameInfo(t=float(P["t"][k]), speed=float(P["speed"][k]), phase=int(P["phase"][k]),
                             state=int(P["state"][k]), alt=float(P["pos"][k, 2]), h_target=float(P["h_target"][k]),
                             tilt=float(P["tilt_deg"][k]), throttle=float(P["throttle"][k]), wind=np.asarray(P["wind"][k]),
                             push_active=bool(pa[k]) if pa is not None else False,
                             push_n=float(np.linalg.norm(pf[k])) if pf is not None else 0.0,
                             cam_right=Rc[k][:, 0], cam_fwd=-Rc[k][:, 2], frame=k, n_frames=n,
                             slow_factor=_slow_factor(shots, int(P["seg"][k]), float(P["t"][k])) if shots else float("nan"),
                             delta=np.asarray(P["delta"][k], float) if "delta" in P else None,
                             kind=str(shots[int(P["seg"][k])].get("cam", "")) if shots and 0 <= int(P["seg"][k]) < len(shots) else ""))
    return out


def _slow_factor(shots: list[dict], seg: int, t: float) -> float:
    if seg < 0 or seg >= len(shots):
        return float("nan")
    for w in shots[seg].get("slow", []):
        if float(w["a"]) <= t <= float(w["b"]):
            return float(w["factor"])
    return float("nan")


def frame_values(fi: FrameInfo) -> dict[str, str]:
    """The lower-third texts of one frame (pure; units and formulas fixed here)."""
    ph = STATE_KO[5] if fi.state == 5 else PHASE_KO.get(fi.phase, f"P{fi.phase}")
    wh = float(np.hypot(fi.wind[0], fi.wind[1]))
    v = {"phase": ph, "alt": f"{fi.alt:.2f} m",
         "target": f"목표 {fi.h_target:.1f} m" if np.isfinite(fi.h_target) else "",
         "tilt": f"{fi.tilt:.1f}°", "thr": f"{100.0 * fi.throttle:.0f} %",
         "wind": "무풍" if wh < 0.05 else f"{wh:.1f} m/s",
         "push": f"{fi.push_n:.0f} N" if fi.push_active else "—",
         "time": f"T+ {fi.t:.1f} s",
         "slow": (f"슬로모션  {(fi.slow_factor if np.isfinite(fi.slow_factor) else fi.speed):.2g}×"
                  if fi.speed < 0.97 else "")}
    return v


def wind_screen_angle(fi: FrameInfo) -> float:
    """Direction the air moves, as seen from the camera: 0 = screen right, pi/2 = away from the camera (screen up)."""
    w = np.array([fi.wind[0], fi.wind[1], 0.0])
    r = fi.cam_right.copy()
    r[2] = 0.0
    f = fi.cam_fwd.copy()
    f[2] = 0.0
    r /= max(float(np.linalg.norm(r)), 1e-9)
    f /= max(float(np.linalg.norm(f)), 1e-9)
    return math.atan2(float(w @ f), float(w @ r))


# --------------------------------------------------------------------------- #
# drawing
# --------------------------------------------------------------------------- #
_FONTS: dict[tuple[int, bool], Any] = {}


def font(size: int, bold: bool = False) -> Any:
    from PIL import ImageFont
    k = (int(size), bold)
    if k not in _FONTS:
        _FONTS[k] = ImageFont.truetype(FONT_BOLD if bold else FONT_REG, int(size))
    return _FONTS[k]


def _rrect(d: Any, box: tuple[float, float, float, float], r: float, fill: tuple) -> None:
    d.rounded_rectangle([round(x) for x in box], radius=round(r), fill=fill)


def _shadowed_text(base: Any, xy: tuple[float, float], text: str, fnt: Any, fill: tuple, blur: float = 2.0,
                   anchor: str = "la", shadow_alpha: int = 160) -> None:
    from PIL import Image, ImageDraw, ImageFilter
    sh = Image.new("RGBA", base.size, (0, 0, 0, 0))
    ImageDraw.Draw(sh).text((xy[0] + 1, xy[1] + 2), text, font=fnt, fill=(0, 0, 0, shadow_alpha), anchor=anchor)
    base.alpha_composite(sh.filter(ImageFilter.GaussianBlur(blur)))
    ImageDraw.Draw(base).text(xy, text, font=fnt, fill=fill, anchor=anchor)


def grade(img: Any, strength: float = 0.16) -> Any:
    """The shared look applied to every rendered frame of both variants: a light radial vignette (corners
    -``strength`` x ~1.0 in linear-ish terms).  AgX Medium High Contrast is already in the render."""
    from PIL import Image
    a = np.asarray(img.convert("RGB"), dtype=np.float32)
    h, w = a.shape[:2]
    y = (np.arange(h, dtype=np.float32) - 0.5 * h) / (0.5 * h)
    x = (np.arange(w, dtype=np.float32) - 0.5 * w) / (0.5 * w)
    r2 = (x[None, :] ** 2) * 0.62 + (y[:, None] ** 2) * 0.38
    m = 1.0 - strength * np.clip(r2, 0.0, 1.6) ** 1.35
    return Image.fromarray(np.clip(a * m[:, :, None], 0, 255).astype(np.uint8))


def tag_clean(img: Any, text: str = TAG_CLEAN) -> Any:
    """The cinematic version: only the small honest tag, bottom right."""
    from PIL import Image
    W, H = img.size
    s = H / 1080.0
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    f = font(int(22 * s))
    from PIL import ImageDraw
    d = ImageDraw.Draw(ov)
    tw = d.textlength(text, font=f)
    # a faint backing pill keeps the tag legible over bright props / floor without shouting
    _rrect(d, (W - 40 * s - tw - 16 * s, H - 64 * s, W - 26 * s, H - 26 * s), 18 * s, (0, 0, 0, 80))
    _shadowed_text(ov, (W - 40 * s, H - 34 * s), text, f, (255, 255, 255, 225), anchor="rs")
    return Image.alpha_composite(img.convert("RGBA"), ov).convert("RGB")


def _sparkline(d: Any, box: tuple[float, float, float, float], tele: Telemetry, t_now: float, s: float) -> None:
    x0, y0, x1, y1 = box
    t0 = t_now - SPARK_S
    sel = (tele.t >= t0) & (tele.t <= t_now)
    if np.count_nonzero(sel) < 2:
        return
    tt = tele.t[sel]
    for arr, col, lo_min in ((tele.alt[sel], ALT_COLOR + (235,), 1.0), (tele.tilt[sel], ACCENT + (235,), 5.0)):
        lo, hi = 0.0, max(float(np.max(arr)), lo_min)
        xs = x0 + (tt - t0) / SPARK_S * (x1 - x0)
        ys = y1 - (np.clip(arr, lo, hi) - lo) / (hi - lo) * (y1 - y0)
        step = max(1, int(xs.size / 260))
        pts = list(zip(xs[::step].tolist(), ys[::step].tolist())) + [(float(xs[-1]), float(ys[-1]))]
        d.line(pts, fill=col, width=max(2, round(2.0 * s)), joint="curve")
        d.ellipse([xs[-1] - 3.5 * s, ys[-1] - 3.5 * s, xs[-1] + 3.5 * s, ys[-1] + 3.5 * s], fill=col)


def _arrow(d: Any, c: tuple[float, float], ang: float, r: float, col: tuple, w: int) -> None:
    dx, dy = math.cos(ang), -math.sin(ang)
    p0 = (c[0] - dx * r * 0.75, c[1] - dy * r * 0.75)
    p1 = (c[0] + dx * r * 0.75, c[1] + dy * r * 0.75)
    d.line([p0, p1], fill=col, width=w)
    nx, ny = -dy, dx
    d.polygon([(p1[0] + dx * r * 0.2, p1[1] + dy * r * 0.2), (p1[0] - dx * r * 0.3 + nx * r * 0.32, p1[1] - dy * r * 0.3 + ny * r * 0.32),
               (p1[0] - dx * r * 0.3 - nx * r * 0.32, p1[1] - dy * r * 0.3 - ny * r * 0.32)], fill=col)


def _info_pill(d: Any, x1: float, y0: float, text: str, f: Any, s: float) -> None:
    """Top-right honesty pill with a neutral 'i' glyph (no red dot: that reads as a REC light)."""
    tw = d.textlength(text, font=f)
    _rrect(d, (x1 - tw - 64 * s, y0, x1, y0 + 42 * s), 21 * s, (8, 12, 18, 175))
    cx, cy, r = x1 - tw - 40 * s, y0 + 21 * s, 9 * s
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(200, 208, 218, 255), width=max(1, round(1.6 * s)))
    d.text((cx, cy + 0.5 * s), "i", font=font(int(15 * s), bold=True), fill=(200, 208, 218, 255), anchor="mm")
    d.text((x1 - 20 * s, y0 + 21 * s), text, font=f, fill=(255, 255, 255, 255), anchor="rm")


HUD_ROW_FRAC = 0.078            # the lower row's panel height (fraction of the frame height); + margin <= 11 %
HUD_MARGIN_FRAC = 0.030


def hud_box_frac() -> float:
    """Fraction of the frame height covered by the lower row incl. its bottom margin (<= 0.11)."""
    return HUD_ROW_FRAC + HUD_MARGIN_FRAC


VANE_COLORS = ((42, 120, 214), (235, 104, 52), (27, 175, 122), (237, 161, 0))   # tools/sim_video.py vane slots
VANE_REF_RAD = math.radians(20.0)                                               # full-scale deflection (command limit)
_THETAS: list[float] | None = None


def vane_thetas() -> list[float]:
    """Vane azimuths (rad, body frame) from params.json via tools.sim_video.VehicleGeom (read-only), cached."""
    global _THETAS
    if _THETAS is None:
        try:
            from tools import sim_video as sv
            _THETAS = list(sv.VehicleGeom.from_params().vane_thetas_rad)
        except Exception:                                   # noqa: BLE001 - the indicator falls back to 0/90/180/270
            _THETAS = [0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi]
    return _THETAS


def vane_readout(delta: Sequence[float], thetas: Sequence[float]) -> dict[str, Any]:
    """Pure: what the vane indicator shows.  Body frame seen from BELOW (screen x = body x, screen y down = body y).
    Vane i deflected by delta_i pushes the vehicle along +e_theta_i = (-sin theta_i, cos theta_i) (SPEC 8.2,
    tools/sim_video.e_theta); 'net' = sum_i delta_i e_theta_i (rad, the pitch / yaw correction); 'trim' = the mean
    deflection (deg, the common-mode roll trim - the 'pinwheel')."""
    d = np.asarray(delta, float)[:len(thetas)]
    E = np.array([[-math.sin(th), math.cos(th)] for th in thetas])
    net = d @ E
    return {"net": net, "net_deg": float(math.degrees(float(np.linalg.norm(net)))),
            "trim_deg": float(math.degrees(float(np.mean(d)))), "delta_deg": [float(math.degrees(x)) for x in d],
            "screen_dirs": [(-math.sin(th), math.cos(th)) for th in thetas]}


def _vane_inset(d: Any, box: tuple[float, float, float, float], fi: FrameInfo, s: float) -> None:
    """Vane-angle indicator (art review round 2: replaces the unreadable 10 s sparkline): the duct exit seen from
    below with the four vanes in their tools/sim_video colours, each with an arrow of its side force (length ~
    deflection), the net correction arrow in the centre and the roll trim as text."""
    x0, y0, x1, y1 = box
    _rrect(d, box, 12 * s, (8, 12, 18, 175))
    d.text((x0 + 14 * s, y0 + 10 * s), "베인 각도 · 아래에서 본 모습", font=font(int(15 * s)), fill=(165, 175, 188, 255))
    if fi.delta is None:
        return
    th = vane_thetas()
    ro = vane_readout(fi.delta, th)
    cx, cy = 0.5 * (x0 + x1), y0 + 0.50 * (y1 - y0)
    R = 0.215 * (y1 - y0)
    d.ellipse([cx - R, cy - R, cx + R, cy + R], outline=(120, 130, 142, 255), width=max(1, round(2 * s)))
    d.ellipse([cx - 0.36 * R, cy - 0.36 * R, cx + 0.36 * R, cy + 0.36 * R], outline=(80, 88, 98, 255),
              width=max(1, round(1.5 * s)))
    for i, t_ in enumerate(th):
        col = VANE_COLORS[i % 4]
        ux, uy = math.cos(t_), math.sin(t_)               # body (cos, sin) -> screen (x, y down) from below
        d.line([(cx + 0.36 * R * ux, cy + 0.36 * R * uy), (cx + 1.18 * R * ux, cy + 1.18 * R * uy)],
               fill=col + (255,), width=max(2, round(5 * s)))
        dd = ro["delta_deg"][i]
        ex, ey = ro["screen_dirs"][i]
        L = 0.75 * R * float(np.clip(math.radians(dd) / VANE_REF_RAD, -1.0, 1.0))
        mx, my = cx + 0.80 * R * ux, cy + 0.80 * R * uy
        if abs(L) > 1.5 * s:
            _arrow(d, (mx + 0.5 * L * ex, my + 0.5 * L * ey), math.atan2(-(L * ey), L * ex), abs(L) / 1.5,
                   col + (255,), max(2, round(2.4 * s)))
        lx, ly = cx + 1.55 * R * ux, cy + 1.55 * R * uy
        d.text((lx, ly), f"{dd:+.0f}°", font=font(int(15 * s), bold=True), fill=col + (255,), anchor="mm")
    n = ro["net"]
    L = 0.9 * R * min(1.0, float(np.linalg.norm(n)) / VANE_REF_RAD)
    if L > 2.0 * s:
        _arrow(d, (cx + 0.5 * L * n[0] / np.linalg.norm(n), cy + 0.5 * L * n[1] / np.linalg.norm(n)),
               math.atan2(-float(n[1]), float(n[0])), L / 1.5, (255, 255, 255, 255), max(2, round(3 * s)))
    d.text((x0 + 14 * s, y1 - 12 * s), f"합력 {ro['net_deg']:.1f}°   롤 트림 {ro['trim_deg']:+.1f}°",
           font=font(int(15 * s)), fill=(225, 230, 236, 255), anchor="ls")


def overlay_pro(img: Any, fi: FrameInfo, tele: Telemetry, fps: float = 30.0) -> Any:
    from PIL import Image, ImageDraw
    W, H = img.size
    s0 = H / 1080.0
    # close-ups: a compact lower row (art review round 2: the row covered the lower fin)
    s = s0 * (0.8 if fi.kind == "closeup" else 1.0)
    base = img.convert("RGBA")
    ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    v = frame_values(fi)
    pc = ABORT_COLOR if fi.state == 5 else PHASE_COLOR.get(fi.phase, (200, 200, 200))
    # ---- one slim lower row ----
    cols = [("비행 단계", v["phase"], None, 150), ("고도", v["alt"], v["target"], 215), ("기울기", v["tilt"], None, 120),
            ("스로틀", v["thr"], None, 120), ("바람", v["wind"], "wind", 175)]
    if tele.has_push:
        cols.append(("밀기 힘", v["push"], "push", 120))
    pw = sum(c[3] for c in cols) * s + 36 * s
    ph = HUD_ROW_FRAC * H * s / s0
    x0, y1 = 48 * s0, H - HUD_MARGIN_FRAC * H
    y0 = y1 - ph
    _rrect(d, (x0, y0, x0 + pw, y1), 10 * s, (8, 12, 18, 165))
    _rrect(d, (x0, y0, x0 + 6 * s, y1), 3 * s, pc + (255,))
    x = x0 + 24 * s
    fv = font(int(30 * s), bold=True)
    for cap, val, sub, wpx in cols:
        d.text((x, y0 + 9 * s), cap, font=font(int(16 * s)), fill=(165, 175, 188, 255))
        colr = (255, 255, 255, 255)
        if sub == "push" and fi.push_active:
            colr = ACCENT + (255,)
        if cap == "비행 단계":
            colr = pc + (255,)
        d.text((x, y0 + 30 * s), val, font=fv, fill=colr)
        vw = d.textlength(val, font=fv)
        if sub == "wind" and v["wind"] != "무풍":
            cx = x + vw + 22 * s
            cy = y0 + 50 * s
            d.ellipse([cx - 14 * s, cy - 14 * s, cx + 14 * s, cy + 14 * s], outline=(170, 180, 192, 200), width=max(1, round(1.5 * s)))
            _arrow(d, (cx, cy), wind_screen_angle(fi), 14 * s, (120, 190, 255, 255), max(2, round(2.6 * s)))
        elif sub and sub not in ("wind", "push"):
            d.text((x + vw + 8 * s, y0 + 46 * s), sub, font=font(int(15 * s)), fill=(150, 160, 172, 255))
        x += wpx * s
    # ---- time code (right end of the row) ----
    d.text((W - 48 * s0, y1 - 8 * s0), v["time"], font=font(int(19 * s0)), fill=(225, 230, 236, 220), anchor="rs")
    # ---- vane-angle indicator (bottom right, above the time code) ----
    if fi.delta is not None:
        bw = 250 * s0
        _vane_inset(d, (W - 48 * s0 - bw, y0 - 14 * s0 - bw, W - 48 * s0, y0 - 14 * s0), fi, s0)
    base = Image.alpha_composite(base, ov)
    s = s0
    # ---- top right: honest tag + slow motion ----
    ov2 = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d2 = ImageDraw.Draw(ov2)
    bx1, by0 = W - 48 * s, 42 * s
    _info_pill(d2, bx1, by0, tele.prov.tag_pro, font(int(21 * s), bold=True), s)
    if v["slow"]:
        sf = font(int(22 * s), bold=True)
        sw2 = d2.textlength(v["slow"], font=sf)
        _rrect(d2, (bx1 - sw2 - 40 * s, by0 + 54 * s, bx1, by0 + 94 * s), 20 * s, (255, 150, 60, 215))
        d2.text((bx1 - 20 * s, by0 + 74 * s), v["slow"], font=sf, fill=(20, 14, 8, 255), anchor="rm")
    base = Image.alpha_composite(base, ov2)
    # ---- top left: title line + event call-out ----
    _shadowed_text(base, (56 * s, 44 * s), tele.subtitle, font(int(20 * s)), (225, 230, 236, 235))
    live = [(event_alpha(e, fi.t), e) for e in tele.events]
    live = [(a, e) for a, e in live if a > 0.004]
    if live:
        a, e = live[-1]
        ef = font(int(30 * s), bold=True)
        ov3 = Image.new("RGBA", base.size, (0, 0, 0, 0))
        d3 = ImageDraw.Draw(ov3)
        ew = d3.textlength(e[2], font=ef)
        col = {"gust": (90, 168, 255), "push": ACCENT, "abort": ABORT_COLOR, "touchdown": (64, 220, 150)}.get(e[1], (235, 240, 245))
        _rrect(d3, (56 * s, 82 * s, 56 * s + ew + 52 * s, 132 * s), 10 * s, (8, 12, 18, int(170 * a)))
        _rrect(d3, (56 * s, 82 * s, 62 * s, 132 * s), 3 * s, col + (int(255 * a),))
        d3.text((82 * s, 107 * s), e[2], font=ef, fill=(255, 255, 255, int(255 * a)), anchor="lm")
        base = Image.alpha_composite(base, ov3)
    return base.convert("RGB")


def fit_lines(d: Any, text: str, max_w: float, size: int, min_size: int, bold: bool = False) -> tuple[list[str], Any]:
    """Shrink (to min_size) then wrap at spaces into at most 2 lines so ``text`` fits ``max_w`` pixels."""
    sz = size
    while sz > min_size and d.textlength(text, font=font(sz, bold)) > max_w:
        sz -= 2
    f = font(sz, bold)
    if d.textlength(text, font=f) <= max_w:
        return [text], f
    words = text.split(" ")
    best = [text]
    for k in range(1, len(words)):
        a, b = " ".join(words[:k]), " ".join(words[k:])
        if d.textlength(a, font=f) <= max_w and d.textlength(b, font=f) <= max_w:
            best = [a, b]
            if d.textlength(a, font=f) >= d.textlength(b, font=f):
                break
    return best, f


def title_card(bg: Any, tele: Telemetry, a: float = 1.0) -> Any:
    """A dark title card (shown BEFORE the footage, never over the hero moment): ``bg`` (the first rendered frame)
    blurred and darkened under the title, the subtitle, the provenance line and the honesty pill."""
    from PIL import Image, ImageDraw, ImageFilter
    W, H = bg.size
    s = H / 1080.0
    blur = bg.convert("RGB").filter(ImageFilter.GaussianBlur(22 * s))
    base = Image.blend(blur, Image.new("RGB", bg.size, (5, 7, 11)), 0.80).convert("RGBA")
    ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    al = int(255 * a)
    cx = W / 2
    lines, tf = fit_lines(d, tele.title, 0.80 * W, int(58 * s), int(34 * s), bold=True)
    for i, ln in enumerate(lines):
        d.text((cx, H * 0.45 - 18 * s - (len(lines) - 1 - i) * tf.size * 1.25), ln, font=tf, fill=(255, 255, 255, al), anchor="ms")
    d.line([(cx - 120 * s, H * 0.45 + 6 * s), (cx + 120 * s, H * 0.45 + 6 * s)], fill=ACCENT + (al,), width=max(2, round(3 * s)))
    d.text((cx, H * 0.45 + 54 * s), tele.subtitle, font=font(int(27 * s)), fill=(225, 230, 236, int(240 * a)), anchor="ms")
    d.text((cx, H * 0.45 + 96 * s), tele.prov.card_note, font=font(int(21 * s)), fill=(190, 198, 208, int(230 * a)), anchor="ms")
    base = Image.alpha_composite(base, ov)
    ov2 = Image.new("RGBA", base.size, (0, 0, 0, 0))
    _info_pill(ImageDraw.Draw(ov2), W - 48 * s, 42 * s, tele.prov.tag_pro, font(int(21 * s), bold=True), s)
    if a < 1.0:
        ov2.putalpha(Image.eval(ov2.getchannel("A"), lambda x: int(x * a)))
    return Image.alpha_composite(base, ov2).convert("RGB")


def title_frames(bg: Any, tele: Telemetry, seconds: float, fps: float = 30.0) -> list[Any]:
    """The title card sequence: fade in 0.5 s from black, hold, fade out 0.4 s to black (the footage cuts in after)."""
    from PIL import Image
    n = int(round(seconds * fps))
    card = title_card(bg, tele, 1.0)
    black = Image.new("RGB", card.size, (0, 0, 0))
    out = []
    for i in range(n):
        tt = i / fps
        a = min(1.0, tt / 0.5, (seconds - tt) / 0.4)
        out.append(Image.blend(black, card, max(0.0, a)))
    return out


def _tilt_chart(d: Any, box: tuple[float, float, float, float], tele: Telemetry, s: float, al: int,
                reveal: float = 1.0) -> None:
    """Whole-flight tilt (deg) vs time with push / gust markers: the attitude recovery at a glance; ``reveal`` (0..1)
    draws the curve in from the left (the end card animates it instead of a static 5 s card)."""
    x0, y0, x1, y1 = box
    _rrect(d, (x0, y0, x1, y1), 14 * s, (14, 19, 27, int(0.85 * al)))
    d.text((x0 + 24 * s, y0 + 18 * s), "기울기 (°) · 비행 전체", font=font(int(21 * s), bold=True), fill=(225, 230, 236, al))
    px0, py0, px1, py1 = x0 + 60 * s, y0 + 66 * s, x1 - 24 * s, y1 - 46 * s
    t = tele.t
    if t.size < 2:
        return
    hi = max(5.0, float(np.ceil(np.nanmax(tele.tilt) / 5.0) * 5.0))
    for k in range(0, int(hi) + 1, max(1, int(hi // 4 // 5 * 5) or 5)):
        y = py1 - k / hi * (py1 - py0)
        d.line([(px0, y), (px1, y)], fill=(60, 70, 84, al), width=1)
        d.text((px0 - 10 * s, y), f"{k}", font=font(int(15 * s)), fill=(150, 160, 172, al), anchor="rm")
    for tm, kind in tele.marks:
        x = px0 + (tm - t[0]) / (t[-1] - t[0]) * (px1 - px0)
        col = (ACCENT if kind == "push" else (90, 168, 255)) + (int(0.8 * al),)
        d.line([(x, py0), (x, py1)], fill=col, width=max(1, round(1.5 * s)))
    step = max(1, int(t.size / 700))
    xs = px0 + (t[::step] - t[0]) / (t[-1] - t[0]) * (px1 - px0)
    ys = py1 - np.clip(tele.tilt[::step], 0, hi) / hi * (py1 - py0)
    m = max(2, int(math.ceil(float(np.clip(reveal, 0.0, 1.0)) * xs.size)))
    d.line(list(zip(xs[:m].tolist(), ys[:m].tolist())), fill=(255, 255, 255, al), width=max(2, round(2 * s)), joint="curve")
    if reveal < 1.0:
        d.ellipse([xs[m - 1] - 4 * s, ys[m - 1] - 4 * s, xs[m - 1] + 4 * s, ys[m - 1] + 4 * s], fill=ACCENT + (al,))
    d.text((px0, py1 + 12 * s), f"0 s", font=font(int(15 * s)), fill=(150, 160, 172, al))
    d.text((px1, py1 + 12 * s), f"{t[-1] - t[0]:.0f} s", font=font(int(15 * s)), fill=(150, 160, 172, al), anchor="ra")
    lx = px0 + 90 * s
    if any(k == "push" for _, k in tele.marks):
        d.line([(lx, py1 + 22 * s), (lx + 22 * s, py1 + 22 * s)], fill=ACCENT + (al,), width=3)
        d.text((lx + 30 * s, py1 + 12 * s), "밀기", font=font(int(15 * s)), fill=(190, 198, 208, al))
        lx += 110 * s
    if any(k == "gust" for _, k in tele.marks):
        d.line([(lx, py1 + 22 * s), (lx + 22 * s, py1 + 22 * s)], fill=(90, 168, 255, al), width=3)
        d.text((lx + 30 * s, py1 + 12 * s), "돌풍", font=font(int(15 * s)), fill=(190, 198, 208, al))


def end_card(last: Any, tele: Telemetry, a: float = 1.0, tag: str | None = None, reveal: float = 1.0) -> Any:
    """Summary card over the darkened, blurred last frame: key numbers (left), the whole-flight tilt chart (right),
    credits, the honesty pill."""
    from PIL import Image, ImageDraw, ImageFilter
    W, H = last.size
    s = H / 1080.0
    tag = tag or tele.prov.tag_pro
    bg = last.convert("RGB").filter(ImageFilter.GaussianBlur(18 * s))
    dark = Image.new("RGB", bg.size, (6, 9, 14))
    base = Image.blend(bg, dark, 0.35 + 0.45 * a).convert("RGBA")
    ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    al = int(255 * a)
    x0 = 150 * s
    d.text((x0, 150 * s), "결과 요약", font=font(int(54 * s), bold=True), fill=(255, 255, 255, al))
    d.line([(x0, 228 * s), (x0 + 140 * s, 228 * s)], fill=ACCENT + (al,), width=max(2, round(4 * s)))
    lines, tf = fit_lines(d, tele.title, 0.80 * W, int(25 * s), int(19 * s))
    for i, ln in enumerate(lines):
        d.text((x0, 250 * s + i * 32 * s), ln, font=tf, fill=(205, 212, 220, al))
    y = 340 * s
    for k, (lab, val) in enumerate(tele.summary[:6]):
        cx = x0 + (k % 2) * 400 * s
        cy = y + (k // 2) * 128 * s
        d.text((cx, cy), lab, font=font(int(21 * s)), fill=(165, 175, 188, al))
        d.text((cx, cy + 30 * s), val, font=font(int(44 * s), bold=True), fill=(255, 255, 255, al))
    _tilt_chart(d, (x0 + 830 * s, 330 * s, W - 150 * s, 720 * s), tele, s, al, reveal)
    yc = H - 220 * s
    for i, c in enumerate(tele.credits):
        d.text((x0, yc + i * 34 * s), c, font=font(int(21 * s)), fill=(175, 184, 196, al))
    base = Image.alpha_composite(base, ov)
    ov2 = Image.new("RGBA", base.size, (0, 0, 0, 0))
    _info_pill(ImageDraw.Draw(ov2), W - 150 * s, 150 * s, tag, font(int(21 * s), bold=True), s)
    if a < 1.0:
        ov2.putalpha(Image.eval(ov2.getchannel("A"), lambda x: int(x * a)))
    return Image.alpha_composite(base, ov2).convert("RGB")


# --------------------------------------------------------------------------- #
# encoding
# --------------------------------------------------------------------------- #
def ffmpeg_exe() -> str:
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


def encode(frame_dir: str, pattern: str, out_mp4: str, fps: float = 30.0, start: int = 1, crf: int = 18,
           pre_dir: str = "", audio: str = "") -> list[str]:
    """frame_dir/pattern (printf, e.g. 'frame_%05d.png') -> H.264 High, CRF 18, yuv420p, +faststart; ``pre_dir``
    (same pattern, from 1) is played first (the title card); ``audio`` (a WAV) is muxed as AAC 192 kb/s."""
    cmd = [ffmpeg_exe(), "-y", "-loglevel", "error"]
    if pre_dir:
        cmd += ["-framerate", f"{fps:g}", "-start_number", "1", "-i", os.path.join(pre_dir, pattern)]
    cmd += ["-framerate", f"{fps:g}", "-start_number", str(start), "-i", os.path.join(frame_dir, pattern)]
    if audio:
        cmd += ["-i", audio]
    if pre_dir:
        cmd += ["-filter_complex", "[0:v][1:v]concat=n=2:v=1:a=0[v]", "-map", "[v]"]
        if audio:
            cmd += ["-map", "2:a"]
    elif audio:
        cmd += ["-map", "0:v", "-map", "1:a"]
    cmd += ["-c:v", "libx264", "-profile:v", "high", "-preset", "medium", "-crf", str(crf), "-pix_fmt", "yuv420p"]
    if audio:
        cmd += ["-c:a", "aac", "-b:a", "192k", "-shortest"]
    cmd += ["-movflags", "+faststart", out_mp4]
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        raise RuntimeError(f"ffmpeg failed: {r.stderr[-800:]}")
    return cmd
