"""Blender flight-video pipeline driver (run with the project venv).

  python tools/blender_render.py --log X.npz [--scene outdoor|indoor|auto] [--shots auto|turntable|FILE.json|JSON]
         [--hud pro|clean|both|none] [--engine eevee|cycles|auto] [--quality draft|final] [--out DIR] [--label NAME]
         [--frames a:b] [--still t1,t2,...] [--res WxH] [--samples N] [--threads N] [--person auto|on|off]
         [--end-card S] [--dry-run]

Pipeline: the log (tools.sim_video.load_log contract, via render.logio) -> flight events -> shot list (automatic,
the turntable reveal, or a JSON list) -> frame plan (render.plan: per video frame vehicle pose, vanes, jet, wind,
person, camera; slow motion = a finer sim-time grid) -> ``blender.exe --background --factory-startup --python
render/blender/main.py`` renders the raw frames (resumable: existing PNGs are skipped) -> render.hud composites the
analysis ('pro') and/or cinematic ('clean') overlays -> imageio-ffmpeg encodes H.264 High CRF 18 yuv420p +faststart
(when every frame of the edit exists).  A JSON sidecar records the log sha256, the settings, the shot list, the
warnings and the measured seconds per frame.

``--engine auto`` picks per shot kind (ENGINE_AUTO).  Measured 2026-10-21 on this PC (Intel UHD 750, headless Blender
5.2.2): EEVEE renders only the FIRST frame of a session (later ones come out black; Vulkan is refused for this
driver), so every shot kind maps to Cycles CPU, and every rendered frame is checked for being non-empty
(``check_frames``: an empty frame is deleted, reported and re-rendered by the next run).
Everything except the Blender render is a pure function of the log and the options (tests: tools/test_blender_render.py).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from typing import Any

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from render import hud as HUD  # noqa: E402
from render import plan as PL  # noqa: E402
from render import shots as SH  # noqa: E402
from render.logio import load_log  # noqa: E402

BLENDER = os.environ.get("BLENDER_EXE") or \
    r"C:\Users\yc012\AppData\Local\Programs\Blender\blender-5.2.2-windows-x64\blender.exe"
MAIN = os.path.join(ROOT, "render", "blender", "main.py")
DEV_OUT = os.path.join(ROOT, "sim", "out", "video", "blender_dev")
FPS = 30.0
# per shot kind: Cycles everywhere (EEVEE is unreliable headless on the UHD 750, see the module docstring)
ENGINE_AUTO = {"wide": "cycles", "spectator": "cycles", "orbit": "cycles", "crane": "cycles", "closeup": "cycles",
               "turntable": "cycles"}


def resolve_scene(scene: str, log: Any) -> str:
    """'indoor' for a demo log (meta.profile == 'demo', or 'demo' / 'indoor' in the scenario / title, or pushes at a
    low hover), else 'outdoor'; ``scene`` forces one."""
    if scene in ("outdoor", "indoor"):
        return scene
    if scene not in ("auto", ""):
        raise ValueError("scene must be outdoor | indoor | auto")
    if str(log.meta.get("profile", "")).lower() == "demo":
        return "indoor"
    words = (str(log.meta.get("scenario", "")) + " " + str(log.meta.get("title", ""))).lower().replace("_", " ").split()
    if "demo" in words or "indoor" in words:
        return "indoor"
    return "indoor" if log.profile == "demo" else "outdoor"


def load_shots(spec: str, log: Any, indoor: bool) -> tuple[list[SH.Shot], list[SH.Event], dict[str, str]]:
    """(shots, events, display texts): a shot-list JSON in its dict form may carry 'title_ko' / 'subtitle_ko'."""
    ev = SH.detect_events(log)
    if spec in ("", "auto"):
        return SH.auto_shots(ev, float(log.t[0]), float(log.t[-1]), indoor=indoor), ev, {}
    txt = spec
    if os.path.exists(spec):
        with open(spec, "r", encoding="utf-8") as f:
            txt = f.read()
    obj = json.loads(txt)
    return (SH.normalise(SH.shots_from_json(obj), float(log.t[0]), float(log.t[-1])), ev, SH.shot_list_display(obj))


def vane_thetas() -> list[float]:
    """Vane azimuths (rad) from params.json via tools.sim_video.VehicleGeom (read-only)."""
    from tools import sim_video as sv
    return list(sv.VehicleGeom.from_params().vane_thetas_rad)


def parse_still_times(spec: str, log: Any) -> list[float]:
    """'--still' tokens: sim times, or 'peak:A-B' = where the vanes work hardest in [A, B] (render.shots.vane_peak_time)."""
    out = []
    for tok in [x.strip() for x in spec.split(",") if x.strip()]:
        if tok.startswith("peak:"):
            a, b = (float(x) for x in tok[5:].split("-"))
            out.append(SH.vane_peak_time(log, a, b, vane_thetas()))
        else:
            out.append(float(tok))
    return out


def still_frames(plan_t: np.ndarray, seg: np.ndarray, times: list[float]) -> list[int]:
    """Blender frame (1-based) nearest to each requested sim time inside the edit; ValueError when a time is not in
    any shot."""
    out = []
    for tq in times:
        k = int(np.argmin(np.abs(plan_t - tq)))
        if abs(float(plan_t[k]) - tq) > 0.2:
            raise ValueError(f"sim time {tq} s is not inside any shot of the edit")
        out.append(k + 1)
    return out


def engine_for_frames(plan: dict[str, Any], engine: str) -> dict[str, list[int]]:
    n = int(plan["meta"]["frames"])
    if engine in ("eevee", "cycles"):
        return {engine: list(range(1, n + 1))}
    kinds = plan["meta"]["cam_kinds"]
    out: dict[str, list[int]] = {}
    for k in range(n):
        e = ENGINE_AUTO.get(kinds[int(plan["seg"][k])], "cycles")
        out.setdefault(e, []).append(k + 1)
    return out


def check_frames(raw_dir: str, frames: list[int], min_mean: float = 2.0) -> list[int]:
    """Delete and return the frames whose PNG is (nearly) black - the EEVEE failure mode on this PC."""
    from PIL import Image
    bad = []
    for fr in frames:
        p = os.path.join(raw_dir, f"frame_{fr:05d}.png")
        if not os.path.exists(p):
            continue
        with Image.open(p) as im:
            m = float(np.asarray(im.convert("L").resize((64, 36))).mean())
        if m < min_mean:
            os.remove(p)
            bad.append(fr)
    return bad


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_blender(plan_path: str, raw_dir: str, engine: str, quality: str, frames: list[int], res: str, samples: int,
                threads: int, timing: str, log_path: str, extra: list[str] | None = None,
                low_priority: bool = True) -> dict[str, Any]:
    ff = os.path.join(os.path.dirname(raw_dir), f"_frames_{engine}.json")
    with open(ff, "w", encoding="utf-8") as f:
        json.dump(frames, f)
    cmd = [BLENDER, "--background", "--factory-startup", "--python", MAIN, "--", "--plan", plan_path, "--out", raw_dir,
           "--engine", engine.upper(), "--quality", quality, "--frames-file", ff, "--res", res, "--timing", timing]
    if samples:
        cmd += ["--samples", str(samples)]
    if threads:
        cmd += ["--threads", str(threads)]
    cmd += extra or []
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    t0 = time.time()
    with open(log_path, "a", encoding="utf-8", errors="replace") as lf:
        lf.write(f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} {' '.join(cmd)}\n")
        lf.flush()
        # below-normal priority on Windows: other jobs on this PC (simulation campaigns, firmware builds) keep the CPU
        flags = getattr(subprocess, "BELOW_NORMAL_PRIORITY_CLASS", 0) if (low_priority and os.name == "nt") else 0
        r = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT, env=env, creationflags=flags)
    info: dict[str, Any] = {"engine": engine, "returncode": r.returncode, "wall_s": round(time.time() - t0, 1)}
    with open(log_path, "r", encoding="utf-8", errors="replace") as lf:
        lines = lf.read().splitlines()
    for ln in reversed(lines):
        if ln.startswith("BUILD "):
            try:
                info["build"] = json.loads(ln[6:])
            except ValueError:
                pass
            break
    if r.returncode != 0:
        tail = "\n".join(lines[-25:])
        raise RuntimeError(f"Blender failed ({r.returncode}); log {log_path}:\n{tail}")
    return info


def composite(P: dict[str, np.ndarray], raw_dir: str, out_dirs: dict[str, str], frames: list[int], tele: HUD.Telemetry | None,
              shots_meta: list[dict] | None = None,
              fps: float = FPS) -> int:
    """Raw frames -> the shared grade (vignette) -> 'pro' overlays / 'clean' tag (the provenance-aware strings)."""
    from PIL import Image
    fis = HUD.frames_from_plan(P, shots_meta)
    clean_tag = tele.prov.tag_clean if tele is not None else HUD.TAG_RENDER     # no log: the studio turntable
    n = 0
    for fr in frames:
        src = os.path.join(raw_dir, f"frame_{fr:05d}.png")
        if not os.path.exists(src):
            continue
        img = None
        for var, d in out_dirs.items():
            dst = os.path.join(d, f"frame_{fr:05d}.png")
            if os.path.exists(dst) and os.path.getmtime(dst) >= os.path.getmtime(src):
                continue
            if img is None:
                img = HUD.grade(Image.open(src).convert("RGB"))
            if var == "pro" and tele is not None:
                o = HUD.overlay_pro(img, fis[fr - 1], tele, fps)
            else:
                o = HUD.tag_clean(img, clean_tag)
            o.save(dst, compress_level=3)
            n += 1
    return n


def append_end_card(dir_: str, n_frames: int, tele: HUD.Telemetry, seconds: float, fps: float = FPS,
                    raw_last: str = "") -> int:
    """End card after the last footage frame, built over the graded raw last frame (not the HUD frame)."""
    from PIL import Image
    src = raw_last if raw_last and os.path.exists(raw_last) else os.path.join(dir_, f"frame_{n_frames:05d}.png")
    last = HUD.grade(Image.open(src).convert("RGB"))
    m = int(round(seconds * fps))
    for i in range(m):
        a = min(1.0, (i + 1) / (0.6 * fps))
        rv = float(np.clip((i + 1 - 0.4 * fps) / (1.4 * fps), 0.0, 1.0))      # the tilt curve draws in over 1.4 s
        HUD.end_card(last, tele, a, reveal=rv).save(os.path.join(dir_, f"frame_{n_frames + 1 + i:05d}.png"),
                                                    compress_level=3)
    return m


def write_title(dir_: str, raw_first: str, tele: HUD.Telemetry, seconds: float, fps: float = FPS) -> int:
    """The dark title card BEFORE the footage (frames 1..n of ``dir_``)."""
    from PIL import Image
    os.makedirs(dir_, exist_ok=True)
    frames = HUD.title_frames(HUD.grade(Image.open(raw_first).convert("RGB")), tele, seconds, fps)
    for i, im in enumerate(frames):
        im.save(os.path.join(dir_, f"frame_{i + 1:05d}.png"), compress_level=3)
    return len(frames)


def qa_frames(raw_dir: str, plan: dict[str, Any], frames: list[int], every: int = 10) -> dict[str, Any]:
    """Frame QA (render.qa) on the rendered frames (every ``every``-th of a clip, all stills): the matte-black shell's
    median / chroma / highlight coverage, and indoors the vehicle's shadow from the stage key spot."""
    from render import qa as QA
    P = {k: v for k, v in plan.items() if k != "meta"}
    meta = plan.get("meta", {})
    out: dict[str, Any] = {"limit_srgb": QA.SHELL_MAX_SRGB, "limit_chroma": QA.SHELL_MAX_CHROMA,
                           "limit_hi_frac": QA.SHELL_MAX_HI_FRAC, "frames": {}, "shadow": {}, "warnings": []}
    sel = frames if len(frames) <= 12 else frames[::max(1, every)]
    kl = (meta.get("key_light") or {}).get("pos")
    for fr in sel:
        p = os.path.join(raw_dir, f"frame_{fr:05d}.png")
        if not os.path.exists(p):
            continue
        r = QA.shell_luminance(p, ROOT, P, fr)
        out["frames"][str(fr)] = r
        out["warnings"] += QA.gate_warnings(fr, r)
        if kl is not None:
            sh = QA.shadow_contrast(p, P, fr, kl, float(meta.get("floor_z", float(meta.get("ground_z", 0.0)) - 0.018)))
            if sh:
                out["shadow"][str(fr)] = sh
                if sh["darken"] < QA.SHADOW_MIN_DARKEN:
                    out["warnings"].append(f"frame {fr}: vehicle shadow only {100 * sh['darken']:.0f} % darker than the "
                                           f"floor (< {100 * QA.SHADOW_MIN_DARKEN:.0f} %)")
    meds = [v["median"] for v in out["frames"].values() if np.isfinite(v["median"])]
    if meds:
        out["median_of_medians"] = round(float(np.median(meds)), 1)
    return out


def pose_gate(plan: dict[str, Any]) -> dict[str, Any] | None:
    """render.pose.pose_qa over every frame of the edit (soles planted while pushing, pole / fists clear of the head,
    grips reached); None without a person."""
    if not plan["meta"].get("person") or "person_feet" not in plan:
        return None
    from render import pose as PO
    P = {k: v for k, v in plan.items() if k != "meta"}
    floor_z = float(plan["meta"].get("floor_z", float(plan["meta"]["ground_z"]) - 0.018))
    r = PO.pose_qa(P, floor_z)
    r["fail"] = r["fail"][:20]
    return r


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--log", default="")
    ap.add_argument("--scene", default="auto")
    ap.add_argument("--shots", default="auto")
    ap.add_argument("--hud", default="both", choices=("pro", "clean", "both", "none"))
    ap.add_argument("--engine", default="auto", choices=("eevee", "cycles", "auto"))
    ap.add_argument("--quality", default="final", choices=("draft", "final"))
    ap.add_argument("--out", default=DEV_OUT)
    ap.add_argument("--label", default="")
    ap.add_argument("--frames", default="")
    ap.add_argument("--still", default="")
    ap.add_argument("--res", default="1920x1080")
    ap.add_argument("--samples", type=int, default=0)
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--person", default="auto", choices=("auto", "on", "off"))
    ap.add_argument("--end-card", type=float, default=3.0)
    ap.add_argument("--title-s", type=float, default=3.0, help="dark title card before the footage (pro version)")
    ap.add_argument("--title", default="")
    ap.add_argument("--subtitle", default="")
    ap.add_argument("--hdri", default="alps_field")
    ap.add_argument("--no-grass", action="store_true")
    ap.add_argument("--no-dust-volume", action="store_true", help="skip the volumetric dust skirt (outdoor)")
    ap.add_argument("--tt-seconds", type=float, default=8.0, help="turntable reveal length (s)")
    ap.add_argument("--audio", default="on", choices=("on", "off"),
                    help="mux the synthesised EDF sound bed (render.audio; an illustration, credited on the end card)")
    ap.add_argument("--normal-priority", action="store_true", help="run Blender at normal priority (default: below normal)")
    ap.add_argument("--dry-run", action="store_true", help="plan + shot list only, no Blender")
    args = ap.parse_args(argv)
    args.out = os.path.abspath(args.out)          # Blender resolves relative output paths against its own cwd

    t_start = time.time()
    log = None
    events: list[SH.Event] = []
    display: dict[str, str] = {}
    if args.shots == "turntable":
        plan = PL.turntable_plan(FPS, args.tt_seconds)
        label = args.label or "turntable_reveal"
        shots = [SH.Shot(0.0, args.tt_seconds, "turntable", note="reveal")]
        scene = "studio"
    else:
        if not args.log:
            ap.error("--log is required unless --shots turntable")
        log = load_log(args.log)
        scene = resolve_scene(args.scene, log)
        indoor = scene == "indoor"
        shots, events, display = load_shots(args.shots, log, indoor)
        if not shots:
            raise SystemExit("empty shot list")
        cfg = PL.SceneCfg(indoor=indoor, sun_az_deg=135.0)
        plan = PL.build_plan(log, shots, FPS, cfg, person=args.person)
        label = args.label or (os.path.splitext(os.path.basename(args.log))[0] + f"_{scene}")
    work = os.path.join(args.out, label)
    raw = os.path.join(work, "raw")
    os.makedirs(raw, exist_ok=True)
    plan_path = os.path.join(work, "plan.npz")
    PL.save_plan(plan, plan_path)
    with open(os.path.join(work, "shots.json"), "w", encoding="utf-8") as f:
        json.dump([s.to_json() for s in shots], f, indent=1)
    n = int(plan["meta"]["frames"])
    desc = SH.describe(shots, FPS) if args.shots != "turntable" else [f"turntable {n} frames"]
    print("\n".join(desc))
    print(f"{n} frames = {n / FPS:.1f} s of video; scene {scene}; person {plan['meta'].get('person')}")
    pose = pose_gate(plan)
    if pose is not None:
        print(f"pose QA: {'ok' if pose['ok'] else 'FAIL'} over {pose['frames']} frames, worst {pose['worst']}")
        for f_ in pose["fail"][:5]:
            print("POSE WARNING:", f_)
    # frame selection
    still_t: list[float] = []
    if args.still:
        still_t = parse_still_times(args.still, log) if log is not None else [float(x) for x in args.still.split(",")]
        sel = still_frames(plan["t"], plan["seg"], still_t)
    elif args.frames:
        a, b = args.frames.split(":")
        sel = list(range(max(1, int(a)), min(n, int(b)) + 1))
    else:
        sel = list(range(1, n + 1))
    if args.dry_run:
        print("frames:", sel[:10], "..." if len(sel) > 10 else "")
        return 0
    groups = engine_for_frames(plan, args.engine)
    timing = os.path.join(work, "timing.json")
    blog = os.path.join(work, "blender.log")
    runs = []
    extra = ["--hdri", args.hdri] + (["--no-grass"] if args.no_grass else []) + (["--no-dust-volume"] if args.no_dust_volume else [])
    for eng, frs in groups.items():
        want = [f for f in frs if f in set(sel)]
        if not want:
            continue
        runs.append(run_blender(plan_path, raw, eng, args.quality, want, args.res, args.samples, args.threads, timing, blog,
                                extra, low_priority=not args.normal_priority))
        bad = check_frames(raw, want)
        if bad:
            runs[-1]["empty_frames_removed"] = bad
            print(f"WARNING: {len(bad)} empty ({eng}) frames removed; re-run to render them again: {bad[:10]}")
    # overlays
    tele = None
    if log is not None:
        tele = HUD.telemetry_from_log(log, events, title=args.title or display.get("title_ko", ""),
                                      subtitle=args.subtitle or display.get("subtitle_ko", ""),
                                      person=bool(plan["meta"].get("person")), audio=args.audio == "on",
                                      edit_t=plan["t"])
    variants = {"pro": ["pro"], "clean": ["clean"], "both": ["pro", "clean"], "none": []}[args.hud]
    if args.shots == "turntable" and "pro" in variants:
        variants = ["clean"] if "clean" in variants or args.hud == "both" else []
    out_dirs = {v: os.path.join(work, f"hud_{v}") for v in variants}
    for d in out_dirs.values():
        os.makedirs(d, exist_ok=True)
    P = {k: v for k, v in plan.items() if k != "meta"}
    nc = composite(P, raw, out_dirs, sel, tele, plan["meta"].get("shots"))
    qa = qa_frames(raw, plan, sel)
    for w in qa["warnings"]:
        print("QA WARNING:", w)
    # encode when the whole edit exists
    videos = {}
    audio_info = None
    complete = all(os.path.exists(os.path.join(raw, f"frame_{k:05d}.png")) for k in range(1, n + 1))
    if complete and not args.still:
        for v, d in out_dirs.items():
            nn = n
            pre = ""
            n_end = n_title = 0
            if v == "pro" and tele is not None and args.end_card > 0:
                n_end = append_end_card(d, n, tele, args.end_card, raw_last=os.path.join(raw, f"frame_{n:05d}.png"))
                nn = n + n_end
            if v == "pro" and tele is not None and args.title_s > 0:
                pre = os.path.join(work, "title_pro")
                n_title = write_title(pre, os.path.join(raw, "frame_00001.png"), tele, args.title_s)
                nn += n_title
            wav = ""
            if args.audio == "on" and tele is not None:
                from render import audio as AU
                wav = os.path.join(work, f"sound_{v}.wav")
                audio_info = AU.track_for(P, float(plan["meta"].get("t_ref_n", 1.0)), wav, FPS, pre_s=n_title / FPS,
                                          post_s=n_end / FPS)
            mp4 = os.path.join(args.out, f"{label}_{v}.mp4")
            HUD.encode(d, "frame_%05d.png", mp4, FPS, pre_dir=pre, audio=wav)
            videos[v] = {"path": mp4, "frames": nn, "bytes": os.path.getsize(mp4), "audio": bool(wav)}
    # sidecar
    tm = {}
    if os.path.exists(timing):
        with open(timing, "r", encoding="utf-8") as f:
            tm = json.load(f)
    per_engine: dict[str, list[float]] = {}
    for eng, frs in groups.items():
        vals = [tm[str(k)] for k in frs if str(k) in tm]
        if vals:
            per_engine[eng] = vals
    side = {
        "label": label, "log": os.path.abspath(args.log) if args.log else None, "log_sha256": log.sha256 if log else None,
        "log_loader": log.loader if log else None, "log_warnings": log.warnings if log else [],
        "scene": scene, "settings": {k: getattr(args, k) for k in ("engine", "quality", "res", "samples", "threads", "hud",
                                                                    "person", "hdri", "shots")},
        "blender": BLENDER, "host": platform.node(), "frames": n, "video_s": round(n / FPS, 2), "fps": FPS,
        "shots": [s.to_json() for s in shots], "shot_table": desc, "selected_frames": len(sel),
        "s_per_frame": {e: {"mean": round(float(np.mean(v)), 2), "min": round(float(np.min(v)), 2),
                            "max": round(float(np.max(v)), 2), "n": len(v)} for e, v in per_engine.items()},
        "runs": runs, "composited": nc, "videos": videos, "plan_meta": plan["meta"],
        "qa_shell_luminance": qa, "qa_pose": pose, "provenance": (tele.prov.kind if tele is not None else "none"),
        "audio": audio_info, "still_times": still_t, "still_frames": sel if args.still else [],
        "wall_s": round(time.time() - t_start, 1),
        "honesty": "CG render of a logged motion (see 'provenance': sim6dof = 6-DOF simulation, synthetic = kinematic "
                   "test pattern). Every on-screen number is a logged value; effects listed in plan_meta.illustrations "
                   "are illustrations driven by logged values, not simulated physics.",
    }
    with open(os.path.join(args.out, f"{label}.json"), "w", encoding="utf-8") as f:
        json.dump(side, f, indent=1, ensure_ascii=False, default=str)
    print(json.dumps({"label": label, "s_per_frame": side["s_per_frame"], "videos": videos}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
