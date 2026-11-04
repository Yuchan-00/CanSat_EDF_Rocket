#!/usr/bin/env python3
"""Generate ``firmware/atmega128_fc/src/gains.h`` (SPEC 8.6) for the ATmega128 flight computer.

Source: ``sim/out/gains.json`` (written by the simulation's tuner) when it exists and holds every
tuned key, otherwise ``params.json`` ``control``.  The key list is ``sim.controller.GAIN_KEYS`` so
the header can never drift from the reference controller.

The loop/IMU/baro rates are hardware facts of the firmware (Timer2 tick, MPU-6050 and MS5611
schedule) and always come from ``params.json`` ``control``; when ``gains.json`` was tuned at a
different ``loop_hz`` or against a different ``params.json`` the header says so
(``GAINS_LOOP_HZ_MISMATCH`` / ``GAINS_PARAMS_STALE``) and ``main.c`` turns that into a
compile-time ``#warning`` - gains tuned for another loop rate are flown only knowingly.

Stale rule (v0.7.1, architect's decision 13): ``gains.json`` records the sha256 of the whole ``params.json`` it was
tuned against, but the attitude tuning (``sim/run_scenarios.py`` ``tune_gains``: the ballistic 6-DOF scenarios on the
vehicle that ``params.json`` + the CAD report describe) does not read every key.  ``GAINS_PARAMS_STALE`` is therefore 0
when the whole file is the tuned one OR when its ATTITUDE-TUNING SUBTREES are: every key of ``params.json`` except
``ATTITUDE_TUNING_EXCLUDED`` -

* ``meta.date``, ``meta.version``           bookkeeping of the file, read by no simulation;
* ``mission.mode``, ``mission.mode_note``   the build's mission choice; the 6-DOF's ``Scenario.mode`` is deliberately not
                                            ``mission.mode`` (sim/sim6dof.py) and the tuner flies the ballistic scenarios;
* ``mission.vtol``                          the VTOL mission block (sim/onboard.py VTOL build, sim/vertical_ctrl.py, the
                                            VTOL 6-DOF runs) - the ballistic scenarios never read it;
* ``aero.vtol_*``                           VTOL-only aerodynamics: params.json aero.vtol_aero_note "VTOL 6-DOF only (the
                                            ballistic model stays bit-identical)";
* ``control.vtol_overrides``                the VTOL build's attitude-gain overrides (v0.7.1b): sim/controller.py
                                            gains_for_mode applies them in the VTOL build only, on top of the tuned
                                            gains; the ballistic tuner and the ballistic build never read them (v0.8);
* ``printer.color*``                        the display colour of the printed parts (CadQuery STEP colour, renders:
                                            cad/cadquery/rocket_geometry.py, tools/render_scene.py); no simulation reads
                                            it (the mass comes from printer.density_g_cm3, which counts).

Everything else - the vehicle geometry and masses, the EDF, vanes, servos, battery, electronics, recovery, control
(gains, rates), the ballistic mission thresholds, the remaining aero keys, the flight computer, the presets - counts.
The digest of those subtrees (``attitude_tuning_digest``: sha256 of the canonical JSON without the excluded keys) is
compared with the tuned file's: from ``gains.json`` ``_meta.params_attitude_digest`` when the tuner writes it, else from
``TUNED_PARAMS_DIGESTS`` (whole-file sha256 -> digest), an entry recorded once from the archived params file itself and
re-provable with ``--prove <that file>`` (its sha256 must be the one gains.json names, and its digest the recorded one).
A change in any counted key makes the gains STALE again.

Invariant: deterministic (no timestamps), names its source and that file's sha256; the build never
edits it.  Run from the project root: ``.venv/Scripts/python.exe tools/gen_gains_header.py``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import struct
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from sim.controller import GAIN_KEYS  # noqa: E402
from sim.sim6dof import SensorConfig  # noqa: E402

PARAMS = os.path.join(ROOT, "params.json")
GAINS_JSON = os.path.join(ROOT, "sim", "out", "gains.json")
OUT = os.path.join(ROOT, "firmware", "atmega128_fc", "src", "gains.h")
RATE_KEYS = ("loop_hz", "imu_hz", "baro_hz")
# the params.json keys the attitude tuning does not depend on (module docstring); "*" ends a key prefix
ATTITUDE_TUNING_EXCLUDED = (("meta", "date"), ("meta", "version"), ("mission", "mode"), ("mission", "mode_note"),
                            ("mission", "vtol"), ("aero", "vtol_*"), ("printer", "color*"), ("control", "vtol_overrides"))
# whole-file sha256 of a params.json the attitude gains were tuned against -> attitude_tuning_digest of that file.
# 0a5c261b...: the v0.6.3 params.json (sim/out/gains.json _meta.params_sha256, tuned 2026-10-15); recorded on 2026-10-20
# from the archived copy params_v063_before_vtol.json (sha256 0a5c261b5411396ee7abf68522a9aa9041d5fd83cf842d763485d5ecf4f234fc
# verified), provable with --prove.  v0.6.3 -> v0.7.1 changed only meta.date / meta.version, mission.mode / mode_note /
# vtol, aero.vtol_* and printer.color / color_note (all excluded).
TUNED_PARAMS_DIGESTS = {
    "0a5c261b5411396ee7abf68522a9aa9041d5fd83cf842d763485d5ecf4f234fc": "9db31b9c6efe61f8e82efac0b2e81bc1228456f6ef1023e4258227dc1f6c4500",
}
# estimator constants the simulation exports next to the gains (fallback: its SensorConfig default)
EST_KEYS = {"est_pad_avg_tau_s": "pad_avg_tau_s"}
BOOL_KEYS = ("yaw_hold",)


def sha256_file(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def attitude_tuning_subtrees(p: dict) -> dict:
    """params.json without ATTITUDE_TUNING_EXCLUDED (a deep copy)."""
    out = json.loads(json.dumps(p))
    for top, key in ATTITUDE_TUNING_EXCLUDED:
        node = out.get(top)
        if not isinstance(node, dict):
            continue
        for k in [k for k in node if (k.startswith(key[:-1]) if key.endswith("*") else k == key)]:
            del node[k]
    return out


def attitude_tuning_digest(p: dict) -> str:
    """sha256 of the canonical JSON (sorted keys, no whitespace) of the attitude-tuning subtrees."""
    text = json.dumps(attitude_tuning_subtrees(p), sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def rel(path: str) -> str:
    """``path`` relative to the project root with '/' separators; the absolute path when it is on another drive
    (Windows: os.path.relpath raises ValueError across drives, e.g. a --out or a test file under %TEMP%)."""
    try:
        r = os.path.relpath(path, ROOT)
    except ValueError:
        r = os.path.abspath(path)
    return r.replace(os.sep, "/")


def cf(x: float) -> str:
    v = struct.unpack("<f", struct.pack("<f", float(x)))[0]
    s = f"{v:.9g}"
    if "e" not in s and "." not in s:
        s += ".0"
    return s + "f"


def resolve(params_path: str = PARAMS, gains_path: str = GAINS_JSON) -> dict:
    """The gain set the firmware flies, with provenance.  Public: the cross-check uses it."""
    with open(params_path, "r", encoding="utf-8") as f:
        p = json.load(f)
    control = p["control"]
    g = {k: control[k] for k in GAIN_KEYS}
    info = {"source": "params.json control", "source_path": params_path, "warnings": [], "tuned_loop_hz": None,
            "loop_mismatch": False, "params_stale": False}
    if os.path.exists(gains_path):
        try:
            with open(gains_path, "r", encoding="utf-8") as f:
                gj = json.load(f)
            missing = [k for k in GAIN_KEYS if k not in gj and k not in RATE_KEYS]
            if missing:
                info["warnings"].append(f"{rel(gains_path)} lacks {missing}; using params.json control")
            else:
                for k in GAIN_KEYS:
                    if k not in RATE_KEYS:
                        g[k] = gj[k]
                info["source"] = "sim/out/gains.json"
                info["source_path"] = gains_path
                tl = gj.get("loop_hz")
                info["tuned_loop_hz"] = tl
                if tl is not None and int(tl) != int(control["loop_hz"]):
                    info["loop_mismatch"] = True
                    info["warnings"].append(f"gains.json was tuned at loop_hz {tl} but the firmware runs "
                                            f"params.control.loop_hz {control['loop_hz']} - re-run the sim tuning")
                meta = gj.get("_meta", {})
                info["stale_rule"] = "whole params.json"
                if meta.get("params_sha256") and meta["params_sha256"] != sha256_file(params_path):
                    tuned = meta.get("params_attitude_digest") or TUNED_PARAMS_DIGESTS.get(meta["params_sha256"])
                    now = attitude_tuning_digest(p)
                    info["attitude_digest"] = now
                    if tuned and tuned == now:
                        info["stale_rule"] = (f"params.json differs from the tuned one ({meta['params_sha256'][:16]}) only "
                                              f"outside the attitude-tuning subtrees (digest {now[:16]})")
                    else:
                        info["params_stale"] = True
                        info["stale_rule"] = ("attitude-tuning subtrees changed" if tuned else
                                              "no attitude-tuning digest recorded for the tuned params.json")
                        info["warnings"].append("gains.json _meta.params_sha256 differs from params.json and "
                                                + ("the attitude-tuning subtrees changed" if tuned else
                                                   "no attitude-tuning digest is recorded for it")
                                                + " - gains were tuned against another vehicle definition")
                info["tuned_meta"] = {k: meta.get(k) for k in ("mass_source", "mass_kg", "z_cg_mm", "tuning_scenario")}
        except (OSError, ValueError) as exc:
            info["warnings"].append(f"cannot read {gains_path}: {exc!r}; using params.json control")
    for k in RATE_KEYS:
        g[k] = control[k]
    gj_est = {}
    if info["source"] == "sim/out/gains.json":
        with open(gains_path, "r", encoding="utf-8") as f:
            gj_est = {k: v for k, v in json.load(f).items() if k in EST_KEYS}
    sens = SensorConfig()
    for k, attr in EST_KEYS.items():
        if k in gj_est:
            g[k] = float(gj_est[k])
        else:
            g[k] = float(getattr(sens, attr))
            info["warnings"].append(f"{k} not in the gains source; SensorConfig.{attr} = {g[k]} used")
    # PadAlignGyroEstimator: n_avg = max(1, round(pad_avg_tau_s / dt)), dt = 1 / loop_hz
    g["est_pad_n_avg"] = max(1, int(round(g["est_pad_avg_tau_s"] / (1.0 / float(control["loop_hz"])))))
    info["gains"] = g
    info["sha256"] = sha256_file(info["source_path"])
    return info


def render(info: dict) -> str:
    g = info["gains"]
    src_rel = rel(info["source_path"])
    out = ["/* gains.h - GENERATED by tools/gen_gains_header.py. DO NOT EDIT.",
           " *",
           f" * Source: {src_rel}  sha256 {info['sha256']}",
           " * Rates (loop/imu/baro) always from params.json control: they are fixed by the hardware schedule.",
           " * Keys: sim.controller.GAIN_KEYS (SPEC 8.6).",
           " */",
           "#ifndef GAINS_H",
           "#define GAINS_H",
           "",
           f'#define GAINS_SOURCE                       "{src_rel}"',
           f'#define GAINS_SOURCE_SHA256_PREFIX         "{info["sha256"][:16]}"']
    for k in GAIN_KEYS:
        name = "GAIN_" + k.upper()
        if k in RATE_KEYS:
            val = str(int(g[k]))
        elif k in BOOL_KEYS:
            val = "1" if bool(g[k]) else "0"
        else:
            val = cf(g[k])
        out.append(f"#define {name:<34} {val}")
    out += ["",
            "/* firmware estimator (sim/estimator.py PadAlignGyroEstimator) */",
            f"#define GAIN_EST_PAD_AVG_TAU_S             {cf(g['est_pad_avg_tau_s'])}",
            f"#define GAIN_EST_PAD_N_AVG                 {g['est_pad_n_avg']}U  /* round(tau / dt) */",
            "",
            f"#define GAINS_TUNED_LOOP_HZ                {int(info['tuned_loop_hz']) if info['tuned_loop_hz'] else 0}"
            "  /* loop_hz recorded in the gains source (0 = params.json) */",
            f"#define GAINS_LOOP_HZ_MISMATCH             {1 if info['loop_mismatch'] else 0}",
            f"#define GAINS_PARAMS_STALE                 {1 if info['params_stale'] else 0}"
            + (f"  /* {info['stale_rule']} */" if info.get("stale_rule") else ""),
            "",
            "#endif /* GAINS_H */",
            ""]
    return "\n".join(out)


def _diff_keys(a, b, path: str = "") -> list[str]:
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b)):
            out += _diff_keys(a.get(k, "<absent>"), b.get(k, "<absent>"), f"{path}.{k}" if path else k)
        return out
    return [] if a == b else [path]


def prove(old_path: str, params_path: str = PARAMS, gains_path: str = GAINS_JSON) -> int:
    """--prove: the archived params file vs gains.json and the recorded digest; the keys that differ from params.json
    and whether every one of them is excluded."""
    with open(gains_path, "r", encoding="utf-8") as f:
        meta = json.load(f).get("_meta", {})
    with open(old_path, "r", encoding="utf-8") as f:
        old = json.load(f)
    with open(params_path, "r", encoding="utf-8") as f:
        cur = json.load(f)
    sha = sha256_file(old_path)
    dg_old, dg_cur = attitude_tuning_digest(old), attitude_tuning_digest(cur)
    rec = TUNED_PARAMS_DIGESTS.get(sha)
    changed = _diff_keys(old, cur)
    counted = _diff_keys(attitude_tuning_subtrees(old), attitude_tuning_subtrees(cur))
    print(f"{rel(old_path)}: sha256 {sha}")
    print(f"  gains.json _meta.params_sha256 {meta.get('params_sha256')}: {'MATCH' if sha == meta.get('params_sha256') else 'DIFFERENT'}")
    print(f"  attitude-tuning digest {dg_old}; recorded {rec}: {'MATCH' if rec == dg_old else 'DIFFERENT'}")
    print(f"  params.json digest     {dg_cur}: {'EQUAL' if dg_cur == dg_old else 'DIFFERENT'}")
    print(f"  keys changed since: {changed}")
    print(f"  of which in the attitude-tuning subtrees: {counted or 'none'}")
    ok = sha == meta.get("params_sha256") and rec == dg_old
    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="exit 1 if the header on disk is not up to date")
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--prove", metavar="PARAMS_FILE",
                    help="check that PARAMS_FILE is the params.json gains.json names (sha256) and that its attitude-tuning "
                         "digest is the one TUNED_PARAMS_DIGESTS records; print the subtrees that differ from params.json")
    a = ap.parse_args(argv)
    if a.prove:
        return prove(a.prove)
    info = resolve()
    text = render(info)
    for w in info["warnings"]:
        print(f"gen_gains_header: WARNING {w}")
    old = open(a.out, "r", encoding="utf-8").read() if os.path.exists(a.out) else None
    if a.check:
        ok = old == text
        print(f"{rel(a.out)}: {'up to date' if ok else 'STALE'}")
        return 0 if ok else 1
    if old != text:
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    print(f"{rel(a.out)}: {'written' if old != text else 'unchanged'} from {info['source']} "
          f"(sha256 {info['sha256'][:16]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
