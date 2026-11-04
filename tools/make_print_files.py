"""Write print-ready STLs (print orientation baked in) and slice them with Bambu Studio.

Invariant enforced: the print files are derived from the Fusion 360 exports
(cad/exports/fusion/stl, vehicle coordinates) by rigid rotations only - no
geometry is edited here - and every print time / filament weight in the
print plan comes from Bambu Studio's own slicer (P2S + Bambu PLA Matte
system profiles), not from an estimate.  The support listed per part is what the
slicer wrote into the G-code header (``enable_support`` / ``support_type`` / ``support_style``), not an assumption; it is
configured as: none for the parts PARTS marks without supports; BASE_SUPPORT_TYPE (tree(auto)) for the payload, nose and
vanes; the PART_OVERRIDES for the launch pad (normal(auto), on the build plate only) and the intake (normal(auto), snug,
support_expansion -0.5 mm, plus the window support blockers below).  The header's support_type and support_style are
recorded (support_type_used / support_style_used); a support_type other than the configured one is printed as a WARNING.

Freshness (v0.6.1): the Fusion exports are used only when ``cad/exports/fusion360_report.json`` was written from
the current ``params.json`` (same sha256), by the repository's ``SCRIPT_VERSION``, with no failed component and
with every part's STL listed in its ``exports`` (a component that failed to export leaves the previous run's STL
on disk).  Otherwise nothing is written and the exit code is 2.  ``print_plan.json`` records that provenance
(``source``) and ``--check`` reports whether the print files on disk match the current params and script.
v0.6.2 (verifier findings on v0.6.1): the Fusion report must also be for the preset being sliced (``--preset``, default
params.json meta.preset = BUDGET), and every required-feature block of ``derived.extra`` (intake_windows,
intake_servo_leads, servo_leads, com_mount, ...) must have an empty ``containment_unanswered`` list - a feature whose
BRepBody.pointContainment evidence Fusion could not give is unconfirmed geometry - unless ``--allow-unanswered`` is given
(then the lists are recorded in ``source`` and --check prints them).
v0.6.2 (user decision A, intake_module.window_bottom_end; verifier finding: the slicer still put supports inside every
window): the intake is printed from a Bambu Studio project, cad/exports/print/intake_module.3mf = the oriented part + one
support_blocker volume per window opening (from the Layout) + Bambu's own resolved settings, and its sliced G-code is
checked (``window_support_check``: every support extrusion mapped back to the design frame through the slicer's own
placement, registration proven on the band ring and on every strut side face, then counted where its bead can overlap
a window opening - the whole sector up to the strut faces and the band's full radial extent, grown by half the support
line width; support hugging a strut is inside it).  Anything but PASS is recorded in the plan, makes --check report
STALE and the exit code 3.  The bare part is sliced first as a control (and for the settings); max_bridge_length /
bridge_no_support were measured and do not help (PART_OVERRIDES).  A part has one print file: writing its STL removes
an older <part>.3mf and writing the project removes an older <part>.stl.
v0.6.3 (geometry-aware freshness): print_plan.json records the sha256 of every source STL as it was read for slicing
(``source.stl_sha256``), and ``--check`` judges the print set by its geometry, not by the params.json bytes: the print
files are current when (1) the Fusion run on disk is valid for the preset - made by the repository's script from the
current params.json for that preset, every part exported, no failed component, no unanswered point containment (unless
the plan was made with --allow-unanswered) - (2) every sliced part's source STL still has the sha256 recorded when it
was sliced, (3) every print file of the plan is on disk and (4) the intake window check passed.  A params.json change
that changes no printed STL therefore leaves the set current once the Fusion script has been run on it (until then
the STLs on disk are not known to be the current design's: not verified, exit 2); the plan's own params_sha256 /
script_version are only reported.  ``--adopt`` gives a plan written before v0.6.3 its source hashes, part by part,
only where the print file is proven to be the current source STL oriented (byte-identical STL, or the same part mesh in
the 3MF project); nothing is assumed for a part whose print file is missing or differs.

Usage:
    python tools/make_print_files.py            # orient + slice everything
    python tools/make_print_files.py --no-slice # orient only
    python tools/make_print_files.py --check    # are the print files current? (exit 0 yes, 2 no)
    python tools/make_print_files.py --adopt    # record the source STL hashes of a pre-v0.6.3 plan (verified per part)
    python tools/make_print_files.py --preset NAME --allow-unanswered   # other preset / accept unanswered containment
Outputs: cad/exports/print/<part>.stl (intake_module.3mf instead for the intake when its blocked project was made),
print_plan.{json,md}
Exit codes: 0 ok, 2 refused / stale / not verified, 3 written but the intake window support check is not PASS.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import zipfile

import numpy as np
import trimesh

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SRC = os.path.join(ROOT, "cad", "exports", "fusion", "stl")
OUT = os.path.join(ROOT, "cad", "exports", "print")
FUSION_REPORT = os.path.join(ROOT, "cad", "exports", "fusion360_report.json")
PARAMS_JSON = os.path.join(ROOT, "params.json")
FUSION_SCRIPT = os.path.join(ROOT, "cad", "fusion360", "CanSatEDFRocket", "CanSatEDFRocket.py")
BAMBU =r"C:\Program Files\Bambu Studio\bambu-studio.exe"
PROFILES = r"C:\Program Files\Bambu Studio\resources\profiles\BBL"
MACHINE = "Bambu Lab P2S 0.4 nozzle"
PROCESS = "0.20mm Standard @BBL P2S"
FILAMENT = "Bambu PLA Matte @BBL P2S"
FILAMENT_D_MM = 1.75

sys.path.insert(0, os.path.join(ROOT, "cad"))
import layout  # noqa: E402

# part -> (quantity, list of candidate orientations, supports needed, why)
# orientation = list of (axis, degrees) rotations applied in order (vehicle frame -> print frame)
PARTS: dict[str, tuple[int, list[list[tuple[str, float]]], bool, str]] = {
    "propulsion_module": (1, [[("z", 45.0)]], False,
                          "upright, exhaust exit on the bed, turned 45 deg on the plate (v0.9: fins on the axes, 246.8 mm "
                          "across - axis-aligned the P2S plate refuses it); no support: fairing keels 45 deg, pocket ceilings "
                          "bridge onto the service-side lintels, the lintels bridge the 40 mm openings"),
    "intake_module": (1, [[("x", 180.0)]], True,
                      "printed flipped (SPEC 5.1 / 12): the spigot stands on the bed; the 45 deg floor wedges keep supports out "
                      "of the closed lead channels only in this orientation (upright, the channel roofs would need support "
                      "inside); v0.6.2: every window ceiling is a 45 deg truncated gable whose flat is bridged, and the part "
                      "is a Bambu project with a support blocker over each window opening (snug supports, shrunk 0.5 mm), "
                      "so no support inside the windows - checked on the sliced G-code; supports stay under the bulkhead "
                      "inside the spigot, under the shelf ledge, the spigot step and the lead bulges"),
    "avionics_module": (1, [[]], False, "upright"),
    "payload_module": (1, [[]], True, "upright; the bulkhead 15 mm above the bed and (v0.6) the four COM-board bosses "
                                      "under it need short supports inside the skirt"),
    "nose_cone": (1, [[]], True, "base down; the anchor bar spans the bore 4 mm above the skirt and needs supports"),
    "avionics_tray": (1, [[("y", 90.0)]], False, "flat"),
    "piston": (1, [[("x", 180.0)]], False, "flat, spring recess facing up"),
    "vane": (4, [[("x", -90.0)]], True, "flat; the D5 shaft stands 1 mm proud of the 3 mm plate -> supports on build plate"),
    "vane_horn": (4, [[("y", 90.0)]], False, "flat on its 8 x 17 face; the D-bore prints vertical (round)"),
    "latch_arm": (1, [[]], False, "flat"),
    "launch_pad": (1, [[("x", 180.0)]], True, "flipped: stands on the fin-locator ribs, legs up; grid supports fill the 4 mm gap under the plate"),
    # v0.9 direct-drive vanes and the open motor-lead groove (vane_drive.mode 'direct', intake wire_channel.open)
    "vane_coupler": (4, [[("y", 90.0)]], False, "upright, flange on the bed, D-bore vertical"),
    "servo_cover": (4, [[("x", 90.0)]], False, "flat"),
    "wire_cover": (1, [[("y", 90.0)]], False, "flat"),
}


def orient_matrix(mesh: trimesh.Trimesh, rots: list[tuple[str, float]]) -> np.ndarray:
    """4 x 4 vehicle -> print-STL transform of ``orient``: the rotations in order, then dropped onto the bed and
    centred on the 256 mm plate."""
    T = np.eye(4)
    for axis, deg in rots:
        vec = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}[axis]
        T = trimesh.transformations.rotation_matrix(math.radians(deg), vec) @ T
    v = trimesh.transform_points(mesh.vertices, T)
    lo, hi = v.min(axis=0), v.max(axis=0)
    return trimesh.transformations.translation_matrix(
        [128.0 - (lo[0] + hi[0]) / 2, 128.0 - (lo[1] + hi[1]) / 2, -lo[2]]) @ T


def orient(mesh: trimesh.Trimesh, rots: list[tuple[str, float]]) -> trimesh.Trimesh:
    m = mesh.copy()
    m.apply_transform(orient_matrix(mesh, rots))
    return m


def _rel(path: str) -> str:
    """``path`` relative to the project root with '/' separators (absolute when on another drive)."""
    try:
        return os.path.relpath(path, ROOT).replace(os.sep, "/")
    except ValueError:
        return path


def _sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def repo_script_version() -> str:
    """SCRIPT_VERSION of the repository's Fusion script (the one tools/install_fusion_script.py installs)."""
    with open(FUSION_SCRIPT, "r", encoding="utf-8") as f:
        m = re.search(r'^SCRIPT_VERSION = "([^"]+)"', f.read(), re.M)
    if m is None:
        raise RuntimeError(f"SCRIPT_VERSION not found in {FUSION_SCRIPT}")
    return m.group(1)


def default_preset() -> str:
    """The preset the print files are made for unless --preset says otherwise: the repository params.json's
    meta.preset (BUDGET), the one ``layout.build()`` lays out."""
    return str(layout.load_params()["meta"]["preset"])


def unanswered_containment(rep: dict) -> dict[str, list[str]]:
    """Every non-empty ``containment_unanswered`` list of the report's required-feature blocks (derived.extra.<block>:
    intake_windows, intake_servo_leads, servo_leads, com_mount, ... - any block that carries one): features whose
    BRepBody.pointContainment evidence Fusion could not give, so their geometry is not confirmed."""
    extra = ((rep.get("derived") or {}).get("extra") or {})
    out: dict[str, list[str]] = {}
    for name, block in extra.items():
        if isinstance(block, dict):
            lst = block.get("containment_unanswered")
            if lst:
                out[name] = [str(x) for x in lst]
    return out


def fusion_source(names: list[str], preset: str | None = None,
                  allow_unanswered: bool = False) -> tuple[dict, list[str]]:
    """Provenance of the Fusion exports (from fusion360_report.json) and every reason they must not be used to make
    the print files of ``names`` for ``preset`` (default: params.json meta.preset); an empty list means they were built
    for that preset from the current params.json by the repository's script, with every part exported in that run and
    every required feature's point-containment evidence answered (unless ``allow_unanswered``)."""
    if not os.path.isfile(FUSION_REPORT):
        return {}, [f"{_rel(FUSION_REPORT)} missing: no Fusion run on record"]
    with open(FUSION_REPORT, "r", encoding="utf-8") as f:
        rep = json.load(f)
    gv = str(rep.get("generator_version", ""))
    m = re.match(r"CanSatEDFRocket\.py (\S+);", gv)
    want = preset or default_preset()
    unanswered = unanswered_containment(rep)
    src = {"fusion_report": _rel(FUSION_REPORT),
           "params_sha256": rep.get("params_sha256"), "script_version": m.group(1) if m else None,
           "generator_version": gv, "preset": rep.get("preset"), "preset_sliced": want,
           "containment_unanswered": unanswered, "allow_unanswered": bool(allow_unanswered)}
    problems: list[str] = []
    if src["preset"] != want:
        problems.append(f"the Fusion run is for preset {src['preset']!r}, these print files are for {want!r} "
                        f"(set cad/fusion360/preset.txt and run the script again)")
    if unanswered and not allow_unanswered:
        problems.append("required features without point-containment evidence in that Fusion run (their geometry is "
                        "not confirmed; --allow-unanswered accepts them knowingly): "
                        + "; ".join(f"{k}: {', '.join(v)}" for k, v in unanswered.items()))
    sha_now = _sha256(PARAMS_JSON)
    if src["params_sha256"] != sha_now:
        problems.append(f"params.json changed since the Fusion run (report params_sha256 {str(src['params_sha256'])[:12]}..., "
                        f"params.json now {sha_now[:12]}...)")
    ver_now = repo_script_version()
    if src["script_version"] != ver_now:
        problems.append(f"the Fusion run used CanSatEDFRocket.py {src['script_version']}, the repository has {ver_now}")
    failed = rep.get("failed_components") or {}
    if failed:
        problems.append(f"the Fusion run has failed components: {', '.join(sorted(failed))}")
    exported = rep.get("exports") or {}
    missing = [n for n in names if f"STL {n}" not in exported]
    if missing:
        problems.append(f"STL not exported by that Fusion run (any file on disk is from an older run): {', '.join(missing)}")
    return src, problems


def source_stl_path(name: str) -> str:
    return os.path.join(SRC, f"{name}.stl")


def load_source_stl(name: str) -> tuple[trimesh.Trimesh, str]:
    """The Fusion export of ``name`` (vehicle coordinates) and the sha256 of exactly the bytes the mesh was read from."""
    with open(source_stl_path(name), "rb") as f:
        data = f.read()
    return trimesh.load(trimesh.util.wrap_as_stream(data), file_type="stl", force="mesh"), hashlib.sha256(data).hexdigest()


def check_current(preset: str | None = None) -> int:
    """--check (v0.6.3, geometry-aware): 0 when the print set is current for ``preset`` (default params.json meta.preset),
    else 2.  Current = (1) the Fusion run on disk is valid for the preset (``fusion_source``: the current params.json and
    Fusion script, every part exported, no failed component, every required feature's containment answered - or accepted
    with --allow-unanswered when the plan was made), (2) every part's source STL has the sha256 the plan recorded when it
    was sliced (or adopted), (3) every print file of the plan is on disk, (4) a required intake window check passed.  The
    plan's own params_sha256 / script_version are reported, not judged: a params.json or script change that changes no
    printed STL leaves the set current once Fusion has been run on it."""
    plan_path = os.path.join(OUT, "print_plan.json")
    problems: list[str] = []
    notes: list[str] = []
    want = preset or default_preset()
    if not os.path.isfile(plan_path):
        problems.append(f"{_rel(plan_path)} missing")
    else:
        with open(plan_path, "r", encoding="utf-8") as f:
            plan = json.load(f)
        src = plan.get("source") or {}
        parts = plan.get("parts") or {}
        if not src:
            problems.append("print_plan.json records no source (written before v0.6.1): the Fusion exports the print "
                            "files were made from are unknown")
        else:
            if src.get("preset") != want:
                problems.append(f"made from a Fusion run of preset {src.get('preset')!r}, not {want!r}")
            # (1) the Fusion run on disk describes the current design for this preset
            rep, rep_problems = fusion_source(list(PARTS), want, bool(src.get("allow_unanswered")))
            problems += [f"the Fusion run on disk is not valid for {want!r}: {p}" for p in rep_problems]
            if rep.get("containment_unanswered"):
                notes.append("the plan was made with --allow-unanswered; the current Fusion run leaves unanswered: "
                             + "; ".join(f"{k}: {', '.join(v)}" for k, v in rep["containment_unanswered"].items()))
            # (2) every sliced part's source STL is byte for byte the one it was sliced (or adopted) from
            recorded = src.get("stl_sha256") or {}
            if not recorded:
                problems.append("print_plan.json records no source STL sha256 (written before v0.6.3): which Fusion "
                                "exports the print files were made from is unknown - run make_print_files.py, or "
                                "--adopt to prove it part by part")
            else:
                for n in PARTS:
                    if n not in parts:
                        continue
                    path = source_stl_path(n)
                    if n not in recorded:
                        problems.append(f"{n}: no source STL sha256 recorded"
                                        + (f" ({src['stl_sha256_not_adopted'][n]})"
                                           if n in (src.get("stl_sha256_not_adopted") or {}) else ""))
                    elif not os.path.isfile(path):
                        problems.append(f"{n}: source STL {_rel(path)} missing")
                    elif _sha256(path) != recorded[n]:
                        problems.append(f"{n}: {_rel(path)} changed since it was sliced (sha256 "
                                        f"{recorded[n][:12]}... -> {_sha256(path)[:12]}...)")
            # reported only: the bytes of params.json / the script version the plan was made from
            sha_now = _sha256(PARAMS_JSON)
            if src.get("params_sha256") != sha_now:
                notes.append(f"sliced from a Fusion run of params.json {str(src.get('params_sha256'))[:12]}... (now "
                             f"{sha_now[:12]}...)")
            if src.get("script_version") != repo_script_version():
                notes.append(f"sliced from CanSatEDFRocket.py {src.get('script_version')} exports (the repository has "
                             f"{repo_script_version()})")
        absent = [n for n in PARTS if n not in parts]
        if absent:
            problems.append(f"parts not in the plan: {', '.join(absent)}")
        # (3) the plan's print files are on disk
        for n, v in parts.items():
            pf = v.get("print_file")
            if not pf:
                problems.append(f"{n}: the plan names no print file")
            elif not os.path.isfile(os.path.join(OUT, pf)):
                problems.append(f"{n}: print file {pf} is not in {_rel(OUT)} (deleted or renamed after slicing)")
        # (4) v0.6.2 decision A: a part with a required window check must have passed it on its sliced G-code
        for n, v in parts.items():
            ws = v.get("window_support")
            if ws and ws.get("required") and ws.get("status") != "PASS":
                problems.append(f"{n}: window support check {ws.get('status')}"
                                + (f" ({ws['reason']})" if ws.get("reason") else ""))
            elif n == "intake_module" and v.get("ok") and not ws:
                problems.append(f"{n}: sliced without the v0.6.2 window support check")
    for n in notes:
        print(f"NOTE: {n}")
    if problems:
        print("STALE: cad/exports/print is not shown to match the current design - do not print these files:")
        for p in problems:
            print(f"  - {p}")
        return 2
    print(f"OK: every print file was made from the Fusion export now on disk, and that Fusion run is valid for {want!r} "
          "(current params.json and script, every part exported, nothing failed or unanswered)"
          + (" - no printed STL changed since slicing" if notes else ""))
    return 0


def print_file_is_oriented_source(entry: dict, mesh: trimesh.Trimesh) -> tuple[bool, str]:
    """Is the plan entry's print file ``mesh`` (a source STL) in the entry's orientation, as main() writes it?  STL:
    byte-identical to ``orient(mesh, rotations)`` exported; 3MF project: its normal part has exactly that mesh (vertices
    as write_bambu_project writes them, same faces).  Returns (verdict, evidence)."""
    pf = entry.get("print_file")
    path = os.path.join(OUT, pf) if pf else ""
    if not pf or not os.path.isfile(path):
        return False, f"print file {pf} not in {_rel(OUT)}"
    m = orient(mesh, [(str(ax), float(deg)) for ax, deg in entry.get("rotations", [])])
    if pf.endswith(".stl"):
        with open(path, "rb") as f:
            same = f.read() == m.export(file_type="stl")
        return same, ("byte-identical to the oriented source STL" if same else "differs from the oriented source STL")
    if pf.endswith(".3mf"):
        try:
            with zipfile.ZipFile(path) as z:
                cfg = z.read("Metadata/model_settings.config").decode("utf-8", "replace")
                obj = z.read("3D/Objects/object_1.model").decode("utf-8", "replace")
        except (KeyError, zipfile.BadZipFile, OSError) as e:
            return False, f"{pf} is not a project make_print_files wrote ({e})"
        ids = re.findall(r"<part id=\"(\d+)\" subtype=\"normal_part\"", cfg)
        blk = re.search(rf"<object id=\"{ids[0]}\"[^>]*>(.*?)</object>", obj, re.S) if len(ids) == 1 else None
        if blk is None:
            return False, f"{pf}: no single normal part"
        V = re.findall(r"<vertex x=\"([^\"]+)\" y=\"([^\"]+)\" z=\"([^\"]+)\"", blk.group(1))
        F = re.findall(r"<triangle v1=\"(\d+)\" v2=\"(\d+)\" v3=\"(\d+)\"", blk.group(1))
        same = (V == [(f"{x:.6f}", f"{y:.6f}", f"{z:.6f}") for x, y, z in np.asarray(m.vertices)]
                and F == [(str(a), str(b), str(c)) for a, b, c in np.asarray(m.faces)])
        return same, (f"the project's part mesh is the oriented source STL ({len(V)} vertices, {len(F)} triangles)" if same
                      else "the project's part mesh differs from the oriented source STL")
    return False, f"{pf}: unknown print file type"


def adopt_sources(preset: str | None = None) -> int:
    """--adopt: give a print_plan.json written before v0.6.3 its source STL hashes.  A part's current source STL hash
    is recorded only when its print file is proven to be that STL oriented as the plan says
    (print_file_is_oriented_source); parts already recorded are kept, every other part is listed with the reason in
    source.stl_sha256_not_adopted.  Writes nothing else.  Exit 0 when every plan part has a hash afterwards, else 2."""
    plan_path = os.path.join(OUT, "print_plan.json")
    if not os.path.isfile(plan_path):
        print(f"REFUSED: {_rel(plan_path)} missing")
        return 2
    with open(plan_path, "r", encoding="utf-8") as f:
        plan = json.load(f)
    src = plan.get("source") or {}
    want = preset or default_preset()
    if not src or src.get("preset") != want:
        print(f"REFUSED: the plan was not made for preset {want!r} (source preset {src.get('preset')!r})")
        return 2
    recorded = dict(src.get("stl_sha256") or {})
    how = dict(src.get("stl_sha256_how") or {}) if isinstance(src.get("stl_sha256_how"), dict) else {}
    not_adopted: dict[str, str] = {}
    for n, entry in (plan.get("parts") or {}).items():
        if n in recorded:
            continue
        if not os.path.isfile(source_stl_path(n)):
            not_adopted[n] = f"source STL {_rel(source_stl_path(n))} missing"
            continue
        mesh, sha = load_source_stl(n)
        ok, why = print_file_is_oriented_source(entry, mesh)
        if ok:
            recorded[n] = sha
            how[n] = f"adopted (--adopt): {why}"
        else:
            not_adopted[n] = f"not adopted: {why}"
        print(f"{n}: {'adopted ' + sha[:12] + '...' if ok else 'NOT adopted'} - {why}")
    src["stl_sha256"] = {n: recorded[n] for n in plan.get("parts", {}) if n in recorded}
    src["stl_sha256_how"] = {n: how.get(n, "hashed when the STL was read for slicing") for n in src["stl_sha256"]}
    if not_adopted:
        src["stl_sha256_not_adopted"] = not_adopted
    else:
        src.pop("stl_sha256_not_adopted", None)
    plan["source"] = src
    with open(plan_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(plan, f, indent=2, ensure_ascii=False)
    print(f"{_rel(plan_path)}: {len(src['stl_sha256'])} of {len(plan.get('parts', {}))} parts carry a source STL sha256"
          + (f"; not adopted: {', '.join(not_adopted)}" if not_adopted else ""))
    return 0 if not not_adopted else 2


def _profile_copy(kind: str, name: str, overrides: dict, tmp: str) -> str:
    """Copy a Bambu system profile, apply overrides, return the new path (inheritance kept)."""
    src = os.path.join(PROFILES, kind, f"{name}.json")
    with open(src, "r", encoding="utf-8") as f:
        prof = json.load(f)
    prof.update(overrides)
    dst = os.path.join(tmp, f"{kind}_{re.sub(r'[^A-Za-z0-9]+', '_', name)}_{len(overrides)}.json")
    with open(dst, "w", encoding="utf-8") as f:
        json.dump(prof, f, indent=1)
    return dst


# per-part slicer overrides (only where the defaults do not work)
PART_OVERRIDES: dict[str, dict] = {
    # 240 mm plate on a 256 mm bed: an auto brim + skirt push it off the plate
    # tree supports under the flipped pad spread outside the plate and the slicer refuses the plate
    "launch_pad": {"brim_type": "no_brim", "skirt_loops": "0", "support_type": "normal(auto)",
                   "support_on_build_plate_only": "1"},
    # v0.6.2 (user decision A: no support inside the windows): the intake is sliced as a Bambu project with a support
    # blocker over every window opening (intake_window_blockers).  Blockers only drop support CONTACTS.  Measured on
    # the CadQuery twin (BUDGET, Bambu Studio 02.08.02.61, support path inside the four openings): no blockers 31.6 m
    # (snug) / 22.4 m (tree); with blockers, tree (organic, the process default) 9.8 m - its branches route from the bed
    # through the windows to the shelf ledge - tree_slim / tree_hybrid 4.6 m, snug normal 0.5 m (a 1.4 mm pillar at one
    # window centre), snug normal with support_expansion -0.5 0 m.  -0.5 shrinks every support 0.5 mm in from its
    # overhang's edges; the spigot-step ring, bulkhead, shelf-ledge and bulge supports stay, the 14 / 9 mm lead-notch
    # ceilings lose most of theirs and print as bridges (shorter than the accepted 20 mm window bridge); -0.8 also
    # dropped the 1.4 mm spigot-step support.  max_bridge_length 25 / bridge_no_support 1 changed nothing in the
    # windows (the band's perimeters span them as curved overhang walls, not "straight bridges"), and bridge_no_support
    # dropped most of the support under the bulkhead over the spigot bore - neither is used.
    "intake_module": {"support_type": "normal(auto)", "support_style": "snug", "support_expansion": "-0.5"},
}
BASE_SUPPORT_TYPE = "tree(auto)"          # process default for every part with supports (PART_OVERRIDES may change it)
SUPPORT_LABEL_KO = {"none": "없음", "tree(auto)": "트리", "tree(manual)": "트리 (수동)",
                    "normal(auto)": "일반 (그리드)", "normal(manual)": "일반 (그리드, 수동)"}
SUPPORT_STYLE_LABEL_KO = {"snug": "밀착형", "grid": "그리드", "tree_slim": "가는 트리", "tree_strong": "굵은 트리",
                          "tree_hybrid": "혼합 트리", "tree_organic": "유기형 트리"}


def configured_support_type(name: str, supports: bool) -> str:
    """Support type the slicer is told to use for a part: 'none', else the process value after its overrides."""
    if not supports:
        return "none"
    return str(PART_OVERRIDES.get(name, {}).get("support_type", BASE_SUPPORT_TYPE))


def used_support_type(name: str, supports: bool, entry: dict) -> str:
    """Support type actually used: what Bambu Studio wrote into the G-code header (enable_support, support_type)
    when the part was sliced, else the configured one."""
    seen = entry.get("settings_in_gcode") or {}
    if entry.get("ok") and "enable_support" in seen:
        return "none" if seen["enable_support"] == "0" else seen.get("support_type", configured_support_type(name, supports))
    return configured_support_type(name, supports)


def support_label(kind: str, style: str | None = None) -> str:
    """Korean label of a support type (and of a non-default support_style from the G-code header)."""
    if kind == "none":
        return SUPPORT_LABEL_KO["none"]
    base = SUPPORT_LABEL_KO.get(kind, kind)
    if style and style != "default":
        base = base.split(" (")[0] + f" ({SUPPORT_STYLE_LABEL_KO.get(style, style)})"
        return f"{base} (`{kind}`, `{style}`)"
    return f"{base} (`{kind}`)"


# --------------------------------------------------------------------------- #
# v0.6.2 (verifier finding on user decision A, intake_module.window_bottom_end): no support inside the intake windows.
# The gable faces are steeper than the support threshold, but Bambu Studio still supports the flat window ceiling (the
# ring's perimeters span it as curved overhang walls, which it does not treat as bridges).  So the intake is sliced as
# a Bambu project with one support_blocker modifier per window opening (intake_window_blockers, from the Layout), and
# the sliced G-code is checked: no support extrusion may lie inside any window opening (window_support_check).
# --------------------------------------------------------------------------- #
BLOCKER_R_MARGIN_MM = 0.3            # window blocker: r [r_band_in - this, body_r + this] over the window sector ...
BLOCKER_CEILING_MM = 1.0             # ... and Z [z_window_bottom - this, z_window_top] (1 mm into the ceiling)
BLOCKER_ARC_STEPS = 48
SUPPORT_FEATURE_PREFIX = "Support"   # Bambu feature names: Support, Support interface, Support transition
WALL_FEATURES = ("Outer wall", "Inner wall")
CEILING_FEATURES = ("Bridge", "Overhang wall")   # how the layer spanning a window ceiling is printed (evidence only)
SAMPLE_STEP_MM = 1.0                 # extrusion paths are sampled at most this far apart
# Window-opening probe (verifier finding on v0.6.2: the probe stayed 1 deg inside the strut faces and 0.3 mm inside the
# band faces, so it could not see support hugging a strut - the slicer puts that support's centre line only
# support_object_xy_distance + support_line_width / 2 = 0.35 + 0.2 mm inside the face).  The opening is the whole window
# sector up to the radial strut faces x the band r [r_band_in, body_r] x Z [z_window_bottom, z_window_top]; a support
# sample counts when its BEAD can overlap the opening: the probe is the opening grown by half the support line width
# (G-code support_line_width, else line_width) across the strut faces and the band faces and by half the layer height in
# Z.  Support the slicer keeps beside the band (in the bore behind a strut, outside the body) has its bead
# support_object_xy_distance outside a face, so it stays out of the probe while the frame error is below that distance
# - the registration below bounds it (STRUT_FACE_TOL_MM, RING_FIT_TOL_MM < 0.35 mm); measured on the CadQuery twin:
# strut faces within 0.001 mm, the nearest support outside the openings 0.445 mm (bead edge, bore side) from the probe.
RING_LAYER_MARGIN_MM = 0.5           # registration ring: the solid band above the windows, this far inside its ends
RING_FIT_TOL_MM = 0.25               # its outer perimeter centre line must fit body_r - line_w / 2 within this ...
RING_FIT_RMS_MAX_MM = 0.1            # ... with at most this rms deviation
STRUT_CHECK_MARGIN_MM = 2.0          # strut-wall registration layers: z_gable_top + this .. z_window_top - this
STRUT_FACE_TOL_MM = 0.1              # every strut side face's outer-wall centre line must lie line_w / 2 inside the face
                                     # within this (median over its samples; fixes the frame's turn about the axis, which
                                     # the ring cannot see, and bounds any XY shift to this / cos(window_half_angle))
STRUT_FACE_MIN_SAMPLES = 20          # ... from at least this many samples per face
STRUT_FACE_SELECT_MM = 1.0           # outer-wall samples within this of a face, r_band_in + line_w .. body_r - line_w
_NUM = r"(-?(?:\d+\.?\d*|\.\d+))"
_RE_X, _RE_Y, _RE_E = re.compile(r"X" + _NUM), re.compile(r"Y" + _NUM), re.compile(r"E" + _NUM)
_RE_I, _RE_J = re.compile(r"I" + _NUM), re.compile(r"J" + _NUM)


def sector_mesh(r0: float, r1: float, a0_deg: float, a1_deg: float, z0: float, z1: float,
                n: int = BLOCKER_ARC_STEPS) -> trimesh.Trimesh:
    """Closed annular sector r [r0, r1], theta [a0, a1] (deg, a1 - a0 < 360), Z [z0, z1], outward winding (n arc steps)."""
    a = np.radians(np.linspace(a0_deg, a1_deg, n + 1))
    m = n + 1
    V = np.array([(r * math.cos(t), r * math.sin(t), z) for z in (z0, z1) for r in (r0, r1) for t in a])

    def i(zi: int, ri: int, k: int) -> int:
        return (zi * 2 + ri) * m + k

    F = []
    for k in range(n):
        F += [[i(0, 0, k), i(0, 1, k + 1), i(0, 1, k)], [i(0, 0, k), i(0, 0, k + 1), i(0, 1, k + 1)],      # bottom
              [i(1, 0, k), i(1, 1, k), i(1, 1, k + 1)], [i(1, 0, k), i(1, 1, k + 1), i(1, 0, k + 1)],      # top
              [i(0, 1, k), i(0, 1, k + 1), i(1, 1, k + 1)], [i(0, 1, k), i(1, 1, k + 1), i(1, 1, k)],      # outer
              [i(0, 0, k), i(1, 0, k + 1), i(0, 0, k + 1)], [i(0, 0, k), i(1, 0, k), i(1, 0, k + 1)]]      # inner
    F += [[i(0, 0, 0), i(0, 1, 0), i(1, 1, 0)], [i(0, 0, 0), i(1, 1, 0), i(1, 0, 0)],                      # end a0
          [i(0, 0, n), i(1, 1, n), i(0, 1, n)], [i(0, 0, n), i(1, 0, n), i(1, 1, n)]]                      # end a1
    out = trimesh.Trimesh(V, np.array(F), process=True)
    if not (out.is_watertight and out.is_winding_consistent and out.volume > 0):
        raise ValueError(f"sector mesh r {r0}..{r1} theta {a0_deg}..{a1_deg} z {z0}..{z1} is not a closed solid")
    return out


def intake_window_blockers(L) -> list[trimesh.Trimesh]:
    """One support-blocker solid per window (design frame): the window sector (Layout.window_sectors), r [r_band_in -
    BLOCKER_R_MARGIN_MM, body_r + BLOCKER_R_MARGIN_MM], Z [z_window_bottom - BLOCKER_CEILING_MM, z_window_top]."""
    return [sector_mesh(L.r_band_in - BLOCKER_R_MARGIN_MM, L.body_r + BLOCKER_R_MARGIN_MM, lo, hi,
                        L.z_window_bottom - BLOCKER_CEILING_MM, L.z_window_top) for lo, hi in L.window_sectors]


_NS_3MF = ('xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
           'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
           'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p"')
_IDENTITY_3MF = "1 0 0 0 1 0 0 0 1 0 0 0"
_IDENTITY_16 = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"


def write_bambu_project(path: str, name: str, part: trimesh.Trimesh,
                        modifiers: list[tuple[trimesh.Trimesh, str, str]], project_settings: bytes) -> None:
    """A Bambu Studio project (3MF) with one object: ``part`` (normal_part, print coordinates) plus ``modifiers``
    [(mesh, subtype e.g. 'support_blocker', name)], and ``project_settings`` (Metadata/project_settings.config - the
    CLI crashes on a Bambu project without it; take it from a project Bambu Studio exported).  Deterministic bytes."""
    meshes = [(part, "normal_part", name)] + list(modifiers)
    obj = ['<?xml version="1.0" encoding="UTF-8"?>', f'<model unit="millimeter" xml:lang="en-US" {_NS_3MF}>',
           ' <metadata name="BambuStudio:3mfVersion">1</metadata>', ' <resources>']
    for k, (mm, _, _) in enumerate(meshes, start=1):
        obj += [f'  <object id="{k}" type="model">', '   <mesh>', '    <vertices>']
        obj += [f'     <vertex x="{x:.6f}" y="{y:.6f}" z="{z:.6f}"/>' for x, y, z in np.asarray(mm.vertices)]
        obj += ['    </vertices>', '    <triangles>']
        obj += [f'     <triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in np.asarray(mm.faces)]
        obj += ['    </triangles>', '   </mesh>', '  </object>']
    obj += [' </resources>', ' <build/>', '</model>']
    top = len(meshes) + 1
    mod = ['<?xml version="1.0" encoding="UTF-8"?>', f'<model unit="millimeter" xml:lang="en-US" {_NS_3MF}>',
           ' <metadata name="Application">BambuStudio-02.08.02.61</metadata>',
           ' <metadata name="BambuStudio:3mfVersion">1</metadata>', ' <resources>', f'  <object id="{top}" type="model">',
           '   <components>']
    mod += [f'    <component p:path="/3D/Objects/object_1.model" objectid="{k}" transform="{_IDENTITY_3MF}"/>'
            for k in range(1, top)]
    mod += ['   </components>', '  </object>', ' </resources>', ' <build>',
            f'  <item objectid="{top}" transform="{_IDENTITY_3MF}" printable="1"/>', ' </build>', '</model>']
    cfg = ['<?xml version="1.0" encoding="UTF-8"?>', '<config>', f'  <object id="{top}">',
           f'    <metadata key="name" value="{name}"/>', '    <metadata key="extruder" value="1"/>']
    for k, (_, subtype, pname) in enumerate(meshes, start=1):
        cfg += [f'    <part id="{k}" subtype="{subtype}">', f'      <metadata key="name" value="{pname}"/>',
                f'      <metadata key="matrix" value="{_IDENTITY_16}"/>', '    </part>']
    cfg += ['  </object>', '  <plate>', '    <metadata key="plater_id" value="1"/>', '    <model_instance>',
            f'      <metadata key="object_id" value="{top}"/>', '      <metadata key="instance_id" value="0"/>',
            '    </model_instance>', '  </plate>', '</config>']
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/'
            '2006/relationships">\n <Relationship Target="{}" Id="rel-1" Type="http://schemas.microsoft.com/'
            '3dmanufacturing/2013/01/3dmodel"/>\n</Relationships>\n')
    files = [("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.'
              'org/package/2006/content-types">\n <Default Extension="rels" ContentType="application/vnd.openxmlformats-'
              'package.relationships+xml"/>\n <Default Extension="model" ContentType="application/vnd.ms-package.'
              '3dmanufacturing-3dmodel+xml"/>\n</Types>\n'),
             ("_rels/.rels", rels.format("/3D/3dmodel.model")),
             ("3D/_rels/3dmodel.model.rels", rels.format("/3D/Objects/object_1.model")),
             ("3D/3dmodel.model", "\n".join(mod) + "\n"),
             ("3D/Objects/object_1.model", "\n".join(obj) + "\n"),
             ("Metadata/model_settings.config", "\n".join(cfg) + "\n"),
             ("Metadata/project_settings.config", project_settings)]
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for fname, data in files:
            info = zipfile.ZipInfo(fname, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, data.encode("utf-8") if isinstance(data, str) else data)


_RESOLVE_INDEX: dict[str, dict] = {}


def resolve_profile(kind: str, name: str) -> dict:
    """A Bambu system profile with its ``inherits`` chain and ``include`` files flattened (the AMS probe's resolver)."""
    if kind not in _RESOLVE_INDEX:
        idx = {}
        d = os.path.join(PROFILES, kind)
        for fn in sorted(os.listdir(d)):
            if fn.endswith(".json"):
                try:
                    with open(os.path.join(d, fn), "r", encoding="utf-8") as f:
                        j = json.load(f)
                except (OSError, ValueError):
                    continue
                if isinstance(j, dict) and "name" in j:
                    idx[j["name"]] = j
        _RESOLVE_INDEX[kind] = idx
    d = _RESOLVE_INDEX[kind][name]
    out: dict = {}
    if d.get("inherits"):
        out.update(resolve_profile(kind, d["inherits"]))
    for inc in d.get("include", []) or []:
        if inc in _RESOLVE_INDEX[kind]:
            out.update({k: v for k, v in resolve_profile(kind, inc).items()
                        if k not in ("name", "type", "from", "instantiation", "setting_id")})
    out.update({k: v for k, v in d.items() if k not in ("inherits", "include")})
    return out


def resolved_profile_copy(kind: str, name: str, overrides: dict, tmp: str) -> str:
    """Write the RESOLVED system profile + overrides (no inherits left) and return its path."""
    prof = resolve_profile(kind, name)
    prof.update(overrides)
    dst = os.path.join(tmp, f"resolved_{kind}_{re.sub(r'[^A-Za-z0-9]+', '_', name)}.json")
    with open(dst, "w", encoding="utf-8") as f:
        json.dump(prof, f, indent=1)
    return dst


def write_bambu_multi_project(path: str, name: str, parts: list[tuple[trimesh.Trimesh, str, int]],
                              project_settings: bytes) -> None:
    """livery (2026-11-04): a Bambu Studio project with ONE object made of several normal parts, each on its own
    filament: parts = [(mesh in print coordinates, part name, extruder 1-based)].  The parts share their boundary faces
    (white body with the logo pocket + the orange inlay that fills it), so the slicer gives every region to exactly one
    filament.  Deterministic bytes (fixed zip dates, fixed number formats)."""
    obj = ['<?xml version="1.0" encoding="UTF-8"?>', f'<model unit="millimeter" xml:lang="en-US" {_NS_3MF}>',
           ' <metadata name="BambuStudio:3mfVersion">1</metadata>', ' <resources>']
    for k, (mm, _, _) in enumerate(parts, start=1):
        obj += [f'  <object id="{k}" type="model">', '   <mesh>', '    <vertices>']
        obj += [f'     <vertex x="{x:.6f}" y="{y:.6f}" z="{z:.6f}"/>' for x, y, z in np.asarray(mm.vertices)]
        obj += ['    </vertices>', '    <triangles>']
        obj += [f'     <triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in np.asarray(mm.faces)]
        obj += ['    </triangles>', '   </mesh>', '  </object>']
    obj += [' </resources>', ' <build/>', '</model>']
    top = len(parts) + 1
    mod = ['<?xml version="1.0" encoding="UTF-8"?>', f'<model unit="millimeter" xml:lang="en-US" {_NS_3MF}>',
           ' <metadata name="Application">BambuStudio-02.08.02.61</metadata>',
           ' <metadata name="BambuStudio:3mfVersion">1</metadata>', ' <resources>', f'  <object id="{top}" type="model">',
           '   <components>']
    mod += [f'    <component p:path="/3D/Objects/object_1.model" objectid="{k}" transform="{_IDENTITY_3MF}"/>'
            for k in range(1, top)]
    mod += ['   </components>', '  </object>', ' </resources>', ' <build>',
            f'  <item objectid="{top}" transform="{_IDENTITY_3MF}" printable="1"/>', ' </build>', '</model>']
    cfg = ['<?xml version="1.0" encoding="UTF-8"?>', '<config>', f'  <object id="{top}">',
           f'    <metadata key="name" value="{name}"/>', '    <metadata key="extruder" value="1"/>']
    for k, (_, pname, ext) in enumerate(parts, start=1):
        cfg += [f'    <part id="{k}" subtype="normal_part">', f'      <metadata key="name" value="{pname}"/>',
                f'      <metadata key="matrix" value="{_IDENTITY_16}"/>', f'      <metadata key="extruder" value="{ext}"/>',
                '    </part>']
    cfg += ['  </object>', '  <plate>', '    <metadata key="plater_id" value="1"/>', '    <model_instance>',
            f'      <metadata key="object_id" value="{top}"/>', '      <metadata key="instance_id" value="0"/>',
            '    </model_instance>', '  </plate>', '</config>']
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/'
            '2006/relationships">\n <Relationship Target="{}" Id="rel-1" Type="http://schemas.microsoft.com/'
            '3dmanufacturing/2013/01/3dmodel"/>\n</Relationships>\n')
    files = [("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.'
              'org/package/2006/content-types">\n <Default Extension="rels" ContentType="application/vnd.openxmlformats-'
              'package.relationships+xml"/>\n <Default Extension="model" ContentType="application/vnd.ms-package.'
              '3dmanufacturing-3dmodel+xml"/>\n</Types>\n'),
             ("_rels/.rels", rels.format("/3D/3dmodel.model")),
             ("3D/_rels/3dmodel.model.rels", rels.format("/3D/Objects/object_1.model")),
             ("3D/3dmodel.model", "\n".join(mod) + "\n"),
             ("3D/Objects/object_1.model", "\n".join(obj) + "\n"),
             ("Metadata/model_settings.config", "\n".join(cfg) + "\n"),
             ("Metadata/project_settings.config", project_settings)]
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for fname, data in files:
            info = zipfile.ZipInfo(fname, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, data.encode("utf-8") if isinstance(data, str) else data)


# livery: the 2-filament slice (white body + orange inlay); every number is read from the CLI's result / G-code / slice info
# prime tower positions tried in order (min corner, mm): the AMS probe's (20, 205) worked for a 120 mm tube; a taller
# rib-wall tower grows its footprint, and the CLI refuses a plate whose tower leaves the printable area ("Found G-code
# outside of the printable area") - the first candidate that slices is used and recorded (front corners first: the
# P2S wrap-detection zone is at the back, x 153-216 / y 235-256)
TOWER_XY_CANDIDATES = ((20.0, 205.0), (15.0, 15.0), (195.0, 15.0), (15.0, 170.0))
NON_PART_TOOL_FEATURES = ("Support", "Brim", "Skirt", "Custom", "Prime", "Wipe", "none")
DAB_PATH_MM = 0.4             # reviewfix F5: a layer whose part path of a tool is below this prints only a dab (info)
# reviewfix F1 (2026-11-04): the 2-colour print check on the sliced G-code - the orange part must be ONE pass of the
# outer line (path <= INLAY_PATH_RATIO_MAX x the single-pass length inlay outer area / layer height; the classic wall
# generator without detect_thin_wall ran every chord twice, 1.79x) and its grams within INLAY_G_REL_TOL of the solid's
# volume x density (the double pass gave +45 %)
INLAY_PATH_RATIO_MAX = 1.15
INLAY_G_REL_TOL = 0.10


def _move_length(cmd: str, x0: float, y0: float, x1: float, y1: float, body: str, axn: dict) -> float:
    """XY length of one G1 line or G2 (clockwise) / G3 (counter-clockwise) arc whose centre is (x0 + I, y0 + J)."""
    if cmd in ("G2", "G3"):
        mi, mj = axn["I"].search(body), axn["J"].search(body)
        if mi or mj:
            cx = x0 + (float(mi.group(1)) if mi else 0.0)
            cy = y0 + (float(mj.group(1)) if mj else 0.0)
            r = math.hypot(x0 - cx, y0 - cy)
            a0, a1 = math.atan2(y0 - cy, x0 - cx), math.atan2(y1 - cy, x1 - cx)
            sweep = ((a1 - a0) if cmd == "G3" else (a0 - a1)) % (2 * math.pi)
            if sweep < 1e-12:
                sweep = 2 * math.pi
            return r * sweep
    return math.hypot(x1 - x0, y1 - y0)


def inlay_print_check(two: dict, inlay_volume_mm3: float, inlay_depth_mm: float, layer_h_mm: float, rho: float,
                      inlay_tool: str = "1") -> dict:
    """reviewfix F1: is the orange inlay printed as ONE outer-line pass of the drawn logo?  (M, from the 2-colour G-code)
    single-pass length = (inlay volume / depth) / layer height = the inlay's outer area / layer height (the per-layer
    chord sum); PASS needs path / single pass <= INLAY_PATH_RATIO_MAX and |orange part g / (V x rho) - 1| <=
    INLAY_G_REL_TOL.  The per-layer counts (empty swaps, dabs) are information, not part of the verdict."""
    path = float((two.get("part_path_mm_by_tool") or {}).get(inlay_tool, math.nan))
    g = float((two.get("part_g_by_tool") or {}).get(inlay_tool, math.nan))
    single = inlay_volume_mm3 / inlay_depth_mm / layer_h_mm
    solid_g = inlay_volume_mm3 * rho / 1000.0
    ratio, g_rel = path / single, g / solid_g - 1.0
    ok = (math.isfinite(ratio) and ratio <= INLAY_PATH_RATIO_MAX and math.isfinite(g_rel)
          and abs(g_rel) <= INLAY_G_REL_TOL)
    return {"status": "PASS" if ok else "FAIL", "orange_part_path_mm": path, "single_pass_path_mm": single,
            "path_ratio": ratio, "path_ratio_max": INLAY_PATH_RATIO_MAX, "orange_part_g": g, "solid_g": solid_g,
            "g_rel": g_rel, "g_rel_tol": INLAY_G_REL_TOL, "layer_height_mm": layer_h_mm,
            "inlay_depth_mm": inlay_depth_mm, "inlay_volume_mm3": inlay_volume_mm3,
            "per_layer_info": (two.get("part_layers_by_tool") or {}).get(inlay_tool)}


def _run_cli_watch(cmd: list[str], cwd: str, result_json: str, timeout_s: float = 3600.0) -> int | None:
    """Run the CLI; kill it 30 s after it wrote result.json (the multi-filament CLI sometimes does not exit, AMS probe)."""
    p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0, seen = time.time(), None
    while True:
        rc = p.poll()
        if rc is not None:
            return rc
        now = time.time()
        if seen is None and os.path.isfile(result_json):
            seen = now
        if (seen is not None and now - seen > 30.0) or now - t0 > timeout_s:
            p.kill()
            p.wait()
            return None
        time.sleep(1.0)


def parse_gcode_tools(path: str, rho: float) -> dict:
    """Per-filament G-code accounting (relative E): part grams per tool (features minus support / brim / skirt /
    custom / prime tower and minus the purge blocks), prime-tower grams per tool, purge grams per tool (``; FLUSH_START``
    .. ``; FLUSH_END`` extrusion, and the P2S's virtual ``; VFLUSH_START`` .. ``; VFLUSH_END`` blocks whose ``;VG1 E..``
    comment lines state the purge the AMS / firmware does; attributed to the filament being loaded), tool changes after the first layer started (count, z
    range, per direction), feature grams per tool, and the E-weighted part centroid height per tool (layer mid-plane).
    reviewfix (2026-11-04, review F1 / F5): also the extruding XY path length of the part per tool (G1 lines and G2 / G3
    arcs from I / J), and per tool the layers with part extrusion, the changes INTO the tool on a layer where it extrudes
    no part ('empty' swaps) and the layers whose part path is shorter than 0.4 mm ('dabs')."""
    a = math.pi / 4 * FILAMENT_D_MM ** 2
    feat, tool, z, rel, lh = "none", None, 0.0, True, 0.2
    started, in_flush = False, False
    by: dict[tuple[str, int], float] = {}
    flush: dict[int, float] = {}
    part_ez: dict[int, list[float]] = {}
    changes: list[dict] = []
    path_by: dict[int, float] = {}                      # part path length per tool (mm)
    layer_part: dict[tuple[int, int], float] = {}       # (layer, tool) -> part path (mm)
    layer_z: dict[int, float] = {}
    layer = -1
    x = y = 0.0
    absolute_xy = True
    num, znum = re.compile(r"E(-?[0-9.]+)"), re.compile(r"Z(-?[0-9.]+)")
    axn = {k: re.compile(k + r"(-?[0-9.]+)") for k in "XYIJ"}
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if s.startswith(";"):
                if s.startswith("; FEATURE:"):
                    feat = s.split(":", 1)[1].strip()
                elif s.startswith("; CHANGE_LAYER"):
                    started = True
                    layer += 1
                elif s.startswith("; Z_HEIGHT:"):
                    try:
                        z = float(s.split(":", 1)[1])
                        layer_z.setdefault(layer, z)
                    except ValueError:
                        pass
                elif s.startswith(("; FLUSH_START", "; VFLUSH_START")):
                    in_flush = True
                elif s.startswith(("; FLUSH_END", "; VFLUSH_END")):
                    in_flush = False
                elif in_flush and s.startswith(";VG1 "):
                    # P2S: the purge is done by the AMS / firmware (M620.10); the G-code carries it only as these
                    # commented ';VG1 E..' lines of the virtual flush block (the filament being loaded)
                    mv_ = num.search(s[1:].split(";", 1)[0])
                    if mv_ and float(mv_.group(1)) > 0:
                        t_ = tool if tool is not None else 0
                        flush[t_] = flush.get(t_, 0.0) + float(mv_.group(1))
                else:
                    m = re.match(r"; layer_height = ([0-9.]+)", s)
                    if m:
                        lh = float(m.group(1))
                continue
            if s.startswith("M82"):
                rel = False
            elif s.startswith("M83"):
                rel = True
            elif s.startswith("G90") and not s.startswith("G90."):
                absolute_xy = True
            elif s.startswith("G91") and not s.startswith("G91."):
                absolute_xy = False
            m = re.match(r"T(\d+)\s*$", s)
            if m:
                t = int(m.group(1))
                if t < 16:
                    if started and tool is not None and t != tool:
                        changes.append({"z": z, "from": tool, "to": t, "layer": layer})
                    tool = t
                continue
            if not s.startswith(("G0 ", "G1 ", "G2 ", "G3 ")):
                continue
            body = s.split(";", 1)[0]
            mx, my = axn["X"].search(body), axn["Y"].search(body)
            x0, y0 = x, y
            if absolute_xy:
                x = float(mx.group(1)) if mx else x
                y = float(my.group(1)) if my else y
            else:
                x += float(mx.group(1)) if mx else 0.0
                y += float(my.group(1)) if my else 0.0
            if s.startswith("G0 "):
                continue
            me = num.search(body)
            if not me:
                continue
            e = float(me.group(1))
            if e <= 0 or not rel:
                continue
            t = tool if tool is not None else 0
            if in_flush:
                flush[t] = flush.get(t, 0.0) + e
                continue
            if "X" not in body and "Y" not in body:
                key = ("_e_only", t)
            else:
                key = (feat, t)
            by[key] = by.get(key, 0.0) + e
            if key[0] != "_e_only" and not any(x_ in key[0] for x_ in NON_PART_TOOL_FEATURES):
                acc = part_ez.setdefault(t, [0.0, 0.0])
                acc[0] += e
                acc[1] += e * z
                seg = _move_length(s[:2], x0, y0, x, y, body, axn)
                path_by[t] = path_by.get(t, 0.0) + seg
                layer_part[(layer, t)] = layer_part.get((layer, t), 0.0) + seg
    g = lambda mm: mm * a * rho / 1000.0        # noqa: E731
    tools = sorted({t for (_, t) in by} | set(flush))
    part_g = {t: sum(g(mm) for (ft, tt), mm in by.items() if tt == t and ft != "_e_only"
                     and not any(x in ft for x in NON_PART_TOOL_FEATURES)) for t in tools}
    tower_g = {t: sum(g(mm) for (ft, tt), mm in by.items() if tt == t and "Prime tower" in ft) for t in tools}
    other_g = {t: sum(g(mm) for (ft, tt), mm in by.items() if tt == t) - part_g[t] - tower_g[t] for t in tools}
    zs = [c["z"] for c in changes]
    per_tool_layers = {}
    for t in tools:
        lays = {L_: p for (L_, tt), p in layer_part.items() if tt == t and p > 0}
        # 'empty' swaps: a change into or out of the tool on a layer where it prints no part - a swap bought for
        # nothing.  Bambu alternates the tool order per layer (one change per orange layer), so after a run of an ODD
        # number of orange layers the next layer starts on the orange and changes back (parity), and a layer whose
        # orange is dropped (a near-empty island end) still gets its change
        empty = sorted({c["layer"] for c in changes if t in (c["from"], c["to"]) and c["layer"] not in lays})
        dabs = sorted((round(layer_z.get(L_, math.nan), 3), round(p, 3)) for L_, p in lays.items() if p < DAB_PATH_MM)
        per_tool_layers[str(t)] = {"layers_with_part": len(lays), "changes_in": sum(1 for c in changes if c["to"] == t),
                                   "empty_layers": len(empty),
                                   "empty_layer_z_mm": [round(layer_z.get(L_, math.nan), 3) for L_ in empty],
                                   f"dab_layers_path_lt_{DAB_PATH_MM:g}_mm": [list(d) for d in dabs]}
    return {"part_g_by_tool": {str(t): part_g[t] for t in tools},
            "part_path_mm_by_tool": {str(t): path_by.get(t, 0.0) for t in tools},
            "part_layers_by_tool": per_tool_layers,
            "prime_tower_g_by_tool": {str(t): tower_g[t] for t in tools},
            "purge_g_by_tool": {str(t): g(flush.get(t, 0.0)) for t in tools},
            "other_g_by_tool": {str(t): other_g[t] for t in tools},
            "feature_g_by_tool": {f"{ft}|T{tt}": g(mm) for (ft, tt), mm in sorted(by.items())},
            "part_z_centroid_plate_mm_by_tool": {str(t): (v[1] / v[0] - lh / 2) for t, v in part_ez.items() if v[0] > 0},
            "changes": len(changes), "change_z_range_mm": [min(zs), max(zs)] if zs else None,
            "changes_by_direction": {f"T{a_}->T{b_}": sum(1 for c in changes if (c["from"], c["to"]) == (a_, b_))
                                     for a_, b_ in sorted({(c["from"], c["to"]) for c in changes})}}


def slice_multi_project(project: str, tmp: str, tag: str) -> dict:
    """Slice a Bambu project (settings inside it, no --load-settings) and read the plate result, the slice info's
    per-filament usage and the project settings the slicer used."""
    work = os.path.join(tmp, tag)
    out = os.path.join(work, "out")
    os.makedirs(out, exist_ok=True)
    cmd = [BAMBU, "--slice", "0", "--arrange", "0", "--orient", "0", "--outputdir", out, "--export-3mf", "sliced.3mf",
           project]
    rp = os.path.join(out, "result.json")
    t0 = time.time()
    rc = _run_cli_watch(cmd, work, rp)
    if not os.path.isfile(rp):
        return {"ok": False, "error": f"no result.json (exit {rc})"}
    with open(rp, "r", encoding="utf-8") as f:
        res = json.load(f)
    if res.get("return_code", -1) != 0:
        return {"ok": False, "error": res.get("error_string", "slice failed")}
    sliced = os.path.join(out, "sliced.3mf")
    with zipfile.ZipFile(sliced) as z:
        info = z.read("Metadata/slice_info.config").decode("utf-8", "replace")
        ps = json.loads(z.read("Metadata/project_settings.config").decode("utf-8"))
    used = {m.group(1): {"used_m": float(m.group(2)), "used_g": float(m.group(3))}
            for m in re.finditer(r'<filament id="(\d+)"[^>]*used_m="([0-9.]+)"[^>]*used_g="([0-9.]+)"', info)}
    plate = res["sliced_plates"][0]
    return {"ok": True, "main_predication_s": float(plate.get("main_predication", 0.0)),
            "total_predication_s": float(plate.get("total_predication", 0.0)),
            "filament_change_times": plate.get("filament_change_times"),
            "feature_type_times_s": plate.get("feature_type_times"), "slice_info_used": used,
            "flush_volumes_matrix": ps.get("flush_volumes_matrix"), "flush_multiplier": ps.get("flush_multiplier"),
            "enable_prime_tower": ps.get("enable_prime_tower"), "filament_colour": ps.get("filament_colour"),
            "wall_generator": ps.get("wall_generator"), "wall_loops": ps.get("wall_loops"),
            "detect_thin_wall": ps.get("detect_thin_wall"),
            "cli_wall_s": round(time.time() - t0, 1), "_gcode": os.path.join(out, "plate_1.gcode"), "_project": sliced}


def slice_two_colour(white: trimesh.Trimesh, inlay: trimesh.Trimesh, names: tuple[str, str], tmp: str, rho: float,
                     extra: dict, colours: tuple[str, str], flush_mm3: tuple[float, float],
                     cli_default_flush_mm3: float = 280.0, inlay_depth_mm: float | None = None) -> dict:
    """livery (2026-11-04): slice the avionics module as ONE object of two parts on two filaments (1 = the white body,
    2 = the orange inlay), print coordinates (both meshes already oriented by the same matrix).  The profiles are the
    P2S / 0.20 mm Standard / PLA Matte SYSTEM profiles fully RESOLVED (``resolved_profile_copy``: inherits / include
    flattened - the CLI does not resolve them for profiles given as file paths: with ``_profile_copy`` it slices a 200 x
    200 mm bed, arachne walls, 0.40 mm lines and a generic start / filament-change G-code, and a 2-filament plate gets no
    real filament changes; measured 2026-11-04, AMS probe) + the CanSat process values + ``extra`` (the part class) + two
    copies of the filament; the project settings are then the CLI's own export of that (the white body sliced alone),
    plus: filament colours, the flush matrix [[0, w->o], [o->w, 0]] (mm^3), multiplier 1, the prime tower ON at the first
    of TOWER_XY_CANDIDATES that slices (refusals recorded).  So this project is the print the user opens in Bambu
    Studio with the P2S profile.  Three slices
    (all M): the chosen flush pair, the CLI default flush both ways (comparison) and a single-colour control (both parts
    on filament 1, same geometry).  Returns the plan entry; private keys '_project' (the sliced 2-colour project).
    reviewfix F1 (2026-11-04, independent review): ``detect_thin_wall`` = 1.  The 0.42 mm wide orange region is ONE line
    wide; the classic wall generator without thin-wall detection made it a collapsed outer-wall loop and ran every chord
    twice (out and back, 14,565 mm of path for an 8,109 mm logo, orange 1.29 g for a 0.89 g solid: the slot overfilled
    ~1.6x, a raised ridge on a 'flush' inlay).  With it the orange is one pass (8,145 mm, 0.90 g) and the white body is
    unchanged to 0.001 g (review fix_try, M).  Arachne is not a fix (orange under-filled 0.72 g, white walls change).
    With ``inlay_depth_mm`` the entry carries ``inlay_print_check`` (one pass, grams vs the solid)."""
    work = tempfile.mkdtemp(dir=tmp)
    proc = resolved_profile_copy("process", PROCESS, {
        "name": "CanSat 2-colour", "wall_loops": "5", "top_shell_layers": "5", "bottom_shell_layers": "5",
        "sparse_infill_density": "15%", "sparse_infill_pattern": "gyroid", "enable_support": "0",
        "support_type": BASE_SUPPORT_TYPE, "support_threshold_angle": "30", "brim_type": "auto_brim", **(extra or {}),
        "detect_thin_wall": "1"}, work)
    fil = resolved_profile_copy("filament", FILAMENT, {"name": "PLA Matte rho", "filament_density": [f"{rho:.2f}"]}, work)
    machine = resolved_profile_copy("machine", MACHINE, {}, work)
    stl = os.path.join(work, "base.stl")
    white.export(stl)
    out = os.path.join(work, "base_out")
    os.makedirs(out)
    cmd = [BAMBU, "--slice", "0", "--arrange", "0", "--orient", "0", "--load-settings", f"{machine};{proc}",
           "--load-filaments", f"{fil};{fil}", "--outputdir", out, "--export-3mf", "base.3mf", stl]
    rc = _run_cli_watch(cmd, work, os.path.join(out, "result.json"), 1800.0)
    base3 = os.path.join(out, "base.3mf")
    if not os.path.isfile(base3):
        return {"ok": False, "error": f"settings export failed (exit {rc})"}
    settings = json.loads(project_settings_of(base3).decode("utf-8"))
    if settings.get("printable_area") != ["0x0", "256x0", "256x256", "0x256"]:
        return {"ok": False, "error": f"the resolved P2S profile gave the bed {settings.get('printable_area')}"}
    for k, v in list(settings.items()):     # the CLI may keep one entry for both identical filaments: make it two
        if k.startswith("filament_") and isinstance(v, list) and len(v) == 1:
            settings[k] = [v[0], v[0]]
    settings["filament_colour"] = list(colours)
    settings["flush_multiplier"] = ["1"]
    settings["enable_prime_tower"] = "1"
    settings["detect_thin_wall"] = "1"          # reviewfix F1: one orange pass (see the docstring)
    runs = {}
    tower_refused = []

    def run(tag: str, matrix, ext2: int, xy) -> dict:
        s = dict(settings)
        s["flush_volumes_matrix"] = [f"{x:g}" for x in matrix]
        s["wipe_tower_x"] = [f"{xy[0]:g}"]
        s["wipe_tower_y"] = [f"{xy[1]:g}"]
        proj = os.path.join(work, f"{tag}.3mf")
        write_bambu_multi_project(proj, names[0], [(white, names[0], 1), (inlay, names[1], ext2)],
                                  json.dumps(s, indent=4).encode("utf-8"))
        r = slice_multi_project(proj, work, tag)
        if r.get("ok"):
            r.update(parse_gcode_tools(r["_gcode"], rho))
        r["tower_xy"] = list(xy)
        return r
    tower = None
    for k, xy in enumerate(TOWER_XY_CANDIDATES):
        r = run(f"two_colour_t{k}", (0, flush_mm3[0], flush_mm3[1], 0), 2, xy)
        if r.get("ok"):
            tower, runs["two_colour"] = xy, r
            break
        tower_refused.append({"tower_xy": list(xy), "error": r.get("error")})
    if tower is None:
        return {"ok": False, "error": "no prime-tower position sliced", "tower_refused": tower_refused}
    runs["two_colour_cli_default_flush"] = run("two_colour_cli_default_flush",
                                               (0, cli_default_flush_mm3, cli_default_flush_mm3, 0), 2, tower)
    runs["single_colour_control"] = run("single_colour_control", (0, flush_mm3[0], flush_mm3[1], 0), 1, tower)
    two, dflt, ctrl = runs["two_colour"], runs["two_colour_cli_default_flush"], runs["single_colour_control"]
    if not (two.get("ok") and ctrl.get("ok")):
        return {"ok": False, "error": two.get("error") or ctrl.get("error"),
                "runs": {k: _public(v) for k, v in runs.items()}}
    prep_s = ctrl["total_predication_s"] - ctrl["main_predication_s"]      # start-up (heating, calibration), M
    for r in (two, dflt, ctrl):
        if r.get("ok"):
            r["time_s"] = r["total_predication_s"] - prep_s      # comparable with the other parts' main_predication
    used = two["slice_info_used"]
    entry = {"ok": True, "two_colour": True, "time_s": two["time_s"],
             "filament_g": sum(v["used_g"] for v in used.values()),
             "filament_g_by_colour": {colours[int(k) - 1]: v["used_g"] for k, v in used.items()},
             "part_g_by_tool": two["part_g_by_tool"], "purge_g_by_tool": two["purge_g_by_tool"],
             "prime_tower_g_by_tool": two["prime_tower_g_by_tool"], "swaps": two["changes"],
             "swap_z_range_plate_mm": two["change_z_range_mm"], "swaps_by_direction": two["changes_by_direction"],
             "flush_volumes_mm3": {"white_to_orange": flush_mm3[0], "orange_to_white": flush_mm3[1]},
             "orange_features_g": {k: v for k, v in two["feature_g_by_tool"].items() if k.endswith("|T1")},
             "prep_time_s": prep_s, "prime_tower_xy_mm": list(tower), "prime_tower_refused": tower_refused,
             "settings_in_project": {k: two.get(k) for k in (
                 "flush_volumes_matrix", "flush_multiplier", "enable_prime_tower", "filament_colour", "wall_generator",
                 "wall_loops", "detect_thin_wall")},
             "part_path_mm_by_tool": two.get("part_path_mm_by_tool"), "part_layers_by_tool": two.get("part_layers_by_tool"),
             "runs": {k: _public(v) for k, v in runs.items()}, "_project": two["_project"], "_gcode": two["_gcode"]}
    if inlay_depth_mm:
        lh = float(str(settings.get("layer_height", "0.2")).strip('[]"\''))
        entry["inlay_print_check"] = inlay_print_check(two, float(inlay.volume), float(inlay_depth_mm), lh, rho)
    entry["added_vs_single_colour"] = {
        k: {"time_s": r["time_s"] - ctrl["time_s"],
            "purge_g": sum(r["purge_g_by_tool"].values()), "prime_tower_g": sum(r["prime_tower_g_by_tool"].values()),
            "filament_g": sum(v["used_g"] for v in r["slice_info_used"].values())
                          - sum(v["used_g"] for v in ctrl["slice_info_used"].values()),
            "swaps": r["changes"]}
        for k, r in (("flush_" + "_".join(f"{x:g}" for x in flush_mm3), two),
                     (f"flush_{cli_default_flush_mm3:g}_cli_default", dflt)) if r.get("ok")}
    entry["single_colour_control"] = {"time_s": ctrl["time_s"], "filament_g": sum(v["used_g"] for v in
                                                                                 ctrl["slice_info_used"].values()),
                                      "part_g": sum(ctrl["part_g_by_tool"].values())}
    return entry


def project_settings_of(project_3mf: str) -> bytes:
    """Metadata/project_settings.config of a project Bambu Studio exported (its fully resolved process / filament /
    machine settings)."""
    with zipfile.ZipFile(project_3mf) as z:
        return z.read("Metadata/project_settings.config")


def gcode_extrusion_samples(gcode: str, keep=None) -> tuple[dict[str, np.ndarray], dict[str, str]]:
    """Points along every extruding move (G1 / G2 / G3 with E > 0) of a Bambu Studio G-code, per feature
    ('; FEATURE:' blocks), and the '; key = value' settings of its config block.

    Each move is sampled at the centres of n = ceil(length / SAMPLE_STEP_MM) equal pieces (arcs along the arc, from
    I / J); a sample is (x, y, z_mid, piece length) with z_mid = '; Z_HEIGHT:' - '; LAYER_HEIGHT:' / 2.
    ``keep(feature, z_mid)`` (default: everything) selects the moves that are collected."""
    feat, z, lh, x, y = None, 0.0, 0.2, 0.0, 0.0
    out: dict[str, list] = {}
    settings: dict[str, str] = {}
    in_cfg = False
    with open(gcode, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            c0 = line[0] if line else ""
            if c0 == ";":
                if line.startswith("; FEATURE:"):
                    feat = line.split(":", 1)[1].strip()
                elif line.startswith("; Z_HEIGHT:"):
                    z = float(line.split(":", 1)[1])
                elif line.startswith("; LAYER_HEIGHT:"):
                    lh = float(line.split(":", 1)[1])
                elif line.startswith("; CONFIG_BLOCK_START"):
                    in_cfg = True
                elif line.startswith("; CONFIG_BLOCK_END"):
                    in_cfg = False
                elif in_cfg and " = " in line:
                    k, v = line[2:].split(" = ", 1)
                    settings[k.strip()] = v.strip()
                continue
            if c0 != "G" or line[:3] not in ("G1 ", "G2 ", "G3 "):
                continue
            code = line.split(";", 1)[0]
            mx, my, me = _RE_X.search(code), _RE_Y.search(code), _RE_E.search(code)
            nx = float(mx.group(1)) if mx else x
            ny = float(my.group(1)) if my else y
            if me and float(me.group(1)) > 0.0 and feat and (nx != x or ny != y):
                zm = z - 0.5 * lh
                if keep is None or keep(feat, zm):
                    if line[1] == "1":
                        seg = math.hypot(nx - x, ny - y)
                        n = max(1, int(math.ceil(seg / SAMPLE_STEP_MM)))
                        t = (np.arange(n) + 0.5) / n
                        px, py = x + (nx - x) * t, y + (ny - y) * t
                    else:
                        mi, mj = _RE_I.search(code), _RE_J.search(code)
                        cx = x + (float(mi.group(1)) if mi else 0.0)
                        cy = y + (float(mj.group(1)) if mj else 0.0)
                        r = math.hypot(x - cx, y - cy)
                        a0, a1 = math.atan2(y - cy, x - cx), math.atan2(ny - cy, nx - cx)
                        sweep = (a1 - a0) % (2 * math.pi) if line[1] == "3" else -((a0 - a1) % (2 * math.pi))
                        seg = abs(sweep) * r
                        n = max(1, int(math.ceil(seg / SAMPLE_STEP_MM)))
                        a = a0 + sweep * (np.arange(n) + 0.5) / n
                        px, py = cx + r * np.cos(a), cy + r * np.sin(a)
                    out.setdefault(feat, []).append(np.column_stack(
                        [px, py, np.full(n, zm), np.full(n, seg / n)]))
            x, y = nx, ny
    return {k: np.vstack(v) for k, v in out.items()}, settings


def _circle_fit(xy: np.ndarray) -> tuple[float, float, float]:
    """Algebraic (Kasa) least-squares circle through the points: centre x, y and radius."""
    A = np.c_[2 * xy[:, 0], 2 * xy[:, 1], np.ones(len(xy))]
    b = xy[:, 0] ** 2 + xy[:, 1] ** 2
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    return float(cx), float(cy), float(math.sqrt(max(c + cx * cx + cy * cy, 0.0)))


def _matrix_3mf(s: str) -> np.ndarray:
    """4 x 4 (column vectors) from a 3MF transform attribute 'm00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32'
    (row-vector convention: p' = p M)."""
    v = [float(t) for t in s.split()]
    if len(v) != 12:
        raise ValueError(f"3MF transform {s!r} does not have 12 numbers")
    M = np.eye(4)
    M[:3, :3] = np.array(v[:9]).reshape(3, 3).T
    M[:3, 3] = v[9:]
    return M


def plate_transform(project_3mf: str, stl_bounds: np.ndarray) -> tuple[np.ndarray, dict]:
    """Sliced-model -> plate (G-code) transform Bambu Studio used, from the project it exported with the G-code.  On
    loading it recentres the mesh (object vertices = model - the part's source_offset in model_settings.config) and
    places it by component then build-item transform; the item carries its arrange, which may rotate the part about Z
    (measured: 225 deg for the flipped intake STL, 315 deg for the same mesh as a 3MF).  Returns
    item @ component @ translate(-source_offset) after checking that the object vertices + source_offset have the
    sliced model's bounds (``stl_bounds``) within 0.01 mm.  Raises ValueError otherwise."""
    with zipfile.ZipFile(project_3mf) as z:
        model = z.read("3D/3dmodel.model").decode("utf-8", "replace")
        cfg = z.read("Metadata/model_settings.config").decode("utf-8", "replace")
        items = re.findall(r"<item\b[^>]*\btransform=\"([^\"]+)\"", model)
        normal = re.findall(r"<part id=\"(\d+)\" subtype=\"normal_part\"[^>]*>(.*?)</part>", cfg, re.S)
        if len(items) != 1 or len(normal) != 1:
            raise ValueError(f"{len(items)} build items / {len(normal)} normal parts in {project_3mf} (expected 1 / 1)")
        pid, pcfg = normal[0]
        comps = [c for c in re.findall(r"<component\b[^>]*>", model) if re.search(rf"\bobjectid=\"{pid}\"", c)]
        if len(comps) != 1:
            raise ValueError(f"{len(comps)} components for the normal part {pid} in {project_3mf} (expected 1)")
        path = re.search(r"\bp:path=\"/?([^\"]+)\"", comps[0])
        tr = re.search(r"\btransform=\"([^\"]+)\"", comps[0])
        if path is None or tr is None:
            raise ValueError(f"component of part {pid} without path / transform in {project_3mf}")
        obj = z.read(path.group(1)).decode("utf-8", "replace")
    so = [re.findall(rf"<metadata key=\"source_offset_{a}\" value=\"([^\"]+)\"", pcfg) for a in "xyz"]
    if any(len(s) != 1 for s in so):
        raise ValueError(f"no source_offset for the normal part in {project_3mf} model_settings.config")
    off = np.array([float(s[0]) for s in so])
    m = re.search(rf"<object id=\"{pid}\"[^>]*>(.*?)</object>", obj, re.S)
    if m is None:
        raise ValueError(f"no object {pid} in {path.group(1)} of {project_3mf}")
    V = np.array(re.findall(r"<vertex x=\"([^\"]+)\" y=\"([^\"]+)\" z=\"([^\"]+)\"", m.group(1)), dtype=float)
    if not len(V):
        raise ValueError(f"no vertices in object {pid} of {project_3mf}")
    placed = V + off
    dev = float(np.abs(np.vstack([placed.min(axis=0), placed.max(axis=0)]) - np.asarray(stl_bounds)).max())
    if dev > 0.01:
        raise ValueError(f"the project's mesh + source_offset differs from the sliced model by {dev:.3f} mm (bounds)")
    item = _matrix_3mf(items[0])
    M = item @ _matrix_3mf(tr.group(1)) @ trimesh.transformations.translation_matrix(-off)
    rot = math.degrees(math.atan2(M[1, 0], M[0, 0])) % 360.0
    return M, {"arrange_rotation_z_deg": round(rot, 3), "model_to_plate_shift": [round(float(v), 3) for v in M[:3, 3]],
               "mesh_vs_model_bounds_mm": round(dev, 4)}


def _setting_mm(settings: dict[str, str], *keys: str) -> tuple[float | None, str | None]:
    """The first positive number among the G-code config ``keys`` and its key ((None, None) if none is): Bambu writes
    0 for a line width that falls back to line_width."""
    for k in keys:
        try:
            v = float(str(settings.get(k, "")).strip())
        except ValueError:
            continue
        if v > 0.0:
            return v, k
    return None, None


def window_support_check(gcode: str, project_3mf: str, L, mesh: trimesh.Trimesh,
                         rots: list[tuple[str, float]]) -> dict:
    """v0.6.2: support extrusion inside the intake's window openings in a sliced G-code (Bambu Studio), in the design
    frame.  ``mesh`` is the intake in vehicle coordinates and ``rots`` the orientation it was sliced in.

    Frame: design -> sliced STL is ``orient_matrix(mesh, rots)``, STL -> G-code the slicer's own build-item transform
    (``plate_transform``, from the project exported beside the G-code).  Registration evidence (no PASS without it):
    (1) the outer perimeter of the solid band above the windows (Z [z_window_top, z_intake_bulkhead_bottom] inside
    RING_LAYER_MARGIN_MM) is a circle about the predicted axis (within RING_FIT_TOL_MM) of radius body_r - line_w / 2
    (within RING_FIT_TOL_MM, rms <= RING_FIT_RMS_MAX_MM); (2) at mid-window height (above the gables) the outer-wall
    centre line of every strut side face lies line_w / 2 inside that face (median within STRUT_FACE_TOL_MM over at
    least STRUT_FACE_MIN_SAMPLES samples: this fixes the turn about the axis the ring cannot see), no wall centre line
    lies in a window opening and every strut has wall extrusion.

    Probe (the constants above say why): per window, the sector up to the radial strut faces x r [r_band_in, body_r] x
    Z [z_window_bottom, z_window_top], grown by half the support line width across the strut faces (perpendicular
    distance to the face planes) and the band faces, and by half the layer height in Z: every support sample whose bead
    can overlap the opening, including support hugging a strut.  The gussets below the gable are solid, so nothing
    prints there.  Support = every feature named Support*.  Also reported: the nearest support sample outside every
    probe (its bead gap to the probe and on which side), and the Bridge / Overhang wall extrusion of the layer that
    spans each ceiling (in the 1 mm above it).  status: PASS (registered, no support in any opening), FAIL (support in
    an opening), NOT_VERIFIED (no G-code / project, no line width / support line width / layer height in its config,
    axis not vertical in print, or registration failed)."""
    res: dict = {"status": "NOT_VERIFIED", "gcode_check": "support extrusion inside the window openings",
                 "probe": f"window sectors up to the strut faces, r {L.r_band_in:.2f}..{L.body_r:.2f}, "
                          f"Z {L.z_window_bottom:.2f}..{L.z_window_top:.2f} (design frame)"}
    if not gcode or not os.path.isfile(gcode) or not project_3mf or not os.path.isfile(project_3mf):
        res["reason"] = "no G-code or no exported project"
        return res
    T = orient_matrix(mesh, rots)
    Vs = trimesh.transform_points(mesh.vertices, T)        # the sliced STL
    try:
        P, plate = plate_transform(project_3mf, np.vstack([Vs.min(axis=0), Vs.max(axis=0)]))
    except (ValueError, KeyError, OSError) as e:
        res["reason"] = f"slicer placement unknown: {e}"
        return res
    G = P @ T                                            # design -> G-code
    if abs(abs(G[2, 2]) - 1.0) > 1e-9:
        res["reason"] = f"the intake axis is not vertical in the print orientation {rots}: check not implemented"
        return res
    Ginv = np.linalg.inv(G)

    def zrange(z0: float, z1: float) -> tuple[float, float]:
        a, b = G[2, 2] * z0 + G[2, 3], G[2, 2] * z1 + G[2, 3]
        return min(a, b), max(a, b)

    ring = zrange(L.z_window_top + RING_LAYER_MARGIN_MM, L.z_intake_bulkhead_bottom - RING_LAYER_MARGIN_MM)
    mid = zrange(L.z_gable_top + STRUT_CHECK_MARGIN_MM, L.z_window_top - STRUT_CHECK_MARGIN_MM)
    win = zrange(L.z_window_bottom - 1.0, L.z_window_top + 1.0)
    if mid[0] >= mid[1] or ring[0] >= ring[1]:
        res["reason"] = "no strut-wall or ring layers to register on"
        return res

    def keep(feat: str, z: float) -> bool:
        if feat.startswith(SUPPORT_FEATURE_PREFIX) or feat in CEILING_FEATURES:
            return True
        return feat in WALL_FEATURES and (ring[0] <= z <= ring[1] or mid[0] <= z <= mid[1])

    pts, settings = gcode_extrusion_samples(gcode, keep)
    empty = np.empty((0, 4))
    lw, _ = _setting_mm(settings, "outer_wall_line_width", "line_width")
    lw_s, lw_s_key = _setting_mm(settings, "support_line_width", "line_width")
    lh, _ = _setting_mm(settings, "layer_height")
    xy_gap, _ = _setting_mm(settings, "support_object_xy_distance")
    if lw is None or lw_s is None or lh is None:
        res["reason"] = (f"the G-code config gives no outer wall line width ({lw}), support line width ({lw_s}) or layer "
                         f"height ({lh}): the probe tolerances are unknown")
        return res
    grow, grow_z = 0.5 * lw_s, 0.5 * lh
    res["probe"] += f", grown by {grow:g} mm (half the support line width) across the faces and {grow_z:g} mm in Z"
    res["probe_tolerances"] = {
        "support_line_width_mm": lw_s, "support_line_width_from": lw_s_key, "grow_mm": grow, "grow_z_mm": grow_z,
        "support_object_xy_distance_mm": xy_gap,
        # where the slicer puts the centre line of support hugging a strut face (inside the opening, never cut away)
        "hugging_support_centre_inside_face_mm": None if xy_gap is None else round(xy_gap + grow, 3)}
    # (1) ring circle about the predicted axis
    ax, ay = float(G[0, 3]), float(G[1, 3])              # the design axis (0, 0, z) in G-code XY
    ow = pts.get("Outer wall", empty)
    ow = ow[(ow[:, 2] >= ring[0]) & (ow[:, 2] <= ring[1])]
    ow = ow[np.hypot(ow[:, 0] - ax, ow[:, 1] - ay) > 0.5 * (L.r_band_in + L.body_r)]   # the ring's outer contour
    reg: dict = {"slicer_placement": plate, "axis_predicted_xy": [round(ax, 3), round(ay, 3)], "ring_samples": int(len(ow))}
    res["registration"] = reg
    problems = []
    r_expect = L.body_r - lw / 2
    if len(ow) < 50:
        problems.append(f"only {len(ow)} outer-wall samples on the ring above the windows")
    else:
        cx, cy, R = _circle_fit(ow[:, :2])
        rms = float(np.sqrt(np.mean((np.hypot(ow[:, 0] - cx, ow[:, 1] - cy) - R) ** 2)))
        off = math.hypot(cx - ax, cy - ay)
        reg.update({"ring_fit_centre_xy": [round(cx, 3), round(cy, 3)], "ring_fit_centre_offset": round(off, 3),
                    "ring_outer_perimeter_r": round(R, 3), "expected_r": round(r_expect, 3), "ring_fit_rms": round(rms, 4)})
        if off > RING_FIT_TOL_MM or abs(R - r_expect) > RING_FIT_TOL_MM or rms > RING_FIT_RMS_MAX_MM:
            problems.append(f"ring outer perimeter: centre {off:.3f} mm off the predicted axis, r {R:.3f} vs "
                            f"{r_expect:.3f} (tol {RING_FIT_TOL_MM:g}), rms {rms:.3f}")

    def polar(Q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        D = (Ginv @ np.c_[Q[:, :3], np.ones(len(Q))].T).T
        return np.hypot(D[:, 0], D[:, 1]), np.degrees(np.arctan2(D[:, 1], D[:, 0])), D[:, 2]

    half = L.window_half_angle_deg

    def face_distance(r: np.ndarray, th: np.ndarray, theta_w: float, side: int = 0) -> np.ndarray:
        """Perpendicular distance (mm) into window ``theta_w`` from the plane of its strut face at theta_w + side x
        half angle (side 0: the nearer face); negative behind the face (in the strut)."""
        dphi = (th - theta_w + 180.0) % 360.0 - 180.0
        return r * np.sin(np.radians(half - (np.abs(dphi) if side == 0 else side * dphi)))

    def centre_clearance(r: np.ndarray, th: np.ndarray, theta_w: float) -> tuple[np.ndarray, np.ndarray]:
        """How far a point lies outside the opening's horizontal section (<= 0 inside) - the largest of its distances
        past the bore face, the outer face and the nearer strut face, a lower bound of the true distance - and which
        of the three it is (0 bore side, 1 outside, 2 strut side)."""
        c = np.stack([L.r_band_in - r, r - L.body_r, -face_distance(r, th, theta_w)])
        return c.max(axis=0), c.argmax(axis=0)

    def in_opening(r, th, zd, theta_w, g: float, z0: float, z1: float) -> np.ndarray:
        """In window ``theta_w``'s opening grown by g across the faces (Z [z0, z1] as given)."""
        return (centre_clearance(r, th, theta_w)[0] < g) & (zd >= z0) & (zd <= z1)

    # (2) strut walls at mid-window height: on every strut, none in an opening, every side face where it belongs
    W = np.vstack([pts.get(k, empty) for k in WALL_FEATURES])
    W = W[(W[:, 2] >= mid[0]) & (W[:, 2] <= mid[1])]
    r, th, zd = polar(W)
    band = (r > L.r_band_in - 0.5) & (r < L.body_r + 0.5)
    walls_in_openings = int(sum(int(in_opening(r, th, zd, tw, 0.0, -np.inf, np.inf).sum()) for tw in L.window_thetas))
    per_strut = [int((band & (np.abs((th - ts + 180.0) % 360.0 - 180.0) <= 0.5 * L.strut_angle_deg + 0.5)).sum())
                 for ts in L.strut_thetas]
    reg.update({"wall_samples_in_openings_mid_window": walls_in_openings, "wall_samples_per_strut_mid_window": per_strut})
    if walls_in_openings:
        problems.append(f"{walls_in_openings} wall samples inside the window openings at mid-window height")
    if not per_strut or min(per_strut) == 0:
        problems.append(f"no wall extrusion on a strut at mid-window height ({per_strut})")
    O = pts.get("Outer wall", empty)
    O = O[(O[:, 2] >= mid[0]) & (O[:, 2] <= mid[1])]
    ro, tho, _ = polar(O)
    side_rows = (ro > L.r_band_in + lw) & (ro < L.body_r - lw)          # the side faces, clear of the strut corners
    faces = []
    for tw in L.window_thetas:
        near_w = np.abs((tho - tw + 180.0) % 360.0 - 180.0) < 90.0
        for side in (-1, 1):
            d = face_distance(ro, tho, tw, side)
            sel = side_rows & near_w & (np.abs(d) < STRUT_FACE_SELECT_MM)
            err = float(np.median(d[sel])) + 0.5 * lw if sel.any() else None
            faces.append({"face_theta_deg": round((tw + side * half) % 360.0, 3), "samples": int(sel.sum()),
                          "wall_offset_error_mm": None if err is None else round(err, 4)})
    reg["strut_faces"] = faces
    errs = [abs(f["wall_offset_error_mm"]) for f in faces if f["wall_offset_error_mm"] is not None]
    reg["strut_face_offset_error_max_mm"] = round(max(errs), 4) if errs else None
    off_faces = [f for f in faces if f["samples"] < STRUT_FACE_MIN_SAMPLES
                 or abs(f["wall_offset_error_mm"]) > STRUT_FACE_TOL_MM]
    if off_faces:
        problems.append("strut side faces not where the design puts them (outer-wall centre line line_w / 2 inside, "
                        f"tol {STRUT_FACE_TOL_MM:g} mm, >= {STRUT_FACE_MIN_SAMPLES} samples): " + ", ".join(
                            f"{f['face_theta_deg']:g} deg {f['wall_offset_error_mm']} mm / {f['samples']} samples"
                            for f in off_faces))
    # support and bridge extrusion per window
    sup = [(k, v) for k, v in pts.items() if k.startswith(SUPPORT_FEATURE_PREFIX)]
    S = np.vstack([v for _, v in sup] or [empty])
    is_if = np.concatenate([np.full(len(v), "interface" in k.lower()) for k, v in sup] or [np.zeros(0, bool)])
    zsel = (S[:, 2] >= win[0]) & (S[:, 2] <= win[1])
    S, is_if = S[zsel], is_if[zsel]
    B = np.vstack([pts.get(k, empty) for k in CEILING_FEATURES])
    sr, sth, szd = polar(S)
    br, bth, bzd = polar(B)
    z_lo, z_hi = L.z_window_bottom - grow_z, L.z_window_top + grow_z
    in_z = (szd >= z_lo) & (szd <= z_hi)
    windows, clear, clear_side, clear_w = [], np.full(len(S), np.inf), np.zeros(len(S), int), np.zeros(len(S), int)
    for k, tw in enumerate(L.window_thetas):
        c, c_side = centre_clearance(sr, sth, tw)
        s = (c < grow) & in_z
        nearer = c < clear
        clear, clear_side, clear_w = np.where(nearer, c, clear), np.where(nearer, c_side, clear_side), \
            np.where(nearer, k, clear_w)
        b = in_opening(br, bth, bzd, tw, 0.0, L.z_window_bottom - 1.0, L.z_window_bottom)
        windows.append({"theta_deg": round(float(tw), 3), "support_samples": int(s.sum()),
                        "support_path_mm": round(float(S[s, 3].sum()), 1),
                        "support_interface_path_mm": round(float(S[s & is_if, 3].sum()), 1),
                        "support_z_design": [round(float(szd[s].min()), 2), round(float(szd[s].max()), 2)] if s.any() else None,
                        "ceiling_overhang_path_mm": round(float(B[b, 3].sum()), 1)})
    outside = in_z & (clear >= grow)
    if outside.any():
        i = int(np.argmin(np.where(outside, clear, np.inf)))
        res["nearest_support_outside_openings"] = {
            "bead_gap_mm": round(float(clear[i] - grow), 3),
            "side": ("bore side", "outside the body", "strut side")[int(clear_side[i])],
            "window_theta_deg": round(float(L.window_thetas[int(clear_w[i])]), 3),
            "r_design": round(float(sr[i]), 3), "z_design": round(float(szd[i]), 2)}
    else:
        res["nearest_support_outside_openings"] = None
    res.update({"windows": windows,
                "support_path_mm_whole_part": round(float(sum(v[:, 3].sum() for k, v in pts.items()
                                                              if k.startswith(SUPPORT_FEATURE_PREFIX))), 1),
                "max_bridge_length_in_gcode": settings.get("max_bridge_length"),
                "bridge_no_support_in_gcode": settings.get("bridge_no_support")})
    if problems:
        res["reason"] = "registration failed: " + "; ".join(problems)
        return res
    bad = [w for w in windows if w["support_samples"]]
    res["status"] = "FAIL" if bad else "PASS"
    if bad:
        res["reason"] = "support inside the window openings: " + "; ".join(
            f"window {w['theta_deg']:g} deg {w['support_path_mm']:g} mm at design z {w['support_z_design']}" for w in bad)
    return res


def slice_one(stl: str, supports: bool, tmp: str, rho: float, extra: dict | None = None) -> dict:
    """Slice one model file (STL or Bambu project 3MF) with the CanSat process.  Besides the plan values, a successful
    result carries the private keys '_gcode' and '_project' (paths under ``tmp``; dropped before the plan is written)."""
    work = tempfile.mkdtemp(dir=tmp)
    proc = _profile_copy("process", PROCESS, {
        "name": f"CanSat 5wall {'sup' if supports else 'nosup'}",
        "wall_loops": "5", "top_shell_layers": "5", "bottom_shell_layers": "5",
        "sparse_infill_density": "15%", "sparse_infill_pattern": "gyroid",
        "enable_support": "1" if supports else "0", "support_type": BASE_SUPPORT_TYPE,
        "support_threshold_angle": "30", "brim_type": "auto_brim",
        **(extra or {}),
    }, work)
    fil = _profile_copy("filament", FILAMENT, {"name": "PLA Matte rho", "filament_density": [f"{rho:.2f}"]}, work)
    machine = os.path.join(PROFILES, "machine", f"{MACHINE}.json")
    out = os.path.join(work, "out")
    os.makedirs(out)
    cmd = [BAMBU, "--slice", "0", "--arrange", "1", "--orient", "0",
           "--load-settings", f"{machine};{proc}", "--load-filaments", fil,
           "--outputdir", out, "--export-3mf", "plate.3mf", stl]
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=1800, cwd=work)
    res_path = os.path.join(out, "result.json")
    if not os.path.exists(res_path):
        return {"ok": False, "error": f"no result.json (exit {r.returncode})"}
    with open(res_path, "r", encoding="utf-8") as f:
        res = json.load(f)
    if res.get("return_code", -1) != 0:
        return {"ok": False, "error": res.get("error_string", "slice failed")}
    gcode = os.path.join(out, "plate_1.gcode")
    length = None
    settings_seen = {}
    with open(gcode, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = re.match(r"; total filament length \[mm\] : ([0-9.]+)", line)
            if m:
                length = float(m.group(1))
            m2 = re.match(r"; (wall_loops|sparse_infill_density|sparse_infill_pattern|enable_support|support_type|"
                          r"support_style|filament_density) = (.*)", line)
            if m2:
                settings_seen[m2.group(1)] = m2.group(2).strip()
    plate = res["sliced_plates"][0]
    t_s = float(plate.get("main_predication", 0.0))
    grams = (length or 0.0) * math.pi / 4 * FILAMENT_D_MM ** 2 * rho / 1000.0
    return {"ok": True, "time_s": t_s, "filament_mm": length, "filament_g": grams, "settings_in_gcode": settings_seen,
            "support_time_s": float(plate.get("feature_type_times", {}).get("Support", 0.0))
                              + float(plate.get("feature_type_times", {}).get("Support interface", 0.0)),
            "_gcode": gcode, "_project": os.path.join(out, "plate.3mf")}


def _public(entry: dict) -> dict:
    return {k: v for k, v in entry.items() if not k.startswith("_")}


def window_check_summary(res: dict) -> dict:
    """The short form of a window_support_check result kept for a control slice."""
    return {"status": res.get("status"), "reason": res.get("reason"),
            "support_path_mm_in_openings": round(sum(w["support_path_mm"] for w in res.get("windows", [])), 1),
            "support_interface_path_mm_in_openings": round(sum(w.get("support_interface_path_mm", 0.0)
                                                               for w in res.get("windows", [])), 1),
            "windows_with_support": [w["theta_deg"] for w in res.get("windows", []) if w["support_samples"]]}


def slice_intake_with_blockers(mesh: trimesh.Trimesh, rots: list[tuple[str, float]], L, tmp: str, rho: float,
                               supports: bool = True, extra: dict | None = None) -> tuple[dict, str | None]:
    """v0.6.2 intake slice (decision A).  (1) The bare oriented part is sliced (control: its window check shows what
    the slicer does without blockers, and its exported project gives Bambu's resolved settings).  (2) A Bambu project
    = the same part + intake_window_blockers (oriented the same way) + those settings is written and sliced; its G-code
    gets window_support_check.  Returns (plan entry of the blocked slice, path of the blocked project or None).  The
    entry's 'window_support' is the check of the blocked slice, 'window_support_without_blockers' the control's summary."""
    T = orient_matrix(mesh, rots)
    part = orient(mesh, rots)
    bare_path = os.path.join(tmp, "intake_module_bare.stl")
    part.export(bare_path)
    bare = slice_one(bare_path, supports, tmp, rho, extra)
    if not bare.get("ok"):
        return bare, None
    control = window_support_check(bare["_gcode"], bare["_project"], L, mesh, rots)
    blockers = []
    for k, b in enumerate(intake_window_blockers(L), start=1):
        b.apply_transform(T)
        blockers.append((b, "support_blocker", f"window_blocker_{k}"))
    proj = os.path.join(tmp, "intake_module.3mf")
    write_bambu_project(proj, "intake_module", part, blockers, project_settings_of(bare["_project"]))
    entry = slice_one(proj, supports, tmp, rho, extra)
    entry["window_support_without_blockers"] = window_check_summary(control)
    entry["window_blockers"] = {"count": len(blockers), "r_mm": [round(L.r_band_in - BLOCKER_R_MARGIN_MM, 2),
                                                                  round(L.body_r + BLOCKER_R_MARGIN_MM, 2)],
                                "z_design_mm": [round(L.z_window_bottom - BLOCKER_CEILING_MM, 2), round(L.z_window_top, 2)],
                                "sectors_deg": [[round(lo, 3), round(hi, 3)] for lo, hi in L.window_sectors]}
    if entry.get("ok"):
        entry["window_support"] = window_support_check(entry["_gcode"], entry["_project"], L, mesh, rots)
    else:
        entry["window_support"] = {"status": "NOT_VERIFIED", "reason": f"blocked project not sliced: {entry.get('error')}"}
    return entry, proj


def fmt_t(s: float) -> str:
    h, m = divmod(int(round(s / 60.0)), 60)
    return f"{h} h {m:02d} min"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--no-slice", action="store_true")
    ap.add_argument("--check", action="store_true", help="only report whether the print files match the current design")
    ap.add_argument("--adopt", action="store_true",
                    help="record the source STL sha256 in a print_plan.json written before v0.6.3, for every part whose "
                         "print file is proven to be the current source STL oriented (nothing is sliced or rewritten)")
    ap.add_argument("--preset", default=None, help="preset to make the print files for (default: params.json meta.preset); "
                                                   "the Fusion report must be for this preset")
    ap.add_argument("--allow-unanswered", action="store_true",
                    help="accept a Fusion run whose required features have unanswered point-containment evidence "
                         "(recorded in print_plan.json source)")
    a = ap.parse_args(argv)
    if a.check:
        return check_current(a.preset)
    if a.adopt:
        return adopt_sources(a.preset)
    source, problems = fusion_source(list(PARTS), a.preset, a.allow_unanswered)
    if problems:
        print("REFUSED: the Fusion exports in cad/exports/fusion are not from the current design; nothing written:")
        for p in problems:
            print(f"  - {p}")
        print("Run tools/install_fusion_script.py, run CanSatEDFRocket in Fusion 360 (Utilities > Scripts), "
              "check it with tools/compare_stl_slices.py, then run this again.")
        return 2
    if source["containment_unanswered"]:
        print("WARNING (--allow-unanswered): required features without point-containment evidence: " + "; ".join(
            f"{k}: {', '.join(v)}" for k, v in source["containment_unanswered"].items()))
    L = layout.build(source["preset_sliced"])
    rho = float(L.p["printer"]["density_g_cm3"])
    bed_xy = L.p["printer"]["bed_x"] - 2 * L.p["printer"]["usable_margin"]
    bed_z = L.p["printer"]["bed_z"] - L.p["printer"]["usable_margin"]
    os.makedirs(OUT, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix="cansat_slice_")
    exceptions = ", ".join(f"{n}: {o['support_type']}" + (f" / {o['support_style']}" if "support_style" in o else "")
                           + (f" / support_expansion {o['support_expansion']} mm" if "support_expansion" in o else "")
                           for n, o in PART_OVERRIDES.items() if "support_type" in o)
    blocked_parts = {"intake_module"} if L.has_window_gable else set()
    plan = {"source": source, "machine": MACHINE, "process_base": PROCESS, "filament": FILAMENT,
            "overrides": f"wall_loops 5, top/bottom 5 layers, 15 % gyroid, {BASE_SUPPORT_TYPE} supports where listed"
                         + (f" (except {exceptions})" if exceptions else "") + ", auto brim"
                         + ("; intake_module: Bambu project with a support blocker over every window opening"
                            if blocked_parts else ""),
            "parts": {}}
    # v0.6.3: the sha256 of every source STL exactly as read here - --check compares the STLs on disk with these
    source["stl_sha256"], source["stl_sha256_how"] = {}, {}
    window_fail: list[str] = []
    try:
        for name, (qty, cands, sup, why) in PARTS.items():
            if name == "propulsion_module":   # v0.9: fins on the plate diagonals (axis-aligned fins: P2S wrap zone)
                rz = (45.0 - L.fin_thetas[0]) % 90.0
                cands = [[("z", rz)]] if rz else [[]]
            src = source_stl_path(name)
            if not os.path.exists(src):
                print(f"skip {name}: {src} missing")
                continue
            mesh, stl_sha = load_source_stl(name)
            source["stl_sha256"][name] = stl_sha
            source["stl_sha256_how"][name] = "hashed when the STL was read for slicing"
            best = None
            for k, rots in enumerate(cands):
                m = orient(mesh, rots)
                ext = m.extents
                fits = ext[0] <= bed_xy and ext[1] <= bed_xy and ext[2] <= bed_z
                cand_path = os.path.join(tmp, f"{name}_{k}.stl")
                m.export(cand_path)
                entry = {"rotations": rots, "extents_mm": [round(float(x), 1) for x in ext], "fits_bed": bool(fits)}
                proj = None
                if not a.no_slice and os.path.exists(BAMBU):
                    if name in blocked_parts:
                        res, proj = slice_intake_with_blockers(mesh, rots, L, tmp, rho, sup, PART_OVERRIDES.get(name))
                        entry.update(res)
                    else:
                        entry.update(slice_one(cand_path, sup, tmp, rho, PART_OVERRIDES.get(name)))
                    inlay_name = getattr(L, "livery_inlay_part", None) if getattr(L, "has_livery", False) else None
                    if inlay_name and name == L.livery_part and entry.get("ok"):
                        # livery (2026-11-04): the module is printed with its orange inlay as one 2-filament project
                        if os.path.exists(source_stl_path(inlay_name)):
                            inl, inl_sha = load_source_stl(inlay_name)
                            source["stl_sha256"][inlay_name] = inl_sha
                            inl.apply_transform(orient_matrix(mesh, rots))
                            fl = L.p["livery"]["ams"]["flush_volumes_mm3"]
                            r2 = slice_two_colour(m, inl, (name, inlay_name), tmp, rho, PART_OVERRIDES.get(name) or {},
                                                  (L.livery_colours["body"]["hex"], L.livery_colours["logo"]["hex"]),
                                                  (float(fl["white_to_orange"]), float(fl["orange_to_white"])),
                                                  inlay_depth_mm=float(L.livery_depth))
                            entry["two_colour_slice"] = _public(r2)
                            if r2.get("ok"):
                                proj = r2["_project"]
                                entry["time_s"], entry["filament_g"] = r2["time_s"], r2["filament_g"]
                                ic = r2.get("inlay_print_check") or {}
                                if ic.get("status") != "PASS":     # reviewfix F1: one orange pass, grams vs solid
                                    window_fail.append(f"{name}: inlay print check {ic.get('status')} (path ratio "
                                                       f"{ic.get('path_ratio')}, grams {ic.get('g_rel')})")
                        else:
                            entry["two_colour_slice"] = {"ok": False, "error": f"{inlay_name}.stl missing in the Fusion "
                                                                               "exports: single-colour print file"}
                            print(f"WARNING {name}: {inlay_name}.stl missing - the print file has no YUCHAN inlay")
                print(f"{name} cand {k} {rots}: {entry.get('extents_mm')} fits={fits} "
                      f"{fmt_t(entry['time_s']) if entry.get('ok') else entry.get('error', 'not sliced')}"
                      f"{'' if not entry.get('ok') else f', {entry['filament_g']:.1f} g'}")
                key = entry["time_s"] if entry.get("ok") else float("inf")
                if best is None or key < best[0]:
                    best = (key, k, m, entry, proj)
            _, k, m, entry, proj = best
            if proj is not None:
                # the print file is the project (part + window blockers + settings); an STL would lose the blockers
                shutil.copyfile(proj, os.path.join(OUT, f"{name}.3mf"))
                entry["print_file"] = f"{name}.3mf"
            else:
                m.export(os.path.join(OUT, f"{name}.stl"))
                entry["print_file"] = f"{name}.stl"
            # a part has ONE print file: the counterpart an earlier run left (an intake_module.3mf after --no-slice, no
            # Bambu Studio or a failed bare slice; an STL after a blocked slice) is from other geometry or settings
            stale = os.path.join(OUT, f"{name}.stl" if proj is not None else f"{name}.3mf")
            if os.path.exists(stale):
                os.remove(stale)
                print(f"{name}: removed the stale {os.path.basename(stale)} (this run wrote {entry['print_file']})")
            if name in blocked_parts:
                ws = entry.get("window_support") or {"status": "NOT_VERIFIED", "reason": "not sliced (--no-slice or no "
                                                     "Bambu Studio): the window blockers need a slice"}
                ws["required"] = True
                entry["window_support"] = ws
                print(f"{name} window support check: {ws['status']}" + (f" - {ws['reason']}" if ws.get("reason") else ""))
                if ws["status"] != "PASS":
                    window_fail.append(f"{name}: {ws['status']}" + (f" ({ws['reason']})" if ws.get("reason") else ""))
            used = used_support_type(name, sup, entry)
            if used != configured_support_type(name, sup):
                print(f"WARNING {name}: slicer used support {used!r}, configured {configured_support_type(name, sup)!r}")
            plan["parts"][name] = {"quantity": qty, "supports": sup, "support_type_used": used,
                                   "support_style_used": (entry.get("settings_in_gcode") or {}).get("support_style"),
                                   "why": why, "chosen_candidate": k, **_public(entry)}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # totals
    tot_t = sum(v.get("time_s", 0.0) * v["quantity"] for v in plan["parts"].values() if v.get("ok"))
    tot_g = sum(v.get("filament_g", 0.0) * v["quantity"] for v in plan["parts"].values() if v.get("ok"))
    plan["totals"] = {"time_s": tot_t, "filament_g": tot_g,
                      "vehicle_only_time_s": tot_t - plan["parts"].get("launch_pad", {}).get("time_s", 0.0),
                      "vehicle_only_filament_g": tot_g - plan["parts"].get("launch_pad", {}).get("filament_g", 0.0)}
    with open(os.path.join(OUT, "print_plan.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(plan, f, indent=2, ensure_ascii=False)
    lines = ["# 출력 계획 (Bambu Studio CLI로 슬라이스한 실제 값)", "",
             f"원본: Fusion 360 `CanSatEDFRocket.py {source['script_version']}` 내보내기, 프리셋 `{source['preset']}`, "
             f"params.json sha256 `{str(source['params_sha256'])[:12]}…` (`python tools/make_print_files.py --check`로 최신 여부 확인).", "",
             f"프린터 `{MACHINE}`, 공정 `{PROCESS}` 기반 + {plan['overrides']}, 필라멘트 `{FILAMENT}` (밀도 {rho} g/cm³).", "",
             "| 부품 | 출력 파일 | 수량 | 출력 방향 | 서포트 | 크기 (mm) | 1개 출력 시간 | 1개 필라멘트 |",
             "|---|---|---|---|---|---|---|---|"]
    for name, v in plan["parts"].items():
        sl = support_label(v["support_type_used"], v.get("support_style_used"))
        lines.append(f"| {name} | `{v['print_file']}` | {v['quantity']} | {v['why']} | {sl} | "
                     f"{' x '.join(str(x) for x in v['extents_mm'])} | "
                     f"{fmt_t(v['time_s']) if v.get('ok') else '-'} | {v['filament_g']:.1f} g |" if v.get("ok") else
                     f"| {name} | `{v['print_file']}` | {v['quantity']} | {v['why']} | {sl} (설정값) | "
                     f"{v['extents_mm']} | {('슬라이스 실패: ' + str(v['error'])) if 'error' in v else '슬라이스 안 함'} | - |")
    lines += ["", "서포트 열은 Bambu Studio가 G-code 머리말에 기록한 `enable_support` / `support_type` / `support_style` 값입니다 "
                  "(슬라이스하지 않은 부품은 설정값)."]
    for name, v in plan["parts"].items():
        ws = v.get("window_support")
        if not ws:
            continue
        ctrl = v.get("window_support_without_blockers") or {}
        per = ", ".join(f"{w['theta_deg']:g}° {w['support_path_mm']:g} mm" for w in ws.get("windows", []))
        lines += ["", f"**{name} 창 안 서포트 검사 (v0.6.2, 결정 A)**: **{ws['status']}**"
                      + (f" — {ws['reason']}" if ws.get("reason") else "") + ".",
                  (f"출력 파일 `{v['print_file']}`은 Bambu Studio 프로젝트입니다. 창마다 서포트 차단(support blocker) 볼륨이 "
                   "들어 있으니 STL 대신 이 파일을 여세요. " if v["print_file"].endswith(".3mf") else
                   f"출력 파일 `{v['print_file']}`에는 창 서포트 차단 볼륨이 없습니다(슬라이스해야 만들어집니다). ")
                  + f"슬라이스한 G-code의 서포트 압출 경로를 설계 좌표로 되돌려 창 개구부({ws.get('probe', '-')}) 안을 "
                    f"셌습니다: {per or '-'}."
                  + (f" 차단 볼륨 없이 슬라이스하면 {ctrl.get('status')}, 창 안 서포트 {ctrl.get('support_path_mm_in_openings')} mm"
                     if ctrl else "") + "."]
    lines += ["", f"**합계 (수량 반영)**: {fmt_t(tot_t)}, PLA {tot_g:.0f} g "
                  f"(발사대 제외 기체만: {fmt_t(plan['totals']['vehicle_only_time_s'])}, {plan['totals']['vehicle_only_filament_g']:.0f} g)", ""]
    with open(os.path.join(OUT, "print_plan.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
    print("\n".join(lines[-2:]))
    if window_fail:
        print("FAIL (v0.6.2 decision A, no support inside the intake windows; reviewfix F1, one orange inlay pass): "
              + "; ".join(window_fail))
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
