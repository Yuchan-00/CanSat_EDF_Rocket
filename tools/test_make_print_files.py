"""tools/make_print_files.py.

Freshness guard: print files are made only from a Fusion run of the current params.json by the repository's script,
for the preset being sliced (BUDGET by default), with every part exported and no unanswered point-containment evidence
(any derived.extra.<block>.containment_unanswered) unless --allow-unanswered is given; main() refuses (exit 2, nothing
written) otherwise, and --check flags a plan without that provenance.

v0.6.3 geometry-aware freshness: main() records the sha256 of every source STL it read; --check passes when the Fusion
run on disk is valid for the preset (current params.json and script, every part exported, nothing failed or
unanswered) and every sliced part's source STL still has its recorded hash and its print file is on disk - so a
params.json or script change whose Fusion run exports the same printed STLs leaves the set current, a changed STL names
its part, an invalid run on record is never hidden by unchanged STLs.  --adopt records the hashes of a pre-v0.6.3 plan
only for parts whose print file is proven to be the oriented source (byte-identical STL / the 3MF part mesh).

Intake windows (decision A, no support inside them): the window blockers cover the openings only; the Bambu project
writer is deterministic; plate_transform reads the slicer's placement; the G-code sampler handles lines, arcs and the
config block; --check flags a sliced intake whose window check is missing or not PASS.  window_support_check on
synthetic G-code placed like a Bambu arrange: PASS / FAIL per window, including support in the strip the v0.6.2 probe
left out (hugging a strut face, the last degree before a strut, the band's bore and outer faces, a bore bead that
overlaps the opening) - each such case is asserted to lie outside that old probe - while support the slicer keeps
support_object_xy_distance off the band passes; NOT_VERIFIED for a wrong placement, a 0.3 deg turn about the axis (the
ring cannot see it, the strut faces do), a config without layer height and a missing G-code.

main() keeps one print file per part: writing the intake STL (--no-slice, no Bambu Studio, a failed bare slice) removes
an older intake_module.3mf, writing the project removes an older intake_module.stl (slicing stubbed).

With Bambu Studio installed and a current CadQuery twin (cad/exports/cadquery): the real slice through
slice_intake_with_blockers - the bare control gets support in every window (the check is not vacuous), the blocked
project (normal / snug, support_expansion -0.5) none, registered on the ring and all eight strut faces."""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys
import zipfile

import numpy as np
import pytest
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_print_files as mpf  # noqa: E402

DEFAULT = mpf.default_preset()
ALT = next(iter(mpf.layout.load_params()["alt_presets"]))
ROOT = mpf.ROOT


def _stand_in_stls(tmp_path, monkeypatch) -> None:
    """cad/exports/fusion/stl stand-in: one small box per part (deterministic bytes, a different box per part)."""
    src = tmp_path / "fusion_stl_all"
    src.mkdir(exist_ok=True)
    for k, n in enumerate(mpf.PARTS):
        box = trimesh.creation.box(extents=[10.0 + k, 20.0, 30.0])
        box.apply_translation([0.0, 0.0, 100.0 + k])
        box.export(str(src / f"{n}.stl"))
    monkeypatch.setattr(mpf, "SRC", str(src))


def _setup(tmp_path, monkeypatch, *, sha=None, version=None, failed=None, exports=None, preset=None, unanswered=None,
           params=b'{"x": 1}\n'):
    """A params.json, a Fusion report of it (the keywords make it invalid in one way each), stand-in source STLs."""
    _stand_in_stls(tmp_path, monkeypatch)
    params_path = tmp_path / "params.json"
    params_path.write_bytes(params)
    params = params_path
    report = tmp_path / "fusion360_report.json"
    names = list(mpf.PARTS)
    extra = {"intake_windows": {"features_ok": 5, "features_total": 5, "containment_unanswered": []},
             "com_mount": {"features_ok": 8, "features_total": 8, "containment_unanswered": []},
             "pushrod_len_mm_each": [24.7]}
    for block, labels in (unanswered or {}).items():
        extra.setdefault(block, {})["containment_unanswered"] = list(labels)
    rep = {"generator_version": f"CanSatEDFRocket.py {version or mpf.repo_script_version()}; Fusion 2705.1.30",
           "preset": preset or DEFAULT, "params_sha256": sha or mpf._sha256(str(params)),
           "failed_components": failed or {}, "derived": {"extra": extra},
           "exports": {f"STL {n}": f"x/{n}.stl" for n in (names if exports is None else exports)}}
    report.write_text(json.dumps(rep), encoding="utf-8")
    monkeypatch.setattr(mpf, "PARAMS_JSON", str(params))
    monkeypatch.setattr(mpf, "FUSION_REPORT", str(report))
    monkeypatch.setattr(mpf, "OUT", str(tmp_path / "print"))
    return names


def test_current_fusion_run_is_accepted(tmp_path, monkeypatch):
    names = _setup(tmp_path, monkeypatch)
    src, problems = mpf.fusion_source(names)
    assert problems == []
    assert src["script_version"] == mpf.repo_script_version() and src["preset"] == DEFAULT == "BUDGET_90mm_6S"
    assert src["preset_sliced"] == DEFAULT and src["containment_unanswered"] == {}


@pytest.mark.parametrize("kw, needle", [
    ({"sha": "0" * 64}, "params.json changed"),
    ({"version": "0.0.1"}, "CanSatEDFRocket.py 0.0.1"),
    ({"failed": {"intake_module": "boom"}}, "failed components: intake_module"),
    ({"exports": ["vane"]}, "STL not exported"),
    ({"preset": ALT}, f"preset {ALT!r}"),                                                  # v0.6.2: another preset
    ({"unanswered": {"intake_windows": ["window_45"]}}, "intake_windows: window_45"),      # v0.6.2: no evidence
    ({"unanswered": {"servo_leads": ["servo_rebate"], "com_mount": ["com_hole_+22_+22"]}}, "com_mount: com_hole_+22_+22"),
    ({"unanswered": {"intake_servo_leads": ["servo_channel"]}}, "intake_servo_leads: servo_channel"),
])
def test_stale_fusion_run_is_refused(tmp_path, monkeypatch, kw, needle):
    names = _setup(tmp_path, monkeypatch, **kw)
    _, problems = mpf.fusion_source(names)
    assert any(needle in p for p in problems), problems
    # main() refuses before writing anything
    assert mpf.main(["--no-slice"]) == 2
    assert not (tmp_path / "print").exists()


def test_unanswered_containment_needs_the_flag(tmp_path, monkeypatch):
    """Unanswered point containment is refused by default and accepted only with --allow-unanswered (then recorded)."""
    names = _setup(tmp_path, monkeypatch, unanswered={"intake_windows": ["intake_band", "window_45"],
                                                      "com_mount": ["com_boss_+22_+22"]})
    _, problems = mpf.fusion_source(names)
    assert len(problems) == 1 and "--allow-unanswered" in problems[0]
    src, problems = mpf.fusion_source(names, allow_unanswered=True)
    assert problems == []
    assert src["containment_unanswered"] == {"intake_windows": ["intake_band", "window_45"],
                                             "com_mount": ["com_boss_+22_+22"]}
    assert src["allow_unanswered"] is True
    # the flag never hides another problem
    names = _setup(tmp_path, monkeypatch, failed={"intake_module": "boom"}, unanswered={"intake_windows": ["window_45"]})
    _, problems = mpf.fusion_source(names, allow_unanswered=True)
    assert any("failed components" in p for p in problems)


def test_preset_must_match_the_one_sliced(tmp_path, monkeypatch):
    """A report of the alt preset is refused for the default slice and accepted for --preset <alt>; a default report
    is refused for --preset <alt>."""
    names = _setup(tmp_path, monkeypatch, preset=ALT)
    assert mpf.fusion_source(names)[1]
    src, problems = mpf.fusion_source(names, ALT)
    assert problems == [] and src["preset_sliced"] == ALT
    names = _setup(tmp_path, monkeypatch)
    assert any(f"{ALT!r}" in p for p in mpf.fusion_source(names, ALT)[1])
    assert mpf.main(["--no-slice", "--preset", ALT]) == 2
    assert not (tmp_path / "print").exists()


def _write_plan(tmp_path, names, *, hashes=True, print_files=True, parts=None, **source_over) -> dict:
    """print_plan.json as main() writes it for the current Fusion run (source from fusion_source, the source STL hashes
    of the STLs now on disk unless ``hashes`` is False), with a print file per part on disk unless ``print_files`` is
    False; ``source_over`` overrides source keys."""
    out = tmp_path / "print"
    out.mkdir(exist_ok=True)
    src, _ = mpf.fusion_source(names)
    if hashes:
        src["stl_sha256"] = {n: mpf._sha256(mpf.source_stl_path(n)) for n in names}
    src.update(source_over)
    plan = {"source": src, "parts": parts if parts is not None else {n: {"print_file": f"{n}.stl"} for n in names}}
    if print_files:
        for v in plan["parts"].values():
            (out / v["print_file"]).write_bytes(b"print file")
    (out / "print_plan.json").write_text(json.dumps(plan), encoding="utf-8")
    return plan


def test_check_flags_plan_without_source(tmp_path, monkeypatch):
    names = _setup(tmp_path, monkeypatch)
    out = tmp_path / "print"
    out.mkdir()
    plan = {"parts": {n: {"print_file": f"{n}.stl"} for n in names}}
    for n in names:
        (out / f"{n}.stl").write_bytes(b"print file")
    (out / "print_plan.json").write_text(json.dumps(plan), encoding="utf-8")
    assert mpf.check_current() == 2
    _write_plan(tmp_path, names)
    assert mpf.check_current() == 0
    # v0.6.2: a plan made from another preset's run is stale; so is one made from a run with unanswered containment
    # without the flag (the run on disk has it unanswered too)
    assert mpf.check_current(ALT) == 2
    names = _setup(tmp_path, monkeypatch, unanswered={"intake_windows": ["window_45"]})
    _write_plan(tmp_path, names)
    assert mpf.check_current() == 2
    _write_plan(tmp_path, names, allow_unanswered=True)
    assert mpf.check_current() == 0


# --------------------------------------------------------------------------- #
# v0.6.3: geometry-aware freshness (source STL hashes), --adopt
# --------------------------------------------------------------------------- #
def test_check_is_geometry_aware(tmp_path, monkeypatch, capsys):
    """A params.json (or Fusion script) change that changes no printed STL leaves the print set current once Fusion
    has been run on it; before that run the STLs on disk are not known to be the new design's (not verified); a
    changed printed STL makes the set stale, naming the part."""
    names = _setup(tmp_path, monkeypatch)
    _write_plan(tmp_path, names)
    assert mpf.check_current() == 0
    # params.json changes, the Fusion run on disk is of the old one
    (tmp_path / "params.json").write_bytes(b'{"x": 2}\n')
    capsys.readouterr()
    assert mpf.check_current() == 2
    out = capsys.readouterr().out
    assert "not valid for" in out and "params.json changed since the Fusion run" in out
    # the Fusion script is run on the new params.json and exports the same printed STLs: current, without re-slicing
    _setup(tmp_path, monkeypatch, params=b'{"x": 2}\n')
    assert mpf.check_current() == 0
    out = capsys.readouterr().out
    assert "NOTE: sliced from a Fusion run of params.json" in out and out.rstrip().endswith("no printed STL changed since slicing")
    # a new Fusion script version whose run exports the same STLs: still current (the plan's version is only noted)
    monkeypatch.setattr(mpf, "repo_script_version", lambda: "9.9.9")
    assert mpf.check_current() == 2                                     # the run on disk is by the older script
    _setup(tmp_path, monkeypatch, params=b'{"x": 2}\n')                 # run again by 9.9.9
    assert mpf.check_current() == 0
    assert "sliced from CanSatEDFRocket.py" in capsys.readouterr().out
    # a printed STL changes (the vane): stale, the part named
    box = trimesh.creation.box(extents=[3.0, 4.0, 5.0])
    box.export(mpf.source_stl_path("vane"))
    assert mpf.check_current() == 2
    out = capsys.readouterr().out
    assert "vane: " in out and "changed since it was sliced" in out and "vane_horn" not in out
    os.remove(mpf.source_stl_path("vane"))
    assert mpf.check_current() == 2 and "vane: source STL" in capsys.readouterr().out


@pytest.mark.parametrize("kw, needle", [
    ({"failed": {"vane": "boom"}}, "failed components: vane"),
    ({"exports": [n for n in mpf.PARTS if n != "vane"]}, "STL not exported by that Fusion run"),
    ({"unanswered": {"com_mount": ["com_hole_+22_+22"]}}, "com_mount: com_hole_+22_+22"),
    ({"preset": ALT}, f"preset {ALT!r}"),
    ({"version": "0.0.1"}, "CanSatEDFRocket.py 0.0.1"),
])
def test_check_needs_a_valid_fusion_run(tmp_path, monkeypatch, capsys, kw, needle):
    """Unchanged STLs are not enough: the Fusion run on disk must be valid for the preset (no failed component, every
    part exported, containment answered, this preset, the repository's script)."""
    names = _setup(tmp_path, monkeypatch)
    _write_plan(tmp_path, names)
    assert mpf.check_current() == 0
    _setup(tmp_path, monkeypatch, **kw)                                 # same STLs, an invalid run on record
    capsys.readouterr()
    assert mpf.check_current() == 2
    assert needle in capsys.readouterr().out


def test_check_needs_the_hashes_and_the_print_files(tmp_path, monkeypatch, capsys):
    names = _setup(tmp_path, monkeypatch)
    _write_plan(tmp_path, names, hashes=False)                          # a plan written before v0.6.3
    assert mpf.check_current() == 2 and "--adopt" in capsys.readouterr().out
    plan = _write_plan(tmp_path, names)
    del plan["source"]["stl_sha256"]["piston"]
    (tmp_path / "print" / "print_plan.json").write_text(json.dumps(plan), encoding="utf-8")
    assert mpf.check_current() == 2 and "piston: no source STL sha256 recorded" in capsys.readouterr().out
    _write_plan(tmp_path, names)
    os.rename(tmp_path / "print" / "launch_pad.stl", tmp_path / "print" / "launch_pad-printed.stl")
    assert mpf.check_current() == 2
    assert "launch_pad: print file launch_pad.stl is not in" in capsys.readouterr().out


def test_main_records_the_source_stl_hashes(tmp_path, monkeypatch):
    """main() hashes exactly the bytes it orients; --adopt on that plan without the hashes proves every part again."""
    names = _setup(tmp_path, monkeypatch)
    hashes = {n: mpf._sha256(mpf.source_stl_path(n)) for n in names}
    assert mpf.main(["--no-slice"]) == 3                                # the intake window check needs a slice
    path = tmp_path / "print" / "print_plan.json"
    plan = json.loads(path.read_text(encoding="utf-8"))
    assert plan["source"]["stl_sha256"] == hashes
    assert set(plan["source"]["stl_sha256_how"].values()) == {"hashed when the STL was read for slicing"}
    del plan["source"]["stl_sha256"], plan["source"]["stl_sha256_how"]
    path.write_text(json.dumps(plan), encoding="utf-8")
    assert mpf.adopt_sources() == 0
    again = json.loads(path.read_text(encoding="utf-8"))
    assert again["source"]["stl_sha256"] == hashes and "stl_sha256_not_adopted" not in again["source"]
    assert again["parts"] == plan["parts"]


def _oriented_print_set(tmp_path, names) -> dict:
    """Print files as main() writes them from the stand-in STLs (the intake as a Bambu project with a blocker) and a
    pre-v0.6.3 plan (no source hashes) naming them."""
    out = tmp_path / "print"
    out.mkdir(exist_ok=True)
    parts = {}
    for n in names:
        rots = mpf.PARTS[n][1][0]
        m = mpf.orient(trimesh.load(mpf.source_stl_path(n), force="mesh"), rots)
        if n == "intake_module":
            mpf.write_bambu_project(str(out / f"{n}.3mf"), n, m, [(mpf.sector_mesh(5, 8, 0, 90, 0, 10), "support_blocker",
                                                                   "b")], b'{"x": "1"}')
            pf = f"{n}.3mf"
        else:
            m.export(str(out / f"{n}.stl"))
            pf = f"{n}.stl"
        parts[n] = {"rotations": [list(r) for r in rots], "print_file": pf, "ok": False}
    return _write_plan(tmp_path, names, hashes=False, print_files=False, parts=parts)


def test_adopt_proves_each_part_or_leaves_it_out(tmp_path, monkeypatch, capsys):
    names = _setup(tmp_path, monkeypatch)
    plan = _oriented_print_set(tmp_path, names)
    # three parts that cannot be proven: a print file renamed after slicing, a print file of other geometry, a source
    # STL that changed since slicing
    os.rename(tmp_path / "print" / "launch_pad.stl", tmp_path / "print" / "launch_pad-printed.stl")
    trimesh.creation.box(extents=[1.0, 1.0, 1.0]).export(str(tmp_path / "print" / "piston.stl"))
    trimesh.creation.box(extents=[5.0, 5.0, 5.0]).export(mpf.source_stl_path("vane"))
    assert mpf.adopt_sources() == 2
    got = json.loads((tmp_path / "print" / "print_plan.json").read_text(encoding="utf-8"))
    src = got["source"]
    assert set(src["stl_sha256"]) == set(names) - {"launch_pad", "piston", "vane"}
    assert all(h == mpf._sha256(mpf.source_stl_path(n)) for n, h in src["stl_sha256"].items())
    assert "byte-identical" in src["stl_sha256_how"]["vane_horn"]
    assert "part mesh is the oriented source STL" in src["stl_sha256_how"]["intake_module"]
    assert set(src["stl_sha256_not_adopted"]) == {"launch_pad", "piston", "vane"}
    assert got["parts"] == plan["parts"]                                # nothing but the source hashes written
    capsys.readouterr()
    assert mpf.check_current() == 2
    out = capsys.readouterr().out
    for n in ("launch_pad", "piston", "vane"):
        assert f"  - {n}: no source STL sha256 recorded (not adopted:" in out, n
    assert "  - vane_horn" not in out and "  - intake_module" not in out
    # the intake project's part mesh moved by 1 um: not the oriented source
    m = mpf.orient(trimesh.load(mpf.source_stl_path("intake_module"), force="mesh"), mpf.PARTS["intake_module"][1][0])
    m.apply_translation([0.0, 0.0, 1e-3])
    mpf.write_bambu_project(str(tmp_path / "print" / "intake_module.3mf"), "intake_module", m, [], b"{}")
    ok, why = mpf.print_file_is_oriented_source(plan["parts"]["intake_module"],
                                                trimesh.load(mpf.source_stl_path("intake_module"), force="mesh"))
    assert not ok and "differs" in why


# --------------------------------------------------------------------------- #
# v0.6.2 decision A: no support inside the intake windows
# --------------------------------------------------------------------------- #
L_DEF = mpf.layout.build(DEFAULT)
ROTS = mpf.PARTS["intake_module"][1][0]


def test_check_flags_a_failed_or_missing_window_check(tmp_path, monkeypatch):
    """--check: a required window check that is not PASS, or a sliced intake without one, makes the plan STALE; an
    unsliced intake (--no-slice) does not."""
    names = _setup(tmp_path, monkeypatch)
    out = tmp_path / "print"
    plan = _write_plan(tmp_path, names)

    def check(entry):
        plan["parts"]["intake_module"] = {"print_file": "intake_module.stl", **entry}
        (out / "print_plan.json").write_text(json.dumps(plan), encoding="utf-8")
        return mpf.check_current()

    assert check({}) == 0                                                                     # not sliced
    assert check({"ok": True}) == 2                                                           # sliced, no check
    assert check({"ok": True, "window_support": {"status": "FAIL", "required": True, "reason": "x"}}) == 2
    assert check({"ok": True, "window_support": {"status": "NOT_VERIFIED", "required": True}}) == 2
    assert check({"ok": True, "window_support": {"status": "PASS", "required": True}}) == 0


def test_window_blockers_cover_the_openings_only():
    """One closed blocker per window: the sector volume r [r_band_in - 0.3, body_r + 0.3], Z [z_window_bottom - 1,
    z_window_top]; it contains the opening and not the struts, the bore or the ring above the windows."""
    L = L_DEF
    bl = mpf.intake_window_blockers(L)
    assert len(bl) == len(L.window_sectors) == L.strut_count
    r0, r1 = L.r_band_in - mpf.BLOCKER_R_MARGIN_MM, L.body_r + mpf.BLOCKER_R_MARGIN_MM
    h = L.z_window_top - (L.z_window_bottom - mpf.BLOCKER_CEILING_MM)
    exact = 0.5 * (r1 ** 2 - r0 ** 2) * math.radians(L.window_angle_deg) * h
    zm = 0.5 * (L.z_window_bottom + L.z_window_top)
    for b, tw, ts in zip(bl, L.window_thetas, L.strut_thetas):
        assert b.is_watertight and b.is_winding_consistent
        assert b.volume == pytest.approx(exact, rel=2e-3)          # chordal polygon of BLOCKER_ARC_STEPS steps
        pol = lambda r, t, z: [r * math.cos(math.radians(t)), r * math.sin(math.radians(t)), z]  # noqa: E731
        rm = 0.5 * (L.r_band_in + L.body_r)
        inside = b.contains([pol(rm, tw, zm), pol(rm, tw, L.z_window_bottom + 0.1), pol(rm, tw + L.window_half_angle_deg - 0.5, zm)])
        outside = b.contains([pol(rm, ts, zm), pol(L.r_band_in - 1.0, tw, zm), pol(rm, tw, L.z_window_bottom - 2.0),
                              pol(rm, tw, L.z_window_top + 0.5)])
        assert inside.all() and not outside.any()


def test_bambu_project_writer_is_deterministic(tmp_path):
    part = trimesh.creation.box(extents=[10, 20, 30])
    part.apply_translation([128, 128, 15])
    blk = mpf.sector_mesh(5, 8, 0, 90, 0, 10)
    paths = [tmp_path / "a.3mf", tmp_path / "b.3mf"]
    for p in paths:
        mpf.write_bambu_project(str(p), "box", part, [(blk, "support_blocker", "blk")], b'{"x": "1"}')
    assert paths[0].read_bytes() == paths[1].read_bytes()
    with zipfile.ZipFile(paths[0]) as z:
        cfg = z.read("Metadata/model_settings.config").decode()
        assert '<part id="1" subtype="normal_part">' in cfg and '<part id="2" subtype="support_blocker">' in cfg
        assert z.read("Metadata/project_settings.config") == b'{"x": "1"}'
        obj = z.read("3D/Objects/object_1.model").decode()
        assert obj.count("<vertex ") == len(part.vertices) + len(blk.vertices)
        assert z.read("3D/3dmodel.model").decode().count("<component ") == 2


def _rv(M: np.ndarray) -> str:
    """4 x 4 (column vectors) -> 3MF transform attribute (row-vector convention)."""
    return " ".join(f"{v:.9g}" for v in list(M[:3, :3].T.reshape(-1)) + list(M[:3, 3]))


def _exported_project(path, part, item: np.ndarray, extra_mesh=None) -> None:
    """A project the way Bambu Studio exports it: object vertices recentred by the part's source_offset, component
    = translate(source_offset), the arrange in the build item; optionally a second (modifier) part."""
    lo, hi = part.bounds
    off = 0.5 * (lo + hi)
    meshes = [part.vertices - off] + ([extra_mesh.vertices] if extra_mesh is not None else [])
    objs = "".join(f'<object id="{k}" type="model"><mesh><vertices>'
                   + "".join(f'<vertex x="{x}" y="{y}" z="{z}"/>' for x, y, z in V) + "</vertices></mesh></object>"
                   for k, V in enumerate(meshes, start=1))
    comp = trimesh.transformations.translation_matrix(off)
    comps = f'<component p:path="/3D/Objects/object_1.model" objectid="1" transform="{_rv(comp)}"/>'
    parts = (f'<part id="1" subtype="normal_part"><metadata key="source_offset_x" value="{off[0]}"/>'
             f'<metadata key="source_offset_y" value="{off[1]}"/><metadata key="source_offset_z" value="{off[2]}"/></part>')
    if extra_mesh is not None:
        comps += f'<component p:path="/3D/Objects/object_1.model" objectid="2" transform="{_rv(np.eye(4))}"/>'
        parts += '<part id="2" subtype="support_blocker"><metadata key="name" value="b"/></part>'
    with zipfile.ZipFile(path, "w") as z:
        z.writestr("3D/3dmodel.model", f'<model><resources><object id="9"><components>{comps}</components></object>'
                                       f'</resources><build><item objectid="9" transform="{_rv(item)}"/></build></model>')
        z.writestr("3D/Objects/object_1.model", f"<model><resources>{objs}</resources></model>")
        z.writestr("Metadata/model_settings.config", f'<config><object id="9">{parts}</object></config>')


def test_plate_transform_reads_the_slicers_placement(tmp_path):
    part = trimesh.creation.box(extents=[10, 20, 30])
    part.apply_translation([128, 128, 15])
    item = trimesh.transformations.translation_matrix([99.15, 280.17, 0]) @ \
        trimesh.transformations.rotation_matrix(math.radians(225), [0, 0, 1])
    p = tmp_path / "plate.3mf"
    _exported_project(str(p), part, item, extra_mesh=mpf.sector_mesh(5, 8, 0, 90, 0, 10))
    M, info = mpf.plate_transform(str(p), part.bounds)
    assert np.allclose(M, item, atol=1e-6)
    assert info["arrange_rotation_z_deg"] == pytest.approx(225.0) and info["mesh_vs_model_bounds_mm"] < 1e-6
    with pytest.raises(ValueError, match="differs from the sliced model"):
        mpf.plate_transform(str(p), part.bounds + 1.0)


def test_gcode_samples_lines_arcs_and_config(tmp_path):
    g = tmp_path / "t.gcode"
    g.write_text("\n".join([
        "; CONFIG_BLOCK_START", "; line_width = 0.4", "; max_bridge_length = 10", "; CONFIG_BLOCK_END",
        "; Z_HEIGHT: 1.2", "; LAYER_HEIGHT: 0.2", "; FEATURE: Support",
        "G1 X0 Y0 F3000", "G1 X10 Y0 E.5", "G1 X10 Y5 E-.8", "G1 X0 Y5 F3000",          # retract and travel: no samples
        "; FEATURE: Outer wall", "G1 X10 Y0 F3000", "G3 X0 Y10 I-10 J0 E.7",               # quarter arc r 10 about 0,0
    ]) + "\n", encoding="utf-8")
    pts, cfg = mpf.gcode_extrusion_samples(str(g))
    assert cfg == {"line_width": "0.4", "max_bridge_length": "10"}
    s = pts["Support"]
    assert len(s) == 10 and s[:, 3].sum() == pytest.approx(10.0) and np.allclose(s[:, 2], 1.1)
    a = pts["Outer wall"]
    assert len(a) == 16 and a[:, 3].sum() == pytest.approx(5 * math.pi, rel=1e-9)
    assert np.allclose(np.hypot(a[:, 0], a[:, 1]), 10.0) and (a[:, 0] > 0).all() and (a[:, 1] > 0).all()
    assert set(mpf.gcode_extrusion_samples(str(g), lambda f, z: f == "Support")[0]) == {"Support"}


LW = 0.4                       # line_width / support_line_width of the synthetic G-code
XY_GAP = 0.35                  # its support_object_xy_distance (the Bambu P2S 0.20 mm Standard value)
HUG = XY_GAP + LW / 2          # centre line of support the slicer lays against a face: xy gap + half a line inside
SYN_CONFIG = {"line_width": "0.4", "outer_wall_line_width": "0", "support_line_width": "0",
              "support_object_xy_distance": "0.35", "layer_height": "0.2"}


def _arc(r: float, t0: float, t1: float, z: float, n: int | None = None) -> list[tuple[float, float, float]]:
    """Design-frame points of an arc of radius r about the intake axis, theta t0 -> t1 (deg) at height z."""
    n = n or max(2, int(abs(t1 - t0) * 2))
    return [(r * math.cos(math.radians(t)), r * math.sin(math.radians(t)), z) for t in np.linspace(t0, t1, n + 1)]


def _face_line(theta_w: float, side: int, delta: float, z: float, s0: float = 57.5, s1: float = 59.5,
               n: int = 20) -> list[tuple[float, float, float]]:
    """Design-frame points of a line parallel to the strut face of window theta_w at theta_w + side x window half
    angle, ``delta`` mm from it into the window (negative: behind it, in the strut), s0 .. s1 mm along the face."""
    tf = math.radians(theta_w + side * L_DEF.window_half_angle_deg)
    u, nrm = (math.cos(tf), math.sin(tf)), (side * math.sin(tf), -side * math.cos(tf))   # nrm points into the window
    return [(s * u[0] + delta * nrm[0], s * u[1] + delta * nrm[1], z) for s in np.linspace(s0, s1, n + 1)]


def _in_v062_probe(p: tuple[float, float, float], L) -> bool:
    """The probe this check had before the verifier finding: 1 deg inside the strut faces, 0.3 mm inside the band."""
    r, th = math.hypot(p[0], p[1]), math.degrees(math.atan2(p[1], p[0]))
    return any(abs((th - tw + 180.0) % 360.0 - 180.0) < L.window_half_angle_deg - 1.0 and L.r_band_in + 0.3 < r
               < L.body_r - 0.3 and L.z_window_bottom <= p[2] <= L.z_window_top for tw in L.window_thetas)


def _synthetic_intake_gcode(path, G: np.ndarray, L, support: list[list[tuple[float, float, float]]],
                            config: dict | None = None) -> None:
    """G-code of the sliced intake reduced to what window_support_check reads, made in the design frame and placed
    by G (design -> plate): the solid ring above the windows (outer contour + hole), the strut walls at mid-window
    height (outer-wall centre lines LW / 2 inside every face, as the slicer lays them), and support paths given as
    design-frame polylines."""
    def path_lines(pts, feat):
        q = [G @ np.array([x, y, z, 1.0]) for x, y, z in pts]
        return [f"; Z_HEIGHT: {q[0][2] + 0.1:.3f}", "; LAYER_HEIGHT: 0.2", f"; FEATURE: {feat}",
                f"G1 X{q[0][0]:.4f} Y{q[0][1]:.4f} F3000"] + [f"G1 X{p[0]:.4f} Y{p[1]:.4f} E.02" for p in q[1:]]

    cfg = SYN_CONFIG if config is None else config
    lines = ["; CONFIG_BLOCK_START"] + [f"; {k} = {v}" for k, v in cfg.items()] + ["; CONFIG_BLOCK_END"]
    for z in np.linspace(L.z_window_top + 1.0, L.z_intake_bulkhead_bottom - 1.0, 4):
        lines += path_lines(_arc(L.body_r - LW / 2, 0, 360, z, 720), "Outer wall")
        lines += path_lines(_arc(L.r_band_in + LW / 2, 0, 360, z, 720), "Outer wall")
    half = 0.5 * L.strut_angle_deg
    for z in np.linspace(L.z_gable_top + 3.0, L.z_window_top - 3.0, 4):
        for ts in L.strut_thetas:
            lines += path_lines(_arc(L.body_r - LW / 2, ts - half + 0.3, ts + half - 0.3, z), "Outer wall")
            lines += path_lines(_arc(L.r_band_in + LW / 2, ts - half + 0.3, ts + half - 0.3, z), "Inner wall")
        for tw in L.window_thetas:
            for side in (-1, 1):
                lines += path_lines(_face_line(tw, side, -LW / 2, z, L.r_band_in + LW / 2, L.body_r - LW / 2, 26),
                                    "Outer wall")
    for pts in support:
        lines += path_lines(pts, "Support")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _synthetic_case(tmp_path, monkeypatch, support, config=None):
    """(mesh, P, G-code path, project path) of a synthetic intake slice placed by a Bambu-like arrange P, with
    plate_transform patched to return P."""
    L = L_DEF
    mesh = trimesh.creation.box(extents=[2 * L.body_r, 2 * L.body_r, L.z_intake_spigot_top - L.z_intake_bottom])
    mesh.apply_translation([0, 0, 0.5 * (L.z_intake_spigot_top + L.z_intake_bottom)])
    T = mpf.orient_matrix(mesh, ROTS)
    P = trimesh.transformations.translation_matrix([-80.17, 99.15, 0]) @ \
        trimesh.transformations.rotation_matrix(math.radians(315), [0, 0, 1])           # an arrange like Bambu's
    g, proj = tmp_path / "plate_1.gcode", tmp_path / "plate.3mf"
    proj.write_bytes(b"stand-in")
    _synthetic_intake_gcode(g, P @ T, L, support, config)
    monkeypatch.setattr(mpf, "plate_transform", lambda p, b: (P, {"arrange_rotation_z_deg": 315.0}))
    return mesh, P @ T, P, g, proj


_H, _RIN, _ROUT = L_DEF.window_half_angle_deg, L_DEF.r_band_in, L_DEF.body_r
_ZM = 0.5 * (L_DEF.z_gable_top + L_DEF.z_window_top)                                     # mid-window, above the gables


@pytest.mark.parametrize("support, status, flagged, v062_blind", [
    pytest.param([], "PASS", [], False, id="none"),
    pytest.param([_arc(58.5, 40.0, 50.0, 260.0)], "FAIL", [45.0], False, id="window-centre"),
    pytest.param([_arc(58.5, 220.0, 230.0, L_DEF.z_window_bottom + 0.3)], "FAIL", [225.0], False, id="under-a-ceiling"),
    pytest.param([_arc(54.0, 40.0, 50.0, 260.0), _arc(63.0, 40.0, 50.0, 260.0), _arc(58.5, -5.0, 5.0, 260.0)],
                 "PASS", [], False, id="bore-outside-strut"),
    # the strip of each opening beside the struts and the band faces, which the v0.6.2 probe did not look at
    pytest.param([_face_line(45.0, 1, HUG, _ZM)], "FAIL", [45.0], True, id="hugging-strut-90-from-window-45"),
    pytest.param([_face_line(225.0, -1, HUG, _ZM)], "FAIL", [225.0], True, id="hugging-strut-180-from-window-225"),
    pytest.param([_arc(58.5, 315.0 + _H - 0.9, 315.0 + _H - 0.1, _ZM)], "FAIL", [315.0], True, id="last-degree-before-strut-0"),
    pytest.param([_arc(_RIN + 0.15, 130.0, 140.0, _ZM)], "FAIL", [135.0], True, id="at-the-bore-face"),
    pytest.param([_arc(_ROUT - 0.15, 310.0, 320.0, _ZM)], "FAIL", [315.0], True, id="at-the-outer-face"),
    pytest.param([_arc(_RIN - LW / 2 + 0.05, 130.0, 140.0, _ZM)], "FAIL", [135.0], True, id="bore-bead-overlapping-0.05"),
    # support the slicer keeps support_object_xy_distance off the band, all the way round, bore side and outside
    pytest.param([_arc(_RIN - HUG, 0.0, 359.5, _ZM), _arc(_ROUT + HUG, 0.0, 359.5, _ZM)], "PASS", [], False,
                 id="xy-gap-beside-the-band"),
])
def test_window_support_check_on_synthetic_gcode(tmp_path, monkeypatch, support, status, flagged, v062_blind):
    L = L_DEF
    if v062_blind:                    # the case lies wholly in the strip the old probe left out: it would have PASSed
        assert not any(_in_v062_probe(p, L) for pts in support for p in pts)
    mesh, _, _, g, proj = _synthetic_case(tmp_path, monkeypatch, support)
    res = mpf.window_support_check(str(g), str(proj), L, mesh, ROTS)
    assert res["status"] == status, res.get("reason")
    assert [w["theta_deg"] for w in res["windows"] if w["support_samples"]] == flagged
    reg = res["registration"]
    assert reg["ring_fit_centre_offset"] < 0.01 and reg["wall_samples_in_openings_mid_window"] == 0
    assert min(reg["wall_samples_per_strut_mid_window"]) > 0
    assert len(reg["strut_faces"]) == 2 * L.strut_count and reg["strut_face_offset_error_max_mm"] < 0.01
    tol = res["probe_tolerances"]
    assert tol["grow_mm"] == pytest.approx(LW / 2) and tol["support_line_width_from"] == "line_width"
    assert tol["hugging_support_centre_inside_face_mm"] == pytest.approx(HUG)
    if status == "PASS" and support:  # the support beside the openings is reported with its bead gap to the probe
        near = res["nearest_support_outside_openings"]
        assert near["bead_gap_mm"] >= XY_GAP - 0.01


def test_window_support_check_is_not_verified_without_registration(tmp_path, monkeypatch):
    """A wrong placement, a small turn about the axis that only the strut faces reveal, a config without the layer
    height and a missing G-code are NOT_VERIFIED, never PASS."""
    L = L_DEF
    mesh, G, P, g, proj = _synthetic_case(tmp_path, monkeypatch, [])
    assert mpf.window_support_check(str(g), str(proj), L, mesh, ROTS)["status"] == "PASS"
    # the slicer turned the part 45 deg more than the project says
    Q = trimesh.transformations.rotation_matrix(math.radians(45), [0, 0, 1]) @ P
    monkeypatch.setattr(mpf, "plate_transform", lambda p, b: (Q, {}))
    bad = mpf.window_support_check(str(g), str(proj), L, mesh, ROTS)
    assert bad["status"] == "NOT_VERIFIED" and "registration failed" in bad["reason"]
    # 0.3 deg about the intake axis: the ring fits perfectly, the strut faces are 0.3 mm off
    Q = trimesh.transformations.rotation_matrix(math.radians(0.3), [0, 0, 1], point=[G[0, 3], G[1, 3], 0.0]) @ P
    monkeypatch.setattr(mpf, "plate_transform", lambda p, b: (Q, {}))
    bad = mpf.window_support_check(str(g), str(proj), L, mesh, ROTS)
    assert bad["status"] == "NOT_VERIFIED" and "strut side faces" in bad["reason"], bad.get("reason")
    assert bad["registration"]["ring_fit_centre_offset"] < 0.01
    assert 0.25 < bad["registration"]["strut_face_offset_error_max_mm"] < 0.35
    # no layer height in the config: the probe's Z tolerance is unknown
    mesh, _, _, g, proj = _synthetic_case(tmp_path, monkeypatch, [],
                                          {k: v for k, v in SYN_CONFIG.items() if k != "layer_height"})
    bad = mpf.window_support_check(str(g), str(proj), L, mesh, ROTS)
    assert bad["status"] == "NOT_VERIFIED" and "layer height" in bad["reason"]
    # no G-code / no project
    assert mpf.window_support_check(str(tmp_path / "none.gcode"), str(proj), L, mesh, ROTS)["status"] == "NOT_VERIFIED"


def test_setting_mm_falls_back_on_zero():
    assert mpf._setting_mm({"support_line_width": "0", "line_width": "0.42"}, "support_line_width", "line_width") \
        == (0.42, "line_width")
    assert mpf._setting_mm({"layer_height": "x"}, "layer_height") == (None, None)


def _intake_stl_source(tmp_path, monkeypatch) -> None:
    """cad/exports/fusion/stl stand-in holding only an intake STL (main() skips the missing parts)."""
    src = tmp_path / "fusion_stl"
    src.mkdir()
    box = trimesh.creation.box(extents=[120, 120, 200])
    box.apply_translation([0, 0, 250])
    box.export(str(src / "intake_module.stl"))
    monkeypatch.setattr(mpf, "SRC", str(src))


@pytest.mark.parametrize("how", ["no-slice", "no-bambu", "bare-slice-failed"])
def test_intake_stl_removes_a_stale_project(tmp_path, monkeypatch, how):
    """When the intake is written as an STL, an intake_module.3mf from an earlier run is removed (it would carry other
    geometry); the window check is NOT_VERIFIED and the exit code 3."""
    _setup(tmp_path, monkeypatch)
    _intake_stl_source(tmp_path, monkeypatch)
    out = tmp_path / "print"
    out.mkdir()
    (out / "intake_module.3mf").write_bytes(b"older project")
    argv = ["--no-slice"] if how == "no-slice" else []
    if how == "no-bambu":
        monkeypatch.setattr(mpf, "BAMBU", str(tmp_path / "no-bambu.exe"))
    elif how == "bare-slice-failed":
        monkeypatch.setattr(mpf, "BAMBU", sys.executable)
        monkeypatch.setattr(mpf, "slice_intake_with_blockers",
                            lambda *a, **k: ({"ok": False, "error": "bare slice failed"}, None))
    assert mpf.main(argv) == 3
    assert (out / "intake_module.stl").is_file() and not (out / "intake_module.3mf").exists()
    plan = json.loads((out / "print_plan.json").read_text(encoding="utf-8"))
    entry = plan["parts"]["intake_module"]
    assert entry["print_file"] == "intake_module.stl" and entry["window_support"]["status"] == "NOT_VERIFIED"


def test_intake_project_removes_a_stale_stl(tmp_path, monkeypatch):
    """When the blocked project is made, it is the intake's print file and an older intake_module.stl is removed."""
    _setup(tmp_path, monkeypatch)
    _intake_stl_source(tmp_path, monkeypatch)
    out = tmp_path / "print"
    out.mkdir()
    (out / "intake_module.stl").write_bytes(b"older stl")
    monkeypatch.setattr(mpf, "BAMBU", sys.executable)

    def fake_slice(mesh, rots, L, tmp, rho, supports=True, extra=None):
        proj = os.path.join(tmp, "intake_module.3mf")
        with open(proj, "wb") as f:
            f.write(b"blocked project")
        return ({"ok": True, "time_s": 3600.0, "filament_mm": 1000.0, "filament_g": 3.0,
                 "settings_in_gcode": {"enable_support": "1", "support_type": "normal(auto)", "support_style": "snug"},
                 "window_support": {"status": "PASS", "windows": []}}, proj)

    monkeypatch.setattr(mpf, "slice_intake_with_blockers", fake_slice)
    assert mpf.main([]) == 0
    assert (out / "intake_module.3mf").read_bytes() == b"blocked project" and not (out / "intake_module.stl").exists()
    plan = json.loads((out / "print_plan.json").read_text(encoding="utf-8"))
    assert plan["parts"]["intake_module"]["print_file"] == "intake_module.3mf"


CQ_STL = os.path.join(ROOT, "cad", "exports", "cadquery", "stl", "intake_module.stl")
CQ_REPORT = os.path.join(ROOT, "cad", "exports", "cadquery_report.json")


def _twin_state() -> str | None:
    """Why the real-slice test cannot run (None: it can)."""
    if not os.path.exists(mpf.BAMBU):
        return f"Bambu Studio not installed ({mpf.BAMBU})"
    if not (os.path.isfile(CQ_STL) and os.path.isfile(CQ_REPORT)):
        return "no CadQuery twin export (python cad/cadquery/build_rocket.py)"
    with open(CQ_REPORT, "r", encoding="utf-8") as f:
        rep = json.load(f)
    with open(mpf.PARAMS_JSON, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    if rep.get("params_sha256") != sha or rep.get("preset") != DEFAULT:
        return "CadQuery twin is stale (python cad/cadquery/build_rocket.py)"
    return None


@pytest.mark.skipif(_twin_state() is not None, reason=str(_twin_state()))
def test_sliced_intake_has_no_support_inside_the_windows(tmp_path):
    """Real Bambu Studio slice of the CadQuery intake (the Fusion export's twin) through the make_print_files intake
    path: without blockers the slicer puts support in every window (so the check sees it), with them none - over the
    whole opening up to the strut faces, registered on the ring and on all eight strut faces."""
    L = L_DEF
    mesh = trimesh.load(CQ_STL, force="mesh")
    entry, proj = mpf.slice_intake_with_blockers(mesh, ROTS, L, str(tmp_path), float(L.p["printer"]["density_g_cm3"]),
                                                 True, mpf.PART_OVERRIDES["intake_module"])
    assert entry.get("ok"), entry.get("error")
    assert proj and os.path.isfile(proj)
    ctrl = entry["window_support_without_blockers"]
    assert ctrl["status"] == "FAIL" and ctrl["windows_with_support"] == [round(t, 3) for t in L.window_thetas]
    ws = entry["window_support"]
    reg = ws["registration"]
    assert reg["ring_fit_centre_offset"] < 0.05 and reg["wall_samples_in_openings_mid_window"] == 0
    assert len(reg["strut_faces"]) == 2 * L.strut_count
    assert min(f["samples"] for f in reg["strut_faces"]) >= mpf.STRUT_FACE_MIN_SAMPLES
    assert reg["strut_face_offset_error_max_mm"] < 0.05
    assert ws["probe_tolerances"]["grow_mm"] > 0.0
    assert ws["status"] == "PASS", ws.get("reason")
    # the support next to the openings (the shelf-ledge support in the bore) keeps a bead gap to the probe
    assert ws["nearest_support_outside_openings"]["bead_gap_mm"] > 0.0
    assert entry["settings_in_gcode"]["support_type"] == "normal(auto)"
    assert entry["settings_in_gcode"]["support_style"] == "snug"


def _two_tool_gcode(path, double_pass: bool) -> None:
    """Synthetic 2-filament G-code: 4 layers; T1 prints a 10 mm outer-wall line on layers 0-2 (out and back when
    ``double_pass``) plus a quarter arc of r 2 mm (G3, I/J), T0 a 20 mm wall every layer; layer 3 has no T1 part but the
    tool order alternates, so its change back to T0 is an 'empty' swap."""
    e_mm = 0.42 * 0.2 / (math.pi / 4 * mpf.FILAMENT_D_MM ** 2)      # E per mm of a 0.42 x 0.2 bead
    L = ["; layer_height = 0.2", "M83", "G90", "T0"]
    tool = 0
    for k in range(4):
        L += ["; CHANGE_LAYER", f"; Z_HEIGHT: {0.2 * (k + 1):.1f}"]
        order = (0, 1) if k % 2 == 0 else (1, 0)
        for t in order:
            if t != tool:
                L.append(f"T{t}")
                tool = t
            if t == 0:
                L += ["; FEATURE: Outer wall", "G0 X0 Y0", "G1 X20 Y0 E1.0"]
            elif k < 3:
                L += ["; FEATURE: Outer wall", "G0 X0 Y10", f"G1 X10 Y10 E{10 * e_mm:.6f}"]
                if double_pass:
                    L.append(f"G1 X0 Y10 E{10 * e_mm:.6f}")
                L += ["G0 X2 Y20", f"G3 X0 Y22 I-2 J0 E{math.pi * e_mm:.6f}"]
            L += ["; FEATURE: Prime tower", "G1 X200 Y200 E0.2"]
    open(path, "w", encoding="utf-8").write("\n".join(L) + "\n")


@pytest.mark.parametrize("double_pass", [False, True])
def test_two_colour_gcode_path_and_inlay_print_check(tmp_path, double_pass):
    """reviewfix F1 / F5 (2026-11-04): parse_gcode_tools measures the part path per tool (lines and G2 / G3 arcs), the
    layers with part, the 'empty' swaps (parity) and dabs; inlay_print_check FAILS a double pass and PASSES one pass."""
    g = tmp_path / "plate_1.gcode"
    _two_tool_gcode(str(g), double_pass)
    r = mpf.parse_gcode_tools(str(g), 1.31)
    one = 10.0 + math.pi                      # per layer: 10 mm line + quarter arc of r 2
    exp = 3 * (one + (10.0 if double_pass else 0.0))
    assert r["part_path_mm_by_tool"]["1"] == pytest.approx(exp, rel=1e-9)
    assert r["part_path_mm_by_tool"]["0"] == pytest.approx(4 * 20.0)
    info = r["part_layers_by_tool"]["1"]
    assert info["layers_with_part"] == 3 and info["empty_layers"] == 1 and info["empty_layer_z_mm"] == [0.8]
    # the inlay solid that prints as ONE pass of `one` mm per layer: V = path x depth x layer height
    depth, lh = 0.42, 0.2
    vol = 3 * one * depth * lh
    ck = mpf.inlay_print_check(r, vol, depth, lh, 1.31)
    assert ck["path_ratio"] == pytest.approx(exp / (3 * one))
    assert ck["g_rel"] == pytest.approx(exp / (3 * one) - 1.0, abs=1e-5)
    assert ck["status"] == ("FAIL" if double_pass else "PASS")
    if double_pass:
        assert ck["path_ratio"] > mpf.INLAY_PATH_RATIO_MAX and ck["g_rel"] > mpf.INLAY_G_REL_TOL
