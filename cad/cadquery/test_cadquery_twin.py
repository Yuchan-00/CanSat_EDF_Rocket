"""Pytest suite for the CadQuery twin (SPEC sections 1-7).

Invariant under test: every component of both presets builds into a valid solid whose
measured geometry (wall thickness, module Z ranges, cavity volume) agrees with Layout, the
report follows the SPEC 6 schema, and every preset passes every section-7 check except the
rows listed in KNOWN_CONTRACT_FAILS (empty since v0.5b; an entry must be pinned by a strict
xfail test so it turns into a failure the moment the contract is fixed).  v0.5: the EDF
tab-mount features (SPEC 3.7 / 5.1: thin liner, screw lugs, counterbore to t_cbore_end, two-box
wire slot, intake wire channel / shelf notch / closed bulge, v0.5d 45 deg floor wedge) are probed on
the solids.  v0.5d (both presets): the tray stops and the blind propulsion joint holes are probed, and
the lip_clamp preset must reproduce the v0.4 solids bit for bit except avionics_module / propulsion_module,
which differ from v0.4 by exactly the analytic tray-stop and hole-floor volumes (plus one contract leak,
the KAIST lip pocket, pinned by a strict xfail).  v0.6 (both presets): payload_module carries the four COM-board
bosses with blind holes - exactly the analytic boss / hole volumes against the same module without the v0.6 params
keys - and the new placeholder ``com`` (Layout box, params mass); the KAIST v0.4 identity test allows exactly those
two changes (KAIST_V06_CHANGED / KAIST_V06_NEW_PARTS) and checks the mass / COM shift they cause; C22 is shown to
FAIL on solids without the bosses, without the holes, with through holes and with the board 1 mm low.
v0.6 servo leads (SPEC 3.8 / 5.1): the route's ribbon proxies are probed on the solids with a negative control (the
module built without the servo cuts blocks them), and the lip_clamp preset carries no route and no lead mass.
v0.6.1 (both presets, SPEC 5.1): the intake's windowed band is window_wall_t thick with radial-sector windows - band,
struts (section, radial side faces), windows (open through, closed right outside) and the measured opening at every
probe radius are probed on the solids; C9 is shown to FAIL with a skin across a window, a 1.2 mm strut or a holed
margin ring; the KAIST v0.4 identity test allows exactly the band change (KAIST_V061_CHANGED: the v0.4 intake is rebuilt
here from primitives, pinned to its v0.4 sha, equal outside the band and analytic inside it).
v0.6.2 (both presets): the truncated-gable lower window ends are probed on the solids independently of C9 (boundary
found by bisection on Solid.isInside at both band surfaces: flat bridge, slope_deg faces, every face >= 35 deg along
the curved wall), the gussets equal Layout's integral, bad window_bottom_end params are rejected, C9 FAILs for square or
wrong-end windows; the ``radio_log`` placeholder (Layout box, params mass, hanging from the payload bulkhead) passes C24,
which FAILs for a misplaced box; the KAIST v0.4 identity allows the gussets (in the band volume), the misc mass 120 ->
100 g (same box) and the new radio_log entry (KAIST_V062_*).
v0.6.3 (both presets, three ATmega128A): every printed part's report entry is bit-identical to the v0.6.2 one
(V062_PRINTED_SHA256 - the user is printing those parts); the placeholder ``fc`` is ``nav`` (the same entry: KAIST_V063_RENAMED)
and the new ``fcu`` board (Layout box, params mass, foam_t above the intake bulkhead) passes C25, which FAILs for a board
moved up / down / sideways or one that covers the vent holes, and is N/A without electronics.fcu; bad electronics.fcu
params are rejected by Layout.
v0.8 (both presets, GPS pod - payload_module.gps_pod): every printed part but payload_module / avionics_module is still the
v0.6.2 entry bit for bit, and those two rebuilt without the GPS features (with_gps=False) are their v0.6.2 entries bit for
bit, so the pod / raceway / pocket (only adds material) and the entry hole (only removes it) are the only changes; the new
printed part gps_pod_lid, the placeholders gps / gps_ground_plane and the lumped gps_lead join the report (the KAIST v0.4
identity allows exactly those: KAIST_V08_*); the pod is probed on the solids (walls, sloped floor, lid seat, air gap, open
sky, bosses, raceway, entry hole), C26 PASSes and is shown to FAIL without the pod, with the lid / patch moved, a blocked
raceway, undrilled bosses or no entry hole; bad gps_pod params are rejected by Layout; the lead route and mass follow Layout.
v0.8.1 (both presets, GNS board - electronics.gns / avionics_module.gns_mount): avionics_module gains four diamond bosses on
the -x tube wall (exactly the clipped prisms beyond the bore, minus the tap holes; analytic boss volume; 45 deg faces on the
solid) and the new placeholder ``gns`` sits on their tips; the GPS lead now ends at the GNS board and the glink lead runs to
the NAV; the GPS pod walls / floor / lid are thinner (params mass_mitigation_v081); the KAIST v0.4 identity allows the
new ``gns`` entry and the bosses (KAIST_V081_NEW_PARTS); C27 PASSes and FAILs for the board off the tips / into them, undrilled
bosses or a module without bosses; bad gns_mount params are rejected by Layout.
Run: ``python -m pytest cad/cadquery/test_cadquery_twin.py -q`` from the project root.
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import build_rocket as br  # noqa: E402
import check_geometry as cg  # noqa: E402
import rocket_geometry as rg  # noqa: E402
from layout import PARAMS_PATH, Layout, load_params  # noqa: E402

# The 66 mm / 70 mm-EDF preset was removed in params v0.2.0 (T/W 0.93: it cannot lift itself).
PRESETS = [None] + list(load_params()["alt_presets"])
PRESET_IDS = [load_params()["meta"]["preset"]] + list(load_params()["alt_presets"])


@pytest.fixture(scope="session", params=PRESETS, ids=PRESET_IDS)
def built(request):
    L = rg.layout_for(request.param)
    parts = rg.build_all(L)
    report = br.make_report(L, parts)
    return request.param, L, parts, report


def _section_thickness(shape, r_in: float, r_out: float, z: float, h: float = 1.0, theta: float = 0.0) -> float:
    """Radial material thickness of ``shape`` along the radial direction theta (default +X) at height z (bbox of a
    thin probe; the shape is turned by -theta so the probe always runs along +X).

    The probe is 0.1 mm wide so the bore's curvature sag (w^2 / 8 r ~ 2e-5 mm) is negligible;
    for tapered walls (nose) pass a small ``h`` because the bbox spans r(z)..r(z + h).
    """
    if theta:
        shape = shape.rotate(rg.V(0, 0, 0), rg.V(0, 0, 1), -theta)
    probe = rg._box(r_in - 5.0, r_out + 5.0, -0.05, 0.05, z, z + h)
    common = shape.intersect(probe)
    bb = common.BoundingBox()
    return bb.xmax - bb.xmin


# --------------------------------------------------------------------------- #
def test_every_component_is_a_valid_solid(built):
    _, L, parts, _ = built
    # v0.6: + com; v0.6.2: + radio_log; v0.6.3: + fcu; + servo_leads; v0.8: + gps_pod_lid, gps, gps_ground_plane, gps_lead;
    # v0.8.1: + gns; v0.9 (direct drive): 4 horns / 4 arms / 4 pushrods -> 4 couplers / 4 covers / 4 horn discs (same
    # count), + wire_cover (open motor-lead groove), + motor_leads (lumped)
    # v0.9 fix round: + 4 brass hinge bushings (direct drive)
    # livery (2026-11-04): + the orange inlay part avionics_logo (one solid per island and copy)
    assert len(parts) == (43 + (1 if L.servo_leads else 0) + (4 if L.has_gps_pod else 0) + (1 if L.has_gns else 0)
                          + (1 if getattr(L, "wire_ch_open", False) else 0) + (1 if getattr(L, "has_motor_leads", False) else 0)
                          + (L.vane_count if getattr(L, "dd_bushing", False) else 0)
                          + (1 if getattr(L, "has_livery", False) else 0))
    for name, p in parts.items():
        if p.kind == "lumped":
            assert p.shape is None and p.mass_g > 0
            continue
        assert p.shape is not None, name
        assert p.shape.isValid(), f"{name} is not a valid shape"
        assert p.shape.Volume() > 0.0, name
        n_solids = len(p.shape.Solids())
        if name == "edf_unit":
            assert n_solids == 2, "EDF placeholder = lip+housing and a separate hub"
        elif name == "gps":
            assert n_solids == 2, "v0.8 GPS placeholder = patch and board (stacked, apart)"
        elif name == "avionics_logo":
            assert n_solids == len(L.livery_thetas) * len(L.livery_polys_vz), "livery: one solid per island and copy"
        else:
            assert n_solids == 1, f"{name} splits into {n_solids} solids"


def test_printed_part_names_match_layout(built):
    _, L, parts, _ = built
    bases = {p.base for p in parts.values() if p.kind == "printed"}
    assert bases == set(L.printed_parts)
    for base in L.printed_parts:
        assert base in rg.PRINTED_BUILDERS and base in rg.PRINT_ORIENTATION


def test_report_schema(built):
    _, L, parts, report = built
    for key in ("generator", "generator_version", "preset", "params_sha256", "parts", "totals", "derived", "warnings"):
        assert key in report, key
    assert report["generator"] == "cadquery"
    assert report["preset"] == L.p["meta"]["preset"]
    part_keys = {"kind", "volume_mm3", "density_g_cm3", "mass_g", "bbox_mm", "com_mm", "inertia_about_com_kg_m2"}
    for name, e in report["parts"].items():
        assert set(e) == part_keys, name
        assert e["kind"] in ("printed", "placeholder", "lumped")
        assert len(e["bbox_mm"]) == 2 and len(e["bbox_mm"][0]) == 3
        assert len(e["com_mm"]) == 3
        assert len(e["inertia_about_com_kg_m2"]) == 3 and all(len(r) == 3 for r in e["inertia_about_com_kg_m2"])
    for key in ("mass_g", "printed_mass_g", "com_mm", "inertia_about_com_kg_m2", "post_ejection"):
        assert key in report["totals"], key
    for key in ("mass_g", "com_mm", "inertia_about_com_kg_m2", "descent_mass_g"):
        assert key in report["totals"]["post_ejection"], key
    for key in ("pushrod_len_mm", "nose_cavity_volume_L", "footprint_d_mm", "vehicle_h_mm", "inlet_area_ratio"):
        assert key in report["derived"], key
    for name in ("edf_unit", "battery", "esc", "nav", "fcu", "misc_electronics", "com", "radio_log", "servo_release", "cansat_payload",
                 "spring", "chute_pack", "joint_screws_inserts",
                 "vane_drive_hardware" if L.direct_drive else "pushrods_horns_clevises", "launch_pad", "latch_arm"):
        assert name in report["parts"], name
    for i in range(1, 5):   # v0.9 direct drive: couplers, covers and horn discs instead of horns, arms and pushrods
        for stem in (("vane", "vane_coupler", "servo_cover", "servo_vane", "servo_horn") if L.direct_drive else
                     ("vane", "vane_horn", "servo_vane", "servo_arm", "pushrod")):
            assert f"{stem}_{i}" in report["parts"]
    if L.direct_drive:
        assert {"wire_cover", "motor_leads"} <= set(report["parts"])
        assert set(report["extra"]["direct_drive"]) >= {"hinge_moment_kgcm", "servo_torque_kgcm", "fairing_u_mm"}
        assert set(report["extra"]["mass_model"]) >= {"mode", "table", "parts", "printed_mass_g"}
    if L.has_gps_pod:   # v0.8
        for name in ("gps_pod_lid", "gps", "gps_ground_plane", "gps_lead"):
            assert name in report["parts"], name
        assert set(report["extra"]["gps"]) >= {"added_mass_g", "added_total_g", "antenna_phase_centre_mm",
                                                "antenna_lever_arm_from_cg_mm", "noise_source_distance_mm"}
    if L.has_gns:       # v0.8.1
        assert "gns" in report["parts"]
        assert set(report["extra"]["gns"]) >= {"added_mass_g", "added_total_g", "board_box_mm", "distance_mm",
                                                "pod_mitigation", "thrust_to_weight_nominal_cad", "glink_lead_len_mm"}


def test_params_sha256_matches_file(built):
    _, _, _, report = built
    with open(PARAMS_PATH, "rb") as f:
        assert report["params_sha256"] == hashlib.sha256(f.read()).hexdigest()


def test_tube_wall_thickness_equals_body_wall(built):
    _, L, parts, _ = built
    wall = L.p["body"]["wall"]
    checks = {
        "avionics_module": L.z_avionics_bottom + 0.5 * (L.z_rail_bottom - L.z_avionics_bottom) + 2.0,  # plain tube below the rails
        "payload_module": L.z_envelope_bottom + 20.0,                                                    # plain tube beside the envelope
        "intake_module": L.z_intake_bottom + 2.0,                                                        # skirt (below the screw holes)
    }
    # tabs: the intake skirt is probed opposite the wire channel (its bulge reaches down to wire_bulge_z0)
    theta = {"intake_module": L.wire_theta + 180.0 if L.tabs else 0.0}
    for name, z in checks.items():
        t = _section_thickness(parts[name].shape, L.body_id / 2, L.body_r, z, theta=theta.get(name, 0.0))
        assert abs(t - wall) < 1e-3, f"{name} wall {t:.4f} != {wall}"
    # propulsion duct wall = (vane_section_od - duct_id)/2, measured between the pods / bosses (v0.9 direct drive: between
    # the fins, clear of the lead pipes, at 45 deg)
    t = _section_thickness(parts["propulsion_module"].shape, L.duct_r, L.vane_section_r, L.z_vane_section_top - 15.0,
                           theta=45.0 if L.direct_drive else 0.0)
    assert abs(t - (L.vane_section_od - L.duct_id) / 2) < 1e-3
    # nose shell: constant wall along the surface NORMAL (what Fusion's ShellFeature makes), so the
    # radial thickness at height z is wall * sqrt(1 + (dr/dz)^2)
    for frac in (0.25, 0.5, 0.75):
        x = frac * L.nose_len
        z = L.z_nose_base + x
        r_out = L.nose_outer_radius(x)
        hx = 1e-3 * L.nose_len
        drdz = (L.nose_outer_radius(x + hx) - L.nose_outer_radius(x - hx)) / (2 * hx)
        expected = L.nose_wall * math.sqrt(1.0 + drdz * drdz)
        t = _section_thickness(parts["nose_cone"].shape, r_out - 2 * expected, r_out, z, h=0.01)
        assert abs(t - expected) < 0.03, f"nose radial wall {t:.3f} at z={z:.1f}, expected {expected:.3f}"


def test_module_z_ranges_match_layout(built):
    _, L, parts, _ = built
    for name, (z0, z1) in L.module_z_ranges().items():
        bb = parts[name].shape.BoundingBox()
        assert abs(bb.zmin - z0) < 1e-6, f"{name} zmin {bb.zmin} != {z0}"
        assert abs(bb.zmax - z1) < 1e-6, f"{name} zmax {bb.zmax} != {z1}"


def test_vane_and_horn_instances_are_identical(built):
    _, L, parts, report = built
    stems = (("vane", "vane_coupler", "servo_cover", "servo_vane", "servo_horn") if L.direct_drive else
             ("vane", "vane_horn", "servo_vane", "servo_arm", "pushrod"))
    for stem in stems:
        masses = [report["parts"][f"{stem}_{i + 1}"]["mass_g"] for i in range(L.vane_count)]
        assert max(masses) - min(masses) < 1e-6 * max(masses), stem
        radii = [math.hypot(*report["parts"][f"{stem}_{i + 1}"]["com_mm"][:2]) for i in range(L.vane_count)]
        assert max(radii) - min(radii) < 1e-6, stem


def test_totals_are_sums_of_parts(built):
    _, L, parts, report = built
    e = report["parts"]
    vehicle = [n for n, p in parts.items() if p.in_vehicle]
    assert "launch_pad" not in vehicle
    assert abs(report["totals"]["mass_g"] - sum(e[n]["mass_g"] for n in vehicle)) < 1e-6
    assert abs(report["totals"]["printed_mass_g"] - sum(e[n]["mass_g"] for n in vehicle if parts[n].kind == "printed")) < 1e-6
    post = report["totals"]["post_ejection"]
    excluded = sum(e[n]["mass_g"] for n in rg.POST_EJECTION_EXCLUDED)
    assert abs(post["mass_g"] - (report["totals"]["mass_g"] - excluded)) < 1e-6
    assert abs(post["descent_mass_g"] - (post["mass_g"] + e["chute_pack"]["mass_g"])) < 1e-6
    # placeholder masses come from params; printed masses from density x measured volume (mass model 'solid') or, v0.9
    # (printer.mass_model 'slicer_part_g'), from the slicer table / class fallback recorded in extra.mass_model
    assert abs(e["edf_unit"]["mass_g"] - L.p["edf"]["mass_g"]) < 1e-9
    assert abs(e["cansat_payload"]["mass_g"] - L.p["payload"]["mass_g"]) < 1e-9
    rho = L.p["printer"]["density_g_cm3"]
    mm = report["extra"]["mass_model"]
    for n in vehicle:
        if parts[n].kind != "printed":
            continue
        rec = mm["parts"][n]
        assert abs(e[n]["mass_g"] - rec["mass_g"]) < 1e-9, n
        assert abs(rec["solid_mass_g"] - rho * e[n]["volume_mm3"] / 1000.0) < 1e-9, n
        assert abs(e[n]["density_g_cm3"] * e[n]["volume_mm3"] / 1000.0 - e[n]["mass_g"]) < 1e-9, n
    if mm["mode"] == "solid":
        assert abs(e["piston"]["mass_g"] - rho * e["piston"]["volume_mm3"] / 1000.0) < 1e-9


def test_inertia_tensors_are_physical(built):
    _, _, _, report = built
    for name, e in report["parts"].items():
        if e["kind"] == "lumped":
            continue
        m = e["inertia_about_com_kg_m2"]
        for i in range(3):
            assert m[i][i] > 0.0, name
            for j in range(3):
                assert abs(m[i][j] - m[j][i]) < 1e-12, name
        d = [m[0][0], m[1][1], m[2][2]]
        assert d[0] + d[1] >= d[2] * (1 - 1e-9) and d[1] + d[2] >= d[0] * (1 - 1e-9) and d[0] + d[2] >= d[1] * (1 - 1e-9), name
    t = report["totals"]["inertia_about_com_kg_m2"]
    assert t[2][2] < t[0][0] and t[2][2] < t[1][1]            # slender vehicle: roll inertia smallest


def test_measured_derived_values_match_layout(built):
    _, L, parts, report = built
    d = report["derived"]
    assert abs(d["nose_cavity_volume_L"] - L.nose_cavity_volume_L) / L.nose_cavity_volume_L < 0.01
    # circumscribed footprint = fin tip corners (tip radius and half the fin thickness)
    assert abs(d["footprint_d_mm"] - 2.0 * math.hypot(L.fin_tip_r, L.fin_t / 2)) < 1e-6
    assert d["footprint_d_mm"] - L.footprint_d < 0.05
    assert abs(d["vehicle_h_mm"] - L.vehicle_h) < 1e-6
    if L.direct_drive:      # v0.9: no pushrods
        assert d["pushrod_len_mm"] == 0.0
    else:
        assert d["pushrod_len_mm"] > 0.0
        assert d["pushrod_len_mm"] == pytest.approx(rg.pushrod_length(L, 0))
    assert d["inlet_area_ratio"] == L.inlet_area_ratio
    # v0.6.1: the opening of the built band, measured by probe shells across it, is the radial-sector design at every
    # probe radius - v0.6.2 minus the gussets: n x Layout.window_open_area_mm2(r) (n x window_angle x r x window_h less
    # the gable); the minimum is the innermost probe, INLET_PROBE_GAP + T/2 outside r_band_in, and Layout's minimum is
    # at r_band_in itself (sampled across the band)
    ex = report["extra"]
    wa = math.radians(L.window_angle_deg)
    probes = ex["inlet_opening_probes_mm2"]
    assert len(probes) == rg.INLET_PROBE_COUNT
    for r, a in probes:
        assert a == pytest.approx(L.strut_count * L.window_open_area_mm2(r), rel=1e-6), (r, a)
        if L.has_window_gable:                                   # the gussets take area at every radius
            assert a < L.strut_count * wa * r * L.window_h - L.strut_count * 100.0
    r0 = probes[0][0]
    assert r0 == pytest.approx(L.r_band_in + rg.INLET_PROBE_GAP + rg.INLET_PROBE_T / 2)
    assert ex["inlet_area_mm2_measured"] == min(a for _, a in probes) == probes[0][1]
    assert ex["inlet_area_ratio_measured"] == pytest.approx(
        L.strut_count * L.window_open_area_mm2(r0) / L.fan_annulus_area_mm2, rel=1e-6)
    assert ex["inlet_area_mm2_measured_outer"] == probes[-1][1]
    assert L.inlet_area_r_min == L.r_band_in
    assert L.inlet_area_mm2 == pytest.approx(L.strut_count * L.window_open_area_mm2(L.r_band_in))
    assert L.inlet_area_outer_mm2 == pytest.approx(L.strut_count * L.window_open_area_mm2(L.body_r))
    assert L.inlet_area_square_mm2 == pytest.approx(L.strut_count * wa * L.r_band_in * L.window_h)
    assert L.inlet_area_ratio >= L.p["intake_module"]["min_inlet_area_ratio"]
    # v0.6.2: the gussets measured on the built intake are Layout's integral
    wg = ex["window_gable"]
    assert wg["gusset_volume_mm3_measured"] == pytest.approx(L.gusset_volume_mm3, rel=1e-6)
    assert wg["gusset_volume_mm3_layout"] == L.gusset_volume_mm3 > 0.0


def test_vane_deflection_sign_convention(built):
    """+delta moves the trailing edge toward +e_theta (SPEC 8.2)."""
    _, L, _, _ = built
    te_box = rg._box(L.vane_r_in, L.vane_r_out, -L.vane_t, L.vane_t, L.vane_te_z - 1.0, L.vane_te_z + 2.0)
    v = rg.vane(L, 0, +L.p["vane"]["max_deflection_mech_deg"])
    te = v.intersect(rg._box(L.vane_r_in, L.vane_r_out, -20, 20, 0, L.vane_hinge_z - 0.5 * L.vane_chord * 0.5))
    assert te.Center().y > 0.5, "trailing edge did not move toward +Y for vane 1 at +delta"
    del te_box


def test_stl_export_is_binary_and_watertight(tmp_path):
    """Every printed part exports to a binary STL that is watertight and within 0.1 % of the exact volume."""
    import struct

    import trimesh

    L = rg.layout_for(None)
    parts = rg.build_all(L)
    report = br.make_report(L, parts)
    files = br.export_stl(parts, str(tmp_path))
    assert set(files) == set(L.printed_parts)
    for base, path in files.items():
        with open(path, "rb") as f:
            header, n_tris = f.read(80), struct.unpack("<I", f.read(4))[0]
        assert not header[:5].lower().startswith(b"solid"), f"{base}: ASCII STL"
        assert os.path.getsize(path) == 84 + 50 * n_tris, f"{base}: binary size mismatch"
        mesh = trimesh.load(path, force="mesh")
        assert mesh.is_watertight, f"{base}: STL not watertight"
        key = base if base in report["parts"] else f"{base}_1"
        exact = report["parts"][key]["volume_mm3"]
        assert abs(mesh.volume / exact - 1.0) < 1e-3, f"{base}: mesh volume {mesh.volume:.1f} vs exact {exact:.1f}"


def test_output_paths_use_preset_suffix():
    base = br.output_paths(None)
    assert base["report"].endswith("cadquery_report.json") and base["stl_dir"].endswith("stl")
    alt = br.output_paths("SOME_PRESET")
    assert alt["report"].endswith("cadquery_report_SOME_PRESET.json")
    assert alt["stl_dir"].endswith("stl_SOME_PRESET")
    assert alt["step"].endswith("CanSatEDFRocket_SOME_PRESET.step")


def test_both_presets_build_without_exceptions():
    for preset in PRESETS:
        parts = rg.build_all(rg.layout_for(preset))
        assert all(p.shape.isValid() for p in parts.values() if p.shape is not None)


@pytest.fixture(scope="session")
def checks():
    out = {}
    for preset, pid in zip(PRESETS, PRESET_IDS):
        L, parts, report = br.build(preset, export=False, verbose=False)
        out[pid] = {c.id: c for c in cg.Checker(L, parts, report).run()}
    return out


# SPEC 7 rows that FAIL because of the params/layout contract, not because of the twin.  An entry must be
# pinned by a strict xfail test and removed when the contract is fixed.  v0.5b: empty - C13 (static margin
# 0.508 cal, WARN) and C20 (12 x 5 bundle clear, 6.09 mm gates) no longer fail.
KNOWN_CONTRACT_FAILS: dict[str, set[str]] = {}
TAB_ROWS = ("C18", "C19", "C20", "C21", "C23")    # v0.6: C23 (servo leads) runs with the tab mount's route


def test_every_preset_passes_every_spec_check(checks):
    for pid, rows in checks.items():
        assert len([c for c in rows if c.startswith("C")]) == 32, pid      # v0.6: C22, C23; v0.6.2: C24; v0.6.3: C25; v0.8: C26;
        #                                                                    v0.8.1: C27; v0.9: C28 (coupler), C29 (bullets);
        #                                                                    v0.9 fix round: C33 (lateral CG); wire09c: C34
        #                                                                    (clock coverage), C35 (axial retention)
        assert rows["C33"].status == cg.PASS, (pid, rows["C33"].measured)  # fix round: standing trim <= 5 % of the authority
        assert rows["C26"].status == cg.PASS, (pid, rows["C26"].measured)  # params v0.8 carry the GPS pod
        assert rows["C27"].status == cg.PASS, (pid, rows["C27"].measured)  # params v0.8.1 carry the GNS board
        assert rows["C22"].status == cg.PASS, (pid, rows["C22"].measured)  # params v0.6 carry the COM board
        assert rows["C24"].status == cg.PASS, (pid, rows["C24"].measured)  # params v0.6.2 carry the radio/log box
        assert rows["C25"].status == cg.PASS, (pid, rows["C25"].measured)  # params v0.6.3 carry the FCU board
        assert rows["C9"].values["gable"]["ok"], (pid, rows["C9"].measured)  # v0.6.2 gable evidence present
        known = KNOWN_CONTRACT_FAILS.get(pid, set())
        fails = [f"{pid} {c.id}: {c.measured}" for c in rows.values() if c.status == cg.FAIL and c.id not in known]
        assert not fails, fails
        warn_ids = {c.id for c in rows.values() if c.status == cg.WARN}
        assert warn_ids <= {"C10", "C12", "C13", "X1", "X2"}, (pid, warn_ids)


def test_tab_rows_follow_the_mount_mode(checks):
    """C18-C21 and C23 run (PASS/FAIL) for edf.mount = tabs and are N/A, never FAIL, for lip_clamp.  wire09c: with the
    clock mount (no tab screws) C19 is N/A with its reason and C34 / C35 run; without it C34 / C35 are N/A."""
    for preset, pid in zip(PRESETS, PRESET_IDS):
        L = rg.layout_for(preset)
        tabs, clock = L.tabs, getattr(L, "clock_mount", False)
        rows = [c for c in TAB_ROWS if not (clock and c == "C19")]
        for cid in rows:
            st = checks[pid][cid].status
            assert (st != cg.NA) if tabs else (st == cg.NA), (pid, cid, st)
        if tabs:
            assert {checks[pid][c].status for c in rows} == {cg.PASS}, pid
        if clock:
            assert checks[pid]["C19"].status == cg.NA and "C35" in checks[pid]["C19"].measured
            assert checks[pid]["C34"].status == cg.PASS and checks[pid]["C35"].status == cg.PASS, pid
        else:
            assert checks[pid]["C34"].status == checks[pid]["C35"].status == cg.NA


def test_c13_static_margin_is_reported_honestly(checks):
    """C13 grades the measured margin with the SPEC thresholds (no loosening): >= 1.0 PASS, >= 0.5 WARN."""
    for pid, rows in checks.items():
        c = rows["C13"].values
        assert c["margin_cal"] == pytest.approx((c["z_cg"] - c["z_cp"]) / rg.layout_for(
            None if pid == load_params()["meta"]["preset"] else pid).body_od)
        expect = cg.PASS if c["margin_cal"] >= 1.0 else (cg.WARN if c["margin_cal"] >= 0.5 else cg.FAIL)
        if c["cg_above_hinge"] < 250.0:
            expect = cg.FAIL
        assert rows["C13"].status == expect, (pid, c["margin_cal"])


# --------------------------------------------------------------------------- #
# v0.5 EDF tab mount (SPEC 3.7, 5.1, 6) - probed on the solids of the tab-mount preset
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="session")
def tabs_model():
    """The v0.5 SCREW tab mount (lugs, tab screws, two-box wire slot, lead notch).  wire09c: the default preset uses the
    clock mount (propulsion_module.tab_mount 'clock_keyways'); the screw mount stays a supported mode, so these probes run
    on the default params with tab_mount 'screws' (clock_model below has the clock-mount probes)."""
    for preset in PRESETS:
        p = load_params(preset=preset) if preset else load_params()
        if p["edf"].get("mount") == "tabs":
            p["propulsion_module"]["tab_mount"] = "screws"
            L = Layout(p)
            return L, rg.build_all(L)
    pytest.skip("no preset uses edf.mount = 'tabs'")


def _radial_material(shape, L, theta: float, z: float, r0: float, r1: float, v: float = 0.0, h: float = 0.1):
    """(r_min, r_max) of every solid of ``shape`` inside a thin radial probe at theta, sorted by r."""
    common = shape.intersect(rg._box_polar(r0, r1, v - 0.05, v + 0.05, z, z + h, theta))
    out = []
    for s in common.Solids():
        rs = [math.hypot(p.X, p.Y) for p in s.Vertices()]
        out.append((min(rs), max(rs)))
    return sorted(out)


def _probe_free(shape, probe) -> bool:
    return rg.intersection_volume(probe, shape) < 1e-6


class _approx_segs:
    """``segments == _approx_segs(expected, abs=tol)``: same number of (r_min, r_max) segments, each end within tol
    (pytest.approx does not compare lists of tuples)."""

    def __init__(self, expected, abs: float):  # noqa: A002 - mirrors pytest.approx
        self.expected, self.tol = [tuple(e) for e in expected], abs

    def __eq__(self, other) -> bool:
        other = [tuple(o) for o in other]
        return len(other) == len(self.expected) and all(
            len(o) == len(e) and all(abs(a - b) <= self.tol for a, b in zip(o, e)) for o, e in zip(other, self.expected))

    def __repr__(self) -> str:
        return f"_approx_segs({self.expected}, abs={self.tol})"


def test_mount_mode_per_preset():
    modes = {pid: rg.layout_for(p).edf_mount for p, pid in zip(PRESETS, PRESET_IDS)}
    assert modes["KAIST_90mm_12S"] == "lip_clamp"
    assert modes[load_params()["meta"]["preset"]] in ("tabs", "lip_clamp")
    for p in PRESETS:
        L = rg.layout_for(p)
        if not L.tabs:   # lip_clamp: no cylindrical section, the cone ends at the spigot bottom
            assert L.cone_z1 == L.z_prop_spigot_bottom and L.z_liner_top is None


def _lug_probe_zs(L) -> list[float]:
    """One Z per screw lug, block_margin_z/2 above its bottom: inside the lug, outside every counterbore band
    (each counterbore keeps block_margin_z to its lug's ends) and below the deck."""
    dz = L.p["propulsion_module"]["tab_screw"]["block_margin_z"] / 2
    return [z0 + dz for z0, _ in L.boss_z_ranges]


def _shell_volume(shell) -> float:
    """Volume enclosed by a closed shell (sign depends on the shell orientation)."""
    import cadquery as cq
    return cq.Solid.makeSolid(shell).Volume()


def _lug_gap_zs(L) -> list[float]:
    """Mid-Z of every gap between two screw lugs."""
    r = L.boss_z_ranges
    return [0.5 * (r[i][1] + r[i + 1][0]) for i in range(len(r) - 1)]


def test_keyways_are_open_to_the_top_and_have_a_floor(tabs_model):
    L, parts = tabs_model
    prop = parts["propulsion_module"].shape
    e = 0.1
    for th in L.tab_thetas:
        channel = rg._box_polar(L.keyway_r_in + e, L.keyway_r_out - e, -L.keyway_w / 2 + e, L.keyway_w / 2 - e,
                                L.keyway_z0 + e, L.z_prop_top + rg.OVERLAP, th)
        assert _probe_free(prop, channel), f"keyway at {th} deg is blocked"
        # the lowest lug forms the floor: material right under the whole tab footprint outside the bore
        floor = rg._box_polar(L.edf_bore_r + e, L.tab_r_out, -L.tab_t / 2, L.tab_t / 2, L.keyway_z0 - 0.5, L.keyway_z0, th)
        assert rg.intersection_volume(floor, prop) == pytest.approx(
            0.5 * L.tab_t * (L.tab_r_out - L.edf_bore_r - e), rel=1e-6), f"keyway floor incomplete at {th} deg"
        mat = _radial_material(prop, L, th, L.block_z0 + 0.5 * (L.keyway_z0 - L.block_z0), L.duct_r, L.body_r + 1.0)
        assert mat == _approx_segs([(L.edf_bore_r, L.body_r)], abs=1e-3), mat
        # in a lug the keyway stops at keyway_r_out (lug + skin beyond it) ...
        for z in _lug_probe_zs(L):
            mat = _radial_material(prop, L, th, max(z, L.keyway_z0 + e), L.keyway_r_in - 1.0, L.body_r + 1.0)
            assert mat == _approx_segs([(L.keyway_r_out, L.body_r)], abs=1e-3), (z, mat)
        # ... between the lugs it opens into the sleeve-skin cavity (skin only); in the spigot ring the ring
        # outside it stays
        for z in _lug_gap_zs(L):
            mat = _radial_material(prop, L, th, z, L.keyway_r_in - 1.0, L.body_r + 1.0)
            assert mat == _approx_segs([(L.body_r - L.cone_wall, L.body_r)], abs=1e-3), (z, mat)
        mat = _radial_material(prop, L, th, 0.5 * (L.z_prop_spigot_bottom + L.z_prop_spigot_top), L.keyway_r_in - 1.0,
                               L.body_r + 1.0)
        assert mat == _approx_segs([(L.keyway_r_out, L.spigot_od / 2)], abs=1e-3), mat


def test_screw_lugs_fill_sleeve_to_skin(tabs_model):
    L, parts = tabs_model
    prop = parts["propulsion_module"].shape
    two = [(L.edf_bore_r, L.sleeve_r), (L.body_r - L.cone_wall, L.body_r)]   # sleeve | cavity | skin
    for th in L.tab_thetas:
        assert len(rg.tab_screw_lugs(L, th)) == len(L.boss_z_ranges)
        lo, hi = rg.tab_boss_thetas(L, th)
        assert hi - lo == pytest.approx(L.block_theta_far + L.block_theta_head)
        for z in _lug_probe_zs(L):
            for a in (lo + 1.0, hi - 1.0):       # inside the sector: one solid bore -> skin
                mat = _radial_material(prop, L, a, z, L.duct_r, L.body_r + 1.0)
                assert mat == _approx_segs([(L.edf_bore_r, L.body_r)], abs=0.01), (a, z, mat)
            for a in (lo - 2.0, hi + 2.0):       # outside the sector: sleeve and skin with the cavity between
                mat = _radial_material(prop, L, a, z, L.duct_r, L.body_r + 1.0)
                assert mat == _approx_segs(two, abs=0.01), (a, z, mat)
        for z in _lug_gap_zs(L):                 # the lugs are separate: sleeve and skin between them
            for a in (lo + 1.0, hi - 1.0):
                mat = _radial_material(prop, L, a, z, L.duct_r, L.body_r + 1.0)
                assert mat == _approx_segs(two, abs=0.01), (a, z, mat)


def test_tab_screw_tunnels_exit_the_skin_and_tap_holes_have_material(tabs_model):
    L, parts = tabs_model
    prop, edf = parts["propulsion_module"].shape, parts["edf_unit"].shape
    skin = rg._tube(L.body_r, L.body_r - L.cone_wall, L.z_cyl0, L.z_prop_spigot_bottom)
    t_end = L.tab_screw_t_cbore_end + cg.C19_TUNNEL_PAST_END
    # the counterbore's edge nearest the axis has left the skin before the cutter ends
    assert math.hypot(L.tab_screw_r - L.tab_screw_d_cbore / 2, L.tab_screw_t_cbore_end) > L.body_r
    t0, t1 = L.tab_screw_t_tap_end + cg.C19_PROBE_INSET, -L.tab_screw_t_keyway
    annulus = math.pi / 4 * (cg.C19_TAP_MAT_PROBE_D ** 2 - L.tab_screw_d_tap ** 2) * (t1 - t0)
    for th in L.tab_thetas:
        for z in L.tab_screw_zs:
            tunnel = rg.tab_screw_probe(L, th, z, cg.C19_TUNNEL_PROBE_D, L.tab_screw_t_head + 0.2, t_end)
            assert _probe_free(prop, tunnel)
            assert rg.intersection_volume(tunnel, skin) > 10.0, "the tunnel must run through where the skin is"
            x, y, _ = L.tab_screw_point(th, z, t_end)
            assert math.hypot(x, y) > L.body_r + 1.0
            # the full counterbore (O d_cbore less 0.05) is open from the seat to the outside: no skin brow
            cb = rg.tab_screw_probe(L, th, z, L.tab_screw_d_cbore - 0.05, L.tab_screw_t_head + 0.05, t_end)
            assert _probe_free(prop, cb), "counterbore blocked (skin brow)"
            seat = rg.tab_screw_probe(L, th, z, cg.C19_TUNNEL_PROBE_D, L.tab_screw_t_head - 1.0, L.tab_screw_t_head - 0.2)
            assert rg.intersection_volume(seat, prop) > 1.0, "no counterbore floor (head seat)"
            clear = rg.tab_screw_probe(L, th, z, L.tab_screw_d_clear - 0.05, L.tab_screw_t_keyway, L.tab_screw_t_head)
            assert _probe_free(prop, clear)
            assert _probe_free(prop, rg.tab_screw_probe(L, th, z, cg.C19_TAP_FREE_PROBE_D, t0, t1))
            mat = rg.intersection_volume(rg.tab_screw_probe(L, th, z, cg.C19_TAP_MAT_PROBE_D, t0, t1), prop)
            assert mat >= cg.C19_TAP_MAT_FRACTION * annulus and mat == pytest.approx(annulus, rel=1e-3)
            slot = rg.tab_screw_probe(L, th, z, cg.SHANK_PROBE_D, -L.tab_t / 2, L.tab_t / 2)
            assert _probe_free(edf, slot), "screw axis misses the tab slot"
            ctrl = rg.tab_screw_probe(L, th, z, L.tab_slot_w + cg.C19_SLOT_CONTROL_EXTRA_D, -L.tab_t / 2, L.tab_t / 2)
            assert rg.intersection_volume(ctrl, edf) > 1.0, "positive control: the tab is not on the screw axis"


def test_outer_skin_profile_and_liner(tabs_model):
    L, parts = tabs_model
    prop = parts["propulsion_module"].shape
    th = L.wire_theta + 180.0          # opposite the wire slot: no lug / web / slot there (asserted here)

    def ang(a: float, b: float) -> float:
        return abs((a - b + 180.0) % 360.0 - 180.0)

    assert ang(th, L.wire_theta) > 90.0
    for t in L.tab_thetas:
        lo, hi = rg.tab_boss_thetas(L, t)
        assert ang(th, 0.5 * (lo + hi)) > 0.5 * (hi - lo) + 2.0, ("probe angle inside a lug sector", th, t)
    for w in L.web_thetas:
        assert ang(th, w) > 5.0, ("probe angle on a web", th, w)
    for z in (L.z_cyl0 + 2.0, 0.5 * (L.z_cyl0 + L.z_prop_spigot_bottom), L.z_prop_spigot_bottom - L.body_wall - 2.0):
        mat = _radial_material(prop, L, th, z, L.duct_r, L.body_r + 1.0)
        assert abs(mat[-1][1] - L.body_r) < 0.01 and abs(mat[-1][0] - (L.body_r - L.cone_wall)) < 0.01, (z, mat)
    z = 0.5 * (L.cone_z0 + L.cone_z1)
    mat = _radial_material(prop, L, th, z, L.duct_r, L.body_r + 1.0)
    assert abs(mat[-1][1] - L.cone_outer_radius(z)) < 0.05, (z, mat)
    # thin liner: duct tube r [duct_r, liner_r_out], the cavity behind it, then the sleeve
    z = 0.5 * (L.z_edf_shoulder + L.z_liner_ring_bottom)
    mat = _radial_material(prop, L, th, z, L.duct_r - 2.0, L.body_r + 1.0)
    assert mat[:2] == _approx_segs([(L.duct_r, L.liner_r_out), (L.edf_bore_r, L.sleeve_r)], abs=0.01), mat
    # top ring r [duct_r, sleeve_r] over [z_liner_ring_bottom, z_liner_top]
    z = 0.5 * (L.z_liner_ring_bottom + L.z_liner_top) - 0.05
    mat = _radial_material(prop, L, th, z, L.duct_r - 2.0, L.body_r + 1.0)
    assert mat[0] == pytest.approx((L.duct_r, L.sleeve_r), abs=0.01), mat
    z = L.z_liner_top + 0.5 * (L.z_edf_housing_bottom - L.z_liner_top)  # above the liner: the EDF bore
    mat = _radial_material(prop, L, th, z, L.duct_r - 2.0, L.body_r + 1.0)
    assert abs(mat[0][0] - L.edf_bore_r) < 0.01, mat
    # the cavity behind the liner is closed: an inner shell enclosing exactly that annulus
    cavity = math.pi * (L.edf_bore_r ** 2 - L.liner_r_out ** 2) * (L.z_liner_ring_bottom - L.z_edf_shoulder)
    inner = [abs(_shell_volume(sh)) for sh in prop.Shells()]
    if getattr(L, "ml_low_cut_uz", None):
        # v0.9 lead-exit round (2026-10-29, HARDWARE contract change): the motor-lead notch (rg.wire_low_cut) cuts through the
        # liner at wire_theta on purpose, so the cavity behind it is open there and only there - no inner shell, and the
        # liner tube and top ring are complete everywhere outside the notch
        assert not any(abs(v - cavity) < 1e-3 * cavity for v in inner), (cavity, inner)
        notch = rg.wire_low_cut(L)
        for r0, r1, z0, z1 in ((L.duct_r, L.liner_r_out, L.z_edf_shoulder, L.z_liner_ring_bottom),
                               (L.duct_r, L.sleeve_r, L.z_liner_ring_bottom, L.z_liner_top)):
            ring = rg._tube(r1 - 0.05, r0 + 0.05, z0 + 0.05, z1 - 0.05).cut(notch)
            missing = ring.cut(prop).Volume()
            assert missing < 1e-3, (r0, r1, z0, z1, missing)
    else:
        assert any(abs(v - cavity) < 1e-3 * cavity for v in inner), (cavity, inner)


def _open_motor_groove_asserts(L, prop, intake, edf, cover) -> None:
    """v0.9 (intake wire_channel.open, user decision 3): the three-lead proxies are clear along the whole path; the
    propulsion slot is as v0.5d but 22 wide; the intake groove is open outward over its whole height (cover off), the
    lip-pocket wall stays around the bell, the gate over the lip and the bulkhead hole are open, the wire_cover seats on
    the side walls; the bullets pass straight up through the intake (C29's proxy) and up the covered groove."""
    for k, leg in rg.motor_lead_legs(L).items():
        for nm, s in (("prop", prop), ("intake", intake), ("edf", edf), ("cover", cover)):
            assert _probe_free(s, leg), f"motor-lead leg {k} hits {nm}"
    th, e = L.wire_theta, 0.1
    z = 0.5 * (L.wire_slot_z0 + L.z_prop_spigot_bottom - L.body_wall)          # propulsion slot (a): only the skin
    assert _radial_material(prop, L, th, z, L.duct_r, L.body_r + 1.0) == _approx_segs(
        [(L.body_r - L.cone_wall, L.body_r)], abs=1e-3)
    assert _radial_material(prop, L, th, 0.5 * (L.z_prop_spigot_bottom + L.z_prop_spigot_top),
                            L.duct_r, L.body_r + 1.0) == []                     # (b) through the spigot ring
    groove = rg._box_polar(L.wire_ch_r0 + e, L.wire_ch_r1 + 3.0, -L.wire_ch_w / 2 + e, L.wire_ch_w / 2 - e,
                           L.z_intake_bottom - 1.0, L.wire_groove_z1 - e, th)
    assert _probe_free(intake, groove), "open groove blocked (cover off)"
    gate = rg._box_polar(L.wire_notch_r0 + e, L.wire_ch_r1 + 3.0, -L.wire_gate_w / 2 + e, L.wire_gate_w / 2 - e,
                         L.wire_notch_z0 + e, L.wire_ch_z1 - e, th)
    assert _probe_free(intake, gate), "gate over the lip blocked"
    hw_h = getattr(L, "wire_hole_w", L.wire_ch_w) / 2          # v0.9 fix round: the hole's own width (20)
    hole = rg._box_polar(L.wire_hole_r0 + e, L.wire_hole_r1 - e, -hw_h + e, hw_h - e,
                         L.z_intake_bulkhead_bottom - 1.0, L.z_intake_bulkhead_top + 1.0, th)
    assert _probe_free(intake, hole), "bulkhead hole blocked"
    # the 1.2 mm lip-pocket wall stays beside the wire theta; nothing outside it (open groove).  v0.9 fix round: on the wire
    # theta itself the wall is slit (lip_slit) - a bare lead slides from the interior into the groove
    zp = 0.5 * (L.z_intake_skirt_top + L.z_lip_pocket_top)
    slit = getattr(L, "wire_lip_slit_w", 0.0)
    v_wall = 0.5 * slit + 1.0 if slit else 0.0
    # (the groove's floor is the plane u = wire_ch_r0, so off the wire theta its radius is hypot(wire_ch_r0, v))
    assert _radial_material(intake, L, th, zp, L.duct_r, 75.0, v=v_wall) == _approx_segs(
        [(L.lip_pocket_id / 2, math.hypot(L.wire_ch_r0, v_wall))], abs=0.05)
    if slit:
        assert _radial_material(intake, L, th, zp, L.duct_r, 75.0) == []
        for zc in getattr(L, "ml_anchor_z", []):          # tie anchors: material at the inner face, the tunnel open
            t_, h_ = L.ml_anchor_tunnel
            u0 = L.ml_anchor_u + L.ml_anchor_wall
            assert _probe_free(intake, rg._box_polar(u0 + e, u0 + t_ - e, -L.ml_anchor_w / 2, L.ml_anchor_w / 2,
                                                     zc - h_ / 2 + e, zc + h_ / 2 - e, th)), zc
            blk = rg._box_polar(L.ml_anchor_u + e, u0 - e, -L.ml_anchor_w / 2 + e, L.ml_anchor_w / 2 - e,
                                zc - L.ml_anchor_h / 2 + e, zc + L.ml_anchor_h / 2 - e, th)
            assert rg.intersection_volume(blk, intake) == pytest.approx(blk.Volume(), rel=1e-6), zc
    # cover on the side walls, not in the intake; bullets straight up through the intake and up the covered groove
    assert rg.intersection_volume(cover, intake) < 1e-6
    u_in, u_g = 0.5 * (L.wire_hole_r0 + L.wire_hole_r1), 0.5 * (L.wire_ch_r0 + L.wire_ch_r1)
    for s in rg.bullet_group(L, u_in, L.z_intake_bottom - L.ml_bullet_len, L.z_intake_bulkhead_top + L.ml_bullet_len).values():
        assert _probe_free(intake, s)
    for s in rg.bullet_group(L, u_g, L.z_intake_bottom, L.z_lip_pocket_top).values():
        for body in (intake, cover, prop, edf):
            assert _probe_free(body, s)
    # every cross-section the layout sized passes
    assert all(min(d["margin_w"], d["margin_h"]) >= 0.0 for d in L.ml_sections.values()), L.ml_sections


def test_wire_slot_and_intake_wire_channel(tabs_model):
    L, parts = tabs_model
    prop, intake, edf = (parts[n].shape for n in ("propulsion_module", "intake_module", "edf_unit"))
    if getattr(L, "wire_ch_open", False):
        _open_motor_groove_asserts(L, prop, intake, edf, parts["wire_cover"].shape)
        return
    # every leg of the C20 bundle is clear (step out / step in inside their SPEC windows)
    for k, leg in rg.wire_bundle_legs(L).items():
        for nm, s in (("prop", prop), ("intake", intake), ("edf", edf)):
            assert _probe_free(s, leg), f"bundle leg {k} hits {nm}"
    st = rg.wire_bundle_steps(L)
    assert L.wire_gate_lo_z0 <= st["out"][0] and st["out"][1] <= L.z_intake_skirt_top
    assert L.wire_notch_z0 <= st["in"][0] and st["in"][1] <= L.wire_ch_z1
    th, e = L.wire_theta, 0.1
    # (a) lower slot box: below the deck it stops short of the skin (skin intact), in the deck it leaves
    # body_r - wire_slot_lower_r_out; (b) above the deck nothing of the propulsion module at the wire theta
    z = 0.5 * (L.wire_slot_z0 + L.z_prop_spigot_bottom - L.body_wall)
    assert _radial_material(prop, L, th, z, L.duct_r, L.body_r + 1.0) == _approx_segs(
        [(L.body_r - L.cone_wall, L.body_r)], abs=1e-3)
    z = L.z_prop_spigot_bottom - 0.5 * L.body_wall
    assert _radial_material(prop, L, th, z, L.duct_r, L.body_r + 1.0) == _approx_segs(
        [(L.wire_slot_lower_r_out, L.body_r)], abs=1e-3)
    assert _radial_material(prop, L, th, 0.5 * (L.z_prop_spigot_bottom + L.z_prop_spigot_top),
                            L.duct_r, L.body_r + 1.0) == []
    # intake: main channel void above the floor wedge (from wire_gate_lo_z0 up), shelf notch void
    channel = rg._box_polar(L.wire_ch_r0 + e, L.wire_ch_r1 - e, -L.wire_ch_w / 2 + e, L.wire_ch_w / 2 - e,
                            L.wire_gate_lo_z0 + e, L.wire_ch_z1 - e, th)
    assert _probe_free(intake, channel), "intake wire channel blocked"
    # v0.5d floor wedge: the triangle wire_ch_floor_rz is solid over the channel width, the channel above its
    # 45 deg face (down to wire_ch_z0) is void, and the floor reaches wire_gate_lo_z0 at wire_ch_r1
    (ra, za), (rb, _), (_, zc) = L.wire_ch_floor_rz
    assert rb - ra == pytest.approx(zc - za) and za == pytest.approx(L.wire_ch_z0)            # 45 deg
    assert za + (L.wire_ch_r1 - ra) == pytest.approx(L.wire_gate_lo_z0)
    wedge = rg.intake_wire_floor_wedge(L)
    assert rg.intersection_volume(wedge, intake) == pytest.approx(wedge.Volume(), rel=1e-6)
    s2 = math.sqrt(2.0)          # the void polygon below wire_gate_lo_z0, every edge moved e inward (the 45 deg face
    above = rg._prism_uz([(L.wire_ch_r0 + e, L.wire_ch_z0 + e), (ra + e * (1 - s2), L.wire_ch_z0 + e),   # by e)
                          (L.wire_ch_r1 - e, L.wire_gate_lo_z0 + e * (s2 - 1)),
                          (L.wire_ch_r1 - e, L.wire_gate_lo_z0 + e), (L.wire_ch_r0 + e, L.wire_gate_lo_z0 + e)],
                         -L.wire_ch_w / 2 + e, L.wire_ch_w / 2 - e, th)
    assert _probe_free(intake, above), "channel above the floor wedge blocked"
    zf = 0.5 * (L.wire_ch_z0 + L.wire_gate_lo_z0)       # on the radial line: wedge face at r = ra + (z - za), merged
    assert _radial_material(intake, L, th, zf, L.duct_r, 70.0) == _approx_segs(   # with the bulge's outer wall
        [(ra + (zf - za), L.wire_bulge_r_out)], abs=0.01)
    notch = rg._box_polar(L.wire_notch_r0 + e, L.wire_ch_r0 + rg.WIRE_NOTCH_PAST_CH_R0 - e, -L.wire_ch_w / 2 + e,
                          L.wire_ch_w / 2 - e, L.wire_notch_z0 + e, L.wire_ch_z1 - e, th)
    assert _probe_free(intake, notch), "shelf notch blocked"
    # 1.2 mm pocket wall left between the lip and the channel; bulge wall outside the channel
    mat = _radial_material(intake, L, th, 0.5 * (L.z_intake_skirt_top + L.z_lip_pocket_top), L.duct_r, 70.0)
    assert mat == _approx_segs([(L.lip_pocket_id / 2, L.wire_ch_r0), (L.wire_ch_r1, L.wire_bulge_r_out)], abs=0.01), mat
    # over the notch only the bulge's outer wall is left; beside the bulge the shelf is intact
    z_sh = 0.5 * (L.z_lip_pocket_top + L.z_shelf_top)
    assert _radial_material(intake, L, th, z_sh, L.duct_r, 70.0) == _approx_segs(
        [(L.wire_ch_r1, L.wire_bulge_r_out)], abs=0.01)
    a = th + math.degrees(math.atan2(L.wire_bulge_w / 2, L.shelf_id / 2)) + 2.0
    assert _radial_material(intake, L, a, z_sh, L.duct_r, 70.0) == _approx_segs([(L.shelf_id / 2, L.body_r)], abs=0.01)
    # the bulge is closed below wire_ch_z0 (fused with the skirt) and above wire_ch_z1 (v0.6.1: fused with the
    # window_wall_t band, which starts at z_shelf_top below wire_ch_z1)
    assert L.z_shelf_top < L.wire_ch_z1 < L.wire_bulge_z1
    for z, r_in in ((0.5 * (L.wire_bulge_z0 + L.wire_ch_z0), L.body_id / 2), (0.5 * (L.wire_ch_z1 + L.wire_bulge_z1), L.r_band_in)):
        assert _radial_material(intake, L, th, z, L.duct_r, 70.0) == _approx_segs(
            [(r_in, L.wire_bulge_r_out)], abs=0.01), z
    # v0.6.1: over the notch the band is opened from the shelf notch's start out to the channel (step-in through it)
    z_band = 0.5 * (L.z_shelf_top + L.wire_ch_z1)
    assert _radial_material(intake, L, th, z_band, L.duct_r, 70.0) == _approx_segs(
        [(L.wire_ch_r1, L.wire_bulge_r_out)], abs=0.01)
    # intake struts sit on strut_thetas (one over the wire channel), windows between them, above the bulge
    assert L.z_window_bottom > L.wire_bulge_z1
    zw = 0.5 * (L.z_window_bottom + L.z_window_top)
    for s in L.strut_thetas:
        assert _radial_material(intake, L, s, zw, L.r_band_in - 1.0, L.body_r + 1.0) == _approx_segs(
            [(L.r_band_in, L.body_r)], abs=1e-3), s
    for w in L.window_thetas:
        assert _radial_material(intake, L, w, zw, L.r_band_in - 1.0, L.body_r + 1.0) == [], w


def test_servo_lead_route(tabs_model):
    """SPEC 3.8 / 5.1 v0.6: every ribbon proxy of the route is clear; the passages exist only because of the servo
    cuts (negative control on the module built from params without servo_leads); the radial sections at the column /
    gather theta are the SPEC ones; the lumped lead mass sits at the route centroid."""
    L, parts = tabs_model
    assert L.servo_leads
    prop, intake, edf = (parts[n].shape for n in ("propulsion_module", "intake_module", "edf_unit"))
    for k in range(L.vane_count):
        for nm, leg in rg.servo_lead_legs(L, k).items():
            for mn, s in (("prop", prop), ("intake", intake), ("edf", edf)):
                assert _probe_free(s, leg), f"lead {k + 1} leg {nm} hits {mn}"
    for nm, leg in rg.servo_bundle_legs(L).items():
        for mn, s in (("prop", prop), ("intake", intake), ("edf", edf)):
            assert _probe_free(s, leg), f"bundle leg {nm} hits {mn}"
    # negative control: the same proxies hit the propulsion module built without the servo-lead cuts
    p0 = load_params(PARAMS_PATH, None)
    p0.pop("servo_leads")
    L0 = Layout(p0)
    assert not L0.servo_leads
    old = rg.propulsion_module(L0)
    for nm in (("T", "S", "N") if L.direct_drive else ("W", "S", "N")):   # v0.9: T = the lead pipe's tunnel
        assert rg.intersection_volume(rg.servo_lead_legs(L, 0)[nm], old) > 1.0, nm
    assert rg.intersection_volume(rg.servo_bundle_legs(L)["R-"], old) > 1.0
    # radial sections at the first column: deck (notch a; the skirt seat stays), spigot (notch b), rebate
    th, e = L.sl_col_thetas[0], 1e-3
    zd = L.z_prop_spigot_bottom - 0.5 * L.body_wall
    assert _radial_material(prop, L, th, zd, L.duct_r, L.body_r + 1.0) == _approx_segs(
        [(L.edf_bore_r, L.sl_notch_u0), (L.spigot_od / 2, L.body_r)], abs=1e-3)
    zs = 0.5 * (L.z_prop_spigot_bottom + L.sl_rebate_z0)
    assert _radial_material(prop, L, th, zs, L.duct_r, L.body_r + 1.0) == _approx_segs(
        [(L.edf_bore_r, L.sl_notch_u0)], abs=1e-3)
    zr = 0.5 * (L.sl_rebate_z0 + L.z_prop_spigot_top)
    for t in (th, L.sl_gather_theta, L.tab_thetas[0]):           # inside the rebate (and over a keyway)
        mat = _radial_material(prop, L, t, zr, L.duct_r, L.body_r + 1.0)
        assert all(b <= L.sl_rebate_r0 + e for _, b in mat), (t, mat)
    t_free = 0.5 * (L.wire_theta + L.sl_rebate_th0)               # outside its sector the spigot ring is whole
    assert _radial_material(prop, L, t_free, zr, L.duct_r, L.body_r + 1.0) == _approx_segs(
        [(L.edf_bore_r, L.spigot_od / 2)], abs=1e-3)
    # entry slot: the shoulder ring is cut from sl_entry_u0 outward (inside the ring, below the annulus)
    z_ring = 0.5 * (L.sl_entry_z_mouth + L.z_edf_shoulder)
    mat = _radial_material(prop, L, th, z_ring, L.duct_r, L.body_r + 1.0)
    assert mat == _approx_segs([(L.duct_r, L.sl_entry_u0)], abs=1e-3), mat
    if L.direct_drive:
        # v0.9: the lead pipe's tunnel is open from the pocket to the entry slot (a thin rod on its axis) and the pipe
        # exists only with the route (the model without servo_leads has neither)
        a, b = L.sl_pipe_points(0)
        assert _probe_free(prop, rg._cyl_between(1.0, a, b))
        ring = rg._cyl_between(L.sl_pipe_d - 0.5, a, b).cut(rg._cyl_between(L.sl_tunnel_d + 0.5, a, b, ext0=1.0, ext1=1.0))
        assert rg.intersection_volume(ring, prop) > rg.intersection_volume(ring, old) + 1.0
    else:
        # pod window: open through the far end wall over the window's u / Z range (blocked without the cuts)
        zw = 0.5 * (L.sl_win_z0 + L.sl_win_z1)
        v0, v1 = L.pod_cavity_w / 2 + e, L.pod_w / 2 - e
        window = rg._box_polar(L.sl_win_u0 + e, L.sl_win_u1 - e, v0 if L.sl_side > 0 else -v1, v1 if L.sl_side > 0 else -v0,
                               zw - 0.5, zw + 0.5, L.pod_thetas[0])
        assert _probe_free(prop, window) and not _probe_free(old, window)
    # intake at the gather theta: pocket wall, channel, bulge wall beside the bell; the floor wedge is solid
    zi = 0.5 * (L.z_intake_skirt_top + L.z_lip_pocket_top)
    assert _radial_material(intake, L, L.sl_gather_theta, zi, L.duct_r, 70.0) == _approx_segs(
        [(L.lip_pocket_id / 2, L.sl_ch_r0), (L.sl_ch_r1, L.sl_bulge_r_out)], abs=0.01)
    wedge = rg.intake_servo_floor_wedge(L)
    assert rg.intersection_volume(wedge, intake) == pytest.approx(wedge.Volume(), rel=1e-6)
    # the lumped lead mass is in the vehicle at the route centroid
    lead = parts["servo_leads"]
    assert lead.kind == "lumped" and lead.mass_g == pytest.approx(L.sl_lead_mass_g)
    assert lead.point_mm == pytest.approx(tuple(L.sl_lead_com))
    assert all(a > b > 0 for a, b in zip(L.sl_route_len, L.sl_route_len_modelled))


# --------------------------------------------------------------------------- #
# v0.6.1, both presets: window_wall_t intake band with radial-sector windows (SPEC 5.1, check C9)
# --------------------------------------------------------------------------- #
def test_intake_band_struts_and_windows(built):
    """SPEC 5.1 v0.6.1: the band Z [z_shelf_top, z_intake_bulkhead_bottom] is window_wall_t thick (r_band_in..body_r,
    outer surface = body OD); every strut keeps strut_w at body_r with radial side faces (mid-height section = the
    design sector; angular edges at strut_angle / 2 both at the inner and at the outer face); every window is open
    through over its whole sector and Z range and closed right outside it; the margin rings are the full band; the
    shelf stands out past the band; Layout's numbers are the design ones."""
    _, L, parts, _ = built
    intake = parts["intake_module"].shape
    im, mf = L.p["intake_module"], L.p["printer"]["min_feature"]
    assert L.window_wall_t == im["window_wall_t"] >= L.body_wall
    assert L.r_band_in == pytest.approx(L.body_r - im["window_wall_t"])
    assert L.strut_angle_deg == pytest.approx(math.degrees(im["strut_w"] / L.body_r))
    assert L.window_angle_deg == pytest.approx(360.0 / im["strut_count"] - L.strut_angle_deg)
    assert [0.5 * (a + b) for a, b in L.window_sectors] == pytest.approx(L.window_thetas)
    assert L.strut_w_in == pytest.approx(L.strut_w * L.r_band_in / L.body_r) and L.strut_w_in >= mf
    assert L.shelf_ledge == pytest.approx(L.r_band_in - L.shelf_id / 2) and L.shelf_ledge >= mf
    sect = 0.5 * math.radians(L.strut_angle_deg) * (L.body_r ** 2 - L.r_band_in ** 2)
    assert L.strut_section_mm2 == pytest.approx(sect)
    e, h = 0.05, 0.1
    zm0 = 0.5 * (L.z_window_bottom + L.z_window_top)
    hs = 0.5 * L.strut_angle_deg
    # v0.9 fix round: the motor-lead tie anchors stand on the wire_theta strut's inner face - that strut is sectioned in
    # the free span between two anchors nearest mid-height (as check C9), the others at mid-height
    anc = list(getattr(L, "ml_anchor_z", []) or [])
    if anc:
        stops = [L.z_shelf_top] + anc + [L.z_intake_bulkhead_bottom]
        z_free = min((0.5 * (a + b) for a, b in zip(stops[:-1], stops[1:])), key=lambda z: abs(z - zm0))
    for s in L.strut_thetas:
        zm = z_free if (anc and abs(((s - L.wire_theta + 180.0) % 360.0) - 180.0) < 1e-6) else zm0
        half = hs + 0.25 * L.window_angle_deg
        slab = rg._sector(L.r_band_in - 1.0, L.body_r + 1.0, s - half, s + half, zm, zm + h)
        assert rg.intersection_volume(slab, intake) / h == pytest.approx(sect, rel=1e-6), s
        for r in (L.r_band_in + e, L.body_r - e):            # radial side faces: same angular edges at both faces
            d = math.degrees(e / r)
            ring = rg._sector(r - 0.01, r + 0.01, s - hs + d, s + hs - d, zm, zm + h)
            assert rg.intersection_volume(ring, intake) == pytest.approx(ring.Volume(), rel=1e-6), (s, r)
            for a, b in ((s - hs - 1.0, s - hs - d), (s + hs + d, s + hs + 1.0)):
                assert _probe_free(intake, rg._sector(r - 0.01, r + 0.01, a, b, zm, zm + h)), (s, r, a, b)
    d = math.degrees(e / L.r_band_in)
    for (lo, hi), w in zip(L.window_sectors, L.window_thetas):
        win = rg._sector(L.r_band_in - 1.0, L.body_r + 1.0, lo + d, hi - d, L.z_window_bottom + e, L.z_window_top - e)
        if L.has_window_gable:      # v0.6.2: open above the lower boundary (the gussets stand under it)
            win = win.intersect(rg.intake_window_gable_prism(L, w).translate(rg.V(0.0, 0.0, e)))
        assert _probe_free(intake, win), w
        for z0, z1 in ((L.z_window_bottom - 2 * e, L.z_window_bottom - e), (L.z_window_top + e, L.z_window_top + 2 * e)):
            band = rg._sector(L.r_band_in + e, L.body_r - e, lo + d, hi - d, z0, z1)   # closed right outside the window
            assert rg.intersection_volume(band, intake) == pytest.approx(band.Volume(), rel=1e-6), (w, z0)
        for z in (0.5 * (L.z_shelf_top + L.z_window_bottom), 0.5 * (L.z_window_top + L.z_intake_bulkhead_bottom)):
            assert _radial_material(intake, L, w, z, L.shelf_id / 2 - 1.0, L.body_r + 1.0) == _approx_segs(
                [(L.r_band_in, L.body_r)], abs=1e-3), (w, z)
    # the shelf (lip stop) stands out past the band's inner face, between two struts (no lead notch there)
    z_sh = 0.5 * (L.z_lip_pocket_top + L.z_shelf_top)
    assert _radial_material(intake, L, L.window_thetas[0], z_sh, L.duct_r, L.body_r + 1.0) == _approx_segs(
        [(L.shelf_id / 2, L.body_r)], abs=1e-3)
    # only the band changed: the skirt below the shelf is still the body wall
    assert _radial_material(intake, L, L.window_thetas[0], L.z_intake_bottom + 2.0, L.duct_r, L.body_r + 1.0) == \
        _approx_segs([(L.body_id / 2, L.body_r)], abs=1e-3)


@pytest.mark.parametrize("change, message", [
    ({"window_wall_t": 1.0}, "window_wall_t 1 < body.wall"),                       # thinner than the tube
    ({"window_wall_t": 7.5}, "shelf"),                                              # the shelf no longer stands out
    # the leads would run in the band (v0.9: the 41-52 hole is far inside the band - move the hole out to test the rule)
    ({"wire_channel": {**load_params(PARAMS_PATH, None)["intake_module"]["wire_channel"], "bulkhead_hole_r": [46.0, 57.5]}},
     "bulkhead wire hole"),
    ({"strut_w": 1.0}, "strut inner face"),                                         # < printer.min_feature
    ({"strut_w": 80.0}, "wider than the window"),                                   # v0.6.2: the 20 mm bridge no longer fits
    ({"min_inlet_area_ratio": 6.2}, "inlet area ratio"),                            # < minimum: square windows (6.46) would
                                                                                    # pass, the v0.6.2 gussets count (5.97)
])
def test_layout_rejects_a_bad_intake_band(change, message):
    """v0.6.1: Layout raises with a clear message (before any CAD runs) when the windowed band breaks a rule."""
    p = load_params(PARAMS_PATH, None)
    p["intake_module"].update(change)
    with pytest.raises(ValueError, match=message):
        Layout(p)
    Layout(load_params(PARAMS_PATH, None))                                          # the real params pass


def test_c9_fails_on_a_skinned_window_a_thin_strut_or_a_holed_ring(default_build):
    """C9 is positive evidence of the v0.6.1 band: it FAILs when a 0.2 mm skin is left across one window, when one
    strut is cut back to the old 1.2 mm wall, and when a margin ring has a hole; PASS on the real model."""
    L, parts, report = default_build
    assert cg.Checker(L, parts, report).c9_inlet().status == cg.PASS
    intake = parts["intake_module"].shape

    def status(shape) -> str:
        p2 = dict(parts)
        p2["intake_module"] = dataclasses.replace(parts["intake_module"], shape=shape)
        return cg.Checker(L, p2, report).c9_inlet().status

    lo, hi = L.window_sectors[0]
    skin = rg._sector(L.body_r - 0.2, L.body_r, lo, hi, L.z_window_bottom, L.z_window_top)
    assert status(rg._fuse([intake, skin])) == cg.FAIL                                  # skin across a window
    s, hs = L.strut_thetas[1], 0.5 * L.strut_angle_deg + 1.0
    thin = intake.cut(rg._sector(L.r_band_in - 1.0, L.body_r - L.body_wall, s - hs, s + hs, L.z_window_bottom,
                                 L.z_window_top))
    assert status(thin) == cg.FAIL                                                      # one strut back to 1.2 mm
    w = L.window_thetas[2]
    holed = intake.cut(rg._sector(L.r_band_in - 1.0, L.body_r + 1.0, w - 2.0, w + 2.0, L.z_window_top + 0.5,
                                  L.z_intake_bulkhead_bottom - 0.5))
    assert status(holed) == cg.FAIL                                                     # hole in the ring above


# --------------------------------------------------------------------------- #
# v0.6.2, both presets: truncated-gable lower window ends (intake_module.window_bottom_end, check C9)
# --------------------------------------------------------------------------- #
def _boundary_z(solid, x: float, y: float, z0: float, z1: float, steps: int = 30) -> float:
    """Z where the material at (x, y) ends going up from z0 (inside) to z1 (void): bisection on Solid.isInside."""
    assert solid.isInside(rg.V(x, y, z0)) and not solid.isInside(rg.V(x, y, z1))
    for _ in range(steps):
        zm = 0.5 * (z0 + z1)
        if solid.isInside(rg.V(x, y, zm)):
            z0 = zm
        else:
            z1 = zm
    return 0.5 * (z0 + z1)


def test_window_gable_on_the_solids(built):
    """SPEC 5.1 v0.6.2: the LOWER end of every window is the truncated gable - in the window frame the material under
    the window ends at z_window_bottom over the bridge |y'| <= bridge_w/2 and at z_window_bottom + (|y'| - bridge_w/2) x
    tan(slope) beyond it, found by bisection on the solid at both band surfaces across the whole window (independent of
    the C9 edge analysis); along each surface that boundary is flat on the bridge and >= 35 deg from horizontal
    everywhere else (printability: no support in the windows when the intake prints flipped), as Layout reports; the
    gussets measured on the solid are Layout's volume; the upper end stays square (open right under z_window_top over the
    whole sector, closed right above it)."""
    _, L, parts, _ = built
    assert L.has_window_gable
    intake = parts["intake_module"].shape
    hf, t = L.gable_half_flat, L.gable_tan
    surf = {"outer": L.body_r - 0.01, "inner": L.r_band_in + 0.01}
    slope_min = {k: math.inf for k in surf}
    for (lo, hi), w in zip(L.window_sectors, L.window_thetas):
        for nm, r in surf.items():
            prev = None
            n = 24
            for k in range(n + 1):
                phi = math.radians(-L.window_half_angle_deg + 0.02 + k * (2 * L.window_half_angle_deg - 0.04) / n)
                x, y = r * math.cos(math.radians(w) + phi), r * math.sin(math.radians(w) + phi)
                zb = _boundary_z(intake, x, y, L.z_window_bottom - 1.0, L.z_gable_top + 1.0)
                yp = r * math.sin(phi)
                assert zb == pytest.approx(L.z_window_bottom + max(0.0, abs(yp) - hf) * t, abs=1e-4), (w, nm, k)
                if prev is not None:
                    dz, ds = zb - prev[1], r * (phi - prev[0])
                    on_bridge = abs(r * math.sin(prev[0])) <= hf and abs(yp) <= hf
                    if on_bridge:
                        assert abs(dz) < 1e-4, (w, nm, k)
                    elif min(abs(r * math.sin(prev[0])), abs(yp)) >= hf:     # both samples on one gable face
                        slope_min[nm] = min(slope_min[nm], math.degrees(math.atan2(abs(dz), abs(ds))))
                prev = (phi, zb)
    for nm in surf:
        assert slope_min[nm] >= 35.0 and slope_min[nm] >= L.gable_edge_slope_min_deg - 0.1, (nm, slope_min)
    assert 35.0 <= L.gable_edge_slope_min_deg < L.gable_slope_deg
    # gussets on the solid = Layout's integral; the design gusset solids are exactly the material there
    # (v0.9: out to body_r - the open motor-lead groove's roof wedge stands outside the body inside the sectors' angles)
    region = rg._fuse([rg._sector(L.r_band_in - 1.0, L.body_r, lo, hi, L.z_window_bottom, L.z_gable_top + 1.0)
                       for lo, hi in L.window_sectors])
    assert _volume_or_0(intake.intersect(region)) == pytest.approx(L.gusset_volume_mm3, rel=1e-6)
    gus = rg._fuse(rg.intake_window_gussets(L))
    assert _volume_or_0(gus.cut(intake)) < 1e-6 and _volume_or_0(intake.intersect(region).cut(gus)) < 1e-6
    # upper end square: open right under the window top over the whole (inset) sector, solid right above it
    e = 0.05
    for (lo, hi), w in zip(L.window_sectors, L.window_thetas):
        d = math.degrees(e / L.r_band_in)
        assert _probe_free(intake, rg._sector(L.r_band_in - 1.0, L.body_r + 1.0, lo + d, hi - d, L.z_window_top - 2 * e,
                                              L.z_window_top - e)), w
        top = rg._sector(L.r_band_in + e, L.body_r - e, lo + d, hi - d, L.z_window_top + e, L.z_window_top + 2 * e)
        assert rg.intersection_volume(top, intake) == pytest.approx(top.Volume(), rel=1e-6), w


@pytest.mark.parametrize("change, message", [
    ({"bridge_w": 70.0}, "wider than the window"),                     # wider than the window at r_band_in (2 x 33.83)
    ({"bridge_w": -1.0}, "must be >= 0"),
    ({"slope_deg": 25.0}, "outside 30..60"),
    ({"slope_deg": 65.0}, "outside 30..60"),
    ({"slope_deg": 30.0}, r"\(< 35\)"),                                # edge on the curved wall 24.9 deg < 35
])
def test_layout_rejects_a_bad_window_gable(change, message):
    """v0.6.2: Layout raises with a clear message for nonsense window_bottom_end params (before any CAD runs)."""
    p = load_params(PARAMS_PATH, None)
    p["intake_module"]["window_bottom_end"].update(change)
    with pytest.raises(ValueError, match=message):
        Layout(p)
    p = load_params(PARAMS_PATH, None)
    p["intake_module"]["window_h"] = 25.0                              # the 25.61 mm gable would fill the window
    with pytest.raises(ValueError, match="taller than the window"):
        Layout(p)
    Layout(load_params(PARAMS_PATH, None))                             # the real params pass


def test_c9_fails_on_a_square_or_wrong_end_window(default_build):
    """C9's v0.6.2 evidence: square-ended windows (the v0.6.1 cut) FAIL the gusset probe and the edge evidence; a gable
    at the wrong (upper) end FAILs both the gusset probe at the bottom and the window clearance above the boundary;
    PASS on the real model."""
    L, parts, report = default_build
    c9 = cg.Checker(L, parts, report).c9_inlet()
    assert c9.status == cg.PASS and c9.values["gable"]["ok"]
    intake = parts["intake_module"].shape

    def check(shape):
        p2 = dict(parts)
        p2["intake_module"] = dataclasses.replace(parts["intake_module"], shape=shape)
        return cg.Checker(L, p2, report).c9_inlet()

    square = intake.cut(*rg.intake_window_sectors(L))
    c = check(square)
    assert c.status == cg.FAIL
    g = c.values["gable"]
    assert g["gusset_void_max"] > 1000.0 and g["missing"] and g["flat_off_bridge"] > 0
    zm = 0.5 * (L.z_window_bottom + L.z_window_top)
    upper = [s.mirror("XY", (0.0, 0.0, zm)) for s in rg.intake_window_gussets(L)]
    wrong = rg._fuse([square] + upper)
    c = check(wrong)
    assert c.status == cg.FAIL
    assert c.values["gable"]["gusset_void_max"] > 1000.0 and max(c.values["window_probe_mm3"].values()) > 1000.0


# --------------------------------------------------------------------------- #
# v0.6.2, both presets: radio / logger box under the payload bulkhead (check C24)
# --------------------------------------------------------------------------- #
def test_radio_log_placeholder(built):
    """The ``radio_log`` placeholder is the Layout box (params dims, centred at r_center on theta_deg, top at the payload
    bulkhead underside) with the params mass, its own colour, module avionics_module; it hangs from the bulkhead
    (touching, no overlap), stays aboard after ejection, and every Layout clearance holds its limit."""
    _, L, parts, report = built
    rl, el = parts["radio_log"], L.p["electronics"]["radio_log"]
    assert rl.kind == "placeholder" and rl.module == "avionics_module" and rl.color == rg.COLORS["radio_log"]
    assert rl.color not in [v for k, v in rg.COLORS.items() if k != "radio_log"]
    assert report["parts"]["radio_log"]["mass_g"] == pytest.approx(el["mass_g"], abs=1e-9)
    bb = rl.shape.BoundingBox()
    assert (bb.xlen, bb.ylen, bb.zlen) == pytest.approx(tuple(el["dims_lwh"]), abs=1e-9)
    th = math.radians(el["theta_deg"])
    assert (0.5 * (bb.xmin + bb.xmax), 0.5 * (bb.ymin + bb.ymax)) == pytest.approx(
        (el["r_center"] * math.cos(th), el["r_center"] * math.sin(th)), abs=1e-9)
    assert bb.zmax == pytest.approx(L.z_payload_bulkhead_bottom, abs=1e-9)
    pay = parts["payload_module"].shape
    assert rg.intersection_volume(rl.shape, pay) < 1e-6 and rl.shape.distance(pay) < 1e-6
    assert "radio_log" not in rg.POST_EJECTION_EXCLUDED
    mc = L.radio_log_min_clear
    for nm, v in L.radio_log_clearance.items():
        lim = 1.0 if nm == "spigot bore (radial)" else (0.0 if "footprint" in nm else mc)
        assert v >= lim, (nm, v)


def test_c24_fails_for_a_misplaced_radio_log(default_build):
    """C24 is positive evidence on the solids: PASS on the real model; FAIL for the box 1 mm low (not hanging from the
    bulkhead), 3.5 mm toward the COM board (1.5 mm < min_clear_to_tray), 5 mm toward the wall (no bore clearance), and
    N/A (never FAIL) without the placeholder."""
    L, parts, report = default_build
    assert cg.Checker(L, parts, report).c24_radio_log().status == cg.PASS

    def status(shape) -> str:
        p2 = dict(parts)
        p2["radio_log"] = dataclasses.replace(parts["radio_log"], shape=shape)
        return cg.Checker(L, p2, report).c24_radio_log().status

    box = parts["radio_log"].shape
    assert status(box.translate(rg.V(0.0, 0.0, -1.0))) == cg.FAIL
    toward_com = rg.V(0.0, 3.5, 0.0) if L.radio_log_y[1] < L.com_board_y[0] else rg.V(0.0, -3.5, 0.0)
    assert status(box.translate(toward_com)) == cg.FAIL
    assert status(box.translate(rg.V(0.0, -5.0, 0.0) if L.radio_log_y[0] < 0 else rg.V(0.0, 5.0, 0.0))) == cg.FAIL
    p2 = {n: q for n, q in parts.items() if n != "radio_log"}
    p_no = load_params(PARAMS_PATH, None)
    del p_no["electronics"]["radio_log"]
    assert cg.Checker(Layout(p_no), p2, report).c24_radio_log().status == cg.NA


def test_kaist_has_no_servo_lead_route(built):
    """v0.6 servo leads are a tab-mount feature: the lip_clamp preset keeps no route and no lead mass."""
    _, L, _, report = built
    if L.tabs:
        pytest.skip("tab-mount preset")
    assert not L.servo_leads and "servo_leads" not in report["parts"]


def test_edf_unit_tab_placeholder(tabs_model):
    L, parts = tabs_model
    edf = parts["edf_unit"].shape
    bb = edf.BoundingBox()
    assert abs(bb.zmin - (L.z_edf_housing_bottom - L.p["edf"]["motor_protrusion_rear"])) < 1e-6
    assert abs(bb.zmax - L.z_edf_lip_top) < 1e-6
    assert len(edf.Solids()) == 2
    for th in L.tab_thetas:
        mat = _radial_material(edf, L, th, L.tab_slot_zs[1], 0.0, L.body_r)
        r_slot0, r_slot1 = L.tab_slot_r - L.tab_slot_len_radial / 2, L.tab_slot_r + L.tab_slot_len_radial / 2
        assert [round(r, 6) for seg in mat[-2:] for r in seg] == pytest.approx(
            [L.p["edf"]["housing_id"] / 2, r_slot0, r_slot1, L.tab_r_out], abs=1e-3), mat
    # the lip is a ring (ID housing_id) in tabs mode (probed midway between two tabs)
    mat = _radial_material(edf, L, L.tab_thetas[0] + 180.0 / L.tab_count, 0.5 * (L.z_edf_lip_bottom + L.z_edf_lip_top),
                           0.0, L.body_r)
    assert mat[-1][0] == pytest.approx(L.p["edf"]["housing_id"] / 2, abs=1e-3)
    assert mat[-1][1] == pytest.approx(L.p["edf"]["lip_od"] / 2, abs=1e-3)


# --------------------------------------------------------------------------- #
# v0.5d, both presets: tray stops (SPEC 5.2) and blind propulsion joint holes (SPEC 3.2)
# --------------------------------------------------------------------------- #
V05D_KEYS = (("body", "prop_joint_hole_depth"), ("avionics_module", "tray_stop_h"))   # params keys v0.5d added
V06_KEYS = (("payload_module", "com_mount"), ("electronics", "com"),               # params keys v0.6 added (COM board)
            ("electronics", "radio_log"),   # v0.6.2: the radio/log box beside it takes its clearance from com_mount
            ("electronics", "fcu"))         # v0.6.3: so does the FCU board on the intake bulkhead
V08_KEYS = (("payload_module", "gps_pod"), ("avionics_module", "gps_entry"),   # params keys v0.8 added (GPS pod)
            ("electronics", "gns"), ("avionics_module", "gns_mount"))      # v0.8.1: the GNS board needs the pod
PRECISE_GPROP_EPS = 1e-9      # adaptive OCC volume integration: the report's default GProp integration is off by
                              # up to 5.4 mm^3 (1.6e-5 relative, BUDGET propulsion module) - too coarse for 75 mm^3
REPORT_GPROP_REL_ERR = 2e-5   # bound on that default-integration error (measured <= 1.6e-5) for report-level deltas


def _layout_pre_v05d(preset, restore: dict | None = None, drop_v06: bool = False) -> Layout:
    """Layout of ``preset`` without the params keys v0.5d added (no tray stops, through propulsion joint holes) and,
    with ``drop_v06``, without the v0.6 COM-board keys too; ``restore`` {(section, key): value} sets params back to
    older values."""
    p = load_params(PARAMS_PATH, preset)
    for sec, key in V05D_KEYS + (V06_KEYS + V08_KEYS if drop_v06 else ()):   # v0.4 had neither
        p[sec].pop(key, None)
    for (sec, key), val in (restore or {}).items():
        p[sec][key] = val
    return Layout(p)


def _layout_pre_v06(preset) -> Layout:
    """Layout of ``preset`` without the v0.6 COM-board params keys only (no bosses, no ``com`` placeholder)."""
    p = load_params(PARAMS_PATH, preset)
    for sec, key in V06_KEYS:
        p[sec].pop(key, None)
    return Layout(p)


def _com_boss_added_mm3(L) -> float:
    """v0.6: material the COM bosses add under the payload bulkhead: per boss the ring Ø boss_d minus the tap hole over
    boss_h (the part of each boss cylinder that runs OVERLAP into the bulkhead is bulkhead material already)."""
    return len(L.com_boss_xy) * math.pi / 4 * (L.com_boss_d ** 2 - L.com_hole_d ** 2) * L.com_boss_h


def _com_hole_removed_mm3(L) -> float:
    """v0.6: bulkhead material the blind holes remove above the boss tops: Ø hole_d over hole_depth - boss_h each."""
    return len(L.com_boss_xy) * math.pi / 4 * L.com_hole_d ** 2 * (L.com_hole_depth - L.com_boss_h)


def _precise_volume(shape) -> float:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props, PRECISE_GPROP_EPS)
    return props.Mass()


def _volume_or_0(shape) -> float:
    """Volume of a boolean result that may be empty."""
    return 0.0 if shape is None or shape.wrapped.IsNull() or not shape.Solids() else _precise_volume(shape)


def _tray_stop_added_mm3(L) -> float:
    """Analytic material the two tray stops add: per slot the area between y = rail_y - rail_depth (the ribs' inner
    end) and the bore y = sqrt(Ri^2 - x^2) for x in tray_x +- rail_gap/2, times tray_stop_h (the rest of each block
    lies in the tube wall, which is already material)."""
    ri = L.body_id / 2

    def prim(x: float) -> float:          # integral of sqrt(ri^2 - x^2) dx
        return 0.5 * (x * math.sqrt(ri * ri - x * x) + ri * ri * math.asin(x / ri))

    x0, x1 = L.tray_x - L.rail_gap / 2, L.tray_x + L.rail_gap / 2
    area = prim(x1) - prim(x0) - (L.rail_y - L.rail_depth) * (x1 - x0)
    return 2.0 * area * L.tray_stop_h


def _blind_hole_floor_mm3(L, n: int = 2000) -> float:
    """Analytic material the blind propulsion joint holes keep (vs through holes), all holes: per hole the
    O joint_hole_spigot_d cylinder between the EDF bore surface (r = edf_bore_r) and the flat hole bottom
    (r0 = spigot_od/2 - prop_joint_hole_depth): pi a^2 r0 - int_{-a}^{a} 2 sqrt(a^2 - y^2) sqrt(Rb^2 - y^2) dy
    (Simpson in y = a sin(phi))."""
    a, rb = L.p["body"]["joint_hole_spigot_d"] / 2, L.edf_bore_r
    r0 = L.spigot_od / 2 - L.prop_joint_hole_depth
    h, s = math.pi / n, 0.0
    for k in range(n + 1):
        ph = -math.pi / 2 + k * h
        f = 2.0 * a * a * math.cos(ph) ** 2 * math.sqrt(rb * rb - (a * math.sin(ph)) ** 2)
        s += (1 if k in (0, n) else (4 if k % 2 else 2)) * f
    return len(L.joint_thetas) * (math.pi * a * a * r0 - s * h / 3.0)


def test_tray_stops(built):
    """SPEC 5.2 v0.5d (both presets): in each rail slot a block between the ribs, the ribs' radial extent, Z
    [z_tray_stop_bottom, z_tray_bottom]; the tray rests on it; the slot is open above it (tray) and below it."""
    preset, L, parts, _ = built
    av, tray = parts["avionics_module"].shape, parts["avionics_tray"].shape
    assert L.tray_stop_h and L.z_tray_stop_bottom == pytest.approx(L.z_tray_bottom - L.tray_stop_h)
    assert L.z_tray_stop_bottom >= L.z_rail_bottom - 1e-9
    e = 0.05
    x0, x1 = L.tray_x - L.rail_gap / 2 + e, L.tray_x + L.rail_gap / 2 - e
    y_in, y_bore = L.rail_y - L.rail_depth, math.sqrt((L.body_id / 2) ** 2 - x1 ** 2)
    for sy in (+1.0, -1.0):
        def box(z0: float, z1: float, sy=sy):
            ya, yb = (y_in + e, y_bore - e) if sy > 0 else (-y_bore + e, -y_in - e)
            return rg._box(x0, x1, ya, yb, z0, z1)
        block = box(L.z_tray_stop_bottom + e, L.z_tray_bottom - e)
        assert rg.intersection_volume(block, av) == pytest.approx(block.Volume(), rel=1e-6), ("stop not solid", sy)
        assert _probe_free(av, box(L.z_tray_bottom + e, L.z_tray_bottom + 5.0)), ("slot blocked above the stop", sy)
        assert _probe_free(av, box(L.z_tray_stop_bottom - 2.0, L.z_tray_stop_bottom - e)), ("material below", sy)
        # the block reaches the ribs on both sides (one solid with the rail pair): material just outside x0 / x1
        for xa, xb in ((x0 - 2 * e, x0 - e), (x1 + e, x1 + 2 * e)):
            ya, yb = (y_in + e, y_bore - e) if sy > 0 else (-y_bore + e, -y_in - e)
            side = rg._box(xa, xb, ya, yb, L.z_tray_stop_bottom + e, L.z_tray_bottom - e)
            assert rg.intersection_volume(side, av) == pytest.approx(side.Volume(), rel=1e-6)
    assert tray.BoundingBox().zmin == pytest.approx(L.z_tray_bottom, abs=1e-6)    # the tray rests on the stops
    assert rg.intersection_volume(tray, av) < 1e-6
    # nothing but the blocks changed: the module minus the same module without stops is exactly the analytic volume
    old = rg.avionics_module(_layout_pre_v05d(preset))
    assert _volume_or_0(av.cut(old)) == pytest.approx(_tray_stop_added_mm3(L), rel=1e-6)
    assert _volume_or_0(old.cut(av)) < 1e-6
    assert _precise_volume(av) - _precise_volume(old) == pytest.approx(_tray_stop_added_mm3(L), rel=1e-6)


def test_propulsion_joint_holes_are_blind(built):
    """SPEC 3.2 v0.5d (both presets): the propulsion spigot joint holes run from spigot_od/2 - prop_joint_hole_depth
    out through the spigot OD; material stays between the EDF bore and the hole bottom."""
    preset, L, parts, _ = built
    prop = parts["propulsion_module"].shape
    r0 = rg.prop_joint_hole_r0(L)
    assert r0 == pytest.approx(L.spigot_od / 2 - L.prop_joint_hole_depth) and r0 > L.edf_bore_r
    zc = L.z_prop_spigot_bottom + L.joint_screw_dz
    d = L.p["body"]["joint_hole_spigot_d"]
    for th in L.joint_thetas:
        assert _radial_material(prop, L, th, zc - 0.05, L.duct_r, L.body_r + 1.0) == _approx_segs(
            [(L.edf_bore_r, r0)], abs=1e-3), th                     # on the axis: only the floor, bore -> bottom
        assert _probe_free(prop, rg._radial_cyl(d - 0.05, r0 + 0.05, L.spigot_od / 2 + 1.0, th, zc)), th
        assert rg.intersection_volume(rg._radial_cyl(d, r0 - 0.5, r0 - 0.05, th, zc), prop) == pytest.approx(
            math.pi / 4 * d * d * 0.45, rel=1e-4), th               # flat bottom, full section
    old = rg.propulsion_module(_layout_pre_v05d(preset))           # the same module with through holes
    assert _volume_or_0(prop.cut(old)) == pytest.approx(_blind_hole_floor_mm3(L), rel=1e-5)
    assert _volume_or_0(old.cut(prop)) < 1e-6
    assert _precise_volume(prop) - _precise_volume(old) == pytest.approx(_blind_hole_floor_mm3(L), rel=1e-5)


# v0.4 KAIST_90mm_12S report (params sha 49d2d150..., cadquery 2.8.0): sha256 of every part entry as canonical JSON
# (json.dumps(entry, sort_keys=True): kind, volume, density, mass, bbox, COM, inertia - bit for bit).
KAIST_V04_SHA256 = {
    "propulsion_module": "c4d0d6d023bda5c8e30658118eb64f4705ad63906ca426ecd9624d68be87e43d",
    "intake_module": "56c3d8a83765bbfa43395b4879187ca72b9f41962b84ad2e3ad147e406b35943",
    "avionics_module": "c146ab1ff273d7dc1e67c3a4f969cb2b23ee0122afd19c176352af1fb4e9190c",
    "avionics_tray": "07e89d8c75f772b456090342a515abe0b0fc7e05eab57bc0c3b539ba26413e13",
    "payload_module": "8a42029236e3d9d28ee686e04d9c7038472d166a3541cdf148443b57d82c8bdc",
    "piston": "bb490180596217511dddc5b847c9f9d4c9ec887900b6de3b5dfc87b27607d1eb",
    "nose_cone": "99cab9b89bfbe0f2002f872e9975e43c50b59c374b0c471ef5acba1c6dec581e",
    "vane_1": "f5a8cedf0a3af67968b789ca9031c58530b401a0332f773c1b78f13c7f745c43",
    "vane_2": "19a1091b8d47d3aa30da1db73425fb5e34246b02a098778b5279622a469c9143",
    "vane_3": "440e2133b7749fbce7f2708ce4a23776b7f12df9b76b9d607d3c322740d5ce73",
    "vane_4": "7abc56bbdac1b2348c48d3d7dbb6f50db6d9445bb422f7fac2931924f3f449bf",
    "vane_horn_1": "4bbd020798eb4fd52553299d43670becc83486936e211e56041acbf5bb3c02a3",
    "vane_horn_2": "c27fcada3c30571dd5a32ac198fbc142770f55aa2ce0f6356a30262df29496e7",
    "vane_horn_3": "096d2cf687e392a48b6c29d6771c3ee128e6b04776081eefbbdc0b01a7ad60d2",
    "vane_horn_4": "819ea3f984b011e7f118e210f5031ef433b114cf1058ada9be686e4d95648e45",
    "latch_arm": "835d2a487515cc327646989b1d3cdbdc7278559634a7377a140fb30197a10694",
    "launch_pad": "62cb8e1d404974275f7433ea7a06c5c497b8404772043c068c6f9fee96a3f226",
    "edf_unit": "7e85a765b5aa09c5293eba0afdd4d7574390e57673d735d69c44d4cbc74538c2",
    "battery": "f0909f1dc06705f94c8c0d318dd067267ced3192e70340e1cfd5992d917718e3",
    "esc": "60945b7326d10e153c3775faf9d979b6cb87932e6968531313fcb1e550971305",
    "fc": "7c3caecd19dfdb5937e7c8074ce209ca59038dda5a7ac1223aec0c4bde772ad7",
    "misc_electronics": "d415935c3961d153b3892a9e524a59acb1ed133f5a98a085ef81d16d60ee8325",
    "servo_vane_1": "2654da43b14d4c20b86797f9604a8667b455b90addd1a758b9548247f20bfc2d",
    "servo_vane_2": "7b7d4e7f2811e076b02733e58b0cce5aa0463075ebdb00ed261ec35f1866e87a",
    "servo_vane_3": "adea69623553f7bb75d9208cebac994038b1315d84d0243414829575d70deb23",
    "servo_vane_4": "39f96b144f96a1810f708aab42d9d58aa9abb81b1d032c854ae8a7d4470c94b2",
    "servo_release": "32adf7e7ae7caeb93fa488738c6cede38d3f0459e7c2cc7b3620d3ac10624491",
    "servo_arm_1": "4cecc7d31a3a86af79bbfee3314e79eea3ef76648893998fefee096e75889208",
    "servo_arm_2": "6bc9ad67fbb95de24c7a39d6aa3e43ff9143b28b5ca0c931404316b2d49c100b",
    "servo_arm_3": "2d1d95ed25f7cc14f29d1c25428981f74046848f784d468d89816fbd8976c253",
    "servo_arm_4": "0f95e38c585ab8967d7edd06773352f1684c674a2b278bf38c6ba21b5ac2d88d",
    "pushrod_1": "7eac0285a8c138d0c3e7f2ad41fa6dd5fd61b12debda8fa81da0664e94278ad9",
    "pushrod_2": "8c1ef472fb0a9e6752267c270386eb3cdfeb6ee38b105ec64255d16b922adaa9",
    "pushrod_3": "afe1a77115a6d91b5d02157441c68ab6350ce0e0ec1091dd5a4530930b1d994b",
    "pushrod_4": "5842f2481e529e79032402676b6c570b56c0fa1e6cdd7b3949db9bcc704d32d6",
    "cansat_payload": "2f11d3de615e84fd67c5ccf1130f9a072d0f71a551b7fb264b030438721aea40",
    "spring": "5c1320efccb38ec4c9e89ac51101ff16483d93d9e0f54a8a3273be9e5f780764",
    "chute_pack": "22b271fc89b7708563a413ad36f2b526865e3c91b5fb40dda024d27e76085280",
    "joint_screws_inserts": "778a066c584dfc9962d50b57b681b2cdbf98027d467bebbb3d3c4ebad0778452",
    "pushrods_horns_clevises": "62f3c50dd6b97518b7ed496ffb3edc602cbb8cf57ec2587a7d1d54027ad8320e",
}
KAIST_V04_VOLUME = {"propulsion_module": 421705.30716847174, "intake_module": 81888.47744687973,
                    "avionics_module": 163895.26024656883}
KAIST_V04_TOTAL = (4251.882872943017, 429.6034145908286)   # mass_g, com z
# v0.5d contract: KAIST = v0.4 except the tray stops and the blind propulsion joint holes (both presets)
KAIST_V05D_CHANGED = ("avionics_module", "propulsion_module")
# The v0.5d lip-pocket clearance (1.5, floating tab-mount bell) is overridden back to 0.5 for KAIST in
# alt_presets, so the KAIST intake_module is v0.4 again (test_kaist_intake_module_is_v04).
KAIST_CONTRACT_LEAKS: tuple[str, ...] = ()
# v0.5d builder fix that also changes the v0.4 rebuild: the avionics vent-slot cutter no longer bites the rail-rib ends.
# Upper bound of the restored material: 4 slots x 2 rib-pairs' worth (8 mm wide) x 2 mm x vent height 40 mm.
KAIST_V05D_BUILDER_FIXES = {"avionics_module": 4 * 8.0 * 2.0 * 40.0}
KAIST_V04_RESTORE: dict = {}
# v0.6 contract (both presets, two ATmega128A): payload_module gains the four COM-board bosses + blind holes (exactly
# _com_boss_added_mm3 / _com_hole_removed_mm3; its v0.4 rebuild without the v0.6 keys must stay bit-identical), and one
# new placeholder, the COM board, joins the report (params mass at the Layout box centre).  Nothing else may change.
KAIST_V06_CHANGED = ("payload_module",)
KAIST_V06_NEW_PARTS = ("com",)
# v0.6.1 contract (both presets, printability fix: user 2026-10-15, the 1.2 mm struts broke when the supports came off):
# the intake module's windowed band Z [z_shelf_top, z_intake_bulkhead_bottom] becomes window_wall_t thick (inward) with
# radial-sector windows, and strut_w 14 -> 18.  The v0.4 intake (1.2 mm band, two through-slab windows for 14 mm struts)
# is rebuilt in this file from primitives (_intake_v04 - the builder no longer has that code path) and pinned to its
# v0.4 sha; outside the band the current module equals it, inside the band both equal their analytic volumes, the old
# band lies inside the new one, and the report's mass / COM move by exactly that analytic difference.
KAIST_V061_CHANGED = ("intake_module",)
KAIST_V04_STRUT_W = 14.0        # intake_module.strut_w of v0.4 .. v0.6.0
# v0.6.2 contract (both presets, user 2026-10-15): the intake band also keeps the window gussets under the truncated-gable
# lower ends (Layout.gusset_volume_mm3 at gusset_centroid_z, in _band_volume_moment); electronics.misc 120 -> 100 g on
# KAIST (its box unchanged: the entry rebuilt with the old mass is the v0.4 one) and the new placeholder radio_log (20 g,
# Layout box) - total electronics mass unchanged.  Nothing else may change.
KAIST_V062_NEW_PARTS = ("radio_log",)
KAIST_V062_MASS_CHANGED = {"misc_electronics": 120.0}   # placeholder -> its v0.4 .. v0.6.1 params mass (g)
# v0.6.3 contract (both presets, three ATmega128A): electronics.fc is electronics.nav - the placeholder keeps its box and
# mass under the new name (its entry is the v0.4 "fc" entry bit for bit) - and the new placeholder fcu (params mass at the
# Layout box centre, on the intake bulkhead) joins the report.  Nothing else may change.
KAIST_V063_RENAMED = {"nav": "fc"}      # new name -> v0.4 name
KAIST_V063_NEW_PARTS = ("fcu",)
# v0.8 contract (both presets, GPS pod): payload_module gains the pod / raceway / pocket (material added only) and
# avionics_module loses the lead entry hole (removed only); both rebuilt with_gps=False are their pre-v0.8 solids, which the
# v0.4 / v0.5d / v0.6 deltas are judged on; the GPS delta itself is measured (boolean difference), its mass and COM shift
# included in the totals check; four new report entries.  Nothing else may change.
KAIST_V08_CHANGED = ("payload_module", "avionics_module")
KAIST_V08_NEW_PARTS = ("gps_pod_lid", "gps", "gps_ground_plane", "gps_lead")
# v0.8.1 contract (both presets, GNS board): avionics_module also gains the four GNS bosses (added) and their tap holes
# (removed) - still measured as part of the with_gps=False delta (the GNS features follow with_gps) - and the new
# placeholder gns (Layout box, Layout.gns_mass_g) joins the report.  Nothing else may change.
KAIST_V081_NEW_PARTS = ("gns",)


def _intake_v04(L, strut_w: float):
    """The v0.4 .. v0.6.0 intake module of a lip_clamp (KAIST) preset, rebuilt with the builder's old code path -
    same primitives, same order: the 1.2 mm band r body_id/2..body_r and two orthogonal through-slab windows of width
    2 body_r sin(window_angle / 2), window_angle = 2 pi / strut_count - strut_w / body_r, the first slab along the first
    window theta."""
    assert not L.tabs and not L.servo_leads
    parts = [
        rg._tube(L.body_r, L.body_id / 2, L.z_intake_bottom, L.z_intake_skirt_top),
        rg._tube(L.body_r, L.lip_pocket_id / 2, L.z_intake_skirt_top, L.z_lip_pocket_top),
        rg._tube(L.body_r, L.shelf_id / 2, L.z_lip_pocket_top, L.z_shelf_top),
        rg._tube(L.body_r, L.body_id / 2, L.z_shelf_top, L.z_intake_bulkhead_bottom),
        rg._cyl_z(L.body_r, L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top),
        rg._tube(L.spigot_od / 2, L.spigot_id / 2, L.z_intake_spigot_bottom, L.z_intake_spigot_top),
    ]
    body = rg._fuse(parts)
    wa = 2 * math.pi / L.strut_count - strut_w / L.body_r
    big, w = L.body_r + 2 * rg.OVERLAP, L.body_r * math.sin(wa / 2)
    cutters = [rg._rot_z(rg._box(-big, big, -w, w, L.z_window_bottom, L.z_window_top), L.window_thetas[0]),
               rg._rot_z(rg._box(-w, w, -big, big, L.z_window_bottom, L.z_window_top), L.window_thetas[0])]
    zb0, zb1 = L.z_intake_bulkhead_bottom - rg.OVERLAP, L.z_intake_bulkhead_top + rg.OVERLAP
    cutters.append(rg._cyl_z(L.cable_hole_d / 2, zb0, zb1))
    for th in L.vent_hole_thetas:
        c = rg._polar(L.vent_hole_r, 0.0, th, 0.0)
        cutters.append(rg._cyl_z(L.vent_hole_d / 2, zb0, zb1, c.x, c.y))
    for th in L.joint_thetas:
        cutters.append(rg._radial_cyl(L.p["body"]["joint_hole_outer_d"], L.body_id / 2 - 2 * rg.OVERLAP,
                                      L.body_r + 2 * rg.OVERLAP, th, L.z_intake_bottom + L.joint_screw_dz))
        cutters.append(rg._radial_cyl(L.p["body"]["joint_hole_spigot_d"], L.spigot_id / 2 - 2 * rg.OVERLAP,
                                      L.spigot_od / 2 + 2 * rg.OVERLAP, th, L.z_intake_spigot_bottom + L.joint_screw_dz))
    return rg._cut(body, cutters)


def _band_volume_moment(L, strut_w_v04: float | None = None) -> tuple[float, float]:
    """Analytic (volume mm^3, Z moment mm^4) of the intake's windowed band Z [z_shelf_top, z_intake_bulkhead_bottom]:
    v0.6.1 (default) the ring r_band_in..body_r minus strut_count radial sectors of window_angle over the windows;
    with ``strut_w_v04`` the v0.4 band - ring body_id/2..body_r minus the two orthogonal slabs of half width
    a = body_r sin(window_angle / 2) (each slab takes S(R) - S(r) of the annulus, S(rho) = 2 (a sqrt(rho^2 - a^2) +
    rho^2 asin(a / rho)); the slabs' common square lies inside the bore, asserted)."""
    R, H = L.body_r, L.z_intake_bulkhead_bottom - L.z_shelf_top
    zc_band, zc_win = 0.5 * (L.z_shelf_top + L.z_intake_bulkhead_bottom), 0.5 * (L.z_window_bottom + L.z_window_top)
    gus, gus_z = 0.0, 0.0
    if strut_w_v04 is None:
        ri = L.r_band_in
        win = L.strut_count * 0.5 * math.radians(L.window_angle_deg) * (R * R - ri * ri) * L.window_h
        if L.has_window_gable:      # v0.6.2: the gussets stay under the gable (Layout's integral, checked on the solid)
            gus, gus_z = L.gusset_volume_mm3, L.gusset_centroid_z
    else:
        ri = L.body_id / 2
        a = R * math.sin(0.5 * (2 * math.pi / L.strut_count - strut_w_v04 / R))
        assert a * math.sqrt(2.0) < ri and L.strut_count == 4

        def s_disc(rho: float) -> float:
            return 2.0 * (a * math.sqrt(rho * rho - a * a) + rho * rho * math.asin(a / rho))
        win = 2.0 * (s_disc(R) - s_disc(ri)) * L.window_h
    ring = math.pi * (R * R - ri * ri) * H
    return ring - win + gus, ring * zc_band - win * zc_win + gus * gus_z


def _entry_sha(entry: dict) -> str:
    return hashlib.sha256(json.dumps(entry, sort_keys=True).encode()).hexdigest()


def _printed_entry(name: str, shape, L) -> dict:
    """Report entry (SPEC 6) of a printed module, exactly as build_rocket.make_report writes it."""
    return br.part_entry(rg.Part(name, "printed", name, shape, rg.PRINTED_MODULE[name], rg.PRINT_ORIENTATION[name],
                                 density_g_cm3=L.p["printer"]["density_g_cm3"]))


def test_kaist_lip_clamp_matches_v04(built):
    """KAIST (lip_clamp) vs the v0.4 report: every part not changed by the v0.5d / v0.6 / v0.6.1 contracts is
    bit-identical; the v0.4 solids of the changed modules, rebuilt in-process from the same Layout without the v0.5d and
    v0.6 params (the intake: _intake_v04 with the v0.4 strut width), are bit-identical to the v0.4 entries, and the
    current modules differ from them by exactly the analytic tray-stop / hole-floor / COM-boss / COM-hole / intake-band
    (v0.6.2: with the window gussets) volumes (boolean differences and precise volumes, 1e-3 relative); the new report
    entries are the v0.6 ``com`` and the v0.6.2 ``radio_log`` placeholders (params mass at the Layout box centre) and
    misc_electronics carries the v0.6.2 params mass on its v0.4 box; the report's mass and COM moved by exactly that
    material plus those placeholders."""
    preset, L, parts, report = built
    if L.tabs:
        pytest.skip("tab-mount preset")
    assert L.p["meta"]["preset"] == "KAIST_90mm_12S"
    old_names = set(KAIST_V063_RENAMED.values())
    assert set(report["parts"]) == ((set(KAIST_V04_SHA256) - old_names) | set(KAIST_V063_RENAMED) | set(KAIST_V06_NEW_PARTS)
                                    | set(KAIST_V062_NEW_PARTS) | set(KAIST_V063_NEW_PARTS) | set(KAIST_V08_NEW_PARTS)
                                    | set(KAIST_V081_NEW_PARTS))
    changed = (set(KAIST_V05D_CHANGED) | set(KAIST_V06_CHANGED) | set(KAIST_V061_CHANGED) | set(KAIST_CONTRACT_LEAKS)
               | set(KAIST_V062_MASS_CHANGED) | set(KAIST_V08_CHANGED))
    rename = {old: new for new, old in KAIST_V063_RENAMED.items()}      # v0.4 name -> report name
    same = [n for n in KAIST_V04_SHA256 if n not in changed]
    diff = [n for n in same if _entry_sha(report["parts"][rename.get(n, n)]) != KAIST_V04_SHA256[n]]
    assert not diff, [(n, report["parts"][rename.get(n, n)]["volume_mm3"]) for n in diff]
    L04 = _layout_pre_v05d(preset, restore=KAIST_V04_RESTORE, drop_v06=True)
    assert not L04.has_com_board and not L04.has_gps_pod
    zh = L.z_prop_spigot_bottom + L.joint_screw_dz
    # v0.6.1 band: the old band lies inside the new one (new struts are wider at body_r and radial-sided, the old ones
    # narrow inward), so the change only adds material: new band - v0.4 band, analytic, at its own Z centroid
    assert L.strut_w >= KAIST_V04_STRUT_W and L.r_band_in <= L.body_id / 2
    v_new, m_new = _band_volume_moment(L)
    v_old, m_old = _band_volume_moment(L, KAIST_V04_STRUT_W)
    # (added, its Z centroid, removed, its Z centroid) per changed module, analytic
    expect = {
        "avionics_module": (_tray_stop_added_mm3(L), 0.5 * (L.z_tray_stop_bottom + L.z_tray_bottom), 0.0, 0.0),
        "propulsion_module": (_blind_hole_floor_mm3(L), zh, 0.0, 0.0),
        "payload_module": (_com_boss_added_mm3(L), L.z_com_boss_tip + 0.5 * L.com_boss_h,
                           _com_hole_removed_mm3(L), L.z_com_boss_top + 0.5 * (L.com_hole_depth - L.com_boss_h)),
        "intake_module": (v_new - v_old, (m_new - m_old) / (v_new - v_old), 0.0, 0.0),
    }
    assert set(expect) == set(KAIST_V05D_CHANGED) | set(KAIST_V06_CHANGED) | set(KAIST_V061_CHANGED)
    v04_builders = {"intake_module": lambda L_: _intake_v04(L_, KAIST_V04_STRUT_W)}   # the builder lost this code path
    rho = L.p["printer"]["density_g_cm3"]
    dm, dmz, v_mod = 0.0, 0.0, 0.0
    for name, (add, z_add, rem, z_rem) in expect.items():
        old = v04_builders.get(name, rg.PRINTED_BUILDERS[name])(L04)
        if name in KAIST_V05D_BUILDER_FIXES:
            # the v0.5d vent-slot over-travel fix changes this module's v0.4 rebuild too: it may only ADD material
            # (the rail-rib ends the old cutter bit off), bounded by KAIST_V05D_BUILDER_FIXES[name] mm^3
            v04 = KAIST_V04_VOLUME[name]
            dv_fix = _precise_volume(old) - v04
            assert -REPORT_GPROP_REL_ERR * v04 <= dv_fix <= KAIST_V05D_BUILDER_FIXES[name], (name, dv_fix)
        else:
            entry04 = _printed_entry(name, old, L04)
            assert _entry_sha(entry04) == KAIST_V04_SHA256[name], f"{name}: v0.4 rebuild differs"
            v04 = entry04["volume_mm3"]          # = the v0.4 report's volume (same sha)
            if name in KAIST_V04_VOLUME:
                assert v04 == KAIST_V04_VOLUME[name]
        v_mod += v04
        full = parts[name].shape
        # v0.8: the analytic v0.5d / v0.6 / v0.6.1 deltas are judged on the module without the GPS features; the GPS
        # delta (pod: added only; entry hole: removed only) is measured and carried into the mass / COM bookkeeping
        new = rg.PRINTED_BUILDERS[name](L, with_gps=False) if name in KAIST_V08_CHANGED else full
        g_add, g_rem, z_gadd, z_grem = 0.0, 0.0, 0.0, 0.0
        if name in KAIST_V08_CHANGED:
            ga, gr = full.cut(new), new.cut(full)
            g_add, g_rem = _volume_or_0(ga), _volume_or_0(gr)
            z_gadd = ga.Center().z if g_add > 0 else 0.0
            z_grem = gr.Center().z if g_rem > 0 else 0.0
            if name == "payload_module":
                assert g_rem < 1e-6 < g_add, (name, g_add, g_rem)
            elif L.has_gns:          # v0.8.1: the GNS bosses add, the entry hole (and the tap holes' wall part) remove
                assert g_add > 1e-6 and g_rem > 1e-6, (name, g_add, g_rem)
            else:
                assert g_add < 1e-6 < g_rem, (name, g_add, g_rem)
        tol = 1e-3 * max(add, rem)
        assert _volume_or_0(new.cut(old)) == pytest.approx(add, abs=max(tol, 1e-6)), name
        assert _volume_or_0(old.cut(new)) == pytest.approx(rem, abs=max(tol, 1e-6)), name
        assert _precise_volume(new) - _precise_volume(old) == pytest.approx(add - rem, abs=max(tol, 1e-6)), name
        # the report (default GProp integration, <= REPORT_GPROP_REL_ERR relative) agrees to that precision
        base_v = _precise_volume(old) if name in KAIST_V05D_BUILDER_FIXES else v04
        dv_rep = report["parts"][name]["volume_mm3"] - base_v
        assert abs(dv_rep - (add - rem + g_add - g_rem)) <= tol + REPORT_GPROP_REL_ERR * v04, (name, dv_rep, add - rem)
        fix = base_v - v04
        dm += rho * (add - rem + fix + g_add - g_rem) / 1000.0
        dmz += (rho * (add * z_add - rem * z_rem + g_add * z_gadd - g_rem * z_grem) / 1000.0
                + rho * fix / 1000.0 * (L.z_vent_slot_bottom + 0.5 * L.vent_slot_h))
    # v0.6 new placeholder: the COM board box with its params mass, in the avionics module
    for name in KAIST_V06_NEW_PARTS:
        e = report["parts"][name]
        assert parts[name].kind == "placeholder" and parts[name].module == "avionics_module"
        assert e["mass_g"] == pytest.approx(L.p["electronics"]["com"]["mass_g"], abs=1e-9)
        box = list(zip(L.com_board_x, L.com_board_y, L.com_board_z))
        assert e["bbox_mm"] == [pytest.approx(list(box[0]), abs=1e-6), pytest.approx(list(box[1]), abs=1e-6)]
        assert e["com_mm"] == pytest.approx([0.0, 0.0, L.com_board_center_z], abs=1e-6)
        dm += e["mass_g"]
        dmz += e["mass_g"] * L.com_board_center_z
    # v0.6.2: misc_electronics is the v0.4 box with the new params mass (rebuilt with the old mass it is the v0.4 entry),
    # and the radio/log box is new (Layout box, params mass)
    for name, m_old in KAIST_V062_MASS_CHANGED.items():
        e = report["parts"][name]
        assert e["mass_g"] == pytest.approx(L.p["electronics"]["misc"]["mass_g"], abs=1e-9) and e["mass_g"] < m_old
        assert _entry_sha(br.part_entry(dataclasses.replace(parts[name], mass_g=m_old))) == KAIST_V04_SHA256[name]
        dm += e["mass_g"] - m_old
        dmz += (e["mass_g"] - m_old) * e["com_mm"][2]
    for name in KAIST_V062_NEW_PARTS:
        e = report["parts"][name]
        assert parts[name].kind == "placeholder" and parts[name].module == "avionics_module"
        assert e["mass_g"] == pytest.approx(L.p["electronics"]["radio_log"]["mass_g"], abs=1e-9)
        assert e["com_mm"] == pytest.approx(list(L.radio_log_center), abs=1e-6)
        dm += e["mass_g"]
        dmz += e["mass_g"] * L.radio_log_center[2]
    assert sum(KAIST_V062_MASS_CHANGED.values()) == pytest.approx(
        L.p["electronics"]["misc"]["mass_g"] + L.p["electronics"]["radio_log"]["mass_g"])   # electronics mass unchanged
    # v0.6.3: the FCU board is new (Layout box on the intake bulkhead, params mass)
    for name in KAIST_V063_NEW_PARTS:
        e = report["parts"][name]
        assert parts[name].kind == "placeholder" and parts[name].module == "avionics_module"
        assert e["mass_g"] == pytest.approx(L.p["electronics"]["fcu"]["mass_g"], abs=1e-9)
        assert e["com_mm"] == pytest.approx(list(L.fcu_center), abs=1e-6)
        dm += e["mass_g"]
        dmz += e["mass_g"] * L.fcu_center[2]
    # v0.8: the GPS lid, placeholders and lead are new (report entries at their own COM)
    for name in KAIST_V08_NEW_PARTS:
        e = report["parts"][name]
        assert parts[name].module == "payload_module"
        dm += e["mass_g"]
        dmz += e["mass_g"] * e["com_mm"][2]
    # v0.8.1: the GNS board is new (Layout box on its bosses, Layout.gns_mass_g)
    for name in KAIST_V081_NEW_PARTS:
        e = report["parts"][name]
        assert parts[name].kind == "placeholder" and parts[name].module == "avionics_module"
        assert e["mass_g"] == pytest.approx(L.gns_mass_g, abs=1e-9)
        assert e["com_mm"] == pytest.approx(list(L.gns_center), abs=1e-6)
        dm += e["mass_g"]
        dmz += e["mass_g"] * L.gns_center[2]
    m0, z0 = KAIST_V04_TOTAL
    m_new, z_new = report["totals"]["mass_g"], report["totals"]["com_mm"][2]
    assert abs((m_new - m0) - dm) <= 1e-3 * abs(dm) + rho * REPORT_GPROP_REL_ERR * v_mod / 1000.0, (m_new - m0, dm)
    assert z_new == pytest.approx((m0 * z0 + dmz) / (m0 + dm), abs=1e-3)


def test_kaist_intake_module_is_v04_outside_the_band(built):
    """v0.5d contract (alt_presets keeps the 0.5 pocket clearance) + v0.6.1: the KAIST intake_module is the v0.4 one
    (its rebuild _intake_v04 is pinned to the v0.4 sha) everywhere outside the windowed band Z [z_shelf_top,
    z_intake_bulkhead_bottom]; inside it the current module and the v0.4 one each hold exactly their analytic band
    volume, and the v0.4 band lies inside the new one."""
    _, L, parts, report = built
    if L.tabs:
        pytest.skip("tab-mount preset")
    old = _intake_v04(L, KAIST_V04_STRUT_W)
    assert _entry_sha(_printed_entry("intake_module", old, L)) == KAIST_V04_SHA256["intake_module"]
    assert _entry_sha(report["parts"]["intake_module"]) != KAIST_V04_SHA256["intake_module"]     # it did change
    new = parts["intake_module"].shape
    region = rg._cyl_z(L.body_r + 1.0, L.z_shelf_top, L.z_intake_bulkhead_bottom)
    new_out, old_out = new.cut(region), old.cut(region)
    assert _volume_or_0(new_out.cut(old_out)) < 1e-6 and _volume_or_0(old_out.cut(new_out)) < 1e-6
    assert _precise_volume(new_out) == pytest.approx(_precise_volume(old_out), rel=1e-9)
    assert _volume_or_0(new.intersect(region)) == pytest.approx(_band_volume_moment(L)[0], rel=1e-6)
    assert _volume_or_0(old.intersect(region)) == pytest.approx(_band_volume_moment(L, KAIST_V04_STRUT_W)[0], rel=1e-6)
    assert _volume_or_0(old.cut(new)) < 1e-6



# --------------------------------------------------------------------------- #
# v0.6, both presets: COM board (second ATmega128A) under the payload bulkhead (SPEC 5.3 / 6, check C22)
# --------------------------------------------------------------------------- #
def test_com_mount_bosses_and_blind_holes(built):
    """Four bosses Ø com_boss_d x com_boss_h on the com_pattern square (centred, edges along x / y) under the payload
    bulkhead, each with a clear blind Ø com_hole_d hole com_hole_depth deep from its tip and bulkhead material above
    it; nothing else of payload_module changed (boolean differences against the same module without the v0.6 keys are
    the analytic boss / hole volumes); the ``com`` placeholder is the Layout box with the params mass, hanging from the
    boss tips (touching, no overlap)."""
    preset, L, parts, report = built
    assert L.has_com_board and len(L.com_boss_xy) == 4
    assert sorted(L.com_boss_xy) == sorted((sx * L.com_pattern / 2, sy * L.com_pattern / 2)
                                           for sx in (-1, 1) for sy in (-1, 1))
    assert L.z_com_boss_top == L.z_payload_bulkhead_bottom and L.com_board_z[1] == pytest.approx(L.z_com_boss_tip)
    assert 0.0 < L.com_hole_skin == pytest.approx(L.z_payload_bulkhead_top - L.z_com_boss_tip - L.com_hole_depth)
    pay = parts["payload_module"].shape
    old = rg.payload_module(_layout_pre_v06(preset))
    assert _volume_or_0(pay.cut(old)) == pytest.approx(_com_boss_added_mm3(L), rel=1e-6)
    assert _volume_or_0(old.cut(pay)) == pytest.approx(_com_hole_removed_mm3(L), rel=1e-6)
    e = 0.05
    for x, y in L.com_boss_xy:
        cyl = lambda r, z0, z1, x=x, y=y: rg._cyl_z(r, z0, z1, x, y)   # noqa: E731
        ring = cyl(L.com_boss_d / 2 - e, L.z_com_boss_tip + e, L.z_com_boss_top - e).cut(
            cyl(L.com_hole_d / 2 + e, L.z_com_boss_tip, L.z_com_boss_top))
        assert rg.intersection_volume(ring, pay) == pytest.approx(ring.Volume(), rel=1e-6)          # solid boss
        outside = cyl(L.com_boss_d / 2 + 1.0, L.z_com_boss_tip + e, L.z_com_boss_top - e).cut(
            cyl(L.com_boss_d / 2 + e, L.z_com_boss_tip, L.z_com_boss_top))
        assert _probe_free(pay, outside)                                                            # boss Ø exact
        assert _probe_free(pay, cyl(L.com_boss_d / 2 + 1.0, L.z_com_boss_tip - 1.0, L.z_com_boss_tip - e))   # tip
        assert _probe_free(pay, cyl(L.com_hole_d / 2 - e, L.z_com_boss_tip - 1.0, L.z_com_hole_end - e))    # open hole
        ring_h = cyl(L.com_hole_d / 2 + e, L.z_com_boss_tip + e, L.z_com_hole_end - e).cut(
            cyl(L.com_hole_d / 2 - e, L.z_com_boss_tip, L.z_com_hole_end))
        assert rg.intersection_volume(ring_h, pay) == pytest.approx(
            math.pi * ((L.com_hole_d / 2 + e) ** 2 - (L.com_hole_d / 2) ** 2) * (L.z_com_hole_end - L.z_com_boss_tip - 2 * e),
            rel=1e-6)                                                                               # hole Ø exact
        floor = cyl(L.com_hole_d / 2, L.z_com_hole_end + e, L.z_payload_bulkhead_top - e)
        assert rg.intersection_volume(floor, pay) == pytest.approx(floor.Volume(), rel=1e-6)        # blind
    com = parts["com"]
    assert com.kind == "placeholder" and com.module == "avionics_module" and com.color == rg.COLORS["com"]
    entry = report["parts"]["com"]
    assert entry["mass_g"] == pytest.approx(L.p["electronics"]["com"]["mass_g"], abs=1e-9)
    bb = com.shape.BoundingBox()
    assert (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax) == pytest.approx(
        (*L.com_board_x, *L.com_board_y, *L.com_board_z), abs=1e-9)
    assert (bb.xlen, bb.ylen, bb.zlen) == pytest.approx(tuple(L.p["electronics"]["com"]["dims_lwh"]), abs=1e-9)
    assert rg.intersection_volume(com.shape, pay) < 1e-6 and com.shape.distance(pay) < 1e-6           # seated
    assert "com" not in rg.POST_EJECTION_EXCLUDED                                                    # stays aboard


@pytest.fixture(scope="session")
def default_build():
    return br.build(None, export=False, verbose=False)


def test_c22_fails_without_the_mount(default_build):
    """C22 is positive evidence: it FAILs for a payload module without the bosses, with solid (undrilled) bosses, with
    holes run through the bulkhead, and for the board 1 mm lower (tray clearance and seat); PASS on the real model."""
    L, parts, report = default_build
    assert cg.Checker(L, parts, report).c22_com_board().status == cg.PASS
    pay = parts["payload_module"].shape

    def status(**shapes) -> str:
        p2 = dict(parts)
        for name, shp in shapes.items():
            p2[name] = dataclasses.replace(parts[name], shape=shp)
        return cg.Checker(L, p2, report).c22_com_board().status

    assert status(payload_module=rg.payload_module(_layout_pre_v06(None))) == cg.FAIL          # no bosses
    plugs = [rg._cyl_z(L.com_hole_d / 2, L.z_com_boss_tip, L.z_com_hole_end, x, y) for x, y in L.com_boss_xy]
    assert status(payload_module=rg._fuse([pay] + plugs)) == cg.FAIL                            # bosses not drilled
    through = [rg._cyl_z(L.com_hole_d / 2, L.z_com_hole_end - 0.1, L.z_payload_bulkhead_top + 1.0, x, y)
               for x, y in L.com_boss_xy]
    assert status(payload_module=pay.cut(*through)) == cg.FAIL                                   # holes not blind
    low = parts["com"].shape.translate(rg.V(0.0, 0.0, -1.0))
    assert status(com=low) == cg.FAIL                                                            # board 1 mm low


# --------------------------------------------------------------------------- #
# v0.6.3, both presets: printed parts = v0.6.2, fc -> nav, FCU board on the intake bulkhead (check C25)
# --------------------------------------------------------------------------- #
# sha256 of every printed part's v0.6.2 report entry (cad/exports/cadquery_report*.json of params sha a07c805b...,
# cadquery 2.8.0, the run that exported the STLs being printed): canonical JSON as _entry_sha
V062_PRINTED_SHA256 = {
    "BUDGET_90mm_6S": {
        "propulsion_module": "f617252927d21aec118d98a0e77e118b43a94469af18ce0f75d60d460564be0c",
        "intake_module": "8241b5c2fdc082db9c161a001504f737c3d5d4d90730b97fefc795e552d3a528",
        "avionics_module": "01e12971baa992fd7c6d36b1c4c430dd9c1309852f0c6f8584d6e590d4cdac5b",
        "avionics_tray": "83f25da4a07fa07eaea52a0f465eea2a822689ea1dce3ca42ef42e6817b9c574",
        "payload_module": "4ec1d78873f0e104a94d77e44f08bc169e159726ffdccfd48c4621f218b1e022",
        "piston": "d8836752ac0d71fb3e2fe2975c4486001716ffc3909825826bdab3d157440396",
        "nose_cone": "01e46a3d9b69eb99d8339a1f83dc71de79bd1b0a0c09fc2c80c5914bbc11cad6",
        "vane_1": "f5a8cedf0a3af67968b789ca9031c58530b401a0332f773c1b78f13c7f745c43",
        "vane_2": "19a1091b8d47d3aa30da1db73425fb5e34246b02a098778b5279622a469c9143",
        "vane_3": "440e2133b7749fbce7f2708ce4a23776b7f12df9b76b9d607d3c322740d5ce73",
        "vane_4": "7abc56bbdac1b2348c48d3d7dbb6f50db6d9445bb422f7fac2931924f3f449bf",
        "vane_horn_1": "4bbd020798eb4fd52553299d43670becc83486936e211e56041acbf5bb3c02a3",
        "vane_horn_2": "c27fcada3c30571dd5a32ac198fbc142770f55aa2ce0f6356a30262df29496e7",
        "vane_horn_3": "096d2cf687e392a48b6c29d6771c3ee128e6b04776081eefbbdc0b01a7ad60d2",
        "vane_horn_4": "819ea3f984b011e7f118e210f5031ef433b114cf1058ada9be686e4d95648e45",
        "latch_arm": "a695de5ba5b883cce47c530c7c77bb3acaeb7b1c4d57884fac24c3e2b3d5b809",
        "launch_pad": "62cb8e1d404974275f7433ea7a06c5c497b8404772043c068c6f9fee96a3f226",
    },
    "KAIST_90mm_12S": {
        "propulsion_module": "9807c9ce42df306d7abd22c3d9c14b45d2c1712f5f9f227be7497fa34b29aea9",
        "intake_module": "80d1b800ac85c21e846148d489f13f94beb43453aad822d97f0ca3bccf13a605",
        "avionics_module": "1002b037f9baac3408ff6df3029000751d76e3834f1d9f06a9a155e0579f6423",
        "avionics_tray": "07e89d8c75f772b456090342a515abe0b0fc7e05eab57bc0c3b539ba26413e13",
        "payload_module": "a3e674469c150795201b241a805c77215d3b9f2c47a8bf1c0d2d1081af8f38f1",
        "piston": "bb490180596217511dddc5b847c9f9d4c9ec887900b6de3b5dfc87b27607d1eb",
        "nose_cone": "99cab9b89bfbe0f2002f872e9975e43c50b59c374b0c471ef5acba1c6dec581e",
        "vane_1": "f5a8cedf0a3af67968b789ca9031c58530b401a0332f773c1b78f13c7f745c43",
        "vane_2": "19a1091b8d47d3aa30da1db73425fb5e34246b02a098778b5279622a469c9143",
        "vane_3": "440e2133b7749fbce7f2708ce4a23776b7f12df9b76b9d607d3c322740d5ce73",
        "vane_4": "7abc56bbdac1b2348c48d3d7dbb6f50db6d9445bb422f7fac2931924f3f449bf",
        "vane_horn_1": "4bbd020798eb4fd52553299d43670becc83486936e211e56041acbf5bb3c02a3",
        "vane_horn_2": "c27fcada3c30571dd5a32ac198fbc142770f55aa2ce0f6356a30262df29496e7",
        "vane_horn_3": "096d2cf687e392a48b6c29d6771c3ee128e6b04776081eefbbdc0b01a7ad60d2",
        "vane_horn_4": "819ea3f984b011e7f118e210f5031ef433b114cf1058ada9be686e4d95648e45",
        "latch_arm": "835d2a487515cc327646989b1d3cdbdc7278559634a7377a140fb30197a10694",
        "launch_pad": "62cb8e1d404974275f7433ea7a06c5c497b8404772043c068c6f9fee96a3f226",
    },
}


V08_CHANGED_PRINTED = ("payload_module", "avionics_module")   # v0.8: the GPS pod / raceway / pocket and the entry hole
V08_NEW_PRINTED = ("gps_pod_lid",)


# v0.9 (user decisions 2026-10-27, BUDGET preset only - KAIST keeps the pushrod linkage, its 45 deg fins and the solid
# mass model): RE-PINNED because the HARDWARE changed, not to hide a regression.  Decision 1 (direct-drive vanes): the
# propulsion module gets the four fin-root fairings, keels, lead pipes, pockets and cover tap holes instead of the pods
# (fins at 0/90/180/270), the vane shaft is 1.5 mm longer with a longer D-flat (coupler); decision 3 (motor leads): the
# propulsion wire slot 14 -> 22 mm, the intake's open 22 x 11 groove, 14 mm gate, 4 mm side walls, roof wedge, 22 x 11
# bulkhead hole; the launch pad's legs turned 45 deg to stay under the fins.  Before / after (solid volume x 1.31): propulsion
# 437.71 -> 443.83 g, intake 132.98 -> 135.90 g, vane 4.27 -> 4.30 g, launch pad 356.80 -> 356.80 g (rotated only).  New
# printed parts vane_coupler x4 (0.88 g), servo_cover x4 (3.38 g), wire_cover (2.19 g); vane_horn x4 removed.  Decision 2
# (mass calibration): the report's printed masses come from the slicer table, so every BUDGET report entry differs from its
# solid entry; the pins below are the SOLID entries (br.part_entry without the mass model), which carry the geometry.
V09_CHANGED_PRINTED = {"BUDGET_90mm_6S": ("propulsion_module", "intake_module", "vane_1", "vane_2", "vane_3", "vane_4",
                                          "launch_pad")}
V09_REMOVED_PRINTED = {"BUDGET_90mm_6S": ("vane_horn_1", "vane_horn_2", "vane_horn_3", "vane_horn_4")}
V09_SOLID_SHA256 = {"BUDGET_90mm_6S": {
    # v0.9 fix round re-pin (HARDWARE changed, reviewer defects; CAD-1 pins ae8384af / 804810bf / 9c354871.. / d72b43f8..):
    # propulsion - hinge bore 5.4 -> 6.0 for the brass bushing, 22 mm wire slot's inner face moved to the chord (no sliver):
    #   338801.3 -> 338499.4 mm^3 (solid 443.83 -> 443.43 g, slicer 350.08 -> 350.00 g);
    # intake - lip-pocket slit, 3 motor-lead tie anchors + tunnels, groove skirt cut from the bore chord, bulkhead hole
    #   22 x 11 at r 41-52 -> 20 x 11 at r 42-53: 103738.9 -> 104567.1 mm^3 (slicer 129.56 -> 130.16 g);
    # vanes - D-flat on the +v (service) side instead of -Z, shaft end 0.5 -> 1.5 mm short of the hub end: 3279.4 -> 3262.5;
    # couplers - horn socket (slot + hub relief) instead of a screwed flange, set-screw tap, round bore past the shaft:
    #   670.7 -> 711.9 mm^3 (slicer 0.87 -> 0.92 g)
    # v0.9 lead-exit re-pin (2026-10-29, HARDWARE changed - user: the motor leads start at the motor's lowest end and follow a
    # stator vane, so they leave the EDF under its rear rim; fix-round pin 34f9c380..): propulsion - the lead notch through the
    # liner / sleeve under the housing rim + lower slot (Layout.ml_low_cut_uz, 12.4 wide) below the unchanged 22 mm box:
    #   338499.4 -> 337865.8 mm^3 (solid 443.43 -> 442.60 g, slicer 350.00 -> 349.13 g)
    # v0.9 lead-exit PHOTO-ROUND re-pin (2026-10-29, HARDWARE changed - user photos docs/실물사진/EDF_모터선_출구_2026-10-29_1/2:
    # the leads leave the motor's front ring as a heat-shrink bundle along a stator vane and under the rear rim; round-1 pin
    # f1955d35.., history/v0.9_wire_round1): the notch re-cut for the bundle (bottom 3.84 mm deeper, chamfer / outer face
    # from Layout.ml_low_cut_uz): 337865.8 -> 337765.4 mm^3 (solid 442.60 -> 442.47 g, slicer 349.13 -> 349.01 g)
    # wire09c CLOCK-MOUNT re-pin (2026-11-03, HARDWARE changed - user 2026-10-30 'make the lead passage generous, I do not
    # want to measure'; photo-round pins 03be5a24.. / 4665d2f9.., history/v0.9_wire_round2): propulsion - no tab screws /
    # lugs / 22 mm slot / lead notch; 27 keyways round the circle with 3.0 mm floor pads (45 deg corbel), servo-column guide
    # walls, the lead passage cut and its blister + 45 deg V keel (Layout._clock_fan / _clock_seat):
    #   337765.4 -> 319224.9 mm^3 (solid 442.47 -> 418.18 g, slicer 349.01 -> 346.08 g);
    # intake - bulkhead hole r 42-53 -> 41-53.2 for the O4.3 leads of the generosity box (plenum throat 3.00, the bullet
    #   pass beside the 14 mm gate): 104567.1 -> 104452.9 mm^3 (slicer 130.16 -> 130.15 g)
    # wire09c FIX-ROUND re-pin (2026-11-04, HARDWARE changed - independent review F1: the 1.73 mm sleeve tooth beside the
    # 193.55 deg hover-seat keyway (and a 1.30 mm sliver beside the passage) failed tangential bending under the motor
    # torque on one tab; clock-mount pin dcde374f..): propulsion - keyways re-planned as a seat chain with every tooth
    # >= 5.06 mm (27 -> 25 keyways, 24.75 .. 333.50 deg; Layout._clock_seat), fan floor 0.1 mm lower for the turned
    # 12 x 5 ribbon (F5): 319224.9 -> 320358.7 mm^3 (slicer 346.08 -> 346.79 g); intake unchanged
    "propulsion_module": "456cef6b98e1d06b69feef08526bcc8d442aad19ddaf74e58d8b562eb04bd13a",
    "intake_module": "36b0f269d671e6f6121103afcb874b6b0885cc0b3b5008010e7b01de34308293",
    "vane_1": "84763707fae767fe4ecda29e3765fd155f9cfd64a8d7db1104257b8b05126018",
    "vane_2": "8b314dbfba01c4620bb745fbdbc7de88065b660350bf2c55fbc1f4bd2bf7490f",
    "vane_3": "4369a9e8ff1d8824f057b96f7da366b37424dbd4ea5d205b06e2d3db0645155e",
    "vane_4": "73ec00550159ab8b5da9add0d623599084a0e3b9326ef44f586e2919ccaf4f63",
    "vane_coupler_1": "4d3ad6ed367a34504d7d64e1fa3de3853bc42363d4dd1efaee3fcca62fd3c4dd",
    "vane_coupler_2": "104f51b22837d0c2a4021d8d587423f3aaabe0a53cbe364ae51aa3578194647d",
    "vane_coupler_3": "62f22ba11a77aef34a8842915f864cad4a588469a9b191dda18f2f1806b5ef09",
    "vane_coupler_4": "c79853301f7c12f5154a984779ead544a6ef31376661a6cb0ca4db111d932330",
    "servo_cover_1": "95aa1143bd81ac3f4bb7b7044d7a5078a40fc5e570f38c2e4b89c067b0230e27",
    "servo_cover_2": "3047a4ea7a0e1f4eefefbf69d9a04c157fdfe6127d39c0a908361a742f33e117",
    "servo_cover_3": "786205a736f4ea4b59ac8e55b972af5558daa84f9e8e158d2994e773fd063d19",
    "servo_cover_4": "8fc68957a4f9fb0091bada31022026be170cdb7b565214c37635dafa12929656",
    "wire_cover": "c612854adbd69581c6d914ec0d76e4f7ace5181710bb40e8c35cc5ef911ded42",
    "launch_pad": "34e068d1ba7aea904d809898f31ae7fd881ff3b4947ff63cf0deccee6c18f2ec",
    # LIVERY pins (2026-11-04, HARDWARE changed - user decisions: white body + the orange YUCHAN inlay, a flush 0.42 mm
    # 2-colour inlay on both sides of the avionics module; history/v0.9_pre_livery): the avionics module is now the WHITE
    # body = the plain module minus the logo pocket (plain 97520.82 -> 96839.82 mm^3 report volume, slicer 124.75 ->
    # 123.62 g) and the new printed part avionics_logo is the inlay (680.99 mm^3, 20 solids, slicer 1.29 g); white +
    # inlay = the plain module (C36, test_livery.py).  The plain module (with_livery=False) is still the v0.8.1 module, so
    # the V08_CHANGED_PRINTED rebuild below is taken without the pocket as well.  REVIEWFIX (2026-11-04, independent review
    # F1 / F3): the SOLID pins are unchanged (same geometry); only the slicer grams changed - inlay 1.29 -> 0.90 g (one
    # orange pass, detect_thin_wall), white body 123.62 -> 123.92 g (rocket) / 57.48 -> 57.74 g (VTOL) on one slicer basis.
    "avionics_module": "5eb5c45389e38c2cc868b30d532f782859cd814baea03d7280181ac632e69aaa",
    "avionics_logo": "3a60c0d01f81ff26cfff6446ff0cab24aaeda8be077ba70ef1d322b33fd4050d"}}


def test_printed_parts_are_the_v062_parts(built):
    """v0.6.3 changed no printed part (the user printed the v0.6.2 exports); v0.8 changes exactly payload_module and
    avionics_module by the GPS features and adds gps_pod_lid; v0.9 (BUDGET) changes exactly the parts of
    V09_CHANGED_PRINTED, removes the horns and adds the couplers / covers.  Every other printed part's SOLID entry
    (volume, solid mass, bbox, COM, inertia) is the v0.6.2 entry bit for bit (with the 'solid' mass model the report entry
    itself), those two v0.8 modules rebuilt without the GPS features are their v0.6.2 entries, and the v0.9 parts are their
    v0.9 pins (V09_SOLID_SHA256)."""
    _, L, parts, report = built
    preset = L.p["meta"]["preset"]
    pins = V062_PRINTED_SHA256[preset]
    v09 = V09_SOLID_SHA256.get(preset, {})
    printed = {n for n, q in parts.items() if q.kind == "printed"}
    assert L.has_gps_pod and printed == ((set(pins) - set(V09_REMOVED_PRINTED.get(preset, ()))) | set(V08_NEW_PRINTED)
                                         | set(v09))
    assert ("avionics_logo" in printed) == bool(getattr(L, "has_livery", False))      # livery 2026-11-04
    solid = report["extra"]["mass_model"]["mode"] == "solid"
    same = sorted(set(pins) - set(V08_CHANGED_PRINTED) - set(V09_CHANGED_PRINTED.get(preset, ()))
                  - set(V09_REMOVED_PRINTED.get(preset, ())))
    diff = [n for n in same if _entry_sha(_printed_entry(parts[n].base, parts[n].shape, L) if not solid else report["parts"][n])
            != pins[n]]
    assert not diff, [(n, report["parts"][n]["volume_mm3"]) for n in diff]
    for name in V08_CHANGED_PRINTED:
        kw = {"with_livery": False} if name == "avionics_module" else {}      # livery: without the logo pocket too
        base = rg.PRINTED_BUILDERS[name](L, with_gps=False, **kw)
        assert _entry_sha(_printed_entry(name, base, L)) == pins[name], f"{name} without the GPS features differs from v0.6.2"
        assert _entry_sha(report["parts"][name]) != pins[name], f"{name}: the GPS features are missing"
    for name, sha in v09.items():
        assert _entry_sha(_printed_entry(parts[name].base, parts[name].shape, L)) == sha, f"{name} differs from its v0.9 pin"
    for name in V09_CHANGED_PRINTED.get(preset, ()):
        assert v09[name] != pins[name], name


def test_nav_and_fcu_placeholders(built):
    """``nav`` is the v0.6.2 ``fc`` box (tray +x face, ESC top + 10, electronics.nav dims / mass) and the tray's board
    holes are still on its centre; ``fcu`` is the Layout box flat on the intake bulkhead (foam_t above its top face),
    centred, params dims / mass, its own colour, avionics zone, aboard after ejection; every Layout limit holds, all four
    vent holes are reachable with two diagonal tie runs, and the centre cable hole is under the board."""
    _, L, parts, report = built
    el = L.p["electronics"]
    assert "fc" not in parts and "fc" not in report["parts"] and "fc" not in rg.COLORS
    nav, (nl, nw, nh) = parts["nav"], el["nav"]["dims_lwh"]
    assert nav.kind == "placeholder" and nav.module == "avionics_module" and nav.color == rg.COLORS["nav"]
    bb = nav.shape.BoundingBox()
    x0, z0 = L.tray_x + L.tray_t / 2, L.esc_z[1] + 10.0
    assert (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax) == pytest.approx(
        (x0, x0 + nh, -nw / 2, nw / 2, z0, z0 + nl), abs=1e-9)
    assert report["parts"]["nav"]["mass_g"] == pytest.approx(el["nav"]["mass_g"], abs=1e-9)
    tray, h, e = parts["avionics_tray"].shape, L.fc_hole_pattern / 2, 0.3
    assert L.nav_center_z == pytest.approx(0.5 * (bb.zmin + bb.zmax))
    for sy in (1.0, -1.0):
        for sz in (1.0, -1.0):
            hole = rg._box(L.tray_x - L.tray_t / 2, L.tray_x + L.tray_t / 2, sy * h - e, sy * h + e,
                           L.nav_center_z + sz * h - e, L.nav_center_z + sz * h + e)
            assert rg.intersection_volume(hole, tray) < 1e-6, (sy, sz)           # the hole is there
    fcu, fe = parts["fcu"], el["fcu"]
    assert fcu.kind == "placeholder" and fcu.module == "avionics_module" and fcu.color == rg.COLORS["fcu"]
    assert fcu.color not in [v for k, v in rg.COLORS.items() if k != "fcu"]
    assert report["parts"]["fcu"]["mass_g"] == pytest.approx(fe["mass_g"], abs=1e-9)
    assert report["parts"]["fcu"]["com_mm"] == pytest.approx(list(L.fcu_center), abs=1e-6)
    bb = fcu.shape.BoundingBox()
    assert (bb.xlen, bb.ylen, bb.zlen) == pytest.approx(tuple(fe["dims_lwh"]), abs=1e-9)
    assert (0.5 * (bb.xmin + bb.xmax), 0.5 * (bb.ymin + bb.ymax)) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert bb.zmin == pytest.approx(L.z_intake_bulkhead_top + fe["foam_t"], abs=1e-9)
    intake = parts["intake_module"].shape
    assert rg.intersection_volume(fcu.shape, intake) < 1e-6
    assert fcu.shape.distance(intake) == pytest.approx(fe["foam_t"], abs=1e-6)       # the bulkhead right under it
    assert "fcu" not in rg.POST_EJECTION_EXCLUDED
    for nm, v in L.fcu_clearance.items():
        lim = L.fcu_limits[nm]
        assert lim is None or v >= lim - 1e-9, (nm, v)
    assert sorted(L.fcu_vent_reachable) == sorted(L.vent_hole_thetas)
    assert sorted(L.fcu_tie_pairs) == [(45.0, 225.0), (135.0, 315.0)]                   # the diagonals cross the board
    assert L.fcu_cable_hole == "covered" and L.fcu_seat_area_exact
    assert L.fcu_seat_area_mm2 == pytest.approx(bb.xlen * bb.ylen - math.pi * L.cable_hole_d ** 2 / 4)


def test_c25_fails_for_a_misplaced_fcu(default_build):
    """C25 is positive evidence on the solids: PASS on the real model; FAIL for the board 1 mm higher (1 mm short of the
    tray clearance, foam gap 4), 1 mm lower (foam gap 2), 24 mm toward the motor-lead hole (the rising bundle within
    0.5 mm) and for a 64 mm board (every vent hole covered: no cable-tie run); N/A without the placeholder."""
    L, parts, report = default_build
    ok = cg.Checker(L, parts, report).c25_fcu_board()
    assert ok.status == cg.PASS, ok.measured
    assert ok.values["foam_gap"] == pytest.approx(L.fcu_foam_t, abs=1e-6)
    assert ok.values["seat_fill"] == pytest.approx(L.fcu_seat_fraction, abs=1e-3)
    assert ok.values["clearance"]["avionics_tray"] == pytest.approx(L.fcu_min_clear, abs=1e-6)
    assert sorted(ok.values["vent_reachable"]) == sorted(L.vent_hole_thetas) and ok.values["cable_hole_covered"]
    assert sorted(ok.values["tie_pairs"]) == sorted(L.fcu_tie_pairs)

    def check(shape) -> cg.Check:
        p2 = dict(parts)
        p2["fcu"] = dataclasses.replace(parts["fcu"], shape=shape)
        return cg.Checker(L, p2, report).c25_fcu_board()

    box = parts["fcu"].shape
    up = check(box.translate(rg.V(0.0, 0.0, 1.0)))
    assert up.status == cg.FAIL and up.values["clearance"]["avionics_tray"] == pytest.approx(L.fcu_min_clear - 1.0, abs=1e-6)
    down = check(box.translate(rg.V(0.0, 0.0, -1.0)))
    assert down.status == cg.FAIL and down.values["foam_gap"] == pytest.approx(L.fcu_foam_t - 1.0, abs=1e-6)
    side = check(box.translate(rg.V(24.0, 0.0, 0.0)))
    assert side.status == cg.FAIL and side.values["clearance"]["motor-lead bundle (C20 leg E)"] < L.fcu_min_clear
    big = check(rg._box(-32.0, 32.0, -32.0, 32.0, *L.fcu_z))
    assert big.status == cg.FAIL and big.values["vent_reachable"] == [] and big.values["tie_pairs"] == []
    p2 = {n: q for n, q in parts.items() if n != "fcu"}
    p_no = load_params(PARAMS_PATH, None)
    del p_no["electronics"]["fcu"]
    assert cg.Checker(Layout(p_no), p2, report).c25_fcu_board().status == cg.NA


@pytest.mark.parametrize("change, message", [
    ({"foam_t": 4.0}, "from avionics_tray"),                    # board top 1 mm closer to the tray (1 < 2)
    ({"dims_lwh": [64.0, 64.0, 15.0]}, "no cable-tie run"),     # covers every vent hole
    ({"dims_lwh": [50.0, 50.0, 0.0]}, "dims_lwh must be positive"),
    ({"foam_t": -1.0}, "foam_t >= 0"),
])
def test_layout_rejects_a_bad_fcu(change, message):
    p = load_params(PARAMS_PATH, None)
    p["electronics"]["fcu"].update(change)
    with pytest.raises(ValueError, match=message):
        Layout(p)



# --------------------------------------------------------------------------- #
# v0.8, both presets: GPS pod on the payload tube (payload_module.gps_pod, avionics_module.gps_entry, check C26)
# --------------------------------------------------------------------------- #
def _entry_hole_mm3(L) -> float:
    """Analytic tube-wall material the GPS entry hole removes: the Ø d radial cylinder through the wall r body_id/2 ..
    body_r, int_{-a}^{a} 2 sqrt(a^2 - y^2) (sqrt(R^2 - y^2) - sqrt(r^2 - y^2)) dy (Simpson in y = a sin(phi))."""
    a, R, r = L.gps_entry_d / 2, L.body_r, L.body_id / 2
    n, s = 2000, 0.0
    h = math.pi / n
    for k in range(n + 1):
        ph = -math.pi / 2 + k * h
        y = a * math.sin(ph)
        f = 2.0 * (a * math.cos(ph)) ** 2 * (math.sqrt(R * R - y * y) - math.sqrt(r * r - y * y))
        s += (1 if k in (0, n) else (4 if k % 2 else 2)) * f
    return s * h / 3.0


def _lid_mm3(L) -> float:
    """Analytic volume of gps_pod_lid: the plate (u from the arc r = gps_lid_r_in to gps_u_out, |v| <= gps_w_h, minus the
    two c x c corner triangles) x lid t, plus the posts below it, minus the post holes and the two screw holes."""
    R, w, c = L.gps_lid_r_in, L.gps_w_h, L.gps_corner_c

    def prim(v: float) -> float:          # integral of sqrt(R^2 - v^2) dv
        return 0.5 * (v * math.sqrt(R * R - v * v) + R * R * math.asin(v / R))
    area = L.gps_u_out * 2 * w - (prim(w) - prim(-w)) - c * c
    posts = len(L.gps_post_uv) * math.pi / 4 * L.gps_post_d ** 2 * (L.gps_post_z[1] - L.gps_post_z[0])
    holes = len(L.gps_post_uv) * math.pi / 4 * L.gps_post_hole_d ** 2 * (L.gps_post_hole_z1 - L.gps_post_z[0])
    screws = len(L.gps_boss_hole_uv) * math.pi / 4 * L.gps_lid_screw_d ** 2 * L.gps_lid_t
    return area * L.gps_lid_t + posts - holes - screws


def _pod_frame(shape, L):
    """``shape`` turned by -gps_theta: the pod frame's u along +X, v along +Y."""
    return shape.rotate(rg.V(0, 0, 0), rg.V(0, 0, 1), -L.gps_theta)


def _z_segments(shape, x: float, y: float, z0: float, z1: float, e: float = 0.05) -> list[tuple[float, float]]:
    """(z_min, z_max) of every solid of ``shape`` inside a thin vertical probe at (x, y), sorted."""
    common = shape.intersect(rg._box(x - e, x + e, y - e, y + e, z0, z1))
    return sorted((s.BoundingBox().zmin, s.BoundingBox().zmax) for s in common.Solids())


def test_gps_pod_on_the_solids(built):
    """The pod, its lid, the placeholders, the raceway and the entry hole on the solids: the pod only adds material to
    the v0.6.2 payload module, outside the bore, from the payload module's bottom edge (raceway) up to the rim; the outer
    wall, the side wall and the sloped floor have their params thickness and the floor's slope; the cavity over the plate is
    empty except for the corner bosses; the lid is its analytic volume, from the plate top to the lid top, clear of the pod
    and the nose; the placeholders are the Layout boxes with params masses; the entry hole removes exactly its analytic
    volume; the antenna and lead entries follow Layout."""
    preset, L, parts, report = built
    assert L.has_gps_pod
    pay, lid, av = parts["payload_module"].shape, parts["gps_pod_lid"].shape, parts["avionics_module"].shape
    base = rg.payload_module(L, with_gps=False)
    assert _volume_or_0(base.cut(pay)) < 1e-6                                  # the pod removes nothing
    added = pay.cut(base)
    bb = added.BoundingBox()
    assert bb.zmin == pytest.approx(L.z_payload_bottom, abs=1e-6) and bb.zmax == pytest.approx(L.gps_z_rim, abs=1e-6)
    assert max(math.hypot(v.X, v.Y) for v in added.Vertices()) == pytest.approx(L.gps_r_corner, abs=1e-6)
    assert min(math.hypot(v.X, v.Y) for v in added.Vertices()) >= L.body_id / 2 - 1e-6        # nothing in the bore
    assert _probe_free(pay, rg._cyl_z(L.body_id / 2 - 0.05, *L.gps_cut_z))                   # the piston passes
    pf = _pod_frame(pay, L)
    zw = L.gps_z_rim - 3.0
    outer = pf.intersect(rg._box(L.gps_u_cav_out - 5.0, L.gps_u_out + 2.0, -0.05, 0.05, zw, zw + 0.1)).BoundingBox()
    assert (outer.xmin, outer.xmax) == pytest.approx((L.gps_u_cav_out, L.gps_u_out), abs=1e-6)          # outer wall
    side = pf.intersect(rg._box(L.gps_u_c - 0.05, L.gps_u_c + 0.05, 0.0, L.gps_w_h + 2.0, zw, zw + 0.1)).BoundingBox()
    assert (side.ymin, side.ymax) == pytest.approx((L.gps_cav_hw, L.gps_w_h), abs=1e-6)                 # side wall
    for u in (L.gps_u_c - 15.0, L.gps_u_c, L.gps_u_c + 15.0):                                           # sloped floor
        segs = _z_segments(pf, u, 0.0, L.gps_z_foot - 5.0, L.gps_z_rim + 1.0, e=0.001)   # 1:1 slope: +-e in z
        assert segs == _approx_segs([(L.gps_floor_under_z(u), L.gps_floor_top_z(u))], abs=2e-3), (u, segs)
    assert L.gps_floor_top_z(L.gps_u_c + 10.0) - L.gps_floor_top_z(L.gps_u_c) == pytest.approx(
        10.0 * math.tan(math.radians(L.gps_floor_slope_deg)))
    z_lo, z_hi = L.gps_z_gp_bottom - L.gps_cavity_clear + 0.05, L.gps_z_rim - 0.05
    assert _probe_free(pf, rg._box(L.body_r + 0.05, L.gps_boss_u[0] - 0.05, -L.gps_cav_hw + 0.05, L.gps_cav_hw - 0.05, z_lo, z_hi))
    assert _probe_free(pf, rg._box(L.gps_boss_u[0], L.gps_u_cav_out - 0.05, -L.gps_boss_v[0] + 0.05, L.gps_boss_v[0] - 0.05,
                                   z_lo, z_hi))
    for v0, v1 in ((L.gps_boss_v[0], L.gps_boss_v[1]), (-L.gps_boss_v[1], -L.gps_boss_v[0])):        # bosses solid
        blk = rg._box(L.gps_boss_u[0] + 0.05, L.gps_boss_u[1] - 0.05, v0 + 0.05, v1 - 0.05, L.gps_z_rim - 0.5, L.gps_z_rim - 0.05)
        for hu, hv in L.gps_boss_hole_uv:
            blk = blk.cut(rg._cyl_z(L.gps_boss_hole_d / 2 + 0.05, L.gps_z_rim - 1.0, L.gps_z_rim, hu, hv))
        for sgn in (1.0, -1.0):                                       # the corner chamfer takes the outer corner
            blk = blk.cut(rg._poly_pod(rg.gps_corner_pts(L, sgn), L.gps_z_rim - 1.0, L.gps_z_rim, 0.0))
        assert rg.intersection_volume(blk, pf) == pytest.approx(blk.Volume(), rel=1e-6)
    # lid
    lb = lid.BoundingBox()
    assert (lb.zmin, lb.zmax) == pytest.approx((L.gps_z_gp_top, L.gps_z_lid_top), abs=1e-6)
    assert L.gps_z_lid_top == pytest.approx(L.z_payload_spigot_bottom - L.p["payload_module"]["gps_pod"]["lid"]["gap_below_nose_skirt"])
    assert _precise_volume(lid) == pytest.approx(_lid_mm3(L), rel=1e-6)
    assert rg.intersection_volume(lid, pay) < 1e-6 and rg.intersection_volume(lid, parts["nose_cone"].shape) < 1e-6
    assert parts["gps_pod_lid"].orientation == "flat" and parts["gps_pod_lid"].module == "payload_module"
    # placeholders, antenna, lead
    (pu, pv, pz), (bu, bv, bz) = L.gps_patch_box, L.gps_board_box
    g = parts["gps"]
    assert g.kind == "placeholder" and report["parts"]["gps"]["mass_g"] == pytest.approx(
        L.p["electronics"]["gps"]["board_mass_g"], abs=1e-9)
    assert g.shape.Volume() == pytest.approx((pu[1] - pu[0]) * (pv[1] - pv[0]) * (pz[1] - pz[0])
                                             + (bu[1] - bu[0]) * (bv[1] - bv[0]) * (bz[1] - bz[0]), rel=1e-9)
    gpb = _pod_frame(g.shape, L).BoundingBox()
    assert (gpb.zmin, gpb.zmax) == pytest.approx((bz[0], pz[1]), abs=1e-6)
    s_gp, n_gp = L.gps_gp_side, L.gps_gp_notch
    assert parts["gps_ground_plane"].shape.Volume() == pytest.approx((s_gp * s_gp - 2 * n_gp * n_gp) * L.gps_gp_t, rel=1e-9)
    assert report["parts"]["gps_ground_plane"]["mass_g"] == pytest.approx(L.gps_gp_mass_g, abs=1e-9)
    t = math.radians(L.gps_theta)
    u_a, _, z_a = L.gps_antenna_uvz
    assert L.gps_antenna_xyz == pytest.approx((u_a * math.cos(t), u_a * math.sin(t), z_a), abs=1e-9)
    assert z_a == pytest.approx(pz[1] - L.p["payload_module"]["gps_pod"]["stack"]["phase_centre_below_patch_top_ASSUMED"])
    le = report["parts"]["gps_lead"]
    assert le["kind"] == "lumped" and le["mass_g"] == pytest.approx(L.gps_lead_mass_g) and le["com_mm"] == pytest.approx(
        list(L.gps_lead_com))
    for n in ("gps", "gps_ground_plane", "gps_lead", "gps_pod_lid"):
        assert n not in rg.POST_EJECTION_EXCLUDED                                            # stays aboard
    # entry hole: the avionics module loses exactly the analytic hole, nothing else changes (v0.8.1: judged on the module
    # without the GNS bosses, which test_gns_board_on_the_solids accounts for)
    av0 = rg.avionics_module(L, with_gps=False)
    av = rg.avionics_module(L, with_gns=False)
    assert _volume_or_0(av.cut(av0)) < 1e-6
    assert _volume_or_0(av0.cut(av)) == pytest.approx(_entry_hole_mm3(L), rel=1e-4)
    ex = report["extra"]["gps"]
    assert ex["added_total_g"] == pytest.approx(sum(ex["added_mass_g"].values()))
    assert ex["antenna_lever_arm_from_cg_mm"] == pytest.approx(
        [a_ - c_ for a_, c_ in zip(L.gps_antenna_xyz, report["totals"]["com_mm"])])


def test_gps_lead_route_and_mass(built):
    """The lead route starts at the board in the pod, runs outside the tube down to the entry hole (no separation
    interface: below the nose edge, never in the payload bore), lies inside the avionics bore from the hole on and ends
    on the GNS board's top (v0.8.1; the NAV top without the GNS); every bay clearance Layout holds is >= 0; mass and COM
    are the params formula."""
    _, L, _, _ = built
    pts = L.gps_lead_polyline
    k = L.gps_lead_bay_start
    assert all(math.hypot(q[0], q[1]) > L.body_r for q in pts[:k])
    assert all(math.hypot(q[0], q[1]) < L.body_id / 2 for q in pts[k:])
    assert max(q[2] for q in pts) < L.z_payload_spigot_bottom
    if L.has_gns:
        assert pts[-1][2] == pytest.approx(L.gns_z[1] + L.gps_bundle_d / 2)
        assert L.gns_x[0] < pts[-1][0] < L.gns_x[1] and L.gns_y[0] < pts[-1][1] < L.gns_y[1]
        assert len(pts) - k == 2                                     # entry hole -> GNS header, one straight leg
    else:
        assert pts[-1][2] == pytest.approx(L.nav_z[1] + L.gps_bundle_d / 2) and L.nav_x[0] < pts[-1][0] < L.nav_x[1]
    seg = sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))
    assert L.gps_lead_route_len == pytest.approx(seg)
    ld = L.p["payload_module"]["gps_pod"]["lead"]
    assert L.gps_lead_len_mm == pytest.approx(seg + ld["slack_mm"])
    assert L.gps_lead_mass_g == pytest.approx(L.gps_lead_len_mm * 1e-3 * ld["mass_g_per_m_ASSUMED"]
                                              + ld["connector_pair_g_ASSUMED"] + ld["hardware_g_ASSUMED"])
    assert all(v >= -1e-9 for k_, v in L.gps_clearance.items() if k_.startswith("lead (bay)"))


def test_c26_fails_for_a_bad_pod(default_build):
    """C26 is positive evidence: it FAILs for the payload module without the pod, the lid 0.5 mm high, the patch 1 mm
    high (air gap 2), the ground plane 1 mm toward the tube (touches the tube wall), a plug in the raceway, undrilled
    lid-screw bosses and the avionics module without the entry hole; PASS on the real model and N/A without the pod."""
    L, parts, report = default_build
    ok = cg.Checker(L, parts, report).c26_gps_pod()
    assert ok.status == cg.PASS, ok.measured
    assert ok.values["measured"]["patch -> lid (BRep)"] == pytest.approx(L.gps_patch_air_gap, abs=1e-6)

    def status(**shapes) -> str:
        p2 = dict(parts)
        for name, shp in shapes.items():
            p2[name] = dataclasses.replace(parts[name], shape=shp)
        return cg.Checker(L, p2, report).c26_gps_pod().status

    pay = parts["payload_module"].shape
    t = L.gps_theta
    assert status(payload_module=rg.payload_module(L, with_gps=False)) == cg.FAIL               # no pod
    assert status(gps_pod_lid=parts["gps_pod_lid"].shape.translate(rg.V(0.0, 0.0, 0.5))) == cg.FAIL   # lid high
    assert status(gps=parts["gps"].shape.translate(rg.V(0.0, 0.0, 1.0))) == cg.FAIL                 # air gap 2
    toward = rg.V(-math.cos(math.radians(t)), -math.sin(math.radians(t)), 0.0)
    assert status(gps_ground_plane=parts["gps_ground_plane"].shape.translate(toward)) == cg.FAIL     # touches the tube
    zm = 0.5 * (L.gps_pk_solid_z[1] + L.gps_rw_z[1])
    plug = rg._box_polar(*L.gps_rw_void_u, -L.gps_rw_void_hv, L.gps_rw_void_hv, zm - 5.0, zm + 5.0, t)
    assert status(payload_module=rg._fuse([pay, plug])) == cg.FAIL                                 # raceway blocked
    plugs = []
    for u, v in L.gps_boss_hole_uv:
        x, y, _ = L.gps_xyz(u, v, 0.0)
        plugs.append(rg._cyl_z(L.gps_boss_hole_d / 2, L.gps_boss_hole_z0, L.gps_z_rim, x, y))
    assert status(payload_module=rg._fuse([pay] + plugs)) == cg.FAIL                               # bosses not drilled
    assert status(avionics_module=rg.avionics_module(L, with_gps=False)) == cg.FAIL                # no entry hole
    p_no = load_params(PARAMS_PATH, None)
    del p_no["payload_module"]["gps_pod"]
    p_no["electronics"].pop("gns", None)                  # v0.8.1: the GNS board needs the pod
    p2 = {n: q for n, q in parts.items() if not n.startswith(("gps", "gns"))}
    assert cg.Checker(Layout(p_no), p2, report).c26_gps_pod().status == cg.NA


@pytest.mark.parametrize("path, value, message", [
    (("payload_module", "gps_pod", "floor_slope_deg"), 40.0, "must rise at >= 45"),
    (("payload_module", "gps_pod", "ground_plane", "side"), 20.0, "smaller than the patch"),
    (("payload_module", "gps_pod", "ground_plane", "corner_notch"), 3.0, "notch clears the corner boss"),
    (("payload_module", "gps_pod", "raceway", "w"), 2.0, "smaller than the lead bundle"),
    (("payload_module", "gps_pod", "stack", "board_standoff_h"), 12.0, "board bottom only"),
    (("payload_module", "gps_pod", "lid", "post_inset"), 9.0, "patch keep-out"),
    (("payload_module", "gps_pod", "wall_t"), 1.0, "min_feature"),
    (("payload_module", "gps_pod", "lead", "bay_cross_y"), 0.0, "lead route (in the avionics bay )?hits"),   # v0.8.1: glink
    (("avionics_module", "gps_entry", "theta_deg"), 190.0, "must equal"),
])
def test_layout_rejects_a_bad_gps_pod(path, value, message):
    p = load_params(PARAMS_PATH, None)
    d = p
    for key in path[:-1]:
        d = d[key]
    d[path[-1]] = value
    with pytest.raises(ValueError, match=message):
        Layout(p)


# --------------------------------------------------------------------------- #
# v0.8.1, both presets: GNS board (electronics.gns, avionics_module.gns_mount, check C27)
# --------------------------------------------------------------------------- #
def _gns_boss_mm3(L) -> float:
    """Analytic material one GNS boss adds inside the bore: int over v of the diamond's height 2 (hd - |v - v_h|) times
    its length from the tip u_back to the bore u_wall(v) (Simpson, fine), for the boss at +v_h (all four are mirror images
    in v / translates in z)."""
    hd, vh, n = L.gns_boss_hd, L.gns_pattern / 2, 4000
    h, acc = 2 * hd / n, 0.0
    for k in range(n + 1):
        v = vh - hd + k * h
        f = 2 * (hd - abs(v - vh)) * (L.gns_u_wall(v) - L.gns_u_back)
        acc += f * (1 if k in (0, n) else (4 if k % 2 else 2))
    return acc * h / 3.0


def test_gns_board_on_the_solids(built):
    """The GNS board and its bosses on the solids: the avionics module is the v0.8 module plus exactly the four boss
    prisms clipped at the OD minus the four tap holes (symmetric difference ~0), each boss adds its analytic volume, its
    cross-section half way along is the diamond (45 deg faces) minus the hole, and the holes stop hole_depth past the tip;
    the placeholder is the Layout box on the boss tips (touching, not overlapping), params mass + screws, its own colour,
    avionics zone, aboard after ejection; every Layout limit holds; the report's extra.gns adds up."""
    _, L, parts, report = built
    assert L.has_gns
    av, gns = parts["avionics_module"].shape, parts["gns"].shape
    av08 = rg.avionics_module(L, with_gns=False)
    rebuilt = rg._cut(rg._fuse([av08] + rg.gns_bosses(L)), rg.gns_hole_cutters(L))
    assert _volume_or_0(rebuilt.cut(av)) < 1e-6 and _volume_or_0(av.cut(rebuilt)) < 1e-6
    for k, b in enumerate(rg.gns_bosses(L)):
        assert _volume_or_0(b.cut(av08)) == pytest.approx(_gns_boss_mm3(L), rel=1e-4), k
    # cross-section half way along each boss: the diamond minus the hole, z / v extents +- hd
    th = L.gns_theta
    for (vh, zh) in L.gns_hole_vz:
        u_m = L.gns_u_back + 0.5 * (L.gns_u_wall(abs(vh) + L.gns_boss_hd) - L.gns_u_back)
        e_ = L.gns_boss_hd + 0.5
        sl = av.intersect(rg._box_polar(u_m, u_m + 0.01, vh - e_, vh + e_, zh - e_, zh + e_, th))
        bb = sl.BoundingBox()
        assert (bb.zmin, bb.zmax) == pytest.approx((zh - L.gns_boss_hd, zh + L.gns_boss_hd), abs=1e-6)
        area = sl.Volume() / 0.01
        assert area == pytest.approx(2 * L.gns_boss_hd ** 2 - math.pi * (L.gns_hole_d / 2) ** 2, rel=1e-4)
        assert _probe_free(av, rg._radial_cyl(L.gns_hole_d - 0.1, L.gns_u_back + 0.05, L.gns_hole_u[1] - 0.05, th, zh, vh))
        assert not _probe_free(av, rg._radial_cyl(L.gns_hole_d - 0.1, L.gns_hole_u[1] + 0.05, L.gns_hole_u[1] + 0.5, th, zh, vh))
    # the placeholder on the tips
    part, el = parts["gns"], L.p["electronics"]["gns"]
    assert part.kind == "placeholder" and part.module == "avionics_module" and part.color == rg.COLORS["gns"]
    assert part.color not in [c for k, c in rg.COLORS.items() if k != "gns"]
    bb = gns.BoundingBox()
    assert (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax) == pytest.approx(
        (*L.gns_x, *L.gns_y, *L.gns_z), abs=1e-9)
    assert (bb.xlen, bb.ylen, bb.zlen) == pytest.approx((el["dims_lwh"][2], el["dims_lwh"][1], el["dims_lwh"][0]), abs=1e-9)
    assert rg.intersection_volume(gns, av) < 1e-6 and gns.distance(av) == pytest.approx(0.0, abs=1e-6)
    gm = L.p["avionics_module"]["gns_mount"]
    assert report["parts"]["gns"]["mass_g"] == pytest.approx(el["mass_g_ASSUMED"] + gm["screws_g_ASSUMED"], abs=1e-9)
    assert report["parts"]["gns"]["com_mm"] == pytest.approx(list(L.gns_center), abs=1e-6)
    assert "gns" not in rg.POST_EJECTION_EXCLUDED
    assert L.gns_z[1] == pytest.approx(L.gps_entry_z - L.gps_entry_d / 2 - gm["top_below_entry_hole"])
    for k, v in L.gns_clearance.items():
        assert v >= -1e-9, (k, v)
    # glink lead: GNS top -> NAV top, inside the bore, the params mass formula
    pts, bd = L.glink_lead_polyline, L.gps_bundle_d
    assert pts[0][2] == pytest.approx(L.gns_z[1] + bd / 2) and pts[-1][2] == pytest.approx(L.nav_z[1] + bd / 2)
    assert all(math.hypot(q[0], q[1]) <= L.body_id / 2 - bd / 2 - 1.0 + 1e-9 for q in pts)
    gl = gm["glink_lead"]
    assert L.glink_mass_g == pytest.approx((L.glink_route_len + gl["slack_mm"]) * 1e-3 * gl["mass_g_per_m_ASSUMED"]
                                           + gl["connector_pair_g_ASSUMED"])
    ex = report["extra"]["gns"]
    assert ex["added_total_g"] == pytest.approx(sum(ex["added_mass_g"].values()))
    assert ex["added_mass_g"]["bosses + tap holes (in avionics_module)"] == pytest.approx(
        (report["parts"]["avionics_module"]["volume_mm3"] - av08.Volume()) * L.p["printer"]["density_g_cm3"] / 1000.0)
    assert ex["thrust_to_weight_nominal_cad"] == pytest.approx(report["extra"]["thrust_to_weight_nominal"])
    assert ex["pod_mitigation"]["saved_g"] > 0.0


def test_c27_fails_for_a_bad_gns(default_build):
    """C27 is positive evidence: PASS on the real model; FAIL for the board 1 mm toward the axis (off the boss tips), 0.5 mm
    toward the wall (into the bosses), undrilled bosses, the module without the bosses, and the GNS board 6 mm closer
    to the battery than the clearance allows; N/A without electronics.gns."""
    L, parts, report = default_build
    ok = cg.Checker(L, parts, report).c27_gns_board()
    assert ok.status == cg.PASS, ok.measured
    assert ok.values["measured"]["boss tip area / design"] == pytest.approx(1.0, abs=1e-3)

    def status(**shapes) -> str:
        p2 = dict(parts)
        for name, shp in shapes.items():
            p2[name] = dataclasses.replace(parts[name], shape=shp)
        return cg.Checker(L, p2, report).c27_gns_board().status

    c, sn = L.gns_cs
    inward = rg.V(-c, -sn, 0.0)                     # toward the vehicle axis (u decreasing)
    box = parts["gns"].shape
    assert status(gns=box.translate(inward)) == cg.FAIL                                   # off the tips
    assert status(gns=box.translate(inward * -0.5)) == cg.FAIL                            # into the bosses
    av = parts["avionics_module"].shape
    plugs = [rg._radial_cyl(L.gns_hole_d, L.gns_hole_u[0], L.gns_hole_u[1], L.gns_theta, z, v) for v, z in L.gns_hole_vz]
    assert status(avionics_module=rg._fuse([av] + plugs)) == cg.FAIL                      # bosses not drilled
    assert status(avionics_module=rg.avionics_module(L, with_gns=False)) == cg.FAIL       # no bosses
    if c < 0:      # -x wall: the battery is +x of it; 1 mm short of the clearance (the board is off the tips then, too)
        shift = L.battery_x[0] - L.gns_x[1] - (L.gns_min_clear - 1.0)
        p2 = dict(parts)
        p2["gns"] = dataclasses.replace(parts["gns"], shape=box.translate(rg.V(shift, 0.0, 0.0)))
        res = cg.Checker(L, p2, report).c27_gns_board()
        assert res.status == cg.FAIL and res.values["clearance"]["battery"] == pytest.approx(L.gns_min_clear - 1.0, abs=1e-6)
    p_no = load_params(PARAMS_PATH, None)
    del p_no["electronics"]["gns"]
    p2 = {n: q for n, q in parts.items() if n != "gns"}
    assert cg.Checker(Layout(p_no), p2, report).c27_gns_board().status == cg.NA


@pytest.mark.parametrize("path, value, message", [
    (("avionics_module", "gns_mount", "theta_deg"), 170.0, "multiple of 90"),
    (("avionics_module", "gns_mount", "u_back"), 54.0, "bore"),
    (("avionics_module", "gns_mount", "u_back"), 52.0, "PCB back only"),
    (("avionics_module", "gns_mount", "boss_half_diag"), 2.5, "boss walls below"),
    (("avionics_module", "gns_mount", "hole_depth"), 5.0, "boss walls below"),
    (("avionics_module", "gns_mount", "pattern"), 50.0, "off the"),
    (("payload_module", "gps_pod", "lead", "bay_route_z_above_tray_top"), 0.0, "glink lead route hits"),
])
def test_layout_rejects_a_bad_gns(path, value, message):
    p = load_params(PARAMS_PATH, None)
    d = p
    for key in path[:-1]:
        d = d[key]
    d[path[-1]] = value
    with pytest.raises(ValueError, match=message):
        Layout(p)


def test_gns_needs_the_gps_pod():
    p = load_params(PARAMS_PATH, None)
    del p["payload_module"]["gps_pod"]
    with pytest.raises(ValueError, match="needs the GPS pod"):
        Layout(p)
    del p["electronics"]["gns"]
    assert not Layout(p).has_gns


# --------------------------------------------------------------------------- #
# wire09c clock mount (user 2026-10-30: 'make the motor-lead passage generous, I do not want to measure') - the keyways
# round the circle, the lead passage + blister, the generosity-box corner grid, the floor pad, the axial retention
# --------------------------------------------------------------------------- #
import check_clock as ckc  # noqa: E402


@pytest.fixture(scope="session")
def clock_model():
    L = rg.layout_for(None, "vtol")
    if not getattr(L, "clock_mount", False):
        pytest.skip("the default params do not use the clock mount")
    prop = rg.propulsion_module(L)
    return L, prop, ckc.local_module(L, prop)


def _cov(angles, theta_p, e_cov, step=0.05):
    worst, gaps = 0.0, 0
    for i in range(int(round(360.0 / step))):
        phi = -180.0 + i * step
        best = min(abs(((a + phi - theta_p) + 180.0) % 360.0 - 180.0) for a in angles)
        worst = max(worst, best)
        gaps += best > e_cov + 1e-9
    return worst, gaps


def test_clock_keyways_cover_every_azimuth_and_a_missing_keyway_leaves_a_gap(clock_model):
    """Coverage from the keyway list (independent of Layout's loop): 100 %, worst residual <= E_cov; NEGATIVE CONTROL:
    the keyway list with one keyway (and its seats) removed leaves a gap."""
    L, _, _ = clock_model
    rc = ckc.coverage_recheck(L, step=0.05)
    assert rc["coverage"] == 1.0 and not rc["gaps"] and rc["worst_residual_deg"] <= L.lp_E_cov
    assert len(L.ck_keyways) >= 20 and len(L.ck_hover) >= 1 and L.ck_min_tooth >= 1.2 - 1e-9
    angs = [c["a"] for c in L.ck_configs] + [c["a"] + 180.0 for c in L.ck_configs]
    assert _cov(angs, L.wire_theta, L.lp_E_cov)[1] == 0
    drop = L.ck_keyways[len(L.ck_keyways) // 3]
    cf = [c for c in L.ck_configs if abs(((c["a"] - drop) + 180) % 360 - 180) > 0.01
          and abs(((c["b"] - drop) + 180) % 360 - 180) > 0.01]
    angs2 = [c["a"] for c in cf] + [c["a"] + 180.0 for c in cf]
    assert _cov(angs2, L.wire_theta, L.lp_E_cov)[1] > 0


def test_clock_floor_pad_judge_fix_and_negative_control():
    """The floor pad at pad_t 3.0 passes the one-tab touchdown bending rule (factor >= 5); NEGATIVE CONTROL: the
    concept's 1.2 mm pad makes C18 FAIL (factor < 3)."""
    L = rg.layout_for(None, "vtol")
    assert ckc.pad_hand_calc(L)["factor_status"] >= ckc.PAD_FACTOR_PASS
    p = load_params(config="vtol")
    p["propulsion_module"]["clock_keyways"]["pad_t"] = 1.2
    L12 = Layout(p)
    assert ckc.pad_hand_calc(L12)["factor_status"] < ckc.PAD_FACTOR_WARN
    parts = rg.build_all(L12)
    st, _, _, _ = ckc.c18_clock(L12, parts["propulsion_module"].shape, parts["intake_module"].shape)
    assert st == cg.FAIL


def test_clock_corner_grid_negative_controls(clock_model):
    """The C29 grid would see a collision: the widest corner turned 6 deg past E_design and the R15 corner pushed 3 mm
    past the outer wall both intersect the module; the same corners at +-E_design and at the wall do not."""
    L, _, loc = clock_model
    E = L.lp_E_design
    cw = {"S": max(L.gb_S), "sec": "round12", "h": 6.0, "w": 6.0, "od": max(L.gb_od), "R": max(L.gb_R),
          "gap": max(L.gb_gap), "inset": 0.0, "early": 0.0}
    for e in (-E, E):
        assert ckc._v_env(rg.clock_lead_env(L, cw, e, True), loc) < ckc.CLOCK_CLEAR_TOL_MM3
        assert ckc._v_env(rg.clock_lead_dropin(L, cw, e), loc) < ckc.CLOCK_CLEAR_TOL_MM3
    assert ckc._v_env(rg.clock_lead_env(L, cw, E + 6.0, True), loc) > 1.0
    assert ckc._v_env(rg.clock_lead_env(L, dict(cw, early=-3.0), 0.0, True), loc) > 1.0
    # the R10 / S28 corners are in the grid explicitly (no monotonic argument)
    cs = L.clock_corners(module_only=True)
    assert {c["R"] for c in cs} >= {7.0, 10.0, 15.0} and {c["S"] for c in cs} >= {20.0, 28.0, 35.0}


def _env_points(L, c, e, frac=0.98, n_ring=8):
    """Points inside one corner's grown envelope: on its centre line and on rings at frac x the envelope half around it
    (bundle box corners / face centres, single-lead / bend / leg / steer tubes)."""
    pth = L.clock_lead_path(c)
    th, th0 = L.wire_theta + e, L.wire_theta
    pts = []
    for k in range(9):
        u = pth["rm"] + 0.5 + (pth["r_e"] - pth["rm"] - 1.0) * k / 8
        for dv in (-1, 0, 1):
            for dz in (-1, 0, 1):
                pts.append(rg._pt_th(u, dv * frac * pth["wb"], pth["z_c"] + dz * frac * pth["hb"], th))
    cl = []
    for k in range(13):
        a = 0.5 * math.pi * k / 12
        cl.append((pth["r_s"] + pth["R"] * math.sin(a), pth["z_s"] + pth["R"] - pth["R"] * math.cos(a), th))
    for k in range(1, 9):
        cl.append((pth["r_v"], pth["z_v0"] + (pth["z_r0"] - pth["z_v0"]) * k / 8, th))
    a0, a1 = rg._pt_th(pth["r_v"], 0, pth["z_r0"], th), rg._pt_th(L.lp_u_top, 0, pth["z_r1"], th0)
    for u, z, t in cl:
        q = rg._pt_th(u, 0, z, t)
        pts.append(q)
        for j in range(n_ring):
            ph = 2 * math.pi * j / n_ring
            pts.append(rg._pt_th(u + frac * pth["rho"] * math.cos(ph), frac * pth["rho"] * math.sin(ph), z, t))
            pts.append(rg._pt_th(u, frac * pth["rho"] * math.cos(ph), z + frac * pth["rho"] * math.sin(ph), t))
    for k in range(1, 8):
        q = a0 + (a1 - a0) * (k / 8)
        pts.append(q)
    return pts


def test_clock_corner_grid_sampled_crosscheck(clock_model):
    """Second, independent method next to C29's booleans (graft from concept A): points inside every module corner's
    grown envelope at e -E / 0 / +E are classified against the solid module - none may be inside material; and points in
    the passage void at random-free stations are void.  NEGATIVE CONTROL: the same sampler finds material for the corner
    turned 6 deg past E_design."""
    L, prop, loc = clock_model
    E = L.lp_E_design
    inside = []
    n = 0
    for c in L.clock_corners(module_only=True):
        if c["early"] != 0.0 and c["S"] != max(L.gb_S):
            continue
        for e in (-E, 0.0, E):
            for q in _env_points(L, c, e):
                n += 1
                if loc.isInside(q, 1e-4):
                    inside.append((ckc.corner_tag(c, e), round(q.x, 2), round(q.y, 2), round(q.z, 2)))
    assert n > 10000 and not inside, inside[:5]
    cw = {"S": max(L.gb_S), "sec": "round12", "h": 6.0, "w": 6.0, "od": max(L.gb_od), "R": max(L.gb_R),
          "gap": max(L.gb_gap), "inset": 0.0, "early": 0.0}
    assert any(loc.isInside(q, 1e-4) for q in _env_points(L, cw, E + 6.0))
    # the passage void: stations inside the fan outline shrunk by 0.5, fan floor + 0.5 .. module top
    void = 0
    for k in range(40):
        u = L.lp_u_a + 4.0 + (L.lp_r_wall - 0.6 - L.lp_u_a - 4.0) * k / 39
        for v in (-L.lp_hw + 0.5, 0.0, L.lp_hw - 0.5):
            for z in (L.lp_z_fan_floor + 0.5, 0.5 * (L.lp_z_fan_floor + L.lp_z_blister_top), L.lp_z_blister_top - 1.0):
                q = rg._pt_th(u, v, z, L.wire_theta)
                void += prop.isInside(q, 1e-4)
    assert void == 0


def test_clock_blister_and_keel_on_the_solid(clock_model):
    """The blister stands out of the body to Layout's outer radius (STL-free: the solid's extent at theta_P), its walls
    are >= lead_passage.wall, and the keel's bottom is no lower than Layout says (the V reaches z_bb - hk)."""
    L, prop, _ = clock_model
    bw = ckc.blister_walls(L, prop)
    assert min(bw.values()) >= L.lp_wall - ckc.BLISTER_WALL_TOL, bw
    # the blister's outer wall and top (above the fin plate: from the fan floor up)
    probe = rg._box_polar(L.body_r + 1.0, L.lp_blister_r_max + 2.0, -0.5, 0.5, L.lp_z_fan_floor, L.lp_z_blister_top + 5.0,
                          L.wire_theta)
    bb = rg._rot_z(prop.intersect(probe), -L.wire_theta).BoundingBox()
    assert bb.xmax == pytest.approx(L.lp_blister_r_max, abs=0.05)       # the outer corner arc hypot(r_out, hw) at v = 0
    assert bb.zmax == pytest.approx(L.lp_z_blister_top, abs=0.05)
    # the keel off the fin plate (v 3..3.2): its bottom is the 45 deg face, hk - v below the blister bottom, at the widest step
    u0, u1, hk = max(L.lp_keel, key=lambda t: t[2])
    um = 0.5 * (u0 + u1)
    probe = rg._box_polar(um - 0.1, um + 0.1, 3.0, 3.2, L.lp_keel_z_low - 5.0, L.lp_z_blister_bottom + 0.5, L.wire_theta)
    bb = prop.intersect(probe).BoundingBox()
    z_face = L.lp_z_blister_bottom - (hk - 3.0)          # the 45 deg face at v 3.0; the 0.2 mm probe spans 0.2 mm of z
    assert z_face - 0.05 <= bb.zmin <= z_face + 0.2


def test_clock_retention_rules():
    """C35 inputs: foam compression inside its band for every stack case, engagement after the largest lift >= 30 mm, the
    shelf and keyway-bearing factors >= 5 (E / A)."""
    L = rg.layout_for(None, "vtol")
    r = ckc.retention(L, rg.intake_module(L))
    assert r["ok"], r
    assert r["engaged_after_lift_mm"] >= ckc.RETENTION_ENGAGED_MIN and r["shelf_factor"] >= 5 and r["bearing_factor"] >= 5
    # fix round (review F4): the static weight on 2 / 1 pads is reported with its spin-up travel (A)
    st = r["static_weight_A"]
    s2, s1 = st["2 pads (two-tab seat)"], st["1 pad (hover seat)"]
    assert 0.0 < s2["compression"] <= 0.5 and s2["travel_to_shelf_mm"] > r["lift_max_mm"]
    assert s1["stress_kpa"] > s2["stress_kpa"] and s1["travel_to_shelf_mm"] >= s2["travel_to_shelf_mm"]


def test_clock_sleeve_teeth_bending_and_negative_control(clock_model):
    """Fix round (review F1): every sleeve tooth beside a keyway - measured on the solid, also the two beside the lead
    passage - carries the C35 design torque on ONE tab in tangential bending (fixed-fixed, uniform) with a factor >= 5,
    and the planner keeps every tooth >= its w_req; NEGATIVE CONTROL: one extra keyway 6.6 deg beside an existing one
    (like the old 193.55 / 200.15 pair) leaves a narrow tooth that FAILs."""
    L, prop, _ = clock_model
    tb = ckc.teeth_bending(L, prop)
    assert tb["ok"] and tb["factor_min"] >= ckc.RETENTION_FACTOR_MIN, tb["worst"]
    assert tb["keyway_sides_found"] == 2 * len(L.ck_keyways)
    assert sum(1 for t in tb["teeth"] if None in t["sides"]) == 2          # the two teeth beside the passage
    assert L.ck_min_tooth >= L.ck_tooth["w_req_mm"] and tb["w_min_mm"] >= L.ck_tooth["w_req_mm"] - 0.05
    L2 = rg.layout_for(None, "vtol")
    k = min(a for a in L2.ck_keyways if a > 185.0)
    L2.ck_keyways = sorted(L2.ck_keyways + [round(k + 6.6, 3)])
    tb2 = ckc.teeth_bending(L2, rg.propulsion_module(L2))
    assert not tb2["ok"] and tb2["factor_min"] < 1.0, tb2["worst"]


def test_clock_turned_ribbon_section_in_the_box():
    """Fix round (review F5): the 12 x 5 ribbon turned to any angle (its half diagonal both ways) is a corner section of
    the C29 grid, it sets the fan floor and enters the jet blockage / drag area."""
    L = rg.layout_for(None, "vtol")
    h, w = L.gb_sections["ribbon12x5_turned"]
    assert abs(h - math.hypot(6.0, 2.5)) < 1e-9 and abs(w - h) < 1e-9
    assert any(c["sec"] == "ribbon12x5_turned" for c in L.clock_corners())
    assert L.ml_blockage["worst_corner"]["sec"] == "ribbon12x5_turned"
    assert L.lp_hw >= w + L.lp_wcl
