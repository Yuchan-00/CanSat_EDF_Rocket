"""Build the CadQuery twin, export STL / STEP and write the SPEC section 6 mass-property report.

Invariant: every number in ``cadquery_report.json`` is measured from the solids built by
``rocket_geometry`` (OCC volume properties) or copied from Layout/params - nothing is
estimated.  Outputs (default preset):

* cad/exports/cadquery/stl/<printed part>.stl          binary STL per printed part, vehicle coordinates
* cad/exports/cadquery/CanSatEDFRocket.step            STEP assembly, one part per component
* cad/exports/cadquery_report.json                     SPEC 6 schema

For a non-default ``--preset`` the files become stl_<preset>/, CanSatEDFRocket_<preset>.step and
cadquery_report_<preset>.json.  The launch pad is reported as a part but excluded from the
vehicle totals (ground equipment).  v0.6.2: ``extra.window_gable`` records the window gussets measured on the
built intake against Layout's integral, and the parts list carries the ``radio_log`` placeholder.  v0.6.3: the parts list
carries ``nav`` (was ``fc``) and the new ``fcu`` placeholder; no printed part changes, so the STL exports of a v0.6.2
tree and a v0.6.3 tree are the same solids.  v0.8 (GPS pod): payload_module carries the pod, raceway and connector pocket,
avionics_module the lead entry hole, the new printed part ``gps_pod_lid`` is exported (gps_pod_lid.stl), the parts list
carries the placeholders ``gps`` / ``gps_ground_plane`` and the lumped ``gps_lead``, and ``extra.gps`` records what the pod
adds (measured against the same modules built without it) and the antenna lever arm from the CG.  v0.8.1 (GNS board):
avionics_module carries the four GNS bosses and tap holes, the parts list carries the ``gns`` placeholder, and ``extra.gns``
records what the board adds (bosses measured against the module without them, the GPS lead change against the v0.8 route),
the GPS-pod mass mitigation re-measured against params, the board's distances and the CAD-mass T/W.
livery (user decisions 2026-11-04): with ``Layout.has_livery`` the parts list carries the orange inlay ``avionics_logo``
(exported as avionics_logo.stl, STEP colour orange) beside the white ``avionics_module`` (with the logo pocket); each has
its own printed mass (the 2-colour slice's per-filament part grams in the slicer table) and ``extra.livery`` records the
plain-module union, colours and the 2-colour slice summary.
2026-11-05 logic power (VTOL battery build, params configs.vtol.logic_power): ``extra.logic_power`` is the pending-mass
record (the pack, switch, lead, ties and the printed holed cap against the flight model; written also as
cad/exports/logic_power_pending_mass_vtol.json / .md) and the STEP carries the logic-power solids.  The named switch
logic_power.in_flight_mass_model (false: user decision 4, mass deferred) keeps them out of ``parts`` / ``totals`` and measures
the top cap's mass entry on the cap without the switch hole (``Part.mass_shape``; the STL is the holed cap,
``extra.printed_solid_volume_override_mm3``); the slicer table may hold '<part>@<variant>' entries (printed_mass_model).
Switch-collar round (user decision 2026-11-05): the printed cap also carries the protective collar (rocket_geometry
.logic_collar), behind the same mass switch - the flight-mass entry is still the cap without the hole and without the
collar (the previous round's solid, so the flight mass is unchanged); ``extra.logic_power.top_cap`` measures the hole and
the collar apart, ``extra.logic_power.layout.collar`` records the collar, and ``derived.vehicle_h_mm`` counts the printed
cap's real solid (the collar rim is the battery build's top).
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import struct
import sys
import time

import cadquery as cq
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import massprops as mp  # noqa: E402
import rocket_geometry as rg  # noqa: E402
import layout as layout_mod  # noqa: E402
from layout import PARAMS_PATH, Layout, load_params  # noqa: E402

ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
EXPORT_DIR = os.path.join(ROOT, "cad", "exports")
CQ_EXPORT_DIR = os.path.join(EXPORT_DIR, "cadquery")
ASSEMBLY_NAME = "CanSatEDFRocket"
STL_TOLERANCE_MM = 0.05          # absolute chordal deviation (0.4 mm nozzle, 0.2 mm layers)
STL_ANGULAR_TOLERANCE = 0.1      # rad
STL_VOLUME_REL_MAX = 1e-3        # v0.8: mesh volume within 0.1 % of the exact solid (test_stl_export's limit) ...
STL_REFINE_MAX = 3               # ... else re-export with both tolerances halved, at most this many times


def params_sha256(path: str = PARAMS_PATH) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def default_preset(path: str = PARAMS_PATH) -> str:
    return load_params(path)["meta"]["preset"]


def preset_suffix(preset: str | None) -> str:
    if preset is None or preset == default_preset():
        return ""
    return f"_{preset}"


def config_of(config: str | None, path: str = PARAMS_PATH) -> str:
    """v0.9 CAD-2: the hardware configuration a run builds (params mission.config when not given)."""
    return config or (load_params(path)["mission"].get("config") or "rocket")


def output_paths(preset: str | None, config: str | None = None, power: str | None = None) -> dict[str, str]:
    """Output files of a preset; v0.9 CAD-2: + the configuration suffix ('' rocket, '_vtol', '_vtol_tether', ...)."""
    sfx = preset_suffix(preset) + layout_mod.config_suffix(config_of(config), power)
    return {
        "stl_dir": os.path.join(CQ_EXPORT_DIR, f"stl{sfx}"),
        "step": os.path.join(CQ_EXPORT_DIR, f"{ASSEMBLY_NAME}{sfx}.step"),
        "report": os.path.join(EXPORT_DIR, f"cadquery_report{sfx}.json"),
        "checks_md": os.path.join(EXPORT_DIR, f"cadquery_checks{sfx}.md"),
        "checks_json": os.path.join(EXPORT_DIR, f"cadquery_checks{sfx}.json"),
        # 2026-11-05: the logic-power pending-mass report (VTOL battery build only)
        "logic_pending_json": os.path.join(EXPORT_DIR, f"logic_power_pending_mass{sfx}.json"),
        "logic_pending_md": os.path.join(EXPORT_DIR, f"logic_power_pending_mass{sfx}.md"),
    }


# --------------------------------------------------------------------------- #
# report
# --------------------------------------------------------------------------- #
SLICER_VOLUME_REL_TOL = 1e-6      # v0.9: a slicer-table entry is used only for the solid it was sliced from


def load_slicer_table(L: Layout) -> tuple[dict, str | None]:
    """v0.9 (printer.mass_model): the slicer part-mass table (parts.<base> -> part_g, solid_volume_mm3, ...) or {}."""
    mm = L.p["printer"].get("mass_model") or {}
    if mm.get("mode") != "slicer_part_g" or not mm.get("table"):
        return {}, None
    path = os.path.join(ROOT, mm["table"])
    if not os.path.exists(path):
        return {}, mm["table"]
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f).get("parts", {}), mm["table"]


def printed_mass_model(L: Layout, base: str, solid_volume_mm3: float, table: dict) -> dict:
    """v0.9 printed-part mass (printer.mass_model): 'slicer_part_g' -> the table's part grams when its solid volume is this
    part's (SLICER_VOLUME_REL_TOL), else the part class's effective_density_fallback_g_cm3 x volume ('class_fallback');
    mode 'solid' (or no mass_model) -> volume x printer.density_g_cm3 (the v0.2-v0.8 model)."""
    pr = L.p["printer"]
    mm = pr.get("mass_model") or {}
    rho = pr["density_g_cm3"]
    solid_g = solid_volume_mm3 * rho / 1000.0
    if mm.get("mode") != "slicer_part_g":
        return {"mass_g": solid_g, "model": "solid", "solid_mass_g": solid_g}
    cls = (pr.get("part_class") or {}).get(base)
    ent = table.get(base)
    # 2026-11-05: a part sliced in two geometries keeps the other one's slice as '<base>@<variant>' (the top cap with /
    # without the logic power switch hole: the battery build prints it with the hole, the tether / hybrid builds and the
    # flight-mass model while logic_power.in_flight_mass_model is false use the cap without it) - the entry whose solid
    # volume is THIS solid's is used, the same SLICER_VOLUME_REL_TOL rule
    cands = [(base, ent)] + sorted((k, v) for k, v in table.items() if k.startswith(base + "@"))
    for key, e_ in cands:
        if e_ and e_.get("part_g") and abs(e_["solid_volume_mm3"] / solid_volume_mm3 - 1.0) <= SLICER_VOLUME_REL_TOL:
            out = {"mass_g": float(e_["part_g"]), "model": "slicer_part_g", "solid_mass_g": solid_g, "class": cls,
                   "settings": e_.get("settings"), "slicer_z_centroid_design_mm": e_.get("part_z_centroid_design_mm")}
            if key != base:
                out["table_entry"] = key
            return out
    if cls is None:
        raise KeyError(f"printer.part_class has no class for printed part {base!r}")
    eff = pr["print_classes"][cls]["effective_density_fallback_g_cm3"]
    return {"mass_g": solid_volume_mm3 * eff / 1000.0, "model": "class_fallback", "solid_mass_g": solid_g, "class": cls,
            "why": ("no slicer-table entry" if not ent else
                    f"table solid volume {ent.get('solid_volume_mm3'):.1f} != {solid_volume_mm3:.1f} mm^3 (re-slice)")}


def part_entry(part: rg.Part, L: Layout | None = None, table: dict | None = None,
               models: dict | None = None) -> dict:
    """SPEC 6 report entry.  v0.9: a printed part's mass follows printer.mass_model (printed_mass_model) when L is given:
    the entry keeps the SPEC 6 keys - density_g_cm3 becomes the effective density (mass / solid volume), the inertia is
    the solid's scaled by mass / solid mass - and the model record goes into ``models`` (report extra.mass_model.parts).
    With the 'solid' model the entry is exactly the solid entry (bit for bit)."""
    if part.kind == "lumped":
        e = mp.lumped_properties(part.mass_g, part.point_mm)
    elif part.kind == "printed":
        # 2026-11-05: Part.mass_shape (the top cap without the logic switch hole while the mass decision is deferred)
        e = mp.part_properties(part.shape if part.mass_shape is None else part.mass_shape, density_g_cm3=part.density_g_cm3)
        if L is not None:
            m = printed_mass_model(L, part.base, e["volume_mm3"], table or {})
            if part.mass_shape is not None:
                m["mass_shape"] = ("the solid without the logic power switch hole and the switch collar "
                                   "(logic_power.in_flight_mass_model false, user decisions 2026-11-05) - the printed part / "
                                   "STL has both")
            if models is not None:
                models[part.name] = m
            if m["model"] != "solid":
                k = m["mass_g"] / e["mass_g"] if e["mass_g"] > 0 else 1.0
                e["mass_g"] = m["mass_g"]
                e["density_g_cm3"] = m["mass_g"] * 1000.0 / e["volume_mm3"]
                e["inertia_about_com_kg_m2"] = [[x * k for x in row] for row in e["inertia_about_com_kg_m2"]]
    else:
        e = mp.part_properties(part.shape, mass_g=part.mass_g)
    return {"kind": part.kind, **e}


def max_radius_mm(shape: cq.Shape) -> float:
    """Largest distance of any vertex from the vehicle axis (circumscribed footprint radius)."""
    return max(math.hypot(v.X, v.Y) for v in shape.Vertices())


def window_gusset_volume_mm3(L: Layout, intake: cq.Shape) -> float:
    """v0.6.2: volume of ``intake`` inside the window sectors over Z [z_window_bottom, z_gable_top + 1] (r 1 mm past
    the band faces) - the gussets under the gable, measured on the solid; NaN when OCC cannot compute it."""
    # v0.9: out to body_r only (the motor-lead groove's roof wedge stands outside the body beside the 0 deg strut,
    # inside the window sectors' angles but outside the band)
    region = rg._fuse([rg._sector(L.r_band_in - 1.0, L.body_r, lo, hi, L.z_window_bottom, L.z_gable_top + 1.0)
                       for lo, hi in L.window_sectors])
    v = rg.intersection_volume(intake, region)
    return v


def gps_extra(L: Layout, parts: dict[str, rg.Part], entries: dict[str, dict], totals: dict) -> dict:
    """v0.8: what the GPS pod adds, measured: the pod + raceway + pocket material (payload_module minus the same module
    built without the GPS features), the lid, the placeholders and the lead; the entry hole's volume; the antenna phase
    centre and its lever arm from the vehicle CG (the NAV subtracts w x r); the noise-source distances (Layout)."""
    rho = L.p["printer"]["density_g_cm3"]
    pay0 = rg.payload_module(L, with_gps=False).Volume()
    av0 = rg.avionics_module(L, with_gps=False).Volume()
    shell_g = (entries["payload_module"]["volume_mm3"] - pay0) * rho / 1000.0
    # v0.8.1: the hole alone - the module with the GPS features but without the GNS bosses (those are extra.gns)
    hole_mm3 = av0 - rg.avionics_module(L, with_gns=False).Volume()
    added = {"pod + raceway + pocket (in payload_module)": shell_g, "gps_pod_lid": entries["gps_pod_lid"]["mass_g"],
             "gps (board + patch)": entries["gps"]["mass_g"], "gps_ground_plane": entries["gps_ground_plane"]["mass_g"],
             "gps_lead (lead, connector, hardware)": entries["gps_lead"]["mass_g"],
             "avionics entry hole (removed)": -hole_mm3 * rho / 1000.0}
    arm = [a - c for a, c in zip(L.gps_antenna_xyz, totals["com_mm"])]
    return {"added_mass_g": added, "added_total_g": sum(added.values()),
            "payload_module_without_gps_mass_g": pay0 * rho / 1000.0,
            "avionics_entry_hole_mm3": hole_mm3,
            "antenna_phase_centre_mm": list(L.gps_antenna_xyz),
            "antenna_lever_arm_from_cg_mm": arm, "antenna_lever_arm_norm_mm": math.sqrt(sum(a * a for a in arm)),
            "noise_source_distance_mm": dict(L.gps_noise_mm),
            "note": "phase centre ASSUMED (payload_module.gps_pod.stack.phase_centre_below_patch_top_ASSUMED); masses of the "
                    "board (electronics.gps.board_mass_g), plate, lead and hardware are params values (ASSUMED until weighed)"}


def _layout_with(L: Layout, edit) -> Layout:
    """A Layout of a deep copy of L's params after ``edit(params)`` (v0.8.1 comparisons against the v0.8 build)."""
    p = copy.deepcopy(L.p)
    edit(p)
    return Layout(p)


def gns_extra(L: Layout, parts: dict[str, rg.Part], entries: dict[str, dict], totals: dict) -> dict:
    """v0.8.1: what the GNS board adds, measured: the four bosses (avionics_module minus the same module built without
    them), the placeholder (board + screws), the GPS lead change (this route vs the v0.8 route to the NAV, Layout without
    electronics.gns), the glink lead estimate (inside the board's ASSUMED mass); the GPS-pod mass mitigation re-measured
    against payload_module.gps_pod.mass_mitigation_v081.from (payload module + lid + lead built with those values); the
    board's distances (ESC, antenna, ...) and the vehicle T/W at thrust_nominal (CAD mass, without the sizing balance)."""
    rho = L.p["printer"]["density_g_cm3"]
    av0 = rg.avionics_module(L, with_gns=False).Volume()
    bosses_g = (entries["avionics_module"]["volume_mm3"] - av0) * rho / 1000.0
    L08 = _layout_with(L, lambda p: p["electronics"].pop("gns", None))
    added = {"gns (board + leads ASSUMED, screws)": entries["gns"]["mass_g"], "bosses + tap holes (in avionics_module)": bosses_g,
             "gps_lead: route to the GNS instead of the NAV": entries["gps_lead"]["mass_g"] - L08.gps_lead_mass_g}
    mit = L.p["payload_module"]["gps_pod"].get("mass_mitigation_v081") or {}
    pod_mit = None
    if mit.get("from"):
        def back(p: dict) -> None:
            g = p["payload_module"]["gps_pod"]
            for path, val in mit["from"].items():
                d = g
                keys = path.split(".")
                for k in keys[:-1]:
                    d = d[k]
                d[keys[-1]] = val
        Lb = _layout_with(L, back)
        before = {"payload_module": rg.payload_module(Lb).Volume() * rho / 1000.0,
                  "gps_pod_lid": rg.gps_pod_lid(Lb).Volume() * rho / 1000.0, "gps_lead": Lb.gps_lead_mass_g}
        now = {k: entries[k]["mass_g"] for k in before}
        pod_mit = {"params_from": mit["from"], "params_now": {k: (L.p["payload_module"]["gps_pod"][k.split(".")[0]][k.split(".")[1]]
                                                                  if "." in k else L.p["payload_module"]["gps_pod"][k])
                                                              for k in mit["from"]},
                   "mass_from_g": before, "mass_now_g": now, "saved_g": sum(before.values()) - sum(now.values()),
                   "antenna_phase_centre_from_mm": list(Lb.gps_antenna_xyz)}
    m_kg = totals["mass_g"] * 1e-3
    return {"added_mass_g": added, "added_total_g": sum(added.values()),
            "board_box_mm": [list(L.gns_x), list(L.gns_y), list(L.gns_z)], "board_centre_mm": list(L.gns_center),
            "board_mass_g": L.gns_board_mass_g, "screws_g": L.gns_screws_g, "theta_deg": L.gns_theta,
            "boss_hole_vz_mm": [list(q) for q in L.gns_hole_vz], "boss_half_diag_mm": L.gns_boss_hd,
            "hole_u_mm": list(L.gns_hole_u), "hole_d_mm": L.gns_hole_d,
            "gps_lead_route_mm": L.gps_lead_route_len, "gps_lead_route_v08_mm": L08.gps_lead_route_len,
            "glink_lead_len_mm": L.glink_len_mm, "glink_lead_mass_g_estimate": L.glink_mass_g,
            "distance_mm": dict(L.gns_distance_mm), "esc_target_mm": L.gns_esc_target_mm,
            "esc_best_on_wall_mm": L.gns_esc_best_on_wall_mm, "clearance_mm": dict(L.gns_clearance),
            "pod_mitigation": pod_mit,
            "thrust_to_weight_nominal_cad": L.p["edf"]["thrust_nominal_N"] / (m_kg * 9.80665),
            "note": "board mass electronics.gns.mass_g_ASSUMED (20 g board + 2 g shifter + 3 g leads per its note) + "
                    "avionics_module.gns_mount.screws_g_ASSUMED; the glink lead estimate is inside the board mass, not added "
                    "again; header positions (gps_header_v / glink_header_v) ASSUMED"}


def livery_extra(L: Layout, parts: dict[str, rg.Part], entries: dict[str, dict], models: dict, table: dict) -> dict:
    """livery (user decisions 2026-11-04): the avionics module printed as TWO parts of one 2-filament object - the white
    body (``avionics_module``, with the logo pocket) and the orange inlay (``livery.inlay_part``) - each with its own mass
    (printer.mass_model: the slicer's per-filament part grams of the 2-colour project, else the class fallback), so the
    vehicle mass keeps both; the plain module (no pocket) is measured for the union (C36 judges it)."""
    pr = L.p["printer"]
    w, i = L.livery_part, L.livery_inlay_part
    plain = rg.avionics_module(L, with_livery=False)
    vp, vw, vi = mp.precise_volume(plain), mp.precise_volume(parts[w].shape), mp.precise_volume(parts[i].shape)
    rho = pr["density_g_cm3"]
    ams = (table.get(i) or {}).get("two_colour") or (table.get(w) or {}).get("two_colour")
    return {"part": w, "inlay_part": i, "copies_theta_deg": list(L.livery_thetas), "inlay_depth_mm": L.livery_depth,
            "white_behind_mm": L.livery_white_behind, "logo_file": L.livery_file, "logo_file_sha256": L.livery_file_sha256,
            "polygon_set": L.livery_poly_set, "islands_per_copy": len(L.livery_polys_vz),
            "z_vehicle_mm": list(L.livery_z_range), "z_print_mm": list(L.livery_z_print),
            "colours": {"body": dict(L.livery_colours["body"]), "logo": dict(L.livery_colours["logo"]),
                        "printed_parts": {"color": pr.get("color"), "hex": pr.get("color_hex"), "code": pr.get("color_code")},
                        "launch_pad": (pr.get("color_exceptions") or {}).get("launch_pad")},
            "volume_precise_mm3": {"plain_module": vp, "white_body": vw, "inlay": vi,
                                   "white_plus_inlay_minus_plain": vw + vi - vp},
            "mass_g": {"white_body": entries[w]["mass_g"], "inlay": entries[i]["mass_g"],
                       "sum": entries[w]["mass_g"] + entries[i]["mass_g"],
                       "plain_solid_at_rho": vp * rho / 1000.0,
                       "models": {w: (models.get(w) or {}).get("model"), i: (models.get(i) or {}).get("model")}},
            "two_colour_slice": ams,
            "note": "STEP colours: printed parts white (PRINTED_COLOR), the inlay orange (LIVERY_COLOR), the launch pad "
                    "black (unchanged). The inlay is one compound of disjoint islands (2 copies x islands)."}


def make_report(L: Layout, parts: dict[str, rg.Part], warnings: list[str] | None = None,
                build_time_s: float | None = None) -> dict:
    """SPEC section 6 report (identical schema for both generators) plus an ``extra`` block."""
    warnings = list(warnings or [])
    table, table_path = load_slicer_table(L)
    models: dict[str, dict] = {}
    entries = {name: part_entry(p, L, table, models) for name, p in parts.items()}
    fallback = sorted({p.base for n, p in parts.items() if p.kind == "printed"
                       and models.get(n, {}).get("model") == "class_fallback"})
    if fallback:
        warnings.append("printed-part mass from the class fallback density (no matching slicer-table entry - run "
                        "cad/cadquery/make_v09_print_files.py, then rebuild): " + ", ".join(fallback))

    vehicle = [n for n, p in parts.items() if p.in_vehicle]
    printed = [n for n in vehicle if parts[n].kind == "printed"]
    totals = mp.combine([entries[n] for n in vehicle])
    vtol = getattr(L, "vtol", False)
    post_names = [n for n in vehicle if n not in rg.POST_EJECTION_EXCLUDED]
    post = mp.combine([entries[n] for n in post_names])
    # v0.9 CAD-2 VTOL: nothing is ejected and no chute is carried (the vehicle lands under power)
    post["descent_mass_g"] = post["mass_g"] + (0.0 if vtol else entries["chute_pack"]["mass_g"])
    totals["printed_mass_g"] = sum(entries[n]["mass_g"] for n in printed)
    totals["post_ejection"] = post

    zmin = min(entries[n]["bbox_mm"][0][2] for n in vehicle if parts[n].kind != "lumped")
    zmax = max(entries[n]["bbox_mm"][1][2] for n in vehicle if parts[n].kind != "lumped")
    if getattr(L, "has_logic_power", False) and "logic_switch" not in parts and not getattr(L, "lp_rocket", False):
        # review fix (2026-11-05, F1): the height is geometry - the logic switch lever (the battery build's top) counts
        # whatever logic_power.in_flight_mass_model is (the mass switch keeps it out of the masses only)
        zmax = max(zmax, rg.logic_switch(L).BoundingBox().zmax)
    # switch-collar round (2026-11-05): a printed part whose mass entry is measured on another solid (the top cap without
    # the switch hole and the collar) counts with its REAL solid - the collar rim is the battery build's top
    for n in vehicle:
        if parts[n].kind == "printed" and parts[n].mass_shape is not None:
            zmax = max(zmax, mp.bbox(parts[n].shape)[1][2])
    # v0.9 direct drive: no pushrods (pushrod_len_mm = 0 keeps the SPEC 6 schema)
    pushrod_lens = [rg.pushrod_length(L, i) for i in range(L.vane_count)] if not L.direct_drive else [0.0] * L.vane_count
    if max(pushrod_lens) - min(pushrod_lens) > 1e-6:
        warnings.append(f"pushrod lengths differ between pods: {pushrod_lens}")
    cavity_mm3 = 0.0 if vtol else rg.nose_cavity_volume_mm3(L, parts["nose_cone"].shape)
    # v0.6.1: the opening of the built intake band at INLET_PROBE_COUNT radii; the minimum is the inlet area (C9)
    openings = rg.intake_opening_areas_mm2(L, parts["intake_module"].shape)
    areas = [a for _, a in openings]
    window_area = math.nan if any(math.isnan(a) for a in areas) else min(areas)
    footprint = 2.0 * max_radius_mm(parts["propulsion_module"].shape)
    gussets = window_gusset_volume_mm3(L, parts["intake_module"].shape) if L.has_window_gable else 0.0

    derived = {
        "pushrod_len_mm": pushrod_lens[0],
        "nose_cavity_volume_L": cavity_mm3 / 1e6,
        "footprint_d_mm": footprint,
        "vehicle_h_mm": zmax - zmin,
        "inlet_area_ratio": L.inlet_area_ratio,
    }
    extra = {
        "excluded_from_totals": [n for n, p in parts.items() if not p.in_vehicle],
        "launch_pad_mass_g": entries["launch_pad"]["mass_g"] if "launch_pad" in entries else None,
        "inlet_area_mm2_layout": L.inlet_area_mm2,
        "inlet_area_mm2_layout_outer": L.inlet_area_outer_mm2,
        "inlet_area_mm2_measured": window_area,
        "inlet_area_ratio_measured": window_area / L.fan_annulus_area_mm2,
        "inlet_area_mm2_measured_outer": areas[-1],
        "inlet_area_ratio_measured_outer": areas[-1] / L.fan_annulus_area_mm2,
        "inlet_area_ratio_layout_outer": L.inlet_area_ratio_outer,
        "inlet_opening_probes_mm2": [[r, a] for r, a in openings],
        "inlet_opening_z_mm": list(rg.intake_opening_z_range(L)),
        # v0.6.2 window gable: gussets measured on the built intake (band over the window sectors, z_window_bottom ..
        # z_gable_top) against Layout's integral; the square-window inlet area for comparison
        "window_gable": ({"slope_deg": L.gable_slope_deg, "bridge_w_mm": L.gable_bridge_w,
                          "gable_h_inner_mm": L.gable_h_inner, "gable_h_outer_mm": L.gable_h_outer,
                          "edge_slope_min_deg_layout": L.gable_edge_slope_min_deg,
                          "gusset_volume_mm3_measured": gussets, "gusset_volume_mm3_layout": L.gusset_volume_mm3,
                          "gusset_mass_g_measured": gussets * L.p["printer"]["density_g_cm3"] / 1000.0,
                          "inlet_area_mm2_square_windows": L.inlet_area_square_mm2,
                          "inlet_area_ratio_square_windows": L.inlet_area_ratio_square}
                         if L.has_window_gable else None),
        "nose_cavity_volume_L_layout": None if vtol else L.nose_cavity_volume_L,
        "pushrod_len_mm_all": pushrod_lens,
        "spigot_shoulder_t_mm": rg.spigot_shoulder_t(L),
        "latch_arm_tip_xy_latched": None if vtol else list(rg.latch_arm_tip_xy(L)),
        "latch_arm_tip_xy_released": None if vtol else list(rg.latch_arm_tip_xy(L, released=True)),
        "cg_above_vane_hinge_mm": totals["com_mm"][2] - L.vane_hinge_z,
        "thrust_to_weight_nominal": L.p["edf"]["thrust_nominal_N"] / (totals["mass_g"] * 1e-3 * 9.80665),
        "inertia_convention": "standard tensor about the part COM, I_xy = -sum(m x y); kg*m^2",
        "cadquery_version": cq.__version__,
        "build_time_s": build_time_s,
    }
    if getattr(L, "has_gps_pod", False):
        extra["gps"] = gps_extra(L, parts, entries, totals)
    if getattr(L, "has_gns", False):
        extra["gns"] = gns_extra(L, parts, entries, totals)
    if not getattr(L, "vtol", False) and getattr(L, "fcu_part", "") == "carrier":
        # rocket-carrier round (2026-11-14): the carrier PCB rev 3.1 at the FCU spot, its envelope and clearances (Layout)
        extra["carrier"] = {"box_mm": [list(L.fcu_x), list(L.fcu_y), list(L.fcu_z)], "mass_g": L.fcu_mass_g,
                            "envelope_boxes_mm": {k: [list(a) for a in b] for k, b in L.carrier_boxes.items()},
                            "release_pad_xyz_mm": list(L.carrier_release_pad) if L.carrier_release_pad else None,
                            "vent_tie_pairs_deg": [list(t) for t in L.fcu_tie_pairs],
                            "clearance_mm": dict(L.fcu_clearance),
                            "limits_mm": {k: v for k, v in L.fcu_limits.items()},
                            "worst_envelope_box": dict(L.carrier_box_worst),
                            "gps_lead_end": getattr(L, "gps_lead_end", None),
                            "release_lead_clearance_mm": dict(getattr(L, "rl_clearance", {}) or {})}
    # v0.9: mass model summary, direct-drive and motor-lead numbers
    printed_names = [n for n in vehicle if parts[n].kind == "printed"]
    extra["mass_model"] = {
        "mode": (L.p["printer"].get("mass_model") or {}).get("mode", "solid"), "table": table_path,
        "printed_solid_mass_g": sum(models.get(n, {}).get("solid_mass_g", entries[n]["mass_g"]) for n in printed_names),
        "printed_mass_g": totals["printed_mass_g"],
        "by_model": {m: sorted({parts[n].base for n in printed_names if models.get(n, {}).get("model") == m})
                     for m in ("slicer_part_g", "class_fallback", "solid")},
        "parts": models}
    # v0.9 fix round (geometry reviewer): OCC's default volume integration under-reports some solids (VTOL propulsion module
    # -0.7 %); the report entries keep it (bit-identical history; printed grams come from the slicer anyway) and the precise
    # value is recorded here for every printed base where the two differ by more than 0.01 %
    prec = {}
    for n in printed_names:
        if parts[n].base in prec:
            continue
        vp = mp.precise_volume(parts[n].shape if parts[n].mass_shape is None else parts[n].mass_shape)
        vr = entries[n]["volume_mm3"]
        if abs(vp / vr - 1.0) > 1e-4:
            prec[parts[n].base] = {"report_mm3": vr, "precise_mm3": vp, "rel": vp / vr - 1.0}
    extra["printed_volume_precise"] = prec
    if getattr(L, "has_livery", False):
        extra["livery"] = livery_extra(L, parts, entries, models, table)
    if L.direct_drive:
        extra["direct_drive"] = {
            "fin_thetas_deg": list(L.fin_thetas), "column_thetas_deg": list(L.sl_col_thetas) if L.servo_leads else None,
            "coupler_u_mm": [L.dd_u_hub0, L.dd_u_hub1, L.dd_u_flange1],
            "horn_u_mm": ([getattr(L, "dd_u_screw0", L.dd_u_flange1), getattr(L, "dd_u_arm0", L.dd_u_flange1), L.dd_u_horn1,
                           getattr(L, "dd_u_collar1", L.dd_u_horn1)]),
            # v0.9 fix round: set screw / socket / bushing / foam / retention / lost-motion estimate
            "set_screw_u_mm": getattr(L, "dd_set_screw_u", None), "flat_side": getattr(L, "dd_flat_side", "-z"),
            "socket_slot_half_w_mm": getattr(L, "dd_slot_hw", None), "inward_stop_mm": getattr(L, "dd_inward_stop", None),
            "shaft_end_to_horn_screw_mm": getattr(L, "dd_shaft_to_screw", None),
            "bushing": ({"od": L.dd_bush_od, "id_ASSUMED": L.dd_bush_id, "u_mm": list(L.dd_bush_u), "mass_g": L.dd_bush_mass_g,
                         "tilt_deg": L.dd_bush_tilt_deg} if getattr(L, "dd_bushing", False) else None),
            "foam_compression": list(L.dd_foam_compression) if getattr(L, "dd_foam", False) else None,
            "deadband_pp_deg": ({"socket": L.dd_deadband_slot_deg, "gear_backlash_ASSUMED": L.dd_gear_backlash_deg,
                                 "total": L.dd_deadband_pp_deg} if hasattr(L, "dd_deadband_pp_deg") else None),
            "case_u_mm": [L.dd_u_case_top, L.dd_u_case_bot], "case_z_mm": list(L.dd_z_case), "ears_z_mm": list(L.dd_z_ear),
            "fairing_u_mm": list(L.fair_u), "fairing_half_w_mm": L.fair_hv, "fairing_z_mm": list(L.fair_z),
            "keel_z0_mm": L.fair_keel_z0, "pocket_u_mm": list(L.dd_pocket_u), "pocket_z_mm": list(L.dd_pocket_z),
            "shaft_r_out_mm": L.vane_shaft_r_out, "d_flat_r_in_mm": L.vane_d_flat_r_in, "shaft_engagement_mm": L.dd_engagement,
            "servo_float_mm": L.dd_float, "print_tolerance_ASSUMED_mm": L.dd_tol,
            "hinge_moment_kgcm": L.vane_hinge_moment_kgcm, "vane_side_load_N": L.vane_side_load_N,
            "servo_torque_kgcm": L.servo_torque_kgcm, "hinge_margin": L.vane_hinge_margin,
            "lead_pipe_walls_mm": dict(getattr(L, "sl_pipe_walls", {})),
            "servo_lead_route_mm": list(L.sl_route_len) if L.servo_leads else None,
            "servo_lead_extension_mm": list(L.sl_ext_len) if L.servo_leads else None}
    if getattr(L, "has_motor_leads", False):
        extra["motor_leads"] = {"route_mm": L.ml_route_len, "lead_len_each_mm": L.ml_lead_len, "mass_g": L.ml_mass_g,
                                "com_mm": list(L.ml_com), "sections_mm": L.ml_sections,
                                "joint_zone_mm": [list(b) for b in L.ml_joint_box],
                                "joint_zone_clearance_mm": dict(L.ml_joint_clearance),
                                "note": "lead OD, g/m, bullet size and mass are motor_leads.*_ASSUMED - measure"}
        if getattr(L, "clock_mount", False):    # wire09c clock mount (user 2026-10-30: generous, no measuring)
            extra["motor_leads"]["clock_route"] = {
                "lead_passage_theta_deg": L.wire_theta, "E_design_deg": L.lp_E_design, "E_cov_deg": L.lp_E_cov,
                "tab_play_deg": L.lp_tab_play_deg, "fan_half_width_mm": L.lp_hw, "r_wall_mm": L.lp_r_wall,
                "blister_r_out_mm": L.lp_blister_r_out, "blister_r_max_mm": L.lp_blister_r_max,
                "fan_floor_z_mm": L.lp_z_fan_floor, "blister_z_mm": [L.lp_z_blister_bottom, L.lp_z_blister_top],
                "keel_low_z_mm": L.lp_keel_z_low, "keyways_deg": list(L.ck_keyways), "keyway_floor_z_mm": L.lp_z_floor,
                "min_sleeve_tooth_mm": L.ck_min_tooth, "seated_configs": len(L.ck_configs), "hover_seats": len(L.ck_hover),
                "tooth_bending_plan_E": dict(L.ck_tooth), "seat_plan_mod180": [list(s_) for s_ in L.ck_seat_plan],
                "coverage": L.ck_coverage, "worst_residual_deg": L.ck_worst_residual_deg,
                "largest_seat_gap_deg": L.ck_largest_gap_deg, "nominal_tab_thetas_deg": list(L.tab_thetas),
                "pad_t_mm": L.ck_pad_t, "pad_span_mm": L.ck_pad_span,
                "generosity_box": L.p["motor_leads"]["generosity_box"], "longest_corner": dict(L.ml_clock_longest),
                "lower_len_mm (longest corner to the steer end)": L.ml_lower_len,
                "jet_drag_E": {k: (list(v) if isinstance(v, tuple) else v) for k, v in L.ml_jet_drag.items()},
                "jet_blockage": {k: v for k, v in L.ml_blockage.items()}, "duct_window_mm2_E": L.lp_duct_window_mm2,
                "clearances_mm_E": dict(L.ml_lower_clear), "lumped_extras_g_ASSUMED": L.ml_rtv_g,
                "foam_compression": dict(L.ar_foam_compression), "lift_max_mm": L.ar_lift_max,
                "note": "wire09c: the EDF is clocked to ONE lead passage (blister); keyways round the circle; the factory "
                        "sleeve is kept; every corner of motor_leads.generosity_box is checked by C29 (no measuring)"}
        elif getattr(L, "ml_lower_uz", None):     # v0.9 lead-exit, photo round (user photos 2026-10-29)
            extra["motor_leads"]["lower_route"] = {
                "motor_rear_end_z_mm": L.ml_z_motor_bottom, "motor_front_ring_z_mm_ASSUMED": list(L.ml_ring_z),
                "exit_z_mm": L.ml_z_exit, "bundle_od_mm_ASSUMED": L.ml_bundle_d, "shrink_end_u_mm_ASSUMED": L.ml_u_shrink_end,
                "bend_start_minus_shrink_end_mm": L.ml_shrink_end_margin,
                "transition_len_mm_E": L.ml_transition_len,
                "bend_start_minus_shrink_and_transition_mm": L.ml_shrink_bend_margin,
                "bundle_bend_angle_deg": L.ml_shrink_bend_phi_deg,
                "exit_angle_tol_deg_E": L.ml_exit_angle_tol_deg, "exit_angle_measured": L.ml_theta_measured,
                "exit_angle_uncertainty_deg_ASSUMED": L.ml_theta_uncert_deg,
                "stator_run_z_mm": L.ml_z_run, "stator_te_z_mm_ASSUMED": L.ml_stator_te_z, "stator_te_gap_mm": L.ml_stator_te_gap,
                "slot_u_mm": L.ml_u_slot, "bend_r_centre_mm_ASSUMED": L.ml_bend_r, "segments_mm": dict(L.ml_lower_segments),
                "lower_len_mm": L.ml_lower_len, "rtv_bond_u_mm": list(L.ml_bond_u), "free_spans_mm": list(L.ml_lower_free_spans),
                "rtv_g_ASSUMED": L.ml_rtv_g, "jet_drag_E": {k: (list(v) if isinstance(v, tuple) else v) for k, v in L.ml_jet_drag.items()},
                "notch_uz_mm": [list(q) for q in L.ml_low_cut_uz], "notch_w_mm": L.ml_low_cut_w,
                "clearances_mm": dict(L.ml_lower_clear), "jet_blockage": dict(L.ml_blockage),
                "note": "photo round: exit at the motor's front ring, heat-shrink bundle behind one stator vane, under the rear "
                        "rim, single leads bend up the slot. edf.wire_exit motor_exit / stator_vane and motor_leads.lower_route "
                        "(bundle) are ASSUMED (read off the user photos) except the rim / motor heights (drawing data) - measure. "
                        "Fix round: the factory sleeve is replaced by an 18 mm re-sleeve (assembly spec), the leads go from the "
                        "triangle to a row across the slot before the bend (transition, E)"}
    out = {
        "generator": "cadquery",
        "generator_version": f"cadquery {cq.__version__}",
        "preset": L.p["meta"]["preset"],
        "params_sha256": params_sha256(),
        "parts": entries,
        "totals": totals,
        "derived": derived,
        "warnings": warnings,
        "extra": extra,
    }
    if vtol:     # v0.9 CAD-2: the configuration keys (absent = rocket, so the rocket report keeps its schema)
        out["config"] = L.config
        out["power_source"] = L.power_source
        extra["vtol"] = vtol_extra(L, parts, entries, totals)
    if getattr(L, "lp_rocket", False):     # rocket-carrier round (2026-11-14): the logic power flies (no pending record)
        extra["logic_power"] = rocket_logic_power_extra(L, parts, entries, totals)
    elif getattr(L, "has_logic_power", False):     # 2026-11-05 logic power (battery build): pending-mass record
        extra["logic_power"] = logic_power_extra(L, parts, entries, totals, table)
        # the printed solid of a part whose mass entry is measured on another solid (make_v09_print_files slices the STL)
        extra["printed_solid_volume_override_mm3"] = {
            p_.base: mp.volume_properties(p_.shape)[0] for p_ in parts.values()
            if p_.kind == "printed" and p_.mass_shape is not None}
    return out


LOGIC_MASS_TARGET_G = 1650.0      # architect target, VTOL battery build (meta.v09_cad2_note, sizing/vtol_sizing.py targets_g)


def _mass_summary(t: dict) -> dict:
    I = t["inertia_about_com_kg_m2"]
    return {"mass_g": t["mass_g"], "com_mm": list(t["com_mm"]), "lateral_offset_mm": math.hypot(t["com_mm"][0], t["com_mm"][1]),
            "inertia_diag_kg_m2": [I[0][0], I[1][1], I[2][2]], "inertia_about_com_kg_m2": I}


def collar_record(L: Layout) -> dict:
    """Switch-collar round (2026-11-05): the protective collar's Layout numbers (dimensions A / design, sweep, finger,
    sky and section values E) for the pending-mass report and the checks."""
    return {"inner_d_mm": 2 * L.lp_col_ri, "outer_d_mm": 2 * L.lp_col_ro, "wall_t_mm": L.lp_col_t,
            "z_mm": list(L.lp_col_z), "h_over_plate_mm": L.lp_col_z[1] - L.lp_col_z[0],
            "notch_w_mm": L.lp_col_notch_w, "notch_bottom_z_mm": L.lp_col_notch_z0, "notch_axis": L.lp_col_axis,
            "notch_shape": "U (round bottom of radius notch_w / 2, review fix 2026-11-05)",
            "notch_round_centre_z_mm": L.lp_col_notch_zc,
            "finger_path_E": {k: v for k, v in L.lp_col_finger_path.items()},
            "plate_root_E": dict(L.lp_col_plate_root),
            "wall": dict(L.lp_col_wall), "volume_mm3_layout": L.lp_col_volume_mm3, "centroid_mm": list(L.lp_col_centroid),
            "lever_sweep": {"pivot_z_mm": L.lp_lever_pivot_z, "arm_mm": L.lp_lever_len, "throw_deg_A": L.lp_lever_throw_deg,
                            "top_z_mm": L.lp_lever_env_top_z, "tip_y_mm": L.lp_lever_tip_y,
                            "end_corner_z_mm": L.lp_lever_corner_z},
            "hull_chord_y_mm": L.lp_col_hull_chord_y,
            "finger": {k: (list(map(list, v)) if k == "centres" else v) for k, v in L.lp_col_finger.items()},
            "gps_sky_E": dict(L.lp_col_gps),
            "section_E": {"arc_z_min_mm3": L.lp_col_section["arc"]["z_min"], "arc_area_mm2": L.lp_col_section["arc"]["area"],
                          "arc_arm_mm": L.lp_col_section["arc_arm_mm"], "ring_z_mm3": L.lp_col_section["ring_z_mm3"],
                          "ring_arm_mm": L.lp_col_section["ring_arm_mm"],
                          "arc_z_over_plate_mm": L.lp_col_section["arc_z_over_plate_mm"],
                          "arc_half_width_mm": L.lp_col_section["arc_half_width_mm"],
                          "arc_k_per_mm2": L.lp_col_section["arc_k_per_mm2"]},
            "clearance_mm": {k: v for k, v in L.lp_clearance.items() if "collar" in k or "lever sweep" in k
                             or "finger tip" in k or "finger path" in k or "nut driver" in k}}


def rocket_logic_power_extra(L: Layout, parts: dict[str, rg.Part], entries: dict[str, dict], totals: dict) -> dict:
    """rocket-carrier round (2026-11-14, params logic_power switch_mount 'tube_wall', in_flight_mass_model true by default -
    user decision pending): the logic pack, the wall switch, its lead and the retention are report parts (they fly); this
    record names them, their masses / COMs, the vehicle WITHOUT them (for comparison) and the Layout lead / switch data.
    The arm switch and its lead stay inside electronics.misc; the release lead is reported (Layout.rl_*), not added."""
    names = sorted(rg.logic_power_parts(L))
    inn = [n for n in names if n in entries]
    m_lp = sum(entries[n]["mass_g"] for n in inn)
    tm, tc = totals["mass_g"], totals["com_mm"]
    wo_m = tm - m_lp
    wo_c = [(tm * tc[i] - sum(entries[n]["mass_g"] * entries[n]["com_mm"][i] for n in inn)) / wo_m for i in range(3)]
    return {"in_flight_mass_model": bool(L.logic_in_flight), "switch_mount": "tube_wall",
            "parts_in_report": inn, "parts_g": {n: entries[n]["mass_g"] for n in inn}, "total_g": m_lp,
            "without_logic_power": {"mass_g": wo_m, "com_mm": wo_c},
            "wall_switches": {k: {"theta_deg": w["theta"], "z_mm": w["z"], "hole_d": w["hole_d"], "connector": w["connector"]}
                              for k, w in L.lp_wall_sw.items()},
            "leads_mm": {"switch -> J15 (route + slack)": L.lp_switch_lead_len_mm, "arm switch -> J16 (route + slack)":
                         L.lp_arm_lead_len_mm, "pack -> J14 (route + slack, the pack's own lead)": L.lp_pack_lead_len_mm},
            "arm_lead_mass_g_inside_misc": L.lp_arm_lead_mass_g,
            "release_lead": ({"route_mm": L.rl_route_mm, "length_mm": L.rl_len_mm, "mass_g_E": L.rl_mass_g,
                              "in_mass_model": L.rl_in_mass_model, "com_mm": list(L.rl_com)}
                             if getattr(L, "has_release_lead", False) else None)}


def logic_power_extra(L: Layout, parts: dict[str, rg.Part], entries: dict[str, dict], totals: dict, table: dict) -> dict:
    """2026-11-05 (user decisions 1-4, params configs.vtol.logic_power): the logic pack, switch, switch lead and ties
    (rocket_geometry.logic_power_parts, measured like any part) and the printed top cap WITH the switch hole, against the
    flight mass model.  ``in_flight_mass_model`` false (the user deferred the mass decision): the report's parts / totals
    are the flight model WITHOUT them (the cap entry measured on the cap without the hole) and ``with_logic_power`` is what
    the vehicle weighs with them - the pending mass decision; true: they are in the parts / totals already and
    ``without_logic_power`` is the model without them (for comparison)."""
    lp = L.p["logic_power"]
    vehicle = [n for n, p_ in parts.items() if p_.in_vehicle]
    lpp = rg.logic_power_parts(L)
    pend = {n: part_entry(p_) for n, p_ in lpp.items()}
    cap = parts["top_cap"]
    m_real: dict = {}
    real = part_entry(rg.Part(cap.name, cap.kind, cap.base, cap.shape, cap.module, cap.orientation, cap.color,
                              density_g_cm3=cap.density_g_cm3), L, table, m_real)
    m_noh: dict = {}
    noh = part_entry(rg.Part(cap.name, cap.kind, cap.base, rg.top_cap(L, with_logic_hole=False), cap.module,
                             cap.orientation, cap.color, density_g_cm3=cap.density_g_cm3), L, table, m_noh)
    others = [entries[n] for n in vehicle if n != "top_cap" and n not in lpp]
    with_ = mp.combine(others + [real] + list(pend.values()))
    without = mp.combine(others + [noh])
    fs, ws, os_ = _mass_summary(totals), _mass_summary(with_), _mass_summary(without)
    full = fs if L.logic_in_flight else ws
    base = os_ if L.logic_in_flight else fs
    # switch-collar round (2026-11-05): the printed cap carries the hole AND the protective collar; the flight model has
    # neither (both are logic-power kit); the hole and the collar are measured apart on the cap with the hole only
    v_hole_only = mp.part_properties(rg.top_cap(L, with_logic_collar=False), density_g_cm3=cap.density_g_cm3)["volume_mm3"]
    has_col = bool(getattr(L, "has_lp_collar", False))
    rec_cap = {"printed_volume_mm3 (with the switch hole and the collar)": real["volume_mm3"],
               "flight_model_volume_mm3 (without them)": noh["volume_mm3"],
               "hole_removed_mm3": noh["volume_mm3"] - v_hole_only,
               # review fix (2026-11-05): the U notches' curved intersections need the adaptive integration (the default
               # GProp call under-reports this difference by ~0.05 mm^3, 2.5e-5); the entries' volumes stay default
               "collar_added_mm3": (mp.precise_volume(rg.top_cap(L)) - mp.precise_volume(rg.top_cap(L, with_logic_collar=False))
                                    if has_col else real["volume_mm3"] - v_hole_only),
               "collar_volume_mm3_layout": getattr(L, "lp_col_volume_mm3", 0.0) if has_col else 0.0,
               "has_collar": has_col,
               "printed_mass_g": real["mass_g"], "printed_mass_model": m_real.get("top_cap"),
               "flight_model_mass_g": noh["mass_g"], "flight_model_mass_model": m_noh.get("top_cap"),
               "mass_delta_g (printed - flight model)": real["mass_g"] - noh["mass_g"]}
    return {
        "decisions": {k: lp[k] for k in ("note", "applies_note", "in_flight_mass_model_note")},
        "in_flight_mass_model": bool(L.logic_in_flight),
        "applies_to_power_sources": list(lp["applies_to_power_sources"]),
        "pending_parts": pend,
        "pending_parts_g": {n: e["mass_g"] for n, e in pend.items()},
        "top_cap": rec_cap,
        "flight_model": fs,
        "with_logic_power": ws,
        "without_logic_power": os_,
        "delta_full_minus_without": {
            "mass_g": full["mass_g"] - base["mass_g"],
            "com_mm": [a - b for a, b in zip(full["com_mm"], base["com_mm"])],
            "lateral_offset_mm": full["lateral_offset_mm"] - base["lateral_offset_mm"],
            "inertia_diag_rel": [a / b - 1.0 for a, b in zip(full["inertia_diag_kg_m2"], base["inertia_diag_kg_m2"])]},
        "mass_target_g": LOGIC_MASS_TARGET_G,
        # review fix (2026-11-05, F3): T/W at the design-envelope thrusts (edf.thrust_v09, unmeasured: E) without / with
        "tw_E": {f"{t:g} kgf": [t * 1e3 / base["mass_g"], t * 1e3 / full["mass_g"]]
                 for t in ((L.p.get("edf") or {}).get("thrust_v09") or {}).get("design_envelope_kgf_at_22v2", [])},
        "over_target_g": {"without": base["mass_g"] - LOGIC_MASS_TARGET_G, "with": full["mass_g"] - LOGIC_MASS_TARGET_G},
        "layout": {
            "pack_box_mm": [list(L.lp_pack_x), list(L.lp_pack_y), list(L.lp_pack_z)],
            "switch_xy_mm": list(L.lp_sw_xy), "switch_r_theta": [L.lp_sw_r, L.lp_sw_theta], "switch_hole_d_mm": L.lp_sw_hole_d,
            "switch_top_z_mm": L.lp_sw_top_z, "switch_lugs_bottom_z_mm": L.lp_sw_lugs[2][0],
            "switch_lead_polyline_mm": [list(q) for q in L.lp_switch_lead_polyline],
            "pack_lead_polyline_mm": [list(q) for q in L.lp_pack_lead_polyline],
            "switch_lead_route_len_mm_E": [L.lp_switch_lead_route_mm, L.lp_switch_lead_len_mm],
            "pack_lead_route_len_mm_E": [L.lp_pack_lead_route_mm, L.lp_pack_lead_len_mm],
            "strap_z_mm": list(L.lp_strap_z), "tie_z_mm": L.lp_tie_z, "retention_E": dict(L.lp_retention_E),
            "carrier_plugs_mm": {k: [list(a) for a in v] for k, v in L.lp_plugs.items()},
            "clearance_mm": dict(L.lp_clearance), "limits_mm": dict(L.lp_limits),
            "exposure_mm": dict(L.lp_exposure), "gps_lever_shadow_deg": dict(L.lp_gps_lever_shadow_deg),
            "j15_nearest_window": list(getattr(L, "lp_j15_window", (math.nan, math.nan))),
            **({"collar": collar_record(L)} if has_col else {})},
        "note": ("pending mass decision (user 2026-11-05 '이 배터리로 진행, 질량은 나중에 결정'): flight_model = the "
                 "report totals the sim / firmware headers / mass targets consume; with_logic_power = the vehicle with "
                 "the pack, switch, lead, ties and the printed (holed) cap. Masses of the pack (vendor), switch, lead "
                 "and ties are ASSUMED; the printed cap is the slicer table's (M) when its holed slice exists"),
    }


def collar_md(c: dict, tc: dict) -> list[str]:
    """Switch-collar round (2026-11-05): the Korean section of the pending-mass report on the protective collar."""
    g, s = c["gps_sky_E"], c["section_E"]
    return [
        "## 2b. 스위치 보호 칼라 (사용자 결정 2026-11-05: \"상단 캡과 한 몸으로 출력하는 보호 칼라를 달아줘\")", "",
        f"* 상단 캡과 한 몸으로 출력하는 둥근 칼라입니다. 안지름 {c['inner_d_mm']:.1f} mm, 벽 {c['wall_t_mm']:.1f} mm "
        f"(0.4 mm 선 {c['wall']['lines_of_nozzle']:.0f}줄), 판 위 높이 {c['h_over_plate_mm']:.1f} mm(테두리 z {c['z_mm'][1]:.2f}), "
        f"손가락 홈 2개(U자 홈, 폭 {c['notch_w_mm']:.1f} mm, 둥근 바닥의 가장 낮은 곳 z {c['notch_bottom_z_mm']:.2f}, "
        f"레버가 젖혀지는 기체 y 방향). 손끝이 레버를 가운데 너머로 미는 동안 홈과의 거리 최소 "
        f"{c['finger_path_E']['min_mm']:.2f} mm(E, 기준 ≥ {c['finger_path_E']['margin_min_mm']:g}).",
        f"* 칼라 재료 {tc['collar_added_mm3']:.1f} mm³(M). 이 재료도 로직 전원 부품이라 같은 스위치 "
        "`in_flight_mass_model` 뒤에 있습니다: 비행 모델 캡은 구멍도 칼라도 없는 캡입니다(C40).",
        f"* 레버가 가장 높을 때 z {c['lever_sweep']['top_z_mm']:.2f}(A 치수로 계산) → 테두리가 "
        f"{c['z_mm'][1] - c['lever_sweep']['top_z_mm']:.2f} mm 높습니다(기준 ≥ 2.0). GPS 패치 중심에서 본 칼라 꼭대기 "
        f"{g['top_elevation_deg']:.2f}°, 폭 {g['azimuth_width_deg']:.1f}°, 수신기 기본 마스크 {g['receiver_mask_deg_A']:g}°(A) 위 하늘의 "
        f"{g['blocked_sky_above_mask_pct']:.2f} %를 가림(E, 기준 ≤ {g['limit_pct']:g} %).",
        f"* 옆 하중 손계산 단면(E): 가장 약한 높이(판 위 {s['arc_z_over_plate_mm']:.2f} mm)의 호 하나 "
        f"Z {s['arc_z_min_mm3']:.1f} mm³(팔 {s['arc_arm_mm']:.2f} mm), 판 위 전체 링 Z {s['ring_z_mm3']:.0f} mm³. "
        f"판 뿌리(E): 판을 가장 먼 받침 b {c['plate_root_E']['b_farthest_support_mm']:.1f} mm에서 고정한 원판으로 봄. "
        "판정은 검사 C41.", ""]


def write_logic_pending(report: dict, path_json: str, path_md: str) -> None:
    """2026-11-05: the pending-mass report (extra.logic_power) as JSON + a short Korean Markdown summary."""
    lp = report["extra"]["logic_power"]
    write_json({"generator": "cadquery", "params_sha256": report["params_sha256"], "preset": report["preset"],
                "config": report.get("config"), "power_source": report.get("power_source"), **lp}, path_json)
    f, w = lp["flight_model"], lp["with_logic_power"]
    d = lp["delta_full_minus_without"]
    tc = lp["top_cap"]
    ly = lp["layout"]
    pend = lp["pending_parts_g"]

    def row(nm: str, t: dict) -> str:
        c = t["com_mm"]
        i = t["inertia_diag_kg_m2"]
        return (f"| {nm} | {t['mass_g']:.2f} | ({c[0]:+.2f}, {c[1]:+.2f}, {c[2]:.2f}) | {t['lateral_offset_mm']:.2f} | "
                f"{i[0]:.5f} / {i[1]:.5f} / {i[2]:.6f} |")
    pm = (tc["printed_mass_model"] or {}).get("model")
    lines = [
        "# 로직 전원 - 질량 결정 대기 보고서 (VTOL 배터리 구성)", "",
        "**사용자 결정 4 (2026-11-05): \"이 배터리로 진행, 질량은 나중에 결정\".** "
        f"그래서 `params.json` `configs.vtol.logic_power.in_flight_mass_model` = **{str(lp['in_flight_mass_model']).lower()}** 입니다. "
        "아래 부품(스위치 보호 칼라 포함)은 CAD 고체로 모양·간섭 검사(C37-C39, C41)에만 쓰고, 시뮬레이션·펌웨어 헤더·질량 목표가 읽는 질량"
        "(보고서 parts / totals)에는 넣지 않았습니다(C40이 확인).", "",
        f"params sha256 `{report['params_sha256'][:16]}…`, 만든 명령: `python cad/cadquery/build_rocket.py --config vtol`.", "",
        "표시: M = 도구로 잰 값(CadQuery 질량 특성, 슬라이서), E = 추정, A = 가정(산 물건으로 재야 함).", "",
        "## 1. 무게·무게중심·관성 (M: CadQuery 질량 특성, 아래 부품 질량은 A)", "",
        "| 경우 | 질량 g | 무게중심 (x, y, z) mm | 옆 치우침 mm | Ixx / Iyy / Izz kg m² |", "|---|---|---|---|---|",
        row("지금 비행 모델 (시뮬레이션·펌웨어가 읽는 값)", f),
        row("로직 전원을 넣은 전체", w), "",
        f"* 차이(전체 − 지금): **{d['mass_g']:+.2f} g**, 무게중심 ({d['com_mm'][0]:+.2f}, {d['com_mm'][1]:+.2f}, "
        f"{d['com_mm'][2]:+.2f}) mm, 옆 치우침 {d['lateral_offset_mm']:+.2f} mm, 관성 Ixx {100 * d['inertia_diag_rel'][0]:+.2f} % / "
        f"Iyy {100 * d['inertia_diag_rel'][1]:+.2f} % / Izz {100 * d['inertia_diag_rel'][2]:+.2f} %.",
        f"* 질량 목표 {lp['mass_target_g']:.0f} g 대비: 지금 {lp['over_target_g']['without']:+.1f} g, 넣으면 "
        f"**{lp['over_target_g']['with']:+.1f} g**.",
        "* 옆 치우침 판정(C33과 같은 식)은 검사 보고서 `cadquery_checks_vtol.md`의 C40 줄에 지금 / 전체 두 값으로 있습니다.",
        *([("* 추력 대 무게(E, 추력은 아직 재지 않음 - `edf.thrust_v09` 설계 범위): "
            + ", ".join(f"{k} → 로직 전원 없이 {v[0]:.2f}, 넣으면 {v[1]:.2f}" for k, v in lp["tw_E"].items())
            + " (목표 1.3).")] if lp.get("tw_E") else []), "",
        "## 2. 넣지 않은 부품 (질량은 A, 위치는 CAD)", "",
        "| 부품 | g | 근거 |", "|---|---|---|",
        f"| logic_pack (1S 4200 mAh, 디바이스마트 16021596) | {pend.get('logic_pack', 0):.2f} | 판매자 무게 84 g, 자체 리드·플러그 포함(A) |",
        f"| logic_switch (MTS-102, 디바이스마트 13029107) | {pend.get('logic_switch', 0):.2f} | A (산 물건을 잴 것) |",
        f"| logic_switch_lead (스위치 → J15, {ly['switch_lead_route_len_mm_E'][1]:.0f} mm, E) | "
        f"{pend.get('logic_switch_lead', 0):.2f} | 5 g/m + 커넥터 0.5 g (A) |",
        f"| logic_retention (긴 6S 끈 + 케이블 타이) | {pend.get('logic_retention', 0):.2f} | A |",
        (f"| 상단 캡 스위치 구멍 + 보호 칼라 (출력 부품 − 비행 모델) | {tc['mass_delta_g (printed - flight model)']:+.3f} | "
         f"구멍 −{tc['hole_removed_mm3']:.1f} mm³, 칼라 +{tc['collar_added_mm3']:.1f} mm³ (M, CadQuery 고체); 출력 부품 무게 "
         f"모델 `{pm}` (출력 캡 {tc['printed_mass_g']:.2f} g, 비행 모델 캡 {tc['flight_model_mass_g']:.2f} g) |"
         if tc.get("has_collar") else
         f"| 상단 캡 스위치 구멍 (출력 부품 − 비행 모델) | {tc['mass_delta_g (printed - flight model)']:+.3f} | "
         f"부피 −{tc['hole_removed_mm3']:.1f} mm³ (M); 출력 부품 무게 모델 `{pm}` |"), "",
        *(collar_md(ly["collar"], tc) if ly.get("collar") else []),
        "## 3. 결정하면 할 일", "",
        "1. `in_flight_mass_model`을 true로 바꾸고 CAD 보고서(`build_rocket.py --config vtol`)와 검사"
        "(`check_geometry.py --config vtol`)를 다시 만듭니다.",
        "2. 크기 산정(`python sizing/vtol_sizing.py`), 강도 손계산(`python sizing/vtol_strength.py` → "
        "`docs/vtol_strength_checks.json`: 착지·다리 하중이 보고서 질량에 비례, 트레이에 1S 팩 하중 추가), 추력 대 무게·"
        "호버 전류·비행 시간, 시뮬레이션(게인·시나리오), 펌웨어 헤더(`params_gen.h`, `gains.h`, `bridge_config_gen.h`)와 "
        "HEX를 다시 만듭니다(제어가 바뀌면 전/후 비교 영상 규칙).",
        "3. 질량 목표(1,650 g)를 넘는 만큼은 사용자가 정합니다(목표를 바꾸거나 다른 곳을 줄이기).", ""]
    os.makedirs(os.path.dirname(path_md), exist_ok=True)
    with open(path_md, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines))


def vtol_extra(L: Layout, parts: dict[str, rg.Part], entries: dict[str, dict], totals: dict) -> dict:
    """v0.9 CAD-2 VTOL record: top cap stations / clearances / tether distances, the windows, fin holes and carrier, the
    footprint tip-over angles at the CG height, vane lever arm and authority (moment / inertia), and the removed material
    measured on the solids (each lightening feature: module volume without it - with it)."""
    rho = L.p["printer"]["density_g_cm3"]
    com = totals["com_mm"]
    I = totals["inertia_about_com_kg_m2"]
    h_cg = com[2] - 0.0                      # the fins stand on z = 0 (fin_te_z) - the vehicle stands on its fin TEs
    r_tip = L.fin_tip_r
    # support polygon = square with the fin tips on the axes: inradius r_tip cos 45 (worst direction, between two fins)
    tip_over = {"cg_height_mm": h_cg, "fin_tip_r_mm": r_tip,
                "angle_toward_a_fin_deg": math.degrees(math.atan2(r_tip, h_cg)),
                "angle_between_fins_deg": math.degrees(math.atan2(r_tip * math.cos(math.radians(45.0)), h_cg))}
    # v0.9 fix round (mass reviewer: 'tip-over angles ignore the lateral CG offset'): the worst direction with the CG's
    # lateral offset (support polygon = the square through the four fin tips; fin thickness ignored, conservative)
    cx, cy = com[0], com[1]
    fins = [(r_tip * math.cos(math.radians(t)), r_tip * math.sin(math.radians(t))) for t in L.fin_thetas]
    to_fin = min(math.degrees(math.atan2(r_tip - (cx * fx + cy * fy) / r_tip, h_cg)) for fx, fy in fins)
    edges = []
    for i in range(len(fins)):
        (ax, ay), (bx, by) = fins[i], fins[(i + 1) % len(fins)]
        nx, ny = (by - ay), -(bx - ax)                        # outward normal of the edge (counter-clockwise polygon)
        nn = math.hypot(nx, ny)
        edges.append(((ax * nx + ay * ny) - (cx * nx + cy * ny)) / nn)
    tip_over.update({"cg_lateral_offset_mm": math.hypot(cx, cy), "cg_xy_mm": [cx, cy],
                     "angle_toward_a_fin_with_offset_deg": to_fin,
                     "angle_between_fins_with_offset_deg": math.degrees(math.atan2(min(edges), h_cg))})
    lv = (com[2] - L.vane_hinge_z) * 1e-3
    removed = {}
    if getattr(L, "has_fin_hole", False):
        removed["fin holes (4)"] = 4 * L.fin_hole_area_mm2 * L.fin_t
    if getattr(L, "has_av_windows", False):
        removed["avionics windows (solid volume cut, mm^3)"] = sum(
            rg.intersection_volume(rg._tube(L.body_r, L.body_id / 2, L.avw_z0 - 1.0, L.avw_z1 + 1.0), c)
            for c in rg.avionics_window_cutters(L))
    if getattr(L, "has_tray_holes", False):
        (yi, yo), (z0, z1) = L.tray_hole_y, L.tray_hole_z
        removed["tray holes (2)"] = 2 * (yo - yi) * (z1 - z0) * L.tray_t
    return {
        "config": L.config, "power_source": L.power_source, "has_battery": L.has_battery,
        "top_cap": {"z_skirt_bottom": L.z_cap_skirt_bottom, "z_plate": [L.z_cap_plate_bottom, L.z_cap_top],
                    "z_under": L.z_cap_under, "rib_thetas_deg": list(L.cap_rib_thetas), "rib_t_h": [L.cap_rib_t, L.cap_rib_h],
                    "ring_r": list(L.cap_ring_r), "hub_d": L.cap_hub_d, "holes": {k: list(v) for k, v in L.cap_holes.items()},
                    "clearance_mm": dict(L.cap_clearance), "tether_mm": dict(L.cap_tether),
                    "gps_boot_shadow_deg": {"azimuth_width": L.cap_gps_boot_az_deg, "elevation_top": L.cap_gps_boot_el_deg},
                    "gps_lead_mm": L.gps_lead_len, "gps_lead_g": L.gps_lead_mass_g},
        "carrier": {"box_mm": [list(L.fcu_x), list(L.fcu_y), list(L.fcu_z)], "mass_g": L.fcu_mass_g,
                    "clearance_mm": dict(L.fcu_clearance)},
        "avionics_windows": ({"thetas_deg": list(L.avw_thetas), "w_mm": L.avw_w, "z_mm": [L.avw_z0, L.avw_z1],
                              "strut_w_mm": L.avw_strut_w, "rail_clear_deg": L.avw_rail_clear_deg,
                              "open_area_mm2": L.avw_open_area_mm2} if getattr(L, "has_av_windows", False) else None),
        "fin_hole": ({"uz_mm": [list(q) for q in L.fin_hole_uz], "area_mm2": L.fin_hole_area_mm2,
                      "frame_mm": dict(L.fin_hole_frame)} if getattr(L, "has_fin_hole", False) else None),
        "removed_solid_mm3": removed,
        "removed_solid_g_at_rho": {k: v * rho / 1000.0 for k, v in removed.items()},
        "tether_items_g": {k: m for k, (m, _) in L.tether_items.items()},
        "tether_boxes": {k: {"box_mm": [list(x), list(y), list(z)], "mass_g": m}
                         for k, (x, y, z, m) in getattr(L, "tether_boxes", {}).items()},
        "hardpoint_stack": ({"z_bolt_end": L.z_bolt_end, "z_nut": list(L.z_cap_nut), "z_washer": list(L.z_cap_washer),
                             "clearance_below_mm": dict(L.cap_stack_clearance), "washer_g": L.cap_washer_mass_g,
                             "bolt_g_ASSUMED": L.cap_bolt_mass_g, "nut_g_ASSUMED": L.cap_nut_mass_g}
                            if getattr(L, "cap_hardpoint", False) else None),
        "tip_over": tip_over,
        "vane_lever_arm_mm": lv * 1e3,
        "inertia_diag_kg_m2": [I[0][0], I[1][1], I[2][2]],
        "vehicle_h_mm": L.vehicle_h, "footprint_d_mm": L.footprint_d,
    }


# --------------------------------------------------------------------------- #
# exports
# --------------------------------------------------------------------------- #
def stl_mesh_volume(path: str) -> float:
    """Enclosed volume (mm^3) of a binary STL: the sum of the signed tetrahedra of its triangles (divergence theorem)."""
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 80)[0]
    rec = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
    v = np.frombuffer(data, dtype=rec, count=n, offset=84)["v"].astype(np.float64)
    return float(np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])).sum() / 6.0)


STL_TOLERANCES_USED: dict[str, float] = {}
STL_OUT_OF_LIMIT: dict[str, float] = {}      # v0.9 fix round: part -> relative volume error of an accepted-anyway mesh


def export_stl(parts: dict[str, rg.Part], stl_dir: str) -> dict[str, str]:
    """One binary STL per printed part name (first instance of vane / vane_horn).  v0.8: a part whose mesh volume is more
    than STL_VOLUME_REL_MAX off the exact solid is re-exported with the tolerances halved (at most STL_REFINE_MAX times;
    the v0.8 payload module's pod changes how the tube faces tessellate: +0.11 % at 0.05 mm); the tolerance used per part
    is kept in STL_TOLERANCES_USED (report extra.stl_tolerance_mm).  Parts within the limit at the base tolerance export
    exactly as before."""
    os.makedirs(stl_dir, exist_ok=True)
    written: dict[str, str] = {}
    STL_TOLERANCES_USED.clear()
    STL_OUT_OF_LIMIT.clear()
    for part in parts.values():
        if part.kind != "printed" or part.base in written:
            continue
        path = os.path.join(stl_dir, f"{part.base}.stl")
        exact = mp.precise_volume(part.shape)             # v0.9 fix round: the precise volume (Shape.Volume() was biased)
        tol, ang = STL_TOLERANCE_MM, STL_ANGULAR_TOLERANCE
        for k in range(STL_REFINE_MAX + 1):
            ok = part.shape.exportStl(path, tolerance=tol, angularTolerance=ang, ascii=False, relative=False)
            if not ok:
                raise RuntimeError(f"STL export failed for {part.base}")
            rel = stl_mesh_volume(path) / exact - 1.0
            if abs(rel) <= STL_VOLUME_REL_MAX:
                break
            if k == STL_REFINE_MAX:       # v0.9 fix round: never accept an out-of-limit mesh silently
                STL_OUT_OF_LIMIT[part.base] = rel
                print(f"WARNING: {part.base}.stl volume {100 * rel:+.3f} % off the solid after {k} refinements")
                break
            tol, ang = tol / 2.0, ang / 2.0
        STL_TOLERANCES_USED[part.base] = tol
        written[part.base] = path
    return written


def export_step(parts: dict[str, rg.Part], step_path: str, extra_parts: dict[str, rg.Part] | None = None) -> str:
    """STEP assembly, one part per component; 2026-11-05: ``extra_parts`` (the logic-power solids while their mass is
    pending) are added as parts of the assembly too (geometry, not in the report's parts)."""
    os.makedirs(os.path.dirname(step_path), exist_ok=True)
    assy = cq.Assembly(name=ASSEMBLY_NAME)
    for part in list(parts.values()) + [q for n, q in (extra_parts or {}).items() if n not in parts]:
        if part.shape is None:
            continue
        assy.add(part.shape, name=part.name, color=cq.Color(*part.color))
    assy.export(step_path, exportType="STEP")
    return step_path


def write_json(obj: dict, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)
        f.write("\n")


# --------------------------------------------------------------------------- #
def build(preset: str | None, export: bool = True, verbose: bool = True,
          write_report: bool | None = None, config: str | None = None,
          power: str | None = None) -> tuple[Layout, dict[str, rg.Part], dict]:
    """Build (and optionally export) one preset; ``write_report`` defaults to ``export`` so a
    report-only caller (check_geometry) does not overwrite the file list of a full build.  v0.9 CAD-2: ``config`` /
    ``power`` select the hardware configuration and the VTOL power source (files carry their suffix)."""
    if write_report is None:
        write_report = export
    t0 = time.time()
    L = rg.layout_for(preset) if config is None and power is None else rg.layout_for(preset, config, power)
    parts = rg.build_all(L)
    t_build = time.time() - t0
    paths = output_paths(preset, L.config, L.power_source)
    warnings: list[str] = []
    stl_files: dict[str, str] = {}
    if export:
        stl_files = export_stl(parts, paths["stl_dir"])
        export_step(parts, paths["step"], rg.logic_power_parts(L) if getattr(L, "has_logic_power", False) else None)
        warnings += [f"{b}.stl volume {100 * r:+.3f} % off the solid (accepted after {STL_REFINE_MAX} refinements)"
                     for b, r in STL_OUT_OF_LIMIT.items()]
    report = make_report(L, parts, warnings, build_time_s=t_build)
    report["extra"]["stl_files"] = {k: os.path.relpath(v, ROOT).replace(os.sep, "/") for k, v in stl_files.items()}
    report["extra"]["stl_tolerance_mm"] = dict(STL_TOLERANCES_USED) if export else None
    report["extra"]["step_file"] = os.path.relpath(paths["step"], ROOT).replace(os.sep, "/") if export else None
    report["extra"]["total_time_s"] = time.time() - t0
    if write_report:
        write_json(report, paths["report"])
        if "logic_power" in report["extra"] and not getattr(L, "lp_rocket", False):
            write_logic_pending(report, paths["logic_pending_json"], paths["logic_pending_md"])
    if verbose:
        t = report["totals"]
        print(f"preset {report['preset']}" + (f" config {L.config} power {L.power_source}" if L.vtol else "")
              + f": {len(parts)} parts built in {t_build:.1f} s, total {time.time() - t0:.1f} s")
        lp = report["extra"]["launch_pad_mass_g"]
        print(f"  mass {t['mass_g']:.1f} g (printed {t['printed_mass_g']:.1f} g"
              + (f", launch pad {lp:.1f} g excluded)" if lp is not None else ")"))
        print(f"  COM z {t['com_mm'][2]:.1f} mm, Ixx {t['inertia_about_com_kg_m2'][0][0]:.4f} "
              f"Izz {t['inertia_about_com_kg_m2'][2][2]:.5f} kg m^2")
        pe = t["post_ejection"]
        print(f"  post-ejection mass {pe['mass_g']:.1f} g, COM z {pe['com_mm'][2]:.1f} mm, descent mass {pe['descent_mass_g']:.1f} g")
        d = report["derived"]
        print(f"  pushrod {d['pushrod_len_mm']:.2f} mm, nose cavity {d['nose_cavity_volume_L']:.3f} L, "
              f"footprint {d['footprint_d_mm']:.1f} mm, height {d['vehicle_h_mm']:.1f} mm, inlet ratio {d['inlet_area_ratio']:.2f} "
              f"(measured min {report['extra']['inlet_area_ratio_measured']:.3f}, outer "
              f"{report['extra']['inlet_area_ratio_measured_outer']:.3f})")
        wg = report["extra"].get("window_gable")
        if wg:
            print(f"  window gussets {wg['gusset_volume_mm3_measured']:.1f} mm^3 measured (layout {wg['gusset_volume_mm3_layout']:.1f}), "
                  f"{wg['gusset_mass_g_measured']:.2f} g; intake {report['parts']['intake_module']['mass_g']:.2f} g")
        gx = report["extra"].get("gps")
        if gx:
            print(f"  GPS pod adds {gx['added_total_g']:.1f} g (" + ", ".join(f"{k} {v:.2f}" for k, v in gx["added_mass_g"].items())
                  + f"); antenna lever arm from the CG {gx['antenna_lever_arm_norm_mm']:.1f} mm")
        nx = report["extra"].get("gns")
        if nx:
            print(f"  GNS board adds {nx['added_total_g']:.2f} g (" + ", ".join(f"{k} {v:.2f}" for k, v in nx["added_mass_g"].items())
                  + ")" + (f"; pod mitigation saves {nx['pod_mitigation']['saved_g']:.2f} g" if nx.get("pod_mitigation") else "")
                  + f"; T/W (CAD mass) {nx['thrust_to_weight_nominal_cad']:.4f}; ESC {nx['distance_mm']['ESC (box)']:.1f} mm")
        print(f"  report -> {paths['report']}")
        if export:
            print(f"  STL x{len(stl_files)} -> {paths['stl_dir']}")
            print(f"  STEP -> {paths['step']}")
    return L, parts, report


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--preset", default=None, help="alt_presets key (default: params.json meta.preset)")
    ap.add_argument("--no-export", action="store_true", help="skip STL/STEP export (report only)")
    ap.add_argument("--config", default=None, choices=list(layout_mod.CONFIGS),
                    help="v0.9 CAD-2 hardware configuration (default: params.json mission.config)")
    ap.add_argument("--power", default=None, help="v0.9 CAD-2 VTOL power source: battery | tether | hybrid")
    args = ap.parse_args(argv)
    build(args.preset, export=not args.no_export, write_report=True, config=args.config, power=args.power)
    return 0


if __name__ == "__main__":
    sys.exit(main())
