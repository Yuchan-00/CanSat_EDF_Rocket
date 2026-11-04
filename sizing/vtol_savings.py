"""v0.9 CAD-2: attribute the VTOL mass saving to each step (architect: "each saving listed in grams").

Method (measured on the CadQuery solids, converted with the slicer): for every printed geometry step the module is built
twice - the VTOL configuration with and without that one override (params deep copy, everything else equal) - and the
solid-volume difference is converted to printed grams with that module's effective density in the VTOL slicer table
(part grams / solid volume of the exact sliced solid).  Steps that remove whole parts / placeholders are read from the
CAD reports (rocket vs VTOL battery).  The sum of the steps is compared with the reports' total difference; the residual
(interaction between steps, e.g. the windows on the shorter tube) is reported, not hidden.

Output: docs/vtol_savings.json.  Usage: python sizing/vtol_savings.py
"""
from __future__ import annotations

import copy
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
for _p in (os.path.join(ROOT, "cad"), os.path.join(ROOT, "cad", "cadquery")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import layout  # noqa: E402
import massprops as mp  # noqa: E402
import rocket_geometry as rg  # noqa: E402


def rep(path: str) -> dict:
    with open(os.path.join(ROOT, path), "r", encoding="utf-8") as f:
        return json.load(f)


def volume(p: dict, builder) -> float:
    # v0.9 fix round: the precise (adaptive) OCC volume - Shape.Volume() under-reported the propulsion module by ~2150 mm^3
    # and overstated the duct / fairing steps by ~5 g (geometry reviewer)
    return mp.precise_volume(builder(layout.Layout(p)))


def main() -> int:
    pv = layout.load_params(config="vtol")
    rv, rr = rep("cad/exports/cadquery_report_vtol.json"), rep("cad/exports/cadquery_report.json")
    table = rep("cad/exports/print/provisional_v09_cadquery_vtol/slicer_part_masses.json")["parts"]
    rho_eff = {k: v["part_g"] * 1000.0 / v["solid_volume_mm3"] for k, v in table.items()}   # g/cm3 -> g per 1000 mm3

    def step(label: str, module: str, builder, edit, base=None) -> dict:
        """saving of one override: the module built with it and without it; base edits both sides first (so a
        step measured on a window-less tube does not count the windows again)."""
        p1 = copy.deepcopy(pv)
        if base:
            base(p1)
        p0 = copy.deepcopy(p1)
        edit(p0)
        v_with, v_without = volume(p1, builder), volume(p0, builder)
        d_g = (v_without - v_with) * rho_eff[module] / 1000.0
        return {"step": label, "module": module, "removed_solid_mm3": v_without - v_with, "saving_g": d_g}

    rocket_am = rr["parts"]["avionics_module"]
    steps = [
        step("duct wall 2.4 -> 1.6 mm (vane_section_od 94.8 -> 93.2)", "propulsion_module", rg.propulsion_module,
             lambda p: p["propulsion_module"].update(vane_section_od=94.8),
             base=lambda p: p["fin"].update(span=76.0)),     # (fix round: at the rocket's span - 94.8 + 76.8 overruns the bed)
        step("fairing wall / floor / top 1.6 -> 1.2 mm", "propulsion_module", rg.propulsion_module,
             lambda p: p["vane_drive"]["fairing"].update(wall=1.6, floor_t=1.6, top_t=1.6)),
        # v0.9 fix round: no fin holes any more (leg strength); the VTOL fin span 76 -> 76.8 keeps the 246.8 mm footprint
        step("fin span 76 -> 76.8 mm (footprint 246.8; fix round, a cost)", "propulsion_module", rg.propulsion_module,
             lambda p: p["fin"].update(span=76.0)),
        step("EDF section 100 -> 92 mm", "propulsion_module", rg.propulsion_module,
             lambda p: p["propulsion_module"].update(edf_section_min_h=100.0)),
        step("intake bulkhead 2.0 -> 1.2 mm", "intake_module", rg.intake_module,
             lambda p: p["intake_module"].update(bulkhead_t=2.0)),
        step("avionics windows (14)", "avionics_module", rg.avionics_module,
             lambda p: p["avionics_module"].update(windows=None)),
        step("avionics tube 200 -> 150 mm (+ tray 160 -> 112, battery / ESC at the tray bottom)", "avionics_module",
             rg.avionics_module, lambda p: p["avionics_module"].update(tube_h=200.0, tray_h=160.0),
             base=lambda p: p["avionics_module"].update(windows=None)),
        step("tray lightening holes", "avionics_tray", rg.avionics_tray, lambda p: p["avionics_module"].update(tray_holes=None)),
        step("tray 160 -> 112 mm, no NAV holes", "avionics_tray", rg.avionics_tray,
             lambda p: p["avionics_module"].update(tray_h=160.0, tube_h=200.0, nav_holes=True),
             base=lambda p: p["avionics_module"].update(tray_holes=None)),
        # v0.9 fix round: the tray plane moved 25 -> 14.6 (lateral CG) and the tray grew 100 -> 107.5 wide to reach the rails
        step("tray 100 -> 107.5 wide at the new tray plane (fix round, a cost)", "avionics_tray", rg.avionics_tray,
             lambda p: p["avionics_module"].update(tray_w=100.0, tray_plane_x=25.0),
             base=lambda p: p["avionics_module"].update(tray_holes=None)),
    ]
    # the shorter-tube step is measured on a window-less tube, so subtract the windows' share only once
    av_short = steps[6]
    av_short["note"] = "measured without windows on both sides (the windows are their own step)"
    steps[8]["note"] = "measured without the lightening holes on both sides (their own step); the VTOL tray has no NAV holes"
    rocket_vs = {n: rr["parts"].get(n, {}).get("mass_g", 0.0) for n in set(rr["parts"]) | set(rv["parts"])}
    vtol_vs = {n: rv["parts"].get(n, {}).get("mass_g", 0.0) for n in rocket_vs}
    removed = {n: rocket_vs[n] - vtol_vs[n] for n in rocket_vs if abs(rocket_vs[n] - vtol_vs[n]) > 1e-9
               and n not in ("propulsion_module", "intake_module", "avionics_module", "avionics_tray")}
    groups = {
        "payload chain (decision 2: CanSat, chute pack, payload module, piston, spring, release servo, latch arm)":
            ["cansat_payload", "chute_pack", "payload_module", "piston", "spring", "servo_release", "latch_arm"],
        "nose cone + GPS pod lid -> top cap (architect a)": ["nose_cone", "gps_pod_lid", "top_cap"],
        "four boards + radio box -> one carrier PCB (architect e, ASSUMED masses)": ["nav", "fcu", "com", "gns", "radio_log",
                                                                                     "carrier"],
        "misc electronics 40 -> 25 g (UBEC / buzzer / wiring; ASSUMED)": ["misc_electronics"],
        "GPS ground plane 50 -> 40 mm and the shorter GPS lead (architect f)": ["gps_ground_plane", "gps_lead"],
        "servo / motor leads (architect f: the stack is shorter above the bulkhead, the leads end there - ~0)":
            ["servo_leads", "motor_leads"],
    }
    whole = {g: {"parts": {n: removed.get(n, 0.0) for n in ns}, "saving_g": sum(removed.get(n, 0.0) for n in ns)}
             for g, ns in groups.items()}
    other = {n: v for n, v in removed.items() if not any(n in ns for ns in groups.values())}
    printed_mod = {m: rr["parts"][m]["mass_g"] - rv["parts"][m]["mass_g"]
                   for m in ("propulsion_module", "intake_module", "avionics_module", "avionics_tray")}
    by_step_mod: dict[str, float] = {}
    for s in steps:
        by_step_mod[s["module"]] = by_step_mod.get(s["module"], 0.0) + s["saving_g"]
    out = {"rocket_mass_g": rr["totals"]["mass_g"], "vtol_battery_mass_g": rv["totals"]["mass_g"],
           "total_saving_g": rr["totals"]["mass_g"] - rv["totals"]["mass_g"],
           "geometry_steps": steps, "whole_part_groups": whole, "other_part_changes_g": other,
           "printed_module_change_g_reports": printed_mod, "printed_module_change_g_steps": by_step_mod,
           "printed_module_residual_g": {m: printed_mod[m] - by_step_mod.get(m, 0.0) for m in printed_mod},
           "method": __doc__.split("\n\n")[1].replace("\n", " ")}
    with open(os.path.join(ROOT, "docs", "vtol_savings.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
        f.write("\n")
    print(f"rocket {out['rocket_mass_g']:.1f} g -> VTOL {out['vtol_battery_mass_g']:.1f} g (-{out['total_saving_g']:.1f} g)")
    for s in steps:
        print(f"  {s['saving_g']:7.1f} g  {s['step']}")
    for g, v in whole.items():
        print(f"  {v['saving_g']:7.1f} g  {g}")
    print("  other:", {k: round(v, 2) for k, v in other.items()})
    print("  module residual (reports - steps):", {k: round(v, 2) for k, v in out["printed_module_residual_g"].items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
