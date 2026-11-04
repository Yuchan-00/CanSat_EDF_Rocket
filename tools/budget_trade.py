"""Budget trade study: printed mass, flight mass, payload capacity and cost of candidate configurations.

Invariant enforced: every candidate is a set of params.json overrides that must pass
``cad/layout.py`` (all geometric constraints), its printed mass comes from the same
analytic structure estimator the sizing tool uses, and every price comes from
``docs/budget/price_research_2026-10-13.json`` (with URL) or is labelled ASSUMED.

Non-printed mass: every ``params.electronics`` entry with a ``mass_g`` (v0.6.3: esc, nav - the v0.5 ``fc`` board -,
fcu, com, radio_log, misc), plus EDF, battery, five servos, recovery and hardware.  The cost column is the v0.3
trade-study list BOM_BASE (one ATmega128 board); the current bill of materials is tools/budget_bom.py
(docs/budget/bom.json).

Usage: python tools/budget_trade.py            -> table on stdout + docs/budget/trade_study.json
"""
from __future__ import annotations

import copy
import json
import math
import os
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "cad"))
sys.path.insert(0, os.path.join(ROOT, "sizing"))
import layout  # noqa: E402
import structure_estimate as se  # noqa: E402

G = 9.80665

# --------------------------------------------------------------------------- #
# propulsion options (price KRW incl. shipping; thrust = vendor/proxy table)
# --------------------------------------------------------------------------- #
EDF_HOBBYOCEAN_90_6S = {
    "model": "Hobbyocean 90mm 12-blade EDF, 6S 1450KV (AliExpress 1005007300026404)",
    "fan_d": 90.0, "housing_od": 98.0, "housing_id": 91.0, "lip_od": 112.0, "lip_t_ASSUMED": 4.0,
    "len_lip_top_to_housing_bottom_ASSUMED": 85.0, "hub_d_ASSUMED": 40.0, "mass_g": 369.0, "cells": 6,
    "voltage_nominal_V": 22.2, "thrust_nominal_N": 3.09 * G, "thrust_peak_N": 3.79 * G,
    "current_nominal_A": 102.0, "current_peak_A": 124.0, "power_nominal_W": 2264.0, "power_peak_W": 3125.0,
    "kv": 1450, "rotation": "CW (viewed from intake) - ASSUMED, verify on the unit (sets the sign of the residual swirl torque)",
    "thrust_source": "PROXY: Powerfun 90mm 6S 1450KV table (Banggood p-1612488): 3.09 kg @ 22.2 V/102 A, 3.79 kg @ 25.2 V/124 A; "
                     "Hobbyocean publishes no thrust - bench test before flight",
    "sources": ["https://ko.aliexpress.com/item/1005007300026404.html",
                "https://usa.banggood.com/Powerfun-EDF-90mm-Ducted-Fan-6S-1450KV-Brushless-Motor-12-Blades-Propeller-for-RC-Airplane-Plane-p-1612488.html"],
}

# --------------------------------------------------------------------------- #
# bill of materials (KRW, incl. shipping where known); "owned" items listed separately
# --------------------------------------------------------------------------- #
BOM_BASE = [
    ("EDF 90mm 6S 1450KV (Hobbyocean)", 83400, "https://ko.aliexpress.com/item/1005007300026404.html"),
    ("ESC Hobbywing SkyWalker 100A V2 (5V/7A BEC)", 51300, "https://ko.aliexpress.com/item/1005012523818954.html"),
    ("LiPo CNHL G+Plus 6S 2200mAh 70C", 40207 + 9372, "CNHL global store (USD 29.99 + USD 6.99 shipping)"),
    ("MG90S x5", 13932, "AliExpress 5-pack (price_research electronics)"),
    ("ATmega128 minimum-system board (check 16 MHz crystal)", 8735 + 1953, "AliExpress (price_research electronics)"),
    ("MPU-6050 GY-521", 2572, "AliExpress"),
    ("MS5611 GY-63", 4780 + 1973, "AliExpress XRY"),
    ("I2C level shifter 4ch", 344 + 1417, "AliExpress"),
    ("AMS1117 3.3 V module", 660, "Mechasolution"),
    ("OpenLog clone + microSD (card ASSUMED owned)", 3980, "AliExpress"),
    ("SBUS receiver FlySky FS-A8S V2", 9959 + 1871, "AliExpress"),
    ("NPN inverter + resistors, divider resistors (ASSUMED)", 1000, "ASSUMED"),
    ("XT60 + 12 AWG silicone wire", 1800 + 6000, "RCbank"),
    ("M3/M2 screws (M3 kit 340 pcs)", 9680, "DeviceMart"),
    ("Ripstop nylon 1 x 1.5 m x2 + cord (DIY 1.4 m chute)", 2 * 7602 + 3000, "AliExpress ripstop; cord ASSUMED"),
    ("Compression spring OD 40 (ASSUMED)", 3000, "ASSUMED"),
]
PLA_KRW_PER_KG = 18000          # Bambu KR store refill, price_research electronics
PLA_WASTE = 1.20                # supports, brim, purge; slicer gives the real number later
OPTIONAL_IF_NOT_OWNED = [
    ("FlySky FS-i6X transmitter + iA6B", 77061, "AliExpress (new-user price; regular 117,957)"),
    ("LiPo charger iMAX B6AC clone", 31400, "AliExpress"),
    ("USBasp ISP programmer", 4400, "DeviceMart"),
]


def deep_update(base: dict, over: dict) -> dict:
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_update(base[k], v)
        else:
            base[k] = copy.deepcopy(v)
    return base


REQUIRED_ELECTRONICS = ("esc", "nav", "misc")   # v0.6.3: params.electronics.fc was renamed nav (the sensor board)


def electronics_masses(el: dict) -> dict[str, float]:
    """{name: mass_g} of every params.electronics entry that carries a mass_g (esc, nav, fcu, com, radio_log, misc, ...).
    Refuses a params.json whose electronics lack one of REQUIRED_ELECTRONICS or still use the pre-v0.6.3 'fc' key."""
    if "fc" in el:
        raise KeyError("params.electronics.fc: v0.6.3 renamed it electronics.nav (+ electronics.fcu) - update the overrides")
    missing = [k for k in REQUIRED_ELECTRONICS if not isinstance(el.get(k), dict) or "mass_g" not in el[k]]
    if missing:
        raise KeyError(f"params.electronics.{'/'.join(missing)}.mass_g missing")
    return {k: float(v["mass_g"]) for k, v in el.items() if isinstance(v, dict) and "mass_g" in v}


def evaluate(name: str, overrides: dict, tw_min: float = 1.25, thrust_derate: float = 0.94) -> dict:
    p = deep_update(layout.load_params(), copy.deepcopy(overrides))
    lay = layout.Layout(p)                                         # raises on any geometric violation
    est = se.StructureEstimator(lay, p).estimate_all()
    printed = {k: v.mass_g for k, v in est.items() if v.kind == "printed" and not k.startswith("launch_pad")}
    printed_g = sum(printed.values())
    el = p["electronics"]
    rec = p["recovery"]
    hw = {k: float(v) for k, v in p["hardware_mass_g"].items() if isinstance(v, (int, float))}   # v0.9: the dict has a note
    if getattr(lay, "direct_drive", False):            # v0.9: the linkage lump becomes the cover / coupler screws
        hw.pop("pushrods_horns_clevises", None)
        hw["vane_drive_hardware"] = float(p["vane_drive"]["hardware_g_ASSUMED"])
    if getattr(lay, "has_motor_leads", False):         # v0.9: 3 x 12 AWG + bullets on the Layout route (ASSUMED)
        hw["motor_leads"] = float(lay.ml_mass_g)
    non_printed = {
        "edf": p["edf"]["mass_g"], "battery": p["battery"]["mass_g"], "servos": 5 * p["servo"]["mass_g"],
        # every electronics item params.json gives a mass: esc, nav (v0.6.3, the v0.5 'fc' board), fcu (v0.6.3), com
        # (v0.6), radio_log (v0.6), misc - so a new or renamed board is counted, never silently dropped
        **electronics_masses(el),
        "chute_pack": rec["chute_mass_g"] + rec["lines_swivel_mass_g"] + rec["shock_cord_mass_g"],
        "hardware": sum(hw.values()),
    }
    dry_g = printed_g + sum(non_printed.values())
    thrust_n = p["edf"]["thrust_nominal_N"] * thrust_derate * p["mission"]["climb_throttle"] ** p["edf"]["throttle_exponent"]
    max_total_g = thrust_n / (tw_min * G) * 1000.0
    payload_max_g = max_total_g - dry_g
    pla_kg = printed_g / 1000.0 * PLA_WASTE
    cost = sum(c for _, c, _ in BOM_BASE) + PLA_KRW_PER_KG * math.ceil(pla_kg)
    return {"name": name, "printed_g": round(printed_g), "printed_by_part": {k: round(v) for k, v in printed.items()},
            "non_printed_g": {k: round(v) for k, v in non_printed.items()}, "dry_g": round(dry_g),
            "design_thrust_N": round(thrust_n, 2), "tw_min": tw_min, "max_total_g": round(max_total_g),
            "payload_max_g": round(payload_max_g), "pla_kg_with_waste": round(pla_kg, 2),
            "cost_krw": cost, "vehicle_h_mm": round(lay.vehicle_h), "body_od": lay.body_od}


BUDGET_COMMON = {
    "edf": EDF_HOBBYOCEAN_90_6S,
    "propulsion_module": {"duct_id": 90.0},
    "battery": {"config": "CNHL G+Plus 6S 2200mAh 70C (single pack)", "cells": 6, "capacity_mAh": 2200, "c_rating": 70,
                "mass_g": 349.0, "dims_lwh": [109.0, 35.0, 51.0], "min_cell_v": 3.3},
    "electronics": {"esc": {"model": "Hobbywing SkyWalker 100A V2 (100 A cont / 120 A peak, 5V/7A BEC)", "mass_g": 92.0,
                            "dims_lwh": [85.0, 36.0, 9.0]},
                    # v0.6: 60 -> 40 g, the LoRa module / OpenLog / radio level shifter (20 g) are electronics.radio_log
                    "misc": {"mass_g": 40.0}},
    "recovery": {"vehicle_chute_d_m": 1.4, "chute_mass_g": 70.0, "lines_swivel_mass_g": 15.0, "shock_cord_mass_g": 15.0},
}


def candidates() -> list[tuple[str, dict]]:
    out = []
    for wall in (2.0, 1.6, 1.2):
        for env_h, pay in ((200.0, 400.0), (150.0, 350.0)):
            o = copy.deepcopy(BUDGET_COMMON)
            deep_update(o, {"body": {"wall": wall, "spigot_wall": max(1.2, wall)},
                            "propulsion_module": {"cone_wall": wall, "sleeve_wall": max(1.6, wall)},
                            "nose": {"wall": wall},
                            "payload": {"envelope_h": env_h, "mass_g": pay}})
            out.append((f"wall {wall} / envelope h {env_h:.0f}", o))
    return out


def main() -> int:
    rows = []
    for name, o in candidates():
        try:
            rows.append(evaluate(name, o))
        except Exception as exc:  # noqa: BLE001 - report infeasible geometry as a row
            rows.append({"name": name, "error": repr(exc)})
    for r in rows:
        if "error" in r:
            print(f"{r['name']:<32} INFEASIBLE: {r['error']}")
            continue
        print(f"{r['name']:<32} printed {r['printed_g']:5d} g  dry {r['dry_g']:5d} g  thrust {r['design_thrust_N']:5.1f} N  "
              f"max total {r['max_total_g']:5d} g  -> payload max {r['payload_max_g']:5d} g   cost {r['cost_krw']:,} KRW  h {r['vehicle_h_mm']} mm")
    os.makedirs(os.path.join(ROOT, "docs", "budget"), exist_ok=True)
    with open(os.path.join(ROOT, "docs", "budget", "trade_study.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump({"rows": rows, "bom": BOM_BASE, "optional_if_not_owned": OPTIONAL_IF_NOT_OWNED,
                   "_note": "cost_krw = the v0.3 trade-study list 'bom' (one ATmega128 board); the v0.6.3 bill of "
                            "materials is docs/budget/bom.json (tools/budget_bom.py). non_printed_g counts every "
                            "params.electronics entry with a mass_g."}, f, indent=1, ensure_ascii=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
