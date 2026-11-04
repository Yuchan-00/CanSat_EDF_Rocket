"""Write the budget configuration into params.json.

Invariant enforced: the budget design is one set of overrides (BUDGET below); the
previous full design stays reproducible as ``alt_presets["KAIST_90mm_12S"]``, whose
values are copied from params.json before it is changed (never typed by hand).

    python tools/make_budget_preset.py --as-alt      # add BUDGET_90mm_6S as an alt preset only
    python tools/make_budget_preset.py --make-default # budget becomes the default, full design -> alt preset
    python tools/make_budget_preset.py --check        # read only: does params.json hold these overrides? (0 yes, 1 no)

v0.6.3: the overrides use electronics.nav (the v0.5 electronics.fc board) and the v0.6 misc (40 g, the radio parts in
electronics.radio_log); electronics.fcu / .com / .radio_log are base keys shared by both presets.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
PARAMS = os.path.join(ROOT, "params.json")
sys.path.insert(0, os.path.join(ROOT, "tools"))
import budget_trade as bt  # noqa: E402

BUDGET_NAME = "BUDGET_90mm_6S"
FULL_NAME = "KAIST_90mm_12S"

LIGHT = {
    "body": {"wall": 1.2, "spigot_wall": 1.6},
    "propulsion_module": {"cone_wall": 1.2, "sleeve_wall": 1.6, "web_t": 1.6, "vane_section_od": 94.8,
                          "edf_section_min_h": 100.0},
    "fin": {"t": 2.4},
    "nose": {"wall": 1.2},
    "avionics_module": {"tube_h": 200.0, "tray_h": 160.0, "tray_t": 2.0, "rail_gap": 2.4},
    "payload_module": {"piston_t": 2.4, "bulkhead_t": 2.0, "piston_recess_depth": 1.0},
    "intake_module": {"bulkhead_t": 2.0, "shelf_t": 2.4},
}
# static margin (coast phase): the light vehicle's CG dropped to 0.14 cal; these restore ~0.54 cal (same as v0.2)
STABILITY = {
    "fin": {"span": 76.0, "tip_chord": 60.0},                 # tip r 123.4 -> footprint 246.8 <= 248 bed
    "avionics_module": {"battery_above_misc": True},           # battery on top of the tray: CG up
    "nose": {"length": 140.0},                                 # nose CP lower; cavity 0.66 L >= 0.4 L
    "intake_module": {"window_h": 117.0},                      # +50 mm between EDF and avionics: CG up
    "recovery": {"chute_pack_placeholder_d": 70.0, "chute_pack_placeholder_h": 60.0,
                 "chute_pack_volume_L_required": 0.4},
}
PAYLOAD = {
    "standard": "International CanSat class: can D66 x 115 mm, <= 350 g (budget design). The KAIST classes "
                "(650-1000 g, D110 x 200 mm envelope) need the full preset KAIST_90mm_12S.",
    "envelope_d": 110.0, "envelope_h": 150.0, "mass_g": 350.0, "can_d": 66.0, "can_h": 115.0, "chute_pack_h": 35.0,
}
EXTRA = {
    # v0.6.3: the v0.5 'fc' board is electronics.nav (the sensor computer, same board and spot); electronics.fcu, .com
    # and .radio_log are base keys that both presets share, so the budget preset does not override them.  v0.6: misc
    # 60 -> 40 g, its LoRa / OpenLog parts moved to electronics.radio_log.
    "electronics": {
        "nav": {"model": "NAV board: ATmega128A minimum-system board (16 MHz crystal) + GY-521 MPU-6050 + GY-63 MS5611 "
                         "on a 4-ch level shifter (3.3 V side) - v0.6.3: the sensor computer, at the v0.5 flight-computer "
                         "spot", "mass_g": 40.0, "dims_lwh": [50.0, 50.0, 15.0]},
        "misc": {"items": "5 V logic UBEC, buzzer, arming switch, wiring (servos run from the ESC's 5 V / 7 A BEC); v0.6: "
                          "the LoRa module, OpenLog and radio level shifter moved to electronics.radio_log",
                 "mass_g": 40.0, "dims_lwh": [30.0, 40.0, 20.0]},
    },
    "mission": {"budget_note": "100 A ESC (120 A peak 10 s): full throttle on a fresh pack draws ~114 A for the first "
                               "seconds (proxy table 124 A @ 25.2 V); bench-measure current before flight"},
}


def budget_overrides() -> dict:
    o = copy.deepcopy(bt.BUDGET_COMMON)
    bt.deep_update(o, copy.deepcopy(LIGHT))
    bt.deep_update(o, {"payload": copy.deepcopy(PAYLOAD)})
    bt.deep_update(o, copy.deepcopy(EXTRA))
    bt.deep_update(o, copy.deepcopy(STABILITY))
    return o


def reverse_overrides(base: dict, over: dict) -> dict:
    """Values of ``base`` at every leaf path of ``over`` (missing keys -> None marker skipped)."""
    out: dict = {}
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            sub = reverse_overrides(base[k], v)
            if sub:
                out[k] = sub
        elif k in base:
            out[k] = copy.deepcopy(base[k])
        elif isinstance(base, dict):
            out[k] = None            # key did not exist in the full design
    return out


def strip_none(d: dict) -> dict:
    return {k: (strip_none(v) if isinstance(v, dict) else v) for k, v in d.items() if v is not None}


def override_diffs(base: dict, over: dict, path: str = "") -> list[str]:
    """Every leaf of ``over`` whose value ``base`` does not have (missing key or other value), as 'a.b.c: ...' lines."""
    out: list[str] = []
    for k, v in over.items():
        here = f"{path}.{k}" if path else k
        if isinstance(v, dict):
            if not isinstance(base.get(k), dict):
                out.append(f"{here}: missing in params.json")
            else:
                out += override_diffs(base[k], v, here)
        elif k not in base:
            out.append(f"{here}: missing in params.json (budget value {v!r})")
        elif base[k] != v:
            out.append(f"{here}: budget value {v!r}, params.json {base[k]!r}")
    return out


def check(p: dict) -> int:
    """--check (read only): does params.json hold the budget design this file defines?  Compared with the top level
    when the budget is the default preset, else with alt_presets[BUDGET_NAME]."""
    over = budget_overrides()
    if p["meta"].get("preset") == BUDGET_NAME:
        where, base = "params.json (default preset)", p
    else:
        where, base = f"params.json alt_presets.{BUDGET_NAME}", p.get("alt_presets", {}).get(BUDGET_NAME, {})
    diffs = override_diffs(base, over)
    if diffs:
        print(f"DIFFERS: {len(diffs)} budget override(s) not in {where}:")
        for d in diffs:
            print(f"  - {d}")
        return 1
    def leaves(d: dict) -> int:
        return sum(leaves(v) if isinstance(v, dict) else 1 for v in d.values())

    print(f"OK: {where} holds every one of the {leaves(over)} budget override values")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--as-alt", action="store_true")
    g.add_argument("--make-default", action="store_true")
    g.add_argument("--check", action="store_true", help="read only: compare the budget overrides with params.json")
    a = ap.parse_args(argv)
    with open(PARAMS, "r", encoding="utf-8") as f:
        p = json.load(f)
    if a.check:
        return check(p)
    over = budget_overrides()
    if a.as_alt:
        p.setdefault("alt_presets", {})[BUDGET_NAME] = over
    else:
        if p["meta"].get("preset") == BUDGET_NAME:
            print("budget is already the default")
            return 0
        full = strip_none(reverse_overrides(p, over))
        # keys introduced by the budget design need explicit full-design values (else they would be inherited)
        full.setdefault("avionics_module", {})["battery_above_misc"] = False
        full.setdefault("propulsion_module", {})["edf_section_min_h"] = 125.0
        full["note"] = ("Full design v0.2 (KAIST D110 x 200 mm / 1 kg payload, JP Hobby 90 mm 12S EDF, 160 A HV ESC, "
                        "2 x 6S 2200, 12S UBECs). Not priced in docs/budget; its 12S propulsion and power parts alone "
                        "exceed the 200-300k KRW budget.")
        alt = {k: v for k, v in p.get("alt_presets", {}).items() if k != BUDGET_NAME}
        bt.deep_update(p, over)
        # keys that exist only in the budget design are removed by the full preset via explicit values
        alt[FULL_NAME] = full
        p["alt_presets"] = alt
        p["meta"]["preset"] = BUDGET_NAME
        p["meta"]["version"] = "0.3.0"
    with open(PARAMS, "w", encoding="utf-8", newline="\n") as f:
        json.dump(p, f, ensure_ascii=False, indent=2)
    print("params.json updated:", "alt preset" if a.as_alt else "budget is now the default; full design ->", FULL_NAME)
    return 0


if __name__ == "__main__":
    sys.exit(main())
