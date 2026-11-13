"""Probe the installed KiCad 10 libraries for every symbol / footprint the carrier spec intends to use.

Invariant: nothing here is typed from memory - every pin number / name and every pad number is read from the
library file on disk through ai_eda.tools.kicad.library (the AI_EDA engine's only library reader). Output:
spec/library_probe.json = {"symbols": {lib:name: {path, pins: [[number, name, type, unit], ...]}},
"footprints": {lib:name: {path, pads: [number, ...], pad_types: {number: type}, attributes: [...]}}, "missing": [...]}.
Deterministic (sorted, no timestamps).

rev 3: footprints also carry "pad_types" ({pad number: smd | thru_hole | ...}) and "attributes" (smd /
through_hole / exclude_from_bom ...), which the fit-set bookkeeping uses to tell SMT from THT joints.

Two modes
  probe (default)  write spec/library_probe.json (input of build_spec.py)
  gate             the rev 3 library gate, run AFTER build_spec.py: every symbol / footprint named in
                   spec/carrier_spec.json is loaded again from disk, and every pin the spec uses in a net or a
                   no-connect must be a pin NUMBER of the symbol and a pad NUMBER of the footprint. A symbol pin
                   without a pad, or a symbol pin that is in no net and not a declared no-connect, is an error;
                   pads without a symbol pin are listed. Report: out/reports/library_gate.json. Exit 1 on failure.

Run (cwd = scripts/):  B:/Claude/.venv/Scripts/python.exe -X utf8 -B probe_library.py [gate]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, r"B:\Claude")

from ai_eda.ir import LibraryRef  # noqa: E402
from ai_eda.tools.kicad.library import KicadLibrary, LibraryLookupError  # noqa: E402

HERE = Path(__file__).resolve().parent.parent

SYMBOLS = [
    "MCU_Microchip_ATmega:ATmega128A-A",
    "Regulator_Linear:AMS1117-3.3",
    "Transistor_FET:BSS138",
    "Transistor_FET:AO3401A",
    "Transistor_BJT:MMBT3904",
    "Device:Crystal_GND24",
    "Device:R",
    "Device:C",
    "Device:C_Polarized",
    "Device:LED",
    "Device:NetTie_2",
    "Device:D_Schottky",
    "Device:D_Zener",
    "Connector:AVR-ISP-6",
    "Connector_Generic:Conn_01x01",  # rev 3.1: J13 release-signal pad
    "Connector_Generic:Conn_01x02",
    "Connector_Generic:Conn_01x03",
    "Connector_Generic:Conn_01x04",
    "Connector_Generic:Conn_01x06",
    "Connector_Generic:Conn_01x08",
    "Connector_Generic:Conn_01x12",
    "Connector_Generic:Conn_02x03_Odd_Even",
    "Connector_Generic:Conn_02x03_Counter_Clockwise",   # rev 3: U8 TPS61023 (SOT-563: pads 1-3 left, 4-6 right)
    "Comparator:MCP65R41",                              # rev 3: U9 TLV3012B (pin table checked in rev3_checks.py)
    "Jumper:SolderJumper_2_Open",
    "Jumper:SolderJumper_3_Bridged12",
    "Device:L",
    "Connector_Generic_MountingPin:Conn_01x04_MountingPin",
    "Mechanical:MountingHole",
    "power:+5V",
    "power:+3V3",
    "power:GND",
    "power:PWR_FLAG",
    "RF_Module:ESP-12F",                                # rev 3: U7 (the footprint stays RF_Module:ESP-12E)
]

FOOTPRINTS = [
    "Package_QFP:TQFP-64_14x14mm_P0.8mm",
    "Package_TO_SOT_SMD:SOT-223-3_TabPin2",
    "Package_TO_SOT_SMD:SOT-23",
    "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
    "Resistor_SMD:R_0603_1608Metric",
    "Capacitor_SMD:C_0603_1608Metric",
    "Capacitor_SMD:C_0805_2012Metric",
    "Capacitor_SMD:C_1206_3216Metric",
    "Capacitor_SMD:CP_Elec_6.3x5.8",
    "Capacitor_SMD:CP_Elec_6.3x5.4",
    "LED_SMD:LED_0603_1608Metric",
    "Diode_SMD:D_SOD-123",
    "NetTie:NetTie-2_SMD_Pad0.5mm",
    "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm",
    "MountingHole:MountingHole_3.2mm_M3",
    "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal",
    "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm",  # rev 3.1: J13 release-signal pad
    "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal",
    "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal",
    "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Horizontal",
    "Inductor_SMD:L_0805_2012Metric",
    "Jumper:SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm",
    "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical",
    "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical",
    "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical",
    "Connector_PinSocket_1.27mm:PinSocket_1x12_P1.27mm_Vertical",
    "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal",
    "Connector_JST:JST_PH_S2B-PH-K_1x02_P2.00mm_Horizontal",
    "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical",
    "Connector_JST:JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical",
    "Capacitor_SMD:C_0402_1005Metric",
    "Capacitor_Tantalum_SMD:CP_EIA-3216-18_Kemet-A",
    "Capacitor_Tantalum_SMD:CP_EIA-3528-21_Kemet-B",
    "RF_Module:ESP-12E",
    "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical",
    # rev 3 power block
    "Package_TO_SOT_SMD:SOT-563",
    "Package_TO_SOT_SMD:SOT-23-6",
    "Inductor_SMD:L_Changjiang_FXL0420",
    # rev 3 PROG_HEADERS = 'B' (JLC-fitted programming headers outside the tie band)
    "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical_SMD",
    "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical_SMD_Pin1Left",
    "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical_SMD_Pin1Right",
]


def _num_key(s: str) -> tuple[int, str]:
    return (int(s) if s.isdigit() else 10_000, s)


def probe() -> int:
    lib = KicadLibrary()
    out: dict = {"symbols": {}, "footprints": {}, "missing": []}
    for lid in SYMBOLS:
        library, name = lid.split(":", 1)
        try:
            sd = lib.load_symbol(LibraryRef(library=library, name=name))
        except LibraryLookupError as exc:
            out["missing"].append(f"symbol {lid}: {exc}")
            continue
        pins = sorted({(p.number, p.name, p.electrical_type, p.unit) for p in sd.pins},
                      key=lambda t: _num_key(t[0]))
        out["symbols"][lid] = {
            "path": str(sd.library_path),
            "footprint_property": sd.properties.get("Footprint", ""),
            "pins": [list(p) for p in pins],
        }
    for lid in FOOTPRINTS:
        library, name = lid.split(":", 1)
        try:
            fd = lib.load_footprint(LibraryRef(library=library, name=name))
        except LibraryLookupError as exc:
            out["missing"].append(f"footprint {lid}: {exc}")
            continue
        pads = sorted({p.number for p in fd.pads if p.number}, key=_num_key)
        types: dict[str, set[str]] = {}
        for p in fd.pads:
            if p.number:
                types.setdefault(p.number, set()).add(str(p.pad_type))
        out["footprints"][lid] = {"path": str(lib.footprint_file(library, name)), "pads": pads,
                                  "pad_types": {n: "+".join(sorted(types[n])) for n in pads},
                                  "attributes": sorted(str(a) for a in (fd.attributes or []))}
    dst = HERE / "spec" / "library_probe.json"
    dst.write_text(json.dumps(out, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(f"symbols {len(out['symbols'])}, footprints {len(out['footprints'])}, missing {len(out['missing'])}")
    for m in out["missing"]:
        print("MISSING", m)
    return 1 if out["missing"] else 0


def pin_number(member: str) -> tuple[str, str]:
    """Spec member -> (refdes, pin number): "U7.VCC(8)" -> ("U7", "8"); "R1.2" -> ("R1", "2")."""
    ref, pin = member.split(".", 1)
    if pin.endswith(")") and "(" in pin:
        pin = pin[pin.rindex("(") + 1:-1]
    return ref, pin


def gate() -> int:
    lib = KicadLibrary()
    spec = json.loads((HERE / "spec" / "carrier_spec.json").read_text(encoding="utf-8"))
    used: dict[str, set[str]] = {}
    for n in spec["nets"]:
        for m in n["members"]:
            ref, pin = pin_number(m)
            used.setdefault(ref, set()).add(pin)
    # no-connects are declared by pin NAME or number ("U7.ADC", "J7.MP"); resolved below against the library symbol
    nc_decl: dict[str, list[str]] = {}
    for n in spec["no_connect"]:
        ref, _, pin = (n["pin"] if isinstance(n, dict) else n).partition(".")
        nc_decl.setdefault(ref, []).append(pin)
        used.setdefault(ref, set())
    errors: list[str] = []
    notes: list[str] = []
    sym_cache: dict[str, set[str]] = {}
    fp_cache: dict[str, set[str]] = {}
    name_cache: dict[str, dict[str, set[str]]] = {}
    rows = []
    for part in spec["parts"]:
        ref = part["refdes"]
        try:
            if part["symbol"] not in sym_cache:
                library, name = part["symbol"].split(":", 1)
                sym_pins = lib.load_symbol(LibraryRef(library=library, name=name)).pins
                sym_cache[part["symbol"]] = {p.number for p in sym_pins}
                name_cache[part["symbol"]] = {}
                for p in sym_pins:
                    name_cache[part["symbol"]].setdefault(p.name, set()).add(p.number)
            if part["footprint"] not in fp_cache:
                library, name = part["footprint"].split(":", 1)
                fp_cache[part["footprint"]] = {p.number for p in
                                               lib.load_footprint(LibraryRef(library=library, name=name)).pads
                                               if p.number}
        except LibraryLookupError as exc:
            errors.append(f"{ref}: {exc}")
            continue
        pins, pads = sym_cache[part["symbol"]], fp_cache[part["footprint"]]
        u = used.setdefault(ref, set())
        for decl in nc_decl.get(ref, []):
            nums = name_cache[part["symbol"]].get(decl) or ({decl} if decl in pins else set())
            if not nums:
                errors.append(f"{ref}: no-connect {decl} is neither a pin name nor a pin number of {part['symbol']}")
            for num in nums:
                if num in u:
                    errors.append(f"{ref}: pin {num} ({decl}) is in a net and declared no-connect")
                u.add(num)
        for pin in sorted(u - pins, key=_num_key):
            errors.append(f"{ref}: used pin {pin} is not a pin number of {part['symbol']}")
        for pin in sorted(u - pads, key=_num_key):
            errors.append(f"{ref}: used pin {pin} is not a pad number of {part['footprint']}")
        for pin in sorted(pins - pads, key=_num_key):
            errors.append(f"{ref}: symbol pin {pin} of {part['symbol']} has no pad on {part['footprint']}")
        for pin in sorted(pins - u, key=_num_key):
            errors.append(f"{ref}: symbol pin {pin} is neither in a net nor a declared no-connect")
        extra = sorted(pads - pins, key=_num_key)
        if extra:
            notes.append(f"{ref}: pads without a symbol pin on {part['footprint']}: {', '.join(extra)}")
        rows.append({"refdes": ref, "symbol": part["symbol"], "footprint": part["footprint"],
                     "used_pins": len(u), "symbol_pins": len(pins), "pads": len(pads)})
    for ref in sorted(set(used) - {p["refdes"] for p in spec["parts"]}):
        errors.append(f"{ref}: used in a net / no-connect but not a part")
    rep = {
        "check": "library gate (rev 3)",
        "label": "M - read from the KiCad library files on disk by ai_eda.tools.kicad.library",
        "spec": "spec/carrier_spec.json",
        "prog_headers": spec.get("prog_headers"),
        "parts": len(rows),
        "symbols_used": sorted(sym_cache),
        "footprints_used": sorted(fp_cache),
        "symbols_used_count": len(sym_cache),
        "footprints_used_count": len(fp_cache),
        "used_pins_total": sum(r["used_pins"] for r in rows),
        "errors": errors,
        "notes": notes,
        "status": "PASS" if not errors else "FAIL",
    }
    dst = HERE / "out" / "reports" / "library_gate.json"
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(json.dumps(rep, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(f"library gate: {rep['status']} - {len(rows)} parts, {len(sym_cache)} symbols, {len(fp_cache)} footprints, "
          f"{rep['used_pins_total']} used pins, {len(errors)} errors, {len(notes)} notes")
    for e in errors:
        print("ERROR", e)
    return 1 if errors else 0


def main() -> int:
    mode = sys.argv[1] if len(sys.argv) > 1 else "probe"
    if mode == "probe":
        return probe()
    if mode == "gate":
        return gate()
    print("usage: probe_library.py [probe|gate]")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
