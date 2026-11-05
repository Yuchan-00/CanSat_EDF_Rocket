"""Probe the installed KiCad 10 libraries for every symbol / footprint the carrier spec intends to use.

Invariant: nothing here is typed from memory - every pin number / name and every pad number is read from the
library file on disk through ai_eda.tools.kicad.library (the AI_EDA engine's only library reader). Output:
spec/library_probe.json = {"symbols": {lib:name: {path, pins: [[number, name, type, unit], ...]}},
"footprints": {lib:name: {path, pads: [number, ...]}}, "missing": [...]}. Deterministic (sorted, no timestamps).

Run: B:/Claude/.venv/Scripts/python.exe -X utf8 scripts/probe_library.py
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
    "Connector_Generic:Conn_01x02",
    "Connector_Generic:Conn_01x03",
    "Connector_Generic:Conn_01x04",
    "Connector_Generic:Conn_01x05",
    "Connector_Generic:Conn_01x06",
    "Connector_Generic:Conn_01x08",
    "Connector_Generic:Conn_01x12",
    "Connector_Generic:Conn_02x03_Odd_Even",
    "Jumper:SolderJumper_2_Open",
    "Jumper:SolderJumper_3_Bridged12",
    "Device:L",
    "Connector_Generic_MountingPin:Conn_01x04_MountingPin",
    "Mechanical:MountingHole",
    "power:+5V",
    "power:+3V3",
    "power:GND",
    "power:PWR_FLAG",
    "RF_Module:ESP-12E",
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
    "LED_SMD:LED_0603_1608Metric",
    "Diode_SMD:D_SOD-123",
    "NetTie:NetTie-2_SMD_Pad0.5mm",
    "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm",
    "MountingHole:MountingHole_3.2mm_M3",
    "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical",
    "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal",
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
]


def main() -> int:
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
                      key=lambda t: (int(t[0]) if t[0].isdigit() else 10_000, t[0]))
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
        pads = sorted({p.number for p in fd.pads if p.number},
                      key=lambda s: (int(s) if s.isdigit() else 10_000, s))
        out["footprints"][lid] = {"path": str(lib.footprint_file(library, name)), "pads": pads}
    dst = HERE / "spec" / "library_probe.json"
    dst.write_text(json.dumps(out, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(f"symbols {len(out['symbols'])}, footprints {len(out['footprints'])}, missing {len(out['missing'])}")
    for m in out["missing"]:
        print("MISSING", m)
    return 1 if out["missing"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
