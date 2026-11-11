"""Shared inputs for the VTOL carrier board scripts.

Invariant: the only design input is ``spec/carrier_spec.json`` (written by
``build_spec.py``). Every component, pin and net in the Universal Circuit IR
built here comes from that file; pin numbers, pin names and electrical types
come from the KiCad 10 symbol libraries on disk (``ai_eda.tools.kicad.library``),
never from memory. A spec pin that does not resolve against the library symbol
is an error, not a guess.

Run with ``B:/Claude/.venv/Scripts/python.exe`` (package ``ai_eda``; the engine
repo is imported, never edited).
"""

from __future__ import annotations

import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path

from ai_eda.ir import (
    CircuitIR,
    Component,
    LibraryRef,
    Net,
    Pin,
    PinElectricalType,
    PinRef,
    Provenance,
    ProvenanceKind,
)
from ai_eda.ir.nets import NetKind
from ai_eda.ir.project import ProjectMeta
from ai_eda.tools.kicad.library import KicadLibrary, SymbolDef

ROOT = Path(__file__).resolve().parent.parent  # .../pcb/vtol_carrier
SPEC_PATH = ROOT / "spec" / "carrier_spec.json"
KICAD_DIR = ROOT / "kicad"
OUT_DIR = ROOT / "out"
WORK_DIR = KICAD_DIR / "work"
#: copper layer count of the delivered board. User decision 2026-11-05: try 2 layers first; if routing does not reach
#: 0 unconnected within ~90 min of attempts, 4 layers (F.Cu signal / In1.Cu GND / In2.Cu power / B.Cu signal).
#: CARRIER_LAYERS overrides it for experiments; CARRIER_WORK_DIR sends build_pcb.py's unrouted board elsewhere.
import os as _os_l  # noqa: E402

LAYERS = int(_os_l.environ.get("CARRIER_LAYERS", "4"))
if LAYERS not in (2, 4):
    raise ValueError(f"CARRIER_LAYERS must be 2 or 4, got {LAYERS}")
BUILD_WORK_DIR = Path(_os_l.environ["CARRIER_WORK_DIR"]) if _os_l.environ.get("CARRIER_WORK_DIR") else WORK_DIR
PROJECT_ID = "vtol_carrier"
#: fixed bookkeeping time so the IR JSON dump is byte-stable (created_at is not design content)
FIXED_TIME = datetime(2026, 11, 5, 0, 0, 0, tzinfo=timezone.utc)

KICAD_BIN = Path(r"C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin")
KICAD_CLI = KICAD_BIN / "kicad-cli.exe"
KICAD_PY = KICAD_BIN / "python.exe"

_MEMBER_RE = re.compile(r"^([A-Z]+[0-9]+)\.(.+?)(?:\(([^()]+)\))?$")

#: schematic sheets: (sheet name, file stem, spec blocks, title)
SHEETS: list[tuple[str, str, tuple[str, ...], str]] = [
    ("Power", "power", ("PWR",), "1S logic battery input, reverse-polarity P-FET, power-switch lead, 5 V boost module, AMS1117-3.3, ground tie, logic-battery monitor"),
    ("NAV", "nav", ("NAV",), "NAV ATmega128A + GY-521 IMU + BMP280 barometer"),
    ("FCU", "fcu", ("FCU",), "FCU ATmega128A"),
    ("COM", "com", ("COM",), "COM ATmega128A + ESP-12E Wi-Fi bridge (soldered), its shifters, straps and flash header"),
    ("GNS", "gns", ("GNS",), "GNS ATmega128A + NEO-7M GPS lead"),
    ("Level_shift", "level_shift", ("SHIFT",), "BSS138 level shifters 5 V <-> 3.3 V"),
    ("IO_connectors", "io", ("IO", "LOG", "LINK"), "Servo / ESC / arm / buzzer / power-sense headers, OpenLog, inter-chip links"),
]


def load_spec() -> dict:
    return json.loads(SPEC_PATH.read_text(encoding="utf-8"))


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def split_lib_id(lib_id: str) -> tuple[str, str]:
    lib, _, name = lib_id.partition(":")
    if not lib or not name:
        raise ValueError(f"bad library id {lib_id!r}")
    return lib, name


#: rev 2b: every footprint is the spec package. The spec's BOM lines (LCSC C-numbers / devicemart no=, user rule
#: 2026-11-05: parts only from JLCPCB or devicemart) were verified online for exactly these packages, so the rev-2
#: 0603 -> 0402 shrink of the stopped attempt is NOT used (it would put unsourced parts on the board).
FOOTPRINT_OVERRIDES: dict[str, str] = {}
#: parts that keep the spec package although their class is overridden (none at the moment)
FOOTPRINT_KEEP: set[str] = set()

#: Programming-only footprints that sit in the cable-tie band (x -3..+3 mm) and are therefore NOT populated (rev-1
#: practice, README_ko.md): ISP is done with the avionics module lifted off the intake, i.e. with the tie removed
#: (firmware/README.md:655-659). The 2x3 2.54 mm footprints stay USBasp-compatible (press a header / pogo adapter
#: into the holes, or solder a header if the tie is routed elsewhere) - user decision.
ISP_IN_BAND = ("J19", "J20", "J21", "J22")
#: reset-hold shunt headers, also programming-only: in the band, unpopulated (press a 2-pin header + shunt into the
#: holes, or short them with a jumper wire, while programming)
HOLD_IN_BAND: tuple[str, ...] = ("J24", "J25", "J26")
#: ESP-12E flash header J3 (rev 2c: 2x3, same pin numbers): programming-only like the ISP headers, at the top of the
#: band right beside the ESP-12E's GPIO0 / TXD0 / RXD0 pins (rev 2b had a 1x6 under the OpenLog at the far +x edge -
#: four ESP nets crossed the board; rev-2b review E-B1 / PF-B1)
ESP_FLASH_IN_BAND: tuple[str, ...] = ("J3",)
IN_BAND = ESP_FLASH_IN_BAND + ISP_IN_BAND + HOLD_IN_BAND
UNPOPULATED_PROG: tuple[str, ...] = ()
POPULATE_OVERRIDES = {r: False for r in IN_BAND + UNPOPULATED_PROG}


def populated(part: dict) -> bool:
    """Spec ``populate`` is True, False or "optional (DNP)"; only True is fitted by default."""
    return POPULATE_OVERRIDES.get(part["refdes"], part["populate"] is True)


def footprint_of(part: dict) -> str:
    """Footprint actually used for a spec part (spec value unless overridden above)."""
    if part["refdes"] in FOOTPRINT_KEEP:
        return part["footprint"]
    return FOOTPRINT_OVERRIDES.get(part["footprint"], part["footprint"])


def resolve_pin(member: str, symbol: SymbolDef) -> tuple[str, str]:
    """``"U1.VCC(21)"`` / ``"C1.1"`` / ``"U5.PG0"`` -> ``(ref, pin number)`` checked against the library symbol."""
    m = _MEMBER_RE.match(member)
    if not m:
        raise ValueError(f"cannot parse pin reference {member!r}")
    ref, token, number = m.group(1), m.group(2), m.group(3)
    numbers = {p.number for p in symbol.pins}
    if number is not None:
        if number not in numbers:
            raise ValueError(f"{member}: pin number {number} not in library symbol {symbol.lib_id}")
        pin = symbol.pin(number)
        assert pin is not None
        if pin.name not in (token, "~") and token != number:
            raise ValueError(f"{member}: library pin {number} is named {pin.name!r}, spec says {token!r}")
        return ref, number
    if token in numbers:
        return ref, token
    named = [p.number for p in symbol.pins if p.name == token]
    if len(named) != 1:
        raise ValueError(f"{member}: pin name {token!r} matches {len(named)} pins of {symbol.lib_id}")
    return ref, named[0]


def _prov(kind: ProvenanceKind, note: str) -> Provenance:
    return Provenance(kind=kind, note=note, created_at=FIXED_TIME)


def build_ir(spec: dict | None = None, library: KicadLibrary | None = None) -> CircuitIR:
    """CircuitIR from the spec: components (library-verified symbol + footprint, library pins), nets, no-connects."""
    spec = spec or load_spec()
    library = library or KicadLibrary()
    symbols: dict[str, SymbolDef] = {}
    components: list[Component] = []
    nc_pins: dict[str, set[str]] = {}
    parts = {p["refdes"]: p for p in spec["parts"]}
    for p in spec["parts"]:
        slib, sname = split_lib_id(p["symbol"])
        sref = library.resolve_symbol(LibraryRef(library=slib, name=sname))
        if not sref.verified:
            raise ValueError(f"{p['refdes']}: symbol {p['symbol']} not in the KiCad library")
        symbols[p["refdes"]] = library.load_symbol(sref)
    for item in spec["no_connect"]:
        ref, num = resolve_pin(item["pin"], symbols[item["pin"].split(".")[0]])
        nc_pins.setdefault(ref, set()).add(num)
    for p in spec["parts"]:
        ref = p["refdes"]
        sym = symbols[ref]
        slib, sname = split_lib_id(p["symbol"])
        flib, fname = split_lib_id(footprint_of(p))
        fref = library.resolve_footprint(LibraryRef(library=flib, name=fname))
        if not fref.verified:
            raise ValueError(f"{ref}: footprint {p['footprint']} not in the KiCad library")
        label = p.get("label", "A")
        kind = ProvenanceKind.ASSUMPTION if "A" in label else ProvenanceKind.USER_REQUIREMENT
        pins = []
        for sp in sym.pins:
            etype = PinElectricalType.NO_CONNECT if sp.number in nc_pins.get(ref, set()) else sp.ir_type
            pins.append(Pin(number=sp.number, name=sp.name, electrical_type=etype,
                            provenance=_prov(ProvenanceKind.AUTHORITATIVE, f"KiCad library symbol {sym.lib_id} ({sym.library_path})")))
        desc = p["role"] + ("" if populated(p) else " [DNP]")
        components.append(Component(
            ref=ref,
            value=p["value"],
            description=desc,
            pins=pins,
            symbol=LibraryRef(library=slib, name=sname, verified=True, library_path=sym.library_path),
            footprint=fref,
            provenance=_prov(kind, f"spec {SPEC_PATH.name} part {ref}: {p.get('source', '')} (label {label}); block {p['block']}"),
        ))
    nets: list[Net] = []
    seen: dict[tuple[str, str], str] = {}
    for n in spec["nets"]:
        prefs = []
        for member in n["members"]:
            ref = member.split(".")[0]
            if ref not in parts:
                raise ValueError(f"net {n['name']}: unknown part {ref}")
            r, num = resolve_pin(member, symbols[ref])
            if (r, num) in seen:
                raise ValueError(f"{member} in nets {seen[(r, num)]} and {n['name']}")
            seen[(r, num)] = n["name"]
            prefs.append(PinRef(component_ref=r, pin_number=num))
        kind = {"power": NetKind.POWER, "ground": NetKind.GROUND}.get(n.get("kind", ""), NetKind.SIGNAL)
        nets.append(Net(name=n["name"], kind=kind, pins=prefs, net_class=net_class_of(n["name"]),
                        provenance=_prov(ProvenanceKind.DERIVED, "; ".join(n.get("sources", []))[:500])))
    # every library pin is either in a net or declared no-connect (refuse silent floating pins)
    for c in components:
        for pin in c.pins:
            key = (c.ref, pin.number)
            if key not in seen and pin.electrical_type is not PinElectricalType.NO_CONNECT:
                raise ValueError(f"pin {c.ref}.{pin.number} ({pin.name}) is neither connected nor no-connect in the spec")
            if key in seen and pin.electrical_type is PinElectricalType.NO_CONNECT:
                raise ValueError(f"pin {c.ref}.{pin.number} is both in net {seen[key]} and no-connect")
    return CircuitIR(
        project=ProjectMeta(id=PROJECT_ID, name="VTOL avionics carrier", created_at=FIXED_TIME,
                            description="76 x 70 mm carrier rev 2c: 4x ATmega128A, GY-521, BMP280, ESP-12E, OpenLog, NEO-7M lead, 1S logic battery + Pololu 5 V buck-boost module"),
        components=components,
        nets=nets,
    )


#: net class per net (stated in README_ko.md and written to the .kicad_pro)
#: logic-battery path to the boost module: 1.34 A peak at a 3.0 V cell (spec power, E) -> >= 1.0 mm (spec layout_rules)
POWER_NETS_WIDE = ("VBAT_LOGIC_RAW", "VBAT_LOGIC", "VBAT_LOGIC_SW")
#: servo supply and return (4 x MG90S, stall ~0.7 A each, A): >= 1.5 mm (spec layout_rules, rev-1 review)
SERVO_NETS = ("SERVO_5V", "GND_PWR")
#: GND is routed as Default tracks for connectivity; the GND pours on both layers carry the current.
POWER_NETS = ("+5V", "+3V3", "GPS_VCC")


def net_class_of(name: str) -> str:
    if name in SERVO_NETS:
        return "Servo"
    if name in POWER_NETS_WIDE:
        return "PowerHigh"
    if name in POWER_NETS or name.endswith("_AVCC"):
        return "Power"
    return "Default"


def block_of(spec: dict) -> dict[str, str]:
    return {p["refdes"]: p["block"] for p in spec["parts"]}


def sheet_of_block(block: str) -> str:
    for name, _stem, blocks, _t in SHEETS:
        if block in blocks:
            return name
    raise KeyError(block)


def dump_ir(ir: CircuitIR, path: Path) -> str:
    """Write the IR JSON (LF, sorted keys) and return its design content hash."""
    data = ir.model_dump(mode="json")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes((json.dumps(data, indent=1, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8"))
    return ir.content_hash()
