"""Build spec/carrier_spec.json - the electrical specification of the VTOL avionics carrier PCB (rev 2) - and check it.

rev 2 (2026-11-05): ESP-12E (ESP8266) Wi-Fi module soldered on the carrier instead of the PLM100 LoRa module, a 1S Li-Po
logic battery (DTP105565P 3.7 V 4200 mAh) through a 5 V BOOST module instead of 2S + buck, and the first build's review
findings designed in. rev 1 (PLM100 + 2S) is kept in archive/rev1_plm100_2s/ (its script: scripts_rev1/build_spec.py).
rev 2b (same day): 5th ATmega128A removed (user), 5 V module = Pololu S9V11F5S6CMA (buck-boost with a low-voltage
cut-off), every BOM line sourced from JLCPCB/LCSC or devicemart (spec/parts_sources.json, written ONLINE by
scripts/fetch_parts_sources.py; this script reads it offline), 4-layer fallback plan.

Invariants this script enforces (it exits non-zero and writes nothing when one fails):
* every part's symbol and footprint exist in the installed KiCad 10 libraries (spec/library_probe.json, written by
  scripts/probe_library.py from the library files on disk) and the footprint's pad numbers equal the symbol's pin
  numbers - no pin number is typed here: a member "U2.PD3" is resolved to pin 28 from the library symbol;
* every pin of every part is in exactly one net or in the explicit no-connect list (with a reason);
* every net has >= 2 members and a source / reason;
* signal paths (UART TX -> RX, SPI, I2C, PWM, ADC, ISP, ESP strapping / reset) are traced through the netlist (series
  resistors and BSS138 channels only, never through a supply rail): a swapped TX / RX or a broken path fails the build;
* no net that a path reaches holds two UART / SPI drivers (TX on TX);
* a 3.3 V-only pin (ESP-12E, BMP280, GPS RX, OpenLog RXI, I2C module side) never reaches an ATmega128A pin except
  through a BSS138 channel;
* the ESP-12E flash pins (CS0, MISO, MOSI, SCLK, GPIO9, GPIO10) are connected to nothing;
* every part with a BOM line maps to an online-verified source whose package matches the footprint (chip size read
  from the footprint name); a part without a source fails the build;
* the candidate logic-battery spots are computed from the CAD report (read-only) - fit inside the bay bore, overlap with
  the CAD parts' boxes, lateral CG shift.
Deterministic: same inputs -> byte-identical JSON (sorted, no timestamps).

Run: B:/Claude/.venv/Scripts/python.exe -X utf8 scripts/build_spec.py
"""
from __future__ import annotations

import json
import math
import re
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
PROJ = HERE.parent.parent                      # B:/0)AI_Cad/CanSat_EDF_Rocket
PROBE = json.loads((HERE / "spec" / "library_probe.json").read_text(encoding="utf-8"))
OUT = HERE / "spec" / "carrier_spec.json"
CAD = json.loads((PROJ / "cad" / "exports" / "cadquery_report_vtol.json").read_text(encoding="utf-8"))
PARAMS = json.loads((PROJ / "params.json").read_text(encoding="utf-8"))

# ----------------------------------------------------------------------------------------------- sources
P = "params.json"                      # B:/0)AI_Cad/CanSat_EDF_Rocket/params.json
BH = "firmware/atmega128_fc/src/board.h"
PG = "firmware/atmega128_fc/src/params_gen.h"
RM = "firmware/README.md"
PIO = "firmware/atmega128_fc/platformio.ini"
CADR = "cad/exports/cadquery_report_vtol.json"
CADC = "cad/exports/cadquery_checks_vtol.md"
HW = "docs/v0.9_하드웨어_변경.md"
# 2026-11-11: the P3 study is not in this repository - its kept copy is the deliverables folder
# B:/0)AI_Cad/CanSat_v0.9/02_펌웨어/설계/제안3_5번째칩 (README.md); rev 2c has no 5th-chip footprint.  The label below is
# unchanged because the spec and the KiCad IR quote it
P3 = "fw09/p3/README.md (scratch study, P3 fifth-chip proposal)"
USER4 = "user decision 2026-11-04"
USER5 = "user decision 2026-11-05"
REV = "rev-1 independent review 2026-11-05"
REV2 = "rev-2b independent review 2026-11-05"
ESPLIB = "KiCad RF_Module:ESP-12E (symbol RF_Module.kicad_sym, footprint RF_Module.pretty/ESP-12E.kicad_mod)"
ESPA = ("A: Espressif ESP8266EX datasheet / hardware design guidelines and the Ai-Thinker ESP-12E/F datasheet are NOT "
        "on disk - check them")
BAT = "devicemart no=16021596 product page (read by the main session 2026-11-05)"

SOURCES = {
    "params.json": "B:/0)AI_Cad/CanSat_EDF_Rocket/params.json (read-only; line numbers as of 2026-11-04 15:15)",
    BH: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + BH,
    PG: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + PG + " (generated from params.json)",
    RM: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + RM,
    PIO: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + PIO,
    "firmware/atmega128_fc/src/hal/*.c": "B:/0)AI_Cad/CanSat_EDF_Rocket/firmware/atmega128_fc/src/hal/",
    "firmware/atmega128_fc/fc_config.json": "B:/0)AI_Cad/CanSat_EDF_Rocket/firmware/atmega128_fc/fc_config.json",
    CADR: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + CADR + " (parts.*.bbox_mm, totals.mass_g / com_mm)",
    CADC: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + CADC,
    HW: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + HW,
    "cad/layout.py": "B:/0)AI_Cad/CanSat_EDF_Rocket/cad/layout.py",
    "sim/*.py": "B:/0)AI_Cad/CanSat_EDF_Rocket/sim/ (tether.py, link_model.py, sim6dof.py)",
    P3: "C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/fw09/p3/README.md",
    "KiCad libraries": "C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/share/kicad/{symbols,footprints} "
                       "(probed: spec/library_probe.json)",
    ESPLIB: "C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/share/kicad/footprints/RF_Module.pretty/ESP-12E.kicad_mod "
            "(module outline F.Fab -8..8 x -12..12 mm, antenna end at local -y; keep-out zone on every copper layer "
            "x -8.12..8.12, y -12.12..-4.80: no tracks, vias, pads, pour, footprints)",
    BAT: "1S Li-Po 'DTP105565P' 3.7 V 4200 mAh, 10.5 x 55.5 x 67 mm, 84 g, lead code 'C5264RR'",
    "devicemart": "user-chosen modules: no=15442588 (BMP280 TYE-BMP29), no=10895400 (NEO-7M SZH-NT07), no=1247052 "
                  "(GY-521 SZH-EK007); pin orders not printed on the pages (A)",
    REV: "C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb/"
         "review_{electrical,pcb-fab,mech-fit}/ (findings E1-E11, F1-F9, MF-1..MF-9 of the rev-1 board)",
}

# ----------------------------------------------------------------------------------------------- library ids
MCU = ("MCU_Microchip_ATmega:ATmega128A-A", "Package_QFP:TQFP-64_14x14mm_P0.8mm")
LDO = ("Regulator_Linear:AMS1117-3.3", "Package_TO_SOT_SMD:SOT-223-3_TabPin2")
ESP = ("RF_Module:ESP-12E", "RF_Module:ESP-12E")
NFET = ("Transistor_FET:BSS138", "Package_TO_SOT_SMD:SOT-23")
PFET = ("Transistor_FET:AO3401A", "Package_TO_SOT_SMD:SOT-23")
NPN = ("Transistor_BJT:MMBT3904", "Package_TO_SOT_SMD:SOT-23")
XTAL = ("Device:Crystal_GND24", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm")
R0603 = ("Device:R", "Resistor_SMD:R_0603_1608Metric")
C0402 = ("Device:C", "Capacitor_SMD:C_0402_1005Metric")
C0603 = ("Device:C", "Capacitor_SMD:C_0603_1608Metric")
C0805 = ("Device:C", "Capacitor_SMD:C_0805_2012Metric")
C1206 = ("Device:C", "Capacitor_SMD:C_1206_3216Metric")
CPOL = ("Device:C_Polarized", "Capacitor_SMD:CP_Elec_6.3x5.4")   # 6.3 x 5.4 can: the sourced part (C2833309)
H5V = ("Connector_Generic:Conn_01x05", "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical")
TANT_A = ("Device:C_Polarized", "Capacitor_Tantalum_SMD:CP_EIA-3216-18_Kemet-A")
TANT_B = ("Device:C_Polarized", "Capacitor_Tantalum_SMD:CP_EIA-3528-21_Kemet-B")
IND = ("Device:L", "Inductor_SMD:L_0805_2012Metric")
LED = ("Device:LED", "LED_SMD:LED_0603_1608Metric")
SCHOTTKY = ("Device:D_Schottky", "Diode_SMD:D_SOD-123")
NETTIE = ("Device:NetTie_2", "NetTie:NetTie-2_SMD_Pad0.5mm")
SJ3 = ("Jumper:SolderJumper_3_Bridged12", "Jumper:SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm")
ISP = ("Connector:AVR-ISP-6", "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical")
H2X3V = ("Connector_Generic:Conn_02x03_Odd_Even", "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical")
H2V = ("Connector_Generic:Conn_01x02", "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical")
H2H = ("Connector_Generic:Conn_01x02", "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal")
H3H = ("Connector_Generic:Conn_01x03", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal")
H4H = ("Connector_Generic:Conn_01x04", "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal")
H6V = ("Connector_Generic:Conn_01x06", "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical")
H8V = ("Connector_Generic:Conn_01x08", "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical")
S6H = ("Connector_Generic:Conn_01x06", "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Horizontal")
JSTGH = ("Connector_Generic_MountingPin:Conn_01x04_MountingPin",
         "Connector_JST:JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical")
JSTPH = ("Connector_Generic:Conn_01x02", "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical")
SPOX2 = ("Connector_Generic:Conn_01x02", "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical")

RAILS = {"+5V", "+3V3", "GND", "GND_PWR", "SERVO_5V", "VBAT_LOGIC_RAW", "VBAT_LOGIC", "VBAT_LOGIC_SW"}

# ----------------------------------------------------------------------------------------------- model
PARTS: dict[str, dict] = {}
NETS: dict[str, dict] = {}
NC: dict[str, str] = {}             # "REF.pin" -> reason
_counters: dict[str, int] = defaultdict(int)
ERRORS: list[str] = []


def _next(prefix: str) -> str:
    while True:
        _counters[prefix] += 1
        ref = f"{prefix}{_counters[prefix]}"
        if ref not in PARTS:
            return ref


def part(ref: str | None, prefix: str, lib: tuple[str, str], value: str, role: str, source: str,
         populate: bool | str = True, label: str = "F", note: str | None = None, block: str = "") -> str:
    ref = ref or _next(prefix)
    if ref in PARTS:
        ERRORS.append(f"duplicate refdes {ref}")
    sym, fp = lib
    if sym not in PROBE["symbols"]:
        ERRORS.append(f"{ref}: symbol {sym} not probed / not on disk")
    if fp not in PROBE["footprints"]:
        ERRORS.append(f"{ref}: footprint {fp} not probed / not on disk")
    PARTS[ref] = {"refdes": ref, "role": role, "value": value, "symbol": sym, "footprint": fp,
                  "populate": populate, "label": label, "source": source, "block": block}
    if note:
        PARTS[ref]["note"] = note
    return ref


def net(name: str, *members: str, source: str = "", note: str | None = None, kind: str | None = None) -> None:
    n = NETS.setdefault(name, {"name": name, "members": [], "sources": [], "notes": []})
    n["members"].extend(members)
    if source and source not in n["sources"]:
        n["sources"].append(source)
    if note and note not in n["notes"]:
        n["notes"].append(note)
    if kind:
        n["kind"] = kind


def nc(member: str, reason: str) -> None:
    NC[member] = reason


def resolve(member: str) -> list[str]:
    """'U2.PD3' -> ['28'] from the library symbol (pin name first, then pin number)."""
    ref, _, fn = member.partition(".")
    if ref not in PARTS:
        ERRORS.append(f"member {member}: unknown refdes")
        return []
    pins = PROBE["symbols"][PARTS[ref]["symbol"]]["pins"]
    by_name = [p[0] for p in pins if p[1] == fn]
    if by_name:
        return by_name
    by_num = [p[0] for p in pins if p[0] == fn]
    if by_num:
        return by_num
    ERRORS.append(f"member {member}: no pin named / numbered {fn!r} in {PARTS[ref]['symbol']}")
    return []


def r(value: str, a: str, b: str, role: str, source: str, populate: bool | str = True, label: str = "A",
      block: str = "", lib=R0603) -> str:
    ref = part(None, "R", lib, value, role, source, populate, label, block=block)
    net(a, f"{ref}.1")
    net(b, f"{ref}.2")
    return ref


def c(value: str, a: str, b: str, role: str, source: str, populate: bool | str = True, label: str = "A",
      block: str = "", lib=C0603) -> str:
    ref = part(None, "C", lib, value, role, source, populate, label, block=block)
    net(a, f"{ref}.1")
    net(b, f"{ref}.2")
    return ref


# ----------------------------------------------------------------------------------------------- MCU blocks
DEC0402 = (f"; 0402, pad <= 3 mm from its VCC / GND pin pair on F.Cu with its own GND via, pre-routed and LOCKED before "
           f"Freerouting ({REV} F1: rev-1 caps were 3.5 mm away but had 71-92 mm of routed path)")
XTAL0402 = (f"; 0402, crystal + load caps <= 3 mm from XTAL1 / XTAL2, hand-routed on F.Cu without vias, locked, solid GND "
            f"under them, >= 1 mm from the board edge ({REV} F2)")
MCU_REFS = {"NAV": "U1", "FCU": "U2", "COM": "U3", "GNS": "U4"}
ISP_REFS = {"NAV": "J19", "FCU": "J20", "COM": "J21", "GNS": "J22"}
# The unpopulated 5th ATmega128A (U5 + Y5, C33-C40, L5, R9-R10, D5, J23) was REMOVED by the user (2026-11-05, "3번으로
# 가자 미실장인데 왜 만들어둔거야"). Its reference designators are left as gaps so every later refdes stays the same
# as in the stopped rev-2 attempt (placement / routing scripts key on them).
REMOVED_5TH = {"U": 1, "Y": 1, "C": 8, "L": 1, "R": 2, "D": 1}
REMOVED_5TH_REFS = ["U5", "Y5", "C33", "C34", "C35", "C36", "C37", "C38", "C39", "C40", "L5", "R9", "R10", "D5", "J23"]


def mcu_block(role: str, populate: bool | str, led_pin: str, led_src: str, mosi: str, miso: str, sck: str) -> None:
    u, j = MCU_REFS[role], ISP_REFS[role]
    blk = role
    part(u, "U", MCU, "ATmega128A-AU 16 MHz 5 V", f"{role} microcontroller",
         {"NAV": f"{P} flight_computer.nav (line 2169-2184); {BH}:5-7, 46-56",
          "FCU": f"{P} flight_computer.pins (line 2147-2167); {BH}:8-10, 20-44",
          "COM": f"{P} flight_computer.com.pins (line 2208-2223); {BH}:11-12, 58-76",
          "GNS": f"{P} flight_computer.gns.pins (line 2266-2271); {BH}:95-102"}[role],
         populate, "F", block=blk)
    src_sup = f"{RM}:310 (VCC, AVCC(LC), 100 nF per VCC/GND pair); {PIO}:51-56 (16 MHz crystal, CKOPT on)"
    net("+5V", f"{u}.VCC", f"{u}.~{{PEN}}", f"{j}.VCC", source=f"{P} flight_computer.vcc_V 5.0 (line 2137)")
    net("GND", f"{u}.GND", f"{j}.GND")
    c("100n", "+5V", "GND", f"{role} VCC pin 21 decoupling", src_sup + DEC0402, populate, block=blk, lib=C0402)
    c("100n", "+5V", "GND", f"{role} VCC pin 52 decoupling", src_sup + DEC0402, populate, block=blk, lib=C0402)
    c("10u", "+5V", "GND", f"{role} local bulk", "A: 10 uF ceramic per chip (bulk next to the chip)", populate,
      block=blk, lib=C0805)
    lref = part(None, "L", IND, "10uH", f"{role} AVCC filter inductor", f"{RM}:310 'AVCC(LC)'; value A",
                populate, "A", block=blk)
    net("+5V", f"{lref}.1")
    net(f"{role}_AVCC", f"{lref}.2", f"{u}.AVCC", source=f"{RM}:310 AVCC(LC)")
    c("100n", f"{role}_AVCC", "GND", f"{role} AVCC decoupling", src_sup + DEC0402, populate, block=blk, lib=C0402)
    net(f"{role}_AREF", f"{u}.AREF",
        source=f"firmware/atmega128_fc/src/hal/adc.h:1-8 (AREF carries only its 100 nF; internal 2.56 V ref); {BH}:37")
    c("100n", f"{role}_AREF", "GND", f"{role} AREF capacitor (internal 2.56 V reference)",
      f"{BH}:37; firmware/atmega128_fc/src/hal/adc.h:5-6; {RM}:205", populate, "F", block=blk)
    y = part(None, "Y", XTAL, "16 MHz, CL 18 pF, 3225 SMD (X322516MRB4SI)", f"{role} crystal",
             f"{P} flight_computer.f_cpu_hz 16000000 (line 2136); {PIO}:51-55 lfuse 0x3F (ext. crystal, CKSEL 1111, "
             "SUT 11), hfuse 0xCF (CKOPT programmed = full-swing oscillator)" + XTAL0402, populate, "F", block=blk,
             note="SMD 3225 chosen over HC-49S for height / mass (A; P3 README section 6 lists both). CL 18 pF from the "
                  "LCSC page of C70562 (F, spec/parts_sources.json); load caps 27 pF: C = 2 x (CL - Cstray) = "
                  "2 x (18 - 3..5) = 26-30 pF (formula and stray A) -> 27 pF C0G (LCSC C1557) gives ~16.5-18.5 pF "
                  "effective (E); rev 2b used 22 pF (~+10..20 ppm, E) - changed after the rev-2b review E-m2. The JLC "
                  "Basic crystal C13738 has CL 9 pF and was NOT used")
    net(f"{role}_XTAL1", f"{u}.XTAL1", f"{y}.1")
    net(f"{role}_XTAL2", f"{u}.XTAL2", f"{y}.3")
    net("GND", f"{y}.G")
    c("27p", f"{role}_XTAL1", "GND", f"{role} crystal load cap",
      f"{P3} section 6 (2 x 22 pF) matched to the crystal's CL 18 pF -> 27 pF ({REV2} E-m2; value A)" + XTAL0402,
      populate, block=blk, lib=C0402)
    c("27p", f"{role}_XTAL2", "GND", f"{role} crystal load cap",
      f"{P3} section 6 (2 x 22 pF) matched to the crystal's CL 18 pF -> 27 pF ({REV2} E-m2; value A)" + XTAL0402,
      populate, block=blk, lib=C0402)
    net(f"{role}_RESET", f"{u}.~{{RESET}}", f"{j}.~{{RST}}", source=f"{RM}:311 ISP pin 5 RST")
    r("10k", f"{role}_RESET", "+5V", f"{role} reset pull-up", f"{P3} section 6 (reset 10 kOhm + 100 nF)", populate,
      block=blk)
    c("100n", f"{role}_RESET", "GND", f"{role} reset capacitor", f"{P3} section 6 (reset 10 kOhm + 100 nF)", populate,
      block=blk)
    part(j, "J", ISP, "AVR ISP 2x3 (USBasp)", f"{role} ISP header (MOSI = PE0/PDI, MISO = PE1/PDO, SCK = PB1)",
         f"{RM}:311 (MOSI->PE0, MISO<-PE1, SCK->PB1); {BH}:43", populate, "F", block=blk,
         note="silkscreen: chip name + pin-1 mark next to the header (rev-1 had none)")
    net(mosi, f"{u}.PE0", f"{j}.MOSI")
    net(miso, f"{u}.PE1", f"{j}.MISO")
    net(sck, f"{u}.PB1", f"{j}.SCK")
    r("1k", f"{role}_LED", f"{role}_LED_A", f"{role} status LED resistor", led_src, populate, "F", block=blk)
    d = part(None, "D", LED, "LED 0603 (colour A)", f"{role} status LED", led_src, populate, "F", block=blk)
    net(f"{role}_LED", f"{u}.{led_pin}", source=led_src)
    net(f"{role}_LED_A", f"{d}.A")
    net("GND", f"{d}.K")


# ----------------------------------------------------------------------------------------------- level shifter
def bss138(q: str, hv: str, lv: str, role: str, source: str, populate: bool | str = True, r_hv: str = "10k",
           r_lv: str = "10k", pop_hv: bool | str | None = None, src_hv: str = "", src_lv: str = "",
           block: str = "SHIFT") -> None:
    part(q, "Q", NFET, "BSS138", f"level shifter channel: {role}", source, populate, "F", block=block)
    net("+3V3", f"{q}.G")
    net(lv, f"{q}.S")
    net(hv, f"{q}.D")
    r(r_hv, hv, "+5V", f"{q} HV (5 V) pull-up", "standard BSS138 bidirectional shifter (A values)" + src_hv,
      populate if pop_hv is None else pop_hv, block=block)
    r(r_lv, lv, "+3V3", f"{q} LV (3.3 V) pull-up", "standard BSS138 bidirectional shifter (A values)" + src_lv, populate,
      block=block)


def build() -> None:
    # ======================================================================= MCUs
    mcu_block("NAV", True, "PC1", f"{P} flight_computer.nav.pins.led PC1 (line 2182); {BH}:54",
              "NAV_ISP_MOSI", "NAV_ISP_MISO", "NLINK_SCK")
    mcu_block("FCU", True, "PC0", f"{P} flight_computer.pins.led PC0 (line 2156); {BH}:38",
              "FCU_ISP_MOSI", "FCU_LOG_TX", "NLINK_SCK")
    mcu_block("COM", True, "PC0", f"{P} flight_computer.com.pins.led PC0 (line 2211); {BH}:63",
              "COM_ILINK_RX", "ILINK_C2F", "COM_ISP_SCK")
    mcu_block("GNS", True, "PC0", f"{P} flight_computer.gns.pins.led PC0 (line 2271); {BH}:101",
              "GNS_GLINK_RX", "GLINK_G2N", "GNS_ISP_SCK")
    for prefix, n in REMOVED_5TH.items():      # 5th chip removed (user 2026-11-05): keep later refdes unchanged
        _counters[prefix] += n

    # ======================================================================= inter-chip links (no shifter: all 5 V)
    net("NLINK_SS", "U1.PB0", "U2.PB0",
        source=f"{P} flight_computer.nav.pins.spi_ss / pins.nlink_ss (line 2161, 2178); {BH}:29-30, 50, 79", kind="spi")
    r("10k", "NLINK_SS", "+5V", "nlink SS pull-up (FCU idle-high while the NAV is in reset / programmed)",
      f"{BH}:29-30, 55-56 (FCU SS pull-up keeps the FCU SPI idle); external 10 kOhm in addition to the internal one: A",
      True, block="LINK")
    net("NLINK_SCK", "U1.PB1", "U2.PB1",
        source=f"{P} pins.nlink_sck (line 2162); {BH}:31-32, 51, 80 (also the NAV and FCU ISP SCK)", kind="spi")
    net("NLINK_MOSI", "U1.PB2", "U2.PB2", source=f"{BH}:33, 52, 82", kind="spi")
    net("NLINK_MISO", "U1.PB3", "U2.PB3", source=f"{BH}:34, 53, 83 (NAV input with pull-up)", kind="spi")
    net("FCU_ILINK_TX", "U2.PD3", source=f"{P} pins.ilink_tx (line 2160); {BH}:42, 89", kind="uart")
    r("1k", "FCU_ILINK_TX", "COM_ILINK_RX", "ilink series resistor FCU TXD1 -> COM RXD0",
      f"{P} flight_computer.ilink.series_resistor_ohm 1000 (line 2228); {BH}:42, 89", True, "F", block="LINK")
    net("COM_ILINK_RX", "U3.PE0", source=f"{P} com.pins.ilink_rx (line 2209); {BH}:59-61", kind="uart")
    net("ILINK_C2F", "U2.PD2", "U3.PE1",
        source=f"{P} pins.ilink_rx / com.pins.ilink_tx (line 2159, 2210); {BH}:41, 62, 90", kind="uart")
    net("NAV_GLINK_TX", "U1.PD3", source=f"{BH}:99, 105 (NAV PD3 TXD1 -> GNS PE0 RXD0)", kind="uart")
    r("1k", "NAV_GLINK_TX", "GNS_GLINK_RX", "glink series resistor NAV TXD1 -> GNS RXD0 (like the ilink one)",
      f"A: {BH}:105 wires it direct; the 1 kOhm copies the ilink rule ({BH}:59-61) because GNS PE0 is its ISP PDI "
      "(programming the GNS with the NAV running would fight the NAV's TXD1)", True, "A", block="LINK")
    net("GNS_GLINK_RX", "U4.PE0", source=f"{P} gns.pins.glink_rx (line 2269); {BH}:99", kind="uart")
    net("GLINK_G2N", "U1.PD2", "U4.PE1", source=f"{P} gns.pins.glink_tx (line 2270); {BH}:100, 106", kind="uart")
    # programming hold jumpers (one board has no unpluggable link harness)
    part("J24", "J", H2V, "2-pin header + shunt", "NAV reset hold (shunt on = NAV held in reset)",
         f"{RM}:647-651 (program the FCU / GNS: hold the NAV in reset or unplug the link); {BH}:31-32, 99-100",
         True, "A", block="LINK", note="shunt on while programming the FCU or the GNS; silkscreen 'NAV RST HOLD'")
    net("NAV_RESET", "J24.1")
    net("GND", "J24.2")
    part("J25", "J", H2V, "2-pin header + shunt", "FCU reset hold (shunt on = FCU held in reset)",
         f"{RM}:651 (program the COM: unplug the ilink); {BH}:59-61", True, "A", block="LINK",
         note="shunt on while programming the COM (FCU PD3 / PD2 go high-Z); silkscreen 'FCU RST HOLD'")
    net("FCU_RESET", "J25.1")
    net("GND", "J25.2")
    part("J26", "J", H2V, "2-pin header + shunt", "COM reset hold (shunt on = COM held in reset)",
         f"NEW (rev 2): while the ESP-12E is flashed through J3 the COM must not toggle the ESP's RST (COM PC6) or drive "
         f"its RXD0 (COM PD3); {RM}:647-651 hold-in-reset approach", True, "A", block="COM",
         note="shunt on while flashing the ESP-12E; silkscreen 'COM RST HOLD'")
    net("COM_RESET", "J26.1")
    net("GND", "J26.2")

    # ======================================================================= power: 1S logic battery -> boost -> 5 V -> LDO -> 3.3 V
    part("J14", "J", SPOX2, "Molex SPOX 5267-02A 2.50 mm 2-pin vertical (mates the pack's 5264-type housing, A)",
         "logic battery input (1S Li-Po DTP105565P 3.7 V 4200 mAh ONLY, separate from the 6S)",
         f"{USER4} (separate logic battery); {USER5} (this pack, {BAT}); 'C5264RR' read as a Molex 5264-type 2.50 mm "
         f"2-pin housing, red / black leads (A - check the bought pack); vertical so the lead leaves upward into the "
         f"free space beside the 6S pack ({REV} MF-4 bend room)", True, "A", block="PWR",
         note="pin 1 = + (red), pin 2 = - (A: check the pack's polarity with a meter before the first plug-in - Q10 "
              "blocks a reversed pack). Silkscreen '+ - 1S LOGIC BAT 3.0-4.2V' with the + at pin 1. A 2S / 3S pack must "
              "never be plugged in (boost module input and Q10 not rated for it in this design)")
    net("VBAT_LOGIC_RAW", "J14.1", source=USER5)
    net("GND", "J14.2")
    part("Q10", "Q", PFET, "AO3401A (-30 V, -4 A, Vgs(th) -0.5..-1.3 V, Rds(on) <= 85 mOhm at Vgs -2.5 V: A)",
         "reverse-polarity protection (P-MOSFET, high side)",
         f"{USER4} (reverse-polarity protection); part and data A (AO3401A datasheet not on disk)", True, "A",
         block="PWR",
         note="drain = battery +, source = load: the body diode starts conduction, Vgs = -Vbat turns it on; a reversed "
              "pack leaves it off. 1S worst case: Vgs = -3.0 V at the PCM-limited end (-2.5 V if the PCM cuts at 2.5 V, "
              "A) is beyond the -2.5 V Rds(on) rating point and >= 1.9x the max |Vgs(th)| 1.3 V (A) -> fully on; drop "
              "<= 85 mOhm x 1.34 A = 0.11 V at the boost peak (E). |Vgs| <= 4.2 V (+ ringing, E < 8.4 V) stays inside "
              "+-12 V: the rev-1 gate zener is dropped")
    net("VBAT_LOGIC_RAW", "Q10.D")
    net("VBAT_LOGIC", "Q10.S")
    net("LBAT_PFET_G", "Q10.G")
    r("10k", "LBAT_PFET_G", "GND", "Q10 gate resistor", "A", True, block="PWR")
    part("J15", "J", JSTPH, "JST-PH 2.0 2-pin vertical: lead to an external logic power switch",
         "logic power switch",
         f"{USER4} ('power switch'); the arm switch is NOT a power switch ({P} pins.arm_switch line 2158, {RM}:319); "
         f"{REV} MF-5 (rev-1 J15 unreachable with the vehicle closed); {REV2} MF2-4 (a Dupont plug on a 2.54 mm "
         f"vertical header rose 3.1 mm above the carrier envelope -> JST-PH vertical, mated ~11 mm like J17, A)",
         True, "A", block="PWR",
         note="J15 cannot be reached with the vehicle closed (the carrier sits at z 327.1-342.1 inside the intake "
              "spigot, the bay windows start above it): flight use = a lead to a switch (>= 2 A, 2-pole contacts "
              "paralleled or a rated mini toggle, A) where it can be reached - see power.switch_reachability (user "
              "decision). Silkscreen 'LOGIC PWR SW'")
    net("VBAT_LOGIC", "J15.1")
    net("VBAT_LOGIC_SW", "J15.2")
    c("22u 10V", "VBAT_LOGIC_SW", "GND", "boost input capacitor 1", "A (boost input ripple at 1.3 A peak)", True,
      block="PWR", lib=C1206)
    c("22u 10V", "VBAT_LOGIC_SW", "GND", "boost input capacitor 2", "A", True, block="PWR", lib=C1206)
    part("J6", "J", H5V, "1x5 2.54 mm: Pololu S9V11F5S6CMA 5 V step-up/step-down module (VOUT, GND, VIN, EN, PG)",
         "5 V regulator module (boost from the 1S pack, adjustable low-voltage cut-off)",
         f"{USER5} (1S cannot feed a buck / LDO to 5 V; 'prefer a boost MODULE on a pin header'; 'add an under-voltage "
         f"cutoff or at least make the logic-battery monitor mandatory'); devicemart no=15855364 = Pololu #2870 "
         f"(pololu.com/product/2870: 5 V, 1.7 A typ. max, VIN 2-16 V after a 3 V start, EN pin with an 11-turn pot to "
         f"VIN = adjustable low-VIN cut-off, off below EN 0.7 V / on above 0.8 V = ~114 % hysteresis, PG open-drain, "
         f"12.7 x 15.3 x 6.4 mm, no reverse-voltage protection - read 2026-11-05); {P} "
         f"configs.vtol.electronics.carrier.model '5 V buck for the logic' (line 1755) is superseded", True, "A",
         block="PWR",
         note="pin order A: the vendor page names the five edge pins VOUT, GND, VIN, EN, PG in this order - check the "
              "silkscreen of the bought module before soldering. The module sits FLAT on its supplied straight 1x6 "
              "header strip (5 pins used; SEL left open = 5 V output, never bridged to VIN = 6 V). EN and PG are not "
              "wired on the carrier: EN keeps the module's own pot (set the cut-off to 3.2 V before the first flight: "
              "EN = 0.7 V x Vbat / 3.2 V, e.g. 0.81 V at 3.7 V; restart only above ~3.65 V, A/E); PG unused (firmware "
              "does not read it). Height on the header ~2.5 + 6.4 = ~9 mm < 13.4 mm (E). Startup current limited to "
              "~700 mA until 5 V is reached (vendor) - the board's typical 0.27 A load starts fine (E)")
    net("+5V", "J6.1", source=f"{P} flight_computer.vcc_V 5.0 (line 2137); {RM}:244 (5 V logic rail)")
    net("GND", "J6.2")
    net("VBAT_LOGIC_SW", "J6.3")
    nc("J6.4", "Pololu EN: module-internal 11-turn pot to VIN sets the low-voltage cut-off; not wired on the carrier "
               "(a later revision could pull it low from the FCU for a soft-off)")
    nc("J6.5", "Pololu PG (power good, open drain): not read by the firmware; left open")
    c("22u", "+5V", "GND", "5 V rail bulk at the boost output", "A", True, block="PWR", lib=C1206)
    part("U6", "U", LDO, "AMS1117-3.3", "3.3 V LDO (ESP-12E, BMP280, GPS, shifter LV side)",
         f"{P} configs.vtol.electronics.carrier.model 'AMS1117-3.3' (line 1755); {RM}:245-246", True, "F",
         block="PWR")
    net("+5V", "U6.VI")
    net("+3V3", "U6.VO", source=f"{RM}:245-246; {P} electronics.gps.power '3.3 V rail' (line 1012)")
    net("GND", "U6.GND")
    c("10u", "+5V", "GND", "AMS1117 input capacitor", "A (AMS1117 datasheet class value)", True, block="PWR", lib=C0805)
    c("22u 10V tantalum", "+3V3", "GND", "AMS1117 output capacitor (stability, ESR)",
      f"A (AMS1117 class datasheets: >= 22 uF tantalum, ESR in a stable window - check the bought vendor's sheet); "
      f"tantalum, not a large MLCC ({REV} E5)", True, block="PWR", lib=TANT_A)
    # logic battery monitor on the FCU's free PF3 (populated; firmware does not read it yet)
    r("47k 1%", "VBAT_LOGIC_SW", "FCU_LBAT_ADC", "logic battery divider top (NEW, firmware does not read PF3 yet)",
      f"{USER5} (populated by default, ratio for 4.2 V max vs the internal 2.56 V ref): 47k / 47k -> 2.10 V at 4.2 V "
      f"(82 % of full scale), 1.50 V at 3.0 V, 5.0 mV/count, 21 uA drain only while switched on (E). PF3/ADC3 is free "
      f"on the FCU (PF0-PF2 used, PF4-PF7 freed JTAG pins: {PG}:164-173; {P} flight_computer.fuses_note line 2146) "
      f"({REV} E1)", True, "F", block="PWR")
    r("47k 1%", "FCU_LBAT_ADC", "GND", "logic battery divider bottom (NEW)", f"{USER5}; see the top resistor", True,
      "F", block="PWR")
    c("100n", "FCU_LBAT_ADC", "GND", "logic battery divider filter (NEW)", "A (source 23.5 kOhm, sampled slowly)", True,
      block="PWR")
    net("FCU_LBAT_ADC", "U2.PF3",
        source="NEW (not in params / firmware): logic-battery sense on FCU PF3/ADC3 - mandatory monitor of the 1S pack "
               "(boost modules keep running down to ~2 V, A)", kind="adc")

    # ======================================================================= ground tie
    part("NT1", "NT", NETTIE, "net tie", "single tie point logic GND <-> propulsion GND (GND_PWR), at J12 pin 3",
         f"{USER4} (common ground, one tie point); {RM}:247, 323 (GND common); {BH}:84, 91, 107", True, "A",
         block="PWR",
         note="place NT1 touching J12 (ESC lead) pin 3; servo return currents stay on GND_PWR (J8-J13 pin 3 -> J12 pin 3)")
    net("GND", "NT1.1")
    net("GND_PWR", "NT1.2", source=f"{USER4}; {RM}:322-324 (servo 5 V rail from the ESC BEC, grounds common)")

    # ======================================================================= ESC + servos (FCU PWM, servo power from the ESC BEC)
    servo_src = (f"{RM}:201-203, 315-317, 322-324 (servo V+ = ESC BEC rail, not the logic 5 V); {P} electronics.esc "
                 f"'5V/7A BEC' (line 943)")
    net("SERVO_5V", source=servo_src)
    outs = [("J8", "PB5", "VANE1", f"{P} pins.vane_1_servo PB5/OC1A (line 2148); {BH}:23"),
            ("J9", "PB6", "VANE2", f"{P} pins.vane_2_servo PB6/OC1B (line 2149); {BH}:24"),
            ("J10", "PB7", "VANE3", f"{P} pins.vane_3_servo PB7/OC1C (line 2150); {BH}:25"),
            ("J11", "PE3", "VANE4", f"{P} pins.vane_4_servo PE3/OC3A (line 2151); {BH}:26"),
            ("J12", "PE4", "ESC", f"{P} pins.esc PE4/OC3B (line 2152); {BH}:27")]
    for j, pin, name, src in outs:
        pop = name != "RELEASE"
        role = {"ESC": "ESC lead (signal, BEC +5 V out, GND)",
                "RELEASE": "release servo header (VTOL has no release servo)"}.get(name, f"vane servo {name[-1]} lead (SIG, V+, GND)")
        part(j, "J", H3H, "1x3 2.54 mm right-angle header (S, +, -)", role, src, pop, "F", block="IO",
             note=("unpopulated: configs.vtol removes the release servo, but the firmware still drives PE5 "
                   "(src/main.c:518) - footprint kept in the top slot whose mated plug would touch the spigot bore "
                   f"({REV} MF-2), header and series resistor DNP") if name == "RELEASE" else
             ("ESC red wire = BEC +5 V OUT into SERVO_5V (never into the logic +5V); ESC GND here is the single ground "
              "tie (NT1). Silkscreen 'ESC S + -'") if name == "ESC" else f"silkscreen 'VANE{name[-1]} S + -'")
        net(f"{name}_PWM", f"U2.{pin}", source=src, kind="pwm")
        r("220", f"{name}_PWM", f"{name}_SIG", f"{name} signal series resistor (long lead, back-power limit)",
          "A: 220 Ohm series on a ~0.5 m servo lead limits back-power into an unpowered servo / ESC", pop, block="IO")
        net(f"{name}_SIG", f"{j}.1")
        net("SERVO_5V", f"{j}.2")
        net("GND_PWR", f"{j}.3")
    # rev 2c: the unpopulated release-servo header J13 and its DNP 220 Ohm series resistor are removed (configs.vtol has
    # no release servo; the user removed the unpopulated 5th MCU for the same reason, 2026-11-05). Their refdes stay
    # gaps so every later refdes is unchanged; U2 PE5 (OC3C, still driven by the firmware) is left open.
    _next("R")
    nc("U2.PE5", f"release-servo PWM OC3C ({P} pins.release_servo line 2153; {BH}:28; pwm.c:49 drives it): configs.vtol "
                 "has no release servo -> output left open (rev 2c removed the unpopulated J13 header, "
                 f"{REV2} PF-m6 / MF2-1)")
    r("10k", "ESC_PWM", "GND", "ESC signal pull-down (motor off while the FCU boots / resets)",
      f"{RM}:316 (10 kOhm pull-down on the ESC signal)", True, "F", block="IO")
    for name in ("VANE1", "VANE2", "VANE3", "VANE4"):
        r("10k", f"{name}_PWM", "GND", f"{name} PWM pull-down (servo line low while the FCU boots / is held in reset)",
          f"{REV} E8 (pins Hi-Z until pwm.c:48-49 sets them; J25 holds the FCU in reset while the COM is programmed); "
          f"same as the ESC pull-down ({RM}:316)", True, "A", block="IO")
    c("220u 10V", "SERVO_5V", "GND_PWR", "servo rail bulk capacitor", "A (servo current spikes on the BEC rail)",
      True, block="IO", lib=CPOL)

    # ======================================================================= FCU I/O: arm switch, buzzer, V sense, log
    part("J16", "J", H2H, "1x2 right-angle header", "arming switch lead (top cap toggle, switch to GND)",
         f"{P} pins.arm_switch PC2 (line 2158); {P} configs.vtol.top_cap.switch_hole (line 1852-1857); {BH}:40", True,
         "F", block="IO", note="silkscreen 'ARM SW' (open = safe)")
    net("ARM_SW_EXT", "J16.1")
    net("GND", "J16.2")
    r("1k", "ARM_SW_EXT", "FCU_ARM", "arm input series resistor (ESD / lead noise)", "A", True, block="IO")
    net("FCU_ARM", "U2.PC2",
        source=f"{P} pins.arm_switch (line 2158); firmware/atmega128_fc/src/hal/gpio.c:12-13 (internal pull-up)")
    r("10k", "FCU_ARM", "+5V", "arm input external pull-up (in addition to the internal one)", "A", True, block="IO")
    c("100n", "FCU_ARM", "GND", "arm input filter", "A", True, block="IO")
    part("J17", "J", JSTPH, "JST-PH 2.0 2-pin vertical (top entry)", "buzzer lead (5 V active buzzer in the misc box)",
         f"{P} pins.buzzer PC1 NPN driver (line 2157); configs.vtol.electronics.misc.items (line 1788); {BH}:39; vertical "
         f"so no mated plug points at the motor-lead joint zone ({REV} MF-3)", True, "F", block="IO",
         note="silkscreen 'BUZ + -'")
    net("+5V", "J17.1")
    net("BUZ_N", "J17.2")
    part("Q9", "Q", NPN, "MMBT3904", "buzzer low-side driver", f"{P} pins.buzzer 'PC1 (NPN driver)'; {RM}:206", True,
         "F", block="IO")
    net("FCU_BUZ", "U2.PC1", source=f"{P} pins.buzzer (line 2157)")
    r("1k", "FCU_BUZ", "BUZ_B", "buzzer driver base resistor", "A", True, block="IO")
    net("BUZ_B", "Q9.B")
    r("10k", "BUZ_B", "GND", "buzzer driver base pull-down (off during reset)", "A", True, block="IO")
    net("GND", "Q9.E")
    net("BUZ_N", "Q9.C")
    d = part(None, "D", SCHOTTKY, "SS14 / B5819W class Schottky", "buzzer flyback diode", "A", True, "A", block="IO")
    net("+5V", f"{d}.K")
    net("BUZ_N", f"{d}.A")
    # power sense connector (no GND pin: the reference comes through the single tie NT1)
    part("J18", "J", H4H, "1x4 right-angle header",
         "power-sense lead: V_in (6S bus), V_tether, V_buffer, buffer-current sign",
         f"{P} pins.vbat_adc / v_tether_adc / v_buffer_adc / buf_isign (line 2155, 2165-2167); "
         f"{P} power.electrical_plant_ASSUMED.adc_full_scale_v (line 2028)", True, "F", block="IO",
         note="no GND pin on purpose: a second ground wire to the propulsion side would close a loop with the ESC lead "
              "ground; divider returns use the logic GND, tied once at NT1. Silkscreen 'VIN VTET VBUF ISGN - 6S SENSE "
              "ONLY, NO GND' with pin 1 marked")
    divs = [("1", "VS_VIN", "FCU_VBAT_ADC", "PF0",
             f"{P} pins.vbat_adc 'PF0/ADC0 (divider 47k/4.7k, internal 2.56 V ref)' (line 2155); {PG}:164-165; {RM}:205, 318"),
            ("2", "VS_VTETHER", "FCU_VTET_ADC", "PF1", f"{P} pins.v_tether_adc (line 2165); {PG}:171"),
            ("3", "VS_VBUFFER", "FCU_VBUF_ADC", "PF2", f"{P} pins.v_buffer_adc (line 2166); {PG}:172")]
    for pin, ext, adc, mcu_pin, src in divs:
        net(ext, f"J18.{pin}", source=src)
        r("47k 1%", ext, adc, f"{mcu_pin} divider top", src, True, "F", block="IO")
        r("4.7k 1%", adc, "GND", f"{mcu_pin} divider bottom", src, True, "F", block="IO")
        c("100n", adc, "GND", f"{mcu_pin} divider filter", src, True, "F", block="IO")
        net(adc, f"U2.{mcu_pin}", source=src, kind="adc")
    net("VS_BUF_ISIGN", "J18.4", source=f"{P} pins.buf_isign (line 2167); {PG}:173-174")
    r("10k", "VS_BUF_ISIGN", "FCU_BUF_ISIGN", "buffer-current sign series resistor",
      f"A; 10k limits the clamp current while the hybrid-board signal level is undefined ({REV} E7)", True, block="IO")
    r("100k", "FCU_BUF_ISIGN", "GND", "buffer-current sign pull-down (battery / tether builds: reads low)", "A", True,
      block="IO")
    net("FCU_BUF_ISIGN", "U2.PC3", source=f"{P} pins.buf_isign PC3 (line 2167); {PG}:173-174")
    # OpenLog lying flat in a right-angle socket (FCU USART0 TX only, through a shifter channel)
    part("J5", "J", S6H, "1x6 2.54 mm right-angle socket: OpenLog lies FLAT beside it (outline reserved)",
         "OpenLog logger (microSD) on the carrier",
         f"{P} pins.log_tx PE1/TXD0 (line 2154); {RM}:204, 378-390; {BH}:35-36; lying flat, not upright: an upright "
         f"OpenLog reaches 23+ mm, over the 13.4 mm limit ({REV} MF-1)", True, "F", block="LOG",
         note="OpenLog header order A (SparkFun: GRN, RXI, TXO, VCC, GND, BLK) - check the clone's silkscreen; solder a "
              "right-angle male header on the OpenLog so it plugs in parallel to the carrier. Reserved outline: 19 x 16 "
              "mm (A, measure) on the side the socket opens to, no parts under it (its microSD socket hangs below), card "
              "slot toward a free edge for removal. Silkscreen 'LOG GRN RXI TXO VCC GND BLK'")
    net("FCU_LOG_TX", source=f"{P} pins.log_tx (line 2154); {BH}:35 (also the FCU ISP PDO)", kind="uart")
    bss138("Q8", "FCU_LOG_TX", "OPENLOG_RXI", "FCU PE1 TXD0 -> OpenLog RXI (57600 8N1)",
           f"{RM}:204, 382 (log through one shifter channel, RXI 3.3 V); channel chosen here ({RM}:385 left it open)")
    net("OPENLOG_RXI", "J5.2")
    net("+5V", "J5.4", source=f"{RM}:204, 383 (OpenLog VCC 5 V)")
    net("GND", "J5.5", "J5.6")
    nc("J5.1", "OpenLog GRN/DTR: not used (no bootloader reset)")
    nc("J5.3", "OpenLog TXO: not used (FCU PE0 is the ISP PDI only)")

    # ======================================================================= NAV sensors: I2C through two shifter channels
    lv_i2c = (f"; 10k (not 4.7k): the GY-521 (4.7k, A) and BMP280 (10k, A) modules carry their own 3.3 V pull-ups - keeps "
              f"the low-level sink current < 3 mA ({REV} E3; measure the bought modules, DNP if they pull up hard)")
    bss138("Q1", "I2C_SCL_5V", "I2C_SCL_3V3", "NAV I2C SCL (400 kHz)",
           f"{P} nav.pins.i2c_scl (line 2175); {BH}:47-48; {RM}:199", r_hv="4.7k", r_lv="10k", src_lv=lv_i2c)
    bss138("Q2", "I2C_SDA_5V", "I2C_SDA_3V3", "NAV I2C SDA",
           f"{P} nav.pins.i2c_sda (line 2176); {BH}:47-48; {RM}:199", r_hv="4.7k", r_lv="10k", src_lv=lv_i2c)
    net("I2C_SCL_5V", "U1.PD0",
        source="firmware/atmega128_fc/src/hal/twi.c:229 (internal pull-ups off: external pull-ups on the shifter)",
        kind="i2c")
    net("I2C_SDA_5V", "U1.PD1", kind="i2c")
    part("J1", "J", H8V, "1x8 2.54 mm (GY-521 soldered flat on straight pins)", "IMU module GY-521 MPU-6050 [SZH-EK007]",
         f"devicemart no=1247052; {P} flight_computer.nav (line 2169-2184); {RM}:197 (module VCC 5 V, I2C 3.3 V, 0x68)",
         True, "A", block="NAV",
         note="pin order A: VCC, GND, SCL, SDA, XDA, XCL, AD0, INT - check the silkscreen. Mount flat, parallel to the "
              f"carrier, with a 2 mm foam / nylon support under the far edge (cantilever, {REV} MF-9) - area reserved, "
              "no parts under the module; set fc_config.json imu_mounting after placement")
    net("+5V", "J1.1")
    net("GND", "J1.2", "J1.7")
    net("I2C_SCL_3V3", "J1.3", source="NAV I2C 3.3 V side")
    net("I2C_SDA_3V3", "J1.4")
    nc("J1.5", "GY-521 XDA: auxiliary I2C master, unused")
    nc("J1.6", "GY-521 XCL: auxiliary I2C master, unused")
    bss138("Q3", "IMU_INT_5V", "IMU_INT_3V3", "GY-521 INT -> NAV PE6/INT6 (firmware polls; optional)",
           f"{P} nav.pins.imu_int 'not used, polled' (line 2177); {BH}:49; {RM}:199 (shifter ch4 optional)",
           populate="optional (DNP)", pop_hv=True,
           src_hv=f"; HV pull-up fitted even with Q3 DNP so NAV PE6 never floats ({REV} E11)")
    net("IMU_INT_5V", "U1.PE6")
    net("IMU_INT_3V3", "J1.8")
    part("J2", "J", S6H, "1x6 2.54 mm right-angle socket (module lies flat)", "barometer module BMP280 [TYE-BMP29]",
         f"devicemart no=15442588 (BMP280, 3.3 V, I2C/SPI, right-angle 6-pin header already soldered); replaces the "
         f"GY-63 MS5611 ({USER4})", True, "A", block="NAV",
         note="pin order A: VCC, GND, SCL, SDA, CSB, SDO (common GY-BMP280 order) - check the silkscreen. CSB high = "
              f"I2C; SDO high = address 0x77 (= the firmware's barometer address CAL_MS5611_ADDR 0x77, {PG}:964). "
              "Module outline reserved flat beside the socket")
    net("+3V3", "J2.1", "J2.5", "J2.6")
    net("GND", "J2.2")
    net("I2C_SCL_3V3", "J2.3")
    net("I2C_SDA_3V3", "J2.4")

    # ======================================================================= radio: ESP-12E (ESP8266) soldered on the carrier
    esp_src = f"{USER5} ('ESP-12E 직접 납땜', 'PLM100 완전히 빼기'); {ESPLIB}"
    part("U7", "U", ESP, "ESP-12E (ESP8266EX + 4 MB SPI flash, PCB antenna)",
         "Wi-Fi radio: transparent UART <-> Wi-Fi bridge for the COM (no flight computing on it)", esp_src, True, "F",
         block="COM",
         note="antenna end (footprint local -y, keep-out y -12.12..-4.80) flush with the +y board edge; the footprint's "
              "keep-out (all copper layers) + a board rule area 4 mm wider on both sides (no copper either layer, no "
              "parts, A: Espressif / Ai-Thinker guidance - check). Runs ONLY a UART <-> Wi-Fi bridge (project rule: "
              "all flight computing on the ATmega128s); its firmware is a follow-up")
    net("+3V3", "U7.VCC", source=f"{ESPLIB} pin 8 VCC; 3.0-3.6 V supply ({ESPA})")
    net("GND", "U7.GND")
    c("100u 10V tantalum", "+3V3", "GND", "ESP-12E bulk (TX bursts), at the VCC pin",
      f"{USER5} '>= 100 uF + 100 nF close to the module'; {ESPA}; 10 V rating = 33 % of rating on 3.3 V (6.3 V part = "
      f"52 %, above the usual <= 50 % tantalum derating, A - {REV2} E-m4)", True, block="COM", lib=TANT_B)
    c("10u", "+3V3", "GND", "ESP-12E VCC ceramic", ESPA, True, block="COM", lib=C0805)
    c("100n", "+3V3", "GND", "ESP-12E VCC decoupling (at pin 8, own GND via)", ESPA, True, block="COM", lib=C0402)
    net("ESP_EN", "U7.EN", source=f"{ESPLIB} pin 3 EN (CH_PD): high = chip enabled ({ESPA})")
    r("10k", "ESP_EN", "+3V3", "ESP EN (CH_PD) pull-up", f"{USER5} 'EN pull-up 10k'; {ESPA}", True, block="COM")
    c("100n", "ESP_EN", "GND", "ESP EN RC delay (enable after the 3.3 V rail has settled)",
      f"{ESPA} (RC on CHIP_EN recommended)", True, block="COM")
    net("ESP_GPIO15", "U7.GPIO15", source=f"{ESPLIB} pin 16; GPIO15 low at boot = boot from flash / UART ({ESPA})")
    r("10k", "ESP_GPIO15", "GND", "ESP GPIO15 boot-strap pull-down", f"{USER5}; {ESPA}", True, block="COM")
    net("ESP_GPIO2", "U7.GPIO2", source=f"{ESPLIB} pin 17; GPIO2 high at boot ({ESPA})")
    r("10k", "ESP_GPIO2", "+3V3", "ESP GPIO2 boot-strap pull-up", f"{USER5}; {ESPA}", True, block="COM")
    net("ESP_GPIO0", "U7.GPIO0", source=f"{ESPLIB} pin 18; GPIO0 high = run from flash, low at reset = UART download ({ESPA})")
    r("10k", "ESP_GPIO0", "+3V3", "ESP GPIO0 boot-strap pull-up (run from flash)", f"{USER5}; {ESPA}", True,
      block="COM")
    # host link: COM USART1 (link_uart) <-> ESP UART0 - BOTH directions through BSS138 channels
    net("COM_LINK_TX", "U3.PD3",
        source=f"{P} com.pins.link_uart_tx 'PD3/TXD1 (to module RXD through a BSS138 channel)' (line 2213); {BH}:66",
        kind="uart")
    bss138("Q4", "COM_LINK_TX", "ESP_RXD_SH", "COM PD3 TXD1 -> ESP-12E RXD0 (GPIO3, pin 21)",
           f"{P} com.pins.link_uart_tx (line 2213); {RM}:254, 261 (MCU -> module through a channel)", block="COM")
    r("1k", "ESP_RXD_SH", "ESP_RXD0", "ESP RXD0 series resistor (a programmer on J3 wins against the COM side)",
      f"{USER5} ('series resistors on the COM side' so the COM does not fight the programmer); value A", True,
      block="COM")
    net("ESP_RXD0", "U7.GPIO3/RXD", "J3.4", source=f"{ESPLIB} pin 21 GPIO3/RXD (UART0 RX, also the flash download RX)",
        kind="uart")
    net("ESP_TXD0", "U7.GPIO1/TXD", "J3.3",
        source=f"{ESPLIB} pin 22 GPIO1/TXD (UART0 TX; prints a boot log at 74880 baud after every reset, {ESPA})",
        kind="uart")
    part("Q11", "Q", NFET, "BSS138", "level shifter channel: ESP-12E TXD0 (GPIO1) -> COM PD2 RXD1",
         f"{USER5} ('this time put the ESP TX -> AVR RX direction through a BSS138 channel too'); {REV} E2 (3.3 V direct "
         f"drive leaves <= 0.3 V over V_IH 3.0 V); replaces {P} com.pins.link_uart_rx 'direct' (line 2212)", True, "F",
         block="COM")
    net("+3V3", "Q11.G")
    net("ESP_TXD0", "Q11.S")
    net("COM_LINK_RX", "Q11.D", "U3.PD2",
        source=f"{P} com.pins.link_uart_rx PD2/RXD1 (line 2212); {BH}:65", kind="uart")
    r("10k", "COM_LINK_RX", "+5V", "Q11 HV (5 V) pull-up", "standard BSS138 bidirectional shifter (A values)", True,
      block="COM")
    r("10k", "ESP_TXD0", "+3V3", "Q11 LV (3.3 V) pull-up", "standard BSS138 bidirectional shifter (A values)", True,
      block="COM")
    # ESP RST from COM PC6 (the sx127x reset pin, untouched by the lora_uart build) through a channel
    net("COM_ESP_RESET", "U3.PC6",
        source=f"{P} com.pins.lora_reset PC6 (line 2222); {BH}:74; PC6 is driven only by "
               "firmware/atmega128_fc/src/hal/sx127x.c:87-88, 113 - the lora_uart build leaves it an input (Hi-Z)")
    bss138("Q5", "COM_ESP_RESET", "ESP_RST", "COM PC6 -> ESP-12E RST (pin 1, active low)",
           f"rev-1 used COM PC6 via Q5 for PLM100 RESET_N - reused ({USER5}); {RM}:276 (sx127x RESET through a channel)",
           block="COM", src_lv=f"; this LV pull-up IS the ESP RST 10k pull-up ({USER5} 'RST pull-up 10k'; {ESPA})")
    net("ESP_RST", "U7.~{RST}", "J3.5", source=f"{ESPLIB} pin 1 ~RST (active low)")
    # ESP programming header (3.3 V logic ONLY)
    part("J3", "J", H2X3V, "2x3 2.54 mm vertical: ESP-12E programming header (3.3 V logic ONLY)",
         "ESP-12E flash header: 1 IO0, 2 GND, 3 TXD0, 4 RXD0, 5 RST, 6 GND",
         f"{USER5} (programming header: GND, ESP TXD0, ESP RXD0, GPIO0 to GND via jumper, RST; 3.3 V only)", True, "A",
         block="COM",
         note="pin order chosen here: a shunt on pins 1-2 (IO0-GND) = download mode at the next reset; touch / shunt 5-6 "
              "(RST-GND) = reset. Programmer = a 3.3 V USB-UART: its RX to pin 3, TX to pin 4, GND to pin 2 / 6 - NO "
              "supply pin: the board runs from the logic battery while flashing (ESP RF calibration peaks exceed most "
              "USB-UART 3.3 V outputs, A). A 5 V USB-UART would exceed the ESP's 3.6 V abs. max (A). Silkscreen "
              "'ESP IO0 GND TX RX RST GND - 3V3 ONLY'")
    net("ESP_GPIO0", "J3.1")
    net("GND", "J3.2", "J3.6")
    for pin, why in [("ADC", "TOUT / ADC input (max 1.0 V): unused, left open"),
                     ("GPIO16", "unused (deep-sleep wake not used)"),
                     ("GPIO14", "unused"), ("GPIO12", "unused"), ("GPIO13", "unused (UART0 swap pins not used)"),
                     ("GPIO4", "unused (spare: a future 'bridge ready' line to COM PC5 would need a BSS138 channel)"),
                     ("GPIO5", "unused"),
                     ("CS0", "SPI FLASH pin - must stay unconnected"), ("MISO", "SPI FLASH pin - must stay unconnected"),
                     ("GPIO9", "SPI FLASH pin (QIO) - must stay unconnected"),
                     ("GPIO10", "SPI FLASH pin (QIO) - must stay unconnected"),
                     ("MOSI", "SPI FLASH pin - must stay unconnected"), ("SCLK", "SPI FLASH pin - must stay unconnected")]:
        nc(f"U7.{pin}", f"ESP-12E {why} ({ESPLIB}; {ESPA})")
    for pin, why in [("PC3", "link_m0 (Ebyte M0): the lora_uart firmware drives it low; the ESP has no M0"),
                     ("PC4", "link_m1 (Ebyte M1): driven low by the firmware; the ESP has no M1"),
                     ("PC5", "link_aux (Ebyte AUX): input with internal pull-up = 'ready' when open "
                             "(firmware/atmega128_fc/src/hal/lora_uart.c:69-70)"),
                     ("PB0", "lora_spi_nss (sx127x backend only)"), ("PB2", "lora_spi_mosi (sx127x only)"),
                     ("PB3", "lora_spi_miso (sx127x only)"), ("PE7", "lora_dio0 (sx127x only)")]:
        nc(f"U3.{pin}", f"COM {why} - {P} com.pins (line 2214-2221); left open for the ESP-12E (firmware follow-up)")

    # ======================================================================= GPS lead (NEO-7M in the top cap) -> GNS USART1
    part("J7", "J", JSTGH, "JST-GH 1.25 mm 4-pin vertical (top entry)", "GPS lead (u-blox NEO-7M module in the top cap)",
         f"{P} configs.vtol.top_cap.gps (line 1821-1851, lead_hole line 1840-1844); payload_module.gps_pod connector "
         f"'JST-GH 1.25 mm 4-pin'; devicemart no=10895400; vertical: the lead drops from the top-cap hole and plugs in "
         f"from above ({REV} MF-6)", True, "A", block="GNS",
         note="carrier pin order (A, the lead is user-made): 1 VCC, 2 GPS TXD (-> GNS RX), 3 GPS RXD (<- GNS TX), 4 GND; "
              "the GPS end follows the NEO-7M board silkscreen (A: VCC, GND, TXD, RXD, PPS). PPS not wired (firmware "
              "unused). Silkscreen 'GPS 1V 2TX 3RX 4G'")
    net("GPS_VCC", "J7.1", source=f"{P} electronics.gps.power '3.3 V rail, ~45 mA' (line 1012)")
    net("GND", "J7.4", "J7.MP")
    part("JP1", "JP", SJ3, "solder jumper, 1-2 bridged", "GPS supply select: 1-2 = 3.3 V (default, params), 2-3 = 5 V",
         f"{P} electronics.gps.power (line 1012); NEO-7M board supply '3.3-5 V' typical (A, not on the product page)",
         True, "A", block="GNS")
    net("+3V3", "JP1.A")
    net("GPS_VCC", "JP1.C")
    net("+5V", "JP1.B")
    c("10u", "GPS_VCC", "GND", "GPS supply bulk at the connector", "A", True, block="GNS", lib=C0805)
    net("GNS_GPS_TX", "U4.PD3",
        source=f"{P} gns.pins.gps_tx 'PD3/TXD1 (to GPS RX through the level shifter)' (line 2268); {BH}:98", kind="uart")
    bss138("Q6", "GNS_GPS_TX", "GPS_RXD_3V3", "GNS PD3 TXD1 -> GPS RXD (38400 8N1)",
           f"{P} gns.pins.gps_tx (line 2268); electronics.gps.interface 'BOTH lines level-shifted, GPS RX abs max 3.6 V' "
           f"(line 1011)")
    net("GPS_RXD_3V3", "J7.3")
    net("GNS_GPS_RX", "U4.PD2",
        source=f"{P} gns.pins.gps_rx (line 2267); {BH}:97; firmware/atmega128_fc/src/hal/gps_uart.c:299 (RXD1 pull-up)",
        kind="uart")
    bss138("Q7", "GNS_GPS_RX", "GPS_TXD_3V3", "GPS TXD -> GNS PD2 RXD1",
           f"{P} gns.pins.gps_rx (line 2267); electronics.gps.interface (line 1011)")
    net("GPS_TXD_3V3", "J7.2")


# ----------------------------------------------------------------------------------------------- checks
MCU_PIN_NC_REASON = "unused ATmega128A I/O (left open; firmware may enable its pull-up)"
THREE_V3_ONLY = ["J2.3", "J2.4", "J2.5", "J2.6", "J7.3", "J5.2", "J1.3", "J1.4", "U7.GPIO3/RXD", "U7.GPIO1/TXD",
                 "U7.~{RST}", "U7.EN", "U7.GPIO0", "U7.GPIO2", "U7.GPIO15", "J3.1", "J3.3", "J3.4", "J3.5"]
ESP_FLASH_PINS = ["U7.CS0", "U7.MISO", "U7.MOSI", "U7.SCLK", "U7.GPIO9", "U7.GPIO10"]
TX_DRIVERS = ["U2.PD3", "U3.PE1", "U1.PD3", "U4.PE1", "U3.PD3", "U4.PD3", "U2.PE1", "U7.GPIO1/TXD", "J7.2", "U1.PB0",
              "U1.PB1", "U1.PB2", "U2.PB3"]
PATHS = [  # (description, from member, to member)
    ("ilink FCU TXD1 -> COM RXD0", "U2.PD3", "U3.PE0"), ("ilink COM TXD0 -> FCU RXD1", "U3.PE1", "U2.PD2"),
    ("glink NAV TXD1 -> GNS RXD0", "U1.PD3", "U4.PE0"), ("glink GNS TXD0 -> NAV RXD1", "U4.PE1", "U1.PD2"),
    ("nlink SS NAV -> FCU", "U1.PB0", "U2.PB0"), ("nlink SCK NAV -> FCU", "U1.PB1", "U2.PB1"),
    ("nlink MOSI NAV -> FCU", "U1.PB2", "U2.PB2"), ("nlink MISO FCU -> NAV", "U2.PB3", "U1.PB3"),
    ("radio COM TXD1 -> ESP RXD0 (GPIO3, pin 21) via Q4 + 1k", "U3.PD3", "U7.GPIO3/RXD"),
    ("radio ESP TXD0 (GPIO1, pin 22) -> COM RXD1 via Q11", "U7.GPIO1/TXD", "U3.PD2"),
    ("radio COM PC6 -> ESP RST (pin 1) via Q5", "U3.PC6", "U7.~{RST}"),
    ("ESP flash header TXD0 <- ESP", "U7.GPIO1/TXD", "J3.3"), ("ESP flash header RXD0 -> ESP", "J3.4", "U7.GPIO3/RXD"),
    ("ESP flash header RST", "J3.5", "U7.~{RST}"), ("ESP flash header IO0", "J3.1", "U7.GPIO0"),
    ("GPS GNS TXD1 -> GPS RXD", "U4.PD3", "J7.3"), ("GPS GPS TXD -> GNS RXD1", "J7.2", "U4.PD2"),
    ("log FCU TXD0 -> OpenLog RXI", "U2.PE1", "J5.2"),
    ("I2C SCL NAV -> GY-521", "U1.PD0", "J1.3"), ("I2C SCL NAV -> BMP280", "U1.PD0", "J2.3"),
    ("I2C SDA NAV -> GY-521", "U1.PD1", "J1.4"), ("I2C SDA NAV -> BMP280", "U1.PD1", "J2.4"),
    ("IMU INT -> NAV PE6 (optional)", "J1.8", "U1.PE6"),
    ("vane 1 PWM", "U2.PB5", "J8.1"), ("vane 2 PWM", "U2.PB6", "J9.1"), ("vane 3 PWM", "U2.PB7", "J10.1"),
    ("vane 4 PWM", "U2.PE3", "J11.1"), ("ESC PWM", "U2.PE4", "J12.1"),
    ("V_in sense -> PF0", "J18.1", "U2.PF0"), ("V_tether sense -> PF1", "J18.2", "U2.PF1"),
    ("V_buffer sense -> PF2", "J18.3", "U2.PF2"), ("buffer-current sign -> PC3", "J18.4", "U2.PC3"),
    ("arm switch -> PC2", "J16.1", "U2.PC2"), ("buzzer PC1 -> NPN base", "U2.PC1", "Q9.B"),
    ("logic battery sense -> PF3", "VBATSW", "U2.PF3"),
]
for _role, _u in MCU_REFS.items():
    _j = ISP_REFS[_role]
    PATHS += [(f"{_role} ISP MOSI -> PE0/PDI", f"{_j}.MOSI", f"{_u}.PE0"),
              (f"{_role} ISP MISO <- PE1/PDO", f"{_u}.PE1", f"{_j}.MISO"),
              (f"{_role} ISP SCK -> PB1", f"{_j}.SCK", f"{_u}.PB1"),
              (f"{_role} ISP RST -> RESET", f"{_j}.~{{RST}}", f"{_u}.~{{RESET}}")]
STRAPS = [  # (pin, rail it must be pulled to through exactly one resistor, why)
    ("U7.EN", "+3V3", "EN high = chip enabled"), ("U7.~{RST}", "+3V3", "RST high = run"),
    ("U7.GPIO0", "+3V3", "GPIO0 high at reset = run from flash"), ("U7.GPIO2", "+3V3", "GPIO2 high at boot"),
    ("U7.GPIO15", "GND", "GPIO15 low at boot"),
]


def natural(s: str) -> list:
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", s)]


def check() -> dict:
    report: dict = {}
    pin_net: dict[str, str] = {}
    for n in NETS.values():
        res = []
        for m in n["members"]:
            ref = m.split(".")[0]
            for num in resolve(m):
                key = f"{ref}.{num}"
                if key in pin_net and pin_net[key] != n["name"]:
                    ERRORS.append(f"pin {key} ({m}) in two nets: {pin_net[key]} and {n['name']}")
                pin_net[key] = n["name"]
                res.append(f"{m}({num})" if m.split('.', 1)[1] != num else m)
        n["resolved"] = sorted(set(res), key=natural)
        if len(set(res)) < 2:
            ERRORS.append(f"net {n['name']} has fewer than 2 members: {n['members']}")
        if not n["sources"] and n["name"] not in RAILS and not any(n["name"].startswith(f"{k}_") for k in MCU_REFS):
            n["sources"].append("derived: support / pull-up / series-resistor node (see the part entries)")
    nc_keys: dict[str, str] = {}
    for m, why in NC.items():
        ref = m.split(".")[0]
        for num in resolve(m):
            nc_keys[f"{ref}.{num}"] = why
    unassigned = []
    for ref, p in PARTS.items():
        sym = PROBE["symbols"][p["symbol"]]
        fp_pads = set(PROBE["footprints"][p["footprint"]]["pads"])
        sym_pins = {pp[0] for pp in sym["pins"]}
        if fp_pads != sym_pins:
            ERRORS.append(f"{ref}: footprint pads {sorted(fp_pads)} != symbol pins {sorted(sym_pins)}")
        for num, name, _t, _u in sym["pins"]:
            key = f"{ref}.{num}"
            if key in pin_net and key in nc_keys:
                ERRORS.append(f"{key} is both connected ({pin_net[key]}) and no-connect")
            if key not in pin_net and key not in nc_keys:
                if ref in MCU_REFS.values():
                    nc_keys[key] = MCU_PIN_NC_REASON + f" [{name}]"
                    NC[f"{ref}.{name}"] = MCU_PIN_NC_REASON
                else:
                    unassigned.append(f"{key} ({name})")
    for u in unassigned:
        ERRORS.append(f"pin not connected and not declared no-connect: {u}")
    report["pins_total"] = sum(len(PROBE["symbols"][p["symbol"]]["pins"]) for p in PARTS.values())
    report["pins_connected"] = len(pin_net)
    report["pins_no_connect"] = len(nc_keys)

    def member_net(m: str) -> str | None:
        ref = m.split(".")[0]
        nums = resolve(m)
        return pin_net.get(f"{ref}.{nums[0]}") if nums else None

    # graph: nodes = nets; edges through resistors and BSS138 S<->D; supply rails block
    adj: dict[str, set[str]] = defaultdict(set)
    bss_edges: set[frozenset] = set()
    for ref, p in PARTS.items():
        pairs = []
        if p["symbol"] == "Device:R":
            pairs = [("1", "2")]
        elif p["symbol"] == "Transistor_FET:BSS138":
            pairs = [("2", "3")]
        for a, b in pairs:
            na, nb = pin_net.get(f"{ref}.{a}"), pin_net.get(f"{ref}.{b}")
            if na and nb and na not in RAILS and nb not in RAILS:
                adj[na].add(nb)
                adj[nb].add(na)
                if p["symbol"] == "Transistor_FET:BSS138":
                    bss_edges.add(frozenset((na, nb)))

    def reach(start: str, through_shifter: bool = True) -> set[str]:
        seen, todo = {start}, [start]
        while todo:
            cur = todo.pop()
            for nx in adj[cur]:
                if nx in seen or (not through_shifter and frozenset((cur, nx)) in bss_edges):
                    continue
                seen.add(nx)
                todo.append(nx)
        return seen

    path_rows = []
    for desc, a, b in PATHS:
        nb = member_net(b)
        if a == "VBATSW":
            na = "VBAT_LOGIC_SW"
            ok = any(PARTS[ref]["symbol"] == "Device:R" and {pin_net.get(f"{ref}.1"), pin_net.get(f"{ref}.2")} == {na, nb}
                     for ref in PARTS)
            path_rows.append({"path": desc, "from": "VBAT_LOGIC_SW", "to": f"{b} [{nb}]", "ok": ok})
        else:
            na = member_net(a)
            ok = bool(na and nb and nb in reach(na))
            path_rows.append({"path": desc, "from": f"{a} [{na}]", "to": f"{b} [{nb}]", "ok": ok})
        if not ok:
            ERRORS.append(f"signal path broken: {desc} ({a} -> {b})")
    report["signal_paths"] = path_rows
    conflicts = 0
    for d in TX_DRIVERS:
        comp = reach(member_net(d) or "")
        others = [o for o in TX_DRIVERS if o != d and member_net(o) in comp]
        if others:
            conflicts += 1
            ERRORS.append(f"driver conflict: {d} reaches other drivers {others}")
    report["driver_conflicts"] = conflicts
    mcu_nets = {pin_net[k] for k in pin_net if k.split(".")[0] in MCU_REFS.values()} - RAILS
    for m in THREE_V3_ONLY:
        n0 = member_net(m)
        if n0 is None or n0 in RAILS:
            continue
        hit = reach(n0, through_shifter=False) & mcu_nets
        if hit:
            ERRORS.append(f"3.3 V-only pin {m} reaches a 5 V ATmega128A net without a shifter: {sorted(hit)}")
    report["three_v3_only_pins_checked"] = len(THREE_V3_ONLY)
    for m in ESP_FLASH_PINS:
        nums = resolve(m)
        if nums and f"{m.split('.')[0]}.{nums[0]}" in pin_net:
            ERRORS.append(f"ESP-12E flash pin {m} is connected - it must stay open")
    report["esp_flash_pins_open"] = len(ESP_FLASH_PINS)
    strap_rows = []
    for pin, rail, why in STRAPS:
        n0 = member_net(pin)
        res_to_rail = [ref for ref, p in PARTS.items() if p["symbol"] == "Device:R" and p["populate"] is True and
                       {pin_net.get(f"{ref}.1"), pin_net.get(f"{ref}.2")} == {n0, rail}]
        ok = len(res_to_rail) == 1
        strap_rows.append({"pin": pin, "net": n0, "to": rail, "resistor": res_to_rail, "why": why, "ok": ok})
        if not ok:
            ERRORS.append(f"ESP strap {pin}: expected one fitted resistor to {rail}, found {res_to_rail}")
    report["esp_straps"] = strap_rows
    report["bss138_channels"] = sorted((k for k, p in PARTS.items() if p["symbol"] == "Transistor_FET:BSS138"),
                                       key=natural)
    return report


# ----------------------------------------------------------------------------------------------- CAD-derived numbers
def battery_spots() -> dict:
    """Candidate places for the 10.5 x 55.5 x 67 mm logic pack in the VTOL bay, from the CAD report boxes (read-only)."""
    parts = CAD["parts"]
    bore_r = PARAMS["body"]["od"] / 2 - PARAMS["body"]["wall"]           # avionics tube inner radius
    z0 = parts["battery"]["bbox_mm"][0][2]                               # tray bottom + 2 (same seat as the 6S / ESC)
    bx0, bx1 = parts["battery"]["bbox_mm"][0][0], parts["battery"]["bbox_mm"][1][0]
    by1 = parts["battery"]["bbox_mm"][1][1]
    ex1 = parts["esc"]["bbox_mm"][1][0]
    t, w, length, gap = 10.5, 55.5, 67.0, 0.5
    spots = {
        "A_minus_x_face_of_6S": ([bx0 - gap - t, bx0 - gap], [-w / 2, w / 2], [z0, z0 + length],
                                 "flat against the 6S pack's -x face (battery-side windows 136-224 deg)"),
        "B_plus_x_face_of_ESC": ([ex1 + gap, ex1 + gap + t], [-w / 2, w / 2], [z0, z0 + length],
                                 "flat against the ESC's +x face (ESC-side cooling windows 316-44 deg)"),
        "C_plus_y_side_of_6S": ([bx1 - gap - w, bx1 - gap], [by1 + gap, by1 + gap + t], [z0, z0 + length],
                                "on edge beside the 6S pack's +y side (side windows 90 / 112 deg)"),
        "D_minus_y_side_of_6S": ([bx1 - gap - w, bx1 - gap], [-by1 - gap - t, -by1 - gap], [z0, z0 + length],
                                 "on edge beside the 6S pack's -y side (side windows 248 / 270 deg)"),
    }
    m0 = CAD["totals"]["mass_g"]
    cx0, cy0, cz0 = CAD["totals"]["com_mm"]
    mass = 84.0
    others = {k: v["bbox_mm"] for k, v in parts.items()
              if k in ("battery", "esc", "misc_electronics", "avionics_tray", "carrier")}
    gps_lead = PARAMS["configs"]["vtol"]["top_cap"]["gps"]["lead_hole"]
    glx = gps_lead["r"] * math.cos(math.radians(gps_lead["theta_deg"]))
    gly = gps_lead["r"] * math.sin(math.radians(gps_lead["theta_deg"]))
    out = {}
    for name, (xs, ys, zs, where) in spots.items():
        corner_r = max(math.hypot(x, y) for x in xs for y in ys)
        overlaps = []
        for k, bb in others.items():
            if (xs[0] < bb[1][0] and xs[1] > bb[0][0] and ys[0] < bb[1][1] and ys[1] > bb[0][1]
                    and zs[0] < bb[1][2] and zs[1] > bb[0][2]):
                overlaps.append(k)
        xc, yc, zc = sum(xs) / 2, sum(ys) / 2, sum(zs) / 2
        nx = (m0 * cx0 + mass * xc) / (m0 + mass)
        ny = (m0 * cy0 + mass * yc) / (m0 + mass)
        nz = (m0 * cz0 + mass * zc) / (m0 + mass)
        gps_hit = xs[0] <= glx <= xs[1] and ys[0] <= gly <= ys[1]
        out[name] = {"where": where, "box_mm": {"x": [round(v, 2) for v in xs], "y": [round(v, 2) for v in ys],
                                                "z": [round(v, 2) for v in zs]},
                     "corner_r_mm": round(corner_r, 2), "bore_clearance_mm": round(bore_r - corner_r, 2),
                     "overlaps_cad_parts": overlaps, "gps_lead_drop_inside": gps_hit,
                     "cg_after_mm": [round(nx, 2), round(ny, 2), round(nz, 2)],
                     "lateral_cg_off_axis_mm": round(math.hypot(nx, ny), 2)}
    return {"label": "E (computed by scripts/build_spec.py from the CAD report boxes; pack = rigid box, nothing drawn)",
            "pack_mm": [t, w, length], "mass_g": mass, "bay_bore_r_mm": bore_r,
            "vehicle_now": {"mass_g": round(m0, 1), "cg_mm": [round(cx0, 2), round(cy0, 2), round(cz0, 2)],
                            "lateral_cg_off_axis_mm": round(math.hypot(cx0, cy0), 2),
                            "source": f"{CADR} totals; {CADC}:41 (C33 WARN: CG 1.34 mm off axis, trim 7.1 %)"},
            "gps_lead_hole_xy_mm": [round(glx, 1), round(gly, 1)],
            "spots": out}


# ----------------------------------------------------------------------------------------------- part sources (BOM)
SRC_FILE = HERE / "spec" / "parts_sources.json"
# LCSC package text that must appear for the footprint used (catches a wrong-size substitute)
PKG_EXPECT = {"r10k": "0603", "r1k": "0603", "r4k7": "0603", "r47k": "0603", "r220": "0603", "r100k": "0603",
              "c100n0402": "0402", "c22p0402": "0402", "c27p0402": "0402", "c100n0603": "0603", "c10u0805": "0805", "c22u1206": "1206",
              "tant22u": "3216-18", "tant100u": "3528-21", "elec220u": "6.3xL5.4", "ind10u": "0805", "led": "0603",
              "schottky": "SOD-123", "bss138": "SOT-23", "pfet": "SOT-23", "npn": "SOT-23", "ldo": "SOT-223",
              "mcu": "TQFP-64", "xtal": "3225", "esp": "24x16", "gh4": "1.25", "ph2": "2mm", "spox2": "2.5mm",
              "hdr2x3": "2.54", "hdr1x2v": "2.54", "hdr1x6v": "2.54", "hdr1x3ra": "2.54", "hdr1x2ra": "2.54",
              "hdr1x4ra": "2.54", "sock1x6ra": "2.54"}
NO_BOM = {"NT1": "net tie: copper only, no part", "JP1": "solder jumper: copper only (bridged 1-2 in the copper)"}
# lead / mating / loose items that have no footprint of their own: (key, qty, for)
EXTRA_BOM = [
    ("shunt", 4, "jumper shunts for J24, J25, J26 (reset holds, fitted only while programming) and J3 pins 1-2 (ESP "
                 "download mode); J15 (JST-PH) is bridged on the bench with the switch lead closed"),
    ("bmp280", 1, "plugs into J2 with its own right-angle header"),
    ("openlog", 1, "plugs into J5, lies flat"),
    ("hdr1x6ra", 1, "1x6 right-angle male header soldered on the OpenLog (so it lies flat in J5)"),
    ("neo7m", 1, "GPS module in the top cap (not on the carrier)"),
    ("ghr4", 2, "GPS lead housings (carrier end J7 + NEO-7M end if it has a GH socket; else a 2.54 Dupont end, A)"),
    ("ghcrimp", 8, "GPS lead crimp contacts"),
    ("phr2", 2, "JST-PH 2-pin housings: buzzer lead (mates J17) + power-switch lead (mates J15)"),
    ("phcrimp", 4, "JST-PH crimp contacts (buzzer lead 2 + switch lead 2)"),
    ("toggle", 1, "logic power switch; its Dupont lead end is cut and re-terminated with a JST-PH housing (phr2 + "
                  "2 phcrimp) to plug onto J15 (switch reachability = user decision)"),
    ("logic_battery", 1, "1S logic pack, its own 5264-type plug mates J14"),
]


def bom_key(p: dict) -> str | None:
    fp, v, ref = p["footprint"], p["value"], p["refdes"]
    if ref in NO_BOM:
        return None
    if ref == "J1":
        return "gy521"            # the GY-521's own straight 1x8 header is soldered into J1
    if ref == "J6":
        return "boost"            # the Pololu module's own straight header strip is soldered into J6
    table = [("TQFP-64", "mcu"), ("SOT-223", "ldo"), ("RF_Module:ESP-12E", "esp"), ("Crystal_SMD_3225", "xtal"),
             ("D_SOD-123", "schottky"), ("LED_0603", "led"), ("L_0805", "ind10u"), ("Kemet-A", "tant22u"),
             ("Kemet-B", "tant100u"), ("CP_Elec", "elec220u"), ("PinHeader_2x03", "hdr2x3"),
             ("PinHeader_1x02_P2.54mm_Vertical", "hdr1x2v"), ("PinHeader_1x06_P2.54mm_Vertical", "hdr1x6v"),
             ("PinHeader_1x03_P2.54mm_Horizontal", "hdr1x3ra"), ("PinHeader_1x02_P2.54mm_Horizontal", "hdr1x2ra"),
             ("PinHeader_1x04_P2.54mm_Horizontal", "hdr1x4ra"), ("PinSocket_1x06_P2.54mm_Horizontal", "sock1x6ra"),
             ("Molex_SPOX", "spox2"), ("JST_GH", "gh4"), ("JST_PH", "ph2")]
    for frag, key in table:
        if frag in fp:
            return key
    if p["symbol"] == "Transistor_FET:BSS138":
        return "bss138"
    if p["symbol"] == "Transistor_FET:AO3401A":
        return "pfet"
    if p["symbol"] == "Transistor_BJT:MMBT3904":
        return "npn"
    if p["symbol"] == "Device:R":
        val = v.split()[0]
        return {"10k": "r10k", "1k": "r1k", "4.7k": "r4k7", "47k": "r47k", "220": "r220", "100k": "r100k"}.get(val)
    if p["symbol"] == "Device:C":
        size = re.search(r"C_(\d{4})_", fp).group(1)
        return {("0402", "100n"): "c100n0402", ("0402", "22p"): "c22p0402", ("0402", "27p"): "c27p0402",
                ("0603", "100n"): "c100n0603",
                ("0805", "10u"): "c10u0805", ("1206", "22u"): "c22u1206"}.get((size, v.split()[0]))
    return None


def sourcing(parts: list[dict]) -> dict:
    """Attach the online-verified source to every part; build the JLC BOM view and the devicemart purchase list."""
    if not SRC_FILE.exists():
        ERRORS.append(f"{SRC_FILE} missing - run scripts/fetch_parts_sources.py (online, read-only)")
        return {}
    src = json.loads(SRC_FILE.read_text(encoding="utf-8"))
    for e in src.get("errors", []):
        ERRORS.append(f"parts source error (fetch_parts_sources.py): {e}")
    allsrc = {**src["lcsc"], **src["devicemart"]}
    lines: dict[str, dict] = {}
    for p in parts:
        key = bom_key(p)
        if key is None:
            if p["refdes"] in NO_BOM:
                p["bom"] = {"line": None, "why": NO_BOM[p["refdes"]]}
                continue
            ERRORS.append(f"{p['refdes']} ({p['value']}, {p['footprint']}): no BOM source mapping")
            continue
        s = allsrc.get(key)
        if s is None or not s.get("found"):
            ERRORS.append(f"{p['refdes']}: source '{key}' not verified online")
            continue
        m_size = re.search(r"_(\d{4})_\d{4}Metric", p["footprint"])   # chip size read from the FOOTPRINT name
        want = m_size.group(1) if m_size else PKG_EXPECT.get(key)
        if want and want not in (s.get("package") or ""):
            ERRORS.append(f"{p['refdes']}: source {s['part_number']} package {s.get('package')!r} does not match "
                          f"footprint {p['footprint']} (expected '{want}')")
        smd = s["source"] == "JLCPCB/LCSC" and not (s.get("package") or "").startswith(("Plugin", "Push-Pull"))
        assembly = ("JLC SMT assembly" if smd else
                    "hand-solder (THT; JLC THT assembly optional)" if s["source"] == "JLCPCB/LCSC" else
                    "hand-solder (devicemart module)")
        p["bom"] = {"line": key, "source": s["source"], "part_number": s["part_number"],
                    "mpn": s.get("mpn") or s.get("product_name"), "jlc_type": s.get("jlc_library_type"),
                    "assembly": assembly}
        ln = lines.setdefault(key, {"line": key, "refs": [], "dnp_refs": [], "qty_populated": 0,
                                    "value": p["value"], "footprint": p["footprint"], **p["bom"]})
        if p["populate"] is True:
            ln["refs"].append(p["refdes"])
            ln["qty_populated"] += 1
        else:
            ln["dnp_refs"].append(p["refdes"])
    extra = []
    for key, qty, why in EXTRA_BOM:
        s = allsrc.get(key)
        if s is None or not s.get("found"):
            ERRORS.append(f"extra BOM item '{key}' not verified online")
            continue
        extra.append({"line": key, "qty": qty, "for": why, "source": s["source"], "part_number": s["part_number"],
                      "mpn": s.get("mpn") or s.get("product_name"), "jlc_type": s.get("jlc_library_type")})
    board_lines = sorted(lines.values(), key=lambda d: natural(d["refs"][0] if d["refs"] else d["dnp_refs"][0]))
    jlc = [d for d in board_lines if d["source"] == "JLCPCB/LCSC"]
    dm_board = [d for d in board_lines if d["source"] == "devicemart"]
    detail = {k: {kk: v[kk] for kk in ("source", "part_number", "for", "urls", "fetched", "raw_sha256") if kk in v}
              | {"mpn_or_name": v.get("mpn") or v.get("product_name"), "package": v.get("package"),
                 "stock": v.get("stock", (v.get("search_record") or {}).get("stock_field")),
                 "status": v.get("jlc_library_type") or (v.get("search_record") or {}).get("goods_status"),
                 "price_krw": (v.get("search_record") or {}).get("sale_price_krw")}
              for k, v in sorted(allsrc.items())}
    return {
        "rule": "user 2026-11-05: every part from the JLCPCB parts library (LCSC C-number) or devicemart.co.kr (no=); "
                "verified online by scripts/fetch_parts_sources.py (read-only GETs, no login / cart / purchase)",
        "fetched": src.get("fetched"), "evidence": "spec/parts_sources.json; raw pages out/reports/sourcing/",
        "jlc_bom_columns": ["Comment", "Designator", "Footprint", "LCSC Part #"],
        "board_lines": board_lines,
        "jlc_lines": len(jlc), "jlc_basic_lines": sum(1 for d in jlc if d["jlc_type"] == "Basic"),
        "jlc_extended_lines": sum(1 for d in jlc if d["jlc_type"] == "Extended"),
        "devicemart_on_board": dm_board,
        "extra_items": extra,
        "devicemart_purchase_list": [d for d in extra if d["source"] == "devicemart"] +
                                    [{"line": d["line"], "qty": d["qty_populated"], "for": ", ".join(d["refs"]),
                                      "source": d["source"], "part_number": d["part_number"], "mpn": d["mpn"]}
                                     for d in dm_board],
        "lcsc_loose_items": [d for d in extra if d["source"] == "JLCPCB/LCSC"],
        "alternatives": {"esp_alt": "devicemart no=1378118 '[정품] ESP8266 ESP-12E' if U7 is hand-soldered instead of "
                                    "JLC-assembled"},
        "sources_detail": detail,
        "notes": ["JLC 'Extended' lines add a per-line loading fee at assembly (A: JLC policy, not fetched)",
                  "THT headers / sockets / SPOX / PH: hand-soldered after JLC SMT assembly (or JLC THT assembly)",
                  "devicemart 'stock' field meaning is undocumented; goods_status 'normal' = on sale at fetch time",
                  "stock numbers change daily - re-run scripts/fetch_parts_sources.py before ordering"],
    }


# ----------------------------------------------------------------------------------------------- narrative sections
def narrative(spots: dict) -> dict:
    return {
        "board": {
            "outline_mm": {"x": [-38.0, 38.0], "y": [-35.0, 35.0], "size": [76.0, 70.0], "corner_radius_mm": 1.0,
                           "label": "F (size) / A (corner radius)",
                           "source": f"{P} configs.vtol.electronics.carrier.dims_lwh [76, 70, 15] (line 1758-1762); "
                                     f"{CADR} parts.carrier.bbox_mm [[-38,-35,327.1],[38,35,342.1]]"},
            "frame": "board origin = vehicle axis; board +x = CAD +x (ESC / misc side, motor-lead hole at theta 0 deg), "
                     "+y = CAD +y (theta 90 deg); component side = top = +Z (up, toward the tray). "
                     "Source: cad/layout.py (carrier centred, edges along x / y); "
                     f"{CADR} parts.carrier.bbox_mm",
            "layers": 2, "thickness_mm": 1.6, "copper": "1 oz (A)",
            "stackup_note": f"user chose 2 layers first ({P} carrier.note line 1785 left 4 layers to the PCB round): "
                            "signals mainly on F.Cu, B.Cu mostly a solid GND pour (A)",
            "layer_plan": {
                "decision": f"{USER5} ('다시 돌리다가 답이 안보이면 4층 기판으로 바꿔'): try 2 layers WITHOUT the 5th chip "
                            "first; if routing does not reach 0 unconnected within ~90 min of routing attempts, switch "
                            "to 4 layers. Never loosen the fab rules or the rev-1 lessons to force a 2-layer result.",
                "two_layer_rules": "JLCPCB 2-layer standard: track / space >= 0.127 mm used as minimum (wider classes "
                                   "for power), vias 0.3 / 0.6 mm (A: JLC capability, as in scripts/build_schematic.py)",
                "four_layer_fallback": {
                    "stackup": ["F.Cu: signals + parts", "In1.Cu: solid GND plane (no tracks)",
                                "In2.Cu: power - +5V, +3V3 and SERVO_5V as pours / wide tracks (GND_PWR trunk on an "
                                "edge, see grounding)", "B.Cu: signals (+ GND fill)"],
                    "board": "JLCPCB standard 4-layer, 1.6 mm, 1 oz outer / 0.5 oz inner (A: JLC default; name the "
                             "stackup in the order)",
                    "rules": "keep the 2-layer track / space / via rules where possible (JLC 4-layer allows 0.09 mm, "
                             "A - not used unless needed); vias 0.3 / 0.6 mm through-hole only, no blind / buried",
                    "keepouts": "the ESP-12E antenna keep-out applies to ALL four layers (no inner plane under the "
                                "antenna)"},
                "cost_note": "4 layers cost noticeably more than 2 layers at JLCPCB for a 76 x 70 mm board (E; prices "
                             "not fetched) - reported to the user with the routed result",
                "freed_area": "the removed 5th chip block (~20 x 20 mm incl. its crystal, caps and ISP header, E) is "
                              "free for routing / the boost module"},
            "height_limit_top_mm": {"value": 13.4, "label": "F/E",
                                    "why": "carrier envelope 15 mm incl. components minus 1.6 mm board "
                                           f"({P} carrier.dims_lwh; {CADR} carrier z 327.1-342.1, tray bottom z 344.1)"},
            "bottom_side": "no components: the board sits on 3 mm double-sided foam tape (carrier.foam_t, line 1763); THT "
                           f"leads <= 1.5 mm under the board, flush-cut to ~1.0 mm recommended ({REV} MF-8)",
            "mounting": {"holes": "none",
                         "method": f"3 mm foam tape + ONE cable tie across the board top between the intake-bulkhead "
                                   f"vent holes at 90 / 270 deg (d 8 mm at r 40: centres (0, +40) / (0, -40)) - {P} "
                                   "configs.vtol.electronics.carrier.note (line 1785), configs.vtol.intake_module "
                                   "(line 1686-1691)",
                         "tie_band": {"x_mm": [-3.0, 3.0], "y_mm": [-35.0, 35.0],
                                      "rule": "no components inside the band (traces / vias allowed); A width for a 2.5-3.6 mm tie"}},
            "keepouts": [
                {"what": "motor-lead bulkhead hole 20 x 11 at r 42-53, theta 0 deg + bullet joint zone (4 mm from the +x "
                         "edge)", "rule": "+x edge, |y| < 14 mm: no lead connectors, no I2C / radio / crystal parts (A, "
                                          "EMI from the 100 A motor leads)",
                 "source": f"{P} intake_module.wire_channel.bulkhead_hole_* (line 824-850); motor_leads.joint_zone (line "
                           f"423); {CADR} extra.motor_leads.joint_zone_clearance_mm.carrier = 4.0"},
                {"what": "ESP-12E antenna", "rule": "footprint keep-out (all copper layers, 16.24 x 7.32 mm) at the +y "
                         "edge + a rule area extending 4 mm beyond both module sides along the edge: no copper on either "
                         "layer, no parts, no GND stitching vias (A; Espressif / Ai-Thinker ask for the antenna to "
                         "overhang the board or for a copper-free area, ideally >= 15 mm clear of metal - check)",
                 "source": ESPLIB},
                {"what": "board corners 5.3 mm inside the intake spigot bore r 57.0", "rule": "no part overhangs the "
                         "outline; mated plug bodies checked against the bore and the joint screws in a script "
                         f"({REV} MF-2 / MF-6 / MF-7)", "source": f"{P} carrier.note (line 1785); {CADR} "
                                                                    "extra.vtol.carrier.clearance_mm"},
                {"what": "tray bottom z 344.1 (2 mm above the carrier envelope)", "rule": "13.4 mm top height limit "
                         "everywhere", "source": f"{CADR} parts.avionics_tray.bbox_mm"},
                {"what": "reserved module outlines", "rule": "no parts under: OpenLog (19 x 16, J5, A), BMP280 module "
                         "(J2, A), GY-521 (21 x 16 + support pad, J1, A), Pololu S9V11F5S6CMA (12.7 x 15.3 x 6.4 mm, "
                         "J6, vendor page; its trimmer must stay reachable from above) - measure the bought modules",
                 "source": f"{REV} MF-1, MF-9; pololu.com/product/2870"},
            ],
            "edges": [
                {"edge": "-x (x = -38)", "connectors": ["J8", "J9", "J10", "J11"],
                 "why": f"vane-servo leads enter the bay through the bulkhead servo hole at theta 180 deg "
                        f"({P} servo_leads.gather_theta_deg / bulkhead_hole_r, line 763-768); right-angle headers facing "
                        f"-x. The top slot, whose mated plug would touch the intake spigot bore (r 57.17 > 57.0, E), "
                        f"carries no right-angle header ({REV} MF-2): the DNP release header J13 is removed (rev 2c, "
                        f"see removed_release_header) and the -x / +y corner takes the vertical J14 / J15"},
                {"edge": "-x, y < -12", "connectors": ["J18 (6S sense)"],
                 "why": f"power-sense lead: 6S pack on the tray's -x face ({CADR} parts.battery x -37.4..13.6), tether / "
                        f"hybrid kit boxes on the -x side ({P} power.tether_kit_ASSUMED.boxes line 1912) - A"},
                {"edge": "+x (x = +38), y > +14", "connectors": ["J12 (ESC lead + NT1 tie, upper slot)",
                                                                  "J17 (buzzer, below)"],
                 "why": f"ESC on the tray's +x face, misc box (buzzer) above it ({CADR} parts.esc x 15.6..24.6, "
                        "misc_electronics x 15.6..27.6); |y| < 14 kept clear for the motor leads; J12's mated plug >= "
                        f"2.0 mm from the motor-lead bullet zone ({REV} MF-3)"},
                {"edge": "-x / +y corner (x < -25, y > +18)", "connectors": ["J14 (logic battery, vertical)",
                                                                              "J15 (power-switch lead, vertical)"],
                 "why": f"{USER5} 'battery A (6S pack -x face)': the 1S pack (power.battery_spots A, x -48.4..-37.9, "
                        "y -27.75..+27.75, z 346.1..413.1, E) sits straight above the -x edge; with its lead end at +y "
                        "the lead drops ~6 mm in plan to J14 without crossing the board top, the servo leads (they leave "
                        "the -x edge at y -12..+22 toward the theta-180 hole) or the ESC / motor leads (+x side); "
                        f"vertical plugs only (a right-angle plug in this corner would hit the bore, {REV} MF-2); "
                        f"{REV2} PF-M4 / MF2-1 (rev 2b had them in the +x / +y corner)"},
                {"edge": "+y (y = +35), x -24..-8", "connectors": ["U7 ESP-12E antenna end (flush with the edge)"],
                 "why": "antenna faces theta 90 deg: the side windows at 90 / 112 deg are just above the carrier "
                        f"({P} configs.vtol.avionics_module.windows.extra_thetas_deg line 1724-1729); no metal straight "
                        "above it (the 6S pack ends at y +17.5); far from the ESC (+x), the motor leads (0 deg) and the "
                        "servo headers (-x edge); x <= -8 keeps the tie band (|x| < 3) and the vent hole (0, +40) clear "
                        "(A, placement confirms)"},
                {"edge": "+y (y = +35), x > +5", "connectors": ["J16 (arm switch)"],
                 "why": f"arming switch in the top cap at theta 90 deg, r 40 ({P} configs.vtol.top_cap.switch_hole, "
                        "line 1852-1857): its lead drops along the +y wall"},
                {"edge": "-y (y = -35), x < -5", "connectors": ["J7 (GPS, JST-GH vertical, plugged from above)"],
                 "why": f"GPS lead drops through the top-cap lead hole at theta 230 deg, r 37 = (x -23.8, y -28.3) "
                        f"({P} configs.vtol.top_cap.gps.lead_hole, line 1840-1844)"},
                {"edge": "interior (top)", "connectors": ["J19-J22 ISP", "J24 / J25 / J26 reset-hold shunts",
                                                          "J3 ESP flash header", "J5 OpenLog (flat)", "J1 GY-521",
                                                          "J2 BMP280", "J6 5 V module (Pololu, flat)"],
                 "why": f"ISP / flashing only with the avionics module lifted off the intake ({RM}:655-659); GY-521 near "
                        "the board centre (vibration, CG), outside the tie band (A)"},
            ],
        },
        "power": {
            "decisions": [f"{USER4} ('전원은 칩셋 전원용 배터리를 하나 추가한다고 가정하고 진행해줘'): logic on ONE added "
                          "battery, NO 6S-to-logic converter (the '5 V logic UBEC' from the 6S of "
                          f"{RM}:212, 244 is dropped)",
                          f"{USER5} ('배터리는 이 모델', devicemart no=16021596): 1S Li-Po DTP105565P 3.7 V 4200 mAh "
                          "replaces the rev-1 2S 300 mAh assumption -> 5 V BOOST stage"],
            "battery": {
                "model": "DTP105565P (3.7 V 4200 mAh) C5264RR", "cells": "1S Li-Po", "v_nominal": 3.7, "v_full": 4.2,
                "capacity_mAh": 4200, "dims_mm": [10.5, 55.5, 67.0], "mass_g": 84.0,
                "label": f"F ({BAT}) for the values above",
                "assumed": ["'P' suffix read as a built-in protection circuit (PCM) with over-discharge cut-off ~2.5-3.0 V "
                            "and over-current cut-off (A, not on the product page - measure / ask the seller)",
                            "'C5264RR' read as a Molex 5264-type 2.50 mm 2-pin housing, red / red-black leads (A) - "
                            "check the plug and its polarity on the bought pack before soldering J14"]},
            "tree": [
                "J14 logic battery (1S, 3.0-4.2 V) -> Q10 AO3401A reverse-polarity P-FET -> VBAT_LOGIC -> J15 switch lead "
                "-> VBAT_LOGIC_SW -> 2 x 22 uF -> J6 pin 3 VIN of the Pololu S9V11F5S6CMA (buck-boost, fixed 5 V, "
                "adjustable low-VIN cut-off) -> J6 pin 1 VOUT -> +5V",
                "+5V -> 4 x ATmega128A VCC / AVCC (LC), GY-521 VCC, OpenLog VCC, buzzer, shifter HV pull-ups, U6 "
                "AMS1117-3.3 input",
                "+5V -> U6 AMS1117-3.3 -> +3V3 -> ESP-12E VCC (100 uF + 10 uF + 100 nF at the pin), BMP280 VCC, GPS VCC "
                "(JP1 default), BSS138 gates + LV pull-ups, ESP strap pull-ups",
                "VBAT_LOGIC_SW -> 47k / 47k -> FCU PF3 (logic battery monitor, populated)",
                f"ESC BEC 5 V / 7 A (J12 pin 2) -> SERVO_5V -> J8-J11 (+ J13 DNP) pin 2 - separate from +5V ({RM}:212, "
                f"322-324; {P} electronics.esc line 943): servos are NOT on the logic battery",
                "6S propulsion bus is only SENSED (J18 -> 47k / 4.7k dividers -> FCU PF0 / PF1 / PF2), never powers the "
                "logic"],
            "input_range_V": {"logic_battery_1S": [3.0, 4.2],
                              "label": "F (Li-Po 4.2 V full) / A (3.0 V usable end; the PCM may cut at 2.5-3.0 V, A)",
                              "propulsion_sensed": [0.0, 28.16],
                              "propulsion_note": f"divider full scale {P} power.electrical_plant_ASSUMED.adc_full_scale_v "
                                                 f"28.16 (line 2028); 6S max 25.2 V ({RM}:205)"},
            "load_estimate": {
                "label": "E (datasheet-class values marked A; nothing measured)",
                "rail_3V3_mA": {"ESP-12E (Wi-Fi associated, UART bridge: ~80 avg; TX bursts / RF calibration up to ~400, "
                                "A Espressif-class figures)": [80, 400],
                                f"NEO-7M board ({P} electronics.gps.power ~45 mA, line 1012; NEO-6M value, A)": 45,
                                "BMP280 module (A)": 1, "shifter / I2C / strap pull-ups (E)": 2},
                "rail_3V3_total_mA": {"typical": 128, "peak": 448},
                "rail_5V_mA": {"ATmega128A x4 @16 MHz (25 mA each, A)": 100, "status LEDs x4 (3 mA)": 12,
                               "GY-521 (MPU-6050 3.9 mA + LED + LDO, A)": 6, "OpenLog (avg 20 / peak 80, A)": [20, 80],
                               "buzzer (A)": [0, 30], "AMS1117 quiescent (A)": 5, "HV pull-ups (E)": 2,
                               "3.3 V rail (above)": [128, 448]},
                "rail_5V_total_mA": {"typical": 273, "peak": 683},
                "boost": {"module": "Pololu S9V11F5S6CMA #2870 (devicemart no=15855364): buck-boost 5 V, 1.7 A typ. "
                                    "max for inputs near the output, VIN 2-16 V after a 3 V start, efficiency 85-95 % "
                                    "typ. (vendor page, read 2026-11-05)",
                          "efficiency_A": 0.85,
                          "input_current_A": {"typical_at_3.7V": 0.43, "typical_at_3.0V": 0.54, "peak_at_3.0V": 1.34,
                                              "peak_at_4.2V": 0.96},
                          "formula": "I_in = 5.0 V x I_5V / 0.85 / V_cell (E)",
                          "module_rating_needed": "0.68 A at 5 V peak from 3.0-4.2 V in (E). The vendor's 1.7 A is "
                                                  "for VIN near 5 V; at a 3.0-3.7 V input the continuous limit is "
                                                  "lower (vendor graph not read: A ~1 A) - margin ~1.5x at the peak "
                                                  "(E), measure the 5 V droop during an ESP TX burst"},
                "ams1117_dissipation_W": {"typical": 0.22, "peak_burst": 0.76,
                                          "formula": "(5.0 - 3.3) x I_3V3 (E); SOT-223 on its pour: ~+20 C at the typical "
                                                     "load (E, theta_JA ~90 C/W A); dropout ~1.0-1.1 V at 0.45 A (A) < 1.7 V "
                                                     "headroom"},
                "q10_drop_V": "<= 0.11 V at 1.34 A (85 mOhm at Vgs -2.5 V, A); J15 switch + leads ~0.05 V (E)",
                "runtime_h": {"typical": 7.7,
                              "formula": "4200 mAh x 0.8 usable (A) / 0.434 A (E) at 3.7 V nominal; Wi-Fi TX duty "
                                         "raises the mean - budget >= 5 h (E) before recharge",
                              "vs_flight": f"{P} mission.vtol.max_flight_s 40 / hard_cutoff_s 60 (line 1253-1255): the "
                                           "pack is never the endurance limit; it must be CHARGED before a flight day"},
                "c_rate": "peak 1.34 A = 0.32 C of 4200 mAh (E)",
                "servo_rail": "not on the logic battery (ESC BEC 5 V / 7 A)",
            },
            "over_discharge": {
                "risk": "boost converters keep running down to ~2 V input (the Pololu S9V11 runs to 2 V after start-up, "
                        "vendor page): without a cut-off the cell can be pulled below its safe end (Li-Po damage) - and "
                        "if any cut-off acts in flight, ALL four chips reset (loss of control)",
                "protection_in_this_design": [
                    "HARDWARE cut-off in the 5 V module: the S9V11F5S6CMA's EN pin sits on an 11-turn pot to VIN "
                    "(vendor: off below EN 0.7 V, on above 0.8 V, sleep draw ~7 uA per volt of VIN). Set it to 3.2 V "
                    "(A: protects the cell, ~3.65 V needed to restart = 114 %, vendor) before the first flight; it "
                    "is the LAST line, not a flight-time alarm",
                    "the pack's PCM (A, assumed from the 'P' suffix: cut-off ~2.5-3.0 V) - MEASURE it with a bench load "
                    "before relying on it",
                    "logic-battery monitor on FCU PF3 POPULATED (47k / 47k, 2.10 V at 4.2 V) - firmware does not read it "
                    "yet (follow-up: telemetry + refuse to ARM below ~3.6 V / land below ~3.4 V, A thresholds)",
                    "J15 switch after Q10: with the switch off nothing on the board drains the pack (boost module and "
                    "divider are after the switch)",
                    "procedure (until the firmware alarm exists): measure the pack before each flight, >= 3.8 V "
                    "(E: >= ~50 % charge)"],
                "not_added": "no separate supervisor / load switch: the module's own cut-off has 14 % hysteresis "
                             "(vendor), which is larger than the pack's IR rebound at 1.3 A (E: ~0.05-0.1 V with ~40-70 "
                             "mOhm, A) - no on/off cycling expected; measure",
                "sag_check": "a 3.2 V cut-off with 1.34 A peak and ~50 mOhm pack + 0.11 V Q10 + 0.05 V switch/leads "
                             "(E) trips when the open-circuit cell is ~3.4 V: the pre-flight check (>= 3.8 V) keeps a "
                             "~0.4 V margin (E)",
            },
            "phantom_power_through_6S_sense": {
                "finding": f"{REV} E9",
                "estimate": "logic off, 6S plugged: V_in divider node 25.2 x 4.7 / 51.7 = 2.29 V behind 4.27 kOhm -> "
                            "PF0 clamp diode into the dead +5V rail ~(2.29 - 0.5) / 4.27k = 0.42 mA (E); below the ~1 mA "
                            "clamp guidance (A); the 5 V module (synchronous buck-boost) may pass some of it to its input side (A). Rail may float "
                            "~1-2 V with every chip held by BOD (4.0 V) - no damage expected (E). Back-feed into the "
                            "Pololu VOUT while it is unpowered: not specified on the vendor page (A: sub-mA through "
                            "its output stage is harmless) - follow the procedure",
                "mitigation": "procedure: switch the logic ON before plugging the 6S, unplug the 6S first (report; no "
                              "part added)"},
            "switch_reachability": {
                "finding": f"{REV} MF-5 (rev-1 J15 unreachable with the vehicle closed)",
                "fact": f"the carrier sits at z 327.1-342.1 inside the intake spigot; the bay windows start above it (z >= "
                        f"343.1, {P} configs.vtol.avionics_module.windows.z_above_lower_joint line 1732) - J14 / J15 on the "
                        "board cannot be reached with the vehicle closed (E from the CAD boxes)",
                "options_user_decision": [
                    "(a) J15 lead (~150-250 mm, A) to a small toggle in a 3D-printed clip at a side window (90 or 112 "
                    "deg, w 16 mm) - CAD change",
                    "(b) a second toggle hole in the top cap beside the arm switch - CAD change (new hole)",
                    "(c) no switch: J15 shunt fitted, the pack's own plug is the switch - place the pack (battery_spots) "
                    "so its plug sits behind a side window and unplug it there (no CAD change if the spot allows; "
                    "measure)"]},
            "ground": {"single_tie_point": "NT1 at J12 (ESC lead) pin 3 - see grounding"},
            "mass": {
                "battery_g": 84.0, "label": "F (vendor 84 g) / E (totals)",
                "not_in_params": True,
                "vtol_battery_config_now_g": 1653.6, "target_g": 1650.0,
                "source": f"{HW}:438 (VTOL battery 1,653.6 g, 3.6 g over 1,650); {CADR} totals.mass_g 1653.7",
                "with_pack_g": 1737.6, "over_target_g": 87.6,
                "possible_offsets_E": ["carrier: params' 50 g ASSUMED breakdown holds 'lora_module_and_antenna' 14.0 g "
                                       "(line 1771); an ESP-12E + caps ~2.5 g (A) -> ~-11 g if the board is weighed",
                                       "misc box 25 g ASSUMED lump includes the '5 V logic UBEC' (line 1788) that this "
                                       "design does not use -> a few g (E)"],
                "decision": f"{USER5}: proceed with the 84 g pack; the overrun is a user decision for later (params.json "
                            "not edited)"},
            "battery_spots": spots,
        },
        "grounding": {
            "nets": {"GND": "logic ground (logic battery -, all chips, modules, dividers, arm switch, buzzer, GPS, ESP)",
                     "GND_PWR": "propulsion-side ground on the carrier: J12 ESC lead pin 3 (= ESC BEC ground = 6S -), "
                                "servo headers pin 3, servo bulk capacitor"},
            "single_tie_point": "NT1 net tie placed at J12 (ESC lead) pin 3. It is the ONLY connection between the logic "
                                "battery's ground and the propulsion side. Reason: the ESC / servo PWM signals need a "
                                f"common reference ({RM}:247, 323; {BH}:84, 91, 107 'common ground')",
            "rules": ["J18 (power sense) has NO ground pin: the dividers return to logic GND and see the bus negative "
                      "through NT1",
                      "servo return currents flow J8-J13 pin 3 -> GND_PWR -> J12 pin 3, never through the logic GND",
                      "with the ESC lead unplugged the logic ground floats from the propulsion side - bench-safe",
                      "measurement error: V_in sees the drop of the ESC's internal negative path (E: < 0.1 V at 100 A)"],
        },
        "level_shifting": {
            "principle": f"every MCU <-> 3.3 V-only device signal through a BSS138 channel, BOTH directions this time "
                         f"(rev-1 left module TX -> MCU RX direct: {REV} E2); {RM}:254-255; docs/SPEC.md:1178",
            "channels": [
                {"q": "Q1", "signal": "NAV PD0 SCL <-> GY-521 / BMP280 SCL", "pullups": "4.7k (5 V) / 10k (3.3 V)", "populate": True},
                {"q": "Q2", "signal": "NAV PD1 SDA <-> GY-521 / BMP280 SDA", "pullups": "4.7k (5 V) / 10k (3.3 V)", "populate": True},
                {"q": "Q3", "signal": "GY-521 INT -> NAV PE6 (firmware polls)", "pullups": "10k (fitted) / 10k (DNP)", "populate": "optional (DNP)"},
                {"q": "Q4", "signal": "COM PD3 TXD1 -> ESP RXD0 (GPIO3) via 1k", "pullups": "10k / 10k", "populate": True},
                {"q": "Q5", "signal": "COM PC6 -> ESP RST (LV pull-up = RST pull-up)", "pullups": "10k / 10k", "populate": True},
                {"q": "Q6", "signal": "GNS PD3 TXD1 -> GPS RXD", "pullups": "10k / 10k", "populate": True},
                {"q": "Q7", "signal": "GPS TXD -> GNS PD2 RXD1", "pullups": "10k / 10k", "populate": True},
                {"q": "Q8", "signal": "FCU PE1 TXD0 -> OpenLog RXI", "pullups": "10k / 10k", "populate": True},
                {"q": "Q11", "signal": "ESP TXD0 (GPIO1) -> COM PD2 RXD1 (NEW)", "pullups": "10k / 10k", "populate": True},
            ],
            "count": {"populated": 8, "optional": 1, "total_footprints": 9},
            "speed_note": "10k pull-ups: rise ~0.5 us with ~20 pF (E) - fine up to 115200 baud (8.7 us bit); the I2C "
                          "pair uses 4.7k on the 5 V side",
            "no_shifter_links": f"nlink, ilink, glink: all chips at 5 V ({BH}:84, 92, 107)",
        },
        "esp12e": {
            "part": "ESP-12E (Ai-Thinker class, ESP8266EX + SPI flash + PCB antenna), soldered (castellated pads)",
            "library": ESPLIB,
            "pins_used": {"1 ~RST": "ESP_RST: Q5 LV (10k pull-up to 3V3) <- COM PC6; J3 pin 5",
                          "3 EN": "10k to 3V3 + 100 nF to GND",
                          "8 VCC": "+3V3 with 100 uF tantalum + 10 uF + 100 nF at the pin", "15 GND": "GND",
                          "16 GPIO15": "10k to GND", "17 GPIO2": "10k to 3V3", "18 GPIO0": "10k to 3V3; J3 pin 1",
                          "21 GPIO3/RXD": "ESP_RXD0 <- 1k <- Q4 <- COM PD3 TXD1; J3 pin 4",
                          "22 GPIO1/TXD": "ESP_TXD0 -> Q11 -> COM PD2 RXD1; J3 pin 3"},
            "pins_open": "2 ADC, 4 GPIO16, 5 GPIO14, 6 GPIO12, 7 GPIO13, 19 GPIO4, 20 GPIO5; 9-14 flash pins (must "
                         "stay open)",
            "boot_modes": "reset with GPIO0 high / GPIO2 high / GPIO15 low = run from flash; GPIO0 low (J3 shunt 1-2) = "
                          f"UART download ({ESPA})",
            "uart": "UART0 at the ESP bridge's baud (params flight_computer.link.esp_bridge.uart_baud 38400 8N1 - the COM's esp_bridge backend and firmware/esp_bridge use it; link.uart_baud 9600 is the LoRa fallback's only; corrected 2026-11-11, this text said 9600; "
                    "115200 also fine for the shifters); the ESP prints a boot log at 74880 baud on TXD0 after each "
                    "reset: the COM receives garbage bytes then (link_proto sync + CRC rejects them, A - check "
                    "com_link)",
            "flashing": ["fit J26 (COM held in reset: PC6 Hi-Z -> RST high via Q5's pull-ups; PD3 Hi-Z -> the ESP RXD0 "
                         "line idles high)",
                         "fit a shunt on J3 pins 1-2 (IO0 to GND)", "logic battery on (J15 closed)",
                         "3.3 V USB-UART: RX -> J3.3, TX -> J3.4, GND -> J3.2; reset by shorting J3.5-J3.6 briefly",
                         "flash, then remove the J3 1-2 shunt and J26, reset",
                         "the 1k between Q4 and RXD0 lets the programmer win even if J26 is forgotten (the COM could "
                         "still reset the ESP through PC6 only in a non-lora_uart build)"],
            "antenna": {"edge": "+y, antenna end flush with y = +35, module x about -24..-8 (A, placement)",
                        "keepout": "footprint keep-out (all copper layers) + rule area 4 mm wider on both sides, no "
                                   "copper on either layer, no parts (A)",
                        "option_overhang": "antenna part (7.3 mm) overhanging the +y edge to y ~42.3 gives the best "
                                           "pattern: corner r ~48.7 < bore 57.0, ~3.9 mm from the vent hole (0, +40) "
                                           "(E) - changes the carrier box in CAD -> user decision"},
            "rf_in_the_vehicle": [
                f"walls: every printed part is PLA Matte Ivory White ({P} printer.material / color, line 52-54; "
                "TiO2-pigmented, A): at the carrier height the antenna looks through the intake spigot (1.6 mm) + the "
                f"avionics tube (1.2 mm) PLA ({P} body.wall / spigot_wall, line 211-215) - low loss at 2.4 GHz (A)",
                "no carbon / aluminium structure found in params / CAD near the bay (E: params grep); metal nearby = the "
                "6S pouch (Al laminate) ~21 mm above / beside the antenna corner (pack y <= 17.5, z >= 346.1 vs antenna "
                "y >= 27.7, z ~330: E), the ESC (+x, ~35 mm), the motor leads (theta 0 deg, ~55 mm), the EDF motor "
                "~140 mm below (E from CAD boxes) - radiation upward / toward -x is partly shadowed, sideways toward "
                "+y through the PLA is open",
                "a logic pack in spot C (+y side of the 6S) would sit right above the antenna - prefer spot A / D for RF "
                "(E)",
                f"GPS (top cap, ~135 mm above): ESP 2.4 GHz harmonics are not at 1575 MHz but its spurious / clock noise "
                f"can desense the NEO-7M - compare C/N0 with Wi-Fi on / off ({P} electronics.gps.rf_notes line 1014 asks "
                "the same for LoRa)"],
            "range": {"wifi_pcb_antenna_los_m": [100, 300], "label": "A (typical ESP-12E figures, not measured)",
                      "vtol_need": f"{P} mission.vtol.ceiling_m 25 / geofence_m 35 (line 1258-1259), hover 5 m: "
                                   "inside the Wi-Fi range with margin (E)",
                      "rocket_need": f"{P} mission.target_altitude_m 140 / apogee_limit_m 150 (line 1212, 1232) + "
                                     "chute drift: at or beyond the Wi-Fi range (E) - LoRa (km) was chosen for this",
                      "decision": "user decision: Wi-Fi is fine for the VTOL envelope; the rocket build would need the "
                                  "same COM change and a range test (read-only report: params "
                                  f"flight_computer.link.radio 'lora_uart' line 2241, electronics.radio_log 'LoRa module "
                                  f"+ antenna' line 983)"},
            "regulatory": "2.4 GHz Wi-Fi is licence-exempt in Korea for certified modules; check the ESP-12E's KC "
                          "certification (A)",
        },
        "per_chip_support": {
            "each_mcu": ["16 MHz crystal (3225 SMD, CL 16-18 pF A) + 2 x 22 pF to GND, <= 3 mm from XTAL1 / XTAL2, "
                         "F.Cu only, no vias, locked, GND under, >= 1 mm from the board edge",
                         "RESET: 10 kOhm to +5V + 100 nF to GND; ISP pin 5",
                         "100 nF on VCC pin 21 and on VCC pin 52 (pad-to-pin <= 3 mm, own GND via, locked), 10 uF bulk",
                         "AVCC (pin 64): 10 uH series from +5V + 100 nF ('AVCC(LC)')",
                         "AREF (pin 62): 100 nF only - internal 2.56 V reference, never an external voltage on AREF",
                         "PEN (pin 1): tied to +5V (A: PEN low at power-on would enter SPI programming mode)",
                         "GND pins 22 / 53 / 63 to the GND pour",
                         "AVR ISP 2x3 header: 1 MISO = PE1/PDO, 2 VCC, 3 SCK = PB1, 4 MOSI = PE0/PDI, 5 RST, 6 GND",
                         "status LED + 1 kOhm (NAV PC1; FCU / COM / GNS PC0)"],
            "fuses": f"lfuse 0x3F, hfuse 0xCF, efuse 0xFF for every chip ({PIO}:51-56; {RM}:414-418): ext. crystal, "
                     "BOD 4.0 V, JTAG off (frees PF4-PF7), M103C off",
            "source": f"{RM}:196, 310-311; {BH}:37; {P3} section 6",
        },
        "isp_procedure": [
            {"chip": "NAV (J19)", "before": "nothing", "why": f"{RM}:649 (NAV PE0 / PE1 unused in flight)"},
            {"chip": "FCU (J20)", "before": "shunt on J24 (NAV held in reset); OpenLog may stay (Q8 isolates RXI)",
             "why": f"{RM}:650; {P} pins.nlink_sck (line 2162): NAV drives FCU PB1 = ISP SCK"},
            {"chip": "COM (J21)", "before": "shunt on J25 (FCU held in reset)",
             "why": f"{RM}:651; {BH}:59-62: COM PE0 / PE1 = ilink"},
            {"chip": "GNS (J22)", "before": "shunt on J24 (NAV held in reset)", "why": f"{BH}:99-100"},
            {"chip": "ESP-12E (J3)", "before": "shunt on J26 (COM held in reset) + shunt J3 1-2; 3.3 V USB-UART",
             "why": "see esp12e.flashing"},
            {"chip": "all", "before": "remove every shunt after programming; power the board from the logic battery and "
                                      "set the USBasp to 'target powered' (do not feed 5 V from both)",
             "why": f"{RM}:653"},
        ],
        "removed_5th_mcu": {
            "decision": f"{USER5} ('3번으로 가자 미실장인데 왜 만들어둔거야'): the unpopulated 5th ATmega128A and "
                        "everything that only served it are REMOVED",
            "removed_refs": REMOVED_5TH_REFS,
            "removed_parts": "U5 ATmega128A footprint, its 16 MHz crystal Y5 + 2 x 22 pF, 2 x 100 nF VCC + 10 uF bulk, "
                             "AVCC 10 uH + 100 nF, AREF 100 nF, reset 10k + 100 nF, LED + 1k, ISP header J23 and the "
                             "SPARE_* nets (no reset-hold shunt existed for it)",
            "refdes_note": "gaps left on purpose so all other refdes match the stopped rev-2 attempt",
            "origin": f"{P} configs.vtol.electronics.carrier.model 'optional 5th ATmega128A (timing study)' (line 1755); "
                      "docs/SPEC.md:1391 (10.6.1) - params / docs not edited -> listed in conflicts",
        },
        "layout_rules": {
            "note": f"lessons of the rev-1 review ({REV}) - the placement / routing steps must meet them and check them "
                    "with scripts (reuse scratchpad/pcb/review_*/ checks)",
            "rules": [
                "decoupling: every MCU / ESP VCC 100 nF pad <= 3 mm path to its pin, same layer, own GND via; pre-route "
                "and LOCK before Freerouting; scripted check of the cap-pad-to-pin routed length",
                "crystals: Y + 2 x 22 pF <= 3 mm from XTAL1 / XTAL2, hand-routed on F.Cu, no vias, locked, solid GND "
                "under, >= 1 mm from the board edge",
                "silkscreen on F.SilkS next to every off-board connector: servo 'S + -' + function, ESC, logic battery "
                "'+ - 1S', 6S sense pin names + 'NO GND', GPS pins, power switch, arm, buzzer, ISP name + pin-1, ESP "
                "flash header pin names + '3V3 ONLY', reset-hold names; refdes where they fit",
                "GND: B.Cu mostly solid GND, signals mainly on F.Cu, >= 60 stitching vias (none in the antenna "
                "keep-out), no thin GND tracks, nlink SPI over unbroken GND, GND_PWR trunk along the board edge not "
                "across the B.Cu pour",
                "servo power: SERVO_5V / GND_PWR >= 1.5 mm wide or poured, no neck-downs, more than one via per layer "
                "change",
                "boost path: VBAT_LOGIC_RAW -> Q10 -> J15 -> J6 pin 3 and J6 pin 1 -> +5V >= 1.0 mm (1.34 A peak, E), short; the Pololu module flat on J6 with its cut-off trimmer reachable from above",
                "ESP-12E: VCC caps at pin 8; antenna keep-out free of copper on both layers and of parts; the 1k "
                "series resistor next to the ESP RXD0 pin",
                "DRC: 0 errors, hole-to-hole warnings 0; schematic parity",
                "mechanical (MF-1..MF-9): OpenLog flat with its outline reserved; mated-plug outlines (JR servo 14.5 "
                "x 7.8 x 3.3, Dupont 14.0, JST / SPOX plugs, A) checked against the spigot bore r 57.0 and the joint "
                "screws in a script; ESC plug >= 2.0 mm from the motor-lead bullet zone; logic-battery connector with "
                "bend room; THT leads <= 1.5 mm under the board; module supports (GY-521 cantilever)",
                "keep the rev-1 good practice: two-stage Freerouting, determinism check, power-class width trade-off "
                "measurements"]},
    }


def firmware_followup() -> list[dict]:
    return [
        {"area": "ESP-12E bridge firmware (new, runs on the ESP8266)", "owner": "firmware round (user decides)",
         "items": ["transparent UART0 <-> Wi-Fi bridge only (UDP or TCP), no flight logic (project rule: all flight "
                   "computing on the ATmega128s)",
                   f"UART0 at the ESP bridge's baud: {P} flight_computer.link.esp_bridge.uart_baud 38400 (hal/esp_bridge.c; link.uart_baud 9600 is the LoRa fallback's - corrected 2026-11-11); it changes only if the COM "
                   "driver changes it",
                   "Wi-Fi mode: ESP as access point (laptop / phone joins) or station to a ground AP / a second ESP8266 - "
                   "user decision; fixed channel, no power save during flight (latency), TX power setting",
                   "boot log at 74880 baud on TXD0 after reset (cannot be disabled without GPIO15 tricks, A) - the COM "
                   "must tolerate it",
                   "optional: use GPIO4 (open on this board) as a 'bridge ready' line in a later board revision"]},
        {"area": "COM radio driver", "owner": "firmware round (user decides)",
         "items": ["the lora_uart backend (firmware/atmega128_fc/src/hal/lora_uart.c) drives M0 / M1 (PC3 / PC4, open "
                   "on this board) low and reads AUX (PC5, open = ready through its pull-up, line 69-70): with a "
                   "transparent ESP bridge at 9600 8N1 it may work UNCHANGED (E) - not the route taken: the COM has its own esp_bridge backend at 38400 (hal/esp_bridge.c, 2026-11-06)",
                   "PC6 (ESP RST through Q5) is untouched by the lora_uart build (Hi-Z -> ESP runs); a driver change may "
                   "pulse it low to restart a hung bridge",
                   "sim/link_model.py EBYTE_* timing constants (line 11-17) and the lora_uart air-rate model "
                   f"({P} link.lora_uart_air_rate_bps 2400, line 2245) do not describe Wi-Fi: re-derive / measure the "
                   "link latency and loss (control-relevant only through the link-loss timers)"]},
        {"area": "ground station", "owner": "firmware / tools round",
         "items": ["laptop / phone on the ESP's Wi-Fi (UDP socket) or a second ESP8266 + USB-UART on the ground running "
                   "the mirror bridge; tools/ground_station.py talks to a serial port today - add a UDP / TCP transport "
                   "(follow-up)"]},
        {"area": "rocket configuration", "owner": "user decision (read-only report)",
         "items": [f"the rocket build still uses LoRa ({P} flight_computer.link.radio 'lora_uart' line 2241; "
                   "electronics.radio_log 'LoRa module + antenna' line 983; firmware lora_uart / sx127x builds): using "
                   "the ESP there needs the same COM / ground change and a range check against 140-150 m apogee + drift"]},
        {"area": "NAV barometer (BMP280 replaces MS5611)", "owner": "firmware + sim round (control-relevant)",
         "items": ["new driver: BMP280 chip-id 0x58, trim registers, oversampling / filter settings, 20-bit raw values, "
                   "integer compensation (Bosch datasheet, not on disk - A); replaces hal/ms5611.c and core/ms5611_math.c; "
                   "I2C address kept at 0x77 by SDO high",
                   "NAV timing must be re-measured (bus script, background math)",
                   "sim baro noise model and the estimator / altitude-filter tuning -> CONTROL-RELEVANT change: archive "
                   "the before state and make before / after comparison videos (user rule) before adopting"]},
        {"area": "GNS GPS (NEO-7M replaces NEO-6M)", "owner": "firmware check",
         "items": ["MON-VER of u-blox 7 is '00070000' -> ubx_monver_module() returns UNKNOWN "
                   "(firmware/atmega128_fc/src/core/gps_ubx.c:276-298): reported only - verify on the unit",
                   "NEO-7M max navigation rate 10 Hz >= the 5 Hz the firmware sets (A: u-blox 7 data, not on disk); the "
                   "UBX CFG / NAV messages used exist in the u-blox 7 protocol (A)",
                   f"fit of the NEO-7M board + antenna in the top-cap GPS spot (board 36 x 25.8 x 4 + 25.5 mm patch, {P} "
                   "configs.vtol.top_cap.gps line 1821-1849) - measure, CAD not changed"]},
        {"area": "FCU", "owner": "firmware (recommended)",
         "items": ["read PF3 (logic battery, 47k / 47k, 2.56 V ref: V = counts x 2.56 / 1024 x 2) into telemetry; refuse "
                   "to ARM below ~3.6 V, LAND below ~3.4 V (A thresholds) - the board fits the divider by default",
                   f"PC3 buf_isign is defined ({PG}:173-174) but no firmware code reads it; its input format (logic level "
                   f"of the hybrid board) is undefined ({REV} E7)",
                   "fc_config.json imu_mounting must be set from the GY-521 orientation chosen in placement"]},
        {"area": "sim / power model", "owner": "sim round",
         "items": ["sim/tether.py:171 bec_min_v 6.0 (avionics brown-out with the 6S bus) no longer applies: the logic "
                   "runs from its own 1S pack; after a tether cut / sag the avionics keep running (servos lose the BEC) "
                   "-> scenario change, not done here; the firmware power rules that assume one bus need a look"]},
    ]


def user_decisions() -> list[str]:
    return [
        "DECIDED 2026-11-05: logic battery at spot A (-x face of the 6S, battery_spot_A_assumed); open: how to strap "
        "it, and another spot for the tether / hybrid builds (spot A overlaps their bus clamp / XT90 boxes) - CAD round",
        "mass: +84 g pack -> VTOL battery configuration ~1,737.6 g, ~87.6 g over 1,650 g (E); partial offsets ~-11 g "
        "carrier, a few g misc (E)",
        "logic power switch reachability (power.switch_reachability options a / b / c)",
        "5 V module: Pololu S9V11F5S6CMA (chosen: fixed 5 V buck-boost with an adjustable low-voltage cut-off, "
        "devicemart 15855364) - accept, and set its cut-off (3.2 V proposed, A); on the carrier (default) or "
        "off-board on a lead",
        "low-voltage cut-off level of the 5 V module (3.2 V proposed, A): it protects the cell but switches the whole "
        "avionics off when reached - the pre-flight >= 3.8 V check and the PF3 monitor (firmware follow-up) come first",
        "2 layers first; if routing does not close within ~90 min -> 4 layers (JLC standard stackup, higher board cost) - "
        "the routing step reports which one was delivered",
        "parts sources: JLC 'Extended' lines (ATmega128A, ESP-12E, BSS138, tantalum 22 uF, electrolytic, connectors) "
        "add assembly loading fees (A) - accept, or hand-solder the THT parts; the ESP-12E can be JLC-assembled (LCSC "
        "C89297) or hand-soldered (devicemart 1378118)",
        "ESP-12E antenna flush with keep-out (default) or overhanging the +y edge (better RF, CAD carrier box change)",
        "DECIDED 2026-11-05: Wi-Fi kept, range extended on the protocol / ground side (radio_decision_2026_11_05: "
        "ESP-NOW 1 Mbps, ground ESP-07S class + high-gain antenna, >= 1 km ground range test before flight, LoRa "
        "fallback = archive/rev1_plm100_2s)",
        "ESP bridge firmware, ground side (laptop / phone AP or second ESP8266) and COM driver check - separate round",
        "BMP280 driver + sim baro model - separate round, control-relevant (archived before / after videos)",
        "NEO-7M fit in the top cap and MON-VER behaviour - measure / confirm",
        "accept the 1 kOhm series resistor on glink NAV TXD1 -> GNS RXD0 and the reset-hold shunts J24 / J25 / J26",
        "OpenLog lying flat on the carrier in a right-angle socket (outline reserved) - confirm the clone's size",
        "modules socketed or soldered (GY-521 soldered flat on its pins + support pad recommended, A)",
    ]


def conflicts() -> list[str]:
    return [
        f"carrier model text ({P} line 1755) lists GY-63 MS5611, a LoRa module and a '5 V buck for the logic'; the user "
        "replaced them (BMP280 2026-11-04; ESP-12E and 1S + boost 2026-11-05) -> this spec follows the user; params not edited",
        f"mass_breakdown_g_ASSUMED 'gy521_gy63_modules' 3.8 g and 'lora_module_and_antenna' 14.0 g ({P} line 1770-1771) "
        "describe the old parts",
        f"power: {RM}:212, 244-247 and {P} flight_computer.ilink.note (line 2229) put the logic on a 5 V UBEC from the 6S; "
        f"{USER4} adds a separate logic battery, {USER5} makes it 1S -> boost, no 6S-to-logic converter",
        f"{P} configs.vtol.electronics.misc.items '5 V logic UBEC (or the carrier's buck)' (line 1788) -> neither: boost "
        "on the carrier",
        f"{P} flight_computer.com.pins.link_uart_rx 'PD2/RXD1 (module TXD, direct)' (line 2212) and {BH}:65 -> this spec "
        f"puts the ESP TX -> COM RX direction through Q11 ({USER5}, {REV} E2); firmware-neutral",
        f"{P} flight_computer.link.radio 'lora_uart' + lora_uart_note 'Ebyte ... M0 = M1 = 0' (line 2241-2246), "
        f"model_note 'the LoRa module is the only radio' (line 2259) -> the carrier carries an ESP-12E Wi-Fi bridge; "
        "firmware unchanged, follow-up",
        "sim/tether.py:171 bec_min_v models an avionics brown-out with the bus; with the logic battery the avionics stay up",
        f"electronics.gps.interface ({P} line 1011) still names the NAV USART1 (v0.8); {BH}:95-102 and "
        "flight_computer.gns.pins (v0.8.1) put the GPS on the GNS -> firmware wins (GNS)",
        "electronics.gps names NEO-6M (GY-NEO6MV2, line 1006); the user's part is NEO-7M (MON-VER 00070000 -> UNKNOWN in "
        "gps_ubx.c:276-298)",
        f"glink is wired direct in {BH}:105; this spec adds a 1 kOhm series resistor (A) because one board has no glink "
        "connector to unplug for GNS programming",
        f"programming: {RM}:647-651 says 'unplug the nlink / ilink / glink connector'; one carrier has none -> reset-hold "
        "shunts J24 (NAV) / J25 (FCU) / J26 (COM, for ESP flashing)",
        "release servo: configs.vtol removes it, but the firmware still drives PE5 (pwm.c:49, main.c:518) -> rev 2c "
        "removes the unpopulated J13 header and its series resistor (PE5 left open: a PWM output driving nothing); "
        "params / firmware not edited",
        f"OpenLog shifter channel was undefined ({RM}:385-387) -> Q8 here",
        f"GPS supply: {P} electronics.gps.power '3.3 V rail' vs NEO-7M boards that often take 3.3-5 V (A) -> JP1, 3.3 V default",
        f"VTOL mass over target: task text says ~3.7 g, {HW}:438 says 1,653.6 g = 3.6 g over ({CADR} totals 1653.7 g) - "
        "the doc / CAD value is used",
        f"{BH}:1 header says 'three ATmega128A' while the same file documents four (NAV, FCU, COM, GNS) - comment only",
        f"5th ATmega128A: {P} configs.vtol.electronics.carrier.model (line 1755) 'an unpopulated footprint for an optional "
        "5th ATmega128A (timing study)' and docs/SPEC.md:1391 (10.6.1) keep it; the user removed it (2026-11-05) -> not on "
        "this board; params / docs not edited",
        f"crystal load capacitance: {P3} section 6 gives 2 x 22 pF; the JLC Basic 16 MHz 3225 crystal (C13738) has CL 9 pF "
        "-> the spec uses C70562 X322516MRB4SI (CL 18 pF) with 27 pF caps (rev-2b review E-m2)",
        f"{BH}:35-36 says 'unplug the OpenLog while programming the FCU'; on this board the OpenLog sits behind Q8 (FCU PE1 "
        "only reaches its RXI through the shifter) -> it may stay plugged",
    ]


def assumptions() -> list[str]:
    return [
        "logic pack DTP105565P has a PCM ('P' suffix) cutting at ~2.5-3.0 V; its plug is a Molex 5264-type 2.50 mm "
        "2-pin housing, pin 1 = + (red) - check both on the bought pack",
        "5 V module Pololu S9V11F5S6CMA: pin order VOUT, GND, VIN, EN, PG along one edge (vendor text order - check the "
        "silkscreen); ~1 A continuous from a 3.0-3.7 V input (vendor graph not read); efficiency 85 %; cut-off set to 3.2 V",
        "AO3401A: Vgs(th) -0.5..-1.3 V, Rds(on) <= 85 mOhm at Vgs -2.5 V, Vgs max +-12 V (datasheet not on disk)",
        "ESP-12E: VCC 3.0-3.6 V, ~80 mA average / up to ~400 mA bursts, I/O 3.3 V only (not 5 V tolerant), straps "
        "EN / RST / GPIO0 / GPIO2 high, GPIO15 low, boot log 74880 baud (Espressif / Ai-Thinker data not on disk)",
        "Wi-Fi range with the PCB antenna 100-300 m line of sight; PLA walls low-loss at 2.4 GHz",
        "BMP280 module pin order VCC, GND, SCL, SDA, CSB, SDO; CSB and SDO tied to 3.3 V (I2C, 0x77)",
        "GY-521 pin order VCC, GND, SCL, SDA, XDA, XCL, AD0, INT; AD0 to GND = 0x68; module VCC 5 V",
        "NEO-7M board pins VCC, GND, TXD, RXD (+ PPS), 3.3 V via JP1; carrier JST-GH order 1 VCC, 2 GPS TXD, 3 GPS RXD, 4 GND",
        "OpenLog header order GRN, RXI, TXO, VCC, GND, BLK; ~19 x 16 mm board lying flat; VCC 5 V",
        "ATmega128A 25 mA each at 16 MHz / 5 V; OpenLog 20 mA avg / 80 mA peak; NEO-7M 45 mA",
        "AVCC filter 10 uH + 100 nF (SDFL2012S100KTF rated 15 mA, AVCC draw < 1 mA, A); crystals 3225 SMD CL 18 pF "
        "with 22 pF caps; PEN tied to VCC",
        "parts: the LCSC / devicemart parts in parts_sources.json match the KiCad footprints by package name (scripted); "
        "pin-1 / pad orientation of THT headers and modules still to be checked against the bought parts",
        "MTS-102 harness toggle (devicemart 13029107) contact rating >= 2 A at 5 V DC (A, typical MTS-102 data; the page "
        "is checked in parts_sources.json) for the 1.34 A boost-input peak",
        "BSS138 pull-ups 10k / 10k (I2C 4.7k on 5 V / 10k on 3.3 V); servo / ESC series 220 Ohm + 10k pull-downs; arm "
        "input 1k series + 10k pull-up + 100 nF; ESP RXD0 series 1k",
        "logic ground tied to the propulsion ground only at NT1 (J12 pin 3)",
        "2 layers, 1 oz copper, near-solid bottom GND pour; corner radius 1 mm; tie band x -3..+3 mm free of parts",
        "servo / ESC / sense / arm leads on right-angle headers; logic battery (SPOX), buzzer (JST-PH) and GPS (JST-GH) "
        "vertical, plugged from above; THT leads <= 1.5 mm under the board for the 3 mm foam",
        "edge assignment from the CAD lead entries (servo hole 180 deg, GPS lead hole 230 deg, arm switch 90 deg, ESC / "
        "misc +x); ESP antenna toward +y (side windows 90 / 112 deg)",
    ]


def main() -> int:
    build()
    rep = check()
    spots = battery_spots()
    if ERRORS:
        for e in ERRORS:
            print("ERROR", e)
        print(f"{len(ERRORS)} error(s) - spec NOT written")
        return 1
    parts = sorted(PARTS.values(), key=lambda p: natural(p["refdes"]))
    bom = sourcing(parts)
    if ERRORS:
        for e in ERRORS:
            print("ERROR", e)
        print(f"{len(ERRORS)} error(s) - spec NOT written")
        return 1
    nets = sorted(NETS.values(), key=lambda n: natural(n["name"]))
    for n in nets:
        n["members"] = n.pop("resolved")
        if not n["notes"]:
            n.pop("notes")
    ncl = sorted(({"pin": k, "reason": v} for k, v in NC.items()), key=lambda d: natural(d["pin"]))
    summary = {
        "parts": len(parts),
        "parts_populated": sum(1 for p in parts if p["populate"] is True),
        "parts_dnp_or_optional": sum(1 for p in parts if p["populate"] is not True),
        "nets": len(nets),
        "by_prefix": {k: sum(1 for p in parts if re.match(rf"{k}\d", p["refdes"]))
                      for k in ["U", "J", "Q", "R", "C", "L", "Y", "D", "JP", "NT"]},
    }
    spec = {
        "schema": "vtol_carrier_spec/3",
        "revision": "rev 2c (2026-11-05): rev 2b (ESP-12E Wi-Fi + 1S logic battery with a 5 V buck-boost module "
                    "(low-voltage cut-off), 5th ATmega128A removed, every part sourced from JLCPCB/LCSC or devicemart, "
                    "verified online) + the rev-2b independent review fixes (rev_2c_changes); rev 1 (PLM100 LoRa + 2S) "
                    "archived in archive/rev1_plm100_2s/",
        "title": "CanSat EDF VTOL avionics carrier PCB - electrical specification (not drawn yet)",
        "status": "electrical spec only; the KiCad schematic and board are generated from it later (scripts/). "
                  "NOT_VERIFIED: nothing built, drawn or measured; checks below are scripted netlist checks, not ERC / DRC",
        "labels": {"F": "from a file (params / firmware / docs / vendor page / KiCad library)", "M": "measured by a tool",
                   "E": "estimated", "A": "assumed (reason given)"},
        "sources": SOURCES,
        "user_parts": {
            "barometer": "BMP280 module [SMG] TYE-BMP29 (devicemart 15442588) - replaces GY-63 MS5611",
            "radio": "ESP-12E (ESP8266) soldered on the carrier (user 2026-11-05) - replaces the PLM100 LoRa module of rev 1",
            "gps": "NEO-7M module SZH-NT07 (devicemart 10895400) - in the top cap, carrier has the lead connector only",
            "imu": "GY-521 MPU-6050 SZH-EK007 (devicemart 1247052) - as before",
            "logic_battery": "DTP105565P 1S 3.7 V 4200 mAh 84 g (devicemart 16021596) - user 2026-11-05",
        },
        "changes_from_rev1": [
            "PLM100 sockets J3 / J4 and all LORA_* nets removed; U7 ESP-12E + straps + J3 flash header + J26 COM reset hold",
            "ESP TX -> COM RX through a new BSS138 channel Q11 (rev-1 E2)",
            "2S + buck (J6 1x4) -> 1S Molex SPOX input + 5 V buck-boost module with low-voltage cut-off (Pololu "
            "S9V11F5S6CMA) on J6 1x5 (J4 no longer used); Q10 zener dropped",
            "logic battery divider 10k / 3.3k -> 47k / 47k (4.2 V max), still fitted",
            "OpenLog: lead pad row -> right-angle socket, module lies flat with its outline reserved (MF-1)",
            "layout_rules carry the rev-1 review findings (F1, F2, silkscreen, GND, servo power, MF-1..MF-9)",
            "rev 2b: 5th ATmega128A and its support parts removed (user); 5 V module = Pololu S9V11F5S6CMA on J6 1x5 "
            "with a hardware low-voltage cut-off; crystal CL 18 pF part; servo bulk cap footprint 6.3 x 5.4 (sourced "
            "part); 4-layer fallback plan; BOM sources (LCSC C-numbers / devicemart no=) verified online",
        ],
        "rev_2c_changes": [
            f"J14 (1S input) + J15 (power-switch lead) moved to the -x / +y corner next to battery spot A ({USER5} "
            f"'battery A'; {REV2} PF-M4 / MF2-1); J15 is a JST-PH 2-pin vertical instead of a 2.54 mm Dupont header "
            f"(mated plug inside the 13.4 mm height, {REV2} MF2-4)",
            f"release-servo header J13 (unpopulated, VTOL has no release servo) and its DNP series resistor removed; U2 PE5 "
            f"no-connect ({REV2} MF2-1 / PF-m6: it blocked the corner and sat 2-6 mm from the antenna)",
            f"ESP flash header J3: 1x6 under the OpenLog at the far +x edge -> 2x3 (same pin numbers) at the top of the "
            f"tie band right beside the ESP-12E's GPIO0 / TXD0 / RXD0 pins (rev 2b: four ESP nets crossed the board and "
            f"stayed unrouted, {REV2} E-B1 / PF-B1)",
            f"crystal load caps 22 pF -> 27 pF (CL 18 pF crystal, {REV2} E-m2); ESP bulk 100 uF 6.3 V -> 10 V tantalum "
            f"({REV2} E-m4)",
            f"layout (scripts, not this spec): each MCU GND pin 22 / 53 / 63 gets its own locked GND via inside the "
            f"TQFP pad ring ({REV2} E-B2), stitching vias never in pads (PF-M3), J6 / J14 supply pads solid to the "
            f"planes (PF-M1), silkscreen labels on F.SilkS beside each connector (PF-M2 / MF2-9), JLC CPL rotation "
            f"offsets (PF-M5), power neck-downs widened where clearance allows (PF-M6)",
        ],
        "battery_spot_A_assumed": {
            "decision": f"{USER5} '배터리는 A 위치(6S 팩 −x 면)로 해줘'",
            "pack_box_vehicle_mm": {"x": [-48.4, -37.9], "y": [-27.75, 27.75], "z": [346.1, 413.1]},
            "label": "E (from power.battery_spots A; the CAD itself is not edited here - the CAD round places the pack)",
            "orientation": "lead end at +y (the lead drops ~6 mm in plan to J14 at the -x / +y board corner)",
            "valid_for": "battery power configuration only: in the tether / hybrid builds this box overlaps the tether "
                         "bus clamp (~13,500 mm3) and XT90 (~190-940 mm3) boxes (rev-2b review MF2-2, E) -> user decision",
        },
        "radio_decision_2026_11_05": {
            "decision": f"{USER5} '빠듯하면 그냥 lora로 할까? 전송 거리를 늘릴 수 있는 방법이 있다면 가능하면 wifi방식을 "
                        "채택하고, 아니면 lora로 하자' -> Wi-Fi (ESP8266) adopted; the soldered ESP-12E (PCB antenna) stays",
            "range_plan": "ESP-NOW (or fixed-rate 802.11b) at 1 Mbps (sensitivity ~-98 dBm vs ~-72 dBm at 802.11n high "
                          "rates, A: Espressif ESP8266EX datasheet, not on disk), TX +20 dBm; ground station = a second "
                          "ESP8266 with an external-antenna module (ESP-07S class, u.FL) + 8-9 dBi omni or 12-14 dBi "
                          "directional 2.4 GHz antenna",
            "link_budget_E": "20 dBm - 2 dBi (vehicle, in body) + 8 dBi (ground) - FSPL - 10 dB margin: ~-78 dBm at "
                             "500 m, ~-84 dBm at 1 km (14 dB margin), ~-90 dBm at 2 km",
            "flight_condition": "ground range test at >= 2x the largest expected slant range (e.g. 1 km) before flight; "
                                "if it fails, fall back to LoRa (PLM100 first build, archive/rev1_plm100_2s/)",
            "regulatory": "Korean 2.4 GHz EIRP limits apply to a high-gain ground antenna (A - check)",
            "board": "ESP-12E antenna at the +y edge with its keep-out, facing a PLA wall, away from the 6S pack, ESC "
                     "and motor leads; ESP bridge firmware / ESP-NOW protocol / ground station = firmware follow-up",
        },
        "summary": summary,
        **narrative(spots),
        "parts": parts,
        "nets": nets,
        "no_connect": ncl,
        "checks": {"tool": "scripts/build_spec.py (scripted netlist checks; not ERC / DRC)", "result": "PASS", **rep},
        "firmware_followup": firmware_followup(),
        "user_decisions": user_decisions(),
        "conflicts": conflicts(),
        "assumptions": assumptions(),
        "bom_sources": bom,
    }
    OUT.write_text(json.dumps(spec, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {OUT}: {summary}")
    print(f"checks: pins {rep['pins_total']} = connected {rep['pins_connected']} + nc {rep['pins_no_connect']}; "
          f"paths {sum(1 for p in rep['signal_paths'] if p['ok'])}/{len(rep['signal_paths'])}; "
          f"bss138 {len(rep['bss138_channels'])}; straps {sum(1 for s in rep['esp_straps'] if s['ok'])}/{len(rep['esp_straps'])}")
    for k, v in spots["spots"].items():
        print(k, v["bore_clearance_mm"], v["overlaps_cad_parts"], v["gps_lead_drop_inside"], v["cg_after_mm"],
              v["lateral_cg_off_axis_mm"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
