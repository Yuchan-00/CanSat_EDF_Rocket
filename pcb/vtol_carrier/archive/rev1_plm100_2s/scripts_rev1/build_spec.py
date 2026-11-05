"""Build spec/carrier_spec.json - the electrical specification of the VTOL avionics carrier PCB - and check it.

Invariants this script enforces (it exits non-zero and writes nothing when one fails):
* every part's symbol and footprint exist in the installed KiCad 10 libraries (spec/library_probe.json, written by
  scripts/probe_library.py from the library files on disk) and the footprint's pad numbers equal the symbol's pin
  numbers - no pin number is typed here: a member "U2.PD3" is resolved to pin 28 from the library symbol;
* every pin of every part is in exactly one net or in the explicit no-connect list (with a reason);
* every net has >= 2 members and a source / reason;
* signal paths (UART TX -> RX, SPI, I2C, PWM, ADC, ISP) are traced through the netlist (series resistors and BSS138
  channels only, never through a supply rail): a swapped TX / RX or a broken path fails the build;
* no net that a path reaches holds two UART / SPI drivers (TX on TX);
* a 3.3 V-only input pin never reaches an ATmega128A pin except through a BSS138 channel.
Deterministic: same inputs -> byte-identical JSON (sorted, no timestamps).

Run: B:/Claude/.venv/Scripts/python.exe -X utf8 scripts/build_spec.py
"""
from __future__ import annotations

import json
import re
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
PROBE = json.loads((HERE / "spec" / "library_probe.json").read_text(encoding="utf-8"))
OUT = HERE / "spec" / "carrier_spec.json"

# ----------------------------------------------------------------------------------------------- sources
P = "params.json"                      # B:/0)AI_Cad/CanSat_EDF_Rocket/params.json
BH = "firmware/atmega128_fc/src/board.h"
PG = "firmware/atmega128_fc/src/params_gen.h"
RM = "firmware/README.md"
PIO = "firmware/atmega128_fc/platformio.ini"
DS = "PLM100 Hardware Datasheet V1.0 (plm100_1.pdf)"
UG = "PLM100 User's Guide V2.2 (plm100_3.pdf)"
CG = "PLM100 User's Guide V1.4 (plm100_2.pdf, 'configuration guide')"
CADR = "cad/exports/cadquery_report_vtol.json"
P3 = "fw09/p3/README.md (scratch study, P3 fifth-chip proposal)"
USER = "user decision 2026-11-04"
REV = "independent review 2026-11-05"

SOURCES = {
    "params.json": "B:/0)AI_Cad/CanSat_EDF_Rocket/params.json (read-only; line numbers as of 2026-11-04)",
    BH: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + BH,
    PG: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + PG + " (generated from params.json)",
    RM: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + RM,
    PIO: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + PIO,
    "firmware/atmega128_fc/src/hal/*.c": "B:/0)AI_Cad/CanSat_EDF_Rocket/firmware/atmega128_fc/src/hal/",
    "firmware/atmega128_fc/fc_config.json": "B:/0)AI_Cad/CanSat_EDF_Rocket/firmware/atmega128_fc/fc_config.json",
    CADR: "B:/0)AI_Cad/CanSat_EDF_Rocket/" + CADR,
    "cad/layout.py": "B:/0)AI_Cad/CanSat_EDF_Rocket/cad/layout.py",
    DS: "C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/tools_dl/"
        "plm100/plm100_1.pdf (text: scratchpad/pcb/src/plm100_1.txt; figures: scratchpad/pcb/img/plm100_1_p*.jpg)",
    UG: "C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/tools_dl/"
        "plm100/plm100_3.pdf",
    CG: "C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/tools_dl/"
        "plm100/plm100_2.pdf",
    P3: "C:/Users/yc012/AppData/Local/Temp/claude/B--Claude/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/fw09/p3/README.md",
    "KiCad libraries": "C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/share/kicad/{symbols,footprints} "
                       "(probed: spec/library_probe.json)",
    "devicemart": "user-chosen products: devicemart no=15442588 (BMP280 TYE-BMP29), no=1289751 (PLM100), "
                  "no=10895400 (NEO-7M SZH-NT07), no=1247052 (GY-521 SZH-EK007); pin orders not printed (A)",
}

# ----------------------------------------------------------------------------------------------- library ids
MCU = ("MCU_Microchip_ATmega:ATmega128A-A", "Package_QFP:TQFP-64_14x14mm_P0.8mm")
LDO = ("Regulator_Linear:AMS1117-3.3", "Package_TO_SOT_SMD:SOT-223-3_TabPin2")
NFET = ("Transistor_FET:BSS138", "Package_TO_SOT_SMD:SOT-23")
PFET = ("Transistor_FET:AO3401A", "Package_TO_SOT_SMD:SOT-23")
NPN = ("Transistor_BJT:MMBT3904", "Package_TO_SOT_SMD:SOT-23")
XTAL = ("Device:Crystal_GND24", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm")
R0603 = ("Device:R", "Resistor_SMD:R_0603_1608Metric")
C0402 = ("Device:C", "Capacitor_SMD:C_0402_1005Metric")
C0603 = ("Device:C", "Capacitor_SMD:C_0603_1608Metric")
C0805 = ("Device:C", "Capacitor_SMD:C_0805_2012Metric")
C1206 = ("Device:C", "Capacitor_SMD:C_1206_3216Metric")
CPOL = ("Device:C_Polarized", "Capacitor_SMD:CP_Elec_6.3x5.8")
TANT = ("Device:C_Polarized", "Capacitor_Tantalum_SMD:CP_EIA-3216-18_Kemet-A")
ZENER = ("Device:D_Zener", "Diode_SMD:D_SOD-123")
IND = ("Device:L", "Inductor_SMD:L_0805_2012Metric")
LED = ("Device:LED", "LED_SMD:LED_0603_1608Metric")
SCHOTTKY = ("Device:D_Schottky", "Diode_SMD:D_SOD-123")
NETTIE = ("Device:NetTie_2", "NetTie:NetTie-2_SMD_Pad0.5mm")
SJ3 = ("Jumper:SolderJumper_3_Bridged12", "Jumper:SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm")
ISP = ("Connector:AVR-ISP-6", "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical")
H2V = ("Connector_Generic:Conn_01x02", "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical")
H2H = ("Connector_Generic:Conn_01x02", "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal")
H3H = ("Connector_Generic:Conn_01x03", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal")
H4H = ("Connector_Generic:Conn_01x04", "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal")
H4V = ("Connector_Generic:Conn_01x04", "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical")
H6V = ("Connector_Generic:Conn_01x06", "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical")
H8V = ("Connector_Generic:Conn_01x08", "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical")
S6H = ("Connector_Generic:Conn_01x06", "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Horizontal")
S6V = ("Connector_Generic:Conn_01x06", "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical")
S12 = ("Connector_Generic:Conn_01x12", "Connector_PinSocket_1.27mm:PinSocket_1x12_P1.27mm_Vertical")
JSTGH = ("Connector_Generic_MountingPin:Conn_01x04_MountingPin",
         "Connector_JST:JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical")
JSTPH = ("Connector_Generic:Conn_01x02", "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical")

RAILS = {"+5V", "+3V3", "GND", "GND_PWR", "SERVO_5V", "VBAT_LOGIC_RAW", "VBAT_LOGIC", "VBAT_LOGIC_SW"}

# ----------------------------------------------------------------------------------------------- model
PARTS: dict[str, dict] = {}
NETS: dict[str, dict] = {}
NC: dict[str, str] = {}             # "REF.pinnumber" -> reason
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
DEC0402 = f"; 0402 so its pads sit under the supply / GND pin pair, pre-routed in place ({REV} F1)"
XTAL0402 = f"; 0402, crystal + load caps placed as one cluster at pins 23 / 24 and pre-routed on F.Cu ({REV} F2)"
MCU_REFS = {"NAV": "U1", "FCU": "U2", "COM": "U3", "GNS": "U4", "SPARE": "U5"}
ISP_REFS = {"NAV": "J19", "FCU": "J20", "COM": "J21", "GNS": "J22", "SPARE": "J23"}


def mcu_block(role: str, populate: bool | str, led_pin: str, led_src: str, mosi: str, miso: str, sck: str) -> None:
    u, j = MCU_REFS[role], ISP_REFS[role]
    blk = role
    part(u, "U", MCU, "ATmega128A-AU 16 MHz 5 V", f"{role} microcontroller",
         {"NAV": f"{P} flight_computer.nav (line ~2170); {BH}:5-7",
          "FCU": f"{P} flight_computer.pins (line 2147-2167); {BH}:8-10",
          "COM": f"{P} flight_computer.com.pins (line 2208-2222); {BH}:11-12",
          "GNS": f"{P} flight_computer.gns.pins (line 2266-2271); {BH}:95-102",
          "SPARE": f"{P} configs.vtol.electronics.carrier.model (line 1754); SPEC.md 10.6.1 (5th footprint unpopulated, P3 reserve)"}[role],
         populate, "F", block=blk,
         note=("unpopulated 5th ATmega128A: power, ground, crystal, reset, AVCC/AREF, decoupling and its own ISP only "
               "(the P3 nets of fw09/p3 README section 6 are NOT routed - P3 is a fallback with a hardware-measurement "
               "trigger; bodge-wire them if P3 is ever adopted)") if role == "SPARE" else None)
    src_sup = f"{RM}:310 (VCC, AVCC(LC), 100 nF per VCC/GND pair); {PIO}:51-56 (16 MHz crystal, CKOPT on)"
    net("+5V", f"{u}.VCC", f"{u}.~{{PEN}}", f"{j}.VCC", source=f"{P} flight_computer.vcc_V 5.0 (line 2137)")
    net("GND", f"{u}.GND", f"{j}.GND")
    c("100n", "+5V", "GND", f"{role} VCC pin 21 decoupling", src_sup + DEC0402, populate, block=blk, lib=C0402)
    c("100n", "+5V", "GND", f"{role} VCC pin 52 decoupling", src_sup + DEC0402, populate, block=blk, lib=C0402)
    c("10u", "+5V", "GND", f"{role} local bulk", "A: 10 uF ceramic per chip (bulk next to the chip)", populate,
      block=blk, lib=C0805)
    lref = part(None, "L", IND, "10uH", f"{role} AVCC filter inductor", f"{RM}:310 / 332 / 345 'AVCC(LC)'; value A",
                populate, "A", block=blk)
    net("+5V", f"{lref}.1")
    net(f"{role}_AVCC", f"{lref}.2", f"{u}.AVCC", source=f"{RM}:310 AVCC(LC)")
    c("100n", f"{role}_AVCC", "GND", f"{role} AVCC decoupling", src_sup + DEC0402, populate, block=blk, lib=C0402)
    net(f"{role}_AREF", f"{u}.AREF",
        source=f"firmware/atmega128_fc/src/hal/adc.h:4-6 (AREF carries only its 100 nF; internal 2.56 V ref); {BH}:37")
    c("100n", f"{role}_AREF", "GND", f"{role} AREF capacitor (internal 2.56 V reference)",
      f"{BH}:37; firmware/atmega128_fc/src/hal/adc.h:4-6; {RM}:205", populate, "F", block=blk)
    y = part(None, "Y", XTAL, "16 MHz, CL 16-18 pF (A), 3225 SMD", f"{role} crystal",
             f"{P} flight_computer.f_cpu_hz 16000000; {PIO}:51-55 lfuse 0x3F (ext. crystal, CKSEL 1111, SUT 11), "
             "hfuse 0xCF (CKOPT programmed = full-swing oscillator)", populate, "F", block=blk,
             note="SMD 3225 chosen over HC-49S for height / mass (A; P3 README section 6 lists both)")
    net(f"{role}_XTAL1", f"{u}.XTAL1", f"{y}.1")
    net(f"{role}_XTAL2", f"{u}.XTAL2", f"{y}.3")
    net("GND", f"{y}.G")
    c("22p", f"{role}_XTAL1", "GND", f"{role} crystal load cap", f"{P3} section 6 (2 x 22 pF); value A (match the crystal CL)"
      + XTAL0402, populate, block=blk, lib=C0402)
    c("22p", f"{role}_XTAL2", "GND", f"{role} crystal load cap", f"{P3} section 6 (2 x 22 pF); value A" + XTAL0402, populate,
      block=blk, lib=C0402)
    net(f"{role}_RESET", f"{u}.~{{RESET}}", f"{j}.~{{RST}}", source=f"{RM}:311 ISP pin 5 RST")
    r("10k", f"{role}_RESET", "+5V", f"{role} reset pull-up", f"{P3} section 6 (reset 10 kOhm + 100 nF)", populate, block=blk)
    c("100n", f"{role}_RESET", "GND", f"{role} reset capacitor", f"{P3} section 6 (reset 10 kOhm + 100 nF)", populate, block=blk)
    part(j, "J", ISP, "AVR ISP 2x3 (USBasp)", f"{role} ISP header (MOSI = PE0/PDI, MISO = PE1/PDO, SCK = PB1)",
         f"{RM}:311 / 213 (MOSI->PE0, MISO<-PE1, SCK->PB1); {BH}:43", populate, "F", block=blk)
    net(mosi, f"{u}.PE0", f"{j}.MOSI")
    net(miso, f"{u}.PE1", f"{j}.MISO")
    net(sck, f"{u}.PB1", f"{j}.SCK")
    # status LED (active high, 1 kOhm)
    lr = r("1k", f"{role}_LED", f"{role}_LED_A", f"{role} status LED resistor", led_src, populate, "F", block=blk)
    d = part(None, "D", LED, "LED 0603 (colour A)", f"{role} status LED", led_src, populate, "F", block=blk)
    net(f"{role}_LED", f"{u}.{led_pin}", source=led_src)
    net(f"{role}_LED_A", f"{d}.A")
    net("GND", f"{d}.K")
    _ = lr


# ----------------------------------------------------------------------------------------------- level shifter
def bss138(q: str, hv: str, lv: str, role: str, source: str, populate: bool | str = True, r_hv: str = "10k",
           r_lv: str = "10k", pop_hv: bool | str | None = None, src_hv: str = "", src_lv: str = "") -> None:
    part(q, "Q", NFET, "BSS138", f"level shifter channel: {role}", source, populate, "F", block="SHIFT")
    net("+3V3", f"{q}.G")
    net(lv, f"{q}.S")
    net(hv, f"{q}.D")
    r(r_hv, hv, "+5V", f"{q} HV (5 V) pull-up", "standard BSS138 bidirectional shifter (A values)" + src_hv,
      populate if pop_hv is None else pop_hv, block="SHIFT")
    r(r_lv, lv, "+3V3", f"{q} LV (3.3 V) pull-up", "standard BSS138 bidirectional shifter (A values)" + src_lv, populate,
      block="SHIFT")


def build() -> None:
    # ======================================================================= MCUs
    mcu_block("NAV", True, "PC1", f"{P} flight_computer.nav.pins.led PC1 (line 2182); {BH}:54",
              "NAV_ISP_MOSI", "NAV_ISP_MISO", "NLINK_SCK")
    mcu_block("FCU", True, "PC0", f"{P} flight_computer.pins.led PC0 (line 2156); {BH}:38",
              "FCU_ISP_MOSI", "FCU_LOG_TX", "NLINK_SCK")
    mcu_block("COM", True, "PC0", f"{P} flight_computer.com.pins.led PC0 (line 2211); {BH}:63",
              "COM_ILINK_RX", "ILINK_C2F", "COM_ISP_SCK")
    mcu_block("GNS", True, "PC0", f"{P} flight_computer.gns.pins.led PC0; {BH}:101, 139-144",
              "GNS_GLINK_RX", "GLINK_G2N", "GNS_ISP_SCK")
    mcu_block("SPARE", False, "PC0", "A: same LED as the FCU / COM / GNS (P3 README section 6 lists 'LED + resistor')",
              "SPARE_ISP_MOSI", "SPARE_ISP_MISO", "SPARE_ISP_SCK")

    # ======================================================================= inter-chip links (no shifter: all 5 V)
    net("NLINK_SS", "U1.PB0", "U2.PB0", source=f"{P} flight_computer.nav.pins.spi_ss / pins.nlink_ss (line 2161, 2178); {BH}:29-30, 50, 79",
        kind="spi")
    r("10k", "NLINK_SS", "+5V", "nlink SS pull-up (FCU idle-high while the NAV is in reset / programmed)",
      f"{BH}:29-30, 55-56 (FCU SS pull-up keeps the FCU SPI idle); external 10 kOhm in addition to the internal one: A",
      True, block="LINK")
    net("NLINK_SCK", source=f"{P} pins.nlink_sck (line 2162); {BH}:31-32, 51, 80 (also the NAV and FCU ISP SCK)", kind="spi")
    net("NLINK_MOSI", "U1.PB2", "U2.PB2", source=f"{BH}:33, 52, 82", kind="spi")
    net("NLINK_MISO", "U1.PB3", "U2.PB3", source=f"{BH}:34, 53, 83 (NAV input with pull-up)", kind="spi")
    net("NLINK_SCK", "U1.PB1", "U2.PB1")
    # ilink FCU USART1 <-> COM USART0
    net("FCU_ILINK_TX", "U2.PD3", source=f"{P} pins.ilink_tx (line 2160); {BH}:42, 89", kind="uart")
    r("1k", "FCU_ILINK_TX", "COM_ILINK_RX", "ilink series resistor FCU TXD1 -> COM RXD0",
      f"{P} flight_computer.ilink.series_resistor_ohm 1000 (line 2228); {BH}:42, 89; {RM}:355", True, "F", block="LINK")
    net("COM_ILINK_RX", "U3.PE0", source=f"{P} com.pins.ilink_rx (line 2209); {BH}:59-61", kind="uart")
    net("ILINK_C2F", "U2.PD2", "U3.PE1", source=f"{P} pins.ilink_rx / com.pins.ilink_tx (line 2159, 2210); {BH}:41, 62, 90",
        kind="uart")
    # glink NAV USART1 <-> GNS USART0
    net("NAV_GLINK_TX", "U1.PD3", source=f"{BH}:99, 105 (NAV PD3 TXD1 -> GNS PE0 RXD0)", kind="uart")
    r("1k", "NAV_GLINK_TX", "GNS_GLINK_RX", "glink series resistor NAV TXD1 -> GNS RXD0 (NEW, like the ilink one)",
      f"A: {BH}:105 wires it direct; the 1 kOhm copies the ilink rule ({BH}:59-61) because GNS PE0 is its ISP PDI "
      "(programming the GNS with the NAV running would fight the NAV's TXD1)", True, "A", block="LINK")
    net("GNS_GLINK_RX", "U4.PE0", source=f"{P} gns.pins.glink_rx (line ~2269); {BH}:99", kind="uart")
    net("GLINK_G2N", "U1.PD2", "U4.PE1", source=f"{P} gns.pins.glink_tx; {BH}:100, 106", kind="uart")
    # programming hold jumpers (no unpluggable harness on one board)
    part("J24", "J", H2V, "2-pin header + shunt", "NAV reset hold (shunt on = NAV held in reset)",
         f"{RM}:649-650 (program the FCU: hold the NAV in reset); {P} pins.nlink_sck (line 2162); "
         f"{BH}:99-100 (GNS ISP shares the glink)", True, "A", block="LINK",
         note="replaces 'unplug the nlink / glink connector' of the multi-board build: shunt on while programming the FCU or the GNS")
    net("NAV_RESET", "J24.1")
    net("GND", "J24.2")
    part("J25", "J", H2V, "2-pin header + shunt", "FCU reset hold (shunt on = FCU held in reset)",
         f"{RM}:651 (program the COM: unplug the ilink); {BH}:59-61", True, "A", block="LINK",
         note="replaces 'unplug the ilink connector': shunt on while programming the COM (FCU PD3 / PD2 go high-Z)")
    net("FCU_RESET", "J25.1")
    net("GND", "J25.2")

    # ======================================================================= power: logic battery -> buck -> 5 V -> LDO -> 3.3 V
    part("J14", "J", JSTPH, "JST-PH 2.0 2-pin vertical (top entry)", "logic battery input (2S LiPo ONLY, separate from the 6S)",
         f"{USER} (separate logic battery); connector type A (match the purchased pack); vertical so the battery lead "
         f"leaves upward toward the battery and no plug sticks out toward the spigot wall ({REV} MF-4)", True, "A", block="PWR",
         note="pin 1 = +, pin 2 = - (A: JST-PH polarity differs between vendors - Q10 protects a reversed plug). 2S only: "
              "D7 clamps the Q10 gate, a 3S pack is still outside the design range")
    net("VBAT_LOGIC_RAW", "J14.1", source=USER)
    net("GND", "J14.2")
    part("Q10", "Q", PFET, "AO3401A (-30 V, -4 A)", "reverse-polarity protection (P-MOSFET, high side)",
         f"{USER} (reverse-polarity protection); part A", True, "A", block="PWR",
         note="drain = battery +, source = load: the body diode starts conduction, Vgs = -Vbat turns it on; a reversed "
              "pack leaves it off. Vgs -8.4 V at 2S full charge, inside the +-12 V rating (A, AO3401A datasheet not on disk); "
              "the gate-source zener clamps hot-plug ringing / a wrong pack")
    net("VBAT_LOGIC_RAW", "Q10.D")
    net("VBAT_LOGIC", "Q10.S")
    net("LBAT_PFET_G", "Q10.G")
    r("10k", "LBAT_PFET_G", "GND", "Q10 gate resistor", "A", True, block="PWR")
    part("J15", "J", H2V, "2-pin header + shunt (or wires to an external switch)", "logic power switch",
         f"{USER} ('power switch'); the arm switch is NOT a power switch ({P} pins.arm_switch line 2158, {RM}:319)",
         True, "A", block="PWR",
         note="shunt on = logic on. A second toggle in the top cap needs a new hole = CAD change (user decision)")
    net("VBAT_LOGIC", "J15.1")
    net("VBAT_LOGIC_SW", "J15.2")
    c("10u 25V", "VBAT_LOGIC_SW", "GND", "buck input capacitor", "A", True, block="PWR", lib=C1206)
    part("J6", "J", H4V, "5 V buck module (>= 1 A, Vin 6-8.4 V) on a 1x4 2.54 mm header", "logic 5 V buck module",
         f"{P} configs.vtol.electronics.carrier.model '5 V buck for the logic' (line 1754); misc.items line 1788",
         True, "A", block="PWR",
         note="module pin order A: 1 VIN, 2 GND, 3 VOUT, 4 EN/NC - check against the purchased module before soldering")
    net("VBAT_LOGIC_SW", "J6.1")
    net("GND", "J6.2")
    net("+5V", "J6.3", source=f"{P} flight_computer.vcc_V 5.0 (line 2137); {RM}:244 (5 V logic rail)")
    nc("J6.4", "buck module EN / spare pin left open (A: module enabled by default)")
    c("22u", "+5V", "GND", "5 V rail bulk at the buck output", "A", True, block="PWR", lib=C1206)
    part("U6", "U", LDO, "AMS1117-3.3", "3.3 V LDO (sensors, radio, GPS, shifter LV side)",
         f"{P} configs.vtol.electronics.carrier.model 'AMS1117-3.3' (line 1754); {RM}:245-246", True, "F", block="PWR")
    net("+5V", "U6.VI")
    net("+3V3", "U6.VO", source=f"{RM}:245-246; {P} electronics.gps.power '3.3 V rail' (line 1012)")
    net("GND", "U6.GND")
    c("10u", "+5V", "GND", "AMS1117 input capacitor", "A (AMS1117 datasheet class value)", True, block="PWR", lib=C0805)
    c("22u 10V tantalum", "+3V3", "GND", "AMS1117 output capacitor (stability, ESR)",
      f"A (AMS1117 class datasheets: >= 22 uF tantalum, ESR in a stable window - check the bought vendor's sheet); "
      f"tantalum instead of a 1206 MLCC ({REV} E5 / F9)", True, block="PWR", lib=TANT)
    # logic battery divider on the FCU's free PF3 (new, unused by firmware)
    r("10k", "VBAT_LOGIC_SW", "FCU_LBAT_ADC", "logic battery divider top (NEW, firmware does not read PF3 yet)",
      "A: PF3/ADC3 is free on the FCU (PF0-PF2 used, PF4-PF7 = JTAG pins freed by the fuse) "
      f"({PG}:164-172; firmware/atmega128_fc/src/hal/adc.h:21-22); 8.4 V -> 2.08 V < 2.56 V ref; fitted by default so "
      f"the logic pack can be measured at PF3 / on the board ({REV} E1)", True, block="PWR")
    r("3.3k", "FCU_LBAT_ADC", "GND", "logic battery divider bottom (NEW)", f"A; fitted ({REV} E1)", True, block="PWR")
    c("100n", "FCU_LBAT_ADC", "GND", "logic battery divider filter (NEW)", f"A; fitted ({REV} E1)", True, block="PWR")
    net("FCU_LBAT_ADC", "U2.PF3", source="NEW (not in params / firmware): optional logic-battery sense on FCU PF3/ADC3")

    # ======================================================================= ground tie
    part("NT1", "NT", NETTIE, "net tie", "single tie point logic GND <-> propulsion GND (GND_PWR), at J12 pin 3",
         f"{USER} (common ground, one tie point); {RM}:247, 323 (GND common); {BH}:84, 91, 107", True, "A", block="PWR",
         note="place NT1 touching J12 (ESC lead) pin 3; servo return currents stay on GND_PWR (J8-J13 pin 3 -> J12 pin 3)")
    net("GND", "NT1.1")
    net("GND_PWR", "NT1.2", source=f"{USER}; {RM}:322-324 (servo 5 V rail from the ESC BEC, grounds common)")

    # ======================================================================= ESC + servos (FCU PWM, servo power from the ESC BEC)
    servo_src = f"{RM}:201-203, 315-317, 322-324 (servo V+ = ESC BEC rail, not the logic 5 V); {P} electronics.misc.items (line 995)"
    net("SERVO_5V", source=servo_src)
    outs = [("J8", "PB5", "VANE1", f"{P} pins.vane_1_servo PB5/OC1A; {BH}:23"),
            ("J9", "PB6", "VANE2", f"{P} pins.vane_2_servo PB6/OC1B; {BH}:24"),
            ("J10", "PB7", "VANE3", f"{P} pins.vane_3_servo PB7/OC1C; {BH}:25"),
            ("J11", "PE3", "VANE4", f"{P} pins.vane_4_servo PE3/OC3A; {BH}:26"),
            ("J12", "PE4", "ESC", f"{P} pins.esc PE4/OC3B (line 2152); {BH}:27"),
            ("J13", "PE5", "RELEASE", f"{P} pins.release_servo PE5/OC3C; {BH}:28; firmware/atmega128_fc/src/hal/pwm.c:49")]
    for j, pin, name, src in outs:
        pop = False if name == "RELEASE" else True
        role = {"ESC": "ESC lead (signal, BEC +5 V out, GND)", "RELEASE": "release servo header (VTOL has no release servo)"}.get(
            name, f"vane servo {name[-1]} lead (SIG, V+, GND)")
        part(j, "J", H3H, "1x3 2.54 mm right-angle header (S, +, -)", role, src, pop, "F", block="IO",
             note=("unpopulated: configs.vtol.note (line ~1700) removes the release servo, but the firmware still drives "
                   "PE5 (src/main.c:518) - footprint kept, header and series resistor DNP") if name == "RELEASE" else
             ("ESC red wire = BEC +5 V OUT into SERVO_5V (never into the logic +5V); ESC GND here is the single ground "
              "tie (NT1)") if name == "ESC" else None)
        net(f"{name}_PWM", f"U2.{pin}", source=src, kind="pwm")
        r("220", f"{name}_PWM", f"{name}_SIG", f"{name} signal series resistor (long lead, back-power limit)",
          "A: 220 Ohm series on a ~0.5 m servo lead (README 4.11 lead lengths 434-539 mm) limits back-power into an "
          "unpowered servo / ESC", pop, block="IO")
        net(f"{name}_SIG", f"{j}.1")
        net("SERVO_5V", f"{j}.2")
        net("GND_PWR", f"{j}.3")
    r("10k", "ESC_PWM", "GND", "ESC signal pull-down (motor off while the FCU boots / resets)",
      f"{RM}:316 (10 kOhm pull-down on the ESC signal)", True, "F", block="IO")
    c("220u 10V", "SERVO_5V", "GND_PWR", "servo rail bulk capacitor", "A (servo current spikes on the BEC rail)",
      True, block="IO", lib=CPOL)

    # ======================================================================= FCU I/O: arm switch, buzzer, LED done, V sense, log
    part("J16", "J", H2H, "1x2 right-angle header", "arming switch lead (top cap toggle, switch to GND)",
         f"{P} pins.arm_switch PC2 (line 2158); {P} configs.vtol.top_cap.switch_hole (line 1852); {BH}:40", True, "F",
         block="IO")
    net("ARM_SW_EXT", "J16.1")
    net("GND", "J16.2")
    r("1k", "ARM_SW_EXT", "FCU_ARM", "arm input series resistor (ESD / lead noise)", "A", True, block="IO")
    net("FCU_ARM", "U2.PC2", source=f"{P} pins.arm_switch (line 2158); firmware/atmega128_fc/src/hal/gpio.c:12-13 (internal pull-up)")
    r("10k", "FCU_ARM", "+5V", "arm input external pull-up (in addition to the internal one)", "A", True, block="IO")
    c("100n", "FCU_ARM", "GND", "arm input filter", "A", True, block="IO")
    part("J17", "J", JSTPH, "JST-PH 2.0 2-pin vertical (top entry)", "buzzer lead (5 V active buzzer in the misc box)",
         f"{P} pins.buzzer PC1 NPN driver (line 2157); configs.vtol.electronics.misc.items (line 1788); {BH}:39; vertical so "
         f"no mated plug points at the motor-lead joint zone below J12 ({REV} MF-3)",
         True, "F", block="IO")
    net("+5V", "J17.1")
    net("BUZ_N", "J17.2")
    part("Q9", "Q", NPN, "MMBT3904", "buzzer low-side driver", f"{P} pins.buzzer 'PC1 (NPN driver)'; {RM}:206", True, "F",
         block="IO")
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
    part("J18", "J", H4H, "1x4 right-angle header", "power-sense lead: V_in (6S bus), V_tether, V_buffer, buffer-current sign",
         f"{P} pins.vbat_adc / v_tether_adc / v_buffer_adc / buf_isign (line 2155, 2165-2167); {P} power.electrical_plant_ASSUMED.note",
         True, "F", block="IO",
         note="no GND pin on purpose: a second ground wire to the propulsion side would close a loop with the ESC lead "
              "ground; divider returns use the logic GND, tied once at NT1")
    divs = [("1", "VS_VIN", "FCU_VBAT_ADC", "PF0", f"{P} pins.vbat_adc 'PF0/ADC0 (divider 47k/4.7k, internal 2.56 V ref)' (line 2155); {PG}:164-165; {RM}:205, 318"),
            ("2", "VS_VTETHER", "FCU_VTET_ADC", "PF1", f"{P} pins.v_tether_adc (line 2165); {PG}:171 (ASSUMED until the carrier is routed)"),
            ("3", "VS_VBUFFER", "FCU_VBUF_ADC", "PF2", f"{P} pins.v_buffer_adc (line 2166); {PG}:172 (ASSUMED until the carrier is routed)")]
    for pin, ext, adc, mcu_pin, src in divs:
        net(ext, f"J18.{pin}", source=src)
        r("47k 1%", ext, adc, f"{mcu_pin} divider top", src, True, "F", block="IO")
        r("4.7k 1%", adc, "GND", f"{mcu_pin} divider bottom", src, True, "F", block="IO")
        c("100n", adc, "GND", f"{mcu_pin} divider filter", src, True, "F", block="IO")
        net(adc, f"U2.{mcu_pin}", source=src, kind="adc")
    net("VS_BUF_ISIGN", "J18.4", source=f"{P} pins.buf_isign (line 2167); {PG}:173")
    r("10k", "VS_BUF_ISIGN", "FCU_BUF_ISIGN", "buffer-current sign series resistor",
      f"A; 10k (not 1k) limits the clamp current while the hybrid-board signal level is undefined ({REV} E7)", True,
      block="IO")
    r("100k", "FCU_BUF_ISIGN", "GND", "buffer-current sign pull-down (battery / tether builds: reads low)", "A", True,
      block="IO")
    net("FCU_BUF_ISIGN", "U2.PC3", source=f"{P} pins.buf_isign PC3 (line 2167); {PG}:173-174")
    # OpenLog (FCU USART0 TX only, through a shifter channel)
    part("J5", "J", H6V, "OpenLog lead: 1x6 2.54 mm holes (lead wires soldered in, or a low right-angle header)",
         "OpenLog logger lead (microSD logger off the carrier)",
         f"{P} pins.log_tx PE1/TXD0 (line 2154); {RM}:204, 378-390; {BH}:35-36; no upright socket: an OpenLog standing in "
         f"it reaches 23+ mm above the board, over the 13.4 mm limit ({REV} MF-1)", True, "F", block="LOG",
         note="OpenLog header order A (SparkFun: GRN, RXI, TXO, VCC, GND, BLK) - check the clone's silkscreen. Only RXI, "
              "VCC, GND carry signals. The module does NOT fit flat on the carrier (its ~16.5 x 22 mm body, A, overlaps the "
              "GY-521 body by >= 4 mm in the only free area, over the DNP U5): where it goes (misc box on a lead suggested) "
              "is a user / CAD decision")
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
    bss138("Q1", "I2C_SCL_5V", "I2C_SCL_3V3", "NAV I2C SCL (400 kHz)", f"{P} nav.pins.i2c_scl (line 2175); {BH}:47-48; {RM}:199, 334",
           r_hv="4.7k", r_lv="10k", src_lv=lv_i2c)
    bss138("Q2", "I2C_SDA_5V", "I2C_SDA_3V3", "NAV I2C SDA", f"{P} nav.pins.i2c_sda; {BH}:47-48; {RM}:199, 334",
           r_hv="4.7k", r_lv="10k", src_lv=lv_i2c)
    net("I2C_SCL_5V", "U1.PD0", source=f"firmware/atmega128_fc/src/hal/twi.c:229 (internal pull-ups off: external pull-ups on the shifter)", kind="i2c")
    net("I2C_SDA_5V", "U1.PD1", kind="i2c")
    part("J1", "J", H8V, "1x8 2.54 mm (GY-521 soldered flat or socketed)", "IMU module GY-521 MPU-6050 [SZH-EK007]",
         f"devicemart no=1247052; {P} nav (line ~2170); {RM}:197 (module VCC 5 V, I2C 3.3 V, 0x68)", True, "A", block="NAV",
         note="pin order A: VCC, GND, SCL, SDA, XDA, XCL, AD0, INT - check the silkscreen. Mount flat, parallel to the "
              "carrier; set fc_config.json imu_mounting after placement")
    net("+5V", "J1.1")
    net("GND", "J1.2", "J1.7")
    net("I2C_SCL_3V3", "J1.3", source="NAV I2C 3.3 V side")
    net("I2C_SDA_3V3", "J1.4")
    nc("J1.5", "GY-521 XDA: auxiliary I2C master, unused")
    nc("J1.6", "GY-521 XCL: auxiliary I2C master, unused")
    bss138("Q3", "IMU_INT_5V", "IMU_INT_3V3", "GY-521 INT -> NAV PE6/INT6 (firmware polls; optional)",
           f"{P} nav.pins.imu_int 'not used, polled' (line 2177); {BH}:49; {RM}:199, 335 (shifter ch4 optional)",
           populate="optional (DNP)", pop_hv=True,
           src_hv=f"; HV pull-up fitted even with Q3 DNP so NAV PE6 never floats ({REV} E11)")
    net("IMU_INT_5V", "U1.PE6")
    net("IMU_INT_3V3", "J1.8")
    part("J2", "J", S6H, "1x6 2.54 mm right-angle socket (module lies flat)", "barometer module BMP280 [TYE-BMP29]",
         "devicemart no=15442588 (BMP280, 3.3 V, I2C/SPI, right-angle 6-pin header); replaces the GY-63 MS5611 "
         f"({USER})", True, "A", block="NAV",
         note="pin order A: VCC, GND, SCL, SDA, CSB, SDO - check the silkscreen. CSB high = I2C; SDO high = address 0x77 "
              f"(= the firmware's barometer address CAL_MS5611_ADDR 0x77, {PG}:964)")
    net("+3V3", "J2.1", "J2.5", "J2.6")
    net("GND", "J2.2")
    net("I2C_SCL_3V3", "J2.3")
    net("I2C_SDA_3V3", "J2.4")

    # ======================================================================= LoRa radio: PLNetworks PLM100 on two 1x12 1.27 mm sockets
    lora_src = (f"{DS} p.8-9 table 2 (pin description), p.8 fig 3 (pin map: J2 = pins 1-12, J3 = pins 13-24, "
                "HEADER12_1.27mm), p.12 fig 4 / p.13 fig 5 (rows 11.60 mm apart, 3.70 mm from the sides, 1.27 mm pitch)")
    part("J3", "J", S12, "1x12 1.27 mm socket (PLM100 pins 1-12)", "LoRa module PLM100 row J2 (module pins 1..12)",
         lora_src + f"; devicemart no=1289751; {USER}", True, "F", block="COM",
         note="socket pin n = module pin n. Placed 11.60 mm (centre to centre) from J4, pin 1 at the end AWAY from the "
              "antenna; pin 12 (ANT) beside the module's u.FL receptacle")
    part("J4", "J", S12, "1x12 1.27 mm socket (PLM100 pins 13-24)", "LoRa module PLM100 row J3 (module pins 13..24)",
         lora_src, True, "F", block="COM",
         note="socket pin k = module pin 12 + k (pin 1 = module 13 at the antenna end, pin 12 = module 24): J4 pin 1 is "
              "level with J3 pin 12")
    net("+3V3", "J3.1", source=f"{DS} p.8 table 2 pin 1 VDD 2.1-3.6 V; p.9 table 3 (3.3 V, max 40 mA); {UG} p.6 (TX up to 100 mA)")
    net("GND", "J3.3", "J4.1", "J4.3", source=f"{DS} p.8 table 2 pin 3 VeREF+ 'ADC unused -> Ground', pins 13 / 15 GND")
    c("10u", "+3V3", "GND", "PLM100 VDD bulk", f"{DS} p.15 fig 8 (C6 10 uF on DVCC)", True, "F", block="COM", lib=C0805)
    c("100n", "+3V3", "GND", "PLM100 VDD decoupling", f"{DS} p.15 fig 8 (C5 0.1 uF on DVCC)", True, "F", block="COM")
    # host UART = UART1 (module pins 19 TXD, 20 RXD) at the console port
    net("COM_LINK_TX", "U3.PD3", source=f"{P} com.pins.link_uart_tx 'PD3/TXD1 (to module RXD through a BSS138 channel)' (line 2213); {BH}:66",
        kind="uart")
    bss138("Q4", "COM_LINK_TX", "LORA_UART1_RXD", "COM PD3 TXD1 -> PLM100 UART1_RXD (pin 20)",
           f"{P} com.pins.link_uart_tx (line 2213); {RM}:254, 261; {UG} p.6 1.3 (host = UART1, 115200 default)")
    net("LORA_UART1_RXD", "J4.8", source=f"{DS} p.8 table 2 pin 20 UART1_RXD")
    net("LORA_UART1_TXD", "J4.7", "U3.PD2",
        source=f"{DS} p.8 table 2 pin 19 UART1_TXD; {P} com.pins.link_uart_rx 'PD2/RXD1 (module TXD, direct)' (line 2212); {BH}:65",
        note="direct: module 3.3 V output into the ATmega128A input (V_IH 3.0 V at 5 V, 0.3 V margin; module V_OH not in "
             "the datasheet - A, measure)", kind="uart")
    # RESET_N from COM PC6 (the sx127x reset pin, untouched by the lora_uart build)
    net("COM_LORA_RESET", "U3.PC6", source=f"{P} com.pins.lora_reset PC6 (line 2222); {BH}:74; firmware/atmega128_fc/src/hal/lora_uart.c:65-70 (lora_uart build drives only M0/M1, reads AUX)")
    bss138("Q5", "COM_LORA_RESET", "LORA_RESET_N", "COM PC6 -> PLM100 RESET_N (pin 2, active low)",
           f"{DS} p.8 table 2 pin 2 RESET_N; p.16 fig 9 (47k pull-up + 100 nF); {RM}:276 (sx127x RESET through a channel)")
    net("LORA_RESET_N", "J3.2")
    c("100n", "LORA_RESET_N", "GND", "PLM100 RESET_N capacitor", f"{DS} p.16 fig 9 (C31 100 nF)", True, "F", block="COM")
    net("LORA_GPIO5", "J3.11", source=f"{DS} p.16 fig 9 (GPIO5 = USER_KEY with 47 kOhm pull-up)")
    r("47k", "LORA_GPIO5", "+3V3", "PLM100 GPIO5 (USER_KEY / 'Default') pull-up - never asserted",
      f"{DS} p.16 fig 9 (R36 47k)", True, "F", block="COM")
    for pin, why in [("J3.4", "GPIO2 unused"), ("J3.5", "UART0_RXD (sensor UART) unused"),
                     ("J3.6", "UART0_TXD (sensor UART) unused"), ("J3.7", "J_TEST (JTAG test mode) - only the JTAG connector in fig 8"),
                     ("J3.8", "GPIO1 unused"),
                     ("J3.10", "GPIO4 unused"), ("J3.12", "ANT pin: only for an external SMA option; the antenna uses the module's u.FL (MM9329-2700) receptacle (DS p.8 table 1)"),
                     ("J4.2", "pin 14 GPIO6 unused"), ("J4.4", "pin 16 GPIO7 unused"), ("J4.5", "pin 17 I2C_SDA unused"),
                     ("J4.6", "pin 18 I2C_SCL unused"), ("J4.9", "pin 21 TMS (JTAG)"), ("J4.10", "pin 22 TCK (JTAG)"),
                     ("J4.11", "pin 23 TDO (JTAG)"), ("J4.12", "pin 24 TDI (JTAG)")]:
        nc(pin, f"PLM100 {why} ({DS} p.8-9 table 2)")
    for pin, why in [("PC3", "link_m0 (Ebyte M0): the firmware drives it low; PLM100 has no M0"),
                     ("PC4", "link_m1 (Ebyte M1): driven low by the firmware; PLM100 has no M1"),
                     ("PC5", "link_aux (Ebyte AUX): input with internal pull-up = 'ready' when open (lora_uart.c:69-70)"),
                     ("PB0", "lora_spi_nss (sx127x backend only)"), ("PB2", "lora_spi_mosi (sx127x only)"),
                     ("PB3", "lora_spi_miso (sx127x only)"), ("PE7", "lora_dio0 (sx127x only)")]:
        nc(f"U3.{pin}", f"COM {why} - {P} com.pins (line 2214-2222); left open for the PLM100 (firmware follow-up)")

    # ======================================================================= GPS lead (NEO-7M in the top cap) -> GNS USART1
    part("J7", "J", JSTGH, "JST-GH 1.25 mm 4-pin vertical (top entry)", "GPS lead (u-blox NEO-7M module in the top cap)",
         f"{P} payload_module.gps_pod.connector_pocket.connector 'JST-GH 1.25 mm 4-pin'; configs.vtol.top_cap.gps "
         f"(line 1830-1850); devicemart no=10895400; vertical: the lead drops from the top-cap hole and plugs in from above, "
         f"no plug beyond the -y edge toward the 225 deg joint screw ({REV} MF-6)", True, "A", block="GNS",
         note="carrier pin order (A, the lead is user-made): 1 VCC, 2 GPS TXD (-> GNS RX), 3 GPS RXD (<- GNS TX), 4 GND; "
              "the GPS end follows the NEO-7M board silkscreen (A: VCC, GND, TXD, RXD, PPS). PPS not wired (firmware unused)")
    net("GPS_VCC", "J7.1", source=f"{P} electronics.gps.power '3.3 V rail, ~45 mA' (line 1012)")
    net("GND", "J7.4", "J7.MP")
    part("JP1", "JP", SJ3, "solder jumper, 1-2 bridged", "GPS supply select: 1-2 = 3.3 V (default, params), 2-3 = 5 V",
         f"{P} electronics.gps.power (line 1012); NEO-7M board supply '3.3-5 V' typical (A, not on the product page)",
         True, "A", block="GNS")
    net("+3V3", "JP1.A")
    net("GPS_VCC", "JP1.C")
    net("+5V", "JP1.B")
    c("10u", "GPS_VCC", "GND", "GPS supply bulk at the connector", "A", True, block="GNS", lib=C0805)
    net("GNS_GPS_TX", "U4.PD3", source=f"{P} gns.pins.gps_tx 'PD3/TXD1 (to GPS RX through the level shifter)' (line 2268); {BH}:98",
        kind="uart")
    bss138("Q6", "GNS_GPS_TX", "GPS_RXD_3V3", "GNS PD3 TXD1 -> GPS RXD (38400 8N1)",
           f"{P} gns.pins.gps_tx (line 2268); electronics.gps.interface 'BOTH lines level-shifted, GPS RX abs max 3.6 V' (line 1011)")
    net("GPS_RXD_3V3", "J7.3")
    net("GNS_GPS_RX", "U4.PD2", source=f"{P} gns.pins.gps_rx (line 2267); {BH}:97; firmware/atmega128_fc/src/hal/gps_uart.c:299 (RXD1 pull-up)",
        kind="uart")
    bss138("Q7", "GNS_GPS_RX", "GPS_TXD_3V3", "GPS TXD -> GNS PD2 RXD1",
           f"{P} gns.pins.gps_rx (line 2267); electronics.gps.interface (line 1011)")
    net("GPS_TXD_3V3", "J7.2")

    # ======================================================================= review additions (appended so refdes stay stable)
    dz = part(None, "D", ZENER, "10 V zener (BZT52C10 class), SOD-123", "Q10 gate-source clamp (cathode = source)",
              f"{REV} E4 (AO3401A Vgs abs. max +-12 V, A: datasheet not on disk; hot-plug ringing up to ~2 x Vbat, E); "
              "part A", True, "A", block="PWR")
    net("VBAT_LOGIC", f"{dz}.K")
    net("LBAT_PFET_G", f"{dz}.A")
    for name in ("VANE1", "VANE2", "VANE3", "VANE4"):
        r("10k", f"{name}_PWM", "GND", f"{name} PWM pull-down (servo line low while the FCU boots / is held in reset)",
          f"{REV} E8 (pins Hi-Z until pwm.c:48-49 sets them; J25 holds the FCU in reset while the COM is programmed); "
          f"same as the ESC pull-down ({RM}:316)", True, "A", block="IO")
    r("47k", "LORA_GPIO3", "+3V3", "PLM100 GPIO3 (UART1 RX enable when AT+GPIO=4) pull-up - RX stays enabled",
      f"{DS} p.8 fig 3 pin map (pin 9 = GPIO3); {UG} p.7 (GPIO3 HIGH = UART1 RX enabled), p.13 (+GPIO, bit2); "
      f"{REV} E10", True, "F", block="COM")
    net("LORA_GPIO3", "J3.9", source=f"{DS} p.8 fig 3 (pin 9 GPIO3); {UG} p.7")


# ----------------------------------------------------------------------------------------------- checks
MCU_PIN_NC_REASON = "unused ATmega128A I/O (left open; firmware may enable its pull-up)"
THREE_V3_ONLY_INPUTS = ["J2.3", "J2.4", "J2.5", "J2.6", "J4.8", "J3.2", "J3.11", "J7.3", "J5.2", "J1.3", "J1.4"]
TX_DRIVERS = ["U2.PD3", "U3.PE1", "U1.PD3", "U4.PE1", "U3.PD3", "U4.PD3", "U2.PE1", "J4.7", "J7.2", "U1.PB0", "U1.PB1",
              "U1.PB2", "U2.PB3"]
PATHS = [  # (description, from member, to member)
    ("ilink FCU TXD1 -> COM RXD0", "U2.PD3", "U3.PE0"), ("ilink COM TXD0 -> FCU RXD1", "U3.PE1", "U2.PD2"),
    ("glink NAV TXD1 -> GNS RXD0", "U1.PD3", "U4.PE0"), ("glink GNS TXD0 -> NAV RXD1", "U4.PE1", "U1.PD2"),
    ("nlink SS NAV -> FCU", "U1.PB0", "U2.PB0"), ("nlink SCK NAV -> FCU", "U1.PB1", "U2.PB1"),
    ("nlink MOSI NAV -> FCU", "U1.PB2", "U2.PB2"), ("nlink MISO FCU -> NAV", "U2.PB3", "U1.PB3"),
    ("LoRa COM TXD1 -> PLM100 UART1_RXD (pin 20)", "U3.PD3", "J4.8"),
    ("LoRa PLM100 UART1_TXD (pin 19) -> COM RXD1", "J4.7", "U3.PD2"),
    ("LoRa COM PC6 -> PLM100 RESET_N (pin 2)", "U3.PC6", "J3.2"),
    ("GPS GNS TXD1 -> GPS RXD", "U4.PD3", "J7.3"), ("GPS GPS TXD -> GNS RXD1", "J7.2", "U4.PD2"),
    ("log FCU TXD0 -> OpenLog RXI", "U2.PE1", "J5.2"),
    ("I2C SCL NAV -> GY-521", "U1.PD0", "J1.3"), ("I2C SCL NAV -> BMP280", "U1.PD0", "J2.3"),
    ("I2C SDA NAV -> GY-521", "U1.PD1", "J1.4"), ("I2C SDA NAV -> BMP280", "U1.PD1", "J2.4"),
    ("IMU INT -> NAV PE6 (optional)", "J1.8", "U1.PE6"),
    ("vane 1 PWM", "U2.PB5", "J8.1"), ("vane 2 PWM", "U2.PB6", "J9.1"), ("vane 3 PWM", "U2.PB7", "J10.1"),
    ("vane 4 PWM", "U2.PE3", "J11.1"), ("ESC PWM", "U2.PE4", "J12.1"), ("release PWM (DNP)", "U2.PE5", "J13.1"),
    ("V_in sense -> PF0", "J18.1", "U2.PF0"), ("V_tether sense -> PF1", "J18.2", "U2.PF1"),
    ("V_buffer sense -> PF2", "J18.3", "U2.PF2"), ("buffer-current sign -> PC3", "J18.4", "U2.PC3"),
    ("arm switch -> PC2", "J16.1", "U2.PC2"), ("buzzer PC1 -> NPN base", "U2.PC1", "Q9.B"),
    ("logic battery sense -> PF3 (optional)", "VBATSW", "U2.PF3"),
]
for role, u in MCU_REFS.items():
    j = ISP_REFS[role]
    PATHS += [(f"{role} ISP MOSI -> PE0/PDI", f"{j}.MOSI", f"{u}.PE0"), (f"{role} ISP MISO <- PE1/PDO", f"{u}.PE1", f"{j}.MISO"),
              (f"{role} ISP SCK -> PB1", f"{j}.SCK", f"{u}.PB1"), (f"{role} ISP RST -> RESET", f"{j}.~{{RST}}", f"{u}.~{{RESET}}")]


def check() -> dict:
    report: dict = {}
    # resolve members, pin uniqueness
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
        if len({x.split('(')[0] for x in n["members"]}) < 2 and len(set(res)) < 2:
            ERRORS.append(f"net {n['name']} has fewer than 2 members: {n['members']}")
        if not n["sources"] and n["name"] not in RAILS and not n["name"].startswith(tuple(MCU_REFS)) \
                and not any(n["name"].startswith(f"{k}_") for k in MCU_REFS):
            n["sources"].append("derived: support / pull-up / series-resistor node (see the part entries)")
    # NC resolution
    nc_keys: dict[str, str] = {}
    for m, why in NC.items():
        ref = m.split(".")[0]
        for num in resolve(m):
            nc_keys[f"{ref}.{num}"] = why
    # every pin assigned
    unassigned = []
    for ref, p in PARTS.items():
        sym = PROBE["symbols"][p["symbol"]]
        fp_pads = set(PROBE["footprints"][p["footprint"]]["pads"])
        sym_pins = {pp[0] for pp in sym["pins"]}
        if p["symbol"] != "Mechanical:MountingHole" and fp_pads != sym_pins:
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

    # graph for paths: nodes = nets; edges through 2-terminal series parts (R) and BSS138 S<->D; rails blocked
    def member_net(m: str) -> str | None:
        ref = m.split(".")[0]
        nums = resolve(m)
        return pin_net.get(f"{ref}.{nums[0]}") if nums else None

    adj: dict[str, set[str]] = defaultdict(set)
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

    def reach(start: str, through_shifter: bool = True) -> set[str]:
        seen, todo = {start}, [start]
        while todo:
            cur = todo.pop()
            for nx in adj[cur]:
                if nx in seen:
                    continue
                if not through_shifter:
                    via_bss = any(PARTS[ref]["symbol"] == "Transistor_FET:BSS138" and
                                  {pin_net.get(f"{ref}.2"), pin_net.get(f"{ref}.3")} == {cur, nx} for ref in PARTS)
                    if via_bss:
                        continue
                seen.add(nx)
                todo.append(nx)
        return seen

    path_rows = []
    for desc, a, b in PATHS:
        na = "VBAT_LOGIC_SW" if a == "VBATSW" else member_net(a)
        nb = member_net(b)
        if a == "VBATSW":   # rail start: one hop through the divider resistor
            ok = any(PARTS[ref]["symbol"] == "Device:R" and {pin_net.get(f"{ref}.1"), pin_net.get(f"{ref}.2")} == {na, nb}
                     for ref in PARTS)
            path_rows.append({"path": desc, "from": "VBAT_LOGIC_SW", "to": b, "ok": ok})
        else:
            ok = bool(na and nb and nb in reach(na))
            path_rows.append({"path": desc, "from": f"{a} [{na}]", "to": f"{b} [{nb}]", "ok": ok})
        if not ok:
            ERRORS.append(f"signal path broken: {desc} ({a} -> {b})")
    report["signal_paths"] = path_rows
    # driver conflicts
    for d in TX_DRIVERS:
        comp = reach(member_net(d) or "")
        others = [o for o in TX_DRIVERS if o != d and member_net(o) in comp]
        if others:
            ERRORS.append(f"driver conflict: {d} reaches other drivers {others}")
    report["driver_conflicts"] = 0
    # 3.3 V-only inputs never reach an ATmega pin without a shifter
    mcu_nets = {pin_net[k] for k in pin_net if k.split(".")[0] in MCU_REFS.values()}
    bad = []
    for m in THREE_V3_ONLY_INPUTS:
        n0 = member_net(m)
        if n0 is None:
            continue
        comp = reach(n0, through_shifter=False)
        if comp & mcu_nets:
            bad.append(f"{m} reaches {sorted(comp & mcu_nets)}")
    for b_ in bad:
        ERRORS.append(f"3.3 V-only input reaches a 5 V ATmega128A pin without a shifter: {b_}")
    report["three_v3_inputs_checked"] = len(THREE_V3_ONLY_INPUTS)
    report["bss138_channels"] = sorted(k for k, p in PARTS.items() if p["symbol"] == "Transistor_FET:BSS138")
    return report


def natural(s: str) -> list:
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", s)]


# ----------------------------------------------------------------------------------------------- narrative sections
def narrative() -> dict:
    return {
        "board": {
            "outline_mm": {"x": [-38.0, 38.0], "y": [-35.0, 35.0], "size": [76.0, 70.0], "corner_radius_mm": 1.0,
                           "label": "F (size) / A (corner radius)",
                           "source": f"{P} configs.vtol.electronics.carrier.dims_lwh [76, 70, 15] (line 1758); {CADR} parts.carrier.bbox_mm "
                                     "[[-38,-35,327.1],[38,35,342.1]]"},
            "frame": "board origin = vehicle axis; board +x = CAD +x (ESC / misc side, motor-lead hole at theta 0 deg), "
                     "+y = CAD +y (theta 90 deg); component side = top = +Z (up, toward the tray). "
                     f"Source: cad/layout.py:1110-1131 (centred, edges along x / y)",
            "layers": 2, "thickness_mm": 1.6, "copper": "1 oz (A)",
            "stackup_note": f"user chose 2 layers; {P} carrier.note (line 1754) said 'a 4-layer layout ... is the PCB round's job' - "
                            "2 layers with a near-solid bottom GND pour (A)",
            "height_limit_top_mm": {"value": 13.4, "label": "F/E",
                                    "why": "carrier envelope 15 mm incl. components minus 1.6 mm board "
                                           f"({P} carrier.dims_lwh; {CADR} carrier z 327.1-342.1, tray bottom z 344.1)"},
            "bottom_side": "no components: the board sits on 3 mm double-sided foam tape (carrier.foam_t, line 1763); "
                           "THT leads flush-cut to <= 1.0 mm after soldering (A; 1.4-1.8 mm untrimmed would press 47-60 % "
                           f"into the foam, {REV} MF-8)",
            "mounting": {"holes": "none",
                         "method": f"3 mm foam tape + ONE cable tie running across the board top between the intake-bulkhead "
                                   f"vent holes at 90 / 270 deg (d 8 mm at r 40 mm: centres (0, +40) / (0, -40)) - {P} "
                                   "configs.vtol.electronics.carrier.note (line 1754), configs.vtol.intake_module "
                                   "(line 1688-1690), intake_module.vent_hole_r / vent_hole_d (line 814-816)",
                         "tie_band": {"x_mm": [-3.0, 3.0], "y_mm": [-35.0, 35.0], "rule": "no components and no tall "
                                      "parts inside the band (traces / vias allowed); A width for a 2.5-3.6 mm tie"}},
            "keepouts": [
                {"what": "motor-lead bulkhead hole 20 x 11 at r 42-53, theta 0 deg + bullet joint zone u 42-54.5 (4 mm from the "
                         "+x edge)", "rule": "+x edge, |y| < 14 mm: no lead connectors, no I2C / radio / crystal parts "
                                             "(A margin, EMI from the 100 A motor leads)",
                 "source": f"{P} intake_module.wire_channel.bulkhead_hole_* ; motor_leads.joint_zone; {CADR} "
                           "extra.motor_leads.joint_zone_clearance_mm.carrier = 4.0"},
                {"what": "board corners 5.3 mm inside the intake spigot bore", "rule": "no part overhangs the outline",
                 "source": f"{P} carrier.note (line 1754)"},
                {"what": "tray plane x = 14.6 (2 mm above the carrier envelope)", "rule": "13.4 mm top height limit everywhere",
                 "source": f"{P} configs.vtol.avionics_module.tray_plane_x (line 1745); {CADR} parts.avionics_tray.bbox_mm"},
            ],
            "edges": [
                {"edge": "-x (x = -38)", "connectors": ["J13 (DNP, top slot)", "J8", "J9", "J10", "J11"],
                 "why": f"vane-servo leads enter the bay through the bulkhead servo hole at theta 180 deg, r 50-56 "
                        f"({P} servo_leads.gather_theta_deg 180 / bulkhead_hole_r [50, 56], line 763-768); "
                        "right-angle headers facing -x. The fitted vanes 1-4 take the four lower slots (pin 1 at y +19.13 / "
                        "+10.41 / +1.69 / -7.03); the top slot (y +27.85), whose mated plug would touch the intake spigot "
                        f"bore (r 57.17 > 57.0, E), carries the DNP release header J13 ({REV} MF-2)"},
                {"edge": "-x, y < -12", "connectors": ["J18"],
                 "why": "power-sense lead: 6S pack on the tray's -x face, tether / hybrid kit boxes on the -x side "
                        f"({P} power.tether_kit_ASSUMED.boxes; {CADR} parts.battery x -37.4..13.6) - A"},
                {"edge": "+x (x = +38), y > +14", "connectors": ["J12 (ESC lead + NT1 tie, top slot)", "J17 (buzzer, below)",
                                                                  "J14 (logic battery, vertical, corner)"],
                 "why": f"ESC on the tray's +x face, misc box (buzzer, wiring junction) above it ({CADR} parts.esc x 15.6..24.6, "
                        "misc_electronics x 15.6..27.6); |y| < 14 kept clear for the motor leads. J12 takes the upper slot "
                        "(pins y +26.15..+21.07) so its mated plug stays >= 2 mm from the motor-lead joint zone; J17 sits below "
                        f"it ({REV} MF-3). Logic battery location not in CAD (user decision; misc box suggested, A)"},
                {"edge": "+y (y = +35), x > +5", "connectors": ["J16 (arm switch)"],
                 "why": f"arming switch in the top cap at theta 90 deg, r 40 ({P} configs.vtol.top_cap.switch_hole, line 1852): "
                        "its lead drops along the +y wall; x > +5 keeps the tie band free"},
                {"edge": "-y (y = -35), x < -5", "connectors": ["J7 (GPS, JST-GH vertical, plugged from above)"],
                 "why": f"GPS lead drops through the top-cap lead hole at theta 230 deg, r 37 = (x -23.8, y -28.3) "
                        f"({P} configs.vtol.top_cap.gps.lead_hole, line 1840; cad/layout.py:1995-1999 routes it straight down)"},
                {"edge": "-x / +y quadrant", "connectors": ["J3", "J4 (PLM100)"],
                 "why": "antenna end (module pins 12 / 13, u.FL) toward the +y / -x corner, away from the ESC (+x) and the "
                        f"motor leads (0 deg); antenna hangs up the bay wall ({RM}:286-291, {P} electronics.radio_log.note) - A"},
                {"edge": "interior (top)", "connectors": ["J19-J23 ISP", "J24 / J25 reset-hold", "J15 power shunt",
                                                          "J5 OpenLog lead (holes, no upright socket)",
                                                          "J1 GY-521", "J2 BMP280", "J6 buck"],
                 "why": "ISP reachable after lifting the avionics module off the intake "
                        f"({RM}:655-659); GY-521 near the board centre (vibration, CG), outside the tie band (A)"},
            ],
        },
        "power": {
            "decision": f"{USER} (Korean: '전원은 칩셋 전원용 배터리를 하나 추가한다고 가정하고 진행해줘'): logic powered by ONE added "
                        "logic battery; NO 6S-to-logic converter on the carrier (the old '5 V logic UBEC' from the 6S of "
                        f"{RM}:166, 212, 244 is dropped)",
            "tree": [
                "J14 logic battery (2S LiPo) -> Q10 AO3401A reverse-polarity P-FET -> VBAT_LOGIC -> J15 power shunt / switch -> "
                "VBAT_LOGIC_SW -> J6 5 V buck module -> +5V",
                "+5V -> 4 x ATmega128A VCC / AVCC (LC), GY-521 VCC, OpenLog VCC, buzzer, shifter HV pull-ups, U6 AMS1117-3.3 input",
                "+5V -> U6 AMS1117-3.3 -> +3V3 -> PLM100 VDD, BMP280 VCC, GPS VCC (JP1 default), BSS138 gates + LV pull-ups",
                "ESC BEC 5 V / 7 A (J12 pin 2) -> SERVO_5V -> J8-J11 (+ J13 DNP) pin 2 - separate from +5V "
                f"({RM}:212, 322-324; {P} electronics.esc 'BEC 5V/7A' line 942, misc.items line 995)",
                "6S propulsion bus is only SENSED (J18 -> 47k / 4.7k dividers -> FCU PF0 / PF1 / PF2), never powers the logic",
            ],
            "input_range_V": {"logic_battery_2S": [6.0, 8.4], "label": "F (LiPo 3.0-4.2 V/cell) / A (3.0 V cut-off)",
                              "propulsion_sensed": [0.0, 28.16],
                              "propulsion_note": f"divider full scale {P} power.electrical_plant_ASSUMED.adc_full_scale_v 28.16 "
                                                 f"(line ~2028); 6S max 25.2 V ({RM}:205); tether set point 25.2 V (power.supply.v_set)"},
            "load_estimate": {
                "label": "E (datasheet-class values, A where marked; nothing measured)",
                "rail_5V_mA": {"ATmega128A x4 @16 MHz (25 mA each, A)": 100, "status LEDs x4 (3 mA)": 12,
                               "GY-521 (MPU-6050 3.9 mA + LED + LDO, A)": 6, "OpenLog (avg 20 / peak 80, A)": [20, 80],
                               "buzzer (A)": [0, 30], "AMS1117 quiescent (A)": 5, "3.3 V rail (below)": [62, 148]},
                "rail_3V3_mA": {"PLM100 (RX 14 typ, TX 40 DS p.10 / 100 UG p.6)": [14, 100],
                                f"NEO-7M board ({P} electronics.gps.power ~45 mA, NEO-6M value, A)": 45,
                                "BMP280 module (A)": 1, "shifter / I2C pull-ups (E)": 2},
                "rail_5V_total_mA": {"typical": 205, "peak": 381},
                "rail_3V3_total_mA": {"typical": 62, "peak": 148},
                "battery_side": "buck efficiency 85 % (A): typical 5 V x 0.205 A / 0.85 = 1.21 W -> 0.163 A at 7.4 V; "
                                "peak 2.24 W -> 0.30 A at 7.4 V, 0.37 A at 6.0 V",
                "ldo_dissipation_W": "(5.0 - 3.3) x 0.148 = 0.25 W peak in SOT-223 (E)",
                "servo_rail": "not on the logic battery (ESC BEC 5 V / 7 A)",
            },
            "battery_choice": {
                "pick": "2S LiPo 300 mAh (7.4 V nominal, 6.0-8.4 V), JST-PH lead",
                "label": "A (class chosen here; user decides the exact pack)",
                "why": ["2S min 6.0 V > buck Vout 5.0 V + ~0.8 V module dropout (A) = 5.8 V: the 5 V rail stays regulated "
                        "over the whole discharge without a boost stage; BOD 4.0 V on every chip (lfuse 0x3F, "
                        f"{RM}:416) keeps margin",
                        "AO3401A drop at 0.37 A ~ 0.02 V (Rds(on) ~60 mOhm, A) - a Schottky (~0.4 V) would eat the margin",
                        "300 mAh x 0.8 usable / 0.163 A = 1.47 h typical (E): pad wait + several flights "
                        f"({P} mission.vtol.power_rules_ASSUMED.tether_endurance.max_flight_s 180 s)",
                        "peak 0.37 A = 1.2 C of a 300 mAh pack (E)"],
                "alternative": "1S LiPo 300 mAh + 5 V boost module: ~8-9 g pack + ~1 g boost (E) vs ~20 g for 2S (E); "
                               "boost peak input ~0.68 A at 3.3 V (E); needs a different J6 module - user decision",
                "mass_g": {"value": 20.0, "label": "E (typical 2S 300 mAh catalogue class, incl. lead)",
                           "not_in_params": True,
                           "note": "params.json mass budgets do not hold this pack; the VTOL battery configuration is "
                                   "already ~3.7 g over its 1,650 g target -> ~24 g over with the pack (E) - user decision; "
                                   "dropping the 6S->5 V UBEC from the misc box (25 g ASSUMED lump, "
                                   f"{P} configs.vtol.electronics.misc, line 1787) may offset a few grams (E)"},
            },
            "power_switch": "J15: a removable shunt (or wires to an external toggle) in series after Q10. Logic off = shunt "
                            "removed or battery unplugged. The arming switch (J16, FCU PC2) is NOT a power switch.",
            "reverse_polarity": "Q10 AO3401A high-side P-FET, gate via 10 kOhm to GND",
        },
        "grounding": {
            "nets": {"GND": "logic ground (logic battery -, all chips, modules, dividers, arm switch, buzzer, GPS)",
                     "GND_PWR": "propulsion-side ground on the carrier: J12 ESC lead pin 3 (= ESC BEC ground = 6S -), "
                                "servo headers pin 3, servo bulk capacitor"},
            "single_tie_point": "NT1 net tie placed at J12 (ESC lead) pin 3. It is the ONLY connection between the logic "
                                "battery's ground and the propulsion side. Reason: the ESC / servo PWM signals need a common "
                                f"reference ({RM}:247, 323; {BH}:84, 91, 107 'common ground')",
            "rules": ["J18 (power sense) has NO ground pin: the 47k / 4.7k dividers return to logic GND and see the bus "
                      "negative through NT1 (a second ground wire would close a loop with the ESC lead ground through the "
                      "ESC's 100 A negative path)",
                      "servo return currents flow J8-J13 pin 3 -> GND_PWR -> J12 pin 3 and never through the logic GND",
                      "with the ESC lead unplugged the logic ground floats from the propulsion side (no current path) - "
                      "bench-safe",
                      "measurement error: V_in sees the drop of the ESC's internal negative path between the pack and the "
                      "BEC ground (E: < 0.1 V at 100 A for a few mOhm, A)"],
        },
        "level_shifting": {
            "principle": f"MCU -> 3.3 V-only device through a BSS138 channel; I2C both ways through channels; device -> MCU "
                         f"direct only where the source says so ({RM}:254-255; SPEC.md:1178, 1239)",
            "channels": [
                {"q": "Q1", "signal": "NAV PD0 SCL <-> GY-521 / BMP280 SCL", "pullups": "4.7k (5 V) / 10k (3.3 V, modules add theirs)", "populate": True},
                {"q": "Q2", "signal": "NAV PD1 SDA <-> GY-521 / BMP280 SDA", "pullups": "4.7k (5 V) / 10k (3.3 V, modules add theirs)", "populate": True},
                {"q": "Q3", "signal": "GY-521 INT -> NAV PE6 (firmware polls)", "pullups": "10k (fitted, PE6 never floats) / 10k (DNP)", "populate": "optional (DNP)"},
                {"q": "Q4", "signal": "COM PD3 TXD1 -> PLM100 UART1_RXD (pin 20)", "pullups": "10k / 10k", "populate": True},
                {"q": "Q5", "signal": "COM PC6 -> PLM100 RESET_N (pin 2)", "pullups": "10k / 10k", "populate": True},
                {"q": "Q6", "signal": "GNS PD3 TXD1 -> GPS RXD", "pullups": "10k / 10k", "populate": True},
                {"q": "Q7", "signal": "GPS TXD -> GNS PD2 RXD1", "pullups": "10k / 10k", "populate": True},
                {"q": "Q8", "signal": "FCU PE1 TXD0 -> OpenLog RXI", "pullups": "10k / 10k", "populate": True},
            ],
            "count": {"populated": 7, "optional": 1, "total_footprints": 8},
            "direct_3v3_to_5v": ["PLM100 UART1_TXD (pin 19) -> COM PD2 RXD1: 3.3 V vs V_IH 3.0 V, 0.3 V margin "
                                 f"({P} com.pins.link_uart_rx 'direct', line 2212; {P} electronics.gps.interface notes the same "
                                 "0.3 V margin)"],
            "no_shifter_links": f"nlink, ilink, glink: all chips at 5 V ({BH}:84, 92, 107)",
        },
        "per_chip_support": {
            "each_mcu": ["16 MHz crystal (3225 SMD, CL 16-18 pF A) + 2 x 22 pF to GND (crystal ground pads to GND)",
                         "RESET: 10 kOhm to +5V + 100 nF to GND; ISP pin 5",
                         "100 nF on VCC pin 21 and on VCC pin 52, 10 uF bulk",
                         "AVCC (pin 64): 10 uH series from +5V + 100 nF ('AVCC(LC)')",
                         "AREF (pin 62): 100 nF only - internal 2.56 V reference, never an external voltage on AREF",
                         "PEN (pin 1): tied to +5V (A: PEN low at power-on would enter SPI programming mode)",
                         "GND pins 22 / 53 / 63 to the GND pour",
                         "AVR ISP 2x3 header: 1 MISO = PE1/PDO, 2 VCC, 3 SCK = PB1, 4 MOSI = PE0/PDI, 5 RST, 6 GND",
                         "status LED + 1 kOhm (NAV PC1; FCU / COM / GNS PC0)"],
            "fuses": f"lfuse 0x3F, hfuse 0xCF, efuse 0xFF for every chip ({PIO}:51-56; {RM}:414-418): ext. crystal, "
                     "BOD 4.0 V, JTAG off (frees PF4-PF7), M103C off",
            "source": f"{RM}:196, 310-311, 332-333, 345-346; {BH}:37; {P3} section 6",
        },
        "isp_procedure": [
            {"chip": "NAV (J19)", "before": "nothing", "why": f"{RM}:649 (NAV PE0 / PE1 unused in flight; FCU SS pull-up idles the FCU SPI)"},
            {"chip": "FCU (J20)", "before": "shunt on J24 (NAV held in reset); OpenLog may stay (Q8 isolates RXI; it may log "
                                            "garbage)", "why": f"{RM}:650; {P} pins.nlink_sck (line 2162): NAV drives FCU PB1 = ISP SCK"},
            {"chip": "COM (J21)", "before": "shunt on J25 (FCU held in reset)",
             "why": f"{RM}:651; {BH}:59-62: COM PE0 / PE1 = ilink; the 1 kOhm limits the fight to 5 mA but the programmer "
                    "may still mis-read"},
            {"chip": "GNS (J22)", "before": "shunt on J24 (NAV held in reset)",
             "why": f"{BH}:99-100 (GNS PE0 / PE1 = ISP = glink); README 0.9 'unplug the glink to program the GNS'"},
            {"chip": "SPARE (J23, DNP)", "before": "nothing (own ISP pins only)", "why": "no shared nets"},
            {"chip": "all", "before": "remove every shunt after programming; power the board from the logic battery and set "
                                      "the USBasp to 'target powered' (do not feed 5 V from both)",
             "why": f"{RM}:653"},
        ],
        "unpopulated_5th_mcu": {
            "refs": "U5 + Y, C, L, R, LED of block SPARE + J23",
            "connected": "+5V, GND, crystal, RESET (pull-up / cap), AVCC (LC), AREF cap, decoupling, its own ISP header, LED",
            "not_connected": f"the P3 nets ({P3} section 6: elink A PE1 <-> B PE0 / B PE1 <-> A PE0 through 1 kOhm, glink split "
                             "B PD3 -> GNS PE0, MS5611 / barometer bus moved to B PD0 / PD1, P3b BAT54C wired-AND + 10 kOhm). "
                             "Reason: P3 is a fallback with a hardware-measurement trigger (SPEC.md 10.6.1; "
                             "docs/v0.9_펌웨어_결과.md 8장); the footprint is for that study only.",
        },
    }


def firmware_followup() -> list[dict]:
    return [
        {"area": "COM radio driver (PLM100 replaces Ebyte E22 / SX1276)", "owner": "firmware round (user decides)",
         "items": [
             f"Host port = PLM100 UART1 (pins 19 TXD / 20 RXD), ASCII AT command interface, 115200 8N1 default "
             f"({UG} p.6 1.3, table 1); the firmware's LINK_UART_BAUD is 9600 ({PG}:879): set AT+CBAR=9600 once "
             f"(supported 9600-115200, {UG} p.9) or change link.uart_baud",
             f"The module boots in AT mode (no transparent mode): TX = AT+DATA=<string> (ASCII) or AT+BDAT=<len><hex> "
             f"(hex text, first byte = length) ({UG} p.13-14); RX arrives as 'RECV:<id>:<RSSI>:<data>' lines on the "
             f"console, binary only with AT+RXDP=1 (HEX display) ({UG} p.10, 14); TX result 'TX DONE(n)(SUCCESS|FAIL)'",
             f"Echo is on by default: AT+ECHO=0 ({UG} p.10); destination AT+DEST=<last 3 EUI bytes> or FFFFFF broadcast "
             f"({UG} p.13); retries AT+RTRY=0 (the link protocol retries itself)",
             f"RF settings: AT+FREQ=9 -> 920.3 + 9 x 0.2 = 922.1 MHz = link.sx127x.freq_hz 922100000 ({UG} p.11); "
             f"AT+SPFA=7, AT+BWTH=1 (125 kHz in V2.2 numbering; V1.4 used 0 = 125 kHz - check the module's firmware "
             f"version, {UG} p.12 vs {CG} p.10), AT+CORA=1 (4/5), AT+OPWR=14 (Korean max 14 dBm, {UG} p.12), "
             f"AT+PRMB=8 (16 when mixing P/N M100C(P)E with W100C(P)E at SF7, {UG} p.2)",
             "The binary link_proto frames (0xC5 0x5C ... CRC) must be hex-encoded for AT+BDAT -> 2x bytes on the UART; "
             "the IEEE 802.15.4e MAC overhead is not documented: sim/link_model.py EBYTE_* constants (line 78-80) and "
             "the 2 Hz poll / TELEM rate (README 0.5: lora_uart 2.014 Hz, thin) must be re-derived and measured",
             "M0 / M1 (PC3 / PC4) and AUX (PC5) have no PLM100 counterpart: left open (firmware still drives M0 / M1 low "
             "and reads AUX = ready through its pull-up - harmless). RESET_N is on PC6 (the sx127x reset pin, untouched "
             "by the lora_uart build): the driver may pulse it low at boot; AT+RSET reboot takes ~2-3 s (UG p.9)",
             "KC certification of the PLM100 in the 920 MHz band: check (A) before flight"]},
        {"area": "ground station", "owner": "firmware / tools round",
         "items": ["the ground side needs a second PLM100 + a USB-UART at the console baud (AT protocol both ends)",
                   "tools/ground_station.py must send AT+BDAT / parse RECV lines; firmware/ground_bridge (SX127x only, "
                   "firmware/ground_bridge/README.md:1-4) is not used"]},
        {"area": "NAV barometer (BMP280 replaces MS5611)", "owner": "firmware + sim round (control-relevant)",
         "items": ["new driver: BMP280 chip-id 0x58, trim registers, oversampling / filter / standby settings, 20-bit raw "
                   "values, integer compensation (Bosch datasheet, not on disk - A); replaces hal/ms5611.c and "
                   "core/ms5611_math.c; I2C address kept at 0x77 by SDO high (fc_config.json i2c_addr.ms5611 119)",
                   "NAV timing must be re-measured (bus script, background math) - simbench / co-sim",
                   "sim/sim6dof.py:881 baro_noise_m 0.25 m (MS5611) and the fan static-pressure offset model, the "
                   "estimator / AltitudeFilter tuning -> CONTROL-RELEVANT change: archive the before state and make "
                   "before / after comparison videos (user rule) before adopting"]},
        {"area": "GNS GPS (NEO-7M replaces NEO-6M)", "owner": "firmware check",
         "items": ["MON-VER hwVersion of u-blox 7 is '00070000' -> ubx_monver_module() returns UNKNOWN "
                   "(firmware/atmega128_fc/src/core/gps_ubx.c:276-298): only reported (gns_simbench), no functional "
                   "effect found - verify on the unit",
                   "UBX CFG-PRT / CFG-RATE / CFG-MSG / CFG-NAV5 and NAV-POSLLH / VELNED / SOL exist in the u-blox 7 protocol; "
                   "NEO-7M max navigation rate 10 Hz >= the 5 Hz the firmware sets (A: u-blox 7 data, not on disk); boot "
                   "probe at the factory 9600 then 38400 (gps_ubx.c:476) is compatible",
                   f"the 0.22 s latency ({P} flight_computer.nav.gps.latency_s_ASSUMED) is a u-blox 6 value - re-check",
                   "fit of the NEO-7M board + its antenna in the top-cap GPS spot (params board 36 x 25.8 x 4 mm + 25.5 mm "
                   "patch, configs.vtol.top_cap.gps) - measure, CAD not changed"]},
        {"area": "FCU", "owner": "firmware (optional)",
         "items": ["PF3 logic-battery divider (NEW, fitted by default) is not read by the firmware yet: add ADC3 to the "
                   "telemetry / a low-logic-battery alarm (user decision); until then measure the pack before each flight",
                   f"PC3 buf_isign is defined ({PG}:173-174) but no firmware code reads it today",
                   "fc_config.json imu_mounting (+x,+y,+z) must be set from the GY-521 orientation chosen in placement"]},
        {"area": "sim / power model", "owner": "sim round",
         "items": [f"sim/tether.py:171 bec_min_v (avionics brown out below 6 V on the bus) no longer applies: the logic "
                   "runs from its own battery; after a tether cut / sag the avionics keep running (servos lose the BEC) "
                   "-> reference / scenario change, not done here"]},
    ]


def user_decisions() -> list[str]:
    return [
        "logic battery: 2S 300 mAh (~20 g E) or 1S + boost (~10 g E); where it sits (misc box above the ESC suggested, A); "
        "the VTOL battery build is ~3.7 g over 1,650 g already -> ~24 g over (E)",
        "logic power switch: shunt / unplug only (default) or a second toggle in the top cap (new hole = CAD change)",
        "PLM100 firmware port (COM driver, link timing model, ground station, a second PLM100 on the ground) - separate round",
        "BMP280 driver + sim baro model - separate round, control-relevant (archived before / after videos)",
        "NEO-7M fit in the top cap and MON-VER behaviour - measure / confirm",
        "accept the new 1 kOhm series resistor on glink NAV TXD1 -> GNS RXD0 and the reset-hold jumpers J24 / J25",
        "logic power switch reachable from outside: J15 is inside the closed intake spigot (avionics windows start at z 343.1); "
        "a lead from J15 to a switch in a side window or the top cap needs a CAD change (review MF-5)",
        "OpenLog location: it cannot stand in a socket on the carrier (23+ mm > 13.4 mm) and does not fit flat (body overlaps "
        "the GY-521 body); J5 is now a lead pad row - put the OpenLog on a ~15-20 cm lead, e.g. in the misc box (CAD / mass "
        "placement decision, review MF-1)",
        "joint-screw length for the intake / avionics joint (M3 x 6 max suggested, review MF-6) and connector-plug envelopes "
        "in the CAD carrier placeholder (review MF-7) - next CAD round",
        "ADC pins PF1 / PF2 / PC3 kept as params (docs/v0.9_펌웨어_결과.md 9장 decision 3)",
        "antenna placement (u.FL pigtail) and GY-521 orientation (fc_config.json imu_mounting)",
        "modules socketed or soldered (GY-521 soldered flat recommended for vibration, A)",
    ]


def conflicts() -> list[str]:
    return [
        f"PLM100 package: task text says '24-pin castellated SMD'; {DS} p.8 fig 3 and p.12-13 fig 4 / 5 show two 1x12 "
        "1.27 mm headers (HEADER12 / socket12_1.27mm) 11.60 mm apart -> two library 1x12 1.27 mm sockets, no custom footprint",
        f"PLM100 max current: {DS} p.9 table 3 says 40 mA; {UG} p.6 says 100 mA (TX) -> budget uses 100 mA",
        f"PLM100 bandwidth numbering: {UG} p.12 BWTH 1 = 125 kHz; {CG} p.10 BWTH 0 = 125 kHz",
        f"PLM100 host port: firmware lora_uart expects an Ebyte transparent UART with M0 / M1 / AUX ({PG}:150-155, "
        f"lora_uart.c:65-70); PLM100 is an AT command modem on UART1 at 115200 ({UG} p.6-8) -> firmware follow-up",
        f"link.uart_baud 9600 ({P} line 2242) vs PLM100 console default 115200 -> AT+CBAR=9600 or firmware change",
        f"carrier model text ({P} line 1754) still lists GY-63 MS5611 and a generic LoRa module; mass_breakdown_g_ASSUMED "
        "'gy521_gy63_modules', 'lora_module_and_antenna' are for the old parts (user replaced them 2026-11-04)",
        f"power: {RM}:166, 212, 244 and {P} flight_computer.ilink.note put the logic on a 5 V UBEC from the 6S; the user "
        "decision 2026-11-04 adds a separate logic battery -> no 6S-to-logic converter on the carrier",
        "sim/tether.py:171 bec_min_v models an avionics brown-out with the bus; with the logic battery the avionics stay up",
        f"electronics.gps.interface ({P} line 1011) still names the NAV USART1 (v0.8); firmware board.h:95-102 and "
        "flight_computer.gns.pins (v0.8.1) put the GPS on the GNS -> firmware wins (GNS)",
        f"electronics.gps names NEO-6M (GY-NEO6MV2); the user's part is NEO-7M (MON-VER 00070000 -> UNKNOWN in gps_ubx.c:278-298)",
        f"glink is wired direct in {BH}:105; this spec adds a 1 kOhm series resistor (A, like the ilink) because one "
        "board has no glink connector to unplug for GNS programming",
        f"programming: {RM}:647-651 says 'unplug the nlink / ilink / glink connector'; one carrier has none -> reset-hold "
        "shunts J24 (NAV) / J25 (FCU)",
        f"release servo: configs.vtol.note removes it, but the firmware still drives PE5 (pwm.c:49, main.c:518) -> J13 "
        "footprint DNP",
        f"OpenLog shifter channel was undefined ({RM}:385-387) -> Q8 here",
        f"GPS supply: {P} electronics.gps.power '3.3 V rail' vs NEO-7M boards that often take 3.3-5 V via an on-board LDO "
        "(A) -> JP1 solder jumper, 3.3 V default",
        "SPEC.md:1640-1642 says GPS patch at 180 deg / gland at 0 deg; params configs.vtol.top_cap (fix round) has gland "
        "180 deg / patch 0 deg -> params wins (no carrier impact)",
        f"{BH}:1 header says 'three ATmega128A' while the same file documents four (NAV, FCU, COM, GNS) - comment only",
    ]


def assumptions() -> list[str]:
    return [
        "BMP280 module pin order VCC, GND, SCL, SDA, CSB, SDO (common GY-BMP280 order; not printed on the product page) - check the silkscreen",
        "BMP280 at 0x77: SDO tied to 3.3 V (same address as the firmware's MS5611), CSB tied to 3.3 V = I2C mode",
        "GY-521 pin order VCC, GND, SCL, SDA, XDA, XCL, AD0, INT; AD0 tied to GND = 0x68; module VCC 5 V (on-board LDO, README 2)",
        "NEO-7M board pins VCC, GND, TXD, RXD (+ PPS); supply 3.3 V via JP1 (5 V option); carrier JST-GH order 1 VCC, 2 GPS TXD, 3 GPS RXD, 4 GND",
        "OpenLog header order GRN, RXI, TXO, VCC, GND, BLK (SparkFun); VCC 5 V",
        "5 V buck module on a 1x4 2.54 mm header, order VIN, GND, VOUT, EN/NC, >= 1 A, Vin 6-8.4 V",
        "PLM100 has male 1.27 mm header pins (datasheet fig 4 side view 2.00 mm under a 1.0 mm PCB) -> two 1x12 1.27 mm sockets",
        "PLM100 pins 1 / 24 are 2.82 mm from the module's far edge, pins 12 / 13 5.22 mm from the antenna edge (fig 4; fig 5 labels are ambiguous) - 1:1 print check before ordering",
        "PLM100 UART1_TXD drives >= 3.0 V (V_IH of the ATmega128A at 5 V); not stated in the datasheet - measure",
        "PLM100 GPIO5 (USER_KEY) pulled up 47 kOhm so it is never asserted; GPIO3 / JTAG / UART0 / I2C / GPIO left open",
        "logic battery 2S LiPo 300 mAh, JST-PH, 3.0 V/cell cut-off; mass ~20 g (E, not in params)",
        "ATmega128A 25 mA each at 16 MHz / 5 V; OpenLog 20 mA avg / 80 mA peak; buck efficiency 85 %",
        "AVCC filter 10 uH + 100 nF; crystals 3225 SMD CL 16-18 pF with 22 pF caps; PEN tied to VCC",
        "BSS138 pull-ups 10k / 10k (I2C 4.7k on 5 V / 10k on 3.3 V, the modules add their own); servo / ESC series 220 Ohm "
        "+ 10k pull-downs; arm input 1k series + 10k pull-up + 100 nF",
        "logic ground tied to the propulsion ground only at NT1 (J12 pin 3)",
        "2 layers, 1 oz copper, near-solid bottom GND pour; corner radius 1 mm; tie band x -3..+3 mm free of parts",
        "servo / ESC / sense / arm / buzzer leads on right-angle headers; logic battery (JST-PH) and GPS (JST-GH) vertical, "
        "plugged from above (height limit 13.4 mm above the board); THT leads flush-cut <= 1.0 mm for the 3 mm foam",
        "OpenLog on a lead soldered into J5 (location off the carrier is a user decision); 57600 baud over ~0.2 m is fine (E)",
        "edge assignment from the CAD lead entries (servo hole 180 deg, GPS lead hole 230 deg, arm switch 90 deg, ESC / misc +x)",
    ]


def main() -> int:
    build()
    rep = check()
    if ERRORS:
        for e in ERRORS:
            print("ERROR", e)
        print(f"{len(ERRORS)} error(s) - spec NOT written")
        return 1
    parts = sorted(PARTS.values(), key=lambda p: natural(p["refdes"]))
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
        "schema": "vtol_carrier_spec/1",
        "title": "CanSat EDF VTOL avionics carrier PCB - electrical specification (not drawn yet)",
        "status": "electrical spec; the KiCad schematic and board are generated from it (scripts/). NOT_VERIFIED: nothing "
                  "built or measured",
        "labels": {"F": "from a file (params / firmware / docs / datasheet / KiCad library)", "M": "measured by a tool",
                   "E": "estimated", "A": "assumed (reason given)"},
        "sources": SOURCES,
        "user_parts": {
            "barometer": "BMP280 module [SMG] TYE-BMP29 (devicemart 15442588) - replaces GY-63 MS5611",
            "radio": "PLNetworks PLM100 LoRa module (devicemart 1289751) - replaces Ebyte E22 / SX1276 options",
            "gps": "NEO-7M module SZH-NT07 (devicemart 10895400) - in the top cap, carrier has the lead connector only",
            "imu": "GY-521 MPU-6050 SZH-EK007 (devicemart 1247052) - as before",
        },
        "summary": summary,
        **narrative(),
        "parts": parts,
        "nets": nets,
        "no_connect": ncl,
        "checks": {"tool": "scripts/build_spec.py (scripted netlist checks; not ERC / DRC)", "result": "PASS", **rep},
        "firmware_followup": firmware_followup(),
        "user_decisions": user_decisions(),
        "conflicts": conflicts(),
        "assumptions": assumptions(),
    }
    OUT.write_text(json.dumps(spec, indent=1, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {OUT}: {summary}")
    print(f"checks: pins {rep['pins_total']} = connected {rep['pins_connected']} + nc {rep['pins_no_connect']}; "
          f"paths {sum(1 for p in rep['signal_paths'] if p['ok'])}/{len(rep['signal_paths'])}; "
          f"bss138 {len(rep['bss138_channels'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
