"""Tests for ``tools/gen_firmware_params.py`` (the ``params_gen.h`` generator), v0.6.3 three-chip flight computer.

Covers: each chip's pin table checked on its own (FCU ``flight_computer.pins``, NAV ``flight_computer.nav.pins``,
COM ``flight_computer.com.pins``) - a pin used twice on one chip is refused, the same pin name on another chip
is not; the nlink on the SPI pins of both ends (PB0 = SS on the NAV and the FCU), no I2C / sensor pin left on the
FCU, the nlink SPI clock (an exact AVR divider a slave can follow), mode, byte gap (longer than the FCU slave
interrupt needs), exchange time and frame timeout, the NAV tick / clock / MCU, the nlink macros and the NAV LED;
the platformio.ini NAV / FCU source filters and every env name; the ilink only on that chip's USART (FCU USART1
PD2/PD3, COM USART0 PE0/PE1) and the platformio.ini
``-D ILINK_USART`` of each role agreeing with it; peripheral signals on their fixed pins (SX127x NSS = SS);
pin labels that are not an alternate function of the pin, pins the ATmega128 does not have, radio pins left
in the FCU table, missing signals; the ilink baud (U2X = 0, refused beyond 1 % error), the STAT divider
(com.tick_hz / stat_hz must be an integer), com.tick_hz = control.loop_hz, com.f_cpu_hz / com.mcu;
FC_PARAMS_TAG = low byte of FC_PARAMS_SHA256_PREFIX; the radio macros (v0.5 names) coming from com.pins; the
v0.5 refusals (radio settings, link UART baud, link budget); the header on disk being current; v0.7: MISSION_MODE, the
VTOL supervisor / vertical-controller macros equal to the reference's parameter objects, the ballistic macros unchanged
in both modes and without mission.vtol (a v0.6.3 params.json), the VTOL refusals, the vertical gains' source and stale
flag, the TELEM2-sized SX127x timeout and link budget of the VTOL build; v0.7.1: one block per build profile (field /
demo) behind the MISSION_PROFILE switch, each equal to the reference's objects of that profile (supervisor V8 / V9 / V10
integers, the vertical controller v2's integer steps and derived constants, the NAV estimator and altitude filter), the
estimator gains from sim/out/vtol_gains.json, the link budget from sim/link_model.py budget(mode=...), NLINK_FCU_THR_SCALE;
tools/gen_gains_header.py's attitude-tuning stale rule (the digest of the attitude-tuning subtrees).
fw09 (v0.9 firmware round): G1 - per VTOL configuration (battery / tether / hybrid) the vehicle constants equal
sim.vehicle.load_vehicle(config='vtol', power=...) and the parameter objects sim/sim6dof.py builds from that vehicle
(VerticalParams, PositionParams, VtolAltitudeFilter) to 1e-12, the header's #if FC_VEHICLE blocks and the C preprocessor
give each build exactly its configuration's numbers, only VCFG_MACROS differ, the rocket set (every ballistic build) is
the rocket vehicle and unchanged by the merge, the build refusals (no FC_VCFG on an AVR VTOL image, FC_VCFG 3, PWR_SRC
!= FC_VCFG, a configuration without its CAD report), FC_PARAMS_TAG per configuration, platformio.ini's FC_VCFG in every
VTOL env; W1 - NLINK_WD_* and its checks; SPEC 5 - the power pins from flight_computer.pins; the three generated headers
current and the bridge header following sim/link_proto.py (RP1).

Run: ``.venv/Scripts/python.exe -m pytest tools/test_gen_firmware_params.py -q -p no:cacheprovider``
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
for _p in (HERE, ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import gen_firmware_params as gfp  # noqa: E402
from sim import link_proto as lp  # noqa: E402  (v0.9 tests)

with open(gfp.PARAMS, "r", encoding="utf-8") as _f:
    BASE = json.load(_f)
F_CPU = int(BASE["flight_computer"]["f_cpu_hz"])


@pytest.fixture(scope="module")
def base() -> dict:
    return gfp.derive()


@pytest.fixture(scope="module")
def header(base) -> str:
    return gfp.render(base)


def macros(text: str) -> dict[str, str]:
    """#define NAME VALUE (comment stripped) of a rendered header (a name defined in several profile blocks: the last; in
    the fw09 G1 vehicle blocks the rocket set comes last, so this is the rocket set)."""
    out = {}
    for m in re.finditer(r"^#define (\w+)[ \t]+([^\n]*?)[ \t]*(?:/\*[^\n]*)?$", text, re.M):
        out[m.group(1)] = m.group(2)
    return out


def vehicle_view(text: str, vcfg: str = "rocket") -> str:
    """fw09 G1: the header text as a build of configuration ``vcfg`` sees it - every generated
    '#if FC_VEHICLE == FC_VCFG_<X>' / '#elif ...' / '#else  /* FC_VCFG_ROCKET' / '#endif  /* FC_VEHICLE */' block replaced
    by the lines of that configuration's branch (the rocket: the #else branch)."""
    out, keep, inside = [], True, False
    for ln in text.split("\n"):
        m = re.match(r"^#(if|elif) FC_VEHICLE == FC_VCFG_(\w+)$", ln)
        if m:
            inside, keep = True, m.group(2).lower() == vcfg
            continue
        if inside and ln.startswith("#else  /* FC_VCFG_ROCKET"):
            keep = vcfg == "rocket"
            continue
        if inside and ln.startswith("#endif  /* FC_VEHICLE */"):
            inside, keep = False, True
            continue
        if keep:
            out.append(ln)
    return "\n".join(out)


def profile_macros(text: str, name: str, vcfg: str = "rocket") -> dict[str, str]:
    """The macros of one build profile's block (#if / #elif MISSION_PROFILE == MISSION_PROFILE_ID_<NAME>) as a build of
    the vehicle configuration ``vcfg`` sees them (fw09 G1: the vehicle blocks nested in the profile block resolved)."""
    m = re.search(rf"^#(?:if|elif) MISSION_PROFILE == MISSION_PROFILE_ID_{name.upper()}\b[^\n]*\n(.*?)^#(?:elif|else)\b",
                  vehicle_view(text, vcfg), re.M | re.S)
    assert m, name
    return macros(m.group(1))


def derive_with(tmp_path, *mutations) -> dict:
    p = copy.deepcopy(BASE)
    for mut in mutations:
        mut(p)
    path = tmp_path / "params.json"
    path.write_text(json.dumps(p, indent=2), encoding="utf-8")
    return gfp.derive(params_path=str(path))


def refused(tmp_path, *mutations, needles: tuple[str, ...] = ()) -> str:
    with pytest.raises(gfp.GenError) as ei:
        derive_with(tmp_path, *mutations)
    msg = str(ei.value)
    for n in needles:
        assert n in msg, msg
    return msg


def fcu(key, val):
    return lambda p: p["flight_computer"]["pins"].__setitem__(key, val)


def com(key, val):
    return lambda p: p["flight_computer"]["com"]["pins"].__setitem__(key, val)


def com_del(key):
    return lambda p: p["flight_computer"]["com"]["pins"].pop(key)


def nav(key, val):
    return lambda p: p["flight_computer"]["nav"]["pins"].__setitem__(key, val)


def fc_set(path: str, val):
    """'com.tick_hz' / 'ilink.baud' / 'link.sx127x.freq_hz' under flight_computer."""
    def mut(p):
        node = p["flight_computer"]
        keys = path.split(".")
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = val
    return mut


# --------------------------------------------------------------------------- #
# the current params.json
# --------------------------------------------------------------------------- #
def test_current_params_pin_maps(base):
    f, c, n = base["fcu_pins"], base["com_pins"], base["nav_pins"]
    assert f["ilink_rx"] == ("D", 2) and f["ilink_tx"] == ("D", 3)          # FCU USART1
    assert f["log_tx"] == ("E", 1) and not set(f) & set(gfp.SENSOR_SIGNALS)  # v0.6.3: no I2C / sensor on the FCU
    nl = ("ss", "sck", "mosi", "miso")
    assert [f[f"nlink_{k}"] for k in nl] == [n[f"spi_{k}"] for k in nl] == [("B", 0), ("B", 1), ("B", 2), ("B", 3)]
    assert n["i2c_scl"] == ("D", 0) and n["i2c_sda"] == ("D", 1) and n["imu_int"] == ("E", 6) and n["led"] == ("C", 1)
    assert not {"ilink_rx", "ilink_tx"} & set(n)
    assert c["ilink_rx"] == ("E", 0) and c["ilink_tx"] == ("E", 1)          # COM USART0
    assert c["link_uart_rx"] == ("D", 2) and c["link_uart_tx"] == ("D", 3)  # Ebyte on COM USART1
    assert [c[k] for k in ("lora_spi_nss", "lora_spi_sck", "lora_spi_mosi", "lora_spi_miso")] == \
        [("B", 0), ("B", 1), ("B", 2), ("B", 3)]
    assert c["lora_dio0"] == ("E", 7) and c["lora_reset"] == ("C", 6) and c["led"] == ("C", 0)
    assert f["led"] == ("C", 0)                       # the same pin name on the other chip is another pin
    assert not set(f) & set(gfp.RADIO_SIGNALS)        # no radio pin on the FCU any more
    for chip, pins in (("fcu", f), ("com", c), ("nav", n)):   # no pin twice on one chip
        assert len(set(pins.values())) == len(pins), chip


def test_header_macros_ilink_com_and_v05_names(base, header):
    m = macros(header)
    assert m["ILINK_BAUD"] == "250000UL"
    assert m["ILINK_UBRR"] == "3" and m["ILINK_U2X"] == "0"
    assert m["ILINK_STAT_DIV"] == "250U"
    assert m["COM_TICK_HZ"] == "250" and m["FC_LOOP_HZ"] == "250"
    assert m["PIN_COM_LED_PORT"] == "C" and m["PIN_COM_LED_BIT"] == "0"
    assert m["ILINK_USART_FCU"] == "1" and m["ILINK_USART_COM"] == "0"
    assert "error +0.00 % (limit 1 %)" in header
    # every v0.5 pin / link macro name is still generated
    for name in ("PIN_LED_PORT", "PIN_LED_BIT", "PIN_BUZZER_PORT", "PIN_ARM_SW_PORT", "PIN_LINK_M0_PORT", "PIN_LINK_M0_BIT",
                 "PIN_LINK_M1_PORT", "PIN_LINK_AUX_PORT", "PIN_LORA_NSS_PORT", "PIN_LORA_DIO0_PORT", "PIN_LORA_RESET_PORT",
                 "PIN_LORA_RESET_BIT", "LINK_UART_BAUD", "LINK_UART_UBRR", "LINK_UART_U2X", "LINK_AUX_WAIT_TICKS",
                 "SX127X_TX_TIMEOUT_TICKS", "FC_LOG_UBRR", "PWM_OUT_VANE1", "VBAT_ADC_CHANNEL", "FC_PARAMS_SHA256_PREFIX"):
        assert name in m, name
    assert (m["PIN_LINK_M0_PORT"], m["PIN_LINK_M0_BIT"]) == ("C", "3")
    assert (m["PIN_LORA_DIO0_PORT"], m["PIN_LORA_DIO0_BIT"]) == ("E", "7")


def test_radio_macros_come_from_com_pins(tmp_path):
    d = derive_with(tmp_path, com("link_m0", "PC7"), com("lora_reset", "PA0"))
    m = macros(gfp.render(d))
    assert (m["PIN_LINK_M0_PORT"], m["PIN_LINK_M0_BIT"]) == ("C", "7")
    assert (m["PIN_LORA_RESET_PORT"], m["PIN_LORA_RESET_BIT"]) == ("A", "0")
    assert (m["PIN_LED_PORT"], m["PIN_LED_BIT"]) == ("C", "0")              # FCU LED untouched


def test_fc_params_tag_is_low_byte_of_the_sha_prefix(base, header, tmp_path):
    with open(gfp.PARAMS, "rb") as f:
        h = hashlib.sha256(f.read()).hexdigest()
    m = macros(header)
    assert m["FC_PARAMS_SHA256_PREFIX"] == f'"{h[:16]}"'
    assert base["params_tag"] == int(h[:16], 16) & 0xFF == int(h[14:16], 16)
    assert m["FC_PARAMS_TAG"] == f"0x{base['params_tag']:02X}u"
    # another params.json -> the tag follows that file's hash
    d = derive_with(tmp_path, lambda p: p["meta"].__setitem__("date", "2099-01-01"))
    h2 = hashlib.sha256((tmp_path / "params.json").read_bytes()).hexdigest()
    assert h2 != h and d["params_tag"] == int(h2[14:16], 16)
    assert macros(gfp.render(d))["FC_PARAMS_TAG"] == f"0x{int(h2[14:16], 16):02X}u"


def test_platformio_ilink_usart_matches_the_pin_check():
    with open(os.path.join(gfp.FW_DIR, "platformio.ini"), "r", encoding="utf-8") as f:
        ini = f.read()
    for chip, section in (("fcu", "fcu_common"), ("com", "com_common")):
        body = re.search(rf"^\[{section}\]\n(.*?)(?=^\[)", ini, re.M | re.S)
        assert body, section
        flags = re.findall(r"-D\s*ILINK_USART=(\d)", body.group(1))
        assert flags == [str(gfp.ILINK_USART[chip])], (section, flags)


def test_header_has_the_ilink_usart_compile_guard(header):
    assert "#if defined(ILINK_USART) && defined(FC_ROLE_FCU) && ILINK_USART != ILINK_USART_FCU" in header
    assert "#if defined(ILINK_USART) && defined(FC_ROLE_COM) && ILINK_USART != ILINK_USART_COM" in header


def test_header_on_disk_is_current(capsys):
    assert gfp.main(["--check"]) == 0, capsys.readouterr().out


# --------------------------------------------------------------------------- #
# the CAD report's hash ignores the build timings (reproducible --check)
# --------------------------------------------------------------------------- #
def _cad_copy(tmp_path, name: str, mutate=None, indent=None) -> str:
    with open(gfp.CAD_REPORT, "r", encoding="utf-8") as f:
        rep = json.load(f)
    if mutate:
        mutate(rep)
    path = tmp_path / name
    path.write_text(json.dumps(rep, indent=indent), encoding="utf-8")
    return str(path)


def test_cad_report_hash_ignores_build_timings_and_formatting(tmp_path):
    h0 = gfp.cad_report_sha256(gfp.CAD_REPORT)
    with open(gfp.CAD_REPORT, "r", encoding="utf-8") as f:
        extra = json.load(f)["extra"]
    assert all(k in extra for _, k in gfp.CAD_REPORT_VOLATILE)                         # the fields really exist

    def retime(rep):
        rep["extra"]["build_time_s"] = 123.456
        rep["extra"]["total_time_s"] = 789.0

    def drop_timings(rep):
        del rep["extra"]["build_time_s"], rep["extra"]["total_time_s"]

    def reorder(rep):
        rep["parts"] = dict(reversed(list(rep["parts"].items())))

    for name, mut, indent in (("retimed.json", retime, 1), ("compact.json", None, None), ("untimed.json", drop_timings, 4),
                              ("reordered.json", reorder, 2)):
        p = _cad_copy(tmp_path, name, mut, indent)
        assert gfp.cad_report_sha256(p) == h0, name
    # the file bytes differ (the old sha256 of the raw file would have gone STALE) ...
    assert gfp.sha256(_cad_copy(tmp_path, "retimed2.json", retime, 1)) != gfp.sha256(gfp.CAD_REPORT)
    # ... while anything about the vehicle changes the hash: a part mass, its CG, the params it was built from, a warning
    for name, mut in (("mass.json", lambda r: r["parts"]["vane_1"].__setitem__("mass_g", r["parts"]["vane_1"]["mass_g"] + 1e-9)),
                      ("com.json", lambda r: r["parts"]["battery"]["com_mm"].__setitem__(2, 0.0)),
                      ("params.json", lambda r: r.__setitem__("params_sha256", "0" * 64)),
                      ("warn.json", lambda r: r["warnings"].append("x")),
                      ("extra.json", lambda r: r["extra"].__setitem__("spigot_shoulder_t_mm", 1.3))):
        assert gfp.cad_report_sha256(_cad_copy(tmp_path, name, mut)) != h0, name


def test_header_records_the_canonical_cad_hash(tmp_path, base, header):
    """params_gen.h names the CAD report by its canonical hash and says which fields it leaves out; a report that
    differs only in its build timings derives the same header text (apart from its own path)."""
    line = next(ln for ln in header.splitlines() if "cad/exports/cadquery_report.json (canonical)" in ln)
    assert line.split()[-1] == gfp.cad_report_sha256(gfp.CAD_REPORT)
    assert "extra.build_time_s / extra.total_time_s" in header
    p = _cad_copy(tmp_path, "cadquery_report.json",
                  lambda r: r["extra"].update(build_time_s=0.001, total_time_s=99.0), indent=3)
    d = gfp.derive(cad_path=p)
    assert dict(d["sources"])[gfp.rel(p) + " (canonical)"] == dict(base["sources"])[
        "cad/exports/cadquery_report.json (canonical)"]
    assert gfp.render(d).replace(gfp.rel(p), "cad/exports/cadquery_report.json") == header


# --------------------------------------------------------------------------- #
# v0.6.3: NAV chip, nlink SPI
# --------------------------------------------------------------------------- #
def test_header_nlink_and_nav_macros(base, header):
    m = macros(header)
    assert m["NAV_TICK_HZ"] == "250" == m["FC_LOOP_HZ"]
    assert m["NLINK_SPI_HZ"] == "2000000UL" and m["NLINK_SPI_DIV"] == "8"
    assert (m["NLINK_SPI_SPR"], m["NLINK_SPI_2X"], m["NLINK_SPI_MODE"]) == ("1", "1", "0")    # F_CPU / 8: SPR 01, SPI2X
    assert m["NLINK_BYTE_GAP_US"] == "4.0f" and m["NLINK_BYTE_GAP_CYCLES"] == "64UL"
    # params flight_computer.nlink.frame_timeout_ms 4.6 (v0.6.3: 4.5 -> 5.0 -> 4.6; > 4.6 starved the fallback ticks'
    # background, see the params note) x 2 Timer1 counts / us
    assert BASE["flight_computer"]["nlink"]["frame_timeout_ms"] == 4.6
    assert m["NLINK_FRAME_TIMEOUT_US"] == "4600UL" and m["NLINK_FRAME_TIMEOUT_T1_COUNTS"] == "9200U"
    # SS low .. SS high: 2 x (64 + 8) + 72 x 64 + 71 x (64 + 11) = 10077 cycles (the simavr-timed exchange) -> 630 us
    assert m["NLINK_XFER_US"] == "630U" and abs(base["nl_xfer_s"] - 10077 / 16e6) < 1e-12
    assert (m["PIN_NAV_LED_PORT"], m["PIN_NAV_LED_BIT"]) == ("C", "1")
    assert (m["PIN_LED_PORT"], m["PIN_LED_BIT"]) == ("C", "0")                # FCU LED untouched
    assert "NAV PB0 SS -> FCU PB0 SS" in header and "FCU PB3 MISO -> NAV PB3" in header


@pytest.mark.parametrize("hz, spr, x2", [(1000000, 1, 0), (4000000, 0, 0), (500000, 2, 1)])
def test_nlink_clock_other_exact_dividers_accepted(tmp_path, hz, spr, x2):
    """fw09 W1: a slower clock lengthens the exchange, and the exchange watchdog (params 1000 us) must stay above 1.5 x
    it - where 1000 us is too short the clock alone is refused by that check, and accepted with the smallest whole
    watchdog above 1.5 x the exchange."""
    xfer = derive_with(tmp_path, fc_set("nlink.spi_clock_hz", hz), fc_set("nlink.exchange_watchdog_us", 3000))["nl_wd"]
    wd = max(1000, math.floor(1.5 * xfer["xfer_us"]) + 1)
    if wd != 1000:
        refused(tmp_path, fc_set("nlink.spi_clock_hz", hz), needles=("must exceed 1.5 x the exchange",))
    d = derive_with(tmp_path, fc_set("nlink.spi_clock_hz", hz), fc_set("nlink.exchange_watchdog_us", wd))
    m = macros(gfp.render(d))
    assert (m["NLINK_SPI_SPR"], m["NLINK_SPI_2X"]) == (str(spr), str(x2)) and m["NLINK_SPI_HZ"] == f"{hz}UL"
    assert m["NLINK_WD_US"] == f"{wd}UL" and m["NLINK_WD_COUNTS"] == f"{wd * 16}u"


@pytest.mark.parametrize("hz, needles", [
    (3000000, ("spi_clock_hz 3e+06", "F_CPU /")),                # no exact divider
    (2100000, ("only F_CPU /",)),
    (8000000, ("> F_CPU / 4", "slave")),                         # F_CPU / 2: a slave cannot follow
    (125000, ("exchange takes", "NAV tick")),                    # F_CPU / 128: 72 bytes > half a tick
    (250000, ("exchange takes", "2646 us")),                     # F_CPU / 64
])
def test_nlink_bad_clock_refused(tmp_path, hz, needles):
    refused(tmp_path, fc_set("nlink.spi_clock_hz", hz), needles=needles)


@pytest.mark.parametrize("mut, needles", [
    (fc_set("nlink.spi_mode", 1), ("spi_mode 1", "mode 0")),
    (fc_set("nlink.spi_mode", 3), ("spi_mode 3",)),
    (fc_set("nlink.byte_gap_us", 1.0), ("byte_gap_us 1", "16 cycles <", "slave interrupt")),
    (fc_set("nlink.byte_gap_us", 0), ("must be > 0",)),
    (fc_set("nlink.frame_timeout_ms", 4.0), ("frame_timeout_ms 4.0", "one and two")),
    (fc_set("nlink.frame_timeout_ms", 8.0), ("frame_timeout_ms 8.0",)),
    (fc_set("nlink.frame_timeout_ms", 4.0005), ("whole number of us",)),
    (lambda p: p["flight_computer"].pop("nlink"), ("flight_computer.nlink missing",)),
    (lambda p: p["flight_computer"].pop("nav"), ("flight_computer.nav missing",)),
    (fc_set("nav.tick_hz", 500), ("flight_computer.nav.tick_hz 500", "control.loop_hz 250")),
    (fc_set("nav.f_cpu_hz", 8000000), ("flight_computer.nav.f_cpu_hz 8000000",)),
    (fc_set("nav.mcu", "ATmega2560"), ("flight_computer.nav.mcu",)),
])
def test_nlink_and_nav_refusals(tmp_path, mut, needles):
    refused(tmp_path, mut, needles=needles)


def test_nlink_short_gap_at_the_limit_accepted(tmp_path):
    d = derive_with(tmp_path, fc_set("nlink.byte_gap_us", 2.0))            # 32 cycles >= 27
    assert d["nl_gap_cycles"] == 32 and macros(gfp.render(d))["NLINK_BYTE_GAP_CYCLES"] == "32UL"
    refused(tmp_path, fc_set("nlink.byte_gap_us", 1.6), needles=("26 cycles <",))   # 25.6 -> 26 cycles < 27


@pytest.mark.parametrize("mut, needles", [
    (fcu("nlink_ss", "PB4"), ("flight_computer.pins.nlink_ss", "SS is fixed to PB0")),
    (fcu("nlink_miso", "PB2/MOSI"), ("flight_computer.pins.nlink_miso", "MISO is fixed to PB3")),
    (nav("spi_ss", "PB4 (output)"), ("flight_computer.nav.pins.spi_ss", "SS is fixed to PB0", "stays master")),
    (nav("spi_sck", "PB7/SCK"), ("SCK is not an alternate function of PB7",)),
    (nav("i2c_scl", "PD2"), ("flight_computer.nav.pins.i2c_scl", "SCL is fixed to PD0")),
    (nav("led", "PB0"), ("flight_computer.nav.pins.", "PB0 on the NAV")),
    (fcu("led", "PB3"), ("flight_computer.pins.", "PB3 on the FCU")),
    (fcu("i2c_scl", "PD0/SCL"), ("i2c_scl is not a signal of the FCU", "flight_computer.nav.pins")),
    (fcu("imu_int", "PE6/INT6"), ("imu_int is not a signal of the FCU", "NAV")),
    (nav("ilink_rx", "PD2/RXD1"), ("ilink_rx is not a signal of the NAV", "nlink SPI")),
    (lambda p: p["flight_computer"]["nav"]["pins"].pop("spi_miso"), ("flight_computer.nav.pins", "spi_miso missing")),
    (lambda p: p["flight_computer"]["pins"].pop("nlink_ss"), ("flight_computer.pins", "nlink_ss missing")),
])
def test_nlink_and_nav_pin_refusals(tmp_path, mut, needles):
    refused(tmp_path, mut, needles=needles)


def test_nav_pins_independent_of_the_other_chips(tmp_path):
    # NAV LED on PC0 (= FCU LED, COM LED) and on PE1 (= FCU log TX, COM ilink TX): other chips, other pins
    d = derive_with(tmp_path, nav("led", "PC0"))
    assert d["nav_pins"]["led"] == d["fcu_pins"]["led"] == d["com_pins"]["led"] == ("C", 0)
    d = derive_with(tmp_path, nav("led", "PE1"))
    m = macros(gfp.render(d))
    assert (m["PIN_NAV_LED_PORT"], m["PIN_NAV_LED_BIT"]) == ("E", "1")


def _ini() -> str:
    with open(os.path.join(gfp.FW_DIR, "platformio.ini"), "r", encoding="utf-8") as f:
        return f.read()


def _section(ini: str, name: str) -> str:
    m = re.search(rf"^\[{re.escape(name)}\]\n(.*?)(?=^\[|\Z)", ini, re.M | re.S)
    assert m, name
    return m.group(1)


def test_platformio_nav_and_fcu_filters_and_env_names():
    ini = _ini()
    envs = re.findall(r"^\[env:(\w+)\]", ini, re.M)
    v062 = ["fcu", "fcu_bench_tether", "fcu_bench_hover", "fcu_bench_vanes", "fcu_simbench", "fcu_simbench_micro", "fcu_cosim",
            "com", "com_lora_uart", "com_sx127x", "com_simbench", "com_simbench_sx127x", "com_cosim", "com_cosim_sx127x"]
    assert all(e in envs for e in v062), envs                                 # every existing env name kept
    assert {"nav", "nav_simbench", "nav_cosim"} <= set(envs)
    nav_c = _section(ini, "nav_common")
    assert re.findall(r"-D\s*(FC_ROLE_\w+)", nav_c) == ["FC_ROLE_NAV"] and "ILINK_USART" not in nav_c
    for f in ("-<*>", "+<nav/>", "+<core/>", "+<hal/twi.c>", "+<hal/mpu6050.c>", "+<hal/bmp280.c>", "+<hal/ms5611.c>",
              "+<hal/nlink_spi.c>",
              "+<hal/watchdog.c>", "+<hal/stack.c>", "+<hal/timer.c>"):
        assert f in nav_c, f
    for f in ("pwm.c", "ilink_uart.c", "lora_uart.c", "sx127x.c", "adc.c", "+<*>"):
        assert f not in nav_c, f
    fcu_c = _section(ini, "fcu_common")
    for f in ("-<nav/>", "-<com/>", "-<hal/twi.c>", "-<hal/mpu6050.c>", "-<hal/bmp280.c>", "-<hal/ms5611.c>",
              "-<hal/lora_uart.c>",
              "-<hal/sx127x.c>"):
        assert f in fcu_c, f
    assert "-D FC_ROLE_FCU" in fcu_c
    # fw09 G1: every VTOL env names its vehicle configuration (the battery vehicle's: FC_VCFG=0)
    for env, flags in (("nav", ("FC_VCFG",)), ("nav_simbench", ("FC_SIMBENCH", "FC_VCFG")),
                       ("nav_cosim", ("FC_SIMBENCH", "FC_COSIM", "FC_VCFG"))):
        body = _section(ini, f"env:{env}")
        assert "${nav_common.build_src_filter}" in body and "${nav_common.build_flags}" in body, env
        assert sorted(re.findall(r"-D\s*(FC_\w+)", body)) == sorted(flags), env
        assert re.findall(r"-D\s*FC_VCFG=(\d)", body) == ["0"], env
    default = re.search(r"^default_envs\s*=\s*(.*)$", ini, re.M).group(1)
    assert "nav" in [e.strip() for e in default.split(",")]


# --------------------------------------------------------------------------- #
# per-chip pin conflicts
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("mut, needles", [
    (fcu("buzzer", "PC0 (NPN driver)"), ("flight_computer.pins.buzzer", ".led", "PC0 on the FCU")),
    (fcu("arm_switch", "PD2"), ("flight_computer.pins.arm_switch", ".ilink_rx", "PD2 on the FCU")),
    (fcu("led", "PE1"), ("flight_computer.pins.led", ".log_tx", "PE1 on the FCU")),
    (com("link_m0", "PC0"), ("flight_computer.com.pins.", "PC0 on the COM")),
    (com("lora_reset", "PE7"), ("flight_computer.com.pins.lora_reset", ".lora_dio0", "PE7 on the COM")),
    (com("led", "PE1"), ("flight_computer.com.pins.led", ".ilink_tx", "PE1 on the COM")),
    (com("link_aux", "PB2"), ("flight_computer.com.pins.link_aux", ".lora_spi_mosi", "PB2 on the COM")),
])
def test_pin_used_twice_on_one_chip_is_refused(tmp_path, mut, needles):
    refused(tmp_path, mut, needles=needles)


def test_same_pin_on_the_other_chip_is_accepted(tmp_path):
    # FCU buzzer on PC4 (= COM link_m1), COM LED on PC2 (= FCU arm switch), FCU LED on PC6 (= COM radio RESET)
    # (fw09 SPEC 5: PC3 = COM link_m0 is the FCU's buf_isign now, so the v0.6.3 example PC3 is taken on the FCU itself)
    d = derive_with(tmp_path, fcu("buzzer", "PC4 (NPN driver)"), com("led", "PC2"), fcu("led", "PC6"))
    assert d["fcu_pins"]["buzzer"] == d["com_pins"]["link_m1"] == ("C", 4)
    assert d["fcu_pins"]["led"] == d["com_pins"]["lora_reset"] == ("C", 6)
    assert d["com_pins"]["led"] == d["fcu_pins"]["arm_switch"] == ("C", 2)
    assert d["fcu_pins"]["buf_isign"] == d["com_pins"]["link_m0"] == ("C", 3)
    refused(tmp_path, fcu("buzzer", "PC3 (NPN driver)"), needles=("buf_isign and .buzzer both use PC3 on the FCU",))


# --------------------------------------------------------------------------- #
# ilink on the chip's own USART, fixed peripheral pins, labels
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("muts, needles", [
    ((fcu("ilink_rx", "PE0/RXD0"), fcu("ilink_tx", "PE1/TXD0")), ("flight_computer.pins.ilink_rx", "USART1", "ILINK_USART=1")),
    ((fcu("ilink_tx", "PE1/TXD0"),), ("flight_computer.pins.ilink_tx", "USART1")),
    ((fcu("ilink_rx", "PD3/TXD1"), fcu("ilink_tx", "PD2/RXD1")), ("flight_computer.pins.ilink_rx", "RXD1 = PD2")),
    ((com("ilink_rx", "PD2/RXD1"), com("ilink_tx", "PD3/TXD1")), ("flight_computer.com.pins.ilink_rx", "USART0", "ILINK_USART=0")),
    ((com("ilink_tx", "PD3/TXD1"),), ("flight_computer.com.pins.ilink_tx", "USART0")),
    ((com("ilink_rx", "PE2"),), ("flight_computer.com.pins.ilink_rx", "RXD0 = PE0")),
])
def test_ilink_not_on_the_chips_usart_is_refused(tmp_path, muts, needles):
    refused(tmp_path, *muts, needles=needles)


@pytest.mark.parametrize("mut, needles", [
    (fcu("ilink_rx", "PD2/RXD0"), ("RXD0 is not an alternate function of PD2",)),         # label typo
    (com("lora_dio0", "PE7/INT6"), ("INT6 is not an alternate function of PE7",)),
    (com("lora_reset", "PG5"), ("no PG5",)),                                              # port G has PG0..PG4
    (com("link_uart_tx", "PD4"), ("link_uart_tx", "TXD1 is fixed to PD3")),
    (com("lora_spi_nss", "PB4"), ("lora_spi_nss", "SS is fixed to PB0")),
    (com("lora_spi_sck", "PB7/SCK"), ("SCK is not an alternate function of PB7",)),
    (fcu("log_tx", "PD3/TXD1"), ("log_tx", "TXD0 is fixed to PE1")),
    (fcu("esc", "PE4/OC1B"), ("flight_computer.pins.esc", "OCnx")),
    (com_del("lora_reset"), ("flight_computer.com.pins", "lora_reset missing")),
    (fcu("link_m0", "PC3"), ("flight_computer.pins", "link_m0 is not a signal of the FCU", "flight_computer.com.pins")),
    (com("buzzer", "PC1"), ("flight_computer.com.pins", "buzzer is not a signal of the COM")),
    (lambda p: p["flight_computer"].pop("com"), ("flight_computer.com missing",)),
    (lambda p: p["flight_computer"].pop("ilink"), ("flight_computer.ilink missing",)),
])
def test_bad_pin_tables_are_refused(tmp_path, mut, needles):
    refused(tmp_path, mut, needles=needles)


# --------------------------------------------------------------------------- #
# ilink baud, STAT divider, COM tick / clock / MCU
# --------------------------------------------------------------------------- #
def test_ilink_uart_setting_u2x0_and_one_percent():
    assert gfp.ilink_uart_setting(F_CPU, 250000) == (3, 0.0)
    assert gfp.ilink_uart_setting(F_CPU, 500000) == (1, 0.0)
    assert gfp.ilink_uart_setting(F_CPU, 1000000) == (0, 0.0)
    ubrr, err = gfp.ilink_uart_setting(F_CPU, 76800)
    assert ubrr == 12 and abs(err - (F_CPU / (16 * 13) / 76800 - 1)) < 1e-15 and abs(err) < 0.002
    # 24691 baud: -1.22 % with U2X = 0 (UBRR 40) - inside the 2 % of the other UARTs, outside the ilink's 1 %
    x = F_CPU / (16 * 41) / 24691 - 1
    assert 0.01 < abs(x) < 0.02
    with pytest.raises(gfp.GenError, match=r"-1\.22 %"):
        gfp.ilink_uart_setting(F_CPU, 24691)
    # the general UART helper would take U2X = 1 there; the ilink never does
    assert gfp.uart_setting(F_CPU, 24691)[1] == 1
    for bad in (115200, 230400, 0):
        with pytest.raises(gfp.GenError):
            gfp.ilink_uart_setting(F_CPU, bad)


@pytest.mark.parametrize("baud, needle", [(115200, "-3.55 %"), (230400, "+8.51 %"), (24691, "-1.22 %"),
                                          (250000.5, "not an integer")])
def test_inexact_ilink_baud_is_refused(tmp_path, baud, needle):
    refused(tmp_path, fc_set("ilink.baud", baud), needles=("flight_computer.ilink.baud", needle))


def test_exact_ilink_baud_other_value_accepted(tmp_path):
    d = derive_with(tmp_path, fc_set("ilink.baud", 500000))
    m = macros(gfp.render(d))
    assert m["ILINK_BAUD"] == "500000UL" and m["ILINK_UBRR"] == "1" and m["ILINK_U2X"] == "0"


@pytest.mark.parametrize("stat_hz, div", [(1.0, 250), (0.5, 500), (0.1, 2500), (2, 125), (250, 1)])
def test_stat_divider_integer_accepted(tmp_path, stat_hz, div):
    d = derive_with(tmp_path, fc_set("ilink.stat_hz", stat_hz))
    assert d["stat_div"] == div
    assert macros(gfp.render(d))["ILINK_STAT_DIV"] == f"{div}U"


@pytest.mark.parametrize("stat_hz, needle", [(3.0, "not com.tick_hz 250 Hz / integer"), (0.3, "/ integer"),
                                             (300, "/ integer"), (0, "must be > 0"), (-1, "must be > 0")])
def test_stat_divider_non_integer_refused(tmp_path, stat_hz, needle):
    refused(tmp_path, fc_set("ilink.stat_hz", stat_hz), needles=("flight_computer.ilink.stat_hz", needle))


@pytest.mark.parametrize("tick", [200, 500, 250.5])
def test_com_tick_other_than_control_tick_refused(tmp_path, tick):
    refused(tmp_path, fc_set("com.tick_hz", tick), needles=("flight_computer.com.tick_hz", "control.loop_hz 250"))


def test_com_clock_and_mcu_checked(tmp_path):
    refused(tmp_path, fc_set("com.f_cpu_hz", 8000000), needles=("flight_computer.com.f_cpu_hz 8000000",))
    refused(tmp_path, fc_set("com.mcu", "ATmega2560"), needles=("flight_computer.com.mcu",))


# --------------------------------------------------------------------------- #
# v0.5 refusals (radio / link UART / link budget) still hold
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("mut", [
    fc_set("link.sx127x.freq_hz", 433500000),        # outside the frequency_note band
    fc_set("link.sx127x.freq_hz", 923280000),        # channel edge + BW/2 leaves the band
    fc_set("link.sx127x.freq_hz", 1100000000),       # outside the chip
    fc_set("link.sx127x.spreading_factor", 6),       # implicit header only
    fc_set("link.sx127x.bandwidth_hz", 100000),
    fc_set("link.sx127x.tx_power_dbm", 20),
    fc_set("link.sx127x.coding_rate_denom", 9),
    fc_set("link.uart_baud", 230400),                # UBRR error 3.5 %
    fc_set("link.radio", "xbee"),
    fc_set("link.vehicle_id", 300),
    fc_set("link.lora_uart_air_rate_bps", 300),      # one lost frame would trip I1p
])
def test_v05_link_refusals(tmp_path, mut):
    refused(tmp_path, mut)


# --------------------------------------------------------------------------- #
# v0.7: mission mode, the VTOL supervisor / vertical-controller macros, gains source, TELEM2 sizing
# --------------------------------------------------------------------------- #
VTOL_TICK_MACROS = {"ONB_V_SPOOL_TICKS": "spool_ticks", "ONB_V_LINK_LAND_TICKS": "link_land_ticks",
                    "ONB_V_ENDURANCE_TICKS": "endurance_ticks", "ONB_V_HARD_CUTOFF_TICKS": "hard_cutoff_ticks",
                    "ONB_V_FALL_CONFIRM_TICKS": "v_fall_confirm_ticks",
                    "ONB_V_SAFE_AFTER_LANDED_TICKS": "safe_after_landed_ticks"}
VTOL_DM_MACROS = {"ONB_V_HOVER_DM": "hover_dm", "ONB_V_ALT_STEP_DM": "alt_step_dm", "ONB_V_ALT_MIN_DM": "alt_min_dm",
                  "ONB_V_ALT_MAX_DM": "alt_max_dm"}
VTOL_F_MACROS = {"ONB_V_CEILING_M": "ceiling_m", "ONB_V_GEOFENCE_M": "v_geofence_m", "ONB_V_FALL_VZ_MPS": "v_fall_vz_mps"}
VC_F_MACROS = {"VC_CLIMB_RATE_MPS": "climb_rate_mps", "VC_DESCENT_RATE_MPS": "descent_rate_mps",
               "VC_TOUCHDOWN_RATE_MPS": "touchdown_rate_mps", "VC_FLARE_ALTITUDE_M": "flare_altitude_m",
               "VC_ALT_KP": "alt_kp", "VC_VZ_KP": "vz_kp", "VC_VZ_KI": "vz_ki", "VC_VZ_I_MAX": "vz_i_max",
               "VC_VZ_MAX_UP_MPS": "vz_max_up_mps", "VC_VZ_MAX_DOWN_MPS": "vz_max_down_mps", "VC_U_MIN": "u_min",
               "VC_U_MAX": "u_max", "VC_U_HOVER": "u_hover", "VC_THROTTLE_EXP": "throttle_exp",
               "VC_COS_TILT_MIN": "cos_tilt_min", "VC_TD_VZ_STILL_MPS": "td_vz_still_mps"}
BALLISTIC_ONB = ("ONB_CLIMB_MILLI", "ONB_RAMP_TICKS", "ONB_COAST_HOLD_TICKS", "ONB_TILT_ABORT_DEG", "ONB_ARM_TILT_MAX_DEG",
                 "ONB_MIN_CELL_V", "ONB_GEOFENCE_M", "ONB_APOGEE_LIMIT_M", "ONB_APOGEE_K", "ONB_FALL_VZ_MPS",
                 "ONB_FALL_CONFIRM_TICKS", "ONB_LINK_ABORT_TICKS", "ONB_LINK_DISARM_TICKS", "ONB_MAX_MOTOR_TICKS",
                 "ONB_RELEASE_AFTER_CUT_TICKS", "ONB_ABORT_RELEASE_TICKS", "ONB_TILT_RATE_ABORT_SQ",
                 "ONB_SPIN_RATE_ABORT_RAD_S", "ONB_RATE_PERSIST_TICKS", "ONB_GYRO_SATURATION_ABORT", "ONB_IMU_FAIL_TICKS",
                 "ONB_BARO_FAIL_TICKS")


def _mode(mode):
    return lambda p: p["mission"].__setitem__("mode", mode)


def test_vtol_macros_are_the_reference_parameters(base, header):
    from sim import link_proto as lp
    from sim.onboard import SupervisorParams, vtol_profiles, vtol_values
    from sim.state_machine import VtolAltitudeFilter
    from sim.vertical_ctrl import VerticalParams
    m = macros(header)
    assert lp.mission_mode(BASE) == lp.MODE_VTOL and base["mode"] == lp.MODE_VTOL
    assert m["MISSION_MODE_BALLISTIC"] == "0" and m["MISSION_MODE_VTOL"] == "1" and m["MISSION_MODE"] == "MISSION_MODE_VTOL"
    assert m["MISSION_HAS_VTOL_PARAMS"] == "1"
    gi = gfp.vtol_gains(BASE)
    assert m["VC_GAINS_TUNED"] == ("1" if gi["tuned"] else "0") and m["VC_GAINS_SOURCE"] == f'"{gi["source"]}"'
    names = vtol_profiles(BASE)
    assert set(names) == {"field", "demo"} and base["vt"]["default"] == vtol_values(BASE)["profile"] == "field"
    for nm in names:
        pm = profile_macros(header, nm)
        spv = SupervisorParams.from_params(BASE, fc_config_path=gfp.FC_CONFIG, mode="vtol", profile=nm)
        for name, field in VTOL_TICK_MACROS.items():
            assert pm[name] == f"{getattr(spv, field)}UL", (nm, name)
        for name, field in VTOL_DM_MACROS.items():
            assert pm[name] == str(getattr(spv, field)), (nm, name)
        for name, field in VTOL_F_MACROS.items():
            assert pm[name] == gfp.cf(getattr(spv, field)), (nm, name)
        # v0.7.1 supervisor integers: V10 timeout, the baro-lost limit, the geofence in dm
        assert pm["ONB_V_LAND_TIMEOUT_TICKS"] == f"{spv.land_timeout_ticks}UL"
        assert pm["ONB_V_BARO_LIMIT_BASE_TICKS"] == f"{spv.baro_limit_base_ticks}UL"
        assert pm["ONB_V_BARO_LIMIT_MT_PER_DM"] == f"{spv.baro_limit_mt_per_dm}UL"
        assert pm["ONB_V_GEOFENCE_DM"] == str(spv.v_geofence_dm)
        vp = VerticalParams.from_params(BASE, base["m_full"], base["t_max"], profile=nm, **gi["gains"])
        for name, field in VC_F_MACROS.items():
            assert pm[name] == gfp.cf(getattr(vp, field)), (nm, name)
        for name, field in (("VC_UP_UM", "up_um"), ("VC_DN_UM", "dn_um"), ("VC_TD_UM", "td_um"), ("VC_FLARE_UM", "flare_um")):
            assert pm[name] == f"{getattr(vp, field)}L", (nm, name)
        assert pm["VC_RAMP_TICKS"] == f"{vp.ramp_ticks}U" and pm["VC_LANDED_CONFIRM_TICKS"] == f"{vp.landed_confirm_ticks}U"
        assert pm["VC_FLOOR_TICKS"] == f"{vp.floor_ticks}U" and pm["VC_VZ_FF"] == ("1" if vp.vz_ff else "0")
        derived = {"VC_UM_TICK_TO_MPS": vp.loop_hz / 1e6, "VC_VZ_KI_DT": vp.vz_ki * vp.dt, "VC_INV_U_HOVER": 1.0 / vp.u_hover,
                   "VC_LEAD_MPS": vp.still_lead * vp.touchdown_rate_mps, "VC_SUPPORT_MPS2": vp.support_g * vp.g,
                   "VC_G": vp.g, "VC_THRUST_ALPHA": vp.thrust_alpha, "VC_STILL_ALPHA": vp.still_alpha,
                   "VC_SUPPORT_ALPHA": vp.support_alpha,
                   # v0.7.1b thrust learning (mission.vtol.thrust_learning)
                   "VC_KT_TAU_S": vp.kt_tau_s, "VC_KT_ALPHA": vp.kt_alpha, "VC_KT_MIN": vp.kt_min, "VC_KT_MAX": vp.kt_max,
                   "VC_KT_MAX_TILT_DEG": vp.kt_max_tilt_deg, "VC_KT_COS_MIN": vp.kt_cos_min}
        for name, x in derived.items():
            assert pm[name] == gfp.cf(x), (nm, name)
        tl = vtol_values(BASE, nm)["thrust_learning"]
        assert (vp.kt_tau_s, vp.kt_min, vp.kt_max, vp.kt_max_tilt_deg) == (tl["tau_s_ASSUMED"], tl["k_min"], tl["k_max"],
                                                                           tl["max_tilt_deg"])
        # v0.7.1b: -1/n and 1/ramp_ticks are no longer stored (vertical_ctrl.h: 8 B of RAM)
        assert "VC_NEG_INV_EXP" not in pm and "VC_INV_RAMP_TICKS" not in pm
        assert pm["VC_THROTTLE_EXP_INT"] == m["MIX_THROTTLE_EXP_INT"]
        # the hover throttle of the controller is the generator's own FC_HOVER_THROTTLE (same mass, T_max, n)
        assert pm["VC_U_HOVER"] == m["FC_HOVER_THROTTLE"]
        # the NAV estimator: mission.vtol.nav_estimator of the profile (or the tuned kp / ki)
        ne = vtol_values(BASE, nm)["nav_estimator"]
        kp, ki = gi["est"].get("kp", float(ne["kp_ASSUMED"])), gi["est"].get("ki", float(ne["ki_ASSUMED"]))
        f_a = gi["est"].get("accel_lpf_hz", float(ne["accel_lpf_hz_ASSUMED"]))
        dt = 1.0 / int(BASE["control"]["loop_hz"])
        est = {"EST_V_KP": kp, "EST_V_KI": ki, "EST_V_KI_DT": ki * dt, "EST_V_ACCEL_LPF_HZ": f_a,
               "EST_V_GATE_MPS2": float(ne["accel_gate"]) * gfp.G0, "EST_V_G": gfp.G0,
               "EST_V_ALPHA_A": 1.0 - math.exp(-2.0 * math.pi * f_a * dt),
               "EST_V_BIAS_LIMIT_RAD_S": math.radians(float(ne["bias_limit_dps"]))}
        for name, x in est.items():
            assert pm[name] == gfp.cf(x), (nm, name)
        # the NAV altitude filter: the reference's own gains from the 6-DOF's SensorConfig
        from sim.sim6dof import SensorConfig
        sc = SensorConfig()
        f = VtolAltitudeFilter(sc.altitude_crossover_hz, sc.altitude_zeta,
                               float(getattr(sc, "vtol_alt_bias_tau_s", 40.0)), 0.0, sc.pad_avg_tau_s)
        for name, x in (("ALTV_K1", f.k1), ("ALTV_K2", f.k2), ("ALTV_K3", f.k3), ("ALTV_BIAS_LIMIT_MPS2", f.bias_limit),
                        ("ALTV_COMP_M", float(vtol_values(BASE, nm).get("baro_thrust_comp_m", 0.0)))):
            assert pm[name] == gfp.cf(x), (nm, name)
        assert pm["ALTV_PAD_N_AVG"] == f"{max(1, int(round(sc.pad_avg_tau_s * int(BASE['control']['loop_hz']))))}U"
    # the profiles really differ where params.json says so, and the default comes first (first-#define text parsers)
    pf, pd = profile_macros(header, "field"), profile_macros(header, "demo")
    assert pf["ONB_V_HOVER_DM"] != pd["ONB_V_HOVER_DM"] and pf["VC_UP_UM"] != pd["VC_UP_UM"]
    assert header.index("MISSION_PROFILE_ID_FIELD  /* ===") < header.index("MISSION_PROFILE_ID_DEMO  /* ===")
    assert "#if defined(MISSION_PROFILE_DEMO)" in header and m["NLINK_FCU_THR_SCALE"] == "250u"
    # the ballistic set is the v0.6.3 SupervisorParams (mode 'ballistic') whatever mission.mode says
    spb = SupervisorParams.from_params(BASE, fc_config_path=gfp.FC_CONFIG, mode="ballistic")
    assert m["ONB_MAX_MOTOR_TICKS"] == f"{spb.max_motor_ticks}UL" and m["ONB_GEOFENCE_M"] == gfp.cf(spb.geofence_m)


def test_profile_refusals_and_single_profile(tmp_path):
    refused(tmp_path, lambda p: p["mission"]["vtol"]["profiles"].__setitem__("Demo-2", {"hover_altitude_m": 2.0}),
            needles=("profile name",))
    refused(tmp_path, lambda p: p["mission"]["vtol"]["profiles"]["demo"].__setitem__("hover_altitude_mm", 2.0),
            needles=("hover_altitude_mm",))
    d = derive_with(tmp_path, lambda p: p["mission"]["vtol"]["profiles"].pop("demo"))
    text = gfp.render(d)
    assert list(d["vt"]["profiles"]) == ["field"] and "MISSION_PROFILE_DEMO" not in text
    assert macros(text)["MISSION_PROFILE"] == "MISSION_PROFILE_ID_FIELD"
    # the default profile follows mission.vtol.profile: demo first, the field switch for the other
    d = derive_with(tmp_path, lambda p: p["mission"]["vtol"].__setitem__("profile", "demo"))
    text = gfp.render(d)
    assert list(d["vt"]["profiles"])[0] == "demo" and "#if defined(MISSION_PROFILE_FIELD)" in text
    assert text.index("MISSION_PROFILE_ID_DEMO  /* ===") < text.index("MISSION_PROFILE_ID_FIELD  /* ===")


def test_ballistic_mode_header_keeps_the_v063_macros(tmp_path, header):
    mv = macros(header)
    d = derive_with(tmp_path, _mode("ballistic"))
    mb = macros(gfp.render(d))
    assert mb["MISSION_MODE"] == "MISSION_MODE_BALLISTIC" and mb["MISSION_HAS_VTOL_PARAMS"] == "1"
    for name in BALLISTIC_ONB:                                   # identical in both builds
        assert mb[name] == mv[name], name
    # the reply the build sends sizes the SX127x TX timeout: TELEM (36 B) vs TELEM2 (39 B) / v0.8 with GPS TELEM3 (45 B)
    assert d["telem_bytes"] == 36 and "ToA(TELEM 36 B)" in gfp.render(d)
    # 2026-11-10: the VTOL battery build's reply carries the logic battery (TELEM6 40 B / with GPS TELEM7 46 B)
    vt = gfp.derive()
    vt_bytes = vt["telem_bytes"]
    vt_name = {39: "TELEM2", 45: "TELEM3", 40: "TELEM6", 46: "TELEM7"}[vt_bytes]
    lbat_build = BASE["flight_computer"]["logic_battery_monitor"]["telemetry_builds"] == "vtol_battery"
    assert vt_bytes == (46 if lbat_build else 45) and f"ToA({vt_name} {vt_bytes} B)" in header
    assert int(mb["SX127X_TX_TIMEOUT_TICKS"].rstrip("UL")) <= int(mv["SX127X_TX_TIMEOUT_TICKS"].rstrip("UL"))
    # the link budget: the VTOL build's downlink is the longer frame
    for r in d["budgets"]:
        assert d["budgets"][r]["telem_frame_bytes"] == 36
    vt = gfp.derive()
    for r in vt["budgets"]:
        b, bb = vt["budgets"][r], d["budgets"][r]
        assert b["telem_frame_bytes"] == vt_bytes and b["downlink_s"] > bb["downlink_s"] and b["rtt_worst_s"] > bb["rtt_worst_s"]


def test_v063_params_without_mode_or_vtol(tmp_path, header):
    d = derive_with(tmp_path, lambda p: p["mission"].pop("mode"), lambda p: p["mission"].pop("vtol"))
    text = gfp.render(d)
    m = macros(text)
    assert d["mode"] == "ballistic" and d["vt"] is None
    assert m["MISSION_MODE"] == "MISSION_MODE_BALLISTIC" and m["MISSION_HAS_VTOL_PARAMS"] == "0"
    assert not [k for k in m if k.startswith(("VC_", "ONB_V_"))]
    mv = macros(header)
    for name in BALLISTIC_ONB:
        assert m[name] == mv[name], name


def test_vtol_refusals(tmp_path):
    refused(tmp_path, lambda p: p["mission"].pop("vtol"), needles=("mission.vtol",))
    refused(tmp_path, lambda p: p["mission"].__setitem__("mode", "hover"), needles=("mission.mode",))
    refused(tmp_path, lambda p: p["mission"]["vtol"].__setitem__("alt_max_m", 30.0), needles=("ceiling",))
    refused(tmp_path, lambda p: p["mission"]["vtol"].__setitem__("fall_vz_mps", -0.8), needles=("fall_vz_mps",))
    refused(tmp_path, lambda p: p["mission"]["vtol"].__setitem__("u_min", 1.0), needles=("u_min",))
    refused(tmp_path, lambda p: p["mission"]["vtol"].__setitem__("hard_cutoff_s", 20.0), needles=("hard_cutoff",))


def test_vtol_gains_source_and_stale_flag(tmp_path):
    p = copy.deepcopy(BASE)
    params = tmp_path / "params.json"
    params.write_text(json.dumps(p), encoding="utf-8")
    sha = hashlib.sha256(params.read_bytes()).hexdigest()
    g = tmp_path / "vtol_gains.json"
    assert not gfp.vtol_gains(p, str(params), str(g))["tuned"]                         # absent: params ASSUMED
    assert gfp.vtol_gains(p, str(params), str(g))["gains"] == {k: float(v) for k, v in
                                                               p["mission"]["vtol"]["gains_ASSUMED"].items()}
    g.write_text(json.dumps({"alt_kp": 1.4, "vz_kp": 0.2, "vz_ki": 0.1, "vz_i_max": 0.12, "_meta": {"params_sha256": sha}}),
                 encoding="utf-8")
    gi = gfp.vtol_gains(p, str(params), str(g))
    assert gi["tuned"] and not gi["params_stale"] and gi["gains"]["alt_kp"] == 1.4 and not gi["warnings"]
    g.write_text(json.dumps({"alt_kp": 1.4, "vz_kp": 0.2, "vz_ki": 0.1, "vz_i_max": 0.12, "_meta": {"params_sha256": "0" * 64}}),
                 encoding="utf-8")
    gi = gfp.vtol_gains(p, str(params), str(g))
    assert gi["tuned"] and gi["params_stale"] and "another params.json" in gi["warnings"][0]
    g.write_text(json.dumps({"alt_kp": 1.4, "vz_kp": 0.2, "vz_ki": float("nan")}), encoding="utf-8")
    gi = gfp.vtol_gains(p, str(params), str(g))
    assert not gi["tuned"] and "lacks finite" in gi["warnings"][0]
    # v0.7.1: the NAV estimator's kp / ki from the same file (nav_estimator block or the 6-DOF's gm_* names)
    g.write_text(json.dumps({"alt_kp": 1.4, "vz_kp": 0.2, "vz_ki": 0.1, "vz_i_max": 0.12, "gm_kp": 0.2, "gm_ki": 0.01,
                             "_meta": {"params_sha256": sha}}), encoding="utf-8")
    gi = gfp.vtol_gains(p, str(params), str(g))
    assert gi["est_tuned"] and gi["est"] == {"kp": 0.2, "ki": 0.01} and not gi["params_stale"]
    g.write_text(json.dumps({"est_kp": 0.15, "est_ki": 0.01, "est_accel_lpf_hz": 1.0}), encoding="utf-8")   # --tune names
    assert gfp.vtol_gains(p, str(params), str(g))["est"] == {"kp": 0.15, "ki": 0.01, "accel_lpf_hz": 1.0}
    g.write_text(json.dumps({"nav_estimator": {"kp_ASSUMED": 0.12}, "_meta": {"params_sha256": "0" * 64}}), encoding="utf-8")
    gi = gfp.vtol_gains(p, str(params), str(g))
    assert gi["est"] == {"kp": 0.12} and not gi["tuned"] and gi["params_stale"]
    g.write_text(json.dumps({"est_kp": -1.0}), encoding="utf-8")
    gi = gfp.vtol_gains(p, str(params), str(g))
    assert not gi["est_tuned"] and any("finite number >= 0" in w for w in gi["warnings"])


def test_gains_header_stale_rule_hashes_the_attitude_tuning_subtrees(tmp_path):
    """tools/gen_gains_header.py: a params.json that differs from the tuned one only in excluded keys keeps the
    attitude gains current; any other key makes them STALE."""
    import gen_gains_header as ggh
    p = copy.deepcopy(BASE)
    base_dg = ggh.attitude_tuning_digest(p)
    for mut in (lambda q: q["meta"].__setitem__("version", "9.9"), lambda q: q["mission"]["vtol"].__setitem__("alt_max_m", 3),
                lambda q: q["mission"].__setitem__("mode", "ballistic"), lambda q: q["aero"].__setitem__("vtol_new_ASSUMED", 1),
                lambda q: q["printer"].__setitem__("color", "Red")):
        q = copy.deepcopy(p)
        mut(q)
        assert ggh.attitude_tuning_digest(q) == base_dg
    for mut in (lambda q: q["control"].__setitem__("loop_hz", 200), lambda q: q["edf"].__setitem__("thrust_nominal_N", 31.0),
                lambda q: q["mission"].__setitem__("tilt_abort_deg", 40.0), lambda q: q["aero"].__setitem__("rho_kg_m3", 1.1),
                lambda q: q["printer"].__setitem__("density_g_cm3", 1.25), lambda q: q.__setitem__("new_block", {})):
        q = copy.deepcopy(p)
        mut(q)
        assert ggh.attitude_tuning_digest(q) != base_dg
    # resolve(): tuned against another file whose attitude subtrees are these -> not stale; changed -> stale
    params = tmp_path / "params.json"
    params.write_text(json.dumps(p), encoding="utf-8")
    gj = {k: 0.1 for k in ggh.GAIN_KEYS}
    gj["_meta"] = {"params_sha256": "f" * 64, "params_attitude_digest": base_dg}
    gpath = tmp_path / "gains.json"
    gpath.write_text(json.dumps(gj), encoding="utf-8")
    info = ggh.resolve(str(params), str(gpath))
    assert not info["params_stale"] and "only outside the attitude-tuning subtrees" in info["stale_rule"]
    p2 = copy.deepcopy(p)
    p2["vane"]["cl_alpha_per_rad"] = 3.0
    params.write_text(json.dumps(p2), encoding="utf-8")
    info = ggh.resolve(str(params), str(gpath))
    assert info["params_stale"] and info["stale_rule"] == "attitude-tuning subtrees changed"
    gj["_meta"] = {"params_sha256": "f" * 64}                   # no digest recorded anywhere: stale
    gpath.write_text(json.dumps(gj), encoding="utf-8")
    assert ggh.resolve(str(params), str(gpath))["params_stale"]
    # the recorded v0.6.3 entry: sim/out/gains.json names it, and the current params.json has its digest
    with open(ggh.GAINS_JSON, "r", encoding="utf-8") as f:
        sha = json.load(f)["_meta"]["params_sha256"]
    assert sha in ggh.TUNED_PARAMS_DIGESTS


def test_vtol_attitude_overrides_and_learning_guard(tmp_path, header):
    """v0.7.1b: params.json control.vtol_overrides -> VTOL_OV_<KEY> (core/fc_config.c fc_vtol_gains, the ballistic build
    never reads them; sim.controller.gains_for_mode is the reference); an override the C cannot apply, a loop rate or a
    bad value is refused; VC_KT_R_MIN of core/vertical_ctrl.h equals the reference's KT_R_MIN_ASSUMED."""
    import re
    from sim import vertical_ctrl as vcm
    from sim.controller import vtol_overrides
    m = macros(header)
    ov = vtol_overrides(BASE["control"])
    assert m["VTOL_OV_COUNT"] == str(len(ov)) and set(ov) <= set(gfp.VTOL_OV_C_FIELDS)
    for k, val in ov.items():                               # v0.9: the integer switches as integers, the rest float32
        want = (("1" if val else "0") if k == "yaw_hold" else
                (str(int(val)) if k in gfp.VTOL_OV_INT_CHOICES else gfp.cf(float(val))))
        assert m[f"VTOL_OV_{k.upper()}"] == want, k
    fc = open(os.path.join(gfp.FW_DIR, "src", "core", "fc_config.c"), encoding="utf-8").read()
    for k, field in gfp.VTOL_OV_C_FIELDS.items():          # every key the generator may write, fc_vtol_gains applies
        if field.startswith("v9."):                         # v0.9: the options come from the VTOL_V09_* block (below)
            continue
        assert f"#ifdef VTOL_OV_{k.upper()}" in fc and f"g->{field} = VTOL_OV_{k.upper()}" in fc, k
    # v0.9: VTOL_V09_* = the reference AttitudeController's own option values and derived constants (VTOL gains)
    from sim.controller import AttitudeController, gains_for_mode
    gj = os.path.join(gfp.ROOT, "sim", "out", "gains.json")
    tuned = json.load(open(gj, encoding="utf-8")) if os.path.exists(gj) else None
    g = gains_for_mode(BASE["control"], tuned, lp.MODE_VTOL)

    class _Plant:
        delta_max_rad = 0.35
        t_max_n = 30.0
    ac = AttitudeController.__new__(AttitudeController)
    ac.dt = 1.0 / float(g["loop_hz"])
    tau = float(g.get("ff_rotor_tau_s", 0.15))
    assert m["VTOL_V09_ERR_MODE"] == str(int(g.get("att_err_mode", 0)))
    assert m["VTOL_V09_AW_PER_AXIS"] == str(int(bool(g.get("aw_per_axis", 0))))
    assert m["VTOL_V09_FF_GYRO"] == str(int(bool(g.get("ff_rotor_gyro", 0))))
    assert m["VTOL_V09_YAW_TRIM_RESERVE"] == str(int(bool(g.get("mix_yaw_trim_reserve", 0))))
    assert m["VTOL_V09_FF_A"] == gfp.cf(ac.dt / (tau + ac.dt))
    k_r = float(g.get("ff_rotor_h_nms", 0.0)) / tau / float(g["roll_priority_scale"])
    assert m["VTOL_V09_FF_NKR"] == gfp.cf(-float(g.get("ff_rotor_react", 0)) * k_r)
    for key in ("VTOL_V09_ERR_MODE", "VTOL_V09_FF_NKR", "VTOL_V09_FF_A", "VTOL_V09_YAW_TRIM_RESERVE"):
        assert key in fc, key
    vh = open(os.path.join(gfp.FW_DIR, "src", "core", "vertical_ctrl.h"), encoding="utf-8").read()
    mm = re.search(r"#define VC_KT_R_MIN ([0-9.]+)f", vh)
    assert mm and float(mm.group(1)) == vcm.KT_R_MIN_ASSUMED
    for bad in ({"rate_i_max_q": 0.3}, {"loop_hz": 500}, {"rate_i_max_z": -0.1}, {"rate_i_max_z": "x"},
                {"att_err_mode": 3}, {"att_err_mode": 1.5}, {"aw_per_axis": 2}, {"ff_rotor_tau_s": 0.0},
                {"mix_yaw_reserve_deg": 8.0}, {"yaw_i_thrust_norm": 1}):    # v0.9: not ported / out of range
        p = copy.deepcopy(BASE)
        p["control"]["vtol_overrides"] = dict(bad, note="test")
        path = tmp_path / "params.json"
        path.write_text(json.dumps(p), encoding="utf-8")
        with pytest.raises(gfp.GenError):
            gfp.derive(str(path))


# ---- v0.8: GPS position hold (POS_*), the NAV GPS side (GPS_*), the take-off check, the generator's limits ----
POS_F_FIELDS = {"POS_T": "T", "POS_G": "g", "POS_TW": "tw", "POS_KP_POS": "kp_pos", "POS_KV": "kv", "POS_KI": "ki",
                "POS_KD": "kd", "POS_ALPHA_D": "alpha_d", "POS_I_MAX": "i_max", "POS_V_MAX": "v_max",
                "POS_V_MAX_LAND": "v_max_land", "POS_FLARE_SPEED": "flare_speed", "POS_FLARE_ALTITUDE_M": "flare_altitude_m",
                "POS_COS_CAP": "cos_cap", "POS_COS_FLARE_CAP": "cos_flare_cap", "POS_COS_EXCESS": "cos_excess",
                "POS_SIN_EXCESS": "sin_excess", "POS_MARGIN_G": "margin_g", "POS_MARGIN_FRAC": "margin_frac",
                "POS_H_ENABLE_M": "h_enable_m", "POS_FENCE_M": "fence_m", "POS_SPIN_Z_RAD": "spin_z_rad",
                "POS_SPIN_XY_RAD2": "spin_xy_rad2", "POS_AUTHORITY_MIN_KTW": "authority_min_ktw",
                "POS_TILT_FADE_M": "tilt_fade_m", "POS_SLEW_PER_TICK": "slew_per_tick"}
POS_U_FIELDS = {"POS_STEP_TICKS": "step_ticks", "POS_GUARD_STEPS": "guard_steps", "POS_TIMEOUT_STEPS": "timeout_steps",
                "POS_LAND_STEPS": "land_steps", "POS_PLAUS_LIMITED_STEPS": "plaus_limited_steps",
                "POS_PLAUS_VEXC_STEPS": "plaus_vexc_steps", "POS_PLAUS_RESET_WINDOW_STEPS": "plaus_reset_window_steps",
                "POS_PLAUS_RESETS": "plaus_resets"}


def test_v08_position_macros_are_the_reference_parameters(base, header):
    """Every POS_* macro of each build profile = sim.position_ctrl.PositionParams.from_params(profile) (the controller's
    model mass with the GPS pod, T_max), the supervisor switches and the take-off check (VC_TAKEOFF_*) likewise."""
    from sim.onboard import SupervisorParams, vtol_profiles
    from sim.position_ctrl import PositionParams
    from sim.vertical_ctrl import VerticalParams
    gi = gfp.vtol_gains(BASE)
    for nm in vtol_profiles(BASE):
        pm = profile_macros(header, nm)
        spv = SupervisorParams.from_params(BASE, fc_config_path=gfp.FC_CONFIG, mode="vtol", profile=nm)
        assert pm["ONB_V_GPS_HOLD"] == ("1" if spv.gps_hold else "0") and pm["ONB_V_TAKEOFF_CHECK"] == \
            ("1" if spv.takeoff_check else "0")
        vp = VerticalParams.from_params(BASE, base["m_full"], base["t_max"], profile=nm, **gi["gains"])
        assert pm["VC_TAKEOFF_TICKS"] == f"{vp.takeoff_ticks}U" and pm["VC_TAKEOFF_CLIMB_M"] == gfp.cf(vp.takeoff_climb_m)
        if not spv.gps_hold:
            assert "POS_KP_POS" not in pm
            continue
        pp = PositionParams.from_params(BASE, base["m_full"], base["t_max"], g=gfp.G0, profile=nm, **gi["pos"])
        for name, field in POS_F_FIELDS.items():
            assert pm[name] == gfp.cf(getattr(pp, field)), (nm, name)
        for name, field in POS_U_FIELDS.items():
            assert pm[name] == f"{getattr(pp, field)}U", (nm, name)
        assert pm["POS_PLAUS_VT"] == gfp.cf(pp.v_max + pp.plaus_v_excess_mps)
        assert pp.step_ticks >= 5 and 1 <= pp.plaus_resets <= 4          # the five-tick pipeline, POS_RESETS_MAX


def test_v08_gps_macros_are_the_reference_parameters(base, header):
    """GPS_* = sim.gps_nav.NavGpsParams.from_params and its fixed gains (compute_gains), the configurator's tick counts
    (GpsConfigurator), the integer thresholds that decide like the reference's float compares."""
    from sim import gps_nav as gpn
    m = macros(header)
    gp = gpn.NavGpsParams.from_params(BASE)
    gn = gp.gains()
    cfg = gpn.GpsConfigurator(gp.loop_hz, gp.rate_hz)
    assert m["GPS_STEP_TICKS"] == f"{gp.step_ticks}U" == "5U" and m["GPS_EPOCH_SPLIT_TICKS"] == "4"
    assert m["GPS_T_STEP"] == gfp.cf(gp.t_step) and m["GPS_INV_STEP"] == gfp.cf(1.0 / gp.step_ticks)
    assert m["GPS_LOOP_HZ_F"] == gfp.cf(float(gp.loop_hz)) and m["GPS_MS_PER_TICK"] == f"{gp.ms_per_tick}U"
    assert m["GPS_LATENCY_MS"] == f"{gp.latency_ms}U" and m["GPS_N_CORR"] == f"{gp.n_corr}U" and 1 <= gp.n_corr <= 4
    for name, n in (("GPS_CFG_N_WAIT", cfg.n_wait), ("GPS_CFG_N_ACK", cfg.n_ack), ("GPS_CFG_N_SETTLE", cfg.n_settle),
                    ("GPS_CFG_N_VER", cfg.n_ver)):
        assert m[name] == f"{n}U", name
    assert m["GPS_BAUD"] == f"{gpn.GPS_BAUD}UL" and m["GPS_MEAS_MS"] == f"{int(round(1000.0 / gp.rate_hz))}U"
    for tag, gs in (("STEADY", gn.steady), ("ACQ", gn.acq)):
        for r in range(3):
            assert m[f"GPS_K_{tag}_P_{r}"] == gfp.cf(gs.k_p[r]) and m[f"GPS_K_{tag}_V_{r}"] == gfp.cf(gs.k_v[r])
            for c in range(2):
                assert m[f"GPS_K_{tag}_PV_{r}{c}"] == gfp.cf(gs.k_pv[r][c])
    assert m["GPS_GATE_V2"] == gfp.cf(gp.gate_sigma * gp.gate_sigma * gn.s_vv)
    assert m["GPS_GATE_P2"] == gfp.cf(gp.gate_p_m * gp.gate_p_m)
    # integer thresholds: the largest mm / cm/s whose reference compare (x / 1000 <= max) holds
    for name, thr, scale in (("GPS_GATE_MAX_HACC_MM", gp.gate_max_hacc_m, 1000.0),
                             ("GPS_GATE_MAX_SACC_CMS", gp.gate_max_sacc_mps, 100.0),
                             ("GPS_OK_MAX_HACC_MM", gp.ok_max_hacc_m, 1000.0)):
        x = int(m[name].rstrip("UL"))
        assert x / scale <= thr < (x + 1) / scale, name
    for i, ax in enumerate("XYZ"):
        assert m[f"GPS_LEVER_B_{ax}_M"] == gfp.cf(gp.lever_b_m[i] if gp.lever_comp else 0.0)


def test_v08_refusals(tmp_path, monkeypatch):
    """The generator refuses what core/position_ctrl.c / core/gps_nav.c cannot run."""
    import dataclasses
    real = gfp.PositionParams.from_params

    def five_resets(*a, **kw):                            # plausibility (c) with more resets than the stamp ring holds
        return dataclasses.replace(real(*a, **kw), plaus_resets=5)

    monkeypatch.setattr(gfp.PositionParams, "from_params", five_resets)
    refused(tmp_path, needles=("plaus_resets", "POS_RESETS_MAX"))
    monkeypatch.setattr(gfp.PositionParams, "from_params", real)
    refused(tmp_path, lambda p: p["flight_computer"]["nav"]["gps"].__setitem__("epoch_split_ticks", 3),
            needles=("epoch_split_ticks",))
    refused(tmp_path, lambda p: p["flight_computer"]["nav"]["gps"].__setitem__("fusion_hz", 25), needles=("fusion_hz",))
    refused(tmp_path, lambda p: p["flight_computer"]["nav"]["gps"].__setitem__("usart", 0), needles=("usart",))
    refused(tmp_path, lambda p: p["mission"]["vtol"]["position_hold"].__setitem__("loop_hz", 125),
            needles=("loop_hz",))


def test_v08_gains_header_ignores_the_vtol_overrides():
    """tools/gen_gains_header.py: control.vtol_overrides (the VTOL build's attitude overrides, v0.7.1b) is outside the
    ballistic attitude-tuning digest - editing it does not make gains.h STALE."""
    import gen_gains_header as ggh
    p = copy.deepcopy(BASE)
    dg = ggh.attitude_tuning_digest(p)
    q = copy.deepcopy(p)
    q["control"]["vtol_overrides"] = {"rate_i_max_z": 0.5, "note": "x"}
    assert ggh.attitude_tuning_digest(q) == dg
    q["control"]["rate_i_max_z_new"] = 1.0
    assert ggh.attitude_tuning_digest(q) != dg


# --------------------------------------------------------------------------- #
# v0.8.1: the four-chip build (glink, the GNS board, the touchdown policy)
# --------------------------------------------------------------------------- #
def test_v081_glink_macros_are_the_reference_values(base, header):
    """GLINK_* / GNS_LOOP_HZ = sim/glink_proto.py (baud, frame sizes and line times, the five-tick cycle), the UBRR the
    params carry (U2X = 0, exact at 16 MHz), the ASSUMED latency within the 4-entry result ring."""
    from sim import glink_proto as glp
    m = macros(header)
    g = BASE["flight_computer"]["glink"]
    assert base["vt"]["glink"] is not None
    assert m["GLINK_BAUD"] == f"{glp.BAUD}UL" == f"{int(g['baud'])}UL"
    ubrr = int(m["GLINK_UBRR"])
    assert ubrr == int(g["ubrr"]) and m["GLINK_U2X"] == "0"
    assert F_CPU / (16 * (ubrr + 1)) == glp.BAUD                       # exact: both ends run the same crystal
    lat = int(m["GLINK_LATENCY_TICKS"].rstrip("U"))
    assert lat == int(g.get("latency_ticks_ASSUMED", glp.LATENCY_TICKS_DEFAULT))
    assert gfp.GLINK_LATENCY_RANGE[0] <= lat <= gfp.GLINK_LATENCY_RANGE[1]
    assert m["GLINK_STATE_FRAME_BYTES"] == f"{glp.STATE_FRAME_BYTES}U" == "60U"
    assert m["GLINK_RESULT_FRAME_BYTES"] == f"{glp.RESULT_FRAME_BYTES}U" == "26U"
    assert m["GLINK_STATE_FRAME_US"] == f"{round(glp.frame_time_s(glp.STATE_FRAME_BYTES) * 1e6)}UL"
    assert m["GLINK_RESULT_FRAME_US"] == f"{round(glp.frame_time_s(glp.RESULT_FRAME_BYTES) * 1e6)}UL"
    assert m["GNS_LOOP_HZ"] == f"{int(BASE['flight_computer']['gns']['loop_hz'])}U"
    assert int(m["GNS_LOOP_HZ"].rstrip("U")) * glp.STEP_TICKS == int(m["GPS_LOOP_HZ_F"].rstrip("f").split(".")[0])


def test_v081_touchdown_policy_macro_and_c_fields(tmp_path, base, header):
    """POS_TOUCHDOWN_KEEP_TRIM per profile = PositionParams.touchdown_policy (params touchdown_policy.policy); the host
    test / cross-check mapping (pos_c_fields) carries it as keep_trim; 'fade' (the v0.8 comparison law) renders 0."""
    from sim.onboard import vtol_profiles
    from sim.position_ctrl import PositionParams
    gi = gfp.vtol_gains(BASE)
    seen = 0
    for nm in vtol_profiles(BASE):
        pm = profile_macros(header, nm)
        if "POS_KP_POS" not in pm:
            assert "POS_TOUCHDOWN_KEEP_TRIM" not in pm
            continue
        pp = PositionParams.from_params(BASE, base["m_full"], base["t_max"], g=gfp.G0, profile=nm, **gi["pos"])
        keep = pp.touchdown_policy == "keep_trim"
        assert pm["POS_TOUCHDOWN_KEEP_TRIM"] == ("1" if keep else "0"), nm
        f = gfp.pos_c_fields(pp)
        code = gfp.POS_TD_POLICY_CODES[pp.touchdown_policy]      # v0.9: the policy code (keep_vel 2)
        assert f["keep_trim"] == code and pm["POS_TOUCHDOWN_POLICY"] == str(code)
        assert set(gfp.POS_C_FLOAT_FIELDS) | set(gfp.POS_C_INT_FIELDS) | {"plaus_vt", "keep_trim"} == set(f)
        seen += 1
    assert seen >= 1
    d = derive_with(tmp_path, lambda p: p["mission"]["vtol"]["position_hold"]["touchdown_policy"].__setitem__(
        "policy", "fade"))
    m = macros(gfp.render(d))
    assert m["POS_TOUCHDOWN_KEEP_TRIM"] == "0"
    refused(tmp_path, lambda p: p["mission"]["vtol"]["position_hold"]["touchdown_policy"].__setitem__("policy", "hover"),
            needles=("touchdown_policy",))


def test_v081_gps_c_fields_match_the_gps_macros(base, header):
    """gps_c_fields (the host test's 'gp.<key>=' overrides) holds the values the GPS_* macros carry - the cross-check of
    the GNS node starts from the same gps_nav_params_t as core/fc_config.c fc_default_gns."""
    from sim import gps_nav as gpn
    m = macros(header)
    gp = gpn.NavGpsParams.from_params(BASE)
    f = gfp.gps_c_fields(gp)
    assert f["step_ticks"] == int(m["GPS_STEP_TICKS"].rstrip("U")) and f["n_corr"] == int(m["GPS_N_CORR"].rstrip("U"))
    assert gfp.cf(f["t_step"]) == m["GPS_T_STEP"] and gfp.cf(f["inv_step"]) == m["GPS_INV_STEP"]
    assert gfp.cf(f["loop_hz_f"]) == m["GPS_LOOP_HZ_F"] and gfp.cf(f["gate_v2"]) == m["GPS_GATE_V2"]
    assert f["gate_max_hacc_mm"] == int(m["GPS_GATE_MAX_HACC_MM"].rstrip("UL"))
    assert f["ok_max_hacc_mm"] == int(m["GPS_OK_MAX_HACC_MM"].rstrip("UL"))
    for r in range(3):
        assert gfp.cf(f[f"s_kp{r}"]) == m[f"GPS_K_STEADY_P_{r}"] and gfp.cf(f[f"a_kv{r}"]) == m[f"GPS_K_ACQ_V_{r}"]
    for i, ax in enumerate("XYZ"):
        assert gfp.cf(f[f"lever_b{i}"]) == m[f"GPS_LEVER_B_{ax}_M"]
    assert f["yaw_window_steps"] == gfp.yaw_window_steps(gp) >= 1


def _gl(key, val):
    return lambda p: p["flight_computer"]["glink"].__setitem__(key, val)


def _gns(key, val):
    return lambda p: p["flight_computer"]["gns"].__setitem__(key, val)


@pytest.mark.parametrize("mut, needles", [
    (_gl("baud", 115200), ("glink.baud", "BAUD")),
    (_gl("ubrr", 4), ("glink.ubrr",)),
    (_gl("latency_ticks_ASSUMED", 1), ("latency_ticks_ASSUMED",)),
    (_gl("latency_ticks_ASSUMED", 17), ("latency_ticks_ASSUMED",)),
    (_gns("loop_hz", 25), ("gns.loop_hz",)),
    (_gns("f_cpu_hz", 8000000), ("gns.f_cpu_hz",)),
    (_gns("mcu", "ATmega328P"), ("gns.mcu",)),
    (lambda p: p["flight_computer"].pop("gns"), ("flight_computer.gns",)),
])
def test_v081_glink_refusals(tmp_path, mut, needles):
    """The generator refuses a glink / GNS setting the four-chip core cannot run (core/glink_proto.c frames at one
    baud, the NAV's 4-entry result ring, one STATE / RESULT pair per five NAV ticks, one F_CPU)."""
    refused(tmp_path, mut, needles=needles)


def test_v081_glink_latency_range_accepted(tmp_path):
    """L = 2 (the GNS answers in the NAV's next tick at the earliest) and L = 16 (the ring's depth) render."""
    from sim import glink_proto as glp
    for lat in gfp.GLINK_LATENCY_RANGE:
        d = derive_with(tmp_path, _gl("latency_ticks_ASSUMED", lat))
        assert d["vt"]["glink"]["latency"] == lat
        assert macros(gfp.render(d))["GLINK_LATENCY_TICKS"] == f"{lat}U"
    assert gfp.GLINK_LATENCY_RANGE[0] <= glp.LATENCY_TICKS_DEFAULT <= gfp.GLINK_LATENCY_RANGE[1]


# ---- v0.9: vertical-controller take-off / reference / touchdown macros, power rules ----
def test_v09_vertical_macros_are_the_reference_values(header):
    """VC_RAMP_TOTAL_TICKS / VC_TO_* / VC_REF_APPROACH_* / VC_TD_REL_* per profile = sim.vertical_ctrl.VerticalParams of
    that profile (the cached properties the reference flies); VC_TD_REL_OK separates -release_max from the next lattice
    value; the two-slope ceiling reaches u_max at k + 1 = ramp_total_ticks (core/vertical_ctrl.c short cut)."""
    from sim.onboard import vtol_profiles
    from sim.vertical_ctrl import VerticalParams
    gi = gfp.vtol_gains(BASE)
    d = gfp.derive()
    for nm in vtol_profiles(BASE):
        pm = profile_macros(header, nm)
        vp = VerticalParams.from_params(BASE, d["m_full"], d["t_max"], g=gfp.G0, profile=nm, **gi["gains"])
        assert pm["VC_RAMP_TOTAL_TICKS"] == f"{vp.ramp_total_ticks}U", nm
        assert pm["VC_TO_U_START"] == gfp.cf(vp.to_u_start) and pm["VC_TO_S2"] == gfp.cf(vp.to_slew_per_s / vp.loop_hz)
        assert pm["VC_TO_S1"] == gfp.cf(vp.to_u_start / vp.ramp_ticks)
        assert pm["VC_TO_SEARCH_F"] == gfp.cf(vp.to_search_factor) and pm["VC_TO_SEARCH_VZ"] == gfp.cf(vp.to_search_vz_mps)
        assert pm["VC_KT_ALPHA_TO"] == gfp.cf(vp.kt_alpha_to)
        assert pm["VC_TO_AIRBORNE_MPS2"] == gfp.cf(vp.g * (1.0 + vp.to_airborne_fz))
        assert pm["VC_REF_APPROACH_SHIFT"] == str(vp.ref_approach_shift)
        assert pm["VC_REF_APPROACH_MIN_UM"] == f"{vp.approach_min_um}L"
        assert pm["VC_LAND_REF_REBASE"] == ("1" if vp.land_ref_rebase else "0")
        assert pm["VC_TD_REL_STEP"] == gfp.cf(vp.td_release_per_s * vp.dt) and pm["VC_TD_REL_MAX"] == gfp.cf(vp.td_release_max)
        assert vp.cap_at(vp.ramp_total_ticks - 1) == vp.u_max and vp.cap_at(vp.ramp_total_ticks - 2) < vp.u_max
        thr = float(pm["VC_TD_REL_OK"].rstrip("f"))
        step, mx = vp.td_release_per_s * vp.dt, vp.td_release_max
        assert -mx < thr < -mx + step                       # between -max and the next lattice value
        assert pm["VC_KT_MIN"] == gfp.cf(vp.kt_min)


def test_v09_power_macros_are_the_reference_values(header):
    """PWR_* per profile = sim.onboard.SupervisorParams pw_* (the power monitor's Q4 codes and ticks); ONB_W_* =
    tether_limits(tethered=True); the v_aux integer form equals power_link.quant_v for every Q4 code."""
    from sim import power_link as pl
    from sim.onboard import SupervisorParams, tether_limits, vtol_profiles
    for nm in vtol_profiles(BASE):
        pm = profile_macros(header, nm)
        sp = SupervisorParams.from_params(BASE, fc_config_path=gfp.FC_CONFIG, mode=lp.MODE_VTOL, profile=nm)
        assert pm["PWR_RULES"] == ("1" if sp.power_rules else "0")
        if not sp.power_rules:
            continue
        for k in ("pw_arm_lo_q", "pw_arm_hi_q", "pw_lost_q", "pw_sag_q", "pw_sag_clear_q", "pw_buf_low_q", "pw_buf_res_q",
                  "pw_d2_q", "pw_i8buf_q", "pw_lost_ticks", "pw_sag_warn_ticks", "pw_sag_land_ticks", "pw_buf_ticks",
                  "pw_buf_res_ticks", "pw_d2_ticks", "pw_sag_release_ticks"):
            assert pm[f"PWR_{k[3:].upper()}"] == f"{getattr(sp, k)}U", (nm, k)
        lim = tether_limits(sp, tethered=True)
        assert pm["ONB_W_HOVER_DM"] == str(lim.hover_dm) and pm["ONB_W_ALT_MAX_DM"] == str(lim.alt_max_dm)
        assert pm["ONB_W_ENDURANCE_TICKS"] == f"{lim.endurance_ticks}UL"
        mul, add, sh = (int(pm[k].rstrip("UL")) for k in ("PWR_VAUX_MUL", "PWR_VAUX_ADD", "PWR_VAUX_SHIFT"))
        for y in range(0, 16 * 1023 + 1):
            assert min((y * mul + add) >> sh, 255) == pl.quant_v(y * sp.pw_v_per_count / 16.0), y
    m = macros(header)
    assert m["PWR_V_TETHER_ADC_CHANNEL"] in ("1", "2", "3") and m["PWR_V_BUFFER_ADC_CHANNEL"] in ("1", "2", "3")
    assert m["PWR_CAL_IN_GAIN_Q15"] == "32768UL" and m["PWR_CAL_IN_OFFSET"] == "0"


def test_v09_refuses_options_not_ported(tmp_path):
    """mission.vtol.takeoff_v09 u_slew_per_s / cap_margin > 0 are reference options the C core does not have."""
    refused(tmp_path, lambda p: p["mission"]["vtol"]["takeoff_v09"].__setitem__("u_slew_per_s", 0.5),
            needles=("u_slew_per_s",))
    refused(tmp_path, lambda p: p["mission"]["vtol"]["takeoff_v09"].__setitem__("cap_margin", 0.1),
            needles=("cap_margin",))


# --------------------------------------------------------------------------- #
# fw09 G1 (SPEC_firmware_v09 3.2): per-configuration vehicle constants
# --------------------------------------------------------------------------- #
VCFG_N = {"battery": 0, "tether": 1, "hybrid": 2}
_CPP_N = [0]


def _cpp(text: str, tmp_path, *defs: str) -> tuple[int, dict[str, str], str]:
    """The C preprocessor's macro table of a header text (zig cc -E -dM, as the host tests compile; -D__AVR__ in defs
    stands for an AVR image): (exit code, {name: value}, stderr)."""
    pytest.importorskip("ziglang")
    import subprocess
    _CPP_N[0] += 1
    path = tmp_path / f"params_gen_{_CPP_N[0]}.h"
    path.write_text(text, encoding="utf-8")
    r = subprocess.run([sys.executable, "-m", "ziglang", "cc", "-E", "-dM", "-x", "c", "-U__AVR__", *defs, str(path)],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    out = {}
    for ln in r.stdout.splitlines():
        m = re.match(r"#define (\w+)(?:\s+(.*))?$", ln)
        if m:
            out[m.group(1)] = (m.group(2) or "").strip()
    return r.returncode, out, r.stderr


def _same_fields(got, want, tol: float = 1e-12) -> None:
    """Every dataclass field of two parameter objects equal (floats within tol, before float32 rounding)."""
    for name in want.__dataclass_fields__:
        a, b = getattr(got, name), getattr(want, name)
        if isinstance(b, float) and not isinstance(b, bool):
            assert abs(a - b) <= tol, (name, a, b)
        else:
            assert a == b, (name, a, b)


@pytest.mark.parametrize("cfg", gfp.VCFGS)
def test_g1_vehicle_constants_are_the_reference_vehicle(base, cfg):
    """Each VTOL configuration's constants = sim.vehicle.load_vehicle(config='vtol', power=cfg) (mass, CG, inertia, lever
    arm, hover throttle) and the parameter objects sim/sim6dof.py builds from that vehicle (VerticalParams, PositionParams,
    VtolAltitudeFilter with the vertical controller's u_hover) - every field to 1e-12 before float32 rounding."""
    from sim import vehicle as vm
    from sim.position_ctrl import PositionParams
    from sim.sim6dof import SensorConfig
    from sim.state_machine import VtolAltitudeFilter
    from sim.vertical_ctrl import VerticalParams
    veh = vm.load_vehicle(gfp.ROOT, config="vtol", power=cfg)
    dc = base["vcfg"][cfg]
    assert veh.hardware_config == "vtol" and veh.power_source == cfg and dc["vcfg_name"] == cfg
    assert os.path.samefile(dc["cad_path"], veh.source_path)
    assert os.path.basename(dc["cad_path"]) == f"cadquery_report_vtol{'' if cfg == 'battery' else '_' + cfg}.json"
    for key, ref in (("m_full", veh.full.mass_kg), ("l_v", veh.l_v_m), ("z_cg", veh.z_cg_m),
                     ("m_post", veh.post_ejection.mass_kg), ("hover_u", veh.hover_throttle), ("a_e", veh.a_e_m2),
                     ("s_v", veh.s_v_m2), ("r_v", veh.r_v_m), ("t_max", veh.t_max_n)):
        assert abs(dc[key] - ref) <= 1e-12, (cfg, key, dc[key], ref)
    for i in range(3):
        for j in range(3):
            assert abs(dc["i_full"][i][j] - float(veh.full.inertia_kg_m2[i, j])) <= 1e-12, (cfg, i, j)
    # the configuration really differs from the rocket (the G1 defect: the rocket's 351 mm / 0.916 in every VTOL image)
    assert abs(dc["l_v"] - base["l_v"]) > 0.1 and abs(dc["hover_u"] - base["hover_u"]) > 0.15
    gi, sens = dc["vt"]["gains"], SensorConfig()
    for nm, pr in dc["vt"]["profiles"].items():
        vp = VerticalParams.from_params(veh.params, veh.full.mass_kg, veh.t_max_n, profile=nm, **gi["gains"])
        _same_fields(pr["vp"], vp)
        for attr in ("u_hover", "inv_u_hover", "to_search_factor", "hover_margin"):
            assert abs(getattr(pr["vp"], attr) - getattr(vp, attr)) <= 1e-12, (cfg, nm, attr)
        assert pr["pp"] is not None, (cfg, nm)          # the field / demo profiles fly the position hold
        pp = PositionParams.from_params(veh.params, veh.full.mass_kg, veh.t_max_n, g=gfp.G0, profile=nm, **gi["pos"])
        _same_fields(pr["pp"], pp)
        f = VtolAltitudeFilter.from_params(veh.params, u_hover=vp.u_hover, crossover_hz=sens.altitude_crossover_hz,
                                           zeta=sens.altitude_zeta, baro_latency_s=sens.baro_latency_s,
                                           bias_tau_s=sens.vtol_alt_bias_tau_s, pad_bias_tau_s=sens.pad_avg_tau_s)
        a = pr["alt"]
        for k in ("k1", "k2", "k3"):
            assert abs(a[k] - getattr(f, k)) <= 1e-12, (cfg, nm, k)
        assert a["comp_m"] == f.comp_m and a["bias_limit"] == f.bias_limit and abs(a["pad_tau_s"] - f.pad_bias_tau_s) <= 1e-12
        # the NAV filter's thrust compensation runs on VC_INV_U_HOVER (core/fc_config.c): the filter's own 1 / u_hover
        assert abs(pr["vp"].inv_u_hover - f.inv_u_hover) <= 1e-12 and f.throttle_exp == float(veh.params["edf"]["throttle_exponent"])


@pytest.mark.parametrize("cfg", gfp.VCFGS)
def test_g1_header_macros_of_each_configuration(base, header, cfg, tmp_path):
    """The header as an AVR build of the configuration sees it (the generated #if FC_VEHICLE blocks resolved, and the C
    preprocessor with -D FC_VCFG) carries that configuration's numbers rounded once to float32."""
    dc = base["vcfg"][cfg]
    view = macros(vehicle_view(header, cfg))
    expect = {"MIX_L_V_M": gfp.cf(dc["l_v"]), "FC_HOVER_THROTTLE": gfp.cf(dc["hover_u"]), "VEH_MASS_KG": gfp.cf(dc["m_full"]),
              "VEH_Z_CG_M": gfp.cf(dc["z_cg"]), "VEH_IXX_KG_M2": gfp.cf(dc["i_full"][0][0]),
              "VEH_IYY_KG_M2": gfp.cf(dc["i_full"][1][1]), "VEH_IZZ_KG_M2": gfp.cf(dc["i_full"][2][2]),
              "VEH_POST_EJECTION_MASS_KG": gfp.cf(dc["m_post"]), "FC_PARAMS_TAG": f"0x{dc['params_tag']:02X}u",
              "FC_VEHICLE_NAME": f'"vtol_{cfg}"'}
    for k, v in expect.items():
        assert view[k] == v, (cfg, k)
    for nm, pr in dc["vt"]["profiles"].items():
        pm = profile_macros(header, nm, cfg)
        assert pm["VC_U_HOVER"] == gfp.cf(pr["vp"].u_hover) == view["FC_HOVER_THROTTLE"], (cfg, nm)
        assert pm["VC_INV_U_HOVER"] == gfp.cf(pr["vp"].inv_u_hover), (cfg, nm)
        assert pm["VC_TO_SEARCH_F"] == gfp.cf(pr["vp"].to_search_factor), (cfg, nm)
        assert pm["POS_TW"] == gfp.cf(pr["pp"].tw), (cfg, nm)
        for k in ("k1", "k2", "k3"):
            assert pm[f"ALTV_{k.upper()}"] == gfp.cf(pr["alt"][k]), (cfg, nm, k)
    default = dc["vt"]["default"]
    rc, cm, err = _cpp(header, tmp_path, f"-DFC_VCFG={VCFG_N[cfg]}", "-D__AVR__", "-DFC_ROLE_NAV")
    assert rc == 0, err
    want = {**view, **profile_macros(header, default, cfg)}
    for k in gfp.VCFG_MACROS:
        assert cm[k] == want[k], (cfg, k, cm[k], want[k])
    assert cm["FC_VEHICLE"] == "FC_VCFG" and cm["FC_VCFG"] == str(VCFG_N[cfg])


def test_g1_rocket_set_is_the_rocket_vehicle_and_flies_every_ballistic_build(base, header, tmp_path):
    """The top-level (rocket) set = sim.vehicle.load_vehicle() of the rocket (cadquery_report.json, the v0.8.1 numbers);
    the merged header's AVR ballistic view and its host view without FC_VCFG define exactly the macros a rocket-only
    render does (the merge changes nothing in the ballistic images' constants)."""
    from sim import vehicle as vm
    veh = vm.load_vehicle(gfp.ROOT)
    assert veh.hardware_config == "rocket" and os.path.samefile(base["cad_path"], veh.source_path)
    for key, ref in (("m_full", veh.full.mass_kg), ("l_v", veh.l_v_m), ("z_cg", veh.z_cg_m), ("hover_u", veh.hover_throttle),
                     ("m_post", veh.post_ejection.mass_kg)):
        assert abs(base[key] - ref) <= 1e-12, key
    rocket_only = gfp.render(dict(base, vcfg={}))
    assert "FC_VEHICLE == FC_VCFG_BATTERY" not in rocket_only
    for defs in (("-D__AVR__", "-DFC_MODE_BALLISTIC", "-DFC_ROLE_FCU"), ("-D__AVR__", "-DFC_MODE_BALLISTIC", "-DFC_ROLE_NAV"),
                 ("-D__AVR__", "-DFC_MODE_BALLISTIC", "-DFC_ROLE_NAV", "-DFC_VCFG=1"), ()):
        rc, merged, err = _cpp(header, tmp_path, *defs)
        rc2, alone, err2 = _cpp(rocket_only, tmp_path, *defs)
        assert rc == 0 and rc2 == 0, err + err2
        assert merged == alone, (defs, {k: (merged.get(k), alone.get(k)) for k in set(merged) | set(alone)
                                        if merged.get(k) != alone.get(k)})
        assert merged["FC_VEHICLE"] == "FC_VCFG_ROCKET" and merged["MIX_L_V_M"] == gfp.cf(veh.l_v_m)
        assert merged["FC_PARAMS_TAG"] == f"0x{base['params_tag']:02X}u"


def test_g1_configurations_differ_only_in_the_vehicle_macros(header, tmp_path):
    """Between the four views of the header only VCFG_MACROS differ (the selector FC_VEHICLE / FC_VCFG apart); every
    vehicle macro differs between the rocket and each VTOL configuration, and the VTOL configurations from each other."""
    rc, rocket, err = _cpp(header, tmp_path)
    assert rc == 0, err
    views = {}
    for cfg, n in VCFG_N.items():
        rc, v, err = _cpp(header, tmp_path, f"-DFC_VCFG={n}")
        assert rc == 0, err
        diff = {k for k in set(v) | set(rocket) if v.get(k) != rocket.get(k)} - {"FC_VEHICLE", "FC_VCFG"}
        assert diff == set(gfp.VCFG_MACROS), (cfg, sorted(diff ^ set(gfp.VCFG_MACROS)))
        views[cfg] = v
    for k in gfp.VCFG_MACROS:
        assert len({views[c][k] for c in VCFG_N}) == 3, k


def test_g1_render_refuses_a_difference_outside_the_vehicle_macros(base):
    """A configuration that would change any other macro (here the vane force centroid) is not written."""
    bad = dict(base, vcfg=dict(base["vcfg"]))
    bad["vcfg"]["tether"] = dict(base["vcfg"]["tether"], r_v=base["r_v"] * 1.01)
    with pytest.raises(gfp.GenError) as ei:
        gfp.render(bad)
    assert "MIX_R_V_M" in str(ei.value) and "VCFG_MACROS" in str(ei.value)


@pytest.mark.parametrize("defs, ok, needle", [
    (("-D__AVR__", "-DFC_ROLE_NAV"), False, "names its vehicle configuration"),
    (("-D__AVR__", "-DFC_ROLE_GNS"), False, "names its vehicle configuration"),
    (("-D__AVR__", "-DFC_ROLE_COM"), False, "names its vehicle configuration"),
    (("-D__AVR__", "-DFC_ROLE_FCU", "-DFC_VCFG=3"), False, "must not fly the rocket"),
    (("-DFC_VCFG=4",), False, "FC_VCFG must be 0"),
    (("-D__AVR__", "-DFC_ROLE_FCU", "-DFC_VCFG=1"), False, "PWR_SRC must equal FC_VCFG"),
    (("-D__AVR__", "-DFC_ROLE_FCU", "-DFC_VCFG=0", "-DPWR_SRC=1"), False, "PWR_SRC must equal FC_VCFG"),
    (("-D__AVR__", "-DFC_ROLE_FCU", "-DFC_VCFG=2", "-DPWR_SRC=1"), False, "PWR_SRC must equal FC_VCFG"),
    (("-D__AVR__", "-DFC_ROLE_FCU", "-DFC_VCFG=2", "-DPWR_SRC=2"), True, "FC_VCFG_HYBRID"),
    (("-D__AVR__", "-DFC_ROLE_FCU", "-DFC_VCFG=0", "-DPWR_SRC=0"), True, "FC_VCFG_BATTERY"),   # power_mon.h's default
    (("-D__AVR__", "-DFC_ROLE_FCU", "-DFC_VCFG=0"), True, "FC_VCFG_BATTERY"),
    (("-D__AVR__", "-DFC_ROLE_NAV", "-DFC_MODE_BALLISTIC"), True, "FC_VCFG_ROCKET"),
    (("-D__AVR__", "-DFC_ROLE_NAV", "-DFC_MODE_BALLISTIC", "-DFC_VCFG=2"), True, "FC_VCFG_ROCKET"),   # ignored
    (("-DFC_VCFG=3",), True, "FC_VCFG_ROCKET"),                          # host builds may ask for the rocket
    ((), True, "FC_VCFG_ROCKET"),                                        # host builds without FC_VCFG
])
def test_g1_build_refusals(header, tmp_path, defs, ok, needle):
    """A VTOL AVR image must name its configuration (and not the rocket's), an FCU's PWR_SRC must equal FC_VCFG; the
    ballistic builds fly the rocket whatever FC_VCFG says; host builds default to the rocket set."""
    rc, m, err = _cpp(header, tmp_path, *defs)
    if ok:
        assert rc == 0, err
        want = {"FC_VCFG_ROCKET": "3", "FC_VCFG_BATTERY": "0", "FC_VCFG_TETHER": "1", "FC_VCFG_HYBRID": "2"}[needle]
        assert m["FC_VEHICLE"] in (needle, "FC_VCFG") and (m["FC_VEHICLE"] == needle or m["FC_VCFG"] == want)
    else:
        assert rc != 0 and needle in err, err


def _vtol_root(tmp_path, reports: dict[str, str | None], mutate=None) -> str:
    """A CAD export folder for derive(vtol_root=): {configuration: the project's report to copy under its name | None}."""
    root = tmp_path / "vtol_root"
    ex = root / "cad" / "exports"
    ex.mkdir(parents=True, exist_ok=True)
    for cfg, src in reports.items():
        if src is None:
            continue
        with open(gfp.vtol_report_path(src), "r", encoding="utf-8") as f:
            rep = json.load(f)
        if mutate and cfg in mutate:
            mutate[cfg](rep)
        (ex / os.path.basename(gfp.vtol_report_path(cfg))).write_text(json.dumps(rep), encoding="utf-8")
    return str(root)


def test_g1_configuration_without_its_cad_report_does_not_build(tmp_path):
    """sim.vehicle.load_vtol_vehicle's refusals: no report, or a report of another power source under the configuration's
    name -> that configuration is unavailable, its builds stop at an #error; the others and the rocket build."""
    root = _vtol_root(tmp_path, {"battery": "battery", "tether": None, "hybrid": "tether"})
    d = gfp.derive(vtol_root=root)
    assert set(d["vcfg"]) == {"battery"} and set(d["vcfg_errors"]) == {"tether", "hybrid"}
    assert "no usable CAD report" in d["vcfg_errors"]["tether"] and "none found" in d["vcfg_errors"]["tether"]
    assert "'tether'" in d["vcfg_errors"]["hybrid"] and "ignored" in d["vcfg_errors"]["hybrid"]
    assert any(w.startswith("VTOL hybrid:") and "will not compile" in w for w in d["warnings"])
    text = gfp.render(d)
    assert '#error "no usable CAD report for the VTOL tether configuration' in text
    assert '#error "no usable CAD report for the VTOL hybrid configuration' in text
    assert "cadquery_report_vtol_tether.json (canonical) absent" in text
    for n, ok in ((0, True), (1, False), (2, False)):
        rc, m, err = _cpp(text, tmp_path, f"-DFC_VCFG={n}", "-D__AVR__", "-DFC_ROLE_NAV")
        assert (rc == 0) == ok, (n, err)
        if ok:
            assert m["MIX_L_V_M"] == gfp.cf(d["vcfg"]["battery"]["l_v"])
        else:
            assert "no usable CAD report" in err
    rc, m, err = _cpp(text, tmp_path, "-D__AVR__", "-DFC_ROLE_NAV", "-DFC_MODE_BALLISTIC")
    assert rc == 0 and m["MIX_L_V_M"] == gfp.cf(d["l_v"]), err
    # a report with a failed component is refused like in the reference
    root2 = _vtol_root(tmp_path / "b", {"battery": "battery", "tether": "tether", "hybrid": "hybrid"},
                       mutate={"tether": lambda r: r.__setitem__("failed_components", ["top_cap"])})
    d2 = gfp.derive(vtol_root=root2)
    assert set(d2["vcfg"]) == {"battery", "hybrid"} and "failed components" in d2["vcfg_errors"]["tether"]
    # params.json without configs.vtol: no VTOL configuration at all
    d3 = derive_with(tmp_path, lambda p: p.pop("configs"))
    assert not d3["vcfg"] and set(d3["vcfg_errors"]) == set(gfp.VCFGS) and "configs" in d3["vcfg_errors"]["battery"]


def test_g1_params_tag_names_the_configuration(base, tmp_path):
    """FC_PARAMS_TAG: the rocket keeps the low byte of the params sha256; each VTOL configuration the low byte of
    sha256(params sha | vtol_<name> | its CAD report's canonical sha | k), k the first value not taken by an earlier
    set - four distinct tags; a change of one VTOL vehicle changes its tag."""
    psha = gfp.sha256(gfp.PARAMS)
    tags = [base["params_tag"]]
    for cfg in gfp.VCFGS:
        dc = base["vcfg"][cfg]
        assert dc["cad_sha"] == gfp.cad_report_sha256(dc["cad_path"])

        def t(k, _dc=dc, _cfg=cfg):
            return int(hashlib.sha256(f"{psha}|vtol_{_cfg}|{_dc['cad_sha']}|{k}".encode()).hexdigest()[:16], 16) & 0xFF
        assert dc["params_tag"] == t(dc["params_tag_k"]), cfg
        assert all(t(k) in tags for k in range(dc["params_tag_k"])), cfg
        tags.append(dc["params_tag"])
    assert len(set(tags)) == 4 and base["params_tag"] == int(psha[14:16], 16)
    root = _vtol_root(tmp_path, {c: c for c in gfp.VCFGS},
                      mutate={"hybrid": lambda r: r["parts"]["buffer_pack"].__setitem__(
                          "mass_g", r["parts"]["buffer_pack"]["mass_g"] + 1.0)})
    d = gfp.derive(vtol_root=root)
    assert d["vcfg"]["hybrid"]["cad_sha"] != base["vcfg"]["hybrid"]["cad_sha"]
    assert d["vcfg"]["hybrid"]["params_tag"] != base["vcfg"]["hybrid"]["params_tag"]
    assert d["vcfg"]["hybrid"]["m_full"] == pytest.approx(base["vcfg"]["hybrid"]["m_full"] + 1e-3, abs=1e-12)
    for cfg in ("battery", "tether"):
        assert d["vcfg"][cfg]["params_tag"] == base["vcfg"][cfg]["params_tag"], cfg


def test_g1_platformio_every_vtol_env_names_its_configuration():
    """Every env is either a ballistic build (no FC_VCFG) or names one configuration: *_wired_buffer 2, *_wired 1, the
    others 0; PWR_SRC == FC_VCFG wherever PWR_SRC is set and on every FCU power build; the wired flight images and their
    co-simulation twins exist for every chip and the flight images are default envs."""
    ini = _ini()
    envs = re.findall(r"^\[env:(\w+)\]\n(.*?)(?=^\[|\Z)", ini, re.M | re.S)
    for env, body in envs:
        ballistic = "FC_MODE_BALLISTIC" in body or "ballistic_flags" in body
        v = re.findall(r"-D\s*FC_VCFG=(\d)", body)
        p = re.findall(r"-D\s*PWR_SRC=(\d)", body)
        if ballistic:
            assert not v and not p, env
            continue
        want = "2" if env.endswith("_wired_buffer") else ("1" if env.endswith("_wired") else "0")
        assert v == [want], (env, v)
        if p or (env.startswith("fcu") and want != "0"):
            assert p == v, (env, p, v)
    names = {e for e, _ in envs}
    default = [e.strip() for e in re.search(r"^default_envs\s*=\s*(.*)$", ini, re.M).group(1).split(",")]
    for role in ("nav", "gns", "fcu", "com"):
        for sfx in ("", "_wired", "_wired_buffer"):
            assert f"{role}{sfx}" in names and f"{role}{sfx}" in default, (role, sfx)
            assert f"{role}_cosim{sfx}" in names, (role, sfx)
    assert set(default) <= names


# ---- fw09 W1: the nlink exchange watchdog constant ----
def test_w1_nlink_watchdog_macros(base, header):
    m = macros(header)
    xfer = int(m["NLINK_XFER_US"].rstrip("U"))
    push = gfp.nav_push_min_us_vtol()
    assert m["NLINK_WD_US"] == "1000UL" and m["NLINK_WD_COUNTS"] == "16000u"
    assert base["nl_wd"] == dict(us=1000, counts=16000, xfer_us=xfer, push_us=push, limit_us=base["tick_us"] - push)
    assert 1000 > 1.5 * xfer and 1000 < base["tick_us"] - push
    assert BASE["flight_computer"]["nlink"]["exchange_watchdog_us"] == 1000


@pytest.mark.parametrize("val, needles", [
    (945, ("must exceed 1.5 x the exchange NLINK_XFER_US 630",)),
    (3110, ("must stay below FC_TICK_US 4000 - NAV_PUSH_MIN_US_VTOL",)),
    (1000.5, ("whole number of us",)),
    (0, ("whole number of us > 0",)),
    (5000, ("Timer3 counts",)),
    ("1000", ("is not a number",)),
    (True, ("is not a number",)),
])
def test_w1_nlink_watchdog_refusals(tmp_path, val, needles):
    refused(tmp_path, fc_set("nlink.exchange_watchdog_us", val), needles=needles)


def test_w1_nlink_watchdog_limits_accepted_and_key_required(tmp_path):
    for val, counts in ((946, 15136), (3109, 49744)):
        d = derive_with(tmp_path, fc_set("nlink.exchange_watchdog_us", val))
        assert d["nl_wd"]["counts"] == counts and macros(gfp.render(d))["NLINK_WD_COUNTS"] == f"{counts}u"
    refused(tmp_path, lambda p: p["flight_computer"]["nlink"].pop("exchange_watchdog_us"),
            needles=("exchange_watchdog_us missing",))


# ---- fw09 SPEC 5: the power pins from params flight_computer.pins ----
def test_v09_power_pins_come_from_params(base, header):
    m = macros(header)
    assert (m["PWR_V_TETHER_ADC_CHANNEL"], m["PWR_V_BUFFER_ADC_CHANNEL"], m["PIN_BUF_ISIGN_PORT"],
            m["PIN_BUF_ISIGN_BIT"]) == ("1", "2", "C", "3")
    assert (base["fcu_pins"]["v_tether_adc"], base["fcu_pins"]["v_buffer_adc"], base["fcu_pins"]["buf_isign"]) == \
        (("F", 1), ("F", 2), ("C", 3))
    assert not hasattr(gfp, "PWR_PINS_ASSUMED")
    for key in ("v_tether_adc", "v_buffer_adc", "buf_isign"):
        line = next(ln for ln in header.splitlines() if f"flight_computer.pins.{key} = " in ln)
        assert "ASSUMED until the v0.9 carrier PCB is routed" in line and BASE["flight_computer"]["pins"][key] in line


@pytest.mark.parametrize("mut, needles", [
    (lambda p: p["flight_computer"]["pins"].pop("v_tether_adc"), ("v_tether_adc missing",)),
    (lambda p: p["flight_computer"]["pins"].pop("buf_isign"), ("buf_isign missing",)),
    (fcu("v_tether_adc", "PF0/ADC0"), ("PF0 on the FCU",)),
    (fcu("buf_isign", "PC0"), ("PC0 on the FCU",)),
    (fcu("v_buffer_adc", "PF5/ADC5"), ("PF0..PF3",)),
    (fcu("v_buffer_adc", "PC4"), ("PF0..PF3",)),
    (fcu("v_buffer_adc", "PF2/ADC3"), ("not an alternate function",)),
])
def test_v09_power_pin_refusals(tmp_path, mut, needles):
    refused(tmp_path, mut, needles=needles)


# ---- the three generated headers; the bridge header follows sim/link_proto.py (RP1) ----
def test_all_three_generated_headers_are_current(capsys):
    import importlib.util
    import gen_gains_header as ggh
    spec = importlib.util.spec_from_file_location(
        "gen_bridge_config", os.path.join(ROOT, "firmware", "ground_bridge", "gen_bridge_config.py"))
    gbc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gbc)
    assert gfp.main(["--check"]) == 0, capsys.readouterr().out
    assert ggh.main(["--check"]) == 0, capsys.readouterr().out
    assert gbc.main(["--check"]) == 0, capsys.readouterr().out


def test_bridge_config_follows_link_proto_after_rp1(monkeypatch):
    """LINK_LEN_MAX / LINK_MAX_FRAME_BYTES come from sim/link_proto.py; once it defines TELEM v4 / v5 (fw09 RP1: 41 / 47 B,
    LEN_MAX 42) the header names the longest frame and lists their time on air."""
    import importlib.util
    import layout
    spec = importlib.util.spec_from_file_location(
        "gen_bridge_config_rp1", os.path.join(ROOT, "firmware", "ground_bridge", "gen_bridge_config.py"))
    gbc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gbc)
    p = layout.load_params(gfp.PARAMS)
    now = gbc.render(p)
    assert f"#define LINK_LEN_MAX              {lp.LEN_MAX}\n" in now
    if getattr(lp, "TELEM4_FRAME_BYTES", None) is None:          # before RP1: no TELEM v4 / v5 in the header
        assert "TELEM4" not in now
    else:                                                        # fw09 integration: RP1 is in sim/link_proto.py, so the
        assert (lp.LEN_MAX, lp.TELEM4_FRAME_BYTES, lp.TELEM5_FRAME_BYTES) == (42, 41, 47)   # header is the RP1 one now
        assert "#define LINK_MAX_FRAME_BYTES      47 " in now and "TELEM4 41 B = " in now and "TELEM5 47 B = " in now
    monkeypatch.setattr(gbc.lp, "LEN_MAX", 42)
    monkeypatch.setattr(gbc.lp, "TELEM4_FRAME_BYTES", 41, raising=False)
    monkeypatch.setattr(gbc.lp, "TELEM5_FRAME_BYTES", 47, raising=False)
    rp1 = gbc.render(p)
    assert "#define LINK_LEN_MAX              42\n" in rp1 and "#define LINK_MAX_FRAME_BYTES      47 " in rp1
    assert "v0.9: the TELEM5 frame of the power builds" in rp1 and "TELEM4 41 B = " in rp1 and "TELEM5 47 B = " in rp1


# --------------------------------------------------------------------------- #
# 2026-11-06: the ESP8266 bridge backend (link.vehicle_radio, link.esp_bridge) and the logic-battery pin
# --------------------------------------------------------------------------- #
def test_esp_bridge_backend_macros(base, header):
    lk = BASE["flight_computer"]["link"]
    assert lk["vehicle_radio"] == "esp_bridge" and lk["radio"] == "lora_uart"     # backend vs the sim's link model
    m = macros(header)
    assert m["FC_RADIO_LORA_UART"] == "1" and m["FC_RADIO_SX127X"] == "2" and m["FC_RADIO_ESP_BRIDGE"] == "3"
    assert m["FC_LINK_RADIO"] == "FC_RADIO_ESP_BRIDGE"
    assert m["ESP_UART_BAUD"] == "38400UL" and m["ESP_UART_UBRR"] == "25" and m["ESP_UART_U2X"] == "0"
    assert abs(F_CPU / (16 * 26) / 38400 - 1) <= lk["esp_bridge"]["uart_max_error"]
    assert m["ESP_RESET_HOLD_MS"] == "2U" and m["ESP_BOOT_TICKS"] == "100UL"           # 400 ms at 250 Hz
    # the LoRa backend macros stay (fallback).  2026-11-10: the link MODEL follows the vehicle radio (sim/link_model.py
    # model_radio), the LoRa one stays in the header as the fallback budget and sizes LINK_RTT_WORST_MS
    assert m["LINK_UART_BAUD"] == "9600UL" and "esp_bridge (selected)" in header and "lora_uart (selected)" not in header
    assert base["vradio"] == "esp_bridge" and base["radio"] == "esp_bridge" and base["fallback"] == "lora_uart"
    assert set(base["budgets"]) >= {"esp_bridge", "lora_uart", "sx127x"}
    fb, sel = base["budgets"]["lora_uart"], base["budgets"]["esp_bridge"]
    assert sel["rtt_worst_s"] < fb["rtt_worst_s"]
    assert int(m["LINK_RTT_WORST_MS"].rstrip("UL")) == round(fb["rtt_worst_s"] * 1000)


def test_vehicle_radio_defaults_to_the_link_model(tmp_path):
    d = derive_with(tmp_path, lambda p: p["flight_computer"]["link"].pop("vehicle_radio"))
    m = macros(gfp.render(d))
    assert d["vradio"] == "lora_uart" and m["FC_LINK_RADIO"] == "FC_RADIO_LORA_UART"
    assert m["ESP_UART_UBRR"] == "25"                                    # the esp_bridge block still generates its macros
    d = derive_with(tmp_path, fc_set("link.vehicle_radio", "sx127x"))
    assert macros(gfp.render(d))["FC_LINK_RADIO"] == "FC_RADIO_SX127X"


@pytest.mark.parametrize("mut, needles", [
    (fc_set("link.vehicle_radio", "wifi"), ("vehicle_radio 'wifi'",)),
    (fc_set("link.radio", "esp_bridge"), ("link.radio 'esp_bridge'",)),          # the link MODEL has no Wi-Fi
    (fc_set("link.esp_bridge.uart_baud", 57600), ("uart_baud 57600", "exceed uart_max_error")),
    (fc_set("link.esp_bridge.uart_baud", 115200), ("uart_baud 115200",)),        # +8.5 % with U2X 0
    (fc_set("link.esp_bridge.uart_max_error", 0.05), ("uart_max_error",)),
    (fc_set("link.esp_bridge.reset_hold_ms", 0), ("reset_hold_ms",)),
    (fc_set("link.esp_bridge.reset_hold_ms", 2.5), ("reset_hold_ms",)),
    (fc_set("link.esp_bridge.boot_discard_ms", 0), ("boot_discard_ms",)),
    (lambda p: p["flight_computer"]["link"].pop("esp_bridge"), ("needs flight_computer.link.esp_bridge",)),
])
def test_esp_bridge_refusals(tmp_path, mut, needles):
    refused(tmp_path, mut, needles=needles)


def test_logic_battery_pin_and_scale(base, header):
    m = macros(header)
    assert base["fcu_pins"]["logic_bat_adc"] == ("F", 3) and m["LOGIC_BAT_ADC_CHANNEL"] == "3"
    assert float(m["LOGIC_BAT_V_PER_COUNT"].rstrip("f")) == pytest.approx(2.56 / 1024 * 2, rel=1e-7)
    # 2026-11-10: the VTOL battery FCU sends it (TELEM v6 / v7): one PF3 conversion per second and the integer scale of
    # the lbat byte (0.02 V steps) from the same divider
    assert BASE["flight_computer"]["logic_battery_monitor"]["telemetry_builds"] == "vtol_battery"
    assert m["LOGIC_BAT_TELEM"] == "1" and m["LOGIC_BAT_DIV_TICKS"] == f"{int(base['loop_hz'])}U" == "250U"
    q = int(m["LOGIC_BAT_Q_X1024"].rstrip("UL"))
    assert q == round(2.56 / 1024 * 2 * lp.LBAT_SCALE * 1024) == 256
    for counts in (0, 1, 600, 772, 840, 1023):                           # the header's integers == the sim's reader
        assert lp.lbat_q_from_counts(counts, q) == min(255, max(1, (counts * q + 512) >> 10))
    assert lp.lbat_q_from_counts(772, q) == 193                          # 3.86 V on the 47k / 47k divider
    # which firmware source reads PF3: only the FCU's main.c, and only inside an FC_LOGIC_BAT block (hal/adc.h: VTOL
    # battery build without a power monitor) - the rocket FCU has no divider.  The float scale stays unused (no float
    # in the tick: the integer LOGIC_BAT_Q_X1024 is the one in the code)
    src = os.path.join(ROOT, "firmware", "atmega128_fc", "src")
    users = {}
    for root, _, files in os.walk(src):
        for fn in files:
            if fn.endswith((".c", ".h")) and fn != "params_gen.h":
                with open(os.path.join(root, fn), encoding="utf-8", errors="replace") as f:
                    text = f.read()
                assert "LOGIC_BAT_V_PER_COUNT" not in text, fn
                code = re.sub(r"/\*.*?\*/", lambda mm: "\n" * mm.group(0).count("\n"), text, flags=re.S)
                if "LOGIC_BAT_ADC_CHANNEL" in code:
                    users[os.path.relpath(os.path.join(root, fn), src).replace(os.sep, "/")] = code
    assert sorted(users) == ["main.c"]
    stack, n_uses = [], 0                                                # the open #if chain: the branch text now active
    for line in users["main.c"].splitlines():
        t = line.strip()
        if t.startswith("#if"):
            stack.append(t)
        elif t.startswith(("#elif", "#else")):
            stack[-1] = t
        elif t.startswith("#endif"):
            stack.pop()
        elif "LOGIC_BAT_ADC_CHANNEL" in t:
            n_uses += 1
            assert any(re.match(r"#(if|elif) FC_LOGIC_BAT\b", c) for c in stack), (line, stack)
    assert n_uses >= 2 and not stack
    with open(os.path.join(src, "hal", "adc.h"), encoding="utf-8", errors="replace") as f:
        adc_h = f.read()
    assert "#if FC_VTOL && !PM_ANY && defined(LOGIC_BAT_TELEM) && defined(FC_VCFG) && defined(FC_VCFG_BATTERY)" in adc_h
    assert "#if LOGIC_BAT_TELEM && FC_VCFG == FC_VCFG_BATTERY\n#define FC_LOGIC_BAT 1" in adc_h


def test_logic_battery_telemetry_only_on_the_vtol_battery_build(tmp_path):
    d = derive_with(tmp_path, fc_set("logic_battery_monitor.telemetry_builds", "none"))
    assert macros(gfp.render(d))["LOGIC_BAT_TELEM"] == "0" and d["telem_bytes"] in (39, 45)   # TELEM2 / TELEM3 again
    db = derive_with(tmp_path, _mode("ballistic"))                       # the rocket: TELEM v1, whatever the flag says
    assert db["telem_bytes"] == 36


@pytest.mark.parametrize("mut, needles", [
    (fcu("logic_bat_adc", "PF2/ADC2"), ("PF2 on the FCU",)),                   # = v_buffer_adc
    (fcu("logic_bat_adc", "PF4/ADC4"), ("PF0..PF3",)),                         # a JTAG pin
    (fc_set("logic_battery_monitor.r_bottom_ohm", 470000), ("full 1S cell",)),  # 3.8 V at the pin > 2.56 V
    (fc_set("logic_battery_monitor.vref_v", 5.0), ("2.56 V",)),
])
def test_logic_battery_refusals(tmp_path, mut, needles):
    refused(tmp_path, mut, needles=needles)


def test_platformio_builds_every_com_backend():
    ini = _ini()
    envs = re.findall(r"^\[env:(\w+)\]", ini, re.M)
    for e in ("com_esp_bridge", "com_esp_bridge_ballistic", "com_simbench_esp", "com_simbench_esp_ballistic"):
        assert e in envs, e
    default = [e.strip() for e in re.search(r"^default_envs\s*=\s*(.*)$", ini, re.M).group(1).split(",")]
    assert {"com", "com_lora_uart", "com_sx127x", "com_esp_bridge"} <= set(default)
    assert "+<hal/esp_bridge.c>" in _section(ini, "com_common") and "-<hal/esp_bridge.c>" in _section(ini, "fcu_common")
    for env in ("com_cosim_cl", "com_cosim_cl_wired", "com_cosim_cl_wired_buffer", "com_cosim"):  # the co-simulation keeps
        assert "-D FC_LINK_RADIO=FC_RADIO_LORA_UART" in _section(ini, f"env:{env}"), env      # the LoRa link timing
