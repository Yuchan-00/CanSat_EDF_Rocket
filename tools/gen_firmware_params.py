#!/usr/bin/env python3
"""Generate ``firmware/atmega128_fc/src/params_gen.h`` for the three ATmega128A of the flight computer.

v0.6.3 (``flight_computer.architecture``): one header serves every chip - the NAV (MPU-6050 + barometer - BMP280 by default - on
I2C, attitude estimator, altitude filter; pins ``flight_computer.nav.pins``), the FCU (onboard supervisor,
attitude control, PWM, log; pins ``flight_computer.pins``) and the COM (radio - ESP-12E bridge by default, LoRa fallback - uplink decoding, TELEM
building; pins ``flight_computer.com.pins``).  NAV -> FCU: the nlink SPI (``flight_computer.nlink``, SPEC
10.5; NAV master, FCU slave, both on the ATmega128 SPI pins PB0-PB3 with PB0 = SS); FCU <-> COM: the ilink
UART (``flight_computer.ilink``, SPEC 10.4).  Each chip's pin table is checked on its own.

Sources (read only):

* ``params.json``                          mission interlocks, plant, servo, battery, the FCU and COM
                                           pins, the ilink and the LoRa link (``flight_computer.link``)
* ``cad/layout.py`` (``Layout``)           vane area, hinge height, duct diameter, vane azimuths
* ``cad/exports/cadquery_report.json``     CG (vane lever arm l_v) and inertia, recomposed from the
                                           report's parts without the launch pad exactly as
                                           ``sim/vehicle.py`` does
* ``sim/onboard.py`` ``SupervisorParams``  the onboard supervisor's thresholds and tick counts (SPEC 8.8,
                                           interlocks I1-I10; v0.7 SPEC 8.9 the VTOL rules V1-V7) - the
                                           firmware gets exactly the numbers the reference supervisor uses
* ``sim/vertical_ctrl.py`` ``VerticalParams``  v0.7 VTOL vertical controller (rates, limits, hover throttle from
                                           the CAD mass, take-off ramp, touchdown detector), the gains from
                                           ``sim/out/vtol_gains.json`` when it holds every key, else
                                           ``mission.vtol.gains_ASSUMED`` (``VC_GAINS_*`` flags say which)
* ``sim/link_proto.py``                    frame sizes (SPEC 10.3)
* ``sim/nlink_proto.py``                   the nlink exchange length (SPEC 10.5)
* ``sim/link_model.py``                    link latency budget (airtime, round trip, reply timeout)
* ``sim/state_machine.py``                 ``G0`` (and the AltitudeFilter that alt_filter.c mirrors)
* ``sim/sim6dof.py`` ``SensorConfig``      IMU / filter constants the simulation flies with
* ``firmware/atmega128_fc/fc_config.json`` per-vehicle calibration (servo signs/trims, IMU mounting,
                                           ESC range, release-servo positions, I9 times, radio driver
                                           limits)

Invariant: the header is a pure function of those files - no timestamps, no host paths, stable
ordering - and names every source with its sha256 (the CAD report's of its canonical JSON without the build
timings ``extra.build_time_s`` / ``extra.total_time_s``, which change on every CadQuery build:
``cad_report_sha256``, so ``--check`` does not go STALE on a rebuild of the same vehicle).  Every derived hardware setting (Timer2/Timer0
and PWM dividers, the IMU read lead, UBRR, TWBR, SPI divider, SX127x registers, log/baro dividers,
timer thresholds in control ticks, the ilink UBRR and STAT divider, the nlink SPI divider, byte gap and
frame timeout) is computed here and checked; the generator refuses to write a header the ATmega128 or
the radio cannot realise (a pin that is not on its ATmega128 peripheral, a pin used twice on the same
chip, an ilink that is not on that chip's USART, an nlink not on the SPI pins with PB0 = SS on both
chips, an SPI clock the AVR cannot divide exactly or a slave cannot follow, an SPI mode other than 0, a
byte gap shorter than the FCU slave interrupt needs to load its next byte, an exchange that takes half a
NAV tick, a frame timeout outside one to two ticks, a NAV tick other than the control tick, baud error,
an ilink baud off by more than 1 % with U2X = 0, a STAT rate that is not
COM tick / integer, a COM tick other than the control tick, frequency outside the band named in
``flight_computer.link.frequency_note``, SF6 / unknown bandwidth, power outside the PA range, a link
whose single lost frame would already trip the pad link-loss interlock).
Mission mode (v0.7): ``params.json mission.mode`` becomes ``MISSION_MODE`` (``MISSION_MODE_VTOL`` /
``MISSION_MODE_BALLISTIC``, a build-time choice); the ballistic macros keep their v0.6.3 values in both modes and
the VTOL ones (``ONB_*`` VTOL ticks / limits, ``VC_*``) are written whenever ``mission.vtol`` exists.  The SX127x
TX timeout is sized for the TELEM frame the build sends (TELEM v2, 39 bytes, in the VTOL build).  The vertical
gains live here, not in ``gains.h``: they belong to the mission (``mission.vtol``, tuned by the 6-DOF VTOL runs
into ``sim/out/vtol_gains.json``), next to every other VTOL number, while ``gains.h`` stays the attitude tuner's
``sim.controller.GAIN_KEYS`` file.  v0.7.1b: the thrust learning (``VC_KT_*``, mission.vtol.thrust_learning) and the
VTOL build's attitude-gain overrides (``VTOL_OV_<KEY>``, params.json ``control.vtol_overrides`` through
``sim.controller.vtol_overrides``; core/fc_config.c ``fc_vtol_gains``) - the ballistic build never reads them.
v0.8 (GPS position hold, params mission.vtol.position_hold / flight_computer.nav.gps): per build profile the FCU
position controller (``POS_*``, ``sim.position_ctrl.PositionParams.from_params(profile)``), the take-off check
(``VC_TAKEOFF_*``) and the supervisor switches ``ONB_V_GPS_HOLD`` / ``ONB_V_TAKEOFF_CHECK``; once for every profile the
NAV GPS side (``GPS_*``: ``sim.gps_nav.NavGpsParams.from_params`` with the fixed gains of ``compute_gains`` evaluated here
in float64, the ON-gate / usable thresholds as the exact integer mm / cm/s bounds, the boot configurator's tick counts,
USART1 at 38400 with its UBRR checked against params flight_computer.nav.gps.ubrr and PD2 / PD3 free on the NAV).  The
position gains come from ``sim/out/vtol_gains.json`` when it carries them (a ``position_hold`` block or flat ``pos_<key>``
entries, keys ``POS_GAIN_KEYS``), else params.json ``position_hold.gains_ASSUMED``, else the reference module constants.
The TELEM frame of a VTOL build with position hold is TELEM v3 (45 bytes).
v0.8.1 (four ATmega128A, params flight_computer.architecture_v081 / gns / glink): the GPS receiver, the GPS filter and the
position controller run on the GNS board (core/gns_node.h); the NAV's USART1 carries glink (``GLINK_*``: 250 kbaud,
UBRR checked against params glink.ubrr with U2X = 0 and <= 1 % error, the latency L = glink.latency_ticks_ASSUMED in
2..16 (the 4-entry result ring), both frames inside one 50 Hz cycle, the timing estimate of sim/glink_proto.py); the GNS
pin table (``flight_computer.gns.pins``: the GPS on USART1 PD2 / PD3, glink on USART0 PE0 / PE1) is checked like the other
chips', PD2 / PD3 stay free on the NAV (glink); per profile the touchdown policy (``POS_TOUCHDOWN_KEEP_TRIM``, params
position_hold.touchdown_policy.policy).  The ``GPS_*`` block is the GNS's (the same sim/gps_nav.py parameters).
v0.9 firmware round (fw09 SPEC_firmware_v09 3.2, G1 - per-configuration vehicle constants): the header carries one
constant set per vehicle configuration and every build flies the set of its own configuration, selected by ``FC_VCFG``
(platformio.ini): 0 VTOL battery, 1 VTOL tether, 2 VTOL hybrid (= the FCU power builds' ``PWR_SRC``), each derived
exactly as the 6-DOF reference flies that vehicle (``sim.vehicle.load_vtol_vehicle``: params.json with ``configs.vtol``
merged and ``power.source`` set, ``cad/exports/cadquery_report_vtol[_tether|_hybrid].json``; every parameter object
from that params dict); the ballistic builds (and host builds without ``FC_VCFG``) fly the rocket set (params.json as
is, ``cadquery_report.json``, the v0.8.1 numbers).  Only the macros of ``VCFG_MACROS`` may differ between the sets
(the generator renders the whole header once per configuration and refuses any other difference); ``FC_PARAMS_TAG``
names the configuration (and its CAD report), so a mixed image set is flagged by the NAV / FCU tag checks.  A
configuration whose CAD report is missing or unusable gets an ``#error`` (its builds do not compile).  W1: the nlink
exchange watchdog ``NLINK_WD_*`` from ``flight_computer.nlink.exchange_watchdog_us`` (> 1.5 x NLINK_XFER_US, < the tick
minus the NAV push floor ``NAV_PUSH_MIN_US_VTOL`` of src/nav/nav.h).  The v0.9 power pins come from
``flight_computer.pins`` (v_tether_adc, v_buffer_adc, buf_isign; checked with the FCU's pin table).

Run from the project root:  ``.venv/Scripts/python.exe tools/gen_firmware_params.py``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import struct
import sys
from fractions import Fraction

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
for _p in (ROOT, os.path.join(ROOT, "cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import layout as layout_mod  # noqa: E402  (cad/layout.py)
from sim import gps_nav as gpn  # noqa: E402
from sim import link_model as lm  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim import nlink_proto as nlp  # noqa: E402
from sim import vehicle as vm  # noqa: E402  (fw09 G1: the VTOL configurations' vehicles)
from sim.controller import vtol_overrides  # noqa: E402
from sim.onboard import SupervisorParams  # noqa: E402
from sim.position_ctrl import PositionParams  # noqa: E402
from sim.sim6dof import SensorConfig  # noqa: E402
from sim.vertical_ctrl import VerticalParams  # noqa: E402
from sim.state_machine import G0  # noqa: E402

PARAMS = os.path.join(ROOT, "params.json")
CAD_REPORT = os.path.join(ROOT, "cad", "exports", "cadquery_report.json")
FW_DIR = os.path.join(ROOT, "firmware", "atmega128_fc")
FC_CONFIG = os.path.join(FW_DIR, "fc_config.json")
OUT = os.path.join(FW_DIR, "src", "params_gen.h")
SIM6DOF = os.path.join(ROOT, "sim", "sim6dof.py")
VTOL_GAINS_JSON = os.path.join(ROOT, "sim", "out", "vtol_gains.json")
VTOL_GAIN_KEYS = ("alt_kp", "vz_kp", "vz_ki", "vz_i_max")      # mission.vtol.gains_ASSUMED / vtol_gains.json
VTOL_EST_GAIN_KEYS = ("kp", "ki", "accel_lpf_hz")               # mission.vtol.nav_estimator *_ASSUMED / vtol_gains.json est_*
# v0.8 position-hold gains: PositionParams field <- the names a gains file may use (position_hold.gains_ASSUMED names first)
POS_GAIN_KEYS = {"kp_pos": ("kp_pos",), "kv": ("kv",), "ki": ("ki",), "kd": ("kd",), "i_max": ("i_max_mps2", "i_max"),
                 "kd_lpf_hz": ("kd_lpf_hz",), "tilt_rate_dps": ("tilt_rate_dps",),
                 "v_max_land": ("v_max_land_mps", "v_max_land")}
# reference implementations src/core transcribes (recorded for provenance; re-verify when they change)
MIRRORED = [os.path.join(ROOT, "sim", n) for n in ("onboard.py", "link_proto.py", "link_model.py", "state_machine.py",
                                                   "quat.py", "controller.py", "estimator.py", "edf.py",
                                                   "ilink_proto.py", "nlink_proto.py", "vertical_ctrl.py",
                                                   "gps_nav.py", "position_ctrl.py", "glink_proto.py")]

NAV_H = os.path.join(FW_DIR, "src", "nav", "nav.h")   # NAV_PUSH_MIN_US_VTOL (W1 watchdog check; read, not a source)
NON_VEHICLE_PARTS = ("launch_pad",)          # same exclusion as sim/vehicle.py
EJECTED_PARTS = ("cansat_payload", "nose_cone", "chute_pack")
# fw09 G1: the VTOL vehicle configurations (params power.source) in FC_VCFG order; FC_VCFG = the FCU's PWR_SRC
VCFGS = ("battery", "tether", "hybrid")
VCFG_ID = {"battery": 0, "tether": 1, "hybrid": 2, "rocket": 3}
VCFG_MACRO = {"battery": "FC_VCFG_BATTERY", "tether": "FC_VCFG_TETHER", "hybrid": "FC_VCFG_HYBRID",
              "rocket": "FC_VCFG_ROCKET"}
VCFG_ENV_SUFFIX = {"battery": "", "tether": "_wired", "hybrid": "_wired_buffer"}   # platformio.ini env names
# the only macros whose value may differ between the configurations (render() refuses any other difference)
VCFG_MACROS = frozenset({
    "FC_PARAMS_TAG", "FC_VEHICLE_NAME",
    "MIX_L_V_M", "FC_HOVER_THROTTLE",
    "VEH_MASS_KG", "VEH_Z_CG_M", "VEH_IXX_KG_M2", "VEH_IYY_KG_M2", "VEH_IZZ_KG_M2", "VEH_POST_EJECTION_MASS_KG",
    "VC_U_HOVER", "VC_INV_U_HOVER", "VC_TO_SEARCH_F", "POS_TW"})
NLINK_WD_TIMER3_PRESCALER = 1                # W1: Timer3 runs at clk / 1 and restarts at 0 on every exchange (P1 I3)
NLINK_WD_XFER_FACTOR = 1.5                   # W1: the watchdog must exceed 1.5 x the exchange (fw09 SPEC 3.3 / 5)
LIPO_V_MAX_CELL = 4.2                        # LiPo chemistry: full-charge cell voltage (ADC range check only)
TIMER2_PRESCALERS = {1: 1, 8: 2, 64: 3, 256: 4, 1024: 5}      # ATmega128 Timer2 CS22:0
TIMER0_PRESCALERS = {1: 1, 8: 2, 32: 3, 64: 4, 128: 5, 256: 6, 1024: 7}   # ATmega128 Timer0 CS02:0
IMU_READ_BYTES = 14                          # ACCEL_XOUT_H .. GYRO_ZOUT_L burst
IMU_LEAD_MARGIN = 1.25                       # IMU read started this much earlier than its bus time
TIMER13_PRESCALERS = {1: 1, 8: 2, 64: 3, 256: 4, 1024: 5}     # ATmega128 Timer1/3 CSn2:0
ADC_PRESCALERS = {2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7}
ADC_CLK_MAX_HZ = 200_000                     # ATmega128 datasheet: 50..200 kHz for 10-bit resolution
UART_MAX_ERR = 0.02                          # baud error limit
# 2026-11-06: COM radio backends (params flight_computer.link.vehicle_radio; FC_LINK_RADIO): the two LoRa ones and the
# ESP8266 Wi-Fi bridge (hal/esp_bridge.c, firmware/esp_bridge) - since 2026-11-10 all three are sim/link_model.py RADIOS
VEHICLE_RADIOS = tuple(dict.fromkeys(tuple(lm.RADIOS) + ("esp_bridge",)))
# 2026-11-10: NAV barometer drivers (params flight_computer.nav.baro_sensor; FC_BARO_SENSOR, src/hal/baro.h).  A params
# file without the key (snapshots older than 2026-11-10) builds the MS5611 driver it was written for.
BARO_SENSORS = ("bmp280", "ms5611")
BARO_SENSOR_LEGACY = "ms5611"
ESP_UART_CLK_HZ = 80_000_000                 # ESP8266 UART clock (= firmware/esp_bridge/gen_esp_bridge_config.py; A)
ILINK_MAX_ERR = 0.01                         # ilink (U2X = 0): both chips run the same UBRR from 16 MHz crystals,
                                             # so they agree with each other; the tighter limit keeps the frames
                                             # readable by a USB-UART tap at the nominal ILINK_BAUD
MCU_NAMES = ("ATmega128", "ATmega128A")
# ATmega128 SPI master clock: F_CPU / div; div -> (SPR1:0, SPI2X)  (datasheet, SPCR/SPSR)
SPI_DIVIDERS = {2: (0, 1), 4: (0, 0), 8: (1, 1), 16: (1, 0), 32: (2, 1), 64: (2, 0), 128: (3, 0)}
# MPU-6050 register settings (product specification rev 3.4, tables 6.1/6.2 and register map 4.4-4.6)
MPU_GYRO_FS = {250.0: (0, 131.0), 500.0: (1, 65.5), 1000.0: (2, 32.8), 2000.0: (3, 16.4)}      # range dps -> FS_SEL, LSB/dps
MPU_ACCEL_FS = {2.0: (0, 16384.0), 4.0: (1, 8192.0), 8.0: (2, 4096.0), 16.0: (3, 2048.0)}      # range g -> AFS_SEL, LSB/g
MPU_DLPF_GYRO_DELAY_S = {0: 0.00098, 1: 0.0019, 2: 0.0028, 3: 0.0048, 4: 0.0083, 5: 0.0134, 6: 0.0186}
# ATmega128 alternate functions of the six output-compare pins (datasheet, "Alternate Port Functions")
OC_PINS = {"OC1A": "PB5", "OC1B": "PB6", "OC1C": "PB7", "OC3A": "PE3", "OC3B": "PE4", "OC3C": "PE5"}
OC_CHANNEL_ID = {"OC1A": 0, "OC1B": 1, "OC1C": 2, "OC3A": 3, "OC3B": 4, "OC3C": 5}
# ATmega128 port pins and their alternate functions (datasheet 2467, "Alternate Port Functions"; port G has
# five pins).  A pin text "Pxn/NAME ..." in params.json must name one of the functions of Pxn.
ALT_FUNCTIONS: dict[str, tuple[str, ...]] = {
    **{f"PA{i}": (f"AD{i}",) for i in range(8)},
    "PB0": ("SS",), "PB1": ("SCK",), "PB2": ("MOSI",), "PB3": ("MISO",), "PB4": ("OC0",), "PB5": ("OC1A",),
    "PB6": ("OC1B",), "PB7": ("OC2", "OC1C"),
    **{f"PC{i}": (f"A{8 + i}",) for i in range(8)},
    "PD0": ("SCL", "INT0"), "PD1": ("SDA", "INT1"), "PD2": ("RXD1", "INT2"), "PD3": ("TXD1", "INT3"),
    "PD4": ("ICP1",), "PD5": ("XCK1",), "PD6": ("T1",), "PD7": ("T2",),
    "PE0": ("RXD0", "PDI"), "PE1": ("TXD0", "PDO"), "PE2": ("XCK0", "AIN0"), "PE3": ("OC3A", "AIN1"),
    "PE4": ("OC3B", "INT4"), "PE5": ("OC3C", "INT5"), "PE6": ("T3", "INT6"), "PE7": ("ICP3", "INT7"),
    "PF0": ("ADC0",), "PF1": ("ADC1",), "PF2": ("ADC2",), "PF3": ("ADC3",), "PF4": ("ADC4", "TCK"),
    "PF5": ("ADC5", "TMS"), "PF6": ("ADC6", "TDO"), "PF7": ("ADC7", "TDI"),
    "PG0": ("WR",), "PG1": ("RD",), "PG2": ("ALE",), "PG3": ("TOSC2",), "PG4": ("TOSC1",),
}
PIN_OF_FUNCTION = {fn: pin for pin, fns in ALT_FUNCTIONS.items() for fn in fns}
# v0.6.3 chips (params.json flight_computer.architecture) and the signals each one owns.  fixed: the ATmega128
# peripheral function the signal needs, which fixes its pin; gpio: any port pin not used otherwise on that chip.
# lora_spi_nss / nav spi_ss are the SPI SS pin: those masters drive SS as an output, and an SPI master whose SS
# (PB0) is an input that goes low drops out of master mode (datasheet, "SS Pin Functionality"); the FCU's nlink_ss
# is the slave's hardware select input, which only PB0 is.
ILINK_USART = {"fcu": 1, "com": 0}           # = platformio.ini -D ILINK_USART of [fcu_common] / [com_common]; NAV: none
CHIP_SIGNALS = {
    "fcu": dict(where="flight_computer.pins",
                pwm=("vane_1_servo", "vane_2_servo", "vane_3_servo", "vane_4_servo", "esc", "release_servo"),
                fixed={"log_tx": "TXD0", "nlink_ss": "SS", "nlink_sck": "SCK", "nlink_mosi": "MOSI",
                       "nlink_miso": "MISO"},
                # v0.9 (fw09 SPEC 5): the power monitor's inputs (V_tether / V_buffer ADC, the buffer-current sign)
                # 2026-11-06: logic_bat_adc = the carrier's logic-battery monitor (PF3, read by no firmware yet)
                gpio=("led", "buzzer", "arm_switch", "buf_isign"),
                adc=("vbat_adc", "v_tether_adc", "v_buffer_adc", "logic_bat_adc")),
    "nav": dict(where="flight_computer.nav.pins", pwm=(),
                fixed={"i2c_scl": "SCL", "i2c_sda": "SDA", "imu_int": "INT6", "spi_ss": "SS", "spi_sck": "SCK",
                       "spi_mosi": "MOSI", "spi_miso": "MISO"},
                gpio=("led",), adc=()),
    "com": dict(where="flight_computer.com.pins", pwm=(),
                fixed={"link_uart_rx": "RXD1", "link_uart_tx": "TXD1", "lora_spi_nss": "SS", "lora_spi_sck": "SCK",
                       "lora_spi_mosi": "MOSI", "lora_spi_miso": "MISO"},
                gpio=("led", "link_m0", "link_m1", "link_aux", "lora_dio0", "lora_reset"), adc=()),
    # v0.8.1: the GNS board (flight_computer.gns): the GPS receiver (NEO-7M; NEO-6M in v0.8) on USART1, glink to the NAV on USART0 (its ISP pins)
    "gns": dict(where="flight_computer.gns.pins", pwm=(),
                fixed={"gps_rx": "RXD1", "gps_tx": "TXD1", "glink_rx": "RXD0", "glink_tx": "TXD0"},
                gpio=("led",), adc=()),
}
RADIO_SIGNALS = ("link_uart_rx", "link_uart_tx", "link_m0", "link_m1", "link_aux", "lora_spi_nss", "lora_spi_sck",
                 "lora_spi_mosi", "lora_spi_miso", "lora_dio0", "lora_reset")
SENSOR_SIGNALS = ("i2c_scl", "i2c_sda", "imu_int")
# nlink (SPEC 10.5): the FCU slave's SPI_STC interrupt (hal/nlink_spi.c, hand-written) writes the next return byte to
# SPDR this many CPU cycles after SPIF at the latest when no other interrupt is running: 4 response + up to 4 for a
# multi-cycle instruction in progress + 3 vector JMP + 16 to the SPDR write (counted from its instruction list and the
# avr-objdump of the FCU build).  The master's byte gap must be longer, or every return byte is late.
NLINK_SLAVE_REPLY_CYCLES = 27
# the NAV master's own overhead (avr-objdump of hal/nlink_spi.c nlink_spi_exchange, timed in simavr): SPIF -> next SPDR
# write = byte gap + 11 cycles; SS low -> first SPDR write and last SPIF -> SS high = byte gap + 8 cycles each
NLINK_MASTER_LOOP_CYCLES = 11
NLINK_MASTER_SS_CYCLES = 8
NLINK_XFER_MAX_TICK_FRACTION = 0.5           # an exchange may take at most this share of the NAV tick
NLINK_XFER_WARN_TICK_FRACTION = 0.25
# SX1276/77/78 (Semtech datasheet rev 7): crystal, register codes, ranges
SX127X_FXOSC_HZ = 32_000_000
SX127X_FREQ_RANGE_HZ = (137_000_000, 1_020_000_000)      # SX1276 (SX1278: 137-525 MHz)
SX127X_LF_HF_SPLIT_HZ = 525_000_000                      # below: LF port (LowFrequencyModeOn = 1, RSSI offset -164)
SX127X_BW_CODES = {7800: 0, 10400: 1, 15600: 2, 20800: 3, 31250: 4, 41700: 5, 62500: 6, 125000: 7, 250000: 8, 500000: 9}
SX127X_LORAWAN_PUBLIC_SYNC = 0x34
SX127X_LDRO_TSYM_S = 0.016                               # LowDataRateOptimize when T_sym > 16 ms (same rule as sim.link_model)


class GenError(RuntimeError):
    pass


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


# cad/cadquery/build_rocket.py writes its wall-clock build timings into the CAD report on every build; they say nothing
# about the vehicle, so the report is hashed without them (cad_report_sha256) and --check stays reproducible
CAD_REPORT_VOLATILE = (("extra", "build_time_s"), ("extra", "total_time_s"))


def cad_report_canonical(path: str) -> bytes:
    """The CAD report as canonical JSON: the volatile timing fields CAD_REPORT_VOLATILE removed, keys sorted, no
    whitespace, UTF-8, floats as Python's shortest round-trip repr - the same bytes for the same vehicle whatever the
    build took or how the file was formatted."""
    with open(path, "rb") as f:
        rep = json.loads(f.read().decode("utf-8"))
    for keys in CAD_REPORT_VOLATILE:
        node = rep
        for k in keys[:-1]:
            node = node.get(k) if isinstance(node, dict) else None
        if isinstance(node, dict):
            node.pop(keys[-1], None)
    return json.dumps(rep, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def cad_report_sha256(path: str) -> str:
    """sha256 of cad_report_canonical (what params_gen.h records for the CAD report)."""
    return hashlib.sha256(cad_report_canonical(path)).hexdigest()


def rel(path: str) -> str:
    try:
        return os.path.relpath(path, ROOT).replace(os.sep, "/")
    except ValueError:                     # another drive (Windows): keep the absolute path
        return path.replace(os.sep, "/")


def f32(x: float) -> float:
    """Round a double to the nearest IEEE single."""
    return struct.unpack("<f", struct.pack("<f", float(x)))[0]


def cf(x: float) -> str:
    """C float literal that parses to exactly f32(x)."""
    v = f32(x)
    if not math.isfinite(v):
        raise GenError(f"non-finite constant {x!r}")
    s = f"{v:.9g}"
    if "e" not in s and "." not in s and "n" not in s:
        s += ".0"
    return s + "f"


def ticks_ge(seconds: float, loop_hz: int) -> int:
    """Smallest tick count n with n / loop_hz >= seconds (exact arithmetic on the double value,
    values within 1e-9 tick of an integer snap to it so 0.3 s -> 75 ticks at 250 Hz).
    Identical to sim.onboard.ticks_ge."""
    x = Fraction(seconds) * loop_hz
    n = math.ceil(x)
    if n - x > 1 - Fraction(1, 10**9):     # x is a hair above an integer -> that integer
        n -= 1
    return max(int(n), 0)


def compose(parts: dict, exclude: tuple[str, ...]) -> tuple[float, list[float], list[list[float]]]:
    """Mass [kg], COM [m], inertia about the COM [kg m^2] of the report parts minus ``exclude``
    (``name == ex`` or ``name.startswith(ex + '_')``, zero-mass parts skipped) - sim/vehicle.py rules."""
    sel = [(n, pt) for n, pt in parts.items()
           if not any(n == ex or n.startswith(ex + "_") for ex in exclude) and float(pt["mass_g"]) != 0.0]
    if not sel:
        raise GenError("CAD report: no parts left after exclusions")
    m = sum(float(pt["mass_g"]) * 1e-3 for _, pt in sel)
    c = [sum(float(pt["mass_g"]) * 1e-3 * float(pt["com_mm"][i]) * 1e-3 for _, pt in sel) / m for i in range(3)]
    inertia = [[0.0] * 3 for _ in range(3)]
    for _, pt in sel:
        mi = float(pt["mass_g"]) * 1e-3
        d = [float(pt["com_mm"][i]) * 1e-3 - c[i] for i in range(3)]
        dd = sum(v * v for v in d)
        for i in range(3):
            for j in range(3):
                inertia[i][j] += float(pt["inertia_about_com_kg_m2"][i][j]) + mi * ((dd if i == j else 0.0) - d[i] * d[j])
    return m, c, inertia


def uart_setting(f_cpu: int, baud: int) -> tuple[int, int, float]:
    """(UBRR, U2X, relative error) with the smallest |error|; U2X = 0 wins ties."""
    best = None
    for u2x in (0, 1):
        div = 8 if u2x else 16
        ubrr = int(round(f_cpu / (div * baud) - 1))
        if not 0 <= ubrr <= 4095:
            continue
        err = f_cpu / (div * (ubrr + 1)) / baud - 1.0
        if best is None or abs(err) < abs(best[2]) - 1e-12:
            best = (ubrr, u2x, err)
    if best is None or abs(best[2]) > UART_MAX_ERR:
        raise GenError(f"no UBRR setting reaches {baud} baud within {UART_MAX_ERR:.0%} at {f_cpu} Hz")
    return best


def esp_bridge_settings(lk: dict, f_cpu: int, loop_hz: int) -> dict | None:
    """The COM's ESP bridge backend (hal/esp_bridge.c) from flight_computer.link.esp_bridge: USART1 baud with U2X = 0 (as
    the ilink: simavr ignores U2X) within uart_max_error on the COM and on the ESP's 80 MHz UART clock, the reset hold
    and the boot window in COM ticks.  None if the block is absent."""
    eb = lk.get("esp_bridge")
    if eb is None:
        return None
    baud, max_err = int(eb["uart_baud"]), float(eb["uart_max_error"])
    if not 0.0 < max_err <= UART_MAX_ERR:
        raise GenError(f"link.esp_bridge.uart_max_error {max_err} must be in (0, {UART_MAX_ERR}]")
    if baud <= 0:
        raise GenError(f"link.esp_bridge.uart_baud {baud} must be > 0")
    x = f_cpu / (16 * baud) - 1                                  # U2X = 0 (the ilink's rule, without its 1 % limit)
    ubrr = min((u for u in (math.floor(x), math.ceil(x)) if 0 <= u <= 4095),
               key=lambda u: (abs(f_cpu / (16 * (u + 1)) / baud - 1.0), u), default=0)
    err = f_cpu / (16 * (ubrr + 1)) / baud - 1.0
    esp_div = round(ESP_UART_CLK_HZ / baud)
    esp_err = ESP_UART_CLK_HZ / esp_div / baud - 1.0
    if abs(err) > max_err or abs(esp_err) > max_err:
        raise GenError(f"link.esp_bridge.uart_baud {baud}: COM error {err:+.3%} (UBRR {ubrr}, U2X 0) / ESP error "
                       f"{esp_err:+.4%} exceed uart_max_error {max_err:.2%}")
    hold_ms = int(eb["reset_hold_ms"])
    if hold_ms != eb["reset_hold_ms"] or not 1 <= hold_ms <= 20:
        raise GenError(f"link.esp_bridge.reset_hold_ms {eb['reset_hold_ms']} must be an integer 1..20 (boot-time busy wait)")
    boot_s = float(eb["boot_discard_ms"]) / 1e3
    boot_ticks = ticks_ge(boot_s, loop_hz)
    if not 1 <= boot_ticks <= 0xFFFF:
        raise GenError(f"link.esp_bridge.boot_discard_ms {eb['boot_discard_ms']} gives {boot_ticks} COM ticks (1..65535)")
    return {"baud": baud, "ubrr": ubrr, "err": err, "esp_err": esp_err, "max_err": max_err, "hold_ms": hold_ms,
            "boot_ms": float(eb["boot_discard_ms"]), "boot_ticks": boot_ticks}


def ilink_uart_setting(f_cpu: int, baud: int) -> tuple[int, float]:
    """(UBRR, relative error) of the ilink with U2X = 0 (simavr's USART model ignores U2X, so the
    two-chip co-simulation and the real chips must run the same normal-speed setting)."""
    if baud <= 0:
        raise GenError(f"flight_computer.ilink.baud {baud} must be > 0")
    x = f_cpu / (16 * baud) - 1
    cands = [u for u in (math.floor(x), math.ceil(x)) if 0 <= u <= 4095]
    if not cands:
        raise GenError(f"flight_computer.ilink.baud {baud}: UBRR {x:.2f} outside 0..4095 at {f_cpu} Hz (U2X = 0)")
    ubrr = min(cands, key=lambda u: (abs(f_cpu / (16 * (u + 1)) / baud - 1.0), u))
    err = f_cpu / (16 * (ubrr + 1)) / baud - 1.0
    if abs(err) > ILINK_MAX_ERR:
        raise GenError(f"flight_computer.ilink.baud {baud}: the nearest U2X = 0 setting (UBRR {ubrr}) is off by "
                       f"{err * 100:+.2f} % at {f_cpu} Hz, limit {ILINK_MAX_ERR:.0%}")
    return ubrr, err


def parse_pin(text: str) -> tuple[str, int]:
    """'PD2/RXD1 (note)' -> ('D', 2).  The pin must exist on the ATmega128 and a '/NAME' after it must be
    one of that pin's alternate functions."""
    m = re.match(r"\s*P([A-G])([0-7])\b", text)
    if not m:
        raise GenError(f"pin {text!r} does not start with P<port><bit>")
    name = f"P{m.group(1)}{m.group(2)}"
    if name not in ALT_FUNCTIONS:
        raise GenError(f"pin {text!r}: the ATmega128 has no {name} (port G is PG0..PG4)")
    lab = re.match(r"\s*P[A-G][0-7]\s*/\s*([A-Z][A-Z0-9]*)", text)
    if lab and lab.group(1) not in ALT_FUNCTIONS[name]:
        raise GenError(f"pin {text!r}: {lab.group(1)} is not an alternate function of {name} "
                       f"({' / '.join(ALT_FUNCTIONS[name])})")
    return m.group(1), int(m.group(2))


def chip_pins(chip: str, pins: dict) -> tuple[dict[str, tuple[str, int]], dict[str, str]]:
    """Check one chip's pin table on its own: every signal of CHIP_SIGNALS[chip] present and no other,
    PWM outputs on their OCnx pin, peripheral signals on the pin the ATmega128 fixes, the ilink on that
    chip's USART (ILINK_USART), no pin used twice on the chip.  Returns ({signal: (port, bit)}, {pwm signal:
    OCnx}).  The same pin on the other chip is a different pin."""
    sig = CHIP_SIGNALS[chip]
    where, tag = sig["where"], chip.upper()
    u = ILINK_USART.get(chip)
    ilink = {"ilink_rx": f"RXD{u}", "ilink_tx": f"TXD{u}"} if u is not None else {}
    fixed = {**sig["fixed"], **ilink}
    expected = list(sig["pwm"]) + list(fixed) + list(sig["gpio"]) + list(sig["adc"])
    if not isinstance(pins, dict):
        raise GenError(f"{where} missing")
    missing = [k for k in expected if k not in pins]
    if missing:
        raise GenError(f"{where}: {', '.join(missing)} missing (SPEC 10.3 / 10.4 wiring)")
    extra = sorted(k for k in pins if k not in expected and k != "note" and not k.startswith("_"))
    if extra:
        hint = ""
        if chip == "fcu" and set(extra) & set(RADIO_SIGNALS):
            hint = " (v0.6: the radio pins live in flight_computer.com.pins)"
        elif chip == "fcu" and set(extra) & set(SENSOR_SIGNALS):
            hint = " (v0.6.3: the sensors and their I2C live on the NAV, flight_computer.nav.pins)"
        elif chip == "nav" and set(extra) & {"ilink_rx", "ilink_tx"}:
            hint = " (the NAV talks only to the FCU, over the nlink SPI)"
        raise GenError(f"{where}: {', '.join(extra)} is not a signal of the {tag}{hint}")
    used: dict[tuple[str, int], str] = {}
    parsed: dict[str, tuple[str, int]] = {}
    pwm_map: dict[str, str] = {}

    def pin_of(key: str) -> tuple[str, int]:
        try:
            return parse_pin(str(pins[key]))
        except GenError as exc:
            raise GenError(f"{where}.{key}: {exc}") from None

    def claim(key: str) -> tuple[str, int]:
        pin = pin_of(key)
        if pin in used:
            raise GenError(f"{where}.{key} and .{used[pin]} both use P{pin[0]}{pin[1]} on the {tag}")
        used[pin] = key
        parsed[key] = pin
        return pin

    for key in sig["pwm"]:
        m = re.match(r"\s*(P[A-G][0-7])\s*/\s*(OC[13][ABC])\b", pins[key])
        if not m or OC_PINS[m.group(2)] != m.group(1):
            raise GenError(f"{where}.{key} = {pins[key]!r} is not a valid ATmega128 OCnx pin")
        pwm_map[key] = m.group(2)
        claim(key)
    if len(set(pwm_map.values())) != len(pwm_map):
        raise GenError(f"two outputs share a compare channel: {pwm_map}")
    for key, fn in fixed.items():
        pin_name = PIN_OF_FUNCTION[fn]
        if pin_of(key) != (pin_name[1], int(pin_name[2])):
            if key in ilink:
                raise GenError(f"{where}.{key} = {pins[key]!r}: the {tag} ilink runs on its USART{u} "
                               f"({fn} = {pin_name}; platformio.ini [{chip}_common] -D ILINK_USART={u})")
            why = (" (the SPI's SS pin: a master keeps it an output so it stays master, a slave's hardware select is "
                   "only this pin)") if fn == "SS" else ""
            raise GenError(f"{where}.{key} = {pins[key]!r}: the ATmega128 {fn} is fixed to {pin_name}{why}")
        claim(key)
    for key in sig["gpio"]:
        claim(key)
    for key in sig["adc"]:
        claim(key)
    return parsed, pwm_map


AXES = {"+x": (0, 1), "-x": (0, -1), "+y": (1, 1), "-y": (1, -1), "+z": (2, 1), "-z": (2, -1)}


def imu_mounting(cfg: dict) -> tuple[list[int], list[int]]:
    """body axis i = sign[i] * imu[index[i]]; must be a proper rotation."""
    idx, sgn = [], []
    for key in ("body_x", "body_y", "body_z"):
        a = AXES.get(str(cfg[key]).strip().lower())
        if a is None:
            raise GenError(f"fc_config imu_mounting.{key} = {cfg[key]!r}; use one of {sorted(AXES)}")
        idx.append(a[0])
        sgn.append(a[1])
    if sorted(idx) != [0, 1, 2]:
        raise GenError(f"imu_mounting uses an IMU axis twice: {cfg}")
    mat = [[0] * 3 for _ in range(3)]
    for i in range(3):
        mat[i][idx[i]] = sgn[i]
    det = (mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1])
           - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
           + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]))
    if det != 1:
        raise GenError(f"imu_mounting {cfg} is a reflection (det {det}); a mounted chip is always a proper rotation")
    return idx, sgn


def band_from_note(note: str) -> tuple[float, float]:
    """The permitted band [Hz] named in flight_computer.link.frequency_note ('920.9-923.3 MHz')."""
    m = re.search(r"(\d+(?:\.\d+)?)\s*[-–]\s*(\d+(?:\.\d+)?)\s*MHz", note)
    if not m:
        raise GenError(f"flight_computer.link.frequency_note {note!r} names no band 'a-b MHz': the generator "
                       "only accepts a frequency inside the band written there")
    lo, hi = float(m.group(1)) * 1e6, float(m.group(2)) * 1e6
    if not lo < hi:
        raise GenError(f"frequency_note band {lo}-{hi} Hz is empty")
    return lo, hi


def sx127x_registers(lk: dict, cfg_link: dict, loop_hz: int, warnings: list[str],
                     telem_bytes: int = lp.TELEM_FRAME_BYTES) -> dict:
    """LoRa-mode register values for link.sx127x (SX1276/77/78 datasheet section 6.4 register map); the TX timeout is
    sized for ``telem_bytes`` (the TELEM frame the build sends: v1 36 B ballistic, v2 39 B VTOL)."""
    sx = lk["sx127x"]
    f_hz = float(sx["freq_hz"])
    lo, hi = SX127X_FREQ_RANGE_HZ
    if not lo <= f_hz <= hi:
        raise GenError(f"link.sx127x.freq_hz {f_hz:.0f} outside the SX1276 range {lo:.0f}..{hi:.0f} Hz")
    frf = int(round(f_hz * (1 << 19) / SX127X_FXOSC_HZ))
    if not 0 < frf < (1 << 24):
        raise GenError(f"FRF {frf} does not fit 24 bits")
    f_real = frf * SX127X_FXOSC_HZ / float(1 << 19)
    bw = float(sx["bandwidth_hz"])
    bw_code = SX127X_BW_CODES.get(int(round(bw)))
    if bw_code is None or abs(bw - round(bw)) > 1e-6:
        raise GenError(f"link.sx127x.bandwidth_hz {bw} is not an SX127x LoRa bandwidth {sorted(SX127X_BW_CODES)}")
    band = band_from_note(str(lk.get("frequency_note", "")))
    if not (band[0] <= f_real - bw / 2.0 and f_real + bw / 2.0 <= band[1]):
        raise GenError(f"channel {f_real / 1e6:.4f} MHz +- {bw / 2e3:.1f} kHz leaves the band {band[0] / 1e6:.1f}-"
                       f"{band[1] / 1e6:.1f} MHz of flight_computer.link.frequency_note (check the legality of any other "
                       "band at rra.go.kr and update params.json first)")
    sf = int(sx["spreading_factor"])
    if sf == 6:
        raise GenError("SF6 needs the implicit-header mode; SPEC 10.3 requires an explicit header")
    if not 7 <= sf <= 12:
        raise GenError(f"spreading factor {sf} outside 7..12")
    crd = int(sx["coding_rate_denom"])
    if not 5 <= crd <= 8:
        raise GenError(f"coding rate 4/{crd} outside 4/5..4/8")
    pre = int(sx["preamble_symbols"])
    if not 6 <= pre <= 0xFFFF:
        raise GenError(f"preamble {pre} symbols outside 6..65535")
    sync = int(sx["sync_word"])
    if not 0 <= sync <= 0xFF:
        raise GenError(f"sync word {sync} is not a byte")
    if sync == SX127X_LORAWAN_PUBLIC_SYNC:
        warnings.append("link.sx127x.sync_word 0x34 is the LoRaWAN public-network word; a private link should use another")
    pwr = int(sx["tx_power_dbm"])
    pa_boost = bool(cfg_link["sx127x_pa_boost"])
    if pa_boost:
        if not 2 <= pwr <= 17:
            raise GenError(f"tx_power_dbm {pwr} outside 2..17 dBm of the PA_BOOST pin (20 dBm needs PaDac, not supported)")
        pa_config = 0x80 | (pwr - 2)
    else:
        if not 0 <= pwr <= 15:
            raise GenError(f"tx_power_dbm {pwr} outside 0..15 dBm of the RFO pin")
        pa_config = 0x70 | pwr                                   # MaxPower 7 -> Pmax 15 dBm, Pout = OutputPower
    t_sym = (1 << sf) / bw
    ldro = t_sym > SX127X_LDRO_TSYM_S
    lf = f_hz < SX127X_LF_HF_SPLIT_HZ
    toa_cmd = lm.lora_time_on_air(lp.CMD_FRAME_BYTES, sf, bw, crd, pre)
    toa_tlm = lm.lora_time_on_air(telem_bytes, sf, bw, crd, pre)
    tx_timeout_s = float(cfg_link["sx127x_tx_timeout_factor"]) * toa_tlm + float(cfg_link["sx127x_tx_timeout_margin_s"])
    return dict(f_hz=f_hz, f_real=f_real, frf=frf, band=band, bw=bw, bw_code=bw_code, sf=sf, crd=crd, pre=pre, sync=sync,
                pwr=pwr, pa_boost=pa_boost, pa_config=pa_config, ldro=ldro, lf=lf, t_sym=t_sym,
                mc1=(bw_code << 4) | ((crd - 4) << 1) | 0,               # explicit header
                mc2=(sf << 4) | 0x04,                                    # RxPayloadCrcOn
                mc3=(0x08 if ldro else 0x00) | 0x04,                     # AgcAutoOn
                lna=0x20 if lf else 0x23, rssi_offset=-164 if lf else -157,
                toa_cmd=toa_cmd, toa_tlm=toa_tlm, tx_timeout_s=tx_timeout_s, telem_bytes=telem_bytes,
                tx_timeout_ticks=max(1, ticks_ge(tx_timeout_s, loop_hz)))


# --------------------------------------------------------------------------- #
# derivation
# --------------------------------------------------------------------------- #
def vtol_gains(p: dict, params_path: str = PARAMS, gains_path: str = VTOL_GAINS_JSON) -> dict:
    """The vertical (and, v0.7.1, NAV-estimator) gains the firmware flies, with provenance (public: the cross-check uses
    the same set).

    Vertical: ``sim/out/vtol_gains.json`` (the 6-DOF VTOL tuner's output) when it holds every key of ``VTOL_GAIN_KEYS``
    as a finite number, else ``mission.vtol.gains_ASSUMED``.  Estimator (``est``, overrides of
    ``mission.vtol.nav_estimator`` kp_ASSUMED / ki_ASSUMED / accel_lpf_hz_ASSUMED; the gate and the bias limit stay
    params.json): from the same JSON when it carries them - flat ``est_kp`` / ``est_ki`` / ``est_accel_lpf_hz`` (what
    sim/vtol_scenarios.py --tune writes, EGAIN_KEYS), ``gm_*`` (the 6-DOF's SensorConfig names) or a ``nav_estimator``
    block - each a finite number >= 0, else none (params.json).  ``params_stale``: the JSON names
    (``_meta.params_sha256`` or ``params_sha256``) a params.json other than this one - the gains were tuned against another
    vehicle definition (the flag covers both sets: one file, one tuning run)."""
    g = {k: float(p["mission"]["vtol"]["gains_ASSUMED"][k]) for k in VTOL_GAIN_KEYS}
    info = {"gains": g, "source": "params.json mission.vtol.gains_ASSUMED", "source_path": params_path, "tuned": False,
            "params_stale": False, "warnings": [], "est": {}, "est_tuned": False,
            "est_source": "params.json mission.vtol.nav_estimator", "pos": {}, "pos_tuned": False,
            "pos_source": ("params.json mission.vtol.position_hold.gains_ASSUMED"
                           if (p["mission"]["vtol"].get("position_hold") or {}).get("gains_ASSUMED")
                           else "sim/position_ctrl.py module constants (params.json has no position_hold.gains_ASSUMED)")}
    if os.path.exists(gains_path):
        try:
            with open(gains_path, "r", encoding="utf-8") as f:
                gj = json.load(f)
            bad = [k for k in VTOL_GAIN_KEYS if not isinstance(gj.get(k), (int, float)) or not math.isfinite(float(gj[k]))]
            if bad:
                info["warnings"].append(f"{rel(gains_path)} lacks finite {bad}; using params.json mission.vtol.gains_ASSUMED")
            else:
                info.update(gains={k: float(gj[k]) for k in VTOL_GAIN_KEYS}, source=rel(gains_path),
                            source_path=gains_path, tuned=True)
            ne = gj.get("nav_estimator") if isinstance(gj.get("nav_estimator"), dict) else {}
            est = {}
            for k in VTOL_EST_GAIN_KEYS:
                for v in (ne.get(k), ne.get(f"{k}_ASSUMED"), gj.get(f"est_{k}"), gj.get(f"gm_{k}")):
                    if isinstance(v, (int, float)) and not isinstance(v, bool):
                        if not (math.isfinite(float(v)) and float(v) >= 0.0):
                            info["warnings"].append(f"{rel(gains_path)}: estimator {k} = {v} is not a finite number >= 0; "
                                                    "using params.json mission.vtol.nav_estimator")
                        else:
                            est[k] = float(v)
                        break
            if est:
                info.update(est=est, est_tuned=True, est_source=rel(gains_path))
                info["source_path"] = gains_path
            pb = gj.get("position_hold") if isinstance(gj.get("position_hold"), dict) else {}
            pos = {}
            for k, names in POS_GAIN_KEYS.items():
                for v in [pb.get(nm) for nm in names] + [gj.get(f"pos_{nm}") for nm in names]:
                    if isinstance(v, (int, float)) and not isinstance(v, bool):
                        if not (math.isfinite(float(v)) and float(v) >= 0.0):
                            info["warnings"].append(f"{rel(gains_path)}: position gain {k} = {v} is not a finite number "
                                                    ">= 0; using the params.json / module value")
                        else:
                            pos[k] = float(v)
                        break
            if pos:
                info.update(pos=pos, pos_tuned=True, pos_source=rel(gains_path))
                info["source_path"] = gains_path
            sha = (gj.get("_meta") or {}).get("params_sha256") or gj.get("params_sha256")
            if (info["tuned"] or est or pos) and sha and sha != sha256(params_path):
                info["params_stale"] = True
                info["warnings"].append(f"{rel(gains_path)} was tuned against another params.json (params_sha256 "
                                        "differs) - re-run the VTOL gain tuning (sim/vtol_scenarios.py --tune)")
        except (OSError, ValueError) as exc:
            info["warnings"].append(f"cannot read {rel(gains_path)}: {exc!r}; using params.json mission.vtol.gains_ASSUMED")
    info["sha256"] = sha256(info["source_path"])
    return info


def vtol_estimator_constants(v: dict, loop_hz: int, g0: float, est: dict) -> dict:
    """The v0.7.1 VTOL NAV estimator constants (sim/estimator.py GatedMahonyEstimator) of one profile's merged
    ``mission.vtol`` values ``v``, with the tuned kp / ki of ``est`` (vtol_gains) when present."""
    ne = v.get("nav_estimator")
    if not isinstance(ne, dict):
        raise GenError("mission.vtol.nav_estimator missing (v0.7.1 VTOL NAV estimator)")
    if str(ne.get("type", "gated_mahony")) != "gated_mahony":
        raise GenError(f"mission.vtol.nav_estimator.type {ne.get('type')!r} is not 'gated_mahony' (core/att_estimator.h)")
    dt = 1.0 / loop_hz
    kp = float(est.get("kp", ne["kp_ASSUMED"]))
    ki = float(est.get("ki", ne["ki_ASSUMED"]))
    f_a = float(est.get("accel_lpf_hz", ne["accel_lpf_hz_ASSUMED"]))
    gate, lim_dps = float(ne["accel_gate"]), float(ne["bias_limit_dps"])
    if not (kp >= 0 and ki >= 0 and 0 < gate < 1 and f_a > 0 and lim_dps >= 0):
        raise GenError("mission.vtol.nav_estimator: need kp, ki >= 0, 0 < accel_gate < 1, accel_lpf_hz > 0, "
                       "bias_limit_dps >= 0")
    return dict(kp=kp, ki=ki, ki_dt=ki * dt, accel_gate=gate, gate_mps2=gate * g0, accel_lpf_hz=f_a,
                alpha_a=1.0 - math.exp(-2.0 * math.pi * f_a * dt), bias_limit_dps=lim_dps,
                bias_limit_rad_s=math.radians(lim_dps), g=g0)


def vtol_alt_filter_constants(v: dict, loop_hz: int, u_hover: float, n_exp: float, sens) -> dict:
    """The v0.7.1 VTOL NAV altitude filter constants (sim/state_machine.py VtolAltitudeFilter): the 6-DOF's own
    SensorConfig crossover / damping / bias pole / pad time constant (sim/sim6dof.py builds the filter from them), the
    reference's bias limit, mission.vtol.baro_thrust_comp_m of the profile, u_hover of the vertical controller."""
    from sim.state_machine import VTOL_ALT_BIAS_LIMIT_MPS2_ASSUMED, VTOL_ALT_BIAS_TAU_S_ASSUMED, VtolAltitudeFilter
    dt = 1.0 / loop_hz
    bias_tau = float(getattr(sens, "vtol_alt_bias_tau_s", VTOL_ALT_BIAS_TAU_S_ASSUMED))
    pad_tau = float(sens.pad_avg_tau_s)
    comp = float(v.get("baro_thrust_comp_m", 0.0))
    try:
        f = VtolAltitudeFilter(float(sens.altitude_crossover_hz), float(sens.altitude_zeta), bias_tau, 0.0, pad_tau,
                               VTOL_ALT_BIAS_LIMIT_MPS2_ASSUMED, comp, u_hover, n_exp)
    except ValueError as exc:
        raise GenError(f"VTOL altitude filter: {exc}") from exc
    n_avg = max(1, int(round(pad_tau / dt)))                     # VtolAltitudeFilter.pad(): the same expression
    if n_avg > 0xFFFF:
        raise GenError(f"VTOL altitude filter pad N = {n_avg} does not fit uint16")
    return dict(k1=f.k1, k2=f.k2, k3=f.k3, bias_tau_s=bias_tau, pad_tau_s=pad_tau, pad_n_avg=n_avg,
                bias_limit=VTOL_ALT_BIAS_LIMIT_MPS2_ASSUMED, comp_m=comp, crossover_hz=float(sens.altitude_crossover_hz),
                zeta=float(sens.altitude_zeta))


def derive_vtol(p: dict, params_path: str, cfg_path: str, loop_hz: int, m_full: float, t_max: float,
                warnings: list[str], vbat_v_per_count: float = 0.0275) -> dict | None:
    """The VTOL build's supervisor, vertical-controller, NAV estimator and NAV altitude-filter parameters (SPEC 8.9) for
    EVERY build profile of params.json (v0.7.1 mission.vtol.profile / profiles: the reference's own parameter objects
    ``SupervisorParams.from_params(mode='vtol', profile=)``, ``VerticalParams.from_params(profile=)``), checked for the C
    types.  ``profiles`` = {name: {...}} with the default (mission.vtol.profile) first, then params.json's order; ``spv``
    / ``vp`` = the default profile's.  None when params.json has no ``mission.vtol`` (a v0.6.3 file: ballistic only)."""
    from sim.onboard import vtol_profiles, vtol_values
    ms = p["mission"]
    if "vtol" not in ms:
        if lp.mission_mode(p) == lp.MODE_VTOL:
            raise GenError("mission.mode = 'vtol' needs mission.vtol")
        return None
    try:
        names = list(vtol_profiles(p))
        default = vtol_values(p)["profile"]
    except ValueError as exc:
        raise GenError(f"mission.vtol profiles: {exc}") from exc
    for nm in names:
        if not re.fullmatch(r"[a-z][a-z0-9_]*", nm):
            raise GenError(f"mission.vtol profile name {nm!r}: lower-case letters, digits and '_' only (it becomes the "
                           f"build switch MISSION_PROFILE_{nm.upper()})")
    names = [default] + [nm for nm in names if nm != default]
    gi = vtol_gains(p, params_path)
    warnings.extend(gi["warnings"])
    sens = SensorConfig()
    n_exp = float(p["edf"]["throttle_exponent"])
    profiles = {}
    for nm in names:
        try:
            v = vtol_values(p, nm)
            spv = SupervisorParams.from_params(p, fc_config_path=cfg_path, mode=lp.MODE_VTOL, profile=nm)
        except ValueError as exc:
            raise GenError(f"mission.vtol (profile {nm}): {exc}") from exc
        try:
            vp = VerticalParams.from_params(p, m_full, t_max, g=G0, profile=nm, **gi["gains"])
        except ValueError as exc:
            raise GenError(f"mission.vtol (profile {nm}, vertical controller): {exc}") from exc
        tag = f"mission.vtol ({nm})"
        secs = {"spool_ticks": ("spool_warning_s",), "link_land_ticks": ("link_loss_land_s",),
                "endurance_ticks": ("max_flight_s", "land_reserve_s"), "hard_cutoff_ticks": ("hard_cutoff_s",),
                "v_fall_confirm_ticks": ("fall_confirm_s",), "safe_after_landed_ticks": ("safe_after_landed_s",)}
        for k, keys in secs.items():
            n = getattr(spv, k)
            if not 0 <= n < 0xFFFFFFFF:
                raise GenError(f"VTOL {k} = {n} ticks does not fit uint32 ({nm})")
            sec = float(v[keys[0]]) - (float(v[keys[1]]) if len(keys) > 1 else 0.0)
            if abs(n / loop_hz - sec) > 1e-9:
                warnings.append(f"{tag}.{' - '.join(keys)} = {sec:g} s is not a whole number of ticks; firmware uses {n}")
        for k in ("ramp_ticks", "landed_confirm_ticks", "floor_ticks"):
            if not 0 <= getattr(vp, k) <= 0xFFFF:
                raise GenError(f"vertical controller {k} = {getattr(vp, k)} does not fit uint16 ({nm})")
        if not 0 <= spv.land_timeout_ticks <= 0xFFFF:
            raise GenError(f"{tag}.touchdown.land_floor_timeout_s = {spv.land_timeout_ticks} ticks: the controller's "
                           "low_in_land_ticks counter is uint16 (<= 65535)")
        if not (0 <= spv.baro_limit_base_ticks < 0xFFFFFFFF and 0 <= spv.baro_limit_mt_per_dm <= 0xFFFFFFFF
                and 0 <= spv.v_geofence_dm <= 0x7FFF):
            raise GenError(f"{tag}.baro_lost: the time-limit integers do not fit their C types")
        if spv.baro_limit_base_ticks + -(-spv.v_geofence_dm * spv.baro_limit_mt_per_dm // 1000) > 0xFFFFFFFF:
            raise GenError(f"{tag}.baro_lost: the longest time limit overflows uint32")
        for k in ("up_um", "dn_um", "td_um"):
            if not 1 <= getattr(vp, k) <= 100_000_000:
                raise GenError(f"vertical controller {k} = {getattr(vp, k)} um per tick outside 1..1e8 ({nm})")
        if not -2_000_000_000 <= vp.flare_um <= 2_000_000_000:
            raise GenError(f"{tag}.flare_altitude_m outside +-2000 m")
        if abs(vp.landed_confirm_ticks / loop_hz - float(v["landed_confirm_s"])) > 1e-9:
            warnings.append(f"{tag}.landed_confirm_s = {v['landed_confirm_s']} s is not a whole number of ticks")
        for k in ("hover_altitude_m", "alt_step_m", "alt_min_m", "alt_max_m", "geofence_m"):
            if abs(float(v[k]) * 10.0 - round(float(v[k]) * 10.0)) > 1e-9:
                warnings.append(f"{tag}.{k} = {v[k]} m is not a whole number of decimetres (TELEM v2 / UP-DOWN unit); "
                                "the firmware rounds it")
        for k, step in (("climb_rate_mps", vp.up_um), ("descent_rate_mps", vp.dn_um), ("touchdown_rate_mps", vp.td_um)):
            if abs(vp.effective_rate(step) - float(v[k])) > 1e-9:
                warnings.append(f"{tag}.{k} = {v[k]} m/s is not a whole number of um per tick; the reference flies "
                                f"{vp.effective_rate(step):.6f} m/s")
        if vp.hover_margin <= 0.0:
            warnings.append(f"VTOL ({nm}): hover throttle {vp.u_hover:.4f} >= u_max {vp.u_max:g} - the vehicle cannot hover")
        elif vp.hover_margin < 0.1 and nm == default:
            warnings.append(f"VTOL: hover throttle {vp.u_hover:.4f}, only {vp.hover_margin:.3f} of throttle above it (u_max "
                            f"{vp.u_max:g}, T/W {t_max / (m_full * G0):.3f}): climbs and gust rejection will saturate")
        # fall_vz must lie below every commanded descent, or a normal landing trips V5
        if not spv.v_fall_vz_mps < -vp.vz_max_down_mps:
            raise GenError(f"{tag}.fall_vz_mps {spv.v_fall_vz_mps} must be below -vz_max_down_mps "
                           f"{-vp.vz_max_down_mps}: a commanded descent would trip the V5 fall abort")
        est = vtol_estimator_constants(v, loop_hz, G0, gi["est"])
        if v.get("nav_estimator") != ms["vtol"].get("nav_estimator") or (
                v.get("baro_thrust_comp_m") != ms["vtol"].get("baro_thrust_comp_m")):
            warnings.append(f"profile {nm} overrides mission.vtol.nav_estimator / baro_thrust_comp_m: the firmware flies "
                            "the profile's values, sim/estimator.py / sim/state_machine.py from_params read mission.vtol")
        alt = vtol_alt_filter_constants(v, loop_hz, vp.u_hover, n_exp, sens)
        pp = None
        if spv.gps_hold:                                         # v0.8 position hold of this profile
            try:
                pp = PositionParams.from_params(p, m_full, t_max, g=G0, profile=nm, **gi["pos"])
            except (ValueError, KeyError) as exc:
                raise GenError(f"{tag}.position_hold: {exc}") from exc
            if pp.fcu_hz != loop_hz or pp.step_ticks < 5:
                raise GenError(f"{tag}.position_hold.loop_hz {pp.loop_hz}: the five-job pipeline (core/position_ctrl.h, "
                               f"POS_PIPELINE_TICKS 4) needs control.loop_hz / loop_hz >= 5, got {pp.step_ticks}")
            for k in ("guard_steps",):
                if not 1 <= getattr(pp, k) <= 255:
                    raise GenError(f"{tag}.position_hold: {k} = {getattr(pp, k)} does not fit the uint8 guard timer")
            for k in ("timeout_steps", "land_steps", "plaus_limited_steps", "plaus_vexc_steps", "plaus_reset_window_steps"):
                if not 0 <= getattr(pp, k) <= 0xFFFF:
                    raise GenError(f"{tag}.position_hold: {k} = {getattr(pp, k)} does not fit uint16")
            if not 1 <= pp.plaus_resets <= 4:
                raise GenError(f"{tag}.position_hold: plaus_resets {pp.plaus_resets} outside 1..4 (core/position_ctrl.h "
                               "POS_RESETS_MAX)")
        if not 0 <= vp.takeoff_ticks <= 0xFFFF:
            raise GenError(f"{tag}.position_hold.takeoff_check_ASSUMED: {vp.takeoff_ticks} ticks does not fit uint16")
        profiles[nm] = dict(spv=spv, vp=vp, v=v, est=est, alt=alt, pp=pp)
    pad_tau_gains = ggh_pad_tau()
    if abs(float(sens.pad_avg_tau_s) - pad_tau_gains) > 1e-12:
        warnings.append(f"SensorConfig.pad_avg_tau_s {sens.pad_avg_tau_s} differs from gains.h GAIN_EST_PAD_AVG_TAU_S "
                        f"{pad_tau_gains}: the NAV altitude filter's pad average and the estimator's pad alignment run "
                        "different windows")
    dflt = profiles[default]
    try:
        att_ov = vtol_overrides(p["control"])
    except ValueError as exc:
        raise GenError(str(exc)) from exc
    for k, val in list(att_ov.items()):
        if k in VTOL_OV_NOT_PORTED:
            if isinstance(val, (int, float)) and not isinstance(val, bool) and float(val) == 0.0:
                del att_ov[k]                             # 0 = off: the reference's default, nothing to port
                continue
            raise GenError(f"control.vtol_overrides.{k} = {val!r}: the v0.9 option is evaluated in the reference but NOT "
                           "ported to the C core (sim/controller.py docstring: not flown) - port it before enabling it")
        if k not in VTOL_OV_C_FIELDS:
            raise GenError(f"control.vtol_overrides.{k}: core/fc_config.c fc_vtol_gains has no field for it")
        if k in VTOL_OV_INT_CHOICES:                      # v0.9 integer switches
            ok = isinstance(val, int) and not isinstance(val, bool) and val in VTOL_OV_INT_CHOICES[k]
        elif k == "ff_rotor_h_nms":                       # signed: s I_rot omega_ref (the rotation sign s)
            ok = isinstance(val, (int, float)) and not isinstance(val, bool) and math.isfinite(float(val))
        elif k == "ff_rotor_tau_s":
            ok = (isinstance(val, (int, float)) and not isinstance(val, bool) and math.isfinite(float(val))
                  and float(val) > 0.0)
        else:
            ok = (isinstance(val, bool) if k == "yaw_hold" else (isinstance(val, (int, float)) and not isinstance(val, bool)
                                                               and math.isfinite(float(val)) and float(val) >= 0.0))
        if not ok:
            raise GenError(f"control.vtol_overrides.{k} = {val!r}: not a valid value")
    att_v09 = vtol_att_v09(p, att_ov)
    for nm, pr in profiles.items():                       # v0.9 vertical / power options of each profile
        vpn = pr["vp"]
        if vpn.u_slew_per_s > 0.0 or vpn.to_cap_margin > 0.0:
            raise GenError(f"mission.vtol.takeoff_v09 ({nm}): u_slew_per_s / cap_margin > 0 are reference options NOT "
                           "ported to core/vertical_ctrl.c (evaluated, not flown) - port them before enabling them")
        pr["v09"] = vertical_v09(vpn, nm)
        pr["pwr"] = power_v09(pr["spv"], nm, loop_hz, vbat_v_per_count)
    gps = derive_gps(p, loop_hz) if any(pr["pp"] is not None for pr in profiles.values()) else None
    glink = None
    if gps is not None and isinstance(p["flight_computer"].get("glink"), dict):   # v0.8.1 four-chip build
        glink = derive_glink(p, int(p["flight_computer"]["f_cpu_hz"]), loop_hz, gps["gp"], warnings)
        for nm, pr in profiles.items():
            if pr["pp"] is not None and pr["pp"].land_steps * pr["pp"].step_ticks > 0xFFFE:
                raise GenError(f"position_hold ({nm}): land_steps x step_ticks > 65534 - the FCU's GNS-loss age is uint16")
    return dict(spv=dflt["spv"], vp=dflt["vp"], gains=gi, profiles=profiles, default=default, att_ov=att_ov, gps=gps,
                glink=glink, att_v09=att_v09)


def _max_int_le(thr: float, scale: float) -> int:
    """The largest integer x >= 0 with x / scale <= thr in float64 (the reference's ``hacc_mm / 1000.0 <= max_hacc_m``),
    so the firmware's integer compare decides exactly like it."""
    if not (math.isfinite(thr) and thr >= 0.0):
        raise GenError(f"GPS threshold {thr} must be a finite number >= 0")
    x = int(math.floor(thr * scale))
    while (x + 1) / scale <= thr:
        x += 1
    while x > 0 and x / scale > thr:
        x -= 1
    if x / scale > thr or x > 0xFFFFFFFF:
        raise GenError(f"GPS threshold {thr}: no uint32 bound")
    return x


def derive_gps(p: dict, loop_hz: int) -> dict:
    """v0.8 NAV GPS side (the same for every profile): sim/gps_nav.py NavGpsParams.from_params and its fixed gains
    (compute_gains in float64), the integer forms of its thresholds, the boot configurator's tick counts."""
    try:
        gp = gpn.NavGpsParams.from_params(p)
    except (ValueError, KeyError) as exc:
        raise GenError(f"flight_computer.nav.gps / position_hold: {exc}") from exc
    g = p["flight_computer"]["nav"]["gps"]
    if gp.loop_hz != loop_hz or gp.step_ticks != 5:
        raise GenError(f"flight_computer.nav.gps.fusion_hz {gp.fusion_hz}: core/gps_nav.c runs the step at j = 4 and the "
                       "epoch fusion in four jobs j = 0..3 (loop_hz / fusion_hz must be 5)")
    if int(g.get("epoch_split_ticks", 4)) != 4:
        raise GenError("flight_computer.nav.gps.epoch_split_ticks must be 4 (core/gps_nav.c E0..E3)")
    if int(g.get("usart", 1)) != 1:
        raise GenError("flight_computer.nav.gps.usart must be 1 (the NAV's USART1, PD2 RXD1 / PD3 TXD1)")
    if not gp.yaw_window_s > 0:
        raise GenError("the yaw window must be > 0 s")
    gains = gp.gains()
    cfg = gpn.GpsConfigurator(gp.loop_hz, gp.rate_hz)
    meas_ms = int(round(1000.0 / gp.rate_hz))
    win = 0
    while win * gp.t_step < gp.yaw_window_s - 1e-9:              # the reference's (s_e - m0) t_step >= window - 1e-9
        win += 1
    if win > 0xFFFF or gp.fresh_ticks > 0xFFFFFFFF or gp.gate_stable_ticks > 0xFFFFFFFF:
        raise GenError("GPS tick counts do not fit their C types")
    for k in ("acq_epochs", "reset_after_epochs", "vel_reset_epochs"):
        if not 0 <= getattr(gp, k) <= 0xFFFF:
            raise GenError(f"GPS {k} does not fit uint16")
    if not (0 <= gp.gate_min_sv <= 255 and 0 <= gp.ok_min_sv <= 255 and 1 <= gp.n_corr <= 4):
        raise GenError("GPS satellite counts / n_corr out of range")
    baud = int(g.get("baud", gpn.GPS_BAUD))
    if baud != gpn.GPS_BAUD:
        raise GenError(f"flight_computer.nav.gps.baud {baud} != sim/gps_nav.py GPS_BAUD {gpn.GPS_BAUD} (CFG-PRT)")
    psi0 = -math.radians(gp.pad_azimuth_deg)
    return dict(gp=gp, gains=gains, cfg=cfg, meas_ms=meas_ms, yaw_window_steps=win, baud=baud, psi0=psi0,
                gate_max_hacc_mm=_max_int_le(gp.gate_max_hacc_m, 1000.0),
                gate_max_sacc_cms=_max_int_le(gp.gate_max_sacc_mps, 100.0),
                ok_max_hacc_mm=_max_int_le(gp.ok_max_hacc_m, 1000.0), ubrr_cfg=g.get("ubrr"))


# v0.7.1b: control.vtol_overrides key -> the att_gains_t field core/fc_config.c fc_vtol_gains writes (VTOL_OV_<KEY>)
VTOL_OV_C_FIELDS = {
    "att_kp_xy": "att_kp_xy", "att_kp_z": "att_kp_z", "rate_sp_max_xy_rad_s": "rate_sp_max_xy_rad_s",
    "rate_sp_max_z_rad_s": "rate_sp_max_z_rad_s", "rate_kp_xy": "rate.rate_kp_xy", "rate_ki_xy": "rate.rate_ki_xy",
    "rate_kd_xy": "rate.rate_kd_xy", "rate_i_max_xy": "rate.rate_i_max_xy", "rate_kp_z": "rate.rate_kp_z",
    "rate_ki_z": "rate.rate_ki_z", "rate_kd_z": "rate.rate_kd_z", "rate_i_max_z": "rate.rate_i_max_z",
    "d_term_lpf_hz": "rate.d_term_lpf_hz", "gyro_lpf_hz": "rate.gyro_lpf_hz",
    "thrust_estimate_floor_fraction": "thrust_estimate_floor_fraction", "roll_priority_scale": "roll_priority_scale",
    "yaw_hold": "yaw_hold",
    # v0.9 (sim/controller.py VTOL_EXTRA_KEYS; core/attitude_ctrl.h att_v09_t, filled by core/fc_config.c fc_vtol_gains)
    "att_err_mode": "v9.err_mode", "aw_per_axis": "v9.aw_per_axis", "ff_rotor_h_nms": "v9.ff_h",
    "ff_rotor_tau_s": "v9.ff_tau", "ff_rotor_react": "v9.ff_react", "ff_rotor_gyro": "v9.ff_gyro",
    "mix_yaw_trim_reserve": "v9.yaw_trim_reserve"}
# v0.9 integer switches of control.vtol_overrides (rendered as integers)
VTOL_OV_INT_CHOICES = {"att_err_mode": (0, 1, 2), "aw_per_axis": (0, 1), "ff_rotor_gyro": (0, 1),
                       "mix_yaw_trim_reserve": (0, 1)}
# v0.9 reference options evaluated and NOT flown, not ported to C (0 / absent only; sim/controller.py docstring)
VTOL_OV_NOT_PORTED = ("yaw_i_thrust_norm", "mix_yaw_reserve_deg")


def vtol_att_v09(p: dict, att_ov: dict) -> dict:
    """v0.9 attitude options of the VTOL build (sim/controller.py AttitudeController.__init__, VTOL_EXTRA_KEYS): the
    switches and the rotor feed-forward constants the reference derives once - a = dt / (tau + dt) and the reaction
    gain -react h_ref / tau / roll_priority_scale (one constant on (u - u_lag)), in float64 from the VTOL build's
    merged gains (sim.controller.gains_for_mode: params.control <- sim/out/gains.json <- control.vtol_overrides)."""
    from sim.controller import gains_for_mode
    gpath = os.path.join(ROOT, "sim", "out", "gains.json")
    tuned = None
    if os.path.exists(gpath):
        with open(gpath, "r", encoding="utf-8") as f:
            tuned = json.load(f)
    g = gains_for_mode(p["control"], tuned, lp.MODE_VTOL)
    dt = 1.0 / float(g["loop_hz"])
    h = float(g.get("ff_rotor_h_nms", 0.0))
    tau = float(g.get("ff_rotor_tau_s", 0.15))
    react = float(g.get("ff_rotor_react", 0))
    rps = float(g["roll_priority_scale"])
    k = h / tau / rps if rps > 0.0 else 0.0
    return dict(err_mode=int(g.get("att_err_mode", 0)), aw_per_axis=int(bool(g.get("aw_per_axis", 0))),
                ff_on=int(h != 0.0), ff_h=h, ff_tau=tau, ff_react=react, ff_gyro=int(bool(g.get("ff_rotor_gyro", 0))),
                yaw_trim_reserve=int(bool(g.get("mix_yaw_trim_reserve", 0))), ff_a=dt / (tau + dt),
                ff_nkr=(-react * k) if react else 0.0, rps=rps, loop_hz=int(g["loop_hz"]))


def td_release_threshold(step: float, mx: float) -> float:
    """v0.9 fix round 5b (sim/vertical_ctrl.py): 'still' confirms when td_bias <= -release_max + 1e-9 (float64).  The
    bias moves on the lattice -k step (descending from 0, clamped at -release_max) / -release_max + j step; the float32
    accumulation is a few ulp off that lattice, so the firmware compares with a threshold half way between -release_max
    and the closest lattice value above it (the same decision on every tick)."""
    if not (step > 0.0 and mx > 0.0):
        return -mx
    k = 0
    while (k + 1) * step < mx - 1e-9:
        k += 1
    gap = mx - k * step                                   # the last descending value -k step above -mx (k step < mx - 1e-9)
    if gap < 1e-5:
        raise GenError(f"touchdown release: release_max {mx} is within {gap:.2e} of a multiple of the step {step} - the "
                       "float32 accumulation cannot tell the reference's 1e-9 tolerance apart (choose another value)")
    return -mx + 0.5 * min(gap, step)


def vertical_v09(vp, nm: str) -> dict:
    """v0.9 vertical-controller constants of one profile (sim/vertical_ctrl.py docstrings 1b, 1c, 3b, 4c, 5b)."""
    rt = vp.ramp_total_ticks
    if not 0 <= rt <= 0xFFFF:
        raise GenError(f"vertical controller ({nm}): ramp_total_ticks {rt} does not fit uint16")
    if vp.to_u_start > 0.0 and vp.ramp_ticks > 0 and vp.cap_at(rt - 1) != vp.u_max:
        raise GenError(f"vertical controller ({nm}): the two-slope ceiling at k + 1 = ramp_total_ticks {rt} is "
                       f"{vp.cap_at(rt - 1)!r}, not u_max - core/vertical_ctrl.c returns u_max there without arithmetic")
    if not 0 <= vp.ref_approach_shift <= 31:
        raise GenError(f"mission.vtol.ref_approach_shift {vp.ref_approach_shift} outside 0..31")
    step = vp.td_release_per_s * vp.dt
    two = vp.to_u_start > 0.0 and vp.ramp_ticks > 0
    return dict(rt=rt, two=two, u_start=vp.to_u_start, s1=(vp.to_u_start / vp.ramp_ticks) if two else 0.0,
                s2=(vp.to_slew_per_s / vp.loop_hz) if two else 0.0,
                boost=vp.to_boost_per_s > 0.0 and vp.takeoff_ticks > 0, search_f=vp.to_search_factor,
                search_vz=vp.to_search_vz_mps, kt_alpha_to=vp.kt_alpha_to,
                liftoff=(vp.kt_alpha_to > 0.0 or vp.to_cap_margin > 0.0 or vp.to_ref_hold),
                airborne_mps2=vp.g * (1.0 + vp.to_airborne_fz), ref_hold=bool(vp.to_ref_hold) and rt > 0,
                approach_shift=int(vp.ref_approach_shift), approach_min_um=int(vp.approach_min_um),
                land_rebase=bool(vp.land_ref_rebase), td_rel=vp.td_release_per_s > 0.0, td_step=step,
                td_max=vp.td_release_max, td_ok=td_release_threshold(step, vp.td_release_max))


PWR_COUNTER_LIMITS = {"pw_lost_ticks": 0xFF, "pw_buf_ticks": 0xFF, "pw_d2_ticks": 0xFF, "pw_sag_release_ticks": 0xFF,
                      "pw_sag_warn_ticks": 0xFFFF, "pw_sag_land_ticks": 0xFFFF, "pw_buf_res_ticks": 0xFFFF}
# v0.9 power ADC / GPIO pins: params flight_computer.pins v_tether_adc / v_buffer_adc / buf_isign (fw09 SPEC 5; the pin
# strings say ASSUMED until the v0.9 carrier PCB is routed), claimed with the FCU's pin table (chip_pins)


def vaux_integer_form(v_per_count: float) -> tuple[int, int, int]:
    """sim/power_link.py quant_v(y v_per_count / 16) for every Q4 code y in 0 .. 16 x 1023 as min((y mul + add) >>
    shift, 255) (exhaustively verified; the FCU's TELEM v4 / v5 v_aux byte)."""
    from sim import power_link as pl
    want = [pl.quant_v(y * v_per_count / 16.0) for y in range(16 * 1023 + 1)]
    for shift in range(8, 25):
        mul = round(v_per_count / 16.0 * pl.V_SCALE * (1 << shift))
        for add in (1 << (shift - 1), (1 << (shift - 1)) - 1, (1 << (shift - 1)) + 1):
            if mul * 16368 + add < (1 << 32) and all(min((y * mul + add) >> shift, 255) == q for y, q in enumerate(want)):
                return mul, add, shift
    raise GenError(f"power tail v_aux: no exact integer form for adc_v_per_count {v_per_count}")


def power_v09(spv, nm: str, loop_hz: int, vbat_v_per_count: float) -> dict | None:
    """v0.9 power rules of one profile (sim/onboard.py PowerMonitor / power_fields / tether_limits; mission.vtol.
    power_rules_ASSUMED): the Q4 codes and tick counts exactly as the reference holds them, the tethered limits of the
    wired builds (build constants), the v_aux integer form.  None without the params block (battery only)."""
    from sim.onboard import tether_limits
    if not spv.power_rules:
        return None
    d = {k: getattr(spv, k) for k in ("pw_shift", "pw_arm_lo_q", "pw_arm_hi_q", "pw_buf_arm_lo_q", "pw_buf_arm_hi_q",
                                      "pw_lost_q", "pw_lost_margin_q", "pw_lost_ticks", "pw_sag_q", "pw_sag_clear_q",
                                      "pw_sag_warn_ticks", "pw_sag_land_ticks", "pw_sag_release_ticks", "pw_buf_low_q",
                                      "pw_buf_ticks", "pw_buf_res_q", "pw_buf_res_ticks", "pw_d2_q", "pw_d2_ticks",
                                      "pw_i8buf_q", "pw_v_per_count")}
    for k, lim in PWR_COUNTER_LIMITS.items():
        if not 1 <= d[k] <= lim:
            raise GenError(f"power_rules_ASSUMED ({nm}): {k} = {d[k]} ticks does not fit the C counter (1..{lim})")
    for k, v in d.items():
        if k.endswith("_q") and not 0 <= v <= 16 * 1023:
            raise GenError(f"power_rules_ASSUMED ({nm}): {k} = {v} outside the Q4 range of a 10-bit code")
    if not 0 <= d["pw_shift"] <= 8:
        raise GenError(f"power_rules_ASSUMED ({nm}): filter_shift {d['pw_shift']} outside 0..8")
    if abs(d["pw_v_per_count"] - vbat_v_per_count) > 1e-12:
        raise GenError(f"power_rules_ASSUMED ({nm}): adc_v_per_count {d['pw_v_per_count']} != the FCU's VBAT_V_PER_COUNT "
                       f"{vbat_v_per_count} - the Q4 thresholds would not be the ADC's codes")
    lim = tether_limits(spv, tethered=True)
    if not (0 < lim.endurance_ticks < 0xFFFFFFFF and 0 < lim.hard_cutoff_ticks < 0xFFFFFFFF):
        raise GenError(f"power_rules_ASSUMED ({nm}): tethered endurance / hard cutoff ticks do not fit uint32")
    d["lim"] = lim
    d["vaux"] = vaux_integer_form(spv.pw_v_per_count)
    return d


def yaw_window_steps(gp) -> int:
    """The smallest d with d t_step >= yaw_window_s - 1e-9 (the reference's window test, core/gps_nav.h)."""
    win = 0
    while win * gp.t_step < gp.yaw_window_s - 1e-9:
        win += 1
    return win


def gps_c_fields(gp) -> dict:
    """Every gps_nav_params_t field (core/gps_nav.h) of a sim.gps_nav.NavGpsParams, as the firmware holds it (float32
    values, the integer bounds of the thresholds, the fixed gains of compute_gains) - what core/fc_config.c
    fc_default_gps_nav fills from the GPS_* macros of _render_gps; the host test / cross-check pass it as 'gp.<key>='
    overrides (tools/build_host_test.py, tools/crosscheck_controller.py)."""
    g = gp.gains()
    psi0 = -math.radians(gp.pad_azimuth_deg)
    d = {"step_ticks": gp.step_ticks, "ms_per_tick": gp.ms_per_tick, "latency_ms": gp.latency_ms,
         "loop_hz_f": f32(float(gp.loop_hz)), "inv_step": f32(1.0 / gp.step_ticks), "t_step": f32(gp.t_step),
         "psi0": f32(psi0), "c_psi0": f32(math.cos(psi0)), "s_psi0": f32(math.sin(psi0)),
         "gate_min_sv": gp.gate_min_sv, "gate_max_hacc_mm": _max_int_le(gp.gate_max_hacc_m, 1000.0),
         "gate_max_sacc_cms": _max_int_le(gp.gate_max_sacc_mps, 100.0), "gate_stable_ticks": gp.gate_stable_ticks,
         "ok_min_sv": gp.ok_min_sv, "ok_max_hacc_mm": _max_int_le(gp.ok_max_hacc_m, 1000.0),
         "gate_v2": f32(gp.gate_sigma * gp.gate_sigma * g.s_vv), "gate_p2": f32(gp.gate_p_m * gp.gate_p_m),
         "n_corr": gp.n_corr, "acq_epochs": gp.acq_epochs, "reset_after_epochs": gp.reset_after_epochs,
         "vel_reset_epochs": gp.vel_reset_epochs, "fresh_ticks": gp.fresh_ticks, "yaw_correction": int(gp.yaw_correction),
         "yaw_window_steps": yaw_window_steps(gp), "yaw_min_dv2": f32(gp.yaw_min_dv_mps ** 2),
         "yaw_sigma_dv2": f32(gp.yaw_sigma_dv_mps ** 2), "yaw_q_rad2": f32(math.radians(1.0) ** 2 * gp.yaw_q_deg2_per_s),
         "p_psi0": f32(math.radians(gp.yaw_sigma0_deg) ** 2), "head_ok_rad2": f32(math.radians(gp.head_ok_deg) ** 2),
         "yaw_sigma_rel2": f32(gp.yaw_sigma_rel * gp.yaw_sigma_rel)}
    for i in range(3):
        d[f"lever_b{i}"] = f32(gp.lever_b_m[i] if gp.lever_comp else 0.0)
    for tag, gs in (("s", g.steady), ("a", g.acq)):
        for r in range(3):
            for c in range(2):
                d[f"{tag}_kpv{r}{c}"] = f32(gs.k_pv[r][c])
            d[f"{tag}_kp{r}"] = f32(gs.k_p[r])
            d[f"{tag}_kv{r}"] = f32(gs.k_v[r])
    return d


POS_C_FLOAT_FIELDS = ("T", "g", "tw", "kp_pos", "kv", "ki", "kd", "alpha_d", "i_max", "v_max", "v_max_land", "flare_speed",
                      "flare_altitude_m", "cos_cap", "cos_flare_cap", "cos_excess", "sin_excess", "margin_g", "margin_frac",
                      "h_enable_m", "fence_m", "spin_z_rad", "spin_xy_rad2", "authority_min_ktw", "tilt_fade_m",
                      "slew_per_tick")
POS_C_INT_FIELDS = ("step_ticks", "guard_steps", "timeout_steps", "land_steps", "plaus_limited_steps", "plaus_vexc_steps",
                    "plaus_reset_window_steps", "plaus_resets")


def pos_c_fields(pp) -> dict:
    """Every pos_params_t base field (core/position_ctrl.h; the derived ones are pos_params_derive's) of a
    sim.position_ctrl.PositionParams as the firmware holds it - the POS_* macros of _render_vtol_profile."""
    d = {k: f32(getattr(pp, k)) for k in POS_C_FLOAT_FIELDS}
    d.update({k: int(getattr(pp, k)) for k in POS_C_INT_FIELDS})
    d["plaus_vt"] = f32(pp.v_max + pp.plaus_v_excess_mps)
    d["keep_trim"] = POS_TD_POLICY_CODES[pp.touchdown_policy]     # v0.9: the policy code (0 fade, 1 keep_trim, 2 keep_vel)
    return d


# v0.9: position_hold.touchdown_policy.policy -> pos_params_t.keep_trim (core/position_ctrl.h POS_TD_*; the field keeps its
# v0.8.1 name and its values 0 / 1, keep_vel is 2)
POS_TD_POLICY_CODES = {"fade": 0, "keep_trim": 1, "keep_vel": 2}


GLINK_LATENCY_RANGE = (2, 16)                 # core/nlink_proto.h NL_GNS_L_MIN / NL_GNS_L_MAX (the 4-entry result ring)


def derive_glink(p: dict, f_cpu: int, loop_hz: int, gp, warnings: list[str]) -> dict:
    """v0.8.1 NAV <-> GNS glink (params flight_computer.glink / gns): the UART setting (U2X = 0, like the ilink: both chips
    run the same UBRR from 16 MHz crystals; simavr ignores U2X), the latency L, the GNS cycle and the frame times; the
    reference's arrival estimate (sim.glink_proto.timing_estimate) is recorded with its margin at L."""
    from sim import glink_proto as glp
    fc = p["flight_computer"]
    g, gns = fc["glink"], fc.get("gns")
    if not isinstance(gns, dict):
        raise GenError("flight_computer.glink needs flight_computer.gns (the board at its other end)")
    if gns.get("mcu") not in MCU_NAMES:
        raise GenError(f"flight_computer.gns.mcu {gns.get('mcu')!r}: the core is built for the ATmega128(A)")
    if int(gns.get("f_cpu_hz", f_cpu)) != f_cpu:
        raise GenError(f"flight_computer.gns.f_cpu_hz {gns.get('f_cpu_hz')} != f_cpu_hz {f_cpu}: params_gen.h carries one "
                       "F_CPU (the UBRR / timer settings of every board)")
    baud = int(g["baud"])
    if baud != glp.BAUD:
        raise GenError(f"flight_computer.glink.baud {baud} != sim/glink_proto.py BAUD {glp.BAUD}")
    x = f_cpu / (16 * baud) - 1
    cands = [u for u in (math.floor(x), math.ceil(x)) if 0 <= u <= 4095]
    if not cands:
        raise GenError(f"flight_computer.glink.baud {baud}: UBRR {x:.2f} outside 0..4095 at {f_cpu} Hz (U2X = 0)")
    ubrr = min(cands, key=lambda u: (abs(f_cpu / (16 * (u + 1)) / baud - 1.0), u))
    err = f_cpu / (16 * (ubrr + 1)) / baud - 1.0
    if abs(err) > ILINK_MAX_ERR:
        raise GenError(f"flight_computer.glink.baud {baud}: UBRR {ubrr} (U2X = 0) is off by {err * 100:+.2f} %")
    if g.get("ubrr") is not None and int(g["ubrr"]) != ubrr:
        raise GenError(f"flight_computer.glink.ubrr {g['ubrr']} != the computed UBRR {ubrr} (U2X = 0) for {baud} baud")
    lat = int(g.get("latency_ticks_ASSUMED", glp.LATENCY_TICKS_DEFAULT))
    if not GLINK_LATENCY_RANGE[0] <= lat <= GLINK_LATENCY_RANGE[1]:
        raise GenError(f"flight_computer.glink.latency_ticks_ASSUMED {lat} outside {GLINK_LATENCY_RANGE} (the GNS answers "
                       "in the NAV's next tick at the earliest; the NAV's 4-entry result ring holds 16 ticks)")
    gns_hz = int(gns.get("loop_hz", loop_hz // gp.step_ticks))
    if gns_hz * gp.step_ticks != loop_hz or gp.step_ticks != glp.STEP_TICKS:
        raise GenError(f"flight_computer.gns.loop_hz {gns_hz} x the GPS step ticks {gp.step_ticks} != control.loop_hz "
                       f"{loop_hz}: one STATE / RESULT pair per {glp.STEP_TICKS} NAV ticks")
    state_s = glp.frame_time_s(glp.STATE_FRAME_BYTES, baud)
    result_s = glp.frame_time_s(glp.RESULT_FRAME_BYTES, baud)
    if state_s + result_s >= 1.0 / gns_hz:
        raise GenError(f"glink: STATE {state_s * 1e3:.2f} ms + RESULT {result_s * 1e3:.2f} ms do not fit the "
                       f"{1e3 / gns_hz:g} ms GNS cycle")
    est = glp.timing_estimate(p)
    if est["margin_ms_at_params_L"] < 0.0:
        warnings.append(f"glink: the arrival ESTIMATE (sim/glink_proto.py timing_estimate, {est['result_at_nav_ms']:.2f} ms "
                        f"after the sample tick) misses the frame build of k + {lat} by {-est['margin_ms_at_params_L']:.2f} "
                        "ms - the co-simulation must measure it")
    return dict(baud=baud, ubrr=ubrr, err=err, latency=lat, gns_hz=gns_hz, state_s=state_s, result_s=result_s, est=est)


def nav_push_min_us_vtol(path: str = NAV_H) -> int:
    """src/nav/nav.h NAV_PUSH_MIN_US_VTOL: the VTOL NAV's push floor (re-derived by tools/run_nav_simbench.py)."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            m = re.search(r"^#define\s+NAV_PUSH_MIN_US_VTOL\s+(\d+)[uUlL]*\b", f.read(), re.M)
    except OSError as exc:
        raise GenError(f"cannot read {rel(path)} (NAV_PUSH_MIN_US_VTOL, the W1 watchdog check): {exc}") from None
    if not m:
        raise GenError(f"{rel(path)}: #define NAV_PUSH_MIN_US_VTOL not found (the W1 watchdog check needs the push floor)")
    return int(m.group(1))


def nlink_watchdog(nlk: dict, f_cpu: int, xfer_s: float, tick_us: int, nav_h: str = NAV_H) -> dict:
    """fw09 W1 (SPEC_firmware_v09 3.3 / 5): flight_computer.nlink.exchange_watchdog_us -> NLINK_WD_US / NLINK_WD_COUNTS
    (Timer3 at clk / NLINK_WD_TIMER3_PRESCALER, restarted at 0 by every exchange).  Checks: a whole number of
    microseconds and of Timer3 counts below the 16-bit overflow; > NLINK_WD_XFER_FACTOR x NLINK_XFER_US (a normal
    exchange never trips it); < FC_TICK_US - NAV_PUSH_MIN_US_VTOL (a tripped exchange still ends inside its tick)."""
    if "exchange_watchdog_us" not in nlk:
        raise GenError("flight_computer.nlink.exchange_watchdog_us missing (fw09 W1, the NAV's exchange watchdog)")
    raw = nlk["exchange_watchdog_us"]
    if isinstance(raw, bool) or not isinstance(raw, (int, float)) or not math.isfinite(float(raw)):
        raise GenError(f"flight_computer.nlink.exchange_watchdog_us {raw!r} is not a number")
    wd = Fraction(str(raw))
    if wd.denominator != 1 or wd <= 0:
        raise GenError(f"flight_computer.nlink.exchange_watchdog_us {raw!r} must be a whole number of us > 0")
    wd_us = int(wd)
    counts = Fraction(wd_us * f_cpu, 1_000_000 * NLINK_WD_TIMER3_PRESCALER)
    if counts.denominator != 1 or not 0 < counts <= 0xFFFF:
        raise GenError(f"flight_computer.nlink.exchange_watchdog_us {wd_us}: {float(counts):g} Timer3 counts (clk / "
                       f"{NLINK_WD_TIMER3_PRESCALER}) is not a whole number in 1..65535 (the watchdog compares TCNT3)")
    xfer_us = math.ceil(xfer_s * 1e6)
    if not wd_us > NLINK_WD_XFER_FACTOR * xfer_us:
        raise GenError(f"flight_computer.nlink.exchange_watchdog_us {wd_us} must exceed {NLINK_WD_XFER_FACTOR:g} x the "
                       f"exchange NLINK_XFER_US {xfer_us} us = {NLINK_WD_XFER_FACTOR * xfer_us:g} us (a normal exchange "
                       "would trip it)")
    push = nav_push_min_us_vtol(nav_h)
    if not wd_us < tick_us - push:
        raise GenError(f"flight_computer.nlink.exchange_watchdog_us {wd_us} must stay below FC_TICK_US {tick_us} - "
                       f"NAV_PUSH_MIN_US_VTOL {push} = {tick_us - push} us (src/nav/nav.h): a tripped exchange must end "
                       "inside its tick")
    return dict(us=wd_us, counts=int(counts), xfer_us=xfer_us, push_us=push, limit_us=tick_us - push)


def ggh_pad_tau() -> float:
    """The estimator's pad-averaging time constant gains.h carries (tools/gen_gains_header.py resolve)."""
    tools = os.path.join(ROOT, "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import gen_gains_header as ggh  # noqa: E402  (tools/)
    return float(ggh.resolve()["gains"]["est_pad_avg_tau_s"])


class VcfgUnavailable(GenError):
    """fw09 G1: a VTOL configuration without a usable CAD report (or without params.json configs.vtol): its builds get an
    #error in the header, the other configurations are generated."""


def vtol_report_path(vcfg: str, root: str = ROOT) -> str:
    """The CAD report sim.vehicle.load_vtol_vehicle reads for a VTOL configuration of the default preset."""
    stem, ext = os.path.splitext(vm.CAD_REPORT_CANDIDATES[0])
    return os.path.join(root, f"{stem}{layout_mod.config_suffix('vtol', vcfg)}{ext}")


def derive(params_path: str = PARAMS, cad_path: str = CAD_REPORT, cfg_path: str = FC_CONFIG,
           vtol_root: str | None = None) -> dict:
    """Every number of params_gen.h, in SI, plus provenance.  Raises GenError on any inconsistency.

    The top-level dict is the ROCKET set (params.json as is, ``cad_path``): what every ballistic build and the host builds
    without -D FC_VCFG fly - the v0.8.1 numbers.  fw09 G1: ``d['vcfg'][name]`` (name in VCFGS) is the same derivation for
    the VTOL configuration ``name`` exactly as sim.vehicle.load_vehicle(config='vtol', power=name) builds the vehicle the
    6-DOF flies (params with configs.vtol merged, the configuration's CAD report under ``vtol_root``, default the project);
    ``d['vcfg_errors'][name]`` says why a configuration is unavailable (its builds get an #error).  Each set carries its own
    ``params_tag``; ``sources`` (shared) names every CAD report."""
    d = _derive_one(params_path, cad_path, cfg_path, None)
    # the rocket set's VTOL block serves only host builds without FC_VCFG: say so in its VTOL warnings
    d["warnings"] = [f"rocket set (host builds without FC_VCFG only): {w_}" if w_.startswith(("VTOL:", "VTOL (")) else w_
                     for w_ in d["warnings"]]
    d["vcfg"], d["vcfg_errors"] = {}, {}
    if d["vt"] is None:                                   # no mission.vtol (a v0.6.3 params.json): ballistic only
        for nm in VCFGS:
            d["vcfg_errors"][nm] = "params.json has no mission.vtol"
    else:
        for nm in VCFGS:
            try:
                d["vcfg"][nm] = _derive_one(params_path, None, cfg_path, nm, vtol_root or ROOT)
            except VcfgUnavailable as exc:
                d["vcfg_errors"][nm] = str(exc)
                d["warnings"].append(f"VTOL {nm}: {exc} - the FC_VCFG={VCFG_ID[nm]} builds will not compile")
    for nm, dc in d["vcfg"].items():                      # one warning list (the reference's warnings once)
        for w_ in dc["warnings"]:
            tagged = w_ if w_.startswith("VTOL ") else f"VTOL {nm}: {w_}"
            if w_ not in d["warnings"] and tagged not in d["warnings"]:
                d["warnings"].append(tagged)
    # FC_PARAMS_TAG per configuration: the rocket keeps the low byte of the params sha256 (unchanged definition); a VTOL
    # configuration takes the low byte of sha256(params sha | vtol_<name> | its CAD report's canonical sha | k) with the
    # smallest k that differs from every tag assigned before it (rocket, battery, tether, hybrid): all tags distinct
    psha = d["sources"][0][1]
    used = {d["params_tag"]}
    for nm in VCFGS:
        if nm not in d["vcfg"]:
            continue
        dc, k = d["vcfg"][nm], 0
        while True:
            t = int(hashlib.sha256(f"{psha}|vtol_{nm}|{dc['cad_sha']}|{k}".encode()).hexdigest()[:16], 16) & 0xFF
            if t not in used:
                break
            k += 1
        dc["params_tag"], dc["params_tag_k"] = t, k
        used.add(t)
    # sources: the rocket's list with every VTOL configuration's CAD report after the rocket report
    src = list(d["sources"])
    i_cad = next(i for i, (pth, _) in enumerate(src) if pth.endswith(" (canonical)"))
    for j, nm in enumerate(VCFGS):
        rp = vtol_report_path(nm, vtol_root or ROOT)
        if nm in d["vcfg"]:
            rp = d["vcfg"][nm]["cad_path"]
        src.insert(i_cad + 1 + j, (rel(rp) + " (canonical)",
                                   cad_report_sha256(rp) if os.path.exists(rp) else "absent"))
    d["sources"] = src
    for dc in d["vcfg"].values():
        dc["sources"] = src
    return d


def _derive_one(params_path: str, cad_path: str | None, cfg_path: str, vcfg: str | None = None,
                vtol_root: str = ROOT) -> dict:
    """One constant set.  ``vcfg`` None: the rocket (params.json as is, the CAD report ``cad_path``, the generator's own
    composition - the v0.8.1 numbers).  ``vcfg`` 'battery' / 'tether' / 'hybrid' (fw09 G1): params.json with configs.vtol
    merged and power.source = vcfg (cad/layout.py load_params, as sim.vehicle.load_vehicle(config='vtol', power=vcfg)),
    the vehicle sim.vehicle.load_vtol_vehicle builds from that configuration's CAD report (the 6-DOF's mass properties,
    the same refusals: preset / config / power source, failed or missing components) - VcfgUnavailable when there is
    none."""
    if vcfg is None:
        p = layout_mod.load_params(params_path)
    else:
        try:
            p = layout_mod.load_params(params_path, None, "vtol", vcfg)
        except (KeyError, ValueError) as exc:
            raise VcfgUnavailable(f"params.json cannot load the VTOL {vcfg} configuration: {exc}") from None
    L = layout_mod.Layout(p)
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    fc, ctl, ms, sv, vn, edf, bat = (p["flight_computer"], p["control"], p["mission"], p["servo"], p["vane"],
                                     p["edf"], p["battery"])
    warnings: list[str] = []

    # ---- mass properties (CAD report, launch pad excluded) --------------------
    if vcfg is None:
        with open(cad_path, "r", encoding="utf-8") as f:
            cad = json.load(f)
        parts = cad.get("parts")
        if not isinstance(parts, dict) or not parts:
            raise GenError(f"{rel(cad_path)} has no parts")
        m_full, c_full, i_full = compose(parts, NON_VEHICLE_PARTS)
        m_post, c_post, _ = compose(parts, NON_VEHICLE_PARTS + EJECTED_PARTS)
        tot = cad.get("totals", {})
        if "mass_g" in tot and abs(float(tot["mass_g"]) * 1e-3 - m_full) > 0.02 * m_full:
            warnings.append(f"CAD totals.mass_g {tot['mass_g']:.0f} g differs from the parts sum {m_full * 1e3:.0f} g; parts sum used")
        if cad.get("params_sha256") and cad["params_sha256"] != sha256(params_path):
            warnings.append("cadquery_report.json was built from a different params.json (params_sha256 differs) - rebuild the CAD")
    else:
        try:
            veh = vm.load_vtol_vehicle(vtol_root, p, params_path, vcfg)
        except ValueError as exc:
            raise VcfgUnavailable(f"{exc} (expected {rel(vtol_report_path(vcfg, vtol_root))})") from None
        if abs(float(veh.layout.vane_hinge_z) - float(L.vane_hinge_z)) > 1e-12:
            raise GenError(f"VTOL {vcfg}: the vehicle's vane hinge differs from the configuration's layout")
        cad_path = veh.source_path
        with open(cad_path, "r", encoding="utf-8") as f:
            cad = json.load(f)
        m_full = float(veh.full.mass_kg)
        c_full = [float(x) for x in veh.full.com_m]
        i_full = [[float(x) for x in row] for row in veh.full.inertia_kg_m2]
        m_post = float(veh.post_ejection.mass_kg)
        c_post = [float(x) for x in veh.post_ejection.com_m]
        warnings.extend(f"VTOL {vcfg}: {w_}" for w_ in veh.warnings)
    z_cg = c_full[2]
    hinge_z = L.vane_hinge_z * 1e-3
    l_v = z_cg - hinge_z
    if l_v <= 0:
        raise GenError(f"CG {z_cg:.3f} m is below the vane hinge {hinge_z:.3f} m")

    # ---- plant (SPEC 8.3-8.5) ------------------------------------------------
    d = L.duct_id * 1e-3
    hub = float(edf["exit_hub_fraction"]) * d
    a_e = math.pi / 4.0 * (d * d - hub * hub)
    s_v = L.vane_area_mm2 * 1e-6
    r_v = float(vn["force_centroid_r"]) * 1e-3
    t_max = float(edf["thrust_nominal_N"])
    n_exp = float(edf["throttle_exponent"])
    n_exp_int = int(n_exp) if n_exp == int(n_exp) and 1 <= n_exp <= 4 else 0
    rot = str(edf["rotation"]).upper()
    swirl_sign = -1.0 if "CCW" in rot else (1.0 if "CW" in rot else None)
    if swirl_sign is None:
        raise GenError(f"edf.rotation {edf['rotation']!r} must say CW or CCW")
    omega_max = float(edf["kv"]) * float(edf["voltage_nominal_V"]) * 2.0 * math.pi / 60.0
    hover_u = (m_full * G0 / t_max) ** (1.0 / n_exp)
    if hover_u >= 1.0:
        raise GenError(f"hover throttle {hover_u:.3f} >= 1: vehicle cannot lift off")
    thetas = list(L.vane_thetas)
    if len(thetas) != 4 or any(abs(((t - thetas[0]) - 90.0 * i)) > 1e-9 for i, t in enumerate(thetas)):
        raise GenError(f"mixer assumes 4 vanes at 90 deg steps, Layout.vane_thetas = {thetas}")
    if abs(thetas[0]) > 1e-9:
        raise GenError("SPEC 8.1: body x points to vane 1 (vane.theta0_deg must be 0)")

    # ---- servo / PWM --------------------------------------------------------
    f_cpu = int(fc["f_cpu_hz"])
    presc = int(fc["pwm_timer_prescaler"])
    top = int(fc["pwm_top"])
    if presc not in TIMER13_PRESCALERS:
        raise GenError(f"pwm_timer_prescaler {presc} not available on Timer1/3")
    frame_hz = Fraction(f_cpu, presc * (top + 1))
    if frame_hz != int(fc["servo_frame_hz"]) or frame_hz != int(sv["pwm_hz"]):
        raise GenError(f"PWM frame {float(frame_hz)} Hz != servo_frame_hz {fc['servo_frame_hz']} / servo.pwm_hz {sv['pwm_hz']}")
    counts_per_us = Fraction(f_cpu, presc * 1_000_000)
    if counts_per_us.denominator != 1:
        raise GenError(f"PWM tick {1 / float(counts_per_us)} us is not 1/integer us")
    counts_per_us = int(counts_per_us)
    us_per_deg = float(sv["pwm_us_per_deg"])
    center = float(sv["pwm_center_us"])
    signs = [float(s) for s in cfg["servo_signs"]]
    trims = [float(t) for t in cfg["servo_trims_us"]]
    if len(signs) != 4 or any(s not in (-1.0, 1.0) for s in signs) or len(trims) != 4:
        raise GenError("fc_config servo_signs must be 4 x +-1, servo_trims_us 4 numbers")
    mech = float(vn["max_deflection_mech_deg"])
    cmd = float(vn["max_deflection_cmd_deg"])
    if cmd > mech:
        raise GenError("vane.max_deflection_cmd_deg exceeds the mechanical limit")
    vane_lo = [center + t - mech * us_per_deg for t in trims]
    vane_hi = [center + t + mech * us_per_deg for t in trims]
    rel_cfg = cfg["release_servo"]
    rs = float(rel_cfg["sign"])
    latched = center + rs * float(rel_cfg["latched_offset_deg"]) * us_per_deg
    released = latched + rs * float(ms["deploy_servo_travel_deg"]) * us_per_deg
    esc = cfg["esc_us"]
    period_us = 1e6 / float(frame_hz)
    for name, us in [("release latched", latched), ("release released", released), ("esc off", esc["off"]),
                     ("esc max", esc["max"])] + [(f"vane {i + 1} lo", v) for i, v in enumerate(vane_lo)] + \
                    [(f"vane {i + 1} hi", v) for i, v in enumerate(vane_hi)]:
        if not 500.0 <= float(us) <= 2500.0 or float(us) >= period_us:
            raise GenError(f"{name} pulse {us:.1f} us outside 500..2500 us")
    if not esc["off"] <= esc["min"] < esc["max"]:
        raise GenError("esc_us must satisfy off <= min < max")

    # ---- pins: each chip on its own (FCU flight_computer.pins, COM flight_computer.com.pins) ----
    pins = fc["pins"]
    com = fc.get("com")
    if not isinstance(com, dict):
        raise GenError("flight_computer.com missing (v0.6: the LoRa radio runs on the COM ATmega128A, SPEC 10.4)")
    nav = fc.get("nav")
    if not isinstance(nav, dict):
        raise GenError("flight_computer.nav missing (v0.6.3: the sensors and the estimator run on the NAV ATmega128A, "
                       "SPEC 10.5)")
    fcu_pins, pwm_map = chip_pins("fcu", pins)
    for key in ("v_tether_adc", "v_buffer_adc", "logic_bat_adc"):   # v0.9 power monitor ADC inputs (fw09 SPEC 5), logic bat
        if fcu_pins[key][0] != "F" or not 0 <= fcu_pins[key][1] <= 3:
            raise GenError(f"flight_computer.pins.{key} = {pins[key]!r}: the power ADC inputs must be PF0..PF3 (PF4-PF7 "
                           "are the JTAG pins)")
    nav_pins, _ = chip_pins("nav", nav.get("pins"))
    gns_pins = chip_pins("gns", fc["gns"].get("pins"))[0] if isinstance(fc.get("gns"), dict) else None
    com_pins, _ = chip_pins("com", com.get("pins"))
    for name, mcu in (("flight_computer.mcu", fc.get("mcu", MCU_NAMES[-1])), ("flight_computer.nav.mcu", nav.get("mcu")),
                      ("flight_computer.com.mcu", com.get("mcu"))):
        if mcu not in MCU_NAMES:
            raise GenError(f"{name} = {mcu!r}: this firmware and its pin check are for {' / '.join(MCU_NAMES)}")
    for name, chip in (("flight_computer.nav.f_cpu_hz", nav), ("flight_computer.com.f_cpu_hz", com)):
        if int(chip["f_cpu_hz"]) != f_cpu:
            raise GenError(f"{name} {chip['f_cpu_hz']} != flight_computer.f_cpu_hz {f_cpu}: one params_gen.h (FC_F_CPU_HZ, "
                           "every UBRR / tick count / SPI divider) and one platformio board_build.f_cpu serve every chip")
    mv = re.search(r"ADC([0-7]).*divider\s*([\d.]+)\s*k\s*/\s*([\d.]+)\s*k.*?([\d.]+)\s*V\s*ref", pins["vbat_adc"])
    if not mv:
        raise GenError(f"cannot parse flight_computer.pins.vbat_adc {pins['vbat_adc']!r}")
    adc_ch, r_top, r_bot, vref = int(mv.group(1)), float(mv.group(2)) * 1e3, float(mv.group(3)) * 1e3, float(mv.group(4))
    if fcu_pins["vbat_adc"] != ("F", adc_ch):
        raise GenError(f"vbat_adc pin {pins['vbat_adc']!r} is not PF{adc_ch}")
    if abs(vref - 2.56) > 1e-9:
        raise GenError("the ATmega128 internal reference is 2.56 V; vbat_adc must say so")
    vbat_scale = float(cfg.get("vbat_scale", 1.0))
    if not 0.8 <= vbat_scale <= 1.2:
        raise GenError(f"fc_config vbat_scale {vbat_scale} outside 0.8..1.2 - check the divider")
    vbat_per_count = vref / 1024.0 * (r_top + r_bot) / r_bot * vbat_scale
    v_adc_full = int(bat["cells"]) * LIPO_V_MAX_CELL * r_bot / (r_top + r_bot)
    if v_adc_full >= vref:
        raise GenError(f"full pack gives {v_adc_full:.2f} V at the ADC pin >= Vref {vref} V")
    # 2026-11-06: the logic-battery monitor (flight_computer.logic_battery_monitor; carrier rev 2c PF3, 47k / 47k): scale
    # and pin.  2026-11-10: the VTOL battery FCU reads it (hal/adc.h FC_LOGIC_BAT) and sends it in TELEM v6 / v7; the
    # rocket FCU (no divider) and the power builds do not
    lbm = fc.get("logic_battery_monitor")
    lbat = None
    if isinstance(lbm, dict):
        lrt, lrb, lvref = float(lbm["r_top_ohm"]), float(lbm["r_bottom_ohm"]), float(lbm["vref_v"])
        if abs(lvref - 2.56) > 1e-9:
            raise GenError("flight_computer.logic_battery_monitor.vref_v: the ATmega128 internal reference is 2.56 V")
        if LIPO_V_MAX_CELL * lrb / (lrt + lrb) >= lvref:
            raise GenError(f"logic battery: a full 1S cell gives {LIPO_V_MAX_CELL * lrb / (lrt + lrb):.2f} V at PF"
                           f"{fcu_pins['logic_bat_adc'][1]} >= Vref {lvref} V")
        lbat = {"ch": fcu_pins["logic_bat_adc"][1], "v_per_count": lvref / 1024.0 * (lrt + lrb) / lrb,
                "warn_v": float(lbm["ground_warn_below_v_ASSUMED"]), "r_top": lrt, "r_bot": lrb,
                # 2026-11-10: the TELEM v6 / v7 byte from the code in integers (sim.link_proto.lbat_q_from_counts):
                # lbat = (counts x Q_X1024 + 512) >> 10, Q_X1024 = round(v_per_count x LBAT_SCALE x 1024)
                "telem": lp.lbat_build(p, lp.MODE_VTOL),
                "q_x1024": int(round(lvref / 1024.0 * (lrt + lrb) / lrb * lp.LBAT_SCALE * 1024.0))}
        if lbat["q_x1024"] <= 0 or abs(lbat["q_x1024"] / 1024.0 - lbat["v_per_count"] * lp.LBAT_SCALE) > 1e-3:
            raise GenError(f"logic battery: LOGIC_BAT_Q_X1024 {lbat['q_x1024']} does not represent "
                           f"{lbat['v_per_count'] * lp.LBAT_SCALE:.6f} lbat steps per count within 0.1 %")
    adc_ps = next((bits for div, bits in sorted(ADC_PRESCALERS.items()) if f_cpu / div <= ADC_CLK_MAX_HZ), None)
    adc_div = next(div for div, bits in ADC_PRESCALERS.items() if bits == adc_ps)

    # ---- timing --------------------------------------------------------------
    loop_hz = int(ctl["loop_hz"])
    t2 = None
    for pre in sorted(TIMER2_PRESCALERS):
        q = Fraction(f_cpu, pre * loop_hz)
        if q.denominator == 1 and 1 <= q <= 256:
            t2 = (pre, int(q) - 1)
            break
    if t2 is None:
        raise GenError(f"Timer2 CTC cannot produce exactly {loop_hz} Hz from {f_cpu} Hz")
    ct = str(fc.get("control_timer", ""))
    mp_, mo_ = re.search(r"prescaler\s*(\d+)", ct), re.search(r"OCR2\s*=\s*(\d+)", ct)
    if mp_ and mo_ and (int(mp_.group(1)), int(mo_.group(1))) != t2:
        raise GenError(f"flight_computer.control_timer says {ct!r} but loop_hz {loop_hz} needs prescaler {t2[0]}, OCR2 {t2[1]}")
    if t2[0] not in TIMER0_PRESCALERS:
        raise GenError(f"Timer0 cannot use the Timer2 prescaler {t2[0]} for the IMU read lead timer")
    # Timer0 runs the same period, phase-advanced so the MPU-6050 burst read (started by its compare ISR)
    # is complete when the control tick fires: bits = (SLA+W, reg, SLA+R, data) x 9 + START/rep-START/STOP
    imu_bits = (3 + IMU_READ_BYTES) * 9 + 3
    imu_read_s = imu_bits / float(fc["i2c_hz"])
    count_s = t2[0] / f_cpu
    imu_lead_counts = math.ceil(imu_read_s * IMU_LEAD_MARGIN / count_s)
    if imu_lead_counts >= t2[1]:
        raise GenError("IMU read lead exceeds the control period")
    dt = 1.0 / loop_hz
    tick_us = Fraction(1_000_000, loop_hz)
    if tick_us.denominator != 1:
        raise GenError("loop period is not an integer number of microseconds")

    def divider(rate_hz: float, name: str) -> int:
        q = Fraction(loop_hz) / Fraction(rate_hz)
        if q.denominator != 1 or q < 1:
            raise GenError(f"{name} {rate_hz} Hz is not loop_hz / integer")
        return int(q)

    log_div = divider(fc["log_hz"], "flight_computer.log_hz")
    baro_div = divider(ctl["baro_hz"], "control.baro_hz")
    baro_sensor = str(fc.get("nav", {}).get("baro_sensor", BARO_SENSOR_LEGACY))
    if baro_sensor not in BARO_SENSORS:
        raise GenError(f"flight_computer.nav.baro_sensor {baro_sensor!r} not in {BARO_SENSORS}")
    imu_hz = int(ctl["imu_hz"])
    if 1000 % imu_hz:
        raise GenError("MPU-6050 with DLPF on samples at 1 kHz / (1 + SMPLRT_DIV): imu_hz must divide 1000")
    smplrt_div = 1000 // imu_hz - 1
    i2c_hz = int(fc["i2c_hz"])
    twbr = Fraction(f_cpu, i2c_hz) - 16
    if twbr.denominator != 1 or twbr % 2 or not 10 <= twbr / 2 <= 255:
        raise GenError(f"I2C {i2c_hz} Hz needs TWBR = (F_CPU/SCL - 16)/2 integer in 10..255")
    twbr = int(twbr / 2)
    ubrr_log = uart_setting(f_cpu, int(fc["log_baud"]))

    # ---- onboard supervisor (SPEC 8.8; the reference's own parameter object) ----
    if ms.get("powered_descent_enabled"):
        raise GenError("mission.powered_descent_enabled is true but the onboard supervisor has no powered descent")
    try:
        mode = lp.mission_mode(p)
    except ValueError as exc:
        raise GenError(str(exc)) from exc
    # the ballistic parameters (ONB_* of v0.6.3) are written in both modes: the same object v0.6.3 used
    sp = SupervisorParams.from_params(p, fc_config_path=cfg_path, mode=lp.MODE_BALLISTIC)
    if sp.loop_hz != loop_hz:
        raise GenError("SupervisorParams.loop_hz != control.loop_hz")
    if sp.i9_source != rel(cfg_path):
        raise GenError(f"sim/onboard.py took the I9 times from {sp.i9_source!r}, not {rel(cfg_path)}")
    onb_sec = {"ramp_ticks": "mission.throttle_ramp_s", "coast_hold_ticks": "mission.coast_control_hold_s",
               "fall_confirm_ticks": "mission.fall_deploy_confirm_s", "link_abort_ticks": "mission.link_loss_abort_s",
               "link_disarm_ticks": "mission.link_loss_disarm_s", "max_motor_ticks": "mission.max_motor_time_s",
               "release_after_cut_ticks": "mission.release_after_cut_s",
               "abort_release_ticks": "mission.abort_release_delay_s", "rate_persist_ticks": "mission.rate_abort_persist_s",
               "imu_fail_ticks": "fc_config imu_fail_kill_s", "baro_fail_ticks": "fc_config baro_fail_kill_s"}
    for k, src in onb_sec.items():
        n = getattr(sp, k)
        if not 0 <= n < 0xFFFFFFFF:
            raise GenError(f"{k} = {n} ticks does not fit uint32")
        sec_key = src.split(" ")[-1].split(".")[-1]
        sec = float(ms[sec_key]) if src.startswith("mission.") else float(cfg[sec_key])
        if abs(n / loop_hz - sec) > 1e-9:
            warnings.append(f"{src} = {sec} s is not a whole number of {dt * 1e3:.1f} ms ticks; firmware uses {n} ticks")
    if sp.link_abort_ticks <= sp.link_disarm_ticks:
        warnings.append("link_loss_abort_s <= link_loss_disarm_s")
    if not 0 < sp.climb_milli <= lp.THROTTLE_SCALE:
        raise GenError(f"mission.climb_throttle -> {sp.climb_milli} per mille outside 1..{lp.THROTTLE_SCALE}")
    if sp.fall_vz_mps >= 0:
        raise GenError("mission.fall_deploy_vz_mps must be negative (falling)")
    if sp.geofence_m <= sp.apogee_limit_m:
        warnings.append(f"geofence {sp.geofence_m} m <= apogee limit {sp.apogee_limit_m} m: I5 acts before I4")
    vt = derive_vtol(p, params_path, cfg_path, loop_hz, m_full, t_max, warnings, vbat_per_count)
    imu_idx, imu_sgn = imu_mounting(cfg["imu_mounting"])
    gc = cfg["pad_still"]

    # ---- sensors (sim/sim6dof.py SensorConfig) --------------------------------
    sens = SensorConfig()
    for field_name in ("gyro_range_dps", "gyro_lsb_per_dps", "accel_range_g", "accel_lsb_per_g",
                       "imu_dlpf_tau_s", "altitude_crossover_hz", "altitude_zeta"):
        if not hasattr(sens, field_name):
            raise GenError(f"sim/sim6dof.py SensorConfig has no {field_name!r}; the firmware estimator mirrors it")
    if getattr(sens, "attitude_estimator", "pad_gyro") != "pad_gyro":
        warnings.append(f"SensorConfig.attitude_estimator default is {sens.attitude_estimator!r}; the firmware flies pad_gyro")
    gfs = MPU_GYRO_FS.get(float(sens.gyro_range_dps))
    afs = MPU_ACCEL_FS.get(float(sens.accel_range_g))
    if gfs is None or abs(gfs[1] - float(sens.gyro_lsb_per_dps)) > 1e-9:
        raise GenError(f"SensorConfig gyro {sens.gyro_range_dps} dps / {sens.gyro_lsb_per_dps} LSB is not an MPU-6050 FS_SEL")
    if afs is None or abs(afs[1] - float(sens.accel_lsb_per_g)) > 1e-9:
        raise GenError(f"SensorConfig accel {sens.accel_range_g} g / {sens.accel_lsb_per_g} LSB is not an MPU-6050 AFS_SEL")
    dlpf = min(MPU_DLPF_GYRO_DELAY_S, key=lambda c: abs(MPU_DLPF_GYRO_DELAY_S[c] - float(sens.imu_dlpf_tau_s)))
    if abs(MPU_DLPF_GYRO_DELAY_S[dlpf] - float(sens.imu_dlpf_tau_s)) > 1e-6:
        raise GenError(f"SensorConfig.imu_dlpf_tau_s {sens.imu_dlpf_tau_s} is not an MPU-6050 DLPF_CFG gyro delay")
    if dlpf == 0:
        raise GenError("DLPF_CFG 0 runs the gyro at 8 kHz; SMPLRT_DIV assumes the 1 kHz rate of DLPF_CFG 1..6")

    # ---- LoRa link (SPEC 10.3) -------------------------------------------------
    lk = fc["link"]
    cl = cfg["link"]
    # 2026-11-10: the link MODEL follows vehicle_radio (sim.link_model.model_radio: the esp_bridge model); link.radio is
    # the LoRa fallback's model and must be one of the LoRa ones
    fallback = str(lk["radio"])
    if fallback not in lm.LORA_RADIOS:
        raise GenError(f"flight_computer.link.radio {fallback!r} (the LoRa fallback model) not in {lm.LORA_RADIOS}")
    radio = lm.model_radio(p)
    if radio not in lm.RADIOS:
        raise GenError(f"link model {radio!r} not in {lm.RADIOS}")
    # 2026-11-06: the COM backend the plain envs build (flight_computer.link.vehicle_radio, default link.radio).  link.radio
    # stays the link MODEL of sim/link_model.py (the budgets below, the ground's reply timeout): the model has no Wi-Fi,
    # and the LoRa model is the pessimistic case for the ESP-NOW bridge (E, docs/v0.9_펌웨어_결과.md 11장)
    vradio = str(lk.get("vehicle_radio", radio))
    if vradio not in VEHICLE_RADIOS:
        raise GenError(f"flight_computer.link.vehicle_radio {vradio!r} not in {VEHICLE_RADIOS}")
    esp = esp_bridge_settings(lk, f_cpu, loop_hz) if (vradio == "esp_bridge" or "esp_bridge" in lk) else None
    if vradio == "esp_bridge" and esp is None:
        raise GenError("flight_computer.link.vehicle_radio esp_bridge needs flight_computer.link.esp_bridge")
    vid = int(lk["vehicle_id"])
    if not 0 <= vid <= 0xFF:
        raise GenError(f"link.vehicle_id {vid} is not a byte")
    link_baud = int(lk["uart_baud"])
    ubrr_link = uart_setting(f_cpu, link_baud)
    if float(lk["poll_hz"]) <= 0:
        raise GenError("link.poll_hz must be > 0")
    bytes_per_tick = link_baud / 10.0 / loop_hz
    if bytes_per_tick > 8:
        warnings.append(f"link UART {link_baud} baud delivers {bytes_per_tick:.1f} bytes per tick; RX ring sized for <= 8")
    gps_tlm = bool(mode == lp.MODE_VTOL and vt and vt["spv"].gps_hold)
    lbat_tlm = bool(mode == lp.MODE_VTOL and lp.lbat_build(p, mode))   # 2026-11-10: the battery build's TELEM v6 / v7
    telem_bytes = lm.telem_frame_bytes(mode, gps=gps_tlm, lbat=lbat_tlm)                 # the reply this build sends
    sx = sx127x_registers(lk, cl, loop_hz, warnings, telem_bytes=telem_bytes)
    spi_max = float(cl["sx127x_spi_hz_max"])
    spi = None
    for div in sorted(SPI_DIVIDERS):
        if f_cpu / div <= spi_max:
            spi = (div,) + SPI_DIVIDERS[div]
            break
    if spi is None:
        raise GenError(f"no SPI divider reaches <= {spi_max} Hz")
    aux_wait_ticks = max(1, ticks_ge(float(cl["aux_wait_max_s"]), loop_hz))
    seed_conv = int(cl["nonce_seed_adc_conversions"])
    if not 8 <= seed_conv <= 255:
        raise GenError("fc_config link.nonce_seed_adc_conversions must be 8..255")
    budgets = {}
    for r in lm.RADIOS:
        cfg_r = lm.LinkConfig.from_params(p, radio=r)
        b = lm.budget(cfg_r, mode=mode, gps=gps_tlm, lbat=lbat_tlm)   # v0.7.1: sized for the reply frame of this build
        if int(b["telem_frame_bytes"]) != telem_bytes:
            raise GenError(f"sim/link_model.py budget(mode={mode!r}) sizes the downlink for {b['telem_frame_bytes']} B, "
                           f"this build sends {telem_bytes} B")
        # gap between two uplink frames at the vehicle when exactly one request or reply is lost:
        # the ground gives the lost exchange up after reply_timeout, the next one takes one round trip
        req_ok = max(1.0 / cfg_r.poll_hz, b["rtt_worst_s"])
        req_lost = max(1.0 / cfg_r.poll_hz, b["reply_timeout_s"])
        b["poll_interval_s"] = req_ok
        b["single_loss_gap_s"] = req_ok + req_lost
        budgets[r] = b
    bsel = budgets[radio]
    for r_chk in dict.fromkeys((radio, fallback)):          # 2026-11-10: the model and the LoRa fallback
        if budgets[r_chk]["single_loss_gap_s"] >= float(ms["link_loss_disarm_s"]):
            raise GenError(f"{r_chk}: one lost frame leaves the vehicle {budgets[r_chk]['single_loss_gap_s']:.3f} s without "
                           f"an uplink, >= link_loss_disarm_s {ms['link_loss_disarm_s']} s - every lost frame would disarm "
                           "on the pad")
    if not bsel["poll_hz_achievable"]:
        warnings.append(f"{radio}: link.poll_hz {lk['poll_hz']} Hz is not reachable - the worst round trip is "
                        f"{bsel['rtt_worst_s'] * 1e3:.0f} ms, so the ground polls at {1.0 / bsel['poll_interval_s']:.2f} Hz")
    margin = float(ms["link_loss_disarm_s"]) - bsel["single_loss_gap_s"]
    if margin < 0.15:
        warnings.append(f"{radio}: a single lost frame leaves only {margin * 1e3:.0f} ms to the pad link-loss interlock "
                        f"(gap {bsel['single_loss_gap_s'] * 1e3:.0f} ms vs link_loss_disarm_s); two in a row disarm on the pad")

    # ---- COM tick + FCU <-> COM ilink (SPEC 10.4; flight_computer.com / .ilink) ----------------
    com_tick_hz = int(com["tick_hz"])
    if com_tick_hz != com["tick_hz"] or com_tick_hz != loop_hz:
        raise GenError(f"flight_computer.com.tick_hz {com['tick_hz']} != control.loop_hz {loop_hz}: the radio drivers' "
                       "LINK_AUX_WAIT_TICKS / SX127X_TX_TIMEOUT_TICKS are counted in control ticks and run on the COM")
    il = fc.get("ilink")
    if not isinstance(il, dict):
        raise GenError("flight_computer.ilink missing (SPEC 10.4)")
    il_baud = int(il["baud"])
    if il_baud != il["baud"]:
        raise GenError(f"flight_computer.ilink.baud {il['baud']} is not an integer")
    il_ubrr, il_err = ilink_uart_setting(f_cpu, il_baud)
    stat_hz = Fraction(str(il["stat_hz"]))                      # the decimal written in params.json
    if stat_hz <= 0:
        raise GenError(f"flight_computer.ilink.stat_hz {il['stat_hz']} must be > 0")
    stat_div = Fraction(com_tick_hz) / stat_hz
    if stat_div.denominator != 1 or not 1 <= stat_div <= 0xFFFF:
        raise GenError(f"flight_computer.ilink.stat_hz {il['stat_hz']} Hz is not com.tick_hz {com_tick_hz} Hz / integer "
                       f"(1..65535): ILINK_STAT_DIV would be {float(stat_div):g}")
    r_series = float(il.get("series_resistor_ohm", 0.0))
    if r_series < 0:
        raise GenError("flight_computer.ilink.series_resistor_ohm must be >= 0")
    # ---- NAV tick + NAV -> FCU nlink SPI (SPEC 10.5; flight_computer.nav / .nlink) ----------------------------
    nav_tick_hz = nav.get("tick_hz")
    if nav_tick_hz != loop_hz:
        raise GenError(f"flight_computer.nav.tick_hz {nav_tick_hz} != control.loop_hz {loop_hz}: the FCU's tick starts on "
                       "the NAV's frame, so both run the control tick (and the NAV reuses FC_T2_OCR / FC_IMU_LEAD_COUNTS)")
    nlk = fc.get("nlink")
    if not isinstance(nlk, dict):
        raise GenError("flight_computer.nlink missing (SPEC 10.5)")
    spi_hz = float(nlk["spi_clock_hz"])
    nl_div = next((dv for dv in sorted(SPI_DIVIDERS) if abs(f_cpu / dv - spi_hz) < 1e-6), None)
    if nl_div is None:
        raise GenError(f"flight_computer.nlink.spi_clock_hz {spi_hz:g}: the ATmega128 SPI master makes only F_CPU / "
                       f"{{{', '.join(str(dv) for dv in sorted(SPI_DIVIDERS))}}} = "
                       f"{', '.join(f'{f_cpu / dv:g}' for dv in sorted(SPI_DIVIDERS))} Hz")
    if nl_div < 4:
        raise GenError(f"flight_computer.nlink.spi_clock_hz {spi_hz:g} > F_CPU / 4: the FCU's SPI slave needs SCK high and "
                       "low for more than 2 CPU cycles each (datasheet, SPI slave mode)")
    if nlk.get("spi_mode") != 0:
        raise GenError(f"flight_computer.nlink.spi_mode {nlk.get('spi_mode')!r}: hal/nlink_spi.c and SPEC 10.5 use mode 0 "
                       "(the slave presents its byte when SS falls / SPDR is written)")
    gap_us = float(nlk["byte_gap_us"])
    if not gap_us > 0:
        raise GenError(f"flight_computer.nlink.byte_gap_us {gap_us:g} must be > 0")
    gap_cycles = math.ceil(gap_us * f_cpu / 1e6 - 1e-9)
    if gap_cycles < NLINK_SLAVE_REPLY_CYCLES:
        raise GenError(f"flight_computer.nlink.byte_gap_us {gap_us:g} = {gap_cycles} cycles < {NLINK_SLAVE_REPLY_CYCLES}: "
                       "the FCU's slave interrupt could not load its next return byte in the gap")
    xfer_cycles = (2 * (gap_cycles + NLINK_MASTER_SS_CYCLES) + nlp.XFER_BYTES * 8 * nl_div
                   + (nlp.XFER_BYTES - 1) * (gap_cycles + NLINK_MASTER_LOOP_CYCLES))
    xfer_s = xfer_cycles / f_cpu                                 # SS low .. SS high of one exchange, no interrupts
    if xfer_s >= NLINK_XFER_MAX_TICK_FRACTION * dt:
        raise GenError(f"one nlink exchange takes {xfer_s * 1e6:.0f} us >= {NLINK_XFER_MAX_TICK_FRACTION:.0%} of the "
                       f"{dt * 1e6:.0f} us NAV tick (spi_clock_hz {spi_hz:g}, byte_gap_us {gap_us:g})")
    if xfer_s > NLINK_XFER_WARN_TICK_FRACTION * dt:
        warnings.append(f"one nlink exchange takes {xfer_s * 1e6:.0f} us, {xfer_s / dt:.0%} of the NAV tick")
    timeout_s = float(nlk["frame_timeout_ms"]) * 1e-3
    if not dt < timeout_s < 2 * dt:
        raise GenError(f"flight_computer.nlink.frame_timeout_ms {nlk['frame_timeout_ms']}: must lie between one and two "
                       f"control ticks ({dt * 1e3:g} .. {2 * dt * 1e3:g} ms) - the FCU's own tick must not start before a "
                       "frame that is on time, and one lost frame must cost one tick")
    timeout_us = Fraction(str(nlk["frame_timeout_ms"])) * 1000
    if timeout_us.denominator != 1:
        raise GenError(f"flight_computer.nlink.frame_timeout_ms {nlk['frame_timeout_ms']} is not a whole number of us")
    timeout_t1 = int(timeout_us) * counts_per_us
    if timeout_t1 > top:
        raise GenError(f"frame timeout {int(timeout_us)} us = {timeout_t1} Timer1 counts exceeds one PWM frame ({top + 1}): "
                       "timer_elapsed_counts could not measure it")
    nl_wd = nlink_watchdog(nlk, f_cpu, xfer_s, int(tick_us))
    # ---- v0.8 GPS on USART1 (flight_computer.nav.gps; v0.8.1: of the GNS board - the NAV's USART1 PD2 RXD1 / PD3 TXD1
    # carries glink, so it stays free on the NAV either way) ----
    gps_uart = None
    if vt and vt.get("gps"):
        user = "glink (v0.8.1)" if vt.get("glink") else "the GPS"
        for pin, nm in ((("D", 2), "RXD1"), (("D", 3), "TXD1")):
            for sig, pp_ in nav_pins.items():
                if pp_ == pin:
                    raise GenError(f"flight_computer.nav.pins.{sig} uses PD{pin[1]}, {user}'s {nm} (USART1)")
        if vt.get("glink") and gns_pins is None:
            raise GenError("v0.8.1 glink: flight_computer.gns.pins missing")
        gps_uart = uart_setting(f_cpu, vt["gps"]["baud"])
        if vt["gps"]["ubrr_cfg"] is not None and int(vt["gps"]["ubrr_cfg"]) != gps_uart[0]:
            raise GenError(f"flight_computer.nav.gps.ubrr {vt['gps']['ubrr_cfg']} != the computed UBRR {gps_uart[0]} "
                           f"(U2X {gps_uart[1]}) for {vt['gps']['baud']} baud")
    params_sha = sha256(params_path)
    params_tag = int(params_sha[:16], 16) & 0xFF                # low byte of FC_PARAMS_SHA256_PREFIX
    vgain_src = ([] if vt is None or not (vt["gains"]["tuned"] or vt["gains"]["est_tuned"])
                 else [(rel(vt["gains"]["source_path"]), vt["gains"]["sha256"])])

    return dict(
        vcfg_name=vcfg or "rocket", cad_path=cad_path, cad_sha=cad_report_sha256(cad_path), nl_wd=nl_wd,
        p=p, L=L, cfg=cfg, warnings=warnings, mode=mode, vt=vt, telem_bytes=telem_bytes,
        sources=[(rel(params_path), params_sha), ("cad/layout.py", sha256(os.path.join(ROOT, "cad", "layout.py"))),
                 (rel(cad_path) + " (canonical)", cad_report_sha256(cad_path)), (rel(SIM6DOF), sha256(SIM6DOF)),
                 (rel(cfg_path), sha256(cfg_path))] + vgain_src
                + [(rel(r) + " (mirrored)", sha256(r)) for r in MIRRORED if os.path.exists(r)],
        cad_generator=f"{cad.get('generator', '?')} {cad.get('generator_version', '')}".strip(),
        m_full=m_full, c_full=c_full, i_full=i_full, m_post=m_post, c_post=c_post,
        z_cg=z_cg, hinge_z=hinge_z, l_v=l_v, r_v=r_v, s_v=s_v, a_e=a_e, cl_alpha=float(vn["cl_alpha_per_rad"]),
        t_max=t_max, n_exp=n_exp, n_exp_int=n_exp_int, tau=float(edf["thrust_time_constant_s"]),
        rho=float(p["aero"]["rho_kg_m3"]), p_nom=float(edf["power_nominal_W"]), omega_max=omega_max,
        swirl_fraction=float(edf["residual_swirl_torque_fraction_ASSUMED"]), swirl_sign=swirl_sign, hover_u=hover_u,
        delta_cmd=math.radians(cmd), delta_mech=math.radians(mech), stall=math.radians(float(vn["stall_deg"])),
        center=center, us_per_deg=us_per_deg, signs=signs, trims=trims, vane_lo=vane_lo, vane_hi=vane_hi,
        latched=latched, released=released, esc=esc, pwm_map=pwm_map, fcu_pins=fcu_pins, com_pins=com_pins,
        nav_pins=nav_pins, nav_tick_hz=int(nav_tick_hz), nl_spi_hz=spi_hz, nl_div=nl_div, nl_spi=SPI_DIVIDERS[nl_div],
        nl_gap_us=gap_us, nl_gap_cycles=gap_cycles, nl_xfer_s=xfer_s, nl_timeout_us=int(timeout_us),
        nl_timeout_t1=timeout_t1,
        com_tick_hz=com_tick_hz, il_baud=il_baud, il_ubrr=il_ubrr, il_err=il_err, stat_hz=float(stat_hz),
        stat_div=int(stat_div), r_series=r_series, params_tag=params_tag,
        f_cpu=f_cpu, presc=presc, top=top, counts_per_us=counts_per_us, frame_hz=int(frame_hz),
        loop_hz=loop_hz, dt=dt, tick_us=int(tick_us), t2=t2, imu_lead_counts=imu_lead_counts, imu_read_s=imu_read_s,
        count_s=count_s, log_div=log_div, baro_div=baro_div, baro_sensor=baro_sensor, imu_hz=imu_hz,
        smplrt_div=smplrt_div, i2c_hz=i2c_hz, twbr=twbr, ubrr_log=ubrr_log,
        log_baud=int(fc["log_baud"]), log_hz=int(fc["log_hz"]),
        adc_ch=adc_ch, r_top=r_top, r_bot=r_bot, vref=vref, vbat_per_count=vbat_per_count, adc_ps=adc_ps, adc_div=adc_div,
        lbat=lbat,
        cells=int(bat["cells"]), ms=ms, sp=sp, g0=float(G0),
        sens=sens, imu_idx=imu_idx, imu_sgn=imu_sgn, pad_still=gc, gfs=gfs, afs=afs, dlpf=dlpf,
        pad_still_ticks=ticks_ge(float(gc["duration_s"]), loop_hz),
        arm_debounce_ticks=max(1, ticks_ge(float(cfg["arm_switch_debounce_s"]), loop_hz)),
        radio=radio, fallback=fallback, vradio=vradio, esp=esp, vid=vid, link_baud=link_baud, ubrr_link=ubrr_link, lk=lk, sx=sx, spi=spi, aux_wait_ticks=aux_wait_ticks,
        seed_conv=seed_conv, budgets=budgets, gps_uart=gps_uart, gns_pins=gns_pins,
    )


# --------------------------------------------------------------------------- #
# header
# --------------------------------------------------------------------------- #
def render_vtol(d: dict, define, w) -> None:
    """MISSION_MODE, the build profile switch and the VTOL block (SPEC 8.9; v0.7.1): per build profile the supervisor's
    VTOL fields (sim/onboard.py vtol_fields), the vertical controller (sim/vertical_ctrl.py VerticalParams), the NAV
    estimator (sim/estimator.py GatedMahonyEstimator) and the NAV altitude filter (sim/state_machine.py
    VtolAltitudeFilter), read by core/fc_config.c only.

    Profiles (v0.7.1, params.json mission.vtol.profile / profiles): every profile is rendered, the default
    (mission.vtol.profile) first; a build picks one with -D MISSION_PROFILE_<NAME> (e.g. -D MISSION_PROFILE_DEMO for the
    indoor demonstration), none = the default.  Why a compile-time switch in one header rather than a generator option:
    the header stays a pure function of the sources (one file on disk, --check stays meaningful) and the field and demo
    HEX files are built from the same header by two PlatformIO envs; text parsers that take the first #define of a name
    (tools of the co-simulation / simbench) read the default profile."""
    vt = d["vt"]
    w("/* ---- VTOL parameters (MISSION_MODE: at the top of this header, fw09 G1 needs it for FC_VEHICLE) ---- */")
    define("MISSION_HAS_VTOL_PARAMS", "1" if vt else "0", "mission.vtol present (the VTOL macros below)")
    define("NLINK_FCU_THR_SCALE", f"{nlp.FCU_THR_SCALE}u",
           "v0.7.1 nlink FCU frame byte 7 thr_q = round(u x this) (sim/nlink_proto.py FCU_THR_SCALE)")
    if not vt:
        return
    gi, names, default = vt["gains"], list(vt["profiles"]), vt["default"]
    w("")
    w("/* ---- VTOL build profile (v0.7.1, params.json mission.vtol.profile / profiles; chosen at BUILD time) ----")
    w(f" * default (no switch): '{default}' = params.json mission.vtol.profile; other profiles: -D MISSION_PROFILE_<NAME>")
    w(" * (" + ", ".join(f"-D MISSION_PROFILE_{nm.upper()}" for nm in names if nm != default) + ").  Every safety rule is")
    w(" * the same in every profile; only the numbers of params.json mission.vtol.profiles.<name> differ.  The NAV, FCU and")
    w(" * COM of one vehicle are built with the same profile (FC_PARAMS_TAG does not tell the profiles apart). */")
    for i, nm in enumerate(names):
        define(f"MISSION_PROFILE_ID_{nm.upper()}", str(i + 1), f"mission.vtol profile '{nm}'")
    others = [nm for nm in names if nm != default]
    if len(others) > 1:
        w("#if " + " + ".join(f"defined(MISSION_PROFILE_{nm.upper()})" for nm in others) + " > 1")
        w('#error "more than one MISSION_PROFILE_<NAME> switch defined"')
        w("#endif")
    for i, nm in enumerate(others):
        w(f"#{'if' if i == 0 else 'elif'} defined(MISSION_PROFILE_{nm.upper()})")
        define("MISSION_PROFILE", f"MISSION_PROFILE_ID_{nm.upper()}", "")
        w(f'#define MISSION_PROFILE_NAME               "{nm}"')
    w("#else" if others else "/* single profile */")
    define("MISSION_PROFILE", f"MISSION_PROFILE_ID_{default.upper()}", "the default profile")
    w(f'#define MISSION_PROFILE_NAME               "{default}"')
    if others:
        w("#endif")
    w("")
    w("/* ---- gains of the VTOL loops (the same in every profile) ----")
    w(f" * vertical: {gi['source']}; NAV estimator kp / ki / accel_lpf_hz: {gi['est_source']}")
    w(f" * (sha256 {gi['sha256'][:16]}); VC_GAINS_TUNED 0 = the ASSUMED starting values of params.json, not tuned on the")
    w(" * 6-DOF; VC_GAINS_PARAMS_STALE 1 = sim/out/vtol_gains.json was tuned against another params.json */")
    w(f'#define VC_GAINS_SOURCE                    "{gi["source"]}"')
    define("VC_GAINS_TUNED", "1" if gi["tuned"] else "0", "sim/out/vtol_gains.json used for the vertical gains")
    define("VC_GAINS_PARAMS_STALE", "1" if gi["params_stale"] else "0", "")
    w(f'#define EST_V_GAINS_SOURCE                 "{gi["est_source"]}"')
    define("EST_V_GAINS_TUNED", "1" if gi["est_tuned"] else "0",
           "sim/out/vtol_gains.json used for the estimator kp / ki / accel_lpf_hz")
    w(f'#define POS_GAINS_SOURCE                   "{gi["pos_source"]}"')
    define("POS_GAINS_TUNED", "1" if gi["pos_tuned"] else "0", "sim/out/vtol_gains.json used for the position-hold gains")
    if vt.get("gps"):
        _render_gps(d, define, w, vt["gps"])
    if vt.get("glink"):
        _render_glink(d, define, w, vt["glink"])
    w("")
    w("/* ---- v0.7.1b attitude gains of the VTOL build (params.json control.vtol_overrides; the same in every profile) ----")
    w(" * core/fc_config.c fc_vtol_gains = fc_default_gains (gains.h) with each VTOL_OV_<KEY> below; the ballistic build")
    w(" * calls fc_default_gains and never reads these (sim/controller.py gains_for_mode). */")
    define("VTOL_OV_COUNT", str(len(vt["att_ov"])), "control.vtol_overrides entries")
    for k, val in vt["att_ov"].items():
        define(f"VTOL_OV_{k.upper()}", ("1" if val else "0") if k == "yaw_hold" else
               (str(int(val)) if k in VTOL_OV_INT_CHOICES else cf(float(val))),
               f"control.vtol_overrides.{k} (params.json control.{k} = {d['p']['control'].get(k)})")
    a9 = vt["att_v09"]
    w("/* ---- v0.9 attitude options of the VTOL build (sim/controller.py VTOL_EXTRA_KEYS, the reference's defaults when a")
    w(" * key is absent; core/attitude_ctrl.h att_v09_t via core/fc_config.c fc_vtol_gains).  The rotor feed-forward")
    w(" * constants are derived once in float64 from the VTOL build's merged gains (gains_for_mode): a = dt / (tau + dt),")
    w(" * NKR = -react h_ref / tau / roll_priority_scale (the yaw moment command gets NKR (u - u_lag)) ---- */")
    define("VTOL_V09_ERR_MODE", str(a9["err_mode"]), "att_err_mode: 0 rotation vector, 1 reduced error while tilted, 2 always")
    define("VTOL_V09_AW_PER_AXIS", str(a9["aw_per_axis"]), "aw_per_axis: per-axis conditional anti-windup")
    define("VTOL_V09_FF_ON", str(a9["ff_on"]), f"ff_rotor_h_nms = {a9['ff_h']:g} N m s != 0: rotor feed-forwards on")
    define("VTOL_V09_FF_A", cf(a9["ff_a"]), f"dt / (ff_rotor_tau_s {a9['ff_tau']:g} + dt): the u_lag filter")
    define("VTOL_V09_FF_NKR", cf(a9["ff_nkr"]),
           f"-ff_rotor_react {a9['ff_react']:g} x h_ref / tau / roll_priority_scale {a9['rps']:g} [N m per unit du]")
    define("VTOL_V09_FF_H", cf(a9["ff_h"]), "ff_rotor_h_nms (signed: s I_rot omega_ref; the gyroscopic term)")
    define("VTOL_V09_FF_GYRO", str(a9["ff_gyro"]), "ff_rotor_gyro: gyroscopic decoupling (h_z = -h_ref u_lag)")
    define("VTOL_V09_YAW_TRIM_RESERVE", str(a9["yaw_trim_reserve"]), "mix_yaw_trim_reserve: the mixer keeps the yaw trim")
    for i, nm in enumerate(names):
        w("")
        w(f"#{'if' if i == 0 else 'elif'} MISSION_PROFILE == MISSION_PROFILE_ID_{nm.upper()}"
          f"  /* ======== profile '{nm}'{' (default)' if nm == default else ''} ======== */")
        _render_vtol_profile(d, define, w, vt["profiles"][nm], nm)
    w("#else")
    w('#error "MISSION_PROFILE names no profile of params.json mission.vtol"')
    w("#endif  /* MISSION_PROFILE */")


def _render_vtol_profile(d: dict, define, w, pr: dict, nm: str) -> None:
    """The VTOL macros of one profile (render_vtol)."""
    ms = d["ms"]
    spv, vp, v, est, alt = pr["spv"], pr["vp"], pr["v"], pr["est"], pr["alt"]
    td, bl = v.get("touchdown", {}), v.get("baro_lost", {})
    w("/* ---- VTOL supervisor (SPEC 8.9, sim/onboard.py SupervisorParams.from_params(mode='vtol', profile)); ticks, dm ----")
    w(" * the ballistic ONB_* above are shared (ARM gates, I2, I3, I8 V_cell, I9, I1p, abort_release_ticks) */")
    define("ONB_V_SPOOL_TICKS", f"{spv.spool_ticks}UL", f"mission.vtol.spool_warning_s = {v['spool_warning_s']:g} s")
    define("ONB_V_HOVER_DM", str(spv.hover_dm), f"hover_altitude_m = {v['hover_altitude_m']:g} m (ON target)")
    define("ONB_V_ALT_STEP_DM", str(spv.alt_step_dm), f"alt_step_m = {v['alt_step_m']:g} m (UP / DOWN)")
    define("ONB_V_ALT_MIN_DM", str(spv.alt_min_dm), f"alt_min_m = {v['alt_min_m']:g} m")
    define("ONB_V_ALT_MAX_DM", str(spv.alt_max_dm), f"alt_max_m = {v['alt_max_m']:g} m")
    define("ONB_V_LINK_LAND_TICKS", f"{spv.link_land_ticks}UL", f"V1 link_loss_land_s = {v['link_loss_land_s']:g} s")
    define("ONB_V_ENDURANCE_TICKS", f"{spv.endurance_ticks}UL",
           f"V2 / REJ_ENDURANCE max_flight_s {v['max_flight_s']:g} - land_reserve_s {v['land_reserve_s']:g} s (motor time "
           "since boot)")
    define("ONB_V_HARD_CUTOFF_TICKS", f"{spv.hard_cutoff_ticks}UL", f"V6 hard_cutoff_s = {v['hard_cutoff_s']:g} s")
    define("ONB_V_CEILING_M", cf(spv.ceiling_m), f"V3 ceiling_m = {v['ceiling_m']:g} m")
    define("ONB_V_GEOFENCE_M", cf(spv.v_geofence_m), f"V4 geofence_m = {v['geofence_m']:g} m")
    define("ONB_V_FALL_VZ_MPS", cf(spv.v_fall_vz_mps), "V5 fall_vz_mps")
    define("ONB_V_FALL_CONFIRM_TICKS", f"{spv.v_fall_confirm_ticks}UL", f"V5 fall_confirm_s = {v['fall_confirm_s']:g} s")
    define("ONB_V_SAFE_AFTER_LANDED_TICKS", f"{spv.safe_after_landed_ticks}UL",
           f"LANDED -> SAFE safe_after_landed_s = {v['safe_after_landed_s']:g} s")
    define("ONB_V_LAND_TIMEOUT_TICKS", f"{spv.land_timeout_ticks}UL",
           f"V10 touchdown.land_floor_timeout_s_ASSUMED = {float(td.get('land_floor_timeout_s_ASSUMED', 0.0)):g} s")
    define("ONB_V_BARO_LIMIT_BASE_TICKS", f"{spv.baro_limit_base_ticks}UL",
           f"V8/V9 ticks_ge(baro_lost.time_limit_factor {float(bl.get('time_limit_factor', 1.5)):g} x flare "
           f"{v['flare_altitude_m']:g} m / touchdown {v['touchdown_rate_mps']:g} m/s + margin "
           f"{float(bl.get('time_limit_margin_s', 0.0)):g} s)")
    define("ONB_V_BARO_LIMIT_MT_PER_DM", f"{spv.baro_limit_mt_per_dm}UL",
           f"V8/V9 round(factor x loop_hz / (10 x descent {v['descent_rate_mps']:g} m/s) x 1000) milli-ticks per dm")
    define("ONB_V_GEOFENCE_DM", str(spv.v_geofence_dm), "V8/V9 clamp of h at the baro loss [0.1 m]")
    w("/* ---- VTOL vertical controller v2 (sim/vertical_ctrl.py VerticalParams.from_params(profile); SI, float32; um) ---- */")
    define("VC_CLIMB_RATE_MPS", cf(vp.climb_rate_mps), "mission.vtol.climb_rate_mps (informative: VC_UP_UM flies it)")
    define("VC_DESCENT_RATE_MPS", cf(vp.descent_rate_mps), "mission.vtol.descent_rate_mps (informative: VC_DN_UM)")
    define("VC_TOUCHDOWN_RATE_MPS", cf(vp.touchdown_rate_mps), "mission.vtol.touchdown_rate_mps (informative: VC_TD_UM)")
    define("VC_UP_UM", f"{vp.up_um}L", "reference step per tick [um] = round(climb_rate x 1e6 / loop_hz)")
    define("VC_DN_UM", f"{vp.dn_um}L", "round(descent_rate x 1e6 / loop_hz)")
    define("VC_TD_UM", f"{vp.td_um}L", "round(touchdown_rate x 1e6 / loop_hz), below the flare altitude")
    define("VC_FLARE_UM", f"{vp.flare_um}L", "round(flare_altitude_m x 1e6)")
    define("VC_UM_TICK_TO_MPS", cf(vp.loop_hz / 1e6), "loop_hz x 1e-6: dh_ref [m/s] per um of reference step")
    define("VC_FLARE_ALTITUDE_M", cf(vp.flare_altitude_m), "mission.vtol.flare_altitude_m")
    define("VC_ALT_KP", cf(vp.alt_kp), "[1/s] v_z command per metre of reference error")
    define("VC_VZ_KP", cf(vp.vz_kp), "[s/m] throttle per m/s of vertical-speed error")
    define("VC_VZ_KI", cf(vp.vz_ki), "[1/m] throttle per metre of integrated error (informative: VC_VZ_KI_DT)")
    define("VC_VZ_KI_DT", cf(vp.vz_ki * vp.dt), "vz_ki x dt")
    define("VC_VZ_I_MAX", cf(vp.vz_i_max), "integrator limit (throttle)")
    define("VC_VZ_MAX_UP_MPS", cf(vp.vz_max_up_mps), "mission.vtol.vz_max_up_mps")
    define("VC_VZ_MAX_DOWN_MPS", cf(vp.vz_max_down_mps), "mission.vtol.vz_max_down_mps")
    define("VC_U_MIN", cf(vp.u_min), "mission.vtol.u_min")
    define("VC_U_MAX", cf(vp.u_max), "mission.vtol.u_max")
    define("VC_U_HOVER", cf(vp.u_hover),
           f"(m g / T_max)^(1/n), m = {d['m_full']:.4f} kg, T_max {d['t_max']:g} N: {vp.hover_margin:.3f} above it to u_max")
    define("VC_INV_U_HOVER", cf(vp.inv_u_hover), "1 / u_hover")
    define("VC_THROTTLE_EXP", cf(vp.throttle_exp), "edf.throttle_exponent (thrust feed-forward inverse)")
    define("VC_THROTTLE_EXP_INT", str(d["n_exp_int"]), "2: u_hover / sqrtf(k_T c) and x^2; else the powf fallback")
    define("VC_COS_TILT_MIN", cf(vp.cos_tilt_min), f"cos(mission.tilt_abort_deg {ms['tilt_abort_deg']:g}): feed-forward floor")
    define("VC_RAMP_TICKS", f"{vp.ramp_ticks}U", f"mission.throttle_ramp_s = {ms['throttle_ramp_s']:g} s (take-off ceiling ramp)")
    define("VC_LANDED_CONFIRM_TICKS", f"{vp.landed_confirm_ticks}U", f"landed_confirm_s = {v['landed_confirm_s']:g} s")
    define("VC_FLOOR_TICKS", f"{vp.floor_ticks}U",
           f"touchdown.floor_s_ASSUMED = {float(td.get('floor_s_ASSUMED', 1.0)):g} s (the loop pinned downward)")
    define("VC_TD_VZ_STILL_MPS", cf(vp.td_vz_still_mps),
           "still: |v_lpf| <= TD_STILL_FRACTION_ASSUMED x touchdown_rate_mps (ASSUMED, sim/vertical_ctrl.py)")
    define("VC_STILL_LEAD", cf(vp.still_lead), "TD_STILL_LEAD_ASSUMED (informative: VC_LEAD_MPS)")
    define("VC_LEAD_MPS", cf(vp.still_lead * vp.touchdown_rate_mps),
           f"still / V10 lead: h_err <= -this (= {vp.still_lead * vp.touchdown_rate_mps / vp.alt_kp:.3f} m of reference lead)")
    define("VC_SUPPORT_G", cf(vp.support_g), "touchdown.support_g_ASSUMED (informative: VC_SUPPORT_MPS2)")
    define("VC_SUPPORT_MPS2", cf(vp.support_g * vp.g), "support: d_lpf > support_g x g")
    define("VC_G", cf(vp.g), "sim/state_machine.py G0")
    define("VC_THRUST_ALPHA", cf(vp.thrust_alpha), f"1 - exp(-dt / edf.thrust_time_constant_s {vp.thrust_tau_s:g} s)")
    define("VC_STILL_ALPHA", cf(vp.still_alpha), f"1 - exp(-dt / TD_STILL_LPF_TAU_S_ASSUMED {vp.still_tau_s:g} s)")
    define("VC_SUPPORT_ALPHA", cf(vp.support_alpha), f"1 - exp(-dt / TD_SUPPORT_LPF_TAU_S_ASSUMED {vp.support_tau_s:g} s)")
    define("VC_VZ_FF", "1" if vp.vz_ff else "0", "mission.vtol.vz_ref_feedforward")
    w("/* v0.7.1b thrust learning (mission.vtol.thrust_learning; sim/vertical_ctrl.py docstring 4b): k_T += alpha (f_z / (g r)")
    w(" * - k_T) in free flight above the flare band, clamped; u_ff = u_hover / sqrtf(k_T c), support d = f_z - k_T g r */")
    define("VC_KT_TAU_S", cf(vp.kt_tau_s), "thrust_learning.tau_s_ASSUMED (informative: VC_KT_ALPHA; 0 = no learning)")
    define("VC_KT_ALPHA", cf(vp.kt_alpha), "1 - exp(-dt / tau_s_ASSUMED)")
    define("VC_KT_MIN", cf(vp.kt_min), "thrust_learning.k_min (clamp)")
    define("VC_KT_MAX", cf(vp.kt_max), "thrust_learning.k_max (clamp)")
    define("VC_KT_MAX_TILT_DEG", cf(vp.kt_max_tilt_deg), "thrust_learning.max_tilt_deg (informative: VC_KT_COS_MIN)")
    define("VC_KT_COS_MIN", cf(vp.kt_cos_min), "cos(max_tilt_deg): learn only while cos(tilt) > this")
    w("/* ---- VTOL NAV estimator (sim/estimator.py GatedMahonyEstimator; mission.vtol.nav_estimator; pad: gains.h N_avg) ---- */")
    define("EST_V_KP", cf(est["kp"]), "[1/s] Mahony correction (nav_estimator.kp_ASSUMED or the tuned value)")
    define("EST_V_KI", cf(est["ki"]), "[1/s^2] in-flight bias learning (informative: EST_V_KI_DT)")
    define("EST_V_KI_DT", cf(est["ki_dt"]), "ki x dt")
    define("EST_V_ACCEL_GATE", cf(est["accel_gate"]), "nav_estimator.accel_gate (| |a_lpf| - g | < this x g)")
    define("EST_V_GATE_MPS2", cf(est["gate_mps2"]), "accel_gate x g")
    define("EST_V_G", cf(est["g"]), "sim/state_machine.py G0")
    define("EST_V_ACCEL_LPF_HZ", cf(est["accel_lpf_hz"]), "nav_estimator.accel_lpf_hz_ASSUMED or the tuned value")
    define("EST_V_ALPHA_A", cf(est["alpha_a"]), "1 - exp(-2 pi accel_lpf_hz dt)")
    define("EST_V_BIAS_LIMIT_RAD_S", cf(est["bias_limit_rad_s"]),
           f"nav_estimator.bias_limit_dps = {est['bias_limit_dps']:g} deg/s (per axis, relative to the pad bias)")
    w("/* ---- VTOL NAV altitude filter (sim/state_machine.py VtolAltitudeFilter; SensorConfig, mission.vtol) ---- */")
    define("ALTV_K1", cf(alt["k1"]), f"2 zeta w + p (f_c {alt['crossover_hz']:g} Hz, zeta {alt['zeta']:g}, bias pole "
                                      f"1 / {alt['bias_tau_s']:g} s)")
    define("ALTV_K2", cf(alt["k2"]), "w^2 + 2 zeta w p")
    define("ALTV_K3", cf(alt["k3"]), "w^2 p")
    define("ALTV_BIAS_TAU_S", cf(alt["bias_tau_s"]), "SensorConfig.vtol_alt_bias_tau_s / VTOL_ALT_BIAS_TAU_S_ASSUMED")
    define("ALTV_PAD_N_AVG", f"{alt['pad_n_avg']}U", f"round(pad tau {alt['pad_tau_s']:g} s / dt): pad bias running mean, "
                                                     "then EMA 1/N")
    define("ALTV_BIAS_LIMIT_MPS2", cf(alt["bias_limit"]), "VTOL_ALT_BIAS_LIMIT_MPS2_ASSUMED")
    define("ALTV_COMP_M", cf(alt["comp_m"]), "mission.vtol.baro_thrust_comp_m (0 = not calibrated: no compensation)")
    w("/* ---- v0.8 take-off check (sim/vertical_ctrl.py docstring 6; position_hold.takeoff_check_ASSUMED; 0 ticks = off) ---- */")
    tc = (v.get("position_hold") or {}).get("takeoff_check_ASSUMED") or {}
    define("VC_TAKEOFF_CLIMB_M", cf(vp.takeoff_climb_m), f"climb_m = {float(tc.get('climb_m', 0.0)):g} m")
    define("VC_TAKEOFF_TICKS", f"{vp.takeoff_ticks}U", f"ticks_ge(within_s = {float(tc.get('within_s', 0.0)):g} s)")
    w("/* ---- v0.8 supervisor switches (sim/onboard.py SupervisorParams.gps_hold / takeoff_check) ---- */")
    define("ONB_V_GPS_HOLD", "1" if spv.gps_hold else "0", "position_hold.enabled: ON gate (REJ_GPS), V11-V14, TELEM v3")
    define("ONB_V_TAKEOFF_CHECK", "1" if spv.takeoff_check else "0", "V15 (position_hold.takeoff_check_ASSUMED present)")
    _render_v09_vertical(define, w, pr["v09"], vp)
    _render_v09_power(define, w, pr["pwr"], spv)
    pp = pr.get("pp")
    if pp is None:
        w("/* no position hold in this profile (position_hold.enabled false): POS_* inert */")
        return
    w("/* ---- v0.8 FCU position hold (sim/position_ctrl.py PositionParams.from_params(profile); SI, float32; steps) ---- */")
    define("POS_STEP_TICKS", f"{pp.step_ticks}U", f"control.loop_hz {pp.fcu_hz} / position_hold.loop_hz {pp.loop_hz}")
    define("POS_T", cf(pp.T), "1 / position_hold.loop_hz [s]")
    define("POS_G", cf(pp.g), "sim/state_machine.py G0")
    define("POS_TW", cf(pp.tw), f"model T/W = T_max {d['t_max']:g} N / (m {d['m_full']:.4f} kg g) (GPS pod in the CAD mass)")
    define("POS_KP_POS", cf(pp.kp_pos), "[1/s] v_sp per metre")
    define("POS_KV", cf(pp.kv), "[1/s] m/s^2 per m/s")
    define("POS_KI", cf(pp.ki), "[1/s^2]")
    define("POS_KD", cf(pp.kd), "[s] velocity-derivative feedback")
    define("POS_ALPHA_D", cf(pp.alpha_d), f"1 - exp(-2 pi kd_lpf_hz {pp.kd_lpf_hz:g} Hz T)")
    define("POS_I_MAX", cf(pp.i_max), "integrator limit [m/s^2]")
    define("POS_V_MAX", cf(pp.v_max), "position_hold.v_max_mps_ASSUMED")
    define("POS_V_MAX_LAND", cf(pp.v_max_land), "LAND: slower return to the hold point")
    define("POS_FLARE_SPEED", cf(pp.flare_speed), "position_hold.flare_speed_mps_ASSUMED (LAND below the flare altitude)")
    define("POS_FLARE_ALTITUDE_M", cf(pp.flare_altitude_m), "mission.vtol.flare_altitude_m (this profile)")
    define("POS_COS_CAP", cf(pp.cos_cap), f"cos(tilt_cap_deg {pp.tilt_cap_deg:g})")
    define("POS_COS_FLARE_CAP", cf(pp.cos_flare_cap), f"cos(min(cap, flare_tilt_cap_deg {pp.flare_tilt_cap_deg:g}))")
    define("POS_COS_EXCESS", cf(pp.cos_excess), f"cos(tilt_guard_excess_deg {pp.tilt_guard_excess_deg:g})")
    define("POS_SIN_EXCESS", cf(pp.sin_excess), "sin(tilt_guard_excess_deg)")
    define("POS_MARGIN_G", cf(pp.margin_g), "tilt margin rule: m = min(margin_g, margin_frac max(k_T T/W - 1, 0))")
    define("POS_MARGIN_FRAC", cf(pp.margin_frac), "")
    define("POS_H_ENABLE_M", cf(pp.h_enable_m), "engage above this NAV height")
    define("POS_FENCE_M", cf(pp.fence_m), f"position_hold.fence_m ({pp.profile}): V12 LAND")
    define("POS_SPIN_Z_RAD", cf(pp.spin_z_rad), f"guard |w_z| > spin_guard rate_dps {pp.spin_z_dps:g}")
    define("POS_SPIN_XY_RAD2", cf(pp.spin_xy_rad2), f"guard |w_xy|^2 > (release_dps {pp.spin_xy_dps:g})^2 [(rad/s)^2]")
    define("POS_GUARD_STEPS", f"{pp.guard_steps}U", f"spin_guard hold_s {pp.guard_hold_s:g} s")
    define("POS_TIMEOUT_STEPS", f"{pp.timeout_steps}U", f"gps_loss.timeout_s {pp.gps_timeout_s:g} s -> attitude only")
    define("POS_LAND_STEPS", f"{pp.land_steps}U", f"+ land_after_s {pp.gps_land_after_s:g} s -> V11 LAND request")
    define("POS_PLAUS_LIMITED_STEPS", f"{pp.plaus_limited_steps}U", f"plausibility (a) {pp.plaus_limited_s:g} s")
    define("POS_PLAUS_VEXC_STEPS", f"{pp.plaus_vexc_steps}U", f"plausibility (b) {pp.plaus_v_excess_s:g} s")
    define("POS_PLAUS_RESET_WINDOW_STEPS", f"{pp.plaus_reset_window_steps}U", f"plausibility (c) window {pp.plaus_reset_window_s:g} s")
    define("POS_PLAUS_RESETS", f"{pp.plaus_resets}U", "plausibility (c) resets in the window")
    define("POS_PLAUS_VT", cf(pp.v_max + pp.plaus_v_excess_mps), "plausibility (b): v_max + plaus_v_excess_mps")
    define("POS_AUTHORITY_MIN_KTW", cf(pp.authority_min_ktw), "authority_check_ASSUMED.min_kT_TW: V14 LAND request")
    define("POS_TILT_FADE_M", cf(pp.tilt_fade_m), "touchdown tilt fade height")
    define("POS_SLEW_PER_TICK", cf(pp.slew_per_tick), f"radians(tilt_rate_dps {pp.tilt_rate_dps:g}) / control.loop_hz")
    define("POS_TOUCHDOWN_KEEP_TRIM", "1" if pp.touchdown_policy == "keep_trim" else "0",
           f"position_hold.touchdown_policy.policy = {pp.touchdown_policy} (v0.8.1: 1 keep_trim, 0 the v0.8 fade)")
    define("POS_TOUCHDOWN_POLICY", str(POS_TD_POLICY_CODES[pp.touchdown_policy]),
           f"v0.9 policy code: 0 fade, 1 keep_trim, 2 keep_vel (pos_params_t.keep_trim)")


def _render_v09_vertical(define, w, v9: dict, vp) -> None:
    """v0.9 vertical-controller macros of one profile (sim/vertical_ctrl.py docstrings 1b, 1c, 3b, 4c, 5b)."""
    w("/* ---- v0.9 take-off / reference / touchdown (sim/vertical_ctrl.py 1b 1c 3b 4c 5b; mission.vtol.takeoff_v09,")
    w(" * ref_approach_*, land_ref_rebase, touchdown.release_*_ASSUMED; core/vertical_ctrl.h vc_params_t) ---- */")
    define("VC_RAMP_TOTAL_TICKS", f"{v9['rt']}U", "ramp_ticks + ceil((u_max - u_start) loop_hz / slew_per_s): the ramp is over")
    define("VC_TO_TWO_SLOPE", "1" if v9["two"] else "0", "takeoff_v09.u_start > 0: the two-slope take-off ceiling (3b)")
    define("VC_TO_U_START", cf(v9["u_start"]), "takeoff_v09.u_start (end of the pad segment)")
    define("VC_TO_S1", cf(v9["s1"]), "u_start / ramp_ticks: ceiling per tick on the pad segment")
    define("VC_TO_S2", cf(v9["s2"]), "slew_per_s / loop_hz: ceiling per tick on the second segment")
    define("VC_TO_BOOST", "1" if v9["boost"] else "0", "take-off thrust search on (boost_per_s > 0 and the take-off check)")
    define("VC_TO_SEARCH_F", cf(v9["search_f"]), "1 - 2 boost_per_s / (loop_hz u_hover): k_T x this per searching tick")
    define("VC_TO_SEARCH_VZ", cf(v9["search_vz"]), "TAKEOFF_SEARCH_VZ_FRAC_ASSUMED x climb_rate_mps: search while v_z <= this")
    define("VC_KT_ALPHA_TO", cf(v9["kt_alpha_to"]), "1 - exp(-dt / takeoff_v09.learn_tau_s): k_T from the lift-off (4c); 0 off")
    define("VC_TO_LIFTOFF", "1" if v9["liftoff"] else "0", "the lift-off latch runs (learn_tau_s > 0 or ref_hold)")
    define("VC_TO_AIRBORNE_MPS2", cf(v9["airborne_mps2"]), "g (1 + airborne_fz_frac): the lift-off specific force")
    define("VC_TO_REF_HOLD", "1" if v9["ref_hold"] else "0", "takeoff_v09.ref_hold: the reference waits for the lift-off")
    define("VC_REF_APPROACH_SHIFT", str(v9["approach_shift"]), "mission.vtol.ref_approach_shift (1b; 0 = off)")
    define("VC_REF_APPROACH_MIN_UM", f"{v9['approach_min_um']}L",
           f"round(ref_approach_min_mps {vp.ref_approach_min_mps:g} x 1e6 / loop_hz), >= 1 [um per tick]")
    define("VC_LAND_REF_REBASE", "1" if v9["land_rebase"] else "0", "mission.vtol.land_ref_rebase (1c)")
    define("VC_TD_RELEASE", "1" if v9["td_rel"] else "0", "touchdown.release_per_s_ASSUMED > 0 (5b)")
    define("VC_TD_REL_STEP", cf(v9["td_step"]), f"release_per_s {vp.td_release_per_s:g} x dt: bias step per tick")
    define("VC_TD_REL_MAX", cf(v9["td_max"]), "release_max_ASSUMED: the bias limit")
    define("VC_TD_REL_OK", cf(v9["td_ok"]), "'still' confirms with td_bias <= this (the reference's -max + 1e-9, see the "
           "generator's td_release_threshold)")


def _render_v09_power(define, w, pw: dict | None, spv) -> None:
    """v0.9 power-rule macros of one profile (sim/onboard.py PowerMonitor; mission.vtol.power_rules_ASSUMED)."""
    w("/* ---- v0.9 power rules (sim/onboard.py PowerMonitor / power_fields; Q4 codes = round(16 V / adc_v_per_count);")
    w(" * a firmware env picks the build with -D PWR_SRC=1 (wired) / 2 (wired_buffer); battery builds read none of these) ---- */")
    define("PWR_RULES", "1" if pw else "0", "mission.vtol.power_rules_ASSUMED present")
    if not pw:
        return
    for k in ("pw_shift", "pw_arm_lo_q", "pw_arm_hi_q", "pw_buf_arm_lo_q", "pw_buf_arm_hi_q", "pw_lost_q", "pw_lost_margin_q",
              "pw_lost_ticks", "pw_sag_q", "pw_sag_clear_q", "pw_sag_warn_ticks", "pw_sag_land_ticks", "pw_sag_release_ticks",
              "pw_buf_low_q", "pw_buf_ticks", "pw_buf_res_q", "pw_buf_res_ticks", "pw_d2_q", "pw_d2_ticks", "pw_i8buf_q"):
        unit = "Q4" if k.endswith("_q") else ("shift" if k == "pw_shift" else "ticks")
        val = pw[k]
        extra = f" = {val * pw['pw_v_per_count'] / 16.0:.4f} V" if k.endswith("_q") else ""
        define(f"PWR_{k[3:].upper()}", f"{val}{'U' if unit != 'shift' else ''}", f"SupervisorParams.{k} [{unit}]{extra}")
    mul, add, sh = pw["vaux"]
    define("PWR_VAUX_MUL", f"{mul}UL", "TELEM v4 / v5 v_aux = min((y_Q4 MUL + ADD) >> SHIFT, 255) = power_link.quant_v(volts)")
    define("PWR_VAUX_ADD", f"{add}UL", "(exhaustively checked over every Q4 code)")
    define("PWR_VAUX_SHIFT", str(sh), "")
    lim = pw["lim"]
    w("/* tethered limits of the wired builds (sim/onboard.py tether_limits: min(profile, power_rules_ASSUMED.tether_limits);")
    w(" * endurance / hard cutoff from tether_endurance) - core/fc_config.c fills onb_params_t with them when PWR_SRC != 0 */")
    define("ONB_W_HOVER_DM", str(lim.hover_dm), f"profile {spv.hover_dm}")
    define("ONB_W_ALT_MIN_DM", str(lim.alt_min_dm), f"profile {spv.alt_min_dm}")
    define("ONB_W_ALT_MAX_DM", str(lim.alt_max_dm), f"profile {spv.alt_max_dm}")
    define("ONB_W_CEILING_M", cf(lim.ceiling_m), f"profile {spv.ceiling_m:g} m")
    define("ONB_W_GEOFENCE_M", cf(lim.geofence_m), f"profile {spv.v_geofence_m:g} m")
    define("ONB_W_ENDURANCE_TICKS", f"{lim.endurance_ticks}UL", f"profile {spv.endurance_ticks}")
    define("ONB_W_HARD_CUTOFF_TICKS", f"{lim.hard_cutoff_ticks}UL", f"profile {spv.hard_cutoff_ticks}")


def _render_v09_power_hw(d: dict, define, w) -> None:
    """v0.9 power ADC channels, the buffer-current sign GPIO and the per-channel two-point calibration (params
    power.adc_calibration_ASSUMED {v_in|v_tether|v_buffer: {points: [[code, volts], [code, volts]]}}; absent = identity).
    The pins: params flight_computer.pins v_tether_adc / v_buffer_adc / buf_isign (fw09 SPEC 5), already claimed by
    chip_pins with the FCU's other pins (no pin used twice)."""
    p = d["p"]
    pins = p["flight_computer"]["pins"]
    w("")
    w("/* ---- v0.9 power monitor hardware (FCU; wired builds only).  Pins from params flight_computer.pins (ASSUMED until")
    w(" * the v0.9 carrier PCB is routed): V_tether before D1 and V_buffer before D2 on free ADC inputs with the PF0")
    w(" * divider (47k/4.7k), the hybrid board's buffer-current sign on a free GPIO (telemetry flag only) ---- */")
    for key, macro in (("v_tether_adc", "PWR_V_TETHER_ADC_CHANNEL"), ("v_buffer_adc", "PWR_V_BUFFER_ADC_CHANNEL")):
        port, bit = d["fcu_pins"][key]
        src = f"flight_computer.pins.{key} = {pins[key]}"
        if port != "F" or not 0 <= bit <= 3:
            raise GenError(f"flight_computer.pins.{key}: P{port}{bit} - the power ADC inputs must be PF0..PF3 (PF4-PF7 "
                           "are the JTAG pins)")
        define(macro, str(bit), src)
    port, bit = d["fcu_pins"]["buf_isign"]
    src = f"flight_computer.pins.buf_isign = {pins['buf_isign']}"
    w(f"#define PIN_BUF_ISIGN_PORT                 {port}  /* {src} */")
    define("PIN_BUF_ISIGN_BIT", str(bit), "")
    cal = ((p.get("power") or {}).get("adc_calibration_ASSUMED") or {})
    vpc = float(d["vbat_per_count"])
    for ch, tag in (("v_in", "IN"), ("v_tether", "TET"), ("v_buffer", "BUF")):
        gain_q15, off = 32768, 0
        src = "identity (no calibration in params power.adc_calibration_ASSUMED)"
        c = cal.get(ch)
        if c:
            (c1, v1), (c2, v2) = c["points"]
            if not (0 <= c1 < c2 <= 1023 and v2 > v1):
                raise GenError(f"power.adc_calibration_ASSUMED.{ch}: need two points with code1 < code2, volts1 < volts2")
            gain = (v2 - v1) / (c2 - c1) / vpc            # true volts per count / the nominal one
            off = round(v1 / vpc - gain * c1)
            gain_q15 = round(gain * 32768)
            if not (16384 <= gain_q15 <= 65535 and -64 <= off <= 64):
                raise GenError(f"power.adc_calibration_ASSUMED.{ch}: gain {gain:.4f} / offset {off} outside 0.5..2 / +-64")
            src = f"two-point: ({c1}, {v1} V), ({c2}, {v2} V)"
        define(f"PWR_CAL_{tag}_GAIN_Q15", f"{gain_q15}UL", f"code' = ((code x this + 16384) >> 15) + offset: {src}")
        define(f"PWR_CAL_{tag}_OFFSET", str(off), "")


def _render_gps(d: dict, define, w, gx: dict) -> None:
    """The GPS receiver / filter block (the same for every profile; v0.8: the NAV's, v0.8.1: the GNS board's)."""
    gp, gn, cfg = gx["gp"], gx["gains"], gx["cfg"]
    w("")
    w("/* ---- GPS receiver + filter (sim/gps_nav.py NavGpsParams.from_params; params flight_computer.nav.gps, the same for")
    w(" * every profile; v0.8.1: on the GNS board, core/gns_node.h - v0.8: on the NAV): USART1 PD2 RXD1 / PD3 TXD1 through")
    w(" * the level shifter; the fixed gains are sim/gps_nav.py compute_gains in float64 (no covariance propagation) ---- */")
    u = d["gps_uart"]
    define("GPS_BAUD", f"{gx['baud']}UL", "flight_computer.nav.gps.baud, 8N1 (CFG-PRT)")
    define("GPS_UBRR", str(u[0]), f"U2X={u[1]}, error {u[2] * 100:+.2f} % (= flight_computer.nav.gps.ubrr, checked)")
    define("GPS_U2X", str(u[1]), "")
    define("GPS_RATE_HZ", cf(gp.rate_hz), "flight_computer.nav.gps.rate_hz (CFG-RATE)")
    define("GPS_MEAS_MS", f"{gx['meas_ms']}U", "CFG-RATE measurement period")
    define("GPS_CFG_N_WAIT", f"{cfg.n_wait}U", f"boot wait {gpn.CFG_BOOT_WAIT_S:g} s (ASSUMED)")
    define("GPS_CFG_N_ACK", f"{cfg.n_ack}U", f"ACK timeout {gpn.CFG_ACK_TIMEOUT_S:g} s (ASSUMED)")
    define("GPS_CFG_N_SETTLE", f"{cfg.n_settle}U", f"after the 9600-baud CFG-PRT {gpn.CFG_SETTLE_S:g} s (ASSUMED)")
    define("GPS_CFG_N_VER", f"{cfg.n_ver}U", f"MON-VER timeout {gpn.CFG_VER_TIMEOUT_S:g} s (ASSUMED)")
    define("GPS_STEP_TICKS", f"{gp.step_ticks}U", f"control.loop_hz / fusion_hz {gp.fusion_hz}")
    define("GPS_EPOCH_SPLIT_TICKS", "4", "flight_computer.nav.gps.epoch_split_ticks (E0..E3, checked)")
    define("GPS_MS_PER_TICK", f"{gp.ms_per_tick}U", "1000 / loop_hz")
    define("GPS_LATENCY_MS", f"{gp.latency_ms}U", f"latency_s_ASSUMED {gp.latency_s:g} s")
    define("GPS_LOOP_HZ_F", cf(float(gp.loop_hz)), "T = n / this")
    define("GPS_T_STEP", cf(gp.t_step), "step_ticks / loop_hz")
    define("GPS_INV_STEP", cf(1.0 / gp.step_ticks), "1 / step_ticks (the mean force of a cycle)")
    for i, ax in enumerate("XYZ"):
        define(f"GPS_LEVER_B_{ax}_M", cf(gp.lever_b_m[i] if gp.lever_comp else 0.0), f"antenna from the CG, body ({gp.lever_source})")
    define("GPS_PSI0_RAD", cf(gx["psi0"]), f"-pad_azimuth_deg {gp.pad_azimuth_deg:g}")
    define("GPS_C_PSI0", cf(math.cos(gx["psi0"])), "")
    define("GPS_S_PSI0", cf(math.sin(gx["psi0"])), "")
    define("GPS_GATE_MIN_SV", f"{gp.gate_min_sv}U", "ON gate gps_gate_on_ASSUMED.min_sv")
    define("GPS_GATE_MAX_HACC_MM", f"{gx['gate_max_hacc_mm']}UL", f"largest mm with mm / 1000 <= max_hacc_m {gp.gate_max_hacc_m:g}")
    define("GPS_GATE_MAX_SACC_CMS", f"{gx['gate_max_sacc_cms']}UL", f"largest cm/s with x / 100 <= max_sacc_mps {gp.gate_max_sacc_mps:g}")
    define("GPS_GATE_STABLE_TICKS", f"{gp.gate_stable_ticks}UL", f"stable_s {gp.gate_stable_s:g} s")
    define("GPS_OK_MIN_SV", f"{gp.ok_min_sv}U", "in flight: usable (ASSUMED)")
    define("GPS_OK_MAX_HACC_MM", f"{gx['ok_max_hacc_mm']}UL", f"usable: hAcc <= {gp.ok_max_hacc_m:g} m (ASSUMED)")
    w(f"/* gains (q_acc {gp.q_acc:g}, q_b {gp.q_b:g}, sigma_p {gp.sigma_p_m:g} m, sigma_v {gp.sigma_v_mps:g} m/s, sigma_b0 "
      f"{gp.sigma_b0_mps2:g}; acquisition = epoch {gp.acq_gain_epoch} of the take-off Riccati, {gp.acq_epochs} epochs) */")
    for tag, gs in (("STEADY", gn.steady), ("ACQ", gn.acq)):
        for r in range(3):
            for c in range(2):
                define(f"GPS_K_{tag}_PV_{r}{c}", cf(gs.k_pv[r][c]), "")
            define(f"GPS_K_{tag}_P_{r}", cf(gs.k_p[r]), "")
            define(f"GPS_K_{tag}_V_{r}", cf(gs.k_v[r]), "")
    define("GPS_GATE_V2", cf(gp.gate_sigma * gp.gate_sigma * gn.s_vv), f"(gate_sigma {gp.gate_sigma:g})^2 S_vv {gn.s_vv:.6g}")
    define("GPS_GATE_P2", cf(gp.gate_p_m * gp.gate_p_m), f"gate_p {gp.gate_p_m:g} m squared")
    define("GPS_N_CORR", f"{gp.n_corr}U", "buffer entries corrected after a fusion")
    define("GPS_ACQ_EPOCHS", f"{gp.acq_epochs}U", "")
    define("GPS_RESET_AFTER_EPOCHS", f"{gp.reset_after_epochs}U", f"reset_after_s {gp.reset_after_s:g} s")
    define("GPS_VEL_RESET_EPOCHS", f"{gp.vel_reset_epochs}U", "")
    define("GPS_FRESH_TICKS", f"{gp.fresh_ticks}UL", f"FIX_OK: lost_after_epochs {gp.lost_after_epochs}")
    define("GPS_YAW_CORRECTION", "1" if gp.yaw_correction else "0", "")
    define("GPS_YAW_WINDOW_STEPS", f"{gx['yaw_window_steps']}U", f"yaw_window_s {gp.yaw_window_s:g} s")
    define("GPS_YAW_MIN_DV2", cf(gp.yaw_min_dv_mps ** 2), f"yaw_min_dv {gp.yaw_min_dv_mps:g} m/s squared")
    define("GPS_YAW_SIGMA_DV2", cf(gp.yaw_sigma_dv_mps ** 2), f"yaw_sigma_dv {gp.yaw_sigma_dv_mps:g} m/s squared")
    define("GPS_YAW_Q_RAD2", cf(math.radians(1.0) ** 2 * gp.yaw_q_deg2_per_s), f"{gp.yaw_q_deg2_per_s:g} deg^2/s in rad^2/s")
    define("GPS_P_PSI0", cf(math.radians(gp.yaw_sigma0_deg) ** 2), f"(sigma0 {gp.yaw_sigma0_deg:g} deg)^2")
    define("GPS_HEAD_OK_RAD2", cf(math.radians(gp.head_ok_deg) ** 2), f"head_ok {gp.head_ok_deg:g} deg squared")


def _render_glink(d: dict, define, w, gl: dict) -> None:
    """The v0.8.1 NAV <-> GNS link block (params flight_computer.glink / gns; sim/glink_proto.py)."""
    from sim import glink_proto as glp
    est = gl["est"]
    w("")
    w("/* ---- v0.8.1 glink: NAV USART1 <-> GNS USART0 (params flight_computer.glink / gns; sim/glink_proto.py, core/glink_proto.h)")
    w(" * - STATE 60 B NAV -> GNS and RESULT 26 B back once per 50 Hz cycle, ISR-driven both ends, CRC-16; the GNS board")
    w(" * (core/gns_node.h) runs the GPS receiver on ITS USART1 (GPS_BAUD / GPS_UBRR below) ---- */")
    define("GLINK_BAUD", f"{gl['baud']}UL", "flight_computer.glink.baud, 8N1")
    define("GLINK_UBRR", str(gl["ubrr"]), f"U2X = 0, error {gl['err'] * 100:+.2f} % (= flight_computer.glink.ubrr, checked)")
    define("GLINK_U2X", "0", "both ends normal speed (as the ilink)")
    define("GLINK_LATENCY_TICKS", f"{gl['latency']}U",
           "glink.latency_ticks_ASSUMED: NAV ticks from the STATE sample to the nlink frame with the TILT part (ASSUMED "
           "until the four-core co-simulation measures it)")
    define("GLINK_STATE_FRAME_BYTES", f"{glp.STATE_FRAME_BYTES}U", f"{gl['state_s'] * 1e3:.2f} ms on the line")
    define("GLINK_RESULT_FRAME_BYTES", f"{glp.RESULT_FRAME_BYTES}U", f"{gl['result_s'] * 1e3:.2f} ms on the line")
    define("GLINK_STATE_FRAME_US", f"{round(gl['state_s'] * 1e6)}UL", "")
    define("GLINK_RESULT_FRAME_US", f"{round(gl['result_s'] * 1e6)}UL", "")
    define("GNS_LOOP_HZ", f"{gl['gns_hz']}U", "flight_computer.gns.loop_hz: one STATE / RESULT pair per cycle")
    w(f"/* timing ESTIMATE (sim/glink_proto.py timing_estimate, v0.8 C cycle counts): the RESULT reaches the NAV "
      f"{est['result_at_nav_ms']:.2f} ms after")
    w(f" * the sample tick's start; the earliest feasible L {est['latency_ticks_min_est']}, margin at L = {gl['latency']}: "
      f"{est['margin_ms_at_params_L']:.2f} ms */")


def render(d: dict) -> str:
    """The header text.  fw09 G1: the whole header is rendered once per constant set (the rocket = ``d``, every available
    VTOL configuration ``d['vcfg'][name]``); lines equal in every set are written once, each run of differing lines
    becomes one ``#if FC_VEHICLE == FC_VCFG_<NAME>`` block with every set's lines (battery, tether, hybrid, then the
    rocket in the #else).  A differing line that is not a #define of VCFG_MACROS (the same name in every set) is refused:
    the configurations may differ only in the vehicle constants."""
    sets = [(nm, d["vcfg"][nm]) for nm in VCFGS if nm in (d.get("vcfg") or {})]
    rocket = _render_one(d, d)
    if not sets:
        return "\n".join(rocket)
    texts = [(nm, _render_one(dc, d)) for nm, dc in sets] + [("rocket", rocket)]
    n = len(rocket)
    for nm, t in texts:
        if len(t) != n:
            raise GenError(f"the VTOL {nm} header has {len(t)} lines, the rocket's {n}: the configurations differ in "
                           "structure, not only in the vehicle constants")
    out: list[str] = []
    i = 0
    while i < n:
        if all(t[i] == rocket[i] for _, t in texts):
            out.append(rocket[i])
            i += 1
            continue
        j = i
        while j < n and not all(t[j] == rocket[j] for _, t in texts):
            names = set()
            for nm, t in texts:
                m = re.match(r"#define (\w+)\s", t[j])
                names.add(m.group(1) if m else None)
            if len(names) != 1 or None in names or not names <= VCFG_MACROS:
                raise GenError(f"header line {j + 1} differs between the vehicle configurations and is not a #define of "
                               f"VCFG_MACROS: {[t[j][:90] for _, t in texts]}")
            j += 1
        for k, (nm, t) in enumerate(texts):
            if nm == "rocket":
                out.append(f"#else  /* {VCFG_MACRO['rocket']}: the ballistic builds, host builds without FC_VCFG */")
            else:
                out.append(f"#{'if' if k == 0 else 'elif'} FC_VEHICLE == {VCFG_MACRO[nm]}")
            out.extend(t[i:j])
        out.append("#endif  /* FC_VEHICLE */")
        i = j
    return "\n".join(out)


def _render_vehicle_select(top: dict, define, w) -> None:
    """fw09 G1: the FC_VCFG -> FC_VEHICLE selection and its build refusals (identical in every set; ``top`` = derive())."""
    errs = top.get("vcfg_errors") or {}
    w("/* ---- vehicle configuration (fw09 G1, SPEC_firmware_v09 3.2): every build flies the vehicle constants of ONE CAD")
    w(" * report - MIX_L_V_M, FC_HOVER_THROTTLE, VEH_*, VC_U_HOVER, VC_INV_U_HOVER, VC_TO_SEARCH_F, POS_TW, FC_PARAMS_TAG,")
    w(" * FC_VEHICLE_NAME are written once per configuration below (#if FC_VEHICLE == ...); every other macro is the same in")
    w(" * every configuration (the generator checks it).  platformio.ini sets -D FC_VCFG in every VTOL env of the NAV, FCU,")
    w(" * GNS and COM (FC_VCFG = PWR_SRC of the FCU's power builds):")
    for nm in VCFGS:
        rp = rel(vtol_report_path(nm))
        if nm in (top.get("vcfg") or {}):
            rp = rel(top["vcfg"][nm]["cad_path"])
        envs = f"<role>{VCFG_ENV_SUFFIX[nm]}" if VCFG_ENV_SUFFIX[nm] else "<role> (the plain VTOL envs)"
        w(f" *   {VCFG_ID[nm]} {VCFG_MACRO[nm]:<16} VTOL {nm:<8} envs {envs:<28} {rp}")
    w(f" *   {VCFG_ID['rocket']} {VCFG_MACRO['rocket']:<16} rocket        every ballistic build (FC_VCFG ignored) "
      f"{rel(top['cad_path'])}")
    w(" * A VTOL AVR build without FC_VCFG, with FC_VCFG 3, or of a configuration without a usable CAD report does not")
    w(" * compile; host builds without FC_VCFG (unit tests) get the rocket set; an FCU build's PWR_SRC must equal FC_VCFG.")
    w(" * FC_PARAMS_TAG names the configuration, so a mixed image set is flagged (NAV_RC_TAG_MISMATCH / LOG_COM_TAG_MISMATCH)")
    w(" * ---- */")
    for nm in VCFGS + ("rocket",):
        define(VCFG_MACRO[nm], str(VCFG_ID[nm]), f"VTOL {nm}" if nm != "rocket" else "the rocket (ballistic builds)")
    w("#if defined(FC_MODE_BALLISTIC) || (!defined(FC_MODE_VTOL) && MISSION_MODE == MISSION_MODE_BALLISTIC)")
    define("FC_VEHICLE", "FC_VCFG_ROCKET", "the ballistic mission flies the rocket")
    w("#elif defined(FC_VCFG)")
    w("#if FC_VCFG < FC_VCFG_BATTERY || FC_VCFG > FC_VCFG_ROCKET")
    w('#error "FC_VCFG must be 0 (VTOL battery), 1 (VTOL tether), 2 (VTOL hybrid) or 3 (the rocket: host builds only)"')
    w("#endif")
    w("#if defined(__AVR__) && FC_VCFG == FC_VCFG_ROCKET")
    w('#error "a VTOL firmware image must not fly the rocket\'s vehicle constants (fw09 G1): -D FC_VCFG=0, 1 or 2"')
    w("#endif")
    define("FC_VEHICLE", "FC_VCFG", "")
    w("#elif defined(__AVR__)")
    w('#error "a VTOL firmware build names its vehicle configuration (fw09 G1): -D FC_VCFG=0 (battery), 1 (tether), '
      '2 (hybrid)"')
    w("#else")
    define("FC_VEHICLE", "FC_VCFG_ROCKET", "host builds without -D FC_VCFG (unit tests): the rocket set")
    w("#endif")
    w("#if defined(FC_ROLE_FCU) && FC_VEHICLE != FC_VCFG_ROCKET && \\")
    w("    ((defined(PWR_SRC) && PWR_SRC != FC_VEHICLE) || (!defined(PWR_SRC) && FC_VEHICLE != FC_VCFG_BATTERY))")
    w('#error "an FCU build\'s PWR_SRC must equal FC_VCFG (battery: no PWR_SRC, tether: 1, hybrid: 2)"')
    w("#endif")
    for nm in VCFGS:
        if nm in errs:
            why = re.sub(r"[^ -~]", "?", errs[nm]).replace('"', "'").replace("\\", "/")
            w(f"#if FC_VEHICLE == {VCFG_MACRO[nm]}")
            w(f'#error "no usable CAD report for the VTOL {nm} configuration (fw09 G1): {why[:300]}"')
            w("#endif  /* FC_VEHICLE */")


def _render_one(d: dict, top: dict) -> list[str]:
    """One constant set's header lines (``top`` = derive(): the shared sources and the configuration list)."""
    p, ms, sp = d["p"], d["ms"], d["sp"]
    pins = p["flight_computer"]["pins"]
    out: list[str] = []
    w = out.append

    def define(name: str, value: str, comment: str = "") -> None:
        w(f"#define {name:<34} {value}" + (f"  /* {comment} */" if comment else ""))

    w("/* params_gen.h - GENERATED by tools/gen_firmware_params.py. DO NOT EDIT.")
    w(" *")
    w(" * Sources (sha256):")
    for path, h in top["sources"]:
        w(f" *   {path:<42} {h}")
    w(" * (canonical) = sha256 of the file as canonical JSON (sorted keys, no whitespace) without its volatile build")
    w(" * timings " + " / ".join(".".join(k) for k in CAD_REPORT_VOLATILE) + ", so a CadQuery rebuild of the same vehicle")
    w(" * leaves this header unchanged.")
    w(f" * CAD report generator: {top['cad_generator']}; mass properties recomposed from its parts")
    w(" * without the launch pad (sim/vehicle.py rules).")
    w(" * Re-run the generator after changing any source; the build never edits this file.")
    w(" */")
    w("#ifndef PARAMS_GEN_H")
    w("#define PARAMS_GEN_H")
    w("")
    w(f'#define FC_PARAMS_SHA256_PREFIX            "{top["sources"][0][1][:16]}"')
    w("")
    w("/* ---- mission mode (params.json mission.mode, v0.7; a build-time choice: the COM sends TELEM2 / expects SNAP2 and")
    w(" * the FCU runs the VTOL supervisor + vertical controller when MISSION_MODE == MISSION_MODE_VTOL) ---- */")
    define("MISSION_MODE_BALLISTIC", "0", "the v0.4 - v0.6.3 climb / CUT / RELEASE mission (= LP_MODE_BALLISTIC)")
    define("MISSION_MODE_VTOL", "1", "v0.7 vertical take-off and landing (= LP_MODE_VTOL)")
    define("MISSION_MODE", "MISSION_MODE_VTOL" if d["mode"] == lp.MODE_VTOL else "MISSION_MODE_BALLISTIC",
           f"mission.mode = {d['mode']}")
    w("")
    _render_vehicle_select(top, define, w)
    w("")
    if d["vcfg_name"] == "rocket":
        define("FC_PARAMS_TAG", f"0x{d['params_tag']:02X}u",
               "low byte of FC_PARAMS_SHA256_PREFIX; COM IL_STAT params_tag, compared by the FCU")
    else:
        define("FC_PARAMS_TAG", f"0x{d['params_tag']:02X}u",
               f"VTOL {d['vcfg_name']}: low byte of sha256(params sha | vtol_{d['vcfg_name']} | CAD canonical sha | "
               f"{d['params_tag_k']}); the NAV / FCU / COM tag checks")
    w(f'#define FC_VEHICLE_NAME                    "{"rocket" if d["vcfg_name"] == "rocket" else "vtol_" + d["vcfg_name"]}"'
      f"  /* {rel(d['cad_path'])} */")
    w(f'#define FC_PRESET                          "{p["meta"].get("preset", "")}"')
    w("")
    w("/* ---- MCU, control tick (Timer2 CTC), PWM (Timer1/3 fast PWM mode 14) ---- */")
    define("FC_F_CPU_HZ", f"{d['f_cpu']}UL", "flight_computer.f_cpu_hz")
    define("FC_LOOP_HZ", str(d["loop_hz"]), "control.loop_hz")
    define("FC_DT_S", cf(d["dt"]), "1 / loop_hz")
    define("FC_TICK_US", f"{d['tick_us']}UL", "")
    define("FC_T2_PRESCALER", str(d["t2"][0]), "")
    define("FC_T2_CS_BITS", str(TIMER2_PRESCALERS[d["t2"][0]]), "CS22:0")
    define("FC_T2_OCR", str(d["t2"][1]), f"F_CPU/(presc*(OCR+1)) = {d['loop_hz']} Hz exactly")
    define("FC_T0_CS_BITS", str(TIMER0_PRESCALERS[d["t2"][0]]), "CS02:0, Timer0 = IMU read lead timer, same period")
    define("FC_IMU_LEAD_COUNTS", str(d["imu_lead_counts"]),
           f"{d['imu_lead_counts'] * d['count_s'] * 1e6:.0f} us before the tick (bus time {d['imu_read_s'] * 1e6:.0f} us x {IMU_LEAD_MARGIN})")
    define("FC_PWM_PRESCALER", str(d["presc"]), "flight_computer.pwm_timer_prescaler")
    define("FC_PWM_CS_BITS", str(TIMER13_PRESCALERS[d["presc"]]), "CSn2:0")
    define("FC_PWM_TOP", f"{d['top']}U", f"ICR1/ICR3 -> {d['frame_hz']} Hz frames")
    define("FC_PWM_FRAME_HZ", str(d["frame_hz"]), "servo.pwm_hz = flight_computer.servo_frame_hz")
    define("FC_PWM_COUNTS_PER_US", str(d["counts_per_us"]), f"{1 / d['counts_per_us']:.2f} us resolution")
    define("FC_LOG_HZ", str(d["log_hz"]), "flight_computer.log_hz")
    define("FC_LOG_DIV", str(d["log_div"]), "control ticks per log record")
    define("FC_BARO_HZ", str(round(d["loop_hz"] / d["baro_div"])), "control.baro_hz")
    define("FC_BARO_DIV", str(d["baro_div"]), "control ticks per barometer pressure sample")
    w("#define FC_BARO_MS5611                     1   /* hal/ms5611.c (GY-63) */")
    w("#define FC_BARO_BMP280                     2   /* 2026-11-10: hal/bmp280.c (Bosch BMP280, forced mode) */")
    w("#ifndef FC_BARO_SENSOR                     /* a PlatformIO env may force the other driver */")
    define("FC_BARO_SENSOR", {"ms5611": "FC_BARO_MS5611", "bmp280": "FC_BARO_BMP280"}[d["baro_sensor"]],
           f"flight_computer.nav.baro_sensor = {d['baro_sensor']} (src/hal/baro.h)")
    w("#endif")
    define("FC_IMU_HZ", str(d["imu_hz"]), "control.imu_hz (MPU-6050 internal sample rate)")
    define("FC_MPU_SMPLRT_DIV", str(d["smplrt_div"]), "1 kHz / (1 + div) with the DLPF on")
    define("FC_I2C_HZ", f"{d['i2c_hz']}UL", "flight_computer.i2c_hz")
    define("FC_TWBR", str(d["twbr"]), "TWPS = 0")
    define("FC_LOG_BAUD", f"{d['log_baud']}UL", "flight_computer.log_baud, 8N1")
    define("FC_LOG_UBRR", str(d["ubrr_log"][0]), f"U2X={d['ubrr_log'][1]}, error {d['ubrr_log'][2] * 100:+.2f} %")
    define("FC_LOG_U2X", str(d["ubrr_log"][1]), "")
    define("FC_ADC_PS_BITS", str(d["adc_ps"]), f"ADC clock F_CPU/{d['adc_div']} = {d['f_cpu'] / d['adc_div'] / 1e3:.0f} kHz")
    define("FC_G0", cf(d["g0"]), "sim/state_machine.py G0 (ISA altitude, altitude filter)")
    w("")
    w("/* ---- pins (FCU flight_computer.pins, NAV flight_computer.nav.pins, COM flight_computer.com.pins; each chip")
    w(" * checked on its own against the ATmega128 alternate functions, no pin used twice on a chip) ---- */")
    w("#define PWM_CH_OC1A 0")
    w("#define PWM_CH_OC1B 1")
    w("#define PWM_CH_OC1C 2")
    w("#define PWM_CH_OC3A 3")
    w("#define PWM_CH_OC3B 4")
    w("#define PWM_CH_OC3C 5")
    for i in range(4):
        oc = d["pwm_map"][f"vane_{i + 1}_servo"]
        define(f"PWM_OUT_VANE{i + 1}", f"PWM_CH_{oc}", pins[f"vane_{i + 1}_servo"])
    define("PWM_OUT_ESC", f"PWM_CH_{d['pwm_map']['esc']}", pins["esc"])
    define("PWM_OUT_RELEASE", f"PWM_CH_{d['pwm_map']['release_servo']}", pins["release_servo"])
    com_pins = p["flight_computer"]["com"]["pins"]
    chip_table = {"fcu": pins, "com": com_pins, "nav": p["flight_computer"]["nav"]["pins"]}
    for chip, key, macro in (("fcu", "led", "LED"), ("fcu", "buzzer", "BUZZER"), ("fcu", "arm_switch", "ARM_SW"),
                             ("nav", "led", "NAV_LED"),
                             ("com", "link_m0", "LINK_M0"), ("com", "link_m1", "LINK_M1"), ("com", "link_aux", "LINK_AUX"),
                             ("com", "lora_spi_nss", "LORA_NSS"), ("com", "lora_dio0", "LORA_DIO0"),
                             ("com", "lora_reset", "LORA_RESET"), ("com", "led", "COM_LED")):
        if (chip, key) == ("nav", "led"):
            w("/* NAV (flight_computer.nav.pins; its I2C and SPI pins are fixed by the ATmega128: TWI PD0/PD1, SPI PB0-PB3) */")
        if (chip, key) == ("com", "link_m0"):
            w("/* COM (flight_computer.com.pins): the radio pins keep their v0.5 names */")
        port, bit = d[f"{chip}_pins"][key]
        define(f"PIN_{macro}_PORT", port, chip_table[chip][key])
        define(f"PIN_{macro}_BIT", str(bit), "")
    define("VBAT_ADC_CHANNEL", str(d["adc_ch"]), pins["vbat_adc"])
    define("VBAT_V_PER_COUNT", cf(d["vbat_per_count"]), f"{d['vref']} V/1024 * ({d['r_top'] / 1e3:g}k+{d['r_bot'] / 1e3:g}k)/{d['r_bot'] / 1e3:g}k x vbat_scale")
    define("VBAT_CELLS", str(d["cells"]), "battery.cells")
    if d["lbat"] is not None:
        lb = d["lbat"]
        define("LOGIC_BAT_ADC_CHANNEL", str(lb["ch"]), pins["logic_bat_adc"])
        define("LOGIC_BAT_V_PER_COUNT", cf(lb["v_per_count"]),
               f"2.56 V/1024 * ({lb['r_top'] / 1e3:g}k+{lb['r_bot'] / 1e3:g}k)/{lb['r_bot'] / 1e3:g}k")
        define("LOGIC_BAT_TELEM", "1" if lb["telem"] else "0",
               "2026-11-10: the VTOL battery FCU (FC_VCFG 0) reads PF3 and sends lbat (TELEM v6 / v7; "
               "logic_battery_monitor.telemetry_builds)")
        define("LOGIC_BAT_DIV_TICKS", f"{int(d['loop_hz'])}U", "one PF3 conversion per second (the V_cell one of that tick "
                                                                "is skipped)")
        define("LOGIC_BAT_Q_X1024", f"{lb['q_x1024']}UL", f"lbat = (counts x {lb['q_x1024']} + 512) >> 10, 1..255 "
                                                         f"(link_proto.h LP_LBAT_SCALE {int(lp.LBAT_SCALE)} per volt)")
    _render_v09_power_hw(d, define, w)
    w("")
    w("/* ---- mixer plant (SPEC 8.3-8.5; SI, rad) ---- */")
    define("MIX_L_V_M", cf(d["l_v"]), f"z_cg {d['z_cg'] * 1e3:.3f} mm - vane_hinge_z {d['hinge_z'] * 1e3:.3f} mm")
    define("MIX_R_V_M", cf(d["r_v"]), "vane.force_centroid_r")
    define("MIX_S_V_M2", cf(d["s_v"]), "Layout.vane_area_mm2")
    define("MIX_CL_ALPHA", cf(d["cl_alpha"]), "vane.cl_alpha_per_rad")
    define("MIX_A_E_M2", cf(d["a_e"]), "pi/4 (duct_id^2 - (exit_hub_fraction duct_id)^2)")
    define("MIX_T_MAX_N", cf(d["t_max"]), "edf.thrust_nominal_N")
    define("MIX_THROTTLE_EXP", cf(d["n_exp"]), "edf.throttle_exponent")
    define("MIX_THROTTLE_EXP_INT", str(d["n_exp_int"]), "integer exponent fast path (0 = use powf)")
    define("MIX_DELTA_MAX_RAD", cf(d["delta_cmd"]), "vane.max_deflection_cmd_deg")
    define("MIX_DELTA_MECH_RAD", cf(d["delta_mech"]), "vane.max_deflection_mech_deg")
    define("MIX_PWM_CENTER_US", cf(d["center"]), "servo.pwm_center_us")
    define("MIX_PWM_US_PER_DEG", cf(d["us_per_deg"]), "servo.pwm_us_per_deg")
    for i in range(4):
        define(f"CAL_SERVO_SIGN_{i + 1}", cf(d["signs"][i]), "fc_config servo_signs")
        define(f"CAL_SERVO_TRIM_US_{i + 1}", cf(d["trims"][i]), "fc_config servo_trims_us")
        define(f"CAL_VANE_MIN_US_{i + 1}", cf(d["vane_lo"][i]), "centre + trim - mech limit")
        define(f"CAL_VANE_MAX_US_{i + 1}", cf(d["vane_hi"][i]), "centre + trim + mech limit")
    define("CAL_RELEASE_LATCHED_US", cf(d["latched"]), "fc_config release_servo")
    define("CAL_RELEASE_RELEASED_US", cf(d["released"]), "+ mission.deploy_servo_travel_deg")
    define("CAL_ESC_OFF_US", cf(d["esc"]["off"]), "fc_config esc_us.off")
    define("CAL_ESC_MIN_US", cf(d["esc"]["min"]), "fc_config esc_us.min (u = 0)")
    define("CAL_ESC_MAX_US", cf(d["esc"]["max"]), "fc_config esc_us.max (u = 1)")
    w("")
    w("/* ---- EDF model (SPEC 8.7, sim/edf.py) ---- */")
    define("EDF_TAU_S", cf(d["tau"]), "edf.thrust_time_constant_s")
    define("EDF_RHO", cf(d["rho"]), "aero.rho_kg_m3")
    define("EDF_P_NOM_W", cf(d["p_nom"]), "edf.power_nominal_W")
    define("EDF_OMEGA_MAX_RAD_S", cf(d["omega_max"]), "kv * V_nom * 2 pi / 60")
    define("EDF_SWIRL_FRACTION", cf(d["swirl_fraction"]), "edf.residual_swirl_torque_fraction_ASSUMED")
    define("EDF_SWIRL_SIGN", cf(d["swirl_sign"]), f"edf.rotation: {p['edf']['rotation'][:24]}")
    define("FC_HOVER_THROTTLE", cf(d["hover_u"]), f"(m g / T_max)^(1/n), m = {d['m_full']:.4f} kg (bench builds)")
    w("")
    w("/* ---- vehicle mass properties (information / logging) ---- */")
    define("VEH_MASS_KG", cf(d["m_full"]), "CAD parts without launch pad")
    define("VEH_Z_CG_M", cf(d["z_cg"]), "")
    define("VEH_IXX_KG_M2", cf(d["i_full"][0][0]), "")
    define("VEH_IYY_KG_M2", cf(d["i_full"][1][1]), "")
    define("VEH_IZZ_KG_M2", cf(d["i_full"][2][2]), "")
    define("VEH_POST_EJECTION_MASS_KG", cf(d["m_post"]), "")
    w("")
    w("/* ---- onboard supervisor (SPEC 8.8, sim/onboard.py SupervisorParams.from_params); times in control ticks ----")
    w(" * mission.target_altitude_m / apogee_detect_vz_mps / apogee_confirm_s / coast_timeout_s belong to the ground")
    w(" * station's assist mode and are deliberately absent here. */")
    define("ONB_CLIMB_MILLI", f"{sp.climb_milli}U", f"mission.climb_throttle = {ms['climb_throttle']} (x1000)")
    define("ONB_RAMP_TICKS", f"{sp.ramp_ticks}UL", f"mission.throttle_ramp_s = {ms['throttle_ramp_s']:g} s")
    define("ONB_COAST_HOLD_TICKS", f"{sp.coast_hold_ticks}UL", f"mission.coast_control_hold_s = {ms.get('coast_control_hold_s', 0.0):g} s")
    define("ONB_TILT_ABORT_DEG", cf(sp.tilt_abort_deg), "I2 mission.tilt_abort_deg")
    define("ONB_ARM_TILT_MAX_DEG", cf(sp.arm_tilt_max_deg), "ARM gate mission.arm_tilt_max_deg")
    define("ONB_MIN_CELL_V", cf(sp.min_cell_v), "ARM gate, I8 battery.min_cell_v")
    define("ONB_GEOFENCE_M", cf(sp.geofence_m), "I5 mission.geofence_altitude_m")
    define("ONB_APOGEE_LIMIT_M", cf(sp.apogee_limit_m), "I4 mission.apogee_limit_m")
    define("ONB_APOGEE_K", cf(sp.apogee_k), f"I4 coast_drag_factor {sp.coast_drag_factor:g} / (2 g) [s^2/m]")
    define("ONB_FALL_VZ_MPS", cf(sp.fall_vz_mps), "I6 mission.fall_deploy_vz_mps")
    define("ONB_FALL_CONFIRM_TICKS", f"{sp.fall_confirm_ticks}UL", f"I6 mission.fall_deploy_confirm_s = {ms['fall_deploy_confirm_s']:g} s")
    define("ONB_LINK_ABORT_TICKS", f"{sp.link_abort_ticks}UL", f"I1 mission.link_loss_abort_s = {ms['link_loss_abort_s']:g} s")
    define("ONB_LINK_DISARM_TICKS", f"{sp.link_disarm_ticks}UL", f"I1p mission.link_loss_disarm_s = {ms['link_loss_disarm_s']:g} s")
    define("ONB_MAX_MOTOR_TICKS", f"{sp.max_motor_ticks}UL", f"I7 mission.max_motor_time_s = {ms['max_motor_time_s']:g} s")
    define("ONB_RELEASE_AFTER_CUT_TICKS", f"{sp.release_after_cut_ticks}UL", f"mission.release_after_cut_s = {ms['release_after_cut_s']:g} s")
    define("ONB_ABORT_RELEASE_TICKS", f"{sp.abort_release_ticks}UL", f"mission.abort_release_delay_s = {ms['abort_release_delay_s']:g} s")
    define("ONB_TILT_RATE_ABORT_SQ", cf(sp.tilt_rate_abort_rad_s ** 2),
           f"I3 (rad/s)^2: |w_xy| > {ms['tilt_rate_abort_dps']:g} deg/s (mission.tilt_rate_abort_dps)")
    define("ONB_SPIN_RATE_ABORT_RAD_S", cf(sp.spin_rate_abort_rad_s), f"I3 |w_z| > {ms['spin_rate_abort_dps']:g} deg/s")
    define("ONB_RATE_PERSIST_TICKS", f"{sp.rate_persist_ticks}UL", f"I3 mission.rate_abort_persist_s = {ms['rate_abort_persist_s']:g} s")
    define("ONB_GYRO_SATURATION_ABORT", "1" if sp.gyro_saturation_abort else "0", "I3 mission.gyro_saturation_abort")
    define("ONB_IMU_FAIL_TICKS", f"{sp.imu_fail_ticks}UL", f"I9 fc_config imu_fail_kill_s = {d['cfg']['imu_fail_kill_s']:g} s")
    define("ONB_BARO_FAIL_TICKS", f"{sp.baro_fail_ticks}UL", f"I9 fc_config baro_fail_kill_s = {d['cfg']['baro_fail_kill_s']:g} s")
    w("")
    render_vtol(d, define, w)
    w("")
    w("/* ---- LoRa ground link (SPEC 10.3; flight_computer.link, fc_config link) ---- */")
    w("#define FC_RADIO_LORA_UART                 1")
    w("#define FC_RADIO_SX127X                    2")
    w("#define FC_RADIO_ESP_BRIDGE                3   /* 2026-11-06: ESP8266 Wi-Fi bridge (hal/esp_bridge.c) */")
    w("#ifndef FC_LINK_RADIO                      /* a PlatformIO env may force another backend */")
    define("FC_LINK_RADIO", {"lora_uart": "FC_RADIO_LORA_UART", "sx127x": "FC_RADIO_SX127X",
                             "esp_bridge": "FC_RADIO_ESP_BRIDGE"}[d["vradio"]],
           f"flight_computer.link.vehicle_radio = {d['vradio']} (link model {d['radio']}, LoRa fallback model "
           f"link.radio = {d['fallback']})")
    w("#endif")
    es = d["esp"]
    if es is not None:
        define("ESP_UART_BAUD", f"{es['baud']}UL", "flight_computer.link.esp_bridge.uart_baud, 8N1 (esp_bridge: COM USART1)")
        define("ESP_UART_UBRR", str(es["ubrr"]), f"U2X=0, error {es['err'] * 100:+.2f} % (ESP 80 MHz divider "
                                                 f"{es['esp_err'] * 100:+.3f} %; limit {es['max_err'] * 100:g} %)")
        define("ESP_UART_U2X", "0", "as the ilink: simavr's USART ignores U2X")
        define("ESP_RESET_HOLD_MS", f"{es['hold_ms']}U", "link.esp_bridge.reset_hold_ms: PC6 low at boot (ESP RST)")
        define("ESP_BOOT_TICKS", f"{es['boot_ticks']}UL",
               f"link.esp_bridge.boot_discard_ms = {es['boot_ms']:g} ms: RX dropped after the reset release (boot log)")
    define("LINK_VEHICLE_ID", str(d["vid"]), "flight_computer.link.vehicle_id")
    define("LINK_POLL_HZ", cf(float(d["lk"]["poll_hz"])),
           f"flight_computer.link.poll_hz: the ground's request rate (informative; achievable "
           f"{1 / d['budgets'][d['radio']]['poll_interval_s']:.2f} Hz)")
    define("LINK_UART_BAUD", f"{d['link_baud']}UL", "flight_computer.link.uart_baud, 8N1 (lora_uart: COM USART1)")
    define("LINK_UART_UBRR", str(d["ubrr_link"][0]), f"U2X={d['ubrr_link'][1]}, error {d['ubrr_link'][2] * 100:+.2f} %")
    define("LINK_UART_U2X", str(d["ubrr_link"][1]), "")
    define("LINK_AUX_WAIT_TICKS", f"{d['aux_wait_ticks']}U", f"fc_config link.aux_wait_max_s = {d['cfg']['link']['aux_wait_max_s']:g} s")
    define("LINK_NONCE_SEED_CONVERSIONS", f"{d['seed_conv']}U", "fc_config link.nonce_seed_adc_conversions")
    sx = d["sx"]
    define("SX127X_FRF", f"0x{sx['frf']:06X}UL",
           f"link.sx127x.freq_hz {sx['f_hz']:.0f} -> {sx['f_real']:.1f} Hz (FXOSC 32 MHz, step 61.04 Hz); band "
           f"{sx['band'][0] / 1e6:.1f}-{sx['band'][1] / 1e6:.1f} MHz")
    define("SX127X_FRF_MSB", f"0x{(sx['frf'] >> 16) & 0xFF:02X}", "RegFrfMsb 0x06")
    define("SX127X_FRF_MID", f"0x{(sx['frf'] >> 8) & 0xFF:02X}", "RegFrfMid 0x07")
    define("SX127X_FRF_LSB", f"0x{sx['frf'] & 0xFF:02X}", "RegFrfLsb 0x08")
    define("SX127X_LOW_FREQ_MODE", "1" if sx["lf"] else "0", "RegOpMode bit 3 (LF port below 525 MHz)")
    define("SX127X_MODEM_CONFIG1", f"0x{sx['mc1']:02X}", f"BW {sx['bw'] / 1e3:g} kHz, CR 4/{sx['crd']}, explicit header")
    define("SX127X_MODEM_CONFIG2", f"0x{sx['mc2']:02X}", f"SF{sx['sf']}, RxPayloadCrcOn")
    define("SX127X_MODEM_CONFIG3", f"0x{sx['mc3']:02X}", f"AgcAutoOn, LowDataRateOptimize {'on' if sx['ldro'] else 'off'} (T_sym {sx['t_sym'] * 1e3:.2f} ms)")
    define("SX127X_PA_CONFIG", f"0x{sx['pa_config']:02X}", f"{sx['pwr']} dBm on {'PA_BOOST' if sx['pa_boost'] else 'RFO'}")
    define("SX127X_LNA", f"0x{sx['lna']:02X}", "RegLna: max gain" + (", LNA boost HF" if not sx["lf"] else ""))
    define("SX127X_SYNC_WORD", f"0x{sx['sync']:02X}", "link.sx127x.sync_word")
    define("SX127X_PREAMBLE", f"{sx['pre']}U", "link.sx127x.preamble_symbols")
    define("SX127X_RSSI_OFFSET", str(sx["rssi_offset"]), "packet RSSI = offset + RegPktRssiValue (datasheet 5.5.5)")
    define("SX127X_TX_TIMEOUT_TICKS", f"{sx['tx_timeout_ticks']}UL",
           f"{sx['tx_timeout_s'] * 1e3:.0f} ms = factor x ToA("
           f"{ {lp.TELEM3_FRAME_BYTES: 'TELEM3', lp.TELEM2_FRAME_BYTES: 'TELEM2', lp.TELEM7_FRAME_BYTES: 'TELEM7', lp.TELEM6_FRAME_BYTES: 'TELEM6'}.get(sx['telem_bytes'], 'TELEM')}"
           f" {sx['telem_bytes']} B) {sx['toa_tlm'] * 1e3:.1f} ms + margin")
    div, spr, spi2x = d["spi"]
    define("SX127X_SPI_SPR", str(spr), f"SPI F_CPU/{div} = {d['f_cpu'] / div / 1e3:.0f} kHz (<= fc_config sx127x_spi_hz_max)")
    define("SX127X_SPI_2X", str(spi2x), "")
    w("/* link latency budget (sim/link_model.py; *_ASSUMED module constants there), informative:")
    for r, b in d["budgets"].items():
        sel = " (selected)" if r == d["radio"] else ""
        w(f" *   {r}{sel}: uplink {b['uplink_s'] * 1e3:.1f} ms, downlink {b['downlink_s'] * 1e3:.1f} ms, round trip worst "
          f"{b['rtt_worst_s'] * 1e3:.1f} ms, reply timeout {b['reply_timeout_s'] * 1e3:.1f} ms,")
        w(f" *     poll every {b['poll_interval_s'] * 1e3:.0f} ms ({1 / b['poll_interval_s']:.2f} Hz, link.poll_hz "
          f"{b['poll_hz']:g}), uplink gap after one lost frame {b['single_loss_gap_s'] * 1e3:.0f} ms")
    w(" */")
    # 2026-11-10: the slowest backend's (the LoRa fallback model): the benches' / co-simulation's synthetic grounds poll
    # at it with every COM backend (their LoRa COMs must keep up); the selected model's number is in the comment above
    define("LINK_RTT_WORST_MS", f"{round(d['budgets'][d['fallback']]['rtt_worst_s'] * 1e3)}U",
           f"the LoRa fallback model {d['fallback']} (the synthetic grounds' poll period; informative)")
    define("LINK_SINGLE_LOSS_GAP_MS", f"{round(d['budgets'][d['radio']]['single_loss_gap_s'] * 1e3)}U",
           "< link_loss_disarm_s, checked by the generator")
    w("")
    uf, uc = ILINK_USART["fcu"], ILINK_USART["com"]

    def pn(chip: str, key: str) -> str:
        return "P{}{}".format(*d[f"{chip}_pins"][key])

    w("/* ---- COM tick + FCU <-> COM ilink (SPEC 10.4; flight_computer.com, flight_computer.ilink) ----")
    w(f" * wiring: FCU {pn('fcu', 'ilink_tx')} TXD{uf} -> {d['r_series']:g} ohm (ilink.series_resistor_ohm) -> "
      f"COM {pn('com', 'ilink_rx')} RXD{uc};")
    w(f" *         COM {pn('com', 'ilink_tx')} TXD{uc} -> FCU {pn('fcu', 'ilink_rx')} RXD{uf}; common GND (board.h) */")
    define("COM_TICK_HZ", str(d["com_tick_hz"]),
           "flight_computer.com.tick_hz = control.loop_hz (LINK_* / SX127X_* tick counts run on the COM)")
    define("ILINK_BAUD", f"{d['il_baud']}UL",
           f"flight_computer.ilink.baud, 8N1: {d['il_baud'] / 10 / d['loop_hz']:g} bytes per control tick")
    define("ILINK_UBRR", str(d["il_ubrr"]),
           f"U2X=0, error {d['il_err'] * 100:+.2f} % (limit {ILINK_MAX_ERR * 100:g} %)")
    define("ILINK_U2X", "0", "always 0: simavr's USART ignores U2X")
    define("ILINK_STAT_DIV", f"{d['stat_div']}U",
           f"COM ticks per IL_STAT: com.tick_hz {d['com_tick_hz']} / ilink.stat_hz {d['stat_hz']:g}")
    define("ILINK_USART_FCU", str(uf), f"flight_computer.pins.ilink_rx/tx = RXD{uf}/TXD{uf}")
    define("ILINK_USART_COM", str(uc), f"flight_computer.com.pins.ilink_rx/tx = RXD{uc}/TXD{uc}")
    w("#if defined(ILINK_USART) && defined(FC_ROLE_FCU) && ILINK_USART != ILINK_USART_FCU")
    w('#error "platformio.ini [fcu_common] -D ILINK_USART differs from flight_computer.pins.ilink_rx/ilink_tx"')
    w("#endif")
    w("#if defined(ILINK_USART) && defined(FC_ROLE_COM) && ILINK_USART != ILINK_USART_COM")
    w('#error "platformio.ini [com_common] -D ILINK_USART differs from flight_computer.com.pins.ilink_rx/ilink_tx"')
    w("#endif")
    w("")
    w("/* ---- NAV tick + NAV -> FCU nlink SPI (SPEC 10.5; flight_computer.nav, flight_computer.nlink) ----")
    w(f" * wiring: NAV {pn('nav', 'spi_ss')} SS -> FCU {pn('fcu', 'nlink_ss')} SS, NAV {pn('nav', 'spi_sck')} SCK -> FCU "
      f"{pn('fcu', 'nlink_sck')}, NAV {pn('nav', 'spi_mosi')} MOSI -> FCU {pn('fcu', 'nlink_mosi')},")
    w(f" *         FCU {pn('fcu', 'nlink_miso')} MISO -> NAV {pn('nav', 'spi_miso')}; common GND (board.h).  NAV = master, "
      f"FCU = slave, mode 0, MSB first */")
    define("NAV_TICK_HZ", str(d["nav_tick_hz"]), "flight_computer.nav.tick_hz = control.loop_hz (checked)")
    div, spr, spi2x = (d["nl_div"],) + d["nl_spi"]
    define("NLINK_SPI_HZ", f"{round(d['nl_spi_hz'])}UL", f"flight_computer.nlink.spi_clock_hz = F_CPU / {div}")
    define("NLINK_SPI_DIV", str(div), "")
    define("NLINK_SPI_SPR", str(spr), "SPCR SPR1:0 (NAV master)")
    define("NLINK_SPI_2X", str(spi2x), "SPSR SPI2X (NAV master)")
    define("NLINK_SPI_MODE", "0", "flight_computer.nlink.spi_mode (CPOL 0, CPHA 0)")
    define("NLINK_BYTE_GAP_US", cf(d["nl_gap_us"]), "flight_computer.nlink.byte_gap_us")
    define("NLINK_BYTE_GAP_CYCLES", f"{d['nl_gap_cycles']}UL",
           f"CPU cycles; > the FCU slave ISR's {NLINK_SLAVE_REPLY_CYCLES} cycles to its SPDR write (checked)")
    define("NLINK_XFER_US", f"{math.ceil(d['nl_xfer_s'] * 1e6)}U",
           f"SS low .. high of one exchange without interrupts ({nlp.XFER_BYTES} bytes, gaps, master loop; informative)")
    define("NLINK_FRAME_TIMEOUT_US", f"{d['nl_timeout_us']}UL",
           "flight_computer.nlink.frame_timeout_ms (one to two control ticks, checked)")
    define("NLINK_FRAME_TIMEOUT_T1_COUNTS", f"{d['nl_timeout_t1']}U",
           "the same in Timer1 counts (FC_PWM_COUNTS_PER_US; < FC_PWM_TOP + 1, for timer_elapsed_counts)")
    wd = d["nl_wd"]
    w("/* fw09 W1: the NAV master's exchange watchdog (hal/nlink_spi.c nlink_spi_exchange_wait): an exchange that has not")
    w(f" * ended NLINK_WD_COUNTS Timer3 counts (clk / {NLINK_WD_TIMER3_PRESCALER}; Timer3 restarts at 0 on every exchange) "
      "after its start is closed by the main loop.")
    w(f" * Checked: {wd['us']} us > {NLINK_WD_XFER_FACTOR:g} x NLINK_XFER_US {wd['xfer_us']} us and < FC_TICK_US - "
      f"NAV_PUSH_MIN_US_VTOL {wd['push_us']} (src/nav/nav.h) = {wd['limit_us']} us */")
    define("NLINK_WD_US", f"{wd['us']}UL", "flight_computer.nlink.exchange_watchdog_us")
    define("NLINK_WD_COUNTS", f"{wd['counts']}u", f"NLINK_WD_US x F_CPU / 1e6 / {NLINK_WD_TIMER3_PRESCALER} [Timer3 counts]")
    w("")
    w("/* ---- IMU + estimators (sim/sim6dof.py SensorConfig: what the simulation flies with) ---- */")
    s = d["sens"]
    define("IMU_GYRO_FS_SEL", str(d["gfs"][0]), f"+-{s.gyro_range_dps:g} deg/s")
    define("IMU_GYRO_LSB_PER_DPS", cf(d["gfs"][1]), "SensorConfig.gyro_lsb_per_dps")
    define("IMU_GYRO_RAD_S_PER_LSB", cf(math.radians(1.0) / d["gfs"][1]), "")
    define("IMU_ACCEL_AFS_SEL", str(d["afs"][0]), f"+-{s.accel_range_g:g} g")
    define("IMU_ACCEL_MPS2_PER_LSB", cf(d["g0"] / d["afs"][1]), "g0 / SensorConfig.accel_lsb_per_g")
    define("IMU_DLPF_CFG", str(d["dlpf"]), f"gyro delay {s.imu_dlpf_tau_s * 1e3:g} ms (SensorConfig.imu_dlpf_tau_s)")
    define("ALT_CROSSOVER_HZ", cf(s.altitude_crossover_hz), "SensorConfig.altitude_crossover_hz")
    define("ALT_ZETA", cf(s.altitude_zeta), "SensorConfig.altitude_zeta")
    w("")
    w("/* ---- firmware configuration / calibration (fc_config.json) ---- */")
    for i, ax in enumerate("XYZ"):
        define(f"CAL_IMU_BODY_{ax}_INDEX", str(d["imu_idx"][i]), f"imu_mounting.body_{ax.lower()}")
        define(f"CAL_IMU_BODY_{ax}_SIGN", str(d["imu_sgn"][i]), "")
    cfg = d["cfg"]
    define("CAL_MPU6050_ADDR", f"0x{int(cfg['i2c_addr']['mpu6050']):02X}", "7-bit")
    define("CAL_MS5611_ADDR", f"0x{int(cfg['i2c_addr']['ms5611']):02X}", "7-bit")
    define("CAL_BMP280_ADDR", f"0x{int(cfg['i2c_addr'].get('bmp280', 0x77)):02X}", "7-bit (SDO high = 0x77, low = 0x76)")
    gc = d["pad_still"]
    define("CAL_PAD_STILL_TICKS", f"{d['pad_still_ticks']}U", f"{gc['duration_s']} s still before 'aligned'")
    define("CAL_PAD_STILL_RAD_S", cf(math.radians(float(gc["rate_dps"]))), f"{gc['rate_dps']} deg/s")
    define("CAL_PAD_STILL_ACCEL_TOL", cf(float(gc["accel_tol_g"]) * d["g0"]), f"{gc['accel_tol_g']} g")
    define("CAL_VBAT_LPF_HZ", cf(cfg["battery_filter_hz"]), "")
    define("CAL_ARM_DEBOUNCE_TICKS", f"{d['arm_debounce_ticks']}U", f"{cfg['arm_switch_debounce_s']} s")
    define("CAL_GROUND_BARO_TAU_S", cf(cfg["ground_baro_tau_s"]), "pad altitude reference LPF")
    w("")
    w("#endif /* PARAMS_GEN_H */")
    w("")
    return "\n".join(out).split("\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="exit 1 if the header on disk is not up to date")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args(argv)
    try:
        d = derive()
    except (GenError, KeyError) as exc:
        print(f"gen_firmware_params: ERROR {exc}", file=sys.stderr)
        return 2
    text = render(d)
    for wmsg in d["warnings"]:
        print(f"gen_firmware_params: WARNING {wmsg}")
    old = open(a.out, "r", encoding="utf-8").read() if os.path.exists(a.out) else None
    if a.check:
        ok = old == text
        print(f"{rel(a.out)}: {'up to date' if ok else 'STALE'}")
        return 0 if ok else 1
    if old != text:
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    sp, sx = d["sp"], d["sx"]
    print(f"{rel(a.out)}: {'written' if old != text else 'unchanged'} (sha256 {hashlib.sha256(text.encode()).hexdigest()[:16]})")
    for nm, dc in [("rocket", d)] + [(n, d["vcfg"][n]) for n in VCFGS if n in d["vcfg"]]:
        pos = next((pr["pp"] for pr in dc["vt"]["profiles"].values() if pr["pp"] is not None), None) if dc["vt"] else None
        print(f"  FC_VCFG {VCFG_ID[nm]} {nm:<7}: l_v = {dc['l_v'] * 1e3:.3f} mm (z_cg {dc['z_cg'] * 1e3:.3f} mm, m "
              f"{dc['m_full']:.4f} kg), hover u = {dc['hover_u']:.4f}"
              + (f", POS_TW {pos.tw:.4f}" if pos is not None else "")
              + f", tag 0x{dc['params_tag']:02X} ({rel(dc['cad_path'])})")
    for nm, why in d["vcfg_errors"].items():
        print(f"  FC_VCFG {VCFG_ID[nm]} {nm:<7}: UNAVAILABLE - {why}")
    print(f"  A_e = {d['a_e'] * 1e6:.1f} mm^2, S_v = {d['s_v'] * 1e6:.1f} mm^2 (every configuration); nlink watchdog "
          f"{d['nl_wd']['us']} us = {d['nl_wd']['counts']} Timer3 counts (> {NLINK_WD_XFER_FACTOR:g} x {d['nl_wd']['xfer_us']} "
          f"us, < {d['nl_wd']['limit_us']} us)")
    print(f"  Timer2 /{d['t2'][0]} OCR2={d['t2'][1]} -> {d['loop_hz']} Hz; PWM /{d['presc']} TOP={d['top']} -> {d['frame_hz']} Hz; "
          f"TWBR={d['twbr']}; UBRR log={d['ubrr_log'][0]} (U2X {d['ubrr_log'][1]}, {d['ubrr_log'][2] * 100:+.2f} %), "
          f"link={d['ubrr_link'][0]} (U2X {d['ubrr_link'][1]}, {d['ubrr_link'][2] * 100:+.2f} %)")
    print(f"  supervisor ticks: ramp {sp.ramp_ticks}, coast hold {sp.coast_hold_ticks}, I1 {sp.link_abort_ticks}, "
          f"I1p {sp.link_disarm_ticks}, I3 {sp.rate_persist_ticks}, I6 {sp.fall_confirm_ticks}, I7 {sp.max_motor_ticks}, "
          f"I9 {sp.imu_fail_ticks}/{sp.baro_fail_ticks}, release {sp.release_after_cut_ticks}/{sp.abort_release_ticks}")
    print(f"  mission.mode {d['mode']} (MISSION_MODE), TELEM frame {d['telem_bytes']} B")
    if d["vt"]:
        gi = d["vt"]["gains"]
        for nm, pr in d["vt"]["profiles"].items():
            spv, vp, est = pr["spv"], pr["vp"], pr["est"]
            print(f"  VTOL profile {nm}{' (default)' if nm == d['vt']['default'] else f' (-D MISSION_PROFILE_{nm.upper()})'}: "
                  f"spool {spv.spool_ticks}, V1 {spv.link_land_ticks}, V2 {spv.endurance_ticks}, V6 {spv.hard_cutoff_ticks}, "
                  f"V5 {spv.v_fall_confirm_ticks}, V10 {spv.land_timeout_ticks}, baro limit {spv.baro_limit_base_ticks} + "
                  f"{spv.baro_limit_mt_per_dm / 1000:g}/dm, LANDED->SAFE {spv.safe_after_landed_ticks}; target "
                  f"{spv.hover_dm / 10:g} m in [{spv.alt_min_dm / 10:g}, {spv.alt_max_dm / 10:g}] step {spv.alt_step_dm / 10:g}; "
                  f"steps {vp.up_um}/{vp.dn_um}/{vp.td_um} um, flare {vp.flare_um} um")
        vp = d["vt"]["vp"]
        est = d["vt"]["profiles"][d["vt"]["default"]]["est"]
        print(f"  vertical controller: u_hover {vp.u_hover:.4f} (authority {vp.hover_margin:.3f} to u_max {vp.u_max:g}), "
              f"ramp {vp.ramp_ticks}, touchdown {vp.landed_confirm_ticks} ticks; gains {gi['source']}"
              f"{' (STALE params)' if gi['params_stale'] else ''}: " + ", ".join(f"{k} {vp.__dict__[k]:g}" for k in VTOL_GAIN_KEYS)
              + f"; NAV estimator kp {est['kp']:g} ki {est['ki']:g} accel_lpf {est['accel_lpf_hz']:g} Hz ({gi['est_source']})")
    if d["esp"] is not None:
        es = d["esp"]
        print(f"  COM backend {d['vradio']}; ESP bridge USART1 {es['baud']} baud UBRR={es['ubrr']} (U2X 0, "
              f"{es['err'] * 100:+.2f} %), reset hold {es['hold_ms']} ms, boot window {es['boot_ticks']} COM ticks")
    print(f"  radio model {d['radio']} (vehicle id {d['vid']}); sx127x FRF 0x{sx['frf']:06X} = {sx['f_real']:.1f} Hz, "
          f"MC1 0x{sx['mc1']:02X} MC2 0x{sx['mc2']:02X} MC3 0x{sx['mc3']:02X} PA 0x{sx['pa_config']:02X}, "
          f"ToA CMD {sx['toa_cmd'] * 1e3:.1f} ms / TELEM {sx['toa_tlm'] * 1e3:.1f} ms, SPI F_CPU/{d['spi'][0]}")
    print(f"  ilink {d['il_baud']} baud UBRR={d['il_ubrr']} (U2X 0, {d['il_err'] * 100:+.2f} %), IL_STAT {d['stat_hz']:g} Hz "
          f"(ILINK_STAT_DIV = {d['stat_div']} ticks of the {d['com_tick_hz']} Hz COM tick), FCU USART{ILINK_USART['fcu']} "
          f"<-> COM USART{ILINK_USART['com']}, params tag 0x{d['params_tag']:02X}")
    print(f"  nlink SPI {d['nl_spi_hz'] / 1e6:g} MHz (F_CPU/{d['nl_div']}, SPR {d['nl_spi'][0]} SPI2X {d['nl_spi'][1]}), gap "
          f"{d['nl_gap_us']:g} us = {d['nl_gap_cycles']} cycles, exchange {d['nl_xfer_s'] * 1e6:.0f} us of the "
          f"{d['tick_us']} us tick, frame timeout {d['nl_timeout_us']} us = {d['nl_timeout_t1']} Timer1 counts, "
          f"NAV tick {d['nav_tick_hz']} Hz")
    for r, b in d["budgets"].items():
        print(f"  link {r}: up {b['uplink_s'] * 1e3:.1f} ms, down {b['downlink_s'] * 1e3:.1f} ms, rtt worst "
              f"{b['rtt_worst_s'] * 1e3:.1f} ms, poll {1 / b['poll_interval_s']:.2f} Hz (asked {b['poll_hz']:g}), "
              f"single-loss uplink gap {b['single_loss_gap_s'] * 1e3:.0f} ms vs I1p {p_ms(d)} ms")
    return 0


def p_ms(d: dict) -> int:
    return round(float(d["ms"]["link_loss_disarm_s"]) * 1e3)


if __name__ == "__main__":
    sys.exit(main())
