#!/usr/bin/env python3
"""Four-core co-simulation of the v0.9 flight computer with a closed-loop plant (SPEC_firmware_v09 WP-S / S1, section 7):
NAV + GNS + FCU + COM in one libsimavr process (``tools/cosim/cosim4.c``), the plant in the harness
(``tools/cosim/plant4.c``), the NAV's MPU-6050 and barometer (BMP280 or MS5611: the image's ``FC_BARO_SENSOR``) on
its TWI (``tools/cosim/twi_models.c``).

What it does:

1. builds the co-simulation images (PlatformIO; ``PLATFORMIO_RUN_JOBS`` or ``--pio-jobs``, default 2):
   closed loop (52 s): NAV ``nav_cosim_twi`` (the real TWI drivers), GNS ``gns_cosim_cl`` / ``gns_cosim_cl_demo`` (its
   synthetic receiver reports the plant's antenna), FCU ``fcu_cosim_cl`` / ``_demo`` / ``_nogps`` / ``_wired`` /
   ``_wired_buffer``, COM ``com_cosim_cl`` (its synthetic ground flies the field profile: ON, DOWN, UP, OFF);
   open loop (15 s, the v0.8.1 / P1 runs): ``nav_cosim``, ``gns_cosim``, ``fcu_cosim``, ``com_cosim``;
2. reads what the harness needs from the images: the plant's constants from the FCU env's own ``params_gen.h`` (a host
   probe compiled with the env's -D flags: the vehicle the image describes), the PWM compare registers and channels
   (``run_cosim.pwm_registers``), the RAM blocks ``g_cosim_report`` (COM), ``g_cosim_gps`` / ``g_gns_cosim_report``
   (GNS) with avr-nm;
3. builds the harness with zig cc against PlatformIO's libsimavr.a (``tools/cosim/build/cosim4.exe``);
4. runs the scenarios of SPEC section 4 S1d, each its own run, at most ``--jobs`` (default 2) at a time;
5. analyses every event file (records: cosim4.c header) and judges the co-simulation rows of SPEC section 7
   (3, 4, 5, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19), the closed loop's validation against the bench plant and the
   job lengths the FCU's background reserves need; writes ``host_test/build/cosim4_report.json``.

Definitions (cycles are the co-simulation's global 16 MHz clock):
  NAV tick              its Timer2 compare (NT t2);  IMU sample = its Timer0 compare before it (NT t0: the IMU lead,
                        present since the harness honours TCNT0's preload; t2 - t0 is measured per tick)
  NAV frame delivered   SS high of the exchange that starts in that tick (P1's frame match by SS low)
  NAV past the IMU lead the NAV's marker down (end of its time-critical part) after the next tick's Timer0 compare
  FCU time-critical     its tick marker up -> down (T rise -> fall), interrupts included (they ran in the window)
  NAV tick -> vane      NAV tick -> the FCU's last vane compare-register write (+2 cycles: the low byte commits) of the
                        FCU tick that runs on that frame;  IMU sample -> vane: from the Timer0 compare instead
  flight / HOLD         the FCU's state / phase from its nlink reply (FCU4 state, pflags phase) in every exchange
  calm flight           POWERED, outside [fault_on - 0.1 s, fault_off + 1 s]

usage: run_cosim4.py [--only name,...] [--jobs 2] [--no-build] [--analyse-only] [--extra-tree NAME=FWDIR ...]
       [--report PATH]
"""
from __future__ import annotations

import argparse
import bisect
import concurrent.futures
import configparser
import hashlib
import json
import math
import os
import re
import statistics
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, field

TOOLS = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(TOOLS, ".."))
ROOT = os.path.normpath(os.path.join(FW, "..", ".."))
COSIM = os.path.join(TOOLS, "cosim")
HB = os.path.join(COSIM, "build")
RUNS = os.path.join(HB, "runs4")
REPORT = os.path.join(FW, "host_test", "build", "cosim4_report.json")
sys.path.insert(0, TOOLS)
import run_cosim as rc  # noqa: E402  (avr tools, simavr dir, PWM registers, avr-nm)

CYC = 16.0                                   # cycles per us
TICK_CYC = 64000                             # 4 ms
CL_SECONDS = 52.0
OL_SECONDS = 15.0
FAULT_AT_CL_MS = 20000.0                     # inside HOLD of the field profile (HOLD ~17..30 s)
FAULT_AT_OL_MS = 10000.0                     # inside POWERED of the open-loop runs (as fw09/spec cosim_suite.py)
WIND = (0.5, 0.3)                            # m/s^2, the plant's horizontal wind at zero ground speed (ASSUMED): HOLD flies a tilt
DRAG_K = 0.3                                 # 1/s, horizontal linear drag (ASSUMED): air speed of that wind 1.7 / 1.0 m/s
LP_R = {1: "I1_LINK_LOSS", 2: "I1P_LINK_LOSS_PAD", 3: "I2_TILT", 4: "I3_RATE", 7: "I6_FALL", 8: "I7_MOTOR_TIME",
        9: "I8_VCELL", 10: "I9_IMU", 11: "I9_BARO", 12: "I10_RESET", 14: "V_LINK_LAND", 15: "V_ENDURANCE",
        16: "V_CEILING", 17: "V_GEOFENCE", 18: "V_FALL", 19: "V_HARD_CUTOFF", 20: "V_TAKEOFF_GATE",
        21: "V_BARO_LIMIT_LANDED", 22: "V_BARO_LIMIT_ABORT", 23: "V_LAND_TIMEOUT", 24: "V_GPS_LOST", 25: "V_FENCE",
        26: "V_GPS_IMPLAUSIBLE", 27: "V_AUTHORITY", 28: "V_TAKEOFF_CHECK", 0x30: "W_LOST", 0x31: "W_SAG", 0x32: "W_BUF",
        0x33: "W_D2"}
BARO_LOST_REASONS = {11, 21, 22}
STATES = {0: "SAFE", 1: "ARMED", 2: "POWERED", 3: "UNPOWERED", 4: "ABORT", 5: "RELEASED"}
PHASES = {0: "NONE", 1: "SPOOL", 2: "CLIMB", 3: "HOLD", 4: "LAND", 5: "LANDED"}
NL_F_BARO_OK, NL_F_BARO_NEW = 0x02, 0x20

# ---------------------------------------------------------------------------------------------------------------- envs
CL_ENVS = {"nav": "nav_cosim_twi", "gns": "gns_cosim_cl", "gns_demo": "gns_cosim_cl_demo", "com": "com_cosim_cl",
           "fcu": "fcu_cosim_cl", "fcu_demo": "fcu_cosim_cl_demo", "fcu_nogps": "fcu_cosim_cl_nogps",
           "fcu_wired": "fcu_cosim_cl_wired", "fcu_wired_buffer": "fcu_cosim_cl_wired_buffer"}
OL_ENVS = {"nav": "nav_cosim", "gns": "gns_cosim", "com": "com_cosim", "fcu": "fcu_cosim"}
# fw09 integration (G1): FC_PARAMS_TAG carries the vehicle configuration, so the four images of one closed-loop run come
# from ONE configuration - the battery builds (fcu, fcu_demo, fcu_nogps: FC_VCFG=0) fly the battery NAV / GNS / COM, the
# tethered (fcu_wired: 1) and hybrid (fcu_wired_buffer: 2) FCUs their own *_wired / *_wired_buffer NAV / GNS / COM twins
# (a mixed set is flagged NAV_RC_TAG_MISMATCH by the NAV and LOG_COM_TAG_MISMATCH by the FCU, and the NAV's altitude
# filter would carry another vehicle's 1 / u_hover)
CL_SFX = {"fcu": "", "fcu_demo": "", "fcu_nogps": "", "fcu_wired": "_wired", "fcu_wired_buffer": "_wired_buffer"}


def cl_set(build: str) -> dict:
    """the four closed-loop envs of an FCU build (one vehicle configuration)"""
    sfx = CL_SFX[build]
    return {"nav": CL_ENVS["nav"] + sfx, "com": CL_ENVS["com"] + sfx, "fcu": CL_ENVS[build],
            "gns": CL_ENVS["gns_demo"] if build == "fcu_demo" else CL_ENVS["gns"] + sfx}


def cl_all_envs() -> list[str]:
    return sorted({e for b in CL_SFX for e in cl_set(b).values()})


@dataclass
class Scn:
    name: str
    build: str = "fcu"                       # fcu / fcu_demo / fcu_nogps / fcu_wired / fcu_wired_buffer (closed loop)
    fault: str = "none"
    at_ms: float = FAULT_AT_CL_MS
    len_ms: float = 0.0
    extra: dict = field(default_factory=dict)
    loop: str = "closed"                     # closed / open
    tree: str = "base"                       # base = this tree; else an --extra-tree name
    kind: str = "normal"                     # normal / transient / fault / compare / g1
    expect: str = ""                         # the designed outcome (row 18)
    mixed: bool = False                      # fw09 review F1: the battery NAV / GNS / COM with this build's FCU (G1 test)

    @property
    def path(self) -> str:
        return os.path.join(RUNS, f"{self.name}.txt")


def scenarios(extra_trees: dict) -> list[Scn]:
    s = [Scn("cl_normal_fcu", expect="field profile: ON, CLIMB, HOLD >= 10 s tilted, DOWN / UP, OFF -> LAND, LANDED, SAFE"),
         Scn("cl_normal_fcu_nogps", "fcu_nogps", expect="field profile, attitude-only (no position hold)"),
         Scn("cl_normal_fcu_wired", "fcu_wired", expect="field profile on the tether limits (alt_max 2 m)"),
         Scn("cl_normal_fcu_wired_buffer", "fcu_wired_buffer", expect="field profile on the tether limits (alt_max 2 m)"),
         Scn("cl_demo", "fcu_demo", expect="demo profile (1.5 m hover)"),
         Scn("cl_gps_loss_3s", fault="gps_cut", len_ms=3000.0, kind="transient",
             expect="GPS source cut 3 s in HOLD: position hold degrades / recovers per gps_loss rules, no NAV effect"),
         Scn("cl_gps_loss_20s", fault="gps_cut", at_ms=19000.0, len_ms=20000.0, kind="transient",
             expect="GPS source cut 20 s in HOLD: V11 GPS_LOST -> attitude-only LAND, no NAV effect"),
         Scn("cl_gns_hold", fault="gns_hold", kind="fault",
             expect="GNS held in reset: V11 attitude-only hold / LAND, no NAV effect"),
         Scn("cl_gns_pulse", fault="gns_pulse", len_ms=1000.0, kind="fault",
             expect="GNS reset 1 s: TRIM fallback / V11 per the slot rules, no NAV effect"),
         Scn("cl_nav_pulse", fault="nav_pulse", len_ms=1.0 + 65.0 + 16384 / 16000.0, extra={"nav_reset_byte": 30},
             kind="fault", expect="NAV reset in flight: I9 ABORT (reason 10)"),
         Scn("cl_nl_flip", fault="nl_flip", len_ms=1000.0, extra={"nl_every": 8}, kind="fault",
             expect="bit flips: frames refused, no abort"),
         Scn("cl_nl_drop", fault="nl_drop", len_ms=1000.0, extra={"nl_every": 8}, kind="fault",
             expect="dropped bytes: timeouts / resyncs, no abort"),
         Scn("cl_nl_extra", fault="nl_extra", len_ms=1000.0, extra={"nl_every": 8}, kind="fault",
             expect="phantom bytes: frames refused / resyncs, no abort"),
         Scn("cl_nl_noise", fault="nl_noise", len_ms=1000.0, extra={"nl_ber": "2e-4", "seed": 20261015}, kind="fault",
             expect="bit errors 2e-4: frames refused, no abort"),
         Scn("cl_cut", fault="cut", len_ms=4000.0, kind="fault", expect="ilink cut 4 s: link LAND (V_LINK_LAND), no abort"),
         Scn("cl_reset", fault="reset", len_ms=500.0, kind="fault", expect="COM reset 0.5 s: no abort, no LAND"),
         Scn("cl_nl_lose_cmp", fault="nl_lose_cmp", extra={"nl_lose_byte": 30}, kind="fault",
             expect="W1: the lost compare closed by the NAV's exchange watchdog: 1 missed frame, no abort"),
         Scn("cl_babble", fault="babble", len_ms=1000.0, kind="fault",
             expect="COM babble 1 s: flood detected, the FCU keeps its tick, no abort"),
         Scn("cl_mixed_images", "fcu_wired", mixed=True, kind="g1",
             expect="G1 (SPEC 3.2, fw09 review F1): a MIXED image set - the tether FCU with the battery NAV / GNS / COM - "
                    "is refused on the ground: the NAV flags the tag, every ON is answered and refused, the FCU stays SAFE"),
         Scn("ol_normal", loop="open", kind="compare", expect="the P1 open-loop run (15 s) on this tree's images")]
    for t in extra_trees:
        s.append(Scn(f"ol_normal_{t}", loop="open", tree=t, kind="compare", expect=f"open-loop normal, tree {t}"))
        s.append(Scn(f"ol_nl_lose_cmp_{t}", loop="open", tree=t, fault="nl_lose_cmp", at_ms=FAULT_AT_OL_MS,
                     extra={"nl_lose_byte": 30}, kind="fault", expect=f"lost compare, tree {t}"))
    return s


# ---------------------------------------------------------------------------------------------------------------- tools
def sh(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", **kw)


def ini_envs(fw: str) -> configparser.RawConfigParser:
    cp = configparser.RawConfigParser(strict=False)
    cp.read(os.path.join(fw, "platformio.ini"), encoding="utf-8")
    return cp


def env_defines(fw: str, env: str) -> list[str]:
    """the -D flags of a PlatformIO env (its build_flags with the ${sec.key} references resolved)"""
    cp = ini_envs(fw)

    def get(sec: str, key: str, depth: int = 0) -> str:
        if depth > 8 or not cp.has_option(sec, key):
            return ""
        v = cp.get(sec, key)
        return re.sub(r"\$\{([\w:]+)\.(\w+)\}", lambda m: get(m.group(1), m.group(2), depth + 1), v)
    flags = get(f"env:{env}", "build_flags")
    out = []
    for tok in re.findall(r"-D\s*([A-Za-z_]\w*(?:=\S+)?)", flags):
        out.append("-D" + tok)
    return out


PROBE_NAMES = ("VC_U_HOVER", "MIX_THROTTLE_EXP", "MIX_T_MAX_N", "MIX_L_V_M", "MIX_R_V_M", "MIX_S_V_M2", "MIX_CL_ALPHA",
               "MIX_A_E_M2", "VEH_IXX_KG_M2", "VEH_IYY_KG_M2", "VEH_IZZ_KG_M2", "VEH_MASS_KG", "EDF_TAU_S", "FC_G0",
               "CAL_ESC_MIN_US", "CAL_ESC_MAX_US", "MIX_PWM_CENTER_US", "MIX_PWM_US_PER_DEG", "CAL_SERVO_SIGN_1",
               "CAL_SERVO_SIGN_2", "CAL_SERVO_SIGN_3", "CAL_SERVO_SIGN_4", "CAL_SERVO_TRIM_US_1", "CAL_SERVO_TRIM_US_2",
               "CAL_SERVO_TRIM_US_3", "CAL_SERVO_TRIM_US_4", "GPS_LEVER_B_X_M", "GPS_LEVER_B_Y_M", "GPS_LEVER_B_Z_M",
               "GPS_PSI0_RAD", "FC_TICK_US", "NLINK_XFER_US", "FC_IMU_LEAD_COUNTS", "FC_T2_PRESCALER",
               "GLINK_LATENCY_TICKS", "FC_LOOP_HZ", "ONB_V_HOVER_DM", "ONB_V_ALT_MAX_DM", "ONB_W_ALT_MAX_DM",
               "ONB_V_ALT_STEP_DM", "VC_CLIMB_RATE_MPS", "VC_DESCENT_RATE_MPS", "FC_PARAMS_TAG",
               "FC_BARO_SENSOR", "FC_BARO_BMP280")


def header_probe(fw: str, env: str) -> dict:
    """the constants of src/params_gen.h as the env compiles them (its -D flags select the profile / configuration)"""
    os.makedirs(HB, exist_ok=True)
    src = os.path.join(HB, f"probe_{env}.c")
    lines = ["/* generated by tools/run_cosim4.py: params_gen.h as env " + env + " compiles it */", "#include <stdio.h>",
             '#include "params_gen.h"', "int main(void)", "{"]
    for n in PROBE_NAMES:
        lines += [f"#ifdef {n}", f'    printf("{n}=%.10g\\n", (double)({n}));', "#endif"]
    lines += ["    return 0;", "}"]
    with open(src, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    exe = os.path.join(HB, f"probe_{env}" + (".exe" if os.name == "nt" else ""))
    defs = [d for d in env_defines(fw, env)]
    cmd = [sys.executable, "-m", "ziglang", "cc"] + (["-target", "x86_64-windows-gnu"] if os.name == "nt" else [])
    r = sh(cmd + ["-std=c99", "-O0", "-I", os.path.join(fw, "src"), *defs, "-o", exe, src])
    if r.returncode != 0:
        raise SystemExit(f"header probe {env} failed:\n{r.stdout}{r.stderr}")
    o = sh([exe])
    out = {}
    for ln in o.stdout.splitlines():
        k, _, v = ln.partition("=")
        out[k] = float(v)
    return out


def pio_build(fw: str, envs: list[str], jobs: int) -> dict:
    r = sh([sys.executable, "-m", "platformio", "run", "-j", str(jobs), "-d", fw, *sum((["-e", e] for e in envs), [])])
    if r.returncode != 0:
        raise SystemExit(f"PlatformIO build failed:\n{r.stdout[-4000:]}\n{r.stderr[-2000:]}")
    out = {}
    for e in envs:
        hx = os.path.join(fw, ".pio", "build", e, "firmware.hex")
        with open(hx, "rb") as f:
            out[e] = {"hex": hx, "elf": hx[:-3] + "elf", "sha256": hashlib.sha256(f.read()).hexdigest()}
    blocks = re.split(r"^Processing (\S+)", r.stdout, flags=re.M)
    for i in range(1, len(blocks) - 1, 2):
        env, text = blocks[i], blocks[i + 1]
        ram = re.search(r"RAM:.*used (\d+) bytes", text)
        fl = re.search(r"Flash:.*used (\d+) bytes", text)
        if env in out:
            out[env]["static_ram_bytes"] = int(ram.group(1)) if ram else None
            out[env]["flash_bytes"] = int(fl.group(1)) if fl else None
    return out


def image_info(fw: str, envs: list[str]) -> dict:
    out = {}
    for e in envs:
        hx = os.path.join(fw, ".pio", "build", e, "firmware.hex")
        if not os.path.exists(hx):
            raise SystemExit(f"missing {hx} (build it, or drop --no-build)")
        with open(hx, "rb") as f:
            out[e] = {"hex": hx, "elf": hx[:-3] + "elf", "sha256": hashlib.sha256(f.read()).hexdigest()}
    return out


def sym(elf: str, name: str) -> tuple[int, int]:
    syms = rc.nm_symbols(elf)
    if name not in syms:
        raise SystemExit(f"{elf}: symbol {name} not found")
    a, n = syms[name]
    return a - 0x800000, n


def build_harness() -> tuple[str, list[str]]:
    os.makedirs(HB, exist_ok=True)
    exe = os.path.join(HB, "cosim4" + (".exe" if os.name == "nt" else ""))
    sim = rc.simavr_dir()
    cmd = [sys.executable, "-m", "ziglang", "cc"] + (["-target", "x86_64-windows-gnu"] if os.name == "nt" else [])
    cmd += ["-std=c99", "-O2", "-Wall", "-Wextra", "-I", os.path.join(sim, "include", "simavr"), "-I",
            os.path.join(FW, "src", "com"), "-I", COSIM, "-o", exe, os.path.join(COSIM, "cosim4.c"),
            os.path.join(COSIM, "plant4.c"), os.path.join(COSIM, "twi_models.c"), os.path.join(COSIM, "elf_stubs.c"),
            os.path.join(sim, "lib", "libsimavr.a")]
    cmd += ["-lws2_32"] if os.name == "nt" else ["-lm"]
    r = sh(cmd)
    if r.returncode != 0:
        raise SystemExit(f"cosim4 build failed:\n{r.stdout}{r.stderr}")
    return exe, sorted(set(re.findall(r"warning: [^\n]*", r.stdout + r.stderr)))


def source_defines(path: str, names: tuple[str, ...]) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        s = f.read()
    out = {}
    for n in names:
        m = re.search(rf"#define\s+{n}\s+\(?\s*(\d+)\s*u?\s*\)?", s)
        if m:
            out[n] = int(m.group(1))
    m = re.search(r"#define\s+COM_TEXT_RESERVE_US\s+\(BG_RESERVE_US\s*\+\s*(\d+)u\)", s)
    if m and "BG_RESERVE_US" in out:
        out["COM_TEXT_RESERVE_US"] = out["BG_RESERVE_US"] + int(m.group(1))
    return out


# ---------------------------------------------------------------------------------------------------------------- runs
def plant_args(h: dict) -> list[str]:
    us_per_rad = h["MIX_PWM_US_PER_DEG"] * 180.0 / math.pi
    c0 = [h["MIX_PWM_CENTER_US"] + h.get(f"CAL_SERVO_TRIM_US_{i}", 0.0) for i in range(1, 5)]
    k = [h.get(f"CAL_SERVO_SIGN_{i}", 1.0) * us_per_rad for i in range(1, 5)]
    return [f"pl_u_hover={h['VC_U_HOVER']!r}", f"pl_n_exp={h['MIX_THROTTLE_EXP']!r}", f"pl_t_max={h['MIX_T_MAX_N']!r}",
            f"pl_l_v={h['MIX_L_V_M']!r}", f"pl_r_v={h['MIX_R_V_M']!r}", f"pl_s_v={h['MIX_S_V_M2']!r}",
            f"pl_cl_a={h['MIX_CL_ALPHA']!r}", f"pl_a_e={h['MIX_A_E_M2']!r}", f"pl_ixx={h['VEH_IXX_KG_M2']!r}",
            f"pl_iyy={h['VEH_IYY_KG_M2']!r}", f"pl_izz={h['VEH_IZZ_KG_M2']!r}", f"pl_tau={h['EDF_TAU_S']!r}",
            f"pl_g0={h['FC_G0']!r}", f"pl_esc_min_us={h['CAL_ESC_MIN_US']!r}", f"pl_esc_max_us={h['CAL_ESC_MAX_US']!r}",
            "pl_c0_us=" + ",".join(repr(x) for x in c0), "pl_k_us_per_rad=" + ",".join(repr(x) for x in k),
            "pl_lever_b=" + ",".join(repr(h.get(f"GPS_LEVER_B_{a}_M", 0.0)) for a in "XYZ"),
            f"pl_psi0={h.get('GPS_PSI0_RAD', 0.0)!r}", f"pl_wind_ax={WIND[0]!r}", f"pl_wind_ay={WIND[1]!r}",
            f"pl_drag_k={DRAG_K!r}", f"pl_baro_comp_m={nav_comp_m()!r}", "pl_log_ms=4"]


def nav_comp_m() -> float:
    """nav_simbench.h NAV_SB_COMP_M: the simbench builds' baro thrust compensation (m at hover thrust; a timing input
    there).  nav_cosim_twi compiles it too, so the plant's baro carries the error it removes (else the NAV's altitude is
    off by it in flight - measured 0.48 m rms before this was added)"""
    with open(os.path.join(FW, "src", "nav", "nav_simbench.h"), "r", encoding="utf-8") as f:
        m = re.search(r"#define\s+NAV_SB_COMP_M\s+([0-9.]+)f?", f.read())
    return float(m.group(1)) if m else 0.0


def run_one(exe: str, s: Scn, ctx: dict) -> tuple[Scn, float, int, str]:
    tree = ctx["trees"][s.tree]
    pwm = ",".join(str(a) for a in tree["pwm"])
    if s.loop == "closed":
        im = tree["cl_images"]
        cs = cl_set(s.build)
        if s.mixed:                                  # fw09 review F1: this FCU with the battery configuration's other chips
            cs = {**cl_set("fcu"), "fcu": cs["fcu"]}
        fcu_env, gns_env = cs["fcu"], cs["gns"]
        args = [exe, f"fcu={im[fcu_env]['hex']}", f"com={im[cs['com']]['hex']}", f"nav={im[cs['nav']]['hex']}",
                f"gns={im[gns_env]['hex']}", f"out={s.path}", f"seconds={CL_SECONDS}", f"pwm={pwm}",
                "pwm_map=" + ",".join(str(c) for c in tree["pwm_map"]),
                f"report={tree['cl_syms'][cs['com']]['com_report'][0]}",
                "gps={}:{}".format(*tree["cl_syms"][gns_env]["gps"]),
                "gnsrep={}:{}".format(*tree["cl_syms"][gns_env]["gnsrep"]), "twi=1", "twifix=1", "t0fix=1", "plant=1",
                f"baro={tree['cl_baro'][cs['nav']]}",
                *plant_args(tree["header"][fcu_env])]
    else:
        im = tree["ol_images"]
        args = [exe, f"fcu={im[OL_ENVS['fcu']]['hex']}", f"com={im[OL_ENVS['com']]['hex']}",
                f"nav={im[OL_ENVS['nav']]['hex']}", f"gns={im[OL_ENVS['gns']]['hex']}", f"out={s.path}",
                f"seconds={OL_SECONDS}", f"pwm={pwm}", f"report={tree['ol_syms']['com_report'][0]}", "t0fix=1"]
    if s.fault != "none":
        args += [f"fault={s.fault}", f"fault_at_ms={s.at_ms}", f"fault_len_ms={s.len_ms}"]
    args += [f"{k}={v}" for k, v in s.extra.items()]
    os.makedirs(RUNS, exist_ok=True)
    t0 = time.time()
    r = sh(args, timeout=3600)
    with open(s.path[:-4] + ".cmd", "w", encoding="utf-8") as f:
        f.write(" ".join(args) + "\n" + r.stdout[-3000:] + r.stderr[-3000:] + f"\nexit {r.returncode}\n")
    return s, time.time() - t0, r.returncode, (r.stdout + r.stderr)[-2000:]


# ---------------------------------------------------------------------------------------------------------------- parse
def kvhex(tokens: list[str]) -> dict:
    out = {}
    for t in tokens:
        k, _, v = t.partition("=")
        if not _:
            continue
        try:
            out[k] = int(v, 16)
        except ValueError:
            pass
    return out


def kvnum(tokens: list[str]) -> dict:
    out = {}
    for t in tokens:
        k, _, v = t.partition("=")
        if not _:
            continue
        try:
            out[k] = int(v, 0)
        except ValueError:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def parse(path: str) -> dict:
    ev = {"NT": [], "T": [], "X": [], "XR": [], "E": [], "P": [], "C": {}, "NC": {}, "UL": [], "GS": [], "GR": [], "GQ": [],
          "CR": [], "MX": {}, "M": {}, "MG": {}, "H": ""}
    with open(path, encoding="latin-1") as f:
        for ln in f:
            a = ln.split()
            if not a:
                continue
            tag = a[0]
            if tag == "NT":
                ev["NT"].append((int(a[1]), int(a[2]), int(a[3]), int(a[4]), int(a[5]), int(a[6])))
            elif tag == "T":
                m = re.search(r"pwm=([0-9,]+)", ln)
                ev["T"].append((int(a[1]), int(a[2]), int(a[3]), [int(x) for x in m.group(1).split(",")] if m else None))
            elif tag == "X" and len(a) >= 17:
                ev["X"].append((int(a[1]), int(a[2]), int(a[3]), int(a[4]), int(a[14]), a[15], a[16]))
                # fw09 review SYS-2: (SS low, faults, the FCU received what the NAV sent: frx '=')
                ev["XR"].append((int(a[3]), int(a[14]), a[17] == "=" if len(a) > 17 else None))
            elif tag == "P" and len(a) >= 22:
                ev["P"].append(tuple(float(x) for x in a[1:22]))
            elif tag == "C" and len(a) > 3:
                kind = a[2]
                d = kvhex(a[3:])
                if "t" in d:
                    ev["C"].setdefault(kind, []).append((int(a[1]), d))
            elif tag == "NC" and len(a) > 3:
                d = kvhex(a[3:])
                ev["NC"].setdefault(a[2], []).append((int(a[1]), d))
            elif tag == "E":
                ev["E"].append((int(a[1]), a[2], " ".join(a[3:])))
            elif tag == "UL":
                ev["UL"].append(tuple(int(x) for x in a[1:12]))
            elif tag == "GS":
                ev["GS"].append((int(a[1]), int(a[2])))
            elif tag == "GR":
                ev["GR"].append((int(a[1]), int(a[2])))
            elif tag == "GQ":
                ev["GQ"].append((int(a[1]), a[2]))
            elif tag == "CR":
                ev["CR"].append(kvnum(a[2:]))
            elif tag in ("MX", "M", "MG"):
                ev[tag] = kvnum(a[1:])
            elif tag == "H":
                ev["H"] = ln.strip()
    return ev


def merge_lines(lines: list) -> list[tuple[int, dict]]:
    """the FCU / NAV report lines of one kind: parts with the same t= merged, in time order"""
    by_t: dict = {}
    first: dict = {}
    for t, d in lines:
        k = d["t"]
        by_t.setdefault(k, {}).update(d)
        first.setdefault(k, t)
    return sorted(((first[k], v) for k, v in by_t.items()), key=lambda x: x[0])


def stat(v: list, nd: int = 1) -> dict | None:
    v = [x for x in v if x is not None]
    if not v:
        return None
    s = sorted(v)
    return {"n": len(v), "min": round(s[0], nd), "mean": round(statistics.mean(v), nd),
            "p99": round(s[int(0.99 * (len(s) - 1))], nd), "max": round(s[-1], nd)}


# ---------------------------------------------------------------------------------------------------------------- analysis
def frames(stream: list, lens: dict) -> list:
    """glink frames A5 5A type seq payload crc: (type, seq, payload, t_first, t_last)"""
    out = []
    i, n = 0, len(stream)
    while i + 4 <= n:
        if stream[i][1] == 0xA5 and stream[i + 1][1] == 0x5A and stream[i + 2][1] in lens:
            typ = stream[i + 2][1]
            tot = 4 + lens[typ] + 2
            if i + tot <= n:
                out.append((typ, stream[i + 3][1], bytes(b for _, b in stream[i + 4:i + 4 + lens[typ]]), stream[i][0],
                            stream[i + tot - 1][0]))
                i += tot
                continue
        i += 1
    return out


def glink(ev: dict, L: int, calm=lambda t: True) -> dict:
    """STATE (NAV tick 5 m + 4) -> RESULT -> the TILT part in the nlink frame of NAV tick k + L (fw09/spec an_cosim4.py)"""
    BYTE = 640
    NT = {}
    for ep, tick, rise, fall, t2, t0 in ev["NT"]:
        NT[(ep, tick)] = (rise, fall, t2, t0)
    states = [f for f in frames(ev["GS"], {1: 54, 2: 20}) if f[0] == 1]
    results = [f for f in frames(ev["GR"], {1: 54, 2: 20}) if f[0] == 2]
    res_by_seq: dict = {}
    for r in results:
        res_by_seq.setdefault(r[1], []).append(r)
    margins, cycles = [], []
    for st in states:
        k = int.from_bytes(st[2][0:4], "little")
        cand = [r for r in res_by_seq.get(st[1], []) if r[3] > st[4]]
        if not cand:
            continue
        r = cand[0]
        t_state_end, t_res_end = st[4] + BYTE, r[4] + BYTE
        cycles.append((r[3] - t_state_end) / CYC)
        # the tick k + L of the epoch the STATE belongs to: the NT whose t2 is the latest before the STATE's write
        ep = next((e for (e, kk) in NT if kk == k and NT[(e, kk)][2] <= st[3] < NT[(e, kk)][2] + 2 * TICK_CYC), None)
        if ep is not None and (ep, k + L) in NT and calm(st[3]):
            margins.append((NT[(ep, k + L)][0] - t_res_end) / CYC)
    ok = bad = bad_fault = 0
    nts = sorted(((v[2], e, k) for (e, k), v in NT.items()))
    t2s = [x[0] for x in nts]
    for x in ev["X"]:
        mo = bytes.fromhex(x[5]) if x[5] != "-" else b""
        if len(mo) < 72 or mo[0] != 0xD5 or mo[1] != 0x5D or mo[2] != 0x04 or x[4]:
            continue
        slot = mo[65:70]
        if (slot[0] & 7) != 0:
            continue
        i = bisect.bisect_right(t2s, x[2]) - 1
        if i < 0 or x[2] - t2s[i] >= TICK_CYC or (nts[i][2] & 0xFF) != mo[3]:
            continue
        kf = nts[i][2]
        ks = kf - L
        if (ks - 4) % 5 == 0 and ((ks - 4) // 5) & 0x1F == slot[0] >> 3:
            ok += 1
        elif calm(x[3]):
            bad += 1
        else:
            bad_fault += 1
    return {"L": L, "states": len(states), "results": len(results), "matched": len(cycles),
            "gns_cycle_us": stat(cycles), "margin_to_tick_kL_us": stat(margins), "tilt_parts_at_k_plus_L": ok,
            "tilt_parts_wrong": bad, "tilt_parts_wrong_in_fault_window": bad_fault,
            "_doc": "margins and wrong TILT parts outside the fault window (calm); a GNS fault breaks the slot by design"}


def fault_window(ev: dict, s: Scn) -> tuple[float, float] | None:
    on = next((t for t, n, _ in ev["E"] if n == "fault_on"), None)
    off = next((t for t, n, _ in ev["E"] if n == "fault_off"), None)
    if on is None:
        return None
    end = (off if off is not None else 1e18) + 1.0 * 16e6
    return (on - 0.1 * 16e6, end)


def analyse(s: Scn, ctx: dict) -> dict:
    tree = ctx["trees"][s.tree]
    h = tree["header"][CL_ENVS[s.build] if s.loop == "closed" else "fcu_cosim"]
    consts = tree["main_c"]
    ev = parse(s.path)
    L = int(h.get("GLINK_LATENCY_TICKS", 3))
    lead_an = int(h["FC_IMU_LEAD_COUNTS"] * h["FC_T2_PRESCALER"])
    _, outs = tree["pwm_outs"]
    vane = [outs[f"VANE{i}"] for i in range(1, 5)]
    out: dict = {"scenario": s.name, "build": s.build, "loop": s.loop, "tree": s.tree, "fault": s.fault,
                 "fault_at_ms": s.at_ms if s.fault != "none" else None, "fault_len_ms": s.len_ms,
                 "expect": s.expect, "events": [e for e in ev["E"] if e[1] not in ("com_start", "nav_start", "gns_start")][:40]}
    crash = [e for e in ev["E"] if e[1] == "crash"]
    end = next((e for e in ev["E"] if e[1] == "end"), None)
    out["completed"] = end is not None and not crash
    out["crash"] = crash
    # ---- the FCU's state / phase from its nlink reply (FCU4), one per exchange
    tl = []                                                      # (t, state, phase, pflags)
    navf = []                                                    # (t, flags, h, vz, tilt_deg, q)
    nav_tag_mismatch = 0                                         # fw09 review F1: frames with reset_cause NAV_RC_TAG_MISMATCH
    for n, ep, lo, hi, faults, mosi, miso in ev["X"]:
        if faults:
            continue
        mi = bytes.fromhex(miso) if miso != "-" else b""
        if len(mi) >= 13 and mi[0] == 0xD5 and mi[1] == 0x5D and mi[2] == 0x05 and mi[4] <= 5:
            tl.append((hi, mi[4], mi[8] & 7, mi[8]))
        mo = bytes.fromhex(mosi) if mosi != "-" else b""
        if len(mo) >= 72 and mo[0] == 0xD5 and mo[1] == 0x5D and mo[2] == 0x04:
            q = struct.unpack_from("<4f", mo, 4)
            navf.append((hi, mo[63], struct.unpack_from("<f", mo, 36)[0], struct.unpack_from("<f", mo, 40)[0],
                         struct.unpack_from("<f", mo, 48)[0], q))
            if mo[64] & 0x20:                                    # nlink v4: flags 63, reset_cause 64 (nav/nav.h 0x20)
                nav_tag_mismatch += 1
    out["nav_frames_tag_mismatch"] = [nav_tag_mismatch, len(navf)]
    out["mixed_images"] = s.mixed
    tlt = [x[0] for x in tl]

    def st_at(t: float) -> tuple[int, int]:
        i = bisect.bisect_right(tlt, t) - 1
        return (tl[i][1], tl[i][2]) if i >= 0 else (0, 0)

    fw_ = fault_window(ev, s)

    def calm(t: float) -> bool:
        return fw_ is None or not (fw_[0] <= t <= fw_[1])
    flight = [x[0] for x in tl if x[1] == 2]
    out["flight_window_s"] = [round(flight[0] / 16e6, 3), round(flight[-1] / 16e6, 3)] if flight else None
    # phase history (changes)
    hist = []
    for t, st, ph, pf in tl:
        if not hist or hist[-1][1:3] != [STATES.get(st, st), PHASES.get(ph, ph)]:
            hist.append([round(t / 16e6, 3), STATES.get(st, st), PHASES.get(ph, ph)])
    out["state_history"] = hist[:40]
    hold = [(t, st, ph) for t, st, ph, _ in tl if st == 2 and ph == 3]
    segs = []
    for t, _, _ in hold:
        if segs and t - segs[-1][1] < 2 * TICK_CYC:
            segs[-1][1] = t
        else:
            segs.append([t, t])
    out["hold_s"] = round(sum(b - a for a, b in segs) / 16e6, 3)
    out["hold_longest_s"] = round(max((b - a for a, b in segs), default=0) / 16e6, 3)

    def in_hold(t: float) -> bool:
        return any(a <= t <= b for a, b in segs)
    # ---- NAV ticks
    NT = sorted(ev["NT"], key=lambda r: r[4])
    t2s = [r[4] for r in NT]
    X = sorted(ev["X"], key=lambda x: x[2])
    xlo = [x[2] for x in X]
    leads, nav_rows = [], []
    for i, (ep, tick, rise, fall, t2, t0) in enumerate(NT[:-1]):
        nxt = NT[i + 1]
        lead = t2 - t0 if 0 < t2 - t0 < 3 * lead_an else None
        if lead is not None:
            leads.append(lead / CYC)
        lead_next = nxt[4] - nxt[5] if 0 < nxt[4] - nxt[5] < 3 * lead_an else None
        limit = nxt[5] if lead_next is not None else nxt[4] - lead_an
        j = bisect.bisect_left(xlo, t2)
        x = X[j] if j < len(X) and X[j][2] < nxt[4] else None
        st, ph = st_at(t2)
        nav_rows.append({"t": t2, "flight": st == 2, "hold": in_hold(t2), "deliver": None if x is None else (x[3] - t2) / CYC,
                         "end": (fall - t2) / CYC, "past_lead": fall > limit})
    out["imu_lead_us"] = stat(leads)
    out["imu_lead_source"] = ("measured: NAV Timer0 compare -> Timer2 compare (harness t0fix: TCNT0 preload honoured)"
                              if leads else f"analytic {lead_an / CYC} us (Timer0 compare not seen)")
    out["nav_frame_delivered_us"] = {k: stat([r["deliver"] for r in nav_rows if f(r)]) for k, f in
                                     (("pad", lambda r: not r["flight"]), ("flight", lambda r: r["flight"]),
                                      ("hold", lambda r: r["hold"]))}
    out["nav_marker_end_us"] = {k: stat([r["end"] for r in nav_rows if f(r)]) for k, f in
                                (("pad", lambda r: not r["flight"]), ("flight", lambda r: r["flight"]),
                                 ("hold", lambda r: r["hold"]))}
    steps = [abs(b["deliver"] - a["deliver"]) for a, b in zip(nav_rows, nav_rows[1:])
             if a["flight"] and b["flight"] and a["deliver"] is not None and b["deliver"] is not None]
    out["nav_frame_step_us_max_flight"] = round(max(steps), 1) if steps else None
    out["nav_past_next_imu_lead"] = {"all": [sum(r["past_lead"] for r in nav_rows), len(nav_rows)],
                                     "flight": [sum(r["past_lead"] for r in nav_rows if r["flight"]),
                                                sum(1 for r in nav_rows if r["flight"])]}
    # ---- FCU ticks: time-critical, NAV tick -> vane, IMU -> vane
    xhi = [x[3] for x in X]
    fcu_rows = []
    lead_by_t2 = {r[4]: (r[4] - r[5]) if 0 < r[4] - r[5] < 3 * lead_an else None for r in NT}
    for tick, rise, fall, pwm in ev["T"]:
        st, ph = st_at(rise)
        row = {"t": rise, "flight": st == 2, "hold": in_hold(rise), "calm": calm(rise), "crit": (fall - rise) / CYC}
        if pwm and all(pwm[c] for c in vane):
            j = bisect.bisect_right(xhi, rise) - 1
            # fw09 integration (agent A's request): pair only with a COMPLETE exchange (72 bytes): an exchange the NAV's W1
            # watchdog cut short ends late (SS high 1.1 - 1.6 ms after its start) and is followed by the FCU's frame-timeout
            # tick, which no NAV frame triggered (pairing them gave false 3.3 - 3.6 ms NAV tick -> vane figures)
            if j >= 0 and rise - X[j][3] < 400 * CYC and X[j][5] != "-" and len(X[j][5]) >= 144:
                i = bisect.bisect_right(t2s, X[j][2]) - 1
                if i >= 0 and X[j][2] - t2s[i] < TICK_CYC:
                    tv = max(pwm[c] for c in vane) + 2
                    t2 = t2s[i]
                    row["ntv"] = (tv - t2) / CYC
                    ld = lead_by_t2.get(t2)
                    row["imu"] = (tv - t2 + (ld if ld is not None else lead_an)) / CYC
                    row["split"] = ((X[j][3] - t2) / CYC, (rise - X[j][3]) / CYC, (tv - rise) / CYC)
        fcu_rows.append(row)

    def fsel(key, pred):
        return stat([r.get(key) for r in fcu_rows if pred(r)])
    for key, name in (("crit", "fcu_crit_us"), ("ntv", "nav_tick_to_vane_us"), ("imu", "imu_to_vane_us")):
        out[name] = {"pad": fsel(key, lambda r: not r["flight"]), "flight": fsel(key, lambda r: r["flight"]),
                     "hold": fsel(key, lambda r: r["hold"]), "calm_flight": fsel(key, lambda r: r["flight"] and r["calm"])}
    # mode changes (SPEC row 17): the state / phase changes and the operator's commands, the figures within +-1 s
    marks = [(h_[0] * 16e6, f"{h_[1]}/{h_[2]}") for h_ in hist[1:]]
    names = {8: "ON", 9: "OFF", 10: "UP", 11: "DOWN"}
    marks += [(r[6] * CYC, "cmd " + names[r[3]]) for r in ev["UL"] if r[0] == 0 and r[3] in names and r[4] & 1]
    trans = []
    for tm, what in sorted(marks):
        w = [r for r in fcu_rows if abs(r["t"] - tm) < 16e6]
        wn = [r for r in nav_rows if abs(r["t"] - tm) < 16e6]
        trans.append({"t_s": round(tm / 16e6, 3), "what": what,
                      "nav_tick_to_vane_max": max((r["ntv"] for r in w if "ntv" in r), default=None),
                      "imu_to_vane_max": max((r["imu"] for r in w if "imu" in r), default=None),
                      "fcu_crit_max": max((r["crit"] for r in w), default=None),
                      "nav_past_lead": sum(r["past_lead"] for r in wn)})
    out["transitions"] = trans
    top = sorted((r for r in fcu_rows if "ntv" in r and r["flight"]), key=lambda r: -r["ntv"])[:5]
    out["nav_tick_to_vane_top5"] = [{"t_s": round(r["t"] / 16e6, 4), "total_us": round(r["ntv"], 1),
                                     "frame_us": round(r["split"][0], 1), "fcu_late_us": round(r["split"][1], 1),
                                     "fcu_to_vane_us": round(r["split"][2], 1)} for r in top]
    # ---- NAV report lines
    sw = merge_lines(ev["NC"].get("SW", []))
    last_sw = sw[-1][1] if sw else {}
    out["nav_sw"] = {"work_flight_max_us": last_sw.get("wf"), "work_pad_max_us": last_sw.get("wp"),
                     "work_flight_mean_us": round(last_sw["ws"] / last_sw["wn"], 1) if last_sw.get("wn") else None,
                     "flight_ticks": last_sw.get("wn"), "into_imu_lead": last_sw.get("wl"), "past_tick": last_sw.get("wt"),
                     "late_start_max_us": last_sw.get("ls"), "compute_end_max_us": last_sw.get("ce"),
                     "late_push_ticks": last_sw.get("lp"), "stack_free_min": min((d.get("sf", 0xFFFF) for _, d in sw),
                                                                                  default=None),
                     "nav_epochs": len({r[0] for r in ev["NT"]})}
    # fw09 W1 (agent A's request): the NAV's SN line "rs" = nlink_spi_stats()->resyncs = exchanges its watchdog closed
    sn_nav = merge_lines(ev["NC"].get("SN", []))
    out["nav_sw"]["w1_trips"] = max((d.get("rs", 0) for _, d in sn_nav), default=None) if sn_nav else None
    # ---- baro: frames with BARO_NEW per second of flight, BARO_OK in flight
    fl_frames = [f for f in navf if st_at(f[0])[0] == 2]
    per_s: dict = {}
    for f in fl_frames:
        per_s.setdefault(int(f[0] // 16e6), []).append(f[1])
    full = sorted(per_s)[1:-1]
    rates = [sum(1 for fl in per_s[k] if fl & NL_F_BARO_NEW) for k in full]
    out["baro"] = {"flight_seconds": len(full), "samples_per_s": stat(rates, 0),
                   "seconds_outside_50pm1": sum(1 for r in rates if abs(r - 50) > 1),
                   "flight_frames_without_baro_ok": sum(1 for f in fl_frames if not f[1] & NL_F_BARO_OK),
                   "twi_d1_conversions": ev["MX"].get("d1"), "twi_adc_reads_early": ev["MX"].get("adc_early"),
                   # 2026-11-10: the barometer model of the run (MX baro=; a harness before it: the MS5611) and the
                   # BMP280 model's counters - a data read while the measurement runs returns the PREVIOUS sample
                   "sensor": ev["MX"].get("baro", "ms5611"), "twi_bmp280_forced": ev["MX"].get("bmp_forced"),
                   "twi_bmp280_data_reads": ev["MX"].get("bmp_data_reads"),
                   "twi_bmp280_reads_early": ev["MX"].get("bmp_early")}
    # ---- the FCU's report lines
    C = {k: merge_lines(v) for k, v in ev["C"].items()}
    last = {k: (v[-1][1] if v else {}) for k, v in C.items()}
    sn = last.get("SN", {})
    sv = last.get("SV", {})
    out["fcu"] = {"abort_tick": sn.get("at"), "abort_reason": sn.get("ar"),
                  "abort_reason_name": LP_R.get(sn.get("ar") or 0), "land_reason": sv.get("lr"),
                  "land_reason_name": LP_R.get(sv.get("lr") or 0), "land_tick": sv.get("lk"),
                  "hold_ticks": sv.get("ho"), "landed_tick": sv.get("lt"), "safe_tick": sv.get("sf"),
                  "touchdown_by": sv.get("tb"), "esc_u_max": (sv.get("um") or 0) / 1000.0,
                  "esc_u_hold_min": (sv.get("uh") or 0) / 1000.0 if sv.get("uh") not in (None, 0xFFFF) else None,
                  "frames_valid": sn.get("fv"), "frames_bad": sn.get("fb"), "timeout_ticks": sn.get("ft"),
                  "seq_lost": sn.get("fq"), "resyncs": sn.get("rs"), "overruns": sn.get("fo"),
                  "stack_free_min": min((d.get("sf", 0xFFFF) for _, d in C.get("SB", [])), default=None),
                  "pos": {k: last.get("SP", {}).get(k) for k in ("pm", "pq", "pe", "pp", "pl", "xs", "xb", "xt", "xf", "gg",
                                                                   "tz")},
                  "ilink": {k: last.get("SI", {}).get(k) for k in ("ic", "is", "ib", "ro", "re", "bx", "fl", "sr")}}
    out["fcu"]["trim_fallback_ticks"] = last.get("SP", {}).get("xf")
    # fw09 review SYS-2: what the harness injected (X records: fault bits, and whether the FCU received what the NAV
    # sent) against the FCU's counters over the same span (its last SN / SB line before fault_on -> its last line) - the
    # bench check nav_frames_all_counted, here; and the FCU's tick incl. background in the fault window (the bench check
    # babble_loop_inside_tick)
    f_on = next((t for t, nm, _ in ev["E"] if nm == "fault_on"), None)
    f_off = next((t for t, nm, _ in ev["E"] if nm == "fault_off"), None)
    if f_on is not None:
        xr = [r for r in ev["XR"] if r[1]]
        inj = {"exchanges_faulted": len(xr), "mosi_flip": sum(1 for r in xr if r[1] & 1),
               "miso_flip": sum(1 for r in xr if r[1] & 2), "drop": sum(1 for r in xr if r[1] & 4),
               "extra": sum(1 for r in xr if r[1] & 8), "noise": sum(1 for r in xr if r[1] & 16),
               "fcu_received_other_bytes": sum(1 for r in xr if r[2] is False),
               "fcu_received_other_bytes_unfaulted": sum(1 for r in ev["XR"] if not r[1] and r[2] is False)}

        def delta(kind: str, keys: tuple) -> dict:
            rows = [(t, d) for t, d in C.get(kind, []) if any(k in d for k in keys)]
            before = [d for t, d in rows if t < f_on]
            if not rows or not before:
                return {k: None for k in keys}
            return {k: (rows[-1][1].get(k, 0) - before[-1].get(k, 0)) for k in keys}
        fd = {**delta("SN", ("fb", "ft", "rs", "fq")), **delta("SB", ("ov",))}
        out["nlink_fault_accounting"] = {"injected": inj, "fcu_delta": {
            "frames_bad": fd["fb"], "timeouts": fd["ft"], "resyncs": fd["rs"], "seq_lost": fd["fq"],
            "no_frame_ticks": fd["ov"]}}
        w_end = (f_off if f_off is not None else f_on) + 1.1 * 16e6
        out["fcu_tick_incl_background_fault_window_us_max"] = max(
            (d.get("tm", 0) for t, d in C.get("SB", []) if f_on <= t <= w_end and "tm" in d), default=None)
    # record drops: counter increments between consecutive lines, attributed to calm flight when both ends are
    drops = {"calm_flight": 0, "flight_fault_window": 0, "pad_or_landed": 0, "by_counter": {}}
    for kind, keys in (("SL", ("fd", "sd")), ("SV", ("vd",)), ("SP", ("gd",)), ("SI", ("ed",))):
        rows = C.get(kind, [])
        for key in keys:
            prev = None
            for t, d in rows:
                if key not in d:
                    continue
                if prev is not None and d[key] > prev[1]:
                    inc = d[key] - prev[1]
                    drops["by_counter"][key] = drops["by_counter"].get(key, 0) + inc
                    a, b = prev[0], t
                    fl = st_at(a)[0] == 2 and st_at(b)[0] == 2
                    if fl and calm(a) and calm(b):
                        drops["calm_flight"] += inc
                    elif fl:
                        drops["flight_fault_window"] += inc
                    else:
                        drops["pad_or_landed"] += inc
                prev = (t, d[key])
    out["record_drops"] = drops
    # status records in HOLD (the 1 Hz flush + status record: SI "sr" increments)
    si = [(t, d["sr"]) for t, d in C.get("SI", []) if "sr" in d]
    in_h = [(t, v) for t, v in si if in_hold(t)]
    gaps = [b[1] - a[1] for a, b in zip(in_h, in_h[1:])]
    out["log_flush_in_hold"] = {"status_lines_in_hold": len(in_h), "status_records_per_line": stat(gaps, 0),
                                "lines_without_a_new_status_record": sum(1 for g in gaps if g < 1)}
    # background (row 13) and job lengths (row 14)
    sl_h = [d for t, d in C.get("SL", []) if in_hold(t)]
    sb_h = [d for t, d in C.get("SB", []) if in_hold(t)]
    spacing = []
    for a, b in zip(X, X[1:]):
        if in_hold(a[3]) and not a[4] and not b[4]:
            spacing.append((b[2] - a[3]) / CYC)
    out["background_hold"] = {"bg_job_end_after_anchor_us_max": max((d.get("je", 0) for d in sl_h), default=None),
                              "late_pass_job_end_us_max": max((d.get("le", 0) for d in sl_h), default=None),
                              "tick_incl_background_us_max": max((d.get("tm", 0) for d in sb_h), default=None),
                              "frame_to_next_exchange_us_min": round(min(spacing), 1) if spacing else None,
                              "bg_end_limit_us": consts.get("BG_END_US")}
    jobs = {"bg_job_on_bg_reserve_us": ("SI", "bj"), "text_job_us": ("SI", "bt"), "record_job_us": ("SI", "br"),
            "bg_job_normal_pass_us": ("SI", "nj"), "text_job_normal_pass_us": ("SI", "nt"),
            "record_job_normal_pass_us": ("SI", "nr"),
            "longest_bg_job_us": ("SB", "bg"), "snap_job_us": ("SB", "sn"), "ilink_drain_us": ("SB", "il"),
            "position_job_us": ("SP", "pb"), "position_piece_us": ("SP", "pj"), "position_job_end_us": ("SP", "pz"),
            "simbench_own_us": ("SB", "ox")}
    out["job_lengths_max"] = {k: max((d.get(f, 0) for _, d in C.get(kd, [])), default=None) for k, (kd, f) in jobs.items()}
    out["job_lengths_max"]["longest_bg_job_kind"] = next((d.get("bk") for _, d in reversed(C.get("SB", []))
                                                         if d.get("bg") == out["job_lengths_max"]["longest_bg_job_us"]),
                                                         None)
    # simbench_bg_step counts one job in several maxima (bj: every job but the text ones, br: every job but SNAP / text,
    # pb: position jobs): the 1 s windows whose bj / br equal that window's pb are the position job's; the others give
    # the longest non-position job (the SP line nearest in time, < 0.6 s)
    sp = [(t, d.get("pb", 0)) for t, d in C.get("SP", [])]
    spt = [x[0] for x in sp]

    def pb_near(t: int) -> int | None:
        """the longest position job of the two SP windows that overlap the SI window ending at t (None: unknown)"""
        i = bisect.bisect_right(spt, t)
        c = [sp[j][1] for j in (i - 1, i) if 0 <= j < len(sp) and abs(sp[j][0] - t) < 1.1 * 16e6]
        return max(c) if len(c) == 2 else None
    nonpos = {"bj": [], "br": []}
    for t, d in C.get("SI", []):
        p = pb_near(t)
        for k in ("bj", "br"):
            if k in d and p is not None and d[k] > p:     # longer than every position job around it: not one of them
                nonpos[k].append(d[k])
    out["job_lengths_max"]["bg_job_on_bg_reserve_excl_position_us"] = max(nonpos["bj"], default=None)
    out["job_lengths_max"]["record_job_excl_position_us"] = max(nonpos["br"], default=None)
    # ---- ilink commands (COM uplink records), answered, in HOLD.  fw09 review SYS-1: every COM epoch - the records of a
    # COM restarted by the reset fault carry the COM's own clock since that restart (us), so they are placed at the
    # epoch's release (E com_release, in order) + their time
    rel = [t for t, nm, _ in ev["E"] if nm == "com_release"]
    end_t = end[0] if end else 0
    cmds = []
    for ep, u, seq, cmd, flags, number, t_up, t_q, t_snap, t_tel in ev["UL"]:
        if not flags & 1:
            continue
        if ep > len(rel):
            continue                                             # an epoch without its release record: not placeable
        t = (rel[ep - 1] if ep >= 1 else 0) + t_up * CYC
        if t > end_t - 0.1 * 16e6:
            continue
        cmds.append({"t": t, "ep": ep, "cmd": cmd, "answered": bool(flags & 8), "hold": in_hold(t),
                     "flight": st_at(t)[0] == 2, "latency_ms": round((t_tel - t_up) / 1000.0, 2) if flags & 8 else None})
    out["ilink"] = {"uplinks": len(cmds), "unanswered": sum(1 for c in cmds if not c["answered"]),
                    "uplinks_in_hold": sum(1 for c in cmds if c["hold"]),
                    "unanswered_in_hold": sum(1 for c in cmds if c["hold"] and not c["answered"]),
                    "unanswered_in_calm_flight": sum(1 for c in cmds if c["flight"] and calm(c["t"]) and not c["answered"]),
                    "commands_sent": {str(k): sum(1 for c in cmds if c["cmd"] == k) for k in sorted({c["cmd"] for c in cmds})},
                    "com_epochs": sorted({c["ep"] for c in cmds}),
                    "uplinks_after_com_reset": sum(1 for c in cmds if c["ep"] >= 1),
                    "unanswered_after_com_reset": sum(1 for c in cmds if c["ep"] >= 1 and not c["answered"]),
                    "off_sent_s": [round(c["t"] / 16e6, 3) for c in cmds if c["cmd"] == 9],
                    "on_sent": sum(1 for c in cmds if c["cmd"] == 8),
                    "on_unanswered": sum(1 for c in cmds if c["cmd"] == 8 and not c["answered"]),
                    "uplink_to_telem_ms": stat([c["latency_ms"] for c in cmds], 2)}
    # ---- glink
    out["glink"] = glink(ev, L, calm)
    # ---- GNS report (last GQ) and the plant
    if ev["GQ"]:
        raw = bytes.fromhex(ev["GQ"][-1][1])
        if len(raw) >= 62:
            f = struct.unpack_from("<7H6H6B4H5H3HIH", raw)
            names = ("magic", "cycles", "refused", "fin_max_us", "poll_max_us", "busy_pass_max_us", "stack_free", "states",
                     "hold_steps", "gaps", "restarts", "imu_bad", "clock_slips", "cfg_state", "cfg_ok", "ver_ok", "module",
                     "snap_flags", "pos_mode", "epochs", "fused", "ubx_frames", "ubx_bad_ck", "gl_rx_bytes",
                     "gl_rx_overflow", "gl_rx_errors", "gl_tx_bytes", "gl_bad_crc", "gps_rx_overflow", "rcv_epochs",
                     "rcv_dropped", "ticks", "gps_cut_epochs")
            out["gns_report"] = dict(zip(names, f))
            pm = []
            for t, hx in ev["GQ"]:
                b = bytes.fromhex(hx)
                if len(b) >= 32:
                    pm.append((round(t / 16e6, 2), b[31]))      # gns_cosim_report_t.pos_mode (offset 31)
            out["gns_pos_mode_changes"] = [x for i, x in enumerate(pm) if i == 0 or x[1] != pm[i - 1][1]][:20]
    out["plant"] = {k: ev["MX"].get(k) for k in ("h_max", "td_v", "touchdowns", "liftoffs", "tilt_max_deg", "x", "y")}
    out["harness"] = {k: ev["MX"].get(k) for k in ("wdt_fcu", "wdt_com", "wdt_nav", "wdt_gns", "t0fix_nav", "t0fix_gns",
                                                   "nl_lose", "twi_msgs", "bursts", "d1", "d2", "adc_early", "baro",
                                                   "bmp_forced", "bmp_done", "bmp_data_reads", "bmp_early",
                                                   "bmp_calib_reads", "bmp_resets", "bmp_busy_writes", "bmp_normal_writes",
                                                   "twi_rescaled", "twi_not_found", "scl_cycles_per_bit",
                                                   "twi_bits_hist")}
    out["stack_lowest_sp_run"] = {k: ev["M"].get(k) for k in ("fcu_sp_min_run", "nav_sp_min_run", "com_sp_min_run")}
    if s.loop == "closed" and ev["P"]:
        out["validation"] = validate(ev, h, navf, st_at, segs)
        P = ev["P"]
        hold_p = [p for p in P if in_hold(p[0])]
        out["plant"]["hold_tilt_deg"] = stat([p[20] for p in hold_p], 2)
        out["plant"]["hold_h_m"] = stat([p[1] for p in hold_p], 3)
    return out


def validate(ev: dict, h: dict, navf: list, st_at, segs) -> dict:
    """S1e: (a) the co-simulation plant's height against the single-chip bench plant (src/simbench.c plant_step, ported
    line by line) flown by the same ESC commands and tilt; (b) the NAV's estimates in its frames against the plant"""
    P = ev["P"]
    g, uh, tau, dt = h["FC_G0"], h["VC_U_HOVER"], h["EDF_TAU_S"], 0.004
    hb = vb = ul = 0.0
    air = False
    errs, i, tn = [], 0, P[0][0]
    hmax_b = 0.0
    tdv_b = 0.0
    while i < len(P):
        while i + 1 < len(P) and P[i + 1][0] <= tn:
            i += 1
        p = P[i]
        cos_t = math.cos(math.radians(p[20]))
        r = ul / uh
        thrust = r * r if abs(h["MIX_THROTTLE_EXP"] - 2.0) < 1e-9 else (ul / uh) ** h["MIX_THROTTLE_EXP"]
        a = g * (thrust * cos_t - 1.0)
        ul += (p[13] - ul) * (dt / (tau + dt))
        if not air and a <= 0.0:
            vb = 0.0
        else:
            air = True
            vb += a * dt
            hb += vb * dt
            hmax_b = max(hmax_b, hb)
            if hb <= 0.0 and vb < 0.0:
                tdv_b = max(tdv_b, -vb)
                hb = vb = 0.0
                air = False
        if st_at(p[0])[0] == 2:
            errs.append(hb - p[1])
        tn += dt * 16e6
        if i == len(P) - 1:
            break
    pt = [p[0] for p in P]
    est_h, est_tilt = [], []
    for t, fl, hm, vz, tilt, q in navf:
        if st_at(t)[0] != 2:
            continue
        j = bisect.bisect_right(pt, t) - 1
        if j < 0:
            continue
        est_h.append(hm - P[j][1])
        est_tilt.append(tilt - P[j][20])
    rms = (lambda v: round(math.sqrt(sum(x * x for x in v) / len(v)), 4) if v else None)
    return {"bench_plant_replay": {"note": "src/simbench.c plant_step (vertical point mass, EDF lag, u_hover from the "
                                           "image) flown at 4 ms by the co-simulation's ESC commands and tilt",
                                   "h_error_rms_m": rms(errs), "h_error_max_m": round(max(map(abs, errs)), 4) if errs else None,
                                   "bench_h_max_m": round(hmax_b, 4), "cosim_h_max_m": ev["MX"].get("h_max"),
                                   "bench_touchdown_mps": round(tdv_b, 4), "cosim_touchdown_mps": ev["MX"].get("td_v")},
            "nav_estimate_vs_plant": {"h_error_rms_m": rms(est_h), "h_error_max_m": round(max(map(abs, est_h)), 4) if est_h else None,
                                      "tilt_error_rms_deg": rms(est_tilt),
                                      "tilt_error_max_deg": round(max(map(abs, est_tilt)), 4) if est_tilt else None,
                                      "frames": len(est_h)}}


# ---------------------------------------------------------------------------------------------------------------- judge
def judge(res: dict, ctx: dict) -> dict:
    """the co-simulation rows of SPEC section 7 over the runs (each row: PASS / FAIL / NOT_JUDGED with the numbers)"""
    rows: dict = {}
    incomplete = [k for k, v in res.items() if not v.get("completed")]
    res = {k: v for k, v in res.items() if not v.get("missing")}   # no event file: in "harness" (FAIL) only
    # the S1d closed-loop runs (the G1 mixed-image run never flies: judged in row 18.g1 only, and in rows 3 / 14)
    cl = {k: v for k, v in res.items() if v.get("loop") == "closed" and v.get("tree") == "base" and v.get("kind") != "g1"}
    normal = {k: v for k, v in cl.items() if k.startswith("cl_normal") or k == "cl_demo"}

    def mx(v, *path):
        for p in path:
            if v is None:
                return None
            v = v.get(p)
        return v

    def row(n, crit, limit, ok, values, note=""):
        rows[str(n)] = {"criterion": crit, "limit": limit, "verdict": "PASS" if ok else "FAIL", "values": values,
                        **({"note": note} if note else {})}
    # row 3: NAV ticks into the next IMU lead, every scenario
    v3 = {k: {"marker": v["nav_past_next_imu_lead"]["all"], "sw_counter": mx(v, "nav_sw", "into_imu_lead")}
          for k, v in res.items() if v.get("tree") == "base"}
    row(3, "NAV ticks into the next IMU lead (marker + SW counter)", "0 in every scenario",
        all(x["marker"][0] == 0 and (x["sw_counter"] or 0) == 0 for x in v3.values()), v3)
    # row 4: NAV work in flight with TWI
    v4 = {k: mx(v, "nav_sw", "work_flight_max_us") for k, v in cl.items()}
    m4 = max((x for x in v4.values() if x is not None), default=None)
    row(4, "NAV mandatory work in flight with the TWI models (SW wf)", "<= 3200 us; accept max <= 3000 us",
        m4 is not None and m4 <= 3000, {"max_us": m4, "by_run": v4,
                                        "mean_us": {k: mx(v, "nav_sw", "work_flight_mean_us") for k, v in cl.items()}})
    # row 5: FCU time-critical incl. ISRs, every build, in HOLD
    v5 = {k: {"flight_max": mx(v, "fcu_crit_us", "flight", "max"), "hold_max": mx(v, "fcu_crit_us", "hold", "max")}
          for k, v in normal.items()}
    m5 = max((x["flight_max"] or 0 for x in v5.values()), default=None)
    row(5, "FCU time-critical incl. its interrupts (T rise -> fall), every build, in HOLD", "<= 3200 us",
        m5 is not None and m5 <= 3200 and all(x["hold_max"] is not None for k, x in v5.items()), v5)
    # rows 7, 8: IMU -> vane, NAV tick -> vane, closed loop incl. TWI and HOLD
    v7 = {k: {"flight_max": mx(v, "imu_to_vane_us", "flight", "max"), "hold_max": mx(v, "imu_to_vane_us", "hold", "max")}
          for k, v in cl.items()}
    m7 = max((x["flight_max"] or 0 for x in v7.values()), default=None)
    row(7, "IMU sample -> vane (closed loop incl. TWI and HOLD; lead measured)", "<= 4710 us; accept max <= 4500 us",
        m7 is not None and 0 < m7 <= 4500, {"max_us": m7, "by_run": v7})
    v8 = {k: {"flight_max": mx(v, "nav_tick_to_vane_us", "flight", "max"),
              "hold_max": mx(v, "nav_tick_to_vane_us", "hold", "max")} for k, v in cl.items()}
    m8 = max((x["flight_max"] or 0 for x in v8.values()), default=None)
    row(8, "NAV tick -> vane (closed loop incl. TWI and HOLD)", "<= 4000 us; accept max <= 3800 us",
        m8 is not None and 0 < m8 <= 3800, {"max_us": m8, "by_run": v8})
    # row 9: the barometer (BMP280 since 2026-11-10; MS5611 in an image built with it) at 50 Hz in flight, no BARO-lost
    # in a normal run; BMP280: additionally no data read while a measurement runs (the chip would hand the previous
    # sample out silently - the MS5611 answers 0, which its driver refuses and the sample count shows)
    v9 = {k: {"sensor": mx(v, "baro", "sensor"), "bmp280_reads_early": mx(v, "baro", "twi_bmp280_reads_early"),
              "samples_per_s": mx(v, "baro", "samples_per_s"), "seconds_outside_50pm1": mx(v, "baro", "seconds_outside_50pm1"),
              "frames_without_baro_ok": mx(v, "baro", "flight_frames_without_baro_ok"),
              "baro_lost_reason": v["fcu"]["land_reason"] in BARO_LOST_REASONS or v["fcu"]["abort_reason"] in BARO_LOST_REASONS}
          for k, v in normal.items()}
    row(9, "barometer (BMP280; MS5611 as a build option) compensation at 50 Hz in flight (frames with BARO_NEW per "
           "second), no BARO-lost event in a normal run, no BMP280 data read during a measurement",
        "50 +- 1 samples per s", all(x["seconds_outside_50pm1"] == 0 and x["frames_without_baro_ok"] == 0
                                       and not x["baro_lost_reason"] and x["samples_per_s"]
                                       and not x["bmp280_reads_early"] for x in v9.values()), v9,
        "the per-tick compensation (pow 0.2 / tick) is the NAV flight-image profile's figure (navflight); the co-simulation "
        "counts the samples the altitude filter used (NL_F_BARO_NEW in the NAV's frames)")
    # row 11: flight-record drops in calm flight
    v11 = {k: v["record_drops"] for k, v in normal.items()}
    row(11, "flight-record drops in calm flight (FCU fd / sd / vd / gd / ed increments)", "0 in field / demo / nogps / wired / "
        "wired_buffer", all(x["calm_flight"] == 0 for x in v11.values()) and len(v11) == 5, v11)
    # row 12: every ilink command answered in HOLD, HOLD >= 10 s
    v12 = {k: {"hold_s": v["hold_longest_s"], "uplinks_in_hold": v["ilink"]["uplinks_in_hold"],
               "unanswered_in_hold": v["ilink"]["unanswered_in_hold"]} for k, v in normal.items()}
    row(12, "every ilink command answered in position HOLD; HOLD >= 10 s", "0 unanswered, HOLD >= 10 s",
        all(x["unanswered_in_hold"] == 0 and x["uplinks_in_hold"] > 0 for x in v12.values())
        and (v12.get("cl_normal_fcu", {}).get("hold_s") or 0) >= 10.0, v12,
        "HOLD >= 10 s is required of the field-profile run cl_normal_fcu (the tether builds hold at their alt_max)")
    # row 13: background ends before the next exchange; keeps up (HOLD)
    v13 = {k: v["background_hold"] for k, v in normal.items()}
    ok13 = all(x["bg_job_end_after_anchor_us_max"] is not None and x["bg_job_end_after_anchor_us_max"] <= x["bg_end_limit_us"]
               and x["frame_to_next_exchange_us_min"] is not None
               and x["bg_job_end_after_anchor_us_max"] <= x["frame_to_next_exchange_us_min"]
               and (x["tick_incl_background_us_max"] or 0) < 4000 for x in v13.values())
    row(13, "FCU background: the latest job ends before the next exchange (anchor-relative, BG_END_US and the measured "
        "frame -> next exchange), a tick incl. background < 4000 us, in HOLD", "as the bench checks", ok13, v13)
    # row 14: job lengths vs reserves (every base run; the normal runs and the fault runs also apart)
    mc = ctx["trees"]["base"]["main_c"]

    def jmax(sel) -> dict:
        jl: dict = {}
        for k, v in res.items():
            if v.get("tree") != "base" or not sel(k, v):
                continue
            for j, x in v["job_lengths_max"].items():
                if isinstance(x, int) and j != "longest_bg_job_kind":
                    jl[j] = max(jl.get(j, 0), x)
        return jl
    jl = jmax(lambda k, v: True)
    jl_normal = jmax(lambda k, v: k in normal)
    jl_fault = {k: v["job_lengths_max"] for k, v in res.items() if v.get("tree") == "base" and k not in normal
                and v.get("loop") == "closed"}
    # fw09 integration: agent B's simbench.c reports every job against the reserve it STARTS on (bj: SNAP frame + status
    # record on BG_RESERVE_US; br: the record jobs on REC_RESERVE_US; bt: the text jobs on COM_TEXT_RESERVE_US; pj: a
    # position piece on POS_RESERVE_US / POS_WALL_*; pos_work_now - right after the time-critical part, no reserve - only
    # in pb), so the '_excl_position' window heuristic of the first version is no longer needed (kept as information)
    # The VTOL background runs two passes (main.c time_left_us): the normal pass starts a job only if it ends `reserve`
    # before BG_END_US; the late pass (g_late, on into the exchange window) only if it ends LATE_WALL(reserve) before
    # LATE_END_US - the reserve stretched x 115 / 64 + 10 us for the exchange's slave interrupts.  So a normal-pass job is
    # judged against the reserve, a late-pass job against LATE_WALL(reserve) (simbench.c: nj / nt / nr = normal pass only;
    # bj / bt / br = both passes, an upper bound of the late-pass jobs).
    def late_wall(us: int | None) -> int | None:
        return None if us is None else ((us * 115) >> 6) + 10
    checks = {"normal pass: bg_job_normal_pass_us <= BG_RESERVE_US": ("bg_job_normal_pass_us", "BG_RESERVE_US", False),
              "normal pass: text_job_normal_pass_us <= COM_TEXT_RESERVE_US": ("text_job_normal_pass_us",
                                                                              "COM_TEXT_RESERVE_US", False),
              "normal pass: record_job_normal_pass_us <= REC_RESERVE_US": ("record_job_normal_pass_us", "REC_RESERVE_US",
                                                                           False),
              "both passes: bg_job_on_bg_reserve_us <= LATE_WALL(BG_RESERVE_US)": ("bg_job_on_bg_reserve_us",
                                                                                   "BG_RESERVE_US", True),
              "both passes: text_job_us <= LATE_WALL(COM_TEXT_RESERVE_US)": ("text_job_us", "COM_TEXT_RESERVE_US", True),
              "both passes: record_job_us <= LATE_WALL(REC_RESERVE_US)": ("record_job_us", "REC_RESERVE_US", True),
              "position_piece_us <= POS_RESERVE_US": ("position_piece_us", "POS_RESERVE_US", False)}
    isr = mc.get("ILINK_RX_ISR_CYCLES", 51)

    def reserve_for(job: int | None) -> int | None:
        """main.c FC_RESERVE_FOR_US(job): the job stretched by a COM sending all the time (the ilink RX interrupt every
        640-cycle byte) and the log UDRE (46 of every 2778 cycles), rounded up to 10 us (agent B's method, WP-F)"""
        if job is None:
            return None
        return ((job * (640 + isr) * 2778 + job * 46 * 640 + 640 * 2778 - 1) // (640 * 2778) + 9) // 10 * 10

    def reserve_for_late(job: int | None) -> int | None:
        """the smallest reserve R (10 us steps) with LATE_WALL(R) >= FC_RESERVE_FOR_US(job): agent B's rule applied to a
        late-pass job, whose measured length already includes the exchange's slave interrupts"""
        need = reserve_for(job)
        if need is None:
            return None
        r = 0
        while late_wall(r) < need:
            r += 10
        return r

    def worst(j: str) -> str | None:
        return max(((kk, vv["job_lengths_max"].get(j)) for kk, vv in res.items()
                    if vv.get("tree") == "base" and isinstance(vv.get("job_lengths_max", {}).get(j), int)),
                   key=lambda x: x[1], default=(None, None))[0]
    vals = {}
    for k, (j, r, late) in checks.items():
        lim = late_wall(mc.get(r)) if late else mc.get(r)
        vals[k] = {"job_us_all_runs": jl.get(j), "job_us_normal_runs": jl_normal.get(j), "reserve_us": mc.get(r),
                   "limit_us": lim, "worst_run": worst(j),
                   "B_rule_min_reserve_us": None if r == "POS_RESERVE_US" else
                   (reserve_for_late(jl.get(j)) if late else reserve_for(jl.get(j)))}
    measured_in = {n: mc.get(n) for n in ("FC_BG_JOB_US_VTOL", "FC_REC_JOB_US_VTOL", "FC_TEXT_JOB_US_VTOL",
                                          "FC_BG_LATE_JOB_US_VTOL", "FC_REC_LATE_JOB_US_VTOL", "FC_TEXT_LATE_JOB_US_VTOL")}
    row(14, "background reserves cover the longest job (job lengths measured in the co-simulation, every run, ISRs "
        "included; normal pass vs the reserve, late pass vs LATE_WALL(reserve))", "longest job <= reserve",
        all(x["job_us_all_runs"] is not None and x["limit_us"] is not None and x["job_us_all_runs"] <= x["limit_us"]
            for x in vals.values()),
        {"checks": vals, "main_c_job_inputs": measured_in, "job_lengths_max_us_all_runs": jl,
         "job_lengths_max_us_normal_runs": jl_normal, "job_lengths_fault_runs": jl_fault},
        "nj / nt / nr = the normal pass, bj / bt / br = both passes (agent B's per-reserve maxima; the pass split and pj "
        "= position pieces only were added in the integration: pos_work_now runs on no reserve and is only in pb = "
        "position_job_us).  B_rule_min_reserve_us = agent B's rule main.c FC_RESERVE_FOR_US(job) for the normal pass; for "
        "the late pass the smallest R with LATE_WALL(R) >= FC_RESERVE_FOR_US(job)")
    # row 16: glink L = 3
    v16 = {k: {"margin_min_us": mx(v, "glink", "margin_to_tick_kL_us", "min"),
               "tilt_ok": v["glink"]["tilt_parts_at_k_plus_L"], "tilt_wrong": v["glink"]["tilt_parts_wrong"]}
           for k, v in cl.items() if v["build"] in ("fcu", "fcu_demo")}
    row(16, "glink latency L = 3: RESULT complete before the frame build of tick k + L; TILT parts at k + L",
        "margin >= 2000 us, all TILT parts at k + L",
        all(x["margin_min_us"] is not None and x["margin_min_us"] >= 2000 and x["tilt_wrong"] == 0 and x["tilt_ok"] > 0
            for k, x in v16.items() if k in ("cl_normal_fcu", "cl_demo")), v16,
        "judged on the GPS builds' normal runs (fcu, fcu_demo); fault runs listed for information")
    # row 17: transients - every S1d scenario keeps rows 3, 7, 8, 11, 12, 16
    v17 = {}
    for k, v in cl.items():
        r3 = v["nav_past_next_imu_lead"]["all"][0] == 0 and (mx(v, "nav_sw", "into_imu_lead") or 0) == 0
        r7 = (mx(v, "imu_to_vane_us", "calm_flight", "max") or 1e9) <= 4500
        r8 = (mx(v, "nav_tick_to_vane_us", "calm_flight", "max") or 1e9) <= 3800
        r11 = v["record_drops"]["calm_flight"] == 0
        r12 = v["ilink"]["unanswered_in_calm_flight"] == 0
        g = v["glink"]
        r16 = v["build"] not in ("fcu", "fcu_demo") or ((mx(g, "margin_to_tick_kL_us", "min") or 0) >= 2000
                                                        and g["tilt_parts_wrong"] == 0)
        # fw09 review SYS-5: the log flush (1 Hz flush + status record) inside HOLD - every status line of HOLD with a new
        # status record, and one status line per HOLD second (the whole seconds of HOLD, one window lost at an edge)
        lf = v["log_flush_in_hold"]
        hold_whole_s = int(v.get("hold_s") or 0)
        rflush = (lf["lines_without_a_new_status_record"] == 0
                  and lf["status_lines_in_hold"] >= max(0, hold_whole_s - 1))
        v17[k] = {"row3": r3, "row7": r7, "row8": r8, "row11": r11, "row12": r12, "row16": r16, "log_flush": rflush,
                  "imu_to_vane_calm_max": mx(v, "imu_to_vane_us", "calm_flight", "max"),
                  "nav_tick_to_vane_calm_max": mx(v, "nav_tick_to_vane_us", "calm_flight", "max"),
                  "status_lines_in_hold": lf["status_lines_in_hold"], "hold_s": v.get("hold_s"),
                  "lines_without_a_new_status_record": lf["lines_without_a_new_status_record"],
                  "completed": v["completed"]}
    row(17, "transients (take-off, mode changes, GPS loss, log flush) and every S1d run: rows 3, 7, 8, 11, 12, 16 hold "
        "(calm flight: outside the fault window); the log flush in HOLD: a new status record in every status line, one "
        "line per HOLD second", "each PASS",
        all(all(x[r] for r in ("row3", "row7", "row8", "row11", "row12", "row16", "log_flush")) and x["completed"]
            for x in v17.values()), v17)
    # row 18: fault outcomes
    v18 = {}
    for k, v in res.items():
        if v.get("kind") not in ("fault", "transient", "g1") and not k.startswith("cl_gps"):
            continue
        f = v["fcu"]
        ar, lr = f["abort_reason"], f["land_reason"]
        # the first LAND before the operator's OFF (COM_SB_OFF_S 30 s; open loop: none) was a rule's
        early_land = bool(f["land_tick"]) and f["land_tick"] * 0.004 < (29.5 if v["loop"] == "closed" else 1e9)
        # fw09 review SYS-1: the LAND's own cause - an OFF the operator sent before it, else the supervisor's reason byte
        # at the first LAND (SV lr: a rule's code 1..0x33, LP_R) - instead of naming every LAND after 29.5 s "operator OFF"
        land_s = round(f["land_tick"] * 0.004, 3) if f["land_tick"] else None
        # the LAND phase's first FCU4 frame on the co-simulation clock (the FCU's tick count is not that clock)
        land_g = next((h_[0] for h_ in v.get("state_history", []) if h_[2] == "LAND"), None)
        off_before = [t for t in v["ilink"].get("off_sent_s", []) if land_g is not None and t <= land_g]
        if land_s is None:
            land_cause = None
        elif off_before:
            land_cause = "operator OFF"
        elif lr in LP_R:
            land_cause = LP_R[lr]
        else:
            land_cause = f"reason byte 0x{(lr or 0):02x} (no OFF sent before it)"
        outcome = {"abort": LP_R.get(ar) if ar else None, "abort_tick": f["abort_tick"],
                   "land_before_off": early_land, "land_reason": land_cause, "land_s": land_s, "land_phase_seen_s": land_g,
                   "fcu_land_reason_byte": lr, "operator_off_sent_s": v["ilink"].get("off_sent_s"),
                   "nav_epochs": v["nav_sw"]["nav_epochs"], "nav_wdt_resets": v["harness"].get("wdt_nav"),
                   "nav_w1_trips": v["nav_sw"].get("w1_trips"),
                   "timeouts": f["timeout_ticks"], "resyncs": f["resyncs"], "frames_bad": f["frames_bad"],
                   "gns_pos_mode_changes": v.get("gns_pos_mode_changes"), "completed": v["completed"]}
        name = v["fault"]
        acc = v.get("nlink_fault_accounting")
        if name == "nav_pulse":
            ok = ar == 10
        elif name in ("nl_flip", "nl_drop", "nl_extra", "nl_noise"):
            ok = not ar and v["nav_sw"]["nav_epochs"] == 1 and not early_land
            # fw09 review SYS-2 (the bench's nav_frames_all_counted): every injected fault the FCU met counted, nothing
            # else - refused frames (CRC) for a byte changed at the FCU, a timeout + resync for a lost byte, a refused
            # frame + resync for a phantom byte; each lost frame is one sequence gap and one tick without a frame
            inj, d = (acc or {}).get("injected", {}), (acc or {}).get("fcu_delta", {})
            if name in ("nl_flip", "nl_noise"):
                exp = {"frames_bad": inj.get("fcu_received_other_bytes"), "timeouts": 0, "resyncs": 0}
            elif name == "nl_drop":
                exp = {"frames_bad": 0, "timeouts": inj.get("drop"), "resyncs": inj.get("drop")}
            else:
                exp = {"frames_bad": inj.get("extra"), "timeouts": 0, "resyncs": inj.get("extra")}
            counted = (acc is not None and all(d.get(kk) == vv for kk, vv in exp.items())
                       and (exp["frames_bad"] or 0) + (exp["timeouts"] or 0) > 0
                       and d.get("seq_lost") == d.get("frames_bad", 0) + d.get("timeouts", 0)
                       and d.get("no_frame_ticks") == d.get("frames_bad", 0) + d.get("timeouts", 0)
                       and inj.get("fcu_received_other_bytes_unfaulted") == 0)
            outcome["faults_counted"] = {"ok": counted, "expected": exp, "fcu_delta": d, "injected": inj}
            ok = ok and counted
        elif name == "reset":
            # COM reset 0.5 s: no abort, the NAV unaffected, no LAND before the operator's OFF time and no link-rule LAND,
            # every uplink of the restarted COM answered.  The synthetic operator restarts with the COM image (its script
            # runs on the COM's clock), so its OFF comes 30 s after the restart: a later LAND is judged by its cause - the
            # operator's OFF, or the endurance rule when no OFF was sent before it (designed, not a fault effect)
            il = v["ilink"]
            ok = (not ar and v["nav_sw"]["nav_epochs"] == 1 and not early_land and lr not in (1, 2, 14)
                  and il.get("uplinks_after_com_reset", 0) > 0 and il.get("unanswered_after_com_reset", 1) == 0
                  and land_cause in (None, "operator OFF", "V_ENDURANCE"))
            outcome["uplinks_after_com_reset"] = il.get("uplinks_after_com_reset")
            outcome["unanswered_after_com_reset"] = il.get("unanswered_after_com_reset")
        elif name == "cut":
            ok = not ar and early_land and lr == 14
        elif name == "nl_lose_cmp":
            # W1: the watchdog closed the cut exchange (>= 1 trip where the NAV reports it), no NAV reset, no abort
            trips = v["nav_sw"].get("w1_trips")
            ok = (not ar and (v["harness"].get("wdt_nav") or 0) == 0 and v["nav_sw"]["nav_epochs"] == 1
                  and (trips is None or trips >= 1))
        elif name in ("gns_hold", "gns_pulse"):
            # held: the slot stops -> TRIM fallback -> V11 GPS_LOST LAND (attitude only); a 1 s pulse: the fallback
            # bridges it (no LAND required); either way the NAV runs on (one epoch, no tick into the IMU lead)
            nav_ok = v["nav_sw"]["nav_epochs"] == 1 and v["nav_past_next_imu_lead"]["all"][0] == 0
            ok = not ar and nav_ok and (lr == 24 and early_land if name == "gns_hold" else True)
            outcome["designed"] = "V11 attitude-only hold / land, no NAV effect"
            outcome["trim_fallback_ticks"] = f["pos"].get("xf")
        elif name == "babble":
            # fw09 review SYS-2 (the bench's babble_loop_inside_tick): the FCU's tick incl. background stays inside the
            # tick in the windows of the babble
            tw = v.get("fcu_tick_incl_background_fault_window_us_max")
            ok = not ar and tw is not None and 0 < tw < 4000
            outcome["fcu_tick_incl_background_fault_window_us_max"] = tw
        elif name == "gps_cut":
            # 3 s: DR -> ATT -> POS again, no LAND; 20 s: V11 GPS_LOST LAND; the NAV runs on
            long_cut = (v.get("fault_len_ms") or 0) > 5000
            ok = not ar and v["nav_sw"]["nav_epochs"] == 1 and ((early_land and lr == 24) if long_cut else not early_land)
        elif v.get("kind") == "g1":
            # fw09 review F1: a mixed image set never leaves SAFE; the NAV flags the other FCU's tag; every ON is answered
            # (and refused: the state stays SAFE); nothing aborts
            il = v["ilink"]
            states = {h_[1] for h_ in v.get("state_history", [])}
            tagf = v.get("nav_frames_tag_mismatch", [0, 0])
            ok = (not ar and states == {"SAFE"} and il.get("on_sent", 0) > 0 and il.get("on_unanswered", 1) == 0
                  and tagf[0] > 0 and v.get("flight_window_s") is None)
            outcome.update({"states_seen": sorted(states), "on_sent": il.get("on_sent"),
                            "on_unanswered": il.get("on_unanswered"), "nav_frames_tag_mismatch": tagf})
        else:
            ok = not ar
        if name in ("gns_hold", "gns_pulse", "gps_cut"):
            # fw09 review SYS-2 (the bench's trim_fallback_only_in_the_dropout): TRIM fallback ticks <= 5 per 50 Hz cycle
            # without a RESULT (STATEs the NAV sent - RESULTs that came back) + 2 cycles
            g = v["glink"]
            dropped = max(0, (g.get("states") or 0) - (g.get("results") or 0))
            xf = f.get("trim_fallback_ticks") or 0
            tr_ok = xf <= 5 * (dropped + 2)
            outcome["trim_fallback_check"] = {"ok": tr_ok, "trim_ticks": xf, "cycles_without_result": dropped,
                                              "limit": 5 * (dropped + 2)}
            ok = ok and tr_ok
        outcome["verdict"] = "PASS" if ok and v["completed"] else "FAIL"
        outcome["expect"] = v["expect"]
        v18[k] = outcome
    # the same outcomes by fault family (run_simbench.py defers its self-load checks to these: SELF_LOAD_CHECKS)
    fam = {"18.nlink": ("nl_flip", "nl_drop", "nl_extra", "nl_noise", "nav_pulse"),
           "18.gns": ("gns_hold", "gns_pulse", "gps_cut"), "18.ilink": ("cut", "reset", "babble"),
           "18.w1": ("nl_lose_cmp",)}
    for rn, faults in fam.items():
        sel = {k: x for k, x in v18.items() if res[k].get("tree") == "base" and res[k].get("fault") in faults}
        rows[rn] = {"criterion": f"faults {', '.join(faults)}: designed outcomes (part of row 18)",
                    "verdict": "PASS" if sel and all(x["verdict"] == "PASS" for x in sel.values()) else "FAIL",
                    "values": {k: x["verdict"] for k, x in sel.items()}}
    sel = {k: x for k, x in v18.items() if res[k].get("tree") == "base" and res[k].get("kind") == "g1"}
    rows["18.g1"] = {"criterion": "G1 mixed image set refused on the ground (SPEC 3.2; part of row 18)",
                     "verdict": "PASS" if sel and all(x["verdict"] == "PASS" for x in sel.values()) else "FAIL",
                     "values": {k: x["verdict"] for k, x in sel.items()}}
    # fw09 W1 (agent A's request): no watchdog trip in any normal / transient run (closed loop, this tree)
    quiet = {k: v["nav_sw"].get("w1_trips") for k, v in res.items()
             if v.get("tree") == "base" and v.get("loop") == "closed" and v.get("kind") in ("normal", "transient")}
    rows["18.w1"]["values"]["w1_trips_normal_and_transient_runs"] = quiet
    if any(x for x in quiet.values()):
        rows["18.w1"]["verdict"] = "FAIL"
    row(18, "faults: designed outcomes", "nav_pulse I9 ABORT; nl_* no abort, every injected fault counted; reset no "
        "abort, no link LAND, the restarted COM answered; cut link LAND; nl_lose_cmp 1 missed frame, no abort (W1); GNS "
        "hold / pulse V11 hold / land, no NAV effect, TRIM ticks inside the dropout; babble: the tick incl. background "
        "< 4000 us; G1 mixed image set refused on the ground",
        all(x["verdict"] == "PASS" for k, x in v18.items() if res[k].get("tree") == "base"), v18)
    # row 19: exchange watchdog cost, open loop, a tree with W1 against one without
    comp = {k: v for k, v in res.items() if k.startswith("ol_normal")}
    v19 = {k: {"nav_tick_to_vane_flight_max": mx(v, "nav_tick_to_vane_us", "flight", "max"),
               "frame_delivered_flight_max": mx(v, "nav_frame_delivered_us", "flight", "max"),
               "imu_to_vane_flight_max": mx(v, "imu_to_vane_us", "flight", "max")} for k, v in comp.items()}
    wd = ctx.get("wd_pair")
    if wd and f"ol_normal_{wd[0]}" in v19 and f"ol_normal_{wd[1]}" in v19:
        a, b = v19[f"ol_normal_{wd[0]}"], v19[f"ol_normal_{wd[1]}"]
        d1 = round(b["nav_tick_to_vane_flight_max"] - a["nav_tick_to_vane_flight_max"], 1)
        d2 = round(b["frame_delivered_flight_max"] - a["frame_delivered_flight_max"], 1)
        ra, rb = res[f"ol_normal_{wd[0]}"], res[f"ol_normal_{wd[1]}"]
        dm1 = round(mx(rb, "nav_tick_to_vane_us", "flight", "mean") - mx(ra, "nav_tick_to_vane_us", "flight", "mean"), 1)
        dm2 = round(mx(rb, "nav_frame_delivered_us", "flight", "mean") - mx(ra, "nav_frame_delivered_us", "flight", "mean"), 1)
        row(19, "exchange watchdog cost (open loop: tree with W1 minus tree without; flight max and mean)", "<= 5 us",
            abs(d1) <= 5 and abs(d2) <= 5, {"delta_nav_tick_to_vane_us": d1, "delta_frame_delivered_us": d2,
                                             "delta_mean_nav_tick_to_vane_us": dm1, "delta_mean_frame_delivered_us": dm2,
                                             "runs": v19},
            f"trees {wd[0]} (without W1) and {wd[1]} (with W1); the base images carry no W1 (agent A's WP-N)")
    else:
        rows["19"] = {"criterion": "exchange watchdog cost", "verdict": "NOT_JUDGED", "values": v19,
                      "note": "needs --extra-tree pairs with and without W1 (--wd-pair NAME_WITHOUT,NAME_WITH)"}
    if incomplete:
        rows["harness"] = {"verdict": "FAIL", "incomplete_runs": incomplete}
    return rows


# ---------------------------------------------------------------------------------------------------------------- main
def tree_ctx(fw: str, build: bool, jobs: int, closed: bool) -> dict:
    envs_cl = cl_all_envs()
    envs_ol = sorted(set(OL_ENVS.values()))
    envs = (envs_cl if closed else []) + envs_ol
    if build:
        imgs = pio_build(fw, envs, jobs)
    else:
        imgs = image_info(fw, envs)
    t: dict = {"fw": fw, "images": {e: {k: v for k, v in imgs[e].items() if k in ("sha256", "static_ram_bytes",
                                                                                    "flash_bytes")} for e in envs}}
    old_fw = rc.FW
    try:
        rc.FW = fw                                                  # pwm_registers reads this tree's params_gen.h
        pwm, outs = rc.pwm_registers()
    finally:
        rc.FW = old_fw
    t["pwm"], t["pwm_outs"] = pwm, (pwm, outs)
    t["pwm_map"] = [outs["VANE1"], outs["VANE2"], outs["VANE3"], outs["VANE4"], outs["ESC"]]
    t["ol_images"] = {e: imgs[e] for e in envs_ol}
    t["ol_syms"] = {"com_report": sym(imgs["com_cosim"]["elf"], "g_cosim_report")}
    t["header"] = {"fcu_cosim": header_probe(fw, "fcu_cosim")}
    # fw09 (WP-F, agent B): the VTOL reserves are main.c BG_RESERVE_US_VTOL / REC_RESERVE_US_VTOL / COM_TEXT_EXTRA_US_VTOL
    # (BG_RESERVE_US / REC_RESERVE_US / COM_TEXT_RESERVE_US are macros of them now); FC_*_JOB_US_VTOL = the job lengths
    # they were derived from (0 = not measured)
    mc = source_defines(os.path.join(fw, "src", "main.c"), (
        "BG_RESERVE_US_VTOL", "REC_RESERVE_US_VTOL", "COM_TEXT_EXTRA_US_VTOL", "POS_RESERVE_US", "NLINK_EARLY_US",
        "FC_BG_JOB_US_VTOL", "FC_REC_JOB_US_VTOL", "FC_TEXT_JOB_US_VTOL", "FC_BG_LATE_JOB_US_VTOL",
        "FC_REC_LATE_JOB_US_VTOL", "FC_TEXT_LATE_JOB_US_VTOL", "ILINK_RX_ISR_CYCLES"))
    if "BG_RESERVE_US_VTOL" in mc:
        mc["BG_RESERVE_US"] = mc["BG_RESERVE_US_VTOL"]
        mc["REC_RESERVE_US"] = mc.get("REC_RESERVE_US_VTOL")
        if "COM_TEXT_EXTRA_US_VTOL" in mc:
            mc["COM_TEXT_RESERVE_US"] = mc["BG_RESERVE_US_VTOL"] + mc["COM_TEXT_EXTRA_US_VTOL"]
    else:                                                       # a tree before fw09 WP-F (one reserve set)
        mc.update(source_defines(os.path.join(fw, "src", "main.c"), ("BG_RESERVE_US", "REC_RESERVE_US")))
    t["main_c"] = mc
    h0 = t["header"]["fcu_cosim"]
    if "NLINK_EARLY_US" in t["main_c"]:
        t["main_c"]["BG_END_US"] = int(h0["FC_TICK_US"]) - t["main_c"]["NLINK_EARLY_US"] - int(h0["NLINK_XFER_US"])
    if closed:
        t["cl_images"] = {e: imgs[e] for e in envs_cl}
        t["cl_syms"] = {}
        for b in CL_SFX:
            cs = cl_set(b)
            c = cs["com"]
            t["cl_syms"].setdefault(c, {"com_report": sym(imgs[c]["elf"], "g_cosim_report")})
            g = cs["gns"]
            if g not in t["cl_syms"]:
                t["cl_syms"][g] = {"gps": sym(imgs[g]["elf"], "g_cosim_gps"),
                                   "gnsrep": sym(imgs[g]["elf"], "g_gns_cosim_report")}
                if t["cl_syms"][g]["gps"][1] != 24:
                    raise SystemExit(f"{g}: g_cosim_gps is {t['cl_syms'][g]['gps'][1]} bytes, the harness writes 24")
            t["header"][cs["fcu"]] = header_probe(fw, cs["fcu"])
            # the four images of one run must carry one FC_PARAMS_TAG (fw09 G1)
            hn = header_probe(fw, cs["nav"])
            # 2026-11-10: the barometer model at 0x77 is the one this NAV image was built for (params_gen.h
            # FC_BARO_SENSOR as the NAV env compiles it; a tree without the define has the MS5611 driver only)
            t.setdefault("cl_baro", {})[cs["nav"]] = (
                "bmp280" if "FC_BARO_SENSOR" in hn and hn["FC_BARO_SENSOR"] == hn.get("FC_BARO_BMP280") else "ms5611")
            tags = {cs["nav"]: hn.get("FC_PARAMS_TAG"),
                    **{e: header_probe(fw, e).get("FC_PARAMS_TAG") for e in (cs["gns"], cs["com"])}}
            tags[cs["fcu"]] = t["header"][cs["fcu"]].get("FC_PARAMS_TAG")
            if len(set(tags.values())) != 1:
                raise SystemExit(f"closed-loop set of {b}: FC_PARAMS_TAG differs between its images {tags}")
            t.setdefault("cl_tags", {})[b] = tags
    return t


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--only", help="comma-separated scenario names (default: all)")
    ap.add_argument("--jobs", type=int, default=2, help="simulator processes at a time (default 2)")
    ap.add_argument("--pio-jobs", type=int, default=int(os.environ.get("PLATFORMIO_RUN_JOBS", "2")))
    ap.add_argument("--no-build", action="store_true", help="use the images already built")
    ap.add_argument("--analyse-only", action="store_true", help="analyse the event files already in tools/cosim/build/runs4")
    ap.add_argument("--extra-tree", action="append", default=[], help="NAME=<firmware/atmega128_fc dir with .pio builds>: "
                    "open-loop runs on its images (row 19 / nl_lose_cmp with W1)")
    ap.add_argument("--wd-pair", help="NAME_WITHOUT_W1,NAME_WITH_W1 of the extra trees (row 19)")
    ap.add_argument("--report", default=REPORT)
    a = ap.parse_args(argv)
    extra = dict(x.split("=", 1) for x in a.extra_tree)
    scns = scenarios(extra)
    if a.only:
        want = set(a.only.split(","))
        scns = [s for s in scns if s.name in want]
    ctx: dict = {"trees": {}}
    if a.wd_pair:
        ctx["wd_pair"] = a.wd_pair.split(",")
    t0 = time.time()
    need_cl = any(s.loop == "closed" for s in scns)
    ctx["trees"]["base"] = tree_ctx(FW, not (a.no_build or a.analyse_only), a.pio_jobs, True)
    for name, fw in extra.items():
        ctx["trees"][name] = tree_ctx(os.path.abspath(fw), False, a.pio_jobs, False)
    exe, warns = build_harness()
    print(f"harness {exe} built ({len(warns)} warnings); trees: {', '.join(ctx['trees'])}; {len(scns)} runs", flush=True)
    results: dict = {}
    runinfo: dict = {}
    if not a.analyse_only:
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
            futs = [pool.submit(run_one, exe, s, ctx) for s in scns]
            for fu in concurrent.futures.as_completed(futs):
                s, dt, rcode, tail = fu.result()
                runinfo[s.name] = {"wall_s": round(dt, 1), "exit": rcode}
                print(f"  {s.name}: exit {rcode}, {dt:.0f} s", flush=True)
    for s in scns:
        if not os.path.exists(s.path):
            results[s.name] = {"scenario": s.name, "completed": False, "missing": True, "kind": s.kind, "tree": s.tree,
                               "loop": s.loop}
            continue
        r = analyse(s, ctx)
        r["kind"] = s.kind
        r["run"] = runinfo.get(s.name)
        results[s.name] = r
    rows = judge(results, ctx)
    report = {"what": "SPEC_firmware_v09 section 7 co-simulation rows, four-core closed-loop co-simulation (WP-S)",
              "source_digest": rc.rsb.source_digest(FW),     # run_simbench.py defers its self-load checks on equality
              "complete_set": not a.only, "generated_s": round(time.time() - t0, 1), "harness": {"exe": exe, "warnings": warns,
                                                                     "sha256": hashlib.sha256(open(exe, "rb").read()).hexdigest()},
              "plant_wind_mps2_ASSUMED": WIND, "plant_drag_k_per_s_ASSUMED": DRAG_K, "fault_at_ms_closed_loop": FAULT_AT_CL_MS,
              "trees": {k: {kk: vv for kk, vv in v.items() if kk in ("fw", "images", "main_c", "pwm_map", "cl_baro")} | {
                  "header_fcu_cosim_cl": v["header"].get("fcu_cosim_cl")} for k, v in ctx["trees"].items()},
              "rows": rows, "runs": results}
    os.makedirs(os.path.dirname(a.report), exist_ok=True)
    with open(a.report, "w", encoding="utf-8", newline="\n") as f:
        json.dump(report, f, indent=1, default=str)
    print(f"-> {a.report}")
    for k, v in rows.items():
        print(f"  row {k}: {v['verdict']}  {v.get('criterion', '')}")
    return 0 if all(v["verdict"] != "FAIL" for v in rows.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
