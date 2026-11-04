#!/usr/bin/env python3
"""Measure the 250 Hz loop of the FCU firmware (v0.7 flight control unit, both mission modes) in the simavr simulator.

v0.7: params.json ``mission.mode`` is a build-time choice (``src/fc_mode.h``).  ``--mode vtol`` (the plain envs, which
fly params.json's mode - vtol) runs the VTOL profile of ``src/simbench.h`` (ON -> spool warning -> CLIMB -> HOLD -> UP /
DOWN -> OFF -> LAND -> touchdown -> LANDED -> SAFE -> a second ON and the NAV reset in its climb), the synthetic NAV
flying a vertical point mass on the FCU's own ESC command, and parses its ``SV`` line; ``--mode ballistic`` runs the
v0.6.3 profile on the ``*_ballistic`` envs; ``--mode both`` (default) runs both.  Each mode writes
``host_test/build/simbench_report_<mode>.json``; ``simbench_report.json`` is the report of params.json's mode.  The v0.6.3
reference numbers (``V063``) are this tool's run of the v0.6.3 source snapshot.

v0.7.1 (VTOL build): the FCU tick order is attitude controller -> vanes -> ilink RX -> FCU stack (supervisor + vertical
controller v2) -> ESC (params.json mission.vtol.fcu_tick_order_note; the sections below are named in that order); the
synthetic NAV pushes at ``SB_NAV_PUSH_US_VTOL`` (``tools/run_nav_simbench.py --mode vtol`` checks it is not earlier than
the VTOL NAV's measured frame-ready time) and its body-z specific force is the plant's (g on the ground: the support
signature).  ``--profile field|demo|both`` (default both) runs the VTOL bench of each build profile (``fcu_simbench`` /
``fcu_simbench_demo``; the micro-benchmark and co-simulation builds are the field profile's - the demo profile changes
numbers, not code).  New VTOL checks: the FCU frame's thr_q byte (``SI tq`` = 0: 0 outside POWERED, <= 250; ``SV tx`` the
largest, equal to the largest ESC command), the touchdown confirmed by a detector signature (``SV tb``), the v0.7.1
timing criteria (params.json mission.vtol / the architect's decision 12): NAV tick -> vanes <= one tick, IMU sample ->
vanes = the NAV's FC_IMU_LEAD_COUNTS lead + NAV tick -> vanes <= the 6-DOF's FC_COMPUTE_DELAY_S_MEASURED (4.71 ms, from
sim/fc_timing.py), time-critical <= 80 %, flight records: none dropped in calm flight (reported separately under the COM
floods); the ballistic checks are v0.7.0's.

Builds the PlatformIO environments ``fcu_simbench`` (the FCU's flight loop with a synthetic NAV on the nlink and a
synthetic COM on the ilink, see ``src/simbench.h``) and ``fcu_simbench_micro`` (per-operation cycle counts at boot),
runs each ELF in PlatformIO's ``tool-simavr`` (ATmega128 core, 16 MHz, cycle-accurate for the AVR instruction set) and
parses the ASCII report lines (numbers in hex, formats in ``src/simbench.h``):

* ``SB  t=.. st=.. cm=.. cc=.. tm=.. ov=.. sf=.. ct=.. bg=.. bk=.. sn=.. il=.. ox=.. o1a=.. .. s=..`` once per
  simulated second: the time-critical part (from the navigation frame to the end of the tick's outputs, Timer1, 0.5 us)
  and its sections, the loop including the background, ticks without a valid navigation frame, the stack high-water
  mark of the real loop, the longest background job, SNAP job and ilink drain, the simbench's own work, the compare
  registers;
* ``SN  t=.. fv= fb= ft= fq= rs= fo= pw= pc= la= lc= lm= ls= bm= be= f1= fp= fn= nr= nl= at= ar=`` a quarter second
  later: the nlink - valid / bad frames, timeout ticks, NAV ticks lost, SPI resyncs / overruns; tick start -> last vane
  register write (pw, pc with the controller active), NAV tick -> last vane register write (la, lc), the FCU's share of
  the tick start's lateness after the frame (lm; ls = the simbench's own share: its bypass injection), the background's
  margin to the next exchange as on the vehicle (bm) and anchor -> background end (be); the fallback ticks' timing
  (f1 first after a frame, fp / fn the period), the NAV reset's FCU tick (nr) and the first ABORT (at, reason ar);
* ``SL  t=.. nw=.. nb=.. cx=.. sd=.. fd=.. bl=.. up=.. rp=.. sa=.. ua=.. bs=.. rn=.. rl=.. rs=.. rt=..`` half a second
  after SB: ilink bytes waiting at a tick / decoded in the time-critical part, ticks an IL_CMD could not complete within
  its budget, SNAPs replaced, flight records dropped, valid frames without NL_F_BARO_OK, IL_CMD / SNAP / IL_STAT counts,
  CMDs without an answer, bad SNAP frames, and the latency in ticks from the first tick that starts after an IL_CMD's
  last byte to the tick whose supervisor answered it (``rs``, v0.6 requires 0; ``rt`` right behind an IL_STAT);
* ``SI  t=.. ic=.. is=.. ib=.. ro=.. re=.. bx=.. fl=.. bb=.. bj=.. bt=.. ed=.. ub=.. bz=.. sz=..`` 0.75 s after SB: the
  FCU's ilink counters, D6 (background share used up, flood ticks), records dropped, CMDs lost right after a babble,
  fast / slow babble bytes;
* ``SV  t=.. st=.. ph=.. pv=.. tg=.. ps=.. ho=.. sa=.. um=.. uh=.. lt=.. sf=.. vd=.. hm=.. hc=.. tv=.. ok=.. rj=.. hp=..``
  (VTOL, 7/8 s after SB): phase now / phases seen (bit 1 << LP_PHASE_*), target [dm], motor starts, ticks in HOLD,
  POWERED ticks with the vertical controller at its ceiling, ESC command max / min in HOLD (x1000), the first LANDED tick
  and the SAFE tick after it, 'V' records dropped, the synthetic plant's max / current height (mm) and hardest ground
  contact (mm/s), the synthetic ground's commands executed / rejected and ONs executed;
* ``SBM name=cycles`` per-operation micro-benchmarks (32 calls averaged; ``main_*`` single timed calls).

The synthetic NAV (simbench.h) runs its own 250 Hz tick and completes the frame of tick k ``SB_NAV_FRAME_US`` after it
(push at ``SB_NAV_PUSH_US`` + the ``NLINK_XFER_US`` exchange), up to ``SB_NAV_JITTER_US`` earlier; it switches pad /
flight mode on the FCU frame it gets back and injects a broken frame, a missing frame, five missing frames and a NAV
reset in flight (-> I9 ABORT after ONB_IMU_FAIL_TICKS).  The synthetic COM sends IL_CMD at the ground's poll rate (ARM,
LAUNCH with the nonce from the FCU's SNAPs), IL_STAT behind / in front of CMDs, and babbles twice in flight: the v0.6
phase-3 sub-flood pattern C5 5C 08 01 every 300 us (R1) and the full-rate flood (D6).  The bypasses have no interrupt:
the COM spends the RX ISR's cycles (SB_ISR_CYCLES, checked against the ELF) where it injects each byte.

Interrupt load on the time-critical part (not in its measured figure): every ISR of the fcu ELF is counted from its
disassembly (avr-objdump; 4 cycles response + 3 for the vector JMP + the path through the ISR), then
  ilink RX   x the largest burst a working COM sends (IL_CMD + IL_STAT back to back, 56 B, D4),
  log UDRE   x the log bytes that can leave during the part (one per 10 / FC_LOG_BAUD s),
are added to the measured maximum.  The ilink UDRE is held off during the part; the nlink slave interrupt runs during
the exchange, i.e. while the FCU waits for the frame (the bench checks that the background has ended by then), so no
exchange overlaps the part as long as part + NLINK_XFER_US + the NAV's early margin < one tick (checked).  Every FCU
interrupt that can run during an exchange must stay under the ~58 cycles the protocol agent measured as harmless for the
slave's return bytes (hal/nlink_spi.h Timing): checked from the disassembly; there must be no timer interrupt on the FCU.

    .venv/Scripts/python.exe firmware/atmega128_fc/tools/run_simbench.py

Numbers are a measurement of the code, not of the board; the firmware logs the same loop time on the vehicle.

v0.8 (VTOL with GPS position hold; main.c / simbench.h "v0.8"): the synthetic NAV sends nlink v3 frames (the GPS slot of a
synthetic snapshot: no snapshot for its first second, the ON gate's flags once its pad alignment is done) pushed at the
v0.8 NAV's measured compute end (``SB_NAV_PUSH_US_VTOL``); the synthetic COM reads SNAP3.  The report lines are streamed
into a 128-byte console ring, so a kind's fields come in several lines with the same prefix and the same ``t=`` (merged
here per window); a new ``SP`` line reports the position hold: published mode / requests, engaged ticks, modes seen, the
ticks whose position work (core/onboard.h onb_stack_bg pieces + attitude_ctrl_set_tilt) had to be finished at the next
tick's start before its attitude controller (``pl``), the longest position job, the longest such late finish, the end of a
position job in the exchange window, 'G' records dropped, slot sets / broken cycles, the first tick with the GPS ON gate,
the largest tilt setpoint.  ``--profile nogps`` runs the VTOL build without position hold (``fcu_simbench_nogps``).

v0.8.1 (the four-chip set; main.c "v0.8.1"): the FCU runs the GNS chain (core/onboard.h ONB_CHAIN_GNS: the nlink v4 GNS
slot receiver, no position controller); the synthetic NAV sends nlink v4 frames whose GNS slot comes from a synthetic GNS
(simbench.c: a RESULT per 50 Hz cycle through the NAV's own slot transmitter, TILT engaged while the motor runs, a 4-cycle
dropout at 15 s: the TRIM fallback without V11) and decodes the FCU's nlink v4 replies; it pushes at the v0.8.1 NAV's
measured compute end (SB_NAV_PUSH_US_VTOL).  The tilt work (the builder, attitude_ctrl_set_tilt and its finish) runs
right after the time-critical part (main.c pos_work_now: no late finish), the other background jobs get a second pass
into the exchange window up to the earliest next frame (main.c g_late, LATE_END_US; the "le" figure must stay before it).
New figures: SP ``xt`` (TILT parts), ``xf`` (POWERED ticks in the TRIM fallback), ``xr`` / ``xd`` (the synthetic GNS's
RESULTs / dropout cycles); SV ``lr`` / ``lk`` (the supervisor's reason / tick at the first LAND), ``wo`` / ``ws`` / ``wn``
(POWERED ticks: the FCU's own work per tick - tick start -> background end minus the bench's own time - max / sum / count);
SL ``le``.  The bench's synthetic peers spend ~0.4-1.0 ms of each simulated tick themselves (SB ``ox``): when the FCU's
own work comes close to the tick, that overhead pushes the simulated loop over it, which the real chip would not see -
``own_work`` separates the two.

v0.9 (fw09 WP-B): the checks that measure the bench's own load (``SELF_LOAD_CHECKS``: NAV tick -> vanes, IMU -> vanes,
the background's end / keeping up, record drops, the babble loop, the TRIM fallback, the nlink frame counts, the HOLD
micro-benchmark, the background reserves) move their PASS / FAIL to the row of the closed-loop four-core co-simulation
(``tools/run_cosim4.py``) that judges the same quantity, once its complete report (``host_test/build/cosim4_report.json``)
was made from these sources (equal ``source_digest``): the check becomes ``cosim_row<N>`` with the co-simulation's verdict
and the bench's figure is kept under ``bench_self_load`` with the bench's own work per tick.  ``--rejudge`` re-applies that
to the VTOL reports already written (``tools/run_all_benches.py`` does after the co-simulation).  The power profiles
(``wired`` / ``wired_buffer``) expect the commands their tethered altitude limits allow (``power_profile_cmds``).
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import subprocess
import sys
import time

FW = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ROOT = os.path.normpath(os.path.join(FW, "..", ".."))
OUT_DIR = os.path.join(FW, "host_test", "build")
# envs per mode: the plain envs fly params.json's mission.mode (vtol), the *_ballistic envs pin the ballistic mission
MODE_ENVS = {"vtol": {"bench": "fcu_simbench", "micro": "fcu_simbench_micro", "flight": "fcu", "cosim": "fcu_cosim"},
             "vtol_demo": {"bench": "fcu_simbench_demo", "flight": "fcu_demo"},
             "vtol_nogps": {"bench": "fcu_simbench_nogps", "flight": "fcu_nogps"},
             # v0.9 power builds (fw09 scratch): the indoor tethered FCU without the GPS
             "vtol_wired": {"bench": "fcu_simbench_wired", "flight": "fcu_wired"},
             "vtol_wired_buffer": {"bench": "fcu_simbench_wired_buffer", "flight": "fcu_wired_buffer"},
             "ballistic": {"bench": "fcu_simbench_ballistic", "micro": "fcu_simbench_micro_ballistic",
                           "flight": "fcu_ballistic", "cosim": "fcu_cosim_ballistic"}}
SECTION_NAMES = ["NAV frame (take + nl_decode_nav), arm switch",
                 "ilink RX (bytes waiting at the tick, only those an IL_CMD can complete in)",
                 "onboard supervisor (commands, interlocks, sensor booleans from the frame)", "attitude controller + mixer",
                 "vane pulses written (end of the frame -> PWM path)",
                 "ESC / release written, anchor, battery, records, reply snapshot, NAV reply, LED, loop stamp",
                 "background (SNAP frame, ilink decode, records, log packing, stack scan, status)"]
# v0.7.1 VTOL tick order (main.c, simbench.h SB_MARK): the attitude controller and the vanes before the FCU stack;
# v0.8: the position work a previous tick left is finished in front of the attitude controller (its section)
SECTION_NAMES_VTOL = ["NAV frame (take + nl_decode_nav4), arm switch, sensor booleans",
                      "late position work (v0.8) + attitude controller + mixer (the previous tick's control_active / ESC "
                      "throttle / tilt setpoint)",
                      "vane pulses written (end of the frame -> PWM path)",
                      "ilink RX (bytes waiting at the tick, only those an IL_CMD can complete in)",
                      "FCU stack rt: GNS slot receiver + supervisor (commands, VTOL rules) + vertical controller v2 + "
                      "the nlink v4 FCU fields (onb_stack_rt)",
                      "ESC / release written, anchor, battery, records, reply snapshot, NAV reply (thr_q), LED, loop stamp",
                      "background (position pieces, SNAP3 frame, ilink decode, records incl. 'V' / 'G', log packing, "
                      "stack scan, status)"]
VTOL_SECTION_STACK = 4                    # index of the FCU stack section in SECTION_NAMES_VTOL
FC_COMPUTE_DELAY_US_6DOF = 4710           # sim/fc_timing.py FC_COMPUTE_DELAY_S_MEASURED (checked against it in bench())
VC_TD_NAMES = {0: "none", 1: "still", 2: "support", 3: "floor", 4: "all"}   # core/vertical_ctrl.h VC_TD_*
# v0.6 phase 3 (two chips, sensors + estimator on the FCU; host_test/build/simbench_report.json of that run)
V06 = {"crit_max_us": 2971, "crit_plus_all_isr_us": 3308.5, "flight_total_max_us": 3853, "flight_stack_free_bytes": 1365,
       "flight_flash_bytes": 39652, "flight_static_ram_bytes": 2313, "bg_reserve_us": 420}
# v0.6.3 (the v0.6.3 source snapshot's own run_simbench.py, run 2026-10-20 in the v0.7 integration session with the same
# simavr / avr-gcc; fcu_simbench_report of that run)
V063 = {"crit_max_us": 1944, "crit_plus_all_isr_us": 2157.0, "flight_total_max_us": 3038, "nav_tick_to_vanes_us": 3931,
        "stack_used_bytes": 370, "flight_stack_free_bytes": 1309, "flight_flash_bytes": 27956,
        "flight_static_ram_bytes": 2417, "supervisor_section_us": 237, "micro_stack_never_touched_bytes": 70}
STATES = ["SAFE", "ARMED", "POWERED", "UNPOWERED", "RELEASED", "ABORT"]
PHASES = ["NONE", "SPOOL", "CLIMB", "HOLD", "LAND", "LANDED"]                  # src/core/link_proto.h LP_PHASE_*
BG_KINDS = {1: "SNAP frame", 2: "ilink drain", 3: "'N' record", 4: "event/link record", 5: "flight-record part",
            6: "IL_STAT 'C' record", 7: "COM text record", 8: "NAV text record", 9: "'V' record",
            10: "tilt work: builder / attitude_ctrl_set_tilt / finish (v0.8.1 pos_work_now)", 11: "'G' record"}  # main.c BG_*
CRIT_LIMIT = 0.80                         # time-critical part <= 80 % of the tick
STACK_MIN_FREE = 1024                     # bytes of never-touched stack the flight build must keep
RAM_BYTES = 4096                          # ATmega128 SRAM
ISR_EXCHANGE_LIMIT = 58                   # cycles: a competing interrupt this long left every slave return byte on time
LP_R_I9_IMU = 10                          # src/core/link_proto.h
SB_ALIAS = {"cm": "crit_max_us", "cc": "crit_ctrl_max_us", "tm": "total_max_us", "ov": "no_frame_ticks", "sf": "stack_free",
            "ct": "ctrl_ticks", "bg": "bg_step_max_us", "bk": "bg_step_max_kind", "sn": "snap_step_max_us",
            "il": "ilink_drain_max_us", "ox": "simbench_overhead_max_us", "o1a": "ocr1a", "o3a": "ocr3a", "o3b": "ocr3b",
            "o3c": "ocr3c"}
SN_ALIAS = {"fv": "nl_frames", "fb": "nl_bad", "ft": "nl_timeouts", "fq": "nl_seq_lost", "rs": "nl_resyncs",
            "fo": "nl_overruns", "pw": "pwm_us_max", "pc": "pwm_ctrl_us_max", "la": "nav_to_pwm_us_max",
            "lc": "nav_to_pwm_ctrl_us_max", "lm": "start_late_fcu_us_max", "ls": "start_late_simbench_us_max",
            "bm": "bg_margin_us_min", "be": "bg_end_us_max", "f1": "fallback_first_us_max", "fp": "fallback_period_us_max",
            "fn": "fallback_period_us_min", "nr": "nav_reset_tick", "nl": "nav_built_late", "at": "abort_tick",
            "ar": "abort_reason"}
SL_ALIAS = {"nw": "ilink_wait_max_bytes", "nb": "ilink_crit_max_bytes", "cx": "ilink_crit_over", "sd": "snap_dropped",
            "fd": "flight_records_dropped", "bl": "baro_not_ok_frames", "bn": "bg_end_no_isr_emulation_us_max",
            "je": "bg_job_end_us_max", "le": "late_job_end_us_max", "up": "il_cmd_sent", "rp": "snap_received",
            "sa": "il_stat_sent", "ua": "cmd_unanswered", "bs": "bad_snap_or_wire_full", "rn": "latency_min_ticks",
            "rl": "latency_queued_max_ticks", "rs": "latency_max_ticks", "rt": "latency_after_stat_max_ticks"}
SI_ALIAS = {"ic": "il_cmds", "is": "il_snaps", "ib": "il_bad", "ro": "rx_overflow", "re": "rx_errors", "bx": "bg_capped_ticks",
            "fl": "flood_ticks", "bb": "bg_bytes_max", "bj": "bg_job_max_us", "bt": "text_job_max_us", "ed": "evq_dropped",
            "br": "rec_job_max_us", "sr": "status_written", "tq": "thr_q_bad_frames",
            "ub": "cmd_unanswered_after_babble", "bz": "babble_bytes", "sz": "slow_babble_bytes"}
SV_ALIAS = {"ph": "phase", "pv": "phases_seen", "tg": "target_dm", "ps": "motor_starts", "ho": "hold_ticks",
            "sa": "sat_ticks", "um": "u_max_milli", "uh": "u_hold_min_milli", "lt": "landed_tick", "sf": "safe_tick",
            "vd": "vrec_dropped", "hm": "plant_h_max_mm", "hc": "plant_h_mm", "tv": "plant_touchdown_mmps",
            "ok": "cmds_executed", "rj": "cmds_rejected", "hp": "hops", "tx": "thr_q_max", "tb": "td_by_first",
            "lr": "land_reason_first", "lk": "land_tick_first", "wo": "own_work_max_us", "ws": "own_work_sum_us",
            "wn": "own_work_ticks", "sk": "cmds_skipped"}
SIGNED16 = {"rn", "rl", "rs", "rt", "bm"}

# ---- interrupt model (see the module docstring) ----
ISR_ENTRY_CYCLES = 7                      # 4 interrupt response + 3 vector-table JMP (ATmega128, 16-bit PC)
VECTORS = {"nlink_spi": 17, "log_udre": 19, "ilink_rx": 30, "ilink_udre": 31, "tick_timer2": 9, "timer0": 15,
           "twi": 33, "adc": 21}
EXCHANGE_ISRS = ("log_udre", "ilink_rx", "ilink_udre")   # can run while the NAV's exchange is in progress
# fw09 review F2: the v0.9 power builds' ADC chain interrupt (hal/adc.c; 1 conversion a tick in wired, 3 in wired_buffer)
# can meet an exchange too (a resuming NAV's first exchange on the FCU's own clock), so it is held to the same limit and
# counted in the time-critical part (every conversion of the chain started in step 5); absent from the other builds
PM_ADC_CHAIN = {"wired": 1, "wired_buffer": 3}
ABSENT_ON_FCU = ("tick_timer2", "timer0", "twi")
ILINK_BURST_BYTES = 6 + 8 + 6 + 36        # IL_CMD + IL_STAT frames back to back: a working COM's largest burst (D4)
CYCLES = {**{m: 1 for m in ("add adc sub subi sbc sbci and andi or ori eor com neg sbr cbr inc dec tst clr ser mov movw "
                             "ldi in out lsl lsr rol ror asr swap bst bld sec clc sen cln sez clz sei cli ses cls sev clv "
                             "set clt seh clh nop cp cpc cpi wdr sleep").split()},
          **{m: 2 for m in "adiw sbiw mul muls mulsu fmul fmuls fmulsu push pop lds sts ld st ldd std sbi cbi".split()},
          "lpm": 3, "elpm": 3}
SKIPS = {"cpse", "sbrc", "sbrs", "sbic", "sbis"}


def simavr_exe() -> str:
    home = os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")
    exe = os.path.join(home, "packages", "tool-simavr", "bin", "simavr" + (".exe" if os.name == "nt" else ""))
    if not os.path.exists(exe):
        raise SystemExit(f"simavr not found at {exe}: python -m platformio pkg install -g --tool platformio/tool-simavr")
    return exe


def objdump_exe() -> str:
    home = os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")
    exe = os.path.join(home, "packages", "toolchain-atmelavr", "bin", "avr-objdump" + (".exe" if os.name == "nt" else ""))
    if not os.path.exists(exe):
        raise SystemExit(f"avr-objdump not found at {exe} (PlatformIO toolchain-atmelavr)")
    return exe


STATIC_RAM: dict[str, int] = {}
FLASH: dict[str, int] = {}
WARNINGS: dict[str, list[str]] = {}


def build(env: str) -> str:
    r = subprocess.run([sys.executable, "-m", "platformio", "run", "-d", FW, "-e", env], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        raise SystemExit(f"build {env} failed:\n{r.stdout[-3000:]}\n{r.stderr[-2000:]}")
    m = re.search(r"RAM:.*used (\d+) bytes", r.stdout)
    if m:
        STATIC_RAM[env] = int(m.group(1))
    m = re.search(r"Flash:.*used (\d+) bytes", r.stdout)
    if m:
        FLASH[env] = int(m.group(1))
    WARNINGS[env] = sorted({w for w in re.findall(r"warning: (.*)", r.stdout + r.stderr)})
    return os.path.join(FW, ".pio", "build", env, "firmware.elf")


def run(elf: str, timeout_s: float = 300.0) -> str:
    """simavr's console (stdout, then stderr).  v0.8: a crashed core ("avr_sadly_crashed": simavr then waits for gdb
    forever) is stopped at once and its output returned (the caller judges it); a run past timeout_s still raises."""
    import threading
    proc = subprocess.Popen([simavr_exe(), "-m", "atmega128", "-f", "16000000", elf], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, stdin=subprocess.DEVNULL)
    out: list[bytes] = []
    err: list[bytes] = []
    crashed = threading.Event()

    def reader(pipe, acc: list[bytes]) -> None:
        for line in iter(pipe.readline, b""):
            acc.append(line)
            if b"avr_sadly_crashed" in line:
                crashed.set()
    ts = [threading.Thread(target=reader, args=(proc.stdout, out), daemon=True),
          threading.Thread(target=reader, args=(proc.stderr, err), daemon=True)]
    for t in ts:
        t.start()
    t0 = time.monotonic()
    timed_out = False
    while proc.poll() is None:
        if crashed.is_set():
            proc.kill()
            break
        if time.monotonic() - t0 > timeout_s:
            timed_out = True
            proc.kill()
            break
        time.sleep(0.1)
    proc.wait()
    for t in ts:
        t.join(timeout=5.0)
    text = (b"".join(out) + b"".join(err)).decode("latin-1")
    if timed_out:
        raise subprocess.TimeoutExpired(proc.args, timeout_s, output=text)
    return text


def _kv(line: str, alias: dict[str, str]) -> dict:
    row = {}
    for k, v in re.findall(r"(\w+)=(0x[0-9a-f]+)", line):
        if k == "s":
            continue
        x = int(v, 16)
        if k in SIGNED16 and x >= 0x8000:
            x -= 0x10000
        row[alias.get(k, k)] = x
    return row


SP_ALIAS = {"pm": "pos_mode", "pq": "pos_requests_seen", "pe": "pos_engaged_ticks", "pp": "pos_modes_seen",
            "pl": "pos_late_ticks", "pb": "pos_job_max_us", "pf": "pos_late_finish_max_us", "pz": "pos_job_end_us_max",
            "gd": "grec_dropped", "xs": "slot_sets", "xb": "slot_broken", "gg": "gps_gate_ok_tick", "tz": "tilt_max_mrad",
            "xt": "slot_tilts", "xf": "fallback_ticks", "xr": "gns_results", "xd": "gns_dropout_cycles"}


def _merged(text: str, prefix: str, alias: dict) -> list[dict]:
    """the rows of one line kind; v0.8: a kind's fields come in several lines with the same prefix and the same t=
    (simbench.c report_line: < 128-byte lines) - merged into one row per window, in order"""
    rows: list[dict] = []
    for m in re.finditer(rf"{prefix} (t=[ -~]*)", text):
        line = m.group(1)
        row = _kv(line, alias)
        secs = re.search(r"(?<![a-z0-9])s=((?:0x[0-9a-f]+ ?)+)", line)
        if secs:
            row["sections_us"] = [int(x, 16) for x in secs.group(1).split()]
        if rows and rows[-1].get("t") == row.get("t"):
            rows[-1].update(row)
        else:
            rows.append(row)
    return rows


def parse_sv(text: str) -> list[dict]:
    return _merged(text, "SV", SV_ALIAS)


def parse_sp(text: str) -> list[dict]:
    return _merged(text, "SP", SP_ALIAS)


def parse(text: str) -> tuple[list[dict], list[dict], list[dict], list[dict]]:
    sb = _merged(text, "SB", SB_ALIAS)
    for row in sb:
        row.setdefault("sections_us", [])
    return sb, _merged(text, "SN", SN_ALIAS), _merged(text, "SL", SL_ALIAS), _merged(text, "SI", SI_ALIAS)


def parse_sbm(text: str) -> dict[str, int]:
    return {k: int(v, 0) for k, v in re.findall(r"SBM (\w+)=(0x[0-9a-f]+|\d+)", text)}


def params() -> dict:
    try:
        with open(os.path.join(ROOT, "params.json"), "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def source_defines(path: str, names: tuple[str, ...]) -> dict[str, int]:
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()
    out = {}
    for n in names:
        m = re.search(rf"#define\s+{n}\s+\(?(\d+)[uUlL]*\b", src)
        if not m:
            raise SystemExit(f"{os.path.relpath(path, ROOT)}: #define {n} not found")
        out[n] = int(m.group(1))
    return out


# ---------------------------------------------------------------- ISR cycles from the disassembly
def disassemble(elf: str) -> dict[str, list[tuple[int, int, str, str]]]:
    """function name -> [(address, size in bytes, mnemonic, operands incl. objdump's target comment)]"""
    r = subprocess.run([objdump_exe(), "-d", elf], capture_output=True, text=True, encoding="utf-8", errors="replace")
    funcs: dict[str, list[tuple[int, int, str, str]]] = {}
    cur = None
    for line in r.stdout.splitlines():
        m = re.match(r"^[0-9a-f]+ <(\S+)>:$", line)
        if m:
            cur = funcs.setdefault(m.group(1), [])
            continue
        m = re.match(r"^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )+)\s*\t(\S+)\s*(.*)$", line)
        if m and cur is not None:
            cur.append((int(m.group(1), 16), len(m.group(2).split()), m.group(3), m.group(4)))
    return funcs


def _target(ops: str) -> int:
    m = re.search(r";\s*0x([0-9a-f]+)", ops)
    if not m:
        raise ValueError(f"no branch target in {ops!r}")
    return int(m.group(1), 16)


def isr_cycles(ins: list[tuple[int, int, str, str]]) -> dict:
    """{'longest': cycles of the longest path to RETI, 'normal': the path with every conditional branch / skip NOT
    taken} - both including ISR_ENTRY_CYCLES; 'normal' is None if that path does not end in RETI.  Raises on a call or
    an indirect jump (the callee's time is unknown here) and on a loop."""
    idx = {a: i for i, (a, _, _, _) in enumerate(ins)}
    memo: dict[int, int] = {}

    def step(i: int, active: frozenset) -> int:
        if i in memo:
            return memo[i]
        if i in active:
            raise ValueError("loop in ISR")
        a, size, mn, ops = ins[i]
        act = active | {i}
        if mn in ("reti", "ret"):
            c = 4
        elif mn == "rjmp":
            c = 2 + step(idx[_target(ops)], act)
        elif mn.startswith("br"):
            c = max(1 + step(i + 1, act), 2 + step(idx[_target(ops)], act))
        elif mn in SKIPS:
            c = max(1 + step(i + 1, act), (2 if ins[i + 1][1] == 2 else 3) + step(i + 2, act))
        elif mn in CYCLES:
            c = CYCLES[mn] + step(i + 1, act)
        else:
            raise ValueError(f"unsupported instruction in ISR: {mn} {ops}")
        memo[i] = c
        return c

    longest = ISR_ENTRY_CYCLES + step(0, frozenset())
    i, normal = 0, ISR_ENTRY_CYCLES
    for _ in range(len(ins) + 1):
        a, size, mn, ops = ins[i]
        if mn in ("reti", "ret"):
            normal += 4
            break
        if mn == "rjmp":
            normal += 2
            i = idx[_target(ops)]
        elif mn.startswith("br") or mn in SKIPS:
            normal += 1
            i += 1
        else:
            normal += CYCLES[mn]
            i += 1
    else:
        normal = None
    return {"longest": longest, "normal": normal, "instructions": len(ins)}


def isr_table(elf: str) -> dict:
    funcs = disassemble(elf)
    out = {}
    for name, vec in VECTORS.items():
        ins = funcs.get(f"__vector_{vec}")
        if not ins:
            out[name] = {"vector": vec, "error": "not in the ELF"}
            continue
        try:
            out[name] = {"vector": vec, **isr_cycles(ins)}
        except (ValueError, KeyError) as e:
            out[name] = {"vector": vec, "error": str(e)}
    return out


# ---------------------------------------------------------------- the bench
def power_profile_cmds() -> int:
    """v0.9 (fw09 WP-B): the commands the VTOL profile's operator gets executed in a power (tether) build when its synthetic
    ground respects the tethered altitude limits (src/simbench.c: UP / DOWN only when the new target stays inside
    [ONB_W_ALT_MIN_DM, ONB_W_ALT_MAX_DM]; the field profile's step): ON, OFF, ON2 + the UP / DOWN that fit"""
    with open(os.path.join(FW, "src", "params_gen.h"), "r", encoding="utf-8") as f:
        gen = f.read()

    def first(name: str) -> int:                     # the first definition = the field profile's section
        m = re.search(rf"#define\s+{name}\s+(\d+)", gen)
        if not m:
            raise SystemExit(f"params_gen.h: {name} not found")
        return int(m.group(1))
    hov, amin, amax, step = (first(n) for n in ("ONB_W_HOVER_DM", "ONB_W_ALT_MIN_DM", "ONB_W_ALT_MAX_DM", "ONB_V_ALT_STEP_DM"))
    up = hov + step <= amax
    tgt = hov + (step if up else 0)
    down = amin <= tgt - step
    return 3 + int(up) + int(down)


def bench(mode: str, tick_us: float, loop_hz: int, log_baud: float, consts: dict, profile: str = "field") -> dict:
    envs = MODE_ENVS[f"vtol_{profile}" if (mode == "vtol" and profile in ("demo", "nogps", "wired", "wired_buffer")) else mode]
    ENV, FLIGHT_ENV = envs["bench"], envs["flight"]
    vtol = mode == "vtol"
    gps = vtol and profile not in ("nogps", "wired", "wired_buffer")
    section_names = SECTION_NAMES_VTOL if vtol else SECTION_NAMES
    elf = build(ENV)
    text = run(elf)
    rows, navs, links, ils = parse(text)
    svs = parse_sv(text) if vtol else []
    sps = parse_sp(text) if gps else []
    if not rows or "SB END" not in text:
        print(text[-2000:])
        raise SystemExit(f"{ENV}: simbench produced no report (did the firmware reach the loop?)")
    flight = [r for r in rows if r.get("st") in (2, 3)]
    pad = [r for r in rows if r.get("st") in (0, 1)]
    lflight = [r for r in links if r.get("st") in (2, 3)]
    bab0 = consts["SB_BABBLE_S"] * loop_hz
    bab1 = bab0 + consts["SB_BABBLE_TICKS"]
    sbab0 = consts["SB_SLOW_BABBLE_S"] * loop_hz
    sbab1 = sbab0 + consts["SB_SLOW_BABBLE_TICKS"]
    babble = [r for r in rows if bab0 < r["t"] <= bab1 + loop_hz]          # 1 s windows holding babble ticks
    slow = [r for r in rows if sbab0 < r["t"] <= sbab1 + loop_hz]
    calm_flight = [r for r in flight if r not in babble and r not in slow]

    def agg(rs: list[dict], key: str, fn=max, default=0):
        return fn((r[key] for r in rs if key in r), default=default)

    def ocr_us(rs: list[dict], key: str) -> list[float]:
        return sorted({r[key] / 2.0 for r in rs if key in r})

    n_sec = len(section_names)
    sections_flight = [max((r["sections_us"][i] for r in flight if len(r["sections_us"]) > i), default=0)
                       for i in range(n_sec)]
    sections_all = [max((r["sections_us"][i] for r in rows if len(r["sections_us"]) > i), default=0) for i in range(n_sec)]
    last = links[-1] if links else {}
    lasti = ils[-1] if ils else {}
    lastn = navs[-1] if navs else {}
    firstn = navs[0] if navs else {}
    lat = [r["latency_max_ticks"] for r in links if "latency_max_ticks" in r]
    lat_stat = [r["latency_after_stat_max_ticks"] for r in links if "latency_after_stat_max_ticks" in r]
    lat_min = [r["latency_min_ticks"] for r in links if "latency_min_ticks" in r]
    lat_q = [r["latency_queued_max_ticks"] for r in links if "latency_queued_max_ticks" in r]
    s = {
        "env": ENV, "mode": mode, "profile": profile if vtol else None, "windows": len(rows), "tick_us": tick_us,
        "states": [STATES[r["st"]] if r.get("st", 99) < 6 else r.get("st") for r in rows],
        "pad_crit_max_us": agg(pad, "crit_max_us"), "flight_crit_max_us": agg(flight, "crit_max_us"),
        "crit_max_us": agg(rows, "crit_max_us"),
        "flight_total_max_us": agg(flight, "total_max_us"), "pad_total_max_us": agg(pad, "total_max_us"),
        "no_frame_ticks": rows[-1].get("no_frame_ticks", 0), "stack_free_min_bytes": min(r.get("stack_free", 0) for r in rows),
        "bg_step_max_us": agg(rows, "bg_step_max_us"), "snap_step_max_us": agg(rows, "snap_step_max_us"),
        "ilink_drain_max_us": agg(rows, "ilink_drain_max_us"),
        "simbench_overhead_max_us": agg(rows, "simbench_overhead_max_us"),
        "bg_step_max_kind": BG_KINDS.get(max(rows, key=lambda r: r.get("bg_step_max_us", 0)).get("bg_step_max_kind")),
        "control_ticks": rows[-1].get("ctrl_ticks", 0),
        "flight_sections_max_us": dict(zip(section_names, sections_flight)),
        "sections_max_us": dict(zip(section_names, sections_all)),
        "nlink": {
            "frames": lastn.get("nl_frames"), "bad": lastn.get("nl_bad"), "timeouts": lastn.get("nl_timeouts"),
            "timeouts_at_boot": firstn.get("nl_timeouts"), "seq_lost": lastn.get("nl_seq_lost"),
            "spi_resyncs": lastn.get("nl_resyncs"), "spi_overruns": lastn.get("nl_overruns"),
            "built_late": lastn.get("nav_built_late"),
            "tick_start_to_vanes_us_max": agg(navs, "pwm_us_max"),
            "tick_start_to_vanes_ctrl_us_max": agg(navs, "pwm_ctrl_us_max"),
            "nav_tick_to_vanes_us_max": agg(navs, "nav_to_pwm_us_max"),
            "nav_tick_to_vanes_ctrl_us_max": agg(navs, "nav_to_pwm_ctrl_us_max"),
            "start_late_fcu_us_max": agg(navs, "start_late_fcu_us_max"),
            "start_late_simbench_us_max": agg(navs, "start_late_simbench_us_max"),
            "bg_margin_us_min": agg(navs, "bg_margin_us_min", fn=min, default=None),
            "bg_end_us_max": agg(navs, "bg_end_us_max"),
            "bg_end_no_isr_emulation_us_max": agg(links, "bg_end_no_isr_emulation_us_max"),
            "bg_job_end_us_max": agg(links, "bg_job_end_us_max"),
            "fallback_first_us_max": lastn.get("fallback_first_us_max"),
            "fallback_period_us_max": lastn.get("fallback_period_us_max"),
            "fallback_period_us_min": lastn.get("fallback_period_us_min"),
            "nav_reset_tick": lastn.get("nav_reset_tick"), "abort_tick": lastn.get("abort_tick"),
            "abort_reason": lastn.get("abort_reason"),
            "_doc": "single-chip bypass: the synthetic NAV completes the frame of NAV tick k SB_NAV_FRAME_US after it (up to "
                    "SB_NAV_JITTER_US earlier); nav_tick_to_vanes = that + the FCU's share of the tick start's lateness + "
                    "tick start -> last vane register write (Timer1); the simbench's own share (bypass injection) is "
                    "excluded; bg_margin = next exchange start - (frame + FCU lateness + anchor -> background end)",
        },
        "ilink": {
            "il_cmd_sent": last.get("il_cmd_sent", 0), "snap_received": last.get("snap_received", 0),
            "il_stat_sent": last.get("il_stat_sent", 0), "cmd_unanswered": last.get("cmd_unanswered", 0),
            "bad_snap_or_wire_full": last.get("bad_snap_or_wire_full", 0), "snap_dropped": last.get("snap_dropped", 0),
            "crit_over": last.get("ilink_crit_over", 0),
            "wait_at_tick_max_bytes": agg(links, "ilink_wait_max_bytes"),
            "crit_decoded_max_bytes": agg(links, "ilink_crit_max_bytes"),
            "flight_crit_decoded_max_bytes": agg(lflight, "ilink_crit_max_bytes"),
            "latency_max_ticks": max(lat, default=None), "latency_min_ticks": min(lat_min, default=None),
            "latency_after_stat_max_ticks": max(lat_stat, default=None),
            "latency_queued_max_ticks": max(lat_q, default=None),
            "fcu_il_cmds": lasti.get("il_cmds"), "fcu_il_snaps": lasti.get("il_snaps"), "fcu_il_bad": lasti.get("il_bad"),
            "rx_overflow": lasti.get("rx_overflow"), "rx_errors": lasti.get("rx_errors"),
            "bg_capped_ticks": lasti.get("bg_capped_ticks"), "bg_bytes_max": agg(ils, "bg_bytes_max"),
            "_doc": "latency: ticks from the first tick that starts after an IL_CMD's last byte to the tick whose "
                    "supervisor answered it (SNAP.tick); 'after_stat' = the CMD came right behind an IL_STAT; 'queued' = "
                    "to the tick the SNAP was in the ilink TX ring",
        },
        "babble": {
            "ticks": [bab0, bab1], "bytes_sent": lasti.get("babble_bytes"), "flood_ticks": lasti.get("flood_ticks"),
            "rx_overflow": lasti.get("rx_overflow"), "cmd_unanswered_after": lasti.get("cmd_unanswered_after_babble"),
            "crit_max_us": agg(babble, "crit_max_us"), "total_max_us": agg(babble, "total_max_us"),
            "calm_flight_total_max_us": agg(calm_flight, "total_max_us", default=None),
            "_doc": "the synthetic COM floods the ilink at ILINK_BAUD (C5 5C 31 02 sync spam and noise, 100 B a tick); "
                    "windows = the one-second report windows holding babble ticks",
        },
        "slow_babble": {
            "ticks": [sbab0, sbab1], "period_us": consts["SB_SLOW_BABBLE_US"], "bytes_sent": lasti.get("slow_babble_bytes"),
            "crit_max_us": agg(slow, "crit_max_us"), "total_max_us": agg(slow, "total_max_us"),
            "_doc": "R1: the v0.6 phase-3 sub-flood pattern C5 5C 08 01 (an IL_CMD header) one byte every period; the "
                    "flight-record packing must not starve (flight_records_dropped stays 0 over the whole run)",
        },
        "flight_records_dropped": last.get("flight_records_dropped", 0),
        "flight_records_dropped_by_window": [
            {"t_s": round(b["t"] / loop_hz, 2), "state": STATES[b["st"]] if b.get("st", 99) < 6 else b.get("st"),
             "dropped": b.get("flight_records_dropped", 0) - a.get("flight_records_dropped", 0)}
            for a, b in zip([{}] + links[:-1], links) if b.get("flight_records_dropped", 0) - a.get("flight_records_dropped", 0)],
        "status_records_written": lasti.get("status_written"),
        "baro_not_ok_frames": last.get("baro_not_ok_frames", 0), "evq_dropped": lasti.get("evq_dropped", 0),
        "outputs_us": {"pad": {k: ocr_us(pad, k) for k in ("ocr1a", "ocr3a", "ocr3b", "ocr3c")},
                       "flight": {k: ocr_us(flight, k) for k in ("ocr1a", "ocr3a", "ocr3b", "ocr3c")},
                       "_doc": "pulse widths read back from OCR1A (vane 1), OCR3A (vane 4), OCR3B (ESC), OCR3C (release), "
                               "assuming 2 timer counts per us (params_gen FC_PWM_COUNTS_PER_US)"},
        "v06_reference": V06, "v063_reference": V063,
    }
    if vtol:
        sv = svs[-1] if svs else {}
        pv = sv.get("phases_seen", 0)
        s["vtol"] = {
            "phases_seen": [PHASES[i] for i in range(len(PHASES)) if pv & (1 << i)],
            "motor_starts": sv.get("motor_starts"), "hold_s": sv.get("hold_ticks", 0) / loop_hz,
            "sat_high_s": sv.get("sat_ticks", 0) / loop_hz,
            "esc_u_max": sv.get("u_max_milli", 0) / 1000.0, "esc_u_hold_min": sv.get("u_hold_min_milli", 0) / 1000.0,
            "landed_s": sv.get("landed_tick", 0) / loop_hz, "safe_after_landed_s": sv.get("safe_tick", 0) / loop_hz,
            "plant_h_max_m": sv.get("plant_h_max_mm", 0) / 1000.0,
            "plant_touchdown_mps": sv.get("plant_touchdown_mmps", 0) / 1000.0,
            "cmds_executed": sv.get("cmds_executed"), "cmds_rejected": sv.get("cmds_rejected"), "hops": sv.get("hops"),
            "cmds_skipped": sv.get("cmds_skipped"),     # fw09 (agent B): UP / DOWN the tethered altitude window left out
            "vrec_dropped": sv.get("vrec_dropped"), "final_state": STATES[rows[-1]["st"]] if rows[-1].get("st", 99) < 6 else None,
            "thr_q_max": sv.get("thr_q_max"), "thr_q_bad_frames": lasti.get("thr_q_bad_frames"),
            "td_by_first": VC_TD_NAMES.get(sv.get("td_by_first", -1), sv.get("td_by_first")),
            "land_reason_first": sv.get("land_reason_first"), "land_tick_first": sv.get("land_tick_first"),
            "own_work_max_us": sv.get("own_work_max_us"), "own_work_ticks": sv.get("own_work_ticks"),
            "own_work_mean_us": None if not sv.get("own_work_ticks") or sv.get("own_work_sum_us") is None else
            round(sv["own_work_sum_us"] / sv["own_work_ticks"], 1),
            "_doc": "the synthetic NAV flies a vertical point mass on the FCU's own ESC command (T = T_max u^n, EDF lag, "
                    "no drag / ground effect / wind); the synthetic ground sends ON, UP, DOWN, OFF, ON with the nonce of "
                    "the latest SNAP2 / SNAP3 (src/simbench.h VTOL profile)",
        }
    if gps:
        sp = sps[-1] if sps else {}
        pm = sp.get("pos_modes_seen", 0)
        s["position_hold"] = {
            "modes_seen": [n for i, n in enumerate(("OFF", "VEL", "POS", "DR", "ATT")) if pm & (1 << i)],
            "requests_seen": sp.get("pos_requests_seen"), "engaged_s": sp.get("pos_engaged_ticks", 0) / loop_hz,
            "late_ticks": sp.get("pos_late_ticks"), "late_fraction_of_engaged":
                round(sp.get("pos_late_ticks", 0) / max(1, sp.get("pos_engaged_ticks", 0)), 3),
            "job_max_us": max((r.get("pos_job_max_us", 0) for r in sps), default=None),
            "late_finish_max_us": max((r.get("pos_late_finish_max_us", 0) for r in sps), default=None),
            "job_end_us_max": max((r.get("pos_job_end_us_max", 0) for r in sps), default=None),
            "grec_dropped": sp.get("grec_dropped"), "slot_sets": sp.get("slot_sets"), "slot_broken": sp.get("slot_broken"),
            "gps_gate_ok_s": None if not sp.get("gps_gate_ok_tick") else sp["gps_gate_ok_tick"] / loop_hz,
            "slot_tilts": sp.get("slot_tilts"), "fallback_ticks": sp.get("fallback_ticks"),
            "gns_results": sp.get("gns_results"), "gns_dropout_cycles": sp.get("gns_dropout_cycles"),
            "tilt_max_deg": round(sp.get("tilt_max_mrad", 0) * 1e-3 * 180.0 / math.pi, 2),
            "_doc": "late_ticks: ticks whose position work (onb_stack_bg pieces, the tilt builder, attitude_ctrl_set_tilt) "
                    "the background could not finish before the next frame - finished at the next tick's start before its "
                    "attitude controller (main.c pos_finish_late), inside the time-critical part (late_finish_max_us); "
                    "job_end_us_max: anchor -> the end of a position job allowed into the exchange window (must stay "
                    "before the earliest next frame, FC_TICK_US - NLINK_EARLY_US)",
        }
    s["crit_fraction"] = s["crit_max_us"] / tick_us
    # the simbench build carries extra static RAM (its own buffers); the stack DEPTH it measured is the flight loop's,
    # so the flight build keeps RAM - its static RAM - that depth (+ one ISR frame, not exercised here)
    flight_elf = build(FLIGHT_ENV)
    s["static_ram_bytes"] = STATIC_RAM.get(ENV)
    s["flight_env"] = FLIGHT_ENV
    s["flight_static_ram_bytes"] = STATIC_RAM.get(FLIGHT_ENV)
    s["flight_flash_bytes"] = FLASH.get(FLIGHT_ENV)
    s["stack_used_bytes"] = RAM_BYTES - s["static_ram_bytes"] - s["stack_free_min_bytes"]
    s["flight_stack_free_bytes"] = RAM_BYTES - s["flight_static_ram_bytes"] - s["stack_used_bytes"]

    # ---- interrupts, from the flight ELF's disassembly
    isr = isr_table(flight_elf)
    rx = isr["ilink_rx"].get("normal")
    udre = isr["log_udre"].get("normal")
    log_byte_us = 10e6 / log_baud
    n_log = math.ceil(s["crit_max_us"] / log_byte_us)
    load = {"ilink_rx": (ILINK_BURST_BYTES, rx), "log_udre": (n_log, udre)}
    adc_n = PM_ADC_CHAIN.get(profile, 0) if vtol else 0
    exchange_isrs = EXCHANGE_ISRS + (("adc",) if adc_n else ())
    if adc_n:                                    # fw09 review F2: every conversion of the chain, its longest path
        load["adc"] = (adc_n, isr["adc"].get("longest"))
    us = {k: (n * c / 16.0 if c is not None else None) for k, (n, c) in load.items()}
    spi = isr["nlink_spi"]
    s["interrupts"] = {
        "isr_cycles": isr,
        "per_crit": {k: {"count": n, "cycles_each": c, "us": None if us[k] is None else round(us[k], 1)}
                     for k, (n, c) in load.items()},
        "crit_plus_ilink_rx_us": round(s["crit_max_us"] + (us["ilink_rx"] or 0.0), 1),
        "crit_plus_all_us": round(s["crit_max_us"] + sum(v or 0.0 for v in us.values()), 1),
        "exchange_isr_longest_cycles": {k: isr[k].get("longest") for k in exchange_isrs},
        "adc_chain_in_power_build": {"conversions_per_tick": adc_n, "vector_in_flight_elf": "error" not in isr["adc"]},
        "nlink_spi_per_byte_cycles": spi.get("normal"),
        "nlink_spi_exchange_cycles": None if spi.get("normal") is None else spi["normal"] * 72,
        "_doc": "cycles = 4 response + 3 vector JMP + the ISR (ilink RX / log UDRE: straight-line normal path of the "
                "hand-written ISRs); counts per time-critical part: ilink RX = IL_CMD + IL_STAT (56 B), log UDRE = one "
                "byte per 10 / FC_LOG_BAUD s; the ilink UDRE is held off, no nlink exchange overlaps the part.  "
                "exchange_isr_longest_cycles: the longest path of every FCU interrupt that can run during an exchange "
                f"(limit {ISR_EXCHANGE_LIMIT}, hal/nlink_spi.h Timing)",
    }
    s["crit_plus_ilink_rx_fraction"] = s["interrupts"]["crit_plus_ilink_rx_us"] / tick_us
    s["crit_plus_all_isr_fraction"] = s["interrupts"]["crit_plus_all_us"] / tick_us
    s["thr_q_bad_frames"] = lasti.get("thr_q_bad_frames")
    s["bg_job_max_us"] = agg(ils, "bg_job_max_us")
    s["rec_job_max_us"] = agg(ils, "rec_job_max_us")
    s["text_job_max_us"] = agg(ils, "text_job_max_us")
    il, bb, sbb, nl = s["ilink"], s["babble"], s["slow_babble"], s["nlink"]
    bg_end_limit = consts["FC_TICK_US"] - consts["NLINK_EARLY_US"] - consts["NLINK_XFER_US"]
    s["bg_end_limit_us"] = bg_end_limit
    # the sample -> PWM path: NAV tick -> the frame at the FCU (the NAV's share, SB_NAV_FRAME_US in this bench) + the
    # FCU's (lateness + tick start -> the last vane register): the FCU's share alone, for the NAV's budget
    s["fcu_share_frame_to_vanes_us_max"] = (nl["start_late_fcu_us_max"] or 0) + (nl["tick_start_to_vanes_ctrl_us_max"] or 0)
    s["nav_budget_frame_complete_after_tick_us"] = tick_us - s["fcu_share_frame_to_vanes_us_max"]
    # v0.7.1: IMU sample -> vanes = the NAV's lead (the MPU-6050 burst starts FC_IMU_LEAD_COUNTS before its tick) + NAV
    # tick -> vanes of this bench (its synthetic NAV pushes at SB_NAV_PUSH_US: the measured NAV frame-ready time)
    s["imu_lead_us"] = consts["IMU_LEAD_US"]
    s["imu_sample_to_vanes_us_max"] = round(consts["IMU_LEAD_US"] + (nl["nav_tick_to_vanes_ctrl_us_max"] or 0), 1)
    s["imu_sample_to_vanes_limit_us"] = FC_COMPUTE_DELAY_US_6DOF
    # flight records dropped, by cause (the architect's v0.7.1 criterion: none in calm flight; the rest reported): each SL
    # window (previous SL tick, this SL tick] is classed by the injected disturbance it overlaps - the COM floods (slow /
    # fast babble), the NAV faults (broken frame, missing frame, the SB_NAV_GAP_TICKS gap: fallback ticks), the NAV reset
    # in flight and everything after it (I9 ABORT -> RELEASED) - else calm
    nav_faults = [(consts["SB_NAV_BAD_TICK"], consts["SB_NAV_BAD_TICK"] + loop_hz),
                  (consts["SB_NAV_MISS_TICK"], consts["SB_NAV_MISS_TICK"] + loop_hz),
                  (consts["SB_NAV_GAP_TICK"], consts["SB_NAV_GAP_TICK"] + consts["SB_NAV_GAP_TICKS"] + loop_hz)]
    reset_tick = nl["nav_reset_tick"] or 10 ** 9
    drops_by_cause = {"calm": 0, "com_flood": 0, "nav_fault": 0, "nav_reset_and_after": 0}
    for a, b in zip([{}] + links[:-1], links):
        d = max(0, b.get("flight_records_dropped", 0) - a.get("flight_records_dropped", 0))
        lo, hi = a.get("t", 0), b["t"]

        def hits(x0: float, x1: float) -> bool:
            return lo < x1 and hi > x0
        if not d:
            continue
        if hi > reset_tick:
            drops_by_cause["nav_reset_and_after"] += d
        elif hits(bab0, bab1) or hits(sbab0, sbab1):
            drops_by_cause["com_flood"] += d
        elif any(hits(x0, x1) for x0, x1 in nav_faults):
            drops_by_cause["nav_fault"] += d
        else:
            drops_by_cause["calm"] += d
    s["flight_records_dropped_by_cause"] = drops_by_cause
    s["flight_records_dropped_calm"] = drops_by_cause["calm"]
    nav_reset_ok = (nl["abort_tick"] is not None and nl["nav_reset_tick"] and nl["abort_reason"] == LP_R_I9_IMU and
                    nl["abort_tick"] - nl["nav_reset_tick"] in (consts["ONB_IMU_FAIL_TICKS"], consts["ONB_IMU_FAIL_TICKS"] + 1))
    if vtol:
        v = s["vtol"]
        exp_cmds = power_profile_cmds() if profile in ("wired", "wired_buffer") else 5
        v["cmds_expected"] = exp_cmds
        profile_ok = (set(PHASES[1:]) <= set(v["phases_seen"]) and v["motor_starts"] == 2 and v["hops"] == 2
                      and v["cmds_executed"] == exp_cmds and v["cmds_rejected"] == 0 and v["hold_s"] > 0
                      and v.get("cmds_skipped") == 5 - exp_cmds        # fw09 (agent B): the ground left out exactly these
                      and 0 < v["landed_s"] < v["safe_after_landed_s"] and v["final_state"] == "RELEASED")
    pos_checks = {}
    if gps:
        ph = s["position_hold"]
        pos_checks = {
            "pos_hold_engaged_in_pos_mode": "POS" in ph["modes_seen"] and ph["engaged_s"] > 0,
            # v0.8.1: the LAND requests the supervisor saw in flight - the GNS chain's requests include the FCU's own
            # GPS_LOST while no TILT part came since boot (bit 0 on the pad before the GNS's first answer): judged by the
            # LAND reason instead (no V11-V14 LAND: the first LAND, if any, is the operator's OFF)
            "no_gps_land_rule_fired": s["vtol"]["land_reason_first"] in (0, None),
            "pos_work_never_late": ph["late_ticks"] == 0,
            "pos_jobs_end_before_earliest_frame": (ph["job_end_us_max"] or 0) <= tick_us - consts["NLINK_EARLY_US"],
            "gps_gate_before_on": ph["gps_gate_ok_s"] is not None and ph["gps_gate_ok_s"] <= consts["SB_ON_S"],
            "no_grec_dropped": ph["grec_dropped"] == 0,
            "slot_sets_every_5_ticks": (ph["slot_sets"] or 0) >= 0.9 * (s["nlink"]["frames"] or 0) / 5,
            "tilt_part_every_cycle": (ph["slot_tilts"] or 0) >= (ph["slot_sets"] or 0),
            "trim_fallback_only_in_the_dropout": (ph["fallback_ticks"] or 0) <= 5 * ((ph["gns_dropout_cycles"] or 0) + 2),
        }
    exp_timeouts = consts["SB_NAV_GAP_TICKS"] + 1 + consts["SB_NAV_RESET_TICKS"]
    s["checks"] = {
        "crit_le_80pct": s["crit_fraction"] <= CRIT_LIMIT,
        "crit_plus_ilink_rx_isr_le_80pct": rx is not None and s["crit_plus_ilink_rx_fraction"] <= CRIT_LIMIT,
        "crit_plus_all_isr_le_80pct": s["crit_plus_all_isr_fraction"] <= CRIT_LIMIT,
        "nav_tick_to_vanes_le_1_tick": 0 < (nl["nav_tick_to_vanes_us_max"] or 0) <= tick_us,
        "no_exchange_overlaps_crit": s["crit_max_us"] + consts["NLINK_XFER_US"] + consts["NLINK_EARLY_US"] < tick_us,
        "bg_ends_before_next_exchange": (nl["bg_end_us_max"] or 0) <= bg_end_limit and (nl["bg_margin_us_min"] or -1) >= 0,
        "bg_jobs_end_before_next_exchange": 0 < (nl["bg_job_end_us_max"] or 0) <= bg_end_limit,
        **({"late_pass_jobs_end_before_earliest_frame": (agg(links, "late_job_end_us_max") or 0)
            <= tick_us - consts["NLINK_EARLY_US"]} if vtol else {}),
        "exchange_isrs_le_58_cycles": all((isr[k].get("longest") or 999) <= ISR_EXCHANGE_LIMIT for k in exchange_isrs),
        "no_timer_or_twi_isr_on_fcu": all("error" in isr[k] for k in ABSENT_ON_FCU),
        "adc_isr_only_in_power_builds": ("error" not in isr["adc"]) == bool(adc_n),
        "stack_free_ge_1024": s["flight_stack_free_bytes"] >= STACK_MIN_FREE,
        "reached_powered": bool(flight),
        **({"vtol_profile_spool_climb_hold_land_landed_safe": profile_ok, "no_vtol_record_dropped": v["vrec_dropped"] == 0,
            # v0.7.1
            "thr_q_byte_rule": v["thr_q_bad_frames"] == 0 and 0 < (v["thr_q_max"] or 0) <= 250
                               and abs((v["thr_q_max"] or 0) - round(v["esc_u_max"] * 250)) <= 1,
            "touchdown_by_a_signature": v["td_by_first"] in ("still", "support", "floor", "all"),
            "imu_sample_to_vanes_le_6dof_delay": 0 < s["imu_sample_to_vanes_us_max"] <= FC_COMPUTE_DELAY_US_6DOF,
            "no_flight_record_dropped_in_calm_flight": s["flight_records_dropped_calm"] == 0, **pos_checks}
           if vtol else {"thr_q_byte_zero": s.get("thr_q_bad_frames", 0) == 0}),
        "nav_frames_all_counted": (nl["bad"] == 1 and nl["seq_lost"] == 1 + 1 + consts["SB_NAV_GAP_TICKS"]
                                   and nl["timeouts"] - (nl["timeouts_at_boot"] or 0) == exp_timeouts
                                   and nl["spi_resyncs"] == 0 and nl["spi_overruns"] == 0 and nl["built_late"] == 0
                                   and s["no_frame_ticks"] == nl["timeouts"] + nl["bad"]),
        "fallback_first_after_timeout": consts["NLINK_FRAME_TIMEOUT_US"] <= (nl["fallback_first_us_max"] or 0)
                                        <= consts["NLINK_FRAME_TIMEOUT_US"] + 150,
        "fallback_period_timer2": (nl["fallback_period_us_min"] or 0) >= tick_us - 20
                                  and (nl["fallback_period_us_max"] or 0) <= tick_us + 150,
        "nav_reset_in_flight_i9_abort": bool(nav_reset_ok),
        "cmd_latency_0": il["latency_max_ticks"] == 0 and (il["latency_min_ticks"] or 0) >= 0,
        "every_cmd_answered": il["snap_received"] > 0 and il["cmd_unanswered"] == 0 and il["bad_snap_or_wire_full"] == 0,
        "background_keeps_up": s["flight_total_max_us"] < tick_us,
        "babble_flood_detected": (bb["flood_ticks"] or 0) >= consts["SB_BABBLE_TICKS"] - 2 and (bb["rx_overflow"] or 0) > 0,
        "slow_babble_sent": (sbb["bytes_sent"] or 0) >= consts["SB_SLOW_BABBLE_TICKS"] * tick_us / consts["SB_SLOW_BABBLE_US"] - 10,
        "no_flight_record_dropped": s["flight_records_dropped"] == 0,
        "no_record_dropped": s["evq_dropped"] == 0,
        "status_record_every_second": (s["status_records_written"] or 0) >= len(rows) - 2,
        "no_frame_without_baro_ok": s["baro_not_ok_frames"] == 0,
        "babble_loop_inside_tick": 0 < bb["total_max_us"] < tick_us and 0 < sbb["total_max_us"] < tick_us,
        "babble_at_most_one_cmd_lost_after_each": (bb["cmd_unanswered_after"] or 0) <= 2,
        "simbench_isr_emulation_matches_elf": rx == consts["SB_ISR_CYCLES"],
    }
    s["pass"] = all(s["checks"].values())
    print(f"[{mode}] {ENV} ({len(rows)} one-second windows, states {' '.join(dict.fromkeys(s['states']))}, "
          f"{s['control_ticks']} ticks with the controller active):")
    print(f"  time-critical part (frame -> end of the tick's outputs): pad {s['pad_crit_max_us']} us, flight "
          f"{s['flight_crit_max_us']} us -> max {100.0 * s['crit_fraction']:.1f} % of the {tick_us:.0f} us tick (limit "
          f"{CRIT_LIMIT:.0%}; v0.6.3 FCU {V063['crit_max_us']} us)")
    pc = s["interrupts"]["per_crit"]
    print(f"  + interrupts (from the fcu ELF): ilink RX {rx} cycles x {ILINK_BURST_BYTES} B = {pc['ilink_rx']['us']} us, "
          f"log UDRE {udre} x {n_log} = {pc['log_udre']['us']} us"
          + (f", ADC chain {pc['adc']['cycles_each']} x {pc['adc']['count']} = {pc['adc']['us']} us" if "adc" in pc else "")
          + f" -> {s['interrupts']['crit_plus_all_us']} us = "
          f"{100 * s['crit_plus_all_isr_fraction']:.1f} % (v0.6.3 FCU {V063['crit_plus_all_isr_us']} us)")
    print(f"  IMU sample -> vanes <= {s['imu_sample_to_vanes_us_max']} us = NAV lead {s['imu_lead_us']:.0f} us + NAV tick -> "
          f"vanes (limit {FC_COMPUTE_DELAY_US_6DOF} us = the 6-DOF's FC_COMPUTE_DELAY_S_MEASURED); flight records dropped "
          f"in calm flight {s['flight_records_dropped_calm']} (whole run {s['flight_records_dropped']}: by cause "
          f"{s['flight_records_dropped_by_cause']})")
    print(f"  sample -> PWM: NAV tick -> frame at the FCU {consts['SB_NAV_FRAME_US']} us (synthetic NAV: push at "
          f"{consts['SB_NAV_PUSH_US']} us + {consts['NLINK_XFER_US']} us exchange) + FCU lateness <= "
          f"{nl['start_late_fcu_us_max']} us + tick start -> last vane register <= {nl['tick_start_to_vanes_ctrl_us_max']} us "
          f"-> NAV tick -> vanes <= {nl['nav_tick_to_vanes_ctrl_us_max']} us with the controller active "
          f"(<= {nl['nav_tick_to_vanes_us_max']} us any state; limit {tick_us:.0f}).  The FCU's share is "
          f"{s['fcu_share_frame_to_vanes_us_max']} us: the NAV's frame must be complete <= "
          f"{s['nav_budget_frame_complete_after_tick_us']:.0f} us after its tick")
    print(f"  nlink: {nl['frames']} valid frames, {nl['bad']} bad, {nl['timeouts']} timeout ticks ({nl['timeouts_at_boot']} at "
          f"boot), {nl['seq_lost']} NAV ticks lost, resyncs {nl['spi_resyncs']}, overruns {nl['spi_overruns']}; fallback: "
          f"first {nl['fallback_first_us_max']} us after the last tick (timeout {consts['NLINK_FRAME_TIMEOUT_US']}), then "
          f"{nl['fallback_period_us_min']}..{nl['fallback_period_us_max']} us (Timer2); NAV reset at tick "
          f"{nl['nav_reset_tick']} -> ABORT at tick {nl['abort_tick']} (reason {nl['abort_reason']}: I9 IMU)")
    print(f"  background: anchor -> end <= {nl['bg_end_us_max']} us (limit {bg_end_limit}; without the bypass's lumped "
          f"RX-interrupt emulation {nl['bg_end_no_isr_emulation_us_max']} us; the latest background job ended "
          f"{nl['bg_job_end_us_max']} us after the anchor), margin to the next exchange >= "
          f"{nl['bg_margin_us_min']} us; tick-start lateness: FCU <= {nl['start_late_fcu_us_max']} us, simbench bypass "
          f"<= {nl['start_late_simbench_us_max']} us (excluded)")
    print(f"  incl. background:   pad {s['pad_total_max_us']} us, flight {s['flight_total_max_us']} us (v0.6 "
          f"{V06['flight_total_max_us']}); longest background job {s['bg_step_max_us']} us ({s['bg_step_max_kind']}; "
          f"SNAP {s['snap_step_max_us']} us), on BG_RESERVE_US {s['bg_job_max_us']} us, record jobs "
          f"{s['rec_job_max_us']} us, text {s['text_job_max_us']} us, "
          f"ilink drain {s['ilink_drain_max_us']} us; simbench's own work {s['simbench_overhead_max_us']} us")
    print(f"  ilink: {il['il_cmd_sent']} IL_CMD + {il['il_stat_sent']} IL_STAT from the synthetic COM, {il['snap_received']} "
          f"SNAPs back ({il['cmd_unanswered']} CMDs unanswered, {il['bad_snap_or_wire_full']} bad); bytes waiting at a "
          f"tick <= {il['wait_at_tick_max_bytes']}, decoded in the time-critical part <= {il['crit_decoded_max_bytes']} "
          f"({il['crit_over']} ticks over budget); background share used up in {il['bg_capped_ticks']} ticks")
    if s["flight_records_dropped_by_window"]:
        print("  flight records dropped, by report window: " + ", ".join(
            f"{w['dropped']} by {w['t_s']:.1f} s ({w['state']})" for w in s["flight_records_dropped_by_window"])
              + f"; status records written {s['status_records_written']} in {len(rows)} s")
    print(f"  IL_CMD last byte -> supervisor: <= {il['latency_max_ticks']} ticks (>= {il['latency_min_ticks']}); "
          f"right behind an IL_STAT <= {il['latency_after_stat_max_ticks']} ticks; -> SNAP queued <= "
          f"{il['latency_queued_max_ticks']} ticks; SNAPs replaced {il['snap_dropped']}, flight records dropped "
          f"{s['flight_records_dropped']}, records dropped {s['evq_dropped']}")
    print(f"  slow babble (R1) ticks {sbab0}..{sbab1}: {sbb['bytes_sent']} bytes C5 5C 08 01 every {sbb['period_us']} us, "
          f"time-critical max {sbb['crit_max_us']} us, incl. background {sbb['total_max_us']} us")
    print(f"  babble (D6) ticks {bab0}..{bab1}: {bb['bytes_sent']} bytes, flood ticks {bb['flood_ticks']}, RX overflow "
          f"{bb['rx_overflow']}, time-critical max {bb['crit_max_us']} us, incl. background {bb['total_max_us']} us (calm "
          f"flight {bb['calm_flight_total_max_us'] if bb['calm_flight_total_max_us'] is not None else 'n/a: every flight window '
          f'holds babble'}); CMDs lost right after the babbles: {bb['cmd_unanswered_after']}")
    print(f"  ticks without a valid navigation frame {s['no_frame_ticks']}; stack: {s['stack_used_bytes']} bytes deep "
          f"(simbench {s['static_ram_bytes']} B static, {s['stack_free_min_bytes']} B never touched) -> flight build "
          f"{FLIGHT_ENV} ({s['flight_static_ram_bytes']} B static, {s['flight_flash_bytes']} B flash) keeps "
          f"{s['flight_stack_free_bytes']} B free (limit {STACK_MIN_FREE}; v0.6.3 {V063['flight_stack_free_bytes']} B, "
          f"{V063['flight_static_ram_bytes']} B static, {V063['flight_flash_bytes']} B flash, {V063['stack_used_bytes']} B deep)")
    for name, v in s["flight_sections_max_us"].items():
        print(f"    {v:5d} us  {name}")
    if vtol:
        v = s["vtol"]
        print(f"  FCU own work per POWERED tick (tick start -> background end, the bench's own time removed): max "
              f"{v['own_work_max_us']} us, mean {v['own_work_mean_us']} us over {v['own_work_ticks']} ticks; first LAND "
              f"reason {v['land_reason_first']} at tick {v['land_tick_first']}; late-pass jobs end <= "
              f"{agg(links, 'late_job_end_us_max')} us after the anchor (earliest frame {tick_us - consts['NLINK_EARLY_US']:.0f})")
    if gps:
        ph = s["position_hold"]
        print(f"  GNS slot: TILT parts {ph['slot_tilts']}, POWERED ticks in the TRIM fallback {ph['fallback_ticks']} "
              f"(synthetic GNS dropout {ph['gns_dropout_cycles']} cycles), RESULTs {ph['gns_results']}")
        print(f"  position hold: modes {' '.join(ph['modes_seen'])}, engaged {ph['engaged_s']:.1f} s, LAND requests "
              f"{ph['requests_seen']}; position work late (finished at the next tick start) in {ph['late_ticks']} ticks "
              f"({100 * ph['late_fraction_of_engaged']:.0f} % of the engaged ticks), late finish <= {ph['late_finish_max_us']} us, "
              f"longest job {ph['job_max_us']} us, a job in the exchange window ended <= {ph['job_end_us_max']} us after the "
              f"anchor; 'G' dropped {ph['grec_dropped']}; slot sets {ph['slot_sets']} (broken {ph['slot_broken']}); GPS ON "
              f"gate at {ph['gps_gate_ok_s']} s; tilt setpoint <= {ph['tilt_max_deg']} deg")
    if vtol:
        v = s["vtol"]
        print(f"  VTOL profile: phases {' '.join(v['phases_seen'])}, {v['motor_starts']} motor starts, {v['hops']} ONs, "
              f"commands executed {v['cmds_executed']} / rejected {v['cmds_rejected']}; HOLD {v['hold_s']:.1f} s, ESC "
              f"{v['esc_u_hold_min']:.3f}..{v['esc_u_max']:.3f} (ceiling {v['sat_high_s']:.2f} s in POWERED); synthetic plant max "
              f"{v['plant_h_max_m']:.2f} m, hardest ground contact {v['plant_touchdown_mps']:.3f} m/s; LANDED at "
              f"{v['landed_s']:.2f} s, SAFE at {v['safe_after_landed_s']:.2f} s; final state {v['final_state']}; "
              f"'V' records dropped {v['vrec_dropped']}; thr_q max {v['thr_q_max']} (ESC u max x 250 = "
              f"{round(v['esc_u_max'] * 250)}), frames out of the thr_q rule {v['thr_q_bad_frames']}; first touchdown "
              f"confirmed by the {v['td_by_first']} signature")
    o = s["outputs_us"]
    print(f"  output pulses read back from the compare registers [us]: pad ESC {o['pad']['ocr3b']}, release "
          f"{o['pad']['ocr3c']}, vane1 {o['pad']['ocr1a'][:1]}..{o['pad']['ocr1a'][-1:]}; flight ESC {o['flight']['ocr3b'][:1]}.."
          f"{o['flight']['ocr3b'][-1:]}, vane1 {o['flight']['ocr1a'][:1]}..{o['flight']['ocr1a'][-1:]}")
    return s


# v0.9 (fw09 WP-B): the checks of this single-chip bench that measure the bench's OWN load (the synthetic NAV / COM / plant
# and the report lines run on the FCU chip: SPEC_firmware_v09 section 7 note) -> the row of the closed-loop four-core
# co-simulation (tools/run_cosim4.py, host_test/build/cosim4_report.json) that judges the same quantity on the flight code.
# Once that report judged the row on THESE sources (its source_digest equals source_digest() of this tree), the check's
# PASS / FAIL is the co-simulation's ("cosim_row<N>: ..." in the checks) and the bench's own figure is kept as information
# under "bench_self_load" with the bench's own work per tick.  Without such a report the check stays a bench check.
SELF_LOAD_CHECKS = {
    "nav_tick_to_vanes_le_1_tick": "8", "imu_sample_to_vanes_le_6dof_delay": "7",
    "bg_ends_before_next_exchange": "13", "background_keeps_up": "13",
    "no_flight_record_dropped_in_calm_flight": "11", "no_vtol_record_dropped": "11", "no_flight_record_dropped": "11",
    "babble_loop_inside_tick": "18.ilink", "trim_fallback_only_in_the_dropout": "18.gns", "nav_frames_all_counted": "18.nlink",
    "micro_vtol_hold_benchmark_in_hold": "14", "bg_reserve_covers_longest_job": "14", "text_reserve_covers_it": "14",
    "rec_reserve_covers_record_jobs": "14"}
COSIM4_REPORT = os.path.join(OUT_DIR, "cosim4_report.json")


def source_digest(fw: str = FW) -> str:
    """sha256 over the firmware sources (src/**/*.c, *.h, platformio.ini), relative path + bytes - the co-simulation
    report carries the same digest of the tree it ran on"""
    h = hashlib.sha256()
    files = [os.path.join(fw, "platformio.ini")]
    for d, _, fs in os.walk(os.path.join(fw, "src")):
        files += [os.path.join(d, f) for f in fs if f.endswith((".c", ".h"))]
    for p in sorted(files, key=lambda x: os.path.relpath(x, fw).replace("\\", "/")):
        h.update(os.path.relpath(p, fw).replace("\\", "/").encode())
        with open(p, "rb") as f:
            h.update(f.read())
    return h.hexdigest()


def defer_self_load(env_report: dict, checks: dict, mode: str) -> None:
    """move the self-load checks to the co-simulation rows that judged them (see SELF_LOAD_CHECKS); in place"""
    info: dict = {"cosim_report": os.path.relpath(COSIM4_REPORT, ROOT), "deferred": {}, "kept_as_bench_checks": []}
    rep = None
    if mode == "vtol":
        try:
            with open(COSIM4_REPORT, "r", encoding="utf-8") as f:
                rep = json.load(f)
        except (OSError, ValueError):
            rep = None
    digest = source_digest()
    if rep is not None and rep.get("source_digest") != digest:
        info["note"] = "co-simulation report of other sources (digest differs): every check stays a bench check"
        rep = None
    elif rep is not None and not rep.get("complete_set"):
        info["note"] = "co-simulation report of a partial run (--only): every check stays a bench check"
        rep = None
    elif rep is None:
        info["note"] = "no co-simulation report (run tools/run_cosim4.py first): every check stays a bench check"
    rows = (rep or {}).get("rows", {})
    for name, row in SELF_LOAD_CHECKS.items():
        if name not in checks:
            continue
        r = rows.get(row)
        if r is None or r.get("verdict") not in ("PASS", "FAIL"):
            info["kept_as_bench_checks"].append(name)
            continue
        info["deferred"][name] = {"bench_value": checks.pop(name), "cosim_row": row, "cosim_verdict": r["verdict"],
                                  "cosim_criterion": r.get("criterion")}
        checks[f"cosim_row{row}"] = r["verdict"] == "PASS"
    info["bench_own_work"] = {"simbench_overhead_max_us": env_report.get("simbench_overhead_max_us"),
                              "start_late_simbench_us_max": (env_report.get("nlink") or {}).get("start_late_simbench_us_max"),
                              "fcu_own_work_max_us": (env_report.get("vtol") or {}).get("own_work_max_us"),
                              "_doc": "the bench's own work per tick on the FCU chip (ox: after the background; ls: the "
                                      "bypass injection at the tick start) and the FCU's own work with the bench's time "
                                      "removed - why the bench's timing figures are information, not criteria"}
    env_report["bench_self_load"] = info


def rejudge() -> int:
    """the VTOL reports of this tree against the current co-simulation report: restore the deferred checks, defer again"""
    ok = True
    for name in ("simbench_report_vtol.json", "simbench_report_vtol_demo.json", "simbench_report_vtol_nogps.json",
                 "simbench_report_vtol_wired.json", "simbench_report_vtol_wired_buffer.json"):
        path = os.path.join(OUT_DIR, name)
        try:
            with open(path, "r", encoding="utf-8") as f:
                rep = json.load(f)
        except (OSError, ValueError):
            print(f"{name}: missing (run the bench first)")
            ok = False
            continue
        envs = [k for k, v in rep.items() if isinstance(v, dict) and "checks" in v]
        for env in envs:
            er = rep[env]
            checks = er["checks"]
            for k in [k for k in checks if k.startswith("cosim_row")]:
                del checks[k]
            for k, d in (er.get("bench_self_load") or {}).get("deferred", {}).items():
                checks[k] = d["bench_value"]
            defer_self_load(er, checks, "vtol")
            er["pass"] = all(checks.values())
        rep["pass"] = all(rep[e]["pass"] for e in envs)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(rep, f, indent=1)
        deferred = sorted((rep[envs[0]].get("bench_self_load") or {}).get("deferred", {})) if envs else []
        print(f"{name}: {'PASS' if rep['pass'] else 'FAIL'}; deferred to the co-simulation: {', '.join(deferred) or '-'}")
        ok = ok and rep["pass"]
    return 0 if ok else 1


def micro_checks(micro: dict, consts: dict, bg_job_max_us: int, text_max_us: int, rx_cycles: int | None,
                 rec_job_max_us: int | None = None) -> dict:
    """the main.c byte-time reserves against the measured ilink paths (CPU cycles, 16 per us); VTOL: the record jobs'
    own reserve REC_RESERVE_US against the longest record job"""
    stretch = 1.0 + (rx_cycles or 0) / (16.0 * 40.0)        # RX ISR per 40 us byte time while bytes keep coming
    per_byte = max(micro.get("main_ilink_rx_bg_stat_cmd_56B", 0) / 56.0, micro.get("main_ilink_rx_bg_syncspam_32B", 0) / 32.0,
                   micro.get("main_ilink_rx_bg_noise_32B", 0) / 32.0)
    chunk_worst = per_byte * consts["ILINK_CHUNK"] + 16.0 * 20.0       # + an IL_STAT's handling (36-byte copy)
    return {
        "micro_cmds_decoded_4": micro.get("main_check_cmds_decoded_of_4") == 4,
        "micro_crit_fed_16": micro.get("main_check_crit_bytes_of_16") == consts["ILINK_CRIT_BYTES"],
        "micro_bg_fed_64": micro.get("main_check_bg_bytes_of_64") == 64,
        "byte_reserve_covers_bg_byte": per_byte * stretch <= consts["ILINK_BYTE_US"] * 16.0,
        "chunk_reserve_covers_chunk": chunk_worst * stretch <= (consts["ILINK_FRAME_US"] + consts["ILINK_CHUNK"] *
                                                                consts["ILINK_BYTE_US"]) * 16.0,
        "bg_reserve_covers_longest_job": 0 < bg_job_max_us * stretch <= consts["BG_RESERVE_US"],
        "text_reserve_covers_it": text_max_us * stretch <= consts["COM_TEXT_RESERVE_US"],
        **({"rec_reserve_covers_record_jobs": 0 < rec_job_max_us * stretch <= consts["REC_RESERVE_US"]}
           if "REC_RESERVE_US" in consts else {}),
    }


def run_mode(mode: str, p: dict, profile: str = "field") -> dict:
    """One mode's bench + micro + co-simulation build; returns its report (with ["pass"]).  VTOL profile 'demo': the
    demo bench and flight build only (the micro / co-simulation builds are the field profile's: same code)."""
    MICRO_ENV, COSIM_ENV = MODE_ENVS[mode]["micro"], MODE_ENVS[mode]["cosim"]
    demo = mode == "vtol" and profile in ("demo", "nogps", "wired", "wired_buffer")       # bench + flight build only
    ENV = MODE_ENVS[f"vtol_{profile}" if demo else mode]["bench"]
    sfx = mode.upper()
    loop_hz = int(p.get("control", {}).get("loop_hz", 250))
    tick_us = 1e6 / loop_hz
    log_baud = float(p.get("flight_computer", {}).get("log_baud", 57600))
    consts = {**source_defines(os.path.join(FW, "src", "main.c"),
                               ("ILINK_CRIT_BYTES", "ILINK_BG_BYTES", "ILINK_BYTE_US", "ILINK_FRAME_US", "ILINK_CHUNK",
                                "NLINK_EARLY_US")),
              # fw09 (WP-F): the background reserves per mission (main.c BG_RESERVE_US_VTOL / _BALLISTIC)
              "BG_RESERVE_US": source_defines(os.path.join(FW, "src", "main.c"), (f"BG_RESERVE_US_{sfx}",))[
                  f"BG_RESERVE_US_{sfx}"],
              **{k[:-len(sfx) - 1]: v for k, v in source_defines(
                  os.path.join(FW, "src", "simbench.h"),
                  tuple(f"{n}_{sfx}" for n in ("SB_BABBLE_S", "SB_BABBLE_TICKS", "SB_SLOW_BABBLE_S",
                                               "SB_SLOW_BABBLE_TICKS"))).items()},
              **source_defines(os.path.join(FW, "src", "simbench.h"),
                               ("SB_SLOW_BABBLE_US", "SB_NAV_GAP_TICKS", "SB_NAV_RESET_TICKS", "SB_NAV_BAD_TICK",
                                "SB_NAV_MISS_TICK", "SB_NAV_GAP_TICK")),
              # v0.7.1: the synthetic NAV's push time per build (simbench.h SB_NAV_PUSH_US_VTOL / _BALLISTIC)
              "SB_NAV_PUSH_US": source_defines(os.path.join(FW, "src", "simbench.h"), (f"SB_NAV_PUSH_US_{sfx}",))[
                  f"SB_NAV_PUSH_US_{sfx}"],
              **source_defines(os.path.join(FW, "src", "simbench.c"), ("SB_ISR_CYCLES",)),
              **source_defines(os.path.join(FW, "src", "params_gen.h"),
                               ("FC_TICK_US", "NLINK_XFER_US", "NLINK_FRAME_TIMEOUT_US", "ONB_IMU_FAIL_TICKS",
                                "FC_IMU_LEAD_COUNTS", "FC_T2_PRESCALER", "FC_F_CPU_HZ"))}
    consts["SB_NAV_FRAME_US"] = consts["SB_NAV_PUSH_US"] + consts["NLINK_XFER_US"]
    consts["IMU_LEAD_US"] = consts["FC_IMU_LEAD_COUNTS"] * consts["FC_T2_PRESCALER"] * 1e6 / consts["FC_F_CPU_HZ"]
    _fct = os.path.join(ROOT, "sim", "fc_timing.py")
    with open(_fct, "r", encoding="utf-8") as f:
        m = re.search(r"^FC_COMPUTE_DELAY_S_MEASURED\s*=\s*([0-9.eE+-]+)", f.read(), re.M)
    if not m or round(float(m.group(1)) * 1e6) != FC_COMPUTE_DELAY_US_6DOF:
        raise SystemExit(f"sim/fc_timing.py FC_COMPUTE_DELAY_S_MEASURED is not {FC_COMPUTE_DELAY_US_6DOF} us: update "
                         f"run_simbench.py FC_COMPUTE_DELAY_US_6DOF (the 6-DOF's vane delay is the criterion)")
    # fw09 (WP-F): COM_TEXT_RESERVE_US = BG_RESERVE_US + COM_TEXT_EXTRA_US_<MODE> (main.c)
    consts["COM_TEXT_RESERVE_US"] = consts["BG_RESERVE_US"] + source_defines(
        os.path.join(FW, "src", "main.c"), (f"COM_TEXT_EXTRA_US_{sfx}",))[f"COM_TEXT_EXTRA_US_{sfx}"]
    if mode == "vtol":                                        # the record jobs' own reserve (main.c, VTOL build)
        consts["REC_RESERVE_US"] = source_defines(os.path.join(FW, "src", "main.c"), ("REC_RESERVE_US_VTOL",))[
            "REC_RESERVE_US_VTOL"]
        consts.update(source_defines(os.path.join(FW, "src", "main.c"), ("POS_RESERVE_US",)))
        consts.update(source_defines(os.path.join(FW, "src", "simbench.h"), ("SB_ON_S",)))
    report: dict = {"crit_limit_fraction": CRIT_LIMIT, "mode": mode, "profile": profile if mode == "vtol" else None,
                    "role": f"FCU (v0.8 flight control unit, frame-triggered, {mode} build)", "main_c": consts}
    report[ENV] = bench(mode, tick_us, loop_hz, log_baud, consts, profile)
    if demo:
        checks = report[ENV]["checks"]
        defer_self_load(report[ENV], checks, mode)            # v0.9 WP-B: self-load checks -> the co-simulation rows
        report["build_warnings"] = {k: v for k, v in WARNINGS.items() if v}
        report[ENV]["pass"] = report["pass"] = all(checks.values())
        print(f"  [vtol {profile}] checks: " + ", ".join(f"{k} {'ok' if v else 'FAIL'}" for k, v in checks.items())
              + f" -> {'PASS' if report['pass'] else 'FAIL'}")
        return report
    elf = build(MICRO_ENV)
    try:
        text = run(elf)
    except subprocess.TimeoutExpired as exc:                 # reported as a failed check, the other runs go on
        text = (exc.output or "") + "\nRUNNER: simavr timeout\n"
    report["micro_crashed"] = "avr_sadly_crashed" in text or "RUNNER: simavr timeout" in text
    if report["micro_crashed"]:
        m = re.search(r"Invalid Opcode SP=([0-9a-fA-F]+)", text)
        report["micro_crash_note"] = (f"{MICRO_ENV} stopped before / in its boot-time benchmarks: "
                                      + (f"execution left the code (invalid opcode, SP 0x{m.group(1)})" if m else
                                         "no end of run")
                                      + f"; static RAM {STATIC_RAM.get(MICRO_ENV)} B of {RAM_BYTES} B")
        print(f"  {report['micro_crash_note']}")
    report["micro_cycles"] = parse_sbm(text)
    report["micro_env_static_ram_bytes"] = STATIC_RAM.get(MICRO_ENV)
    # the micro build runs the same loop afterwards, its flight shifted by the boot-time benchmarks: a second trajectory
    mrows, _, _, _ = parse(text)
    mflight = [r for r in mrows if r.get("st") in (2, 3)]
    report["micro_env_flight_crit_max_us"] = max((r.get("crit_max_us", 0) for r in mflight), default=None)
    report["micro_env_stack_free_min_bytes"] = min((r.get("stack_free", 0) for r in mrows), default=None)
    if mode == "vtol":                                       # the HOLD benchmarks really ran in HOLD (no rule fired)
        report["micro_vtol_hold_kept"] = report["micro_cycles"].get("vtol_check_hold_kept") == 1
    ok2 = report["micro_env_flight_crit_max_us"] is not None and report["micro_env_flight_crit_max_us"] <= CRIT_LIMIT * tick_us
    checks = report[ENV]["checks"]
    checks["second_trajectory_crit_le_80pct"] = ok2
    checks["micro_build_stack_never_exhausted"] = (report["micro_env_stack_free_min_bytes"] or 0) > 0
    checks["micro_build_runs_without_crash"] = not report["micro_crashed"]
    if mode == "vtol":
        checks["micro_vtol_hold_benchmark_in_hold"] = report["micro_vtol_hold_kept"]
    checks.update(micro_checks(report["micro_cycles"], consts, report[ENV]["bg_job_max_us"],
                               report[ENV]["text_job_max_us"],
                               report[ENV]["interrupts"]["isr_cycles"]["ilink_rx"].get("normal"),
                               report[ENV]["rec_job_max_us"]))
    defer_self_load(report[ENV], checks, mode)                # v0.9 WP-B: self-load checks -> the co-simulation rows
    build(COSIM_ENV)                                          # the co-simulation FCU (real nlink SPI + ilink) must build
    report["cosim_env"] = {"env": COSIM_ENV, "static_ram_bytes": STATIC_RAM.get(COSIM_ENV),
                           "flash_bytes": FLASH.get(COSIM_ENV)}
    report["build_warnings"] = {k: v for k, v in WARNINGS.items() if v}
    report[ENV]["pass"] = all(checks.values())
    print(f"{MICRO_ENV}: the same loop after the boot-time benchmarks (another flight trajectory): time-critical max "
          f"{report['micro_env_flight_crit_max_us']} us in flight ({'ok' if ok2 else 'FAIL'}), stack never touched "
          f"{report['micro_env_stack_free_min_bytes']} B of its {RAM_BYTES - (report['micro_env_static_ram_bytes'] or 0)} B")
    print(f"{COSIM_ENV}: builds ({report['cosim_env']['static_ram_bytes']} B static RAM, "
          f"{report['cosim_env']['flash_bytes']} B flash)")
    print("per-operation cycles (avr-libc float, 32-call average; composite entries are optimistic: the compiler "
          "hoists loop-invariant work; main_* = one timed call of main.c's own path):")
    for k, v in report["micro_cycles"].items():
        print(f"    {k:<34} {v:6d}")
    isr = report[ENV]["interrupts"]["isr_cycles"]
    print("ISR cycles (fcu ELF, incl. 7 cycles entry): " +
          ", ".join(f"{k} " + (f"normal {v['normal']} / longest {v['longest']}" if "error" not in v else v["error"])
                    for k, v in isr.items()))
    for env, w in report["build_warnings"].items():
        print(f"build warnings {env}: " + "; ".join(w))
    print(f"  [{mode}] checks: " + ", ".join(f"{k} {'ok' if v else 'FAIL'}" for k, v in checks.items())
          + f" -> {'PASS' if report[ENV]['pass'] else 'FAIL'}")
    report["pass"] = report[ENV]["pass"]
    return report


def main(argv: list[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--mode", choices=("both", "vtol", "ballistic"), default="both",
                    help="vtol: the plain envs (params.json mission.mode); ballistic: the *_ballistic envs; both (default)")
    ap.add_argument("--profile", choices=("both", "field", "demo", "nogps", "all", "wired", "wired_buffer", "power"), default="both",
                    help="VTOL build profile(s): field = fcu_simbench, demo = fcu_simbench_demo; both (default); nogps = "
                         "fcu_simbench_nogps (v0.8 without position hold); all = field, demo, nogps")
    ap.add_argument("--rejudge", action="store_true", help="v0.9: no simulation - re-apply the co-simulation deferral of the "
                    "self-load checks (SELF_LOAD_CHECKS) to the VTOL reports already in host_test/build and rewrite them")
    a = ap.parse_args(argv)
    if a.rejudge:
        return rejudge()
    p = params()
    params_mode = str(p.get("mission", {}).get("mode", "ballistic"))
    if params_mode != "vtol" and a.mode in ("both", "vtol"):
        raise SystemExit(f"params.json mission.mode is {params_mode!r}: the plain envs are not the VTOL build")
    modes = ["vtol", "ballistic"] if a.mode == "both" else [a.mode]
    profiles = {"both": ["field", "demo"], "all": ["field", "demo", "nogps"],
                "power": ["wired", "wired_buffer"]}.get(a.profile, [a.profile])
    runs = [(m, pr) for m in modes for pr in (profiles if m == "vtol" else ["field"])]
    ok = True
    for mode, prof in runs:
        report = run_mode(mode, p, prof)
        ok = ok and report["pass"]
        os.makedirs(OUT_DIR, exist_ok=True)
        if mode == "vtol" and prof in ("demo", "nogps", "wired", "wired_buffer"):
            paths = [os.path.join(OUT_DIR, f"simbench_report_vtol_{prof}.json")]
        else:
            paths = [os.path.join(OUT_DIR, f"simbench_report_{mode}.json")]
            if mode == params_mode:
                paths.append(os.path.join(OUT_DIR, "simbench_report.json"))
        for path in paths:
            with open(path, "w", encoding="utf-8", newline="\n") as f:
                json.dump(report, f, indent=1)
            print(f"-> {os.path.relpath(path, ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
