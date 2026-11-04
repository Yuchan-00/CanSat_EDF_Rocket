"""Flight-computer output timing: compute delay, PWM-frame zero-order hold and
servo pulse decoding (ATmega128, ``params.flight_computer``).

Model, per output channel c (4 vane servos + ESC), control period
dt = 1 / ``control.loop_hz``, PWM frame period T_f = 1 / ``servo_frame_hz``:

1. compute delay: the command computed from the sensor sample taken at tick k
   (t_k = k dt) is *available* (written to the OCRnx register) at t_k + d,
   d = ``FCTiming.delay_s(dt)`` with sub-tick resolution.  Default: the v0.6.3
   three-chip co-simulation's measured maximum ``FC_COMPUTE_DELAY_S_MEASURED``
   (NAV IMU sample -> NAV estimate -> nlink frame -> FCU controller -> last vane
   register); ``compute_delay_ticks`` keeps the integer form of the v0.5 single
   chip (1 = the next tick, 4.0 ms).  VTOL build (v0.7.1 tick order: attitude ->
   vanes -> supervisor -> vertical controller -> ESC): the vanes keep ``delay_s``,
   the ESC channel gets ``FCTiming.esc_delay_s`` = ``delay_s`` +
   ``FC_VTOL_ESC_EXTRA_S_ASSUMED`` (``sim.sim6dof`` runs a separate latch for it).
2. frame latch (zero-order hold): the timer copies OCRnx into the compare unit
   only at its frame edges t_e = phase_c + n T_f (double-buffered OCR in
   fast-PWM mode 14).  Each edge takes the latest available command (a command
   that becomes available exactly at t_e is taken).  Frames are asynchronous to
   the control tick: phase_c is a fixed offset of unknown value (v0.6.3: the
   FCU's tick follows the NAV's frame, the PWM timers run free on the FCU), so
   the report sweeps it; all channels of one 16-bit timer share it
   (``timer_of_channel``).
3. pulse decode: a servo (and the ESC) acts on a pulse at its falling edge,
   i.e. ``pulse_width_s(c, value)`` after the frame edge.
4. downstream (in ``sim.sim6dof``): servo first-order lag + rate limit +
   mechanical saturation on the decoded command; thrust lag on the ESC command.

``frame_hz <= 0`` disables 2-3 (ideal actuator interface: the available command
is applied at the tick itself), which is the reference for the sensitivity
study.

v0.8 (architect 2026-10-21, params flight_computer.nav.schedule_note): the NAV publishes its frame after the
gyro prediction (``sim.estimator`` docstring) - this fixes the v0.7.1 co-simulation's IMU -> vane 5.2 ms > 4.71 ms and
NAV tick -> vane 4.7 ms > 4.0 ms.  The 6-DOF keeps ``FC_COMPUTE_DELAY_S_MEASURED`` (4.71 ms) as the vane path: it is the
bound the v0.8 build must meet, NOT a v0.8 measurement (the integration agent's co-simulation measures it); what the v0.8
order changes inside the tick is modelled in ``sim.sim6dof`` itself (the FCU gets the altitude filter of the previous
tick, the corrections one tick later).  The 50 Hz position step runs after the ESC write (its 4-tick job pipeline is in
``sim.onboard.FcuStack``), so the ESC path keeps ``FC_VTOL_ESC_EXTRA_S_ASSUMED``.

v0.8.1 (architect 2026-10-21 20:30, four ATmega128A): the GPS filter and the position controller run on the GNS board;
the GNS path delay is ``glink_latency_ticks`` (params flight_computer.glink.latency_ticks_ASSUMED = 3, ASSUMED until the
four-core co-simulation measures it): the NAV's STATE of tick k comes back as the nlink v4 TILT part in the frame of tick
k + L (``sim.glink_proto`` module docstring: the arrival estimate ~1.4 ms into tick k + 2 from the v0.8 C cycle counts,
so L = 2 is missed by ~0.4 ms and L = 3 leaves ~3.6 ms).  ``gns_chain_ticks`` gives the NAV-sample -> attitude and ->
supervisor delays of both chains; the 6-DOF (``sim.sim6dof``) flies the v0.8.1 chain with the real glink / nlink v4
bytes and this L (Scenario.gps.chain / glink_latency_ticks).  The vane and ESC paths are unchanged: the FCU no longer
runs the position step at all (its background is the no-GPS v0.8 build's, which passed: IMU -> vane 4.23 ms).

Invariant: ``tick`` is the only way a command reaches the plant; it returns
every change of the plant-side command with its exact time so the integrator
can break its steps there.
"""
from __future__ import annotations

import math
import re
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Sequence

EPS_T = 1e-9   # s, tie tolerance between frame edges and control ticks (float grid)

# v0.6.3 three-chip co-simulation (firmware/atmega128_fc/tools/run_cosim.py -> host_test/build/cosim_report.json,
# normal.by_backend.<radio>.latency_imu_sample_to_vane_equiv_flight_us, ~108 k flight ticks per backend): NAV IMU
# sample (MPU-6050 burst start, 0.496 ms before the NAV tick) -> the FCU's last vane OCR write, every interrupt
# charged: max 4709.0 us (sx127x) / 4692.9 us (lora_uart), p99 4.53 ms, p50 4.24 ms.  The model takes the maximum,
# rounded up to 10 us (conservative: the latched command is never younger than on the hardware).  v0.5 / v0.6.2 had
# one control tick (4.0 ms) by construction.
FC_COMPUTE_DELAY_S_MEASURED = 4.71e-3
# VTOL build (v0.7.1 FCU tick order, params mission.vtol.fcu_tick_order_note): attitude controller -> vane registers ->
# supervisor -> vertical controller -> ESC register.  The vane path keeps FC_COMPUTE_DELAY_S_MEASURED (it costs what it
# costs in the ballistic build: the v0.7 co-simulation measured 4621 / 4647 us ballistic); the ESC register is written
# after the supervisor + the vertical controller.  ASSUMED extra for that path until the co-simulation measures the
# v0.7.1 build: the v0.7 measurement of onb_stack_step (supervisor + vertical controller) = 748 us (simbench), 741 /
# 796 us (co-simulation, lora_uart / sx127x), BEFORE decision (11)'s cost cut (target <= ~400 us) - the 796 us maximum
# rounded up, i.e. conservative (the ESC command is modelled older than the cheaper v0.7.1 code will make it).
FC_VTOL_ESC_EXTRA_S_ASSUMED = 0.80e-3
# v0.8.1 GNS path (sim.glink_proto): NAV ticks from the STATE sample to the nlink frame carrying the TILT part - params
# flight_computer.glink.latency_ticks_ASSUMED; this is the fallback when params lack the key (the same number)
GLINK_LATENCY_TICKS_ASSUMED = 3
V08_POS_PIPELINE_TICKS = 4         # sim.position_ctrl.PIPELINE_TICKS (the v0.8 FCU job pipeline)
V08_SLOT_SET_TICKS = 5             # v0.8: the five-frame GPS slot set completes 5 ticks after the NAV's latch


def glink_latency_ticks(p: dict | None) -> int:
    """params flight_computer.glink.latency_ticks_ASSUMED (ASSUMED until the co-simulation measures it)."""
    if p is None:
        return GLINK_LATENCY_TICKS_ASSUMED
    g = p.get("flight_computer", {}).get("glink", {})
    return int(g.get("latency_ticks_ASSUMED", GLINK_LATENCY_TICKS_ASSUMED))


def gns_chain_ticks(chain: str, latency_ticks: int = GLINK_LATENCY_TICKS_ASSUMED) -> dict[str, int]:
    """Control ticks from the NAV's 50 Hz sample (NAV tick k, k mod 5 == 4) to (a) the attitude controller flying the
    position loop's tilt, (b) the supervisor acting on its LAND request, and the age of the FCU-side inputs at the step.
    "fcu" (v0.8): set complete k + 5, step latched there, committed k + 9, builder -> attitude k + 10, request k + 10.
    "gns" (v0.8.1): TILT part in the frame of k + L, builder -> attitude k + L + 1, STATUS set complete k + L + 4 (the
    supervisor reads it that tick); the FCU fields the GNS used are the FCU's tick k - 2 (nlink lag)."""
    if chain == "fcu":
        att = V08_SLOT_SET_TICKS + V08_POS_PIPELINE_TICKS + 1
        return {"sample_to_attitude": att, "sample_to_request": att, "fcu_inputs_age_at_attitude": att - V08_SLOT_SET_TICKS}
    if chain == "gns":
        L = int(latency_ticks)
        return {"sample_to_attitude": L + 1, "sample_to_request": L + 4, "fcu_inputs_age_at_attitude": L + 1 + 2}
    raise ValueError(f"chain {chain!r}")


@dataclass
class FCTiming:
    """Timing options of one simulation run (defaults = the ATmega128 baseline)."""

    compute_delay_ticks: int | None = None  # integer form (v0.5: 1 = next tick); used when compute_delay_s is None
    frame_hz: float | None = None           # None -> params.flight_computer.servo_frame_hz; <= 0 -> no frame latch
    phase_s: float | None = None            # first Timer1 frame edge after t = 0; None -> half a control period
    timer3_phase_s: float = 0.0             # extra offset of the Timer3 channels (vane 4, ESC) vs Timer1
    pulse_decode: bool = True               # servo/ESC act at the falling edge of their pulse
    compute_delay_s: float | None = None    # sample -> output register [s]; None -> ticks above, else the measured max
    esc_extra_s: float | None = None        # VTOL only: ESC path = the vane path + this; None -> FC_VTOL_ESC_EXTRA_S_ASSUMED

    def esc_delay_s(self, dt_c: float) -> float:
        """VTOL build (v0.7.1 tick order): sample -> ESC register [s] = the vane-path delay + the supervisor and
        vertical-controller time (``FC_VTOL_ESC_EXTRA_S_ASSUMED`` unless ``esc_extra_s`` is given).  The ballistic build
        writes every channel with the one ``delay_s`` (v0.6.3)."""
        extra = FC_VTOL_ESC_EXTRA_S_ASSUMED if self.esc_extra_s is None else float(self.esc_extra_s)
        if extra < 0.0 or not math.isfinite(extra):
            raise ValueError(f"ESC extra delay {extra!r} s must be finite and >= 0")
        return self.delay_s(dt_c) + extra

    def delay_s(self, dt_c: float) -> float:
        """Sample -> command available in the output registers [s] (sub-tick resolution)."""
        if self.compute_delay_s is not None and self.compute_delay_ticks is not None:
            raise ValueError("give compute_delay_s or compute_delay_ticks, not both")
        if self.compute_delay_s is not None:
            d = float(self.compute_delay_s)
        elif self.compute_delay_ticks is not None:
            d = int(self.compute_delay_ticks) * float(dt_c)
        else:
            d = FC_COMPUTE_DELAY_S_MEASURED
        if d < 0.0 or not math.isfinite(d):
            raise ValueError(f"compute delay {d!r} s must be finite and >= 0")
        return d


def timer_of_pin(pin_text: str) -> int:
    """'PB5/OC1A' -> 1, 'PE4/OC3B' -> 3 (the 16-bit timer that drives the pin)."""
    m = re.search(r"OC(\d)[A-C]", pin_text)
    if not m:
        raise ValueError(f"pin {pin_text!r} is not a 16-bit timer output compare pin")
    return int(m.group(1))


def timer_of_channel(fc: dict) -> list[int]:
    """Timer number of [vane 1..4, ESC] from ``params.flight_computer.pins``."""
    pins = fc["pins"]
    return [timer_of_pin(pins[f"vane_{i}_servo"]) for i in range(1, 5)] + [timer_of_pin(pins["esc"])]


def frame_hz_from_timer(fc: dict) -> float:
    """f_cpu / (prescaler (TOP + 1)) - cross-check of ``servo_frame_hz``."""
    return float(fc["f_cpu_hz"]) / (float(fc["pwm_timer_prescaler"]) * (float(fc["pwm_top"]) + 1.0))


class OutputLatch:
    """Compute delay (sub-tick) + per-channel frame ZOH + pulse decode for n channels."""

    def __init__(self, dt_c: float, delay_s: float, frame_hz: float, phases_s: Sequence[float],
                 init_values: Sequence[float], pulse_width_s: Callable[[int, float], float] | None = None):
        if not (delay_s >= 0.0 and math.isfinite(delay_s)):
            raise ValueError("compute delay must be finite and >= 0 s")
        self.dt = float(dt_c)
        self.n_ch = len(init_values)
        self.d_s = float(delay_s)
        self.zoh = frame_hz is not None and frame_hz > 0.0
        self.t_f = 1.0 / frame_hz if self.zoh else math.inf
        self.pulse_width_s = pulse_width_s if (self.zoh and pulse_width_s is not None) else (lambda c, v: 0.0)
        init = [float(v) for v in init_values]
        # commands not yet superseded: (t_available, values), oldest first; ``available`` = the newest one whose
        # time has been reached by the last edge (ZOH) or tick (no ZOH) processed - the initial values before any
        self.queue: deque[tuple[float, list[float]]] = deque()
        self.available = list(init)
        # first edge at or after t = 0 for each channel
        self.next_edge = [((float(p) % self.t_f) if self.zoh else math.inf) for p in phases_s]
        self.pending: list[tuple[float, int, float]] = []      # (t_apply, channel, value), sorted
        self.latched = list(init)                               # what each timer currently outputs
        self.sent = list(init)                                  # last plant-side value emitted per channel
        self.edges_log: list[tuple[float, int, float]] = []     # (t_edge, channel, value) - for tests / plots
        self.keep_edges_log = False

    def _advance(self, t: float) -> None:
        """Make every command available at or before ``t`` (ties within EPS_T count as available)."""
        while self.queue and self.queue[0][0] <= t + EPS_T:
            self.available = self.queue.popleft()[1]

    def tick(self, k: int, command: Sequence[float]) -> list[tuple[float, int, float]]:
        """Push the command computed at tick k (available at t_k + delay) and return the plant-side command
        changes (t, channel, value) with t in [t_k, t_{k+1}), sorted by time."""
        t0 = k * self.dt
        t1 = t0 + self.dt
        self.queue.append((t0 + self.d_s, [float(v) for v in command]))
        if not self.zoh:
            out = []
            while self.queue and self.queue[0][0] < t1 - EPS_T:
                t_a, vals = self.queue.popleft()
                t_a = t0 if t_a <= t0 + EPS_T else t_a          # a whole number of ticks lands on the tick itself
                self.available = vals
                for c in range(self.n_ch):
                    if vals[c] != self.sent[c]:              # unchanged commands need no integrator break
                        out.append((t_a, c, vals[c]))
                        self.sent[c] = vals[c]
            out.sort(key=lambda e: e[0])
            return out
        # frame edges of every channel in [t0, t1), in time order (the availability pointer only moves forward)
        edges: list[tuple[float, int]] = []
        for c in range(self.n_ch):
            te = self.next_edge[c]
            while te < t1 - EPS_T:
                edges.append((te, c))
                te += self.t_f
            self.next_edge[c] = te
        edges.sort()
        for te, c in edges:
            self._advance(te)
            v = self.available[c]
            self.latched[c] = v
            if self.keep_edges_log:
                self.edges_log.append((te, c, v))
            if v != self.sent[c]:
                self.pending.append((te + self.pulse_width_s(c, v), c, v))
                self.sent[c] = v
        self.pending.sort(key=lambda e: e[0])
        out = [e for e in self.pending if e[0] < t1 - EPS_T]
        self.pending = [e for e in self.pending if e[0] >= t1 - EPS_T]
        return out


def mean_wait_after_available(dt_c: float, frame_hz: float, phase_s: float, delay_s: float = 0.0,
                              n_frames: int = 200) -> float:
    """Mean time between a command becoming available (at t_k + ``delay_s``) and the frame
    edge that latches it, i.e. the age of the latched command beyond the compute
    delay.  Integer tick/frame ratios give one fixed value, others alternate; a
    delay that is a whole number of ticks gives the same value as no delay."""
    if frame_hz is None or frame_hz <= 0.0:
        return 0.0
    t_f = 1.0 / frame_hz
    waits = []
    for n in range(n_frames):
        te = phase_s % t_f + n * t_f
        k = math.floor((te - delay_s + EPS_T) / dt_c)       # newest command available at or before the edge
        waits.append(max(te - (k * dt_c + delay_s), 0.0))
    return sum(waits) / len(waits)


@dataclass
class TimingSummary:
    """Numbers quoted in the report for one timing configuration."""

    loop_hz: float
    frame_hz: float
    compute_delay_s: float
    mean_wait_s: float
    zoh_equiv_delay_s: float
    pulse_decode_s: float
    total_equiv_delay_s: float
    notes: list[str] = field(default_factory=list)


def summarize(dt_c: float, timing: FCTiming, frame_hz: float, phase_s: float, pulse_center_s: float) -> TimingSummary:
    zoh = frame_hz is not None and frame_hz > 0.0
    comp = timing.delay_s(dt_c)
    wait = mean_wait_after_available(dt_c, frame_hz, phase_s, comp) if zoh else 0.0
    hold = 0.5 / frame_hz if zoh else 0.5 * dt_c      # ZOH equivalent delay T/2 (frame, or the control tick itself)
    pulse = pulse_center_s if (zoh and timing.pulse_decode) else 0.0
    return TimingSummary(1.0 / dt_c, frame_hz if zoh else 0.0, comp, wait, hold, pulse, comp + wait + hold + pulse)
