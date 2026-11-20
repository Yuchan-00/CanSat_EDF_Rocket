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
   chip (1 = the next tick, 4.0 ms).  Every channel (vanes and ESC) uses the one ``delay_s``.
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


@dataclass
class FCTiming:
    """Timing options of one simulation run (defaults = the ATmega128 baseline)."""

    compute_delay_ticks: int | None = None  # integer form (v0.5: 1 = next tick); used when compute_delay_s is None
    frame_hz: float | None = None           # None -> params.flight_computer.servo_frame_hz; <= 0 -> no frame latch
    phase_s: float | None = None            # first Timer1 frame edge after t = 0; None -> half a control period
    timer3_phase_s: float = 0.0             # extra offset of the Timer3 channels (vane 4, ESC) vs Timer1
    pulse_decode: bool = True               # servo/ESC act at the falling edge of their pulse
    compute_delay_s: float | None = None    # sample -> output register [s]; None -> ticks above, else the measured max

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
