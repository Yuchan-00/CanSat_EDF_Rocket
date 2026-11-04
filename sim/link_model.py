"""Latency and loss model of the LoRa ground link (SPEC §10.3) for the simulation
and for the ground station's timeouts.  Pure Python (no numpy).

Two backends (``flight_computer.link.radio``):

``lora_uart`` - Ebyte E22 / E220 / E32-class module in transparent mode on both
ends.  One hop (sender UART -> module -> air -> module -> receiver UART)::

    t_arrival = [ground serial latency]                         (PC side only, ASSUMED)
              + n_bytes x 10 / uart_baud                        sender -> module (8N1: 10 bits per byte)
              + EBYTE_IDLE_GAP_BYTES_ASSUMED x 10 / uart_baud   the module ends a packet after this UART silence
              + EBYTE_MODULE_PROCESS_S_ASSUMED                  module TX start + RX output processing
              + (8 n_bytes + EBYTE_AIR_OVERHEAD_BITS_ASSUMED) / lora_uart_air_rate_bps   on air
              + n_bytes x 10 / uart_baud                        module -> receiver
              + [ground serial latency]

  Ebyte does not publish the LoRa SF / BW behind its "air data rate" settings,
  so the on-air time is modelled as the frame bits at the nominal air rate plus
  an ASSUMED fixed overhead of 160 bits (preamble + sync + PHY header + CRC,
  about 20 LoRa symbols at ~8 bits per symbol).  Measure the round trip with the
  real modules (ground station log: request -> reply) and replace these numbers.

``sx127x`` - SX1276/77/78 driven over SPI by the flight computer.  On air:
Semtech AN1200.13 time-on-air (``lora_time_on_air``) with ``link.sx127x``
SF, BW, CR, preamble, explicit header, payload CRC on, low-data-rate
optimisation when T_sym > 16 ms.  Per hop: ``SX127X_HOP_OVERHEAD_S_ASSUMED``
(FIFO load over SPI, TX ramp, DIO0 service).  The PC reaches the ground SX127x
through a USB-UART bridge at ``link.uart_baud`` (ASSUMED ground hardware).

On the vehicle (v0.6.3: the COM ATmega128A drives the radio, the FCU runs the
supervisor): a received uplink frame reaches the FCU over the ilink after
``COM_FORWARD_S`` and is handled at the FCU's first control tick after that;
the TELEM reply is handed to the vehicle radio ``FC_REPLY_DELAY_S`` after the
uplink frame arrived (measured in the three-chip co-simulation: COM forwarding,
wait for the FCU tick, the FCU's reply snapshot over the ilink, TELEM build).
The model uses one conservative constant (the worst normal case), so the mean
and the worst round trip are the same number.  The ground's reply timeout is
built on ``FC_REPLY_ALLOWANCE_S`` instead - the latest reply the vehicle's COM
may still legitimately send (its reply window was derived from it) - so the
half-duplex discipline does not depend on the typical reply time.

Loss: every frame is lost independently with probability ``link.sim_packet_loss``
(``random.Random(seed)``, one draw per frame in transmission order, so a run is
reproducible); frames whose air time overlaps an outage window are lost; two
frames of opposite direction whose air times overlap both collide (half-duplex
radio) - the ground scheduler is meant to make that impossible, the model
counts it.

Reply frame size (v0.7.1): ``round_trip`` / ``budget`` take ``mode`` or ``telem_bytes``; the default is the
ballistic TELEM v1 frame (36 B, the v0.6.3 numbers bit for bit - a ballistic run must not change because
params.json now defaults to mission.mode 'vtol'); mode='vtol' sizes the downlink for the 39-byte TELEM2.  The
on-vehicle reply delays (FC_REPLY_DELAY_S, FC_REPLY_ALLOWANCE_S) are the v0.6.3 co-simulation's; the VTOL
co-simulation has not re-measured them (the COM builds TELEM2 from SNAP2: 3 more bytes on the COM's UART work).
v0.8: ``gps=True`` with mode='vtol' sizes it for the 45-byte TELEM3 (VTOL + GPS position hold; SNAP3 62 B on the
ilink) - also not re-measured in the co-simulation yet.
v0.9 (firmware round RP1): ``power=True`` with mode='vtol' sizes it for the power builds' frames (tether / hybrid:
TELEM v4 41 B, with ``gps`` TELEM v5 47 B; ``sim.link_proto`` TYPE_TELEM4 / TYPE_TELEM5, the power tail of
``sim.power_link``); the battery builds keep TELEM v2 / v3 (``power`` False, the default).

Invariant: every number not in params.json is a module constant whose name ends
in ``_ASSUMED`` (or is the 8N1 framing fact, one of the two on-vehicle delays
measured in the co-simulation, or the reply allowance shared with the COM
firmware - their evidence is in their comments), and ``budget()`` reports all
of them.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

try:
    from sim import link_proto as lp
except ImportError:  # pragma: no cover
    import link_proto as lp  # type: ignore

UART_BITS_PER_BYTE = 10                     # 8N1: start + 8 data + stop (framing fact, not an assumption)
EBYTE_AIR_OVERHEAD_BITS_ASSUMED = 160.0     # preamble + sync + header + CRC at the nominal air rate (~20 symbols)
EBYTE_IDLE_GAP_BYTES_ASSUMED = 3.0          # transparent mode: packet ends after 3 byte-times of UART silence
EBYTE_MODULE_PROCESS_S_ASSUMED = 0.005      # per hop: module TX start + RX output latency
SX127X_HOP_OVERHEAD_S_ASSUMED = 0.001       # per hop: SPI FIFO load, TX ramp-up, DIO0 interrupt service
GROUND_SERIAL_LATENCY_S_ASSUMED = 0.004     # PC USB-serial driver latency, each direction (FTDI latency timer set low)
# On-vehicle reply path, v0.6.3 three-chip co-simulation (firmware/atmega128_fc/tools/run_cosim.py ->
# host_test/build/cosim_report.json, normal.by_backend.<radio>.reply_uplink_to_telem_us_distribution, ~1,000 uplinks per
# backend): uplink frame complete at the COM's radio driver -> TELEM handed to the vehicle radio, normal flight
# max 12.534 ms (lora_uart) / 13.172 ms (sx127x), p50 10.7 / 9.4 ms (2026-10-16 run on params 0a5c261b; the
# 2026-10-15 run gave 12.649 / 13.135 ms) - the spread is the wait for the FCU tick.  Model value = the worst normal case rounded up to the next ms (not the mean).  Not modelled: a
# burst of uplinks queued at the COM (reply_delay_us_burst: up to 38.7 ms in the 2026-10-16 report, 43.5 ms in an
# earlier v0.6.3 run) - the reply timeout's REPLY_TIMEOUT_MARGIN_S_ASSUMED (40 ms) still covers 43.5 - 14 = 29.5 ms of it.
# (v0.5 single chip: FC_REPLY_DELAY_TICKS = 16 control ticks, 64 ms + up to one tick.)
FC_REPLY_DELAY_S = 0.014
# Uplink side, same report: uplink complete -> IL_CMD queued at the FCU (reply_delay_us_normal.up_to_q_us) max
# 0.300 ms (lora_uart) / 0.517 ms (sx127x), rounded up; the supervisor takes it at the FCU's first tick after that
# (il_cmd_latency_ticks = 0 in every normal run).
COM_FORWARD_S = 0.0006
# The ground's reply timeout is NOT built on FC_REPLY_DELAY_S but on this allowance: the latest on-vehicle reply the
# ground waits for before it may transmit again (half-duplex).  The COM's reply gate lets the vehicle answer up to
# firmware src/com/com_link.h COM_REPLY_WINDOW_MS = 80 ms after the uplink (+ the radio's start), a window derived
# from this allowance + REPLY_TIMEOUT_MARGIN_S_ASSUMED (h + start <= 108 ms) and re-checked against this module by
# firmware/atmega128_fc/tools/run_com_simbench.py; the measured bursts (<= 43.5 ms) fit too.  A timeout built on the
# 14 ms above (54 ms of allowance) would let a legitimate late reply collide with the ground's next request - which,
# since v0.6.3, can be an assist CUT / RELEASE sent at the timeout itself.  Shrink it only together with the COM window.
# (= the v0.5 single-chip allowance, FC_REPLY_DELAY_TICKS + 1 = 17 ticks at 250 Hz.)
FC_REPLY_ALLOWANCE_S = 0.068
# The ground gives a request up after the worst-case round trip + a jitter margin.  Keep it tight: after one lost
# uplink frame the vehicle sees no frame for ~ (poll interval + timeout), which must stay below link_loss_disarm_s.
REPLY_TIMEOUT_FACTOR_ASSUMED = 1.0
REPLY_TIMEOUT_MARGIN_S_ASSUMED = 0.04       # module latency jitter allowance (measure on the real modules)
RADIOS = ("lora_uart", "sx127x")


def uart_time_s(n_bytes: int, baud: float) -> float:
    return n_bytes * UART_BITS_PER_BYTE / float(baud)


def lora_symbols(payload_len: int, sf: int, cr_denom: int, explicit_header: bool = True, crc: bool = True,
                 ldro: bool = False) -> int:
    """Number of payload symbols, Semtech AN1200.13 / SX1276 datasheet §4.1.1.6:
    8 + max(ceil((8 PL - 4 SF + 28 + 16 CRC - 20 IH) / (4 (SF - 2 DE))) (CR + 4), 0), CR = cr_denom - 4."""
    if not 6 <= sf <= 12:
        raise ValueError(f"spreading factor {sf} outside 6..12")
    if not 5 <= cr_denom <= 8:
        raise ValueError(f"coding rate 4/{cr_denom} outside 4/5..4/8")
    ih = 0 if explicit_header else 1
    de = 1 if ldro else 0
    num = 8 * payload_len - 4 * sf + 28 + 16 * (1 if crc else 0) - 20 * ih
    den = 4 * (sf - 2 * de)
    return 8 + max(math.ceil(num / den) * cr_denom, 0)


def lora_time_on_air(payload_len: int, sf: int, bw_hz: float, cr_denom: int, preamble: int,
                     explicit_header: bool = True, crc: bool = True, ldro: bool | None = None) -> float:
    """LoRa packet time on air [s] (AN1200.13): (n_preamble + 4.25) T_sym + n_payload T_sym, T_sym = 2^SF / BW.
    ``ldro`` None -> on when T_sym > 16 ms (the SX127x rule)."""
    t_sym = (2 ** sf) / float(bw_hz)
    if ldro is None:
        ldro = t_sym > 0.016
    return (preamble + 4.25) * t_sym + lora_symbols(payload_len, sf, cr_denom, explicit_header, crc, ldro) * t_sym


@dataclass(frozen=True)
class LinkConfig:
    radio: str
    uart_baud: float
    air_rate_bps: float
    sf: int
    bw_hz: float
    cr_denom: int
    preamble: int
    loss: float
    poll_hz: float
    loop_hz: float

    @classmethod
    def from_params(cls, p: dict, radio: str | None = None, loss: float | None = None) -> "LinkConfig":
        lk = p["flight_computer"]["link"]
        sx = lk["sx127x"]
        r = radio or lk["radio"]
        if r not in RADIOS:
            raise ValueError(f"link.radio {r!r} not in {RADIOS}")
        return cls(radio=r, uart_baud=float(lk["uart_baud"]), air_rate_bps=float(lk["lora_uart_air_rate_bps"]),
                   sf=int(sx["spreading_factor"]), bw_hz=float(sx["bandwidth_hz"]), cr_denom=int(sx["coding_rate_denom"]),
                   preamble=int(sx["preamble_symbols"]), loss=float(lk["sim_packet_loss"] if loss is None else loss),
                   poll_hz=float(lk["poll_hz"]), loop_hz=float(p["control"]["loop_hz"]))


@dataclass(frozen=True)
class HopTiming:
    """Offsets from the moment the sender starts writing the frame [s]."""

    air_start: float
    air_end: float
    arrival: float
    parts: dict


def hop_timing(cfg: LinkConfig, direction: str, n_bytes: int) -> HopTiming:
    """``direction`` 'up' (ground -> vehicle) or 'down' (vehicle -> ground)."""
    if direction not in ("up", "down"):
        raise ValueError(direction)
    pc = GROUND_SERIAL_LATENCY_S_ASSUMED
    parts: dict[str, float] = {}
    if cfg.radio == "lora_uart":
        u = uart_time_s(n_bytes, cfg.uart_baud)
        parts["pc_serial_tx"] = pc if direction == "up" else 0.0
        parts["uart_to_module"] = u
        parts["idle_gap"] = EBYTE_IDLE_GAP_BYTES_ASSUMED * UART_BITS_PER_BYTE / cfg.uart_baud
        parts["module_process"] = EBYTE_MODULE_PROCESS_S_ASSUMED
        parts["air"] = (8.0 * n_bytes + EBYTE_AIR_OVERHEAD_BITS_ASSUMED) / cfg.air_rate_bps
        parts["uart_from_module"] = u
        parts["pc_serial_rx"] = pc if direction == "down" else 0.0
        pre = parts["pc_serial_tx"] + parts["uart_to_module"] + parts["idle_gap"] + parts["module_process"]
        post = parts["uart_from_module"] + parts["pc_serial_rx"]
    else:
        u = uart_time_s(n_bytes, cfg.uart_baud)
        parts["pc_serial_tx"] = pc if direction == "up" else 0.0
        parts["bridge_uart_tx"] = u if direction == "up" else 0.0
        parts["radio_overhead"] = SX127X_HOP_OVERHEAD_S_ASSUMED
        parts["air"] = lora_time_on_air(n_bytes, cfg.sf, cfg.bw_hz, cfg.cr_denom, cfg.preamble)
        parts["bridge_uart_rx"] = u if direction == "down" else 0.0
        parts["pc_serial_rx"] = pc if direction == "down" else 0.0
        pre = parts["pc_serial_tx"] + parts["bridge_uart_tx"] + parts["radio_overhead"]
        post = parts["bridge_uart_rx"] + parts["pc_serial_rx"]
    air = parts["air"]
    return HopTiming(pre, pre + air, pre + air + post, parts)


TELEM_FRAME_SIZES = (lp.TELEM_FRAME_BYTES, lp.TELEM2_FRAME_BYTES, lp.TELEM3_FRAME_BYTES, lp.TELEM4_FRAME_BYTES,
                     lp.TELEM5_FRAME_BYTES)


def telem_frame_bytes(mode: str | None = None, telem_bytes: int | None = None, gps: bool = False,
                      power: bool = False) -> int:
    """The downlink reply frame of a build: ``telem_bytes`` if given, else TELEM v1 (36 B) for the ballistic build or
    no mode (the v0.6.3 default every existing caller keeps), TELEM v2 (``lp.TELEM2_FRAME_BYTES`` = 39 B) for 'vtol',
    TELEM v3 (``lp.TELEM3_FRAME_BYTES`` = 45 B) for 'vtol' with ``gps`` (v0.8 position hold); v0.9 RP1 ``power`` (the
    tether / hybrid builds, 'vtol' only): TELEM v4 (41 B) / with ``gps`` TELEM v5 (47 B).
    Deliberately NOT params.json mission.mode: a ballistic run under a vtol params file must keep its v0.6.3 timing."""
    if telem_bytes is not None:
        if int(telem_bytes) not in TELEM_FRAME_SIZES:
            raise ValueError(f"telem_bytes {telem_bytes} is neither TELEM ({lp.TELEM_FRAME_BYTES}), TELEM2 "
                             f"({lp.TELEM2_FRAME_BYTES}), TELEM3 ({lp.TELEM3_FRAME_BYTES}), TELEM4 "
                             f"({lp.TELEM4_FRAME_BYTES}) nor TELEM5 ({lp.TELEM5_FRAME_BYTES})")
        return int(telem_bytes)
    if mode is None or lp.check_mode(mode) == lp.MODE_BALLISTIC:
        if power:
            raise ValueError("the power builds (TELEM v4 / v5) are VTOL builds (mode 'vtol')")
        return lp.TELEM_FRAME_BYTES
    if power:
        return lp.TELEM5_FRAME_BYTES if gps else lp.TELEM4_FRAME_BYTES
    return lp.TELEM3_FRAME_BYTES if gps else lp.TELEM2_FRAME_BYTES


def round_trip(cfg: LinkConfig, telem_bytes: int | None = None, mode: str | None = None,
               gps: bool = False, power: bool = False) -> dict[str, float]:
    """Request (CMD frame) -> reply (TELEM frame) at the ground: uplink + the on-vehicle reply delay + downlink.
    The reply delay is the measured worst normal case, so the model's mean and worst round trip are equal (the
    FCU tick phase is inside the measured number).  The reply frame is ``telem_frame_bytes(mode, telem_bytes)``:
    v1 36 B by default (ballistic, v0.6.3 numbers bit for bit), the VTOL build's 39-byte TELEM2 with mode='vtol'
    (v0.7.1; +3 bytes on the downlink: lora_uart +16.2 ms, sx127x +8.2 ms of time on air at the params settings);
    with ``gps`` the 45-byte TELEM3 (v0.8); v0.9 RP1 ``power``: TELEM4 41 B / TELEM5 47 B (tether / hybrid builds)."""
    nb = telem_frame_bytes(mode, telem_bytes, gps, power)
    up = hop_timing(cfg, "up", lp.CMD_FRAME_BYTES).arrival
    down = hop_timing(cfg, "down", nb).arrival
    worst = up + FC_REPLY_DELAY_S + down
    allowed = up + FC_REPLY_ALLOWANCE_S + down          # the latest reply the ground must still wait for
    return {"uplink_s": up, "downlink_s": down, "fc_reply_delay_s": FC_REPLY_DELAY_S, "com_forward_s": COM_FORWARD_S,
            "fc_reply_allowance_s": FC_REPLY_ALLOWANCE_S, "rtt_mean_s": worst, "rtt_worst_s": worst,
            "rtt_allowed_s": allowed,
            "reply_timeout_s": REPLY_TIMEOUT_FACTOR_ASSUMED * allowed + REPLY_TIMEOUT_MARGIN_S_ASSUMED,
            "max_poll_hz": 1.0 / worst}


def budget(cfg: LinkConfig, telem_bytes: int | None = None, mode: str | None = None, gps: bool = False,
           power: bool = False) -> dict:
    """Everything the report / the firmware generator prints about the link timing, for the reply frame
    ``telem_frame_bytes(mode, telem_bytes, gps, power)`` (default: the ballistic TELEM v1; mode='vtol': TELEM2, 39 B; with
    ``gps``: TELEM3, 45 B; v0.9 ``power``: TELEM4 41 B / TELEM5 47 B)."""
    nb = telem_frame_bytes(mode, telem_bytes, gps, power)
    rt = round_trip(cfg, telem_bytes=nb)
    return {
        "radio": cfg.radio, "uart_baud": cfg.uart_baud, "poll_hz": cfg.poll_hz, "loss": cfg.loss,
        "cmd_frame_bytes": lp.CMD_FRAME_BYTES, "telem_frame_bytes": nb,
        "uplink": hop_timing(cfg, "up", lp.CMD_FRAME_BYTES).parts,
        "downlink": hop_timing(cfg, "down", nb).parts,
        **rt,
        "poll_hz_achievable": rt["max_poll_hz"] >= cfg.poll_hz,
        "measured": {
            "FC_REPLY_DELAY_S": FC_REPLY_DELAY_S,
            "COM_FORWARD_S": COM_FORWARD_S,
            "source": "v0.6.3 three-chip co-simulation, firmware/atmega128_fc/host_test/build/cosim_report.json "
                      "(normal flight: uplink at the COM radio driver -> TELEM handed to the radio max 12.649 ms "
                      "lora_uart / 13.135 ms sx127x, rounded up; uplink -> IL_CMD at the FCU max 0.517 ms)",
        },
        "assumed": {
            "EBYTE_AIR_OVERHEAD_BITS_ASSUMED": EBYTE_AIR_OVERHEAD_BITS_ASSUMED,
            "EBYTE_IDLE_GAP_BYTES_ASSUMED": EBYTE_IDLE_GAP_BYTES_ASSUMED,
            "EBYTE_MODULE_PROCESS_S_ASSUMED": EBYTE_MODULE_PROCESS_S_ASSUMED,
            "SX127X_HOP_OVERHEAD_S_ASSUMED": SX127X_HOP_OVERHEAD_S_ASSUMED,
            "GROUND_SERIAL_LATENCY_S_ASSUMED": GROUND_SERIAL_LATENCY_S_ASSUMED,
            "REPLY_TIMEOUT_FACTOR_ASSUMED": REPLY_TIMEOUT_FACTOR_ASSUMED,
            "REPLY_TIMEOUT_MARGIN_S_ASSUMED": REPLY_TIMEOUT_MARGIN_S_ASSUMED,
        },
        "sx127x": {"sf": cfg.sf, "bw_hz": cfg.bw_hz, "cr": f"4/{cfg.cr_denom}", "preamble": cfg.preamble,
                   "toa_cmd_s": lora_time_on_air(lp.CMD_FRAME_BYTES, cfg.sf, cfg.bw_hz, cfg.cr_denom, cfg.preamble),
                   "toa_telem_s": lora_time_on_air(nb, cfg.sf, cfg.bw_hz, cfg.cr_denom, cfg.preamble)},
    }


@dataclass
class Transit:
    direction: str
    data: bytes
    t_tx: float
    t_air_start: float
    t_air_end: float
    t_arrival: float
    lost: str = ""            # "", "random", "outage", "collision"


@dataclass
class LinkModel:
    """Stateful link: ``send`` a frame, ``deliver`` the ones that have arrived by now."""

    cfg: LinkConfig
    seed: int = 0
    outages: list[tuple[float, float]] = field(default_factory=list)   # [t0, t1) windows of total loss

    def __post_init__(self) -> None:
        self.rng = random.Random(self.seed)
        self.in_flight: list[Transit] = []
        self.log: list[Transit] = []
        self.stats = {"up_sent": 0, "down_sent": 0, "up_lost": 0, "down_lost": 0, "random": 0, "outage": 0,
                      "collision": 0}

    def _in_outage(self, a: float, b: float) -> bool:
        return any(a < t1 and b > t0 for (t0, t1) in self.outages)

    def send(self, direction: str, data: bytes, t_tx: float) -> Transit:
        h = hop_timing(self.cfg, direction, len(data))
        tr = Transit(direction, bytes(data), t_tx, t_tx + h.air_start, t_tx + h.air_end, t_tx + h.arrival)
        self.stats[f"{direction}_sent"] += 1
        if self.rng.random() < self.cfg.loss:        # always one draw per frame (reproducible sequence)
            tr.lost = "random"
        elif self._in_outage(tr.t_air_start, tr.t_air_end):
            tr.lost = "outage"
        for o in self.in_flight:
            if o.direction != direction and o.t_air_start < tr.t_air_end and tr.t_air_start < o.t_air_end:
                if not o.lost:
                    o.lost = "collision"
                if not tr.lost:
                    tr.lost = "collision"
        self.in_flight.append(tr)
        self.log.append(tr)
        return tr

    def deliver(self, direction: str, t_now: float) -> list[Transit]:
        """Frames of ``direction`` that arrived at or before ``t_now`` (lost ones are discarded and counted)."""
        out, keep = [], []
        for tr in self.in_flight:
            if tr.direction == direction and tr.t_arrival <= t_now + 1e-12:
                if tr.lost:
                    self.stats[f"{direction}_lost"] += 1
                    self.stats[tr.lost] += 1
                else:
                    out.append(tr)
            else:
                keep.append(tr)
        self.in_flight = keep
        out.sort(key=lambda x: x.t_arrival)
        return out
