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
2026-11-10 control round (logic-battery telemetry): ``lbat=True`` with mode='vtol' (the VTOL battery build on the
carrier rev 2c, ``logic_tail_build``) sizes it for TELEM v6 40 B / with ``gps`` TELEM v7 46 B (TELEM v2 / v3 + the
1-byte logic-battery tail, ``sim.link_proto`` TYPE_TELEM6 / TYPE_TELEM7).

``esp_bridge`` (2026-11-10 control round) - the ESP8266 Wi-Fi bridge of ``firmware/esp_bridge`` on both ends: a
transparent UART <-> ESP-NOW bridge at ``link.esp_bridge.uart_baud`` (38400 8N1 on the COM, both ESPs and the ground PC)
and ESP-NOW unicast on 802.11b at ``link.esp_bridge.air_rate_bps`` (1 Mbps, A).  One hop::

    t_arrival = [ground serial latency]                              (PC side only, ASSUMED, as for LoRa)
              + n_bytes x 10 / esp_uart_baud                         sender -> its ESP (the bridge sends a complete
                                                                     frame when its last byte arrives: README "하는 일")
              + ESP_HOP_PROCESS_S_ASSUMED                            UART RX FIFO timeout, bridge loop, esp_now_send,
                                                                     the receiver's callback -> UART write (both ESPs)
              + DIFS + CW_min / 2 slots                              802.11b DCF channel access (mean backoff)
              + PLCP + (ESPNOW_FRAME_OVERHEAD_BYTES_ASSUMED + n) x 8 / air_rate     on air
              + n_bytes x 10 / esp_uart_baud                         its ESP -> receiver (the whole packet, then UART)
              + [ground serial latency]

  plus, per MAC retry (unicast: the receiver ACKs, the sender retries up to ``ESP_MAC_ATTEMPTS_ASSUMED`` attempts),
  ACK timeout + DIFS + CW_k / 2 slots (CW doubling from CW_min to CW_max) + the frame again.  The nominal (no retry) hop
  gives ``rtt_mean_s``; every attempt used gives ``rtt_worst_s`` and the reply timeout (the latest delivered reply).
  ``LinkModel`` draws the retries of each delivered frame from an independent-attempt channel whose per-attempt failure
  probability gives the frame loss ``link.sim_packet_loss`` after the last attempt (p_att = loss^(1 / attempts), A),
  on its own random stream (the loss draws keep the LoRa sequence: one draw per frame); ESP-NOW frames do not collide
  (CSMA/CA defers instead - the two stations hear each other, E).  The 802.11b DSSS timing constants are standard
  values (IEEE 802.11 clause 16 / 17, E: the standard is not on disk); the ESP8266 SDK's retry limit, its processing
  time and the ESP-NOW frame layout are ASSUMED (Espressif documentation not on disk) - measure the real round trip
  (ground station log: request -> reply) and replace them.

  What the bridge design fixes, and what the model takes from it (``firmware/esp_bridge/README.md`` "하는 일" /
  "송신 순서와 진단", ``firmware/esp_bridge/src/esp_bridge_config_gen.h``):
  * 38400 8N1 on both UARTs (COM <-> vehicle ESP, ground ESP <-> PC): 10 / 38400 s per byte, a CMD (14 B) 3.6 ms, a
    TELEM v7 (46 B) 12.0 ms per UART - the largest part of a hop (design fact; 38400 = UBRR 25 at 16 MHz, +0.16 %).
  * the chunker's flush timeouts are NOT in a normal hop: a complete frame leaves at its last byte.  Only bytes that
    are no frame wait for the idle flush (BRIDGE_IDLE_FLUSH_US 781 us = 3 byte times) and a frame cut short for
    BRIDGE_FRAME_WAIT_US 13 021 us; a frame damaged on a UART therefore arrives up to 13.0 ms late and fails the CRC -
    the model counts it as lost (``link.sim_packet_loss``), not as late (E).
  * one ESP-NOW packet in flight per ESP (the next waits for the send callback, at most 30 ms; 50 ms of refusals drop
    it): with 2 Hz half-duplex polling each ESP sends one packet per exchange, so the gate adds nothing here (E).  A
    burst of requests would queue behind it - not modelled (the COM answers only the newest one anyway).
  * on-vehicle part = the LoRa models' measured constants: FC_REPLY_DELAY_S 14 ms and COM_FORWARD_S 0.6 ms.  The COM
    bench with the ESP backend (``firmware/atmega128_fc/tools/run_com_simbench.py`` com_simbench_esp /
    com_simbench_esp_ballistic, simavr, M): uplink complete -> IL_CMD queued <= 0.27 ms (0.24 ms in the 2026-11-06
    run), uplink complete -> TELEM at the UART <= 7.9 ms with the synthetic FCU, no busy line (the reply goes to the
    UART at once) - inside those constants and inside FC_REPLY_ALLOWANCE_S, which the bench re-checks against this
    model with every MAC attempt on both hops.
  E / A summary: UART times are design facts; 802.11b frame and DCF times are E (standard values); bridge processing,
  the ESP-NOW overhead bytes, the SDK retry limit and the per-attempt loss model are A; nothing is measured on a
  real ESP pair yet.

Model selection (2026-11-10): ``model_radio(p)`` = ``flight_computer.link.vehicle_radio`` when this module has a
model for it (``RADIOS``: the ESP bridge now), else ``link.radio`` (the LoRa fallback model).  ``LinkConfig.from_params``
without ``radio`` uses it, so the simulation, the ground station's reply timeout and the firmware generator's link
budget follow the radio the COM is built for; ``link.radio`` stays the LoRa fallback's model.

Invariant: every number not in params.json is a module constant whose name ends
in ``_ASSUMED`` (or is the 8N1 framing fact, an 802.11b DSSS standard value, one of the
two on-vehicle delays measured in the co-simulation, or the reply allowance shared with
the COM firmware - their evidence is in their comments), and ``budget()`` reports all
of them.  The lora_uart / sx127x numbers are bit for bit those of the v0.9 model.
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
RADIOS = ("lora_uart", "sx127x", "esp_bridge")
LORA_RADIOS = ("lora_uart", "sx127x")       # the LoRa fallback models (link.radio)
# ---- esp_bridge (2026-11-10): ESP8266 UART <-> ESP-NOW bridge on both ends (firmware/esp_bridge) ----
# per hop, both ESPs together: the sender's UART RX FIFO timeout interrupt (ESP8266 Arduino core: a few byte times),
# the bridge loop (bridge_core: a complete frame is sent at its last byte), esp_now_send -> MAC, and the receiver's
# recv callback -> UART write start.  Nothing measured (no ESP board on the bench) - A.
ESP_HOP_PROCESS_S_ASSUMED = 0.001
# ESP-NOW vendor-specific action frame around the payload: MAC header 24 + category 1 + OUI 3 + random 4 + vendor element
# (id 1, length 1, OUI 3, type 1, version 1) + FCS 4 = 43 bytes (Espressif ESP-NOW frame format, not on disk - A)
ESPNOW_FRAME_OVERHEAD_BYTES_ASSUMED = 43
# unicast ESP-NOW: attempts per frame before the SDK reports a failure (1 + the 802.11 dot11ShortRetryLimit 7; the
# ESP8266 SDK's value is not documented on disk - A).  firmware/esp_bridge then counts the packet as lost.
ESP_MAC_ATTEMPTS_ASSUMED = 8
# 802.11b DSSS (1 Mbps, long preamble) timing - standard values (IEEE 802.11 clauses 16 / 17; E: not on disk)
DSSS_PLCP_S = 192e-6                        # long PLCP preamble 144 us + PLCP header 48 us
DSSS_SLOT_S = 20e-6
DSSS_SIFS_S = 10e-6
DSSS_DIFS_S = DSSS_SIFS_S + 2 * DSSS_SLOT_S  # 50 us
DSSS_CW_MIN = 31
DSSS_CW_MAX = 1023
DSSS_ACK_BYTES = 14
DSSS_ACK_TIMEOUT_S = DSSS_SIFS_S + DSSS_SLOT_S + DSSS_PLCP_S   # no ACK started: the sender gives the attempt up


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


def model_radio(p: dict) -> str:
    """The link model of this params file: ``flight_computer.link.vehicle_radio`` (the COM's backend) when ``RADIOS``
    has a model for it, else ``link.radio`` (the LoRa fallback model; also the v0.9 files without vehicle_radio)."""
    lk = p["flight_computer"]["link"]
    v = lk.get("vehicle_radio")
    return str(v) if v in RADIOS else str(lk["radio"])


def esp_hop_extra_s(n_bytes: int, air_rate_bps: float, retries: int) -> float:
    """802.11b DCF time added by ``retries`` MAC retries of one ESP-NOW frame (each: ACK timeout, DIFS, the mean backoff
    of the doubled contention window, the frame again); 0 for none."""
    air = DSSS_PLCP_S + 8.0 * (ESPNOW_FRAME_OVERHEAD_BYTES_ASSUMED + n_bytes) / float(air_rate_bps)
    t = 0.0
    for k in range(1, int(retries) + 1):
        cw = min((DSSS_CW_MIN + 1) * (2 ** k) - 1, DSSS_CW_MAX)
        t += DSSS_ACK_TIMEOUT_S + DSSS_DIFS_S + 0.5 * cw * DSSS_SLOT_S + air
    return t


def esp_attempt_fail(loss: float) -> float:
    """Per-attempt failure probability of the independent-attempt ESP-NOW channel whose frame loss after
    ``ESP_MAC_ATTEMPTS_ASSUMED`` attempts is ``loss`` (A)."""
    return float(loss) ** (1.0 / ESP_MAC_ATTEMPTS_ASSUMED) if loss > 0.0 else 0.0


def esp_retry_pmf(loss: float) -> list[float]:
    """P(k MAC retries | the frame was delivered), k = 0 .. ESP_MAC_ATTEMPTS_ASSUMED - 1 (truncated geometric)."""
    q = esp_attempt_fail(loss)
    w = [(q ** k) * (1.0 - q) for k in range(ESP_MAC_ATTEMPTS_ASSUMED)]
    s = sum(w)
    return [x / s for x in w] if s > 0.0 else [1.0] + [0.0] * (ESP_MAC_ATTEMPTS_ASSUMED - 1)


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
    esp_uart_baud: float = 0.0              # link.esp_bridge.uart_baud (esp_bridge only; 0 = not configured)
    esp_air_rate_bps: float = 0.0           # link.esp_bridge.air_rate_bps (esp_bridge only; 0 = not configured)

    @classmethod
    def from_params(cls, p: dict, radio: str | None = None, loss: float | None = None) -> "LinkConfig":
        """``radio`` None: ``model_radio(p)`` (link.vehicle_radio when modelled, else link.radio)."""
        lk = p["flight_computer"]["link"]
        sx = lk["sx127x"]
        r = radio or model_radio(p)
        if r not in RADIOS:
            raise ValueError(f"link.radio {r!r} not in {RADIOS}")
        esp = lk.get("esp_bridge") or {}
        if r == "esp_bridge" and not ("uart_baud" in esp and "air_rate_bps" in esp):
            raise ValueError("the esp_bridge link model needs flight_computer.link.esp_bridge.uart_baud / air_rate_bps")
        return cls(radio=r, uart_baud=float(lk["uart_baud"]), air_rate_bps=float(lk["lora_uart_air_rate_bps"]),
                   sf=int(sx["spreading_factor"]), bw_hz=float(sx["bandwidth_hz"]), cr_denom=int(sx["coding_rate_denom"]),
                   preamble=int(sx["preamble_symbols"]), loss=float(lk["sim_packet_loss"] if loss is None else loss),
                   poll_hz=float(lk["poll_hz"]), loop_hz=float(p["control"]["loop_hz"]),
                   esp_uart_baud=float(esp.get("uart_baud", 0.0)), esp_air_rate_bps=float(esp.get("air_rate_bps", 0.0)))


@dataclass(frozen=True)
class HopTiming:
    """Offsets from the moment the sender starts writing the frame [s]."""

    air_start: float
    air_end: float
    arrival: float
    parts: dict


def hop_timing(cfg: LinkConfig, direction: str, n_bytes: int, retries: int = 0) -> HopTiming:
    """``direction`` 'up' (ground -> vehicle) or 'down' (vehicle -> ground).  ``retries``: esp_bridge only, the MAC
    retries before the delivered attempt (0 = the nominal hop; ``ESP_MAC_ATTEMPTS_ASSUMED`` - 1 = the worst)."""
    if direction not in ("up", "down"):
        raise ValueError(direction)
    pc = GROUND_SERIAL_LATENCY_S_ASSUMED
    parts: dict[str, float] = {}
    if retries and cfg.radio != "esp_bridge":
        raise ValueError("MAC retries are modelled for esp_bridge only")
    if cfg.radio == "esp_bridge":
        if not 0 <= int(retries) < ESP_MAC_ATTEMPTS_ASSUMED:
            raise ValueError(f"retries {retries} outside 0..{ESP_MAC_ATTEMPTS_ASSUMED - 1}")
        u = uart_time_s(n_bytes, cfg.esp_uart_baud)
        parts["pc_serial_tx"] = pc if direction == "up" else 0.0
        parts["uart_to_esp"] = u
        parts["esp_process"] = ESP_HOP_PROCESS_S_ASSUMED
        parts["channel_access"] = DSSS_DIFS_S + 0.5 * DSSS_CW_MIN * DSSS_SLOT_S
        parts["mac_retries"] = esp_hop_extra_s(n_bytes, cfg.esp_air_rate_bps, int(retries))
        parts["air"] = DSSS_PLCP_S + 8.0 * (ESPNOW_FRAME_OVERHEAD_BYTES_ASSUMED + n_bytes) / cfg.esp_air_rate_bps
        parts["uart_from_esp"] = u
        parts["pc_serial_rx"] = pc if direction == "down" else 0.0
        pre = (parts["pc_serial_tx"] + parts["uart_to_esp"] + parts["esp_process"] + parts["channel_access"]
               + parts["mac_retries"])
        post = parts["uart_from_esp"] + parts["pc_serial_rx"]
    elif cfg.radio == "lora_uart":
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
                     lp.TELEM5_FRAME_BYTES, lp.TELEM6_FRAME_BYTES, lp.TELEM7_FRAME_BYTES)


def telem_frame_bytes(mode: str | None = None, telem_bytes: int | None = None, gps: bool = False,
                      power: bool = False, lbat: bool = False) -> int:
    """The downlink reply frame of a build: ``telem_bytes`` if given, else TELEM v1 (36 B) for the ballistic build or
    no mode (the v0.6.3 default every existing caller keeps), TELEM v2 (``lp.TELEM2_FRAME_BYTES`` = 39 B) for 'vtol',
    TELEM v3 (``lp.TELEM3_FRAME_BYTES`` = 45 B) for 'vtol' with ``gps`` (v0.8 position hold); v0.9 RP1 ``power`` (the
    tether / hybrid builds, 'vtol' only): TELEM v4 (41 B) / with ``gps`` TELEM v5 (47 B); 2026-11-10 ``lbat`` (the VTOL
    battery build on the carrier: the logic-battery tail): TELEM v6 (40 B) / with ``gps`` TELEM v7 (46 B).
    Deliberately NOT params.json mission.mode: a ballistic run under a vtol params file must keep its v0.6.3 timing."""
    if telem_bytes is not None:
        if int(telem_bytes) not in TELEM_FRAME_SIZES:
            raise ValueError(f"telem_bytes {telem_bytes} is none of the TELEM v1-v7 frames {TELEM_FRAME_SIZES}")
        return int(telem_bytes)
    if power and lbat:
        raise ValueError("a reply carries the power tail (TELEM v4 / v5) or the logic-battery tail (v6 / v7), not both")
    if mode is None or lp.check_mode(mode) == lp.MODE_BALLISTIC:
        if power or lbat:
            raise ValueError("the power / logic-battery builds (TELEM v4 - v7) are VTOL builds (mode 'vtol')")
        return lp.TELEM_FRAME_BYTES
    if power:
        return lp.TELEM5_FRAME_BYTES if gps else lp.TELEM4_FRAME_BYTES
    if lbat:
        return lp.TELEM7_FRAME_BYTES if gps else lp.TELEM6_FRAME_BYTES
    return lp.TELEM3_FRAME_BYTES if gps else lp.TELEM2_FRAME_BYTES


def round_trip(cfg: LinkConfig, telem_bytes: int | None = None, mode: str | None = None,
               gps: bool = False, power: bool = False, lbat: bool = False) -> dict[str, float]:
    """Request (CMD frame) -> reply (TELEM frame) at the ground: uplink + the on-vehicle reply delay + downlink.
    The reply delay is the measured worst normal case, so for the LoRa models the mean and worst round trip are equal
    (the FCU tick phase is inside the measured number).  esp_bridge: ``rtt_mean_s`` is the nominal exchange (no MAC
    retry), ``rtt_worst_s`` / the reply timeout use every MAC attempt on both hops (``ESP_MAC_ATTEMPTS_ASSUMED``).
    The reply frame is ``telem_frame_bytes(mode, telem_bytes, gps, power, lbat)``:
    v1 36 B by default (ballistic, v0.6.3 numbers bit for bit), the VTOL build's 39-byte TELEM2 with mode='vtol'
    (v0.7.1; +3 bytes on the downlink: lora_uart +16.2 ms, sx127x +8.2 ms of time on air at the params settings);
    with ``gps`` the 45-byte TELEM3 (v0.8); v0.9 RP1 ``power``: TELEM4 41 B / TELEM5 47 B (tether / hybrid builds);
    ``lbat``: TELEM6 40 B / TELEM7 46 B (the battery build with the logic-battery tail)."""
    return round_trip_bytes(cfg, telem_frame_bytes(mode, telem_bytes, gps, power, lbat))


def round_trip_bytes(cfg: LinkConfig, nb: int) -> dict[str, float]:
    """``round_trip`` for a reply frame of ``nb`` bytes without the TELEM-size check (studies of proposed frames:
    ``sim.power_link.telem4_budget``)."""
    up = hop_timing(cfg, "up", lp.CMD_FRAME_BYTES).arrival
    down = hop_timing(cfg, "down", nb).arrival
    mean = up + FC_REPLY_DELAY_S + down
    if cfg.radio == "esp_bridge":
        r = ESP_MAC_ATTEMPTS_ASSUMED - 1
        up_w = hop_timing(cfg, "up", lp.CMD_FRAME_BYTES, retries=r).arrival
        down_w = hop_timing(cfg, "down", nb, retries=r).arrival
    else:
        up_w, down_w = up, down
    worst = up_w + FC_REPLY_DELAY_S + down_w
    allowed = up_w + FC_REPLY_ALLOWANCE_S + down_w      # the latest reply the ground must still wait for
    out = {"uplink_s": up, "downlink_s": down, "fc_reply_delay_s": FC_REPLY_DELAY_S, "com_forward_s": COM_FORWARD_S,
           "fc_reply_allowance_s": FC_REPLY_ALLOWANCE_S, "rtt_mean_s": mean, "rtt_worst_s": worst,
           "rtt_allowed_s": allowed,
           "reply_timeout_s": REPLY_TIMEOUT_FACTOR_ASSUMED * allowed + REPLY_TIMEOUT_MARGIN_S_ASSUMED,
           "max_poll_hz": 1.0 / worst}
    if cfg.radio == "esp_bridge":
        out.update(uplink_worst_s=up_w, downlink_worst_s=down_w)
    return out


def budget(cfg: LinkConfig, telem_bytes: int | None = None, mode: str | None = None, gps: bool = False,
           power: bool = False, lbat: bool = False) -> dict:
    """Everything the report / the firmware generator prints about the link timing, for the reply frame
    ``telem_frame_bytes(mode, telem_bytes, gps, power, lbat)`` (default: the ballistic TELEM v1; mode='vtol': TELEM2,
    39 B; with ``gps``: TELEM3, 45 B; v0.9 ``power``: TELEM4 41 B / TELEM5 47 B; ``lbat``: TELEM6 40 B / TELEM7 46 B)."""
    nb = telem_frame_bytes(mode, telem_bytes, gps, power, lbat)
    rt = round_trip(cfg, telem_bytes=nb)
    if cfg.radio == "esp_bridge":
        r = ESP_MAC_ATTEMPTS_ASSUMED - 1
        pmf = esp_retry_pmf(cfg.loss)
        mean_extra = sum(pk * (esp_hop_extra_s(lp.CMD_FRAME_BYTES, cfg.esp_air_rate_bps, k)
                               + esp_hop_extra_s(nb, cfg.esp_air_rate_bps, k)) for k, pk in enumerate(pmf))
        return {
            "radio": cfg.radio, "uart_baud": cfg.esp_uart_baud, "poll_hz": cfg.poll_hz, "loss": cfg.loss,
            "cmd_frame_bytes": lp.CMD_FRAME_BYTES, "telem_frame_bytes": nb,
            "uplink": hop_timing(cfg, "up", lp.CMD_FRAME_BYTES).parts,
            "downlink": hop_timing(cfg, "down", nb).parts,
            "uplink_worst": hop_timing(cfg, "up", lp.CMD_FRAME_BYTES, retries=r).parts,
            "downlink_worst": hop_timing(cfg, "down", nb, retries=r).parts,
            **rt,
            "poll_hz_achievable": rt["max_poll_hz"] >= cfg.poll_hz,
            "esp_retry_pmf_at_loss": pmf, "esp_attempt_fail_at_loss": esp_attempt_fail(cfg.loss),
            "rtt_expected_at_loss_s": rt["rtt_mean_s"] + mean_extra,
            "measured": {
                "FC_REPLY_DELAY_S": FC_REPLY_DELAY_S,
                "COM_FORWARD_S": COM_FORWARD_S,
                "source": "v0.6.3 three-chip co-simulation (the on-vehicle reply path; the ESP backend hands the TELEM "
                          "to its UART at once - firmware/atmega128_fc/tools/run_com_simbench.py esp_bridge)",
            },
            "assumed": {
                "ESP_HOP_PROCESS_S_ASSUMED": ESP_HOP_PROCESS_S_ASSUMED,
                "ESPNOW_FRAME_OVERHEAD_BYTES_ASSUMED": ESPNOW_FRAME_OVERHEAD_BYTES_ASSUMED,
                "ESP_MAC_ATTEMPTS_ASSUMED": ESP_MAC_ATTEMPTS_ASSUMED,
                "GROUND_SERIAL_LATENCY_S_ASSUMED": GROUND_SERIAL_LATENCY_S_ASSUMED,
                "REPLY_TIMEOUT_FACTOR_ASSUMED": REPLY_TIMEOUT_FACTOR_ASSUMED,
                "REPLY_TIMEOUT_MARGIN_S_ASSUMED": REPLY_TIMEOUT_MARGIN_S_ASSUMED,
                "esp_air_rate_bps (params link.esp_bridge.air_rate_bps)": cfg.esp_air_rate_bps,
            },
            "standard_80211b_dsss": {"plcp_s": DSSS_PLCP_S, "slot_s": DSSS_SLOT_S, "sifs_s": DSSS_SIFS_S,
                                     "difs_s": DSSS_DIFS_S, "cw_min": DSSS_CW_MIN, "cw_max": DSSS_CW_MAX,
                                     "ack_timeout_s": DSSS_ACK_TIMEOUT_S},
        }
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
    retries: int = 0          # esp_bridge: MAC retries before the delivered attempt (0 for LoRa)
    radio: str = ""           # the model that carried it (``LinkModel.cfg_schedule`` can change it mid-run)


@dataclass
class LinkModel:
    """Stateful link: ``send`` a frame, ``deliver`` the ones that have arrived by now."""

    cfg: LinkConfig
    seed: int = 0
    outages: list[tuple[float, float]] = field(default_factory=list)   # [t0, t1) windows of total loss
    # study hook (2026-11-10): [(t_from, LinkConfig), ...] - a frame sent at t >= t_from travels on that model (e.g. an
    # ESP link whose latency steps to the LoRa model's mid-flight); the ground keeps its own timeout / t_link default
    cfg_schedule: list[tuple[float, LinkConfig]] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.rng = random.Random(self.seed)
        # esp_bridge MAC retries: their own stream, so the loss draws keep the one-draw-per-frame sequence
        self.rng_retry = random.Random((int(self.seed) * 1000003) ^ 0x5E5B)
        self.in_flight: list[Transit] = []
        self.log: list[Transit] = []
        self.stats = {"up_sent": 0, "down_sent": 0, "up_lost": 0, "down_lost": 0, "random": 0, "outage": 0,
                      "collision": 0, "retries": 0}

    def _in_outage(self, a: float, b: float) -> bool:
        return any(a < t1 and b > t0 for (t0, t1) in self.outages)

    def cfg_at(self, t: float) -> LinkConfig:
        cfg = self.cfg
        for t0, c in sorted(self.cfg_schedule, key=lambda x: x[0]):
            if t >= t0:
                cfg = c
        return cfg

    def _retries(self, cfg: LinkConfig) -> int:
        """MAC retries of a delivered esp_bridge frame: inverse CDF of ``esp_retry_pmf(cfg.loss)`` (one draw)."""
        u = self.rng_retry.random()
        acc = 0.0
        pmf = esp_retry_pmf(cfg.loss)
        for k, pk in enumerate(pmf):
            acc += pk
            if u < acc:
                return k
        return len(pmf) - 1

    def send(self, direction: str, data: bytes, t_tx: float) -> Transit:
        cfg = self.cfg_at(t_tx) if self.cfg_schedule else self.cfg
        esp = cfg.radio == "esp_bridge"
        lost = self.rng.random() < cfg.loss               # always one draw per frame (reproducible sequence)
        k = self._retries(cfg) if (esp and not lost) else 0
        h = hop_timing(cfg, direction, len(data), retries=k)
        tr = Transit(direction, bytes(data), t_tx, t_tx + h.air_start, t_tx + h.air_end, t_tx + h.arrival,
                     retries=k, radio=cfg.radio)
        self.stats[f"{direction}_sent"] += 1
        self.stats["retries"] += k
        if lost:
            tr.lost = "random"
        elif self._in_outage(tr.t_air_start, tr.t_air_end):
            tr.lost = "outage"
        for o in self.in_flight:
            if esp or o.radio == "esp_bridge":
                break                                     # ESP-NOW: CSMA/CA defers, the two stations hear each other (E)
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
