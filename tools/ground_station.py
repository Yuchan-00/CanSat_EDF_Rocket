"""CanSat EDF Rocket ground station (SPEC §8.8 "Ground station", §10.3 LoRa link).

The ground station is the link master: it polls the vehicle over the user's LoRa
module, shows the telemetry, logs everything to CSV and sends the operator's
commands.  The rocket flies one mission (params.json ``mission.mode`` = ``ballistic``, v0.4 - v0.6.3):
ARM / DISARM / LAUNCH / THROTTLE / CUT / RELEASE / KILL and the assist rules below; the vehicle only stabilises its
attitude and runs its safety interlocks (I1-I10).  Ctrl+C sends KILL.  (The VTOL build of v0.7 - v0.9 - ON / OFF / UP /
DOWN, profiles, the GPS / GNS and the tether power builds - was removed with the VTOL vehicle on 2026-11-18.)

Everything about the protocol and the command handling comes from the reference
modules - nothing is re-implemented here:

* ``sim.link_proto``           frame format, CRC, scaling, streaming decoder;
* ``sim.ground_station_logic`` ``GroundAssist``: polling, repeat-until-ack with the
  same SEQ, priority of CUT / RELEASE / KILL, the nonce of the latest telemetry on
  every LAUNCH / THROTTLE transmission, the link-delay estimate and the assist
  mode (auto-CUT / auto-RELEASE on each telemetry frame and, v0.6.3, at a reply
  timeout on the latest telemetry extrapolated by its age);
* ``sim.link_model``           the reply timeout and the default link delay of the
  selected radio backend (``--radio``, default ``flight_computer.link.radio``);
* ``params.json`` (through ``cad/layout.py``) every number.

Radio hardware (the bytes on the serial port are the bytes on air), ``--link-hw`` (default
``flight_computer.link.vehicle_radio``, the COM backend the vehicle is built with):
* ``esp_bridge`` (2026-11-06, the default): the ground ESP8266 (ESP-07S class, external
  antenna) running ``firmware/esp_bridge`` (role ground) on a 3.3 V USB-UART, serial at
  ``link.esp_bridge.uart_baud`` 8N1; the port is opened with DTR / RTS released so a board with
  the usual auto-reset wiring is not reset or put into its download mode;
* ``lora_uart`` (LoRa fallback): an Ebyte E22 / E220 / E32-class module in transparent mode
  (M0 = M1 = 0) on a USB-UART adapter, UART at ``link.uart_baud`` 8N1;
* ``sx127x`` (LoRa fallback): the USB <-> LoRa bridge in ``firmware/ground_bridge`` (Arduino Nano
  + SX1276-class module), same bytes, serial at ``link.uart_baud``.
The link MODEL (``--radio``: reply timeout, default link delay, the practice link): 2026-11-10 the model of
``--link-hw`` (``link_model_of``; ``sim.link_model`` has the ESP-NOW model ``esp_bridge`` since the control round), so
the reply timeout always matches the radio on ``--port``; ``--radio`` still overrides it.  (2026-11-06 .. 11-09 the model
stayed lora_uart.)

Modes::

    python tools/ground_station.py --port COM5                 # real link
    python tools/ground_station.py --sim                       # practice (virtual vehicle)
    python tools/ground_station.py --replay logs/ground_station/gs_....csv

Operator commands (typed at the ``gs>`` prompt, read by a separate thread)::

    arm | disarm | launch (asks for confirmation) | throttle <u> | cut | release |
    kill (also Ctrl+C) | assist on|off | target <m> | status | help | quit
    practice only: sim link off|on, sim switch open|closed

Safety rules this program adds on top of the vehicle's own checks (the safe
reading of SPEC §8.8 - the vehicle rejects the same commands anyway):
* ARM / LAUNCH / THROTTLE are refused without fresh telemetry (age <
  ``mission.link_loss_disarm_s``): a command that adds energy, typed during a link
  outage, is never sent later by surprise.  DISARM / CUT / RELEASE / KILL are always
  queued.
* LAUNCH only when the latest telemetry shows ARMED, after a typed confirmation.
* THROTTLE u > 0 only with ``--allow-throttle-cmd`` (tethered tests), capped at
  ``mission.climb_throttle``, confirmed when it would start the motor from ARMED.
* ``target`` only in (0, ``mission.apogee_limit_m``].
* ``--script`` (scripted operator) only in practice mode.
* quitting while the vehicle is ARMED / POWERED / UNPOWERED asks for
  confirmation (the vehicle then disarms by I1p or aborts by I1).

Log (CSV, one row per event, flushed at once): ``TLM`` every decoded telemetry
frame (``note`` = "old" when it is older than the newest one), ``UP`` every
uplink frame sent (heartbeats included), ``TMO`` every request without a reply,
``CMD`` every command queued (operator or assist, with the assist's reasoning),
``ACK`` every command result, ``EVT`` the ground logic's own event lines, ``MSG``
the station's messages (refusals, mode changes).  The raw frame bytes are in
``hex``, so ``--replay`` decodes the same bytes again.

Invariant: the station never invents vehicle data (everything shown or decided
comes from decoded telemetry) and never sends a command that neither the
operator nor the SPEC's assist rule asked for; every frame it sends or receives
is in the log before it is shown.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import dataclasses
import math
import os
import queue
import random
import signal
import sys
import threading
import time
import unicodedata
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Callable

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
for _p in (ROOT, os.path.join(ROOT, "cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import layout as layout_mod  # noqa: E402  (cad/layout.py: params.json + preset)
from sim import link_proto as lp  # noqa: E402
from sim.ground_station_logic import GroundAssist, GroundParams  # noqa: E402
from sim.link_model import RADIOS, LinkConfig, model_radio, round_trip  # noqa: E402

DEFAULT_LOG_DIR = os.path.join(ROOT, "logs", "ground_station")
LIVE_STATES = (lp.STATE_ARMED, lp.STATE_POWERED, lp.STATE_UNPOWERED)     # the link keeps these alive
LOSS_WINDOW_S = 30.0                  # display: request-loss percentage over this window (display choice)
UI_REFRESH_S = 0.25                   # status block refresh period (display choice)
LOOP_SLEEP_S = 0.005                  # main-loop period in real time (well below the ~0.1 s link timescales)
FAST_STEP_S = 0.002                   # --fast: virtual-clock step (half a 250 Hz control tick)
DOUBLE_CTRL_C_S = 2.0                 # a second Ctrl+C within this time quits (after the KILL got out)
QUIT_FLUSH_S = 1.5                    # after a double Ctrl+C keep the link up this long for the KILL to be acked
EVENT_LINES = 7                       # status block: recent event lines shown
SCRIPT_RETRY_EVERY_S = 0.25           # --script: retry a command refused for stale telemetry this often ...
SCRIPT_RETRY_FOR_S = 5.0              # ... for this long (practice scripted operator, like a person retrying)
MIN_UI_ROWS, MIN_UI_COLS = 26, 80     # smaller console -> plain line output (status block is 18 lines + prompt)
PROMPT = "gs> "
EOF_GIVE_UP, EOF_RETRY_S = 5, 0.2     # prompt thread: consecutive EOFs before it treats stdin as closed

CSV_COLUMNS = ["t_wall", "t_gs_s", "kind", "seq", "cmd", "arg", "nonce", "source", "result", "n_tx", "state",
               "flags", "reason", "ack_seq", "t_ms", "h_m", "vz_mps", "roll_deg", "pitch_deg", "yaw_deg",
               "tilt_deg", "throttle", "vcell_v", "loop_max_us", "rtt_s", "t_link_s", "pred_apogee_m", "note", "hex"]


# --------------------------------------------------------------------------- #
# parameters
# --------------------------------------------------------------------------- #
def load_params(preset: str | None = None) -> dict:
    return layout_mod.load_params(layout_mod.PARAMS_PATH, preset)


def link_timing(p: dict, radio: str | None, mode: str | None = None) -> dict:
    """Reply timeout and default link delay of the radio backend (``sim.link_model``; the rocket replies with the 36-byte
    TELEM v1 frame).  ``radio`` None: the params' link model (``sim.link_model.model_radio``: link.vehicle_radio when
    modelled).  ``mode``: the vehicle build's mission (``ballistic``, the only one; None = params.json mission.mode)."""
    mode = lp.mission_mode(p) if mode is None else lp.check_mode(mode)
    cfg = LinkConfig.from_params(p, radio=radio)
    rt = round_trip(cfg)
    return {"radio": cfg.radio, "mode": mode, "reply_timeout_s": rt["reply_timeout_s"],
            "t_link_default_s": rt["rtt_mean_s"], "rtt_worst_s": rt["rtt_worst_s"], "max_poll_hz": rt["max_poll_hz"],
            "poll_hz": cfg.poll_hz, "uart_baud": cfg.esp_uart_baud if cfg.radio == "esp_bridge" else cfg.uart_baud}


def ground_params(p: dict, radio: str | None, target_m: float | None = None, mode: str | None = None) -> GroundParams:
    """``mode``: the vehicle build (``ballistic``, the only one); None = params.json ``mission.mode``."""
    lt = link_timing(p, radio, mode)
    return GroundParams.from_params(p, reply_timeout_s=lt["reply_timeout_s"], t_link_default_s=lt["t_link_default_s"],
                                    target_altitude_m=target_m, mode=lt["mode"])


def clip_cols(text: str, cols: int) -> str:
    """``text`` cut to ``cols`` console columns: a wide (East Asian W / F) character - the Korean reason texts, v0.7.1 -
    takes two, every other character one (an ASCII line is cut exactly as ``text[:cols]``)."""
    n = 0
    for i, ch in enumerate(text):
        n += 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
        if n > cols:
            return text[:i]
    return text


# --------------------------------------------------------------------------- #
# transports: read(now) -> [(t_arrival, bytes)], write(bytes, now)
# --------------------------------------------------------------------------- #
class SerialTransport:
    """A transparent LoRa module on a USB-UART adapter, the SX127x bridge or the ground ESP-NOW bridge (same bytes).
    ``port`` is a port name (COM5, /dev/ttyUSB0) or a pyserial URL (loop://, socket://host:port); ``ser`` injects an open
    serial-like object (in_waiting / read / write / close) instead.  ``release_lines`` (the ESP bridge): DTR and RTS are
    set inactive before the port opens - an ESP8266 board with the usual DTR / RTS auto-reset transistors would
    otherwise be reset (or put into its download mode) when the station opens the port."""

    def __init__(self, port: str, baud: int, ser=None, release_lines: bool = False):
        if ser is None:
            try:
                import serial  # pyserial
            except ImportError as exc:  # pragma: no cover - environment
                raise SystemExit("pyserial is missing: .venv/Scripts/python.exe -m pip install pyserial") from exc
            ser = serial.serial_for_url(port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=0,
                                        write_timeout=1.0, do_not_open=release_lines)
            if release_lines:
                ser.dtr = False
                ser.rts = False
                ser.open()
        self.ser = ser
        self.desc = f"{port} @ {baud} 8N1" + (" (DTR / RTS released)" if release_lines else "")

    def read(self, now: float) -> list[tuple[float, bytes]]:
        n = self.ser.in_waiting
        return [(now, self.ser.read(n))] if n else []

    def write(self, data: bytes, now: float) -> None:
        self.ser.write(data)

    def close(self) -> None:
        self.ser.close()


class NullTransport:
    """Replay: nothing is ever sent."""

    desc = "replay (no radio)"

    def read(self, now: float) -> list[tuple[float, bytes]]:
        return []

    def write(self, data: bytes, now: float) -> None:  # pragma: no cover - replay never polls
        pass

    def close(self) -> None:
        pass


class VirtualVehicle:
    """Practice mode (``--sim``): the real onboard supervisor (``sim.onboard``) behind the link model
    (``sim.link_model``: latency, loss, half-duplex) on a vertical point mass.

    Point mass, vertical only: m dv/dt = T - 1/2 rho v|v| (C_d A_ref + C_dA_chute) - m g, thrust lag
    of ``sim.edf``, battery ``FlightSim.battery_v_cell``, chute inflation ``SimOptions.chute_inflate_tau_s``;
    altitude / v_z through the firmware's ``AltitudeFilter`` from a noisy, delayed baro
    (``SensorConfig``) and the true acceleration plus accelerometer noise.  Attitude is not
    simulated: tilt / roll / pitch / yaw are the estimator noise of ``SensorConfig.att_noise_rad``
    around upright.  Every constant comes from ``sim.sim6dof.FlightSim`` built on ``params.json`` and
    the CAD mass report - this is a practice tool, not a flight prediction (use ``sim/run_scenarios.py``).
    """

    def __init__(self, radio: str | None, loss: float | None, seed: int, mode: str | None = None):
        from sim.estimator import white_noise_sigma
        from sim.link_model import COM_FORWARD_S, FC_REPLY_DELAY_S
        from sim.onboard import OnboardInputs, UplinkFrame
        from sim.sim6dof import PAD_STATES, FlightSim, Scenario
        from sim.vehicle import load_vehicle

        self._Inputs, self._Uplink = OnboardInputs, UplinkFrame
        self._pad_states = PAD_STATES                  # SAFE, ARMED: the NAV's pad mode
        pj = load_params()
        self.mode = lp.mission_mode(pj) if mode is None else lp.check_mode(mode)
        veh = load_vehicle()
        sc = Scenario(name="practice", radio=radio, packet_loss=loss, seed=seed)
        self.fs = fs = FlightSim(veh, sc)
        self.mass_source = veh.mass_source
        self.p = fs.p
        self.sup, self.link, self.dec, self.alt = fs.sup, fs.link, fs.uplink_decoder, fs.alt
        self.vehicle_id = fs.vehicle_id
        self.dt = fs.dt_c
        sens = fs.sc.sensors
        self.sens = sens
        self.rng = random.Random(seed * 7919 + 13)
        self.n_baro = max(1, int(round(fs.ctrl_hz / float(self.p["control"]["baro_hz"]))))
        self.n_lat = int(round(sens.baro_latency_s * float(self.p["control"]["baro_hz"])))
        self.sig_acc = white_noise_sigma(sens.accel_noise_density_mps2_sqrthz, sens.imu_dlpf_tau_s,
                                         float(self.p["control"]["imu_hz"]))
        self.att_sig_deg = math.degrees(sens.att_noise_rad)
        self.align_s = sens.pad_avg_tau_s
        self.k = 0
        self.h = 0.0
        self.v = 0.0
        self.a = 0.0
        self.T = 0.0
        self.energy = 0.0
        self.u = 0.0
        self.on_ground = True
        self.lifted = False
        self.t_release: float | None = None
        self.t_liftoff: float | None = None
        self.t_touchdown: float | None = None
        self.touchdown_vz: float | None = None
        self.touchdowns = 0
        self.h_max = 0.0
        self.t_apogee: float | None = None
        self.arm_switch = True
        self.baro_q: deque[float] = deque()
        # "practice: ..." = the operator's practice controls (shown); "truth: ..." / "VEH ..." = what only the
        # simulator knows (never shown during the run - a real operator sees telemetry only; printed at the end)
        self.events: list[tuple[float, str]] = []
        # on-vehicle link timing (sim.link_model, v0.6.3 co-simulation): COM -> FCU forwarding, uplink -> TELEM handed
        self.com_forward_s, self.reply_delay_s = COM_FORWARD_S, FC_REPLY_DELAY_S
        # the NAV's view of the supervisor state (pad mode = altitude held at 0), as sim.sim6dof: lag ticks behind
        lag = int(sens.nav_state_lag_ticks)
        self.nav_states: deque = deque([self.sup.state] * lag, maxlen=lag)

    # ---- link ------------------------------------------------------------------------------------
    def uplink(self, data: bytes, now: float) -> None:
        self.link.send("up", data, now)

    def set_link(self, on: bool, now: float) -> str:
        outs = self.link.outages
        if not on:
            if outs and outs[-1][1] == float("inf"):
                return "link already off"
            outs.append((now, float("inf")))
            self.events.append((now, "practice: link off"))
            return "practice: radio link OFF (every frame lost) - watch I1p / I1 act on the vehicle"
        if outs and outs[-1][1] == float("inf"):
            outs[-1] = (outs[-1][0], now)
            self.events.append((now, "practice: link on"))
            return "practice: radio link ON"
        return "link already on"

    def set_switch(self, closed: bool, now: float) -> str:
        self.arm_switch = closed
        self.events.append((now, f"practice: arm switch {'closed' if closed else 'open'}"))
        return f"practice: arm switch {'CLOSED' if closed else 'OPEN (acts as KILL on the vehicle)'}"

    def advance(self, now: float) -> list[tuple[float, bytes]]:
        while self.k * self.dt <= now + 1e-12:
            self._tick(self.k, self.k * self.dt)
            self.k += 1
        return [(tr.t_arrival, tr.data) for tr in self.link.deliver("down", now)]

    # ---- one control tick ------------------------------------------------------------------------
    def _tick(self, k: int, t: float) -> None:
        fs, sens, rng = self.fs, self.sens, self.rng
        rx_tr = self.link.deliver("up", t - self.com_forward_s)
        frames = [self._Uplink.from_frame(f) for f in self.dec.feed(b"".join(tr.data for tr in rx_tr))] if rx_tr else []
        # sensors -> altitude filter (as sim6dof: held at the pad reference while the NAV's lagged state is SAFE/ARMED)
        h_baro = None
        if k % self.n_baro == 0:
            hb = self.h + rng.gauss(0.0, sens.baro_noise_m)
            if sens.baro_quant_m > 0.0:                      # the barometer's output step (sim.sim6dof.BARO_MODELS)
                hb = math.floor(hb / sens.baro_quant_m + 0.5) * sens.baro_quant_m
            self.baro_q.append(hb)
            if len(self.baro_q) > self.n_lat:
                h_baro = self.baro_q.popleft()
        pad = self.nav_states[0] in self._pad_states
        if pad:
            self.alt.h, self.alt.v = 0.0, 0.0
        else:
            self.alt.predict(self.a + rng.gauss(0.0, self.sig_acc), self.dt)
        if h_baro is not None:
            self.alt.correct(h_baro, t)
        h_nav, vz_nav = self.alt.h, self.alt.v
        roll, pitch = rng.gauss(0.0, self.att_sig_deg), rng.gauss(0.0, self.att_sig_deg)
        yaw = rng.gauss(0.0, self.att_sig_deg)
        v_cell = fs.battery_v_cell(self.energy, self.u)
        inp = self._Inputs(tick=k, frames=frames, h_m=h_nav, vz_mps=vz_nav, tilt_deg=math.hypot(roll, pitch),
                           gyro_rad_s=(0.0, 0.0, 0.0), v_cell=v_cell,
                           aligned=t >= self.align_s, arm_switch=self.arm_switch, roll_deg=roll, pitch_deg=pitch,
                           yaw_deg=yaw)
        out = self.sup.update(inp)
        u = out.throttle
        self.nav_states.append(out.state)
        for e in out.events:
            self.events.append((t, "VEH " + e))
        if out.reply_seq is not None:
            t_reply = max(t, max(tr.t_arrival for tr in rx_tr) + self.reply_delay_s) if rx_tr else t
            self.link.send("down", lp.encode_telemetry(self.vehicle_id, out.reply_seq, out.telemetry), t_reply)
        if out.release and self.t_release is None:
            self.t_release = t
        self.u = u
        self._physics(t)

    def _physics(self, t: float) -> None:
        fs, dt = self.fs, self.dt
        self.T = fs.edf.step(self.T, self.u, dt)
        released = self.t_release is not None
        m = fs.cfg_post.m if released else fs.cfg_full.m
        cda = fs.cd * fs.a_ref
        if released:
            cda += fs.chute_cda_full * (1.0 - math.exp(-(t - self.t_release) / fs.sc.options.chute_inflate_tau_s))
        drag = 0.5 * fs.rho * cda * self.v * abs(self.v)
        a = (self.T - drag) / m - fs.g
        if self.on_ground:
            if a <= 0.0:
                self.a, self.v = 0.0, 0.0
                self.energy += fs.edf.power(self.u) * dt
                return
            self.on_ground = False
            if not self.lifted:
                self.lifted = True
                self.t_liftoff = t
                self.events.append((t, f"truth: liftoff (T = {self.T:.1f} N)"))
        self.a = a
        self.v += a * dt
        self.h += self.v * dt
        self.energy += fs.edf.power(self.u) * dt
        if self.h > self.h_max:
            self.h_max = self.h
            self.t_apogee = t
        if self.h <= 0.0 and self.v < 0.0:
            self.touchdown_vz = self.v
            self.t_touchdown = t
            self.touchdowns += 1
            self.events.append((t, f"truth: touchdown at {abs(self.v):.1f} m/s"))
            self.h, self.v, self.a = 0.0, 0.0, 0.0
            self.on_ground = True

    def truth(self) -> dict:
        return {"h_m": self.h, "vz_mps": self.v, "h_max_m": self.h_max, "t_apogee_s": self.t_apogee,
                "t_liftoff_s": self.t_liftoff, "t_release_s": self.t_release, "t_touchdown_s": self.t_touchdown,
                "touchdown_vz_mps": self.touchdown_vz, "state": self.sup.state.name}


class SimTransport:
    def __init__(self, vv: VirtualVehicle):
        self.vv = vv
        self.desc = (f"practice: virtual {vv.mode} vehicle, {vv.fs.link_cfg.radio}, packet loss "
                     f"{vv.fs.link_cfg.loss:.0%}")

    def read(self, now: float) -> list[tuple[float, bytes]]:
        return self.vv.advance(now)

    def write(self, data: bytes, now: float) -> None:
        self.vv.uplink(data, now)

    def close(self) -> None:
        pass


# --------------------------------------------------------------------------- #
# CSV log
# --------------------------------------------------------------------------- #
class CsvLog:
    def __init__(self, path: str):
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)
        self.path = path
        self.f = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.f, fieldnames=CSV_COLUMNS, extrasaction="raise")
        self.w.writeheader()
        self.f.flush()
        self.rows = 0

    def row(self, t_gs: float, kind: str, **fields) -> None:
        r = {"t_wall": datetime.now().isoformat(timespec="milliseconds"), "t_gs_s": f"{t_gs:.4f}", "kind": kind}
        for k, v in fields.items():
            r[k] = f"{v:.6g}" if isinstance(v, float) else v
        self.w.writerow(r)
        self.f.flush()
        self.rows += 1

    def close(self) -> None:
        if not self.f.closed:
            self.f.close()


def tlm_fields(t: lp.Telemetry) -> dict:
    d = {"state": t.state_name, "flags": f"0x{t.flags:02X}", "reason": t.reason_name, "ack_seq": t.ack_seq,
         "nonce": f"0x{t.nonce:04X}", "t_ms": t.t_ms, "h_m": t.h_m, "vz_mps": t.vz_mps, "roll_deg": t.roll_deg,
         "pitch_deg": t.pitch_deg, "yaw_deg": t.yaw_deg, "tilt_deg": t.tilt_deg, "throttle": t.throttle,
         "vcell_v": t.vcell_v, "loop_max_us": t.loop_max_us}
    return d


def read_log(path: str) -> list[dict]:
    with open(path, "r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def summarize_log(rows: list[dict]) -> dict:
    """What a log says happened (used by --replay, the exit summary and the tests)."""
    tl = [r for r in rows if r["kind"] == "TLM" and r.get("note") != "old"]
    states: list[str] = []
    for r in tl:
        if not states or states[-1] != r["state"]:
            states.append(r["state"])
    reasons: list[str] = []
    for r in tl:
        if r["reason"] not in ("", "none") and (not reasons or reasons[-1] != r["reason"]):
            reasons.append(r["reason"])
    # the reason byte holds the last interlock code once one has fired (latched until power cycle; a later rejection
    # only sets the REJECTED flag), otherwise the last rejection code
    flag_seen = any(int(r["flags"], 16) & lp.FLAG_INTERLOCK for r in tl if r.get("flags"))
    interlocks = [x for x in reasons if not x.startswith("rejected")]
    if flag_seen and not interlocks:
        interlocks.append("interlock flag set (no telemetry carried its reason code)")
    n_up = sum(1 for r in rows if r["kind"] == "UP")
    n_tmo = sum(1 for r in rows if r["kind"] == "TMO")
    rtts = [float(r["rtt_s"]) for r in rows if r["kind"] == "TLM" and r.get("rtt_s")]
    acks = [r for r in rows if r["kind"] == "ACK"]
    h = [float(r["h_m"]) for r in tl]
    vz = [float(r["vz_mps"]) for r in tl]
    return {
        "telemetry_frames": len(tl), "uplink_frames": n_up, "timeouts": n_tmo,
        "request_loss_pct": 100.0 * n_tmo / n_up if n_up else 0.0,
        "rtt_mean_s": sum(rtts) / len(rtts) if rtts else None, "rtt_max_s": max(rtts) if rtts else None,
        "duration_s": float(rows[-1]["t_gs_s"]) - float(rows[0]["t_gs_s"]) if rows else 0.0,
        "h_max_m": max(h) if h else None, "vz_max_mps": max(vz) if vz else None, "vz_min_mps": min(vz) if vz else None,
        "states": states, "reasons": reasons, "interlock_flag": flag_seen, "interlocks": interlocks,
        "commands": [{"cmd": r["cmd"], "source": r["source"], "seq": r["seq"], "result": r["result"], "n_tx": r["n_tx"],
                      "t_gs_s": float(r["t_gs_s"]), "note": r["note"]} for r in acks],
    }


# --------------------------------------------------------------------------- #
# operator commands
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Action:
    name: str                     # arm disarm launch throttle cut release kill assist target status help quit sim
    value: float | bool | str | None = None
    sub: str = ""                 # sim: link | switch
    confirmed: bool = False


HELP = ("commands: arm | disarm | launch | throttle <u> | cut | release | kill (Ctrl+C) | assist on|off | target <m> | "
        "status | help | quit   practice: sim link off|on, sim switch open|closed")
_ALIASES = {"q": "quit", "exit": "quit", "h": "help", "?": "help", "k": "kill", "abort": "kill", "stop": "kill"}


def parse_command(line: str) -> Action | None:
    """Text -> Action (raises ValueError with a message for the operator)."""
    words = line.strip().split()
    if not words:
        return None
    w = _ALIASES.get(words[0].lower(), words[0].lower())
    args = [a.lower() for a in words[1:]]
    simple = ("arm", "disarm", "launch", "cut", "release", "kill", "status", "help", "quit")
    if w in simple:
        if args:
            raise ValueError(f"'{w}' takes no argument")
        return Action(w)
    if w == "throttle":
        if len(args) != 1:
            raise ValueError("usage: throttle <u>  (0 .. 1)")
        try:
            u = float(args[0])
        except ValueError:
            raise ValueError(f"throttle: '{args[0]}' is not a number") from None
        if not math.isfinite(u) or u < 0.0:
            raise ValueError("throttle must be a number >= 0")
        return Action("throttle", u)
    if w == "assist":
        if len(args) != 1 or args[0] not in ("on", "off"):
            raise ValueError("usage: assist on|off")
        return Action("assist", args[0] == "on")
    if w == "target":
        if len(args) != 1:
            raise ValueError("usage: target <m>")
        try:
            m = float(args[0])
        except ValueError:
            raise ValueError(f"target: '{args[0]}' is not a number") from None
        if not math.isfinite(m):
            raise ValueError("target must be a finite number")
        return Action("target", m)
    if w == "sim":
        ok = {"link": ("off", "on"), "switch": ("open", "closed")}
        if len(args) != 2 or args[0] not in ok or args[1] not in ok[args[0]]:
            raise ValueError("usage: sim link off|on  or  sim switch open|closed")
        return Action("sim", args[1] in ("on", "closed"), sub=args[0])
    raise ValueError(f"unknown command '{words[0]}'. {HELP}")


def parse_script(text: str) -> list[tuple[float, Action]]:
    """``"1:arm, 4:launch, 70:quit"`` -> [(t, Action)] (confirmed: the script is the operator's decision)."""
    out = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        t_s, _, cmd = item.partition(":")
        act = parse_command(cmd)
        if act is None:
            raise ValueError(f"script item '{item}': no command")
        out.append((float(t_s), dataclasses.replace(act, confirmed=True)))
    return sorted(out, key=lambda x: x[0])


# --------------------------------------------------------------------------- #
# the station
# --------------------------------------------------------------------------- #
class TapDecoder:
    """Wraps the ``GroundAssist`` decoder so the station can log every decoded frame (the logic itself
    only returns the telemetry that is newer than what it already has)."""

    def __init__(self, inner: lp.FrameDecoder):
        self.inner = inner
        self.tapped: list[lp.Frame] = []

    @property
    def stats(self) -> dict:
        return self.inner.stats

    def feed(self, data) -> list[lp.Frame]:
        frames = self.inner.feed(data)
        self.tapped.extend(frames)
        return frames

    def reset(self) -> None:
        self.inner.reset()


class GroundStation:
    def __init__(self, p: dict, transport, gp: GroundParams, assist: bool = True, log: CsvLog | None = None,
                 allow_throttle: bool = False, radio: str = "", replay: bool = False):
        self.p = p
        m = p["mission"]
        self.climb = float(m["climb_throttle"])
        self.apogee_limit = float(m["apogee_limit_m"])
        self.fresh_s = float(m["link_loss_disarm_s"])        # telemetry younger than this = link alive
        self.link_abort_s = float(m["link_loss_abort_s"])
        self.help = HELP
        self.transport = transport
        self.gs = GroundAssist(gp, assist=assist)
        self.tap = TapDecoder(self.gs.decoder)
        self.gs.decoder = self.tap
        self.log = log
        self.allow_throttle = allow_throttle
        self.radio = radio
        self.replay = replay
        self.messages: deque[tuple[float, str]] = deque(maxlen=200)
        self.new_messages: list[tuple[float, str]] = []      # not yet shown (plain mode prints them)
        self._n_rec = 0
        self._done: set[int] = set()
        self._n_ev = 0
        self._loss_hist: deque[tuple[float, int, int]] = deque()   # (t, replies, timeouts)
        self._prev_state: int | None = None
        self._prev_reason = lp.REASON_NONE
        self._prev_ilock = False
        self.quit_requested = False
        self.refused = ""                 # why the last request() was refused ("" = it was carried out)
        self.replay_row: dict | None = None
        self.t_start: float | None = None
        self._last_rtt: float | None = None

    # ---- messages ---------------------------------------------------------------------------------
    def say(self, now: float, text: str, log: bool = True) -> str:
        self.messages.append((now, text))
        self.new_messages.append((now, text))
        if log and self.log:
            self.log.row(now, "MSG", note=text)
        return text

    # ---- link -------------------------------------------------------------------------------------
    def step(self, now: float) -> None:
        if self.t_start is None:
            self.t_start = now
        gs = self.gs
        for t_arr, data in self.transport.read(now):
            self._receive(data, t_arr)
        tmo0, seq0 = gs.stats["timeouts"], gs.sent_seq
        up = gs.poll(now)
        if gs.stats["timeouts"] > tmo0 and self.log:
            self.log.row(now, "TMO", seq=seq0, note=f"no reply within {gs.gp.reply_timeout_s:.3f} s")
        if up is not None:
            self.transport.write(up, now)
            if self.log:
                (fr,) = lp.FrameDecoder(gs.gp.vehicle_id, (lp.TYPE_CMD,)).feed(up)
                c = lp.decode_command(fr)
                self.log.row(now, "UP", seq=fr.seq, cmd=c.name, arg=c.arg, nonce=f"0x{c.nonce:04X}", hex=up.hex())
        self._bookkeeping(now)

    def _receive(self, data: bytes, t_arr: float) -> list[lp.Telemetry]:
        gs = self.gs
        n_rtt = len(gs.rtt_all)
        self.tap.tapped.clear()
        new = gs.receive(data, t_arr)
        rtt = gs.rtt_all[-1][1] if len(gs.rtt_all) > n_rtt else None
        if rtt is not None:
            self._last_rtt = rtt
        fresh = list(new)
        for fr in self.tap.tapped:
            tlm = lp.decode_telemetry(fr)
            is_new = tlm in fresh
            if is_new:
                fresh.remove(tlm)
            if self.log:
                self.log.row(t_arr, "TLM", seq=fr.seq, **tlm_fields(tlm), rtt_s=rtt if rtt is not None else "",
                             t_link_s=gs.t_link, pred_apogee_m=gs.predicted_apogee(tlm.h_m, tlm.vz_mps, gs.t_link),
                             note="" if is_new else "old", hex=lp.encode_frame(fr.veh, fr.type, fr.seq, fr.payload).hex())
            rtt = None
            if is_new:
                self._on_new_telemetry(tlm, t_arr)
        return new

    def feed_replay(self, frame_hex: str, t: float, row: dict | None = None) -> None:
        self.replay_row = row
        self._receive(bytes.fromhex(frame_hex), t)
        self._bookkeeping(t)

    def _on_new_telemetry(self, t: lp.Telemetry, now: float) -> None:
        if self._prev_state != t.state:
            prev = lp.STATE_NAMES.get(self._prev_state, "-") if self._prev_state is not None else "-"
            self.say(now, f"vehicle state {prev} -> {t.state_name}", log=False)
            self._prev_state = t.state
        ilock = t.flag(lp.FLAG_INTERLOCK)
        if t.reason != self._prev_reason or (ilock and not self._prev_ilock):
            if 0 < t.reason < 0x40:
                self.say(now, f"INTERLOCK on the vehicle: {t.reason_name}", log=False)
            elif ilock and not self._prev_ilock:
                self.say(now, f"INTERLOCK on the vehicle (flag set; reason byte already shows {t.reason_name})", log=False)
            self._prev_reason = t.reason
            self._prev_ilock = ilock

    def _bookkeeping(self, now: float) -> None:
        gs, log = self.gs, self.log
        for r in gs.records[self._n_rec:]:
            if log:
                log.row(now, "CMD", cmd=r.name, arg=r.arg, source=r.source, note=r.note)
        self._n_rec = len(gs.records)
        for r in gs.records:
            if r.result and id(r) not in self._done:
                self._done.add(id(r))
                if log:
                    log.row(now, "ACK", seq=r.seq if r.seq is not None else "", cmd=r.name, arg=r.arg, source=r.source,
                            result=r.result, n_tx=r.n_tx,
                            note=f"queued {r.t_queued:.3f}, first tx {r.t_first_tx if r.t_first_tx is not None else '-'}")
                lat = f", {r.t_ack - r.t_queued:.2f} s" if r.t_ack is not None else ""
                self.say(now, f"{r.name} ({r.source}): {r.result} (sent {r.n_tx}x{lat})", log=False)
        for t_e, e in gs.events[self._n_ev:]:
            if log:
                log.row(t_e, "EVT", note=e)
            if e.startswith(("queue", "synced", "renumber", "assist", "target")):
                self.say(t_e, e, log=False)
        self._n_ev = len(gs.events)
        st = gs.stats
        if not self._loss_hist or self._loss_hist[-1][1:] != (st["replies"], st["timeouts"]):
            self._loss_hist.append((now, st["replies"], st["timeouts"]))
        while len(self._loss_hist) > 2 and self._loss_hist[1][0] < now - LOSS_WINDOW_S:
            self._loss_hist.popleft()

    # ---- derived status ---------------------------------------------------------------------------
    def age(self, now: float) -> float | None:
        return None if self.gs.t_latest is None else now - self.gs.t_latest

    def fresh(self, now: float) -> bool:
        a = self.age(now)
        return a is not None and a < self.fresh_s

    def loss_pct(self) -> tuple[float | None, float | None]:
        """(window, total) share of requests that got no reply."""
        st = self.gs.stats
        done = st["replies"] + st["timeouts"]
        total = 100.0 * st["timeouts"] / done if done else None
        if len(self._loss_hist) >= 2:
            (_, r0, t0), (_, r1, t1) = self._loss_hist[0], self._loss_hist[-1]
            d = (r1 - r0) + (t1 - t0)
            return (100.0 * (t1 - t0) / d if d else None), total
        return None, total

    # ---- operator ---------------------------------------------------------------------------------
    def confirmation_prompt(self, act: Action) -> str | None:
        """Text of the confirmation question this action needs (read by the input thread), or None."""
        if act.confirmed:
            return None
        t = self.gs.latest
        if act.name == "launch":
            return "LAUNCH: the motor starts at climb throttle. Type 'yes' to launch: "
        if (act.name == "throttle" and act.value and self.allow_throttle and t is not None
                and t.state == lp.STATE_ARMED):
            return f"THROTTLE {min(float(act.value), self.climb):.3f} from ARMED starts the motor. Type 'yes': "
        if act.name == "quit" and t is not None and t.state in LIVE_STATES:
            what = "disarm (I1p)" if t.state == lp.STATE_ARMED else "ABORT (I1) and open the release"
            return f"vehicle is {t.state_name}: without the link it will {what}. Type 'yes' to quit: "
        return None

    def request(self, act: Action, now: float) -> str:
        """Execute an operator action; returns the message for the operator (also logged)."""
        gs = self.gs
        t = gs.latest
        n = act.name
        self.refused = ""
        if self.replay and n not in ("status", "help", "quit"):
            self.refused = "replay"
            return self.say(now, f"{n}: replay - nothing is sent")
        if n == "help":
            return self.help
        if n == "status":
            return " | ".join(self.status_lines(now, 200)[1:5])
        if n == "quit":
            if self.confirmation_prompt(act) is not None:
                self.refused = "confirm"
                return self.say(now, "quit needs confirmation")
            self.quit_requested = True
            return self.say(now, "quit")
        if n == "kill":
            gs.kill(now)
            return self.say(now, "KILL queued (pre-empts everything; repeated until acknowledged)")
        if n == "cut":
            gs.cut(now)
            return self.say(now, "CUT queued")
        if n == "release":
            gs.release(now)
            return self.say(now, "RELEASE queued")
        if n == "assist":
            gs.set_assist(bool(act.value), now)
            return self.say(now, f"assist {'ON' if act.value else 'OFF'}"
                            + ("" if act.value else ": you must CUT and RELEASE yourself (onboard I4 / I6 still act)"))
        if n == "target":
            m = float(act.value)
            if not 0.0 < m <= self.apogee_limit:
                self.refused = "range"
                return self.say(now, f"target refused: must be in (0, {self.apogee_limit:.0f}] m (mission.apogee_limit_m)")
            gs.gp = dataclasses.replace(gs.gp, target_altitude_m=m)
            note = " (auto-CUT already queued this flight)" if gs.cut_queued else ""
            return self.say(now, f"assist target altitude {m:.1f} m{note}")
        if n == "sim":
            vv = getattr(self.transport, "vv", None)
            if vv is None:
                self.refused = "mode"
                return self.say(now, "'sim' commands exist only in practice mode (--sim)")
            return self.say(now, vv.set_link(bool(act.value), now) if act.sub == "link" else vv.set_switch(bool(act.value), now))
        # commands that add energy need a live link: never sent later by surprise
        if n in ("arm", "launch", "throttle"):
            if not self.fresh(now):
                a = self.age(now)
                why = "no telemetry yet" if a is None else f"last telemetry {a:.1f} s old"
                self.refused = "stale"
                return self.say(now, f"{n.upper()} refused: {why} (link must be alive)")
        if n == "arm":
            gs.arm(now)
            return self.say(now, "ARM queued")
        if n == "disarm":
            gs.disarm(now)
            return self.say(now, "DISARM queued")
        if n == "launch":
            if t is None or t.state != lp.STATE_ARMED:
                self.refused = "state"
                return self.say(now, f"LAUNCH refused: vehicle is {t.state_name if t else '?'}, not ARMED")
            if not act.confirmed:
                self.refused = "confirm"
                return self.say(now, "LAUNCH needs confirmation")
            gs.launch(now)
            return self.say(now, f"LAUNCH queued (nonce of the latest telemetry: 0x{t.nonce:04X}, refreshed on every repeat)")
        if n == "throttle":
            u = float(act.value)
            if u > 0.0 and not self.allow_throttle:
                self.refused = "throttle"
                return self.say(now, "THROTTLE > 0 refused: start the station with --allow-throttle-cmd (tethered tests only)")
            capped = min(u, self.climb)
            if t is not None and t.state == lp.STATE_ARMED and capped > 0.0 and not act.confirmed:
                self.refused = "confirm"
                return self.say(now, "THROTTLE from ARMED needs confirmation")
            gs.throttle(capped, now)
            extra = f" (capped from {u:.3f} at climb_throttle)" if capped < u else ""
            return self.say(now, f"THROTTLE {capped:.3f} queued{extra}")
        return self.say(now, f"unknown action {n}")  # pragma: no cover

    def interrupt(self, now: float) -> str:
        """Ctrl+C: KILL (pre-empting anything pending); a KILL still waiting for its ack is kept, not re-queued."""
        p = self.gs.pending
        if p is not None and p.cmd == lp.CMD_KILL and not p.result:
            return self.say(now, f"Ctrl+C: KILL already pending (sent {p.n_tx}x). Ctrl+C again within "
                                 f"{DOUBLE_CTRL_C_S:.0f} s quits.")
        self.gs.kill(now)
        return self.say(now, f"Ctrl+C: KILL queued. Ctrl+C again within {DOUBLE_CTRL_C_S:.0f} s quits.")

    # ---- display ----------------------------------------------------------------------------------
    def status_lines(self, now: float, width: int) -> list[str]:
        gs, t = self.gs, self.gs.latest
        lines = []
        hdr = f"CanSat EDF ground station | {self.transport.desc} | veh {gs.gp.vehicle_id} | t {now:7.1f} s"
        if self.log:
            hdr += f" | log {os.path.basename(self.log.path)}"
        lines.append(hdr)
        a = self.age(now)
        if t is None:                                    # same number of lines as with telemetry (block height fixed)
            lines.append(f"STATE  ?            no telemetry yet (polling {gs.stats['polls']}x)")
            lines += ["flags  -", "h      -", "att    -"]
        else:
            link = "LINK OK" if a is not None and a < self.fresh_s else (
                f"NO TELEMETRY {a:.1f} s: vehicle disarms (I1p) after {self.fresh_s:.1f} s on the pad, "
                f"aborts (I1) after {self.link_abort_s:.1f} s in flight")
            lines.append(f"STATE  {t.state_name:<10} assist {'ON ' if gs.assist else 'OFF'} target {gs.gp.target_altitude_m:6.1f} m"
                         f" | t_veh {t.t_s:8.3f} s | age {a:5.2f} s | {link}")
            fl = " ".join(n.upper() if t.flag(b) else "-" * len(n) for b, n in lp.FLAG_NAMES.items())
            lines.append(f"flags  {fl} | nonce 0x{t.nonce:04X}")
            pred = gs.predicted_apogee(t.h_m, t.vz_mps, gs.t_link)
            lines.append(f"h {t.h_m:7.1f} m   v_z {t.vz_mps:+7.2f} m/s   predicted apogee {pred:6.1f} m   tilt {t.tilt_deg:6.2f} deg")
            lines.append(f"att    roll {t.roll_deg:+7.2f}  pitch {t.pitch_deg:+7.2f}  yaw {t.yaw_deg:+8.2f} deg | "
                         f"throttle {t.throttle:5.3f} | V_cell {t.vcell_v:4.2f} V | loop_max {t.loop_max_us} us")
        if self.replay_row is not None:
            r = self.replay_row
            lines.append(f"link   (logged) t_link {r.get('t_link_s') or '-'} s, rtt {r.get('rtt_s') or '-'} s")
        else:
            lw, ltot = self.loss_pct()
            last = f"{self._last_rtt:.3f}" if self._last_rtt is not None else "-"
            lws = f"{lw:4.1f} %" if lw is not None else "  -  "
            lts = f"{ltot:4.1f} %" if ltot is not None else "-"
            st, ds = gs.stats, self.tap.stats
            lines.append(f"link   delay {gs.t_link:5.3f} s (last {last}) | loss {lws} last {LOSS_WINDOW_S:.0f} s, {lts} total"
                         f" | polls {st['polls']} replies {st['replies']} timeouts {st['timeouts']}"
                         f" | bad CRC {ds['bad_crc']} skipped {ds['skipped_bytes']} B")
        if t is not None and t.reason:
            kind = "interlock" if t.reason < 0x40 else "command rejected"
            lines.append(f"reason {t.reason_name} ({kind})" + ("   [an interlock has acted since boot]" if t.flag(lp.FLAG_INTERLOCK) else ""))
        else:
            lines.append("reason none")
        rec = gs.records[-1] if gs.records else None
        if rec is None:
            lines.append("ack    -")
        else:
            res = rec.result or ("waiting for telemetry" if rec.seq is None and not gs.synced else "pending")
            seq = rec.seq if rec.seq is not None else "-"
            lines.append(f"ack    {rec.name}{' ' + str(rec.arg) if rec.cmd == lp.CMD_THROTTLE else ''} ({rec.source}) seq {seq}"
                         f" sent {rec.n_tx}x: {res}")
        lines.append("-- events " + "-" * 40)
        evs = list(self.messages)[-EVENT_LINES:]
        for t_e, e in evs:
            lines.append(f"{t_e:8.2f}  {e}")
        lines += [""] * (EVENT_LINES - len(evs))
        lines.append("-" * 50)
        lines.append(self.help)
        return [clip_cols(ln, max(width - 1, 20)) for ln in lines]

    def plain_line(self, now: float) -> str:
        t, gs = self.gs.latest, self.gs
        if t is None:
            return f"[{now:7.2f}] no telemetry (polls {gs.stats['polls']})"
        lw, _ = self.loss_pct()
        a = self.age(now)
        return (f"[{now:7.2f}] {t.state_name:<9} h {t.h_m:6.1f} vz {t.vz_mps:+6.2f} tilt {t.tilt_deg:5.2f} thr {t.throttle:4.2f} "
                f"Vc {t.vcell_v:4.2f} | delay {gs.t_link:4.2f} s loss {'-' if lw is None else f'{lw:.0f}'} % age {a:4.2f} | "
                f"{t.reason_name} | assist {'on' if gs.assist else 'off'}")


# --------------------------------------------------------------------------- #
# console (no curses): status block at the top, prompt below
# --------------------------------------------------------------------------- #
def _enable_vt() -> bool:
    if not sys.stdout.isatty():
        return False
    if os.name != "nt":
        return True
    try:
        import ctypes
        from ctypes import wintypes
        k32 = ctypes.windll.kernel32
        h = k32.GetStdHandle(-11)
        mode = wintypes.DWORD()
        if not k32.GetConsoleMode(h, ctypes.byref(mode)):
            return False
        return bool(k32.SetConsoleMode(h, mode.value | 0x0004))       # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except Exception:  # pragma: no cover - platform specific
        return False


class Console:
    """``ui`` = status block refreshed in place (VT sequences, scroll region below it for the prompt);
    otherwise plain lines (pipes, old consoles, --plain)."""

    def __init__(self, want_ui: bool, quiet: bool = False, force_ui: bool = False):
        self.lock = threading.Lock()
        self.quiet = quiet
        self.ui = (want_ui or force_ui) and _enable_vt()
        self.block_h = 0
        try:
            sys.stdout.reconfigure(errors="replace")
        except Exception:  # pragma: no cover
            pass
        if self.ui and not force_ui:
            cols, rows = self._size()
            if rows < MIN_UI_ROWS or cols < MIN_UI_COLS:
                self.ui = False

    @staticmethod
    def _size() -> tuple[int, int]:
        try:
            s = os.get_terminal_size()
            return s.columns, s.lines
        except OSError:
            return 120, 40

    def draw(self, station: GroundStation, now: float) -> None:
        if not self.ui:
            for t_e, e in station.new_messages:
                self.say(f"  {t_e:8.2f}  {e}")
            station.new_messages.clear()
            return
        station.new_messages.clear()
        cols, rows = self._size()
        lines = station.status_lines(now, cols)
        out = []
        if self.block_h != len(lines):
            # (re)layout: clear the screen, scroll region below the block, prompt on the bottom row
            self.block_h = len(lines)
            out.append(f"\x1b[2J\x1b[{self.block_h + 1};{rows}r\x1b[{rows};1H")
        out.append("\x1b7")
        for i, ln in enumerate(lines):
            out.append(f"\x1b[{i + 1};1H\x1b[2K{ln}")
        out.append("\x1b8")
        with self.lock:
            sys.stdout.write("".join(out))
            sys.stdout.flush()

    def say(self, text: str) -> None:
        with self.lock:
            if self.ui:      # the prompt thread waits on the bottom line: replace its prompt, print, prompt again
                sys.stdout.write(f"\r\x1b[2K{text}\n{PROMPT}")
            else:
                sys.stdout.write(text + "\n")
            sys.stdout.flush()

    def note(self, text: str) -> None:
        """A line from the prompt thread itself (it prints its own prompt next)."""
        with self.lock:
            sys.stdout.write(text + "\n")
            sys.stdout.flush()

    def close(self) -> None:
        if self.ui:
            _, rows = self._size()
            with self.lock:
                sys.stdout.write(f"\x1b[r\x1b[{rows};1H\n")
                sys.stdout.flush()
            self.ui = False                           # the summary after the run prints plain lines


def input_thread(station: GroundStation, console: Console, q: queue.Queue, stop: threading.Event) -> None:
    """Reads operator lines; asks the confirmation questions; hands Actions to the main loop."""
    # On Windows a console Ctrl+C makes input() in this thread return / raise EOF once, while stdin redirected
    # from NUL reports isatty() == True and raises EOF forever: give up after EOF_GIVE_UP consecutive EOFs.
    interactive = sys.stdin.isatty()
    eofs = 0
    while not stop.is_set():
        try:
            line = input(PROMPT)
            eofs = 0
        except (EOFError, KeyboardInterrupt):
            eofs += 1
            if not interactive or stop.is_set() or eofs >= EOF_GIVE_UP:
                q.put(Action("eof"))
                return
            time.sleep(EOF_RETRY_S)
            continue
        try:
            act = parse_command(line)
        except ValueError as exc:
            console.note(str(exc))
            continue
        if act is None:
            continue
        ask = station.confirmation_prompt(act)
        if ask is not None:
            try:
                ans = input(ask)
            except (EOFError, KeyboardInterrupt):
                ans = ""
            if ans.strip().lower() != "yes":
                console.note(f"{act.name}: cancelled")
                continue
            act = dataclasses.replace(act, confirmed=True)
        q.put(act)


# --------------------------------------------------------------------------- #
# run loops
# --------------------------------------------------------------------------- #
def run_live(station: GroundStation, console: Console, clock: Callable[[], float], *, fast: bool = False,
             script: list[tuple[float, Action]] | None = None, max_time_s: float | None = None,
             interactive: bool = True, on_step: Callable[[float], None] | None = None) -> None:
    """Main loop (main thread): link step, operator actions, display.  Ctrl+C is caught by a SIGINT
    handler that only records the key press, so it never interrupts a half-done serial / log write;
    the loop turns it into KILL (a second one within DOUBLE_CTRL_C_S quits after the KILL is acked)."""
    q: queue.Queue = queue.Queue()
    stop = threading.Event()
    prompt_started = False                     # the prompt thread starts after the first draw laid out the screen
    pending = [(t, t, act) for t, act in (script or [])]          # (due, first due, action), sorted by due
    last_draw = -math.inf
    last_plain = -math.inf
    presses: list[int] = []                    # SIGINT handler -> loop
    ctrl_c: list[float] = []
    quit_at: float | None = None
    vv = getattr(station.transport, "vv", None)
    n_vev = 0
    old_handler = None
    if threading.current_thread() is threading.main_thread():
        old_handler = signal.signal(signal.SIGINT, lambda signum, frame: presses.append(signum))
    try:
        while True:
            now = clock()
            station.step(now)
            if vv is not None:
                for t_e, e in vv.events[n_vev:]:
                    if e.startswith("practice:"):
                        station.say(t_e, e, log=False)
                n_vev = len(vv.events)
            while presses:
                presses.pop()
                ctrl_c = [t for t in ctrl_c if now - t < DOUBLE_CTRL_C_S]
                if ctrl_c and quit_at is None:
                    station.say(now, f"second Ctrl+C: quitting once the KILL is acknowledged (at most {QUIT_FLUSH_S:.1f} s)")
                    quit_at = now + QUIT_FLUSH_S
                ctrl_c.append(now)
                msg = station.interrupt(now)
                if console.ui:
                    console.say(msg)
            while pending and pending[0][0] <= now:
                t_due, t_first, act = pending.pop(0)
                if act.name == "sim":
                    arg = f" {act.sub} " + {"switch": ("open", "closed")}.get(act.sub, ("off", "on"))[bool(act.value)]
                else:
                    arg = "" if act.value is None else " " + str(act.value)
                station.say(now, f"script: {act.name}{arg}", log=False)
                station.request(act, now)
                # a scripted operator does what a person would: retry a command refused for a stale link
                if station.refused == "stale" and now - t_first < SCRIPT_RETRY_FOR_S:
                    bisect.insort(pending, (now + SCRIPT_RETRY_EVERY_S, t_first, act), key=lambda x: x[0])
            while not station.quit_requested:
                try:
                    act = q.get_nowait()
                except queue.Empty:
                    break
                if act.name == "eof":
                    station.say(now, "operator input closed (stdin)")
                    continue
                msg = station.request(act, now)
                if console.ui:
                    console.say(msg)
            if on_step is not None:
                on_step(now)
            if quit_at is not None and (now >= quit_at or _interrupt_done(station)):
                break
            if station.quit_requested:
                break
            if max_time_s is not None and now >= max_time_s:
                station.say(now, f"max time {max_time_s:.0f} s reached")
                break
            if now - last_draw >= UI_REFRESH_S:
                console.draw(station, now)
                last_draw = now
                if interactive and not prompt_started:
                    prompt_started = True
                    threading.Thread(target=input_thread, args=(station, console, q, stop), daemon=True).start()
            if not console.ui and not console.quiet and now - last_plain >= 1.0:
                console.say(station.plain_line(now))
                last_plain = now
            if not fast:
                time.sleep(LOOP_SLEEP_S)
    finally:
        stop.set()
        if old_handler is not None:
            signal.signal(signal.SIGINT, old_handler)
        console.draw(station, clock())


def _interrupt_done(station: GroundStation) -> bool:
    """the KILL Ctrl+C queued has its answer"""
    recs = [r for r in station.gs.records if r.cmd == lp.CMD_KILL]
    return bool(recs) and bool(recs[-1].result)


_kill_done = _interrupt_done                          # v0.6.3 name


def run_replay(path: str, p: dict, console: Console, speed: float, fast: bool) -> dict:
    rows = read_log(path)
    if not rows:
        raise SystemExit(f"{path}: empty log")
    radio_guess = None
    if any(r.get("phase") or r.get("gps_flags") or r.get("pwr_flags") for r in rows if r["kind"] == "TLM"):
        raise SystemExit(f"{path}: a VTOL log (TELEM v2 - v7 columns): the VTOL build was removed 2026-11-18")
    gp = ground_params(p, radio_guess, mode=lp.MODE_BALLISTIC)
    st = GroundStation(p, NullTransport(), gp, assist=False, log=None, replay=True)
    t0 = float(rows[0]["t_gs_s"])
    wall0 = time.monotonic()
    last_draw = -math.inf
    last_plain = -math.inf
    for r in rows:
        t = float(r["t_gs_s"])
        if not fast:
            wait = (t - t0) / max(speed, 1e-6) - (time.monotonic() - wall0)
            if wait > 0:
                time.sleep(wait)
        k = r["kind"]
        if k == "TLM" and r.get("hex"):
            st.feed_replay(r["hex"], t, r)
        elif k == "CMD":
            st.say(t, f"[log] queue {r['cmd']} ({r['source']}{': ' + r['note'] if r['note'] else ''})", log=False)
        elif k == "ACK":
            st.say(t, f"[log] {r['cmd']} ({r['source']}): {r['result']} (sent {r['n_tx']}x)", log=False)
        elif k == "MSG":
            st.say(t, f"[log] {r['note']}", log=False)
        if t - last_draw >= UI_REFRESH_S:
            console.draw(st, t)
            last_draw = t
        if not console.ui and not console.quiet and t - last_plain >= 1.0 and st.gs.latest is not None:
            console.say(st.plain_line(t))
            last_plain = t
    console.draw(st, float(rows[-1]["t_gs_s"]))
    return summarize_log(rows)


def print_summary(console: Console, s: dict, extra: dict | None = None) -> None:
    def f(x, fmt="{:.1f}"):
        return "-" if x is None else fmt.format(x)

    console.say("---- summary ----")
    console.say(f"duration {s['duration_s']:.1f} s, telemetry {s['telemetry_frames']}, uplink {s['uplink_frames']}, "
                f"timeouts {s['timeouts']} ({s['request_loss_pct']:.1f} %), RTT mean {f(s['rtt_mean_s'], '{:.3f}')} s "
                f"max {f(s['rtt_max_s'], '{:.3f}')} s")
    console.say(f"states: {' -> '.join(s['states']) or '-'}")
    console.say(f"h max {f(s['h_max_m'])} m (telemetry), v_z {f(s['vz_min_mps'])} .. {f(s['vz_max_mps'])} m/s")
    console.say(f"interlocks: {', '.join(s['interlocks']) or 'none'}")
    for c in s["commands"]:
        console.say(f"  {c['t_gs_s']:8.2f}  {c['cmd']:<8} {c['source']:<8} {c['result']} (sent {c['n_tx']}x)")
    if extra:
        console.say("practice truth: " + ", ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}"
                                                    for k, v in extra.items()))


# --------------------------------------------------------------------------- #
# command line
# --------------------------------------------------------------------------- #
def list_ports() -> list[str]:
    try:
        from serial.tools import list_ports as lpm
    except ImportError:  # pragma: no cover
        return []
    return [f"{x.device}  {x.description}" for x in lpm.comports()]


LINK_HW = ("esp_bridge", "lora_uart", "sx127x")     # ground radio hardware (2026-11-06; --radio is the link MODEL)


def default_link_hw(p: dict) -> str:
    """the ground hardware that matches the vehicle's COM backend: flight_computer.link.vehicle_radio (else link.radio)"""
    lk = p["flight_computer"]["link"]
    hw = str(lk.get("vehicle_radio", lk["radio"]))
    return hw if hw in LINK_HW else str(lk["radio"])


def link_model_of(p: dict, link_hw: str | None, radio: str | None = None) -> str:
    """2026-11-10: the link model the station times its requests with: --radio if given, else the model of the ground
    hardware --link-hw (every LINK_HW has one in sim.link_model.RADIOS), else the params' model (model_radio) - so the
    reply timeout always matches the radio on --port (the ESP model's 0.25 s timeout would cut a LoRa reply)."""
    if radio is not None:
        return radio
    return link_hw if link_hw in RADIOS else model_radio(p)


def serial_baud(p: dict, link_hw: str, baud: int | None = None) -> int:
    """--baud, else the UART baud of that hardware: link.esp_bridge.uart_baud for the ESP bridge (the same on both ESPs and
    the COM, firmware/esp_bridge), link.uart_baud for the LoRa module / the SX127x bridge"""
    if baud is not None:
        return int(baud)
    lk = p["flight_computer"]["link"]
    if link_hw == "esp_bridge":
        return int(lk["esp_bridge"]["uart_baud"])
    return int(lk["uart_baud"])


def build_arg_parser(p: dict) -> argparse.ArgumentParser:
    lk = p["flight_computer"]["link"]
    ap = argparse.ArgumentParser(description="CanSat EDF Rocket LoRa ground station (SPEC 8.8 / 10.3)")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", help="serial port of the ground ESP bridge, the ground LoRa module or the SX127x bridge "
                                    "(e.g. COM5)")
    src.add_argument("--sim", action="store_true", help="practice mode: in-process virtual vehicle")
    src.add_argument("--replay", metavar="CSV", help="replay a ground-station log")
    src.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    ap.add_argument("--link-hw", choices=LINK_HW, default=default_link_hw(p),
                    help="the ground radio hardware on --port: esp_bridge (ground ESP8266, firmware/esp_bridge), "
                         "lora_uart (Ebyte module on a USB-UART) or sx127x (firmware/ground_bridge); default "
                         "flight_computer.link.vehicle_radio. Sets the default --baud (and releases DTR / RTS for the ESP)")
    ap.add_argument("--baud", type=int, default=None,
                    help="serial baud (default: link.esp_bridge.uart_baud for --link-hw esp_bridge, else link.uart_baud)")
    ap.add_argument("--mode", choices=tuple(lp.MISSION_MODES), default=lp.mission_mode(p),
                    help="the vehicle build: ballistic (ARM ... KILL), the rocket's only mission; default params.json "
                         "mission.mode")
    ap.add_argument("--radio", choices=RADIOS, default=None,
                    help="radio link MODEL: sets the reply timeout / default link delay (sim.link_model) and the practice "
                         "link; default (2026-11-10): the model of --link-hw (esp_bridge / lora_uart / sx127x), so the "
                         "timeout always matches the hardware on --port")
    ap.add_argument("--preset", default=None, help="params.json preset (default: the file's preset)")
    ap.add_argument("--no-assist", action="store_true", help="start with assist mode off")
    ap.add_argument("--target", type=float, default=None, help="assist target altitude [m] (default mission.target_altitude_m)")
    ap.add_argument("--allow-throttle-cmd", action="store_true", help="allow 'throttle <u>' with u > 0 (tethered tests)")
    ap.add_argument("--log", default=None, help="CSV log path (default logs/ground_station/gs_<time>.csv)")
    ap.add_argument("--no-log", action="store_true", help="practice / replay only: do not write a log")
    ap.add_argument("--plain", action="store_true", help="plain line output instead of the status block")
    ap.add_argument("--ui", action="store_true",
                    help=f"status block even on a console smaller than {MIN_UI_COLS}x{MIN_UI_ROWS} (lines are cut)")
    ap.add_argument("--quiet", action="store_true", help="plain mode: events only, no periodic status line")
    ap.add_argument("--loss", type=float, default=None, help="practice: packet loss 0..1 (default link.sim_packet_loss)")
    ap.add_argument("--seed", type=int, default=1, help="practice: random seed")
    ap.add_argument("--speed", type=float, default=1.0, help="practice / replay: time factor")
    ap.add_argument("--fast", action="store_true", help="practice / replay: run as fast as possible (virtual clock)")
    ap.add_argument("--script", default=None, help="practice only: scripted operator, e.g. \"1:arm,4:launch,60:quit\"")
    ap.add_argument("--max-time", type=float, default=None, help="practice: stop after this many seconds")
    return ap


def main(argv: list[str] | None = None) -> int:
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--preset", default=None)
    known, _ = pre.parse_known_args(argv)
    p = load_params(known.preset)
    ap = build_arg_parser(p)
    a = ap.parse_args(argv)
    a.radio = link_model_of(p, a.link_hw, a.radio)   # 2026-11-10: default = the model of the hardware on --port
    if a.list_ports:
        for s in list_ports() or ["(no serial ports found)"]:
            print(s)
        return 0
    if not (a.port or a.sim or a.replay):
        ap.print_usage()
        print("give --port COMx (real link), --sim (practice) or --replay <csv>. Serial ports:")
        for s in list_ports() or ["  (none found)"]:
            print("  " + s)
        return 2
    if a.script and not a.sim:
        ap.error("--script is allowed only with --sim (practice)")
    if (a.fast or a.speed != 1.0) and not (a.sim or a.replay):
        ap.error("--fast / --speed only with --sim or --replay")
    if a.no_log and not (a.sim or a.replay):
        ap.error("--no-log only in practice / replay: a real link is always logged")
    if not a.sim and (a.loss is not None or a.max_time is not None):
        ap.error("--loss / --max-time only with --sim")
    if a.loss is not None and not 0.0 <= a.loss <= 1.0:
        ap.error("--loss must be in 0..1")
    if a.target is not None and not 0.0 < a.target <= float(p["mission"]["apogee_limit_m"]):
        ap.error(f"--target must be in (0, {p['mission']['apogee_limit_m']}] m (mission.apogee_limit_m)")
    if a.ui and (a.plain or a.fast):
        ap.error("--ui cannot be combined with --plain / --fast")
    console = Console(want_ui=not a.plain and not a.fast, quiet=a.quiet, force_ui=a.ui)

    if a.replay:
        s = run_replay(a.replay, p, console, a.speed, a.fast)
        print_summary(console, s)
        console.close()
        return 0

    lt = link_timing(p, a.radio, a.mode)
    gp = ground_params(p, a.radio, a.target, mode=a.mode)
    vv = None
    if a.sim:
        vv = VirtualVehicle(a.radio, a.loss, a.seed, mode=a.mode)
        transport = SimTransport(vv)
    else:
        transport = SerialTransport(a.port, serial_baud(p, a.link_hw, a.baud), release_lines=a.link_hw == "esp_bridge")
    log = None
    if not a.no_log:
        path = a.log or os.path.join(DEFAULT_LOG_DIR, f"gs_{'sim_' if a.sim else ''}{datetime.now():%Y%m%d_%H%M%S}.csv")
        log = CsvLog(path)
    st = GroundStation(p, transport, gp, assist=not a.no_assist, log=log, allow_throttle=a.allow_throttle_cmd,
                       radio=lt["radio"])
    hw = "" if (a.sim or a.replay) else f" ({a.link_hw})"
    st.say(0.0, f"start: {a.mode} build, {transport.desc}{hw}, radio model {lt['radio']}: reply timeout "
                f"{lt['reply_timeout_s']:.3f} s, nominal RTT {lt['t_link_default_s']:.3f} s, poll {lt['poll_hz']:.1f} Hz "
                f"(achievable {lt['max_poll_hz']:.2f} Hz)" + (f", mass {vv.mass_source}" if vv else ""))
    if log:
        console.say(f"log: {log.path}")
    script = parse_script(a.script) if a.script else None
    if a.fast:
        tv = [0.0]

        def clock() -> float:
            tv[0] += FAST_STEP_S
            return tv[0]
    else:
        t0 = time.monotonic()
        speed = a.speed

        def clock() -> float:
            return (time.monotonic() - t0) * speed
    max_time = a.max_time
    if a.fast and max_time is None:
        max_time = 600.0
    try:
        run_live(st, console, clock, fast=a.fast, script=script, max_time_s=max_time, interactive=not a.fast)
    finally:
        transport.close()
        if log:
            log.close()
        console.close()
    if log:
        print_summary(console, summarize_log(read_log(log.path)), vv.truth() if vv else None)
        console.say(f"log: {log.path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
