"""v0.9 power-source identifiers, reason codes and the TELEM v4 / v5 power tail (tether / hybrid builds).

Pure Python (no numpy).  ``sim.onboard`` (the supervisor's power rules) and ``sim.ground_station_logic`` (the operator
texts) import it; nothing here knows about time or the vehicle state.

Power builds (a build-time choice of the FCU firmware, orthogonal to the field / demo profiles; tether proposal
firmware.build_profiles - named 'wired' in code to avoid the collision with the mechanical ``fcu_bench_tether``):

    PWR_BATTERY       0  the flight pack (the v0.8.1 rules unchanged: I8 on V_cell = V_in / 6)
    PWR_WIRED         1  battery-less, powered over the tether (params power.source 'tether'): V_in = the tether node
    PWR_WIRED_BUFFER  2  tether + 6S buffer behind the ideal diodes D1 / D2 (params power.source 'hybrid'): V_tether before
                         D1, V_buffer before D2

Reason codes of the power rules (interlock range 1..63, a block of their own from 0x30 so the codes link_proto still
has to assign to other rules - e.g. the V16 yaw-authority code the control round asked for - cannot collide):

    R_W_LOST   0x30  wired_buffer: the tether is lost (V_tether low or below the buffer) -> LAND on the buffer
    R_W_SAG    0x31  wired: the supply sags / is in its current limit (V_in low) -> throttle ceiling frozen, then LAND
    R_W_BUF    0x32  wired_buffer: the buffer below buf_low_cell_v under load -> LAND
    R_W_D2     0x33  wired_buffer: the buffer is being charged while tethered (D2 shorted) -> LAND
    REJ_POWER  0x58  ON / UP refused: the supply / tether / buffer voltage is outside the ON window (W_ARM), or UP while
                     the supply is sagging

v0.9 firmware round (RP1, fw09 SPEC 6.1): ``sim.link_proto`` defines the codes and their English names (the same values
and texts this module registered before); the names here are re-exported from it, and ``register_reason_names`` (still
called by its importers) is a no-op on the adopted entries (``setdefault``: link_proto's own entry wins).

TELEM v4 / v5 power tail (RP1: ``sim.link_proto`` TYPE_TELEM4 0x23 / TYPE_TELEM5 0x24, the COM firmware builds them from
the FCU's ilink SNAP2 / SNAP3 power bytes, the 6-DOF's ground link carries them in the tether / hybrid builds): 2 bytes
appended to the build's frame, a new TYPE each (one fixed length per TYPE, the link_proto rule); a frame carries the tail
exactly when its build bits are not the battery build (``link_proto.power_tail_due``):

    v_aux     u8  0.125 V (0 .. 31.875 V): the buffer voltage in wired_buffer, V_tether (= V_in) in wired
    pwr_flags u8  PF_* below; bits 6-7 = the power build

    TELEM v4 = TELEM v2 + tail: 33 payload bytes, 41-byte frame (the indoor tether builds without the GPS)
    TELEM v5 = TELEM v3 + tail: 39 payload bytes, 47-byte frame (a tether build with the GPS); LEN_MAX 40 -> 42

V_in stays in the TELEM vcell field (V_in / 6, 1 mV per cell): in the battery-less build it IS the tether voltage, in the
hybrid the bus (the tether's while it powers the vehicle).  Why 2 bytes and not 3 (V_tether + V_buffer + flags): on the
lora_uart backend a 48-byte reply (TELEM v3 + 3) gives a round trip of 0.502 s = 1.99 Hz, below the 2 Hz poll; 47 bytes
give 2.01 Hz (``telem4_budget``; sx127x: 4.3-4.4 Hz).

Invariant: the codes, flag bits and scalings here are the single definition the supervisor and the ground station
share; the encode / decode pair is exact (decode(encode(x)) == x for every in-range integer field).
"""
from __future__ import annotations

from dataclasses import dataclass

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import link_proto as _lp
except ImportError:  # pragma: no cover
    import link_proto as _lp  # type: ignore

PWR_BATTERY, PWR_WIRED, PWR_WIRED_BUFFER = 0, 1, 2
PWR_NAMES = {PWR_BATTERY: "battery", PWR_WIRED: "wired", PWR_WIRED_BUFFER: "wired_buffer"}
PWR_NAMES_KO = {PWR_BATTERY: "배터리", PWR_WIRED: "유선(배터리 없음)", PWR_WIRED_BUFFER: "하이브리드(유선 + 버퍼 배터리)"}
# params power.source / the sim's plant names -> the build
SOURCE_TO_BUILD = {"battery": PWR_BATTERY, "tether": PWR_WIRED, "wired": PWR_WIRED, "hybrid": PWR_WIRED_BUFFER,
                   "wired_buffer": PWR_WIRED_BUFFER}

R_W_LOST = _lp.R_W_LOST          # 0x30 (RP1: defined in sim.link_proto)
R_W_SAG = _lp.R_W_SAG            # 0x31
R_W_BUF = _lp.R_W_BUF            # 0x32
R_W_D2 = _lp.R_W_D2              # 0x33
REJ_POWER = _lp.REJ_POWER        # 0x58
POWER_INTERLOCK_REASONS = _lp.POWER_INTERLOCK_REASONS
POWER_REJECT_REASONS = _lp.POWER_REJECT_REASONS
POWER_REASON_NAMES = {c: _lp.REASON_NAMES[c] for c in POWER_INTERLOCK_REASONS + POWER_REJECT_REASONS}

# pwr_flags (TELEM v4 / v5 tail; OnboardOutputs.pwr)
PF_TETHER_OK = 0x01      # tethered builds: V_tether in its normal range (not lost)
PF_SAG = 0x02            # wired: the supply is sagging - throttle ceiling frozen, UP refused (W_SAG stage 1)
PF_BUF_LOW = 0x04        # wired_buffer: the buffer below buf_low_cell_v or its landing reserve (W_BUF condition)
PF_BUF_SHARE = 0x08      # wired_buffer: the buffer is discharging (the board's current-sign output)
PF_LOST = 0x10           # wired_buffer: W_LOST latched (the tether was lost in this flight)
PF_D2 = 0x20             # wired_buffer: the buffer is being charged (W_D2 condition)
PF_BUILD_SHIFT = 6       # bits 6-7: the power build (PWR_*)
PF_NAMES = {PF_TETHER_OK: "tether_ok", PF_SAG: "sag", PF_BUF_LOW: "buf_low", PF_BUF_SHARE: "buf_share",
            PF_LOST: "lost", PF_D2: "d2"}
V_SCALE = 8.0            # u8 per volt (0.125 V)
TAIL_BYTES = 2
TELEM2_PAYLOAD_LEN_REF = 31      # link_proto.TELEM2_PAYLOAD_LEN (checked in the tests)
TELEM3_PAYLOAD_LEN_REF = 37      # link_proto.TELEM3_PAYLOAD_LEN
FRAME_OVERHEAD_REF = 8           # sync 2 + LEN 1 + VEH TYPE SEQ 3 + CRC 2
TELEM4_FRAME_BYTES = FRAME_OVERHEAD_REF + TELEM2_PAYLOAD_LEN_REF + TAIL_BYTES     # 41 (= link_proto.TELEM4_FRAME_BYTES)
TELEM5_FRAME_BYTES = FRAME_OVERHEAD_REF + TELEM3_PAYLOAD_LEN_REF + TAIL_BYTES     # 47 (= link_proto.TELEM5_FRAME_BYTES)
if (TELEM4_FRAME_BYTES, TELEM5_FRAME_BYTES, PF_BUILD_SHIFT) != (_lp.TELEM4_FRAME_BYTES, _lp.TELEM5_FRAME_BYTES,
                                                               _lp.PWR_TAIL_BUILD_SHIFT):
    raise ImportError("sim.power_link: the TELEM v4 / v5 tail disagrees with sim.link_proto")


def build_of_source(src: str | None) -> int:
    """PWR_* of a params power.source / plant source name ('battery', 'tether' / 'wired', 'hybrid' / 'wired_buffer')."""
    if src is None:
        return PWR_BATTERY
    if src not in SOURCE_TO_BUILD:
        raise ValueError(f"power source {src!r} not in {sorted(SOURCE_TO_BUILD)}")
    return SOURCE_TO_BUILD[src]


def register_reason_names(lp_mod) -> None:
    """Add the power codes' English names to ``lp_mod.REASON_NAMES`` (``setdefault``: an existing entry wins)."""
    for c, n in POWER_REASON_NAMES.items():
        lp_mod.REASON_NAMES.setdefault(c, n)


def quant_v(v: float | None) -> int:
    """Volts -> the u8 tail field (0.125 V, round half up, clamped 0..255; None / NaN -> 0)."""
    if v is None or v != v:
        return 0
    q = int(v * V_SCALE + 0.5) if v > 0.0 else 0
    return min(max(q, 0), 255)


@dataclass(frozen=True)
class PowerTelem:
    """The TELEM v4 / v5 power tail (wire integers; physical values are properties)."""

    v_aux_q: int = 0
    flags: int = 0

    @property
    def v_aux_v(self) -> float:
        return self.v_aux_q / V_SCALE

    @property
    def build(self) -> int:
        return (self.flags >> PF_BUILD_SHIFT) & 0x03

    def flag(self, bit: int) -> bool:
        return bool(self.flags & bit)

    def encode(self) -> bytes:
        for x in (self.v_aux_q, self.flags):
            if not 0 <= int(x) <= 255:
                raise ValueError(f"power tail field {x} outside u8")
        return bytes((self.v_aux_q, self.flags))

    @classmethod
    def decode(cls, b: bytes) -> "PowerTelem":
        if len(b) != TAIL_BYTES:
            raise ValueError(f"power tail is {TAIL_BYTES} bytes, got {len(b)}")
        return cls(b[0], b[1])

    @classmethod
    def from_physical(cls, v_aux: float | None, flags: int, build: int) -> "PowerTelem":
        return cls(quant_v(v_aux), (int(flags) & 0x3F) | ((int(build) & 0x03) << PF_BUILD_SHIFT))


def telem4_budget(cfg, lm_mod=None) -> dict:
    """Downlink time and round trip of the TELEM v4 (41 B) and v5 (47 B) frames against TELEM v2 (39 B) and v3 (45 B) for
    ``cfg`` (a ``sim.link_model.LinkConfig``): uplink + the on-vehicle reply delay + downlink, the reply timeout and the
    achievable poll rate (the ``round_trip`` formulas; RP1: ``link_model.round_trip(cfg, mode='vtol', power=True)`` gives
    the same telem4 / telem5 numbers), plus the rejected 3-byte tail on TELEM v3 (48 B)."""
    if lm_mod is None:
        try:
            from sim import link_model as lm_mod  # type: ignore
        except ImportError:  # pragma: no cover
            import link_model as lm_mod  # type: ignore
    lp_ = lm_mod.lp
    out: dict = {"radio": cfg.radio, "poll_hz": cfg.poll_hz}
    up = lm_mod.hop_timing(cfg, "up", lp_.CMD_FRAME_BYTES).arrival
    for name, nb in (("telem2", lp_.TELEM2_FRAME_BYTES), ("telem3", lp_.TELEM3_FRAME_BYTES), ("telem4", TELEM4_FRAME_BYTES),
                     ("telem5", TELEM5_FRAME_BYTES), ("telem3_plus3", lp_.TELEM3_FRAME_BYTES + 3)):
        hop = lm_mod.hop_timing(cfg, "down", nb)
        worst = up + lm_mod.FC_REPLY_DELAY_S + hop.arrival
        allowed = up + lm_mod.FC_REPLY_ALLOWANCE_S + hop.arrival
        out[name] = {"frame_bytes": nb, "air_s": hop.parts["air"], "downlink_s": hop.arrival, "rtt_s": worst,
                     "reply_timeout_s": lm_mod.REPLY_TIMEOUT_FACTOR_ASSUMED * allowed + lm_mod.REPLY_TIMEOUT_MARGIN_S_ASSUMED,
                     "max_poll_hz": 1.0 / worst, "poll_ok": 1.0 / worst >= cfg.poll_hz}
    return out
