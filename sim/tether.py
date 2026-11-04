"""v0.9 power plant (battery | tether | hybrid) and tether mechanics of the VTOL hardware configuration (6-DOF plant
side only: nothing here reaches the flight computer except the measured signals of ``PowerSystem.measure``).

Sources: docs/tether_power_proposal.json (sim.mechanics / sim.power / model_assumptions), docs/유선전원_검토.md,
docs/v0.9_하드웨어_변경.md section 5, sizing/tw_table_v09.json; numbers from params.json ``edf`` / ``power`` (the
v0.9 plant blocks ``edf.thrust_v09``, ``power.electrical_plant_ASSUMED``, ``power.tether_plant_ASSUMED``) and the
dataclass defaults below, each marked ASSUMED where nothing was measured.

Electrics (one bus = the ESC input, evaluated once per control tick on the lagged thrust; closed forms, no iteration):

* fan electrical power   P(T) = P_ref (T / T_ref)^1.5 (proxy table 2264 W at 30.30 N / 22.2 V); P1 'proxy efficiency'
  T_ref = 30.30 N, P2 'pessimistic' T_ref = the plant's T_max at 22.2 V (the fan absorbs 2264 W at full throttle but
  makes only T_max) - ``FanPower``;
* thrust at the bus voltage V: T_max(V) = T_max(22.2 V) (V / 22.2)^k, k = 1.61 (the proxy table's 22.2 -> 25.2 V ratio)
  - applied by ``sim.sim6dof.FlightSim.plant_thrust`` through v_cell = V / 6;
* battery: 6S, OCV per cell from the state of charge (``OCV_TABLE`` ASSUMED typical LiPo), pack resistance R_int (18 mOhm
  ASSUMED, the sizing value) - V = (V_oc + sqrt(V_oc^2 - 4 R P)) / 2;
* tether: supply CC/CV at v_set with the current limit i_limit (RSP-1500-24 class; overload -> constant current, latch-off
  after latch_s in the CC region), loop resistance R_loop (8.5 m 10 AWG + XT90-S, 59.5 mOhm); V_esc = V_s - I R_loop;
* hybrid: tether D1 and buffer D2 ideal diodes (forward drop diode_v, LM74700 regulation 20 mV) OR-ed on the bus; the buffer
  (6S 650 mAh, OCV(SOC), R_int 30 mOhm ASSUMED) discharges only (D2); crossover I_x = (V_s - V_buf) / R_loop (40.3 A at
  3.80 V / cell);
* a load the sources cannot carry (P(T) above the maximum deliverable power) puts the bus at its maximum-power point and
  limits the thrust the ESC can hold (``BusState.p_max_w`` -> ``FlightSim``); below ``esc_min_v`` the ESC stops, below
  ``bec_min_v`` the avionics brown out (ASSUMED values; the sim keeps the FCU logic running - its outputs then have no
  effect: no thrust, servos unpowered and held);
* events (``PowerEvent``): unplug (the XT90-S at the vehicle: the tether side is gone, mechanics unchanged), cut (the
  conductors: tether power gone AND the cable's feed force gone - the catch line stays), supply_trip (latched off), sag
  (the supply set point ramps to ``value`` V over ``ramp_s``), r_loop (loop resistance x ``value``: a long / thin tether);
* measured signals (``measure``): V_in = V_bus through the 47k / 4.7k divider to a 10-bit ADC (2.56 V reference: 28.2 V
  full scale, 1 LSB RMS noise ASSUMED, pin chosen in the firmware round), V_tether (before D1) and V_buffer (before D2)
  with the same divider (proposal: two free ADC pins), the buffer-current sign (hybrid board, ASSUMED available).

Mechanics (``TetherConfig``, ``catch_line_force``, ``cable_feed_force``): an overhead anchor (pulley on a gantry) at
anchor_height above the ground; two elements act on the top-cap eye bolt (``Vehicle.tether_attach_body_m``):

* the power cable: feed 'tension' = a constant-tension feed (handler / balancer) T_c at the vehicle along the line to the
  pulley, a first-order Gauss-Markov wander (sigma, tau 2 s ASSUMED; the proposal's 0.5-2 N), or feed 'slack' = the handler
  pays out ``cable_slack_m`` more than the straight distance and the cable hangs as a catenary between the eye bolt and the
  pulley (weight per length ``w_kg_per_m``: 2 x 10 AWG silicone + Dyneema, 0.139 kg/m from the proposal's 230 g / 1.5 m /
  1.10); plus half the drag of the span (cross-flow C_d ASSUMED on the bundle width);
* the catch line (strength member + bungee shock section): the handler pays out L0 so that it keeps ``belay_slack_m`` of
  slack (first-order follow, tau_h, rate limit, a belay lock above lock_tension_n - a fall is faster than the handler);
  taut (D > L0) it is a spring-damper k_b (600 N/m ASSUMED), zeta 0.3 ASSUMED on the vehicle mass (the Dyneema in series,
  EA ~3e5 N, is stiff by comparison and neglected);
* snag: a jerk F_j (20 N) for 0.2 s at 30 deg from the vertical (proposal W5), half-sine, at the eye bolt.

Not modelled (limits stated): the cable's own dynamics (waves, swing of the hanging loop), the pulley friction, the
handler's reaction beyond a first-order follow, the bus capacitance hold-up (ms), regeneration into the bus (the clamp
is assumed to absorb it), conductor heating (the resistance stays at 20 C).

Invariant: pure functions / small state machines on floats (numpy only for the catenary solve); every constant is a
field with its source; the FCU never reads anything but ``measure``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

# ---- LiPo open-circuit voltage per cell vs state of charge (ASSUMED typical curve; measure the packs) ----
OCV_SOC = (0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00)
OCV_V = (3.27, 3.50, 3.61, 3.67, 3.70, 3.75, 3.79, 3.83, 3.87, 3.93, 4.00, 4.08, 4.20)
SOURCES = ("battery", "tether", "hybrid")
EVENT_KINDS = ("unplug", "cut", "supply_trip", "sag", "r_loop")
BELAY_DEVICES = ("lagged", "srl", "autolock")      # catch-line device (TetherConfig.belay_device; fix round)
SRL_EPS_M = 0.005          # an unlocked device keeps L0 this far beyond the distance (no bungee stretch inside a step)
RELEASE_LOAD_N = 2.0       # a locked device counts as unloaded at or below this catch-line tension (ASSUMED)
CATCH_LOADED_N = 2.0       # metrics: the catch line 'carries load' above this (an SRL's retract force is below it)
SUPPLY_OFF, SUPPLY_CV, SUPPLY_CC, SUPPLY_TRIPPED = 0, 1, 2, 3
SUPPLY_NAMES = {SUPPLY_OFF: "none", SUPPLY_CV: "CV", SUPPLY_CC: "CC", SUPPLY_TRIPPED: "tripped"}


def ocv_cell(soc: float) -> float:
    """Resting cell voltage at a state of charge (``OCV_TABLE``, linear interpolation, clipped to 0..1)."""
    s = min(max(float(soc), 0.0), 1.0)
    for i in range(1, len(OCV_SOC)):
        if s <= OCV_SOC[i]:
            f = (s - OCV_SOC[i - 1]) / (OCV_SOC[i] - OCV_SOC[i - 1])
            return OCV_V[i - 1] + f * (OCV_V[i] - OCV_V[i - 1])
    return OCV_V[-1]


def soc_of_ocv(v_cell: float) -> float:
    """Inverse of ``ocv_cell`` (clipped to the table)."""
    v = float(v_cell)
    if v <= OCV_V[0]:
        return 0.0
    for i in range(1, len(OCV_V)):
        if v <= OCV_V[i]:
            f = (v - OCV_V[i - 1]) / (OCV_V[i] - OCV_V[i - 1])
            return OCV_SOC[i - 1] + f * (OCV_SOC[i] - OCV_SOC[i - 1])
    return 1.0


# --------------------------------------------------------------------------- #
# fan power
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class FanPower:
    """Electrical input power of the EDF vs thrust: P = p_ref (T / t_ref)^exponent (tether proposal
    model_assumptions.power_vs_thrust; sizing/vtol_sizing.py).  ``t_ref`` = 30.30 N for P1 (proxy efficiency), = the
    plant's T_max at 22.2 V for P2 (pessimistic)."""

    p_ref_w: float = 2264.0
    t_ref_n: float = 30.302548499999997
    exponent: float = 1.5

    def power(self, thrust_n: float) -> float:
        t = thrust_n if thrust_n > 0.0 else 0.0
        return self.p_ref_w * (t / self.t_ref_n) ** self.exponent

    def thrust_at_power(self, p_w: float) -> float:
        p = p_w if p_w > 0.0 else 0.0
        return self.t_ref_n * (p / self.p_ref_w) ** (1.0 / self.exponent)


def fan_power_from_params(p: dict, t_max_22_n: float, model: float = 0.0) -> FanPower:
    """``model`` 0 = P1 (t_ref = the proxy table's 30.30 N), 1 = P2 (t_ref = the plant's T_max at 22.2 V); values in
    between interpolate t_ref linearly (the campaign's efficiency spread)."""
    e = p["edf"]
    tv = e.get("thrust_v09") or {}
    t1 = float(e["thrust_nominal_N"])
    t_ref = t1 + float(model) * (float(t_max_22_n) - t1)
    return FanPower(float(e["power_nominal_W"]), t_ref, float(tv.get("power_exponent", 1.5)))


# --------------------------------------------------------------------------- #
# electrical configuration, events, state
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class PowerEvent:
    """A scripted power event: ``kind`` in EVENT_KINDS at absolute time ``t_s``; ``value``: sag -> the supply set point
    [V] reached after ``ramp_s``; r_loop -> the loop-resistance factor."""

    t_s: float
    kind: str
    value: float = 0.0
    ramp_s: float = 0.0

    def __post_init__(self) -> None:
        if self.kind not in EVENT_KINDS:
            raise ValueError(f"power event {self.kind!r} not in {EVENT_KINDS}")


@dataclass(frozen=True)
class PowerConfig:
    """One run's electrical plant (defaults: params power.electrical_plant_ASSUMED via ``from_params``)."""

    source: str = "battery"
    # main pack (battery source)
    cells: int = 6
    capacity_mah: float = 2200.0
    r_int_ohm: float = 0.018                   # pack resistance ASSUMED (sizing: 18 mOhm)
    v_rest_cell: float = 4.15                  # resting cell voltage at the start (SimOptions.batt_v_full_cell)
    # supply + tether
    v_set: float = 25.2                        # RSP-1500-24 set point (power.supply.v_set)
    i_limit_a: float = 60.0                    # CC limit (architect: 60 A; datasheet CC onset 105-135 % of 63 A)
    latch_s: float = 5.0                       # time in CC before the latch-off (datasheet: 5 s)
    r_loop_ohm: float = 0.0595                 # route 58.5 + XT90-S 1.0 mOhm (power.supply)
    # hybrid buffer (D2) and the ideal diodes
    buf_cells: int = 6
    buf_capacity_mah: float = 650.0
    buf_r_int_ohm: float = 0.030               # ASSUMED (proposal sim.power.buffer)
    buf_v_rest_cell: float = 3.80              # storage voltage (power.hybrid_extra_ASSUMED.buffer_pack)
    diode_v: float = 0.02                      # LM74700-Q1 forward regulation 20 mV
    # ESC / avionics
    esc_min_v: float = 12.0                    # ESC stops below this bus voltage (ASSUMED)
    bec_min_v: float = 6.0                     # avionics brown-out (ASSUMED)
    # ADC (V_in divider 47k / 4.7k, 2.56 V reference, 10 bit; noise ASSUMED)
    adc_full_scale_v: float = 28.16
    adc_bits: int = 10
    adc_noise_lsb: float = 1.0
    buf_i_sign_threshold_a: float = 0.5        # buffer-current sign output of the hybrid board (ASSUMED)
    events: tuple = ()
    # ---- v0.9 fix round (physics review medium: ADC gain, regeneration); every default = off (bit-identical) ----
    # per-channel ADC gain error (V_in, V_tether, V_buffer): the ATmega128A internal 2.56 V reference is 2.3-2.7 V
    # (-10 / +5.5 %) and the 47k / 4.7k divider +-1-2 %; reading = V (1 + err) before the quantiser.  0 = calibrated
    # exactly; the campaign samples the two-point-calibrated residual +-0.5 % (procedure: calibrate before a tether flight)
    adc_gain_err: tuple = (0.0, 0.0, 0.0)
    # regeneration (ESC active freewheeling on a throttle reduction): when the commanded thrust falls faster than the
    # rotor coasts down, P_regen = regen_eff (I omega |domega/dt| - P_shaft) flows back into the bus.  0 = off (the
    # v0.9 plant).  A battery takes it; a tether / hybrid bus cannot sink current (supply, ideal diodes): the bus
    # capacitance charges, the brake chopper (clamp_r_ohm on at clamp_v_on, off at clamp_v_off; proposal cad.bus_clamp,
    # ASSUMED values) dumps it, and above the supply's OVP (RSP-1500-24: 27.6-32.4 V) the supply latches off.
    regen_eff: float = 0.0
    motor_eff: float = 0.85                    # shaft / electrical power of the fan drive (ASSUMED)
    bus_c_f: float = 4.0e-3                    # ESC cap bank 4 x 1000 uF (proposal model_assumptions)
    clamp_v_on: float = 26.6
    clamp_v_off: float = 26.0
    clamp_r_ohm: float = 1.5                   # proposal: 1-2 Ohm; math.inf = no clamp
    ovp_v: float = 27.6                        # supply OVP trip (datasheet minimum 27.6 V)
    i_aux_a: float = 0.5                       # avionics + servo draw that the bus always carries (ASSUMED)

    def __post_init__(self) -> None:
        if self.source not in SOURCES:
            raise ValueError(f"power source {self.source!r} not in {SOURCES}")

    @classmethod
    def from_params(cls, p: dict, source: str = "battery", **kw: Any) -> "PowerConfig":
        pw = p.get("power") or {}
        ep = pw.get("electrical_plant_ASSUMED") or {}
        sup = pw.get("supply") or {}
        hx = (pw.get("hybrid_extra_ASSUMED") or {}).get("buffer_pack") or {}
        b = p.get("battery") or {}
        d: dict[str, Any] = {"source": source}
        if b:
            d.update(cells=int(b.get("cells", 6)), capacity_mah=float(b.get("capacity_mAh", 2200.0)))
        if sup:
            d.update(v_set=float(sup.get("v_set", 25.2)), i_limit_a=float(sup.get("rated_A", 60.0)),
                     r_loop_ohm=(float(sup.get("route_loop_mohm", 58.5)) + float(sup.get("connector_mohm_ASSUMED", 1.0))) * 1e-3)
        if hx:
            d.update(buf_capacity_mah=float(hx.get("capacity_mAh", 650.0)),
                     buf_v_rest_cell=float(hx.get("storage_v_per_cell", 3.80)))
        for k_param, k_cfg in (("battery_r_int_ohm", "r_int_ohm"), ("supply_latch_s", "latch_s"),
                               ("buffer_r_int_ohm", "buf_r_int_ohm"), ("diode_v", "diode_v"), ("esc_min_v", "esc_min_v"),
                               ("bec_min_v", "bec_min_v"), ("adc_full_scale_v", "adc_full_scale_v"),
                               ("adc_noise_lsb", "adc_noise_lsb")):
            if k_param in ep:
                d[k_cfg] = float(ep[k_param])
        d.update(kw)
        return cls(**d)


@dataclass
class BusState:
    """The bus of one tick (all SI)."""

    v_bus: float = 0.0               # ESC input voltage
    i_load: float = 0.0              # ESC input current
    i_tether: float = 0.0            # through D1
    i_buffer: float = 0.0            # through D2 (>= 0) or the main pack
    v_tether: float = 0.0            # before D1 at the vehicle (0 when disconnected)
    v_buffer: float = 0.0            # buffer / pack terminal
    v_supply: float = 0.0            # supply terminal
    supply_state: int = SUPPLY_OFF
    soc: float = 1.0                 # main pack
    soc_buffer: float = float("nan")
    p_load_w: float = 0.0
    p_max_w: float = float("inf")    # deliverable maximum (thrust limit when exceeded)
    limited: bool = False            # the load exceeded the deliverable power this tick
    connected: bool = False          # tether power present at the vehicle
    esc_ok: bool = True              # bus above esc_min_v
    avionics_ok: bool = True         # bus above bec_min_v


class PowerSystem:
    """The electrical plant of one run (state: pack / buffer charge, the supply's CC timer and latch, the events)."""

    def __init__(self, cfg: PowerConfig, fan: FanPower, seed: int = 0):
        self.cfg = cfg
        self.fan = fan
        self.rng = np.random.default_rng([int(seed), 0x7E7])
        self.q_c = cfg.capacity_mah * 3.6           # coulombs
        self.qb_c = cfg.buf_capacity_mah * 3.6
        self.soc = soc_of_ocv(cfg.v_rest_cell) if cfg.source == "battery" else float("nan")
        self.soc_b = soc_of_ocv(cfg.buf_v_rest_cell) if cfg.source == "hybrid" else float("nan")
        self.tethered = cfg.source in ("tether", "hybrid")
        self.plugged = self.tethered
        self.cable_cut = False
        self.tripped = False
        self.cc_time = 0.0
        self.v_set = cfg.v_set
        self.r_loop = cfg.r_loop_ohm
        self.events = sorted(cfg.events, key=lambda e: e.t_s)
        self.sag: tuple[float, float, float, float] | None = None   # (t0, t1, v0, v1)
        self.log: list[tuple[float, str]] = []
        self.last = BusState()
        # regeneration (fix round; ``set_rotor`` from the flight sim): rotor energy at the fan's T_max, the thrust lag
        self.rotor_e_full_j = 0.0
        self.rotor_t_fan_n = 1.0
        self.rotor_tau_s = 0.15
        self.v_cap: float | None = None            # bus capacitor voltage while the regeneration holds the bus up
        self.chopper_on = False
        self.regen_j = 0.0                         # energy returned to the bus (diagnostics)
        self.clamp_j = 0.0                         # energy the brake chopper dumped
        self.p_regen_w = 0.0

    def set_rotor(self, e_full_j: float, t_fan_n: float, tau_s: float) -> None:
        """Regeneration model inputs: the rotor's kinetic energy at the fan's T_max (0.5 I omega_ref^2; omega^2 ~ T),
        that T_max and the plant's thrust time constant (the rotor follows the thrust lag)."""
        self.rotor_e_full_j = float(e_full_j)
        self.rotor_t_fan_n = max(float(t_fan_n), 1e-9)
        self.rotor_tau_s = max(float(tau_s), 1e-6)

    def regen_power(self, thrust_n: float, t_cmd_n: float | None) -> float:
        """Power the ESC returns to the bus this tick [W] (>= 0): with E = e_full T / T_fan and dT/dt = (t_cmd - T) / tau,
        regen = eff (-dE/dt - motor_eff P_elec(T)) when the rotor is braked faster than it coasts; 0 with the motor
        commanded off (the ESC coasts: brake-on-stop OFF is a procedure item) or regen_eff 0."""
        c = self.cfg
        if c.regen_eff <= 0.0 or t_cmd_n is None or t_cmd_n <= 0.0 or self.rotor_e_full_j <= 0.0:
            return 0.0
        tp = max(thrust_n, 0.0)
        de = self.rotor_e_full_j / self.rotor_t_fan_n * (t_cmd_n - tp) / self.rotor_tau_s
        net = c.motor_eff * self.fan.power(tp) + de
        return c.regen_eff * (-net) if net < 0.0 else 0.0

    # ---- events ----
    def _apply_events(self, t: float) -> None:
        while self.events and self.events[0].t_s <= t:
            e = self.events.pop(0)
            if e.kind == "unplug":
                self.plugged = False
                self.log.append((t, "POWER tether unplugged at the vehicle (XT90-S)"))
            elif e.kind == "cut":
                self.plugged = False
                self.cable_cut = True
                self.log.append((t, "POWER tether cut (conductors and cable feed gone; catch line intact)"))
            elif e.kind == "supply_trip":
                self.tripped = True
                self.log.append((t, "POWER supply tripped (latched off)"))
            elif e.kind == "sag":
                self.sag = (t, t + max(e.ramp_s, 0.0), self.v_set, float(e.value))
                self.log.append((t, f"POWER supply set point ramps {self.v_set:.2f} -> {e.value:.2f} V over {e.ramp_s:.1f} s"))
            elif e.kind == "r_loop":
                self.r_loop = self.cfg.r_loop_ohm * float(e.value)
                self.log.append((t, f"POWER loop resistance x{e.value:.2f} = {self.r_loop * 1e3:.1f} mOhm"))
        if self.sag is not None:
            t0, t1, v0, v1 = self.sag
            f = 1.0 if t >= t1 else (t - t0) / max(t1 - t0, 1e-9)
            self.v_set = v0 + f * (v1 - v0)

    # ---- the bus ----
    def step(self, t: float, dt: float, thrust_n: float, t_cmd_n: float | None = None) -> BusState:
        """Advance to time t (events), solve the bus for the fan power of the (lagged) thrust, integrate the charge over
        dt.  Returns the BusState of this tick.  ``t_cmd_n`` (the steady thrust of the ESC command; fix round) feeds the
        regeneration model (``regen_power``; off unless cfg.regen_eff > 0)."""
        c = self.cfg
        self._apply_events(t)
        p_reg = self.regen_power(thrust_n, t_cmd_n)
        self.p_regen_w = p_reg
        if p_reg > 0.0:
            return self._step_regen(t, dt, p_reg)
        self.v_cap = None
        self.chopper_on = False
        p = self.fan.power(thrust_n)
        st = BusState(p_load_w=p, soc=self.soc, soc_buffer=self.soc_b)
        tether_up = self.tethered and self.plugged and not self.tripped
        st.connected = tether_up
        if c.source == "battery":
            voc = c.cells * ocv_cell(self.soc)
            v, lim, pmax = _single_source(voc, c.r_int_ohm, math.inf, p)
            st.v_bus, st.limited, st.p_max_w = v, lim, pmax
            st.i_load = st.i_buffer = (p if not lim else pmax) / v if v > 1e-9 else 0.0
            st.v_buffer = v
            self.soc = max(self.soc - st.i_buffer * dt / self.q_c, 0.0)
        else:
            vs = self.v_set if tether_up else 0.0
            vb = c.buf_cells * ocv_cell(self.soc_b) if c.source == "hybrid" else 0.0
            v, it, ib, lim, pmax, cc = _hybrid_bus(vs, self.r_loop, c.i_limit_a, vb, c.buf_r_int_ohm, c.diode_v, p,
                                                   tether_up, c.source == "hybrid")
            st.v_bus, st.i_tether, st.i_buffer, st.limited, st.p_max_w = v, it, ib, lim, pmax
            st.i_load = it + ib
            st.v_supply = vs if not cc else min(vs, v + (c.diode_v if c.source == "hybrid" else 0.0) + it * self.r_loop)
            st.v_tether = (st.v_supply - it * self.r_loop) if tether_up else 0.0
            st.v_buffer = (vb - ib * c.buf_r_int_ohm) if c.source == "hybrid" else 0.0
            if tether_up:
                if cc:
                    self.cc_time += dt
                    st.supply_state = SUPPLY_CC
                    if self.cc_time >= c.latch_s:
                        self.tripped = True
                        self.log.append((t, f"POWER supply latched off after {self.cc_time:.2f} s in constant current "
                                            f"({c.i_limit_a:.0f} A)"))
                else:
                    self.cc_time = 0.0
                    st.supply_state = SUPPLY_CV
            else:
                st.supply_state = SUPPLY_TRIPPED if self.tripped else SUPPLY_OFF
            if c.source == "hybrid":
                self.soc_b = max(self.soc_b - ib * dt / self.qb_c, 0.0)
        st.soc, st.soc_buffer = self.soc, self.soc_b
        st.esc_ok = st.v_bus >= c.esc_min_v
        st.avionics_ok = st.v_bus >= c.bec_min_v
        self.last = st
        return st

    def _step_regen(self, t: float, dt: float, p_reg: float) -> BusState:
        """A tick with the ESC regenerating ``p_reg`` W into the bus (fix round, physics review medium).  Battery: the pack
        absorbs it.  Tether / hybrid: the supply and the ideal diodes cannot sink current - the bus capacitance charges
        (20 sub-steps), the brake chopper dumps above clamp_v_on (hysteresis to clamp_v_off), and a bus above the
        supply's OVP latches the supply off (a permanent tether power loss: logged as a POWER event)."""
        c = self.cfg
        st = BusState(p_load_w=-p_reg, soc=self.soc, soc_buffer=self.soc_b)
        tether_up = self.tethered and self.plugged and not self.tripped
        self.regen_j += p_reg * dt
        if c.source == "battery":
            voc = c.cells * ocv_cell(self.soc)
            i = -p_reg / max(voc, 1.0)
            v = voc - i * c.r_int_ohm
            st.v_bus, st.i_load, st.i_buffer, st.v_buffer = v, i, i, v
            self.soc = min(self.soc - i * dt / self.q_c, 1.0)
        else:
            floor_v = self.v_set if tether_up else 0.0
            if c.source == "hybrid":
                floor_v = max(floor_v, c.buf_cells * ocv_cell(self.soc_b) - c.diode_v)
            v = self.v_cap if self.v_cap is not None else max(self.last.v_bus, 0.0)
            v = max(v, floor_v)                       # below its set point the supply (or the buffer) holds the bus
            n = 20
            h = dt / n
            for _ in range(n):
                if self.chopper_on and v <= c.clamp_v_off:
                    self.chopper_on = False
                elif not self.chopper_on and v >= c.clamp_v_on:
                    self.chopper_on = True
                i_ch = v / c.clamp_r_ohm if (self.chopper_on and math.isfinite(c.clamp_r_ohm) and c.clamp_r_ohm > 0.0) else 0.0
                self.clamp_j += v * i_ch * h
                v = max(v + (p_reg / max(v, 1.0) - c.i_aux_a - i_ch) / c.bus_c_f * h, floor_v)
                if tether_up and v >= c.ovp_v:
                    self.tripped = True
                    tether_up = False
                    floor_v = (c.buf_cells * ocv_cell(self.soc_b) - c.diode_v) if c.source == "hybrid" else 0.0
                    self.log.append((t, f"POWER supply OVP latched off: bus {v:.2f} V >= {c.ovp_v:.2f} V on regeneration "
                                        f"({p_reg:.0f} W into a bus that cannot sink current)"))
            self.v_cap = v
            st.v_bus, st.i_load = v, -p_reg / max(v, 1.0)
            st.v_supply = self.v_set if tether_up else 0.0
            st.v_tether = st.v_supply
            st.v_buffer = (c.buf_cells * ocv_cell(self.soc_b)) if c.source == "hybrid" else 0.0
            st.supply_state = SUPPLY_CV if tether_up else (SUPPLY_TRIPPED if self.tripped else SUPPLY_OFF)
        st.connected = tether_up
        st.soc, st.soc_buffer = self.soc, self.soc_b
        st.esc_ok = st.v_bus >= c.esc_min_v
        st.avionics_ok = st.v_bus >= c.bec_min_v
        self.last = st
        return st

    def thrust_limit_n(self, st: BusState) -> float:
        """The thrust the ESC can hold on this bus: inf unless the load was limited (then the thrust at p_max)."""
        if not st.esc_ok:
            return 0.0
        return self.fan.thrust_at_power(st.p_max_w) if st.limited else math.inf

    # ---- measured signals (what the FCU may read) ----
    def adc(self, v: float, ch: int = 0) -> float:
        """A divider + 10-bit ADC reading in volts (quantised, 1 LSB RMS noise ASSUMED, clipped to full scale); channel
        ``ch`` (0 V_in, 1 V_tether, 2 V_buffer) carries its gain error ``cfg.adc_gain_err[ch]`` (fix round)."""
        c = self.cfg
        e = float(c.adc_gain_err[ch]) if c.adc_gain_err else 0.0
        if e != 0.0:
            v = v * (1.0 + e)
        lsb = c.adc_full_scale_v / (2 ** c.adc_bits)
        code = round(max(v, 0.0) / lsb + float(self.rng.normal(0.0, c.adc_noise_lsb)))
        return min(max(code, 0), 2 ** c.adc_bits - 1) * lsb

    def measure(self, st: BusState) -> dict[str, Any]:
        """V_in (bus), V_tether, V_buffer through the ADC; the buffer-current sign (hybrid)."""
        out: dict[str, Any] = {"v_in": self.adc(st.v_bus, 0)}
        if self.cfg.source in ("tether", "hybrid"):
            out["v_tether"] = self.adc(st.v_tether, 1)
        if self.cfg.source == "hybrid":
            out["v_buffer"] = self.adc(st.v_buffer, 2)
            out["buf_i_pos"] = bool(st.i_buffer > self.cfg.buf_i_sign_threshold_a)
        return out


def _single_source(voc: float, r: float, i_max: float, p: float) -> tuple[float, bool, float]:
    """Bus voltage of one source (V_oc, R, current limit) carrying P: the high root of V^2 - V_oc V + R P = 0 (current
    limit -> V = P / i_max on the CC branch); collapse -> the maximum-power point.  Returns (V, limited, P_max)."""
    if voc <= 0.0:
        return 0.0, p > 0.0, 0.0
    pmax_cv = voc * voc / (4.0 * r)
    v_knee = voc - i_max * r if math.isfinite(i_max) else -math.inf
    pmax = pmax_cv if v_knee <= 0.5 * voc else v_knee * i_max
    if p <= 0.0:
        return voc, False, pmax
    disc = voc * voc - 4.0 * r * p
    if disc >= 0.0:
        v = 0.5 * (voc + math.sqrt(disc))
        if v >= v_knee:
            return v, False, pmax
    if math.isfinite(i_max) and p <= pmax:
        return p / i_max, False, pmax
    v = 0.5 * voc if v_knee <= 0.5 * voc else v_knee
    return v, True, pmax


def _hybrid_bus(vs: float, rt: float, i_lim: float, vb: float, rb: float, vd: float, p: float, tether_up: bool,
                hybrid: bool) -> tuple[float, float, float, bool, float, bool]:
    """The diode-OR bus: tether (V_s - vd, R_loop, CC at i_lim) and buffer (V_b - vd, R_b, discharge only).  Returns
    (V, I_tether, I_buffer, limited, P_max, tether in CC).  Closed forms per conduction case."""
    vd = vd if hybrid else 0.0                     # the battery-less build has no D1 (the tether feeds the bus directly)
    vt = (vs - vd) if tether_up else 0.0
    vbb = (vb - vd) if hybrid else 0.0

    def currents(v: float) -> tuple[float, float, bool]:
        it = min(max((vt - v) / rt, 0.0), i_lim) if tether_up else 0.0
        ib = max((vbb - v) / rb, 0.0) if hybrid else 0.0
        cc = tether_up and (vt - v) / rt >= i_lim - 1e-9
        return it, ib, cc

    def pmax_scan() -> tuple[float, float]:
        hi = max(vt, vbb)
        best_v, best_p = hi, 0.0
        for k in range(1, 81):
            v = hi * k / 80.0
            it, ib, _ = currents(v)
            if v * (it + ib) > best_p:
                best_v, best_p = v, v * (it + ib)
        return best_v, best_p

    if not tether_up and not hybrid:
        return 0.0, 0.0, 0.0, p > 0.0, 0.0, False
    if p <= 0.0:
        return max(vt, vbb), 0.0, 0.0, False, math.inf, False
    cands: list[float] = []
    # case A: tether alone in CV
    if tether_up:
        d = vt * vt - 4.0 * rt * p
        if d >= 0.0:
            cands.append(0.5 * (vt + math.sqrt(d)))
        cands.append(p / i_lim)                                    # tether alone in CC
    # case B: buffer alone
    if hybrid:
        d = vbb * vbb - 4.0 * rb * p
        if d >= 0.0:
            cands.append(0.5 * (vbb + math.sqrt(d)))
    # case C: both, tether CV
    if tether_up and hybrid:
        a = 1.0 / rt + 1.0 / rb
        b = vt / rt + vbb / rb
        d = b * b - 4.0 * a * p
        if d >= 0.0:
            cands.append((b + math.sqrt(d)) / (2.0 * a))
        # case D: both, tether CC
        b2 = vbb / rb + i_lim
        d2 = b2 * b2 - 4.0 * p / rb
        if d2 >= 0.0:
            cands.append(0.5 * rb * (b2 + math.sqrt(d2)))
    # the highest candidate that is self-consistent (the sources deliver exactly P there)
    for v in sorted(cands, reverse=True):
        if v <= 0.0:
            continue
        it, ib, cc = currents(v)
        if abs(v * (it + ib) - p) <= 1e-6 * max(p, 1.0):
            return v, it, ib, False, math.inf, cc
    v, pm = pmax_scan()
    it, ib, cc = currents(v)
    return v, it, ib, True, pm, cc


# --------------------------------------------------------------------------- #
# tether mechanics
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class TetherConfig:
    """The overhead tether rig (tether proposal sim.mechanics, recommendation ii / iii; every number ASSUMED unless the
    proposal computed it).  World frame of ``sim.sim6dof``: z up, the pad top plate at z = 0, the ground at the pad's
    foot level (``FlightSim.ground_z``)."""

    anchor_xy_m: tuple = (0.0, 0.0)            # pulley position over the pad (set-up offset sampled by the campaign)
    anchor_height_m: float = 3.75              # pulley above the ground (proposal: 3.5-4.0 m)
    feed: str = "tension"                      # 'tension' (constant-tension feed) | 'slack' (paid-out slack, catenary)
    tension_n: float = 1.0                     # mean cable tension at the vehicle (proposal: 0.5-2.0 N)
    tension_sigma_n: float = 0.3               # Gauss-Markov wander of the handler / balancer (ASSUMED)
    tension_tau_s: float = 2.0                 # proposal: random walk tau 2 s
    cable_slack_m: float = 0.5                 # slack feed: cable length beyond the straight distance
    w_kg_per_m: float = 0.139                  # 2 x 10 AWG silicone + Dyneema (proposal lifted_tether 230 g / 1.5 m / 1.10)
    cable_d_m: float = 0.012                   # drag width of the bundle (2 x OD 5.5 side by side, ASSUMED)
    cable_cd: float = 1.2                      # cross-flow drag coefficient of the bundle (ASSUMED, circular cylinder)
    belay_slack_m: float = 0.2                 # catch line slack the handler keeps (proposal: 0-0.3 m)
    handler_tau_s: float = 0.4                 # handler follow time constant (ASSUMED)
    handler_rate_mps: float = 0.8              # max pay-out / take-in rate (ASSUMED; a fall outruns it)
    lock_tension_n: float = 10.0               # belay lock: no pay-out above this line tension (ASSUMED)
    bungee_k_n_m: float = 600.0                # bungee shock section (proposal: 600 N/m ASSUMED)
    bungee_zeta: float = 0.3                   # proposal: zeta 0.3 ASSUMED (on the vehicle mass)
    # fix round (physics review HIGH 2, control review HIGH 2): the catch-line device.  'lagged' = the v0.9 tether-round
    # law (first-order follow of D + belay_slack, no pay-out above lock_tension_n) - kept for comparison only, it holds a
    # vehicle up on slow descents; 'srl' = self-retracting lifeline: the line kept taut by srl_retract_n, free pay-out
    # below the lock speed, locks when the line pays out faster than lock_speed_mps (a fall), releases when unloaded;
    # 'autolock' = an auto-locking belay device with a handler: takes in with the handler lag to keep belay_slack_m,
    # pays out freely (no force) below lock_speed_mps, locks above it, releases when unloaded.  ASSUMED values.
    belay_device: str = "lagged"
    srl_retract_n: float = 0.6
    lock_speed_mps: float = 1.5
    release_hold_s: float = 0.3                # the line must stay unloaded this long before a locked device releases
    snag_t_s: float | None = None              # jerk (proposal W5): start ...
    snag_force_n: float = 20.0                 # ... peak force ...
    snag_duration_s: float = 0.2               # ... half-sine duration ...
    snag_angle_deg: float = 30.0               # ... from the vertical ...
    snag_azimuth_deg: float = 0.0              # ... toward this world azimuth

    def __post_init__(self) -> None:
        if self.feed not in ("tension", "slack"):
            raise ValueError(f"tether feed {self.feed!r} not in ('tension', 'slack')")
        if self.belay_device not in BELAY_DEVICES:
            raise ValueError(f"belay_device {self.belay_device!r} not in {BELAY_DEVICES}")
        if self.feed == "slack" and not self.cable_slack_m > 0.0:
            raise ValueError("slack feed needs cable_slack_m > 0 (a taut power cable is not modelled)")

    @classmethod
    def from_params(cls, p: dict, **kw: Any) -> "TetherConfig":
        tp = (p.get("power") or {}).get("tether_plant_ASSUMED") or {}
        d: dict[str, Any] = {}
        for k in ("anchor_height_m", "tension_n", "tension_sigma_n", "tension_tau_s", "cable_slack_m", "w_kg_per_m",
                  "cable_d_m", "cable_cd", "belay_slack_m", "handler_tau_s", "handler_rate_mps", "lock_tension_n",
                  "bungee_k_n_m", "bungee_zeta", "srl_retract_n", "lock_speed_mps", "release_hold_s"):
            if k in tp:
                d[k] = float(tp[k])
        if "belay_device" in tp:
            d["belay_device"] = str(tp["belay_device"])
        d.update(kw)
        return cls(**d)

    def anchor_world(self, ground_z_m: float) -> tuple[float, float, float]:
        return float(self.anchor_xy_m[0]), float(self.anchor_xy_m[1]), float(ground_z_m) + float(self.anchor_height_m)

    def snag_force(self, t: float) -> tuple[float, float, float]:
        """World force of the jerk at time t (half-sine), 0 outside it."""
        if self.snag_t_s is None or not (self.snag_t_s <= t < self.snag_t_s + self.snag_duration_s):
            return 0.0, 0.0, 0.0
        f = self.snag_force_n * math.sin(math.pi * (t - self.snag_t_s) / self.snag_duration_s)
        a, b = math.radians(self.snag_angle_deg), math.radians(self.snag_azimuth_deg)
        return f * math.sin(a) * math.cos(b), f * math.sin(a) * math.sin(b), f * math.cos(a)


def catch_line_force(d: tuple[float, float, float], v_att: tuple[float, float, float], l0: float, k: float,
                     c: float, l0_dot: float = 0.0) -> tuple[float, float, float, float]:
    """Spring-damper of the catch line: ``d`` = anchor - attachment (world), ``v_att`` = the attachment's velocity.  Taut
    (|d| > L0): F = k (|d| - L0) + c (d|d|/dt - dL0/dt) >= 0 along d - the damper acts on the bungee's stretch rate
    (fix round, physics review HIGH 2: the v0.9 law damped d|d|/dt with L0 held and ignored the handler's pay-out, up to
    14 N of spurious force).  Returns (Fx, Fy, Fz, tension)."""
    dx, dy, dz = d
    D = math.sqrt(dx * dx + dy * dy + dz * dz)
    if D <= l0 or D < 1e-9:
        return 0.0, 0.0, 0.0, 0.0
    ux, uy, uz = dx / D, dy / D, dz / D
    dd = -(ux * v_att[0] + uy * v_att[1] + uz * v_att[2])       # d|d|/dt (the anchor is fixed)
    f = k * (D - l0) + c * (dd - l0_dot)
    if f <= 0.0:
        return 0.0, 0.0, 0.0, 0.0
    return f * ux, f * uy, f * uz, f


def catenary_end_force(d: tuple[float, float, float], length_m: float, w_n_per_m: float,
                       h_max_n: float = 20.0) -> tuple[float, float, float]:
    """World force a hanging cable of ``length_m`` (weight w per metre) exerts on its lower end P when its upper end is
    the anchor A (``d`` = A - P): the catenary through both points (horizontal span b, rise h); the horizontal tension
    H = w a and the end slope sinh(-x0 / a).  A cable shorter than the straight distance is treated as taut with
    H = h_max_n (the catch line takes over); b below 1 cm is taken as 1 cm."""
    dx, dy, dz = d
    b = math.hypot(dx, dy)
    ex, ey = (dx / b, dy / b) if b > 1e-9 else (1.0, 0.0)
    b = max(b, 0.01)
    h = dz
    s2 = length_m * length_m - h * h
    if s2 <= b * b * (1.0 + 1e-9):
        # taut (fix round, physics review low: the v0.9 branch returned ~4500 N for a vertical span): the tension
        # h_max_n along the chord
        Dc = math.sqrt(dx * dx + dy * dy + dz * dz)
        if Dc < 1e-9:
            return 0.0, 0.0, 0.0
        return h_max_n * dx / Dc, h_max_n * dy / Dc, h_max_n * dz / Dc
    r = math.sqrt(s2) / b                               # sinh(z) / z = r, z = b / (2 a)
    z = math.sqrt(6.0 * (r - 1.0)) if r < 1.5 else math.log(2.0 * r * max(math.log(2.0 * r), 1.0))
    for _ in range(40):
        sz, cz = math.sinh(z), math.cosh(z)
        f = sz - r * z
        fp = cz - r
        if abs(fp) < 1e-14:
            break
        dz_ = f / fp
        z = max(z - dz_, 1e-6)
        if abs(dz_) < 1e-12 * max(z, 1.0):
            break
    a = b / (2.0 * z)
    H = min(w_n_per_m * a, h_max_n)
    x0 = 0.5 * b - a * math.atanh(max(min(h / length_m, 0.999999999), -0.999999999))
    slope = math.sinh(-x0 / a)                           # dy/dx at P (x = 0)
    return H * ex, H * ey, H * slope


def catenary_loop(d: tuple[float, float, float], length_m: float) -> tuple[float, float]:
    """Where the slack-fed power cable hangs (fix round, physics review medium 4): (depth of the lowest point below the
    lower end P [m] (0 when the cable rises from P), horizontal distance of that point from P [m]).  ``d`` = anchor - P
    (world).  A span narrower than 1 cm hangs as a vertical loop of depth (L - h) / 2 at P."""
    dx, dy, h = d
    b = math.hypot(dx, dy)
    s2 = length_m * length_m - h * h
    if s2 <= b * b * (1.0 + 1e-9):
        return 0.0, 0.0
    if b < 0.01:
        return max(0.5 * (length_m - h), 0.0), 0.0
    r = math.sqrt(s2) / b
    z = math.sqrt(6.0 * (r - 1.0)) if r < 1.5 else math.log(2.0 * r * max(math.log(2.0 * r), 1.0))
    for _ in range(40):
        sz, cz = math.sinh(z), math.cosh(z)
        fp = cz - r
        if abs(fp) < 1e-14:
            break
        dz_ = (sz - r * z) / fp
        z = max(z - dz_, 1e-6)
        if abs(dz_) < 1e-12 * max(z, 1.0):
            break
    a = b / (2.0 * z)
    x0 = 0.5 * b - a * math.atanh(max(min(h / length_m, 0.999999999), -0.999999999))
    if x0 <= 0.0:                                        # the vertex lies beyond P: the cable rises from P
        return 0.0, 0.0
    return a * (math.cosh(x0 / a) - 1.0), x0


@dataclass
class TetherState:
    """The rig's state of one run: the handler's paid-out catch-line length, the feed tension wander, the catch-line
    device state (fix round: locked, the free retract force, the line speed) and the cable loop."""

    l0: float = 0.0
    tc: float = 0.0
    tension_catch: float = 0.0
    tension_cable: float = 0.0
    cable_force_w: tuple = (0.0, 0.0, 0.0)
    l0_dot: float = 0.0
    d_prev: float | None = None
    t_prev: float | None = None
    ddot: float = 0.0
    locked: bool = False
    unloaded_s: float = 0.0
    retract_n: float = 0.0                 # SRL retract force while unlocked (held force along the line)
    locks: int = 0
    loop_depth_m: float = 0.0
    loop_x_m: float = 0.0


class TetherRig:
    """The mechanics of one run: ``begin`` at the start pose, ``update`` once per physics step (handler, feed wander,
    catenary / feed force, drag share) - the catch-line spring-damper is evaluated inside the RK4 derivative from
    ``deriv_args``."""

    def __init__(self, cfg: TetherConfig, ground_z_m: float, mass_kg: float, seed: int = 0, cable_present: bool = True):
        self.cfg = cfg
        self.A = cfg.anchor_world(ground_z_m)
        self.k = float(cfg.bungee_k_n_m)
        self.c = 2.0 * float(cfg.bungee_zeta) * math.sqrt(self.k * float(mass_kg))
        self.rng = np.random.default_rng([int(seed), 0x7E8])
        self.st = TetherState()
        self.cable_present = cable_present
        self.w = float(cfg.w_kg_per_m) * 9.80665

    def begin(self, p_att: np.ndarray) -> None:
        d = np.asarray(self.A) - np.asarray(p_att)
        D = float(np.linalg.norm(d))
        self.st.l0 = D + (SRL_EPS_M if self.cfg.belay_device == "srl" else float(self.cfg.belay_slack_m))
        self.st.tc = float(self.cfg.tension_n)
        self.st.d_prev, self.st.t_prev = D, None

    def distance(self, p_att: np.ndarray) -> float:
        return float(np.linalg.norm(np.asarray(self.A) - np.asarray(p_att)))

    def catch_free_w(self, p_att: np.ndarray) -> tuple[float, float, float]:
        """The SRL's retract force on the vehicle (world, toward the anchor) while unlocked; 0 for the other devices."""
        if self.st.retract_n <= 0.0:
            return 0.0, 0.0, 0.0
        d = np.asarray(self.A) - np.asarray(p_att)
        D = float(np.linalg.norm(d))
        if D < 1e-9:
            return 0.0, 0.0, 0.0
        f = self.st.retract_n / D
        return float(d[0]) * f, float(d[1]) * f, float(d[2]) * f

    def _device(self, h: float, D: float, ddot: float) -> None:
        """The catch-line device (fix round): 'srl' / 'autolock' - free pay-out below lock_speed_mps, lock above it,
        release after release_hold_s unloaded; 'srl' keeps the line taut (retract force), 'autolock' takes in with the
        handler lag to belay_slack_m."""
        cfg, st = self.cfg, self.st
        if st.locked:
            st.unloaded_s = st.unloaded_s + h if st.tension_catch <= RELEASE_LOAD_N else 0.0
            if st.unloaded_s >= cfg.release_hold_s:
                st.locked, st.unloaded_s = False, 0.0
        if not st.locked:
            if D + SRL_EPS_M > st.l0:                      # the line has to pay out
                if ddot > cfg.lock_speed_mps:
                    st.locked = True                        # a fall: the drum / cam locks, the bungee arrests
                    st.locks += 1
                else:
                    st.l0 = D + SRL_EPS_M
            if not st.locked:
                if cfg.belay_device == "srl":
                    st.l0 = D + SRL_EPS_M                   # the retract spring takes in at once
                else:
                    tgt = D + cfg.belay_slack_m
                    if st.l0 > tgt:
                        st.l0 += max((tgt - st.l0) / max(cfg.handler_tau_s, 1e-6), -cfg.handler_rate_mps) * h
        st.retract_n = cfg.srl_retract_n if (cfg.belay_device == "srl" and not st.locked) else 0.0

    def update(self, t: float, h: float, p_att: np.ndarray, v_att: np.ndarray, wind: tuple, rho: float) -> None:
        """Handler (catch-line length), feed wander and the cable's force (held over the next step)."""
        cfg, st = self.cfg, self.st
        d = np.asarray(self.A) - np.asarray(p_att)
        D = float(np.linalg.norm(d))
        # line speed from the previous update (the steps are uneven: command edges split them)
        dtp = t - st.t_prev if st.t_prev is not None else 0.0
        if st.d_prev is not None and dtp > 1e-6:
            st.ddot = (D - st.d_prev) / dtp
            st.d_prev, st.t_prev = D, t
        elif st.d_prev is None or st.t_prev is None:
            st.d_prev, st.t_prev = D, t
        ddot = st.ddot
        l0_old = st.l0
        if cfg.belay_device == "lagged":
            # v0.9 handler: follow D + slack with a first-order lag and a rate limit; the belay locks under load
            rate = (D + cfg.belay_slack_m - st.l0) / max(cfg.handler_tau_s, 1e-6)
            rate = min(max(rate, -cfg.handler_rate_mps), cfg.handler_rate_mps)
            if st.tension_catch > cfg.lock_tension_n and rate > 0.0:
                rate = 0.0
            st.l0 += rate * h
        else:
            self._device(h, D, ddot)
        st.l0_dot = (st.l0 - l0_old) / h if h > 0.0 else 0.0
        # constant-tension feed: Gauss-Markov wander around the mean (never below 0)
        a = math.exp(-h / max(cfg.tension_tau_s, 1e-6))
        st.tc = cfg.tension_n + a * (st.tc - cfg.tension_n) + math.sqrt(1.0 - a * a) * cfg.tension_sigma_n * float(
            self.rng.normal())
        if not self.cable_present or D < 1e-6:
            st.cable_force_w = (0.0, 0.0, 0.0)
            st.tension_cable = 0.0
            return
        u = d / D
        if cfg.feed == "tension":
            tc = max(st.tc, 0.0)
            f = (tc * u[0], tc * u[1], tc * u[2])
        else:
            f = catenary_end_force((float(d[0]), float(d[1]), float(d[2])), D + cfg.cable_slack_m, self.w)
            st.loop_depth_m, st.loop_x_m = catenary_loop((float(d[0]), float(d[1]), float(d[2])), D + cfg.cable_slack_m)
        # half the cross-flow drag of the span (anchor fixed: the mid-span moves at half the vehicle's speed)
        vm = 0.5 * np.asarray(v_att) - np.asarray(wind)
        vperp = vm - float(vm @ u) * u
        sp = float(np.linalg.norm(vperp))
        fd = -0.5 * (0.5 * rho * cfg.cable_cd * cfg.cable_d_m * D * sp) * vperp
        st.cable_force_w = (f[0] + float(fd[0]), f[1] + float(fd[1]), f[2] + float(fd[2]))
        st.tension_cable = math.sqrt(sum(x * x for x in f))

    def cut_cable(self) -> None:
        self.cable_present = False
        self.st.cable_force_w = (0.0, 0.0, 0.0)
        self.st.tension_cable = 0.0
