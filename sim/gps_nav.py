"""v0.8 GPS on the NAV: the u-blox UBX protocol reference and the GPS-aided horizontal estimator - the reference.

Executable specification for the NAV firmware (``firmware/atmega128_fc/src/core/gps_ubx.c`` / ``gps_nav.c``, to be
ported; float32 on the AVR, float64 here except where a step is integer and therefore exact in both).  Pure Python, no
numpy, no RNG, no clock: every state explicit, every number a module constant with its source or a params.json value.

Hardware (params.json ``electronics.gps``, ``flight_computer.nav.gps``; architect v0.8): u-blox NEO-6M on a GY-NEO6MV2
board (NEO-M8N = upgrade path) on the NAV's USART1 (PD2 RXD1 / PD3 TXD1), 38400 8N1 through a level shifter; UBX
NAV-POSLLH + NAV-VELNED + NAV-SOL at 5 Hz; antenna in the option-B pod under the nose skirt, ~321 mm from the CG.

Part 1 - UBX protocol (byte-exact; ``sim/out/gps_vectors.json`` holds the C vectors)
================================================================================
Frame: 0xB5 0x62 | class | id | length u16 LE | payload | CK_A CK_B, the 8-bit Fletcher checksum over class, id, the
two length bytes and the payload (CK_A += b; CK_B += CK_A, both mod 256).  Little-endian fields.  Messages used:

    NAV-POSLLH 01 02  28 B  iTOW U4 ms, lon I4 1e-7 deg, lat I4 1e-7 deg, height I4 mm, hMSL I4 mm, hAcc U4 mm, vAcc U4 mm
    NAV-SOL    01 06  52 B  iTOW U4, fTOW I4 ns, week I2, gpsFix U1 (3 = 3D), flags X1 (bit0 gpsFixOK), ecefX/Y/Z I4 cm,
                            pAcc U4 cm, ecefVX/VY/VZ I4 cm/s, sAcc U4 cm/s, pDOP U2 0.01, res U1, numSV U1, res U4
    NAV-VELNED 01 12  36 B  iTOW U4, velN/velE/velD I4 cm/s, speed U4, gSpeed U4 cm/s, heading I4 1e-5 deg, sAcc U4 cm/s,
                            cAcc U4 1e-5 deg
    ACK-ACK 05 01 / ACK-NAK 05 00  2 B  clsID, msgID of the acknowledged message
    CFG-PRT 06 00 20 B (UART1, 38400, 8N1, in UBX+NMEA, out UBX only), CFG-RATE 06 08 6 B (200 ms, 1 cycle, GPS time),
    CFG-MSG 06 01 3 B (class, id, rate 1 on the current port), CFG-NAV5 06 24 36 B (mask 0x0001: dynModel 6 = airborne
    < 1 g only), MON-VER 0A 04 (poll: length 0; answer: swVersion char[30], hwVersion char[10], romVersion char[30], ...).

Streaming parser (``UbxParser``, one byte at a time, bounded work per byte like ``sim.ilink_proto``): SYNC1 (0xB5,
else skipped) -> SYNC2 (0x62; another 0xB5 keeps SYNC2; else back to SYNC1) -> class -> id -> length (2 bytes; >
``UBX_LEN_LIMIT`` = 256: ``bad_len``, back to SYNC1) -> payload (stored up to ``UBX_MAX_PAYLOAD`` = 64 bytes, the
rest counted but not stored: ``truncated``) -> CK_A -> CK_B (either wrong: ``bad_ck``, back to SYNC1).  No replay: a
frame that starts inside a rejected candidate is lost (the next 5 Hz epoch replaces it).  64 B holds NAV-SOL (52) and
MON-VER's swVersion + hwVersion (40); every other message the NAV needs is shorter.

Epoch assembly (``EpochAssembler``): NAV-POSLLH, NAV-VELNED and NAV-SOL with the SAME iTOW form one epoch, in any
order; the epoch is complete (stamped with the NAV tick of its last message) when all three arrived.  A message with a
different iTOW while an epoch is open drops the open one (``incomplete``) and opens a new one.  Why by iTOW: the three
messages leave the receiver 10-15 ms apart (140 B at 38400 baud = 36 ms); pairing them by arrival would mix epochs.

Epoch time (``ItowClock``, review v0.8 (8)): D = (NAV_ms - iTOW) mod 2^32 as int32 at the completion tick
(NAV_ms = tick x 1000 / loop_hz, an integer: 4 ms per tick); offset O = D on the first epoch or when |D - O| > 1000 ms
(boot, week roll-over), else O = min(D, O + ``ITOW_LEAK_MS`` = 1): the earliest-arriving epoch defines the offset, the
1 ms per epoch leak (5 ms/s) follows the drift of the NAV clock (crystal; a ceramic resonator stays within 0.5 %).
Measurement time of the epoch on the NAV clock: t_e = iTOW + O - L, L = round(latency_s x 1000) (params
flight_computer.nav.gps.latency_s_ASSUMED 0.22 s, ArduPilot's u-blox 6 lag: from the measurement to the complete
arrival).  The arrival jitter (UART, tick quantisation, the receiver's output load) is removed by the minimum; the
latency itself is not observable and stays ASSUMED.

Boot configuration (``GpsConfigurator``, review v0.8 (8); the NEO-6M has no flash: its configuration lives in the
battery-backed RAM, so the NAV re-sends it at every boot): wait ``CFG_BOOT_WAIT_S``; probe at 38400 (CFG-PRT, ACK
checked - a receiver kept configured by its backup cell answers here); else switch the NAV UART to 9600 (the factory
default), send CFG-PRT (the receiver changes its baud rate: that ACK is not checked), settle, back to 38400, CFG-PRT
again with its ACK checked; then CFG-RATE, CFG-MSG x 3, CFG-NAV5, each ACK-ACK checked (NAK or ``CFG_ACK_TIMEOUT_S``
without an answer: retry, ``CFG_TRIES`` in all); then poll MON-VER (``CFG_VER_TIMEOUT_S``, ``CFG_TRIES``).  Result:
CFG_OK (every frame acknowledged), VER_OK (MON-VER answered) and the module class from hwVersion ("00040007" u-blox 6,
"00070000" u-blox 7 / NEO-7M (2026-11-06), "00080000" u-blox 8 / M8; anything else 'unknown' - clones often answer
"00040007" as well, so a fake is NOT reliably detected).  The module class is reported only: nothing depends on it.  The configurator never blocks: one call per NAV tick, its actions (UART baud, bytes to send) are returned.

Part 2 - NAV GPS-aided horizontal estimator (``NavGpsFilter``)
=============================================================
Frames (z up everywhere): body b (x to vane 1, z the thrust axis); the NAV estimator's level frame l (the attitude
quaternion's frame: yaw 0 = the pad heading); world W = x NORTH, y WEST, z up (right-handed; the sim's world frame).
GPS -> W: x = (lat - lat0) k_N, y = -(lon - lon0) k_E with the WGS-84 meridian / prime-vertical radii at the origin
(``LocalFrame``; lat0, lon0 = the first usable fix after boot); v_x = velN, v_y = -velE.  Heading psi = the yaw of l in
W = -pad_azimuth (params mission.vtol.position_hold.heading.pad_azimuth_deg, compass bearing of the vehicle's +X, 0 =
north) + the yaw correction (below); c_psi, s_psi cached, recomputed only when the correction changes.

Per axis i in {x, y} of W, state p_i, v_i (CG position / velocity) and b_i (the error of the measured horizontal
acceleration: tilt-estimate error x g + accelerometer bias); the measured acceleration a_m = a + b.  Every NAV tick (in
the correction part, AFTER the frame push - the v0.8 NAV order) the body specific force is summed (3 additions).
The tick's cycle position j = k mod 5 (k = NAV ticks since boot, uint32; 5 = loop_hz / fusion_hz):

  j = 4  STEP (50 Hz): f = mean body specific force of the cycle's ticks; R = R(q) of the CORRECTED attitude of this
         tick; a_l = (R f)_xy; a_W = Rz(psi) a_l; for each axis: a = a_W,i - b_i, p_i += v_i T + a T^2 / 2, v_i += a T
         (T = n_ticks / loop_hz, 0.02 s).  Lever arm (antenna from the CG, body l_b from params): r = R l_b,
         v_lever = (R w) x (R l_b) = R (w x l_b) (w = the tick's bias-corrected body rate; the identity needs only rows 0
         and 1 of R - about 50 multiplications per step in all), both rotated by psi; the delayed-state buffer
         entry m = k // 5 (ring of ``BUF_N`` = 16, indexed by m mod 16, no search) stores the ANTENNA position
         p + r_W,xy and velocity v + v_lever,W,xy, the IMU-only velocity u (u += (a_W - b) T, no GPS correction; the
         yaw check) and v_lever,W.  Then the snapshot for the nlink slots is latched (sim.nlink_proto: the frames of the
         next five ticks carry it).
  j = 0  EPOCH FUSION of the newest epoch completed since the previous fusion (older ones of the same cycle are
         dropped: ``n_epoch_dropped``; at 5 Hz never two).  The C port splits it over j = 0..3 (E0 innovations, E1 gates
         and resets, E2 corrections and buffer, E3 yaw and the gate timers: params flight_computer.nav.gps.
         epoch_split_ticks 4); nothing else changes the state between j = 0 and j = 3, so the split gives the same
         numbers as this reference doing it at once.
         State time = the last step m_now = k // 5 - 1; age_ms = (20 m_now + 16) - t_e (int32), a = floor((age_ms +
         10) / 20) steps (>= 0; a >= BUF_N or a stale entry: dropped), s_e = m_now - a, age = a T.
         Quality: usable = gpsFixOK and gpsFix >= 3 and numSV >= ``OK_MIN_SV`` and hAcc <= ``OK_MAX_HACC_M``; good (the ON
         gate) = gpsFixOK and gpsFix == 3 and numSV >= min_sv and hAcc <= max_hacc and sAcc <= max_sacc (params
         gps_gate_on_ASSUMED).  Not usable: no fusion (counts as a missing epoch).
         First usable epoch: origin = its lat / lon, p = -r_W (antenna -> CG), v = 0 on the pad else the GPS velocity,
         b = 0, buffer cleared, INIT.
         Innovations: y_p = z_p - P_ant[s_e], y_v = z_v - V_ant[s_e] (2-D).  Gates (precomputed): position |y_p|^2 >
         ``GATE_P_M``^2 -> rejected (both axes); velocity per axis y_v,i^2 > (``GATE_SIGMA``)^2 S_vv -> rejected (S_vv =
         the steady-state innovation variance).  Fixed gains (review v0.8 (1): no covariance propagation): the
         steady-state Kalman gains of the model below at the epoch rate, K_pv (3 x 2) when both are accepted, K_p / K_v
         (3 x 1) for one of them (from the same steady prior); the ACQUISITION set (the take-off Riccati gain at epoch
         ``ACQ_GAIN_EPOCH``) for ``ACQ_EPOCHS`` fused epochs after the take-off, the first fix and every reset.
         dx = K y (per axis) at the epoch, propagated to the state time: p += dx_p + age dx_v - age^2/2 dx_b, v += dx_v -
         age dx_b, b += dx_b; the newest ``N_CORR`` buffer entries s >= s_e (the ones the next epoch can read: the
         latency is 11 steps, the epoch period 10) get the same with their own age (s - s_e) T.
         Position reset (PX4's xy reset idea, the prototype): the position rejected continuously for ``RESET_AFTER_S``
         while the velocity is accepted -> p += y_p (state and the corrected entries), reset_count += 1, the cumulative
         reset offset += y_p; the FCU holds its point in the reset-free frame p - offset, so a GPS jump never moves the
         vehicle.  Velocity reset (prototype, against the gating death spiral of a heading error): the velocity rejected
         in ``VEL_RESET_EPOCHS`` consecutive epochs -> v += y_v, p += y_v age, b = 0, reset_count += 1 (the offset is
         NOT changed: that position change corrects a motion the IMU path missed, the vehicle must not follow it back).
         FIX_OK = INIT and an epoch used (a measurement accepted or a reset) within ``LOST_AFTER_EPOCHS`` / rate_hz
         (``fresh_ticks``, evaluated at the latch).
         ON gate: GATE_OK = FIX_OK and every epoch good since a tick >= stable_s (params) ago; a non-good epoch or a gap
         of more than fresh_ticks between epochs restarts the stability time.
         Yaw (no magnetometer, review v0.8 (6)): a window opens at a fused epoch whose velocity is accepted on both
         axes, with the GPS CG velocity g0 = z_v - v_lever[s_e] and the IMU-only velocity u0 = u[s_e]; at the first such
         epoch >= ``YAW_WINDOW_S`` later: du = u - u0, dg = g - g0; if |du| >= ``YAW_MIN_DV_MPS`` and |dg| > |du| / 2:
         angle d = atan2(du x dg, du . dg) (the level frame must turn by d), a 1-state Kalman update weighted by the
         information of the window: R = (SIGMA_DV / |du|)^2 + SIGMA_REL^2 (the angle-noise floor, physics-review
         fix), K = P / (P + R), correction += K d, P = (1 - K) P; at
         EVERY window close, used or not, first P_psi += Q_PSI (window time) (physics-review fix: the variance grew only
         on used windows); the window reopens there.  HEAD_OK = P_psi <= HEAD_OK_DEG^2.  Observable only while the
         vehicle accelerates horizontally (pushes, gusts, its own corrections); in a steady hover the heading error stays
         what the pad set-up left (params heading.tolerance_deg 15 since the physics review) plus the z-gyro drift - see
         "heading error".
         GPS-IMU inconsistency (physics-review fix, HIGH 2): each fused flight epoch shifts "velocity rejected on either
         axis" into a 5-epoch history; GF_INCONS = at least ``INCONS_MIN_REJ`` (3) of the last ``INCONS_EPOCHS`` (5)
         rejected (cleared in pad mode).  The FCU's plausibility rule (a) counts a limited step only while it is set.
  pad    (``on_pad`` = sim.nlink_proto.nav_pad_mode: FCU SAFE / ARMED): v = 0, b = 0, u = 0 at every step; fusion
         uses the position only (K_p) and no resets of the velocity; the first flight step re-arms the acquisition gains.

Model behind the gains (per axis, white acceleration noise q_acc [m^2/s^3] on v, random-walk q_b [m^2/s^5] on b, epoch
T_e = 1 / rate_hz; R = diag(sigma_p^2, sigma_v^2)): the prototype's TUNED values (report_gps-hold-prototype.json).
With them: steady position gain 0.010 per epoch (the GPS position is followed with ~20 s: its slow wander is not
chased), velocity gain 0.36, b time constant 7 s; the acquisition gains 2.2 s.

Heading error (how the loop tolerates it): the FCU closes the loop on the NAV's W-frame p, v.  A heading error e makes
the NAV rotate its IMU acceleration by e: the velocity filter then sees a GPS velocity that disagrees with the
predicted one by ~2 sin(e/2) |a| T per step - the 0.36 velocity gain absorbs that as long as the gates pass (5 sigma
of S_vv), and the residual is an acceleration error that b learns (an apparent tilt bias ~ a sin e).  The FCU's tilt is
computed in W and rotated into l by -psi: its direction error is e, so the position loop's restoring force has a
component cos(e) along the error and sin(e) across it.  What limits e is NOT the loop's linear phase margin (the
earlier text said "stable while cos(e) > 0 with the loop's phase margin"; the v0.8 physics review measured it in the
6-DOF): WITHOUT the yaw correction 20 deg held, 30 deg was marginal (tilt at the cap, touchdown 0.63 m/s) and 45 deg
ended in V13 (field 57 m, demo 15 m away) - and a gain set with a 49.5 deg linear phase margin behaved the same; the
limit is the NAV velocity gate: the gated innovations trigger velocity resets (4-6 per flight) and plausibility rule
(c).  WITH the yaw correction 45 deg held (field calm 0.89 m RMS).  Hence params heading.tolerance_deg 15 (architect
18:10: phone compass +-5 deg and the vane-1 mark), so that the correction is margin, not the primary means.  The yaw
correction shrinks e whenever the vehicle accelerates; the velocity reset stops the gating spiral at large e.

v0.8.1 (architect 2026-10-21 20:30, params flight_computer.architecture_v081 / gns / glink): the receiver, Part 1 and
this filter move to the FOURTH ATmega128A, the GNS board (the three-chip build measured NAV mandatory work 98.7 % mean /
119.5 % max of the tick).  ``sim.glink_proto.GnsNode`` runs them: the UBX bytes on the GNS's USART1 are polled every
4 ms (``NavGps.tick``'s parser / configurator / assembler / iTOW clock part; the epoch stamps are NAV ticks - the GNS
sets its tick clock from each STATE frame's tick); the NAV keeps summing the body specific force per tick (3
additions) and sends, at k mod 5 == 4 after its correction part, the glink STATE frame (the sum, the corrected q, the
bias-corrected rate, h, its pad flag, the FCU's flags); on that frame the GNS does ``gns_step`` (the j = 4 STEP), the
position controller (``sim.position_ctrl``) on the latched state, the RESULT frame, then ``gns_fuse`` (the j = 0
fusion, k = the frame's tick + 1).  The order STEP m -> fusion -> STEP m + 1 is the v0.8 NAV's, so the filter's numbers
are the v0.8 ones up to the float32 transport of its inputs.  The ``NavGpsParams.loop_hz`` (250) is the GNS's poll / tick
clock = the NAV tick.  A STATE frame lost on the wire: the next one propagates over the gap with its mean force (the buffer
entry of the lost step stays stale: an epoch that needs it is dropped); a NAV reset or a gap longer than the buffer:
``restart_timebase``.

Invariant: nothing here sees the truth; inputs are UBX bytes / decoded epochs, the IMU samples and the NAV's corrected
attitude; outputs are the snapshot (sim.nlink_proto quantises it into the rotating slot) and diagnostics.
"""
from __future__ import annotations

import json
import math
import os
import struct
import sys
from dataclasses import dataclass, field, replace
from typing import Iterable, Sequence

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import link_proto as lp
except ImportError:  # pragma: no cover
    import link_proto as lp  # type: ignore

_HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.normpath(os.path.join(_HERE, "..", "params.json"))
VECTORS_PATH = os.path.join(_HERE, "out", "gps_vectors.json")

# --------------------------------------------------------------------------- #
# UBX constants (u-blox 6 Receiver Description incl. Protocol Specification, GPS.G6-SW-10018; == gps_ubx.h)
# --------------------------------------------------------------------------- #
UBX_SYNC1 = 0xB5
UBX_SYNC2 = 0x62
CLS_NAV, CLS_ACK, CLS_CFG, CLS_MON = 0x01, 0x05, 0x06, 0x0A
ID_NAV_POSLLH, ID_NAV_SOL, ID_NAV_VELNED = 0x02, 0x06, 0x12
ID_ACK_NAK, ID_ACK_ACK = 0x00, 0x01
ID_CFG_PRT, ID_CFG_MSG, ID_CFG_RATE, ID_CFG_NAV5 = 0x00, 0x01, 0x08, 0x24
ID_MON_VER = 0x04
POSLLH_FMT = "<IiiiiII"
VELNED_FMT = "<IiiiIIiII"
SOL_FMT = "<IihBBiiiIiiiIHBBI"
ACK_FMT = "<BB"
CFG_PRT_FMT = "<BBHIIHHHH"
CFG_RATE_FMT = "<HHH"
CFG_MSG_FMT = "<BBB"
CFG_NAV5_FMT = "<HBBiIbBHHHHBBIII"
LEN_POSLLH = struct.calcsize(POSLLH_FMT)       # 28
LEN_VELNED = struct.calcsize(VELNED_FMT)       # 36
LEN_SOL = struct.calcsize(SOL_FMT)             # 52
LEN_CFG_PRT = struct.calcsize(CFG_PRT_FMT)     # 20
LEN_CFG_NAV5 = struct.calcsize(CFG_NAV5_FMT)   # 36
UBX_OVERHEAD = 8                               # sync x2, class, id, length x2, CK_A, CK_B
UBX_MAX_PAYLOAD = 64                           # bytes stored (NAV-SOL 52, MON-VER sw + hw 40)
UBX_LEN_LIMIT = 256                            # a longer length field is corruption (resync)
SOL_FLAG_FIX_OK = 0x01                         # NAV-SOL flags bit 0 gpsFixOK
FIX_3D = 3
CFG_PRT_MODE_8N1 = 0x000008D0                  # charLen 8, no parity, 1 stop bit
PROTO_UBX, PROTO_NMEA = 0x0001, 0x0002
DYN_AIRBORNE_1G = 6                            # CFG-NAV5 dynModel (params flight_computer.nav.gps.dyn_model)
NAV5_MASK_DYN = 0x0001
GPS_BAUD = 38400                               # params flight_computer.nav.gps.baud
GPS_BAUD_DEFAULT = 9600                        # NEO-6 factory default (data sheet)
HW_UBLOX6 = "00040007"                          # MON-VER hwVersion: u-blox 6 (NEO-6M)
HW_UBLOX8 = "00080000"                          # u-blox 8 / M8 (NEO-M8N)
HW_UBLOX7 = "00070000"                          # u-blox 7 (NEO-7M, the user's part 2026-11-05; report only - A: u-blox 7
                                                # protocol spec not on disk, the string from the u-blox convention)
MODULE_UNKNOWN, MODULE_UBLOX6, MODULE_UBLOX7, MODULE_UBLOX8 = 0, 6, 7, 8

# --------------------------------------------------------------------------- #
# configurator timing (ASSUMED: the data sheet gives no command latency; u-blox answers a CFG within ~10-100 ms)
# --------------------------------------------------------------------------- #
CFG_BOOT_WAIT_S = 0.5          # after the NAV boot (the GPS is powered with it) before the first command
CFG_ACK_TIMEOUT_S = 0.3        # wait for ACK-ACK / ACK-NAK of one CFG frame
CFG_SETTLE_S = 0.1             # after the 9600-baud CFG-PRT: its 28 bytes take 29 ms at 9600, then the receiver switches
CFG_VER_TIMEOUT_S = 0.5        # MON-VER answer (~100 B)
CFG_TRIES = 3                  # attempts per frame

# --------------------------------------------------------------------------- #
# epoch time
# --------------------------------------------------------------------------- #
ITOW_LEAK_MS = 1               # offset leak per epoch (5 ms/s at 5 Hz; module docstring)
ITOW_JUMP_MS = 1000            # |D - O| above this re-learns the offset (boot, week roll-over, a receiver reset)

# --------------------------------------------------------------------------- #
# NAV filter constants (TUNED = the v0.8 prototype's scratch study; ASSUMED = no source)
# --------------------------------------------------------------------------- #
KF_Q_ACC = 0.02                # [m^2/s^3] acceleration white noise PSD (vibration 0.1 g at 250 Hz ~ 0.004) TUNED
KF_Q_B = 4e-4                  # [m^2/s^5] accel-error random walk PSD TUNED
KF_SIGMA_P_M = 3.0             # GPS position measurement sigma for the gains TUNED (> the white error: the Gauss-Markov
                               # wander is slow - the Doppler velocity carries the short term)
KF_SIGMA_V_MPS = 0.15          # GPS velocity measurement sigma TUNED
KF_SIGMA_V0_MPS = 0.1          # v at take-off (known 0 on the pad) ASSUMED
KF_SIGMA_B0_MPS2 = 0.15        # b at take-off (accel bias 10 mg + alignment) ASSUMED
ACQ_GAIN_EPOCH = 10            # acquisition gains = the take-off Riccati gain at this epoch (b tau 2.2 s) ASSUMED
ACQ_EPOCHS = 50                # ... used for this many fused epochs (10 s at 5 Hz; review v0.8 (1): "first ~10 s")
GATE_SIGMA = 5.0               # velocity innovation gate [sigma of S_vv] ASSUMED (prototype)
GATE_P_M = 3.0                 # position innovation gate |y_p| (2-D) TUNED (prototype: normal flights 0.7-1.3 m RMS,
                               # max 2.2 m; the gain's inflated R would let a 5 m multipath jump in)
RESET_AFTER_S = 1.5            # position rejected this long (velocity accepted) -> position reset ASSUMED (prototype)
VEL_RESET_EPOCHS = 5           # velocity rejected in this many consecutive epochs -> velocity reset (prototype)
LOST_AFTER_EPOCHS = 3          # no used epoch for this many periods -> FIX_OK clear ASSUMED (prototype)
OK_MIN_SV = 5                  # in flight: fewer satellites = not usable ASSUMED (prototype)
OK_MAX_HACC_M = 10.0           # in flight: worse hAcc = not usable ASSUMED (prototype)
BUF_N = 16                     # delayed-state ring (steps): 0.32 s > the 0.22 s latency + jitter; power of 2
YAW_WINDOW_S = 1.0             # yaw check window ASSUMED (prototype)
YAW_MIN_DV_MPS = 0.4           # IMU velocity change needed in a window ASSUMED (prototype)
YAW_SIGMA_DV_MPS = 0.15        # velocity-change error of a window: sqrt(2 x 0.1^2 (GPS, DS 0.1 m/s CEP inflated) +
                               # 0.05^2 (IMU over 1 s)) ASSUMED - the angle sigma is SIGMA_DV / |du| (review v0.8 (6))
YAW_SIGMA0_DEG = 15.0          # heading uncertainty after the pad set-up (phone compass +-5 deg; params heading.
                               # tolerance_deg 15 since the physics review) ASSUMED
YAW_Q_DEG2_PER_S = 1.8         # heading random walk ASSUMED - physics-review fix (medium 6): matched to the sim's own gyro
                               # z-bias model (SensorConfig: Gauss-Markov sigma 0.25 deg/s, tau 60 s, after the pad
                               # average): yaw drift 1-sigma 5.9 deg at 40 s, 10.3 deg at 60 s = 1.77 deg^2/s over the 60 s
                               # hover (scratch v08/physreview/calc_yawdrift.py); the old 0.04 deg^2/s (0.2 deg / sqrt(s))
                               # was 22-44x too small in variance and HEAD_OK overconfident
YAW_SIGMA_REL = 0.25           # angle-noise floor of a window [rad] (R = (SIGMA_DV / |du|)^2 + SIGMA_REL^2) ASSUMED -
                               # physics-review fix (medium 6): with R = (SIGMA_DV / |du|)^2 alone the measured window angles
                               # erred more than modelled (tilt-estimate error x g, latency and lever residuals all grow
                               # with the manoeuvre), so HEAD_OK was set while |psi error| > 10 deg in 7-8 % of the airborne
                               # time.  6-DOF trade-off study on FRESH seeds (scratch v08/fix/yaw2_*: calm 90-99, demo
                               # 190-199, heading set-up errors 45 / 90 deg seeds 70-74), floor 0 / 0.15 / 0.25 / 0.35 /
                               # 0.5 rad: heading-error RMS p50 6.1 / 5.5 / 4.8 / 3.7 / 2.5 deg (calm), HEAD_OK-while-wrong
                               # 8.4 / 5.7 / 3.5 / 2.1 / 0 % (calm), 7.5 / 5.8 / 3.2 / 0 / 0 % (demo) - but a 90 deg set-up
                               # mistake ended in V13 (40-72 m away, 0.9-3.8 m/s) in 5 of 5 seeds at 0.5 and in 0 of 5 at
                               # 0.25 (and at 0): 0.25 keeps the gross-error recovery and halves the overconfidence
HEAD_OK_DEG = 10.0             # heading confidence flag ASSUMED
INCONS_EPOCHS = 5              # GPS-IMU inconsistency flag: the GPS velocity rejected (either axis) in at least ...
INCONS_MIN_REJ = 3             # ... this many of the last INCONS_EPOCHS fused flight epochs (physics review HIGH 2: the
                               # FCU's plausibility rule (a) counts only while it is set; wind saturation keeps the
                               # innovations inside the gate) ASSUMED
LEVER_B_MM_CAD = (-74.3, -15.1, 312.2)   # antenna from the CG, body [mm] - CAD feasibility report option B (pod at theta
                                         # 192 deg, GPS mass included); params electronics.gps.antenna_lever_b_mm wins

# snapshot flags (nlink v3 GPS slot STATUS byte 0, TELEM v3 gps_flags, SNAP v3)
GF_FIX_OK = 0x01       # the estimator's GPS is current (FIX_OK above)
GF_GATE_OK = 0x02      # ON gate passed (good fix for stable_s)
GF_HEAD_OK = 0x04      # heading confidence
GF_FIX3D = 0x08        # the latest epoch: gpsFixOK and 3D
GF_CFG_OK = 0x10       # boot configuration acknowledged
GF_INIT = 0x20         # origin set, estimator initialised
GF_VER_OK = 0x40       # MON-VER answered
GF_INCONS = 0x80       # GPS-IMU inconsistent: the GPS velocity rejected in >= INCONS_MIN_REJ of the last INCONS_EPOCHS
                       # flight epochs (physics-review fix; this bit was GF_PAD, the filter's pad mode - nothing read it:
                       # the FCU knows its own state, the NAV's pad mode follows it)
GF_NAMES = {GF_FIX_OK: "fix_ok", GF_GATE_OK: "gate_ok", GF_HEAD_OK: "head_ok", GF_FIX3D: "fix3d", GF_CFG_OK: "cfg_ok",
            GF_INIT: "init", GF_VER_OK: "ver_ok", GF_INCONS: "incons"}

# WGS-84
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
E7 = 1e-7


# =========================================================================== #
# Part 1 - UBX protocol
# =========================================================================== #
def ubx_checksum(data: Iterable[int]) -> tuple[int, int]:
    """8-bit Fletcher (CK_A, CK_B) over ``data`` = class, id, length lo, length hi, payload."""
    a = b = 0
    for x in data:
        a = (a + (x & 0xFF)) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def ubx_frame(cls: int, mid: int, payload: bytes = b"") -> bytes:
    """A complete UBX frame (gps_ubx_encode)."""
    n = len(payload)
    if not (0 <= cls <= 0xFF and 0 <= mid <= 0xFF and n <= 0xFFFF):
        raise ValueError("class / id u8, length u16")
    body = bytes([cls, mid, n & 0xFF, n >> 8]) + bytes(payload)
    ca, cb = ubx_checksum(body)
    return bytes([UBX_SYNC1, UBX_SYNC2]) + body + bytes([ca, cb])


@dataclass(frozen=True)
class NavPosllh:
    itow_ms: int
    lon_e7: int
    lat_e7: int
    height_mm: int
    hmsl_mm: int
    hacc_mm: int
    vacc_mm: int

    def pack(self) -> bytes:
        return struct.pack(POSLLH_FMT, self.itow_ms, self.lon_e7, self.lat_e7, self.height_mm, self.hmsl_mm,
                           self.hacc_mm, self.vacc_mm)

    @classmethod
    def unpack(cls, p: bytes) -> "NavPosllh":
        return cls(*struct.unpack(POSLLH_FMT, p))


@dataclass(frozen=True)
class NavVelned:
    itow_ms: int
    vel_n_cms: int
    vel_e_cms: int
    vel_d_cms: int
    speed_cms: int
    gspeed_cms: int
    heading_e5: int
    sacc_cms: int
    cacc_e5: int

    def pack(self) -> bytes:
        return struct.pack(VELNED_FMT, self.itow_ms, self.vel_n_cms, self.vel_e_cms, self.vel_d_cms, self.speed_cms,
                           self.gspeed_cms, self.heading_e5, self.sacc_cms, self.cacc_e5)

    @classmethod
    def unpack(cls, p: bytes) -> "NavVelned":
        return cls(*struct.unpack(VELNED_FMT, p))


@dataclass(frozen=True)
class NavSol:
    itow_ms: int
    ftow_ns: int = 0
    week: int = 0
    gps_fix: int = 0
    flags: int = 0
    ecef_x_cm: int = 0
    ecef_y_cm: int = 0
    ecef_z_cm: int = 0
    pacc_cm: int = 0
    ecef_vx_cms: int = 0
    ecef_vy_cms: int = 0
    ecef_vz_cms: int = 0
    sacc_cms: int = 0
    pdop_e2: int = 0
    reserved1: int = 0
    num_sv: int = 0
    reserved2: int = 0

    def pack(self) -> bytes:
        return struct.pack(SOL_FMT, self.itow_ms, self.ftow_ns, self.week, self.gps_fix, self.flags, self.ecef_x_cm,
                           self.ecef_y_cm, self.ecef_z_cm, self.pacc_cm, self.ecef_vx_cms, self.ecef_vy_cms,
                           self.ecef_vz_cms, self.sacc_cms, self.pdop_e2, self.reserved1, self.num_sv, self.reserved2)

    @classmethod
    def unpack(cls, p: bytes) -> "NavSol":
        return cls(*struct.unpack(SOL_FMT, p))

    @property
    def fix_ok(self) -> bool:
        return bool(self.flags & SOL_FLAG_FIX_OK)


@dataclass(frozen=True)
class UbxMsg:
    """One complete frame from the parser (payload stored up to UBX_MAX_PAYLOAD bytes)."""

    cls: int
    id: int
    length: int
    payload: bytes
    truncated: bool = False


def decode_msg(m: UbxMsg):
    """The typed message (NavPosllh / NavVelned / NavSol / ('ACK' | 'NAK', cls, id) / MonVer) or None (unknown or a
    wrong length - a message whose length is not its type's is never decoded)."""
    if m.truncated:
        if (m.cls, m.id) == (CLS_MON, ID_MON_VER) and m.length >= 40:
            return MonVer.unpack(m.payload)
        return None
    key = (m.cls, m.id)
    if key == (CLS_NAV, ID_NAV_POSLLH) and m.length == LEN_POSLLH:
        return NavPosllh.unpack(m.payload)
    if key == (CLS_NAV, ID_NAV_VELNED) and m.length == LEN_VELNED:
        return NavVelned.unpack(m.payload)
    if key == (CLS_NAV, ID_NAV_SOL) and m.length == LEN_SOL:
        return NavSol.unpack(m.payload)
    if m.cls == CLS_ACK and m.id in (ID_ACK_ACK, ID_ACK_NAK) and m.length == 2:
        a, b = struct.unpack(ACK_FMT, m.payload)
        return ("ACK" if m.id == ID_ACK_ACK else "NAK", a, b)
    if key == (CLS_MON, ID_MON_VER) and m.length >= 40:
        return MonVer.unpack(m.payload)
    return None


@dataclass(frozen=True)
class MonVer:
    sw: str
    hw: str

    @classmethod
    def unpack(cls, p: bytes) -> "MonVer":
        def s(b: bytes) -> str:
            return b.split(b"\x00", 1)[0].decode("ascii", errors="replace").strip()
        return cls(s(p[0:30]), s(p[30:40]))

    @property
    def module(self) -> int:
        return {HW_UBLOX6: MODULE_UBLOX6, HW_UBLOX7: MODULE_UBLOX7, HW_UBLOX8: MODULE_UBLOX8}.get(self.hw, MODULE_UNKNOWN)


def mon_ver_payload(sw: str, hw: str, ext: Sequence[str] = ()) -> bytes:
    """A MON-VER answer payload (tests / the receiver stub): sw char[30], hw char[10], rom char[30], ext char[30]..."""
    def pad(t: str, n: int) -> bytes:
        b = t.encode("ascii")[:n]
        return b + bytes(n - len(b))
    return pad(sw, 30) + pad(hw, 10) + pad("", 30) + b"".join(pad(e, 30) for e in ext)


# ---- configuration frames (the NAV's boot sequence; byte-exact) ----
def cfg_prt_uart1(baud: int = GPS_BAUD) -> bytes:
    """CFG-PRT for UART1: 8N1 at ``baud``, in UBX + NMEA, out UBX only (no NMEA output at all)."""
    return ubx_frame(CLS_CFG, ID_CFG_PRT, struct.pack(CFG_PRT_FMT, 1, 0, 0, CFG_PRT_MODE_8N1, int(baud),
                                                      PROTO_UBX | PROTO_NMEA, PROTO_UBX, 0, 0))


def cfg_rate(meas_ms: int = 200) -> bytes:
    """CFG-RATE: measurement period (200 ms = 5 Hz, params flight_computer.nav.gps.rate_hz), 1 cycle, GPS time."""
    return ubx_frame(CLS_CFG, ID_CFG_RATE, struct.pack(CFG_RATE_FMT, int(meas_ms), 1, 1))


def cfg_msg(cls: int, mid: int, rate: int = 1) -> bytes:
    """CFG-MSG (3-byte form): output rate of one message on the current port (1 = every solution)."""
    return ubx_frame(CLS_CFG, ID_CFG_MSG, struct.pack(CFG_MSG_FMT, cls, mid, rate))


def cfg_nav5_airborne() -> bytes:
    """CFG-NAV5 with mask 0x0001: only dynModel = 6 (airborne < 1 g) is applied, every other field 0 (ignored)."""
    return ubx_frame(CLS_CFG, ID_CFG_NAV5, struct.pack(CFG_NAV5_FMT, NAV5_MASK_DYN, DYN_AIRBORNE_1G, 0, 0, 0, 0, 0, 0,
                                                       0, 0, 0, 0, 0, 0, 0, 0))


def mon_ver_poll() -> bytes:
    return ubx_frame(CLS_MON, ID_MON_VER, b"")


def config_sequence(rate_hz: float = 5.0) -> list[tuple[str, bytes]]:
    """The ACK-checked frames after the port is at 38400, in order (name, frame)."""
    meas = int(round(1000.0 / rate_hz))
    return [("CFG-RATE", cfg_rate(meas)), ("CFG-MSG POSLLH", cfg_msg(CLS_NAV, ID_NAV_POSLLH)),
            ("CFG-MSG VELNED", cfg_msg(CLS_NAV, ID_NAV_VELNED)), ("CFG-MSG SOL", cfg_msg(CLS_NAV, ID_NAV_SOL)),
            ("CFG-NAV5", cfg_nav5_airborne())]


# ---- streaming parser ----
_S_SYNC1, _S_SYNC2, _S_CLASS, _S_ID, _S_LEN1, _S_LEN2, _S_PAYLOAD, _S_CKA, _S_CKB = range(9)
PARSER_STAT_KEYS = ("frames", "bad_ck", "bad_len", "truncated", "skipped")


class UbxParser:
    """Byte-at-a-time UBX parser (module docstring; == gps_ubx_parse_byte).  Counters are uint16 (wrap)."""

    def __init__(self) -> None:
        self.stats = {k: 0 for k in PARSER_STAT_KEYS}
        self.reset()

    def reset(self) -> None:
        self.state = _S_SYNC1
        self.cls = self.id = 0
        self.length = 0
        self.n = 0
        self.ck_a = self.ck_b = 0
        self.buf = bytearray()

    def _count(self, k: str, n: int = 1) -> None:
        self.stats[k] = (self.stats[k] + n) & 0xFFFF

    def _ck(self, b: int) -> None:
        self.ck_a = (self.ck_a + b) & 0xFF
        self.ck_b = (self.ck_b + self.ck_a) & 0xFF

    def feed_byte(self, b: int) -> UbxMsg | None:
        b &= 0xFF
        s = self.state
        if s == _S_SYNC1:
            if b == UBX_SYNC1:
                self.state = _S_SYNC2
            else:
                self._count("skipped")
            return None
        if s == _S_SYNC2:
            if b == UBX_SYNC2:
                self.state = _S_CLASS
                self.ck_a = self.ck_b = 0
            elif b == UBX_SYNC1:
                self._count("skipped")            # B5 B5 62: the second B5 starts the frame
            else:
                self._count("skipped", 2)
                self.state = _S_SYNC1
            return None
        if s == _S_CLASS:
            self.cls = b
            self._ck(b)
            self.state = _S_ID
            return None
        if s == _S_ID:
            self.id = b
            self._ck(b)
            self.state = _S_LEN1
            return None
        if s == _S_LEN1:
            self.length = b
            self._ck(b)
            self.state = _S_LEN2
            return None
        if s == _S_LEN2:
            self.length |= b << 8
            self._ck(b)
            if self.length > UBX_LEN_LIMIT:
                self._count("bad_len")
                self.state = _S_SYNC1
                return None
            self.n = 0
            self.buf = bytearray()
            self.state = _S_PAYLOAD if self.length > 0 else _S_CKA
            return None
        if s == _S_PAYLOAD:
            if self.n < UBX_MAX_PAYLOAD:
                self.buf.append(b)
            self.n += 1
            self._ck(b)
            if self.n >= self.length:
                self.state = _S_CKA
            return None
        if s == _S_CKA:
            if b != self.ck_a:
                self._count("bad_ck")
                self.state = _S_SYNC1
                return None
            self.state = _S_CKB
            return None
        # _S_CKB
        self.state = _S_SYNC1
        if b != self.ck_b:
            self._count("bad_ck")
            return None
        self._count("frames")
        tr = self.length > UBX_MAX_PAYLOAD
        if tr:
            self._count("truncated")
        return UbxMsg(self.cls, self.id, self.length, bytes(self.buf), tr)

    def feed(self, data: Iterable[int]) -> list[UbxMsg]:
        out = []
        for b in data:
            m = self.feed_byte(b)
            if m is not None:
                out.append(m)
        return out


# ---- epoch assembly ----
@dataclass(frozen=True)
class GpsEpoch:
    """One navigation epoch (POSLLH + VELNED + SOL of one iTOW) as the NAV holds it."""

    itow_ms: int
    lat_e7: int
    lon_e7: int
    hmsl_mm: int
    hacc_mm: int
    vacc_mm: int
    vel_n_cms: int
    vel_e_cms: int
    vel_d_cms: int
    sacc_cms: int
    gps_fix: int
    fix_ok: bool
    num_sv: int
    pacc_cm: int
    tick: int = 0                  # NAV tick at which it completed (the last message parsed)

    @property
    def hacc_m(self) -> float:
        return self.hacc_mm / 1000.0

    @property
    def sacc_mps(self) -> float:
        return self.sacc_cms / 100.0


ASSEMBLER_STAT_KEYS = ("epochs", "incomplete", "duplicate")
_PART_POS, _PART_VEL, _PART_SOL = 1, 2, 4


class EpochAssembler:
    """iTOW matching (module docstring)."""

    def __init__(self) -> None:
        self.stats = {k: 0 for k in ASSEMBLER_STAT_KEYS}
        self.itow: int | None = None
        self.mask = 0
        self.pos: NavPosllh | None = None
        self.vel: NavVelned | None = None
        self.sol: NavSol | None = None

    def _count(self, k: str) -> None:
        self.stats[k] = (self.stats[k] + 1) & 0xFFFF

    def add(self, msg, tick: int) -> GpsEpoch | None:
        """Feed one decoded NAV message (others are ignored); returns the epoch it completes, else None."""
        if isinstance(msg, NavPosllh):
            part = _PART_POS
        elif isinstance(msg, NavVelned):
            part = _PART_VEL
        elif isinstance(msg, NavSol):
            part = _PART_SOL
        else:
            return None
        if self.itow is None or msg.itow_ms != self.itow:
            if self.mask:
                self._count("incomplete")
            self.itow, self.mask = msg.itow_ms, 0
            self.pos = self.vel = self.sol = None
        if self.mask & part:
            self._count("duplicate")
        self.mask |= part
        if part == _PART_POS:
            self.pos = msg
        elif part == _PART_VEL:
            self.vel = msg
        else:
            self.sol = msg
        if self.mask != (_PART_POS | _PART_VEL | _PART_SOL):
            return None
        p, v, s = self.pos, self.vel, self.sol
        self.itow, self.mask = None, 0
        self._count("epochs")
        return GpsEpoch(itow_ms=p.itow_ms, lat_e7=p.lat_e7, lon_e7=p.lon_e7, hmsl_mm=p.hmsl_mm, hacc_mm=p.hacc_mm,
                        vacc_mm=p.vacc_mm, vel_n_cms=v.vel_n_cms, vel_e_cms=v.vel_e_cms, vel_d_cms=v.vel_d_cms,
                        sacc_cms=v.sacc_cms, gps_fix=s.gps_fix, fix_ok=s.fix_ok, num_sv=s.num_sv, pacc_cm=s.pacc_cm,
                        tick=int(tick))


def _i32(x: int) -> int:
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x


class ItowClock:
    """Learned NAV-ms - iTOW offset (module docstring); all integer arithmetic mod 2^32."""

    def __init__(self, ms_per_tick: int, latency_ms: int):
        self.ms_per_tick = int(ms_per_tick)
        self.latency_ms = int(latency_ms)
        self.offset: int | None = None
        self.relearned = 0

    def update(self, itow_ms: int, tick: int) -> int:
        """Learn from an epoch completed at ``tick``; returns its measurement time t_e on the NAV ms clock (uint32)."""
        d = _i32(self.ms_per_tick * tick - itow_ms)
        if self.offset is None or abs(_i32(d - self.offset)) > ITOW_JUMP_MS:
            if self.offset is not None:
                self.relearned += 1
            self.offset = d
        else:
            self.offset = min(d, _i32(self.offset + ITOW_LEAK_MS))
        return (itow_ms + self.offset - self.latency_ms) & 0xFFFFFFFF


# ---- boot configuration ----
CFGS_WAIT, CFGS_PROBE, CFGS_SWITCH, CFGS_SETTLE, CFGS_PRT, CFGS_SEQ, CFGS_VER, CFGS_DONE = range(8)


class GpsConfigurator:
    """The NAV's boot configuration of the receiver (module docstring).  ``tick(msgs)`` once per NAV tick with the
    decoded messages parsed in that tick; returns actions [('baud', n) | ('tx', bytes)] for the UART driver."""

    def __init__(self, loop_hz: int, rate_hz: float = 5.0, baud: int = GPS_BAUD):
        self.hz = int(loop_hz)
        self.baud = int(baud)
        self.seq = config_sequence(rate_hz)
        self.n_wait = max(1, lp.round_half_away(CFG_BOOT_WAIT_S * self.hz))
        self.n_ack = max(1, lp.round_half_away(CFG_ACK_TIMEOUT_S * self.hz))
        self.n_settle = max(1, lp.round_half_away(CFG_SETTLE_S * self.hz))
        self.n_ver = max(1, lp.round_half_away(CFG_VER_TIMEOUT_S * self.hz))
        self.state = CFGS_WAIT
        self.t = 0                    # ticks in the current state
        self.tries = 0
        self.idx = 0                  # index into seq
        self.acked = 0                # ACK-ACK count of the sequence
        self.prt_acked = False
        self.cfg_ok = False
        self.ver: MonVer | None = None
        self.naks = 0
        self.timeouts = 0
        self.log: list[str] = []

    @property
    def done(self) -> bool:
        return self.state == CFGS_DONE

    @property
    def ver_ok(self) -> bool:
        return self.ver is not None

    def _go(self, st: int) -> None:
        self.state, self.t = st, 0

    def tick(self, msgs: Sequence) -> list[tuple]:
        acts: list[tuple] = []
        s = self.state
        self.t += 1
        acks = [m for m in msgs if isinstance(m, tuple) and len(m) == 3 and m[0] in ("ACK", "NAK")]
        if s == CFGS_WAIT:
            if self.t >= self.n_wait:
                acts += [("baud", self.baud), ("tx", cfg_prt_uart1(self.baud))]
                self.tries = 1
                self._go(CFGS_PROBE)
        elif s in (CFGS_PROBE, CFGS_PRT):
            got = [a for a in acks if (a[1], a[2]) == (CLS_CFG, ID_CFG_PRT)]
            if got and got[-1][0] == "ACK":
                self.prt_acked = True
                self.log.append(f"CFG-PRT acknowledged at {self.baud} ({'probe' if s == CFGS_PROBE else 'after 9600'})")
                acts.append(("tx", self.seq[0][1]))
                self.idx, self.tries = 0, 1
                self._go(CFGS_SEQ)
            elif got or self.t >= self.n_ack:
                if got:
                    self.naks += 1
                else:
                    self.timeouts += 1
                if s == CFGS_PROBE:                     # not at 38400: the factory default 9600
                    acts += [("baud", GPS_BAUD_DEFAULT), ("tx", cfg_prt_uart1(self.baud))]
                    self._go(CFGS_SETTLE)
                elif self.tries < CFG_TRIES:
                    self.tries += 1
                    acts += [("baud", GPS_BAUD_DEFAULT), ("tx", cfg_prt_uart1(self.baud))]
                    self._go(CFGS_SETTLE)
                else:
                    self.log.append("CFG-PRT never acknowledged: continuing at 38400 without it")
                    acts.append(("tx", self.seq[0][1]))
                    self.idx, self.tries = 0, 1
                    self._go(CFGS_SEQ)
        elif s == CFGS_SETTLE:
            if self.t >= self.n_settle:
                acts += [("baud", self.baud), ("tx", cfg_prt_uart1(self.baud))]
                self._go(CFGS_PRT)
        elif s == CFGS_SEQ:
            name, fr = self.seq[self.idx]
            cid = fr[3]
            got = [a for a in acks if (a[1], a[2]) == (CLS_CFG, cid)]
            ok = bool(got) and got[-1][0] == "ACK"
            fail = (bool(got) and not ok) or self.t >= self.n_ack
            if ok or (fail and self.tries >= CFG_TRIES):
                if ok:
                    self.acked += 1
                else:
                    self.log.append(f"{name} not acknowledged after {CFG_TRIES} tries")
                if fail and not ok:
                    if got:
                        self.naks += 1
                    else:
                        self.timeouts += 1
                self.idx += 1
                self.tries = 1
                if self.idx < len(self.seq):
                    acts.append(("tx", self.seq[self.idx][1]))
                    self._go(CFGS_SEQ)
                else:
                    self.cfg_ok = self.prt_acked and self.acked == len(self.seq)
                    acts.append(("tx", mon_ver_poll()))
                    self.tries = 1
                    self._go(CFGS_VER)
            elif fail:
                if got:
                    self.naks += 1
                else:
                    self.timeouts += 1
                self.tries += 1
                acts.append(("tx", fr))
                self._go(CFGS_SEQ)
        elif s == CFGS_VER:
            vs = [m for m in msgs if isinstance(m, MonVer)]
            if vs:
                self.ver = vs[-1]
                self.log.append(f"MON-VER sw '{self.ver.sw}' hw '{self.ver.hw}'")
                self._go(CFGS_DONE)
            elif self.t >= self.n_ver:
                if self.tries < CFG_TRIES:
                    self.tries += 1
                    acts.append(("tx", mon_ver_poll()))
                    self._go(CFGS_VER)
                else:
                    self.timeouts += 1
                    self._go(CFGS_DONE)
        return acts


class UbxReceiverStub:
    """A minimal receiver for tests and co-simulation: answers CFG frames with ACK-ACK (or ACK-NAK for the ids in
    ``nak``) at its current baud, changes its baud on CFG-PRT (after the ACK), answers MON-VER, ignores bytes sent at
    the wrong baud.  Not a GPS model (the 6-DOF owns the measurement errors)."""

    def __init__(self, baud: int = GPS_BAUD_DEFAULT, hw: str = HW_UBLOX6, sw: str = "7.03 (45969)",
                 nak: Sequence[int] = (), silent: bool = False):
        self.baud = int(baud)
        self.hw, self.sw = hw, sw
        self.nak = set(nak)
        self.silent = silent
        self.parser = UbxParser()
        self.cfg: dict[int, bytes] = {}

    def receive(self, data: bytes, baud: int) -> bytes:
        """Bytes the NAV sent at ``baud``; returns the receiver's answer bytes (sent at the receiver's baud AFTER the
        call - a baud change applies after its own ACK)."""
        if baud != self.baud or self.silent:
            return b""
        out = b""
        new_baud = None
        for m in self.parser.feed(data):
            if m.cls == CLS_CFG:
                ok = m.id not in self.nak
                out += ubx_frame(CLS_ACK, ID_ACK_ACK if ok else ID_ACK_NAK, bytes([m.cls, m.id]))
                if ok:
                    self.cfg[m.id] = m.payload
                    if m.id == ID_CFG_PRT and len(m.payload) == LEN_CFG_PRT:
                        new_baud = struct.unpack(CFG_PRT_FMT, m.payload)[4]
            elif (m.cls, m.id) == (CLS_MON, ID_MON_VER) and m.length == 0:
                out += ubx_frame(CLS_MON, ID_MON_VER, mon_ver_payload(self.sw, self.hw))
        ans_baud = self.baud
        if new_baud is not None:
            self.baud = int(new_baud)
        return out if ans_baud == baud else b""


# =========================================================================== #
# local frame
# =========================================================================== #
@dataclass(frozen=True)
class LocalFrame:
    """Flat-earth W frame (x north, y west) at an origin (lat0, lon0) [1e-7 deg]: x = dlat k_n, y = -dlon k_e."""

    lat0_e7: int
    lon0_e7: int
    k_n: float                 # metres per 1e-7 deg of latitude
    k_e: float                 # metres per 1e-7 deg of longitude

    @classmethod
    def at(cls, lat0_e7: int, lon0_e7: int) -> "LocalFrame":
        phi = math.radians(lat0_e7 * E7)
        s = math.sin(phi)
        w = 1.0 - WGS84_E2 * s * s
        m_rad = WGS84_A * (1.0 - WGS84_E2) / (w * math.sqrt(w))
        n_rad = WGS84_A / math.sqrt(w)
        d = math.radians(E7)
        return cls(int(lat0_e7), int(lon0_e7), d * m_rad, d * n_rad * math.cos(phi))

    def to_w(self, lat_e7: int, lon_e7: int) -> tuple[float, float]:
        dlat = int(lat_e7) - self.lat0_e7
        dlon = ((int(lon_e7) - self.lon0_e7 + 1800000000) % 3600000000) - 1800000000
        return dlat * self.k_n, -dlon * self.k_e

    def to_llh(self, x: float, y: float) -> tuple[int, int]:
        """Inverse (truth -> UBX, the receiver model): integer 1e-7 deg, rounded half away from zero."""
        return (self.lat0_e7 + lp.round_half_away(x / self.k_n), self.lon0_e7 + lp.round_half_away(-y / self.k_e))


def epoch_frames(ep: GpsEpoch, week: int = 2400, order: str = "psv") -> bytes:
    """UBX bytes of one epoch as a NEO-6M sends them (POSLLH, SOL, VELNED by default; ``order`` p/s/v letters):
    the receiver side for tests and the 6-DOF (the 6-DOF's error model chooses the fields)."""
    p = NavPosllh(ep.itow_ms, ep.lon_e7, ep.lat_e7, ep.hmsl_mm, ep.hmsl_mm, ep.hacc_mm, ep.vacc_mm)
    vn, ve = ep.vel_n_cms, ep.vel_e_cms
    gs = lp.round_half_away(math.hypot(vn, ve))
    sp = lp.round_half_away(math.sqrt(vn * vn + ve * ve + ep.vel_d_cms * ep.vel_d_cms))
    hd = lp.round_half_away((math.degrees(math.atan2(ve, vn)) % 360.0) * 1e5) if gs > 0 else 0
    v = NavVelned(ep.itow_ms, vn, ve, ep.vel_d_cms, sp, gs, hd, ep.sacc_cms, 5000000)
    s = NavSol(ep.itow_ms, 0, week, ep.gps_fix, (SOL_FLAG_FIX_OK | 0x0C) if ep.fix_ok else 0x0C, pacc_cm=ep.pacc_cm,
               sacc_cms=ep.sacc_cms, pdop_e2=150, num_sv=ep.num_sv)
    parts = {"p": ubx_frame(CLS_NAV, ID_NAV_POSLLH, p.pack()), "s": ubx_frame(CLS_NAV, ID_NAV_SOL, s.pack()),
             "v": ubx_frame(CLS_NAV, ID_NAV_VELNED, v.pack())}
    return b"".join(parts[c] for c in order)


# =========================================================================== #
# Part 2 - the NAV GPS-aided horizontal estimator
# =========================================================================== #
def _fq(T: float, qa: float, qb: float) -> tuple[list, list]:
    F = [[1.0, T, -0.5 * T * T], [0.0, 1.0, -T], [0.0, 0.0, 1.0]]
    T2, T3, T4, T5 = T * T, T ** 3, T ** 4, T ** 5
    Q = [[qa * T3 / 3.0 + qb * T5 / 20.0, qa * T2 / 2.0 + qb * T4 / 8.0, -qb * T3 / 6.0],
         [qa * T2 / 2.0 + qb * T4 / 8.0, qa * T + qb * T3 / 3.0, -qb * T2 / 2.0],
         [-qb * T3 / 6.0, -qb * T2 / 2.0, qb * T]]
    return F, Q


def _mm(A: list, B: list) -> list:
    return [[sum(A[i][k] * B[k][j] for k in range(len(B))) for j in range(len(B[0]))] for i in range(len(A))]


def _tr(A: list) -> list:
    return [list(r) for r in zip(*A)]


def _riccati(P0: list, n: int, T: float, qa: float, qb: float, sp: float, sv: float) -> tuple[list, list]:
    """n epoch updates from the posterior P0: (gain K_pv of each epoch, the last prior)."""
    F, Q = _fq(T, qa, qb)
    P = [r[:] for r in P0]
    Ks, Pm = [], None
    for _ in range(n):
        Pm = [[a + b for a, b in zip(r1, r2)] for r1, r2 in zip(_mm(_mm(F, P), _tr(F)), Q)]
        s00, s01, s11 = Pm[0][0] + sp * sp, Pm[0][1], Pm[1][1] + sv * sv
        det = s00 * s11 - s01 * s01
        i00, i01, i11 = s11 / det, -s01 / det, s00 / det
        K = [[Pm[i][0] * i00 + Pm[i][1] * i01, Pm[i][0] * i01 + Pm[i][1] * i11] for i in range(3)]
        IKH = [[(1.0 if i == j else 0.0) - (K[i][0] if j == 0 else 0.0) - (K[i][1] if j == 1 else 0.0)
                for j in range(3)] for i in range(3)]
        P = _mm(IKH, Pm)
        P = [[0.5 * (P[i][j] + P[j][i]) for j in range(3)] for i in range(3)]
        Ks.append(K)
    return Ks, Pm


@dataclass(frozen=True)
class FilterGains:
    """One gain set: K_pv (3 x 2, columns p and v), K_p and K_v (3) for a single accepted measurement."""

    k_pv: tuple
    k_p: tuple
    k_v: tuple


@dataclass(frozen=True)
class GpsGains:
    steady: FilterGains
    acq: FilterGains
    s_pp: float                # steady innovation variances (gates)
    s_vv: float
    prior: tuple               # the steady prior covariance (diagnostics)


def compute_gains(rate_hz: float, q_acc: float = KF_Q_ACC, q_b: float = KF_Q_B, sigma_p: float = KF_SIGMA_P_M,
                  sigma_v: float = KF_SIGMA_V_MPS, sigma_v0: float = KF_SIGMA_V0_MPS, sigma_b0: float = KF_SIGMA_B0_MPS2,
                  acq_epoch: int = ACQ_GAIN_EPOCH) -> GpsGains:
    """The fixed gains (module docstring): steady state of the epoch-rate Riccati recursion (converged from a wide
    prior, 3000 epochs) and the acquisition set = the gain at ``acq_epoch`` after a take-off prior (the steady position
    rows, v sigma_v0, b sigma_b0 uncorrelated).  Computed once in float64 (the firmware takes the numbers from
    params_gen.h)."""
    T = 1.0 / float(rate_hz)
    Ks, Pm = _riccati([[sigma_p ** 2, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, sigma_b0 ** 2]], 3000, T, q_acc, q_b,
                      sigma_p, sigma_v)

    def single(P: list) -> tuple[tuple, tuple]:
        sp_, sv_ = P[0][0] + sigma_p ** 2, P[1][1] + sigma_v ** 2
        return tuple(P[i][0] / sp_ for i in range(3)), tuple(P[i][1] / sv_ for i in range(3))

    kp, kv = single(Pm)
    steady = FilterGains(tuple(tuple(r) for r in Ks[-1]), kp, kv)
    P0 = [[Pm[0][0], Pm[0][1], 0.0], [Pm[1][0], max(Pm[1][1], sigma_v0 ** 2), 0.0], [0.0, 0.0, sigma_b0 ** 2]]
    Ka, Pa = _riccati(P0, max(1, int(acq_epoch)), T, q_acc, q_b, sigma_p, sigma_v)
    kpa, kva = single(Pa)
    acq = FilterGains(tuple(tuple(r) for r in Ka[-1]), kpa, kva)
    return GpsGains(steady, acq, Pm[0][0] + sigma_p ** 2, Pm[1][1] + sigma_v ** 2, tuple(tuple(r) for r in Pm))


@dataclass(frozen=True)
class NavGpsParams:
    """Every number of the NAV GPS part.  ``from_params`` is the constructor the sim and the generator use."""

    loop_hz: int = 250
    fusion_hz: int = 50
    rate_hz: float = 5.0
    latency_s: float = 0.22
    lever_b_m: tuple = tuple(x / 1000.0 for x in LEVER_B_MM_CAD)
    lever_source: str = "CAD report option B (module constant)"
    pad_azimuth_deg: float = 0.0
    gate_min_sv: int = 6
    gate_max_hacc_m: float = 5.0
    gate_max_sacc_mps: float = 1.0
    gate_stable_s: float = 5.0
    ok_min_sv: int = OK_MIN_SV
    ok_max_hacc_m: float = OK_MAX_HACC_M
    q_acc: float = KF_Q_ACC
    q_b: float = KF_Q_B
    sigma_p_m: float = KF_SIGMA_P_M
    sigma_v_mps: float = KF_SIGMA_V_MPS
    sigma_v0_mps: float = KF_SIGMA_V0_MPS
    sigma_b0_mps2: float = KF_SIGMA_B0_MPS2
    acq_gain_epoch: int = ACQ_GAIN_EPOCH
    acq_epochs: int = ACQ_EPOCHS
    gate_sigma: float = GATE_SIGMA
    gate_p_m: float = GATE_P_M
    reset_after_s: float = RESET_AFTER_S
    vel_reset_epochs: int = VEL_RESET_EPOCHS
    lost_after_epochs: int = LOST_AFTER_EPOCHS
    yaw_correction: bool = True
    yaw_window_s: float = YAW_WINDOW_S
    yaw_min_dv_mps: float = YAW_MIN_DV_MPS
    yaw_sigma_dv_mps: float = YAW_SIGMA_DV_MPS
    yaw_sigma0_deg: float = YAW_SIGMA0_DEG
    yaw_q_deg2_per_s: float = YAW_Q_DEG2_PER_S
    yaw_sigma_rel: float = YAW_SIGMA_REL
    head_ok_deg: float = HEAD_OK_DEG
    lever_comp: bool = True

    @property
    def step_ticks(self) -> int:
        return self.loop_hz // self.fusion_hz

    @property
    def t_step(self) -> float:
        return self.step_ticks / self.loop_hz

    @property
    def ms_per_tick(self) -> int:
        return 1000 // self.loop_hz

    @property
    def latency_ms(self) -> int:
        return lp.round_half_away(self.latency_s * 1000.0)

    @property
    def n_corr(self) -> int:
        """Buffer entries corrected after a fusion: the ones the next epoch can read (latency steps - epoch period
        steps + 2, at least 1, at most BUF_N)."""
        ls = lp.round_half_away(self.latency_s / self.t_step)
        ps = lp.round_half_away(1.0 / (self.rate_hz * self.t_step))
        return max(1, min(BUF_N, ls - ps + 2))

    @property
    def gate_stable_ticks(self) -> int:
        from fractions import Fraction
        x = Fraction(self.gate_stable_s) * self.loop_hz
        return int(math.ceil(x))

    @property
    def fresh_ticks(self) -> int:
        return int(math.ceil(self.lost_after_epochs * self.loop_hz / self.rate_hz))

    @property
    def reset_after_epochs(self) -> int:
        return max(1, int(math.ceil(self.reset_after_s * self.rate_hz - 1e-9)))

    def gains(self) -> GpsGains:
        return compute_gains(self.rate_hz, self.q_acc, self.q_b, self.sigma_p_m, self.sigma_v_mps, self.sigma_v0_mps,
                             self.sigma_b0_mps2, self.acq_gain_epoch)

    def check(self) -> None:
        if self.loop_hz % self.fusion_hz or self.step_ticks < 1:
            raise ValueError("flight_computer.nav.gps.fusion_hz must divide control.loop_hz")
        if 1000 % self.loop_hz:
            raise ValueError("control.loop_hz must divide 1000 (integer NAV milliseconds per tick)")
        if not (self.rate_hz > 0 and self.latency_s >= 0 and len(self.lever_b_m) == 3):
            raise ValueError("gps rate_hz > 0, latency_s >= 0, lever arm 3 components")
        if lp.round_half_away(self.latency_s / self.t_step) >= BUF_N - 2:
            raise ValueError(f"latency {self.latency_s} s does not fit the {BUF_N}-step delayed-state buffer")

    @classmethod
    def from_params(cls, p: dict, **overrides) -> "NavGpsParams":
        """params.json flight_computer.nav.gps (rate_hz, latency_s_ASSUMED, fusion_hz), control.loop_hz,
        mission.vtol.position_hold (heading.pad_azimuth_deg, gps_gate_on_ASSUMED), electronics.gps.antenna_lever_b_mm
        (if present; else the CAD-report constant).  Keyword overrides replace fields (studies)."""
        g = p["flight_computer"]["nav"]["gps"]
        ph = p["mission"]["vtol"].get("position_hold", {})
        gate = ph.get("gps_gate_on_ASSUMED", {})
        kw = dict(loop_hz=int(p["control"]["loop_hz"]), fusion_hz=int(g.get("fusion_hz", 50)),
                  rate_hz=float(g.get("rate_hz", 5)), latency_s=float(g.get("latency_s_ASSUMED", 0.22)),
                  pad_azimuth_deg=float(ph.get("heading", {}).get("pad_azimuth_deg", 0.0)))
        if gate:
            if str(gate.get("fix", "3D")) != "3D":
                raise ValueError("position_hold.gps_gate_on_ASSUMED.fix must be '3D'")
            kw.update(gate_min_sv=int(gate["min_sv"]), gate_max_hacc_m=float(gate["max_hacc_m"]),
                      gate_max_sacc_mps=float(gate["max_sacc_mps"]), gate_stable_s=float(gate["stable_s"]))
        lev = p.get("electronics", {}).get("gps", {}).get("antenna_lever_b_mm")
        if lev is not None:
            kw.update(lever_b_m=tuple(float(x) / 1000.0 for x in lev), lever_source="params electronics.gps."
                                                                                   "antenna_lever_b_mm")
        kw.update({k: v for k, v in overrides.items() if v is not None})
        gp = cls(**kw)
        gp.check()
        return gp


@dataclass(frozen=True)
class GpsSnapshot:
    """What the NAV latches at each 50 Hz step for the nlink slots: wire integers (quantised here, once)."""

    flags: int = 0
    num_sv: int = 0
    hacc_dm: int = 255
    reset_count: int = 0
    off_x_cm: int = 0
    off_y_cm: int = 0
    px_cm: int = 0
    py_cm: int = 0
    vx_cms: int = 0
    vy_cms: int = 0
    cos_q15: int = 32767
    sin_q15: int = 0

    def flag(self, b: int) -> bool:
        return bool(self.flags & b)


I16 = lp.I16
Q15 = 32767


def quant_cm(x_m: float) -> int:
    return lp.quantize(float(x_m), 100.0, I16)


def quant_q15(x: float) -> int:
    return lp.quantize(float(x), float(Q15), (-Q15, Q15))


def quant_hacc_dm(hacc_mm: int) -> int:
    """hAcc mm -> 0.1 m, rounded half up (integer: (mm + 50) // 100), saturated to 255."""
    return min(255, (int(hacc_mm) + 50) // 100)


@dataclass
class _Entry:
    m: int = -1
    pa: list = field(default_factory=lambda: [0.0, 0.0])   # antenna position (W)
    va: list = field(default_factory=lambda: [0.0, 0.0])   # antenna velocity (W)
    u: list = field(default_factory=lambda: [0.0, 0.0])    # IMU-only velocity (yaw check)
    lv: list = field(default_factory=lambda: [0.0, 0.0])   # lever-arm velocity (W)


class NavGpsFilter:
    """The NAV's GPS-aided horizontal estimator (module docstring Part 2).  Call ``tick`` once per NAV tick, in the
    correction part after the frame push, with the tick's body specific force, the CORRECTED attitude, the
    bias-corrected body rate, the pad flag and the epochs completed in this tick (with their t_e)."""

    def __init__(self, gp: NavGpsParams):
        gp.check()
        self.gp = gp
        self.gains = gp.gains()
        self.k = -1                                 # NAV tick index of the last call
        self.acc = [0.0, 0.0, 0.0]
        self.n_acc = 0
        self.buf = [_Entry() for _ in range(BUF_N)]
        self.m_now = -1                             # last step index
        self.pending: list[tuple[GpsEpoch, int]] = []   # (epoch, t_e ms) completed since the last fusion
        self.p = [0.0, 0.0]
        self.v = [0.0, 0.0]
        self.b = [0.0, 0.0]
        self.u = [0.0, 0.0]
        self.psi0 = -math.radians(gp.pad_azimuth_deg)
        self.yaw_corr = 0.0
        self.c_psi, self.s_psi = math.cos(self.psi0), math.sin(self.psi0)
        self.p_psi = math.radians(gp.yaw_sigma0_deg) ** 2
        self.yaw_win: tuple | None = None           # (m, g0x, g0y, u0x, u0y)
        self.yaw_used = 0
        self.frame: LocalFrame | None = None
        self.init = False
        self.pad = True
        self.acq_left = 0
        self.rej_p_run = 0                          # consecutive epochs with the position rejected
        self.rej_v_run = 0
        self.rej_v_hist = 0                         # the last INCONS_EPOCHS flight epochs, bit 0 = newest: velocity rejected
        self.reset_count = 0
        self.off = [0.0, 0.0]                       # cumulative reset offset (W, m)
        self.k_last_used = None                     # tick of the last used epoch
        self.good_since = None                      # tick since which every epoch was good (ON gate)
        self.k_last_epoch = None
        self.last_fix3d = False
        self.last_sv = 0
        self.last_hacc_mm = 0xFFFFFFFF
        self.cfg_ok = False
        self.ver_ok = False
        self.snap = GpsSnapshot()
        self.cycle = 0                              # step counter (slot cycle counter)
        self.stats = {"fused": 0, "rej_p": 0, "rej_v": 0, "pos_resets": 0, "vel_resets": 0, "dropped_old": 0,
                      "dropped_stale": 0, "n_epoch_dropped": 0, "unusable": 0}
        self.last_innov = (0.0, 0.0, 0.0, 0.0)

    # ---- helpers ----
    def _rot(self, x: float, y: float) -> tuple[float, float]:
        """level -> W: Rz(psi)."""
        return self.c_psi * x - self.s_psi * y, self.s_psi * x + self.c_psi * y

    @property
    def psi(self) -> float:
        return self.psi0 + self.yaw_corr

    def _set_psi(self) -> None:
        self.c_psi, self.s_psi = math.cos(self.psi), math.sin(self.psi)

    def fix_ok(self, k: int) -> bool:
        return (self.init and self.k_last_used is not None and k - self.k_last_used <= self.gp.fresh_ticks)

    def gate_ok(self, k: int) -> bool:
        """ON gate: initialised, the fix current (FIX_OK) and every epoch good for stable_s (no gap > fresh)."""
        return (self.fix_ok(k) and self.good_since is not None and k - self.good_since >= self.gp.gate_stable_ticks)

    def head_ok(self) -> bool:
        return self.p_psi <= math.radians(self.gp.head_ok_deg) ** 2

    def incons(self) -> bool:
        """GPS-IMU inconsistent: the velocity rejected in >= INCONS_MIN_REJ of the last INCONS_EPOCHS flight epochs."""
        return bin(self.rej_v_hist & ((1 << INCONS_EPOCHS) - 1)).count("1") >= INCONS_MIN_REJ

    # ---- the tick ----
    def tick(self, k: int, accel_b: Sequence[float], q_corr: Sequence[float], rate_b: Sequence[float], on_pad: bool,
             epochs: Sequence[tuple[GpsEpoch, int]] = ()) -> GpsSnapshot:
        """NAV tick k (k = 0 at the first tick after boot); ``epochs`` = (epoch, t_e ms on the NAV clock) completed this
        tick (``ItowClock.update``).  Returns the latest latched snapshot."""
        self.k = k
        for e in epochs:
            self.pending.append(e)
        self.acc[0] += float(accel_b[0])
        self.acc[1] += float(accel_b[1])
        self.acc[2] += float(accel_b[2])
        self.n_acc += 1
        j = k % self.gp.step_ticks
        if j == 0 and self.pending:
            if len(self.pending) > 1:
                self.stats["n_epoch_dropped"] += len(self.pending) - 1
            ep, te = self.pending[-1]
            self.pending = []
            self._fuse(k, ep, te)
        if j == self.gp.step_ticks - 1:
            self._step(k, q_corr, rate_b, on_pad)
        return self.snap

    # ---- v0.8.1: the same filter on the GNS board (sim.glink_proto.GnsNode), driven by the NAV's glink STATE frame ----
    def gns_step(self, k: int, acc_sum: Sequence[float], n: int, q_corr: Sequence[float], rate_b: Sequence[float],
                 on_pad: bool) -> GpsSnapshot:
        """The 50 Hz STEP of NAV tick k (k mod 5 == 4) from a STATE frame: ``acc_sum`` = the body specific force summed
        over ``n`` NAV ticks (the frame's sum, scaled by the GNS to the ticks since its previous frame after a lost
        frame), the corrected attitude and the bias-corrected rate of tick k, the NAV's pad flag.  Identical to the
        j = 4 work of ``tick`` (the v0.8 NAV) with the sum delivered instead of accumulated.  Returns the latched
        snapshot."""
        self.k = k
        self.acc = [float(acc_sum[0]), float(acc_sum[1]), float(acc_sum[2])]
        self.n_acc = int(n)
        self._step(k, q_corr, rate_b, on_pad)
        return self.snap

    def gns_fuse(self, k: int) -> bool:
        """The j = 0 work of ``tick``: fuse the newest pending epoch (older ones of the cycle dropped), with k = the
        NAV tick it stands for (the STATE frame's tick + 1: the GNS receives the frame in the NAV's next tick).
        Returns True when an epoch was fused or rejected (i.e. one was pending)."""
        if not self.pending:
            return False
        if len(self.pending) > 1:
            self.stats["n_epoch_dropped"] += len(self.pending) - 1
        ep, te = self.pending[-1]
        self.pending = []
        self._fuse(k, ep, te)
        return True

    def restart_timebase(self) -> None:
        """v0.8.1 GNS: the NAV's tick clock jumped (a NAV reset: the STATE tick went backwards) or a STATE gap longer than
        the delayed-state buffer: the buffer, the pending epochs and the state are discarded - the next usable epoch
        re-initialises the estimator at the GPS position (the local frame's origin and the cumulative reset offset are
        kept: the GNS did not reset, its hold point stays meaningful); counted in ``stats['timebase_restarts']``."""
        for e in self.buf:
            e.m = -1
        self.pending = []
        self.m_now = -1
        self.acc = [0.0, 0.0, 0.0]
        self.n_acc = 0
        self.init = False
        self.k_last_used = None
        self.good_since = None
        self.k_last_epoch = None
        self.yaw_win = None
        self.stats["timebase_restarts"] = self.stats.get("timebase_restarts", 0) + 1

    def _step(self, k: int, q: Sequence[float], w_b: Sequence[float], on_pad: bool) -> None:
        gp = self.gp
        n = max(self.n_acc, 1)
        f0, f1, f2 = self.acc[0] / n, self.acc[1] / n, self.acc[2] / n
        T = n / gp.loop_hz
        self.acc = [0.0, 0.0, 0.0]
        self.n_acc = 0
        # rows 0 and 1 of R(q) (body -> level); row 2 is never needed: (R w) x (R l) = R (w x l)
        qw, qx, qy, qz = (float(c) for c in q)
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qw * qz)
        r02 = 2.0 * (qx * qz + qw * qy)
        r10 = 2.0 * (qx * qy + qw * qz)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qw * qx)
        ax, ay = self._rot(r00 * f0 + r01 * f1 + r02 * f2, r10 * f0 + r11 * f1 + r12 * f2)
        if gp.lever_comp:
            l0, l1, l2 = gp.lever_b_m
        else:
            l0 = l1 = l2 = 0.0
        w0, w1, w2 = (float(c) for c in w_b)
        c0, c1, c2 = w1 * l2 - w2 * l1, w2 * l0 - w0 * l2, w0 * l1 - w1 * l0     # w x l (body)
        lpx, lpy = self._rot(r00 * l0 + r01 * l1 + r02 * l2, r10 * l0 + r11 * l1 + r12 * l2)
        lvx, lvy = self._rot(r00 * c0 + r01 * c1 + r02 * c2, r10 * c0 + r11 * c1 + r12 * c2)
        if on_pad:
            self.v = [0.0, 0.0]
            self.b = [0.0, 0.0]
            self.u = [0.0, 0.0]
            self.pad = True
            self.yaw_win = None
            self.rej_v_hist = 0
        else:
            if self.pad:                            # take-off: v known 0, b restarts, acquisition gains
                self.v = [0.0, 0.0]
                self.b = [0.0, 0.0]
                self.u = [0.0, 0.0]
                self.acq_left = gp.acq_epochs
                self.pad = False
            for i, a_w in ((0, ax), (1, ay)):
                a = a_w - self.b[i]
                self.p[i] += self.v[i] * T + 0.5 * a * T * T
                self.v[i] += a * T
                self.u[i] += a * T
        self.m_now = k // gp.step_ticks
        e = self.buf[self.m_now % BUF_N]
        e.m = self.m_now
        e.pa = [self.p[0] + lpx, self.p[1] + lpy]
        e.va = [self.v[0] + lvx, self.v[1] + lvy]
        e.u = [self.u[0], self.u[1]]
        e.lv = [lvx, lvy]
        self._lever_now = (lpx, lpy)
        self.cycle = (self.cycle + 1) & 0xFFFF
        self._latch(k)

    def _latch(self, k: int) -> None:
        f = 0
        if self.fix_ok(k):
            f |= GF_FIX_OK
        if self.gate_ok(k):
            f |= GF_GATE_OK
        if self.head_ok():
            f |= GF_HEAD_OK
        if self.last_fix3d:
            f |= GF_FIX3D
        if self.cfg_ok:
            f |= GF_CFG_OK
        if self.init:
            f |= GF_INIT
        if self.ver_ok:
            f |= GF_VER_OK
        if self.incons():
            f |= GF_INCONS
        self.snap = GpsSnapshot(flags=f, num_sv=min(255, int(self.last_sv)), hacc_dm=quant_hacc_dm(self.last_hacc_mm)
                                if self.last_hacc_mm != 0xFFFFFFFF else 255, reset_count=self.reset_count & 0xFF,
                                off_x_cm=quant_cm(self.off[0]), off_y_cm=quant_cm(self.off[1]),
                                px_cm=quant_cm(self.p[0]), py_cm=quant_cm(self.p[1]), vx_cms=quant_cm(self.v[0]),
                                vy_cms=quant_cm(self.v[1]), cos_q15=quant_q15(self.c_psi), sin_q15=quant_q15(self.s_psi))

    def quality(self, ep: GpsEpoch) -> tuple[bool, bool]:
        """(usable in flight, good for the ON gate)."""
        gp = self.gp
        usable = ep.fix_ok and ep.gps_fix >= FIX_3D and ep.num_sv >= gp.ok_min_sv and ep.hacc_m <= gp.ok_max_hacc_m
        good = (ep.fix_ok and ep.gps_fix == FIX_3D and ep.num_sv >= gp.gate_min_sv and ep.hacc_m <= gp.gate_max_hacc_m
                and ep.sacc_mps <= gp.gate_max_sacc_mps)
        return usable, good

    def _fuse(self, k: int, ep: GpsEpoch, te_ms: int) -> None:
        gp = self.gp
        if self.k_last_epoch is not None and k - self.k_last_epoch > gp.fresh_ticks:
            self.good_since = None                  # a gap in the epochs restarts the ON gate's stability time
        self.k_last_epoch = k
        self.last_fix3d = bool(ep.fix_ok and ep.gps_fix == FIX_3D)
        self.last_sv = ep.num_sv
        self.last_hacc_mm = ep.hacc_mm
        usable, good = self.quality(ep)
        if good:
            if self.good_since is None:
                self.good_since = k
        else:
            self.good_since = None
        if not usable:
            self.stats["unusable"] += 1
            return
        if self.frame is None:
            self.frame = LocalFrame.at(ep.lat_e7, ep.lon_e7)
        zx, zy = self.frame.to_w(ep.lat_e7, ep.lon_e7)
        zvx, zvy = ep.vel_n_cms / 100.0, -ep.vel_e_cms / 100.0
        if not self.init or self.m_now < 0:
            lpx, lpy = getattr(self, "_lever_now", (0.0, 0.0))
            self.p = [zx - lpx, zy - lpy]
            self.v = [0.0, 0.0] if self.pad else [zvx, zvy]
            self.b = [0.0, 0.0]
            for e in self.buf:
                e.m = -1
            self.init = True
            self.acq_left = gp.acq_epochs
            self.k_last_used = k
            return
        # delayed state
        age_ms = _i32(gp.ms_per_tick * (gp.step_ticks * self.m_now + gp.step_ticks - 1) - te_ms)
        step_ms = gp.ms_per_tick * gp.step_ticks
        a = (age_ms + step_ms // 2) // step_ms
        if a < 0:
            a = 0
        if a >= BUF_N:
            self.stats["dropped_old"] += 1
            return
        s_e = self.m_now - a
        ent = self.buf[s_e % BUF_N]
        if ent.m != s_e:
            self.stats["dropped_stale"] += 1
            return
        age = a * gp.t_step
        yp = [zx - ent.pa[0], zy - ent.pa[1]]
        yv = [zvx - ent.va[0], zvy - ent.va[1]]
        self.last_innov = (yp[0], yp[1], yv[0], yv[1])
        G = self.gains.acq if self.acq_left > 0 else self.gains.steady
        acc_p = yp[0] * yp[0] + yp[1] * yp[1] <= gp.gate_p_m * gp.gate_p_m
        gv2 = gp.gate_sigma * gp.gate_sigma * self.gains.s_vv
        acc_v = [yv[0] * yv[0] <= gv2, yv[1] * yv[1] <= gv2]
        used = False
        if self.pad:
            acc_v = [False, False]                  # pad: velocity is known zero, position only
        if not acc_p:
            self.stats["rej_p"] += 1
            self.rej_p_run += 1
        else:
            self.rej_p_run = 0
        if not self.pad:
            rej_v = not (acc_v[0] and acc_v[1])
            if rej_v:
                self.stats["rej_v"] += 1
                self.rej_v_run += 1
            else:
                self.rej_v_run = 0
            self.rej_v_hist = ((self.rej_v_hist << 1) | int(rej_v)) & ((1 << INCONS_EPOCHS) - 1)
        # position reset (persistent disagreement, velocity consistent)
        if (not acc_p and self.rej_p_run >= gp.reset_after_epochs and (self.pad or (acc_v[0] and acc_v[1]))):
            self._shift(yp[0], yp[1], s_e)
            self.stats["pos_resets"] += 1
            self.rej_p_run = 0
            self.acq_left = gp.acq_epochs
            used = True
        # velocity reset (gating death spiral)
        if not self.pad and gp.vel_reset_epochs > 0 and self.rej_v_run >= gp.vel_reset_epochs:
            for i in range(2):
                self.v[i] += yv[i]
                self.p[i] += yv[i] * age
                self.b[i] = 0.0
            for s in range(self.m_now - gp.n_corr + 1, self.m_now + 1):
                en = self.buf[s % BUF_N]
                if en.m == s and s >= s_e:
                    ae = (s - s_e) * gp.t_step
                    for i in range(2):
                        en.va[i] += yv[i]
                        en.pa[i] += yv[i] * ae
            self.reset_count += 1                   # counted (FCU plausibility) but NOT in the offset: the position
            self.stats["vel_resets"] += 1           # change is the correction of a missed motion, not a GPS jump
            self.rej_v_run = 0
            self.acq_left = gp.acq_epochs
            acc_v = [False, False]
            used = True
        # fixed-gain update per axis
        for i in range(2):
            up, uv = acc_p, acc_v[i]
            if up and uv:
                K = G.k_pv
                dx = [K[r][0] * yp[i] + K[r][1] * yv[i] for r in range(3)]
            elif up:
                dx = [G.k_p[r] * yp[i] for r in range(3)]
            elif uv:
                dx = [G.k_v[r] * yv[i] for r in range(3)]
            else:
                continue
            if self.pad:
                dx = [dx[0], 0.0, 0.0]
            self._apply(i, dx, age, s_e)
            used = True
        if used:
            self.k_last_used = k
            self.stats["fused"] += 1
            if self.acq_left > 0 and not self.pad:
                self.acq_left -= 1
        # yaw (information-weighted)
        if gp.yaw_correction and not self.pad and acc_v[0] and acc_v[1]:
            gx, gy = zvx - ent.lv[0], zvy - ent.lv[1]
            if self.yaw_win is None:
                self.yaw_win = (s_e, gx, gy, ent.u[0], ent.u[1])
            elif (s_e - self.yaw_win[0]) * gp.t_step >= gp.yaw_window_s - 1e-9:
                m0, g0x, g0y, u0x, u0y = self.yaw_win
                dux, duy = ent.u[0] - u0x, ent.u[1] - u0y
                dgx, dgy = gx - g0x, gy - g0y
                nu2 = dux * dux + duy * duy
                # the heading variance grows over EVERY closed window, used or not (physics-review fix, medium 6: it
                # grew only on used windows - most of a calm hover - so HEAD_OK stayed set while the gyro drifted)
                self.p_psi += math.radians(1.0) ** 2 * gp.yaw_q_deg2_per_s * (s_e - m0) * gp.t_step
                if nu2 >= gp.yaw_min_dv_mps ** 2 and 4.0 * (dgx * dgx + dgy * dgy) > nu2:
                    ang = math.atan2(dux * dgy - duy * dgx, dux * dgx + duy * dgy)
                    r = gp.yaw_sigma_dv_mps * gp.yaw_sigma_dv_mps / nu2 + gp.yaw_sigma_rel * gp.yaw_sigma_rel
                    kk = self.p_psi / (self.p_psi + r)
                    self.yaw_corr += kk * ang
                    self.p_psi *= (1.0 - kk)
                    self._set_psi()
                    self.yaw_used += 1
                self.yaw_win = (s_e, gx, gy, ent.u[0], ent.u[1])

    def _apply(self, i: int, dx: list, age: float, s_e: int) -> None:
        """dx at the epoch, propagated to now (Phi(age)) and to the newest N_CORR buffer entries s >= s_e."""
        self.p[i] += dx[0] + age * dx[1] - 0.5 * age * age * dx[2]
        self.v[i] += dx[1] - age * dx[2]
        self.b[i] += dx[2]
        for s in range(self.m_now - self.gp.n_corr + 1, self.m_now + 1):
            en = self.buf[s % BUF_N]
            if en.m == s and s >= s_e:
                ae = (s - s_e) * self.gp.t_step
                en.pa[i] += dx[0] + ae * dx[1] - 0.5 * ae * ae * dx[2]
                en.va[i] += dx[1] - ae * dx[2]

    def _shift(self, dx: float, dy: float, s_e: int) -> None:
        """Position reset by (dx, dy): the state, the newest buffer entries and the cumulative offset."""
        self.p[0] += dx
        self.p[1] += dy
        for s in range(self.m_now - self.gp.n_corr + 1, self.m_now + 1):
            en = self.buf[s % BUF_N]
            if en.m == s and s >= s_e:
                en.pa[0] += dx
                en.pa[1] += dy
        self.off[0] += dx
        self.off[1] += dy
        self.reset_count += 1


class NavGps:
    """The NAV's whole GPS side per tick: UART bytes -> parser -> configurator / epoch assembler + iTOW clock -> filter.
    ``tick`` returns (actions for the UART, snapshot).  The 6-DOF may instead feed decoded epochs to ``NavGpsFilter``
    with ``ItowClock`` itself (same numbers)."""

    def __init__(self, gp: NavGpsParams, configure: bool = True):
        self.gp = gp
        self.parser = UbxParser()
        self.asm = EpochAssembler()
        self.clock = ItowClock(gp.ms_per_tick, gp.latency_ms)
        self.filter = NavGpsFilter(gp)
        self.cfg = GpsConfigurator(gp.loop_hz, gp.rate_hz) if configure else None
        if not configure:
            self.filter.cfg_ok = True

    def tick(self, k: int, rx: bytes, accel_b, q_corr, rate_b, on_pad: bool) -> tuple[list, GpsSnapshot]:
        msgs = []
        eps = []
        for m in self.parser.feed(rx):
            d = decode_msg(m)
            if d is None:
                continue
            msgs.append(d)
            ep = self.asm.add(d, k)
            if ep is not None:
                eps.append((ep, self.clock.update(ep.itow_ms, k)))
        acts = []
        if self.cfg is not None:
            acts = self.cfg.tick(msgs)
            self.filter.cfg_ok = self.cfg.cfg_ok
            self.filter.ver_ok = self.cfg.ver_ok
        snap = self.filter.tick(k, accel_b, q_corr, rate_b, on_pad, eps)
        return acts, snap


# =========================================================================== #
# test vectors for the C port (sim/out/gps_vectors.json)
# =========================================================================== #
def _msg_entry(m: UbxMsg) -> dict:
    d = decode_msg(m)
    out = {"cls": m.cls, "id": m.id, "length": m.length, "payload_hex": m.payload.hex(), "truncated": m.truncated}
    if isinstance(d, (NavPosllh, NavVelned, NavSol)):
        out["fields"] = dict(vars(d))
    elif isinstance(d, tuple):
        out["ack"] = list(d)
    elif isinstance(d, MonVer):
        out["mon_ver"] = {"sw": d.sw, "hw": d.hw, "module": d.module}
    return out


def _stream_vector(name: str, data: bytes, note: str = "") -> dict:
    ps = UbxParser()
    msgs = ps.feed(data)
    one = UbxParser()
    msgs1 = [x for b in data for x in [one.feed_byte(b)] if x is not None]
    if msgs1 != msgs or one.stats != ps.stats:  # pragma: no cover - self-check
        raise AssertionError(name)
    return {"name": name, "note": note, "hex": data.hex(), "expected": [_msg_entry(m) for m in msgs],
            "stats": dict(ps.stats), "state_after": ps.state}


def make_test_vectors() -> dict:
    """Deterministic vectors: checksums, every message's fields <-> bytes, the configuration frames, parser streams
    (good, bad checksum, partial, garbage, B5 B5 62, corrupted length, truncated MON-VER, back to back), the epoch
    assembler (orders, a missing message, a duplicate), the iTOW clock (jitter, leak, re-learn) and the local frame."""
    import random
    rng = random.Random(20261021)
    pos = NavPosllh(345600200, 1269780123, 375665432, 45678, 41234, 2500, 3900)
    vel = NavVelned(345600200, 12, -34, 5, 37, 36, 28743210, 30, 4500000)
    sol = NavSol(345600200, -12345, 2400, 3, 0x0D, 312345678, 98765432, 345678901, 350, 12, -34, 5, 30, 150, 0, 9, 0)
    frames = {"posllh": ubx_frame(CLS_NAV, ID_NAV_POSLLH, pos.pack()),
              "velned": ubx_frame(CLS_NAV, ID_NAV_VELNED, vel.pack()),
              "sol": ubx_frame(CLS_NAV, ID_NAV_SOL, sol.pack()),
              "ack_prt": ubx_frame(CLS_ACK, ID_ACK_ACK, bytes([CLS_CFG, ID_CFG_PRT])),
              "nak_nav5": ubx_frame(CLS_ACK, ID_ACK_NAK, bytes([CLS_CFG, ID_CFG_NAV5])),
              "mon_ver_neo6": ubx_frame(CLS_MON, ID_MON_VER, mon_ver_payload("7.03 (45969)", HW_UBLOX6)),
              "mon_ver_m8_ext": ubx_frame(CLS_MON, ID_MON_VER, mon_ver_payload("ROM CORE 3.01 (107888)", HW_UBLOX8,
                                                                               ["FWVER=SPG 3.01", "PROTVER=18.00"])),
              "mon_ver_poll": mon_ver_poll()}
    msgs = [{"name": k, "hex": v.hex(), **_msg_entry(UbxParser().feed(v)[0])} for k, v in frames.items()]
    cfgs = [{"name": "CFG-PRT 38400", "hex": cfg_prt_uart1(38400).hex()},
            {"name": "CFG-PRT 9600 (not used)", "hex": cfg_prt_uart1(9600).hex()}]
    cfgs += [{"name": n, "hex": f.hex()} for n, f in config_sequence(5.0)]
    cfgs.append({"name": "MON-VER poll", "hex": mon_ver_poll().hex()})
    streams = []
    ep_b = frames["posllh"] + frames["sol"] + frames["velned"]
    streams.append(_stream_vector("epoch_psv", ep_b, "one NEO-6M epoch: POSLLH, SOL, VELNED"))
    streams.append(_stream_vector("garbage_and_nmea", b"$GPGGA,123519,4807.038,N*47\r\n\x00\xff\xb5" + ep_b,
                                  "NMEA text, junk and a lone B5 before an epoch"))
    streams.append(_stream_vector("b5_b5_62", b"\xb5" + frames["ack_prt"], "B5 B5 62: the second B5 starts the frame"))
    for j in (2, 3, 6, 20, len(frames["velned"]) - 2, len(frames["velned"]) - 1):
        bad = bytearray(frames["velned"])
        bad[j] ^= 0x10
        streams.append(_stream_vector(f"bitflip_velned_b{j}", bytes(bad) + frames["ack_prt"],
                                      f"byte {j} flipped -> bad_ck (or bad_len / resync), the ACK behind it survives"))
    bad = bytearray(frames["posllh"])
    bad[4], bad[5] = 0x10, 0x02                     # length 528 > UBX_LEN_LIMIT
    streams.append(_stream_vector("bad_length", bytes(bad) + frames["sol"], "length 528: bad_len, resync"))
    streams.append(_stream_vector("partial_then_good", frames["posllh"][:17] + frames["velned"],
                                  "a frame cut after 17 bytes, then a complete one: the cut one swallows bytes of "
                                  "the next until its length is reached and fails its checksum; the complete one may "
                                  "be lost (no replay) - the expected result is normative"))
    streams.append(_stream_vector("truncated_mon_ver", frames["mon_ver_m8_ext"], "MON-VER 130 B: stored 64, truncated"))
    streams.append(_stream_vector("partial_end", frames["sol"][:30], "stream ends inside a frame: nothing, state kept"))
    streams.append(_stream_vector("back_to_back", b"".join(frames.values()), "every frame back to back"))
    noise = bytes(rng.randrange(256) for _ in range(300))
    streams.append(_stream_vector("random_noise", noise, "300 random bytes"))
    streams.append(_stream_vector("noise_epoch_noise", noise[:40] + ep_b + noise[40:80] + ep_b,
                                  "an epoch between noise"))
    # epoch assembler
    asm_cases = []

    def run_asm(name: str, items: list[tuple], note: str = "") -> None:
        a = EpochAssembler()
        out = []
        for tick, m in items:
            e = a.add(m, tick)
            if e is not None:
                out.append(dict(vars(e)))
        asm_cases.append({"name": name, "note": note,
                          "input": [{"tick": t, "kind": type(m).__name__, "itow_ms": m.itow_ms} for t, m in items],
                          "epochs": out, "stats": dict(a.stats)})

    def ep_msgs(itow: int) -> tuple:
        return (replace(pos, itow_ms=itow), replace(sol, itow_ms=itow), replace(vel, itow_ms=itow))

    p0, s0, v0 = ep_msgs(1000)
    p1, s1, v1 = ep_msgs(1200)
    run_asm("psv", [(10, p0), (12, s0), (14, v0)])
    run_asm("vsp", [(10, v0), (12, s0), (14, p0)], "any order")
    run_asm("missing_sol", [(10, p0), (14, v0), (60, p1), (62, s1), (64, v1)], "1000 incomplete, 1200 complete")
    run_asm("duplicate", [(10, p0), (11, p0), (12, s0), (14, v0)], "a repeated POSLLH")
    run_asm("old_itow", [(10, p1), (12, s0), (14, v1), (15, s1)], "a stale message breaks the open epoch")
    # iTOW clock
    clk = ItowClock(4, 220)
    clock_cases = []
    tick = 1000
    for n, (itow, jit) in enumerate([(345600000, 0), (345600200, 2), (345600400, 0), (345600600, 5), (345600800, 1),
                                     (345601000, 0), (345601200, 0), (345601400, 3), (345601600, 0),
                                     (345601800, 0)] + [(1000, 0), (1200, 1)]):
        tick_c = tick + 50 * n + jit if itow > 2000 else 9000 + jit
        te = clk.update(itow, tick_c)
        clock_cases.append({"itow_ms": itow, "tick": tick_c, "t_e_ms": te, "offset": clk.offset,
                            "relearned": clk.relearned})
    # local frame
    lf = LocalFrame.at(375665432, 1269780123)
    lframe = {"lat0_e7": lf.lat0_e7, "lon0_e7": lf.lon0_e7, "k_n": lf.k_n, "k_e": lf.k_e,
              "points": [{"lat_e7": la, "lon_e7": lo, "x_m": lf.to_w(la, lo)[0], "y_m": lf.to_w(la, lo)[1]}
                         for la, lo in ((375665432, 1269780123), (375666332, 1269780123), (375665432, 1269781256),
                                        (375655432, 1269770123), (375665432 + 135000, 1269780123 - 170000))]}
    return {"_doc": ("u-blox UBX vectors (v0.8 GPS) generated by sim/gps_nav.py make_test_vectors(): messages (fields <-> "
                     "bytes), configuration frames (exact bytes the NAV sends at boot), parser streams (every message the "
                     "byte-wise parser returns and its counters - normative for gps_ubx_parse_byte), the epoch assembler "
                     "(iTOW matching), the iTOW clock (t_e = iTOW + offset - latency, offset = leaky minimum) and the "
                     "local frame (x north, y west; floats: compare within 1e-6 relative, float32)."),
            "constants": {"SYNC": [UBX_SYNC1, UBX_SYNC2], "UBX_MAX_PAYLOAD": UBX_MAX_PAYLOAD, "UBX_LEN_LIMIT": UBX_LEN_LIMIT,
                          "LEN_POSLLH": LEN_POSLLH, "LEN_VELNED": LEN_VELNED, "LEN_SOL": LEN_SOL,
                          "ITOW_LEAK_MS": ITOW_LEAK_MS, "ITOW_JUMP_MS": ITOW_JUMP_MS, "BUF_N": BUF_N,
                          "flags": {v: k for k, v in GF_NAMES.items()}},
            "checksum_check": {"hex": "0102", "ck": list(ubx_checksum(b"\x01\x02"))},
            "messages": msgs, "config_frames": cfgs, "streams": streams, "assembler": asm_cases,
            "itow_clock": clock_cases, "local_frame": lframe}


def write_test_vectors(path: str = VECTORS_PATH) -> dict:
    v = make_test_vectors()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(v, f, indent=1)
        f.write("\n")
    return v


if __name__ == "__main__":  # pragma: no cover
    _v = write_test_vectors()
    sys.stdout.write(f"wrote {os.path.relpath(VECTORS_PATH, os.path.join(_HERE, '..'))}: {len(_v['messages'])} messages, "
                     f"{len(_v['streams'])} streams, {len(_v['assembler'])} assembler cases\n")
