/* nav_simbench.h - cycle measurement of the NAV's real 250 Hz loop in the simavr ATmega128 simulator
 * (PlatformIO envs "nav_simbench" / "nav_cosim", -D FC_SIMBENCH; v0.6.3).  tools/run_nav_simbench.py builds,
 * runs and judges it (host_test/build/nav_simbench_report.json).
 *
 * The NAV firmware is unchanged except for its sensor SOURCES, which are the FCU simbench's (src/simbench.c of
 * v0.6.2, same trajectory and pad timing): the raw MPU-6050 burst (still converted by mpu6050_convert) is prepared
 * after the previous tick's measurement - still on the pad (small gyro offsets, 1 g) while the vehicle state is
 * SAFE / ARMED, oscillating rates (1.3 / 0.9 / 0.5 Hz, 0.6 / 0.5 / 0.3 rad/s) and 15 m/s^2 of thrust otherwise;
 * the barometer bus transfers are skipped while its compensation and altitude (powf) still run in their
 * phases (datasheet PROM, D1 / D2 around the datasheet example).  No I2C slave: the Timer0 lead hook is not set,
 * the TWI interrupts and the I2C bus time are added analytically by run_nav_simbench.py (avr-objdump of the ELF).
 * The console is the simbench-only USART0 (hal/uart_log.c, 57600 8N1); the flight NAV has no log port.
 *
 * ---- single-chip bench (nav_simbench: FC_SIMBENCH without FC_COSIM) ----
 * nlink_spi's single-chip bypass (hal/nlink_spi.h): nlink_spi_exchange copies the frame instead of clocking the
 * SPI (so the exchange section is NOT the wire time - nav_cosim measures that), and a synthetic FCU, run after the
 * NAV's background, answers it:
 *   - takes the NAV's frame (nlink_spi_take_tx), checks it with nl_decode_nav (sync, TYPE, CRC) and SEQ
 *     continuity, and the contract's NAV rules (core/nlink_proto.h): BOOT in exactly the frames built before the
 *     first valid FCU frame reached the NAV (2: the first exchange returns zeros, the second the first reply),
 *     pad / flight mode two exchanges behind the FCU's state (on a tick without BARO_NEW a pad-mode frame has
 *     h = v_z = 0 exactly - the altitude filter is held - and a flight-mode frame has the predicted values), every
 *     frame of a flying state with IMU_OK and EST_VALID (the FCU's imu_ok rule);
 *   - its state is the supervisor's in miniature: SAFE, ARMED after NAV_SB_ARM_S once the frame says ALIGNED,
 *     IMU_OK and BARO_OK (the ARM gate's NAV part), POWERED after NAV_SB_LAUNCH_S;
 *   - replies with nl_encode_fcu (SEQ = the frame it accepted, its state, params_tag = FC_PARAMS_TAG) through
 *     nlink_spi_inject_reply: the next exchange returns it, one FCU tick old as on the wire;
 *   - fault injection: from NAV_SB_CORRUPT_S every NAV_SB_CORRUPT_EVERY-th reply has a CRC byte flipped (the NAV
 *     must keep its last valid state: checked through the pad / flight signature, and its bad-reply counter must
 *     equal the corruptions + 1 for the zeros of the first exchange), and at NAV_SB_TAG_S one valid reply carries
 *     another params_tag (exactly one NAV frame must then carry NAV_RC_TAG_MISMATCH).
 * The vehicle state that drives the synthetic IMU is the synthetic FCU's.
 *
 * ---- co-simulation build (nav_cosim: FC_SIMBENCH + FC_COSIM) ----
 * The synthetic sensors and the report lines stay, the synthetic FCU is compiled out and the REAL SPI master runs
 * (hal/nlink_spi.c, the flight code): wired by a simavr harness to an fcu_cosim FCU (nlink_spi.h FC_COSIM notes),
 * or run alone (run_nav_simbench.py does: simavr's SPI master completes every byte at the programmed clock, MISO
 * reads 0x00, so no FCU frame is ever valid and the NAV stays in pad mode - this gives the real exchange time).  The
 * synthetic IMU follows the NAV's own view of the state (the last valid FCU frame).  Tick marker for a harness:
 *   PORTA (unused on the NAV board) = (tick << 1) | 1 when the main loop starts the tick (after the Timer2 ISR and
 *          the wake-up), = tick << 1 right after nlink_spi_exchange returned (SS is high: the frame is delivered).
 *          DDRA = 0xFF from boot.  Two OUT instructions per tick, this build only.
 *
 * Report lines (USART0, every number hex "0x..", one line < 216 bytes), both builds unless marked:
 *   "SB BOOT rc= sr= xb= gc= sh= bd=" once: reset_cause sent in the frames (with the NAV_RC_* bits), stack reserve,
 *          NL_XFER_BYTES, NLINK_BYTE_GAP_CYCLES, NLINK_SPI_HZ / 1000 (kHz), 1 = cosim build;
 *   "SB t= st= pd= cm= cp= cf= fr= fp= ff= fn= jx= tm= ov= sf= bg= ox= s=<6 sections>"   1 Hz:
 *          the NAV's view of the FCU state, pad mode (1) this tick; time-critical part = tick -> frame delivered
 *          (SS high): window max (cm), pad / flight-mode max (cp / cf, cumulative); frame ready after the Timer2
 *          ISR's own Timer1 stamp (fr window max, fp / ff pad / flight max, fn min, jx = the largest increase from
 *          one tick to the next - the FCU's tick spacing is the tick plus that - all cumulative; jx two's complement);
 *          incl. the background (tm); missed ticks; never-touched stack; longest background job (barometer math step);
 *          the simbench's own work; section maxima of the window: sensors | estimator + tilt + altitude | frame build +
 *          nl_encode_nav | nlink_spi_exchange | FCU reply decode + LED | background (us, Timer1);
 *   "SN t= st= fv= fb= ex= rs= lm= rc= rd="   0.5 s later: valid / refused FCU replies at the NAV, exchanges (driver),
 *          exchanges its W1 watchdog closed (fw09; nlink_spi_stats()->resyncs, 0 in the bench builds: no watchdog),
 *          the loop_us_max and reset_cause fields of the frame sent this tick, report lines dropped (console busy);
 *   "SF t= nv= nx= sg= pm= pc= ef= bc= tg= cr= br= fb= bl= al= ev= ar= pw= ss="   single-chip only, 0.25 s after
 *          SN: the synthetic FCU's checks - NAV frames valid / refused, SEQ gaps, pad / flight signature mismatches /
 *          frames checked, flying frames without IMU_OK + EST_VALID, BOOT frames, TAG_MISMATCH frames, corrupted
 *          replies injected, replies returned that the NAV had to refuse (corrupted + the first exchange's zeros) and
 *          the NAV's own refused count at the same tick, frames without BARO_OK after the first with it (late baro),
 *          ticks of the first ALIGNED / EST_VALID frame, of ARMED and POWERED, its state;
 *   "SB END" before the CPU sleeps with interrupts off (simavr exits), after NAV_SB_SECONDS.
 * v0.7.1: the SF line adds "eg=" (flight-mode frames with nav.h NAV_F_EST_GATE, VTOL: the estimator correction ran) and
 *   "ep=" (pad-mode frames with it: must stay 0); the SB line (VTOL build) "ce=" (the latest compute end - frame encoded,
 *   before the wait to the push floor nav.h NAV_PUSH_MIN_US_VTOL - after the Timer2 ISR's stamp, us, cumulative maximum)
 *   and "lp=" (ticks whose compute ended past the floor and pushed at once, cumulative), before "s=".
 *
 * v0.8.1 VTOL build (the four-chip set: USART1 = the glink to the GNS, hal/glink_uart.h; the v0.8 GPS side, its synthetic
 * receiver (a NEO-6M model) and the lines SG / SI / SH / SJ of v0.8 moved to the GNS bench, src/gns/gns_simbench.h):
 *   - the synthetic FCU decodes the nlink v4 frame (nl_decode_nav4), feeds its slot into the FCU's own slot receiver
 *     (core/nlink_proto.h nl_gns_rx: sets, broken cycles, TILT parts) and checks every TILT part's timing: the synthetic GNS
 *     writes its cycle m into the TILT part's y component, which must ride in the frame of NAV tick 5 m + 4 + L
 *     (GLINK_LATENCY_TICKS); it replies with nl_encode_fcu4 (pflags: HOLD | MOTOR_ON while POWERED, kt_q 1.0);
 *   - nav_simbench (single chip, not GLINK_WIRED): a synthetic GNS on the glink - it takes the STATE frames the NAV queued
 *     (copies, hal/glink_uart.c), checks them (sync / TYPE / CRC by its decoder, k mod 5 == 4, SEQ = k / 5, n = 5 ticks
 *     after the first, the FCU fields of the synthetic FCU's reply) and answers each with a RESULT whose first byte
 *     completes NAV_SB_GNS_CYCLE_US after the STATE's last byte on the wire (60 bytes x 40 us after it was queued) + one
 *     byte time, on the Timer3 bench wire into RXD1 (byte-accurate 8N1 timing; the RX ISR's ring code).  Faults, by cycle:
 *     NAV_SB_GL_MISS_M no RESULT, NAV_SB_GL_LATE_M a RESULT one tick too late for its frame build (must be dropped),
 *     NAV_SB_GL_CRC_M a RESULT with a broken CRC, NAV_SB_GL_SEQ_M a RESULT with another SEQ - four cycles that must be
 *     missed (SLOT_EMPTY: no TILT part for them at the FCU), nothing else;
 *   - nav_simbench_glink (FC_SIMBENCH + GLINK_WIRED; never flash): the same synthetic FCU, the REAL glink USART1 both ways
 *     and no synthetic GNS - a simavr harness wires a gns_cosim GNS to it (the glink latency measurement); the tick marker
 *     on PORTA as nav_cosim's (below).
 *   Report lines (VTOL): "SW" (as v0.8: tick -> mandatory work), "SG t= ss= rt= la= ms= rs= df= dc= dt= dw= ds= ux= ue= tr="
 *   (the NAV's glink side: STATE frames sent, RESULT frames taken, slot latches / misses, results into the ring, decoder
 *   frames / bad CRC / bad TYPE / wrong direction / skipped bytes, USART1 RX overflow / errors, TX frames refused),
 *   "SH t= sa= sb= pb= pn= pt= gb= ra= rm= mn= mx=" (the STATE's write after the tick's Timer2 stamp - max / min; the
 *   frame build after the stamp - max; the pre-build decode - longest, most bytes, frames taken there; the glink tick
 *   section max; the RESULT completion after its sample tick's stamp - max / min; the margin from a RESULT's completion to
 *   the build of frame 5 m + 4 + L - min / max, us, signed), "SJ t= gs= gx= gr= gk= gm= xs= xb= xt= ti= tb= tm=" (the
 *   synthetic GNS: STATE frames decoded / with a wrong field, RESULTs sent, faults injected; the synthetic FCU's slot
 *   receiver: sets, broken cycles, TILT parts, TILT parts at the right tick, at a wrong tick, cycles without one).
 *
 * Invariant: compiled to nothing unless FC_SIMBENCH is defined; the synthetic sources replace only inputs, never
 * a line of the measured NAV code (the co-simulation's tick marker adds two port writes per tick to its own build).
 */
#ifndef FC_NAV_SIMBENCH_H
#define FC_NAV_SIMBENCH_H

#if defined(FC_COSIM) && !defined(FC_SIMBENCH)
#error "FC_COSIM is a variant of the simbench: build it with -D FC_SIMBENCH -D FC_COSIM (env nav_cosim)"
#endif

/* v0.9 (fw09 WP-S): env nav_cosim_twi = nav_cosim + NAV_SB_TWI - the REAL MPU-6050 / barometer (BMP280; MS5611 as the build option) drivers on the TWI (the
 * four-core co-simulation's tools/cosim/twi_models.c answers them from its closed-loop plant): the synthetic sources below
 * are not used (their functions stay, empty), the flight build's sensor init and its Timer0 IMU-lead hook run, and the
 * tick marker and the report lines stay.  nav_main.c selects its sensor sources with NAV_SB_SYNTH_SENSORS (1 = the
 * synthetic sources of the bench / open-loop co-simulation builds, 0 = the real drivers: flight and nav_cosim_twi). */
#if defined(FC_SIMBENCH) && !defined(NAV_SB_TWI)
#define NAV_SB_SYNTH_SENSORS 1
#else
#define NAV_SB_SYNTH_SENSORS 0
#endif
#if defined(NAV_SB_TWI) && !(defined(FC_SIMBENCH) && defined(FC_COSIM))
#error "NAV_SB_TWI is a co-simulation build: -D FC_SIMBENCH -D FC_COSIM -D NAV_SB_TWI (env nav_cosim_twi)"
#endif

#if defined(FC_SIMBENCH) && (defined(FC_COSIM) || defined(GLINK_WIRED))
#include <avr/io.h>
#define NAV_SB_COSIM_TICK(t) (PORTA = (uint8_t)((uint8_t)((uint8_t)(t) << 1) | 1u))
#define NAV_SB_COSIM_SENT(t) (PORTA = (uint8_t)((uint8_t)(t) << 1))
#else
#define NAV_SB_COSIM_TICK(t) ((void)0)
#define NAV_SB_COSIM_SENT(t) ((void)0)
#endif

#ifdef FC_SIMBENCH
#include <stdbool.h>
#include <stdint.h>

#include "fc_mode.h"
#include "hal/baro.h"
#if FC_VTOL
#include "core/glink_proto.h"
#endif

#if FC_VTOL
#define NAV_SB_SECTIONS 8            /* v0.8 VTOL: marks 0..7, sections 1..7 (see the SB line; v0.8.1 section 5 = glink) */
#else
#define NAV_SB_SECTIONS 7            /* marks 0..6: sections 1..6 (see the SB line) */
#endif
#ifndef NAV_SB_SECONDS
#ifdef FC_SIMBENCH_SECONDS
#define NAV_SB_SECONDS FC_SIMBENCH_SECONDS
#elif defined(FC_COSIM)
#define NAV_SB_SECONDS 15            /* = fcu_cosim's FC_SIMBENCH_SECONDS / the COM's COM_SB_SECONDS */
#else
#define NAV_SB_SECONDS 12
#endif
#endif
#define NAV_SB_ARM_S 3u              /* single-chip: the synthetic FCU arms after this many seconds (once ALIGNED) ... */
#define NAV_SB_LAUNCH_S 4u           /* ... and goes POWERED after this many */
#define NAV_SB_CORRUPT_S 6u          /* single-chip: from this second ... */
#define NAV_SB_CORRUPT_EVERY 50u     /* ... every 50th FCU reply has a CRC byte flipped */
#define NAV_SB_TAG_S 10u             /* single-chip: one valid reply with another params_tag at this second */
/* v0.7.1 VTOL build (fc_mode.h FC_VTOL): the worst-case estimator / altitude-filter path.  The flight-mode specific force
 * is a hover's (|f| ~ g: the gated Mahony correction runs every tick - the 15 m/s^2 of the ballistic bench would keep its
 * gate closed), the synthetic FCU's replies carry the hover throttle in thr_q while POWERED, and the altitude filter
 * flies a non-zero baro thrust compensation (NAV_SB_COMP_M, a simbench input: the flight build's params.json value 0.0 =
 * uncalibrated skips the compensation's cycles, a calibrated vehicle runs them) */
#define NAV_SB_COMP_M 0.5f           /* m at hover thrust (ASSUMED, timing only) */
/* round(FC_HOVER_THROTTLE x 250): fw09 integration (G1, agent B's request) - the hover throttle of the build's vehicle
 * configuration (params_gen.h; battery 0.7301 -> 183, tether 168, hybrid 175; the rocket set 0.9160 -> 229 as before) */
#define NAV_SB_THR_Q_HOVER ((uint8_t)(FC_HOVER_THROTTLE * 250.0f + 0.5f))
/* v0.8.1: the synthetic GNS (single chip).  Its answer time: the GNS's STATE-complete -> RESULT-queued time measured by
 * tools/run_gns_simbench.py (gns_simbench / gns_simbench_demo, worst case incl. a poll in progress and the last byte's RX
 * interrupt: 3820.2 / 3859.2 us, 2026-10-22), rounded up to 10 us (the runner checks this value against that report) - a
 * bench input, the two-core harness measures the real path */
#ifndef NAV_SB_GNS_CYCLE_US
#define NAV_SB_GNS_CYCLE_US 3860u
#endif
#define NAV_SB_GL_MISS_M 400u        /* cycles (NAV tick 5 m + 4) with a fault: no RESULT ... */
#define NAV_SB_GL_LATE_M 450u        /* ... one NAV tick too late for its frame build ... */
#define NAV_SB_GL_CRC_M 500u         /* ... a broken CRC ... */
#define NAV_SB_GL_SEQ_M 550u         /* ... another SEQ */

/* one tick's figures, filled by nav_main.c after the background */
typedef struct {
    uint32_t tick;
    uint8_t  fcu_state;              /* the NAV's view: state of the last valid FCU frame (SAFE before the first) */
    bool     pad;                    /* pad mode this tick (altitude held, ground reference) */
    uint16_t crit_us;                /* tick -> frame delivered (SS high), Timer1 from the loop's t0, us */
    uint16_t ready_us;               /* the same from the Timer2 ISR's own Timer1 stamp (incl. ISR + wake-up), us */
    uint16_t total_us;               /* incl. the background, us */
    uint16_t overruns;               /* missed ticks */
    uint16_t fcu_valid, fcu_bad;     /* FCU replies accepted / refused by the NAV */
    uint16_t loop_us_max;            /* the loop_us_max field of the frame sent this tick */
    uint8_t  reset_cause;            /* the reset_cause field of the frame sent this tick */
#if FC_VTOL
    uint16_t work_us;                /* v0.8: tick -> the end of the tick's mandatory work (correction, altitude, glink, reply) */
    const gl_nav_t *gl;              /* v0.8.1: the NAV's glink side (report only) */
#endif
} nav_sb_tick_t;

void           nav_sb_init(void);            /* boot: console, [cosim: tick marker port], the SB BOOT line */
void           nav_sb_boot_line(uint8_t reset_cause);
void           nav_sb_prepare(uint32_t next_tick, uint8_t state);
const uint8_t *nav_sb_imu_raw(void);         /* synthetic MPU-6050 burst, converted by mpu6050_convert */
void           nav_sb_baro(uint32_t tick, uint8_t phase, baro_t *b);
void           nav_sb_mark(uint8_t i);
void           nav_sb_bg_step(uint16_t us);  /* one background job's duration */
uint16_t       nav_sb_ready_us(void);        /* now - the Timer2 ISR's Timer1 stamp of the current tick, us */
/* v0.7.1 VTOL build: the frame is encoded (nav_main.c push_wait, before the wait to the floor NAV_PUSH_MIN_US_VTOL): its
 * time after the Timer2 ISR's stamp ("ce=", maximum) and whether it was already past the floor ("lp=") */
void           nav_sb_compute_end(bool late);
/* after the background: [single-chip: the synthetic FCU's turn], the next tick's sensor sources, the report lines */
void           nav_sb_end_of_loop(const nav_sb_tick_t *t);
#if FC_VTOL
/* v0.8.1 glink: bytes decoded (and RESULT frames taken) in NAV tick k; the STATE frame written (ok: queued); the frame
 * build of tick k (after the pre-build decode of nb bytes that took us) */
void           nav_sb_glink_rx(uint32_t tick, uint8_t n, uint8_t taken);
void           nav_sb_glink_state(uint32_t tick, bool ok);
void           nav_sb_slot_build(uint32_t tick, uint8_t nb, uint16_t us);
#endif
#endif

#endif /* FC_NAV_SIMBENCH_H */
