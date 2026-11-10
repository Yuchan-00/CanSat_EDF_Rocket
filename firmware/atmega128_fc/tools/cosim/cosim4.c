/* cosim4.c - four-chip co-simulation of the v0.9 flight computer: four ATmega128 cores at 16 MHz in one libsimavr
 * process (PlatformIO tool-simavr), wired as on the board, with a closed-loop plant (SPEC_firmware_v09 WP-S / S1).
 * Adopted from the generated timing82/profile/cosim4/cosim4.c (= tools/cosim/cosim.c, the v0.6.3 three-chip harness,
 * + the v0.8.1 GNS core on the glink); tools/run_cosim4.py builds and runs it, analyses the event files and judges the
 * co-simulation rows of SPEC section 7.  tools/cosim/cosim.c / tools/run_cosim.py stay for the ballistic 3-core build.
 *
 *   NAV  Intel HEX of env nav_cosim_twi (closed loop: the REAL MPU-6050 / barometer drivers on the TWI, twi_models.c) or
 *        nav_cosim (open loop: synthetic sensors): the real nlink SPI MASTER (hal/nlink_spi.c), the real glink USART1,
 *        the tick marker on PORTA (src/nav/nav_simbench.h) and its report lines on USART0 (a simbench-only console)
 *   FCU  Intel HEX of env fcu_cosim* : the real nlink SPI SLAVE with its SPI_STC interrupt, the real ilink USART1 with its
 *        RX / UDRE interrupts, the tick marker on PORTA (src/simbench.h), report lines on USART0 (the OpenLog port), the
 *        PWM compare registers (the plant's inputs)
 *   COM  Intel HEX of env com_cosim* (lora_uart): the in-firmware synthetic ground feeds the radio driver, the real ilink
 *        USART0, the report struct g_cosim_report in RAM (src/com/com_cosim.h)
 *   GNS  Intel HEX of env gns_cosim*: the real glink USART0, the in-firmware synthetic GPS receiver; closed loop: the
 *        receiver reports the plant's antenna position from the harness-written RAM block g_cosim_gps (gps=,
 *        src/gns/gns_simbench.h); its figures in g_gns_cosim_report (gnsrep=, dumped as GQ records)
 *   wires  nlink: NAV PB0 (SS) -> FCU PB0, NAV SPI byte -> FCU SPI input, FCU shift register -> NAV SPI input (MISO)
 *          ilink: FCU USART1 TX -> COM USART0 RX and COM USART0 TX -> FCU USART1 RX (8N1, ILINK_BAUD)
 *          glink: NAV USART1 TX -> GNS USART0 RX and GNS USART0 TX -> NAV USART1 RX
 *   plant  plant4.c (cosim4.h): the FCU's ESC / vane compare registers -> thrust, vane moments, attitude, height,
 *          horizontal motion, ground contact -> the NAV's TWI sensor registers (twi_models.c) and the GNS's GPS block,
 *          every pl_dt_us (250 us) of global time; logged as P records.
 *
 * simavr corrections added in the adoption (SPEC S1b / S1c):
 *   - Timer0 preload (t0fix=1): simavr 1.8 restarts a timer from 0 when its clock is switched on and ignores the TCNT0
 *     written while it was stopped (hal/timer.c preloads FC_IMU_LEAD_COUNTS so Timer0's compare - the IMU lead - comes
 *     FC_IMU_LEAD_COUNTS before Timer2's).  When TCCR0 switches the clock on, the harness replays the stored TCNT0 through
 *     simavr's own TCNT0 write handler, which re-bases the running timer: the lead then exists in the simulation (the NT
 *     record's t2 - t0 measures it; simavr re-bases with tov_top = OCR0, so the lead is 31 x 64000 / 249 = 7968 cycles
 *     instead of 7936: +2.0 us, stated in the report).  Done on the NAV and the GNS (both run hal/timer.c's Timer0).
 *   - TWI byte time (twi=1 twifix=1): twi_models.c re-schedules simavr's TWI state timer from N us to N SCL periods.
 *   - Watchdog reset: a core whose own watchdog expires makes simavr 1.8 swap avr->run for its reset callback, and the
 *     avr_reset() that callback runs crashes (access violation) in the watchdog module's own reset once the firmware has
 *     enabled the watchdog - the deterministic 0xC0000005 of SPEC section 2 row 8 (fw09/spec/fw_t0: the stalled NAV's
 *     watchdog at 10.0696 s, 63 ms after its lost exchange; measured with a SIGSEGV handler: the NAV core, PC 0, right
 *     after avr->run changed).  The harness now sees avr->run change after the instruction and performs the reset
 *     itself (core_hold_reset: as simavr's avr_reset without the watchdog module's reset, released at once, MCUCSR.WDRF
 *     set; record E <core>_wdt_reset).  A SIGSEGV still flushes the event file and records "E t crash core=..".
 *
 * Time.  One global clock in CPU cycles (16 MHz): core cycle + core offset = global (the FCU's offset is 0; the NAV's and
 * the COM's are their start delays, moved again while the harness holds a core in reset).  Lockstep: the core that is
 * behind executes the next instruction (ties: FCU, then NAV, then COM - so every FCU SPDR write at or before the time
 * the NAV starts a byte is known when it starts); a no-op cycle timer every `step` cycles bounds how far a sleeping
 * core jumps ahead, so a byte crossing a wire reaches the other core at most `step` cycles late (default 16 = 1 us;
 * the SPI bytes are scheduled exactly, below).
 *
 * Interrupt response.  simavr charges an interrupt the vector's JMP and the RETI but not the chip's 4-cycle interrupt
 * response (tools/cosim/isr_cost.c measures it: 7 cycles per bare-RETI interrupt against 11 on the ATmega128).  With
 * isr_resp=4 (default) the harness adds those 4 cycles to the core's clock when an interrupt starts running, so every
 * core - its timers, its own Timer1 measurements, the nlink slave's reply-byte latency - runs at the chip's interrupt
 * cost; isr_resp=0 keeps simavr's (tools/run_cosim.py runs isr_cost both ways and checks 11 / 7).
 *
 * nlink (SPI).  simavr's SPI (measured by tools/cosim/spi_probe.c on every run): the master raises SPI_IRQ_OUTPUT with
 * its SPDR 8 x the SCK divider cycles after the SPDR write (64 at F_CPU / 8) and takes SPI_IRQ_INPUT as the received
 * byte (SPDR, SPIF); a slave takes SPI_IRQ_INPUT whatever its SS pin says, and answers it on SPI_IRQ_OUTPUT with the
 * byte it just RECEIVED (its one SPDR byte is overwritten first), never the byte it loaded.  So the harness does not
 * use the slave's output; it models the FCU's shift register as on the chip:
 *   - a byte STARTS when the NAV writes SPDR with SPE + MSTR (ts); it is the slave's only if FCU SS (PB0) is low and
 *     the FCU's SPI is on - else MISO floats (the NAV's pull-up: 0xFF) and the FCU sees nothing;
 *   - MISO = the shift register at ts: the FCU's last SPDR write before ts, or - if it wrote none since the previous
 *     byte ended - the byte received last (the echo of a late slave);
 *   - an FCU SPDR write while a byte is shifting (ts..te) is a write collision: ignored, counted (WCOL);
 *   - the byte ENDS at te = ts + 8 x the NAV's SCK divider (from its SPCR / SPSR): an FCU cycle timer delivers the
 *     NAV's byte to the FCU's SPI input exactly then (SPIF + SPI_STC), and the shift register holds the received byte;
 *     the NAV gets the MISO byte when its own SPI raises the output (simavr's master timing);
 *   - SS: the FCU's PB0 is driven by the NAV's PB0 as an EXTERNAL level (AVR_IOCTL_IOPORT_SET_EXTERNAL, else simavr
 *     re-applies the FCU's own pull-up on every PORTB write); a NAV that does not drive PB0 (reset, before
 *     nlink_spi_init) leaves the FCU's pull-up: high.  SS rising while a byte shifts aborts that byte at the slave (the
 *     chip resets its SPI logic); the shift register keeps its content (assumption, not measurable here).
 * The deadline for a reply byte is the master's SPDR write (ts); on the chip the slave's MSB must be on MISO before the
 * first SCK edge, about half an SCK period later - the model is conservative by those ~4 cycles.
 *
 * nlink faults, from fault_at_ms for fault_len_ms, counted per exchange from the first one that starts in the window
 * (n = 0, 1, ...; every nl_every-th):
 *   nl_flip   exchange n % nl_every == 0: one MOSI bit flipped at the FCU (byte (n*13+5) % 72, bit n % 8);
 *             exchange n % nl_every == nl_every / 2: one MISO bit flipped at the NAV (byte n % 10, bit (n+3) % 8)
 *   nl_drop   exchange n % nl_every == 0: MOSI byte (n*17+3) % 72 never reaches the FCU (no SCK at the slave: the NAV
 *             reads the slave's MSB 8 times)
 *   nl_extra  exchange n % nl_every == 0: a phantom byte 0xFF (an SCK glitch burst) shifts through the slave in the gap
 *             after byte (n*19+7) % 71
 *   nl_noise  every bit on MOSI and MISO flipped with probability nl_ber (xorshift PRNG, seed)
 * NAV faults:
 *   nav_hold  the NAV is held in reset from fault_at_ms to the end (fault_len_ms ignored)
 *   nav_pulse the NAV is reset at byte nav_reset_byte of the first exchange that starts after fault_at_ms, held
 *             fault_len_ms (reset pulse + the chip's reset time-out), then runs again (MCUCSR = EXTRF)
 * ilink faults (v0.6): cut (both directions), reset (the COM held in reset for fault_len_ms), babble (the COM's bytes
 * replaced by the pattern babble=<hex> at the full wire rate).  A core in reset: simavr's avr_reset() minus the watchdog
 * module's reset, which crashes once the firmware has enabled the watchdog (core_hold_reset).
 *
 * USART timing.  simavr's USART raises UART_IRQ_OUTPUT at the UDR write (the start bit); the receiving USART raises RXC
 * one byte time after the byte entered its input FIFO.  simavr 1.8 counts a parity bit in every frame (11 bit times per
 * 8N1 byte) and computes the byte time only when UBRRnL is written; wire_bits=10 (default) keeps every USART of every
 * chip at the byte time its registers configure (record W); wire_bits=11 keeps simavr's (diagnostic).
 *
 * Output (out=<file>, one record per line, cycles are global):
 *   H key=value...                  the configuration
 *   W core uart simavr_cycles cycles_used t        a USART's byte time corrected (wire_bits=10)
 *   R i t_in t_rxc byte src         COM -> FCU byte i: on the wire at t_in, RXC at the FCU at t_rxc (src c COM, b babble)
 *   D dir t byte why                an ilink byte not delivered (dir c2f / f2c; why cut / babble / rxoff)
 *   S t byte                        FCU -> COM byte (FCU USART1 UDR write = start bit)
 *   U t byte                        COM USART1 TX byte (lora_uart: the module UART, i.e. the TELEM frames)
 *   C t text                        an FCU console line (ASCII, starting with 'S')
 *   NC t text                       a NAV console line (ASCII, starting with 'S')
 *   F t0 t1 hex                     an FCU log frame on the console (0xA5 0x5A type len payload fletcher16)
 *   T tick rise fall busy=c nisr=n v17=c:n v19=c:n v30=c:n v31=c:n vo=c:n pwm=t0,..,t5 rd=hex,hex,...
 *                                   one FCU tick at the end of its time-critical part: tick marker up / down (PA0), the
 *                                   cycles spent in interrupts inside [rise, fall] (busy, nisr) split by vector (17 nlink
 *                                   SPI, 19 log UDRE, 30 ilink RX, 31 ilink UDRE, o other), the last write of each PWM
 *                                   compare register inside [rise, fall] (channels 0..5, pwm= gives their OCRnxH
 *                                   addresses: simavr shares at most 4 hooked I/O registers per core and the low bytes
 *                                   already carry the timer's own hook, so the high byte is watched - avr-gcc writes it
 *                                   first, the low byte (the commit) follows in the next instruction, 1-2 cycles later;
 *                                   0: none), the fall= reads
 *   B tick nisr hex                 at the start of FCU tick + 1: the interrupts since `tick` started, the rise= reads
 *   O tick hex                      the fallchg= read when it differs from the previous tick's
 *   NT ep tick rise fall t2 t0 busy=c nisr=n v9=c:n v15=c:n v19=c:n vo=c:n
 *                                   one NAV tick (epoch ep = NAV boots): marker up / down, its Timer2 compare (the tick),
 *                                   the Timer0 compare before it (the IMU lead), interrupts inside [t2, fall] by vector
 *                                   (9 Timer2, 15 Timer0, 19 console UDRE, o other)
 *   NB ep tick hex                  at the start of NAV tick + 1: the navrise= reads (they describe `tick`)
 *   X n ep ss_lo ss_hi nb t_first t_last late late_hdr wcol margin_min:byte:vec isr_lat_max dly_max faults mosi miso_rx frx
 *     miso_clean
 *                                   one nlink exchange (SS low .. high): NAV bytes, first byte start / last byte end;
 *                                   reply bytes 1.. not loaded before the master's next byte (late; of them 1..9:
 *                                   late_hdr), write collisions, the smallest margin (cycles) from a reply write to the
 *                                   byte that sends it (with that byte's index and the FCU interrupt that was running at
 *                                   its SPIF, 0 none), the largest SPIF -> reply-write latency, the largest lateness
 *                                   of a delivery to the FCU (cycles past te), the fault mask (1 MOSI flip, 2 MISO flip,
 *                                   4 drop, 8 extra, 16 noise, 32 reset); MOSI as the NAV sent it, MISO as the NAV
 *                                   received it, the bytes the FCU received ('=' if MOSI exactly), the clean MISO's
 *                                   first 10 bytes ('=' if the NAV received exactly those)
 *   E t name ...                    harness events (com_start, nav_start, fault_on, fault_off, com_reset, nav_reset, ...)
 *   UL ep u seq cmd flags number t_uplink t_queued t_snap t_telem    COM uplink records (com_cosim.h), epoch ep
 *   ST ep k kind t_us               COM IL_STAT records
 *   CR ep name=value...             the COM report's header and counters at the last valid snapshot of epoch ep
 *   CS ep pass_max_s_us...          the COM's per-second pass maxima
 *   I core vector runs cycles min max     interrupt totals (RUNNING edge to RETI)
 *   M name=value...                 end of run: core states, cycles, lowest SP per core, wire / nlink counters
 *   L vec:n:lat_max:margin_min...   the slave's reply writes by the FCU interrupt running at SPIF (0: none): count, the
 *                                   largest SPIF -> reply-write latency, the smallest margin to the master's next byte
 *
 * usage: cosim fcu=<hex> com=<hex> nav=<hex> out=<events.txt> [report=<data index of g_cosim_report>]
 *              [rise=a:n,...] [fall=a:n,...] [fallchg=a:n] [navrise=a:n,...] [pwm=a0,...,a5] [delay_us=0]
 *              [nav_delay_us=0] [wire_bits=10] [step=16] [seconds=16] [isr_resp=4] [snap_us=500]
 *              [fault=none|cut|reset|babble|nav_hold|nav_pulse|nl_flip|nl_drop|nl_extra|nl_noise] [fault_at_ms=6000]
 *              [fault_len_ms=1000] [babble=c55c3102] [nl_every=8] [nl_ber=1e-4] [nav_reset_byte=30] [seed=1]
 *
 * v0.9 additions (cosim4):
 *   faults  nl_lose_cmp  the NAV's Timer3 compare-B interrupt enable (ETIMSK.OCIE3B) is cleared when the NAV writes
 *                        byte nl_lose_byte of the first exchange that starts after fault_at_ms: the interrupt-paced
 *                        exchange stops there (W1: the NAV's exchange watchdog must close it)
 *           gns_hold     the GNS is held in reset from fault_at_ms to the end
 *           gns_pulse    the GNS is held in reset for fault_len_ms, then runs again (MCUCSR = EXTRF)
 *           gps_cut      the GNS's synthetic receiver sends nothing for fault_len_ms (g_cosim_gps.cut)
 *   records P t h vz x y vx vy roll pitch yaw p q r u_cmd u_lag d1 d2 d3 d4 air tilt   the plant every pl_log_ms
 *           GS t b / GR t b              a NAV -> GNS / GNS -> NAV glink byte
 *           GQ t hex                     the GNS's g_gns_cosim_report every 250 ms (gnsrep=)
 *           MG ... / MX ...              the GNS core's figures; the corrections' and the plant's counters
 *   arguments gns=<hex> [gns_delay_us=0] [gps=a:n] [gnsrep=a:n] [twi=0|1] [twifix=1] [t0fix=1] [plant=0|1]
 *           [baro=ms5611|bmp280]  the barometer model at 0x77 (twi=1): the one the NAV image was built for
 *           [pl_dt_us=250] [pl_log_ms=20] [pwm_map=v1,v2,v3,v4,esc] [pl_<name>=value ...] [nl_lose_byte=30]
 *
 * Invariant: the harness only wires, observes and injects the configured faults - it never writes a firmware's RAM
 * except the GNS's harness-owned g_cosim_gps block (the plant's GPS antenna, closed loop), and of the simulated hardware
 * only the USART byte times, the interrupt response cycles above, a held core's reset and MCUCSR.EXTRF / WDRF when it
 * releases it, the FCU's PB0 level (the NAV drives it), Timer0's preload and the TWI state timer (both simavr
 * corrections above), the sensor registers on the NAV's TWI (twi_models.c) and, as the nl_lose_cmp fault, the NAV's
 * ETIMSK.OCIE3B.
 */
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "avr_ioport.h"
#include "avr_spi.h"
#include "avr_uart.h"
#include "sim_avr.h"
#include "sim_cycle_timers.h"
#include "sim_hex.h"
#include "sim_interrupts.h"
#include "sim_io.h"
#include "sim_regbit.h"

#include "com_cosim.h"
#include "cosim4.h"

#define F_CPU_HZ 16000000ULL
#define CYC_PER_US 16ULL
#define NVEC 64
#define D_SPCR 0x2D                          /* data-space addresses (ATmega128) */
#define D_SPSR 0x2E
#define D_SPDR 0x2F
#define D_DDRB 0x37
#define D_PORTB 0x38
#define D_SPL 0x5D
#define D_SPH 0x5E
#define D_MCUCSR 0x54
#define D_UCSR1B 0x9A
#define SPCR_SPE 0x40
#define SPCR_MSTR 0x10
#define SPSR_SPI2X 0x01
#define RXEN1_BIT 4
#define MCUCSR_EXTRF 0x02
#define MCUCSR_WDRF 0x08
#define D_ETIMSK 0x7D                        /* ATmega128 extended I/O */
#define ETIMSK_OCIE3B 0x08
#define D_TCNT0 0x52
#define D_TCCR0 0x53
#define GPS_MAGIC 0x7067u                     /* gns_simbench.h g_cosim_gps.magic ('gp') */
#define GPS_BLOCK_BYTES 24u
#define V_T2_COMP 9                          /* ATmega128 vectors */
#define V_T0_COMP 15
#define V_SPI_STC 17
#define V_U0_UDRE 19                         /* FCU log port / NAV console */
#define V_IL_RX 30                           /* FCU USART1 RX */
#define V_IL_UDRE 31                         /* FCU USART1 UDRE */
#define MAXEP 8                              /* COM epochs (one per (re)start) */
#define MAXUL 1024
#define MAXST 256
#define NREAD 8
#define NXB 160                              /* bytes kept per exchange (72 + phantoms, guard) */
#define SP_RUN_AFTER_CYC (100000ULL * CYC_PER_US)   /* 100 ms: the boot code (init frames) is over */

/* ------------------------------------------------------------------ configuration */
static const char *g_fcu_hex, *g_com_hex, *g_nav_hex, *g_gns_hex, *g_out_path, *g_fault = "none";
static long long g_report = -1;
static double g_gns_delay_us, g_delay_us, g_nav_delay_us, g_fault_at_ms = 6000.0, g_fault_len_ms = 1000.0, g_seconds = 16.0,
              g_snap_us = 500.0, g_nl_ber = 1e-4;
static int g_wire_bits = 10, g_step = 16, g_isr_resp = 4, g_nl_every = 8, g_nav_reset_byte = 30;
static uint64_t g_seed = 1;
static uint8_t g_bab[64];
static unsigned g_bab_n;
static uint32_t g_pwm_addr[6];
static unsigned g_pwm_n;
typedef struct {
    unsigned n;
    uint32_t a[NREAD], len[NREAD];
} rdlist_t;
static rdlist_t g_rd_rise, g_rd_fall, g_rd_chg, g_rd_navrise, g_rd_gps, g_rd_gnsrep;
/* v0.9 (cosim4) */
static int g_twi_on, g_twifix = 1, g_t0fix = 1, g_plant_on, g_nl_lose_byte = 30;
static double g_pl_dt_us = 250.0, g_pl_log_ms = 20.0;
static int g_pwm_map[5] = {0, 1, 2, 3, 4};   /* vane 1..4, ESC -> PWM channel (pwm= order) */
static plant_par_t g_pp = {
    0.916, 2.0, 30.3, 0.351, 0.028, 0.00096, 2.4, 0.00507, 0.1319, 0.1325, 0.00502, 0.15, 9.80665,
    1000.0, 2000.0, {1500.0, 1500.0, 1500.0, 1500.0}, {636.0, 636.0, 636.0, 636.0},
    0.017453293, -0.0087266463, 0.0, 0.0, 0.0, {0.0, 0.0, 0.0}, 0.0, 0.002, 0.02, 1.5, 100009.0, 0.0, 0x5EEDull};

static FILE *g_out;

static void die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    exit(2);
}

static void parse_rdlist(rdlist_t *l, const char *s)
{
    char *end;
    l->n = 0;
    while (*s) {
        if (l->n >= NREAD) die("too many reads in %s", s);
        l->a[l->n] = (uint32_t)strtoul(s, &end, 0);
        if (*end != ':') die("read list item needs addr:len (%s)", s);
        l->len[l->n] = (uint32_t)strtoul(end + 1, &end, 0);
        if (l->a[l->n] + l->len[l->n] > 0x1100u || l->len[l->n] == 0) die("read outside the ATmega128 RAM (%s)", s);
        l->n++;
        s = *end == ',' ? end + 1 : end;
    }
}

static void parse_hex_pattern(const char *s)
{
    g_bab_n = 0;
    while (s[0] && s[1] && g_bab_n < sizeof g_bab) {
        char b[3] = {s[0], s[1], 0};
        g_bab[g_bab_n++] = (uint8_t)strtoul(b, NULL, 16);
        s += 2;
    }
    if (!g_bab_n) die("babble pattern empty");
}

static void parse_ints(const char *s, int *out, unsigned n, const char *what)
{
    char *end;
    unsigned k = 0;
    while (*s && k < n) {
        out[k++] = (int)strtol(s, &end, 0);
        s = *end == ',' ? end + 1 : end;
    }
    if (k != n) die("%s takes %u numbers", what, n);
}

static void parse_dbls(const char *s, double *out, unsigned n, const char *what)
{
    char *end;
    unsigned k = 0;
    while (*s && k < n) {
        out[k++] = strtod(s, &end);
        s = *end == ',' ? end + 1 : end;
    }
    if (k != n) die("%s takes %u numbers", what, n);
}

/* pl_<name>=value: the plant's parameters (cosim4.h plant_par_t; run_cosim4.py passes the image's params_gen.h values) */
static int parse_plant(const char *k, const char *v)
{
    static const struct {
        const char *name;
        size_t off;
    } tab[] = {
        {"u_hover", offsetof(plant_par_t, u_hover)}, {"n_exp", offsetof(plant_par_t, n_exp)},
        {"t_max", offsetof(plant_par_t, t_max)}, {"l_v", offsetof(plant_par_t, l_v)}, {"r_v", offsetof(plant_par_t, r_v)},
        {"s_v", offsetof(plant_par_t, s_v)}, {"cl_a", offsetof(plant_par_t, cl_a)}, {"a_e", offsetof(plant_par_t, a_e)},
        {"ixx", offsetof(plant_par_t, ixx)}, {"iyy", offsetof(plant_par_t, iyy)}, {"izz", offsetof(plant_par_t, izz)},
        {"tau", offsetof(plant_par_t, tau)}, {"g0", offsetof(plant_par_t, g0)},
        {"esc_min_us", offsetof(plant_par_t, esc_min_us)}, {"esc_max_us", offsetof(plant_par_t, esc_max_us)},
        {"pad_roll", offsetof(plant_par_t, pad_roll)}, {"pad_pitch", offsetof(plant_par_t, pad_pitch)},
        {"wind_ax", offsetof(plant_par_t, wind_ax)}, {"wind_ay", offsetof(plant_par_t, wind_ay)},
        {"drag_k", offsetof(plant_par_t, drag_k)},
        {"psi0", offsetof(plant_par_t, psi0)}, {"gyro_noise", offsetof(plant_par_t, gyro_noise)},
        {"accel_noise", offsetof(plant_par_t, accel_noise)}, {"baro_noise_pa", offsetof(plant_par_t, baro_noise_pa)},
        {"p_ground_pa", offsetof(plant_par_t, p_ground_pa)}, {"baro_comp_m", offsetof(plant_par_t, baro_comp_m)}};
    unsigned i;
    if (strncmp(k, "pl_", 3)) return 0;
    k += 3;
    if (!strcmp(k, "c0_us")) {
        parse_dbls(v, g_pp.c0_us, 4, "pl_c0_us");
        return 1;
    }
    if (!strcmp(k, "k_us_per_rad")) {
        parse_dbls(v, g_pp.k_us_per_rad, 4, "pl_k_us_per_rad");
        return 1;
    }
    if (!strcmp(k, "lever_b")) {
        parse_dbls(v, g_pp.lever_b, 3, "pl_lever_b");
        return 1;
    }
    if (!strcmp(k, "dt_us")) {
        g_pl_dt_us = strtod(v, NULL);
        return 1;
    }
    if (!strcmp(k, "log_ms")) {
        g_pl_log_ms = strtod(v, NULL);
        return 1;
    }
    for (i = 0; i < sizeof tab / sizeof tab[0]; i++)
        if (!strcmp(k, tab[i].name)) {
            *(double *)((char *)&g_pp + tab[i].off) = strtod(v, NULL);
            return 1;
        }
    return 0;
}

static void parse_pwm(const char *s)
{
    char *end;
    g_pwm_n = 0;
    while (*s && g_pwm_n < 6u) {
        g_pwm_addr[g_pwm_n++] = (uint32_t)strtoul(s, &end, 0);
        s = *end == ',' ? end + 1 : end;
    }
}

static int fault_is(const char *f)
{
    return !strcmp(g_fault, f);
}

static void parse_args(int argc, char **argv)
{
    static const char *faults[] = {"none", "cut", "reset", "babble", "nav_hold", "nav_pulse", "nl_flip", "nl_drop",
                                   "nl_extra", "nl_noise", "nl_lose_cmp", "gns_hold", "gns_pulse", "gps_cut"};
    unsigned k;
    int i, ok = 0;
    parse_hex_pattern("c55c3102");
    for (i = 1; i < argc; i++) {
        char *eq = strchr(argv[i], '=');
        const char *v;
        if (!eq) die("argument without '=': %s", argv[i]);
        *eq = 0;
        v = eq + 1;
        if (!strcmp(argv[i], "fcu")) g_fcu_hex = v;
        else if (!strcmp(argv[i], "com")) g_com_hex = v;
        else if (!strcmp(argv[i], "nav")) g_nav_hex = v;
        else if (!strcmp(argv[i], "gns")) g_gns_hex = v;
        else if (!strcmp(argv[i], "gns_delay_us")) g_gns_delay_us = strtod(v, NULL);
        else if (!strcmp(argv[i], "out")) g_out_path = v;
        else if (!strcmp(argv[i], "report")) g_report = strtoll(v, NULL, 0);
        else if (!strcmp(argv[i], "rise")) parse_rdlist(&g_rd_rise, v);
        else if (!strcmp(argv[i], "fall")) parse_rdlist(&g_rd_fall, v);
        else if (!strcmp(argv[i], "fallchg")) parse_rdlist(&g_rd_chg, v);
        else if (!strcmp(argv[i], "navrise")) parse_rdlist(&g_rd_navrise, v);
        else if (!strcmp(argv[i], "pwm")) parse_pwm(v);
        else if (!strcmp(argv[i], "delay_us")) g_delay_us = strtod(v, NULL);
        else if (!strcmp(argv[i], "nav_delay_us")) g_nav_delay_us = strtod(v, NULL);
        else if (!strcmp(argv[i], "wire_bits")) g_wire_bits = atoi(v);
        else if (!strcmp(argv[i], "step")) g_step = atoi(v);
        else if (!strcmp(argv[i], "seconds")) g_seconds = strtod(v, NULL);
        else if (!strcmp(argv[i], "isr_resp")) g_isr_resp = atoi(v);
        else if (!strcmp(argv[i], "fault")) g_fault = v;
        else if (!strcmp(argv[i], "fault_at_ms")) g_fault_at_ms = strtod(v, NULL);
        else if (!strcmp(argv[i], "fault_len_ms")) g_fault_len_ms = strtod(v, NULL);
        else if (!strcmp(argv[i], "babble")) parse_hex_pattern(v);
        else if (!strcmp(argv[i], "snap_us")) g_snap_us = strtod(v, NULL);
        else if (!strcmp(argv[i], "nl_every")) g_nl_every = atoi(v);
        else if (!strcmp(argv[i], "nl_ber")) g_nl_ber = strtod(v, NULL);
        else if (!strcmp(argv[i], "nav_reset_byte")) g_nav_reset_byte = atoi(v);
        else if (!strcmp(argv[i], "seed")) g_seed = strtoull(v, NULL, 0);
        else if (!strcmp(argv[i], "gps")) parse_rdlist(&g_rd_gps, v);
        else if (!strcmp(argv[i], "gnsrep")) parse_rdlist(&g_rd_gnsrep, v);
        else if (!strcmp(argv[i], "twi")) g_twi_on = atoi(v);
        else if (!strcmp(argv[i], "twifix")) g_twifix = atoi(v);
        else if (!strcmp(argv[i], "twidbg")) g_twi_debug_n = atoi(v);
        else if (!strcmp(argv[i], "baro")) {
            if (!strcmp(v, "bmp280")) twi_models_set_baro(TWI_BARO_BMP280);
            else if (!strcmp(v, "ms5611")) twi_models_set_baro(TWI_BARO_MS5611);
            else die("baro=%s: ms5611 or bmp280", v);
        }
        else if (!strcmp(argv[i], "t0fix")) g_t0fix = atoi(v);
        else if (!strcmp(argv[i], "plant")) g_plant_on = atoi(v);
        else if (!strcmp(argv[i], "pwm_map")) parse_ints(v, g_pwm_map, 5, "pwm_map");
        else if (!strcmp(argv[i], "nl_lose_byte")) g_nl_lose_byte = atoi(v);
        else if (parse_plant(argv[i], v)) (void)0;
        else die("unknown argument %s", argv[i]);
        *eq = '=';
    }
    if (!g_fcu_hex || !g_com_hex || !g_nav_hex || !g_gns_hex || !g_out_path)
        die("usage: cosim fcu=<hex> com=<hex> nav=<hex> out=<file> [key=value...]");
    if (g_wire_bits != 10 && g_wire_bits != 11) die("wire_bits must be 10 or 11");
    if (g_step < 1 || g_step > 4096) die("step must be 1..4096");
    if (g_isr_resp < 0 || g_isr_resp > 8) die("isr_resp must be 0..8");
    if (g_nl_every < 2) die("nl_every must be >= 2");
    if (g_nav_reset_byte < 0 || g_nav_reset_byte > 71) die("nav_reset_byte must be 0..71");
    if (g_pwm_n != 0 && g_pwm_n != 6) die("pwm takes 6 addresses (channels 0..5)");
    for (k = 0; k < sizeof faults / sizeof faults[0]; k++)
        if (fault_is(faults[k])) ok = 1;
    if (!ok) die("unknown fault %s", g_fault);
    if (g_rd_chg.n > 1) die("fallchg takes one addr:len");
    if (g_rd_gps.n > 1 || (g_rd_gps.n && g_rd_gps.len[0] != GPS_BLOCK_BYTES))
        die("gps takes one addr:%u (gns_simbench.h gns_cosim_gps_t)", GPS_BLOCK_BYTES);
    if (g_rd_gnsrep.n > 1) die("gnsrep takes one addr:len");
    if (g_nl_lose_byte < 1 || g_nl_lose_byte > 71) die("nl_lose_byte must be 1..71");
    for (k = 0; k < 5; k++)
        if (g_pwm_map[k] < 0 || g_pwm_map[k] > 5) die("pwm_map entries must be channels 0..5");
    if (g_plant_on && g_pwm_n != 6) die("plant=1 needs pwm= (the six compare registers)");
    if (g_pl_dt_us < 16.0 || g_pl_dt_us > 4000.0) die("pl_dt_us must be 16..4000");
    if (!g_seed) g_seed = 1;
}

/* ------------------------------------------------------------------ cores */
typedef struct {
    avr_t *avr;
    char tag;                                /* 'F' FCU, 'C' COM, 'N' NAV */
    int idx;                                 /* 0 F, 1 C, 2 N */
    int64_t off;                             /* global cycle = avr->cycle + off */
    int state;
    int held;                                /* held in reset by the harness */
    int epoch;                               /* (re)starts */
    uint16_t sp_min, sp_min_run;             /* lowest SP: whole run / from SP_RUN_AFTER_CYC after (re)start */
    uint64_t t_start;                        /* global cycle of the core's (re)start */
    avr_uart_t *uart[2];
    avr_cycle_count_t uart_want[2];          /* the byte time uart_fix set (0: none yet) */
    uint64_t isr_in[NVEC], isr_n[NVEC], isr_tot[NVEC], isr_min[NVEC], isr_max[NVEC];
    int depth;
    uint64_t busy_from;
    /* the measured window (FCU: tick marker up .. down; NAV: Timer2 compare .. marker down) */
    int win;
    uint64_t win_from, win_busy, win_nisr, win_c[NVEC], win_n[NVEC], tick_nisr;
    avr_run_t run0;                          /* simavr's run callback: a watchdog reset swaps it (core_wdt_reset) */
    unsigned wdt_resets, t0fix;
    uint8_t tccr0_prev;
} core_t;
static core_t F, C, N, G;
static core_t *volatile g_run;               /* the core inside avr_run (the crash handler names it) */

static inline uint64_t gnow(const core_t *c)
{
    return (uint64_t)((int64_t)c->avr->cycle + c->off);
}

static int done(const core_t *c)
{
    return c->state == cpu_Done || c->state == cpu_Crashed;
}

static avr_cycle_count_t step_timer(avr_t *avr, avr_cycle_count_t when, void *param)
{
    (void)avr;
    (void)param;
    return when + (avr_cycle_count_t)g_step;
}

static void quiet_logger(avr_t *avr, const int level, const char *fmt, va_list ap)
{
    if (level <= LOG_ERROR) {
        fprintf(stderr, "[%s] ", avr == F.avr ? "FCU" : avr == C.avr ? "COM" : avr == N.avr ? "NAV" : avr == G.avr ? "GNS" : "?");
        vfprintf(stderr, fmt, ap);
    }
}

static void find_uarts(core_t *c)
{
    avr_io_t *p;
    for (p = c->avr->io_port; p; p = p->next) {
        if (p->kind && !strcmp(p->kind, "uart")) {
            avr_uart_t *u = (avr_uart_t *)p;
            if (u->name == '0' || u->name == '1') c->uart[u->name - '0'] = u;
        }
    }
    if (!c->uart[0] || !c->uart[1]) die("%c: USART0 / USART1 not found in simavr's io list", c->tag);
}

static void no_stdio(core_t *c)
{
    int i;
    uint32_t flags = 0;
    for (i = 0; i < 2; i++) {                /* the hooks below print: simavr's console echo off */
        avr_ioctl(c->avr, AVR_IOCTL_UART_GET_FLAGS('0' + i), &flags);
        flags &= ~(uint32_t)(AVR_UART_FLAG_STDIO | AVR_UART_FLAG_POLL_SLEEP);   /* POLL_SLEEP: wall-clock usleep only */
        avr_ioctl(c->avr, AVR_IOCTL_UART_SET_FLAGS('0' + i), &flags);
    }
}

/* wire_bits=10: the byte time of the frame the firmware configured (start + data + parity if enabled + stop bits, from
 * UBRR / U2X / UCSZ / UPM / USBS as they stand now), instead of simavr's.  Checked after every instruction of the core,
 * so the correction is in place before the first byte. */
static void uart_fix(core_t *c, int i)
{
    static const int db_tab[8] = {5, 6, 7, 8, 8, 8, 8, 9};
    avr_uart_t *p = c->uart[i];
    avr_t *a = c->avr;
    uint32_t ubrr, cpbit, db, sb, upm;
    avr_cycle_count_t want;
    if (!avr_regbit_get(a, p->txen) && !avr_regbit_get(a, p->rxen)) return;
    ubrr = (uint32_t)avr_regbit_get(a, p->ubrrl) | ((uint32_t)avr_regbit_get(a, p->ubrrh) << 8);
    cpbit = (ubrr + 1u) * (avr_regbit_get(a, p->u2x) ? 8u : 16u);
    db = (uint32_t)db_tab[avr_regbit_get(a, p->ucsz) | (avr_regbit_get(a, p->ucsz2) << 2)];
    sb = 1u + avr_regbit_get(a, p->usbs);
    upm = (uint32_t)(a->data[p->r_ucsrc] >> 4) & 3u;
    want = (avr_cycle_count_t)cpbit * (1u + db + (upm ? 1u : 0u) + sb);
    if (p->cycles_per_byte != want) {
        if (want != c->uart_want[i])
            fprintf(g_out, "W %c %c %llu %llu %llu\n", c->tag, p->name, (unsigned long long)p->cycles_per_byte,
                    (unsigned long long)want, (unsigned long long)gnow(c));
        p->cycles_per_byte = want;
    }
    c->uart_want[i] = want;
}

static void after_step(core_t *c)
{
    const uint16_t sp = (uint16_t)(c->avr->data[D_SPL] | (c->avr->data[D_SPH] << 8));
    if (sp < c->sp_min && sp >= 0x100u) c->sp_min = sp;
    if (sp < c->sp_min_run && sp >= 0x100u && gnow(c) >= c->t_start + SP_RUN_AFTER_CYC) c->sp_min_run = sp;
    if (g_wire_bits == 10) {
        uart_fix(c, 0);
        uart_fix(c, 1);
    }
}

static void load_hex(core_t *c, const char *hex)
{
    uint32_t size = 0, start = 0;
    uint8_t *code;
    FILE *probe = fopen(hex, "rb");          /* v0.9: simavr's read_ihex_file crashes on a missing file (measured) */
    if (!probe) die("cannot open %s", hex);
    fclose(probe);
    code = read_ihex_file(hex, &size, &start);
    if (!code) die("cannot read %s", hex);
    if (start + size > c->avr->flashend + 1u) die("%s does not fit the flash", hex);
    memcpy(c->avr->flash + start, code, size);
    c->avr->codeend = start + size - 1u;
    c->avr->pc = start;
    free(code);
}

/* ------------------------------------------------------------------ interrupt accounting */
static core_t *core_by_idx(unsigned i)
{
    return i == 0 ? &F : i == 1 ? &C : i == 2 ? &N : &G;
}

static void win_open(core_t *c, uint64_t t)
{
    c->win = 1;
    c->win_from = t;
    c->win_busy = c->win_nisr = 0;
    memset(c->win_c, 0, sizeof c->win_c);
    memset(c->win_n, 0, sizeof c->win_n);
}

static void isr_running_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    const uintptr_t pv = (uintptr_t)param;
    core_t *c = core_by_idx((unsigned)(pv >> 8));
    const unsigned v = (unsigned)(pv & 0xFFu);
    const uint64_t t = gnow(c);
    (void)irq;
    if (value) {
        c->isr_in[v] = t + 1u;               /* +1: 0 means "not running" */
        if (c->depth++ == 0) c->busy_from = t;
        if (g_isr_resp) c->avr->cycle += (avr_cycle_count_t)g_isr_resp;   /* the chip's interrupt response */
        return;
    }
    if (c->isr_in[v]) {
        const uint64_t t_in = c->isr_in[v] - 1u, d = t - t_in;
        c->isr_n[v]++;
        c->isr_tot[v] += d;
        c->tick_nisr++;
        if (d < c->isr_min[v]) c->isr_min[v] = d;
        if (d > c->isr_max[v]) c->isr_max[v] = d;
        if (c->win && t_in >= c->win_from) {
            c->win_c[v] += d;
            c->win_n[v]++;
            c->win_nisr++;
        }
        c->isr_in[v] = 0;
        if (c->depth > 0 && --c->depth == 0 && c->win && c->busy_from >= c->win_from) c->win_busy += t - c->busy_from;
    }
}

static void hook_isrs(core_t *c)
{
    unsigned v;
    for (v = 1; v < 35u; v++) {
        avr_irq_t *irq = avr_get_interrupt_irq(c->avr, (uint8_t)v);
        c->isr_min[v] = ~0ull;
        if (irq)
            avr_irq_register_notify(irq + AVR_INT_IRQ_RUNNING, isr_running_hook,
                                    (void *)(uintptr_t)(((unsigned)c->idx << 8) | v));
    }
}

/* ------------------------------------------------------------------ the ilink wire */
static avr_irq_t *g_fcu_in, *g_com_in;
static int g_cut, g_babble_on;
#define RQ 4096u
static struct {
    uint64_t t;
    uint8_t b;
    char src;
} g_rq[RQ];                                  /* bytes in the FCU's USART1 FIFO, in order */
static uint32_t g_rq_head, g_rq_tail;
static uint64_t g_r_index, g_rxc_unmatched, g_rq_overflow;
static uint64_t g_c2f_bytes, g_f2c_bytes, g_c2f_cut, g_f2c_cut, g_c2f_babbled, g_c2f_rxoff, g_bab_bytes, g_u1_bytes;

static void fcu_rx_push(uint8_t b, char src, uint64_t t)
{
    if (!(F.avr->data[D_UCSR1B] & (1u << RXEN1_BIT))) {   /* receiver off (boot): simavr drops the byte */
        g_c2f_rxoff++;
        fprintf(g_out, "D c2f %llu %u rxoff\n", (unsigned long long)t, b);
        return;
    }
    if (g_rq_head - g_rq_tail >= RQ) {
        g_rq_overflow++;
    } else {
        g_rq[g_rq_head % RQ].t = t;
        g_rq[g_rq_head % RQ].b = b;
        g_rq[g_rq_head % RQ].src = src;
        g_rq_head++;
    }
    avr_raise_irq(g_fcu_in, b);
}

static void fcu_rxc_pending_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (!value) return;
    if (g_rq_tail == g_rq_head) {
        g_rxc_unmatched++;
        return;
    }
    fprintf(g_out, "R %llu %llu %llu %u %c\n", (unsigned long long)g_r_index++, (unsigned long long)g_rq[g_rq_tail % RQ].t,
            (unsigned long long)gnow(&F), g_rq[g_rq_tail % RQ].b, g_rq[g_rq_tail % RQ].src);
    g_rq_tail++;
}

static void com_tx0_hook(struct avr_irq_t *irq, uint32_t value, void *param)      /* COM USART0 -> FCU USART1 */
{
    const uint64_t t = gnow(&C);
    (void)irq;
    (void)param;
    g_c2f_bytes++;
    if (g_cut || g_babble_on) {
        if (g_cut) g_c2f_cut++;
        else g_c2f_babbled++;
        fprintf(g_out, "D c2f %llu %u %s\n", (unsigned long long)t, value & 0xFFu, g_cut ? "cut" : "babble");
        return;
    }
    fcu_rx_push((uint8_t)value, 'c', t);
}

static void fcu_tx1_hook(struct avr_irq_t *irq, uint32_t value, void *param)      /* FCU USART1 -> COM USART0 */
{
    const uint64_t t = gnow(&F);
    (void)irq;
    (void)param;
    g_f2c_bytes++;
    fprintf(g_out, "S %llu %u\n", (unsigned long long)t, value & 0xFFu);
    if (g_cut) {
        g_f2c_cut++;
        fprintf(g_out, "D f2c %llu %u cut\n", (unsigned long long)t, value & 0xFFu);
        return;
    }
    avr_raise_irq(g_com_in, value & 0xFFu);  /* a COM in reset has RXEN = 0: simavr drops it there */
}

static void com_tx1_hook(struct avr_irq_t *irq, uint32_t value, void *param)      /* COM USART1: the radio module */
{
    (void)irq;
    (void)param;
    g_u1_bytes++;
    fprintf(g_out, "U %llu %u\n", (unsigned long long)gnow(&C), value & 0xFFu);
}

static unsigned g_bab_k;
static avr_cycle_count_t babble_pump(avr_t *avr, avr_cycle_count_t when, void *param)
{
    (void)avr;
    (void)param;
    if (!g_babble_on) return 0;
    fcu_rx_push(g_bab[g_bab_k++ % g_bab_n], 'b', when);
    g_bab_bytes++;
    return when + F.uart[1]->cycles_per_byte;
}

/* ------------------------------------------------------------------ v0.8.1: the glink wire (NAV USART1 <-> GNS USART0) */
static avr_irq_t *g_gns_in, *g_nav1_in;
static uint64_t g_n2g_bytes, g_g2n_bytes;

static void nav_tx1_hook(struct avr_irq_t *irq, uint32_t value, void *param)      /* NAV USART1 -> GNS USART0 */
{
    (void)irq;
    (void)param;
    g_n2g_bytes++;
    fprintf(g_out, "GS %llu %u\n", (unsigned long long)gnow(&N), value & 0xFFu);
    avr_raise_irq(g_gns_in, value & 0xFFu);
}

static void gns_tx0_hook(struct avr_irq_t *irq, uint32_t value, void *param)      /* GNS USART0 -> NAV USART1 */
{
    (void)irq;
    (void)param;
    g_g2n_bytes++;
    fprintf(g_out, "GR %llu %u\n", (unsigned long long)gnow(&G), value & 0xFFu);
    avr_raise_irq(g_nav1_in, value & 0xFFu);
}

/* ------------------------------------------------------------------ consoles: report lines, log frames */
typedef struct {
    char line[600];
    unsigned ll;
    uint64_t line_t, bytes;
    uint8_t lf[300];
    unsigned lfn;
    uint64_t lf_t0;
} console_t;
static console_t g_fcon, g_ncon;

static void console_byte(console_t *k, core_t *c, const char *tag, uint8_t b, int log_frames)
{
    const uint64_t t = gnow(c);
    k->bytes++;
    if (log_frames) {                                      /* log frames: A5 5A type len payload fletcher16 */
        if (k->lfn == 0) {
            if (b == 0xA5) { k->lf[k->lfn++] = b; k->lf_t0 = t; }
        } else if (k->lfn == 1) {
            if (b == 0x5A) k->lf[k->lfn++] = b;
            else k->lfn = b == 0xA5 ? 1u : 0u;
        } else {
            k->lf[k->lfn++] = b;
            if (k->lfn >= 4u && k->lfn == (unsigned)k->lf[3] + 6u) {
                unsigned i;
                fprintf(g_out, "F %llu %llu ", (unsigned long long)k->lf_t0, (unsigned long long)t);
                for (i = 0; i < k->lfn; i++) fprintf(g_out, "%02x", k->lf[i]);
                fputc('\n', g_out);
                k->lfn = 0;
            } else if (k->lfn >= sizeof k->lf) {
                k->lfn = 0;
            }
        }
    }
    if (b == '\n' || b == '\r') {                          /* ASCII lines */
        if (k->ll && k->line[0] == 'S') {
            k->line[k->ll] = 0;
            fprintf(g_out, "%s %llu %s\n", tag, (unsigned long long)k->line_t, k->line);
        }
        k->ll = 0;
        return;
    }
    if (b >= 0x20 && b < 0x7F && k->ll < sizeof k->line - 1u) {
        if (k->ll == 0) k->line_t = t;
        k->line[k->ll++] = (char)b;
    } else {
        k->ll = 0;
    }
}

static void fcu_tx0_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    console_byte(&g_fcon, &F, "C", (uint8_t)value, 1);
}

static void nav_tx0_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    console_byte(&g_ncon, &N, "NC", (uint8_t)value, 0);
}

static void put_reads(const core_t *c, const rdlist_t *l, char lead)
{
    unsigned i, k;
    for (i = 0; i < l->n; i++) {
        if (i || lead) fputc(i ? ',' : lead, g_out);
        for (k = 0; k < l->len[i]; k++) fprintf(g_out, "%02x", c->avr->data[l->a[i] + k]);
    }
}

/* ------------------------------------------------------------------ FCU PWM compare registers */
static uint64_t g_pwm_t[6];                  /* last commit inside the current FCU window */

static void pwm_write_hook(struct avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    /* v0.9 (cosim4): store the byte.  simavr calls a registered I/O write hook INSTEAD of storing the value (sim_core
     * _avr_set_r), and the OCRnxH addresses carry no simavr hook of their own: since v0.6.3 the harness's hook swallowed
     * every high-byte write, so the FCU's compare registers held only their low bytes (OCR1A = 0x00B8 for a 1500 us
     * centre, 0x0BB8) - invisible to the timing figures (the write's moment is what they use), fatal for a plant that
     * reads the pulses.  simavr's timer reads the high byte from data[] when the low byte (its hooked commit) is written. */
    avr->data[addr] = v;
    if (F.win) g_pwm_t[(uintptr_t)param] = gnow(&F);
}

/* ------------------------------------------------------------------ FCU tick marker (PORTA) */
static int g_pa;
static uint32_t g_tick_now, g_ticks_seen, g_marker_jumps, g_tick_have;
static uint64_t g_rise;
static uint8_t g_chg_last[256];
static int g_chg_have;

static void porta_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    const uint64_t t = gnow(&F);
    (void)irq;
    (void)param;
    if ((value & 1u) && !(g_pa & 1)) {                     /* a tick starts */
        const uint32_t lo = (value >> 1) & 0x7Fu;
        uint32_t tk = g_tick_now + 1u;
        if (g_tick_have) {                                 /* the previous tick's record, now complete */
            fprintf(g_out, "B %lu %llu", (unsigned long)g_tick_now, (unsigned long long)F.tick_nisr);
            put_reads(&F, &g_rd_rise, ' ');
            fputc('\n', g_out);
        }
        if ((tk & 0x7Fu) != lo) {                          /* the first tick, or ticks were missed */
            tk = (tk & ~0x7Fu) | lo;
            if (tk < g_tick_now + 1u) tk += 0x80u;
            if (g_tick_have) g_marker_jumps++;
        }
        g_tick_now = tk;
        g_tick_have = 1;
        g_ticks_seen++;
        g_rise = t;
        F.tick_nisr = 0;
        memset(g_pwm_t, 0, sizeof g_pwm_t);
        win_open(&F, t);
    } else if (!(value & 1u) && (g_pa & 1)) {              /* end of the time-critical part */
        uint64_t oc = 0, on = 0;
        unsigned v;
        F.win = 0;
        for (v = 0; v < NVEC; v++)
            if (v != V_IL_RX && v != V_IL_UDRE && v != V_U0_UDRE && v != V_SPI_STC) {
                oc += F.win_c[v];
                on += F.win_n[v];
            }
        fprintf(g_out, "T %lu %llu %llu busy=%llu nisr=%llu v17=%llu:%llu v19=%llu:%llu v30=%llu:%llu v31=%llu:%llu "
                       "vo=%llu:%llu pwm=%llu,%llu,%llu,%llu,%llu,%llu rd=",
                (unsigned long)g_tick_now, (unsigned long long)g_rise, (unsigned long long)t,
                (unsigned long long)F.win_busy, (unsigned long long)F.win_nisr,
                (unsigned long long)F.win_c[V_SPI_STC], (unsigned long long)F.win_n[V_SPI_STC],
                (unsigned long long)F.win_c[V_U0_UDRE], (unsigned long long)F.win_n[V_U0_UDRE],
                (unsigned long long)F.win_c[V_IL_RX], (unsigned long long)F.win_n[V_IL_RX],
                (unsigned long long)F.win_c[V_IL_UDRE], (unsigned long long)F.win_n[V_IL_UDRE],
                (unsigned long long)oc, (unsigned long long)on, (unsigned long long)g_pwm_t[0],
                (unsigned long long)g_pwm_t[1], (unsigned long long)g_pwm_t[2], (unsigned long long)g_pwm_t[3],
                (unsigned long long)g_pwm_t[4], (unsigned long long)g_pwm_t[5]);
        put_reads(&F, &g_rd_fall, 0);
        fputc('\n', g_out);
        if (g_rd_chg.n) {
            const uint8_t *p = F.avr->data + g_rd_chg.a[0];
            const uint32_t n = g_rd_chg.len[0] < sizeof g_chg_last ? g_rd_chg.len[0] : (uint32_t)sizeof g_chg_last;
            if (!g_chg_have || memcmp(p, g_chg_last, n)) {
                uint32_t k;
                fprintf(g_out, "O %lu ", (unsigned long)g_tick_now);
                for (k = 0; k < n; k++) fprintf(g_out, "%02x", p[k]);
                fputc('\n', g_out);
                memcpy(g_chg_last, p, n);
                g_chg_have = 1;
            }
        }
    }
    g_pa = (int)value;
}

/* ------------------------------------------------------------------ NAV tick marker (PORTA), Timer2 / Timer0 */
static int g_npa;
static uint32_t g_ntick, g_nticks_seen, g_nmarker_jumps, g_ntick_have;
static uint64_t g_nrise, g_nt2, g_nt0, g_nt2_tick, g_nt0_tick;

static void nav_t2_pending_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    const uint64_t t = gnow(&N);
    (void)irq;
    (void)param;
    if (!value || N.held) return;
    g_nt2 = t;
    win_open(&N, t);                                       /* the NAV's tick: interrupts counted from its compare */
}

static void nav_t0_pending_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (value && !N.held) g_nt0 = gnow(&N);
}

static void nav_porta_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    const uint64_t t = gnow(&N);
    (void)irq;
    (void)param;
    if (N.held) return;
    if ((value & 1u) && !(g_npa & 1)) {
        const uint32_t lo = (value >> 1) & 0x7Fu;
        uint32_t tk = g_ntick + 1u;
        if (g_ntick_have) {
            fprintf(g_out, "NB %d %lu", N.epoch, (unsigned long)g_ntick);
            put_reads(&N, &g_rd_navrise, ' ');
            fputc('\n', g_out);
        }
        if ((tk & 0x7Fu) != lo) {
            tk = (tk & ~0x7Fu) | lo;
            if (tk < g_ntick + 1u) tk += 0x80u;
            if (g_ntick_have) g_nmarker_jumps++;
        }
        g_ntick = tk;
        g_ntick_have = 1;
        g_nticks_seen++;
        g_nrise = t;
        g_nt2_tick = g_nt2;
        g_nt0_tick = g_nt0;
    } else if (!(value & 1u) && (g_npa & 1)) {
        uint64_t oc = 0, on = 0;
        unsigned v;
        N.win = 0;
        for (v = 0; v < NVEC; v++)
            if (v != V_T2_COMP && v != V_T0_COMP && v != V_U0_UDRE) {
                oc += N.win_c[v];
                on += N.win_n[v];
            }
        fprintf(g_out, "NT %d %lu %llu %llu %llu %llu busy=%llu nisr=%llu v9=%llu:%llu v15=%llu:%llu v19=%llu:%llu "
                       "vo=%llu:%llu\n",
                N.epoch, (unsigned long)g_ntick, (unsigned long long)g_nrise, (unsigned long long)t,
                (unsigned long long)g_nt2_tick, (unsigned long long)g_nt0_tick, (unsigned long long)N.win_busy,
                (unsigned long long)N.win_nisr, (unsigned long long)N.win_c[V_T2_COMP],
                (unsigned long long)N.win_n[V_T2_COMP], (unsigned long long)N.win_c[V_T0_COMP],
                (unsigned long long)N.win_n[V_T0_COMP], (unsigned long long)N.win_c[V_U0_UDRE],
                (unsigned long long)N.win_n[V_U0_UDRE], (unsigned long long)oc, (unsigned long long)on);
    }
    g_npa = (int)value;
}

/* ------------------------------------------------------------------ the nlink (SPI) */
#define F_NL_MOSI_FLIP 1u
#define F_NL_MISO_FLIP 2u
#define F_NL_DROP 4u
#define F_NL_EXTRA 8u
#define F_NL_NOISE 16u
#define F_NL_RESET 32u
static int g_ss = 1;                         /* the FCU's PB0 level (NAV-driven or its pull-up) */
static int g_nl_fault_on;                    /* an nl_* fault window is open */
static uint64_t g_nl_fault_n;                /* exchanges started in the window */
static int g_nav_reset_armed, g_nav_reset_req;
static int g_lose_armed;                     /* nl_lose_cmp: armed by the fault window, fires once */
static uint64_t g_lose_done, g_lose_x;
static uint64_t g_fault_off;                 /* (the fault schedule's, tentative definition: see the resets section) */
static uint64_t g_rng;

static struct {                              /* the FCU's SPI shift register, as on the chip */
    uint8_t shift, prev_shift;
    int in_xfer;                             /* a byte (or a phantom) is shifting through the slave */
    uint64_t ts;                             /* its start */
    uint64_t last_load_t, prev_load_t;       /* the last two SPDR loads (the FCU can be one instruction ahead) */
    int loaded_since_te;
    uint64_t last_te;                        /* end of the previous byte at the slave */
    unsigned blk;                            /* the FCU interrupt running when that byte ended (0: none) */
} S;

typedef struct {                             /* one byte (or phantom phase) the FCU's cycle timer completes */
    int used, aborted, kind;                 /* kind 0 real byte end, 1 phantom start, 2 phantom end */
    uint8_t v;
    uint64_t ts, te;
} dslot_t;
#define NSLOT 8
static dslot_t g_ds[NSLOT];
/* nl_extra: the phantom byte (8 SCK edges of a glitch burst at ~4 MHz) shifts through the slave from te + 30 to te + 62
 * cycles after the real byte k: after the FCU's interrupt has loaded reply byte k + 1 (~27 cycles), before the NAV's
 * byte k + 1 (75 cycles) */
#define PHANTOM_START 30u
#define PHANTOM_END 62u

static struct {                              /* the exchange in progress (SS low) */
    int open;
    uint64_t n, ss_lo, t_first, t_last;
    unsigned nb, nfrx, late, late_hdr, wcol, margin_byte, margin_blk;
    int64_t margin_min;
    uint64_t isr_lat_max, dly_max;
    unsigned faults;
    uint8_t mosi[NXB], miso_rx[NXB], miso_clean[NXB], frx[NXB];
} X;
static uint64_t g_x_n, g_spi_bytes, g_spi_late, g_spi_late_hdr, g_spi_wcol, g_spi_dropped, g_spi_extra, g_spi_flips,
    g_spi_aborted, g_spi_ss_high_bytes, g_spi_out_mismatch, g_spi_dly_max, g_ss_edges;
static uint64_t g_lat_by_vec[NVEC], g_lat_n_by_vec[NVEC];   /* SPIF -> reply write: max and count by the interrupt
                                                            * running at SPIF (0: none) */
static int64_t g_margin_by_vec[NVEC];         /* reply write -> the master's next byte: min, by the same */
static uint8_t g_nav_last_mosi;              /* the byte the NAV wrote to SPDR (start of the byte) */
static uint8_t g_nav_next_miso;              /* what the NAV's SPI input gets at the end of the byte */
static int g_nav_byte_live;                  /* a NAV byte is in flight (its SPI output not yet raised) */

static double rng_uniform(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 7;
    g_rng ^= g_rng << 17;
    return (double)(g_rng >> 11) * (1.0 / 9007199254740992.0);
}

static uint8_t noise_byte(void)
{
    uint8_t m = 0;
    int b;
    for (b = 0; b < 8; b++)
        if (rng_uniform() < g_nl_ber) m |= (uint8_t)(1u << b);
    return m;
}

static int fcu_spi_slave_on(void)
{
    const uint8_t spcr = F.avr->data[D_SPCR];
    return (spcr & SPCR_SPE) && !(spcr & SPCR_MSTR);
}

static void fcu_ss_set(int level)
{
    avr_ioport_external_t e;
    level = level ? 1 : 0;
    if (level == g_ss) return;
    g_ss = level;
    g_ss_edges++;
    e.name = 'B';
    e.mask = 1;
    e.value = (unsigned)level;
    avr_ioctl(F.avr, AVR_IOCTL_IOPORT_SET_EXTERNAL('B'), &e);
    avr_raise_irq(avr_io_getirq(F.avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 0), (uint32_t)level);
}

static void x_close(uint64_t t, unsigned extra_faults)
{
    unsigned k;
    if (!X.open) return;
    X.faults |= extra_faults;
    fprintf(g_out, "X %llu %d %llu %llu %u %llu %llu %u %u %u %lld:%u:%u %llu %llu %u ", (unsigned long long)X.n, N.epoch,
            (unsigned long long)X.ss_lo, (unsigned long long)t, X.nb, (unsigned long long)X.t_first,
            (unsigned long long)X.t_last, X.late, X.late_hdr, X.wcol,
            (long long)(X.margin_min == INT64_MAX ? -1 : X.margin_min), X.margin_byte, X.margin_blk,
            (unsigned long long)X.isr_lat_max, (unsigned long long)X.dly_max, X.faults);
    for (k = 0; k < X.nb; k++) fprintf(g_out, "%02x", X.mosi[k]);
    if (!X.nb) fputc('-', g_out);
    fputc(' ', g_out);
    for (k = 0; k < X.nb; k++) fprintf(g_out, "%02x", X.miso_rx[k]);
    if (!X.nb) fputc('-', g_out);
    fputc(' ', g_out);
    if (X.nfrx == X.nb && !memcmp(X.frx, X.mosi, X.nb)) {
        fputc('=', g_out);
    } else {
        for (k = 0; k < X.nfrx; k++) fprintf(g_out, "%02x", X.frx[k]);
        if (!X.nfrx) fputc('-', g_out);
    }
    fputc(' ', g_out);
    {
        const unsigned m = X.nb < 10u ? X.nb : 10u;
        if (!memcmp(X.miso_clean, X.miso_rx, m)) {
            fputc('=', g_out);
        } else {
            for (k = 0; k < m; k++) fprintf(g_out, "%02x", X.miso_clean[k]);
        }
    }
    fputc('\n', g_out);
    X.open = 0;
}

static void x_open(uint64_t t)
{
    memset(&X, 0, sizeof X);
    X.open = 1;
    X.n = g_x_n++;
    X.ss_lo = t;
    X.margin_min = INT64_MAX;
    if (g_nl_fault_on) g_nl_fault_n++;
}

/* SS rising aborts a byte in the slave (the chip resets its SPI logic there) */
static void slave_abort(void)
{
    unsigned i;
    if (S.in_xfer) {
        S.in_xfer = 0;
        g_spi_aborted++;
    }
    for (i = 0; i < NSLOT; i++)
        if (g_ds[i].used) g_ds[i].aborted = 1;
}

static void nav_pb0_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    const uint64_t t = gnow(&N);
    int level;
    (void)irq;
    (void)param;
    if (N.held) return;
    level = (N.avr->data[D_DDRB] & 1u) ? (int)(value & 1u) : 1;   /* not driven: the FCU's pull-up */
    if (level == g_ss) return;
    if (!level) {
        fcu_ss_set(0);
        x_open(t);
    } else {
        fcu_ss_set(1);
        slave_abort();
        x_close(t, 0);
    }
}

/* FCU SPDR written (chained after simavr's SPI write): a load of the shift register, or a write collision */
static void fcu_spdr_write(struct avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    const uint64_t t = gnow(&F);
    (void)avr;
    (void)addr;
    (void)param;
    if (!fcu_spi_slave_on()) return;
    if (S.in_xfer) {                                       /* a byte is shifting: collision */
        g_spi_wcol++;
        if (X.open) X.wcol++;
        return;
    }
    S.prev_shift = S.shift;
    S.prev_load_t = S.last_load_t;
    S.shift = v;
    S.last_load_t = t;
    S.loaded_since_te = 1;
}

static int slot_alloc(void)
{
    int i;
    for (i = 0; i < NSLOT; i++)
        if (!g_ds[i].used) return i;
    die("nlink: no free delivery slot");
    return -1;
}

static avr_cycle_count_t slot_fire(avr_t *avr, avr_cycle_count_t when, void *param);

static void slot_schedule(int i, uint64_t t_global)
{
    const uint64_t now = gnow(&F);
    if (t_global <= now) {
        slot_fire(F.avr, F.avr->cycle, &g_ds[i]);
        return;
    }
    avr_cycle_timer_register(F.avr, (avr_cycle_count_t)(t_global - now), slot_fire, &g_ds[i]);
}

/* FCU context, at the slot's time: a byte (or a phantom) completes at the slave / a phantom starts */
static avr_cycle_count_t slot_fire(avr_t *avr, avr_cycle_count_t when, void *param)
{
    dslot_t *d = (dslot_t *)param;
    const uint64_t t = gnow(&F);
    (void)avr;
    (void)when;
    if (!d->used) return 0;
    if (d->kind == 1) {                                    /* phantom byte starts: the shift register is busy */
        if (!d->aborted) {
            if (S.loaded_since_te && S.last_load_t > d->ts) {   /* a load the FCU made after ts: a collision */
                S.shift = S.prev_shift;
                S.last_load_t = S.prev_load_t;
                g_spi_wcol++;
                if (X.open) X.wcol++;
            }
            S.in_xfer = 1;
            S.ts = d->ts;
        }
        d->used = 0;
        return 0;
    }
    if (t - d->te > g_spi_dly_max) g_spi_dly_max = t - d->te;
    if (X.open && t - d->te > X.dly_max) X.dly_max = t - d->te;
    if (!d->aborted && S.in_xfer) {
        unsigned v;
        S.in_xfer = 0;
        S.shift = d->v;                                    /* the received byte stays in the shift register */
        S.last_te = d->te;
        S.loaded_since_te = 0;
        S.blk = 0;
        for (v = 1; v < NVEC; v++)
            if (F.isr_in[v]) {
                S.blk = v;
                break;
            }
        if (!g_ss && fcu_spi_slave_on()) {
            avr_raise_irq(avr_io_getirq(F.avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT), d->v);
            if (X.open && X.nfrx < NXB) X.frx[X.nfrx++] = d->v;
        }
    }
    d->used = 0;
    return 0;
}

/* NAV SPDR written (chained after simavr's SPI write): with SPE + MSTR a byte starts now */
static void nav_spdr_write(struct avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    static const unsigned div_tab[4] = {4u, 16u, 64u, 128u};
    const uint64_t ts = gnow(&N);
    const uint8_t spcr = N.avr->data[D_SPCR];
    unsigned div, k;
    uint64_t te;
    uint8_t mosi_rx = v, miso_clean, miso_rx;
    int slave, drop = 0, extra = 0;
    (void)avr;
    (void)addr;
    (void)param;
    if (!(spcr & SPCR_SPE) || !(spcr & SPCR_MSTR)) return;
    div = div_tab[spcr & 3u];
    if (N.avr->data[D_SPSR] & SPSR_SPI2X) div /= 2u;
    te = ts + 8u * div;
    g_nav_last_mosi = v;
    g_nav_byte_live = 1;
    g_spi_bytes++;
    if (!X.open) {                                         /* clocked without SS low: nothing at the slave */
        g_spi_ss_high_bytes++;
        g_nav_next_miso = 0xFF;
        return;
    }
    k = X.nb;
    if (k == 0) X.t_first = ts;
    slave = !g_ss && fcu_spi_slave_on();
    /* the reply byte k: loaded by the FCU after byte k-1 ended (its interrupt), before this byte starts? */
    if (slave && S.loaded_since_te && S.last_load_t > ts) {   /* the FCU ran one instruction past ts: that write was late */
        S.shift = S.prev_shift;
        S.last_load_t = S.prev_load_t;
        S.loaded_since_te = S.prev_load_t > S.last_te;
        g_spi_wcol++;
        X.wcol++;
    }
    if (slave && k >= 1u) {
        if (S.loaded_since_te) {
            const int64_t m = (int64_t)(ts - S.last_load_t);
            const uint64_t lat = S.last_load_t - S.last_te;
            if (m < X.margin_min) {
                X.margin_min = m;
                X.margin_byte = k;
                X.margin_blk = S.blk;
            }
            if (lat > X.isr_lat_max) X.isr_lat_max = lat;
            if (lat > g_lat_by_vec[S.blk]) g_lat_by_vec[S.blk] = lat;
            if (!g_lat_n_by_vec[S.blk] || m < g_margin_by_vec[S.blk]) g_margin_by_vec[S.blk] = m;
            g_lat_n_by_vec[S.blk]++;
        } else {
            X.late++;
            g_spi_late++;
            if (k <= 9u) {
                X.late_hdr++;
                g_spi_late_hdr++;
            }
        }
    }
    miso_clean = slave ? S.shift : 0xFFu;
    miso_rx = miso_clean;
    /* faults */
    if (g_nl_fault_on) {
        const uint64_t n = g_nl_fault_n - 1u;              /* this exchange, counted in the window */
        const uint64_t e = (uint64_t)g_nl_every;
        if (fault_is("nl_flip")) {
            if (n % e == 0u && k == (unsigned)((n * 13u + 5u) % 72u)) {
                mosi_rx ^= (uint8_t)(1u << (n % 8u));
                X.faults |= F_NL_MOSI_FLIP;
                g_spi_flips++;
            }
            if (n % e == e / 2u && k == (unsigned)(n % 10u)) {
                miso_rx ^= (uint8_t)(1u << ((n + 3u) % 8u));
                X.faults |= F_NL_MISO_FLIP;
                g_spi_flips++;
            }
        } else if (fault_is("nl_drop")) {
            if (n % e == 0u && k == (unsigned)((n * 17u + 3u) % 72u)) {
                drop = 1;
                X.faults |= F_NL_DROP;
                g_spi_dropped++;
            }
        } else if (fault_is("nl_extra")) {
            if (n % e == 0u && k == (unsigned)((n * 19u + 7u) % 71u)) {
                extra = 1;
                X.faults |= F_NL_EXTRA;
                g_spi_extra++;
            }
        } else if (fault_is("nl_noise")) {
            const uint8_t a = noise_byte(), b = noise_byte();
            if (a || b) {
                mosi_rx ^= a;
                miso_rx ^= b;
                X.faults |= F_NL_NOISE;
                g_spi_flips += (uint64_t)__builtin_popcount(a) + (uint64_t)__builtin_popcount(b);
            }
        }
    }
    if (slave && drop)                                     /* no SCK at the slave: the master reads its MSB 8 times */
        miso_rx = (S.shift & 0x80u) ? 0xFFu : 0x00u;
    if (slave && !drop) {
        int i = slot_alloc();
        S.in_xfer = 1;
        S.ts = ts;
        g_ds[i].used = 1;
        g_ds[i].aborted = 0;
        g_ds[i].kind = 0;
        g_ds[i].v = mosi_rx;
        g_ds[i].ts = ts;
        g_ds[i].te = te;
        slot_schedule(i, te);
        if (extra) {                                       /* a phantom byte in the gap after this one */
            int j = slot_alloc(), m;
            g_ds[j].used = 1;
            g_ds[j].aborted = 0;
            g_ds[j].kind = 1;
            g_ds[j].ts = te + PHANTOM_START;
            g_ds[j].te = te + PHANTOM_END;
            g_ds[j].v = 0xFFu;
            slot_schedule(j, g_ds[j].ts);
            m = slot_alloc();
            g_ds[m] = g_ds[j];
            g_ds[m].kind = 0;
            slot_schedule(m, g_ds[m].te);
        }
    }
    if (k < NXB) {
        X.mosi[k] = v;
        X.miso_clean[k] = miso_clean;
        X.miso_rx[k] = miso_rx;
    }
    X.nb++;
    X.t_last = te;
    g_nav_next_miso = miso_rx;
    if (g_lose_armed && (int)k == g_nl_lose_byte) {        /* nl_lose_cmp: the next compare-B interrupt is lost */
        g_lose_armed = 0;
        g_lose_done++;
        g_fault_off = ts + (uint64_t)(g_fault_len_ms * 1000.0 * (double)CYC_PER_US + 0.5);
        g_lose_x = X.n;
        N.avr->data[D_ETIMSK] = (uint8_t)(N.avr->data[D_ETIMSK] & ~ETIMSK_OCIE3B);
        fprintf(g_out, "E %llu nl_lose_cmp exchange=%llu byte=%u\n", (unsigned long long)ts, (unsigned long long)X.n, k);
    }
    if (fault_is("nav_pulse") && g_nav_reset_armed && (int)k == g_nav_reset_byte) {
        g_nav_reset_armed = 0;
        g_nav_reset_req = 1;                               /* done by the main loop after this instruction */
    }
}

/* the NAV's SPI completes a byte (simavr's master timing): hand it the MISO byte */
static void nav_spi_out_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (N.held) return;
    if (g_nav_byte_live && (uint8_t)value != g_nav_last_mosi) g_spi_out_mismatch++;
    g_nav_byte_live = 0;
    avr_raise_irq(avr_io_getirq(N.avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT), g_nav_next_miso);
}

/* ------------------------------------------------------------------ the COM report (com_cosim.h) */
static int g_ep;                             /* COM epoch: 0 at the first start, +1 per reset */
static com_cosim_uplink_t g_ul[MAXEP][MAXUL];
static uint8_t g_ul_have[MAXEP][MAXUL];
static com_cosim_stat_t g_st[MAXEP][MAXST];
static uint8_t g_st_have[MAXEP][MAXST];
static com_cosim_report_t g_rep[MAXEP];
static int g_rep_ok[MAXEP];
static uint64_t g_rep_t[MAXEP], g_snaps_taken, g_snaps_odd;

static void com_snapshot(void)
{
    com_cosim_report_t r;
    unsigned u, n, k;
    if (g_report < 0 || g_ep >= MAXEP) return;
    memcpy(&r, C.avr->data + g_report, sizeof r);
    if (r.magic != COM_COSIM_MAGIC || r.version != COM_COSIM_VERSION || r.size != sizeof r) return;
    if (r.seqlock & 1u) {                    /* the COM is between REP_BEGIN and REP_END: next time */
        g_snaps_odd++;
        return;
    }
    g_snaps_taken++;
    g_rep[g_ep] = r;
    g_rep_ok[g_ep] = 1;
    g_rep_t[g_ep] = gnow(&C);
    n = r.ul_count;
    for (u = n > COM_COSIM_UPLINKS ? n - COM_COSIM_UPLINKS : 0u; u < n && u < MAXUL; u++) {
        const com_cosim_uplink_t *s = &r.ul[u % COM_COSIM_UPLINKS];
        if (s->number == (uint8_t)u) {
            g_ul[g_ep][u] = *s;
            g_ul_have[g_ep][u] = 1;
        }
    }
    n = r.stat_count;
    for (k = n > COM_COSIM_STATS ? n - COM_COSIM_STATS : 0u; k < n && k < MAXST; k++) {
        const com_cosim_stat_t *s = &r.st[k % COM_COSIM_STATS];
        if (s->number == (uint16_t)k) {
            g_st[g_ep][k] = *s;
            g_st_have[g_ep][k] = 1;
        }
    }
}

static void com_dump(void)
{
    int ep;
    unsigned u, k;
    for (ep = 0; ep <= g_ep && ep < MAXEP; ep++) {
        const com_cosim_report_t *r = &g_rep[ep];
        const com_cosim_counters_t *c = &r->c;
        for (u = 0; u < MAXUL; u++)
            if (g_ul_have[ep][u]) {
                const com_cosim_uplink_t *s = &g_ul[ep][u];
                fprintf(g_out, "UL %d %u %u %u %u %u %lu %lu %lu %lu\n", ep, u, s->seq, s->cmd, s->flags, s->number,
                        (unsigned long)s->t_uplink_us, (unsigned long)s->t_cmd_queued_us, (unsigned long)s->t_snap_us,
                        (unsigned long)s->t_telem_us);
            }
        for (k = 0; k < MAXST; k++)
            if (g_st_have[ep][k])
                fprintf(g_out, "ST %d %u %u %lu\n", ep, k, g_st[ep][k].kind, (unsigned long)g_st[ep][k].t_us);
        if (!g_rep_ok[ep]) {
            fprintf(g_out, "CR %d valid=0\n", ep);
            continue;
        }
        fprintf(g_out,
                "CR %d valid=1 at=%llu radio=%u done=%u t_us=%lu ticks=%lu passes=%lu pass_max_us=%u stack_free_min=%u "
                "sec1=%u sec2=%u sec3=%u sec4=%u sec5=%u ground_state=%u ground_acked=%u reset_cause=%u "
                "reply_window_ticks=%u tick_hz=%u ul_count=%u stat_count=%u",
                ep, (unsigned long long)g_rep_t[ep], r->radio, r->done, (unsigned long)r->t_us, (unsigned long)r->ticks,
                (unsigned long)r->passes, r->pass_max_us, r->stack_free_min, r->sec_max_us[1], r->sec_max_us[2],
                r->sec_max_us[3], r->sec_max_us[4], r->sec_max_us[5], r->ground_state, r->ground_acked, r->reset_cause,
                r->reply_window_ticks, r->tick_hz, r->ul_count, r->stat_count);
        fprintf(g_out,
                " uplinks=%u cmd_forwarded=%u il_tx_dropped=%u il_wrong_type=%u snaps_accepted=%u replies=%u "
                "replies_superseded=%u replies_refused=%u replies_expired=%u stats_sent=%u stats_piggyback=%u "
                "stats_quiet=%u il_rx_bytes=%u il_rx_overflow=%u il_rx_errors=%u il_bad_crc=%u il_bad_len=%u "
                "il_bad_type=%u il_skipped=%u radio_rx_overflow=%u radio_rx_errors=%u radio_tx_frames=%u "
                "radio_tx_timeouts=%u telem_bad=%u telem_out_of_window=%u telem_unmatched=%u nonce_sent=%u events_lost=%u\n",
                c->uplinks, c->cmd_forwarded, c->il_tx_dropped, c->il_wrong_type, c->snaps_accepted, c->replies,
                c->replies_superseded, c->replies_refused, c->replies_expired, c->stats_sent, c->stats_piggyback,
                c->stats_quiet, c->il_rx_bytes, c->il_rx_overflow, c->il_rx_errors, c->il_bad_crc, c->il_bad_len,
                c->il_bad_type, c->il_skipped, c->radio_rx_overflow, c->radio_rx_errors, c->radio_tx_frames,
                c->radio_tx_timeouts, c->telem_bad, c->telem_out_of_window, c->telem_unmatched, c->nonce_sent,
                c->events_lost);
        fprintf(g_out, "CS %d", ep);
        for (k = 0; k < COM_COSIM_SECONDS; k++) fprintf(g_out, " %u", r->pass_max_s_us[k]);
        fputc('\n', g_out);
    }
}

/* ------------------------------------------------------------------ resets, the fault schedule */
static uint64_t g_fault_on, g_fault_off, g_max_cyc;
static int g_gps_cut;                        /* gps_cut: the GNS's receiver silent (g_cosim_gps.cut) */
static int g_fault_state;                    /* 0 before, 1 active, 2 over */

/* the RESET pin pulled low: what simavr 1.8's avr_reset() does (I/O registers cleared, SP = RAMEND, PC = the reset
 * vector, SREG = 0, interrupt and cycle-timer tables reset, the core's and every peripheral's reset) except the watchdog
 * module's reset, which crashes (access violation) once the firmware has enabled the watchdog - the firmwares program
 * their watchdog again at boot (WDTCR = 0 after the clear: off until then, as on the chip).  RAM is kept, as on the chip;
 * the firmware's startup code repaints the stack and clears .bss.  The core does not run until t_release (global). */
static void core_hold_reset(core_t *c, uint64_t t_release)
{
    avr_t *a = c->avr;
    avr_io_t *p;
    int i;
    c->held = 1;                             /* first: the hooks ignore what the peripheral resets raise */
    a->state = cpu_Running;
    for (i = 0x20; i <= a->ioend; i++) a->data[i] = 0;
    a->data[D_SPL] = (uint8_t)(a->ramend & 0xFFu);
    a->data[D_SPH] = (uint8_t)(a->ramend >> 8);
    a->pc = a->reset_pc;
    for (i = 0; i < 8; i++) a->sreg[i] = 0;
    avr_interrupt_reset(a);
    avr_cycle_timer_reset(a);
    if (a->reset) a->reset(a);
    for (p = a->io_port; p; p = p->next)
        if (p->reset && !(p->kind && !strcmp(p->kind, "watchdog"))) p->reset(p);
    c->state = cpu_Running;
    avr_cycle_timer_register(a, (avr_cycle_count_t)g_step, step_timer, NULL);
    no_stdio(c);
    c->uart_want[0] = c->uart_want[1] = 0;
    c->depth = 0;
    c->win = 0;
    memset(c->isr_in, 0, sizeof c->isr_in);
    c->off = (int64_t)t_release - (int64_t)a->cycle;     /* frozen until the release */
    c->t_start = t_release;
}

static void core_release(core_t *c, uint64_t t, const char *name)
{
    c->held = 0;
    c->epoch++;
    c->avr->data[D_MCUCSR] |= MCUCSR_EXTRF;  /* the RESET pin was held low: an external reset */
    fprintf(g_out, "E %llu %s_release mcucsr=0x%02x\n", (unsigned long long)t, name, c->avr->data[D_MCUCSR]);
}

static void nav_reset(uint64_t t, uint64_t t_release)
{
    fcu_ss_set(1);                           /* PB0 no longer driven: the FCU's pull-up */
    slave_abort();
    x_close(t, F_NL_RESET);
    g_nav_byte_live = 0;
    core_hold_reset(&N, t_release);
    g_npa = 0;
    g_ntick = 0;
    g_ntick_have = 0;
    N.tccr0_prev = 0;
    if (g_twi_on) twi_models_reset();
    fprintf(g_out, "E %llu nav_reset cycle_after_reset=%llu resume_at=%llu\n", (unsigned long long)t,
            (unsigned long long)N.avr->cycle, (unsigned long long)t_release);
}

static void scenario(uint64_t t)
{
    if (g_fault_state == 0 && t >= g_fault_on) {
        g_fault_state = 1;
        fprintf(g_out, "E %llu fault_on %s\n", (unsigned long long)t, g_fault);
        if (fault_is("cut")) {
            g_cut = 1;
        } else if (fault_is("babble")) {
            g_babble_on = 1;
            avr_cycle_timer_register(F.avr, 1, babble_pump, NULL);
        } else if (fault_is("reset")) {
            com_snapshot();                  /* the epoch's last state */
            core_hold_reset(&C, g_fault_off);
            if (g_ep + 1 < MAXEP) g_ep++;
            fprintf(g_out, "E %llu com_reset cycle_after_reset=%llu resume_at=%llu\n", (unsigned long long)t,
                    (unsigned long long)C.avr->cycle, (unsigned long long)g_fault_off);
        } else if (fault_is("nav_hold")) {
            g_fault_off = g_max_cyc + 1u;    /* to the end */
            nav_reset(t, g_fault_off);
        } else if (fault_is("nav_pulse")) {
            g_nav_reset_armed = 1;           /* nav_spdr_write: at byte nav_reset_byte of the next exchange */
            g_fault_off = ~0ull;
        } else if (fault_is("nl_lose_cmp")) {
            g_lose_armed = 1;                /* nav_spdr_write: at byte nl_lose_byte of the next exchange */
            g_fault_off = ~0ull;             /* (the window closes when it fired: nav_spdr_write) */
        } else if (fault_is("gns_hold")) {
            g_fault_off = g_max_cyc + 1u;
            core_hold_reset(&G, g_fault_off);
            G.tccr0_prev = 0;
            fprintf(g_out, "E %llu gns_reset hold\n", (unsigned long long)t);
        } else if (fault_is("gns_pulse")) {
            core_hold_reset(&G, g_fault_off);
            G.tccr0_prev = 0;
            fprintf(g_out, "E %llu gns_reset resume_at=%llu\n", (unsigned long long)t, (unsigned long long)g_fault_off);
        } else if (fault_is("gps_cut")) {
            g_gps_cut = 1;
        } else if (!strncmp(g_fault, "nl_", 3)) {
            g_nl_fault_on = 1;
        }
    } else if (g_fault_state == 1 && t >= g_fault_off) {
        g_fault_state = 2;
        fprintf(g_out, "E %llu fault_off %s\n", (unsigned long long)t, g_fault);
        g_cut = 0;
        g_babble_on = 0;
        g_nl_fault_on = 0;
        g_lose_armed = 0;
        g_gps_cut = 0;
        if (C.held) core_release(&C, t, "com");
        if (N.held) core_release(&N, t, "nav");
        if (G.held) core_release(&G, t, "gns");
    }
}

/* ------------------------------------------------------------------ a core's own watchdog reset (S1c) */
/* simavr 1.8: the watchdog timer swaps avr->run for a callback that runs avr_reset(), which crashes in the watchdog
 * module's reset once the firmware has enabled the watchdog (the SPEC row 8 crash).  After every instruction the main
 * loop compares avr->run with the core's own; on a change the harness resets the core as core_hold_reset does (RAM kept,
 * I/O cleared, the watchdog module left alone) and releases it at once with MCUCSR.WDRF - the chip's watchdog reset. */
static void core_wdt_reset(core_t *c)
{
    static const char *const name[4] = {"fcu", "com", "nav", "gns"};
    const uint64_t t = gnow(c);
    c->avr->run = c->run0;
    c->wdt_resets++;
    if (c == &N) {
        fcu_ss_set(1);
        slave_abort();
        x_close(t, F_NL_RESET);
        g_nav_byte_live = 0;
    } else if (c == &C) {
        com_snapshot();
    }
    core_hold_reset(c, t);
    if (c == &N) {
        g_npa = 0;
        g_ntick = 0;
        g_ntick_have = 0;
        if (g_twi_on) twi_models_reset();
    } else if (c == &C) {
        if (g_ep + 1 < MAXEP) g_ep++;
    } else if (c == &F) {
        g_pa = 0;
    }
    c->held = 0;
    c->epoch++;
    c->tccr0_prev = 0;
    c->avr->data[D_MCUCSR] |= MCUCSR_WDRF;
    fprintf(g_out, "E %llu %s_wdt_reset mcucsr=0x%02x\n", (unsigned long long)t, name[c->idx & 3],
            c->avr->data[D_MCUCSR]);
}

/* ------------------------------------------------------------------ Timer0's preload (S1b) */
/* chained after simavr's TCCR0 handler: when the clock goes on, replay the TCNT0 the firmware preloaded through simavr's
 * own TCNT0 write handler (it re-bases a running timer; a stopped one ignores the value) */
static void tccr0_write_hook(struct avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    core_t *c = core_by_idx((unsigned)(uintptr_t)param);
    (void)addr;
    if (g_t0fix && (v & 7u) && !(c->tccr0_prev & 7u) && !c->held) {
        const uint8_t pre = avr->data[D_TCNT0];
        const avr_io_addr_t a = AVR_DATA_TO_IO(D_TCNT0);
        if (pre && avr->io[a].w.c) {
            avr->io[a].w.c(avr, D_TCNT0, pre, avr->io[a].w.param);
            c->t0fix++;
            fprintf(g_out, "E %llu %c_t0_preload tcnt0=%u\n", (unsigned long long)gnow(c), c->tag, pre);
        }
    }
    c->tccr0_prev = v;
}

/* ------------------------------------------------------------------ the plant (S1a, plant4.c) */
static plant_t g_pl;
static uint64_t g_pl_next, g_pl_cyc, g_pl_log_next, g_pl_log_cyc, g_gq_next, g_gps_writes;
static uint8_t g_gps_seq;

static double pwm_pulse_us(unsigned ch)              /* the FCU's compare register, us (FC_PWM_COUNTS_PER_US = 2) */
{
    const uint32_t hi = g_pwm_addr[ch];
    const unsigned v = (unsigned)F.avr->data[hi - 1u] | ((unsigned)F.avr->data[hi] << 8);
    return (double)v * 0.5;
}

static void put_le32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)((uint32_t)v >> 8);
    p[2] = (uint8_t)((uint32_t)v >> 16);
    p[3] = (uint8_t)((uint32_t)v >> 24);
}

/* the GNS's g_cosim_gps (gns_simbench.h): magic, seq (+1 per write), cut, north / east mm, v_north / v_east mm/s, up mm */
static void gps_block_write(void)
{
    double n, e, vn, ve;
    uint8_t *b;
    if (!g_rd_gps.n || G.held) return;
    b = G.avr->data + g_rd_gps.a[0];
    plant_gps(&g_pl, &g_pp, &n, &e, &vn, &ve);
    b[0] = (uint8_t)(GPS_MAGIC & 0xFFu);
    b[1] = (uint8_t)(GPS_MAGIC >> 8);
    b[2] = ++g_gps_seq;
    b[3] = (uint8_t)(g_gps_cut ? 1u : 0u);
    put_le32(b + 4, (int32_t)lround(n * 1000.0));
    put_le32(b + 8, (int32_t)lround(e * 1000.0));
    put_le32(b + 12, (int32_t)lround(vn * 1000.0));
    put_le32(b + 16, (int32_t)lround(ve * 1000.0));
    put_le32(b + 20, (int32_t)lround(g_pl.h * 1000.0));
    g_gps_writes++;
}

static void plant_tick(uint64_t t)
{
    double pulse[5], gyro[3], accel[3], p_pa;
    unsigned i;
    for (i = 0; i < 5u; i++) pulse[i] = pwm_pulse_us((unsigned)g_pwm_map[i]);
    plant_step(&g_pl, &g_pp, pulse, g_pl_dt_us * 1e-6);
    plant_sensors(&g_pl, &g_pp, gyro, accel, &p_pa);
    if (g_twi_on) twi_set_truth(gyro, accel, p_pa);
    gps_block_write();
    if (t >= g_pl_log_next) {
        double r, p, y;
        g_pl_log_next = t + g_pl_log_cyc;
        plant_euler(&g_pl, &r, &p, &y);
        fprintf(g_out, "P %llu %.4f %.4f %.4f %.4f %.4f %.4f %.3f %.3f %.3f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %d "
                       "%.3f\n", (unsigned long long)t, g_pl.h, g_pl.vz, g_pl.x, g_pl.y, g_pl.vx, g_pl.vy, r * 57.29577951,
                p * 57.29577951, y * 57.29577951, g_pl.w[0], g_pl.w[1], g_pl.w[2], g_pl.u_cmd, g_pl.u_lag, g_pl.delta[0],
                g_pl.delta[1], g_pl.delta[2], g_pl.delta[3], g_pl.airborne, plant_tilt_deg(&g_pl));
    }
}

static void gq_dump(uint64_t t)
{
    unsigned k;
    if (!g_rd_gnsrep.n || G.held) return;
    fprintf(g_out, "GQ %llu ", (unsigned long long)t);
    for (k = 0; k < g_rd_gnsrep.len[0]; k++) fprintf(g_out, "%02x", G.avr->data[g_rd_gnsrep.a[0] + k]);
    fputc('\n', g_out);
}

/* ------------------------------------------------------------------ crash handler (S1c hardening) */
static void on_crash(int sig)
{
    core_t *c = g_run;
    fprintf(stderr, "cosim4: crash (signal %d) in core %c at t=%llu pc=0x%x\n", sig, c ? c->tag : '-',
            c ? (unsigned long long)gnow(c) : 0ull, c ? (unsigned)c->avr->pc : 0u);
    if (g_out) {
        fprintf(g_out, "\nE %llu crash core=%c pc=0x%x signal=%d\n", c ? (unsigned long long)gnow(c) : 0ull,
                c ? c->tag : '-', c ? (unsigned)c->avr->pc : 0u, sig);
        fflush(g_out);
    }
    _Exit(3);
}

/* ------------------------------------------------------------------ main */
static void core_init(core_t *c, char tag, int idx, const char *hex, uint64_t start)
{
    memset(c, 0, sizeof *c);
    c->tag = tag;
    c->idx = idx;
    c->avr = avr_make_mcu_by_name("atmega128");
    if (!c->avr) die("simavr has no atmega128 core");
    avr_init(c->avr);
    c->avr->frequency = (uint32_t)F_CPU_HZ;
    load_hex(c, hex);
    no_stdio(c);
    find_uarts(c);
    c->sp_min = c->sp_min_run = 0xFFFFu;
    c->state = cpu_Running;
    c->off = (int64_t)start;
    c->t_start = start;
    avr_cycle_timer_register(c->avr, (avr_cycle_count_t)g_step, step_timer, NULL);
    hook_isrs(c);
    c->run0 = c->avr->run;
}

int main(int argc, char **argv)
{
    uint64_t next_snap = 0, snap_cyc;
    unsigned v;
    int i;
    static char obuf[1 << 20];
    signal(SIGSEGV, on_crash);
    parse_args(argc, argv);
    g_rng = g_seed;
    g_out = fopen(g_out_path, "wb");
    if (!g_out) die("cannot write %s", g_out_path);
    setvbuf(g_out, obuf, _IOFBF, sizeof obuf);
    fprintf(g_out, "H");
    for (i = 1; i < argc; i++) fprintf(g_out, " %s", argv[i]);
    fputc('\n', g_out);
    avr_global_logger_set(quiet_logger);
    core_init(&F, 'F', 0, g_fcu_hex, 0);
    core_init(&C, 'C', 1, g_com_hex, (uint64_t)(g_delay_us * (double)CYC_PER_US + 0.5));
    core_init(&N, 'N', 2, g_nav_hex, (uint64_t)(g_nav_delay_us * (double)CYC_PER_US + 0.5));
    core_init(&G, 'G', 3, g_gns_hex, (uint64_t)(g_gns_delay_us * (double)CYC_PER_US + 0.5));
    g_fault_on = !fault_is("none") ? (uint64_t)(g_fault_at_ms * 1000.0 * (double)CYC_PER_US + 0.5) : ~0ull;
    g_fault_off = g_fault_on == ~0ull ? ~0ull : g_fault_on + (uint64_t)(g_fault_len_ms * 1000.0 * (double)CYC_PER_US + 0.5);
    g_max_cyc = (uint64_t)(g_seconds * (double)F_CPU_HZ);
    snap_cyc = (uint64_t)(g_snap_us * (double)CYC_PER_US);
    if (snap_cyc < 1000u) snap_cyc = 1000u;
    fprintf(g_out, "E %lld com_start\n", (long long)C.off);
    fprintf(g_out, "E %lld nav_start\n", (long long)N.off);

    /* FCU: ilink USART1, log USART0, tick marker, nlink slave (SPDR writes, PB0 driven by the NAV), PWM registers */
    g_fcu_in = avr_io_getirq(F.avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_INPUT);
    avr_irq_register_notify(avr_io_getirq(F.avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_OUTPUT), fcu_tx1_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(F.avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), fcu_tx0_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(F.avr, AVR_IOCTL_IOPORT_GETIRQ('A'), IOPORT_IRQ_REG_PORT), porta_hook, NULL);
    {
        avr_irq_t *rx = avr_get_interrupt_irq(F.avr, V_IL_RX);
        if (!rx) die("FCU: USART1 RX vector irq missing");
        avr_irq_register_notify(rx + AVR_INT_IRQ_PENDING, fcu_rxc_pending_hook, NULL);
    }
    avr_register_io_write(F.avr, D_SPDR, fcu_spdr_write, NULL);
    g_ss = 0;
    fcu_ss_set(1);                           /* idle: the FCU's pull-up (no NAV output yet) */
    g_ss_edges = 0;
    for (v = 0; v < g_pwm_n; v++) avr_register_io_write(F.avr, (avr_io_addr_t)g_pwm_addr[v], pwm_write_hook, (void *)(uintptr_t)v);
    /* COM */
    g_com_in = avr_io_getirq(C.avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
    avr_irq_register_notify(avr_io_getirq(C.avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), com_tx0_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(C.avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_OUTPUT), com_tx1_hook, NULL);
    /* NAV: SPI master, PB0, console, tick marker, Timer2 / Timer0 compares */
    avr_register_io_write(N.avr, D_SPDR, nav_spdr_write, NULL);
    avr_irq_register_notify(avr_io_getirq(N.avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT), nav_spi_out_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(N.avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 0), nav_pb0_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(N.avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), nav_tx0_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(N.avr, AVR_IOCTL_IOPORT_GETIRQ('A'), IOPORT_IRQ_REG_PORT), nav_porta_hook, NULL);
    {
        avr_irq_t *t2 = avr_get_interrupt_irq(N.avr, V_T2_COMP), *t0 = avr_get_interrupt_irq(N.avr, V_T0_COMP);
        if (!t2 || !t0) die("NAV: TIMER2 / TIMER0 COMP vector irq missing");
        avr_irq_register_notify(t2 + AVR_INT_IRQ_PENDING, nav_t2_pending_hook, NULL);
        avr_irq_register_notify(t0 + AVR_INT_IRQ_PENDING, nav_t0_pending_hook, NULL);
    }

    /* v0.8.1 GNS: the glink USART0 both ways, the NAV's USART1 */
    g_gns_in = avr_io_getirq(G.avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
    g_nav1_in = avr_io_getirq(N.avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_INPUT);
    avr_irq_register_notify(avr_io_getirq(N.avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_OUTPUT), nav_tx1_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(G.avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), gns_tx0_hook, NULL);
    fprintf(g_out, "E %lld gns_start\n", (long long)G.off);
    /* v0.9: Timer0's preload (NAV, GNS), the NAV's TWI sensors, the plant */
    avr_register_io_write(N.avr, D_TCCR0, tccr0_write_hook, (void *)(uintptr_t)2u);
    avr_register_io_write(G.avr, D_TCCR0, tccr0_write_hook, (void *)(uintptr_t)3u);
    if (g_twi_on) twi_models_init(N.avr, g_twifix, g_seed, g_pp.gyro_noise, g_pp.accel_noise, g_pp.baro_noise_pa);
    g_pp.seed = g_seed;
    plant_init(&g_pl, &g_pp);
    g_pl_cyc = (uint64_t)(g_pl_dt_us * (double)CYC_PER_US + 0.5);
    g_pl_log_cyc = (uint64_t)(g_pl_log_ms * 1000.0 * (double)CYC_PER_US + 0.5);
    g_pl_next = 0;
    if (g_plant_on) {
        double gyro[3], accel[3], p_pa;
        plant_sensors(&g_pl, &g_pp, gyro, accel, &p_pa);
        if (g_twi_on) twi_set_truth(gyro, accel, p_pa);
    }

    for (;;) {
        core_t *c = &F;
        uint64_t t = gnow(&F), gf;
        if (done(&F) || t >= g_max_cyc) break;
        if (!done(&N) && gnow(&N) < t) {                   /* ties: FCU first, then NAV, then COM */
            c = &N;
            t = gnow(&N);
        }
        if (!done(&C) && gnow(&C) < t) {
            c = &C;
            t = gnow(&C);
        }
        if (!done(&G) && gnow(&G) < t) {
            c = &G;
            t = gnow(&G);
        }
        if (g_fault_state < 2 && t >= g_fault_on) scenario(t);
        while (g_plant_on && t >= g_pl_next) {             /* the plant at its own step, in global time */
            plant_tick(g_pl_next);
            g_pl_next += g_pl_cyc;
        }
        if (t >= g_gq_next) {
            gq_dump(t);
            g_gq_next = t + 250000ull * CYC_PER_US;
        }
        if (c->held) continue;                             /* (its clock stands at the release: not reached) */
        g_run = c;
        c->state = avr_run(c->avr);
        g_run = 0;
        if (c->avr->run != c->run0) core_wdt_reset(c);     /* its watchdog fired (simavr would crash in avr_reset) */
        after_step(c);
        if (c == &N && g_nav_reset_req) {
            const uint64_t tn = gnow(&N);
            g_nav_reset_req = 0;
            g_fault_off = tn + (uint64_t)(g_fault_len_ms * 1000.0 * (double)CYC_PER_US + 0.5);
            nav_reset(tn, g_fault_off);
        }
        if (c == &F) {
            gf = gnow(&F);
            if (gf >= next_snap) {
                next_snap = gf + snap_cyc;
                if (!C.held && C.avr->cycle > 0u) com_snapshot();
            }
        }
    }
    if (!C.held) com_snapshot();
    x_close(gnow(&N), 0);
    fprintf(g_out, "E %llu end fcu_state=%d com_state=%d nav_state=%d gns_state=%d\n", (unsigned long long)gnow(&F), F.state,
            C.state, N.state, G.state);
    com_dump();
    for (i = 0; i < 4; i++) {
        core_t *c = core_by_idx((unsigned)i);
        for (v = 0; v < NVEC; v++)
            if (c->isr_n[v])
                fprintf(g_out, "I %c %u %llu %llu %llu %llu\n", c->tag, v, (unsigned long long)c->isr_n[v],
                        (unsigned long long)c->isr_tot[v], (unsigned long long)c->isr_min[v],
                        (unsigned long long)c->isr_max[v]);
    }
    fprintf(g_out,
            "M fcu_state=%d com_state=%d nav_state=%d fcu_cycle=%llu com_cycle=%llu nav_cycle=%llu com_off=%lld "
            "nav_off=%lld fcu_sp_min=%u com_sp_min=%u nav_sp_min=%u fcu_sp_min_run=%u com_sp_min_run=%u nav_sp_min_run=%u "
            "ticks_seen=%lu last_tick=%lu marker_jumps=%lu nav_ticks_seen=%lu nav_last_tick=%lu nav_marker_jumps=%lu "
            "nav_epochs=%d c2f_bytes=%llu f2c_bytes=%llu c2f_cut=%llu f2c_cut=%llu "
            "c2f_babbled=%llu c2f_rxoff=%llu babble_bytes=%llu rx_delivered=%llu rx_unmatched=%llu rx_queue_left=%lu "
            "rx_queue_overflow=%llu com_u1_bytes=%llu console_bytes=%llu nav_console_bytes=%llu com_epochs=%d "
            "com_snapshots=%llu com_snapshots_odd=%llu fcu_uart1_cpb=%llu fcu_uart0_cpb=%llu com_uart0_cpb=%llu "
            "com_uart1_cpb=%llu nav_uart0_cpb=%llu exchanges=%llu spi_bytes=%llu spi_late=%llu spi_late_hdr=%llu "
            "spi_wcol=%llu spi_dropped=%llu spi_extra=%llu spi_flips=%llu spi_aborted=%llu spi_ss_high_bytes=%llu "
            "spi_out_mismatch=%llu spi_dly_max=%llu ss_edges=%llu isr_resp=%d\n",
            F.state, C.state, N.state, (unsigned long long)F.avr->cycle, (unsigned long long)C.avr->cycle,
            (unsigned long long)N.avr->cycle, (long long)C.off, (long long)N.off, F.sp_min, C.sp_min, N.sp_min,
            F.sp_min_run, C.sp_min_run, N.sp_min_run, (unsigned long)g_ticks_seen, (unsigned long)g_tick_now,
            (unsigned long)g_marker_jumps, (unsigned long)g_nticks_seen, (unsigned long)g_ntick,
            (unsigned long)g_nmarker_jumps, N.epoch + 1, (unsigned long long)g_c2f_bytes, (unsigned long long)g_f2c_bytes,
            (unsigned long long)g_c2f_cut, (unsigned long long)g_f2c_cut, (unsigned long long)g_c2f_babbled,
            (unsigned long long)g_c2f_rxoff, (unsigned long long)g_bab_bytes, (unsigned long long)g_r_index,
            (unsigned long long)g_rxc_unmatched, (unsigned long)(g_rq_head - g_rq_tail), (unsigned long long)g_rq_overflow,
            (unsigned long long)g_u1_bytes, (unsigned long long)g_fcon.bytes, (unsigned long long)g_ncon.bytes, g_ep + 1,
            (unsigned long long)g_snaps_taken, (unsigned long long)g_snaps_odd,
            (unsigned long long)F.uart[1]->cycles_per_byte, (unsigned long long)F.uart[0]->cycles_per_byte,
            (unsigned long long)C.uart[0]->cycles_per_byte, (unsigned long long)C.uart[1]->cycles_per_byte,
            (unsigned long long)N.uart[0]->cycles_per_byte, (unsigned long long)g_x_n, (unsigned long long)g_spi_bytes,
            (unsigned long long)g_spi_late, (unsigned long long)g_spi_late_hdr, (unsigned long long)g_spi_wcol,
            (unsigned long long)g_spi_dropped, (unsigned long long)g_spi_extra, (unsigned long long)g_spi_flips,
            (unsigned long long)g_spi_aborted, (unsigned long long)g_spi_ss_high_bytes,
            (unsigned long long)g_spi_out_mismatch, (unsigned long long)g_spi_dly_max, (unsigned long long)g_ss_edges,
            g_isr_resp);
    fprintf(g_out, "MG gns_state=%d gns_cycle=%llu gns_off=%lld gns_sp_min=%u gns_sp_min_run=%u n2g_bytes=%llu "
            "g2n_bytes=%llu nav_uart1_cpb=%llu gns_uart0_cpb=%llu gns_uart1_cpb=%llu\n", G.state,
            (unsigned long long)G.avr->cycle, (long long)G.off, G.sp_min, G.sp_min_run, (unsigned long long)g_n2g_bytes,
            (unsigned long long)g_g2n_bytes, (unsigned long long)N.uart[1]->cycles_per_byte,
            (unsigned long long)G.uart[0]->cycles_per_byte, (unsigned long long)G.uart[1]->cycles_per_byte);
    {
        const twi_stats_t *ts = twi_models_stats();
        fprintf(g_out, "MX wdt_fcu=%u wdt_com=%u wdt_nav=%u wdt_gns=%u t0fix_nav=%u t0fix_gns=%u nl_lose=%llu nl_lose_x=%llu "
                       "twi=%d twifix=%d twi_msgs=%llu twi_reads=%llu bursts=%llu burst_reads=%llu ms_cmds=%llu d1=%llu "
                       "d2=%llu adc_reads=%llu adc_early=%llu prom_reads=%llu twi_rescaled=%llu twi_not_found=%llu "
                       "scl_cycles_per_bit=%llu twi_bits_hist=",
                F.wdt_resets, C.wdt_resets, N.wdt_resets, G.wdt_resets, N.t0fix, G.t0fix, (unsigned long long)g_lose_done,
                (unsigned long long)g_lose_x, g_twi_on, g_twifix, (unsigned long long)ts->msgs,
                (unsigned long long)ts->reads, (unsigned long long)ts->bursts, (unsigned long long)ts->burst_reads,
                (unsigned long long)ts->ms_cmds, (unsigned long long)ts->d1_convs, (unsigned long long)ts->d2_convs,
                (unsigned long long)ts->adc_reads, (unsigned long long)ts->adc_early, (unsigned long long)ts->prom_reads,
                (unsigned long long)ts->rescaled, (unsigned long long)ts->not_found,
                (unsigned long long)ts->scl_cycles_per_bit);
        for (v = 0; v < 16u; v++) fprintf(g_out, "%s%llu", v ? "," : "", (unsigned long long)ts->rescale_hist[v]);
        fprintf(g_out, " baro=%s bmp_forced=%llu bmp_done=%llu bmp_data_reads=%llu bmp_early=%llu bmp_calib_reads=%llu "
                       "bmp_resets=%llu bmp_busy_writes=%llu bmp_normal_writes=%llu", twi_models_baro_name(),
                (unsigned long long)ts->bmp_forced, (unsigned long long)ts->bmp_done,
                (unsigned long long)ts->bmp_data_reads, (unsigned long long)ts->bmp_early,
                (unsigned long long)ts->bmp_calib_reads, (unsigned long long)ts->bmp_resets,
                (unsigned long long)ts->bmp_busy_writes, (unsigned long long)ts->bmp_normal_writes);
        fprintf(g_out, " plant=%d h_max=%.4f td_v=%.4f touchdowns=%u liftoffs=%u tilt_max_deg=%.3f gps_writes=%llu "
                       "x=%.4f y=%.4f h=%.4f\n", g_plant_on, g_pl.h_max, g_pl.td_v, g_pl.touchdowns, g_pl.liftoffs,
                g_pl.tilt_max_deg, (unsigned long long)g_gps_writes, g_pl.x, g_pl.y, g_pl.h);
    }
    fprintf(g_out, "L");
    for (v = 0; v < NVEC; v++)
        if (g_lat_n_by_vec[v])
            fprintf(g_out, " %u:%llu:%llu:%lld", v, (unsigned long long)g_lat_n_by_vec[v],
                    (unsigned long long)g_lat_by_vec[v], (long long)g_margin_by_vec[v]);
    fputc('\n', g_out);
    fclose(g_out);
    return 0;
}
