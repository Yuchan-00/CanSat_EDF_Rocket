/* adc.h - battery voltage on ADC VBAT_ADC_CHANNEL, internal 2.56 V reference.
 *
 * Non-blocking: adc_poll() returns the result of the conversion started on
 * the previous tick (13 ADC clocks = 104 us at 125 kHz << 4 ms) and starts
 * the next one.
 * Invariant: AREF carries only its 100 nF capacitor - the internal 2.56 V
 * reference must never be selected with an external voltage on AREF.
 */
#ifndef FC_ADC_H
#define FC_ADC_H

#include <stdbool.h>
#include <stdint.h>

void adc_init(void);
bool adc_poll(uint16_t *counts);      /* true when a new 10-bit result was read */
/* boot only (blocking, ~0.1 ms per conversion): rotate-xor of n battery-channel conversions, whose
 * LSB noise seeds the LAUNCH nonce (SPEC 10.3) - not a security feature, only makes a stale LAUNCH
 * from an earlier session useless; leaves a conversion running for adc_poll */
uint16_t adc_entropy_seed(uint8_t n);

#include "../core/power_mon.h"
#include "../fc_mode.h"

/* 2026-11-10 control round: the logic-battery monitor (FCU PF3, VTOL carrier rev 2c: 1S pack through 47k / 47k) is read by
 * the VTOL battery build (FC_VCFG 0, no power monitor) when params_gen.h LOGIC_BAT_TELEM is 1; the rocket FCU (no
 * divider) and the tether / hybrid builds (their own ADC chain) do not.  One PF3 conversion every LOGIC_BAT_DIV_TICKS
 * ticks in place of that tick's V_cell conversion (adc_poll_mux); the byte goes to the reply snapshot (main.c). */
#if FC_VTOL && !PM_ANY && defined(LOGIC_BAT_TELEM) && defined(FC_VCFG) && defined(FC_VCFG_BATTERY)
#if LOGIC_BAT_TELEM && FC_VCFG == FC_VCFG_BATTERY
#define FC_LOGIC_BAT 1
#endif
#endif
#ifndef FC_LOGIC_BAT
#define FC_LOGIC_BAT 0
#endif
#if FC_LOGIC_BAT
/* adc_poll with a channel: the result of the conversion started by the previous call (*ch = its channel), then the next
 * conversion on next_ch (VBAT_ADC_CHANNEL or LOGIC_BAT_ADC_CHANNEL); false while still converting (nothing changes) */
bool     adc_poll_mux(uint16_t *counts, uint8_t *ch, uint8_t next_ch);
/* boot only (blocking, ~0.2 ms): one conversion of ch, then a V_cell conversion left running for adc_poll_mux */
uint16_t adc_read_blocking(uint8_t ch);
#endif

#if PM_ANY
/* v0.9 power builds (core/power_mon.h): one conversion chain per tick over V_in (VBAT_ADC_CHANNEL) and, in wired_buffer,
 * V_tether / V_buffer (PWR_V_*_ADC_CHANNEL; PF1 / PF2 PROPOSED - PF4-PF7 are the JTAG pins), chained in the ADC
 * interrupt (3 x 13 ADC clocks = 312 us at 125 kHz; fw09 review: started in step 5 of the tick - main.c, the FCU stack,
 * ~1.6 ms after the nlink exchange in the FCU bench -, not at the very start of the time-critical part as written before:
 * it still ends ~1.9 ms after the exchange, >= 1.1 ms before the next one can start (BG_END_US 3020 us), so on frame-led
 * ticks the ADC interrupt never runs during an exchange.  On the FCU's own fallback clock (no frames) a resuming NAV's
 * first exchange can meet it: one ADC interrupt (~80 cycles) delays the slave interrupt - at worst a bad frame or a
 * resync, counted; ~3 x 60 cycles).  adc_chain_take returns the codes of the chain started by its previous call
 * (PM_CODE_NONE for a channel the chain did not reach - an overrun: cannot happen in a 4 ms tick) and starts the next
 * chain.  adc_poll is not used in these builds. */
void     adc_chain_init(void);
void     adc_chain_take(uint16_t codes[3]);
uint16_t adc_chain_vin(void);                 /* V_in of the current chain, PM_CODE_NONE before it is converted */
#endif

#endif /* FC_ADC_H */
