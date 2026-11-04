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
