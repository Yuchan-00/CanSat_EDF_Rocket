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

#include "../fc_mode.h"

/* 2026-11-10 control round: the logic-battery monitor (FCU PF3, VTOL carrier rev 2c: 1S pack through 47k / 47k) is read by
 * the VTOL battery build (FC_VCFG 0, no power monitor) when params_gen.h LOGIC_BAT_TELEM is 1; the rocket FCU (no
 * divider) and the tether / hybrid builds (their own ADC chain) do not.  One PF3 conversion every LOGIC_BAT_DIV_TICKS
 * ticks in place of that tick's V_cell conversion (adc_poll_mux); the byte goes to the reply snapshot (main.c). */


#endif /* FC_ADC_H */
