/* adc.c - see adc.h */
#include "adc.h"

#include <avr/io.h>

#include "../params_gen.h"

#if PM_ANY
#include <avr/interrupt.h>
#endif

void adc_init(void)
{
    ADMUX = (uint8_t)((1 << REFS1) | (1 << REFS0) | (VBAT_ADC_CHANNEL & 0x07));   /* internal 2.56 V */
    ADCSRA = (uint8_t)((1 << ADEN) | (FC_ADC_PS_BITS & 0x07));
    ADCSRA |= (1 << ADSC);                                                          /* first (discarded-quality) conversion */
}

bool adc_poll(uint16_t *counts)
{
    if (ADCSRA & (1 << ADSC))
        return false;                     /* still converting */
    *counts = ADCW;
    ADCSRA |= (1 << ADSC);
    return true;
}

uint16_t adc_entropy_seed(uint8_t n)
{
    uint16_t s = 0, w;
    uint8_t i;
    for (i = 0; i < n; i++) {
        w = 0;
        while ((ADCSRA & (1 << ADSC)) && ++w != 0u) {   /* bounded: 13 ADC clocks = 104 us at 125 kHz */
        }
        s = (uint16_t)((s << 3) | (s >> 13));
        s ^= ADCW;
        ADCSRA |= (1 << ADSC);
    }
    return s;
}

#if PM_ANY
/* the chain: PM_CH_IN, then (wired_buffer) PM_CH_TET, PM_CH_BUF */
#if PM_BUF
#define ADC_CHAIN_N 3u
#define ADC_MUX_OF(i) ((i) == 0u ? VBAT_ADC_CHANNEL : ((i) == 1u ? PWR_V_TETHER_ADC_CHANNEL : PWR_V_BUFFER_ADC_CHANNEL))
#else
#define ADC_CHAIN_N 1u
#define ADC_MUX_OF(i) VBAT_ADC_CHANNEL
#endif
#define ADC_REFS ((uint8_t)((1 << REFS1) | (1 << REFS0)))   /* internal 2.56 V (see the invariant in adc.h) */
static volatile uint16_t s_code[ADC_CHAIN_N];
static volatile uint8_t s_idx;                        /* the channel converting; ADC_CHAIN_N = the chain is done */

void adc_chain_init(void)
{
    uint8_t i;
    for (i = 0; i < ADC_CHAIN_N; i++)
        s_code[i] = PM_CODE_NONE;
    while (ADCSRA & (1 << ADSC)) {                    /* the boot entropy conversions' last one (<= 104 us) */
    }
    s_idx = ADC_CHAIN_N;                              /* nothing running: the first take returns NONE */
    ADCSRA |= (1 << ADIF);                            /* clear a stale flag; the chain enables the interrupt */
}

/* The chain interrupt.  fw09 review F2: it may meet an nlink exchange (a resuming NAV's first exchange on the FCU's own
 * clock, adc.h), so it keeps the limit of every FCU interrupt that can run during one: <= 58 cycles incl. the 7-cycle entry
 * (hal/nlink_spi.h Timing; tools/run_simbench.py EXCHANGE_ISRS checks the image).  The indexed store of the first version
 * (s_code[i]: the Z pointer, RAMPZ saved, 79 / 85 cycles) is replaced by one branch per chain position with constant
 * addresses.  Same behaviour in every reachable state: the interrupt is enabled only by adc_chain_take (s_idx = 0) and
 * disabled by the conversion of the last position, so s_idx is 0 .. ADC_CHAIN_N - 1 here; position i stores its code in
 * s_code[i], starts the conversion of position i + 1 (ADC_MUX_OF) or ends the chain, and sets s_idx = i + 1.
 * tools/adc_chain_test runs this chain in simavr with three distinct input voltages. */
#if PM_BUF
ISR(ADC_vect)
{
    const uint8_t i = s_idx;
    if (i == 0u) {
        s_code[0] = ADCW;
        ADMUX = (uint8_t)(ADC_REFS | (ADC_MUX_OF(1u) & 0x07u));
        ADCSRA |= (1 << ADSC);
        s_idx = 1u;
    } else if (i == 1u) {
        s_code[1] = ADCW;
        ADMUX = (uint8_t)(ADC_REFS | (ADC_MUX_OF(2u) & 0x07u));
        ADCSRA |= (1 << ADSC);
        s_idx = 2u;
    } else {
        s_code[2] = ADCW;
        ADCSRA &= (uint8_t)~(1 << ADIE);
        s_idx = 3u;
    }
}
#else
ISR(ADC_vect)
{
    s_code[0] = ADCW;                                 /* the only position (s_idx == 0) */
    ADCSRA &= (uint8_t)~(1 << ADIE);
    s_idx = 1u;
}
#endif

void adc_chain_take(uint16_t codes[3])
{
    uint8_t i;
    const uint8_t done = s_idx;
    codes[PM_CH_IN] = codes[PM_CH_TET] = codes[PM_CH_BUF] = PM_CODE_NONE;
    if (done < ADC_CHAIN_N) {                         /* the previous chain did not finish (cannot in 4 ms): stop it */
        ADCSRA &= (uint8_t)~(1 << ADIE);
        while (ADCSRA & (1 << ADSC)) {
        }
    }
    for (i = 0; i < ADC_CHAIN_N && i < done; i++)
#if PM_BUF
        codes[i] = s_code[i];                         /* PM_CH_IN, PM_CH_TET, PM_CH_BUF in chain order */
#else
        codes[PM_CH_IN] = s_code[i];
#endif
    s_idx = 0;
    ADMUX = (uint8_t)(ADC_REFS | (VBAT_ADC_CHANNEL & 0x07u));
    ADCSRA |= (uint8_t)((1 << ADIF) | (1 << ADIE));
    ADCSRA |= (1 << ADSC);
}

uint16_t adc_chain_vin(void)
{
    return s_idx >= 1u ? s_code[0] : PM_CODE_NONE;     /* V_in of the running chain (converted first, ~110 us in) */
}
#endif
