/* adc.c - see adc.h */
#include "adc.h"

#include <avr/io.h>

#include "../params_gen.h"


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

