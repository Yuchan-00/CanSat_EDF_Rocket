/* pwm.c - see pwm.h */
#ifdef FC_ROLE_NAV   /* fw09 W1 (integration, agent A's request): no NAV env lists this file (platformio.ini nav_common) */
#error "Timer3 belongs to the VTOL NAV nlink (hal/nlink_spi.c, W1): this driver writes Timer3"
#endif
#include "pwm.h"

#include <avr/io.h>
#include <util/atomic.h>

#include "../params_gen.h"

void pwm_write(uint8_t ch, uint16_t counts)
{
    if (counts > FC_PWM_TOP)
        counts = FC_PWM_TOP;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        switch (ch) {
        case PWM_CH_OC1A: OCR1A = counts; break;
        case PWM_CH_OC1B: OCR1B = counts; break;
        case PWM_CH_OC1C: OCR1C = counts; break;
        case PWM_CH_OC3A: OCR3A = counts; break;
        case PWM_CH_OC3B: OCR3B = counts; break;
        case PWM_CH_OC3C: OCR3C = counts; break;
        default: break;
        }
    }
}

void pwm_init(const uint16_t counts[PWM_N_CH])
{
    uint8_t ch;
    TCCR1B = 0;                     /* timers stopped while configuring */
    TCCR3B = 0;
    for (ch = 0; ch < PWM_N_CH; ch++)
        pwm_write(ch, counts[ch]);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ICR1 = FC_PWM_TOP;
        ICR3 = FC_PWM_TOP;
        TCNT1 = 0;
        TCNT3 = 0;
    }
    /* mode 14: WGMn3:0 = 1110; non-inverting outputs (COMnx1 = 1) */
    TCCR1A = (uint8_t)((1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1) | (1 << WGM11));
    TCCR3A = (uint8_t)((1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM31));
    TCCR1B = (uint8_t)((1 << WGM13) | (1 << WGM12) | (FC_PWM_CS_BITS & 0x07));
    TCCR3B = (uint8_t)((1 << WGM33) | (1 << WGM32) | (FC_PWM_CS_BITS & 0x07));
    /* only now drive the pins: PB5..PB7 (OC1A..C), PE3..PE5 (OC3A..C) */
    DDRB |= (uint8_t)((1 << PB5) | (1 << PB6) | (1 << PB7));
    DDRE |= (uint8_t)((1 << PE3) | (1 << PE4) | (1 << PE5));
}
