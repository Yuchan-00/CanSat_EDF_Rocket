/* gpio.c - see gpio.h */
#include "gpio.h"

#include "../board.h"

void gpio_init(void)
{
    LED_DDR |= (uint8_t)(1 << PIN_LED_BIT);
    BUZZER_DDR |= (uint8_t)(1 << PIN_BUZZER_BIT);
    LED_PORT &= (uint8_t)~(1 << PIN_LED_BIT);
    BUZZER_PORT &= (uint8_t)~(1 << PIN_BUZZER_BIT);
    ARM_SW_DDR &= (uint8_t)~(1 << PIN_ARM_SW_BIT);
    ARM_SW_PORT |= (uint8_t)(1 << PIN_ARM_SW_BIT);        /* pull-up: open switch reads high = safe */
}

void gpio_led(bool on)
{
    if (on)
        LED_PORT |= (uint8_t)(1 << PIN_LED_BIT);
    else
        LED_PORT &= (uint8_t)~(1 << PIN_LED_BIT);
}

void gpio_buzzer(bool on)
{
    if (on)
        BUZZER_PORT |= (uint8_t)(1 << PIN_BUZZER_BIT);
    else
        BUZZER_PORT &= (uint8_t)~(1 << PIN_BUZZER_BIT);
}

bool gpio_arm_switch_raw(void)
{
    return (ARM_SW_PINR & (1 << PIN_ARM_SW_BIT)) == 0;
}
