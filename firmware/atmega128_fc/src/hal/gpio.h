/* gpio.h - status LED, buzzer, arm switch (pins from params_gen.h via board.h).
 * Invariant: the arm switch reads "armed" only when the pin is pulled to GND;
 * an open or broken wire reads "safe".
 */
#ifndef FC_GPIO_H
#define FC_GPIO_H

#include <stdbool.h>

void gpio_init(void);
void gpio_led(bool on);
void gpio_buzzer(bool on);
bool gpio_arm_switch_raw(void);      /* true = switch closed (pin low) */

#endif /* FC_GPIO_H */
