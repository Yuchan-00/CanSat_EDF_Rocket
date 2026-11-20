/* watchdog.c - see watchdog.h */
#include "watchdog.h"

#include <avr/io.h>
#include <avr/wdt.h>

#include "../core/onboard.h"
#include "../fc_mode.h"

#define RECORD_MAGIC 0xC5A7u

typedef struct {
    uint16_t magic;
    uint8_t state;
    uint8_t check;               /* ~state */
} flight_record_t;

static uint8_t s_reset_cause __attribute__((section(".noinit")));
static flight_record_t s_record __attribute__((section(".noinit")));

void watchdog_early_init(void) __attribute__((naked, used, section(".init3")));
void watchdog_early_init(void)
{
    s_reset_cause = MCUCSR & 0x1F;
    MCUCSR = 0;
    wdt_disable();
}

uint8_t watchdog_reset_cause(void)
{
    return s_reset_cause;
}

void watchdog_start(void)
{
    wdt_enable(WDT_TIMEOUT);
}

void watchdog_kick(void)
{
    wdt_reset();
}

void flight_record_store(uint8_t state)
{
    s_record.magic = RECORD_MAGIC;
    s_record.state = state;
    s_record.check = (uint8_t)~state;
}

uint8_t flight_record_state(void)
{
    return s_record.state;
}

bool flight_record_inflight_reset(void)
{
    bool warm = (s_reset_cause & (RESET_WATCHDOG | RESET_BROWN_OUT)) && !(s_reset_cause & RESET_POWER_ON);
    bool valid = s_record.magic == RECORD_MAGIC && (uint8_t)(s_record.check ^ s_record.state) == 0xFFu;
    /* POWERED, UNPOWERED, ABORT, RELEASED (v0.6.3; core/onboard.h onboard_flying) */
    return warm && valid && onboard_flying(s_record.state);
}
