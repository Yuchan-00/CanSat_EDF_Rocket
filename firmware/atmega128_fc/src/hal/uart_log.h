/* uart_log.h - transmit-only USART0 log port (FC_LOG_BAUD 8N1 -> OpenLog) with an
 * interrupt-driven ring buffer.
 *
 * Invariant: logging can never stall the control loop - uart_log_write either
 * queues a whole frame or drops it (counted), it never waits for the UART.
 */
#ifndef FC_UART_LOG_H
#define FC_UART_LOG_H

#include <stdbool.h>
#include <stdint.h>

#include "../fc_mode.h"

#ifndef UART_LOG_BUF
#if defined(FC_SIMBENCH) && defined(FC_ROLE_FCU) && FC_VTOL
#define UART_LOG_BUF 128u                 /* v0.8: the FCU simbench's console (its flight records are packed and framed, not
                                           * sent; text records <= 102 B, report lines < 128 B): 128 bytes of RAM for the
                                           * v0.8 VTOL FCU beside the synthetic peers.  The flight builds and the ballistic
                                           * simbench (v0.7.1's console) keep 256. */
#else
#define UART_LOG_BUF 256u                 /* power of two; > 2 flight frames */
#endif
#endif

void     uart_log_init(void);
bool     uart_log_write(const uint8_t *data, uint16_t len);
uint16_t uart_log_dropped(void);
uint16_t uart_log_free(void);
/* streaming one frame into the ring without a buffer of its own (the simbench report lines): uart_log_begin(len) is true
 * when len bytes are free - then at most len bytes may be put; uart_log_put appends beyond the head (the UDRE interrupt
 * does not see them yet); uart_log_commit hands them over at once */
bool     uart_log_begin(uint16_t len);
void     uart_log_put(const uint8_t *b, uint8_t n);
void     uart_log_commit(void);

#endif /* FC_UART_LOG_H */
