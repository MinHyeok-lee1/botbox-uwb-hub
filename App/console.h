/**
 * @file console.h
 * @brief Text console on the debug UART.
 *
 * Output is queued and written by the console task, so callers never block on
 * the UART. Call only from task context, never from an ISR.
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Create the console objects and task. Call before the scheduler starts. */
void console_init(void);

/** Queue a formatted message. Output that does not fit the queue is dropped. */
void console_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

/** Number of messages dropped or truncated because the queue was full. */
uint32_t console_get_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H */
