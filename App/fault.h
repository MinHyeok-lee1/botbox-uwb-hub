/**
 * @file fault.h
 * @brief Last-resort handling for unrecoverable faults.
 */
#ifndef FAULT_H
#define FAULT_H

typedef enum
{
  FAULT_HARD_FAULT,
  FAULT_STACK_OVERFLOW,
  FAULT_MALLOC_FAILED,
  FAULT_INIT_FAILED,
} fault_code_t;

/**
 * Report the fault on the debug UART and halt, blinking all LEDs.
 * Safe to call from any context, including fault handlers. Never returns.
 *
 * @param detail Optional extra text such as a task name, or NULL.
 */
void fault_panic(fault_code_t code, const char *detail) __attribute__((noreturn));

#endif /* FAULT_H */
