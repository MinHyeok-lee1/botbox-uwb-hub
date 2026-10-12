/**
 * @file uart_test.h
 * @brief Bring-up check for the UART5 output port.
 */
#ifndef UART_TEST_H
#define UART_TEST_H

/** Create the UART test tasks. Call once before the scheduler starts. */
void uart_test_init(void);

#endif /* UART_TEST_H */
