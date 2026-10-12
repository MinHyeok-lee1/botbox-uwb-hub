/**
 * @file rs485_test.h
 * @brief Bring-up check for the half-duplex RS485 port.
 */
#ifndef RS485_TEST_H
#define RS485_TEST_H

/** Create the RS485 test task. Call once before the scheduler starts. */
void rs485_test_init(void);

#endif /* RS485_TEST_H */
