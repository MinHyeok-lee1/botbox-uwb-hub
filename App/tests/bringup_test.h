/**
 * @file bringup_test.h
 * @brief Entry point for the bring-up test selected by BOARD_TEST_MODE.
 *
 * This is the only header of App/tests that the application includes. With
 * BOARD_TEST_MODE set to BOARD_TEST_NONE both functions do nothing.
 */
#ifndef BRINGUP_TEST_H
#define BRINGUP_TEST_H

#include "can_port.h"

/**
 * Change the CAN port configuration where the selected test needs it: the
 * loopback self-test runs both ports in internal loopback.
 */
void bringup_test_adjust_can_config(can_port_config_t config[CAN_PORT_COUNT]);

/** Create the tasks of the selected test. Call once before the scheduler starts. */
void bringup_test_init(void);

#endif /* BRINGUP_TEST_H */
