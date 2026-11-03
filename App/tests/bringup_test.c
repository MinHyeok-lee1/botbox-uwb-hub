/**
 * @file bringup_test.c
 * @brief Entry point for the bring-up test selected by BOARD_TEST_MODE.
 */
#include "bringup_test.h"

#include <stddef.h>

#include "board_config.h"
#include "can_linktest.h"
#include "can_loadtest.h"
#include "can_monitor.h"
#include "can_selftest.h"
#include "led_test.h"
#include "rs485_test.h"
#include "uart_test.h"

void bringup_test_adjust_can_config(can_port_config_t config[CAN_PORT_COUNT])
{
#if BOARD_TEST_MODE == BOARD_TEST_CAN_LOOPBACK
  for (size_t i = 0U; i < CAN_PORT_COUNT; i++)
  {
    config[i].enabled = true;
    config[i].internal_loopback = true;
  }
#elif BOARD_TEST_MODE == BOARD_TEST_CAN_MONITOR
  config[BOARD_CAN_MONITOR_PORT].enabled = true;
#elif BOARD_TEST_MODE == BOARD_TEST_LED
  /* The CAN driver toggles the CAN LEDs on every received frame. */
  for (size_t i = 0U; i < CAN_PORT_COUNT; i++)
  {
    config[i].enabled = false;
  }
#else
  (void)config;
#endif
}

void bringup_test_init(void)
{
#if BOARD_TEST_MODE == BOARD_TEST_CAN_LOOPBACK
  can_selftest_init();
#elif BOARD_TEST_MODE == BOARD_TEST_CAN_LINK
  can_linktest_init();
#elif BOARD_TEST_MODE == BOARD_TEST_CAN_LOAD
  can_loadtest_init();
#elif BOARD_TEST_MODE == BOARD_TEST_UART
  uart_test_init();
#elif BOARD_TEST_MODE == BOARD_TEST_RS485
  rs485_test_init();
#elif BOARD_TEST_MODE == BOARD_TEST_CAN_MONITOR
  can_monitor_init();
#elif BOARD_TEST_MODE == BOARD_TEST_LED
  led_test_init();
#endif
}
