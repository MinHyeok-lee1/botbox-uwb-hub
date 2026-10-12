/**
 * @file board_config.h
 * @brief Board-level assignments shared by the application modules.
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"
#include "usart.h"

/* Heartbeat LED. The board has no dedicated status LED, so the UART LED is used. */
#define BOARD_HEARTBEAT_LED_PORT LED_UART_GPIO_Port
#define BOARD_HEARTBEAT_LED_PIN  LED_UART_Pin
#define BOARD_HEARTBEAT_PERIOD_MS 500U

/* Debug console (ST-LINK virtual COM port). */
#define BOARD_CONSOLE_UART (&hlpuart1)

/* Bring-up test selection (see App/tests):
   NONE          no test; the application runs on its own
   CAN_LOOPBACK  CAN internal loopback self-test, no wiring needed
   CAN_LINK      1 frame/s exchange between two boards on the real CAN bus
   CAN_LOAD      one cycle of the two-board CAN load test
   UART          continuous output and line check on UART5
   RS485         request/reply exchange on the RS485 bus */
#define BOARD_TEST_NONE 0
#define BOARD_TEST_CAN_LOOPBACK 1
#define BOARD_TEST_CAN_LINK 2
#define BOARD_TEST_CAN_LOAD 3
#define BOARD_TEST_UART 4
#define BOARD_TEST_RS485 5
#define BOARD_TEST_MODE BOARD_TEST_RS485

/* CAN ports the application uses. The sample boards only have the CAN1
   transceiver fitted; set BOARD_CAN2_ENABLE to 1 on hardware that has both.
   The loopback self-test enables both ports regardless of these. */
#define BOARD_CAN1_ENABLE 1
#define BOARD_CAN2_ENABLE 0

/* CAN FD transmitter delay compensation. The driver only applies it when the
   data prescaler is 1 or 2, as ISO 11898-1 requires: with a prescaler of 5 every
   BRS frame failed on the board, in internal loopback and on the bus. The offset
   is derived from the data bit timing so that the secondary sample point sits at
   the data sample point after the measured loop delay. */
#define BOARD_CAN_TDC_ENABLE 1
#define BOARD_CAN_TDC_FILTER 0U

/* RS485 (USART3). The driver-enable timing is set in CubeMX. Switching the
   receiver off while transmitting keeps the port from receiving its own bytes
   on boards where the transceiver's receiver stays enabled; it is harmless
   where the transceiver already blocks the echo. */
#define BOARD_RS485_MUTE_RECEIVER_DURING_TX 1

#endif /* BOARD_CONFIG_H */
