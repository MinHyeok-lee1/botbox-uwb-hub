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
                 (both on BOARD_CAN_TEST_PORT)
   CAN_MONITOR   receive only on BOARD_CAN_MONITOR_PORT and print the frames;
                 transmits nothing
   UART          continuous output and line check on UART5
   RS485         request/reply exchange on the RS485 bus
   LED           the four LEDs light one after another; CAN ports stay off */
#define BOARD_TEST_NONE 0
#define BOARD_TEST_CAN_LOOPBACK 1
#define BOARD_TEST_CAN_LINK 2
#define BOARD_TEST_CAN_LOAD 3
#define BOARD_TEST_UART 4
#define BOARD_TEST_RS485 5
#define BOARD_TEST_CAN_MONITOR 6
#define BOARD_TEST_LED 7
#define BOARD_TEST_MODE BOARD_TEST_NONE

/* Port the two-board link and load tests run on (CAN_PORT_1 or CAN_PORT_2,
   from can_port.h). Both boards must use the same port, wired to each other. */
#define BOARD_CAN_TEST_PORT CAN_PORT_2

/* Port the receive-only monitor test listens on (CAN_PORT_1 or CAN_PORT_2). */
#define BOARD_CAN_MONITOR_PORT CAN_PORT_1

/* CAN ports the application uses. The early sample boards only had the CAN1
   transceiver fitted; the finished hardware has both. The loopback self-test
   enables both ports regardless of these. */
#define BOARD_CAN1_ENABLE 1
#define BOARD_CAN2_ENABLE 1

/* First CAN2 identifier of the follow result sent to the drive board: classic
   frames use this and the next two, CAN FD uses this one only. */
#define BOARD_UWB_OUTPUT_CAN_ID_BASE 0x300U

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
