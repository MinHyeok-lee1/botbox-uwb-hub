/**
 * @file uart_port.h
 * @brief Buffered byte-stream transport on the UART peripherals.
 *
 * Transmission is queued and sent by DMA from interrupt context, so a write
 * normally returns as soon as the bytes are buffered. Reception runs on a
 * circular DMA buffer with idle-line detection. All functions are task-context
 * only.
 *
 * Any number of tasks may write to a port. Each port supports ONE reading task:
 * uart_port_read() must not be called for the same port from two tasks.
 *
 * The data is a plain byte stream. Gaps between bursts on the wire are not
 * reported, so a protocol on top has to frame its messages itself.
 */
#ifndef UART_PORT_H
#define UART_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  UART_PORT_UART5, /* Processed-data output port. */
  UART_PORT_RS485, /* USART3, half-duplex with hardware driver enable. */
  UART_PORT_COUNT,
} uart_port_id_t;

typedef struct
{
  uint32_t tx_bytes;         /* Bytes handed to the hardware and completed. */
  uint32_t tx_dropped_bytes; /* Bytes a write could not queue before its timeout. */
  uint32_t tx_failed;        /* DMA transmissions that did not start or ended in error. */
  uint32_t tx_queue_high_water;
  uint32_t rx_bytes;
  uint32_t rx_dropped_bytes; /* RX buffer was full. */
  uint32_t rx_overrun_errors;
  uint32_t rx_framing_errors;
  uint32_t rx_noise_errors;
  uint32_t rx_restarts;      /* Reception restarted after an error. */
  uint32_t rx_restart_failures;
  bool rx_active;            /* The receiver is running. */
} uart_port_stats_t;

/**
 * Create the buffers and start reception. Call once before the scheduler starts.
 *
 * @return false if a buffer could not be created or a receiver did not start.
 */
bool uart_port_init(void);

/**
 * Queue bytes for transmission.
 *
 * Waits up to @p timeout for buffer space. A call is not interleaved with other
 * writers, but if it times out only the first part of the data is queued.
 * Queued bytes that the hardware then fails to send are dropped and counted in
 * the statistics (tx_failed, tx_dropped_bytes).
 *
 * @return Number of bytes queued, which is @p length unless the timeout expired.
 */
size_t uart_port_write(uart_port_id_t port_id, const uint8_t *data, size_t length,
                       TickType_t timeout);

/**
 * Read received bytes. One reading task per port.
 *
 * @return Number of bytes copied, at least 1 unless the timeout expired first.
 */
size_t uart_port_read(uart_port_id_t port_id, uint8_t *data, size_t length, TickType_t timeout);

/**
 * Wait until everything queued has left the wire. On the RS485 port this is
 * the point where the bus has been released and a reply can arrive.
 *
 * @return false if transmission was still in progress when the timeout expired.
 */
bool uart_port_wait_tx_idle(uart_port_id_t port_id, TickType_t timeout);

void uart_port_get_stats(uart_port_id_t port_id, uart_port_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* UART_PORT_H */
