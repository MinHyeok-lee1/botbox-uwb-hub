/**
 * @file uart_port.c
 * @brief Buffered byte-stream transport on the UART peripherals.
 */
#include "uart_port.h"

#include <stdbool.h>

#include "board_config.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"
#include "usart.h"

#define UART_PORT_TX_QUEUE_SIZE 2048U
#define UART_PORT_TX_CHUNK_SIZE 256U
#define UART_PORT_RX_QUEUE_SIZE 512U
#define UART_PORT_RX_DMA_SIZE 128U
#define UART_PORT_IDLE_POLL_MS 1U

typedef struct
{
  UART_HandleTypeDef *huart;
  /* Half-duplex bus: the receiver is switched off while this port transmits,
     so its own bytes echoed by the transceiver are not received. */
  bool mute_receiver_during_tx;

  /* Transmit: writers fill the queue under the mutex; the queue is drained one
     DMA chunk at a time, chained from the transmit-complete interrupt. */
  SemaphoreHandle_t tx_mutex;
  StaticSemaphore_t tx_mutex_control;
  StreamBufferHandle_t tx_queue;
  StaticStreamBuffer_t tx_queue_control;
  uint8_t tx_queue_storage[UART_PORT_TX_QUEUE_SIZE + 1U];
  uint8_t tx_chunk[UART_PORT_TX_CHUNK_SIZE];
  uint16_t tx_chunk_length;
  /* Whoever sets this owns the read side of the TX queue until it is cleared:
     a writer for the moment it starts the first chunk, then the interrupt
     chain. */
  volatile bool tx_busy;
  /* True while a DMA chunk is actually being transmitted. */
  volatile bool tx_in_flight;

  /* Receive: circular DMA buffer copied into the queue on half, full and idle
     events. rx_position is how far the DMA buffer has been copied so far. */
  StreamBufferHandle_t rx_queue;
  StaticStreamBuffer_t rx_queue_control;
  uint8_t rx_queue_storage[UART_PORT_RX_QUEUE_SIZE + 1U];
  uint8_t rx_dma_buffer[UART_PORT_RX_DMA_SIZE];
  uint16_t rx_position;

  volatile uint32_t tx_bytes;
  volatile uint32_t tx_dropped_bytes;
  volatile uint32_t tx_failed;
  volatile uint32_t tx_queue_high_water;
  volatile uint32_t rx_bytes;
  volatile uint32_t rx_dropped_bytes;
  volatile uint32_t rx_overrun_errors;
  volatile uint32_t rx_framing_errors;
  volatile uint32_t rx_noise_errors;
  volatile uint32_t rx_restarts;
  volatile uint32_t rx_restart_failures;
} uart_port_t;

static uart_port_t ports[UART_PORT_COUNT] = {
  [UART_PORT_UART5] = {.huart = &huart5},
  [UART_PORT_RS485] = {.huart = &huart3, .mute_receiver_during_tx = BOARD_RS485_MUTE_RECEIVER_DURING_TX},
};

static uart_port_t *find_port(const UART_HandleTypeDef *huart)
{
  for (size_t i = 0U; i < UART_PORT_COUNT; i++)
  {
    if (ports[i].huart == huart)
    {
      return &ports[i];
    }
  }
  return NULL;
}

/* @return false if the receiver was already running or could not be started. */
static bool start_reception(uart_port_t *port)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->rx_dma_buffer, UART_PORT_RX_DMA_SIZE) !=
      HAL_OK)
  {
    return false;
  }
  port->rx_position = 0U;
  return true;
}

static void set_receiver_enabled(const uart_port_t *port, bool enabled)
{
  if (!port->mute_receiver_during_tx)
  {
    return;
  }
  if (enabled)
  {
    ATOMIC_SET_BIT(port->huart->Instance->CR1, USART_CR1_RE);
  }
  else
  {
    ATOMIC_CLEAR_BIT(port->huart->Instance->CR1, USART_CR1_RE);
  }
}

/* Nothing is in flight any more: give up the read side of the TX queue. */
static void release_transmitter(uart_port_t *port)
{
  set_receiver_enabled(port, true);
  port->tx_busy = false;
}

/* Send queued bytes until a DMA transmission is running or the queue is empty.
   The caller owns the read side of the TX queue (tx_busy). A chunk the HAL
   refuses is dropped and the next one is tried, so queued data never waits for
   another write to get going again.

   @param task_woken NULL when called from a task, otherwise the interrupt's
                     yield flag. */
static void transmit_queued(uart_port_t *port, BaseType_t *task_woken)
{
  bool from_isr = (task_woken != NULL);

  for (;;)
  {
    bool started;
    size_t length =
      from_isr ? xStreamBufferReceiveFromISR(port->tx_queue, port->tx_chunk,
                                             UART_PORT_TX_CHUNK_SIZE, task_woken)
               : xStreamBufferReceive(port->tx_queue, port->tx_chunk, UART_PORT_TX_CHUNK_SIZE, 0U);

    if (length == 0U)
    {
      release_transmitter(port);
      return;
    }

    set_receiver_enabled(port, false);
    port->tx_chunk_length = (uint16_t)length;

    /* From a task, the UART interrupts are kept out until tx_in_flight matches
       what the HAL was told, so the error callback cannot mistake a chunk that
       has not started yet for one that ended in error. */
    if (!from_isr)
    {
      taskENTER_CRITICAL();
    }
    started = (HAL_UART_Transmit_DMA(port->huart, port->tx_chunk, port->tx_chunk_length) == HAL_OK);
    port->tx_in_flight = started;
    if (!started)
    {
      port->tx_failed++;
      port->tx_dropped_bytes += length;
    }
    if (!from_isr)
    {
      taskEXIT_CRITICAL();
    }

    if (started)
    {
      return;
    }
  }
}

/* Start transmitting if nothing is in flight. Called by a writer holding the mutex. */
static void kick_transmit(uart_port_t *port)
{
  bool claimed;

  taskENTER_CRITICAL();
  claimed = !port->tx_busy;
  if (claimed)
  {
    port->tx_busy = true;
  }
  taskEXIT_CRITICAL();

  if (claimed)
  {
    transmit_queued(port, NULL);
  }
}

/* The counter is also updated from the UART interrupts. */
static void count_dropped(uart_port_t *port, size_t length)
{
  taskENTER_CRITICAL();
  port->tx_dropped_bytes += length;
  taskEXIT_CRITICAL();
}

static bool receiver_is_active(const uart_port_t *port)
{
  return port->huart->RxState == HAL_UART_STATE_BUSY_RX;
}

bool uart_port_init(void)
{
  bool ok = true;

  for (size_t i = 0U; i < UART_PORT_COUNT; i++)
  {
    uart_port_t *port = &ports[i];

    port->tx_mutex = xSemaphoreCreateMutexStatic(&port->tx_mutex_control);
    port->tx_queue = xStreamBufferCreateStatic(UART_PORT_TX_QUEUE_SIZE, 1U,
                                               port->tx_queue_storage, &port->tx_queue_control);
    port->rx_queue = xStreamBufferCreateStatic(UART_PORT_RX_QUEUE_SIZE, 1U,
                                               port->rx_queue_storage, &port->rx_queue_control);
    if ((port->tx_mutex == NULL) || (port->tx_queue == NULL) || (port->rx_queue == NULL) ||
        !start_reception(port))
    {
      ok = false;
    }
  }
  return ok;
}

size_t uart_port_write(uart_port_id_t port_id, const uint8_t *data, size_t length,
                       TickType_t timeout)
{
  TimeOut_t timeout_state;
  TickType_t remaining = timeout;
  size_t queued = 0U;

  if ((port_id >= UART_PORT_COUNT) || (data == NULL))
  {
    return 0U;
  }

  uart_port_t *port = &ports[port_id];

  vTaskSetTimeOutState(&timeout_state);
  if (xSemaphoreTake(port->tx_mutex, remaining) != pdTRUE)
  {
    count_dropped(port, length);
    return 0U;
  }

  /* Queued in chunks so that a write larger than the queue can make progress
     while the hardware drains it. */
  while (queued < length)
  {
    size_t chunk = length - queued;

    if (chunk > UART_PORT_TX_CHUNK_SIZE)
    {
      chunk = UART_PORT_TX_CHUNK_SIZE;
    }
    if (xTaskCheckForTimeOut(&timeout_state, &remaining) != pdFALSE)
    {
      remaining = 0U;
    }

    size_t accepted = xStreamBufferSend(port->tx_queue, &data[queued], chunk, remaining);
    uint32_t waiting = (uint32_t)xStreamBufferBytesAvailable(port->tx_queue);

    if (waiting > port->tx_queue_high_water)
    {
      port->tx_queue_high_water = waiting;
    }
    kick_transmit(port);
    queued += accepted;
    if (accepted < chunk)
    {
      break;
    }
  }

  if (queued < length)
  {
    count_dropped(port, length - queued);
  }
  (void)xSemaphoreGive(port->tx_mutex);
  return queued;
}

size_t uart_port_read(uart_port_id_t port_id, uint8_t *data, size_t length, TickType_t timeout)
{
  if ((port_id >= UART_PORT_COUNT) || (data == NULL))
  {
    return 0U;
  }

  uart_port_t *port = &ports[port_id];

  /* A restart that failed in the error callback is tried again here, so the
     receiver does not stay off for good. */
  if (!receiver_is_active(port))
  {
    taskENTER_CRITICAL();
    if (!receiver_is_active(port) && start_reception(port))
    {
      port->rx_restarts++;
    }
    taskEXIT_CRITICAL();
  }
  return xStreamBufferReceive(port->rx_queue, data, length, timeout);
}

bool uart_port_wait_tx_idle(uart_port_id_t port_id, TickType_t timeout)
{
  TimeOut_t timeout_state;
  TickType_t remaining = timeout;

  if (port_id >= UART_PORT_COUNT)
  {
    return false;
  }

  const uart_port_t *port = &ports[port_id];

  vTaskSetTimeOutState(&timeout_state);
  while (port->tx_busy || (xStreamBufferIsEmpty(port->tx_queue) == pdFALSE))
  {
    if (xTaskCheckForTimeOut(&timeout_state, &remaining) != pdFALSE)
    {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(UART_PORT_IDLE_POLL_MS));
  }
  return true;
}

void uart_port_get_stats(uart_port_id_t port_id, uart_port_stats_t *stats)
{
  if ((port_id >= UART_PORT_COUNT) || (stats == NULL))
  {
    return;
  }

  const uart_port_t *port = &ports[port_id];

  stats->tx_bytes = port->tx_bytes;
  stats->tx_dropped_bytes = port->tx_dropped_bytes;
  stats->tx_failed = port->tx_failed;
  stats->tx_queue_high_water = port->tx_queue_high_water;
  stats->rx_bytes = port->rx_bytes;
  stats->rx_dropped_bytes = port->rx_dropped_bytes;
  stats->rx_overrun_errors = port->rx_overrun_errors;
  stats->rx_framing_errors = port->rx_framing_errors;
  stats->rx_noise_errors = port->rx_noise_errors;
  stats->rx_restarts = port->rx_restarts;
  stats->rx_restart_failures = port->rx_restart_failures;
  stats->rx_active = receiver_is_active(port);
}

/* HAL callbacks, called from the UART and DMA interrupt handlers. */

static void queue_received(uart_port_t *port, uint16_t start, uint16_t end,
                           BaseType_t *task_woken)
{
  size_t length = (size_t)(end - start);

  if (length == 0U)
  {
    return;
  }

  size_t accepted =
    xStreamBufferSendFromISR(port->rx_queue, &port->rx_dma_buffer[start], length, task_woken);

  port->rx_bytes += accepted;
  port->rx_dropped_bytes += (length - accepted);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uart_port_t *port = find_port(huart);
  BaseType_t task_woken = pdFALSE;

  if (port == NULL)
  {
    return;
  }

  /* Size is the position reached in the circular DMA buffer, not a byte count.
     The same position can be reported twice (full, then idle), and a smaller
     position means the buffer wrapped since the last event. */
  if (Size < port->rx_position)
  {
    queue_received(port, port->rx_position, UART_PORT_RX_DMA_SIZE, &task_woken);
    port->rx_position = 0U;
  }
  queue_received(port, port->rx_position, Size, &task_woken);
  port->rx_position = Size;

  portYIELD_FROM_ISR(task_woken);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_port_t *port = find_port(huart);
  BaseType_t task_woken = pdFALSE;

  if (port == NULL)
  {
    return;
  }

  port->tx_in_flight = false;
  port->tx_bytes += port->tx_chunk_length;
  transmit_queued(port, &task_woken);
  portYIELD_FROM_ISR(task_woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_port_t *port = find_port(huart);
  BaseType_t task_woken = pdFALSE;

  if (port == NULL)
  {
    return;
  }

  /* A transmit DMA error ends the chunk in flight without a transmit-complete
     event, so the chain has to be continued from here. A receive DMA error
     looks the same in the UART handle (the HAL sets the same error code and
     marks the transmitter ready) but leaves the transmit DMA running, and its
     transmit-complete event still arrives; only the DMA channel tells them
     apart. */
  if (port->tx_in_flight && (HAL_DMA_GetError(huart->hdmatx) != HAL_DMA_ERROR_NONE))
  {
    port->tx_in_flight = false;
    port->tx_failed++;
    port->tx_dropped_bytes += port->tx_chunk_length;
    transmit_queued(port, &task_woken);
  }

  if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
  {
    port->rx_overrun_errors++;
  }
  if ((huart->ErrorCode & HAL_UART_ERROR_FE) != 0U)
  {
    port->rx_framing_errors++;
  }
  if ((huart->ErrorCode & HAL_UART_ERROR_NE) != 0U)
  {
    port->rx_noise_errors++;
  }

  /* With DMA reception the HAL aborts the receiver on any receive error, so it
     has to be started again. Bytes still in the DMA buffer are discarded. */
  if (!receiver_is_active(port))
  {
    if (start_reception(port))
    {
      port->rx_restarts++;
    }
    else
    {
      port->rx_restart_failures++;
    }
  }
  portYIELD_FROM_ISR(task_woken);
}
