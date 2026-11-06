/**
 * @file can_port.c
 * @brief Queued CAN FD frame transport on the FDCAN peripherals.
 */
#include "can_port.h"

#include <stddef.h>

#include "board_config.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "queue.h"
#include "task.h"

#define CAN_PORT_RX_QUEUE_LENGTH 16U
#define CAN_PORT_TX_QUEUE_LENGTH 16U
#define CAN_PORT_TX_TASK_STACK_WORDS 384U
#define CAN_PORT_TX_TASK_POLL_MS 50U
#define CAN_PORT_BUS_OFF_RECOVERY_MS 100U
#define CAN_PORT_CLASSIC_MAX_LENGTH 8U
#define CAN_PORT_DLC_COUNT 16U
#define CAN_PORT_TDC_MAX_DATA_PRESCALER 2U
#define CAN_PORT_TDC_MAX_OFFSET 127U
#define CAN_PORT_ALL_TX_BUFFERS (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2)
#define CAN_PORT_HW_TX_BUFFERS  3U
#define CAN_PORT_NOTIFICATIONS                                                 \
  (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |            \
   FDCAN_IT_TX_COMPLETE | FDCAN_IT_BUS_OFF)

/* The HAL header fields carry the raw DLC code, which the table below relies on. */
_Static_assert(FDCAN_DLC_BYTES_64 == (CAN_PORT_DLC_COUNT - 1U), "unexpected FDCAN DLC encoding");

static const uint8_t DLC_TO_LENGTH[CAN_PORT_DLC_COUNT] = {0U,  1U,  2U,  3U,  4U,  5U,  6U,  7U,
                                                          8U,  12U, 16U, 20U, 24U, 32U, 48U, 64U};

typedef struct
{
  FDCAN_HandleTypeDef *hfdcan;
  GPIO_TypeDef *led_port;
  uint16_t led_pin;
  can_port_config_t config;

  QueueHandle_t rx_queue;
  QueueHandle_t tx_queue;
  StaticQueue_t rx_queue_control;
  StaticQueue_t tx_queue_control;
  uint8_t rx_queue_storage[CAN_PORT_RX_QUEUE_LENGTH * sizeof(can_frame_t)];
  uint8_t tx_queue_storage[CAN_PORT_TX_QUEUE_LENGTH * sizeof(can_frame_t)];

  /* TX buffers holding a frame that has not completed yet. */
  volatile uint32_t tx_pending_buffers;

  uint32_t tdc_offset; /* 0 when transmitter delay compensation is off. */

  volatile bool started;
  volatile bool start_failed;
  volatile bool bus_off;
  volatile TickType_t bus_off_tick;

  volatile uint32_t rx_frames;
  volatile uint32_t rx_queue_dropped;
  volatile uint32_t rx_queue_high_water;
  volatile uint32_t rx_fifo_lost;
  volatile uint32_t tx_frames;
  volatile uint32_t tx_failed;
  volatile uint32_t bus_off_events;
  volatile uint8_t bus_off_last_error;
  volatile uint8_t bus_off_data_last_error;
  volatile uint8_t bus_off_tdc_value;
} can_port_t;

static can_port_t ports[CAN_PORT_COUNT] = {
  [CAN_PORT_1] = {.hfdcan = &hfdcan1, .led_port = LED_CAN1_GPIO_Port, .led_pin = LED_CAN1_Pin},
  [CAN_PORT_2] = {.hfdcan = &hfdcan2, .led_port = LED_CAN2_GPIO_Port, .led_pin = LED_CAN2_Pin},
};

static StaticTask_t tx_task_control;
static StackType_t tx_task_stack[CAN_PORT_TX_TASK_STACK_WORDS];
static TaskHandle_t tx_task_handle;

static can_port_t *find_port(const FDCAN_HandleTypeDef *hfdcan)
{
  for (size_t i = 0U; i < CAN_PORT_COUNT; i++)
  {
    if (ports[i].hfdcan == hfdcan)
    {
      return &ports[i];
    }
  }
  return NULL;
}

static bool length_to_dlc(uint8_t length, uint32_t *dlc)
{
  for (uint32_t code = 0U; code < CAN_PORT_DLC_COUNT; code++)
  {
    if (DLC_TO_LENGTH[code] == length)
    {
      *dlc = code;
      return true;
    }
  }
  return false;
}

static bool configure_filters(FDCAN_HandleTypeDef *hfdcan)
{
  /* Mask 0 matches every identifier: accept all data frames into RX FIFO 0. */
  FDCAN_FilterTypeDef filter = {
    .IdType = FDCAN_STANDARD_ID,
    .FilterIndex = 0U,
    .FilterType = FDCAN_FILTER_MASK,
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
    .FilterID1 = 0U,
    .FilterID2 = 0U,
  };

  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    return false;
  }
  filter.IdType = FDCAN_EXTENDED_ID;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    return false;
  }
  return HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE) == HAL_OK;
}

/* Offset in kernel clock periods from the start of a data bit to its sample point. */
static uint32_t data_sample_point_offset(const FDCAN_HandleTypeDef *hfdcan)
{
  return (1U + hfdcan->Init.DataTimeSeg1) * hfdcan->Init.DataPrescaler;
}

static bool configure_tdc(can_port_t *port)
{
  FDCAN_HandleTypeDef *hfdcan = port->hfdcan;
  uint32_t offset = data_sample_point_offset(hfdcan);

  port->tdc_offset = 0U;
  if (!BOARD_CAN_TDC_ENABLE || (hfdcan->Init.DataPrescaler > CAN_PORT_TDC_MAX_DATA_PRESCALER) ||
      (offset > CAN_PORT_TDC_MAX_OFFSET))
  {
    return true;
  }
  if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, offset, BOARD_CAN_TDC_FILTER) != HAL_OK)
  {
    return false;
  }
  if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK)
  {
    return false;
  }
  port->tdc_offset = offset;
  return true;
}

static bool start_port(can_port_t *port)
{
  FDCAN_HandleTypeDef *hfdcan = port->hfdcan;

  if (port->config.internal_loopback)
  {
    hfdcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
    {
      return false;
    }
  }

  if (!configure_filters(hfdcan))
  {
    return false;
  }
  if (!configure_tdc(port))
  {
    return false;
  }
  if (HAL_FDCAN_ActivateNotification(hfdcan, CAN_PORT_NOTIFICATIONS, CAN_PORT_ALL_TX_BUFFERS) !=
      HAL_OK)
  {
    return false;
  }
  if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
  {
    return false;
  }

  port->started = true;
  return true;
}

/* Bus-off leaves the peripheral in init mode; restarting it rejoins the bus. */
static void recover_bus_off(can_port_t *port)
{
  if ((xTaskGetTickCount() - port->bus_off_tick) < pdMS_TO_TICKS(CAN_PORT_BUS_OFF_RECOVERY_MS))
  {
    return;
  }

  /* Stopping discards the frames still waiting in the hardware TX buffers. */
  (void)HAL_FDCAN_Stop(port->hfdcan);
  /* The critical section keeps a TX complete interrupt that was already
     pending from updating the mask while it is cleared. HAL_FDCAN_Stop() stays
     outside it because its timeouts need the HAL tick interrupt. */
  taskENTER_CRITICAL();
  port->tx_pending_buffers = 0U;
  taskEXIT_CRITICAL();
  if (HAL_FDCAN_Start(port->hfdcan) == HAL_OK)
  {
    port->bus_off = false;
  }
  else
  {
    port->bus_off_tick = xTaskGetTickCount();
  }
}

static void pump_tx(can_port_t *port)
{
  can_frame_t frame;

  /* The frame stays in the queue until the hardware has accepted it, so a
     refused frame is retried instead of being lost. */
  while ((HAL_FDCAN_GetTxFifoFreeLevel(port->hfdcan) > 0U) &&
         (xQueuePeek(port->tx_queue, &frame, 0U) == pdTRUE))
  {
    bool accepted;

    FDCAN_TxHeaderTypeDef header = {
      .Identifier = frame.id,
      .IdType = frame.extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = (frame.fd && frame.brs) ? FDCAN_BRS_ON : FDCAN_BRS_OFF,
      .FDFormat = frame.fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0U,
    };

    /* The length was validated in can_port_send(). */
    (void)length_to_dlc(frame.length, &header.DataLength);

    /* Keeps the TX complete interrupt out until the buffer is marked pending. */
    taskENTER_CRITICAL();
    accepted = (HAL_FDCAN_AddMessageToTxFifoQ(port->hfdcan, &header, frame.data) == HAL_OK);
    if (accepted)
    {
      port->tx_pending_buffers |= port->hfdcan->LatestTxFifoQRequest;
    }
    taskEXIT_CRITICAL();

    if (!accepted)
    {
      port->tx_failed++;
      return;
    }
    (void)xQueueReceive(port->tx_queue, &frame, 0U);
  }
}

static void can_tx_task(void *argument)
{
  (void)argument;

  /* Started here rather than in can_port_init() so that the HAL tick and the
     FDCAN interrupts are live while the peripheral is brought up. */
  for (size_t i = 0U; i < CAN_PORT_COUNT; i++)
  {
    if (ports[i].config.enabled && !start_port(&ports[i]))
    {
      ports[i].start_failed = true;
    }
  }

  for (;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAN_PORT_TX_TASK_POLL_MS));

    for (size_t i = 0U; i < CAN_PORT_COUNT; i++)
    {
      can_port_t *port = &ports[i];

      if (!port->started)
      {
        continue;
      }
      if (port->bus_off)
      {
        recover_bus_off(port);
        continue;
      }
      pump_tx(port);
    }
  }
}

bool can_port_init(const can_port_config_t config[CAN_PORT_COUNT])
{
  static const osThreadAttr_t tx_task_attributes = {
    .name = "can_tx",
    .cb_mem = &tx_task_control,
    .cb_size = sizeof(tx_task_control),
    .stack_mem = tx_task_stack,
    .stack_size = sizeof(tx_task_stack),
    .priority = osPriorityAboveNormal,
  };

  for (size_t i = 0U; i < CAN_PORT_COUNT; i++)
  {
    can_port_t *port = &ports[i];

    port->config = config[i];
    port->rx_queue = xQueueCreateStatic(CAN_PORT_RX_QUEUE_LENGTH, sizeof(can_frame_t),
                                        port->rx_queue_storage, &port->rx_queue_control);
    port->tx_queue = xQueueCreateStatic(CAN_PORT_TX_QUEUE_LENGTH, sizeof(can_frame_t),
                                        port->tx_queue_storage, &port->tx_queue_control);
    if ((port->rx_queue == NULL) || (port->tx_queue == NULL))
    {
      return false;
    }
  }

  tx_task_handle = (TaskHandle_t)osThreadNew(can_tx_task, NULL, &tx_task_attributes);
  return tx_task_handle != NULL;
}

bool can_port_send(can_port_id_t port_id, const can_frame_t *frame, TickType_t timeout)
{
  uint32_t dlc;

  if ((port_id >= CAN_PORT_COUNT) || (frame == NULL) || !ports[port_id].config.enabled ||
      ports[port_id].start_failed)
  {
    return false;
  }
  if (!length_to_dlc(frame->length, &dlc))
  {
    return false;
  }
  if (!frame->fd && (frame->length > CAN_PORT_CLASSIC_MAX_LENGTH))
  {
    return false;
  }
  if (xQueueSend(ports[port_id].tx_queue, frame, timeout) != pdTRUE)
  {
    return false;
  }

  (void)xTaskNotifyGive(tx_task_handle);
  return true;
}

uint32_t can_port_tx_pending(can_port_id_t port_id)
{
  if ((port_id >= CAN_PORT_COUNT) || (ports[port_id].tx_queue == NULL))
  {
    return 0U;
  }
  uint32_t pending = (uint32_t)uxQueueMessagesWaiting(ports[port_id].tx_queue);
  const uint32_t hardware_free = HAL_FDCAN_GetTxFifoFreeLevel(ports[port_id].hfdcan);
  if (hardware_free < CAN_PORT_HW_TX_BUFFERS)
  {
    pending += CAN_PORT_HW_TX_BUFFERS - hardware_free;
  }
  return pending;
}

bool can_port_receive(can_port_id_t port_id, can_frame_t *frame, TickType_t timeout)
{
  if ((port_id >= CAN_PORT_COUNT) || (frame == NULL))
  {
    return false;
  }
  return xQueueReceive(ports[port_id].rx_queue, frame, timeout) == pdTRUE;
}

void can_port_get_stats(can_port_id_t port_id, can_port_stats_t *stats)
{
  FDCAN_ErrorCountersTypeDef counters = {0};
  FDCAN_ProtocolStatusTypeDef status = {0};

  if ((port_id >= CAN_PORT_COUNT) || (stats == NULL))
  {
    return;
  }

  const can_port_t *port = &ports[port_id];

  (void)HAL_FDCAN_GetErrorCounters(port->hfdcan, &counters);
  (void)HAL_FDCAN_GetProtocolStatus(port->hfdcan, &status);
  stats->last_error = (uint8_t)status.LastErrorCode;
  stats->data_last_error = (uint8_t)status.DataLastErrorCode;
  stats->tdc_value = (uint8_t)status.TDCvalue;
  stats->enabled = port->config.enabled;
  stats->started = port->started;
  stats->start_failed = port->start_failed;
  stats->tdc_offset = (uint8_t)port->tdc_offset;
  stats->bus_off = port->bus_off;
  stats->tx_error_count = (uint8_t)counters.TxErrorCnt;
  stats->rx_error_count = (uint8_t)counters.RxErrorCnt;
  stats->rx_frames = port->rx_frames;
  stats->rx_queue_dropped = port->rx_queue_dropped;
  stats->rx_queue_high_water = port->rx_queue_high_water;
  stats->rx_fifo_lost = port->rx_fifo_lost;
  stats->tx_frames = port->tx_frames;
  stats->tx_failed = port->tx_failed;
  stats->bus_off_events = port->bus_off_events;
  stats->bus_off_last_error = port->bus_off_last_error;
  stats->bus_off_data_last_error = port->bus_off_data_last_error;
  stats->bus_off_tdc_value = port->bus_off_tdc_value;
}

uint32_t can_port_tx_task_stack_free(void)
{
  return (uint32_t)uxTaskGetStackHighWaterMark(tx_task_handle);
}

/* HAL callbacks, called from the FDCAN interrupt handlers. */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  can_port_t *port = find_port(hfdcan);
  BaseType_t task_woken = pdFALSE;

  if (port == NULL)
  {
    return;
  }
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    port->rx_fifo_lost++;
  }

  /* The hardware FIFO holds only three frames, so drain it completely. */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    FDCAN_RxHeaderTypeDef header;
    can_frame_t frame;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK)
    {
      break;
    }

    frame.id = header.Identifier;
    frame.rx_tick = xTaskGetTickCountFromISR();
    frame.length = DLC_TO_LENGTH[header.DataLength & (CAN_PORT_DLC_COUNT - 1U)];
    frame.extended = (header.IdType == FDCAN_EXTENDED_ID);
    frame.fd = (header.FDFormat == FDCAN_FD_CAN);
    frame.brs = (header.BitRateSwitch == FDCAN_BRS_ON);

    if (xQueueSendFromISR(port->rx_queue, &frame, &task_woken) == pdTRUE)
    {
      uint32_t waiting = (uint32_t)uxQueueMessagesWaitingFromISR(port->rx_queue);

      port->rx_frames++;
      if (waiting > port->rx_queue_high_water)
      {
        port->rx_queue_high_water = waiting;
      }
    }
    else
    {
      port->rx_queue_dropped++;
    }
    HAL_GPIO_TogglePin(port->led_port, port->led_pin);
  }

  portYIELD_FROM_ISR(task_woken);
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
  can_port_t *port = find_port(hfdcan);
  BaseType_t task_woken = pdFALSE;

  if (port == NULL)
  {
    return;
  }

  /* The HAL reports every buffer that has ever completed, not only the new ones. */
  uint32_t completed = BufferIndexes & port->tx_pending_buffers;

  port->tx_pending_buffers &= ~completed;
  port->tx_frames += (uint32_t)__builtin_popcount(completed);
  vTaskNotifyGiveFromISR(tx_task_handle, &task_woken);
  portYIELD_FROM_ISR(task_woken);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  can_port_t *port = find_port(hfdcan);
  FDCAN_ProtocolStatusTypeDef status = {0};
  BaseType_t task_woken = pdFALSE;

  if ((port == NULL) || ((ErrorStatusITs & FDCAN_IT_BUS_OFF) == 0U))
  {
    return;
  }
  /* The interrupt fires on both entering and leaving bus-off. */
  if ((HAL_FDCAN_GetProtocolStatus(hfdcan, &status) != HAL_OK) || (status.BusOff == 0U))
  {
    return;
  }

  port->bus_off_events++;
  port->bus_off_last_error = (uint8_t)status.LastErrorCode;
  port->bus_off_data_last_error = (uint8_t)status.DataLastErrorCode;
  port->bus_off_tdc_value = (uint8_t)status.TDCvalue;
  port->bus_off_tick = xTaskGetTickCountFromISR();
  port->bus_off = true;
  vTaskNotifyGiveFromISR(tx_task_handle, &task_woken);
  portYIELD_FROM_ISR(task_woken);
}
