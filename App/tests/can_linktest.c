/**
 * @file can_linktest.c
 * @brief Bring-up check of one CAN port on a real bus between two boards.
 *
 * The port is BOARD_CAN_TEST_PORT. Both boards run this same firmware. Once a second each board sends one frame
 * (rotating between classic CAN, CAN FD and CAN FD with bit rate switching)
 * carrying a sequence number and a pattern, and checks every frame it receives
 * from the peer for a valid pattern and a continuous sequence. The CAN
 * identifier is derived from the MCU unique ID so the two boards differ.
 */
#include "can_linktest.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "board_config.h"
#include "can_port.h"
#include "cmsis_os.h"
#include "console.h"
#include "task.h"

#define LINKTEST_PORT BOARD_CAN_TEST_PORT
#define LINKTEST_TASK_STACK_WORDS 384U
#define LINKTEST_PERIOD_MS 1000U
#define LINKTEST_BASE_ID 0x100U
#define LINKTEST_NODE_MASK 0xFFU
#define LINKTEST_CLASSIC_LENGTH 8U
#define LINKTEST_SEQUENCE_BYTES 4U

typedef enum
{
  LINKTEST_KIND_CLASSIC,
  LINKTEST_KIND_FD,
  LINKTEST_KIND_FD_BRS,
  LINKTEST_KIND_COUNT,
} linktest_kind_t;

static const char *const KIND_NAMES[LINKTEST_KIND_COUNT] = {"classic", "fd", "fd+brs"};

static StaticTask_t task_control;
static StackType_t task_stack[LINKTEST_TASK_STACK_WORDS];

static uint32_t own_id;
static uint32_t tx_queue_full_count;
static uint32_t rx_ok_count;
static uint32_t rx_bad_count;
static uint32_t rx_gap_count;
static uint32_t peer_id;
static uint32_t peer_sequence;
static bool peer_seen;

static uint32_t derive_own_id(void)
{
  uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
  uint32_t node = (uid ^ (uid >> 8U) ^ (uid >> 16U) ^ (uid >> 24U)) & LINKTEST_NODE_MASK;

  return LINKTEST_BASE_ID | node;
}

static bool send_frame(linktest_kind_t kind, uint32_t sequence)
{
  /* Static to keep the 72-byte frame off the task stack. */
  static can_frame_t frame;

  frame.id = own_id;
  frame.extended = false;
  frame.fd = (kind != LINKTEST_KIND_CLASSIC);
  frame.brs = (kind == LINKTEST_KIND_FD_BRS);
  frame.length = frame.fd ? CAN_FRAME_MAX_LENGTH : LINKTEST_CLASSIC_LENGTH;
  for (size_t i = 0U; i < LINKTEST_SEQUENCE_BYTES; i++)
  {
    frame.data[i] = (uint8_t)(sequence >> (8U * i));
  }
  for (size_t i = LINKTEST_SEQUENCE_BYTES; i < frame.length; i++)
  {
    frame.data[i] = (uint8_t)(sequence + i);
  }

  return can_port_send(LINKTEST_PORT, &frame, 0U);
}

static void check_frame(const can_frame_t *frame)
{
  uint32_t sequence = 0U;

  if (frame->length < LINKTEST_CLASSIC_LENGTH)
  {
    rx_bad_count++;
    return;
  }
  for (size_t i = 0U; i < LINKTEST_SEQUENCE_BYTES; i++)
  {
    sequence |= (uint32_t)frame->data[i] << (8U * i);
  }
  for (size_t i = LINKTEST_SEQUENCE_BYTES; i < frame->length; i++)
  {
    if (frame->data[i] != (uint8_t)(sequence + i))
    {
      rx_bad_count++;
      return;
    }
  }

  if (peer_seen && (frame->id == peer_id) && (sequence != (peer_sequence + 1U)))
  {
    rx_gap_count++;
  }
  peer_seen = true;
  peer_id = frame->id;
  peer_sequence = sequence;
  rx_ok_count++;
}

static void receive_until(TickType_t deadline)
{
  static can_frame_t frame;

  for (;;)
  {
    int32_t remaining = (int32_t)(deadline - xTaskGetTickCount());

    if (remaining <= 0)
    {
      return;
    }
    if (!can_port_receive(LINKTEST_PORT, &frame, (TickType_t)remaining))
    {
      return;
    }
    check_frame(&frame);
  }
}

static void report(linktest_kind_t kind, uint32_t sequence)
{
  can_port_stats_t stats;

  can_port_get_stats(LINKTEST_PORT, &stats);
  console_printf("link can%u id=0x%03lX sent %-7s seq=%lu | rx ok=%lu bad=%lu gap=%lu peer=0x%03lX\r\n",
                 (unsigned)LINKTEST_PORT + 1U, (unsigned long)own_id, KIND_NAMES[kind],
                 (unsigned long)sequence,
                 (unsigned long)rx_ok_count, (unsigned long)rx_bad_count,
                 (unsigned long)rx_gap_count, (unsigned long)peer_id);
  console_printf("     tx=%lu rx=%lu lost=%lu txq_full=%lu tec=%u rec=%u busoff=%lu lec=%u dlec=%u "
                 "tdcv=%u\r\n",
                 (unsigned long)stats.tx_frames, (unsigned long)stats.rx_frames,
                 (unsigned long)(stats.rx_fifo_lost + stats.rx_queue_dropped),
                 (unsigned long)tx_queue_full_count, (unsigned)stats.tx_error_count,
                 (unsigned)stats.rx_error_count, (unsigned long)stats.bus_off_events,
                 (unsigned)stats.last_error, (unsigned)stats.data_last_error,
                 (unsigned)stats.tdc_value);
}

static void linktest_task(void *argument)
{
  uint32_t sequence = 0U;
  TickType_t deadline = xTaskGetTickCount();

  (void)argument;
  own_id = derive_own_id();

  for (;;)
  {
    linktest_kind_t kind = (linktest_kind_t)(sequence % LINKTEST_KIND_COUNT);

    deadline += pdMS_TO_TICKS(LINKTEST_PERIOD_MS);
    if (!send_frame(kind, sequence))
    {
      tx_queue_full_count++;
    }
    receive_until(deadline);
    report(kind, sequence);
    sequence++;
  }
}

void can_linktest_init(void)
{
  static const osThreadAttr_t task_attributes = {
    .name = "can_linktest",
    .cb_mem = &task_control,
    .cb_size = sizeof(task_control),
    .stack_mem = task_stack,
    .stack_size = sizeof(task_stack),
    .priority = osPriorityBelowNormal,
  };

  (void)osThreadNew(linktest_task, NULL, &task_attributes);
}
