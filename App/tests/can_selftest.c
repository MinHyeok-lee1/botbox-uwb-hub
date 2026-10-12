/**
 * @file can_selftest.c
 * @brief Bring-up check for the CAN ports in internal loopback mode.
 *
 * Once a second each port sends one frame and expects to receive the same
 * frame back. The frame kind rotates between classic CAN, CAN FD and CAN FD
 * with bit rate switching so that a failure can be attributed to the nominal
 * phase, the FD format or the data phase. Results go to the console.
 */
#include "can_selftest.h"

#include <string.h>

#include "FreeRTOS.h"
#include "can_port.h"
#include "cmsis_os.h"
#include "console.h"
#include "task.h"

#define SELFTEST_TASK_STACK_WORDS 384U
#define SELFTEST_PERIOD_MS 1000U
#define SELFTEST_TIMEOUT_MS 100U
#define SELFTEST_STANDARD_ID 0x123U
#define SELFTEST_EXTENDED_ID 0x18DAF110U
#define SELFTEST_CLASSIC_LENGTH 8U

typedef enum
{
  SELFTEST_KIND_CLASSIC,
  SELFTEST_KIND_FD,
  SELFTEST_KIND_FD_BRS,
  SELFTEST_KIND_COUNT,
} selftest_kind_t;

static const char *const KIND_NAMES[SELFTEST_KIND_COUNT] = {"classic", "fd", "fd+brs"};

static StaticTask_t task_control;
static StackType_t task_stack[SELFTEST_TASK_STACK_WORDS];

static uint32_t pass_count[CAN_PORT_COUNT][SELFTEST_KIND_COUNT];
static uint32_t fail_count[CAN_PORT_COUNT][SELFTEST_KIND_COUNT];

static bool frames_match(const can_frame_t *sent, const can_frame_t *received)
{
  return (sent->id == received->id) && (sent->length == received->length) &&
         (sent->extended == received->extended) && (sent->fd == received->fd) &&
         (sent->brs == received->brs) &&
         (memcmp(sent->data, received->data, sent->length) == 0);
}

static bool run_once(can_port_id_t port_id, selftest_kind_t kind, uint32_t sequence)
{
  /* Static to keep two 72-byte frames off the task stack. */
  static can_frame_t sent;
  static can_frame_t received;

  sent.extended = (((sequence / SELFTEST_KIND_COUNT) & 1U) != 0U);
  sent.id = sent.extended ? SELFTEST_EXTENDED_ID : SELFTEST_STANDARD_ID;
  sent.fd = (kind != SELFTEST_KIND_CLASSIC);
  sent.brs = (kind == SELFTEST_KIND_FD_BRS);
  sent.length = sent.fd ? CAN_FRAME_MAX_LENGTH : SELFTEST_CLASSIC_LENGTH;
  for (size_t i = 0U; i < sent.length; i++)
  {
    sent.data[i] = (uint8_t)(sequence + i);
  }

  if (!can_port_send(port_id, &sent, pdMS_TO_TICKS(SELFTEST_TIMEOUT_MS)))
  {
    return false;
  }
  if (!can_port_receive(port_id, &received, pdMS_TO_TICKS(SELFTEST_TIMEOUT_MS)))
  {
    return false;
  }
  return frames_match(&sent, &received);
}

static void report(can_port_id_t port_id, selftest_kind_t kind, bool passed)
{
  can_port_stats_t stats;
  const uint32_t *pass = pass_count[port_id];
  const uint32_t *fail = fail_count[port_id];

  can_port_get_stats(port_id, &stats);
  console_printf("can%u %-7s %s | pass c/f/b=%lu/%lu/%lu fail=%lu/%lu/%lu\r\n",
                 (unsigned)(port_id + 1U), KIND_NAMES[kind], passed ? "ok  " : "FAIL",
                 (unsigned long)pass[SELFTEST_KIND_CLASSIC], (unsigned long)pass[SELFTEST_KIND_FD],
                 (unsigned long)pass[SELFTEST_KIND_FD_BRS],
                 (unsigned long)fail[SELFTEST_KIND_CLASSIC], (unsigned long)fail[SELFTEST_KIND_FD],
                 (unsigned long)fail[SELFTEST_KIND_FD_BRS]);
  console_printf("     tx=%lu rx=%lu lost=%lu tec=%u rec=%u busoff=%lu lec=%u dlec=%u tdcv=%u\r\n",
                 (unsigned long)stats.tx_frames, (unsigned long)stats.rx_frames,
                 (unsigned long)(stats.rx_fifo_lost + stats.rx_queue_dropped),
                 (unsigned)stats.tx_error_count, (unsigned)stats.rx_error_count,
                 (unsigned long)stats.bus_off_events, (unsigned)stats.bus_off_last_error,
                 (unsigned)stats.bus_off_data_last_error, (unsigned)stats.bus_off_tdc_value);
}

static void selftest_task(void *argument)
{
  uint32_t sequence = 0U;

  (void)argument;

  for (;;)
  {
    selftest_kind_t kind = (selftest_kind_t)(sequence % SELFTEST_KIND_COUNT);

    osDelay(SELFTEST_PERIOD_MS);

    for (size_t i = 0U; i < CAN_PORT_COUNT; i++)
    {
      can_port_id_t port_id = (can_port_id_t)i;
      bool passed = run_once(port_id, kind, sequence);

      if (passed)
      {
        pass_count[i][kind]++;
      }
      else
      {
        fail_count[i][kind]++;
      }
      report(port_id, kind, passed);
    }
    sequence++;
  }
}

void can_selftest_init(void)
{
  static const osThreadAttr_t task_attributes = {
    .name = "can_selftest",
    .cb_mem = &task_control,
    .cb_size = sizeof(task_control),
    .stack_mem = task_stack,
    .stack_size = sizeof(task_stack),
    .priority = osPriorityBelowNormal,
  };

  (void)osThreadNew(selftest_task, NULL, &task_attributes);
}
