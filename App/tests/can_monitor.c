/**
 * @file can_monitor.c
 * @brief Receive-only check of one CAN port against an existing device.
 *
 * Nothing is transmitted, so a classic CAN device on the bus never sees a CAN
 * FD frame from this board. The port still acknowledges what it receives.
 *
 * The port is BOARD_CAN_MONITOR_PORT. The data path is the one the
 * application will use: the FDCAN receive interrupt puts each frame into the
 * driver's RX queue (see can_port.c), and
 * the task here wakes up periodically, takes out everything that is waiting
 * and prints it. A summary line follows once a second. Printing is capped per
 * second so that a busy bus cannot flood the console; frames over the cap are
 * still taken out of the queue and counted.
 */
#include "can_monitor.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "board_config.h"
#include "can_port.h"
#include "cmsis_os.h"
#include "console.h"
#include "task.h"

#define CAN_MONITOR_PORT BOARD_CAN_MONITOR_PORT
/* Number printed in "canN": CAN_PORT_1 is 0. */
#define CAN_MONITOR_PORT_NUMBER ((unsigned)CAN_MONITOR_PORT + 1U)
#define CAN_MONITOR_STACK_WORDS 512U
/* How often the task checks the RX queue. The queue holds 16 frames, so this
   period copes with up to 1600 frames/s. */
#define CAN_MONITOR_POLL_MS 10U
#define CAN_MONITOR_REPORT_MS 1000U
#define CAN_MONITOR_POLLS_PER_REPORT (CAN_MONITOR_REPORT_MS / CAN_MONITOR_POLL_MS)
/* Keeps a busy bus from flooding the console (115200 baud, about 60 bytes a line). */
#define CAN_MONITOR_FRAMES_SHOWN_PER_REPORT 50U
#define CAN_MONITOR_BYTES_SHOWN 8U
#define CAN_MONITOR_HEX_CHARS_PER_BYTE 3U

static StaticTask_t task_control;
static StackType_t task_stack[CAN_MONITOR_STACK_WORDS];

static void print_frame(const can_frame_t *frame)
{
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  char data_text[(CAN_MONITOR_BYTES_SHOWN * CAN_MONITOR_HEX_CHARS_PER_BYTE) + 1U];
  size_t shown = (frame->length < CAN_MONITOR_BYTES_SHOWN) ? frame->length : CAN_MONITOR_BYTES_SHOWN;
  size_t position = 0U;

  for (size_t i = 0U; i < shown; i++)
  {
    data_text[position++] = ' ';
    data_text[position++] = HEX_DIGITS[frame->data[i] >> 4U];
    data_text[position++] = HEX_DIGITS[frame->data[i] & 0xFU];
  }
  data_text[position] = '\0';

  console_printf("can%u rx | id=0x%lX %s %s len=%u |%s%s\r\n", CAN_MONITOR_PORT_NUMBER,
                 (unsigned long)frame->id,
                 frame->extended ? "ext" : "std",
                 frame->fd ? (frame->brs ? "fd+brs" : "fd") : "classic", (unsigned)frame->length,
                 data_text, (frame->length > shown) ? " ..." : "");
}

static void report(uint32_t frames_total, uint32_t frames_this_period, uint32_t batch_max)
{
  can_port_stats_t stats;

  can_port_get_stats(CAN_MONITOR_PORT, &stats);
  console_printf("can%u monitor | %lu frames/s total=%lu | rec=%u tec=%u lec=%u busoff=%lu\r\n",
                 CAN_MONITOR_PORT_NUMBER, (unsigned long)frames_this_period, (unsigned long)frames_total,
                 (unsigned)stats.rx_error_count, (unsigned)stats.tx_error_count,
                 (unsigned)stats.last_error, (unsigned long)stats.bus_off_events);
  console_printf("      per_poll_max=%lu rxq_max=%lu rxq_drop=%lu fifo_lost=%lu console_drop=%lu\r\n",
                 (unsigned long)batch_max, (unsigned long)stats.rx_queue_high_water,
                 (unsigned long)stats.rx_queue_dropped, (unsigned long)stats.rx_fifo_lost,
                 (unsigned long)console_get_drop_count());
}

static void can_monitor_task(void *argument)
{
  TickType_t wake_time = xTaskGetTickCount();
  uint32_t polls = 0U;
  uint32_t frames_total = 0U;
  uint32_t frames_this_period = 0U;
  uint32_t batch_max = 0U; /* Most frames found waiting at one poll. */

  (void)argument;

  for (;;)
  {
    can_frame_t frame;
    uint32_t batch = 0U;

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CAN_MONITOR_POLL_MS));

    /* Take out everything the interrupt has queued since the last poll. */
    while (can_port_receive(CAN_MONITOR_PORT, &frame, 0U))
    {
      if (frames_this_period < CAN_MONITOR_FRAMES_SHOWN_PER_REPORT)
      {
        print_frame(&frame);
      }
      frames_total++;
      frames_this_period++;
      batch++;
    }
    if (batch > batch_max)
    {
      batch_max = batch;
    }

    polls++;
    if (polls >= CAN_MONITOR_POLLS_PER_REPORT)
    {
      report(frames_total, frames_this_period, batch_max);
      polls = 0U;
      frames_this_period = 0U;
    }
  }
}

void can_monitor_init(void)
{
  static const osThreadAttr_t task_attributes = {
    .name = "can_monitor",
    .cb_mem = &task_control,
    .cb_size = sizeof(task_control),
    .stack_mem = task_stack,
    .stack_size = sizeof(task_stack),
    .priority = osPriorityNormal,
  };

  (void)osThreadNew(can_monitor_task, NULL, &task_attributes);
}
