/**
 * @file rs485_test.c
 * @brief Bring-up check for the half-duplex RS485 port.
 *
 * Only one node may drive an RS485 bus at a time, so the test is a
 * request/reply exchange. Both boards run this same firmware and pick their
 * role by listening first: a board that hears a request becomes the responder,
 * a board that hears nothing for its listen window becomes the requester. The
 * window length depends on the MCU unique ID, so two boards powered up together
 * still settle on different roles.
 *
 * The requester sends a request line (see test_line.h) and expects a reply line
 * with the same sequence number; the responder waits a short turnaround time
 * and answers. Exchanges run back to back, which exercises the driver-enable
 * switching in both directions.
 */
#include "rs485_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "console.h"
#include "main.h"
#include "task.h"
#include "test_line.h"
#include "uart_port.h"

#define RS485_TEST_PORT UART_PORT_RS485
#define RS485_TEST_REQUEST_TAG "RQ"
#define RS485_TEST_REPLY_TAG "RP"
#define RS485_TEST_STACK_WORDS 512U
#define RS485_TEST_REPORT_MS 1000U
#define RS485_TEST_LISTEN_BASE_MS 500U
#define RS485_TEST_LISTEN_STEP_MS 8U /* Per unit of the node number, 0-255. */
#define RS485_TEST_NODE_MASK 0xFFU
#define RS485_TEST_WRITE_TIMEOUT_MS 100U
#define RS485_TEST_REPLY_TIMEOUT_MS 100U
/* Idle time left on the bus before a node starts driving it. */
#define RS485_TEST_TURNAROUND_MS 2U
#define RS485_TEST_READ_CHUNK 64U

static StaticTask_t task_control;
static StackType_t task_stack[RS485_TEST_STACK_WORDS];

static test_line_reader_t reader;
static uint8_t tx_line[TEST_LINE_LENGTH];
static TickType_t report_deadline;

/* Requester counters. */
static uint32_t exchanges_ok;
static uint32_t exchanges_timed_out;
static uint32_t round_trip_max_ms;
/* Responder counters. */
static uint32_t requests_answered;
/* Common counters. */
static uint32_t lines_bad;
static uint32_t lines_unexpected;
static uint32_t write_failures;

static uint32_t node_number(void)
{
  uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();

  return (uid ^ (uid >> 8U) ^ (uid >> 16U) ^ (uid >> 24U)) & RS485_TEST_NODE_MASK;
}

/* Wait up to @p timeout for the next complete line, valid or not. */
static bool read_line(test_line_t *line, TickType_t timeout)
{
  static uint8_t chunk[RS485_TEST_READ_CHUNK];
  static size_t chunk_length;
  static size_t chunk_index;
  TimeOut_t timeout_state;
  TickType_t remaining = timeout;

  vTaskSetTimeOutState(&timeout_state);
  for (;;)
  {
    while (chunk_index < chunk_length)
    {
      uint8_t byte = chunk[chunk_index];

      chunk_index++;
      if (test_line_feed(&reader, byte, line))
      {
        return true;
      }
    }

    if (xTaskCheckForTimeOut(&timeout_state, &remaining) != pdFALSE)
    {
      return false;
    }
    chunk_length = uart_port_read(RS485_TEST_PORT, chunk, sizeof(chunk), remaining);
    chunk_index = 0U;
  }
}

static void send_line(const char *tag, uint32_t sequence)
{
  test_line_build(tx_line, tag, sequence);
  if (uart_port_write(RS485_TEST_PORT, tx_line, TEST_LINE_LENGTH,
                      pdMS_TO_TICKS(RS485_TEST_WRITE_TIMEOUT_MS)) != TEST_LINE_LENGTH)
  {
    write_failures++;
  }
}

static void report_if_due(bool is_requester)
{
  static uint32_t previous_count;
  uart_port_stats_t stats;
  uint32_t count = is_requester ? exchanges_ok : requests_answered;

  if ((int32_t)(report_deadline - xTaskGetTickCount()) > 0)
  {
    return;
  }
  report_deadline += pdMS_TO_TICKS(RS485_TEST_REPORT_MS);

  uart_port_get_stats(RS485_TEST_PORT, &stats);
  if (is_requester)
  {
    console_printf("rs485 requester | %3lu exch/s | ok=%lu timeout=%lu bad=%lu unexp=%lu rtt_max=%lu ms\r\n",
                   (unsigned long)(count - previous_count), (unsigned long)exchanges_ok,
                   (unsigned long)exchanges_timed_out, (unsigned long)lines_bad,
                   (unsigned long)lines_unexpected, (unsigned long)round_trip_max_ms);
  }
  else
  {
    console_printf("rs485 responder | %3lu replies/s | answered=%lu bad=%lu unexp=%lu\r\n",
                   (unsigned long)(count - previous_count), (unsigned long)requests_answered,
                   (unsigned long)lines_bad, (unsigned long)lines_unexpected);
  }
  console_printf("      wrfail=%lu txdrop=%lu txfail=%lu rxdrop=%lu\r\n",
                 (unsigned long)write_failures, (unsigned long)stats.tx_dropped_bytes,
                 (unsigned long)stats.tx_failed, (unsigned long)stats.rx_dropped_bytes);
  console_printf("      rx errors: overrun=%lu framing=%lu noise=%lu rx_restarts=%lu\r\n",
                 (unsigned long)stats.rx_overrun_errors, (unsigned long)stats.rx_framing_errors,
                 (unsigned long)stats.rx_noise_errors, (unsigned long)stats.rx_restarts);
  previous_count = count;
}

/* Answer a request once the requester has had time to release the bus. */
static void answer(uint32_t sequence)
{
  vTaskDelay(pdMS_TO_TICKS(RS485_TEST_TURNAROUND_MS));
  send_line(RS485_TEST_REPLY_TAG, sequence);
  requests_answered++;
}

static void count_other_line(const test_line_t *line)
{
  if (line->valid)
  {
    lines_unexpected++;
  }
  else
  {
    lines_bad++;
  }
}

static void run_responder(void)
{
  test_line_t line;

  for (;;)
  {
    if (read_line(&line, pdMS_TO_TICKS(RS485_TEST_REPORT_MS)))
    {
      if (test_line_has_tag(&line, RS485_TEST_REQUEST_TAG))
      {
        answer(line.sequence);
      }
      else
      {
        count_other_line(&line);
      }
    }
    report_if_due(false);
  }
}

/* Wait for the reply to @p sequence. Other lines are counted and skipped. */
static bool wait_for_reply(uint32_t sequence)
{
  TimeOut_t timeout_state;
  TickType_t remaining = pdMS_TO_TICKS(RS485_TEST_REPLY_TIMEOUT_MS);
  test_line_t line;

  vTaskSetTimeOutState(&timeout_state);
  while (read_line(&line, remaining))
  {
    if (test_line_has_tag(&line, RS485_TEST_REPLY_TAG) && (line.sequence == sequence))
    {
      return true;
    }
    count_other_line(&line);
    if (xTaskCheckForTimeOut(&timeout_state, &remaining) != pdFALSE)
    {
      break;
    }
  }
  return false;
}

static void run_requester(void)
{
  uint32_t sequence = 0U;

  for (;;)
  {
    TickType_t start = xTaskGetTickCount();

    send_line(RS485_TEST_REQUEST_TAG, sequence);
    /* The reply timeout starts once the request has left the wire. */
    (void)uart_port_wait_tx_idle(RS485_TEST_PORT, pdMS_TO_TICKS(RS485_TEST_WRITE_TIMEOUT_MS));

    if (wait_for_reply(sequence))
    {
      uint32_t round_trip_ms = (uint32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);

      exchanges_ok++;
      if (round_trip_ms > round_trip_max_ms)
      {
        round_trip_max_ms = round_trip_ms;
      }
    }
    else
    {
      exchanges_timed_out++;
    }

    sequence++;
    report_if_due(true);
    vTaskDelay(pdMS_TO_TICKS(RS485_TEST_TURNAROUND_MS));
  }
}

static void rs485_test_task(void *argument)
{
  uint32_t listen_ms = RS485_TEST_LISTEN_BASE_MS + (node_number() * RS485_TEST_LISTEN_STEP_MS);
  TimeOut_t timeout_state;
  TickType_t remaining = pdMS_TO_TICKS(listen_ms);
  test_line_t line;

  (void)argument;
  report_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(RS485_TEST_REPORT_MS);

  /* Listen first: hearing a request means another node is already the requester. */
  vTaskSetTimeOutState(&timeout_state);
  while (read_line(&line, remaining))
  {
    if (test_line_has_tag(&line, RS485_TEST_REQUEST_TAG))
    {
      console_printf("rs485: heard a request, acting as responder\r\n");
      answer(line.sequence);
      run_responder();
    }
    if (xTaskCheckForTimeOut(&timeout_state, &remaining) != pdFALSE)
    {
      break;
    }
  }

  console_printf("rs485: bus quiet for %lu ms, acting as requester\r\n", (unsigned long)listen_ms);
  run_requester();
}

void rs485_test_init(void)
{
  static const osThreadAttr_t task_attributes = {
    .name = "rs485_test",
    .cb_mem = &task_control,
    .cb_size = sizeof(task_control),
    .stack_mem = task_stack,
    .stack_size = sizeof(task_stack),
    .priority = osPriorityNormal,
  };

  (void)osThreadNew(rs485_test_task, NULL, &task_attributes);
}
