/**
 * @file uart_test.c
 * @brief Bring-up check for the UART5 output port.
 *
 * A writer task sends test lines (see test_line.h) back to back, which keeps
 * the port at its full output rate. A reader task checks every line that comes
 * back for a valid checksum and a continuous sequence. The same firmware
 * therefore works with the TX and RX pins of one board jumpered together, with
 * two boards cross-connected, or with a PC that just displays the lines.
 */
#include "uart_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "console.h"
#include "task.h"
#include "test_line.h"
#include "uart_port.h"

#define UART_TEST_PORT UART_PORT_UART5
#define UART_TEST_TAG "U5"
#define UART_TEST_WRITER_STACK_WORDS 256U
#define UART_TEST_READER_STACK_WORDS 512U
#define UART_TEST_REPORT_MS 1000U
#define UART_TEST_WRITE_TIMEOUT_MS 100U
#define UART_TEST_READ_CHUNK 64U
/* A sequence that jumps backwards by more than this means the sender restarted. */
#define SEQUENCE_REWIND 0x80000000U

static StaticTask_t writer_control;
static StackType_t writer_stack[UART_TEST_WRITER_STACK_WORDS];
static StaticTask_t reader_control;
static StackType_t reader_stack[UART_TEST_READER_STACK_WORDS];

/* Written by the writer task, read by the reader task. */
static volatile uint32_t lines_sent;
static volatile uint32_t lines_cut_short;

/* Reader task state. */
static test_line_reader_t reader;
static uint32_t lines_ok;
static uint32_t lines_bad;
static uint32_t lines_missed;
static uint32_t sender_restarts;
static uint32_t last_sequence;
static bool sequence_known;

static void writer_task(void *argument)
{
  static uint8_t buffer[TEST_LINE_LENGTH];
  uint32_t sequence = 0U;

  (void)argument;

  for (;;)
  {
    test_line_build(buffer, UART_TEST_TAG, sequence);
    /* The blocking write paces this loop to the UART's output rate. */
    if (uart_port_write(UART_TEST_PORT, buffer, TEST_LINE_LENGTH,
                        pdMS_TO_TICKS(UART_TEST_WRITE_TIMEOUT_MS)) == TEST_LINE_LENGTH)
    {
      lines_sent++;
    }
    else
    {
      lines_cut_short++;
    }
    sequence++;
  }
}

static void check_line(const test_line_t *line)
{
  if (!test_line_has_tag(line, UART_TEST_TAG))
  {
    /* The first line after start-up is usually a fragment; it only counts as
       bad once a complete line has been seen. */
    if (sequence_known)
    {
      lines_bad++;
    }
    return;
  }

  if (sequence_known)
  {
    uint32_t skipped = line->sequence - (last_sequence + 1U);

    if (skipped >= SEQUENCE_REWIND)
    {
      sender_restarts++;
    }
    else
    {
      lines_missed += skipped;
    }
  }
  sequence_known = true;
  last_sequence = line->sequence;
  lines_ok++;
}

static void report(void)
{
  static uint32_t previous_tx_bytes;
  static uint32_t previous_lines_ok;
  uart_port_stats_t stats;

  uart_port_get_stats(UART_TEST_PORT, &stats);
  console_printf("uart5 | tx %5lu B/s rx %3lu lines/s | ok=%lu bad=%lu miss=%lu restarts=%lu\r\n",
                 (unsigned long)(stats.tx_bytes - previous_tx_bytes),
                 (unsigned long)(lines_ok - previous_lines_ok), (unsigned long)lines_ok,
                 (unsigned long)lines_bad, (unsigned long)lines_missed,
                 (unsigned long)sender_restarts);
  console_printf("      sent=%lu cut=%lu txdrop=%lu txfail=%lu txq_max=%lu rxdrop=%lu\r\n",
                 (unsigned long)lines_sent, (unsigned long)lines_cut_short,
                 (unsigned long)stats.tx_dropped_bytes, (unsigned long)stats.tx_failed,
                 (unsigned long)stats.tx_queue_high_water, (unsigned long)stats.rx_dropped_bytes);
  console_printf("      rx errors: overrun=%lu framing=%lu noise=%lu rx_restarts=%lu\r\n",
                 (unsigned long)stats.rx_overrun_errors, (unsigned long)stats.rx_framing_errors,
                 (unsigned long)stats.rx_noise_errors, (unsigned long)stats.rx_restarts);
  previous_tx_bytes = stats.tx_bytes;
  previous_lines_ok = lines_ok;
}

static void reader_task(void *argument)
{
  static uint8_t chunk[UART_TEST_READ_CHUNK];
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(UART_TEST_REPORT_MS);

  (void)argument;

  for (;;)
  {
    int32_t remaining = (int32_t)(deadline - xTaskGetTickCount());

    if (remaining > 0)
    {
      size_t length = uart_port_read(UART_TEST_PORT, chunk, sizeof(chunk), (TickType_t)remaining);

      for (size_t i = 0U; i < length; i++)
      {
        test_line_t line;

        if (test_line_feed(&reader, chunk[i], &line))
        {
          check_line(&line);
        }
      }
      continue;
    }

    report();
    deadline += pdMS_TO_TICKS(UART_TEST_REPORT_MS);
  }
}

void uart_test_init(void)
{
  static const osThreadAttr_t writer_attributes = {
    .name = "uart_test_wr",
    .cb_mem = &writer_control,
    .cb_size = sizeof(writer_control),
    .stack_mem = writer_stack,
    .stack_size = sizeof(writer_stack),
    .priority = osPriorityBelowNormal,
  };
  static const osThreadAttr_t reader_attributes = {
    .name = "uart_test_rd",
    .cb_mem = &reader_control,
    .cb_size = sizeof(reader_control),
    .stack_mem = reader_stack,
    .stack_size = sizeof(reader_stack),
    .priority = osPriorityNormal,
  };

  (void)osThreadNew(reader_task, NULL, &reader_attributes);
  (void)osThreadNew(writer_task, NULL, &writer_attributes);
}
