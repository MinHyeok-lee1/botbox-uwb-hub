/**
 * @file console.c
 * @brief Text console on the debug UART.
 */
#include "console.h"

#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "board_config.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "stream_buffer.h"

#define CONSOLE_QUEUE_SIZE     1024U
#define CONSOLE_LINE_SIZE      128U
#define CONSOLE_TX_CHUNK_SIZE  64U
#define CONSOLE_TX_TIMEOUT_MS  100U
#define CONSOLE_TASK_STACK_WORDS 256U

static const char CONSOLE_BANNER[] = "\r\nbotbox-uwb-hub: console ready\r\n";

static StaticStreamBuffer_t queue_control;
static uint8_t queue_storage[CONSOLE_QUEUE_SIZE + 1U];
static StreamBufferHandle_t queue_handle;

static StaticSemaphore_t writer_mutex_control;
static SemaphoreHandle_t writer_mutex;

/* Shared by all writers, guarded by writer_mutex. */
static char line_buffer[CONSOLE_LINE_SIZE];

static StaticTask_t task_control;
static StackType_t task_stack[CONSOLE_TASK_STACK_WORDS];

static volatile uint32_t drop_count;

static void console_task(void *argument)
{
  uint8_t chunk[CONSOLE_TX_CHUNK_SIZE];

  (void)argument;
  (void)HAL_UART_Transmit(BOARD_CONSOLE_UART, (const uint8_t *)CONSOLE_BANNER,
                          sizeof(CONSOLE_BANNER) - 1U, CONSOLE_TX_TIMEOUT_MS);

  for (;;)
  {
    size_t length = xStreamBufferReceive(queue_handle, chunk, sizeof(chunk), portMAX_DELAY);
    if (length == 0U)
    {
      continue;
    }
    (void)HAL_UART_Transmit(BOARD_CONSOLE_UART, chunk, (uint16_t)length, CONSOLE_TX_TIMEOUT_MS);
  }
}

void console_init(void)
{
  static const osThreadAttr_t task_attributes = {
    .name = "console",
    .cb_mem = &task_control,
    .cb_size = sizeof(task_control),
    .stack_mem = task_stack,
    .stack_size = sizeof(task_stack),
    .priority = osPriorityLow,
  };

  queue_handle = xStreamBufferCreateStatic(CONSOLE_QUEUE_SIZE, 1U, queue_storage, &queue_control);
  writer_mutex = xSemaphoreCreateMutexStatic(&writer_mutex_control);
  (void)osThreadNew(console_task, NULL, &task_attributes);
}

void console_printf(const char *format, ...)
{
  va_list arguments;

  if (xSemaphoreTake(writer_mutex, portMAX_DELAY) != pdTRUE)
  {
    return;
  }

  va_start(arguments, format);
  int formatted = vsnprintf(line_buffer, sizeof(line_buffer), format, arguments);
  va_end(arguments);

  if (formatted > 0)
  {
    size_t length = (size_t)formatted;
    if (length >= sizeof(line_buffer))
    {
      length = sizeof(line_buffer) - 1U;
      drop_count++;
    }
    if (xStreamBufferSend(queue_handle, line_buffer, length, 0U) != length)
    {
      drop_count++;
    }
  }

  (void)xSemaphoreGive(writer_mutex);
}

uint32_t console_get_drop_count(void)
{
  return drop_count;
}
