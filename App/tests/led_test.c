/**
 * @file led_test.c
 * @brief Chase pattern over the four board LEDs.
 *
 * The LEDs light one at a time, in the order of LED_TEST_LEDS, and the console
 * names each one as it lights, so a dead LED or a swapped label shows up right
 * away. The pins are driven high to light an LED (CubeMX sets them low at
 * start-up). The test owns all four LEDs: the CAN ports are kept off and the
 * heartbeat does not toggle its LED in this mode (see bringup_test.c and
 * app_main.c).
 */
#include "led_test.h"

#include <stddef.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "console.h"
#include "main.h"

#define LED_TEST_STACK_WORDS 256U
#define LED_TEST_STEP_MS 250U

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  const char *name;
} led_test_led_t;

static const led_test_led_t LED_TEST_LEDS[] = {
  {LED_CAN1_GPIO_Port, LED_CAN1_Pin, "LED_CAN1 (PA10)"},
  {LED_CAN2_GPIO_Port, LED_CAN2_Pin, "LED_CAN2 (PA9)"},
  {LED_RS485_GPIO_Port, LED_RS485_Pin, "LED_RS485 (PA8)"},
  {LED_UART_GPIO_Port, LED_UART_Pin, "LED_UART (PC9)"},
};
#define LED_TEST_LED_COUNT (sizeof(LED_TEST_LEDS) / sizeof(LED_TEST_LEDS[0]))

static StaticTask_t task_control;
static StackType_t task_stack[LED_TEST_STACK_WORDS];

static void led_test_task(void *argument)
{
  uint32_t round = 0U;

  (void)argument;

  for (size_t i = 0U; i < LED_TEST_LED_COUNT; i++)
  {
    HAL_GPIO_WritePin(LED_TEST_LEDS[i].port, LED_TEST_LEDS[i].pin, GPIO_PIN_RESET);
  }
  console_printf("led test: %u LEDs, %u ms each\r\n", (unsigned)LED_TEST_LED_COUNT,
                 (unsigned)LED_TEST_STEP_MS);

  for (;;)
  {
    round++;
    for (size_t i = 0U; i < LED_TEST_LED_COUNT; i++)
    {
      const led_test_led_t *led = &LED_TEST_LEDS[i];

      HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
      console_printf("led %lu.%u on: %s\r\n", (unsigned long)round, (unsigned)(i + 1U), led->name);
      osDelay(LED_TEST_STEP_MS);
      HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
    }
  }
}

void led_test_init(void)
{
  static const osThreadAttr_t attributes = {
    .name = "led_test",
    .cb_mem = &task_control,
    .cb_size = sizeof(task_control),
    .stack_mem = task_stack,
    .stack_size = sizeof(task_stack),
    .priority = osPriorityNormal,
  };

  (void)osThreadNew(led_test_task, NULL, &attributes);
}
