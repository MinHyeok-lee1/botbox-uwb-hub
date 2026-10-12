/**
 * @file fault.c
 * @brief Last-resort handling for unrecoverable faults.
 *
 * Runs with interrupts disabled and must not rely on the HAL tick or the RTOS,
 * so the UART is written by polling its registers directly.
 */
#include "fault.h"

#include <stddef.h>
#include <stdint.h>

#include "board_config.h"

/* Rough busy-wait count for a visibly fast blink at 170 MHz. */
#define FAULT_BLINK_DELAY_LOOPS 2000000U
/* Gives up on a stuck UART instead of hanging before the LEDs blink. */
#define FAULT_TX_WAIT_LOOPS     1000000U

static void fault_write(const char *text)
{
  USART_TypeDef *uart = BOARD_CONSOLE_UART->Instance;

  if (text == NULL)
  {
    return;
  }

  for (; *text != '\0'; text++)
  {
    uint32_t remaining = FAULT_TX_WAIT_LOOPS;
    while (((uart->ISR & USART_ISR_TXE_TXFNF) == 0U) && (remaining > 0U))
    {
      remaining--;
    }
    if (remaining == 0U)
    {
      return;
    }
    uart->TDR = (uint8_t)*text;
  }
}

static const char *fault_name(fault_code_t code)
{
  switch (code)
  {
    case FAULT_HARD_FAULT:
      return "hard fault";
    case FAULT_STACK_OVERFLOW:
      return "stack overflow";
    case FAULT_MALLOC_FAILED:
      return "malloc failed";
    case FAULT_INIT_FAILED:
      return "init failed";
    default:
      return "unknown";
  }
}

void fault_panic(fault_code_t code, const char *detail)
{
  __disable_irq();

  fault_write("\r\n*** FAULT: ");
  fault_write(fault_name(code));
  if (detail != NULL)
  {
    fault_write(" (");
    fault_write(detail);
    fault_write(")");
  }
  fault_write(" ***\r\n");

  for (;;)
  {
    HAL_GPIO_TogglePin(LED_UART_GPIO_Port, LED_UART_Pin);
    HAL_GPIO_TogglePin(GPIOA, LED_CAN1_Pin | LED_CAN2_Pin | LED_RS485_Pin);
    for (volatile uint32_t i = 0U; i < FAULT_BLINK_DELAY_LOOPS; i++)
    {
    }
  }
}
