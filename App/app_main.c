/**
 * @file app_main.c
 * @brief Application entry point: creates the application tasks.
 */
#include "app_main.h"

#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "board_config.h"
#include "bringup_test.h"
#include "can_port.h"
#include "cmsis_os.h"
#include "console.h"
#include "fault.h"
#include "firmware_info.h"
#include "task.h"
#include "sys_node.h"
#include "uart_port.h"
#include "uwb_task.h"

#define HEARTBEAT_TASK_STACK_WORDS 384U

static const char *const CAN_PORT_NAMES[CAN_PORT_COUNT] = {
  [CAN_PORT_1] = "can1",
  [CAN_PORT_2] = "can2",
};

static StaticTask_t heartbeat_task_control;
static StackType_t heartbeat_task_stack[HEARTBEAT_TASK_STACK_WORDS];

static can_port_config_t can_config[CAN_PORT_COUNT] = {
  [CAN_PORT_1] = {.enabled = BOARD_CAN1_ENABLE},
  [CAN_PORT_2] = {.enabled = BOARD_CAN2_ENABLE},
};

/* Print how each CAN port came up, once the driver has finished starting them.
   @return false while a port is still starting. */
static bool report_can_ports(void)
{
  can_port_stats_t stats[CAN_PORT_COUNT];

  for (size_t i = 0U; i < CAN_PORT_COUNT; i++)
  {
    can_port_get_stats((can_port_id_t)i, &stats[i]);
    if (stats[i].enabled && !stats[i].started && !stats[i].start_failed)
    {
      return false;
    }
  }

  for (size_t i = 0U; i < CAN_PORT_COUNT; i++)
  {
    if (!stats[i].enabled)
    {
      console_printf("%s: disabled\r\n", CAN_PORT_NAMES[i]);
      continue;
    }
    console_printf("%s: %s%s, tdc offset %u%s\r\n", CAN_PORT_NAMES[i],
                   stats[i].started ? "started" : "START FAILED",
                   can_config[i].internal_loopback ? " (internal loopback)" : "",
                   (unsigned)stats[i].tdc_offset, (stats[i].tdc_offset == 0U) ? " (off)" : "");
  }
  return true;
}

static void heartbeat_task(void *argument)
{
  bool can_ports_reported = false;

  (void)argument;

  firmware_info_report();

  for (;;)
  {
    if (!can_ports_reported)
    {
      can_ports_reported = report_can_ports();
    }
#if BOARD_TEST_MODE != BOARD_TEST_LED
    HAL_GPIO_TogglePin(BOARD_HEARTBEAT_LED_PORT, BOARD_HEARTBEAT_LED_PIN);
#endif
    osDelay(BOARD_HEARTBEAT_PERIOD_MS);
  }
}

void app_main_init(void)
{
  static const osThreadAttr_t heartbeat_attributes = {
    .name = "heartbeat",
    .cb_mem = &heartbeat_task_control,
    .cb_size = sizeof(heartbeat_task_control),
    .stack_mem = heartbeat_task_stack,
    .stack_size = sizeof(heartbeat_task_stack),
    .priority = osPriorityLow,
  };

  console_init();
  firmware_info_load_configuration();

  bringup_test_adjust_can_config(can_config);
  if (!can_port_init(can_config))
  {
    fault_panic(FAULT_INIT_FAILED, "can_port");
  }
  if (!uart_port_init())
  {
    fault_panic(FAULT_INIT_FAILED, "uart_port");
  }

  bringup_test_init();
#if BOARD_TEST_MODE == BOARD_TEST_NONE
  /* The bring-up tests read the CAN1 and CAN2 queues themselves, so the UWB
     task and the IF-5 node only run without them. The IF-5 inbox must exist
     before the UWB task forwards frames to it. */
  if (!sys_node_init())
  {
    fault_panic(FAULT_INIT_FAILED, "sys_node");
  }
  uwb_task_init();
#endif
  (void)osThreadNew(heartbeat_task, NULL, &heartbeat_attributes);
}
