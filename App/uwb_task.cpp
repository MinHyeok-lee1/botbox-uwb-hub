/**
 * @file uwb_task.cpp
 * @brief Task that turns the UWB sensor frames on CAN1 into the follow result.
 *
 * The task reads the CAN1 RX queue often, so the queue cannot fill up between
 * two calculations, and runs UwbManager::update() once per UWB_TASK_PERIOD_MS,
 * the period the algorithm was tuned for on botbox+. Only this task touches
 * the manager.
 *
 * Each result is sent to the drive board on CAN2 (uwb_output.cpp) and
 * printed on the console. Output on UART5 is not implemented yet.
 */
#include "uwb_task.h"

#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "board_config.h"
#include "can_port.h"
#include "cmsis_os.h"
#include "console.h"
#include "firmware_configuration.hpp"
#include "pdoa_output.hpp"
#include "sys_node.h"
#include "task.h"
#include "uwb_manager.hpp"
#include "uwb_output.hpp"

namespace {

constexpr can_port_id_t UWB_SENSOR_PORT = CAN_PORT_1;
constexpr uint32_t UWB_TASK_STACK_WORDS = 1024U;
/* How often the CAN1 RX queue is emptied. The queue holds 16 frames, so this
   copes with up to 1600 frames/s. */
constexpr uint32_t UWB_POLL_PERIOD_MS = 10U;
constexpr uint32_t UWB_POLLS_PER_UPDATE =
    botbox::config::UWB_TASK_PERIOD_MS / UWB_POLL_PERIOD_MS;
static_assert(botbox::config::UWB_TASK_PERIOD_MS % UWB_POLL_PERIOD_MS == 0U,
              "The UWB task period must be a whole number of polls");
/* Free stack and CAN counters are printed once per this many results. */
constexpr uint32_t UWB_UPDATES_PER_HEALTH_REPORT = 26U; /* about 5 s */

constexpr uint32_t UWB_SENSOR_ID_FIRST = UWB_CAN_BASE_ADDRESS + UWB_CAN_LEFT_NODE;
constexpr uint32_t UWB_SENSOR_ID_LAST = UWB_CAN_BASE_ADDRESS + UWB_CAN_BACK_NODE;

StaticTask_t task_control;
StackType_t task_stack[UWB_TASK_STACK_WORDS];
uint32_t ignored_frames; /* Frames on CAN1 that are not UWB sensor packets. */

const char *mode_name(UwbFollowMode mode) {
  switch (mode) {
  case UwbFollowMode::DEFAULT:    return "3sensor";
  case UwbFollowMode::FRONT_ONLY: return "front";
  case UwbFollowMode::BACK_ONLY:  return "back";
  case UwbFollowMode::LEFT_BACK:  return "left+back";
  case UwbFollowMode::RIGHT_BACK: return "right+back";
  case UwbFollowMode::LEFT_ONLY:  return "left";
  case UwbFollowMode::RIGHT_ONLY: return "right";
  }
  return "?";
}

const char *direction_name(UwbDirection direction) {
  switch (direction) {
  case UwbDirection::UNKNOWN:  return "-";
  case UwbDirection::MID:      return "mid";
  case UwbDirection::FORWARD:  return "fwd";
  case UwbDirection::BACKWARD: return "back";
  }
  return "?";
}

const char *turn_name(UwbTurn turn) {
  switch (turn) {
  case UwbTurn::UNKNOWN: return "-";
  case UwbTurn::CENTER:  return "center";
  case UwbTurn::LEFT:    return "left";
  case UwbTurn::RIGHT:   return "right";
  }
  return "?";
}

const char *range_rule_name(UwbRangeRule rule) {
  switch (rule) {
  case UwbRangeRule::NONE:         return "none";
  case UwbRangeRule::THREE_SENSOR: return "3sensor";
  case UwbRangeRule::FRONT_ONLY:   return "front";
  case UwbRangeRule::BACK_ONLY:    return "back";
  }
  return "?";
}

const char *vector_quality_name(UwbVectorQuality quality) {
  switch (quality) {
  case UwbVectorQuality::NONE:          return "none";
  case UwbVectorQuality::THREE_SENSOR:  return "3sensor";
  case UwbVectorQuality::FRONT_ASSUMED: return "front";
  }
  return "?";
}

const char *stop_reason_name(UwbStopReason reason) {
  switch (reason) {
  case UwbStopReason::NONE:           return "none";
  case UwbStopReason::NOT_FOLLOWING:  return "not_started";
  case UwbStopReason::SENSOR_TIMEOUT: return "sensor_timeout";
  case UwbStopReason::SENSOR_LOSS:    return "sensor_loss";
  }
  return "?";
}

/* Pass every waiting UWB sensor packet to the manager and IF-5 frames to
   sys_node. Anything else on the bus is counted and dropped: the manager tells
   the sensors apart by the low bits of the identifier only and reads four
   bytes without checking. */
void drain_sensor_frames(UwbManager &manager) {
  can_frame_t frame;

  while (can_port_receive(UWB_SENSOR_PORT, &frame, 0U)) {
    if (!frame.extended && sys_node_wants(frame.id)) {
      sys_node_forward(UWB_SENSOR_PORT, &frame);
      continue;
    }
    if (pdoa_output_on_frame(frame)) {
      continue;
    }
    if (frame.extended || frame.fd || frame.id < UWB_SENSOR_ID_FIRST ||
        frame.id > UWB_SENSOR_ID_LAST || frame.length < UWB_PACKET_SIZE) {
      ignored_frames++;
      continue;
    }

    CANRxMessage message = {};
    message.nodeID = frame.id;
    message.rxTick = frame.rx_tick;
    for (size_t i = 0U; i < UWB_PACKET_SIZE; i++) {
      message.data[i] = frame.data[i];
    }
    manager.on_receive(&message);
  }
}

void print_result(UwbManager &manager, uint32_t sequence) {
  console_printf(
      "uwb %lu | valid=%u cur=%u mode=%s dir=%s%s turn=%s bturn=%s ratio=%u ang=%d sync=%u\r\n",
      static_cast<unsigned long>(sequence),
      static_cast<unsigned>(manager.is_valid()),
      static_cast<unsigned>(manager.is_steering_current()),
      mode_name(manager.follow_mode()),
      direction_name(manager.get_direction()),
      manager.is_direction_flip_pending() ? "(held)" : "",
      turn_name(manager.get_turn()), turn_name(manager.get_back_turn()),
      static_cast<unsigned>(manager.get_turn_ratio()),
      static_cast<int>(manager.get_turn_angle_deg()),
      static_cast<unsigned>(manager.is_turn_angle_sample_synchronized()));
  console_printf(
      "    dist L=%d R=%d F=%d B=%d | range %s f=%d b=%d l=%d r=%d | fault L=%u R=%u B=%u stop=%s\r\n",
      static_cast<int>(manager.left_distance()),
      static_cast<int>(manager.right_distance()),
      static_cast<int>(manager.front_distance()),
      static_cast<int>(manager.back_distance()),
      range_rule_name(manager.range_rule()),
      static_cast<int>(manager.range_front()),
      static_cast<int>(manager.range_back()),
      static_cast<int>(manager.range_left()),
      static_cast<int>(manager.range_right()),
      static_cast<unsigned>(manager.left_sensor_fault()),
      static_cast<unsigned>(manager.right_sensor_fault()),
      static_cast<unsigned>(manager.back_sensor_fault()),
      stop_reason_name(manager.get_stop_reason()));

  const UwbVector &vector = manager.vector();
  const uint32_t age_ms = manager.sample_age_ms();
  const long age = (age_ms == UWB_SAMPLE_AGE_UNKNOWN_MS)
                       ? -1L
                       : static_cast<long>(age_ms);
  if (vector.quality == UwbVectorQuality::NONE) {
    console_printf("    vector none | age=%ld ms\r\n", age);
    return;
  }
  const int bearing = vector.bearingDdeg;
  const int magnitude = (bearing < 0) ? -bearing : bearing;
  console_printf("    vector %s dist=%d bearing=%s%d.%d deg | age=%ld ms\r\n",
                 vector_quality_name(vector.quality),
                 static_cast<int>(vector.distanceMm), (bearing < 0) ? "-" : "",
                 magnitude / 10, magnitude % 10, age);
}

void print_health(UwbManager &manager) {
  can_port_stats_t stats;
  can_port_stats_t output_bus;
  const UwbOutputStats &output = uwb_output_stats();

  can_port_get_stats(UWB_SENSOR_PORT, &stats);
  can_port_get_stats(CAN_PORT_2, &output_bus);
  console_printf(
      "uwb health | rx L=%lu R=%lu B=%lu ignored=%lu | rxq_max=%lu rxq_drop=%lu fifo_lost=%lu | stack_free=%lu\r\n",
      static_cast<unsigned long>(manager.left_rx_count()),
      static_cast<unsigned long>(manager.right_rx_count()),
      static_cast<unsigned long>(manager.back_rx_count()),
      static_cast<unsigned long>(ignored_frames),
      static_cast<unsigned long>(stats.rx_queue_high_water),
      static_cast<unsigned long>(stats.rx_queue_dropped),
      static_cast<unsigned long>(stats.rx_fifo_lost),
      static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)));
  console_printf(
      "uwb output | sent=%lu skipped=%lu refused=%lu | can2 tx=%lu txfail=%lu tec=%u busoff=%lu\r\n",
      static_cast<unsigned long>(output.published),
      static_cast<unsigned long>(output.skipped_busy),
      static_cast<unsigned long>(output.refused),
      static_cast<unsigned long>(output_bus.tx_frames),
      static_cast<unsigned long>(output_bus.tx_failed),
      static_cast<unsigned>(output_bus.tx_error_count),
      static_cast<unsigned long>(output_bus.bus_off_events));
  if (pdoa_output_enabled()) {
    const PdoaOutputStats &pdoa = pdoa_output_stats();
    const pdoa::PipelineStats &pipe = pdoa_output_pipeline_stats();
    constexpr uint32_t CYCLES_PER_US = 170U;
    console_printf(
        "pdoa if0 | obs=%lu cal=%lu bad=%lu | acc=%lu dup=%lu late=%lu miss=%lu noage=%lu nocenter=%lu core_ref=%lu | core %s max=%luus last=%luus\r\n",
        static_cast<unsigned long>(pdoa.obs_frames), static_cast<unsigned long>(pdoa.cal_frames),
        static_cast<unsigned long>(pdoa.bad_frames),
        static_cast<unsigned long>(pipe.obs_accepted),
        static_cast<unsigned long>(pipe.obs_duplicate), static_cast<unsigned long>(pipe.obs_late),
        static_cast<unsigned long>(pipe.obs_missing), static_cast<unsigned long>(pipe.obs_no_rf_time),
        static_cast<unsigned long>(pipe.obs_without_center),
        static_cast<unsigned long>(pipe.obs_core_refused + pipe.range_core_refused),
        pdoa.core_ready ? "ok" : "INIT FAILED",
        static_cast<unsigned long>(pdoa.core_cycles_max / CYCLES_PER_US),
        static_cast<unsigned long>(pdoa.core_cycles_last / CYCLES_PER_US));
    console_printf(
        "pdoa 0x244 | sent=%lu refused=%lu restarts=%lu (cal=%lu boot=%lu silence=%lu sn=%lu) | replaced=%lu expired=%lu aborted=%lu fd_refused=%lu%s\r\n",
        static_cast<unsigned long>(pdoa.sent), static_cast<unsigned long>(pdoa.refused),
        static_cast<unsigned long>(pdoa.restarts), static_cast<unsigned long>(pipe.restarts_cal),
        static_cast<unsigned long>(pipe.restarts_boot_id),
        static_cast<unsigned long>(pipe.restarts_silence),
        static_cast<unsigned long>(pipe.restarts_regression),
        static_cast<unsigned long>(output_bus.latest_replaced),
        static_cast<unsigned long>(output_bus.latest_expired),
        static_cast<unsigned long>(output_bus.latest_aborted),
        static_cast<unsigned long>(output_bus.fd_refused),
        pdoa_output_suppresses_legacy() ? " (pdoa only)" : "");
  }
}

void uwb_task(void *argument) {
  UwbManager &manager = UwbManager::get_instance();
  TickType_t wake_time = xTaskGetTickCount();
  uint32_t polls = 0U;
  uint32_t sequence = 0U;

  (void)argument;

  manager.init();
  (void)manager.start();
  pdoa_output_init(xTaskGetTickCount());

  for (;;) {
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(UWB_POLL_PERIOD_MS));
    drain_sensor_frames(manager);
    // Before the 190 ms branch below: 0x244 has its own 40 ms tick.
#if BOARD_CAN2_ENABLE
    pdoa_output_poll(xTaskGetTickCount());
#endif

    polls++;
    if (polls < UWB_POLLS_PER_UPDATE) {
      continue;
    }
    polls = 0U;

    manager.update();
    sequence++;
    uwb_output_record_status(manager);
#if BOARD_CAN2_ENABLE
    uwb_output_publish(manager, static_cast<uint8_t>(sequence));
#endif
    print_result(manager, sequence);
    if ((sequence % UWB_UPDATES_PER_HEALTH_REPORT) == 0U) {
      print_health(manager);
    }
  }
}

} // namespace

void uwb_task_init(void) {
  static const osThreadAttr_t task_attributes = {
      .name = "uwb",
      .attr_bits = 0U,
      .cb_mem = &task_control,
      .cb_size = sizeof(task_control),
      .stack_mem = task_stack,
      .stack_size = sizeof(task_stack),
      .priority = osPriorityNormal,
      .tz_module = 0U,
      .reserved = 0U,
  };

  (void)osThreadNew(uwb_task, nullptr, &task_attributes);
}
