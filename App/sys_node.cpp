/**
 * @file sys_node.cpp
 * @brief IF-5 phase 1 of the hub: heartbeat, CFG READ, ESTOP observation.
 */
#include "sys_node.h"

#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "board_config.h"
#include "cmsis_os.h"
#include "console.h"
#include "fdcan.h"
#include "firmware_configuration.hpp"
#include "if5_codec.hpp"
#include "if5_node.hpp"
#include "pdoa_output.hpp"
#include "pdoa_publisher.hpp"
#include "queue.h"
#include "task.h"
#include "uwb_output.hpp"
#include "version.h"

static_assert(configTICK_RATE_HZ == 1000U, "sys_node treats one tick as one millisecond");

namespace {

constexpr uint8_t OWN_NODE = if5::NODE_ID_COLLECTION_BOARD;

constexpr uint32_t TASK_STACK_WORDS = 512U;
constexpr uint32_t POLL_PERIOD_MS = 10U;
constexpr uint32_t HEARTBEAT_PERIOD_MS = 100U;
constexpr uint32_t HEALTH_REPORT_PERIOD_MS = 5000U;
constexpr uint32_t MS_PER_S = 1000U;
constexpr uint32_t KBPS = 1000U;

// Frames handled per port per poll, so one busy bus cannot starve the other
// or the heartbeat.
constexpr uint32_t MAX_FRAMES_PER_POLL = 16U;
constexpr uint32_t INBOX_LENGTH = 8U;

// A result older than this many UWB periods means the UWB task has stalled.
constexpr uint32_t RESULT_STALE_PERIODS = 3U;

// TX admission per sender on a port (see uwb_output.cpp PUBLISH_MAX_PENDING):
// a heartbeat waits for an empty port so it never delays a UWB result; CFG
// replies are rare and may queue behind a few frames.
constexpr uint32_t HEARTBEAT_MAX_PENDING = 1U;
constexpr uint32_t CFG_REPLY_MAX_PENDING = 3U;

const char *const PORT_NAMES[CAN_PORT_COUNT] = {"can1", "can2"};

struct InboxEntry {
  can_port_id_t port;
  uint32_t tick;
  if5::RawFrame frame;
};

struct PortCounters {
  uint32_t heartbeat_sent;
  uint32_t heartbeat_skipped; // port busy, bus-off or refused
  uint32_t cfg_requests;
  uint32_t cfg_replies_dropped;
  uint32_t rejected; // IF-5 identifier with a malformed frame
  uint32_t ignored;  // not an IF-5 frame (CAN2 only; CAN1 is filtered by uwb_task)
  uint32_t own_id_heard; // another node uses our heartbeat ID
};

StaticTask_t task_control;
StackType_t task_stack[TASK_STACK_WORDS];
QueueHandle_t inbox;
StaticQueue_t inbox_control;
uint8_t inbox_storage[INBOX_LENGTH * sizeof(InboxEntry)];
volatile uint32_t inbox_dropped;

PortCounters counters[CAN_PORT_COUNT];
if5::EstopMonitor estop_monitor;
if5::PeerTable peers;
uint32_t uptime_ms;

if5::RawFrame to_raw(const can_frame_t &frame) {
  if5::RawFrame raw = {};
  raw.id = frame.id;
  raw.extended = frame.extended;
  raw.fd = frame.fd;
  raw.length = frame.length;
  for (size_t i = 0U; i < sizeof(raw.data) && i < frame.length; i++) {
    raw.data[i] = frame.data[i];
  }
  return raw;
}

uint16_t external_bitrate_kbps() {
  const FDCAN_InitTypeDef &init = hfdcan2.Init;
  const uint32_t clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
  const uint32_t quanta = 1U + init.NominalTimeSeg1 + init.NominalTimeSeg2;
  return static_cast<uint16_t>(clock / (init.NominalPrescaler * quanta) / KBPS);
}

bool port_usable(can_port_id_t port, const can_port_stats_t &stats) {
  (void)port;
  return stats.enabled && stats.started && !stats.start_failed && !stats.bus_off;
}

bool send_frame(can_port_id_t port, uint32_t id, const uint8_t *data, uint8_t length,
                uint32_t max_pending) {
  if (can_port_tx_pending(port) >= max_pending) {
    return false;
  }
  can_frame_t frame = {};
  frame.id = id;
  frame.length = length;
  for (size_t i = 0U; i < length; i++) {
    frame.data[i] = data[i];
  }
  return can_port_send(port, &frame, 0U);
}

uint8_t error_count() {
  uint32_t errors = inbox_dropped;
  for (size_t i = 0U; i < CAN_PORT_COUNT; i++) {
    can_port_stats_t stats;
    can_port_get_stats(static_cast<can_port_id_t>(i), &stats);
    errors += counters[i].rejected + stats.rx_fifo_lost + stats.rx_queue_dropped;
  }
  return if5::saturate_u8(errors);
}

if5::NodeState current_state(uint32_t now) {
  bool port_fault = false;
  for (size_t i = 0U; i < CAN_PORT_COUNT; i++) {
    can_port_stats_t stats;
    can_port_get_stats(static_cast<can_port_id_t>(i), &stats);
    port_fault = port_fault || (stats.enabled && (stats.start_failed || stats.bus_off));
  }

  // PDOA_ONLY: 0x240 is not sent, so the state follows the 0x244 result
  // (provisional for the PDoA vehicle test; IF-5 v0.11 defines it on 0x240).
  if (pdoa_output_suppresses_legacy()) {
    const PdoaOutputSnapshot pdoa = pdoa_output_snapshot();
    const uint32_t stale_ms = RESULT_STALE_PERIODS * pdoa::PUBLISH_PERIOD_MS;
    const if5::StateInputs inputs = {
        pdoa.has_result,
        pdoa.has_result && (now - pdoa.tick) > stale_ms,
        static_cast<uint8_t>(pdoa.follow_ready ? if5::RESULT_STATUS_VALID : 0U),
        port_fault,
    };
    return if5::heartbeat_state(inputs);
  }

  const UwbOutputSnapshot result = uwb_output_snapshot();
  const uint32_t stale_ms = RESULT_STALE_PERIODS * botbox::config::UWB_TASK_PERIOD_MS;
  const if5::StateInputs inputs = {
      result.has_result,
      result.has_result && (now - result.tick) > stale_ms,
      result.status,
      port_fault,
  };
  return if5::heartbeat_state(inputs);
}

void send_heartbeats(uint32_t now) {
  const if5::Heartbeat heartbeat = {
      current_state(now),
      if5::PROTOCOL_VERSION,
      static_cast<uint16_t>(APP_BUILD),
      if5::LEVEL_L1,
      error_count(),
      static_cast<uint16_t>(uptime_ms / MS_PER_S),
  };
  uint8_t data[if5::HEARTBEAT_LENGTH];
  if5::pack_heartbeat(heartbeat, data);

  for (size_t i = 0U; i < CAN_PORT_COUNT; i++) {
    const can_port_id_t port = static_cast<can_port_id_t>(i);
    can_port_stats_t stats;
    can_port_get_stats(port, &stats);
    if (!stats.enabled) {
      continue;
    }
#if !BOARD_IF5_HEARTBEAT_CAN1
    if (port == CAN_PORT_1) {
      continue;
    }
#endif
    if (port_usable(port, stats) &&
        send_frame(port, if5::heartbeat_id(OWN_NODE), data, if5::HEARTBEAT_LENGTH,
                   HEARTBEAT_MAX_PENDING)) {
      counters[i].heartbeat_sent++;
    } else {
      counters[i].heartbeat_skipped++;
    }
  }
}

void handle_estop(const if5::Estop &estop, uint32_t tick) {
  // Observation only: no latch and no RELEASE authority until the protocol
  // document defines who may release in the L1 integration configuration.
  estop_monitor.on_estop(estop, tick);
  console_printf("if5: estop %s from node 0x%02X reason %u seq %u\r\n",
                 (estop.command == if5::EstopCommand::STOP) ? "STOP" : "RELEASE",
                 static_cast<unsigned>(estop.node), static_cast<unsigned>(estop.reason),
                 static_cast<unsigned>(estop.sequence));
}

void handle_cfg(can_port_id_t port, const if5::CfgRequest &request) {
  // Output rate: the 0x244 period in PDOA_ONLY, the 0x240 period otherwise.
  const uint32_t output_period_ms = pdoa_output_suppresses_legacy()
                                        ? pdoa::PUBLISH_PERIOD_MS
                                        : botbox::config::UWB_TASK_PERIOD_MS;
  const if5::NodeInfo info = {
      static_cast<uint16_t>(output_period_ms),
      OWN_NODE,
      external_bitrate_kbps(),
  };
  const if5::CfgResponse response = if5::answer_cfg(request, info);
  uint8_t data[if5::CFG_LENGTH];
  if5::pack_cfg_response(response, data);

  counters[port].cfg_requests++;
  if (!send_frame(port, if5::cfg_response_id(OWN_NODE), data, if5::CFG_LENGTH,
                  CFG_REPLY_MAX_PENDING)) {
    counters[port].cfg_replies_dropped++;
  }
}

void handle_frame(can_port_id_t port, const if5::RawFrame &frame, uint32_t tick) {
  if (if5::is_estop_id(frame.id)) {
    if5::Estop estop;
    if (!if5::parse_estop(frame, estop)) {
      counters[port].rejected++;
      return;
    }
    handle_estop(estop, tick);
    return;
  }

  if (if5::is_heartbeat_id(frame.id)) {
    uint8_t node;
    if5::NodeState state;
    if (!if5::parse_heartbeat(frame, node, state)) {
      counters[port].rejected++;
      return;
    }
    if (node == OWN_NODE) {
      counters[port].own_id_heard++;
      return;
    }
    peers.on_heartbeat(static_cast<uint8_t>(port), node, state, tick);
    return;
  }

  if (frame.id == if5::cfg_request_id(OWN_NODE)) {
    if5::CfgRequest request;
    if (!if5::parse_cfg_request(frame, OWN_NODE, request)) {
      counters[port].rejected++;
      return;
    }
    handle_cfg(port, request);
    return;
  }

  counters[port].ignored++;
}

void drain_inbox() {
  InboxEntry entry;
  for (uint32_t i = 0U; i < MAX_FRAMES_PER_POLL && xQueueReceive(inbox, &entry, 0U) == pdTRUE;
       i++) {
    handle_frame(entry.port, entry.frame, entry.tick);
  }
}

void drain_external_bus() {
  can_frame_t frame;
  for (uint32_t i = 0U; i < MAX_FRAMES_PER_POLL && can_port_receive(CAN_PORT_2, &frame, 0U);
       i++) {
    handle_frame(CAN_PORT_2, to_raw(frame), frame.rx_tick);
  }
}

void print_health(uint32_t now) {
  for (size_t i = 0U; i < CAN_PORT_COUNT; i++) {
    const PortCounters &c = counters[i];
    console_printf("if5 %s: hb sent %lu skip %lu, cfg req %lu drop %lu, rejected %lu, "
                   "ignored %lu, own id %lu, peers",
                   PORT_NAMES[i], static_cast<unsigned long>(c.heartbeat_sent),
                   static_cast<unsigned long>(c.heartbeat_skipped),
                   static_cast<unsigned long>(c.cfg_requests),
                   static_cast<unsigned long>(c.cfg_replies_dropped),
                   static_cast<unsigned long>(c.rejected), static_cast<unsigned long>(c.ignored),
                   static_cast<unsigned long>(c.own_id_heard));
    for (uint8_t node = 0U; node < if5::PeerTable::NODE_COUNT; node++) {
      if (peers.ever_seen(static_cast<uint8_t>(i), node)) {
        console_printf(" 0x%02X:%s/%u", static_cast<unsigned>(node),
                       peers.is_online(static_cast<uint8_t>(i), node, now) ? "on" : "OFF",
                       static_cast<unsigned>(peers.state(static_cast<uint8_t>(i), node)));
      }
    }
    console_printf("\r\n");
  }
  console_printf("if5: state %u, err %u, estop stop %lu release %lu%s, inbox drop %lu, "
                 "stack free %lu\r\n",
                 static_cast<unsigned>(current_state(now)), static_cast<unsigned>(error_count()),
                 static_cast<unsigned long>(estop_monitor.stop_count()),
                 static_cast<unsigned long>(estop_monitor.release_count()),
                 estop_monitor.stop_seen(now) ? " (STOP active)" : "",
                 static_cast<unsigned long>(inbox_dropped),
                 static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)));
}

void sys_task(void *argument) {
  (void)argument;
  TickType_t wake_time = xTaskGetTickCount();
  uint32_t last_tick = wake_time;
  uint32_t next_heartbeat = wake_time;
  uint32_t next_report = wake_time + HEALTH_REPORT_PERIOD_MS;

  for (;;) {
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(POLL_PERIOD_MS));
    const uint32_t now = xTaskGetTickCount();
    uptime_ms += now - last_tick; // survives the 32-bit tick wrap
    last_tick = now;

    drain_inbox();
#if BOARD_CAN2_ENABLE
    drain_external_bus();
#endif

    if (static_cast<int32_t>(now - next_heartbeat) >= 0) {
      next_heartbeat += HEARTBEAT_PERIOD_MS;
      send_heartbeats(now);
    }
    if (static_cast<int32_t>(now - next_report) >= 0) {
      next_report += HEALTH_REPORT_PERIOD_MS;
      print_health(now);
    }
  }
}

} // namespace

bool sys_node_init(void) {
  static const osThreadAttr_t task_attributes = {
      .name = "sys",
      .attr_bits = 0U,
      .cb_mem = &task_control,
      .cb_size = sizeof(task_control),
      .stack_mem = task_stack,
      .stack_size = sizeof(task_stack),
      .priority = osPriorityNormal,
      .tz_module = 0U,
      .reserved = 0U,
  };

  inbox = xQueueCreateStatic(INBOX_LENGTH, sizeof(InboxEntry), inbox_storage, &inbox_control);
  if (inbox == nullptr) {
    return false;
  }
  return osThreadNew(sys_task, nullptr, &task_attributes) != nullptr;
}

bool sys_node_wants(uint32_t id) { return if5::is_if5_rx_id(id, OWN_NODE); }

void sys_node_forward(can_port_id_t port, const can_frame_t *frame) {
  if (inbox == nullptr || frame == nullptr || port >= CAN_PORT_COUNT) {
    return;
  }
  const InboxEntry entry = {port, frame->rx_tick, to_raw(*frame)};
  if (xQueueSend(inbox, &entry, 0U) != pdTRUE) {
    inbox_dropped++;
  }
}
