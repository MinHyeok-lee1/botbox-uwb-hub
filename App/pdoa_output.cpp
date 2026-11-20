#include "pdoa_output.hpp"

#include <cstddef>

#include "FreeRTOS.h"
#include "firmware_configuration.hpp"
#include "pdoa_publisher.hpp"
#include "pdoa_result.hpp"
#include "task.h"

namespace {

constexpr can_port_id_t OUTPUT_PORT = CAN_PORT_2;

// IF-0 frames of the PDoA anchor (UWB team, 2026-11-20). Layouts pending.
constexpr uint32_t IF0_OBSERVATION_ID = 0x224U;
constexpr uint32_t IF0_CALIBRATION_ID = 0x234U;

// A 0x244 frame that cannot leave within one publish period is replaced or
// aborted, so the receiver never gets one older than this.
constexpr uint32_t RESULT_MAX_TX_AGE_MS = pdoa::PUBLISH_PERIOD_MS;

pdoa::Publisher publisher;
PdoaOutputStats stats;
PdoaOutputSnapshot snapshot;

void record_snapshot(uint32_t now_ms) {
  const pdoa::Result &result = publisher.result();
  const bool ready = result.distance_valid && result.lateral_valid;
  taskENTER_CRITICAL();
  snapshot.has_result = true;
  snapshot.follow_ready = ready;
  snapshot.tick = now_ms;
  taskEXIT_CRITICAL();
}

uint32_t output_mode() {
  return botbox::config::firmware_configuration().pdoa_follow.output_mode;
}

} // namespace

void pdoa_output_init(uint32_t now_ms) { publisher.restart(now_ms); }

bool pdoa_output_enabled() {
  const uint32_t mode = output_mode();
  return mode == botbox::config::PDOA_OUTPUT_BOTH || mode == botbox::config::PDOA_OUTPUT_PDOA_ONLY;
}

bool pdoa_output_suppresses_legacy() {
  return output_mode() == botbox::config::PDOA_OUTPUT_PDOA_ONLY;
}

bool pdoa_output_on_frame(const can_frame_t &frame) {
  if (frame.extended || frame.fd) {
    return false;
  }
  if (frame.id == IF0_OBSERVATION_ID) {
    stats.observation_frames++;
    return true;
  }
  if (frame.id == IF0_CALIBRATION_ID) {
    stats.calibration_frames++;
    return true;
  }
  return false;
}

void pdoa_output_poll(uint32_t now_ms) {
  if (!pdoa_output_enabled()) {
    return;
  }
  record_snapshot(now_ms);
  if (!publisher.is_due(now_ms)) {
    return;
  }

  can_frame_t frame = {};
  frame.id = pdoa::RESULT_FRAME_ID;
  frame.length = pdoa::RESULT_FRAME_LENGTH;
  publisher.build_frame(now_ms, frame.data);

  if (!can_port_send_latest(OUTPUT_PORT, &frame, RESULT_MAX_TX_AGE_MS)) {
    // Stays due; retried on the next poll.
    stats.refused++;
    return;
  }
  publisher.mark_sent(now_ms);
  stats.sent++;
}

const PdoaOutputStats &pdoa_output_stats() { return stats; }

PdoaOutputSnapshot pdoa_output_snapshot() {
  taskENTER_CRITICAL();
  const PdoaOutputSnapshot copy = snapshot;
  taskEXIT_CRITICAL();
  return copy;
}
