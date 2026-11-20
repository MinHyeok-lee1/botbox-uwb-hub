#include "pdoa_output.hpp"

#include <cstddef>

#include "FreeRTOS.h"
#include "firmware_configuration.hpp"
#include "pdoa_if0.hpp"
#include "pdoa_publisher.hpp"
#include "pdoa_result.hpp"
#include "stm32g4xx.h"
#include "task.h"

namespace {

constexpr can_port_id_t OUTPUT_PORT = CAN_PORT_2;

// rf_age is computed when the frame is built; a frame that has not left this
// long after that is replaced or aborted, so rf_age under-reports by at most
// this much (on a free bus a frame leaves within about 0.3 ms).
constexpr uint32_t RESULT_MAX_TX_AGE_MS = 20U;

// The Core instance is about 12.5 kB: static storage, owned by the UWB task.
pdoa::Pipeline pipeline;
pdoa::Publisher publisher;
PdoaOutputStats stats;
PdoaOutputSnapshot snapshot;

uint32_t output_mode() {
  return botbox::config::firmware_configuration().pdoa_follow.output_mode;
}

void start_cycle_counter() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void record_snapshot(uint32_t now_ms) {
  const pdoa::Result &result = publisher.result();
  const bool ready = result.distance_valid && result.lateral_valid;
  taskENTER_CRITICAL();
  snapshot.has_result = true;
  snapshot.follow_ready = ready;
  snapshot.tick = now_ms;
  taskEXIT_CRITICAL();
}

// The RESTARTED window starts when the reset runs: a frame may have waited in
// the RX queue, so its RX time can be in the past.
void restart_publisher() {
  publisher.restart(xTaskGetTickCount());
  stats.restarts++;
}

void handle_calibration(const can_frame_t &frame) {
  stats.cal_frames++;
  pdoa::Calibration cal;
  if (!pdoa::is_if0_frame_shape(frame.extended, frame.fd, frame.length) ||
      !pdoa::decode_calibration(frame.data, cal)) {
    stats.bad_frames++;
    return;
  }
  if (pipeline.on_calibration(cal, frame.rx_tick)) {
    restart_publisher();
  }
}

void handle_observation(const can_frame_t &frame) {
  stats.obs_frames++;
  pdoa::Observation obs;
  if (!pdoa::is_if0_frame_shape(frame.extended, frame.fd, frame.length) ||
      !pdoa::decode_observation(frame.data, obs)) {
    stats.bad_frames++;
    return;
  }

  const uint32_t start = DWT->CYCCNT;
  const pdoa::Pipeline::ObservationOutcome outcome = pipeline.on_observation(obs, frame.rx_tick);
  if (outcome.restarted) {
    restart_publisher();
  }
  if (!outcome.accepted) {
    return;
  }
  // One new RF observation = one new 0x244 result.
  publisher.submit(pipeline.read(frame.rx_tick), true);
  const uint32_t cycles = DWT->CYCCNT - start;
  stats.core_cycles_last = cycles;
  if (cycles > stats.core_cycles_max) {
    stats.core_cycles_max = cycles;
  }
}

} // namespace

void pdoa_output_init(uint32_t now_ms) {
  start_cycle_counter();
  stats.core_ready = pipeline.init();
  publisher.restart(now_ms);
}

bool pdoa_output_enabled() {
  const uint32_t mode = output_mode();
  return mode == botbox::config::PDOA_OUTPUT_BOTH || mode == botbox::config::PDOA_OUTPUT_PDOA_ONLY;
}

bool pdoa_output_suppresses_legacy() {
  return output_mode() == botbox::config::PDOA_OUTPUT_PDOA_ONLY;
}

bool pdoa_output_on_frame(const can_frame_t &frame) {
  if (frame.extended || (frame.id != pdoa::IF0_OBS_ID && frame.id != pdoa::IF0_CAL_ID)) {
    return false;
  }
  if (!pdoa_output_enabled() || !stats.core_ready) {
    return true; // ours, but the PDoA path is off
  }
  if (frame.id == pdoa::IF0_CAL_ID) {
    handle_calibration(frame);
  } else {
    handle_observation(frame);
  }
  return true;
}

void pdoa_output_poll(uint32_t now_ms) {
  if (!pdoa_output_enabled()) {
    return;
  }
  // Tag lost and CAL timeout happen without new input.
  if (stats.core_ready) {
    publisher.submit(pipeline.read(now_ms), false);
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

const pdoa::PipelineStats &pdoa_output_pipeline_stats() { return pipeline.stats(); }

PdoaOutputSnapshot pdoa_output_snapshot() {
  taskENTER_CRITICAL();
  const PdoaOutputSnapshot copy = snapshot;
  taskEXIT_CRITICAL();
  return copy;
}
