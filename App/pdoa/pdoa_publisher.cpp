#include "pdoa_publisher.hpp"

namespace pdoa {

void Publisher::restart(uint32_t now_ms) {
  const Result lost{};
  if (started_ && !same_content(result_, lost)) {
    sequence_++;
  }
  result_ = lost;
  restarted_until_ms_ = now_ms + RESTART_WINDOW_MS;
  if (!started_) {
    next_tick_ms_ = now_ms;
    started_ = true;
  }
}

void Publisher::submit(const Result &result, bool new_observation) {
  const Result next = normalized(result);
  if (!new_observation && same_content(result_, next)) {
    return;
  }
  result_ = next;
  sequence_++;
}

bool Publisher::is_due(uint32_t now_ms) const {
  return started_ && time_reached(now_ms, next_tick_ms_);
}

void Publisher::build_frame(uint32_t now_ms, uint8_t out[RESULT_FRAME_LENGTH]) const {
  pack_result_frame(result_, sequence_, is_restarted(now_ms), now_ms, out);
}

void Publisher::mark_sent(uint32_t now_ms) {
  next_tick_ms_ += PUBLISH_PERIOD_MS;
  /* After a long stall (bus without ACK) restart the cadence from now instead
     of sending a burst of catch-up frames. */
  if (time_reached(now_ms, next_tick_ms_)) {
    next_tick_ms_ = now_ms + PUBLISH_PERIOD_MS;
  }
}

bool Publisher::is_restarted(uint32_t now_ms) const {
  return started_ && !time_reached(now_ms, restarted_until_ms_);
}

} // namespace pdoa
