/**
 * @file pdoa_publisher.hpp
 * @brief Sequence, RESTARTED window and send timing of the 0x244 frame.
 *
 * Pure code. The owner feeds results with submit() whenever the estimator
 * was evaluated, calls restart() at boot and on every estimator reset, and
 * asks is_due() on each poll. The frame is built at send time, so rf_age is
 * the age when the frame is queued.
 *
 * Rules (BBX v0.2 §2, §6 and the hub reply):
 * - seq goes up by one per new result: a new observation, or a content change
 *   without one (e.g. tag lost). Resends keep seq; only rf_age grows.
 * - RESTARTED is set for RESTART_WINDOW_MS after restart(). Its fall is not a
 *   new result.
 * - Frames go out on a fixed 40 ms tick with the latest result. A tick that
 *   could not be sent stays due; the tick advances only on mark_sent().
 */
#pragma once

#include <cstdint>

#include "pdoa_result.hpp"

namespace pdoa {

constexpr uint32_t PUBLISH_PERIOD_MS = 40U;
constexpr uint32_t RESTART_WINDOW_MS = 1000U;

class Publisher {
public:
  /** Starts with a "tag lost, no observation" result and RESTARTED set. */
  void restart(uint32_t now_ms);

  /**
   * Hands over the estimator's current result. `new_observation` is true when
   * it comes from a new RF observation (new 0x224 obs seq); otherwise seq only
   * moves if the content changed.
   */
  void submit(const Result &result, bool new_observation);

  bool is_due(uint32_t now_ms) const;

  /** Builds the frame for now. Does not change state. */
  void build_frame(uint32_t now_ms, uint8_t out[RESULT_FRAME_LENGTH]) const;

  /** The frame built for now_ms was queued. */
  void mark_sent(uint32_t now_ms);

  bool is_restarted(uint32_t now_ms) const;
  uint8_t sequence() const { return sequence_; }
  const Result &result() const { return result_; }

private:
  Result result_{};
  uint8_t sequence_ = 0U;
  bool started_ = false;
  uint32_t restarted_until_ms_ = 0U;
  uint32_t next_tick_ms_ = 0U;
};

} // namespace pdoa
