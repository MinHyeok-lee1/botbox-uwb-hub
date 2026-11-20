/**
 * @file pdoa_result.hpp
 * @brief PDoA follow result and the UWB_PDOA_TEST frame 0x244 (BBX v0.2).
 *
 * Pure code: no HAL, no RTOS. Times are milliseconds on a free-running
 * 32-bit counter (the RTOS tick at 1 kHz) and are compared wrap-safe.
 *
 * Specification: docs/BBX_PDoA_TestCAN_0x244_v0.2_20261120.md.
 */
#pragma once

#include <cstdint>

namespace pdoa {

constexpr uint32_t RESULT_FRAME_ID = 0x244U;
constexpr uint8_t RESULT_FRAME_LENGTH = 8U;

/* b0-1 of the status byte. Note the order differs from the Core's direction
   enum (0 UNKNOWN, 1 LEFT, 2 CENTER, 3 RIGHT); the adapter maps it. */
enum class Lateral : uint8_t {
  UNKNOWN = 0U,
  CENTER = 1U,
  LEFT = 2U,
  RIGHT = 3U,
};

/* b2-3 of the status byte. HELD is never produced by the UWB_V3_ONLY Core
   build used for the vehicle test, but stays encodable. */
enum class Source : uint8_t {
  NONE = 0U,
  OBSERVED = 1U,
  HELD = 2U,
  REFERENCE = 3U,
};

constexpr uint16_t DISTANCE_INVALID = 0xFFFFU;
constexpr int16_t PHASE_INVALID = 0x7FFF;
constexpr int16_t PHASE_CDEG_MIN = -18000;
constexpr int16_t PHASE_CDEG_MAX = 17999;
constexpr uint8_t QUALITY_MAX = 100U;
constexpr uint8_t QUALITY_INVALID = 0xFFU;
constexpr uint8_t RF_AGE_INVALID = 0xFFU;
constexpr uint32_t RF_AGE_UNIT_MS = 4U;
/* 0xFF means "1020 ms or more, or no observation" (0xFE = 1016 ms). */
constexpr uint32_t RF_AGE_SATURATION_MS = 0xFFU * RF_AGE_UNIT_MS;

/**
 * What the estimator says about the tag. The publisher adds the sequence
 * number, the RESTARTED bit and the age.
 */
struct Result {
  bool tag_lost = true;
  bool distance_valid = false;
  uint16_t distance_mm = DISTANCE_INVALID;
  bool lateral_valid = false;
  Lateral lateral = Lateral::UNKNOWN;
  Source source = Source::NONE;
  int16_t phase_cdeg = PHASE_INVALID;
  uint8_t quality = QUALITY_INVALID;
  /* Time of the real RF observation behind this result, in hub ms (CAN
     reception time minus the anchor's elapsed time). For a lateral result it
     is the observation that supports the lateral decision. */
  bool has_observation = false;
  uint32_t observed_ms = 0U;
};

/** True when the two results carry the same content (observation time included). */
bool same_content(const Result &a, const Result &b);

/**
 * Applies the sender guarantees of v0.2 §3 and range checks: an inconsistent
 * or out-of-range field makes that part invalid, it is never clamped into a
 * valid value.
 */
Result normalized(const Result &result);

/** rf_age byte: floor(age / 4 ms); 0xFF for no observation or >= 1020 ms. */
uint8_t encode_rf_age(bool has_observation, uint32_t observed_ms, uint32_t now_ms);

/** Packs one 0x244 frame. `result` is normalized first. */
void pack_result_frame(const Result &result, uint8_t sequence, bool restarted, uint32_t now_ms,
                       uint8_t out[RESULT_FRAME_LENGTH]);

/** Wrap-safe "a is at or after b". */
inline bool time_reached(uint32_t a, uint32_t b) {
  return static_cast<int32_t>(a - b) >= 0;
}

} // namespace pdoa
