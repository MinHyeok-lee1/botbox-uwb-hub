/**
 * @file pdoa_if0.hpp
 * @brief IF-0 R1 frames of the PDoA anchor: OBS_RAW 0x224 and CAL_STATE 0x234.
 *
 * Pure decoding, no HAL. Contract: docs/IF0_BYTE_CONTRACT_20261120.md
 * (research contract R1, anchor AID 0x2001).
 */
#pragma once

#include <cstdint>

namespace pdoa {

constexpr uint32_t IF0_OBS_ID = 0x224U;
constexpr uint32_t IF0_CAL_ID = 0x234U;
constexpr uint8_t IF0_FRAME_LENGTH = 8U;

constexpr int16_t IF0_PDOA_INVALID = 0x7FFF;
constexpr uint8_t IF0_STS_Q_INVALID = 0xFFU;
constexpr uint16_t IF0_DISTANCE_INVALID = 0xFFFFU;
constexpr uint8_t IF0_OBS_AGE_SATURATED = 0xFEU; /* 254 ms or more */
constexpr uint8_t IF0_OBS_AGE_UNKNOWN = 0xFFU;
constexpr int32_t IF0_PHI0_INVALID = 0x7FFFFF;
constexpr int32_t IF0_PHI0_LIMIT_MDEG = 180000;

/* LR_SIGN codes of CAL_STATE b5-6. The Core applies -1 itself. */
constexpr uint8_t IF0_LR_SIGN_UNKNOWN = 0U;
constexpr uint8_t IF0_LR_SIGN_PLUS = 1U;
constexpr uint8_t IF0_LR_SIGN_MINUS = 3U;

struct Observation {
  uint8_t sn;
  bool phase_valid;   /* b0 */
  bool range_valid;   /* b1, and the distance is not 0xFFFF */
  uint8_t reason;     /* b2-5: Q_FAIL, STS0_FAIL, STS1_FAIL, ADC_WARN */
  bool cal_valid;     /* b6 */
  bool cal_gen_lsb;   /* b7 */
  int16_t pdoa_raw;   /* rad * 2048, 0x7FFF = not read */
  uint8_t sts_q;      /* 0xFF invalid, 0xFE saturated */
  uint16_t distance_mm; /* 0xFFFE = 65534 mm or more */
  uint8_t obs_age_ms; /* 0xFE >= 254 ms, 0xFF unknown */
};

struct Calibration {
  uint8_t state;      /* 0 INVALID, 1 CENTER_ONLY, 2 CENTER_SIGN */
  int32_t phi0_mdeg;  /* sign-extended i24, 0x7FFFFF invalid */
  uint16_t generation;
  bool nvm_mismatch;
  bool capture_active;
  bool source_nvm;
  bool cal_valid;     /* b3 */
  bool dirty;
  uint8_t lr_sign;    /* code, see IF0_LR_SIGN_* */
  uint8_t boot_id;    /* 1-255 */

  /** Fields whose change resets the estimator (contract §4.3). */
  bool same_identity(const Calibration &other) const {
    return state == other.state && phi0_mdeg == other.phi0_mdeg &&
           generation == other.generation && lr_sign == other.lr_sign &&
           boot_id == other.boot_id;
  }

  /** A center the Core can use: valid state, value in range, sign -1. */
  bool usable() const {
    return state != 0U && cal_valid && phi0_mdeg != IF0_PHI0_INVALID &&
           phi0_mdeg >= -IF0_PHI0_LIMIT_MDEG && phi0_mdeg <= IF0_PHI0_LIMIT_MDEG &&
           lr_sign == IF0_LR_SIGN_MINUS;
  }
};

/* Frame checks the caller does before decoding (contract §1). */
inline bool is_if0_frame_shape(bool extended, bool fd, uint8_t length) {
  return !extended && !fd && length == IF0_FRAME_LENGTH;
}

/** Decodes OBS_RAW. Returns false for values the contract never sends. */
bool decode_observation(const uint8_t data[IF0_FRAME_LENGTH], Observation &out);

/** Decodes CAL_STATE. Returns false for a reserved code or boot_id 0. */
bool decode_calibration(const uint8_t data[IF0_FRAME_LENGTH], Calibration &out);

} // namespace pdoa
