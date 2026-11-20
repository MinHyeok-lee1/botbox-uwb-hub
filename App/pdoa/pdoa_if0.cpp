#include "pdoa_if0.hpp"

namespace pdoa {

namespace {

constexpr uint8_t OBS_PHASE_VALID = 1U << 0;
constexpr uint8_t OBS_RANGE_VALID = 1U << 1;
constexpr uint8_t OBS_REASON_SHIFT = 2U;
constexpr uint8_t OBS_REASON_MASK = 0x0FU;
constexpr uint8_t OBS_CAL_VALID = 1U << 6;
constexpr uint8_t OBS_CAL_GEN_LSB = 1U << 7;

constexpr uint8_t CAL_NVM_MISMATCH = 1U << 0;
constexpr uint8_t CAL_CAPTURE_ACTIVE = 1U << 1;
constexpr uint8_t CAL_SRC_NVM = 1U << 2;
constexpr uint8_t CAL_VALID = 1U << 3;
constexpr uint8_t CAL_DIRTY = 1U << 4;
constexpr uint8_t CAL_LR_SIGN_SHIFT = 5U;
constexpr uint8_t CAL_LR_SIGN_MASK = 0x03U;
constexpr uint8_t CAL_RESERVED = 1U << 7;
constexpr uint8_t CAL_STATE_MAX = 2U;
constexpr uint8_t LR_SIGN_RESERVED = 2U;
constexpr uint8_t BOOT_ID_NONE = 0U;

uint16_t get_u16(const uint8_t *bytes) {
  return static_cast<uint16_t>(bytes[0] | (bytes[1] << 8));
}

int32_t get_i24(const uint8_t *bytes) {
  uint32_t value = static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8) |
                   (static_cast<uint32_t>(bytes[2]) << 16);
  if (value == static_cast<uint32_t>(IF0_PHI0_INVALID)) {
    return IF0_PHI0_INVALID;
  }
  if ((value & 0x800000U) != 0U) {
    value |= 0xFF000000U;
  }
  return static_cast<int32_t>(value);
}

} // namespace

bool decode_observation(const uint8_t data[IF0_FRAME_LENGTH], Observation &out) {
  const uint8_t flags = data[1];
  out.sn = data[0];
  out.phase_valid = (flags & OBS_PHASE_VALID) != 0U;
  out.reason = static_cast<uint8_t>((flags >> OBS_REASON_SHIFT) & OBS_REASON_MASK);
  out.cal_valid = (flags & OBS_CAL_VALID) != 0U;
  out.cal_gen_lsb = (flags & OBS_CAL_GEN_LSB) != 0U;
  out.pdoa_raw = static_cast<int16_t>(get_u16(&data[2]));
  out.sts_q = data[4];
  out.distance_mm = get_u16(&data[5]);
  out.obs_age_ms = data[7];
  out.range_valid = (flags & OBS_RANGE_VALID) != 0U && out.distance_mm != IF0_DISTANCE_INVALID;

  /* The anchor never sets CAL_GEN_LSB without CAL_VALID (contract §2.1). */
  if (out.cal_gen_lsb && !out.cal_valid) {
    return false;
  }
  return true;
}

bool decode_calibration(const uint8_t data[IF0_FRAME_LENGTH], Calibration &out) {
  const uint8_t flags = data[6];
  out.state = data[0];
  out.phi0_mdeg = get_i24(&data[1]);
  out.generation = get_u16(&data[4]);
  out.nvm_mismatch = (flags & CAL_NVM_MISMATCH) != 0U;
  out.capture_active = (flags & CAL_CAPTURE_ACTIVE) != 0U;
  out.source_nvm = (flags & CAL_SRC_NVM) != 0U;
  out.cal_valid = (flags & CAL_VALID) != 0U;
  out.dirty = (flags & CAL_DIRTY) != 0U;
  out.lr_sign = static_cast<uint8_t>((flags >> CAL_LR_SIGN_SHIFT) & CAL_LR_SIGN_MASK);
  out.boot_id = data[7];

  if (out.state > CAL_STATE_MAX || (flags & CAL_RESERVED) != 0U ||
      out.lr_sign == LR_SIGN_RESERVED || out.boot_id == BOOT_ID_NONE) {
    return false;
  }
  return true;
}

} // namespace pdoa
