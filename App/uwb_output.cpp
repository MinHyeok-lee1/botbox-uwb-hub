#include "uwb_output.hpp"

#include <algorithm>
#include <cstddef>

#include "can_port.h"

namespace {

constexpr can_port_id_t OUTPUT_PORT = CAN_PORT_2;

// IF-1 message identifiers (interface specification v0.11). Fixed by the
// contract with third-party receivers: do not move them.
constexpr uint32_t UWB_RESULT_A_ID = 0x240U;
constexpr uint32_t UWB_RESULT_B_ID = 0x241U;
constexpr uint8_t UWB_RESULT_LENGTH = 8U;

// UWB_RESULT_B is optional and off by default. It becomes selectable with
// the CFG output mask (parameter 0x0003) in a later step.
constexpr bool UWB_RESULT_B_ENABLED = false;

// Invalid values of the new messages.
constexpr uint16_t INVALID_U16 = 0xFFFFU;
constexpr int16_t INVALID_I16 = 0x7FFF;

// direction field.
constexpr uint8_t DIRECTION_UNKNOWN = 0U;
constexpr uint8_t DIRECTION_FORWARD = 1U;
constexpr uint8_t DIRECTION_BACKWARD = 2U;
constexpr uint8_t DIRECTION_LEFT = 3U;
constexpr uint8_t DIRECTION_RIGHT = 4U;

// status field.
constexpr uint8_t STATUS_VALID = 1U << 0;
constexpr uint8_t STATUS_BEARING_VALID = 1U << 1;
constexpr uint8_t STATUS_TAG_LOST = 1U << 2;
constexpr uint8_t STATUS_DEGRADED = 1U << 3;
// Bit 4 BEARING_SRC stays 0: the bearing comes from the range geometry.
constexpr uint8_t STATUS_ANCHOR_SHIFT = 5U;
// Anchor numbers: 1 = left (0x201), 2 = right (0x202), 3 = back (0x203),
// the same order as the UWB_SENSOR_* bits.
constexpr uint8_t ANCHOR_MASK_BITS = 0x7U;

// quality field: provisional weights until they are set from measurements
// (specification check 7).
constexpr uint32_t QUALITY_ANCHOR_WEIGHT = 50U;
constexpr uint32_t QUALITY_FRESHNESS_WEIGHT = 30U;
constexpr uint32_t QUALITY_THREE_SENSOR_GEOMETRY = 20U;
constexpr uint32_t QUALITY_FRONT_ASSUMED_GEOMETRY = 10U;

// A result whose oldest sensor sample is older than this is not VALID. The
// sensors send about 5 Hz, so this allows two missed frames per sensor. The
// manager's receive_timeout_ms (default 10 s) is far too long for a receiver
// that, by the specification, judges validity from the status bits only.
// Becomes a CFG parameter later.
constexpr uint32_t RESULT_MAX_SAMPLE_AGE_MS = 600U;

constexpr uint32_t QUALITY_MAX = 100U;

// Degrees * 100 per bearingDdeg unit (0.1 degree).
constexpr int32_t BEARING_CDEG_PER_DDEG = 10;

UwbOutputStats stats;

void put_u16(uint8_t *bytes, uint16_t value) {
  bytes[0] = static_cast<uint8_t>(value & 0xFFU);
  bytes[1] = static_cast<uint8_t>(value >> 8);
}

void put_i16(uint8_t *bytes, int16_t value) {
  put_u16(bytes, static_cast<uint16_t>(value));
}

uint8_t bit_count(uint8_t value) {
  uint8_t count = 0U;
  for (; value != 0U; value &= static_cast<uint8_t>(value - 1U)) {
    count++;
  }
  return count;
}

uint8_t configured_sensor_mask() {
  uint8_t mask = UWB_SENSOR_LEFT | UWB_SENSOR_RIGHT;
  if (uwb_follow_configuration().back_uwb_enabled != 0U) {
    mask |= UWB_SENSOR_BACK;
  }
  return mask;
}

bool is_sample_stale(const UwbManager &manager) {
  return manager.sample_age_ms() > RESULT_MAX_SAMPLE_AGE_MS;
}

// The result may be used only while the current cycle produced it from
// recent samples.
bool is_result_valid(const UwbManager &manager) {
  return manager.is_valid() && manager.is_steering_current() &&
         !is_sample_stale(manager);
}

uint16_t to_distance(int16_t distance_mm) {
  return (distance_mm >= 0) ? static_cast<uint16_t>(distance_mm) : INVALID_U16;
}

// Nearest valid distance among the sensors in `sensors`.
uint16_t nearest_sensor_distance(const UwbManager &manager, uint8_t sensors) {
  uint16_t nearest = INVALID_U16;
  if ((sensors & UWB_SENSOR_LEFT) != 0U) {
    nearest = std::min(nearest, to_distance(manager.left_distance()));
  }
  if ((sensors & UWB_SENSOR_RIGHT) != 0U) {
    nearest = std::min(nearest, to_distance(manager.right_distance()));
  }
  if ((sensors & UWB_SENSOR_BACK) != 0U) {
    nearest = std::min(nearest, to_distance(manager.back_distance()));
  }
  return nearest;
}

// Distance from the reference point (front-sensor midpoint) to the tag.
uint16_t result_distance(const UwbManager &manager) {
  if (!is_result_valid(manager)) {
    return INVALID_U16;
  }

  const UwbVector &vector = manager.vector();
  if (vector.quality != UwbVectorQuality::NONE && vector.distanceMm >= 0) {
    return static_cast<uint16_t>(vector.distanceMm);
  }

  // No vector: the samples are not synchronized, the geometry check failed
  // or the mode does not give one. With both front sensors the distance from
  // the reference point follows from them directly (unsmoothed). Otherwise
  // the nearest sensor the mode uses is the best estimate; its reference
  // point is that sensor, and DEGRADED is set in these modes.
  const UwbFollowMode mode = manager.follow_mode();
  if (mode == UwbFollowMode::DEFAULT || mode == UwbFollowMode::FRONT_ONLY) {
    return to_distance(manager.front_distance());
  }
  return nearest_sensor_distance(manager, manager.used_sensor_mask());
}

// Bearing in 0.01 degree. Sent whenever there is a vector; BEARING_VALID
// tells whether the geometry is complete.
int16_t result_bearing(const UwbManager &manager) {
  const UwbVector &vector = manager.vector();
  if (!is_result_valid(manager) || vector.quality == UwbVectorQuality::NONE) {
    return INVALID_I16;
  }
  return static_cast<int16_t>(static_cast<int32_t>(vector.bearingDdeg) *
                              BEARING_CDEG_PER_DDEG);
}

// Only the three-sensor solution places the tag unambiguously; the front
// pair assumes the tag is in front.
bool is_bearing_valid(const UwbManager &manager) {
  return is_result_valid(manager) &&
         manager.vector().quality == UwbVectorQuality::THREE_SENSOR;
}

// Same criterion as botbox+: front/back from the distance difference. A tag
// level with the cart (MID) is reported on the side the turn points to; in
// LEFT_BACK / RIGHT_BACK this is the side sensor that is closer than the back
// one, so it is a turn-in-place hint rather than a measured side.
uint8_t result_direction(const UwbManager &manager) {
  if (!is_result_valid(manager)) {
    return DIRECTION_UNKNOWN;
  }

  switch (manager.get_direction()) {
  case UwbDirection::FORWARD:
    return DIRECTION_FORWARD;
  case UwbDirection::BACKWARD:
    return DIRECTION_BACKWARD;
  case UwbDirection::MID:
    if (manager.get_turn() == UwbTurn::LEFT) {
      return DIRECTION_LEFT;
    }
    if (manager.get_turn() == UwbTurn::RIGHT) {
      return DIRECTION_RIGHT;
    }
    return DIRECTION_UNKNOWN;
  case UwbDirection::UNKNOWN:
    return DIRECTION_UNKNOWN;
  }
  return DIRECTION_UNKNOWN;
}

uint8_t result_quality(const UwbManager &manager) {
  if (!is_result_valid(manager)) {
    return 0U;
  }

  // At least the two front sensors are always configured.
  const uint32_t configured = bit_count(configured_sensor_mask());
  const uint32_t used =
      bit_count(static_cast<uint8_t>(manager.used_sensor_mask() &
                                     configured_sensor_mask()));
  uint32_t quality = (QUALITY_ANCHOR_WEIGHT * used) / configured;

  const uint32_t timeout = uwb_follow_configuration().receive_timeout_ms;
  const uint32_t age = manager.sample_age_ms();
  if (age < timeout) {
    quality += (QUALITY_FRESHNESS_WEIGHT * (timeout - age)) / timeout;
  }

  const UwbVector &vector = manager.vector();
  switch (vector.quality) {
  case UwbVectorQuality::THREE_SENSOR:
    // Full weight for distances that agree, none at the rejection limit.
    if (vector.residualMm < UWB_VECTOR_RESIDUAL_MAX_MM) {
      quality += (QUALITY_THREE_SENSOR_GEOMETRY *
                  (UWB_VECTOR_RESIDUAL_MAX_MM - vector.residualMm)) /
                 UWB_VECTOR_RESIDUAL_MAX_MM;
    }
    break;
  case UwbVectorQuality::FRONT_ASSUMED:
    quality += QUALITY_FRONT_ASSUMED_GEOMETRY;
    break;
  case UwbVectorQuality::NONE:
    break;
  }
  return static_cast<uint8_t>(std::min(quality, QUALITY_MAX));
}

uint8_t result_status(const UwbManager &manager) {
  if (!is_result_valid(manager)) {
    const UwbStopReason reason = manager.get_stop_reason();
    const bool tag_lost = reason == UwbStopReason::SENSOR_LOSS ||
                          reason == UwbStopReason::SENSOR_TIMEOUT ||
                          (manager.is_valid() && is_sample_stale(manager));
    if (tag_lost) {
      return STATUS_TAG_LOST;
    }
    // Still following but no current result: the mode is changing because
    // a sensor dropped out.
    return manager.is_valid() ? STATUS_DEGRADED : 0U;
  }

  const uint8_t used = manager.used_sensor_mask();
  uint8_t status = STATUS_VALID;
  if (is_bearing_valid(manager)) {
    status |= STATUS_BEARING_VALID;
  }
  if ((configured_sensor_mask() & ~used) != 0U) {
    status |= STATUS_DEGRADED;
  }
  status |= static_cast<uint8_t>((used & ANCHOR_MASK_BITS)
                                 << STATUS_ANCHOR_SHIFT);
  return status;
}

void pack_result_a(const UwbManager &manager, uint8_t sequence,
                   uint8_t data[UWB_RESULT_LENGTH]) {
  put_u16(&data[0], result_distance(manager));
  put_i16(&data[2], result_bearing(manager));
  data[4] = result_direction(manager);
  data[5] = result_quality(manager);
  data[6] = result_status(manager);
  data[7] = sequence;
}

void pack_result_b(const UwbManager &manager, uint8_t sequence,
                   uint8_t data[UWB_RESULT_LENGTH]) {
  const bool valid = is_bearing_valid(manager);
  const UwbVector &vector = manager.vector();
  put_i16(&data[0], valid ? vector.xMm : INVALID_I16);
  put_i16(&data[2], valid ? vector.yMm : INVALID_I16);
  put_i16(&data[4], INVALID_I16); // range_rate: reserved
  data[6] = 0U;                   // reserved: tag ID candidate
  data[7] = sequence;
}

bool send_result(uint32_t id, const uint8_t data[UWB_RESULT_LENGTH]) {
  can_frame_t frame = {};
  frame.id = id;
  frame.extended = false;
  frame.fd = false;
  frame.brs = false;
  frame.length = UWB_RESULT_LENGTH;
  for (size_t i = 0U; i < UWB_RESULT_LENGTH; i++) {
    frame.data[i] = data[i];
  }
  return can_port_send(OUTPUT_PORT, &frame, 0U);
}

} // namespace

void uwb_output_publish(const UwbManager &manager, uint8_t sequence) {
  // Nothing acknowledges the frames while no receiver is on the bus, so they
  // wait in the queue. Skipping keeps at most one old publication waiting
  // instead of a backlog that would be sent late.
  if (can_port_tx_pending(OUTPUT_PORT) != 0U) {
    stats.skipped_busy++;
    return;
  }

  uint8_t data[UWB_RESULT_LENGTH];
  pack_result_a(manager, sequence, data);
  if (!send_result(UWB_RESULT_A_ID, data)) {
    stats.refused++;
    return;
  }

  if (UWB_RESULT_B_ENABLED) {
    pack_result_b(manager, sequence, data);
    if (!send_result(UWB_RESULT_B_ID, data)) {
      stats.refused++;
      return;
    }
  }
  stats.published++;
}

const UwbOutputStats &uwb_output_stats() { return stats; }
