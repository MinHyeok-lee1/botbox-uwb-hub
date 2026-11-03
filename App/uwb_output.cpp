#include "uwb_output.hpp"

#include <cstddef>

#include "board_config.h"
#include "can_port.h"
#include "firmware_configuration.hpp"

namespace {

using botbox::config::CanOutputFrameFormat;

constexpr can_port_id_t OUTPUT_PORT = CAN_PORT_2;
constexpr uint32_t OUTPUT_CAN_ID_BASE = BOARD_UWB_OUTPUT_CAN_ID_BASE;
constexpr uint8_t PROTOCOL_VERSION = 1U;
constexpr size_t BLOCK_SIZE = 8U;
constexpr size_t BLOCK_COUNT = 3U;
static_assert(BLOCK_SIZE * BLOCK_COUNT <= CAN_FRAME_MAX_LENGTH,
              "The blocks must fit one CAN FD frame");

// Block 0 flags.
constexpr uint8_t FLAG_TRACKING_VALID = 1U << 0;
constexpr uint8_t FLAG_STEERING_CURRENT = 1U << 1;
constexpr uint8_t FLAG_DIRECTION_FLIP_PENDING = 1U << 2;
constexpr uint8_t FLAG_ANGLE_SAMPLES_SYNCHRONIZED = 1U << 3;

// Block 0 hub status.
constexpr uint8_t HUB_CONFIGURATION_FROM_FLASH = 1U << 0;
constexpr uint8_t HUB_SENSOR_BUS_FAULT = 1U << 1;

// Block 2 sample age: 10 ms units, 254 = 2.54 s or older, 255 = unknown.
constexpr uint32_t SAMPLE_AGE_UNIT_MS = 10U;
constexpr uint8_t SAMPLE_AGE_MAX = 254U;
constexpr uint8_t SAMPLE_AGE_UNKNOWN = 255U;

UwbOutputStats stats;

void put_i16(uint8_t *bytes, int16_t value) {
  const uint16_t raw = static_cast<uint16_t>(value);
  bytes[0] = static_cast<uint8_t>(raw & 0xFFU);
  bytes[1] = static_cast<uint8_t>(raw >> 8);
}

uint8_t hub_status() {
  uint8_t status = 0U;
  if (botbox::config::firmware_configuration_status() ==
      botbox::config::ConfigurationLoadStatus::LOADED_FROM_FLASH) {
    status |= HUB_CONFIGURATION_FROM_FLASH;
  }

  can_port_stats_t sensor_bus;
  can_port_get_stats(CAN_PORT_1, &sensor_bus);
  if (!sensor_bus.started || sensor_bus.bus_off) {
    status |= HUB_SENSOR_BUS_FAULT;
  }
  return status;
}

uint8_t sample_age_code(uint32_t age_ms) {
  if (age_ms == UWB_SAMPLE_AGE_UNKNOWN_MS) {
    return SAMPLE_AGE_UNKNOWN;
  }
  const uint32_t units = age_ms / SAMPLE_AGE_UNIT_MS;
  return (units > SAMPLE_AGE_MAX) ? SAMPLE_AGE_MAX : static_cast<uint8_t>(units);
}

// Writes the three 8-byte blocks back to back into `payload`.
void pack_blocks(const UwbManager &manager, uint8_t sequence,
                 uint8_t payload[BLOCK_SIZE * BLOCK_COUNT]) {
  uint8_t *block0 = &payload[0];
  uint8_t *block1 = &payload[BLOCK_SIZE];
  uint8_t *block2 = &payload[2U * BLOCK_SIZE];
  const UwbVector &vector = manager.vector();

  uint8_t flags = 0U;
  if (manager.is_valid()) {
    flags |= FLAG_TRACKING_VALID;
  }
  if (manager.is_steering_current()) {
    flags |= FLAG_STEERING_CURRENT;
  }
  if (manager.is_direction_flip_pending()) {
    flags |= FLAG_DIRECTION_FLIP_PENDING;
  }
  if (manager.is_turn_angle_sample_synchronized()) {
    flags |= FLAG_ANGLE_SAMPLES_SYNCHRONIZED;
  }

  block0[0] = sequence;
  block0[1] = PROTOCOL_VERSION;
  block0[2] = flags;
  block0[3] = static_cast<uint8_t>(manager.follow_mode());
  block0[4] = static_cast<uint8_t>(manager.get_stop_reason());
  block0[5] = static_cast<uint8_t>(
      (static_cast<uint8_t>(manager.get_direction()) & 0x3U) |
      ((static_cast<uint8_t>(manager.get_turn()) & 0x3U) << 2) |
      ((static_cast<uint8_t>(manager.get_back_turn()) & 0x3U) << 4));
  block0[6] = manager.get_turn_ratio();
  block0[7] = hub_status();

  block1[0] = sequence;
  block1[1] = static_cast<uint8_t>(
      (static_cast<uint8_t>(manager.range_rule()) & 0xFU) |
      ((static_cast<uint8_t>(vector.quality) & 0xFU) << 4));
  put_i16(&block1[2], manager.range_front());
  put_i16(&block1[4], manager.range_back());
  put_i16(&block1[6], manager.range_left());

  block2[0] = sequence;
  block2[1] = sample_age_code(manager.sample_age_ms());
  put_i16(&block2[2], manager.range_right());
  put_i16(&block2[4], vector.distanceMm);
  put_i16(&block2[6], vector.bearingDdeg);
}

} // namespace

void uwb_output_publish(const UwbManager &manager, uint8_t sequence) {
  // Nothing acknowledges the frames while the drive board is absent, so they
  // wait in the queue. Skipping keeps at most one old publication waiting
  // instead of a backlog that would be sent late.
  if (can_port_tx_pending(OUTPUT_PORT) != 0U) {
    stats.skipped_busy++;
    return;
  }

  uint8_t payload[BLOCK_SIZE * BLOCK_COUNT];
  pack_blocks(manager, sequence, payload);

  const auto format = static_cast<CanOutputFrameFormat>(
      botbox::config::firmware_configuration().can_output.frame_format);
  can_frame_t frame = {};
  frame.extended = false;

  if (format == CanOutputFrameFormat::CLASSIC) {
    frame.fd = false;
    frame.brs = false;
    frame.length = BLOCK_SIZE;
    for (size_t block = 0U; block < BLOCK_COUNT; block++) {
      frame.id = OUTPUT_CAN_ID_BASE + block;
      for (size_t i = 0U; i < BLOCK_SIZE; i++) {
        frame.data[i] = payload[(block * BLOCK_SIZE) + i];
      }
      // A refused frame leaves the set incomplete; the drive board drops
      // incomplete sets, so the rest is not retried.
      if (!can_port_send(OUTPUT_PORT, &frame, 0U)) {
        stats.refused++;
        return;
      }
    }
  } else {
    frame.id = OUTPUT_CAN_ID_BASE;
    frame.fd = true;
    frame.brs = (format == CanOutputFrameFormat::FD_BRS);
    frame.length = BLOCK_SIZE * BLOCK_COUNT;
    for (size_t i = 0U; i < frame.length; i++) {
      frame.data[i] = payload[i];
    }
    if (!can_port_send(OUTPUT_PORT, &frame, 0U)) {
      stats.refused++;
      return;
    }
  }
  stats.published++;
}

const UwbOutputStats &uwb_output_stats() { return stats; }
