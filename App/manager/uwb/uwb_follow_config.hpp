#pragma once

#include <cstdint>

#include "firmware_configuration.hpp"

// Array storage remains a compile-time concern. The number of active entries
// is read from flash and validated against these capacities at boot.
static constexpr uint8_t UWB_MODE_DECISION_BUFFER_CAPACITY =
    static_cast<uint8_t>(
        botbox::config::UWB_MODE_DECISION_BUFFER_CAPACITY);
static constexpr uint8_t UWB_MOVE_BUFFER_CAPACITY =
    static_cast<uint8_t>(botbox::config::UWB_MOVE_BUFFER_CAPACITY);
static constexpr uint8_t UWB_FOLLOW_MODE_COUNT = 7U;

inline const botbox::config::UwbFollowConfiguration &
uwb_follow_configuration() {
  return botbox::config::firmware_configuration().uwb_follow;
}
