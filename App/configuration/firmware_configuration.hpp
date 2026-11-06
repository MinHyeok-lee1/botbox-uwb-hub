#pragma once

#include <cstddef>
#include <cstdint>

#include "schema_compatibility.hpp"

namespace botbox {
namespace config {

// Last 4 KiB of the STM32G474RE flash. The linker script reserves it. This is
// one page in single-bank mode and two pages in dual-bank mode, so the same
// address works with either DBANK option-byte setting.
static constexpr uint32_t CONFIGURATION_FLASH_ADDRESS = 0x0807F000U;
static constexpr uint32_t CONFIGURATION_FLASH_SIZE = 4U * 1024U;

// ASCII "BXUH" as a little-endian uint32_t.
static constexpr uint32_t CONFIGURATION_MAGIC = 0x48555842U;
static constexpr uint16_t CONFIGURATION_BINARY_FORMAT_VERSION =
    botbox::compatibility::CURRENT_CONFIGURATION_BINARY_FORMAT_VERSION;
static constexpr uint16_t CONFIGURATION_SCHEMA_VERSION =
    botbox::compatibility::CURRENT_CONFIGURATION_SCHEMA_VERSION;

// The current loader decodes one packed payload layout. Keep the advertised
// range exact until version-specific legacy decoders/migrations are added.
static_assert(
    botbox::compatibility::
            MINIMUM_SUPPORTED_CONFIGURATION_BINARY_FORMAT_VERSION ==
        CONFIGURATION_BINARY_FORMAT_VERSION,
    "Add legacy binary-format decoders before widening compatibility");
static_assert(
    botbox::compatibility::MINIMUM_SUPPORTED_CONFIGURATION_SCHEMA_VERSION ==
        CONFIGURATION_SCHEMA_VERSION,
    "Add legacy configuration-schema decoders before widening compatibility");

// These are storage capacities. The active sizes come from configuration.
static constexpr uint32_t UWB_MODE_DECISION_BUFFER_CAPACITY = 16U;
// Longest wait, in task cycles, before a direct FORWARD <-> BACKWARD change is
// accepted.
static constexpr uint32_t UWB_DIRECTION_FLIP_CONFIRM_MAX = 16U;

// Period of the task that calls UwbManager::update(). The mode decision buffer
// takes one sample per cycle, so the validation of
// mode_observation_timeout_ms depends on it. Same value as botbox+.
static constexpr uint32_t UWB_TASK_PERIOD_MS = 190U;

// Fields follow the packed BXUH payload order. Keep the two adjacent uint16_t
// spacing fields together so the following uint32_t remains 4-byte aligned.
struct UwbFollowConfiguration {
  uint32_t receive_timeout_ms;
  uint32_t distance_filter_enabled;
  uint32_t mode_decision_buffer_size;
  uint32_t mode_decision_majority_count;
  uint32_t mode_observation_timeout_ms;

  uint16_t front_sensor_spacing_mm;
  uint16_t front_rear_sensor_spacing_mm;

  uint32_t center_in_threshold_lr_mm;
  uint32_t center_out_threshold_lr_mm;
  uint32_t center_back_in_threshold_lr_mm;
  uint32_t center_back_out_threshold_lr_mm;
  uint32_t stop_threshold_fb_mm;
  uint32_t mid_threshold_fb_mm;
  float turn_ratio_smoothing_alpha;
  float turn_angle_deadzone_deg;
  float turn_angle_full_scale_deg;
  float turn_angle_smoothing_alpha;
  float turn_angle_max_step_deg;
  uint32_t direction_change_turn_hold_ms;
  uint32_t max_sensor_sample_skew_ms;
  uint32_t direction_flip_confirm_count;
  uint32_t min_turn_ratio_forward_percent;
  uint32_t min_turn_ratio_backward_percent;
  uint32_t close_back_turn_limit_distance_mm;
  uint32_t close_back_min_turn_ratio_percent;
  float initial_turn_ratio_percent;
  uint32_t back_uwb_enabled;
  float vector_smoothing_alpha;
};

struct FirmwareConfigurationPayload {
  UwbFollowConfiguration uwb_follow;
};

struct FirmwareConfigurationHeader {
  uint32_t magic;
  uint16_t binary_format_version;
  uint16_t configuration_schema_version;
  uint32_t payload_length;
  uint32_t payload_checksum;
};

struct FirmwareConfigurationImage {
  FirmwareConfigurationHeader header;
  FirmwareConfigurationPayload payload;
};

static_assert(sizeof(FirmwareConfigurationHeader) == 16U,
              "Configuration header ABI changed");
static_assert(offsetof(UwbFollowConfiguration, front_sensor_spacing_mm) == 20U,
              "Front sensor spacing offset changed");
static_assert(offsetof(UwbFollowConfiguration, front_rear_sensor_spacing_mm) == 22U,
              "Front/rear sensor spacing offset changed");
static_assert(offsetof(UwbFollowConfiguration, center_in_threshold_lr_mm) == 24U,
              "UWB uint32 alignment no longer matches packed BXUH payload");
static_assert(sizeof(UwbFollowConfiguration) == 108U,
              "UWB configuration ABI changed");
static_assert(sizeof(FirmwareConfigurationPayload) == 108U,
              "Configuration payload ABI changed");
static_assert(sizeof(FirmwareConfigurationImage) == 124U,
              "Configuration image ABI changed");
static_assert(sizeof(FirmwareConfigurationImage) <= CONFIGURATION_FLASH_SIZE,
              "Configuration does not fit in the reserved flash area");

enum class ConfigurationLoadStatus : uint8_t {
  NOT_INITIALIZED,
  LOADED_FROM_FLASH,
  EMPTY_FLASH,
  INVALID_HEADER,
  CRC_MISMATCH,
  INVALID_VALUES,
};

ConfigurationLoadStatus initialize_firmware_configuration();
ConfigurationLoadStatus firmware_configuration_status();
const char *firmware_configuration_status_string();
const FirmwareConfigurationPayload &firmware_configuration();
const FirmwareConfigurationPayload &default_firmware_configuration();

} // namespace config
} // namespace botbox
