#include "firmware_configuration.hpp"

#include <cstdint>
#include <cstring>

extern "C" const uint8_t __configuration_start__[];

namespace botbox {
namespace config {
namespace {

constexpr uint32_t MAX_RECEIVE_TIMEOUT_MS = 300000U;
constexpr uint32_t MAX_MODE_TIMEOUT_MS = 60000U;
constexpr uint32_t MAX_FRONT_SENSOR_SPACING_MM = 3000U;
constexpr uint32_t MAX_FRONT_REAR_SENSOR_SPACING_MM = 10000U;
constexpr uint32_t MAX_TURN_THRESHOLD_MM = 1000U;
constexpr uint32_t MAX_DIRECTION_THRESHOLD_MM = 5000U;
constexpr uint32_t MAX_CLOSE_BACK_DISTANCE_MM = 10000U;

uint32_t minimum_mode_observation_timeout_ms(
    uint32_t decision_buffer_size) {
  if (decision_buffer_size <= 1U) {
    return 0U;
  }
  return (decision_buffer_size - 1U) * UWB_TASK_PERIOD_MS;
}

const FirmwareConfigurationPayload DEFAULT_CONFIGURATION = {
{
    10000U, // receive_timeout_ms
    0U,     // distance_filter_enabled
    7U,     // mode_decision_buffer_size
    4U,     // mode_decision_majority_count
    1500U,  // mode_observation_timeout_ms

    400U,  // front_sensor_spacing_mm
    1200U, // front_rear_sensor_spacing_mm

    100U, // center_in_threshold_lr_mm
    150U, // center_out_threshold_lr_mm
    100U, // center_back_in_threshold_lr_mm
    150U, // center_back_out_threshold_lr_mm
    450U, // stop_threshold_fb_mm
    280U, // mid_threshold_fb_mm
    0.35f, // turn_ratio_smoothing_alpha
    5.0f,  // turn_angle_deadzone_deg
    60.0f, // turn_angle_full_scale_deg
    0.40f, // turn_angle_smoothing_alpha
    15.0f, // turn_angle_max_step_deg
    250U,  // direction_change_turn_hold_ms
    300U,  // max_sensor_sample_skew_ms
    3U,    // direction_flip_confirm_count
    15U,   // min_turn_ratio_forward_percent
    45U,   // min_turn_ratio_backward_percent
    2000U, // close_back_turn_limit_distance_mm
    60U,    // close_back_min_turn_ratio_percent
    100.0f, // initial_turn_ratio_percent
    1U,     // back_uwb_enabled
    0.4f    // vector_smoothing_alpha
}};

const FirmwareConfigurationPayload *active_configuration =
    &DEFAULT_CONFIGURATION;
FirmwareConfigurationImage loaded_configuration;
ConfigurationLoadStatus load_status =
    ConfigurationLoadStatus::NOT_INITIALIZED;

uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

bool valid_distance_ranges(const UwbFollowConfiguration &configuration) {
  const uint32_t distances[] = {
      configuration.center_in_threshold_lr_mm,
      configuration.center_out_threshold_lr_mm,
      configuration.center_back_in_threshold_lr_mm,
      configuration.center_back_out_threshold_lr_mm,
      configuration.stop_threshold_fb_mm,
      configuration.mid_threshold_fb_mm,
      configuration.close_back_turn_limit_distance_mm,
  };

  for (size_t i = 0; i < sizeof(distances) / sizeof(distances[0]); ++i) {
    if (distances[i] > 32767U) {
      return false;
    }
  }

  return true;
}

bool valid_tuning_ranges(const UwbFollowConfiguration &configuration) {
  const uint32_t turn_thresholds[] = {
      configuration.center_in_threshold_lr_mm,
      configuration.center_out_threshold_lr_mm,
      configuration.center_back_in_threshold_lr_mm,
      configuration.center_back_out_threshold_lr_mm,
  };
  for (size_t i = 0;
       i < sizeof(turn_thresholds) / sizeof(turn_thresholds[0]); ++i) {
    if (turn_thresholds[i] > MAX_TURN_THRESHOLD_MM) {
      return false;
    }
  }

  if (configuration.stop_threshold_fb_mm > MAX_DIRECTION_THRESHOLD_MM ||
      configuration.mid_threshold_fb_mm > MAX_DIRECTION_THRESHOLD_MM) {
    return false;
  }

  if (configuration.close_back_turn_limit_distance_mm >
      MAX_CLOSE_BACK_DISTANCE_MM) {
    return false;
  }

  return true;
}

bool valid_values(const FirmwareConfigurationPayload &payload) {
  const UwbFollowConfiguration &configuration = payload.uwb_follow;

  return configuration.receive_timeout_ms > 0U &&
         configuration.receive_timeout_ms <= MAX_RECEIVE_TIMEOUT_MS &&
         configuration.distance_filter_enabled <= 1U &&
         configuration.mode_decision_buffer_size > 0U &&
         configuration.mode_decision_buffer_size <=
             UWB_MODE_DECISION_BUFFER_CAPACITY &&
         configuration.mode_decision_majority_count > 0U &&
         configuration.mode_decision_majority_count <=
             configuration.mode_decision_buffer_size &&
         configuration.mode_observation_timeout_ms > 0U &&
         configuration.mode_observation_timeout_ms <=
             MAX_MODE_TIMEOUT_MS &&
         configuration.mode_observation_timeout_ms >=
             minimum_mode_observation_timeout_ms(
                 configuration.mode_decision_buffer_size) &&
         configuration.front_sensor_spacing_mm > 0U &&
         configuration.front_sensor_spacing_mm <=
             MAX_FRONT_SENSOR_SPACING_MM &&
         configuration.front_rear_sensor_spacing_mm > 0U &&
         configuration.front_rear_sensor_spacing_mm <=
             MAX_FRONT_REAR_SENSOR_SPACING_MM &&
         configuration.center_in_threshold_lr_mm <=
             configuration.center_out_threshold_lr_mm &&
         configuration.center_back_in_threshold_lr_mm <=
             configuration.center_back_out_threshold_lr_mm &&
         configuration.mid_threshold_fb_mm <=
             configuration.stop_threshold_fb_mm &&
         valid_distance_ranges(configuration) &&
         valid_tuning_ranges(configuration) &&
         configuration.turn_ratio_smoothing_alpha > 0.0f &&
         configuration.turn_ratio_smoothing_alpha <= 1.0f &&
         configuration.turn_angle_deadzone_deg >= 0.0f &&
         configuration.turn_angle_deadzone_deg <
             configuration.turn_angle_full_scale_deg &&
         configuration.turn_angle_full_scale_deg <= 90.0f &&
         configuration.turn_angle_smoothing_alpha > 0.0f &&
         configuration.turn_angle_smoothing_alpha <= 1.0f &&
         configuration.turn_angle_max_step_deg > 0.0f &&
         configuration.turn_angle_max_step_deg <= 90.0f &&
         configuration.direction_change_turn_hold_ms <= 5000U &&
         configuration.max_sensor_sample_skew_ms > 0U &&
         configuration.max_sensor_sample_skew_ms <=
             configuration.receive_timeout_ms &&
         configuration.direction_flip_confirm_count > 0U &&
         configuration.direction_flip_confirm_count <=
             UWB_DIRECTION_FLIP_CONFIRM_MAX &&
         configuration.min_turn_ratio_forward_percent <= 100U &&
         configuration.min_turn_ratio_backward_percent <= 100U &&
         configuration.close_back_min_turn_ratio_percent <= 100U &&
         configuration.initial_turn_ratio_percent >= 0.0f &&
         configuration.initial_turn_ratio_percent <= 100.0f &&
         configuration.back_uwb_enabled <= 1U &&
         configuration.vector_smoothing_alpha > 0.0f &&
         configuration.vector_smoothing_alpha <= 1.0f;
}

} // namespace

ConfigurationLoadStatus initialize_firmware_configuration() {
  std::memcpy(&loaded_configuration, __configuration_start__,
              sizeof(loaded_configuration));
  const FirmwareConfigurationImage *image = &loaded_configuration;
  active_configuration = &DEFAULT_CONFIGURATION;

  if (image->header.magic == 0xFFFFFFFFU) {
    load_status = ConfigurationLoadStatus::EMPTY_FLASH;
    return load_status;
  }

  if (image->header.magic != CONFIGURATION_MAGIC ||
      image->header.binary_format_version !=
          CONFIGURATION_BINARY_FORMAT_VERSION ||
      image->header.configuration_schema_version !=
          CONFIGURATION_SCHEMA_VERSION ||
      image->header.payload_length != sizeof(FirmwareConfigurationPayload)) {
    load_status = ConfigurationLoadStatus::INVALID_HEADER;
    return load_status;
  }

  const uint32_t calculated_crc = crc32(
      reinterpret_cast<const uint8_t *>(&image->payload),
      sizeof(image->payload));
  if (calculated_crc != image->header.payload_checksum) {
    load_status = ConfigurationLoadStatus::CRC_MISMATCH;
    return load_status;
  }

  if (!valid_values(image->payload)) {
    load_status = ConfigurationLoadStatus::INVALID_VALUES;
    return load_status;
  }

  active_configuration = &loaded_configuration.payload;
  load_status = ConfigurationLoadStatus::LOADED_FROM_FLASH;
  return load_status;
}

ConfigurationLoadStatus firmware_configuration_status() {
  if (load_status == ConfigurationLoadStatus::NOT_INITIALIZED) {
    initialize_firmware_configuration();
  }
  return load_status;
}

const char *firmware_configuration_status_string() {
  switch (firmware_configuration_status()) {
  case ConfigurationLoadStatus::NOT_INITIALIZED:
    return "not initialized";
  case ConfigurationLoadStatus::LOADED_FROM_FLASH:
    return "loaded from flash";
  case ConfigurationLoadStatus::EMPTY_FLASH:
    return "empty flash; using defaults";
  case ConfigurationLoadStatus::INVALID_HEADER:
    return "invalid header; using defaults";
  case ConfigurationLoadStatus::CRC_MISMATCH:
    return "CRC mismatch; using defaults";
  case ConfigurationLoadStatus::INVALID_VALUES:
    return "invalid values; using defaults";
  }
  return "unknown; using defaults";
}

const FirmwareConfigurationPayload &firmware_configuration() {
  if (load_status == ConfigurationLoadStatus::NOT_INITIALIZED) {
    initialize_firmware_configuration();
  }
  return *active_configuration;
}

const FirmwareConfigurationPayload &default_firmware_configuration() {
  return DEFAULT_CONFIGURATION;
}

} // namespace config
} // namespace botbox
