#pragma once

#include <cstddef>
#include <cstdint>

#include "schema_compatibility.hpp"

namespace botbox {
namespace firmware {

static constexpr uint32_t FIRMWARE_METADATA_ADDRESS = 0x08000200U;
static constexpr uint16_t FIRMWARE_METADATA_SIZE = 32U;

#pragma pack(push, 1)
struct FirmwareMetadata {
  char magic[4];
  uint16_t schema_version;
  uint16_t header_size;
  uint16_t version_major;
  uint16_t version_minor;
  uint16_t version_patch;
  uint16_t flags;
  uint16_t minimum_configuration_binary_format_version;
  uint16_t current_configuration_binary_format_version;
  uint16_t minimum_configuration_schema_version;
  uint16_t current_configuration_schema_version;
  uint8_t reserved[8];
};
#pragma pack(pop)

static_assert(sizeof(FirmwareMetadata) == FIRMWARE_METADATA_SIZE,
              "Firmware metadata size must remain stable");
static_assert(offsetof(FirmwareMetadata, schema_version) == 4U,
              "Firmware metadata schema_version offset changed");
static_assert(offsetof(FirmwareMetadata, header_size) == 6U,
              "Firmware metadata header_size offset changed");
static_assert(offsetof(FirmwareMetadata, version_major) == 8U,
              "Firmware metadata version_major offset changed");
static_assert(offsetof(FirmwareMetadata, version_minor) == 10U,
              "Firmware metadata version_minor offset changed");
static_assert(offsetof(FirmwareMetadata, version_patch) == 12U,
              "Firmware metadata version_patch offset changed");
static_assert(offsetof(FirmwareMetadata, flags) == 14U,
              "Firmware metadata flags offset changed");
static_assert(
    offsetof(FirmwareMetadata,
             minimum_configuration_binary_format_version) == 16U,
    "Firmware metadata minimum configuration binary format offset changed");
static_assert(
    offsetof(FirmwareMetadata,
             current_configuration_binary_format_version) == 18U,
    "Firmware metadata current configuration binary format offset changed");
static_assert(offsetof(FirmwareMetadata,
                       minimum_configuration_schema_version) == 20U,
              "Firmware metadata minimum configuration schema offset changed");
static_assert(
    offsetof(FirmwareMetadata, current_configuration_schema_version) == 22U,
    "Firmware metadata current configuration schema offset changed");
static_assert(offsetof(FirmwareMetadata, reserved) == 24U,
              "Firmware metadata reserved offset changed");

}  // namespace firmware
}  // namespace botbox

extern "C" const botbox::firmware::FirmwareMetadata botbox_firmware_metadata;
