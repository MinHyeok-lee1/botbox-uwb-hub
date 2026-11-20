// Host test for the flash configuration loader: current schema, schema 1
// migration and rejected images.
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "firmware_configuration.hpp"

using namespace botbox::config;

// Stands in for the linker-provided CONFIG region.
extern "C" {
alignas(4) uint8_t __configuration_start__[CONFIGURATION_FLASH_SIZE];
}

namespace {

int failures = 0;

#define CHECK(condition)                                                    \
  do {                                                                      \
    if (!(condition)) {                                                     \
      std::printf("FAIL line %d: %s\n", __LINE__, #condition);              \
      ++failures;                                                           \
    }                                                                       \
  } while (0)

uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

// Writes header + payload bytes into the fake flash (rest erased).
void write_image(uint16_t schema, const void *payload, uint32_t length) {
  std::memset(__configuration_start__, 0xFF, sizeof(__configuration_start__));
  FirmwareConfigurationHeader header = {
      CONFIGURATION_MAGIC, CONFIGURATION_BINARY_FORMAT_VERSION, schema, length,
      crc32(static_cast<const uint8_t *>(payload), length)};
  std::memcpy(__configuration_start__, &header, sizeof(header));
  std::memcpy(__configuration_start__ + sizeof(header), payload, length);
}

ConfigurationLoadStatus load() { return initialize_firmware_configuration(); }

void test_current_schema() {
  FirmwareConfigurationPayload payload = default_firmware_configuration();
  payload.pdoa_follow.output_mode = PDOA_OUTPUT_PDOA_ONLY;
  write_image(CONFIGURATION_SCHEMA_VERSION, &payload, sizeof(payload));
  CHECK(load() == ConfigurationLoadStatus::LOADED_FROM_FLASH);
  CHECK(firmware_configuration().pdoa_follow.output_mode ==
        PDOA_OUTPUT_PDOA_ONLY);
}

void test_schema_1_migration() {
  FirmwareConfigurationPayloadSchema1 old_payload = {
      default_firmware_configuration().uwb_follow};
  old_payload.uwb_follow.receive_timeout_ms = 4321U; // distinguishable
  write_image(LEGACY_SCHEMA_1, &old_payload, sizeof(old_payload));
  CHECK(load() == ConfigurationLoadStatus::LOADED_MIGRATED_FROM_SCHEMA_1);
  CHECK(firmware_configuration().uwb_follow.receive_timeout_ms == 4321U);
  CHECK(firmware_configuration().pdoa_follow.output_mode ==
        PDOA_OUTPUT_LEGACY_ONLY);
}

void test_schema_1_with_current_length_rejected() {
  FirmwareConfigurationPayload payload = default_firmware_configuration();
  write_image(LEGACY_SCHEMA_1, &payload, sizeof(payload));
  CHECK(load() == ConfigurationLoadStatus::INVALID_HEADER);
}

void test_current_schema_with_old_length_rejected() {
  FirmwareConfigurationPayloadSchema1 old_payload = {
      default_firmware_configuration().uwb_follow};
  write_image(CONFIGURATION_SCHEMA_VERSION, &old_payload, sizeof(old_payload));
  CHECK(load() == ConfigurationLoadStatus::INVALID_HEADER);
}

void test_unknown_schema_rejected() {
  FirmwareConfigurationPayload payload = default_firmware_configuration();
  write_image(3U, &payload, sizeof(payload));
  CHECK(load() == ConfigurationLoadStatus::INVALID_HEADER);
}

void test_bad_crc() {
  FirmwareConfigurationPayload payload = default_firmware_configuration();
  write_image(CONFIGURATION_SCHEMA_VERSION, &payload, sizeof(payload));
  __configuration_start__[sizeof(FirmwareConfigurationHeader)] ^= 0x01U;
  CHECK(load() == ConfigurationLoadStatus::CRC_MISMATCH);
  CHECK(firmware_configuration().pdoa_follow.output_mode ==
        PDOA_OUTPUT_LEGACY_ONLY);
}

void test_invalid_output_mode() {
  FirmwareConfigurationPayload payload = default_firmware_configuration();
  payload.pdoa_follow.output_mode = PDOA_OUTPUT_PDOA_ONLY + 1U;
  write_image(CONFIGURATION_SCHEMA_VERSION, &payload, sizeof(payload));
  CHECK(load() == ConfigurationLoadStatus::INVALID_VALUES);
  CHECK(firmware_configuration().pdoa_follow.output_mode ==
        PDOA_OUTPUT_LEGACY_ONLY);
}

void test_empty_flash() {
  std::memset(__configuration_start__, 0xFF, sizeof(__configuration_start__));
  CHECK(load() == ConfigurationLoadStatus::EMPTY_FLASH);
}

} // namespace

int main() {
  test_current_schema();
  test_schema_1_migration();
  test_schema_1_with_current_length_rejected();
  test_current_schema_with_old_length_rejected();
  test_unknown_schema_rejected();
  test_bad_crc();
  test_invalid_output_mode();
  test_empty_flash();
  if (failures == 0) {
    std::printf("configuration tests passed\n");
  }
  return failures == 0 ? 0 : 1;
}
