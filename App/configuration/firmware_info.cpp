#include "firmware_info.h"

#include "console.h"
#include "firmware_configuration.hpp"
#include "firmware_metadata.hpp"

void firmware_info_load_configuration(void) {
  botbox::config::initialize_firmware_configuration();
}

void firmware_info_report(void) {
  console_printf(
      "firmware: v%u.%u.%u\r\n",
      static_cast<unsigned int>(botbox_firmware_metadata.version_major),
      static_cast<unsigned int>(botbox_firmware_metadata.version_minor),
      static_cast<unsigned int>(botbox_firmware_metadata.version_patch));
  console_printf("configuration: %s (schema %u)\r\n",
                 botbox::config::firmware_configuration_status_string(),
                 static_cast<unsigned int>(
                     botbox::config::CONFIGURATION_SCHEMA_VERSION));
}
