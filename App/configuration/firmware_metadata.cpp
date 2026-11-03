#include "firmware_metadata.hpp"

#include "firmware_configuration.hpp"
#include "version.h"

extern "C" const botbox::firmware::FirmwareMetadata botbox_firmware_metadata
    __attribute__((used, section(".firmware_metadata"), aligned(4))) = {
        {'B', 'U', 'F', 'W'},
        botbox::compatibility::FIRMWARE_METADATA_SCHEMA_VERSION,
        botbox::firmware::FIRMWARE_METADATA_SIZE,
        APP_VERSION_MAJOR,
        APP_VERSION_MINOR,
        APP_VERSION_PATCH,
        0U,
        botbox::compatibility::
            MINIMUM_SUPPORTED_CONFIGURATION_BINARY_FORMAT_VERSION,
        botbox::compatibility::CURRENT_CONFIGURATION_BINARY_FORMAT_VERSION,
        botbox::compatibility::
            MINIMUM_SUPPORTED_CONFIGURATION_SCHEMA_VERSION,
        botbox::compatibility::CURRENT_CONFIGURATION_SCHEMA_VERSION,
        {0U},
};
