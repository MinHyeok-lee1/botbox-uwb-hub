if(NOT DEFINED FIRMWARE_BIN)
    message(FATAL_ERROR "FIRMWARE_BIN is required")
endif()

if(NOT EXISTS "${FIRMWARE_BIN}")
    message(FATAL_ERROR "Firmware BIN does not exist: ${FIRMWARE_BIN}")
endif()

foreach(version_part MAJOR MINOR PATCH)
    if(NOT DEFINED EXPECTED_VERSION_${version_part})
        message(FATAL_ERROR "EXPECTED_VERSION_${version_part} is required")
    endif()
endforeach()

foreach(compatibility_value
        EXPECTED_METADATA_SCHEMA
        EXPECTED_MINIMUM_CONFIGURATION_BINARY_FORMAT
        EXPECTED_CURRENT_CONFIGURATION_BINARY_FORMAT
        EXPECTED_MINIMUM_CONFIGURATION_SCHEMA
        EXPECTED_CURRENT_CONFIGURATION_SCHEMA)
    if(NOT DEFINED ${compatibility_value})
        message(FATAL_ERROR "${compatibility_value} is required")
    endif()
endforeach()

set(FIRMWARE_METADATA_OFFSET 512)
set(FIRMWARE_METADATA_SIZE 32)
set(FIRMWARE_METADATA_END 544)

file(SIZE "${FIRMWARE_BIN}" firmware_size)
if(firmware_size LESS FIRMWARE_METADATA_END)
    message(FATAL_ERROR
        "Firmware BIN is too small to contain metadata at offset 0x200")
endif()

file(
    READ "${FIRMWARE_BIN}" firmware_metadata_hex
    OFFSET ${FIRMWARE_METADATA_OFFSET}
    LIMIT ${FIRMWARE_METADATA_SIZE}
    HEX
)
string(TOUPPER "${firmware_metadata_hex}" firmware_metadata_hex)

function(read_uint16_le source_hex byte_offset output_variable)
    math(EXPR character_offset "${byte_offset} * 2")
    math(EXPR high_character_offset "${character_offset} + 2")
    string(SUBSTRING "${source_hex}" ${character_offset} 2 low_byte_hex)
    string(SUBSTRING "${source_hex}" ${high_character_offset} 2 high_byte_hex)
    math(EXPR value "0x${low_byte_hex} + (0x${high_byte_hex} << 8)")
    set(${output_variable} ${value} PARENT_SCOPE)
endfunction()

string(SUBSTRING "${firmware_metadata_hex}" 0 8 metadata_magic)
if(NOT metadata_magic STREQUAL "42554657")
    message(FATAL_ERROR
        "Firmware metadata magic mismatch at BIN offset 0x200")
endif()

read_uint16_le("${firmware_metadata_hex}" 4 metadata_schema_version)
read_uint16_le("${firmware_metadata_hex}" 6 metadata_header_size)
read_uint16_le("${firmware_metadata_hex}" 8 firmware_version_major)
read_uint16_le("${firmware_metadata_hex}" 10 firmware_version_minor)
read_uint16_le("${firmware_metadata_hex}" 12 firmware_version_patch)
read_uint16_le("${firmware_metadata_hex}" 14 metadata_flags)
read_uint16_le("${firmware_metadata_hex}" 16 minimum_configuration_binary_format_version)
read_uint16_le("${firmware_metadata_hex}" 18 current_configuration_binary_format_version)
read_uint16_le("${firmware_metadata_hex}" 20 minimum_configuration_schema_version)
read_uint16_le("${firmware_metadata_hex}" 22 current_configuration_schema_version)

if(NOT metadata_schema_version EQUAL EXPECTED_METADATA_SCHEMA)
    message(FATAL_ERROR
        "Unsupported firmware metadata schema version: ${metadata_schema_version}")
endif()
if(NOT metadata_header_size EQUAL FIRMWARE_METADATA_SIZE)
    message(FATAL_ERROR
        "Firmware metadata header size mismatch: ${metadata_header_size}")
endif()
if(NOT firmware_version_major EQUAL EXPECTED_VERSION_MAJOR OR
   NOT firmware_version_minor EQUAL EXPECTED_VERSION_MINOR OR
   NOT firmware_version_patch EQUAL EXPECTED_VERSION_PATCH)
    message(FATAL_ERROR
        "Firmware metadata version ${firmware_version_major}.${firmware_version_minor}.${firmware_version_patch} "
        "does not match project version "
        "${EXPECTED_VERSION_MAJOR}.${EXPECTED_VERSION_MINOR}.${EXPECTED_VERSION_PATCH}")
endif()
if(NOT metadata_flags EQUAL 0)
    message(FATAL_ERROR "Firmware metadata flags must be zero for schema version 1")
endif()

if(minimum_configuration_binary_format_version LESS 1 OR
   minimum_configuration_binary_format_version GREATER
   current_configuration_binary_format_version)
    message(FATAL_ERROR
        "Firmware metadata configuration binary format range is invalid")
endif()
if(minimum_configuration_schema_version LESS 1 OR
   minimum_configuration_schema_version GREATER
   current_configuration_schema_version)
    message(FATAL_ERROR
        "Firmware metadata configuration schema range is invalid")
endif()

if(NOT minimum_configuration_binary_format_version EQUAL
       EXPECTED_MINIMUM_CONFIGURATION_BINARY_FORMAT OR
   NOT current_configuration_binary_format_version EQUAL
       EXPECTED_CURRENT_CONFIGURATION_BINARY_FORMAT OR
   NOT minimum_configuration_schema_version EQUAL
       EXPECTED_MINIMUM_CONFIGURATION_SCHEMA OR
   NOT current_configuration_schema_version EQUAL
       EXPECTED_CURRENT_CONFIGURATION_SCHEMA)
    message(FATAL_ERROR
        "Firmware metadata compatibility values do not match the manifest")
endif()

string(SUBSTRING "${firmware_metadata_hex}" 48 16 metadata_reserved)
if(NOT metadata_reserved STREQUAL "0000000000000000")
    message(FATAL_ERROR
        "Firmware metadata reserved bytes must be zero for schema version 1")
endif()

message(STATUS
    "Verified firmware metadata: "
    "v${firmware_version_major}.${firmware_version_minor}.${firmware_version_patch} "
    "at BIN offset 0x200; "
    "BXUH binary ${minimum_configuration_binary_format_version}-"
    "${current_configuration_binary_format_version}, "
    "configuration schema ${minimum_configuration_schema_version}-"
    "${current_configuration_schema_version}")
