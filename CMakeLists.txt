cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# The UWB follow algorithm (App/manager/uwb) is C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME botbox-uwb-hub)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
# The version is written into the firmware metadata header and version.h.
project(${CMAKE_PROJECT_NAME} VERSION 0.1.0)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Configuration versions the firmware and the uploader agree on. Checked here
# and compiled into the firmware metadata.
set(SCHEMA_COMPATIBILITY_MANIFEST
    ${CMAKE_CURRENT_SOURCE_DIR}/compatibility/schema-compatibility.json)
set(CONFIGURATION_SCHEMA
    ${CMAKE_CURRENT_SOURCE_DIR}/configuration/configuration.schema.json)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${SCHEMA_COMPATIBILITY_MANIFEST}
    ${CONFIGURATION_SCHEMA}
)

file(READ ${SCHEMA_COMPATIBILITY_MANIFEST} SCHEMA_COMPATIBILITY_JSON)
file(READ ${CONFIGURATION_SCHEMA} CONFIGURATION_SCHEMA_JSON)

string(JSON COMPATIBILITY_MANIFEST_VERSION
    GET "${SCHEMA_COMPATIBILITY_JSON}" manifest_version)
if(NOT COMPATIBILITY_MANIFEST_VERSION EQUAL 1)
    message(FATAL_ERROR
        "Unsupported schema compatibility manifest version: "
        "${COMPATIBILITY_MANIFEST_VERSION}")
endif()

string(JSON FIRMWARE_METADATA_SCHEMA_VERSION
    GET "${SCHEMA_COMPATIBILITY_JSON}"
    firmware emitted_firmware_metadata_schema)
string(JSON FIRMWARE_MINIMUM_CONFIGURATION_BINARY_FORMAT_VERSION
    GET "${SCHEMA_COMPATIBILITY_JSON}"
    firmware configuration binary_format minimum_supported)
string(JSON FIRMWARE_CURRENT_CONFIGURATION_BINARY_FORMAT_VERSION
    GET "${SCHEMA_COMPATIBILITY_JSON}"
    firmware configuration binary_format current)
string(JSON FIRMWARE_MINIMUM_CONFIGURATION_SCHEMA_VERSION
    GET "${SCHEMA_COMPATIBILITY_JSON}"
    firmware configuration schema minimum_supported)
string(JSON FIRMWARE_CURRENT_CONFIGURATION_SCHEMA_VERSION
    GET "${SCHEMA_COMPATIBILITY_JSON}"
    firmware configuration schema current)
string(JSON CONFIGURATION_BINARY_FORMAT_VERSION
    GET "${CONFIGURATION_SCHEMA_JSON}" binary_format version)
string(JSON CONFIGURATION_SCHEMA_VERSION
    GET "${CONFIGURATION_SCHEMA_JSON}" configuration_schema_version)
string(JSON UPLOADER_CONFIGURATION_BINARY_FORMAT_VERSION
    GET "${SCHEMA_COMPATIBILITY_JSON}"
    uploader generated_configuration binary_format)
string(JSON UPLOADER_CONFIGURATION_SCHEMA_VERSION
    GET "${SCHEMA_COMPATIBILITY_JSON}"
    uploader generated_configuration schema)
string(JSON UPLOADER_METADATA_SCHEMA_COUNT
    LENGTH "${SCHEMA_COMPATIBILITY_JSON}"
    uploader supported_firmware_metadata_schemas)

foreach(COMPATIBILITY_VALUE
        FIRMWARE_METADATA_SCHEMA_VERSION
        FIRMWARE_MINIMUM_CONFIGURATION_BINARY_FORMAT_VERSION
        FIRMWARE_CURRENT_CONFIGURATION_BINARY_FORMAT_VERSION
        FIRMWARE_MINIMUM_CONFIGURATION_SCHEMA_VERSION
        FIRMWARE_CURRENT_CONFIGURATION_SCHEMA_VERSION)
    if(${COMPATIBILITY_VALUE} LESS 1 OR ${COMPATIBILITY_VALUE} GREATER 65535)
        message(FATAL_ERROR
            "${COMPATIBILITY_VALUE} must be a positive uint16")
    endif()
endforeach()

if(FIRMWARE_MINIMUM_CONFIGURATION_BINARY_FORMAT_VERSION GREATER
   FIRMWARE_CURRENT_CONFIGURATION_BINARY_FORMAT_VERSION)
    message(FATAL_ERROR
        "Firmware minimum Configuration binary format exceeds current")
endif()
if(FIRMWARE_MINIMUM_CONFIGURATION_SCHEMA_VERSION GREATER
   FIRMWARE_CURRENT_CONFIGURATION_SCHEMA_VERSION)
    message(FATAL_ERROR
        "Firmware minimum Configuration schema exceeds current")
endif()
if(NOT FIRMWARE_CURRENT_CONFIGURATION_BINARY_FORMAT_VERSION EQUAL
       CONFIGURATION_BINARY_FORMAT_VERSION)
    message(FATAL_ERROR
        "Firmware current Configuration binary format does not match "
        "configuration.schema.json")
endif()
if(NOT FIRMWARE_CURRENT_CONFIGURATION_SCHEMA_VERSION EQUAL
       CONFIGURATION_SCHEMA_VERSION)
    message(FATAL_ERROR
        "Firmware current Configuration schema does not match "
        "configuration.schema.json")
endif()
if(NOT UPLOADER_CONFIGURATION_BINARY_FORMAT_VERSION EQUAL
       CONFIGURATION_BINARY_FORMAT_VERSION OR
   NOT UPLOADER_CONFIGURATION_SCHEMA_VERSION EQUAL
       CONFIGURATION_SCHEMA_VERSION)
    message(FATAL_ERROR
        "Uploader generated Configuration versions do not match "
        "configuration.schema.json")
endif()
if(UPLOADER_METADATA_SCHEMA_COUNT LESS 1)
    message(FATAL_ERROR
        "Uploader supported Firmware metadata schema list is empty")
endif()

set(UPLOADER_SUPPORTS_EMITTED_METADATA_SCHEMA FALSE)
math(EXPR UPLOADER_METADATA_SCHEMA_LAST
    "${UPLOADER_METADATA_SCHEMA_COUNT} - 1")
foreach(METADATA_SCHEMA_INDEX RANGE 0 ${UPLOADER_METADATA_SCHEMA_LAST})
    string(JSON UPLOADER_METADATA_SCHEMA
        GET "${SCHEMA_COMPATIBILITY_JSON}"
        uploader supported_firmware_metadata_schemas ${METADATA_SCHEMA_INDEX})
    if(UPLOADER_METADATA_SCHEMA LESS 1 OR
       UPLOADER_METADATA_SCHEMA GREATER 65535)
        message(FATAL_ERROR
            "Uploader Firmware metadata schemas must be positive uint16 values")
    endif()
    if(UPLOADER_METADATA_SCHEMA EQUAL FIRMWARE_METADATA_SCHEMA_VERSION)
        set(UPLOADER_SUPPORTS_EMITTED_METADATA_SCHEMA TRUE)
    endif()
endforeach()
if(NOT UPLOADER_SUPPORTS_EMITTED_METADATA_SCHEMA)
    message(FATAL_ERROR
        "Uploader does not support the Firmware metadata schema emitted by "
        "the current Firmware")
endif()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/schema_compatibility.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/schema_compatibility.hpp
    @ONLY
)

# version.h is generated into the build tree so the source tree stays clean.
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/App/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/version.h
    @ONLY
)

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# The linker script is passed only as a -T flag (cmake/gcc-arm-none-eabi.cmake),
# so CMake does not see it as an input. Relink when it changes: it places the
# configuration area and the firmware metadata.
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY
    LINK_DEPENDS ${CMAKE_SOURCE_DIR}/STM32G474xx_FLASH.ld)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/app_main.c
    App/console.c
    App/fault.c
    App/drivers/can_port.c
    App/drivers/uart_port.c
    # Flash-backed configuration and firmware metadata
    App/configuration/firmware_configuration.cpp
    App/configuration/firmware_info.cpp
    App/configuration/firmware_metadata.cpp
    App/uwb_task.cpp
    App/uwb_output.cpp
    # UWB follow algorithm from botbox+, without the move/stop decision
    App/manager/uwb/uwb_data_instance.cpp
    App/manager/uwb/uwb_manager.cpp
    App/manager/uwb/uwb_move_maker.cpp
    App/manager/uwb/uwb_vector.cpp
    App/manager/uwb/uwb_filter.c
    App/manager/uwb/uwb_filter_anchor_stub.c
    # Bring-up tests, selected with BOARD_TEST_MODE in App/board_config.h
    App/tests/bringup_test.c
    App/tests/can_linktest.c
    App/tests/can_loadtest.c
    App/tests/can_monitor.c
    App/tests/can_selftest.c
    App/tests/led_test.c
    App/tests/rs485_test.c
    App/tests/test_line.c
    App/tests/uart_test.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_BINARY_DIR}/generated
    App
    App/configuration
    App/drivers
    App/manager/uwb
    App/tests
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Versioned BIN and HEX images for the uploader, then check that the metadata
# in the BIN matches the project version and the compatibility manifest.
set(FIRMWARE_ARTIFACT_BASENAME
    "${CMAKE_PROJECT_NAME}-v${PROJECT_VERSION}")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${FIRMWARE_ARTIFACT_BASENAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${FIRMWARE_ARTIFACT_BASENAME}.bin
    COMMAND ${CMAKE_COMMAND}
        -DFIRMWARE_BIN=$<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${FIRMWARE_ARTIFACT_BASENAME}.bin
        -DEXPECTED_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
        -DEXPECTED_VERSION_MINOR=${PROJECT_VERSION_MINOR}
        -DEXPECTED_VERSION_PATCH=${PROJECT_VERSION_PATCH}
        -DEXPECTED_METADATA_SCHEMA=${FIRMWARE_METADATA_SCHEMA_VERSION}
        -DEXPECTED_MINIMUM_CONFIGURATION_BINARY_FORMAT=${FIRMWARE_MINIMUM_CONFIGURATION_BINARY_FORMAT_VERSION}
        -DEXPECTED_CURRENT_CONFIGURATION_BINARY_FORMAT=${FIRMWARE_CURRENT_CONFIGURATION_BINARY_FORMAT_VERSION}
        -DEXPECTED_MINIMUM_CONFIGURATION_SCHEMA=${FIRMWARE_MINIMUM_CONFIGURATION_SCHEMA_VERSION}
        -DEXPECTED_CURRENT_CONFIGURATION_SCHEMA=${FIRMWARE_CURRENT_CONFIGURATION_SCHEMA_VERSION}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_firmware_metadata.cmake
    VERBATIM
)
