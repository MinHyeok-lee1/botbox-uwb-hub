/**
 * @file firmware_info.h
 * @brief C entry points for the firmware version and the flash configuration.
 */
#ifndef FIRMWARE_INFO_H
#define FIRMWARE_INFO_H

#ifdef __cplusplus
extern "C" {
#endif

/** Read and check the configuration image in flash. Call once at start-up,
    before anything reads the configuration. Defaults are used if it is not valid. */
void firmware_info_load_configuration(void);

/** Print the firmware version and where the configuration came from. Task context only. */
void firmware_info_report(void);

#ifdef __cplusplus
}
#endif

#endif /* FIRMWARE_INFO_H */
