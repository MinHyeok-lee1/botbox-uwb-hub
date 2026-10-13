/**
 * @file app_main.h
 * @brief Application entry point.
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/** Create the application tasks. Call once before the scheduler starts. */
void app_main_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
