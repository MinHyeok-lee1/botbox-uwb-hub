/**
 * @file uwb_task.h
 * @brief Task that turns the UWB sensor frames on CAN1 into the follow result.
 */
#ifndef UWB_TASK_H
#define UWB_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/** Create the UWB task. Call once before the scheduler starts, after
    can_port_init() and after the configuration has been loaded. */
void uwb_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* UWB_TASK_H */
