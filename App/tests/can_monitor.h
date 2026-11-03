/**
 * @file can_monitor.h
 * @brief Receive-only check of one CAN port against an existing device.
 */
#ifndef CAN_MONITOR_H
#define CAN_MONITOR_H

/** Create the monitor task. Call once before the scheduler starts. */
void can_monitor_init(void);

#endif /* CAN_MONITOR_H */
