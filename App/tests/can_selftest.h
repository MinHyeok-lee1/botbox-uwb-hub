/**
 * @file can_selftest.h
 * @brief Bring-up check for the CAN ports in internal loopback mode.
 */
#ifndef CAN_SELFTEST_H
#define CAN_SELFTEST_H

/** Create the self-test task. Call once before the scheduler starts. */
void can_selftest_init(void);

#endif /* CAN_SELFTEST_H */
