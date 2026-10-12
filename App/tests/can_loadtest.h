/**
 * @file can_loadtest.h
 * @brief Load test for CAN1 on a real bus between two boards.
 */
#ifndef CAN_LOADTEST_H
#define CAN_LOADTEST_H

/** Create the load-test tasks. Call once before the scheduler starts. */
void can_loadtest_init(void);

#endif /* CAN_LOADTEST_H */
