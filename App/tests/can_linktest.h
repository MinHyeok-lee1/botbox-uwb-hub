/**
 * @file can_linktest.h
 * @brief Bring-up check of one CAN port on a real bus between two boards.
 */
#ifndef CAN_LINKTEST_H
#define CAN_LINKTEST_H

/** Create the link-test task. Call once before the scheduler starts. */
void can_linktest_init(void);

#endif /* CAN_LINKTEST_H */
