/**
 * @file sys_node.h
 * @brief IF-5 common messages of the hub (node 0x01), phase 1.
 *
 * Sends SYS_HEARTBEAT (0x701) at 10 Hz on every started CAN port, answers
 * CFG_REQ (0x601) READ with CFG_RSP (0x581) on the bus the request came from,
 * observes SYS_ESTOP (0x011-0x02F) and tracks peer heartbeats. ESTOP latching,
 * RELEASE authority, the CFG 0x0003 output mask and peer-driven DEGRADED wait
 * for the protocol document (see .claude/docs/if5-phase1-plan.md).
 *
 * CAN2 is read by this task directly. CAN1 is read by the UWB task, which hands
 * IF-5 frames over with sys_node_forward(). Runs only with BOARD_TEST_NONE.
 */
#ifndef SYS_NODE_H
#define SYS_NODE_H

#include <stdbool.h>

#include "can_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Create the inbox and the task. Call once before the scheduler starts. */
bool sys_node_init(void);

/** True for identifiers the hub's IF-5 handling listens to. */
bool sys_node_wants(uint32_t id);

/** Hand over a frame received on @p port. Task context, does not block. */
void sys_node_forward(can_port_id_t port, const can_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* SYS_NODE_H */
