#pragma once

// PDoA single-anchor follow output: 0x244 (BBX v0.2) on CAN2, driven from
// uwb_task. Active when the flash configuration pdoa_follow.output_mode is
// BOTH or PDOA_ONLY.
//
// IF-0 input (0x224 observation, 0x234 calibration) is counted but not decoded
// yet: the byte layout and the E2 Core are still to be delivered by the UWB
// team. Until then the result stays "tag lost, no observation" and LAT_VALID
// is never set.

#include <cstdint>

#include "can_port.h"

struct PdoaOutputStats {
  uint32_t observation_frames; // 0x224 received on CAN1
  uint32_t calibration_frames; // 0x234 received on CAN1
  uint32_t sent;               // frames handed to the CAN2 latest slot
  uint32_t refused;            // the driver refused a frame
};

// Call once from the UWB task before its loop.
void pdoa_output_init(uint32_t now_ms);

// True when the configuration enables 0x244.
bool pdoa_output_enabled();

// True when 0x240/0x241 must not be sent (PDOA_ONLY).
bool pdoa_output_suppresses_legacy();

// Offer a CAN1 frame. Returns true when it is an IF-0 PDoA frame (consumed).
bool pdoa_output_on_frame(const can_frame_t &frame);

// Call on every UWB task poll (10 ms). Sends 0x244 on its 40 ms tick.
void pdoa_output_poll(uint32_t now_ms);

const PdoaOutputStats &pdoa_output_stats();

// Latest 0x244 result for the IF-5 heartbeat in PDOA_ONLY mode. `follow_ready`
// is true when the result carries a valid distance and lateral. Safe to call
// from any task.
struct PdoaOutputSnapshot {
  bool has_result;
  bool follow_ready;
  uint32_t tick; // RTOS tick of the last poll that evaluated the result
};
PdoaOutputSnapshot pdoa_output_snapshot();
