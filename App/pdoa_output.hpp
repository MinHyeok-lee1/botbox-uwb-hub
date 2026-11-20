#pragma once

// PDoA single-anchor follow: IF-0 R1 frames from the anchor on CAN1 (OBS_RAW
// 0x224, CAL_STATE 0x234) -> E2 Core (V3) -> 0x244 (BBX v0.2) on CAN2.
// Driven from uwb_task; active when the flash configuration
// pdoa_follow.output_mode is BOTH or PDOA_ONLY.

#include <cstdint>

#include "can_port.h"
#include "pdoa_pipeline.hpp"

struct PdoaOutputStats {
  uint32_t obs_frames;    // 0x224 received on CAN1
  uint32_t cal_frames;    // 0x234 received on CAN1
  uint32_t bad_frames;    // wrong shape or refused by the decoder
  uint32_t sent;          // frames handed to the CAN2 latest slot
  uint32_t refused;       // the driver refused a frame
  uint32_t restarts;      // estimator resets (RESTARTED re-armed)
  bool core_ready;        // the Core accepted its configuration
  uint32_t core_cycles_max;  // longest observation + read, CPU cycles
  uint32_t core_cycles_last;
};

// Call once from the UWB task before its loop.
void pdoa_output_init(uint32_t now_ms);

// True when the configuration enables 0x244.
bool pdoa_output_enabled();

// True when 0x240/0x241 must not be sent (PDOA_ONLY).
bool pdoa_output_suppresses_legacy();

// Offer a CAN1 frame. Returns true when it is an IF-0 PDoA frame (consumed).
// Processes it at once, with the frame's RX tick as its time.
bool pdoa_output_on_frame(const can_frame_t &frame);

// Call on every UWB task poll (10 ms): evaluates the result (tag lost, CAL
// timeout) and sends 0x244 on its 40 ms tick.
void pdoa_output_poll(uint32_t now_ms);

const PdoaOutputStats &pdoa_output_stats();
const pdoa::PipelineStats &pdoa_output_pipeline_stats();

// Latest 0x244 result for the IF-5 heartbeat in PDOA_ONLY mode. `follow_ready`
// is true when the result carries a valid distance and lateral. Safe to call
// from any task.
struct PdoaOutputSnapshot {
  bool has_result;
  bool follow_ready;
  uint32_t tick; // RTOS tick of the last poll that evaluated the result
};
PdoaOutputSnapshot pdoa_output_snapshot();
