#pragma once

// Publishes the follow result on CAN2 as the IF-1 messages UWB_RESULT_A
// (0x240) and, optionally, UWB_RESULT_B (0x241) of the UWB follow module
// interface specification v0.11. The mapping from the follow state is
// described in .claude/docs/uwb-can-output-spec.md.

#include <cstdint>

#include "uwb_manager.hpp"

struct UwbOutputStats {
  uint32_t published;    // publications fully queued
  uint32_t skipped_busy; // skipped: the previous one had not left the queue
  uint32_t refused;      // the driver refused a frame (port off or queue full)
};

// Queue this cycle's publication. Call once per UwbManager::update(); the
// sequence must advance by one on every call, sent or not.
void uwb_output_publish(const UwbManager &manager, uint8_t sequence);

const UwbOutputStats &uwb_output_stats();
