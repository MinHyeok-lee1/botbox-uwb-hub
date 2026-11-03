#pragma once

// Publishes the follow result to the drive board on CAN2. The frame layout is
// specified in .claude/docs/uwb-can-output-spec.md.

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
