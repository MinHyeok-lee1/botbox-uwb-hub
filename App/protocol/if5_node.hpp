#pragma once

// IF-5 node logic of the collection board (hub), phase 1: heartbeat state,
// CFG READ answers, ESTOP observation and peer heartbeat tracking. Pure logic
// driven by ticks, so it runs on the host (tests/host).
//
// Deliberately NOT implemented until the protocol document settles them
// (see .claude/docs/if5-phase1-plan.md, "Pending spec decisions"):
//  - ESTOP latch and RELEASE authority: STOP/RELEASE are only observed and
//    reported, nothing is latched.
//  - CFG 0x0003 output mask: answered UNSUPPORTED (bit order undefined).
//  - Peers required for DEGRADED: peers are tracked and reported only.

#include <cstdint>

#include "if5_codec.hpp"

namespace if5 {

// CFG parameters the hub knows.
constexpr uint16_t PARAM_OUTPUT_RATE_HZ = 0x0001U;
constexpr uint16_t PARAM_PAIRED_TAG_ID = 0x0002U;
constexpr uint16_t PARAM_OUTPUT_MASK = 0x0003U;
constexpr uint16_t PARAM_UPSTREAM_LATENCY_MS = 0x0005U;
constexpr uint16_t PARAM_NODE_ID = 0x0020U;
constexpr uint16_t PARAM_EXTERNAL_BITRATE_KBPS = 0x0021U;

// Bits of the UWB_RESULT_A status byte (IF-1) the heartbeat state looks at.
constexpr uint8_t RESULT_STATUS_VALID = 1U << 0;
constexpr uint8_t RESULT_STATUS_DEGRADED = 1U << 3;

// Peer heartbeat timeout (spec: 500 ms without a heartbeat = offline).
constexpr uint32_t PEER_TIMEOUT_MS = 500U;
// A STOP counts as active while STOPs keep arriving: senders repeat it every
// 100 ms while their cause lasts, and 300 ms without one is the spec's
// "no STOP received" window.
constexpr uint32_t ESTOP_ACTIVE_WINDOW_MS = 300U;

struct StateInputs {
  bool has_result;         // the UWB task produced at least one result
  bool result_stale;       // no result within the expected time
  uint8_t result_status;   // status byte of the last UWB_RESULT_A
  bool port_fault;         // an enabled CAN port failed to start or is bus-off
};

// FAULT > DEGRADED > RUN > READY > BOOT. FAULT is never reported: a fault
// panic halts the firmware.
NodeState heartbeat_state(const StateInputs &inputs);

// Static facts the CFG READ answers come from.
struct NodeInfo {
  uint16_t output_period_ms;
  uint8_t node_id;
  uint16_t external_bitrate_kbps;
};

// Phase 1 answers READ only; every other operation is UNSUPPORTED.
CfgResponse answer_cfg(const CfgRequest &request, const NodeInfo &info);

// Adds without wrapping past 255 (heartbeat err_count).
uint8_t saturate_u8(uint32_t value);

// Records STOP / RELEASE events from all nodes without acting on them.
class EstopMonitor {
public:
  void on_estop(const Estop &estop, uint32_t tick_ms);
  // A STOP from any node within ESTOP_ACTIVE_WINDOW_MS, and no later RELEASE.
  bool stop_seen(uint32_t now_ms) const;
  uint32_t stop_count() const { return _stopCount; }
  uint32_t release_count() const { return _releaseCount; }
  const Estop &last() const { return _last; }
  bool has_event() const { return _hasEvent; }

private:
  Estop _last = {};
  bool _hasEvent = false;
  bool _stopPending = false;
  uint32_t _lastStopTick = 0U;
  uint32_t _stopCount = 0U;
  uint32_t _releaseCount = 0U;
};

// Last heartbeat per (port, node). Ports are 0-based indexes.
class PeerTable {
public:
  static constexpr uint8_t PORT_COUNT = 2U;
  // Spec node IDs are 0x01-0x1F (0x10-0x1F third party); higher IDs are ignored.
  static constexpr uint8_t NODE_COUNT = 0x20U;

  void on_heartbeat(uint8_t port, uint8_t node, NodeState state, uint32_t tick_ms);
  bool is_online(uint8_t port, uint8_t node, uint32_t now_ms) const;
  bool ever_seen(uint8_t port, uint8_t node) const;
  NodeState state(uint8_t port, uint8_t node) const;

private:
  struct Entry {
    bool seen;
    NodeState state;
    uint32_t tick_ms;
  };
  Entry _entries[PORT_COUNT][NODE_COUNT] = {};
};

} // namespace if5
