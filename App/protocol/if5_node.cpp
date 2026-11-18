#include "if5_node.hpp"

namespace if5 {
namespace {

constexpr uint32_t MS_PER_S = 1000U;
constexpr uint8_t U8_MAX = 0xFFU;

} // namespace

NodeState heartbeat_state(const StateInputs &inputs) {
  if (!inputs.has_result) {
    return inputs.port_fault ? NodeState::DEGRADED : NodeState::BOOT;
  }
  if (inputs.port_fault || inputs.result_stale) {
    return NodeState::DEGRADED;
  }
  // A sensor dropped out: the result is reduced (VALID) or missing for a
  // cycle while the follow mode changes (not VALID).
  if ((inputs.result_status & RESULT_STATUS_DEGRADED) != 0U) {
    return NodeState::DEGRADED;
  }
  if ((inputs.result_status & RESULT_STATUS_VALID) != 0U) {
    return NodeState::RUN;
  }
  // No tag or tag lost: the hub works, it has nothing to report.
  return NodeState::READY;
}

CfgResponse answer_cfg(const CfgRequest &request, const NodeInfo &info) {
  CfgResponse response = {CfgResult::UNSUPPORTED, request.param, 0U};
  if (request.op != static_cast<uint8_t>(CfgOp::READ)) {
    // WRITE / SAVE / REBOOT are spec stage 2.
    return response;
  }

  switch (request.param) {
  case PARAM_OUTPUT_RATE_HZ:
    // Nearest whole Hz of the fixed algorithm period (190 ms -> 5).
    response.result = CfgResult::OK;
    response.value = (MS_PER_S + info.output_period_ms / 2U) / info.output_period_ms;
    break;
  case PARAM_NODE_ID:
    response.result = CfgResult::OK;
    response.value = info.node_id;
    break;
  case PARAM_EXTERNAL_BITRATE_KBPS:
    response.result = CfgResult::OK;
    response.value = info.external_bitrate_kbps;
    break;
  case PARAM_PAIRED_TAG_ID:       // spec stage 2
  case PARAM_OUTPUT_MASK:         // TODO(spec): bit order not defined yet
  case PARAM_UPSTREAM_LATENCY_MS: // not measured yet (spec check 12)
  default:
    break;
  }
  return response;
}

uint8_t saturate_u8(uint32_t value) {
  return (value > U8_MAX) ? U8_MAX : static_cast<uint8_t>(value);
}

void EstopMonitor::on_estop(const Estop &estop, uint32_t tick_ms) {
  _last = estop;
  _hasEvent = true;
  if (estop.command == EstopCommand::STOP) {
    _stopCount++;
    _stopPending = true;
    _lastStopTick = tick_ms;
  } else {
    _releaseCount++;
    _stopPending = false;
  }
}

bool EstopMonitor::stop_seen(uint32_t now_ms) const {
  return _stopPending && (now_ms - _lastStopTick) <= ESTOP_ACTIVE_WINDOW_MS;
}

void PeerTable::on_heartbeat(uint8_t port, uint8_t node, NodeState state, uint32_t tick_ms) {
  if (port >= PORT_COUNT || node >= NODE_COUNT) {
    return;
  }
  _entries[port][node] = {true, state, tick_ms};
}

bool PeerTable::is_online(uint8_t port, uint8_t node, uint32_t now_ms) const {
  if (!ever_seen(port, node)) {
    return false;
  }
  return (now_ms - _entries[port][node].tick_ms) <= PEER_TIMEOUT_MS;
}

bool PeerTable::ever_seen(uint8_t port, uint8_t node) const {
  return port < PORT_COUNT && node < NODE_COUNT && _entries[port][node].seen;
}

NodeState PeerTable::state(uint8_t port, uint8_t node) const {
  return ever_seen(port, node) ? _entries[port][node].state : NodeState::BOOT;
}

} // namespace if5
