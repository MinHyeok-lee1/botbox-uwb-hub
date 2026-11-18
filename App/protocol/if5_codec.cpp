#include "if5_codec.hpp"

namespace if5 {
namespace {

void put_u16(uint8_t *bytes, uint16_t value) {
  bytes[0] = static_cast<uint8_t>(value & 0xFFU);
  bytes[1] = static_cast<uint8_t>(value >> 8);
}

void put_u32(uint8_t *bytes, uint32_t value) {
  for (uint32_t i = 0U; i < 4U; i++) {
    bytes[i] = static_cast<uint8_t>((value >> (8U * i)) & 0xFFU);
  }
}

uint16_t get_u16(const uint8_t *bytes) {
  return static_cast<uint16_t>(bytes[0] | (static_cast<uint16_t>(bytes[1]) << 8));
}

uint32_t get_u32(const uint8_t *bytes) {
  uint32_t value = 0U;
  for (uint32_t i = 0U; i < 4U; i++) {
    value |= static_cast<uint32_t>(bytes[i]) << (8U * i);
  }
  return value;
}

bool is_classic_standard(const RawFrame &frame, uint8_t length) {
  return !frame.extended && !frame.fd && frame.length == length;
}

} // namespace

bool is_estop_id(uint32_t id) { return id >= ESTOP_ID_FIRST && id <= ESTOP_ID_LAST; }

bool is_heartbeat_id(uint32_t id) {
  return id > HEARTBEAT_BASE_ID && id <= HEARTBEAT_BASE_ID + NODE_ID_MAX;
}

bool is_if5_rx_id(uint32_t id, uint8_t own_node) {
  return is_estop_id(id) || is_heartbeat_id(id) || id == cfg_request_id(own_node);
}

void pack_heartbeat(const Heartbeat &heartbeat, uint8_t data[HEARTBEAT_LENGTH]) {
  data[0] = static_cast<uint8_t>(heartbeat.state);
  data[1] = heartbeat.protocol_version;
  put_u16(&data[2], heartbeat.firmware_build);
  data[4] = heartbeat.level;
  data[5] = heartbeat.error_count;
  put_u16(&data[6], heartbeat.uptime_s);
}

void pack_cfg_response(const CfgResponse &response, uint8_t data[CFG_LENGTH]) {
  data[0] = static_cast<uint8_t>(response.result);
  put_u16(&data[1], response.param);
  data[3] = 0U;
  put_u32(&data[4], response.value);
}

bool parse_heartbeat(const RawFrame &frame, uint8_t &node, NodeState &state) {
  if (!is_heartbeat_id(frame.id) || !is_classic_standard(frame, HEARTBEAT_LENGTH)) {
    return false;
  }
  if (frame.data[0] > static_cast<uint8_t>(NodeState::FAULT)) {
    return false;
  }
  node = static_cast<uint8_t>(frame.id - HEARTBEAT_BASE_ID);
  state = static_cast<NodeState>(frame.data[0]);
  return true;
}

bool parse_estop(const RawFrame &frame, Estop &estop) {
  if (!is_estop_id(frame.id) || !is_classic_standard(frame, ESTOP_LENGTH)) {
    return false;
  }
  const uint8_t command = frame.data[0];
  if (command != static_cast<uint8_t>(EstopCommand::STOP) &&
      command != static_cast<uint8_t>(EstopCommand::RELEASE)) {
    return false;
  }
  const uint8_t node = static_cast<uint8_t>(frame.id - ESTOP_BASE_ID);
  // The source byte repeats the node for the UART path; a mismatch means the
  // frame is not trustworthy.
  if (frame.data[1] != node) {
    return false;
  }
  estop.node = node;
  estop.command = static_cast<EstopCommand>(command);
  estop.source = frame.data[1];
  estop.reason = frame.data[2];
  estop.sequence = frame.data[3];
  return true;
}

bool parse_cfg_request(const RawFrame &frame, uint8_t own_node, CfgRequest &request) {
  if (frame.id != cfg_request_id(own_node) || !is_classic_standard(frame, CFG_LENGTH)) {
    return false;
  }
  request.op = frame.data[0];
  request.param = get_u16(&frame.data[1]);
  request.value = get_u32(&frame.data[4]);
  return true;
}

} // namespace if5
