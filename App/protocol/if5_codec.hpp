#pragma once

// IF-5 common messages of the UWB follow module interface specification v0.11
// (docs/02_UWB_추종_모듈_통신_인터페이스_명세서_v0.11.pdf, pages 11-13):
// SYS_HEARTBEAT, SYS_ESTOP and CFG_REQ / CFG_RSP. Pure packing and parsing,
// no RTOS or HAL, so it builds and runs on the host (tests/host).
//
// All new messages are standard 11-bit IDs, classic CAN, little endian.

#include <cstdint>

namespace if5 {

constexpr uint8_t NODE_ID_COLLECTION_BOARD = 0x01U;
constexpr uint8_t NODE_ID_MAX = 0x7FU; // the ID bases leave 7 bits for the node

constexpr uint32_t ESTOP_BASE_ID = 0x010U;
constexpr uint32_t ESTOP_ID_FIRST = 0x011U;
constexpr uint32_t ESTOP_ID_LAST = 0x02FU;
constexpr uint32_t CFG_RSP_BASE_ID = 0x580U;
constexpr uint32_t CFG_REQ_BASE_ID = 0x600U;
constexpr uint32_t HEARTBEAT_BASE_ID = 0x700U;

constexpr uint8_t HEARTBEAT_LENGTH = 8U;
constexpr uint8_t ESTOP_LENGTH = 4U;
constexpr uint8_t CFG_LENGTH = 8U;

// Protocol version in SYS_HEARTBEAT: major 0, minor 11 (spec v0.11).
constexpr uint8_t PROTOCOL_VERSION = 0x0BU;
// Configuration code: 1 = L1 integration (the hub only publishes UWB results).
constexpr uint8_t LEVEL_L1 = 1U;

enum class NodeState : uint8_t {
  BOOT = 0U,
  READY = 1U,
  RUN = 2U,
  DEGRADED = 3U,
  FAULT = 4U,
};

enum class EstopCommand : uint8_t {
  RELEASE = 0U,
  STOP = 1U,
};

enum class CfgOp : uint8_t {
  READ = 1U,
  WRITE = 2U,
  SAVE = 3U,
  REBOOT = 4U,
};

enum class CfgResult : uint8_t {
  OK = 0U,
  UNSUPPORTED = 1U,
  OUT_OF_RANGE = 2U,
  SAVE_FAILED = 3U,
};

// A received frame, reduced to what the IF-5 parsers look at.
struct RawFrame {
  uint32_t id;
  bool extended;
  bool fd;
  uint8_t length;
  uint8_t data[8];
};

struct Heartbeat {
  NodeState state;
  uint8_t protocol_version;
  uint16_t firmware_build;
  uint8_t level;
  uint8_t error_count;
  uint16_t uptime_s;
};

struct Estop {
  uint8_t node; // from the CAN ID
  EstopCommand command;
  uint8_t source; // byte 1, equal to node in a valid frame
  uint8_t reason;
  uint8_t sequence;
};

struct CfgRequest {
  uint8_t op; // raw, so unknown operations can be answered
  uint16_t param;
  uint32_t value;
};

struct CfgResponse {
  CfgResult result;
  uint16_t param;
  uint32_t value;
};

constexpr uint32_t heartbeat_id(uint8_t node) { return HEARTBEAT_BASE_ID + node; }
constexpr uint32_t cfg_request_id(uint8_t node) { return CFG_REQ_BASE_ID + node; }
constexpr uint32_t cfg_response_id(uint8_t node) { return CFG_RSP_BASE_ID + node; }

bool is_estop_id(uint32_t id);
bool is_heartbeat_id(uint32_t id);
// Any IF-5 identifier the hub listens to (ESTOP range, own CFG_REQ, heartbeats).
bool is_if5_rx_id(uint32_t id, uint8_t own_node);

void pack_heartbeat(const Heartbeat &heartbeat, uint8_t data[HEARTBEAT_LENGTH]);
void pack_cfg_response(const CfgResponse &response, uint8_t data[CFG_LENGTH]);

// Parsers return false for a frame that does not have the exact IF-5 form
// (standard ID in range, classic, exact length, known command).
bool parse_heartbeat(const RawFrame &frame, uint8_t &node, NodeState &state);
bool parse_estop(const RawFrame &frame, Estop &estop);
bool parse_cfg_request(const RawFrame &frame, uint8_t own_node, CfgRequest &request);

} // namespace if5
