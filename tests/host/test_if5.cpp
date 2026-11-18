// Host tests of the IF-5 codec and node logic (App/protocol).
// Run: tests/host/run_if5_tests.sh
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "if5_codec.hpp"
#include "if5_node.hpp"

namespace {

int failures = 0;

#define CHECK(condition)                                                                 \
  do {                                                                                   \
    if (!(condition)) {                                                                  \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                   \
      failures++;                                                                        \
    }                                                                                    \
  } while (0)

if5::RawFrame frame(uint32_t id, uint8_t length, std::initializer_list<uint8_t> bytes) {
  if5::RawFrame raw = {};
  raw.id = id;
  raw.length = length;
  size_t i = 0U;
  for (uint8_t b : bytes) {
    raw.data[i++] = b;
  }
  return raw;
}

void test_pack_heartbeat_layout() {
  const if5::Heartbeat hb = {if5::NodeState::RUN, if5::PROTOCOL_VERSION, 0x1234U,
                             if5::LEVEL_L1, 7U, 0xABCDU};
  uint8_t data[8];
  if5::pack_heartbeat(hb, data);
  const uint8_t expected[8] = {2U, 0x0BU, 0x34U, 0x12U, 1U, 7U, 0xCDU, 0xABU};
  CHECK(std::memcmp(data, expected, 8) == 0);
  CHECK(if5::heartbeat_id(1U) == 0x701U);
}

void test_parse_estop_valid_stop_and_release() {
  if5::Estop e;
  CHECK(if5::parse_estop(frame(0x013U, 4U, {1U, 0x03U, 2U, 9U}), e));
  CHECK(e.node == 3U && e.command == if5::EstopCommand::STOP && e.reason == 2U &&
        e.sequence == 9U);
  CHECK(if5::parse_estop(frame(0x02FU, 4U, {0U, 0x1FU, 0U, 0U}), e));
  CHECK(e.command == if5::EstopCommand::RELEASE);
}

void test_parse_estop_rejects_malformed() {
  if5::Estop e;
  CHECK(!if5::parse_estop(frame(0x013U, 8U, {1U, 3U, 0U, 0U}), e)); // DLC
  CHECK(!if5::parse_estop(frame(0x013U, 4U, {2U, 3U, 0U, 0U}), e)); // command
  CHECK(!if5::parse_estop(frame(0x013U, 4U, {1U, 4U, 0U, 0U}), e)); // source mismatch
  CHECK(!if5::parse_estop(frame(0x010U, 4U, {1U, 0U, 0U, 0U}), e)); // below range
  CHECK(!if5::parse_estop(frame(0x030U, 4U, {1U, 0x20U, 0U, 0U}), e)); // above range
  auto ext = frame(0x013U, 4U, {1U, 3U, 0U, 0U});
  ext.extended = true;
  CHECK(!if5::parse_estop(ext, e));
  auto fd = frame(0x013U, 4U, {1U, 3U, 0U, 0U});
  fd.fd = true;
  CHECK(!if5::parse_estop(fd, e));
}

void test_parse_cfg_request() {
  if5::CfgRequest r;
  CHECK(if5::parse_cfg_request(frame(0x601U, 8U, {1U, 0x21U, 0x00U, 0U, 1U, 2U, 3U, 4U}), 1U,
                               r));
  CHECK(r.op == 1U && r.param == 0x0021U && r.value == 0x04030201U);
  CHECK(!if5::parse_cfg_request(frame(0x602U, 8U, {1U}), 1U, r)); // other node
  CHECK(!if5::parse_cfg_request(frame(0x601U, 7U, {1U}), 1U, r));  // DLC
}

void test_parse_heartbeat() {
  uint8_t node;
  if5::NodeState state;
  CHECK(if5::parse_heartbeat(frame(0x703U, 8U, {3U}), node, state));
  CHECK(node == 3U && state == if5::NodeState::DEGRADED);
  CHECK(!if5::parse_heartbeat(frame(0x703U, 8U, {5U}), node, state)); // unknown state
  CHECK(!if5::parse_heartbeat(frame(0x700U, 8U, {1U}), node, state)); // node 0
  CHECK(!if5::parse_heartbeat(frame(0x780U, 8U, {1U}), node, state)); // beyond 7 bits
}

void test_rx_id_filter() {
  CHECK(if5::is_if5_rx_id(0x011U, 1U));
  CHECK(if5::is_if5_rx_id(0x601U, 1U));
  CHECK(if5::is_if5_rx_id(0x702U, 1U));
  CHECK(!if5::is_if5_rx_id(0x602U, 1U));
  CHECK(!if5::is_if5_rx_id(0x201U, 1U)); // legacy sensor frame
  CHECK(!if5::is_if5_rx_id(0x240U, 1U)); // own UWB result
}

void test_pack_cfg_response() {
  uint8_t data[8];
  if5::pack_cfg_response({if5::CfgResult::OK, 0x0021U, 500U}, data);
  const uint8_t expected[8] = {0U, 0x21U, 0x00U, 0U, 0xF4U, 0x01U, 0U, 0U};
  CHECK(std::memcmp(data, expected, 8) == 0);
}

void test_answer_cfg() {
  const if5::NodeInfo info = {190U, 1U, 500U};
  auto read = [&](uint16_t param) {
    return if5::answer_cfg({static_cast<uint8_t>(if5::CfgOp::READ), param, 0U}, info);
  };
  CHECK(read(0x0001U).result == if5::CfgResult::OK && read(0x0001U).value == 5U);
  CHECK(read(0x0020U).value == 1U);
  CHECK(read(0x0021U).value == 500U);
  CHECK(read(0x0002U).result == if5::CfgResult::UNSUPPORTED);
  CHECK(read(0x0003U).result == if5::CfgResult::UNSUPPORTED);
  CHECK(read(0x0005U).result == if5::CfgResult::UNSUPPORTED);
  CHECK(read(0x1234U).result == if5::CfgResult::UNSUPPORTED && read(0x1234U).value == 0U);
  CHECK(read(0x1234U).param == 0x1234U);
  for (uint8_t op : {0U, 2U, 3U, 4U, 99U}) {
    const auto r = if5::answer_cfg({op, 0x0020U, 7U}, info);
    CHECK(r.result == if5::CfgResult::UNSUPPORTED && r.value == 0U && r.param == 0x0020U);
  }
}

void test_heartbeat_state_precedence() {
  using if5::NodeState;
  const uint8_t valid = if5::RESULT_STATUS_VALID;
  const uint8_t degraded = if5::RESULT_STATUS_DEGRADED;
  CHECK(if5::heartbeat_state({false, false, 0U, false}) == NodeState::BOOT);
  CHECK(if5::heartbeat_state({false, false, 0U, true}) == NodeState::DEGRADED);
  CHECK(if5::heartbeat_state({true, false, 0U, false}) == NodeState::READY);
  CHECK(if5::heartbeat_state({true, false, 1U << 2, false}) == NodeState::READY); // tag lost
  CHECK(if5::heartbeat_state({true, false, valid, false}) == NodeState::RUN);
  CHECK(if5::heartbeat_state({true, false, valid | degraded, false}) == NodeState::DEGRADED);
  CHECK(if5::heartbeat_state({true, false, degraded, false}) == NodeState::DEGRADED);
  CHECK(if5::heartbeat_state({true, true, valid, false}) == NodeState::DEGRADED); // stale
  CHECK(if5::heartbeat_state({true, false, valid, true}) == NodeState::DEGRADED); // port
}

void test_saturate() {
  CHECK(if5::saturate_u8(0U) == 0U);
  CHECK(if5::saturate_u8(255U) == 255U);
  CHECK(if5::saturate_u8(256U) == 255U);
  CHECK(if5::saturate_u8(0xFFFFFFFFU) == 255U);
}

void test_estop_monitor_observes_without_latch() {
  if5::EstopMonitor m;
  CHECK(!m.stop_seen(0U));
  m.on_estop({3U, if5::EstopCommand::STOP, 3U, 1U, 0U}, 1000U);
  CHECK(m.stop_seen(1000U) && m.stop_seen(1300U));
  CHECK(!m.stop_seen(1301U)); // sender stopped repeating
  m.on_estop({3U, if5::EstopCommand::STOP, 3U, 1U, 1U}, 1400U);
  m.on_estop({2U, if5::EstopCommand::RELEASE, 2U, 0U, 0U}, 1450U);
  CHECK(!m.stop_seen(1450U));
  CHECK(m.stop_count() == 2U && m.release_count() == 1U);
  // Tick wrap.
  m.on_estop({3U, if5::EstopCommand::STOP, 3U, 1U, 2U}, 0xFFFFFF00U);
  CHECK(m.stop_seen(0x10U));
}

void test_peer_table() {
  if5::PeerTable t;
  CHECK(!t.is_online(1U, 3U, 0U));
  t.on_heartbeat(1U, 3U, if5::NodeState::RUN, 100U);
  CHECK(t.is_online(1U, 3U, 600U));
  CHECK(!t.is_online(1U, 3U, 601U));
  CHECK(!t.is_online(0U, 3U, 100U)); // ports are separate
  CHECK(t.state(1U, 3U) == if5::NodeState::RUN);
  t.on_heartbeat(1U, 0x40U, if5::NodeState::RUN, 100U); // out of table: ignored
  CHECK(!t.ever_seen(1U, 0x40U));
  t.on_heartbeat(0U, 2U, if5::NodeState::READY, 0xFFFFFF00U);
  CHECK(t.is_online(0U, 2U, 0xF0U)); // tick wrap, 496 ms later
}

} // namespace

int main() {
  test_pack_heartbeat_layout();
  test_parse_estop_valid_stop_and_release();
  test_parse_estop_rejects_malformed();
  test_parse_cfg_request();
  test_parse_heartbeat();
  test_rx_id_filter();
  test_pack_cfg_response();
  test_answer_cfg();
  test_heartbeat_state_precedence();
  test_saturate();
  test_estop_monitor_observes_without_latch();
  test_peer_table();
  if (failures != 0) {
    std::printf("%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("if5 host tests passed\n");
  return 0;
}
