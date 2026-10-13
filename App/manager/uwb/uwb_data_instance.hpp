#pragma once

// State of one UWB sensor: parses its CAN packet and keeps the latest distance
// and fault flags. Taken from botbox+ uwb_can_controller.hpp; the CAN bus
// controller that lived in the same file is gone because the hub's can_port
// driver owns the bus.

#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>

extern "C" {
#include "uwb_filter.h"
}

// 기존 UWB 패킷 구조 유지
union UwbPacket {
  struct __attribute__((__packed__)) {
    uint8_t start;
    int16_t distance;
    uint8_t checksum;
  };
  uint8_t byteStream[4];
};

constexpr uint8_t UWB_PACKET_SIZE = sizeof(UwbPacket);
constexpr uint8_t UWB_PACKET_START_VALUE = 0xAA;
// 기존 코드에서 int8_t였으나 distance가 int16_t이므로 수정하여 캐스팅 이슈 방지
static constexpr int16_t UWB_INVALID_VALUE = -1;

static constexpr uint16_t UWB_CAN_BASE_ADDRESS = 0x0200;
static constexpr uint8_t UWB_CAN_LEFT_NODE = 0x01;
static constexpr uint8_t UWB_CAN_RIGHT_NODE = 0x02;
static constexpr uint8_t UWB_CAN_BACK_NODE = 0x03;

// One received sensor frame, filled from the hub's can_frame_t. Only the
// fields the UWB code reads are kept from the botbox+ type of the same name.
constexpr uint8_t UWB_CAN_MAX_DATA_SIZE = 8;

struct CANRxMessage {
  uint32_t nodeID; // CAN identifier, e.g. 0x201 for the left sensor
  uint8_t data[UWB_CAN_MAX_DATA_SIZE];
};

class UwbDataInstance {
public:
  UwbDataInstance()
      : _distance(UWB_INVALID_VALUE), _valid(false), _lastRxTick(0),
        _rxCount(0), _sameDistanceCount(0), _tagLossCount(0),
        _packetBrokenCount(0), _lastParsedDistance(UWB_INVALID_VALUE),
        _hasReceived(false), _tagLost(false), _packetBroken(false),
        _distanceInvalid(false),
        _lastPayload{0}, _filter{} {
    uwb_distance_lpf_reset(&_filter);
  }

  void reset();

  void on_receive(CANRxMessage *message);

  bool is_valid() const { return _valid; }
  bool has_received() const { return _hasReceived; }
  bool is_tag_lost() const { return _tagLost; }
  bool is_packet_broken() const { return _packetBroken; }
  bool is_distance_invalid() const { return _distanceInvalid; }
  bool is_timed_out(uint32_t ms) const {
    if (_lastRxTick == 0) {
      return true;
    }
    return (xTaskGetTickCount() - _lastRxTick) > pdMS_TO_TICKS(ms);
  }
  int16_t get_raw_distance() const { return _distance; }
  int16_t get_filtered_distance() {
    return static_cast<int16_t>(
        uwb_distance_lpf_update_mm(&_filter, static_cast<int32_t>(_distance)));
  }
  int16_t get_distance(bool filterEnabled) {
    return filterEnabled ? get_filtered_distance() : get_raw_distance();
  }
  uint32_t rx_count() const { return _rxCount; }
  uint32_t same_distance_count() const { return _sameDistanceCount; }
  uint32_t tag_loss_count() const { return _tagLossCount; }
  uint32_t packet_broken_count() const { return _packetBrokenCount; }
  uint32_t last_rx_tick() const { return _lastRxTick; }
  const uint8_t *last_payload() const { return _lastPayload; }

private:
  int16_t _distance;
  bool _valid;
  uint32_t _lastRxTick;
  uint32_t _rxCount;
  uint32_t _sameDistanceCount;
  uint32_t _tagLossCount;
  uint32_t _packetBrokenCount;
  int16_t _lastParsedDistance;
  bool _hasReceived;
  bool _tagLost;
  bool _packetBroken;
  bool _distanceInvalid;
  uint8_t _lastPayload[UWB_PACKET_SIZE];

  uwb_distance_lpf_t _filter;
};
