#include "uwb_data_instance.hpp"

namespace {

// XOR of the bytes before the checksum byte.
uint8_t xor_checksum(const uint8_t *bytes, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++) {
    checksum ^= bytes[i];
  }
  return checksum;
}

} // namespace

void UwbDataInstance::reset() {
  _valid = false;
  _distance = UWB_INVALID_VALUE;
  _lastRxTick = 0;
  _hasReceived = false;
  _tagLost = false;
  _packetBroken = false;
  _distanceInvalid = false;
  uwb_distance_lpf_reset(&_filter);
}

void UwbDataInstance::on_receive(CANRxMessage *message) {
  if (message == nullptr)
    return;

  _lastRxTick = xTaskGetTickCount();
  _hasReceived = true;

  UwbPacket packet;
  for (uint8_t i = 0; i < UWB_PACKET_SIZE; i++) {
    packet.byteStream[i] = message->data[i];
    _lastPayload[i] = message->data[i];
  }

  const bool startOk = packet.start == UWB_PACKET_START_VALUE;
  const bool checksumOk =
      xor_checksum(packet.byteStream, UWB_PACKET_SIZE - 1) == packet.checksum;
  _packetBroken = !startOk || !checksumOk;
  // -1 is the protocol's explicit tag-loss value. Any smaller value is not a
  // physical distance and is reported separately from framing/checksum errors.
  _distanceInvalid =
      !_packetBroken && packet.distance < UWB_INVALID_VALUE;
  _tagLost = !_packetBroken && !_distanceInvalid &&
             (packet.distance == UWB_INVALID_VALUE);
  _valid = !_packetBroken && !_distanceInvalid && !_tagLost;

  _rxCount++;
  if (_tagLost)
    _tagLossCount++;
  if (_packetBroken)
    _packetBrokenCount++;
  if (packet.distance == _lastParsedDistance) {
    _sameDistanceCount++;
  } else {
    _sameDistanceCount = 0;
  }
  _lastParsedDistance = packet.distance;
  _distance = packet.distance;
}
