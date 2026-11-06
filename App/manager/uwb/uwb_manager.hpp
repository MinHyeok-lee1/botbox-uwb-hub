#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "singleton_base.hpp"
#include "task.h"
#include "uwb_data_instance.hpp"
#include "uwb_follow_config.hpp"
#include "uwb_move_maker.hpp"
#include "uwb_vector.hpp"

enum class UwbFollowMode : uint8_t {
  DEFAULT,
  FRONT_ONLY,
  BACK_ONLY,
  LEFT_BACK,
  RIGHT_BACK,
  LEFT_ONLY,
  RIGHT_ONLY,
};

// A decision may include a transition state or an unimplemented fallback mode.
// Unimplemented modes must be handled as a safe stop by the caller.
enum class UwbModeDecision : uint8_t {
  STOP,
  KEEP_CURRENT_MODE,
  DEFAULT,
  FRONT_ONLY,
  BACK_ONLY,
  LEFT_BACK,
  RIGHT_BACK,
  LEFT_ONLY,
  RIGHT_ONLY,
};

static constexpr uint32_t UWB_SAMPLE_AGE_UNKNOWN_MS = UINT32_MAX;

// Bits of UwbManager::used_sensor_mask().
static constexpr uint8_t UWB_SENSOR_LEFT = 1U << 0;
static constexpr uint8_t UWB_SENSOR_RIGHT = 1U << 1;
static constexpr uint8_t UWB_SENSOR_BACK = 1U << 2;

// Why the hub has no steering result. Stopping for distance is the drive
// board's decision and is not reported here.
enum class UwbStopReason : uint8_t {
  NONE,
  NOT_FOLLOWING,
  SENSOR_TIMEOUT,
  SENSOR_LOSS,
};

// Which botbox+ move/stop check the drive board applies to the range_*()
// distances. The hub has already put the right sensor value in each slot for
// the current follow mode; a slot that the check does not use holds
// UWB_INVALID_VALUE.
enum class UwbRangeRule : uint8_t {
  NONE,         // no usable distance
  THREE_SENSOR, // front and back ranges, left/right for the close-stop check
  FRONT_ONLY,   // front range, left/right for the close-stop check
  BACK_ONLY,    // back range only
};

enum class UwbSensorFault : uint8_t {
  NONE,
  TIMEOUT,
  SIGNAL_LOSS,
  PACKET_BROKEN,
  INVALID_DISTANCE,
};

class UwbManager : public Singleton<UwbManager> {
  friend class Singleton<UwbManager>;

private:
  UwbManager();

  UwbDataInstance _leftUwb;
  UwbDataInstance _rightUwb;
  UwbDataInstance _backUwb;

  int16_t _distanceLeft;
  int16_t _distanceRight;
  int16_t _distanceBack;
  // Left/Right are kept for future single-sensor fallback extensions.
  int16_t _lastAliveDistanceLeft;
  int16_t _lastAliveDistanceRight;
  int16_t _lastAliveDistanceBack;

  int16_t _calculatedDistanceFront;

  bool _isFollowingAvailable;
  UwbFollowMode _followMode;
  bool _isFrontOnlyModeEnabled;
  bool _isBackOnlyModeEnabled;
  bool _isLeftBackModeEnabled;
  bool _isRightBackModeEnabled;
  bool _isLeftOnlyModeEnabled;
  bool _isRightOnlyModeEnabled;
  TickType_t _modeTransitionStartTick;
  bool _isModeTransitionTiming;
  UwbFollowMode _modeDecisionBuffer[UWB_MODE_DECISION_BUFFER_CAPACITY];
  uint8_t _modeDecisionBufferIndex;
  uint8_t _modeDecisionBufferCount;

  UwbDirection _direction;
  UwbTurn _turn;
  UwbTurn _backTurn;
  uint32_t _turnRatio;
  bool _isTurnAngleSampleSynchronized;
  bool _isSteeringCurrent;
  UwbRangeRule _rangeRule;
  int16_t _rangeFront;
  int16_t _rangeBack;
  int16_t _rangeLeft;
  int16_t _rangeRight;
  UwbStopReason _stopReason;
  UwbVector _vector;
  uint32_t _sampleAgeMs;

  UwbMoveMaker _maker;
  UwbVectorEstimator _vectorEstimator;

  void update_follow_state();
  void update_output_estimates();
  uint32_t current_sample_age_ms() const;
  bool are_front_samples_synchronized() const;

  void stop_following();
  void apply_mode_decision(UwbModeDecision modeDecision, bool isLeftSignalLost,
                           bool isRightSignalLost, bool isBackSignalLost);
  void reset_mode_transition();
  UwbModeDecision decide_mode(bool isLeftSignalLost, bool isRightSignalLost,
                              bool isBackSignalLost,
                              bool hasCurrentFollowMode);
  bool is_follow_mode_available(UwbFollowMode mode, bool isLeftSignalLost,
                                bool isRightSignalLost,
                                bool isBackSignalLost) const;
  UwbModeDecision apply_mode_transition(UwbModeDecision requestedMode,
                                        bool isCurrentModeAvailable);
  void update_received_distances();
  void update_last_alive_distances(bool isLeftSignalLost,
                                   bool isRightSignalLost,
                                   bool isBackSignalLost);
  void update_front_distance();
  void update_back_distance();
  void update_left_back_distances();
  void update_right_back_distances();
  void update_left_only_distance();
  void update_right_only_distance();
  bool are_three_sensor_samples_synchronized() const;
  void apply_front_only_mode();
  void apply_default_mode();
  void apply_back_only_mode();
  void apply_left_back_mode();
  void apply_right_back_mode();
  void apply_left_only_mode();
  void apply_right_only_mode();
  void apply_current_follow_mode();
  void apply_close_back_turn_limit();
  void set_ranges(UwbRangeRule rule, int16_t front, int16_t back,
                  int16_t left, int16_t right);

public:
  void init();
  bool start();
  void pause();
  void update();

  void on_receive(CANRxMessage *message);
  void restart();

  bool is_valid() const { return _isFollowingAvailable; };
  UwbFollowMode follow_mode() const { return _followMode; }
  bool is_front_only_mode() const {
    return _followMode == UwbFollowMode::FRONT_ONLY;
  }
  bool is_back_only_mode() const {
    return _followMode == UwbFollowMode::BACK_ONLY;
  }
  bool is_front_only_mode_enabled() const {
    return _isFrontOnlyModeEnabled;
  }
  bool is_back_only_mode_enabled() const {
    return _isBackOnlyModeEnabled;
  }
  bool is_left_back_mode_enabled() const {
    return _isLeftBackModeEnabled;
  }
  bool is_right_back_mode_enabled() const {
    return _isRightBackModeEnabled;
  }
  bool is_left_only_mode_enabled() const {
    return _isLeftOnlyModeEnabled;
  }
  bool is_right_only_mode_enabled() const {
    return _isRightOnlyModeEnabled;
  }
  bool is_timeout();

  UwbDirection get_direction() const { return _direction; }
  UwbTurn get_turn() const { return _turn; }
  UwbTurn get_back_turn() const { return _backTurn; }
  uint8_t get_turn_ratio() const { return _turnRatio; }
  int16_t get_turn_angle_deg() const { return _maker.turn_angle_deg(); }
  bool is_turn_angle_sample_synchronized() const {
    return _isTurnAngleSampleSynchronized;
  }
  // False when the steering values were not recalculated in the last
  // update(): they are then left over from an earlier cycle.
  bool is_steering_current() const { return _isSteeringCurrent; }
  // True while a direct FORWARD <-> BACKWARD change is waiting to be confirmed
  // and get_direction() still returns the previous direction.
  bool is_direction_flip_pending() const {
    return _isSteeringCurrent && _followMode == UwbFollowMode::DEFAULT &&
           _maker.is_direction_flip_pending();
  }
  UwbRangeRule range_rule() const { return _rangeRule; }
  int16_t range_front() const { return _rangeFront; }
  int16_t range_back() const { return _rangeBack; }
  int16_t range_left() const { return _rangeLeft; }
  int16_t range_right() const { return _rangeRight; }
  UwbStopReason get_stop_reason() const { return _stopReason; }
  // Tag position for the CAN2 output; quality NONE when there is none.
  const UwbVector &vector() const { return _vector; }
  // Age, at the last update(), of the oldest sensor sample the current follow
  // mode uses. UWB_SAMPLE_AGE_UNKNOWN_MS while not following.
  uint32_t sample_age_ms() const { return _sampleAgeMs; }
  // UWB_SENSOR_* bits of the sensors the current follow mode uses; 0 while
  // not following.
  uint8_t used_sensor_mask() const;

  int16_t left_distance() const { return _distanceLeft; }
  int16_t right_distance() const { return _distanceRight; }
  int16_t front_distance() const { return _calculatedDistanceFront; }
  int16_t back_distance() const { return _distanceBack; }

  bool left_timeout() {
    return _leftUwb.is_timed_out(
        uwb_follow_configuration().receive_timeout_ms);
  }
  bool right_timeout() {
    return _rightUwb.is_timed_out(
        uwb_follow_configuration().receive_timeout_ms);
  }
  bool back_timeout() {
    if (uwb_follow_configuration().back_uwb_enabled == 0U) {
      return false;
    }
    return _backUwb.is_timed_out(
        uwb_follow_configuration().receive_timeout_ms);
  }
  UwbSensorFault left_sensor_fault() const {
    if (_leftUwb.is_timed_out(
            uwb_follow_configuration().receive_timeout_ms)) {
      return UwbSensorFault::TIMEOUT;
    }
    if (_leftUwb.is_packet_broken()) {
      return UwbSensorFault::PACKET_BROKEN;
    }
    if (_leftUwb.is_distance_invalid()) {
      return UwbSensorFault::INVALID_DISTANCE;
    }
    return _leftUwb.is_tag_lost() ? UwbSensorFault::SIGNAL_LOSS
                                  : UwbSensorFault::NONE;
  }
  UwbSensorFault right_sensor_fault() const {
    if (_rightUwb.is_timed_out(
            uwb_follow_configuration().receive_timeout_ms)) {
      return UwbSensorFault::TIMEOUT;
    }
    if (_rightUwb.is_packet_broken()) {
      return UwbSensorFault::PACKET_BROKEN;
    }
    if (_rightUwb.is_distance_invalid()) {
      return UwbSensorFault::INVALID_DISTANCE;
    }
    return _rightUwb.is_tag_lost() ? UwbSensorFault::SIGNAL_LOSS
                                   : UwbSensorFault::NONE;
  }
  UwbSensorFault back_sensor_fault() const {
    if (uwb_follow_configuration().back_uwb_enabled == 0U) {
      return UwbSensorFault::NONE;
    }
    if (_backUwb.is_timed_out(
            uwb_follow_configuration().receive_timeout_ms)) {
      return UwbSensorFault::TIMEOUT;
    }
    if (_backUwb.is_packet_broken()) {
      return UwbSensorFault::PACKET_BROKEN;
    }
    if (_backUwb.is_distance_invalid()) {
      return UwbSensorFault::INVALID_DISTANCE;
    }
    return _backUwb.is_tag_lost() ? UwbSensorFault::SIGNAL_LOSS
                                  : UwbSensorFault::NONE;
  }
  bool left_signal_lost() const {
    return _leftUwb.is_tag_lost() || _leftUwb.is_packet_broken() ||
           _leftUwb.is_distance_invalid();
  }
  bool right_signal_lost() const {
    return _rightUwb.is_tag_lost() || _rightUwb.is_packet_broken() ||
           _rightUwb.is_distance_invalid();
  }
  bool back_signal_lost() const {
    if (uwb_follow_configuration().back_uwb_enabled == 0U) {
      return false;
    }
    return _backUwb.is_tag_lost() || _backUwb.is_packet_broken() ||
           _backUwb.is_distance_invalid();
  }

  uint32_t left_rx_count() const { return _leftUwb.rx_count(); }
  uint32_t right_rx_count() const { return _rightUwb.rx_count(); }
  uint32_t back_rx_count() const { return _backUwb.rx_count(); }
  uint32_t left_same_distance_count() const {
    return _leftUwb.same_distance_count();
  }
  uint32_t right_same_distance_count() const {
    return _rightUwb.same_distance_count();
  }
  uint32_t back_same_distance_count() const {
    return _backUwb.same_distance_count();
  }
  uint32_t left_tag_loss_count() const {
    return _leftUwb.tag_loss_count();
  }
  uint32_t right_tag_loss_count() const {
    return _rightUwb.tag_loss_count();
  }
  uint32_t back_tag_loss_count() const {
    return _backUwb.tag_loss_count();
  }
  uint32_t left_packet_broken_count() const {
    return _leftUwb.packet_broken_count();
  }
  uint32_t right_packet_broken_count() const {
    return _rightUwb.packet_broken_count();
  }
  uint32_t back_packet_broken_count() const {
    return _backUwb.packet_broken_count();
  }
  const uint8_t *left_payload() const { return _leftUwb.last_payload(); }
  const uint8_t *right_payload() const { return _rightUwb.last_payload(); }
  const uint8_t *back_payload() const { return _backUwb.last_payload(); }
};
