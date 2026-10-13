#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "singleton_base.hpp"
#include "task.h"
#include "uwb_data_instance.hpp"
#include "uwb_follow_config.hpp"
#include "uwb_move_maker.hpp"

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

enum class UwbStopReason : uint8_t {
  NONE,
  NOT_FOLLOWING,
  SENSOR_TIMEOUT,
  SENSOR_LOSS,
  DISTANCE_LIMIT_OR_BUFFER,
  MOTION_HISTORY_MISMATCH,
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

  bool _hasReliableMotionHistory;
  UwbFollowMode _lastReliableFollowMode;
  UwbDirection _lastReliableDirection;
  UwbTurn _lastReliableTurn;
  UwbTurn _lastReliableBackTurn;
  uint32_t _lastReliableTurnRatio;
  TickType_t _lastReliableMotionTick;

  UwbDirection _direction;
  UwbTurn _turn;
  UwbTurn _backTurn;
  uint32_t _turnRatio;
  bool _isTurnAngleSampleSynchronized;
  UwbMove _move;
  UwbStopReason _stopReason;

  UwbMoveMaker _maker;

  void stop_following();
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
  void remember_reliable_motion();
  bool has_recent_reliable_motion() const;
  bool keep_recent_motion();
  bool is_opposite_to_recent_motion(UwbDirection direction) const;
  bool keep_recent_motion_if_opposite();
  bool is_left_only_escape_plausible() const;
  bool is_right_only_escape_plausible() const;
  void update_stop_reason_from_move();

public:
  void init();
  bool start();
  void pause();
  void update();

  void on_receive(CANRxMessage *message);
  void restart();

  const bool is_valid() const { return _isFollowingAvailable; };
  const UwbFollowMode follow_mode() const { return _followMode; }
  const bool is_front_only_mode() const {
    return _followMode == UwbFollowMode::FRONT_ONLY;
  }
  const bool is_back_only_mode() const {
    return _followMode == UwbFollowMode::BACK_ONLY;
  }
  const bool is_front_only_mode_enabled() const {
    return _isFrontOnlyModeEnabled;
  }
  const bool is_back_only_mode_enabled() const {
    return _isBackOnlyModeEnabled;
  }
  const bool is_left_back_mode_enabled() const {
    return _isLeftBackModeEnabled;
  }
  const bool is_right_back_mode_enabled() const {
    return _isRightBackModeEnabled;
  }
  const bool is_left_only_mode_enabled() const {
    return _isLeftOnlyModeEnabled;
  }
  const bool is_right_only_mode_enabled() const {
    return _isRightOnlyModeEnabled;
  }
  const bool is_timeout();

  const UwbDirection get_direction() const { return _direction; }
  const UwbTurn get_turn() const { return _turn; }
  const UwbTurn get_back_turn() const { return _backTurn; }
  const uint8_t get_turn_ratio() const { return _turnRatio; }
  const int16_t get_turn_angle_deg() const { return _maker.turn_angle_deg(); }
  const bool is_turn_angle_sample_synchronized() const {
    return _isTurnAngleSampleSynchronized;
  }
  const UwbMove get_move() const { return _move; }
  const UwbStopReason get_stop_reason() const { return _stopReason; }

  const int16_t left_distance() const { return _distanceLeft; }
  const int16_t right_distance() const { return _distanceRight; }
  const int16_t front_distance() const { return _calculatedDistanceFront; }
  const int16_t back_distance() const { return _distanceBack; }

  const bool left_timeout() {
    return _leftUwb.is_timed_out(
        uwb_follow_configuration().receive_timeout_ms);
  }
  const bool right_timeout() {
    return _rightUwb.is_timed_out(
        uwb_follow_configuration().receive_timeout_ms);
  }
  const bool back_timeout() {
    if (uwb_follow_configuration().back_uwb_enabled == 0U) {
      return false;
    }
    return _backUwb.is_timed_out(
        uwb_follow_configuration().receive_timeout_ms);
  }
  const UwbSensorFault left_sensor_fault() const {
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
  const UwbSensorFault right_sensor_fault() const {
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
  const UwbSensorFault back_sensor_fault() const {
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
  const bool left_signal_lost() const {
    return _leftUwb.is_tag_lost() || _leftUwb.is_packet_broken() ||
           _leftUwb.is_distance_invalid();
  }
  const bool right_signal_lost() const {
    return _rightUwb.is_tag_lost() || _rightUwb.is_packet_broken() ||
           _rightUwb.is_distance_invalid();
  }
  const bool back_signal_lost() const {
    if (uwb_follow_configuration().back_uwb_enabled == 0U) {
      return false;
    }
    return _backUwb.is_tag_lost() || _backUwb.is_packet_broken() ||
           _backUwb.is_distance_invalid();
  }

  const uint32_t left_rx_count() const { return _leftUwb.rx_count(); }
  const uint32_t right_rx_count() const { return _rightUwb.rx_count(); }
  const uint32_t back_rx_count() const { return _backUwb.rx_count(); }
  const uint32_t left_same_distance_count() const {
    return _leftUwb.same_distance_count();
  }
  const uint32_t right_same_distance_count() const {
    return _rightUwb.same_distance_count();
  }
  const uint32_t back_same_distance_count() const {
    return _backUwb.same_distance_count();
  }
  const uint32_t left_tag_loss_count() const {
    return _leftUwb.tag_loss_count();
  }
  const uint32_t right_tag_loss_count() const {
    return _rightUwb.tag_loss_count();
  }
  const uint32_t back_tag_loss_count() const {
    return _backUwb.tag_loss_count();
  }
  const uint32_t left_packet_broken_count() const {
    return _leftUwb.packet_broken_count();
  }
  const uint32_t right_packet_broken_count() const {
    return _rightUwb.packet_broken_count();
  }
  const uint32_t back_packet_broken_count() const {
    return _backUwb.packet_broken_count();
  }
  const uint8_t *left_payload() const { return _leftUwb.last_payload(); }
  const uint8_t *right_payload() const { return _rightUwb.last_payload(); }
  const uint8_t *back_payload() const { return _backUwb.last_payload(); }
};
