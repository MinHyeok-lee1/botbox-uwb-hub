#include "uwb_manager.hpp"
#include "diagnostic_manager.hpp"

#include <algorithm>
#include <cmath>

#include "gpio_input_manager.hpp"
#include "log_manager.hpp"

namespace {

uint32_t tick_distance(uint32_t left, uint32_t right) {
  return std::min(left - right, right - left);
}

} // namespace

UwbManager::UwbManager()
    : _distanceLeft(UWB_INVALID_VALUE),
      _distanceRight(UWB_INVALID_VALUE),
      _distanceBack(UWB_INVALID_VALUE),
      _lastAliveDistanceLeft(UWB_INVALID_VALUE),
      _lastAliveDistanceRight(UWB_INVALID_VALUE),
      _lastAliveDistanceBack(UWB_INVALID_VALUE),
      _calculatedDistanceFront(UWB_INVALID_VALUE),
      _isFollowingAvailable(false), _followMode(UwbFollowMode::DEFAULT),
      _isFrontOnlyModeEnabled(true),
      _isBackOnlyModeEnabled(true),
      _isLeftBackModeEnabled(true),
      _isRightBackModeEnabled(true),
      _isLeftOnlyModeEnabled(true),
      _isRightOnlyModeEnabled(true),
      _modeTransitionStartTick(0), _isModeTransitionTiming(false),
      _modeDecisionBuffer{},
      _modeDecisionBufferIndex(0),
      _modeDecisionBufferCount(0),
      _hasReliableMotionHistory(false),
      _lastReliableFollowMode(UwbFollowMode::DEFAULT),
      _lastReliableDirection(UwbDirection::UNKNOWN),
      _lastReliableTurn(UwbTurn::UNKNOWN),
      _lastReliableBackTurn(UwbTurn::UNKNOWN),
      _lastReliableTurnRatio(50), _lastReliableMotionTick(0),
      _direction(UwbDirection::UNKNOWN), _turn(UwbTurn::UNKNOWN),
      _backTurn(UwbTurn::UNKNOWN), _turnRatio(50),
      _isTurnAngleSampleSynchronized(false), _move(UwbMove::STOP),
      _stopReason(UwbStopReason::NOT_FOLLOWING) {}

// The hub's can_port driver owns the CAN bus, so there is nothing to set up or
// start here. Both functions are kept for the callers.
void UwbManager::init() {}

bool UwbManager::start() { return true; }

void UwbManager::pause() {
  _leftUwb.reset();
  _rightUwb.reset();
  _backUwb.reset();
  _distanceLeft = UWB_INVALID_VALUE;
  _distanceRight = UWB_INVALID_VALUE;
  _distanceBack = UWB_INVALID_VALUE;
  _lastAliveDistanceLeft = UWB_INVALID_VALUE;
  _lastAliveDistanceRight = UWB_INVALID_VALUE;
  _lastAliveDistanceBack = UWB_INVALID_VALUE;
  _calculatedDistanceFront = UWB_INVALID_VALUE;
  stop_following();

  DIAG().clear_error(SystemError::UWB_LEFT_TIMEOUT_FAULT);
  DIAG().clear_error(SystemError::UWB_RIGHT_TIMEOUT_FAULT);
  DIAG().clear_error(SystemError::UWB_BACK_TIMEOUT_FAULT);
  DIAG().clear_error(SystemError::UWB_LEFT_SENSOR_FAULT);
  DIAG().clear_error(SystemError::UWB_RIGHT_SENSOR_FAULT);
  DIAG().clear_error(SystemError::UWB_BACK_SENSOR_FAULT);
}

void UwbManager::update() {
  const auto &configuration = uwb_follow_configuration();
  const bool isBackUwbEnabled = configuration.back_uwb_enabled != 0U;
  // AUTO start and sensor recovery have no active mode to preserve, so their
  // first usable sensor combination must not enter transition hysteresis.
  const bool hasCurrentFollowMode = _isFollowingAvailable;
  update_received_distances();

  const bool isLeftTimedOut =
      _leftUwb.is_timed_out(configuration.receive_timeout_ms);
  const bool isRightTimedOut =
      _rightUwb.is_timed_out(configuration.receive_timeout_ms);
  const bool isBackTimedOut =
      isBackUwbEnabled &&
      _backUwb.is_timed_out(configuration.receive_timeout_ms);
  const bool hasAnySensorTimeout =
      isLeftTimedOut || isRightTimedOut || isBackTimedOut;

  const bool isLeftTagLost  = !isLeftTimedOut  && _leftUwb.is_tag_lost();
  const bool isRightTagLost = !isRightTimedOut && _rightUwb.is_tag_lost();
  const bool isBackTagLost =
      isBackUwbEnabled && !isBackTimedOut && _backUwb.is_tag_lost();

  const bool isLeftPacketBroken  = !isLeftTimedOut  && _leftUwb.is_packet_broken();
  const bool isRightPacketBroken = !isRightTimedOut && _rightUwb.is_packet_broken();
  const bool isBackPacketBroken =
      isBackUwbEnabled && !isBackTimedOut && _backUwb.is_packet_broken();

  const bool isLeftDistanceInvalid =
      !isLeftTimedOut && _leftUwb.is_distance_invalid();
  const bool isRightDistanceInvalid =
      !isRightTimedOut && _rightUwb.is_distance_invalid();
  const bool isBackDistanceInvalid =
      isBackUwbEnabled && !isBackTimedOut && _backUwb.is_distance_invalid();

  const bool isLeftSignalLost =
      isLeftTagLost || isLeftPacketBroken || isLeftDistanceInvalid;
  const bool isRightSignalLost =
      isRightTagLost || isRightPacketBroken || isRightDistanceInvalid;
  const bool isBackSignalLost =
      isBackTagLost || isBackPacketBroken || isBackDistanceInvalid;

  update_last_alive_distances(isLeftSignalLost, isRightSignalLost,
                              isBackSignalLost);

  isLeftTimedOut  ? DIAG().set_error(SystemError::UWB_LEFT_TIMEOUT_FAULT)
                  : DIAG().clear_error(SystemError::UWB_LEFT_TIMEOUT_FAULT);
  isRightTimedOut ? DIAG().set_error(SystemError::UWB_RIGHT_TIMEOUT_FAULT)
                  : DIAG().clear_error(SystemError::UWB_RIGHT_TIMEOUT_FAULT);
  isBackTimedOut  ? DIAG().set_error(SystemError::UWB_BACK_TIMEOUT_FAULT)
                  : DIAG().clear_error(SystemError::UWB_BACK_TIMEOUT_FAULT);

  isLeftSignalLost  ? DIAG().set_error(SystemError::UWB_LEFT_SENSOR_FAULT)
                    : DIAG().clear_error(SystemError::UWB_LEFT_SENSOR_FAULT);
  isRightSignalLost ? DIAG().set_error(SystemError::UWB_RIGHT_SENSOR_FAULT)
                    : DIAG().clear_error(SystemError::UWB_RIGHT_SENSOR_FAULT);
  isBackSignalLost  ? DIAG().set_error(SystemError::UWB_BACK_SENSOR_FAULT)
                    : DIAG().clear_error(SystemError::UWB_BACK_SENSOR_FAULT);

  if (hasAnySensorTimeout) {
    _stopReason = UwbStopReason::SENSOR_TIMEOUT;
    stop_following();
    return;
  }

  _isFollowingAvailable = true;
  _isTurnAngleSampleSynchronized = false;

  if (!isBackUwbEnabled) {
    if (isLeftSignalLost || isRightSignalLost) {
      _stopReason = UwbStopReason::SENSOR_LOSS;
      stop_following();
      return;
    }

    reset_mode_transition();
    update_front_distance();
    apply_front_only_mode();
    return;
  }

  const UwbModeDecision modeDecision =
      decide_mode(isLeftSignalLost, isRightSignalLost, isBackSignalLost,
                  hasCurrentFollowMode);

  switch (modeDecision) {
  case UwbModeDecision::FRONT_ONLY:
    update_front_distance();
    apply_front_only_mode();
    return;

  case UwbModeDecision::BACK_ONLY:
    update_back_distance();
    apply_back_only_mode();
    return;

  case UwbModeDecision::LEFT_BACK:
    update_left_back_distances();
    apply_left_back_mode();
    return;

  case UwbModeDecision::RIGHT_BACK:
    update_right_back_distances();
    apply_right_back_mode();
    return;

  case UwbModeDecision::LEFT_ONLY:
    update_left_only_distance();
    apply_left_only_mode();
    return;

  case UwbModeDecision::RIGHT_ONLY:
    update_right_only_distance();
    apply_right_only_mode();
    return;

  case UwbModeDecision::KEEP_CURRENT_MODE:
    if (is_follow_mode_available(_followMode, isLeftSignalLost,
                                 isRightSignalLost, isBackSignalLost)) {
      apply_current_follow_mode();
    }
    return;

  case UwbModeDecision::STOP:
    _stopReason = UwbStopReason::SENSOR_LOSS;
    stop_following();
    return;

  case UwbModeDecision::DEFAULT:
    update_front_distance();
    apply_default_mode();
    return;
  }
}

void UwbManager::stop_following() {
  _isFollowingAvailable = false;
  _followMode           = UwbFollowMode::DEFAULT;
  reset_mode_transition();
  _hasReliableMotionHistory = false;
  _lastReliableFollowMode   = UwbFollowMode::DEFAULT;
  _lastReliableDirection    = UwbDirection::UNKNOWN;
  _lastReliableTurn         = UwbTurn::UNKNOWN;
  _lastReliableBackTurn     = UwbTurn::UNKNOWN;
  _lastReliableTurnRatio    = 50;
  _lastReliableMotionTick   = 0;
  _direction = UwbDirection::UNKNOWN;
  _turn      = UwbTurn::UNKNOWN;
  _backTurn  = UwbTurn::UNKNOWN;
  _turnRatio = 50;
  _isTurnAngleSampleSynchronized = false;
  _move      = UwbMove::STOP;
  _maker.reset();
}

void UwbManager::reset_mode_transition() {
  _modeTransitionStartTick = 0;
  _isModeTransitionTiming  = false;
  _modeDecisionBufferIndex = 0;
  _modeDecisionBufferCount = 0;
  for (uint8_t i = 0; i < UWB_MODE_DECISION_BUFFER_CAPACITY; i++) {
    _modeDecisionBuffer[i] = UwbFollowMode::DEFAULT;
  }
}

UwbModeDecision UwbManager::decide_mode(bool isLeftSignalLost,
                                        bool isRightSignalLost,
                                        bool isBackSignalLost,
                                        bool hasCurrentFollowMode) {
  const bool isCurrentModeAvailable =
      hasCurrentFollowMode &&
      is_follow_mode_available(_followMode, isLeftSignalLost,
                               isRightSignalLost, isBackSignalLost);

  const auto apply_requested_mode =
      [this, hasCurrentFollowMode,
       isCurrentModeAvailable](UwbModeDecision requestedMode) {
        if (!hasCurrentFollowMode) {
          reset_mode_transition();
          return requestedMode;
        }
        return apply_mode_transition(requestedMode, isCurrentModeAvailable);
      };

  if (!isLeftSignalLost && !isRightSignalLost) {
    const UwbModeDecision requestedMode =
        isBackSignalLost && _isFrontOnlyModeEnabled
            ? UwbModeDecision::FRONT_ONLY
            : isBackSignalLost ? UwbModeDecision::STOP
                            : UwbModeDecision::DEFAULT;
    return apply_requested_mode(requestedMode);
  }

  if (isLeftSignalLost && isRightSignalLost && !isBackSignalLost) {
    return apply_requested_mode(_isBackOnlyModeEnabled
                                    ? UwbModeDecision::BACK_ONLY
                                    : UwbModeDecision::STOP);
  }

  if (!isLeftSignalLost && isRightSignalLost && !isBackSignalLost) {
    return apply_requested_mode(_isLeftBackModeEnabled
                                    ? UwbModeDecision::LEFT_BACK
                                    : UwbModeDecision::STOP);
  }

  if (isLeftSignalLost && !isRightSignalLost && !isBackSignalLost) {
    return apply_requested_mode(_isRightBackModeEnabled
                                    ? UwbModeDecision::RIGHT_BACK
                                    : UwbModeDecision::STOP);
  }

  if (!isLeftSignalLost && isRightSignalLost && isBackSignalLost) {
    return apply_requested_mode(_isLeftOnlyModeEnabled
                                    ? UwbModeDecision::LEFT_ONLY
                                    : UwbModeDecision::STOP);
  }

  if (isLeftSignalLost && !isRightSignalLost && isBackSignalLost) {
    return apply_requested_mode(_isRightOnlyModeEnabled
                                    ? UwbModeDecision::RIGHT_ONLY
                                    : UwbModeDecision::STOP);
  }

  WRITE_LOG("3 SENSOR LOSS STOP\r\n");

  return UwbModeDecision::STOP;
}

bool UwbManager::is_follow_mode_available(UwbFollowMode mode,
                                          bool isLeftSignalLost,
                                          bool isRightSignalLost,
                                          bool isBackSignalLost) const {
  switch (mode) {
  case UwbFollowMode::DEFAULT:
    return !isLeftSignalLost && !isRightSignalLost && !isBackSignalLost;
  case UwbFollowMode::FRONT_ONLY:
    return _isFrontOnlyModeEnabled && !isLeftSignalLost && !isRightSignalLost;
  case UwbFollowMode::BACK_ONLY:
    return _isBackOnlyModeEnabled && !isBackSignalLost;
  case UwbFollowMode::LEFT_BACK:
    return _isLeftBackModeEnabled && !isLeftSignalLost && !isBackSignalLost;
  case UwbFollowMode::RIGHT_BACK:
    return _isRightBackModeEnabled && !isRightSignalLost && !isBackSignalLost;
  case UwbFollowMode::LEFT_ONLY:
    return _isLeftOnlyModeEnabled && !isLeftSignalLost;
  case UwbFollowMode::RIGHT_ONLY:
    return _isRightOnlyModeEnabled && !isRightSignalLost;
  }

  return false;
}

UwbModeDecision
UwbManager::apply_mode_transition(UwbModeDecision requestedMode,
                                  bool isCurrentModeAvailable) {
  const auto &configuration = uwb_follow_configuration();
  const uint8_t bufferSize =
      static_cast<uint8_t>(configuration.mode_decision_buffer_size);

  if (requestedMode == UwbModeDecision::STOP) {
    reset_mode_transition();
    return UwbModeDecision::STOP;
  }

  UwbFollowMode requestedFollowMode = _followMode;
  switch (requestedMode) {
  case UwbModeDecision::DEFAULT:
    requestedFollowMode = UwbFollowMode::DEFAULT;    break;
  case UwbModeDecision::FRONT_ONLY:
    requestedFollowMode = UwbFollowMode::FRONT_ONLY; break;
  case UwbModeDecision::BACK_ONLY:
    requestedFollowMode = UwbFollowMode::BACK_ONLY;  break;
  case UwbModeDecision::LEFT_BACK:
    requestedFollowMode = UwbFollowMode::LEFT_BACK;  break;
  case UwbModeDecision::RIGHT_BACK:
    requestedFollowMode = UwbFollowMode::RIGHT_BACK; break;
  case UwbModeDecision::LEFT_ONLY:
    requestedFollowMode = UwbFollowMode::LEFT_ONLY;  break;
  case UwbModeDecision::RIGHT_ONLY:
    requestedFollowMode = UwbFollowMode::RIGHT_ONLY; break;
  case UwbModeDecision::STOP:
  case UwbModeDecision::KEEP_CURRENT_MODE:
    break;
  }

  if (requestedFollowMode == _followMode) {
    reset_mode_transition();
    return requestedMode;
  }

  const TickType_t nowTick = xTaskGetTickCount();
  if (!_isModeTransitionTiming) {
    _modeTransitionStartTick = nowTick;
    _isModeTransitionTiming  = true;
  }

  _modeDecisionBuffer[_modeDecisionBufferIndex] = requestedFollowMode;
  _modeDecisionBufferIndex =
      (_modeDecisionBufferIndex + 1U) % bufferSize;
  if (_modeDecisionBufferCount < bufferSize) {
    _modeDecisionBufferCount++;
  }

  if (_modeDecisionBufferCount >= bufferSize) {
    uint8_t modeCounts[UWB_FOLLOW_MODE_COUNT] = {0};
    for (uint8_t i = 0; i < bufferSize; i++) {
      const uint8_t modeIndex =
          static_cast<uint8_t>(_modeDecisionBuffer[i]);
      if (modeIndex < UWB_FOLLOW_MODE_COUNT) {
        modeCounts[modeIndex]++;
      }
    }

    const uint8_t requestedModeIndex =
        static_cast<uint8_t>(requestedFollowMode);
    if (requestedModeIndex < UWB_FOLLOW_MODE_COUNT &&
        modeCounts[requestedModeIndex] >=
            configuration.mode_decision_majority_count) {
      reset_mode_transition();
      return requestedMode;
    }
  }

  if ((nowTick - _modeTransitionStartTick) >=
      pdMS_TO_TICKS(configuration.mode_observation_timeout_ms)) {
    if (isCurrentModeAvailable) {
      WRITE_LOG("UWB MODE TRANSITION TIMEOUT KEEP CURRENT %u\r\n",
                static_cast<unsigned int>(_followMode));
      reset_mode_transition();
      return UwbModeDecision::KEEP_CURRENT_MODE;
    }

    WRITE_LOG("UWB MODE TRANSITION TIMEOUT USE LATEST %u\r\n",
              static_cast<unsigned int>(requestedMode));
    reset_mode_transition();
    return requestedMode;
  }

  return UwbModeDecision::KEEP_CURRENT_MODE;
}

void UwbManager::update_received_distances() {
  _distanceLeft = _leftUwb.get_raw_distance();
  _distanceRight = _rightUwb.get_raw_distance();
  _distanceBack = _backUwb.get_raw_distance();
}

void UwbManager::update_last_alive_distances(bool isLeftSignalLost,
                                             bool isRightSignalLost,
                                             bool isBackSignalLost) {
  if (!isLeftSignalLost && _distanceLeft != UWB_INVALID_VALUE) {
    _lastAliveDistanceLeft = _distanceLeft;
  }
  if (!isRightSignalLost && _distanceRight != UWB_INVALID_VALUE) {
    _lastAliveDistanceRight = _distanceRight;
  }
  if (!isBackSignalLost && _distanceBack != UWB_INVALID_VALUE) {
    _lastAliveDistanceBack = _distanceBack;
  }
}

void UwbManager::update_front_distance() {
  const auto &configuration = uwb_follow_configuration();
  const bool filterEnabled = configuration.distance_filter_enabled != 0U;
  _distanceLeft = _leftUwb.get_distance(filterEnabled);
  _distanceRight = _rightUwb.get_distance(filterEnabled);

  const float d          = configuration.front_sensor_spacing_mm;
  const float l          = static_cast<float>(_distanceLeft);
  const float r          = static_cast<float>(_distanceRight);
  const float innerValue =
      std::max(0.0f, (l * l + r * r) / 2.0f - (d * d) / 4.0f);
  _calculatedDistanceFront = static_cast<int16_t>(std::sqrt(innerValue));
}

void UwbManager::update_back_distance() {
  _distanceBack = _backUwb.get_distance(
      uwb_follow_configuration().distance_filter_enabled != 0U);
}

void UwbManager::update_left_back_distances() {
  _distanceLeft = _leftUwb.get_distance(
      uwb_follow_configuration().distance_filter_enabled != 0U);
  update_back_distance();
}

void UwbManager::update_right_back_distances() {
  _distanceRight = _rightUwb.get_distance(
      uwb_follow_configuration().distance_filter_enabled != 0U);
  update_back_distance();
}

void UwbManager::update_left_only_distance() {
  _distanceLeft = _leftUwb.get_distance(
      uwb_follow_configuration().distance_filter_enabled != 0U);
}

void UwbManager::update_right_only_distance() {
  _distanceRight = _rightUwb.get_distance(
      uwb_follow_configuration().distance_filter_enabled != 0U);
}

bool UwbManager::are_three_sensor_samples_synchronized() const {
  if (!_leftUwb.has_received() || !_rightUwb.has_received() ||
      !_backUwb.has_received()) {
    return false;
  }

  const uint32_t leftTick = _leftUwb.last_rx_tick();
  const uint32_t rightTick = _rightUwb.last_rx_tick();
  const uint32_t backTick = _backUwb.last_rx_tick();

  const uint32_t maxSkewTicks = static_cast<uint32_t>(
      pdMS_TO_TICKS(uwb_follow_configuration().max_sensor_sample_skew_ms));
  return tick_distance(leftTick, rightTick) <= maxSkewTicks &&
         tick_distance(leftTick, backTick) <= maxSkewTicks &&
         tick_distance(rightTick, backTick) <= maxSkewTicks;
}

void UwbManager::apply_front_only_mode() {
  _followMode = UwbFollowMode::FRONT_ONLY;
  _direction  = UwbDirection::FORWARD;
  _turn       = _maker.calculate_turn(_distanceLeft, _distanceRight);
  _backTurn   = UwbTurn::UNKNOWN;
  _turnRatio  = _maker.calculate_turn_ratio(_distanceLeft, _distanceRight,
                                            _direction);
  _move       = _maker.calculate_front_only_move(_calculatedDistanceFront,
                                                 _distanceLeft,
                                                 _distanceRight);
  update_stop_reason_from_move();
  if (keep_recent_motion_if_opposite()) {
    return;
  }
  remember_reliable_motion();
}

void UwbManager::apply_default_mode() {
  _followMode = UwbFollowMode::DEFAULT;
  reset_mode_transition();
  update_back_distance();

  _direction = _maker.calculate_direction(_calculatedDistanceFront,
                                          _distanceBack);
  _turn      = _maker.calculate_turn(_distanceLeft, _distanceRight);
  _backTurn  = _maker.calculate_back_turn(_distanceLeft, _distanceRight);
  _isTurnAngleSampleSynchronized = are_three_sensor_samples_synchronized();
  _turnRatio = _maker.calculate_turn_ratio(
      _distanceLeft, _distanceRight, _calculatedDistanceFront, _distanceBack,
      _direction, _isTurnAngleSampleSynchronized);
  apply_close_back_turn_limit();
  _move      = _maker.calculate_move(_direction, _calculatedDistanceFront,
                                     _distanceBack, _distanceLeft,
                                     _distanceRight);
  update_stop_reason_from_move();
  if (keep_recent_motion_if_opposite()) {
    return;
  }
  remember_reliable_motion();
}

void UwbManager::apply_back_only_mode() {
  _followMode = UwbFollowMode::BACK_ONLY;
  _direction  = UwbDirection::BACKWARD;
  _turn       = UwbTurn::UNKNOWN;
  _backTurn   = UwbTurn::CENTER;
  _turnRatio  = 50;
  const int16_t backDistanceForMove =
      (_distanceBack == UWB_INVALID_VALUE) ? _lastAliveDistanceBack
                                           : _distanceBack;
  _move       = _maker.calculate_back_only_move(backDistanceForMove);
  update_stop_reason_from_move();
  if (keep_recent_motion_if_opposite()) {
    return;
  }
  remember_reliable_motion();
}

void UwbManager::apply_left_back_mode() {
  _followMode = UwbFollowMode::LEFT_BACK;
  _turnRatio  = 50;

  if (_distanceLeft >= _distanceBack) {
    // The back sensor is closer to the tag: turn backward-left.
    _direction = UwbDirection::BACKWARD;
    _turn      = UwbTurn::UNKNOWN;
    _backTurn  = UwbTurn::RIGHT;
    apply_close_back_turn_limit();
    _move      = _maker.calculate_back_only_move(_distanceBack);
    update_stop_reason_from_move();
    if (keep_recent_motion_if_opposite()) {
      return;
    }
    remember_reliable_motion();
  } else if (_distanceBack > _distanceLeft) {
    // The left sensor is closer to the tag: turn left in place.
    _direction = UwbDirection::MID;
    _turn      = UwbTurn::LEFT;
    _backTurn  = UwbTurn::UNKNOWN;
    _move      = _maker.calculate_move(_direction, _distanceLeft, _distanceBack);
    update_stop_reason_from_move();
    if (keep_recent_motion_if_opposite()) {
      return;
    }
    remember_reliable_motion();
  }
}

void UwbManager::apply_right_back_mode() {
  _followMode = UwbFollowMode::RIGHT_BACK;
  _turnRatio  = 50;

  if (_distanceRight >= _distanceBack) {
    // The back sensor is closer to the tag: turn backward-right.
    _direction = UwbDirection::BACKWARD;
    _turn      = UwbTurn::UNKNOWN;
    _backTurn  = UwbTurn::LEFT;
    apply_close_back_turn_limit();
    _move      = _maker.calculate_back_only_move(_distanceBack);
    update_stop_reason_from_move();
    if (keep_recent_motion_if_opposite()) {
      return;
    }
    remember_reliable_motion();
  } else if (_distanceBack > _distanceRight) {
    // The right sensor is closer to the tag: turn right in place.
    _direction = UwbDirection::MID;
    _turn      = UwbTurn::RIGHT;
    _backTurn  = UwbTurn::UNKNOWN;
    _move      = _maker.calculate_move(_direction, _distanceRight, _distanceBack);
    update_stop_reason_from_move();
    if (keep_recent_motion_if_opposite()) {
      return;
    }
    remember_reliable_motion();
  }
}

void UwbManager::apply_left_only_mode() {
  _followMode = UwbFollowMode::LEFT_ONLY;
  _turnRatio  = _lastReliableTurnRatio;

  if (!is_left_only_escape_plausible()) {
    keep_recent_motion();
    return;
  }

  if (_lastReliableDirection == UwbDirection::BACKWARD &&
      _lastReliableBackTurn == UwbTurn::RIGHT) {
    _direction = UwbDirection::BACKWARD;
    _turn      = UwbTurn::UNKNOWN;
    _backTurn  = UwbTurn::RIGHT;
  } else {
    _direction = UwbDirection::MID;
    _turn      = UwbTurn::LEFT;
    _backTurn  = UwbTurn::UNKNOWN;
  }

  _move = _maker.calculate_front_only_move(_distanceLeft);
  update_stop_reason_from_move();
  keep_recent_motion_if_opposite();
}

void UwbManager::apply_right_only_mode() {
  _followMode = UwbFollowMode::RIGHT_ONLY;
  _turnRatio  = _lastReliableTurnRatio;

  if (!is_right_only_escape_plausible()) {
    keep_recent_motion();
    return;
  }

  if (_lastReliableDirection == UwbDirection::BACKWARD &&
      _lastReliableBackTurn == UwbTurn::LEFT) {
    _direction = UwbDirection::BACKWARD;
    _turn      = UwbTurn::UNKNOWN;
    _backTurn  = UwbTurn::LEFT;
  } else {
    _direction = UwbDirection::MID;
    _turn      = UwbTurn::RIGHT;
    _backTurn  = UwbTurn::UNKNOWN;
  }

  _move = _maker.calculate_front_only_move(_distanceRight);
  update_stop_reason_from_move();
  keep_recent_motion_if_opposite();
}

void UwbManager::apply_current_follow_mode() {
  switch (_followMode) {
  case UwbFollowMode::DEFAULT:
    update_front_distance();
    apply_default_mode();
    return;
  case UwbFollowMode::FRONT_ONLY:
    update_front_distance();
    apply_front_only_mode();
    return;
  case UwbFollowMode::BACK_ONLY:
    update_back_distance();
    apply_back_only_mode();
    return;
  case UwbFollowMode::LEFT_BACK:
    update_left_back_distances();
    apply_left_back_mode();
    return;
  case UwbFollowMode::RIGHT_BACK:
    update_right_back_distances();
    apply_right_back_mode();
    return;
  case UwbFollowMode::LEFT_ONLY:
    update_left_only_distance();
    apply_left_only_mode();
    return;
  case UwbFollowMode::RIGHT_ONLY:
    update_right_only_distance();
    apply_right_only_mode();
    return;
  }
}

void UwbManager::apply_close_back_turn_limit() {
  const auto &configuration = uwb_follow_configuration();
  if (_direction != UwbDirection::BACKWARD ||
      _distanceBack == UWB_INVALID_VALUE ||
      static_cast<int32_t>(_distanceBack) >
          static_cast<int32_t>(
              configuration.close_back_turn_limit_distance_mm) ||
      _turnRatio >= configuration.close_back_min_turn_ratio_percent) {
    return;
  }

  _turnRatio = static_cast<uint8_t>(
      configuration.close_back_min_turn_ratio_percent);
}

void UwbManager::remember_reliable_motion() {
  if (_move != UwbMove::MOVE) {
    return;
  }

  _hasReliableMotionHistory = true;
  _lastReliableFollowMode   = _followMode;
  _lastReliableDirection    = _direction;
  _lastReliableTurn         = _turn;
  _lastReliableBackTurn     = _backTurn;
  _lastReliableTurnRatio    = _turnRatio;
  _lastReliableMotionTick   = xTaskGetTickCount();
}

bool UwbManager::keep_recent_motion() {
  if (!has_recent_reliable_motion()) {
    return false;
  }

  _followMode = _lastReliableFollowMode;
  _direction  = _lastReliableDirection;
  _turn       = _lastReliableTurn;
  _backTurn   = _lastReliableBackTurn;
  _turnRatio  = _lastReliableTurnRatio;
  _move       = UwbMove::MOVE;
  _stopReason = UwbStopReason::NONE;
  return true;
}

bool UwbManager::has_recent_reliable_motion() const {
  if (!_hasReliableMotionHistory) {
    return false;
  }

  if ((xTaskGetTickCount() - _lastReliableMotionTick) >
      pdMS_TO_TICKS(
          uwb_follow_configuration().single_only_history_timeout_ms)) {
    return false;
  }

  return true;
}

bool UwbManager::is_opposite_to_recent_motion(UwbDirection direction) const {
  if (!has_recent_reliable_motion()) {
    return false;
  }

  return (_lastReliableDirection == UwbDirection::FORWARD &&
          direction == UwbDirection::BACKWARD) ||
         (_lastReliableDirection == UwbDirection::BACKWARD &&
          direction == UwbDirection::FORWARD);
}

bool UwbManager::keep_recent_motion_if_opposite() {
  if (_move != UwbMove::MOVE || !is_opposite_to_recent_motion(_direction)) {
    return false;
  }

  return keep_recent_motion();
}

bool UwbManager::is_left_only_escape_plausible() const {
  if (!_hasReliableMotionHistory) {
    return false;
  }

  if ((xTaskGetTickCount() - _lastReliableMotionTick) >
      pdMS_TO_TICKS(
          uwb_follow_configuration().single_only_history_timeout_ms)) {
    return false;
  }

  if (_lastReliableFollowMode == UwbFollowMode::LEFT_BACK) {
    return true;
  }

  if (_lastReliableDirection == UwbDirection::MID) {
    return _lastReliableTurn == UwbTurn::LEFT;
  }

  if (_lastReliableDirection == UwbDirection::FORWARD) {
    return _lastReliableTurn == UwbTurn::LEFT ||
           _lastReliableTurn == UwbTurn::CENTER;
  }

  if (_lastReliableDirection == UwbDirection::BACKWARD) {
    // OutputManager maps backTurn RIGHT to physical backward-left.
    return _lastReliableBackTurn == UwbTurn::RIGHT;
  }

  return false;
}

bool UwbManager::is_right_only_escape_plausible() const {
  if (!_hasReliableMotionHistory) {
    return false;
  }

  if ((xTaskGetTickCount() - _lastReliableMotionTick) >
      pdMS_TO_TICKS(
          uwb_follow_configuration().single_only_history_timeout_ms)) {
    return false;
  }

  if (_lastReliableFollowMode == UwbFollowMode::RIGHT_BACK) {
    return true;
  }

  if (_lastReliableDirection == UwbDirection::MID) {
    return _lastReliableTurn == UwbTurn::RIGHT;
  }

  if (_lastReliableDirection == UwbDirection::FORWARD) {
    return _lastReliableTurn == UwbTurn::RIGHT ||
           _lastReliableTurn == UwbTurn::CENTER;
  }

  if (_lastReliableDirection == UwbDirection::BACKWARD) {
    // OutputManager maps backTurn LEFT to physical backward-right.
    return _lastReliableBackTurn == UwbTurn::LEFT;
  }

  return false;
}

void UwbManager::update_stop_reason_from_move() {
  _stopReason = (_move == UwbMove::STOP)
                    ? UwbStopReason::DISTANCE_LIMIT_OR_BUFFER
                    : UwbStopReason::NONE;
}

uint8_t uwbAddr = 0x0;
void UwbManager::on_receive(CANRxMessage *message) {
  if (message == nullptr) return;

  uwbAddr = (message->nodeID & 0x0F);
  if (uwbAddr == UWB_CAN_LEFT_NODE) {
    _leftUwb.on_receive(message);
  } else if (uwbAddr == UWB_CAN_RIGHT_NODE) {
    _rightUwb.on_receive(message);
  } else if (uwbAddr == UWB_CAN_BACK_NODE) {
    _backUwb.on_receive(message);
  }
}

void UwbManager::restart() {
  _leftUwb.reset();
  _rightUwb.reset();
  _backUwb.reset();
  _lastAliveDistanceLeft = UWB_INVALID_VALUE;
  _lastAliveDistanceRight = UWB_INVALID_VALUE;
  _lastAliveDistanceBack = UWB_INVALID_VALUE;
}

const bool UwbManager::is_timeout() {
  const auto &configuration = uwb_follow_configuration();
  const uint32_t timeout = configuration.receive_timeout_ms;
  bool isTimeout = _leftUwb.is_timed_out(timeout) ||
                   _rightUwb.is_timed_out(timeout) ||
                   (configuration.back_uwb_enabled != 0U &&
                    _backUwb.is_timed_out(timeout));

  if (isTimeout) {
    _isFollowingAvailable = false;
  }
  return isTimeout;
}
