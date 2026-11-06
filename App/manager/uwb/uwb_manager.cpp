#include "uwb_manager.hpp"

#include <algorithm>
#include <cmath>

#include "console.h"

namespace {

// Turn ratio reported when the distances give no steering information.
constexpr uint32_t NEUTRAL_TURN_RATIO_PERCENT = 50U;

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
      _direction(UwbDirection::UNKNOWN), _turn(UwbTurn::UNKNOWN),
      _backTurn(UwbTurn::UNKNOWN), _turnRatio(NEUTRAL_TURN_RATIO_PERCENT),
      _isTurnAngleSampleSynchronized(false), _isSteeringCurrent(false),
      _rangeRule(UwbRangeRule::NONE), _rangeFront(UWB_INVALID_VALUE),
      _rangeBack(UWB_INVALID_VALUE), _rangeLeft(UWB_INVALID_VALUE),
      _rangeRight(UWB_INVALID_VALUE),
      _stopReason(UwbStopReason::NOT_FOLLOWING),
      _vector{UwbVectorQuality::NONE, UWB_INVALID_VALUE,
              UWB_VECTOR_BEARING_NONE},
      _sampleAgeMs(UWB_SAMPLE_AGE_UNKNOWN_MS) {}

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
}

void UwbManager::update() {
  update_follow_state();
  update_output_estimates();
}

// Values that only the CAN2 output uses; the follow decision does not depend
// on them.
void UwbManager::update_output_estimates() {
  _sampleAgeMs = current_sample_age_ms();

  UwbVectorQuality requested = UwbVectorQuality::NONE;
  if (_isFollowingAvailable && _isSteeringCurrent) {
    if (_followMode == UwbFollowMode::DEFAULT &&
        are_three_sensor_samples_synchronized()) {
      requested = UwbVectorQuality::THREE_SENSOR;
    } else if (_followMode == UwbFollowMode::FRONT_ONLY &&
               are_front_samples_synchronized()) {
      requested = UwbVectorQuality::FRONT_ASSUMED;
    }
  }
  _vector = _vectorEstimator.update(requested, _distanceLeft, _distanceRight,
                                    _distanceBack);
}

uint8_t UwbManager::used_sensor_mask() const {
  if (!_isFollowingAvailable) {
    return 0U;
  }

  switch (_followMode) {
  case UwbFollowMode::DEFAULT:
    return UWB_SENSOR_LEFT | UWB_SENSOR_RIGHT | UWB_SENSOR_BACK;
  case UwbFollowMode::FRONT_ONLY:
    return UWB_SENSOR_LEFT | UWB_SENSOR_RIGHT;
  case UwbFollowMode::BACK_ONLY:
    return UWB_SENSOR_BACK;
  case UwbFollowMode::LEFT_BACK:
    return UWB_SENSOR_LEFT | UWB_SENSOR_BACK;
  case UwbFollowMode::RIGHT_BACK:
    return UWB_SENSOR_RIGHT | UWB_SENSOR_BACK;
  case UwbFollowMode::LEFT_ONLY:
    return UWB_SENSOR_LEFT;
  case UwbFollowMode::RIGHT_ONLY:
    return UWB_SENSOR_RIGHT;
  }
  return 0U;
}

uint32_t UwbManager::current_sample_age_ms() const {
  const uint8_t sensors = used_sensor_mask();
  if (sensors == 0U) {
    return UWB_SAMPLE_AGE_UNKNOWN_MS;
  }

  const TickType_t now = xTaskGetTickCount();
  TickType_t oldest = 0U;
  if ((sensors & UWB_SENSOR_LEFT) != 0U) {
    oldest = std::max<TickType_t>(oldest, now - _leftUwb.last_rx_tick());
  }
  if ((sensors & UWB_SENSOR_RIGHT) != 0U) {
    oldest = std::max<TickType_t>(oldest, now - _rightUwb.last_rx_tick());
  }
  if ((sensors & UWB_SENSOR_BACK) != 0U) {
    oldest = std::max<TickType_t>(oldest, now - _backUwb.last_rx_tick());
  }
  return static_cast<uint32_t>(oldest) * portTICK_PERIOD_MS;
}

void UwbManager::update_follow_state() {
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

  if (hasAnySensorTimeout) {
    _stopReason = UwbStopReason::SENSOR_TIMEOUT;
    stop_following();
    return;
  }

  _isFollowingAvailable = true;
  _isTurnAngleSampleSynchronized = false;
  // Set again by the mode handler that runs below. It stays false in a cycle
  // where the current mode lost its sensors and the next mode is not decided
  // yet, so the receiver can tell that the steering values are old.
  _isSteeringCurrent = false;
  _stopReason = UwbStopReason::NONE;

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

  apply_mode_decision(modeDecision, isLeftSignalLost, isRightSignalLost,
                      isBackSignalLost);

  // The 3-sensor direction is only calculated by apply_default_mode(), which
  // leaves the mode at DEFAULT with current steering. In any other cycle an
  // unconfirmed direction flip is dropped.
  if (_followMode != UwbFollowMode::DEFAULT || !_isSteeringCurrent) {
    _maker.cancel_direction_flip();
  }
}

void UwbManager::apply_mode_decision(UwbModeDecision modeDecision,
                                     bool isLeftSignalLost,
                                     bool isRightSignalLost,
                                     bool isBackSignalLost) {
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
  _direction = UwbDirection::UNKNOWN;
  _turn      = UwbTurn::UNKNOWN;
  _backTurn  = UwbTurn::UNKNOWN;
  _turnRatio = NEUTRAL_TURN_RATIO_PERCENT;
  _isTurnAngleSampleSynchronized = false;
  _isSteeringCurrent = false;
  set_ranges(UwbRangeRule::NONE, UWB_INVALID_VALUE, UWB_INVALID_VALUE,
             UWB_INVALID_VALUE, UWB_INVALID_VALUE);
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

  console_printf("3 SENSOR LOSS STOP\r\n");

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
      console_printf("UWB MODE TRANSITION TIMEOUT KEEP CURRENT %u\r\n",
                static_cast<unsigned int>(_followMode));
      reset_mode_transition();
      return UwbModeDecision::KEEP_CURRENT_MODE;
    }

    console_printf("UWB MODE TRANSITION TIMEOUT USE LATEST %u\r\n",
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

bool UwbManager::are_front_samples_synchronized() const {
  if (!_leftUwb.has_received() || !_rightUwb.has_received()) {
    return false;
  }
  const uint32_t maxSkewTicks = static_cast<uint32_t>(
      pdMS_TO_TICKS(uwb_follow_configuration().max_sensor_sample_skew_ms));
  return tick_distance(_leftUwb.last_rx_tick(), _rightUwb.last_rx_tick()) <=
         maxSkewTicks;
}

void UwbManager::set_ranges(UwbRangeRule rule, int16_t front, int16_t back,
                            int16_t left, int16_t right) {
  _rangeRule  = rule;
  _rangeFront = front;
  _rangeBack  = back;
  _rangeLeft  = left;
  _rangeRight = right;
}

void UwbManager::apply_front_only_mode() {
  _followMode = UwbFollowMode::FRONT_ONLY;
  _direction  = UwbDirection::FORWARD;
  _turn       = _maker.calculate_turn(_distanceLeft, _distanceRight);
  _backTurn   = UwbTurn::UNKNOWN;
  _turnRatio  = _maker.calculate_turn_ratio(_distanceLeft, _distanceRight,
                                            _direction);
  set_ranges(UwbRangeRule::FRONT_ONLY, _calculatedDistanceFront,
             UWB_INVALID_VALUE, _distanceLeft, _distanceRight);
  _isSteeringCurrent = true;
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
  set_ranges(UwbRangeRule::THREE_SENSOR, _calculatedDistanceFront,
             _distanceBack, _distanceLeft, _distanceRight);
  _isSteeringCurrent = true;
}

void UwbManager::apply_back_only_mode() {
  _followMode = UwbFollowMode::BACK_ONLY;
  _direction  = UwbDirection::BACKWARD;
  _turn       = UwbTurn::UNKNOWN;
  _backTurn   = UwbTurn::CENTER;
  _turnRatio  = NEUTRAL_TURN_RATIO_PERCENT;
  const int16_t backDistanceForMove =
      (_distanceBack == UWB_INVALID_VALUE) ? _lastAliveDistanceBack
                                           : _distanceBack;
  set_ranges(UwbRangeRule::BACK_ONLY, UWB_INVALID_VALUE, backDistanceForMove,
             UWB_INVALID_VALUE, UWB_INVALID_VALUE);
  _isSteeringCurrent = true;
}

void UwbManager::apply_left_back_mode() {
  _followMode = UwbFollowMode::LEFT_BACK;
  _turnRatio  = NEUTRAL_TURN_RATIO_PERCENT;

  if (_distanceLeft >= _distanceBack) {
    // The back sensor is closer: use the left-side target bearing
    // (botbox+ hotfix 321ccc1, 2026-11-05).
    _direction = UwbDirection::BACKWARD;
    _turn      = UwbTurn::UNKNOWN;
    _backTurn  = UwbTurn::LEFT;
    apply_close_back_turn_limit();
    set_ranges(UwbRangeRule::BACK_ONLY, UWB_INVALID_VALUE, _distanceBack,
               UWB_INVALID_VALUE, UWB_INVALID_VALUE);
  } else {
    // The left sensor is closer to the tag: turn left in place.
    _direction = UwbDirection::MID;
    _turn      = UwbTurn::LEFT;
    _backTurn  = UwbTurn::UNKNOWN;
    set_ranges(UwbRangeRule::THREE_SENSOR, _distanceLeft, _distanceBack,
               _distanceLeft, _distanceLeft);
  }
  _isSteeringCurrent = true;
}

void UwbManager::apply_right_back_mode() {
  _followMode = UwbFollowMode::RIGHT_BACK;
  _turnRatio  = NEUTRAL_TURN_RATIO_PERCENT;

  if (_distanceRight >= _distanceBack) {
    // The back sensor is closer: use the right-side target bearing
    // (botbox+ hotfix 321ccc1, 2026-11-05).
    _direction = UwbDirection::BACKWARD;
    _turn      = UwbTurn::UNKNOWN;
    _backTurn  = UwbTurn::RIGHT;
    apply_close_back_turn_limit();
    set_ranges(UwbRangeRule::BACK_ONLY, UWB_INVALID_VALUE, _distanceBack,
               UWB_INVALID_VALUE, UWB_INVALID_VALUE);
  } else {
    // The right sensor is closer to the tag: turn right in place.
    _direction = UwbDirection::MID;
    _turn      = UwbTurn::RIGHT;
    _backTurn  = UwbTurn::UNKNOWN;
    set_ranges(UwbRangeRule::THREE_SENSOR, _distanceRight, _distanceBack,
               _distanceRight, _distanceRight);
  }
  _isSteeringCurrent = true;
}

// One front sensor alone cannot tell where the tag is. botbox+ chose the
// direction from its record of recent motion; the hub has no such record, so
// it reports the mode and the distance and leaves the direction to the drive
// board.
void UwbManager::apply_left_only_mode() {
  _followMode = UwbFollowMode::LEFT_ONLY;
  _direction  = UwbDirection::UNKNOWN;
  _turn       = UwbTurn::UNKNOWN;
  _backTurn   = UwbTurn::UNKNOWN;
  _turnRatio  = NEUTRAL_TURN_RATIO_PERCENT;
  set_ranges(UwbRangeRule::FRONT_ONLY, _distanceLeft, UWB_INVALID_VALUE,
             _distanceLeft, _distanceLeft);
  _isSteeringCurrent = true;
}

void UwbManager::apply_right_only_mode() {
  _followMode = UwbFollowMode::RIGHT_ONLY;
  _direction  = UwbDirection::UNKNOWN;
  _turn       = UwbTurn::UNKNOWN;
  _backTurn   = UwbTurn::UNKNOWN;
  _turnRatio  = NEUTRAL_TURN_RATIO_PERCENT;
  set_ranges(UwbRangeRule::FRONT_ONLY, _distanceRight, UWB_INVALID_VALUE,
             _distanceRight, _distanceRight);
  _isSteeringCurrent = true;
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

uint8_t uwbAddr = 0x0;
void UwbManager::on_receive(CANRxMessage *message) {
  if (message == nullptr) return;

  const uint8_t uwbAddr = (message->nodeID & 0x0F);
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

bool UwbManager::is_timeout() {
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
