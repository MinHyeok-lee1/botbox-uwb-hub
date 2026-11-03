#include "uwb_move_maker.hpp"

#include <algorithm>
#include <cmath>

#include "FreeRTOS.h"
#include "task.h"
#include "uwb_direction_angle.hpp"

UwbMoveMaker::UwbMoveMaker() { reset(); }

UwbMoveMaker::~UwbMoveMaker() {}

void UwbMoveMaker::reset() {
  _prevDirection       = UwbDirection::UNKNOWN;
  _prevTurn            = UwbTurn::UNKNOWN;
  _prevBackTurn        = UwbTurn::UNKNOWN;
  _directionFlipCount  = 0;
  _smoothedTurnRatio   = uwb_follow_configuration().initial_turn_ratio_percent;
  _smoothedTurnAngleDeg = 0.0f;
  _turnRatioDirection  = UwbDirection::UNKNOWN;
  _directionChangeTick = 0U;
  _isDirectionChangeHoldActive = false;
}

UwbTurn UwbMoveMaker::calculate_turn(int16_t distanceLeft,
                                     int16_t distanceRight) {
  const auto &configuration = uwb_follow_configuration();
  int16_t diff = distanceLeft - distanceRight;
  UwbTurn turn =
      (std::abs(diff) < configuration.center_in_threshold_lr_mm)
          ? UwbTurn::CENTER
      : (std::abs(diff) > configuration.center_out_threshold_lr_mm)
          ? ((diff > 0) ? UwbTurn::RIGHT : UwbTurn::LEFT)
          : _prevTurn;
  _prevTurn = turn;
  return turn;
}

UwbTurn UwbMoveMaker::calculate_back_turn(int16_t distanceLeft,
                                          int16_t distanceRight) {
  const auto &configuration = uwb_follow_configuration();
  int16_t diff = distanceLeft - distanceRight;
  UwbTurn turn =
      (std::abs(diff) < configuration.center_back_in_threshold_lr_mm)
          ? UwbTurn::CENTER
      : (std::abs(diff) > configuration.center_back_out_threshold_lr_mm)
          ? ((diff > 0) ? UwbTurn::RIGHT : UwbTurn::LEFT)
          : _prevBackTurn;
  _prevBackTurn = turn;
  return turn;
}

uint8_t UwbMoveMaker::calculate_turn_ratio(int16_t distanceLeft,
                                           int16_t distanceRight,
                                           UwbDirection dir) {
  const auto &configuration = uwb_follow_configuration();
  const float rawAngleDeg = calculate_front_only_uwb_angle(
      distanceLeft, distanceRight, configuration.front_sensor_spacing_mm);
  return update_turn_ratio_from_angle(rawAngleDeg, dir, true);
}

uint8_t UwbMoveMaker::calculate_turn_ratio(
    int16_t distanceLeft, int16_t distanceRight, int16_t distanceFront,
    int16_t distanceBack, UwbDirection dir, bool areSamplesSynchronized) {
  const auto &configuration = uwb_follow_configuration();
  const UwbDirectionAngle directionAngle = calculate_uwb_direction_angle(
      distanceLeft, distanceRight, distanceFront, distanceBack,
      configuration.front_sensor_spacing_mm,
      configuration.front_rear_sensor_spacing_mm,
      dir == UwbDirection::BACKWARD);
  return update_turn_ratio_from_angle(directionAngle.angleDeg, dir,
                                      areSamplesSynchronized);
}

uint8_t UwbMoveMaker::update_turn_ratio_from_angle(float rawAngleDeg,
                                                   UwbDirection dir,
                                                   bool allowAngleUpdate) {
  const auto &configuration = uwb_follow_configuration();
  const uint32_t nowTick = static_cast<uint32_t>(xTaskGetTickCount());

  const bool isTravelDirection =
      dir == UwbDirection::FORWARD || dir == UwbDirection::BACKWARD;
  if (isTravelDirection) {
    if (_turnRatioDirection == UwbDirection::UNKNOWN) {
      _turnRatioDirection = dir;
    } else if (dir != _turnRatioDirection) {
      _turnRatioDirection = dir;
      _directionChangeTick = nowTick;
      _isDirectionChangeHoldActive = true;
      _smoothedTurnAngleDeg = 0.0f;
      _smoothedTurnRatio = 100.0f;
    }
  }

  if (_isDirectionChangeHoldActive) {
    const uint32_t holdTicks = static_cast<uint32_t>(
        pdMS_TO_TICKS(configuration.direction_change_turn_hold_ms));
    if ((nowTick - _directionChangeTick) < holdTicks) {
      return 100U;
    }
    _isDirectionChangeHoldActive = false;
  }

  if (!isTravelDirection || !allowAngleUpdate ||
      !std::isfinite(rawAngleDeg)) {
    return static_cast<uint8_t>(_smoothedTurnRatio);
  }

  const float angleTarget =
      _smoothedTurnAngleDeg +
      ((rawAngleDeg - _smoothedTurnAngleDeg) *
       configuration.turn_angle_smoothing_alpha);
  const float angleDelta = std::max(
      -configuration.turn_angle_max_step_deg,
      std::min(configuration.turn_angle_max_step_deg,
               angleTarget - _smoothedTurnAngleDeg));
  _smoothedTurnAngleDeg =
      std::max(0.0f, std::min(90.0f, _smoothedTurnAngleDeg + angleDelta));

  const uint32_t minimumTurnRatio =
      dir == UwbDirection::BACKWARD
          ? configuration.min_turn_ratio_backward_percent
          : configuration.min_turn_ratio_forward_percent;
  const uint8_t targetTurnRatio = calculate_uwb_turn_ratio_target(
      _smoothedTurnAngleDeg, configuration.turn_angle_deadzone_deg,
      configuration.turn_angle_full_scale_deg, minimumTurnRatio);

  _smoothedTurnRatio =
      (_smoothedTurnRatio *
       (1.0f - configuration.turn_ratio_smoothing_alpha)) +
      (static_cast<float>(targetTurnRatio) *
       configuration.turn_ratio_smoothing_alpha);

  return static_cast<uint8_t>(_smoothedTurnRatio);
}

UwbDirection UwbMoveMaker::calculate_direction(int16_t distanceFront,
                                               int16_t distanceBack) {
  const auto &configuration = uwb_follow_configuration();
  UwbDirection direction    = _prevDirection;
  int16_t      diff         = distanceFront - distanceBack;
  uint16_t     absDiff      = static_cast<uint16_t>(std::abs(diff));

  uint16_t dynamicMidThreshold = (_prevDirection == UwbDirection::BACKWARD)
      ? static_cast<uint16_t>(configuration.mid_threshold_fb_mm / 2U)
      : static_cast<uint16_t>(configuration.mid_threshold_fb_mm);

  if (absDiff < dynamicMidThreshold) {
    direction = UwbDirection::MID;
  } else if (absDiff > configuration.stop_threshold_fb_mm) {
    direction = (diff > 0) ? UwbDirection::BACKWARD : UwbDirection::FORWARD;
  }

  // A tag that really moves from the front to the back passes the side of the
  // cart, which reads as MID. A change straight from FORWARD to BACKWARD (or
  // back) is therefore more likely a bad sample, so it must repeat for
  // direction_flip_confirm_count cycles in a row before it is accepted.
  const bool isDirectFlip = (_prevDirection == UwbDirection::FORWARD &&
                             direction == UwbDirection::BACKWARD) ||
                            (_prevDirection == UwbDirection::BACKWARD &&
                             direction == UwbDirection::FORWARD);
  if (isDirectFlip) {
    _directionFlipCount++;
    if (_directionFlipCount < configuration.direction_flip_confirm_count) {
      return _prevDirection;
    }
  }
  _directionFlipCount = 0;

  _prevDirection = direction;
  return direction;
}
