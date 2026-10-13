#include "uwb_move_maker.hpp"

#include <algorithm>
#include <cmath>

#include "FreeRTOS.h"
#include "gpio_input_manager.hpp"
#include "task.h"
#include "uwb_direction_angle.hpp"

UwbMoveMaker::UwbMoveMaker() { reset(); }

UwbMoveMaker::~UwbMoveMaker() {}

void UwbMoveMaker::reset() {
  _moveBufferIndex = 0;
  for (uint8_t i = 0; i < UWB_MOVE_BUFFER_CAPACITY; ++i) {
    _moveBuffer[i] = UwbMove::STOP;
  }
  _prevDirection       = UwbDirection::UNKNOWN;
  _prevTurn            = UwbTurn::UNKNOWN;
  _prevBackTurn        = UwbTurn::UNKNOWN;
  _prevMove            = UwbMove::STOP;
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
  _prevDirection = direction;
  return direction;
}

// ---------------------------------------------------------------------------
// _move_buffer_update (내부 공통 헬퍼)
// ---------------------------------------------------------------------------
static UwbMove move_buffer_update(UwbMove* buf, uint8_t& idx,
                                  uint8_t maxSize, UwbMove move,
                                  bool shouldFastResume) {
  if (shouldFastResume) {
    for (uint8_t i = 0; i < maxSize; ++i) {
      buf[i] = UwbMove::MOVE;
    }
    idx = 0;
    return UwbMove::MOVE;
  }

  buf[idx++] = move;
  if (idx >= maxSize) idx = 0;

  uint8_t cnt = 0;
  for (uint8_t i = 0; i < maxSize; ++i) {
    if (buf[i] == UwbMove::MOVE) cnt++;
  }
  return (cnt > maxSize / 2) ? UwbMove::MOVE : UwbMove::STOP;
}

static bool is_close_stop_distance(int16_t distance, uint16_t minStop) {
  return distance >= 0 && distance <= minStop;
}

UwbMove UwbMoveMaker::calculate_move(UwbDirection dir, int16_t distanceFront,
                                     int16_t distanceBack) {
  return calculate_move(dir, distanceFront, distanceBack, distanceFront,
                        distanceFront);
}

UwbMove UwbMoveMaker::calculate_move(UwbDirection dir, int16_t distanceFront,
                                     int16_t distanceBack, int16_t distanceLeft,
                                     int16_t distanceRight) {
  const auto &configuration = uwb_follow_configuration();
  auto speedLevel = GpioInputManager::get_instance().speed_switch_state();

  const auto minMoveConst =
      speedLevel == SpeedSwitchState::ADC_LOW_LEVEL_1
          ? configuration.min_move_level1_mm
      : speedLevel == SpeedSwitchState::ADC_MID_LEVEL_2
          ? configuration.min_move_level2_mm
          : configuration.min_move_level3_mm;
  const auto minStopConst =
      speedLevel == SpeedSwitchState::ADC_LOW_LEVEL_1
          ? configuration.min_stop_level1_mm
      : speedLevel == SpeedSwitchState::ADC_MID_LEVEL_2
          ? configuration.min_stop_level2_mm
          : configuration.min_stop_level3_mm;
  const auto minMove =
      (dir == UwbDirection::MID) ? minMoveConst * 3 / 2 : minMoveConst;
  const auto minStop =
      (dir == UwbDirection::MID) ? minStopConst * 3 / 2 : minStopConst;

  const auto minMoveBackConst =
      speedLevel == SpeedSwitchState::ADC_LOW_LEVEL_1
          ? configuration.min_move_back_level1_mm
      : speedLevel == SpeedSwitchState::ADC_MID_LEVEL_2
          ? configuration.min_move_back_level2_mm
          : configuration.min_move_back_level3_mm;
  const auto minStopBackConst =
      speedLevel == SpeedSwitchState::ADC_LOW_LEVEL_1
          ? configuration.min_stop_back_level1_mm
      : speedLevel == SpeedSwitchState::ADC_MID_LEVEL_2
          ? configuration.min_stop_back_level2_mm
          : configuration.min_stop_back_level3_mm;
  const auto minMoveBack =
      (dir == UwbDirection::MID) ? minMoveBackConst * 3 / 2 : minMoveBackConst;
  const auto minStopBack =
      (dir == UwbDirection::MID) ? minStopBackConst * 3 / 2 : minStopBackConst;

  bool shouldMove = false;
  bool shouldStop = false;
  const bool isForwardCloseStop =
      dir == UwbDirection::FORWARD &&
      (is_close_stop_distance(distanceFront, minStop) ||
       is_close_stop_distance(distanceLeft, minStop) ||
       is_close_stop_distance(distanceRight, minStop));

  if ((static_cast<int32_t>(distanceFront) <
           static_cast<int32_t>(configuration.max_move_mm) &&
       static_cast<int32_t>(distanceFront) >
           static_cast<int32_t>(minMove)) &&
      (static_cast<int32_t>(distanceBack) <
           static_cast<int32_t>(configuration.max_move_mm) &&
       static_cast<int32_t>(distanceBack) >
           static_cast<int32_t>(minMoveBack))) {
    shouldMove = true;
  }
  if ((static_cast<int32_t>(distanceFront) >=
           static_cast<int32_t>(configuration.max_stop_mm) ||
       (dir == UwbDirection::FORWARD ? isForwardCloseStop
                                     : static_cast<int32_t>(distanceFront) <=
                                           static_cast<int32_t>(minStop))) ||
      (static_cast<int32_t>(distanceBack) >=
           static_cast<int32_t>(configuration.max_stop_mm) ||
       static_cast<int32_t>(distanceBack) <=
           static_cast<int32_t>(minStopBack))) {
    shouldStop = true;
  }

  const UwbMove prevMove = _prevMove;
  UwbMove move = _prevMove;
  if (shouldMove && !shouldStop) {
    move = UwbMove::MOVE;
  } else if (!shouldMove && shouldStop) {
    move = UwbMove::STOP;
  }
  _prevMove = move;

  return move_buffer_update(_moveBuffer, _moveBufferIndex,
                            static_cast<uint8_t>(
                                configuration.move_buffer_size),
                            move,
                            prevMove == UwbMove::STOP &&
                                move == UwbMove::MOVE);
}

// ---------------------------------------------------------------------------
// calculate_front_only_move
// ---------------------------------------------------------------------------
UwbMove UwbMoveMaker::calculate_front_only_move(int16_t distanceFront) {
  return calculate_front_only_move(distanceFront, distanceFront, distanceFront);
}

UwbMove UwbMoveMaker::calculate_front_only_move(int16_t distanceFront,
                                                int16_t distanceLeft,
                                                int16_t distanceRight) {
  const auto &configuration = uwb_follow_configuration();
  auto speedLevel = GpioInputManager::get_instance().speed_switch_state();

  const auto minMove =
      speedLevel == SpeedSwitchState::ADC_LOW_LEVEL_1
          ? configuration.min_move_level1_mm
      : speedLevel == SpeedSwitchState::ADC_MID_LEVEL_2
          ? configuration.min_move_level2_mm
          : configuration.min_move_level3_mm;
  const auto minStop =
      speedLevel == SpeedSwitchState::ADC_LOW_LEVEL_1
          ? configuration.min_stop_level1_mm
      : speedLevel == SpeedSwitchState::ADC_MID_LEVEL_2
          ? configuration.min_stop_level2_mm
          : configuration.min_stop_level3_mm;

  const bool shouldMove =
      (static_cast<int32_t>(distanceFront) <
       static_cast<int32_t>(configuration.max_move_mm)) &&
      (static_cast<int32_t>(distanceFront) > static_cast<int32_t>(minMove));
  const bool shouldStop =
      (static_cast<int32_t>(distanceFront) >=
       static_cast<int32_t>(configuration.max_stop_mm)) ||
      is_close_stop_distance(distanceFront, minStop) ||
      is_close_stop_distance(distanceLeft, minStop) ||
      is_close_stop_distance(distanceRight, minStop);

  const UwbMove prevMove = _prevMove;
  UwbMove move = _prevMove;
  if (shouldMove && !shouldStop) {
    move = UwbMove::MOVE;
  } else if (!shouldMove && shouldStop) {
    move = UwbMove::STOP;
  }
  _prevMove = move;

  return move_buffer_update(_moveBuffer, _moveBufferIndex,
                            static_cast<uint8_t>(
                                configuration.move_buffer_size),
                            move,
                            prevMove == UwbMove::STOP &&
                                move == UwbMove::MOVE);
}

// ---------------------------------------------------------------------------
// calculate_back_only_move
// ---------------------------------------------------------------------------
UwbMove UwbMoveMaker::calculate_back_only_move(int16_t distanceBack) {
  const auto &configuration = uwb_follow_configuration();
  const auto speedLevel = GpioInputManager::get_instance().speed_switch_state();

  const auto minMoveBack =
      speedLevel == SpeedSwitchState::ADC_LOW_LEVEL_1
          ? configuration.min_move_back_level1_mm
      : speedLevel == SpeedSwitchState::ADC_MID_LEVEL_2
          ? configuration.min_move_back_level2_mm
          : configuration.min_move_back_level3_mm;
  const auto minStopBack =
      speedLevel == SpeedSwitchState::ADC_LOW_LEVEL_1
          ? configuration.min_stop_back_level1_mm
      : speedLevel == SpeedSwitchState::ADC_MID_LEVEL_2
          ? configuration.min_stop_back_level2_mm
          : configuration.min_stop_back_level3_mm;

  const bool shouldMove =
      (static_cast<int32_t>(distanceBack) <
       static_cast<int32_t>(configuration.max_move_mm)) &&
      (static_cast<int32_t>(distanceBack) >
       static_cast<int32_t>(minMoveBack));
  const bool shouldStop =
      (static_cast<int32_t>(distanceBack) >=
       static_cast<int32_t>(configuration.max_stop_mm)) ||
      (static_cast<int32_t>(distanceBack) <=
       static_cast<int32_t>(minStopBack));

  const UwbMove prevMove = _prevMove;
  UwbMove move = _prevMove;
  if (shouldMove && !shouldStop) {
    move = UwbMove::MOVE;
  } else if (!shouldMove && shouldStop) {
    move = UwbMove::STOP;
  }
  _prevMove = move;

  return move_buffer_update(_moveBuffer, _moveBufferIndex,
                            static_cast<uint8_t>(
                                configuration.move_buffer_size),
                            move,
                            prevMove == UwbMove::STOP &&
                                move == UwbMove::MOVE);
}
