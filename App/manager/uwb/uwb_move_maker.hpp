#pragma once

#include <cstdint>

#include "uwb_follow_config.hpp"

enum class UwbDirection : uint8_t {
  UNKNOWN,
  MID,
  FORWARD,
  BACKWARD,
};

enum class UwbTurn : uint8_t {
  UNKNOWN,
  CENTER,
  LEFT,
  RIGHT,
};

// Turns the sensor distances into direction, turn and turn ratio. The hub does
// not decide whether to move: the botbox+ move/stop checks, which depend on
// the speed level, are done by the drive board.
class UwbMoveMaker {
private:
  UwbDirection _prevDirection;
  UwbTurn      _prevTurn;
  UwbTurn      _prevBackTurn;
  // Cycles in a row in which the distances asked for a direct
  // FORWARD <-> BACKWARD change that has not been accepted yet.
  uint8_t      _directionFlipCount = 0;

  float _smoothedTurnRatio = 100.0f;
  float _smoothedTurnAngleDeg = 0.0f;
  UwbDirection _turnRatioDirection = UwbDirection::UNKNOWN;
  uint32_t _directionChangeTick = 0U;
  bool _isDirectionChangeHoldActive = false;

  uint8_t update_turn_ratio_from_angle(float rawAngleDeg, UwbDirection dir,
                                       bool allowAngleUpdate);

public:
  UwbMoveMaker();
  ~UwbMoveMaker();

  void reset();

  UwbTurn      calculate_turn(int16_t distanceLeft, int16_t distanceRight);
  UwbTurn      calculate_back_turn(int16_t distanceLeft, int16_t distanceRight);
  uint8_t      calculate_turn_ratio(int16_t distanceLeft, int16_t distanceRight,
                                    UwbDirection dir);
  uint8_t      calculate_turn_ratio(int16_t distanceLeft, int16_t distanceRight,
                                    int16_t distanceFront, int16_t distanceBack,
                                    UwbDirection dir,
                                    bool areSamplesSynchronized);
  UwbDirection calculate_direction(int16_t distanceFront, int16_t distanceBack);
  // True while calculate_direction() is holding the previous direction
  // because a direct FORWARD <-> BACKWARD change is not confirmed yet.
  bool         is_direction_flip_pending() const {
    return _directionFlipCount != 0U;
  }
  // Drops an unconfirmed FORWARD <-> BACKWARD change. Called for every cycle
  // that does not run calculate_direction(), so that only consecutive cycles
  // count towards the confirmation.
  void         cancel_direction_flip() { _directionFlipCount = 0; }
  int16_t      turn_angle_deg() const {
    return static_cast<int16_t>(_smoothedTurnAngleDeg);
  }
};
