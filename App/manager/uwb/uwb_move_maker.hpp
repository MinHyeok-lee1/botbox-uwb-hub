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

enum class UwbMove : uint8_t {
  STOP,
  MOVE,
};

class UwbMoveMaker {
private:
  uint8_t _moveBufferIndex = 0;
  UwbMove _moveBuffer[UWB_MOVE_BUFFER_CAPACITY];

  UwbDirection _prevDirection;
  UwbTurn      _prevTurn;
  UwbTurn      _prevBackTurn;
  UwbMove      _prevMove;

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
  UwbMove      calculate_move(UwbDirection dir, int16_t distanceFront,
                               int16_t distanceBack);
  UwbMove      calculate_move(UwbDirection dir, int16_t distanceFront,
                               int16_t distanceBack, int16_t distanceLeft,
                               int16_t distanceRight);
  UwbMove      calculate_front_only_move(int16_t distanceFront);
  UwbMove      calculate_front_only_move(int16_t distanceFront,
                                         int16_t distanceLeft,
                                         int16_t distanceRight);
  UwbMove      calculate_back_only_move(int16_t distanceBack);
  int16_t      turn_angle_deg() const {
    return static_cast<int16_t>(_smoothedTurnAngleDeg);
  }
};
