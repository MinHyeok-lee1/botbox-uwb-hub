#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

struct UwbDirectionAngle {
  float horizontal;
  float forward;
  float angleDeg;
};

inline float uwb_clamp_unit(float value) {
  return std::max(-1.0f, std::min(1.0f, value));
}

inline UwbDirectionAngle calculate_uwb_direction_angle(
    int16_t distanceLeft, int16_t distanceRight, int16_t distanceFront,
    int16_t distanceBack, float frontSensorSpacingMm,
    float frontRearSensorSpacingMm, bool backward) {
  constexpr float RAD_TO_DEG = 57.2957795131f;

  // Range differences divided by the matching anchor baseline provide
  // bounded left/right and front/back direction components. This is used as
  // a robust bearing estimate, not as an exact tag position.
  const float horizontal = uwb_clamp_unit(
      (static_cast<float>(distanceLeft) -
       static_cast<float>(distanceRight)) /
      frontSensorSpacingMm);
  const float forward = uwb_clamp_unit(
      (static_cast<float>(distanceBack) -
       static_cast<float>(distanceFront)) /
      frontRearSensorSpacingMm);
  const float longitudinal =
      std::max(0.0f, backward ? -forward : forward);
  const float angleDeg =
      std::atan2(std::abs(horizontal), longitudinal) * RAD_TO_DEG;

  return {horizontal, forward, angleDeg};
}

inline float calculate_front_only_uwb_angle(
    int16_t distanceLeft, int16_t distanceRight,
    float frontSensorSpacingMm) {
  constexpr float RAD_TO_DEG = 57.2957795131f;
  const float normalizedDifference = std::min(
      1.0f, std::abs(static_cast<float>(distanceLeft) -
                     static_cast<float>(distanceRight)) /
                frontSensorSpacingMm);
  return std::asin(normalizedDifference) * RAD_TO_DEG;
}

inline uint8_t calculate_uwb_turn_ratio_target(
    float angleDeg, float deadzoneDeg, float fullScaleDeg,
    uint32_t minimumRatioPercent) {
  float normalizedTurn = 0.0f;
  if (angleDeg > deadzoneDeg) {
    normalizedTurn =
        (angleDeg - deadzoneDeg) / (fullScaleDeg - deadzoneDeg);
    normalizedTurn = std::max(0.0f, std::min(1.0f, normalizedTurn));
  }

  const float availableReduction =
      100.0f - static_cast<float>(minimumRatioPercent);
  return static_cast<uint8_t>(
      100.0f - (normalizedTurn * availableReduction));
}
