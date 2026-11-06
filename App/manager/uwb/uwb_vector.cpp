#include "uwb_vector.hpp"

#include <algorithm>
#include <cmath>

#include "uwb_follow_config.hpp"

namespace {

// How far the distances may break the sensor geometry, from range noise,
// before the sample is rejected instead of clamped.
constexpr float GEOMETRY_TOLERANCE_MM = 100.0f;
constexpr float RAD_TO_DDEG = 572.957795131f;
constexpr int16_t DISTANCE_MAX_MM = 32767;
// 0x7FFF marks an invalid coordinate in UWB_RESULT_B.
constexpr int16_t COORDINATE_MAX_MM = 32766;

const UwbVector NO_VECTOR = {UwbVectorQuality::NONE, -1,
                             UWB_VECTOR_BEARING_NONE, 0, 0, 0};

int16_t clamp_mm(float value) {
  const float limited = std::max(-static_cast<float>(COORDINATE_MAX_MM),
                                 std::min(static_cast<float>(COORDINATE_MAX_MM),
                                          value));
  return static_cast<int16_t>(std::lround(limited));
}

} // namespace

void UwbVectorEstimator::reset() {
  _hasEstimate = false;
  _quality = UwbVectorQuality::NONE;
  _x = 0.0f;
  _y = 0.0f;
}

UwbVector UwbVectorEstimator::update(UwbVectorQuality requested,
                                     int16_t distanceLeft,
                                     int16_t distanceRight,
                                     int16_t distanceBack) {
  const auto &configuration = uwb_follow_configuration();

  if (requested == UwbVectorQuality::NONE || distanceLeft < 0 ||
      distanceRight < 0 ||
      (requested == UwbVectorQuality::THREE_SENSOR && distanceBack < 0)) {
    reset();
    return NO_VECTOR;
  }

  const float d = static_cast<float>(configuration.front_sensor_spacing_mm);
  const float l = static_cast<float>(distanceLeft);
  const float r = static_cast<float>(distanceRight);

  // Two sensors d apart cannot differ by more than d.
  if (std::fabs(l - r) > d + GEOMETRY_TOLERANCE_MM) {
    reset();
    return NO_VECTOR;
  }

  // Squared distance from the origin (median of the left/right triangle) and
  // the lateral position, both exact for the planar geometry.
  const float originSquared = (l * l + r * r) / 2.0f - (d * d) / 4.0f;
  if (originSquared <= 0.0f) {
    reset();
    return NO_VECTOR;
  }
  const float y = (r * r - l * l) / (2.0f * d);

  float x = 0.0f;
  float residual = 0.0f;
  if (requested == UwbVectorQuality::THREE_SENSOR) {
    // From B^2 = (x + D)^2 + y^2 and originSquared = x^2 + y^2.
    const float rearSpacing =
        static_cast<float>(configuration.front_rear_sensor_spacing_mm);
    const float b = static_cast<float>(distanceBack);
    x = (b * b - originSquared - rearSpacing * rearSpacing) /
        (2.0f * rearSpacing);
    // Three distances over-determine the two coordinates: the front pair
    // fixes the origin distance and the back sensor fixes x. When they
    // disagree the distances do not belong to one tag position.
    residual = std::fabs(std::hypot(x, y) - std::sqrt(originSquared));
    if (residual > static_cast<float>(UWB_VECTOR_RESIDUAL_MAX_MM)) {
      reset();
      return NO_VECTOR;
    }
  } else {
    const float xSquared = originSquared - y * y;
    const float tolerance =
        GEOMETRY_TOLERANCE_MM * (std::sqrt(originSquared) + GEOMETRY_TOLERANCE_MM);
    if (xSquared < -tolerance) {
      reset();
      return NO_VECTOR;
    }
    x = std::sqrt(std::max(0.0f, xSquared));
  }

  if (!_hasEstimate || _quality != requested) {
    _x = x;
    _y = y;
    _hasEstimate = true;
    _quality = requested;
  } else {
    const float alpha = configuration.vector_smoothing_alpha;
    _x += alpha * (x - _x);
    _y += alpha * (y - _y);
  }

  const float distance = std::min(std::hypot(_x, _y),
                                  static_cast<float>(DISTANCE_MAX_MM));
  int32_t bearing =
      static_cast<int32_t>(std::lround(std::atan2(_y, _x) * RAD_TO_DDEG));
  if (bearing >= 1800) {
    bearing -= 3600;
  }
  return {requested, static_cast<int16_t>(std::lround(distance)),
          static_cast<int16_t>(bearing), clamp_mm(_x), clamp_mm(_y),
          static_cast<uint16_t>(std::lround(residual))};
}
