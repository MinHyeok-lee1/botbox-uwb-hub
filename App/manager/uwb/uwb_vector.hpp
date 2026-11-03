#pragma once

// Position of the tag as a distance and a signed bearing, for the CAN2 output.
//
// Frame of reference: origin at the midpoint of the left and right front
// sensors, x forward, y to the left. The left sensor is at (0, +d/2), the
// right one at (0, -d/2) and the back sensor at (-D, 0) on the centre line,
// with d = front_sensor_spacing_mm and D = front_rear_sensor_spacing_mm. The
// geometry is planar: a tag held higher or lower than the sensors lengthens
// all three distances.

#include <cstdint>

enum class UwbVectorQuality : uint8_t {
  NONE,          // no vector this cycle
  THREE_SENSOR,  // left, right and back: full -180..180 degree bearing
  FRONT_ASSUMED, // left and right only: the tag is assumed to be in front
};

// Bearing value sent when there is none.
static constexpr int16_t UWB_VECTOR_BEARING_NONE = 0x7FFF;

struct UwbVector {
  UwbVectorQuality quality;
  int16_t distanceMm;  // from the origin; -1 without a vector
  int16_t bearingDdeg; // 0.1 degree, -1800..1799, positive to the left
};

class UwbVectorEstimator {
public:
  void reset();

  // Computes this cycle's vector. `requested` is the best quality the current
  // follow mode and sample timing allow; the result is NONE when the
  // distances do not fit the sensor geometry. x and y are smoothed with
  // vector_smoothing_alpha while the quality stays the same.
  UwbVector update(UwbVectorQuality requested, int16_t distanceLeft,
                   int16_t distanceRight, int16_t distanceBack);

private:
  bool _hasEstimate = false;
  UwbVectorQuality _quality = UwbVectorQuality::NONE;
  float _x = 0.0f;
  float _y = 0.0f;
};
