# UWB robot reference point

Status: decided 2026-11-17, **not implemented yet**. The current firmware
still reports from the front-sensor midpoint (see "Current state").

## Definition

`UWB_RESULT_A` (0x240) `distance` is specified as "the filtered distance from
the robot reference point to the tag" (UWB follow module interface
specification v0.11, p. 6). The coordinate frame is ROS REP-103 (x forward,
y left, bearing 0 = straight ahead, positive to the left).

The specification fixes the **byte layout** of 0x240 (unchanged since v0.6),
not the position of the robot reference point. That position is defined by
this hub:

**The robot reference point is the centroid of the three UWB sensors.**

In the sensor frame (origin at the midpoint of the left and right front
sensors, x forward, y left):

| Sensor | Position |
|---|---|
| Left | (0, +d/2) |
| Right | (0, −d/2) |
| Back | (−D, 0) |
| **Reference point (centroid)** | **(−D/3, 0)** |

`d` is `front_sensor_spacing_mm` and `D` is `front_rear_sensor_spacing_mm`
(default 1200 mm, so the reference point is 400 mm behind the front-sensor
midpoint).

Why: forward and backward following then measure from one point on the
vehicle. Only the hub needs to know the sensor geometry. The control board
compares the received distance with its thresholds and does no geometry. Today
it converts the front-midpoint distance to a back-sensor distance with its own
copy of `D`.

## Current state

- `App/manager/uwb/uwb_vector.hpp` / `.cpp`: the vector (x, y, distance,
  bearing) uses the front-sensor midpoint as origin.
- `App/uwb_output.cpp` `result_distance()`, `result_bearing()`, and the
  `UWB_RESULT_B` packer report from that origin. Without a vector they fall
  back to the front distance or the nearest sensor's own distance, and set
  DEGRADED.
- botbox-stm32 (control board) converts the received distance to a
  back-sensor distance with its own `front_rear_sensor_spacing_mm`
  (`Core/App/manager/uwb/uwb_external_follow.hpp` `back_equivalent_distance()`).

## Planned changes (hub)

1. Shift the vector to the reference point: `x_ref = x + D/3`, `y_ref = y`.
   Derive `distance = hypot(x_ref, y_ref)` and
   `bearing = atan2(y_ref, x_ref)` from the shifted position. Keep the
   smoothing in the sensor frame or in the reference frame, whichever is used
   consistently.
2. `UWB_RESULT_B` `tag_x` / `tag_y` use the same reference point, so that
   `tag_x = distance·cos(bearing)` still holds.
3. Fallbacks without a vector need an approximation to the reference point:
   - front pair only, tag assumed in front: compute normally (FRONT_ASSUMED).
   - one front sensor only, tag in front: `front + D/3`.
   - back sensor only, tag behind: `back + 2D/3`.

   Keep DEGRADED set for these. Choose by the reported direction, and document
   the rule next to `result_distance()`.
4. Publish the reference point so that third-party L1 receivers can tell what
   the distance means. Use this document and, optionally, a CFG parameter like
   the upstream delay (CFG 0x0005). If the offset becomes configurable, add it
   to the hub configuration schema (default D/3) and bump the schema version.
5. Bump the hub firmware version. The change alters 0x240 values without
   changing the layout.

## Compatibility

- The hub and the control board must be updated **together**. With only one
  side updated, follow distances are off by about D/3 = 0.4 m forward and
  2D/3 = 0.8 m backward (straight ahead or behind).
- The control board removes its spacing setting and back-distance conversion,
  and moves its distance thresholds to the new reference point. See
  botbox-stm32 `docs/uwb_reference_point_migration.md`.
- After both firmwares are tagged, the uploader re-syncs both contracts.
