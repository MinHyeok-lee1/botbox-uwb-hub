# UWB_RESULT_B (0x241) output

UWB_RESULT_B carries the tag position (`tag_x`, `tag_y`, mm) with the same
`seq` as the UWB_RESULT_A (`0x240`) of the same publication. Interface
specification v0.11 makes it optional and off by default, selectable by the
CFG `0x0003` output mask. The specification does not define the mask bit
order yet (the hub answers CFG READ `0x0003` with UNSUPPORTED), so for now a
build option selects it.

## Build option

| Option | Default | Effect |
| --- | --- | --- |
| `UWB_HUB_RESULT_B_ENABLE` | `OFF` | `ON` sends `0x241` on CAN2 right after each `0x240`. |

CMake prints `UWB_RESULT_B (0x241) output: ON|OFF` at configure time. The
value stays in the build directory's CMake cache.

```sh
# Build and flash every connected ST-LINK with 0x241 on (build/Release-ResultB)
scripts/flash_all_result_b.sh

# Default image, 0x241 off (build/Release)
scripts/flash_all.sh
```

Both scripts write only the firmware pages; the CONFIG page (`0x0807F000`)
is kept. Do not mass-erase or change option bytes.

## Why it is on for the Jetson bridge

The Jetson recording bridge (botbox-stm32, IF-4 UART4) relays `0x240` and
`0x241` unchanged from the control board's CAN3. The Jetson team asked for
`0x241` on 2026-11-17 (team agreement, beyond v0.11). See botbox-stm32
`docs/if4_uart4_recording_protocol_draft.md`, "Navigation state and UWB
relay".

## Notes

- Publication is skipped while frames still wait on CAN2 (no receiver);
  with `0x241` on, one publication queues two frames (CAN2 TX queue is 16).
- `tag_x`/`tag_y` use the current reference point (front sensor midpoint).
  They change when the reference moves to the three-sensor centroid
  (`docs/uwb_reference_point.md`); tell the Jetson team before that release.
- Replace the build option with the CFG `0x0003` mask once the specification
  fixes its bit order.
