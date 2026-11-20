#!/usr/bin/python3
"""Generate the fixed-address firmware configuration image from JSON."""

import argparse
import json
import struct
import sys
import zlib
from pathlib import Path


MAGIC = b"BXUH"
BINARY_FORMAT_VERSION = 1
SCHEMA_VERSION = 1
PDOA_OUTPUT_MODE_MAX = 2  # 0 legacy only, 1 both, 2 PDoA only
HEADER_FORMAT = "<4sHHII"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
FLASH_ADDRESS = 0x0807F000
MODE_BUFFER_CAPACITY = 16
# Period of the firmware task that fills the mode decision buffer
# (UWB_TASK_PERIOD_MS in firmware_configuration.hpp).
UWB_TASK_PERIOD_MS = 190
DIRECTION_FLIP_CONFIRM_MAX = 16
MAX_RECEIVE_TIMEOUT_MS = 300_000
MAX_MODE_TIMEOUT_MS = 60_000
MAX_FRONT_SENSOR_SPACING_MM = 3_000
MAX_FRONT_REAR_SENSOR_SPACING_MM = 10_000
MAX_TURN_THRESHOLD_MM = 1_000
MAX_DIRECTION_THRESHOLD_MM = 5_000
MAX_CLOSE_BACK_DISTANCE_MM = 10_000

UWB_FIELDS = (
    ("receive_timeout_ms", "I"),
    ("distance_filter_enabled", "I"),
    ("mode_decision_buffer_size", "I"),
    ("mode_decision_majority_count", "I"),
    ("mode_observation_timeout_ms", "I"),
    ("front_sensor_spacing_mm", "H"),
    ("front_rear_sensor_spacing_mm", "H"),
    ("center_in_threshold_lr_mm", "I"),
    ("center_out_threshold_lr_mm", "I"),
    ("center_back_in_threshold_lr_mm", "I"),
    ("center_back_out_threshold_lr_mm", "I"),
    ("stop_threshold_fb_mm", "I"),
    ("mid_threshold_fb_mm", "I"),
    ("turn_ratio_smoothing_alpha", "f"),
    ("turn_angle_deadzone_deg", "f"),
    ("turn_angle_full_scale_deg", "f"),
    ("turn_angle_smoothing_alpha", "f"),
    ("turn_angle_max_step_deg", "f"),
    ("direction_change_turn_hold_ms", "I"),
    ("max_sensor_sample_skew_ms", "I"),
    ("direction_flip_confirm_count", "I"),
    ("min_turn_ratio_forward_percent", "I"),
    ("min_turn_ratio_backward_percent", "I"),
    ("close_back_turn_limit_distance_mm", "I"),
    ("close_back_min_turn_ratio_percent", "I"),
    ("initial_turn_ratio_percent", "f"),
    ("back_uwb_enabled", "I"),
    ("vector_smoothing_alpha", "f"),
)

PDOA_FIELDS = (
    ("output_mode", "I"),
)

CONFIGURATION_GROUP_FIELDS = (
    ("uwb_follow", UWB_FIELDS),
    ("pdoa_follow", PDOA_FIELDS),
)

PAYLOAD_FORMAT = "<" + "".join(
    field_type
    for _, fields in CONFIGURATION_GROUP_FIELDS
    for _, field_type in fields
)
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)
IMAGE_SIZE = HEADER_SIZE + PAYLOAD_SIZE


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _validate_group_fields(group_name: str, configuration: dict, fields: tuple) -> None:
    _require(isinstance(configuration, dict), f"{group_name} object is required")
    expected = {name for name, _ in fields}
    actual = set(configuration)
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    _require(not missing, f"missing {group_name} fields: {', '.join(missing)}")
    _require(not extra, f"unknown {group_name} fields: {', '.join(extra)}")


def validate_uwb(configuration: dict) -> None:
    expected = {name for name, _ in UWB_FIELDS}
    actual = set(configuration)
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    _require(not missing, f"missing UWB fields: {', '.join(missing)}")
    _require(not extra, f"unknown UWB fields: {', '.join(extra)}")

    c = configuration
    _require(
        1 <= c["receive_timeout_ms"] <= MAX_RECEIVE_TIMEOUT_MS,
        f"receive_timeout_ms must be 1..{MAX_RECEIVE_TIMEOUT_MS}",
    )
    _require(
        isinstance(c["distance_filter_enabled"], bool)
        or c["distance_filter_enabled"] in (0, 1),
        "distance_filter_enabled must be true/false",
    )
    _require(
        1 <= c["mode_decision_buffer_size"] <= MODE_BUFFER_CAPACITY,
        f"mode_decision_buffer_size must be 1..{MODE_BUFFER_CAPACITY}",
    )
    _require(
        1
        <= c["mode_decision_majority_count"]
        <= c["mode_decision_buffer_size"],
        "mode_decision_majority_count must fit the decision buffer",
    )
    _require(
        1 <= c["mode_observation_timeout_ms"] <= MAX_MODE_TIMEOUT_MS,
        f"mode_observation_timeout_ms must be 1..{MAX_MODE_TIMEOUT_MS}",
    )
    # The firmware rejects the whole image when the timeout is shorter than the
    # time needed to fill the decision buffer once.
    minimum_observation_ms = (c["mode_decision_buffer_size"] - 1) * UWB_TASK_PERIOD_MS
    _require(
        c["mode_observation_timeout_ms"] >= minimum_observation_ms,
        "mode_observation_timeout_ms must be at least "
        f"(mode_decision_buffer_size - 1) * {UWB_TASK_PERIOD_MS} = "
        f"{minimum_observation_ms} ms",
    )
    _require(
        isinstance(c["front_sensor_spacing_mm"], int)
        and not isinstance(c["front_sensor_spacing_mm"], bool)
        and 1 <= c["front_sensor_spacing_mm"] <= MAX_FRONT_SENSOR_SPACING_MM,
        f"front_sensor_spacing_mm must be 1..{MAX_FRONT_SENSOR_SPACING_MM}",
    )
    _require(
        isinstance(c["front_rear_sensor_spacing_mm"], int)
        and not isinstance(c["front_rear_sensor_spacing_mm"], bool)
        and 1
        <= c["front_rear_sensor_spacing_mm"]
        <= MAX_FRONT_REAR_SENSOR_SPACING_MM,
        "front_rear_sensor_spacing_mm must be "
        f"1..{MAX_FRONT_REAR_SENSOR_SPACING_MM}",
    )

    _require(
        c["center_in_threshold_lr_mm"] <= c["center_out_threshold_lr_mm"],
        "front center-in threshold must not exceed center-out threshold",
    )
    _require(
        c["center_back_in_threshold_lr_mm"]
        <= c["center_back_out_threshold_lr_mm"],
        "back center-in threshold must not exceed center-out threshold",
    )
    _require(
        c["mid_threshold_fb_mm"] <= c["stop_threshold_fb_mm"],
        "mid_threshold_fb_mm must not exceed stop_threshold_fb_mm",
    )

    distance_fields = (
        "center_in_threshold_lr_mm",
        "center_out_threshold_lr_mm",
        "center_back_in_threshold_lr_mm",
        "center_back_out_threshold_lr_mm",
        "stop_threshold_fb_mm",
        "mid_threshold_fb_mm",
        "close_back_turn_limit_distance_mm",
    )
    for field in distance_fields:
        _require(0 <= c[field] <= 32767, f"{field} must be 0..32767")

    for field in (
        "center_in_threshold_lr_mm",
        "center_out_threshold_lr_mm",
        "center_back_in_threshold_lr_mm",
        "center_back_out_threshold_lr_mm",
    ):
        _require(
            c[field] <= MAX_TURN_THRESHOLD_MM,
            f"{field} must be 0..{MAX_TURN_THRESHOLD_MM}",
        )

    for field in ("stop_threshold_fb_mm", "mid_threshold_fb_mm"):
        _require(
            c[field] <= MAX_DIRECTION_THRESHOLD_MM,
            f"{field} must be 0..{MAX_DIRECTION_THRESHOLD_MM}",
        )

    _require(
        c["close_back_turn_limit_distance_mm"] <= MAX_CLOSE_BACK_DISTANCE_MM,
        "close_back_turn_limit_distance_mm must be "
        f"0..{MAX_CLOSE_BACK_DISTANCE_MM}",
    )
    _require(
        1 <= c["direction_flip_confirm_count"] <= DIRECTION_FLIP_CONFIRM_MAX,
        "direction_flip_confirm_count must be "
        f"1..{DIRECTION_FLIP_CONFIRM_MAX}",
    )
    _require(
        0 < float(c["turn_ratio_smoothing_alpha"]) <= 1,
        "turn_ratio_smoothing_alpha must be in (0, 1]",
    )
    _require(
        0 <= float(c["turn_angle_deadzone_deg"])
        < float(c["turn_angle_full_scale_deg"])
        <= 90,
        "turn angle thresholds must satisfy 0 <= deadzone < full scale <= 90",
    )
    _require(
        0 < float(c["turn_angle_smoothing_alpha"]) <= 1,
        "turn_angle_smoothing_alpha must be in (0, 1]",
    )
    _require(
        0 < float(c["turn_angle_max_step_deg"]) <= 90,
        "turn_angle_max_step_deg must be in (0, 90]",
    )
    _require(
        0 <= c["direction_change_turn_hold_ms"] <= 5000,
        "direction_change_turn_hold_ms must be 0..5000",
    )
    _require(
        0 < c["max_sensor_sample_skew_ms"] <= c["receive_timeout_ms"],
        "max_sensor_sample_skew_ms must fit the receive timeout",
    )
    for field in (
        "min_turn_ratio_forward_percent",
        "min_turn_ratio_backward_percent",
        "close_back_min_turn_ratio_percent",
        "initial_turn_ratio_percent",
    ):
        _require(0 <= float(c[field]) <= 100, f"{field} must be 0..100")
    _require(
        isinstance(c["back_uwb_enabled"], bool),
        "back_uwb_enabled must be a boolean",
    )
    _require(
        0 < float(c["vector_smoothing_alpha"]) <= 1,
        "vector_smoothing_alpha must be in (0, 1]",
    )


def validate_pdoa(configuration: dict) -> None:
    mode = configuration["output_mode"]
    _require(
        isinstance(mode, int)
        and not isinstance(mode, bool)
        and 0 <= mode <= PDOA_OUTPUT_MODE_MAX,
        f"output_mode must be an integer 0..{PDOA_OUTPUT_MODE_MAX}",
    )


def validate(document: dict) -> None:
    _require(isinstance(document, dict), "configuration document must be an object")
    _require(
        document.get("schema_version") == SCHEMA_VERSION,
        "unsupported schema_version",
    )
    expected_groups = {group_name for group_name, _ in CONFIGURATION_GROUP_FIELDS}
    actual_groups = set(document) - {"schema_version"}
    missing_groups = sorted(expected_groups - actual_groups)
    extra_groups = sorted(actual_groups - expected_groups)
    _require(not missing_groups, f"missing groups: {', '.join(missing_groups)}")
    _require(not extra_groups, f"unknown groups: {', '.join(extra_groups)}")

    for group_name, fields in CONFIGURATION_GROUP_FIELDS:
        _validate_group_fields(group_name, document.get(group_name), fields)

    validate_uwb(document["uwb_follow"])
    validate_pdoa(document["pdoa_follow"])


def build_image(document: dict) -> bytes:
    validate(document)

    values = [
        document[group_name][name]
        for group_name, fields in CONFIGURATION_GROUP_FIELDS
        for name, _ in fields
    ]
    payload = struct.pack(PAYLOAD_FORMAT, *values)
    payload_crc32 = zlib.crc32(payload) & 0xFFFFFFFF
    header = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        BINARY_FORMAT_VERSION,
        SCHEMA_VERSION,
        len(payload),
        payload_crc32,
    )
    return header + payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path, help="configuration JSON")
    parser.add_argument("output", type=Path, help="output binary image")
    args = parser.parse_args()

    try:
        document = json.loads(args.input.read_text(encoding="utf-8"))
        image = build_image(document)
        _require(len(image) == IMAGE_SIZE, "internal image size mismatch")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(image)
    except (OSError, ValueError, TypeError, KeyError, struct.error, json.JSONDecodeError) as error:
        print(f"Configuration generation failed: {error}", file=sys.stderr)
        return 1

    crc = struct.unpack_from("<I", image, 12)[0]
    print(f"Configuration image: {args.output}")
    print(f"Flash address:       0x{FLASH_ADDRESS:08X}")
    print(f"Image size:          {len(image)} bytes")
    print(f"Payload CRC32:       0x{crc:08X}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
