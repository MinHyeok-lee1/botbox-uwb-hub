#!/usr/bin/python3
"""Inspect and edit firmware configuration JSON files."""

import argparse
import json
import struct
import sys
from pathlib import Path

sys.dont_write_bytecode = True

from generate_configuration import (  # noqa: E402
    CONFIGURATION_GROUP_FIELDS,
    FLASH_ADDRESS,
    build_image,
    validate,
)


DEFAULT_CONFIG_JSON = Path("configuration/default.json")
DEFAULT_OUTPUT_BIN = Path("build/configuration/firmware-configuration-v1.bin")
FIELD_TYPES = {
    f"{group_name}.{field_name}": field_type
    for group_name, fields in CONFIGURATION_GROUP_FIELDS
    for field_name, field_type in fields
}


def load_document(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def save_document(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")


def require_configuration(document: dict, group_name: str) -> dict:
    configuration = document.get(group_name)
    if not isinstance(configuration, dict):
        raise ValueError(f"{group_name} object is required")
    return configuration


def resolve_field_path(field: str) -> tuple[str, str, str]:
    if field in FIELD_TYPES:
        path = field
    else:
        matches = [path for path in FIELD_TYPES if path.endswith(f".{field}")]
        if len(matches) != 1:
            raise ValueError(f"unknown or ambiguous field: {field}")
        path = matches[0]
    group_name, field_name = path.split(".", 1)
    return path, group_name, field_name


BOOLEAN_FIELDS = ("distance_filter_enabled", "back_uwb_enabled")


def parse_field_value(field: str, value: str):
    path, _, field_name = resolve_field_path(field)

    if field_name in BOOLEAN_FIELDS:
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off"):
            return False
        raise ValueError(f"{field_name} must be true/false")

    if FIELD_TYPES[path] == "f":
        return float(value)

    return int(value, 0)


def command_list(args: argparse.Namespace) -> int:
    document = load_document(args.config_json)
    validate(document)

    for group_name, fields in CONFIGURATION_GROUP_FIELDS:
        configuration = require_configuration(document, group_name)
        for field, _ in fields:
            print(f"{group_name}.{field}={configuration[field]}")
    return 0


def command_get(args: argparse.Namespace) -> int:
    document = load_document(args.config_json)
    validate(document)

    _, group_name, field_name = resolve_field_path(args.field)
    configuration = require_configuration(document, group_name)
    print(configuration[field_name])
    return 0


def command_set(args: argparse.Namespace) -> int:
    document = load_document(args.config_json)
    _, group_name, field_name = resolve_field_path(args.field)
    configuration = require_configuration(document, group_name)
    configuration[field_name] = parse_field_value(args.field, args.value)
    validate(document)

    output = args.output if args.output is not None else args.config_json
    save_document(output, document)
    print(f"Updated {args.field} in {output}")
    return 0


def command_validate(args: argparse.Namespace) -> int:
    document = load_document(args.config_json)
    build_image(document)
    print(f"Valid configuration: {args.config_json}")
    return 0


def command_generate(args: argparse.Namespace) -> int:
    document = load_document(args.config_json)
    image = build_image(document)
    args.output_bin.parent.mkdir(parents=True, exist_ok=True)
    args.output_bin.write_bytes(image)

    crc = struct.unpack_from("<I", image, 12)[0]
    print(f"Configuration image: {args.output_bin}")
    print(f"Flash address:       0x{FLASH_ADDRESS:08X}")
    print(f"Image size:          {len(image)} bytes")
    print(f"Payload CRC32:       0x{crc:08X}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list")
    list_parser.add_argument("config_json", nargs="?", type=Path, default=DEFAULT_CONFIG_JSON)
    list_parser.set_defaults(handler=command_list)

    get_parser = subparsers.add_parser("get")
    get_parser.add_argument("field")
    get_parser.add_argument("config_json", nargs="?", type=Path, default=DEFAULT_CONFIG_JSON)
    get_parser.set_defaults(handler=command_get)

    set_parser = subparsers.add_parser("set")
    set_parser.add_argument("field")
    set_parser.add_argument("value")
    set_parser.add_argument("config_json", nargs="?", type=Path, default=DEFAULT_CONFIG_JSON)
    set_parser.add_argument("-o", "--output", type=Path)
    set_parser.set_defaults(handler=command_set)

    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("config_json", nargs="?", type=Path, default=DEFAULT_CONFIG_JSON)
    validate_parser.set_defaults(handler=command_validate)

    generate_parser = subparsers.add_parser("generate")
    generate_parser.add_argument("config_json", nargs="?", type=Path, default=DEFAULT_CONFIG_JSON)
    generate_parser.add_argument("output_bin", nargs="?", type=Path, default=DEFAULT_OUTPUT_BIN)
    generate_parser.set_defaults(handler=command_generate)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        return args.handler(args)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
        print(f"Configuration command failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
