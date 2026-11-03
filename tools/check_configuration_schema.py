#!/usr/bin/python3
"""Check that firmware configuration field names stay in sync."""

import ast
import json
import re
import struct
import sys
import zlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER_PATH = ROOT / "App/configuration/firmware_configuration.hpp"
SOURCE_PATH = ROOT / "App/configuration/firmware_configuration.cpp"
GENERATOR_PATH = ROOT / "tools/generate_configuration.py"
DEFAULT_JSON_PATH = ROOT / "configuration/default.json"
FORM_SCHEMA_PATH = ROOT / "configuration/configuration.schema.json"
COMPATIBILITY_MANIFEST_PATH = ROOT / "compatibility/schema-compatibility.json"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


STRUCT_GROUPS = (
    ("uwb_follow", "UwbFollowConfiguration", "UWB_FIELDS"),
    ("can_output", "CanOutputConfiguration", "CAN_OUTPUT_FIELDS"),
)


def extract_struct_fields(header_text: str, struct_name: str) -> list[str]:
    match = re.search(
        rf"struct\s+{struct_name}\s*\{{(?P<body>.*?)\}};",
        header_text,
        re.DOTALL,
    )
    if match is None:
        raise ValueError(f"{struct_name} struct not found")

    fields: list[str] = []
    for line in match.group("body").splitlines():
        field_match = re.search(
            r"\b(?:uint16_t|uint32_t|float)\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*;",
            line,
        )
        if field_match:
            fields.append(field_match.group(1))
    return fields


def extract_header_layout(header_text: str) -> list[tuple[str, str]]:
    match = re.search(
        r"struct\s+FirmwareConfigurationHeader\s*\{(?P<body>.*?)\};",
        header_text,
        re.DOTALL,
    )
    if match is None:
        raise ValueError("FirmwareConfigurationHeader struct not found")
    return re.findall(r"\b(uint(?:16|32)_t)\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*;", match.group("body"))


def extract_cpp_constant(header_text: str, name: str) -> int:
    match = re.search(rf"\b{name}\s*=\s*(0x[0-9A-Fa-f]+|\d+)U?\s*;", header_text)
    if match is None:
        raise ValueError(f"{name} not found")
    return int(match.group(1), 0)


def extract_python_constant(generator_text: str, name: str):
    tree = ast.parse(generator_text)
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise ValueError(f"{name} not found")


def extract_generator_fields(generator_text: str) -> list[str]:
    return [
        field_name
        for _, _, variable_name in STRUCT_GROUPS
        for field_name, _ in extract_generator_layout(generator_text, variable_name)
    ]


def extract_generator_layout(
    generator_text: str, variable_name: str = "UWB_FIELDS"
) -> list[tuple[str, str]]:
    tree = ast.parse(generator_text)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == variable_name:
                    value = ast.literal_eval(node.value)
                    return list(value)
    raise ValueError(f"{variable_name} not found")


def extract_json_fields(json_text: str) -> list[str]:
    document = json.loads(json_text)
    fields: list[str] = []
    for group_name, _, _ in STRUCT_GROUPS:
        configuration = document.get(group_name)
        if not isinstance(configuration, dict):
            raise ValueError(f"{group_name} object not found")
        fields.extend(configuration.keys())
    return fields


def extract_form_schema_fields(schema_text: str) -> list[str]:
    document = json.loads(schema_text)
    groups = document.get("groups")
    if not isinstance(groups, list):
        raise ValueError("groups array not found in configuration.schema.json")

    fields: list[str] = []
    for group_name, _, _ in STRUCT_GROUPS:
        group = next(
            (
                item
                for item in groups
                if isinstance(item, dict) and item.get("key") == group_name
            ),
            None,
        )
        if group is None or not isinstance(group.get("fields"), list):
            raise ValueError(
                f"{group_name} fields not found in configuration.schema.json"
            )
        for field in group["fields"]:
            if not isinstance(field, dict) or not isinstance(field.get("key"), str):
                raise ValueError("invalid field entry in configuration.schema.json")
            fields.append(field["key"])
    return fields


def validate_form_schema(
    schema_text: str,
    default_json_text: str,
    generator_text: str,
) -> list[str]:
    schema = json.loads(schema_text)
    defaults = json.loads(default_json_text)
    errors: list[str] = []

    if schema.get("configuration_schema_version") != defaults.get("schema_version"):
        errors.append("configuration.schema.json version does not match default.json")

    groups = schema.get("groups", [])
    generator_storage_types = {}
    field_paths: set[str] = set()

    for group_name, _, variable_name in STRUCT_GROUPS:
        group = next((item for item in groups if item.get("key") == group_name), None)
        if group is None:
            errors.append(f"configuration.schema.json group is missing: {group_name}")
            continue

        default_values = defaults.get(group_name, {})
        generator_storage_types.update(
            {
                f"{group_name}.{key}": {
                    "H": "uint16",
                    "I": "uint32",
                    "f": "float32",
                }.get(
                    format_code
                )
                for key, format_code in extract_generator_layout(
                    generator_text, variable_name
                )
            }
        )

        for field in group["fields"]:
            key = field["key"]
            path = f"{group_name}.{key}"
            field_paths.add(path)
            value = field.get("default")
            expected = default_values.get(key)
            if value != expected:
                errors.append(
                    f"configuration.schema.json default mismatch for {path}: "
                    f"expected {expected!r}, got {value!r}"
                )

            field_type = field.get("type")
            valid_type = (
                (field_type == "boolean" and isinstance(value, bool))
                or (
                    field_type == "integer"
                    and isinstance(value, int)
                    and not isinstance(value, bool)
                )
                or (
                    field_type == "number"
                    and isinstance(value, (int, float))
                    and not isinstance(value, bool)
                )
            )
            if not valid_type:
                errors.append(f"configuration.schema.json type mismatch for {path}")

            expected_storage_type = generator_storage_types.get(path)
            if field.get("storage_type") != expected_storage_type:
                errors.append(
                    f"configuration.schema.json storage_type mismatch for {path}: "
                    f"expected {expected_storage_type}, got {field.get('storage_type')}"
                )

            if field.get("required") is not True:
                errors.append(
                    f"configuration.schema.json required must be true for {path}"
                )
            if not isinstance(field.get("editable"), bool):
                errors.append(
                    f"configuration.schema.json editable must be boolean for {path}"
                )
            if not isinstance(field.get("label"), str) or not field["label"]:
                errors.append(f"configuration.schema.json label is missing for {path}")
            if (
                not isinstance(field.get("user_label"), str)
                or not field["user_label"]
            ):
                errors.append(
                    f"configuration.schema.json user_label is missing for {path}"
                )
            if (
                not isinstance(field.get("description"), str)
                or not field["description"]
            ):
                errors.append(
                    f"configuration.schema.json description is missing for {path}"
                )
            if (
                not isinstance(field.get("user_description"), str)
                or not field["user_description"]
            ):
                errors.append(
                    f"configuration.schema.json user_description is missing for {path}"
                )

            if field_type != "boolean":
                lower_bound_count = sum(
                    key in field for key in ("minimum", "exclusive_minimum")
                )
                if lower_bound_count != 1:
                    errors.append(
                        "configuration.schema.json numeric field must define exactly "
                        f"one lower bound for {path}"
                    )
                if "maximum" not in field:
                    errors.append(
                        f"configuration.schema.json maximum is missing for {path}"
                    )

            if "minimum" in field and value < field["minimum"]:
                errors.append(
                    f"configuration.schema.json default is below minimum for {path}"
                )
            if "exclusive_minimum" in field and value <= field["exclusive_minimum"]:
                errors.append(
                    f"configuration.schema.json default is below exclusive_minimum for {path}"
                )
            if "maximum" in field and value > field["maximum"]:
                errors.append(
                    f"configuration.schema.json default is above maximum for {path}"
                )

    for index, rule in enumerate(schema.get("rules", [])):
        if not isinstance(rule.get("user_message"), str) or not rule["user_message"]:
            errors.append(
                f"configuration.schema.json user_message is missing for rule {index}"
            )
        referenced_paths = []
        if "left" in rule:
            referenced_paths.append(rule["left"])
        if "right" in rule:
            referenced_paths.append(rule["right"])
        referenced_paths.extend(rule.get("operands", []))
        for path in referenced_paths:
            if path not in field_paths:
                errors.append(
                    f"configuration.schema.json rule {index} references unknown field: {path}"
                )

    return errors


def validate_binary_format(
    schema_text: str,
    header_text: str,
    generator_text: str,
) -> list[str]:
    schema = json.loads(schema_text)
    binary_format = schema["binary_format"]
    errors: list[str] = []
    magic = binary_format.get("magic")
    expected_header = [
        ("uint32_t", "magic"),
        ("uint16_t", "binary_format_version"),
        ("uint16_t", "configuration_schema_version"),
        ("uint32_t", "payload_length"),
        ("uint32_t", "payload_checksum"),
    ]

    if magic != "BXUH" or len(magic.encode("ascii")) != 4:
        errors.append("configuration.schema.json magic must be four ASCII bytes BXUH")
    if binary_format["header"].get("size_bytes") != 16:
        errors.append("configuration.schema.json header size must be 16")
    if extract_header_layout(header_text) != expected_header:
        errors.append("firmware configuration header layout does not match BXUH schema")

    firmware_magic = extract_cpp_constant(header_text, "CONFIGURATION_MAGIC")
    if firmware_magic.to_bytes(4, "little").decode("ascii") != magic:
        errors.append("firmware CONFIGURATION_MAGIC does not match schema magic")
    if extract_python_constant(generator_text, "MAGIC") != magic.encode("ascii"):
        errors.append("generator MAGIC does not match schema magic")
    if extract_python_constant(generator_text, "HEADER_FORMAT") != "<4sHHII":
        errors.append("generator HEADER_FORMAT does not match the 16-byte BXUH header")

    binary_version = binary_format["version"]
    schema_version = schema["configuration_schema_version"]
    compatibility = json.loads(read(COMPATIBILITY_MANIFEST_PATH))
    firmware_configuration = compatibility["firmware"]["configuration"]
    if firmware_configuration["binary_format"]["current"] != binary_version:
        errors.append("firmware binary format version does not match schema")
    if firmware_configuration["schema"]["current"] != schema_version:
        errors.append("firmware configuration schema version does not match schema")
    if not re.search(
        r"CONFIGURATION_BINARY_FORMAT_VERSION\s*=\s*"
        r"botbox::compatibility::CURRENT_CONFIGURATION_BINARY_FORMAT_VERSION\s*;",
        header_text,
    ):
        errors.append("firmware binary format version does not use generated compatibility")
    if not re.search(
        r"CONFIGURATION_SCHEMA_VERSION\s*=\s*"
        r"botbox::compatibility::CURRENT_CONFIGURATION_SCHEMA_VERSION\s*;",
        header_text,
    ):
        errors.append("firmware schema version does not use generated compatibility")
    if extract_python_constant(generator_text, "BINARY_FORMAT_VERSION") != binary_version:
        errors.append("generator binary format version does not match schema")
    if extract_python_constant(generator_text, "SCHEMA_VERSION") != schema_version:
        errors.append("generator configuration schema version does not match schema")
    if extract_python_constant(generator_text, "UWB_TASK_PERIOD_MS") != extract_cpp_constant(
        header_text, "UWB_TASK_PERIOD_MS"
    ):
        errors.append("generator UWB_TASK_PERIOD_MS does not match the firmware")

    generator_layout = [
        field
        for _, _, variable_name in STRUCT_GROUPS
        for field in extract_generator_layout(generator_text, variable_name)
    ]
    payload_format = "<" + "".join(format_code for _, format_code in generator_layout)
    schema_payload_size = sum(
        {"uint16": 2, "uint32": 4, "float32": 4}[field["storage_type"]]
        for group in schema["groups"]
        for field in group["fields"]
    )
    if struct.calcsize(payload_format) != schema_payload_size:
        errors.append(
            "generator payload size does not match configuration.schema.json"
        )
    expected_crc = int(binary_format["checksum"]["validation"]["expected"], 16)
    if zlib.crc32(b"123456789") & 0xFFFFFFFF != expected_crc:
        errors.append("CRC-32/ISO-HDLC validation vector does not match")
    return errors


def extract_default_initializer_comments(source_text: str) -> list[str]:
    match = re.search(
        r"const\s+FirmwareConfigurationPayload\s+DEFAULT_CONFIGURATION\s*=\s*\{(?P<body>.*?)\}\};",
        source_text,
        re.DOTALL,
    )
    if match is None:
        raise ValueError("DEFAULT_CONFIGURATION initializer not found")

    fields: list[str] = []
    for line in match.group("body").splitlines():
        comment_match = re.search(r"//\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*$", line)
        if comment_match:
            fields.append(comment_match.group(1))
    return fields


def compare(reference_name: str, reference: list[str], target_name: str, target: list[str]) -> list[str]:
    errors: list[str] = []
    if reference == target:
        return errors

    reference_set = set(reference)
    target_set = set(target)
    missing = sorted(reference_set - target_set)
    extra = sorted(target_set - reference_set)
    if missing:
        errors.append(f"{target_name} missing fields from {reference_name}: {', '.join(missing)}")
    if extra:
        errors.append(f"{target_name} has extra fields not in {reference_name}: {', '.join(extra)}")

    for index, (left, right) in enumerate(zip(reference, target)):
        if left != right:
            errors.append(
                f"{target_name} order mismatch at index {index}: expected {left}, got {right}"
            )
            break
    if len(reference) != len(target):
        errors.append(
            f"{target_name} field count mismatch: expected {len(reference)}, got {len(target)}"
        )
    return errors


def main() -> int:
    try:
        generator_text = read(GENERATOR_PATH)
        header_text = read(HEADER_PATH)
        form_schema_text = read(FORM_SCHEMA_PATH)
        default_json_text = read(DEFAULT_JSON_PATH)
        schema = {
            "firmware structs": [
                field
                for _, struct_name, _ in STRUCT_GROUPS
                for field in extract_struct_fields(header_text, struct_name)
            ],
            "generator field layouts": extract_generator_fields(generator_text),
            "default.json": extract_json_fields(default_json_text),
            "configuration.schema.json": extract_form_schema_fields(form_schema_text),
            "firmware default comments": extract_default_initializer_comments(read(SOURCE_PATH)),
        }
    except (OSError, ValueError, SyntaxError, json.JSONDecodeError) as error:
        print(f"Configuration schema check failed: {error}", file=sys.stderr)
        return 1

    reference_name = "firmware structs"
    reference = schema[reference_name]
    errors: list[str] = []
    for target_name, target in schema.items():
        if target_name == reference_name:
            continue
        errors.extend(compare(reference_name, reference, target_name, target))
    errors.extend(validate_form_schema(form_schema_text, default_json_text, generator_text))
    errors.extend(validate_binary_format(form_schema_text, header_text, generator_text))

    if errors:
        print("Configuration schema mismatch:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print(f"Configuration schema OK: {len(reference)} payload fields match exactly.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
