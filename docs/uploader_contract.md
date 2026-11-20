# Uploader contract

The desktop uploader (`botbox-uploader`) flashes this firmware and writes its
flash configuration. It does not build from this repository: it vendors a
small set of files at a tag and identifies the board and the images by fixed
headers. This document lists what this repository promises the uploader and
how to change or release it.

Shared formats (the metadata and configuration headers, the manifest
structure) are owned by `botbox-stm32`: `docs/firmware_metadata.md`,
`docs/firmware_configuration.md`, `docs/schema_compatibility.md`. This
repository uses them with its own magics and never changes a shared layout on
its own.

## Contract files

The uploader copies these files at a tag (sparse fetch) and pins them with a
sha256 lock (`vendor/firmware-contracts/uwb-hub/` in the uploader).

| File | Content |
|---|---|
| `configuration/configuration.schema.json` | Current configuration schema (`schema_id` `botbox.uwb_hub.configuration`, magic `BXUH`) |
| `configuration/schemas/vN/configuration.schema.json` | Earlier schemas the uploader can still write (`v1`) |
| `compatibility/schema-compatibility.json` | Compatibility manifest (firmware ranges, uploader section) |
| `CMakeLists.txt` | Read only for `project(... VERSION X.Y.Z)` |

The paths are part of the contract. Moving them needs a matching uploader
change.

## Board identity

| Item | Value | Defined in |
|---|---|---|
| MCU | STM32G474RET6, device id `0x469`, 512 KiB | uploader board profile `uwb-hub` |
| Firmware region | `0x08000000`-`0x0807F000` (linker `FLASH` 508K) | `STM32G474xx_FLASH.ld` |
| Firmware metadata | 32 bytes at `0x08000200`, magic `BUFW` | `App/configuration/firmware_metadata.cpp` |
| Configuration | 16-byte header + 112-byte payload (schema 2; schema 1 was 108) at `0x0807F000` (linker `CONFIG` 4K), magic `BXUH` | `App/configuration/firmware_configuration.hpp` |
| Flash banks | DBANK=1 (dual bank, factory default) required by the uploader; DBANK=0 is refused | option bytes |

The build checks the embedded metadata against `project(VERSION)` and the
manifest (`cmake/verify_firmware_metadata.cmake`, "Verified firmware
metadata" in the build log). The uploader rejects images without `BUFW`
metadata and firmware below **v1.0.0** (no backward compatibility).

How the uploader handles this board:

- Firmware download erases only the pages the image covers, so the CONFIG
  page survives a firmware update. A valid image never reaches `0x0807F000`.
- The configuration image it builds is byte-identical to
  `tools/generate_configuration.py` output for the same values (checked by an
  uploader test against `configuration/default.json`).
- Schema 2 (2026-11-20) appends the `pdoa_follow` group (`output_mode`
  0 legacy only / 1 both / 2 PDoA only, default 0): payload 112 bytes, image
  128 bytes, binary format still 1. The firmware accepts a valid schema 1
  image (108-byte payload) and migrates it in memory (`pdoa_follow` defaulted,
  status "schema 1 migrated" on the console); it never rewrites the flash. The
  uploader may keep writing schema 1 (`schemas/v1/`) to firmware that reads it.
- It applies `mode_observation_timeout_ms >= (mode_decision_buffer_size - 1) x 190 ms`
  from schema 1 and 2, as this firmware does.
- Uploads were verified on hardware (2026-11-18, DBANK=1, CubeProgrammer
  2.23): full install, and a firmware-only update keeps CONFIG. The hub does
  not use DBANK=0, so the uploader refuses it rather than leave it untested.

## Changing the configuration

Keep these in sync, then run `python3 tools/check_configuration_schema.py`:

1. `FirmwareConfigurationPayload` and `DEFAULT_CONFIGURATION`
   (`App/configuration/firmware_configuration.{hpp,cpp}`).
2. `tools/generate_configuration.py`, `configuration/default.json`.
3. `configuration/configuration.schema.json`.
4. `compatibility/schema-compatibility.json`.
5. `tools/configure_configuration.py` (uses the generator tables) and, for the
   multi-group layout, `STRUCT_GROUPS` in `tools/check_configuration_schema.py`.

When the payload layout changes, bump `configuration_schema_version` (and the
binary format only if the header changes). Copy the previous schema to
`configuration/schemas/v<old>/configuration.schema.json` if the uploader must
still write it, and update both version ranges in the manifest. The rules are
the same as for the control board (`botbox-stm32` `docs/schema_compatibility.md`).
Widening the firmware minimum below the current schema needs a decoder in
`firmware_configuration.cpp` and a raised
`OLDEST_DECODABLE_CONFIGURATION_SCHEMA_VERSION` (a `static_assert` enforces it);
`tests/host/run_configuration_tests.sh` covers the schema 1 migration. The
firmware version (`project(VERSION)`) is bumped for a release, not by the
schema change itself; the uploader reads the schema ranges from the manifest.

## Tags

| Tag | Use |
|---|---|
| `hub-vX.Y.Z` | Release. Required by the uploader's release build. |
| `hub-vX.Y.Z-<suffix>` (for example `-dev.1`) | Development pin. Uploader development builds only. |

- `X.Y.Z` must equal `project(... VERSION X.Y.Z)` in `CMakeLists.txt` at the
  tagged commit (the suffix is ignored). The uploader refuses a mismatch.
- Tags never move. To fix one, create a new tag (`-dev.2`, next version).
- The uploader reads tags, not branches. A development tag may sit on a
  feature branch; pushing the tag pushes its commit. Put release tags on
  `main`, so a later rebase cannot leave them outside the main history.
- Delete development tags (local and remote) after the matching release.
- Only tags are pushed for the uploader; there is no release-asset workflow.

## Release checklist

1. `project(VERSION)` bumped; manifest and schema consistent
   (`tools/check_configuration_schema.py`).
2. Release build passes, including "Verified firmware metadata".
3. Host tests pass (`tests/host/run_if5_tests.sh`).
4. Annotated tag `hub-vX.Y.Z` on `main`, pushed.
5. In `botbox-uploader`: `npm run contract:sync -- --board uwb-hub --ref hub-vX.Y.Z`,
   `npm test`, commit the vendored files and lock.

## Current state (2026-11-18)

- Firmware v1.0.0 (`7c10e88` on `feat/if5-sys-node`, not on `main` yet).
- The uploader is pinned to `hub-v1.0.0-dev.1` (`c3306b2`). `hub-v0.1.0-dev.1`
  (`14627cb`) is no longer used; delete both after the `hub-v1.0.0` release.
- Hub uploads verified on a DBANK=1 board (2026-11-18). DBANK=0 is refused by
  the uploader.
