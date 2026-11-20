# Vendored: UWB E2 Core (Intelligent UWB V3)

Do not edit the files in this directory. Replace them as a whole with a new
delivery and update this note.

| Item | Value |
|---|---|
| Delivered | 2026-11-20 by the UWB team, as `docs/uwb_core.tar` (SHA-256 `3b83caeaafc2727370dd5352a1e1a1b0d7fae4a7fe69886d96c62f5ed2173f79`) |
| Source | `uwb-visualization-e1`, branch `codex/uwb-embedded-e1`, path `embedded/uwb_core/` |
| Commits | algorithm `e6e07c1`, handoff `2e57528` |
| Profile | `UWB_V3_ONLY=1` (vehicle test; V2/HELD not computed) |
| Firmware build | `uwb_core.c`, `uwb_math.c`, `uwb_engines.c` with C11, `-ffp-contract=off -fno-fast-math` (set per source in the top-level `CMakeLists.txt`) |

The hub adapter is `App/pdoa/pdoa_pipeline.{hpp,cpp}`; the input contract is
`docs/IF0_BYTE_CONTRACT_20261120.md`, the integration rules are in
`INTEGRATION.md` here.
