# Subtask 1 – Printer Setting & Persistence

## Goal
Introduce a printer-level `klipper_cruise_ratio` setting (default 0.50) that is persisted in presets/3MFs, exposed under **Printer Settings → Motion ability → Advanced**, and synchronized with Klipper via `SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO`.

## Work Items
1. **Config Definition**
   - Extend `MachineEnvelopeConfig` / `PrintConfig` (`src/libslic3r/PrintConfig.{hpp,cpp}`) with a `ConfigOptionFloat klipper_cruise_ratio`.
   - Label/tooltip per spec (“Klipper cruise ratio”, allowed 0.01–0.99). Mark as `comAdvanced`, default `0.5`.
   - Ensure `FullPrintConfig::defaults()` picks up the new option so CLI/3MF serialization continue to work without extra code.

2. **UI Wiring**
   - In `TabPrinter::build_kinematics_page()` (`src/slic3r/GUI/Tab.cpp`), add the new option to the Motion ability page (same “Advanced” optgroup that already holds `emit_machine_limits_to_gcode`).
   - Respect dual-column layout (Normal/Silent) when silent mode is enabled; the option should read/write both columns just like other scalar machine limits.

3. **G-code Emission**
   - Update `GCodeWriter` (`src/libslic3r/GCodeWriter.cpp`) so that when `gcode_flavor == gcfKlipper` it emits `SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=<value>` alongside existing `ACCEL`/`SQUARE_CORNER_VELOCITY` fields.
   - Make sure the command is sent once per print (e.g., together with other velocity limit setup) and respects per-mode overrides if those are ever added.

4. **Runtime Parsing**
   - Extend `GCodeProcessor::process_SET_VELOCITY_LIMIT()` to capture `MINIMUM_CRUISE_RATIO=` tokens and update a new field on `m_time_processor.machine_limits`.
   - When Klipper G-code lacks the token, fall back to the printer profile value so preview math always has a ratio.

5. **Data Plumbing**
   - Feed the stored ratio into `TimeMachine` (prepare to read it from `MachineEnvelopeConfig`, probably via a helper similar to `get_option_value`).
   - Ensure the ratio is serialized/deserialized in 3MF/project configs (verify no hard-coded allowlists need to be updated; if they do, touch `PresetBundle::printer_options()` and related doc comments).

6. **Docs & Validation**
   - Add a short blurb to `doc/print_settings/speed/...` (or a new printer-settings page) describing what the cruise ratio controls.
   - Manual test: switch value, export 3MF, re-import, inspect generated G-code for `MINIMUM_CRUISE_RATIO`.

## Dependencies / Open Questions
- Confirm whether `MachineEnvelopeConfig` currently round-trips every option automatically; if not, extend serialization helpers.
- The UI should hide the field for non-Klipper flavors.
