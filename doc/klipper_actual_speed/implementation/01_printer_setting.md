# Subtask 1 – Printer Setting & Persistence

## Overview
Klipper now receives a persistent `klipper_cruise_ratio` (default `0.5`) so the slicer and backend agree on the minimum cruise ratio used when converting moves into `SET_VELOCITY_LIMIT`. The value is part of printer presets/3MFs, surfaced in the Motion ability page, emitted to G-code once per print, and fed into the preview time estimator so follow‑up subtasks can use it.

## Config & UI
- `MachineEnvelopeConfig` / `PrintConfig` define `klipper_cruise_ratio` as a scalar `ConfigOptionFloat` (`src/libslic3r/PrintConfig.{hpp,cpp}`) with CLI bounds `0–1` (internally clamped to `0.01–0.99`), advanced mode, and categorized under “Machine limits”.
- `Preset.cpp` includes the key in `s_Preset_machine_limits_options`, so printer presets, bundles, and physical printers serialize it without additional plumbing.
- `TabPrinter::build_kinematics_page()` now adds the option right below `emit_machine_limits_to_gcode`. `append_option_line` keeps the Normal/Silent dual column layout; both columns currently edit the same scalar value per the spec requirement.
- `toggle_options()` hides the row when the printer flavor isn’t Klipper, so other firmware pages stay unchanged.

## Firmware Sync
- `GCodeWriter` stores the configured ratio, emits `MINIMUM_CRUISE_RATIO=<value>` alongside the first applicable `SET_VELOCITY_LIMIT`, and caches the last value so the command only appears when the ratio changes (effectively once per print today).
- The helper is integrated into `set_print_acceleration`, `set_jerk_xy`, and `set_accel_and_jerk`, so any path that produces Klipper velocity-limit commands automatically appends the ratio token when needed.

## Parsing & TimeMachine
- `GCodeProcessor::process_SET_VELOCITY_LIMIT()` recognizes `MINIMUM_CRUISE_RATIO=` and updates both `m_time_processor.machine_limits.klipper_cruise_ratio` and every `TimeMachine::minimum_cruise_ratio`.
- `apply_config()` seeds each `TimeMachine` with the printer-profile value before parsing begins, giving the estimator a deterministic fallback whenever the incoming G-code omits the token.
- `TimeMachine::reset()` initializes the ratio to `0.5f`, so new instances have a sane default even before configuration is applied.

## Assumptions & Notes
- Although most Motion ability entries are stored as vectors (Normal/Silent), this ratio remains a single value to mirror the single Klipper command. CLI validation relies on the integer min/max fields, so the config definition advertises `0–1` even though runtime code still clamps to the narrower `0.01–0.99` range before emitting G-code.
- Current work only records and forwards the ratio; no planner math consumes it yet. Downstream subtasks can read the value from `TimeMachine::minimum_cruise_ratio` without additional plumbing.
