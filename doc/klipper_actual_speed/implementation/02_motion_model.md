# Subtask 2 – Motion Model Enhancements

## Overview
- Extended `GCodeProcessor`'s time estimator so Klipper jobs now yield per-move entry/exit/peak speeds that respect square-corner velocity (SCV), acceleration limits, and the printer's minimum cruise ratio.
- Persisted a `MoveVertex::Kinematics` bundle so the preview and downstream tooling can sample v(x) across G0/G1/G2/G3 segments without needing to re-run the planner.

## Planner & Data Plumbing
- `GCodeProcessorResult::MoveVertex` gained a `Kinematics` struct and limiter enum (`Requested`, `Acceleration`, `SCV`, `CruiseRatio`, `Lookahead`, `Prepare`). Every move is seeded with “requested == actual” defaults so non-Klipper flavors remain unchanged.
- `GCodeProcessor::TimeBlock` now tracks the requested feedrate, minimum cruise ratio, SCV/Lookahead flags, and whether the segment contains XY motion. `TimeMachine` instances keep their current SCV (default 5 mm/s) plus a `collect_kinematics` flag (enabled for Klipper Normal mode only).
- Added a `g1_line_id → move indices` table that is populated when the preview vertex for a G1/G2/G3 command is enqueued. Before a `TimeBlock` leaves the planner queue the matching `MoveVertex` range is updated with trapezoid distances, entry/exit/peak values, requested feedrate, and the resolved limiter.
- `TimeBlock::calculate_trapezoid()` enforces the cruise ratio via `v_peak_cr^2 = a (1 - r) d + 0.5 (v_entry^2 + v_exit^2)` (i.e. the spec's `d_acc + d_dec ≤ (1 - r) * distance`). When the constraint bites, both the trapezoid and the block's “nominal” cruise speed are clamped so lookahead iterations see the reduced peak.
- Forward/reverse lookahead now uses the same cruise-ratio allowance that Klipper derives from `accel_to_decel`: every `max_allowable_speed()` call works with `distance = (1 - ratio) · block_length`, so entry/exit smoothing can only consume the portion of the move that Klipper would dedicate to acceleration/deceleration.

## SCV & Lookahead
- `process_SET_VELOCITY_LIMIT` now feeds the parsed `SQUARE_CORNER_VELOCITY` into every `TimeMachine`. If Klipper never emits the command we fall back to 5 mm/s as suggested by the spec.
- During junction planning we compute the XY turn angle using the normalized `enter_direction/exit_direction`. For non-zero XY motion on both segments we compute `v_corner = scv / max(sin(θ/2), ε)` and fold it into `vmax_junction`. Z-only or pure-E moves skip the check.
- The forward/reverse passes now run up to four iterations for Klipper so SCV and cruise-ratio clamps propagate fully. We also flag blocks whose entry speed is reduced by the reverse pass so they can be labeled `Lookahead`.

## Actual Speed Capture
- `TimeMachine::calculate_time()` records kinematics for each finished block before it leaves the queue. Each completed move stores entry/exit/peak speeds in mm/s plus the trapezoid distances (accelerate, cruise, decelerate). Prepare-stage moves force the limiter to `Prepare`.
- When Klipper is not active the new fields remain at their seeded “requested” values so UI consumers can render the same data as before without special handling.

## Edge Cases & Assumptions
- SCV limiting purposely ignores segments without XY motion (pure Z hops / retractions). Requested feedrates for Klipper moves come from the raw `F` command, while the planner still observes machine min feedrates.
- The per-move map uses the last `MoveVertex` emitted for a G1/G2/G3 command. Arcs currently emit a single vertex, so no additional subdivision is required. If future preview code splits arcs into multiple vertices, the map already stores `[first,last]` ranges.
- Minimum cruise ratio is only applied when the flavor is Klipper; other flavors zero out the ratio each time we switch printers.
- Additional iterations are capped at four to avoid pathological runtimes; this matched the convergence we saw on internal test G-code.

