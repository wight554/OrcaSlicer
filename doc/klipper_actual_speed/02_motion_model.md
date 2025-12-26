# Subtask 2 – Motion Model Enhancements

## Goal
Extend the existing time-estimation planner inside `GCodeProcessor` so it can calculate Klipper-aware entry/exit/peak speeds per move, honoring square-corner velocity (SCV), acceleration limits, and the new cruise ratio constraint. Persist these kinematic results so later stages (preview/UI) can consume them.

## Work Items
1. **Data Structures**
   - Augment `GCodeProcessorResult::MoveVertex` (and/or add a parallel `SegmentKinematics` vector) with fields for requested speed, actual entry/exit/peak speeds, cruise distance split, and a `LimitingFactor` enum (`Requested`, `Acceleration`, `SCV`, `CruiseRatio`, `Lookahead`).
   - Ensure `reset()`/copy logic in `GCodeViewer` respects the new fields.

2. **SCV Tracking**
   - Introduce `machine_square_corner_velocity` storage on `MachineEnvelopeConfig` (or reuse the parsed `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY` value explicitly instead of overloading jerk).
   - Whenever parser sees `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=...`, store it; otherwise use printer preset defaults. Provide helper `current_scv(mode)` for planner use.

3. **Cruise Ratio Constraint**
   - Add `float cruise_ratio` to `TimeBlock` plus convenience helpers (e.g., `bool has_cruise_ratio()`).
   - When blocks are created in `process_G1`/`process_G2_G3`, set `cruise_ratio` to the active ratio (for Klipper only; zero disables the check).
   - Update `TimeBlock::calculate_trapezoid()` so that when `cruise_ratio > 0`, it enforces `d_accel + d_decel ≤ (1 - r) * distance` by capping the peak feedrate per the spec equation (v_peak_cr).

4. **SCV Corner Limit**
   - During junction planning (where `vmax_junction` is computed around line ~2860), compute the turn angle θ using the normalized `enter_direction/exit_direction`.
   - Convert the SCV limit `v_corner = scv / max(sin(θ/2), ε)` and fold it into `vmax_junction` (only if both segments have XY motion; handle travel-only cases).

5. **Lookahead Iterations**
   - Wrap the existing forward/reverse pass in `TimeMachine::calculate_time()` with 3–5 iterations (or until convergence) so the cruise-ratio clamp + SCV propagates properly. Guard iterations so legacy flavors retain behaviour.

6. **Recording Actual Speeds**
   - Before a `TimeBlock` is erased from `blocks`, derive entry/exit/peak speeds and limiting factor, then map them back to the move index using `block.g1_line_id` and the move list (`m_result.moves` / sequential ID map).
   - Populate the new per-move kinematic fields (for both extrusion and travel blocks).
   - For blocks split into multiple preview vertices (e.g., arcs), store trapezoid parameters so downstream code can sample v(x).

7. **Edge Cases**
   - Handle zero-length moves, pure E-only retractions, spiral vase mode, prepare-stage moves (should probably mark limiter as `Prepare` so preview can optionally hide them).
   - Ensure first move (the dummy vertex) stays zeroed.

8. **Testing Hooks**
   - Add unit coverage in `tests/libslic3r` that feeds synthetic G-code into `GCodeProcessor` and inspects the new kinematic data for known cases (straight line at low accel, 90° corner with SCV cap, short segment hitting cruise ratio).
   - Document manual regression steps (compare planner output with Klipper `QUERY_PRINT_STATS` on a small file if possible).

## Dependencies / Open Questions
- Need a stable mapping from `g1_line_id` to move indices; verify `m_sequential_view.gcode_ids` already reflects this or build a dedicated map.
- Decide how to treat Z-only moves (probably ignore for actual speed legend to avoid noise).

## Acceptance Criteria
- For Klipper flavor, every move now has actual entry/exit/peak speeds that reflect SCV + cruise ratio.
- For other flavors, existing timing results remain unchanged (the new fields default to requested speeds).
- Unit test demonstrates at least one scenario where actual speed differs from requested.

