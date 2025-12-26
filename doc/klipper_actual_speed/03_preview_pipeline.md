# Subtask 3 – Preview Data Plumbing

## Goal
Expose the computed actual-speed information to the preview renderer so each toolpath vertex (and any interpolated tessellation point) carries both requested and achievable velocities. Update range tracking/state so the new legend tab can scale colors correctly.

## Work Items
1. **Result Struct Updates**
   - Extend `GCodeProcessorResult::MoveVertex` with fields such as `actual_entry_speed`, `actual_exit_speed`, `actual_peak_speed`, `limiting_factor`, and optionally `trapezoid_accel_mm`, `trapezoid_cruise_mm` for downstream sampling.
   - Update `store_move_vertex()`, copy/move constructors, and existing code that zeroes the struct to initialize the new data.

2. **Sampling Utility**
   - Implement a helper (e.g., `MoveVertex::speed_at(float distance_along)` or a free function in `GCodeViewer`) that, given the trapezoid info and a point’s relative distance, returns the interpolated actual speed per the spec’s formulas.
   - For arcs, compute cumulative distance along the interpolation polyline and reuse the same helper.

3. **Range Tracking**
   - Add `Extrusions::Ranges::actual_speed` (and maybe reuse for travels separately) so `refresh()` can build min/max stats from the new per-move data.
   - When the feature is disabled (non-Klipper), quietly keep `actual_speed` equal to `feedrate` so the legend still works.

4. **Path Payload**
   - Extend `GCodeViewer::Path` with `float actual_speed` (or entry/exit pair) alongside existing `feedrate`.
   - Update `TBuffer::add_path()` and the path-merging logic so grouped subpaths only merge when the *actual* speed is similar (avoid averaging segments with different actual speeds).
   - Ensure `render_paths` copy or reference the new value for color selection.

5. **Vertex Attributes**
   - When building vertex buffers (`add_vertices_as_line`, `add_vertices_as_solid`, instanced models), stash the sampled actual speed into an unused component or a parallel array so shaders/CPU color selection can access it.
   - Update smoothing routines (`smooth_vertices` etc.) to preserve the extra attribute so AA triangles don’t smear the speeds.

6. **Sequential View / Slider Data**
   - Populate `m_sequential_view.gcode_ids` / range caps with actual speed info so the horizontal slider and camera marker know which speed to display when hovering moves.
   - Consider keeping a per-layer summary (min/max actual speed) to quickly clamp the legend when layers are filtered.

7. **Performance & Memory**
   - Audit GPU upload paths to ensure the added float(s) keep buffers aligned (PositionNormal3 becomes PositionNormal3Speed?).
   - If GPU attribute limits become an issue, we can store actual speed in the per-path CPU data and assign colors on the CPU—call this out as an implementation decision point.

8. **Testing**
   - Load a simple file, toggle to the new view mode (implemented in Subtask 4) and verify the range map matches the backend numbers (e.g., known slowdowns near sharp turns).
   - Include regression coverage for multi-plate / multi-extruder files to ensure ranges reset properly when reloading.

## Dependencies / Open Questions
- Coordination with Subtask 2 on what exact data is exposed (entry/exit vs per-sample). Adjust helper signatures accordingly.
- Need to confirm whether we also want to expose actual speeds for travel moves or only extrusion paths (spec implies both).

## Acceptance Criteria
- For each move, preview buffers carry accurate actual-speed metadata.
- `refresh()` reports sensible min/max actual speeds even when only a subset of layers is visible.
- No regressions in existing color schemes (Feature type, Flow, etc.).

