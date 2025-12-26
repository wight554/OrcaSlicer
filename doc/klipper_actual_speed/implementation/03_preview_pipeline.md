# Subtask 3 – Preview Data Plumbing

## Overview
- Added actual-speed awareness to the preview stack. Every `MoveVertex` now exposes helpers (`actual_*_speed()`, `actual_speed_at()`) so downstream code can sample the time estimator’s trapezoids without re-running the planner.
- Preview ranges, legends, and render paths received a dedicated “Actual speed” color mode that stays in sync with Klipper’s cruise-ratio and SCV limits while quietly falling back to requested feedrates when Klipper metadata is absent.
- Sequential view, markers, and range caps understand the same data so hovering the slider or camera marker surfaces the achievable velocity instead of the raw `F` value.

## Rendering & Data Flow
- `GCodeViewer::Path` stores a `actual_speed` payload populated via `MoveVertex::actual_peak_speed()`. Merging logic rejects incompatible actual speeds (±5%), preventing high/low-speed segments from being averaged into one colored span.
- `Extrusions::Ranges` tracks `actual_speed` so the legend and shader palette can rescale when filters hide certain move types. Travels reuse the same range, so toggling the new “Actual speed” view paints both extrusions and moves on the same scale.
- Sequential data adds a sibling `actual_speeds` vector alongside `gcode_ids`, letting the marker and slider reuse cached peaks while the UI fetches full `MoveVertex` data when it needs entry/exit samples.
- The toolhead marker shows both requested and actual speeds, annotates the current kinematic phase (accel/cruise/decel), and prints the limiting factor text (“Limited by SCV”, “Limited by Acceleration”, etc.) so the tooltip matches Klipper’s planner diagnostics. Each preview-only subsegment recomputes the phase from its own per-chunk trapezoid, ensuring the tooltip reflects whether that slice is still accelerating, coasting, or slowing down. Range caps automatically adopt the new colors because they read from the updated path payloads.

## Assumptions & Notes
- The horizontal slider continues to expose G-code IDs in its tooltip (to preserve line lookups) while the marker displays the actual velocity; deeper slider UI changes can hook into `SequentialView::actual_speeds` if we want dual readouts later.
- Per-layer actual-speed summaries reuse the existing range recomputation path. Layers filtered out of the viewport automatically shrink the range, so no extra per-layer accumulator was deemed necessary for this iteration.
- No new GPU attributes were introduced; actual speeds stay on the CPU side via `Path` payloads and sequential caches, which keeps AA corner smoothing unchanged while still making the data accessible for future shader work.

