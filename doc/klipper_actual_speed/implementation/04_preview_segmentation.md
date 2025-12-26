# Subtask 4 – Actual Speed Segment Subdivision

## Overview
- `GCodeViewer` now constructs a preview-only copy of the parsed toolpath (`m_preview_moves`) where Klipper-controlled extrusions and travels are subdivided into preview chunks no longer than **1 mm**.
- Each chunk inherits the parent move’s metadata (role, tool, widths, etc.) but stores its own entry/exit/peak speeds, trapezoid slice and travel distance so the existing color pipeline can treat it like a standalone move.
- The pipeline still renders from a single move stream, so all existing preview modes, sequential navigation, and statistics continue to work without duplicating GPU buffers.

## Data Plumbing
- `rebuild_preview_moves()` runs during `GCodeViewer::load()` and copies `GCodeProcessorResult::moves` into `m_preview_moves`.
- For every extrude or travel move with Klipper kinematics, `append_segmented_move()` computes phase boundaries (accel / cruise / decel) from the stored trapezoid distances, then slices the move into ≤1 mm pieces aligned to those phases.
- Each slice:
  - scales `delta_extruder`, `time`, and `layer_duration` proportionally to its share of the parent distance,
  - samples entry/exit/peak speeds via `MoveVertex::actual_speed_at()`,
  - tags its dominate phase so `actual_speed_at()` can keep working with the shorter trapezoid.
- Moves lacking Klipper metadata (other firmware, arcs that already inject interpolation points, zero-length hops, etc.) are copied verbatim to keep their behavior unchanged.

## Rendering & Legend
- `load_toolpaths()` now receives the preview vector and uses it everywhere toolpaths are generated, so paths, ranges, sequential markers, and the legend naturally operate on chunk speeds.
- Because chunked vertices keep their original line width, height, role, extruder, and `gcode_id`, other preview modes and the slider remain visually identical while the Actual Speed view gains the required color gradients.

## Assumptions & Notes
- Arcs are left untouched in this iteration; they already render via dense interpolation points so the preview still shows their curvature correctly. Straight G0/G1 moves—the majority of Klipper motion—receive full 1 mm sampling.
- Segment subdivision always lives in `m_preview_moves`; the raw `GCodeProcessorResult` stays byte-for-byte the same for exports and any non-preview consumers.
- Caching is handled implicitly by storing the expanded move list as part of the viewer state. Reloading the same G-code reuses the existing chunked vector unless the source data changes.

