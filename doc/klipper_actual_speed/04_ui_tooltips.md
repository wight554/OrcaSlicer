# Subtask 4 – UI, Legend & Tooltips

## Goal
Expose the “Actual speed” visualization in the Preview panel: a new legend tab, dropdown entry, and tooltips showing requested vs achievable speeds plus contextual details (entry/exit/peak speed, limiting factor). Keep the UX consistent with existing color schemes.

## Work Items
1. **View Type & Legend Wiring**
   - Add `EViewType::ActualSpeed` to `GCodeViewer` (`GCodeViewer.hpp/.cpp`), insert it into `view_type_items` after the current “Speed” entry, and provide a localized label (e.g., `_u8L("Actual speed")`).
   - Extend the legend rendering switch so selecting the new view displays the `actual_speed` range (from Subtask 3) with a “mm/s” label. Support both linear and auto-clamped scaling (clamp to requested speed range if specified).

2. **Color Selection**
   - Update the color resolver lambda in `refresh_render_paths()` so travels/extrusions pull from `path.actual_speed` when `m_view_type == ActualSpeed`.
   - Decide how to color retractions/other move types (either keep existing travel colors or re-use the gradient with their actual speeds).

3. **Tooltips & Marker Pane**
   - Enhance `SequentialView::Marker::render()` to display:
     - Requested speed (current behaviour),
     - Actual entry/peak/exit speeds,
     - Limiting factor (text, e.g., “Limited by SCV”).
   - If data is unavailable (non-Klipper), show “n/a” or fall back to requested speed.

4. **Per-vertex Hover Tooltip**
   - Wherever we show the quick ImGui tooltip when hovering a path (search for `ImGui::IsItemHovered()` in `render_toolpaths`), append actual speed info and delta vs requested.
   - Optionally include the sample’s `entry_speed`, `exit_speed`, and whether the cruise segment existed.

5. **Legend Tabs**
   - Inside `render_legend()`, add a descriptive block explaining what “Actual speed” represents and the units. Consider showing min/max/avg values near the gradient (similar to Layer Time view).
   - When auto-scaling the gradient, show the derived range (e.g., “0–400 mm/s”) so users know the context.

6. **State Persistence**
   - Remember the last selected view type (`wxGetApp().app_config`) so users who prefer Actual-Speed don’t have to re-select each time.
   - Ensure per-plate stats/export dialogues respect the new state (e.g., `render_calibration_thumbnail` should still render requested speed unless explicitly asked otherwise).

7. **Docs & Help**
   - Update relevant documentation (`doc/print_settings/speed/…` or `SoftFever_doc`) and tooltips to describe how to interpret the gradient and limiting-factor messages.

8. **Testing**
   - Manual smoke test on a Klipper preset: verify the gradient changes near corners/short segments and tooltips show non-zero deltas.
   - Regression test for non-Klipper flavors: the dropdown entry should still appear but act as an alias to requested speed (or hide entirely, depending on UX decision).

## Dependencies / Open Questions
- Requires Subtasks 2 & 3 to populate actual-speed data first.
- Need UX decision: hide Actual Speed for printers that are not set to Klipper, or allow it (displaying requested speed) but show a warning banner.

## Acceptance Criteria
- Preview dropdown contains “Actual speed” and switching to it recolors toolpaths using actual speed values.
- Marker panel + hover tooltip list requested vs actual speeds and the limiting factor.
- No regressions in other legend tabs, and the feature gracefully degrades on non-Klipper printers.

