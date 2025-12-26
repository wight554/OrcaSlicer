
# Addendum: Actual Speed Preview – Preview-only Segment Subdivision

## Context / Problem

The current G-code previewer represents each G-code move as a **single segment with a single speed value**
(both for requested speed and speed-based preview overlays).

While acceptable for performance-oriented previews, this approach is **insufficient for the new
Actual Speed preview**, because:

- Real printer motion within a single G-code move is **not constant-speed**
- Acceleration, deceleration, SCV, cruise_ratio, and lookahead effects all vary **along the move length**
- Long straight moves appear as flat-color bands, hiding real acceleration ramps

This significantly reduces the usefulness of the Actual Speed visualization.

---

## Core Requirement (Hard Rule)

**Actual Speed preview MUST subdivide G-code moves into smaller preview-only chunks such that each chunk
represents an approximately constant speed region.**

> ⚠️ This subdivision is **STRICTLY for preview rendering**.  
> **Exported G-code MUST NOT be modified or split.**

---

## Segment Subdivision Rules

### 1. Subdivision Criteria

A G-code move must be subdivided for Actual Speed preview based on:

- Motion phase boundaries:
  - acceleration phase
  - cruise (constant speed) phase
  - deceleration phase
- AND a maximum spatial resolution constraint

### 2. Spatial Resolution Constraint

- **Maximum preview chunk length:** `1.0 mm`
- Smaller chunks are allowed to align with phase boundaries
- Larger chunks are not allowed, even in cruise

This guarantees spatial fidelity of the preview.

---

## Illustrative Example

Given:
- Move length: `30 mm`
- cruise_ratio: `0.5`
- Acceleration + deceleration consume 50% of the move

Preview-only subdivision:

- Acceleration phase:
  - ~7.5 mm → split into ~1 mm chunks (7–8 chunks)
- Cruise phase:
  - ~15 mm → split into 1 mm chunks (15 chunks)
- Deceleration phase:
  - ~7.5 mm → split into ~1 mm chunks (7–8 chunks)

Each chunk:
- Has its own computed **actual speed**
- Is independently colorized in the Actual Speed legend

---

## Motion Sampling Model (Updated)

After lookahead and peak speed calculation per G-code move:

1. Compute:
   - Entry speed `v_in`
   - Peak speed `v_peak`
   - Exit speed `v_out`
   - Acceleration distance `d_accel`
   - Cruise distance `d_cruise`
   - Deceleration distance `d_decel`

2. Define phase boundaries:
   - `[0, d_accel]`
   - `[d_accel, d_accel + d_cruise]`
   - `[d_accel + d_cruise, L]`

3. Subdivide each phase into preview chunks:
   - Chunk length ≤ `1.0 mm`
   - Phase-aligned where possible

4. For each chunk, compute speed analytically:

   - Acceleration:
     v(x) = sqrt(v_in² + 2·a·x)

   - Cruise:
     v = v_peak

   - Deceleration:
     v(x) = sqrt(v_out² + 2·a·(L − x))

5. Assign each chunk:
   - actual_speed
   - reference to parent G-code move (for tooltips/debugging)

---

## Preview Rendering Requirements

### Actual Speed Legend

- Uses **subdivided preview chunks**, not original G-code moves
- Color mapping is per chunk
- Tooltip shows:
  - Requested speed (original move)
  - Actual speed (chunk)
  - Optional (advanced):
    - Motion phase (accel / cruise / decel)
    - Limiting factor

### Other Preview Modes

All other preview modes MUST remain unchanged and use original G-code moves:
- Requested speed preview
- Feature / line type preview
- Toolpath preview
- Exported G-code

---

## Architectural Implications

### Preview-only Data Structure

Introduce a preview-only structure (example name):

`PreviewMotionSegment`
- start_position
- end_position
- actual_speed
- parent_gcode_move_id

These segments must exist **only in the preview pipeline**.

### Pipeline Separation

Actual Speed preview path:
```
G-code moves
  → motion analysis (lookahead, v_in/v_out)
  → preview-only subdivision
  → Actual Speed rendering
```

All other previews:
```
G-code moves → render as-is
```

---

## Performance Constraints

- Subdivision is enabled **only when Actual Speed legend is active**
- Maximum preview segment density:
  - ~1000 segments per meter at 1 mm resolution
- Acceptable for preview-only use
- Subdivided preview geometry must be cached based on:
  - toolpath hash
  - motion-related settings (accel, SCV, cruise_ratio)

---

## Validation Criteria

- Long straight moves show:
  - smooth color gradients during accel/decel
  - flat color during cruise
- Short moves:
  - often show no cruise
  - visibly acceleration-limited
- Changing cruise_ratio:
  - visibly changes cruise region length
- Exported G-code remains **byte-identical**

---

## Summary

- Do NOT split G-code
- DO split preview geometry for Actual Speed
- Subdivide until:
  - motion phase boundaries
  - OR ≤ 1 mm chunk size
- This is a **sampling and visualization correction**, not a planner rewrite
