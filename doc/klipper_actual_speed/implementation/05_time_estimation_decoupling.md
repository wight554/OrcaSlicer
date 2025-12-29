# Subtask 5 – Time Estimation Decoupling

## Overview
This document describes the fix that decouples time estimation from preview kinematics calculations. The original implementation inadvertently changed time estimation behavior for Klipper printers, which was not the intended design.

## Problem Statement
The initial Klipper cruise ratio and SCV (Square Corner Velocity) implementation modified the time estimation planner in ways that affected print time calculations:

1. **Cruise ratio in `calculate_trapezoid()`**: Limited peak speeds based on cruise ratio constraints
2. **Cruise ratio in lookahead passes**: Used reduced distances in `max_allowable_speed()` calculations
3. **SCV junction limiting**: Applied corner velocity limits in process_G1/G2_G3
4. **Multiple lookahead iterations**: Changed from 1 to 4 iterations for Klipper mode

These changes made time estimates more conservative (longer) for Klipper printers, which contradicted the design intent stated in Subtask 1: *"Current work only records and forwards the ratio; no planner math consumes it yet."*

## Solution
The fix separates time estimation from preview kinematics by:

1. **Reverting time estimation to original algorithm**:
   - `calculate_trapezoid()` uses the standard trapezoidal motion profile without cruise ratio limiting
   - Lookahead passes use full block distance (not cruise-ratio-reduced distance)
   - No SCV junction limiting in time calculation
   - Single lookahead iteration (original behavior)

2. **Storing both unlimited and SCV-limited junction velocities**:
   - `process_G1()` and `process_G2_G3()` now calculate both values
   - Unlimited `vmax_junction` is used for time estimation
   - SCV-limited `scv_max_entry_speed` is stored for preview use
   - Minimal extra computation since SCV is calculated alongside time estimation

3. **Preserving preview kinematics with full Klipper constraints**:
   - `record_block_kinematics()` applies SCV-limited entry speed when available
   - Then applies cruise ratio limiting on top of SCV
   - Preview visualization shows "actual" speeds that respect Klipper's motion constraints
   - Limiting factor detection (`CruiseRatio`, `SCV`, etc.) remains functional for tooltips

## Implementation Details

### Files Modified
- `src/libslic3r/GCode/GCodeProcessor.hpp`
- `src/libslic3r/GCode/GCodeProcessor.cpp`

### Key Changes

#### 1. `TimeBlock` Structure (GCodeProcessor.hpp)
Added `scv_max_entry_speed` field to store SCV-limited junction velocity:
```cpp
struct TimeBlock {
    float max_entry_speed{ 0.0f };     // mm/s - for time estimation
    float scv_max_entry_speed{ 0.0f }; // mm/s - SCV-limited junction speed for preview
    // ...
};
```

#### 2. `TimeBlock::calculate_trapezoid()`
Reverted to original implementation that calculates standard trapezoidal motion profile:
```cpp
void GCodeProcessor::TimeBlock::calculate_trapezoid()
{
    trapezoid.cruise_feedrate = feedrate_profile.cruise;
    float accelerate_distance = estimated_acceleration_distance(...);
    float decelerate_distance = estimated_acceleration_distance(...);
    // Standard trapezoid calculation without cruise ratio
}
```

#### 3. Lookahead Passes
Reverted to use `block.distance` instead of `cruise_ratio_delta_distance(block)`:
```cpp
static void planner_forward_pass_kernel(...)
{
    float entry_speed = std::min(curr.feedrate_profile.entry,
        max_allowable_speed(-prev.acceleration, prev.feedrate_profile.entry, prev.distance));
}
```

#### 4. `TimeMachine::calculate_time()`
Reverted to single iteration:
```cpp
// forward_pass
for (size_t i = 0; i + 1 < blocks.size(); ++i)
    planner_forward_pass_kernel(blocks[i], blocks[i + 1]);

// reverse_pass
for (int i = static_cast<int>(blocks.size()) - 1; i > 0; --i)
    planner_reverse_pass_kernel(blocks[i - 1], blocks[i]);

recalculate_trapezoids(blocks);
```

#### 5. SCV Calculation in `process_G1()` / `process_G2_G3()`
Now calculates both unlimited and SCV-limited junction velocities:
```cpp
// After jerk limiting...
if (limited)
    vmax_junction *= v_factor;

// Calculate SCV-limited junction velocity for Klipper preview (stored separately)
float vmax_junction_scv = vmax_junction;
if (is_klipper && block.has_xy_motion) {
    // Calculate corner velocity based on turn angle
    float v_corner = current_scv(...) / sin_half;
    if (v_corner < vmax_junction_scv) {
        vmax_junction_scv = v_corner;
        block.scv_limited = true;
    }
}

// Store SCV-limited junction velocity for preview
block.scv_max_entry_speed = vmax_junction_scv;

// Time estimation uses unlimited vmax_junction
block.max_entry_speed = vmax_junction;
```

#### 6. `record_block_kinematics()`
Now applies both SCV and cruise ratio constraints for preview:
```cpp
void GCodeProcessor::record_block_kinematics(const TimeBlock& block, ...)
{
    // Start with time estimation values
    float entry_speed = block.feedrate_profile.entry;
    // ...

    // First apply SCV-limited entry speed if applicable
    if (block.scv_limited && block.scv_max_entry_speed < entry_speed) {
        entry_speed = block.scv_max_entry_speed;
        // Recalculate trapezoid with SCV-limited entry
    }

    // Then apply cruise ratio limiting on top of SCV
    if (block.has_cruise_ratio() && ...) {
        // Recalculate trapezoid with limited peak speed
    }
}
```

#### 7. Removed Unused Function
Removed `cruise_ratio_delta_distance()` as it's no longer needed in the time estimation path.

## Behavior Summary

| Feature | Time Estimation | Preview Kinematics |
|---------|-----------------|-------------------|
| Cruise ratio limiting | Not applied | Applied |
| SCV corner limiting | Not applied | Applied |
| Multiple iterations | 1 iteration | N/A |
| Junction velocity | Unlimited | SCV-limited stored |
| Trapezoid calculation | Original algorithm | Recalculated with limits |

## Design Rationale
The approach of storing both SCV-limited and unlimited junction velocities provides:

1. **Minimal overhead**: SCV calculation happens during the same pass as time estimation
2. **Clean separation**: Time estimation code path is completely unchanged
3. **Full preview accuracy**: Preview shows speeds limited by both SCV and cruise ratio
4. **Correct tooltips**: Limiting factor correctly identifies "SCV" or "CruiseRatio"

## Testing Considerations
- Time estimates should match pre-Klipper-feature behavior
- Preview "Actual Speed" visualization should show both SCV and cruise-ratio-limited speeds
- Limiting factor tooltips should correctly identify "SCV" when corner velocity limits
- Limiting factor tooltips should correctly identify "CruiseRatio" when cruise ratio limits
- Non-Klipper printers should be completely unaffected (no SCV calculation, no cruise ratio)
