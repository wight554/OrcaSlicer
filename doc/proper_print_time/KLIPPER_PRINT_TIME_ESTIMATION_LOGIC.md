# Klipper Print Time Estimation Logic - Complete Reference

This document provides a comprehensive breakdown of the Klipper print time estimation algorithm as implemented in the `klipper_estimator` tool. This is intended to be used as a reference for porting this logic to slicer software.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Core Concepts](#2-core-concepts)
3. [Trapezoidal Velocity Profile](#3-trapezoidal-velocity-profile)
4. [Move Types and Classification](#4-move-types-and-classification)
5. [Junction Velocity Calculation](#5-junction-velocity-calculation)
6. [Two-Pass Velocity Planning Algorithm](#6-two-pass-velocity-planning-algorithm)
7. [Move Checkers (Axis/Extruder Limiters)](#7-move-checkers-axisextruder-limiters)
8. [Arc Handling (G2/G3)](#8-arc-handling-g2g3)
9. [Firmware Retraction (G10/G11)](#9-firmware-retraction-g10g11)
10. [Delay Handling](#10-delay-handling)
11. [Configuration Parameters Reference](#11-configuration-parameters-reference)
12. [G-code Commands Affecting Timing](#12-g-code-commands-affecting-timing)
13. [Complete Formulas Reference](#13-complete-formulas-reference)
14. [Implementation Pseudocode](#14-implementation-pseudocode)
15. [Parameter Usage Reference Table](#15-parameter-usage-reference-table)

---

## 1. Overview

The Klipper print time estimator replicates Klipper firmware's motion planning algorithm to accurately predict print duration. The key insight is that 3D printers don't move at constant requested speeds - they must accelerate and decelerate, and slow down at corners.

### High-Level Flow

```
G-code File
    │
    ▼
┌─────────────────────────────┐
│  Parse G-code Commands      │
│  - Extract moves (G0/G1)    │
│  - Track position state     │
│  - Handle special commands  │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│  Create Planning Moves      │
│  - Calculate distance       │
│  - Set velocity limits      │
│  - Apply axis constraints   │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│  Apply Junction Velocities  │
│  - Corner angle calculation │
│  - Centripetal acceleration │
│  - Extruder rate limiting   │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│  Two-Pass Velocity Planning │
│  - Backward pass: limits    │
│  - Forward pass: resolve    │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│  Calculate Move Times       │
│  - Accel/Cruise/Decel times │
│  - Sum all move times       │
└──────────────┴──────────────┘
               │
               ▼
         Total Print Time
```

---

## 2. Core Concepts

### 2.1 Position State

The toolhead position is tracked as a 4D vector: `[X, Y, Z, E]`

- **X, Y, Z**: Cartesian position in mm
- **E**: Extruder position in mm

### 2.2 Position Modes

Each axis can be in one of two modes:
- **Absolute**: Commanded position is the target position
- **Relative**: Commanded position is added to current position

**Default modes:**
- X, Y, Z: Absolute
- E: Relative

**Commands that change modes:**
- `M82`: E axis to Absolute
- `M83`: E axis to Relative
- `G92`: Set current position without moving

### 2.3 Velocity Units

- G-code feedrate `F` is in **mm/min**
- Internal calculations use **mm/s** (divide F by 60)
- Velocity squared (v²) is often used to avoid square root operations

### 2.4 Move Distance

For kinematic moves (XYZ movement):
```
distance = sqrt((x2-x1)² + (y2-y1)² + (z2-z1)²)
```

For extrude-only moves (no XYZ movement):
```
distance = |e2 - e1|
```

---

## 3. Trapezoidal Velocity Profile

Every move follows a trapezoidal velocity profile with three phases:

```
Velocity
    │
    │      ┌─────────────┐ cruise_v
    │     /               \
    │    /                 \
    │   /                   \
    │  /                     \
    │ / start_v               \ end_v
    └─────────────────────────────► Time
      │◄─ accel ─►│◄─ cruise ─►│◄─ decel ─►│
```

### 3.1 Phase Calculations

**Acceleration Distance:**
```
accel_distance = (cruise_v² - start_v²) / (2 × acceleration)
```

**Acceleration Time:**
```
accel_time = accel_distance / ((start_v + cruise_v) / 2)
```
Or equivalently:
```
accel_time = (cruise_v - start_v) / acceleration
```

**Deceleration Distance:**
```
decel_distance = (cruise_v² - end_v²) / (2 × acceleration)
```

**Deceleration Time:**
```
decel_time = decel_distance / ((cruise_v + end_v) / 2)
```

**Cruise Distance:**
```
cruise_distance = total_distance - accel_distance - decel_distance
cruise_distance = max(0, cruise_distance)  // Cannot be negative
```

**Cruise Time:**
```
cruise_time = cruise_distance / cruise_v
```

**Total Move Time:**
```
total_time = accel_time + cruise_time + decel_time
```

### 3.2 Edge Cases

- If `cruise_distance` would be negative, the move is "triangular" - it accelerates then immediately decelerates without reaching full cruise velocity
- The cruise velocity is reduced to ensure `accel_distance + decel_distance <= total_distance`

---

## 4. Move Types and Classification

### 4.1 Kinematic Move

A move where the toolhead physically moves in XYZ space:
```
is_kinematic = (start.xyz != end.xyz)
```

Properties:
- Distance calculated from XYZ displacement
- Limited by `max_velocity` and `max_acceleration`
- Subject to junction velocity constraints

### 4.2 Extrude-Only Move

A move where only the extruder moves (no XYZ movement):
```
is_extrude_only = (start.xyz == end.xyz) AND (start.e != end.e)
```

Properties:
- Distance = |delta_E|
- Limited by `max_extrude_only_velocity` and `max_extrude_only_accel`
- Always starts and ends at velocity 0 (unless in a sequence)

### 4.3 Move Rate Vector

The rate vector represents the direction of movement, normalized to unit length:

For kinematic moves:
```
rate = (end - start) / distance
// rate.x, rate.y, rate.z are direction components
// rate.w (E component) = delta_e / distance
```

For extrude-only moves:
```
rate = [0, 0, 0, sign(delta_e)]
```

---

## 5. Junction Velocity Calculation

The junction velocity is the maximum velocity allowed when transitioning from one move to another. This is critical for accurate time estimation.

### 5.1 Junction Deviation Formula

First, calculate `junction_deviation` from `square_corner_velocity`:
```
junction_deviation = scv² × (√2 - 1) / max_acceleration
```

Where:
- `scv` = square_corner_velocity (typically 5.0 mm/s)
- `√2 - 1 ≈ 0.414`

### 5.2 Angle Between Moves

Calculate the cosine of the angle between consecutive moves:
```
junction_cos_theta = -(current_rate.xyz · previous_rate.xyz)
// Note: Negative because we want the angle at the junction
```

Clamp to valid range:
```
junction_cos_theta = clamp(junction_cos_theta, -0.999999, 0.999999)
```

### 5.3 Special Case: Co-linear Moves

If `junction_cos_theta > 0.999999`, the moves are nearly co-linear (same direction). In this case, skip junction velocity calculation - the moves can transition at full speed.

### 5.4 Junction Velocity Components

Calculate intermediate values:
```
sin_theta_d2 = sqrt(0.5 × (1.0 - junction_cos_theta))
r = sin_theta_d2 / (1.0 - sin_theta_d2)
tan_theta_d2 = sin_theta_d2 / sqrt(0.5 × (1.0 + junction_cos_theta))
```

Calculate centripetal velocity constraints:
```
move_centripetal_v2 = 0.5 × current_distance × tan_theta_d2 × current_acceleration
prev_move_centripetal_v2 = 0.5 × previous_distance × tan_theta_d2 × previous_acceleration
```

### 5.5 Extruder Junction Speed

When the extrusion rate changes between moves, this limits junction velocity:
```
diff_r = |current_rate.e - previous_rate.e|

if diff_r > 0:
    extruder_v2 = (instant_corner_velocity / diff_r)²
else:
    extruder_v2 = current_max_cruise_v2  // No limit
```

### 5.6 Final Junction Velocity

The maximum start velocity for the current move is the minimum of all constraints:
```
max_start_v2 = min(
    extruder_v2,
    r × current_junction_deviation × current_acceleration,
    r × previous_junction_deviation × previous_acceleration,
    move_centripetal_v2,
    prev_move_centripetal_v2,
    current_max_cruise_v2,
    previous_max_cruise_v2,
    previous_max_start_v2 + previous_max_dv2
)
```

### 5.7 Smoothed Velocity Constraint

For deceleration smoothing (prevents jerky motion):
```
max_smoothed_v2 = min(
    max_start_v2,
    previous_max_smoothed_v2 + previous_smoothed_dv2
)
```

---

## 6. Two-Pass Velocity Planning Algorithm

The velocity planning uses a two-pass algorithm to resolve all velocity constraints.

### 6.1 Move Initialization

When a move is created, calculate initial constraints:

For kinematic moves:
```
max_cruise_v2 = velocity²  // From F parameter or max_velocity
max_dv2 = 2 × distance × max_acceleration
smoothed_dv2 = 2 × distance × accel_to_decel
max_start_v2 = 0  // Updated by junction calculation
max_smoothed_v2 = 0
```

For extrude-only moves:
```
max_cruise_v2 = velocity²
max_dv2 = INFINITY  // No acceleration limit propagation
smoothed_dv2 = INFINITY
max_start_v2 = 0
max_smoothed_v2 = 0
```

### 6.2 Accel-to-Decel Calculation

The `accel_to_decel` value determines how aggressively velocity changes are smoothed:

**Using minimum_cruise_ratio (newer method):**
```
accel_to_decel = max_acceleration × (1.0 - minimum_cruise_ratio)
```

**Using max_accel_to_decel (legacy method):**
```
accel_to_decel = min(max_accel_to_decel, max_acceleration)
```

**Default:**
```
accel_to_decel = min(50.0, max_acceleration)
```

### 6.3 Backward Pass

Process moves from last to first, calculating reachable velocities:

```python
next_end_v2 = 0  # Last move must end at 0
next_smoothed_v2 = 0

for move in reversed(moves):
    # Calculate what start velocity is reachable from next move's end
    reachable_start_v2 = next_end_v2 + move.max_dv2
    start_v2 = min(move.max_start_v2, reachable_start_v2)

    # Calculate smoothed velocity constraint
    reachable_smoothed_v2 = next_smoothed_v2 + move.smoothed_dv2
    smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)

    # Check if this move is a "peak" (smoothed constraint is binding)
    if smoothed_v2 < reachable_smoothed_v2:
        # This move constrains the peak velocity
        peak_cruise_v2 = min(move.max_cruise_v2,
                            (smoothed_v2 + reachable_smoothed_v2) × 0.5)

        # Calculate final velocities
        cruise_v2 = min(
            (start_v2 + reachable_start_v2) × 0.5,
            move.max_cruise_v2,
            peak_cruise_v2
        )

        move.start_v = sqrt(min(start_v2, cruise_v2))
        move.cruise_v = sqrt(cruise_v2)
        move.end_v = sqrt(min(next_end_v2, cruise_v2))
    else:
        # Delay processing - will be handled in forward pass
        delayed.append((move, start_v2, next_end_v2))

    next_end_v2 = start_v2
    next_smoothed_v2 = smoothed_v2
```

### 6.4 Forward Pass (Delayed Moves)

Process delayed moves forward to resolve their velocities:

```python
for (move, start_v2, end_v2) in reversed(delayed):
    cruise_v2 = min(start_v2, peak_cruise_v2)
    move.start_v = sqrt(min(start_v2, cruise_v2))
    move.cruise_v = sqrt(cruise_v2)
    move.end_v = sqrt(min(end_v2, cruise_v2))
```

### 6.5 Incremental Processing

For efficiency, the algorithm can process incrementally:
- Maintain a `flush_count` indicating how many moves are finalized
- Only reprocess moves after `flush_count` on each iteration
- This allows streaming processing of G-code

---

## 7. Move Checkers (Axis/Extruder Limiters)

Move checkers apply additional velocity constraints based on axis-specific limits.

### 7.1 Axis Limiter

Limits velocity based on movement along a specific axis:

```python
def check_axis(move, axis, max_velocity, max_accel):
    if move.distance == 0:
        return

    # Calculate ratio: how much is this move in the axis direction
    axis_displacement = |move.delta.xyz · axis|
    ratio = move.distance / axis_displacement

    # Apply scaled limits
    move.limit_speed(max_velocity × ratio, max_accel × ratio)
```

Example: Z-axis limiter with `max_z_velocity=50, max_z_accel=200`
- A pure Z move (ratio=1) is limited to 50 mm/s
- A diagonal move at 45° (ratio≈1.41) can go faster in total

### 7.2 Extruder Limiter

Limits velocity for extrude-only moves:

```python
def check_extruder(move, max_velocity, max_accel):
    if not move.is_extrude_only:
        return

    e_rate = |move.rate.e|
    if move.rate.xy == [0, 0] or e_rate < 0:  # Extrude-only or retract
        inv_extrude_r = 1.0 / e_rate
        move.limit_speed(max_velocity × inv_extrude_r, max_accel × inv_extrude_r)
```

### 7.3 Applying Limits

The `limit_speed` function updates move constraints:

```python
def limit_speed(velocity, acceleration):
    v2 = velocity²
    if v2 < max_cruise_v2:
        max_cruise_v2 = v2

    acceleration = min(self.acceleration, acceleration)
    max_dv2 = 2 × distance × acceleration
    smoothed_dv2 = min(smoothed_dv2, max_dv2)
```

---

## 8. Arc Handling (G2/G3)

Arc moves (G2=clockwise, G3=counter-clockwise) are converted to line segments.

### 8.1 Arc Parameters

```
G2/G3 X{target_x} Y{target_y} Z{target_z} E{extrusion} F{feedrate} I{offset_x} J{offset_y} K{offset_z}
```

- X, Y, Z: Target position
- E: Total extrusion over the arc
- F: Feedrate
- I, J, K: Center offset from current position

### 8.2 Plane Selection

- **G17**: XY plane (default) - uses I, J offsets
- **G18**: XZ plane - uses I, K offsets
- **G19**: YZ plane - uses J, K offsets

### 8.3 Arc to Segments Algorithm

```python
def plan_arc(start, target, center_offset, direction, mm_per_arc_segment):
    # Calculate center position
    center = start + center_offset

    # Calculate radius
    radius = sqrt(center_offset.x² + center_offset.y²)

    # Calculate angular travel
    r_p, r_q = -center_offset.x, -center_offset.y
    rt_alpha = target.alpha - center.alpha
    rt_beta = target.beta - center.beta

    angular_travel = atan2(r_p × rt_beta - r_q × rt_alpha,
                          r_p × rt_alpha + r_q × rt_beta)

    if angular_travel < 0:
        angular_travel += 2π
    if direction == CLOCKWISE:
        angular_travel -= 2π

    # Handle full circle case
    if angular_travel == 0 and start == target:
        angular_travel = 2π

    # Calculate arc length and segments
    linear_travel = target.helical - start.helical  # Z movement
    flat_mm = radius × |angular_travel|
    mm_of_travel = sqrt(flat_mm² + linear_travel²) if linear_travel != 0 else flat_mm

    segments = max(1, floor(mm_of_travel / mm_per_arc_segment))

    # Generate segment points
    theta_per_segment = angular_travel / segments
    linear_per_segment = linear_travel / segments

    for i in 1 to segments:
        cos_ti = cos(i × theta_per_segment)
        sin_ti = sin(i × theta_per_segment)

        new_offset_alpha = -center_offset.x × cos_ti + center_offset.y × sin_ti
        new_offset_beta = -center_offset.x × sin_ti - center_offset.y × cos_ti

        segment_point = [
            center.alpha + new_offset_alpha,
            center.beta + new_offset_beta,
            start.helical + i × linear_per_segment
        ]

        yield segment_point

    yield target  # Final point is exact target
```

### 8.4 Extrusion Distribution

Extrusion is distributed evenly across arc segments:
```
e_per_segment = total_extrusion / num_segments
```

---

## 9. Firmware Retraction (G10/G11)

Firmware retraction generates synthetic moves for retraction and unretraction.

### 9.1 G10 - Retract

Generates 1-2 moves:

1. **Extrusion retract** (if `retract_length > 0`):
   ```
   Move: E = -retract_length at retract_speed
   Kind: "Firmware retract"
   ```

2. **Z hop** (if `lift_z > 0`):
   ```
   Move: Z = +lift_z at current velocity
   Kind: "Firmware retract Z hop"
   ```

State tracking:
```
retracted_state = {
    lifted_z: lift_z,
    unretract_length: retract_length + unretract_extra_length
}
```

### 9.2 G11 - Unretract

Only executes if currently retracted. Generates 1-2 moves:

1. **Extrusion unretract** (if `unretract_length > 0`):
   ```
   Move: E = +unretract_length at unretract_speed
   Kind: "Firmware unretract"
   ```

2. **Z drop** (if `lifted_z > 0`):
   ```
   Move: Z = -lifted_z at current velocity
   Kind: "Firmware unretract Z hop"
   ```

### 9.3 Dynamic Configuration

Via `SET_RETRACTION` command:
```
SET_RETRACTION RETRACT_LENGTH=5 RETRACT_SPEED=50 UNRETRACT_EXTRA_LENGTH=0 UNRETRACT_SPEED=30 LIFT_Z=2
```

---

## 10. Delay Handling

### 10.1 Dwell (G4)

```
G4 P{milliseconds}
```

Adds pause time:
```
delay = P / 1000  // Convert to seconds
// Default: 0.25 seconds if P not specified
```

### 10.2 Indeterminate Delays

These commands add a small placeholder time (0.1s) since actual duration is unknown:

- **G28** (Homing): Actual time depends on probe/endstop position
- **M109** (Wait for hotend temperature)
- **M190** (Wait for bed temperature)
- **M600** (Filament change)
- **TEMPERATURE_WAIT** (Extended command)

### 10.3 Custom Time Addition

Via comment:
```
; ESTIMATOR_ADD_TIME 30 Filament drying
```

Adds 30 seconds with kind "Filament drying"

### 10.4 Initial Move Delay

When a move sequence starts after idle, add 0.25s:
```
if first_move_in_sequence:
    total_time += 0.25
```

---

## 11. Configuration Parameters Reference

### 11.1 Core Printer Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_velocity` | float | 100.0 | Maximum XYZ velocity (mm/s) |
| `max_acceleration` | float | 100.0 | Maximum acceleration (mm/s²) |
| `square_corner_velocity` | float | 5.0 | Maximum velocity at corners (mm/s) |
| `max_accel_to_decel` | float | 50.0 | Deceleration acceleration limit (mm/s²) [legacy] |
| `minimum_cruise_ratio` | float | None | Ratio of cruise velocity maintained (0-1) [newer] |

**Derived values:**
```
junction_deviation = scv² × (√2 - 1) / max_acceleration

accel_to_decel =
    if minimum_cruise_ratio is set:
        max_acceleration × (1 - minimum_cruise_ratio)
    elif max_accel_to_decel is set:
        min(max_accel_to_decel, max_acceleration)
    else:
        min(50.0, max_acceleration)
```

### 11.2 Extruder Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `max_extrude_only_velocity` | float | Max velocity for extrude-only moves (mm/s) |
| `max_extrude_only_accel` | float | Max acceleration for extrude-only moves (mm/s²) |
| `instantaneous_corner_velocity` | float | Max instantaneous E velocity change (mm/s) |

**Usage:**
- `instant_corner_velocity` limits junction velocity when extrusion rate changes
- Used in: `extruder_v2 = (icv / |Δe_rate|)²`

### 11.3 Axis-Specific Limits

| Parameter | Type | Description |
|-----------|------|-------------|
| `max_x_velocity` | float | Optional X-axis velocity limit |
| `max_x_accel` | float | Optional X-axis acceleration limit |
| `max_y_velocity` | float | Optional Y-axis velocity limit |
| `max_y_accel` | float | Optional Y-axis acceleration limit |
| `max_z_velocity` | float | Optional Z-axis velocity limit |
| `max_z_accel` | float | Optional Z-axis acceleration limit |

**Note:** These are only applied if both velocity and acceleration are specified for an axis.

### 11.4 Firmware Retraction Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `retract_length` | float | - | Distance to retract (mm) |
| `retract_speed` | float | - | Retraction speed (mm/s) |
| `unretract_extra_length` | float | 0.0 | Extra prime amount (mm) |
| `unretract_speed` | float | - | Unretraction speed (mm/s) |
| `lift_z` | float | 0.0 | Z-hop distance (mm) |

### 11.5 Arc Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mm_per_arc_segment` | float | None | Arc segment resolution (mm). If not set, G2/G3 are ignored. |

---

## 12. G-code Commands Affecting Timing

### 12.1 Movement Commands

| Command | Effect |
|---------|--------|
| `G0 X Y Z E F` | Rapid move (treated same as G1) |
| `G1 X Y Z E F` | Linear move with optional extrusion |
| `G2 I J K X Y Z E F` | Clockwise arc |
| `G3 I J K X Y Z E F` | Counter-clockwise arc |

### 12.2 Position/Mode Commands

| Command | Effect |
|---------|--------|
| `G17` | Set arc plane to XY |
| `G18` | Set arc plane to XZ |
| `G19` | Set arc plane to YZ |
| `G92 X Y Z E` | Set position without moving |
| `M82` | Set E axis to absolute mode |
| `M83` | Set E axis to relative mode |

### 12.3 Velocity/Acceleration Commands

| Command | Effect |
|---------|--------|
| `M204 S{accel}` | Set max acceleration |
| `M204 P{print} T{travel}` | Set print/travel accel (uses minimum) |
| `SET_VELOCITY_LIMIT VELOCITY={v} ACCEL={a} ACCEL_TO_DECEL={atd} SQUARE_CORNER_VELOCITY={scv}` | Klipper extended command |

### 12.4 Retraction Commands

| Command | Effect |
|---------|--------|
| `G10` | Firmware retract |
| `G11` | Firmware unretract |
| `SET_RETRACTION RETRACT_LENGTH={} ...` | Configure retraction |

### 12.5 Delay Commands

| Command | Effect |
|---------|--------|
| `G4 P{ms}` | Dwell (pause) for P milliseconds |
| `G28` | Home (indeterminate time) |
| `M109 S{temp}` | Wait for hotend (indeterminate) |
| `M190 S{temp}` | Wait for bed (indeterminate) |
| `M600` | Filament change (indeterminate) |

---

## 13. Complete Formulas Reference

### 13.1 Junction Deviation
```
junction_deviation = square_corner_velocity² × (√2 - 1) / max_acceleration
                   = square_corner_velocity² × 0.41421356 / max_acceleration
```

### 13.2 Accel-to-Decel
```
// Modern method (minimum_cruise_ratio)
accel_to_decel = max_acceleration × (1.0 - clamp(minimum_cruise_ratio, 0, 1))

// Legacy method (max_accel_to_decel)
accel_to_decel = min(max_accel_to_decel, max_acceleration)

// Default
accel_to_decel = min(50.0, max_acceleration)
```

### 13.3 Move Velocity Constraints
```
max_cruise_v2 = min(requested_velocity, max_velocity)²
max_dv2 = 2 × distance × acceleration
smoothed_dv2 = 2 × distance × accel_to_decel
```

### 13.4 Junction Angle Components
```
cos_θ = -(current_rate.xyz · previous_rate.xyz)
sin(θ/2) = √(0.5 × (1 - cos_θ))
tan(θ/2) = sin(θ/2) / √(0.5 × (1 + cos_θ))
r = sin(θ/2) / (1 - sin(θ/2))
```

### 13.5 Junction Velocity Constraints
```
centripetal_v2 = 0.5 × distance × tan(θ/2) × acceleration
junction_v2 = r × junction_deviation × acceleration
extruder_v2 = (instant_corner_velocity / |Δe_rate|)²
```

### 13.6 Trapezoidal Profile Times
```
accel_distance = (cruise_v² - start_v²) / (2 × acceleration)
accel_time = accel_distance / ((start_v + cruise_v) / 2)

decel_distance = (cruise_v² - end_v²) / (2 × acceleration)
decel_time = decel_distance / ((cruise_v + end_v) / 2)

cruise_distance = max(0, total_distance - accel_distance - decel_distance)
cruise_time = cruise_distance / cruise_v

total_time = accel_time + cruise_time + decel_time
```

### 13.7 Axis Limiter Scaling
```
axis_ratio = move_distance / |delta.xyz · axis_unit_vector|
limited_velocity = max_axis_velocity × axis_ratio
limited_accel = max_axis_accel × axis_ratio
```

### 13.8 Flow Rate Calculation
```
filament_area = π × (filament_diameter / 2)²
flow_rate = (delta_e / total_time) × filament_area  // mm³/s
```

### 13.9 Line Width Calculation
```
line_width = (e_rate × filament_radius² × π) / layer_height
// where e_rate = delta_e / move_distance
```

---

## 14. Implementation Pseudocode

### 14.1 Main Processing Loop

```python
class PrintTimeEstimator:
    def __init__(self, config):
        self.limits = config.limits
        self.position = [0, 0, 0, 0]  # X, Y, Z, E
        self.velocity = limits.max_velocity
        self.position_modes = [ABSOLUTE, ABSOLUTE, ABSOLUTE, RELATIVE]
        self.move_queue = []
        self.total_time = 0

    def process_gcode_file(self, filepath):
        for line in read_gcode(filepath):
            cmd = parse_gcode(line)
            self.process_command(cmd)

            # Periodically flush to limit memory
            if len(self.move_queue) > 1000:
                self.flush_moves()

        # Final flush
        self.finalize()
        return self.total_time

    def process_command(self, cmd):
        if cmd.type == MOVE:
            self.handle_move(cmd)
        elif cmd.type == SET_VELOCITY:
            self.velocity = cmd.f / 60
        elif cmd.type == M204:
            self.limits.set_acceleration(cmd.accel)
        elif cmd.type == DWELL:
            self.total_time += cmd.duration
        # ... handle other commands

    def handle_move(self, cmd):
        # Calculate new position
        new_pos = self.calculate_new_position(cmd)

        # Create planning move
        move = PlanningMove(self.position, new_pos, self.limits, self.velocity)

        # Apply axis limiters
        for checker in self.limits.move_checkers:
            checker.check(move)

        # Apply junction velocity from previous move
        if self.move_queue:
            prev_move = self.move_queue[-1]
            move.apply_junction(prev_move, self.limits)

        self.move_queue.append(move)
        self.position = new_pos

    def flush_moves(self):
        # Two-pass velocity planning
        self.plan_velocities()

        # Calculate times and accumulate
        for move in self.finalized_moves():
            self.total_time += move.total_time()

    def finalize(self):
        self.flush_moves()
```

### 14.2 Planning Move Creation

```python
class PlanningMove:
    def __init__(self, start, end, limits, requested_velocity):
        self.start = start
        self.end = end

        if self.is_kinematic():
            # XYZ movement
            self.distance = euclidean_distance(start.xyz, end.xyz)
            self.rate = (end - start) / self.distance
            self.velocity = min(requested_velocity, limits.max_velocity)
            self.acceleration = limits.max_acceleration
            self.max_cruise_v2 = self.velocity ** 2
            self.max_dv2 = 2 * self.distance * self.acceleration
            self.smoothed_dv2 = 2 * self.distance * limits.accel_to_decel
        else:
            # Extrude-only
            self.distance = abs(end.e - start.e)
            self.rate = [0, 0, 0, sign(end.e - start.e)]
            self.velocity = requested_velocity
            self.acceleration = INFINITY
            self.max_cruise_v2 = self.velocity ** 2
            self.max_dv2 = INFINITY
            self.smoothed_dv2 = INFINITY

        self.junction_deviation = limits.junction_deviation
        self.max_start_v2 = 0
        self.max_smoothed_v2 = 0
        self.start_v = 0
        self.cruise_v = 0
        self.end_v = 0

    def is_kinematic(self):
        return self.start.xyz != self.end.xyz

    def is_extrude(self):
        return abs(self.end.e - self.start.e) > EPSILON

    def apply_junction(self, prev_move, limits):
        if not self.is_kinematic() or not prev_move.is_kinematic():
            return

        # Calculate angle
        cos_theta = -dot(self.rate.xyz, prev_move.rate.xyz)
        if cos_theta > 0.999999:
            return  # Co-linear

        cos_theta = max(-0.999999, cos_theta)
        sin_theta_d2 = sqrt(0.5 * (1 - cos_theta))
        r = sin_theta_d2 / (1 - sin_theta_d2)
        tan_theta_d2 = sin_theta_d2 / sqrt(0.5 * (1 + cos_theta))

        # Centripetal constraints
        move_centripetal_v2 = 0.5 * self.distance * tan_theta_d2 * self.acceleration
        prev_centripetal_v2 = 0.5 * prev_move.distance * tan_theta_d2 * prev_move.acceleration

        # Extruder constraint
        diff_r = abs(self.rate.e - prev_move.rate.e)
        if diff_r > 0:
            extruder_v2 = (limits.instant_corner_velocity / diff_r) ** 2
        else:
            extruder_v2 = self.max_cruise_v2

        # Final junction velocity
        self.max_start_v2 = min(
            extruder_v2,
            r * self.junction_deviation * self.acceleration,
            r * prev_move.junction_deviation * prev_move.acceleration,
            move_centripetal_v2,
            prev_centripetal_v2,
            self.max_cruise_v2,
            prev_move.max_cruise_v2,
            prev_move.max_start_v2 + prev_move.max_dv2
        )

        self.max_smoothed_v2 = min(
            self.max_start_v2,
            prev_move.max_smoothed_v2 + prev_move.smoothed_dv2
        )

    def total_time(self):
        return self.accel_time() + self.cruise_time() + self.decel_time()

    def accel_time(self):
        dist = self.accel_distance()
        return dist / ((self.start_v + self.cruise_v) / 2)

    def accel_distance(self):
        return (self.cruise_v**2 - self.start_v**2) / (2 * self.acceleration)

    def cruise_time(self):
        return self.cruise_distance() / self.cruise_v

    def cruise_distance(self):
        return max(0, self.distance - self.accel_distance() - self.decel_distance())

    def decel_time(self):
        dist = self.decel_distance()
        return dist / ((self.cruise_v + self.end_v) / 2)

    def decel_distance(self):
        return (self.cruise_v**2 - self.end_v**2) / (2 * self.acceleration)
```

### 14.3 Velocity Planning

```python
def plan_velocities(moves):
    """Two-pass velocity planning algorithm"""

    delayed = []
    next_end_v2 = 0
    next_smoothed_v2 = 0
    peak_cruise_v2 = 0

    # Backward pass
    for move in reversed(moves):
        reachable_start_v2 = next_end_v2 + move.max_dv2
        start_v2 = min(move.max_start_v2, reachable_start_v2)

        reachable_smoothed_v2 = next_smoothed_v2 + move.smoothed_dv2
        smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)

        if smoothed_v2 < reachable_smoothed_v2:
            # This is a constraining point
            if smoothed_v2 + move.smoothed_dv2 > next_smoothed_v2 or delayed:
                peak_cruise_v2 = min(
                    move.max_cruise_v2,
                    (smoothed_v2 + reachable_smoothed_v2) * 0.5
                )

                # Process delayed moves
                if delayed:
                    mc_v2 = peak_cruise_v2
                    for (m, ms_v2, me_v2) in reversed(delayed):
                        mc_v2 = min(mc_v2, ms_v2)
                        m.start_v = sqrt(min(ms_v2, mc_v2))
                        m.cruise_v = sqrt(mc_v2)
                        m.end_v = sqrt(min(me_v2, mc_v2))
                    delayed.clear()

            # Set this move's velocities
            cruise_v2 = min(
                (start_v2 + reachable_start_v2) * 0.5,
                move.max_cruise_v2,
                peak_cruise_v2
            )
            move.start_v = sqrt(min(start_v2, cruise_v2))
            move.cruise_v = sqrt(cruise_v2)
            move.end_v = sqrt(min(next_end_v2, cruise_v2))
        else:
            # Delay this move
            delayed.append((move, start_v2, next_end_v2))

        next_end_v2 = start_v2
        next_smoothed_v2 = smoothed_v2
```

---

## 15. Parameter Usage Reference Table

This section provides a complete reference of every configuration parameter and exactly how it is used in the print time estimation calculations.

### 15.1 Core Motion Parameters

| Parameter | Source | Default | Unit | Exact Usage in Calculations |
|-----------|--------|---------|------|----------------------------|
| `max_velocity` | `[printer]` section | 100.0 | mm/s | **Move creation:** `velocity = min(requested_velocity, max_velocity)` for kinematic moves. Caps the maximum cruise velocity any move can achieve. Used in: `max_cruise_v2 = velocity²` |
| `max_acceleration` | `[printer]` section | 100.0 | mm/s² | **Move creation:** Sets `acceleration` field for kinematic moves. **Velocity constraints:** `max_dv2 = 2 × distance × max_acceleration`. **Junction deviation:** `junction_deviation = scv² × 0.41421356 / max_acceleration`. **Accel-to-decel:** When using `minimum_cruise_ratio`: `accel_to_decel = max_acceleration × (1 - ratio)` |
| `square_corner_velocity` | `[printer]` section | 5.0 | mm/s | **Junction deviation calculation:** `junction_deviation = scv² × (√2 - 1) / max_acceleration`. This derived value is then used in junction velocity: `junction_v2 = r × junction_deviation × acceleration` where `r` is derived from the angle between moves |
| `max_accel_to_decel` | `[printer]` section | 50.0 | mm/s² | **Legacy method for accel_to_decel:** `accel_to_decel = min(max_accel_to_decel, max_acceleration)`. The `accel_to_decel` value is used in: `smoothed_dv2 = 2 × distance × accel_to_decel`. Only used if `minimum_cruise_ratio` is not set |
| `minimum_cruise_ratio` | `[printer]` section | None | ratio (0-1) | **Modern method for accel_to_decel:** `accel_to_decel = max_acceleration × (1.0 - clamp(ratio, 0, 1))`. Takes precedence over `max_accel_to_decel`. A ratio of 0.5 means cruise velocity is maintained at least 50% of the time |

### 15.2 Derived Values (Calculated from Core Parameters)

| Derived Value | Calculation | Usage |
|---------------|-------------|-------|
| `junction_deviation` | `scv² × 0.41421356 / max_acceleration` | Used in junction velocity calculation: `r × junction_deviation × acceleration`. Higher values allow faster cornering |
| `accel_to_decel` | See above (from `minimum_cruise_ratio` or `max_accel_to_decel`) | Used to calculate `smoothed_dv2 = 2 × distance × accel_to_decel`. Controls velocity smoothing between moves |

### 15.3 Extruder Parameters

| Parameter | Source | Default | Unit | Exact Usage in Calculations |
|-----------|--------|---------|------|----------------------------|
| `max_extrude_only_velocity` | `[extruder]` section | - | mm/s | **Extruder limiter check:** For extrude-only moves: `limited_velocity = max_extrude_only_velocity / e_rate`. Applied via `move.limit_speed()` which updates `max_cruise_v2` |
| `max_extrude_only_accel` | `[extruder]` section | - | mm/s² | **Extruder limiter check:** For extrude-only moves: `limited_accel = max_extrude_only_accel / e_rate`. Applied via `move.limit_speed()` which updates `acceleration` and `max_dv2` |
| `instantaneous_corner_velocity` | `[extruder]` section | 1.0 | mm/s | **Extruder junction speed:** When extrusion rate changes between moves: `extruder_v2 = (icv / \|Δe_rate\|)²`. This becomes one of the constraints in `max_start_v2 = min(extruder_v2, ...)` |

### 15.4 Axis-Specific Limit Parameters

| Parameter | Source | Default | Unit | Exact Usage in Calculations |
|-----------|--------|---------|------|----------------------------|
| `max_x_velocity` | `[printer]` section | None | mm/s | **Axis limiter check:** `ratio = move_distance / \|delta.x\|`, then `limited_velocity = max_x_velocity × ratio`. Only applied if both velocity and accel are set |
| `max_x_accel` | `[printer]` section | None | mm/s² | **Axis limiter check:** `limited_accel = max_x_accel × ratio`. Updates move's `acceleration` and recalculates `max_dv2` |
| `max_y_velocity` | `[printer]` section | None | mm/s | **Axis limiter check:** Same as X-axis: `ratio = move_distance / \|delta.y\|`, `limited_velocity = max_y_velocity × ratio` |
| `max_y_accel` | `[printer]` section | None | mm/s² | **Axis limiter check:** `limited_accel = max_y_accel × ratio` |
| `max_z_velocity` | `[printer]` section | None | mm/s | **Axis limiter check:** Same as X/Y: `ratio = move_distance / \|delta.z\|`, `limited_velocity = max_z_velocity × ratio`. Critical for layer changes |
| `max_z_accel` | `[printer]` section | None | mm/s² | **Axis limiter check:** `limited_accel = max_z_accel × ratio`. Z-axis typically has much lower accel (e.g., 200 mm/s²) |

### 15.5 Firmware Retraction Parameters

| Parameter | Source | Default | Unit | Exact Usage in Calculations |
|-----------|--------|---------|------|----------------------------|
| `retract_length` | `[firmware_retraction]` section | - | mm | **G10 retract:** Creates synthetic move with `delta_E = -retract_length`. **G11 unretract:** Combined with `unretract_extra_length` for prime: `unretract_length = retract_length + unretract_extra_length` |
| `retract_speed` | `[firmware_retraction]` section | - | mm/s | **G10 retract:** Sets `velocity = retract_speed` for the retraction move before creating the PlanningMove |
| `unretract_extra_length` | `[firmware_retraction]` section | 0.0 | mm | **G11 unretract:** `unretract_length = retract_length + unretract_extra_length`. Extra material to compensate for oozing |
| `unretract_speed` | `[firmware_retraction]` section | - | mm/s | **G11 unretract:** Sets `velocity = unretract_speed` for the unretraction move |
| `lift_z` | `[firmware_retraction]` section | 0.0 | mm | **G10 retract:** If > 0, creates additional Z move with `delta_Z = +lift_z`. **G11 unretract:** Creates Z move with `delta_Z = -lifted_z` |

### 15.6 Arc Parameters

| Parameter | Source | Default | Unit | Exact Usage in Calculations |
|-----------|--------|---------|------|----------------------------|
| `mm_per_arc_segment` | `[gcode_arcs]` section → `resolution` | None | mm | **Arc segmentation:** `segments = max(1, floor(arc_length / mm_per_arc_segment))`. Each segment becomes a linear move. **If not set:** G2/G3 commands are ignored entirely |

### 15.7 G-code Runtime Parameters

| Parameter | Source | Default | Unit | Exact Usage in Calculations |
|-----------|--------|---------|------|----------------------------|
| `F` (feedrate) | G-code `G0`/`G1` commands | `max_velocity` | mm/min | **Velocity setting:** `velocity = F / 60` (convert to mm/s). Used as `requested_velocity` in move creation: `max_cruise_v2 = min(velocity, max_velocity)²` |
| `P` (dwell) | G-code `G4` command | 250 | ms | **Dwell time:** `delay = P / 1000` seconds. Added directly to total time |

### 15.8 Complete Parameter Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PARAMETER USAGE FLOW                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  PRINTER CONFIG                     MOVE CREATION                            │
│  ┌──────────────────┐              ┌─────────────────────────────────────┐  │
│  │ max_velocity     │──────────────▶│ velocity = min(F/60, max_velocity) │  │
│  │ max_acceleration │──────────────▶│ acceleration = max_acceleration    │  │
│  │                  │              │ max_dv2 = 2 × dist × accel          │  │
│  │ square_corner_   │              │ max_cruise_v2 = velocity²           │  │
│  │ velocity         │──┐           └─────────────────────────────────────┘  │
│  └──────────────────┘  │                          │                          │
│           │            │                          ▼                          │
│           │            │           ┌─────────────────────────────────────┐  │
│           ▼            │           │         AXIS LIMITERS               │  │
│  ┌──────────────────┐  │           │ ┌─────────────────────────────────┐ │  │
│  │ junction_        │  │           │ │ ratio = dist / |axis_component| │ │  │
│  │ deviation =      │◀─┘           │ │ vel = max_axis_vel × ratio      │ │  │
│  │ scv² × 0.414 /   │              │ │ accel = max_axis_accel × ratio  │ │  │
│  │ max_accel        │              │ └─────────────────────────────────┘ │  │
│  └──────────────────┘              └─────────────────────────────────────┘  │
│           │                                       │                          │
│           │                                       ▼                          │
│           │                        ┌─────────────────────────────────────┐  │
│           │                        │       EXTRUDER LIMITER              │  │
│           │                        │ (extrude-only moves)                │  │
│           │                        │ vel = max_extrude_only_vel / e_rate │  │
│           │                        │ accel = max_extrude_only_accel /    │  │
│           │                        │         e_rate                      │  │
│           │                        └─────────────────────────────────────┘  │
│           │                                       │                          │
│           ▼                                       ▼                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    JUNCTION VELOCITY CALCULATION                     │    │
│  │                                                                      │    │
│  │  cos_θ = -dot(rate1.xyz, rate2.xyz)                                 │    │
│  │  sin_θ/2 = sqrt(0.5 × (1 - cos_θ))                                  │    │
│  │  r = sin_θ/2 / (1 - sin_θ/2)                                        │    │
│  │  tan_θ/2 = sin_θ/2 / sqrt(0.5 × (1 + cos_θ))                        │    │
│  │                                                                      │    │
│  │  junction_v2 = r × junction_deviation × acceleration  ◀────────────┼────┤
│  │  centripetal_v2 = 0.5 × distance × tan_θ/2 × acceleration           │    │
│  │  extruder_v2 = (instant_corner_velocity / |Δe_rate|)² ◀─────────────┼────┤
│  │                                                                      │    │
│  │  max_start_v2 = min(junction_v2, centripetal_v2, extruder_v2, ...)  │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│  ┌──────────────────┐                │                                       │
│  │ max_accel_to_    │                ▼                                       │
│  │ decel     OR     │    ┌─────────────────────────────────────────────┐    │
│  │ minimum_cruise_  │───▶│           VELOCITY SMOOTHING                 │    │
│  │ ratio            │    │                                              │    │
│  └──────────────────┘    │  accel_to_decel = max_accel × (1 - ratio)   │    │
│                          │  smoothed_dv2 = 2 × distance × accel_to_decel│    │
│                          │  max_smoothed_v2 = min(max_start_v2,         │    │
│                          │                    prev_smoothed + prev_sdv2)│    │
│                          └─────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    TWO-PASS VELOCITY PLANNING                        │    │
│  │                                                                      │    │
│  │  Resolves: start_v, cruise_v, end_v for each move                   │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                      │                                       │
│                                      ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    TIME CALCULATION (per move)                       │    │
│  │                                                                      │    │
│  │  accel_dist = (cruise_v² - start_v²) / (2 × acceleration)           │    │
│  │  accel_time = accel_dist / ((start_v + cruise_v) / 2)               │    │
│  │                                                                      │    │
│  │  decel_dist = (cruise_v² - end_v²) / (2 × acceleration)             │    │
│  │  decel_time = decel_dist / ((cruise_v + end_v) / 2)                 │    │
│  │                                                                      │    │
│  │  cruise_dist = max(0, distance - accel_dist - decel_dist)           │    │
│  │  cruise_time = cruise_dist / cruise_v                               │    │
│  │                                                                      │    │
│  │  total_time = accel_time + cruise_time + decel_time                 │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 15.9 Parameter Sensitivity Analysis

| Parameter | Impact Level | Effect on Print Time |
|-----------|--------------|---------------------|
| `max_velocity` | **HIGH** | Directly caps cruise velocity. Increasing reduces time for long straight moves |
| `max_acceleration` | **VERY HIGH** | Affects accel/decel phases AND junction velocity calculation. Higher values significantly reduce time on prints with many direction changes |
| `square_corner_velocity` | **HIGH** | Controls cornering speed. Higher values reduce time on prints with many corners/curves. Typical range: 1-15 mm/s |
| `max_accel_to_decel` / `minimum_cruise_ratio` | **MEDIUM** | Affects velocity smoothing. Lower `max_accel_to_decel` (or higher `minimum_cruise_ratio`) = more conservative = longer times |
| `instant_corner_velocity` | **MEDIUM** | Limits junction speed when extrusion rate changes. Most noticeable on prints with variable line widths |
| `max_z_velocity` / `max_z_accel` | **LOW-MEDIUM** | Only affects layer changes and Z-moves. Important for tall prints with many layers |
| `mm_per_arc_segment` | **LOW** | Only affects G2/G3 arcs. Smaller values = more segments = slightly more accurate but slower calculation |

---

## Summary

To accurately estimate Klipper print times in a slicer, you must implement:

1. **Trapezoidal velocity profiles** with proper accel/cruise/decel phases
2. **Junction velocity calculation** considering:
   - Angle between consecutive moves
   - Junction deviation (from square_corner_velocity)
   - Centripetal acceleration limits
   - Extruder rate change limits
3. **Two-pass velocity planning** to resolve constraint conflicts
4. **Move checkers** for axis-specific and extruder-specific limits
5. **Arc conversion** (if supporting G2/G3)
6. **Firmware retraction** handling (if enabled)
7. **Delay handling** for dwells and indeterminate-time operations

The most critical parameters are:
- `max_velocity`, `max_acceleration`
- `square_corner_velocity`
- `max_accel_to_decel` or `minimum_cruise_ratio`
- `instant_corner_velocity` (from extruder config)

All other parameters provide additional accuracy for specific use cases.
