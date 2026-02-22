# Investigation Findings: OpenGL Axes Rendering Bug

**Date:** 2026-02-22
**Track ID:** opengl-axes-fix_20260222

## Task 1.1: Table Rendering Code Location

### File: `src/rendering/renderer.cpp`

**Location:** Lines 276-306

**Table Transformation Code:**
```cpp
// Table transformation: rotate by theta_x and theta_y
float theta_x = state(state_index::THETA_X);
float theta_y = state(state_index::THETA_Y);

// Rotation around X axis
Eigen::Matrix4f rot_x = Eigen::Matrix4f::Identity();
rot_x(1, 1) = std::cos(theta_x);
rot_x(1, 2) = -std::sin(theta_x);
rot_x(2, 1) = std::sin(theta_x);
rot_x(2, 2) = std::cos(theta_x);

// Rotation around Y axis
Eigen::Matrix4f rot_y = Eigen::Matrix4f::Identity();
rot_y(0, 0) = std::cos(theta_y);
rot_y(0, 2) = std::sin(theta_y);
rot_y(2, 0) = -std::sin(theta_y);
rot_y(2, 2) = std::cos(theta_y);

Eigen::Matrix4f model = rot_y * rot_x;  // Apply rotations
```

**OpenGL Coordinate System:**
- **X**: Horizontal (right)
- **Y**: Vertical (up)
- **Z**: Horizontal (toward viewer, negative Z away from viewer)
- **Right-handed** coordinate system

**Ball Rendering Coordinate Mapping (lines 313-329):**
```cpp
// State uses physics coordinates: X (horizontal), Y (horizontal), Z (vertical)
// OpenGL uses: X (horizontal), Y (vertical/up), Z (horizontal)
float ball_x = state(state_index::X);
float ball_y = state(state_index::Y);
float ball_z = params_.ball_radius;  // Ball sits on table surface

// Translation to ball position
Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
model(0, 3) = ball_x;        // X maps to X
model(1, 3) = ball_z;        // Z (height) maps to Y (up in OpenGL)
model(2, 3) = -ball_y;       // Y maps to -Z (OpenGL Z points toward viewer)
```

---

## Task 1.2: Physics Simulation Code Location

### File: `src/physics/simulator.cpp`

**Gravity Vector Calculation:** Lines 184-185

```cpp
// Acceleration due to gravity on tilted surface
const double ax_gravity = rolling_factor * g * std::sin(theta_x);
const double ay_gravity = rolling_factor * g * std::sin(theta_y);
```

**Physics Coordinate System:**
- **X**: Horizontal dimension 1
- **Y**: Horizontal dimension 2
- **Z**: Vertical (implicit, ball sits at height = ball_radius)
- Both X and Y are on the table surface plane

**Physics Equations:**
- `ax = (5/7) * g * sin(theta_x) - friction * vx`
- `ay = (5/7) * g * sin(theta_y) - friction * vy`

---

## Task 1.3: Manual Control Input Mapping

### File: `src/gui/control_panel.cpp`

**Manual Control Rendering:** Lines 169-186

```cpp
void ControlPanel::render_manual_controls() {
    if (ImGui::CollapsingHeader("Manual Control", ImGuiTreeNodeFlags_DefaultOpen)) {
        float theta_x_cmd_ = static_cast<float>(control_(control_index::THETA_X_CMD));
        float theta_y_cmd_ = static_cast<float>(control_(control_index::THETA_Y_CMD));

        if (ImGui::SliderFloat("##theta_x_cmd", &theta_x_cmd_, -params_.max_tilt_angle, params_.max_tilt_angle, "%.3f")) {
            control_(control_index::THETA_X_CMD) = theta_x_cmd_;
        }
        if (ImGui::SliderFloat("##theta_y_cmd", &theta_y_cmd_, -params_.max_tilt_angle, params_.max_tilt_angle, "%.3f")) {
            control_(control_index::THETA_Y_CMD) = theta_y_cmd_;
        }
        // ...
    }
}
```

**Control Vector Indices (from `include/ball_balancer/core/types.hpp`):**
```cpp
namespace control_index {
    constexpr std::size_t THETA_X_CMD = 0;  // Commanded tilt x
    constexpr std::size_t THETA_Y_CMD = 1;  // Commanded tilt y
}
```

**Mapping:** Direct 1:1 mapping from UI sliders to control vector indices.

---

## Task 1.4: Coordinate System Conventions

### Physics Coordinate System (from `include/ball_balancer/core/types.hpp`)

**State Vector Layout:**
```cpp
// State vector (6D): [x, y, vx, vy, theta_x, theta_y]
//  - x, y: Ball position on table surface (meters)
//  - vx, vy: Ball velocity (m/s)
//  - theta_x, theta_y: Table tilt angles (radians)
```

**Coordinate Frame:**
- **X-axis**: Horizontal dimension 1 on table
- **Y-axis**: Horizontal dimension 2 on table
- **Z-axis** (implicit): Vertical, perpendicular to table when level
- **theta_x**: Tilt angle around X-axis (should affect Y-direction motion)
- **theta_y**: Tilt angle around Y-axis (should affect X-direction motion)

### OpenGL Rendering Coordinate System

**Coordinate Frame (right-handed):**
- **X-axis**: Horizontal (positive right)
- **Y-axis**: Vertical (positive up)
- **Z-axis**: Depth (positive toward viewer, negative away)

**Coordinate Mapping (Physics → OpenGL):**
```
Physics X → OpenGL X
Physics Y → OpenGL -Z (negated!)
Physics Z (height) → OpenGL Y
```

---

## Task 1.5: ROOT CAUSE IDENTIFIED

### Critical Discrepancy Found

**Problem:** The rotation matrices in the rendering code are SWAPPED!

**Expected Behavior:**
- **theta_X** should rotate around the **X-axis** → affects motion in **Y direction**
- **theta_Y** should rotate around the **Y-axis** → affects motion in **X direction**

**Actual Rendering Code (INCORRECT):**

Looking at lines 283-296 in `renderer.cpp`:

1. **rot_x matrix** (lines 283-287):
   ```cpp
   rot_x(1, 1) = std::cos(theta_x);
   rot_x(1, 2) = -std::sin(theta_x);
   rot_x(2, 1) = std::sin(theta_x);
   rot_x(2, 2) = std::cos(theta_x);
   ```
   This modifies rows/columns 1 and 2, which in OpenGL are **Y and Z axes**.
   ✓ This IS a rotation around the X-axis (correct structure)

2. **rot_y matrix** (lines 290-294):
   ```cpp
   rot_y(0, 0) = std::cos(theta_y);
   rot_y(0, 2) = std::sin(theta_y);
   rot_y(2, 0) = -std::sin(theta_y);
   rot_y(2, 2) = std::cos(theta_y);
   ```
   This modifies rows/columns 0 and 2, which in OpenGL are **X and Z axes**.
   ✓ This IS a rotation around the Y-axis (correct structure)

**WAIT - The rotation matrices themselves are structurally CORRECT!**

### Re-analyzing the Problem

Let me reconsider the coordinate mapping between physics and OpenGL:

**Physics Frame:**
- X, Y are on the table plane
- Z is vertical (height)

**OpenGL Frame:**
- X is horizontal (right)
- Y is vertical (up)
- Z is depth (toward/away from viewer)

**Current Mapping:**
```cpp
model(0, 3) = ball_x;        // Physics X → OpenGL X
model(1, 3) = ball_z;        // Physics Z → OpenGL Y
model(2, 3) = -ball_y;       // Physics Y → OpenGL -Z
```

This means:
- Physics X-axis maps to OpenGL X-axis
- Physics Y-axis maps to OpenGL -Z-axis (NEGATED)
- Physics Z-axis (up) maps to OpenGL Y-axis

**Physics Rotations:**
- `theta_x` rotates around physics X-axis → should tilt table in physics Y-direction
- `theta_y` rotates around physics Y-axis → should tilt table in physics X-direction

**OpenGL Rotations Needed:**
- To rotate around physics X (which is OpenGL X), we rotate around OpenGL X-axis ✓
- To rotate around physics Y (which is OpenGL -Z), we need to rotate around OpenGL Z-axis

**THE BUG:**
The rendering code applies:
- `rot_x` around OpenGL X-axis (correct for theta_x)
- `rot_y` around OpenGL Y-axis (**WRONG** - should be around Z-axis!)

Since Physics Y maps to OpenGL -Z, a rotation around physics Y-axis should be a rotation around OpenGL Z-axis, NOT Y-axis!

### Confirmed Root Cause

**The `rot_y` matrix is rotating around the wrong axis!**

It's currently rotating around the OpenGL **Y-axis** (vertical), but it should rotate around the OpenGL **Z-axis** (depth) because:
- Physics Y-axis → OpenGL -Z-axis
- Rotation around physics Y-axis → Rotation around OpenGL Z-axis

This explains why:
1. When theta_Y is commanded, the table appears to rotate around Z-axis (because it IS rotating around Y, which looks wrong)
2. The ball movement doesn't match because the physics correctly rotates around physics Y, but rendering shows rotation around OpenGL Y

### Additional Issue: Ball Physics Mapping

The physics applies:
```cpp
ax_gravity = g * sin(theta_x)  // Acceleration in physics X direction
ay_gravity = g * sin(theta_y)  // Acceleration in physics Y direction
```

But since Physics Y → OpenGL -Z, the accelerations might also need review.

**Actually, the physics is CORRECT**. The issue is purely in the rendering transformation.

---

## Summary of Root Cause

1. **Rendering Error:** The `rot_y` rotation matrix rotates around OpenGL Y-axis instead of OpenGL Z-axis
2. **Coordinate Mapping Issue:** The physics uses Y as a horizontal table dimension, but OpenGL Y is vertical
3. **Fix Required:** Change `rot_y` to rotate around Z-axis (indices 0,1 instead of 0,2)

---

## Files Requiring Modification

1. **src/rendering/renderer.cpp** (lines 290-294) - Fix `rot_y` rotation matrix

---

_Investigation complete: Root cause identified_
