# Implementation Plan: Table Legs Rendering & Kinematics Integration

**Track ID:** table-legs-render_20260310
**Spec:** [spec.md](./spec.md)
**Created:** 2026-03-10
**Status:** [x] Complete

## Overview

Wire `TableKinematics` into the simulation loop and render the 3 arms as line segments. Work proceeds in four phases:

1. **Build integration** — register `table_kinematics.cpp` in CMake and confirm the project still compiles.
2. **Servo dynamics + kinematics wiring** — add `ServoAngles` state and first-order servo dynamics to `Application`, drive leg pose from IK (pose mode) and table pose from FK (servo mode).
3. **Rendering** — add `legs_mesh_` VAO to `Renderer`, compute world-space G/E/T points each frame, upload and draw.
4. **GUI** — add "Arm Mechanism" collapsing section to `ControlPanel` with sliders, servo angle readouts, FK display, and IK failure indicator.

---

## Phase 1: Build Integration

Register the existing kinematics source and verify the project compiles cleanly.

### Tasks

- [x] Task 1.1: Add `src/physics/table_kinematics.cpp` to `BALL_BALANCER_SOURCES` in `CMakeLists.txt` and add `include/ball_balancer/physics/table_kinematics.hpp` to `BALL_BALANCER_HEADERS`.
- [x] Task 1.2: Build in Debug mode; resolve any compiler warnings.

### Verification

- [ ] `cmake --build build` completes without errors or new warnings.

---

## Phase 2: Servo Dynamics & Kinematics Wiring

Add `ServoAngles` state, servo dynamics integration, and the IK/FK mode switch to `Application`.

### Tasks

- [x] Task 2.1: Add to `Application` (in `application.cpp` / `application.hpp`):
  - `TableKinematics kinematics_{params_}` member.
  - `ServoAngles servo_angles_{}` — current arm angles (integrated).
  - `ServoAngles servo_cmd_{}` — commanded arm angles (from IK or GUI sliders).
  - `enum class KinematicsMode { Pose, Servo }` with a `kinematics_mode_` member defaulting to `Pose`.
  - `bool ik_failed_{false}` flag updated each step.

- [x] Task 2.2: In **Pose mode** (default): each control tick, call `kinematics_.inverseKinematics(phi, theta, z_t)` to compute `servo_cmd_`; set `ik_failed_` if `nullopt`. Drive `servo_angles_` toward `servo_cmd_` with first-order dynamics: `alpha[i] += (cmd[i] - alpha[i]) / tau * dt` (Euler, at control rate 100 Hz).

- [x] Task 2.3: In **Servo mode**: `servo_cmd_` is set directly by GUI sliders; call `kinematics_.forwardKinematics(servo_angles_, FKMethod::YouTubeClosedForm)` to get `{phi_fk, theta_fk, z_fk}`; write these into `state_(VARPHI_X)`, `state_(THETA_Y)`, `state_(Z_TABLE)` so the physics and renderer see the FK-derived pose.

- [x] Task 2.4: Expose `servo_angles_`, `ik_failed_`, `kinematics_mode_` to the GUI via getters on `Application` or pass them into `ControlPanel::render()` as a new `ArmStatus` struct.

### Verification

- [ ] In Pose mode: print α_i to stdout for a default flat pose; verify all three are finite and consistent with the nominal height.
- [ ] In Servo mode: manually set α_i = 0.3 rad for one arm; verify table tilts plausibly in the 3D view.

---

## Phase 3: Leg Rendering

Add a `legs_mesh_` VAO to `Renderer` and upload/draw the 6 line segments (2 per arm) each frame.

### Tasks

- [x] Task 3.1: Add to `Renderer`:
  - `std::unique_ptr<VertexArray> legs_mesh_` member.
  - `bool show_legs_{true}` flag.
  - Public `set_show_legs(bool)` and `render_legs(const ServoAngles&, double phi, double theta, double z_t)` method.

- [x] Task 3.2: In `Renderer::initialize()`: create `legs_mesh_` with `GL_DYNAMIC_DRAW` hint — allocate 12 vertices (6 segments × 2 endpoints), initial data can be all-zero.

- [x] Task 3.3: In `render_legs()`:
  - For each arm i (0..2): call `kinematics_.groundPoint(i)`, `kinematics_.elbowPosition(i, alpha[i])`, `kinematics_.tableAttachPoint(i, phi, theta, z_t)`.
  - Map physics → OpenGL coordinates: `GL(x, y, z) = (phys_x, phys_z, phys_y)`.
  - Build 12 `Vertex` entries (G, E per lower link; E, T per upper link) with per-arm colours (arm 0 = cyan, arm 1 = yellow, arm 2 = magenta).
  - Upload via `legs_mesh_->set_data(vertices)` and draw with `glDrawArrays(GL_LINES, 0, 12)` using `grid_shader_`.
  - Skip if `!show_legs_`.

- [x] Task 3.4: Call `renderer_.render_legs(servo_angles_, phi, theta, z_t)` in `Application::render()` after the main `renderer_.render(state_)` call.

- [x] Task 3.5: Add `TableKinematics` reference or copy to `Renderer` so `render_legs` can call geometry helpers. Prefer passing a `const TableKinematics&` parameter or precomputing positions in `Application` and passing `std::array<std::array<float,3>,6>` world positions to avoid coupling.

### Verification

- [ ] Three pairs of coloured line segments appear in the 3D viewport, connecting ground → elbow → table attachment for each arm.
- [ ] Segments update visibly when the table tilts (change varphi_x/theta_y via existing manual sliders).

---

## Phase 4: GUI — Arm Mechanism Panel

Add an "Arm Mechanism" collapsing section to `ControlPanel` with all arm-related controls and readouts.

### Tasks

- [x] Task 4.1: Add a `render_arm_mechanism()` method to `ControlPanel` and call it from `ControlPanel::render()` (after the existing separator for Kalman tuning).

- [x] Task 4.2: Section contents:
  - **Show Legs** checkbox — toggles `renderer.set_show_legs()`.
  - **Mode** radio buttons: "Pose (IK)" and "Servo (FK)".
  - **Geometry parameters** (editable sliders): L1 [0.01–0.20 m], L2 [0.01–0.20 m], Rg [0.05–0.25 m], Rt [0.03–0.15 m], z_nominal [0.05–0.30 m]. Changes update `params_` in place and rebuild the `TableKinematics` object.
  - **Servo Angles** read-only text: α₀, α₁, α₂ in degrees.
  - **IK status**: green "OK" or red "FAIL" indicator.
  - **Servo mode sliders**: when in Servo mode, three sliders for α₀, α₁, α₂ in degrees (convert to/from rad internally).
  - **FK verification**: when in Servo mode, show "FK → φ: X.XX°  θ: X.XX°  z: X.XXm".

- [x] Task 4.3: Thread `KinematicsMode`, `ik_failed_`, computed FK pose, and servo angle write-back through the `ControlPanel::render()` signature (or an `ArmStatus` struct). Update `main_window.cpp` call site.

### Verification

- [ ] "Arm Mechanism" section appears in Control Panel.
- [ ] Toggling Show Legs immediately hides/shows the leg geometry.
- [ ] Switching to Servo mode and dragging α₀ visibly tilts the table and FK display updates.
- [ ] When table is tilted past reachable limit, IK status shows red "FAIL".
- [ ] Changing L1 updates the rendered leg lengths.

---

## Final Verification

- [ ] All acceptance criteria in spec.md are met.
- [ ] Project builds without warnings in Debug and Release.
- [ ] No regressions: ball physics, PID control, Kalman filter, and existing plots continue to work as before.
- [ ] Ready for review.

---

_Generated by Conductor. Tasks will be marked [~] in progress and [x] complete._
