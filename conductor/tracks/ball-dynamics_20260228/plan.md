# Implementation Plan: Full Ball Dynamics Implementation

**Track ID:** ball-dynamics_20260228
**Spec:** [spec.md](./spec.md)
**Created:** 2026-02-28
**Status:** [~] In Progress

---

## Overview

Introduce a dedicated `BallDynamics` class that encapsulates the full 3D ball equations of motion derived from first principles. `Simulator` is refactored to delegate ball acceleration to `BallDynamics` while keeping servo dynamics inline. A `TableState` struct decouples the interface. Contact/free-flight mode switching and bounce are handled inside `BallDynamics`. Finally, the control panel and data manager are updated to expose Z-state and contact mode information.

The work is phased so that each phase is independently buildable and verifiable before proceeding.

---

## Phase 1: `TableState` Struct and `BallDynamics` Class Skeleton

Define the clean interface boundary between the table (driven by `Simulator`) and the ball dynamics.

### Tasks

- [x] **Task 1.1** — Add `TableState` struct to `include/ball_balancer/physics/ball_dynamics.hpp`
  - Fields: `phi`, `theta`, `z_t` (angles and table height)
  - Fields: `phi_dot`, `theta_dot`, `z_t_dot` (velocities)
  - Fields: `phi_ddot`, `theta_ddot`, `z_t_ddot` (accelerations, used for Euler terms and N)
  - All `double`, SI units

- [x] **Task 1.2** — Declare `BallDynamics` class in the same header
  - Constructor: `explicit BallDynamics(const SystemParameters& params)`
  - Public method: `void computeAccelerations(const StateVector& state, const TableState& table, double& ax, double& ay, double& az) const`
  - Public method: `bool isInContact(const StateVector& state, const TableState& table) const`
  - Public method: `double computeNormalForce(const StateVector& state, const TableState& table) const`
  - Public method: `void applyBounce(StateVector& state, const TableState& table) const`
  - Member: `SystemParameters params_`
  - `#pragma once`, namespace `ball_balancer`, camelCase methods, trailing `_` members

- [x] **Task 1.3** — Create stub implementation `src/physics/ball_dynamics.cpp` that compiles cleanly
  - All methods return zero / false placeholders

- [x] **Task 1.4** — Add `ball_dynamics.cpp` to `CMakeLists.txt` source list

### Verification

- [ ] `cmake --build build` succeeds with no errors or warnings
- [ ] All existing tests still pass (`ctest --output-on-failure`)

---

## Phase 2: Implement `BallDynamics` Core Physics

Fill in the full equations of motion for contact mode and free flight.

### Tasks

- [x] **Task 2.1** — Implement `isInContact()`
  - Contact condition: `z_b <= z_t + r + x*theta - y*phi` (small-angle surface height)
  - Return `true` when ball is at or below table surface

- [x] **Task 2.2** — Implement `computeNormalForce()`
  - Simplified explicit form (small lateral velocities):
    ```
    N = m * [g + z_t_ddot + 2*x*theta_ddot - 2*y*phi_ddot
              + phi_dot^2*(z_t+r) + theta_dot^2*(z_t+r)]
    ```
  - Clamp to `N >= 0` (cannot push ball through table)

- [x] **Task 2.3** — Implement `computeAccelerations()` — contact mode
  - Relative velocities:
    ```
    v_rel_x = vx - theta_dot*(z_b - z_t)
    v_rel_y = vy + phi_dot*(z_b - z_t)
    ```
  - Contact accelerations (viscous friction, `b = params_.friction_coeff`):
    ```
    ax = -g*theta  - theta_dot^2*x - theta_ddot*z_b - (b/m)*v_rel_x
    ay =  g*phi    - phi_dot^2*y   - phi_ddot*z_b   - (b/m)*v_rel_y
    az = -g + N/m + phi_ddot*y - theta_ddot*x - phi_dot^2*z_b - theta_dot^2*z_b
    ```
  - Note: `phi` = `THETA_X` (roll), `theta` = `THETA_Y` (pitch) — matches existing axis convention

- [x] **Task 2.4** — Implement `computeAccelerations()` — free flight mode
  - When `!isInContact()` and `N <= 0`:
    ```
    ax = 0,  ay = 0,  az = -g
    ```
  - Delegate to contact or free flight based on `isInContact()` and `computeNormalForce()`

- [x] **Task 2.5** — Implement `applyBounce()`
  - Detect collision: ball is in contact and approaching (`vz_b < vz_table_contact`)
  - Table contact point velocity:
    ```
    vz_contact = z_t_dot + vx*theta + x*theta_dot - vy*phi - y*phi_dot
    ```
  - Apply restitution to normal velocity:
    ```
    vz_after = (1 + e)*vz_contact - e*vz_before
    ```
    where `e = params_.bounce_coeff`
  - `vx`, `vy` remain unchanged

### Verification

- [ ] Unit-test `computeNormalForce()`: flat table (`phi=theta=0`, `z_t_ddot=0`) → `N ≈ m*g`
- [ ] Unit-test `computeAccelerations()` contact: flat table, no rotation → `ax=ay=0`, `az≈0` (ball at rest on surface)
- [ ] Unit-test free flight: `ax=ay=0`, `az=-g`
- [ ] Unit-test `isInContact()`: ball above surface → false; at surface → true

---

## Phase 3: Integrate `BallDynamics` into `Simulator`

Wire the new class into the existing RK4 loop, replacing the old simplified model.

### Tasks

- [x] **Task 3.1** — Add `BallDynamics ball_dynamics_` member to `Simulator` (header)
  - Initialise in constructor initialiser list: `ball_dynamics_(params)`

- [x] **Task 3.2** — Add `TableState buildTableState(const StateVector& state) const` private helper to `Simulator`
  - Extract `phi = state(THETA_X)`, `theta = state(THETA_Y)`, `z_t = state(Z_TABLE)` from state
  - Compute `phi_dot` and `theta_dot` from servo dynamics:
    `phi_dot = (current_control_(THETA_X_CMD) - phi) / tau_servo`
    `theta_dot = (current_control_(THETA_Y_CMD) - theta) / tau_servo`
  - `phi_ddot`, `theta_ddot`: forward-difference from previous step or set to zero (conservative)
  - `z_t_dot = 0`, `z_t_ddot = 0` (table Z stub — no actuation)

- [x] **Task 3.3** — Refactor `Simulator::dynamics()`:
  - Remove old simplified `ax_gravity`/`ay_gravity`/`friction` code
  - Call `TableState table = buildTableState(state)`
  - Call `ball_dynamics_.computeAccelerations(state, table, ax, ay, az)`
  - Assign `dstate(VX) = ax`, `dstate(VY) = ay`, `dstate(VZ_BALL) = az`
  - `dstate(Z_BALL) = state(VZ_BALL)` (position derivative = velocity)
  - Keep servo dynamics unchanged: `dstate(THETA_X/Y) = (cmd - theta) / tau`
  - `dstate(Z_TABLE) = 0` (stub)

- [x] **Task 3.4** — Add post-step contact constraint enforcement in `Simulator::step()` (after RK4, before boundary clamp):
  - Compute `z_surface = z_t + r + x*theta - y*phi`
  - If `ball_dynamics_.isInContact(state_, table)` and `state_(Z_BALL) < z_surface`:
    - Snap: `state_(Z_BALL) = z_surface`
    - Call `ball_dynamics_.applyBounce(state_, table)` if approaching
  - This prevents the ball from sinking through the table between steps

- [x] **Task 3.5** — Update `Simulator::reset()` initial state:
  - Set `Z_BALL = Z_TABLE + ball_radius` (ball resting on flat table) in default reset
  - Update `make_initial_state()` helper in `types.hpp` accordingly

- [x] **Task 3.6** — Remove or update `compute_total_energy()` to include Z kinetic and potential energy

### Verification

- [ ] Build succeeds with no warnings
- [ ] With flat table and no control, ball Z stays at `z_t + r` (resting on surface)
- [ ] With tilt applied, ball rolls in correct X/Y directions (existing axis convention preserved)
- [ ] `Z_BALL` in state is non-zero and consistent with table surface position

---

## Phase 4: `SystemParameters` and `types.hpp` Updates

Small, isolated additions to support the new model parameters.

### Tasks

- [x] **Task 4.1** — Add `viscous_friction_coeff` field to `SystemParameters` (N·s/m)
  - Default value: `0.1` (reasonable for ping-pong ball on table)
  - Add a comment explaining it replaces Coulomb friction for numerical stability
  - `BallDynamics` uses this field as `b` in `(b/m)*v_rel`
  - Update `friction_coeff` comment: mark as legacy / unused by new model

- [x] **Task 4.2** — Verify `bounce_coeff` in `SystemParameters` is used (it is: `params_.bounce_coeff` → restitution `e`). No change needed.

- [x] **Task 4.3** — Update `state_index::Z_BALL` and `VZ_BALL` comments in `types.hpp` to remove the "stub, zero dynamics" annotation — they are now live states.

### Verification

- [ ] `types.hpp` and `SystemParameters` compile cleanly
- [ ] Default `viscous_friction_coeff` produces visibly damped ball motion in simulation

---

## Phase 5: Control Panel — Z State and Contact Mode Display

Extend the system status section to show the new live Z state and contact mode.

### Tasks

- [x] **Task 5.1** — Expose contact mode from `BallDynamics` / `Simulator`
  - Add `bool isInContact() const` to `Simulator` (queries `ball_dynamics_` with current state and table)
  - Add `get_contact_mode()` returning `bool` (or an enum if preferred)

- [x] **Task 5.2** — Update `render_system_status()` in `control_panel.cpp`
  - Add "Ball Z: %.4f m" and "Ball Vz: %.4f m/s" to Ball Position/Velocity sections
  - Add "Mode: Contact / Free Flight" indicator with colour coding (green = contact, yellow = free flight)

- [x] **Task 5.3** — Thread `isInContact()` through `Application` → `ControlPanel::render()`
  - If `ControlPanel::render()` already receives the full state, compute contact mode inside the panel using the state (no API change needed if `TableState` can be reconstructed there)
  - Prefer: pass `bool in_contact` as an additional argument to `ControlPanel::render()` to keep panel logic simple

### Verification

- [ ] Control panel shows non-zero Z and Vz values during simulation
- [ ] Contact mode indicator toggles correctly when ball is bounced or free flight is triggered
- [ ] No crashes or regressions in control panel rendering

---

## Phase 6: Integration Test and Polish

End-to-end verification across the full simulation loop.

### Tasks

- [ ] **Task 6.1** — Run simulation: flat table, ball starts at `z_t + r`, zero control → ball stays still for ≥5 s (gravity equilibrium with contact constraint)

- [ ] **Task 6.2** — Run simulation: apply tilt → ball rolls across table with physically plausible acceleration; Z stays on surface

- [ ] **Task 6.3** — Trigger free flight: set `z_t_ddot` large (or lift table manually via slider) → ball detaches, follows parabola, bounces on return

- [ ] **Task 6.4** — Verify Z plot in plotter shows live `ball_z` tracking table surface during contact and parabolic arc during free flight

- [ ] **Task 6.5** — Review `BallDynamics` implementation against source equations in the derivation document — confirm sign conventions match existing `THETA_X`/`THETA_Y` axis mapping in CLAUDE.md

- [ ] **Task 6.6** — Regression: confirm existing PID + Kalman loop still drives ball to setpoint on flat table (horizontal dynamics unchanged in substance)

---

## Final Verification

- [ ] All acceptance criteria in `spec.md` are met
- [ ] `cmake --build` with no warnings (Release and Debug)
- [ ] `ctest --output-on-failure` — all tests pass
- [ ] Application runs at 100 Hz physics / 60 FPS render with no crashes over a 60 s session
- [ ] Ball visually rests on tilted table surface at all times in contact mode
- [ ] Free flight and bounce are observable in both 3D view and Z plot
- [ ] Control panel shows Z, Vz, and contact mode correctly

---

_Generated by Conductor. Tasks will be marked [~] in progress and [x] complete._
