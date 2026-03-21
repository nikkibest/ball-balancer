# Implementation Plan: Geometry-Based Forward Kinematics (β-solver)

**Track ID:** geom-fk_20260321
**Spec:** [spec.md](./spec.md)
**Created:** 2026-03-21
**Status:** [x] Complete

---

## Overview

Add `FKMethod::GeometryBased` to `TableKinematics` implementing a Newton-Raphson
solve on the three β-angle constraint equations. Extract table pose via the exact
arcsin formulae. In `application.cpp`, inject the computed `phi_dot`, `theta_dot`,
`z_t_dot`, `phi_ddot`, `theta_ddot`, `z_t_ddot` into `TableState` in Servo mode.
Finally expose the new method in the GUI dropdown.

---

## Phase 1: Add `FKMethod::GeometryBased` enum value and declare the new solver

**Goal:** Extend the header without breaking existing code. All callers compile.

### Tasks

- [x] Task 1.1: In `include/ball_balancer/physics/table_kinematics.hpp`, add
  `GeometryBased` to `FKMethod` enum with a descriptive comment explaining that
  it solves for `β₁,β₂,β₃` directly via NR on the chord-length constraints.
- [x] Task 1.2: Add private method declaration:
  ```cpp
  std::optional<std::array<double,3>> fkGeometryBased(
      const ServoAngles& servos,
      const ElbowAngles& prevBeta) const;
  ```
  Add companion private method:
  ```cpp
  std::array<double,3> betaResiduals(
      const std::array<double,3>& beta,
      const std::array<double,3>& A,
      const std::array<double,3>& B) const;
  std::array<std::array<double,3>,3> betaJacobian(
      const std::array<double,3>& beta,
      const std::array<double,3>& A,
      const std::array<double,3>& B) const;
  ```
- [x] Task 1.3: In `forwardKinematics()`, add a `case FKMethod::GeometryBased:`
  branch that calls `fkGeometryBased(servos, prev)`. After the switch, the
  β → pose extraction will set `result.elbow.beta[i]` (Task 2 will fill in the
  body; for now return `nullopt` from `fkGeometryBased` as a stub).

### Verification

- [x] Desktop build compiles without errors or warnings.
- [x] Existing FK methods still work at runtime (no regression).

---

## Phase 2: Implement the β Newton-Raphson solver

**Goal:** `fkGeometryBased` solves for β and extracts pose correctly.

### Tasks

- [x] Task 2.1: Implement `fkGeometryBased` in `table_kinematics.cpp`.

  **Pre-compute per-arm constants** (in code's α convention):
  ```cpp
  // α measured from straight-down: cos(α)=horizontal, sin(α)=vertical
  A[i] = Rg_ + L1_ * std::cos(servos.alpha[i]);   // radial reach of elbow
  B[i] = L1_ * std::sin(servos.alpha[i]);           // height of elbow
  ```

  **NR warm-start** from `prevBeta.beta[i]`; use `0.0` if first call
  (or derive a scalar β that satisfies the symmetric case `ρ = Rt`).

  **Iterate NR** (max `NR_MAX_ITER`, tol `NR_TOL`):
  ```
  β^(k+1) = β^(k) − J⁻¹·f(β^(k))
  ```
  using `betaResiduals`, `betaJacobian`, and existing `solve3x3`.

- [x] Task 2.2: Implement `betaResiduals`:
  ```
  ρᵢ = Aᵢ − L2·cos(βᵢ)
  hᵢ = Bᵢ + L2·sin(βᵢ)
  f₀ = ρ₀²+ρ₁²+ρ₀ρ₁+(h₀−h₁)²−3Rt²
  f₁ = ρ₀²+ρ₂²+ρ₀ρ₂+(h₀−h₂)²−3Rt²
  f₂ = ρ₁²+ρ₂²+ρ₁ρ₂+(h₁−h₂)²−3Rt²
  ```

- [x] Task 2.3: Implement `betaJacobian`. Let `sᵢ = L2·sin(βᵢ)` (=`∂ρᵢ/∂βᵢ` with
  sign flip) and `cᵢ = L2·cos(βᵢ)` (=`∂hᵢ/∂βᵢ`). Expanding `∂f₀/∂β₀`:
  ```
  ∂f₀/∂β₀ = 2ρ₀·(L2 sin β₀) + ρ₁·(L2 sin β₀) + 2(h₀−h₁)·(L2 cos β₀)
           = L2·sin β₀·(2ρ₀+ρ₁) + L2·cos β₀·2(h₀−h₁)
  ∂f₀/∂β₁ = 2ρ₁·(L2 sin β₁) + ρ₀·(L2 sin β₁) − 2(h₀−h₁)·(L2 cos β₁)
           = L2·sin β₁·(2ρ₁+ρ₀) − L2·cos β₁·2(h₀−h₁)
  ∂f₀/∂β₂ = 0
  ```
  Fill the full 3×3 similarly. Off-diagonal zeros appear because f₂ involves
  only (β₁,β₂), etc.

- [x] Task 2.4: After NR convergence, compute heights and extract pose:
  ```cpp
  const double h[3] = { B[0]+L2_*sin(beta[0]),
                         B[1]+L2_*sin(beta[1]),
                         B[2]+L2_*sin(beta[2]) };
  const double z_t = (h[0]+h[1]+h[2]) / 3.0;
  // Sign convention: renderer uses Ry(-θ)·Rx(-φ), so negate as in fkYouTube
  const double phi   = -std::asin(std::clamp((h[1]-h[2])/(sqrt(3.)*Rt_), -1.,1.));
  const double cosPhi = std::cos(phi);
  const double theta = -std::asin(std::clamp(
                          (h[1]+h[2]-2.*h[0])/(3.*Rt_*cosPhi), -1.,1.));
  ```
  Return `{{ phi, theta, z_t }}`.

- [x] Task 2.5: In `forwardKinematics()`, after `fkGeometryBased` returns a valid
  pose, set `result.elbow.beta[i]` directly from the solved β (no need to
  back-compute from `atan2` since we already have them). Store solved β in
  a local variable and pass back through `FKResult.elbow`.

### Verification

- [x] With all servos at identical α, `φ=0`, `θ=0`, `z_t = B + L2·sin(β)` where
  `A − L2·cos(β) = Rt` — verify numerically in a debug print or small test.
- [x] With servos slightly asymmetric (e.g., α₀ increased by 0.1 rad), the
  resulting `θ` is non-zero and in the expected direction.
- [x] NR converges in ≤ 5 iterations when warm-started from the previous β.
- [x] Desktop build: application runs in Servo mode with `GeometryBased` selected,
  no crash, arm rendering looks correct.

---

## Phase 3: Populate table-state time derivatives in application loop

**Goal:** Servo mode injects correct `phi_dot`, `theta_dot`, `z_t_dot` (and ddot)
into `TableState` before ball dynamics step.

### Tasks

- [x] Task 3.1: In `include/ball_balancer/core/application.hpp`, add two private
  members to `Application`:
  ```cpp
  struct FKPose { double phi{}, theta{}, z_t{}; };
  struct FKVel  { double phi_dot{}, theta_dot{}, z_t_dot{}; };
  FKPose prev_fk_pose_;
  FKVel  prev_fk_vel_;
  bool   fk_prev_valid_{false};
  ```

- [x] Task 3.2: In `application.cpp`, in the Servo-mode block inside the physics
  sub-step loop (around line 344), after successfully calling `forwardKinematics`:
  ```cpp
  if (fkResult && fk_prev_valid_) {
      const double inv_dt = 1.0 / physics_dt;
      true_table.phi_dot   = (fkResult->phi   - prev_fk_pose_.phi)   * inv_dt;
      true_table.theta_dot = (fkResult->theta - prev_fk_pose_.theta) * inv_dt;
      true_table.z_t_dot   = (fkResult->z_t   - prev_fk_pose_.z_t)   * inv_dt;

      true_table.phi_ddot   = (true_table.phi_dot   - prev_fk_vel_.phi_dot)   * inv_dt;
      true_table.theta_ddot = (true_table.theta_dot - prev_fk_vel_.theta_dot) * inv_dt;
      true_table.z_t_ddot   = (true_table.z_t_dot   - prev_fk_vel_.z_t_dot)   * inv_dt;
  }
  if (fkResult) {
      prev_fk_vel_  = { true_table.phi_dot, true_table.theta_dot, true_table.z_t_dot };
      prev_fk_pose_ = { fkResult->phi, fkResult->theta, fkResult->z_t };
      fk_prev_valid_ = true;
  }
  ```
  Call `simulator_->set_table_state(true_table)` after this block.

- [x] Task 3.3: Reset `fk_prev_valid_ = false` whenever simulation is reset or
  paused, so the first step after resume doesn't use stale derivatives.
  Find the existing reset path in `application.cpp` and add the reset there.

### Verification

- [x] In Servo mode with a non-zero servo command, inspect ball trajectory —
  it should now curve due to correct `theta_dot`/`phi_dot` in friction/centripetal.
- [x] No NaN or inf in `phi_dot` etc. on first frame after reset.
- [x] Existing Pose mode is unaffected (no change to that branch).

---

## Phase 4: GUI — expose `GeometryBased` in FK method selector

**Goal:** User can select the new FK method from the GUI dropdown.

### Tasks

- [x] Task 4.1: In `src/gui/control_panel.cpp`, find the FK method radio button /
  combo block (around line 593). Add a third option:
  ```cpp
  // Extend the existing int-based selector to 3 options
  if (ImGui::RadioButton("Newton-Raphson (pose)", &fkInt, 0))
      arm_status.fk_method = FKMethod::NewtonRaphson;
  if (ImGui::RadioButton("YouTube closed-form",  &fkInt, 1))
      arm_status.fk_method = FKMethod::YouTubeClosedForm;
  if (ImGui::RadioButton("Geometry β-solver",    &fkInt, 2))
      arm_status.fk_method = FKMethod::GeometryBased;
  ```
  Update the `fkInt` initialisation to handle the new value (`2`).

- [x] Task 4.2: Make `GeometryBased` the **default** `fk_method_` in
  `Application` (change the member initialiser in `.hpp` or the constructor).

### Verification

- [x] GUI dropdown shows three options; selecting each switches the FK method.
- [x] `GeometryBased` is selected by default when application starts.
- [x] Switching between methods at runtime does not crash (β warm-start may
  briefly be stale — this is acceptable).

---

## Final Verification

- [x] All acceptance criteria from spec.md are met.
- [x] Desktop build: `cmake --build . -j$(nproc)` succeeds with zero warnings.
- [x] Application runs in Servo mode; ball dynamics look physically reasonable.
- [x] Symmetric servo input (`α₁=α₂=α₃`) → flat table (`φ≈0`, `θ≈0`) ✓.
- [x] Asymmetric input → table tilts in expected direction ✓.
- [x] Old FK methods still selectable and functional ✓.
- [x] No regressions in Pose mode.

---

_Generated by Conductor. Tasks will be marked [~] in progress and [x] complete._
