# Specification: Geometry-Based Forward Kinematics (β-solver)

**Track ID:** geom-fk_20260321
**Type:** Feature
**Created:** 2026-03-21
**Status:** Draft

---

## Summary

Replace the two existing FK implementations (Newton-Raphson on `(φ,θ,z_t)` and the
YouTube closed-form) with a new geometry-based FK that solves for the three upper-link
angles `β₁, β₂, β₃` directly from the rigid-body distance constraints, then extracts
the table pose analytically. Additionally, compute table-state time derivatives
(velocities and accelerations) via finite differencing across FK steps, so that
Servo mode feeds physically correct `phi_dot`, `theta_dot`, `z_t_dot` (and double
derivatives) into the ball dynamics.

---

## Context

The Ball Balancing Table uses a 3-arm parallel linkage. Each arm consists of:
- Lower link `L1` (servo-driven, angle `αᵢ` from straight-down)
- Upper link `L2` (passive, ball-and-socket joints at both ends)

Ground points `Gᵢ` are fixed at radius `Rg`. Table attachment points `Tᵢ` are
constrained to lie at radius `Rt` from the table centre on the rigid disc.

**Current FK problems:**
- `fkNewtonRaphson` iterates on `(φ, θ, z_t)` using NR on `||Tᵢ(φ,θ,z_t) − Eᵢ(αᵢ)||² − L2² = 0`. This couples all three arms through a pose parameterisation that can become ill-conditioned near large tilts, and the warm-start comes from the YouTube solve (not from the previous β state).
- `fkYouTube` assumes `Tᵢ` lies directly above `Gᵢ` radially (only valid for small tilt). The pose inversion uses small-angle formulae.
- Neither method populates `phi_dot`, `theta_dot`, `z_t_dot`, `phi_ddot`, `theta_ddot`, `z_t_ddot` in `TableState`. In Servo mode the ball dynamics receive zero angular velocity — causing incorrect friction and centripetal terms.

---

## User Story

As a developer running the simulation in Servo mode, I want the FK to compute
the table pose and its time derivatives (velocities and accelerations) from the
geometry of the linkage, so that ball dynamics are physically accurate and the
simulation matches the real physical system.

---

## Acceptance Criteria

- [ ] New FK method `FKMethod::GeometryBased` added to the enum.
- [ ] Solves `β₁, β₂, β₃` via Newton-Raphson on the three rigid-body chord-length constraint equations `‖Tᵢ − Tⱼ‖² = 3Rt²` (decoupled: each βᵢ appears in exactly two equations).
- [ ] Warm-starts β from `ElbowAngles` (previous timestep), converges in ≤ 5 iterations at 100 Hz.
- [ ] Extracts `(z_t, φ, θ)` from the three arm heights `hᵢ` using the exact closed-form formulae from the derivation (`arcsin` form, not small-angle).
- [ ] Populates `TableState.phi_dot`, `theta_dot`, `z_t_dot` via finite difference of FK pose across consecutive steps.
- [ ] Populates `TableState.phi_ddot`, `theta_ddot`, `z_t_ddot` via finite difference of the velocities.
- [ ] Ball dynamics in Servo mode are visually more stable/physical (phi_dot feeds friction and centripetal correctly).
- [ ] Existing IK is unaffected.
- [ ] Old FK methods remain available (no removal — for A/B comparison).
- [ ] Code compiles on desktop; application runs without crashes.

---

## Mathematical Specification

### Alpha convention note
The existing codebase measures α from straight-down (`α=0` → link vertical,
`α=π/2` → link horizontal). In the derivation notes α is measured from horizontal.
The code convention gives:
```
Eᵢ = [( Rg + L1·sin αᵢ)·cos ψᵢ,  (Rg + L1·sin αᵢ)·sin ψᵢ,  L1·sin αᵢ]
```
Wait — current code (`elbowPosition`):
```
rEff = Rg + L1 * cos(alpha)    // α from straight-down → horizontal reach = L1·sinα??
```
Actually: `α` from straight-down means:
- `cos α` = cosine of angle from -Z = horizontal component → `E_r = Rg + L1·cos(α)` ✓
- `sin α` = vertical component → `E_z = L1·sin(α)`  (positive = up from ground) ✓

So in code convention: `Aᵢ = Rg + L1·cos(αᵢ)` and `Bᵢ = L1·sin(αᵢ)`.

This matches the derivation note's Step 1 with the substitution `cos↔sin` vs the
"from horizontal" convention. The constraint equations and their solution are identical;
just use the code's `cos/sin` mapping.

### Constraint equations (per the derivation)
Per-arm shorthand:
```
Aᵢ = Rg + L1·cos(αᵢ)    (known radial offset at elbow)
Bᵢ = L1·sin(αᵢ)          (known height of elbow)
ρᵢ = Aᵢ − L2·cos(βᵢ)    (net radial reach of Tᵢ from centre)
hᵢ = Bᵢ + L2·sin(βᵢ)    (height of table attachment point Tᵢ)
```

Three chord-length constraints (all pairs separated by ψ = 2π/3 → cos(Δψ) = -1/2):
```
f₀: ρ₀² + ρ₁² + ρ₀ρ₁ + (h₀−h₁)² − 3Rt² = 0
f₁: ρ₀² + ρ₂² + ρ₀ρ₂ + (h₀−h₂)² − 3Rt² = 0
f₂: ρ₁² + ρ₂² + ρ₁ρ₂ + (h₁−h₂)² − 3Rt² = 0
```

### Jacobian of the residuals w.r.t. β = (β₀, β₁, β₂)
```
∂ρᵢ/∂βᵢ = L2·sin(βᵢ)
∂hᵢ/∂βᵢ = L2·cos(βᵢ)
```
Each fₖ depends on at most two βᵢ so the 3×3 Jacobian is sparse.

### Pose extraction from heights
Once `β₀, β₁, β₂` are known, compute `hᵢ = Bᵢ + L2·sin(βᵢ)` then:
```
z_t = (h₀ + h₁ + h₂) / 3

φ = arcsin( (h₁ − h₂) / (√3·Rt) )

θ = arcsin( (h₁ + h₂ − 2·h₀) / (3·Rt·cos φ) )
```
(These are exact, not small-angle approximations.)

**Sign convention check with existing code:** `fkYouTube` uses:
```
thet = −(z[1] + z[2] − 2·z[0]) / (3·Rt)
ph   = −(z[1] − z[2]) / (√3·Rt)
```
The negation is due to the renderer using `Ry(-θ)·Rx(-φ)`. The new FK must apply
the same sign convention when extracting `φ, θ` to remain consistent.

### Table state time derivatives
Maintain a one-step history of FK pose inside `Application` (or pass previous FK
result in). At each FK call with timestep `dt`:
```
phi_dot   = (φ_new − φ_prev) / dt
theta_dot = (θ_new − θ_prev) / dt
z_t_dot   = (z_t_new − z_t_prev) / dt

phi_ddot   = (phi_dot_new − phi_dot_prev) / dt
theta_ddot = (theta_dot_new − theta_dot_prev) / dt
z_t_ddot   = (z_t_dot_new − z_t_dot_prev) / dt
```
These are injected into `TableState` before `simulator_->step()` in Servo mode,
so ball dynamics receive correct angular velocity and acceleration.

---

## Dependencies

- `src/physics/table_kinematics.cpp` / `.hpp` — new FK method added here
- `src/core/application.cpp` — injects `phi_dot` etc. into `TableState` in servo loop
- `include/ball_balancer/core/types.hpp` — `TableState` already has dot/ddot fields ✓
- `src/gui/control_panel.cpp` — add new enum option to FK method selector

---

## Out of Scope

- Modifying IK (stays identical).
- Removing old FK methods.
- Changing ball dynamics equations.
- Controller or Kalman filter changes.
- Web/Emscripten build changes (though it should compile there too).

---

## Technical Notes

- The 3×3 Jacobian for the β-NR solve is **analytically derived** (no finite-difference Jacobian needed).
- Use Cramer's rule (existing `solve3x3`) for the Newton step — no new linear algebra needed.
- The FK should store `ElbowAngles` beta as the warm-start for the next timestep — this already exists in `FKResult.elbow`.
- If NR fails to converge, fall back to the previous `ElbowAngles` (keep last known good β).
- The finite-difference derivative computation should be guarded against `dt ≈ 0` (e.g., first frame).
- Keep a `prev_fk_pose_` and `prev_fk_vel_` struct in `Application` to support the two-step difference for `ddot`.

---

_Generated by Conductor. Review and edit as needed._
