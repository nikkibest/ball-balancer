---
name: physics-logic
description: >
  Use when analysing, debugging, or modifying the ball-balancer's simulation
  state-flow — which physical states are causes vs. effects in each kinematics
  mode (Pose / Servo). Triggers: "state-flow", "kinematics mode", "physics logic",
  "pose mode", "servo mode", "FK order", "IK order", or any question about which
  states are computed first and what drives what. NOT for general physics theory
  questions unrelated to this project.
---

# Physics-Logic Skill — Ball Balancer State-Flow Analysis

This skill encodes the causal ordering rules for the ball-balancer simulation.
The key insight: which states come first and which are derived depends entirely
on `kinematics_mode_`. Any change to physics, kinematics, or GUI that touches
the simulation loop must respect this ordering or the simulation will be
internally inconsistent (states fighting each other across subsystems).

---

## When to Use This Skill

- User asks about state-flow, simulation logic, or why a state is wrong/stale
- User asks to modify the physics loop, kinematics section, or simulator
- User reports that table pose conflicts with servo angles (or vice versa)
- User asks about enabling/disabling GUI sliders based on mode
- User asks which states should be "primary" vs "derived" inputs
- Any mention of Pose mode vs Servo mode ordering

**NOT** for:
- General C++ or ImGui questions unrelated to state-flow
- Build system or rendering questions

---

## Goal

Correctly identify the causal chain of states for the active kinematics mode,
verify code implements that chain without circular dependencies, and make changes
that preserve the clean separation between Pose-driven and Servo-driven flow.

---

## Connectors / Dependencies

- `src/core/application.cpp` — main physics loop and kinematics block
- `src/physics/simulator.cpp` — RK4 integration, `buildTableState()`
- `src/physics/table_kinematics.cpp` — IK and FK implementations
- `src/gui/control_panel.cpp` — mode-aware slider enabling/disabling
- `include/ball_balancer/core/types.hpp` — state/control indices
- `include/ball_balancer/core/application.hpp` — `fk_method_`, `servo_angles_`, `servo_cmd_`
- `include/ball_balancer/gui/control_panel.hpp` — `ArmStatus` struct
- `CLAUDE.md` — "Kinematics State-Flow" section for reference

---

## Canonical State-Flow

### Pose Mode (`KinematicsMode::Pose`)

Primary input: `current_control_` (phi_cmd, theta_cmd, z_cmd from PID or manual sliders)

```
current_control_
    │  RK4 servo lag: dθ/dt = (cmd − θ) / tau  [inside simulator_.step()]
    ▼
VARPHI_X, THETA_Y, Z_TABLE   ← causal truth
    │  BallDynamics (gravity projected onto tilted surface)
    ▼
X, Y, Z_BALL, VX, VY, VZ_BALL
    │  IK (direct closed-form per arm, no lag — pose IS the truth)
    ▼
servo_angles_  (derived, read-only)
```

GUI rules: table command sliders active; servo angle sliders greyed out.

### Servo Mode (`KinematicsMode::Servo`)

Primary input: `servo_cmd_` (alpha_0_cmd, alpha_1_cmd, alpha_2_cmd from GUI sliders)

```
servo_cmd_
    │  first-order lag: dα/dt = (cmd − α) / tau  [inside physics sub-step loop,
    │                                              BEFORE simulator_.step()]
    ▼
servo_angles_   ← causal truth
    │  FK (Newton-Raphson or YouTubeClosedForm — user-selectable via GUI)
    ▼
VARPHI_X, THETA_Y, Z_TABLE   [injected into simulator state before ball step]
    │  BallDynamics (RK4)
    ▼
X, Y, Z_BALL, VX, VY, VZ_BALL
```

GUI rules: servo angle sliders active; table tilt/height and ball state
sliders greyed out when running.

---

## Process

### Step 1 — Read the relevant files

Read `src/core/application.cpp` (physics loop + kinematics block) and the
file being modified. Identify all state writes and reads in the main loop.

- **HITL:** No
- **Reference file:** `src/core/application.cpp`, `src/physics/simulator.cpp`
- **Output:** Mental map of the current state-flow for each mode

### Step 2 — Trace the causal chain

For each mode, answer:
1. What is the primary input (commanded by the user or PID)?
2. Which state is computed first from that input?
3. What is derived from that state?
4. Is there any back-write that creates a circular dependency?

- **HITL:** No
- **Reference file:** `CLAUDE.md` (Kinematics State-Flow section)
- **Output:** Annotated list of state transitions in causal order

### Step 3 — Identify violations

Flag any of these patterns as incorrect:
- Table states integrated by RK4 AND overwritten by FK in the same frame (conflict)
- Servo angles integrated with tau lag in Pose mode (they should be direct IK)
- FK called after ball physics has already used stale table states (wrong order)
- GUI slider enabled for a state that is derived, not primary, in the active mode

- **HITL:** No
- **Output:** List of specific violations with file:line references

### Step 4 — Propose and apply fix

For each violation, propose the minimal change that restores correct ordering:
- In Servo mode: integrate servo angles BEFORE `simulator_.step()`, inject FK table
  pose into simulator state, then pass `physics_control` with zero servo error so
  RK4 does not fight the FK result.
- In Pose mode: IK must be called AFTER `simulator_.step()` completes; set
  `servo_angles_` = IK result directly (no lag integration).
- GUI: use `ImGui::BeginDisabled(condition)` where condition reflects mode + sim state.

- **HITL:** Show proposed changes to user if non-trivial (> 3 files or architectural)
- **Reference file:** `src/core/application.cpp`
- **Output:** Edited files with corrected state-flow

### Step 5 — Verify consistency

After changes, re-read the physics loop and confirm:
1. No state is written twice to contradictory values in the same frame
2. Primary input states update before derived states
3. GUI sliders are disabled for all derived states when simulation is running

- **HITL:** No
- **Output:** Confirmation that the causal chain is clean, or remaining issues listed

---

## Rules

1. **Pose mode**: servo angles are always derived from IK with no lag — never
   integrate them with tau when in Pose mode.
2. **Servo mode**: servo angle integration and FK injection must happen BEFORE
   `simulator_.step()` so ball dynamics sees the correct table pose.
3. **Servo mode**: always pass `physics_control` with table cmd = current FK
   table pose so the simulator's internal servo dynamics produce zero change.
4. **Servo mode**: snap table state back to FK result after each sub-step to
   prevent floating-point drift from accumulating across physics steps.
5. **GUI**: any state that is derived (not primary) in the active mode must
   use `ImGui::BeginDisabled(true)` when the simulation is running.
6. **Never mix** FK-driven table states with RK4 servo dynamics in the same
   frame — this creates contradictory state updates.
7. When the user reports any state "fighting" or "conflicting" — trace the
   state-flow first before touching any code.

---

## Failure Modes

- **User asks about a third mode (e.g. direct velocity control):** Extend the
  canonical state-flow table; follow the same pattern — identify primary input,
  derive all other states from it.
- **FK fails (returns nullopt):** Keep last valid FK result; do NOT fall back to
  RK4 servo dynamics silently — set `ik_failed_ = true` and display in GUI.
- **Circular state read/write detected:** Stop and ask user before proceeding —
  this is an architectural issue requiring explicit resolution.
- **Linter rewrites function signatures:** Verify `.cpp` and `.hpp` are in sync
  after any linter intervention; do NOT assume the linter's output is correct
  without re-reading both files.

---

## Progressive Updates

- When a new kinematics mode is added, add its canonical state-flow entry to
  this skill's "Canonical State-Flow" section.
- When a bug is found that violates these rules, add the anti-pattern to
  "Failure Modes" with a file:line reference as a regression note.
- When the FK method selection is extended, update the Servo mode flow diagram.
