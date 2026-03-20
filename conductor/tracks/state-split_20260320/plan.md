# Implementation Plan: Split StateVector into BallState and TableState

**Track ID:** state-split_20260320
**Spec:** [spec.md](./spec.md)
**Created:** 2026-03-20
**Status:** [x] Complete

## Overview

Work bottom-up: establish typed interfaces in the lowest-level modules first (types,
BallDynamics, Simulator), then propagate changes up through the Kalman filter, then fix
all consumers (GUI, renderer, plotter, application). Each phase leaves the codebase
compiling and runnable before moving on.

---

## Phase 1: Foundation — Conversion Helpers in `types.hpp`

Add the glue functions that let every other phase convert between the two worlds without
duplicating logic.

### Tasks

- [x] Task 1.1: Add `toStateVector(const BallState&, const TableState&) -> StateVector`
      inline helper to `types.hpp`
- [x] Task 1.2: Add `toBallState(const StateVector&) -> BallState` inline helper to
      `types.hpp`
- [x] Task 1.3: Add `toTableState(const StateVector&) -> TableState` inline helper to
      `types.hpp`
- [x] Task 1.4: Add `make_initial_ball_state()` and `make_initial_table_state()` helpers
      (replacing `make_initial_state()`, which can remain temporarily for Kalman use)

### Verification

- [x] `types.hpp` compiles in isolation (`#include` only, no linking needed)
- [x] Conversion round-trips are correct: `toBallState(toStateVector(b, t)) == b`

---

## Phase 2: `BallDynamics` — Typed Inputs, `BallState`-only Outputs

`BallDynamics` already partially uses `TableState`. Complete the migration so all methods
take `BallState` + `TableState` and never touch table fields.

### Tasks

- [x] Task 2.1: Change `isInContact(const StateVector&, const TableState&)` →
      `isInContact(const BallState&, const TableState&)` in header and implementation
- [x] Task 2.2: Change `computeNormalForce(const StateVector&, const TableState&)` →
      `computeNormalForce(const BallState&, const TableState&)`
- [x] Task 2.3: Change `computeAccelerations(const StateVector&, const TableState&, ...)` →
      `computeAccelerations(const BallState&, const TableState&, ...)`
- [x] Task 2.4: Change `applyBounce(StateVector&, const TableState&)` →
      `applyBounce(BallState&, const TableState&)` — modifies `BallState` only
- [x] Task 2.5: Update all internal field accesses from `state(state_index::X)` etc. to
      `ball.x`, `table.phi`, etc.

### Verification

- [x] Project compiles without warnings
- [x] `ball_dynamics_test.cpp` passes (update test helpers to use `BallState` instead of
      `StateVector`)

---

## Phase 3: `Simulator` — Primary State as `BallState` + `TableState`

This is the largest change. The simulator currently owns a single `StateVector state_`.
Split it into `BallState ball_state_` and `TableState table_state_`.

### Tasks

- [x] Task 3.1: Replace `StateVector state_` member with `BallState ball_state_` and
      `TableState table_state_` in `simulator.hpp`
- [x] Task 3.2: Replace `get_state() const -> const StateVector&` with
      `get_ball_state() const -> const BallState&` and
      `get_table_state() const -> const TableState&`
- [x] Task 3.3: Replace `reset(const StateVector&)` with
      `reset(const BallState&, const TableState&)` (or two separate setters)
- [x] Task 3.4: Replace `set_state(const StateVector&)` with
      `set_ball_state(const BallState&)` and `set_table_state(const TableState&)`
- [x] Task 3.5: Update RK4 integration in `simulator.cpp` — `dynamics()` and sub-step
      loop operate on `BallState`; `TableState` is updated separately per the
      Pose/Servo mode rules already in place
- [x] Task 3.6: Remove `buildTableState()` helper (table state is now first-class) or
      rename to an internal conversion if still needed transiently
- [x] Task 3.7: Update `get_measurement()` to read from `ball_state_` directly
- [x] Task 3.8: Update `enforce_constraints()` to operate on `ball_state_`
- [x] Task 3.9: Update `physics_test.cpp` and `integration_test.cpp` test helpers

### Verification

- [x] Project compiles without warnings
- [x] Application runs; ball physics behave correctly (check ImPlot traces)
- [x] `physics_test.cpp` and `integration_test.cpp` pass

---

## Phase 4: `StateEstimator` (Kalman Filter) — Typed Public Interface

Kalman internals may keep `StateVector` + `state_index::` for matrix algebra. Expose only
`BallState` / `TableState` at the public API boundary.

### Tasks

- [x] Task 4.1: Change `get_state() const -> const StateVector&` →
      `get_ball_state() const -> BallState` and
      `get_table_state() const -> TableState` (converting from internal `x_hat_` on the fly)
- [x] Task 4.2: Change `reset(const StateVector&)` →
      `reset(const BallState&, const TableState&)` — pack into `x_hat_` internally using
      `toStateVector()`
- [x] Task 4.3: Keep `predict(const ControlVector&)` and `update(const MeasurementVector&)`
      signatures unchanged (they don't touch `StateVector` externally)
- [x] Task 4.4: Retain `StateVector x_hat_`, `SystemMatrix`, `ControlMatrix`,
      `MeasurementMatrix`, `state_index::` as private implementation details
- [x] Task 4.5: Update `control_test.cpp` to use `get_ball_state()` / `get_table_state()`
      instead of `get_state()` + `state_index::`

### Verification

- [x] Project compiles without warnings
- [x] `control_test.cpp` passes including the axis-mismatch regression test
      (`StateEstimatorAxisMismatch.VXDrivenByThetaXNotThetaY`)

---

## Phase 5: Consumers — GUI, Renderer, Plotter, Application

Update all call sites that previously received a `StateVector` to receive `BallState` and
`TableState` separately.

### Tasks

- [x] Task 5.1: `Renderer::render(const StateVector&)` →
      `render(const BallState&, const TableState&)` — update header, implementation, and
      call site in `main_window.cpp`
- [x] Task 5.2: `RealTimePlotter::add_data_point(const StateVector&, const ControlVector&)`
      → `add_data_point(const BallState&, const TableState&, const ControlVector&)`
- [x] Task 5.3: `ControlPanel::sync_manual_state(const StateVector&)` →
      `sync_manual_state(const BallState&, const TableState&)`; update
      `get_manual_ball_state()` / `get_manual_table_state()` to replace
      `get_manual_state()`; replace `manual_state_` member with `BallState` +
      `TableState` members
- [x] Task 5.4: `ControlPanel::render_system_status(const StateVector&, bool)` →
      `render_system_status(const BallState&, const TableState&, bool)`
- [x] Task 5.5: `MainWindow::render_system_info(const StateVector&, bool)` →
      `render_system_info(const BallState&, const TableState&, bool)`
- [x] Task 5.6: `MainWindow::render_viewport(const StateVector&, Renderer&)` →
      `render_viewport(const BallState&, const TableState&, Renderer&)`
- [x] Task 5.7: Update `application.cpp` main loop: replace all `simulator_.get_state()`
      calls with `get_ball_state()` / `get_table_state()`; update calls to estimator,
      controller, GUI, renderer, and plotter accordingly

### Verification

- [x] Project compiles without warnings
- [x] Application runs end-to-end: 3D scene renders correctly, ImPlot traces update,
      GUI sliders reflect correct values in both Pose and Servo mode
- [x] No visual regressions (ball position, table tilt, axis labels all correct)

---

## Phase 6: Cleanup

Remove now-dead code and update documentation.

### Tasks

- [x] Task 6.1: Remove `make_initial_state()` helper from `types.hpp` if no longer used
- [x] Task 6.2: Remove `state_index::` namespace from `types.hpp` if Kalman no longer
      needs it (keep if `StateVector` is retained in Kalman internals)
- [x] Task 6.3: Update `StateVector` typedef comment to note it is a Kalman-internal
      type (or remove the typedef entirely if unused)
- [x] Task 6.4: Update `CLAUDE.md` state vector table and Architecture section to
      reflect `BallState` / `TableState` as primary types
- [x] Task 6.5: Run full test suite (`ctest --output-on-failure`) and confirm all pass
- [x] Task 6.6: Attempt web build (`./build_web.sh`) to confirm Emscripten target still
      compiles

### Verification

- [x] `grep -r "StateVector" src/ include/` returns only Kalman-internal usages (if any)
- [x] `grep -r "state_index::" src/ include/` returns only Kalman-internal usages (if any)
- [x] All tests pass; web build succeeds

---

## Final Verification

- [x] All acceptance criteria in `spec.md` are met
- [x] Tests passing (`ctest --output-on-failure`)
- [x] Application runs at 60 FPS with correct physics in both Pose and Servo mode
- [x] Web build compiles and runs in browser
- [x] `CLAUDE.md` updated

---

_Generated by Conductor. Tasks will be marked [~] in progress and [x] complete._
