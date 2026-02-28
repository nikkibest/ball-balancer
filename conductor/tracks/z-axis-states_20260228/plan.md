# Implementation Plan: Z-Axis State Extension

**Track ID:** z-axis-states_20260228
**Spec:** [spec.md](./spec.md)
**Created:** 2026-02-28
**Status:** [~] In Progress

## Overview

Extend the ball-balancing simulation to include a Z (vertical) translatory degree of freedom for both the ball (`z_ball`, `vz_ball`) and the table (`z_table`). The work proceeds in four phases:

1. **State Space** — Extend `StateVector` and all index/alias machinery
2. **Physics Stub** — Zero out z-dynamics so the simulator stays consistent
3. **GUI** — Add manual-state sliders for all 6 user-facing states
4. **Rendering & Plotting** — Update the 3D renderer (axis labels + z-positions) and the real-time plotter

Physics, control, and estimation logic are explicitly left unchanged.

---

## Phase 1: State Space Extension

Extend the core type definitions to accommodate `z_ball`, `vz_ball`, and `z_table`.

### Tasks

- [x] Task 1.1: In `include/ball_balancer/core/types.hpp`, change `StateVector` from `Eigen::Matrix<double, 6, 1>` to `Eigen::Matrix<double, 9, 1>` (or introduce a typedef change)
- [x] Task 1.2: Add index constants to `state_index` namespace: `Z_BALL`, `VZ_BALL`, `Z_TABLE` (following existing pattern for `X`, `Y`, `VX`, `VY`, `THETA_X`, `THETA_Y`)
- [x] Task 1.3: Update any helper functions or factory functions in `types.hpp`/`types.cpp` that construct or validate `StateVector` (e.g., size-dependent helpers)
- [x] Task 1.4: Update `SystemParameters` or related structs if they reference state dimension explicitly (e.g., system matrices A, B, C, D sizes — keep them unchanged or document that they remain 2D for now)
- [x] Task 1.5: Ensure `MeasurementVector` and `ControlVector` are unchanged (they stay 2D)

### Verification

- [x] Project compiles without errors after type changes
- [x] Existing state-index usages (`state_index::X`, etc.) still resolve correctly

---

## Phase 2: Physics Simulator Stub

Update the simulator to handle the expanded state vector without introducing dynamics for the new z-states.

### Tasks

- [ ] Task 2.1: In `src/physics/simulator.cpp`, update the RK4 derivative function to produce zero derivatives for `Z_BALL`, `VZ_BALL`, and `Z_TABLE` indices (i.e., z-positions stay wherever set manually, z-velocity stays zero)
- [ ] Task 2.2: Ensure `Simulator::get_state()` and `Simulator::get_measurement()` compile and return the extended 9D vector (measurement remains 2D — only x, y positions)
- [ ] Task 2.3: Update any boundary enforcement code that clamps ball position to also respect (or simply ignore) the z-axis in this track

### Verification

- [ ] `Simulator::step()` compiles and runs with the 9D state
- [ ] Z-states remain at their initial value through multiple simulation steps (confirm with a debug print or unit test assertion)
- [ ] Application launches and simulation runs without crash

---

## Phase 3: ImGui GUI — Manual State Sliders

Add sliders in the control panel for all six user-adjustable states, enabled only when the simulation is paused.

### Tasks

- [ ] Task 3.1: In `src/gui/control_panel.cpp`, add a "Manual State" section (collapsible `ImGui::CollapsingHeader`) below the existing simulation controls
- [ ] Task 3.2: Add `ImGui::SliderFloat` (or `SliderDouble`) for: `x` ∈ [-0.25, 0.25] m, `y` ∈ [-0.25, 0.25] m, `z_ball` ∈ [-0.1, 0.5] m
- [ ] Task 3.3: Add sliders for: `theta_x` ∈ [-0.174, 0.174] rad, `theta_y` ∈ [-0.174, 0.174] rad, `z_table` ∈ [-0.1, 0.2] m
- [ ] Task 3.4: Disable all sliders (use `ImGui::BeginDisabled` / `ImGui::EndDisabled`) when the simulation is running; enable only when paused
- [ ] Task 3.5: Wire slider values back to the simulator state via a `Simulator::set_state()` method (add this method to `simulator.hpp/.cpp` if not present) or via the application's `current_state_` field

### Verification

- [ ] Sliders appear in the GUI and are greyed out during simulation
- [ ] When simulation is paused, moving a slider updates the value and the 3D rendering reflects the change immediately
- [ ] Slider ranges are physically sensible (no crashes at extremes)

---

## Phase 4: Rendering and Plotting

Update the 3D renderer to use the new z-states and add axis labels. Extend the plotter to display all states.

### Tasks

#### Renderer
- [ ] Task 4.1: In `src/rendering/renderer.cpp`, update ball position transform to use `state[state_index::Z_BALL]` for the Y (up) component (or Z, depending on OpenGL coordinate convention — verify against current up-axis)
- [ ] Task 4.2: Update table position transform to translate by `state[state_index::Z_TABLE]` along the vertical axis
- [ ] Task 4.3: Add axis label rendering for "X", "Y", "Z": implement as small text rendered via ImGui overlay (project 3D tip positions to screen coords using the MVP matrix) or as 3D geometry quads
- [ ] Task 4.4: Verify existing table roll/pitch (theta\_x, theta\_y) rendering is unaffected

#### Plotter / Data Manager
- [ ] Task 4.5: In `include/ball_balancer/visualization/data_manager.hpp`, add fields for `z_ball`, `vz_ball`, `z_table` to the data record struct
- [ ] Task 4.6: In `src/visualization/data_manager.cpp`, update `push()` to store the new state fields
- [ ] Task 4.7: In `src/visualization/real_time_plotter.cpp`, add ImPlot lines/plots for `z_ball` and `z_table` (e.g., in a new "Z Position" subplot, or add as lines to the existing position plot)
- [ ] Task 4.8: Update `DataManager` ring buffer arrays (or `std::array` fields) to include the new data channels

### Verification

- [ ] Moving `z_ball` slider moves the ball up/down in the 3D view
- [ ] Moving `z_table` slider moves the table platform up/down in the 3D view
- [ ] "X", "Y", "Z" labels appear at the correct axes in the 3D viewport
- [ ] Real-time plotter shows `z_ball` and `z_table` traces
- [ ] No existing plot (x, y, control signals, errors) is broken

---

## Final Verification

- [ ] All acceptance criteria in `spec.md` are met
- [ ] Project compiles in Debug and Release without warnings
- [ ] Application runs without crash; simulation pause/resume cycle works correctly
- [ ] All 6 manual-state sliders respond correctly when paused
- [ ] 3D rendering correctly reflects all 6 states simultaneously
- [ ] Axis labels visible and correctly placed
- [ ] Plotter includes z-state channels

---

_Generated by Conductor. Tasks will be marked [~] in progress and [x] complete._
