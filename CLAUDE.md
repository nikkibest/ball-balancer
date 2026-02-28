# Ball Balancer - CLAUDE.md

## Project Overview

Real-time physics simulation and control system for a 2-DOF ball-balancing table. Simulates a ball rolling on a tilting platform, controlled via PID feedback with Kalman filtering for state estimation, and rendered in 3D with OpenGL.

**Stack:** C++17, CMake 3.15+, OpenGL 4.5, Dear ImGui (docking), ImPlot, Eigen3, GLFW, GLAD, GoogleTest
**Platforms:** Desktop (Linux/macOS/Windows) + Web (Emscripten/WebAssembly/WebGL 2.0)

---

## Directory Structure

```
src/                  # Implementation files
  main.cpp            # Desktop entry point
  main_web.cpp        # Web/Emscripten entry point
  core/               # Application lifecycle and types
  physics/            # RK4 physics integration
  control/            # PID controller + Kalman filter
  math/               # Linear algebra utilities
  rendering/          # OpenGL 4.5 renderer, shaders, camera
  gui/                # Dear ImGui control panel and main window
  visualization/      # ImPlot real-time plotting, ring buffer
include/ball_balancer/ # Public headers (mirror of src/)
shaders/              # GLSL shaders (basic, grid; desktop + web variants)
tests/                # GoogleTest unit/integration tests
external/             # Git submodules: glfw, imgui, implot, eigen, glad
conductor/            # Project management tracks and style guides
docs/                 # Architecture and module documentation
```

---

## Build

```bash
# Desktop Release
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j$(nproc)

# Desktop Debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build . -j$(nproc)

# With tests
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON ..
cmake --build . && ctest --output-on-failure

# Web (Emscripten)
./build_web.sh
```

---

## Architecture

### Data Flow
```
Camera (noisy position) → Kalman Filter → PID Controller → Physics Simulator → Renderer/GUI/Plots
```

### Core Types (`include/ball_balancer/core/types.hpp`)
- `StateVector` (6D): `[x, y, vx, vy, theta_x, theta_y]`
- `ControlVector` (2D): `[theta_x_cmd, theta_y_cmd]`
- `MeasurementVector` (2D): `[x_meas, y_meas]`
- Named index access via `state_index::X`, `state_index::VX`, etc.

### Physics Model (`src/physics/simulator.cpp`)
- Rolling sphere on inclined plane: `a = (5/7)*g*sin(θ) - friction`
- Hand-written RK4 integration (10 substeps per 10 ms control tick)
- First-order servo lag: `dtheta/dt = (cmd - theta) / tau_servo`
- All units SI: meters, seconds, kg, radians

### Axis Convention (verified after bug fix, commits 3927021–6642fe5)
- `theta_X` → table rotation around X axis → drives ball in Y direction
- `theta_Y` → table rotation around Y axis → drives ball in X direction
- Physics gravity mapping: positive theta_x tilts table so ball rolls in +Y
- PID and Kalman axes were swapped to match corrected physics

### Control Loop (100 Hz)
1. `simulator_.step(0.01, current_control_)` — physics
2. `estimator_.predict(control)` + `estimator_.update(measurement)` — Kalman
3. `controller_.compute(setpoint, estimated_state)` — PID → new control
4. Render at 60 FPS (fixed-timestep accumulator separates physics from render)

### Key System Parameters
```
Ball: 0.027 kg, 0.020 m radius, ping-pong ball
Table: 0.5m × 0.5m, max tilt ±10° (0.174 rad)
Servo: 50 ms time constant
Camera: 60 Hz, 1 mm noise std dev
```

---

## Code Style (`conductor/code_styleguides/cpp.md`)

- **Classes/Structs:** PascalCase (`PIDController`, `StateEstimator`)
- **Methods/Functions:** camelCase (`computeControl()`, `getPosition()`)
- **Variables:** camelCase; member variables may have trailing underscore (`kp_`, `integral_`)
- **Constants:** ALL_CAPS (`GRAVITY`, `MAX_TILT_ANGLE`)
- **Namespaces:** lowercase with underscores (`ball_balancer::physics`)
- **Indentation:** 4 spaces (no tabs)
- **Braces:** K&R style (opening brace on same line)
- **Line length:** 100 characters max
- **Headers:** `#pragma once`; include order: own header → C headers → stdlib → third-party → project

### Key Conventions
- RAII for all GPU and OS resources; no raw `new`/`delete`
- `std::unique_ptr` for exclusive ownership
- `const` on all accessor methods
- Prefer fixed-size Eigen types (`Vector2d`, `Matrix<double,6,6>`) over dynamic
- Use `C.noalias() = A * B` to avoid Eigen temporaries in hot paths
- Pass large objects by `const&`
- Use `static_cast<>()`, never C-style casts
- `assert()` for debug invariants, exceptions for runtime errors

---

## Testing

Tests live in `tests/`. Build with `-DBUILD_TESTS=ON`.

Currently enabled:
- `control_test.cpp` — PID, Kalman filter, axis-mismatch regression
- `data_manager_test.cpp` — Ring buffer

Commented out in CMakeLists.txt (re-enable if needed):
- `physics_test.cpp`
- `integration_test.cpp`

Critical regression test: `StateEstimatorAxisMismatch.VXDrivenByThetaXNotThetaY` — verifies that positive `theta_x` increases `vx` (not `vy`).

---

## Active Work (Conductor Tracks)

Track: `conductor/tracks/opengl-axes-fix_20260222/`
Status: Investigation complete; fixes landed in recent commits.
See `conductor/tracks/opengl-axes-fix_20260222/plan.md` for remaining tasks.

---

## Important Files

| File | Purpose |
|------|---------|
| `include/ball_balancer/core/types.hpp` | All shared type aliases and `SystemParameters` |
| `src/core/application.cpp` | Main loop, subsystem wiring |
| `src/physics/simulator.cpp` | RK4 physics with servo dynamics |
| `src/control/pid_controller.cpp` | Dual-axis PID with anti-windup |
| `src/control/state_estimator.cpp` | Discrete-time Kalman filter |
| `src/rendering/renderer.cpp` | OpenGL scene rendering |
| `CMakeLists.txt` | Desktop build config |
| `CMakeLists.web.txt` | Emscripten build config |
| `conductor/code_styleguides/cpp.md` | Full C++ style guide |

---

## Common Pitfalls

- **Axis confusion:** `theta_x` tilts the table about X, which rolls the ball along Y. The PID X-axis controls `theta_y` and vice versa — this is correct and intentional after the axes fix.
- **Web build:** Uses OpenGL ES 3.0 shaders (`shaders/*_web.vert/frag`). Desktop uses OpenGL 4.5 core.
- **Eigen in Release:** Use `-DEIGEN_NO_DEBUG` and `-O3 -march=native` for full SIMD performance.
- **ImGui config:** `external/imconfig.h` is the project's ImGui configuration — do not modify without care.
- **Physics subteps:** Simulator runs 10× 1 ms substeps internally per 10 ms control tick for accuracy.
