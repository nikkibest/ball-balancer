# Ball Balancer Architecture

**Project:** Ball Balancer - 2-DOF Platform Control System
**Architect:** C++ Architect Agent
**Date:** 2025-12-08

## System Overview

The ball balancer is a real-time control system that maintains a ball's position on a tilting table platform. The system uses camera-based tracking, PID control, and servo actuators to balance the ball at desired setpoints.

### Key Components

1. **Physics Simulation** - First-principles model of ball/table dynamics
2. **Control System** - PID controller with state estimation
3. **3D Visualization** - OpenGL rendering of ball and table
4. **User Interface** - ImGui control panel for parameter tuning
5. **Real-Time Plotting** - ImPlot visualization of system states

## Architecture Principles

Following **research/cpp-best-practices-modern-programming.md**:

- ✅ **RAII** - All resources owned by objects, automatic cleanup
- ✅ **No raw new/delete** - Smart pointers (unique_ptr) for ownership
- ✅ **Modern C++17** - STL algorithms, const-correctness, value semantics
- ✅ **Clean separation** - Clear module boundaries with minimal coupling
- ✅ **Type safety** - Strong types via Eigen, compile-time size checking

## Directory Structure

```
ball-balancer/
├── CMakeLists.txt              # Build system configuration
├── src/                        # Implementation files
│   ├── core/                   # Application lifecycle
│   │   ├── application.cpp
│   │   └── types.cpp
│   ├── physics/                # ODE models and simulation
│   │   ├── ball_model.cpp
│   │   ├── table_model.cpp
│   │   └── simulator.cpp
│   ├── control/                # Controllers and estimators
│   │   ├── pid_controller.cpp
│   │   └── state_estimator.cpp
│   ├── math/                   # Linear algebra utilities
│   │   └── matrix_utils.cpp
│   ├── rendering/              # OpenGL rendering
│   │   ├── renderer.cpp
│   │   ├── shader.cpp
│   │   └── camera.cpp
│   ├── gui/                    # ImGui user interface
│   │   ├── control_panel.cpp
│   │   └── main_window.cpp
│   ├── visualization/          # ImPlot data visualization
│   │   └── real_time_plotter.cpp
│   └── main.cpp                # Application entry point
├── include/ball_balancer/      # Public headers (interfaces)
│   ├── core/
│   │   ├── application.hpp
│   │   └── types.hpp           # ★ Core interface contract
│   ├── physics/
│   ├── control/
│   ├── math/
│   ├── rendering/
│   ├── gui/
│   └── visualization/
├── shaders/                    # GLSL shader files
│   ├── basic.vert
│   ├── basic.frag
│   └── grid.vert/frag
├── tests/                      # Unit and integration tests
│   ├── physics_test.cpp
│   ├── control_test.cpp
│   └── integration_test.cpp
├── external/                   # Third-party dependencies
│   ├── glfw/
│   ├── imgui/
│   ├── implot/
│   └── googletest/
└── docs/                       # Documentation
    └── ARCHITECTURE.md         # This file
```

## State-Space Representation

**Defined in:** `include/ball_balancer/core/types.hpp`

### State Vector (6D)
```
x = [x, y, vx, vy, theta_x, theta_y]ᵀ

where:
  x, y        - Ball position on table surface (m)
  vx, vy      - Ball velocity (m/s)
  theta_x     - Table tilt angle around X axis (rad)
  theta_y     - Table tilt angle around Y axis (rad)
```

### Control Vector (2D)
```
u = [theta_x_cmd, theta_y_cmd]ᵀ

where:
  theta_x_cmd - Commanded tilt angle X (rad)
  theta_y_cmd - Commanded tilt angle Y (rad)
```

### Measurement Vector (2D)
```
y = [x_meas, y_meas]ᵀ

where:
  x_meas, y_meas - Ball position from camera (m)
```

## Module Interfaces

### Core Module
**Agent:** C++ Architect Agent
**Responsibility:** Type definitions, application lifecycle

**Key Types:**
- `StateVector` - System state (6D)
- `ControlVector` - Control input (2D)
- `MeasurementVector` - Sensor measurement (2D)
- `SystemParameters` - Physical constants

**Interface:** `types.hpp` defines ALL shared types

### Physics Module
**Agent:** Physics Agent
**Responsibility:** First-principles dynamics, ODE integration

**Interface:**
```cpp
class Simulator {
public:
    // Integrate system forward by dt
    void step(double dt, const ControlVector& control);

    // Get current state
    const StateVector& get_state() const;

    // Get simulated measurement (with noise)
    MeasurementVector get_measurement() const;
};
```

**Dependencies:**
- Uses: `StateVector`, `ControlVector` from Core
- Uses: Eigen for linear algebra
- Uses: Boost.Odeint for RK4 integration

### Control Module
**Agent:** Control Agent
**Responsibility:** PID controller, Kalman filter

**Interface:**
```cpp
class PIDController {
public:
    // Compute control output
    ControlVector compute(double dt,
                          const MeasurementVector& setpoint,
                          const MeasurementVector& measurement);
};

class StateEstimator {
public:
    // Update estimate with measurement
    void update(const MeasurementVector& measurement);

    // Get estimated state
    const StateVector& get_estimate() const;
};
```

**Dependencies:**
- Uses: `StateVector`, `ControlVector`, `MeasurementVector` from Core
- Uses: Eigen for matrix operations

### Math Module
**Agent:** Eigen Agent
**Responsibility:** Linear algebra utilities, matrix helpers

**Interface:**
```cpp
namespace matrix_utils {
    // Helper functions for matrix operations
    SystemMatrix compute_linearization(...);
    bool is_controllable(const SystemMatrix& A, const ControlMatrix& B);
    bool is_observable(const SystemMatrix& A, const MeasurementMatrix& C);
}
```

**Dependencies:**
- Uses: Matrix types from Core
- Implements: Advanced Eigen operations

### Rendering Module
**Agent:** OpenGL Agent
**Responsibility:** 3D visualization, shaders, camera

**Interface:**
```cpp
class Renderer {
public:
    // Render scene
    void render(const StateVector& state);

    // Update camera
    void set_camera_view(...);
};
```

**Dependencies:**
- Uses: `StateVector` for object positions
- Uses: OpenGL, GLFW for windowing
- Provides: Rendering context for ImGui

### GUI Module
**Agent:** ImGui Agent
**Responsibility:** Control panel, parameter tuning UI

**Interface:**
```cpp
class MainWindow {
public:
    // Render GUI (call between NewFrame/Render)
    void render();

    // Get user inputs
    MeasurementVector get_setpoint() const;
    PIDGains get_pid_gains() const;
    bool is_simulation_running() const;
};
```

**Dependencies:**
- Uses: Parameter types from Core
- Uses: ImGui framework
- Uses: OpenGL rendering context

### Visualization Module
**Agent:** ImPlot Agent
**Responsibility:** Real-time data plotting

**Interface:**
```cpp
class RealTimePlotter {
public:
    // Add data point
    void add_data(double time, const StateVector& state, const ControlVector& control);

    // Render plots (within ImGui window)
    void render();
};
```

**Dependencies:**
- Uses: State/control types from Core
- Uses: ImPlot framework
- Embeds: Within ImGui windows

## Data Flow

```
┌─────────────┐
│   Camera    │ (simulated sensor)
│ Measurement │
└──────┬──────┘
       │ MeasurementVector
       v
┌─────────────────┐
│ State Estimator │ (Kalman Filter)
│   (Control)     │
└───────┬─────────┘
        │ StateVector (estimated)
        v
┌─────────────────┐      ┌──────────────┐
│ PID Controller  │<─────│ GUI Setpoint │
│   (Control)     │      └──────────────┘
└───────┬─────────┘
        │ ControlVector
        v
┌─────────────────┐
│   Simulator     │ (Physics ODE)
│   (Physics)     │
└───────┬─────────┘
        │ StateVector (true)
        v
   ┌────┴────┐
   │         │
   v         v
┌─────┐  ┌─────────┐
│ 3D  │  │  Plots  │
│Viz  │  │ (ImPlot)│
└─────┘  └─────────┘
```

## Agent Coordination Plan

### Phase 1: Core Infrastructure (Parallel)

**Group 1A - Foundation Types**
- **C++ Architect** ✅ DONE
  - Created: `types.hpp`, `application.hpp`, `CMakeLists.txt`
  - Status: Core interfaces defined

**Group 1B - Independent Modules (CAN WORK IN PARALLEL)**

1. **Physics Agent**
   - Implement: `ball_model.cpp`, `table_model.cpp`, `simulator.cpp`
   - Headers: `ball_model.hpp`, `table_model.hpp`, `simulator.hpp`
   - Use: `StateVector`, `ControlVector` from `types.hpp`
   - Deliverable: Working ODE simulation with Boost.Odeint

2. **Eigen Agent**
   - Implement: `matrix_utils.cpp`
   - Header: `matrix_utils.hpp`
   - Use: Matrix types from `types.hpp`
   - Deliverable: Linear algebra utilities, controllability/observability checks

3. **OpenGL Agent**
   - Implement: `renderer.cpp`, `shader.cpp`, `camera.cpp`
   - Shaders: `basic.vert/frag`, `grid.vert/frag`
   - Use: `StateVector` for positions
   - Deliverable: 3D rendering pipeline with shaders

**Dependencies:** None between Group 1B agents - can work in parallel!

### Phase 2: Integration Layer (Sequential after Phase 1)

**Group 2 - Depends on Phase 1**

4. **Control Agent**
   - Implement: `pid_controller.cpp`, `state_estimator.cpp`
   - Depends on: Physics model interface (for tuning)
   - Use: All state/control types
   - Deliverable: PID with anti-windup, Kalman filter

5. **ImGui Agent**
   - Implement: `control_panel.cpp`, `main_window.cpp`
   - Depends on: OpenGL rendering context
   - Use: Parameter types
   - Deliverable: Control panel UI with docking

6. **ImPlot Agent**
   - Implement: `real_time_plotter.cpp`
   - Depends on: ImGui windows (embeds plots)
   - Use: State/control types
   - Deliverable: Real-time plots of position, velocity, control

**Dependencies:** Control needs Physics interface; ImGui needs OpenGL; ImPlot needs ImGui

### Phase 3: Integration & Testing (Sequential)

7. **C++ Architect**
   - Review all code for C++ quality
   - Implement: `application.cpp`, `main.cpp`
   - Create: Integration tests
   - Verify: RAII, no raw pointers, const-correctness
   - Profile: Performance, memory leaks

## Build Dependencies

### Required
- **CMake** >= 3.15
- **C++17** compiler (GCC, Clang, MSVC)
- **Eigen3** >= 3.3 (linear algebra)
- **Boost** >= 1.65 (Odeint for ODE integration)
- **OpenGL** 4.5+ (rendering)

### To Be Added (external/)
- **GLFW** - Window management
- **ImGui** - GUI framework (docking branch)
- **ImPlot** - Plotting library
- **GoogleTest** - Unit testing (optional)

## Performance Targets

- **Control Loop:** 100 Hz (10 ms period)
- **Rendering:** 60 FPS (16.67 ms frame time)
- **Physics Integration:** RK4 with dt = 1 ms
- **Total Latency:** < 20 ms (sensor to actuator)

## Testing Strategy

### Unit Tests
- **Physics:** Energy conservation, equilibrium points
- **Control:** PID anti-windup, Kalman filter accuracy
- **Math:** Matrix operations, controllability checks

### Integration Tests
- **Closed-loop:** Controller stabilizes system
- **Performance:** Meets timing requirements
- **Memory:** No leaks (valgrind)

## Quality Checklist

Before declaring complete:

- [ ] All modules follow RAII
- [ ] No raw new/delete anywhere
- [ ] Compiles with -Wall -Wextra -Werror
- [ ] Smart pointers used appropriately
- [ ] Const-correctness enforced
- [ ] STL algorithms over raw loops
- [ ] Unit tests pass
- [ ] Integration tests pass
- [ ] Performance targets met
- [ ] No memory leaks
- [ ] Documentation complete

## Next Steps

1. **Coordinate Group 1B agents** (Physics, Eigen, OpenGL) - Parallel work
2. **Review Group 1B output** - Ensure interfaces match
3. **Coordinate Group 2 agents** (Control, ImGui, ImPlot) - Sequential
4. **Integration** - Wire everything together in `application.cpp`
5. **Testing** - Unit and integration tests
6. **Optimization** - Profile and optimize hot paths

---

**Document Status:** Architecture Design Complete
**Next Phase:** Agent Coordination (Group 1B)
