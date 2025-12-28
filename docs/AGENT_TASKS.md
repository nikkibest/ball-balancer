# Agent Coordination Tasks

**Project:** Ball Balancer
**Coordinator:** C++ Architect Agent
**Status:** Architecture Complete - Ready for Implementation

## Architecture Summary

✅ **Completed:**
- Project structure created
- CMakeLists.txt configured
- Core type system defined (`include/ball_balancer/core/types.hpp`)
- Application interface defined (`include/ball_balancer/core/application.hpp`)
- Architecture documented (`docs/ARCHITECTURE.md`)

🚀 **Ready for Implementation:**
- All interfaces defined
- Module boundaries clear
- Dependencies identified

## Phase 1B: Parallel Implementation (Group 1B)

The following 3 agents can work **IN PARALLEL** as their modules are independent:

---

### 1. Physics Agent (@physics-agent)

**Status:** ⏳ Ready to start
**Priority:** High (other modules depend on this for integration)

**Task:** Implement ball/table dynamics from first principles

**Files to Create:**
- `include/ball_balancer/physics/ball_model.hpp`
- `include/ball_balancer/physics/table_model.hpp`
- `include/ball_balancer/physics/simulator.hpp`
- `src/physics/ball_model.cpp`
- `src/physics/table_model.cpp`
- `src/physics/simulator.cpp`

**Requirements:**

1. **Derive Physics Model**
   - Ball rolling on tilting table (no slip condition)
   - Apply Newton's laws: F = ma, τ = Iα
   - Table tilt affects ball acceleration: a = g * sin(theta)
   - Include rolling friction

2. **State-Space Form**
   - Use `StateVector` from `core/types.hpp`
   - Implement: `dx/dt = f(x, u, t)` where:
     - x = [x, y, vx, vy, theta_x, theta_y]
     - u = [theta_x_cmd, theta_y_cmd]

3. **ODE Integration**
   - Use Boost.Odeint with RK4
   - Time step: dt = 0.001 s (1 kHz integration)
   - Implement `Simulator` class with `step(dt, control)` method

4. **Validation**
   - Test: Ball rolls downhill when table tilted
   - Test: Ball stays at rest on flat table
   - Test: Energy conservation (for zero friction case)

**Interface Contract:**
```cpp
class Simulator {
public:
    explicit Simulator(const SystemParameters& params);

    // Integrate system forward by dt with control input
    void step(double dt, const ControlVector& control);

    // Reset to initial state
    void reset(const StateVector& initial_state);

    // Get current true state
    const StateVector& get_state() const;

    // Get simulated measurement (adds camera noise)
    MeasurementVector get_measurement() const;

private:
    SystemParameters params_;
    StateVector state_;
    // ... ODE integrator
};
```

**Knowledge Base:**
- Read: `research/ode-physical-system-modeling-cpp.md`
- Follow: First-principles modeling, validation practices

---

### 2. Eigen Agent (@eigen-agent)

**Status:** ⏳ Ready to start
**Priority:** Medium (utilities for Control Agent)

**Task:** Implement linear algebra utilities and matrix helpers

**Files to Create:**
- `include/ball_balancer/math/matrix_utils.hpp`
- `src/math/matrix_utils.cpp`

**Requirements:**

1. **System Analysis Functions**
   - `is_controllable(A, B)` - Check controllability rank
   - `is_observable(A, C)` - Check observability rank
   - `compute_linearization(...)` - Linearize nonlinear dynamics

2. **Matrix Operations**
   - Safe wrappers that avoid common Eigen pitfalls
   - No `auto` with expressions - always use explicit types
   - Proper aliasing handling with `.eval()` where needed

3. **Type Safety**
   - Use fixed-size matrices from `core/types.hpp`
   - Document matrix dimensions clearly
   - Add assertion checks for dimension compatibility

**Interface Contract:**
```cpp
namespace ball_balancer::matrix_utils {

// Check if system is controllable
bool is_controllable(const SystemMatrix& A, const ControlMatrix& B);

// Check if system is observable
bool is_observable(const SystemMatrix& A, const MeasurementMatrix& C);

// Compute linearized system matrices around operating point
struct LinearizedSystem {
    SystemMatrix A;
    ControlMatrix B;
    MeasurementMatrix C;
    FeedthroughMatrix D;
};

LinearizedSystem compute_linearization(
    const StateVector& operating_point,
    const ControlVector& control_point
);

} // namespace ball_balancer::matrix_utils
```

**Knowledge Base:**
- Read: `research/eigen-cpp-linear-algebra-best-practices.md`
- Follow: No auto with expressions, use decompositions, SIMD optimization

---

### 3. OpenGL Agent (@opengl-agent)

**Status:** ⏳ Ready to start
**Priority:** Medium (visualization only, not critical path)

**Task:** Create 3D rendering system for ball and table

**Files to Create:**
- `include/ball_balancer/rendering/renderer.hpp`
- `include/ball_balancer/rendering/shader.hpp`
- `include/ball_balancer/rendering/camera.hpp`
- `src/rendering/renderer.cpp`
- `src/rendering/shader.cpp`
- `src/rendering/camera.cpp`
- `shaders/basic.vert` - Basic vertex shader
- `shaders/basic.frag` - Basic fragment shader
- `shaders/grid.vert/frag` - Grid floor shaders

**Requirements:**

1. **Scene Elements**
   - Ball (sphere) at position (x, y, z) where z = ball_radius
   - Table platform (tilted plane) with angles (theta_x, theta_y)
   - Grid floor for reference
   - Coordinate axes (X=red, Y=green, Z=blue)

2. **Rendering Pipeline**
   - Use DSA (Direct State Access) - OpenGL 4.5+
   - Single draw call for each object (batching)
   - Simple lighting: directional light + ambient
   - Camera: Orbital camera looking at origin

3. **Shader Management**
   - RAII wrapper for shader programs
   - Vertex + Fragment shaders
   - Uniform management (MVP matrices, colors)

4. **Integration**
   - Provide OpenGL context for ImGui backend
   - Frame time: target 60 FPS (16.67 ms)

**Interface Contract:**
```cpp
class Renderer {
public:
    Renderer();
    ~Renderer();

    // Initialize OpenGL resources
    bool initialize(int width, int height);

    // Render scene with current state
    void render(const StateVector& state);

    // Update camera view
    void set_camera_position(const Eigen::Vector3d& position);
    void set_camera_target(const Eigen::Vector3d& target);

    // Get OpenGL context for ImGui
    void* get_gl_context();

private:
    // RAII wrappers for OpenGL resources
    std::unique_ptr<Shader> basic_shader_;
    std::unique_ptr<Camera> camera_;
    // VAOs, VBOs managed with RAII
};
```

**Knowledge Base:**
- Read: `research/opengl-rendering-best-practices.md`
- Follow: DSA, RAII for GPU resources, batching, no bind calls

---

## Phase 2: Integration Layer (Sequential - After Phase 1B)

### 4. Control Agent (@control-agent)

**Status:** ⏸️ Waiting for Physics Agent
**Priority:** High

**Dependencies:**
- Needs: `Simulator` interface from Physics Agent (for tuning)
- Needs: Matrix utilities from Eigen Agent (optional, can work without)

**Task:** Implement PID controller and Kalman filter

**Files to Create:**
- `include/ball_balancer/control/pid_controller.hpp`
- `include/ball_balancer/control/state_estimator.hpp`
- `src/control/pid_controller.cpp`
- `src/control/state_estimator.cpp`

**Requirements:**

1. **PID Controller**
   - Dual-axis control (X and Y independent)
   - Anti-windup clamping
   - Derivative filtering (low-pass)
   - Configurable gains (Kp, Ki, Kd)

2. **State Estimator (Kalman Filter)**
   - Estimate full state from position measurements
   - Tune Q (process noise) and R (measurement noise)
   - Provide velocity estimates

3. **Performance**
   - Control loop: 100 Hz (10 ms)
   - Settling time: < 2 seconds
   - Overshoot: < 5%

**Coordination:** Will be assigned after Physics Agent completes

---

### 5. ImGui Agent (@imgui-agent)

**Status:** ⏸️ Waiting for OpenGL Agent
**Priority:** Medium

**Dependencies:**
- Needs: OpenGL context from OpenGL Agent

**Task:** Create control panel UI with docking layout

**Files to Create:**
- `include/ball_balancer/gui/control_panel.hpp`
- `include/ball_balancer/gui/main_window.hpp`
- `src/gui/control_panel.cpp`
- `src/gui/main_window.cpp`

**Requirements:**

1. **Control Panel**
   - Start/Stop/Reset buttons
   - Setpoint sliders (X, Y position)
   - PID gain tuning (Kp, Ki, Kd for X and Y)
   - System status display

2. **Docking Layout**
   - Main dockspace over viewport
   - Control panel (left)
   - Plots panel (right)
   - 3D view (center)

3. **Professional Appearance**
   - Load custom font (not default)
   - Clean styling
   - Dynamic sizing

**Coordination:** Will be assigned after OpenGL Agent completes

---

### 6. ImPlot Agent (@implot-agent)

**Status:** ⏸️ Waiting for ImGui Agent
**Priority:** Low

**Dependencies:**
- Needs: ImGui windows from ImGui Agent

**Task:** Add real-time plots for system monitoring

**Files to Create:**
- `include/ball_balancer/visualization/real_time_plotter.hpp`
- `src/visualization/real_time_plotter.cpp`

**Requirements:**

1. **Plot Types**
   - Ball position (X vs Y) - 2D trajectory
   - Position vs time (X and Y separate)
   - Control signals (table angles)
   - Error (setpoint - actual)

2. **Data Management**
   - Ring buffers for streaming data
   - 10 seconds of history
   - 60 Hz update rate

3. **Critical Configuration**
   - **Ensure 32-bit indices enabled** in imconfig.h

**Coordination:** Will be assigned after ImGui Agent completes

---

## Phase 3: Integration (C++ Architect)

### 7. Integration & Testing

**Status:** ⏸️ Waiting for all agents
**Priority:** Critical

**Task:**
- Implement `src/core/application.cpp`
- Implement `src/main.cpp`
- Wire all subsystems together
- Create integration tests
- Final code review

---

## Current Status Summary

| Agent | Status | Can Start | Blocked By |
|-------|--------|-----------|------------|
| C++ Architect | ✅ Complete | - | - |
| Physics Agent | ⏳ Ready | ✅ Yes | None |
| Eigen Agent | ⏳ Ready | ✅ Yes | None |
| OpenGL Agent | ⏳ Ready | ✅ Yes | None |
| Control Agent | ⏸️ Waiting | ❌ No | Physics |
| ImGui Agent | ⏸️ Waiting | ❌ No | OpenGL |
| ImPlot Agent | ⏸️ Waiting | ❌ No | ImGui |
| Integration | ⏸️ Waiting | ❌ No | All |

## Next Action

**You (the user) should invoke these 3 agents IN PARALLEL:**

```
@physics-agent: Implement ball/table dynamics following the task in
docs/AGENT_TASKS.md (Physics Agent section). Use core types from
include/ball_balancer/core/types.hpp.

@eigen-agent: Implement matrix utilities following the task in
docs/AGENT_TASKS.md (Eigen Agent section). Use matrix types from
include/ball_balancer/core/types.hpp.

@opengl-agent: Implement 3D rendering following the task in
docs/AGENT_TASKS.md (OpenGL Agent section). Use StateVector from
include/ball_balancer/core/types.hpp.

These three can work in parallel - no dependencies between them!
```

---

**Architecture Complete. Ready for agent implementation!**
