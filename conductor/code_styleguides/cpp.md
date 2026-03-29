# C++ Style Guide

## General Principles

- **Clarity over cleverness** - Code should be easy to read and understand
- **Type safety** - Use the type system to prevent errors at compile time
- **RAII** - Resource Acquisition Is Initialization for all resource management
- **Modern C++** - Use C++17 features when they improve code quality

## Naming Conventions

### Files

- Header files: `.h` or `.hpp`
- Implementation files: `.cpp`
- Use lowercase with underscores: `ball_physics.cpp`, `pid_controller.h`

### Classes and Structs

- **PascalCase** for class and struct names
- Example: `BallBalancer`, `PIDController`, `KalmanFilter`

```cpp
class BallBalancer {
    // ...
};

struct PhysicsState {
    // ...
};
```

### Functions and Methods

- **camelCase** for function and method names
- Start with lowercase letter
- Example: `updateState()`, `computeControl()`, `getPosition()`

```cpp
void updatePhysics(double dt);
Eigen::Vector2d computeControlSignal();
```

### Variables

- **camelCase** for local variables and parameters
- **camelCase** for member variables, optionally with trailing underscore
- Example: `ballPosition`, `deltaTime`, `velocity_`

```cpp
double deltaTime = 0.01;
Eigen::Vector2d ballPosition;

class Controller {
private:
    double kp_;  // Member variables may have trailing underscore
    double ki_;
    double kd_;
};
```

### Constants

- **ALL_CAPS** with underscores for compile-time constants
- `const` variables may use camelCase if local scope

```cpp
constexpr double GRAVITY = 9.81;
constexpr int MAX_ITERATIONS = 100;

const double timeStep = 0.01;  // Local const can be camelCase
```

### Namespaces

- **lowercase** with underscores if multi-word
- Keep namespace names short and descriptive

```cpp
namespace ball_balancer {
namespace physics {
    // ...
}
}
```

## Formatting

### Indentation

- **4 spaces** (no tabs)
- Use consistent indentation throughout

### Braces

- **K&R style** - Opening brace on same line for functions, classes, control structures

```cpp
class Example {
public:
    void doSomething() {
        if (condition) {
            // ...
        } else {
            // ...
        }
    }
};
```

### Line Length

- Prefer **100 characters** maximum
- Break long lines at logical points

### Whitespace

- Space after control keywords: `if (`, `for (`, `while (`
- No space between function name and parenthesis: `function()`
- Space around binary operators: `a + b`, `x = y`
- No space around member access: `object.member`, `ptr->member`

```cpp
// Good
if (x > 0) {
    result = a + b;
    obj.doSomething();
}

// Bad
if(x>0){
    result=a+b;
    obj . doSomething ();
}
```

## C++17 Features

### Prefer Modern Constructs

```cpp
// Use auto for type deduction when type is obvious
auto position = getPosition();  // Type clear from function name

// Use structured bindings
auto [x, y] = getCoordinates();

// Use std::optional for optional values
std::optional<double> findRoot(double initial);

// Use constexpr for compile-time constants
constexpr double PI = 3.14159265358979323846;
```

### Smart Pointers

- Use `std::unique_ptr` for exclusive ownership
- Use `std::shared_ptr` when shared ownership is needed
- Avoid raw `new`/`delete`

```cpp
auto controller = std::make_unique<PIDController>();
auto renderer = std::make_shared<Renderer>();
```

## Memory Management

### RAII Principle

- Always manage resources via RAII
- Use constructors/destructors for acquisition/release
- Avoid manual resource management

```cpp
class OpenGLBuffer {
public:
    OpenGLBuffer() {
        glGenBuffers(1, &bufferId_);
    }

    ~OpenGLBuffer() {
        glDeleteBuffers(1, &bufferId_);
    }

    // Delete copy, allow move
    OpenGLBuffer(const OpenGLBuffer&) = delete;
    OpenGLBuffer& operator=(const OpenGLBuffer&) = delete;
    OpenGLBuffer(OpenGLBuffer&&) noexcept = default;
    OpenGLBuffer& operator=(OpenGLBuffer&&) noexcept = default;

private:
    GLuint bufferId_;
};
```

## Error Handling

### Use Exceptions for Exceptional Cases

```cpp
// Throw for unrecoverable errors
if (!initializeOpenGL()) {
    throw std::runtime_error("Failed to initialize OpenGL");
}

// Use return codes or std::optional for expected failures
std::optional<Config> loadConfig(const std::string& path);
```

### Assertions

- Use `assert()` for debug-only checks
- Use exceptions for runtime validation

```cpp
#include <cassert>

void updateState(double dt) {
    assert(dt > 0.0);  // Debug check
    if (dt <= 0.0) {   // Runtime check for release
        throw std::invalid_argument("dt must be positive");
    }
    // ...
}
```

## Headers

### Include Guards

- Use `#pragma once` for simplicity

```cpp
#pragma once

// Header content
```

### Include Order

1. Corresponding header (for .cpp files)
2. C system headers
3. C++ standard library headers
4. Third-party library headers
5. Project headers

```cpp
#include "ball_balancer/physics.h"  // Corresponding header

#include <cmath>                     // C system headers
#include <iostream>                  // C++ standard library
#include <vector>

#include <Eigen/Dense>               // Third-party libraries
#include <GLFW/glfw3.h>

#include "ball_balancer/controller.h"  // Project headers
```

## Comments

### When to Comment

- **Don't** comment what the code does (should be obvious)
- **Do** comment why the code does it
- **Do** document public APIs
- **Do** explain non-obvious algorithms or magic numbers

```cpp
// Bad - obvious from code
// Increment i by 1
i++;

// Good - explains reasoning
// Use RK4 integration for better stability with large time steps
integrateRK4(state, dt);

// Good - explains magic number
constexpr double FRICTION_COEFF = 0.02;  // Empirically determined from experiments
```

### Documentation Comments

```cpp
/**
 * Computes the PID control signal for position control.
 *
 * @param error Current position error (setpoint - actual)
 * @param dt Time step since last update
 * @return Control signal to be applied to the actuator
 */
double computeControl(double error, double dt);
```

## Eigen Library Usage

### Prefer Fixed-Size Types When Possible

```cpp
// Fixed-size (preferred for small vectors/matrices)
Eigen::Vector2d position;
Eigen::Matrix2d covariance;

// Dynamic-size (use when size varies)
Eigen::VectorXd state;
Eigen::MatrixXd jacobian;
```

### Use .block() for Submatrix Access

```cpp
Eigen::VectorXd fullState(4);
Eigen::Vector2d position = fullState.head<2>();
Eigen::Vector2d velocity = fullState.tail<2>();
```

### Avoid Temporary Allocations

```cpp
// Good - no temporaries
C.noalias() = A * B;

// Bad - creates temporary
C = A * B;
```

## Performance Considerations

### Pass by const Reference

```cpp
// Large objects by const reference
void processState(const Eigen::VectorXd& state);

// Small objects (primitives, small structs) by value
void setGain(double gain);
```

### Reserve Vector Capacity

```cpp
std::vector<double> trajectory;
trajectory.reserve(1000);  // Avoid reallocations
```

### Prefer constexpr for Compile-Time Computation

```cpp
constexpr double computeGravity() {
    return 9.81;
}

constexpr double GRAVITY = computeGravity();
```

## Platform-Specific Code

### Use Preprocessor Sparingly

```cpp
#ifdef __EMSCRIPTEN__
    // Web-specific code
    emscripten_set_main_loop(mainLoop, 0, 1);
#else
    // Desktop-specific code
    while (!glfwWindowShouldClose(window)) {
        mainLoop();
    }
#endif
```

## Testing

### Test Structure (when tests exist)

```cpp
#include <gtest/gtest.h>

TEST(PIDControllerTest, ProportionalGain) {
    PIDController controller(1.0, 0.0, 0.0);
    double signal = controller.compute(5.0, 0.1);
    EXPECT_DOUBLE_EQ(5.0, signal);
}
```

## Anti-Patterns to Avoid

### Don't Use Raw Pointers for Ownership

```cpp
// Bad
Widget* widget = new Widget();
delete widget;

// Good
auto widget = std::make_unique<Widget>();
```

### Don't Ignore Return Values

```cpp
// Bad
loadConfiguration("config.json");

// Good
if (!loadConfiguration("config.json")) {
    std::cerr << "Failed to load config" << std::endl;
}
```

### Don't Use C-style Casts

```cpp
// Bad
double x = (double)intValue;

// Good
double x = static_cast<double>(intValue);
```

## Project-Specific Conventions

### Physics Simulation

- Use SI units (meters, seconds, kg) throughout
- Document unit conversions explicitly
- Prefer double precision for numerical stability

```cpp
constexpr double BALL_RADIUS = 0.02;    // meters
constexpr double TABLE_SIZE = 0.5;      // meters
constexpr double GRAVITY = 9.81;        // m/s²
```

### Control Systems

- Separate state estimation from control logic
- Use clear naming for gains (kp, ki, kd)
- Document tuning parameters

```cpp
class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

private:
    double kp_;  // Proportional gain
    double ki_;  // Integral gain
    double kd_;  // Derivative gain
};
```
