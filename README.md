# Ball Balancer Simulation

A real-time physics simulation of a ball balancing on a 2-DOF tilting table platform, featuring PID control, Kalman filtering, and 3D visualization.

## Overview

This project simulates a ball rolling on a motorized tilting table. The system uses:
- **Physics Simulation**: Hand-written RK4 integrator for accurate rolling ball dynamics
- **State Estimation**: Kalman filter for estimating ball velocity from noisy position measurements
- **Control**: Dual-axis PID controller for position control
- **Visualization**: Real-time 3D OpenGL rendering with Dear ImGui interface and ImPlot graphs

The system can run natively on desktop (Windows/Linux/macOS) or be compiled to WebAssembly for browser-based execution.

## Features

- First-principles physics model with rolling friction and boundary enforcement
- State estimation using extended Kalman filter
- Configurable PID gains with anti-windup
- Real-time 3D visualization of ball and table
- Live plotting of position, velocity, control signals, and errors
- Interactive GUI for parameter tuning
- Web deployment via Emscripten/WebAssembly

## Prerequisites

### Desktop Build

- **CMake** >= 3.15
- **C++17 compiler** (GCC 7+, Clang 5+, MSVC 2017+)
- **OpenGL** 4.5+ (desktop) or OpenGL ES 3.0 (web)
- **Git** for submodule management

### Platform-Specific Dependencies

**Linux:**
```bash
# Ubuntu/Debian
sudo apt-get install cmake build-essential libgl1-mesa-dev libx11-dev

# Arch Linux
sudo pacman -S cmake gcc mesa libx11
```

**macOS:**
```bash
brew install cmake
# Xcode Command Line Tools provide compilers and OpenGL
```

**Windows:**
- Visual Studio 2017+ or MinGW-w64
- CMake (from cmake.org or via Visual Studio installer)

### Web Build (Optional)

- **Emscripten SDK** for WebAssembly compilation
- See `docs/WEB_BUILD_GUIDE.md` for detailed web build instructions

## Quick Start

### 1. Clone and Initialize Submodules

```bash
git clone https://github.com/yourusername/ball-balancer.git
cd ball-balancer

# Initialize external dependencies
git submodule update --init --recursive
```

This will fetch:
- GLFW (windowing)
- Dear ImGui (GUI)
- ImPlot (plotting)
- Eigen (linear algebra)
- GLAD (OpenGL loader)

### 2. Build (Desktop)

```bash
# Create build directory
mkdir -p build && cd build

# Configure for Release build (recommended for performance)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Build
cmake --build . -j$(nproc)

# Or using make directly:
# make -j$(nproc)
```

**Note**: Release builds use `-O3 -march=native` for 5-20× performance improvement over Debug builds.

### 3. Run

```bash
# From build directory
./ball_balancer
```

### Controls

- **Control Panel**: Adjust setpoint position, PID gains, and Kalman filter parameters
- **Start/Pause**: Control simulation and controller state independently
- **Manual Control**: Directly control table tilt angles
- **Camera Controls** (hold CTRL):
  - **Right-click + drag**: Rotate camera
  - **Left-click + drag**: Pan camera
  - **Scroll wheel**: Zoom in/out

## Build Configurations

### Debug Build (Development)

```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```
- Verbose logging to stdout
- All assertions and runtime checks enabled
- Optimizations disabled for easier debugging

### Release Build (Production)

```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```
- Minimal stdout output (errors only)
- Full compiler optimizations (`-O3 -march=native`)
- Eigen runtime checks disabled
- Recommended for performance testing and deployment

## Testing

```bash
# Enable tests (ON by default)
cmake -DBUILD_TESTS=ON ..
make -j$(nproc)

# Run tests
ctest --output-on-failure
```

## Web Build

For browser-based execution, see the comprehensive web build guide:

```bash
# See detailed instructions
cat docs/WEB_BUILD_GUIDE.md

# Quick build (requires Emscripten SDK installed)
./build_web.sh
```

The web build produces WebAssembly binaries that run in modern browsers with WebGL 2.0 support.

## Project Structure

```
ball-balancer/
├── src/                  # Source files
│   ├── core/             # Application framework
│   ├── physics/          # Physics simulation (RK4 integrator)
│   ├── control/          # PID controller and Kalman filter
│   ├── rendering/        # OpenGL rendering engine
│   ├── gui/              # Dear ImGui interface
│   └── visualization/    # ImPlot data plotting
├── include/              # Header files
│   └── ball_balancer/    # Public API headers
├── external/             # Third-party libraries (git submodules)
│   ├── glfw/             # Windowing
│   ├── imgui/            # GUI framework
│   ├── implot/           # Plotting library
│   ├── eigen/            # Linear algebra
│   └── glad/             # OpenGL loader
├── shaders/              # GLSL shaders (desktop and web variants)
├── tests/                # Unit and integration tests
├── docs/                 # Documentation
├── research/             # Design docs and best practices
└── CMakeLists.txt        # Build configuration
```

## Documentation

- `docs/ARCHITECTURE.md` - System architecture and design
- `docs/PROJECT_COMPLETE.md` - Implementation status
- `docs/WEB_BUILD_GUIDE.md` - Web deployment guide
- `research/` - Best practices documents for each subsystem

## Performance

| Configuration | FPS | Physics Update Rate |
|---------------|-----|---------------------|
| Debug (-O0)   | 60  | 100 Hz (fixed)      |
| Release (-O3) | 60  | 100 Hz (fixed)      |

Release builds achieve **5-20× faster** Kalman filter and physics computations compared to Debug due to Eigen expression template optimization.

## License

[Your License Here]

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes with tests
4. Submit a pull request

## Troubleshooting

**Build fails with "glad.c not found":**
- Run `git submodule update --init --recursive`

**Black screen on startup:**
- Check GPU drivers support OpenGL 4.5+
- Try Debug build for verbose shader loading logs

**Performance issues:**
- Use Release build (`-DCMAKE_BUILD_TYPE=Release`)
- Check GPU driver is using discrete GPU (not integrated)

**Web build fails:**
- Ensure Emscripten SDK is activated: `source ~/emsdk/emsdk_env.sh`
- See `docs/WEB_BUILD_GUIDE.md` for detailed instructions

## Acknowledgments

- **GLFW** - Window management
- **Dear ImGui** - Immediate mode GUI
- **ImPlot** - Real-time plotting
- **Eigen** - Linear algebra
- **Emscripten** - WebAssembly compiler
