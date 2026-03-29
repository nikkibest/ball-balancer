# Tech Stack

## Languages

- **C++17** - Primary development language

## Build System

- **CMake 3.15+** - Cross-platform build configuration
- Support for Debug and Release configurations
- Automated dependency management via git submodules

## Graphics & UI

- **OpenGL 4.5+** (Desktop) / **OpenGL ES 3.0** (Web) - 3D rendering
- **GLFW** - Window management and input handling
- **GLAD** - OpenGL function loader
- **Dear ImGui** - Immediate mode GUI framework
- **ImPlot** - Real-time plotting library

## Mathematics & Physics

- **Eigen** - High-performance linear algebra library
  - Used for matrix operations, state vectors, and Kalman filtering
  - Header-only, no runtime dependencies
- **Boost.Odeint** (Optional) - ODE integration library
  - Currently optional, may be used for advanced physics solvers

## Web Compilation

- **Emscripten SDK** - C++ to WebAssembly compiler
- **WebAssembly (WASM)** - Binary instruction format for web deployment
- **WebGL 2.0** - Browser-based OpenGL ES 3.0 rendering

## Deployment

Desktop (native builds for Windows/Linux/macOS) + Web (via Emscripten/WebAssembly)

### Desktop Platforms

- **Linux** - Primary development platform (Ubuntu/Arch)
- **macOS** - Secondary support
- **Windows** - Secondary support (MSVC/MinGW-w64)

### Web Deployment

- Static HTML/JS/WASM bundle served via HTTP server
- Compatible with modern browsers supporting WebGL 2.0
- No server-side dependencies required

## Development Tools

- **Git** - Version control with submodule management
- **GCC 7+** / **Clang 5+** / **MSVC 2017+** - C++17 compliant compilers
- **CMake** - Build system generator

## Testing (Optional)

- Tests currently optional (BUILD_TESTS flag in CMake)
- Framework: To be determined based on needs
