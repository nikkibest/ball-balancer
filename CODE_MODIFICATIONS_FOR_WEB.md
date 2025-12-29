# Required Code Modifications for Web Build

This document outlines the necessary changes to existing source files to support both desktop and web builds.

## Overview

The following files need conditional compilation directives to work with both native and Emscripten builds:

1. `src/rendering/renderer.cpp` - Shader paths and OpenGL headers
2. `src/rendering/shader.cpp` - Shader version detection
3. `CMakeLists.web.txt` - Web-specific source files

---

## 1. Modify `src/rendering/renderer.cpp`

### Changes Required

Add preprocessor directives for shader paths and OpenGL headers.

### Location: Top of file (after includes)

```cpp
#include <ball_balancer/rendering/renderer.hpp>
#include <cmath>
#include <iostream>

// ADD THESE LINES:
#ifdef __EMSCRIPTEN__
#define SHADER_PATH_PREFIX "/shaders/"
#define SHADER_SUFFIX "_web"
#else
#define SHADER_PATH_PREFIX "shaders/"
#define SHADER_SUFFIX ""
#endif

/**
 * @file renderer.cpp
 * ...
```

### Location: In `Renderer::initialize()` function

Find these lines (approximately line 163-164):

```cpp
// OLD CODE:
basic_shader_ = std::make_unique<Shader>("shaders/basic.vert", "shaders/basic.frag");
grid_shader_ = std::make_unique<Shader>("shaders/grid.vert", "shaders/grid.frag");
```

Replace with:

```cpp
// NEW CODE:
#ifdef __EMSCRIPTEN__
    basic_shader_ = std::make_unique<Shader>("/shaders/basic_web.vert", "/shaders/basic_web.frag");
    grid_shader_ = std::make_unique<Shader>("/shaders/grid_web.vert", "/shaders/grid_web.frag");
#else
    basic_shader_ = std::make_unique<Shader>("shaders/basic.vert", "shaders/basic.frag");
    grid_shader_ = std::make_unique<Shader>("shaders/grid.vert", "shaders/grid.frag");
#endif
```

**Why?** Web builds preload shaders with `/shaders/` prefix (virtual filesystem) and need WebGL-compatible versions.

---

## 2. Modify `src/rendering/shader.cpp`

### Changes Required (Optional but Recommended)

Add shader version validation for WebGL.

### Location: In `Shader::compile_shader()` function

Add after loading shader source:

```cpp
std::string source_code;
// ... existing file reading code ...

// ADD THIS:
#ifdef __EMSCRIPTEN__
// Validate GLSL version for WebGL
if (source_code.find("#version 300 es") == std::string::npos) {
    std::cerr << "WARNING: Web build expects GLSL 300 es shaders" << std::endl;
    std::cerr << "  Shader: " << path << std::endl;
}
#endif
```

**Why?** Provides early warning if wrong shader version is used in web builds.

---

## 3. Update `CMakeLists.web.txt`

### Required Changes

Configure to use web-specific source files.

### Location: Source files section (line ~135)

```cmake
set(BALL_BALANCER_SOURCES
    # Core
    src/core/application_web.cpp    # <-- Use web version instead of application.cpp

    # Physics
    src/physics/simulator.cpp

    # Control
    src/control/pid_controller.cpp
    src/control/state_estimator.cpp

    # Math
    src/math/matrix_utils.cpp

    # Rendering
    src/rendering/renderer.cpp       # <-- Will use conditional compilation
    src/rendering/shader.cpp
    src/rendering/camera.cpp

    # GUI
    src/gui/control_panel.cpp
    src/gui/main_window.cpp

    # Visualization
    src/visualization/real_time_plotter.cpp
    src/visualization/data_manager.cpp

    # Main entry point
    src/main_web.cpp                # <-- Use web version
)
```

### Location: Link options section (line ~200)

Ensure shell template is used:

```cmake
# Add to EMSCRIPTEN_LINK_FLAGS:
"--shell-file ${CMAKE_CURRENT_SOURCE_DIR}/shell.html"
```

**Complete linker flags should be:**

```cmake
set(EMSCRIPTEN_LINK_FLAGS
    # WebGL/GLFW support
    "SHELL:-s USE_WEBGL2=1"
    "SHELL:-s FULL_ES3=1"
    "SHELL:-s USE_GLFW=3"

    # Memory settings
    "SHELL:-s ALLOW_MEMORY_GROWTH=1"
    "SHELL:-s INITIAL_MEMORY=128MB"
    "SHELL:-s STACK_SIZE=5MB"

    # WASM settings
    "SHELL:-s WASM=1"

    # Exception handling
    "SHELL:-s DISABLE_EXCEPTION_CATCHING=0"

    # Main loop handling
    "SHELL:-s NO_EXIT_RUNTIME=1"

    # Optimization
    "${OPTIMIZATION_LEVEL}"

    # Preload shader files
    "--preload-file ${CMAKE_CURRENT_SOURCE_DIR}/shaders@/shaders"

    # Use custom HTML shell
    "--shell-file ${CMAKE_CURRENT_SOURCE_DIR}/shell.html"

    # Export functions
    "SHELL:-s EXPORTED_RUNTIME_METHODS=['ccall','cwrap']"

    # Assertions
    "SHELL:-s ASSERTIONS=$<IF:$<CONFIG:Release>,0,1>"

    # File system
    "SHELL:-s FORCE_FILESYSTEM=1"

    # Minification for release
    $<$<CONFIG:Release>:--closure 1>
)
```

---

## 4. Handle GLAD Dependency

### Issue

GLAD is used for OpenGL function loading on desktop but not needed for Emscripten (uses built-in GL).

### Solution: Update application files

In `src/core/application_web.cpp` (already done):

```cpp
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/html5.h>
#define GLFW_INCLUDE_ES3
#else
#include <glad/glad.h>  // Desktop only
#endif

#include <GLFW/glfw3.h>
```

In `CMakeLists.web.txt` (already done):

```cmake
# GLAD - Create dummy interface library for web
add_library(glad INTERFACE)
```

---

## 5. Optional: Create Unified CMakeLists.txt

For projects wanting a single CMakeLists.txt that works for both builds:

```cmake
cmake_minimum_required(VERSION 3.15)
project(BallBalancer)

# Detect Emscripten
if(EMSCRIPTEN)
    message(STATUS "Configuring for Emscripten/WebAssembly build")
    set(BUILDING_FOR_WEB TRUE)
else()
    message(STATUS "Configuring for native build")
    set(BUILDING_FOR_WEB FALSE)
endif()

# Common settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Platform-specific configuration
if(BUILDING_FOR_WEB)
    # Web-specific settings
    set(CMAKE_EXECUTABLE_SUFFIX ".html")

    # Source files
    set(MAIN_SOURCE src/main_web.cpp)
    set(APP_SOURCE src/core/application_web.cpp)

    # Compiler flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s USE_WEBGL2=1 -s FULL_ES3=1")

else()
    # Desktop-specific settings
    set(MAIN_SOURCE src/main.cpp)
    set(APP_SOURCE src/core/application.cpp)

    # Find desktop dependencies
    find_package(OpenGL REQUIRED)
endif()

# ... rest of CMake configuration
```

---

## 6. Testing Modifications

### Compile Desktop Version

```bash
mkdir build-native
cd build-native
cmake ..
make -j8
./ball_balancer
```

Should work as before with no changes to functionality.

### Compile Web Version

```bash
./build_web.sh
cd build-web
python3 -m http.server 8000
# Open http://localhost:8000/ball_balancer.html
```

Should successfully compile and run in browser.

---

## 7. Debugging Compilation Issues

### Shader Compilation Errors

**Symptom:**
```
ERROR::SHADER::COMPILATION_FAILED
```

**Check:**
1. Verify shader files exist in `shaders/` directory
2. Ensure web shaders use `#version 300 es`
3. Check shader paths match in renderer.cpp

**Debug:**
```cpp
// Add to Shader::compile_shader()
std::cout << "Loading shader: " << path << std::endl;
```

### Linker Errors with GLAD

**Symptom:**
```
undefined reference to `gladLoadGL'
```

**Solution:**
Web build shouldn't link GLAD. Verify CMakeLists.web.txt has:
```cmake
add_library(glad INTERFACE)  # Not STATIC
```

### Memory Issues

**Symptom:**
```
Cannot enlarge memory arrays
```

**Solution:**
Increase in CMakeLists.web.txt:
```cmake
"SHELL:-s INITIAL_MEMORY=256MB"  # Increase from 128MB
```

---

## 8. Verification Checklist

Before committing changes:

- [ ] Desktop build still compiles: `cmake .. && make`
- [ ] Desktop executable runs: `./ball_balancer`
- [ ] Web build compiles: `./build_web.sh`
- [ ] Web shaders load correctly in browser console
- [ ] ImGui interface renders in browser
- [ ] 3D scene renders in browser
- [ ] Physics simulation runs at correct speed
- [ ] No WebGL errors in browser console
- [ ] All controls (reset, start, stop) work
- [ ] Plots update in real-time

---

## Summary of Files to Modify

| File | Modification | Required |
|------|--------------|----------|
| `src/rendering/renderer.cpp` | Add shader path conditionals | Yes |
| `src/rendering/shader.cpp` | Add version validation | Optional |
| `CMakeLists.web.txt` | Configure web build | Yes (already done) |
| `src/core/application.cpp` | Add GLAD conditionals | No (use application_web.cpp) |

---

## Additional Notes

### Why Separate application_web.cpp?

The main loop structure differs fundamentally:

**Desktop:**
```cpp
while (!glfwWindowShouldClose(window)) {
    update();
    render();
}
```

**Web:**
```cpp
emscripten_set_main_loop(update_callback, 0, 1);
// Browser controls the loop
```

Using separate files is cleaner than extensive `#ifdef` blocks.

### Build System Strategy

Two approaches:

1. **Separate CMakeLists** (Current approach)
   - `CMakeLists.txt` for desktop
   - `CMakeLists.web.txt` for web
   - Pros: Clean separation, no complex conditionals
   - Cons: Duplicate configuration

2. **Unified CMakeLists** (Alternative)
   - Single file with `if(EMSCRIPTEN)` blocks
   - Pros: DRY principle, single source of truth
   - Cons: More complex, harder to read

Both are valid. Current approach prioritizes clarity.

---

## Questions?

Refer to:
- `WEB_BUILD_GUIDE.md` - Complete build instructions
- `/home/nds/Projects/ida-ai/research/emscripten-web-compilation-best-practices.md` - Compilation guidelines
- Emscripten documentation: https://emscripten.org/docs/
