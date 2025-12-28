# External Dependencies

This directory contains external libraries required for the ball balancer project.

## ⚠️ Critical: imconfig.h

**`imconfig.h`** is already provided in this directory with 32-bit indices enabled:
```cpp
#define ImDrawIdx unsigned int
```

This is **REQUIRED** for ImPlot to work with high-density plots. Do NOT delete this file!

---

## Required External Libraries

The following libraries need to be cloned into this directory to build the project:

### 1. GLFW (Window Management)

```bash
cd external
git clone https://github.com/glfw/glfw.git
```

**Purpose:** Window creation, input handling, OpenGL context management
**Version:** 3.3+
**License:** zlib/libpng

### 2. ImGui (GUI Framework) - Docking Branch

```bash
cd external
git clone https://github.com/ocornut/imgui.git -b docking
```

**Purpose:** Immediate-mode GUI for control panel and UI
**Version:** Docking branch (for multi-window support)
**License:** MIT

**IMPORTANT:** After cloning, the `imconfig.h` from this directory will be used instead of ImGui's default. This is configured in CMakeLists.txt.

### 3. ImPlot (Plotting Library)

```bash
cd external
git clone https://github.com/epezent/implot.git
```

**Purpose:** Real-time plotting for visualization
**Version:** Latest
**License:** MIT

### 4. GoogleTest (Testing Framework) - Optional

```bash
cd external
git clone https://github.com/google/googletest.git
```

**Purpose:** Unit and integration testing
**Version:** Latest
**License:** BSD-3-Clause

---

## After Cloning Dependencies

Once all dependencies are cloned, your `external/` directory should look like:

```
external/
├── README.md          (this file)
├── imconfig.h         (critical config for ImGui/ImPlot)
├── glfw/              (cloned from GitHub)
├── imgui/             (cloned from GitHub, docking branch)
├── implot/            (cloned from GitHub)
└── googletest/        (optional, cloned from GitHub)
```

## CMake Integration

The project's `CMakeLists.txt` will automatically:
1. Use `external/imconfig.h` for ImGui configuration
2. Add subdirectories for each external library
3. Link against them appropriately

## Quick Setup Script

You can use this script to clone all dependencies at once:

```bash
#!/bin/bash
cd "$(dirname "$0")"  # Go to external directory

echo "Cloning external dependencies..."

# GLFW
if [ ! -d "glfw" ]; then
    echo "Cloning GLFW..."
    git clone https://github.com/glfw/glfw.git
else
    echo "GLFW already exists, skipping"
fi

# ImGui (docking branch)
if [ ! -d "imgui" ]; then
    echo "Cloning ImGui (docking branch)..."
    git clone https://github.com/ocornut/imgui.git -b docking
else
    echo "ImGui already exists, skipping"
fi

# ImPlot
if [ ! -d "implot" ]; then
    echo "Cloning ImPlot..."
    git clone https://github.com/epezent/implot.git
else
    echo "ImPlot already exists, skipping"
fi

# GoogleTest (optional)
if [ ! -d "googletest" ]; then
    echo "Cloning GoogleTest..."
    git clone https://github.com/google/googletest.git
else
    echo "GoogleTest already exists, skipping"
fi

echo ""
echo "External dependencies setup complete!"
echo ""
echo "NOTE: imconfig.h is already provided and configured."
echo "      Do NOT overwrite it with ImGui's default config!"
```

Save this as `setup_deps.sh` in the `external/` directory and run:
```bash
chmod +x setup_deps.sh
./setup_deps.sh
```

---

## Verification

After cloning all dependencies, verify the setup:

```bash
ls -la external/
```

You should see:
- ✅ `imconfig.h` (already present, do not modify)
- ✅ `glfw/` directory
- ✅ `imgui/` directory (docking branch)
- ✅ `implot/` directory
- ✅ `googletest/` directory (optional)

---

## Build Instructions

Once dependencies are set up:

```bash
cd ball-balancer
mkdir build
cd build
cmake ..
make -j$(nproc)
./ball_balancer
```

---

## Troubleshooting

### Error: "ImDrawIdx overflow" or visual glitches in plots

**Cause:** The `imconfig.h` file is not being used, or 32-bit indices are not enabled.

**Solution:**
1. Verify `external/imconfig.h` exists and contains `#define ImDrawIdx unsigned int`
2. Ensure CMakeLists.txt includes `external/` in include directories
3. Clean and rebuild: `rm -rf build && mkdir build && cd build && cmake .. && make`

### Error: Missing GLFW, ImGui, or ImPlot headers

**Cause:** External dependencies not cloned.

**Solution:** Run the setup script above or manually clone the repositories.

### Error: ImGui docking features not available

**Cause:** Wrong ImGui branch (master instead of docking).

**Solution:**
```bash
cd external/imgui
git checkout docking
cd ../..
```

---

## License Information

- **GLFW:** zlib/libpng license
- **ImGui:** MIT license
- **ImPlot:** MIT license
- **GoogleTest:** BSD-3-Clause license

All external libraries are used in compliance with their respective licenses.

---

**Last Updated:** 2025-12-10
**Maintained By:** Ball Balancer Project
