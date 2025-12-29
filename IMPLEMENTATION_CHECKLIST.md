# Implementation Checklist - Web Build Setup

This checklist guides you through applying all necessary changes to enable WebAssembly compilation.

---

## Phase 1: Verify Files (Already Complete)

These files have been created and are ready to use:

- [x] `CMakeLists.web.txt` - Emscripten build configuration
- [x] `build_web.sh` - Automated build script
- [x] `shell.html` - HTML template
- [x] `src/main_web.cpp` - Web entry point
- [x] `src/core/application_web.cpp` - Web application loop
- [x] `shaders/basic_web.vert` - WebGL vertex shader
- [x] `shaders/basic_web.frag` - WebGL fragment shader
- [x] `shaders/grid_web.vert` - Grid vertex shader
- [x] `shaders/grid_web.frag` - Grid fragment shader
- [x] `.gitignore` - Updated with web build artifacts
- [x] `WEB_BUILD_GUIDE.md` - Comprehensive documentation
- [x] `CODE_MODIFICATIONS_FOR_WEB.md` - Modification guide
- [x] `QUICK_START_WEB.md` - Quick reference
- [x] `WEB_DEPLOYMENT_SUMMARY.md` - Complete summary

---

## Phase 2: Apply Code Modifications

### Task 1: Modify `src/rendering/renderer.cpp`

**Status:** ⚠️ REQUIRED

**Location:** `/home/nds/Projects/ball-balancer/src/rendering/renderer.cpp`

#### Step 1.1: Add preprocessor directives at top

Find this section (around line 17):
```cpp
namespace ball_balancer {
```

Add BEFORE the namespace:
```cpp
#ifdef __EMSCRIPTEN__
#define SHADER_PATH_PREFIX "/shaders/"
#define SHADER_SUFFIX "_web"
#else
#define SHADER_PATH_PREFIX "shaders/"
#define SHADER_SUFFIX ""
#endif

namespace ball_balancer {
```

#### Step 1.2: Update shader loading

Find these lines (around line 163-164):
```cpp
basic_shader_ = std::make_unique<Shader>("shaders/basic.vert", "shaders/basic.frag");
grid_shader_ = std::make_unique<Shader>("shaders/grid.vert", "shaders/grid.frag");
```

Replace with:
```cpp
#ifdef __EMSCRIPTEN__
    basic_shader_ = std::make_unique<Shader>("/shaders/basic_web.vert", "/shaders/basic_web.frag");
    grid_shader_ = std::make_unique<Shader>("/shaders/grid_web.vert", "/shaders/grid_web.frag");
#else
    basic_shader_ = std::make_unique<Shader>("shaders/basic.vert", "shaders/basic.frag");
    grid_shader_ = std::make_unique<Shader>("shaders/grid.vert", "shaders/grid.frag");
#endif
```

**Verification:**
```bash
grep -n "basic_web" src/rendering/renderer.cpp
# Should show the new shader paths
```

---

### Task 2: Verify Desktop Build Still Works

**Status:** ⚠️ REQUIRED

Run desktop build to ensure modifications don't break native compilation:

```bash
# Clean build
rm -rf build
mkdir build
cd build

# Configure
cmake ..

# Compile
make -j8

# Test
./ball_balancer
```

**Expected result:** Application runs normally on desktop.

**If fails:** Review changes in renderer.cpp, ensure `#ifdef __EMSCRIPTEN__` blocks are correct.

---

## Phase 3: Install Emscripten

### Task 3: Install Emscripten SDK

**Status:** ⚠️ REQUIRED (if not already installed)

```bash
# Navigate to home directory or preferred location
cd ~

# Clone Emscripten SDK
git clone https://github.com/emscripten-core/emsdk.git

# Enter directory
cd emsdk

# Install latest version
./emsdk install latest

# Activate
./emsdk activate latest

# Add to current shell
source ./emsdk_env.sh

# Verify installation
emcc --version
```

**Expected output:**
```
emcc (Emscripten gcc/clang-like replacement + linker emulating GNU ld) 3.1.50
```

**If fails:** Check internet connection, try `./emsdk install 3.1.50` for specific version.

---

### Task 4: Add Emscripten to Shell Profile (Optional but Recommended)

**Status:** ⏸️ OPTIONAL

To avoid running `source emsdk_env.sh` every time:

```bash
# For bash
echo 'source "$HOME/emsdk/emsdk_env.sh"' >> ~/.bashrc

# For zsh
echo 'source "$HOME/emsdk/emsdk_env.sh"' >> ~/.zshrc
```

Then restart your terminal or run:
```bash
source ~/.bashrc  # or ~/.zshrc
```

---

## Phase 4: Build for Web

### Task 5: Run Automated Build

**Status:** ⚠️ REQUIRED

```bash
# Navigate to project directory
cd /home/nds/Projects/ball-balancer

# Make build script executable (if not already)
chmod +x build_web.sh

# Run build script
./build_web.sh Release
```

**Expected output:**
```
[BUILD] Ball Balancer Web Build Script
[BUILD] Build type: Release
[SUCCESS] Found emcc (Emscripten gcc/clang-like replacement...) 3.1.50
[BUILD] Configuring CMake with Emscripten toolchain...
[SUCCESS] CMake configuration complete
[BUILD] Building with emmake...
[BUILD] Using 8 parallel jobs
[SUCCESS] Build complete
[SUCCESS] Found ball_balancer.html (250K)
[SUCCESS] Found ball_balancer.js (150K)
[SUCCESS] Found ball_balancer.wasm (1.5M)
[SUCCESS] Found ball_balancer.data (20K)
[SUCCESS] ==========================================
[SUCCESS]   Web build completed successfully!
[SUCCESS] ==========================================
```

**If fails:**
- Check error messages carefully
- Verify Emscripten is in PATH: `which emcc`
- Check shader files exist: `ls shaders/*_web.*`
- Review CMakeLists.web.txt for syntax errors

---

### Task 6: Verify Build Output

**Status:** ⚠️ REQUIRED

```bash
cd build-web
ls -lh ball_balancer.*
```

**Expected files:**
```
ball_balancer.html
ball_balancer.js
ball_balancer.wasm
ball_balancer.data
```

**Check file sizes:**
- `.wasm` should be 1-2 MB (Release) or 5-10 MB (Debug)
- `.js` should be 100-200 KB
- `.html` should be 200-300 KB
- `.data` should be 10-50 KB

---

## Phase 5: Test Locally

### Task 7: Start Development Server

**Status:** ⚠️ REQUIRED

```bash
# From build-web directory
cd build-web
python3 -m http.server 8000
```

**Expected output:**
```
Serving HTTP on 0.0.0.0 port 8000 (http://0.0.0.0:8000/) ...
```

---

### Task 8: Open in Browser

**Status:** ⚠️ REQUIRED

1. Open browser (Chrome, Firefox, or Safari)
2. Navigate to: `http://localhost:8000/ball_balancer.html`
3. Wait for loading screen to disappear (3-10 seconds)

**Expected result:**
- Loading screen with progress bar
- Application loads successfully
- 3D scene visible (grid, table, ball)
- ImGui control panel on left/right
- No console errors (F12 → Console tab)

---

### Task 9: Functional Testing

**Status:** ⚠️ REQUIRED

Test each feature:

- [ ] **Loading:** Page loads without errors
- [ ] **3D Rendering:** Scene renders correctly (ball, table, grid)
- [ ] **Camera:** Mouse drag/scroll works to rotate/zoom
- [ ] **ImGui:** Control panel renders and is interactive
- [ ] **Start Button:** Simulation starts when clicked
- [ ] **Physics:** Ball moves realistically on tilted table
- [ ] **Plots:** Real-time data plots update
- [ ] **Stop Button:** Simulation pauses
- [ ] **Reset Button:** Simulation resets to initial state
- [ ] **Performance:** Runs at 50+ FPS (check DevTools)

---

### Task 10: Browser Console Check

**Status:** ⚠️ REQUIRED

1. Press F12 to open Developer Tools
2. Go to Console tab
3. Check for errors (should see startup messages only)

**Acceptable messages:**
```
Ball Balancer Simulation (Web)
System Parameters:
  Ball mass: 27 g
  ...
WebAssembly runtime initialized
Simulation ready!
```

**Unacceptable (errors to fix):**
```
ERROR::SHADER::COMPILATION_FAILED
WebGL: CONTEXT_LOST_WEBGL
Failed to fetch .wasm
```

If errors appear, see Troubleshooting section in WEB_BUILD_GUIDE.md.

---

## Phase 6: Performance Optimization

### Task 11: Measure Performance

**Status:** ⏸️ OPTIONAL

In browser console:
```javascript
// Check FPS
performance.now()
// Interact with simulation for ~10 seconds
performance.now()  // Subtract to get time for X frames
```

**Target:** 50-60 FPS (16-20ms per frame)

---

### Task 12: Optimize if Needed

**Status:** ⏸️ OPTIONAL

If FPS < 40:

1. **Reduce canvas resolution** in shell.html:
   ```javascript
   canvas.width = 1024;   // Was 1280
   canvas.height = 576;   // Was 720
   ```

2. **Ensure Release build:**
   ```bash
   ./build_web.sh Release  # Not Debug
   ```

3. **Check GPU acceleration:**
   - Chrome: `chrome://gpu`
   - Enable hardware acceleration in browser settings

---

## Phase 7: Deployment Preparation

### Task 13: Test on Multiple Browsers

**Status:** ⏸️ RECOMMENDED

Test on:
- [ ] Chrome/Chromium
- [ ] Firefox
- [ ] Safari (if on macOS)
- [ ] Edge (if on Windows)
- [ ] Mobile browser (phone/tablet)

---

### Task 14: Choose Deployment Method

**Status:** ⏸️ WHEN READY TO DEPLOY

Pick one:

#### Option A: GitHub Pages
```bash
git checkout -b gh-pages
cp build-web/* .
git add . && git commit -m "Deploy web build"
git push origin gh-pages
```

#### Option B: Netlify
```bash
npm install -g netlify-cli
netlify deploy --prod --dir=build-web
```

#### Option C: Own Server
Upload `build-web/` contents to web server with:
- MIME type `application/wasm` for .wasm files
- gzip compression enabled
- HTTPS enabled (optional but recommended)

---

## Phase 8: Final Verification

### Task 15: Production Deployment Check

**Status:** ⏸️ AFTER DEPLOYMENT

- [ ] Deployed URL loads successfully
- [ ] HTTPS enabled (check for lock icon)
- [ ] .wasm file downloads correctly (Network tab in DevTools)
- [ ] Application runs at acceptable speed
- [ ] Works on mobile devices
- [ ] Share URL with friends and verify they can access it

---

## Troubleshooting Quick Reference

| Issue | Solution |
|-------|----------|
| `emcc: command not found` | Run `source ~/emsdk/emsdk_env.sh` |
| Shader compilation fails | Verify `shaders/*_web.*` files exist |
| WASM won't load | Use HTTP server, not `file://` |
| Low FPS | Build with Release mode |
| ImGui not visible | Check IMGUI_IMPL_OPENGL_ES3 defined |
| Memory errors | Increase INITIAL_MEMORY in CMakeLists.web.txt |

**For detailed troubleshooting:** See WEB_BUILD_GUIDE.md Section 6.

---

## Success Criteria

You're done when:

1. ✅ All required tasks completed
2. ✅ Desktop build still works
3. ✅ Web build compiles without errors
4. ✅ Application runs in browser at 50+ FPS
5. ✅ All features functional (rendering, GUI, physics, controls)
6. ✅ No console errors
7. ✅ Works on multiple browsers

---

## Getting Help

If stuck:

1. **Check documentation:**
   - WEB_BUILD_GUIDE.md (comprehensive)
   - CODE_MODIFICATIONS_FOR_WEB.md (code changes)
   - QUICK_START_WEB.md (quick reference)

2. **Review best practices:**
   - `/home/nds/Projects/ida-ai/research/emscripten-web-compilation-best-practices.md`

3. **Emscripten resources:**
   - [Official Docs](https://emscripten.org/docs/)
   - [GitHub Discussions](https://github.com/emscripten-core/emscripten/discussions)

4. **Common issues:**
   - Search error message in browser console
   - Check WEB_BUILD_GUIDE.md troubleshooting section
   - Verify all files from Phase 1 exist

---

## Progress Tracking

**Current Status:** Ready to begin implementation

**Next Step:** Start with Task 1 (Modify renderer.cpp)

**Estimated Time:**
- Code modifications: 15-30 minutes
- Emscripten installation: 15-30 minutes
- First build: 5-10 minutes
- Testing: 15-30 minutes
- **Total: 1-2 hours**

---

**Good luck with your web build!**
