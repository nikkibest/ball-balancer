# Ball Balancer - WebAssembly Build Guide

## Overview

This guide provides comprehensive instructions for compiling the Ball Balancer simulation to WebAssembly using Emscripten, enabling it to run directly in web browsers.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Building](#building)
4. [Testing Locally](#testing-locally)
5. [Deployment](#deployment)
6. [Troubleshooting](#troubleshooting)
7. [Architecture](#architecture)

---

## Prerequisites

### Required Software

- **Emscripten SDK** (latest version, 3.1.50+)
- **CMake** 3.15 or higher
- **Python** 3.6+ (for local web server)
- **Modern web browser** with WebAssembly support
  - Chrome 57+
  - Firefox 52+
  - Safari 11+
  - Edge 16+

### System Requirements

- **Linux/macOS**: Recommended for development
- **Windows**: Use WSL2 or native Windows with Emscripten
- **RAM**: 4GB minimum, 8GB recommended
- **Disk Space**: ~2GB for Emscripten SDK + build artifacts

---

## Installation

### Step 1: Install Emscripten SDK

```bash
# Clone the Emscripten SDK repository
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk

# Install the latest version
./emsdk install latest

# Activate the SDK
./emsdk activate latest

# Add to current shell environment
source ./emsdk_env.sh
```

For permanent installation, add to your shell profile:

```bash
# Add to ~/.bashrc or ~/.zshrc
echo 'source "/path/to/emsdk/emsdk_env.sh"' >> ~/.bashrc
```

### Step 2: Verify Installation

```bash
# Check Emscripten compiler
emcc --version

# Should output something like:
# emcc (Emscripten gcc/clang-like replacement + linker emulating GNU ld) 3.1.50
```

### Step 3: Verify CMake

```bash
cmake --version

# Should output CMake 3.15 or higher
```

---

## Building

### Quick Build (Recommended)

Use the provided build script for automated building:

```bash
# Navigate to project directory
cd /home/nds/Projects/ball-balancer

# Run build script (Release mode by default)
./build_web.sh

# Or specify build type
./build_web.sh Release  # Optimized for production
./build_web.sh Debug    # With debugging symbols
```

The script will:
1. Verify prerequisites
2. Create build directory
3. Configure CMake with Emscripten
4. Compile C++ to WebAssembly
5. Generate HTML/JS/WASM files
6. Optionally start a development server

### Manual Build

For more control over the build process:

```bash
# 1. Create and enter build directory
mkdir -p build-web
cd build-web

# 2. Copy web shaders
mkdir -p shaders
cp ../shaders/basic_web.vert shaders/
cp ../shaders/basic_web.frag shaders/
cp ../shaders/grid_web.vert shaders/
cp ../shaders/grid_web.frag shaders/

# 3. Configure with emcmake
emcmake cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF

# 4. Build with emmake
emmake make -j8

# 5. Verify output files
ls -lh ball_balancer.*
```

### Build Options

#### Release Build (Production)

```bash
./build_web.sh Release
```

**Optimizations applied:**
- `-O3` aggressive optimization
- `--closure 1` JavaScript minification
- `-s ASSERTIONS=0` runtime checks disabled
- Smaller file sizes (WASM ~1-2MB)
- Maximum performance

#### Debug Build (Development)

```bash
./build_web.sh Debug
```

**Features:**
- `-O2` moderate optimization
- `-g` debug symbols
- `-s ASSERTIONS=1` runtime checks enabled
- Source maps for debugging
- Larger file sizes (WASM ~5-10MB)

### Build Output

Successful build produces:

```
build-web/
├── ball_balancer.html      # Main HTML page (200-300KB)
├── ball_balancer.js        # JavaScript glue code (100-200KB)
├── ball_balancer.wasm      # WebAssembly binary (1-10MB depending on build type)
└── ball_balancer.data      # Preloaded assets/shaders (10-50KB)
```

---

## Testing Locally

### Option 1: Python HTTP Server (Recommended)

```bash
# From build-web directory
cd build-web
python3 -m http.server 8000

# Open in browser:
# http://localhost:8000/ball_balancer.html
```

### Option 2: Node.js HTTP Server

```bash
# Install http-server globally
npm install -g http-server

# Run from build-web directory
cd build-web
http-server -p 8000

# Open in browser:
# http://localhost:8000/ball_balancer.html
```

### Option 3: PHP Built-in Server

```bash
cd build-web
php -S localhost:8000

# Open in browser:
# http://localhost:8000/ball_balancer.html
```

### Browser Developer Tools

**Enable console for debugging:**
1. Press `F12` to open Developer Tools
2. Navigate to Console tab
3. Check for errors or warnings
4. Application logs appear in real-time

**Performance profiling:**
1. Open Performance tab in DevTools
2. Click Record
3. Interact with simulation
4. Stop recording
5. Analyze frame times and bottlenecks

---

## Deployment

### Web Server Configuration

#### Apache (.htaccess)

```apache
# Enable WASM MIME type
AddType application/wasm .wasm

# Enable compression
<IfModule mod_deflate.c>
    AddOutputFilterByType DEFLATE application/wasm
    AddOutputFilterByType DEFLATE application/javascript
    AddOutputFilterByType DEFLATE text/html
</IfModule>

# Enable caching
<IfModule mod_expires.c>
    ExpiresActive On
    ExpiresByType application/wasm "access plus 1 year"
    ExpiresByType application/javascript "access plus 1 year"
    ExpiresByType text/html "access plus 1 week"
</IfModule>

# Enable CORS (if serving from CDN)
<IfModule mod_headers.c>
    Header set Access-Control-Allow-Origin "*"
</IfModule>
```

#### Nginx

```nginx
server {
    listen 80;
    server_name example.com;
    root /var/www/ball-balancer;

    # WASM MIME type
    types {
        application/wasm wasm;
    }

    # Enable gzip compression
    gzip on;
    gzip_types application/wasm application/javascript text/html;
    gzip_min_length 1000;

    # Caching
    location ~* \.(wasm|js|data)$ {
        expires 1y;
        add_header Cache-Control "public, immutable";
    }

    location / {
        try_files $uri $uri/ /ball_balancer.html;
    }
}
```

#### Static Hosting Services

**GitHub Pages:**
```bash
# 1. Create gh-pages branch
git checkout -b gh-pages

# 2. Copy build-web contents to root
cp build-web/* .

# 3. Commit and push
git add .
git commit -m "Deploy WebAssembly build"
git push origin gh-pages

# Access at: https://username.github.io/ball-balancer/ball_balancer.html
```

**Netlify:**
```bash
# 1. Create netlify.toml in project root
cat > netlify.toml << EOF
[build]
  publish = "build-web"

[[headers]]
  for = "/*.wasm"
  [headers.values]
    Content-Type = "application/wasm"
    Cache-Control = "public, max-age=31536000, immutable"
EOF

# 2. Deploy
netlify deploy --prod --dir=build-web
```

**Vercel:**
```bash
# 1. Install Vercel CLI
npm i -g vercel

# 2. Deploy
vercel --prod build-web
```

### Performance Optimization

**Enable compression:**
- Gzip reduces WASM files by ~40-60%
- Brotli compression gives even better results (~50-70%)

**CDN deployment:**
- Upload static files to CDN (CloudFlare, AWS CloudFront)
- Reduces latency for global users
- Automatic caching and compression

**Lazy loading:**
- Consider splitting large assets
- Load non-critical components on demand

---

## Troubleshooting

### Issue 1: "Emscripten not found"

**Error:**
```
emcc: command not found
```

**Solution:**
```bash
# Ensure Emscripten is in PATH
source /path/to/emsdk/emsdk_env.sh

# Verify
which emcc
```

---

### Issue 2: "Failed to compile shader"

**Error:**
```
ERROR::SHADER::COMPILATION_FAILED
```

**Causes:**
- Wrong shader version (using GLSL 450 instead of 300 es)
- Missing web shaders

**Solution:**
```bash
# Verify web shaders exist
ls shaders/*_web.*

# Should show:
# basic_web.vert, basic_web.frag, grid_web.vert, grid_web.frag

# Rebuild with proper shader paths
./build_web.sh
```

---

### Issue 3: "WASM streaming compile failed"

**Error in browser console:**
```
wasm streaming compile failed: TypeError: Failed to fetch
```

**Causes:**
- Using `file://` protocol instead of HTTP
- Incorrect MIME type for WASM files

**Solution:**
```bash
# Always use HTTP server
cd build-web
python3 -m http.server 8000

# Configure server MIME types (see Deployment section)
```

---

### Issue 4: "Cannot enlarge memory arrays"

**Error:**
```
Cannot enlarge memory arrays. Either (1) compile with -s ALLOW_MEMORY_GROWTH=1...
```

**Solution:**
Already configured in CMakeLists.web.txt:
```cmake
"SHELL:-s ALLOW_MEMORY_GROWTH=1"
"SHELL:-s INITIAL_MEMORY=128MB"
```

If still occurs, increase initial memory in CMakeLists.web.txt.

---

### Issue 5: Performance Issues

**Symptoms:**
- Low FPS (<30)
- Laggy controls
- Long load times

**Solutions:**

1. **Build in Release mode:**
   ```bash
   ./build_web.sh Release
   ```

2. **Check browser GPU acceleration:**
   - Chrome: `chrome://gpu`
   - Enable hardware acceleration in settings

3. **Reduce canvas resolution:**
   Edit shell.html:
   ```javascript
   canvas.width = 1024;  // Reduced from 1280
   canvas.height = 576;  // Reduced from 720
   ```

4. **Profile with DevTools:**
   - Identify bottlenecks in Performance tab
   - Check for memory leaks in Memory tab

---

### Issue 6: ImGui Not Rendering

**Symptoms:**
- GUI panels missing
- Only 3D scene visible

**Causes:**
- ImGui backend not initialized for WebGL
- Wrong OpenGL ES version

**Solution:**
Verify in imgui library definition (CMakeLists.web.txt):
```cmake
target_compile_definitions(imgui PUBLIC
    IMGUI_IMPL_OPENGL_ES3
)
```

---

## Architecture

### Key Differences from Desktop Build

| Component | Desktop | Web |
|-----------|---------|-----|
| **OpenGL Version** | 4.5 Core | ES 3.0 (WebGL 2.0) |
| **Shaders** | GLSL 450 | GLSL 300 es |
| **Main Loop** | `while` loop | `emscripten_set_main_loop` |
| **File Loading** | Direct filesystem | Preloaded virtual FS |
| **Window** | GLFW native | GLFW Emscripten port |
| **OpenGL Loader** | GLAD | Emscripten built-in |

### File Structure

```
ball-balancer/
├── CMakeLists.txt              # Desktop build
├── CMakeLists.web.txt          # Web build (Emscripten)
├── build_web.sh                # Automated build script
├── shell.html                  # HTML template
├── shaders/
│   ├── basic.vert              # Desktop shaders (GLSL 450)
│   ├── basic.frag
│   ├── basic_web.vert          # Web shaders (GLSL 300 es)
│   ├── basic_web.frag
│   ├── grid.vert
│   ├── grid.frag
│   ├── grid_web.vert
│   └── grid_web.frag
├── src/
│   ├── main.cpp                # Desktop entry point
│   ├── main_web.cpp            # Web entry point
│   └── core/
│       ├── application.cpp     # Desktop application
│       └── application_web.cpp # Web application (browser loop)
└── build-web/                  # Build output
    ├── ball_balancer.html
    ├── ball_balancer.js
    ├── ball_balancer.wasm
    └── ball_balancer.data
```

### WebAssembly Compilation Flow

```
┌─────────────────┐
│  C++ Source     │
│  (.cpp, .hpp)   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  em++ Compiler  │
│  (Clang/LLVM)   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  LLVM IR        │
│  (Intermediate) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Binaryen       │
│  (Optimizer)    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  .wasm Binary   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  JavaScript     │
│  Glue Code (.js)│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  HTML Shell     │
│  (User Interface│
└─────────────────┘
```

### Browser Runtime Architecture

```
┌────────────────────────────────────────┐
│           Web Browser                   │
│  ┌──────────────────────────────────┐  │
│  │  HTML (shell.html)               │  │
│  │  - Canvas element                │  │
│  │  - Loading screen                │  │
│  │  - Status display                │  │
│  └───────────────┬──────────────────┘  │
│                  │                      │
│  ┌───────────────▼──────────────────┐  │
│  │  JavaScript (ball_balancer.js)   │  │
│  │  - Module initialization         │  │
│  │  - Event handlers                │  │
│  │  - WebGL context                 │  │
│  └───────────────┬──────────────────┘  │
│                  │                      │
│  ┌───────────────▼──────────────────┐  │
│  │  WebAssembly (ball_balancer.wasm)│  │
│  │  ┌────────────────────────────┐  │  │
│  │  │ Application Loop           │  │  │
│  │  │ - Physics simulation       │  │  │
│  │  │ - PID controller           │  │  │
│  │  │ - State estimator          │  │  │
│  │  │ - OpenGL ES rendering      │  │  │
│  │  │ - ImGui interface          │  │  │
│  │  └────────────────────────────┘  │  │
│  └──────────────────────────────────┘  │
│                                         │
│  ┌──────────────────────────────────┐  │
│  │  Virtual File System (.data)     │  │
│  │  - Preloaded shaders             │  │
│  │  - Other assets                  │  │
│  └──────────────────────────────────┘  │
└────────────────────────────────────────┘
```

---

## Additional Resources

### Official Documentation

- [Emscripten Documentation](https://emscripten.org/docs/)
- [WebAssembly Official Site](https://webassembly.org/)
- [WebGL 2.0 Specification](https://www.khronos.org/webgl/)
- [ImGui Web Examples](https://github.com/ocornut/imgui/tree/master/examples)

### Best Practices Reference

See `/home/nds/Projects/ida-ai/research/emscripten-web-compilation-best-practices.md` for detailed compilation guidelines.

### Community Support

- [Emscripten GitHub Discussions](https://github.com/emscripten-core/emscripten/discussions)
- [WebAssembly Discord](https://discord.gg/webassembly)
- [Stack Overflow - WebAssembly Tag](https://stackoverflow.com/questions/tagged/webassembly)

---

## License

Same as main project license.

## Contributors

Built following Emscripten best practices and WebGL 2.0 specifications.
