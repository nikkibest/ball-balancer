# Quick Start - Web Build

## Prerequisites

```bash
# Install Emscripten
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh
```

## Build

```bash
# One command build
./build_web.sh

# Or manual
mkdir build-web && cd build-web
emcmake cmake .. -DCMAKE_BUILD_TYPE=Release
emmake make -j8
```

## Run

```bash
cd build-web
python3 -m http.server 8000
# Open: http://localhost:8000/ball_balancer.html
```

## Files Created

```
build-web/
├── ball_balancer.html  → Open this in browser
├── ball_balancer.js    → JavaScript glue code
├── ball_balancer.wasm  → WebAssembly binary
└── ball_balancer.data  → Preloaded shaders
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `emcc: command not found` | Run `source /path/to/emsdk/emsdk_env.sh` |
| Shader errors | Verify `shaders/*_web.*` files exist |
| WASM won't load | Use HTTP server, not `file://` |
| Low performance | Build with `./build_web.sh Release` |
| Memory errors | Increase `INITIAL_MEMORY` in CMakeLists.web.txt |

## Code Changes Required

### 1. In `src/rendering/renderer.cpp`

Change shader loading (line ~163):

```cpp
#ifdef __EMSCRIPTEN__
    basic_shader_ = std::make_unique<Shader>("/shaders/basic_web.vert", "/shaders/basic_web.frag");
    grid_shader_ = std::make_unique<Shader>("/shaders/grid_web.vert", "/shaders/grid_web.frag");
#else
    basic_shader_ = std::make_unique<Shader>("shaders/basic.vert", "shaders/basic.frag");
    grid_shader_ = std::make_unique<Shader>("shaders/grid.vert", "shaders/grid.frag");
#endif
```

### 2. Verify CMakeLists.web.txt uses web sources

```cmake
src/core/application_web.cpp  # Not application.cpp
src/main_web.cpp              # Not main.cpp
```

## Deploy

### GitHub Pages

```bash
git checkout -b gh-pages
cp build-web/* .
git add . && git commit -m "Deploy" && git push origin gh-pages
```

### Netlify

```bash
netlify deploy --prod --dir=build-web
```

## Documentation

- Full guide: `WEB_BUILD_GUIDE.md`
- Code changes: `CODE_MODIFICATIONS_FOR_WEB.md`
- Best practices: `/home/nds/Projects/ida-ai/research/emscripten-web-compilation-best-practices.md`
