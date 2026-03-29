---
name: web-deploy
description: >
  Builds the ball-balancer project for WebAssembly using Emscripten and deploys
  the output to the nikkibest.github.io GitHub Pages portfolio. Trigger with
  phrases like "build for web", "deploy to GitHub Pages", "web build", or
  "publish to portfolio". Do NOT trigger for desktop builds or general CMake tasks.
---

# Web Deploy

Compiles the ball-balancer C++ application to WebAssembly via Emscripten,
copies the output (`.js`, `.wasm`) into the GitHub Pages portfolio project,
then commits and pushes the deployment automatically.

## When to Use This Skill

- "build for web" / "web build" / "build WASM"
- "deploy to GitHub Pages" / "publish to portfolio" / "update the site"
- "build and share" / "build and copy files over"

**Not triggered by:**
- Desktop CMake builds (`cmake --build`, `make`, etc.)
- Questions about the build system or CMakeLists

## Goal

Produce an up-to-date WebAssembly build and publish it live to the GitHub Pages
portfolio at `nikkibest.github.io/projects/ball-balancer`.

## Connectors / Dependencies

- `emcc` — Emscripten compiler (must be on PATH; activated via `source emsdk_env.sh`)
- `./build_web.sh` — project build script at repo root
- `/home/nds/Projects/nikkibest.github.io/` — GitHub Pages repo
- `git` — for committing and pushing the portfolio repo

## Process

### Step 1 — Verify Emscripten is active
Run `emcc --version`. If not found, print instructions to activate emsdk and stop.
- **HITL:** No
- **Reference file:** none
- **Output:** Version string printed, or error + instructions.

### Step 2 — Sync CMakeLists source lists
Before building, check that all `.cpp` files under `src/` are listed in
`CMakeLists.web.txt`. Flag any missing files (common cause of linker errors) and
add them automatically.
- **HITL:** No — add missing files silently; report what was added.
- **Reference file:** `CMakeLists.txt` (desktop) as the source of truth for the full file list.
- **Output:** Updated `CMakeLists.web.txt` if needed.

### Step 3 — Run the web build
Run `echo "n" | ./build_web.sh` from the project root. Stream output and watch
for errors. On failure, diagnose the error (missing source, include, or linker
symbol) and fix it before re-running.
- **HITL:** No — retry automatically for known fixable errors (missing source files, missing library in CMakeLists.web.txt).
- **Reference file:** `CMakeLists.web.txt`, `build_web.sh`
- **Output:** `build-web/ball_balancer.js` and `build-web/ball_balancer.wasm`

### Step 4 — Copy output to GitHub Pages
Copy `ball_balancer.js` and `ball_balancer.wasm` from `build-web/` to
`/home/nds/Projects/nikkibest.github.io/projects/ball-balancer/`.
Do NOT overwrite `index.html` or the `shaders/` directory.
- **HITL:** No
- **Reference file:** none
- **Output:** Two files updated in the portfolio repo.

### Step 5 — Commit and push
In `/home/nds/Projects/nikkibest.github.io/`, stage only the two WASM files,
commit with a descriptive message summarising what changed, then push to `origin main`.
- **HITL:** No
- **Reference file:** none
- **Output:** Commit created and pushed; confirm with commit hash and remote URL.

## Rules

1. Never overwrite `index.html` or `shaders/` in the GitHub Pages repo.
2. Never commit files other than `ball_balancer.js` and `ball_balancer.wasm` in the portfolio repo.
3. Always answer "n" to the dev-server prompt in `build_web.sh`.
4. If `CMakeLists.txt` (desktop) is temporarily swapped out by `build_web.sh` and not restored, restore it before finishing.
5. Always check both CMakeLists files are in sync for source file lists.

## Failure Modes

- **`emcc` not found:** Print activation instructions (`source ~/emsdk/emsdk_env.sh`) and stop.
- **Linker errors (undefined symbols):** Find the `.cpp` file defining the missing symbol, add it to `CMakeLists.web.txt`, and retry.
- **Missing `#include` (header not found):** Find the library and add it to `CMakeLists.web.txt` includes/targets.
- **`build_web.sh` leaves `CMakeLists.txt` swapped:** Restore from `CMakeLists.txt.native`.
- **GitHub Pages push fails:** Report the git error without retrying destructively.

## Progressive Updates

When a new source file or library is added to the project and causes a build
failure, update this SKILL.md's Rules section to note the pattern for future runs.
