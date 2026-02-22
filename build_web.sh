#!/bin/bash

###############################################################################
# Ball Balancer - Emscripten Web Build Script
###############################################################################
#
# This script compiles the ball balancer application to WebAssembly using
# Emscripten. It handles the complete build process from CMake configuration
# to final WASM output.
#
# Prerequisites:
#   - Emscripten SDK installed and activated
#   - Run: source /path/to/emsdk/emsdk_env.sh
#
# Usage:
#   ./build_web.sh [release|debug]
#
# Output:
#   build-web/ball_balancer.html  - Main HTML file
#   build-web/ball_balancer.js    - JavaScript glue code
#   build-web/ball_balancer.wasm  - WebAssembly binary
#   build-web/ball_balancer.data  - Preloaded assets (shaders)
#
###############################################################################

set -e  # Exit on error

# Cleanup function to restore CMakeLists.txt on exit/interrupt
cleanup() {
    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    if [ -f "$script_dir/CMakeLists.txt.native" ]; then
        echo "Restoring original CMakeLists.txt..."
        mv "$script_dir/CMakeLists.txt.native" "$script_dir/CMakeLists.txt"
    fi
}

# Trap EXIT signal to ensure cleanup runs on script exit (normal or interrupted)
trap cleanup EXIT

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print with color
print_status() {
    echo -e "${BLUE}[BUILD]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

###############################################################################
# Configuration
###############################################################################

BUILD_TYPE="${1:-Release}"  # Release or Debug
BUILD_DIR="build-web"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_status "Ball Balancer Web Build Script"
print_status "Build type: $BUILD_TYPE"
print_status "Project directory: $PROJECT_DIR"
echo ""

###############################################################################
# Check Prerequisites
###############################################################################

print_status "Checking prerequisites..."

# Check if Emscripten is available
if ! command -v emcc &> /dev/null; then
    print_error "Emscripten not found!"
    echo "Please install and activate Emscripten SDK:"
    echo "  git clone https://github.com/emscripten-core/emsdk.git"
    echo "  cd emsdk"
    echo "  ./emsdk install latest"
    echo "  ./emsdk activate latest"
    echo "  source ./emsdk_env.sh"
    exit 1
fi

# Check Emscripten version
EMCC_VERSION=$(emcc --version | head -n 1)
print_success "Found $EMCC_VERSION"

# Check if CMake is available
if ! command -v cmake &> /dev/null; then
    print_error "CMake not found! Please install CMake."
    exit 1
fi

CMAKE_VERSION=$(cmake --version | head -n 1)
print_success "Found $CMAKE_VERSION"

echo ""

###############################################################################
# Prepare Build Directory
###############################################################################

print_status "Preparing build directory..."

cd "$PROJECT_DIR"

if [ -d "$BUILD_DIR" ]; then
    print_warning "Build directory exists, cleaning..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

print_success "Build directory ready: $BUILD_DIR"
echo ""

###############################################################################
# Copy Web Shaders
###############################################################################

print_status "Preparing web shaders..."

# Create shaders directory in build
mkdir -p shaders

# Copy web-specific shaders (renaming them to match what the code expects)
if [ -f "../shaders/basic_web.vert" ]; then
    cp ../shaders/basic_web.vert shaders/basic_web.vert
    cp ../shaders/basic_web.frag shaders/basic_web.frag
    cp ../shaders/grid_web.vert shaders/grid_web.vert
    cp ../shaders/grid_web.frag shaders/grid_web.frag
    print_success "Web shaders copied"
else
    print_error "Web shaders not found in ../shaders/"
    exit 1
fi

# Temporarily use CMakeLists.web.txt as CMakeLists.txt
cd "$PROJECT_DIR"
if [ ! -f "CMakeLists.txt.native" ]; then
    mv CMakeLists.txt CMakeLists.txt.native
fi
cp CMakeLists.web.txt CMakeLists.txt
print_success "Using CMakeLists.web.txt for configuration"

echo ""

###############################################################################
# Configure CMake
###############################################################################

print_status "Configuring CMake with Emscripten toolchain..."

# Use emcmake to configure (emcmake automatically sets the toolchain file)
# Point to the parent directory which now has CMakeLists.txt from CMakeLists.web.txt
emcmake cmake -S . -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_TESTS=OFF

if [ $? -ne 0 ]; then
    print_error "CMake configuration failed!"
    exit 1
fi

print_success "CMake configuration complete"
echo ""

###############################################################################
# Build
###############################################################################

print_status "Building with emmake..."

# Determine number of parallel jobs
if command -v nproc &> /dev/null; then
    JOBS=$(nproc)
elif command -v sysctl &> /dev/null; then
    JOBS=$(sysctl -n hw.ncpu)
else
    JOBS=4
fi

print_status "Using $JOBS parallel jobs"

# Build with emmake (need to be in the build directory)
cd "$PROJECT_DIR/$BUILD_DIR"
emmake make -j"$JOBS"

if [ $? -ne 0 ]; then
    print_error "Build failed!"
    exit 1
fi

print_success "Build complete"
echo ""

###############################################################################
# Verify Output
###############################################################################

print_status "Verifying output files..."

REQUIRED_FILES=(
    "ball_balancer.html"
    "ball_balancer.js"
    "ball_balancer.wasm"
)

OPTIONAL_FILES=(
    "ball_balancer.data"
)

ALL_FOUND=true
for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        SIZE=$(du -h "$file" | cut -f1)
        print_success "Found $file ($SIZE)"
    else
        print_error "Missing $file"
        ALL_FOUND=false
    fi
done

for file in "${OPTIONAL_FILES[@]}"; do
    if [ -f "$file" ]; then
        SIZE=$(du -h "$file" | cut -f1)
        print_success "Found $file ($SIZE) [optional]"
    else
        print_warning "Missing $file (optional - shaders may be loaded from filesystem)"
    fi
done

echo ""

if [ "$ALL_FOUND" = false ]; then
    print_error "Build verification failed - missing required files"
    exit 1
fi

###############################################################################
# Cleanup and Restore
###############################################################################

# Restore original CMakeLists.txt
cd "$PROJECT_DIR"
if [ -f "CMakeLists.txt.native" ]; then
    mv CMakeLists.txt.native CMakeLists.txt
    print_success "Restored original CMakeLists.txt"
fi

###############################################################################
# Summary
###############################################################################

print_success "=========================================="
print_success "  Web build completed successfully!"
print_success "=========================================="
echo ""
echo "Output files location:"
echo "  $PROJECT_DIR/$BUILD_DIR/"
echo ""
echo "To run the application:"
echo "  1. cd $BUILD_DIR"
echo "  2. python3 -m http.server 8000"
echo "  3. Open http://localhost:8000/ball_balancer.html in your browser"
echo ""
echo "For production deployment:"
echo "  - Ensure proper MIME types for .wasm files (application/wasm)"
echo "  - Enable gzip compression for .js, .wasm, and .data files"
echo "  - Use HTTPS for better performance (enables SharedArrayBuffer)"
echo ""

###############################################################################
# Optional: Start Development Server
###############################################################################

read -p "Start development server now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    print_status "Starting HTTP server on port 8000..."
    echo "Open http://localhost:8000/ball_balancer.html in your browser"
    echo "Press Ctrl+C to stop the server"
    echo ""
    python3 -m http.server 8000
fi
