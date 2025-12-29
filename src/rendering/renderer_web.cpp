// This file contains the web-specific shader path modifications
// Include this in the web build instead of renderer.cpp's shader initialization

// In renderer.cpp, the shader paths need to be changed for web builds:
// Desktop: "shaders/basic.vert", "shaders/basic.frag"
// Web: "/shaders/basic_web.vert", "/shaders/basic_web.frag"

// The easiest approach is to use preprocessor directives in renderer.cpp
// or create this wrapper that redefines the initialize function

#ifdef __EMSCRIPTEN__

// This is a minimal modification guide for renderer.cpp
// Add this at the top of renderer.cpp:

#ifdef __EMSCRIPTEN__
#define SHADER_PATH_PREFIX "/shaders/"
#define SHADER_SUFFIX "_web"
#else
#define SHADER_PATH_PREFIX "shaders/"
#define SHADER_SUFFIX ""
#endif

// Then modify the shader loading lines to:
// basic_shader_ = std::make_unique<Shader>(
//     SHADER_PATH_PREFIX "basic" SHADER_SUFFIX ".vert",
//     SHADER_PATH_PREFIX "basic" SHADER_SUFFIX ".frag"
// );
//
// grid_shader_ = std::make_unique<Shader>(
//     SHADER_PATH_PREFIX "grid" SHADER_SUFFIX ".vert",
//     SHADER_PATH_PREFIX "grid" SHADER_SUFFIX ".frag"
// );

#endif
