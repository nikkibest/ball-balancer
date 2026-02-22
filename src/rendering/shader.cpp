#include <ball_balancer/rendering/shader.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

// Use GLAD for OpenGL function loading (desktop) or GLES3 (Emscripten)
#ifdef __EMSCRIPTEN__
    #include <GLES3/gl3.h>
    #include <emscripten.h>
#else
    #include <glad/glad.h>
#endif

/**
 * @file shader.cpp
 * @brief Implementation of RAII shader wrapper
 *
 * Following OpenGL best practices:
 * - Check compilation/linking errors
 * - RAII for resource cleanup
 * - Type-safe uniform setters
 * - Clear error messages
 *
 * @see research/opengl-rendering-best-practices.md
 */

namespace ball_balancer {

Shader::Shader(const std::string& vertex_path, const std::string& fragment_path)
    : program_id_(0)
{
#ifndef NDEBUG
    #ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_CONSOLE, "[SHADER] Initializing shader: vertex=%s, fragment=%s",
                       vertex_path.c_str(), fragment_path.c_str());
    #endif
        std::cout << "[SHADER] Initializing shader: vertex=" << vertex_path
                  << ", fragment=" << fragment_path << '\n';
#endif

    // Load shader sources from files
    std::string vertex_source = load_shader_source(vertex_path);
    std::string fragment_source = load_shader_source(fragment_path);

    if (vertex_source.empty() || fragment_source.empty()) {
#ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_ERROR, "[SHADER] ERROR: Shader source files not found or empty!");
#endif
        std::cerr << "ERROR::SHADER::FILE_NOT_FOUND" << '\n';
        return;
    }

#ifndef NDEBUG
    #ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_CONSOLE, "[SHADER] Loaded vertex shader (%d bytes), fragment shader (%d bytes)",
                       (int)vertex_source.size(), (int)fragment_source.size());
    #endif
        std::cout << "[SHADER] Loaded vertex shader (" << vertex_source.size()
                  << " bytes), fragment shader (" << fragment_source.size() << " bytes)" << '\n';
#endif

    // Compile shaders
    unsigned int vertex_id = compile_shader(vertex_source, GL_VERTEX_SHADER);
    unsigned int fragment_id = compile_shader(fragment_source, GL_FRAGMENT_SHADER);

    if (vertex_id == 0 || fragment_id == 0) {
#ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_ERROR, "[SHADER] ERROR: Shader compilation failed!");
#endif
        // Compilation failed - cleanup
        if (vertex_id != 0) glDeleteShader(vertex_id);
        if (fragment_id != 0) glDeleteShader(fragment_id);
        return;
    }

#ifndef NDEBUG
    #ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_CONSOLE, "[SHADER] Shaders compiled successfully. Linking program...");
    #endif
        std::cout << "[SHADER] Shaders compiled successfully. Linking program..." << '\n';
#endif

    // Link program
    program_id_ = link_program(vertex_id, fragment_id);

    // Cleanup shader objects (no longer needed after linking)
    glDeleteShader(vertex_id);
    glDeleteShader(fragment_id);

    if (program_id_ == 0) {
#ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_ERROR, "[SHADER] ERROR: Program linking failed!");
#endif
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED" << '\n';
    } else {
#ifndef NDEBUG
        #ifdef __EMSCRIPTEN__
            emscripten_log(EM_LOG_CONSOLE, "[SHADER] Program linked successfully! ID=%d", program_id_);
        #endif
            std::cout << "[SHADER] Program linked successfully! ID=" << program_id_ << '\n';
#endif
    }
}

Shader::~Shader() {
    if (program_id_ != 0) {
        glDeleteProgram(program_id_);
        program_id_ = 0;
    }
}

Shader::Shader(Shader&& other) noexcept
    : program_id_(other.program_id_)
    , uniform_cache_(std::move(other.uniform_cache_))
{
    other.program_id_ = 0;  // Transfer ownership
}

Shader& Shader::operator=(Shader&& other) noexcept {
    if (this != &other) {
        // Delete current resource
        if (program_id_ != 0) {
            glDeleteProgram(program_id_);
        }

        // Transfer ownership
        program_id_ = other.program_id_;
        uniform_cache_ = std::move(other.uniform_cache_);
        other.program_id_ = 0;
    }
    return *this;
}

void Shader::use() const {
    if (program_id_ != 0) {
        glUseProgram(program_id_);
    }
}

std::string Shader::load_shader_source(const std::string& path) const {
    std::ifstream file(path);
    if (!file.is_open()) {
#ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_ERROR, "[SHADER] ERROR: Failed to open shader file: %s", path.c_str());
#endif
        std::cerr << "ERROR::SHADER::FILE_NOT_FOUND: " << path << '\n';
        return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string source = buffer.str();

#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[SHADER] Loaded shader file: %s (%d bytes)", path.c_str(), (int)source.size());
#endif

    return source;
}

unsigned int Shader::compile_shader(const std::string& source, unsigned int type) const {
    // Create shader object
    unsigned int shader_id = glCreateShader(type);

    // Set source and compile
    const char* source_cstr = source.c_str();
    glShaderSource(shader_id, 1, &source_cstr, nullptr);
    glCompileShader(shader_id);

    // Check compilation status
    int success;
    glGetShaderiv(shader_id, GL_COMPILE_STATUS, &success);

    if (!success) {
        // Get error log
        int log_length;
        glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &log_length);

        std::string info_log(log_length, ' ');
        glGetShaderInfoLog(shader_id, log_length, nullptr, &info_log[0]);

        const char* shader_type = (type == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT");
#ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_ERROR, "[SHADER] %s SHADER COMPILATION FAILED:\n%s",
                       shader_type, info_log.c_str());
#endif
        std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n"
                  << "Type: " << shader_type << "\n"
                  << info_log << '\n';

        glDeleteShader(shader_id);
        return 0;
    }

    return shader_id;
}

unsigned int Shader::link_program(unsigned int vertex_id, unsigned int fragment_id) const {
    // Create program
    unsigned int program_id = glCreateProgram();

    // Attach shaders
    glAttachShader(program_id, vertex_id);
    glAttachShader(program_id, fragment_id);

    // Link program
    glLinkProgram(program_id);

    // Check linking status
    int success;
    glGetProgramiv(program_id, GL_LINK_STATUS, &success);

    if (!success) {
        // Get error log
        int log_length;
        glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &log_length);

        std::string info_log(log_length, ' ');
        glGetProgramInfoLog(program_id, log_length, nullptr, &info_log[0]);

#ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_ERROR, "[SHADER] PROGRAM LINKING FAILED:\n%s", info_log.c_str());
#endif
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
                  << info_log << '\n';

        glDeleteProgram(program_id);
        return 0;
    }

    return program_id;
}

int Shader::get_uniform_location(const std::string& name) const {
    if (program_id_ == 0) {
        return -1;
    }

    // Check cache first
    auto it = uniform_cache_.find(name);
    if (it != uniform_cache_.end()) {
        return it->second;
    }

    // Not in cache, query OpenGL
    int location = glGetUniformLocation(program_id_, name.c_str());

    // Cache the result (even if -1, to avoid repeated warnings)
    uniform_cache_[name] = location;

    // Warn only on first lookup if not found
    if (location == -1) {
        std::cerr << "WARNING::SHADER::UNIFORM_NOT_FOUND: " << name << '\n';
    }

    return location;
}

// ============================================================================
// Uniform Setters (Type-Safe)
// ============================================================================

void Shader::set_uniform(const std::string& name, bool value) const {
    int location = get_uniform_location(name);
    if (location == -1) return;
    glUniform1i(location, static_cast<int>(value));
}

void Shader::set_uniform(const std::string& name, int value) const {
    int location = get_uniform_location(name);
    if (location == -1) return;
    glUniform1i(location, value);
}

void Shader::set_uniform(const std::string& name, float value) const {
    int location = get_uniform_location(name);
    if (location == -1) return;
    glUniform1f(location, value);
}

void Shader::set_uniform(const std::string& name, const Eigen::Vector3f& value) const {
    int location = get_uniform_location(name);
    if (location == -1) return;
    glUniform3fv(location, 1, value.data());
}

void Shader::set_uniform(const std::string& name, const Eigen::Vector4f& value) const {
    int location = get_uniform_location(name);
    if (location == -1) return;
    glUniform4fv(location, 1, value.data());
}

void Shader::set_uniform(const std::string& name, const Eigen::Matrix3f& value) const {
    // Eigen matrices are column-major, OpenGL expects column-major, so no transpose
    int location = get_uniform_location(name);
    if (location == -1) return;
    glUniformMatrix3fv(location, 1, false, value.data());
}

void Shader::set_uniform(const std::string& name, const Eigen::Matrix4f& value) const {
    // Eigen matrices are column-major, OpenGL expects column-major, so no transpose
    int location = get_uniform_location(name);
    if (location == -1) return;
    glUniformMatrix4fv(location, 1, false, value.data());
}

} // namespace ball_balancer
