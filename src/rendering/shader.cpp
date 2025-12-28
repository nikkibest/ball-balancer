#include <ball_balancer/rendering/shader.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

// Use GLAD for OpenGL function loading
#include <glad/glad.h>

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
    // Load shader sources from files
    std::string vertex_source = load_shader_source(vertex_path);
    std::string fragment_source = load_shader_source(fragment_path);

    if (vertex_source.empty() || fragment_source.empty()) {
        std::cerr << "ERROR::SHADER::FILE_NOT_FOUND" << std::endl;
        return;
    }

    // Compile shaders
    unsigned int vertex_id = compile_shader(vertex_source, GL_VERTEX_SHADER);
    unsigned int fragment_id = compile_shader(fragment_source, GL_FRAGMENT_SHADER);

    if (vertex_id == 0 || fragment_id == 0) {
        // Compilation failed - cleanup
        if (vertex_id != 0) glDeleteShader(vertex_id);
        if (fragment_id != 0) glDeleteShader(fragment_id);
        return;
    }

    // Link program
    program_id_ = link_program(vertex_id, fragment_id);

    // Cleanup shader objects (no longer needed after linking)
    glDeleteShader(vertex_id);
    glDeleteShader(fragment_id);

    if (program_id_ == 0) {
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED" << std::endl;
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
        std::cerr << "ERROR::SHADER::FILE_NOT_FOUND: " << path << std::endl;
        return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
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

        std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n"
                  << "Type: " << (type == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT") << "\n"
                  << info_log << std::endl;

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

        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
                  << info_log << std::endl;

        glDeleteProgram(program_id);
        return 0;
    }

    return program_id;
}

int Shader::get_uniform_location(const std::string& name) const {
    if (program_id_ == 0) {
        return -1;
    }

    int location = glGetUniformLocation(program_id_, name.c_str());

    if (location == -1) {
        std::cerr << "WARNING::SHADER::UNIFORM_NOT_FOUND: " << name << std::endl;
    }

    return location;
}

// ============================================================================
// Uniform Setters (Type-Safe)
// ============================================================================

void Shader::set_uniform(const std::string& name, bool value) const {
    glUniform1i(get_uniform_location(name), static_cast<int>(value));
}

void Shader::set_uniform(const std::string& name, int value) const {
    glUniform1i(get_uniform_location(name), value);
}

void Shader::set_uniform(const std::string& name, float value) const {
    glUniform1f(get_uniform_location(name), value);
}

void Shader::set_uniform(const std::string& name, const Eigen::Vector3f& value) const {
    glUniform3fv(get_uniform_location(name), 1, value.data());
}

void Shader::set_uniform(const std::string& name, const Eigen::Vector4f& value) const {
    glUniform4fv(get_uniform_location(name), 1, value.data());
}

void Shader::set_uniform(const std::string& name, const Eigen::Matrix3f& value) const {
    // Eigen matrices are column-major, OpenGL expects column-major, so no transpose
    glUniformMatrix3fv(get_uniform_location(name), 1, false, value.data());
}

void Shader::set_uniform(const std::string& name, const Eigen::Matrix4f& value) const {
    // Eigen matrices are column-major, OpenGL expects column-major, so no transpose
    glUniformMatrix4fv(get_uniform_location(name), 1, false, value.data());
}

} // namespace ball_balancer
