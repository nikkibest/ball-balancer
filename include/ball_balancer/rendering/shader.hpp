#pragma once

#include <string>
#include <Eigen/Dense>

/**
 * @file shader.hpp
 * @brief RAII wrapper for OpenGL shader programs
 *
 * Following best practices from research/opengl-rendering-best-practices.md:
 * - RAII for GPU resource management
 * - DSA (Direct State Access) when possible
 * - Proper error checking and validation
 *
 * @see research/opengl-rendering-best-practices.md
 * @see research/cpp-best-practices-modern-programming.md (RAII)
 */

namespace ball_balancer {

/**
 * @brief RAII wrapper for OpenGL shader program
 *
 * Manages shader compilation, linking, and cleanup automatically.
 * Follows RAII principles - shader is deleted in destructor.
 *
 * Usage:
 * ```cpp
 * Shader shader("basic.vert", "basic.frag");
 * if (!shader.is_valid()) {
 *     // Handle error
 * }
 * shader.use();
 * shader.set_uniform("MVP", mvp_matrix);
 * // Draw...
 * ```
 */
class Shader {
public:
    /**
     * @brief Construct shader from file paths
     * @param vertex_path Path to vertex shader source
     * @param fragment_path Path to fragment shader source
     *
     * Automatically compiles and links shaders.
     * Check is_valid() to verify success.
     */
    Shader(const std::string& vertex_path, const std::string& fragment_path);

    /**
     * @brief Destructor - deletes shader program
     *
     * RAII: Automatically cleans up GPU resources.
     */
    ~Shader();

    // Non-copyable (owns GPU resource)
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    // Movable (transfer ownership)
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    /**
     * @brief Use this shader program for rendering
     */
    void use() const;

    /**
     * @brief Check if shader compiled and linked successfully
     * @return true if valid, false if errors occurred
     */
    bool is_valid() const { return program_id_ != 0; }

    /**
     * @brief Get OpenGL program ID
     * @return Shader program handle
     */
    unsigned int get_program_id() const { return program_id_; }

    // ========================================================================
    // Uniform Setters (type-safe)
    // ========================================================================

    void set_uniform(const std::string& name, bool value) const;
    void set_uniform(const std::string& name, int value) const;
    void set_uniform(const std::string& name, float value) const;
    void set_uniform(const std::string& name, const Eigen::Vector3f& value) const;
    void set_uniform(const std::string& name, const Eigen::Vector4f& value) const;
    void set_uniform(const std::string& name, const Eigen::Matrix3f& value) const;
    void set_uniform(const std::string& name, const Eigen::Matrix4f& value) const;

private:
    /**
     * @brief Load shader source from file
     * @param path File path
     * @return Shader source code
     */
    std::string load_shader_source(const std::string& path) const;

    /**
     * @brief Compile shader from source
     * @param source Shader source code
     * @param type GL_VERTEX_SHADER or GL_FRAGMENT_SHADER
     * @return Shader ID (0 if failed)
     */
    unsigned int compile_shader(const std::string& source, unsigned int type) const;

    /**
     * @brief Link vertex and fragment shaders into program
     * @param vertex_id Compiled vertex shader ID
     * @param fragment_id Compiled fragment shader ID
     * @return Program ID (0 if failed)
     */
    unsigned int link_program(unsigned int vertex_id, unsigned int fragment_id) const;

    /**
     * @brief Get uniform location (cached)
     * @param name Uniform name
     * @return Uniform location (-1 if not found)
     */
    int get_uniform_location(const std::string& name) const;

    unsigned int program_id_{0};  // OpenGL program handle
};

} // namespace ball_balancer
