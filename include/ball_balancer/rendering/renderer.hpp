#pragma once

#include <ball_balancer/core/types.hpp>
#include <ball_balancer/rendering/shader.hpp>
#include <ball_balancer/rendering/camera.hpp>
#include <imgui.h>
#include <array>
#include <memory>
#include <vector>

// OpenGL error checking macro (debug builds only)
#ifndef NDEBUG
    #include <iostream>
    #define GL_CHECK(call) do { \
        call; \
        GLenum err = glGetError(); \
        if (err != GL_NO_ERROR) { \
            std::cerr << "OpenGL error " << err << " at " \
                      << __FILE__ << ":" << __LINE__ << '\n'; \
        } \
    } while(0)
#else
    #define GL_CHECK(call) call
#endif

/**
 * @file renderer.hpp
 * @brief OpenGL renderer for ball balancer visualization
 *
 * Renders 3D scene with:
 * - Ball (sphere at current position)
 * - Table (tilted platform)
 * - Grid floor (reference plane)
 * - Coordinate axes (X=red, Y=green, Z=blue)
 *
 * Following best practices from research/opengl-rendering-best-practices.md:
 * - DSA (Direct State Access) for OpenGL 4.5+
 * - RAII for GPU resources (VAO, VBO)
 * - Batching to minimize draw calls
 * - Efficient geometry generation
 *
 * @see research/opengl-rendering-best-practices.md
 */

namespace ball_balancer {

/**
 * @brief Vertex data structure
 */
struct Vertex {
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector3f color;
};

/**
 * @brief RAII wrapper for OpenGL Vertex Array Object
 */
class VertexArray {
public:
    VertexArray();
    ~VertexArray();

    // Non-copyable, movable
    VertexArray(const VertexArray&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;
    VertexArray(VertexArray&& other) noexcept;
    VertexArray& operator=(VertexArray&& other) noexcept;

    void bind() const;
    void unbind() const;

    unsigned int get_vao() const { return vao_; }
    unsigned int get_vbo() const { return vbo_; }
    size_t get_vertex_count() const { return vertex_count_; }

    /**
     * @brief Upload vertex data to GPU
     * @param vertices Vertex data
     */
    void set_data(const std::vector<Vertex>& vertices);

private:
    unsigned int vao_{0};  // Vertex Array Object
    unsigned int vbo_{0};  // Vertex Buffer Object
    size_t vertex_count_{0};
};

/**
 * @brief Main OpenGL renderer for ball balancer
 *
 * Manages all rendering resources and provides high-level render() method.
 * Follows RAII - all GPU resources cleaned up in destructor.
 */
class Renderer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct renderer
     */
    Renderer();

    /**
     * @brief Destructor - cleanup GPU resources
     */
    ~Renderer() = default;

    // Non-copyable (owns GPU resources)
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    /**
     * @brief Initialize OpenGL resources
     * @param width Viewport width
     * @param height Viewport height
     * @return true if successful
     *
     * Must be called after OpenGL context is created.
     * Creates shaders, geometry, and sets up rendering state.
     */
    bool initialize(int width, int height);

    /**
     * @brief Render scene with current ball and table states.
     *
     * Renders:
     * - Ball at position (ball.x, ball.y, ball.z_ball)
     * - Table tilted by (table.phi, table.theta) at height table.z_t
     * - Grid floor
     * - Coordinate axes
     */
    void render(const BallState& ball, const TableState& table);

    /**
     * @brief Update viewport size
     * @param width New width
     * @param height New height
     */
    void resize(int width, int height);

    /**
     * @brief Rebuild the table disc mesh with a new radius.
     *
     * Call whenever arm_Rt changes so the rendered disc matches the
     * simulator boundary and leg attachment radius.
     */
    void set_table_radius(float radius);

    /**
     * @brief Render axis labels "X", "Y", "Z" as an ImGui overlay
     * @param clip_min Top-left of the region to clip labels to (screen pixels)
     * @param clip_max Bottom-right of the region to clip labels to
     *
     * Projects axis tips using the renderer's own matrices and draws labels
     * via the foreground draw list, clipped to [clip_min, clip_max] so they
     * don't overdraw side panels or plot windows.
     */
    void render_axis_labels(ImVec2 clip_min, ImVec2 clip_max);

    /**
     * @brief Render the three arm legs as shaded 3D cylinders with sphere joints.
     *
     * Each arm is drawn as two cylinder links (G→E lower, E→T upper) with a sphere
     * at the elbow joint. Uses Phong lighting via basic_shader_.
     * Arm 0 = cyan, Arm 1 = yellow, Arm 2 = magenta (lower link is darker).
     *
     * @param arm_points  World-space points in physics coordinates for each arm.
     *                    Layout: arm_points[i] = { G, E, T } for arm i (i=0,1,2).
     *                    Points are in physics space (X horizontal, Y horizontal, Z up).
     *                    Renderer remaps to OpenGL: GL(x,y,z) = (phys_x, phys_z, phys_y).
     *
     * Does nothing if show_legs_ is false.
     */
    void render_legs(const std::array<std::array<std::array<float, 3>, 3>, 3>& arm_points);

    /**
     * @brief Show or hide the arm leg geometry.
     */
    void set_show_legs(bool show) { show_legs_ = show; }
    bool get_show_legs() const    { return show_legs_; }

    /**
     * @brief Get camera for user interaction
     * @return Reference to camera
     */
    Camera& get_camera() { return camera_; }
    const Camera& get_camera() const { return camera_; }

private:
    /**
     * @brief Create sphere geometry
     * @param radius Sphere radius
     * @param segments Number of segments (latitude/longitude)
     * @return Vertex data for sphere
     */
    std::vector<Vertex> create_sphere(float radius, int segments = 32);

    /**
     * @brief Create circular disc geometry (table top)
     * @param radius Disc radius (matches arm_Rt)
     * @param segments Number of triangular segments (64 for smooth appearance)
     * @return Vertex data for disc (fan of triangles in XZ plane)
     */
    std::vector<Vertex> create_disc(float radius, int segments = 64);

    /**
     * @brief Create grid floor geometry
     * @param size Grid size
     * @param divisions Number of grid lines
     * @return Vertex data for grid lines
     */
    std::vector<Vertex> create_grid(float size, int divisions);

    /**
     * @brief Create coordinate axes geometry
     * @param length Axis length
     * @return Vertex data for XYZ axes
     */
    std::vector<Vertex> create_axes(float length);

    /**
     * @brief Create unit cylinder geometry (axis along Y, radius=1, Y in [-0.5, +0.5])
     * @param segments Number of radial segments (16 is sufficient)
     * @return Vertex data for cylinder (sides + caps), white colour (overridden by uColor)
     *
     * The unit dimensions let a single matrix (R*S + T) place any cylinder between
     * two world-space points via the stretch-and-orient technique.
     */
    std::vector<Vertex> create_cylinder(int segments = 16);

    // Rendering state
    int width_;
    int height_;

    // Camera
    Camera camera_;

    // Cached view-projection matrix from the most recent render() call.
    // render_axis_labels() reuses this so labels are guaranteed to use the
    // exact same transform as the 3D geometry.
    Eigen::Matrix4f last_vp_;

    // Shaders (RAII)
    std::unique_ptr<Shader> basic_shader_;
    std::unique_ptr<Shader> grid_shader_;

    // Geometry (RAII)
    std::unique_ptr<VertexArray> ball_mesh_;
    std::unique_ptr<VertexArray> table_mesh_;
    std::unique_ptr<VertexArray> grid_mesh_;
    std::unique_ptr<VertexArray> axes_mesh_;
    std::unique_ptr<VertexArray> cylinder_mesh_;     ///< Unit cylinder (Y axis, radius=1) for arm links
    std::unique_ptr<VertexArray> joint_sphere_mesh_; ///< Small sphere for elbow joint visualization

    // Leg visibility toggle
    bool show_legs_{true};

    // System parameters
    SystemParameters params_;
};

} // namespace ball_balancer
