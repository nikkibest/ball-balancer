#include <ball_balancer/rendering/renderer.hpp>
#include <cmath>
#include <iostream>

/**
 * @file renderer.cpp
 * @brief Implementation of OpenGL renderer
 *
 * Geometry generation algorithms:
 * - Sphere: UV sphere with lat/long segments
 * - Plane: Simple quad
 * - Grid: Line strips for floor grid
 * - Axes: Line segments for X/Y/Z axes
 *
 * @see research/opengl-rendering-best-practices.md
 */

namespace ball_balancer {

// Forward declarations for OpenGL functions (placeholders)
extern "C" {
    void glGenVertexArrays(int n, unsigned int* arrays);
    void glGenBuffers(int n, unsigned int* buffers);
    void glBindVertexArray(unsigned int array);
    void glBindBuffer(unsigned int target, unsigned int buffer);
    void glBufferData(unsigned int target, long size, const void* data, unsigned int usage);
    void glVertexAttribPointer(unsigned int index, int size, unsigned int type,
                              unsigned char normalized, int stride, const void* pointer);
    void glEnableVertexAttribArray(unsigned int index);
    void glDeleteVertexArrays(int n, const unsigned int* arrays);
    void glDeleteBuffers(int n, const unsigned int* buffers);
    void glDrawArrays(unsigned int mode, int first, int count);
    void glClear(unsigned int mask);
    void glClearColor(float r, float g, float b, float a);
    void glEnable(unsigned int cap);
    void glViewport(int x, int y, int width, int height);
}

// OpenGL constants (placeholders)
#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#define GL_STATIC_DRAW 0x88E4
#define GL_FLOAT 0x1406
#define GL_TRIANGLES 0x0004
#define GL_LINES 0x0001
#define GL_DEPTH_TEST 0x0B71
#define GL_COLOR_BUFFER_BIT 0x00004000
#define GL_DEPTH_BUFFER_BIT 0x00000100
#endif

// ============================================================================
// VertexArray Implementation
// ============================================================================

VertexArray::VertexArray() {
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
}

VertexArray::~VertexArray() {
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
}

VertexArray::VertexArray(VertexArray&& other) noexcept
    : vao_(other.vao_)
    , vbo_(other.vbo_)
    , vertex_count_(other.vertex_count_)
{
    other.vao_ = 0;
    other.vbo_ = 0;
    other.vertex_count_ = 0;
}

VertexArray& VertexArray::operator=(VertexArray&& other) noexcept {
    if (this != &other) {
        // Delete current resources
        if (vao_ != 0) glDeleteVertexArrays(1, &vao_);
        if (vbo_ != 0) glDeleteBuffers(1, &vbo_);

        // Transfer ownership
        vao_ = other.vao_;
        vbo_ = other.vbo_;
        vertex_count_ = other.vertex_count_;

        other.vao_ = 0;
        other.vbo_ = 0;
        other.vertex_count_ = 0;
    }
    return *this;
}

void VertexArray::bind() const {
    glBindVertexArray(vao_);
}

void VertexArray::unbind() const {
    glBindVertexArray(0);
}

void VertexArray::set_data(const std::vector<Vertex>& vertices) {
    vertex_count_ = vertices.size();

    // Bind VAO
    bind();

    // Bind and upload VBO
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 vertices.size() * sizeof(Vertex),
                 vertices.data(),
                 GL_STATIC_DRAW);

    // Vertex attributes
    // Layout: position (location=0), normal (location=1), color (location=2)

    // Position attribute (3 floats)
    glVertexAttribPointer(0, 3, GL_FLOAT, false, sizeof(Vertex),
                         reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(0);

    // Normal attribute (3 floats)
    glVertexAttribPointer(1, 3, GL_FLOAT, false, sizeof(Vertex),
                         reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);

    // Color attribute (3 floats)
    glVertexAttribPointer(2, 3, GL_FLOAT, false, sizeof(Vertex),
                         reinterpret_cast<void*>(offsetof(Vertex, color)));
    glEnableVertexAttribArray(2);

    unbind();
}

// ============================================================================
// Renderer Implementation
// ============================================================================

Renderer::Renderer()
    : width_(800)
    , height_(600)
    , camera_(Eigen::Vector3f::Zero(), 3.5f, 0.785f, 0.785f)  // Further back and higher angle
{
    params_.initialize();
}

bool Renderer::initialize(int width, int height) {
    width_ = width;
    height_ = height;

    // Initialize OpenGL state
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);  // Dark blue background
    glViewport(0, 0, width_, height_);

    // Create shaders
    basic_shader_ = std::make_unique<Shader>("shaders/basic.vert", "shaders/basic.frag");
    grid_shader_ = std::make_unique<Shader>("shaders/grid.vert", "shaders/grid.frag");

    if (!basic_shader_->is_valid() || !grid_shader_->is_valid()) {
        std::cerr << "ERROR::RENDERER::SHADER_INITIALIZATION_FAILED" << std::endl;
        return false;
    }

    // Create geometry
    ball_mesh_ = std::make_unique<VertexArray>();
    ball_mesh_->set_data(create_sphere(params_.ball_radius, 32));

    table_mesh_ = std::make_unique<VertexArray>();
    table_mesh_->set_data(create_plane(params_.table_length, params_.table_width));

    grid_mesh_ = std::make_unique<VertexArray>();
    grid_mesh_->set_data(create_grid(2.0f, 20));

    axes_mesh_ = std::make_unique<VertexArray>();
    axes_mesh_->set_data(create_axes(0.5f));

    return true;
}

void Renderer::render(const StateVector& state) {
    // Clear framebuffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Get camera matrices
    float aspect = static_cast<float>(width_) / static_cast<float>(height_);
    Eigen::Matrix4f view = camera_.get_view_matrix();
    Eigen::Matrix4f proj = Camera::get_projection_matrix(aspect);

    // ========================================================================
    // Render Grid Floor
    // ========================================================================
    {
        grid_shader_->use();

        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

        grid_shader_->set_uniform("uModel", model);
        grid_shader_->set_uniform("uView", view);
        grid_shader_->set_uniform("uProjection", proj);
        grid_shader_->set_uniform("uColor", Eigen::Vector3f(0.3f, 0.3f, 0.3f));

        grid_mesh_->bind();
        glDrawArrays(GL_LINES, 0, grid_mesh_->get_vertex_count());
        grid_mesh_->unbind();
    }

    // ========================================================================
    // Render Coordinate Axes
    // ========================================================================
    {
        grid_shader_->use();

        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

        grid_shader_->set_uniform("uModel", model);
        grid_shader_->set_uniform("uView", view);
        grid_shader_->set_uniform("uProjection", proj);
        grid_shader_->set_uniform("uColor", Eigen::Vector3f(1.0f, 1.0f, 1.0f));

        axes_mesh_->bind();
        glDrawArrays(GL_LINES, 0, axes_mesh_->get_vertex_count());
        axes_mesh_->unbind();
    }

    // ========================================================================
    // Render Table (tilted platform)
    // ========================================================================
    {
        basic_shader_->use();

        // Table transformation: rotate by theta_x and theta_y
        float theta_x = state(state_index::THETA_X);
        float theta_y = state(state_index::THETA_Y);

        // Rotation around X axis
        Eigen::Matrix4f rot_x = Eigen::Matrix4f::Identity();
        rot_x(1, 1) = std::cos(theta_x);
        rot_x(1, 2) = -std::sin(theta_x);
        rot_x(2, 1) = std::sin(theta_x);
        rot_x(2, 2) = std::cos(theta_x);

        // Rotation around Y axis
        Eigen::Matrix4f rot_y = Eigen::Matrix4f::Identity();
        rot_y(0, 0) = std::cos(theta_y);
        rot_y(0, 2) = std::sin(theta_y);
        rot_y(2, 0) = -std::sin(theta_y);
        rot_y(2, 2) = std::cos(theta_y);

        Eigen::Matrix4f model = rot_y * rot_x;  // Apply rotations
        Eigen::Matrix3f normal_matrix = model.topLeftCorner<3, 3>();

        basic_shader_->set_uniform("uModel", model);
        basic_shader_->set_uniform("uView", view);
        basic_shader_->set_uniform("uProjection", proj);
        basic_shader_->set_uniform("uNormalMatrix", normal_matrix);
        basic_shader_->set_uniform("uColor", Eigen::Vector3f(0.7f, 0.5f, 0.3f));  // Wood color
        basic_shader_->set_uniform("uLightPos", Eigen::Vector3f(2.0f, 3.0f, 2.0f));
        basic_shader_->set_uniform("uViewPos", camera_.get_position());

        table_mesh_->bind();
        glDrawArrays(GL_TRIANGLES, 0, table_mesh_->get_vertex_count());
        table_mesh_->unbind();
    }

    // ========================================================================
    // Render Ball
    // ========================================================================
    {
        basic_shader_->use();

        // Ball position from state
        float x = state(state_index::X);
        float y = state(state_index::Y);
        float z = params_.ball_radius;  // Ball sits on table surface

        // Translation to ball position
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
        model(0, 3) = x;
        model(1, 3) = z;  // Y is up in OpenGL
        model(2, 3) = y;

        Eigen::Matrix3f normal_matrix = model.topLeftCorner<3, 3>();

        basic_shader_->set_uniform("uModel", model);
        basic_shader_->set_uniform("uView", view);
        basic_shader_->set_uniform("uProjection", proj);
        basic_shader_->set_uniform("uNormalMatrix", normal_matrix);
        basic_shader_->set_uniform("uColor", Eigen::Vector3f(0.9f, 0.1f, 0.1f));  // Red ball
        basic_shader_->set_uniform("uLightPos", Eigen::Vector3f(2.0f, 3.0f, 2.0f));
        basic_shader_->set_uniform("uViewPos", camera_.get_position());

        ball_mesh_->bind();
        glDrawArrays(GL_TRIANGLES, 0, ball_mesh_->get_vertex_count());
        ball_mesh_->unbind();
    }
}

void Renderer::resize(int width, int height) {
    width_ = width;
    height_ = height;
    glViewport(0, 0, width_, height_);
}

// ============================================================================
// Geometry Generation
// ============================================================================

std::vector<Vertex> Renderer::create_sphere(float radius, int segments) {
    std::vector<Vertex> vertices;

    const float PI = 3.14159265f;
    const Eigen::Vector3f color(1.0f, 0.5f, 0.2f);  // Orange for ball

    // UV sphere generation
    for (int lat = 0; lat <= segments; ++lat) {
        float theta = lat * PI / segments;
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);

        for (int lon = 0; lon <= segments; ++lon) {
            float phi = lon * 2.0f * PI / segments;
            float sin_phi = std::sin(phi);
            float cos_phi = std::cos(phi);

            Vertex v;
            v.position = Eigen::Vector3f(
                radius * sin_theta * cos_phi,
                radius * cos_theta,
                radius * sin_theta * sin_phi
            );
            v.normal = v.position.normalized();
            v.color = color;

            vertices.push_back(v);
        }
    }

    // Convert to triangles (simplified - actual implementation needs indices)
    return vertices;
}

std::vector<Vertex> Renderer::create_plane(float width, float height) {
    std::vector<Vertex> vertices;

    const Eigen::Vector3f color(0.6f, 0.7f, 0.8f);  // Light blue for table
    const Eigen::Vector3f normal(0.0f, 1.0f, 0.0f);  // Up

    float hw = width / 2.0f;
    float hh = height / 2.0f;

    // Two triangles forming a quad
    // Triangle 1
    vertices.push_back({Eigen::Vector3f(-hw, 0.0f, -hh), normal, color});
    vertices.push_back({Eigen::Vector3f( hw, 0.0f, -hh), normal, color});
    vertices.push_back({Eigen::Vector3f( hw, 0.0f,  hh), normal, color});

    // Triangle 2
    vertices.push_back({Eigen::Vector3f(-hw, 0.0f, -hh), normal, color});
    vertices.push_back({Eigen::Vector3f( hw, 0.0f,  hh), normal, color});
    vertices.push_back({Eigen::Vector3f(-hw, 0.0f,  hh), normal, color});

    return vertices;
}

std::vector<Vertex> Renderer::create_grid(float size, int divisions) {
    std::vector<Vertex> vertices;

    const Eigen::Vector3f color(0.3f, 0.3f, 0.3f);  // Gray
    const Eigen::Vector3f normal(0.0f, 1.0f, 0.0f);

    float step = size / divisions;
    float half = size / 2.0f;

    // Grid lines parallel to X axis
    for (int i = 0; i <= divisions; ++i) {
        float z = -half + i * step;
        vertices.push_back({Eigen::Vector3f(-half, -0.01f, z), normal, color});
        vertices.push_back({Eigen::Vector3f( half, -0.01f, z), normal, color});
    }

    // Grid lines parallel to Z axis
    for (int i = 0; i <= divisions; ++i) {
        float x = -half + i * step;
        vertices.push_back({Eigen::Vector3f(x, -0.01f, -half), normal, color});
        vertices.push_back({Eigen::Vector3f(x, -0.01f,  half), normal, color});
    }

    return vertices;
}

std::vector<Vertex> Renderer::create_axes(float length) {
    std::vector<Vertex> vertices;

    const Eigen::Vector3f normal(0.0f, 1.0f, 0.0f);

    // X axis (red)
    vertices.push_back({Eigen::Vector3f(0.0f, 0.0f, 0.0f), normal, Eigen::Vector3f(1.0f, 0.0f, 0.0f)});
    vertices.push_back({Eigen::Vector3f(length, 0.0f, 0.0f), normal, Eigen::Vector3f(1.0f, 0.0f, 0.0f)});

    // Y axis (green)
    vertices.push_back({Eigen::Vector3f(0.0f, 0.0f, 0.0f), normal, Eigen::Vector3f(0.0f, 1.0f, 0.0f)});
    vertices.push_back({Eigen::Vector3f(0.0f, length, 0.0f), normal, Eigen::Vector3f(0.0f, 1.0f, 0.0f)});

    // Z axis (blue)
    vertices.push_back({Eigen::Vector3f(0.0f, 0.0f, 0.0f), normal, Eigen::Vector3f(0.0f, 0.0f, 1.0f)});
    vertices.push_back({Eigen::Vector3f(0.0f, 0.0f, length), normal, Eigen::Vector3f(0.0f, 0.0f, 1.0f)});

    return vertices;
}

} // namespace ball_balancer
