#include <ball_balancer/rendering/renderer.hpp>
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Use GLAD for OpenGL function loading (desktop) or GLES3 (Emscripten)
#ifdef __EMSCRIPTEN__
    #include <GLES3/gl3.h>
    #include <emscripten.h>
#else
    #include <glad/glad.h>
#endif

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
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(0);

    // Normal attribute (3 floats)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);

    // Color attribute (3 floats)
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
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
    , camera_(Eigen::Vector3f(0.0f, 0.3f, 0.0f), 2.0f, 0.785f, 0.524f)  // Look at table center (slight Y offset), reasonable distance
{
    params_.initialize();
}

bool Renderer::initialize(int width, int height) {
    width_ = width;
    height_ = height;

#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Initializing renderer: %dx%d", width, height);
#endif
    std::cout << "[RENDERER] Initializing renderer: " << width << "x" << height << '\n';

    // Initialize OpenGL state
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);  // Dark blue background
    glViewport(0, 0, width_, height_);

    // Note: Backface culling disabled for simplicity
    // Could enable with glEnable(GL_CULL_FACE) if GLAD is extended with those functions

#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] OpenGL state initialized");
#endif
    std::cout << "[RENDERER] OpenGL state initialized" << '\n';

    // Create shaders - use platform-specific paths
    // WebAssembly builds use WebGL-compatible shaders (GLSL ES 300)
    // Desktop builds use modern OpenGL shaders (GLSL 450)
#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Loading WebGL shaders...");
    basic_shader_ = std::make_unique<Shader>("/shaders/basic_web.vert", "/shaders/basic_web.frag");
    grid_shader_ = std::make_unique<Shader>("/shaders/grid_web.vert", "/shaders/grid_web.frag");
#else
    std::cout << "[RENDERER] Loading desktop shaders..." << '\n';
    basic_shader_ = std::make_unique<Shader>("shaders/basic.vert", "shaders/basic.frag");
    grid_shader_ = std::make_unique<Shader>("shaders/grid.vert", "shaders/grid.frag");
#endif

    if (!basic_shader_->is_valid() || !grid_shader_->is_valid()) {
#ifdef __EMSCRIPTEN__
        emscripten_log(EM_LOG_ERROR, "[RENDERER] ERROR: Shader initialization failed!");
        emscripten_log(EM_LOG_ERROR, "[RENDERER] basic_shader valid: %d, grid_shader valid: %d",
                       basic_shader_->is_valid(), grid_shader_->is_valid());
#endif
        std::cerr << "ERROR::RENDERER::SHADER_INITIALIZATION_FAILED" << '\n';
        std::cerr << "  basic_shader valid: " << basic_shader_->is_valid() << '\n';
        std::cerr << "  grid_shader valid: " << grid_shader_->is_valid() << '\n';
        return false;
    }

#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Shaders loaded successfully");
#endif
    std::cout << "[RENDERER] Shaders loaded successfully" << '\n';

    // Create geometry
#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Creating geometry...");
#endif
    std::cout << "[RENDERER] Creating geometry..." << '\n';

    ball_mesh_ = std::make_unique<VertexArray>();
    std::vector<Vertex> sphere_vertices = create_sphere(params_.ball_radius, 32);  // 32 segments for smooth appearance
#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Created sphere with radius=%f, %d vertices", params_.ball_radius, (int)sphere_vertices.size());
#endif
    std::cout << "[RENDERER] Created sphere with radius=" << params_.ball_radius << ", " << sphere_vertices.size() << " vertices" << '\n';
    ball_mesh_->set_data(sphere_vertices);

    table_mesh_ = std::make_unique<VertexArray>();
    std::vector<Vertex> plane_vertices = create_plane(params_.table_length, params_.table_width);
#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Created plane with %d vertices", (int)plane_vertices.size());
#endif
    std::cout << "[RENDERER] Created plane with " << plane_vertices.size() << " vertices" << '\n';
    table_mesh_->set_data(plane_vertices);

    grid_mesh_ = std::make_unique<VertexArray>();
    std::vector<Vertex> grid_vertices = create_grid(2.0f, 20);
#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Created grid with %d vertices", (int)grid_vertices.size());
#endif
    std::cout << "[RENDERER] Created grid with " << grid_vertices.size() << " vertices" << '\n';
    grid_mesh_->set_data(grid_vertices);

    axes_mesh_ = std::make_unique<VertexArray>();
    std::vector<Vertex> axes_vertices = create_axes(0.5f);
#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Created axes with %d vertices", (int)axes_vertices.size());
#endif
    std::cout << "[RENDERER] Created axes with " << axes_vertices.size() << " vertices" << '\n';
    axes_mesh_->set_data(axes_vertices);

#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Initialization complete!");
#endif
    std::cout << "[RENDERER] Initialization complete!" << '\n';

    return true;
}

void Renderer::render(const StateVector& state) {
    // Get camera matrices
    float aspect = static_cast<float>(width_) / static_cast<float>(height_);
    Eigen::Matrix4f view = camera_.get_view_matrix();
    Eigen::Matrix4f proj = Camera::get_projection_matrix(aspect);

    // ========================================================================
    // Render Grid Floor and Axes (shared shader)
    // ========================================================================
    {
        grid_shader_->use();

        // Set shared uniforms once for both grid and axes
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
        grid_shader_->set_uniform("uModel", model);
        grid_shader_->set_uniform("uView", view);
        grid_shader_->set_uniform("uProjection", proj);

        // Render grid
        grid_shader_->set_uniform("uColor", Eigen::Vector3f(0.3f, 0.3f, 0.3f));
        grid_mesh_->bind();
        glDrawArrays(GL_LINES, 0, grid_mesh_->get_vertex_count());
        grid_mesh_->unbind();

        // Render axes (uModel, uView, uProjection already set)
        grid_shader_->set_uniform("uColor", Eigen::Vector3f(1.0f, 1.0f, 1.0f));
        axes_mesh_->bind();
        glDrawArrays(GL_LINES, 0, axes_mesh_->get_vertex_count());
        axes_mesh_->unbind();
    }

    // ========================================================================
    // Render Table and Ball (shared shader and lighting)
    // ========================================================================
    {
        basic_shader_->use();

        // Set shared uniforms once for both table and ball
        basic_shader_->set_uniform("uView", view);
        basic_shader_->set_uniform("uProjection", proj);
        basic_shader_->set_uniform("uLightPos", Eigen::Vector3f(2.0f, 3.0f, 2.0f));
        basic_shader_->set_uniform("uLightColor", Eigen::Vector3f(1.0f, 1.0f, 1.0f));  // White light
        basic_shader_->set_uniform("uAmbient", 0.3f);

        // Render table
        {
            // Table transformation: rotate by theta_x and theta_y
            float theta_x = state(state_index::THETA_X);
            float theta_y = state(state_index::THETA_Y);

            // Rotation around X axis
            Eigen::Matrix4f rot_x = Eigen::Matrix4f::Identity();
            rot_x(1, 1) = std::cos(theta_x);
            rot_x(1, 2) = -std::sin(theta_x);
            rot_x(2, 1) = std::sin(theta_x);
            rot_x(2, 2) = std::cos(theta_x);

            // Rotation around Z axis (physics Y-axis maps to OpenGL -Z axis)
            // This rotates in the XY plane (around the Z axis pointing toward viewer)
            Eigen::Matrix4f rot_y = Eigen::Matrix4f::Identity();
            rot_y(0, 0) = std::cos(theta_y);
            rot_y(0, 1) = -std::sin(theta_y);
            rot_y(1, 0) = std::sin(theta_y);
            rot_y(1, 1) = std::cos(theta_y);

            Eigen::Matrix4f model = rot_y * rot_x;  // Apply rotations
            Eigen::Matrix3f normal_matrix = model.topLeftCorner<3, 3>();

            basic_shader_->set_uniform("uModel", model);
            basic_shader_->set_uniform("uNormalMatrix", normal_matrix);
            basic_shader_->set_uniform("uColor", Eigen::Vector3f(0.7f, 0.5f, 0.3f));  // Wood color

            table_mesh_->bind();
            glDrawArrays(GL_TRIANGLES, 0, table_mesh_->get_vertex_count());
            table_mesh_->unbind();
        }

        // Render ball (uView, uProjection, lighting already set)
        {
            // Ball position from state
            // State uses physics coordinates: X (horizontal), Y (horizontal), Z (vertical)
            // OpenGL uses: X (horizontal), Y (vertical/up), Z (horizontal)
            float ball_x = state(state_index::X);
            float ball_y = state(state_index::Y);
            float ball_z = params_.ball_radius;  // Ball sits on table surface

#ifdef __EMSCRIPTEN__
            static int log_counter = 0;
            if (log_counter++ < 5) {  // Log first 5 frames only
                emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Ball render: state_x=%f, state_y=%f, height=%f, vertices=%d",
                              ball_x, ball_y, ball_z, ball_mesh_->get_vertex_count());
            }
#endif

            // Translation to ball position
            Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
            model(0, 3) = ball_x;        // X maps to X
            model(1, 3) = ball_z;        // Z (height) maps to Y (up in OpenGL)
            model(2, 3) = -ball_y;       // Y maps to -Z (OpenGL Z points toward viewer)

            Eigen::Matrix3f normal_matrix = model.topLeftCorner<3, 3>();

            basic_shader_->set_uniform("uModel", model);
            basic_shader_->set_uniform("uNormalMatrix", normal_matrix);
            basic_shader_->set_uniform("uColor", Eigen::Vector3f(0.9f, 0.1f, 0.1f));  // Red ball

            ball_mesh_->bind();
            glDrawArrays(GL_TRIANGLES, 0, ball_mesh_->get_vertex_count());
            ball_mesh_->unbind();
        }
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

    const float pi = static_cast<float>(M_PI);
    const Eigen::Vector3f color(1.0f, 0.5f, 0.2f);  // Orange for ball

    // UV Sphere generation
    // Generate vertices in a latitude-longitude pattern
    for (int lat = 0; lat <= segments; ++lat) {
        float theta = lat * pi / segments;  // 0 to pi
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);

        for (int lon = 0; lon <= segments; ++lon) {
            float phi = lon * 2.0f * pi / segments;  // 0 to 2*pi
            float sin_phi = std::sin(phi);
            float cos_phi = std::cos(phi);

            // Vertex position (spherical to Cartesian)
            Eigen::Vector3f pos(
                radius * sin_theta * cos_phi,  // X
                radius * cos_theta,             // Y (up)
                radius * sin_theta * sin_phi    // Z
            );

            // Normal is the normalized position vector
            Eigen::Vector3f normal = pos.normalized();

            // Store vertex (we'll generate indices later)
            vertices.push_back({pos, normal, color});
        }
    }

    // Now generate triangle indices and create triangle list
    std::vector<Vertex> triangle_vertices;
    for (int lat = 0; lat < segments; ++lat) {
        for (int lon = 0; lon < segments; ++lon) {
            // Calculate indices for the quad
            int current = lat * (segments + 1) + lon;
            int next = current + segments + 1;

            // First triangle of quad (current, next, current+1)
            triangle_vertices.push_back(vertices[current]);
            triangle_vertices.push_back(vertices[next]);
            triangle_vertices.push_back(vertices[current + 1]);

            // Second triangle of quad (current+1, next, next+1)
            triangle_vertices.push_back(vertices[current + 1]);
            triangle_vertices.push_back(vertices[next]);
            triangle_vertices.push_back(vertices[next + 1]);
        }
    }

#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[SPHERE] Created UV sphere with %d vertices, radius=%.6f", (int)triangle_vertices.size(), radius);
#endif

    return triangle_vertices;
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
