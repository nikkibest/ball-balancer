#include <ball_balancer/rendering/renderer.hpp>
#include <imgui.h>
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
    , last_vp_(Eigen::Matrix4f::Identity())
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
    std::vector<Vertex> plane_vertices = create_disc(static_cast<float>(params_.table_radius), 64);
#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Created disc with %d vertices", (int)plane_vertices.size());
#endif
    std::cout << "[RENDERER] Created disc with " << plane_vertices.size() << " vertices" << '\n';
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

    // Cylinder mesh: unit cylinder along Y, radius=1, Y in [-0.5, +0.5].
    // Static — a stretch-and-orient model matrix places it between any two points.
    cylinder_mesh_ = std::make_unique<VertexArray>();
    {
        std::vector<Vertex> cyl_verts = create_cylinder(16);
        cylinder_mesh_->set_data(cyl_verts);
        std::cout << "[RENDERER] Created cylinder mesh, " << cyl_verts.size() << " vertices\n";
    }

    // Joint sphere mesh: small sphere for elbow joint visualization.
    joint_sphere_mesh_ = std::make_unique<VertexArray>();
    {
        std::vector<Vertex> sph_verts = create_sphere(0.012f, 16);
        joint_sphere_mesh_->set_data(sph_verts);
        std::cout << "[RENDERER] Created joint sphere mesh, " << sph_verts.size() << " vertices\n";
    }

#ifdef __EMSCRIPTEN__
    emscripten_log(EM_LOG_CONSOLE, "[RENDERER] Initialization complete!");
#endif
    std::cout << "[RENDERER] Initialization complete!" << '\n';

    return true;
}

void Renderer::render(const BallState& ball, const TableState& table) {
    // Get camera matrices
    float aspect = static_cast<float>(width_) / static_cast<float>(height_);
    Eigen::Matrix4f view = camera_.get_view_matrix();
    Eigen::Matrix4f proj = Camera::get_projection_matrix(aspect);

    // Cache VP so render_axis_labels() uses the EXACT same transform
    last_vp_ = proj * view;

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
            // Table transformation: rotate by varphi_x and theta_y, translate by z_table
            float varphi_x = static_cast<float>(table.phi);
            float theta_y  = static_cast<float>(table.theta);
            float z_table  = static_cast<float>(table.z_t);

            // Rotation around X axis
            Eigen::Matrix4f rot_x = Eigen::Matrix4f::Identity();
            rot_x(1, 1) = std::cos(varphi_x);
            rot_x(1, 2) = -std::sin(varphi_x);
            rot_x(2, 1) = std::sin(varphi_x);
            rot_x(2, 2) = std::cos(varphi_x);

            // Rotation around Z axis (physics Y-axis maps to OpenGL -Z axis)
            // This rotates in the XY plane (around the Z axis pointing toward viewer)
            Eigen::Matrix4f rot_y = Eigen::Matrix4f::Identity();
            rot_y(0, 0) = std::cos(theta_y);
            rot_y(0, 1) = -std::sin(theta_y);
            rot_y(1, 0) = std::sin(theta_y);
            rot_y(1, 1) = std::cos(theta_y);

            // Translation by z_table along the vertical (OpenGL Y) axis
            Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
            translation(1, 3) = z_table;

            Eigen::Matrix4f model = translation * rot_y * rot_x;  // Translate after rotations
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
            // Physics coords: X (horizontal), Y (horizontal), Z_BALL (vertical above table)
            // OpenGL coords: X (horizontal), Y (vertical/up), Z (horizontal, into screen)
            float ball_x = static_cast<float>(ball.x);
            float ball_y = static_cast<float>(ball.y);
            // z_ball is an absolute world-frame height (set by BallDynamics / contact snap).
            // The table is rendered at z_t, and the ball rests at z_surface = z_t + r.
            // Do NOT add z_t or ball_radius here — z_ball already encodes both.
            float ball_z = static_cast<float>(ball.z_ball);

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
            model(2, 3) = ball_y;        // Y maps to +Z (positive physics Y → positive GL Z)

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

void Renderer::render_axis_labels(ImVec2 clip_min, ImVec2 clip_max) {
    // Reuse the exact VP matrix cached during render() so the labels are
    // guaranteed to use the same transform as the 3D geometry.
    // Map NDC → screen using the same dimensions the GL viewport covers.
    // The GL viewport is always the full GLFW window, and ImGui's main
    // viewport Pos is the window's top-left in OS screen coordinates.
    const ImGuiViewport* main_vp = ImGui::GetMainViewport();
    const float orig_x = main_vp->Pos.x;
    const float orig_y = main_vp->Pos.y;

    // Screen dimensions: use DisplaySize (logical pixels, same space as ImGui)
    // On non-HiDPI Linux this equals width_/height_ exactly.
    const ImGuiIO& io = ImGui::GetIO();
    const float sc_w = (io.DisplaySize.x > 0.0f) ? io.DisplaySize.x : static_cast<float>(width_);
    const float sc_h = (io.DisplaySize.y > 0.0f) ? io.DisplaySize.y : static_cast<float>(height_);

    // Project a 3D world point → 2D screen position (logical pixels)
    auto project = [&](const Eigen::Vector3f& world_pos) -> ImVec2 {
        Eigen::Vector4f clip = last_vp_ * Eigen::Vector4f(world_pos.x(), world_pos.y(), world_pos.z(), 1.0f);
        if (std::abs(clip.w()) < 1e-6f) {
            return ImVec2(-9999.f, -9999.f);
        }
        const float ndcX =  clip.x() / clip.w();
        const float ndcY = -clip.y() / clip.w();  // NDC Y-up → screen Y-down
        return ImVec2(orig_x + (ndcX * 0.5f + 0.5f) * sc_w,
                      orig_y + (ndcY * 0.5f + 0.5f) * sc_h);
    };

    // Axis tips in OpenGL world space (matches create_axes length = 0.5f):
    //   GL +X → physics X  (red)
    //   GL +Y → physics Z  (blue, vertical/up)
    //   GL +Z → physics Y  (green, horizontal into screen)
    constexpr float L = 0.5f;
    const ImVec2 tip_x = project(Eigen::Vector3f(L,    0.0f, 0.0f));
    const ImVec2 tip_y = project(Eigen::Vector3f(0.0f, 0.0f, L   ));  // GL +Z = physics Y
    const ImVec2 tip_z = project(Eigen::Vector3f(0.0f, L,    0.0f));  // GL +Y = physics Z

    ImDrawList* draw = ImGui::GetForegroundDrawList();
    draw->PushClipRect(clip_min, clip_max, true);

    const float font_size = ImGui::GetFontSize();
    constexpr float off = 5.0f;

    auto draw_label = [&](ImVec2 tip, ImU32 col, const char* text) {
        // Skip labels that projected behind the camera
        if (tip.x < -9000.f) return;
        draw->AddText(ImVec2(tip.x + off, tip.y - font_size * 0.5f), col, text);
    };

    draw_label(tip_x, IM_COL32(255, 80,  80,  255), "X");
    draw_label(tip_y, IM_COL32(80,  255, 80,  255), "Y");
    draw_label(tip_z, IM_COL32(80,  80,  255, 255), "Z");

    draw->PopClipRect();
}

void Renderer::render_legs(
    const std::array<std::array<std::array<float, 3>, 3>, 3>& arm_points)
{
    if (!show_legs_ || !cylinder_mesh_ || !joint_sphere_mesh_) return;

    // Per-arm colours: arm 0 = cyan, arm 1 = yellow, arm 2 = magenta
    static const Eigen::Vector3f ARM_COLORS[3] = {
        Eigen::Vector3f(0.0f, 1.0f, 1.0f),   // cyan
        Eigen::Vector3f(1.0f, 1.0f, 0.0f),   // yellow
        Eigen::Vector3f(1.0f, 0.0f, 1.0f)    // magenta
    };
    // Lower link (G→E) uses a darker shade to visually distinguish the two links.
    constexpr float LOWER_SHADE = 0.60f;

    // Cylinder radius in world-space metres.
    constexpr float CYLINDER_RADIUS = 0.010f;   // 10 mm rod

    // Physics → OpenGL coordinate mapping: GL(x, y, z) = (phys_x, phys_z, phys_y)
    auto phys_to_gl = [](const std::array<float, 3>& p) -> Eigen::Vector3f {
        return Eigen::Vector3f(p[0], p[2], p[1]);
    };

    // Build a model matrix that places the unit cylinder (Y axis, -0.5 to +0.5, radius=1)
    // with its two ends at A and B, with the given world-space cylinder radius.
    auto make_cylinder_matrix = [](
        const Eigen::Vector3f& A,
        const Eigen::Vector3f& B,
        float radius) -> Eigen::Matrix4f
    {
        const Eigen::Vector3f diff = B - A;
        const float L = diff.norm();
        if (L < 1e-6f) return Eigen::Matrix4f::Identity();

        const Eigen::Vector3f d = diff / L;                  // unit direction
        const Eigen::Vector3f canonical(0.0f, 1.0f, 0.0f);  // cylinder's canonical axis

        const float c = canonical.dot(d);
        const Eigen::Vector3f k = canonical.cross(d);        // sin-scaled rotation axis

        Eigen::Matrix3f R;
        if (c < -0.9999f) {
            // d ≈ (0,-1,0): 180° rotation about X axis
            R = Eigen::Matrix3f::Zero();
            R(0, 0) =  1.0f;
            R(1, 1) = -1.0f;
            R(2, 2) = -1.0f;
        } else {
            // Rodrigues' formula: R = I + [k]× + [k]×² / (1 + c)
            Eigen::Matrix3f K;
            K <<    0.0f, -k.z(),  k.y(),
                 k.z(),    0.0f, -k.x(),
                -k.y(),  k.x(),    0.0f;
            R = Eigen::Matrix3f::Identity() + K + K * K * (1.0f / (1.0f + c));
        }

        // Non-uniform scale: radius in X/Z, length L in Y (unit cylinder has radius=1)
        Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
        S(0, 0) = radius;
        S(1, 1) = L;
        S(2, 2) = radius;

        Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
        M.topLeftCorner<3, 3>() = R * S;
        M.col(3).head<3>() = 0.5f * (A + B);   // midpoint translation
        return M;
    };

    // Bind Phong shader (shared by table/ball — already supports uModel, uNormalMatrix, uColor, lighting)
    basic_shader_->use();

    const float aspect = static_cast<float>(width_) / static_cast<float>(height_);
    const Eigen::Matrix4f view = camera_.get_view_matrix();
    const Eigen::Matrix4f proj = Camera::get_projection_matrix(aspect);

    // Set uniforms shared across all 9 draw calls
    basic_shader_->set_uniform("uView",       view);
    basic_shader_->set_uniform("uProjection", proj);
    basic_shader_->set_uniform("uLightPos",   Eigen::Vector3f(2.0f, 3.0f, 2.0f));
    basic_shader_->set_uniform("uLightColor", Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    basic_shader_->set_uniform("uAmbient",    0.3f);

    // =========================================================================
    // 6 cylinders: 3 arms × 2 links (G→E lower, E→T upper)
    // =========================================================================
    cylinder_mesh_->bind();

    for (int arm = 0; arm < 3; ++arm) {
        const auto& pts = arm_points[arm];   // pts[0]=G, pts[1]=E, pts[2]=T

        const Eigen::Vector3f G = phys_to_gl(pts[0]);
        const Eigen::Vector3f E = phys_to_gl(pts[1]);
        const Eigen::Vector3f T = phys_to_gl(pts[2]);

        const Eigen::Vector3f base_col = ARM_COLORS[arm];
        const Eigen::Vector3f dark_col = base_col * LOWER_SHADE;

        // Lower link G → E (darker)
        {
            const Eigen::Matrix4f model = make_cylinder_matrix(G, E, CYLINDER_RADIUS);
            // Non-uniform scale requires full inverse-transpose for correct normals.
            const Eigen::Matrix3f normal_mat =
                model.topLeftCorner<3, 3>().inverse().transpose();
            basic_shader_->set_uniform("uModel",        model);
            basic_shader_->set_uniform("uNormalMatrix", normal_mat);
            basic_shader_->set_uniform("uColor",        dark_col);
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(cylinder_mesh_->get_vertex_count()));
        }

        // Upper link E → T (full colour)
        {
            const Eigen::Matrix4f model = make_cylinder_matrix(E, T, CYLINDER_RADIUS);
            const Eigen::Matrix3f normal_mat =
                model.topLeftCorner<3, 3>().inverse().transpose();
            basic_shader_->set_uniform("uModel",        model);
            basic_shader_->set_uniform("uNormalMatrix", normal_mat);
            basic_shader_->set_uniform("uColor",        base_col);
            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(cylinder_mesh_->get_vertex_count()));
        }
    }

    cylinder_mesh_->unbind();

    // =========================================================================
    // 3 joint spheres at elbow points (E)
    // =========================================================================
    joint_sphere_mesh_->bind();

    for (int arm = 0; arm < 3; ++arm) {
        const Eigen::Vector3f E = phys_to_gl(arm_points[arm][1]);

        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
        model.col(3).head<3>() = E;

        // Pure translation — normal matrix is identity
        const Eigen::Matrix3f normal_mat_sphere = Eigen::Matrix3f::Identity();
        basic_shader_->set_uniform("uModel",        model);
        basic_shader_->set_uniform("uNormalMatrix", normal_mat_sphere);
        basic_shader_->set_uniform("uColor",        ARM_COLORS[arm]);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(joint_sphere_mesh_->get_vertex_count()));
    }

    joint_sphere_mesh_->unbind();
}

void Renderer::resize(int width, int height) {
    width_ = width;
    height_ = height;
    glViewport(0, 0, width_, height_);
}

void Renderer::set_table_radius(float radius) {
    params_.table_radius = static_cast<double>(radius);
    table_mesh_->set_data(create_disc(radius, 64));
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

std::vector<Vertex> Renderer::create_disc(float radius, int segments) {
    std::vector<Vertex> vertices;
    vertices.reserve(static_cast<size_t>(segments) * 3);

    const Eigen::Vector3f color(0.6f, 0.7f, 0.8f);   // Light blue for table
    const Eigen::Vector3f normal(0.0f, 1.0f, 0.0f);   // Up (+Y in GL space)
    const Eigen::Vector3f center(0.0f, 0.0f, 0.0f);
    const float pi = static_cast<float>(M_PI);

    // Fan of triangles: center + two rim points per segment
    for (int i = 0; i < segments; ++i) {
        const float phi0 = static_cast<float>(i)     * 2.0f * pi / static_cast<float>(segments);
        const float phi1 = static_cast<float>(i + 1) * 2.0f * pi / static_cast<float>(segments);

        // GL coord: disc lies in the XZ plane (Y=0), physics X→GL X, physics Y→GL Z
        const Eigen::Vector3f v0(radius * std::cos(phi0), 0.0f, radius * std::sin(phi0));
        const Eigen::Vector3f v1(radius * std::cos(phi1), 0.0f, radius * std::sin(phi1));

        // CCW winding when viewed from above (+Y) → front face
        vertices.push_back({center, normal, color});
        vertices.push_back({v0,     normal, color});
        vertices.push_back({v1,     normal, color});
    }

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

std::vector<Vertex> Renderer::create_cylinder(int segments) {
    std::vector<Vertex> verts;
    verts.reserve(static_cast<size_t>(segments) * 12);

    const float pi = static_cast<float>(M_PI);
    // Neutral white — actual colour is applied per draw call via uColor uniform
    const Eigen::Vector3f white(1.0f, 1.0f, 1.0f);

    // -------------------------------------------------------------------------
    // Side surface: segments quads (each quad = 2 triangles)
    // Cylinder axis = Y, radius = 1, Y in [-0.5, +0.5]
    // -------------------------------------------------------------------------
    for (int i = 0; i < segments; ++i) {
        const float phi0 = static_cast<float>(i)     * 2.0f * pi / static_cast<float>(segments);
        const float phi1 = static_cast<float>(i + 1) * 2.0f * pi / static_cast<float>(segments);

        const float c0 = std::cos(phi0), s0 = std::sin(phi0);
        const float c1 = std::cos(phi1), s1 = std::sin(phi1);

        const Eigen::Vector3f b0(c0, -0.5f, s0);   // bottom-left
        const Eigen::Vector3f b1(c1, -0.5f, s1);   // bottom-right
        const Eigen::Vector3f t0(c0,  0.5f, s0);   // top-left
        const Eigen::Vector3f t1(c1,  0.5f, s1);   // top-right

        const Eigen::Vector3f n0(c0, 0.0f, s0);    // outward lateral normals
        const Eigen::Vector3f n1(c1, 0.0f, s1);

        verts.push_back({b0, n0, white});
        verts.push_back({b1, n1, white});
        verts.push_back({t0, n0, white});

        verts.push_back({t0, n0, white});
        verts.push_back({b1, n1, white});
        verts.push_back({t1, n1, white});
    }

    // -------------------------------------------------------------------------
    // Bottom cap (Y = -0.5, normal = -Y)
    // -------------------------------------------------------------------------
    const Eigen::Vector3f n_bot(0.0f, -1.0f, 0.0f);
    const Eigen::Vector3f n_top(0.0f,  1.0f, 0.0f);
    const Eigen::Vector3f center_bot(0.0f, -0.5f, 0.0f);
    const Eigen::Vector3f center_top(0.0f,  0.5f, 0.0f);

    for (int i = 0; i < segments; ++i) {
        const float phi0 = static_cast<float>(i)     * 2.0f * pi / static_cast<float>(segments);
        const float phi1 = static_cast<float>(i + 1) * 2.0f * pi / static_cast<float>(segments);

        const Eigen::Vector3f v0(std::cos(phi0), -0.5f, std::sin(phi0));
        const Eigen::Vector3f v1(std::cos(phi1), -0.5f, std::sin(phi1));

        // CW when viewed from below → correct CCW winding for outward -Y normal
        verts.push_back({center_bot, n_bot, white});
        verts.push_back({v1,         n_bot, white});
        verts.push_back({v0,         n_bot, white});
    }

    // -------------------------------------------------------------------------
    // Top cap (Y = +0.5, normal = +Y)
    // -------------------------------------------------------------------------
    for (int i = 0; i < segments; ++i) {
        const float phi0 = static_cast<float>(i)     * 2.0f * pi / static_cast<float>(segments);
        const float phi1 = static_cast<float>(i + 1) * 2.0f * pi / static_cast<float>(segments);

        const Eigen::Vector3f v0(std::cos(phi0), 0.5f, std::sin(phi0));
        const Eigen::Vector3f v1(std::cos(phi1), 0.5f, std::sin(phi1));

        // CCW when viewed from above → correct winding for outward +Y normal
        verts.push_back({center_top, n_top, white});
        verts.push_back({v0,         n_top, white});
        verts.push_back({v1,         n_top, white});
    }

    return verts;
}

} // namespace ball_balancer
