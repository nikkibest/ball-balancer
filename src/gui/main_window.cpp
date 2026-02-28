#include <ball_balancer/gui/main_window.hpp>
#include <imgui.h>
#include <imgui_internal.h>  // For DockBuilder API
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include <GLFW/glfw3.h>

/**
 * @file main_window.cpp
 * @brief Implementation of main application window
 *
 * Implements docking layout and UI management following ImGui best practices:
 * - Proper initialization/shutdown sequence
 * - Dockspace setup for flexible layouts
 * - Custom styling for professional appearance
 * - Menu bar for application control
 * - OpenGL viewport integration
 *
 * @see research/dear-imgui-cpp-gui-best-practices.md
 */

namespace ball_balancer {

MainWindow::MainWindow(const SystemParameters& params)
    : params_(params)
    , control_panel_(std::make_unique<ControlPanel>(params))
    , show_control_panel_(true)
    , show_viewport_(true)
    , show_plots_(true)
    , show_demo_(false)
    , exit_requested_(false)
    , first_frame_(true)
    , dockspace_id_(0)
{
}

bool MainWindow::initialize(void* window) {
    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    // Enable docking and viewports (docking branch features)
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#ifndef __EMSCRIPTEN__
    // Viewports not supported in browser (can't create windows outside canvas)
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
#endif

    // Setup custom style
    setup_style();

    // Setup Platform/Renderer backends
    GLFWwindow* glfw_window = static_cast<GLFWwindow*>(window);
    if (!ImGui_ImplGlfw_InitForOpenGL(glfw_window, true)) {
        return false;
    }

    // Set appropriate GLSL version based on platform
#ifdef __EMSCRIPTEN__
    const char* glsl_version = "#version 300 es";  // WebGL 2.0 / OpenGL ES 3.0
#else
    const char* glsl_version = "#version 450";     // Desktop OpenGL 4.5
#endif
    if (!ImGui_ImplOpenGL3_Init(glsl_version)) {
        ImGui_ImplGlfw_Shutdown();
        return false;
    }

    // TODO: Load custom fonts
    // For now, using default font
    // In production, load custom font:
    // io.Fonts->AddFontFromFileTTF("fonts/Roboto-Regular.ttf", 16.0f);
    // io.Fonts->Build();

    return true;
}

void MainWindow::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
}

void MainWindow::begin_frame() {
    // Start ImGui frame
    // IMPORTANT: Call in this order (backend before ImGui::NewFrame)
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void MainWindow::end_frame() {
    // Rendering
    ImGui::Render();

    // Render ImGui draw data
    // This should be called AFTER rendering 3D scene
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Update and render multi-viewport windows if enabled
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        GLFWwindow* backup_current_context = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup_current_context);
    }
}

void MainWindow::render(
    const StateVector& state,
    PIDController& controller,
    StateEstimator& estimator,
    Renderer& renderer,
    RealTimePlotter& plotter
) {
    // Render menu bar (outside dockspace)
    render_menu_bar();

    // Render dockspace
    render_dockspace();

    // Setup default layout on first frame
    if (first_frame_) {
        setup_default_layout();
        first_frame_ = false;
    }

    // Render panels
    if (show_control_panel_) {
        control_panel_->render(state, controller, estimator);
    }

    if (show_viewport_) {
        render_viewport(state, renderer);
    }

    // Plots panel rendered by RealTimePlotter
    if (show_plots_) {
        plotter.render();
    }

    // Optional: Show ImGui demo window
    if (show_demo_) {
        ImGui::ShowDemoWindow(&show_demo_);
    }
}

void MainWindow::setup_style() {
    ImGuiStyle& style = ImGui::GetStyle();

    // Rounding for modern appearance
    style.WindowRounding = 5.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 2.0f;
    style.ScrollbarRounding = 3.0f;
    style.TabRounding = 3.0f;

    // Spacing for comfortable layout
    style.WindowPadding = ImVec2(10.0f, 10.0f);
    style.FramePadding = ImVec2(8.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 8.0f);
    style.ItemInnerSpacing = ImVec2(6.0f, 6.0f);

    // Custom dark theme colors
    ImVec4* colors = style.Colors;

    // Window colors
    colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.12f, 0.12f, 0.12f, 1.00f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.11f, 0.11f, 0.11f, 0.94f);

    // Frame colors (sliders, input boxes)
    colors[ImGuiCol_FrameBg] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);

    // Title bar
    colors[ImGuiCol_TitleBg] = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);

    // Menu bar
    colors[ImGuiCol_MenuBarBg] = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);

    // Scrollbar
    colors[ImGuiCol_ScrollbarBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);

    // Checkboxes and radio buttons
    colors[ImGuiCol_CheckMark] = ImVec4(0.40f, 0.70f, 1.00f, 1.00f);

    // Sliders
    colors[ImGuiCol_SliderGrab] = ImVec4(0.40f, 0.70f, 1.00f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.50f, 0.80f, 1.00f, 1.00f);

    // Buttons
    colors[ImGuiCol_Button] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);

    // Headers (collapsing headers, tree nodes)
    colors[ImGuiCol_Header] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.35f, 0.35f, 0.35f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.45f, 0.45f, 0.45f, 1.00f);

    // Separators
    colors[ImGuiCol_Separator] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
    colors[ImGuiCol_SeparatorHovered] = ImVec4(0.40f, 0.70f, 1.00f, 0.78f);
    colors[ImGuiCol_SeparatorActive] = ImVec4(0.40f, 0.70f, 1.00f, 1.00f);

    // Tabs
    colors[ImGuiCol_Tab] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);
    colors[ImGuiCol_TabActive] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
    colors[ImGuiCol_TabUnfocused] = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);

    // Docking
    colors[ImGuiCol_DockingPreview] = ImVec4(0.40f, 0.70f, 1.00f, 0.50f);
    colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);

    // Text
    colors[ImGuiCol_Text] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
}

void MainWindow::render_menu_bar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Exit", "Alt+F4")) {
                exit_requested_ = true;
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Control Panel", nullptr, &show_control_panel_);
            ImGui::MenuItem("3D Viewport", nullptr, &show_viewport_);
            ImGui::MenuItem("Plots", nullptr, &show_plots_);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &show_demo_);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem("About")) {
                // TODO: Show about dialog
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void MainWindow::render_dockspace() {
    // Create fullscreen dockspace
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    // Window flags for dockspace
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse;
    window_flags |= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    window_flags |= ImGuiWindowFlags_NoBackground;  // Make dockspace transparent to show 3D scene

    // Remove padding for fullscreen effect
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    ImGui::Begin("DockSpace Window", nullptr, window_flags);
    ImGui::PopStyleVar(3);

    // Create dockspace
    dockspace_id_ = ImGui::GetID("MainDockSpace");
    ImGui::DockSpace(dockspace_id_, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);

    ImGui::End();
}

void MainWindow::render_viewport(const StateVector& state, Renderer& renderer) {
    // Make viewport window transparent to show 3D rendering underneath
    ImGui::SetNextWindowBgAlpha(0.0f);  // Fully transparent background

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
    ImGui::Begin("3D Viewport", &show_viewport_, flags);

    // Get viewport position and size for overlay rendering
    ImVec2 viewport_pos  = ImGui::GetWindowPos();
    ImVec2 viewport_size = ImGui::GetContentRegionAvail();

    // NOTE: The 3D scene is rendered to the full window framebuffer in application.cpp
    // This viewport window is transparent, so the 3D rendering shows through

    // Ball position overlay in corner
    ImGui::SetCursorPos(ImVec2(10, 10));
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.8f), "Ball: (%.3f, %.3f)",
                      state(state_index::X), state(state_index::Y));

    // Draw axis labels (X/Y/Z) projected from 3D world to screen
    renderer.render_axis_labels(viewport_pos, viewport_size);

    ImGui::End();
}

void MainWindow::setup_default_layout() {
    // Setup default docking layout on first run
    // This only happens if no saved layout exists

    // Clear any existing layout
    ImGui::DockBuilderRemoveNode(dockspace_id_);
    ImGui::DockBuilderAddNode(dockspace_id_, ImGuiDockNodeFlags_DockSpace);

    // Set dockspace size to fill window
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::DockBuilderSetNodeSize(dockspace_id_, viewport->WorkSize);

    // Split dockspace into sections
    ImGuiID dock_left, dock_right, dock_center;

    // Split: Control panel on left (20% width)
    dock_left = ImGui::DockBuilderSplitNode(dockspace_id_, ImGuiDir_Left, 0.20f, nullptr, &dock_center);

    // Split: Plots on right (25% width)
    dock_right = ImGui::DockBuilderSplitNode(dock_center, ImGuiDir_Right, 0.30f, nullptr, &dock_center);

    // Dock windows to nodes
    ImGui::DockBuilderDockWindow("Control Panel", dock_left);
    ImGui::DockBuilderDockWindow("3D Viewport", dock_center);
    ImGui::DockBuilderDockWindow("Plots", dock_right);

    ImGui::DockBuilderFinish(dockspace_id_);
}

} // namespace ball_balancer
