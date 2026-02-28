#pragma once

#include <ball_balancer/gui/control_panel.hpp>
#include <ball_balancer/rendering/renderer.hpp>
#include <ball_balancer/visualization/real_time_plotter.hpp>
#include <imgui.h>
#include <memory>

/**
 * @file main_window.hpp
 * @brief Main application window with docking layout
 *
 * Manages the overall application UI layout:
 * - Fullscreen dockspace for dockable windows
 * - Menu bar for application control
 * - Control panel (left side by default)
 * - 3D viewport (center)
 * - Plots panel (right side by default)
 *
 * Following best practices from research/dear-imgui-cpp-gui-best-practices.md:
 * - Use docking branch features (DockSpace)
 * - Dynamic sizing based on viewport
 * - Professional menu bar
 * - Window visibility flags for panel control
 *
 * @see research/dear-imgui-cpp-gui-best-practices.md
 */

namespace ball_balancer {

/**
 * @brief Main application window with docking support
 *
 * Provides IDE-like interface with dockable panels:
 * - Menu bar for File/View/Help
 * - Dockspace for flexible panel arrangement
 * - Control panel for parameter tuning
 * - 3D viewport for visualization
 * - Plots panel for real-time data (managed by ImPlot agent)
 *
 * Usage:
 * ```cpp
 * MainWindow window(params);
 * window.initialize();
 *
 * // Main loop:
 * ImGui::NewFrame();
 * window.render(state, controller, estimator, *renderer);
 * ImGui::Render();
 * ```
 */
class MainWindow {
public:
    /**
     * @brief Construct main window
     * @param params System parameters
     */
    explicit MainWindow(const SystemParameters& params);

    /**
     * @brief Initialize ImGui context and style
     * @param window GLFW window pointer
     *
     * Sets up ImGui context, docking, custom style, and backends.
     * Call once after creating OpenGL context.
     */
    bool initialize(void* window);

    /**
     * @brief Cleanup ImGui resources
     *
     * Call before destroying OpenGL context.
     */
    void shutdown();

    /**
     * @brief Start new ImGui frame
     *
     * Call at beginning of frame before any ImGui calls.
     */
    void begin_frame();

    /**
     * @brief End ImGui frame and prepare for rendering
     *
     * Call after all ImGui calls, before ImGui::Render().
     */
    void end_frame();

    /**
     * @brief Render main window UI
     * @param state Current system state
     * @param controller PID controller
     * @param estimator State estimator
     * @param renderer 3D renderer
     * @param plotter Real-time data plotter
     *
     * Renders all UI panels. Call between begin_frame() and end_frame().
     */
    void render(
        const StateVector& state,
        PIDController& controller,
        StateEstimator& estimator,
        Renderer& renderer,
        RealTimePlotter& plotter,
        bool in_contact
    );

    /**
     * @brief Get control panel
     */
    ControlPanel& get_control_panel() { return *control_panel_; }

    /**
     * @brief Check if user requested exit
     */
    bool should_exit() const { return exit_requested_; }

private:
    /**
     * @brief Setup ImGui style (colors, spacing, rounding)
     */
    void setup_style();

    /**
     * @brief Render menu bar
     */
    void render_menu_bar();

    /**
     * @brief Render dockspace
     *
     * Creates fullscreen dockspace where panels can be docked.
     */
    void render_dockspace();

    /**
     * @brief Render 3D viewport window
     * @param state Current system state
     * @param renderer 3D renderer
     */
    void render_viewport(const StateVector& state, Renderer& renderer);

    /**
     * @brief Setup default docking layout (first run only)
     *
     * Creates default layout:
     * - Control panel docked left
     * - Viewport in center
     * - Plots docked right
     */
    void setup_default_layout();

    // System parameters
    SystemParameters params_;

    // UI panels
    std::unique_ptr<ControlPanel> control_panel_;

    // Window visibility flags
    bool show_control_panel_;
    bool show_viewport_;
    bool show_plots_;
    bool show_demo_;

    // Application state
    bool exit_requested_;
    bool first_frame_;

    // Dockspace ID
    ImGuiID dockspace_id_;
};

} // namespace ball_balancer
