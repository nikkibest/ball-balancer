#pragma once

#include <ball_balancer/core/types.hpp>
#include <ball_balancer/control/pid_controller.hpp>
#include <ball_balancer/control/state_estimator.hpp>
#include <imgui.h>

/**
 * @file control_panel.hpp
 * @brief Control panel UI for ball balancer
 *
 * Provides user interface for:
 * - System control (start/stop/reset)
 * - Setpoint adjustment
 * - PID gain tuning
 * - Kalman filter tuning
 * - System status display
 *
 * Following best practices from research/dear-imgui-cpp-gui-best-practices.md:
 * - UI reflects data state directly (no duplicate UI state)
 * - All UI code between Begin() and End()
 * - Dynamic sizing using GetContentRegionAvail()
 * - Proper Begin/End pairs always called
 *
 * @see research/dear-imgui-cpp-gui-best-practices.md
 */

namespace ball_balancer {

/**
 * @brief Application state for UI control
 */
enum class SimulationState {
    Stopped,   ///< Simulation not running
    Running,   ///< Simulation running
    Paused     ///< Simulation paused
};

/**
 * @brief Control panel for ball balancer UI
 *
 * Provides complete UI for controlling the ball balancer:
 * - Play/Pause/Reset controls
 * - Setpoint sliders for target position
 * - PID gain tuning for X and Y axes
 * - Kalman filter parameter tuning
 * - Real-time system status display
 *
 * Usage:
 * ```cpp
 * ControlPanel panel;
 *
 * // In main loop, between ImGui::NewFrame() and ImGui::Render():
 * if (panel.render(state, controller, estimator)) {
 *     // Panel was visible this frame
 * }
 *
 * // Check if user wants to reset
 * if (panel.should_reset()) {
 *     simulator.reset(StateVector::Zero());
 * }
 * ```
 */
class ControlPanel {
public:
    /**
     * @brief Construct control panel
     * @param params System parameters for bounds checking
     */
    explicit ControlPanel(const SystemParameters& params);

    /**
     * @brief Render control panel UI
     * @param state Current system state
     * @param controller PID controller (for gain display/editing)
     * @param estimator State estimator (for tuning display/editing)
     * @return true if window was visible this frame
     *
     * Must be called between ImGui::NewFrame() and ImGui::Render()
     */
    bool render(
        const StateVector& state,
        PIDController& controller,
        StateEstimator& estimator
    );

    /**
     * @brief Get current simulation state
     */
    SimulationState get_state() const { return sim_state_; }

    /**
     * @brief Check if reset was requested
     * @return true if user clicked reset button
     */
    bool should_reset() const { return reset_requested_; }

    /**
     * @brief Clear reset flag (call after handling reset)
     */
    void clear_reset_flag() { reset_requested_ = false; }

    /**
     * @brief Get current setpoint
     * @return Target position [x, y]
     */
    Eigen::Vector2d get_setpoint() const { return setpoint_; }

    /**
     * @brief Set simulation state
     */
    void set_state(SimulationState state) { sim_state_ = state; }

    /**
     * @brief Set window visibility
     */
    void set_visible(bool visible) { is_visible_ = visible; }

    /**
     * @brief Check if window is visible
     */
    bool is_visible() const { return is_visible_; }

private:
    /**
     * @brief Render simulation control section
     */
    void render_simulation_controls();

    /**
     * @brief Render setpoint control section
     */
    void render_setpoint_controls();

    /**
     * @brief Render PID tuning section
     * @param controller PID controller to modify
     */
    void render_pid_tuning(PIDController& controller);

    /**
     * @brief Render Kalman filter tuning section
     * @param estimator State estimator to modify
     */
    void render_kalman_tuning(StateEstimator& estimator);

    /**
     * @brief Render system status section
     * @param state Current system state
     */
    void render_system_status(const StateVector& state);

    // System parameters
    SystemParameters params_;

    // UI state
    bool is_visible_;
    SimulationState sim_state_;
    bool reset_requested_;

    // Control parameters
    Eigen::Vector2d setpoint_;  ///< Target ball position

    // UI helper state
    bool show_advanced_tuning_;  ///< Show advanced Kalman tuning
};

} // namespace ball_balancer
