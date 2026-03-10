#pragma once

#include <ball_balancer/core/types.hpp>
#include <ball_balancer/control/pid_controller.hpp>
#include <ball_balancer/control/state_estimator.hpp>
#include <ball_balancer/physics/table_kinematics.hpp>
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
enum class ControllerState {
    Stopped,   ///< Controller not running
    Running,   ///< Controller running
};

/**
 * @brief Bidirectional arm mechanism status shared between Application and ControlPanel.
 *
 * Application fills this each frame and passes it to ControlPanel::render().
 * ControlPanel writes back mode/cmd changes so Application can act on them.
 */
struct ArmStatus {
    ServoAngles servo_angles{};       ///< Current integrated servo angles (read-only from GUI)
    ServoAngles servo_cmd{};          ///< Commanded servo angles (writable in Servo mode)
    bool        ik_failed{false};     ///< True when last IK call returned nullopt
    KinematicsMode mode{KinematicsMode::Pose}; ///< Active kinematics mode (read/write)
    bool        show_legs{true};      ///< Leg visibility toggle (read/write)
    bool        mode_changed{false};  ///< Set by GUI when mode radio button changes
    bool        cmd_changed{false};   ///< Set by GUI when servo cmd sliders change
    bool        show_legs_changed{false}; ///< Set by GUI when toggle changes

    // FK verification output (filled by Application in Servo mode)
    double fk_phi{0.0};
    double fk_theta{0.0};
    double fk_z{0.0};
    bool   fk_valid{false};

    // Geometry parameter change request (set by GUI sliders; Application rebuilds TableKinematics)
    bool   geom_changed{false};
    double req_L1{0.08};
    double req_L2{0.08};
    double req_Rg{0.10};
    double req_Rt{0.07};
    double req_z_nominal{0.12};
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
     * @param in_contact Whether the ball is currently in contact with the table
     * @param arm_status Arm mechanism state (read/write — GUI fills cmd/mode changes)
     * @return true if window was visible this frame
     *
     * Must be called between ImGui::NewFrame() and ImGui::Render()
     */
    bool render(
        const StateVector& state,
        PIDController& controller,
        StateEstimator& estimator,
        bool in_contact,
        ArmStatus& arm_status
    );

    /**
     * @brief Get current simulation state
     */
    SimulationState get_simulation_state() const { return sim_state_; }

    /**
     * @brief Get current controller state
     */
    ControllerState get_controller_state() const { return ctrl_state_; }

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
     * @brief Check if manual state sliders changed this frame (and simulation is paused)
     */
    bool is_manual_state_changed() const { return manual_state_changed_; }

    /**
     * @brief Clear manual state changed flag (call after applying state to simulator)
     */
    void clear_manual_state_changed() { manual_state_changed_ = false; }


    /**
     * @brief Get manual control input (when controller is off)
     * @return Manual tilt commands [theta_x, theta_y]
     */
    ControlVector get_manual_control() const { return control_; }

    /**
     * @brief Get the manually-edited state vector (when simulation is paused)
     * @return StateVector with values from GUI sliders
     */
    const StateVector& get_manual_state() const { return manual_state_; }

    /**
     * @brief Set the manual state (called by application to sync slider display)
     * @param state Current simulator state to initialise sliders from
     */
    void sync_manual_state(const StateVector& state) { manual_state_ = state; }

    /**
     * @brief Get current setpoint
     * @return Target position [x, y]
     */
    Eigen::Vector2d get_setpoint() const { return setpoint_; }

    /**
     * @brief Set simulation state
     */
    void set_simulation_state(SimulationState state) { sim_state_ = state; }
    
    /**
     * @brief Set controller state
     */
    void set_controller_state(ControllerState state) { ctrl_state_ = state; }

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
     * @brief Render simulation manual controls section (tilt angles)
     */
    void render_manual_controls();

    /**
     * @brief Render manual state editor section (all 6 physical states, paused only)
     * @return true if any slider value changed this frame
     */
    bool render_manual_state_sliders();

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
     * @param in_contact Whether the ball is currently in contact with the table
     */
    void render_system_status(const StateVector& state, bool in_contact);

    /**
     * @brief Render arm mechanism section (kinematics mode, servo angles, FK display).
     * @param arm_status  Bidirectional arm state; GUI reads current values and writes changes.
     */
    void render_arm_mechanism(ArmStatus& arm_status);

    // System parameters
    SystemParameters params_;

    // UI state
    bool is_visible_;
    SimulationState sim_state_;
    ControllerState ctrl_state_;
    bool reset_requested_;
    bool manual_state_changed_{false};  ///< True when a manual-state slider changed this frame

    // Manual paramaters
    ControlVector control_;

    // Control parameters
    Eigen::Vector2d setpoint_;  ///< Target ball position

    // Manual state editor (for paused mode)
    StateVector manual_state_;  ///< State values set by GUI sliders

    // UI helper state
    bool show_advanced_tuning_;  ///< Show advanced Kalman tuning
};

} // namespace ball_balancer
