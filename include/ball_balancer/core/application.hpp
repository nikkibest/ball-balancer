#pragma once

#include <memory>
#include <chrono>
#include <ball_balancer/core/types.hpp>
#include <ball_balancer/physics/table_kinematics.hpp>
#include <ball_balancer/gui/control_panel.hpp>

/**
 * @file application.hpp
 * @brief Main application class that coordinates all subsystems
 *
 * The Application class is the top-level coordinator following RAII principles.
 * It owns all major subsystems and manages their lifecycle.
 *
 * @see research/cpp-best-practices-modern-programming.md (RAII pattern)
 */

// Forward declaration for GLFW
struct GLFWwindow;

namespace ball_balancer {

// Forward declarations to avoid circular dependencies
class Simulator;
class PIDController;
class StateEstimator;
class Renderer;
class MainWindow;
class RealTimePlotter;

// Forward declaration of main loop callback function
void main_loop_iteration();

/**
 * @brief Main application class coordinating all subsystems
 *
 * Responsibilities:
 * - Initialize and own all subsystems (physics, control, rendering, GUI)
 * - Run main event loop
 * - Coordinate communication between subsystems
 * - Ensure proper resource cleanup via RAII
 *
 * Ownership Model (RAII):
 * - Application owns all subsystems via unique_ptr
 * - Subsystems are destroyed automatically when Application is destroyed
 * - No manual resource management needed
 */
class Application {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct application with system parameters
     * @param params Physical parameters of the system
     */
    explicit Application(const SystemParameters& params = SystemParameters{});

    /**
     * @brief Destructor - automatically cleans up all subsystems
     */
    ~Application();

    // Friend function for main loop callback (needed for Emscripten)
    friend void main_loop_iteration();

    // Non-copyable and non-movable (owns unique resources including raw GLFWwindow*)
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;
    Application(Application&&) = delete;
    Application& operator=(Application&&) = delete;

    /**
     * @brief Initialize all subsystems
     * @return true if initialization successful, false otherwise
     */
    [[nodiscard]] bool initialize();

    /**
     * @brief Run main application loop
     *
     * Loop structure:
     * 1. Process input events (GUI, keyboard)
     * 2. Update physics simulation (integrate ODEs)
     * 3. Run state estimator (Kalman filter)
     * 4. Compute control output (PID/LQR)
     * 5. Apply control to simulation
     * 6. Update visualization (render 3D scene, plots, GUI)
     * 7. Check for exit condition
     */
    void run();

    /**
     * @brief Shutdown and cleanup all subsystems
     */
    void shutdown();

    /**
     * @brief Check if application is running
     * @return true if running, false if should exit
     */
    bool is_running() const;

    // -----------------------------------------------------------------------
    // Arm mechanism accessors (used by GUI)
    // -----------------------------------------------------------------------

    const ServoAngles& get_servo_angles() const { return servo_angles_; }
    const ServoAngles& get_servo_cmd()    const { return servo_cmd_; }
    bool               ik_failed()        const { return ik_failed_; }
    KinematicsMode     kinematics_mode()  const { return kinematics_mode_; }

    /// Set kinematics mode (called by GUI mode radio buttons).
    void set_kinematics_mode(KinematicsMode mode) { kinematics_mode_ = mode; }

    /// Write commanded servo angles when in Servo mode (called by GUI sliders).
    void set_servo_cmd(const ServoAngles& cmd) { servo_cmd_ = cmd; }

    /// Access kinematics for geometry parameter rebuilding from GUI.
    TableKinematics& get_kinematics() { return kinematics_; }

private:
    /**
     * @brief Update simulation by one time step
     * @param dt Time step (seconds)
     */
    void update(double dt);

    /**
     * @brief Render current frame
     */
    void render();

    // System parameters
    SystemParameters params_;

    // GLFW window (raw pointer managed by GLFW)
    GLFWwindow* window_;

    // Subsystem ownership (RAII - unique_ptr for exclusive ownership)
    std::unique_ptr<Simulator> simulator_;           // Physics simulation
    std::unique_ptr<PIDController> controller_;      // PID controller
    std::unique_ptr<StateEstimator> estimator_;      // Kalman filter
    std::unique_ptr<Renderer> renderer_;             // OpenGL renderer
    std::unique_ptr<MainWindow> main_window_;        // ImGui main window
    std::unique_ptr<RealTimePlotter> plotter_;       // ImPlot visualization

    // Application state
    bool running_{false};
    double current_time_{0.0};
    double simulation_time_{0.0};
    double accumulator_{0.0};
    ControlVector current_control_;
    MeasurementVector current_measurement_;

    // Frame timing state (moved from static locals in main_loop_iteration)
    std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_time_;
    bool prev_left_mouse_{false};
    bool prev_right_mouse_{false};
    int plot_counter_{0};
    SimulationState prev_sim_state_{SimulationState::Stopped};

    // -----------------------------------------------------------------------
    // Arm mechanism kinematics state
    // -----------------------------------------------------------------------

    /// Stateless kinematics helper (uses params_ geometry fields).
    TableKinematics kinematics_;

    /// Current integrated servo angles α_i (rad). Driven toward servo_cmd_.
    ServoAngles servo_angles_{};

    /// Commanded servo angles α_cmd_i. Set by IK (Pose mode) or GUI (Servo mode).
    ServoAngles servo_cmd_{};

    /// Current upper-link elbow angles β_i (rad). Updated by FK every frame.
    ElbowAngles elbow_angles_{};

    /// Active kinematics mode (Pose = IK, Servo = FK).
    KinematicsMode kinematics_mode_{KinematicsMode::Servo};

    /// True when the last IK call returned nullopt (pose out of reach).
    bool ik_failed_{false};

    /// FK method used in Servo mode (selectable from GUI).
    FKMethod fk_method_{FKMethod::NewtonRaphson};

    // -----------------------------------------------------------------------
    // FK finite-difference history (Servo mode only)
    // -----------------------------------------------------------------------

    /// Pose snapshot from the previous FK step.
    struct FKPose { double phi{0.0}, theta{0.0}, z_t{0.0}; };
    /// Velocity snapshot from the previous FK step.
    struct FKVel  { double phi_dot{0.0}, theta_dot{0.0}, z_t_dot{0.0}; };

    FKPose prev_fk_pose_{};
    FKVel  prev_fk_vel_{};
    /// True once at least one FK step has been stored; gates derivative computation.
    bool   fk_prev_valid_{false};
};

} // namespace ball_balancer
