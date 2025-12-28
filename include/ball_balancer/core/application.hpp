#pragma once

#include <memory>
#include <ball_balancer/core/types.hpp>

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
    /**
     * @brief Construct application with system parameters
     * @param params Physical parameters of the system
     */
    explicit Application(const SystemParameters& params = SystemParameters{});

    /**
     * @brief Destructor - automatically cleans up all subsystems
     */
    ~Application();

    // Non-copyable (owns unique resources)
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    // Movable (transfer ownership)
    Application(Application&&) noexcept = default;
    Application& operator=(Application&&) noexcept = default;

    /**
     * @brief Initialize all subsystems
     * @return true if initialization successful, false otherwise
     */
    bool initialize();

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
    StateVector current_state_;
    ControlVector current_control_;
    MeasurementVector current_measurement_;
};

} // namespace ball_balancer
