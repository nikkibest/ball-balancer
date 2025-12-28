#include <ball_balancer/core/application.hpp>
#include <ball_balancer/physics/simulator.hpp>
#include <ball_balancer/control/pid_controller.hpp>
#include <ball_balancer/control/state_estimator.hpp>
#include <ball_balancer/rendering/renderer.hpp>
#include <ball_balancer/gui/main_window.hpp>
#include <ball_balancer/visualization/real_time_plotter.hpp>
#include <glad/glad.h>  // Must be before GLFW
#include <GLFW/glfw3.h>
#include <iostream>
#include <chrono>

/**
 * @file application.cpp
 * @brief Implementation of main application
 *
 * Wires together all subsystems:
 * - Physics simulation
 * - Control system (PID + Kalman)
 * - 3D rendering
 * - GUI (control panel)
 * - Real-time plotting
 *
 * Implements main loop with proper timing and data flow.
 */

namespace ball_balancer {

Application::Application(const SystemParameters& params)
    : params_(params)
    , window_(nullptr)
    , running_(false)
    , simulation_time_(0.0)
{
}

Application::~Application() {
    shutdown();
}

bool Application::initialize() {
    // ========================================================================
    // Initialize GLFW
    // ========================================================================
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // OpenGL 4.5 Core Profile (matches renderer requirements)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // Required on Mac

    // Create window
    window_ = glfwCreateWindow(1280, 720, "Ball Balancer", nullptr, nullptr);
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);  // Enable vsync

    // Store 'this' pointer for GLFW callbacks
    glfwSetWindowUserPointer(window_, this);

    // Set up mouse callbacks
    glfwSetMouseButtonCallback(window_, [](GLFWwindow* window, int button, int action, int mods) {
        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (app && app->renderer_) {
            double x, y;
            glfwGetCursorPos(window, &x, &y);
            bool pressed = (action == GLFW_PRESS);
            app->renderer_->get_camera().on_mouse_button(button, pressed, x, y);
        }
    });

    glfwSetCursorPosCallback(window_, [](GLFWwindow* window, double x, double y) {
        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (app && app->renderer_) {
            app->renderer_->get_camera().on_mouse_move(x, y);
        }
    });

    glfwSetScrollCallback(window_, [](GLFWwindow* window, double x_offset, double y_offset) {
        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (app && app->renderer_) {
            app->renderer_->get_camera().on_mouse_scroll(y_offset);
        }
    });

    // Load OpenGL functions using GLAD
    if (!gladLoadGL()) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        glfwDestroyWindow(window_);
        glfwTerminate();
        return false;
    }

    // ========================================================================
    // Initialize subsystems
    // ========================================================================

    // Create subsystems (RAII ownership via unique_ptr)
    simulator_ = std::make_unique<Simulator>(params_);
    controller_ = std::make_unique<PIDController>(params_);
    estimator_ = std::make_unique<StateEstimator>(params_);
    renderer_ = std::make_unique<Renderer>();
    main_window_ = std::make_unique<MainWindow>(params_);
    plotter_ = std::make_unique<RealTimePlotter>(params_);

    // Initialize renderer
    if (!renderer_->initialize(1280, 720)) {
        std::cerr << "Failed to initialize renderer" << std::endl;
        return false;
    }

    // Initialize GUI
    if (!main_window_->initialize(window_)) {
        std::cerr << "Failed to initialize GUI" << std::endl;
        return false;
    }

    // Reset simulation to initial state
    StateVector initial_state = StateVector::Zero();
    simulator_->reset(initial_state);
    estimator_->reset(initial_state);
    controller_->reset();
    plotter_->clear();

    simulation_time_ = 0.0;
    running_ = false;

    std::cout << "Application initialized successfully" << std::endl;
    return true;
}

void Application::run() {
    running_ = true;

    // Timing variables
    auto last_frame_time = std::chrono::high_resolution_clock::now();
    double accumulator = 0.0;
    const double physics_dt = params_.control_dt;  // 0.01s = 100Hz

    std::cout << "Starting main loop..." << std::endl;

    // Main loop
    while (!glfwWindowShouldClose(window_) && running_) {
        // ====================================================================
        // Timing
        // ====================================================================
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frame_duration = current_time - last_frame_time;
        last_frame_time = current_time;

        double frame_dt = frame_duration.count();
        accumulator += frame_dt;

        // ====================================================================
        // Poll Events
        // ====================================================================
        glfwPollEvents();

        // ====================================================================
        // Fixed-Timestep Physics/Control Loop
        // ====================================================================
        // Use fixed timestep for deterministic simulation
        // This ensures consistent physics regardless of frame rate

        while (accumulator >= physics_dt) {
            // Check simulation state from GUI
            SimulationState sim_state = main_window_->get_control_panel().get_state();

            if (sim_state == SimulationState::Running) {
                // Get current state
                StateVector true_state = simulator_->get_state();

                // Get noisy measurement (simulates camera)
                MeasurementVector measurement = simulator_->get_measurement();

                // State estimation (Kalman filter)
                estimator_->update(measurement);
                StateVector estimated_state = estimator_->get_state();

                // Get setpoint from GUI
                Eigen::Vector2d setpoint = main_window_->get_control_panel().get_setpoint();

                // Compute control (PID)
                ControlVector control = controller_->compute(setpoint, estimated_state);

                // Apply control to simulation
                simulator_->step(physics_dt, control);

                // Predict next state (Kalman prediction step)
                estimator_->predict(control);

                // Update plots (subsample to 60 Hz to avoid overwhelming ImPlot)
                static int plot_counter = 0;
                if (++plot_counter >= 2) {  // Every 2 physics steps = 50 Hz
                    plotter_->update(simulation_time_, true_state, control, setpoint);
                    plot_counter = 0;
                }

                simulation_time_ += physics_dt;
            }

            accumulator -= physics_dt;
        }

        // ====================================================================
        // Handle Reset Request
        // ====================================================================
        if (main_window_->get_control_panel().should_reset()) {
            StateVector initial_state = StateVector::Zero();
            simulator_->reset(initial_state);
            estimator_->reset(initial_state);
            controller_->reset();
            plotter_->clear();
            simulation_time_ = 0.0;

            main_window_->get_control_panel().clear_reset_flag();
            std::cout << "Simulation reset" << std::endl;
        }

        // ====================================================================
        // Rendering
        // ====================================================================

        // Get current state for rendering
        StateVector current_state = simulator_->get_state();

        // Clear screen
        glClearColor(0.15f, 0.15f, 0.18f, 1.0f);  // Dark blue-gray background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render 3D scene
        renderer_->render(current_state);

        // Render GUI (on top of 3D scene)
        main_window_->begin_frame();
        main_window_->render(current_state, *controller_, *estimator_, *renderer_, *plotter_);
        main_window_->end_frame();

        // Check if user requested exit from GUI
        if (main_window_->should_exit()) {
            running_ = false;
        }

        // Swap buffers
        glfwSwapBuffers(window_);
    }

    std::cout << "Main loop ended" << std::endl;
}

void Application::shutdown() {
    std::cout << "Shutting down application..." << std::endl;

    // Shutdown subsystems (RAII will handle cleanup, but explicit shutdown for GUI)
    if (main_window_) {
        main_window_->shutdown();
    }

    // Destroy subsystems in reverse order
    plotter_.reset();
    main_window_.reset();
    renderer_.reset();
    estimator_.reset();
    controller_.reset();
    simulator_.reset();

    // Cleanup GLFW
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();

    std::cout << "Application shutdown complete" << std::endl;
}

} // namespace ball_balancer
