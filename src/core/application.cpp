#include <ball_balancer/core/application.hpp>
#include <ball_balancer/physics/simulator.hpp>
#include <ball_balancer/control/pid_controller.hpp>
#include <ball_balancer/control/state_estimator.hpp>
#include <ball_balancer/rendering/renderer.hpp>
#include <ball_balancer/gui/main_window.hpp>
#include <ball_balancer/gui/control_panel.hpp>
#include <ball_balancer/visualization/real_time_plotter.hpp>
#ifdef __EMSCRIPTEN__
    #include <GLES3/gl3.h>  // Emscripten's OpenGL ES 3.0 headers
    #include <emscripten.h>
    #include <emscripten/html5.h>
#else
    #include <glad/glad.h>  // Must be before GLFW
#endif
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
const int MAX_PHYSICS_STEPS = 5;

// Static pointer for main loop callback (used by both desktop and web)
static Application* g_app_instance = nullptr;

Application::Application(const SystemParameters& params)
    : params_(params)
    , window_(nullptr)
    , running_(false)
    , simulation_time_(0.0)
    , accumulator_(0.0)
    , kinematics_(params)
{
}

Application::~Application() {
    shutdown();
    // Clear global instance pointer to prevent dangling pointer
    g_app_instance = nullptr;
}

bool Application::initialize() {
    // ========================================================================
    // Initialize GLFW
    // ========================================================================
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << '\n';
        return false;
    }

    // OpenGL version hints - WebGL 2.0 (ES 3.0) for web, OpenGL 4.5 for desktop
#ifdef __EMSCRIPTEN__
    // WebGL 2.0 = OpenGL ES 3.0
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#else
    // OpenGL 4.5 Core Profile for desktop
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // Required on Mac
#endif

    // Create window
    window_ = glfwCreateWindow(1280, 720, "Ball Balancer", nullptr, nullptr);
    if (!window_) {
        std::cerr << "Failed to create GLFW window" << '\n';
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

    glfwSetFramebufferSizeCallback(window_, [](GLFWwindow* window, int width, int height) {
        auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (app && app->renderer_) {
            app->renderer_->resize(width, height);
        }
    });

    // Load OpenGL functions using GLAD (desktop only)
    // Emscripten doesn't need GLAD - OpenGL ES functions are already available
#ifndef __EMSCRIPTEN__
    if (!gladLoadGL()) {
        std::cerr << "Failed to initialize GLAD" << '\n';
        glfwDestroyWindow(window_);
        glfwTerminate();
        return false;
    }
#endif

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
        std::cerr << "Failed to initialize renderer" << '\n';
        return false;
    }

    // Initialize GUI
    if (!main_window_->initialize(window_)) {
        std::cerr << "Failed to initialize GUI" << '\n';
        return false;
    }

    // Reset simulation to initial state
    BallState  initial_ball  = make_initial_ball_state(0.0, 0.0, params_.arm_z_nominal + params_.ball_radius);
    TableState initial_table = make_initial_table_state(0.0, 0.0, params_.arm_z_nominal);
    simulator_->reset(initial_ball, initial_table);
    estimator_->reset(initial_ball, initial_table);
    controller_->reset();
    plotter_->clear();

    simulation_time_ = 0.0;
    accumulator_ = 0.0;
    running_ = false;

    // Initialize frame timing state
    last_frame_time_ = std::chrono::high_resolution_clock::now();
    prev_left_mouse_ = false;
    prev_right_mouse_ = false;
    plot_counter_ = 0;

    std::cout << "Application initialized successfully" << '\n';
    return true;
}

void Application::run() {
    running_ = true;
    g_app_instance = this;  // Set global instance for callback

    std::cout << "Starting main loop..." << '\n';

#ifdef __EMSCRIPTEN__
    // ========================================================================
    // Web Platform: Use emscripten_set_main_loop
    // ========================================================================
    // Emscripten requires using emscripten_set_main_loop to yield control back
    // to the browser event loop. The browser will call our callback on each frame.
    // Set the main loop callback (0 = use browser's requestAnimationFrame, 1 = simulate infinite loop)
    emscripten_set_main_loop(main_loop_iteration, 0, 1);
#else
    // ========================================================================
    // Desktop Platform: Traditional while loop
    // ========================================================================
    while (!glfwWindowShouldClose(window_) && running_) {
        main_loop_iteration();
    }

    std::cout << "Main loop ended" << '\n';
#endif
}

// Main loop iteration function (called once per frame)
// This is in the ball_balancer namespace and declared as a friend in Application
void main_loop_iteration() {
    Application* app = g_app_instance;
    if (!app) return;

    const double physics_dt = app->params_.control_dt;  // 0.01s = 100Hz

    // Check if window should close
    if (glfwWindowShouldClose(app->window_)) {
        app->running_ = false;
#ifdef __EMSCRIPTEN__
        emscripten_cancel_main_loop();
#endif
        return;
    }

    // ====================================================================
    // Timing
    // ====================================================================
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> frame_duration = current_time - app->last_frame_time_;
    app->last_frame_time_ = current_time;

    double frame_dt = frame_duration.count();
    app->accumulator_ += frame_dt;

    // ====================================================================
    // Poll Events
    // ====================================================================
    glfwPollEvents();

    // ====================================================================
    // Camera Input Handling
    // ====================================================================
    {
        // Check if CTRL key is pressed
        bool ctrl_pressed = (glfwGetKey(app->window_, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) ||
                           (glfwGetKey(app->window_, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

        app->renderer_->get_camera().set_ctrl_pressed(ctrl_pressed);

        // Get current mouse position
        double mouse_x, mouse_y;
        glfwGetCursorPos(app->window_, &mouse_x, &mouse_y);

        // Check mouse button states
        bool left_pressed = glfwGetMouseButton(app->window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        bool right_pressed = glfwGetMouseButton(app->window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

        // Track button state changes
        if (left_pressed != app->prev_left_mouse_) {
            app->renderer_->get_camera().on_mouse_button(0, left_pressed, mouse_x, mouse_y);
            app->prev_left_mouse_ = left_pressed;
        }
        if (right_pressed != app->prev_right_mouse_) {
            app->renderer_->get_camera().on_mouse_button(1, right_pressed, mouse_x, mouse_y);
            app->prev_right_mouse_ = right_pressed;
        }

        // Always update mouse position (camera will check if buttons are down)
        app->renderer_->get_camera().on_mouse_move(mouse_x, mouse_y);

        // ----------------------------------------------------------------
        // Keyboard camera controls (hold SHIFT)
        // SHIFT+W/S    : pan up/down
        // SHIFT+A/D    : pan left/right
        // SHIFT+Q/E    : rotate left/right (azimuth)
        // SHIFT+Z/X    : orbit up/down (elevation)
        // SHIFT+Up/Down: zoom in/out
        // ----------------------------------------------------------------
        bool shift_pressed = (glfwGetKey(app->window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
                             (glfwGetKey(app->window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        if (shift_pressed) {
            Camera& cam = app->renderer_->get_camera();
            const float pan_speed    = 0.003f * cam.get_distance();
            const float rotate_speed = 0.02f;
            const float zoom_speed   = 0.05f;

            if (glfwGetKey(app->window_, GLFW_KEY_W) == GLFW_PRESS)
                cam.pan(0.0f,  pan_speed);
            if (glfwGetKey(app->window_, GLFW_KEY_S) == GLFW_PRESS)
                cam.pan(0.0f, -pan_speed);
            if (glfwGetKey(app->window_, GLFW_KEY_A) == GLFW_PRESS)
                cam.pan(-pan_speed, 0.0f);
            if (glfwGetKey(app->window_, GLFW_KEY_D) == GLFW_PRESS)
                cam.pan( pan_speed, 0.0f);
            if (glfwGetKey(app->window_, GLFW_KEY_Q) == GLFW_PRESS)
                cam.rotate( rotate_speed, 0.0f);
            if (glfwGetKey(app->window_, GLFW_KEY_E) == GLFW_PRESS)
                cam.rotate(-rotate_speed, 0.0f);
            if (glfwGetKey(app->window_, GLFW_KEY_Z) == GLFW_PRESS)
                cam.rotate(0.0f,  rotate_speed);
            if (glfwGetKey(app->window_, GLFW_KEY_X) == GLFW_PRESS)
                cam.rotate(0.0f, -rotate_speed);
            if (glfwGetKey(app->window_, GLFW_KEY_UP) == GLFW_PRESS)
                cam.zoom(-zoom_speed);
            if (glfwGetKey(app->window_, GLFW_KEY_DOWN) == GLFW_PRESS)
                cam.zoom( zoom_speed);
        }
    }

    // ====================================================================
    // Fixed-Timestep Physics/Control Loop
    // ====================================================================

    // Hoist GUI state reads outside loop (values don't change mid-loop)
    SimulationState sim_state = app->main_window_->get_control_panel().get_simulation_state();
    ControllerState ctrl_state = app->main_window_->get_control_panel().get_controller_state();
    Eigen::Vector2d setpoint = app->main_window_->get_control_panel().get_setpoint();
    ControlVector manual_control = app->main_window_->get_control_panel().get_manual_control();

    // Detect Stopped/Paused → Running transition: snap ball Z to table surface
    if (sim_state == SimulationState::Running &&
        app->prev_sim_state_ != SimulationState::Running)
    {
        BallState  b = app->simulator_->get_ball_state();
        const TableState& t = app->simulator_->get_table_state();
        b.z_ball  = t.z_t + app->params_.ball_radius;
        b.vz_ball = 0.0;
        app->simulator_->set_ball_state(b);
        app->estimator_->reset(b, t);
    }
    app->prev_sim_state_ = sim_state;

    /* START SIMULATION */
    int steps_taken = 0;
    if (sim_state == SimulationState::Running) {
        while (app->accumulator_ >= physics_dt && steps_taken < MAX_PHYSICS_STEPS) {
            // Get current state
            BallState  true_ball  = app->simulator_->get_ball_state();
            TableState true_table = app->simulator_->get_table_state();

            // ── Servo mode: pre-inject FK-derived table pose so ball dynamics
            //    run against the correct (servo-driven) table state.
            //    Also pre-integrate servo angles for this sub-step.
            if (app->kinematics_mode_ == KinematicsMode::Servo) {
                const double tau = app->params_.servo_time_constant;
                for (int i = 0; i < 3; ++i) {
                    app->servo_angles_.alpha[i] +=
                        (app->servo_cmd_.alpha[i] - app->servo_angles_.alpha[i]) / tau * physics_dt;
                }
                auto fkResult = app->kinematics_.forwardKinematics(
                    app->servo_angles_, app->elbow_angles_, app->fk_method_);
                if (fkResult) {
                    app->elbow_angles_ = fkResult->elbow;
                    true_table.phi   = fkResult->phi;
                    true_table.theta = fkResult->theta;
                    true_table.z_t   = fkResult->z_t;
                    app->simulator_->set_table_state(true_table);
                    app->ik_failed_ = false;
                } else {
                    app->ik_failed_ = true;
                }
                // Re-read after injection
                true_ball  = app->simulator_->get_ball_state();
                true_table = app->simulator_->get_table_state();
            }

            // Get noisy measurement (simulates camera)
            MeasurementVector measurement = app->simulator_->get_measurement();

            // State estimation (Kalman filter)
            app->estimator_->update(measurement);
            BallState  estimated_ball  = app->estimator_->get_ball_state();
            TableState estimated_table = app->estimator_->get_table_state();

            // Compute control with PID or manual (use hoisted GUI state)
            ControlVector control;
            if (ctrl_state == ControllerState::Running) {
                control = app->controller_->compute(setpoint, toStateVector(estimated_ball, estimated_table));
            } else {
                control = manual_control;
            }

            // In Servo mode: table pose comes from FK, not from control commands.
            // Pass a control that matches the current FK table state so the
            // simulator's servo dynamics produce zero change (table stays frozen).
            ControlVector physics_control = control;
            if (app->kinematics_mode_ == KinematicsMode::Servo) {
                physics_control(control_index::VARPHI_X_CMD) = true_table.phi;
                physics_control(control_index::THETA_Y_CMD)  = true_table.theta;
                physics_control(control_index::TABLE_Z_CMD)  = true_table.z_t;
            }

            // Apply control to simulation (ball dynamics + table servo dynamics)
            app->simulator_->step(physics_dt, physics_control);

            // Predict next state (Kalman prediction step)
            app->estimator_->predict(control);

            // Update plots (subsample to 60 Hz to avoid overwhelming ImPlot)
            if (++app->plot_counter_ >= 2) {  // Every 2 physics steps = 50 Hz
                app->plotter_->update(app->simulation_time_, true_ball, true_table, control, setpoint);
                app->plot_counter_ = 0;
            }

            app->simulation_time_ += physics_dt;
            app->accumulator_ -= physics_dt;
            steps_taken++;
        }
    } else { // Simulator not running
        app->accumulator_ = 0.0;
    }

    // If we hit the limit, reset accumulator to avoid infinite catch-up
    if (steps_taken >= MAX_PHYSICS_STEPS) {
        app->accumulator_ = 0.0;
        std::cerr << "Warning: Physics running too slow, skipping time" << '\n';
    }

    // ====================================================================
    // Kinematics: IK / FK and servo angle integration
    //
    // State-flow depends on kinematics_mode_:
    //
    // Pose mode (table pose is the primary input):
    //   1. Physics/RK4 integrates table states from current_control_ (servo lag)
    //   2. Ball dynamics follow from table states
    //   3. IK maps table pose → servo_angles_ directly (no lag — pose is truth)
    //
    // Servo mode (servo angles are the primary input):
    //   1. servo_angles_ integrated toward servo_cmd_ with tau lag
    //   2. FK(servo_angles_) → table pose injected into simulator state
    //   3. Ball dynamics (RK4) run with the FK-derived table pose frozen
    // ====================================================================
    {
        const TableState cur_table = app->simulator_->get_table_state();

        if (app->kinematics_mode_ == KinematicsMode::Pose) {
            // ── Pose mode ────────────────────────────────────────────────
            // Table pose was already integrated by the physics loop above.
            // Compute IK to find the servo angles that realize this pose.
            const double phi   = cur_table.phi;
            const double theta = cur_table.theta;
            const double z_t   = cur_table.z_t;

            auto result = app->kinematics_.inverseKinematics(phi, theta, z_t);
            if (result) {
                // Servo angles track the table pose directly — no lag integration.
                // The table pose is the truth; servo angles are derived.
                app->servo_angles_ = *result;
                app->servo_cmd_    = *result;
                app->ik_failed_    = false;
            } else {
                app->ik_failed_ = true;
                // Keep last valid servo_angles_ so rendering doesn't jump.
            }
        } else {
            // ── Servo mode ───────────────────────────────────────────────
            // Servo integration and FK injection are handled inside the
            // physics sub-step loop above (when Running). When paused,
            // apply FK once so the renderer shows consistent geometry.
            // if (sim_state != SimulationState::Running) {
            //     auto fkPaused = app->kinematics_.forwardKinematics(
            //         app->servo_angles_, app->elbow_angles_);
            //     if (fkPaused) {
            //         app->elbow_angles_ = fkPaused->elbow;
            //         StateVector s = app->simulator_->get_state();
            //         s(state_index::VARPHI_X) = fkPaused->phi;
            //         s(state_index::THETA_Y)  = fkPaused->theta;
            //         s(state_index::Z_TABLE)  = fkPaused->z_t;
            //         app->simulator_->set_state(s);
            //         app->ik_failed_ = false;
            //     } else {
            //         app->ik_failed_ = true;
            //     }
            // }
        }
    }

    // ====================================================================
    // Apply Manual State (when paused and a slider changed)
    // ====================================================================
    if (app->main_window_->get_control_panel().is_manual_state_changed()) {
        BallState  manual_ball  = app->main_window_->get_control_panel().get_manual_ball_state();
        TableState manual_table = app->main_window_->get_control_panel().get_manual_table_state();
        app->simulator_->set_ball_state(manual_ball);
        app->simulator_->set_table_state(manual_table);
        app->estimator_->reset(manual_ball, manual_table);
        app->main_window_->get_control_panel().clear_manual_state_changed();
    }

    // Keep manual state sliders in sync with current simulator state when paused
    if (sim_state != SimulationState::Running) {
        app->main_window_->get_control_panel().sync_manual_state(
            app->simulator_->get_ball_state(), app->simulator_->get_table_state());
    }

    // ====================================================================
    // Handle Reset Request
    // ====================================================================
    if (app->main_window_->get_control_panel().should_reset()) {
        BallState  initial_ball  = make_initial_ball_state(0.0, 0.0, app->params_.arm_z_nominal + app->params_.ball_radius);
        TableState initial_table = make_initial_table_state(0.0, 0.0, app->params_.arm_z_nominal);
        app->simulator_->reset(initial_ball, initial_table);
        app->estimator_->reset(initial_ball, initial_table);
        app->controller_->reset();
        app->plotter_->clear();
        app->simulation_time_ = 0.0;
        app->accumulator_ = 0.0;

        // Reset frame timing state to prevent stale time deltas
        app->last_frame_time_ = std::chrono::high_resolution_clock::now();
        app->plot_counter_ = 0;

        app->main_window_->get_control_panel().clear_reset_flag();
        std::cout << "Simulation reset" << '\n';
    }

    // ====================================================================
    // Rendering
    // ====================================================================

    // Get current state for rendering
    const BallState  current_ball  = app->simulator_->get_ball_state();
    const TableState current_table = app->simulator_->get_table_state();

    // Clear screen and depth buffer ONCE before rendering
    glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable depth testing for 3D scene
    glEnable(GL_DEPTH_TEST);

    // Render 3D scene
    app->renderer_->render(current_ball, current_table);

    // Render arm legs: compute G, E, T for each arm in physics coords, pass to renderer.
    // Use tableAttachPointFromBeta when elbow angles are available (FK-consistent geometry);
    // fall back to tableAttachPoint from pose states otherwise (Pose mode).
    {
        const double phi   = current_table.phi;
        const double theta = current_table.theta;
        const double z_t   = current_table.z_t;

        std::array<std::array<std::array<float, 3>, 3>, 3> arm_pts;
        for (int i = 0; i < 3; ++i) {
            const auto& kin = app->kinematics_;
            const auto G = kin.groundPoint(i);
            const auto E = kin.elbowPosition(i, app->servo_angles_.alpha[i]);
            // In Servo mode the elbow angles are valid FK outputs; use them
            // so T is consistent with the β-constraint geometry.
            // In Pose mode, derive T from the table pose states directly.
            std::array<double, 3> T;
            if (app->kinematics_mode_ == KinematicsMode::Servo) {
                T = kin.tableAttachPointFromBeta(
                        i, app->servo_angles_.alpha[i], app->elbow_angles_.beta[i]);
            } else {
                T = kin.tableAttachPoint(i, phi, theta, z_t);
            }
            arm_pts[i][0] = {static_cast<float>(G[0]), static_cast<float>(G[1]), static_cast<float>(G[2])};
            arm_pts[i][1] = {static_cast<float>(E[0]), static_cast<float>(E[1]), static_cast<float>(E[2])};
            arm_pts[i][2] = {static_cast<float>(T[0]), static_cast<float>(T[1]), static_cast<float>(T[2])};
        }
        app->renderer_->render_legs(arm_pts);
    }

    // Render GUI (on top of 3D scene)
    app->main_window_->begin_frame();

    // Build ArmStatus for GUI exchange
    ArmStatus arm_status;
    arm_status.servo_angles  = app->servo_angles_;
    arm_status.servo_cmd     = app->servo_cmd_;
    arm_status.ik_failed     = app->ik_failed_;
    arm_status.mode          = app->kinematics_mode_;
    arm_status.show_legs     = app->renderer_->get_show_legs();
    arm_status.fk_method     = app->fk_method_;
    // Populate elbow angles (β) from last FK solve
    arm_status.elbow_angles  = app->elbow_angles_;
    // Populate geometry from current params
    arm_status.req_L1        = app->params_.arm_L1;
    arm_status.req_L2        = app->params_.arm_L2;
    arm_status.req_Rg        = app->params_.arm_Rg;
    arm_status.req_Rt        = app->params_.arm_Rt;
    arm_status.req_z_nominal = app->params_.arm_z_nominal;
    // Populate FK result if in Servo mode (reuse already-computed elbow_angles_)
    if (app->kinematics_mode_ == KinematicsMode::Servo) {
        arm_status.fk_phi   = current_table.phi;
        arm_status.fk_theta = current_table.theta;
        arm_status.fk_z     = current_table.z_t;
        arm_status.fk_valid = !app->ik_failed_;
    }

    app->main_window_->render(current_ball, current_table, *app->controller_, *app->estimator_,
                              *app->renderer_, *app->plotter_,
                              app->simulator_->isInContact(), arm_status);
    app->main_window_->end_frame();

    // Apply ArmStatus changes written by GUI
    if (arm_status.show_legs_changed) {
        app->renderer_->set_show_legs(arm_status.show_legs);
    }
    if (arm_status.mode_changed) {
        app->kinematics_mode_ = arm_status.mode;
    }
    if (arm_status.cmd_changed && app->kinematics_mode_ == KinematicsMode::Servo) {
        app->servo_cmd_ = arm_status.servo_cmd;
        // When paused, snap servo_angles_ to cmd immediately so FK sees the new pose.
        if (sim_state != SimulationState::Running) {
            app->servo_angles_ = app->servo_cmd_;
            app->elbow_angles_ = arm_status.elbow_angles;  // Also update elbow angles from GUI sliders
        }
    }
    if (arm_status.fk_method_changed) {
        app->fk_method_ = arm_status.fk_method;
    }
    if (arm_status.geom_changed) {
        app->params_.arm_L1        = arm_status.req_L1;
        app->params_.arm_L2        = arm_status.req_L2;
        app->params_.arm_Rg        = arm_status.req_Rg;
        app->params_.arm_Rt        = arm_status.req_Rt;
        app->params_.arm_z_nominal = arm_status.req_z_nominal;
        // Sync table_radius = arm_Rt, then propagate to all subsystems
        app->params_.initialize();
        app->kinematics_ = TableKinematics(app->params_);
        app->simulator_->set_table_radius(app->params_.table_radius);
        app->renderer_->set_table_radius(static_cast<float>(app->params_.table_radius));
    }

    // Check if user requested exit from GUI
    if (app->main_window_->should_exit()) {
        app->running_ = false;
#ifdef __EMSCRIPTEN__
        emscripten_cancel_main_loop();
#endif
    }

    // Swap buffers
    glfwSwapBuffers(app->window_);
}

void Application::shutdown() {
    std::cout << "Shutting down application..." << '\n';

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

    std::cout << "Application shutdown complete" << '\n';
}

} // namespace ball_balancer
