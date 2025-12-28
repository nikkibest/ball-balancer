#pragma once

#include <Eigen/Dense>
#include <cstddef>

/**
 * @file types.hpp
 * @brief Core type definitions for the ball balancer system
 *
 * This file defines all common types used across modules. It serves as the
 * interface contract between agents (Physics, Control, Eigen, etc.).
 *
 * State Space Representation:
 * - State vector (6D): [x, y, vx, vy, theta_x, theta_y]
 *   * x, y: Ball position on table surface (meters)
 *   * vx, vy: Ball velocity (m/s)
 *   * theta_x, theta_y: Table tilt angles (radians)
 *
 * - Control vector (2D): [theta_x_cmd, theta_y_cmd]
 *   * Commanded table tilt angles (radians)
 *
 * - Measurement vector (2D): [x_meas, y_meas]
 *   * Ball position from camera (meters)
 *
 * @see research/eigen-cpp-linear-algebra-best-practices.md
 * @see research/ode-physical-system-modeling-cpp.md
 */

namespace ball_balancer {

// ============================================================================
// State-Space Types (Fixed-size for performance)
// ============================================================================

/**
 * @brief System state vector (6D)
 *
 * Layout: [x, y, vx, vy, theta_x, theta_y]
 * - x, y: Ball position (m)
 * - vx, vy: Ball velocity (m/s)
 * - theta_x, theta_y: Table tilt angles (rad)
 */
using StateVector = Eigen::Matrix<double, 6, 1>;

/**
 * @brief Control input vector (2D)
 *
 * Layout: [theta_x_cmd, theta_y_cmd]
 * - Commanded table tilt angles (rad)
 */
using ControlVector = Eigen::Matrix<double, 2, 1>;

/**
 * @brief Measurement vector (2D)
 *
 * Layout: [x_meas, y_meas]
 * - Ball position from camera (m)
 */
using MeasurementVector = Eigen::Matrix<double, 2, 1>;

/**
 * @brief State derivative vector (6D)
 *
 * Time derivative of state: dx/dt
 */
using StateDerivative = StateVector;

// ============================================================================
// System Matrices (State-space representation)
// ============================================================================

/**
 * @brief System dynamics matrix A (6x6)
 *
 * For linear system: dx/dt = A*x + B*u
 * Used in LQR controller design and Kalman filter
 */
using SystemMatrix = Eigen::Matrix<double, 6, 6>;

/**
 * @brief Control input matrix B (6x2)
 *
 * Maps control inputs to state derivatives
 */
using ControlMatrix = Eigen::Matrix<double, 6, 2>;

/**
 * @brief Measurement matrix C (2x6)
 *
 * Maps state to measurements: y = C*x
 */
using MeasurementMatrix = Eigen::Matrix<double, 2, 6>;

/**
 * @brief Feedthrough matrix D (2x2)
 *
 * Direct feedthrough: y = C*x + D*u
 * Typically zero for most systems
 */
using FeedthroughMatrix = Eigen::Matrix<double, 2, 2>;

// ============================================================================
// Controller Types
// ============================================================================

/**
 * @brief LQR gain matrix K (2x6)
 *
 * Optimal control law: u = -K*x
 */
using LQRGainMatrix = Eigen::Matrix<double, 2, 6>;

// Note: PIDGains struct is defined in control/pid_controller.hpp

// ============================================================================
// Kalman Filter Types
// ============================================================================

/**
 * @brief Process noise covariance Q (6x6)
 *
 * Models uncertainty in system dynamics
 */
using ProcessNoiseMatrix = SystemMatrix;

/**
 * @brief Measurement noise covariance R (2x2)
 *
 * Models uncertainty in measurements (camera noise)
 */
using MeasurementNoiseMatrix = Eigen::Matrix<double, 2, 2>;

/**
 * @brief State estimate covariance P (6x6)
 *
 * Uncertainty in state estimate
 */
using StateCovarianceMatrix = SystemMatrix;

/**
 * @brief Kalman gain L (6x2)
 *
 * Optimal estimator gain: x_est = x_pred + L*(y - C*x_pred)
 */
using KalmanGainMatrix = Eigen::Matrix<double, 6, 2>;

// ============================================================================
// Physical Constants and Parameters
// ============================================================================

/**
 * @brief Physical parameters of the ball balancer system
 */
struct SystemParameters {
    // Ball properties
    double ball_mass{0.027};           // Mass of ball (kg) - ping pong ball
    double ball_radius{0.020};         // Radius of ball (m) - 40mm diameter
    double ball_inertia{0.0};          // Moment of inertia (computed from mass/radius)

    // Table properties
    double table_length{0.5};          // Table dimension X (m)
    double table_width{0.5};           // Table dimension Y (m)
    double max_tilt_angle{0.174};      // Max tilt angle (rad) = 10 degrees

    // Servo dynamics (simplified first-order)
    double servo_time_constant{0.05};  // Servo response time (s)

    // Environment
    double gravity{9.81};              // Gravitational acceleration (m/s²)
    double friction_coeff{0.01};       // Rolling friction coefficient

    // Camera/sensing
    double camera_fps{60.0};           // Camera frame rate (Hz)
    double camera_noise_std{0.001};    // Camera measurement noise std (m)

    // Control timing
    double control_dt{0.01};           // Control loop time step (s) = 100Hz

    /**
     * @brief Initialize derived parameters
     */
    void initialize() {
        // Ball moment of inertia for solid sphere: I = (2/5)*m*r^2
        ball_inertia = 0.4 * ball_mass * ball_radius * ball_radius;
    }
};

// ============================================================================
// State Indices (for readability)
// ============================================================================

namespace state_index {
    constexpr std::size_t X = 0;         // Ball x position
    constexpr std::size_t Y = 1;         // Ball y position
    constexpr std::size_t VX = 2;        // Ball x velocity
    constexpr std::size_t VY = 3;        // Ball y velocity
    constexpr std::size_t THETA_X = 4;   // Table tilt angle x
    constexpr std::size_t THETA_Y = 5;   // Table tilt angle y
}

namespace control_index {
    constexpr std::size_t THETA_X_CMD = 0;  // Commanded tilt x
    constexpr std::size_t THETA_Y_CMD = 1;  // Commanded tilt y
}

namespace measurement_index {
    constexpr std::size_t X_MEAS = 0;    // Measured x position
    constexpr std::size_t Y_MEAS = 1;    // Measured y position
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Create initial state vector
 * @param x Initial x position (m)
 * @param y Initial y position (m)
 * @return Initialized state vector with zero velocity and tilt
 */
inline StateVector make_initial_state(double x = 0.0, double y = 0.0) {
    StateVector state = StateVector::Zero();
    state(state_index::X) = x;
    state(state_index::Y) = y;
    return state;
}

/**
 * @brief Create control vector from tilt angles
 * @param theta_x Table tilt angle X (rad)
 * @param theta_y Table tilt angle Y (rad)
 * @return Control vector
 */
inline ControlVector make_control(double theta_x, double theta_y) {
    ControlVector control;
    control(control_index::THETA_X_CMD) = theta_x;
    control(control_index::THETA_Y_CMD) = theta_y;
    return control;
}

/**
 * @brief Create measurement vector from ball position
 * @param x Measured x position (m)
 * @param y Measured y position (m)
 * @return Measurement vector
 */
inline MeasurementVector make_measurement(double x, double y) {
    MeasurementVector meas;
    meas(measurement_index::X_MEAS) = x;
    meas(measurement_index::Y_MEAS) = y;
    return meas;
}

} // namespace ball_balancer
