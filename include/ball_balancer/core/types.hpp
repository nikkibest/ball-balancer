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
 * - State vector (9D): [x, y, z_ball, vx, vy, vz_ball, varphi_x, theta_y, z_table]
 *   * x, y: Ball horizontal position on table surface (meters)
 *   * z_ball: Ball vertical position (meters)
 *   * vx, vy: Ball horizontal velocity (m/s)
 *   * vz_ball: Ball vertical velocity (m/s)
 *   * varphi_x, theta_y: Table tilt angles (radians)
 *   * z_table: Table vertical translation (meters)
 *
 * - Control vector (2D): [varphi_x_cmd, theta_y_cmd]
 *   * Commanded table tilt angles (radians)
 *
 * - Measurement vector (2D): [x_meas, y_meas]
 *   * Ball position from camera (meters)
 *
 * Note: z_table has no actuation — it is set manually via the GUI.
 * z_ball and vz_ball are fully live states driven by BallDynamics.
 *
 * @see research/eigen-cpp-linear-algebra-best-practices.md
 * @see research/ode-physical-system-modeling-cpp.md
 */

namespace ball_balancer {

// ============================================================================
// State-Space Types (Fixed-size for performance)
// ============================================================================

/**
 * @brief System state vector (9D)
 *
 * Layout: [x, y, z_ball, vx, vy, vz_ball, varphi_x, theta_y, z_table]
 * - x, y: Ball horizontal position (m)
 * - z_ball: Ball vertical position (m)
 * - vx, vy: Ball horizontal velocity (m/s)
 * - vz_ball: Ball vertical velocity (m/s)
 * - varphi_x, theta_y: Table tilt angles (rad)
 * - z_table: Table vertical translation (m) — no actuation (table Z stub)
 */
using StateVector = Eigen::Matrix<double, 9, 1>;

/**
 * @brief Control input vector (2D)
 *
 * Layout: [varphi_x_cmd, theta_y_cmd, table_z_cmd]
 * - Commanded table tilt angles (rad)
 * - Commanded table height (m)
 */
using ControlVector = Eigen::Matrix<double, 3, 1>;

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
 * @brief System dynamics matrix A (9x9)
 *
 * For linear system: dx/dt = A*x + B*u
 * Used in LQR controller design and Kalman filter.
 * Note: Rows/columns for z_ball, vz_ball, z_table are all zeros (no dynamics).
 */
using SystemMatrix = Eigen::Matrix<double, 9, 9>;

/**
 * @brief Control input matrix B (9x3)
 *
 * Maps control inputs to state derivatives.
 * Third column (TABLE_Z_CMD) is zero — table Z has no linearized servo dynamics.
 */
using ControlMatrix = Eigen::Matrix<double, 9, 3>;

/**
 * @brief Measurement matrix C (2x9)
 *
 * Maps state to measurements: y = C*x
 */
using MeasurementMatrix = Eigen::Matrix<double, 2, 9>;

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
 * @brief LQR gain matrix K (2x9)
 *
 * Optimal control law: u = -K*x
 */
using LQRGainMatrix = Eigen::Matrix<double, 2, 9>;

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
 * @brief State estimate covariance P (9x9)
 *
 * Uncertainty in state estimate
 */
using StateCovarianceMatrix = SystemMatrix;

/**
 * @brief Kalman gain L (9x2)
 *
 * Optimal estimator gain: x_est = x_pred + L*(y - C*x_pred)
 */
using KalmanGainMatrix = Eigen::Matrix<double, 9, 2>;

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
    double ball_inertia{0.0};          // Moment of inertia (auto-computed in constructor via initialize())

    // Table properties
    double table_length{0.5};          // Table dimension X (m)
    double table_width{0.5};           // Table dimension Y (m)
    double max_tilt_angle{0.174};      // Max tilt angle (rad) = 10 degrees
    double min_table_height{0.0};      // Minimum table height (m)
    double max_table_height{0.5};      // Maximum table height (m)

    // Servo dynamics (simplified first-order)
    double servo_time_constant{0.05};  // Servo response time (s)

    // Environment
    double gravity{9.81};              // Gravitational acceleration (m/s²)
    double friction_coeff{0.01};       // Legacy rolling friction coefficient (unused by BallDynamics)
    double viscous_friction_coeff{0.1}; // Viscous friction b (N·s/m) used by BallDynamics — smooth, RK4-stable
    double bounce_coeff{0.5};          // Coefficient of restitution e (0=inelastic, 1=elastic)

    // Camera/sensing
    double camera_fps{60.0};           // Camera frame rate (Hz)
    double camera_noise_std{0.001};    // Camera measurement noise std (m)

    // Control timing
    double control_dt{0.01};           // Control loop time step (s) = 100Hz

    // Arm mechanism geometry (used by TableKinematics)
    double arm_L1{0.08};              // Lower link length: servo pivot to elbow (m)
    double arm_L2{0.08};              // Upper link length: elbow to table attachment (m)
    double arm_Rg{0.10};              // Ground mounting radius: centre to servo pivot (m)
    double arm_Rt{0.07};              // Table mounting radius: centre to table attachment (m)
    double arm_z_nominal{0.12};       // Nominal table height for FK warm-start (m)

    /**
     * @brief Default constructor - initializes derived parameters
     */
    SystemParameters() {
        initialize();
    }

    /**
     * @brief Initialize derived parameters (called automatically by constructor)
     *
     * Computes ball_inertia from ball_mass and ball_radius.
     * @note This is called automatically in the default constructor.
     *       Only call manually if you modify ball_mass or ball_radius after construction.
     */
    void initialize() {
        // Ball moment of inertia for solid sphere: I = (2/5)*m*r^2
        ball_inertia = 0.4 * ball_mass * ball_radius * ball_radius;
    }

    /**
     * @brief Print system parameters to stdout
     */
    void print() const;
};

// ============================================================================
// State Indices (for readability)
// ============================================================================

namespace state_index {
    constexpr std::size_t X = 0;         // Ball x position (m)
    constexpr std::size_t Y = 1;         // Ball y position (m)
    constexpr std::size_t Z_BALL = 2;    // Ball vertical position (m)
    constexpr std::size_t VX = 3;        // Ball x velocity (m/s)
    constexpr std::size_t VY = 4;        // Ball y velocity (m/s)
    constexpr std::size_t VZ_BALL = 5;   // Ball vertical velocity (m/s)
    constexpr std::size_t VARPHI_X = 6;   // Table tilt angle x (rad)
    constexpr std::size_t THETA_Y = 7;   // Table tilt angle y (rad)
    constexpr std::size_t Z_TABLE = 8;   // Table vertical translation (m) — no actuation
}

namespace control_index {
    constexpr std::size_t VARPHI_X_CMD = 0;  // Commanded tilt x
    constexpr std::size_t THETA_Y_CMD = 1;  // Commanded tilt y
    constexpr std::size_t TABLE_Z_CMD = 2; // Commanded table height z
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
 * @param z_ball Initial ball vertical position (m)
 * @param z_table Initial table vertical position (m)
 * @return Initialized state vector with zero velocity and tilt
 */
inline StateVector make_initial_state(
    double x = 0.0, double y = 0.0,
    double z_ball = 0.0, double z_table = 0.0
) {
    StateVector state = StateVector::Zero();
    state(state_index::X) = x;
    state(state_index::Y) = y;
    state(state_index::Z_BALL) = z_ball;
    state(state_index::Z_TABLE) = z_table;
    return state;
}

/**
 * @brief Create control vector from tilt angles
 * @param varphi_x Table tilt angle X (rad)
 * @param theta_y Table tilt angle Y (rad)
 * @return Control vector
 */
inline ControlVector make_control(double varphi_x, double theta_y, double table_z = 0.0) {
    ControlVector control;
    control(control_index::VARPHI_X_CMD) = varphi_x;
    control(control_index::THETA_Y_CMD) = theta_y;
    control(control_index::TABLE_Z_CMD) = table_z;
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

// ============================================================================
// Kinematics Mode
// ============================================================================

/**
 * @brief Controls how the table pose is driven.
 *
 * Pose  — set (φ, θ, z_t) via GUI; IK computes servo commands.
 * Servo — set α_i per arm via GUI; FK computes table pose.
 */
enum class KinematicsMode {
    Pose,   ///< Pose-mode: IK drives servo angles from (phi, theta, z_t)
    Servo   ///< Servo-mode: FK drives table pose from individual arm angles
};

} // namespace ball_balancer
