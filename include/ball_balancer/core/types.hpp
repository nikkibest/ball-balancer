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
// TableState — Table kinematics snapshot passed to BallDynamics
// ============================================================================

/**
 * @brief Snapshot of table kinematics passed to BallDynamics each step.
 *
 * The Simulator builds this from the current state vector and servo dynamics
 * so that BallDynamics remains fully decoupled from Simulator internals.
 *
 * Fields:
 *   phi, theta, z_t         — configuration  (rad, rad, m)
 *   phi_dot, theta_dot, z_t_dot   — velocities     (rad/s, rad/s, m/s)
 *   phi_ddot, theta_ddot, z_t_ddot — accelerations  (rad/s², rad/s², m/s²)
 */
struct TableState {
    // Configuration
    double phi{0.0};        ///< Roll angle about X-axis (rad) — THETA_X
    double theta{0.0};      ///< Pitch angle about Y-axis (rad) — THETA_Y
    double z_t{0.0};        ///< Table centre height (m) — Z_TABLE

    // First derivatives
    double phi_dot{0.0};    ///< Roll rate (rad/s)
    double theta_dot{0.0};  ///< Pitch rate (rad/s)
    double z_t_dot{0.0};    ///< Table vertical velocity (m/s)

    // Second derivatives
    double phi_ddot{0.0};   ///< Roll angular acceleration (rad/s²)
    double theta_ddot{0.0}; ///< Pitch angular acceleration (rad/s²)
    double z_t_ddot{0.0};   ///< Table vertical acceleration (m/s²)
};

// ============================================================================
// BallState — Ball 
// ============================================================================

/**
 * @brief Snapshot of ball dynamics states.
 *
 * The Simulator ball state
 *
 * Fields:
 *   x, y, z_ball      — configuration  (m, m, m)
 *   vx, vy, vz_ball   — velocities     (m/s, m/s, m/s)
 *   ax, ay, az        — accelerations  (m/s², m/s², m/s²)
 */
struct BallState {
    // Configuration
    double x{0.0};          ///< Ball horizontal position (m)
    double y{0.0};          ///< Ball horizontal position (m)
    double z_ball{0.0};     ///< Ball vertical position (m)

    // First derivatives
    double vx{0.0};         ///< Ball horizontal velocity (m/s)
    double vy{0.0};         ///< Ball horizontal velocity (m/s)
    double vz_ball{0.0};    ///< Ball vertical velocity (m/s)

    // Second derivatives
    double ax{0.0};         ///< Ball horizontal acceleration (m/s²)
    double ay{0.0};         ///< Ball horizontal acceleration (m/s²)
    double az_ball{0.0};         ///< Ball vertical acceleration (m/s²)
};

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
    double table_radius{0.07};         // Table disc radius (m) — kept in sync with arm_Rt by initialize()
    double max_tilt_angle{0.174};      // Max tilt angle (rad) = 10 degrees
    double min_table_height{0.01};     // Minimum table height (m)
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
    double arm_L1{0.10};              // Lower link length: servo pivot to elbow (m)
    double arm_L2{0.10};              // Upper link length: elbow to table attachment (m)
    double arm_Rg{0.15};              // Ground mounting radius: centre to servo pivot (m)
    double arm_Rt{0.15};              // Table mounting radius: centre to table attachment (m)
    double arm_z_nominal{0.08};       // Nominal table height for FK warm-start (m)

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
        // Table disc radius matches the arm table-attachment radius
        table_radius = arm_Rt;
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
 * @brief Pack BallState and TableState into a StateVector (for Kalman filter internals).
 * Velocities and accelerations from BallState; config from TableState.
 * TableState accelerations (phi_ddot, theta_ddot, z_t_ddot) are not stored in StateVector.
 */
inline StateVector toStateVector(const BallState& ball, const TableState& table) {
    StateVector s = StateVector::Zero();
    s(state_index::X)        = ball.x;
    s(state_index::Y)        = ball.y;
    s(state_index::Z_BALL)   = ball.z_ball;
    s(state_index::VX)       = ball.vx;
    s(state_index::VY)       = ball.vy;
    s(state_index::VZ_BALL)  = ball.vz_ball;
    s(state_index::VARPHI_X) = table.phi;
    s(state_index::THETA_Y)  = table.theta;
    s(state_index::Z_TABLE)  = table.z_t;
    return s;
}

/**
 * @brief Extract BallState from a StateVector.
 * Accelerations are set to zero (not stored in StateVector).
 */
inline BallState toBallState(const StateVector& s) {
    BallState ball;
    ball.x       = s(state_index::X);
    ball.y       = s(state_index::Y);
    ball.z_ball  = s(state_index::Z_BALL);
    ball.vx      = s(state_index::VX);
    ball.vy      = s(state_index::VY);
    ball.vz_ball = s(state_index::VZ_BALL);
    return ball;
}

/**
 * @brief Extract TableState from a StateVector.
 * Rates and accelerations are set to zero (not stored in StateVector).
 */
inline TableState toTableState(const StateVector& s) {
    TableState table;
    table.phi   = s(state_index::VARPHI_X);
    table.theta = s(state_index::THETA_Y);
    table.z_t   = s(state_index::Z_TABLE);
    return table;
}

/**
 * @brief Create an initial BallState.
 */
inline BallState make_initial_ball_state(
    double x = 0.0, double y = 0.0, double z_ball = 0.0
) {
    BallState ball;
    ball.x      = x;
    ball.y      = y;
    ball.z_ball = z_ball;
    return ball;
}

/**
 * @brief Create an initial TableState.
 */
inline TableState make_initial_table_state(
    double phi = 0.0, double theta = 0.0, double z_t = 0.0
) {
    TableState table;
    table.phi   = phi;
    table.theta = theta;
    table.z_t   = z_t;
    return table;
}

/**
 * @brief Create initial state vector (kept for Kalman filter internal use).
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
