#include <ball_balancer/control/state_estimator.hpp>
#include <ball_balancer/math/matrix_utils.hpp>
#include <cmath>

/**
 * @file state_estimator.cpp
 * @brief Implementation of Kalman filter state estimator
 *
 * Implements discrete-time Kalman filter for ball balancer.
 *
 * Algorithm:
 * 1. Prediction: Use system model to predict next state
 * 2. Update: Correct prediction using measurement
 *
 * The filter estimates velocities (vx, vy) which are not directly measured,
 * and filters noise from position measurements.
 *
 * @see research/control-theory-cpp-implementation-best-practices.md
 */

namespace ball_balancer {

StateEstimator::StateEstimator(
    const SystemParameters& params,
    const KalmanTuning& tuning
)
    : params_(params)
    , tuning_(tuning)
    , x_hat_(StateVector::Zero())
    , P_(SystemMatrix::Identity() * 0.1)  // Initial covariance
    , A_(SystemMatrix::Zero())
    , B_(ControlMatrix::Zero())
    , C_(MeasurementMatrix::Zero())
    , Q_(SystemMatrix::Zero())
    , R_(Eigen::Matrix2d::Zero())
    , dt_(params.control_dt)
{
    // Compute system matrices (linearized model)
    compute_system_matrices();

    // Initialize noise covariance matrices
    initialize_noise_matrices();
}

void StateEstimator::compute_system_matrices() {
    // ========================================================================
    // State-Space Model (Linearized)
    // ========================================================================
    //
    // State: x = [x, y, vx, vy, theta_x, theta_y]
    // Control: u = [theta_x_cmd, theta_y_cmd]
    // Measurement: z = [x, y]
    //
    // Continuous-time linearized dynamics:
    // dx/dt = vx
    // dy/dt = vy
    // dvx/dt = (5/7) * g * theta_y    (ball acceleration due to table tilt)
    // dvy/dt = (5/7) * g * theta_x
    // dtheta_x/dt = (theta_x_cmd - theta_x) / tau_servo
    // dtheta_y/dt = (theta_y_cmd - theta_y) / tau_servo
    //
    // State-space form: dx/dt = A*x + B*u
    // ========================================================================

    const double g = params_.gravity;
    const double rolling_factor = 5.0 / 7.0;  // Solid sphere
    const double tau_servo = 0.05;  // Servo time constant (50ms)

    // A matrix (continuous-time)
    Eigen::Matrix<double, 6, 6> Ac = Eigen::Matrix<double, 6, 6>::Zero();

    // Position derivatives
    Ac(state_index::X, state_index::VX) = 1.0;
    Ac(state_index::Y, state_index::VY) = 1.0;

    // Velocity derivatives (linearized around small angles)
    // dvx/dt = (5/7) * g * sin(theta_y) ≈ (5/7) * g * theta_y
    Ac(state_index::VX, state_index::THETA_Y) = rolling_factor * g;
    Ac(state_index::VY, state_index::THETA_X) = rolling_factor * g;

    // Angle derivatives (first-order servo dynamics)
    Ac(state_index::THETA_X, state_index::THETA_X) = -1.0 / tau_servo;
    Ac(state_index::THETA_Y, state_index::THETA_Y) = -1.0 / tau_servo;

    // B matrix (continuous-time)
    Eigen::Matrix<double, 6, 2> Bc = Eigen::Matrix<double, 6, 2>::Zero();
    Bc(state_index::THETA_X, control_index::THETA_X_CMD) = 1.0 / tau_servo;
    Bc(state_index::THETA_Y, control_index::THETA_Y_CMD) = 1.0 / tau_servo;

    // ========================================================================
    // Discretization (Forward Euler - simple but sufficient for small dt)
    // ========================================================================
    // For more accuracy, could use Tustin or matrix exponential
    // But for dt = 0.01s and slow dynamics, Forward Euler is adequate
    //
    // Ad = I + Ac * dt
    // Bd = Bc * dt
    // ========================================================================

    A_ = SystemMatrix::Identity() + Ac * dt_;
    B_ = Bc * dt_;

    // ========================================================================
    // Measurement matrix C
    // ========================================================================
    // We measure position only: z = [x, y]
    // z = C * x where C extracts position from state

    C_ = MeasurementMatrix::Zero();
    C_(0, state_index::X) = 1.0;  // Measure x
    C_(1, state_index::Y) = 1.0;  // Measure y
}

void StateEstimator::initialize_noise_matrices() {
    // ========================================================================
    // Process Noise Covariance Q
    // ========================================================================
    // Represents uncertainty in system model
    // Diagonal matrix with variance for each state
    //
    // Larger Q => trust model less, trust measurements more
    // Smaller Q => trust model more, trust measurements less

    Q_ = SystemMatrix::Zero();

    // Position uncertainty (due to unmodeled dynamics)
    Q_(state_index::X, state_index::X) =
        tuning_.process_noise_position * tuning_.process_noise_position;
    Q_(state_index::Y, state_index::Y) =
        tuning_.process_noise_position * tuning_.process_noise_position;

    // Velocity uncertainty (larger due to integration)
    Q_(state_index::VX, state_index::VX) =
        tuning_.process_noise_velocity * tuning_.process_noise_velocity;
    Q_(state_index::VY, state_index::VY) =
        tuning_.process_noise_velocity * tuning_.process_noise_velocity;

    // Angle uncertainty (servo model uncertainty)
    Q_(state_index::THETA_X, state_index::THETA_X) =
        tuning_.process_noise_angle * tuning_.process_noise_angle;
    Q_(state_index::THETA_Y, state_index::THETA_Y) =
        tuning_.process_noise_angle * tuning_.process_noise_angle;

    // ========================================================================
    // Measurement Noise Covariance R
    // ========================================================================
    // Represents sensor noise (camera)
    //
    // Larger R => trust measurements less, trust model more
    // Smaller R => trust measurements more, trust model less

    R_ = Eigen::Matrix2d::Zero();
    R_(0, 0) = tuning_.measurement_noise_position * tuning_.measurement_noise_position;
    R_(1, 1) = tuning_.measurement_noise_position * tuning_.measurement_noise_position;
}

void StateEstimator::predict(const ControlVector& control) {
    // ========================================================================
    // Kalman Filter Prediction Step
    // ========================================================================
    //
    // State prediction: x_hat(k|k-1) = A*x_hat(k-1|k-1) + B*u(k-1)
    // Covariance prediction: P(k|k-1) = A*P(k-1|k-1)*A' + Q
    //
    // IMPORTANT: Following Eigen best practices from research:
    // - Never use auto with Eigen expressions
    // - Use .eval() or temporary variables to avoid aliasing
    // ========================================================================

    // State prediction
    // IMPORTANT: Must use .eval() because x_hat_ appears on both sides
    x_hat_ = (A_ * x_hat_ + B_ * control).eval();

    // Covariance prediction
    // IMPORTANT: Must use .eval() because P_ appears on both sides
    // P = A*P*A' + Q
    P_ = (A_ * P_ * A_.transpose() + Q_).eval();
}

void StateEstimator::update(const MeasurementVector& measurement) {
    // ========================================================================
    // Kalman Filter Update Step (Correction)
    // ========================================================================
    //
    // Innovation: y_tilde = z - C*x_hat(k|k-1)
    // Innovation covariance: S = C*P*C' + R
    // Kalman gain: K = P*C'*S^-1
    // State update: x_hat(k|k) = x_hat(k|k-1) + K*y_tilde
    // Covariance update: P(k|k) = (I - K*C)*P
    //
    // IMPORTANT: Never compute matrix inverse directly
    // Use decomposition (LU, QR, etc.) as per Eigen best practices
    // ========================================================================

    // Innovation (prediction error)
    Eigen::Vector2d y_tilde = measurement - C_ * x_hat_;

    // Innovation covariance
    // S = C*P*C' + R
    Eigen::Matrix2d S = C_ * P_ * C_.transpose() + R_;

    // Kalman gain using LU decomposition (not inverse!)
    // K = P*C' * S^-1
    // IMPORTANT: Use .solve() instead of .inverse()
    Eigen::Matrix<double, 6, 2> K_temp = P_ * C_.transpose();
    Eigen::Matrix<double, 6, 2> K = K_temp * S.lu().solve(Eigen::Matrix2d::Identity());

    // State update
    // IMPORTANT: Use .eval() because x_hat_ appears on both sides
    x_hat_ = (x_hat_ + K * y_tilde).eval();

    // Covariance update (Joseph form for numerical stability)
    // P = (I - K*C)*P*(I - K*C)' + K*R*K'
    // Simplified form (less numerically stable but faster):
    // P = (I - K*C)*P
    //
    // Using simplified form for real-time performance
    Eigen::Matrix<double, 6, 6> I_KC = SystemMatrix::Identity() - K * C_;
    P_ = (I_KC * P_).eval();

    // Ensure P remains symmetric (numerical errors can break symmetry)
    P_ = ((P_ + P_.transpose()) * 0.5).eval();
}

void StateEstimator::reset(const StateVector& initial_state) {
    x_hat_ = initial_state;
    P_ = SystemMatrix::Identity() * 0.1;  // Reset covariance
}

void StateEstimator::set_tuning(const KalmanTuning& tuning) {
    tuning_ = tuning;
    initialize_noise_matrices();
}

} // namespace ball_balancer
