#pragma once

#include <ball_balancer/core/types.hpp>

/**
 * @file state_estimator.hpp
 * @brief Kalman filter for state estimation
 *
 * Estimates full 6D state from noisy 2D position measurements.
 *
 * System model:
 * - State: x = [x, y, vx, vy, theta_x, theta_y] (6D)
 * - Measurement: z = [x, y] (2D position from camera)
 * - Process model: x(k+1) = A*x(k) + B*u(k) + w (process noise)
 * - Measurement model: z(k) = C*x(k) + v (measurement noise)
 *
 * Kalman filter provides:
 * - Velocity estimates (not directly measured)
 * - Noise reduction (filters sensor noise)
 * - Prediction (estimates state between measurements)
 *
 * Following best practices from research/control-theory-cpp-implementation-best-practices.md:
 * - Prediction step: propagate state and covariance
 * - Update step: incorporate measurement with optimal Kalman gain
 * - Tuning: Q (process noise), R (measurement noise)
 *
 * @see research/control-theory-cpp-implementation-best-practices.md
 */

namespace ball_balancer {

/**
 * @brief Kalman filter tuning parameters
 * Note: Using float for ImGui compatibility
 */
struct KalmanTuning {
    float process_noise_position{0.001f};    ///< Position process noise std dev (m)
    float process_noise_velocity{0.01f};     ///< Velocity process noise std dev (m/s)
    float process_noise_angle{0.0001f};      ///< Angle process noise std dev (rad)
    float measurement_noise_position{0.005f}; ///< Position measurement noise std dev (m)

    void initialize() {
        process_noise_position = 0.001f;
        process_noise_velocity = 0.01f;
        process_noise_angle = 0.0001f;
        measurement_noise_position = 0.005f;
    }
};

/**
 * @brief Kalman filter for ball balancer state estimation
 *
 * Discrete-time Kalman filter:
 *
 * Prediction:
 * - x_hat(k|k-1) = A*x_hat(k-1|k-1) + B*u(k-1)
 * - P(k|k-1) = A*P(k-1|k-1)*A' + Q
 *
 * Update:
 * - y_tilde = z(k) - C*x_hat(k|k-1)  (innovation)
 * - S = C*P(k|k-1)*C' + R             (innovation covariance)
 * - K = P(k|k-1)*C'*S^-1              (Kalman gain)
 * - x_hat(k|k) = x_hat(k|k-1) + K*y_tilde
 * - P(k|k) = (I - K*C)*P(k|k-1)
 */
class StateEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct state estimator
     * @param params System parameters
     * @param tuning Kalman filter tuning parameters
     */
    StateEstimator(
        const SystemParameters& params,
        const KalmanTuning& tuning = KalmanTuning{}
    );

    /**
     * @brief Prediction step
     * @param control Control input (table tilt commands)
     *
     * Propagates state estimate forward using system model.
     * Call this at control loop rate before measurement arrives.
     */
    void predict(const ControlVector& control);

    /**
     * @brief Update step (correction)
     * @param measurement Position measurement [x, y]
     *
     * Corrects state estimate using measurement.
     * Call this when new measurement is available from camera.
     */
    void update(const MeasurementVector& measurement);

    /**
     * @brief Get estimated ball state (converted from internal x_hat_).
     */
    BallState get_ball_state() const { return toBallState(x_hat_); }

    /**
     * @brief Get estimated table state (converted from internal x_hat_).
     * Note: only phi, theta, z_t are populated; rates are zero.
     */
    TableState get_table_state() const { return toTableState(x_hat_); }

    /**
     * @brief Get estimation error covariance
     * @return Covariance matrix P (9x9)
     */
    const SystemMatrix& get_covariance() const { return P_; }

    /**
     * @brief Reset estimator to initial state
     * @param ball   Initial ball state estimate
     * @param table  Initial table state estimate
     */
    void reset(const BallState& ball, const TableState& table);

    /**
     * @brief Update tuning parameters
     * @param tuning New tuning parameters
     *
     * Updates Q and R matrices from tuning parameters.
     */
    void set_tuning(const KalmanTuning& tuning);

    /**
     * @brief Get current tuning parameters
     */
    const KalmanTuning& get_tuning() const { return tuning_; }

private:
    /**
     * @brief Compute system matrices for linearized model
     *
     * Linearizes nonlinear dynamics around current operating point.
     * Called during initialization and can be updated if needed.
     */
    void compute_system_matrices();

    /**
     * @brief Initialize noise covariance matrices
     *
     * Constructs Q and R from tuning parameters.
     */
    void initialize_noise_matrices();

    // System parameters
    SystemParameters params_;
    KalmanTuning tuning_;

    // State estimate and covariance
    StateVector x_hat_;  ///< State estimate (6D)
    SystemMatrix P_;     ///< Error covariance (6x6)

    // System matrices (linearized model)
    SystemMatrix A_;         ///< State transition matrix (6x6)
    ControlMatrix B_;        ///< Control input matrix (6x2)
    MeasurementMatrix C_;    ///< Measurement matrix (2x6)

    // Noise covariance matrices
    SystemMatrix Q_;              ///< Process noise covariance (6x6)
    Eigen::Matrix2d R_;           ///< Measurement noise covariance (2x2)

    // Sample time
    double dt_;
};

} // namespace ball_balancer
