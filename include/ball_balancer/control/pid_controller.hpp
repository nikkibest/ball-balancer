#pragma once

#include <ball_balancer/core/types.hpp>
#include <limits>

/**
 * @file pid_controller.hpp
 * @brief PID controller for ball position control
 *
 * Implements dual-axis PID control with:
 * - Anti-windup clamping
 * - Derivative filtering (low-pass)
 * - Configurable gains
 * - Derivative-on-measurement to avoid derivative kick
 *
 * Following best practices from research/control-theory-cpp-implementation-best-practices.md:
 * - Anti-windup prevents integral saturation
 * - Derivative filtering reduces noise amplification
 * - Proper discretization for digital implementation
 *
 * @see research/control-theory-cpp-implementation-best-practices.md
 */

namespace ball_balancer {

/**
 * @brief PID controller gains structure
 * Note: Using float for ImGui compatibility
 */
struct PIDGains {
    float Kp{1.0f};  ///< Proportional gain
    float Ki{0.0f};  ///< Integral gain
    float Kd{0.0f};  ///< Derivative gain

    void initialize() {
        Kp = 1.0f;
        Ki = 0.0f;
        Kd = 0.0f;
    }
};

/**
 * @brief Single-axis PID controller
 *
 * Implements PID control for one axis with anti-windup and derivative filtering.
 * Use two instances for dual-axis (X, Y) control.
 */
class PIDAxis {
public:
    /**
     * @brief Construct PID controller for one axis
     * @param gains PID gains (Kp, Ki, Kd)
     * @param sample_time Control loop sample time (seconds)
     * @param output_min Minimum output (default: -inf)
     * @param output_max Maximum output (default: +inf)
     */
    PIDAxis(
        const PIDGains& gains,
        double sample_time,
        double output_min = -std::numeric_limits<double>::infinity(),
        double output_max = std::numeric_limits<double>::infinity()
    );

    /**
     * @brief Compute control output
     * @param setpoint Desired position
     * @param measurement Current position
     * @return Control output (table tilt angle command)
     *
     * Algorithm:
     * - P term: proportional to error
     * - I term: accumulated error with anti-windup
     * - D term: derivative on measurement (not error) to avoid derivative kick
     * - Derivative filtered with low-pass filter
     */
    double compute(double setpoint, double measurement);

    /**
     * @brief Reset controller state
     *
     * Clears integral accumulator and derivative filter state.
     * Call when switching modes or resetting simulation.
     */
    void reset();

    /**
     * @brief Update PID gains
     * @param gains New PID gains
     */
    void set_gains(const PIDGains& gains);

    /**
     * @brief Get current PID gains
     * @return Current gains
     */
    const PIDGains& get_gains() const { return gains_; }

    /**
     * @brief Set output limits
     * @param min Minimum output
     * @param max Maximum output
     */
    void set_output_limits(double min, double max);

    /**
     * @brief Set derivative filter coefficient
     * @param alpha Filter coefficient (0.0 = no filter, 1.0 = heavy filter)
     *
     * Low-pass filter: d_filtered = alpha * d_new + (1 - alpha) * d_filtered_old
     * Typical value: 0.1 for moderate filtering
     */
    void set_derivative_filter(double alpha);

private:
    // PID gains
    PIDGains gains_;

    // State variables
    double integral_;           ///< Accumulated error
    double prev_measurement_;   ///< Previous measurement for derivative
    double derivative_filtered_; ///< Filtered derivative

    // Configuration
    double dt_;                 ///< Sample time (seconds)
    double output_min_;         ///< Minimum output
    double output_max_;         ///< Maximum output
    double integral_min_;       ///< Minimum integral (for anti-windup)
    double integral_max_;       ///< Maximum integral (for anti-windup)
    double derivative_filter_coeff_; ///< Derivative low-pass filter coefficient

    bool first_call_;           ///< Flag for first compute() call
};

/**
 * @brief Dual-axis PID controller for ball position
 *
 * Controls ball position on X and Y axes independently.
 * Each axis has its own PID controller with separate gains.
 *
 * Control law:
 * - Measure ball position (x, y)
 * - Compute errors: e_x = setpoint_x - x, e_y = setpoint_y - y
 * - Compute table tilt commands: theta_x = PID_x(e_x), theta_y = PID_y(e_y)
 */
class PIDController {
public:
    /**
     * @brief Construct dual-axis PID controller
     * @param params System parameters (for output limits)
     */
    explicit PIDController(const SystemParameters& params);

    /**
     * @brief Compute control output
     * @param setpoint Desired ball position (x, y)
     * @param state Current system state (contains ball position)
     * @return Control vector [theta_x_cmd, theta_y_cmd]
     */
    ControlVector compute(
        const Eigen::Vector2d& setpoint,
        const StateVector& state
    );

    /**
     * @brief Reset both axes
     */
    void reset();

    /**
     * @brief Set gains for X axis
     * @param gains New gains for X controller
     */
    void set_x_gains(const PIDGains& gains);

    /**
     * @brief Set gains for Y axis
     * @param gains New gains for Y controller
     */
    void set_y_gains(const PIDGains& gains);

    /**
     * @brief Get X axis gains
     */
    const PIDGains& get_x_gains() const { return pid_x_.get_gains(); }

    /**
     * @brief Get Y axis gains
     */
    const PIDGains& get_y_gains() const { return pid_y_.get_gains(); }

private:
    PIDAxis pid_x_;  ///< PID controller for X axis
    PIDAxis pid_y_;  ///< PID controller for Y axis
};

} // namespace ball_balancer
