#include <ball_balancer/control/pid_controller.hpp>
#include <algorithm>
#include <cmath>

/**
 * @file pid_controller.cpp
 * @brief Implementation of PID controller
 *
 * Implements PID control with anti-windup and derivative filtering
 * following best practices from control theory research.
 *
 * Key features:
 * - Anti-windup: Prevents integral accumulation during saturation
 * - Derivative on measurement: Avoids derivative kick on setpoint changes
 * - Low-pass filtering: Reduces derivative noise amplification
 * - Back-calculation: Unwinding integral when output saturates
 *
 * @see research/control-theory-cpp-implementation-best-practices.md
 */

namespace ball_balancer {

// ============================================================================
// PIDAxis Implementation
// ============================================================================

PIDAxis::PIDAxis(
    const PIDGains& gains,
    double sample_time,
    double output_min,
    double output_max
)
    : gains_(gains)
    , integral_(0.0)
    , prev_measurement_(0.0)
    , derivative_filtered_(0.0)
    , dt_(sample_time)
    , output_min_(output_min)
    , output_max_(output_max)
    , derivative_filter_coeff_(0.1)  // Default: moderate filtering
    , first_call_(true)
{
    // Integral limits for anti-windup
    // Limit integral contribution to output range
    if (std::abs(gains_.Ki) > 1e-10) {
        integral_min_ = output_min_ / gains_.Ki;
        integral_max_ = output_max_ / gains_.Ki;
    } else {
        // If Ki is zero, integral limits don't matter
        integral_min_ = -1e6;
        integral_max_ = 1e6;
    }
}

double PIDAxis::compute(double setpoint, double measurement) {
    // Error
    double error = setpoint - measurement;

    // ========================================================================
    // Proportional Term
    // ========================================================================
    double P = gains_.Kp * error;

    // ========================================================================
    // Integral Term with Anti-Windup
    // ========================================================================
    // Accumulate error (skip on first call after reset)
    if (!first_call_) {
        integral_ += error * dt_;

        // Clamp integral to prevent wind-up
        integral_ = std::clamp(integral_, integral_min_, integral_max_);
    }

    double I = gains_.Ki * integral_;

    // ========================================================================
    // Derivative Term (on measurement, not error)
    // ========================================================================
    // Derivative on measurement to avoid derivative kick when setpoint changes
    // D = -Kd * d(measurement)/dt
    double derivative = 0.0;
    if (!first_call_) {
        derivative = -(measurement - prev_measurement_) / dt_;

        // Low-pass filter to reduce noise amplification
        // First-order filter: y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
        derivative_filtered_ = derivative_filter_coeff_ * derivative +
                              (1.0 - derivative_filter_coeff_) * derivative_filtered_;
    }

    double D = gains_.Kd * derivative_filtered_;

    // ========================================================================
    // Compute Output
    // ========================================================================
    double output_unsat = P + I + D;

    // Clamp output
    double output = std::clamp(output_unsat, output_min_, output_max_);

    // ========================================================================
    // Anti-Windup: Back-Calculation
    // ========================================================================
    // If output is saturated, prevent integral from growing further
    // Undo the integration step if we're saturated (skip on first call)
    if (!first_call_ && output != output_unsat) {
        // We saturated - back out the integration
        integral_ -= error * dt_;
    }

    // ========================================================================
    // Update State
    // ========================================================================
    prev_measurement_ = measurement;
    first_call_ = false;

    return output;
}

void PIDAxis::reset() {
    integral_ = 0.0;
    prev_measurement_ = 0.0;
    derivative_filtered_ = 0.0;
    first_call_ = true;
}

void PIDAxis::set_gains(const PIDGains& gains) {
    gains_ = gains;

    // Recompute integral limits
    if (std::abs(gains_.Ki) > 1e-10) {
        integral_min_ = output_min_ / gains_.Ki;
        integral_max_ = output_max_ / gains_.Ki;
    } else {
        integral_min_ = -1e6;
        integral_max_ = 1e6;
    }
}

void PIDAxis::set_output_limits(double min, double max) {
    output_min_ = min;
    output_max_ = max;

    // Recompute integral limits
    if (std::abs(gains_.Ki) > 1e-10) {
        integral_min_ = output_min_ / gains_.Ki;
        integral_max_ = output_max_ / gains_.Ki;
    }
}

void PIDAxis::set_derivative_filter(double alpha) {
    derivative_filter_coeff_ = std::clamp(alpha, 0.0, 1.0);
}

// ============================================================================
// PIDController Implementation
// ============================================================================

PIDController::PIDController(const SystemParameters& params)
    : pid_x_(PIDGains{}, params.control_dt, -params.max_tilt_angle, params.max_tilt_angle)
    , pid_y_(PIDGains{}, params.control_dt, -params.max_tilt_angle, params.max_tilt_angle)
{
    // Initialize with default gains
    // User should tune these based on system response
    PIDGains default_gains;
    default_gains.Kp = 1.0;   // Moderate proportional gain
    default_gains.Ki = 0.5;   // Small integral for steady-state error
    default_gains.Kd = 0.2;   // Small derivative for damping

    pid_x_.set_gains(default_gains);
    pid_y_.set_gains(default_gains);
}

ControlVector PIDController::compute(
    const Eigen::Vector2d& setpoint,
    const StateVector& state
) {
    // Extract ball position from state
    double x = state(state_index::X);
    double y = state(state_index::Y);

    // Compute control for each axis independently
    double theta_x_cmd = pid_x_.compute(setpoint.x(), x);
    double theta_y_cmd = pid_y_.compute(setpoint.y(), y);

    // Return control vector
    ControlVector control;
    control(control_index::THETA_X_CMD) = theta_x_cmd;
    control(control_index::THETA_Y_CMD) = theta_y_cmd;

    return control;
}

void PIDController::reset() {
    pid_x_.reset();
    pid_y_.reset();
}

void PIDController::set_x_gains(const PIDGains& gains) {
    pid_x_.set_gains(gains);
}

void PIDController::set_y_gains(const PIDGains& gains) {
    pid_y_.set_gains(gains);
}

} // namespace ball_balancer
