#include <ball_balancer/physics/simulator.hpp>
#include <cmath>
#include <algorithm>

/**
 * @file simulator.cpp
 * @brief Implementation of physics simulation for ball balancer
 *
 * Physics derivation (first principles):
 *
 * For a solid sphere rolling without slipping on an inclined plane:
 *
 * 1. Forces on ball:
 *    - Gravity component along slope: F_g = m*g*sin(θ)
 *    - Friction force: F_f = μ*m*g*cos(θ) (opposes motion)
 *    - Normal force: N = m*g*cos(θ)
 *
 * 2. Torque from friction (causes rotation):
 *    τ = F_f * r = I * α
 *
 * 3. No-slip condition:
 *    a = r * α  (linear accel = radius * angular accel)
 *
 * 4. For solid sphere: I = (2/5)*m*r²
 *
 * 5. Solving for acceleration:
 *    F_net = m*a = m*g*sin(θ) - F_f
 *    τ = F_f*r = I*α = I*(a/r)
 *    F_f = I*a/r²
 *
 *    Substituting:
 *    m*a = m*g*sin(θ) - I*a/r²
 *    a*(m + I/r²) = m*g*sin(θ)
 *    a = m*g*sin(θ) / (m + I/r²)
 *    a = g*sin(θ) / (1 + I/(m*r²))
 *
 *    For solid sphere: I/(m*r²) = (2/5)
 *    a = g*sin(θ) / (1 + 2/5) = (5/7)*g*sin(θ)
 *
 * @see research/ode-physical-system-modeling-cpp.md
 */

namespace ball_balancer {

Simulator::Simulator(const SystemParameters& params)
    : params_(params)
    , state_(StateVector::Zero())
    , time_(0.0)
    , current_control_(ControlVector::Zero())
    , rng_(std::random_device{}())
    , noise_dist_(0.0, params.camera_noise_std)
{
    // Initialize derived parameters
    params_.initialize();
}

void Simulator::step(double dt, const ControlVector& control) {
    // Store control for use in ODE function
    current_control_ = control;

    // Clamp control inputs to physical limits
    current_control_(control_index::THETA_X_CMD) = std::clamp(
        current_control_(control_index::THETA_X_CMD),
        -params_.max_tilt_angle,
        params_.max_tilt_angle
    );
    current_control_(control_index::THETA_Y_CMD) = std::clamp(
        current_control_(control_index::THETA_Y_CMD),
        -params_.max_tilt_angle,
        params_.max_tilt_angle
    );

    // Integrate using RK4 (4th-order Runge-Kutta)
    // Manual implementation for better control and no Boost dependency
    StateVector k1, k2, k3, k4;
    StateVector temp_state;

    // k1 = f(t, x)
    dynamics(state_, k1, time_);

    // k2 = f(t + dt/2, x + dt*k1/2)
    temp_state = state_ + (dt * 0.5) * k1;
    dynamics(temp_state, k2, time_ + dt * 0.5);

    // k3 = f(t + dt/2, x + dt*k2/2)
    temp_state = state_ + (dt * 0.5) * k2;
    dynamics(temp_state, k3, time_ + dt * 0.5);

    // k4 = f(t + dt, x + dt*k3)
    temp_state = state_ + dt * k3;
    dynamics(temp_state, k4, time_ + dt);

    // x(t + dt) = x(t) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
    state_ = state_ + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    // Enforce physical constraints (table boundaries)
    enforce_constraints();

    // Update time
    time_ += dt;
}

void Simulator::reset(const StateVector& initial_state) {
    state_ = initial_state;
    time_ = 0.0;
    current_control_.setZero();

    // Ensure initial state respects constraints
    enforce_constraints();
}

MeasurementVector Simulator::get_measurement() {
    MeasurementVector measurement;

    // Extract ball position from state
    measurement(measurement_index::X_MEAS) = state_(state_index::X);
    measurement(measurement_index::Y_MEAS) = state_(state_index::Y);

    // Add Gaussian noise to simulate camera measurement error
    measurement(measurement_index::X_MEAS) += noise_dist_(rng_);
    measurement(measurement_index::Y_MEAS) += noise_dist_(rng_);

    return measurement;
}

double Simulator::compute_total_energy() const {
    const double m = params_.ball_mass;
    const double r = params_.ball_radius;
    const double g = params_.gravity;
    const double I = params_.ball_inertia;

    // Extract state components
    const double x = state_(state_index::X);
    const double y = state_(state_index::Y);
    const double vx = state_(state_index::VX);
    const double vy = state_(state_index::VY);
    const double theta_x = state_(state_index::THETA_X);
    const double theta_y = state_(state_index::THETA_Y);

    // Velocity magnitude
    const double v = std::sqrt(vx*vx + vy*vy);

    // Angular velocity (no-slip condition: ω = v/r)
    const double omega = v / r;

    // Kinetic energy: translational + rotational
    const double KE_trans = 0.5 * m * v * v;
    const double KE_rot = 0.5 * I * omega * omega;

    // Potential energy: height above lowest point
    // Height h = x*sin(theta_x) + y*sin(theta_y) (small angle approximation)
    const double h = x * std::sin(theta_x) + y * std::sin(theta_y) + r;
    const double PE = m * g * h;

    return KE_trans + KE_rot + PE;
}

void Simulator::dynamics(const StateVector& state, StateDerivative& dstate, double /* t */) {
    // Extract state components
    const double x = state(state_index::X);
    const double y = state(state_index::Y);
    (void)x;  // Position not needed for dynamics (only for constraints)
    (void)y;  // Position not needed for dynamics (only for constraints)
    const double vx = state(state_index::VX);
    const double vy = state(state_index::VY);
    const double theta_x = state(state_index::THETA_X);
    const double theta_y = state(state_index::THETA_Y);

    // Extract control commands
    const double theta_x_cmd = current_control_(control_index::THETA_X_CMD);
    const double theta_y_cmd = current_control_(control_index::THETA_Y_CMD);

    // Physical constants
    const double g = params_.gravity;
    const double friction = params_.friction_coeff;
    const double tau_servo = params_.servo_time_constant;

    // Ball rolling acceleration (from first principles derivation)
    // For solid sphere: a = (5/7) * g * sin(theta)
    // This factor accounts for rotational inertia (no-slip rolling)
    constexpr double rolling_factor = 5.0 / 7.0;

    // Acceleration due to gravity on tilted surface
    // Rotation around X-axis (theta_x) tilts table in Y-direction -> ball accelerates in Y
    // Rotation around Y-axis (theta_y) tilts table in X-direction -> ball accelerates in X
    // Negative signs: positive tilt angle should cause ball to roll in negative direction (down the slope)
    const double ax_gravity = -rolling_factor * g * std::sin(theta_y);  // theta_y affects X (negated)
    const double ay_gravity = -rolling_factor * g * std::sin(theta_x);  // theta_x affects Y (negated)

    // Friction opposes motion (viscous damping model)
    const double ax_friction = -friction * vx;
    const double ay_friction = -friction * vy;

    // Total acceleration
    const double ax = ax_gravity + ax_friction;
    const double ay = ay_gravity + ay_friction;

    // State derivatives (first-order ODE form)
    dstate(state_index::X) = vx;                              // dx/dt = vx
    dstate(state_index::Y) = vy;                              // dy/dt = vy
    dstate(state_index::Z_BALL) = 0.0;                        // dz_ball/dt = 0 (stub: no vertical dynamics)
    dstate(state_index::VX) = ax;                             // dvx/dt = ax
    dstate(state_index::VY) = ay;                             // dvy/dt = ay
    dstate(state_index::VZ_BALL) = 0.0;                       // dvz_ball/dt = 0 (stub)

    // Servo dynamics (first-order response to commanded angle)
    // θ' = (θ_cmd - θ) / τ  (exponential approach to commanded angle)
    dstate(state_index::THETA_X) = (theta_x_cmd - theta_x) / tau_servo;
    dstate(state_index::THETA_Y) = (theta_y_cmd - theta_y) / tau_servo;

    dstate(state_index::Z_TABLE) = 0.0;                       // dz_table/dt = 0 (stub: no vertical dynamics)
}

void Simulator::enforce_constraints() {
    // Table dimensions (centered at origin)
    const double half_length = params_.table_length / 2.0;
    const double half_width = params_.table_width / 2.0;
    const double bounce_coeff = params_.bounce_coeff;

    // Clamp X position and reverse velocity at boundaries
    if (state_(state_index::X) > half_length) {
        state_(state_index::X) = half_length;
        state_(state_index::VX) = -bounce_coeff * state_(state_index::VX);
    } else if (state_(state_index::X) < -half_length) {
        state_(state_index::X) = -half_length;
        state_(state_index::VX) = -bounce_coeff * state_(state_index::VX);
    }

    // Clamp Y position and reverse velocity at boundaries
    if (state_(state_index::Y) > half_width) {
        state_(state_index::Y) = half_width;
        state_(state_index::VY) = -bounce_coeff * state_(state_index::VY);
    } else if (state_(state_index::Y) < -half_width) {
        state_(state_index::Y) = -half_width;
        state_(state_index::VY) = -bounce_coeff * state_(state_index::VY);
    }
}

} // namespace ball_balancer
