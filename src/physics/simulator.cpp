#include <ball_balancer/physics/simulator.hpp>
#include <cmath>
#include <algorithm>

/**
 * @file simulator.cpp
 * @brief Physics simulation of ball on tilting table.
 *
 * Primary state is now BallState + TableState (no flat StateVector).
 *
 * Responsibilities:
 *   - RK4 integration of ball dynamics
 *   - First-order servo lag for table angles (Pose mode)
 *   - Post-step constraint enforcement (contact snap, bounce, boundary clamp)
 *   - Measurement noise generation
 *
 * Axis convention (unchanged from project-wide convention):
 *   VARPHI_X (phi)  = roll  around X → drives ball in Y
 *   THETA_Y (theta) = pitch around Y → drives ball in X
 *
 * @see include/ball_balancer/physics/ball_dynamics.hpp for ball EOM derivation
 */

namespace ball_balancer {

Simulator::Simulator(const SystemParameters& params)
    : params_(params)
    , ball_dynamics_(params)
    , ball_state_{}
    , table_state_{}
    , time_(0.0)
    , current_control_(ControlVector::Zero())
    , rng_(std::random_device{}())
    , noise_dist_(0.0, params.camera_noise_std)
{
    params_.initialize();

    // Place ball resting on flat table at default position
    table_state_.z_t  = params_.arm_z_nominal;
    ball_state_.z_ball = params_.arm_z_nominal + params_.ball_radius;
}

void Simulator::step(double dt, const ControlVector& control) {
    current_control_ = control;

    // Update table rates from control command (first-order servo lag)
    update_table_rates();

    // RK4 integration of ball states only.
    // Table configuration (phi, theta, z_t) is integrated separately via
    // Euler in update_table_rates; the rate fields drive the RK4 derivatives.
    BallState k1{}, k2{}, k3{}, k4{}, temp{};

    ball_dynamics_step(ball_state_, table_state_, k1);

    temp.x      = ball_state_.x      + (dt * 0.5) * k1.x;
    temp.y      = ball_state_.y      + (dt * 0.5) * k1.y;
    temp.z_ball = ball_state_.z_ball + (dt * 0.5) * k1.z_ball;
    temp.vx     = ball_state_.vx     + (dt * 0.5) * k1.vx;
    temp.vy     = ball_state_.vy     + (dt * 0.5) * k1.vy;
    temp.vz_ball= ball_state_.vz_ball+ (dt * 0.5) * k1.vz_ball;
    ball_dynamics_step(temp, table_state_, k2);

    temp.x      = ball_state_.x      + (dt * 0.5) * k2.x;
    temp.y      = ball_state_.y      + (dt * 0.5) * k2.y;
    temp.z_ball = ball_state_.z_ball + (dt * 0.5) * k2.z_ball;
    temp.vx     = ball_state_.vx     + (dt * 0.5) * k2.vx;
    temp.vy     = ball_state_.vy     + (dt * 0.5) * k2.vy;
    temp.vz_ball= ball_state_.vz_ball+ (dt * 0.5) * k2.vz_ball;
    ball_dynamics_step(temp, table_state_, k3);

    temp.x      = ball_state_.x      + dt * k3.x;
    temp.y      = ball_state_.y      + dt * k3.y;
    temp.z_ball = ball_state_.z_ball + dt * k3.z_ball;
    temp.vx     = ball_state_.vx     + dt * k3.vx;
    temp.vy     = ball_state_.vy     + dt * k3.vy;
    temp.vz_ball= ball_state_.vz_ball+ dt * k3.vz_ball;
    ball_dynamics_step(temp, table_state_, k4);

    ball_state_.x       += (dt / 6.0) * (k1.x      + 2.0*k2.x      + 2.0*k3.x      + k4.x);
    ball_state_.y       += (dt / 6.0) * (k1.y      + 2.0*k2.y      + 2.0*k3.y      + k4.y);
    ball_state_.z_ball  += (dt / 6.0) * (k1.z_ball + 2.0*k2.z_ball + 2.0*k3.z_ball + k4.z_ball);
    ball_state_.vx      += (dt / 6.0) * (k1.vx     + 2.0*k2.vx     + 2.0*k3.vx     + k4.vx);
    ball_state_.vy      += (dt / 6.0) * (k1.vy     + 2.0*k2.vy     + 2.0*k3.vy     + k4.vy);
    ball_state_.vz_ball += (dt / 6.0) * (k1.vz_ball+ 2.0*k2.vz_ball+ 2.0*k3.vz_ball+ k4.vz_ball);

    // Integrate table configuration with Euler (servo dynamics)
    table_state_.phi   += dt * table_state_.phi_dot;
    table_state_.theta += dt * table_state_.theta_dot;
    table_state_.z_t   += dt * table_state_.z_t_dot;

    // Post-step: enforce contact constraint and boundary conditions
    enforce_constraints();

    time_ += dt;
}

void Simulator::reset(const BallState& ball, const TableState& table) {
    ball_state_  = ball;
    table_state_ = table;
    time_        = 0.0;
    current_control_.setZero();

    // Ensure Z_BALL is at least on the table surface
    const double z_surface = table_state_.z_t + params_.ball_radius;
    if (ball_state_.z_ball < z_surface) {
        ball_state_.z_ball = z_surface;
    }

    enforce_constraints();
}

MeasurementVector Simulator::get_measurement() {
    MeasurementVector measurement;
    measurement(measurement_index::X_MEAS) = ball_state_.x + noise_dist_(rng_);
    measurement(measurement_index::Y_MEAS) = ball_state_.y + noise_dist_(rng_);
    return measurement;
}

double Simulator::compute_total_energy() const {
    const double m  = params_.ball_mass;
    const double r  = params_.ball_radius;
    const double g  = params_.gravity;
    const double I  = params_.ball_inertia;

    // Translational kinetic energy (full 3D)
    const double v2       = ball_state_.vx*ball_state_.vx
                          + ball_state_.vy*ball_state_.vy
                          + ball_state_.vz_ball*ball_state_.vz_ball;
    const double KE_trans = 0.5 * m * v2;

    // Rotational kinetic energy (no-slip rolling, horizontal speed)
    const double vh    = std::sqrt(ball_state_.vx*ball_state_.vx + ball_state_.vy*ball_state_.vy);
    const double omega = vh / r;
    const double KE_rot = 0.5 * I * omega * omega;

    // Gravitational potential energy (full 3D height)
    const double h  = ball_state_.z_ball
                    + ball_state_.x * std::sin(table_state_.theta)
                    + ball_state_.y * std::sin(table_state_.phi);
    const double PE = m * g * h;

    return KE_trans + KE_rot + PE;
}

// ----------------------------------------------------------------------------
// Private helpers
// ----------------------------------------------------------------------------

void Simulator::update_table_rates() {
    const double tau = params_.servo_time_constant;
    table_state_.phi_dot   = (current_control_(control_index::VARPHI_X_CMD) - table_state_.phi)   / tau;
    table_state_.theta_dot = (current_control_(control_index::THETA_Y_CMD)  - table_state_.theta) / tau;
    table_state_.z_t_dot   = (current_control_(control_index::TABLE_Z_CMD)  - table_state_.z_t)   / tau;
    table_state_.phi_ddot   = 0.0;
    table_state_.theta_ddot = 0.0;
    table_state_.z_t_ddot   = 0.0;
}

void Simulator::ball_dynamics_step(
    const BallState& ball, const TableState& table, BallState& d_ball
) {
    double ax{0.0}, ay{0.0}, az{0.0};
    ball_dynamics_.computeAccelerations(ball, table, ax, ay, az);

    // Position derivatives = velocities
    d_ball.x      = ball.vx;
    d_ball.y      = ball.vy;
    d_ball.z_ball = ball.vz_ball;

    // Velocity derivatives = accelerations
    d_ball.vx      = ax;
    d_ball.vy      = ay;
    d_ball.vz_ball = az;
}

void Simulator::enforce_constraints() {
    // --- Contact constraint ---
    // Surface height at current (x, y) under small-angle approximation
    const double z_surface = table_state_.z_t
        + params_.ball_radius
        + ball_state_.x * table_state_.theta
        - ball_state_.y * table_state_.phi;

    if (ball_state_.z_ball < z_surface) {
        // Ball has penetrated the table — snap it to the surface
        ball_state_.z_ball = z_surface;

        // Apply coefficient-of-restitution bounce if approaching.
        ball_dynamics_.applyBounce(ball_state_, table_state_);

        // If after bounce vz_ball is still below z_t_dot, clamp so the ball
        // rides with the rising/falling table rather than tunnelling.
        if (ball_state_.vz_ball < table_state_.z_t_dot) {
            ball_state_.vz_ball = table_state_.z_t_dot;
        }
    }

    // --- Table boundary clamp (circular disc, radius = table_radius) ---
    const double r2 = ball_state_.x * ball_state_.x + ball_state_.y * ball_state_.y;
    const double R      = params_.table_radius;
    const double bounce = params_.bounce_coeff;

    if (r2 > R * R) {
        const double r_dist = std::sqrt(r2);
        const double scale  = R / r_dist;
        const double bx     = ball_state_.x;
        const double by     = ball_state_.y;
        ball_state_.x = bx * scale;
        ball_state_.y = by * scale;

        // Reflect the radial component of velocity and damp by bounce coefficient.
        const double nx  = bx / r_dist;
        const double ny  = by / r_dist;
        const double v_r = ball_state_.vx * nx + ball_state_.vy * ny;

        ball_state_.vx -= (1.0 + bounce) * v_r * nx;
        ball_state_.vy -= (1.0 + bounce) * v_r * ny;
    }
}

} // namespace ball_balancer
