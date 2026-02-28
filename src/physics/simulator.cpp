#include <ball_balancer/physics/simulator.hpp>
#include <cmath>
#include <algorithm>

/**
 * @file simulator.cpp
 * @brief Physics simulation of ball on tilting table.
 *
 * Ball equations of motion are fully delegated to BallDynamics.
 * This file retains only:
 *   - RK4 integration loop
 *   - Servo dynamics (first-order lag for table angles)
 *   - Post-step constraint enforcement (contact snap, bounce, boundary clamp)
 *   - Measurement noise generation
 *
 * Axis convention (unchanged from project-wide convention):
 *   THETA_X (phi)   = roll  around X → drives ball in Y
 *   THETA_Y (theta) = pitch around Y → drives ball in X
 *
 * @see include/ball_balancer/physics/ball_dynamics.hpp for ball EOM derivation
 */

namespace ball_balancer {

Simulator::Simulator(const SystemParameters& params)
    : params_(params)
    , ball_dynamics_(params)
    , state_(StateVector::Zero())
    , time_(0.0)
    , current_control_(ControlVector::Zero())
    , rng_(std::random_device{}())
    , noise_dist_(0.0, params.camera_noise_std)
{
    params_.initialize();

    // Place ball resting on flat table at default position
    state_(state_index::Z_BALL)  = params_.ball_radius;  // z_b = r (flat table, z_t=0)
    state_(state_index::Z_TABLE) = 0.0;
}

void Simulator::step(double dt, const ControlVector& control) {
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

    // RK4 integration
    StateVector k1, k2, k3, k4;
    StateVector temp;

    dynamics(state_, k1, time_);

    temp = state_ + (dt * 0.5) * k1;
    dynamics(temp, k2, time_ + dt * 0.5);

    temp = state_ + (dt * 0.5) * k2;
    dynamics(temp, k3, time_ + dt * 0.5);

    temp = state_ + dt * k3;
    dynamics(temp, k4, time_ + dt);

    state_ = state_ + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    // Post-step: enforce contact constraint and boundary conditions
    enforce_constraints();

    time_ += dt;
}

void Simulator::reset(const StateVector& initial_state) {
    state_ = initial_state;
    time_  = 0.0;
    current_control_.setZero();

    // Ensure initial Z_BALL is at least on the table surface
    const double z_t      = state_(state_index::Z_TABLE);
    const double z_surface = z_t + params_.ball_radius;
    if (state_(state_index::Z_BALL) < z_surface) {
        state_(state_index::Z_BALL) = z_surface;
    }

    enforce_constraints();
}

MeasurementVector Simulator::get_measurement() {
    MeasurementVector measurement;
    measurement(measurement_index::X_MEAS) = state_(state_index::X) + noise_dist_(rng_);
    measurement(measurement_index::Y_MEAS) = state_(state_index::Y) + noise_dist_(rng_);
    return measurement;
}

double Simulator::compute_total_energy() const {
    const double m  = params_.ball_mass;
    const double r  = params_.ball_radius;
    const double g  = params_.gravity;
    const double I  = params_.ball_inertia;

    const double x   = state_(state_index::X);
    const double y   = state_(state_index::Y);
    const double z_b = state_(state_index::Z_BALL);
    const double vx  = state_(state_index::VX);
    const double vy  = state_(state_index::VY);
    const double vz  = state_(state_index::VZ_BALL);

    // Translational kinetic energy (full 3D)
    const double v2      = vx*vx + vy*vy + vz*vz;
    const double KE_trans = 0.5 * m * v2;

    // Rotational kinetic energy (no-slip rolling, using horizontal speed)
    const double vh    = std::sqrt(vx*vx + vy*vy);
    const double omega = vh / r;
    const double KE_rot = 0.5 * I * omega * omega;

    // Gravitational potential energy (full 3D height)
    const double theta_x = state_(state_index::THETA_X);
    const double theta_y = state_(state_index::THETA_Y);
    // Height contribution from tilt (small-angle) + actual z_b
    const double h  = z_b + x * std::sin(theta_y) + y * std::sin(theta_x);
    const double PE = m * g * h;

    (void)x; (void)y;  // used via h above
    return KE_trans + KE_rot + PE;
}

// ----------------------------------------------------------------------------
// Private helpers
// ----------------------------------------------------------------------------

TableState Simulator::buildTableState(const StateVector& state) const {
    TableState table;

    table.phi   = state(state_index::THETA_X);
    table.theta = state(state_index::THETA_Y);
    table.z_t   = state(state_index::Z_TABLE);

    // Angular rates from first-order servo dynamics:
    //   dtheta/dt = (cmd - theta) / tau_servo
    const double tau = params_.servo_time_constant;
    table.phi_dot   = (current_control_(control_index::THETA_X_CMD) - table.phi)   / tau;
    table.theta_dot = (current_control_(control_index::THETA_Y_CMD) - table.theta) / tau;

    // Angular accelerations: zero (conservative; avoids differencing across steps)
    table.phi_ddot   = 0.0;
    table.theta_ddot = 0.0;

    // Table Z is a stub — no actuation
    table.z_t_dot  = 0.0;
    table.z_t_ddot = 0.0;

    return table;
}

void Simulator::dynamics(const StateVector& state, StateDerivative& dstate, double /* t */) {
    const TableState table = buildTableState(state);

    // Ball accelerations from full 3D dynamics
    double ax{0.0}, ay{0.0}, az{0.0};
    ball_dynamics_.computeAccelerations(state, table, ax, ay, az);

    // Ball position derivatives = velocities
    dstate(state_index::X)       = state(state_index::VX);
    dstate(state_index::Y)       = state(state_index::VY);
    dstate(state_index::Z_BALL)  = state(state_index::VZ_BALL);

    // Ball velocity derivatives = accelerations
    dstate(state_index::VX)      = ax;
    dstate(state_index::VY)      = ay;
    dstate(state_index::VZ_BALL) = az;

    // Servo dynamics (first-order lag): dtheta/dt = (cmd - theta) / tau
    const double tau = params_.servo_time_constant;
    dstate(state_index::THETA_X) =
        (current_control_(control_index::THETA_X_CMD) - state(state_index::THETA_X)) / tau;
    dstate(state_index::THETA_Y) =
        (current_control_(control_index::THETA_Y_CMD) - state(state_index::THETA_Y)) / tau;

    // Table Z: stub, no dynamics
    dstate(state_index::Z_TABLE) = 0.0;
}

void Simulator::enforce_constraints() {
    const TableState table = buildTableState(state_);

    // --- Contact constraint ---
    // Surface height at current (x, y) under small-angle approximation
    const double x   = state_(state_index::X);
    const double y   = state_(state_index::Y);
    const double z_surface = table.z_t
        + params_.ball_radius
        + x * table.theta
        - y * table.phi;

    if (state_(state_index::Z_BALL) < z_surface) {
        // Ball has penetrated the table — snap it to the surface
        state_(state_index::Z_BALL) = z_surface;

        // Apply coefficient-of-restitution bounce if approaching
        ball_dynamics_.applyBounce(state_, table);
    }

    // --- Table boundary clamp (X / Y) ---
    const double half_len   = params_.table_length / 2.0;
    const double half_width = params_.table_width  / 2.0;
    const double bounce     = params_.bounce_coeff;

    if (state_(state_index::X) > half_len) {
        state_(state_index::X)  = half_len;
        state_(state_index::VX) = -bounce * state_(state_index::VX);
    } else if (state_(state_index::X) < -half_len) {
        state_(state_index::X)  = -half_len;
        state_(state_index::VX) = -bounce * state_(state_index::VX);
    }

    if (state_(state_index::Y) > half_width) {
        state_(state_index::Y)  = half_width;
        state_(state_index::VY) = -bounce * state_(state_index::VY);
    } else if (state_(state_index::Y) < -half_width) {
        state_(state_index::Y)  = -half_width;
        state_(state_index::VY) = -bounce * state_(state_index::VY);
    }
}

} // namespace ball_balancer
