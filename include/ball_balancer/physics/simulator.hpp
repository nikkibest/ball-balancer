#pragma once

#include <ball_balancer/core/types.hpp>
#include <ball_balancer/physics/ball_dynamics.hpp>
#include <random>

/**
 * @file simulator.hpp
 * @brief Physics simulation of ball on tilting table
 *
 * This simulator models a ball rolling on a 2-DOF tilting table platform.
 * The dynamics are derived from first principles using Newton's laws.
 *
 * Physical Model:
 * - Ball modeled as solid sphere with mass m and radius r
 * - No-slip rolling condition (rolling without slipping)
 * - Table tilt causes gravitational acceleration along surface
 * - Rolling friction opposes motion
 *
 * Equations of Motion (first principles):
 * For a ball rolling on an inclined plane at angle θ:
 *   a = (g * sin(θ)) / (1 + I/(mr²))
 *
 * For solid sphere: I = (2/5)*m*r², so:
 *   a = (5/7) * g * sin(θ)
 *
 * State-Space Form:
 *   dx/dt = vx
 *   dy/dt = vy
 *   dvx/dt = (5/7) * g * sin(theta_x) - friction * vx
 *   dvy/dt = (5/7) * g * sin(theta_y) - friction * vy
 *   dtheta_x/dt = (theta_x_cmd - theta_x) / tau_servo
 *   dtheta_y/dt = (theta_y_cmd - theta_y) / tau_servo
 *
 * @see research/ode-physical-system-modeling-cpp.md
 * @see docs/AGENT_TASKS.md (Physics Agent section)
 */

namespace ball_balancer {

/**
 * @brief Physics simulator for ball balancer system
 *
 * Simulates ball rolling on tilting table using hand-written RK4 integrator.
 * No external ODE library dependency required.
 * Provides simulated camera measurements with configurable noise.
 *
 * Responsibilities:
 * - Integrate ODE system forward in time (manual RK4 implementation)
 * - Model servo dynamics (first-order lag)
 * - Enforce physical constraints (table boundaries)
 * - Provide noisy measurements (camera simulation)
 * - Validate energy conservation (for testing)
 */
class Simulator {
public:
    /**
     * @brief Construct simulator with system parameters
     * @param params Physical parameters of the system
     */
    explicit Simulator(const SystemParameters& params);

    /**
     * @brief Integrate system forward by one time step
     * @param dt Time step (seconds)
     * @param control Control input (commanded table angles)
     *
     * Updates internal state by integrating ODEs using RK4.
     * Clamps ball position to table boundaries.
     */
    void step(double dt, const ControlVector& control);

    /**
     * @brief Reset simulation to initial state
     * @param ball  Initial ball state
     * @param table Initial table state
     */
    void reset(const BallState& ball, const TableState& table);

    /**
     * @brief Get current ball state (no noise)
     */
    const BallState& get_ball_state() const { return ball_state_; }

    /**
     * @brief Get current table state (no noise)
     */
    const TableState& get_table_state() const { return table_state_; }

    /**
     * @brief Set ball state directly (e.g., from GUI sliders when paused)
     */
    void set_ball_state(const BallState& ball) { ball_state_ = ball; }

    /**
     * @brief Set table state directly (e.g., from GUI sliders or FK injection)
     */
    void set_table_state(const TableState& table) {
        table_state_.phi   = table.phi;
        table_state_.theta = table.theta;
        table_state_.z_t   = table.z_t;
        // rates/accelerations are recomputed each step — do not overwrite them here
    }

    /**
     * @brief Update the table radius used for boundary clamping.
     * @param radius New table radius (m)
     */
    void set_table_radius(double radius) { params_.table_radius = radius; }

    /**
     * @brief Get simulated camera measurement (with noise)
     * @return Measured ball position with Gaussian noise
     *
     * Simulates camera by:
     * 1. Extracting ball position from state
     * 2. Adding Gaussian noise (std = camera_noise_std)
     */
    MeasurementVector get_measurement();

    /**
     * @brief Compute total mechanical energy (for validation)
     * @return Total energy (kinetic + potential) in Joules
     *
     * For validation: energy should be conserved (minus friction losses)
     * E = KE_translational + KE_rotational + PE
     *   = (1/2)*m*v² + (1/2)*I*ω² + m*g*h
     */
    double compute_total_energy() const;

    /**
     * @brief Get current time
     * @return Simulation time (seconds)
     */
    double get_time() const { return time_; }

    /**
     * @brief Query whether the ball is currently in contact with the table.
     * @return true if ball centre is at or below the table surface + radius
     */
    bool isInContact() const {
        return ball_dynamics_.isInContact(ball_state_, table_state_);
    }

private:
    /**
     * @brief ODE for ball states: computes (dx, dy, dz_ball, dvx, dvy, dvz_ball).
     *
     * @param ball   Current ball state
     * @param table  Current table state (read-only)
     * @param d_ball Output: ball state derivatives
     */
    void ball_dynamics_step(const BallState& ball, const TableState& table, BallState& d_ball);

    /**
     * @brief Update table_state_ rates from current_control_ (first-order servo lag).
     *
     * Computes phi_dot, theta_dot, z_t_dot from (cmd - current) / tau.
     * Sets accelerations to zero (conservative approximation).
     */
    void update_table_rates();

    /**
     * @brief Enforce contact constraint and table boundary conditions.
     *
     * After each RK4 step:
     *  1. Snap ball Z to table surface if it has penetrated (contact mode).
     *  2. Apply bounce if the ball was approaching the surface.
     *  3. Clamp X/Y to table boundaries with elastic rebound.
     */
    void enforce_constraints();

    // System parameters
    SystemParameters params_;

    // Ball dynamics engine (encapsulates full 3D EOM)
    BallDynamics ball_dynamics_;

    // Primary state
    BallState  ball_state_;
    TableState table_state_;
    double time_;

    // Current control input (stored for rate computation)
    ControlVector current_control_;

    // Random number generation for measurement noise
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
};

} // namespace ball_balancer
