#pragma once

#include <ball_balancer/core/types.hpp>
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
     * @param initial_state Initial state vector
     */
    void reset(const StateVector& initial_state);

    /**
     * @brief Get current true state (no noise)
     * @return Current state vector
     */
    const StateVector& get_state() const { return state_; }

    /**
     * @brief Set simulation state directly (e.g., from GUI sliders when paused)
     * @param state New state vector to apply immediately
     */
    void set_state(const StateVector& state) { state_ = state; }

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

private:
    /**
     * @brief ODE system: dx/dt = f(x, u, t)
     * @param state Current state
     * @param dstate Output: state derivative
     * @param t Current time
     *
     * Implements ball rolling dynamics with servo response.
     */
    void dynamics(const StateVector& state, StateDerivative& dstate, double t);

    /**
     * @brief Clamp ball position to table boundaries
     *
     * Prevents ball from rolling off table by:
     * - Clamping position to [-length/2, length/2] x [-width/2, width/2]
     * - Reversing velocity at boundaries (bounce with energy loss)
     */
    void enforce_constraints();

    // System parameters
    SystemParameters params_;

    // Current state
    StateVector state_;
    double time_;

    // Current control input (stored for ODE function)
    ControlVector current_control_;

    // Random number generation for measurement noise
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
};

} // namespace ball_balancer
