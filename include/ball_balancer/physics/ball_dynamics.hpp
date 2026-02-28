#pragma once

#include <ball_balancer/core/types.hpp>

/**
 * @file ball_dynamics.hpp
 * @brief Full 3D ball dynamics for the ball balancer system
 *
 * Implements the equations of motion derived from first principles for a ball
 * resting on (or flying above) a tilting table. Two discrete modes are handled:
 *
 *   Contact mode  — Ball rests on table surface. Forces: gravity, normal force,
 *                   inertial effects (centrifugal + Euler), and viscous friction.
 *
 *   Free-flight mode — Ball has detached (N ≤ 0). Only gravity acts: az = -g.
 *
 * Coordinate / axis convention (matches existing project convention):
 *   phi   (φ) = THETA_X = roll  around X-axis → drives ball in Y direction
 *   theta (θ) = THETA_Y = pitch around Y-axis → drives ball in X direction
 *
 * All units SI: metres, seconds, radians, Newtons.
 *
 * @see /home/nds/Documents/Obsidian Vault/Ball Dynamics Equations - Ball Balancing System.md
 */

namespace ball_balancer {

// ============================================================================
// TableState — interface between Simulator (table) and BallDynamics (ball)
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
// BallDynamics
// ============================================================================

/**
 * @brief Encapsulates all ball equations of motion.
 *
 * Responsibilities:
 *  - Determine whether the ball is in contact with the table surface
 *  - Compute the table normal force (explicit simplified form)
 *  - Compute ball accelerations for both contact and free-flight modes
 *  - Apply coefficient-of-restitution bounce when the ball re-contacts
 *
 * This class is stateless with respect to the simulation (all state lives in
 * the Simulator's StateVector). Methods are therefore `const`.
 *
 * Physics reference — small-angle contact equations:
 *
 *   v_rel_x = vx - theta_dot*(z_b - z_t)
 *   v_rel_y = vy + phi_dot*(z_b - z_t)
 *
 *   ax = -g*theta  - theta_dot²*x - theta_ddot*z_b - (b/m)*v_rel_x
 *   ay =  g*phi    - phi_dot²*y   - phi_ddot*z_b   - (b/m)*v_rel_y
 *   az = -g + N/m + phi_ddot*y - theta_ddot*x - phi_dot²*z_b - theta_dot²*z_b
 *
 *   N  = m*[g + z_t_ddot + 2*x*theta_ddot - 2*y*phi_ddot
 *             + phi_dot²*(z_t+r) + theta_dot²*(z_t+r)]   (clamped ≥ 0)
 */
class BallDynamics {
public:
    /**
     * @brief Construct with system parameters.
     * @param params Physical parameters (mass, radius, gravity, friction, etc.)
     */
    explicit BallDynamics(const SystemParameters& params);

    /**
     * @brief Test whether the ball is in contact with the table surface.
     *
     * Contact condition (small-angle):
     *   z_b <= z_t + r + x*theta - y*phi
     *
     * @param state Current 9D state vector
     * @param table Current table kinematics
     * @return true if ball centre is at or below the table surface + radius
     */
    bool isInContact(const StateVector& state, const TableState& table) const;

    /**
     * @brief Compute the table normal force on the ball.
     *
     * Uses the simplified explicit form that assumes small lateral velocities
     * (ẋ_b ≈ 0, ẏ_b ≈ 0) to avoid the implicit coupling between N and ẍ/ÿ.
     * Result is clamped to N ≥ 0 (table cannot pull the ball down).
     *
     * @param state Current 9D state vector
     * @param table Current table kinematics
     * @return Normal force in Newtons (≥ 0)
     */
    double computeNormalForce(const StateVector& state, const TableState& table) const;

    /**
     * @brief Compute ball accelerations (ax, ay, az) in the inertial frame.
     *
     * Selects contact or free-flight dynamics based on isInContact() and N.
     *
     * @param state  Current 9D state vector
     * @param table  Current table kinematics
     * @param ax     Output: ball acceleration in X (m/s²)
     * @param ay     Output: ball acceleration in Y (m/s²)
     * @param az     Output: ball acceleration in Z (m/s²)
     */
    void computeAccelerations(
        const StateVector& state,
        const TableState& table,
        double& ax, double& ay, double& az
    ) const;

    /**
     * @brief Apply coefficient-of-restitution bounce to the ball's Z velocity.
     *
     * Should be called after the ball has been detected to have penetrated the
     * table surface (z_b < z_surface + r) and is approaching it.
     *
     * Only modifies state(VZ_BALL); x/y velocities are unchanged.
     *
     * @param state  State vector to modify in place
     * @param table  Current table kinematics
     */
    void applyBounce(StateVector& state, const TableState& table) const;

private:
    SystemParameters params_;
};

} // namespace ball_balancer
