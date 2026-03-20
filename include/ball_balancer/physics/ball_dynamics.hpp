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
 * This class is stateless with respect to the simulation. Methods are therefore `const`.
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
     * @param ball   Current ball state
     * @param table  Current table kinematics
     * @return true if ball centre is at or below the table surface + radius
     */
    bool isInContact(const BallState& ball, const TableState& table) const;

    /**
     * @brief Compute the table normal force on the ball.
     *
     * Uses the simplified explicit form that assumes small lateral velocities
     * (ẋ_b ≈ 0, ẏ_b ≈ 0) to avoid the implicit coupling between N and ẍ/ÿ.
     * Result is clamped to N ≥ 0 (table cannot pull the ball down).
     *
     * @param ball   Current ball state
     * @param table  Current table kinematics
     * @return Normal force in Newtons (≥ 0)
     */
    double computeNormalForce(const BallState& ball, const TableState& table) const;

    /**
     * @brief Compute ball accelerations (ax, ay, az) in the inertial frame.
     *
     * Selects contact or free-flight dynamics based on isInContact() and N.
     * TableState is a read-only input; only ball accelerations are output.
     *
     * @param ball   Current ball state
     * @param table  Current table kinematics (read-only)
     * @param ax     Output: ball acceleration in X (m/s²)
     * @param ay     Output: ball acceleration in Y (m/s²)
     * @param az     Output: ball acceleration in Z (m/s²)
     */
    void computeAccelerations(
        const BallState& ball,
        const TableState& table,
        double& ax, double& ay, double& az
    ) const;

    /**
     * @brief Apply coefficient-of-restitution bounce to the ball's Z velocity.
     *
     * Should be called after the ball has been detected to have penetrated the
     * table surface (z_b < z_surface + r) and is approaching it.
     *
     * Only modifies ball.vz_ball; x/y velocities are unchanged.
     * TableState is a read-only input.
     *
     * @param ball   Ball state to modify in place
     * @param table  Current table kinematics (read-only)
     */
    void applyBounce(BallState& ball, const TableState& table) const;

private:
    SystemParameters params_;
};

} // namespace ball_balancer
