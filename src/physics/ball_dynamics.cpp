#include <ball_balancer/physics/ball_dynamics.hpp>
#include <algorithm>
#include <cmath>

/**
 * @file ball_dynamics.cpp
 * @brief Full 3D ball equations of motion for the ball balancer system.
 *
 * Implements contact and free-flight dynamics derived from first principles.
 * See header for coordinate conventions and equation summary.
 *
 * Notation used throughout (matching derivation document):
 *   phi   (φ) = VARPHI_X  — roll  around X, drives ball in Y
 *   theta (θ) = THETA_Y  — pitch around Y, drives ball in X
 *   r         = ball_radius
 *   m         = ball_mass
 *   g         = gravity
 *   b         = viscous_friction_coeff
 *   e         = bounce_coeff (coefficient of restitution)
 */

namespace ball_balancer {

BallDynamics::BallDynamics(const SystemParameters& params)
    : params_(params)
{
}

// ----------------------------------------------------------------------------
// isInContact
// ----------------------------------------------------------------------------

bool BallDynamics::isInContact(const StateVector& state, const TableState& table) const {
    const double x   = state(state_index::X);
    const double y   = state(state_index::Y);
    const double z_b = state(state_index::Z_BALL);

    // Table surface height at (x, y) under small-angle approximation:
    //   z_surface = z_t + x*theta - y*phi
    // Ball centre must be at z_surface + r when in contact.
    const double z_surface = table.z_t + x * table.theta - y * table.phi;
    return z_b <= z_surface + params_.ball_radius;
}

// ----------------------------------------------------------------------------
// computeNormalForce
// ----------------------------------------------------------------------------

double BallDynamics::computeNormalForce(const StateVector& state, const TableState& table) const {
    const double x   = state(state_index::X);
    const double y   = state(state_index::Y);
    const double m   = params_.ball_mass;
    const double g   = params_.gravity;
    const double r   = params_.ball_radius;

    // Simplified explicit normal force (small lateral velocities assumed):
    //   N = m * [g + z_t_ddot + 2*x*theta_ddot - 2*y*phi_ddot
    //              + phi_dot^2*(z_t + r) + theta_dot^2*(z_t + r)]
    const double z_ref = table.z_t + r;
    const double N = m * (g
        + table.z_t_ddot
        + 2.0 * x * table.theta_ddot
        - 2.0 * y * table.phi_ddot
        + (table.phi_dot * table.phi_dot + table.theta_dot * table.theta_dot) * z_ref);

    // Normal force cannot be negative (table cannot pull ball down)
    return std::max(0.0, N);
}

// ----------------------------------------------------------------------------
// computeAccelerations
// ----------------------------------------------------------------------------

void BallDynamics::computeAccelerations(
    const StateVector& state,
    const TableState& table,
    double& ax, double& ay, double& az
) const {
    const double g = params_.gravity;

    // Free-flight mode: ball has left the surface
    const double N = computeNormalForce(state, table);
    if (!isInContact(state, table) || N <= 0.0) {
        ax = 0.0;
        ay = 0.0;
        az = -g;
        return;
    }

    // --- Contact mode ---
    const double x   = state(state_index::X);
    const double y   = state(state_index::Y);
    const double z_b = state(state_index::Z_BALL);
    const double vx  = state(state_index::VX);
    const double vy  = state(state_index::VY);
    const double m   = params_.ball_mass;
    const double b   = params_.viscous_friction_coeff;

    // Relative velocity of ball w.r.t. table contact point (small-angle):
    //   v_rel_x = vx - theta_dot*(z_b - z_t)
    //   v_rel_y = vy + phi_dot*(z_b - z_t)
    const double dz      = z_b - table.z_t;
    const double v_rel_x = vx - table.theta_dot * dz;
    const double v_rel_y = vy + table.phi_dot  * dz;

    // Contact accelerations (small-angle, viscous friction):
    //   ax = -g*theta  - theta_dot^2*x - theta_ddot*z_b - (b/m)*v_rel_x
    //   ay =  g*phi    - phi_dot^2*y   - phi_ddot*z_b   - (b/m)*v_rel_y
    //   az = -g + N/m + phi_ddot*y - theta_ddot*x - phi_dot^2*z_b - theta_dot^2*z_b
    const double bm = b / m;
    ax = -g * table.theta
         - table.theta_dot * table.theta_dot * x
         - table.theta_ddot * z_b
         - bm * v_rel_x;

    ay =  g * table.phi
         - table.phi_dot * table.phi_dot * y
         - table.phi_ddot * z_b
         - bm * v_rel_y;

    az = -g + N / m
         + table.phi_ddot   * y
         - table.theta_ddot * x
         - (table.phi_dot   * table.phi_dot
          + table.theta_dot * table.theta_dot) * z_b;
}

// ----------------------------------------------------------------------------
// applyBounce
// ----------------------------------------------------------------------------

void BallDynamics::applyBounce(StateVector& state, const TableState& table) const {
    const double x   = state(state_index::X);
    const double y   = state(state_index::Y);
    const double vx  = state(state_index::VX);
    const double vy  = state(state_index::VY);
    const double vz  = state(state_index::VZ_BALL);
    const double e   = params_.bounce_coeff;

    // Velocity of table surface at ball contact point in Z direction (small-angle):
    //   vz_contact = z_t_dot + vx*theta + x*theta_dot - vy*phi - y*phi_dot
    const double vz_contact = table.z_t_dot
        + vx * table.theta + x * table.theta_dot
        - vy * table.phi   - y * table.phi_dot;

    const double v_rel_normal = vz - vz_contact;

    // Only apply bounce if ball is approaching the surface (v_rel_normal < 0)
    if (v_rel_normal >= 0.0) {
        return;
    }

    // Coefficient-of-restitution update:
    //   vz_after = (1 + e)*vz_contact - e*vz_before
    state(state_index::VZ_BALL) = (1.0 + e) * vz_contact - e * vz;
    // vx, vy unchanged (no tangential impulse assumed)
}

} // namespace ball_balancer
