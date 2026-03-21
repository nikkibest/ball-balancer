#include <ball_balancer/physics/table_kinematics.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

/**
 * @file table_kinematics.cpp
 * @brief Forward and inverse kinematics of the 3-arm parallel table mechanism.
 *
 * Variable naming follows the derivation document exactly:
 *   L1, L2  — link lengths
 *   Rg, Rt  — mounting radii
 *   psi[i]  — arm azimuth angle ψᵢ = 2π·i/3
 *   phi     — table roll  φ  (rotation about X)
 *   theta   — table pitch θ  (rotation about Y)
 *   z_t     — table centre height
 *   d_r     — radial offset in arm's vertical plane
 *   d_z     — vertical offset
 *   T[i]    — table attachment point in inertial frame
 *   E[i]    — elbow position in inertial frame
 *   G[i]    — ground attachment point
 *
 * @see Table Dynamics Equations - Ball Balancing System.md
 */

namespace ball_balancer {

// ============================================================================
// Construction
// ============================================================================

TableKinematics::TableKinematics(const SystemParameters& params)
    : L1_(params.arm_L1)
    , L2_(params.arm_L2)
    , Rg_(params.arm_Rg)
    , Rt_(params.arm_Rt)
    , zNominal_(params.arm_z_nominal > 0.0 ? params.arm_z_nominal : 0.12)
{
    assert(L1_ > 0.0 && "arm_L1 must be positive");
    assert(L2_ > 0.0 && "arm_L2 must be positive");
    assert(Rg_ > 0.0 && "arm_Rg must be positive");
    assert(Rt_ > 0.0 && "arm_Rt must be positive");
}

// ============================================================================
// Public — Inverse Kinematics
// ============================================================================

std::optional<ServoAngles> TableKinematics::inverseKinematics(
    double phi, double theta, double z_t) const
{
    ServoAngles result;

    for (int i = 0; i < N_ARMS; ++i) {
        // Step 1: table attachment point Tᵢ
        const auto T = tableAttachPoint(i, phi, theta, z_t);
        // Step 2: ground attachment point Gᵢ
        const auto G = groundPoint(i);

        // Step 3: difference vector dᵢ = Tᵢ − Gᵢ
        const double dx = T[0] - G[0];
        const double dy = T[1] - G[1];

        // Step 4: project onto radial direction r̂ᵢ = [cos ψᵢ, sin ψᵢ, 0]
        const double cPsi = std::cos(PSI[i]);
        const double sPsi = std::sin(PSI[i]);
        const double d_r  =  dx * cPsi + dy * sPsi;   // radial component
        const double d_z  = T[2];                      // vertical (ground z = 0)

        // Step 5: solve 2D IK in the arm's radial plane
        auto alpha = solveArmIK(d_r, d_z);
        if (!alpha) return std::nullopt;   // out of reach

        result.alpha[i] = *alpha;
    }

    return result;
}

// ============================================================================
// Public — Forward Kinematics
// ============================================================================

std::optional<FKResult> TableKinematics::forwardKinematics(
    const ServoAngles& servos,
    const ElbowAngles& prev,
    FKMethod           method) const
{
    std::optional<std::array<double, 3>> pose;

    switch (method) {
        case FKMethod::NewtonRaphson: {
            // Warm-start: recover phi0/theta0/z0 by running a quick YouTube
            // solve, or use zero if that fails.
            double phi0 = 0.0, theta0 = 0.0, z0 = zNominal_;
            if (auto yt = fkYouTube(servos)) {
                phi0 = (*yt)[0]; theta0 = (*yt)[1]; z0 = (*yt)[2];
            }
            pose = fkNewtonRaphson(servos, phi0, theta0, z0);
            break;
        }
        case FKMethod::YouTubeClosedForm:
            pose = fkYouTube(servos);
            break;
        case FKMethod::GeometryBased:
            pose = fkGeometryBased(servos, prev);
            break;
    }

    if (!pose) return std::nullopt;

    FKResult result;
    result.phi   = (*pose)[0];
    result.theta = (*pose)[1];
    result.z_t   = (*pose)[2];

    // Compute upper-link angles β_i from the solved T_i and E_i.
    // β_i is the angle of (T_i − E_i) in the arm's radial plane from horizontal.
    for (int i = 0; i < N_ARMS; ++i) {
        const auto T = tableAttachPoint(i, result.phi, result.theta, result.z_t);
        const auto E = elbowPosition(i, servos.alpha[i]);
        const double dz = T[2] - E[2];
        const double dx = T[0] - E[0];
        const double dy = T[1] - E[1];
        const double cPsi = std::cos(PSI[i]);
        const double sPsi = std::sin(PSI[i]);
        const double dr = dx * cPsi + dy * sPsi;  // radial component in arm plane
        result.elbow.beta[i] = std::atan2(dz, dr);
    }

    return result;
}

// ============================================================================
// Private — Geometry helpers
// ============================================================================

std::array<double, 3> TableKinematics::tableAttachPoint(
    int i, double phi, double theta, double z_t) const
{
    // The renderer applies Ry(-θ)·Rx(-φ) in physics space.
    // Derived from GL coord mapping: GL X = phys X, GL Y = phys Z, GL Z = phys Y.
    //   renderer rot_x (around GL-X) acts on (gl_y, gl_z) = (phys_z, phys_y)
    //     → (phys_z·cosφ - phys_y·sinφ, phys_z·sinφ + phys_y·cosφ) = Rx(-φ) in physics
    //   renderer rot_y (around GL-Y) acts on (gl_x, gl_y) = (phys_x, phys_z)
    //     → Ry(-θ) in physics
    //
    // R = Ry(-θ)·Rx(-φ):
    //   = |  cosθ       sinθ·sinφ   -sinθ·cosφ |
    //     |   0          cosφ         sinφ      |
    //     |  sinθ      -cosθ·sinφ    cosθ·cosφ  |
    //
    // T_i = [0, 0, z_t]^T + R · Rt · [cos(ψᵢ), sin(ψᵢ), 0]^T
    const double cp   = std::cos(phi),   sp = std::sin(phi);
    const double ct   = std::cos(theta), st = std::sin(theta);
    const double cPsi = std::cos(PSI[i]);
    const double sPsi = std::sin(PSI[i]);

    return {
        Rt_ * ( ct * cPsi + st * sp * sPsi),   // x: only cosψ col matters since sinψ col has sinθ·sinφ
        Rt_ * ( cp * sPsi),                     // y: 0·cosψ + cosφ·sinψ
        z_t + Rt_ * ( st * cPsi - ct * sp * sPsi),  // z: sinθ·cosψ − cosθ·sinφ·sinψ
    };
}

std::array<double, 3> TableKinematics::tableAttachPointFromBeta(
    int i, double alpha, double beta) const
{
    // E_i from alpha, then T_i = E_i + L2 * unit_vec(beta, psi_i)
    // The upper link lies in the arm's radial plane (defined by ψ_i).
    // β is measured from horizontal: β=0 → upper link horizontal outward,
    //                                β>0 → upper link tilts upward.
    const auto E = elbowPosition(i, alpha);
    const double cPsi = std::cos(PSI[i]);
    const double sPsi = std::sin(PSI[i]);
    const double cb   = std::cos(beta);
    const double sb   = std::sin(beta);

    return {
        E[0] - L2_ * cb * cPsi,
        E[1] - L2_ * cb * sPsi,
        E[2] + L2_ * sb,
    };
}

std::array<double, 3> TableKinematics::elbowPosition(int i, double alpha) const
{
    // α is measured from straight-down (-Z): α=0 → link hangs vertically,
    // α=π/2 → link points radially outward (horizontal).
    // Always α ∈ [0, π] so the elbow is always at or below ground level.
    //
    // E_i = [(Rg + L1·sinα)·cosψᵢ, (Rg + L1·sinα)·sinψᵢ, -L1·cosα]
    const double ca   = std::cos(alpha);
    const double sa   = std::sin(alpha);
    const double cPsi = std::cos(PSI[i]);
    const double sPsi = std::sin(PSI[i]);
    const double rEff = Rg_ + L1_ * ca;

    return { rEff * cPsi, rEff * sPsi, L1_ * sa };
}

std::array<double, 3> TableKinematics::groundPoint(int i) const
{
    return { Rg_ * std::cos(PSI[i]), Rg_ * std::sin(PSI[i]), 0.0 };
}

// ============================================================================
// Private — Per-arm IK
// ============================================================================

std::optional<double> TableKinematics::solveArmIK(double d_r, double d_z) const
{
    // Solve: αᵢ = atan2(d_z, d_r) + arccos(Cᵢ / Dᵢ)   [elbow-down branch]
    //
    // where:
    //   Dᵢ = sqrt(d_r² + d_z²)                        (ground-to-target distance)
    //   Cᵢ = (d_r² + d_z² + L1² − L2²) / (2·L1)      (from law of cosines)
    //
    // Reachability: |Cᵢ / Dᵢ| ≤ 1  ⟺  |L1 − L2| ≤ Dᵢ ≤ L1 + L2

    const double D2 = d_r * d_r + d_z * d_z;
    const double D  = std::sqrt(D2);

    if (D < std::numeric_limits<double>::epsilon())
        return std::nullopt;   // degenerate: target at ground point

    const double C     = (D2 + L1_ * L1_ - L2_ * L2_) / (2.0 * L1_);
    const double ratio = C / D;

    if (std::abs(ratio) > 1.0 + 1e-9)
        return std::nullopt;   // triangle inequality violated

    const double phi_dir = std::atan2(d_z, d_r);
    const double delta   = std::acos(std::clamp(ratio, -1.0, 1.0));

    // Elbow-below-ground branch: subtract delta so the elbow hangs under
    // the table rather than poking up through it.
    // phi_dir - delta is the first-link angle measured from horizontal (+X axis).
    // Convert to α convention (measured from downward -Z): α = (horiz_angle) + π/2
    const double alpha = phi_dir - delta + M_PI / 2.0;
    return std::max(0.0, alpha);   // clamp to [0, π]; negative would mean elbow above ground
}

// ============================================================================
// Private — FK: Newton-Raphson
// ============================================================================

std::optional<std::array<double, 3>> TableKinematics::fkNewtonRaphson(
    const ServoAngles& servos,
    double phi0, double theta0, double z0) const
{
    // Iterative solution: find (φ, θ, z_t) s.t. fᵢ(φ,θ,z_t) = 0 for i=0,1,2
    //
    // fᵢ = ||Tᵢ(φ,θ,z_t) − Eᵢ(αᵢ)||² − L2²
    //
    // Newton step: Δq = −J⁻¹ · f(q)

    double phi   = phi0;
    double theta = theta0;
    double z_t   = z0;

    for (int iter = 0; iter < NR_MAX_ITER; ++iter) {
        const auto f = residuals(servos, phi, theta, z_t);

        // Convergence check: L∞ norm
        const double fMax = std::max({std::abs(f[0]), std::abs(f[1]), std::abs(f[2])});
        if (fMax < NR_TOL)
            return {{ phi, theta, z_t }};

        const auto J   = jacobian(servos, phi, theta, z_t);
        const auto neg_f = std::array<double, 3>{ -f[0], -f[1], -f[2] };
        const auto dq  = solve3x3(J, neg_f);

        if (!dq) return std::nullopt;   // singular Jacobian

        phi   += (*dq)[0];
        theta += (*dq)[1];
        z_t   += (*dq)[2];
    }

    // Did not converge
    return std::nullopt;
}

// ============================================================================
// Private — FK: YouTube closed-form
// ============================================================================

std::optional<std::array<double, 3>> TableKinematics::fkYouTube(
    const ServoAngles& servos) const
{
    // The YouTube model computes, for each arm i, the height z_i of the
    // table attachment point analytically from the servo angle.
    //
    // In each arm's radial plane the elbow is at:
    //   radial position: Rg + L1·cosα
    //   height:          L1·sinα
    //
    // The table attachment point sits at radial position Rt from the table
    // centre. In the flat-table case it is directly above the ground point
    // at radial distance Rt from the centre, height z_t. For each arm the
    // vertical distance from elbow to table attachment is:
    //
    //   vertical arm reach: h_i − E_z,i  where h_i = z_i (table attachment z)
    //
    // The horizontal arm reach from the elbow must cover Rt − (Rg + L1·cosα_i):
    //   Δr_i = Rt − (Rg + L1·cosα_i)
    //
    // Link constraint (in the radial plane):
    //   (Δr_i)² + (z_i − E_z,i)² = L2²
    //   z_i = E_z,i + sqrt(L2² − Δr_i²)     [upper-branch: table above elbow]
    //
    // This is valid under the small-tilt approximation (d_t ≈ 0).

    double z[3];

    for (int i = 0; i < N_ARMS; ++i) {
        const double alpha  = servos.alpha[i];
        // New convention: α from downward (-Z), α=0 → straight down, α=π/2 → horizontal
        const double E_z    = -L1_ * std::cos(alpha);          // always ≤ 0
        const double E_r    = Rg_ + L1_ * std::sin(alpha);     // elbow radial pos.
        const double dR     = Rt_ - E_r;                     // radial gap

        const double arg = L2_ * L2_ - dR * dR;
        if (arg < 0.0) return std::nullopt;   // arm cannot reach Rt

        z[i] = E_z + std::sqrt(arg);   // table attachment height for arm i
    }

    // Recover pose from the three heights using the small-angle linear inverse:
    //   z_t = (z0 + z1 + z2) / 3
    //   θ   = (z1 + z2 − 2·z0) / (3·Rt)   [arm 0 on +X axis → controls θ]
    //   φ   = (z1 − z2) / (√3·Rt)
    const double z_t  = (z[0] + z[1] + z[2]) / 3.0;
    // With R = Ry(-θ)·Rx(-φ):
    //   z[1]+z[2]−2·z[0] = −3·Rt·θ  →  thet = −(z1+z2−2z0)/(3Rt)
    //   z[1]−z[2]         = −√3·Rt·φ →  ph   = −(z1−z2)/(√3·Rt)
    const double thet = -(z[1] + z[2] - 2.0 * z[0]) / (3.0 * Rt_);
    const double ph   = -(z[1] - z[2]) / (std::sqrt(3.0) * Rt_);

    return {{ ph, thet, z_t }};
}

// ============================================================================
// Private — FK: Geometry-based β-solver
// ============================================================================

std::optional<std::array<double, 3>> TableKinematics::fkGeometryBased(
    const ServoAngles& servos,
    const ElbowAngles& prevBeta) const
{
    // Stub — implementation added in Phase 2 (Task 2.1–2.4)
    return std::nullopt;
}

std::array<double, 3> TableKinematics::betaResiduals(
    const std::array<double, 3>& /*beta*/,
    const std::array<double, 3>& /*A*/,
    const std::array<double, 3>& /*B*/) const
{
    // Stub — implementation added in Phase 2 (Task 2.2)
    return {};
}

std::array<std::array<double, 3>, 3> TableKinematics::betaJacobian(
    const std::array<double, 3>& /*beta*/,
    const std::array<double, 3>& /*A*/,
    const std::array<double, 3>& /*B*/) const
{
    // Stub — implementation added in Phase 2 (Task 2.3)
    return {};
}

// ============================================================================
// Private — Residuals and Jacobian for Newton-Raphson
// ============================================================================

std::array<double, 3> TableKinematics::residuals(
    const ServoAngles& servos,
    double phi, double theta, double z_t) const
{
    std::array<double, 3> f{};

    for (int i = 0; i < N_ARMS; ++i) {
        const auto T = tableAttachPoint(i, phi, theta, z_t);
        const auto E = elbowPosition(i, servos.alpha[i]);

        const double dx = T[0] - E[0];
        const double dy = T[1] - E[1];
        const double dz = T[2] - E[2];

        f[i] = dx*dx + dy*dy + dz*dz - L2_*L2_;
    }

    return f;
}

std::array<std::array<double, 3>, 3> TableKinematics::jacobian(
    const ServoAngles& servos,
    double phi, double theta, double z_t) const
{
    // J[i][j] = ∂fᵢ/∂qⱼ = 2·(Tᵢ − Eᵢ) · ∂Tᵢ/∂qⱼ
    //
    // q = (φ, θ, z_t)
    //
    // ∂T/∂z_t = [0, 0, 1]^T  (same for all arms)
    //
    // ∂R/∂φ = Ry(θ) · ∂Rx(φ)/∂φ
    //       = | 0             0       0        |
    //         | sinθ·cosφ   −sinφ    cosθ·cosφ |
    //         | sinθ·sinφ    cosφ   −cosθ·sinφ |
    //
    // Using R = Ry(-θ)·Rx(-φ) to match renderer convention (see tableAttachPoint).
    //
    // R = |  cosθ       sinθ·sinφ   -sinθ·cosφ |
    //     |   0          cosφ         sinφ      |
    //     |  sinθ      -cosθ·sinφ    cosθ·cosφ  |
    //
    // ∂R/∂φ = |  0       sinθ·cosφ    sinθ·sinφ  |
    //         |  0       −sinφ         cosφ       |
    //         |  0      −cosθ·cosφ   -cosθ·sinφ  |
    //
    // ∂R/∂θ = | −sinθ   cosθ·sinφ   -cosθ·cosφ  |
    //         |  0        0            0          |
    //         |  cosθ   sinθ·sinφ   -sinθ·cosφ  |
    //
    // ∂Tᵢ/∂qⱼ = (∂R/∂qⱼ) · Rt · [cosψᵢ, sinψᵢ, 0]^T

    const double cp = std::cos(phi),   sp = std::sin(phi);
    const double ct = std::cos(theta), st = std::sin(theta);

    // Precompute ∂R/∂φ and ∂R/∂θ acting on Rt·[cosψ, sinψ, 0]:
    //
    // Let p_r = Rt·cosψ,  p_t = Rt·sinψ  (in-plane components of Pᵢᵀ)
    //
    // (∂R/∂φ)·[p_r, p_t, 0]^T:
    //   x: st·cp·p_t
    //   y: −sp·p_t
    //   z: −ct·cp·p_t
    //
    // (∂R/∂θ)·[p_r, p_t, 0]^T:
    //   x: −st·p_r + ct·sp·p_t
    //   y: 0
    //   z:  ct·p_r + st·sp·p_t

    std::array<std::array<double, 3>, 3> J{};

    for (int i = 0; i < N_ARMS; ++i) {
        const auto T = tableAttachPoint(i, phi, theta, z_t);
        const auto E = elbowPosition(i, servos.alpha[i]);

        const double diff_x = T[0] - E[0];
        const double diff_y = T[1] - E[1];
        const double diff_z = T[2] - E[2];

        const double cPsi = std::cos(PSI[i]);
        const double sPsi = std::sin(PSI[i]);
        const double pr   = Rt_ * cPsi;   // Pᵢᵀ radial component
        const double pt   = Rt_ * sPsi;   // Pᵢᵀ tangential component

        // dTi/dphi = [st*cp*pt,  −sp*pt,  −ct*cp*pt]  (from ∂R/∂φ·[pr,pt,0])
        const double dT_dphi_x =  st * cp * pt;
        const double dT_dphi_y = -sp * pt;
        const double dT_dphi_z = -ct * cp * pt;

        // dTi/dtheta = [−st*pr + ct*sp*pt,  0,  ct*pr + st*sp*pt]
        const double dT_dtheta_x = -st * pr + ct * sp * pt;
        const double dT_dtheta_y =  0.0;
        const double dT_dtheta_z =  ct * pr + st * sp * pt;

        // dTi/dz_t = [0, 0, 1]
        const double dT_dzt_z = 1.0;

        // J[i][0] = ∂fᵢ/∂φ   = 2·diff · dTi/dphi
        J[i][0] = 2.0 * (diff_x * dT_dphi_x
                        + diff_y * dT_dphi_y
                        + diff_z * dT_dphi_z);

        // J[i][1] = ∂fᵢ/∂θ   = 2·diff · dTi/dtheta
        J[i][1] = 2.0 * (diff_x * dT_dtheta_x
                        + diff_y * dT_dtheta_y
                        + diff_z * dT_dtheta_z);

        // J[i][2] = ∂fᵢ/∂z_t = 2·diff_z·1
        J[i][2] = 2.0 * diff_z * dT_dzt_z;
    }

    return J;
}

// ============================================================================
// Private — 3×3 linear solver (Cramer's rule)
// ============================================================================

std::optional<std::array<double, 3>> TableKinematics::solve3x3(
    const std::array<std::array<double, 3>, 3>& A,
    const std::array<double, 3>&                b)
{
    // Cramer's rule for a 3×3 system A·x = b.
    // No heap allocation, no dependencies.

    const double det =
        A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
      - A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
      + A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (std::abs(det) < 1e-14) return std::nullopt;

    const double invDet = 1.0 / det;

    // x[0]: replace column 0 with b
    const double det0 =
        b[0]    * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
      - A[0][1] * (b[1]   *A[2][2] - A[1][2]*b[2]   )
      + A[0][2] * (b[1]   *A[2][1] - A[1][1]*b[2]   );

    // x[1]: replace column 1 with b
    const double det1 =
        A[0][0] * (b[1]   *A[2][2] - A[1][2]*b[2]   )
      - b[0]    * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
      + A[0][2] * (A[1][0]*b[2]    - b[1]   *A[2][0]);

    // x[2]: replace column 2 with b
    const double det2 =
        A[0][0] * (A[1][1]*b[2]    - b[1]   *A[2][1])
      - A[0][1] * (A[1][0]*b[2]    - b[1]   *A[2][0])
      + b[0]    * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    return std::array<double, 3>{ det0*invDet, det1*invDet, det2*invDet };
}

} // namespace ball_balancer
