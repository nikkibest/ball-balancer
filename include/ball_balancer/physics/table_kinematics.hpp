#pragma once

#include <ball_balancer/core/types.hpp>
#include <array>
#include <optional>

/**
 * @file table_kinematics.hpp
 * @brief Forward and inverse kinematics of the 3-arm parallel table mechanism.
 *
 * The physical table is a 3-arm parallel linkage. Each arm has:
 *   - Lower link (length L1): driven by a servo motor, angle α_i from horizontal
 *   - Upper link (length L2): passive, ball-and-socket joints at both ends
 *
 * Arm ground attachment points lie at radius Rg from the centre, equally spaced
 * at 120°.  Table attachment points lie at radius Rt from the table centre.
 *
 * The table pose is parameterised by (φ, θ, z_t):
 *   φ     — roll  (rotation about X-axis)
 *   θ     — pitch (rotation about Y-axis)
 *   z_t   — vertical height of table centre
 *
 * Rotation convention (consistent with ball dynamics):
 *   R = Ry(θ) · Rx(φ)
 *
 * Two FK implementations are provided:
 *   NewtonRaphson      — iterative, numerically robust
 *   YouTubeClosedForm  — non-iterative quadratic formula
 *
 * IK is exact per-arm and identical for both models.
 *
 * Arm index convention:
 *   i = 0: ψ = 0°,   ground point on +X axis
 *   i = 1: ψ = 120°
 *   i = 2: ψ = 240°
 *
 * @see Table Dynamics Equations - Ball Balancing System.md
 * @see src/physics/table_kinematics.cpp
 */

namespace ball_balancer {

// ============================================================================
// ServoAngles
// ============================================================================

/**
 * @brief Servo angle triple for the three arms.
 *
 * alpha[i] — servo angle αᵢ (rad), measured from horizontal.
 * Positive angle means the elbow is above the ground mounting point.
 */
struct ServoAngles {
    std::array<double, 3> alpha{45.0, 45.0, 45.0};
};

// ============================================================================
// ElbowAngles
// ============================================================================

/**
 * @brief Upper-link angle triple for the three arms.
 *
 * beta[i] — upper-link angle βᵢ (rad), measured from horizontal in the arm's
 *            radial plane. Used to compute T_i from E_i without re-solving FK.
 */
struct ElbowAngles {
    std::array<double, 3> beta{45.0, 45.0, 45.0};
};

// ============================================================================
// FKResult
// ============================================================================

/**
 * @brief Result of a successful forward-kinematics solve.
 */
struct FKResult {
    double phi{0.0};    ///< Table roll φ (rad)
    double theta{0.0};  ///< Table pitch θ (rad)
    double z_t{0.0};    ///< Table height (m)
    ElbowAngles elbow;  ///< Upper-link angles βᵢ (rad), derived from the FK solution
};

// ============================================================================
// FKMethod
// ============================================================================

/**
 * @brief Selects which forward-kinematics algorithm to use.
 */
enum class FKMethod {
    /**
     * Iterative 3×3 Newton-Raphson on the exact constraint equations.
     * Convergence in ~3–5 iterations when warm-started from the previous
     * timestep. Falls back gracefully near singularities.
     */
    NewtonRaphson,

    /**
     * Closed-form solution via quadratic formula (YouTube model).
     * No iterations required. The final pose inversion uses the small-angle
     * linear formula; accuracy degrades slightly at large tilt angles (> ~15°).
     */
    YouTubeClosedForm,

    /**
     * Geometry-based β-solver: Newton-Raphson on the three rigid-body chord-length
     * constraint equations to solve for the upper-link angles β₁,β₂,β₃ directly.
     *
     * f₀: ρ₀²+ρ₁²+ρ₀ρ₁+(h₀−h₁)²−3Rt² = 0
     * f₁: ρ₀²+ρ₂²+ρ₀ρ₂+(h₀−h₂)²−3Rt² = 0
     * f₂: ρ₁²+ρ₂²+ρ₁ρ₂+(h₁−h₂)²−3Rt² = 0
     *
     * where ρᵢ = Aᵢ − L2·cosβᵢ  and  hᵢ = Bᵢ + L2·sinβᵢ.
     *
     * Pose is then extracted via exact closed-form arcsin formulae.
     * Warm-started from the previous ElbowAngles; converges in ≤5 iterations.
     */
    GeometryBased,
};

// ============================================================================
// TableKinematics
// ============================================================================

/**
 * @brief Kinematics of the 3-arm parallel table mechanism.
 *
 * Stateless: all simulation state lives in the caller (Simulator / StateVector).
 * Thread-safe: all public methods are const.
 *
 * Quick-start:
 * @code
 *   SystemParameters params;
 *   params.arm_L1 = 0.08;  params.arm_L2 = 0.08;
 *   params.arm_Rg = 0.10;  params.arm_Rt = 0.07;
 *   params.arm_z_nominal = 0.12;
 *   TableKinematics kin(params);
 *
 *   // IK: desired pose → servo angles
 *   auto s = kin.inverseKinematics(0.05, -0.03, 0.12);
 *
 *   // FK: servo angles → table pose (Newton-Raphson, warm-started)
 *   auto pose = kin.forwardKinematics(s.value(),
 *                   FKMethod::NewtonRaphson, prev_phi, prev_theta, prev_z);
 * @endcode
 */
class TableKinematics {
public:
    /// Maximum Newton-Raphson iterations.
    static constexpr int    NR_MAX_ITER = 20;
    /// Convergence criterion: L∞ norm of residual vector.
    static constexpr double NR_TOL      = 1e-10;
    /// Number of arms.
    static constexpr int    N_ARMS      = 3;

    // Precomputed arm azimuth angles ψᵢ = 2π(i)/3
    static constexpr double PSI[3] = {
        0.0,
        2.0 * M_PI / 3.0,
        4.0 * M_PI / 3.0
    };

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * @brief Construct from system parameters.
     *
     * Reads arm_L1, arm_L2, arm_Rg, arm_Rt, arm_z_nominal from params.
     * Asserts that all lengths are positive.
     *
     * @param params  System parameters with mechanism geometry fields set.
     */
    explicit TableKinematics(const SystemParameters& params);

    // -------------------------------------------------------------------------
    // Inverse Kinematics
    // -------------------------------------------------------------------------

    /**
     * @brief Compute servo angles from a desired table pose.
     *
     * Exact per-arm solution (elbow-down branch):
     * @code
     *   αᵢ = atan2(d_z,i, d_r,i) + arccos(Cᵢ / Dᵢ)
     *   where
     *     d_r,i = (Tᵢ − Gᵢ) · r̂ᵢ    (radial offset in arm plane)
     *     d_z,i = Tᵢ.z               (vertical offset)
     *     Dᵢ    = sqrt(d_r² + d_z²)
     *     Cᵢ    = (d_r² + d_z² + L1² − L2²) / (2 L1)
     * @endcode
     *
     * Returns std::nullopt if any arm cannot reach its target:
     *   |L1 − L2| > Dᵢ   or   Dᵢ > L1 + L2
     *
     * @param phi    Roll angle φ (rad)
     * @param theta  Pitch angle θ (rad)
     * @param z_t    Table centre height (m)
     * @return Servo angles, or nullopt if any arm is out of reach.
     */
    std::optional<ServoAngles> inverseKinematics(
        double phi, double theta, double z_t) const;

    // -------------------------------------------------------------------------
    // Forward Kinematics
    // -------------------------------------------------------------------------

    /**
     * @brief Compute table pose from servo angles.
     *
     * @param servos  Three servo angles αᵢ (rad).
     * @param prev    Previous elbow angles for warm-starting Newton-Raphson.
     *                Pass default-constructed ElbowAngles{} on first call.
     * @param method  FK algorithm: NewtonRaphson (default) or YouTubeClosedForm.
     * @return FKResult with pose and derived elbow angles, or nullopt on failure.
     *         Failure means:
     *           NewtonRaphson    — did not converge within NR_MAX_ITER
     *           YouTubeClosedForm — negative discriminant (geometrically impossible)
     */
    std::optional<FKResult> forwardKinematics(
        const ServoAngles& servos,
        const ElbowAngles& prev   = ElbowAngles{},
        FKMethod           method = FKMethod::NewtonRaphson) const;

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    double L1()       const { return L1_; }
    double L2()       const { return L2_; }
    double Rg()       const { return Rg_; }
    double Rt()       const { return Rt_; }
    double zNominal() const { return zNominal_; }

    // -------------------------------------------------------------------------
    // Geometry point accessors (public — used by renderer and tests)
    // -------------------------------------------------------------------------

    /**
     * @brief Table attachment point Tᵢ in the inertial frame.
     *
     * T_i = [0, 0, z_t]^T + R(φ,θ) * Rt * [cos(ψᵢ), sin(ψᵢ), 0]^T
     *
     * where R = Ry(θ) · Rx(φ).
     */
    std::array<double, 3> tableAttachPoint(
        int i, double phi, double theta, double z_t) const;

    /**
     * @brief Table attachment point Tᵢ derived from elbow geometry.
     *
     * Computes T_i = E_i + L2 * [cos(ψᵢ)·cos(β), sin(ψᵢ)·cos(β), sin(β)]
     * where β is the upper-link angle from horizontal. Used in Servo mode for
     * FK-consistent rendering without re-solving the full FK.
     *
     * @param i      Arm index (0, 1, 2)
     * @param alpha  Servo angle αᵢ (rad)
     * @param beta   Upper-link angle βᵢ (rad), from FK solve
     */
    std::array<double, 3> tableAttachPointFromBeta(
        int i, double alpha, double beta) const;

    /**
     * @brief Elbow position Eᵢ in the inertial frame.
     *
     * E_i = [(Rg + L1·cosα)·cos(ψᵢ), (Rg + L1·cosα)·sin(ψᵢ), L1·sinα]^T
     */
    std::array<double, 3> elbowPosition(int i, double alpha) const;

    /**
     * @brief Ground attachment point Gᵢ in the inertial frame.
     *
     * G_i = [Rg·cos(ψᵢ), Rg·sin(ψᵢ), 0]^T
     */
    std::array<double, 3> groundPoint(int i) const;

private:

    // -------------------------------------------------------------------------
    // Per-arm IK solver
    // -------------------------------------------------------------------------

    /**
     * @brief Solve one arm's IK given 2D coordinates in the radial plane.
     *
     * @param d_r   Radial offset from ground point to table attachment (m)
     * @param d_z   Vertical offset from ground point to table attachment (m)
     * @return Servo angle α (rad) on the elbow-down branch, or nullopt if
     *         the target is out of reach.
     */
    std::optional<double> solveArmIK(double d_r, double d_z) const;

    // -------------------------------------------------------------------------
    // FK implementations
    // -------------------------------------------------------------------------

    /**
     * @brief FK by iterative Newton-Raphson on the 3×3 constraint system.
     */
    std::optional<std::array<double, 3>> fkNewtonRaphson(
        const ServoAngles& servos,
        double phi0, double theta0, double z0) const;

    /**
     * @brief FK by closed-form quadratic formula (YouTube model).
     *
     * Computes each arm's ball-joint Z height analytically, then recovers
     * (φ, θ, z_t) from the small-angle linear inverse:
     *   z_t = (z1+z2+z3)/3
     *   θ   = (z2+z3−2z1) / (3 Rt)
     *   φ   = (z2−z3) / (√3 Rt)
     */
    std::optional<std::array<double, 3>> fkYouTube(
        const ServoAngles& servos) const;

    /**
     * @brief FK by Newton-Raphson on the three β-angle chord-length constraints.
     *
     * Solves for β₀, β₁, β₂ (upper-link angles) such that the three table
     * attachment points satisfy ‖Tᵢ − Tⱼ‖² = 3Rt² for all pairs.
     * Warm-started from prevBeta for fast convergence.
     *
     * @param servos   Current servo angles αᵢ.
     * @param prevBeta Upper-link angles from the previous timestep (warm-start).
     * @return Array {φ, θ, z_t} derived from solved β, or nullopt on failure.
     */
    std::optional<std::array<double, 3>> fkGeometryBased(
        const ServoAngles& servos,
        const ElbowAngles& prevBeta,
        ElbowAngles&       outBeta) const;

    /**
     * @brief Residuals of the β chord-length constraint system.
     *
     * @param beta  Current β estimate [β₀, β₁, β₂].
     * @param A     Per-arm radial elbow offsets: Aᵢ = Rg + L1·cos(αᵢ).
     * @param B     Per-arm elbow heights:         Bᵢ = L1·sin(αᵢ).
     * @return Residual vector [f₀, f₁, f₂].
     */
    std::array<double, 3> betaResiduals(
        const std::array<double, 3>& beta,
        const std::array<double, 3>& A,
        const std::array<double, 3>& B) const;

    /**
     * @brief Jacobian of the β residuals: J[i][j] = ∂fᵢ/∂βⱼ.
     *
     * @param beta  Current β estimate [β₀, β₁, β₂].
     * @param A     Per-arm radial elbow offsets.
     * @param B     Per-arm elbow heights.
     * @return 3×3 Jacobian matrix.
     */
    std::array<std::array<double, 3>, 3> betaJacobian(
        const std::array<double, 3>& beta,
        const std::array<double, 3>& A,
        const std::array<double, 3>& B) const;

    // -------------------------------------------------------------------------
    // Newton-Raphson residual and Jacobian
    // -------------------------------------------------------------------------

    /**
     * @brief Constraint residuals fᵢ = ||Tᵢ(φ,θ,z_t) − Eᵢ(αᵢ)||² − L2²
     */
    std::array<double, 3> residuals(
        const ServoAngles& servos,
        double phi, double theta, double z_t) const;

    /**
     * @brief 3×3 Jacobian J_{ij} = ∂fᵢ/∂qⱼ, q = (φ, θ, z_t).
     *
     * ∂fᵢ/∂qⱼ = 2 (Tᵢ − Eᵢ) · ∂Tᵢ/∂qⱼ
     *
     * ∂Tᵢ/∂z_t  = [0, 0, 1]^T
     * ∂Tᵢ/∂φ    = (∂R/∂φ) · Pᵢᵀ
     * ∂Tᵢ/∂θ    = (∂R/∂θ) · Pᵢᵀ
     */
    std::array<std::array<double, 3>, 3> jacobian(
        const ServoAngles& servos,
        double phi, double theta, double z_t) const;

    // -------------------------------------------------------------------------
    // Linear solver
    // -------------------------------------------------------------------------

    /**
     * @brief Solve A·x = b for a dense 3×3 system using Cramer's rule.
     *
     * Returns nullopt if |det(A)| < 1e-14 (singular or near-singular).
     * Suitable for the Newton-Raphson inner loop; no heap allocation.
     */
    static std::optional<std::array<double, 3>> solve3x3(
        const std::array<std::array<double, 3>, 3>& A,
        const std::array<double, 3>&                b);

    // -------------------------------------------------------------------------
    // Members
    // -------------------------------------------------------------------------

    double L1_;        ///< Lower link length (m)
    double L2_;        ///< Upper link length (m)
    double Rg_;        ///< Ground mounting radius (m)
    double Rt_;        ///< Table mounting radius (m)
    double zNominal_;  ///< Default FK warm-start height (m)
};

} // namespace ball_balancer
