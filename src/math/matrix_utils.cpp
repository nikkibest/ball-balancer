#include <ball_balancer/math/matrix_utils.hpp>
#include <cmath>

/**
 * @file matrix_utils.cpp
 * @brief Implementation of linear algebra utilities
 *
 * CRITICAL Eigen Best Practices followed:
 * 1. NEVER use auto with Eigen expressions - always explicit types or .eval()
 * 2. Use decompositions (LU, QR) for solving, NEVER compute inverse
 * 3. Handle aliasing with .eval() when output appears in input
 * 4. Prefer fixed-size matrices for performance
 * 5. Compile with -O3 -march=native for SIMD
 *
 * @see research/eigen-cpp-linear-algebra-best-practices.md
 */

namespace ball_balancer {
namespace matrix_utils {

bool is_controllable(const SystemMatrix& A, const ControlMatrix& B) {
    constexpr int n = 9;  // State dimension
    constexpr int m = 2;  // Control dimension

    // Construct controllability matrix C = [B AB A²B A³B A⁴B A⁵B]
    // Size: 6 x 12 (n x n*m)
    Eigen::Matrix<double, n, n * m> C;

    // Build controllability matrix column by column
    SystemMatrix A_power = SystemMatrix::Identity();  // A^0 = I
    for (int i = 0; i < n; ++i) {
        // C[:, i*m:(i+1)*m] = A^i * B
        // IMPORTANT: Use .eval() to avoid aliasing issues
        Eigen::Matrix<double, n, m> A_power_B = (A_power * B).eval();
        C.block<n, m>(0, i * m) = A_power_B;

        // Update A_power = A_power * A
        // IMPORTANT: Must use .eval() since A_power appears on both sides
        if (i < n - 1) {
            A_power = (A_power * A).eval();
        }
    }

    // System is controllable if rank(C) = n
    int rank = compute_rank(C);
    return rank == n;
}

bool is_observable(const SystemMatrix& A, const MeasurementMatrix& C) {
    constexpr int n = 9;  // State dimension
    constexpr int p = 2;  // Measurement dimension

    // Construct observability matrix O = [C; CA; CA²; CA³; CA⁴; CA⁵]
    // Size: 12 x 6 (n*p x n)
    Eigen::Matrix<double, n * p, n> O;

    // Build observability matrix row by row
    SystemMatrix A_power = SystemMatrix::Identity();  // A^0 = I
    for (int i = 0; i < n; ++i) {
        // O[i*p:(i+1)*p, :] = C * A^i
        // IMPORTANT: Use .eval() to avoid aliasing
        Eigen::Matrix<double, p, n> C_A_power = (C * A_power).eval();
        O.block<p, n>(i * p, 0) = C_A_power;

        // Update A_power = A * A_power
        if (i < n - 1) {
            A_power = (A * A_power).eval();
        }
    }

    // System is observable if rank(O) = n
    int rank = compute_rank(O);
    return rank == n;
}

LinearizedSystem compute_linearization(
    const StateVector& operating_point,
    const ControlVector& control_point
) {
    // Unused for now - linearization at equilibrium (0,0)
    (void)operating_point;
    (void)control_point;

    LinearizedSystem sys;

    // For ball balancer, we can derive A and B analytically
    // using small-angle approximation: sin(θ) ≈ θ

    // Physical constants (using default parameters)
    SystemParameters params;
    params.initialize();

    const double g = params.gravity;
    const double friction = params.friction_coeff;
    const double tau_servo = params.servo_time_constant;
    constexpr double rolling_factor = 5.0 / 7.0;

    // State: [x, y, z_ball, vx, vy, vz_ball, theta_x, theta_y, z_table]
    //
    // Dynamics (small angle):
    // dx/dt = vx
    // dy/dt = vy
    // dz_ball/dt = vz_ball (stub: 0)
    // dvx/dt = (5/7)*g*theta_x - friction*vx
    // dvy/dt = (5/7)*g*theta_y - friction*vy
    // dvz_ball/dt = 0 (stub)
    // dtheta_x/dt = (theta_x_cmd - theta_x) / tau_servo
    // dtheta_y/dt = (theta_y_cmd - theta_y) / tau_servo
    // dz_table/dt = 0 (stub)

    // A matrix: Jacobian ∂f/∂x (linearized around operating point)
    sys.A.setZero();

    // dx/dt = vx
    sys.A(state_index::X, state_index::VX) = 1.0;

    // dy/dt = vy
    sys.A(state_index::Y, state_index::VY) = 1.0;

    // dvx/dt = (5/7)*g*theta_x - friction*vx
    sys.A(state_index::VX, state_index::THETA_X) = rolling_factor * g;
    sys.A(state_index::VX, state_index::VX) = -friction;

    // dvy/dt = (5/7)*g*theta_y - friction*vy
    sys.A(state_index::VY, state_index::THETA_Y) = rolling_factor * g;
    sys.A(state_index::VY, state_index::VY) = -friction;

    // dtheta_x/dt = (theta_x_cmd - theta_x) / tau_servo
    sys.A(state_index::THETA_X, state_index::THETA_X) = -1.0 / tau_servo;

    // dtheta_y/dt = (theta_y_cmd - theta_y) / tau_servo
    sys.A(state_index::THETA_Y, state_index::THETA_Y) = -1.0 / tau_servo;

    // B matrix: Jacobian ∂f/∂u
    sys.B.setZero();

    // Only servo dynamics affected by control
    sys.B(state_index::THETA_X, control_index::THETA_X_CMD) = 1.0 / tau_servo;
    sys.B(state_index::THETA_Y, control_index::THETA_Y_CMD) = 1.0 / tau_servo;

    // C matrix: Measurement mapping y = C*x
    // We measure ball position (x, y)
    sys.C.setZero();
    sys.C(measurement_index::X_MEAS, state_index::X) = 1.0;
    sys.C(measurement_index::Y_MEAS, state_index::Y) = 1.0;

    // D matrix: No direct feedthrough
    sys.D.setZero();

    return sys;
}

SystemMatrix matrix_power(const SystemMatrix& A, int n) {
    if (n == 0) {
        return SystemMatrix::Identity();
    }

    if (n == 1) {
        return A;
    }

    if (n < 0) {
        // For negative powers, compute inverse and positive power
        // NOTE: Use LU decomposition, not A.inverse()
        Eigen::PartialPivLU<SystemMatrix> lu(A);
        SystemMatrix A_inv = lu.inverse();  // Only safe use of inverse()
        return matrix_power(A_inv, -n);
    }

    // Repeated squaring for efficiency: O(log n)
    if (n % 2 == 0) {
        // A^n = (A^(n/2))^2
        SystemMatrix half_power = matrix_power(A, n / 2);
        // IMPORTANT: Use .eval() to avoid aliasing
        return (half_power * half_power).eval();
    } else {
        // A^n = A * A^(n-1)
        SystemMatrix power_minus_one = matrix_power(A, n - 1);
        return (A * power_minus_one).eval();
    }
}

bool is_positive_definite(const SystemMatrix& M) {
    // Check if matrix is symmetric (required for positive definiteness)
    const double symmetry_tol = 1e-10;
    if (!M.isApprox(M.transpose(), symmetry_tol)) {
        return false;
    }

    // Compute eigenvalues
    // NOTE: Use SelfAdjointEigenSolver for symmetric matrices (faster)
    Eigen::SelfAdjointEigenSolver<SystemMatrix> solver(M);

    if (solver.info() != Eigen::Success) {
        return false;
    }

    // All eigenvalues must be positive
    // NOTE: Use explicit type, not auto!
    Eigen::Matrix<double, 9, 1> eigenvalues = solver.eigenvalues();

    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) <= 0.0) {
            return false;
        }
    }

    return true;
}

void clamp_matrix(SystemMatrix& M, double max_value) {
    // Clamp each element to [-max_value, max_value]
    for (int i = 0; i < M.rows(); ++i) {
        for (int j = 0; j < M.cols(); ++j) {
            if (M(i, j) > max_value) {
                M(i, j) = max_value;
            } else if (M(i, j) < -max_value) {
                M(i, j) = -max_value;
            }
        }
    }
}

} // namespace matrix_utils
} // namespace ball_balancer
