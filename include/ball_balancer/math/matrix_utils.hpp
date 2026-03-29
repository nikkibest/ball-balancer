#pragma once

#include <ball_balancer/core/types.hpp>
#include <Eigen/Dense>

/**
 * @file matrix_utils.hpp
 * @brief Linear algebra utilities for control systems analysis
 *
 * Provides helper functions for:
 * - Controllability and observability analysis
 * - System linearization
 * - Matrix decompositions and solvers
 *
 * Following best practices from research/eigen-cpp-linear-algebra-best-practices.md:
 * - Never use auto with Eigen expressions
 * - Use fixed-size matrices when dimensions known
 * - Solve with decompositions, not inverse
 * - Handle aliasing with .eval()
 *
 * @see research/eigen-cpp-linear-algebra-best-practices.md
 */

namespace ball_balancer {
namespace matrix_utils {

/**
 * @brief Check if system is controllable
 * @param A System matrix (n x n)
 * @param B Control matrix (n x m)
 * @return true if system is controllable
 *
 * A system is controllable if rank(C) = n, where:
 * C = [B AB A²B ... A^(n-1)B]  (controllability matrix)
 *
 * Uses QR decomposition to compute rank numerically.
 */
[[nodiscard]] bool is_controllable(const SystemMatrix& A, const ControlMatrix& B);

/**
 * @brief Check if system is observable
 * @param A System matrix (n x n)
 * @param C Measurement matrix (p x n)
 * @return true if system is observable
 *
 * A system is observable if rank(O) = n, where:
 * O = [C; CA; CA²; ...; CA^(n-1)]  (observability matrix)
 *
 * Uses QR decomposition to compute rank numerically.
 */
[[nodiscard]] bool is_observable(const SystemMatrix& A, const MeasurementMatrix& C);

/**
 * @brief Linearized system representation
 *
 * Contains A, B, C, D matrices for linear system:
 * dx/dt = A*x + B*u
 * y = C*x + D*u
 */
struct LinearizedSystem {
    SystemMatrix A;          // State matrix (6x6)
    ControlMatrix B;         // Control matrix (6x2)
    MeasurementMatrix C;     // Measurement matrix (2x6)
    FeedthroughMatrix D;     // Feedthrough matrix (2x2)

    /**
     * @brief Default constructor - zero matrices
     */
    LinearizedSystem()
        : A(SystemMatrix::Zero())
        , B(ControlMatrix::Zero())
        , C(MeasurementMatrix::Zero())
        , D(FeedthroughMatrix::Zero())
    {}
};

/**
 * @brief Compute linearized system matrices around operating point
 * @param operating_point State around which to linearize
 * @param control_point Control input at operating point
 * @return Linearized A, B, C, D matrices
 *
 * For ball balancer with small-angle approximation:
 * - A matrix: Jacobian ∂f/∂x
 * - B matrix: Jacobian ∂f/∂u
 * - C matrix: Measurement mapping (constant)
 * - D matrix: Zero (no direct feedthrough)
 *
 * Uses numerical differentiation (finite differences) for Jacobians.
 */
LinearizedSystem compute_linearization(
    const StateVector& operating_point,
    const ControlVector& control_point
);

/**
 * @brief Solve linear system Ax = b using LU decomposition
 * @param A Coefficient matrix
 * @param b Right-hand side vector
 * @return Solution vector x
 *
 * Uses Eigen's PartialPivLU for numerical stability.
 * NEVER use A.inverse() * b - numerically unstable!
 */
template<typename MatrixType, typename VectorType>
VectorType solve_linear_system(const MatrixType& A, const VectorType& b) {
    // Use decomposition for numerical stability
    // NOTE: Explicitly typed to avoid auto with Eigen expressions
    Eigen::PartialPivLU<MatrixType> lu(A);
    VectorType x = lu.solve(b);  // Explicit type, not auto!
    return x;
}

/**
 * @brief Compute matrix power A^n efficiently
 * @param A Matrix to raise to power
 * @param n Power (non-negative integer)
 * @return A^n
 *
 * Uses repeated squaring for efficiency: O(log n) multiplications
 */
SystemMatrix matrix_power(const SystemMatrix& A, int n);

/**
 * @brief Compute rank of matrix using QR decomposition
 * @param M Matrix to analyze
 * @param tolerance Threshold for zero singular values (default: 1e-10)
 * @return Numerical rank of matrix
 *
 * Uses QR decomposition with column pivoting for numerical stability.
 */
template<typename MatrixType>
int compute_rank(const MatrixType& M, double tolerance = 1e-10) {
    // Use ColPivHouseholderQR for rank-revealing decomposition
    Eigen::ColPivHouseholderQR<MatrixType> qr(M);
    qr.setThreshold(tolerance);
    return qr.rank();
}

/**
 * @brief Check if matrix is positive definite
 * @param M Symmetric matrix to check
 * @return true if all eigenvalues are positive
 *
 * Used for checking covariance matrices (Q, R, P) in Kalman filter.
 */
bool is_positive_definite(const SystemMatrix& M);

/**
 * @brief Clamp matrix values to range [-max, max]
 * @param M Matrix to clamp (modified in place)
 * @param max_value Maximum absolute value
 *
 * Useful for numerical stability (prevents overflow in iterations).
 */
void clamp_matrix(SystemMatrix& M, double max_value);

} // namespace matrix_utils
} // namespace ball_balancer
