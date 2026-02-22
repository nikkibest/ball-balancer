/**
 * @file physics_test.cpp
 * @brief Unit tests for physics simulation components
 *
 * Tests cover:
 * - RK4 integration accuracy (T-6)
 * - Boundary enforcement and ball bounce (T-7)
 * - Servo dynamics and actuation limits (T-14)
 */

#include <gtest/gtest.h>
#include "ball_balancer/physics/simulator.hpp"
#include "ball_balancer/core/types.hpp"
#include <cmath>

// ============================================================================
// T-6: RK4 Integration Accuracy Tests
// ============================================================================

/**
 * Test RK4 integration accuracy against known analytical solution.
 *
 * For a ball at rest on a tilted table with theta_x = 0.1 rad,
 * the ball should accelerate down the slope according to:
 * a_x = g * sin(theta) / (1 + I/(m*r^2))
 *
 * For a solid sphere, I = 2/5 * m * r^2, so:
 * a_x = (5/7) * g * sin(theta)
 */
TEST(PhysicsSimulatorRK4, AccelerationOnTiltedTable) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);

    // Initial state: ball at center, table tilted in X direction
    StateVector state = StateVector::Zero();
    state(state_index::THETA_X) = 0.1;  // 0.1 rad tilt

    // Zero control input
    ControlVector control = ControlVector::Zero();
    control(control_index::THETA_X) = 0.1;  // Hold table at 0.1 rad

    // Step forward in time
    constexpr double dt = 0.001;  // 1ms timestep
    StateVector new_state = sim.step(state, control, dt);

    // Compute expected acceleration for solid sphere on incline
    // a = (5/7) * g * sin(theta)
    const double expected_accel = (5.0 / 7.0) * params.gravity * std::sin(0.1);

    // After dt, velocity should be approximately a * dt
    // (for small dt, RK4 should be very accurate)
    const double expected_vx = expected_accel * dt;
    const double actual_vx = new_state(state_index::VX);

    // RK4 should be accurate to within 1% for this simple case
    EXPECT_NEAR(actual_vx, expected_vx, std::abs(expected_vx) * 0.01)
        << "RK4 integration should accurately compute ball acceleration on incline";

    // Ball should move in negative X direction (down the slope)
    EXPECT_LT(actual_vx, 0.0)
        << "Ball should accelerate down the slope (negative X)";
}

/**
 * Test RK4 energy conservation over multiple steps.
 *
 * Without friction and with constant table angle, total mechanical energy
 * (kinetic + potential) should be approximately conserved.
 */
TEST(PhysicsSimulatorRK4, EnergyConservation) {
    SystemParameters params;
    params.initialize();

    // Disable friction for energy conservation test
    const double original_friction = params.ball_friction;
    params.ball_friction = 0.0;

    PhysicsSimulator sim(params);

    // Initial state: ball at rest, table tilted
    StateVector state = StateVector::Zero();
    state(state_index::THETA_X) = 0.1;

    // Compute initial energy (just potential from height)
    // h = x * sin(theta) (approximately, for small theta)
    double initial_pe = params.ball_mass * params.gravity * state(state_index::X) * std::sin(0.1);
    double initial_ke = 0.0;  // At rest
    double initial_energy = initial_pe + initial_ke;

    // Step forward 100ms
    ControlVector control = ControlVector::Zero();
    control(control_index::THETA_X) = 0.1;

    for (int i = 0; i < 100; ++i) {
        state = sim.step(state, control, 0.001);
    }

    // Compute final energy
    double final_pe = params.ball_mass * params.gravity * state(state_index::X) * std::sin(0.1);
    double final_ke = 0.5 * params.ball_mass * (
        state(state_index::VX) * state(state_index::VX) +
        state(state_index::VY) * state(state_index::VY)
    );
    double final_energy = final_pe + final_ke;

    // Energy should be conserved to within a few percent
    // (some numerical error expected over 100 steps)
    double energy_error = std::abs(final_energy - initial_energy) / std::max(std::abs(initial_energy), 1e-9);

    EXPECT_LT(energy_error, 0.05)
        << "Energy should be conserved to within 5% over 100 RK4 steps (no friction)";

    // Restore friction
    params.ball_friction = original_friction;
}

/**
 * Test that RK4 produces smooth, continuous trajectories.
 */
TEST(PhysicsSimulatorRK4, SmoothTrajectory) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);

    StateVector state = StateVector::Zero();
    state(state_index::THETA_X) = 0.1;

    ControlVector control = ControlVector::Zero();
    control(control_index::THETA_X) = 0.1;

    // Step forward and check for discontinuities
    double prev_x = state(state_index::X);
    double prev_vx = state(state_index::VX);

    for (int i = 0; i < 50; ++i) {
        state = sim.step(state, control, 0.001);

        // Position change should match velocity
        double dx = state(state_index::X) - prev_x;
        double expected_dx = state(state_index::VX) * 0.001;

        // Should be within 10% (accounts for acceleration during step)
        EXPECT_NEAR(dx, expected_dx, std::abs(expected_dx) * 0.1)
            << "Position change should be consistent with velocity at step " << i;

        // Velocity should change smoothly (no jumps)
        double dvx = state(state_index::VX) - prev_vx;
        EXPECT_LT(std::abs(dvx), 0.1)  // Max 0.1 m/s change per ms is reasonable
            << "Velocity should change smoothly without discontinuities at step " << i;

        prev_x = state(state_index::X);
        prev_vx = state(state_index::VX);
    }
}

// ============================================================================
// T-7: Boundary Enforcement and Ball Bounce Tests
// ============================================================================

/**
 * Test that ball bounces when hitting table edge.
 *
 * Ball should reverse velocity (with coefficient of restitution) when
 * position exceeds table bounds.
 */
TEST(PhysicsSimulatorBoundary, BallBouncesAtEdge) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);

    // Position ball near edge, moving toward boundary
    StateVector state = StateVector::Zero();
    state(state_index::X) = params.table_size - 0.001;  // Very close to edge
    state(state_index::VX) = 1.0;  // Moving toward boundary

    ControlVector control = ControlVector::Zero();

    // Step forward - ball should hit boundary and bounce
    StateVector new_state = sim.step(state, control, 0.01);

    // After bounce, velocity should reverse and be reduced
    EXPECT_LT(new_state(state_index::VX), 0.0)
        << "Ball velocity should reverse after hitting boundary";

    EXPECT_LT(std::abs(new_state(state_index::VX)), 1.0)
        << "Ball velocity should be reduced after bounce (coefficient of restitution)";

    // Position should be clamped within bounds
    EXPECT_LE(std::abs(new_state(state_index::X)), params.table_size)
        << "Ball position should stay within table bounds";
}

/**
 * Test that ball stays within bounds when at rest at edge.
 */
TEST(PhysicsSimulatorBoundary, BallStaysWithinBounds) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);

    // Position ball exactly at boundary
    StateVector state = StateVector::Zero();
    state(state_index::X) = params.table_size;
    state(state_index::VX) = 0.0;  // At rest

    ControlVector control = ControlVector::Zero();

    // Step forward multiple times
    for (int i = 0; i < 10; ++i) {
        state = sim.step(state, control, 0.001);

        EXPECT_LE(std::abs(state(state_index::X)), params.table_size)
            << "Ball should stay within bounds at step " << i;
        EXPECT_LE(std::abs(state(state_index::Y)), params.table_size)
            << "Ball should stay within bounds at step " << i;
    }
}

/**
 * Test corner bounce (both X and Y boundaries).
 */
TEST(PhysicsSimulatorBoundary, CornerBounce) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);

    // Position ball near corner, moving toward it
    StateVector state = StateVector::Zero();
    state(state_index::X) = params.table_size - 0.001;
    state(state_index::Y) = params.table_size - 0.001;
    state(state_index::VX) = 0.5;
    state(state_index::VY) = 0.5;

    ControlVector control = ControlVector::Zero();

    // Step forward - ball should hit corner and bounce
    StateVector new_state = sim.step(state, control, 0.01);

    // Both velocities should reverse
    EXPECT_LT(new_state(state_index::VX), 0.0)
        << "VX should reverse after corner bounce";
    EXPECT_LT(new_state(state_index::VY), 0.0)
        << "VY should reverse after corner bounce";

    // Position should stay within bounds
    EXPECT_LE(std::abs(new_state(state_index::X)), params.table_size);
    EXPECT_LE(std::abs(new_state(state_index::Y)), params.table_size);
}

// ============================================================================
// T-14: Servo Dynamics and Actuation Limits Tests
// ============================================================================

/**
 * Test that servo dynamics enforce maximum angular velocity.
 */
TEST(PhysicsSimulatorServo, MaxAngularVelocityLimit) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);

    StateVector state = StateVector::Zero();

    // Command table to maximum angle instantly
    ControlVector control = ControlVector::Zero();
    control(control_index::THETA_X) = params.max_table_angle;

    // Step forward - servo should rate-limit the motion
    StateVector new_state = sim.step(state, control, 0.001);

    // Angle should increase, but not exceed max velocity constraint
    double d_theta = new_state(state_index::THETA_X) - state(state_index::THETA_X);
    double angular_velocity = d_theta / 0.001;

    // Allow 10% margin for numerical tolerance
    EXPECT_LE(std::abs(angular_velocity), params.max_table_angular_velocity * 1.1)
        << "Servo should enforce max angular velocity limit";
}

/**
 * Test that servo tracks commanded angle with appropriate time constant.
 */
TEST(PhysicsSimulatorServo, FirstOrderDynamics) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);

    StateVector state = StateVector::Zero();

    // Command step input
    ControlVector control = ControlVector::Zero();
    const double target_angle = 0.05;  // 0.05 rad target
    control(control_index::THETA_X) = target_angle;

    // Simulate for several time constants
    const double time_constant = params.servo_time_constant;
    const double dt = 0.001;
    const int steps = static_cast<int>(5.0 * time_constant / dt);  // 5 time constants

    for (int i = 0; i < steps; ++i) {
        state = sim.step(state, control, dt);
    }

    // After 5 time constants, should be within 1% of target (e^-5 ≈ 0.0067)
    EXPECT_NEAR(state(state_index::THETA_X), target_angle, target_angle * 0.01)
        << "Servo should reach ~99% of target after 5 time constants";
}

/**
 * Test that servo respects maximum table angle limits.
 */
TEST(PhysicsSimulatorServo, MaxAngleLimit) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);

    StateVector state = StateVector::Zero();

    // Command angle far beyond limits
    ControlVector control = ControlVector::Zero();
    control(control_index::THETA_X) = 10.0 * params.max_table_angle;  // 10x the limit
    control(control_index::THETA_Y) = -10.0 * params.max_table_angle;

    // Simulate for a long time
    for (int i = 0; i < 1000; ++i) {
        state = sim.step(state, control, 0.001);
    }

    // Angle should be clamped to limits
    EXPECT_LE(std::abs(state(state_index::THETA_X)), params.max_table_angle)
        << "Table angle should not exceed max_table_angle limit";
    EXPECT_LE(std::abs(state(state_index::THETA_Y)), params.max_table_angle)
        << "Table angle should not exceed max_table_angle limit";
}

/**
 * Test servo independence (X and Y axes don't interfere).
 */
TEST(PhysicsSimulatorServo, AxisIndependence) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);

    StateVector state = StateVector::Zero();

    // Command only X axis
    ControlVector control = ControlVector::Zero();
    control(control_index::THETA_X) = 0.1;
    control(control_index::THETA_Y) = 0.0;

    // Step forward
    StateVector new_state = sim.step(state, control, 0.01);

    // X should change, Y should remain near zero
    EXPECT_GT(new_state(state_index::THETA_X), 0.0)
        << "THETA_X should increase when commanded";
    EXPECT_NEAR(new_state(state_index::THETA_Y), 0.0, 1e-9)
        << "THETA_Y should remain zero when not commanded";
}

// ============================================================================
// System Parameters Tests
// ============================================================================

/**
 * Test T-8: SystemParameters::initialize() contract.
 *
 * Verify that initialize() correctly computes ball_inertia from ball_mass
 * and ball_radius.
 */
TEST(SystemParameters, InitializeComputesInertia) {
    SystemParameters params;

    // Set custom ball parameters
    params.ball_mass = 0.1;      // 100g
    params.ball_radius = 0.025;  // 2.5cm
    params.ball_inertia = 0.0;   // Clear it

    // Call initialize
    params.initialize();

    // Verify inertia is computed correctly for solid sphere: I = (2/5) * m * r^2
    const double expected_inertia = 0.4 * params.ball_mass * params.ball_radius * params.ball_radius;

    EXPECT_DOUBLE_EQ(params.ball_inertia, expected_inertia)
        << "initialize() should compute ball_inertia = 0.4 * mass * radius^2";
}

/**
 * Test that default constructor calls initialize().
 */
TEST(SystemParameters, ConstructorCallsInitialize) {
    SystemParameters params;

    // Default constructor should have called initialize()
    // so ball_inertia should be non-zero
    EXPECT_GT(params.ball_inertia, 0.0)
        << "Default constructor should call initialize() to compute ball_inertia";

    // Verify it matches expected value
    const double expected_inertia = 0.4 * params.ball_mass * params.ball_radius * params.ball_radius;
    EXPECT_DOUBLE_EQ(params.ball_inertia, expected_inertia);
}
