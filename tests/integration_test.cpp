/**
 * @file integration_test.cpp
 * @brief Integration tests for closed-loop ball balancer system
 *
 * Tests cover:
 * - T-10: Closed-loop stability and tracking
 * - Full system integration (Simulator + StateEstimator + DualAxisPIDController)
 * - Setpoint tracking performance
 * - Disturbance rejection
 */

#include <gtest/gtest.h>
#include "ball_balancer/physics/simulator.hpp"
#include "ball_balancer/control/state_estimator.hpp"
#include "ball_balancer/control/dual_axis_pid_controller.hpp"
#include "ball_balancer/core/types.hpp"
#include <cmath>

// ============================================================================
// T-10: Closed-Loop Integration Tests
// ============================================================================

/**
 * Test that closed-loop system can stabilize ball at origin.
 *
 * This is the fundamental test: given arbitrary initial conditions,
 * the controller should bring the ball to rest at (0, 0).
 */
TEST(ClosedLoopIntegration, StabilizeAtOrigin) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);
    StateEstimator estimator(params);
    DualAxisPIDController controller(params);

    // Initial state: ball offset from origin with some velocity
    StateVector state = StateVector::Zero();
    state(state_index::X) = 0.05;   // 5cm offset
    state(state_index::Y) = -0.03;  // 3cm offset (opposite direction)
    state(state_index::VX) = 0.1;   // Moving
    state(state_index::VY) = -0.05; // Moving

    estimator.reset(state);

    // Setpoint: origin
    Eigen::Vector2d setpoint(0.0, 0.0);

    // Closed-loop simulation for 5 seconds
    const double dt = 0.001;
    const int steps = 5000;  // 5 seconds

    for (int i = 0; i < steps; ++i) {
        // Estimate state (in real system, this uses noisy measurements)
        const StateVector& x_hat = estimator.get_state();

        // Compute control
        ControlVector control = controller.compute(x_hat, setpoint, dt);

        // Apply control to simulator
        state = sim.step(state, control, dt);

        // Update estimator (assumes perfect measurement for this test)
        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);
    }

    // After 5 seconds, ball should be very close to origin
    EXPECT_NEAR(state(state_index::X), 0.0, 0.005)
        << "Ball should stabilize near X=0 within 5mm";
    EXPECT_NEAR(state(state_index::Y), 0.0, 0.005)
        << "Ball should stabilize near Y=0 within 5mm";

    // Velocity should be very small
    EXPECT_NEAR(state(state_index::VX), 0.0, 0.01)
        << "Ball should come to rest (VX ~0)";
    EXPECT_NEAR(state(state_index::VY), 0.0, 0.01)
        << "Ball should come to rest (VY ~0)";
}

/**
 * Test setpoint tracking: ball should move to and stabilize at non-zero setpoint.
 */
TEST(ClosedLoopIntegration, TrackNonZeroSetpoint) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);
    StateEstimator estimator(params);
    DualAxisPIDController controller(params);

    // Initial state: ball at origin
    StateVector state = StateVector::Zero();
    estimator.reset(state);

    // Setpoint: off-center position
    Eigen::Vector2d setpoint(0.08, -0.06);  // 8cm, -6cm

    // Closed-loop simulation for 10 seconds
    const double dt = 0.001;
    const int steps = 10000;

    for (int i = 0; i < steps; ++i) {
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);
        state = sim.step(state, control, dt);

        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);
    }

    // Ball should track setpoint within a few mm
    EXPECT_NEAR(state(state_index::X), setpoint.x(), 0.01)
        << "Ball should track setpoint X within 1cm";
    EXPECT_NEAR(state(state_index::Y), setpoint.y(), 0.01)
        << "Ball should track setpoint Y within 1cm";

    // Should be at rest at setpoint
    EXPECT_NEAR(state(state_index::VX), 0.0, 0.02)
        << "Ball should be at rest at setpoint";
    EXPECT_NEAR(state(state_index::VY), 0.0, 0.02)
        << "Ball should be at rest at setpoint";
}

/**
 * Test setpoint change: system should track moving setpoint.
 */
TEST(ClosedLoopIntegration, TrackChangingSetpoint) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);
    StateEstimator estimator(params);
    DualAxisPIDController controller(params);

    StateVector state = StateVector::Zero();
    estimator.reset(state);

    const double dt = 0.001;

    // Phase 1: Track setpoint at (0.05, 0)
    Eigen::Vector2d setpoint(0.05, 0.0);

    for (int i = 0; i < 3000; ++i) {  // 3 seconds
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);
        state = sim.step(state, control, dt);

        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);
    }

    // Should be near first setpoint
    EXPECT_NEAR(state(state_index::X), 0.05, 0.01);

    // Phase 2: Change setpoint to (-0.05, 0.05)
    setpoint = Eigen::Vector2d(-0.05, 0.05);

    for (int i = 0; i < 5000; ++i) {  // 5 more seconds
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);
        state = sim.step(state, control, dt);

        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);
    }

    // Should track new setpoint
    EXPECT_NEAR(state(state_index::X), -0.05, 0.01)
        << "Ball should track new setpoint X";
    EXPECT_NEAR(state(state_index::Y), 0.05, 0.01)
        << "Ball should track new setpoint Y";
}

/**
 * Test system stability: no oscillations or instability at origin.
 */
TEST(ClosedLoopIntegration, NoOscillationsAtOrigin) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);
    StateEstimator estimator(params);
    DualAxisPIDController controller(params);

    // Start very close to origin
    StateVector state = StateVector::Zero();
    state(state_index::X) = 0.001;  // 1mm offset
    estimator.reset(state);

    Eigen::Vector2d setpoint(0.0, 0.0);

    const double dt = 0.001;

    // Settle to equilibrium
    for (int i = 0; i < 2000; ++i) {
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);
        state = sim.step(state, control, dt);

        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);
    }

    // Track position over next 3 seconds to check for oscillations
    double max_deviation = 0.0;
    for (int i = 0; i < 3000; ++i) {
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);
        state = sim.step(state, control, dt);

        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);

        // Track maximum deviation
        double deviation = std::sqrt(state(state_index::X) * state(state_index::X) +
                                    state(state_index::Y) * state(state_index::Y));
        max_deviation = std::max(max_deviation, deviation);
    }

    // Should not oscillate more than 5mm from origin
    EXPECT_LT(max_deviation, 0.005)
        << "System should not oscillate at equilibrium (max deviation < 5mm)";
}

/**
 * Test disturbance rejection: system should recover from external disturbance.
 */
TEST(ClosedLoopIntegration, DisturbanceRejection) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);
    StateEstimator estimator(params);
    DualAxisPIDController controller(params);

    StateVector state = StateVector::Zero();
    estimator.reset(state);

    Eigen::Vector2d setpoint(0.0, 0.0);

    const double dt = 0.001;

    // Stabilize at origin first
    for (int i = 0; i < 2000; ++i) {
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);
        state = sim.step(state, control, dt);

        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);
    }

    // Apply disturbance: sudden velocity impulse
    state(state_index::VX) += 0.3;  // 30 cm/s impulse
    state(state_index::VY) += 0.2;  // 20 cm/s impulse

    // Update estimator with disturbed state
    Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
    estimator.update(measurement);

    // System should recover from disturbance within 5 seconds
    for (int i = 0; i < 5000; ++i) {
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);
        state = sim.step(state, control, dt);

        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);
    }

    // After recovery, should be back near origin
    EXPECT_NEAR(state(state_index::X), 0.0, 0.01)
        << "System should reject disturbance and return to origin";
    EXPECT_NEAR(state(state_index::Y), 0.0, 0.01)
        << "System should reject disturbance and return to origin";
    EXPECT_NEAR(state(state_index::VX), 0.0, 0.02)
        << "Velocity should return to zero after disturbance";
    EXPECT_NEAR(state(state_index::VY), 0.0, 0.02)
        << "Velocity should return to zero after disturbance";
}

/**
 * Test that control output respects actuator limits.
 */
TEST(ClosedLoopIntegration, ControlOutputRespectslimits) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);
    StateEstimator estimator(params);
    DualAxisPIDController controller(params);

    // Extreme initial condition to stress controller
    StateVector state = StateVector::Zero();
    state(state_index::X) = 0.15;   // 15cm - very far from origin
    state(state_index::VX) = 0.5;   // 50cm/s - fast
    estimator.reset(state);

    Eigen::Vector2d setpoint(0.0, 0.0);

    const double dt = 0.001;

    // Run closed loop
    for (int i = 0; i < 1000; ++i) {
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);

        // Control output should always respect limits
        EXPECT_LE(std::abs(control(control_index::THETA_X)), params.max_table_angle)
            << "Control output should not exceed max_table_angle at step " << i;
        EXPECT_LE(std::abs(control(control_index::THETA_Y)), params.max_table_angle)
            << "Control output should not exceed max_table_angle at step " << i;

        state = sim.step(state, control, dt);

        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);
    }
}

/**
 * Test axis decoupling: X and Y axes should be independent.
 */
TEST(ClosedLoopIntegration, AxisDecoupling) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);
    StateEstimator estimator(params);
    DualAxisPIDController controller(params);

    // Initial state: only X-axis offset
    StateVector state = StateVector::Zero();
    state(state_index::X) = 0.05;
    state(state_index::Y) = 0.0;
    estimator.reset(state);

    Eigen::Vector2d setpoint(0.0, 0.0);

    const double dt = 0.001;

    // Run closed loop
    for (int i = 0; i < 3000; ++i) {
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);
        state = sim.step(state, control, dt);

        Eigen::Vector2d measurement(state(state_index::X), state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);

        // Y position should remain very close to zero throughout
        EXPECT_NEAR(state(state_index::Y), 0.0, 0.002)
            << "Y-axis should remain near zero when only X is disturbed at step " << i;
    }

    // X should have stabilized
    EXPECT_NEAR(state(state_index::X), 0.0, 0.01);
}

/**
 * Test estimator convergence: estimator should track true state.
 */
TEST(ClosedLoopIntegration, EstimatorConvergence) {
    SystemParameters params;
    params.initialize();

    PhysicsSimulator sim(params);
    StateEstimator estimator(params);
    DualAxisPIDController controller(params);

    // Initial state: estimator initialized to wrong state
    StateVector true_state = StateVector::Zero();
    true_state(state_index::X) = 0.05;
    true_state(state_index::VX) = 0.1;

    StateVector estimated_state = StateVector::Zero();  // Estimator thinks ball is at origin
    estimator.reset(estimated_state);

    Eigen::Vector2d setpoint(0.0, 0.0);

    const double dt = 0.001;

    // Run closed loop - estimator should converge to true state
    for (int i = 0; i < 2000; ++i) {
        const StateVector& x_hat = estimator.get_state();
        ControlVector control = controller.compute(x_hat, setpoint, dt);
        true_state = sim.step(true_state, control, dt);

        // Provide true measurements to estimator
        Eigen::Vector2d measurement(true_state(state_index::X), true_state(state_index::Y));
        estimator.update(measurement);
        estimator.predict(control);
    }

    // After 2 seconds, estimator should track true state closely
    const StateVector& x_hat = estimator.get_state();

    EXPECT_NEAR(x_hat(state_index::X), true_state(state_index::X), 0.01)
        << "Estimator should converge to true X position";
    EXPECT_NEAR(x_hat(state_index::Y), true_state(state_index::Y), 0.01)
        << "Estimator should converge to true Y position";
    EXPECT_NEAR(x_hat(state_index::VX), true_state(state_index::VX), 0.05)
        << "Estimator should converge to true X velocity";
    EXPECT_NEAR(x_hat(state_index::VY), true_state(state_index::VY), 0.05)
        << "Estimator should converge to true Y velocity";
}
