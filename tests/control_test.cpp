#include <ball_balancer/control/pid_controller.hpp>
#include <ball_balancer/control/state_estimator.hpp>
#include <gtest/gtest.h>
#include <cmath>

using namespace ball_balancer;

/**
 * @file control_test.cpp
 * @brief Tests for PID controller and Kalman filter
 *
 * Includes critical regression test for Kalman axis mismatch bug (fixed in Phase 1).
 */

// ============================================================================
// Regression Test: Kalman Filter Axis Mismatch
// ============================================================================

/**
 * Critical regression test for the Kalman filter axis mismatch.
 *
 * BUG (before fix): StateEstimator A-matrix had VX driven by THETA_Y instead of VARPHI_X
 * FIX: Changed Ac(state_index::VX, state_index::THETA_Y) to Ac(state_index::VX, state_index::VARPHI_X)
 *
 * This test verifies that VX increases when varphi_x > 0 and theta_y == 0.
 */
TEST(StateEstimatorAxisMismatch, VXDrivenByThetaXNotThetaY) {
    SystemParameters params;
    params.initialize();

    StateEstimator estimator(params);

    // Initialize with only X-axis tilt (no Y-axis tilt)
    StateVector init = StateVector::Zero();
    init(state_index::VARPHI_X) = 0.1;  // 0.1 rad tilt in X
    init(state_index::THETA_Y) = 0.0;  // No tilt in Y

    estimator.reset(init);

    // Run prediction step (no control input)
    estimator.predict(ControlVector::Zero());

    const StateVector& x_hat = estimator.get_state();

    // With positive varphi_x, ball should roll in +X direction (VX increases)
    EXPECT_GT(x_hat(state_index::VX), 0.0)
        << "BUG: VX must increase when varphi_x > 0 (axis mismatch if this fails)";

    // With zero theta_y, VY should remain near zero
    EXPECT_NEAR(x_hat(state_index::VY), 0.0, 1e-9)
        << "VY must remain zero when theta_y == 0";
}

/**
 * Test Y-axis dynamics independently.
 */
TEST(StateEstimatorAxisMismatch, VYDrivenByThetaYNotThetaX) {
    SystemParameters params;
    params.initialize();

    StateEstimator estimator(params);

    // Initialize with only Y-axis tilt
    StateVector init = StateVector::Zero();
    init(state_index::VARPHI_X) = 0.0;
    init(state_index::THETA_Y) = 0.1;  // 0.1 rad tilt in Y

    estimator.reset(init);
    estimator.predict(ControlVector::Zero());

    const StateVector& x_hat = estimator.get_state();

    EXPECT_GT(x_hat(state_index::VY), 0.0)
        << "VY must increase when theta_y > 0";

    EXPECT_NEAR(x_hat(state_index::VX), 0.0, 1e-9)
        << "VX must remain zero when varphi_x == 0";
}

// ============================================================================
// PID Controller Tests
// ============================================================================

/**
 * Test PID output clamping.
 */
TEST(PIDAxis, OutputClampingWorks) {
    PIDGains gains;
    gains.Kp = 10.0f;  // High gain to ensure saturation
    gains.Ki = 0.0f;
    gains.Kd = 0.0f;

    const double out_min = -1.0;
    const double out_max = 1.0;

    PIDAxis pid(gains, 0.01, out_min, out_max);

    // Large error should saturate output
    double output = pid.compute(10.0, 0.0);  // error = 10.0
    EXPECT_LE(output, out_max) << "Output must not exceed max limit";
    EXPECT_GE(output, out_min) << "Output must not fall below min limit";
    EXPECT_DOUBLE_EQ(output, out_max) << "With high Kp, output should saturate at max";

    // Negative error
    output = pid.compute(-10.0, 0.0);
    EXPECT_DOUBLE_EQ(output, out_min) << "Negative error should saturate at min";
}

/**
 * Test derivative-on-measurement (no derivative kick on setpoint changes).
 */
TEST(PIDAxis, NoDerivativeKickOnSetpointChange) {
    PIDGains gains;
    gains.Kp = 0.0f;  // Disable P
    gains.Ki = 0.0f;  // Disable I
    gains.Kd = 1.0f;  // Only D term active

    PIDAxis pid(gains, 0.01, -100.0, 100.0);

    // Initialize at setpoint
    pid.compute(0.0, 0.0);

    // Step setpoint but keep measurement constant
    double output = pid.compute(10.0, 0.0);

    // D-term should be zero because measurement didn't change
    EXPECT_NEAR(output, 0.0, 1e-9)
        << "Derivative kick: D-term must be 0 when measurement is unchanged";
}

/**
 * Test PID reset functionality.
 */
TEST(PIDAxis, ResetClearsInternalState) {
    PIDGains gains;
    gains.Kp = 1.0f;
    gains.Ki = 1.0f;
    gains.Kd = 0.1f;

    PIDAxis pid(gains, 0.01, -10.0, 10.0);

    // Run several iterations to accumulate integral
    for (int i = 0; i < 100; ++i) {
        pid.compute(1.0, 0.0);  // Constant error
    }

    // Reset
    pid.reset();

    // After reset, first compute should behave like initialization
    // (only P term active, I and D are zero)
    double output = pid.compute(1.0, 0.0);
    EXPECT_NEAR(output, gains.Kp * 1.0, 1e-6)
        << "After reset, only P term should be active on first call";
}

// ============================================================================
// State Estimator Basic Tests
// ============================================================================

TEST(StateEstimator, InitializationFromParameters) {
    SystemParameters params;
    params.initialize();

    StateEstimator estimator(params);

    // Initial state should be zero
    const StateVector& x_hat = estimator.get_state();
    for (int i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(x_hat(i), 0.0) << "Initial state element " << i << " should be zero";
    }
}

TEST(StateEstimator, ResetSetsState) {
    SystemParameters params;
    params.initialize();

    StateEstimator estimator(params);

    StateVector new_state;
    new_state << 0.1, 0.2, 0.3, 0.4, 0.05, 0.06;

    estimator.reset(new_state);

    const StateVector& x_hat = estimator.get_state();
    for (int i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(x_hat(i), new_state(i))
            << "State element " << i << " should match reset value";
    }
}

/**
 * Test that prediction step advances state forward in time.
 */
TEST(StateEstimator, PredictionAdvancesState) {
    SystemParameters params;
    params.initialize();

    StateEstimator estimator(params);

    // Set initial state with non-zero velocity
    StateVector init;
    init << 0.0, 0.0, 0.1, 0.0, 0.0, 0.0;  // vx = 0.1 m/s
    estimator.reset(init);

    // Run prediction
    ControlVector control = ControlVector::Zero();
    estimator.predict(control);

    const StateVector& x_hat = estimator.get_state();

    // Position X should have changed due to velocity integration
    // (exact value depends on dynamics, just verify state changed)
    EXPECT_NE(x_hat(state_index::X), 0.0) << "Position should change after prediction";
}

/**
 * Test that update step incorporates measurements.
 */
TEST(StateEstimator, UpdateIncorporatesMeasurement) {
    SystemParameters params;
    params.camera_noise_std = 0.0;  // No noise for deterministic test
    params.initialize();

    StateEstimator estimator(params);

    // Set initial state with error
    StateVector init;
    init << 0.1, 0.2, 0.0, 0.0, 0.0, 0.0;
    estimator.reset(init);

    // Provide measurement at origin
    MeasurementVector measurement;
    measurement << 0.0, 0.0;

    estimator.update(measurement);

    const StateVector& x_hat = estimator.get_state();

    // State estimate should move closer to measurement
    EXPECT_LT(std::abs(x_hat(state_index::X)), 0.1)
        << "X estimate should move toward measurement";
    EXPECT_LT(std::abs(x_hat(state_index::Y)), 0.2)
        << "Y estimate should move toward measurement";
}

// ============================================================================
// Dual-Axis PID Controller Tests
// ============================================================================

TEST(PIDController, IndependentAxisControl) {
    SystemParameters params;
    params.initialize();

    PIDController controller(params);

    // Set gains for X axis only
    PIDGains x_gains;
    x_gains.Kp = 2.0f;
    x_gains.Ki = 0.0f;
    x_gains.Kd = 0.0f;
    controller.set_x_gains(x_gains);

    // Zero gains for Y axis
    PIDGains y_gains;
    y_gains.Kp = 0.0f;
    y_gains.Ki = 0.0f;
    y_gains.Kd = 0.0f;
    controller.set_y_gains(y_gains);

    // State with X offset only
    StateVector state = StateVector::Zero();
    state(state_index::X) = 0.5;

    Eigen::Vector2d setpoint(0.0, 0.0);

    ControlVector control = controller.compute(setpoint, state);

    // X control should be non-zero (proportional to error)
    EXPECT_NE(control(control_index::VARPHI_X_CMD), 0.0)
        << "X controller should produce output for X error";

    // Y control should be zero (zero gains)
    EXPECT_DOUBLE_EQ(control(control_index::THETA_Y_CMD), 0.0)
        << "Y controller should produce zero output with zero gains";
}

TEST(PIDController, ResetBothAxes) {
    SystemParameters params;
    params.initialize();

    PIDController controller(params);

    // Run some iterations to accumulate state
    StateVector state = StateVector::Zero();
    state(state_index::X) = 1.0;
    state(state_index::Y) = 1.0;

    Eigen::Vector2d setpoint(0.0, 0.0);

    for (int i = 0; i < 100; ++i) {
        controller.compute(setpoint, state);
    }

    // Reset
    controller.reset();

    // After reset, controller should behave as if just initialized
    ControlVector control = controller.compute(setpoint, state);

    // Exact values depend on gains, but verify control is bounded
    EXPECT_TRUE(std::isfinite(control(0)));
    EXPECT_TRUE(std::isfinite(control(1)));
}
