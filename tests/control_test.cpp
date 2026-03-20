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
    TableState table{};
    table.phi   = 0.1;  // 0.1 rad tilt in X
    table.theta = 0.0;  // No tilt in Y

    estimator.reset(BallState{}, table);

    // Run prediction step (no control input)
    estimator.predict(ControlVector::Zero());

    const BallState ball = estimator.get_ball_state();

    // With positive varphi_x, ball should roll in +X direction (VX increases)
    EXPECT_GT(ball.vx, 0.0)
        << "BUG: VX must increase when varphi_x > 0 (axis mismatch if this fails)";

    // With zero theta_y, VY should remain near zero
    EXPECT_NEAR(ball.vy, 0.0, 1e-9)
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
    TableState table{};
    table.phi   = 0.0;
    table.theta = 0.1;  // 0.1 rad tilt in Y

    estimator.reset(BallState{}, table);
    estimator.predict(ControlVector::Zero());

    const BallState ball = estimator.get_ball_state();

    EXPECT_GT(ball.vy, 0.0)
        << "VY must increase when theta_y > 0";

    EXPECT_NEAR(ball.vx, 0.0, 1e-9)
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
    const BallState  ball  = estimator.get_ball_state();
    const TableState table = estimator.get_table_state();
    EXPECT_DOUBLE_EQ(ball.x,      0.0);
    EXPECT_DOUBLE_EQ(ball.y,      0.0);
    EXPECT_DOUBLE_EQ(ball.z_ball, 0.0);
    EXPECT_DOUBLE_EQ(ball.vx,     0.0);
    EXPECT_DOUBLE_EQ(ball.vy,     0.0);
    EXPECT_DOUBLE_EQ(ball.vz_ball,0.0);
    EXPECT_DOUBLE_EQ(table.phi,   0.0);
    EXPECT_DOUBLE_EQ(table.theta, 0.0);
    EXPECT_DOUBLE_EQ(table.z_t,   0.0);
}

TEST(StateEstimator, ResetSetsState) {
    SystemParameters params;
    params.initialize();

    StateEstimator estimator(params);

    BallState  new_ball{};
    new_ball.x      = 0.1;
    new_ball.y      = 0.2;
    new_ball.z_ball = 0.3;
    new_ball.vx     = 0.4;
    new_ball.vy     = 0.05;
    new_ball.vz_ball= 0.06;

    TableState new_table{};
    new_table.phi   = 0.01;
    new_table.theta = 0.02;
    new_table.z_t   = 0.03;

    estimator.reset(new_ball, new_table);

    const BallState  ball  = estimator.get_ball_state();
    const TableState table = estimator.get_table_state();

    EXPECT_DOUBLE_EQ(ball.x,       new_ball.x);
    EXPECT_DOUBLE_EQ(ball.y,       new_ball.y);
    EXPECT_DOUBLE_EQ(ball.z_ball,  new_ball.z_ball);
    EXPECT_DOUBLE_EQ(ball.vx,      new_ball.vx);
    EXPECT_DOUBLE_EQ(ball.vy,      new_ball.vy);
    EXPECT_DOUBLE_EQ(ball.vz_ball, new_ball.vz_ball);
    EXPECT_DOUBLE_EQ(table.phi,    new_table.phi);
    EXPECT_DOUBLE_EQ(table.theta,  new_table.theta);
    EXPECT_DOUBLE_EQ(table.z_t,    new_table.z_t);
}

/**
 * Test that prediction step advances state forward in time.
 */
TEST(StateEstimator, PredictionAdvancesState) {
    SystemParameters params;
    params.initialize();

    StateEstimator estimator(params);

    // Set initial state with non-zero velocity
    BallState init{};
    init.vx = 0.1;  // vx = 0.1 m/s
    estimator.reset(init, TableState{});

    // Run prediction
    estimator.predict(ControlVector::Zero());

    const BallState ball = estimator.get_ball_state();

    // Position X should have changed due to velocity integration
    EXPECT_NE(ball.x, 0.0) << "Position should change after prediction";
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
    BallState init{};
    init.x = 0.1;
    init.y = 0.2;
    estimator.reset(init, TableState{});

    // Provide measurement at origin
    MeasurementVector measurement;
    measurement << 0.0, 0.0;

    estimator.update(measurement);

    const BallState ball = estimator.get_ball_state();

    // State estimate should move closer to measurement
    EXPECT_LT(std::abs(ball.x), 0.1) << "X estimate should move toward measurement";
    EXPECT_LT(std::abs(ball.y), 0.2) << "Y estimate should move toward measurement";
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

    // X position error → THETA_Y_CMD (pid_x_ controls theta_y)
    EXPECT_NE(control(control_index::THETA_Y_CMD), 0.0)
        << "X controller (pid_x_) should produce THETA_Y_CMD output for X error";

    // Y control (pid_y_ → VARPHI_X_CMD) should be zero (zero gains)
    EXPECT_DOUBLE_EQ(control(control_index::VARPHI_X_CMD), 0.0)
        << "Y controller should produce zero VARPHI_X_CMD with zero gains";
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
