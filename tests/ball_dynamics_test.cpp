#include <gtest/gtest.h>
#include <ball_balancer/physics/ball_dynamics.hpp>
#include <ball_balancer/core/types.hpp>

/**
 * @file ball_dynamics_test.cpp
 * @brief Unit tests for BallDynamics: isInContact, computeNormalForce,
 *        computeAccelerations (contact + free flight), and applyBounce.
 */

namespace ball_balancer {
namespace {

// Helpers -------------------------------------------------------------------

SystemParameters makeDefaultParams() {
    SystemParameters p;
    p.ball_mass             = 0.027;
    p.ball_radius           = 0.020;
    p.gravity               = 9.81;
    p.viscous_friction_coeff = 0.1;
    p.bounce_coeff          = 0.5;
    p.initialize();
    return p;
}

/// Build a BallState with ball resting on a flat table at z_t=0: z_b = r.
BallState restingState(double x = 0.0, double y = 0.0) {
    SystemParameters p = makeDefaultParams();
    BallState b{};
    b.x      = x;
    b.y      = y;
    b.z_ball = p.ball_radius;   // resting on flat table
    return b;
}

/// Flat, stationary table at z_t=0.
TableState flatStaticTable() {
    return TableState{};   // all fields default to 0.0
}

// ===========================================================================
// isInContact
// ===========================================================================

TEST(BallDynamicsContact, BallAtSurfaceIsInContact) {
    BallDynamics bd(makeDefaultParams());
    const auto ball  = restingState();
    const auto table = flatStaticTable();
    EXPECT_TRUE(bd.isInContact(ball, table))
        << "Ball at z_b = r on flat table should be in contact";
}

TEST(BallDynamicsContact, BallAboveSurfaceIsNotInContact) {
    BallDynamics bd(makeDefaultParams());
    SystemParameters p = makeDefaultParams();
    BallState ball = restingState();
    ball.z_ball = p.ball_radius + 0.05;  // 5 cm above surface
    EXPECT_FALSE(bd.isInContact(ball, flatStaticTable()))
        << "Ball 5 cm above surface should NOT be in contact";
}

TEST(BallDynamicsContact, TiltedTableSurfaceHeightAccounted) {
    BallDynamics bd(makeDefaultParams());
    SystemParameters p = makeDefaultParams();
    // Place ball at x=0.1 m, table pitches forward (theta=0.05 rad).
    // Surface height at x=0.1: z_surface = 0 + 0.1*0.05 = 0.005 m
    // Ball z_b = r + 0.005 → in contact
    BallState ball{};
    ball.x      = 0.1;
    ball.z_ball = p.ball_radius + 0.005;

    TableState table;
    table.theta = 0.05;

    EXPECT_TRUE(bd.isInContact(ball, table));

    // Raise ball 1 mm above surface → not in contact
    ball.z_ball += 0.001;
    EXPECT_FALSE(bd.isInContact(ball, table));
}

// ===========================================================================
// computeNormalForce
// ===========================================================================

TEST(BallDynamicsNormalForce, FlatStaticTable) {
    BallDynamics bd(makeDefaultParams());
    SystemParameters p = makeDefaultParams();
    const double expectedN = p.ball_mass * p.gravity;

    const double N = bd.computeNormalForce(restingState(), flatStaticTable());
    EXPECT_NEAR(N, expectedN, 1e-9)
        << "N should equal m*g on flat, static table";
}

TEST(BallDynamicsNormalForce, NeverNegative) {
    BallDynamics bd(makeDefaultParams());
    // Strongly downward table acceleration → N would be negative without clamp
    TableState table;
    table.z_t_ddot = -100.0;   // table free-falling rapidly
    const double N = bd.computeNormalForce(restingState(), table);
    EXPECT_GE(N, 0.0) << "Normal force must be clamped to >= 0";
}

// ===========================================================================
// computeAccelerations — contact mode
// ===========================================================================

TEST(BallDynamicsAccelerations, FlatStaticTableAtRest) {
    BallDynamics bd(makeDefaultParams());

    // Ball resting at origin on flat static table, zero velocities
    BallState  ball  = restingState();
    TableState table = flatStaticTable();

    double ax{1.0}, ay{1.0}, az{1.0};
    bd.computeAccelerations(ball, table, ax, ay, az);

    // Horizontal: no tilt, no rotation → ax = ay = 0
    EXPECT_NEAR(ax, 0.0, 1e-9) << "No horizontal acceleration on flat table";
    EXPECT_NEAR(ay, 0.0, 1e-9) << "No horizontal acceleration on flat table";

    // Vertical: contact mode, N = mg → az = -g + N/m = 0 (ball at rest on surface)
    EXPECT_NEAR(az, 0.0, 1e-6) << "Ball at rest on surface: az should be ~0";
}

TEST(BallDynamicsAccelerations, TiltedTableDrivesXAcceleration) {
    BallDynamics bd(makeDefaultParams());
    SystemParameters p = makeDefaultParams();

    // theta > 0 (THETA_Y) should drive ball in -X direction
    BallState  ball  = restingState();
    TableState table = flatStaticTable();
    table.theta = 0.05;  // 5 deg pitch

    double ax{0.0}, ay{0.0}, az{0.0};
    bd.computeAccelerations(ball, table, ax, ay, az);

    // ax = -g*theta = -9.81*0.05 ≈ -0.49 m/s² (negative for positive theta)
    EXPECT_LT(ax, 0.0) << "Positive theta should drive ball in -X";
    EXPECT_NEAR(ax, -p.gravity * table.theta, 0.01);
    EXPECT_NEAR(ay, 0.0, 1e-6) << "No Y acceleration for pure pitch";
}

TEST(BallDynamicsAccelerations, TiltedTableDrivesYAcceleration) {
    BallDynamics bd(makeDefaultParams());
    SystemParameters p = makeDefaultParams();

    // phi > 0 (VARPHI_X) should drive ball in +Y direction
    BallState  ball  = restingState();
    TableState table = flatStaticTable();
    table.phi = 0.05;  // 5 deg roll

    double ax{0.0}, ay{0.0}, az{0.0};
    bd.computeAccelerations(ball, table, ax, ay, az);

    // ay = g*phi = 9.81*0.05 ≈ +0.49 m/s²
    EXPECT_GT(ay, 0.0) << "Positive phi should drive ball in +Y";
    EXPECT_NEAR(ay, p.gravity * table.phi, 0.01);
    EXPECT_NEAR(ax, 0.0, 1e-6) << "No X acceleration for pure roll";
}

// ===========================================================================
// computeAccelerations — free-flight mode
// ===========================================================================

TEST(BallDynamicsAccelerations, FreeFlight) {
    const SystemParameters p = makeDefaultParams();
    BallDynamics bd(p);

    // Ball well above table → free flight
    BallState ball{};
    ball.z_ball = p.ball_radius + 0.5;  // 50 cm above surface

    // Flat, rapidly downward-accelerating table so N would be negative
    TableState table = flatStaticTable();
    table.z_t_ddot = -50.0;

    double ax{1.0}, ay{1.0}, az{1.0};
    bd.computeAccelerations(ball, table, ax, ay, az);

    EXPECT_NEAR(ax, 0.0, 1e-9)        << "No X acceleration in free flight";
    EXPECT_NEAR(ay, 0.0, 1e-9)        << "No Y acceleration in free flight";
    EXPECT_NEAR(az, -p.gravity, 1e-9) << "az = -g in free flight";
}

// ===========================================================================
// applyBounce
// ===========================================================================

TEST(BallDynamicsBounce, BallApproachingTableBounces) {
    BallDynamics bd(makeDefaultParams());
    SystemParameters p = makeDefaultParams();

    BallState ball = restingState();
    ball.vz_ball = -1.0;  // approaching table at 1 m/s

    TableState table = flatStaticTable();  // vz_contact = 0

    const double vz_before = ball.vz_ball;
    bd.applyBounce(ball, table);
    const double vz_after = ball.vz_ball;

    // vz_after = (1+e)*0 - e*(-1.0) = 0 + 0.5 = +0.5 m/s (bouncing up)
    const double expected = (1.0 + p.bounce_coeff) * 0.0 - p.bounce_coeff * vz_before;
    EXPECT_NEAR(vz_after, expected, 1e-9) << "Bounce velocity incorrect";
    EXPECT_GT(vz_after, 0.0) << "Ball should bounce upward";
}

TEST(BallDynamicsBounce, MovingAwayFromTableNotAffected) {
    BallDynamics bd(makeDefaultParams());

    BallState ball = restingState();
    ball.vz_ball = +1.0;  // already moving away

    const double vz_before = ball.vz_ball;
    bd.applyBounce(ball, flatStaticTable());
    EXPECT_NEAR(ball.vz_ball, vz_before, 1e-9)
        << "applyBounce should not change vz when ball is moving away";
}

TEST(BallDynamicsBounce, HorizontalVelocitiesUnchanged) {
    BallDynamics bd(makeDefaultParams());

    BallState ball = restingState();
    ball.vx      =  0.3;
    ball.vy      = -0.2;
    ball.vz_ball = -0.5;

    bd.applyBounce(ball, flatStaticTable());

    EXPECT_NEAR(ball.vx,  0.3,  1e-9) << "VX must be unchanged by bounce";
    EXPECT_NEAR(ball.vy, -0.2, 1e-9)  << "VY must be unchanged by bounce";
}

} // namespace
} // namespace ball_balancer
