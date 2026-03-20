#include <gtest/gtest.h>
#include <ball_balancer/physics/simulator.hpp>
#include <ball_balancer/core/types.hpp>
#include <cmath>

/**
 * @file integration_test.cpp
 * @brief Phase 6 integration tests for the full BallDynamics + Simulator loop.
 *
 * Tests:
 *  6.1 — Flat table, zero control: ball stays on surface for >=5 s
 *  6.2 — Applied tilt: ball rolls in correct direction; Z tracks surface
 *  6.3 — Free flight / bounce: ball drops, bounces, and stays above surface
 *  6.6 — Regression: simple P-control reduces position error (horizontal dynamics)
 */

namespace ball_balancer {
namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

SystemParameters defaultParams() {
    SystemParameters p;
    p.initialize();
    return p;
}

/// Step simulator for `seconds` at the given dt with constant control.
void runFor(Simulator& sim, double seconds, const ControlVector& ctrl,
            double dt = 0.001) {
    const int steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i) {
        sim.step(dt, ctrl);
    }
}

// ---------------------------------------------------------------------------
// Task 6.1 — Flat table equilibrium
// ---------------------------------------------------------------------------

TEST(Integration, FlatTableBallStaysOnSurface) {
    SystemParameters p = defaultParams();
    Simulator sim(p);

    // Ball starts resting on flat table at origin
    BallState  ball{};
    ball.z_ball = p.ball_radius;   // z_b = r (on surface, z_t=0)
    TableState table{};
    sim.reset(ball, table);

    const ControlVector zero_ctrl = ControlVector::Zero();

    // Run for 5 seconds
    runFor(sim, 5.0, zero_ctrl);

    const BallState  final_ball  = sim.get_ball_state();
    const TableState final_table = sim.get_table_state();
    const double z_surface = final_table.z_t + p.ball_radius;

    // Ball must remain on surface (within 0.5 mm)
    EXPECT_NEAR(final_ball.z_ball, z_surface, 5e-4)
        << "Ball should stay on surface on flat table with zero control";

    // Horizontal drift must be negligible (within 0.5 mm)
    EXPECT_NEAR(final_ball.x, 0.0, 5e-4)
        << "Ball should not drift horizontally on flat table";
    EXPECT_NEAR(final_ball.y, 0.0, 5e-4)
        << "Ball should not drift horizontally on flat table";

    // Ball should be in contact
    EXPECT_TRUE(sim.isInContact())
        << "Ball should remain in contact with flat table";
}

// ---------------------------------------------------------------------------
// Task 6.2 — Tilt drives ball in correct direction; Z tracks surface
// ---------------------------------------------------------------------------

TEST(Integration, TiltDrivesBallInCorrectXDirection) {
    SystemParameters p = defaultParams();
    Simulator sim(p);

    BallState ball{};
    ball.z_ball = p.ball_radius;
    sim.reset(ball, TableState{});

    // Positive THETA_Y_CMD (pitch) should drive ball in -X direction
    ControlVector ctrl = ControlVector::Zero();
    ctrl(control_index::THETA_Y_CMD) = 0.05;  // ~3 degrees

    runFor(sim, 1.0, ctrl);

    const BallState final_ball = sim.get_ball_state();
    EXPECT_LT(final_ball.x, -0.01)
        << "Positive THETA_Y should drive ball in -X direction";

    // Y should remain near zero (pure pitch, no roll)
    EXPECT_NEAR(final_ball.y, 0.0, 0.02)
        << "Pure pitch should not drive ball in Y direction";
}

TEST(Integration, TiltDrivesBallInCorrectYDirection) {
    SystemParameters p = defaultParams();
    Simulator sim(p);

    BallState ball{};
    ball.z_ball = p.ball_radius;
    sim.reset(ball, TableState{});

    // Positive VARPHI_X_CMD (roll / phi) should drive ball in +Y direction
    ControlVector ctrl = ControlVector::Zero();
    ctrl(control_index::VARPHI_X_CMD) = 0.05;

    runFor(sim, 1.0, ctrl);

    const BallState final_ball = sim.get_ball_state();
    EXPECT_GT(final_ball.y, 0.01)
        << "Positive VARPHI_X should drive ball in +Y direction";

    EXPECT_NEAR(final_ball.x, 0.0, 0.02)
        << "Pure roll should not drive ball in X direction";
}

TEST(Integration, ZTracksTableSurfaceDuringTilt) {
    SystemParameters p = defaultParams();
    Simulator sim(p);

    BallState ball{};
    ball.z_ball = p.ball_radius;
    sim.reset(ball, TableState{});

    ControlVector ctrl = ControlVector::Zero();
    ctrl(control_index::THETA_Y_CMD) = 0.05;

    runFor(sim, 1.0, ctrl);

    const BallState  final_ball  = sim.get_ball_state();
    const TableState final_table = sim.get_table_state();

    // Expected surface height at current (x, theta): z_surface = z_t + r + x*theta
    const double z_surface_expected = final_table.z_t + p.ball_radius
                                      + final_ball.x * final_table.theta;

    // Ball should be within 2 mm of the tilted surface
    EXPECT_NEAR(final_ball.z_ball, z_surface_expected, 2e-3)
        << "Ball Z should track tilted table surface";
}

// ---------------------------------------------------------------------------
// Task 6.3 — Free flight and bounce
// ---------------------------------------------------------------------------

TEST(Integration, BallDropsAndBouncesFromFreeFlightHeight) {
    SystemParameters p = defaultParams();
    Simulator sim(p);

    // Place ball 10 cm above the flat table surface (free flight)
    BallState ball{};
    ball.z_ball  = p.ball_radius + 0.10;  // 10 cm above surface
    ball.vz_ball = 0.0;
    sim.reset(ball, TableState{});

    EXPECT_FALSE(sim.isInContact())
        << "Ball should start in free flight";

    const ControlVector zero_ctrl = ControlVector::Zero();

    // Free-fall from 10 cm: t = sqrt(2*0.1/9.81) ≈ 0.14 s.
    // Run 0.5 s to observe at least one bounce.
    runFor(sim, 0.5, zero_ctrl);

    const BallState final_ball = sim.get_ball_state();

    // After bounce ball must be at or above the table surface (not sunk through)
    EXPECT_GE(final_ball.z_ball, p.ball_radius - 1e-4)
        << "Ball should be at or above table surface after bounce";
}

TEST(Integration, InelasticBallSettlesOnSurface) {
    SystemParameters p = defaultParams();
    p.bounce_coeff = 0.0;  // Fully inelastic — ball settles on surface
    Simulator sim(p);

    // Drop ball from 5 cm
    BallState ball{};
    ball.z_ball = p.ball_radius + 0.05;
    sim.reset(ball, TableState{});

    runFor(sim, 1.0, ControlVector::Zero());

    // With zero bounce, ball should come to rest exactly on surface
    EXPECT_TRUE(sim.isInContact())
        << "Ball should be in contact after fully inelastic bounce";
    EXPECT_NEAR(sim.get_ball_state().z_ball, p.ball_radius, 1e-4)
        << "Ball should rest exactly on surface after inelastic bounce";
}

// ---------------------------------------------------------------------------
// Task 6.6 — Regression: horizontal dynamics
// ---------------------------------------------------------------------------

TEST(Integration, ProportionalControlDrivesBallTowardSetpoint) {
    SystemParameters p = defaultParams();
    Simulator sim(p);

    // Ball starts at x=0.1 m, offset from centre
    BallState ball{};
    ball.x      = 0.10;
    ball.z_ball = p.ball_radius;
    sim.reset(ball, TableState{});

    const double Kp = 0.5;     // Simple proportional gain
    const double dt = 0.01;    // 100 Hz control loop
    const int    N  = 800;     // 8 seconds

    for (int i = 0; i < N; ++i) {
        const BallState curr = sim.get_ball_state();

        // P-control: THETA_Y_CMD controls X axis
        ControlVector ctrl = ControlVector::Zero();
        ctrl(control_index::THETA_Y_CMD) = Kp * curr.x;

        sim.step(dt, ctrl);
    }

    const double x_final = std::abs(sim.get_ball_state().x);
    EXPECT_LT(x_final, 0.05)
        << "P-control should reduce position error by at least 50% (from 0.10 m)";
}

} // namespace
} // namespace ball_balancer
