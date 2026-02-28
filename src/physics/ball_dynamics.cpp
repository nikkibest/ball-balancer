#include <ball_balancer/physics/ball_dynamics.hpp>

namespace ball_balancer {

BallDynamics::BallDynamics(const SystemParameters& params)
    : params_(params)
{
}

bool BallDynamics::isInContact(const StateVector& state, const TableState& table) const {
    (void)state;
    (void)table;
    return true;  // stub
}

double BallDynamics::computeNormalForce(const StateVector& state, const TableState& table) const {
    (void)state;
    (void)table;
    return params_.ball_mass * params_.gravity;  // stub: N = mg
}

void BallDynamics::computeAccelerations(
    const StateVector& state,
    const TableState& table,
    double& ax, double& ay, double& az
) const {
    (void)state;
    (void)table;
    ax = 0.0;
    ay = 0.0;
    az = 0.0;  // stub
}

void BallDynamics::applyBounce(StateVector& state, const TableState& table) const {
    (void)state;
    (void)table;  // stub
}

} // namespace ball_balancer
