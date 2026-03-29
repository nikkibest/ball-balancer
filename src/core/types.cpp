#include <ball_balancer/core/types.hpp>
#include <iostream>
#include <cmath>

namespace ball_balancer {

void SystemParameters::print() const {
    std::cout << "System Parameters:\n";
    std::cout << "  Ball mass: " << ball_mass * 1000.0 << " g\n";
    std::cout << "  Ball radius: " << ball_radius * 1000.0 << " mm\n";
    std::cout << "  Table radius: " << table_radius * 100.0 << " cm\n";
    std::cout << "  Max tilt: " << max_tilt_angle * 180.0 / M_PI << " degrees\n";
    std::cout << "  Control rate: " << 1.0 / control_dt << " Hz\n";
}

} // namespace ball_balancer
