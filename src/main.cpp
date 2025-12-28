#include <ball_balancer/core/application.hpp>
#include <ball_balancer/core/types.hpp>
#include <iostream>
#include <exception>

/**
 * @file main.cpp
 * @brief Main entry point for ball balancer application
 *
 * Creates application with default parameters and runs main loop.
 * Handles exceptions and ensures clean shutdown.
 */

int main(int argc, char** argv) {
    (void)argc;  // Unused
    (void)argv;  // Unused

    std::cout << "========================================" << std::endl;
    std::cout << "  Ball Balancer Simulation" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    try {
        // ====================================================================
        // Create System Parameters
        // ====================================================================
        ball_balancer::SystemParameters params;
        params.ball_mass = 0.027;            // 27g ping-pong ball
        params.ball_radius = 0.020;          // 20mm radius
        params.table_length = 0.5;           // 50cm x 50cm table
        params.table_width = 0.5;
        params.max_tilt_angle = 0.174;       // 10 degrees max tilt
        params.gravity = 9.81;               // m/s²
        params.control_dt = 0.01;            // 100 Hz control loop

        std::cout << "System Parameters:" << std::endl;
        std::cout << "  Ball mass: " << params.ball_mass * 1000.0 << " g" << std::endl;
        std::cout << "  Ball radius: " << params.ball_radius * 1000.0 << " mm" << std::endl;
        std::cout << "  Table size: " << params.table_length * 100.0 << " x "
                  << params.table_width * 100.0 << " cm" << std::endl;
        std::cout << "  Max tilt: " << params.max_tilt_angle * 180.0 / M_PI << " degrees" << std::endl;
        std::cout << "  Control rate: " << 1.0 / params.control_dt << " Hz" << std::endl;
        std::cout << std::endl;

        // ====================================================================
        // Create and Initialize Application
        // ====================================================================
        ball_balancer::Application app(params);

        std::cout << "Initializing application..." << std::endl;
        if (!app.initialize()) {
            std::cerr << "Failed to initialize application" << std::endl;
            return 1;
        }
        std::cout << std::endl;

        // ====================================================================
        // Run Main Loop
        // ====================================================================
        std::cout << "Controls:" << std::endl;
        std::cout << "  - Use Control Panel to adjust setpoint and gains" << std::endl;
        std::cout << "  - Click 'Start' to begin simulation" << std::endl;
        std::cout << "  - Close window or File->Exit to quit" << std::endl;
        std::cout << std::endl;

        app.run();

        // ====================================================================
        // Shutdown
        // ====================================================================
        app.shutdown();

        std::cout << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "  Application exited successfully" << std::endl;
        std::cout << "========================================" << std::endl;

        return 0;

    } catch (const std::exception& e) {
        std::cerr << std::endl;
        std::cerr << "========================================" << std::endl;
        std::cerr << "  FATAL ERROR" << std::endl;
        std::cerr << "========================================" << std::endl;
        std::cerr << "Exception: " << e.what() << std::endl;
        std::cerr << std::endl;
        return 1;

    } catch (...) {
        std::cerr << std::endl;
        std::cerr << "========================================" << std::endl;
        std::cerr << "  FATAL ERROR" << std::endl;
        std::cerr << "========================================" << std::endl;
        std::cerr << "Unknown exception caught" << std::endl;
        std::cerr << std::endl;
        return 1;
    }
}
