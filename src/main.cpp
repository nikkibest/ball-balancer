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

    std::cout << "========================================" << '\n';
    std::cout << "  Ball Balancer Simulation" << '\n';
    std::cout << "========================================" << '\n';
    std::cout << '\n';

    try {
        // ====================================================================
        // Create System Parameters (initialized with defaults)
        // ====================================================================
        ball_balancer::SystemParameters params;
#ifndef NDEBUG
        params.print();
        std::cout << '\n';
#endif

        // ====================================================================
        // Create and Initialize Application
        // ====================================================================
        ball_balancer::Application app(params);

#ifndef NDEBUG
        std::cout << "Initializing application..." << '\n';
#endif
        if (!app.initialize()) {
            std::cerr << "Failed to initialize application" << '\n';
            return 1;
        }
#ifndef NDEBUG
        std::cout << '\n';

        // ====================================================================
        // Run Main Loop
        // ====================================================================
        std::cout << "Controls:" << '\n';
        std::cout << "  - Use Control Panel to adjust setpoint and gains" << '\n';
        std::cout << "  - Click 'Start' to begin simulation" << '\n';
        std::cout << "  - Close window or File->Exit to quit" << '\n';
        std::cout << '\n';
#endif

        app.run();

        // ====================================================================
        // Shutdown
        // ====================================================================
        app.shutdown();

#ifndef NDEBUG
        std::cout << '\n';
        std::cout << "========================================" << '\n';
        std::cout << "  Application exited successfully" << '\n';
        std::cout << "========================================" << '\n';
#endif

        return 0;

    } catch (const std::exception& e) {
        std::cerr << '\n';
        std::cerr << "========================================" << '\n';
        std::cerr << "  FATAL ERROR" << '\n';
        std::cerr << "========================================" << '\n';
        std::cerr << "Exception: " << e.what() << '\n';
        std::cerr << '\n';
        return 1;

    } catch (...) {
        std::cerr << '\n';
        std::cerr << "========================================" << '\n';
        std::cerr << "  FATAL ERROR" << '\n';
        std::cerr << "========================================" << '\n';
        std::cerr << "Unknown exception caught" << '\n';
        std::cerr << '\n';
        return 1;
    }
}
