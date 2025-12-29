#include <ball_balancer/core/application.hpp>
#include <ball_balancer/core/types.hpp>
#include <emscripten.h>
#include <emscripten/html5.h>
#include <iostream>
#include <exception>
#include <memory>

/**
 * @file main_web.cpp
 * @brief Web-compatible entry point for ball balancer application
 *
 * Key differences from desktop version:
 * - Uses emscripten_set_main_loop instead of while loop
 * - Proper browser event loop integration
 * - Canvas resize handling
 * - Exception handling for JavaScript interop
 *
 * @see research/emscripten-web-compilation-best-practices.md
 */

// Global application instance (needed for Emscripten main loop callback)
static std::unique_ptr<ball_balancer::Application> g_app;
static ball_balancer::SystemParameters g_params;

/**
 * @brief Main loop iteration called by browser
 *
 * This function is called repeatedly by the browser's event loop via
 * emscripten_set_main_loop. It replaces the traditional while(!done) pattern
 * used in desktop applications.
 */
void main_loop_iteration() {
    if (g_app) {
        try {
            // Application's run() method needs to be adapted for single iteration
            // For now, we'll need to expose a step() method or modify run()
            // This is handled in the Application class modification below

            // Note: The Application::run() method contains the main loop,
            // so we need to modify it to support single-iteration stepping
            // See application_web.cpp for the modified version

        } catch (const std::exception& e) {
            std::cerr << "Error in main loop: " << e.what() << std::endl;
            emscripten_cancel_main_loop();
        }
    }
}

/**
 * @brief Canvas resize callback
 *
 * Handles browser window/canvas resize events to maintain proper aspect ratio
 * and viewport settings.
 */
EM_BOOL on_canvas_resize(int eventType, const EmscriptenUiEvent* uiEvent, void* userData) {
    (void)eventType;
    (void)uiEvent;
    (void)userData;

    // Get new canvas size
    double width, height;
    emscripten_get_element_css_size("#canvas", &width, &height);

    // Update renderer viewport
    if (g_app) {
        // We'll need to expose this method in Application
        // g_app->resize(static_cast<int>(width), static_cast<int>(height));
    }

    return EM_TRUE;
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    std::cout << "========================================" << std::endl;
    std::cout << "  Ball Balancer Simulation (Web)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    try {
        // ====================================================================
        // Create System Parameters
        // ====================================================================
        g_params.ball_mass = 0.027;            // 27g ping-pong ball
        g_params.ball_radius = 0.020;          // 20mm radius
        g_params.table_length = 0.5;           // 50cm x 50cm table
        g_params.table_width = 0.5;
        g_params.max_tilt_angle = 0.174;       // 10 degrees max tilt
        g_params.gravity = 9.81;               // m/s²
        g_params.control_dt = 0.01;            // 100 Hz control loop

        std::cout << "System Parameters:" << std::endl;
        std::cout << "  Ball mass: " << g_params.ball_mass * 1000.0 << " g" << std::endl;
        std::cout << "  Ball radius: " << g_params.ball_radius * 1000.0 << " mm" << std::endl;
        std::cout << "  Table size: " << g_params.table_length * 100.0 << " x "
                  << g_params.table_width * 100.0 << " cm" << std::endl;
        std::cout << "  Max tilt: " << g_params.max_tilt_angle * 180.0 / M_PI << " degrees" << std::endl;
        std::cout << "  Control rate: " << 1.0 / g_params.control_dt << " Hz" << std::endl;
        std::cout << std::endl;

        // ====================================================================
        // Create and Initialize Application
        // ====================================================================
        g_app = std::make_unique<ball_balancer::Application>(g_params);

        std::cout << "Initializing web application..." << std::endl;
        if (!g_app->initialize()) {
            std::cerr << "Failed to initialize application" << std::endl;
            return 1;
        }
        std::cout << std::endl;

        // ====================================================================
        // Set Up Browser Integration
        // ====================================================================

        // Register canvas resize callback
        emscripten_set_resize_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, nullptr, EM_TRUE, on_canvas_resize);

        // ====================================================================
        // Start Main Loop (Browser Event Loop)
        // ====================================================================
        std::cout << "Controls:" << std::endl;
        std::cout << "  - Use Control Panel to adjust setpoint and gains" << std::endl;
        std::cout << "  - Click 'Start' to begin simulation" << std::endl;
        std::cout << "  - Application runs in browser" << std::endl;
        std::cout << std::endl;

        std::cout << "Starting browser main loop..." << std::endl;

        // Start the application's main loop
        // The run() method will internally use emscripten_set_main_loop
        g_app->run();

        // Note: Code after run() won't execute until the page is closed
        // Cleanup is handled by browser's page unload

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
