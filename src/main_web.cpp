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

    std::cout << "========================================" << '\n';
    std::cout << "  Ball Balancer Simulation (Web)" << '\n';
    std::cout << "========================================" << '\n';
    std::cout << '\n';

    try {
        // ====================================================================
        // Create System Parameters (initialized with defaults)
        // ====================================================================
#ifndef NDEBUG
        g_params.print();
        std::cout << '\n';
#endif

        // ====================================================================
        // Create and Initialize Application
        // ====================================================================
        g_app = std::make_unique<ball_balancer::Application>(g_params);

#ifndef NDEBUG
        std::cout << "Initializing web application..." << '\n';
#endif
        if (!g_app->initialize()) {
            std::cerr << "Failed to initialize application" << '\n';
            return 1;
        }
#ifndef NDEBUG
        std::cout << '\n';

        // ====================================================================
        // Set Up Browser Integration
        // ====================================================================

        // Register canvas resize callback
        emscripten_set_resize_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, nullptr, EM_TRUE, on_canvas_resize);

        // ====================================================================
        // Start Main Loop (Browser Event Loop)
        // ====================================================================
        std::cout << "Controls:" << '\n';
        std::cout << "  - Use Control Panel to adjust setpoint and gains" << '\n';
        std::cout << "  - Click 'Start' to begin simulation" << '\n';
        std::cout << "  - Application runs in browser" << '\n';
        std::cout << '\n';

        std::cout << "Starting browser main loop..." << '\n';
#else
        // In release mode, still register resize callback
        emscripten_set_resize_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, nullptr, EM_TRUE, on_canvas_resize);
#endif

        // Start the application's main loop
        // The run() method will internally use emscripten_set_main_loop
        g_app->run();

        // Note: Code after run() won't execute until the page is closed
        // Cleanup is handled by browser's page unload

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
