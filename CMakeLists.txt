cmake_minimum_required(VERSION 3.15)
project(BallBalancer VERSION 1.0.0 LANGUAGES C CXX)

# ============================================================================
# C++ Standard and Compiler Settings
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDEs and tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# Compiler Warnings and Optimization
# ============================================================================
# Note: Compiler flags are set per-target to avoid affecting external libraries

# ============================================================================
# Dependencies
# ============================================================================

# Eigen3 - Linear algebra library (header-only from external/)
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen)
message(STATUS "Using Eigen3 from: ${EIGEN3_INCLUDE_DIR}")

# OpenGL - Graphics rendering
find_package(OpenGL REQUIRED)
message(STATUS "Found OpenGL: ${OPENGL_LIBRARIES}")

# Boost - ODE integration (Boost.Odeint header-only) - Optional for now
find_package(Boost 1.65)
if(Boost_FOUND)
    message(STATUS "Found Boost: ${Boost_INCLUDE_DIRS}")
else()
    message(STATUS "Boost not found - some features may be unavailable")
    set(Boost_INCLUDE_DIRS "")
endif()

# GLFW - Window and input management (will be added to external/)
# ImGui - GUI framework (will be added to external/)
# ImPlot - Plotting library (will be added to external/)

# ============================================================================
# External Libraries (Git Submodules or Manual)
# ============================================================================

# NOTE: Add these libraries to external/ directory:
# - GLFW: https://github.com/glfw/glfw
# - ImGui: https://github.com/ocornut/imgui (docking branch)
# - ImPlot: https://github.com/epezent/implot
# - GoogleTest: https://github.com/google/googletest

# GLFW - Build as subdirectory
add_subdirectory(external/glfw EXCLUDE_FROM_ALL)

# ImGui - Add source files manually (not a CMake project)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# ImPlot - Add source files manually
set(IMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/implot)
set(IMPLOT_SOURCES
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
)

# Create ImGui library
add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/external  # For our custom imconfig.h
)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
target_compile_definitions(imgui PUBLIC IMGUI_USER_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/external/imconfig.h")

# Create ImPlot library
add_library(implot STATIC ${IMPLOT_SOURCES})
target_include_directories(implot PUBLIC
    ${IMPLOT_DIR}
    ${IMGUI_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/external  # For imconfig.h
)
target_link_libraries(implot PUBLIC imgui)

# GLAD - OpenGL function loader
add_library(glad STATIC external/glad/src/glad.c)
target_include_directories(glad PUBLIC external/glad/include)
target_link_libraries(glad PUBLIC ${CMAKE_DL_LIBS})  # For dlopen/dlsym

# ============================================================================
# Source Files
# ============================================================================

set(BALL_BALANCER_SOURCES
    # Core
    src/core/application.cpp
    # Note: types.cpp not needed (header-only type definitions)

    # Physics
    src/physics/simulator.cpp

    # Control
    src/control/pid_controller.cpp
    src/control/state_estimator.cpp

    # Math (Eigen utilities)
    src/math/matrix_utils.cpp

    # Rendering (OpenGL)
    src/rendering/renderer.cpp
    src/rendering/shader.cpp
    src/rendering/camera.cpp

    # GUI (ImGui)
    src/gui/control_panel.cpp
    src/gui/main_window.cpp

    # Visualization (ImPlot)
    src/visualization/real_time_plotter.cpp
    src/visualization/data_manager.cpp

    # Main entry point
    src/main.cpp
)

set(BALL_BALANCER_HEADERS
    include/ball_balancer/core/application.hpp
    include/ball_balancer/core/types.hpp

    include/ball_balancer/physics/simulator.hpp

    include/ball_balancer/control/pid_controller.hpp
    include/ball_balancer/control/state_estimator.hpp

    include/ball_balancer/math/matrix_utils.hpp

    include/ball_balancer/rendering/renderer.hpp
    include/ball_balancer/rendering/shader.hpp
    include/ball_balancer/rendering/camera.hpp

    include/ball_balancer/gui/control_panel.hpp
    include/ball_balancer/gui/main_window.hpp

    include/ball_balancer/visualization/real_time_plotter.hpp
)

# ============================================================================
# Main Executable
# ============================================================================

add_executable(ball_balancer ${BALL_BALANCER_SOURCES} ${BALL_BALANCER_HEADERS})

target_include_directories(ball_balancer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/external  # For imconfig.h
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(ball_balancer PRIVATE
    OpenGL::GL
    glfw
    glad
    imgui
    implot
    ${CMAKE_DL_LIBS}  # For dlopen/dlsym (used by GLAD)
    X11  # X11 library for Linux
)

# Link Boost if found
if(Boost_FOUND)
    target_link_libraries(ball_balancer PRIVATE Boost::boost)
endif()

# Set compiler warnings and optimization for our code only
if(MSVC)
    target_compile_options(ball_balancer PRIVATE /W4 /O2)
else()
    target_compile_options(ball_balancer PRIVATE
        -Wall -Wextra -Wpedantic
        -Wno-maybe-uninitialized  # Suppress false positives from Eigen
        -O2  # Use O2 for faster compilation
    )
endif()

# ============================================================================
# Testing
# ============================================================================

option(BUILD_TESTS "Build unit and integration tests" ON)

if(BUILD_TESTS)
    enable_testing()

    # add_subdirectory(external/googletest EXCLUDE_FROM_ALL)

    set(TEST_SOURCES
        tests/physics_test.cpp
        tests/control_test.cpp
        tests/integration_test.cpp

        # Link implementation files (not main.cpp)
        src/physics/ball_model.cpp
        src/physics/table_model.cpp
        src/control/pid_controller.cpp
        src/control/state_estimator.cpp
        src/math/matrix_utils.cpp
    )

    # add_executable(ball_balancer_tests ${TEST_SOURCES})

    # target_include_directories(ball_balancer_tests PRIVATE
    #     ${CMAKE_CURRENT_SOURCE_DIR}/include
    #     ${CMAKE_CURRENT_SOURCE_DIR}/src
    #     ${EIGEN3_INCLUDE_DIR}
    #     ${Boost_INCLUDE_DIRS}
    # )

    # target_link_libraries(ball_balancer_tests PRIVATE
    #     gtest_main
    #     gmock_main
    #     Eigen3::Eigen
    #     Boost::boost
    # )

    # add_test(NAME BallBalancerTests COMMAND ball_balancer_tests)
endif()

# ============================================================================
# Installation (Optional)
# ============================================================================

install(TARGETS ball_balancer
    RUNTIME DESTINATION bin
)

install(DIRECTORY shaders/
    DESTINATION share/ball_balancer/shaders
)

# ============================================================================
# Documentation
# ============================================================================

message(STATUS "")
message(STATUS "Ball Balancer Configuration Summary:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Build tests: ${BUILD_TESTS}")
message(STATUS "")
message(STATUS "External libraries to add:")
message(STATUS "  - GLFW (window management)")
message(STATUS "  - ImGui (GUI framework)")
message(STATUS "  - ImPlot (plotting)")
message(STATUS "  - GoogleTest (testing, if BUILD_TESTS=ON)")
message(STATUS "")
