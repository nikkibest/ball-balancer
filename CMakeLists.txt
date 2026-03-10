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

# ImPlot3D - 3D plotting library
set(IMPLOT3D_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/implot3d)
set(IMPLOT3D_SOURCES
    ${IMPLOT3D_DIR}/implot3d.cpp
    ${IMPLOT3D_DIR}/implot3d_items.cpp
    ${IMPLOT3D_DIR}/implot3d_meshes.cpp
)
add_library(implot3d STATIC ${IMPLOT3D_SOURCES})
target_include_directories(implot3d PUBLIC
    ${IMPLOT3D_DIR}
    ${IMGUI_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/external  # For imconfig.h
)
target_link_libraries(implot3d PUBLIC imgui)

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
    src/core/types.cpp

    # Physics
    src/physics/simulator.cpp
    src/physics/ball_dynamics.cpp
    src/physics/table_kinematics.cpp

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
    include/ball_balancer/physics/table_kinematics.hpp

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
    implot3d
    ${CMAKE_DL_LIBS}  # For dlopen/dlsym (used by GLAD)
)

# X11 is required on Linux for GLFW/ImGui (but not on macOS or Windows)
if(UNIX AND NOT APPLE)
    find_package(X11)
    if(X11_FOUND)
        target_link_libraries(ball_balancer PRIVATE X11)
    endif()
endif()

# Link Boost if found
if(Boost_FOUND)
    target_link_libraries(ball_balancer PRIVATE Boost::boost)
endif()

# Set compiler warnings and optimization for our code only.
# Use Release build (-DCMAKE_BUILD_TYPE=Release) for production performance.
# Use Debug build (default) for development with full debugging symbols.
if(MSVC)
    target_compile_options(ball_balancer PRIVATE
        /W4
        $<$<CONFIG:Release>:/O2 /DNDEBUG>
        $<$<CONFIG:Debug>:/Od /Zi>
    )
else()
    target_compile_options(ball_balancer PRIVATE
        -Wall -Wextra -Wpedantic
        -Wno-maybe-uninitialized  # Suppress false positives from Eigen
        $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>
        $<$<CONFIG:Debug>:-O0 -g>
    )
    # Disable Eigen's runtime checks in Release builds for maximum performance
    target_compile_definitions(ball_balancer PRIVATE
        $<$<CONFIG:Release>:EIGEN_NO_DEBUG>
    )
endif()

# ============================================================================
# Testing
# ============================================================================

option(BUILD_TESTS "Build unit and integration tests" ON)

# Only build tests for native builds (not Emscripten)
if(BUILD_TESTS AND NOT EMSCRIPTEN)
    enable_testing()

    # Fetch GoogleTest from GitHub
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        EXCLUDE_FROM_ALL
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Implementation sources to link with tests (no GUI/rendering dependencies)
    set(TEST_IMPL_SOURCES
        src/physics/simulator.cpp
        src/physics/ball_dynamics.cpp
        src/control/pid_controller.cpp
        src/control/state_estimator.cpp
        src/math/matrix_utils.cpp
        src/visualization/data_manager.cpp
        src/core/types.cpp
    )

    # Test executable
    add_executable(ball_balancer_tests
        # tests/physics_test.cpp
        tests/control_test.cpp
        tests/data_manager_test.cpp
        tests/ball_dynamics_test.cpp
        tests/integration_test.cpp
        ${TEST_IMPL_SOURCES}
    )

    target_include_directories(ball_balancer_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
    )

    target_link_libraries(ball_balancer_tests PRIVATE
        GTest::gtest_main
    )

    # Apply same compiler flags as main executable
    if(NOT MSVC)
        target_compile_options(ball_balancer_tests PRIVATE
            -Wall -Wextra -Wpedantic
            -Wno-maybe-uninitialized
            $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG -DEIGEN_NO_DEBUG>
            $<$<CONFIG:Debug>:-O0 -g>
        )
    endif()

    # Discover tests for CTest
    include(GoogleTest)
    gtest_discover_tests(ball_balancer_tests)
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
