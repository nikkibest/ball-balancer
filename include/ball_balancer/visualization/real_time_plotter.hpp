#pragma once

#include <ball_balancer/core/types.hpp>
#include <ball_balancer/visualization/data_manager.hpp>
#include <implot.h>
#include <implot3d.h>

/**
 * @file real_time_plotter.hpp
 * @brief Real-time plotting for ball balancer data
 *
 * Provides real-time visualization of system data:
 * - Ball position trajectory (X vs Y)
 * - Position vs time (separate X and Y plots)
 * - Control signals (table tilt angles)
 * - Position error from setpoint
 *
 * Following best practices from research/implot-cpp-plotting-best-practices.md:
 * - CRITICAL: Requires 32-bit indices in imconfig.h
 * - Ring buffers for efficient streaming data (via DataManager)
 * - Always call EndPlot even if BeginPlot returns false
 * - Use fixed-size pre-allocated buffers (std::array, not std::vector)
 *
 * @see research/implot-cpp-plotting-best-practices.md
 * @see research/cpp-best-practices-modern-programming.md - RAII, compile-time allocation
 */

namespace ball_balancer {

/**
 * @brief Real-time plotter for ball balancer
 *
 * Maintains ring buffers for streaming data and provides
 * plotting functions for various visualizations.
 *
 * Usage:
 * ```cpp
 * RealTimePlotter plotter(params);
 *
 * // Update data each frame
 * plotter.update(time, state, control, setpoint);
 *
 * // Render plots
 * ImGui::Begin("Plots");
 * plotter.render();
 * ImGui::End();
 * ```
 */
class RealTimePlotter {
public:
    /**
     * @brief Construct plotter
     * @param params System parameters
     */
    explicit RealTimePlotter(const SystemParameters& params);

    /**
     * @brief Update plotter with new data
     * @param time Current time (seconds)
     * @param state Current system state
     * @param control Current control vector
     * @param setpoint Target position
     */
    void update(
        double time,
        const BallState& ball,
        const TableState& table,
        const ControlVector& control,
        const Eigen::Vector2d& setpoint
    );

    /**
     * @brief Render all plots
     * @return true if plots window was visible
     *
     * Must be called between ImGui::NewFrame() and ImGui::Render()
     */
    bool render();

    /**
     * @brief Clear all data buffers
     */
    void clear();

    /**
     * @brief Set window visibility
     */
    void set_visible(bool visible) { is_visible_ = visible; }

    /**
     * @brief Check if window is visible
     */
    bool is_visible() const { return is_visible_; }

private:
    /**
     * @brief Render 3D trajectory plot (X, Y, Z) using ImPlot3D
     */
    void render_trajectory();

    /**
     * @brief Render position vs time plots
     */
    void render_position_time();

    /**
     * @brief Render control signals plot
     */
    void render_control_signals();

    /**
     * @brief Render error plot
     */
    void render_error();

    /**
     * @brief Render Z position plot (ball_z, table_z vs time)
     */
    void render_z_position();

    // System parameters
    SystemParameters params_;

    // Data storage (fixed-size ring buffer with std::array)
    DataManager data_;

    // Trajectory data (X, Y, Z) - chronologically ordered flat vectors for direct PlotLine use.
    // Filled with push_back until capacity, then rotated once per wrap (every
    // max_trajectory_points_ updates) to evict the oldest point.
    static constexpr size_t max_trajectory_points_ = 1000;
    std::vector<double> traj_flat_x_;
    std::vector<double> traj_flat_y_;
    std::vector<double> traj_flat_z_;
    bool traj_full_{false};  // true once the vectors reach capacity

    // Current setpoint (for plotting)
    Eigen::Vector2d current_setpoint_;

    // UI state
    bool is_visible_;

    // Pre-allocated buffers for plotting (avoid per-frame allocations)
    // These are reused across frames and resized as needed
    std::vector<double> times_buffer_;
    std::vector<double> x_values_buffer_;
    std::vector<double> y_values_buffer_;
    std::vector<double> varphi_x_buffer_;
    std::vector<double> theta_y_buffer_;
    std::vector<double> error_x_buffer_;
    std::vector<double> error_y_buffer_;
    std::vector<double> error_mag_buffer_;
    std::vector<double> z_ball_buffer_;
    std::vector<double> z_table_buffer_;
};

} // namespace ball_balancer
