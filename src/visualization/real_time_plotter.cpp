#include <ball_balancer/visualization/real_time_plotter.hpp>
#include <cmath>
#include <algorithm>

/**
 * @file real_time_plotter.cpp
 * @brief Implementation of real-time plotter
 *
 * Implements ImPlot-based plotting following best practices:
 * - Only call EndPlot() if BeginPlot() returns true (unlike ImGui::Begin/End)
 * - Use ring buffers for efficient streaming data
 * - Downsample if needed for performance
 * - Use appropriate plot types for different data
 *
 * CRITICAL: This requires 32-bit indices in imconfig.h!
 * Add this line to imconfig.h:
 *     #define ImDrawIdx unsigned int
 *
 * @see research/implot-cpp-plotting-best-practices.md
 */

namespace ball_balancer {

RealTimePlotter::RealTimePlotter(const SystemParameters& params)
    : params_(params)
    , data_()  // DataManager uses default constructor (RAII with std::array)
    , current_setpoint_(Eigen::Vector2d::Zero())
    , is_visible_(true)
{
}

void RealTimePlotter::update(
    double time,
    const StateVector& state,
    const ControlVector& control,
    const Eigen::Vector2d& setpoint
) {
    // Store current setpoint
    current_setpoint_ = setpoint;

    // Extract state components
    double x = state(state_index::X);
    double y = state(state_index::Y);
    double vx = state(state_index::VX);
    double vy = state(state_index::VY);
    double theta_x = control(0);  // Use control (not state) for table angles
    double theta_y = control(1);

    // Compute errors
    double error_x = setpoint.x() - x;
    double error_y = setpoint.y() - y;

    // Create data point with all fields
    DataManager::DataPoint point;
    point.time = time;
    point.ball_x = x;
    point.ball_y = y;
    point.ball_vx = vx;
    point.ball_vy = vy;
    point.table_theta_x = theta_x;
    point.table_theta_y = theta_y;
    point.error_x = error_x;
    point.error_y = error_y;
    point.setpoint_x = setpoint.x();
    point.setpoint_y = setpoint.y();

    // Add to data manager (single unified ring buffer)
    data_.add_point(point);

    // Update trajectory (X vs Y)
    trajectory_.push_back(Eigen::Vector2d(x, y));
    if (trajectory_.size() > max_trajectory_points_) {
        trajectory_.pop_front();
    }
}

bool RealTimePlotter::render() {
    // IMPORTANT: Always call End() even if Begin() returns false
    if (!ImGui::Begin("Plots", &is_visible_)) {
        ImGui::End();
        return false;
    }

    // Render plots in collapsing headers for organization
    if (ImGui::CollapsingHeader("Ball Trajectory (X vs Y)", ImGuiTreeNodeFlags_DefaultOpen)) {
        render_trajectory();
    }

    if (ImGui::CollapsingHeader("Position vs Time", ImGuiTreeNodeFlags_DefaultOpen)) {
        render_position_time();
    }

    if (ImGui::CollapsingHeader("Control Signals", ImGuiTreeNodeFlags_DefaultOpen)) {
        render_control_signals();
    }

    if (ImGui::CollapsingHeader("Position Error")) {
        render_error();
    }

    ImGui::End();
    return true;
}

void RealTimePlotter::clear() {
    data_.clear();
    trajectory_.clear();
}

void RealTimePlotter::render_trajectory() {
    // X vs Y trajectory plot (not time series)
    float plot_height = 300.0f;

    // IMPORTANT: Only call EndPlot() if BeginPlot() returns true (unlike ImGui::Begin/End)
    if (!ImPlot::BeginPlot("Ball Trajectory", ImVec2(-1, plot_height),
                           ImPlotFlags_Equal)) {
        return;
    }

    // Setup axes
    double table_x = params_.table_length / 2.0;
    double table_y = params_.table_width / 2.0;

    ImPlot::SetupAxis(ImAxis_X1, "X Position (m)");
    ImPlot::SetupAxis(ImAxis_Y1, "Y Position (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, -table_x, table_x, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -table_y, table_y, ImGuiCond_Always);

    // Plot trajectory if we have data
    if (!trajectory_.empty()) {
        // Extract X and Y into separate arrays for plotting
        std::vector<double> xs, ys;
        xs.reserve(trajectory_.size());
        ys.reserve(trajectory_.size());

        for (const auto& point : trajectory_) {
            xs.push_back(point.x());
            ys.push_back(point.y());
        }

        // Plot trajectory as line
        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.7f, 1.0f, 1.0f));
        ImPlot::PlotLine("Ball Path", xs.data(), ys.data(),
                        static_cast<int>(xs.size()));
        ImPlot::PopStyleColor();

        // Plot current position as marker
        double current_x = xs.back();
        double current_y = ys.back();

        ImPlot::PushStyleColor(ImPlotCol_MarkerFill, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
        ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 8.0f);
        ImPlot::PlotScatter("Current", &current_x, &current_y, 1);
        ImPlot::PopStyleVar();
        ImPlot::PopStyleColor();
    }

    // Plot setpoint as marker
    double setpoint_x = current_setpoint_.x();
    double setpoint_y = current_setpoint_.y();

    ImPlot::PushStyleColor(ImPlotCol_MarkerFill, ImVec4(0.2f, 1.0f, 0.2f, 1.0f));
    ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 10.0f);
    ImPlot::PushStyleVar(ImPlotStyleVar_Marker, ImPlotMarker_Cross);
    ImPlot::PlotScatter("Target", &setpoint_x, &setpoint_y, 1);
    ImPlot::PopStyleVar(2);
    ImPlot::PopStyleColor();

    // Draw table boundaries as reference
    double table_x_corners[] = {-table_x, table_x, table_x, -table_x, -table_x};
    double table_y_corners[] = {-table_y, -table_y, table_y, table_y, -table_y};

    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.5f, 0.5f, 0.5f, 0.5f));
    ImPlot::PlotLine("Table Edge", table_x_corners, table_y_corners, 5);
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();
}

void RealTimePlotter::render_position_time() {
    float plot_height = 200.0f;

    if (data_.empty()) {
        return;  // No data to plot
    }

    // Extract time series data from DataManager
    std::vector<double> times, x_values, y_values;
    times.reserve(data_.size());
    x_values.reserve(data_.size());
    y_values.reserve(data_.size());

    const auto* data_ptr = data_.data();
    const size_t offset = data_.offset();
    const size_t size = data_.size();

    // Extract data from ring buffer (unwrap if needed)
    for (size_t i = 0; i < size; ++i) {
        size_t idx = (offset + i) % DataManager::CAPACITY;
        times.push_back(data_ptr[idx].time);
        x_values.push_back(data_ptr[idx].ball_x);
        y_values.push_back(data_ptr[idx].ball_y);
    }

    double time_min = times.front();
    double time_max = times.back();

    // X position vs time
    if (!ImPlot::BeginPlot("X Position vs Time", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "X Position (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);

    ImPlot::PlotLine("X Position", times.data(), x_values.data(),
                    static_cast<int>(times.size()));

    // Plot setpoint as horizontal line
    double setpoint_x = current_setpoint_.x();
    double sp_times[] = {time_min, time_max};
    double sp_values[] = {setpoint_x, setpoint_x};

    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.2f, 1.0f, 0.2f, 0.7f));
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot::PlotLine("Target X", sp_times, sp_values, 2);
    ImPlot::PopStyleVar();
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();

    // Y position vs time
    if (!ImPlot::BeginPlot("Y Position vs Time", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Y Position (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);

    ImPlot::PlotLine("Y Position", times.data(), y_values.data(),
                    static_cast<int>(times.size()));

    // Plot setpoint as horizontal line
    double setpoint_y = current_setpoint_.y();
    double sp_times2[] = {time_min, time_max};
    double sp_values2[] = {setpoint_y, setpoint_y};

    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.2f, 1.0f, 0.2f, 0.7f));
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot::PlotLine("Target Y", sp_times2, sp_values2, 2);
    ImPlot::PopStyleVar();
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();
}

void RealTimePlotter::render_control_signals() {
    float plot_height = 200.0f;

    if (data_.empty()) {
        return;  // No data to plot
    }

    // Extract time series data from DataManager
    std::vector<double> times, theta_x_deg, theta_y_deg;
    times.reserve(data_.size());
    theta_x_deg.reserve(data_.size());
    theta_y_deg.reserve(data_.size());

    const auto* data_ptr = data_.data();
    const size_t offset = data_.offset();
    const size_t size = data_.size();

    // Extract data from ring buffer
    for (size_t i = 0; i < size; ++i) {
        size_t idx = (offset + i) % DataManager::CAPACITY;
        times.push_back(data_ptr[idx].time);
        theta_x_deg.push_back(data_ptr[idx].table_theta_x * 180.0 / M_PI);
        theta_y_deg.push_back(data_ptr[idx].table_theta_y * 180.0 / M_PI);
    }

    double time_min = times.front();
    double time_max = times.back();

    // Table tilt angles
    if (!ImPlot::BeginPlot("Table Tilt Angles", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Angle (deg)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);

    // Plot theta_x
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
    ImPlot::PlotLine("Theta X", times.data(), theta_x_deg.data(),
                    static_cast<int>(times.size()));
    ImPlot::PopStyleColor();

    // Plot theta_y
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f));
    ImPlot::PlotLine("Theta Y", times.data(), theta_y_deg.data(),
                    static_cast<int>(times.size()));
    ImPlot::PopStyleColor();

    // Plot max tilt limits as horizontal lines
    double max_tilt_deg = params_.max_tilt_angle * 180.0 / M_PI;
    double limit_times[] = {time_min, time_max};
    double limit_pos[] = {max_tilt_deg, max_tilt_deg};
    double limit_neg[] = {-max_tilt_deg, -max_tilt_deg};

    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.0f, 0.0f, 0.5f));
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 1.0f);
    ImPlot::PlotLine("Max Tilt", limit_times, limit_pos, 2);
    ImPlot::PlotLine("Min Tilt", limit_times, limit_neg, 2);
    ImPlot::PopStyleVar();
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();
}

void RealTimePlotter::render_error() {
    float plot_height = 200.0f;

    if (data_.empty()) {
        return;  // No data to plot
    }

    // Extract time series data from DataManager
    std::vector<double> times, error_x_values, error_y_values, error_mag;
    times.reserve(data_.size());
    error_x_values.reserve(data_.size());
    error_y_values.reserve(data_.size());
    error_mag.reserve(data_.size());

    const auto* data_ptr = data_.data();
    const size_t offset = data_.offset();
    const size_t size = data_.size();

    // Extract data from ring buffer
    for (size_t i = 0; i < size; ++i) {
        size_t idx = (offset + i) % DataManager::CAPACITY;
        times.push_back(data_ptr[idx].time);
        double ex = data_ptr[idx].error_x;
        double ey = data_ptr[idx].error_y;
        error_x_values.push_back(ex);
        error_y_values.push_back(ey);
        error_mag.push_back(std::sqrt(ex * ex + ey * ey));
    }

    double time_min = times.front();
    double time_max = times.back();

    // Position error magnitude
    if (!ImPlot::BeginPlot("Position Error", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Error (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);

    // Plot X error
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
    ImPlot::PlotLine("Error X", times.data(), error_x_values.data(),
                    static_cast<int>(times.size()));
    ImPlot::PopStyleColor();

    // Plot Y error
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f));
    ImPlot::PlotLine("Error Y", times.data(), error_y_values.data(),
                    static_cast<int>(times.size()));
    ImPlot::PopStyleColor();

    // Plot error magnitude
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot::PlotLine("Error Magnitude", times.data(), error_mag.data(),
                    static_cast<int>(times.size()));
    ImPlot::PopStyleVar();
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();
}

} // namespace ball_balancer
