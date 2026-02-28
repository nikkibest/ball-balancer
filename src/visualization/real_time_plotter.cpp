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
    // Pre-allocate plotting buffers to avoid per-frame allocations
    const size_t max_capacity = DataManager::CAPACITY;
    times_buffer_.reserve(max_capacity);
    x_values_buffer_.reserve(max_capacity);
    y_values_buffer_.reserve(max_capacity);
    theta_x_buffer_.reserve(max_capacity);
    theta_y_buffer_.reserve(max_capacity);
    error_x_buffer_.reserve(max_capacity);
    error_y_buffer_.reserve(max_capacity);
    error_mag_buffer_.reserve(max_capacity);
    z_ball_buffer_.reserve(max_capacity);
    z_table_buffer_.reserve(max_capacity);
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
    double z = state(state_index::Z_BALL);
    double vx = state(state_index::VX);
    double vy = state(state_index::VY);
    double vz = state(state_index::VZ_BALL);
    double theta_x = control(0);  // Use control (not state) for table angles
    double theta_y = control(1);
    double table_z = state(state_index::Z_TABLE);

    // Compute errors
    double error_x = setpoint.x() - x;
    double error_y = setpoint.y() - y;
    double error_magnitude = std::sqrt(error_x * error_x + error_y * error_y);

    // Create data point with all fields
    DataManager::DataPoint point;
    point.time = time;
    point.ball_x = x;
    point.ball_y = y;
    point.ball_z = z;
    point.ball_vx = vx;
    point.ball_vy = vy;
    point.ball_vz = vz;
    point.table_theta_x = theta_x;
    point.table_theta_y = theta_y;
    point.table_z = table_z;
    point.error_x = error_x;
    point.error_y = error_y;
    point.error_magnitude = error_magnitude;
    point.setpoint_x = setpoint.x();
    point.setpoint_y = setpoint.y();

    // Add to data manager (single unified ring buffer)
    data_.add_point(point);

    // Update trajectory (X vs Y) - ring buffer implementation
    if (trajectory_size_ < max_trajectory_points_) {
        // Still filling the buffer
        trajectory_x_[trajectory_size_] = x;
        trajectory_y_[trajectory_size_] = y;
        ++trajectory_size_;
    } else {
        // Buffer full, wrap around
        trajectory_x_[trajectory_offset_] = x;
        trajectory_y_[trajectory_offset_] = y;
        trajectory_offset_ = (trajectory_offset_ + 1) % max_trajectory_points_;
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

    if (ImGui::CollapsingHeader("Z Position")) {
        render_z_position();
    }

    ImGui::End();
    return true;
}

void RealTimePlotter::clear() {
    data_.clear();
    trajectory_size_ = 0;
    trajectory_offset_ = 0;
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
    if (trajectory_size_ > 0) {
        double current_x, current_y;

        // Plot trajectory as line - pass ring buffer data directly to ImPlot
        if (trajectory_size_ < max_trajectory_points_) {
            // Buffer not full yet - plot from start
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.7f, 1.0f, 1.0f));
            ImPlot::PlotLine("Ball Path", trajectory_x_.data(), trajectory_y_.data(),
                            static_cast<int>(trajectory_size_));
            ImPlot::PopStyleColor();

            current_x = trajectory_x_[trajectory_size_ - 1];
            current_y = trajectory_y_[trajectory_size_ - 1];
        } else {
            // Buffer full - plot in two segments to handle wraparound
            // Segment 1: from offset to end of array
            size_t first_count = max_trajectory_points_ - trajectory_offset_;
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.7f, 1.0f, 1.0f));
            ImPlot::PlotLine("Ball Path##1", &trajectory_x_[trajectory_offset_], &trajectory_y_[trajectory_offset_],
                            static_cast<int>(first_count));

            // Segment 2: from start to offset (if offset > 0)
            if (trajectory_offset_ > 0) {
                ImPlot::PlotLine("Ball Path##2", trajectory_x_.data(), trajectory_y_.data(),
                                static_cast<int>(trajectory_offset_));
            }
            ImPlot::PopStyleColor();

            // Current position is just before offset
            size_t current_idx = (trajectory_offset_ == 0) ? (max_trajectory_points_ - 1) : (trajectory_offset_ - 1);
            current_x = trajectory_x_[current_idx];
            current_y = trajectory_y_[current_idx];
        }

        // Plot current position as marker
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

    // Use pre-allocated buffers (clear and reuse)
    times_buffer_.clear();
    x_values_buffer_.clear();
    y_values_buffer_.clear();

    const auto* data_ptr = data_.data();
    const size_t offset = data_.offset();
    const size_t size = data_.size();

    // Extract data from ring buffer (unwrap if needed)
    for (size_t i = 0; i < size; ++i) {
        size_t idx = (offset + i) % DataManager::CAPACITY;
        times_buffer_.push_back(data_ptr[idx].time);
        x_values_buffer_.push_back(data_ptr[idx].ball_x);
        y_values_buffer_.push_back(data_ptr[idx].ball_y);
    }

    double time_min = times_buffer_.front();
    double time_max = times_buffer_.back();

    // X position vs time
    if (!ImPlot::BeginPlot("X Position vs Time", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "X Position (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);

    ImPlot::PlotLine("X Position", times_buffer_.data(), x_values_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));

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

    ImPlot::PlotLine("Y Position", times_buffer_.data(), y_values_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));

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

    // Use pre-allocated buffers (clear and reuse)
    times_buffer_.clear();
    theta_x_buffer_.clear();
    theta_y_buffer_.clear();

    const auto* data_ptr = data_.data();
    const size_t offset = data_.offset();
    const size_t size = data_.size();

    // Extract data from ring buffer
    for (size_t i = 0; i < size; ++i) {
        size_t idx = (offset + i) % DataManager::CAPACITY;
        times_buffer_.push_back(data_ptr[idx].time);
        theta_x_buffer_.push_back(data_ptr[idx].table_theta_x * 180.0 / M_PI);
        theta_y_buffer_.push_back(data_ptr[idx].table_theta_y * 180.0 / M_PI);
    }

    double time_min = times_buffer_.front();
    double time_max = times_buffer_.back();

    // Table tilt angles
    if (!ImPlot::BeginPlot("Table Tilt Angles", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Angle (deg)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);

    // Plot theta_x
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
    ImPlot::PlotLine("Theta X", times_buffer_.data(), theta_x_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));
    ImPlot::PopStyleColor();

    // Plot theta_y
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f));
    ImPlot::PlotLine("Theta Y", times_buffer_.data(), theta_y_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));
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

    // Use pre-allocated buffers (clear and reuse)
    times_buffer_.clear();
    error_x_buffer_.clear();
    error_y_buffer_.clear();
    error_mag_buffer_.clear();

    const auto* data_ptr = data_.data();
    const size_t offset = data_.offset();
    const size_t size = data_.size();

    // Extract data from ring buffer
    for (size_t i = 0; i < size; ++i) {
        size_t idx = (offset + i) % DataManager::CAPACITY;
        times_buffer_.push_back(data_ptr[idx].time);
        error_x_buffer_.push_back(data_ptr[idx].error_x);
        error_y_buffer_.push_back(data_ptr[idx].error_y);
        error_mag_buffer_.push_back(data_ptr[idx].error_magnitude);  // Use pre-computed value
    }

    double time_min = times_buffer_.front();
    double time_max = times_buffer_.back();

    // Position error magnitude
    if (!ImPlot::BeginPlot("Position Error", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Error (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);

    // Plot X error
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
    ImPlot::PlotLine("Error X", times_buffer_.data(), error_x_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));
    ImPlot::PopStyleColor();

    // Plot Y error
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f));
    ImPlot::PlotLine("Error Y", times_buffer_.data(), error_y_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));
    ImPlot::PopStyleColor();

    // Plot error magnitude
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
    ImPlot::PlotLine("Error Magnitude", times_buffer_.data(), error_mag_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));
    ImPlot::PopStyleVar();
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();
}

void RealTimePlotter::render_z_position() {
    float plot_height = 200.0f;

    if (data_.empty()) {
        return;
    }

    // Use pre-allocated buffers (clear and reuse)
    times_buffer_.clear();
    z_ball_buffer_.clear();
    z_table_buffer_.clear();

    const auto* data_ptr = data_.data();
    const size_t offset = data_.offset();
    const size_t size = data_.size();

    // Extract data from ring buffer
    for (size_t i = 0; i < size; ++i) {
        size_t idx = (offset + i) % DataManager::CAPACITY;
        times_buffer_.push_back(data_ptr[idx].time);
        z_ball_buffer_.push_back(data_ptr[idx].ball_z);
        z_table_buffer_.push_back(data_ptr[idx].table_z);
    }

    double time_min = times_buffer_.front();
    double time_max = times_buffer_.back();

    if (!ImPlot::BeginPlot("Z Position vs Time", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Z Position (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);

    // Plot ball Z (blue)
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.7f, 1.0f, 1.0f));
    ImPlot::PlotLine("Ball Z", times_buffer_.data(), z_ball_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));
    ImPlot::PopStyleColor();

    // Plot table Z (grey)
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
    ImPlot::PlotLine("Table Z", times_buffer_.data(), z_table_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();
}

} // namespace ball_balancer
