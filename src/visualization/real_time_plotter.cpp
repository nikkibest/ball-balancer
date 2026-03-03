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
    varphi_x_buffer_.reserve(max_capacity);
    theta_y_buffer_.reserve(max_capacity);
    error_x_buffer_.reserve(max_capacity);
    error_y_buffer_.reserve(max_capacity);
    error_mag_buffer_.reserve(max_capacity);
    z_ball_buffer_.reserve(max_capacity);
    z_table_buffer_.reserve(max_capacity);

    // Pre-allocate trajectory flat vectors (chronological order for direct PlotLine use)
    traj_flat_x_.reserve(max_trajectory_points_);
    traj_flat_y_.reserve(max_trajectory_points_);
    traj_flat_z_.reserve(max_trajectory_points_);
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
    double varphi_x = control(0);  // Use control (not state) for table angles
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
    point.table_varphi_x = varphi_x;
    point.table_theta_y = theta_y;
    point.table_z = table_z;
    point.error_x = error_x;
    point.error_y = error_y;
    point.error_magnitude = error_magnitude;
    point.setpoint_x = setpoint.x();
    point.setpoint_y = setpoint.y();

    // Add to data manager (single unified ring buffer)
    data_.add_point(point);

    // Update 3D trajectory flat vectors (chronological order for direct PlotLine use).
    // While not yet full: O(1) push_back.
    // Once full: drop the oldest point (front) and append the new one (back).
    // The rotate is O(n) but happens only once per max_trajectory_points_ updates
    // (~10 s at 100 Hz), so the amortised cost is O(1) per update.
    if (!traj_full_) {
        traj_flat_x_.push_back(x);
        traj_flat_y_.push_back(y);
        traj_flat_z_.push_back(z);
        if (traj_flat_x_.size() == max_trajectory_points_) {
            traj_full_ = true;
        }
    } else {
        // Rotate left by 1 to discard oldest, then overwrite the last slot
        std::rotate(traj_flat_x_.begin(), traj_flat_x_.begin() + 1, traj_flat_x_.end());
        std::rotate(traj_flat_y_.begin(), traj_flat_y_.begin() + 1, traj_flat_y_.end());
        std::rotate(traj_flat_z_.begin(), traj_flat_z_.begin() + 1, traj_flat_z_.end());
        traj_flat_x_.back() = x;
        traj_flat_y_.back() = y;
        traj_flat_z_.back() = z;
    }
}

bool RealTimePlotter::render() {
    // IMPORTANT: Always call End() even if Begin() returns false
    if (!ImGui::Begin("Plots", &is_visible_)) {
        ImGui::End();
        return false;
    }

    // Render plots in collapsing headers for organization
    if (ImGui::CollapsingHeader("Ball Trajectory (3D)", ImGuiTreeNodeFlags_DefaultOpen)) {
        render_trajectory();
    }

    if (ImGui::CollapsingHeader("Z Position", ImGuiTreeNodeFlags_DefaultOpen)) {
        render_z_position();
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
    traj_flat_x_.clear();
    traj_flat_y_.clear();
    traj_flat_z_.clear();
    traj_full_ = false;
}

void RealTimePlotter::render_trajectory() {
    const float plot_height = 340.0f;

    const double tx = params_.table_length / 2.0;
    const double ty = params_.table_width  / 2.0;

    // Auto Z range: scan the last 1 s of trajectory (100 points at 100 Hz), add 5% margin
    double z_min_data = 0.0;
    double z_max_data = params_.ball_radius + 0.05;
    if (!traj_flat_z_.empty()) {
        constexpr size_t window = 100;
        size_t n = traj_flat_z_.size();
        auto begin = traj_flat_z_.cend() - static_cast<std::ptrdiff_t>(std::min(n, window));
        z_min_data = *begin;
        z_max_data = *begin;
        for (auto it = begin; it != traj_flat_z_.cend(); ++it) {
            if (*it < z_min_data) z_min_data = *it;
            if (*it > z_max_data) z_max_data = *it;
        }
    }
    double z_margin = (z_max_data - z_min_data) * 0.05 + 0.01;
    double z_lo = z_min_data - z_margin;
    double z_hi = z_max_data + z_margin;

    // Get latest table Z
    double z_t = 0.0;
    if (!data_.empty()) {
        const auto* dp = data_.data();
        size_t last = (data_.offset() + data_.size() - 1) % DataManager::CAPACITY;
        z_t = dp[last].table_z;
    }

    if (z_t < z_lo) {z_lo = z_t - 0.01;}
    if (z_t > z_hi) {z_hi = z_t + 0.01;}

    if (!ImPlot3D::BeginPlot("##3dtraj", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot3D::SetupAxes("Y (m)", "X (m)", "Z (m)");
    ImPlot3D::SetupAxisLimits(ImAxis3D_X, -ty, ty, ImPlot3DCond_Once);
    ImPlot3D::SetupAxisLimits(ImAxis3D_Y, -tx, tx, ImPlot3DCond_Once);
    ImPlot3D::SetupAxisLimits(ImAxis3D_Z, z_lo, z_hi, ImPlot3DCond_Always);
    // Initial view: elevation=30°, azimuth=-135° (approx. isometric)
    ImPlot3D::SetupBoxInitialRotation(30.0, -135.0);
    ImPlot3D::SetupBoxRotation(30.0, -135.0, false, ImPlot3DCond_Once);

    // --- Table surface quad ---
    {
        double qx[] = {-ty, -ty,  ty,  ty};
        double qy[] = {-tx,  tx,  tx, -tx};
        double qz[] = { z_t, z_t, z_t, z_t};
        ImPlot3D::PlotQuad("Table", qx, qy, qz, 4,
            ImPlot3DSpec(
                ImPlot3DProp_LineColor,   ImVec4(0.78f, 0.63f, 0.35f, 0.90f),
                ImPlot3DProp_FillColor,   ImVec4(0.70f, 0.55f, 0.31f, 0.45f),
                ImPlot3DProp_LineWeight,  1.5f
            ));
    }

    // --- Ball trajectory path ---
    if (traj_flat_x_.size() > 1) {
        ImPlot3D::PlotLine("Path",
            traj_flat_y_.data(), traj_flat_x_.data(), traj_flat_z_.data(),
            static_cast<int>(traj_flat_x_.size()),
            ImPlot3DSpec(
                ImPlot3DProp_LineColor,  ImVec4(0.40f, 0.63f, 1.00f, 0.80f),
                ImPlot3DProp_LineWeight, 1.5f,
                ImPlot3DProp_Flags,      ImPlot3DItemFlags_NoLegend
            ));
        // Add path entry to legend via dummy
        ImPlot3D::PlotDummy("Path",
            ImPlot3DSpec(
                ImPlot3DProp_LineColor, ImVec4(0.40f, 0.63f, 1.00f, 0.80f)
            ));
    }

    // --- Current ball position ---
    if (!traj_flat_x_.empty()) {
        double cur_x = traj_flat_y_.back();
        double cur_y = traj_flat_x_.back();
        double cur_z = traj_flat_z_.back();
        ImPlot3D::PlotScatter("Ball", &cur_x, &cur_y, &cur_z, 1,
            ImPlot3DSpec(
                ImPlot3DProp_Marker,          ImPlot3DMarker_Circle,
                ImPlot3DProp_MarkerSize,      8.0f,
                ImPlot3DProp_MarkerFillColor, ImVec4(1.00f, 0.27f, 0.27f, 0.90f),
                ImPlot3DProp_MarkerLineColor, ImVec4(1.00f, 0.78f, 0.78f, 0.80f)
            ));
    }

    // --- Target setpoint ---
    {
        double sp_x = current_setpoint_.y();
        double sp_y = current_setpoint_.x();
        double sp_z = z_t;
        ImPlot3D::PlotScatter("Target", &sp_x, &sp_y, &sp_z, 1,
            ImPlot3DSpec(
                ImPlot3DProp_Marker,          ImPlot3DMarker_Cross,
                ImPlot3DProp_MarkerSize,      10.0f,
                ImPlot3DProp_MarkerLineColor, ImVec4(0.24f, 0.86f, 0.24f, 0.90f)
            ));
    }

    ImPlot3D::EndPlot();
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

    // Auto Y range helper: returns {lo, hi} with 5% margin, min span 0.01
    auto auto_range = [](const std::vector<double>& buf, double extra_lo = 0.0,
                         double extra_hi = 0.0) -> std::pair<double, double> {
        double lo = buf[0], hi = buf[0];
        for (double v : buf) { lo = std::min(lo, v); hi = std::max(hi, v); }
        lo = std::min(lo, extra_lo);
        hi = std::max(hi, extra_hi);
        double margin = std::max((hi - lo) * 0.05, 0.005);
        return {lo - margin, hi + margin};
    };

    // X position vs time
    if (!ImPlot::BeginPlot("X Position vs Time", ImVec2(-1, plot_height))) {
        return;
    }

    {
        double setpoint_x = current_setpoint_.x();
        auto [y_lo, y_hi] = auto_range(x_values_buffer_, setpoint_x, setpoint_x);
        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
        ImPlot::SetupAxis(ImAxis_Y1, "X Position (m)");
        ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, y_lo, y_hi, ImGuiCond_Always);

        ImPlot::PlotLine("X Position", times_buffer_.data(), x_values_buffer_.data(),
                        static_cast<int>(times_buffer_.size()));

        double sp_times[] = {time_min, time_max};
        double sp_values[] = {setpoint_x, setpoint_x};
        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.2f, 1.0f, 0.2f, 0.7f));
        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
        ImPlot::PlotLine("Target X", sp_times, sp_values, 2);
        ImPlot::PopStyleVar();
        ImPlot::PopStyleColor();
    }

    ImPlot::EndPlot();

    // Y position vs time
    if (!ImPlot::BeginPlot("Y Position vs Time", ImVec2(-1, plot_height))) {
        return;
    }

    {
        double setpoint_y = current_setpoint_.y();
        auto [y_lo, y_hi] = auto_range(y_values_buffer_, setpoint_y, setpoint_y);
        ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
        ImPlot::SetupAxis(ImAxis_Y1, "Y Position (m)");
        ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, y_lo, y_hi, ImGuiCond_Always);

        ImPlot::PlotLine("Y Position", times_buffer_.data(), y_values_buffer_.data(),
                        static_cast<int>(times_buffer_.size()));

        double sp_times2[] = {time_min, time_max};
        double sp_values2[] = {setpoint_y, setpoint_y};
        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.2f, 1.0f, 0.2f, 0.7f));
        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
        ImPlot::PlotLine("Target Y", sp_times2, sp_values2, 2);
        ImPlot::PopStyleVar();
        ImPlot::PopStyleColor();
    }

    ImPlot::EndPlot();
}

void RealTimePlotter::render_control_signals() {
    float plot_height = 200.0f;

    if (data_.empty()) {
        return;  // No data to plot
    }

    // Use pre-allocated buffers (clear and reuse)
    times_buffer_.clear();
    varphi_x_buffer_.clear();
    theta_y_buffer_.clear();

    const auto* data_ptr = data_.data();
    const size_t offset = data_.offset();
    const size_t size = data_.size();

    // Extract data from ring buffer
    for (size_t i = 0; i < size; ++i) {
        size_t idx = (offset + i) % DataManager::CAPACITY;
        times_buffer_.push_back(data_ptr[idx].time);
        varphi_x_buffer_.push_back(data_ptr[idx].table_varphi_x * 180.0 / M_PI);
        theta_y_buffer_.push_back(data_ptr[idx].table_theta_y * 180.0 / M_PI);
    }

    double time_min = times_buffer_.front();
    double time_max = times_buffer_.back();

    // Auto Y range: include tilt limits and both channels
    double max_tilt_deg = params_.max_tilt_angle * 180.0 / M_PI;
    double angle_lo = -max_tilt_deg, angle_hi = max_tilt_deg;
    for (double v : varphi_x_buffer_) { angle_lo = std::min(angle_lo, v); angle_hi = std::max(angle_hi, v); }
    for (double v : theta_y_buffer_)  { angle_lo = std::min(angle_lo, v); angle_hi = std::max(angle_hi, v); }
    double angle_margin = std::max((angle_hi - angle_lo) * 0.05, 0.5);

    // Table tilt angles
    if (!ImPlot::BeginPlot("Table Tilt Angles", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Angle (deg)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, angle_lo - angle_margin, angle_hi + angle_margin, ImGuiCond_Always);

    // Plot varphi_x
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
    ImPlot::PlotLine("Theta X", times_buffer_.data(), varphi_x_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));
    ImPlot::PopStyleColor();

    // Plot theta_y
    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 0.4f, 1.0f, 1.0f));
    ImPlot::PlotLine("Theta Y", times_buffer_.data(), theta_y_buffer_.data(),
                    static_cast<int>(times_buffer_.size()));
    ImPlot::PopStyleColor();

    // Plot max tilt limits as horizontal lines
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

    // Auto Y range: include all three error series
    double err_lo = 0.0, err_hi = 0.005;
    for (double v : error_x_buffer_)   { err_lo = std::min(err_lo, v); err_hi = std::max(err_hi, v); }
    for (double v : error_y_buffer_)   { err_lo = std::min(err_lo, v); err_hi = std::max(err_hi, v); }
    for (double v : error_mag_buffer_) { err_lo = std::min(err_lo, v); err_hi = std::max(err_hi, v); }
    double err_margin = std::max((err_hi - err_lo) * 0.05, 0.005);

    // Position error magnitude
    if (!ImPlot::BeginPlot("Position Error##1", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Error (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, err_lo - err_margin, err_hi + err_margin, ImGuiCond_Always);

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

    // Auto Y range: include both ball Z and table Z
    double z_lo = 0.0, z_hi = params_.ball_radius + 0.01;
    for (double v : z_ball_buffer_)  { z_lo = std::min(z_lo, v); z_hi = std::max(z_hi, v); }
    for (double v : z_table_buffer_) { z_lo = std::min(z_lo, v); z_hi = std::max(z_hi, v); }
    double z_margin = std::max((z_hi - z_lo) * 0.05, 0.005);

    if (!ImPlot::BeginPlot("Z Position vs Time", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Z Position (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, z_lo - z_margin, z_hi + z_margin, ImGuiCond_Always);

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
