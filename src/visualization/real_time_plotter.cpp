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

    // Update 3D trajectory ring buffer (X, Y, Z)
    if (trajectory_size_ < max_trajectory_points_) {
        trajectory_x_[trajectory_size_] = x;
        trajectory_y_[trajectory_size_] = y;
        trajectory_z_[trajectory_size_] = z;
        ++trajectory_size_;
    } else {
        trajectory_x_[trajectory_offset_] = x;
        trajectory_y_[trajectory_offset_] = y;
        trajectory_z_[trajectory_offset_] = z;
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
    trajectory_size_ = 0;
    trajectory_offset_ = 0;
}

void RealTimePlotter::render_trajectory() {
    // -------------------------------------------------------------------------
    // 3D trajectory plot using isometric projection onto a 2D ImPlot canvas.
    //
    // Projection basis (right-hand, Z-up isometric):
    //   screen_u =  cos(30°)*X - cos(30°)*Y      (horizontal axis)
    //   screen_v =  sin(30°)*X + sin(30°)*Y + Z  (vertical axis)
    //
    // The plot axes are dimensionless projected coordinates; we fix the range
    // so the table always fits and add Z headroom above.
    // -------------------------------------------------------------------------
    const float plot_height = 340.0f;

    // IMPORTANT: Only call EndPlot() if BeginPlot() returns true
    if (!ImPlot::BeginPlot("Ball Trajectory (3D)", ImVec2(-1, plot_height),
                           ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText)) {
        return;
    }

    // Fix canvas range to table footprint + Z range
    const double tx = params_.table_length / 2.0;
    const double ty = params_.table_width  / 2.0;
    const double tz_max = params_.max_table_height + 0.15; // include free-flight headroom

    // Isometric projection: u = cos30*(X-Y), v = sin30*(X+Y) + Z
    // Range in u: ±cos30*(tx+ty), range in v: 0..sin30*(tx+ty)+tz_max
    const double cos30 = 0.8660254;
    const double sin30 = 0.5;
    const double u_range = cos30 * (tx + ty);
    const double v_min   = -sin30 * (tx + ty);
    const double v_max   = sin30  * (tx + ty) + tz_max;

    ImPlot::SetupAxis(ImAxis_X1, "##u", ImPlotAxisFlags_NoTickLabels);
    ImPlot::SetupAxis(ImAxis_Y1, "##v", ImPlotAxisFlags_NoTickLabels);
    ImPlot::SetupAxisLimits(ImAxis_X1, -u_range, u_range, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, v_min,    v_max,   ImGuiCond_Always);

    // Lambda: project physics (x,y,z) → isometric (u,v)
    auto proj = [&](double x, double y, double z, double& u, double& v) {
        u = cos30 * (x - y);
        v = sin30 * (x + y) + z;
    };

    ImDrawList* dl = ImPlot::GetPlotDrawList();

    // Helper: physics point → ImVec2 pixel position
    auto to_px = [&](double x, double y, double z) -> ImVec2 {
        double u, v;
        proj(x, y, z, u, v);
        return ImPlot::PlotToPixels(ImPlotPoint(u, v));
    };

    // --- Draw table surface outline (bottom face at current table Z) ---
    {
        // Get latest table Z from the most recent data point
        double z_t = 0.0;
        if (trajectory_size_ > 0) {
            if (!data_.empty()) {
                const auto* dp = data_.data();
                size_t last = (data_.offset() + data_.size() - 1) % DataManager::CAPACITY;
                z_t = dp[last].table_z;
            }
        }
        // Four corners of the table top surface
        const double cx[4] = {-tx,  tx,  tx, -tx};
        const double cy[4] = {-ty, -ty,  ty,  ty};
        ImU32 table_col = IM_COL32(180, 140, 80, 120);
        // Fill quad
        ImVec2 p0 = to_px(cx[0], cy[0], z_t);
        ImVec2 p1 = to_px(cx[1], cy[1], z_t);
        ImVec2 p2 = to_px(cx[2], cy[2], z_t);
        ImVec2 p3 = to_px(cx[3], cy[3], z_t);
        dl->AddQuadFilled(p0, p1, p2, p3, table_col);
        // Outline
        ImU32 edge_col = IM_COL32(200, 160, 90, 200);
        dl->AddQuad(p0, p1, p2, p3, edge_col, 1.5f);
        // Vertical legs to ground level
        double z_ground = 0.0;
        ImU32 leg_col = IM_COL32(140, 110, 60, 100);
        for (int i = 0; i < 4; ++i) {
            dl->AddLine(to_px(cx[i], cy[i], z_t),
                        to_px(cx[i], cy[i], z_ground), leg_col, 1.0f);
        }
        // Ground shadow outline
        ImU32 shadow_col = IM_COL32(100, 100, 100, 60);
        dl->AddQuad(to_px(cx[0], cy[0], z_ground),
                    to_px(cx[1], cy[1], z_ground),
                    to_px(cx[2], cy[2], z_ground),
                    to_px(cx[3], cy[3], z_ground), shadow_col, 1.0f);
    }

    // --- Draw projected axis labels ---
    {
        double z_t = 0.0;
        if (!data_.empty()) {
            const auto* dp = data_.data();
            size_t last = (data_.offset() + data_.size() - 1) % DataManager::CAPACITY;
            z_t = dp[last].table_z;
        }
        // X axis arrow
        dl->AddLine(to_px(0, 0, z_t), to_px(tx * 0.6, 0, z_t),
                    IM_COL32(220, 80, 80, 200), 1.5f);
        // Y axis arrow
        dl->AddLine(to_px(0, 0, z_t), to_px(0, ty * 0.6, z_t),
                    IM_COL32(80, 200, 80, 200), 1.5f);
        // Z axis arrow
        dl->AddLine(to_px(0, 0, z_t), to_px(0, 0, z_t + 0.12),
                    IM_COL32(80, 140, 220, 200), 1.5f);
    }

    // --- Draw ball trajectory path ---
    if (trajectory_size_ > 1) {
        ImU32 path_col = IM_COL32(100, 160, 255, 180);
        // Unwrap the ring buffer chronologically
        size_t count = trajectory_size_;
        size_t start = (trajectory_size_ < max_trajectory_points_)
                       ? 0 : trajectory_offset_;
        for (size_t i = 0; i + 1 < count; ++i) {
            size_t a = (start + i)     % max_trajectory_points_;
            size_t b = (start + i + 1) % max_trajectory_points_;
            // Fade older segments
            float alpha = static_cast<float>(i) / static_cast<float>(count - 1);
            ImU32 seg_col = IM_COL32(
                100, 160, 255,
                static_cast<int>(60 + 180 * alpha));
            dl->AddLine(
                to_px(trajectory_x_[a], trajectory_y_[a], trajectory_z_[a]),
                to_px(trajectory_x_[b], trajectory_y_[b], trajectory_z_[b]),
                seg_col, 1.5f);
        }
        (void)path_col;
    }

    // --- Current ball position (red filled circle) ---
    if (trajectory_size_ > 0) {
        size_t cur = (trajectory_size_ < max_trajectory_points_)
                     ? (trajectory_size_ - 1)
                     : ((trajectory_offset_ + max_trajectory_points_ - 1) % max_trajectory_points_);
        ImVec2 ball_px = to_px(trajectory_x_[cur], trajectory_y_[cur], trajectory_z_[cur]);
        dl->AddCircleFilled(ball_px, 7.0f, IM_COL32(255, 70, 70, 230));
        dl->AddCircle(ball_px, 7.0f, IM_COL32(255, 200, 200, 180), 0, 1.5f);

        // Vertical drop line from ball to table surface
        double z_t = 0.0;
        if (!data_.empty()) {
            const auto* dp = data_.data();
            size_t last = (data_.offset() + data_.size() - 1) % DataManager::CAPACITY;
            z_t = dp[last].table_z;
        }
        dl->AddLine(ball_px,
                    to_px(trajectory_x_[cur], trajectory_y_[cur], z_t),
                    IM_COL32(255, 70, 70, 80), 1.0f);
    }

    // --- Target / setpoint (green cross) ---
    {
        double sp_x = current_setpoint_.x();
        double sp_y = current_setpoint_.y();
        double z_t  = 0.0;
        if (!data_.empty()) {
            const auto* dp = data_.data();
            size_t last = (data_.offset() + data_.size() - 1) % DataManager::CAPACITY;
            z_t = dp[last].table_z;
        }
        ImVec2 sp_px = to_px(sp_x, sp_y, z_t);
        const float arm = 8.0f;
        ImU32 sp_col = IM_COL32(60, 220, 60, 220);
        dl->AddLine({sp_px.x - arm, sp_px.y}, {sp_px.x + arm, sp_px.y}, sp_col, 2.0f);
        dl->AddLine({sp_px.x, sp_px.y - arm}, {sp_px.x, sp_px.y + arm}, sp_col, 2.0f);
        dl->AddCircle(sp_px, arm * 0.7f, sp_col, 0, 1.5f);
    }

    // --- Legend overlay (drawn via ImGui on top of the plot) ---
    ImVec2 plot_pos = ImPlot::GetPlotPos();
    ImDrawList* fg = ImGui::GetWindowDrawList();
    float lx = plot_pos.x + 8.0f;
    float ly = plot_pos.y + 8.0f;
    fg->AddCircleFilled({lx + 6, ly + 6},  5.0f, IM_COL32(255,  70,  70, 230));
    fg->AddText({lx + 15, ly},  IM_COL32(255, 255, 255, 200), "Ball");
    ly += 18.0f;
    fg->AddLine({lx, ly + 6}, {lx + 12, ly + 6}, IM_COL32(60, 220, 60, 220), 2.0f);
    fg->AddText({lx + 15, ly}, IM_COL32(255, 255, 255, 200), "Target");
    ly += 18.0f;
    fg->AddLine({lx, ly + 6}, {lx + 12, ly + 6}, IM_COL32(100, 160, 255, 180), 1.5f);
    fg->AddText({lx + 15, ly}, IM_COL32(255, 255, 255, 200), "Path");

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

    // Table tilt angles
    if (!ImPlot::BeginPlot("Table Tilt Angles", ImVec2(-1, plot_height))) {
        return;
    }

    ImPlot::SetupAxis(ImAxis_X1, "Time (s)");
    ImPlot::SetupAxis(ImAxis_Y1, "Angle (deg)");
    ImPlot::SetupAxisLimits(ImAxis_X1, time_min, time_max, ImGuiCond_Always);

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
