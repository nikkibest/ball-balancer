#include <ball_balancer/gui/control_panel.hpp>
#include <cmath>

/**
 * @file control_panel.cpp
 * @brief Implementation of control panel UI
 *
 * Implements ImGui-based control panel following best practices:
 * - Always call End() even if Begin() returns false
 * - Use GetContentRegionAvail() for dynamic sizing
 * - UI reflects data directly (no state duplication)
 * - Collapsing headers for organization
 *
 * @see research/dear-imgui-cpp-gui-best-practices.md
 */

namespace ball_balancer {

ControlPanel::ControlPanel(const SystemParameters& params)
    : params_(params)
    , is_visible_(true)
    , sim_state_(SimulationState::Stopped)
    , reset_requested_(false)
    , setpoint_(Eigen::Vector2d::Zero())
    , show_advanced_tuning_(false)
{
}

bool ControlPanel::render(
    const StateVector& state,
    PIDController& controller,
    StateEstimator& estimator
) {
    // IMPORTANT: Always call End() even if Begin() returns false
    // This is a critical ImGui requirement
    if (!ImGui::Begin("Control Panel", &is_visible_)) {
        ImGui::End();
        return false;
    }

    // Render sections
    render_simulation_controls();
    ImGui::Separator();

    render_setpoint_controls();
    ImGui::Separator();

    render_pid_tuning(controller);
    ImGui::Separator();

    render_kalman_tuning(estimator);
    ImGui::Separator();

    render_system_status(state);

    ImGui::End();
    return true;
}

void ControlPanel::render_simulation_controls() {
    if (ImGui::CollapsingHeader("Simulation Control", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Use available width for buttons
        float button_width = ImGui::GetContentRegionAvail().x;

        // Start/Pause button (changes based on state)
        if (sim_state_ == SimulationState::Running) {
            // Pause button (orange color)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.5f, 0.2f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.6f, 0.3f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 0.7f, 0.4f, 1.0f));

            if (ImGui::Button("Pause", ImVec2(button_width, 0))) {
                sim_state_ = SimulationState::Paused;
            }

            ImGui::PopStyleColor(3);
        } else {
            // Play button (green color)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.2f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.8f, 0.3f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.4f, 0.9f, 0.4f, 1.0f));

            const char* label = (sim_state_ == SimulationState::Paused) ? "Resume" : "Start";
            if (ImGui::Button(label, ImVec2(button_width, 0))) {
                sim_state_ = SimulationState::Running;
            }

            ImGui::PopStyleColor(3);
        }

        // Reset button (red color)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.3f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.9f, 0.4f, 0.4f, 1.0f));

        if (ImGui::Button("Reset", ImVec2(button_width, 0))) {
            reset_requested_ = true;
            sim_state_ = SimulationState::Stopped;
        }

        ImGui::PopStyleColor(3);

        // Status indicator
        ImGui::Spacing();
        ImGui::Text("Status:");
        ImGui::SameLine();

        switch (sim_state_) {
            case SimulationState::Running:
                ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "Running");
                break;
            case SimulationState::Paused:
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Paused");
                break;
            case SimulationState::Stopped:
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Stopped");
                break;
        }
    }
}

void ControlPanel::render_setpoint_controls() {
    if (ImGui::CollapsingHeader("Target Position", ImGuiTreeNodeFlags_DefaultOpen)) {
        // X position setpoint
        float x = static_cast<float>(setpoint_.x());
        float x_limit = static_cast<float>(params_.table_length / 2.0 - params_.ball_radius);

        ImGui::Text("X Position");
        if (ImGui::SliderFloat("##setpoint_x", &x, -x_limit, x_limit, "%.3f m")) {
            setpoint_.x() = x;
        }

        // Y position setpoint
        float y = static_cast<float>(setpoint_.y());
        float y_limit = static_cast<float>(params_.table_width / 2.0 - params_.ball_radius);

        ImGui::Text("Y Position");
        if (ImGui::SliderFloat("##setpoint_y", &y, -y_limit, y_limit, "%.3f m")) {
            setpoint_.y() = y;
        }

        // Quick presets
        ImGui::Spacing();
        ImGui::Text("Presets:");

        float preset_button_width = ImGui::GetContentRegionAvail().x / 3.0f - 4.0f;

        if (ImGui::Button("Center", ImVec2(preset_button_width, 0))) {
            setpoint_ = Eigen::Vector2d::Zero();
        }

        ImGui::SameLine();
        if (ImGui::Button("Corner", ImVec2(preset_button_width, 0))) {
            setpoint_.x() = x_limit * 0.7;
            setpoint_.y() = y_limit * 0.7;
        }

        ImGui::SameLine();
        if (ImGui::Button("Edge", ImVec2(preset_button_width, 0))) {
            setpoint_.x() = x_limit * 0.8;
            setpoint_.y() = 0.0;
        }
    }
}

void ControlPanel::render_pid_tuning(PIDController& controller) {
    if (ImGui::CollapsingHeader("PID Tuning")) {
        ImGui::TextWrapped("Tune PID gains for position control. "
                          "Increase Kp for faster response, Ki for steady-state accuracy, "
                          "Kd for damping.");

        // X Axis PID gains
        if (ImGui::TreeNode("X Axis Gains")) {
            PIDGains x_gains = controller.get_x_gains();
            bool changed = false;

            ImGui::Text("Proportional (Kp)");
            changed |= ImGui::SliderFloat("##kp_x", &x_gains.Kp, 0.0f, 10.0f, "%.2f");

            ImGui::Text("Integral (Ki)");
            changed |= ImGui::SliderFloat("##ki_x", &x_gains.Ki, 0.0f, 5.0f, "%.2f");

            ImGui::Text("Derivative (Kd)");
            changed |= ImGui::SliderFloat("##kd_x", &x_gains.Kd, 0.0f, 2.0f, "%.2f");

            if (changed) {
                controller.set_x_gains(x_gains);
            }

            // Reset to defaults
            if (ImGui::Button("Reset to Default##x", ImVec2(-FLT_MIN, 0))) {
                PIDGains default_gains;
                default_gains.Kp = 1.0;
                default_gains.Ki = 0.5;
                default_gains.Kd = 0.2;
                controller.set_x_gains(default_gains);
            }

            ImGui::TreePop();
        }

        // Y Axis PID gains
        if (ImGui::TreeNode("Y Axis Gains")) {
            PIDGains y_gains = controller.get_y_gains();
            bool changed = false;

            ImGui::Text("Proportional (Kp)");
            changed |= ImGui::SliderFloat("##kp_y", &y_gains.Kp, 0.0f, 10.0f, "%.2f");

            ImGui::Text("Integral (Ki)");
            changed |= ImGui::SliderFloat("##ki_y", &y_gains.Ki, 0.0f, 5.0f, "%.2f");

            ImGui::Text("Derivative (Kd)");
            changed |= ImGui::SliderFloat("##kd_y", &y_gains.Kd, 0.0f, 2.0f, "%.2f");

            if (changed) {
                controller.set_y_gains(y_gains);
            }

            // Reset to defaults
            if (ImGui::Button("Reset to Default##y", ImVec2(-FLT_MIN, 0))) {
                PIDGains default_gains;
                default_gains.Kp = 1.0;
                default_gains.Ki = 0.5;
                default_gains.Kd = 0.2;
                controller.set_y_gains(default_gains);
            }

            ImGui::TreePop();
        }

        // Copy X gains to Y
        if (ImGui::Button("Copy X Gains to Y", ImVec2(-FLT_MIN, 0))) {
            controller.set_y_gains(controller.get_x_gains());
        }
    }
}

void ControlPanel::render_kalman_tuning(StateEstimator& estimator) {
    if (ImGui::CollapsingHeader("State Estimator")) {
        ImGui::TextWrapped("Kalman filter estimates velocity from position measurements. "
                          "Increase process noise to trust measurements more, "
                          "increase measurement noise to trust model more.");

        KalmanTuning tuning = estimator.get_tuning();
        bool changed = false;

        // Measurement noise (most commonly tuned)
        ImGui::Text("Measurement Noise (Camera)");
        ImGui::SetNextItemWidth(-FLT_MIN);
        changed |= ImGui::SliderFloat("##r_pos", &tuning.measurement_noise_position,
                                      0.001f, 0.05f, "%.4f m");

        // Show advanced tuning option
        ImGui::Checkbox("Show Advanced Tuning", &show_advanced_tuning_);

        if (show_advanced_tuning_) {
            ImGui::Spacing();
            ImGui::Text("Process Noise (Model Uncertainty)");

            ImGui::Text("Position");
            ImGui::SetNextItemWidth(-FLT_MIN);
            changed |= ImGui::SliderFloat("##q_pos", &tuning.process_noise_position,
                                          0.0001f, 0.01f, "%.4f m");

            ImGui::Text("Velocity");
            ImGui::SetNextItemWidth(-FLT_MIN);
            changed |= ImGui::SliderFloat("##q_vel", &tuning.process_noise_velocity,
                                          0.001f, 0.1f, "%.4f m/s");

            ImGui::Text("Angle");
            ImGui::SetNextItemWidth(-FLT_MIN);
            changed |= ImGui::SliderFloat("##q_angle", &tuning.process_noise_angle,
                                          0.00001f, 0.001f, "%.5f rad");
        }

        if (changed) {
            estimator.set_tuning(tuning);
        }

        // Reset to defaults
        if (ImGui::Button("Reset to Default##kalman", ImVec2(-FLT_MIN, 0))) {
            KalmanTuning default_tuning;
            default_tuning.initialize();
            estimator.set_tuning(default_tuning);
        }
    }
}

void ControlPanel::render_system_status(const StateVector& state) {
    if (ImGui::CollapsingHeader("System Status", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Ball position
        ImGui::Text("Ball Position:");
        ImGui::Indent();
        ImGui::Text("X: %.4f m", state(state_index::X));
        ImGui::Text("Y: %.4f m", state(state_index::Y));
        ImGui::Unindent();

        ImGui::Spacing();

        // Ball velocity
        ImGui::Text("Ball Velocity:");
        ImGui::Indent();
        ImGui::Text("Vx: %.4f m/s", state(state_index::VX));
        ImGui::Text("Vy: %.4f m/s", state(state_index::VY));

        // Velocity magnitude
        double vx = state(state_index::VX);
        double vy = state(state_index::VY);
        double speed = std::sqrt(vx * vx + vy * vy);
        ImGui::Text("Speed: %.4f m/s", speed);
        ImGui::Unindent();

        ImGui::Spacing();

        // Table tilt
        ImGui::Text("Table Tilt:");
        ImGui::Indent();
        double theta_x_deg = state(state_index::THETA_X) * 180.0 / M_PI;
        double theta_y_deg = state(state_index::THETA_Y) * 180.0 / M_PI;
        ImGui::Text("X: %.2f deg", theta_x_deg);
        ImGui::Text("Y: %.2f deg", theta_y_deg);
        ImGui::Unindent();

        ImGui::Spacing();

        // Error from setpoint
        ImGui::Text("Position Error:");
        ImGui::Indent();
        double error_x = setpoint_.x() - state(state_index::X);
        double error_y = setpoint_.y() - state(state_index::Y);
        double error_mag = std::sqrt(error_x * error_x + error_y * error_y);

        ImGui::Text("X: %.4f m", error_x);
        ImGui::Text("Y: %.4f m", error_y);
        ImGui::Text("Magnitude: %.4f m", error_mag);

        // Color-coded error indicator
        ImGui::Text("Status: ");
        ImGui::SameLine();
        if (error_mag < 0.01) {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "On Target");
        } else if (error_mag < 0.05) {
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.2f, 1.0f), "Near Target");
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "Off Target");
        }
        ImGui::Unindent();
    }
}

} // namespace ball_balancer
