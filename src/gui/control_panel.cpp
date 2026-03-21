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
    , ctrl_state_(ControllerState::Stopped)
    , reset_requested_(false)
    , control_(ControlVector::Zero())
    , setpoint_(Eigen::Vector2d::Zero())
    , manual_state_(StateVector::Zero())
    , show_advanced_tuning_(false)
{
}

bool ControlPanel::render(
    const BallState& ball,
    const TableState& table,
    PIDController& controller,
    StateEstimator& estimator,
    bool in_contact,
    ArmStatus& arm_status
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

    render_state_controls(arm_status);
    ImGui::Separator();

    render_setpoint_controls();
    ImGui::Separator();

    render_pid_tuning(controller);
    ImGui::Separator();

    render_kalman_tuning(estimator);
    ImGui::Separator();

    render_arm_mechanism(arm_status);
    ImGui::Separator();

    render_system_status(ball, table, in_contact);

    ImGui::End();
    return true;
}



void ControlPanel::render_simulation_controls() {
    if (ImGui::CollapsingHeader("Simulation & Controller", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Use available width for buttons
        float button_width = ImGui::GetContentRegionAvail().x;

        // Start/Pause button (changes based on state)
        if (sim_state_ == SimulationState::Running) {
            // Pause button (orange color)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.5f, 0.2f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.6f, 0.3f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 0.7f, 0.4f, 1.0f));

            if (ImGui::Button("Pause Sim", ImVec2(button_width, 0))) {
                sim_state_ = SimulationState::Paused;
            }

            ImGui::PopStyleColor(3);
        } else {
            // Play button (green color)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.2f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.8f, 0.3f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.4f, 0.9f, 0.4f, 1.0f));

            const char* label = (sim_state_ == SimulationState::Paused) ? "Resume Sim" : "Start Sim";
            if (ImGui::Button(label, ImVec2(button_width, 0))) {
                sim_state_ = SimulationState::Running;
            }

            ImGui::PopStyleColor(3);
        }

        // Reset button (red color)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.3f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.9f, 0.4f, 0.4f, 1.0f));

        if (ImGui::Button("Reset Sim", ImVec2(button_width, 0))) {
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

        /* Controller controls */
        // Start button
        // Play button (green color)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.8f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.4f, 0.9f, 0.4f, 1.0f));

        if (ImGui::Button("Start Ctrl", ImVec2(button_width, 0))) {
            ctrl_state_ = ControllerState::Running;
        }
        ImGui::PopStyleColor(3);
        

        // Stop button (red color)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.3f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.9f, 0.4f, 0.4f, 1.0f));

        if (ImGui::Button("Stop Ctrl", ImVec2(button_width, 0))) {
            ctrl_state_ = ControllerState::Stopped;
        }

        ImGui::PopStyleColor(3);

        // Status indicator
        ImGui::Spacing();
        ImGui::Text("Status:");
        ImGui::SameLine();

        switch (ctrl_state_) {
            case ControllerState::Running:
                ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "Running");
                break;
            case ControllerState::Stopped:
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Stopped");
                break;
        }


    }
}

void ControlPanel::render_setpoint_controls() {
    if (ImGui::CollapsingHeader("Target Position", ImGuiTreeNodeFlags_DefaultOpen)) {
        // X position setpoint
        float x = static_cast<float>(setpoint_.x());
        float x_limit = static_cast<float>(params_.table_radius - params_.ball_radius);

        ImGui::Text("X Position");
        if (ImGui::SliderFloat("##setpoint_x", &x, -x_limit, x_limit, "%.3f m")) {
            setpoint_.x() = x;
        }

        // Y position setpoint
        float y = static_cast<float>(setpoint_.y());
        float y_limit = static_cast<float>(params_.table_radius - params_.ball_radius);

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

void ControlPanel::render_system_status(
    const BallState& ball, const TableState& table, bool in_contact
) {
    if (ImGui::CollapsingHeader("System Status", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Contact mode indicator
        ImGui::Text("Mode:");
        ImGui::SameLine();
        if (in_contact) {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "Contact");
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Free Flight");
        }

        ImGui::Spacing();

        // Ball position
        ImGui::Text("Ball Position:");
        ImGui::Indent();
        ImGui::Text("X: %.4f m", ball.x);
        ImGui::Text("Y: %.4f m", ball.y);
        ImGui::Text("Z: %.4f m", ball.z_ball);
        ImGui::Unindent();

        ImGui::Spacing();

        // Ball velocity
        ImGui::Text("Ball Velocity:");
        ImGui::Indent();
        ImGui::Text("Vx: %.4f m/s", ball.vx);
        ImGui::Text("Vy: %.4f m/s", ball.vy);
        ImGui::Text("Vz: %.4f m/s", ball.vz_ball);

        const double speed = std::sqrt(ball.vx * ball.vx + ball.vy * ball.vy);
        ImGui::Text("Speed: %.4f m/s", speed);
        ImGui::Unindent();

        ImGui::Spacing();

        // Table tilt
        ImGui::Text("Table Tilt:");
        ImGui::Indent();
        ImGui::Text("X: %.2f deg", table.phi   * 180.0 / M_PI);
        ImGui::Text("Y: %.2f deg", table.theta  * 180.0 / M_PI);
        ImGui::Unindent();

        ImGui::Spacing();

        // Error from setpoint
        ImGui::Text("Position Error:");
        ImGui::Indent();
        const double error_x   = setpoint_.x() - ball.x;
        const double error_y   = setpoint_.y() - ball.y;
        const double error_mag = std::sqrt(error_x * error_x + error_y * error_y);

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

bool ControlPanel::render_state_controls(ArmStatus& arm_status) {
    manual_state_changed_ = false;

    if (!ImGui::CollapsingHeader("State Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
        return false;
    }

    const bool sim_running = (sim_state_ == SimulationState::Running);
    const bool pose_mode   = (arm_status.mode == KinematicsMode::Pose);
    const bool servo_mode  = (arm_status.mode == KinematicsMode::Servo);

    // ── Kinematics mode selector ────────────────────────────────────────────
    ImGui::Text("Kinematics Mode:");
    ImGui::SameLine();
    int modeInt = servo_mode ? 1 : 0;
    if (ImGui::RadioButton("Pose (IK)##sc", modeInt == 0)) {
        if (modeInt != 0) {
            arm_status.mode = KinematicsMode::Pose;
            arm_status.mode_changed = true;
        }
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Servo (FK)##sc", modeInt == 1)) {
        if (modeInt != 1) {
            arm_status.mode = KinematicsMode::Servo;
            arm_status.mode_changed = true;
        }
    }

    ImGui::Spacing();
    ImGui::Separator();

    // ── 1. Servo angles ─────────────────────────────────────────────────────
    // Enabled: sim off (all free), or sim running + Servo mode
    ImGui::TextUnformatted("Servo Angles (deg)");
    if (pose_mode && sim_running) {ImGui::BeginDisabled();}
    for (int i = 0; i < 3; ++i) {
        float deg = static_cast<float>(arm_status.servo_cmd.alpha[i] * 180.0 / M_PI);
        char label[32];
        snprintf(label, sizeof(label), "alpha_%d##sc_srv", i);
        if (ImGui::SliderFloat(label, &deg, 0.0f, 180.0f, "%.1f")) {
            arm_status.servo_cmd.alpha[i] = static_cast<double>(deg) * M_PI / 180.0;
            arm_status.cmd_changed = true;
            if (!sim_running) {
                // Also update manual_state_ servo angles via arm_status so Application
                // can apply them; no separate manual_state_ entry for servos —
                // set the flag so Application re-runs FK→state.
                manual_state_changed_ = true;
            }
        }
    }
    if (pose_mode && sim_running) {ImGui::EndDisabled();}

    ImGui::Spacing();

    // ── 2. Elbow angles (always read-only display) ───────────────────────────
    ImGui::TextUnformatted("Elbow Angles (deg)");
    if (sim_running) {ImGui::BeginDisabled();}
    for (int i = 0; i < 3; ++i) {
        float deg = static_cast<float>(arm_status.elbow_angles.beta[i] * 180.0 / M_PI);
        char label[32];
        snprintf(label, sizeof(label), "elbow_%d##sc_elb", i);
        if (ImGui::SliderFloat(label, &deg, 0.0f, 180.0f, "%.1f")) {
            arm_status.elbow_angles.beta[i] = static_cast<double>(deg) * M_PI / 180.0;
            arm_status.cmd_changed = true;
            if (!sim_running) {
                // Also update manual_state_ servo angles via arm_status so Application
                // can apply them; no separate manual_state_ entry for servos —
                // set the flag so Application re-runs FK→state.
                manual_state_changed_ = true;
            }
        }
    }
    if (sim_running) {ImGui::EndDisabled();}    

    ImGui::Spacing();

    // ── 3. Table roll, pitch, height ─────────────────────────────────────────
    // Enabled: sim off (all free), or sim running + Pose mode
    const bool table_sliders_enabled = !sim_running || pose_mode;
    ImGui::TextUnformatted("Table Pose");
    ImGui::BeginDisabled(!table_sliders_enabled  && !sim_running);

    const float max_tilt = static_cast<float>(params_.max_tilt_angle);
    const float z_t_min  = 0.01f;
    const float z_t_max  = static_cast<float>(params_.arm_L1 + params_.arm_L2);

    float varphi_x = static_cast<float>(manual_state_(state_index::VARPHI_X));
    float theta_y  = static_cast<float>(manual_state_(state_index::THETA_Y));
    float z_table  = static_cast<float>(manual_state_(state_index::Z_TABLE));

    if (ImGui::SliderFloat("roll (phi)##sc_tbl",   &varphi_x, -max_tilt, max_tilt, "%.3f rad")) {
        manual_state_(state_index::VARPHI_X) = varphi_x;
        manual_state_changed_ = true;
    }
    if (ImGui::SliderFloat("pitch (theta)##sc_tbl", &theta_y, -max_tilt, max_tilt, "%.3f rad")) {
        manual_state_(state_index::THETA_Y) = theta_y;
        manual_state_changed_ = true;
    }
    if (ImGui::SliderFloat("height (z_t)##sc_tbl",  &z_table,  z_t_min,  z_t_max,  "%.3f m")) {
        manual_state_(state_index::Z_TABLE) = z_table;
        manual_state_changed_ = true;
    }
    ImGui::EndDisabled();

    ImGui::Spacing();

    // ── 4. Ball position ──────────────────────────────────────────────────────
    // Enabled: sim off only
    ImGui::TextUnformatted("Ball Position");
    ImGui::BeginDisabled(sim_running);

    const float half_table = static_cast<float>(params_.table_radius);
    float x      = static_cast<float>(manual_state_(state_index::X));
    float y      = static_cast<float>(manual_state_(state_index::Y));
    float z_ball = static_cast<float>(manual_state_(state_index::Z_BALL));

    if (ImGui::SliderFloat("x##sc_ball",     &x,      -half_table, half_table, "%.3f m")) {
        manual_state_(state_index::X) = x;
        manual_state_changed_ = true;
    }
    if (ImGui::SliderFloat("y##sc_ball",     &y,      -half_table, half_table, "%.3f m")) {
        manual_state_(state_index::Y) = y;
        manual_state_changed_ = true;
    }
    if (ImGui::SliderFloat("z##sc_ball",     &z_ball, -0.1f, 0.5f, "%.3f m")) {
        manual_state_(state_index::Z_BALL) = z_ball;
        manual_state_changed_ = true;
    }
    ImGui::EndDisabled();

    // Reset button (only when sim is off)
    ImGui::Spacing();
    ImGui::BeginDisabled(sim_running);
    float btn_w = ImGui::GetContentRegionAvail().x;
    if (ImGui::Button("Zero All States##sc", ImVec2(btn_w, 0))) {
        manual_state_ = StateVector::Zero();
        manual_state_(state_index::Z_TABLE) = params_.min_table_height;
        manual_state_(state_index::Z_BALL)  = params_.min_table_height + params_.ball_radius;
        // Zero servo commands too
        for (int i = 0; i < 3; ++i) arm_status.servo_cmd.alpha[i] = 0.0;
        arm_status.cmd_changed = true;
        manual_state_changed_ = true;
    }
    ImGui::EndDisabled();

    return manual_state_changed_;
}

void ControlPanel::render_arm_mechanism(ArmStatus& arm_status) {
    if (!ImGui::CollapsingHeader("Arm Mechanism")) {
        return;
    }

    // --- Show/Hide Legs toggle ---
    bool showLegs = arm_status.show_legs;
    if (ImGui::Checkbox("Show Legs", &showLegs)) {
        arm_status.show_legs = showLegs;
        arm_status.show_legs_changed = true;
    }

    ImGui::Spacing();

    // --- IK/FK status indicators ---
    ImGui::Text("IK Status:");
    ImGui::SameLine();
    if (arm_status.ik_failed) {
        ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "FAIL");
    } else {
        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "OK");
    }

    // --- FK method + result (shown in Servo mode) ---
    if (arm_status.mode == KinematicsMode::Servo) {
        ImGui::Spacing();
        ImGui::Text("FK Method:");
        ImGui::SameLine();
        int fkInt = 0;
        if (arm_status.fk_method == FKMethod::YouTubeClosedForm) fkInt = 1;
        if (arm_status.fk_method == FKMethod::GeometryBased)     fkInt = 2;
        if (ImGui::RadioButton("NR (pose)##fk", fkInt == 0)) {
            if (fkInt != 0) {
                arm_status.fk_method = FKMethod::NewtonRaphson;
                arm_status.fk_method_changed = true;
            }
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Closed-Form##fk", fkInt == 1)) {
            if (fkInt != 1) {
                arm_status.fk_method = FKMethod::YouTubeClosedForm;
                arm_status.fk_method_changed = true;
            }
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Geometry β##fk", fkInt == 2)) {
            if (fkInt != 2) {
                arm_status.fk_method = FKMethod::GeometryBased;
                arm_status.fk_method_changed = true;
            }
        }

        ImGui::Spacing();
        ImGui::TextUnformatted("FK Result:");
        ImGui::Indent();
        if (arm_status.fk_valid) {
            ImGui::Text("phi:   %.2f deg", arm_status.fk_phi   * 180.0 / M_PI);
            ImGui::Text("theta: %.2f deg", arm_status.fk_theta * 180.0 / M_PI);
            ImGui::Text("z:     %.4f m",   arm_status.fk_z);
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "FK invalid");
        }
        ImGui::Unindent();
    }

    ImGui::Spacing();

    // --- Geometry parameters ---
    if (ImGui::TreeNode("Geometry Parameters")) {
        ImGui::TextWrapped("Edit arm mechanism dimensions. Changes take effect immediately.");
        bool geomChanged = false;

        float L1 = static_cast<float>(arm_status.req_L1);
        float L2 = static_cast<float>(arm_status.req_L2);
        float Rg = static_cast<float>(arm_status.req_Rg);
        float Rt = static_cast<float>(arm_status.req_Rt);
        float zn = static_cast<float>(arm_status.req_z_nominal);

        ImGui::Text("L1 (lower link)");
        geomChanged |= ImGui::SliderFloat("##L1", &L1, 0.01f, 0.20f, "%.3f m");
        ImGui::Text("L2 (upper link)");
        geomChanged |= ImGui::SliderFloat("##L2", &L2, 0.01f, 0.20f, "%.3f m");
        ImGui::Text("Rg (ground radius)");
        geomChanged |= ImGui::SliderFloat("##Rg", &Rg, 0.03f, 0.25f, "%.3f m");
        ImGui::Text("Rt (table radius)");
        geomChanged |= ImGui::SliderFloat("##Rt", &Rt, 0.03f, 0.25f, "%.3f m");
        ImGui::Text("z_nominal (FK warm-start)");
        geomChanged |= ImGui::SliderFloat("##znom", &zn, 0.05f, 0.30f, "%.3f m");

        if (geomChanged) {
            arm_status.req_L1       = static_cast<double>(L1);
            arm_status.req_L2       = static_cast<double>(L2);
            arm_status.req_Rg       = static_cast<double>(Rg);
            arm_status.req_Rt       = static_cast<double>(Rt);
            arm_status.req_z_nominal = static_cast<double>(zn);
            arm_status.geom_changed = true;
        }

        ImGui::TreePop();
    }
}

} // namespace ball_balancer
