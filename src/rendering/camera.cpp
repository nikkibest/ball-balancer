#include <ball_balancer/rendering/camera.hpp>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @file camera.cpp
 * @brief Implementation of orbital camera
 *
 * Camera mathematics:
 * - Spherical to Cartesian coordinate conversion
 * - LookAt view matrix construction
 * - Perspective projection matrix
 *
 * @see research/opengl-rendering-best-practices.md
 */

namespace ball_balancer {

Camera::Camera(
    const Eigen::Vector3f& target,
    float distance,
    float azimuth,
    float elevation
)
    : target_(target)
    , distance_(distance)
    , azimuth_(azimuth)
    , elevation_(elevation)
    , up_(0.0f, 1.0f, 0.0f)  // World up is +Y
{
    // Clamp initial values
    distance_ = std::clamp(distance_, min_distance_, max_distance_);
    elevation_ = std::clamp(elevation_, min_elevation_, max_elevation_);

    // Compute initial position
    update_position();
}

Eigen::Matrix4f Camera::get_view_matrix() const {
    // LookAt matrix: transforms world space to camera space
    //
    // Camera basis vectors:
    // - Forward: normalized(target - position)
    // - Right: normalized(cross(forward, world_up))
    // - Up: cross(right, forward)
    //
    // View matrix:
    // [ Rx  Ry  Rz  -dot(R, eye) ]
    // [ Ux  Uy  Uz  -dot(U, eye) ]
    // [-Fx -Fy -Fz   dot(F, eye) ]
    // [  0   0   0        1      ]

    Eigen::Vector3f forward = (target_ - position_).normalized();
    Eigen::Vector3f right = forward.cross(up_).normalized();
    Eigen::Vector3f up = right.cross(forward).normalized();

    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    // Rotation part
    view(0, 0) = right.x();
    view(0, 1) = right.y();
    view(0, 2) = right.z();

    view(1, 0) = up.x();
    view(1, 1) = up.y();
    view(1, 2) = up.z();

    view(2, 0) = -forward.x();
    view(2, 1) = -forward.y();
    view(2, 2) = -forward.z();

    // Translation part
    view(0, 3) = -right.dot(position_);
    view(1, 3) = -up.dot(position_);
    view(2, 3) = forward.dot(position_);

    return view;
}

Eigen::Matrix4f Camera::get_projection_matrix(
    float aspect_ratio,
    float fov_degrees,
    float near_plane,
    float far_plane
) {
    // Perspective projection matrix
    //
    // [ f/aspect   0         0                    0           ]
    // [    0       f         0                    0           ]
    // [    0       0   (f+n)/(n-f)      2*f*n/(n-f)          ]
    // [    0       0        -1                    0           ]
    //
    // where f = cot(fov/2) = 1/tan(fov/2)

    const float fov_radians = fov_degrees * 3.14159265f / 180.0f;
    const float tan_half_fov = std::tan(fov_radians / 2.0f);
    const float f = 1.0f / tan_half_fov;

    const float n = near_plane;
    const float far = far_plane;

    Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();

    proj(0, 0) = f / aspect_ratio;
    proj(1, 1) = f;
    proj(2, 2) = (far + n) / (n - far);
    proj(2, 3) = (2.0f * far * n) / (n - far);
    proj(3, 2) = -1.0f;

    return proj;
}

void Camera::rotate(float delta_azimuth, float delta_elevation) {
    azimuth_ += delta_azimuth;
    elevation_ += delta_elevation;

    // Clamp elevation to prevent gimbal lock
    elevation_ = std::clamp(elevation_, min_elevation_, max_elevation_);

    // Wrap azimuth to [0, 2π]
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    while (azimuth_ > two_pi) azimuth_ -= two_pi;
    while (azimuth_ < 0.0f) azimuth_ += two_pi;

    update_position();
}

void Camera::zoom(float delta_distance) {
    distance_ += delta_distance;
    distance_ = std::clamp(distance_, min_distance_, max_distance_);
    update_position();
}

void Camera::pan(float delta_x, float delta_y) {
    // Move target in world space
    target_.x() += delta_x;
    target_.y() += delta_y;
    update_position();
}

void Camera::set_target(const Eigen::Vector3f& target) {
    target_ = target;
    update_position();
}

void Camera::reset() {
    target_ = Eigen::Vector3f::Zero();
    distance_ = 2.0f;
    azimuth_ = 0.785f;   // 45 degrees
    elevation_ = 0.524f;  // 30 degrees
    update_position();
}

void Camera::update_position() {
    // Convert spherical coordinates (azimuth, elevation, distance) to Cartesian
    //
    // Spherical coordinate system:
    // - Azimuth (θ): angle in XZ plane from +Z axis
    // - Elevation (φ): angle above XZ plane
    // - Distance (r): radius from target
    //
    // Cartesian conversion:
    // x = r * cos(φ) * sin(θ)
    // y = r * sin(φ)
    // z = r * cos(φ) * cos(θ)

    const float cos_elevation = std::cos(elevation_);
    const float sin_elevation = std::sin(elevation_);
    const float cos_azimuth = std::cos(azimuth_);
    const float sin_azimuth = std::sin(azimuth_);

    // Position relative to target
    Eigen::Vector3f offset;
    offset.x() = distance_ * cos_elevation * sin_azimuth;
    offset.y() = distance_ * sin_elevation;
    offset.z() = distance_ * cos_elevation * cos_azimuth;

    // Absolute position
    position_ = target_ + offset;
}

void Camera::on_mouse_button(int button, bool pressed, double x, double y) {
    // Track button state
    // button: 0=left, 1=right, 2=middle (GLFW convention)
    if (button == 0) {  // Left button
        left_button_down_ = pressed;
    } else if (button == 1) {  // Right button
        right_button_down_ = pressed;
    }

    // Store initial position when button is pressed
    if (pressed) {
        last_mouse_x_ = x;
        last_mouse_y_ = y;
    }
}

void Camera::on_mouse_move(double x, double y) {
    // Calculate mouse delta
    double dx = x - last_mouse_x_;
    double dy = y - last_mouse_y_;

    // Update last position
    last_mouse_x_ = x;
    last_mouse_y_ = y;

    // Only respond to mouse input if CTRL is pressed
    if (!ctrl_pressed_) {
        return;
    }

    // Left button: Pan camera
    if (left_button_down_) {
        // Convert screen-space delta to world-space pan
        // Reduced sensitivity for better control
        const float pan_speed = 0.002f;  // Reduced from 0.005f
        float pan_x = static_cast<float>(-dx) * pan_speed * distance_;
        float pan_y = static_cast<float>(dy) * pan_speed * distance_;  // Y inverted (screen Y down, world Y up)

        pan(pan_x, pan_y);
    }

    // Right button: Rotate camera
    if (right_button_down_) {
        // Convert screen-space delta to angle delta
        // Reduced sensitivity for better control
        const float rotation_speed = 0.002f;  // Reduced from 0.005f
        float delta_azimuth = static_cast<float>(-dx) * rotation_speed;
        float delta_elevation = static_cast<float>(-dy) * rotation_speed;

        rotate(delta_azimuth, delta_elevation);
    }
}

void Camera::on_mouse_scroll(double y_offset) {
    // Only respond to scroll if CTRL is pressed
    if (!ctrl_pressed_) {
        return;
    }

    // Scroll for zoom
    // Positive y_offset = scroll up = zoom in (decrease distance)
    // Negative y_offset = scroll down = zoom out (increase distance)
    const float zoom_speed = 0.2f;
    float delta_distance = static_cast<float>(-y_offset) * zoom_speed;

    zoom(delta_distance);
}

} // namespace ball_balancer
