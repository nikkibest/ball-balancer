#pragma once

#include <Eigen/Dense>

/**
 * @file camera.hpp
 * @brief Orbital camera for 3D visualization
 *
 * Implements an orbital camera that rotates around a target point.
 * Suitable for interactive visualization of the ball balancer system.
 *
 * Controls:
 * - Azimuth/elevation angles for rotation
 * - Distance for zoom in/out
 * - Target point to look at
 *
 * @see research/opengl-rendering-best-practices.md
 */

namespace ball_balancer {

/**
 * @brief Orbital camera for 3D scene visualization
 *
 * The camera orbits around a target point using spherical coordinates:
 * - Azimuth (θ): Rotation around vertical axis (Y)
 * - Elevation (φ): Angle above horizontal plane
 * - Distance (r): Radius from target
 *
 * Position in Cartesian coordinates:
 * x = target.x + r * cos(φ) * sin(θ)
 * y = target.y + r * sin(φ)
 * z = target.z + r * cos(φ) * cos(θ)
 */
class Camera {
public:
    /**
     * @brief Construct camera with default values
     * @param target Look-at target (default: origin)
     * @param distance Distance from target (default: 2.0)
     * @param azimuth Initial azimuth angle in radians (default: π/4)
     * @param elevation Initial elevation angle in radians (default: π/6)
     */
    Camera(
        const Eigen::Vector3f& target = Eigen::Vector3f::Zero(),
        float distance = 2.0f,
        float azimuth = 0.785f,   // 45 degrees
        float elevation = 0.524f  // 30 degrees
    );

    /**
     * @brief Get view matrix (world-to-camera transform)
     * @return 4x4 view matrix
     *
     * Computed using lookAt: view = lookAt(position, target, up)
     */
    Eigen::Matrix4f get_view_matrix() const;

    /**
     * @brief Get projection matrix
     * @param aspect_ratio Viewport width / height
     * @param fov_degrees Field of view in degrees (default: 45)
     * @param near_plane Near clipping plane (default: 0.1)
     * @param far_plane Far clipping plane (default: 100.0)
     * @return 4x4 projection matrix
     *
     * Creates perspective projection matrix.
     */
    static Eigen::Matrix4f get_projection_matrix(
        float aspect_ratio,
        float fov_degrees = 45.0f,
        float near_plane = 0.1f,
        float far_plane = 100.0f
    );

    // ========================================================================
    // Camera Control
    // ========================================================================

    /**
     * @brief Rotate camera around target
     * @param delta_azimuth Change in azimuth angle (radians)
     * @param delta_elevation Change in elevation angle (radians)
     *
     * Azimuth: rotation around Y axis (left/right)
     * Elevation: angle above horizontal (up/down, clamped to [-π/2, π/2])
     */
    void rotate(float delta_azimuth, float delta_elevation);

    /**
     * @brief Zoom camera in/out
     * @param delta_distance Change in distance (negative = zoom in)
     *
     * Distance is clamped to [min_distance, max_distance]
     */
    void zoom(float delta_distance);

    /**
     * @brief Pan camera (move target point)
     * @param delta_x Change in X (world space)
     * @param delta_y Change in Y (world space)
     */
    void pan(float delta_x, float delta_y);

    /**
     * @brief Set camera target (look-at point)
     * @param target New target position
     */
    void set_target(const Eigen::Vector3f& target);

    /**
     * @brief Reset camera to default position
     */
    void reset();

    // ========================================================================
    // Mouse Input Handling
    // ========================================================================

    /**
     * @brief Handle mouse button press/release
     * @param button Mouse button (0=left, 1=right, 2=middle)
     * @param pressed True if pressed, false if released
     * @param x Mouse X position (pixels)
     * @param y Mouse Y position (pixels)
     */
    void on_mouse_button(int button, bool pressed, double x, double y);

    /**
     * @brief Handle mouse movement
     * @param x Mouse X position (pixels)
     * @param y Mouse Y position (pixels)
     *
     * If left button down: Pan camera
     * If right button down: Rotate camera
     */
    void on_mouse_move(double x, double y);

    /**
     * @brief Handle mouse scroll
     * @param y_offset Scroll amount (positive = zoom in, negative = zoom out)
     */
    void on_mouse_scroll(double y_offset);

    // ========================================================================
    // Getters
    // ========================================================================

    Eigen::Vector3f get_position() const { return position_; }
    Eigen::Vector3f get_target() const { return target_; }
    float get_distance() const { return distance_; }
    float get_azimuth() const { return azimuth_; }
    float get_elevation() const { return elevation_; }

private:
    /**
     * @brief Update camera position from spherical coordinates
     *
     * Converts (azimuth, elevation, distance) to Cartesian position.
     */
    void update_position();

    // Camera parameters (spherical coordinates)
    Eigen::Vector3f target_;  // Look-at target
    float distance_;          // Distance from target
    float azimuth_;           // Rotation around Y axis (radians)
    float elevation_;         // Angle above horizontal (radians)

    // Derived state
    Eigen::Vector3f position_;  // Camera position (Cartesian)
    Eigen::Vector3f up_;        // Up vector (world Y)

    // Constraints
    float min_distance_{0.5f};
    float max_distance_{10.0f};
    float min_elevation_{-1.4f};  // ~-80 degrees
    float max_elevation_{1.4f};   // ~+80 degrees

    // Mouse input state
    bool left_button_down_{false};
    bool right_button_down_{false};
    double last_mouse_x_{0.0};
    double last_mouse_y_{0.0};
};

} // namespace ball_balancer
