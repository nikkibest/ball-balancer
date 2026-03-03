#ifndef BALL_BALANCER_DATA_MANAGER_HPP
#define BALL_BALANCER_DATA_MANAGER_HPP

#include <array>
#include <cstddef>

/**
 * @file data_manager.hpp
 * @brief Fixed-size ring buffer for time-series data storage
 *
 * Design principles:
 * - Pre-allocated fixed-size buffers (no runtime allocation)
 * - Ring buffer for efficient time-windowed data
 * - RAII-compliant (automatic cleanup)
 * - Zero-copy access for plotting
 *
 * @see research/cpp-best-practices-modern-programming.md - RAII, no new/delete
 * @see research/implot-cpp-plotting-best-practices.md - Ring buffers for streaming
 */

namespace ball_balancer {

/**
 * @brief Fixed-size ring buffer for real-time data visualization
 *
 * Stores simulation data in pre-allocated arrays. Uses ring buffer
 * pattern for time-windowed data (oldest data gets overwritten).
 *
 * Capacity is fixed at compile-time to avoid runtime allocations.
 *
 * Thread-safety: Not thread-safe. Assumes single-threaded access.
 */
class DataManager {
public:
    /// Maximum number of data points (10 seconds at 100Hz)
    static constexpr std::size_t CAPACITY = 1000;

    /// Data point structure
    struct DataPoint {
        double time{0.0};         ///< Simulation time (seconds)
        double ball_x{0.0};       ///< Ball X position (meters)
        double ball_y{0.0};       ///< Ball Y position (meters)
        double ball_z{0.0};       ///< Ball Z position (meters)
        double ball_vx{0.0};      ///< Ball X velocity (m/s)
        double ball_vy{0.0};      ///< Ball Y velocity (m/s)
        double ball_vz{0.0};      ///< Ball Z velocity (m/s)
        double table_varphi_x{0.0}; ///< Table tilt X (radians)
        double table_theta_y{0.0}; ///< Table tilt Y (radians)
        double table_z{0.0};      ///< Table Z position (meters)
        double error_x{0.0};      ///< Position error X (meters)
        double error_y{0.0};      ///< Position error Y (meters)
        double error_magnitude{0.0}; ///< Pre-computed error magnitude (meters)
        double setpoint_x{0.0};   ///< Setpoint X (meters)
        double setpoint_y{0.0};   ///< Setpoint Y (meters)
    };

    /**
     * @brief Construct empty data manager
     */
    DataManager();

    /**
     * @brief Add a data point to the ring buffer
     *
     * If buffer is full, oldest point is overwritten.
     *
     * @param point Data point to add
     */
    void add_point(const DataPoint& point);

    /**
     * @brief Get current number of valid data points
     *
     * @return Number of points (0 to CAPACITY)
     */
    std::size_t size() const { return size_; }

    /**
     * @brief Check if buffer is empty
     */
    bool empty() const { return size_ == 0; }

    /**
     * @brief Get data pointer for zero-copy access (for ImPlot)
     *
     * @return Pointer to underlying data array
     */
    const DataPoint* data() const { return buffer_.data(); }

    /**
     * @brief Get start index for ring buffer unwrapping
     *
     * When buffer is partially filled (size < CAPACITY), oldest element is at index 0.
     * When buffer is full (size == CAPACITY), returns index of oldest element.
     *
     * @return Index of oldest element for chronological iteration
     */
    std::size_t offset() const { return (size_ < CAPACITY) ? 0 : offset_; }

    /**
     * @brief Clear all data
     */
    void clear();

private:
    std::array<DataPoint, CAPACITY> buffer_; ///< Fixed-size pre-allocated buffer
    std::size_t offset_{0};    ///< Ring buffer write position
    std::size_t size_{0};      ///< Current number of valid points
};

} // namespace ball_balancer

#endif // BALL_BALANCER_DATA_MANAGER_HPP
