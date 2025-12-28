#include <ball_balancer/visualization/data_manager.hpp>

/**
 * @file data_manager.cpp
 * @brief Implementation of fixed-size ring buffer
 *
 * Ring buffer algorithm:
 * - Write to buffer_[offset_]
 * - Increment offset_ (wrap around at CAPACITY)
 * - Increment size_ until CAPACITY (oldest data overwritten)
 *
 * @see research/cpp-best-practices-modern-programming.md
 */

namespace ball_balancer {

DataManager::DataManager() {
    // std::array is automatically initialized (no manual allocation needed)
    // This is RAII in action!
}

void DataManager::add_point(const DataPoint& point) {
    // Write to current position
    buffer_[offset_] = point;

    // Advance ring buffer position
    offset_ = (offset_ + 1) % CAPACITY;

    // Track size (saturates at CAPACITY)
    if (size_ < CAPACITY) {
        ++size_;
    }
}

void DataManager::clear() {
    offset_ = 0;
    size_ = 0;
    // No need to clear buffer_ contents (will be overwritten)
}

} // namespace ball_balancer
