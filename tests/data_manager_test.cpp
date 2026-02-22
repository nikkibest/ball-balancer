#include <ball_balancer/visualization/data_manager.hpp>
#include <gtest/gtest.h>
#include <cmath>

using namespace ball_balancer;

/**
 * @file data_manager_test.cpp
 * @brief Regression tests for DataManager ring buffer
 *
 * Critical regression tests for the offset() bug (fixed in Phase 2):
 * - When buffer is partially filled, offset() must return 0 (oldest at start)
 * - When buffer is full and wrapped, offset() returns index of oldest element
 */

// ============================================================================
// Regression Test: DataManager offset() Bug
// ============================================================================

/**
 * Test that offset() returns 0 while buffer is partially filled.
 * This is a regression test for the bug where offset() returned the write
 * position instead of 0 when size < CAPACITY.
 */
TEST(DataManagerRingBuffer, OffsetIsZeroWhilePartiallyFilled) {
    DataManager dm;

    // Add points up to half capacity
    for (std::size_t i = 0; i < DataManager::CAPACITY / 2; ++i) {
        DataManager::DataPoint pt;
        pt.time = static_cast<double>(i);
        pt.ball_x = static_cast<double>(i);
        dm.add_point(pt);

        // BUG (before fix): offset() returned i+1 instead of 0
        EXPECT_EQ(dm.offset(), 0u)
            << "Partial fill: offset should be 0 after " << (i + 1) << " points";
    }
}

/**
 * Test that offset() correctly wraps when buffer fills.
 */
TEST(DataManagerRingBuffer, OffsetWrapsCorrectlyWhenFull) {
    DataManager dm;

    // Fill buffer to capacity
    for (std::size_t i = 0; i < DataManager::CAPACITY; ++i) {
        DataManager::DataPoint pt;
        pt.time = static_cast<double>(i);
        dm.add_point(pt);
    }

    // After exactly CAPACITY writes, offset wraps to 0
    EXPECT_EQ(dm.offset(), 0u) << "Full buffer should have offset == 0";

    // Add one more point (wraps around)
    dm.add_point({});
    EXPECT_EQ(dm.offset(), 1u) << "Oldest element is now at index 1 after wrap";

    // Add several more and verify offset advances
    for (std::size_t i = 0; i < 10; ++i) {
        dm.add_point({});
        EXPECT_EQ(dm.offset(), (i + 2) % DataManager::CAPACITY)
            << "Offset should advance with each new point after full";
    }
}

// ============================================================================
// Ring Buffer Semantics Tests
// ============================================================================

/**
 * Test that data is accessible in chronological order using offset().
 */
TEST(DataManagerRingBuffer, ChronologicalAccessUsingOffset) {
    DataManager dm;

    // Add points with increasing timestamps
    for (std::size_t i = 0; i < DataManager::CAPACITY + 50; ++i) {
        DataManager::DataPoint pt;
        pt.time = static_cast<double>(i);
        pt.ball_x = static_cast<double>(i * 10);  // Unique identifier
        dm.add_point(pt);
    }

    // Buffer should contain the LAST 1000 points (50 through 1049)
    const auto* data_ptr = dm.data();
    const size_t offset = dm.offset();
    const size_t size = dm.size();

    ASSERT_EQ(size, DataManager::CAPACITY);

    // Verify chronological order
    for (std::size_t i = 0; i < size; ++i) {
        size_t idx = (offset + i) % DataManager::CAPACITY;
        double expected_time = static_cast<double>(50 + i);  // Points 50-1049
        EXPECT_DOUBLE_EQ(data_ptr[idx].time, expected_time)
            << "Point " << i << " should have time " << expected_time;
    }

    // Verify oldest point
    EXPECT_DOUBLE_EQ(data_ptr[offset].time, 50.0)
        << "Oldest point should be at index offset()";

    // Verify newest point (just before offset)
    size_t newest_idx = (offset > 0) ? (offset - 1) : (DataManager::CAPACITY - 1);
    EXPECT_DOUBLE_EQ(data_ptr[newest_idx].time, 1049.0)
        << "Newest point should be just before offset";
}

// ============================================================================
// Basic Functionality Tests
// ============================================================================

TEST(DataManager, DefaultConstruction) {
    DataManager dm;
    EXPECT_EQ(dm.size(), 0u);
    EXPECT_TRUE(dm.empty());
    EXPECT_EQ(dm.offset(), 0u);
}

TEST(DataManager, AddPointIncreasesSize) {
    DataManager dm;

    for (std::size_t i = 1; i <= 10; ++i) {
        dm.add_point({});
        EXPECT_EQ(dm.size(), i);
        EXPECT_FALSE(dm.empty());
    }
}

TEST(DataManager, SizeDoesNotExceedCapacity) {
    DataManager dm;

    // Add more than CAPACITY points
    for (std::size_t i = 0; i < DataManager::CAPACITY * 2; ++i) {
        dm.add_point({});
        EXPECT_LE(dm.size(), DataManager::CAPACITY)
            << "Size must not exceed CAPACITY";
    }

    EXPECT_EQ(dm.size(), DataManager::CAPACITY);
}

TEST(DataManager, ClearResetsState) {
    DataManager dm;

    // Add some points
    for (std::size_t i = 0; i < 100; ++i) {
        dm.add_point({});
    }

    dm.clear();

    EXPECT_EQ(dm.size(), 0u);
    EXPECT_TRUE(dm.empty());
    EXPECT_EQ(dm.offset(), 0u);
}

TEST(DataManager, DataPointerIsNonNull) {
    DataManager dm;
    EXPECT_NE(dm.data(), nullptr);
}

// ============================================================================
// Error Magnitude Field Test
// ============================================================================

/**
 * Test that error_magnitude field is correctly populated.
 * This tests the Phase 2 performance fix where error magnitude is pre-computed.
 */
TEST(DataManager, ErrorMagnitudePreComputed) {
    DataManager dm;

    DataManager::DataPoint pt;
    pt.error_x = 3.0;
    pt.error_y = 4.0;
    pt.error_magnitude = std::sqrt(pt.error_x * pt.error_x + pt.error_y * pt.error_y);

    dm.add_point(pt);

    const auto* data_ptr = dm.data();
    EXPECT_DOUBLE_EQ(data_ptr[0].error_magnitude, 5.0)
        << "Error magnitude should be pre-computed as sqrt(3^2 + 4^2) = 5.0";
}
