#ifndef OFFBOARD_STATE_MACHINE_UTILS_HPP_
#define OFFBOARD_STATE_MACHINE_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdint>

namespace offboard_utils {

// Convert ROS time to PX4 microseconds timestamp
// Uses UNIX epoch (matches PX4 expectation for real hardware)
inline uint64_t ros_time_to_px4_us(const rclcpp::Time& ros_time) {
  // Get nanoseconds since epoch and convert to microseconds
  return static_cast<uint64_t>(ros_time.nanoseconds() / 1000);
}

// Get current ROS time as PX4 microseconds
// This version takes the node's clock to ensure consistency
inline uint64_t get_timestamp_us(rclcpp::Clock::SharedPtr clock) {
  return ros_time_to_px4_us(clock->now());
}

}  // namespace offboard_utils
#endif  // OFFBOARD_STATE_MACHINE_UTILS_HPP_