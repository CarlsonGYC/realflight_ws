#ifndef OFFBOARD_STATE_MACHINE_UTILS_HPP_
#define OFFBOARD_STATE_MACHINE_UTILS_HPP_

#include <chrono>
#include <cstdint>

namespace offboard_utils {

// Get current timestamp in microseconds (for PX4 messages)
inline uint64_t get_timestamp_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()
  ).count();
}

}  // namespace offboard_utils
#endif  // OFFBOARD_STATE_MACHINE_UTILS_HPP_