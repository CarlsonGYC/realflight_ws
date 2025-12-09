#ifndef TRAJ_TEST_SWARM_GOTO_COORDINATOR_HPP
#define TRAJ_TEST_SWARM_GOTO_COORDINATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <map>
#include <vector>
#include <string>

class SwarmGotoCoordinator : public rclcpp::Node
{
public:
  explicit SwarmGotoCoordinator(int total_drones);

private:
  void timer_callback();
  void state_callback(const std_msgs::msg::Int32::SharedPtr msg, int drone_id);
  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg, int drone_id);
  bool all_drones_ready_for_goto() const;
  bool all_drones_ready_for_traj() const;
  void send_goto_command_to_all();
  void send_traj_command_to_all();
  std::string px4_namespace(int drone_id) const;
  bool is_altitude_ready(int drone_id) const;

  int total_drones_;
  std::vector<int> drone_ids_;
  bool goto_sent_;
  bool traj_sent_;

  std::map<int, int> drone_states_;  // drone_id -> state
  std::map<int, double> drone_z_;    // latest z (NED)
  std::map<int, bool> odom_ready_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> state_subs_;
  std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> cmd_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;

  double takeoff_alt_;
  double alt_tol_;

  static constexpr int HOVER_STATE = 4;
  static constexpr int GOTO_STATE = 3;
  static constexpr int TRAJ_STATE = 5;
};

#endif  // TRAJ_TEST_SWARM_GOTO_COORDINATOR_HPP
