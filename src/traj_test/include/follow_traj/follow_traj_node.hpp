#ifndef FOLLOW_TRAJ_NODE_HPP
#define FOLLOW_TRAJ_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <string>
#include <map>

// Keep this enum aligned with offboard_state_machine FsmState values
enum class FsmState {
  INIT = 0,
  ARMING = 1,
  TAKEOFF = 2,
  GOTO = 3,
  HOVER = 4,
  TRAJ = 5,
  END_TRAJ = 6,
  LAND = 7,
  DONE = 8
};

struct TrajectoryPoint {
  double time;
  double x, y, z;
  double vx, vy, vz;
};

class FollowTrajNode : public rclcpp::Node
{
public:
  explicit FollowTrajNode(int drone_id, int total_drones);

private:
  // Core functions
  void timer_callback();
  void state_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void swarm_state_callback(const std_msgs::msg::Int32::SharedPtr msg, int other_drone_id);
  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  
  // Trajectory functions
  bool load_trajectory_from_csv(const std::string& filepath);
  TrajectoryPoint interpolate_trajectory(double t);
  void publish_trajectory_setpoint(double x, double y, double z,
                                   double vx, double vy, double vz,
                                   double yaw);
  void send_state_command(int state);
  
  // Utility functions
  std::string get_px4_namespace(int drone_id);
  bool all_drones_in_traj_state();
  
  // Publishers
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_cmd_pub_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> swarm_state_subs_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Drone identification
  int drone_id_;
  int total_drones_;
  std::string px4_namespace_;
  
  // FSM state tracking
  FsmState current_state_;
  std::map<int, FsmState> swarm_states_;  // Track all drone states
  
  // Trajectory data
  std::vector<TrajectoryPoint> trajectory_;
  bool trajectory_loaded_;
  
  // Trajectory execution state
  bool waiting_for_swarm_;
  bool traj_started_;
  bool traj_completed_;
  rclcpp::Time traj_start_time_;
  bool traj_time_initialized_;
  // Odometry
  double current_x_, current_y_, current_z_;
  bool odom_ready_;
  
  // Parameters
  double timer_period_;
  std::string csv_path_;
  double yaw_setpoint_;
};

#endif // FOLLOW_TRAJ_NODE_HPP