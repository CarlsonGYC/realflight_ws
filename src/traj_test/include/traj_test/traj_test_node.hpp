#ifndef TRAJ_TEST_NODE_HPP_
#define TRAJ_TEST_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "offboard_state_machine/utils.hpp"

#include <chrono>
#include <string>

class TrajTestNode : public rclcpp::Node
{
public:
  explicit TrajTestNode(int drone_id);

private:
  void timer_callback();
  void state_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  
  void generate_circular_trajectory(double t);
  void publish_trajectory_setpoint(double x, double y, double z, 
                                   double vx, double vy, double vz,
                                   double yaw);
  void send_state_command(int state);
  
  std::string get_px4_namespace(int drone_id);
  double calculate_effective_duration();
  double calculate_theta_at_time(double t);
  double calculate_angular_velocity_at_time(double t);  

  // Circle trajectory parameters
  int drone_id_;
  double circle_radius_;              // Radius of circular path [m]
  double circle_duration_;            // Duration for one complete circle [s]
  double circle_init_phase_;         // Initial phase angle of the circle [rad]
  int circle_times_;
  double ramp_up_time_;               // Time to accelerate to max speed [s]
  double ramp_down_time_;             // Time to decelerate to zero [s]
  
  // Circle center (origin) in NED frame
  double circle_center_x_;            // North position of circle center [m]
  double circle_center_y_;            // East position of circle center [m]
  double circle_center_z_;            // Down position of circle center [m] (negative = altitude)
  
  // Control parameters
  double timer_period_;               // Control loop period [s]
  double max_speed_;                  // Maximum linear speed limit [m/s] (-1 = no limit)
  bool use_max_speed_;
  
  // Calculated parameters
  double effective_duration_;         // Actual duration after speed limiting [s]
  double total_constant_duration_;
  double max_angular_vel_;            // Maximum angular velocity [rad/s]
  double angular_acceleration_;       // Angular acceleration [rad/s²]
  
  // State management
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
  
  FsmState current_state_;
  bool waiting_traj_;
  bool traj_command_sent_;
  bool traj_started_;
  bool traj_completed_;
  rclcpp::Time hover_detect_time_;
  rclcpp::Time traj_start_time_;
  double accumulated_elapsed_;        // Accumulated time since traj start [s]
  
  // Current drone position (from odometry)
  double current_x_;
  double current_y_;
  double current_z_;
  bool odom_ready_;
  
  // ROS2 interfaces
  std::string px4_namespace_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // TRAJ_TEST_NODE_HPP_