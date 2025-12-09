#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <optional>
#include <string>

// Reuse the existing FSM state numbering for compatibility with coordinators
enum class SyncFsmState {
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

// Minimum jerk segment helper for smooth setpoints
struct SyncMJerkSegment {
  Eigen::Matrix<double,6,1> ax, ay, az;
  rclcpp::Time t0;
  double T{0.0};

  static SyncMJerkSegment build(const Eigen::Vector3d& p0,
                                const Eigen::Vector3d& v0,
                                const Eigen::Vector3d& a0,
                                const Eigen::Vector3d& pf,
                                const Eigen::Vector3d& vf,
                                const Eigen::Vector3d& af,
                                double T,
                                rclcpp::Time t0);

  void sample(const rclcpp::Time& now,
              Eigen::Vector3d& p,
              Eigen::Vector3d& v,
              Eigen::Vector3d& a) const;

  bool finished(const rclcpp::Time& now) const;
};

class OffboardSyncGoto : public rclcpp::Node
{
public:
  explicit OffboardSyncGoto(int drone_id);

private:
  // Callbacks
  void timer_cb();
  void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void state_cmd_cb(const std_msgs::msg::Int32::SharedPtr msg);

  // Helpers
  void publish_offboard_mode();
  void publish_current_setpoint();
  void send_vehicle_cmd(uint16_t cmd, float p1, float p2);
  void start_mjerk_segment(const Eigen::Vector3d& p_target,
                           double duration,
                           const Eigen::Vector3d& v_target = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d& a_target = Eigen::Vector3d::Zero());
  bool load_first_waypoint();
  void compute_takeoff_target();
  std::string px4_namespace() const;

  // Publishers/subscribers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offb_mode_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_traj_sp_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_state_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_state_cmd_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  int drone_id_;
  double takeoff_alt_;
  double takeoff_time_s_;
  double goto_time_s_;
  double timer_period_s_;
  double landing_time_s_;
  double landing_max_vel_;
  double end_traj_wait_time_;
  std::string traj_base_dir_;
  bool use_raw_traj_;

  // State tracking
  SyncFsmState current_state_;
  int offb_counter_;
  bool odom_ready_;
  bool takeoff_target_ready_;
  bool goto_target_ready_;
  bool takeoff_done_{false};
  bool goto_done_{false};
  bool end_traj_timer_started_{false};

  double current_x_, current_y_, current_z_;
  Eigen::Vector3d current_vel_{0, 0, 0};

  double takeoff_x_, takeoff_y_, takeoff_z_;
  double hover_x_, hover_y_, hover_z_;
  double goto_x_, goto_y_, goto_z_;

  uint8_t nav_state_;
  uint8_t arming_state_;

  std::optional<SyncMJerkSegment> active_seg_;
  Eigen::Vector3d final_position_{0, 0, 0};
  Eigen::Vector3d final_velocity_{0, 0, 0};
  bool has_final_setpoint_{false};
  int final_setpoint_hold_count_{0};
  rclcpp::Time end_traj_start_time_;
};
