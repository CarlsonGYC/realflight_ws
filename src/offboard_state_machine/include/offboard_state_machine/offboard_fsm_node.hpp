// offboard_fsm_node.hpp
// Header file for PX4 offboard control FSM
// Author: Yichao Gao

#ifndef OFFBOARD_FSM_NODE_HPP_
#define OFFBOARD_FSM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <Eigen/Dense>
#include <optional>

// Include full headers instead of forward declarations
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

// FSM states
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

// This must be a complete type (not forward declaration) because std::optional
// needs to know the size and layout of the type at compile time
struct MJerkSegment {
  Eigen::Matrix<double,6,1> ax, ay, az;  // Quintic polynomial coefficients
  rclcpp::Time t0;                        // Start time
  double T{0.0};                          // Duration
  // Static factory method to build a minimum jerk trajectory
  // Solves the boundary value problem for position, velocity, and acceleration
  static MJerkSegment build(
      const Eigen::Vector3d& p0,  // Initial position
      const Eigen::Vector3d& v0,  // Initial velocity
      const Eigen::Vector3d& a0,  // Initial acceleration
      const Eigen::Vector3d& pf,  // Final position
      const Eigen::Vector3d& vf,  // Final velocity (usually zero)
      const Eigen::Vector3d& af,  // Final acceleration (usually zero)
      double T,                    // Duration
      rclcpp::Time t0);           // Start time
  
  // Sample the trajectory at a given time
  void sample(const rclcpp::Time& now,
              Eigen::Vector3d& p,
              Eigen::Vector3d& v,
              Eigen::Vector3d& a) const;
  
  // Check if trajectory is complete
  bool finished(const rclcpp::Time& now) const;
  
  // Get maximum velocity magnitude along the trajectory
  double get_max_velocity() const;
};

// ============================================================================
// OFFBOARD FSM CLASS
// ============================================================================
class OffboardFSM : public rclcpp::Node
{
public:
  explicit OffboardFSM(int drone_id);

private:
  // Callbacks
  void timer_cb();
  void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void state_cmd_cb(const std_msgs::msg::Int32::SharedPtr msg);

  // Helper functions
  void publish_offboard_mode();
  void publish_current_setpoint();
  void send_vehicle_cmd(uint16_t cmd, float p1, float p2);
  void try_set_offboard_and_arm();
  void generate_trajectory();
  bool has_goto_target() const;
  uint64_t get_timestamp_us();
  
  // Minimum jerk trajectory planning
  void start_mjerk_segment(const Eigen::Vector3d& p_target,
                           double duration,
                           const Eigen::Vector3d& v_target = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d& a_target = Eigen::Vector3d::Zero());
  
  // Calculate optimal duration to respect velocity limits
  double calculate_optimal_duration(const Eigen::Vector3d& p_start,
                                   const Eigen::Vector3d& p_target,
                                   const Eigen::Vector3d& v_start,
                                   double max_vel) const;
  
  // Deprecated - kept for compatibility
  void calculate_goto_ramp(double& pos_x, double& pos_y, double& pos_z,
                          double& vel_x, double& vel_y, double& vel_z);
  
  // Publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offb_mode_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_traj_sp_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_state_;

  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_state_cmd_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  int drone_id_;
  double takeoff_alt_;
  double takeoff_time_s_;
  double climb_rate_;
  double landing_time_s_;
  double circle_radius_;
  double inward_offset_;
  int num_drones_;
  double timer_period_s_;
  double alt_tol_;
  double radius_;
  double period_s_;
  bool initial_arming_complete_; 
  // GOTO parameters
  double goto_x_;
  double goto_y_;
  double goto_z_;
  double goto_tol_;
  double goto_max_vel_;              // Maximum velocity constraint
  double goto_accel_time_;           // Minimum duration parameter
  rclcpp::Time goto_start_time_;
  double goto_start_x_, goto_start_y_, goto_start_z_;
  double goto_duration_;
  bool in_goto_transition_;
  
  // Payload offset
  double payload_offset_x_;
  double payload_offset_y_;

  // State variables
  FsmState current_state_;
  int offb_counter_;
  int takeoff_start_count_;
  int takeoff_complete_count_;
  int landing_start_count_;
  
  // Command tracking
  int offboard_cmd_count_;
  int arm_cmd_count_;
  int64_t last_cmd_time_;
  
  // Position tracking
  double takeoff_pos_x_;
  double takeoff_pos_y_;
  double hover_x_;
  double hover_y_;
  double hover_z_;
  double landing_x_;
  double landing_y_;
  double landing_start_z_;
  bool vel_initialized_;
  bool has_final_setpoint_;
  int final_setpoint_hold_count_;
  
  // Current position from odometry
  double current_x_;
  double current_y_;
  double current_z_;
  double last_z_;
  bool odom_ready_;
  
  // Control mode
  bool use_attitude_control_;
  
  // PX4 status
  uint8_t nav_state_;
  uint8_t arming_state_;
  
  // Time tracking
  rclcpp::Time state_start_time_;
  std::chrono::steady_clock::time_point start_time_;
  
  // Namespace
  std::string px4_ns_;

  Eigen::Vector3d final_position_{0, 0, 0};
  Eigen::Vector3d final_velocity_{0, 0, 0};
  Eigen::Vector3d final_acceleration_{0, 0, 0};
  
  // MINIMUM JERK TRAJECTORY STATE
  std::optional<MJerkSegment> active_seg_;  // Current trajectory segment
  
  // Velocity estimation for smooth transitions
  Eigen::Vector3d current_vel_{0.0, 0.0, 0.0};
  Eigen::Vector3d last_pos_{0.0, 0.0, 0.0};
};

#endif  // OFFBOARD_FSM_NODE_HPP_