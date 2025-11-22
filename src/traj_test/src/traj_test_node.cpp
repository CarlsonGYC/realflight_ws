#include "traj_test/traj_test_node.hpp"
#include <cmath>

TrajTestNode::TrajTestNode(int drone_id)
  : Node("traj_test_node_" + std::to_string(drone_id))
  , drone_id_(drone_id)
  , current_state_(FsmState::INIT)
  , waiting_traj_(false)
  , traj_command_sent_(false)
  , traj_started_(false)
  , traj_completed_(false)
  , current_x_(0.0)           
  , current_y_(0.0)           
  , current_z_(0.0)           
  , odom_ready_(false)        
{
  // Parameters
  circle_radius_    = this->declare_parameter("circle_radius", 2.0);
  circle_duration_  = this->declare_parameter("circle_duration", 20.0);
  circle_init_phase_= this->declare_parameter("circle_init_phase", 0.0);
  circle_times_     = this->declare_parameter("circle_times", 1);
  ramp_up_time_     = this->declare_parameter("ramp_up_time", 3.0);
  ramp_down_time_   = this->declare_parameter("ramp_down_time", 3.0);
  
  circle_center_x_  = this->declare_parameter("circle_center_x", 0.0);
  circle_center_y_  = this->declare_parameter("circle_center_y", 0.0);
  circle_center_z_  = this->declare_parameter("circle_center_z", -1.2);
  
  timer_period_     = this->declare_parameter("timer_period", 0.02);
  max_speed_        = this->declare_parameter("max_speed", -1.0);
  use_max_speed_    = (max_speed_ > 0.0);
  
  if (circle_times_ < 1) {
    RCLCPP_WARN(this->get_logger(), 
                "circle_times must be >= 1, setting to 1");
    circle_times_ = 1;
  }
  
  px4_namespace_ = get_px4_namespace(drone_id_);
  effective_duration_ = calculate_effective_duration();
  max_angular_vel_ = 2.0 * M_PI / effective_duration_;
  angular_acceleration_ = max_angular_vel_ / ramp_up_time_;
  double theta_ramp_up = 0.5 * max_angular_vel_ * ramp_up_time_;
  double theta_ramp_down = 0.5 * max_angular_vel_ * ramp_down_time_;
  double theta_ramps_total = theta_ramp_up + theta_ramp_down;
  // Total required angular displacement for N complete circles
  double theta_required = circle_times_ * 2.0 * M_PI;
  // Angular displacement needed during constant velocity phase
  double theta_constant = theta_required - theta_ramps_total;
  // Time needed at constant velocity to cover theta_constant
  total_constant_duration_ = theta_constant / max_angular_vel_;
  
  RCLCPP_INFO(this->get_logger(), 
              "=== Trajectory Test Node for Drone %d ===", drone_id_);
  RCLCPP_INFO(this->get_logger(), "Circle radius: %.2f m", circle_radius_);
  RCLCPP_INFO(this->get_logger(), "Number of circles: %d", circle_times_);
  RCLCPP_INFO(this->get_logger(), "Total time: %.2f s", 
              ramp_up_time_ + total_constant_duration_ + ramp_down_time_);
  
  // Publishers
  traj_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    px4_namespace_ + "in/trajectory_setpoint", 
    rclcpp::QoS(1));
    
  state_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "/state/command_drone_" + std::to_string(drone_id_), 
    rclcpp::QoS(10));
  
  // Subscribers
  state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/state/state_drone_" + std::to_string(drone_id_),
    rclcpp::QoS(10),
    std::bind(&TrajTestNode::state_callback, this, std::placeholders::_1));
    
  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    px4_namespace_ + "out/vehicle_odometry",
    rclcpp::SensorDataQoS(),
    std::bind(&TrajTestNode::odom_callback, this, std::placeholders::_1));
  
  // Timer using node clock (respects use_sim_time)
  timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timer_period_)
      )),
      std::bind(&TrajTestNode::timer_callback, this));
    
  RCLCPP_INFO(this->get_logger(), "Timer initialized at %.0f Hz", 1.0/timer_period_);
}

std::string TrajTestNode::get_px4_namespace(int drone_id)
{
  if (drone_id == 0) {
    return "/fmu/";
  } else {
    return "/px4_" + std::to_string(drone_id) + "/fmu/";
  }
}

double TrajTestNode::calculate_effective_duration()
{
  if (!use_max_speed_) {
    return circle_duration_;
  }
  
  double circumference = 2.0 * M_PI * circle_radius_;
  double min_duration = circumference / max_speed_;
  
  if (min_duration > circle_duration_) {
    return min_duration;
  }
  
  return circle_duration_;
}

double TrajTestNode::calculate_theta_at_time(double t)
{
  double theta = 0.0;
  double omega_max = max_angular_vel_;
  double alpha = angular_acceleration_;
  double alpha_down = omega_max / ramp_down_time_; 
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;  
  double t_down = ramp_down_time_;
  
  // Phase 1: Ramp-up
  if (t <= t_up) {
    theta = 0.5 * alpha * t * t;
  }
  // Phase 2: Constant velocity
  else if (t <= t_up + t_const) {
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double dt = t - t_up;
    theta = theta_at_t_up + omega_max * dt;
  }
  // Phase 3: Ramp-down
  else if (t <= t_up + t_const + t_down) {
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double theta_at_start_down = theta_at_t_up + omega_max * t_const;
    
    double t_start_down = t_up + t_const;
    double dt = t - t_start_down;
    theta = theta_at_start_down + omega_max * dt - 0.5 * alpha_down * dt * dt;
  }
  // Phase 4: Hold final position
  else {
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double theta_at_start_down = theta_at_t_up + omega_max * t_const;
    theta = theta_at_start_down + omega_max * t_down - 0.5 * alpha_down * t_down * t_down;
  }
  
  return theta;
}

double TrajTestNode::calculate_angular_velocity_at_time(double t)
{
  double omega_max = max_angular_vel_;
  double alpha = angular_acceleration_;
  double alpha_down = omega_max / ramp_down_time_;
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;
  double t_down = ramp_down_time_;
  double t_start_down = t_up + t_const;
  
  double current_omega = 0.0;

  if (t <= t_up) {
    current_omega = alpha * t;
  }
  else if (t <= t_start_down) {
    current_omega = omega_max;
  }
  else if (t <= t_start_down + t_down) {
    double dt_down = t - t_start_down;
    current_omega = omega_max - alpha_down * dt_down;  // Use correct deceleration
    current_omega = std::max(0.0, current_omega);
  }
  else {
    current_omega = 0.0;
  }
  
  return current_omega;
}

void TrajTestNode::state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto state = static_cast<FsmState>(msg->data);
  current_state_ = state;

  // Detect first HOVER state entry
  if (state == FsmState::HOVER &&
      !waiting_traj_ &&
      !traj_command_sent_ &&
      !traj_started_ &&
      !traj_completed_) {
    waiting_traj_ = true;
    hover_detect_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "HOVER detected, starting trajectory in 2.0s");
  }

  // Send TRAJ command after delay
  if (waiting_traj_ && !traj_command_sent_ && !traj_started_ && !traj_completed_ &&
      (this->now() - hover_detect_time_).seconds() > 2.0) {
    RCLCPP_INFO(this->get_logger(), "Commanding FSM to TRAJ state");
    send_state_command(static_cast<int>(FsmState::TRAJ));
    traj_command_sent_ = true;
  }

  // TRAJ state entered
  if (state == FsmState::TRAJ && waiting_traj_ && !traj_started_) {
    traj_started_ = true;
    waiting_traj_ = false;
    traj_command_sent_ = false;
    traj_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Trajectory generation started");
  }

  // Premature exit from TRAJ
  if (traj_started_ && state != FsmState::TRAJ && !traj_completed_) {
    traj_started_ = false;
    waiting_traj_ = false;
    traj_command_sent_ = false;
    RCLCPP_WARN(this->get_logger(), "Left TRAJ state early, resetting");
  }
}

void TrajTestNode::odom_callback(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  current_x_ = msg->position[0];
  current_y_ = msg->position[1];
  current_z_ = msg->position[2];
  odom_ready_ = true;
}

void TrajTestNode::timer_callback()
{
  if (!odom_ready_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for odometry");
    return;
  }
  
  // Generate trajectory only in TRAJ state
  if (current_state_ == FsmState::TRAJ && traj_started_ && !traj_completed_) {
    double elapsed = (this->now() - traj_start_time_).seconds();
    
    double total_time = ramp_up_time_ + total_constant_duration_ + ramp_down_time_;
    
    // Check completion
    if (elapsed >= total_time) {
      if (!traj_completed_) {
        RCLCPP_INFO(this->get_logger(), 
                    "Trajectory complete (%d circles)", circle_times_);
        send_state_command(static_cast<int>(FsmState::END_TRAJ));
        traj_completed_ = true;
      }
      return;
    }
    
    // Generate setpoint
    generate_circular_trajectory(elapsed);
  }
}

void TrajTestNode::generate_circular_trajectory(double t)
{
  double x, y, z, vx, vy, vz, yaw;
  
  // Calculate angular velocity at current time
  double current_omega = calculate_angular_velocity_at_time(t);
  
  // Calculate angular position
  double theta = calculate_theta_at_time(t);
  double theta_with_phase = theta + circle_init_phase_;
  
  // Position on circle
  x = circle_center_x_ + circle_radius_ * std::cos(theta_with_phase);
  y = circle_center_y_ + circle_radius_ * std::sin(theta_with_phase);
  z = circle_center_z_;
  
  // Linear velocity tangent to circle
  double v_linear = current_omega * circle_radius_;
  vx = -v_linear * std::sin(theta_with_phase);
  vy =  v_linear * std::cos(theta_with_phase);
  vz = 0.0;
  
  // Yaw pointing forward
  yaw = 3.1415926f;
  
  // Publish setpoint
  publish_trajectory_setpoint(x, y, z, vx, vy, vz, yaw);
  
  // Debug logging with phase info
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;
  std::string phase;
  double current_circle = 0.0;
  
  if (t <= t_up) {
    phase = "RAMP-UP";
    current_circle = 0.0;
  } else if (t <= t_up + t_const) {
    phase = "CONSTANT";
    double elapsed_const = t - t_up;
    current_circle = elapsed_const / effective_duration_;
  } else {
    phase = "RAMP-DOWN";
    current_circle = circle_times_;
  }
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "[%s] t=%.1fs | circle=%.2f/%d | v=%.2f m/s",
                       phase.c_str(), t, current_circle, circle_times_, v_linear);
}

void TrajTestNode::publish_trajectory_setpoint(
  double x, double y, double z,
  double vx, double vy, double vz,
  double yaw)
{
  px4_msgs::msg::TrajectorySetpoint msg;
  
  msg.position[0] = static_cast<float>(x);
  msg.position[1] = static_cast<float>(y);
  msg.position[2] = static_cast<float>(z);
  
  msg.velocity[0] = static_cast<float>(vx);
  msg.velocity[1] = static_cast<float>(vy);
  msg.velocity[2] = static_cast<float>(vz);
  
  msg.yaw = static_cast<float>(yaw);
  msg.timestamp = 0;  // Let PX4 assign timestamp
  
  traj_pub_->publish(msg);
}

void TrajTestNode::send_state_command(int state)
{
  std_msgs::msg::Int32 msg;
  msg.data = state;
  state_cmd_pub_->publish(msg);
  
  RCLCPP_INFO(this->get_logger(), "Sent state command: %d", state);
}