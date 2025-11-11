#include "traj_test/traj_test_node.hpp"
#include <cmath>

TrajTestNode::TrajTestNode(int drone_id)
  : Node("traj_test_node_" + std::to_string(drone_id))
  , drone_id_(drone_id)
  , current_state_(FsmState::INIT)
  , waiting_traj_(false)
  , traj_command_sent_(false)
  , traj_started_(false)
  , current_x_(0.0)           
  , current_y_(0.0)           
  , current_z_(0.0)           
  , odom_ready_(false)        
{
  // Params
  circle_radius_    = this->declare_parameter("circle_radius", 2.0);
  circle_duration_  = this->declare_parameter("circle_duration", 20.0);
  circle_init_phase_= this->declare_parameter("circle_init_phase", 0.0);  // radians
  circle_times_     = this->declare_parameter("circle_times", 1);  // Number of circles
  ramp_up_time_     = this->declare_parameter("ramp_up_time", 3.0);
  ramp_down_time_   = this->declare_parameter("ramp_down_time", 3.0);
  
  // Circle center
  circle_center_x_  = this->declare_parameter("circle_center_x", 0.0);
  circle_center_y_  = this->declare_parameter("circle_center_y", 0.0);
  circle_center_z_  = this->declare_parameter("circle_center_z", -1.2);
  
  timer_period_     = this->declare_parameter("timer_period", 0.02);
  
  // Max speed
  max_speed_        = this->declare_parameter("max_speed", -1.0);
  use_max_speed_    = (max_speed_ > 0.0);
  
  // Validate circle_times
  if (circle_times_ < 1) {
    RCLCPP_WARN(this->get_logger(), 
                "circle_times must be >= 1, setting to 1");
    circle_times_ = 1;
  }
  
  // PX4 namespace
  px4_namespace_ = get_px4_namespace(drone_id_);
  
  // Effective duration (one circle)
  effective_duration_ = calculate_effective_duration();
  
  // Max angular velocity (one circle)
  max_angular_vel_ = 2.0 * M_PI / effective_duration_;
  
  // Angular accel
  angular_acceleration_ = max_angular_vel_ / ramp_up_time_;
  
  // Total constant phase duration (all circles)
  total_constant_duration_ = effective_duration_ * circle_times_;
  
  RCLCPP_INFO(this->get_logger(), 
              "=== Trajectory Test Node for Drone %d ===", drone_id_);

  RCLCPP_INFO(this->get_logger(),
              "Circle Parameters:");
  RCLCPP_INFO(this->get_logger(),
              "  - Radius: %.2f m", circle_radius_);
  RCLCPP_INFO(this->get_logger(),
              "  - Center (NED): [%.2f, %.2f, %.2f] m",
              circle_center_x_, circle_center_y_, circle_center_z_);
  RCLCPP_INFO(this->get_logger(),
              "  - Altitude: %.2f m (above ground)",
              -circle_center_z_);
  RCLCPP_INFO(this->get_logger(),
              "  - Number of circles: %d", circle_times_);
  
  RCLCPP_INFO(this->get_logger(),
              "Timing:");
  RCLCPP_INFO(this->get_logger(),
              "  - Duration per circle: %.2f s", effective_duration_);
  RCLCPP_INFO(this->get_logger(),
              "  - Total constant speed duration: %.2f s (%.2f s × %d circles)", 
              total_constant_duration_, effective_duration_, circle_times_);
  RCLCPP_INFO(this->get_logger(),
              "  - Ramp-up time: %.2f s", ramp_up_time_);
  RCLCPP_INFO(this->get_logger(),
              "  - Ramp-down time: %.2f s", ramp_down_time_);
  RCLCPP_INFO(this->get_logger(),
              "  - Total trajectory time: %.2f s", 
              ramp_up_time_ + total_constant_duration_ + ramp_down_time_);
  
  RCLCPP_INFO(this->get_logger(),
              "Velocity Parameters:");
  RCLCPP_INFO(this->get_logger(),
              "  - Max angular velocity: %.3f rad/s (%.1f deg/s)",
              max_angular_vel_, max_angular_vel_ * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(),
              "  - Angular acceleration: %.3f rad/s²",
              angular_acceleration_);
  
  double max_linear_vel = max_angular_vel_ * circle_radius_;
  RCLCPP_INFO(this->get_logger(),
              "  - Max linear velocity: %.3f m/s",
              max_linear_vel);
  
  // Calculate total angle traveled
  double total_angle_rad = 2.0 * M_PI * circle_times_;
  RCLCPP_INFO(this->get_logger(),
              "  - Total angle during constant phase: %.1f rad (%.1f degrees, %d circles)",
              total_angle_rad, total_angle_rad * 180.0 / M_PI, circle_times_);
  
  if (use_max_speed_) {
    double nominal_speed = (2.0 * M_PI * circle_radius_) / circle_duration_;
    RCLCPP_INFO(this->get_logger(), 
                "Max speed limit: %.2f m/s (nominal: %.2f m/s)", 
                max_speed_, nominal_speed);
    if (effective_duration_ != circle_duration_) {
      RCLCPP_WARN(this->get_logger(), 
                  "Duration adjusted to %.2f s to respect max_speed limit", 
                  effective_duration_);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), 
              "PX4 namespace: %s", px4_namespace_.c_str());
  RCLCPP_INFO(this->get_logger(), "==========================================");
  
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
  
  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_),
    std::bind(&TrajTestNode::timer_callback, this));
    
  RCLCPP_INFO(this->get_logger(), "Trajectory test node initialized");
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
  
  // Circumference
  double circumference = 2.0 * M_PI * circle_radius_;
  
  // Minimum duration based on max speed
  double min_duration = circumference / max_speed_;
  
  // Use larger of configured or minimum required
  if (min_duration > circle_duration_) {
    return min_duration;
  }
  
  return circle_duration_;
}

// Compute angular position at time t.
// Accounts for ramp-up, constant, ramp-down, and stop phases.
double TrajTestNode::calculate_theta_at_time(double t)
{
  double theta = 0.0;
  double omega_max = max_angular_vel_;
  double alpha = angular_acceleration_;
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;  
  double t_down = ramp_down_time_;
  
  // Phase 1: Ramp-up (0 to t_up)
  if (t <= t_up) {
    theta = 0.5 * alpha * t * t;
  }
  // Phase 2: Constant velocity (t_up to t_up + t_const)
  else if (t <= t_up + t_const) {
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double dt = t - t_up;
    theta = theta_at_t_up + omega_max * dt;
  }
  // Phase 3: Ramp-down (t_up + t_const to t_up + t_const + t_down)
  else if (t <= t_up + t_const + t_down) {
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double theta_at_start_down = theta_at_t_up + omega_max * t_const;
    
    double t_start_down = t_up + t_const;
    double dt = t - t_start_down;
    
    theta = theta_at_start_down + omega_max * dt - 0.5 * alpha * dt * dt;
  }
  // Phase 4: Stopped (beyond trajectory time)
  else {
    // Final theta at end of ramp-down
    // double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    // double theta_at_start_down = theta_at_t_up + omega_max * t_const;
    // theta = theta_at_start_down + omega_max * t_down - 0.5 * alpha * t_down * t_down;
    theta = 0.0;
  }
  
  return theta;
}

// Compute angular velocity at time t.
double TrajTestNode::calculate_angular_velocity_at_time(double t)
{
  double omega_max = max_angular_vel_;
  double alpha = angular_acceleration_;
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;  // Use total duration
  double t_down = ramp_down_time_;
  double t_start_down = t_up + t_const;
  
  double current_omega = 0.0;

  // Ramp-up
  if (t <= t_up) {
    current_omega = alpha * t;
  }
  // Constant velocity
  else if (t <= t_start_down) {
    current_omega = omega_max;
  }
  // Ramp-down
  else if (t <= t_start_down + t_down) {
    double dt_down = t - t_start_down;
    current_omega = omega_max - alpha * dt_down;
    current_omega = std::max(0.0, current_omega);
  }
  // Stopped
  else {
    current_omega = 0.0;
  }
  
  return current_omega;
}

void TrajTestNode::state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto state = static_cast<FsmState>(msg->data);
  
  // Update current state
  current_state_ = state;

  // first HOVER detection
  if (state == FsmState::HOVER &&
      !waiting_traj_ &&
      !traj_command_sent_ &&
      !traj_started_) {
    waiting_traj_ = true;
    hover_detect_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "HOVER detected, will start trajectory in 2.0s");
  }

  if (waiting_traj_ && !traj_started_ && !traj_completed_ &&
      (this->now() - hover_detect_time_).seconds() > 2.0) {
    RCLCPP_INFO(this->get_logger(), "Commanding state machine to TRAJ state");
    send_state_command(static_cast<int>(FsmState::TRAJ));
    traj_command_sent_ = true;
    traj_start_time_ = this->now();
  }

  // TRAJ detected
  if (state == FsmState::TRAJ && waiting_traj_ && !traj_started_) {
    traj_started_ = true;
    waiting_traj_ = false;
    traj_command_sent_ = false;
    traj_start_time_ = this->now();
    accumulated_elapsed_ = 0.0;
    RCLCPP_INFO(this->get_logger(),
                "FSM entered TRAJ, trajectory generation started");
  }

  // Exited TRAJ, reset
  if (traj_started_ && state != FsmState::TRAJ) {
    traj_started_ = false;
    waiting_traj_ = false;
    traj_command_sent_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Left TRAJ state, resetting");
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
                         "Waiting for odometry...");
    return;
  }
  
  // Only run in TRAJ state
  if (current_state_ == FsmState::TRAJ && traj_started_ && !traj_completed_) {
    // // Elapsed time since trajectory start
  // if (traj_started_) {
    // double elapsed = (this->now() - traj_start_time_).seconds();
    
    // // Check if complete
    // double total_time = ramp_up_time_ + total_constant_duration_ + ramp_down_time_;
    // if (elapsed >= total_time) {
    //   RCLCPP_INFO(this->get_logger(), 
    //               "Circular trajectory complete (%d circles)", circle_times_);
    //   send_state_command(static_cast<int>(FsmState::END_TRAJ));
    //   traj_started_ = false;
    //   return;
    accumulated_elapsed_ += timer_period_;
    double total_time = ramp_up_time_ + total_constant_duration_ + ramp_down_time_;
    if (accumulated_elapsed_ >= total_time) {
      RCLCPP_INFO(this->get_logger(), 
                  "Circular trajectory complete (%d circles)", circle_times_);
      send_state_command(static_cast<int>(FsmState::END_TRAJ));
      // traj_started_ = false;
      traj_completed_ = true;
      return;
    }
    
    // Generate trajectory point
    generate_circular_trajectory(accumulated_elapsed_);
  }
}

// Generate circular trajectory setpoint
void TrajTestNode::generate_circular_trajectory(double t)
{
  double x, y, z, vx, vy, vz, yaw;
  
  // angular velocity
  double current_omega = calculate_angular_velocity_at_time(t);
  
  // angular position, may exceed one full rotation
  double theta = calculate_theta_at_time(t);
  double theta_with_phase = theta + circle_init_phase_;
  // position on circle
  x = circle_center_x_ + circle_radius_ * std::cos(theta_with_phase);
  y = circle_center_y_ + circle_radius_ * std::sin(theta_with_phase);
  z = circle_center_z_;  // constant altitude
  
  // linear velocity
  double v_linear = current_omega * circle_radius_;
  
  // velocity direction
  vx = -v_linear * std::sin(theta_with_phase);
  vy =  v_linear * std::cos(theta_with_phase);
  vz = 0.0;
  
  // yaw aligned with motion
  // yaw = theta_with_phase + M_PI / 2.0;
  yaw = 0.0;
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  
  // Publish
  publish_trajectory_setpoint(x, y, z, vx, vy, vz, yaw);
  
  // Debug (throttled) with circle count
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;
  std::string phase;
  double current_circle = 0.0;
  
  if (t <= t_up) {
    phase = "RAMP-UP";
    current_circle = 0.0;
  } else if (t <= t_up + t_const) {
    phase = "CONSTANT";
    // which circle
    double elapsed_const = t - t_up;
    current_circle = elapsed_const / effective_duration_;
  } else {
    phase = "RAMP-DOWN";
    current_circle = circle_times_;
  }
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "[%s] t=%.1fs | circle=%.2f/%d | pos=[%.2f,%.2f,%.2f] | v_lin=%.2f m/s",
                       phase.c_str(), t, current_circle, circle_times_,
                       x, y, z, v_linear);
}

void TrajTestNode::publish_trajectory_setpoint(
  double x, double y, double z,
  double vx, double vy, double vz,
  double yaw)
{
  px4_msgs::msg::TrajectorySetpoint msg;
  
  msg.position[0] = x;
  msg.position[1] = y;
  msg.position[2] = z;
  
  msg.velocity[0] = vx;
  msg.velocity[1] = vy;
  msg.velocity[2] = vz;
  
  msg.yaw = yaw;
  
  msg.timestamp = offboard_utils::get_timestamp_us();
  
  traj_pub_->publish(msg);
}

void TrajTestNode::send_state_command(int state)
{
  std_msgs::msg::Int32 msg;
  msg.data = state;
  state_cmd_pub_->publish(msg);
  
  RCLCPP_INFO(this->get_logger(), "Sent state command: %d", state);
}