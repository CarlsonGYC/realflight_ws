#include "follow_traj/follow_traj_node.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

FollowTrajNode::FollowTrajNode(int drone_id, int total_drones)
  : Node("follow_traj_node_" + std::to_string(drone_id))
  , drone_id_(drone_id)
  , total_drones_(total_drones)
  , current_state_(FsmState::INIT)
  , trajectory_loaded_(false)
  , waiting_for_swarm_(false)
  , traj_started_(false)
  , traj_completed_(false)
  , traj_time_initialized_(false)  // NEW FLAG
  , current_x_(0.0)
  , current_y_(0.0)
  , current_z_(0.0)
  , odom_ready_(false)
{
  // Parameters
  timer_period_ = this->declare_parameter("timer_period", 0.02);
  csv_path_ = this->declare_parameter("csv_path", 
    "data/3drone_trajectories_new/drone_" + std::to_string(drone_id_) + "_traj_smoothed_100hz.csv");
  yaw_setpoint_ = this->declare_parameter("yaw_setpoint", 3.1415926);
  
  // Initialize swarm state tracking
  for (int i = 0; i < total_drones_; i++) {
    swarm_states_[i] = FsmState::INIT;
  }
  
  px4_namespace_ = get_px4_namespace(drone_id_);
  
  RCLCPP_INFO(this->get_logger(), 
              "=== Follow Trajectory Node for Drone %d ===", drone_id_);
  RCLCPP_INFO(this->get_logger(), "Total drones in swarm: %d", total_drones_);
  RCLCPP_INFO(this->get_logger(), "CSV path: %s", csv_path_.c_str());
  
  // Load trajectory from CSV
  if (!load_trajectory_from_csv(csv_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory from CSV!");
    return;
  }
  
  // Publishers - RELIABLE QoS for critical messages
  traj_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    px4_namespace_ + "in/trajectory_setpoint", 
    rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));
    
  state_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "/state/command_drone_" + std::to_string(drone_id_), 
    rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE));
  
  // Subscribe to own state - RELIABLE QoS
  state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/state/state_drone_" + std::to_string(drone_id_),
    rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
    std::bind(&FollowTrajNode::state_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Subscribed to own state: /state/state_drone_%d", drone_id_);
  
  // Subscribe to all other drones' states - RELIABLE QoS
  for (int i = 0; i < total_drones_; i++) {
    if (i != drone_id_) {
      auto sub = this->create_subscription<std_msgs::msg::Int32>(
        "/state/state_drone_" + std::to_string(i),
        rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
        [this, i](const std_msgs::msg::Int32::SharedPtr msg) {
          this->swarm_state_callback(msg, i);
        });
      swarm_state_subs_.push_back(sub);
      RCLCPP_INFO(this->get_logger(), "Subscribed to drone %d state", i);
    }
  }
    
  // Subscribe to odometry
  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    px4_namespace_ + "out/vehicle_odometry",
    rclcpp::SensorDataQoS(),
    std::bind(&FollowTrajNode::odom_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Subscribed to odometry: %sout/vehicle_odometry", 
              px4_namespace_.c_str());
  
  // Timer
  timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timer_period_)
      )),
      std::bind(&FollowTrajNode::timer_callback, this));
    
  RCLCPP_INFO(this->get_logger(), "Timer initialized at %.0f Hz", 1.0/timer_period_);
}

std::string FollowTrajNode::get_px4_namespace(int drone_id)
{
  if (drone_id == 0) {
    return "/fmu/";
  } else {
    return "/px4_" + std::to_string(drone_id) + "/fmu/";
  }
}

bool FollowTrajNode::load_trajectory_from_csv(const std::string& filepath)
{
  std::ifstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file: %s", filepath.c_str());
    return false;
  }
  
  std::string line;
  bool first_line = true;
  int point_count = 0;
  
  while (std::getline(file, line)) {
    // Skip header line
    if (first_line) {
      first_line = false;
      continue;
    }
    
    // Skip empty lines
    if (line.empty()) continue;
    
    std::stringstream ss(line);
    std::string value;
    TrajectoryPoint point;
    
    try {
      // Parse CSV: time,x,y,z,vx,vy,vz
      std::getline(ss, value, ',');
      point.time = std::stod(value);
      
      std::getline(ss, value, ',');
      point.x = std::stod(value);
      
      std::getline(ss, value, ',');
      point.y = std::stod(value);
      
      std::getline(ss, value, ',');
      point.z = std::stod(value);
      
      std::getline(ss, value, ',');
      point.vx = std::stod(value);
      
      std::getline(ss, value, ',');
      point.vy = std::stod(value);
      
      std::getline(ss, value, ',');
      point.vz = std::stod(value);
      
      trajectory_.push_back(point);
      point_count++;
      
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Error parsing line: %s", line.c_str());
      continue;
    }
  }
  
  file.close();
  
  if (trajectory_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No valid trajectory points loaded!");
    return false;
  }
  
  trajectory_loaded_ = true;
  RCLCPP_INFO(this->get_logger(), "Loaded %d trajectory points", point_count);
  RCLCPP_INFO(this->get_logger(), "Trajectory duration: %.2f seconds", 
              trajectory_.back().time);
  
  return true;
}

TrajectoryPoint FollowTrajNode::interpolate_trajectory(double t)
{
  // If before trajectory start, return first point
  if (t <= trajectory_[0].time) {
    return trajectory_[0];
  }
  
  // If after trajectory end, return last point
  if (t >= trajectory_.back().time) {
    return trajectory_.back();
  }
  
  // Find the two points to interpolate between
  size_t i = 0;
  for (i = 0; i < trajectory_.size() - 1; i++) {
    if (trajectory_[i].time <= t && t < trajectory_[i + 1].time) {
      break;
    }
  }
  
  // Linear interpolation
  const auto& p1 = trajectory_[i];
  const auto& p2 = trajectory_[i + 1];
  
  double dt = p2.time - p1.time;
  double alpha = (t - p1.time) / dt;
  
  TrajectoryPoint result;
  result.time = t;
  result.x = p1.x + alpha * (p2.x - p1.x);
  result.y = p1.y + alpha * (p2.y - p1.y);
  result.z = p1.z + alpha * (p2.z - p1.z);
  result.vx = p1.vx + alpha * (p2.vx - p1.vx);
  result.vy = p1.vy + alpha * (p2.vy - p1.vy);
  result.vz = p1.vz + alpha * (p2.vz - p1.vz);
  
  return result;
}

bool FollowTrajNode::all_drones_in_traj_state()
{
  for (const auto& [drone_id, state] : swarm_states_) {
    if (state != FsmState::TRAJ) {
      return false;
    }
  }
  return true;
}

void FollowTrajNode::state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto old_state = current_state_;
  auto state = static_cast<FsmState>(msg->data);
  current_state_ = state;
  swarm_states_[drone_id_] = state;  // Update own state in swarm map
  
  // Log state changes
  if (old_state != state) {
    RCLCPP_WARN(this->get_logger(), 
                "STATE CHANGE: %d -> %d", 
                static_cast<int>(old_state), static_cast<int>(state));
  }
  
  // When entering TRAJ state, check if we should start waiting for swarm
  if (state == FsmState::TRAJ && !waiting_for_swarm_ && !traj_started_ && !traj_completed_) {
    waiting_for_swarm_ = true;
    RCLCPP_WARN(this->get_logger(), 
                ">>> Entered TRAJ state, waiting for swarm sync...");
    
    // OPTIMIZATION: For single drone, skip swarm synchronization
    if (total_drones_ == 1) {
      traj_started_ = true;
      waiting_for_swarm_ = false;
      // DON'T set traj_start_time_ here - let timer callback do it!
      
      RCLCPP_WARN(this->get_logger(),
                  "SINGLE DRONE - READY TO START TRAJECTORY");
      
    }
  }
  
  // Start trajectory when all drones are in TRAJ state (multi-drone case)
  if (waiting_for_swarm_ && !traj_started_ && all_drones_in_traj_state()) {
    traj_started_ = true;
    waiting_for_swarm_ = false;
    // DON'T set traj_start_time_ here - let timer callback do it!
    
    RCLCPP_WARN(this->get_logger(),
                "ALL DRONES IN TRAJ - READY TO START TRAJECTORY");
    
  }
  
  // Handle premature exit from TRAJ
  if (traj_started_ && state != FsmState::TRAJ && !traj_completed_) {
    traj_started_ = false;
    waiting_for_swarm_ = false;
    traj_time_initialized_ = false;  // Reset time initialization flag
    RCLCPP_ERROR(this->get_logger(), 
                 "!!! Left TRAJ state early (to state %d), resetting !!!", 
                 static_cast<int>(state));
  }
}

void FollowTrajNode::swarm_state_callback(
  const std_msgs::msg::Int32::SharedPtr msg, int other_drone_id)
{
  auto state = static_cast<FsmState>(msg->data);
  swarm_states_[other_drone_id] = state;
  
  // Log when other drones enter TRAJ state
  if (state == FsmState::TRAJ && waiting_for_swarm_) {
    int traj_count = 0;
    for (const auto& [id, s] : swarm_states_) {
      if (s == FsmState::TRAJ) traj_count++;
    }
    RCLCPP_INFO(this->get_logger(), 
                "Drone %d entered TRAJ (%d/%d drones ready)", 
                other_drone_id, traj_count, total_drones_);
    
    // Check if all are ready
    if (all_drones_in_traj_state() && !traj_started_) {
      traj_started_ = true;
      waiting_for_swarm_ = false;
      // DON'T set traj_start_time_ here - let timer callback do it!
      
      RCLCPP_WARN(this->get_logger(),
                  "ALL DRONES IN TRAJ - READY TO START TRAJECTORY");
      
    }
  }
}

void FollowTrajNode::odom_callback(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  current_x_ = msg->position[0];
  current_y_ = msg->position[1];
  current_z_ = msg->position[2];
  odom_ready_ = true;
}

void FollowTrajNode::timer_callback()
{
  // Debug state periodically
  static int debug_counter = 0;
  if (debug_counter++ % 100 == 0) {  // Every 2s at 50Hz
    RCLCPP_INFO(this->get_logger(), 
                "State: %d, traj_started: %d, time_init: %d, waiting_swarm: %d, completed: %d, odom: %d",
                static_cast<int>(current_state_), traj_started_, traj_time_initialized_,
                waiting_for_swarm_, traj_completed_, odom_ready_);
  }
  
  if (!odom_ready_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for odometry");
    return;
  }
  
  if (!trajectory_loaded_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Trajectory not loaded!");
    return;
  }
  
  // Execute trajectory only when started and in TRAJ state
  if (current_state_ == FsmState::TRAJ && traj_started_ && !traj_completed_) {
    
    // CRITICAL FIX: Initialize start time on FIRST timer execution, not in callback
    if (!traj_time_initialized_) {
      traj_start_time_ = this->now();
      traj_time_initialized_ = true;
      
      RCLCPP_WARN(this->get_logger(),
                  "TRAJECTORY EXECUTION STARTED (t=0.00s)");
      
    }
    
    double elapsed = (this->now() - traj_start_time_).seconds();
    
    // Check if trajectory is complete
    if (elapsed >= trajectory_.back().time) {
      if (!traj_completed_) {
        RCLCPP_WARN(this->get_logger(), 
                    "========================================");
        RCLCPP_WARN(this->get_logger(),
                    "TRAJECTORY COMPLETE - SENDING END_TRAJ");
        RCLCPP_WARN(this->get_logger(), 
                    "========================================");
        
        // Send END_TRAJ command multiple times like traj_test
        for (int i = 0; i < 5; i++) {
          send_state_command(static_cast<int>(FsmState::END_TRAJ));
        }
        
        traj_completed_ = true;
      }
      return;
    }
    
    // Interpolate and publish trajectory point
    TrajectoryPoint point = interpolate_trajectory(elapsed);
    publish_trajectory_setpoint(point.x, point.y, point.z,
                                point.vx, point.vy, point.vz,
                                yaw_setpoint_);
    
    // Debug logging
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "EXECUTING TRAJ: t=%.2fs | pos=(%.2f,%.2f,%.2f) | vel=(%.2f,%.2f,%.2f)",
                         elapsed, point.x, point.y, point.z,
                         point.vx, point.vy, point.vz);
  } else if (current_state_ == FsmState::TRAJ && waiting_for_swarm_) {
    // In TRAJ state but waiting for swarm
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "In TRAJ state, waiting for swarm synchronization...");
  }
}

void FollowTrajNode::publish_trajectory_setpoint(
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
  // msg.velocity[0] = 0.0f;
  // msg.velocity[1] = 0.0f;
  // msg.velocity[2] = 0.0f;
  
  msg.yaw = static_cast<float>(yaw);
  msg.timestamp = 0;  // Let PX4 assign timestamp
  
  traj_pub_->publish(msg);
}

void FollowTrajNode::send_state_command(int state)
{
  std_msgs::msg::Int32 msg;
  msg.data = state;
  state_cmd_pub_->publish(msg);
  
  RCLCPP_INFO(this->get_logger(), "Sent state command: %d", state);
}
