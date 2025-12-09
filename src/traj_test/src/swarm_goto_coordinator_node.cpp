#include "traj_test/swarm_goto_coordinator_node.hpp"
#include <chrono>

SwarmGotoCoordinator::SwarmGotoCoordinator(int total_drones)
  : Node("swarm_goto_coordinator")
  , total_drones_(total_drones)
  , goto_sent_(false)
  , traj_sent_(false)
{
  RCLCPP_INFO(this->get_logger(), "=== Swarm GOTO Coordinator for %d drones ===", total_drones_);

  this->declare_parameter<std::vector<int64_t>>("drone_ids", std::vector<int64_t>{});
  takeoff_alt_ = this->declare_parameter<double>("takeoff_alt", 0.4);
  alt_tol_ = this->declare_parameter<double>("alt_tol", 0.05);
  std::vector<int64_t> drone_ids_param = this->get_parameter("drone_ids").as_integer_array();

  if (drone_ids_param.empty()) {
    RCLCPP_WARN(this->get_logger(), "No drone_ids provided, defaulting to 0..%d", total_drones_ - 1);
    for (int i = 0; i < total_drones_; i++) {
      drone_ids_.push_back(i);
    }
  } else {
    for (auto id : drone_ids_param) {
      drone_ids_.push_back(static_cast<int>(id));
    }
  }

  if (drone_ids_.size() != static_cast<size_t>(total_drones_)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Mismatch: total_drones=%d but got %zu drone IDs",
                 total_drones_, drone_ids_.size());
    throw std::runtime_error("Drone ID count mismatch");
  }

  std::stringstream ss;
  ss << "Using drone IDs: [";
  for (size_t i = 0; i < drone_ids_.size(); i++) {
    ss << drone_ids_[i];
    if (i < drone_ids_.size() - 1) ss << ", ";
  }
  ss << "]";
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  for (int drone_id : drone_ids_) {
    drone_states_[drone_id] = -1;
    odom_ready_[drone_id] = false;
    drone_z_[drone_id] = 0.0;
  }

  for (int drone_id : drone_ids_) {
    auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    auto sub = this->create_subscription<std_msgs::msg::Int32>(
      "/state/state_drone_" + std::to_string(drone_id),
      qos,
      [this, drone_id](const std_msgs::msg::Int32::SharedPtr msg) {
        this->state_callback(msg, drone_id);
      });
    state_subs_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "Subscribed to /state/state_drone_%d", drone_id);
  }

  for (int drone_id : drone_ids_) {
    auto qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    auto pub = this->create_publisher<std_msgs::msg::Int32>(
      "/state/command_drone_" + std::to_string(drone_id),
      qos);
    cmd_pubs_.push_back(pub);
    RCLCPP_INFO(this->get_logger(), "Created command publisher for drone %d", drone_id);
  }

  for (int drone_id : drone_ids_) {
    auto odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      px4_namespace(drone_id) + "out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      [this, drone_id](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        this->odom_callback(msg, drone_id);
      });
    odom_subs_.push_back(odom_sub);
    RCLCPP_INFO(this->get_logger(), "Subscribed to odometry for drone %d", drone_id);
  }

  timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(0.1)
      )),
      std::bind(&SwarmGotoCoordinator::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Coordinator timer started at 10 Hz");
}

void SwarmGotoCoordinator::state_callback(const std_msgs::msg::Int32::SharedPtr msg, int drone_id)
{
  int old_state = drone_states_[drone_id];
  drone_states_[drone_id] = msg->data;
  if (old_state != msg->data) {
    RCLCPP_INFO(this->get_logger(),
                "Drone %d: state %d -> %d%s",
                drone_id, old_state, msg->data,
                (msg->data == HOVER_STATE) ? " (HOVER)" : "");
  }
}

void SwarmGotoCoordinator::send_goto_command_to_all()
{
  std_msgs::msg::Int32 cmd;
  cmd.data = GOTO_STATE;

  RCLCPP_WARN(this->get_logger(), "Broadcasting GOTO command to all drones");

  for (int repeat = 0; repeat < 5; repeat++) {
    for (size_t i = 0; i < drone_ids_.size(); i++) {
      cmd_pubs_[i]->publish(cmd);
      RCLCPP_INFO(this->get_logger(),
                  "  -> Sent GOTO (state=%d) to drone %d (attempt %d/5)",
                  GOTO_STATE, drone_ids_[i], repeat + 1);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  goto_sent_ = true;
  RCLCPP_WARN(this->get_logger(), "GOTO commands sent successfully");
}

void SwarmGotoCoordinator::send_traj_command_to_all()
{
  std_msgs::msg::Int32 cmd;
  cmd.data = TRAJ_STATE;

  RCLCPP_WARN(this->get_logger(), "Broadcasting TRAJ command to all drones");

  for (int repeat = 0; repeat < 5; repeat++) {
    for (size_t i = 0; i < drone_ids_.size(); i++) {
      cmd_pubs_[i]->publish(cmd);
      RCLCPP_INFO(this->get_logger(),
                  "  -> Sent TRAJ (state=%d) to drone %d (attempt %d/5)",
                  TRAJ_STATE, drone_ids_[i], repeat + 1);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  traj_sent_ = true;
  RCLCPP_WARN(this->get_logger(), "TRAJ commands sent successfully");
}

void SwarmGotoCoordinator::timer_callback()
{
  static int tick = 0;
  tick++;

  if (!goto_sent_) {
    if (all_drones_ready_for_goto()) {
      RCLCPP_INFO(this->get_logger(), "All drones at takeoff altitude, sending synchronized GOTO");
      send_goto_command_to_all();
    } else {
      if (tick % 20 == 0) {
        int ready_count = 0;
        int odom_count = 0;
        for (int id : drone_ids_) {
          if (odom_ready_.at(id)) odom_count++;
          if (is_altitude_ready(id)) ready_count++;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Waiting for takeoff altitude: %d/%d ready (odom %d/%d)",
                    ready_count, total_drones_, odom_count, total_drones_);
      }
    }
    return;
  }

  if (!traj_sent_) {
    if (all_drones_ready_for_traj()) {
      RCLCPP_INFO(this->get_logger(), "All drones in HOVER after GOTO, sending synchronized TRAJ");
      send_traj_command_to_all();
    } else {
      if (tick % 20 == 0) {
        int hover_count = 0;
        int unknown_count = 0;
        for (const auto& [drone_id, state] : drone_states_) {
          if (state == HOVER_STATE) hover_count++;
          if (state == -1) unknown_count++;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Waiting post-GOTO: %d/%d in HOVER, unknown=%d",
                    hover_count, total_drones_, unknown_count);
      }
    }
  }
}

void SwarmGotoCoordinator::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg, int drone_id)
{
  drone_z_[drone_id] = msg->position[2];
  odom_ready_[drone_id] = true;
}

bool SwarmGotoCoordinator::is_altitude_ready(int drone_id) const
{
  auto it_ready = odom_ready_.find(drone_id);
  if (it_ready == odom_ready_.end() || !it_ready->second) {
    return false;
  }
  auto it_z = drone_z_.find(drone_id);
  if (it_z == drone_z_.end()) {
    return false;
  }

  double alt = -it_z->second;  // NED: up is negative z
  return alt >= (takeoff_alt_ - alt_tol_);
}

bool SwarmGotoCoordinator::all_drones_ready_for_goto() const
{
  for (int id : drone_ids_) {
    if (!is_altitude_ready(id)) {
      return false;
    }
  }
  return true;
}

bool SwarmGotoCoordinator::all_drones_ready_for_traj() const
{
  for (const auto& [drone_id, state] : drone_states_) {
    if (state != HOVER_STATE) {
      return false;
    }
  }
  return true;
}

std::string SwarmGotoCoordinator::px4_namespace(int drone_id) const
{
  if (drone_id == 0) {
    return "/fmu/";
  }
  return "/px4_" + std::to_string(drone_id) + "/fmu/";
}
