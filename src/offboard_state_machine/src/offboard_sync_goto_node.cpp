// Coordinated takeoff + goto node without modifying the original FSM
#include "offboard_state_machine/offboard_sync_goto_node.hpp"
#include "offboard_state_machine/utils.hpp"

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

using namespace px4_msgs::msg;

/* Minimum-jerk segment helpers */

static Eigen::Matrix<double,6,1> solve_axis(double p0, double v0, double a0,
                                            double pf, double vf, double af, double T)
{
  Eigen::Matrix<double,6,6> M;
  M << 1, 0,   0,    0,     0,      0,
       0, 1,   0,    0,     0,      0,
       0, 0,   2,    0,     0,      0,
       1, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T,
       0, 1, 2*T, 3*T*T, 4*T*T*T, 5*T*T*T*T,
       0, 0,   2,   6*T,  12*T*T,  20*T*T*T;

  Eigen::Matrix<double,6,1> b;
  b << p0, v0, a0, pf, vf, af;
  Eigen::Matrix<double,6,1> results = M.fullPivLu().solve(b);
  return results;
}

SyncMJerkSegment SyncMJerkSegment::build(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& a0,
    const Eigen::Vector3d& pf,
    const Eigen::Vector3d& vf,
    const Eigen::Vector3d& af,
    double T,
    rclcpp::Time t0)
{
  SyncMJerkSegment seg;
  seg.ax = solve_axis(p0.x(), v0.x(), a0.x(), pf.x(), vf.x(), af.x(), T);
  seg.ay = solve_axis(p0.y(), v0.y(), a0.y(), pf.y(), vf.y(), af.y(), T);
  seg.az = solve_axis(p0.z(), v0.z(), a0.z(), pf.z(), vf.z(), af.z(), T);
  seg.t0 = t0;
  seg.T = T;
  return seg;
}

void SyncMJerkSegment::sample(const rclcpp::Time& now,
                              Eigen::Vector3d& p,
                              Eigen::Vector3d& v,
                              Eigen::Vector3d& a) const
{
  double t = (now - t0).seconds();
  t = std::clamp(t, 0.0, T);

  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;

  auto eval = [&](const Eigen::Matrix<double,6,1>& c) {
    double pos = c.coeff(0) + c.coeff(1) * t + c.coeff(2) * t2 +
                 c.coeff(3) * t3 + c.coeff(4) * t4 + c.coeff(5) * t5;
    double vel = c.coeff(1) + 2.0 * c.coeff(2) * t + 3.0 * c.coeff(3) * t2 +
                 4.0 * c.coeff(4) * t3 + 5.0 * c.coeff(5) * t4;
    double acc = 2.0 * c.coeff(2) + 6.0 * c.coeff(3) * t + 12.0 * c.coeff(4) * t2 +
                 20.0 * c.coeff(5) * t3;
    return std::array<double,3>{pos, vel, acc};
  };

  auto rx = eval(ax);
  auto ry = eval(ay);
  auto rz = eval(az);
  p = {rx[0], ry[0], rz[0]};
  v = {rx[1], ry[1], rz[1]};
  a = {rx[2], ry[2], rz[2]};
}

bool SyncMJerkSegment::finished(const rclcpp::Time& now) const
{
  return (now - t0).seconds() >= T;
}


/* OffboardSyncGoto implementation */

OffboardSyncGoto::OffboardSyncGoto(int drone_id)
: Node("offboard_sync_goto_" + std::to_string(drone_id))
, drone_id_(drone_id)
, takeoff_alt_(declare_parameter("takeoff_alt", 0.4))
, takeoff_time_s_(declare_parameter("takeoff_time", 5.0))
, goto_time_s_(declare_parameter("goto_time", 10.0))
, timer_period_s_(declare_parameter("timer_period", 0.02))
, landing_time_s_(declare_parameter("landing_time", 5.0))
, landing_max_vel_(declare_parameter("landing_max_vel", 0.3))
, end_traj_wait_time_(declare_parameter("end_traj_wait_time", 5.0))
, traj_base_dir_(declare_parameter("traj_base_dir", std::string("data/3drone_trajectories_new")))
, use_raw_traj_(declare_parameter("use_raw_traj", false))
, current_state_(SyncFsmState::INIT)
, offb_counter_(0)
, odom_ready_(false)
, takeoff_target_ready_(false)
, goto_target_ready_(false)
, current_x_(0.0)
, current_y_(0.0)
, current_z_(0.0)
, takeoff_x_(0.0)
, takeoff_y_(0.0)
, takeoff_z_(0.0)
, hover_x_(0.0)
, hover_y_(0.0)
, hover_z_(0.0)
, goto_x_(0.0)
, goto_y_(0.0)
, goto_z_(0.0)
, nav_state_(0)
, arming_state_(0)
{
  RCLCPP_INFO(get_logger(), "=== Offboard Sync GOTO for drone %d ===", drone_id_);
  RCLCPP_INFO(get_logger(), "Takeoff: %.2fm over %.1fs, GOTO duration: %.1fs",
              takeoff_alt_, takeoff_time_s_, goto_time_s_);
  RCLCPP_INFO(get_logger(), "Trajectory dir: %s (raw=%s)",
              traj_base_dir_.c_str(), use_raw_traj_ ? "true" : "false");

  load_first_waypoint();

  auto px4_ns = px4_namespace();
  pub_offb_mode_ = create_publisher<OffboardControlMode>(px4_ns + "in/offboard_control_mode", 10);
  pub_traj_sp_   = create_publisher<TrajectorySetpoint>(px4_ns + "in/trajectory_setpoint", 10);
  pub_cmd_       = create_publisher<VehicleCommand>(px4_ns + "in/vehicle_command", 10);
  pub_state_     = create_publisher<std_msgs::msg::Int32>("/state/state_drone_" + std::to_string(drone_id_), 10);

  sub_status_ = create_subscription<VehicleStatus>(
      px4_ns + "out/vehicle_status_v1",
      rclcpp::SensorDataQoS(),
      std::bind(&OffboardSyncGoto::status_cb, this, std::placeholders::_1));

  sub_odom_ = create_subscription<VehicleOdometry>(
      px4_ns + "out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      std::bind(&OffboardSyncGoto::odom_cb, this, std::placeholders::_1));

  sub_state_cmd_ = create_subscription<std_msgs::msg::Int32>(
      "/state/command_drone_" + std::to_string(drone_id_),
      rclcpp::QoS(10),
      std::bind(&OffboardSyncGoto::state_cmd_cb, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timer_period_s_))),
      std::bind(&OffboardSyncGoto::timer_cb, this));

  RCLCPP_INFO(get_logger(), "Control timer at %.1f Hz", 1.0 / timer_period_s_);
}

std::string OffboardSyncGoto::px4_namespace() const
{
  if (drone_id_ == 0) {
    return "/fmu/";
  }
  return "/px4_" + std::to_string(drone_id_) + "/fmu/";
}

bool OffboardSyncGoto::load_first_waypoint()
{
  namespace fs = std::filesystem;
  std::string file = "drone_" + std::to_string(drone_id_) +
                     (use_raw_traj_ ? "_traj_raw_20hz.csv" : "_traj_smoothed_100hz.csv");
  fs::path csv_path = fs::path(traj_base_dir_) / file;

  std::ifstream fin(csv_path);
  if (!fin.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open CSV: %s", csv_path.c_str());
    return false;
  }

  std::string header;
  std::getline(fin, header);
  std::string line;
  if (!std::getline(fin, line)) {
    RCLCPP_ERROR(get_logger(), "CSV %s has no data rows", csv_path.c_str());
    return false;
  }

  std::stringstream ss(line);
  std::string token;
  std::vector<double> values;
  while (std::getline(ss, token, ',')) {
    try {
      values.push_back(std::stod(token));
    } catch (const std::exception&) {
      values.push_back(0.0);
    }
  }

  if (values.size() < 4) {
    RCLCPP_ERROR(get_logger(), "CSV %s first row malformed", csv_path.c_str());
    return false;
  }

  goto_x_ = values[1];
  goto_y_ = values[2];
  goto_z_ = values[3];
  goto_target_ready_ = true;

  RCLCPP_INFO(get_logger(), "Goto target from CSV: [%.2f, %.2f, %.2f] (%s)",
              goto_x_, goto_y_, goto_z_, csv_path.c_str());
  return true;
}

void OffboardSyncGoto::compute_takeoff_target()
{
  if (!odom_ready_) {
    return;
  }
  takeoff_x_ = current_x_;
  takeoff_y_ = current_y_;
  takeoff_z_ = current_z_ - takeoff_alt_;  // NED: up is negative z

  hover_x_ = takeoff_x_;
  hover_y_ = takeoff_y_;
  hover_z_ = takeoff_z_;
  takeoff_target_ready_ = true;

  RCLCPP_INFO(get_logger(), "Takeoff target set to current XY with z=%.2f (delta %.2f)",
              takeoff_z_, -takeoff_alt_);
}

void OffboardSyncGoto::status_cb(const VehicleStatus::SharedPtr msg)
{
  nav_state_    = msg->nav_state;
  arming_state_ = msg->arming_state;
}

void OffboardSyncGoto::odom_cb(const VehicleOdometry::SharedPtr msg)
{
  current_vel_ = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
  current_x_ = msg->position[0];
  current_y_ = msg->position[1];
  current_z_ = msg->position[2];
  odom_ready_ = true;

  if (!takeoff_target_ready_) {
    compute_takeoff_target();
  }
}

void OffboardSyncGoto::state_cmd_cb(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto cmd_state = static_cast<SyncFsmState>(msg->data);
  if (cmd_state == SyncFsmState::GOTO) {
    if (!goto_target_ready_) {
      RCLCPP_ERROR(get_logger(), "GOTO command received but goto target is not ready");
      return;
    }
    if (!odom_ready_) {
      RCLCPP_WARN(get_logger(), "GOTO command received but odom not ready yet");
      return;
    }

    Eigen::Vector3d p_target(goto_x_, goto_y_, goto_z_);
    start_mjerk_segment(p_target, goto_time_s_);
    current_state_ = SyncFsmState::GOTO;
    goto_done_ = false;
    offb_counter_ = 0;
    RCLCPP_WARN(get_logger(), "Starting synchronized GOTO -> [%.2f, %.2f, %.2f] (T=%.1fs)",
                goto_x_, goto_y_, goto_z_, goto_time_s_);
  } else if (cmd_state == SyncFsmState::TRAJ) {
    current_state_ = SyncFsmState::TRAJ;
    RCLCPP_WARN(get_logger(), "Switching to TRAJ mode (external setpoints)");
  } else if (cmd_state == SyncFsmState::END_TRAJ) {
    current_state_ = SyncFsmState::END_TRAJ;
    hover_x_ = current_x_;
    hover_y_ = current_y_;
    hover_z_ = current_z_;
    end_traj_start_time_ = now();
    end_traj_timer_started_ = true;
    RCLCPP_INFO(get_logger(), "END_TRAJ: hold and prepare for landing after wait");
  } else if (cmd_state == SyncFsmState::LAND) {
    Eigen::Vector3d p_target(current_x_, current_y_, 0.0);
    start_mjerk_segment(p_target, 3.0);
    current_state_ = SyncFsmState::LAND;
    RCLCPP_WARN(get_logger(), "Received LAND command, descending");
  }
}

void OffboardSyncGoto::send_vehicle_cmd(uint16_t cmd, float p1, float p2)
{
  VehicleCommand msg{};
  msg.timestamp = offboard_utils::get_timestamp_us(get_clock());
  msg.param1 = p1;
  msg.param2 = p2;
  msg.command = cmd;
  msg.target_system = static_cast<uint8_t>(drone_id_ + 1);  // match tested FSM behavior
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  pub_cmd_->publish(msg);
}

void OffboardSyncGoto::start_mjerk_segment(const Eigen::Vector3d& p_target,
                                           double duration,
                                           const Eigen::Vector3d& v_target,
                                           const Eigen::Vector3d& a_target)
{
  if (!odom_ready_) {
    RCLCPP_ERROR(get_logger(), "Cannot start segment without odometry");
    return;
  }

  Eigen::Vector3d p0(current_x_, current_y_, current_z_);
  active_seg_ = SyncMJerkSegment::build(
      p0, current_vel_, Eigen::Vector3d::Zero(), p_target, v_target, a_target, duration, now());
  has_final_setpoint_ = false;
  final_setpoint_hold_count_ = 0;
}

void OffboardSyncGoto::publish_offboard_mode()
{
  OffboardControlMode m;
  bool has_active_seg = active_seg_.has_value();

  if (current_state_ == SyncFsmState::TRAJ) {
    m.position     = true;
    m.velocity     = true;
    m.acceleration = true;
    m.attitude     = false;
    m.body_rate    = false;
  } else if (has_active_seg) {
    m.position     = true;
    m.velocity     = false;
    m.acceleration = false;
    m.attitude     = false;
    m.body_rate    = false;
  } else {
    m.position     = true;
    m.velocity     = false;
    m.acceleration = false;
    m.attitude     = false;
    m.body_rate    = false;
  }
  m.timestamp = offboard_utils::get_timestamp_us(get_clock());
  pub_offb_mode_->publish(m);
}

void OffboardSyncGoto::publish_current_setpoint()
{
  TrajectorySetpoint sp{};
  sp.timestamp = offboard_utils::get_timestamp_us(get_clock());
  sp.yaw = 3.1415926f;
  sp.yawspeed = 0.0f;

  if (current_state_ == SyncFsmState::TRAJ) {
    // Allow external trajectory publisher to own the setpoints
    return;
  }

  if (active_seg_.has_value()) {
    Eigen::Vector3d p, v, a;
    active_seg_->sample(now(), p, v, a);
    sp.position[0] = static_cast<float>(p.x());
    sp.position[1] = static_cast<float>(p.y());
    sp.position[2] = static_cast<float>(p.z());
    sp.velocity[0] = static_cast<float>(v.x());
    sp.velocity[1] = static_cast<float>(v.y());
    sp.velocity[2] = static_cast<float>(v.z());
    sp.acceleration[0] = static_cast<float>(a.x());
    sp.acceleration[1] = static_cast<float>(a.y());
    sp.acceleration[2] = static_cast<float>(a.z());

    if (active_seg_->finished(now())) {
      final_position_ = p;
      final_velocity_ = v;
      has_final_setpoint_ = true;
      final_setpoint_hold_count_ = 0;
      active_seg_.reset();
    }
  } else if (has_final_setpoint_) {
    const double DECAY_TC = 0.05;
    double decay = std::exp(-timer_period_s_ / DECAY_TC);
    final_velocity_ *= decay;

    sp.position[0] = static_cast<float>(final_position_.x());
    sp.position[1] = static_cast<float>(final_position_.y());
    sp.position[2] = static_cast<float>(final_position_.z());
    sp.velocity[0] = static_cast<float>(final_velocity_.x());
    sp.velocity[1] = static_cast<float>(final_velocity_.y());
    sp.velocity[2] = static_cast<float>(final_velocity_.z());
    sp.acceleration[0] = 0.0f;
    sp.acceleration[1] = 0.0f;
    sp.acceleration[2] = 0.0f;

    final_setpoint_hold_count_++;
    if (final_setpoint_hold_count_ > 10 || final_velocity_.norm() < 0.01) {
      has_final_setpoint_ = false;
      final_setpoint_hold_count_ = 0;
    }
  } else {
    auto clamp_z = [](double z) {
      const double ground_tol = 0.0;
      return std::min(z, ground_tol);
    };

    switch (current_state_) {
      case SyncFsmState::INIT:
      case SyncFsmState::ARMING:
        sp.position[0] = static_cast<float>(odom_ready_ ? current_x_ : 0.0);
        sp.position[1] = static_cast<float>(odom_ready_ ? current_y_ : 0.0);
        sp.position[2] = static_cast<float>(clamp_z(odom_ready_ ? current_z_ : -0.05));
        break;

      case SyncFsmState::TAKEOFF:
      case SyncFsmState::HOVER:
        sp.position[0] = static_cast<float>(hover_x_);
        sp.position[1] = static_cast<float>(hover_y_);
        sp.position[2] = static_cast<float>(hover_z_);
        break;

      case SyncFsmState::GOTO:
        sp.position[0] = static_cast<float>(goto_target_ready_ ? goto_x_ : current_x_);
        sp.position[1] = static_cast<float>(goto_target_ready_ ? goto_y_ : current_y_);
        sp.position[2] = static_cast<float>(goto_target_ready_ ? goto_z_ : current_z_);
        break;

      case SyncFsmState::LAND:
      case SyncFsmState::DONE:
        sp.position[0] = static_cast<float>(current_x_);
        sp.position[1] = static_cast<float>(current_y_);
        sp.position[2] = 0.0f;
        break;

      case SyncFsmState::TRAJ:
      case SyncFsmState::END_TRAJ:
        sp.position[0] = static_cast<float>(current_x_);
        sp.position[1] = static_cast<float>(current_y_);
        sp.position[2] = static_cast<float>(current_z_);
        break;
    }

    sp.velocity[0] = 0.0f;
    sp.velocity[1] = 0.0f;
    sp.velocity[2] = 0.0f;
    sp.acceleration[0] = 0.0f;
    sp.acceleration[1] = 0.0f;
    sp.acceleration[2] = 0.0f;
  }

  if (std::isfinite(sp.position[2]) && sp.position[2] > 0.0f) {
    sp.position[2] = 0.0f;
  }

  pub_traj_sp_->publish(sp);
}

void OffboardSyncGoto::timer_cb()
{
  publish_offboard_mode();
  publish_current_setpoint();

  switch (current_state_) {
    case SyncFsmState::INIT:
      if (offb_counter_ >= 20) {
        current_state_ = SyncFsmState::ARMING;
        offb_counter_ = 0;
      }
      break;

    case SyncFsmState::ARMING: {
      bool is_offboard = (nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD);
      bool is_armed = (arming_state_ == VehicleStatus::ARMING_STATE_ARMED);

      if (is_offboard && is_armed && takeoff_target_ready_) {
        start_mjerk_segment(Eigen::Vector3d(takeoff_x_, takeoff_y_, takeoff_z_), takeoff_time_s_);
        current_state_ = SyncFsmState::TAKEOFF;
        takeoff_done_ = false;
        offb_counter_ = 0;
        RCLCPP_INFO(get_logger(), "Takeoff segment started to z=%.2f (T=%.1fs)",
                    takeoff_z_, takeoff_time_s_);
      } else {
        if (!is_offboard && offb_counter_ % 50 == 0) {
          send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Requesting OFFBOARD");
        }
        if (is_offboard && !is_armed && offb_counter_ % 50 == 0) {
          send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Requesting ARM");
        }
      }
      break;
    }

    case SyncFsmState::TAKEOFF:
      if (!active_seg_.has_value()) {
        // Stay in TAKEOFF state but hold the achieved takeoff position
        hover_x_ = takeoff_x_;
        hover_y_ = takeoff_y_;
        hover_z_ = takeoff_z_;
        takeoff_done_ = true;
        // Do not change state here; coordinator will command GOTO explicitly
      }
      break;

    case SyncFsmState::GOTO:
      if (!active_seg_.has_value() && !goto_done_) {
        hover_x_ = goto_x_;
        hover_y_ = goto_y_;
        hover_z_ = goto_z_;
        goto_done_ = true;
        current_state_ = SyncFsmState::HOVER;  // single transition after GOTO completion
        RCLCPP_INFO(get_logger(), "GOTO finished, switching to HOVER");
      }
      break;

    case SyncFsmState::HOVER:
    case SyncFsmState::TRAJ:
      break;

    case SyncFsmState::END_TRAJ: {
      if (end_traj_timer_started_) {
        double elapsed = (now() - end_traj_start_time_).seconds();
        if (elapsed >= end_traj_wait_time_ && !active_seg_.has_value()) {
          Eigen::Vector3d p_target(current_x_, current_y_, 0.0);
          start_mjerk_segment(p_target, landing_time_s_);
          current_state_ = SyncFsmState::LAND;
          RCLCPP_INFO(get_logger(),
                      "END_TRAJ wait done (%.1fs), auto-landing over %.1fs",
                      elapsed, landing_time_s_);
        }
      }
      break;
    }

    case SyncFsmState::LAND:
      if (!active_seg_.has_value() && std::abs(current_z_) < 0.1) {
        send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f, 0.f);
        current_state_ = SyncFsmState::DONE;
        RCLCPP_INFO(get_logger(), "Landing complete, disarmed");
      }
      break;

    case SyncFsmState::DONE:
      break;
  }

  std_msgs::msg::Int32 st;
  st.data = static_cast<int>(current_state_);
  pub_state_->publish(st);
  ++offb_counter_;
}
