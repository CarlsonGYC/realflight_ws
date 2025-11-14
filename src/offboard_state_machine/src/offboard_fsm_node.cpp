// Finite-state machine for PX4 offboard control (ROS 2)
// Author: Yichao Gao 

#include "offboard_state_machine/offboard_fsm_node.hpp"
#include "offboard_state_machine/utils.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace px4_msgs::msg;


/*  Helpers                                                           */

static float wrap_pi(float x)
{
  while (x >  M_PI) x -= 2.f * M_PI;
  while (x < -M_PI) x += 2.f * M_PI;
  return x;
}

static float enu_to_ned_yaw(float yaw_enu)
{
  float y = yaw_enu + static_cast<float>(M_PI_2);
  return wrap_pi(y);
}


/*  MJerkSegment Implementation                                       */

MJerkSegment MJerkSegment::build(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& a0,
    const Eigen::Vector3d& pf,
    const Eigen::Vector3d& vf,
    const Eigen::Vector3d& af,
    double T,
    rclcpp::Time t0)
{
  // Solve quintic polynomial for each axis
  auto solve_axis = [](double p0, double v0, double a0, 
                      double pf, double vf, double af, double T) {
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
  };
  
  MJerkSegment seg;
  seg.ax = solve_axis(p0.x(), v0.x(), a0.x(), pf.x(), vf.x(), af.x(), T);
  seg.ay = solve_axis(p0.y(), v0.y(), a0.y(), pf.y(), vf.y(), af.y(), T);
  seg.az = solve_axis(p0.z(), v0.z(), a0.z(), pf.z(), vf.z(), af.z(), T);
  seg.t0 = t0;
  seg.T = T;
  return seg;
}

void MJerkSegment::sample(const rclcpp::Time& now,
                          Eigen::Vector3d& p,
                          Eigen::Vector3d& v,
                          Eigen::Vector3d& a) const
{
  double t = (now - t0).seconds();
  
  // Clamp negative time due to clock jitter
  if (t < -0.01) {
    t = 0.0;
  }
  
  t = std::clamp(t, 0.0, T);
  
  double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
  
  // Evaluate quintic polynomial
  auto eval = [&](const Eigen::Matrix<double,6,1>& c) {
    double pos = c.coeff(0) + c.coeff(1)*t + c.coeff(2)*t2 + 
                 c.coeff(3)*t3 + c.coeff(4)*t4 + c.coeff(5)*t5;
    double vel = c.coeff(1) + 2.0*c.coeff(2)*t + 3.0*c.coeff(3)*t2 + 
                 4.0*c.coeff(4)*t3 + 5.0*c.coeff(5)*t4;
    double acc = 2.0*c.coeff(2) + 6.0*c.coeff(3)*t + 12.0*c.coeff(4)*t2 + 
                 20.0*c.coeff(5)*t3;
    return std::array<double,3>{pos, vel, acc};
  };
  
  auto rx = eval(ax), ry = eval(ay), rz = eval(az);
  p = {rx[0], ry[0], rz[0]};
  v = {rx[1], ry[1], rz[1]};
  a = {rx[2], ry[2], rz[2]};
}

bool MJerkSegment::finished(const rclcpp::Time& now) const
{
  return (now - t0).seconds() >= T;
}

double MJerkSegment::get_max_velocity() const
{
  double max_vel_sq = 0.0;
  const int num_samples = 100;
  
  for (int i = 0; i <= num_samples; ++i) {
    double t = T * static_cast<double>(i) / static_cast<double>(num_samples);
    double t2 = t*t, t3 = t2*t, t4 = t3*t;
    
    double vx = ax.coeff(1) + 2.0*ax.coeff(2)*t + 3.0*ax.coeff(3)*t2 + 
                4.0*ax.coeff(4)*t3 + 5.0*ax.coeff(5)*t4;
    double vy = ay.coeff(1) + 2.0*ay.coeff(2)*t + 3.0*ay.coeff(3)*t2 + 
                4.0*ay.coeff(4)*t3 + 5.0*ay.coeff(5)*t4;
    double vz = az.coeff(1) + 2.0*az.coeff(2)*t + 3.0*az.coeff(3)*t2 + 
                4.0*az.coeff(4)*t3 + 5.0*az.coeff(5)*t4;
    
    double vel_sq = vx*vx + vy*vy + vz*vz;
    max_vel_sq = std::max(max_vel_sq, vel_sq);
  }
  
  return std::sqrt(max_vel_sq);
}


/*  Constructor                                                       */

OffboardFSM::OffboardFSM(int drone_id)
: Node("offboard_fsm_node_" + std::to_string(drone_id))
, drone_id_(drone_id)
, takeoff_alt_(declare_parameter("takeoff_alt", 1.2))
, takeoff_time_s_ (declare_parameter("takeoff_time",    3.0))
, climb_rate_     (declare_parameter("climb_rate",      1.0))  
, landing_time_s_ (declare_parameter("landing_time",    5.0))
, circle_radius_  (declare_parameter("circle_radius",   1.4))
, inward_offset_  (declare_parameter("inward_offset",   0.80))
, num_drones_     (declare_parameter("num_drones",      6))
, timer_period_s_ (declare_parameter("timer_period",    0.02))
, alt_tol_        (declare_parameter("alt_tol",         0.03))
, radius_         (declare_parameter("circle_radius_traj", 3.0))
, period_s_       (declare_parameter("circle_period",   20.0))
, initial_arming_complete_(false)
, goto_x_       (declare_parameter<double>("goto_x", std::numeric_limits<double>::quiet_NaN()))
, goto_y_       (declare_parameter<double>("goto_y", std::numeric_limits<double>::quiet_NaN()))
, goto_z_       (declare_parameter<double>("goto_z", std::numeric_limits<double>::quiet_NaN()))
, goto_tol_     (declare_parameter("goto_tol", 0.1))
, goto_max_vel_    (declare_parameter("goto_max_vel", 0.8))
, goto_accel_time_ (declare_parameter("goto_accel_time", 2.0))
, in_goto_transition_(false)
, goto_duration_(0.0)
, goto_start_x_(0.0)
, goto_start_y_(0.0)
, goto_start_z_(0.0)
, payload_offset_x_(declare_parameter("payload_offset_x", 0.0))
, payload_offset_y_(declare_parameter("payload_offset_y", 0.0))
, current_state_(FsmState::INIT)
, offb_counter_(0)
, takeoff_start_count_(0)
, takeoff_complete_count_(-1)
, landing_start_count_(0)
, use_attitude_control_(false)         
, odom_ready_(false)
, vel_initialized_(false)
, has_final_setpoint_(false)
, final_setpoint_hold_count_(0)
, hover_x_(0.0)
, hover_y_(0.0)
, hover_z_(-1.2)
, offboard_cmd_count_(0)
, arm_cmd_count_(0)
, last_cmd_time_(0)
, takeoff_pos_x_(0.0)
, takeoff_pos_y_(0.0)
, landing_x_(0.0)
, landing_y_(0.0)
, landing_start_z_(0.0)
, current_x_(0.0)
, current_y_(0.0)
, current_z_(0.0)
, last_z_(0.0)
, nav_state_(0)
, arming_state_(0)
{
  // Calculate formation position
  double theta    = 2.0 * M_PI * drone_id_ / static_cast<double>(num_drones_);
  double r_target = -inward_offset_;
  takeoff_pos_x_  =  r_target * std::sin(theta) + payload_offset_x_;
  takeoff_pos_y_  =  r_target * std::cos(theta) + payload_offset_y_;
  
  climb_rate_ = takeoff_alt_ / takeoff_time_s_;

  RCLCPP_INFO(get_logger(),
              "FSM drone %d: alt=%.2fm, max_vel=%.2fm/s",
              drone_id_, takeoff_alt_, goto_max_vel_);

  // PX4 namespace
  px4_ns_ = (drone_id_ == 0) ? "/fmu/" :
            "/px4_" + std::to_string(drone_id_) + "/fmu/";

  // Publishers
  pub_offb_mode_ = create_publisher<OffboardControlMode>(
      px4_ns_ + "in/offboard_control_mode", rclcpp::QoS{10});
  pub_cmd_       = create_publisher<VehicleCommand>(
      px4_ns_ + "in/vehicle_command",       rclcpp::QoS{10});
  pub_state_     = create_publisher<std_msgs::msg::Int32>(
      "/state/state_drone_" + std::to_string(drone_id_), rclcpp::QoS{10});
  pub_traj_sp_   = create_publisher<TrajectorySetpoint>(
      px4_ns_ + "in/trajectory_setpoint",   rclcpp::QoS{10});

  // Subscribers
  sub_status_ = create_subscription<VehicleStatus>(
      px4_ns_ + "out/vehicle_status_v1", rclcpp::SensorDataQoS(),
      std::bind(&OffboardFSM::status_cb, this, std::placeholders::_1));

  sub_state_cmd_ = create_subscription<std_msgs::msg::Int32>(
      "/state/command_drone_" + std::to_string(drone_id_), rclcpp::QoS{10},
      std::bind(&OffboardFSM::state_cmd_cb, this, std::placeholders::_1));

  sub_odom_ = create_subscription<VehicleOdometry>(
      px4_ns_ + "out/vehicle_odometry", rclcpp::SensorDataQoS(),
      std::bind(&OffboardFSM::odom_cb, this, std::placeholders::_1));

  // Timer using ROS clock for simulation compatibility
  auto timer_period = std::chrono::duration<double>(timer_period_s_);
  timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period)),
      std::bind(&OffboardFSM::timer_cb, this));
  
  RCLCPP_INFO(get_logger(), "Timer: %.0fHz using ROS clock", 1.0/timer_period_s_);
}

uint64_t OffboardFSM::get_timestamp_us() {
  return offboard_utils::get_timestamp_us(get_clock());
}


/*  Callbacks                                                         */

void OffboardFSM::status_cb(const VehicleStatus::SharedPtr msg)
{
  nav_state_    = msg->nav_state;
  arming_state_ = msg->arming_state;
}

void OffboardFSM::odom_cb(const VehicleOdometry::SharedPtr msg)
{
  current_vel_ = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
  vel_initialized_ = true;
  
  current_x_ = msg->position[0];
  current_y_ = msg->position[1];
  current_z_ = msg->position[2];
  odom_ready_ = true;
}

void OffboardFSM::state_cmd_cb(const std_msgs::msg::Int32::SharedPtr msg)
{
  int s = msg->data;
  if (s < static_cast<int>(FsmState::INIT) ||
      s > static_cast<int>(FsmState::DONE))
    return;

  FsmState new_state = static_cast<FsmState>(s);

  if (new_state == FsmState::LAND) {
    landing_start_count_ = offb_counter_;
    landing_start_z_     = -current_z_;
    landing_x_           = current_x_;
    landing_y_           = current_y_;
    
    Eigen::Vector3d p_target(current_x_, current_y_, 0.0);
    start_mjerk_segment(p_target, landing_time_s_);
  }

  if (new_state == FsmState::TRAJ) {
    state_start_time_     = now();
    use_attitude_control_ = true;
  }

  current_state_ = new_state;
  RCLCPP_WARN(get_logger(), "State override to %d", s);
}

bool OffboardFSM::has_goto_target() const
{
  return std::isfinite(goto_x_) &&
         std::isfinite(goto_y_) &&
         std::isfinite(goto_z_);
}


/*  Duration Calculation with Velocity Limiting                      */

double OffboardFSM::calculate_optimal_duration(
    const Eigen::Vector3d& p_start,
    const Eigen::Vector3d& p_target,
    const Eigen::Vector3d& v_start,
    double max_vel) const
{
  double dist = (p_target - p_start).norm();
  
  const double VELOCITY_SCALE_FACTOR = 1.875;
  double duration = VELOCITY_SCALE_FACTOR * dist / max_vel;
  
  double v0_mag = v_start.norm();
  if (v0_mag > 0.1) {
    duration = std::max(duration, (dist + v0_mag * 1.0) / max_vel);
  }
  
  duration = std::max(duration, std::max(goto_accel_time_, 2.0));
  
  const int MAX_ITERATIONS = 15;
  const double VELOCITY_MARGIN = 0.95;
  
  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
    MJerkSegment test_seg = MJerkSegment::build(
        p_start, v_start, Eigen::Vector3d::Zero(),
        p_target, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        duration, rclcpp::Time(0, 0, RCL_ROS_TIME));
    
    double actual_max_vel = test_seg.get_max_velocity();
    
    if (actual_max_vel <= max_vel * 1.05) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Duration %.2fs, v_max=%.3f (iter %d)",
                   duration, actual_max_vel, iter);
      return duration;
    }
    
    double scale = actual_max_vel / (max_vel * VELOCITY_MARGIN);
    duration *= scale;
    
    if (iter % 5 == 0) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Iter %d: v=%.3f, T→%.2fs",
                   iter, actual_max_vel, duration);
    }
  }
  
  RCLCPP_WARN(this->get_logger(), 
              "Duration optimization timeout, T=%.2fs", duration);
  return duration;
}


/*  Start Minimum Jerk Segment                                       */

void OffboardFSM::start_mjerk_segment(const Eigen::Vector3d& p_target,
                                      double initial_duration,
                                      const Eigen::Vector3d& v_target,
                                      const Eigen::Vector3d& a_target)
{
  Eigen::Vector3d p0(current_x_, current_y_, current_z_);
  
  if (std::abs(p0.z()) < 0.05) {
    p0.z() = 0.0;
  }
  
  Eigen::Vector3d v0 = vel_initialized_ ? current_vel_ : Eigen::Vector3d::Zero();
  Eigen::Vector3d a0 = Eigen::Vector3d::Zero();
  
  double duration = initial_duration;
  const int MAX_ITERATIONS = 15;
  
  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
    MJerkSegment candidate = MJerkSegment::build(
        p0, v0, a0, p_target, v_target, a_target, duration, now());
    
    double actual_max_vel = candidate.get_max_velocity();
    
    if (actual_max_vel <= goto_max_vel_ * 1.05) {
      active_seg_ = candidate;
      
      RCLCPP_INFO(get_logger(), 
                  "Seg [%.2f,%.2f,%.2f]→[%.2f,%.2f,%.2f] T=%.2fs v=%.3f",
                  p0.x(), p0.y(), p0.z(), 
                  p_target.x(), p_target.y(), p_target.z(), 
                  duration, actual_max_vel);
      return;
    }
    
    double scale = actual_max_vel / (goto_max_vel_ * 0.92);
    duration *= scale;
    
    RCLCPP_DEBUG(get_logger(), 
                 "Iter %d: v=%.3f>%.3f, T→%.2fs",
                 iter, actual_max_vel, goto_max_vel_, duration);
  }
  
  active_seg_ = MJerkSegment::build(
      p0, v0, a0, p_target, v_target, a_target, duration, now());
  
  double final_vel = active_seg_->get_max_velocity();
  RCLCPP_WARN(get_logger(), 
              "Duration failed, T=%.2fs v=%.3f>%.3f",
              duration, final_vel, goto_max_vel_);
}


/*  Publish Trajectory Setpoint                                      */

void OffboardFSM::publish_current_setpoint()
{
  if (current_state_ == FsmState::TRAJ) {
    return;
  }
  
  TrajectorySetpoint sp;
  
  for (int i = 0; i < 3; ++i) {
    sp.position[i] = std::nanf("");
    sp.velocity[i] = std::nanf("");
    sp.acceleration[i] = std::nanf("");
  }
  sp.yaw = 0.0f;
  sp.yawspeed = 0.0f;

  if (active_seg_.has_value()) {
    Eigen::Vector3d p, v, a;
    active_seg_->sample(now(), p, v, a);
    
    const double GROUND_TOLERANCE = 0.05;
    if (p.z() > GROUND_TOLERANCE) {
      RCLCPP_ERROR(get_logger(), "z=%.3f corrected", p.z());
      p.z() = 0.0;
      v.z() = 0.0;
      a.z() = 0.0;
    }
    
    sp.position[0] = static_cast<float>(p.x());
    sp.position[1] = static_cast<float>(p.y());
    sp.position[2] = static_cast<float>(p.z());
    sp.velocity[0] = static_cast<float>(v.x());
    sp.velocity[1] = static_cast<float>(v.y());
    sp.velocity[2] = static_cast<float>(v.z());
    sp.acceleration[0] = static_cast<float>(a.x());
    sp.acceleration[1] = static_cast<float>(a.y());
    sp.acceleration[2] = static_cast<float>(a.z());
    
    final_setpoint_hold_count_ = 0;
    
    if (active_seg_->finished(now())) {
      final_position_ = p;
      final_velocity_ = v;
      final_acceleration_ = a;
      has_final_setpoint_ = true;
      
      RCLCPP_INFO(get_logger(), "Seg done v=[%.3f,%.3f,%.3f]",
                  v.x(), v.y(), v.z());
      active_seg_.reset();
    }
  } 
  else if (has_final_setpoint_ && 
           (current_state_ == FsmState::TAKEOFF || 
            current_state_ == FsmState::GOTO)) {
    
    sp.position[0] = static_cast<float>(final_position_.x());
    sp.position[1] = static_cast<float>(final_position_.y());
    sp.position[2] = static_cast<float>(final_position_.z());
    
    // Exponential velocity decay for smooth transition
    const double DECAY_TC = 0.05;
    double decay = std::exp(-timer_period_s_ / DECAY_TC);
    final_velocity_ *= decay;
    
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
      RCLCPP_INFO(get_logger(), "Steady state");
    }
  }
  else {
    has_final_setpoint_ = false;
    final_setpoint_hold_count_ = 0;
    
    switch (current_state_) {
      case FsmState::INIT:
      case FsmState::ARMING:
        sp.position[0] = static_cast<float>(takeoff_pos_x_);
        sp.position[1] = static_cast<float>(takeoff_pos_y_);
        sp.position[2] = -0.05f;
        sp.velocity[0] = 0.0f;
        sp.velocity[1] = 0.0f;
        sp.velocity[2] = 0.0f;
        break;
        
      case FsmState::HOVER:
      case FsmState::END_TRAJ:
        sp.position[0] = static_cast<float>(hover_x_);
        sp.position[1] = static_cast<float>(hover_y_);
        sp.position[2] = static_cast<float>(hover_z_);
        sp.velocity[0] = 0.0f;
        sp.velocity[1] = 0.0f;
        sp.velocity[2] = 0.0f;
        break;
        
      case FsmState::TAKEOFF:
        sp.position[0] = static_cast<float>(takeoff_pos_x_);
        sp.position[1] = static_cast<float>(takeoff_pos_y_);
        sp.position[2] = static_cast<float>(-takeoff_alt_);  
        sp.velocity[0] = 0.0f;
        sp.velocity[1] = 0.0f;
        sp.velocity[2] = 0.0f;
        break;
        
      case FsmState::GOTO:
        if (has_goto_target()) {
          sp.position[0] = static_cast<float>(goto_x_);
          sp.position[1] = static_cast<float>(goto_y_);
          sp.position[2] = static_cast<float>(goto_z_);
        } else if (odom_ready_) {
          sp.position[0] = static_cast<float>(current_x_);
          sp.position[1] = static_cast<float>(current_y_);
          sp.position[2] = static_cast<float>(current_z_);
        } else {
          sp.position[0] = 0.0f;
          sp.position[1] = 0.0f;
          sp.position[2] = -0.05f;
        }
        sp.velocity[0] = 0.0f;
        sp.velocity[1] = 0.0f;
        sp.velocity[2] = 0.0f;
        break;
        
      case FsmState::LAND:
        sp.position[0] = static_cast<float>(landing_x_);
        sp.position[1] = static_cast<float>(landing_y_);
        sp.position[2] = 0.0f;
        sp.velocity[0] = 0.0f;
        sp.velocity[1] = 0.0f;
        sp.velocity[2] = 0.0f;
        break;
        
      case FsmState::DONE:
        sp.position[0] = static_cast<float>(landing_x_);
        sp.position[1] = static_cast<float>(landing_y_);
        sp.position[2] = 0.0f;
        sp.velocity[0] = 0.0f;
        sp.velocity[1] = 0.0f;
        sp.velocity[2] = 0.0f;
        break;
        
      case FsmState::TRAJ:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "TRAJ in fallback");
        break;
    }
  }
  
  // Ground validation
  const float GROUND_TOL = 0.05f;
  if (std::isfinite(sp.position[2]) && sp.position[2] > GROUND_TOL) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "z=%.3f fixed", sp.position[2]);
    sp.position[2] = 0.0f;
    sp.velocity[2] = 0.0f;
    if (std::isfinite(sp.acceleration[2])) {
      sp.acceleration[2] = 0.0f;
    }
  }
  
  // NaN fallback
  if (!std::isfinite(sp.position[0]) || 
      !std::isfinite(sp.position[1]) || 
      !std::isfinite(sp.position[2])) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid SP");
    if (odom_ready_) {
      sp.position[0] = static_cast<float>(current_x_);
      sp.position[1] = static_cast<float>(current_y_);
      sp.position[2] = static_cast<float>(std::min(current_z_, 0.0));
    } else {
      sp.position[0] = 0.0f;
      sp.position[1] = 0.0f;
      sp.position[2] = -0.05f;
    }
    sp.velocity[0] = 0.0f;
    sp.velocity[1] = 0.0f;
    sp.velocity[2] = 0.0f;
  }
  
  sp.timestamp = 0;
  pub_traj_sp_->publish(sp);
}


/*  Main Timer Callback                                               */

void OffboardFSM::timer_cb()
{
  publish_offboard_mode();
  publish_current_setpoint();
  
  switch (current_state_) {
  case FsmState::INIT:
    if (offb_counter_ >= 20) {
      RCLCPP_INFO_ONCE(get_logger(), "Arming sequence start");
      current_state_ = FsmState::ARMING;
      offb_counter_ = 0;
    }
    break;

  case FsmState::ARMING: {
    bool is_offboard = (nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    bool is_armed = (arming_state_ == VehicleStatus::ARMING_STATE_ARMED);
    
    if (is_offboard && is_armed) {
      initial_arming_complete_ = true;
      RCLCPP_INFO(get_logger(), "Drone %d armed in offboard", drone_id_);
      current_state_       = FsmState::TAKEOFF;
      takeoff_start_count_ = offb_counter_;
      last_z_              = current_z_;
      
      Eigen::Vector3d p_start(current_x_, current_y_, current_z_);
      Eigen::Vector3d p_target(takeoff_pos_x_, takeoff_pos_y_, -takeoff_alt_);
      double optimal_duration = calculate_optimal_duration(
          p_start, p_target, current_vel_, goto_max_vel_);
      
      double actual_duration = std::max(optimal_duration, takeoff_time_s_);
      start_mjerk_segment(p_target, actual_duration);
      
      RCLCPP_INFO(get_logger(), "Takeoff T=%.2fs", actual_duration);
      
      offb_counter_        = 0;
      offboard_cmd_count_  = 0;
      arm_cmd_count_       = 0;
    } else {
      if (!is_offboard && !initial_arming_complete_ && offboard_cmd_count_ % 50 == 0) {
        send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Req offboard");
      }
      offboard_cmd_count_++;
      
      if (is_offboard && !initial_arming_complete_ && !is_armed && arm_cmd_count_ % 50 == 0) {
        send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Req arm");
      }
      if (is_offboard) arm_cmd_count_++;
    }
    break;
  }

  case FsmState::TAKEOFF: {
    if (!active_seg_.has_value()) {
      double actual_alt = -current_z_;
      
      if (takeoff_complete_count_ < 0) {
        bool alt_ok = actual_alt >= (takeoff_alt_ - alt_tol_);
        bool vel_ok = std::abs(current_z_ - last_z_) / timer_period_s_ < 0.1;
        
        if (alt_ok && vel_ok) {
          takeoff_complete_count_ = offb_counter_;
          RCLCPP_INFO(get_logger(), "Alt %.2fm reached", actual_alt);
        }
      }
      
      if (takeoff_complete_count_ >= 0) {
        double hover_time = (offb_counter_ - takeoff_complete_count_) * timer_period_s_;
        if (hover_time >= 2.0) {
          if (has_goto_target()) {
            Eigen::Vector3d p_start(current_x_, current_y_, current_z_);
            Eigen::Vector3d p_target(goto_x_, goto_y_, goto_z_);
            
            double duration = calculate_optimal_duration(
                p_start, p_target, current_vel_, goto_max_vel_);
            
            start_mjerk_segment(p_target, duration);
            
            double dist = (p_target - p_start).norm();
            current_state_ = FsmState::GOTO;
            RCLCPP_INFO(get_logger(), "GOTO %.2fm T=%.1fs", dist, duration);
          } else {
            hover_x_ = takeoff_pos_x_;
            hover_y_ = takeoff_pos_y_;
            hover_z_ = -takeoff_alt_;
            current_state_ = FsmState::HOVER;
            RCLCPP_INFO(get_logger(), "HOVER");
          }
          state_start_time_ = now();
          offb_counter_ = 0;
        }
      }
    }
    
    last_z_ = current_z_;
    break;
  }

  case FsmState::GOTO: {
    if (!has_goto_target()) {
      hover_x_ = current_x_;
      hover_y_ = current_y_;
      hover_z_ = current_z_;
      current_state_ = FsmState::HOVER;
      RCLCPP_WARN(get_logger(), "No target, HOVER");
      break;
    }

    if (!active_seg_.has_value()) {
      double dx = current_x_ - goto_x_;
      double dy = current_y_ - goto_y_;
      double dz = current_z_ - goto_z_;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      
      if (dist < goto_tol_) {
        RCLCPP_INFO(get_logger(), "GOTO done err=%.3fm", dist);
        hover_x_ = goto_x_;
        hover_y_ = goto_y_;
        hover_z_ = goto_z_;
        current_state_ = FsmState::HOVER;
        state_start_time_ = now();
        offb_counter_ = 0;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Seg ended err=%.3fm", dist);
      }
    }
    break;
  }

  case FsmState::HOVER:
    if (use_attitude_control_) {
      use_attitude_control_ = false;
    }
    break;

  case FsmState::TRAJ:
    if (!use_attitude_control_) {
      use_attitude_control_ = true;
      state_start_time_ = now();
    }
    break;

  case FsmState::END_TRAJ:
    if (use_attitude_control_) {
      use_attitude_control_ = false;
      state_start_time_ = now();
      RCLCPP_INFO(get_logger(), "Traj complete");
    }
    break;

  case FsmState::LAND: {
    if (!active_seg_.has_value()) {
      double alt = -current_z_;
      if (alt <= 0.15) {
        RCLCPP_INFO(get_logger(), "Landed");
        current_state_ = FsmState::DONE;
        send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f, 0.f);
      }
    }
    break;
  }

  case FsmState::DONE:
    break;
  }

  std_msgs::msg::Int32 st; 
  st.data = static_cast<int>(current_state_);
  pub_state_->publish(st);
  
  ++offb_counter_;
}

void OffboardFSM::publish_offboard_mode()
{
  OffboardControlMode m;
  
  bool has_active_seg = active_seg_.has_value();
  
  if (current_state_ == FsmState::TRAJ) {
    m.position     = true;
    m.velocity     = false;
    m.acceleration = false;
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
  
  m.timestamp = 0;
  pub_offb_mode_->publish(m);
}

void OffboardFSM::send_vehicle_cmd(uint16_t cmd, float p1, float p2)
{
  VehicleCommand m;
  m.command       = cmd;
  m.param1        = p1;
  m.param2        = p2;
  m.target_system = drone_id_ + 1;
  m.target_component = 1;
  m.source_system = 1;
  m.source_component = 1;
  m.from_external = true;
  m.timestamp     = 0;
  pub_cmd_->publish(m);
}

void OffboardFSM::try_set_offboard_and_arm()
{
  send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);
  send_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
}

void OffboardFSM::generate_trajectory()
{
}

void OffboardFSM::calculate_goto_ramp(double& pos_x, double& pos_y, double& pos_z,
                                      double& vel_x, double& vel_y, double& vel_z)
{
}