#include "geom_multilift/geom_multilift_hitl_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>

using std::placeholders::_1;
using namespace std::chrono_literals;

AccKalmanFilter::AccKalmanFilter(double dt_init, double q_var, double r_var)
: init_(false), q_(q_var), dt_(dt_init) {
  F_.setIdentity();
  Q_.setZero();
  H_.setZero();
  H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
  R_ = r_var * Eigen::Matrix3d::Identity();
  x_.setZero();
  P_.setIdentity();
  P_ *= 1e3;
  pos_.setZero();
  vel_.setZero();
  acc_.setZero();
}

void AccKalmanFilter::build(double dt) {
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  double dt2 = dt * dt / 2.0;
  F_.setIdentity();
  F_.block<3,3>(0,3) = dt * I3;
  F_.block<3,3>(0,6) = dt2 * I3;
  F_.block<3,3>(3,6) = dt * I3;

  Q_.setZero();
  Q_.block<3,3>(0,0) = (std::pow(dt,5)/20.0) * q_ * I3;
  Q_.block<3,3>(0,3) = (std::pow(dt,4)/8.0)  * q_ * I3;
  Q_.block<3,3>(0,6) = (std::pow(dt,3)/6.0)  * q_ * I3;
  Q_.block<3,3>(3,0) = Q_.block<3,3>(0,3).transpose();
  Q_.block<3,3>(3,3) = (std::pow(dt,3)/3.0)  * q_ * I3;
  Q_.block<3,3>(3,6) = (std::pow(dt,2)/2.0)  * q_ * I3;
  Q_.block<3,3>(6,0) = Q_.block<3,3>(0,6).transpose();
  Q_.block<3,3>(6,3) = Q_.block<3,3>(3,6).transpose();
  Q_.block<3,3>(6,6) = dt * q_ * I3;
}

void AccKalmanFilter::step(const Eigen::Vector3d &z, double dt) {
  if (!init_) {
    x_.setZero();
    x_.segment<3>(0) = z;
    pos_ = z;
    init_ = true;
    return;
  }
  dt_ = dt;
  build(dt_);
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

  Eigen::Matrix<double,3,1> y = z - H_ * x_;
  Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix<double,9,3> K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  Eigen::Matrix<double,9,9> I9 = Eigen::Matrix<double,9,9>::Identity();
  P_ = (I9 - K * H_) * P_;

  pos_ = x_.segment<3>(0);
  vel_ = x_.segment<3>(3);
  acc_ = x_.segment<3>(6);
}

GeomMultiliftHitlNode::GeomMultiliftHitlNode(int drone_id, int total_drones)
: Node("geom_multilift_hitl_" + std::to_string(drone_id))
, drone_id_(drone_id)
, total_drones_(total_drones)
, dt_nom_(0.01)
, l_(1.0)
, payload_m_(1.5)
, payload_radius_(0.25)
, m_drones_(0.25)
, kq_(9.5)
, kw_(3.4)
, alpha_gain_(0.10)
, z_weight_(0.3)
, thrust_bias_(0.0)
, slowdown_(1.0)
, payload_enu_(true)  // assume mocap publishes ENU PoseStamped
, apply_payload_offset_(true)
, acc_filter_(dt_nom_, 1e-5, 2e-5)
, traj_started_(false)
, traj_completed_(false)
, traj_time_init_(false)
, payload_ready_(false)
, odom_ready_(false)
, sim_pose_ready_(false)
, current_state_(FsmState::INIT)
, payload_omega_init_(false)
{
  dt_nom_ = this->declare_parameter("timer_period", dt_nom_);
  l_ = this->declare_parameter("l", l_);
  payload_m_ = this->declare_parameter("payload_mass", payload_m_);
  payload_radius_ = this->declare_parameter("payload_radius", payload_radius_);
  kq_ = this->declare_parameter("kq", kq_);
  kw_ = this->declare_parameter("kw", kw_);
  z_weight_ = this->declare_parameter("z_weight", z_weight_);
  thrust_bias_ = this->declare_parameter("thrust_bias", thrust_bias_);
  std::string data_root = this->declare_parameter(
    "data_root",
    std::string("data/realflight_traj_new"));
  alpha_gain_ = this->declare_parameter("alpha_gain", alpha_gain_);
  std::string log_path = this->declare_parameter("log_path", std::string(""));
  std::string payload_odom_topic = this->declare_parameter(
    "payload_odom_topic",
    std::string(""));
  std::string payload_pose_topic = this->declare_parameter(
    "payload_pose_topic",
    std::string("/payload_odom"));
  if (payload_odom_topic.empty()) {
    payload_odom_topic = payload_pose_topic;
  }
  apply_payload_offset_ = this->declare_parameter("apply_payload_offset", apply_payload_offset_);

  // build transforms
  T_enu2ned_ << 0, 1, 0,
                1, 0, 0,
                0, 0,-1;
  T_body_ = T_enu2ned_;
  T_flu2frd_ = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();

  payload_enu_ = this->declare_parameter("payload_is_enu", payload_enu_);

  // geometry
  double payload_r = payload_radius_;
  double offset_r = apply_payload_offset_ ? (l_ + payload_r) : 0.0;
  for (int i = 0; i < total_drones_; ++i) {
    double angle = 2.0 * M_PI * i / static_cast<double>(total_drones_);
    double y = payload_r * std::cos(angle);
    double x = payload_r * std::sin(angle);
    rho_.push_back(Eigen::Vector3d(x, y, 0.0));
    double y_off = offset_r * std::cos(angle);
    double x_off = offset_r * std::sin(angle);
    offset_pos_.push_back(Eigen::Vector3d(x_off, y_off, 0.0));
  }
  P_.resize(6, 3 * total_drones_);
  P_.setZero();
  for (int i = 0; i < total_drones_; ++i) {
    P_.block<3,3>(0, 3*i) = Eigen::Matrix3d::Identity();
    P_.block<3,3>(3, 3*i) = hat(rho_[i]);
  }

  // offline data
  data_ = std::make_shared<DataLoaderNew>(data_root, total_drones_);
  traj_duration_ = data_->payload().back().time;

  q_id_.resize(total_drones_, Eigen::Vector3d::Zero());
  mu_id_.resize(total_drones_, Eigen::Vector3d::Zero());
  omega_id_.resize(total_drones_, Eigen::Vector3d::Zero());
  omega_id_dot_.resize(total_drones_, Eigen::Vector3d::Zero());
  mu_id_prev_.resize(total_drones_, Eigen::Vector3d::Zero());
  mu_id_dot_.resize(total_drones_, Eigen::Vector3d::Zero());
  mu_id_ddot_.resize(total_drones_, Eigen::Vector3d::Zero());
  mu_id_dot_prev_.resize(total_drones_, Eigen::Vector3d::Zero());
  mu_id_ddot_prev_.resize(total_drones_, Eigen::Vector3d::Zero());
  q_id_dot_.resize(total_drones_, Eigen::Vector3d::Zero());
  q_id_ddot_.resize(total_drones_, Eigen::Vector3d::Zero());
  mu_dot_filter_init_.resize(total_drones_, false);
  mu_ddot_filter_init_.resize(total_drones_, false);
  mu_history_init_.resize(total_drones_, false);

  payload_pos_.setZero();
  payload_vel_.setZero();
  payload_acc_.setZero();
  payload_q_.setIdentity();
  payload_R_.setIdentity();
  payload_omega_.setZero();
  payload_omega_dot_.setZero();
  payload_omega_prev_.setZero();
  payload_q_enu_.setIdentity();

  drone_pos_.setZero();
  drone_vel_.setZero();
  drone_omega_.setZero();
  drone_R_.setIdentity();
  drone_acc_sp_.setZero();

  // subscriptions
  payload_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    payload_odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&GeomMultiliftHitlNode::payload_odom_cb, this, _1));

  // Simulation pose uses 1-indexed topics: position_drone_1 ... position_drone_N
  std::string sim_topic = "/simulation/position_drone_" + std::to_string(drone_id_ + 1);
  sim_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    sim_topic, rclcpp::QoS(10),
    std::bind(&GeomMultiliftHitlNode::sim_pose_cb, this, _1));

  std::string px4_ns = (drone_id_ == 0) ? "/fmu/" : "/px4_" + std::to_string(drone_id_) + "/fmu/";
  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    px4_ns + "out/vehicle_odometry", rclcpp::SensorDataQoS(),
    std::bind(&GeomMultiliftHitlNode::odom_cb, this, _1));

  lps_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>(
    px4_ns + "out/vehicle_local_position_setpoint", rclcpp::SensorDataQoS(),
    std::bind(&GeomMultiliftHitlNode::lps_setpoint_cb, this, _1));

  state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/state/state_drone_" + std::to_string(drone_id_), rclcpp::QoS(10),
    std::bind(&GeomMultiliftHitlNode::state_cb, this, _1));
  for (int i = 0; i < total_drones_; ++i) {
    swarm_states_[i] = FsmState::INIT;
    if (i == drone_id_) continue;
    auto sub = this->create_subscription<std_msgs::msg::Int32>(
      "/state/state_drone_" + std::to_string(i), rclcpp::QoS(10),
      [this, i](const std_msgs::msg::Int32::SharedPtr msg) {
        this->swarm_state_cb(msg, i);
      });
    swarm_subs_.push_back(sub);
  }

  att_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
    px4_ns + "in/vehicle_attitude_setpoint", rclcpp::QoS(10));
  traj_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    px4_ns + "in/trajectory_setpoint", rclcpp::QoS(10));
  cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "/state/command_drone_" + std::to_string(drone_id_), rclcpp::QoS(10));

  if (!log_path.empty()) {
    log_file_.open(log_path, std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
      log_file_ << "t,"
                // payload pos/vel (NED) + DDP errors (ENU)
                << "x_d_x,x_d_y,x_d_z,x_x,x_y,x_z,ex_ned_x,ex_ned_y,ex_ned_z,ex_enu_x,ex_enu_y,ex_enu_z,"
                << "v_d_x,v_d_y,v_d_z,v_x,v_y,v_z,ev_ned_x,ev_ned_y,ev_ned_z,ev_enu_x,ev_enu_y,ev_enu_z,"
                // DDP quaternion error (ENU, wxyz) + Euler angles (ENU, roll/pitch/yaw)
                << "q_d_w,q_d_x,q_d_y,q_d_z,q_w,q_x,q_y,q_z,eq_w,eq_x,eq_y,eq_z,"
                << "rpy_d_roll,rpy_d_pitch,rpy_d_yaw,rpy_roll,rpy_pitch,rpy_yaw,rpy_err_roll,rpy_err_pitch,rpy_err_yaw,"
                // payload body rates: ENU-mapped (for DDP) and raw body omega
                << "omega_d_enu_x,omega_d_enu_y,omega_d_enu_z,omega_enu_x,omega_enu_y,omega_enu_z,"
                << "eomega_enu_x,eomega_enu_y,eomega_enu_z,omega_body_x,omega_body_y,omega_body_z,"
                // DDP feedback wrench and delta_mu segment
                << "FM_body_Fx,FM_body_Fy,FM_body_Fz,FM_body_Mx,FM_body_My,FM_body_Mz,"
                << "FM_frd_Fx,FM_frd_Fy,FM_frd_Fz,FM_frd_Mx,FM_frd_My,FM_frd_Mz,"
                << "delta_mu_x,delta_mu_y,delta_mu_z,"
                // cable direction/omega setpoints, measurements, and errors
                << "q_id_x,q_id_y,q_id_z,q_i_x,q_i_y,q_i_z,q_diff_x,q_diff_y,q_diff_z,eq_cable_x,eq_cable_y,eq_cable_z,"
                << "omega_id_x,omega_id_y,omega_id_z,omega_i_x,omega_i_y,omega_i_z,"
                << "omega_diff_x,omega_diff_y,omega_diff_z,ew_cable_x,ew_cable_y,ew_cable_z"
                << "\n";
      log_enabled_ = true;
      RCLCPP_INFO(this->get_logger(), "Logging enabled: %s", log_path.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to open log file: %s", log_path.c_str());
    }
  }

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt_nom_),
    std::bind(&GeomMultiliftHitlNode::timer_cb, this));
}

Eigen::Matrix3d GeomMultiliftHitlNode::hat(const Eigen::Vector3d &v) const {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(),
       v.z(), 0, -v.x(),
      -v.y(), v.x(), 0;
  return m;
}

Eigen::Quaterniond GeomMultiliftHitlNode::quat_from_msg(const geometry_msgs::msg::PoseStamped &msg) const {
  return Eigen::Quaterniond(msg.pose.orientation.w,
                            msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z);
}

Eigen::Quaterniond GeomMultiliftHitlNode::quat_from_px4(const px4_msgs::msg::VehicleOdometry &msg) const {
  return Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
}

Eigen::Vector3d GeomMultiliftHitlNode::vee(const Eigen::Matrix3d &M) const {
  return Eigen::Vector3d(
    M(2,1) - M(1,2),
    M(0,2) - M(2,0),
    M(1,0) - M(0,1)
  ) * 0.5;
}

Eigen::Vector3d GeomMultiliftHitlNode::lowpass(
  const Eigen::Vector3d &x, Eigen::Vector3d &y_prev,
  double cutoff_hz, double dt, bool &init) const {
  if (!init) {
    y_prev = x;
    init = true;
    return x;
  }
  double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
  double alpha = dt / (tau + dt);
  y_prev = alpha * x + (1.0 - alpha) * y_prev;
  return y_prev;
}

bool GeomMultiliftHitlNode::all_ready() const {
  for (const auto &kv : swarm_states_) {
    if (kv.second != FsmState::TRAJ) return false;
  }
  return true;
}

void GeomMultiliftHitlNode::payload_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double dt = payload_ready_
    ? (rclcpp::Time(msg->header.stamp) - last_payload_stamp_).seconds()
    : dt_nom_;
  if (dt <= 0.0) dt = dt_nom_;

  Eigen::Vector3d meas(msg->pose.pose.position.x,
                       msg->pose.pose.position.y,
                       msg->pose.pose.position.z);
  Eigen::Quaterniond q_raw(msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z);
  // Enforce quaternion sign continuity for the DDP "direct subtraction" error.
  // q and -q represent the same rotation, but subtraction is not sign-invariant.
  // For your use-case the desired payload attitude is identity, so keep w >= 0
  // to avoid the (-q) representation looking like a large error.
  if (q_raw.w() < 0.0) {
    q_raw.w() *= -1.0;
    q_raw.x() *= -1.0;
    q_raw.y() *= -1.0;
    q_raw.z() *= -1.0;
  }
  if (payload_ready_) {
    double dot = q_raw.w() * payload_q_enu_.w() + q_raw.x() * payload_q_enu_.x() +
      q_raw.y() * payload_q_enu_.y() + q_raw.z() * payload_q_enu_.z();
    if (dot < 0.0) {
      q_raw.w() *= -1.0;
      q_raw.x() *= -1.0;
      q_raw.y() *= -1.0;
      q_raw.z() *= -1.0;
    }
  }
  Eigen::Quaterniond q_meas = q_raw;
  // Keep a copy of the raw ENU quaternion for DDP-style logging (wxyz).
  if (payload_enu_) {
    payload_q_enu_ = q_raw;
  } else {
    // If the incoming pose is already NED/FRD, compute the equivalent ENU/FLU quaternion.
    Eigen::Matrix3d R_ned = q_raw.toRotationMatrix();
    Eigen::Matrix3d R_enu = T_enu2ned_ * R_ned * T_flu2frd_;
    payload_q_enu_ = Eigen::Quaterniond(R_enu);
  }
  if (payload_enu_) {
    meas = T_enu2ned_ * meas;
    Eigen::Matrix3d R_enu = q_raw.toRotationMatrix();
    Eigen::Matrix3d R_ned = T_enu2ned_ * R_enu * T_flu2frd_;
    q_meas = Eigen::Quaterniond(R_ned);
  }
  acc_filter_.step(meas, dt);
  payload_pos_ = acc_filter_.pos();
  payload_vel_ = acc_filter_.vel();
  payload_acc_ = acc_filter_.acc();

  Eigen::Quaterniond q = q_meas;
  Eigen::Matrix3d R_prev = payload_R_;
  Eigen::Matrix3d R_curr = q.toRotationMatrix();
  Eigen::Matrix3d delta = R_prev.transpose() * R_curr;
  Eigen::AngleAxisd aa(delta);
  Eigen::Vector3d omega = aa.axis() * aa.angle() / dt;

  if (!payload_omega_init_) {
    payload_omega_dot_.setZero();
    payload_omega_init_ = true;
  } else {
    payload_omega_dot_ = (omega - payload_omega_prev_) / dt;
  }
  payload_omega_prev_ = omega;
  payload_omega_ = omega;
  payload_q_ = q;
  payload_R_ = R_curr;

  last_payload_odom_ = *msg;
  last_payload_stamp_ = msg->header.stamp;
  payload_ready_ = true;
}

void GeomMultiliftHitlNode::sim_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  Eigen::Vector3d pos_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  drone_pos_ = Eigen::Vector3d(pos_enu.y(), pos_enu.x(), -pos_enu.z());
  last_sim_pose_ = *msg;
  sim_pose_ready_ = true;
}

void GeomMultiliftHitlNode::odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  drone_vel_ = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
  drone_omega_ = Eigen::Vector3d(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);
  drone_R_ = quat_from_px4(*msg).toRotationMatrix();
  odom_ready_ = true;
}

void GeomMultiliftHitlNode::lps_setpoint_cb(const px4_msgs::msg::VehicleLocalPositionSetpoint::SharedPtr msg) {
  drone_acc_sp_ = Eigen::Vector3d(msg->acceleration[0], msg->acceleration[1], msg->acceleration[2]);
}

void GeomMultiliftHitlNode::state_cb(const std_msgs::msg::Int32::SharedPtr msg) {
  current_state_ = static_cast<FsmState>(msg->data);
  swarm_states_[drone_id_] = current_state_;
  if (current_state_ == FsmState::TRAJ && !traj_started_ && !traj_completed_) {
    // In real flight, a coordinator handles state sync; start immediately on TRAJ.
    traj_started_ = true;
    traj_time_init_ = false;
  }
}

void GeomMultiliftHitlNode::swarm_state_cb(const std_msgs::msg::Int32::SharedPtr msg, int other) {
  swarm_states_[other] = static_cast<FsmState>(msg->data);
}

void GeomMultiliftHitlNode::compute_desired(double t) {
  const auto &payload = data_->payload();
  const auto &cables = data_->cables();
  int idx = std::min(static_cast<int>(std::round(t / (data_->dt() * slowdown_))),
                     static_cast<int>(payload.size()) - 2);  // keep one-step margin
  int idx_k = std::min(idx, static_cast<int>(data_->kfb().size()) - 1);
  double dt_des = (control_dt_ > 1e-6) ? control_dt_ : dt_nom_;

  auto p = payload[idx];
  Eigen::Vector3d x_d_enu = Eigen::Vector3d(p.pos.x(), p.pos.y(), p.pos.z());
  Eigen::Vector3d v_d_enu = Eigen::Vector3d(p.vel.x(), p.vel.y(), p.vel.z());
  Eigen::Vector3d a_d_enu = Eigen::Vector3d(p.acc.x(), p.acc.y(), p.acc.z());
  x_d_ = T_enu2ned_ * x_d_enu;
  v_d_ = T_enu2ned_ * v_d_enu;
  a_d_ = T_enu2ned_ * a_d_enu;
  R_d_ = T_body_ * Eigen::Matrix3d::Identity() * T_enu2ned_;  // keep body aligned

  // errors for DDP
  Eigen::Vector3d e_x_ENU = T_enu2ned_ * (payload_pos_ - x_d_);
  Eigen::Vector3d e_v_ENU = T_enu2ned_ * (payload_vel_ - v_d_);
  Eigen::Quaterniond q_d_enu(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector4d q_meas_enu(payload_q_enu_.w(), payload_q_enu_.x(),
                             payload_q_enu_.y(), payload_q_enu_.z());
  Eigen::Vector4d q_des_enu(q_d_enu.w(), q_d_enu.x(), q_d_enu.y(), q_d_enu.z());
  Eigen::Vector4d e_q_ENU = q_meas_enu - q_des_enu;
  Eigen::Vector3d e_Omega_ENU = T_body_ * payload_omega_;

  Eigen::Matrix<double, 6, 13> K = data_->kfb()[idx_k].K;
  Eigen::Matrix<double, 13, 1> e_ddp;
  e_ddp << e_x_ENU, e_v_ENU, e_q_ENU, e_Omega_ENU;
  Eigen::Matrix<double, 6, 1> FM_BODY = alpha_gain_ * K * e_ddp;

  Eigen::Matrix<double, 3, 1> F_BODY = FM_BODY.segment<3>(0);
  Eigen::Matrix<double, 3, 1> M_BODY = FM_BODY.segment<3>(3);
  Eigen::Matrix<double, 3, 1> F_FRD = T_body_ * F_BODY;
  Eigen::Matrix<double, 3, 1> M_FRD = T_body_ * M_BODY;
  Eigen::Matrix<double, 6, 1> FM_FRD;
  FM_FRD << F_FRD, M_FRD;

  Eigen::Matrix<double, Eigen::Dynamic, 1> delta_mu =
    P_.transpose() * (P_ * P_.transpose()).inverse() * FM_FRD;

  for (int i = 0; i < total_drones_; ++i) {
    const auto &c = cables[i][idx];
    Eigen::Vector3d dir_ned = T_enu2ned_ * c.dir;
    double mu_scalar = payload_m_ * c.mu;
    mu_id_[i] = mu_scalar * dir_ned + payload_R_ * delta_mu.segment<3>(3 * i);

    // derivative filters to match Python smoothing (mu affects q_id through normalization)
    if (!mu_history_init_[i]) {
      mu_id_prev_[i] = mu_id_[i];
      mu_id_dot_[i].setZero();
      mu_id_ddot_[i].setZero();
      mu_id_dot_prev_[i].setZero();
      mu_id_ddot_prev_[i].setZero();
      mu_dot_filter_init_[i] = false;
      mu_ddot_filter_init_[i] = false;
      mu_history_init_[i] = true;
    } else {
      Eigen::Vector3d mu_dot = (mu_id_[i] - mu_id_prev_[i]) / dt_des;
      const Eigen::Vector3d mu_dot_prev = mu_id_dot_prev_[i];
      const bool dot_has_prev = mu_dot_filter_init_[i];
      bool init_ref = mu_dot_filter_init_[i];
      mu_id_dot_[i] = lowpass(mu_dot, mu_id_dot_prev_[i], 8.0, dt_des, init_ref);
      mu_dot_filter_init_[i] = init_ref;

      Eigen::Vector3d mu_ddot = Eigen::Vector3d::Zero();
      if (dot_has_prev) {
        mu_ddot = (mu_id_dot_[i] - mu_dot_prev) / dt_des;
      } else {
        mu_id_ddot_prev_[i].setZero();
        mu_ddot_filter_init_[i] = false;
      }
      init_ref = mu_ddot_filter_init_[i];
      mu_id_ddot_[i] = lowpass(mu_ddot, mu_id_ddot_prev_[i], 8.0, dt_des, init_ref);
      mu_ddot_filter_init_[i] = init_ref;

      mu_id_prev_[i] = mu_id_[i];
    }

    double mu_norm = mu_id_[i].norm();
    if (mu_norm < 1e-6) {
      q_id_[i].setZero();
      omega_id_[i].setZero();
      omega_id_dot_[i].setZero();
      q_id_dot_[i].setZero();
      q_id_ddot_[i].setZero();
      continue;
    }

    q_id_[i] = -mu_id_[i] / mu_norm;
    Eigen::Matrix3d proj = Eigen::Matrix3d::Identity() - q_id_[i] * q_id_[i].transpose();
    q_id_dot_[i] = -proj * mu_id_dot_[i] / mu_norm;
    omega_id_[i] = q_id_[i].cross(q_id_dot_[i]);

    // second derivative and desired cable angular acceleration
    double L = mu_norm;
    double L_dot = mu_id_[i].dot(mu_id_dot_[i]) / L;
    Eigen::Matrix3d proj_dot = - (q_id_dot_[i] * q_id_[i].transpose() + q_id_[i] * q_id_dot_[i].transpose());
    Eigen::Vector3d term1 = proj_dot * mu_id_dot_[i];
    Eigen::Vector3d term2 = proj * mu_id_ddot_[i];
    Eigen::Vector3d term3 = (proj * mu_id_dot_[i]) * (L_dot / L);
    q_id_ddot_[i] = -(term1 + term2) / L + term3;
    omega_id_dot_[i] = q_id_dot_[i].cross(q_id_ddot_[i]);
  }
  k_ddp_ = K;
}

void GeomMultiliftHitlNode::run_control(double sim_t) {
  compute_desired(sim_t);

  // payload terms
  Eigen::Matrix3d Omega_hat = hat(payload_omega_);
  Eigen::Matrix3d R_dot = payload_R_ * Omega_hat;

  Eigen::Vector3d g(0.0, 0.0, 9.81);

  Eigen::Vector3d mu_vec = Eigen::Vector3d::Zero();
  Eigen::Vector3d u_total = Eigen::Vector3d::Zero();

  // compute cable dynamics for this drone only
  Eigen::Vector3d vec = -drone_pos_ + payload_pos_ + payload_R_ * rho_[drone_id_];
  Eigen::Vector3d q_i = vec.normalized();
  Eigen::Vector3d q_i_dot = (-drone_vel_ + payload_vel_ + R_dot * rho_[drone_id_]) / l_;
  Eigen::Matrix3d q_hat = hat(q_i);

  Eigen::Vector3d mu_i = mu_id_[drone_id_].dot(q_i) * q_i;
  Eigen::Vector3d a_i = payload_acc_ - g + payload_R_ * (Omega_hat * (Omega_hat * rho_[drone_id_])) - payload_R_ * hat(rho_[drone_id_]) * payload_omega_dot_;
  Eigen::Vector3d omega_i = q_i.cross(q_i_dot);
  double omega_sq = omega_i.squaredNorm();

  Eigen::Vector3d u_parallel = mu_i + m_drones_ * l_ * omega_sq * q_i + m_drones_ * (q_i.dot(a_i) * q_i);

  Eigen::Vector3d e_qi = q_id_[drone_id_].cross(q_i);
  Eigen::Matrix3d q_hat_sq = q_hat * q_hat;
  Eigen::Vector3d e_omega_i = omega_i + q_hat_sq * omega_id_[drone_id_];
  Eigen::Vector3d u_vertical = m_drones_ * l_ * q_hat *
    (-kq_ * e_qi - kw_ * e_omega_i - q_i.dot(omega_id_[drone_id_]) * q_i_dot - q_hat_sq * omega_id_dot_[drone_id_])
    - m_drones_ * q_hat_sq * a_i;

  u_total = u_parallel + u_vertical;
  log_sample(sim_t, q_i, omega_i, e_qi, e_omega_i);
  if (u_total.norm() < 1e-4) return;

  Eigen::Vector3d b3 = -u_total / u_total.norm();
  // fuse with px4 accel setpoint to avoid jerk
  Eigen::Vector3d body_z = Eigen::Vector3d(-drone_acc_sp_.x(), -drone_acc_sp_.y(), -(9.81 - drone_acc_sp_.z()));
  if (body_z.norm() < 1e-3) body_z = b3;
  Eigen::Vector3d blended = (1.0 - z_weight_) * body_z + z_weight_ * b3;
  if (blended.norm() < 1e-6) {
    blended = b3;
  }
  Eigen::Vector3d fused = blended.normalized();

  Eigen::Vector3d A2 = b3.cross(fused);
  if (A2.norm() < 1e-6) {
    Eigen::Vector3d b1_ref = drone_R_.col(0);
    A2 = b3.cross(b1_ref);
    if (A2.norm() < 1e-6) {
      A2 = b3.cross(Eigen::Vector3d(1.0, 0.0, 0.0));
    }
    if (A2.norm() < 1e-6) {
      A2 = b3.cross(Eigen::Vector3d(0.0, 1.0, 0.0));
    }
  }
  Eigen::Vector3d b2c = A2.normalized();
  Eigen::Vector3d A1 = b2c.cross(fused);
  Eigen::Vector3d b1c = A1.normalized();
  Eigen::Matrix3d R_c;
  R_c.col(0) = b1c;
  R_c.col(1) = b2c;
  R_c.col(2) = fused;
  Eigen::Quaternionf q_set(R_c.cast<float>());
  if (att_sp_prev_valid_) {
    float dot = q_set.w() * att_sp_prev_.w() + q_set.x() * att_sp_prev_.x() +
      q_set.y() * att_sp_prev_.y() + q_set.z() * att_sp_prev_.z();
    if (dot < 0.0f) {
      q_set.w() *= -1.0f;
      q_set.x() *= -1.0f;
      q_set.y() *= -1.0f;
      q_set.z() *= -1.0f;
    }
  }
  att_sp_prev_ = q_set;
  att_sp_prev_valid_ = true;

  double f_i = -u_total.dot(drone_R_ * Eigen::Vector3d(0,0,1));
  double norm = -f_i / (7.90 * m_drones_ * 9.81) - thrust_bias_;
  norm = std::clamp(norm, -1.0, -0.1);

  px4_msgs::msg::VehicleAttitudeSetpoint att;
  att.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  att.q_d = {q_set.w(), q_set.x(), q_set.y(), q_set.z()};
  att.thrust_body = {0.0f, 0.0f, static_cast<float>(norm)};
  att_pub_->publish(att);

  // also push per-drone trajectory setpoint (from csv-derived payload/cable data) for visibility
  px4_msgs::msg::TrajectorySetpoint traj;
  traj.timestamp = att.timestamp;
  const Eigen::Vector3d x_id =
    x_d_ + R_d_ * rho_[drone_id_] - l_ * q_id_[drone_id_] - offset_pos_[drone_id_];
  const Eigen::Vector3d v_id = v_d_ - l_ * q_id_dot_[drone_id_];
  const Eigen::Vector3d a_id = a_d_ - l_ * q_id_ddot_[drone_id_];
  traj.position[0] = static_cast<float>(x_id.x());
  traj.position[1] = static_cast<float>(x_id.y());
  traj.position[2] = static_cast<float>(x_id.z());
  traj.velocity[0] = static_cast<float>(v_id.x());
  traj.velocity[1] = static_cast<float>(v_id.y());
  traj.velocity[2] = static_cast<float>(v_id.z());
  traj.acceleration[0] = static_cast<float>(a_id.x());
  traj.acceleration[1] = static_cast<float>(a_id.y());
  traj.acceleration[2] = static_cast<float>(a_id.z());
  traj_pub_->publish(traj);
}

void GeomMultiliftHitlNode::timer_cb() {
  if (!payload_ready_ || !odom_ready_ || !sim_pose_ready_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for payload odom, PX4 odometry, and sim pose");
    return;
  }

  if (current_state_ != FsmState::TRAJ) {
    traj_started_ = false;
    traj_time_init_ = false;
    control_dt_init_ = false;
    for (int i = 0; i < total_drones_; ++i) {
      mu_history_init_[i] = false;
      mu_dot_filter_init_[i] = false;
      mu_ddot_filter_init_[i] = false;
      mu_id_prev_[i].setZero();
      mu_id_dot_[i].setZero();
      mu_id_ddot_[i].setZero();
      mu_id_dot_prev_[i].setZero();
      mu_id_ddot_prev_[i].setZero();
      q_id_dot_[i].setZero();
      q_id_ddot_[i].setZero();
    }
    return;
  }

  if (!traj_started_) {
    if (total_drones_ == 1 || all_ready()) {
      traj_started_ = true;
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for swarm ready");
      return;
    }
  }

  if (!traj_time_init_) {
    traj_start_ = this->now();
    traj_time_init_ = true;
  }

  rclcpp::Time now = this->now();
  if (!control_dt_init_) {
    control_dt_ = dt_nom_;
    last_control_stamp_ = now;
    control_dt_init_ = true;
  } else {
    control_dt_ = (now - last_control_stamp_).seconds();
    if (control_dt_ <= 0.0) {
      control_dt_ = dt_nom_;
    }
    last_control_stamp_ = now;
  }

  double sim_t = (now - traj_start_).seconds();
  if (sim_t >= traj_duration_) {
    if (!traj_completed_) {
      traj_completed_ = true;
      std_msgs::msg::Int32 cmd;
      cmd.data = static_cast<int>(FsmState::END_TRAJ);
      for (int i = 0; i < 5; ++i) cmd_pub_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "Trajectory complete");
    }
    return;
  }

  run_control(sim_t);
}

void GeomMultiliftHitlNode::log_sample(double sim_t,
                                   const Eigen::Vector3d &q_i,
                                   const Eigen::Vector3d &omega_i,
                                   const Eigen::Vector3d &e_qi,
                                   const Eigen::Vector3d &e_omega_i) {
  if (!log_enabled_ || !log_file_.is_open()) return;

  // --- DDP error terms (match px4_offboard/geom_multilift.py) ---
  Eigen::Vector3d e_x_ned = payload_pos_ - x_d_;
  Eigen::Vector3d e_v_ned = payload_vel_ - v_d_;
  Eigen::Vector3d e_x_enu = T_enu2ned_ * e_x_ned;
  Eigen::Vector3d e_v_enu = T_enu2ned_ * e_v_ned;

  Eigen::Vector4d q_d_wxyz(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector4d q_wxyz(payload_q_enu_.w(), payload_q_enu_.x(),
                         payload_q_enu_.y(), payload_q_enu_.z());
  Eigen::Vector4d e_q_wxyz = q_wxyz - q_d_wxyz;

  auto wrap_pi = [](double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  };
  // Use an explicit quaternion->RPY conversion to avoid Euler branch ambiguity
  // (Eigen::eulerAngles may return the pi-shifted solution near identity).
  const double qw = payload_q_enu_.w();
  const double qx = payload_q_enu_.x();
  const double qy = payload_q_enu_.y();
  const double qz = payload_q_enu_.z();
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  double pitch = 0.0;
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  Eigen::Vector3d rpy(roll, pitch, yaw);
  Eigen::Vector3d rpy_d(0.0, 0.0, 0.0);
  Eigen::Vector3d rpy_err(wrap_pi(rpy.x() - rpy_d.x()),
                          wrap_pi(rpy.y() - rpy_d.y()),
                          wrap_pi(rpy.z() - rpy_d.z()));

  Eigen::Vector3d omega_d_enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_enu = T_body_ * payload_omega_;
  Eigen::Vector3d e_omega_enu = omega_enu - omega_d_enu;

  Eigen::Matrix<double, 13, 1> e_ddp;
  e_ddp << e_x_enu, e_v_enu, e_q_wxyz, e_omega_enu;
  Eigen::Matrix<double, 6, 1> FM_body = alpha_gain_ * k_ddp_ * e_ddp;
  Eigen::Matrix<double, 3, 1> F_body = FM_body.segment<3>(0);
  Eigen::Matrix<double, 3, 1> M_body = FM_body.segment<3>(3);
  Eigen::Matrix<double, 3, 1> F_frd = T_body_ * F_body;
  Eigen::Matrix<double, 3, 1> M_frd = T_body_ * M_body;

  Eigen::Matrix<double, 6, 1> FM_frd;
  FM_frd << F_frd, M_frd;

  Eigen::VectorXd delta_mu =
    P_.transpose() * (P_ * P_.transpose()).inverse() * FM_frd;
  Eigen::Vector3d delta_mu_self = delta_mu.segment<3>(3 * drone_id_);

  // --- Cable terms ---
  Eigen::Vector3d q_diff = q_id_[drone_id_] - q_i;
  Eigen::Vector3d omega_diff = omega_i - omega_id_[drone_id_];

  log_file_ << std::fixed << std::setprecision(6)
            << sim_t << ","
            << x_d_.x() << "," << x_d_.y() << "," << x_d_.z() << ","
            << payload_pos_.x() << "," << payload_pos_.y() << "," << payload_pos_.z() << ","
            << e_x_ned.x() << "," << e_x_ned.y() << "," << e_x_ned.z() << ","
            << e_x_enu.x() << "," << e_x_enu.y() << "," << e_x_enu.z() << ","
            << v_d_.x() << "," << v_d_.y() << "," << v_d_.z() << ","
            << payload_vel_.x() << "," << payload_vel_.y() << "," << payload_vel_.z() << ","
            << e_v_ned.x() << "," << e_v_ned.y() << "," << e_v_ned.z() << ","
            << e_v_enu.x() << "," << e_v_enu.y() << "," << e_v_enu.z() << ","
            << q_d_wxyz[0] << "," << q_d_wxyz[1] << "," << q_d_wxyz[2] << "," << q_d_wxyz[3] << ","
            << q_wxyz[0] << "," << q_wxyz[1] << "," << q_wxyz[2] << "," << q_wxyz[3] << ","
            << e_q_wxyz[0] << "," << e_q_wxyz[1] << "," << e_q_wxyz[2] << "," << e_q_wxyz[3] << ","
            << rpy_d.x() << "," << rpy_d.y() << "," << rpy_d.z() << ","
            << rpy.x() << "," << rpy.y() << "," << rpy.z() << ","
            << rpy_err.x() << "," << rpy_err.y() << "," << rpy_err.z() << ","
            << omega_d_enu.x() << "," << omega_d_enu.y() << "," << omega_d_enu.z() << ","
            << omega_enu.x() << "," << omega_enu.y() << "," << omega_enu.z() << ","
            << e_omega_enu.x() << "," << e_omega_enu.y() << "," << e_omega_enu.z() << ","
            << payload_omega_.x() << "," << payload_omega_.y() << "," << payload_omega_.z() << ","
            << FM_body(0) << "," << FM_body(1) << "," << FM_body(2) << ","
            << FM_body(3) << "," << FM_body(4) << "," << FM_body(5) << ","
            << FM_frd(0) << "," << FM_frd(1) << "," << FM_frd(2) << ","
            << FM_frd(3) << "," << FM_frd(4) << "," << FM_frd(5) << ","
            << delta_mu_self.x() << "," << delta_mu_self.y() << "," << delta_mu_self.z() << ","
            << q_id_[drone_id_].x() << "," << q_id_[drone_id_].y() << "," << q_id_[drone_id_].z() << ","
            << q_i.x() << "," << q_i.y() << "," << q_i.z() << ","
            << q_diff.x() << "," << q_diff.y() << "," << q_diff.z() << ","
            << e_qi.x() << "," << e_qi.y() << "," << e_qi.z() << ","
            << omega_id_[drone_id_].x() << "," << omega_id_[drone_id_].y() << "," << omega_id_[drone_id_].z() << ","
            << omega_i.x() << "," << omega_i.y() << "," << omega_i.z() << ","
            << omega_diff.x() << "," << omega_diff.y() << "," << omega_diff.z() << ","
            << e_omega_i.x() << "," << e_omega_i.y() << "," << e_omega_i.z()
            << "\n";
}
