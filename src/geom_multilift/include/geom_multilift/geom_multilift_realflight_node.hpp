#pragma once

#include "geom_multilift/data_loader_new.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/int32.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <fstream>
#include <vector>

class AccKalmanFilter {
public:
  explicit AccKalmanFilter(double dt_init = 0.01, double q_var = 1e-5, double r_var = 2e-5);
  void step(const Eigen::Vector3d &z, double dt);
  const Eigen::Vector3d &pos() const { return pos_; }
  const Eigen::Vector3d &vel() const { return vel_; }
  const Eigen::Vector3d &acc() const { return acc_; }
private:
  void build(double dt);
  Eigen::Matrix<double, 9, 9> F_;
  Eigen::Matrix<double, 9, 9> Q_;
  Eigen::Matrix<double, 3, 9> H_;
  Eigen::Matrix3d R_;
  Eigen::Matrix<double, 9, 1> x_;
  Eigen::Matrix<double, 9, 9> P_;
  bool init_;
  double q_;
  double dt_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
};

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

class GeomMultiliftRealflightNode : public rclcpp::Node {
public:
  GeomMultiliftRealflightNode(int drone_id, int total_drones);

private:
  void payload_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void lps_setpoint_cb(const px4_msgs::msg::VehicleLocalPositionSetpoint::SharedPtr msg);
  void state_cb(const std_msgs::msg::Int32::SharedPtr msg);
  void swarm_state_cb(const std_msgs::msg::Int32::SharedPtr msg, int other);
  void timer_cb();

  // helpers
  Eigen::Matrix3d hat(const Eigen::Vector3d &v) const;
  Eigen::Quaterniond quat_from_msg(const geometry_msgs::msg::PoseStamped &msg) const;
  Eigen::Quaterniond quat_from_px4(const px4_msgs::msg::VehicleOdometry &msg) const;
  bool all_ready() const;

  // desired interpolation
  void compute_desired(double t);

  // controller
  void run_control(double sim_t);
  Eigen::Vector3d vee(const Eigen::Matrix3d &M) const;
  Eigen::Vector3d lowpass(const Eigen::Vector3d &x, Eigen::Vector3d &y_prev,
                          double cutoff_hz, double dt, bool &init) const;
  void log_sample(double sim_t,
                  const Eigen::Vector3d &q_i,
                  const Eigen::Vector3d &omega_i,
                  const Eigen::Vector3d &e_qi,
                  const Eigen::Vector3d &e_omega_i);

  // parameters
  int drone_id_;
  int total_drones_;
  double dt_nom_;
  double l_;           // cable length
  double payload_m_;
  double payload_radius_;
  double m_drones_;
  double kq_;
  double kw_;
  double alpha_gain_;
  double z_weight_;
  double thrust_bias_;
  double thrust_to_weight_ratio_;
  double slowdown_;
  bool payload_enu_;
  bool apply_payload_offset_;

  Eigen::Matrix3d T_enu2ned_;
  Eigen::Matrix3d T_body_;
  Eigen::Matrix3d T_flu2frd_;
  std::vector<Eigen::Vector3d> rho_;
  std::vector<Eigen::Vector3d> offset_pos_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> P_;  // 6 x 3n
  std::ofstream log_file_;
  bool log_enabled_{false};
  Eigen::Quaternionf att_sp_prev_;
  bool att_sp_prev_valid_{false};

  // offline data
  std::shared_ptr<DataLoaderNew> data_;
  double traj_duration_;

  // filters
  AccKalmanFilter acc_filter_;

  // ROS handles
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr payload_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr lps_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> swarm_subs_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // current states
  bool payload_ready_;
  bool odom_ready_;
  bool local_pos_ready_;
  geometry_msgs::msg::PoseStamped last_payload_pose_;
  rclcpp::Time last_payload_stamp_;
  Eigen::Vector3d payload_pos_;
  Eigen::Vector3d payload_vel_;
  Eigen::Vector3d payload_acc_;
  Eigen::Quaterniond payload_q_;
  Eigen::Matrix3d payload_R_;
  Eigen::Quaterniond payload_q_enu_;
  Eigen::Vector3d payload_omega_;
  Eigen::Vector3d payload_omega_dot_;
  Eigen::Vector3d payload_omega_prev_;
  bool payload_omega_init_;

  Eigen::Vector3d drone_pos_;
  Eigen::Vector3d drone_vel_;
  Eigen::Vector3d drone_omega_;
  Eigen::Matrix3d drone_R_;
  Eigen::Vector3d drone_acc_sp_;

  // desired states
  Eigen::Vector3d x_d_;
  Eigen::Vector3d v_d_;
  Eigen::Matrix3d R_d_;
  Eigen::Vector3d a_d_;
  Eigen::Vector3d e_x_;
  Eigen::Vector3d e_v_;
  Eigen::Vector3d e_R_;
  Eigen::Vector3d e_Omega_;
  Eigen::Matrix<double, 6, 13> k_ddp_;
  std::vector<Eigen::Vector3d> q_id_;
  std::vector<Eigen::Vector3d> mu_id_;
  std::vector<Eigen::Vector3d> omega_id_;
  std::vector<Eigen::Vector3d> omega_id_dot_;
  std::vector<Eigen::Vector3d> mu_id_prev_;
  std::vector<Eigen::Vector3d> mu_id_dot_;
  std::vector<Eigen::Vector3d> mu_id_ddot_;
  std::vector<Eigen::Vector3d> mu_id_dot_prev_;
  std::vector<Eigen::Vector3d> mu_id_ddot_prev_;
  std::vector<Eigen::Vector3d> q_id_dot_;
  std::vector<Eigen::Vector3d> q_id_ddot_;
  std::vector<bool> mu_dot_filter_init_;
  std::vector<bool> mu_ddot_filter_init_;
  std::vector<bool> mu_history_init_;

  // timers
  rclcpp::Time traj_start_;
  bool traj_started_;
  bool traj_completed_;
  bool traj_time_init_;
  rclcpp::Time last_control_stamp_;
  bool control_dt_init_{false};
  double control_dt_{0.0};

  // fsm
  FsmState current_state_;
  std::map<int, FsmState> swarm_states_;
};
