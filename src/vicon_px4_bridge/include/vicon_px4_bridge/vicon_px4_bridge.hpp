#ifndef VICON_PX4_BRIDGE_HPP
#define VICON_PX4_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class ViconPX4Bridge : public rclcpp::Node
{
public:
    ViconPX4Bridge();

private:
    // Callback functions
    void viconPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void viconTransformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    
    // Coordinate transformation functions
    Eigen::Matrix3d getTransformationMatrix(const std::string& from_frame,
                                           const std::string& to_frame);
    
    void convertFrame(const geometry_msgs::msg::Pose& pose_in,
                     geometry_msgs::msg::Pose& pose_out);
    
    void convertFrame(const geometry_msgs::msg::Transform& transform_in,
                     geometry_msgs::msg::Transform& transform_out);
    
    void publishToPX4(const geometry_msgs::msg::Pose& pose_output,
                     const rclcpp::Time& timestamp);
    
    // ROS2 publishers and subscribers
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr vicon_transform_sub_;
    
    // Parameters
    std::string vicon_topic_name_;
    std::string px4_topic_name_;
    std::string vicon_topic_type_;
    
    // MODIFIED: Separate world frame and body frame parameters
    std::string input_world_frame_;   // e.g., "ENU"
    std::string output_world_frame_;  // e.g., "NED"
    std::string input_body_frame_;    // e.g., "FLU"
    std::string output_body_frame_;   // e.g., "FRD"
    
    // MODIFIED: Separate transformation matrices and quaternions for world and body frames
    Eigen::Matrix3d R_world_transform_;     // World frame transformation matrix
    Eigen::Matrix3d R_body_transform_;      // Body frame transformation matrix
    Eigen::Quaterniond q_world_transform_;  // World frame transformation quaternion (left multiply)
    Eigen::Quaterniond q_body_transform_;   // Body frame transformation quaternion (right multiply)
};

#endif // VICON_PX4_BRIDGE_HPP