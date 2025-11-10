#ifndef VICON_PX4_BRIDGE_HPP
#define VICON_PX4_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class ViconPX4Bridge : public rclcpp::Node
{
public:
    ViconPX4Bridge();
    ~ViconPX4Bridge() = default;

private:
    // Callbacks
    void viconPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void viconTransformCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    
    // Frame conversion functions
    void convertFrame(const geometry_msgs::msg::Pose& pose_in, 
                     geometry_msgs::msg::Pose& pose_out);
    void convertFrame(const geometry_msgs::msg::Transform& transform_in,
                     geometry_msgs::msg::Transform& transform_out);
    
    // Get transformation matrix based on input/output frames
    Eigen::Matrix3d getTransformationMatrix(const std::string& from_frame, 
                                           const std::string& to_frame);
    Eigen::Quaterniond getFrameRotation(const std::string& from_frame,
                                       const std::string& to_frame);
    
    // Publish to PX4
    void publishToPX4(const geometry_msgs::msg::Pose& pose_ned, 
                      const rclcpp::Time& timestamp);
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr vicon_transform_sub_;
    
    // Publisher
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_pub_;
    
    // Parameters
    std::string vicon_topic_name_;
    std::string px4_topic_name_;
    std::string vicon_topic_type_;
    std::string input_frame_;   // ENU, FLU, NED
    std::string output_frame_;  // NED (for PX4)
    
    // Transformation matrices
    Eigen::Matrix3d R_transform_;
    Eigen::Quaterniond q_transform_;
};

#endif // VICON_PX4_BRIDGE_HPP