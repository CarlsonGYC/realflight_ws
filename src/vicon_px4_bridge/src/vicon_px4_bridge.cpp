#include "vicon_px4_bridge/vicon_px4_bridge.hpp"

ViconPX4Bridge::ViconPX4Bridge() : Node("vicon_px4_bridge")
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("vicon_topic_name", "/vicon/drone/pose");
    this->declare_parameter<std::string>("px4_topic_name", "/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<std::string>("vicon_topic_type", "pose");
    this->declare_parameter<std::string>("input_frame", "ENU");
    this->declare_parameter<std::string>("output_frame", "NED");
    
    // Get parameters
    vicon_topic_name_ = this->get_parameter("vicon_topic_name").as_string();
    px4_topic_name_ = this->get_parameter("px4_topic_name").as_string();
    vicon_topic_type_ = this->get_parameter("vicon_topic_type").as_string();
    input_frame_ = this->get_parameter("input_frame").as_string();
    output_frame_ = this->get_parameter("output_frame").as_string();
    
    RCLCPP_INFO(this->get_logger(), "=== Vicon PX4 Bridge Configuration ===");
    RCLCPP_INFO(this->get_logger(), "Vicon topic: %s", vicon_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "PX4 topic: %s", px4_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic type: %s", vicon_topic_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame conversion: %s -> %s", 
                input_frame_.c_str(), output_frame_.c_str());
    
    // Validate frame types
    if (input_frame_ != "ENU" && input_frame_ != "FLU" && input_frame_ != "NED") {
        RCLCPP_ERROR(this->get_logger(), "Invalid input_frame: %s. Must be ENU, FLU, or NED", 
                     input_frame_.c_str());
        throw std::runtime_error("Invalid input frame");
    }
    
    if (output_frame_ != "NED" && output_frame_ != "ENU" && output_frame_ != "FLU") {
        RCLCPP_ERROR(this->get_logger(), "Invalid output_frame: %s. Must be ENU, FLU, or NED", 
                     output_frame_.c_str());
        throw std::runtime_error("Invalid output frame");
    }
    
    // Calculate transformation matrix and quaternion
    R_transform_ = getTransformationMatrix(input_frame_, output_frame_);
    q_transform_ = getFrameRotation(input_frame_, output_frame_);
    
    RCLCPP_INFO(this->get_logger(), "Transformation matrix:");
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]", 
                R_transform_(0,0), R_transform_(0,1), R_transform_(0,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]", 
                R_transform_(1,0), R_transform_(1,1), R_transform_(1,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]", 
                R_transform_(2,0), R_transform_(2,1), R_transform_(2,2));
    
    // Create publisher
    auto qos_px4 = rclcpp::SensorDataQoS();
    px4_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        px4_topic_name_, 10);
    
    // Create subscriber based on topic type
    if (vicon_topic_type_ == "pose") {
        vicon_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            vicon_topic_name_, qos_px4,
            std::bind(&ViconPX4Bridge::viconPoseCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to PoseStamped: %s", 
                    vicon_topic_name_.c_str());
    } else if (vicon_topic_type_ == "transform") {
        vicon_transform_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            vicon_topic_name_, 10,
            std::bind(&ViconPX4Bridge::viconTransformCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to TransformStamped: %s", 
                    vicon_topic_name_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid topic type: %s. Must be 'pose' or 'transform'",
                     vicon_topic_type_.c_str());
        throw std::runtime_error("Invalid topic type");
    }
    
    RCLCPP_INFO(this->get_logger(), "=== Bridge Ready ===");
}

Eigen::Matrix3d ViconPX4Bridge::getTransformationMatrix(const std::string& from_frame,
                                                        const std::string& to_frame)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    
    if (from_frame == to_frame) {
        return R;
    }
    
    // Define transformation matrices for each frame pair
    if (from_frame == "ENU" && to_frame == "NED") {
        // ENU: X=East, Y=North, Z=Up
        // NED: X=North, Y=East, Z=Down
        // Transformation: [X_ned, Y_ned, Z_ned]^T = R * [X_enu, Y_enu, Z_enu]^T
        R << 0,  1,  0,   // X_ned = Y_enu (North)
             1,  0,  0,   // Y_ned = X_enu (East)
             0,  0, -1;   // Z_ned = -Z_enu (Down = -Up)
    }
    else if (from_frame == "NED" && to_frame == "ENU") {
        // NED to ENU (inverse of above)
        R << 0,  1,  0,   // X_enu = Y_ned (East)
             1,  0,  0,   // Y_enu = X_ned (North)
             0,  0, -1;   // Z_enu = -Z_ned (Up = -Down)
    }
    else if (from_frame == "FLU" && to_frame == "NED") {
        // FLU: X=Forward, Y=Left, Z=Up (body frame, assuming Forward=North at zero yaw)
        // NED: X=North, Y=East, Z=Down
        R << 1,  0,  0,   // X_ned = X_flu (Forward=North)
             0, -1,  0,   // Y_ned = -Y_flu (East=-Left, Right=East)
             0,  0, -1;   // Z_ned = -Z_flu (Down=-Up)
    }
    else if (from_frame == "NED" && to_frame == "FLU") {
        // NED to FLU (inverse of above)
        R << 1,  0,  0,   // X_flu = X_ned (Forward=North)
             0, -1,  0,   // Y_flu = -Y_ned (Left=-East)
             0,  0, -1;   // Z_flu = -Z_ned (Up=-Down)
    }
    else if (from_frame == "ENU" && to_frame == "FLU") {
        // ENU to FLU: First ENU->NED, then NED->FLU
        Eigen::Matrix3d R_enu_to_ned;
        R_enu_to_ned << 0,  1,  0,
                        1,  0,  0,
                        0,  0, -1;
        Eigen::Matrix3d R_ned_to_flu;
        R_ned_to_flu << 1,  0,  0,
                        0, -1,  0,
                        0,  0, -1;
        R = R_ned_to_flu * R_enu_to_ned;
    }
    else if (from_frame == "FLU" && to_frame == "ENU") {
        // FLU to ENU: inverse of above
        Eigen::Matrix3d R_flu_to_ned;
        R_flu_to_ned << 1,  0,  0,
                        0, -1,  0,
                        0,  0, -1;
        Eigen::Matrix3d R_ned_to_enu;
        R_ned_to_enu << 0,  1,  0,
                        1,  0,  0,
                        0,  0, -1;
        R = R_ned_to_enu * R_flu_to_ned;
    }
    
    return R;
}

Eigen::Quaterniond ViconPX4Bridge::getFrameRotation(const std::string& from_frame,
                                                    const std::string& to_frame)
{
    Eigen::Matrix3d R = getTransformationMatrix(from_frame, to_frame);
    Eigen::Quaterniond q(R);
    return q;
}

void ViconPX4Bridge::viconPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    geometry_msgs::msg::Pose pose_converted;
    convertFrame(msg->pose, pose_converted);
    publishToPX4(pose_converted, msg->header.stamp);
}

void ViconPX4Bridge::viconTransformCallback(
    const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    geometry_msgs::msg::Transform transform_converted;
    convertFrame(msg->transform, transform_converted);
    
    // Convert Transform to Pose
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_converted.translation.x;
    pose.position.y = transform_converted.translation.y;
    pose.position.z = transform_converted.translation.z;
    pose.orientation = transform_converted.rotation;
    
    publishToPX4(pose, msg->header.stamp);
}

void ViconPX4Bridge::convertFrame(const geometry_msgs::msg::Pose& pose_in,
                                   geometry_msgs::msg::Pose& pose_out)
{
    // Convert position using transformation matrix
    Eigen::Vector3d pos_in(pose_in.position.x, pose_in.position.y, pose_in.position.z);
    Eigen::Vector3d pos_out = R_transform_ * pos_in;
    
    pose_out.position.x = pos_out.x();
    pose_out.position.y = pos_out.y();
    pose_out.position.z = pos_out.z();
    
    // Convert orientation quaternion
    Eigen::Quaterniond q_in(pose_in.orientation.w, pose_in.orientation.x,
                           pose_in.orientation.y, pose_in.orientation.z);
    
    // Apply frame rotation: q_out = q_frame_rotation * q_in * q_frame_rotation^-1
    // But for our coordinate frame changes, we use: q_out = q_frame_rotation * q_in
    Eigen::Quaterniond q_out = q_transform_ * q_in * q_transform_.inverse();
    q_out.normalize();
    
    pose_out.orientation.w = q_out.w();
    pose_out.orientation.x = q_out.x();
    pose_out.orientation.y = q_out.y();
    pose_out.orientation.z = q_out.z();
}

void ViconPX4Bridge::convertFrame(const geometry_msgs::msg::Transform& transform_in,
                                   geometry_msgs::msg::Transform& transform_out)
{
    // Convert translation
    Eigen::Vector3d trans_in(transform_in.translation.x, 
                            transform_in.translation.y, 
                            transform_in.translation.z);
    Eigen::Vector3d trans_out = R_transform_ * trans_in;
    
    transform_out.translation.x = trans_out.x();
    transform_out.translation.y = trans_out.y();
    transform_out.translation.z = trans_out.z();
    
    // Convert rotation
    Eigen::Quaterniond q_in(transform_in.rotation.w, transform_in.rotation.x,
                           transform_in.rotation.y, transform_in.rotation.z);
    
    Eigen::Quaterniond q_out = q_transform_ * q_in;
    q_out.normalize();
    
    transform_out.rotation.w = q_out.w();
    transform_out.rotation.x = q_out.x();
    transform_out.rotation.y = q_out.y();
    transform_out.rotation.z = q_out.z();
}

void ViconPX4Bridge::publishToPX4(const geometry_msgs::msg::Pose& pose_output,
                                   const rclcpp::Time& timestamp)
{
    px4_msgs::msg::VehicleOdometry odom_msg;
    
    // Set timestamp (PX4 uses microseconds)
    odom_msg.timestamp = timestamp.nanoseconds() / 1000;
    odom_msg.timestamp_sample = odom_msg.timestamp;
    
    // Set pose frame to NED (local frame) - PX4 always expects NED
    odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    
    // Position
    odom_msg.position[0] = static_cast<float>(pose_output.position.x);
    odom_msg.position[1] = static_cast<float>(pose_output.position.y);
    odom_msg.position[2] = static_cast<float>(pose_output.position.z);
    
    // Orientation quaternion (w, x, y, z in PX4)
    odom_msg.q[0] = static_cast<float>(pose_output.orientation.w);
    odom_msg.q[1] = static_cast<float>(pose_output.orientation.x);
    odom_msg.q[2] = static_cast<float>(pose_output.orientation.y);
    odom_msg.q[3] = static_cast<float>(pose_output.orientation.z);
    
    // Velocity (set to NaN as we don't have velocity from pose)
    odom_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    odom_msg.velocity[0] = NAN;
    odom_msg.velocity[1] = NAN;
    odom_msg.velocity[2] = NAN;
    
    // Angular velocity (set to NaN)
    odom_msg.angular_velocity[0] = NAN;
    odom_msg.angular_velocity[1] = NAN;
    odom_msg.angular_velocity[2] = NAN;
    
    // Covariances (unknown)
    for (int i = 0; i < 21; i++) {
        odom_msg.position_variance[i] = NAN;
        odom_msg.orientation_variance[i] = NAN;
        odom_msg.velocity_variance[i] = NAN;
    }
    
    // Publish
    px4_odom_pub_->publish(odom_msg);
    
    // Log at reduced rate
    static int counter = 0;
    if (counter++ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), 
                    "Published: pos=[%.3f, %.3f, %.3f] quat=[%.3f, %.3f, %.3f, %.3f]",
                    odom_msg.position[0], odom_msg.position[1], odom_msg.position[2],
                    odom_msg.q[0], odom_msg.q[1], odom_msg.q[2], odom_msg.q[3]);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ViconPX4Bridge>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("vicon_px4_bridge"), 
                     "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
