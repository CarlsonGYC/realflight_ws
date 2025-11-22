#include "vicon_px4_bridge/vicon_px4_bridge.hpp"

ViconPX4Bridge::ViconPX4Bridge() : Node("vicon_px4_bridge")
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("vicon_topic_name", "/vicon/drone/pose");
    this->declare_parameter<std::string>("px4_topic_name", "/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<std::string>("vicon_topic_type", "pose");
    this->declare_parameter<std::string>("input_world_frame", "ENU");
    this->declare_parameter<std::string>("output_world_frame", "NED");
    this->declare_parameter<std::string>("input_body_frame", "FLU");
    this->declare_parameter<std::string>("output_body_frame", "FRD");
    
    // Get parameters
    vicon_topic_name_ = this->get_parameter("vicon_topic_name").as_string();
    px4_topic_name_ = this->get_parameter("px4_topic_name").as_string();
    vicon_topic_type_ = this->get_parameter("vicon_topic_type").as_string();
    input_world_frame_ = this->get_parameter("input_world_frame").as_string();
    output_world_frame_ = this->get_parameter("output_world_frame").as_string();
    input_body_frame_ = this->get_parameter("input_body_frame").as_string();
    output_body_frame_ = this->get_parameter("output_body_frame").as_string();
    
    RCLCPP_INFO(this->get_logger(), "=== Vicon PX4 Bridge Configuration ===");
    RCLCPP_INFO(this->get_logger(), "Vicon topic: %s", vicon_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "PX4 topic: %s", px4_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic type: %s", vicon_topic_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "World frame: %s -> %s", 
                input_world_frame_.c_str(), output_world_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Body frame: %s -> %s", 
                input_body_frame_.c_str(), output_body_frame_.c_str());
    
    // Validate frame types
    std::vector<std::string> valid_frames = {"ENU", "FLU", "NED", "FRD"};
    auto validate = [&](const std::string& frame, const std::string& name) {
        if (std::find(valid_frames.begin(), valid_frames.end(), frame) == valid_frames.end()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid %s: %s", name.c_str(), frame.c_str());
            throw std::runtime_error("Invalid frame: " + name);
        }
    };
    
    validate(input_world_frame_, "input_world_frame");
    validate(output_world_frame_, "output_world_frame");
    validate(input_body_frame_, "input_body_frame");
    validate(output_body_frame_, "output_body_frame");
    
    // Calculate transformation matrices and quaternions
    // World frame transformation (left multiply)
    R_world_transform_ = getTransformationMatrix(input_world_frame_, output_world_frame_);
    q_world_transform_ = Eigen::Quaterniond(R_world_transform_);
    
    // Body frame transformation (right multiply)
    R_body_transform_ = getTransformationMatrix(input_body_frame_, output_body_frame_);
    q_body_transform_ = Eigen::Quaterniond(R_body_transform_);
    
    RCLCPP_INFO(this->get_logger(), "World transformation matrix (%s -> %s):",
                input_world_frame_.c_str(), output_world_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]", 
                R_world_transform_(0,0), R_world_transform_(0,1), R_world_transform_(0,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]", 
                R_world_transform_(1,0), R_world_transform_(1,1), R_world_transform_(1,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]", 
                R_world_transform_(2,0), R_world_transform_(2,1), R_world_transform_(2,2));
    
    RCLCPP_INFO(this->get_logger(), "Body transformation matrix (%s -> %s):",
                input_body_frame_.c_str(), output_body_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]", 
                R_body_transform_(0,0), R_body_transform_(0,1), R_body_transform_(0,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]", 
                R_body_transform_(1,0), R_body_transform_(1,1), R_body_transform_(1,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]", 
                R_body_transform_(2,0), R_body_transform_(2,1), R_body_transform_(2,2));
    
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
    
    // Define transformation matrices for common frame pairs
    // Convention: R transforms a vector from 'from_frame' to 'to_frame'
    // v_to = R * v_from
    
    if (from_frame == "ENU" && to_frame == "NED") {
        // ENU: X=East, Y=North, Z=Up
        // NED: X=North, Y=East, Z=Down
        R << 0,  1,  0,   // X_ned = Y_enu (North)
             1,  0,  0,   // Y_ned = X_enu (East)
             0,  0, -1;   // Z_ned = -Z_enu (Down)
    }
    else if (from_frame == "NED" && to_frame == "ENU") {
        // NED to ENU (inverse of above)
        R << 0,  1,  0,   // X_enu = Y_ned (East)
             1,  0,  0,   // Y_enu = X_ned (North)
             0,  0, -1;   // Z_enu = -Z_ned (Up)
    }
    else if (from_frame == "FLU" && to_frame == "FRD") {
        // FLU: X=Forward, Y=Left, Z=Up
        // FRD: X=Forward, Y=Right, Z=Down
        R << 1,  0,  0,   // X_frd = X_flu (Forward)
             0, -1,  0,   // Y_frd = -Y_flu (Right = -Left)
             0,  0, -1;   // Z_frd = -Z_flu (Down = -Up)
    }
    else if (from_frame == "FRD" && to_frame == "FLU") {
        // FRD to FLU (inverse of above)
        R << 1,  0,  0,   // X_flu = X_frd (Forward)
             0, -1,  0,   // Y_flu = -Y_frd (Left = -Right)
             0,  0, -1;   // Z_flu = -Z_frd (Up = -Down)
    }
    else if (from_frame == "FLU" && to_frame == "NED") {
        // FLU to NED: assuming FLU Forward aligns with North
        R << 1,  0,  0,   // X_ned = X_flu (Forward=North)
             0, -1,  0,   // Y_ned = -Y_flu (East=-Left)
             0,  0, -1;   // Z_ned = -Z_flu (Down=-Up)
    }
    else if (from_frame == "NED" && to_frame == "FLU") {
        // NED to FLU (inverse of above)
        R << 1,  0,  0,   // X_flu = X_ned (Forward=North)
             0, -1,  0,   // Y_flu = -Y_ned (Left=-East)
             0,  0, -1;   // Z_flu = -Z_ned (Up=-Down)
    }
    else if (from_frame == "ENU" && to_frame == "FRD") {
        // ENU to FRD: chain ENU->NED->FRD
        Eigen::Matrix3d R_enu_to_ned;
        R_enu_to_ned << 0,  1,  0,
                        1,  0,  0,
                        0,  0, -1;
        Eigen::Matrix3d R_ned_to_frd;
        R_ned_to_frd << 1,  0,  0,
                        0,  1,  0,
                        0,  0,  1;  // NED and FRD are same for world frame
        R = R_ned_to_frd * R_enu_to_ned;
    }
    else if (from_frame == "FRD" && to_frame == "ENU") {
        // FRD to ENU (inverse)
        Eigen::Matrix3d R_frd_to_enu = getTransformationMatrix("ENU", "FRD").transpose();
        R = R_frd_to_enu;
    }
    else if (from_frame == "ENU" && to_frame == "FLU") {
        // ENU to FLU: chain ENU->NED->FLU
        Eigen::Matrix3d R_enu_to_ned = getTransformationMatrix("ENU", "NED");
        Eigen::Matrix3d R_ned_to_flu = getTransformationMatrix("NED", "FLU");
        R = R_ned_to_flu * R_enu_to_ned;
    }
    else if (from_frame == "FLU" && to_frame == "ENU") {
        // FLU to ENU (inverse)
        R = getTransformationMatrix("ENU", "FLU").transpose();
    }
    else {
        RCLCPP_WARN(rclcpp::get_logger("vicon_px4_bridge"),
                    "Unknown frame pair: %s -> %s, using identity",
                    from_frame.c_str(), to_frame.c_str());
    }
    
    return R;
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
    // POSITION TRANSFORMATION
    // Transform position using world frame transformation only
    // p_out = R_world * p_in
    Eigen::Vector3d pos_in(pose_in.position.x, pose_in.position.y, pose_in.position.z);
    Eigen::Vector3d pos_out = R_world_transform_ * pos_in;
    
    pose_out.position.x = pos_out.x();
    pose_out.position.y = pos_out.y();
    pose_out.position.z = pos_out.z();
    
    // ORIENTATION TRANSFORMATION
    // Input quaternion: q_in represents rotation from input_world_frame to input_body_frame
    // Output quaternion: q_out represents rotation from output_world_frame to output_body_frame
    //
    // Transformation formula:
    // q_out = q_world_transform * q_in * q_body_transform
    //         └──────┬──────┘         └──────┬──────┘
    //         Left multiply:         Right multiply:
    //         World frame            Body frame
    //         transformation         transformation
    //
    // This is equivalent to: R_out = R_world * R_in * R_body
    
    Eigen::Quaterniond q_in(pose_in.orientation.w, 
                           pose_in.orientation.x,
                           pose_in.orientation.y, 
                           pose_in.orientation.z);
    
    // Apply double transformation: world frame change AND body frame change
    Eigen::Quaterniond q_out = q_world_transform_ * q_in * q_body_transform_;
    q_out.normalize();  // Ensure unit quaternion
    
    pose_out.orientation.w = q_out.w();
    pose_out.orientation.x = q_out.x();
    pose_out.orientation.y = q_out.y();
    pose_out.orientation.z = q_out.z();
}

void ViconPX4Bridge::convertFrame(const geometry_msgs::msg::Transform& transform_in,
                                   geometry_msgs::msg::Transform& transform_out)
{
    // TRANSLATION TRANSFORMATION
    // Transform translation using world frame transformation
    Eigen::Vector3d trans_in(transform_in.translation.x, 
                            transform_in.translation.y, 
                            transform_in.translation.z);
    Eigen::Vector3d trans_out = R_world_transform_ * trans_in;
    
    transform_out.translation.x = trans_out.x();
    transform_out.translation.y = trans_out.y();
    transform_out.translation.z = trans_out.z();
    
    // ROTATION TRANSFORMATION
    // Apply double transformation similar to pose conversion
    Eigen::Quaterniond q_in(transform_in.rotation.w, 
                           transform_in.rotation.x,
                           transform_in.rotation.y, 
                           transform_in.rotation.z);
    
    // Double transformation: q_out = q_world_transform * q_in * q_body_transform
    Eigen::Quaterniond q_out = q_world_transform_ * q_in * q_body_transform_;
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
    
    // Position in NED frame
    odom_msg.position[0] = static_cast<float>(pose_output.position.x);
    odom_msg.position[1] = static_cast<float>(pose_output.position.y);
    odom_msg.position[2] = static_cast<float>(pose_output.position.z);
    
    // Orientation quaternion (w, x, y, z in PX4)
    // This represents rotation from NED to FRD (body frame)
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