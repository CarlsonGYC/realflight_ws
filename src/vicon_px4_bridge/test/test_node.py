#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import math


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy  
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z


class ViconPosePublisher(Node):
    def __init__(self):
        super().__init__('vicon_pose_publisher')
        self.declare_parameter('topic_name', '/vicon/drone/pose')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(PoseStamped, topic_name, 10)
        self.get_logger().info(f'Publishing PoseStamped to: {topic_name}')

        # 40 Hz
        timer_period = 0.025
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        msg = PoseStamped()

        # header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vicon_world'   
        msg.pose.position.x = 1.0
        msg.pose.position.y = 0.5 * math.sin(self.t)
        msg.pose.position.z = 0.5

        # fixed yaw
        # yaw -89 deg, roll=0, pitch=0
        roll = 0.0
        pitch = 0.0
        yaw = math.radians(-89.0)
        qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
        msg.pose.orientation.w = qw
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Publish Pose: pos=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}), "
            f"quat=({qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f})"
        )

        self.t += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = ViconPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
