#!/usr/bin/env python3
"""
px4_world_tf_hitl.py
-------------------------------------------------
* One-shot   /drone_i_init_pos  -> store initial ENU positions
* Compute a shared origin offset (default: swarm centroid -> world origin)
* VehicleOdometry (NED/FRD) -> Pose (ENU/FLU)
* Publish    /simulation/position_drone_i  @ 250 Hz
* Optional   /fmu/in/vehicle_visual_odometry (simulate Vicon)
-------------------------------------------------
"""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from px4_msgs.msg import VehicleOdometry
import tf2_ros
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
)
from px4_offboard.get_data import DataLoader


def sensor_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def ned_to_enu(pos_ned):
    return pos_ned[1], pos_ned[0], -pos_ned[2]


def enu_to_ned(pos_enu):
    return pos_enu[1], pos_enu[0], -pos_enu[2]


def quat_ned_frd_to_enu_flu(q_wxyz):
    def qmul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    SQRT2_INV = 0.70710678118
    q_n2e = (0.0, SQRT2_INV, SQRT2_INV, 0.0)
    q_frd2flu_inv = (0.0, -1.0, 0.0, 0.0)

    q_tmp = qmul(q_n2e, q_wxyz)
    q_enu = qmul(q_tmp, q_frd2flu_inv)

    w, x, y, z = q_enu
    return x, y, z, w


def quat_enu_flu_to_ned_frd(q_xyzw):
    def qmul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    SQRT2_INV = 0.70710678118
    q_n2e = (0.0, SQRT2_INV, SQRT2_INV, 0.0)
    q_n2e_inv = (q_n2e[0], -q_n2e[1], -q_n2e[2], -q_n2e[3])
    q_frd2flu = (0.0, 1.0, 0.0, 0.0)

    x, y, z, w = q_xyzw
    q_enu = (w, x, y, z)

    q_tmp = qmul(q_n2e_inv, q_enu)
    q_ned = qmul(q_tmp, q_frd2flu)
    return q_ned


class PX4WorldTFHitl(Node):
    def __init__(self):
        super().__init__('px4_world_tf_hitl')

        # self.drones = DataLoader().num_drones
        self.drones = 3
        self.namespaces = ['', '/px4_1', '/px4_2', '/px4_3', '/px4_4', '/px4_5', '/px4_6']
        self.publish_rate_hz = 250.0
        self.origin_mode = self.declare_parameter('origin_mode', 'centroid').value
        self.publish_visual_odom = self.declare_parameter('publish_visual_odom', False).value

        self.init_tf = [None] * self.drones
        self.static_sent = [False] * self.drones
        self.origin_ready = False
        self.origin_offset = (0.0, 0.0, 0.0)

        self.last_pose = [None] * self.drones
        self.last_vis_pos_ned = [None] * self.drones
        self.last_vis_time = [None] * self.drones

        self.static_bc = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_bc = tf2_ros.TransformBroadcaster(self)

        self.sim_pub = [
            self.create_publisher(PoseStamped,
                                  f'/simulation/position_drone_{i+1}', 10)
            for i in range(self.drones)
        ]
        self.visual_pub = [
            self.create_publisher(VehicleOdometry,
                                  (ns if ns else '') + '/fmu/in/vehicle_visual_odometry', 10)
            for ns in self.namespaces[:self.drones]
        ]

        for i, ns in enumerate(self.namespaces[:self.drones]):
            self.create_subscription(
                TransformStamped, f'/drone_{i}_init_pos',
                lambda msg, i=i: self._store_init(msg, i), 10)

            self.create_subscription(
                VehicleOdometry, f'{ns}/fmu/out/vehicle_odometry',
                lambda msg, i=i: self._handle_odom(msg, i), sensor_qos())

        self.create_timer(1.0 / self.publish_rate_hz, self._publish_buffer)

    def _store_init(self, msg: TransformStamped, idx: int):
        if self.init_tf[idx] is not None:
            return
        self.init_tf[idx] = deepcopy(msg)
        self._try_compute_origin()
        self._broadcast_static_for_idx(idx)

    def _try_compute_origin(self):
        if self.origin_ready:
            return
        mode = str(self.origin_mode).lower()
        if mode == 'per_drone':
            self.origin_offset = (0.0, 0.0, 0.0)
            self.origin_ready = True
            return
        if mode == 'drone0':
            if self.init_tf[0] is None:
                return
            tr = self.init_tf[0].transform.translation
            self.origin_offset = (-tr.x, -tr.y, -tr.z)
            self.origin_ready = True
        else:
            if any(tf is None for tf in self.init_tf):
                return
            xs = [tf.transform.translation.x for tf in self.init_tf]
            ys = [tf.transform.translation.y for tf in self.init_tf]
            zs = [tf.transform.translation.z for tf in self.init_tf]
            cx = sum(xs) / float(self.drones)
            cy = sum(ys) / float(self.drones)
            cz = sum(zs) / float(self.drones)
            self.origin_offset = (-cx, -cy, -cz)
            self.origin_ready = True

        self._broadcast_all_static()
        self.get_logger().info(
            f'Origin offset ready (mode={mode}): {self.origin_offset[0]:.3f}, '
            f'{self.origin_offset[1]:.3f}, {self.origin_offset[2]:.3f}'
        )

    def _broadcast_all_static(self):
        for idx in range(self.drones):
            self._broadcast_static_for_idx(idx)

    def _broadcast_static_for_idx(self, idx: int):
        if self.static_sent[idx] or self.init_tf[idx] is None:
            return
        if not self.origin_ready:
            return
        init_tr = self.init_tf[idx].transform.translation
        ox, oy, oz = self.origin_offset

        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'world'
        static_tf.child_frame_id = f'px4_{idx}_frame'
        static_tf.transform.rotation.x = self.init_tf[idx].transform.rotation.x
        static_tf.transform.rotation.y = self.init_tf[idx].transform.rotation.y
        static_tf.transform.rotation.z = self.init_tf[idx].transform.rotation.z
        static_tf.transform.rotation.w = self.init_tf[idx].transform.rotation.w
        static_tf.transform.translation.x = init_tr.x + ox
        static_tf.transform.translation.y = init_tr.y + oy
        static_tf.transform.translation.z = init_tr.z + oz

        self.static_bc.sendTransform(static_tf)
        self.static_sent[idx] = True

    def _handle_odom(self, msg: VehicleOdometry, idx: int):
        if self.init_tf[idx] is None or not self.origin_ready:
            self.get_logger().warn(f'Init pose not ready for drone_{idx}')
            return

        px, py, pz = ned_to_enu(msg.position)
        init_tr = self.init_tf[idx].transform.translation
        ox, oy, oz = self.origin_offset
        world_pos = (px + init_tr.x + ox,
                     py + init_tr.y + oy,
                     pz + init_tr.z + oz)

        qx, qy, qz, qw = quat_ned_frd_to_enu_flu(msg.q)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(world_pos[0])
        pose.pose.position.y = float(world_pos[1])
        pose.pose.position.z = float(world_pos[2])
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        self.last_pose[idx] = pose

    def _publish_buffer(self):
        now = self.get_clock().now().to_msg()
        now_sec = now.sec + now.nanosec * 1e-9
        for i, pose in enumerate(self.last_pose):
            if pose is None:
                continue
            pose.header.stamp = now
            self.sim_pub[i].publish(pose)

            tf_msg = TransformStamped()
            tf_msg.header = pose.header
            tf_msg.child_frame_id = f'drone_{i}_world'
            tf_msg.transform.translation.x = pose.pose.position.x
            tf_msg.transform.translation.y = pose.pose.position.y
            tf_msg.transform.translation.z = pose.pose.position.z
            tf_msg.transform.rotation = pose.pose.orientation
            self.tf_bc.sendTransform(tf_msg)

            if not self.publish_visual_odom:
                continue

            pos_enu = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            pos_ned = enu_to_ned(pos_enu)
            q_ned_wxyz = quat_enu_flu_to_ned_frd((
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ))

            dt = None
            if self.last_vis_time[i] is not None:
                dt = max(1e-5, now_sec - self.last_vis_time[i])
            self.last_vis_time[i] = now_sec

            vel_ned = [float('nan')] * 3
            if dt is not None and self.last_vis_pos_ned[i] is not None:
                vel_ned = [
                    (pos_ned[0] - self.last_vis_pos_ned[i][0]) / dt,
                    (pos_ned[1] - self.last_vis_pos_ned[i][1]) / dt,
                    (pos_ned[2] - self.last_vis_pos_ned[i][2]) / dt,
                ]
            self.last_vis_pos_ned[i] = pos_ned

            vis = VehicleOdometry()
            vis.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            vis.timestamp_sample = vis.timestamp
            vis.pose_frame = VehicleOdometry.POSE_FRAME_NED
            vis.position = [float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])]
            vis.q = [float(q_ned_wxyz[0]), float(q_ned_wxyz[1]),
                     float(q_ned_wxyz[2]), float(q_ned_wxyz[3])]
            vis.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            vis.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]
            vis.angular_velocity = [float('nan'), float('nan'), float('nan')]
            vis.position_variance = [float('nan'), float('nan'), float('nan')]
            vis.orientation_variance = [float('nan'), float('nan'), float('nan')]
            vis.velocity_variance = [float('nan'), float('nan'), float('nan')]
            vis.reset_counter = 0
            vis.quality = 100

            self.visual_pub[i].publish(vis)


def main():
    rclpy.init()
    node = PX4WorldTFHitl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
