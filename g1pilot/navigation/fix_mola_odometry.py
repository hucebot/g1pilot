#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return (x, y, z, w)


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (0.0, 0.0, 0.0, 1.0) if n == 0.0 else (x / n, y / n, z / n, w / n)

class FixMolaOdometry(Node):
    def __init__(self):
        super().__init__('fix_mola_odometry')

        self.declare_parameter('in_topic', '/lidar_odometry/pose')
        self.declare_parameter('out_topic', '/lidar_odometry/pose_fixed')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'pelvis')
        self.declare_parameter('normalize_quaternion', True)
        self.declare_parameter('publish_odometry', True)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.normalize_quat = self.get_parameter('normalize_quaternion').value
        self.publish_odometry = self.get_parameter('publish_odometry').value

        q_fix = euler_to_quat(0.0, 0.0, 0.0)
        self.q_prefix = q_fix

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, in_topic, self.cb_odom, 10)
        self.pub = self.create_publisher(Odometry, out_topic, 10) if self.publish_odometry else None

        self.get_logger().info(f"Listening to MOLA odometry on: {in_topic}")
        self.get_logger().info(f"Publishing TF: {self.map_frame} → {self.base_frame}")
        if self.publish_odometry:
            self.get_logger().info(f"Also publishing corrected odometry on: {out_topic}")
        self.get_logger().info("Applied fixed orientation correction: Rz(π) (180° yaw)")

    def cb_odom(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.base_frame

        px, py, pz = msg.pose.pose.position.x, -msg.pose.pose.position.y, msg.pose.pose.position.z
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = pz

        q_in = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        q_out = quat_multiply(self.q_prefix, q_in)
        if self.normalize_quat:
            q_out = quat_normalize(q_out)
        out.pose.pose.orientation = Quaternion(x=q_out[0], y=q_out[1], z=q_out[2], w=q_out[3])

        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist

        if self.pub:
            self.pub.publish(out)

        t = TransformStamped()
        t.header = out.header
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz
        t.transform.rotation = out.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixMolaOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
