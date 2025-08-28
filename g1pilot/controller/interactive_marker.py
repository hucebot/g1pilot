#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import functools
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class InteractiveMarkerEFF(Node):
    def __init__(self):
        super().__init__('multi_ee_goal_markers')

        # --- Parameters (all simple) ---
        self.declare_parameter('fixed_frame', 'pelvis')
        self.declare_parameter('spawn_rate_hz', 5.0)

        # Right hand params
        self.declare_parameter('right_tf_frame', 'right_hand_point_contact')
        self.declare_parameter('right_topic', '/g1pilot/right_hand_goal')
        self.declare_parameter('right_scale', 0.05)

        # Left hand params
        self.declare_parameter('left_tf_frame', 'left_hand_point_contact')
        self.declare_parameter('left_topic', '/g1pilot/left_hand_goal')
        self.declare_parameter('left_scale', 0.05)

        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.spawn_dt = 1.0 / float(self.get_parameter('spawn_rate_hz').value)

        self.right_tf = self.get_parameter('right_tf_frame').get_parameter_value().string_value
        self.right_topic = self.get_parameter('right_topic').get_parameter_value().string_value
        self.right_scale = float(self.get_parameter('right_scale').value)

        self.left_tf = self.get_parameter('left_tf_frame').get_parameter_value().string_value
        self.left_topic = self.get_parameter('left_topic').get_parameter_value().string_value
        self.left_scale = float(self.get_parameter('left_scale').value)

        # --- Infra ---
        self.server = InteractiveMarkerServer(self, "g1_ee_goal_markers")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.qos = QoSProfile(depth=10)

        self.ee_publishers = {
            "right": self.create_publisher(PoseStamped, self.right_topic, self.qos),
            "left": self.create_publisher(PoseStamped, self.left_topic, self.qos),
        }

        self.marker_spawned = {"right": False, "left": False}
        self.timer = self.create_timer(self.spawn_dt, self._try_spawn_missing)

    # ---------- Spawn ----------
    def _try_spawn_missing(self):
        if not self.marker_spawned["right"]:
            self._try_spawn_one("right", self.right_tf, self.right_scale)

        if not self.marker_spawned["left"]:
            self._try_spawn_one("left", self.left_tf, self.left_scale)

    def _try_spawn_one(self, side, tf_frame, scale):
        try:
            trans = self.tf_buffer.lookup_transform(self.fixed_frame, tf_frame, rclpy.time.Time())
            self._spawn_marker(side, tf_frame, scale, trans)
            self.marker_spawned[side] = True
            self.get_logger().info(f"Spawned {side} hand marker from TF '{tf_frame}' in '{self.fixed_frame}'")
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info(f"Waiting for TF: {self.fixed_frame} -> {tf_frame}")

    def _spawn_marker(self, side, tf_frame, cube_size, trans):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.fixed_frame
        int_marker.name = f"{side}_hand_goal"
        int_marker.description = f"{side.title()} Hand Goal"
        int_marker.scale = 0.2

        int_marker.pose.position.x = trans.transform.translation.x
        int_marker.pose.position.y = trans.transform.translation.y
        int_marker.pose.position.z = trans.transform.translation.z
        int_marker.pose.orientation = trans.transform.rotation

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = cube_size
        box_marker.scale.y = cube_size
        box_marker.scale.z = cube_size
        if side == "right":
            box_marker.color.r, box_marker.color.g, box_marker.color.b, box_marker.color.a = (0.2, 0.8, 0.2, 1.0)
        else:
            box_marker.color.r, box_marker.color.g, box_marker.color.b, box_marker.color.a = (0.2, 0.2, 0.8, 1.0)

        always = InteractiveMarkerControl()
        always.always_visible = True
        always.markers.append(box_marker)
        int_marker.controls.append(always)

        self._add_6dof_controls(int_marker)

        self.server.insert(int_marker)
        cb = functools.partial(self._feedback_cb, ee_name=side)
        self.server.setCallback(int_marker.name, cb)
        self.server.applyChanges()

    def _add_6dof_controls(self, int_marker: InteractiveMarker):
        axes = {'x': (1.0, 0.0, 0.0, 1.0),
                'y': (0.0, 1.0, 0.0, 1.0),
                'z': (0.0, 0.0, 1.0, 1.0)}
        for axis, (ox, oy, oz, ow) in axes.items():
            for mode in (InteractiveMarkerControl.ROTATE_AXIS, InteractiveMarkerControl.MOVE_AXIS):
                c = InteractiveMarkerControl()
                c.orientation.x = float(ox)
                c.orientation.y = float(oy)
                c.orientation.z = float(oz)
                c.orientation.w = float(ow)
                c.name = f"{'ROT' if mode==InteractiveMarkerControl.ROTATE_AXIS else 'MOV'}_{axis}"
                c.interaction_mode = mode
                int_marker.controls.append(c)

    # ---------- Feedback ----------
    def _feedback_cb(self, feedback, ee_name: str):
        pub = self.ee_publishers.get(ee_name)
        if pub is None:
            return
        pose = PoseStamped()
        pose.header = feedback.header
        pose.header.frame_id = self.fixed_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = feedback.pose
        pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerEFF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
