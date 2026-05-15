#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Bool

def yaw_from_quat(x, y, z, w):
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)

class Nav2Point(Node):
    def __init__(self):
        super().__init__('nav2point')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('pos_kp', 0.8)
        self.declare_parameter('yaw_kp', 1.5)
        self.declare_parameter('waypoint_tolerance', 0.20)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('joy_topic', '/g1pilot/auto_joy')
        self.declare_parameter('path_topic', '/g1pilot/path')
        self.declare_parameter('auto_enable_topic', '/g1pilot/auto_enable')
        self.declare_parameter('vx_limit', 0.6)
        self.declare_parameter('vy_limit', 0.6)
        self.declare_parameter('wz_limit', 0.5)
        
        self.rate = self.get_parameter('publish_rate').value
        self.pos_kp = self.get_parameter('pos_kp').value
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.wp_tol = self.get_parameter('waypoint_tolerance').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.frame_id = self.get_parameter('frame_id').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.vx_lim = self.get_parameter('vx_limit').value
        self.vy_lim = self.get_parameter('vy_limit').value
        self.wz_lim = self.get_parameter('wz_limit').value
        self.auto_enable_topic = self.get_parameter('auto_enable_topic').value

        qos = QoSProfile(depth=10)
        self.sub_odom = self.create_subscription(Odometry, '/lidar_odometry/pose_fixed', self.cb_odom, qos)
        self.sub_auto_enable = self.create_subscription(Bool, self.auto_enable_topic, self.cb_auto_enable, qos)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.cb_path, qos)
        self.pub_joy = self.create_publisher(Joy, self.joy_topic, qos)
        self.pub_wp_marker = self.create_publisher(Marker, '/g1pilot/waypoint_marker', qos)
        self.pub_goal_marker = self.create_publisher(Marker, '/g1pilot/goal_marker', qos)
        self.timer = self.create_timer(1.0 / self.rate, self.loop)

        self.have_pose = False
        self.auto_enabled = False
        self.path = []
        self.path_frame = self.frame_id
        self.idx = 0
        self.x = self.y = self.yaw = 0.0

        self.logged_no_pose = False
        self.logged_no_path = False
        self.logged_end_path = False

    def cb_odom(self, msg: Odometry):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        self.yaw = yaw_from_quat(qx, qy, qz, qw)
        self.have_pose = True
        self.logged_no_pose = False

    def cb_auto_enable(self, msg: Bool):
        self.auto_enabled = msg.data
        if not self.auto_enabled:
            self.path = []
            self.idx = 0
            self.get_logger().info('Auto navigation disabled — path cleared.')

    def cb_path(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_frame = msg.header.frame_id if msg.header.frame_id else self.frame_id
        self.idx = 0
        self.logged_no_path = False
        self.logged_end_path = False
        if self.path:
            self.publish_goal_marker(self.path[-1][0], self.path[-1][1])

    def publish_goal_marker(self, gx, gy):
        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'g1pilot_goal'
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = gx
        m.pose.position.y = gy
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.9
        self.pub_goal_marker.publish(m)

    def publish_wp_marker(self, wx, wy):
        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'g1pilot_wp'
        m.id = 2
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = wx
        m.pose.position.y = wy
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.10
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.9
        self.pub_wp_marker.publish(m)

    def loop(self):
        try:
            if (len(self.path) == 0):
                if not self.logged_no_path:
                    self.get_logger().warn('No path available.')
                    self.logged_no_path = True
                return
            
            if not self.have_pose and self.auto_enabled:
                if not self.logged_no_pose:
                    self.get_logger().warn('No pose available.')
                    self.logged_no_pose = True
                return

            if (not self.path or len(self.path) == 0) and self.auto_enabled:
                if not self.logged_no_path:
                    self.get_logger().warn('No path available.')
                    self.logged_no_path = True
                return

            if self.idx >= len(self.path) and self.auto_enabled:
                if not self.logged_end_path:
                    self.get_logger().warn('Reached the end of the path.')
                    self.logged_end_path = True
                return

            self.logged_no_pose = False
            self.logged_no_path = False
            self.logged_end_path = False

            wx, wy = self.path[self.idx]
            dx = wx - self.x
            dy = wy - self.y
            dist_wp = math.hypot(dx, dy)

            if self.idx < len(self.path) - 1 and dist_wp <= self.wp_tol:
                self.idx += 1
                wx, wy = self.path[self.idx]
                dx = wx - self.x
                dy = wy - self.y
                dist_wp = math.hypot(dx, dy)

            self.publish_wp_marker(wx, wy)

            dist_goal = math.hypot(self.path[-1][0] - self.x, self.path[-1][1] - self.y)

            joy = Joy()
            joy.header.stamp = self.get_clock().now().to_msg()
            axes = [0.0] * 8
            buttons = [0] * 14

            if dist_goal <= self.goal_tol:
                joy.axes = axes
                joy.buttons = buttons
                self.pub_joy.publish(joy)
                self.path = []
                return

            desired_yaw = math.atan2(dy, dx)
            yaw_err = desired_yaw - self.yaw
            while yaw_err > math.pi:
                yaw_err -= 2 * math.pi
            while yaw_err < -math.pi:
                yaw_err += 2 * math.pi

            vx_w = max(-self.vx_lim, min(self.vx_lim, self.pos_kp * dx))
            vy_w = max(-self.vy_lim, min(self.vy_lim, self.pos_kp * dy))

            c = math.cos(-self.yaw)
            s = math.sin(-self.yaw)
            vx_b = c * vx_w - s * vy_w
            vy_b = s * vx_w + c * vy_w

            wz = max(-self.wz_lim, min(self.wz_lim, self.yaw_kp * yaw_err))

            ax1 = max(-1.0, min(1.0, -vx_b / self.vx_lim))
            ax0 = max(-1.0, min(1.0, -vy_b / self.vy_lim))
            ax3 = max(-1.0, min(1.0, -wz / self.wz_lim))

            axes[1] = ax1
            axes[0] = ax0
            axes[2] = ax3
            buttons[7] = 1

            joy.axes = axes
            joy.buttons = buttons
            self.pub_joy.publish(joy)

        except Exception as e:
            self.get_logger().error(f'Error in loop: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Point()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
