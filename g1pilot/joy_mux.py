#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from copy import deepcopy
import time

class JoyMux(Node):
    def __init__(self):
        super().__init__('joy_mux')
        self.declare_parameter('manual_topic', '/g1pilot/joy_manual')
        self.declare_parameter('auto_topic', '/g1pilot/auto_joy')
        self.declare_parameter('out_topic', '/g1pilot/joy')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('auto_enable_topic', '/g1pilot/auto_enable')
        self.manual_topic = self.get_parameter('manual_topic').value
        self.auto_topic = self.get_parameter('auto_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.rate = self.get_parameter('publish_rate').value
        self.auto_enable_topic = self.get_parameter('auto_enable_topic').value
        qos = QoSProfile(depth=10)
        self.sub_man = self.create_subscription(Joy, self.manual_topic, self.cb_manual, qos)
        self.sub_auto = self.create_subscription(Joy, self.auto_topic, self.cb_auto, qos)
        self.sub_en = self.create_subscription(Bool, self.auto_enable_topic, self.cb_enable, qos)
        self.pub = self.create_publisher(Joy, self.out_topic, qos)
        self.timer = self.create_timer(1.0/self.rate, self.loop)
        self.auto_enabled = False
        self.last_manual = None
        self.last_auto = None
        self.t_last_manual = 0.0
        self.t_last_auto = 0.0

    def cb_manual(self, msg: Joy):
        self.last_manual = msg
        self.t_last_manual = time.time()
        if len(msg.buttons) > 4 and msg.buttons[4] == 1:
            self.auto_enabled = False
            self.pub.publish(msg)

    def cb_auto(self, msg: Joy):
        self.last_auto = msg
        self.t_last_auto = time.time()

    def cb_enable(self, msg: Bool):
        self.auto_enabled = bool(msg.data)

    def loop(self):
        now = time.time()
        out = None
        if self.last_manual and (not self.auto_enabled or now - self.t_last_manual < 0.05):
            out = deepcopy(self.last_manual)
        elif self.auto_enabled and self.last_auto:
            out = deepcopy(self.last_auto)
        elif self.last_manual:
            out = deepcopy(self.last_manual)
        if out:
            self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = JoyMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
