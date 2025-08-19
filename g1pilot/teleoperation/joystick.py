#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import evdev
import threading

class DirectJoyPublisher(Node):
    def __init__(self):
        super().__init__('direct_joy_publisher')

        self.publisher = self.create_publisher(Joy, '/g1pilot/joy_manual', 10)
        self.auto_pub  = self.create_publisher(Bool, '/g1pilot/auto_enable', 10)

        self.declare_parameter("publish_rate", 50.0)
        self.rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.device = self.find_joystick()
        if self.device is None:
            self.get_logger().error('No joystick found!')
            return

        self.get_logger().info(f'Using joystick: {self.device.name} ({self.device.path})')

        self.axes = []
        self.buttons = []
        self.lock = threading.Lock()

        # Estado de autonomía y memoria del botón Triángulo
        self.auto_enabled = False
        self.triangle_prev = 0  # 0 -> no pulsado, 1 -> pulsado

        self.init_controls()

        self.thread = threading.Thread(target=self.read_joystick, daemon=True)
        self.thread.start()

        self.create_timer(1.0 / self.rate, self.publish_joy)

    def find_joystick(self):
        for path in evdev.list_devices():
            dev = evdev.InputDevice(path)
            if dev.name == 'Wireless Controller':  # DualShock 4
                return dev
        return None

    def init_controls(self):
        abs_info = self.device.capabilities().get(evdev.ecodes.EV_ABS, [])
        key_info = self.device.capabilities().get(evdev.ecodes.EV_KEY, [])

        self.axes = [0.0] * len(abs_info)
        self.buttons = [0] * len(key_info)

        self.axis_map = {code: idx for idx, (code, _) in enumerate(abs_info)}
        self.button_map = {code: idx for idx, code in enumerate(key_info)}

    def read_joystick(self):
        BTN_TRIANGLE = evdev.ecodes.BTN_NORTH  # Triángulo (PS) / Y (Xbox)
        for event in self.device.read_loop():
            with self.lock:
                if event.type == evdev.ecodes.EV_ABS and event.code in self.axis_map:
                    idx = self.axis_map[event.code]
                    absinfo = self.device.absinfo(event.code)
                    val = event.value
                    if absinfo.max != absinfo.min:
                        val = (2 * (val - absinfo.min) / (absinfo.max - absinfo.min)) - 1
                    self.axes[idx] = float(val)

                elif event.type == evdev.ecodes.EV_KEY and event.code in self.button_map:
                    idx = self.button_map[event.code]
                    self.buttons[idx] = int(event.value)

                    # --- Toggle autonomía en flanco de subida del Triángulo ---
                    if event.code == BTN_TRIANGLE:
                        if self.triangle_prev == 0 and event.value == 1:
                            self.auto_enabled = not self.auto_enabled
                            self.auto_pub.publish(Bool(data=self.auto_enabled))
                        self.triangle_prev = event.value

    def publish_joy(self):
        with self.lock:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = self.axes.copy()
            msg.buttons = self.buttons.copy()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DirectJoyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
