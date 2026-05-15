#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt6.QtWidgets import (
    QApplication, QWidget, QGridLayout, QPushButton, QVBoxLayout
)
from PyQt6.QtCore import QTimer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from g1pilot.utils.window_style import DarkStyle
from geometry_msgs.msg import PointStamped


class StreamDeck(Node):
    def __init__(self):
        super().__init__('stream_deck')

        self.pub_start = self.create_publisher(Bool, '/g1pilot/start', 10)
        self.pub_start_balancing = self.create_publisher(Bool, '/g1pilot/start_balancing', 10)
        self.pub_arms_enabled = self.create_publisher(Bool, '/g1pilot/arms/enabled', 10)
        self.pub_arms_home = self.create_publisher(Bool, '/g1pilot/arms/home', 10)
        self.pub_left_hand = self.create_publisher(PointStamped, '/left_hand/dx3/action', 10)
        self.pub_right_hand = self.create_publisher(PointStamped, '/right_hand/dx3/action', 10)
        self.pub_emergency_stop = self.create_publisher(Bool, '/g1pilot/emergency_stop', 10)
        self.start_opensot_pub = self.create_publisher(Bool, '/g1pilot/start_opensot', 10)
        self.pub_auto_enable = self.create_publisher(Bool, '/g1pilot/auto_enable', 10)

    def publish_bool(self, pub, value: bool):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def publish_point(self, pub, point: PointStamped):
        pub.publish(point)

class ButtonGUI(QWidget):
    """PyQt6 GUI for ROS2 StreamDeck with timed and emergency behaviors."""

    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.button_states = {}
        self.hand_pairs = {
            "left": {"open": (2, 0), "close": (2, 1)},
            "right": {"open": (3, 0), "close": (3, 1)},
        }

        self.setWindowTitle("G1PILOT - STREAMDECK")
        self.init_ui()
        self.apply_style()

    def init_ui(self):
        main_layout = QVBoxLayout()
        grid = QGridLayout()
        grid.setSpacing(10)
        rows, cols = 5, 5
        self.buttons = {}

        button_actions = {
            (0, 0): ("START", lambda: self.flash_button((0, 0), self.node.pub_start)),
            (0, 1): ("START\nBALANCING", lambda: self.flash_button((0, 1), self.node.pub_start_balancing)),
            (0, 4): ("OPENSOT", lambda: self.toggle_button((0, 4), self.node.start_opensot_pub)),

            (1, 0): ("AUTO\nNAV", lambda: self.toggle_button((1, 0), self.node.pub_auto_enable)),

            (2, 0): ("OPEN\nLEFT\nHAND", lambda: self.toggle_hand("left", "open", self.node.pub_left_hand)),
            (2, 1): ("CLOSE\nLEFT\nHAND", lambda: self.toggle_hand("left", "close", self.node.pub_left_hand)),
            (3, 0): ("OPEN\nRIGHT\nHAND", lambda: self.toggle_hand("right", "open", self.node.pub_right_hand)),
            (3, 1): ("CLOSE\nRIGHT\nHAND", lambda: self.toggle_hand("right", "close", self.node.pub_right_hand)),

            (4, 4): ("EMERGENCY\nSTOP", self.emergency_stop),
        }

        for r in range(rows):
            for c in range(cols):
                btn = QPushButton()
                btn.setMinimumSize(120, 80)

                action = button_actions.get((r, c))
                if action is None:
                    btn.setEnabled(False)
                    btn.setFlat(True)
                    btn.setStyleSheet("""
                        QPushButton {
                            background-color: #1e1e1e;
                            border: 1px solid #333;
                            border-radius: 10px;
                        }
                    """)
                else:
                    label, func = action
                    btn.setText(label)
                    btn.clicked.connect(func)
                    if (r, c) == (4, 4):
                        btn.setStyleSheet("""
                            QPushButton {
                                background-color: #b00000;
                                color: white;
                                font-weight: bold;
                                border: 1px solid #ff4444;
                                border-radius: 10px;
                            }
                            QPushButton:hover {
                                background-color: #ff0000;
                                border: 1px solid #ff6666;
                            }
                        """)

                grid.addWidget(btn, r, c)
                self.buttons[(r, c)] = btn
                self.button_states[(r, c)] = False

        main_layout.addLayout(grid)
        self.setLayout(main_layout)

    def apply_style(self):
        self.setStyleSheet("""
            QPushButton {
                background-color: #2d2d2d;
                color: #ffffff;
                font-size: 15px;
                font-weight: 600;
                border: 1px solid #444;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover:enabled {
                background-color: #3c3c3c;
                border: 1px solid #66b3ff;
            }
            QPushButton:pressed {
                background-color: #1f5fa1;
                border: 1px solid #80c4ff;
            }
            QPushButton:disabled {
                color: #555;
                background-color: #1e1e1e;
                border: 1px solid #2a2a2a;
            }
            QWidget {
                background-color: #111;
            }
        """)

    def set_button_active(self, pos, active=True):
        btn = self.buttons[pos]
        if active:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-weight: bold;
                    border: 1px solid #80ff80;
                    border-radius: 10px;
                }
            """)
        else:
            btn.setStyleSheet("")
            self.apply_style()

        self.button_states[pos] = active

    def flash_button(self, pos, pub, duration=1000):
        """Temporarily activates button for <duration> ms then resets."""
        self.set_button_active(pos, True)
        self.node.publish_bool(pub, True)

        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(lambda: self.deactivate_button(pos, pub))
        timer.start(duration)

    def deactivate_button(self, pos, pub):
        self.set_button_active(pos, False)
        self.node.publish_bool(pub, False)

    def toggle_button(self, pos, pub):
        new_state = not self.button_states[pos]
        self.set_button_active(pos, new_state)
        self.node.publish_bool(pub, new_state)

    def toggle_hand(self, hand_side, action, pub):
        hand_pair = self.hand_pairs[hand_side]
        this_pos = hand_pair[action]
        other_pos = hand_pair["close" if action == "open" else "open"]

        self.set_button_active(this_pos, True)
        self.set_button_active(other_pos, False)
        if action == "open":
            point = PointStamped()
            point.point.x = 0.0
        else:
            point = PointStamped()
            point.point.x = 1.0
        self.node.publish_point(pub, point)

    def emergency_stop(self):
        """Turns all buttons OFF and publishes False to all Bool topics."""

        self.node.publish_bool(self.node.pub_start, False)
        self.node.publish_bool(self.node.pub_start_balancing, False)
        self.node.publish_bool(self.node.pub_arms_enabled, False)
        self.node.publish_bool(self.node.pub_arms_home, False)
        self.node.publish_bool(self.node.pub_auto_enable, False)
        self.node.publish_bool(self.node.pub_emergency_stop, True)

        for pos in self.buttons:
            if pos != (4, 4):
                self.set_button_active(pos, False)

        btn = self.buttons[(4, 4)]
        btn.setStyleSheet("""
            QPushButton {
                background-color: #ff0000;
                color: white;
                font-weight: bold;
                border: 2px solid #ff6666;
                border-radius: 10px;
            }
        """)


def main():
    rclpy.init()
    node = StreamDeck()

    app = QApplication(sys.argv)
    DarkStyle(app)
    gui = ButtonGUI(node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()
    app.quit()


if __name__ == '__main__':
    main()
