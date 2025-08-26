#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import time
import os

os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp/runtime-root")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from PyQt5 import QtWidgets, QtCore


class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


JOINT_NAME = {
    0: "left_hip_pitch_joint",
    1: "left_hip_roll_joint",
    2: "left_hip_yaw_joint",
    3: "left_knee_joint",
    4: "left_ankle_pitch_joint",
    5: "left_ankle_roll_joint",
    6: "right_hip_pitch_joint",
    7: "right_hip_roll_joint",
    8: "right_hip_yaw_joint",
    9: "right_knee_joint",
    10: "right_ankle_pitch_joint",
    11: "right_ankle_roll_joint",
    12: "waist_yaw_joint",
    13: "waist_roll_joint",
    14: "waist_pitch_joint",
    15: "left_shoulder_pitch_joint",
    16: "left_shoulder_roll_joint",
    17: "left_shoulder_yaw_joint",
    18: "left_elbow_joint",
    19: "left_wrist_roll_joint",
    20: "left_wrist_pitch_joint",
    21: "left_wrist_yaw_joint",
    22: "right_shoulder_pitch_joint",
    23: "right_shoulder_roll_joint",
    24: "right_shoulder_yaw_joint",
    25: "right_elbow_joint",
    26: "right_wrist_roll_joint",
    27: "right_wrist_pitch_joint",
    28: "right_wrist_yaw_joint",
}

ALL_IDX = list(JOINT_NAME.keys())

LEFT_ARM_IDX = [
    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,
]

RIGHT_ARM_IDX = [
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
]



class ArmJointPublisher(Node, QtWidgets.QWidget):
    def __init__(self):
        Node.__init__(self, "g1_arm_joint_publisher")
        QtWidgets.QWidget.__init__(self)

        self.declare_parameter("side", "right")  # "left" | "right" | "both"
        self.declare_parameter("publish_topic", "/g1pilot/arm/joint_targets")
        self.declare_parameter("ik_topic", "/ik_arm_joint_targets")
        self.declare_parameter("seed_from_joint_states", True)
        self.declare_parameter("seed_timeout_ms", 500)
        self.declare_parameter("rate_hz", 50)
        self.declare_parameter("alpha", 0.2)

        side = self.get_parameter("side").get_parameter_value().string_value.lower()
        self.publish_topic = self.get_parameter("publish_topic").get_parameter_value().string_value
        ik_topic = self.get_parameter("ik_topic").get_parameter_value().string_value
        self.seed_from_js = self.get_parameter("seed_from_joint_states").get_parameter_value().bool_value
        self.seed_timeout_ms = int(self.get_parameter("seed_timeout_ms").get_parameter_value().integer_value)
        self.rate_hz = float(self.get_parameter("rate_hz").get_parameter_value().double_value or
                             self.get_parameter("rate_hz").value)
        self.alpha = float(self.get_parameter("alpha").get_parameter_value().double_value or
                           self.get_parameter("alpha").value)

        if side == "left":
            controlled = LEFT_ARM_IDX.copy()
        elif side == "both":
            controlled = LEFT_ARM_IDX + RIGHT_ARM_IDX
        else:
            controlled = RIGHT_ARM_IDX.copy()

        self.CONTROLLED_IDX = list(controlled)

        self.setWindowTitle(f"G1 Arm Joint Publisher  —  side={side})")

        self._received_seed = False
        self._initial_seed_done = False

        self.current = [0.0] * len(ALL_IDX)
        self.targets = [0.0] * len(ALL_IDX)
        self.smoothed = [0.0] * len(ALL_IDX)

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(JointState, self.publish_topic, qos)

        self.create_subscription(JointState, ik_topic, self._on_ik_cmd, qos)

        if self.seed_from_js:
            self._seed_sub = self.create_subscription(JointState, "/joint_states", self._on_seed_js, qos)
            self._seed_timer = QtCore.QTimer(self)
            self._seed_timer.setSingleShot(True)
            self._seed_timer.timeout.connect(self._fallback_zero_seed)
            self._seed_timer.start(self.seed_timeout_ms)
        else:
            self._received_seed = True
            self._initial_seed_done = True

        layout = QtWidgets.QVBoxLayout(self)
        self.sliders = {}
        for idx in self.CONTROLLED_IDX:
            label = QtWidgets.QLabel(JOINT_NAME[idx])
            sld = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            sld.setRange(-180, 180)
            sld.setValue(0)
            sld.valueChanged.connect(lambda deg, i=idx: self._on_slider(i, deg))
            layout.addWidget(label)
            layout.addWidget(sld)
            self.sliders[idx] = sld

        home_btn = QtWidgets.QPushButton("Home Position (0°)")
        home_btn.clicked.connect(self._go_home)
        layout.addWidget(home_btn)

        estop_btn = QtWidgets.QPushButton("E-STOP (local)")
        estop_btn.setToolTip("Just sets local targets to 0. Does not publish anything special.")
        estop_btn.setMinimumHeight(64)
        estop_btn.setStyleSheet("background-color: red; color: white;")
        estop_btn.clicked.connect(self._local_estop)
        layout.addWidget(estop_btn)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        period_ms = int(max(1.0, 1000.0 / max(1e-3, self.rate_hz)))
        self.timer.start(period_ms)

    def _on_seed_js(self, msg: JointState):
        """Semilla inicial desde /joint_states (si existe)."""
        if self._initial_seed_done:
            return
        name_to_idx = {v: k for k, v in JOINT_NAME.items()}
        any_found = False
        for name, pos in zip(msg.name, msg.position):
            idx = name_to_idx.get(name)
            if idx is not None:
                self.current[idx] = float(pos)
                any_found = True

        if any_found:
            self.targets = self.current.copy()
            self.smoothed = self.current.copy()
            for idx in self.CONTROLLED_IDX:
                self._set_slider_deg(idx, math.degrees(self.targets[idx]))
            self._received_seed = True
            self._initial_seed_done = True
            if hasattr(self, "_seed_timer"):
                self._seed_timer.stop()

    def _fallback_zero_seed(self):
        if self._initial_seed_done:
            return
        self.current = [0.0] * len(ALL_IDX)
        self.targets = self.current.copy()
        self.smoothed = self.current.copy()
        for idx in self.CONTROLLED_IDX:
            self._set_slider_deg(idx, 0.0)
        self._received_seed = True
        self._initial_seed_done = True

    def _on_ik_cmd(self, msg: JointState):
        full_names = [JOINT_NAME[i] for i in ALL_IDX]
        if list(msg.name) == full_names and len(msg.position) == len(ALL_IDX):
            self.targets = list(msg.position)
            for idx in self.CONTROLLED_IDX:
                self._set_slider_deg(idx, math.degrees(self.targets[idx]))
            return

        name_to_idx = {v: k for k, v in JOINT_NAME.items()}
        for name, pos in zip(msg.name, msg.position):
            idx = name_to_idx.get(name)
            if idx is not None:
                self.targets[idx] = float(pos)
                if idx in self.CONTROLLED_IDX:
                    self._set_slider_deg(idx, math.degrees(float(pos)))

    def _on_slider(self, idx: int, deg: int):
        self.targets[idx] = math.radians(float(deg))

    def _go_home(self):
        for idx in self.CONTROLLED_IDX:
            sld = self.sliders.get(idx)
            if sld:
                sld.setValue(0)
            self.targets[idx] = 0.0

    def _local_estop(self):
        for idx in self.CONTROLLED_IDX:
            self.targets[idx] = 0.0
            self._set_slider_deg(idx, 0.0)

    def _set_slider_deg(self, idx: int, deg: float):
        val = int(round(max(-180, min(180, deg))))
        sld = self.sliders.get(idx)
        if sld is not None:
            sld.blockSignals(True)
            sld.setValue(val)
            sld.blockSignals(False)
    def _tick(self):
        if not self._received_seed:
            return
        for i in ALL_IDX:
            self.smoothed[i] += self.alpha * (self.targets[i] - self.smoothed[i])

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [JOINT_NAME[i] for i in self.CONTROLLED_IDX]
        js.position = [self.smoothed[i] for i in self.CONTROLLED_IDX]
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    node = ArmJointPublisher()
    node.show()

    spinner = QtCore.QTimer()
    spinner.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spinner.start(10)

    app.exec_()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
