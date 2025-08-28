#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from PyQt5 import QtWidgets
from PyQt5 import QtCore

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)

from g1pilot.utils.g1_arm_controller import G1_29_ArmController


def _rpc_get_int(client, api_id):
    try:
        code, data = client._Call(api_id, "{}")
        if code == 0 and data:
            return json.loads(data).get("data")
    except Exception:
        pass
    return None


class G1LocoClient(Node):
    def __init__(self, ui_bridge=None):
        super().__init__("loco_client")
        self.robot_stopped = False
        self.balanced = False
        self.prev_buttons = {}
        self.prev_axis_last = None
        self.control_arms = False
        self.ui_bridge = ui_bridge

        self.declare_parameter('interface', 'eth1')
        interface = self.get_parameter('interface').get_parameter_value().string_value
        self.declare_parameter('arm_controlled', 'both') # Options: 'left', 'right', 'both'
        self.arm_controlled = self.get_parameter('arm_controlled').get_parameter_value().string_value
        self.declare_parameter('enable_arm_ui', False)
        self.enable_arm_ui = self.get_parameter('enable_arm_ui').get_parameter_value().bool_value

        self.declare_parameter('ik_use_waist', False)
        self.declare_parameter('ik_alpha', 0.2)
        self.declare_parameter('ik_max_dq_step', 0.05)
        self.declare_parameter('arm_velocity_limit', 2.0)

        ik_use_waist = self.get_parameter('ik_use_waist').get_parameter_value().bool_value
        ik_alpha = float(self.get_parameter('ik_alpha').value)
        ik_max_dq_step = float(self.get_parameter('ik_max_dq_step').value)
        arm_vel_lim = float(self.get_parameter('arm_velocity_limit').value)

        ChannelFactoryInitialize(0, interface)
        self.robot = LocoClient()
        self.robot.SetTimeout(0.0001)
        self.robot.SetFsmId(4)
        self.robot.Init()
        self.robot.Damp()
        self.current_id = self.get_fsm_id()
        self.current_mode = self.get_fsm_mode()
        self.get_logger().info(f"Current FSM ID: {self.current_id}, Mode: {self.current_mode}")

        self.arm_control = G1_29_ArmController(
            ui_bridge=self.ui_bridge,
            controlled_arms=self.arm_controlled,
            show_ui=self.enable_arm_ui,
            ros_node=self,
        )
        self.arm_control._ik_use_waist = ik_use_waist
        self.arm_control._ik_alpha = ik_alpha
        self.arm_control._ik_max_dq_step = ik_max_dq_step
        self.arm_control.arm_velocity_limit = arm_vel_lim

        self.create_subscription(Bool, 'emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Bool, 'start_balancing', self.start_balancing_callback, 10)
        self.create_subscription(Joy, '/g1pilot/joy', self.joystick_callback, 10)

    def log_once_attr(self, level, msg, attr):
        if hasattr(self, attr):
            return
        setattr(self, attr, True)
        logger = self.get_logger()
        if level == "info":
            logger.info(msg)
        elif level in ("warn", "warning"):
            logger.warn(msg)
        elif level == "error":
            logger.error(msg)
        else:
            logger.info(msg)

    def clear_attr(self, attr):
        if hasattr(self, attr):
            delattr(self, attr)

    # -------- FSM --------
    def get_fsm_id(self):
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_ID)

    def get_fsm_mode(self):
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_MODE)

    def emergency_callback(self, msg: Bool):
        if msg.data:
            self.log_once_attr("warn", "EMERGENCY STOP ACTIVATED!", "_e_stop_activated_logged")
            self.robot_stopped = True
            self.balanced = False
            self.robot.Damp()
            if self.control_arms:
                self.control_arms = False
                self.arm_control.set_control_mode(False)
        else:
            self.clear_attr("_e_stop_activated_logged")

    def start_balancing_callback(self, msg: Bool):
        if msg.data and not self.balanced:
            self.log_once_attr("info", "Starting balancing procedure...", "_start_balance_req_logged")
            self.entering_balancing(max_height=0.5, step=0.02)
            self.log_once_attr("info", "Balancing procedure completed.", "_balance_completed_logged")
        elif self.balanced:
            self.log_once_attr("info", "Already balanced, no action taken.", "_already_balanced_notice_logged")

    def joystick_callback(self, msg: Joy):
        if not self.prev_buttons:
            self.prev_buttons = {i: 0 for i in range(len(msg.buttons))}
        if not self.balanced:
            self.log_once_attr("warn", "Robot is not balanced, cannot move.", "_warn_not_balanced_logged")
        else:
            self.clear_attr("_warn_not_balanced_logged")
        if self.robot_stopped:
            self.log_once_attr("warn", "Robot is stopped, cannot move.", "_warn_robot_stopped_logged")
        else:
            self.clear_attr("_warn_robot_stopped_logged")

        axis_last = msg.axes[-1] if len(msg.axes) else 0.0
        if self.prev_axis_last is None:
            self.prev_axis_last = axis_last

        # UP CHANGE FSM ID 4 (Standby)
        if axis_last == -1.0 and self.prev_axis_last != -1.0:
            self.robot.SetFsmId(4)
            self.log_once_attr("info", "Switched to FSM ID 4 (Standby)", "_switch_fsm_id_4_logged")
            self.robot_stopped = False
            self.balanced = False
        if axis_last != -1.0 and self.prev_axis_last == -1.0:
            self.clear_attr("_switch_fsm_id_4_logged")

        # X -> TOGGLE TO CONTROL THE ARMS
        if msg.buttons[0] == 1 and self.prev_buttons[0] == 0:
            self.control_arms = not self.control_arms
            self.arm_control.set_control_mode(self.control_arms)
            if self.control_arms:
                self.log_once_attr("info", "Enabling arm control mode.", "_enable_arm_control_logged")
            else:
                self.log_once_attr("info", "Disabling arm control mode.", "_disable_arm_control_logged")

        # L1 -> Emergency stop
        if msg.buttons[4] == 1 and self.prev_buttons[4] == 0:
            self.log_once_attr("warn", "Emergency stop button pressed!", "_e_stop_button_pressed_logged")
            self.robot_stopped = True
            self.balanced = False
            self.robot.Damp()
            if self.control_arms:
                self.control_arms = False
                self.arm_control.set_control_mode(False)
        if msg.buttons[4] == 0 and self.prev_buttons[4] == 1:
            self.clear_attr("_e_stop_button_pressed_logged")

        # R1 -> Balancing
        if msg.buttons[5] == 1 and self.prev_buttons[5] == 0:
            if not self.balanced:
                self.log_once_attr("info", "Starting balancing procedure...", "_start_balance_r1_logged")
                self.entering_balancing(max_height=0.5, step=0.02)
                self.log_once_attr("info", "Balancing procedure completed.", "_balance_completed_r1_logged")
            else:
                self.log_once_attr("info", "Already balanced, no action taken.", "_already_balanced_notice_r1_logged")
        if msg.buttons[5] == 0 and self.prev_buttons[5] == 1:
            self.clear_attr("_start_balance_r1_logged")
            self.clear_attr("_balance_completed_r1_logged")
            self.clear_attr("_already_balanced_notice_r1_logged")

        # R2 (hold) -> Move
        if msg.buttons[7] == 0 and not self.robot_stopped and self.balanced:
            self.robot.StopMove()
        if msg.buttons[7] == 1 and not self.robot_stopped and self.balanced:
            vx  = round(msg.axes[1] * 0.5 * -1, 2)
            vy  = round(msg.axes[0] * 0.4 * -1, 2)
            yaw = round(msg.axes[3] * 0.4 * -1, 2)
            if abs(vx) < 0.03 and abs(vy) < 0.03 and abs(yaw) < 0.03:
                self.robot.StopMove()
            else:
                self.robot.Move(vx=vx, vy=vy, vyaw=yaw, continous_move=True)

        self.prev_buttons = {i: msg.buttons[i] for i in range(len(msg.buttons))}
        self.prev_axis_last = axis_last

    # -------- Balancing --------
    def entering_balancing(self, max_height=0.5, step=0.02):
        height = 0.0
        while height < max_height and not self.robot_stopped:
            height += step
            self.robot.SetStandHeight(height)
            if self.get_fsm_mode() == 0 and height >= 0.2:
                self.log_once_attr("info", f"Reached max height: {height}", "_balance_reach_height_logged")
                break
            elif self.get_fsm_mode() != 0:
                self.log_once_attr("warn", "Problems during balancing, stopping...", "_balance_problem_stop_logged")
                break
        self.robot.BalanceStand(1)
        self.robot.SetStandHeight(height)
        self.robot.Start()
        self.balanced = True



class UiBridge(QtCore.QObject):
    runSignal = QtCore.pyqtSignal(object)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.runSignal.connect(self._run)

    @QtCore.pyqtSlot(object)
    def _run(self, fn):
        try:
            fn()
        except Exception as e:
            print(f"[UiBridge] exception in slot: {e}", flush=True)

def main(args=None):
    os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp/runtime-root")

    rclpy.init(args=args)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    ui_bridge = UiBridge()

    node = G1LocoClient(ui_bridge=ui_bridge)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        exit_code = app.exec()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=0.5)
        node.destroy_node()
        rclpy.shutdown()
    return exit_code

if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()