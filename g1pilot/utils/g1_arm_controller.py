#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import time
import threading
import numpy as np
from enum import IntEnum
from PyQt5 import QtWidgets, QtCore

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (LowCmd_ as hg_LowCmd, LowState_ as hg_LowState)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp/runtime-root")


class MotorState:
    def __init__(self):
        self.q = None
        self.dq = None


class G1_29_JointArmIndex(IntEnum):
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kLeftWristRoll     = 19
    kLeftWristPitch    = 20
    kLeftWristyaw      = 21
    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25
    kRightWristRoll     = 26
    kRightWristPitch    = 27
    kRightWristYaw      = 28


class G1_29_JointWristIndex(IntEnum):
    # Left wrist
    kLeftWristRoll  = 19
    kLeftWristPitch = 20
    kLeftWristyaw   = 21
    # Right wrist
    kRightWristRoll  = 26
    kRightWristPitch = 27
    kRightWristYaw   = 28


class G1_29_JointWeakIndex(IntEnum):
    kLeftAnklePitch    = 4
    kRightAnklePitch   = 10
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25


class G1_29_JointIndex(IntEnum):
    # Left leg
    kLeftHipPitch   = 0
    kLeftHipRoll    = 1
    kLeftHipYaw     = 2
    kLeftKnee       = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll  = 5
    # Right leg
    kRightHipPitch   = 6
    kRightHipRoll    = 7
    kRightHipYaw     = 8
    kRightKnee       = 9
    kRightAnklePitch = 10
    kRightAnkleRoll  = 11
    # Torso
    kWaistYaw   = 12
    kWaistRoll  = 13
    kWaistPitch = 14
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kLeftWristRoll     = 19
    kLeftWristPitch    = 20
    kLeftWristyaw      = 21
    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25
    kRightWristRoll     = 26
    kRightWristPitch    = 27
    kRightWristYaw      = 28
    # not used
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34

JOINT_LIMITS_RAD = {
    # piernas
    0: (-2.5307,  2.8798),
    1: (-0.5236,  2.9671),
    2: (-2.7576,  2.7576),
    3: (-0.087267,2.8798),
    4: (-0.87267, 0.5236),
    5: (-0.2618,  0.2618),
    6: (-2.5307,  2.8798),
    7: (-2.9671,  0.5236),
    8: (-2.7576,  2.7576),
    9: (-0.087267,2.8798),
    10:(-0.87267, 0.5236),
    11:(-0.2618,  0.2618),
    # torso
    12:(-2.618,   2.618),
    13:(-0.52,    0.52),
    14:(-0.52,    0.52),
    # brazo izq
    15:(-3.0892,  2.6704),
    16:(-1.5882,  2.2515),
    17:(-2.618,   2.618),
    18:(-1.0472,  2.0944),
    19:(-1.972222054, 1.972222054),
    20:(-1.614429558, 1.614429558),
    21:(-1.614429558, 1.614429558),
    # brazo der
    22:(-3.0892,  2.6704),       # R_SHOULDER_PITCH
    23:(-2.2515,  1.5882),       # R_SHOULDER_ROLL
    24:(-2.618,   2.618),        # R_SHOULDER_YAW
    25:(-1.0472,  2.0944),       # R_ELBOW
    26:(-1.972222054, 1.972222054), # R_WRIST_ROLL
    27:(-1.614429558, 1.614429558), # R_WRIST_PITCH
    28:(-1.614429558, 1.614429558), # R_WRIST_YAW
}
RIGHT_JOINT_INDICES_LIST = [22,23,24,25,26,27,28]

def clamp_right(q7):
    out = []
    for ii, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
        lo, hi = JOINT_LIMITS_RAD[jidx]
        out.append(float(np.clip(q7[ii], lo, hi)))
    return np.array(out, dtype=float)

class DataBuffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()
    def GetData(self):
        with self.lock:
            return self.data
    def SetData(self, data):
        with self.lock:
            self.data = data


class UiBridge(QtCore.QObject):
    runSignal = QtCore.pyqtSignal(object)
    def __init__(self):
        super().__init__()
        self.runSignal.connect(self._run)
    @QtCore.pyqtSlot(object)
    def _run(self, fn):
        try:
            fn()
        except Exception as e:
            print(f"[UiBridge] Error callable: {e}", flush=True)

class ArmRightGUI(QtWidgets.QWidget):
    valuesChanged = QtCore.pyqtSignal(object)

    def __init__(self, get_initial_q_radians_callable, parent=None):
        super().__init__(parent)
        self.setWindowTitle("G1 – Right Arm Control")
        self.setMinimumWidth(480)

        self.joint_ids = RIGHT_JOINT_INDICES_LIST[:]
        self.joint_names = [
            "R Shoulder Pitch", "R Shoulder Roll", "R Shoulder Yaw",
            "R Elbow", "R Wrist Roll", "R Wrist Pitch", "R Wrist Yaw"
        ]

        layout = QtWidgets.QVBoxLayout(self)
        self.sliders = []
        self.value_labels = []

        init_q7 = get_initial_q_radians_callable() or [0.0]*7
        if len(init_q7) != 7:
            init_q7 = [0.0]*7
        init_q7 = clamp_right(init_q7)

        for i, (name, jidx) in enumerate(zip(self.joint_names, self.joint_ids)):
            row = QtWidgets.QHBoxLayout()
            lab = QtWidgets.QLabel(name); lab.setFixedWidth(160)

            lo_rad, hi_rad = JOINT_LIMITS_RAD[jidx]
            lo_deg = int(round(math.degrees(lo_rad)))
            hi_deg = int(round(math.degrees(hi_rad)))

            sld = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            sld.setMinimum(lo_deg); sld.setMaximum(hi_deg)
            sld.setSingleStep(1); sld.setPageStep(5)
            sld.setTickInterval(max(5, (hi_deg-lo_deg)//12))
            sld.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)

            deg0 = int(round(math.degrees(init_q7[i])))
            deg0 = max(lo_deg, min(hi_deg, deg0))
            sld.setValue(deg0)

            val_lab = QtWidgets.QLabel(f"{deg0:>4}°"); val_lab.setFixedWidth(60)
            sld.valueChanged.connect(lambda v, idx=i, vl=val_lab: self._on_slider(idx, v, vl))

            row.addWidget(lab); row.addWidget(sld, 1); row.addWidget(val_lab)
            layout.addLayout(row)
            self.sliders.append(sld); self.value_labels.append(val_lab)

        btns = QtWidgets.QHBoxLayout()
        btn_center = QtWidgets.QPushButton("Center (0°)")
        btn_center.clicked.connect(self._center_all)
        btns.addStretch(1); btns.addWidget(btn_center)
        layout.addLayout(btns)
        self.setLayout(layout)

    def _on_slider(self, idx, value_deg, val_label):
        val_label.setText(f"{int(value_deg):>4}°")
        vals_rad = []
        for sld, jidx in zip(self.sliders, self.joint_ids):
            lo_rad, hi_rad = JOINT_LIMITS_RAD[jidx]
            rad = math.radians(sld.value())
            vals_rad.append(float(np.clip(rad, lo_rad, hi_rad)))
        self.valuesChanged.emit(vals_rad)

    def _center_all(self):
        for sld, jidx in zip(self.sliders, self.joint_ids):
            lo_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][0])))
            hi_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][1])))
            center = 0 if lo_deg <= 0 <= hi_deg else int((lo_deg+hi_deg)/2)
            sld.setValue(center)

    def update_from_robot_pose(self, q_right_rad):
        if not q_right_rad or len(q_right_rad) != 7:
            return
        q_right_rad = clamp_right(q_right_rad)
        for i, (val, jidx) in enumerate(zip(q_right_rad, self.joint_ids)):
            lo_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][0])))
            hi_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][1])))
            deg = int(round(math.degrees(val)))
            deg = max(lo_deg, min(hi_deg, deg))
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(deg)
            self.value_labels[i].setText(f"{deg:>4}°")
            self.sliders[i].blockSignals(False)


class G1_29_ArmController:
    def __init__(self, ui_bridge: QtCore.QObject):
        self.motor_state = [MotorState() for _ in range(35)]

        self.topic_motion_cmd = 'rt/arm_sdk'
        self.topic_low_cmd    = 'rt/lowstate'

        self.q_target      = np.zeros(14)
        self.tauff_target  = np.zeros(14)
        self.kp_high = 300.0; self.kd_high = 3.0
        self.kp_low  = 80.0;  self.kd_low  = 3.0
        self.kp_wrist= 40.0;  self.kd_wrist= 1.5

        self.control_mode = False

        self.all_motor_q = None
        self.arm_velocity_limit = 20.0
        self.control_dt = 1.0 / 250.0

        self._last_cmd_q = None
        self._last_tick_time = None

        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None

        self._bridge = ui_bridge
        self._gui = None

        self.lowstate_subscriber = ChannelSubscriber(self.topic_low_cmd, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self.subscribe_thread.start()

        self.lowcmd_publisher = ChannelPublisher(self.topic_motion_cmd, hg_LowCmd)
        self.lowcmd_publisher.Init()

        print("Waiting for first lowstate message...")
        while not self.lowstate_buffer.GetData():
            time.sleep(0.1)

        print("G1_29_ArmController initialized.")
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()
        self.all_motor_q = self.get_current_motor_q()

        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            self.msg.motor_cmd[jid].q = float(self.all_motor_q[jid.value])

        self.ctrl_lock = threading.Lock()
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state, daemon=True)
        self.publish_thread.start()

    def _get_right_arm_current_q(self):
        try:
            with self.ctrl_lock:
                if hasattr(self, "q_target") and len(self.q_target) == 14:
                    return self.q_target[7:14].tolist()
        except Exception:
            pass
        cur = self.get_current_dual_arm_q()
        return cur[7:14].tolist()

    def _open_gui_if_needed(self):
        def _make_or_show():
            print("[ArmGUI] _make_or_show()", flush=True)
            if self._gui is None:
                self._gui = ArmRightGUI(self._get_right_arm_current_q)
                self._gui.valuesChanged.connect(self._on_gui_values)
            self._gui.move(100, 100)
            self._gui.show(); self._gui.raise_(); self._gui.activateWindow()

        print("[ArmGUI] scheduling GUI create/show (via bridge)", flush=True)
        self._bridge.runSignal.emit(_make_or_show)

    def _close_gui_if_needed(self):
        if self._gui is None:
            return
        print("[ArmGUI] scheduling GUI hide (via bridge)", flush=True)
        self._bridge.runSignal.emit(self._gui.hide)

    def _on_gui_values(self, radians_list):
        if len(radians_list) != 7:
            return
        with self.ctrl_lock:
            self.q_target[7:14] = np.array(radians_list, dtype=float)

    # -------------------- API externa --------------------
    def set_control_mode(self, enabled: bool):
        self.control_mode = enabled
        print(f"[ArmGUI] set_control_mode -> {enabled}", flush=True)
        if enabled:
            with self.ctrl_lock:
                self.q_target = self.get_current_dual_arm_q().copy()
                self.tauff_target = np.zeros_like(self.tauff_target)
                self._last_cmd_q = self.q_target.copy()
            self._arm_set_mode(mode=1)
            self._open_gui_if_needed()
        else:
            self._close_gui_if_needed()

    def get_mode_machine(self):
        msg = self.lowstate_buffer.GetData()
        return getattr(msg, "mode_machine", 0) if msg is not None else 0

    def get_current_dual_arm_q(self):
        msg = self.lowstate_buffer.GetData()
        return np.array([msg.motor_state[id].q for id in G1_29_JointArmIndex], dtype=float)

    def get_current_motor_q(self):
        msg = self.lowstate_buffer.GetData()
        return np.array([msg.motor_state[id].q for id in G1_29_JointIndex], dtype=float)

    def _compute_dt(self):
        now = time.time()
        if self._last_tick_time is None:
            dt = self.control_dt
        else:
            dt = max(1e-4, min(0.1, now - self._last_tick_time))
        self._last_tick_time = now
        return dt

    def clip_arm_q_target(self, target_q, velocity_limit):
        if self._last_cmd_q is None:
            self._last_cmd_q = self.get_current_dual_arm_q().copy()

        dt = self._compute_dt()
        max_step = float(velocity_limit) * dt
        delta = target_q - self._last_cmd_q
        delta = np.clip(delta, -max_step, max_step)
        return self._last_cmd_q + delta

    def _hold_non_arm_joints(self):
        arm_vals  = {m.value for m in G1_29_JointArmIndex}
        weak_vals = {m.value for m in G1_29_JointWeakIndex}
        current_all = self.get_current_motor_q()

        self.msg.mode_pr = 0
        for jid in G1_29_JointIndex:
            if jid.value in arm_vals:
                continue
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in weak_vals:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            else:
                self.msg.motor_cmd[jid].kp = self.kp_high
                self.msg.motor_cmd[jid].kd = self.kd_high
            self.msg.motor_cmd[jid].q   = float(current_all[jid.value])
            self.msg.motor_cmd[jid].dq  = 0.0
            self.msg.motor_cmd[jid].tau = 0.0

    def _arm_set_mode(self, mode:int, kp:float=0.0, kd:float=0.0):
        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = mode
            if mode == 1:
                if jid.value in wrist_vals:
                    self.msg.motor_cmd[jid].kp = self.kp_wrist
                    self.msg.motor_cmd[jid].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[jid].kp = self.kp_low
                    self.msg.motor_cmd[jid].kd = self.kd_low
            else:
                self.msg.motor_cmd[jid].kp = kp
                self.msg.motor_cmd[jid].kd = kd

    def _arm_release_once(self):
        current = self.get_current_dual_arm_q()
        for idx, jid in enumerate(G1_29_JointArmIndex):
            self.msg.motor_cmd[jid].q   = float(current[idx])
            self.msg.motor_cmd[jid].dq  = 0.0
            self.msg.motor_cmd[jid].tau = 0.0
        self._arm_set_mode(mode=0, kp=0.0, kd=0.0)
        self.msg.mode_machine = self.get_mode_machine()
        self.msg.crc = self.crc.Crc(self.msg)
        self.lowcmd_publisher.Write(self.msg)

    def _ctrl_motor_state(self):
        while True:
            if self.control_mode:
                start = time.time()
                with self.ctrl_lock:
                    arm_q_target     = self.q_target.copy()
                    arm_tauff_target = self.tauff_target.copy()

                cliped = self.clip_arm_q_target(arm_q_target,
                                                velocity_limit=self.arm_velocity_limit)

                self.msg.mode_machine = self.get_mode_machine()
                self.msg.mode_pr = 1

                try:
                    self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
                except Exception:
                    pass

                # --- BRAZOS ---
                for idx, jid in enumerate(G1_29_JointArmIndex):
                    self.msg.motor_cmd[jid].mode = 1
                    self.msg.motor_cmd[jid].q   = float(cliped[idx])
                    self.msg.motor_cmd[jid].dq  = 0.0
                    self.msg.motor_cmd[jid].tau = float(arm_tauff_target[idx])

                    if jid.value in {m.value for m in G1_29_JointWristIndex}:
                        self.msg.motor_cmd[jid].kp = self.kp_wrist
                        self.msg.motor_cmd[jid].kd = self.kd_wrist
                    else:
                        self.msg.motor_cmd[jid].kp = self.kp_low
                        self.msg.motor_cmd[jid].kd = self.kd_low

                self.msg.crc = self.crc.Crc(self.msg)
                self.lowcmd_publisher.Write(self.msg)

                self._last_cmd_q = cliped.copy()

                if self._speed_gradual_max:
                    t_elapsed = time.time() - start
                    self.arm_velocity_limit = 20.0 + 10.0 * min(1.0, t_elapsed / 5.0)

                dt = time.time() - start
                time.sleep(max(0.0, self.control_dt - dt))

            else:
                self._hold_non_arm_joints()
                time.sleep(self.control_dt)

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)

def main():
    app = QtWidgets.QApplication([])
    bridge = UiBridge()
    ctrl = G1_29_ArmController(bridge)
    ctrl.set_control_mode(True)
    app.exec_()

if __name__ == "__main__":
    main()
