#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import time
import threading
import numpy as np
from enum import IntEnum
from PyQt5 import QtWidgets, QtCore

import rclpy
from rclpy.time import Time
from rclpy.duration import Duration

# TF2
try:
    from tf2_ros import Buffer, TransformListener
    from tf2_geometry_msgs import do_transform_pose
    _HAS_TF2 = True
except Exception:
    _HAS_TF2 = False

try:
    from geometry_msgs.msg import PoseStamped
except Exception:
    PoseStamped = None

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

    12:(-2.618,   2.618),
    13:(-0.52,    0.52),
    14:(-0.52,    0.52),

    15:(-3.0892,  2.6704),
    16:(-1.5882,  2.2515),
    17:(-2.618,   2.618),
    18:(-1.0472,  2.0944),
    19:(-1.972222054, 1.972222054),
    20:(-1.614429558, 1.614429558),
    21:(-1.614429558, 1.614429558),

    22:(-3.0892,  2.6704),
    23:(-2.2515,  1.5882),
    24:(-2.618,   2.618),
    25:(-1.0472,  2.0944),
    26:(-1.972222054, 1.972222054),
    27:(-1.614429558, 1.614429558),
    28:(-1.614429558, 1.614429558),
}
RIGHT_JOINT_INDICES_LIST = [22,23,24,25,26,27,28]
LEFT_JOINT_INDICES_LIST  = [15,16,17,18,19,20,21]

JOINT_GROUPS = {
    "left":  LEFT_JOINT_INDICES_LIST,
    "right": RIGHT_JOINT_INDICES_LIST,
    "both":  LEFT_JOINT_INDICES_LIST + RIGHT_JOINT_INDICES_LIST,
}

JOINT_NAMES_LEFT = [
    "L Shoulder Pitch", "L Shoulder Roll", "L Shoulder Yaw",
    "L Elbow", "L Wrist Roll", "L Wrist Pitch", "L Wrist Yaw"
]
JOINT_NAMES_RIGHT = [
    "R Shoulder Pitch", "R Shoulder Roll", "R Shoulder Yaw",
    "R Elbow", "R Wrist Roll", "R Wrist Pitch", "R Wrist Yaw"
]

def _names_for(group):
    if group == "left":  return JOINT_NAMES_LEFT
    if group == "right": return JOINT_NAMES_RIGHT
    if group == "both":  return JOINT_NAMES_LEFT + JOINT_NAMES_RIGHT
    raise ValueError(f"Unknown group {group}")

JOINTID_TO_DUALINDEX = {}
for i, jid in enumerate([
    G1_29_JointArmIndex.kLeftShoulderPitch,
    G1_29_JointArmIndex.kLeftShoulderRoll,
    G1_29_JointArmIndex.kLeftShoulderYaw,
    G1_29_JointArmIndex.kLeftElbow,
    G1_29_JointArmIndex.kLeftWristRoll,
    G1_29_JointArmIndex.kLeftWristPitch,
    G1_29_JointArmIndex.kLeftWristyaw,
    G1_29_JointArmIndex.kRightShoulderPitch,
    G1_29_JointArmIndex.kRightShoulderRoll,
    G1_29_JointArmIndex.kRightShoulderYaw,
    G1_29_JointArmIndex.kRightElbow,
    G1_29_JointArmIndex.kRightWristRoll,
    G1_29_JointArmIndex.kRightWristPitch,
    G1_29_JointArmIndex.kRightWristYaw,
]):
    JOINTID_TO_DUALINDEX[jid.value] = i


def clamp_joint_vector(q_vals, joint_id_list):
    out = []
    for ii, jidx in enumerate(joint_id_list):
        lo, hi = JOINT_LIMITS_RAD[jidx]
        out.append(float(np.clip(q_vals[ii], lo, hi)))
    return np.array(out, dtype=float)


# ------------------------------ UI ------------------------------
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


class ArmGUI(QtWidgets.QWidget):
    valuesChanged = QtCore.pyqtSignal(object)

    def __init__(self, title, joint_ids, joint_names, get_initial_q_radians_callable, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setMinimumWidth(720 if len(joint_ids) == 14 else 520)

        self.joint_ids   = joint_ids[:]
        self.joint_names = joint_names[:]

        root = QtWidgets.QVBoxLayout(self)
        self.setLayout(root)

        columns = QtWidgets.QHBoxLayout()
        root.addLayout(columns, stretch=1)

        self.sliders = []
        self.value_labels = []

        init_q = get_initial_q_radians_callable() or [0.0]*len(self.joint_ids)
        if len(init_q) != len(self.joint_ids):
            init_q = [0.0]*len(self.joint_ids)
        init_q = clamp_joint_vector(init_q, self.joint_ids)

        def make_slider_row(name, jidx, deg0):
            row = QtWidgets.QHBoxLayout()
            lab = QtWidgets.QLabel(name)
            lab.setFixedWidth(180)

            lo_rad, hi_rad = JOINT_LIMITS_RAD[jidx]
            lo_deg = int(round(math.degrees(lo_rad)))
            hi_deg = int(round(math.degrees(hi_rad)))

            sld = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            sld.setMinimum(lo_deg); sld.setMaximum(hi_deg)
            sld.setSingleStep(1); sld.setPageStep(5)
            sld.setTickInterval(max(5, (hi_deg - lo_deg) // 12))
            sld.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)

            deg0 = max(lo_deg, min(hi_deg, deg0))
            sld.setValue(deg0)

            val_lab = QtWidgets.QLabel(f"{deg0:>4}°")
            val_lab.setFixedWidth(60)

            return row, lab, sld, val_lab

        if len(self.joint_ids) == 14:
            left_ids  = LEFT_JOINT_INDICES_LIST
            right_ids = RIGHT_JOINT_INDICES_LIST

            left_names  = JOINT_NAMES_LEFT
            right_names = JOINT_NAMES_RIGHT

            col_left  = QtWidgets.QVBoxLayout()
            col_right = QtWidgets.QVBoxLayout()
            columns.addLayout(col_left, 1)
            columns.addSpacing(16)
            columns.addLayout(col_right, 1)

            left_title  = QtWidgets.QLabel("Left Arm")
            right_title = QtWidgets.QLabel("Right Arm")
            left_title.setStyleSheet("font-weight:600;")
            right_title.setStyleSheet("font-weight:600;")
            col_left.addWidget(left_title)
            col_right.addWidget(right_title)

            for i, (name, jidx) in enumerate(zip(left_names, left_ids)):
                deg0 = int(round(math.degrees(init_q[i])))
                row, lab, sld, val_lab = make_slider_row(name, jidx, deg0)
                sld.valueChanged.connect(lambda v, idx=i, vl=val_lab: self._on_slider(idx, v, vl))
                row.addWidget(lab); row.addWidget(sld, 1); row.addWidget(val_lab)
                col_left.addLayout(row)
                self.sliders.append(sld); self.value_labels.append(val_lab)

            offset = 7
            for k, (name, jidx) in enumerate(zip(right_names, right_ids)):
                i = offset + k
                deg0 = int(round(math.degrees(init_q[i])))
                row, lab, sld, val_lab = make_slider_row(name, jidx, deg0)
                sld.valueChanged.connect(lambda v, idx=i, vl=val_lab: self._on_slider(idx, v, vl))
                row.addWidget(lab); row.addWidget(sld, 1); row.addWidget(val_lab)
                col_right.addLayout(row)
                self.sliders.append(sld); self.value_labels.append(val_lab)

        else:
            single_col = QtWidgets.QVBoxLayout()
            columns.addLayout(single_col, 1)

            for i, (name, jidx) in enumerate(zip(self.joint_names, self.joint_ids)):
                deg0 = int(round(math.degrees(init_q[i])))
                row, lab, sld, val_lab = make_slider_row(name, jidx, deg0)
                sld.valueChanged.connect(lambda v, idx=i, vl=val_lab: self._on_slider(idx, v, vl))
                row.addWidget(lab); row.addWidget(sld, 1); row.addWidget(val_lab)
                single_col.addLayout(row)
                self.sliders.append(sld); self.value_labels.append(val_lab)

        btns = QtWidgets.QHBoxLayout()
        btn_center = QtWidgets.QPushButton("Center (0°)")
        btn_center.clicked.connect(self._center_all)
        btns.addStretch(1); btns.addWidget(btn_center)
        root.addLayout(btns)

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
            center = 0 if lo_deg <= 0 <= hi_deg else int((lo_deg + hi_deg) / 2)
            sld.setValue(center)

    def update_from_robot_pose(self, q_rad_in_gui_order):
        if not q_rad_in_gui_order or len(q_rad_in_gui_order) != len(self.joint_ids):
            return
        q_rad_in_gui_order = clamp_joint_vector(q_rad_in_gui_order, self.joint_ids)
        for i, (val, jidx) in enumerate(zip(q_rad_in_gui_order, self.joint_ids)):
            lo_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][0])))
            hi_deg = int(round(math.degrees(JOINT_LIMITS_RAD[jidx][1])))
            deg = int(round(math.degrees(val)))
            deg = max(lo_deg, min(hi_deg, deg))
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(deg)
            self.value_labels[i].setText(f"{deg:>4}°")
            self.sliders[i].blockSignals(False)

class G1_29_ArmController:
    def __init__(self, ui_bridge: QtCore.QObject, controlled_arms: str = 'right',
                 show_ui: bool = True, ros_node=None, urdf_path=None, mesh_dir=None):
        self.motor_state = [MotorState() for _ in range(35)]

        self.topic_motion_cmd = 'rt/arm_sdk'
        self.topic_low_cmd    = 'rt/lowstate'

        self.q_target      = np.zeros(14)
        self.tauff_target  = np.zeros(14)
        self.kp_high = 300.0; self.kd_high = 3.0
        self.kp_low  = 80.0;  self.kd_low  = 3.0
        self.kp_wrist= 40.0;  self.kd_wrist= 1.5

        self.control_mode = False
        self.show_ui = bool(show_ui)

        self.all_motor_q = None
        self.arm_velocity_limit = 2.0
        self.control_dt = 1.0 / 250.0

        self._last_cmd_q = None
        self._last_tick_time = None

        self._speed_gradual_max = False

        self._bridge = ui_bridge
        self._gui = None

        self.controlled_arms = controlled_arms.lower().strip()
        if self.controlled_arms not in ("right", "left", "both"):
            self.controlled_arms = "right"

        self.gui_joint_ids   = JOINT_GROUPS[self.controlled_arms]
        self.gui_joint_names = _names_for(self.controlled_arms)
        self.gui_title       = {
            "right": "G1 – Right Arm Control",
            "left":  "G1 – Left Arm Control",
            "both":  "G1 – Both Arms Control",
        }[self.controlled_arms]

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

        # === IK params ===
        self._ik_enabled = True
        self._ik_alpha = 0.2
        self._ik_max_dq_step = 0.05
        self._ik_damping = 1e-6
        self._ik_max_iter = 60
        self._ik_tol = 1e-4

        self._ik_use_waist = False
        self._ik_track_orientation = False
        self._ik_pos_gain = 1.0
        self._ik_ori_gain = 0.2
        self._ik_debug = False

        self._ik_have_joint_map = False
        self._ik_goal_left  = None
        self._ik_goal_right = None
        self._ik_q_prev_14  = None
        self._ik_q_prev_full = None
        self._log_ik_active = False

        self._ros_node = ros_node
        self._ik_world_frame = 'odom'
        self._tf_buffer = None
        self._tf_listener = None

        self._ee_auto_calibrate = True
        self._ee_off_right_xyz = np.zeros(3)
        self._ee_off_right_rpy_deg = np.zeros(3)
        self._ee_off_left_xyz  = np.zeros(3)
        self._ee_off_left_rpy_deg  = np.zeros(3)

        self._T_off_right_static = None
        self._T_off_left_static  = None
        self._T_off_right_auto   = None
        self._T_off_left_auto    = None
        self._auto_done_right    = False
        self._auto_done_left     = False

        if self._ros_node is not None:
            def _safe_get(name, default):
                try:
                    if self._ros_node.has_parameter(name):
                        return self._ros_node.get_parameter(name).value
                except Exception:
                    pass
                return default

            self._ik_use_waist = bool(_safe_get('ik_use_waist', self._ik_use_waist))
            self._ik_alpha = float(_safe_get('ik_alpha', self._ik_alpha))
            self._ik_max_dq_step = float(_safe_get('ik_max_dq_step', self._ik_max_dq_step))
            self.arm_velocity_limit = float(_safe_get('arm_velocity_limit', self.arm_velocity_limit))
            self._ik_track_orientation = bool(_safe_get('ik_track_orientation', self._ik_track_orientation))
            self._ik_world_frame = str(_safe_get('ik_world_frame', self._ik_world_frame))
            self._ee_auto_calibrate = bool(_safe_get('ee_auto_calibrate', self._ee_auto_calibrate))

            arr = _safe_get('ee_offset_right_xyz', list(self._ee_off_right_xyz))
            if isinstance(arr, (list, tuple)) and len(arr) == 3:
                self._ee_off_right_xyz = np.array(arr, dtype=float)
            arr = _safe_get('ee_offset_right_rpy_deg', list(self._ee_off_right_rpy_deg))
            if isinstance(arr, (list, tuple)) and len(arr) == 3:
                self._ee_off_right_rpy_deg = np.array(arr, dtype=float)

            arr = _safe_get('ee_offset_left_xyz', list(self._ee_off_left_xyz))
            if isinstance(arr, (list, tuple)) and len(arr) == 3:
                self._ee_off_left_xyz = np.array(arr, dtype=float)
            arr = _safe_get('ee_offset_left_rpy_deg', list(self._ee_off_left_rpy_deg))
            if isinstance(arr, (list, tuple)) and len(arr) == 3:
                self._ee_off_left_rpy_deg = np.array(arr, dtype=float)

            if _HAS_TF2:
                try:
                    self._tf_buffer = Buffer()
                    self._tf_listener = TransformListener(self._tf_buffer, self._ros_node)
                    print(f"[IK] TF listo. Frame objetivo: '{self._ik_world_frame}'", flush=True)
                except Exception as e:
                    print(f"[IK] No pude iniciar TF2: {e}", flush=True)

        self._left_arm_names = [
            "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
            "left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint",
        ]
        self._right_arm_names = [
            "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
            "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint",
        ]

        try:
            import pinocchio as pin
            from pinocchio import SE3
            self.pin = pin
            self.SE3 = SE3

            if urdf_path is None or mesh_dir is None:
                try:
                    from ament_index_python.packages import get_package_share_directory
                    pkg_share = get_package_share_directory('g1pilot')
                    urdf_path = urdf_path or os.path.join(pkg_share, 'description_files', 'urdf', 'g1_29dof.urdf')
                    mesh_dir  = mesh_dir  or os.path.join(pkg_share, 'description_files', 'meshes')
                except Exception:
                    pass

            if urdf_path and os.path.exists(urdf_path):
                self.model, _, _ = self.pin.buildModelsFromUrdf(urdf_path, package_dirs=[mesh_dir] if mesh_dir else [])
                self.data = self.pin.Data(self.model)

                _joint_index_to_ros_name = {
                    0: "left_hip_pitch_joint",   1: "left_hip_roll_joint",    2: "left_hip_yaw_joint",
                    3: "left_knee_joint",        4: "left_ankle_pitch_joint", 5: "left_ankle_roll_joint",
                    6: "right_hip_pitch_joint",  7: "right_hip_roll_joint",   8: "right_hip_yaw_joint",
                    9: "right_knee_joint",      10: "right_ankle_pitch_joint",11: "right_ankle_roll_joint",
                    12:"waist_yaw_joint",       13: "waist_roll_joint",      14: "waist_pitch_joint",
                    15:"left_shoulder_pitch_joint", 16:"left_shoulder_roll_joint", 17:"left_shoulder_yaw_joint",
                    18:"left_elbow_joint", 19:"left_wrist_roll_joint", 20:"left_wrist_pitch_joint", 21:"left_wrist_yaw_joint",
                    22:"right_shoulder_pitch_joint",23:"right_shoulder_roll_joint",24:"right_shoulder_yaw_joint",
                    25:"right_elbow_joint",26:"right_wrist_roll_joint",27:"right_wrist_pitch_joint",28:"right_wrist_yaw_joint",
                }
                self._ros_joint_names = [_joint_index_to_ros_name[i] for i in range(29)]
                self._name_to_q_index = {}
                self._name_to_v_index = {}
                for j in range(1, self.model.njoints):
                    jnt = self.model.joints[j]
                    if jnt.nq == 1:
                        nm = self.model.names[j]
                        if nm in self._ros_joint_names:
                            self._name_to_q_index[nm] = jnt.idx_q
                            self._name_to_v_index[nm] = jnt.idx_v

                self._frame_right = 'right_hand_point_contact'
                self._frame_left  = 'left_hand_point_contact'
                names_frames = [f.name for f in self.model.frames]
                self._fid_right = self.model.getFrameId(self._frame_right) if self._frame_right in names_frames else None
                self._fid_left  = self.model.getFrameId(self._frame_left ) if self._frame_left  in names_frames else None

                self._ros_to_g1_index = {v: k for k, v in _joint_index_to_ros_name.items()}

                def _mk_static_T(xyz, rpy_deg):
                    rpy = np.radians(rpy_deg.astype(float))
                    R = self.pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
                    return self.SE3(R, np.array(xyz, dtype=float))
                self._T_off_right_static = _mk_static_T(self._ee_off_right_xyz, self._ee_off_right_rpy_deg)
                self._T_off_left_static  = _mk_static_T(self._ee_off_left_xyz,  self._ee_off_left_rpy_deg)

                self._ik_have_joint_map = True

                if self._ros_node is not None and PoseStamped is not None:
                    qos = rclpy.qos.QoSProfile(depth=10)
                    if self._fid_right is not None:
                        self._ros_node.create_subscription(
                            PoseStamped, '/g1pilot/right_hand_goal',
                            lambda msg: self._ik_target_cb('right', msg), qos)
                    if self._fid_left is not None:
                        self._ros_node.create_subscription(
                            PoseStamped, '/g1pilot/left_hand_goal',
                            lambda msg: self._ik_target_cb('left', msg), qos)
            else:
                print("[IK] URDF don't found", flush=True)
        except Exception as e:
            print(f"[IK] Failed initializing the IK: {e}", flush=True)
            self._ik_have_joint_map = False

        self.publish_thread = threading.Thread(target=self._ctrl_motor_state, daemon=True)
        self.publish_thread.start()

    def _get_gui_initial_q(self):
        try:
            with self.ctrl_lock:
                base14 = self.q_target.copy() if hasattr(self, "q_target") and len(self.q_target) == 14 \
                         else self.get_current_dual_arm_q().copy()
        except Exception:
            base14 = self.get_current_dual_arm_q().copy()
        out = []
        for jid in self.gui_joint_ids:
            dual_idx = JOINTID_TO_DUALINDEX[jid]
            out.append(float(base14[dual_idx]))
        return out

    def _open_gui_if_needed(self):
        if not self.show_ui:
            return
        def _make_or_show():
            if self._gui is None:
                self._gui = ArmGUI(
                    title=self.gui_title,
                    joint_ids=self.gui_joint_ids,
                    joint_names=self.gui_joint_names,
                    get_initial_q_radians_callable=self._get_gui_initial_q
                )
                self._gui.valuesChanged.connect(self._on_gui_values)
            self._gui.move(100, 100)
            self._gui.show(); self._gui.raise_(); self._gui.activateWindow()
        self._bridge.runSignal.emit(_make_or_show)

    def _close_gui_if_needed(self):
        if self._gui is None:
            return
        self._bridge.runSignal.emit(self._gui.hide)

    def _on_gui_values(self, radians_list):
        if len(radians_list) != len(self.gui_joint_ids):
            return
        with self.ctrl_lock:
            if not hasattr(self, "q_target") or len(self.q_target) != 14:
                self.q_target = self.get_current_dual_arm_q().copy()
            for val, jid in zip(radians_list, self.gui_joint_ids):
                dual_idx = JOINTID_TO_DUALINDEX[jid]
                self.q_target[dual_idx] = float(val)

    def set_control_mode(self, enabled: bool):
        self.control_mode = enabled
        print(f"[ArmGUI] set_control_mode -> {enabled}", flush=True)
        if enabled:
            with self.ctrl_lock:
                cur14 = self.get_current_dual_arm_q().copy()
                self.q_target = cur14.copy()
                self.tauff_target = np.zeros_like(self.tauff_target)
                self._last_cmd_q = cur14.copy()
                self._ik_q_prev_14 = cur14.copy()
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
        target_q = np.asarray(target_q, dtype=float).reshape(-1)
        last = np.asarray(self._last_cmd_q, dtype=float).reshape(-1)
        dt = self._compute_dt()
        max_step = float(velocity_limit) * dt
        delta = target_q - last
        delta = np.clip(delta, -max_step, max_step)
        return last + delta

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
                    self.msg.motor_cmd[jid].kd = self.kp_wrist if False else self.kd_wrist
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

    def _transform_pose_to_world(self, ps):
        if (self._ros_node is None) or (self._tf_buffer is None) or (not hasattr(ps, "header")):
            return ps
        src = ps.header.frame_id or ""
        target = self._ik_world_frame
        if not src or src == target:
            return ps
        try:
            tf = self._tf_buffer.lookup_transform(target, src, Time(), timeout=Duration(seconds=0.2))
            return do_transform_pose(ps, tf)
        except Exception as e:
            print(f"[IK] WARN TF {src} -> {target} failed: {e}. Using pose.", flush=True)
            return ps

    def _fk_current_ee(self, side):
        try:
            import pinocchio as pin
        except Exception:
            return None
        q = self.pin.neutral(self.model)
        cur_all = self.get_current_motor_q()
        for jid_idx, ros_name in enumerate(self._ros_joint_names):
            if ros_name in self._name_to_q_index:
                q[self._name_to_q_index[ros_name]] = float(cur_all[jid_idx])
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        fid = self._fid_right if side == 'right' else self._fid_left
        if fid is None:
            return None
        return self.data.oMf[fid]

    def _ik_target_cb(self, side, msg):
        msg_tf = self._transform_pose_to_world(msg)
        if self._ik_debug:
            print(f"[IK] target {side} @'{msg_tf.header.frame_id or self._ik_world_frame}' "
                  f"p=({msg_tf.pose.position.x:.3f},{msg_tf.pose.position.y:.3f},{msg_tf.pose.position.z:.3f})", flush=True)

        if not self._ik_have_joint_map:
            return
        try:
            o = msg_tf.pose.orientation
            p = msg_tf.pose.position
            q = self.pin.Quaternion(o.w, o.x, o.y, o.z)
            T_goal_in = self.SE3(q.matrix(), np.array([p.x, p.y, p.z], dtype=float))

            T_static = self._T_off_right_static if side == 'right' else self._T_off_left_static

            T_auto = self._T_off_right_auto if side == 'right' else self._T_off_left_auto
            auto_done = self._auto_done_right if side == 'right' else self._auto_done_left
            if self._ee_auto_calibrate and not auto_done:
                M_cur = self._fk_current_ee(side)
                if M_cur is not None:
                    T_pre = T_goal_in * T_static
                    T_auto = T_pre.inverse() * M_cur
                    if side == 'right':
                        self._T_off_right_auto = T_auto
                        self._auto_done_right = True
                    else:
                        self._T_off_left_auto = T_auto
                        self._auto_done_left = True
                    t = T_auto.translation
                    print(f"[IK] auto-calibrated {side}: offset transl=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})", flush=True)
                else:
                    pass

            if side == 'right':
                self._ik_goal_right = T_goal_in * T_static * (self._T_off_right_auto if self._T_off_right_auto is not None else self.SE3.Identity())
            else:
                self._ik_goal_left  = T_goal_in * T_static * (self._T_off_left_auto  if self._T_off_left_auto  is not None else self.SE3.Identity())

        except Exception as e:
            print(f"[IK] target_cb error: {e}", flush=True)

    def _ordered_1d_names(self):
        out = []
        for j in range(1, self.model.njoints):
            jnt = self.model.joints[j]
            if jnt.nq == 1:
                nm = self.model.names[j]
                if nm in self._name_to_q_index:
                    out.append(nm)
        return out

    def _ordered_selected_names(self, side):
        wanted = set()
        if self._ik_use_waist:
            wanted.update(["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"])
        if side == 'right':
            wanted.update(self._right_arm_names)
        else:
            wanted.update(self._left_arm_names)
        out = []
        for nm in self._ordered_1d_names():
            if nm in wanted:
                out.append(nm)
        return out

    def _ik_solve_single(self, side, q_init=None):
        if not self._ik_have_joint_map:
            return None

        if side == 'right':
            fid = self._fid_right
            arm_ids = RIGHT_JOINT_INDICES_LIST
            arm_names = self._right_arm_names
        else:
            fid = self._fid_left
            arm_ids = LEFT_JOINT_INDICES_LIST
            arm_names = self._left_arm_names
        if fid is None:
            return None

        if q_init is not None and len(q_init) == self.model.nq:
            q = q_init.copy()
        elif hasattr(self, "_ik_q_prev_full") and (self._ik_q_prev_full is not None) and len(self._ik_q_prev_full) == self.model.nq:
            q = self._ik_q_prev_full.copy()
        else:
            q = self.pin.neutral(self.model)
            cur_all = self.get_current_motor_q()
            for jid_idx, ros_name in enumerate(self._ros_joint_names):
                if ros_name in self._name_to_q_index:
                    q[self._name_to_q_index[ros_name]] = float(cur_all[jid_idx])

        T_goal = self._ik_goal_right if side == 'right' else self._ik_goal_left
        if T_goal is None:
            return None

        selected_names = []
        for j in range(1, self.model.njoints):
            jnt = self.model.joints[j]
            if jnt.nq != 1:
                continue
            nm = self.model.names[j]
            if nm in arm_names:
                selected_names.append(nm)
        if not selected_names:
            return None

        v_cols = [self._name_to_v_index[nm] for nm in selected_names if nm in self._name_to_v_index]
        q_idx  = [self._name_to_q_index[nm] for nm in selected_names if nm in self._name_to_q_index]
        if not v_cols or not q_idx or len(v_cols) != len(q_idx):
            return None

        for _ in range(self._ik_max_iter):
            self.pin.forwardKinematics(self.model, self.data, q)
            self.pin.updateFramePlacements(self.model, self.data)
            M_cur = self.data.oMf[fid]

            J6 = self.pin.computeFrameJacobian(self.model, self.data, q, fid, self.pin.LOCAL_WORLD_ALIGNED)
            J_eff = J6[:, v_cols]

            if self._ik_track_orientation:
                err6 = self.pin.log(M_cur.inverse() * T_goal).vector
                J_use = np.vstack([
                    J_eff[:3, :] * self._ik_pos_gain,
                    J_eff[3:, :] * self._ik_ori_gain
                ])
                err_use = np.hstack([
                    err6[:3] * self._ik_pos_gain,
                    err6[3:] * self._ik_ori_gain
                ])
                err_norm = float(np.linalg.norm(err6))
            else:
                err_use = (T_goal.translation - M_cur.translation).reshape(3)
                J_use = J_eff[:3, :]
                err_norm = float(np.linalg.norm(err_use))

            if self._ik_debug:
                print(f"[IK] {side} | ||err||={err_norm:.4e}", flush=True)

            if err_norm < self._ik_tol:
                break

            JJt = J_use @ J_use.T
            dq_red = J_use.T @ np.linalg.solve(JJt + self._ik_damping * np.eye(J_use.shape[0]), err_use)

            dq_full = np.zeros(self.model.nv)
            for i, vi in enumerate(v_cols):
                step_i = float(np.clip(dq_red[i], -self._ik_max_dq_step, self._ik_max_dq_step))
                dq_full[vi] = step_i

            for i, (nm, qi) in enumerate(zip(selected_names, q_idx)):
                step = dq_full[v_cols[i]]
                q[qi] += step
                g1_idx = self._ros_to_g1_index.get(nm, None)
                if g1_idx is not None and g1_idx in JOINT_LIMITS_RAD:
                    lo, hi = JOINT_LIMITS_RAD[g1_idx]
                    q[qi] = float(np.clip(q[qi], lo, hi))

        self._ik_q_prev_full = q.copy()

        out7 = []
        for jid in arm_ids:
            ros_name = self._ros_joint_names[jid]
            if ros_name in self._name_to_q_index:
                val = float(q[self._name_to_q_index[ros_name]])
            else:
                dual_idx = JOINTID_TO_DUALINDEX[jid]
                val = float(self.get_current_dual_arm_q()[dual_idx])
            lo, hi = JOINT_LIMITS_RAD[jid]
            out7.append(float(np.clip(val, lo, hi)))
        return np.array(out7, dtype=float)

    def _ik_update_q_target(self, base_q14):
        base_q14 = np.asarray(base_q14, dtype=float).reshape(-1)
        if base_q14.size != 14:
            base_q14 = self.get_current_dual_arm_q().astype(float)

        left_q7 = right_q7 = None
        if self._ik_enabled:
            if self.controlled_arms in ('left','both') and self._ik_goal_left is not None:
                left_q7  = self._ik_solve_single('left')
            if self.controlled_arms in ('right','both') and self._ik_goal_right is not None:
                right_q7 = self._ik_solve_single('right')

        q_new = base_q14.copy()
        if left_q7  is not None: q_new[0:7]  = left_q7
        if right_q7 is not None: q_new[7:14] = right_q7

        if (self._ik_q_prev_14 is None) or (np.asarray(self._ik_q_prev_14).reshape(-1).size != 14):
            self._ik_q_prev_14 = q_new.copy()

        q_smooth = (1.0 - self._ik_alpha)*self._ik_q_prev_14 + self._ik_alpha*q_new
        self._ik_q_prev_14 = q_smooth.copy()
        return q_smooth

    def _ctrl_motor_state(self):
        while True:
            if self.control_mode:
                start = time.time()
                with self.ctrl_lock:
                    arm_q_target     = self.q_target.copy()
                    arm_tauff_target = self.tauff_target.copy()
                if self._ik_have_joint_map and self._ik_enabled and (self._ik_goal_left is not None or self._ik_goal_right is not None):
                    if not hasattr(self, "_log_ik_active"):
                        print("[IK] Activated: Using IK to update q_target", flush=True)
                        self._log_ik_active = True
                    arm_q_target = self._ik_update_q_target(arm_q_target)

                cliped = self.clip_arm_q_target(arm_q_target, velocity_limit=self.arm_velocity_limit)

                self.msg.mode_machine = self.get_mode_machine()
                self.msg.mode_pr = 1

                try:
                    self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
                except Exception:
                    pass

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


def main():
    app = QtWidgets.QApplication([])
    bridge = UiBridge()
    ctrl = G1_29_ArmController(bridge, controlled_arms='right')
    ctrl.set_control_mode(True)
    app.exec_()

if __name__ == "__main__":
    main()
