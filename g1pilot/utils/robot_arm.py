import numpy as np
import threading
import time
from enum import IntEnum
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd, LowState_ as hg_LowState
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

kTopicLowCommand_Debug  = "rt/lowcmd"
kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"
G1_29_Num_Motors = 35

class MotorState:
    def __init__(self):
        self.q = None
        self.dq = None

class G1_29_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G1_29_Num_Motors)]

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

class G1_29_ArmController:
    def __init__(self, motion_mode=True, simulation_mode=False):
        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode
        self.kp_high = 120.0
        self.kd_high = 2.0
        self.kp_low = 30.0
        self.kd_low = 2.5
        self.kp_wrist = 15.0
        self.kd_wrist = 1.2
        self.all_motor_q = None
        self.arm_velocity_limit = 6.0
        self.control_dt = 1.0 / 250.0
        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None
        self._q_filt = np.zeros(14)
        self._alpha = 0.95
        self._deadband = np.array([0.012,0.012,0.012,0.012,0.01,0.01,0.01,0.012,0.012,0.012,0.01,0.01,0.01,0.01])
        self._quiet_mode = True
        topic = kTopicLowCommand_Motion if self.motion_mode else kTopicLowCommand_Debug
        self.lowcmd_publisher = ChannelPublisher(topic, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self.subscribe_thread.start()
        while not self.lowstate_buffer.GetData():
            time.sleep(0.1)
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_
        self.msg = self.msg()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()
        self.all_motor_q = self.get_current_motor_q()
        arm_indices = set(member.value for member in G1_29_JointArmIndex)
        for id in G1_29_JointIndex:
            self.msg.motor_cmd[id].mode = 0 if id.value not in arm_indices else 1
            if id.value in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                self.msg.motor_cmd[id].kp = 0.0
                self.msg.motor_cmd[id].kd = 0.0
            self.msg.motor_cmd[id].q = self.all_motor_q[id]
        if self.motion_mode:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state, daemon=True)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.start()

    def set_quiet_mode(self, enable: bool):
        self._quiet_mode = bool(enable)

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G1_29_LowState()
                for id in range(G1_29_Num_Motors):
                    lowstate.motor_state[id].q = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit):
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        return current_q + delta / max(motion_scale, 1.0)

    def _ctrl_motor_state(self):
        while True:
            t0 = time.time()
            with self.ctrl_lock:
                arm_q_target = self.q_target.copy()
                arm_tauff_target = self.tauff_target.copy()
            cliped = self.clip_arm_q_target(arm_q_target, velocity_limit=self.arm_velocity_limit)
            current_q = self.get_current_dual_arm_q()
            err = cliped - current_q
            if self._quiet_mode:
                w = np.clip(np.abs(err) / (self._deadband + 1e-9), 0.0, 1.0)
                cliped = current_q * (1.0 - w) + cliped * w
            self._q_filt = self._alpha * self._q_filt + (1.0 - self._alpha) * cliped
            for idx, id in enumerate(G1_29_JointArmIndex):
                self.msg.motor_cmd[id].mode = 1 if np.abs(err[idx]) > self._deadband[idx] else 0
                self.msg.motor_cmd[id].q = self._q_filt[idx]
                self.msg.motor_cmd[id].dq = 0.0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
            if self._speed_gradual_max:
                tel = t0 - self._gradual_start_time
                self.arm_velocity_limit = 6.0 + (4.0 * min(1.0, tel / 5.0))
            dt = time.time() - t0
            time.sleep(max(0.0, self.control_dt - dt))

    def ctrl_dual_arm(self, q_target, tauff_target):
        with self.ctrl_lock:
            self.q_target = q_target.copy()
            self.tauff_target = tauff_target.copy()

    def get_mode_machine(self):
        return self.lowstate_subscriber.Read().mode_machine

    def get_current_motor_q(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_29_JointIndex])

    def get_current_dual_arm_q(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_29_JointArmIndex])

    def get_current_dual_arm_dq(self):
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in G1_29_JointArmIndex])

    def ctrl_dual_arm_go_home(self):
        with self.ctrl_lock:
            self.q_target[:] = 0.0
        tol = 0.05
        for _ in range(100):
            if np.all(np.abs(self.get_current_dual_arm_q()) < tol):
                break
            time.sleep(0.05)

    def speed_gradual_max(self, t=5.0):
        self._gradual_start_time = time.time()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self):
        self.arm_velocity_limit = 10.0

    def _Is_wrist_motor(self, motor_index):
        return motor_index.value in [19,20,21,26,27,28]

class G1_29_JointArmIndex(IntEnum):
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28

class G1_29_JointIndex(IntEnum):
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11
    kWaistYaw = 12
    kWaistRoll = 13
    kWaistPitch = 14
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34
