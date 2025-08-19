#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Pose, Twist, Point, PointStamped, Quaternion, TransformStamped
)
from tf2_ros import TransformBroadcaster

from astroviz_interfaces.msg import MotorState, MotorStateList

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


def quaternion_multiply(q1, q2):
    """Multiplies two quaternions (w,x,y,z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


class G1JointIndex:
    LeftHipPitch     = 0
    LeftHipRoll      = 1
    LeftHipYaw       = 2
    LeftKnee         = 3
    LeftAnklePitch   = 4
    LeftAnkleRoll    = 5
    RightHipPitch    = 6
    RightHipRoll     = 7
    RightHipYaw      = 8
    RightKnee        = 9
    RightAnklePitch  = 10
    RightAnkleRoll   = 11
    WaistYaw         = 12
    WaistRoll        = 13
    WaistPitch       = 14
    LeftShoulderPitch  = 15
    LeftShoulderRoll   = 16
    LeftShoulderYaw    = 17
    LeftElbow          = 18
    LeftWristRoll      = 19
    LeftWristPitch     = 20
    LeftWristYaw       = 21
    RightShoulderPitch = 22
    RightShoulderRoll  = 23
    RightShoulderYaw   = 24
    RightElbow         = 25
    RightWristRoll     = 26
    RightWristPitch    = 27
    RightWristYaw      = 28


_joint_index_to_ros_name = {
    G1JointIndex.LeftHipPitch:      "left_hip_pitch_joint",
    G1JointIndex.LeftHipRoll:       "left_hip_roll_joint",
    G1JointIndex.LeftHipYaw:        "left_hip_yaw_joint",
    G1JointIndex.LeftKnee:          "left_knee_joint",
    G1JointIndex.LeftAnklePitch:    "left_ankle_pitch_joint",
    G1JointIndex.LeftAnkleRoll:     "left_ankle_roll_joint",
    G1JointIndex.RightHipPitch:     "right_hip_pitch_joint",
    G1JointIndex.RightHipRoll:      "right_hip_roll_joint",
    G1JointIndex.RightHipYaw:       "right_hip_yaw_joint",
    G1JointIndex.RightKnee:         "right_knee_joint",
    G1JointIndex.RightAnklePitch:   "right_ankle_pitch_joint",
    G1JointIndex.RightAnkleRoll:    "right_ankle_roll_joint",
    G1JointIndex.WaistYaw:          "waist_yaw_joint",
    G1JointIndex.WaistRoll:         "waist_roll_joint",
    G1JointIndex.WaistPitch:        "waist_pitch_joint",
    G1JointIndex.LeftShoulderPitch: "left_shoulder_pitch_joint",
    G1JointIndex.LeftShoulderRoll:  "left_shoulder_roll_joint",
    G1JointIndex.LeftShoulderYaw:   "left_shoulder_yaw_joint",
    G1JointIndex.LeftElbow:         "left_elbow_joint",
    G1JointIndex.LeftWristRoll:     "left_wrist_roll_joint",
    G1JointIndex.LeftWristPitch:    "left_wrist_pitch_joint",
    G1JointIndex.LeftWristYaw:      "left_wrist_yaw_joint",
    G1JointIndex.RightShoulderPitch:"right_shoulder_pitch_joint",
    G1JointIndex.RightShoulderRoll: "right_shoulder_roll_joint",
    G1JointIndex.RightShoulderYaw:  "right_shoulder_yaw_joint",
    G1JointIndex.RightElbow:        "right_elbow_joint",
    G1JointIndex.RightWristRoll:    "right_wrist_roll_joint",
    G1JointIndex.RightWristPitch:   "right_wrist_pitch_joint",
    G1JointIndex.RightWristYaw:     "right_wrist_yaw_joint",
}


class RobotState(Node):
    def __init__(self):
        super().__init__('RobotState')

        # --- Params
        self.declare_parameter('interface', 'eth0')
        interface = self.get_parameter('interface').get_parameter_value().string_value
        self.get_logger().info(f'Using interface: {interface}')
        self.ns = '/g1pilot'

        # --- Unitree channel init
        ChannelFactoryInitialize(0, interface)

        # --- Publishers
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", qos_profile)
        self.imu_publisher = self.create_publisher(Imu, f"{self.ns}/imu", qos_profile)
        self.odometry_pub = self.create_publisher(Odometry, f"{self.ns}/odometry", qos_profile)
        self.motor_state_pub = self.create_publisher(MotorStateList, f"{self.ns}/motor_state", qos_profile)

        self.initial_odometry = None

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- JointState setup
        self.joint_indices = sorted(_joint_index_to_ros_name.keys())
        self.joint_names = [_joint_index_to_ros_name[i] for i in self.joint_indices]
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names

        # --- Subscribers (LowState + Odom HF/LF)
        self.subscriber_low_state = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber_low_state.Init(self.callback_lowstate)

        self.subscriber_odom_hf = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.subscriber_odom_hf.Init(self.callback_odometry_hf)

    # === ODOMETRY callbacks ===
    def callback_odometry_hf(self, msg: SportModeState_):
        self._publish_odometry_from_sport(msg, frame_id="odom", child_frame_id="pelvis")


    def _publish_odometry_from_sport(self, est: SportModeState_, frame_id: str = "odom", child_frame_id: str = "pelvis"):
        now = self.get_clock().now().to_msg()

        # --- Odometry message ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id

        # Pose
        if self.initial_odometry is None:
            self.initial_odometry = [
                float(est.position[0]),
                float(est.position[1]),
                float(est.position[2]),

            ]
        odom.pose.pose.position.x = float(est.position[0]) - self.initial_odometry[0]
        odom.pose.pose.position.y = float(est.position[1]) - self.initial_odometry[1]
        odom.pose.pose.position.z = float(est.position[2])

        # Unitree quaternion is [w,x,y,z] -> ROS is x,y,z,w
        qw = float(est.imu_state.quaternion[0])
        qx = float(est.imu_state.quaternion[1])
        qy = float(est.imu_state.quaternion[2])
        qz = float(est.imu_state.quaternion[3])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Pose covariance (36 floats)
        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.05   # var(x)
        pose_cov[7]  = 0.05   # var(y)
        pose_cov[14] = 0.10   # var(z)
        pose_cov[21] = 0.20   # var(roll)
        pose_cov[28] = 0.20   # var(pitch)
        pose_cov[35] = 0.20   # var(yaw)
        odom.pose.covariance = pose_cov

        # Twist (linear from est.velocity; angular: only yaw_speed available)
        odom.twist.twist.linear.x = float(est.velocity[0])
        odom.twist.twist.linear.y = float(est.velocity[1])
        odom.twist.twist.linear.z = float(est.velocity[2])
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(est.yaw_speed)

        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.10
        twist_cov[7]  = 0.10
        twist_cov[14] = 0.20
        twist_cov[21] = 0.30
        twist_cov[28] = 0.30
        twist_cov[35] = 0.30
        odom.twist.covariance = twist_cov

        self.odometry_pub.publish(odom)

        # --- TF: odom -> base_link ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    # === LOWSTATE callback (IMU + joints + motors) ===
    def callback_lowstate(self, msg: LowState_):
        # --- IMU message ---
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Unitree quaternion [w,x,y,z] -> ROS (x,y,z,w)
        qw = float(msg.imu_state.quaternion[0])
        qx = float(msg.imu_state.quaternion[1])
        qy = float(msg.imu_state.quaternion[2])
        qz = float(msg.imu_state.quaternion[3])
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        imu_msg.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu_msg.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu_msg.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu_msg.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu_msg.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu_msg.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_publisher.publish(imu_msg)

        # --- Motors + JointState ---
        posiciones = []
        motor_list_msg = MotorStateList()
        for idx in self.joint_indices:
            if idx < len(msg.motor_state):
                motor_state = MotorState()
                motor_state.name = _joint_index_to_ros_name[idx]
                # Some firmwares provide temperature as array; index 0 used here
                motor_state.temperature = float(getattr(msg.motor_state[idx].temperature, "__getitem__", lambda i: msg.motor_state[idx].temperature)(0)) if hasattr(msg.motor_state[idx].temperature, "__len__") else float(msg.motor_state[idx].temperature)
                motor_state.voltage = float(msg.motor_state[idx].vol)
                motor_state.position = float(msg.motor_state[idx].q)
                motor_state.velocity = float(msg.motor_state[idx].dq)
                motor_list_msg.motor_list.append(motor_state)
                posiciones.append(float(msg.motor_state[idx].q))
            else:
                self.get_logger().error(
                    f"LowState_ has {len(msg.motor_state)} motors; expected index {idx}. Not publishing joints.")
                return

        self.motor_state_pub.publish(motor_list_msg)

        if len(posiciones) != len(self.joint_state_msg.name):
            self.get_logger().error(
                f"Positions ({len(posiciones)}) do not match joint names ({len(self.joint_state_msg.name)}). Not publishing.")
            return

        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = posiciones
        self.joint_pub.publish(self.joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
