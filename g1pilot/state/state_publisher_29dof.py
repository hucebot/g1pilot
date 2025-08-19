import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__("state_publisher")

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)

        joint_names = [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ]

        # robot state
        q = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            -0.6,
            0.0,
            0.0,
            1.2,
            -0.6,
            0.0,
            -0.6,
            0.0,
            0.0,
            1.2,
            -0.6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        # message declarations
        com_tf = TransformStamped()
        com_tf.header.frame_id = "odom"
        com_tf.child_frame_id = "pelvis"
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = joint_names
                joint_state.position = q[7:]

                # update transform
                com_tf.header.stamp = now.to_msg()
                com_tf.transform.translation.x = q[0]
                com_tf.transform.translation.y = q[1]
                com_tf.transform.translation.z = q[2]
                com_tf.transform.rotation = Quaternion(
                    x=q[3],
                    y=q[4],
                    z=q[5],
                    w=q[6]
                )


                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(com_tf)

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


def main():
    node = StatePublisher()


if __name__ == "__main__":
    main()