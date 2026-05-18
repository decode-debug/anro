#!/usr/bin/env python3
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

_cfg_path = os.path.join(
    get_package_share_directory('dobot_magician'), 'config', 'params.yaml'
)
with open(_cfg_path) as _f:
    _cfg = yaml.safe_load(_f)


def rot_x(a):
    return np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(a), -np.sin(a), 0],
            [0, np.sin(a), np.cos(a), 0],
            [0, 0, 0, 1],
        ]
    )


def rot_z(a):
    return np.array(
        [
            [np.cos(a), -np.sin(a), 0, 0],
            [np.sin(a), np.cos(a), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )


def rot_y(a):
    return np.array(
        [
            [np.cos(a), 0, np.sin(a), 0],
            [0, 1, 0, 0],
            [-np.sin(a), 0, np.cos(a), 0],
            [0, 0, 0, 1],
        ]
    )


def trans(x, y, z):
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


class ForwardKin(Node):
    def __init__(self):
        super().__init__('forward_kin')

        self.dim = _cfg['dimensions']  # słownik wymiarów z params.yaml

        self.base_z = self.dim['box_z']
        self.base_turn_z = self.dim['total_base_height'] - self.dim['box_z']
        self.lower_arm_len = self.dim['rear_arm']
        self.upper_arm_len = self.dim['fore_arm']
        self.servo_l = self.dim['servo_l']
        self.gripper_z = self.dim['gripper_z']

        self.joint_positions = {}

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )
        self.pub = self.create_publisher(PoseStamped, '/end_effector_pose', 10)

    def joint_state_callback(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))

        q1 = joint_map.get('box_to_base', 0.0)
        q2 = joint_map.get('base_to_reararm', 0.0)
        q3 = joint_map.get('reararm_to_forearm', 0.0)
        q5 = joint_map.get('servo_to_gripper', 0.0)

        # Kinematyka prosta - metoda geometryczna
        # T_base:     translacja do góry bazy
        # T_q1:       obrót wokół Z (yaw)
        # T_shoulder: translacja do ramienia
        # T_q2:       obrót wokół Y (pitch)
        # T_lower:    translacja wzdłuż lower_arm
        # T_q3:       obrót wokół Y (pitch)
        # T_upper:    translacja wzdłuż upper_arm

        T = (
            trans(0, 0, self.base_z + self.base_turn_z)
            @ rot_z(q1)
            @ rot_y(q2)
            @ trans(0, 0, self.lower_arm_len)
            @ rot_y(q3)
            @ trans(0, 0, self.upper_arm_len)
            @ rot_y(-q2 - q3)
            @ trans(0, 0, -(self.servo_l + self.gripper_z))
            @ rot_y(math.pi / 2)
            @ rot_x(q5)
        )

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()

        # Pozycja
        pose.pose.position.x = float(T[0, 3])
        pose.pose.position.y = float(T[1, 3])
        pose.pose.position.z = float(T[2, 3])

        # Orientacja z macierzy rotacji -> kwaternion
        R = T[:3, :3]
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        pose.pose.orientation.x = float(x)
        pose.pose.orientation.y = float(y)
        pose.pose.orientation.z = float(z)
        pose.pose.orientation.w = float(w)

        self.pub.publish(pose)
        # self.get_logger().info(
        #     f'EE pos: x={T[0,3]:.3f} y={T[1,3]:.3f} z={T[2,3]:.3f}'


def main():
    rclpy.init()
    node = ForwardKin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
