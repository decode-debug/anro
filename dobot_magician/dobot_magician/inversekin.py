#!/usr/bin/env python3
from collections import deque
import math
import os

from ament_index_python.packages import get_package_share_directory
from dobot_magician.geometry_helpers import normalize_angle, yaw_from_quaternion
from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

_cfg_path = os.path.join(
    get_package_share_directory('dobot_magician'), 'config', 'params.yaml'
)
with open(_cfg_path) as _f:
    _cfg = yaml.safe_load(_f)


class InverseKin(Node):
    def __init__(self):
        super().__init__('inverse_kin')

        self.dim = _cfg['dimensions']

        self.base_z = self.dim['box_z']
        self.base_turn_z = self.dim['total_base_height'] - self.dim['box_z']
        self.lower_arm_len = self.dim['rear_arm']
        self.upper_arm_len = self.dim['fore_arm']
        self.servo_l = self.dim['servo_l']
        self.gripper_z = self.dim['gripper_z']

        self.lim = _cfg['joint_limits']
        self.base_min = self.lim['joint1']['min']
        self.base_max = self.lim['joint1']['max']

        self.L1_min = self.lim['joint2']['min']
        self.L1_max = self.lim['joint2']['max']

        self.L2_min = self.lim['joint3']['min']
        self.L2_max = self.lim['joint3']['max']

        self.joint_names = [
            'box_to_base',
            'base_to_reararm',
            'reararm_to_forearm',
            'forearm_to_servo',
            'servo_to_gripper',
        ]
        self.current_joint_positions = {name: 0.0 for name in self.joint_names}
        self.motion_hz = 30.0
        self.motion_duration = 1.5
        self.motion_queue = deque()

        self.sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ik_target',
            self.pose_callback,
            10,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.motion_timer = self.create_timer(
            1.0 / self.motion_hz, self.publish_motion_step
        )
        self.publish_joint_state(
            [self.current_joint_positions[name] for name in self.joint_names]
        )
        self.get_logger().info(
            'InverseKin node started – /clicked_point or /ik_target '
            '→ interpolated /joint_states'
        )

    def joint_state_callback(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in self.current_joint_positions:
                self.current_joint_positions[name] = float(position)

    def point_callback(self, msg: PointStamped):
        self.queue_target(
            msg.point.x,
            msg.point.y,
            msg.point.z,
            tool_yaw=0.0,
        )

    def pose_callback(self, msg: PoseStamped):
        if msg.header.frame_id not in ('', 'base_link'):
            self.get_logger().warn(f'Ignoring IK target in unsupported frame: {
                    msg.header.frame_id}')
            return

        tool_yaw = yaw_from_quaternion(msg.pose.orientation)
        self.queue_target(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            tool_yaw=tool_yaw,
        )

    def queue_target(self, x: float, y: float, z_world: float, tool_yaw: float):
        target_positions = self.solve_ik(x, y, z_world, tool_yaw)
        if target_positions is None:
            return

        start_positions = [
            self.current_joint_positions[name] for name in self.joint_names
        ]
        steps = max(int(self.motion_duration * self.motion_hz), 1)
        self.motion_queue.clear()
        for step in range(1, steps + 1):
            alpha = step / steps
            interpolated = [
                start + (target - start) * alpha
                for start, target in zip(start_positions, target_positions)
            ]
            self.motion_queue.append(interpolated)

        self.get_logger().info(f'Queued IK motion to x={x:.3f}, y={y:.3f}, z={
                z_world:.3f}, yaw={tool_yaw:.3f}')

    def solve_ik(self, x: float, y: float, z_world: float, tool_yaw: float):
        z = z_world - (self.base_z + self.base_turn_z) + (self.servo_l + self.gripper_z)
        r = math.sqrt(x**2 + y**2)
        D = math.sqrt(r**2 + z**2)

        L1 = self.lower_arm_len
        L2 = self.upper_arm_len

        if D > (L1 + L2) or D < abs(L1 - L2):
            self.get_logger().warn(f'Punkt poza zasięgiem! D={D:.3f}, Max={L1+L2}')
            return None

        q1 = math.atan2(y, x)
        a_1 = math.atan2(z, r)
        cos_a2 = self.clamp((L1**2 + D**2 - L2**2) / (2 * L1 * D))
        q2 = math.pi / 2 - (a_1 + math.acos(cos_a2))
        cos_q3 = self.clamp((L1**2 + L2**2 - D**2) / (2 * L1 * L2))
        q3 = math.pi - math.acos(cos_q3)

        q4 = -(q2 + q3) + math.pi
        q5 = normalize_angle(tool_yaw - q1)

        limits = (
            (q1 >= self.base_min and q1 <= self.base_max)
            and (q2 >= self.L1_min and q2 <= self.L1_max)
            and (q3 >= self.L2_min and q3 <= self.L2_max)
        )

        if not limits:
            self.get_logger().warn('Punkt nie miesci sie w limitach zlacz')
            return None

        return [
            float(q1),
            float(q2),
            float(q3),
            float(q4),
            float(q5),
        ]

    def publish_motion_step(self):
        if not self.motion_queue:
            return

        positions = self.motion_queue.popleft()
        self.publish_joint_state(positions)

    def publish_joint_state(self, positions):
        new_msg = JointState()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = ''
        new_msg.name = self.joint_names
        new_msg.position = positions
        self.pub.publish(new_msg)

    @staticmethod
    def clamp(value: float, lower: float = -1.0, upper: float = 1.0) -> float:
        return max(lower, min(upper, value))


def main():
    rclpy.init()
    node = InverseKin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
