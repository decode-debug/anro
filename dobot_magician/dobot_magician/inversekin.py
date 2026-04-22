
#!/usr/bin/env python3
import math
import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import numpy as np
from ament_index_python.packages import get_package_share_directory


_cfg_path = os.path.join(get_package_share_directory('dobot_magician'), 'config', 'params.yaml')
with open(_cfg_path) as _f:
    _cfg = yaml.safe_load(_f)

class InverseKin(Node):
    def __init__(self):
        super().__init__("inverse_kin")

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

        self.sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.callback,
            10,

        )
        self.pub = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )
        self.get_logger().info('InverseKin node started – /clicked_point → /joint_states')

    def callback(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z - (self.base_z + self.base_turn_z) + (self.servo_l + self.gripper_z)

        r = math.sqrt(x**2 + y**2)
        D = math.sqrt(r**2 + z**2)

        L1 = self.lower_arm_len
        L2 = self.upper_arm_len

        if D > (L1 + L2) or D < abs(L1 - L2):
            self.get_logger().warn(f'Punkt poza zasięgiem! D={D:.3f}, Max={L1+L2}')
            return

        q1 = math.atan2(y, x)
        a_1 = math.atan2(z, r)
        cos_a2 = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
        q2 = math.pi / 2 - (a_1 + math.acos(cos_a2))
        cos_q3 = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
        q3 = math.pi - math.acos(cos_q3)

        q4 = -(q2 + q3)+math.pi
        q5 = 0.0

        limits = ((q1 >= self.base_min and q1 <= self.base_max) 
                  and (q2 >= self.L1_min and q2 <= self.L1_max)
                  and (q3 >= self.L2_min and q3 <= self.L2_max))
        

        if not limits:
            self.get_logger().warn('Punkt nie miesci sie w limitach zlacz')
            return

        new_msg = JointState()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = ''

        new_msg.name = [
            'box_to_base',
            'base_to_reararm',
            'reararm_to_forearm',
            'forearm_to_servo',
            'servo_to_gripper',
        ]

        new_msg.position = [
            float(q1),
            float(q2),
            float(q3),
            float(q4),
            float(q5),
        ]

        self.pub.publish(new_msg)
        self.get_logger().info(
            f'Inverse kin node working: q1:{float(q1)}, q2:{float(q2)}, q3:{float(q3)}, q4:{float(q4)}, q5:{float(q5)}'
        )




def main():
    rclpy.init()
    node = InverseKin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()