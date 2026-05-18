import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RosToRviz(Node):
    def __init__(self):
        super().__init__("ros_to_rviz")

        self.subscription = self.create_subscription(
            JointState, "/dobot_joint_states", self.listener_callback, 10
        )

        self.publisher_ = self.create_publisher(JointState, "/joint_states_robot", 10)
        self.get_logger().info(
            "RosToRviz node started – /dobot_joint_states → /joint_states_robot"
        )

    def listener_callback(self, msg):
        joint_map = dict(zip(msg.name, msg.position))

        q1 = joint_map.get("magician_joint_1", 0.0)  # obrót podstawy (oś Z)
        q2 = joint_map.get("magician_joint_2", 0.0)  # ramię tylne (oś Y)
        q3 = joint_map.get("magician_joint_3", 0.0)  # ramię przednie (oś Y)
        q5 = joint_map.get("magician_joint_4", 0.0)  # serwo (oś Y)
        # q5 = joint_map.get('magician_joint_prismatic_l', 0.0)  # obrót grippera (oś Z)

        new_msg = JointState()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = ""

        new_msg.name = [
            "box_to_base",
            "base_to_reararm",
            "reararm_to_forearm",
            "forearm_to_servo",
            "servo_to_gripper",
        ]

        new_msg.position = [
            # box_to_base     – obrót podstawy
            float(q1),
            float(q2),
            float(q3),  # base_to_reararm – ramię tylne
            # reararm_to_forearm – ramię przednie
            float(q3 - q2) + math.pi / 2,
            # forearm_to_servo – kompensacja pionu
            float(-q3) + math.pi / 2,
            q5,  # servo_to_gripper – obrót grippera
        ]

        self.publisher_.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RosToRviz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
