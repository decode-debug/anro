#!/usr/bin/env python3
import random

from dobot_magician.geometry_helpers import pose_from_xyz_yaw
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class RandomScenePublisher(Node):
    def __init__(self):
        super().__init__('random_scene_publisher')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cube_pub = self.create_publisher(
            PoseStamped, '/camera_link/cube_pose', qos
        )
        self.paper_pub = self.create_publisher(
            PoseStamped, '/camera_link/paper_pose', qos
        )

        self.random = random.Random()
        self.publish_timer = self.create_timer(1.0, self.publish_scene)
        self.scene_published = False

    def publish_scene(self):
        if self.scene_published:
            return

        cube_x, cube_y, paper_x, paper_y = self.sample_positions()
        paper_yaw = self.random.uniform(-0.7, 0.7)
        stamp = self.get_clock().now().to_msg()

        cube_pose = pose_from_xyz_yaw(
            'camera_link',
            cube_x,
            cube_y,
            -0.420,
            0.0,
            stamp,
        )
        paper_pose = pose_from_xyz_yaw(
            'camera_link',
            paper_x,
            paper_y,
            -0.4295,
            paper_yaw,
            stamp,
        )

        self.cube_pub.publish(cube_pose)
        self.paper_pub.publish(paper_pose)
        self.scene_published = True
        self.publish_timer.cancel()
        self.get_logger().info(
            'Published random scene in camera_link: '
            f'cube=({cube_x:.3f}, {cube_y:.3f}, -0.420), '
            f'paper=({paper_x:.3f}, {paper_y:.3f}, -0.430), yaw={paper_yaw:.2f}'
        )

    def sample_positions(self) -> tuple[float, float, float, float]:
        for _ in range(50):
            cube_x = self.random.uniform(0.11, 0.19)
            cube_y = self.random.uniform(-0.07, 0.07)
            paper_x = self.random.uniform(0.16, 0.25)
            paper_y = self.random.uniform(-0.09, 0.09)
            if ((cube_x - paper_x) ** 2 + (cube_y - paper_y) ** 2) ** 0.5 >= 0.08:
                return cube_x, cube_y, paper_x, paper_y

        return 0.14, -0.03, 0.22, 0.05


def main(args=None):
    rclpy.init(args=args)
    node = RandomScenePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
