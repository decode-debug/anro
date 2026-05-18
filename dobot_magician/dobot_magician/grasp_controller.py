#!/usr/bin/env python3
import math
from functools import partial

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from dobot_magician.geometry_helpers import pose_from_xyz_yaw, transform_pose, yaw_from_quaternion


class GraspController(Node):
    def __init__(self):
        super().__init__('grasp_controller')

        scene_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(PoseStamped, '/camera_link/cube_pose', self.cube_pose_callback, scene_qos)
        self.create_subscription(PoseStamped, '/camera_link/paper_pose', self.paper_pose_callback, scene_qos)

        self.ik_target_pub = self.create_publisher(PoseStamped, '/ik_target', 10)
        self.cube_attached_pub = self.create_publisher(Bool, '/cube_attached', 10)
        self.cube_pose_base_pub = self.create_publisher(PoseStamped, '/cube_pose_base', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pending_cube_pose = None
        self.pending_paper_pose = None
        self.base_cube_pose = None
        self.base_paper_pose = None

        self.motion_pause = Duration(seconds=1.8)
        self.short_pause = Duration(seconds=0.25)
        self.sequence = []
        self.next_action_time = None
        self.sequence_started = False
        self.sequence_finished = False

        self.control_timer = self.create_timer(0.1, self.tick)

    def cube_pose_callback(self, msg: PoseStamped):
        self.pending_cube_pose = msg
        self.try_transform_pending_poses()

    def paper_pose_callback(self, msg: PoseStamped):
        self.pending_paper_pose = msg
        self.try_transform_pending_poses()

    def tick(self):
        self.try_transform_pending_poses()

        if (not self.sequence_started and not self.sequence_finished and
                self.base_cube_pose is not None and self.base_paper_pose is not None):
            self.plan_sequence()

        if not self.sequence:
            return

        now = self.get_clock().now()
        if self.next_action_time is not None and now < self.next_action_time:
            return

        action, delay = self.sequence.pop(0)
        action()

        if self.sequence:
            self.next_action_time = self.get_clock().now() + delay
        else:
            self.next_action_time = None
            self.sequence_finished = True
            self.get_logger().info('Pick-and-place sequence finished.')

    def try_transform_pending_poses(self):
        if self.pending_cube_pose is not None:
            transformed = self.try_transform_to_base(self.pending_cube_pose)
            if transformed is not None:
                self.base_cube_pose = transformed
                self.pending_cube_pose = None

        if self.pending_paper_pose is not None:
            transformed = self.try_transform_to_base(self.pending_paper_pose)
            if transformed is not None:
                self.base_paper_pose = transformed
                self.pending_paper_pose = None

    def try_transform_to_base(self, pose: PoseStamped) -> PoseStamped | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                pose.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().debug(f'Waiting for transform {pose.header.frame_id} -> base_link: {exc}')
            return None

        return transform_pose(pose, transform, target_frame='base_link')

    def plan_sequence(self):
        cube = self.base_cube_pose.pose.position
        paper = self.base_paper_pose.pose.position
        paper_yaw = yaw_from_quaternion(self.base_paper_pose.pose.orientation)

        cube_grasp_z = cube.z + 0.01
        cube_pregrasp_z = cube_grasp_z + 0.05
        placed_cube_center_z = paper.z + 0.0105
        place_z = placed_cube_center_z + 0.01
        preplace_z = place_z + 0.05

        pre_grasp_pose = self.make_target_pose(cube.x, cube.y, cube_pregrasp_z, paper_yaw)
        grasp_pose = self.make_target_pose(cube.x, cube.y, cube_grasp_z, paper_yaw)
        lift_pose = self.make_target_pose(cube.x, cube.y, cube_pregrasp_z, paper_yaw)
        pre_place_pose = self.make_target_pose(paper.x, paper.y, preplace_z, paper_yaw)
        place_pose = self.make_target_pose(paper.x, paper.y, place_z, paper_yaw)
        retreat_pose = self.make_target_pose(paper.x, paper.y, preplace_z, paper_yaw)

        placed_cube_pose = PoseStamped()
        placed_cube_pose.header.frame_id = 'base_link'
        placed_cube_pose.pose.position.x = paper.x
        placed_cube_pose.pose.position.y = paper.y
        placed_cube_pose.pose.position.z = placed_cube_center_z
        placed_cube_pose.pose.orientation = self.base_paper_pose.pose.orientation

        self.sequence = [
            (partial(self.publish_target_pose, pre_grasp_pose), self.motion_pause),
            (partial(self.publish_target_pose, grasp_pose), self.motion_pause),
            (partial(self.publish_cube_attached, True), self.short_pause),
            (partial(self.publish_target_pose, lift_pose), self.motion_pause),
            (partial(self.publish_target_pose, pre_place_pose), self.motion_pause),
            (partial(self.publish_target_pose, place_pose), self.motion_pause),
            (partial(self.publish_cube_attached, False), self.short_pause),
            (partial(self.publish_cube_pose_base, placed_cube_pose), self.short_pause),
            (partial(self.publish_target_pose, retreat_pose), self.motion_pause),
        ]
        self.sequence_started = True
        self.next_action_time = self.get_clock().now()
        self.get_logger().info(
            'Planned pick-and-place sequence from camera detections in camera_link.'
        )

    def make_target_pose(self, x: float, y: float, z: float, yaw: float) -> PoseStamped:
        pose = pose_from_xyz_yaw('base_link', x, y, z, yaw)
        return pose

    def publish_target_pose(self, pose: PoseStamped):
        msg = PoseStamped()
        msg.header.frame_id = pose.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose.pose
        self.ik_target_pub.publish(msg)
        self.get_logger().info(
            'Published IK target: '
            f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}'
        )

    def publish_cube_attached(self, attached: bool):
        self.cube_attached_pub.publish(Bool(data=attached))

    def publish_cube_pose_base(self, pose: PoseStamped):
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose.pose
        self.cube_pose_base_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraspController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()