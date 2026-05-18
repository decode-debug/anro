#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from dobot_magician.geometry_helpers import transform_pose


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_publisher")

        scene_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.marker_pub = self.create_publisher(MarkerArray, "/scene_markers", 10)
        self.create_subscription(
            PoseStamped, "/camera_link/cube_pose", self.cube_pose_callback, scene_qos
        )
        self.create_subscription(
            PoseStamped, "/camera_link/paper_pose", self.paper_pose_callback, scene_qos
        )
        self.create_subscription(
            PoseStamped, "/cube_pose_base", self.base_cube_pose_callback, 10
        )
        self.create_subscription(
            Bool, "/cube_attached", self.cube_attached_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cube_attached = False
        self.pending_cube_pose = None
        self.pending_paper_pose = None
        self.base_cube_pose = None
        self.base_paper_pose = None

        self.publish_timer = self.create_timer(0.1, self.publish_markers)

    def cube_pose_callback(self, msg: PoseStamped):
        self.pending_cube_pose = msg
        self.try_transform_pending_poses()

    def paper_pose_callback(self, msg: PoseStamped):
        self.pending_paper_pose = msg
        self.try_transform_pending_poses()

    def base_cube_pose_callback(self, msg: PoseStamped):
        self.base_cube_pose = msg

    def cube_attached_callback(self, msg: Bool):
        self.cube_attached = msg.data

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
                "base_link",
                pose.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().debug(
                f"Waiting for transform {pose.header.frame_id} -> base_link: {exc}"
            )
            return None

        return transform_pose(pose, transform, target_frame="base_link")

    def publish_markers(self):
        self.try_transform_pending_poses()

        markers = MarkerArray()
        markers.markers.append(self.build_cube_marker())
        markers.markers.append(self.build_paper_marker())
        self.marker_pub.publish(markers)

    def build_cube_marker(self) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scene"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = 0.98
        marker.color.g = 0.49
        marker.color.b = 0.09
        marker.color.a = 1.0

        if self.cube_attached or self.base_cube_pose is None:
            marker.header.frame_id = "gripper"
            marker.frame_locked = True
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.025
            marker.pose.orientation.w = 1.0
            return marker

        marker.header.frame_id = self.base_cube_pose.header.frame_id
        marker.pose = self.base_cube_pose.pose
        return marker

    def build_paper_marker(self) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scene"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.scale.x = 0.05
        marker.scale.y = 0.10
        marker.scale.z = 0.001
        marker.color.r = 0.93
        marker.color.g = 0.93
        marker.color.b = 0.90
        marker.color.a = 1.0

        if self.base_paper_pose is None:
            marker.header.frame_id = "base_link"
            marker.action = Marker.ADD
            marker.pose.position.x = 0.18
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0005
            marker.pose.orientation.w = 1.0
            return marker

        marker.header.frame_id = self.base_paper_pose.header.frame_id
        marker.action = Marker.ADD
        marker.pose = self.base_paper_pose.pose
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
