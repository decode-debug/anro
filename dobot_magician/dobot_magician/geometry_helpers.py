import math

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    return quaternion


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_multiply(left: Quaternion, right: Quaternion) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = left.x, left.y, left.z, left.w
    x2, y2, z2, w2 = right.x, right.y, right.z, right.w
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector(quaternion: Quaternion, vector: tuple[float, float, float]) -> tuple[float, float, float]:
    vector_quaternion = Quaternion(x=vector[0], y=vector[1], z=vector[2], w=0.0)
    inverse = Quaternion(
        x=-quaternion.x,
        y=-quaternion.y,
        z=-quaternion.z,
        w=quaternion.w,
    )
    rotated = quaternion_multiply(
        Quaternion(x=quaternion.x, y=quaternion.y, z=quaternion.z, w=quaternion.w),
        vector_quaternion,
    )
    rotated = quaternion_multiply(
        Quaternion(x=rotated[0], y=rotated[1], z=rotated[2], w=rotated[3]),
        inverse,
    )
    return rotated[0], rotated[1], rotated[2]


def pose_from_xyz_yaw(frame_id: str, x: float, y: float, z: float, yaw: float, stamp=None) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp is not None:
        pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation = quaternion_from_yaw(yaw)
    return pose


def transform_pose(
    pose_stamped: PoseStamped,
    transform: TransformStamped,
    target_frame: str | None = None,
) -> PoseStamped:
    rotated_position = rotate_vector(
        transform.transform.rotation,
        (
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
        ),
    )
    orientation = quaternion_multiply(transform.transform.rotation, pose_stamped.pose.orientation)

    transformed = PoseStamped()
    transformed.header.frame_id = target_frame or transform.header.frame_id
    transformed.header.stamp = transform.header.stamp
    transformed.pose.position.x = rotated_position[0] + transform.transform.translation.x
    transformed.pose.position.y = rotated_position[1] + transform.transform.translation.y
    transformed.pose.position.z = rotated_position[2] + transform.transform.translation.z
    transformed.pose.orientation.x = orientation[0]
    transformed.pose.orientation.y = orientation[1]
    transformed.pose.orientation.z = orientation[2]
    transformed.pose.orientation.w = orientation[3]
    return transformed