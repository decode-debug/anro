from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dobot_magician")
    default_xacro_path = PathJoinSubstitution(
        [pkg_share, "urdf", "dobot_with_camera.urdf.xacro"]
    )
    default_rviz_config = PathJoinSubstitution([pkg_share, "rviz", "urdf.rviz"])

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_xacro_path,
                description="Sciezka do pliku xacro robota",
            ),
            DeclareLaunchArgument(
                "rvizconfig",
                default_value=default_rviz_config,
                description="Sciezka do pliku konfiguracji RViz",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="dobot_magician",
                executable="forwardkin",
                name="forwardkin",
                output="screen",
            ),
            Node(
                package="dobot_magician",
                executable="inversekin",
                name="inversekin",
                output="screen",
            ),
            Node(
                package="dobot_magician",
                executable="marker_publisher",
                name="marker_publisher",
                output="screen",
            ),
            Node(
                package="dobot_magician",
                executable="random_scene_publisher",
                name="random_scene_publisher",
                output="screen",
            ),
            Node(
                package="dobot_magician",
                executable="grasp_controller",
                name="grasp_controller",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
                output="screen",
            ),
        ]
    )
