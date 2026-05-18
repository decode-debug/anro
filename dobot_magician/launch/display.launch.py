from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare("dobot_magician")
    default_xacro_path = PathJoinSubstitution(
        [pkg_share, "urdf", "dobot_with_camera.urdf.xacro"]
    )
    default_rviz_config = PathJoinSubstitution([pkg_share, "rviz", "urdf.rviz"])

    ld.add_action(
        DeclareLaunchArgument(
            name="model",
            default_value=default_xacro_path,
            description="Sciezka do pliku xacro robota",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config,
            description="Sciezka do pliku konfiguracji RViz",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="use_gui",
            default_value="false",
            description="Uruchom joint_state_publisher_gui do recznego sterowania z suwakiem",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="use_inversekin",
            default_value="true",
            description="Uruchom inversekin, aby PublishPoint w RViz sterowal robotem",
        )
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    ld.add_action(robot_state_publisher_node)

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )
    ld.add_action(joint_state_publisher_gui_node)

    inverse_kin_node = Node(
        package="dobot_magician",
        executable="inversekin",
        name="inversekin",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_inversekin")),
    )
    ld.add_action(inverse_kin_node)

    forward_kin_node = Node(
        package="dobot_magician",
        executable="forwardkin",
        name="forwardkin",
        output="screen",
    )
    ld.add_action(forward_kin_node)

    marker_node = Node(
        package="dobot_magician",
        executable="marker_publisher",
        name="marker_publisher",
        output="screen",
    )
    ld.add_action(marker_node)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        output="screen",
    )
    ld.add_action(rviz_node)
    return ld
