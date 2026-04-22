import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = FindPackageShare('dobot_magician')

    # Ścieżki
    default_xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'dobot.urdf.xacro'])
    default_rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'urdf.rviz'])

    ld.add_action(DeclareLaunchArgument(
        name='model',
        default_value=default_xacro_path,
        description='Ścieżka do pliku xacro robota'))

    ld.add_action(DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config,
        description='Ścieżka do pliku konfiguracji RViz'))

    # Przetwarza xacro → URDF i przekazuje jako parametr robot_description
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    # robot_state_publisher słucha /joint_states_robot (nie /joint_states)
    # dzięki temu joint_state_publisher i dobot_state_publisher nie zaśmiecają danych
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        # remappings=[('/joint_states', '/joint_states_robot')]
    )
    ld.add_action(robot_state_publisher_node)

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )
    ld.add_action(rviz_node)

    # Węzeł mapujący złącza prawdziwego robota → /joint_states dla robot_state_publisher
    # rostorviz_node = Node(
    #     package='dobot_magician',
    #     executable='rostorviz',
    #     name='rostorviz',
    #     output='screen'
    # )
    # ld.add_action(rostorviz_node)

    # Kinematyka prosta → /end_effector_pose
    forward_kin_node = Node(
        package='dobot_magician',
        executable='forwardkin',
        name='forwardkin',
        output='screen'
    )
    ld.add_action(forward_kin_node)

    inverse_kin_node = Node(
        package='dobot_magician',
        executable='inversekin',
        name='inversekin'
    )
    ld.add_action(inverse_kin_node)
    return ld

