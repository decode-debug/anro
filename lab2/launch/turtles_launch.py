"""Plik launch dla naszego węzła kontrolującego żółwie.

Uruchamia on zarówno symulator, jak i nasz węzeł kontrolujący.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generuje opis uruchomienia dla naszego zadania."""
    # Deklaracja argumentów konsoli dla nazw żółwi
    t2_arg = DeclareLaunchArgument('t2_name', default_value='zolw_2')
    t3_arg = DeclareLaunchArgument('t3_name', default_value='zolw_3')

    # Węzeł symulatora (od razu włączamy planszę)
    turtlesim_node = Node(
        package='turtlesim', executable='turtlesim_node', name='sim'
    )

    controller_node = Node(
        package='lab2',
        executable='turtle_controller',
        name='turtle_controller',
        parameters=[
            {
                'turtle2_name': LaunchConfiguration('t2_name'),
                'turtle3_name': LaunchConfiguration('t3_name'),
            }
        ],
    )

    return LaunchDescription([t2_arg, t3_arg, turtlesim_node, controller_node])
