"""Launch file for Dobot pick-and-place sequence."""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the dobot_move node."""
    height_arg = DeclareLaunchArgument('height', default_value='3')

    # dobot_move = Node(
    #     package='lab2',
    #     executable='dobot_move',
    #     name='dobot_move',
    #     output='screen',
    # ) dobot_move
    
    dobot_tower = Node(
        package='lab2',
        executable='dobot_tower',
        name='dobot_tower',
        output='screen',
        parameters=[
            {
                'height' : LaunchConfiguration('height')                
            }
        ], 
    )
    
    # dobot_print_position_node = Node(
    #     package='lab2',
    #     executable='dobot_print_position',
    #     name='dobot_print_position',
    #     output='screen',
    # )dobot_print_position_node
    

    return LaunchDescription([height_arg, dobot_tower])
