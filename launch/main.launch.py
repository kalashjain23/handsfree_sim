from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='handsfree_sim',
            executable='controller',
            name='controller'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        )
    ])
