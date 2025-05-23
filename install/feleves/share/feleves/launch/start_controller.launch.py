# feleves/launch/start_controller.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='feleves', # Fontos, hogy ez a csomagod neve legyen!
            executable='controller', # A Python node entry_point neve a setup.py-ból
            name='feleves_controller',
            output='screen'
        )
    ])
