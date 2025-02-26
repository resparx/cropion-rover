from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cropion_rover',
            executable='main.py',
            name='cropion_rover',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '$(find cropion_rover)/rviz/rover.rviz']
        )
    ]) 