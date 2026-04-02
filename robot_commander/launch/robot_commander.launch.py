from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_commander',
            executable='robot_commander',
            name='robot_commander',
            output='screen'
        )
    ])