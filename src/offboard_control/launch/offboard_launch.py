from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Takeoff to 1.0m and hover
        Node(
            package='offboard_control',
            executable='takeoff_to_1m',
            name='takeoff_to_1m',
            output='screen'
        ),

        # Move forward for 2 seconds, then hover
        Node(
            package='offboard_control',
            executable='forward_for_2s',
            name='forward_for_2s',
            output='screen'
        ),
    ])
