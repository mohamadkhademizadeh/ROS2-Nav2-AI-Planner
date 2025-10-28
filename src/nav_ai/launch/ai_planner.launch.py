from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_ai',
            executable='ai_planner',
            name='ai_planner',
            output='screen',
            parameters=[
                {'use_policy': False},
                {'max_linear': 0.5},
                {'max_angular': 1.2},
                {'safe_distance': 0.6},
            ]
        )
    ])
