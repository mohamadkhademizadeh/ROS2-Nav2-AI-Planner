from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo world
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            os.getenv('GAZEBO_RESOURCE_PATH', ''), '')]),
    )

    # Spawn simple robot via gazebo_ros factory (using a built-in model if available)
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'diff_bot', '-database', 'turtlebot3_burger'],
        output='screen'
    )

    # Remap Nav2 cmd topic to /nav_cmd_vel so AI planner can intercept
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join('/opt/ros/humble/share/nav2_bringup/launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': ''}.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join('/opt/ros/humble/share/slam_toolbox/launch', 'online_async_launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # AI planner
    ai = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                os.getenv('COLCON_CURRENT_PREFIX', '/ws/install/nav_ai/share/nav_ai/launch'),
                'ai_planner.launch.py'
            )
        ])
    )

    # Simple twist mux: Nav2 publishes to /nav_cmd_vel; AI planner reads /scan and /nav_cmd_vel and outputs /cmd_vel
    # In many setups, Nav2 publishes to /cmd_vel directly. You can remap in your robot bringup.
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        spawn,
        nav2,
        slam,
        ai,
    ])
