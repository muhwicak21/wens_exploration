from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('exploration_mode', default_value='frontier_hgrid_role'),
        DeclareLaunchArgument('experiment_seed', default_value='1'),
        DeclareLaunchArgument('drone_num', default_value='4'),
        DeclareLaunchArgument('map_topic', default_value='/swarm_exploration/occupancy_grid'),
        DeclareLaunchArgument('odom_topic_format', default_value='/drone_{id}_visual_slam/odom'),
        DeclareLaunchArgument('target_coverage', default_value='0.95'),
        DeclareLaunchArgument('coverage_hold_time', default_value='5.0'),
        DeclareLaunchArgument('max_duration', default_value='600.0'),
        DeclareLaunchArgument('output_root', default_value='/root/catkin_ws/src/eval/results'),
    ]

    evaluator = Node(
        package='eval',
        executable='exploration_time_evaluator',
        name='exploration_time_evaluator',
        output='screen',
        parameters=[{
            'exploration_mode': LaunchConfiguration('exploration_mode'),
            'experiment_seed': LaunchConfiguration('experiment_seed'),
            'drone_num': LaunchConfiguration('drone_num'),
            'map_topic': LaunchConfiguration('map_topic'),
            'odom_topic_format': LaunchConfiguration('odom_topic_format'),
            'target_coverage': LaunchConfiguration('target_coverage'),
            'coverage_hold_time': LaunchConfiguration('coverage_hold_time'),
            'max_duration': LaunchConfiguration('max_duration'),
            'output_root': LaunchConfiguration('output_root'),
        }],
    )

    return LaunchDescription(args + [evaluator])
