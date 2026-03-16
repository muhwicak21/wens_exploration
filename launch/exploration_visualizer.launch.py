#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    drone_ids_arg = DeclareLaunchArgument(
        'drone_ids',
        default_value='[0, 1, 2, 3]',
        description='List of drone IDs to visualize'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='2.0',
        description='Visualization update rate in Hz'
    )
    
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/global_merged_map',
        description='Topic for merged occupancy grid map'
    )
    
    show_trajectories_arg = DeclareLaunchArgument(
        'show_trajectories',
        default_value='true',
        description='Show drone trajectory paths'
    )
    
    show_statistics_arg = DeclareLaunchArgument(
        'show_statistics',
        default_value='true',
        description='Show exploration statistics text'
    )
    
    show_progress_bar_arg = DeclareLaunchArgument(
        'show_progress_bar',
        default_value='true',
        description='Show visual progress bar'
    )
    
    # Visualizer node
    visualizer_node = Node(
        package='wens_exploration',
        executable='exploration_visualizer',
        name='exploration_visualizer',
        output='screen',
        parameters=[{
            'drone_ids': [0, 1, 2, 3],
            'update_rate': LaunchConfiguration('update_rate'),
            'map_topic': LaunchConfiguration('map_topic'),
            'show_trajectories': LaunchConfiguration('show_trajectories'),
            'show_statistics': LaunchConfiguration('show_statistics'),
            'show_progress_bar': LaunchConfiguration('show_progress_bar'),
        }]
    )
    
    return LaunchDescription([
        drone_ids_arg,
        update_rate_arg,
        map_topic_arg,
        show_trajectories_arg,
        show_statistics_arg,
        show_progress_bar_arg,
        visualizer_node,
    ])
