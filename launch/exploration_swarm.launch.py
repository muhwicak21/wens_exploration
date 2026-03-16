#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():
    """Generate launch description for local exploration system"""
    
    # Declare launch arguments
    declare_drone_ids = DeclareLaunchArgument(
        'drone_ids',
        default_value='[0, 1, 2, 3]',
        description='List of drone IDs to manage'
    )
    
    declare_update_rate = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Exploration loop update rate (Hz)'
    )
    
    declare_goal_timeout = DeclareLaunchArgument(
        'goal_timeout_sec',
        default_value='10.0',
        description='Timeout to detect stuck drone and reassign goal (seconds)'
    )
    
    declare_goal_reached_threshold = DeclareLaunchArgument(
        'goal_reached_threshold',
        default_value='0.6',
        description='Distance threshold to consider goal reached (meters)'
    )
    
    declare_min_frontier_size = DeclareLaunchArgument(
        'min_frontier_size',
        default_value='1',
        description='Minimum cluster size for valid frontier (cells)'
    )
    
    declare_safe_neighborhood = DeclareLaunchArgument(
        'safe_neighborhood',
        default_value='1',
        description='Neighborhood radius for safety check (cells, 1=3x3)'
    )
    
    declare_free_threshold = DeclareLaunchArgument(
        'free_threshold',
        default_value='50',
        description='Maximum occupancy value considered free'
    )
    
    declare_occupied_threshold = DeclareLaunchArgument(
        'occupied_threshold',
        default_value='80',
        description='Minimum occupancy value considered occupied'
    )
    
    declare_random_fallback = DeclareLaunchArgument(
        'random_fallback_enabled',
        default_value='true',
        description='Enable random goal fallback when no frontiers found'
    )
    
    declare_random_range = DeclareLaunchArgument(
        'random_range',
        default_value='[3.0, 8.0]',
        description='Min and max radius for random local goals (meters)'
    )
    
    declare_visited_memory_radius = DeclareLaunchArgument(
        'visited_memory_radius',
        default_value='1.5',
        description='Radius around visited positions to avoid (meters)'
    )
    
    declare_frontier_min_distance = DeclareLaunchArgument(
        'frontier_min_distance',
        default_value='0.5',
        description='Minimum distance between frontiers and current goal (meters)'
    )
    
    declare_debug_mode = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug logging and RViz markers'
    )
    
    # Swarm coordination parameters
    declare_swarm_coordination = DeclareLaunchArgument(
        'swarm_coordination_enabled',
        default_value='true',
        description='Enable lightweight swarm coordination to avoid revisiting explored areas'
    )
    
    declare_swarm_visited_radius = DeclareLaunchArgument(
        'swarm_visited_radius',
        default_value='0.5',
        description='Radius to consider position as visited by swarm (meters)'
    )
    
    declare_swarm_goal_conflict_radius = DeclareLaunchArgument(
        'swarm_goal_conflict_radius',
        default_value='3.0',
        description='Minimum distance between drone goals (meters)'
    )
    
    declare_swarm_share_interval = DeclareLaunchArgument(
        'swarm_share_interval',
        default_value='0.5',
        description='Interval to share visited positions (seconds)'
    )
    
    # Global map frontier detection parameters
    declare_use_global_frontiers = DeclareLaunchArgument(
        'use_global_frontiers',
        default_value='true',
        description='Use global map for frontier detection (prevents revisiting)'
    )
    
    declare_global_safety_check = DeclareLaunchArgument(
        'global_safety_check',
        default_value='true',
        description='Check global map for obstacle avoidance'
    )
    
    # Topic name patterns ({drone_id} substituted per drone at runtime)
    declare_topic_map = DeclareLaunchArgument(
        'topic_map',
        default_value='/drone_{drone_id}/planner/grid/grid_map/occupancy_inflate',
        description='Topic pattern for local inflated occupancy map from EgoPlanner'
    )
    
    declare_topic_odom = DeclareLaunchArgument(
        'topic_odom',
        default_value='/drone_{drone_id}/localization/odometry',
        description='Topic pattern for drone odometry'
    )
    
    declare_topic_swarm_trajs = DeclareLaunchArgument(
        'topic_swarm_trajs',
        default_value='/drone_{drone_id}/planner/planning/swarm_trajs',
        description='Topic pattern for EgoPlanner swarm trajectory broadcasts'
    )
    
    declare_topic_goal = DeclareLaunchArgument(
        'topic_goal',
        default_value='/drone_{drone_id}/move_base_simple/goal',
        description='Topic pattern for publishing exploration goals to EgoPlanner'
    )
    
    # Local exploration node
    exploration_node = Node(
        package='wens_exploration',
        executable='exploration_node_local',
        name='exploration_manager_local',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'drone_ids': LaunchConfiguration('drone_ids'),
            'update_rate': LaunchConfiguration('update_rate'),
            'goal_timeout_sec': LaunchConfiguration('goal_timeout_sec'),
            'goal_reached_threshold': LaunchConfiguration('goal_reached_threshold'),
            'min_frontier_size': LaunchConfiguration('min_frontier_size'),
            'safe_neighborhood': LaunchConfiguration('safe_neighborhood'),
            'free_threshold': LaunchConfiguration('free_threshold'),
            'occupied_threshold': LaunchConfiguration('occupied_threshold'),
            'random_fallback_enabled': LaunchConfiguration('random_fallback_enabled'),
            'random_range': LaunchConfiguration('random_range'),
            'visited_memory_radius': LaunchConfiguration('visited_memory_radius'),
            'frontier_min_distance': LaunchConfiguration('frontier_min_distance'),
            'debug_mode': LaunchConfiguration('debug_mode'),
            # Swarm coordination
            'swarm_coordination_enabled': LaunchConfiguration('swarm_coordination_enabled'),
            'swarm_visited_radius': LaunchConfiguration('swarm_visited_radius'),
            'swarm_goal_conflict_radius': LaunchConfiguration('swarm_goal_conflict_radius'),
            'swarm_share_interval': LaunchConfiguration('swarm_share_interval'),
            # Global map frontier detection
            'use_global_frontiers': LaunchConfiguration('use_global_frontiers'),
            'global_safety_check': LaunchConfiguration('global_safety_check'),
            # Topic name patterns
            'topic_map': LaunchConfiguration('topic_map'),
            'topic_odom': LaunchConfiguration('topic_odom'),
            'topic_swarm_trajs': LaunchConfiguration('topic_swarm_trajs'),
            'topic_goal': LaunchConfiguration('topic_goal'),
        }]
    )

    # Exploration visualizer node - shows map exploration results
    exploration_visualizer_node = launch_ros.actions.Node(
        package='wens_exploration',
        executable='exploration_visualizer',
        name='exploration_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'drone_ids': [0, 1, 2, 3],
            'update_rate': 10.0,
            'map_topic': '/global_merged_map',
            'topic_odom': LaunchConfiguration('topic_odom'),
            'show_progress_bar': True,
        }]
    )
    
    return LaunchDescription([
        declare_drone_ids,
        declare_update_rate,
        declare_goal_timeout,
        declare_goal_reached_threshold,
        declare_min_frontier_size,
        declare_safe_neighborhood,
        declare_free_threshold,
        declare_occupied_threshold,
        declare_random_fallback,
        declare_random_range,
        declare_visited_memory_radius,
        declare_frontier_min_distance,
        declare_debug_mode,
        # Swarm coordination
        declare_swarm_coordination,
        declare_swarm_visited_radius,
        declare_swarm_goal_conflict_radius,
        declare_swarm_share_interval,
        # Global map frontier detection
        declare_use_global_frontiers,
        declare_global_safety_check,
        # Topic name patterns
        declare_topic_map,
        declare_topic_odom,
        declare_topic_swarm_trajs,
        declare_topic_goal,
        exploration_node,
        exploration_visualizer_node,
    ])
