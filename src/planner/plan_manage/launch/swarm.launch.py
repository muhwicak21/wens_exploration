import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 定义参数的 LaunchConfiguration
    map_size_x = LaunchConfiguration('map_size_x', default = 45.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 20.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 5.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')
    exploration_mode = LaunchConfiguration('exploration_mode', default='frontier_hgrid_role')
    run_evaluation = LaunchConfiguration('run_evaluation', default=False)
    experiment_seed = LaunchConfiguration('experiment_seed', default='1')
    target_coverage = LaunchConfiguration('target_coverage', default='0.95')
    coverage_hold_time = LaunchConfiguration('coverage_hold_time', default='5.0')
    max_duration = LaunchConfiguration('max_duration', default='600.0')
    eval_output_root = LaunchConfiguration(
        'eval_output_root',
        default='/root/catkin_ws/src/eval/results')
    
    # 声明全局参数
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    exploration_mode_arg = DeclareLaunchArgument(
        'exploration_mode',
        default_value='frontier_hgrid_role',
        choices=[
            'frontier',
            'frontier_separation',
            'frontier_hgrid',
            'frontier_hgrid_role'
        ],
        description='Exploration ablation configuration'
    )
    enable_multi_drone_separation = PythonExpression([
        "'", exploration_mode,
        "' in ['frontier_separation', 'frontier_hgrid', 'frontier_hgrid_role']"
    ])
    enable_hgrid = PythonExpression([
        "'", exploration_mode,
        "' in ['frontier_hgrid', 'frontier_hgrid_role']"
    ])
    enable_role_assignment = PythonExpression([
        "'", exploration_mode,
        "' == 'frontier_hgrid_role'"
    ])
    odom_topic_format = PythonExpression([
        "'/drone_{id}_' + '", odom_topic, "'"
    ])

    # 地图属性以及是否使用动力学仿真
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap 
    
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')
    
    use_dynamic = LaunchConfiguration('use_dynamic', default=False)  
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    use_dummy_goal = LaunchConfiguration('use_dummy_goal', default=False)
    use_dummy_goal_cmd = DeclareLaunchArgument(
        'use_dummy_goal',
        default_value=use_dummy_goal,
        description='Use random exploration_goal_dummy instead of frontier exploration manager'
    )

    use_rviz = LaunchConfiguration('use_rviz', default=True)
    use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value=use_rviz,
        description='Launch RViz'
    )
    run_evaluation_cmd = DeclareLaunchArgument(
        'run_evaluation',
        default_value=run_evaluation,
        description='Start exploration-time evaluator together with the swarm'
    )
    experiment_seed_cmd = DeclareLaunchArgument(
        'experiment_seed',
        default_value=experiment_seed,
        description='Experiment seed recorded by the evaluator'
    )
    target_coverage_cmd = DeclareLaunchArgument(
        'target_coverage',
        default_value=target_coverage,
        description='Target map coverage ratio for evaluation'
    )
    coverage_hold_time_cmd = DeclareLaunchArgument(
        'coverage_hold_time',
        default_value=coverage_hold_time,
        description='Seconds target coverage must remain reached'
    )
    max_duration_cmd = DeclareLaunchArgument(
        'max_duration',
        default_value=max_duration,
        description='Maximum evaluation duration in seconds'
    )
    eval_output_root_cmd = DeclareLaunchArgument(
        'eval_output_root',
        default_value=eval_output_root,
        description='Directory where evaluator results are written'
    )

    # Map Generator 节点定义
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='log',
        parameters=[
            {'map/x_size': 36.0},
            {'map/y_size': 20.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 200},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 200},
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'sensing/radius': 5.0},
            {'sensing/rate': 1.0},
            {'min_distance': 1.2}
        ],
        condition = UnlessCondition(use_mockamap)
    )

    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='log',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.1},
            {'x_length': PythonExpression(['int(', map_size_x, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z, ')'])},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition = IfCondition(use_mockamap)
    )
    
    # 定义每个 drone 的配置
    drone_configs = [
        {'drone_id': 0, 'init_x': -20.0, 'init_y': -9.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 9.0, 'target_z': 1.0},
        {'drone_id': 1, 'init_x': -20.0, 'init_y': -7.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 7.0, 'target_z': 1.0},
        {'drone_id': 2, 'init_x': -20.0, 'init_y': -5.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 5.0, 'target_z': 1.0},
        {'drone_id': 3, 'init_x': -20.0, 'init_y': -3.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 3.0, 'target_z': 1.0}
        #{'drone_id': 4, 'init_x': -20.0, 'init_y': -1.0, 'init_z': 0.1, 'target_x': 20.0, 'target_y': 1.0, 'target_z': 1.0},
        #{'drone_id': 5, 'init_x': -20.0, 'init_y': 1.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -1.0, 'target_z': 1.0},
        #{'drone_id': 6, 'init_x': -20.0, 'init_y': 3.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -3.0, 'target_z': 1.0},
        #{'drone_id': 7, 'init_x': -20.0, 'init_y': 5.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -5.0, 'target_z': 1.0},
        #{'drone_id': 8, 'init_x': -20.0, 'init_y': 7.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -7.0, 'target_z': 1.0},
        #{'drone_id': 9, 'init_x': -20.0, 'init_y': 9.0,  'init_z': 0.1, 'target_x': 20.0, 'target_y': -9.0, 'target_z': 1.0}
    ]

    # 使用配置定义每个 drone 的 launch
    drone_nodes = []

    for config in drone_configs:        
        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 'launch', 'run_in_sim.launch.py')),
            launch_arguments={
                'use_dynamic_cmd': use_dynamic,
                'drone_id': str(config['drone_id']),
                'init_x': str(config['init_x']),
                'init_y': str(config['init_y']),
                'init_z': str(config['init_z']),
                'target_x': str(config['target_x']),
                'target_y': str(config['target_y']),
                'target_z': str(config['target_z']),
                'flight_type': '5',
                'map_size_x': map_size_x,
                'map_size_y': map_size_y,
                'map_size_z': map_size_z,
                'odom_topic': odom_topic
            }.items()
        )
        drone_nodes.append(drone_launch)

    exploration_goal_dummy_node = Node(
        package='exploration_swarm',
        executable='exploration_goal_dummy',
        name='exploration_goal_dummy',
        output='log',
        condition=IfCondition(use_dummy_goal),
        parameters=[
            {'publish_period': 5.0},
            {'frame_id': 'world'},
            {'map_size_x': map_size_x},
            {'map_size_y': map_size_y},
            {'map_size_z': map_size_z},
            {'map_margin': 1.0},
            {'min_z': 0.8}
        ]
    )

    swarm_exploration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('exploration_swarm'),
                'launch',
                'swarm_exploration_fame_ros2.launch.py')),
        condition=UnlessCondition(use_dummy_goal),
        launch_arguments={
            'map_source_type': 'pointcloud',
            'map_topic': '/swarm_exploration/occupancy_grid',
            'occupancy_grid_topic': '/swarm_exploration/occupancy_grid',
            'pointcloud_map_topic': '/map_generator/global_cloud',
            'local_cloud_topic_suffix': 'pcl_render_node/cloud',
            'grid_cloud_topic_suffix': 'grid/grid_map/occupancy_inflate',
            'odom_topic_suffix': odom_topic,
            'map_size_x': map_size_x,
            'map_size_y': map_size_y,
            'map_size_z': map_size_z,
            'enable_multi_drone_separation': enable_multi_drone_separation,
            'enable_hgrid': enable_hgrid,
            'enable_role_assignment': enable_role_assignment,
        }.items(),
    )

    rviz_config_path = os.path.join(get_package_share_directory('ego_planner'), 'launch', 'exp.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['--display-config', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    evaluation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('eval'),
                'launch',
                'exploration_time_evaluation.launch.py')),
        condition=IfCondition(run_evaluation),
        launch_arguments={
            'exploration_mode': exploration_mode,
            'experiment_seed': experiment_seed,
            'drone_num': str(len(drone_configs)),
            'map_topic': '/swarm_exploration/occupancy_grid',
            'odom_topic_format': odom_topic_format,
            'target_coverage': target_coverage,
            'coverage_hold_time': coverage_hold_time,
            'max_duration': max_duration,
            'output_root': eval_output_root,
        }.items(),
    )
        
    ld = LaunchDescription()
        
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(exploration_mode_arg)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_dummy_goal_cmd)
    ld.add_action(use_rviz_cmd)
    ld.add_action(run_evaluation_cmd)
    ld.add_action(experiment_seed_cmd)
    ld.add_action(target_coverage_cmd)
    ld.add_action(coverage_hold_time_cmd)
    ld.add_action(max_duration_cmd)
    ld.add_action(eval_output_root_cmd)

    # 添加 Map Generator 节点
    ld.add_action(map_generator_node)
    ld.add_action(mockamap_node)

    # 添加 Drone 节点
    for drone in drone_nodes:        
        ld.add_action(drone)

    ld.add_action(exploration_goal_dummy_node)
    ld.add_action(swarm_exploration_launch)
    ld.add_action(evaluation_launch)
    ld.add_action(rviz_node)

    return ld
