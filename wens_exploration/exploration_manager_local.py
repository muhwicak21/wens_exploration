#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from traj_utils.msg import MultiBsplines
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import math
from typing import Dict, List, Tuple, Optional
from .frontier_detector import FrontierDetector
from .pointcloud_converter import PointCloudToGridConverter


class DroneState:
    """Per-drone state tracker for local exploration"""
    
    MAX_VISITED_POSITIONS = 30  # Limit visited memory to prevent over-blocking
    STUCK_THRESHOLD = 50  # Force new goal after this many stuck cycles
    
    def __init__(self, drone_id: int):
        self.drone_id = drone_id
        self.local_map: Optional[OccupancyGrid] = None
        self.odometry: Optional[Odometry] = None
        self.current_goal: Optional[Tuple[float, float, float]] = None
        self.goal_assignment_time: float = 0.0
        self.visited_positions: List[Tuple[float, float, float]] = []
        self.last_frontiers: List[Tuple[float, float, float]] = []
        self.swarm_trajs: Optional[MultiBsplines] = None  # Other drones' trajectories
        self.last_swarm_update: float = 0.0
        self.explored_cells: set = set()  # Track (grid_x, grid_y) cells this drone has explored
        self.last_goal_failure_reason: str = ""  # Debug: why goal assignment failed
        self.consecutive_failures: int = 0  # Count consecutive goal assignment failures
        self.last_position: Optional[Tuple[float, float, float]] = None  # Track if stuck at same position
        self.position_stuck_count: int = 0  # Count how long stuck at same position
    
    def add_visited(self, position: Tuple[float, float, float]):
        """Add position to visited list, keeping only recent positions"""
        self.visited_positions.append(position)
        # Keep only the most recent positions to avoid over-blocking
        if len(self.visited_positions) > self.MAX_VISITED_POSITIONS:
            self.visited_positions = self.visited_positions[-self.MAX_VISITED_POSITIONS:]
    
    def clear_visited_if_stuck(self):
        """Clear most visited positions if stuck too long, keep only the most recent few."""
        keep = max(5, self.MAX_VISITED_POSITIONS // 6)  # Keep ~1/6 of max (≈5 of 30)
        if len(self.visited_positions) > keep:
            self.visited_positions = self.visited_positions[-keep:]
    
    def is_critically_stuck(self) -> bool:
        """Check if drone needs emergency intervention"""
        return self.position_stuck_count > self.STUCK_THRESHOLD
        
    def get_position(self) -> Optional[Tuple[float, float, float]]:
        """Get current drone position from odometry"""
        if self.odometry is None:
            return None
        pos = self.odometry.pose.pose.position
        return (pos.x, pos.y, pos.z)
    
    def get_yaw(self) -> Optional[float]:
        """Extract yaw angle from odometry quaternion"""
        if self.odometry is None:
            return None
        q = self.odometry.pose.pose.orientation
        # Convert quaternion to yaw angle
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def is_goal_reached(self, threshold: float) -> bool:
        """Check if drone reached current goal"""
        if self.current_goal is None or self.odometry is None:
            return False
        
        pos = self.get_position()
        if pos is None:
            return False
        
        dx = pos[0] - self.current_goal[0]
        dy = pos[1] - self.current_goal[1]
        dz = pos[2] - self.current_goal[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return distance < threshold
    
    def is_goal_timeout(self, current_time: float, timeout_sec: float) -> bool:
        """Check if goal assignment timed out"""
        if self.current_goal is None:
            return False
        return (current_time - self.goal_assignment_time) > timeout_sec
    
    def needs_new_goal(self, current_time: float, 
                      goal_reached_threshold: float,
                      goal_timeout_sec: float) -> bool:
        """Determine if drone needs a new goal"""
        if self.current_goal is None:
            return True
        if self.is_goal_reached(goal_reached_threshold):
            return True
        if self.is_goal_timeout(current_time, goal_timeout_sec):
            return True
        return False


class LocalExplorationManager(Node):
    """
    ROS2 Node for per-drone local frontier exploration.
    
    Each drone operates independently using only its local map.
    No global coordination, no RACER dynamics.
    """
    
    def __init__(self):
        super().__init__('exploration_manager_local')
        
        # Parameters
        self.declare_parameter('drone_ids', [0, 1, 2, 3])
        self.declare_parameter('update_rate', 10.0)  # 10Hz for faster response
        self.declare_parameter('goal_timeout_sec', 8.0)
        self.declare_parameter('goal_reached_threshold', 0.6)
        self.declare_parameter('min_frontier_size', 3)
        self.declare_parameter('safe_neighborhood', 1)
        self.declare_parameter('free_threshold', 70)
        self.declare_parameter('occupied_threshold', 80)
        self.declare_parameter('random_fallback_enabled', True)
        self.declare_parameter('random_range', [2.0, 6.0])
        self.declare_parameter('visited_memory_radius', 1.5)
        self.declare_parameter('frontier_min_distance', 0.5)
        self.declare_parameter('orientation_filter_enabled', True)
        self.declare_parameter('forward_cone_angle', 135.0)  # degrees, ±67.5° from forward
        self.declare_parameter('obstacle_safety_margin', 0.25)  # 0.25m clearance from obstacles (reduced for better goal finding)
        self.declare_parameter('map_x_size', 36.0)  # Map boundaries from simulator
        self.declare_parameter('map_y_size', 20.0)
        self.declare_parameter('map_z_size', 3.0)
        self.declare_parameter('map_boundary_margin', 3.0)  # Increased to accommodate spawn positions outside previous bounds
        self.declare_parameter('map_z_min', 0.5)  # Minimum safe flight altitude
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('drone_safety_radius', 1.0)  # Keep 3m distance from other drones
        self.declare_parameter('swarm_traj_timeout', 1.0)  # Consider swarm traj stale after 1 sec
        
        # Swarm coordination parameters
        self.declare_parameter('swarm_coordination_enabled', True)  # Enable shared visited positions
        self.declare_parameter('swarm_visited_radius', 2.5)  # Radius to consider position as visited by swarm
        self.declare_parameter('swarm_goal_conflict_radius', 3.0)  # Min distance between drone goals
        self.declare_parameter('swarm_share_interval', 0.5)  # Share visited positions every 0.5 seconds
        
        # Global map frontier detection parameters
        self.declare_parameter('use_global_frontiers', True)  # Use global map for frontier detection
        self.declare_parameter('global_safety_check', True)  # Check global map for obstacle avoidance
        
        # Topic name patterns ({drone_id} will be substituted at runtime)
        self.declare_parameter('topic_map', '/drone_{drone_id}/planner/grid/grid_map/occupancy_inflate')
        self.declare_parameter('topic_odom', '/drone_{drone_id}/localization/odometry')
        self.declare_parameter('topic_swarm_trajs', '/drone_{drone_id}/planner/planning/swarm_trajs')
        self.declare_parameter('topic_goal', '/drone_{drone_id}/move_base_simple/goal')
        
        # Get parameters
        self.drone_ids = self.get_parameter('drone_ids').value
        self.update_rate = self.get_parameter('update_rate').value
        self.goal_timeout_sec = self.get_parameter('goal_timeout_sec').value
        self.goal_reached_threshold = self.get_parameter('goal_reached_threshold').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.safe_neighborhood = self.get_parameter('safe_neighborhood').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.random_fallback_enabled = self.get_parameter('random_fallback_enabled').value
        self.random_range = self.get_parameter('random_range').value
        self.visited_memory_radius = self.get_parameter('visited_memory_radius').value
        self.frontier_min_distance = self.get_parameter('frontier_min_distance').value
        self.orientation_filter_enabled = self.get_parameter('orientation_filter_enabled').value
        self.forward_cone_angle = self.get_parameter('forward_cone_angle').value
        self.obstacle_safety_margin = self.get_parameter('obstacle_safety_margin').value
        self.map_x_size = self.get_parameter('map_x_size').value
        self.map_y_size = self.get_parameter('map_y_size').value
        self.map_z_size = self.get_parameter('map_z_size').value
        self.map_boundary_margin = self.get_parameter('map_boundary_margin').value
        self.map_z_floor = self.get_parameter('map_z_min').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.drone_safety_radius = self.get_parameter('drone_safety_radius').value
        self.swarm_traj_timeout = self.get_parameter('swarm_traj_timeout').value
        
        # Swarm coordination parameters
        self.swarm_coordination_enabled = self.get_parameter('swarm_coordination_enabled').value
        self.swarm_visited_radius = self.get_parameter('swarm_visited_radius').value
        self.swarm_goal_conflict_radius = self.get_parameter('swarm_goal_conflict_radius').value
        self.swarm_share_interval = self.get_parameter('swarm_share_interval').value
        
        # Global map frontier detection parameters
        self.use_global_frontiers = self.get_parameter('use_global_frontiers').value
        self.global_safety_check = self.get_parameter('global_safety_check').value
        
        # Topic name patterns
        self.topic_map_pattern = self.get_parameter('topic_map').value
        self.topic_odom_pattern = self.get_parameter('topic_odom').value
        self.topic_swarm_trajs_pattern = self.get_parameter('topic_swarm_trajs').value
        self.topic_goal_pattern = self.get_parameter('topic_goal').value
        
        # Calculate map boundaries (map is centered at origin)
        self.map_x_min = -self.map_x_size / 2.0 + self.map_boundary_margin
        self.map_x_max = self.map_x_size / 2.0 - self.map_boundary_margin
        self.map_y_min = -self.map_y_size / 2.0 + self.map_boundary_margin
        self.map_y_max = self.map_y_size / 2.0 - self.map_boundary_margin
        self.map_z_min = self.map_z_floor  # Minimum safe altitude (from parameter)
        self.map_z_max = self.map_z_size - 0.5
        
        # Per-drone state
        self.drones: Dict[int, DroneState] = {
            drone_id: DroneState(drone_id) for drone_id in self.drone_ids
        }
        
        # Frontier detector and converter
        self.frontier_detector = FrontierDetector(
            min_frontier_size=self.min_frontier_size,
            unknown_value=-1,
            free_max=self.free_threshold
        )
        self.pc_converter = PointCloudToGridConverter()
        
        # QoS profiles
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers and Publishers for each drone
        self.map_subscribers = {}
        self.odom_subscribers = {}
        self.swarm_traj_subscribers = {}
        self.goal_publishers = {}
        self.marker_publishers = {}
        
        # Exploration state tracking
        self.drones_with_goals_count = 0  # Track how many drones got goals this cycle
        self.consecutive_no_goals_cycles = 0  # Count cycles where NO drones got goals
        self.last_coverage_report_time = 0.0  # For periodic coverage reporting
        self.coverage_report_interval = 5.0  # Report coverage every 5 seconds
        
        # Persistent global map storage - accumulates data over time
        self.global_map_resolution = 0.15
        self.global_map_width = int(self.map_x_size / self.global_map_resolution)
        self.global_map_height = int(self.map_y_size / self.global_map_resolution)
        self.persistent_merged_data = np.full(
            self.global_map_width * self.global_map_height, -1, dtype=np.int8
        )
        
        # Swarm coordination: Global visited positions from all drones
        # Key: drone_id, Value: list of (x, y, z) positions + current goal
        self.swarm_visited: Dict[int, List[Tuple[float, float, float]]] = {}
        self.swarm_goals: Dict[int, Optional[Tuple[float, float, float]]] = {}
        self.last_swarm_share_time: float = 0.0
        
        for drone_id in self.drone_ids:
            # Subscribe to local inflated map (PointCloud2 from EgoPlanner)
            self.map_subscribers[drone_id] = self.create_subscription(
                PointCloud2,
                self.topic_map_pattern.format(drone_id=drone_id),
                lambda msg, d_id=drone_id: self._map_callback(msg, d_id),
                qos_sensor
            )
            
            # Subscribe to odometry
            self.odom_subscribers[drone_id] = self.create_subscription(
                Odometry,
                self.topic_odom_pattern.format(drone_id=drone_id),
                lambda msg, d_id=drone_id: self._odom_callback(msg, d_id),
                qos_sensor
            )
            
            # Subscribe to swarm trajectories (other drones' planned paths)
            # Use RELIABLE QoS to match the ego_planner publisher's QoS
            self.swarm_traj_subscribers[drone_id] = self.create_subscription(
                MultiBsplines,
                self.topic_swarm_trajs_pattern.format(drone_id=drone_id),
                lambda msg, d_id=drone_id: self._swarm_traj_callback(msg, d_id),
                qos_reliable
            )
            
            # Publisher for goals
            self.goal_publishers[drone_id] = self.create_publisher(
                PoseStamped,
                self.topic_goal_pattern.format(drone_id=drone_id),
                qos_reliable
            )
            
            # Debug marker publishers
            if self.debug_mode:
                self.marker_publishers[f'local_{drone_id}'] = self.create_publisher(
                    MarkerArray,
                    f'/frontier_local_{drone_id}',
                    qos_reliable
                )
                self.marker_publishers[f'chosen_{drone_id}'] = self.create_publisher(
                    MarkerArray,
                    f'/frontier_chosen_{drone_id}',
                    qos_reliable
                )
                self.marker_publishers[f'rejected_{drone_id}'] = self.create_publisher(
                    MarkerArray,
                    f'/frontier_rejected_{drone_id}',
                    qos_reliable
                )
        
        # Swarm coordination: Publisher and Subscribers for sharing visited positions
        if self.swarm_coordination_enabled:
            # Each drone publishes its visited positions
            self.swarm_visited_publishers: Dict[int, any] = {}
            self.swarm_visited_subscribers: Dict[int, any] = {}
            
            for drone_id in self.drone_ids:
                # Publisher for this drone's visited positions
                self.swarm_visited_publishers[drone_id] = self.create_publisher(
                    PoseArray,
                    f'/exploration/drone_{drone_id}/visited',
                    qos_reliable
                )
            
            for drone_id in self.drone_ids:
                # Subscribe to OTHER drones' visited positions only (skip self to avoid loopback)
                for other_id in self.drone_ids:
                    if other_id == drone_id:
                        continue  # Don't subscribe to own topic
                    sub_key = (drone_id, other_id)
                    if sub_key not in self.swarm_visited_subscribers:
                        self.swarm_visited_subscribers[sub_key] = self.create_subscription(
                            PoseArray,
                            f'/exploration/drone_{other_id}/visited',
                            lambda msg, d_id=other_id: self._swarm_visited_callback(msg, d_id),
                            qos_sensor
                        )
        
        # Global merged map publisher for visualization
        self.merged_map_publisher = self.create_publisher(
            OccupancyGrid,
            '/global_merged_map',
            qos_reliable
        )
        
        # Main exploration timer
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self._exploration_loop)
        
        # Merged map publishing timer (1Hz for good visualization without overhead)
        self.merged_map_timer = self.create_timer(1.0, self._publish_merged_map)
        
        self.get_logger().info(
            f'Local Exploration Manager started for drones {self.drone_ids}')
        self.get_logger().info(
            f'Update rate: {self.update_rate} Hz, Goal timeout: {self.goal_timeout_sec}s')
    
    def _map_callback(self, msg: PointCloud2, drone_id: int):
        """Receive and convert PointCloud2 map from EgoPlanner to OccupancyGrid"""
        try:
            # Get drone position for grid centering
            drone_pos = self.drones[drone_id].get_position()
            if drone_pos is None:
                if self.debug_mode:
                    self.get_logger().warn(f'Drone {drone_id} no odometry yet, skipping map')
                return
            # Convert PointCloud2 to OccupancyGrid
            grid = self.pc_converter.convert(msg, drone_pos)
            self.drones[drone_id].local_map = grid
            if self.debug_mode:
                self.get_logger().info(f'Drone {drone_id} map updated: {grid.info.width}x{grid.info.height} grid')
        except Exception as e:
            if self.debug_mode:
                self.get_logger().warn(f'Drone {drone_id} map conversion failed: {e}')
    
    def _odom_callback(self, msg: Odometry, drone_id: int):
        """Receive odometry update"""
        self.drones[drone_id].odometry = msg
    
    def _swarm_traj_callback(self, msg: MultiBsplines, drone_id: int):
        """Receive swarm trajectory updates (other drones' planned paths)"""
        drone = self.drones[drone_id]
        drone.swarm_trajs = msg
        drone.last_swarm_update = self.get_clock().now().nanoseconds / 1e9
        
        if self.debug_mode and len(msg.traj) > 0:
            self.get_logger().info(
                f'Drone {drone_id}: Received {len(msg.traj)} swarm trajectories')
    
    def _swarm_visited_callback(self, msg: PoseArray, drone_id: int):
        """Receive visited positions from another drone"""
        if not self.swarm_coordination_enabled:
            return
        
        # Convert PoseArray to list of positions
        positions = []
        for pose in msg.poses:
            positions.append((pose.position.x, pose.position.y, pose.position.z))
        
        # Extract current goal: last pose is the goal, marked with orientation.z=1.0, w=0.0
        if len(msg.poses) > 0:
            last_pose = msg.poses[-1]
            if last_pose.orientation.z == 1.0 and last_pose.orientation.w == 0.0:
                self.swarm_goals[drone_id] = (
                    last_pose.position.x,
                    last_pose.position.y,
                    last_pose.position.z
                )
                # Don't include the goal pose in the visited positions list
                positions = positions[:-1]
        
        # Store in swarm visited dictionary (AFTER removing goal pose)
        self.swarm_visited[drone_id] = positions
    
    def _publish_swarm_visited(self, drone_id: int):
        """Publish this drone's visited positions and current goal for swarm coordination"""
        if not self.swarm_coordination_enabled:
            return
        
        drone = self.drones[drone_id]
        
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Add visited positions (orientation.w = 1.0 = valid unit quaternion)
        for pos in drone.visited_positions:
            pose = Pose()
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = pos[2]
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        # Also add current position as visited
        current_pos = drone.get_position()
        if current_pos is not None:
            pose = Pose()
            pose.position.x = current_pos[0]
            pose.position.y = current_pos[1]
            pose.position.z = current_pos[2]
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        # Append current goal as the LAST pose, marked with orientation.z = 1.0
        # (orientation.x=0, y=0, z=1, w=0 is a distinguishable non-unit marker)
        if drone.current_goal is not None:
            goal_pose = Pose()
            goal_pose.position.x = drone.current_goal[0]
            goal_pose.position.y = drone.current_goal[1]
            goal_pose.position.z = drone.current_goal[2]
            goal_pose.orientation.z = 1.0  # Sentinel: last pose is current goal
            goal_pose.orientation.w = 0.0
            msg.poses.append(goal_pose)
        
        # Publish
        if drone_id in self.swarm_visited_publishers:
            self.swarm_visited_publishers[drone_id].publish(msg)
    
    def _is_visited_by_swarm(self, position: Tuple[float, float, float], 
                             exclude_drone_id: int) -> bool:
        """Check if position was visited by any OTHER drone in the swarm"""
        if not self.swarm_coordination_enabled:
            return False
        
        x, y, z = position
        
        for drone_id, visited_list in self.swarm_visited.items():
            if drone_id == exclude_drone_id:
                continue  # Skip self
            
            for visited_pos in visited_list:
                dx = x - visited_pos[0]
                dy = y - visited_pos[1]
                distance = math.sqrt(dx*dx + dy*dy)
                if distance < self.swarm_visited_radius:
                    return True
        
        return False
    
    def _is_goal_conflict_with_swarm(self, goal: Tuple[float, float, float],
                                     exclude_drone_id: int) -> bool:
        """Check if goal conflicts with another drone's current goal"""
        if not self.swarm_coordination_enabled:
            return False
        
        x, y, z = goal
        
        # Check against other drones' goals
        for drone_id, other_goal in self.swarm_goals.items():
            if drone_id == exclude_drone_id:
                continue
            
            if other_goal is None:
                continue
            
            dx = x - other_goal[0]
            dy = y - other_goal[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < self.swarm_goal_conflict_radius:
                return True
        
        # Also check local drone goals for same-cycle conflict detection
        # (swarm_goals may not be updated yet within the same exploration cycle)
        for drone_id, drone in self.drones.items():
            if drone_id == exclude_drone_id:
                continue
            if drone.current_goal is None:
                continue
            # Skip if already checked via swarm_goals
            if drone_id in self.swarm_goals and self.swarm_goals[drone_id] is not None:
                continue
            dx = x - drone.current_goal[0]
            dy = y - drone.current_goal[1]
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < self.swarm_goal_conflict_radius:
                return True
        
        return False

    def _detect_global_frontiers(self, drone: DroneState) -> List[Tuple[float, float, float]]:
        """
        Detect frontiers from the GLOBAL persistent map instead of local map.
        This ensures drones don't revisit areas explored by any drone.
        
        A global frontier is a cell that is:
        1. Free in global map (explored and safe)
        2. Adjacent to unknown cells in global map (unexplored territory)
        3. Not too close to obstacles
        
        Returns:
            List of frontier centroids in world coordinates
        """
        # Use persistent global map data
        if self.persistent_merged_data is None or len(self.persistent_merged_data) == 0:
            return []
        
        width = self.global_map_width
        height = self.global_map_height
        resolution = self.global_map_resolution
        
        # Reshape to 2D grid
        try:
            data = self.persistent_merged_data.reshape((height, width))
        except ValueError:
            return []
        
        # Define thresholds
        FREE_MAX = 50
        OCCUPIED_MIN = 65
        UNKNOWN = -1
        
        # Create masks
        free_mask = (data >= 0) & (data < FREE_MAX)
        unknown_mask = (data == UNKNOWN)
        occupied_mask = (data >= OCCUPIED_MIN)
        
        # Count unknown and known cells
        unknown_count = np.sum(unknown_mask)
        
        if unknown_count == 0:
            if self.debug_mode:
                self.get_logger().info(f'Drone {drone.drone_id}: Global map fully explored!')
            return []
        
        # Vectorized 8-connectivity dilation using zero-padded slicing (no edge wrapping)
        def dilate_np(mask: np.ndarray) -> np.ndarray:
            h, w = mask.shape
            padded = np.zeros((h + 2, w + 2), dtype=bool)
            padded[1:-1, 1:-1] = mask
            result = mask.copy()
            for di in range(3):
                for dj in range(3):
                    if di == 1 and dj == 1:
                        continue
                    result |= padded[di:di+h, dj:dj+w]
            return result
        
        unknown_dilated = dilate_np(unknown_mask)
        occupied_dilated = dilate_np(occupied_mask)
        occupied_dilated = dilate_np(occupied_dilated)  # Double dilation for safety
        
        # Global frontier: free, adjacent to unknown, not near obstacles
        frontier_mask = free_mask & unknown_dilated & ~occupied_dilated
        
        if not np.any(frontier_mask):
            return []
        
        # Find frontier cells and cluster them
        frontier_indices = np.argwhere(frontier_mask)
        
        if len(frontier_indices) == 0:
            return []
        
        # Simple clustering: group nearby frontier cells
        # Use a grid-based approach for efficiency
        cluster_resolution = 3.0  # meters - cluster frontiers within this distance
        cluster_grid_size = int(cluster_resolution / resolution)
        
        clusters = {}  # (cluster_row, cluster_col) -> list of frontier indices
        
        for row, col in frontier_indices:
            cluster_key = (row // cluster_grid_size, col // cluster_grid_size)
            if cluster_key not in clusters:
                clusters[cluster_key] = []
            clusters[cluster_key].append((row, col))
        
        # Convert cluster centroids to world coordinates
        centroids = []
        min_cluster_size = 3  # Minimum cells for a valid frontier
        
        for cluster_key, cells in clusters.items():
            if len(cells) < min_cluster_size:
                continue
            
            # Calculate centroid
            avg_row = np.mean([c[0] for c in cells])
            avg_col = np.mean([c[1] for c in cells])
            
            # Snap to nearest frontier cell
            best_dist = float('inf')
            snap_row, snap_col = cells[0]
            for row, col in cells:
                dist = (row - avg_row)**2 + (col - avg_col)**2
                if dist < best_dist:
                    best_dist = dist
                    snap_row, snap_col = row, col
            
            # Convert to world coordinates
            # Global map origin is at (-map_x_size/2, -map_y_size/2)
            world_x = -self.map_x_size / 2.0 + snap_col * resolution
            world_y = -self.map_y_size / 2.0 + snap_row * resolution
            world_z = 1.0  # Default flight height
            
            centroids.append((world_x, world_y, world_z))
        
        return centroids

    def _is_explored_globally(self, position: Tuple[float, float, float]) -> bool:
        """
        Check if a position has been explored in the global map.
        
        Returns True if the cell is known (free or occupied), False if unknown.
        """
        if self.persistent_merged_data is None:
            return False
        
        x, y, z = position
        resolution = self.global_map_resolution
        width = self.global_map_width
        height = self.global_map_height
        
        # Convert to grid coordinates
        col = int((x + self.map_x_size / 2.0) / resolution)
        row = int((y + self.map_y_size / 2.0) / resolution)
        
        if not (0 <= col < width and 0 <= row < height):
            return False  # Outside map bounds
        
        idx = row * width + col
        return self.persistent_merged_data[idx] != -1  # Not unknown = explored

    def _is_safe_in_global_map(self, position: Tuple[float, float, float], 
                               safety_margin: float = 0.5) -> bool:
        """
        Check if position is safe (free space) in the global map with safety margin.
        
        Args:
            position: (x, y, z) world coordinates
            safety_margin: Minimum distance from obstacles in meters
            
        Returns:
            True if position is in free space with adequate clearance
        """
        if self.persistent_merged_data is None:
            return True  # No map yet, assume safe
        
        x, y, z = position
        resolution = self.global_map_resolution
        width = self.global_map_width
        height = self.global_map_height
        
        OCCUPIED_MIN = 65
        
        # Convert to grid coordinates
        col = int((x + self.map_x_size / 2.0) / resolution)
        row = int((y + self.map_y_size / 2.0) / resolution)
        
        if not (0 <= col < width and 0 <= row < height):
            return False  # Outside map bounds
        
        # Check center cell
        idx = row * width + col
        if self.persistent_merged_data[idx] >= OCCUPIED_MIN:
            return False  # Occupied
        
        # Check safety margin around position
        margin_cells = int(safety_margin / resolution) + 1
        
        for dr in range(-margin_cells, margin_cells + 1):
            for dc in range(-margin_cells, margin_cells + 1):
                nr, nc = row + dr, col + dc
                if not (0 <= nc < width and 0 <= nr < height):
                    continue
                    
                nidx = nr * width + nc
                if self.persistent_merged_data[nidx] >= OCCUPIED_MIN:
                    # Check actual distance
                    cell_x = -self.map_x_size / 2.0 + nc * resolution
                    cell_y = -self.map_y_size / 2.0 + nr * resolution
                    dist = math.sqrt((x - cell_x)**2 + (y - cell_y)**2)
                    if dist < safety_margin:
                        return False  # Too close to obstacle
        
        return True

    def _exploration_loop(self):
        """Main exploration loop - process each drone independently"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Reset cycle counters
        self.drones_with_goals_count = 0
        active_drones_count = 0
        
        # Periodically publish swarm visited positions
        if self.swarm_coordination_enabled:
            if current_time - self.last_swarm_share_time > self.swarm_share_interval:
                for drone_id in self.drone_ids:
                    self._publish_swarm_visited(drone_id)
                self.last_swarm_share_time = current_time
        
        if self.debug_mode:
            active_drones = [i for i in self.drone_ids if self.drones[i].local_map is not None]
            if active_drones:
                self.get_logger().info(f'\\n=== Exploration Cycle ===')
                self.get_logger().info(f'Processing drones: {active_drones}')
        
        for drone_id in self.drone_ids:
            drone = self.drones[drone_id]
            
            # Skip if missing data
            if drone.local_map is None or drone.odometry is None:
                if self.debug_mode and drone.local_map is None:
                    self.get_logger().warn(f'Drone {drone_id}: NO MAP (skipping)')
                elif self.debug_mode and drone.odometry is None:
                    self.get_logger().warn(f'Drone {drone_id}: NO ODOM (skipping)')
                continue
            
            active_drones_count += 1
            
            # Check if drone needs new goal
            needs_goal = drone.needs_new_goal(current_time, 
                                              self.goal_reached_threshold,
                                              self.goal_timeout_sec)
            
            # Check if drone is stuck at same position
            current_pos = drone.get_position()
            if current_pos is not None and drone.last_position is not None:
                dx = current_pos[0] - drone.last_position[0]
                dy = current_pos[1] - drone.last_position[1]
                dz = current_pos[2] - drone.last_position[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist < 0.1:  # Less than 10cm movement
                    drone.position_stuck_count += 1
                    if self.debug_mode and drone.position_stuck_count > 20:
                        self.get_logger().warn(
                            f'Drone {drone_id}: STUCK at position {current_pos} for {drone.position_stuck_count} cycles!')
                else:
                    drone.position_stuck_count = 0
            drone.last_position = current_pos
            
            # Force new goal if critically stuck
            if drone.is_critically_stuck():
                if self.debug_mode:
                    self.get_logger().warn(
                        f'Drone {drone_id}: CRITICALLY STUCK - forcing new goal and clearing visited!')
                drone.clear_visited_if_stuck()
                drone.position_stuck_count = 0
                drone.current_goal = None  # Force reassignment
                needs_goal = True
            
            if not needs_goal:
                continue
            
            # Mark old goal as visited if reached
            if drone.current_goal is not None and drone.is_goal_reached(self.goal_reached_threshold):
                drone.add_visited(drone.current_goal)
                if self.debug_mode:
                    self.get_logger().info(f'Drone {drone_id} reached goal {drone.current_goal}')
            
            # Detect frontiers in local map
            if drone.local_map is None:
                if self.debug_mode:
                    self.get_logger().warn(f'Drone {drone_id}: No local map yet')
                continue
            
            # Detect frontiers based on configuration
            if self.use_global_frontiers:
                # PRIMARY: Detect frontiers from GLOBAL map (prevents revisiting explored areas)
                global_frontiers = self._detect_global_frontiers(drone)
                
                # SECONDARY: Detect local frontiers as fallback
                local_frontiers = self.frontier_detector.detect_frontiers(drone.local_map)
                
                # Use global frontiers if available, otherwise fall back to local
                if len(global_frontiers) > 0:
                    frontiers = global_frontiers
                    frontier_source = "GLOBAL"
                else:
                    frontiers = local_frontiers
                    frontier_source = "LOCAL"
                
                if self.debug_mode:
                    self.get_logger().info(
                        f'Drone {drone_id}: {len(frontiers)} raw frontiers ({frontier_source}) '
                        f'[global={len(global_frontiers)}, local={len(local_frontiers)}]')
            else:
                # Use only local frontiers (original behavior)
                frontiers = self.frontier_detector.detect_frontiers(drone.local_map)
                frontier_source = "LOCAL"
                
                if self.debug_mode:
                    self.get_logger().info(f'Drone {drone_id}: {len(frontiers)} raw frontiers (LOCAL only)')
            
            # Frontiers are already in world coordinates from FrontierDetector
            frontier_positions = frontiers
            
            # Filter frontiers (safety + visited); viewpoint_cache avoids recomputation
            safe_frontiers, rejected_frontiers, viewpoint_cache = self._filter_frontiers(
                frontier_positions, drone)
            
            drone.last_frontiers = frontier_positions
            
            if self.debug_mode:
                self.get_logger().info(
                    f'Drone {drone_id}: {len(safe_frontiers)}/{len(frontier_positions)} '
                    f'frontiers SAFE (rejected={len(rejected_frontiers)})')
            
            # Orientation filter: remove back-side frontiers
            forward_frontiers = self._filter_by_orientation(safe_frontiers, drone)
            
            if self.debug_mode and len(safe_frontiers) > 0:
                self.get_logger().info(
                    f'Drone {drone_id}: {len(forward_frontiers)}/{len(safe_frontiers)} '
                    f'frontiers FORWARD-FACING (removed {len(safe_frontiers)-len(forward_frontiers)} backward)')
            
            # Select nearest frontier from forward-facing ones (uses cached viewpoints)
            goal = self._select_nearest_frontier(forward_frontiers, drone, viewpoint_cache)
            
            # Check for goal conflict with other drones
            if goal is not None and self._is_goal_conflict_with_swarm(goal, drone_id):
                if self.debug_mode:
                    self.get_logger().info(
                        f'Drone {drone_id}: Goal {goal} conflicts with another drone, selecting alternative')
                # Try to find a non-conflicting goal from remaining frontiers (use cache)
                goal = None
                for frontier in forward_frontiers:
                    viewpoint = viewpoint_cache.get(frontier) or self._generate_viewpoint_goal(frontier, drone)
                    if viewpoint is not None and not self._is_goal_conflict_with_swarm(viewpoint, drone_id):
                        goal = viewpoint
                        break
            
            # FALLBACK 1: If no forward frontiers, use any safe frontier (allow turning)
            if goal is None and len(safe_frontiers) > 0:
                goal = self._select_nearest_frontier(safe_frontiers, drone, viewpoint_cache)
                # Also check for conflict
                if goal is not None and self._is_goal_conflict_with_swarm(goal, drone_id):
                    goal = None
                    for frontier in safe_frontiers:
                        viewpoint = viewpoint_cache.get(frontier) or self._generate_viewpoint_goal(frontier, drone)
                        if viewpoint is not None and not self._is_goal_conflict_with_swarm(viewpoint, drone_id):
                            goal = viewpoint
                            break
                if self.debug_mode and goal is not None:
                    self.get_logger().info(
                        f'Drone {drone_id}: BACKWARD FALLBACK goal {goal} (drone will turn)')
            
            # FALLBACK 2: Random if no usable frontiers (none detected OR all rejected)
            if goal is None and self.random_fallback_enabled:
                goal = self._generate_random_local_goal(drone)
                if self.debug_mode and goal is not None:
                    self.get_logger().info(
                        f'Drone {drone_id}: RANDOM FALLBACK goal {goal}')
            
            # Publish goal
            if goal is not None:
                self._publish_goal(drone_id, goal, current_time)
                self.drones_with_goals_count += 1
                
                # Update swarm goals locally for immediate conflict detection
                self.swarm_goals[drone_id] = goal
                
                # Mark cells around goal as explored
                self._mark_explored(drone_id, goal)
                
                # Visualize
                if self.debug_mode:
                    self._publish_debug_markers(drone_id, frontier_positions, 
                                               safe_frontiers, rejected_frontiers, goal)
            else:
                # Detailed failure reason logging
                if self.debug_mode:
                    reason = f'NO GOAL: '
                    if len(frontier_positions) == 0:
                        reason += 'No frontiers detected '
                    elif len(safe_frontiers) == 0:
                        reason += f'{len(rejected_frontiers)} frontiers rejected (visited/unsafe/collision) '
                    elif len(forward_frontiers) == 0:
                        reason += f'{len(safe_frontiers)} frontiers behind drone (orientation filter) '
                    else:
                        reason += f'{len(forward_frontiers)} frontiers available but selection failed '
                    
                    drone.last_goal_failure_reason = reason
                    self.get_logger().warn(f'Drone {drone_id}: {reason}')
        
        # Detect when ALL drones stop getting goals
        if active_drones_count > 0 and self.drones_with_goals_count == 0:
            self.consecutive_no_goals_cycles += 1
            
            # Update consecutive failure counts for each drone
            for drone_id, drone in self.drones.items():
                if drone.local_map is not None and drone.odometry is not None:
                    drone.consecutive_failures += 1
            
            if self.debug_mode:
                self.get_logger().error(
                    f'⚠️  ALL {active_drones_count} DRONES STOPPED! '
                    f'(cycle {self.consecutive_no_goals_cycles} with no goals)\n'
                    f'Reasons: ' + 
                    ' | '.join([f'D{d_id}: {d.last_goal_failure_reason} (failures={d.consecutive_failures})' 
                               for d_id, d in self.drones.items() 
                               if d.local_map is not None and d.odometry is not None]))
                
                # Every 5 cycles of being stuck, log detailed analysis
                if self.consecutive_no_goals_cycles % 5 == 0:
                    self._log_stuck_analysis()
        else:
            self.consecutive_no_goals_cycles = 0
            # Reset failure counts for drones that got goals
            for drone_id, drone in self.drones.items():
                if drone.current_goal is not None:
                    drone.consecutive_failures = 0
            
            if self.debug_mode and self.drones_with_goals_count > 0:
                self.get_logger().info(
                    f'✓ {self.drones_with_goals_count}/{active_drones_count} drones got new goals')
        
        # Periodic coverage statistics reporting
        if self.debug_mode:
            if current_time - self.last_coverage_report_time > self.coverage_report_interval:
                self._log_coverage_statistics()
                self.last_coverage_report_time = current_time
    
    
    def _filter_frontiers(self, frontiers: List[Tuple[float, float, float]], 
                         drone: DroneState) -> Tuple[List[Tuple[float, float, float]], 
                                                      List[Tuple[float, float, float]],
                                                      dict]:
        """
        Filter frontiers based on safety and visited memory.
        Note: Safety is checked on the resulting viewpoint, not the frontier itself
        (since frontiers are at obstacle boundaries by definition).
        
        Returns:
            (safe_frontiers, rejected_frontiers, viewpoint_cache)
            viewpoint_cache maps frontier tuple -> precomputed viewpoint tuple
        """
        safe = []
        rejected = []
        viewpoint_cache: dict = {}  # frontier -> viewpoint, avoids recomputing
        swarm_rejected_count = 0
        
        for frontier in frontiers:
            # Check if too close to visited positions (self)
            if self._is_visited(frontier, drone.visited_positions):
                rejected.append(frontier)
                continue
            
            # Check if too close to positions visited by OTHER drones (swarm coordination)
            if self._is_visited_by_swarm(frontier, drone.drone_id):
                rejected.append(frontier)
                swarm_rejected_count += 1
                continue
            
            # Check if too close to current goal
            if drone.current_goal is not None:
                dx = frontier[0] - drone.current_goal[0]
                dy = frontier[1] - drone.current_goal[1]
                dist = math.sqrt(dx*dx + dy*dy)
                if dist < self.frontier_min_distance:
                    rejected.append(frontier)
                    continue
            
            # Generate viewpoint for this frontier (cached to avoid recomputation later)
            viewpoint = self._generate_viewpoint_goal(frontier, drone)
            if viewpoint is None:
                rejected.append(frontier)
                continue
            
            # Check viewpoint safety (not frontier, since frontier is at obstacle boundary)
            if not self._is_local_safe(viewpoint, drone.local_map):
                rejected.append(frontier)
                continue
            
            # Check viewpoint safety in GLOBAL map (prevents hitting known obstacles)
            if self.global_safety_check and not self._is_safe_in_global_map(viewpoint, safety_margin=0.3):
                rejected.append(frontier)
                continue
            
            # NOTE: _is_explored_globally check REMOVED because frontiers ARE in explored space
            # (by definition: frontier = free/explored cell adjacent to unknown/unexplored cell)
            
            # Check collision with other drones' trajectories
            if self._is_collision_with_swarm(viewpoint, drone):
                rejected.append(frontier)
                continue
            
            safe.append(frontier)
            viewpoint_cache[frontier] = viewpoint  # Cache for reuse
        
        if self.debug_mode and swarm_rejected_count > 0:
            self.get_logger().info(
                f'Drone {drone.drone_id}: {swarm_rejected_count} frontiers rejected (explored by other drones)')
        
        return safe, rejected, viewpoint_cache
    
    def _filter_by_orientation(self, frontiers: List[Tuple[float, float, float]],
                              drone: DroneState) -> List[Tuple[float, float, float]]:
        """
        Filter frontiers based on drone orientation.
        Remove frontiers that are behind the drone (back-side filtering).
        
        Args:
            frontiers: List of frontier positions
            drone: Drone state with odometry
            
        Returns:
            List of frontiers in forward hemisphere
        """
        if not self.orientation_filter_enabled:
            return frontiers
        
        drone_pos = drone.get_position()
        drone_yaw = drone.get_yaw()
        
        if drone_pos is None or drone_yaw is None:
            # No orientation info, return all frontiers
            return frontiers
        
        forward_frontiers = []
        half_cone_rad = math.radians(self.forward_cone_angle / 2.0)
        
        for frontier in frontiers:
            # Vector from drone to frontier
            dx = frontier[0] - drone_pos[0]
            dy = frontier[1] - drone_pos[1]
            
            # Angle to frontier
            angle_to_frontier = math.atan2(dy, dx)
            
            # Angular difference from drone heading
            angle_diff = angle_to_frontier - drone_yaw
            
            # Normalize to [-π, π]
            while angle_diff > math.pi:
                angle_diff -= 2.0 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2.0 * math.pi
            
            # Check if within forward cone
            if abs(angle_diff) <= half_cone_rad:
                forward_frontiers.append(frontier)
        
        return forward_frontiers
    
    def _is_visited(self, position: Tuple[float, float, float],
                   visited: List[Tuple[float, float, float]]) -> bool:
        """Check if position was already visited"""
        for visited_pos in visited:
            dx = position[0] - visited_pos[0]
            dy = position[1] - visited_pos[1]
            dz = position[2] - visited_pos[2]
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            if distance < self.visited_memory_radius:
                return True
        return False
    
    def _is_collision_with_swarm(self, position: Tuple[float, float, float],
                                drone: DroneState) -> bool:
        """
        Check if position would collide with other drones' planned trajectories.
        Uses swarm_trajs data from EgoPlanner's trajectory sharing.
        
        Returns True if collision detected (position should be rejected)
        """
        if drone.swarm_trajs is None:
            return False
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check if swarm trajectory data is stale
        if current_time - drone.last_swarm_update > self.swarm_traj_timeout:
            return False
        
        x, y, z = position
        
        # Check against each other drone's trajectory
        for traj in drone.swarm_trajs.traj:
            # Skip if this is the same drone
            if traj.drone_id == drone.drone_id:
                continue
            
            # Check positions along trajectory
            # MultiBsplines contains pos_pts (position points) for each trajectory
            if len(traj.pos_pts) == 0:
                continue
            
            # Check first few waypoints (near-term trajectory)
            # Most critical are the next few seconds of planned motion
            check_points = min(5, len(traj.pos_pts))
            
            for i in range(check_points):
                pt = traj.pos_pts[i]
                dx = x - pt.x
                dy = y - pt.y
                dz = z - pt.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # If too close to another drone's planned path, reject
                if distance < self.drone_safety_radius:
                    return True
        
        return False
    
    def _is_within_map_bounds(self, position: Tuple[float, float, float]) -> bool:
        """Check if position is within map boundaries"""
        x, y, z = position
        return (self.map_x_min <= x <= self.map_x_max and
                self.map_y_min <= y <= self.map_y_max and
                self.map_z_min <= z <= self.map_z_max)
    
    def _is_local_safe(self, position: Tuple[float, float, float],
                      local_map: OccupancyGrid, 
                      allow_unknown: bool = False) -> bool:
        """
        Enhanced safety check:
        - Center cell must be FREE (< occupied_threshold) or UNKNOWN (if allow_unknown)
        - Safety margin around obstacles (0.25m)
        
        Args:
            position: Position to check
            local_map: Local occupancy grid
            allow_unknown: If True, unknown cells are considered safe (for random fallback)
        """
        if local_map is None or local_map.info.width == 0 or local_map.info.height == 0:
            return allow_unknown  # If no map, allow if unknown is OK
        
        resolution = local_map.info.resolution
        origin_x = local_map.info.origin.position.x
        origin_y = local_map.info.origin.position.y
        
        # Convert to grid coordinates
        col = int((position[0] - origin_x) / resolution)
        row = int((position[1] - origin_y) / resolution)
        
        width = local_map.info.width
        height = local_map.info.height
        data = local_map.data
        
        # Check bounds - if outside local map, position is in unknown territory
        if not (0 <= col < width and 0 <= row < height):
            return allow_unknown  # Outside local map = unknown, allow if that's OK
        
        # Check center cell
        center_idx = row * width + col
        center_val = data[center_idx]
        
        # Center must be free, or unknown (if allowed)
        if center_val >= self.occupied_threshold:
            return False  # Obstacle - always reject
        if center_val == -1 and not allow_unknown:
            return False  # Unknown - reject unless explicitly allowed
        
        # Calculate safety radius in grid cells (0.25m margin)
        safety_cells = max(1, int(self.obstacle_safety_margin / resolution))
        
        # Check safety margin around obstacles
        for dr in range(-safety_cells, safety_cells + 1):
            for dc in range(-safety_cells, safety_cells + 1):
                check_row = row + dr
                check_col = col + dc
                
                if not (0 <= check_col < width and 0 <= check_row < height):
                    continue  # Outside local map - don't reject, just skip
                
                idx = check_row * width + check_col
                val = data[idx]
                
                # Any occupied cell in neighborhood = unsafe
                if val >= self.occupied_threshold:
                    return False
        
        return True
    
    def _generate_viewpoint_goal(self, frontier: Tuple[float, float, float], 
                                drone: DroneState) -> Optional[Tuple[float, float, float]]:
        """Generate viewpoint goal for a single frontier"""
        pos = drone.get_position()
        if pos is None:
            return None
        
        # Generate viewpoint: position between drone and frontier
        # Place viewpoint 5.0m away from frontier toward drone
        # This puts the goal deep in known free space while still observing frontier
        fx, fy, fz = frontier
        dx = pos[0] - fx
        dy = pos[1] - fy
        
        # Distance from drone to frontier
        frontier_dist = math.sqrt(dx*dx + dy*dy)
        
        if frontier_dist < 0.1:
            # Drone is at frontier, skip
            return None
        
        if frontier_dist < 6.0:
            # Too close - use halfway point between drone and frontier
            viewpoint_offset = frontier_dist * 0.5
        else:
            # Viewpoint offset distance (meters from frontier toward drone)
            viewpoint_offset = 5.0
        
        # Normalize direction vector
        dx_norm = dx / frontier_dist
        dy_norm = dy / frontier_dist
        
        # Calculate viewpoint position
        viewpoint_x = fx + dx_norm * viewpoint_offset
        viewpoint_y = fy + dy_norm * viewpoint_offset
        viewpoint_z = 1.0  # Fixed altitude for safe navigation
        
        # Clamp to map boundaries
        viewpoint_x = max(self.map_x_min, min(self.map_x_max, viewpoint_x))
        viewpoint_y = max(self.map_y_min, min(self.map_y_max, viewpoint_y))
        viewpoint_z = max(self.map_z_min, min(self.map_z_max, viewpoint_z))
        
        return (viewpoint_x, viewpoint_y, viewpoint_z)
    
    def _select_nearest_frontier(self, frontiers: List[Tuple[float, float, float]], 
                                drone: DroneState,
                                viewpoint_cache: dict = None) -> Optional[Tuple[float, float, float]]:
        """
        Select nearest frontier and return its viewpoint goal.
        
        Args:
            frontiers: List of frontier positions (already filtered for safety)
            drone: DroneState with position
            viewpoint_cache: Optional dict mapping frontier -> precomputed viewpoint
            
        Returns:
            Viewpoint goal position (x, y, z) or None
        """
        pos = drone.get_position()
        if pos is None or len(frontiers) == 0:
            return None
        
        # Find nearest frontier (use only XY distance — Z is always 1.0 for all frontiers)
        nearest_frontier = None
        min_dist = float('inf')
        
        for frontier in frontiers:
            dx = frontier[0] - pos[0]
            dy = frontier[1] - pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < min_dist:
                min_dist = distance
                nearest_frontier = frontier
        
        if nearest_frontier is None:
            return None
        
        # Use cached viewpoint if available, otherwise compute it
        if viewpoint_cache is not None and nearest_frontier in viewpoint_cache:
            return viewpoint_cache[nearest_frontier]
        return self._generate_viewpoint_goal(nearest_frontier, drone)
    
    def _generate_random_local_goal(self, drone: DroneState) -> Optional[Tuple[float, float, float]]:
        """Generate random goal in local free space around drone"""
        pos = drone.get_position()
        if pos is None:
            return None
        
        min_range, max_range = self.random_range
        
        valid_attempts = 0  # Track how many attempts passed basic checks
        
        # Try multiple random samples with different strategies
        for attempt in range(30):  # Increased attempts
            # Vary range based on attempt number
            if attempt < 10:
                # First try close range
                distance = np.random.uniform(min_range, (min_range + max_range) / 2)
            elif attempt < 20:
                # Then try full range
                distance = np.random.uniform(min_range, max_range)
            else:
                # Finally try very close (escape maneuver)
                distance = np.random.uniform(1.0, min_range)
            
            # Random angle - prefer forward direction initially
            if attempt < 5:
                # Prefer forward direction
                yaw = drone.get_yaw() or 0.0
                angle = yaw + np.random.uniform(-math.pi/3, math.pi/3)
            else:
                # Full 360 degrees
                angle = np.random.uniform(0, 2 * np.pi)
            
            # Generate goal
            goal_x = pos[0] + distance * np.cos(angle)
            goal_y = pos[1] + distance * np.sin(angle)
            goal_z = 1.0  # Fixed altitude
            
            # Clamp to map boundaries
            goal_x = max(self.map_x_min, min(self.map_x_max, goal_x))
            goal_y = max(self.map_y_min, min(self.map_y_max, goal_y))
            goal_z = max(self.map_z_min, min(self.map_z_max, goal_z))
            
            goal = (goal_x, goal_y, goal_z)
            
            # Check if safe - allow unknown cells for random fallback
            if self._is_local_safe(goal, drone.local_map, allow_unknown=True):
                # Also check global map safety if enabled
                if self.global_safety_check and not self._is_safe_in_global_map(goal, safety_margin=0.3):
                    continue  # Skip if known obstacle in global map
                
                valid_attempts += 1
                
                # Extra check: make sure it's not in visited
                is_visited = any(
                    math.sqrt((goal[0]-v[0])**2 + (goal[1]-v[1])**2) < self.visited_memory_radius
                    for v in drone.visited_positions
                )
                
                # Prefer goals in unexplored areas (only if using global frontiers)
                is_explored = self.use_global_frontiers and self._is_explored_globally(goal)
                
                if not is_visited and not is_explored:
                    return goal  # Best case: not visited, not explored
                elif not is_visited and valid_attempts < 20 and self.use_global_frontiers:
                    # Continue looking for unexplored areas first
                    continue
                elif not is_visited:
                    return goal  # Fallback: accept if not visited
        
        if self.debug_mode:
            self.get_logger().warn(
                f'Drone {drone.drone_id}: Random fallback FAILED after 30 attempts '
                f'(valid_but_visited={valid_attempts})')
        
        return None
    
    def _mark_explored(self, drone_id: int, position: Tuple[float, float, float]):
        """Mark cells around position as explored by this drone"""
        drone = self.drones.get(drone_id)
        if drone is None:
            return
        
        # Convert position to grid coordinates (using merged map resolution)
        resolution = self.global_map_resolution
        grid_x = int((position[0] + self.map_x_size / 2.0) / resolution)
        grid_y = int((position[1] + self.map_y_size / 2.0) / resolution)
        
        # Mark a small area around the position as explored (3x3 cells)
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                drone.explored_cells.add((grid_x + dx, grid_y + dy))
    
    def _publish_goal(self, drone_id: int, goal: Tuple[float, float, float],
                     current_time: float):
        """Publish goal to EgoPlanner"""
        drone = self.drones[drone_id]
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        msg.pose.orientation.w = 1.0
        
        self.goal_publishers[drone_id].publish(msg)
        
        # Update drone state
        drone.current_goal = goal
        drone.goal_assignment_time = current_time
        
        self.get_logger().info(
            f'Drone {drone_id}: Goal assigned ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f})')
    
    def _publish_debug_markers(self, drone_id: int,
                              all_frontiers: List[Tuple[float, float, float]],
                              safe_frontiers: List[Tuple[float, float, float]],
                              rejected_frontiers: List[Tuple[float, float, float]],
                              chosen_goal: Optional[Tuple[float, float, float]]):
        """Publish RViz visualization markers"""
        
        # All local frontiers
        self._publish_marker_array(
            f'local_{drone_id}',
            all_frontiers,
            ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.6)
        )
        
        # Rejected frontiers
        self._publish_marker_array(
            f'rejected_{drone_id}',
            rejected_frontiers,
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
        )
        
        # Chosen goal
        if chosen_goal is not None:
            self._publish_marker_array(
                f'chosen_{drone_id}',
                [chosen_goal],
                ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                scale=0.4
            )
    
    def _publish_marker_array(self, topic_key: str,
                             positions: List[Tuple[float, float, float]],
                             color: ColorRGBA,
                             scale: float = 0.2):
        """Publish marker array for visualization"""
        if topic_key not in self.marker_publishers:
            return
        
        marker_array = MarkerArray()
        
        # DELETEALL must come FIRST so old markers are cleared before adding new ones
        delete_marker = Marker()
        delete_marker.header.frame_id = 'world'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = topic_key
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for idx, pos in enumerate(positions):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = topic_key
            marker.id = idx + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            marker.color = color
            
            marker_array.markers.append(marker)
        
        self.marker_publishers[topic_key].publish(marker_array)
    
    def _publish_merged_map(self):
        """Publish merged occupancy grid from all drone local maps for RViz visualization.
        
        Uses vectorized numpy operations to avoid blocking the ROS2 executor.
        """
        resolution = self.global_map_resolution
        width = self.global_map_width
        height = self.global_map_height
        
        # Merge all drone local maps INTO the persistent global map (fully vectorized)
        for drone in self.drones.values():
            if drone.local_map is None:
                continue
            
            local_map = drone.local_map
            local_res = local_map.info.resolution
            local_width = local_map.info.width
            local_height = local_map.info.height
            local_origin_x = local_map.info.origin.position.x
            local_origin_y = local_map.info.origin.position.y
            
            # Build col/row index arrays for ALL local cells at once
            local_cols = np.arange(local_width)
            local_rows = np.arange(local_height)
            grid_cols, grid_rows = np.meshgrid(local_cols, local_rows)  # shape (H, W)
            
            world_x = local_origin_x + (grid_cols + 0.5) * local_res
            world_y = local_origin_y + (grid_rows + 0.5) * local_res
            
            merged_col = ((world_x + self.map_x_size / 2.0) / resolution).astype(int)
            merged_row = ((world_y + self.map_y_size / 2.0) / resolution).astype(int)
            
            # Mask: only process cells that map within global bounds
            valid = (
                (merged_col >= 0) & (merged_col < width) &
                (merged_row >= 0) & (merged_row < height)
            )
            
            local_data = np.array(local_map.data, dtype=np.int8).reshape(local_height, local_width)
            local_vals = local_data  # shape (H, W)
            
            # Only update cells that have known values (not -1)
            known_mask = valid & (local_vals != -1)
            
            mc = merged_col[known_mask]
            mr = merged_row[known_mask]
            lv = local_vals[known_mask].astype(np.int8)
            merged_idx = mr * width + mc
            
            # For cells never seen before: set directly
            never_seen = self.persistent_merged_data[merged_idx] == -1
            self.persistent_merged_data[merged_idx[never_seen]] = lv[never_seen]
            
            # For previously seen cells: use latest observation
            # Rationale: the inflated occupancy cloud moves WITH the drone,
            # so a cell previously marked occupied by inflation may now be
            # free. Always taking max() causes permanent obstacle buildup.
            # Instead: real obstacles (value=100) are persistent since they
            # appear consistently; inflated zones (lower values) get updated.
            seen = ~never_seen
            if np.any(seen):
                existing = self.persistent_merged_data[merged_idx[seen]]
                new_vals = lv[seen]
                # Hard obstacles (100) are sticky: once confirmed, keep them
                # For other values, take the latest observation to allow
                # inflation zones to clear as drone moves away
                is_hard_obstacle = (existing == 100) & (new_vals < 100)
                merged = new_vals.copy()
                merged[is_hard_obstacle] = existing[is_hard_obstacle]
                self.persistent_merged_data[merged_idx[seen]] = merged
        
        # Create and publish merged OccupancyGrid using persistent data
        merged_map = OccupancyGrid()
        merged_map.header.frame_id = 'world'
        merged_map.header.stamp = self.get_clock().now().to_msg()
        merged_map.info.resolution = resolution
        merged_map.info.width = width
        merged_map.info.height = height
        merged_map.info.origin.position.x = -self.map_x_size / 2.0
        merged_map.info.origin.position.y = -self.map_y_size / 2.0
        merged_map.info.origin.position.z = 0.0
        merged_map.info.origin.orientation.w = 1.0
        merged_map.data = self.persistent_merged_data.tolist()
        
        self.merged_map_publisher.publish(merged_map)
    
    def _log_coverage_statistics(self):
        """Log detailed coverage statistics and exploration progress"""
        # Calculate coverage from persistent merged map
        total_cells = len(self.persistent_merged_data)
        unknown_cells = np.sum(self.persistent_merged_data == -1)
        explored_cells = total_cells - unknown_cells
        free_cells = np.sum((self.persistent_merged_data >= 0) & (self.persistent_merged_data < 50))
        obstacle_cells = np.sum(self.persistent_merged_data >= 50)
        
        coverage_percent = (explored_cells / total_cells) * 100.0
        
        self.get_logger().info(
            f'\\n' + '='*60 + 
            f'\\nEXPLORATION COVERAGE STATISTICS' +
            f'\\n' + '='*60 +
            f'\\nTotal map cells: {total_cells}' +
            f'\\nExplored cells: {explored_cells} ({coverage_percent:.1f}%)' +
            f'\\nUnknown cells: {unknown_cells} ({(unknown_cells/total_cells)*100:.1f}%)' +
            f'\\n  - Free space: {free_cells} ({(free_cells/total_cells)*100:.1f}%)' +
            f'\\n  - Obstacles: {obstacle_cells} ({(obstacle_cells/total_cells)*100:.1f}%)' +
            f'\\n' + '-'*60
        )
        
        # Per-drone statistics
        for drone_id, drone in self.drones.items():
            if drone.local_map is not None and drone.odometry is not None:
                pos = drone.get_position()
                self.get_logger().info(
                    f'Drone {drone_id}: ' +
                    f'pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}), ' +
                    f'explored_cells={len(drone.explored_cells)}, ' +
                    f'failures={drone.consecutive_failures}, ' +
                    f'stuck_count={drone.position_stuck_count}'
                )
        
        # Check if exploration should be complete
        if coverage_percent >= 90.0:
            self.get_logger().info(
                f'\\n\u2705 EXPLORATION TARGET REACHED: {coverage_percent:.1f}% >= 90%' +
                f'\\n   If drones stopped, this is EXPECTED and CORRECT!'
            )
        elif self.consecutive_no_goals_cycles > 10:
            self.get_logger().error(
                f'\\n\u26a0\ufe0f  EXPLORATION STALLED: Only {coverage_percent:.1f}% < 90%' +
                f'\\n   Drones stopped but map not complete - investigate root cause!'
            )
    
    def _log_stuck_analysis(self):
        """Detailed analysis of why exploration is stuck"""
        self.get_logger().error(
            f'\\n' + '='*60 + 
            f'\\nSTUCK ANALYSIS - Detailed Root Cause Investigation' +
            f'\\n' + '='*60
        )
        
        for drone_id, drone in self.drones.items():
            if drone.local_map is None or drone.odometry is None:
                self.get_logger().error(f'Drone {drone_id}: NO DATA (map={drone.local_map is not None}, odom={drone.odometry is not None})')
                continue
            
            pos = drone.get_position()
            yaw = drone.get_yaw()
            
            self.get_logger().error(
                f'\\nDrone {drone_id} Analysis:' +
                f'\\n  Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})' +
                f'\\n  Yaw: {yaw:.2f} rad ({math.degrees(yaw):.1f}°)' +
                f'\\n  Current goal: {drone.current_goal}' +
                f'\\n  Last failure: {drone.last_goal_failure_reason}' +
                f'\\n  Consecutive failures: {drone.consecutive_failures}' +
                f'\\n  Position stuck count: {drone.position_stuck_count}' +
                f'\\n  Visited positions: {len(drone.visited_positions)}' +
                f'\\n  Last frontiers detected: {len(drone.last_frontiers)}'
            )
            
            # Check boundaries
            in_bounds = (self.map_x_min <= pos[0] <= self.map_x_max and 
                        self.map_y_min <= pos[1] <= self.map_y_max)
            self.get_logger().error(
                f'  Boundary check: {"IN BOUNDS" if in_bounds else "OUT OF BOUNDS!"}' +
                f'\\n    Map bounds: X=[{self.map_x_min:.1f}, {self.map_x_max:.1f}], ' +
                f'Y=[{self.map_y_min:.1f}, {self.map_y_max:.1f}]'
            )
            
            # Check if at corner
            at_corner = (abs(pos[0] - self.map_x_min) < 1.0 or abs(pos[0] - self.map_x_max) < 1.0) and \
                       (abs(pos[1] - self.map_y_min) < 1.0 or abs(pos[1] - self.map_y_max) < 1.0)
            if at_corner:
                self.get_logger().error(f'  \u26a0\ufe0f  STUCK AT CORNER!')
            
            # Analyze local map
            if drone.local_map is not None:
                local_unknown = sum(1 for val in drone.local_map.data if val == -1)
                local_total = len(drone.local_map.data)
                self.get_logger().error(
                    f'  Local map: {local_total} cells, {local_unknown} unknown ' +
                    f'({(local_unknown/local_total)*100:.1f}% unknown)'
                )
        
        # Overall coverage
        coverage_percent = (np.sum(self.persistent_merged_data != -1) / len(self.persistent_merged_data)) * 100.0
        self.get_logger().error(
            f'\\nGlobal Coverage: {coverage_percent:.1f}%' +
            f'\\n' + '='*60
        )


def main(args=None):
    rclpy.init(args=args)
    node = LocalExplorationManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
