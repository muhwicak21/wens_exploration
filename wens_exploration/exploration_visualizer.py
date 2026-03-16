#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point, Vector3
import numpy as np
import math
from typing import Dict, List, Tuple, Optional
from collections import deque


class DroneTrajectory:
    """Track drone trajectory history"""
    MAX_HISTORY = 2000  # Maximum trajectory points to keep
    
    def __init__(self, drone_id: int):
        self.drone_id = drone_id
        self.positions: deque = deque(maxlen=self.MAX_HISTORY)
        self.last_position: Optional[Tuple[float, float, float]] = None
        self.total_distance: float = 0.0
        self.start_time: Optional[float] = None
    
    def add_position(self, x: float, y: float, z: float, time: float):
        """Add a new position to trajectory history"""
        if self.start_time is None:
            self.start_time = time
        
        pos = (x, y, z)
        
        # Calculate distance traveled
        if self.last_position is not None:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            dz = z - self.last_position[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Only add if moved significantly (avoid noise)
            if dist > 0.05:  # 5cm minimum movement
                self.total_distance += dist
                self.positions.append(pos)
                self.last_position = pos
        else:
            self.positions.append(pos)
            self.last_position = pos


class ExplorationVisualizer(Node):
    """ROS2 node for visualizing exploration progress"""
    
    # Drone colors (RGBA)
    DRONE_COLORS = [
        (1.0, 0.0, 0.0, 1.0),  # Red - Drone 0
        (0.0, 1.0, 0.0, 1.0),  # Green - Drone 1
        (0.0, 0.0, 1.0, 1.0),  # Blue - Drone 2
        (1.0, 1.0, 0.0, 1.0),  # Yellow - Drone 3
        (1.0, 0.0, 1.0, 1.0),  # Magenta - Drone 4
        (0.0, 1.0, 1.0, 1.0),  # Cyan - Drone 5
    ]
    
    def __init__(self):
        super().__init__('exploration_visualizer')
        
        # Declare parameters
        self.declare_parameter('drone_ids', [0, 1, 2, 3])
        self.declare_parameter('update_rate', 2.0)  # Hz
        self.declare_parameter('map_topic', '/global_merged_map')
        self.declare_parameter('topic_odom', '/drone_{drone_id}/localization/odometry')
        self.declare_parameter('show_trajectories', True)
        self.declare_parameter('show_coverage_heatmap', True)
        self.declare_parameter('show_statistics', True)
        self.declare_parameter('show_progress_bar', True)
        
        # Get parameters
        self.drone_ids = self.get_parameter('drone_ids').value
        self.update_rate = self.get_parameter('update_rate').value
        self.map_topic = self.get_parameter('map_topic').value
        self.topic_odom_pattern = self.get_parameter('topic_odom').value
        self.show_trajectories = self.get_parameter('show_trajectories').value
        self.show_coverage_heatmap = self.get_parameter('show_coverage_heatmap').value
        self.show_statistics = self.get_parameter('show_statistics').value
        self.show_progress_bar = self.get_parameter('show_progress_bar').value
        
        # State
        self.merged_map: Optional[OccupancyGrid] = None
        self.trajectories: Dict[int, DroneTrajectory] = {
            drone_id: DroneTrajectory(drone_id) for drone_id in self.drone_ids
        }
        self.exploration_start_time: Optional[float] = None
        self.coverage_history: List[Tuple[float, float]] = []  # (time, coverage%)
        
        # QoS
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        
        # Subscribe to merged map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            qos_reliable
        )
        
        # Subscribe to drone odometry (matches EgoPlanner's visual slam output)
        self.odom_subs = {}
        for drone_id in self.drone_ids:
            topic = self.topic_odom_pattern.format(drone_id=drone_id)
            self.odom_subs[drone_id] = self.create_subscription(
                Odometry,
                topic,
                lambda msg, d=drone_id: self._odom_callback(d, msg),
                qos_sensor
            )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            MarkerArray,
            '/exploration/trajectories',
            qos_reliable
        )
        
        self.heatmap_pub = self.create_publisher(
            MarkerArray,
            '/exploration/coverage_heatmap',
            qos_reliable
        )
        
        self.stats_pub = self.create_publisher(
            MarkerArray,
            '/exploration/statistics',
            qos_reliable
        )
        
        self.progress_pub = self.create_publisher(
            MarkerArray,
            '/exploration/progress_bar',
            qos_reliable
        )
        
        self.drone_markers_pub = self.create_publisher(
            MarkerArray,
            '/exploration/drone_positions',
            qos_reliable
        )
        
        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / self.update_rate, self._update_callback)
        
        self.get_logger().info(
            f'Exploration Visualizer started for drones: {self.drone_ids}'
        )
    
    def _map_callback(self, msg: OccupancyGrid):
        """Handle merged map updates"""
        self.merged_map = msg
        
        if self.exploration_start_time is None:
            self.exploration_start_time = self.get_clock().now().nanoseconds / 1e9
    
    def _odom_callback(self, drone_id: int, msg: Odometry):
        """Handle drone odometry updates"""
        pos = msg.pose.pose.position
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        self.trajectories[drone_id].add_position(
            pos.x, pos.y, pos.z, current_time
        )
    
    def _calculate_coverage(self) -> Tuple[float, int, int, int]:
        """
        Calculate exploration coverage statistics.
        
        Returns:
            (coverage_percent, explored_cells, unknown_cells, total_cells)
        """
        if self.merged_map is None:
            return 0.0, 0, 0, 0
        
        data = np.array(self.merged_map.data, dtype=np.int8)
        total_cells = len(data)
        unknown_cells = np.sum(data == -1)
        explored_cells = total_cells - unknown_cells
        
        coverage = (explored_cells / total_cells) * 100.0 if total_cells > 0 else 0.0
        
        return coverage, explored_cells, unknown_cells, total_cells
    
    def _update_callback(self):
        """Periodic update callback"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Calculate coverage
        coverage, explored, unknown, total = self._calculate_coverage()
        
        # Record coverage history
        if self.exploration_start_time is not None:
            elapsed = current_time - self.exploration_start_time
            self.coverage_history.append((elapsed, coverage))
            # Keep only last 1000 entries
            if len(self.coverage_history) > 1000:
                self.coverage_history = self.coverage_history[-1000:]
        
        # Publish visualizations
        if self.show_trajectories:
            self._publish_trajectories()
        
        if self.show_statistics:
            self._publish_statistics(coverage, explored, unknown, total, current_time)
        
        if self.show_progress_bar:
            self._publish_progress_bar(coverage)
        
        self._publish_drone_positions()
        
        # Log periodic updates
        if int(current_time) % 10 == 0:  # Every 10 seconds
            elapsed = current_time - self.exploration_start_time if self.exploration_start_time else 0
            total_dist = sum(t.total_distance for t in self.trajectories.values())
            self.get_logger().info(
                f'Coverage: {coverage:.1f}% | '
                f'Explored: {explored}/{total} cells | '
                f'Time: {elapsed:.0f}s | '
                f'Total distance: {total_dist:.1f}m'
            )
    
    def _publish_trajectories(self):
        """Publish drone trajectory markers"""
        markers = MarkerArray()
        
        for drone_id, trajectory in self.trajectories.items():
            if len(trajectory.positions) < 2:
                continue
            
            # Line strip for trajectory
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f'trajectory_drone_{drone_id}'
            marker.id = drone_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Line width
            
            # Color based on drone ID
            color = self.DRONE_COLORS[drone_id % len(self.DRONE_COLORS)]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8
            
            # Add points
            for pos in trajectory.positions:
                p = Point()
                p.x = pos[0]
                p.y = pos[1]
                p.z = pos[2]
                marker.points.append(p)
            
            markers.markers.append(marker)
        
        self.trajectory_pub.publish(markers)
    
    def _publish_drone_positions(self):
        """Publish current drone position markers"""
        markers = MarkerArray()
        
        for drone_id, trajectory in self.trajectories.items():
            if trajectory.last_position is None:
                continue
            
            pos = trajectory.last_position
            
            # Sphere for current position
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f'drone_position'
            marker.id = drone_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            
            color = self.DRONE_COLORS[drone_id % len(self.DRONE_COLORS)]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            
            markers.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'world'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = f'drone_label'
            text_marker.id = drone_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pos[0]
            text_marker.pose.position.y = pos[1]
            text_marker.pose.position.z = pos[2] + 0.6
            text_marker.scale.z = 0.3  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f'D{drone_id}'
            
            markers.markers.append(text_marker)
        
        self.drone_markers_pub.publish(markers)
    
    def _publish_statistics(self, coverage: float, explored: int, 
                           unknown: int, total: int, current_time: float):
        """Publish exploration statistics as text markers"""
        markers = MarkerArray()
        
        # Calculate elapsed time
        elapsed = current_time - self.exploration_start_time if self.exploration_start_time else 0
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)
        
        # Calculate total distance traveled
        total_distance = sum(t.total_distance for t in self.trajectories.values())
        
        # Calculate coverage rate
        coverage_rate = coverage / elapsed if elapsed > 0 else 0
        
        # Create text display
        stats_text = [
            f'=== EXPLORATION RESULTS ===',
            f'Coverage: {coverage:.1f}%',
            f'Explored: {explored:,} / {total:,} cells',
            f'Unknown: {unknown:,} cells',
            f'',
            f'Time: {minutes:02d}:{seconds:02d}',
            f'Rate: {coverage_rate:.2f}%/s',
            f'Total Distance: {total_distance:.1f}m',
            f'',
            f'=== PER DRONE STATS ===',
        ]
        
        for drone_id, trajectory in self.trajectories.items():
            color_name = ['Red', 'Green', 'Blue', 'Yellow', 'Magenta', 'Cyan'][drone_id % 6]
            stats_text.append(
                f'D{drone_id} ({color_name}): {trajectory.total_distance:.1f}m'
            )
        
        # Position for stats display (top-left corner of map)
        base_x = -18.0
        base_y = 10.0
        base_z = 3.0
        
        for i, line in enumerate(stats_text):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'statistics'
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = base_x
            marker.pose.position.y = base_y - i * 0.4
            marker.pose.position.z = base_z
            marker.scale.z = 0.35  # Text height
            
            # Color coding
            if 'Coverage:' in line:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif 'RESULTS' in line or 'STATS' in line:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = line
            
            markers.markers.append(marker)
        
        self.stats_pub.publish(markers)
    
    def _publish_progress_bar(self, coverage: float):
        """Publish a visual progress bar for coverage"""
        markers = MarkerArray()
        
        # Progress bar position
        bar_x = 0.0
        bar_y = -12.0
        bar_z = 0.1
        bar_width = 20.0
        bar_height = 1.0
        
        # Background bar (gray)
        bg_marker = Marker()
        bg_marker.header.frame_id = 'world'
        bg_marker.header.stamp = self.get_clock().now().to_msg()
        bg_marker.ns = 'progress_bar'
        bg_marker.id = 0
        bg_marker.type = Marker.CUBE
        bg_marker.action = Marker.ADD
        bg_marker.pose.position.x = bar_x
        bg_marker.pose.position.y = bar_y
        bg_marker.pose.position.z = bar_z
        bg_marker.scale.x = bar_width
        bg_marker.scale.y = bar_height
        bg_marker.scale.z = 0.1
        bg_marker.color.r = 0.3
        bg_marker.color.g = 0.3
        bg_marker.color.b = 0.3
        bg_marker.color.a = 0.8
        markers.markers.append(bg_marker)
        
        # Progress fill (green)
        fill_width = (coverage / 100.0) * bar_width
        if fill_width > 0.01:  # Avoid zero-scale markers
            fill_marker = Marker()
            fill_marker.header.frame_id = 'world'
            fill_marker.header.stamp = self.get_clock().now().to_msg()
            fill_marker.ns = 'progress_bar'
            fill_marker.id = 1
            fill_marker.type = Marker.CUBE
            fill_marker.action = Marker.ADD
            fill_marker.pose.position.x = bar_x - (bar_width - fill_width) / 2
            fill_marker.pose.position.y = bar_y
            fill_marker.pose.position.z = bar_z + 0.05
            fill_marker.scale.x = fill_width
            fill_marker.scale.y = bar_height * 0.8
            fill_marker.scale.z = 0.12
            
            # Color gradient from red to green based on coverage
            if coverage < 50:
                fill_marker.color.r = 1.0
                fill_marker.color.g = coverage / 50.0
            else:
                fill_marker.color.r = 1.0 - (coverage - 50) / 50.0
                fill_marker.color.g = 1.0
            fill_marker.color.b = 0.0
            fill_marker.color.a = 1.0
            markers.markers.append(fill_marker)
        
        # Progress text
        text_marker = Marker()
        text_marker.header.frame_id = 'world'
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = 'progress_bar'
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = bar_x
        text_marker.pose.position.y = bar_y
        text_marker.pose.position.z = bar_z + 0.5
        text_marker.scale.z = 0.8
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f'EXPLORATION: {coverage:.1f}%'
        markers.markers.append(text_marker)
        
        self.progress_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
