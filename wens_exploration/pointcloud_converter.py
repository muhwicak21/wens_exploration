import numpy as np
import struct
from typing import Tuple
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header


class PointCloudToGridConverter:
    """Converts PointCloud2 local maps to OccupancyGrid."""

    def __init__(self, resolution: float = 0.1, grid_size: int = 100):
        """
        Initialize the converter.

        Args:
            resolution: Grid resolution in meters/cell
            grid_size: Size of the local grid (grid_size x grid_size cells)
        """
        self.resolution = resolution
        self.grid_size = grid_size
        self.grid_range = grid_size * resolution / 2.0  # meters from center

    def convert(self, cloud: PointCloud2, drone_pos: Tuple[float, float, float] = None) -> OccupancyGrid:
        """
        Convert PointCloud2 to OccupancyGrid.

        The PointCloud2 contains occupied voxel centers. We create a 2D occupancy
        grid centered on the drone position (or cloud center if not provided).

        Args:
            cloud: PointCloud2 message with occupied voxels
            drone_pos: Optional (x, y, z) drone position for centering the grid

        Returns:
            OccupancyGrid message
        """
        # Parse point cloud data
        points = self._parse_pointcloud2(cloud)
        
        if len(points) == 0:
            # Return empty grid
            return self._create_empty_grid(drone_pos or (0, 0, 0), cloud.header)

        # Determine grid center
        if drone_pos is not None:
            center_x, center_y = drone_pos[0], drone_pos[1]
        else:
            # Use point cloud center
            center_x = np.mean(points[:, 0])
            center_y = np.mean(points[:, 1])

        # Create occupancy grid
        grid = OccupancyGrid()
        grid.header = cloud.header
        grid.info.resolution = self.resolution
        grid.info.width = self.grid_size
        grid.info.height = self.grid_size
        
        # Set origin at bottom-left corner
        grid.info.origin.position.x = center_x - self.grid_range
        grid.info.origin.position.y = center_y - self.grid_range
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Initialize grid as unknown
        grid_data = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)

        # Mark free space around drone (assume local area is known)
        drone_col = int((center_x - grid.info.origin.position.x) / self.resolution)
        drone_row = int((center_y - grid.info.origin.position.y) / self.resolution)
        
        # Mark a local region as free (will be overwritten by obstacles)
        free_radius = int(5.0 / self.resolution)  # 5m radius assumed free
        for dr in range(-free_radius, free_radius + 1):
            for dc in range(-free_radius, free_radius + 1):
                r, c = drone_row + dr, drone_col + dc
                if 0 <= r < self.grid_size and 0 <= c < self.grid_size:
                    dist = np.sqrt(dr*dr + dc*dc) * self.resolution
                    if dist < 5.0:
                        grid_data[r, c] = 0  # Free

        # Mark occupied cells from point cloud
        for point in points:
            x, y, z = point
            
            # Convert to grid coordinates
            col = int((x - grid.info.origin.position.x) / self.resolution)
            row = int((y - grid.info.origin.position.y) / self.resolution)
            
            # Check bounds
            if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                # Mark as occupied (100 = definitely occupied)
                grid_data[row, col] = 100

        # Flatten and convert to list
        grid.data = grid_data.flatten().tolist()

        return grid

    def _parse_pointcloud2(self, cloud: PointCloud2) -> np.ndarray:
        """
        Parse PointCloud2 data to extract XYZ points.

        Args:
            cloud: PointCloud2 message

        Returns:
            Nx3 numpy array of points
        """
        if cloud.width == 0 or cloud.height == 0:
            return np.array([])

        # Find field offsets
        x_offset = None
        y_offset = None
        z_offset = None
        
        for field in cloud.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        if x_offset is None or y_offset is None or z_offset is None:
            return np.array([])

        # Extract points
        points = []
        point_step = cloud.point_step
        num_points = cloud.width * cloud.height
        
        for i in range(num_points):
            offset = i * point_step
            
            # Extract x, y, z (assuming float32)
            x = struct.unpack_from('f', cloud.data, offset + x_offset)[0]
            y = struct.unpack_from('f', cloud.data, offset + y_offset)[0]
            z = struct.unpack_from('f', cloud.data, offset + z_offset)[0]
            
            # Check for valid points (filter NaN/Inf)
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])

        return np.array(points) if points else np.array([])

    def _create_empty_grid(self, center_pos: Tuple[float, float, float], 
                          header: Header) -> OccupancyGrid:
        """Create an empty occupancy grid."""
        grid = OccupancyGrid()
        grid.header = header
        grid.info.resolution = self.resolution
        grid.info.width = self.grid_size
        grid.info.height = self.grid_size
        grid.info.origin.position.x = center_pos[0] - self.grid_range
        grid.info.origin.position.y = center_pos[1] - self.grid_range
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        
        # All unknown
        grid.data = [-1] * (self.grid_size * self.grid_size)
        
        return grid
