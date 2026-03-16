import numpy as np
from typing import List, Tuple
from nav_msgs.msg import OccupancyGrid
try:
    from scipy.ndimage import label as ndimage_label
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


class FrontierDetector:
    """Detects frontier cells in an occupancy grid."""

    def __init__(self, min_frontier_size: int = 5, unknown_value: int = -1, free_max: int = 50):
        """
        Initialize the frontier detector.

        Args:
            min_frontier_size: Minimum number of cells for a valid frontier cluster
            unknown_value: Occupancy value representing unknown cells (default -1)
            free_max: Maximum occupancy value for free space (default 50)
        """
        self.min_frontier_size = min_frontier_size
        self.UNKNOWN = unknown_value
        self.FREE_THRESHOLD = free_max
        self.OCCUPIED_THRESHOLD = 65  # Cells above this are considered occupied

    def _dilate(self, mask: np.ndarray) -> np.ndarray:
        """Vectorized 8-connectivity dilation using zero-padded slicing (no edge wrapping)."""
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

    def _label_components(self, mask: np.ndarray) -> Tuple[np.ndarray, int]:
        """
        Connected component labeling using scipy.ndimage.label (8-connectivity).
        Falls back to iterative flood-fill if scipy is unavailable.
        """
        if _HAS_SCIPY:
            structure = np.ones((3, 3), dtype=int)  # 8-connectivity
            labeled, num_labels = ndimage_label(mask, structure=structure)
            return labeled.astype(np.int32), num_labels
        
        # Fallback: iterative flood-fill (no recursion depth issues)
        labeled = np.zeros_like(mask, dtype=np.int32)
        current_label = 0
        h, w = mask.shape
        
        for i in range(h):
            for j in range(w):
                if mask[i, j] and labeled[i, j] == 0:
                    current_label += 1
                    stack = [(i, j)]
                    while stack:
                        ci, cj = stack.pop()
                        if ci < 0 or ci >= h or cj < 0 or cj >= w:
                            continue
                        if labeled[ci, cj] != 0 or not mask[ci, cj]:
                            continue
                        labeled[ci, cj] = current_label
                        for di in [-1, 0, 1]:
                            for dj in [-1, 0, 1]:
                                if di == 0 and dj == 0:
                                    continue
                                stack.append((ci + di, cj + dj))
        
        return labeled, current_label

    def detect_frontiers(self, grid: OccupancyGrid) -> List[Tuple[float, float, float]]:
        """
        Detect frontier cells in an occupancy grid.

        A frontier cell is a free cell that is adjacent to at least one unknown cell.

        Args:
            grid: OccupancyGrid message containing the map

        Returns:
            List of frontier centroids as (x, y, z) tuples in map coordinates
        """
        if grid.info.width == 0 or grid.info.height == 0:
            return []

        # Convert grid data to numpy array
        width = grid.info.width
        height = grid.info.height
        data = np.array(grid.data, dtype=np.int8).reshape((height, width))

        # Create binary masks
        free_mask = (data >= 0) & (data < self.FREE_THRESHOLD)
        unknown_mask = (data == self.UNKNOWN)
        occupied_mask = (data >= self.OCCUPIED_THRESHOLD)

        # Find frontier cells: free cells adjacent to unknown cells
        # Dilate unknown to find cells near unknown
        unknown_dilated = self._dilate(unknown_mask)
        
        # Dilate occupied to create safety margin (avoid frontiers too close to obstacles)
        occupied_dilated = self._dilate(occupied_mask)
        # Apply dilation twice for more safety
        occupied_dilated = self._dilate(occupied_dilated)
        
        # Frontier cells are:
        # 1. Free cells (known free space)
        # 2. Adjacent to unknown cells (next to unexplored)
        # 3. Not adjacent to occupied cells (safe clearance from obstacles)
        frontier_mask = free_mask & unknown_dilated & ~occupied_dilated

        if not np.any(frontier_mask):
            return []

        # Label connected components to find frontier clusters
        labeled_frontiers, num_clusters = self._label_components(frontier_mask)

        # Extract centroids of each frontier cluster
        centroids = []
        for cluster_id in range(1, num_clusters + 1):
            cluster_mask = (labeled_frontiers == cluster_id)
            cluster_size = np.sum(cluster_mask)
            
            # Filter small clusters
            if cluster_size < self.min_frontier_size:
                continue

            # Compute centroid in grid coordinates
            indices = np.argwhere(cluster_mask)
            centroid_row = np.mean(indices[:, 0])
            centroid_col = np.mean(indices[:, 1])
            
            # Snap to nearest free cell in cluster (ensure goal is in free space)
            # Find cluster cell closest to centroid
            distances = np.sqrt((indices[:, 0] - centroid_row)**2 + 
                              (indices[:, 1] - centroid_col)**2)
            nearest_idx = np.argmin(distances)
            snap_row, snap_col = indices[nearest_idx]

            # Convert from grid coordinates to world coordinates (using snapped position)
            world_x = grid.info.origin.position.x + snap_col * grid.info.resolution
            world_y = grid.info.origin.position.y + snap_row * grid.info.resolution
            world_z = grid.info.origin.position.z  # Assume 2D map

            centroids.append((world_x, world_y, world_z))

        return centroids

    def get_frontier_clusters(self, grid: OccupancyGrid) -> List[np.ndarray]:
        """
        Get all frontier cell positions grouped by cluster.

        Args:
            grid: OccupancyGrid message

        Returns:
            List of numpy arrays, each containing (x, y, z) positions for a cluster
        """
        if grid.info.width == 0 or grid.info.height == 0:
            return []

        width = grid.info.width
        height = grid.info.height
        data = np.array(grid.data, dtype=np.int8).reshape((height, width))

        free_mask = (data >= 0) & (data < self.FREE_THRESHOLD)
        unknown_mask = (data == self.UNKNOWN)
        occupied_mask = (data >= self.OCCUPIED_THRESHOLD)

        unknown_dilated = self._dilate(unknown_mask)
        occupied_dilated = self._dilate(occupied_mask)
        frontier_mask = free_mask & unknown_dilated & ~occupied_dilated

        if not np.any(frontier_mask):
            return []

        labeled_frontiers, num_clusters = self._label_components(frontier_mask)

        clusters = []
        for cluster_id in range(1, num_clusters + 1):
            cluster_mask = (labeled_frontiers == cluster_id)
            cluster_size = np.sum(cluster_mask)
            
            if cluster_size < self.min_frontier_size:
                continue

            # Get all cell positions in this cluster
            indices = np.argwhere(cluster_mask)
            positions = []
            for row, col in indices:
                x = grid.info.origin.position.x + col * grid.info.resolution
                y = grid.info.origin.position.y + row * grid.info.resolution
                z = grid.info.origin.position.z
                positions.append([x, y, z])
            
            clusters.append(np.array(positions))

        return clusters
