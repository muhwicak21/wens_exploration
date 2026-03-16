# Topic Interface Reference — `wens_exploration`

This document describes all ROS 2 topics that the `wens_exploration` package **subscribes to** and **publishes**, based on `exploration_swarm.launch.py`.

Two nodes are launched:
- **`exploration_manager_local`** (`exploration_node_local`) — core exploration logic
- **`exploration_visualizer`** (`exploration_visualizer`) — RViz visualization

> `{N}` in topic names is replaced by a drone ID (e.g., `0`, `1`, `2`, `3` by default).

---

## ⚠️ Topic Naming Migration

Topics were updated to match the actual ROS 2 topic names published by the running system (verified via `ros2 topic list`).

| Old Topic (pre-migration) | New Topic (current) |
|---|---|
| `/drone_{N}_grid/grid_map/occupancy_inflate` | `/drone_{N}/planner/grid/grid_map/occupancy_inflate` |
| `/drone_{N}_visual_slam/odom` | `/drone_{N}/localization/odometry` |
| `/drone_{N}_planning/swarm_trajs` | `/drone_{N}/planner/planning/swarm_trajs` |

All other topics remain unchanged.

---

## 📥 Subscribed Topics

### `exploration_manager_local`

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/drone_{N}/planner/grid/grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | Best-effort | Local inflated occupancy map from each drone's EgoPlanner grid map module. Converted internally to `OccupancyGrid` (100×100 cells at 0.15 m/cell). |
| `/drone_{N}/localization/odometry` | `nav_msgs/Odometry` | Best-effort | Drone position and orientation. Used to center the local grid, check goal progress, and detect stuck behavior. |
| `/drone_{N}/planner/planning/swarm_trajs` | `traj_utils/MultiBsplines` | Reliable | Other drones' planned B-spline trajectories from EgoPlanner. Used for inter-drone collision avoidance when assigning new goals. |
| `/exploration/drone_{N}/visited` | `geometry_msgs/PoseArray` | Best-effort | Visited positions shared by other drones (swarm coordination). The last `Pose` in the array encodes the drone's current goal (marked with `orientation.z=1.0, orientation.w=0.0`). Only active when `swarm_coordination_enabled=true`. |

### `exploration_visualizer`

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/global_merged_map` | `nav_msgs/OccupancyGrid` | Reliable | Global merged map published by `exploration_manager_local`. Used to compute coverage statistics. Configurable via `map_topic` parameter. |
| `/drone_{N}/localization/odometry` | `nav_msgs/Odometry` | Best-effort | Drone odometry used to track and draw flight trajectory history in RViz. |

---

## 📤 Published Topics

### `exploration_manager_local`

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/drone_{N}/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Reliable | On demand | Exploration goal sent to EgoPlanner's motion planner. Published when a drone needs a new goal (goal reached, timed out, or stuck). `frame_id = world`. |
| `/exploration/drone_{N}/visited` | `geometry_msgs/PoseArray` | Reliable | ~`swarm_share_interval` (0.5 s default) | This drone's visited positions + current goal, shared with the rest of the swarm for coordination. Encoding: all poses except the last are visited waypoints; the last pose is the current goal (`orientation.z=1.0, w=0.0`). Only active when `swarm_coordination_enabled=true`. |
| `/global_merged_map` | `nav_msgs/OccupancyGrid` | Reliable | 1 Hz | Persistent merged global map assembled from all drones' local maps. Dimensions: `map_x_size × map_y_size` (default 36 m × 20 m) at 0.15 m resolution. Used by the visualizer and as the primary source for frontier detection. |
| `/frontier_local_{N}` *(debug only)* | `visualization_msgs/MarkerArray` | Reliable | On demand | All detected frontier candidates for drone `N`. Only published when `debug_mode=true`. |
| `/frontier_chosen_{N}` *(debug only)* | `visualization_msgs/MarkerArray` | Reliable | On demand | The selected frontier goal for drone `N`. Only published when `debug_mode=true`. |
| `/frontier_rejected_{N}` *(debug only)* | `visualization_msgs/MarkerArray` | Reliable | On demand | Frontiers rejected by safety/swarm filters for drone `N`. Only published when `debug_mode=true`. |

### `exploration_visualizer`

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/exploration/trajectories` | `visualization_msgs/MarkerArray` | Reliable | `update_rate` (10 Hz) | Line-strip markers showing each drone's flight path history. Color-coded by drone ID (0=Red, 1=Green, 2=Blue, 3=Yellow). |
| `/exploration/coverage_heatmap` | `visualization_msgs/MarkerArray` | Reliable | `update_rate` (10 Hz) | Cube markers overlaid on the map showing explored vs. unknown cell density. |
| `/exploration/statistics` | `visualization_msgs/MarkerArray` | Reliable | `update_rate` (10 Hz) | Text markers showing live exploration stats: coverage %, elapsed time, total distance per drone. |
| `/exploration/progress_bar` | `visualization_msgs/MarkerArray` | Reliable | `update_rate` (10 Hz) | Visual progress bar indicating overall map coverage percentage. |
| `/exploration/drone_positions` | `visualization_msgs/MarkerArray` | Reliable | `update_rate` (10 Hz) | Sphere/arrow markers showing each drone's current position in the world frame. |

---

## Topic Data Flow Diagram

```
EgoPlanner (per drone)
  │
  ├─► /drone_{N}/planner/grid/grid_map/occupancy_inflate  ────────────────────────────────►┐
  ├─► /drone_{N}/localization/odometry  ─────────────────────────────────────────────────►┐  │
  └─► /drone_{N}/planner/planning/swarm_trajs  ─────────────────────────────────────────►┐│  │
                                                                                          ││  │
                                                                                          ▼▼  ▼
                                                              ┌──────────────────────────────────────┐
  /exploration/drone_{N}/visited  ◄─────────────────────────►│   exploration_manager_local          │
  (shared between all drones)                                 │                                      │
                                                              │   Internal:                          │
                                                              │   • Frontier detection               │
                                                              │   • Swarm conflict check             │
                                                              │   • Goal assignment                  │
                                                              └──────┬───────────────┬───────────────┘
                                                                     │               │
                                         ┌───────────────────────────┘               │
                                         ▼                                           ▼
                        /drone_{N}/move_base_simple/goal             /global_merged_map (1 Hz)
                        (PoseStamped → EgoPlanner)                         │
                                                                           │
                                                              ┌────────────▼───────────────┐
                        /drone_{N}/localization/odometry ────►│   exploration_visualizer   │
                                                              └──────────────┬─────────────┘
                                                                             │
                                         ┌───────────────────────────────────┤
                                         ▼               ▼                   ▼
                          /exploration/trajectories   /exploration/       /exploration/
                          /exploration/drone_positions  statistics         progress_bar
                          /exploration/coverage_heatmap
                                         │
                                         ▼
                                      RViz2
```

---

## Connecting External Nodes

If you want to connect a **custom node** to this system, here is the minimum required interface:

### To feed sensor data into the exploration manager:
```
Publish → /drone_{N}/planner/grid/grid_map/occupancy_inflate  (sensor_msgs/PointCloud2)
Publish → /drone_{N}/localization/odometry                    (nav_msgs/Odometry)
```

### To receive exploration goals:
```
Subscribe → /drone_{N}/move_base_simple/goal          (geometry_msgs/PoseStamped)
```

### To monitor exploration state:
```
Subscribe → /global_merged_map                        (nav_msgs/OccupancyGrid)
Subscribe → /exploration/statistics                   (visualization_msgs/MarkerArray)
Subscribe → /exploration/progress_bar                 (visualization_msgs/MarkerArray)
```

### To participate in swarm coordination:
```
Publish    → /exploration/drone_{N}/visited           (geometry_msgs/PoseArray)
Subscribe  → /exploration/drone_{M}/visited           (geometry_msgs/PoseArray)  [M ≠ N]
```

---

## `/drone_{N}/planner/planning/swarm_trajs` — Deep Dive

**Message type:** `traj_utils/msg/MultiBsplines`  
**Publisher:** EgoPlanner (each drone broadcasts ALL drones' planned paths)  
**Subscriber:** `exploration_manager_local` (uses it to avoid trajectory conflicts)

### Message Structure

```
MultiBsplines
├── drone_id_from  (int32)   — ID of the drone that sent this broadcast
└── traj[]         (Bspline[]) — Array of B-spline trajectories (one per active drone)
        └── Bspline
               ├── drone_id    (int32)                    — Which drone owns this trajectory
               ├── order       (int32)                    — B-spline order (typically 3 = cubic)
               ├── traj_id     (int64)                    — Unique trajectory ID, increments each replan
               ├── start_time  (builtin_interfaces/Time)  — ROS time when trajectory starts
               ├── knots[]     (float64[])                — Knot vector for the B-spline parameterization
               ├── pos_pts[]   (geometry_msgs/Point[])    — Control points in world frame (x, y, z) [meters]
               ├── yaw_pts[]   (float64[])                — Yaw control points [radians]
               └── yaw_dt      (float64)                  — Time interval between yaw control points [seconds]
```

### What Each Field Means

| Field | Type | Unit | Meaning |
|-------|------|------|---------|
| `drone_id_from` | `int32` | — | The drone that published this message (e.g., `1` for drone 1) |
| `traj[i].drone_id` | `int32` | — | The drone whose trajectory this is (may differ from `drone_id_from`) |
| `traj[i].order` | `int32` | — | B-spline order: `3` = cubic (smooth, 2nd-order continuous) |
| `traj[i].traj_id` | `int64` | — | Increments on every replan; useful to detect if path changed |
| `traj[i].start_time` | `Time` | ROS time | Absolute time at which this trajectory begins execution |
| `traj[i].knots[]` | `float64[]` | seconds | Knot vector defining the B-spline time parameterization |
| `traj[i].pos_pts[]` | `Point[]` | meters | Control points (x, y, z) in the **world** frame. NOT the actual waypoints — the actual path is the B-spline curve through these points |
| `traj[i].yaw_pts[]` | `float64[]` | radians | Yaw angle control points, one per segment |
| `traj[i].yaw_dt` | `float64` | seconds | Time step between consecutive yaw control points |

### How It Is Used by `exploration_manager_local`

When assigning a new goal to drone `N`, the exploration manager:
1. Reads `swarm_trajs` from **all other drones** (stored per drone in `DroneState.swarm_trajs`)
2. Extracts future positions from each trajectory's `pos_pts` control points
3. Checks if the candidate goal is within `drone_safety_radius` (default: **1.0 m**) of any projected position
4. Rejects the goal if it conflicts — preventing two drones from flying into each other

```python
# Simplified logic inside exploration_manager_local.py
for other_drone in other_drones:
    if other_drone.swarm_trajs is None:
        continue
    for bspline in other_drone.swarm_trajs.traj:
        for pt in bspline.pos_pts:
            dist = euclidean((goal.x, goal.y), (pt.x, pt.y))
            if dist < drone_safety_radius:
                goal_rejected = True  # Too close to another drone's planned path
```

### Example Structure (Drone 1 broadcasting, 4-drone swarm)

```
MultiBsplines:
  drone_id_from: 1
  traj:
    [0]:
      drone_id: 0          # Drone 0's trajectory
      order: 3
      traj_id: 42
      start_time: {sec: 1710000123, nanosec: 500000000}
      knots: [0.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3, ...]
      pos_pts:
        - {x: -7.2, y: -6.8, z: 1.0}   # control point 0
        - {x: -5.1, y: -5.3, z: 1.0}   # control point 1
        - {x: -3.0, y: -4.0, z: 1.0}   # control point 2
        ...
      yaw_pts: [0.0, 0.15, 0.32, ...]
      yaw_dt: 0.1
    [1]:
      drone_id: 1          # Drone 1's own trajectory
      ...
    [2]:
      drone_id: 2
      ...
    [3]:
      drone_id: 3
      ...
```

### Reconstructing the Actual Path from Control Points

The `pos_pts` are **B-spline control points**, not the actual flown path. To evaluate the actual position at time `t`:

$$\mathbf{p}(t) = \sum_{i} N_{i,k}(t) \cdot \mathbf{p}_i$$

where $N_{i,k}(t)$ are the B-spline basis functions of order $k$ and $\mathbf{p}_i$ are the control points.

For a quick approximation (used internally), EgoPlanner treats the control points as a rough **polyline** of the intended path. The `exploration_manager_local` uses this approximation for its collision check.

> **Note:** `swarm_trajs` data is considered **stale** after `swarm_traj_timeout` seconds (default: **1.0 s**). If no update is received within that window, the trajectory is ignored for collision checking.

---

## Notes

- **QoS mismatch warning:** The map and odometry topics use `BEST_EFFORT` reliability. Make sure your publishers also use `BEST_EFFORT`, otherwise ROS 2 will silently drop the connection.
- **Swarm coordination topics** (`/exploration/drone_{N}/visited`) are only created when `swarm_coordination_enabled=true` (default: `true`).
- **Debug topics** (`/frontier_local_{N}`, `/frontier_chosen_{N}`, `/frontier_rejected_{N}`) are only created when `debug_mode=true` (default: `false`).
- The `/global_merged_map` topic is both **published** by `exploration_manager_local` and **subscribed** by `exploration_visualizer` — both nodes are launched together by `exploration_swarm.launch.py`.
