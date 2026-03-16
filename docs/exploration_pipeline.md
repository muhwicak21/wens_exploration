# Multi-UAV Exploration System Pipeline
## Launch Commands

```bash
# Step 1: Start the simulation environment
ros2 launch ego_planner swarm.launch.py

# Step 2: Start RViz with exploration visualizer
ros2 launch ego_planner rviz.launch.py

# Step 3: Start exploration
ros2 launch wens_exploration exploration_swarm.launch.py debug_mode:=true
```

## Complete System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                    SYSTEM STARTUP                                        │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                    ┌──────────────────────┼──────────────────────┐
                    ▼                      ▼                      ▼
           ┌───────────────┐      ┌───────────────┐      ┌───────────────┐
           │  Terminal 1   │      │  Terminal 2   │      │  Terminal 3   │
           │  swarm.launch │      │  rviz.launch  │      │ exploration   │
           │               │      │               │      │ _local.launch │
           └───────┬───────┘      └───────┬───────┘      └───────┬───────┘
                   │                      │                      │
                   ▼                      ▼                      ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                               SIMULATION LAYER (EgoPlanner)                              │
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                          │
│   ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                    │
│   │   Drone 0   │  │   Drone 1   │  │   Drone 2   │  │   Drone 3   │                    │
│   │  (Red)      │  │  (Green)    │  │  (Blue)     │  │  (Yellow)   │                    │
│   │ Start:      │  │ Start:      │  │ Start:      │  │ Start:      │                    │
│   │ (-8,-7,1)   │  │ (-8,-2,1)   │  │ (-8,0,1)    │  │ (-8,5,1)    │                    │
│   └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘                    │
│          │                │                │                │                            │
│          ▼                ▼                ▼                ▼                            │
│   ┌──────────────────────────────────────────────────────────────────┐                  │
│   │                     Map Generator                                 │                  │
│   │  /map_generator/global_cloud (PointCloud2) - 3D obstacle map     │                  │
│   └──────────────────────────────────────────────────────────────────┘                  │
│                                                                                          │
│   For each drone, publishes:                                                            │
│   ┌────────────────────────────────────────────────────────────────────────────────┐    │
│   │ /drone_{id}/planner/grid/grid_map/occupancy_inflate (PointCloud2) - Inflated  │    │
│   │ /drone_{id}/localization/odometry                   (Odometry)    - Position  │    │
│   │ /drone_{id}/planner/planning/swarm_trajs            (MultiBsplines) - Paths   │    │
│   └────────────────────────────────────────────────────────────────────────────────┘    │
│                                                                                          │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           │ Sensor Data Flow
                                           ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                          EXPLORATION MANAGER (exploration_node_local)                    │
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                          │
│  ┌─────────────────────────────────────────────────────────────────────────────────┐    │
│  │                           INPUT PROCESSING                                       │    │
│  ├─────────────────────────────────────────────────────────────────────────────────┤    │
│  │                                                                                  │    │
│  │  1. SUBSCRIBE: /drone_{id}/planner/grid/grid_map/occupancy_inflate              │    │
│  │     └─→ PointCloud2 converted to OccupancyGrid (100x100 cells, 0.15m res)       │    │
│  │                                                                                  │    │
│  │  2. SUBSCRIBE: /drone_{id}/localization/odometry                                │    │
│  │     └─→ Update drone position, orientation, track movement                      │    │
│  │                                                                                  │    │
│  │  3. SUBSCRIBE: /drone_{id}/planner/planning/swarm_trajs                         │    │
│  │     └─→ Other drones' planned trajectories for collision avoidance             │    │
│  │                                                                                  │    │
│  │  4. SUBSCRIBE: /exploration/drone_{id}/visited                                  │    │
│  │     └─→ Other drones' visited positions for swarm coordination                 │    │
│  │                                                                                  │    │
│  └─────────────────────────────────────────────────────────────────────────────────┘    │
│                                           │                                              │
│                                           ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────────────────┐    │
│  │                         EXPLORATION LOOP (10 Hz)                                 │    │
│  ├─────────────────────────────────────────────────────────────────────────────────┤    │
│  │                                                                                  │    │
│  │  FOR EACH DRONE:                                                                │    │
│  │  ┌───────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ Step 1: CHECK GOAL STATUS                                                 │  │    │
│  │  │   • Goal reached? (distance < 0.6m)                                       │  │    │
│  │  │   • Goal timeout? (> 10 seconds)                                          │  │    │
│  │  │   • Drone stuck? (same position > 50 cycles)                              │  │    │
│  │  └───────────────────────────────────────────────────────────────────────────┘  │    │
│  │                              │                                                   │    │
│  │                              ▼                                                   │    │
│  │  ┌───────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ Step 2: DETECT FRONTIERS                                                  │  │    │
│  │  │   PRIMARY: Global Map Frontiers (persistent_merged_data)                  │  │    │
│  │  │     • Free cells adjacent to unknown cells                                │  │    │
│  │  │     • Cluster into groups (3m resolution)                                 │  │    │
│  │  │     • Return centroid positions                                           │  │    │
│  │  │   FALLBACK: Local Map Frontiers (if global empty)                         │  │    │
│  │  │     • Same algorithm on drone's local_map                                 │  │    │
│  │  └───────────────────────────────────────────────────────────────────────────┘  │    │
│  │                              │                                                   │    │
│  │                              ▼                                                   │    │
│  │  ┌───────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ Step 3: FILTER FRONTIERS                                                  │  │    │
│  │  │   ✗ Reject if visited by this drone (radius < 1.5m)                       │  │    │
│  │  │   ✗ Reject if visited by OTHER drones (swarm coordination)               │  │    │
│  │  │   ✗ Reject if too close to current goal (< 0.5m)                          │  │    │
│  │  │   ✗ Reject if viewpoint unsafe in local map                               │  │    │
│  │  │   ✗ Reject if viewpoint unsafe in global map (obstacles)                  │  │    │
│  │  │   ✗ Reject if collision with other drone's trajectory                     │  │    │
│  │  └───────────────────────────────────────────────────────────────────────────┘  │    │
│  │                              │                                                   │    │
│  │                              ▼                                                   │    │
│  │  ┌───────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ Step 4: ORIENTATION FILTER                                                │  │    │
│  │  │   • Prefer frontiers in forward hemisphere (±90°)                         │  │    │
│  │  │   • If none forward, allow backward (drone will turn)                     │  │    │
│  │  └───────────────────────────────────────────────────────────────────────────┘  │    │
│  │                              │                                                   │    │
│  │                              ▼                                                   │    │
│  │  ┌───────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ Step 5: SELECT GOAL                                                       │  │    │
│  │  │   • Select nearest frontier to drone                                      │  │    │
│  │  │   • Generate viewpoint (offset from obstacle boundary)                    │  │    │
│  │  │   • Check for goal conflict with other drones (< 3m)                      │  │    │
│  │  │   FALLBACK: Random goal if no frontiers                                   │  │    │
│  │  │     • Random direction, 3-8m distance                                     │  │    │
│  │  │     • Prefer unexplored areas                                             │  │    │
│  │  └───────────────────────────────────────────────────────────────────────────┘  │    │
│  │                              │                                                   │    │
│  │                              ▼                                                   │    │
│  │  ┌───────────────────────────────────────────────────────────────────────────┐  │    │
│  │  │ Step 6: PUBLISH GOAL                                                      │  │    │
│  │  │   /drone_{id}/move_base_simple/goal  (PoseStamped)                        │  │    │
│  │  │   • Mark goal area as explored                                            │  │    │
│  │  │   • Update swarm coordination data                                        │  │    │
│  │  └───────────────────────────────────────────────────────────────────────────┘  │    │
│  │                                                                                  │    │
│  └─────────────────────────────────────────────────────────────────────────────────┘    │
│                                           │                                              │
│                                           ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────────────────┐    │
│  │                        GLOBAL MAP MERGING (1 Hz)                                 │    │
│  ├─────────────────────────────────────────────────────────────────────────────────┤    │
│  │                                                                                  │    │
│  │  persistent_merged_data (240x133 cells = 36m x 20m @ 0.15m resolution)          │    │
│  │                                                                                  │    │
│  │  For each drone's local_map:                                                    │    │
│  │    • Transform local coordinates → global coordinates                          │    │
│  │    • Merge cells: known values overwrite unknown (-1)                          │    │
│  │    • Obstacles (≥65) take priority over free space                             │    │
│  │    • Data persists even after drone moves away                                 │    │
│  │                                                                                  │    │
│  │  PUBLISH: /global_merged_map (OccupancyGrid)                                    │    │
│  │                                                                                  │    │
│  └─────────────────────────────────────────────────────────────────────────────────┘    │
│                                           │                                              │
│                                           ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────────────────┐    │
│  │                         OUTPUT TOPICS                                            │    │
│  ├─────────────────────────────────────────────────────────────────────────────────┤    │
│  │                                                                                  │    │
│  │  /global_merged_map                    (OccupancyGrid) - Merged exploration map │    │
│  │  /drone_{id}/move_base_simple/goal     (PoseStamped)   - Navigation goals       │    │
│  │  /exploration/drone_{id}/visited       (PoseArray)     - Swarm coordination     │    │
│  │                                                                                  │    │
│  └─────────────────────────────────────────────────────────────────────────────────┘    │
│                                                                                          │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                    ┌──────────────────────┴──────────────────────┐
                    │                                             │
                    ▼                                             ▼
┌─────────────────────────────────────────┐   ┌─────────────────────────────────────────┐
│       MOTION PLANNING (EgoPlanner)       │   │        VISUALIZATION (RViz2)            │
├─────────────────────────────────────────┤   ├─────────────────────────────────────────┤
│                                          │   │                                          │
│  For each drone:                         │   │  exploration_visualizer node             │
│                                          │   │                                          │
│  INPUT:                                  │   │  SUBSCRIBES:                             │
│  • /drone_{id}/move_base_simple/goal    │   │  • /global_merged_map                    │
│  • /drone_{id}/planner/grid/grid_map/* │   │  • /drone_{id}/localization/odometry     │
│                                          │   │                                          │
│  PROCESS:                                │   │  PUBLISHES:                              │
│  1. A* path search                       │   │  • /exploration/trajectories             │
│  2. B-spline trajectory optimization    │   │  • /exploration/drone_positions          │
│  3. Collision checking                   │   │  • /exploration/statistics               │
│  4. Velocity/acceleration constraints    │   │  • /exploration/progress_bar             │
│                                          │   │                                          │
│  OUTPUT:                                 │   │  DISPLAYS:                               │
│  • Smooth trajectory commands            │   │  • Coverage: XX.X%                       │
│  • /drone_{id}_plan_vis/optimal_list    │   │  • Time: MM:SS                           │
│                                          │   │  • Distance per drone                    │
│                                          │   │  • Color-coded trajectories              │
│                                          │   │  • Progress bar                          │
│                                          │   │                                          │
└──────────────────────┬───────────────────┘   └─────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              DRONE EXECUTION                                             │
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                          │
│  Each drone follows its optimized trajectory:                                           │
│  • Position control                                                                      │
│  • Obstacle avoidance                                                                   │
│  • Swarm collision avoidance                                                            │
│                                                                                          │
│  Updates odometry → feeds back to exploration manager                                   │
│                                                                                          │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           │ Loop continues until...
                                           ▼
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              EXPLORATION COMPLETE                                        │
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                          │
│  Coverage Check (in _log_coverage_statistics):                                          │
│                                                                                          │
│  total_cells = 240 × 133 = 31,920 cells                                                 │
│  unknown_cells = count(persistent_merged_data == -1)                                    │
│  explored_cells = total_cells - unknown_cells                                           │
│  coverage = explored_cells / total_cells × 100%                                         │
│                                                                                          │
│  ┌────────────────────────────────────────────────────────────────────────────────┐     │
│  │                                                                                 │     │
│  │    Coverage: 100.0%  ████████████████████████████████████████████  COMPLETE   │     │
│  │                                                                                 │     │
│  │    Total Time: XX:XX                                                           │     │
│  │    Total Distance: XXX.X m                                                     │     │
│  │    Drones: D0=XX.Xm, D1=XX.Xm, D2=XX.Xm, D3=XX.Xm                             │     │
│  │                                                                                 │     │
│  └────────────────────────────────────────────────────────────────────────────────┘     │
│                                                                                          │
│  When no more frontiers exist → Exploration complete!                                   │
│                                                                                          │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

## Quick Reference: Topic List

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/map_generator/global_cloud` | PointCloud2 | Simulator | RViz | 3D obstacle visualization |
| `/drone_{id}/planner/grid/grid_map/occupancy_inflate` | PointCloud2 | EgoPlanner | exploration_manager | Local obstacle map |
| `/drone_{id}/localization/odometry` | Odometry | EgoPlanner | exploration_manager, visualizer | Drone position |
| `/drone_{id}/planner/planning/swarm_trajs` | MultiBsplines | EgoPlanner | exploration_manager | Other drones' paths |
| `/global_merged_map` | OccupancyGrid | exploration_manager | visualizer, RViz | Merged exploration map |
| `/drone_{id}/move_base_simple/goal` | PoseStamped | exploration_manager | EgoPlanner | Navigation goal |
| `/exploration/drone_{id}/visited` | PoseArray | exploration_manager | exploration_manager | Swarm coordination |
| `/exploration/trajectories` | MarkerArray | visualizer | RViz | Trajectory history |
| `/exploration/drone_positions` | MarkerArray | visualizer | RViz | Current positions |
| `/exploration/statistics` | MarkerArray | visualizer | RViz | Stats overlay |
| `/exploration/progress_bar` | MarkerArray | visualizer | RViz | Progress bar |


## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drone_ids` | [0,1,2,3] | Active drone IDs |
| `update_rate` | 10.0 Hz | Exploration loop frequency |
| `goal_timeout_sec` | 10.0s | Goal reassignment timeout |
| `goal_reached_threshold` | 0.6m | Distance to consider goal reached |
| `visited_memory_radius` | 1.5m | Avoid revisiting this radius |
| `swarm_goal_conflict_radius` | 3.0m | Min distance between drone goals |
| `use_global_frontiers` | true | Use global map for frontiers |
| `global_safety_check` | true | Check global map for obstacles |
