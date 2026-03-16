# wens_exploration

A **ROS 2** package for autonomous multi-UAV frontier-based exploration, designed to work with the [EgoPlanner](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) swarm simulator.

---

## 📦 Package Overview

| Item | Value |
|------|-------|
| Package name | `wens_exploration` |
| Version | `0.0.1` |
| ROS version | ROS 2 (tested on Humble) |
| Language | Python 3 |
| License | Apache-2.0 |

---

## ✨ Features

- **Frontier-based exploration** — automatically detects and assigns unexplored frontiers to each UAV
- **Multi-UAV swarm coordination** — drones share visited positions and current goals to avoid duplicate exploration
- **Inter-drone collision avoidance** — respects other drones' B-spline trajectories from EgoPlanner
- **Stuck detection & recovery** — monitors drone positions and reassigns goals when a drone is stuck
- **Global merged map** — assembles a persistent merged occupancy grid from all drones' local maps
- **Real-time RViz visualization** — trajectory history, coverage heatmap, live statistics, progress bar

---

## 🗂 Package Structure

```
wens_exploration_mavros/
├── wens_exploration/
│   ├── __init__.py
│   ├── exploration_manager_local.py   # Core exploration logic & swarm coordination
│   ├── exploration_visualizer.py      # RViz visualization node
│   ├── frontier_detector.py           # Frontier detection & clustering
│   └── pointcloud_converter.py        # PointCloud2 → OccupancyGrid converter
├── launch/
│   ├── exploration_swarm.launch.py    # Main launch file (manager + visualizer)
│   └── exploration_visualizer.launch.py
├── docs/
│   ├── exploration_pipeline.md        # Full system architecture & pipeline
│   └── topic_interface.md             # All ROS 2 topics (subscribed & published)
├── resource/
│   └── wens_exploration
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 🔧 Dependencies

- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `std_msgs`
- `visualization_msgs`
- `traj_utils` (from EgoPlanner swarm)
- `numpy`
- `scipy`

---

## 🚀 Quick Start

### 1. Clone into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/muhwicak21/wens_exploration.git
```

### 2. Build

```bash
cd ~/ros2_ws
colcon build --packages-select wens_exploration
source install/setup.bash
```

### 3. Launch (3 terminals)

```bash
# Terminal 1 — EgoPlanner swarm simulation
ros2 launch ego_planner swarm.launch.py

# Terminal 2 — RViz
ros2 launch ego_planner rviz.launch.py

# Terminal 3 — Exploration manager
ros2 launch wens_exploration exploration_swarm.launch.py debug_mode:=true
```

---

## ⚙️ Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `drone_ids` | `[0, 1, 2, 3]` | List of drone IDs to manage |
| `update_rate` | `10.0` | Exploration loop rate (Hz) |
| `goal_timeout_sec` | `10.0` | Timeout before reassigning a stuck drone (s) |
| `goal_reached_threshold` | `0.6` | Distance to consider goal reached (m) |
| `min_frontier_size` | `1` | Minimum frontier cluster size (cells) |
| `safe_neighborhood` | `1` | Safety check radius (cells) |
| `free_threshold` | `50` | Max occupancy value treated as free |
| `occupied_threshold` | `80` | Min occupancy value treated as occupied |
| `debug_mode` | `false` | Publish debug frontier markers |
| `swarm_coordination_enabled` | `true` | Enable swarm visited-position sharing |

---

## 📡 Key Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_{N}/planner/grid/grid_map/occupancy_inflate` | `PointCloud2` | Local inflated occupancy map |
| `/drone_{N}/localization/odometry` | `Odometry` | Drone position & orientation |
| `/drone_{N}/planner/planning/swarm_trajs` | `MultiBsplines` | Other drones' planned trajectories |
| `/exploration/drone_{N}/visited` | `PoseArray` | Shared visited positions (swarm) |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_{N}/move_base_simple/goal` | `PoseStamped` | Exploration goal sent to EgoPlanner |
| `/global_merged_map` | `OccupancyGrid` | Persistent merged global map |
| `/exploration/trajectories` | `MarkerArray` | Drone flight path history (RViz) |
| `/exploration/coverage_heatmap` | `MarkerArray` | Coverage heatmap (RViz) |
| `/exploration/statistics` | `MarkerArray` | Live stats — coverage %, distance |

> See [`docs/topic_interface.md`](docs/topic_interface.md) for the full topic reference.

---

## 📖 Documentation

- [Exploration Pipeline & Architecture](docs/exploration_pipeline.md)
- [Topic Interface Reference](docs/topic_interface.md)

---

## 📝 License

Apache-2.0 — see [LICENSE](LICENSE) for details.
