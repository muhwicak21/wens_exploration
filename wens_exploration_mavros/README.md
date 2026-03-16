# wens_exploration_mavros

**Waypoint Exploration Navigation System (WENS)** – a ROS package for autonomous drone exploration using [MAVROS](http://wiki.ros.org/mavros) with MAVLink-based autopilots (PX4 / ArduPilot).

---

## Features

- **Coverage-path exploration** – boustrophedon (lawnmower) grid exploration
- **Waypoint following** – execute a pre-defined sequence of local-frame waypoints
- **OFFBOARD control helpers** – ready-to-use arming / mode-switching utilities
- **Gazebo simulation world** – pre-built obstacle environment for testing
- **C++ and Python implementations** – choose your preferred language

---

## Package Structure

```
wens_exploration_mavros/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── exploration_params.yaml   # Tuning parameters for the exploration node
│   └── waypoints.yaml            # Pre-defined waypoint list
├── include/
│   └── wens_exploration_mavros/
│       └── exploration_node.h    # C++ exploration node header
├── launch/
│   ├── wens_exploration.launch   # Full simulation launch (Gazebo + MAVROS + exploration)
│   └── waypoint_follower.launch  # MAVROS + waypoint follower only
├── scripts/
│   ├── exploration_node.py       # Python autonomous exploration node
│   ├── waypoint_follower.py      # Python waypoint follower
│   └── offboard_control.py       # Python OFFBOARD control helper
├── src/
│   ├── exploration_node.cpp      # C++ exploration node
│   └── waypoint_manager_node.cpp # C++ waypoint manager node
└── worlds/
    └── exploration_world.world   # Gazebo simulation world with obstacles
```

---

## Dependencies

- ROS Noetic (or Melodic)
- [MAVROS](http://wiki.ros.org/mavros) (`sudo apt install ros-noetic-mavros ros-noetic-mavros-extras`)
- Gazebo (for simulation)
- PX4 SITL or ArduPilot SITL (for simulation)

---

## Building

```bash
cd ~/catkin_ws/src
git clone https://github.com/muhwicak21/wens_exploration.git wens_exploration_mavros
cd ..
catkin_make
source devel/setup.bash
```

---

## Running

### Simulation (Gazebo + PX4 SITL + MAVROS + Exploration)

```bash
roslaunch wens_exploration_mavros wens_exploration.launch
```

### Waypoint Follower only (real hardware or SITL already running)

```bash
roslaunch wens_exploration_mavros waypoint_follower.launch
```

### Python exploration node (standalone)

```bash
rosrun wens_exploration_mavros exploration_node.py
```

---

## Configuration

Edit `config/exploration_params.yaml` to tune the exploration behaviour:

| Parameter                  | Default | Description                              |
|---------------------------|---------|------------------------------------------|
| `exploration_altitude`    | 5.0 m   | Cruise altitude during exploration       |
| `takeoff_altitude`        | 3.0 m   | Initial takeoff altitude                 |
| `grid_spacing`            | 5.0 m   | Row/column spacing in coverage grid      |
| `grid_rows`               | 4       | Number of rows in coverage grid          |
| `grid_cols`               | 4       | Number of columns in coverage grid       |
| `waypoint_tolerance`      | 0.5 m   | Distance to consider a waypoint reached  |
| `hover_time`              | 2.0 s   | Dwell time at each waypoint              |
| `battery_return_threshold`| 20 %    | Battery level that triggers RTL          |

Edit `config/waypoints.yaml` to define a custom waypoint mission.

---

## License

MIT
