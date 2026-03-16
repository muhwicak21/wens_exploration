# wens_exploration

Autonomous UAV exploration stack using MAVROS. This repository contains ROS packages for autonomous aerial exploration with PX4/ArduPilot flight controllers.

## Packages

### `wens_exploration_mavros`
Main exploration package providing:
- **Frontier-based 3D exploration** using OctoMap
- **MAVROS integration** for PX4/ArduPilot (OFFBOARD mode, arming, setpoints)
- **Simulation support** with Gazebo
- **Multiple mission types**: frontier, coverage (boustrophedon), random walk

## Requirements
- ROS Noetic (Ubuntu 20.04) or Melodic (Ubuntu 18.04)
- MAVROS (`ros-<distro>-mavros`)
- OctoMap (`ros-<distro>-octomap-server`, `ros-<distro>-octomap-ros`)
- Gazebo (for simulation)
- PX4 or ArduPilot SITL (for simulation)

## Build

```bash
cd ~/catkin_ws/src
git clone https://github.com/muhwicak21/wens_exploration.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Simulation
```bash
roslaunch wens_exploration_mavros exploration_sim.launch
```

### Real UAV
```bash
roslaunch wens_exploration_mavros exploration_real.launch fcu_url:=/dev/ttyUSB0:921600
```

### Parameters
See `wens_exploration_mavros/config/exploration_params.yaml` for all configurable parameters.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `takeoff_altitude` | 2.5 m | Takeoff and exploration altitude |
| `mission_type` | `frontier` | `frontier`, `coverage`, or `random` |
| `waypoint_tolerance` | 0.5 m | Acceptable distance to waypoint |
| `exploration_timeout` | 300 s | Max exploration time before RTH |

## Package Structure

```
wens_exploration_mavros/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── exploration_params.yaml    # Exploration parameters
│   └── waypoints.yaml             # Example waypoints
├── include/wens_exploration_mavros/
│   └── exploration_manager.h
├── launch/
│   ├── exploration_sim.launch     # Full Gazebo simulation
│   ├── exploration_real.launch    # Real UAV flight
│   └── mavros_only.launch         # MAVROS only
├── scripts/
│   ├── exploration_controller.py  # High-level controller
│   ├── waypoint_navigator.py      # Waypoint navigation
│   └── mission_planner.py         # Mission planning
├── src/
│   ├── exploration_manager.cpp    # C++ exploration manager
│   └── frontier_detector.cpp      # 3D frontier detection
└── worlds/
    └── exploration_world.world    # Gazebo simulation world
```
