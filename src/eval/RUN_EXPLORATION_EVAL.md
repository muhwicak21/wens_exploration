# Run Exploration Evaluation

## Build

```bash
cd /root/catkin_ws
colcon build --packages-select exploration_swarm ego_planner eval --symlink-install
source install/setup.bash
```

## Run One Mode With Evaluation

Example: `frontier_hgrid_role`

```bash
cd /root/catkin_ws
source install/setup.bash

ros2 launch ego_planner swarm.launch.py \
  exploration_mode:=frontier_hgrid_role \
  run_evaluation:=true \
  experiment_seed:=1 \
  use_rviz:=false
```

Results are saved under:

```text
/root/catkin_ws/src/eval/results/
```

Stop and reset the simulation before running the next mode.

## Run All Four Modes

### Frontier Only

```bash
ros2 launch ego_planner swarm.launch.py \
  exploration_mode:=frontier \
  run_evaluation:=true \
  experiment_seed:=1 \
  use_rviz:=false
```

### Frontier + Separation

```bash
ros2 launch ego_planner swarm.launch.py \
  exploration_mode:=frontier_separation \
  run_evaluation:=true \
  experiment_seed:=1 \
  use_rviz:=false
```

### Frontier + HGrid

```bash
ros2 launch ego_planner swarm.launch.py \
  exploration_mode:=frontier_hgrid \
  run_evaluation:=true \
  experiment_seed:=1 \
  use_rviz:=false
```

### Frontier + HGrid + Role

```bash
ros2 launch ego_planner swarm.launch.py \
  exploration_mode:=frontier_hgrid_role \
  run_evaluation:=true \
  experiment_seed:=1 \
  use_rviz:=false
```

## Make Comparison Figure

```bash
cd /root/catkin_ws
source install/setup.bash

python3 -m eval.select_exploration_time_results \
  --results-root /root/catkin_ws/src/eval/results
```

Output:

```text
/root/catkin_ws/src/eval/results/exploration_time_comparison.csv
/root/catkin_ws/src/eval/results/exploration_time_comparison.png
```

## Optional Parameters

```bash
ros2 launch ego_planner swarm.launch.py \
  exploration_mode:=frontier_hgrid_role \
  run_evaluation:=true \
  experiment_seed:=1 \
  target_coverage:=0.95 \
  coverage_hold_time:=5.0 \
  max_duration:=600.0 \
  eval_output_root:=/root/catkin_ws/src/eval/results \
  use_rviz:=false
```
