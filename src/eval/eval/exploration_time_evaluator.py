#!/usr/bin/env python3
"""Measure only exploration time from map coverage and drone odometry."""

from __future__ import annotations

import csv
import re
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node


EXPLORATION_NODE_NAME = '/swarm_exploration_node'


class EvaluationFinished(Exception):
    pass


def now_seconds(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def make_output_dir(output_root: Path, mode: str, seed: int) -> Path:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = output_root / f'{stamp}_{mode}_seed_{seed}'
    candidate = base
    index = 2
    while candidate.exists():
        candidate = output_root / f'{stamp}_{mode}_seed_{seed}_{index}'
        index += 1
    candidate.mkdir(parents=True, exist_ok=False)
    return candidate


def parameter_value_to_int(raw: str) -> Optional[int]:
    match = re.search(r'Integer value is:\s*(-?\d+)', raw)
    if match:
        return int(match.group(1))
    match = re.search(r'value is:\s*(-?\d+)', raw)
    return int(match.group(1)) if match else None


def try_get_exploration_drone_num(default_value: int) -> int:
    try:
        completed = subprocess.run(
            ['ros2', 'param', 'get', EXPLORATION_NODE_NAME, 'drone_num'],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return default_value
    if completed.returncode != 0:
        return default_value
    return parameter_value_to_int(completed.stdout) or default_value


def dump_effective_parameters(fallback: Dict[str, object]) -> Dict[str, object]:
    try:
        completed = subprocess.run(
            ['ros2', 'param', 'dump', EXPLORATION_NODE_NAME],
            check=False,
            capture_output=True,
            text=True,
            timeout=4.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return {'source': 'evaluator_fallback', 'parameters': fallback}

    if completed.returncode != 0 or not completed.stdout.strip():
        return {'source': 'evaluator_fallback', 'parameters': fallback}

    try:
        dumped = yaml.safe_load(completed.stdout) or {}
    except yaml.YAMLError:
        return {
            'source': 'ros2_param_dump_parse_failed',
            'raw': completed.stdout,
            'parameters': fallback,
        }

    return {'source': EXPLORATION_NODE_NAME, 'parameters': dumped}


class ExplorationTimeEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('exploration_time_evaluator')

        self.declare_parameter('drone_num', 4)
        self.declare_parameter('map_topic', '/swarm_exploration/occupancy_grid')
        self.declare_parameter('odom_topic_format', '/drone_{id}_visual_slam/odom')
        self.declare_parameter('target_coverage', 0.95)
        self.declare_parameter('coverage_hold_time', 5.0)
        self.declare_parameter('max_duration', 600.0)
        self.declare_parameter('exploration_mode', 'frontier_hgrid_role')
        self.declare_parameter('experiment_seed', 1)
        self.declare_parameter('output_root', '/root/catkin_ws/src/eval/results')

        declared_drone_num = int(self.get_parameter('drone_num').value)
        self.drone_num = try_get_exploration_drone_num(declared_drone_num)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.odom_topic_format = str(self.get_parameter('odom_topic_format').value)
        self.target_coverage = float(self.get_parameter('target_coverage').value)
        self.coverage_hold_time = float(self.get_parameter('coverage_hold_time').value)
        self.max_duration = float(self.get_parameter('max_duration').value)
        self.exploration_mode = str(self.get_parameter('exploration_mode').value)
        self.experiment_seed = int(self.get_parameter('experiment_seed').value)
        self.output_root = Path(str(self.get_parameter('output_root').value)).expanduser()
        self.output_dir = make_output_dir(
            self.output_root, self.exploration_mode, self.experiment_seed)

        self.odom_ready: List[bool] = [False] * self.drone_num
        self.valid_map_received = False
        self.latest_known_cells = 0
        self.latest_unknown_cells = 0
        self.latest_total_cells = 0
        self.latest_coverage_ratio = 0.0
        self.latest_map_resolution = 0.0
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        self.target_reached_since: Optional[float] = None
        self.last_progress_print = 0.0
        self.finished = False
        self.checkpoint_times = {
            'time_to_80_percent': None,
            'time_to_90_percent': None,
            'time_to_95_percent': None,
        }

        self.coverage_file = (self.output_dir / 'coverage.csv').open(
            'w', newline='', encoding='utf-8')
        self.coverage_writer = csv.DictWriter(
            self.coverage_file,
            fieldnames=[
                'timestamp',
                'elapsed_time',
                'known_cells',
                'unknown_cells',
                'total_cells',
                'coverage_ratio',
            ],
        )
        self.coverage_writer.writeheader()
        self.coverage_file.flush()

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, 10)
        self.odom_subs = []
        for drone_id in range(self.drone_num):
            topic = self.odom_topic_format.format(id=drone_id)
            self.odom_subs.append(
                self.create_subscription(
                    Odometry,
                    topic,
                    lambda msg, did=drone_id: self.on_odom(did, msg),
                    20,
                ))
            self.get_logger().info(f'Subscribed odometry: {topic}')

        self.timeout_timer = self.create_timer(0.5, self.check_timeout)
        self.get_logger().info(
            f'Waiting for map plus {self.drone_num} odometry streams. '
            f'target={100.0 * self.target_coverage:.2f}%, '
            f'hold={self.coverage_hold_time:.2f}s, max={self.max_duration:.2f}s')

    def on_odom(self, drone_id: int, _msg: Odometry) -> None:
        if 0 <= drone_id < len(self.odom_ready):
            self.odom_ready[drone_id] = True
        self.maybe_start(now_seconds(self))

    def on_map(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width == 0 or height == 0 or not msg.data:
            return

        timestamp = now_seconds(self)
        total_cells = len(msg.data)
        unknown_cells = sum(1 for value in msg.data if value == -1)
        known_cells = total_cells - unknown_cells
        coverage_ratio = known_cells / total_cells if total_cells else 0.0

        self.valid_map_received = True
        self.latest_known_cells = known_cells
        self.latest_unknown_cells = unknown_cells
        self.latest_total_cells = total_cells
        self.latest_coverage_ratio = coverage_ratio
        self.latest_map_resolution = float(msg.info.resolution)

        self.maybe_start(timestamp)
        elapsed_time = timestamp - self.start_time if self.start_time is not None else 0.0
        self.write_coverage_row(timestamp, elapsed_time)

        if self.start_time is None:
            return

        self.update_checkpoints(elapsed_time, coverage_ratio)
        self.print_progress(timestamp, elapsed_time, coverage_ratio)
        self.update_completion_hold(timestamp, elapsed_time, coverage_ratio)

    def maybe_start(self, timestamp: float) -> None:
        if self.start_time is not None:
            return
        if not self.valid_map_received:
            return
        if self.latest_known_cells <= 0:
            return
        if not all(self.odom_ready):
            return
        self.start_time = timestamp
        self.get_logger().info('START: map ready and all drone odometry received')

    def write_coverage_row(self, timestamp: float, elapsed_time: float) -> None:
        self.coverage_writer.writerow({
            'timestamp': f'{timestamp:.9f}',
            'elapsed_time': f'{elapsed_time:.9f}',
            'known_cells': self.latest_known_cells,
            'unknown_cells': self.latest_unknown_cells,
            'total_cells': self.latest_total_cells,
            'coverage_ratio': f'{self.latest_coverage_ratio:.9f}',
        })
        self.coverage_file.flush()

    def update_checkpoints(self, elapsed_time: float, coverage_ratio: float) -> None:
        thresholds = {
            'time_to_80_percent': 0.80,
            'time_to_90_percent': 0.90,
            'time_to_95_percent': 0.95,
        }
        for key, threshold in thresholds.items():
            if self.checkpoint_times[key] is None and coverage_ratio >= threshold:
                self.checkpoint_times[key] = elapsed_time

    def print_progress(self, timestamp: float, elapsed_time: float, coverage_ratio: float) -> None:
        if timestamp - self.last_progress_print < 1.0:
            return
        self.get_logger().info(
            f'[{self.exploration_mode}] elapsed={elapsed_time:.1f} s, '
            f'coverage={100.0 * coverage_ratio:.2f}%')
        self.last_progress_print = timestamp

    def update_completion_hold(
        self,
        timestamp: float,
        elapsed_time: float,
        coverage_ratio: float,
    ) -> None:
        if coverage_ratio >= self.target_coverage:
            if self.target_reached_since is None:
                self.target_reached_since = timestamp
            if timestamp - self.target_reached_since >= self.coverage_hold_time:
                self.finish(
                    success=True,
                    completion_reason='TARGET_COVERAGE_REACHED',
                    exploration_time=elapsed_time,
                    end_timestamp=timestamp,
                )
        else:
            self.target_reached_since = None

    def check_timeout(self) -> None:
        if self.start_time is None:
            return
        timestamp = now_seconds(self)
        elapsed_time = timestamp - self.start_time
        if elapsed_time >= self.max_duration:
            self.finish(
                success=False,
                completion_reason='TIMEOUT',
                exploration_time=self.max_duration,
                end_timestamp=timestamp,
            )

    def finish(
        self,
        success: bool,
        completion_reason: str,
        exploration_time: float,
        end_timestamp: float,
    ) -> None:
        if self.finished:
            return
        self.finished = True
        self.end_time = end_timestamp
        self.save_summary(success, completion_reason, exploration_time, end_timestamp)
        self.save_effective_parameters()
        self.coverage_file.flush()
        self.coverage_file.close()

        if success:
            self.get_logger().info(
                '\n'.join([
                    'Exploration completed',
                    f'Mode: {self.exploration_mode}',
                    f'Target coverage: {100.0 * self.target_coverage:.2f}%',
                    f'Exploration time: {exploration_time:.2f} seconds',
                    f'Final coverage: {100.0 * self.latest_coverage_ratio:.2f}%',
                    f'Result directory: {self.output_dir}',
                ]))
        else:
            self.get_logger().info(
                '\n'.join([
                    'Exploration timed out',
                    f'Mode: {self.exploration_mode}',
                    f'Maximum duration: {self.max_duration:.2f} seconds',
                    f'Final coverage: {100.0 * self.latest_coverage_ratio:.2f}%',
                    f'Result directory: {self.output_dir}',
                ]))
        raise EvaluationFinished

    def summary_payload(
        self,
        success: bool,
        completion_reason: str,
        exploration_time: float,
        end_timestamp: float,
    ) -> Dict[str, object]:
        return {
            'experiment': {
                'exploration_mode': self.exploration_mode,
                'experiment_seed': self.experiment_seed,
                'target_coverage': self.target_coverage,
                'coverage_hold_time': self.coverage_hold_time,
                'max_duration': self.max_duration,
            },
            'result': {
                'success': success,
                'completion_reason': completion_reason,
                'exploration_time': exploration_time,
                'final_coverage': self.latest_coverage_ratio,
                'time_to_80_percent': self.checkpoint_times['time_to_80_percent'],
                'time_to_90_percent': self.checkpoint_times['time_to_90_percent'],
                'time_to_95_percent': self.checkpoint_times['time_to_95_percent'],
                'start_timestamp': self.start_time,
                'end_timestamp': end_timestamp,
            },
        }

    def save_summary(
        self,
        success: bool,
        completion_reason: str,
        exploration_time: float,
        end_timestamp: float,
    ) -> None:
        with (self.output_dir / 'summary.yaml').open('w', encoding='utf-8') as stream:
            yaml.safe_dump(
                self.summary_payload(success, completion_reason, exploration_time, end_timestamp),
                stream,
                sort_keys=False,
            )

    def evaluator_parameter_fallback(self) -> Dict[str, object]:
        return {
            'drone_num': self.drone_num,
            'map_topic': self.map_topic,
            'odom_topic_format': self.odom_topic_format,
            'target_coverage': self.target_coverage,
            'coverage_hold_time': self.coverage_hold_time,
            'max_duration': self.max_duration,
            'exploration_mode': self.exploration_mode,
            'experiment_seed': self.experiment_seed,
            'output_root': str(self.output_root),
            'latest_map_resolution': self.latest_map_resolution,
        }

    def save_effective_parameters(self) -> None:
        payload = dump_effective_parameters(self.evaluator_parameter_fallback())
        with (self.output_dir / 'parameters_effective.yaml').open('w', encoding='utf-8') as stream:
            yaml.safe_dump(payload, stream, sort_keys=False)

    def close(self) -> None:
        if not self.coverage_file.closed:
            self.coverage_file.close()


def main() -> None:
    rclpy.init()
    node = ExplorationTimeEvaluator()
    try:
        rclpy.spin(node)
    except EvaluationFinished:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('Evaluation interrupted before completion.')
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
