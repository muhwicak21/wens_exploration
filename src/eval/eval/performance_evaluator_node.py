#!/usr/bin/env python3
"""Create evaluation figures from a saved exploration run."""

from __future__ import annotations

import argparse
import math
import shutil
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


DEFAULT_INPUT = Path('/root/share/run_01.zip')
DEFAULT_OUTPUT = Path('/root/catkin_ws/src/eval/figure')
FIGURE_EXTENSIONS = {'.png', '.pdf', '.jpg', '.jpeg', '.svg'}


class EvaluationComplete(Exception):
    """Raised internally when live evaluation reaches its requested duration."""


def parse_summary(summary_path: Path) -> Dict[str, float]:
    metrics: Dict[str, float] = {}
    if not summary_path.exists():
        return metrics

    for line in summary_path.read_text(encoding='utf-8').splitlines():
        if ':' not in line:
            continue
        key, raw_value = line.split(':', 1)
        try:
            metrics[key.strip()] = float(raw_value.strip())
        except ValueError:
            continue
    return metrics


def find_run_root(path: Path) -> Tuple[Path, tempfile.TemporaryDirectory | None]:
    if path.is_dir():
        return path, None
    if not path.is_file():
        raise FileNotFoundError(f'Input run does not exist: {path}')
    if path.suffix.lower() != '.zip':
        raise ValueError(f'Unsupported input type: {path}. Use a run directory or .zip file.')

    tmp = tempfile.TemporaryDirectory(prefix='swarm_eval_')
    with zipfile.ZipFile(path) as archive:
        archive.extractall(tmp.name)

    extracted = Path(tmp.name)
    children = [child for child in extracted.iterdir() if child.is_dir()]
    if len(children) == 1:
        return children[0], tmp
    return extracted, tmp


def copy_existing_figures(run_root: Path, output_dir: Path) -> List[Path]:
    copied: List[Path] = []
    for source in sorted(run_root.rglob('*')):
        if not source.is_file() or source.suffix.lower() not in FIGURE_EXTENSIONS:
            continue
        destination = output_dir / source.name
        shutil.copy2(source, destination)
        copied.append(destination)
    return copied


def copy_summary(run_root: Path, output_dir: Path) -> Path | None:
    summary_files = sorted(run_root.rglob('evaluation_summary.txt'))
    if not summary_files:
        return None
    destination = output_dir / 'evaluation_summary.txt'
    shutil.copy2(summary_files[0], destination)
    return destination


def make_summary_chart(metrics: Dict[str, float], output_dir: Path) -> Path | None:
    if not metrics:
        return None

    primary_keys = [
        'Final coverage [%]',
        'Time to target coverage [s]',
        'Total distance [m]',
        'Jain fairness index',
        'Workload imbalance',
        'Total assigned goals',
        'Total reached goals',
        'Total failed goals',
    ]
    items = [(key, metrics[key]) for key in primary_keys if key in metrics]
    if not items:
        items = list(metrics.items())

    labels = [key.replace(' [', '\n[') for key, _ in items]
    values = [value for _, value in items]
    colors = ['#287c8e', '#f2a541', '#6a994e', '#58508d', '#c44e52', '#4c78a8', '#72b7b2', '#b279a2']

    width = max(8.0, 1.15 * len(items))
    fig, ax = plt.subplots(figsize=(width, 5.2), constrained_layout=True)
    bars = ax.bar(labels, values, color=colors[: len(items)], edgecolor='#23313a', linewidth=0.8)

    ax.set_title('Exploration Performance Summary')
    ax.set_ylabel('Metric value')
    ax.grid(axis='y', alpha=0.25)
    ax.set_axisbelow(True)
    ax.tick_params(axis='x', labelrotation=28)

    for bar, value in zip(bars, values):
        ax.annotate(
            f'{value:.3g}',
            xy=(bar.get_x() + bar.get_width() / 2.0, bar.get_height()),
            xytext=(0, 4),
            textcoords='offset points',
            ha='center',
            va='bottom',
            fontsize=9,
        )

    output_path = output_dir / 'performance_summary.png'
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return output_path


def evaluate_run(input_path: Path, output_dir: Path) -> Tuple[List[Path], Dict[str, float]]:
    output_dir.mkdir(parents=True, exist_ok=True)
    run_root, tmp = find_run_root(input_path)
    try:
        copied = copy_existing_figures(run_root, output_dir)
        summary_path = copy_summary(run_root, output_dir)
        metrics = parse_summary(summary_path) if summary_path else {}
        summary_chart = make_summary_chart(metrics, output_dir)
        if summary_chart:
            copied.append(summary_chart)
        return copied, metrics
    finally:
        if tmp is not None:
            tmp.cleanup()


class PerformanceEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('performance_evaluator')
        self.declare_parameter('input_path', str(DEFAULT_INPUT))
        self.declare_parameter('output_dir', str(DEFAULT_OUTPUT))

    def run_once(self) -> None:
        input_path = Path(self.get_parameter('input_path').value).expanduser()
        output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        figures, metrics = evaluate_run(input_path, output_dir)

        self.get_logger().info(f'Evaluation figures written to: {output_dir}')
        self.get_logger().info(f'Generated/copied {len(figures)} figure files.')
        if metrics:
            self.get_logger().info(
                f"Final coverage: {metrics.get('Final coverage [%]', 0.0):.3f}%, "
                f"total distance: {metrics.get('Total distance [m]', 0.0):.3f} m, "
                f"reached goals: {metrics.get('Total reached goals', 0.0):.0f}"
            )


@dataclass
class DroneTrack:
    last_position: Optional[Tuple[float, float, float]] = None
    last_time: Optional[float] = None
    current_goal: Optional[Tuple[float, float, float]] = None
    current_goal_reached: bool = True
    distance: float = 0.0
    assigned_goals: int = 0
    reached_goals: int = 0
    active_time: float = 0.0
    idle_time: float = 0.0


class LivePerformanceEvaluator(Node):
    def __init__(
        self,
        drone_num: int,
        output_dir: Path,
        map_topic: str,
        odom_topic_format: str,
        goal_topic_format: str,
        goal_threshold: float,
        active_speed_threshold: float,
        target_coverage: float,
        duration: float,
    ) -> None:
        super().__init__('live_performance_evaluator')
        self.drone_num = drone_num
        self.output_dir = output_dir
        self.goal_threshold = goal_threshold
        self.active_speed_threshold = active_speed_threshold
        self.target_coverage = target_coverage
        self.duration = duration
        self.start_time: Optional[float] = None

        self.tracks = [DroneTrack() for _ in range(drone_num)]
        self.coverage_series: List[Tuple[float, float]] = []
        self.goal_series: List[Tuple[float, int, int]] = []
        self.odom_subs = []
        self.goal_subs = []

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.on_map,
            10,
        )
        for drone_id in range(drone_num):
            odom_topic = odom_topic_format.format(id=drone_id)
            goal_topic = goal_topic_format.format(id=drone_id)
            self.odom_subs.append(
                self.create_subscription(
                    Odometry,
                    odom_topic,
                    lambda msg, did=drone_id: self.on_odom(did, msg),
                    50,
                )
            )
            self.goal_subs.append(
                self.create_subscription(
                    PoseStamped,
                    goal_topic,
                    lambda msg, did=drone_id: self.on_goal(did, msg),
                    20,
                )
            )
            self.get_logger().info(f'Subscribed odom: {odom_topic}')
            self.get_logger().info(f'Subscribed goal: {goal_topic}')
        self.get_logger().info(f'Subscribed map: {map_topic}')

        self.timer = self.create_timer(1.0, self.on_timer)

    def elapsed(self) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        return now - self.start_time

    def on_timer(self) -> None:
        elapsed = self.elapsed()
        assigned = sum(track.assigned_goals for track in self.tracks)
        reached = sum(track.reached_goals for track in self.tracks)
        self.goal_series.append((elapsed, assigned, reached))
        if self.duration > 0.0 and elapsed >= self.duration:
            self.get_logger().info('Requested duration reached; saving evaluation.')
            raise EvaluationComplete

    def on_map(self, msg: OccupancyGrid) -> None:
        known = sum(1 for value in msg.data if value >= 0)
        coverage = 100.0 * known / len(msg.data) if msg.data else 0.0
        self.coverage_series.append((self.elapsed(), coverage))

    def on_goal(self, drone_id: int, msg: PoseStamped) -> None:
        track = self.tracks[drone_id]
        goal = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        if track.current_goal is None or distance(track.current_goal, goal) > 1e-3:
            track.current_goal = goal
            track.current_goal_reached = False
            track.assigned_goals += 1

    def on_odom(self, drone_id: int, msg: Odometry) -> None:
        track = self.tracks[drone_id]
        t = self.elapsed()
        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

        if track.last_position is not None and track.last_time is not None:
            dt = max(0.0, t - track.last_time)
            step = distance(track.last_position, position)
            track.distance += step
            speed = step / dt if dt > 1e-6 else 0.0
            if speed >= self.active_speed_threshold:
                track.active_time += dt
            else:
                track.idle_time += dt

        if (
            track.current_goal is not None
            and not track.current_goal_reached
            and distance(position, track.current_goal) <= self.goal_threshold
        ):
            track.current_goal_reached = True
            track.reached_goals += 1

        track.last_position = position
        track.last_time = t

    def save(self) -> Dict[str, float]:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        metrics = self.compute_metrics()
        write_live_summary(metrics, self.output_dir)
        plot_coverage(self.coverage_series, self.output_dir / 'coverage_over_time.png')
        plot_distance(self.tracks, self.output_dir / 'per_drone_distance.png')
        plot_workload(self.tracks, self.output_dir / 'workload_balance.png')
        plot_active_idle(self.tracks, self.output_dir / 'active_idle_time.png')
        plot_goals(self.goal_series, self.output_dir / 'assigned_reached_goals.png')
        make_summary_chart(metrics, self.output_dir)
        return metrics

    def compute_metrics(self) -> Dict[str, float]:
        distances = [track.distance for track in self.tracks]
        total_distance = sum(distances)
        final_coverage = self.coverage_series[-1][1] if self.coverage_series else 0.0
        time_to_target = 0.0
        for t, coverage in self.coverage_series:
            if coverage >= self.target_coverage:
                time_to_target = t
                break
        if time_to_target == 0.0 and final_coverage < self.target_coverage:
            time_to_target = math.nan
        assigned = sum(track.assigned_goals for track in self.tracks)
        reached = sum(track.reached_goals for track in self.tracks)
        failed = max(0, assigned - reached)
        fairness = jain_fairness(distances)
        mean_distance = total_distance / len(distances) if distances else 0.0
        imbalance = (
            max(abs(value - mean_distance) for value in distances) / mean_distance
            if mean_distance > 1e-9
            else 0.0
        )
        return {
            'Final coverage [%]': final_coverage,
            'Time to target coverage [s]': time_to_target,
            'Total distance [m]': total_distance,
            'Jain fairness index': fairness,
            'Workload imbalance': imbalance,
            'Total assigned goals': float(assigned),
            'Total reached goals': float(reached),
            'Total failed goals': float(failed),
        }


def distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((aa - bb) ** 2 for aa, bb in zip(a, b)))


def jain_fairness(values: List[float]) -> float:
    denom = len(values) * sum(value * value for value in values)
    if denom <= 1e-9:
        return 1.0
    return sum(values) ** 2 / denom


def write_live_summary(metrics: Dict[str, float], output_dir: Path) -> None:
    lines = ['Exploration Evaluation Summary', '']
    for key, value in metrics.items():
        lines.append(f'{key}: {value:.6f}')
    (output_dir / 'evaluation_summary.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def plot_coverage(series: List[Tuple[float, float]], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    if series:
        ax.plot([x for x, _ in series], [y for _, y in series], color='#287c8e', linewidth=2)
    ax.set_title('Coverage Over Time')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Coverage [%]')
    ax.set_ylim(0, 100)
    ax.grid(alpha=0.25)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_distance(tracks: List[DroneTrack], output_path: Path) -> None:
    labels = [f'Drone {i}' for i in range(len(tracks))]
    values = [track.distance for track in tracks]
    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    ax.bar(labels, values, color='#6a994e', edgecolor='#23313a', linewidth=0.8)
    ax.set_title('Per-Drone Distance')
    ax.set_ylabel('Distance [m]')
    ax.grid(axis='y', alpha=0.25)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_workload(tracks: List[DroneTrack], output_path: Path) -> None:
    labels = [f'Drone {i}' for i in range(len(tracks))]
    assigned = [track.assigned_goals for track in tracks]
    reached = [track.reached_goals for track in tracks]
    x = list(range(len(tracks)))
    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    ax.bar([i - 0.18 for i in x], assigned, width=0.36, label='Assigned', color='#4c78a8')
    ax.bar([i + 0.18 for i in x], reached, width=0.36, label='Reached', color='#72b7b2')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_title('Workload Balance')
    ax.set_ylabel('Goals')
    ax.grid(axis='y', alpha=0.25)
    ax.legend()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_active_idle(tracks: List[DroneTrack], output_path: Path) -> None:
    labels = [f'Drone {i}' for i in range(len(tracks))]
    active = [track.active_time for track in tracks]
    idle = [track.idle_time for track in tracks]
    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    ax.bar(labels, active, label='Active', color='#f2a541')
    ax.bar(labels, idle, bottom=active, label='Idle', color='#b8b8b8')
    ax.set_title('Active and Idle Time')
    ax.set_ylabel('Time [s]')
    ax.grid(axis='y', alpha=0.25)
    ax.legend()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_goals(series: List[Tuple[float, int, int]], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    if series:
        ax.step([x for x, _, _ in series], [y for _, y, _ in series], where='post', label='Assigned')
        ax.step([x for x, _, _ in series], [z for _, _, z in series], where='post', label='Reached')
    ax.set_title('Assigned and Reached Goals')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Goals')
    ax.grid(alpha=0.25)
    ax.legend()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Create performance evaluation figures.')
    parser.add_argument('--input', default=str(DEFAULT_INPUT), help='Run directory or .zip file.')
    parser.add_argument('--output-dir', default=str(DEFAULT_OUTPUT), help='Directory for figure output.')
    parser.add_argument('--live', action='store_true', help='Evaluate a running swarm launch.')
    parser.add_argument('--drone-num', type=int, default=4, help='Number of drones to evaluate.')
    parser.add_argument('--map-topic', default='/swarm_exploration/occupancy_grid')
    parser.add_argument('--odom-topic-format', default='/drone_{id}_visual_slam/odom')
    parser.add_argument('--goal-topic-format', default='/drone_{id}_planning/exploration_goal')
    parser.add_argument('--goal-threshold', type=float, default=0.8)
    parser.add_argument('--active-speed-threshold', type=float, default=0.05)
    parser.add_argument('--target-coverage', type=float, default=95.0)
    parser.add_argument('--duration', type=float, default=0.0, help='Seconds to run live mode; 0 means until Ctrl+C.')
    parser.add_argument('--ros-args', nargs=argparse.REMAINDER, help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> None:
    args = parse_args(argv)

    rclpy.init(args=None)
    if args.live:
        live_node = LivePerformanceEvaluator(
            drone_num=args.drone_num,
            output_dir=Path(args.output_dir).expanduser(),
            map_topic=args.map_topic,
            odom_topic_format=args.odom_topic_format,
            goal_topic_format=args.goal_topic_format,
            goal_threshold=args.goal_threshold,
            active_speed_threshold=args.active_speed_threshold,
            target_coverage=args.target_coverage,
            duration=args.duration,
        )
        try:
            rclpy.spin(live_node)
        except KeyboardInterrupt:
            live_node.get_logger().info('Stopping live evaluation and saving figures.')
        except EvaluationComplete:
            pass
        finally:
            metrics = live_node.save()
            live_node.get_logger().info(
                f"Saved live evaluation to {Path(args.output_dir).expanduser()} "
                f"with final coverage {metrics.get('Final coverage [%]', 0.0):.3f}%"
            )
            live_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    else:
        node = PerformanceEvaluator()
        try:
            node.set_parameters([
                Parameter('input_path', Parameter.Type.STRING, args.input),
                Parameter('output_dir', Parameter.Type.STRING, args.output_dir),
            ])
            node.run_once()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
