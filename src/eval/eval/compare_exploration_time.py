#!/usr/bin/env python3
"""Compare exploration completion time results across modes and runs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from statistics import mean, stdev
from typing import Dict, Iterable, List

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

try:
    import pandas as pd
except ImportError:
    pd = None


MODE_LABELS: Dict[str, str] = {
    'frontier': 'Frontier',
    'frontier_separation': 'Frontier + Separation',
    'frontier_hgrid': 'Frontier + HGrid',
    'frontier_hgrid_role': 'Frontier + HGrid + Role',
}
MODE_ORDER = list(MODE_LABELS.keys())


def load_results(results_root: Path) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for result_path in sorted(results_root.glob('**/result.yaml')):
        with result_path.open('r', encoding='utf-8') as stream:
            result = yaml.safe_load(stream) or {}
        rows.append({
            'exploration_mode': result.get('exploration_mode', ''),
            'experiment_run': int(result.get('experiment_run', 0)),
            'status': result.get('status', ''),
            'exploration_time': float(result.get('exploration_time', 0.0)),
            'final_coverage': float(result.get('final_coverage', 0.0)),
            'result_path': str(result_path),
        })
    return rows


def write_csv(path: Path, rows: List[Dict[str, object]], fieldnames: Iterable[str]) -> None:
    fieldnames = list(fieldnames)
    if pd is not None:
        pd.DataFrame(rows)[fieldnames].to_csv(path, index=False)
        return
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, '') for field in fieldnames})


def write_comparison(rows: List[Dict[str, object]], results_root: Path) -> None:
    comparison = sorted(rows, key=lambda row: (row['exploration_mode'], row['experiment_run']))
    write_csv(
        results_root / 'exploration_time_comparison.csv',
        comparison,
        ['exploration_mode', 'experiment_run', 'status', 'exploration_time', 'final_coverage'],
    )


def write_summary(result_rows: List[Dict[str, object]], results_root: Path) -> List[Dict[str, object]]:
    if pd is not None:
        df = pd.DataFrame(result_rows)
        summary_rows = []
        for mode in MODE_ORDER:
            group = df[df['exploration_mode'] == mode]
            success = group[group['status'] == 'SUCCESS']
            timeout = group[group['status'] == 'TIMEOUT']
            summary_rows.append({
                'exploration_mode': mode,
                'successful_runs': int(len(success)),
                'timeout_runs': int(len(timeout)),
                'mean_exploration_time': success['exploration_time'].mean(),
                'std_exploration_time': success['exploration_time'].std(ddof=1),
                'mean_final_coverage': group['final_coverage'].mean(),
            })
        summary = pd.DataFrame(summary_rows)
        summary.to_csv(results_root / 'exploration_time_summary.csv', index=False)
        return summary_rows

    summary_rows = []
    for mode in MODE_ORDER:
        group = [row for row in result_rows if row['exploration_mode'] == mode]
        success = [row for row in group if row['status'] == 'SUCCESS']
        timeout = [row for row in group if row['status'] == 'TIMEOUT']
        success_times = [float(row['exploration_time']) for row in success]
        final_coverages = [float(row['final_coverage']) for row in group]
        summary_rows.append({
            'exploration_mode': mode,
            'successful_runs': int(len(success)),
            'timeout_runs': int(len(timeout)),
            'mean_exploration_time': mean(success_times) if success_times else math.nan,
            'std_exploration_time': stdev(success_times) if len(success_times) > 1 else math.nan,
            'mean_final_coverage': mean(final_coverages) if final_coverages else math.nan,
        })
    write_csv(
        results_root / 'exploration_time_summary.csv',
        summary_rows,
        [
            'exploration_mode',
            'successful_runs',
            'timeout_runs',
            'mean_exploration_time',
            'std_exploration_time',
            'mean_final_coverage',
        ],
    )
    return summary_rows


def plot_single_run(result_rows: List[Dict[str, object]], results_root: Path, target_coverage: float) -> None:
    rows = []
    for mode in MODE_ORDER:
        group = sorted(
            [row for row in result_rows if row['exploration_mode'] == mode],
            key=lambda row: row['experiment_run'])
        if not group:
            rows.append({'mode': mode, 'time': 0.0, 'status': 'MISSING', 'coverage': 0.0})
        else:
            first = group[0]
            rows.append({
                'mode': mode,
                'time': float(first['exploration_time']),
                'status': str(first['status']),
                'coverage': float(first['final_coverage']),
            })

    fig, ax = plt.subplots(figsize=(10, 5.5), constrained_layout=True)
    labels = [MODE_LABELS[row['mode']] for row in rows]
    times = [row['time'] for row in rows]
    colors = ['#287c8e' if row['status'] == 'SUCCESS' else '#c44e52' for row in rows]
    bars = ax.bar(labels, times, color=colors, edgecolor='#23313a', linewidth=0.8)
    ax.set_ylabel(f'Exploration time to {target_coverage:.0f}% coverage [s]')
    ax.set_title('Exploration Time Comparison')
    ax.grid(axis='y', alpha=0.25)
    ax.set_axisbelow(True)
    ax.tick_params(axis='x', rotation=15)
    for bar, row in zip(bars, rows):
        label = f"{row['time']:.1f}s"
        if row['status'] == 'TIMEOUT':
            label = f"TIMEOUT\n{row['coverage']:.1f}%"
        elif row['status'] == 'MISSING':
            label = 'MISSING'
        ax.annotate(
            label,
            xy=(bar.get_x() + bar.get_width() / 2.0, bar.get_height()),
            xytext=(0, 4),
            textcoords='offset points',
            ha='center',
            va='bottom',
            fontsize=9,
        )
    fig.savefig(results_root / 'exploration_time_comparison.png', dpi=300)
    plt.close(fig)


def finite_or_zero(value: object) -> float:
    value = float(value)
    return value if math.isfinite(value) else 0.0


def plot_mean_std(summary: List[Dict[str, object]], results_root: Path, target_coverage: float) -> None:
    labels = [MODE_LABELS[row['exploration_mode']] for row in summary]
    times = [finite_or_zero(row['mean_exploration_time']) for row in summary]
    errors = [finite_or_zero(row['std_exploration_time']) for row in summary]

    fig, ax = plt.subplots(figsize=(10, 5.5), constrained_layout=True)
    bars = ax.bar(
        labels,
        times,
        yerr=errors,
        capsize=5,
        color='#287c8e',
        edgecolor='#23313a',
        linewidth=0.8,
    )
    ax.set_ylabel(f'Mean exploration time to {target_coverage:.0f}% coverage [s]')
    ax.set_title('Exploration Time Mean and Standard Deviation')
    ax.grid(axis='y', alpha=0.25)
    ax.set_axisbelow(True)
    ax.tick_params(axis='x', rotation=15)
    for bar, row in zip(bars, summary):
        label = f"n={int(row['successful_runs'])}"
        if int(row['timeout_runs']) > 0:
            label += f", timeout={int(row['timeout_runs'])}"
        ax.annotate(
            label,
            xy=(bar.get_x() + bar.get_width() / 2.0, bar.get_height()),
            xytext=(0, 4),
            textcoords='offset points',
            ha='center',
            va='bottom',
            fontsize=9,
        )
    fig.savefig(results_root / 'exploration_time_mean_std.png', dpi=300)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description='Compare exploration completion times.')
    parser.add_argument(
        '--results-root',
        default='/root/catkin_ws/src/eval/results',
        help='Directory containing per-run result.yaml files.',
    )
    parser.add_argument('--target-coverage', type=float, default=95.0)
    args = parser.parse_args()

    results_root = Path(args.results_root).expanduser()
    results_root.mkdir(parents=True, exist_ok=True)
    rows = load_results(results_root)
    if not rows:
        raise SystemExit(f'No result.yaml files found under {results_root}')

    write_comparison(rows, results_root)
    summary = write_summary(rows, results_root)
    plot_single_run(rows, results_root, args.target_coverage)
    plot_mean_std(summary, results_root, args.target_coverage)
    print(f'Wrote comparison outputs to {results_root}')


if __name__ == '__main__':
    main()
