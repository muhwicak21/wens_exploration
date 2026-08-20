#!/usr/bin/env python3
"""Select and summarize exploration-time results only."""

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


MODE_LABELS: Dict[str, str] = {
    'frontier': 'Frontier',
    'frontier_separation': 'Frontier + Separation',
    'frontier_hgrid': 'Frontier + HGrid',
    'frontier_hgrid_role': 'Frontier + HGrid + Role',
}
MODE_ORDER = list(MODE_LABELS.keys())


def load_rows(results_root: Path) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for summary_path in sorted(results_root.glob('*/summary.yaml')):
        with summary_path.open('r', encoding='utf-8') as stream:
            summary = yaml.safe_load(stream) or {}
        experiment = summary.get('experiment', {}) or {}
        result = summary.get('result', {}) or {}
        rows.append({
            'mode': experiment.get('exploration_mode', ''),
            'success': bool(result.get('success', False)),
            'exploration_time': result.get('exploration_time'),
            'final_coverage': result.get('final_coverage'),
        })
    return rows


def finite_values(values: Iterable[object]) -> List[float]:
    result = []
    for value in values:
        if value is None:
            continue
        value = float(value)
        if math.isfinite(value):
            result.append(value)
    return result


def summarize(rows: List[Dict[str, object]]) -> List[Dict[str, object]]:
    summary_rows: List[Dict[str, object]] = []
    for mode in MODE_ORDER:
        mode_rows = [row for row in rows if row['mode'] == mode]
        success_rows = [row for row in mode_rows if row['success']]
        timeout_rows = [row for row in mode_rows if not row['success']]
        times = finite_values(row['exploration_time'] for row in success_rows)
        final_coverages = finite_values(row['final_coverage'] for row in mode_rows)
        summary_rows.append({
            'mode': mode,
            'total_runs': len(mode_rows),
            'successful_runs': len(success_rows),
            'timeout_runs': len(timeout_rows),
            'mean_exploration_time': mean(times) if times else math.nan,
            'std_exploration_time': stdev(times) if len(times) > 1 else math.nan,
            'min_exploration_time': min(times) if times else math.nan,
            'max_exploration_time': max(times) if times else math.nan,
            'mean_final_coverage': mean(final_coverages) if final_coverages else math.nan,
        })
    return summary_rows


def write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    fieldnames = [
        'mode',
        'total_runs',
        'successful_runs',
        'timeout_runs',
        'mean_exploration_time',
        'std_exploration_time',
        'min_exploration_time',
        'max_exploration_time',
        'mean_final_coverage',
    ]
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def finite_or_zero(value: object) -> float:
    value = float(value)
    return value if math.isfinite(value) else 0.0


def write_plot(path: Path, rows: List[Dict[str, object]]) -> None:
    labels = [MODE_LABELS[row['mode']] for row in rows]
    means = [finite_or_zero(row['mean_exploration_time']) for row in rows]
    stds = [finite_or_zero(row['std_exploration_time']) for row in rows]

    fig, ax = plt.subplots(figsize=(10, 5.5), constrained_layout=True)
    bars = ax.bar(
        labels,
        means,
        yerr=stds,
        capsize=5,
        color='#287c8e',
        edgecolor='#23313a',
        linewidth=0.8,
    )
    ax.set_ylabel('Mean exploration time to target coverage [s]')
    ax.set_title('Exploration Time Comparison')
    ax.grid(axis='y', alpha=0.25)
    ax.set_axisbelow(True)
    ax.tick_params(axis='x', rotation=15)
    for bar, row in zip(bars, rows):
        label = f"n={row['successful_runs']}"
        if row['timeout_runs']:
            label += f", timeout={row['timeout_runs']}"
        ax.annotate(
            label,
            xy=(bar.get_x() + bar.get_width() / 2.0, bar.get_height()),
            xytext=(0, 4),
            textcoords='offset points',
            ha='center',
            va='bottom',
            fontsize=9,
        )
    fig.savefig(path, dpi=300)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description='Summarize exploration-time results.')
    parser.add_argument(
        '--results-root',
        default='/root/catkin_ws/src/eval/results',
        help='Directory containing timestamped exploration-time result folders.',
    )
    args = parser.parse_args()

    results_root = Path(args.results_root).expanduser()
    rows = load_rows(results_root)
    if not rows:
        raise SystemExit(f'No summary.yaml files found under {results_root}')

    summary_rows = summarize(rows)
    write_csv(results_root / 'exploration_time_comparison.csv', summary_rows)
    write_plot(results_root / 'exploration_time_comparison.png', summary_rows)
    print(f'Wrote exploration-time comparison to {results_root}')


if __name__ == '__main__':
    main()
