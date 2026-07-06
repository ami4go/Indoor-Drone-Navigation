#!/usr/bin/env python3
"""
=============================================================================
 BENCHMARK RUNNER — Compare all 6 path planning algorithms on saved map
 File: benchmark/run_benchmark.py
=============================================================================

 Usage:
   1. First save a map:
        python3 ~/Desktop/Drone_IP/benchmark/save_map.py
   2. Then run this:
        python3 ~/Desktop/Drone_IP/benchmark/run_benchmark.py

 What it does:
   - Loads the saved OccupancyGrid from benchmark/saved_map.npz
   - Runs all 6 algorithms on the SAME map with the SAME start/goal pairs
   - Measures: planning time, path length, waypoints, nodes explored, smoothness
   - Outputs a formatted comparison table + saves results to CSV

 No ROS dependencies — runs as a plain Python script.

=============================================================================
"""

import os
import sys
import csv
import time
import numpy as np

# Add parent dir so we can import planner_library
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from planner_library import PLANNERS, prepare_grid


# ─────────────────────────────────────────────────────────────────────────────
#  TEST CASES — Same start/goal pairs for all algorithms
# ─────────────────────────────────────────────────────────────────────────────
# Format: (name, start_xy, goal_xy)
# Based on the 3-room house: Living(-9 to -3) | Bedroom(-3 to +3) | Study(+3 to +9)
TEST_CASES = [
    ("Bedroom → Study (cross-room)",     (0.0, 0.0),   (6.0, 2.0)),
    ("Bedroom → Living (through door1)", (0.0, 0.0),   (-6.0, 3.0)),
    ("Living → Study (full traverse)",   (-6.0, 2.0),  (6.0, -2.0)),
    ("Short: within Bedroom",            (0.0, 2.0),   (0.0, -2.0)),
    ("Diagonal: corner to corner",       (-7.0, 4.0),  (7.0, -4.0)),
]


def load_map(map_path):
    """Load saved OccupancyGrid from .npz file."""
    if not os.path.exists(map_path):
        print(f"\n  Error: No saved map found at {map_path}")
        print(f"  Run save_map.py first while the simulation is running.\n")
        sys.exit(1)

    data = np.load(map_path)
    raw = data['data']
    width = int(data['width'])
    height = int(data['height'])
    resolution = float(data['resolution'])
    origin_x = float(data['origin_x'])
    origin_y = float(data['origin_y'])

    print(f"  Map loaded: {width}x{height} cells, "
          f"res={resolution:.2f}m, "
          f"origin=({origin_x:.1f}, {origin_y:.1f})")

    return raw, width, height, resolution, origin_x, origin_y


def print_header():
    print()
    print("╔" + "═"*78 + "╗")
    print("║" + "  PATH PLANNING ALGORITHM BENCHMARK".center(78) + "║")
    print("║" + "  6 algorithms × 5 test cases on identical sensor map".center(78) + "║")
    print("╚" + "═"*78 + "╝")
    print()


def print_table(results, test_name):
    """Print a formatted comparison table for one test case."""
    print(f"\n  ┌{'─'*74}┐")
    print(f"  │ {test_name:<72} │")
    print(f"  ├{'─'*14}┬{'─'*11}┬{'─'*11}┬{'─'*9}┬{'─'*12}┬{'─'*12}┤")
    print(f"  │ {'Algorithm':<12} │ {'Time(ms)':>9} │ {'Path(m)':>9} │ {'  WPs':>7} │ {'  Nodes':>10} │ {'Smooth(°)':>10} │")
    print(f"  ├{'─'*14}┼{'─'*11}┼{'─'*11}┼{'─'*9}┼{'─'*12}┼{'─'*12}┤")

    for name, metrics in results:
        if metrics['success']:
            print(f"  │ {name:<12} │ {metrics['planning_time_ms']:>9.1f} │ "
                  f"{metrics['path_length_m']:>9.2f} │ {metrics['waypoint_count']:>7} │ "
                  f"{metrics['nodes_explored']:>10,} │ {metrics['smoothness_deg']:>10.1f} │")
        else:
            print(f"  │ {name:<12} │ {metrics['planning_time_ms']:>9.1f} │ "
                  f"{'FAILED':>9} │ {'  -':>7} │ "
                  f"{metrics['nodes_explored']:>10,} │ {'    -':>10} │")

    print(f"  └{'─'*14}┴{'─'*11}┴{'─'*11}┴{'─'*9}┴{'─'*12}┴{'─'*12}┘")


def print_summary(all_results):
    """Print an overall summary averaging across all test cases."""
    print(f"\n{'='*80}")
    print("  OVERALL SUMMARY (averaged across all test cases)")
    print(f"{'='*80}")

    algo_totals = {}
    for test_name, results in all_results:
        for algo_name, metrics in results:
            if algo_name not in algo_totals:
                algo_totals[algo_name] = {
                    'time': [], 'length': [], 'wps': [],
                    'nodes': [], 'smooth': [], 'success': 0, 'total': 0
                }
            algo_totals[algo_name]['total'] += 1
            if metrics['success']:
                algo_totals[algo_name]['success'] += 1
                algo_totals[algo_name]['time'].append(metrics['planning_time_ms'])
                algo_totals[algo_name]['length'].append(metrics['path_length_m'])
                algo_totals[algo_name]['wps'].append(metrics['waypoint_count'])
                algo_totals[algo_name]['nodes'].append(metrics['nodes_explored'])
                algo_totals[algo_name]['smooth'].append(metrics['smoothness_deg'])

    print(f"\n  ┌{'─'*14}┬{'─'*11}┬{'─'*11}┬{'─'*9}┬{'─'*12}┬{'─'*12}┬{'─'*10}┐")
    print(f"  │ {'Algorithm':<12} │ {'Avg ms':>9} │ {'Avg m':>9} │ {'Avg WPs':>7} │ {'Avg Nodes':>10} │ {'Avg Smth°':>10} │ {'Success':>8} │")
    print(f"  ├{'─'*14}┼{'─'*11}┼{'─'*11}┼{'─'*9}┼{'─'*12}┼{'─'*12}┼{'─'*10}┤")

    for algo_name in PLANNERS.keys():
        t = algo_totals.get(algo_name)
        if t is None or t['success'] == 0:
            print(f"  │ {algo_name:<12} │ {'   -':>9} │ {'   -':>9} │ {'  -':>7} │ {'     -':>10} │ {'     -':>10} │ {'0/' + str(t['total'] if t else 0):>8} │")
            continue

        avg_t = sum(t['time']) / len(t['time'])
        avg_l = sum(t['length']) / len(t['length'])
        avg_w = sum(t['wps']) / len(t['wps'])
        avg_n = sum(t['nodes']) / len(t['nodes'])
        avg_s = sum(t['smooth']) / len(t['smooth'])
        sr = f"{t['success']}/{t['total']}"

        print(f"  │ {algo_name:<12} │ {avg_t:>9.1f} │ {avg_l:>9.2f} │ {avg_w:>7.0f} │ {avg_n:>10,.0f} │ {avg_s:>10.1f} │ {sr:>8} │")

    print(f"  └{'─'*14}┴{'─'*11}┴{'─'*11}┴{'─'*9}┴{'─'*12}┴{'─'*12}┴{'─'*10}┘")


def save_csv(all_results, csv_path):
    """Save all results to a CSV file."""
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Test Case', 'Algorithm', 'Time(ms)', 'Path Length(m)',
                         'Waypoints', 'Nodes Explored', 'Smoothness(deg)', 'Success'])
        for test_name, results in all_results:
            for algo_name, metrics in results:
                writer.writerow([
                    test_name, algo_name,
                    f"{metrics['planning_time_ms']:.1f}",
                    f"{metrics['path_length_m']:.2f}",
                    metrics['waypoint_count'],
                    metrics['nodes_explored'],
                    f"{metrics['smoothness_deg']:.1f}",
                    metrics['success']
                ])
    print(f"\n  Results saved to: {csv_path}")


def main():
    print_header()

    # Load map
    map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'saved_map.npz')
    raw, width, height, resolution, origin_x, origin_y = load_map(map_path)

    # Prepare the inflated grid ONCE (all planners use the same grid)
    grid = np.where(raw.reshape((height, width)) > 50, 1, 0).astype(np.uint8)
    from planner_library import inflate_grid
    inflated = inflate_grid(grid, resolution, 0.25)

    print(f"  Grid inflated: {np.sum(inflated == 1)} obstacle cells")
    print(f"  Test cases: {len(TEST_CASES)}")
    print(f"  Algorithms: {len(PLANNERS)}")
    print(f"  Total runs: {len(TEST_CASES) * len(PLANNERS)}")

    all_results = []

    for test_name, start_xy, goal_xy in TEST_CASES:
        print(f"\n{'─'*80}")
        print(f"  Running: {test_name}")
        print(f"  Start: ({start_xy[0]:.1f}, {start_xy[1]:.1f}) → "
              f"Goal: ({goal_xy[0]:.1f}, {goal_xy[1]:.1f})")

        results = []
        for algo_name, plan_fn in PLANNERS.items():
            path, metrics = plan_fn(
                inflated, resolution, origin_x, origin_y,
                start_xy, goal_xy, margin=0.0  # already inflated
            )
            status = "✓" if metrics['success'] else "✗"
            print(f"    {status} {algo_name:<12} — {metrics['planning_time_ms']:.1f}ms", end="")
            if metrics['success']:
                print(f", {metrics['path_length_m']:.2f}m, {metrics['waypoint_count']} wps")
            else:
                print(" — FAILED")
            results.append((algo_name, metrics))

        print_table(results, test_name)
        all_results.append((test_name, results))

    # Overall summary
    print_summary(all_results)

    # Save to CSV
    csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'benchmark_results.csv')
    save_csv(all_results, csv_path)

    print("\n  Benchmark complete!\n")


if __name__ == '__main__':
    main()
