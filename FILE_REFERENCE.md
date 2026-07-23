# Project File Reference

Quick reference for every file in this project and what it does.

## Root Files
| File | Purpose |
|------|---------|
| `launch_sim.sh` | Main launcher — opens 8 terminals, use `--auto` for autonomous mode, `--algo X` to pick algorithm |
| `README.md` | Project documentation |
| `requirements.txt` | Python dependency list |
| `presentation.html` | Project presentation slides |
| `BTP_report__winter2026.pdf` | BTP report PDF |

## planners/ — Path Planning Algorithms
| File | Algorithm | Status |
|------|-----------|--------|
| `navigator_astar.py` | A* (grid search + heuristic) | Done |
| `navigator_dijkstra.py` | Dijkstra (A* without heuristic) | Done |
| `navigator_bellman_ford.py` | Bellman Ford (edge relaxation) | Done |
| `navigator_prm.py` | PRM (Probabilistic Roadmap) | Done |
| `navigator_rrt.py` | RRT (Rapidly-exploring Random Tree) | Done |
| `navigator_theta_star.py` | Theta* (any-angle A*) | Done |
| `navigator_benchmark.py` | Visual Benchmark (runs all 6, colored paths) | Done |
| `planner_3d.py` | 2.5D multi-layer planner (3 altitude layers, A* + Theta*) | Done |

## core/ — Shared Drone Infrastructure
| File | Purpose |
|------|---------|
| `keyboard_control.py` | Manual WASD drone controller with position display |
| `tf_broadcaster.py` | Publishes drone position as TF transforms (map -> base_link -> camera) |
| `pointcloud_filter.py` | Voxel Grid + Statistical Outlier Removal filter (307K -> 170 pts) |
| `room_scanner.py` | Automated room scanning flight patterns |
| `octomap_3d_query.py` | 3D OctoMap voxel query interface (hash set, O(1) lookups) |
| `semantic_pointcloud.py` | YOLOv8n-seg + depth fusion → colored XYZRGB PointCloud2 for semantic OctoMap |

## worlds/ — Gazebo SDF Environments
| File | Description |
|------|-------------|
| `house_3room.sdf` | Active: 18x12m house, 3 rooms, staggered doorways, 12 furniture items + ceiling fans, hanging light, low pipe |
| `indoor_10x8x3.sdf` | Legacy: 10x8m single room with 3 color-coded obstacles |

## config/ — Configuration
| File | Purpose |
|------|---------|
| `drone_rviz.rviz` | RViz2 layout — OctoMap (grey voxels) + paths (colored) + benchmark markers |
| `octomap_params.yaml` | OctoMap server resolution, range, thresholds |

## benchmark/ — Automated Performance Comparison
| File | Purpose |
|------|---------|
| `save_map.py` | ROS 2 node that saves OctoMap's OccupancyGrid to `saved_map.npz` |
| `planner_library.py` | All 6 algorithms as pure Python functions (no ROS deps) |
| `run_benchmark.py` | Runs all 6 planners on saved map, outputs table + CSV |

## legacy/ — Old / Experimental Scripts
| File | Purpose |
|------|---------|
| `offboard_waypoint_nav.py` | Blind waypoint navigator (proved blind nav crashes into obstacles) |
| `analyze_hover.py` | Post-flight hover stability analyzer |
| `debug_tf.py` | TF transform debugging tool |

## Demo_Pic/ — Screenshots
| File | Shows |
|------|-------|
| `Autonomous_AStar_Nav.png` | A* path in single room (RViz) |
| `House_3Room_Navigation.png` | 3-room house with cross-room A* path |
| `Ref.png` | Full system view (Gazebo + RViz + Terminal) |
| `astar.png` | A* algorithm path (Phase 1) |
| `prm.png` | PRM algorithm path (Phase 1) |
| `rrt.png` | RRT algorithm path (Phase 1) |
| `theta.png` | Theta* algorithm path (Phase 1) |
| `Path 1.png` | Individual path planning demo |
| `Path 2.png` | Individual path planning demo (different goal) |
| `Benchmark 1.png` | Visual benchmark — all 6 colored paths |
| `Benchmark 2.png` | Visual benchmark — different goal |

## docs/ — Documentation
| File | Content |
|------|---------|
| `algorithm_comparison_plan.md` | Detailed plan comparing 5 path planning algorithms |
| `BTP_Legacy_Notes.md` | Legacy BTP notes |
