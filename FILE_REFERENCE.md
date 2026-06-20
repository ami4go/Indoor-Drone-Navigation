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
| `navigator_prm.py` | PRM (Probabilistic Roadmap) | Done |
| `navigator_rrt.py` | RRT (Rapidly-exploring Random Tree) | Done |
| `navigator_theta_star.py` | Theta* (any-angle A*) | Done |
| `navigator_dstar_lite.py` | D* Lite (dynamic replanning) | Pending |

## core/ — Shared Drone Infrastructure
| File | Purpose |
|------|---------|
| `keyboard_control.py` | Manual WASD drone controller with position display |
| `tf_broadcaster.py` | Publishes drone position as TF transforms (map -> base_link -> camera) |
| `pointcloud_filter.py` | Voxel Grid + Statistical Outlier Removal filter (307K -> 170 pts) |
| `room_scanner.py` | Automated room scanning flight patterns |

## worlds/ — Gazebo SDF Environments
| File | Description |
|------|-------------|
| `house_3room.sdf` | Active: 18x12m house, 3 rooms, staggered doorways, 12 furniture items |
| `indoor_10x8x3.sdf` | Legacy: 10x8m single room with 3 color-coded obstacles |

## config/ — Configuration
| File | Purpose |
|------|---------|
| `drone_rviz.rviz` | RViz2 layout — OctoMap (grey voxels) + path (cyan line) |
| `octomap_params.yaml` | OctoMap server resolution, range, thresholds |

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

## docs/ — Documentation
| File | Content |
|------|---------|
| `algorithm_comparison_plan.md` | Detailed plan comparing 5 path planning algorithms |
| `BTP_Legacy_Notes.md` | Legacy BTP notes |
