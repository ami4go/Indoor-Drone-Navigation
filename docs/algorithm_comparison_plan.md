# Multi-Algorithm Path Planning — Comparison Study

Implement 4 additional path planning algorithms alongside the existing A*, then compare results on the same 3-room house environment.

---

## Open Questions

> [!IMPORTANT]
> **"pie\*" / "π\*" is NOT a real path planning algorithm.** There is no standard algorithm called Pi\* in robotics. Did you mean one of these instead?
> - **D\* Lite** — Dynamic A\* that replans when new obstacles are discovered
> - **Potential Fields** — Attractive/repulsive forces (very different approach)
> - **Phi\*** — A variant of Theta\* with interpolation
>
> **I'll proceed with D\* Lite** as the 5th algorithm since it's the most commonly studied "star" variant alongside A\* and Theta\*, and it makes the comparison more interesting (static vs. dynamic planning). Let me know if you meant something else!

---

## 🧠 Algorithm Explanations (4–5 Points Each)

### 1. A\* (Already Implemented ✅)
1. **Grid-based search** — operates on a discretized 2D grid where each cell is either free or occupied
2. **Uses f = g + h** — `g` is the exact cost from start, `h` is the Euclidean distance heuristic to the goal
3. **Priority queue** — always expands the cell with the lowest `f` score, directing search toward the goal
4. **Guarantees optimal path** — because the Euclidean heuristic never overestimates the true distance
5. **8-connected grid** — explores up/down/left/right + 4 diagonals; diagonal cost = √2 ≈ 1.414

---

### 2. PRM (Probabilistic Roadmap)
1. **Sampling-based, NOT grid-based** — instead of checking every grid cell, it randomly scatters N points (e.g., 500) in free space, then connects nearby points with straight-line edges (only if the line doesn't cross an obstacle)
2. **Two-phase algorithm** — *Phase 1 (Construction):* build the roadmap graph once. *Phase 2 (Query):* connect start and goal to the roadmap, then run Dijkstra/A\* on the graph
3. **Multi-query efficient** — once the roadmap is built, you can query different start/goal pairs instantly (unlike A\* which re-searches the entire grid each time)
4. **Probabilistically complete** — given enough random samples, it WILL find a path if one exists, but it's not guaranteed to find the *shortest* path
5. **Weakness in narrow passages** — random samples may fail to land inside tight doorways, requiring many more samples to find the path through corridors

---

### 3. RRT (Rapidly-exploring Random Tree)
1. **Tree-based, grows from start** — starts a tree at the start position, then repeatedly picks a random point in space and extends the nearest tree node toward it by a fixed step size
2. **Biased toward unexplored space** — because random samples are uniform, the tree naturally grows toward large unexplored areas, giving good coverage
3. **Single-query algorithm** — builds a new tree for every start/goal pair (unlike PRM which reuses its roadmap)
4. **Very fast but NOT optimal** — finds *a* path quickly, but it's usually jagged and much longer than the true shortest path. RRT\* (the improved version) converges to optimal but takes longer
5. **Handles high-dimensional spaces well** — commonly used for robotic arms (6+ DOF), but works fine on our 2D grid too

---

### 4. Theta\* (Any-Angle A\*)
1. **Extension of A\*** — uses the same grid and priority queue as A\*, but with one key difference: it checks **line-of-sight** between a node's grandparent and the node itself
2. **Any-angle paths** — if there's a clear line from grandparent to current node (no obstacles in between), it bypasses the parent, creating a shorter diagonal shortcut. A\* is restricted to 45°/90° grid angles; Theta\* can produce any angle
3. **Produces smoother, shorter paths** — paths hug the true shortest geometric route rather than following grid lines
4. **Same optimality guarantee as A\*** — still uses f = g + h with Euclidean heuristic, but g-costs are computed along actual straight-line distances (not grid steps)
5. **Line-of-sight check** — uses Bresenham's line algorithm to verify that a straight line between two cells doesn't pass through any obstacle cell

---

### 5. D\* Lite (Dynamic A\*)
1. **Replanning algorithm** — designed for situations where the robot discovers NEW obstacles while moving (e.g., a door is suddenly blocked). Instead of re-running A\* from scratch, D\* Lite efficiently updates only the affected part of the path
2. **Plans backwards** — builds the search tree from the GOAL to the START. When new obstacles appear near the robot, only the nearby nodes need updating (the goal-side of the tree is unchanged)
3. **Incremental search** — maintains a priority queue across multiple planning calls. After discovering a new obstacle, it only reprocesses nodes affected by the change, making replanning much faster than full A\*
4. **Used in Mars Rovers** — NASA's Mars Exploration Rovers used D\* for terrain navigation, where new obstacles (rocks, slopes) are discovered as the rover moves
5. **Same quality as A\*** — produces optimal paths; the advantage is purely in *speed of replanning*, not path quality

---

## Proposed Changes

The key insight: **all 5 algorithms share the same drone control logic**. Only the `plan()` method changes. So we'll create 5 separate files that each swap the planner class.

### Architecture

```
autonomous_navigator.py        ← A* (existing, unchanged)
navigator_prm.py               ← PRM planner
navigator_rrt.py               ← RRT planner  
navigator_theta_star.py        ← Theta* planner
navigator_dstar_lite.py        ← D* Lite planner
```

Each file is a **complete, standalone script** (copy of `autonomous_navigator.py` with the `SensorMapPlanner` class replaced by the new algorithm). The user picks which one to run.

---

### [NEW] [navigator_prm.py](file:///home/amit/Desktop/Drone_IP/navigator_prm.py)

**PRM Planner replaces `SensorMapPlanner`:**
- Same `__init__` (parse OccupancyGrid, inflate obstacles)
- `plan()`: scatter 500 random points in free space → connect neighbors within 1.5m radius (collision-checked) → connect start & goal → run Dijkstra on the graph
- Uses `scipy.spatial.KDTree` for efficient nearest-neighbor queries
- Helper: `_collision_free_line(p1, p2)` using Bresenham's algorithm

---

### [NEW] [navigator_rrt.py](file:///home/amit/Desktop/Drone_IP/navigator_rrt.py)

**RRT Planner:**
- `plan()`: grow tree from start, max 5000 iterations, step size = 0.3m
- Each iteration: random sample → find nearest tree node → extend toward sample by step size → add to tree if collision-free
- Goal bias: 10% of samples are the goal itself (to speed up convergence)
- Once goal is reached: trace back through tree, then simplify path

---

### [NEW] [navigator_theta_star.py](file:///home/amit/Desktop/Drone_IP/navigator_theta_star.py)

**Theta\* Planner:**
- Same A\* loop but with line-of-sight (LOS) check added
- When expanding a neighbor: if LOS exists from grandparent → neighbor, skip the parent (shorter path)
- LOS check: `_line_of_sight(r1,c1, r2,c2)` using Bresenham's line algorithm
- Produces any-angle paths (smoother than A\*)

---

### [NEW] [navigator_dstar_lite.py](file:///home/amit/Desktop/Drone_IP/navigator_dstar_lite.py)

**D\* Lite Planner:**
- Plans from goal → start (reverse search)
- On first call: full search (similar to A\*)
- Maintains `rhs` (right-hand-side) values and a priority queue with keys `[k1, k2]`
- When the drone discovers new obstacles mid-flight, calls `update_obstacles()` to incrementally fix the path without re-searching everything

---

### [MODIFY] [launch_sim.sh](file:///home/amit/Desktop/Drone_IP/launch_sim.sh)

Add a `--algo` flag so the user can pick the algorithm:
```bash
./launch_sim.sh --auto               # default: A*
./launch_sim.sh --auto --algo prm    # PRM
./launch_sim.sh --auto --algo rrt    # RRT
./launch_sim.sh --auto --algo theta  # Theta*
./launch_sim.sh --auto --algo dstar  # D* Lite
```

---

## Verification Plan

### Automated Tests
For each algorithm, run the same test:
1. Launch `./launch_sim.sh --auto --algo <name>`
2. Wait for exploration to complete
3. Set the same goal position in RViz (e.g., Living Room corner at (-7, 3))
4. Record: **path length**, **number of waypoints**, **planning time**, **success/failure**

### Comparison Metrics

| Metric | What It Measures |
|--------|-----------------|
| Path Length (m) | Total distance of the planned path |
| Waypoints | Number of waypoints after simplification |
| Planning Time (ms) | Time to compute the path |
| Success Rate | Whether the algorithm finds a path through doorways |
| Path Smoothness | Number of sharp turns in the path |

### Expected Results

| Algorithm | Path Length | Speed | Optimal? | Path Quality |
|-----------|-----------|-------|----------|-------------|
| A\* | Shortest on grid | Medium | Yes (grid-optimal) | Staircase-like |
| PRM | Near-optimal | Fast (after build) | No | Smooth, straight segments |
| RRT | Usually longer | Fastest | No | Jagged, needs smoothing |
| Theta\* | True shortest | Medium | Yes (any-angle) | Smoothest |
| D\* Lite | Same as A\* | Fast (replan) | Yes | Same as A\* |
