#!/usr/bin/env python3
"""
=============================================================================
 PLANNER LIBRARY — All 6 path planning algorithms as pure functions
 File: benchmark/planner_library.py
=============================================================================

 No ROS dependencies. Each planner takes:
   (grid, resolution, origin_x, origin_y, start_world, goal_world)
 And returns:
   (path, metrics_dict) or (None, metrics_dict)

 Metrics collected:
   - planning_time_ms: wall clock time for plan()
   - path_length_m: sum of segment distances
   - waypoint_count: number of waypoints after simplification
   - nodes_explored: algorithm-specific search effort
   - smoothness_deg: average turning angle (lower = smoother)

=============================================================================
"""

import math
import time
import heapq
import random
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
#  SHARED UTILITIES
# ─────────────────────────────────────────────────────────────────────────────

def inflate_grid(grid, resolution, margin_m):
    """Grow obstacle cells by margin_m meters."""
    height, width = grid.shape
    cells = int(math.ceil(margin_m / resolution))
    if cells <= 0:
        return grid.copy()
    inflated = grid.copy()
    obstacles = np.argwhere(grid == 1)
    for r, c in obstacles:
        r_lo = max(0, r - cells)
        r_hi = min(height, r + cells + 1)
        c_lo = max(0, c - cells)
        c_hi = min(width, c + cells + 1)
        inflated[r_lo:r_hi, c_lo:c_hi] = 1
    return inflated


def world_to_grid(x, y, origin_x, origin_y, resolution):
    c = int((x - origin_x) / resolution)
    r = int((y - origin_y) / resolution)
    return r, c


def grid_to_world(r, c, origin_x, origin_y, resolution):
    x = origin_x + (c + 0.5) * resolution
    y = origin_y + (r + 0.5) * resolution
    return x, y


def is_free(grid, r, c):
    h, w = grid.shape
    return 0 <= r < h and 0 <= c < w and grid[r, c] == 0


def nearest_free(grid, cell):
    """BFS to find nearest free cell."""
    visited = {cell}
    queue = [cell]
    while queue:
        next_q = []
        for r, c in queue:
            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    nb = (r + dr, c + dc)
                    if nb not in visited:
                        if is_free(grid, *nb):
                            return nb
                        visited.add(nb)
                        next_q.append(nb)
        queue = next_q
    return None


def simplify_collinear(path):
    """Remove collinear intermediate waypoints."""
    if len(path) < 3:
        return path
    result = [path[0]]
    for i in range(1, len(path) - 1):
        dx1 = path[i][0] - path[i - 1][0]
        dy1 = path[i][1] - path[i - 1][1]
        dx2 = path[i + 1][0] - path[i][0]
        dy2 = path[i + 1][1] - path[i][1]
        if abs(dx1 * dy2 - dx2 * dy1) > 1e-4:
            result.append(path[i])
    result.append(path[-1])
    return result


def compute_metrics(path, planning_time_ms, nodes_explored):
    """Compute standard metrics for a path."""
    if path is None:
        return {
            'planning_time_ms': planning_time_ms,
            'path_length_m': 0.0,
            'waypoint_count': 0,
            'nodes_explored': nodes_explored,
            'smoothness_deg': 0.0,
            'success': False
        }

    # Path length
    length = 0.0
    for i in range(1, len(path)):
        length += math.hypot(path[i][0] - path[i-1][0],
                             path[i][1] - path[i-1][1])

    # Smoothness: average turning angle at each waypoint
    angles = []
    for i in range(1, len(path) - 1):
        v1x = path[i][0] - path[i-1][0]
        v1y = path[i][1] - path[i-1][1]
        v2x = path[i+1][0] - path[i][0]
        v2y = path[i+1][1] - path[i][1]
        mag1 = math.hypot(v1x, v1y)
        mag2 = math.hypot(v2x, v2y)
        if mag1 > 1e-6 and mag2 > 1e-6:
            cos_a = max(-1, min(1, (v1x*v2x + v1y*v2y) / (mag1 * mag2)))
            angles.append(math.degrees(math.acos(cos_a)))

    smoothness = sum(angles) / len(angles) if angles else 0.0

    return {
        'planning_time_ms': planning_time_ms,
        'path_length_m': length,
        'waypoint_count': len(path),
        'nodes_explored': nodes_explored,
        'smoothness_deg': smoothness,
        'success': True
    }


def prepare_grid(raw_data, width, height, resolution, margin=0.25):
    """Convert raw occupancy data to inflated binary grid."""
    grid = np.where(raw_data.reshape((height, width)) > 50, 1, 0).astype(np.uint8)
    return inflate_grid(grid, resolution, margin)


def validate_endpoints(grid, start, goal, height, width):
    """Clamp and nudge start/goal to free cells."""
    start = (max(0, min(height-1, start[0])), max(0, min(width-1, start[1])))
    goal = (max(0, min(height-1, goal[0])), max(0, min(width-1, goal[1])))

    if not is_free(grid, *start):
        start = nearest_free(grid, start)
    if not is_free(grid, *goal):
        goal = nearest_free(grid, goal)

    return start, goal


# ─────────────────────────────────────────────────────────────────────────────
#  1. A* PLANNER
# ─────────────────────────────────────────────────────────────────────────────

def plan_astar(grid, resolution, origin_x, origin_y, start_xy, goal_xy, margin=0.25):
    t0 = time.time()
    g_grid = prepare_grid(grid, grid.shape[1] if len(grid.shape) > 1 else 0,
                          grid.shape[0] if len(grid.shape) > 1 else 0,
                          resolution, margin) if len(grid.shape) == 1 else grid
    # If grid is already 2D and inflated, use directly
    if len(grid.shape) == 2:
        g_grid = grid

    height, width = g_grid.shape
    start = world_to_grid(start_xy[0], start_xy[1], origin_x, origin_y, resolution)
    goal = world_to_grid(goal_xy[0], goal_xy[1], origin_x, origin_y, resolution)
    start, goal = validate_endpoints(g_grid, start, goal, height, width)

    if start is None or goal is None:
        return None, compute_metrics(None, (time.time()-t0)*1000, 0)

    open_set = []
    counter = 0
    heapq.heappush(open_set, (0.0, counter, start))
    came_from = {start: None}
    g = {start: 0.0}
    closed = set()
    nodes_explored = 0
    dirs = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    while open_set:
        _, _, cur = heapq.heappop(open_set)
        if cur == goal:
            break
        if cur in closed:
            continue
        closed.add(cur)
        nodes_explored += 1
        for dr, dc in dirs:
            nxt = (cur[0]+dr, cur[1]+dc)
            if not is_free(g_grid, *nxt) or nxt in closed:
                continue
            step = 1.414 if (dr != 0 and dc != 0) else 1.0
            ng = g[cur] + step
            if ng < g.get(nxt, float('inf')):
                g[nxt] = ng
                came_from[nxt] = cur
                f = ng + math.hypot(goal[0]-nxt[0], goal[1]-nxt[1])
                counter += 1
                heapq.heappush(open_set, (f, counter, nxt))

    dt = (time.time()-t0)*1000

    if goal not in came_from:
        return None, compute_metrics(None, dt, nodes_explored)

    path = []
    cur = goal
    while cur is not None:
        path.append(grid_to_world(*cur, origin_x, origin_y, resolution))
        cur = came_from[cur]
    path.reverse()
    path = simplify_collinear(path)
    return path, compute_metrics(path, dt, nodes_explored)


# ─────────────────────────────────────────────────────────────────────────────
#  2. DIJKSTRA PLANNER
# ─────────────────────────────────────────────────────────────────────────────

def plan_dijkstra(grid, resolution, origin_x, origin_y, start_xy, goal_xy, margin=0.25):
    t0 = time.time()
    if len(grid.shape) == 2:
        g_grid = grid
    else:
        g_grid = prepare_grid(grid, 0, 0, resolution, margin)

    height, width = g_grid.shape
    start = world_to_grid(start_xy[0], start_xy[1], origin_x, origin_y, resolution)
    goal = world_to_grid(goal_xy[0], goal_xy[1], origin_x, origin_y, resolution)
    start, goal = validate_endpoints(g_grid, start, goal, height, width)

    if start is None or goal is None:
        return None, compute_metrics(None, (time.time()-t0)*1000, 0)

    open_set = []
    counter = 0
    heapq.heappush(open_set, (0.0, counter, start))
    came_from = {start: None}
    g = {start: 0.0}
    closed = set()
    nodes_explored = 0
    dirs = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    while open_set:
        _, _, cur = heapq.heappop(open_set)
        if cur == goal:
            break
        if cur in closed:
            continue
        closed.add(cur)
        nodes_explored += 1
        for dr, dc in dirs:
            nxt = (cur[0]+dr, cur[1]+dc)
            if not is_free(g_grid, *nxt) or nxt in closed:
                continue
            step = 1.414 if (dr != 0 and dc != 0) else 1.0
            ng = g[cur] + step
            if ng < g.get(nxt, float('inf')):
                g[nxt] = ng
                came_from[nxt] = cur
                f = ng  # NO HEURISTIC — the only difference from A*
                counter += 1
                heapq.heappush(open_set, (f, counter, nxt))

    dt = (time.time()-t0)*1000

    if goal not in came_from:
        return None, compute_metrics(None, dt, nodes_explored)

    path = []
    cur = goal
    while cur is not None:
        path.append(grid_to_world(*cur, origin_x, origin_y, resolution))
        cur = came_from[cur]
    path.reverse()
    path = simplify_collinear(path)
    return path, compute_metrics(path, dt, nodes_explored)


# ─────────────────────────────────────────────────────────────────────────────
#  3. BELLMAN FORD PLANNER
# ─────────────────────────────────────────────────────────────────────────────

def plan_bellman_ford(grid, resolution, origin_x, origin_y, start_xy, goal_xy, margin=0.25):
    t0 = time.time()
    if len(grid.shape) == 2:
        g_grid = grid
    else:
        g_grid = prepare_grid(grid, 0, 0, resolution, margin)

    height, width = g_grid.shape
    start = world_to_grid(start_xy[0], start_xy[1], origin_x, origin_y, resolution)
    goal = world_to_grid(goal_xy[0], goal_xy[1], origin_x, origin_y, resolution)
    start, goal = validate_endpoints(g_grid, start, goal, height, width)

    if start is None or goal is None:
        return None, compute_metrics(None, (time.time()-t0)*1000, 0)

    # Bounded edge list
    padding = 30
    r_lo = max(0, min(start[0], goal[0]) - padding)
    r_hi = min(height, max(start[0], goal[0]) + padding)
    c_lo = max(0, min(start[1], goal[1]) - padding)
    c_hi = min(width, max(start[1], goal[1]) + padding)

    edges = []
    dirs = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    vertices = set()
    for r in range(r_lo, r_hi):
        for c in range(c_lo, c_hi):
            if not is_free(g_grid, r, c):
                continue
            vertices.add((r, c))
            for dr, dc in dirs:
                nr, nc = r+dr, c+dc
                if r_lo <= nr < r_hi and c_lo <= nc < c_hi and is_free(g_grid, nr, nc):
                    w = 1.414 if (dr != 0 and dc != 0) else 1.0
                    edges.append(((r,c), (nr,nc), w))

    dist = {v: float('inf') for v in vertices}
    prev = {v: None for v in vertices}
    dist[start] = 0.0
    nodes_explored = len(vertices)

    max_iters = min(len(vertices) - 1, 500)
    edges_relaxed = 0
    for iteration in range(max_iters):
        updated = False
        for u, v, w in edges:
            if dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                prev[v] = u
                updated = True
                edges_relaxed += 1
        if not updated:
            break

    dt = (time.time()-t0)*1000

    if prev[goal] is None and goal != start:
        return None, compute_metrics(None, dt, nodes_explored)

    path = []
    cur = goal
    while cur is not None:
        path.append(grid_to_world(*cur, origin_x, origin_y, resolution))
        cur = prev[cur]
    path.reverse()
    path = simplify_collinear(path)
    return path, compute_metrics(path, dt, edges_relaxed)


# ─────────────────────────────────────────────────────────────────────────────
#  4. PRM PLANNER
# ─────────────────────────────────────────────────────────────────────────────

def plan_prm(grid, resolution, origin_x, origin_y, start_xy, goal_xy, margin=0.25,
             num_samples=600, k_neighbors=10):
    t0 = time.time()
    if len(grid.shape) == 2:
        g_grid = grid
    else:
        g_grid = prepare_grid(grid, 0, 0, resolution, margin)

    height, width = g_grid.shape
    x_min = origin_x
    x_max = origin_x + width * resolution
    y_min = origin_y
    y_max = origin_y + height * resolution

    # Sample random free-space points
    samples = [start_xy, goal_xy]
    attempts = 0
    while len(samples) < num_samples + 2 and attempts < num_samples * 10:
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        r, c = world_to_grid(x, y, origin_x, origin_y, resolution)
        if is_free(g_grid, r, c):
            samples.append((x, y))
        attempts += 1

    # Doorway seeding
    doorway_regions = [(-3.5, 1.0, -2.5, 3.0), (2.5, -3.0, 3.5, -1.0)]
    for x_lo, y_lo, x_hi, y_hi in doorway_regions:
        for _ in range(30):
            x = random.uniform(x_lo, x_hi)
            y = random.uniform(y_lo, y_hi)
            r, c = world_to_grid(x, y, origin_x, origin_y, resolution)
            if is_free(g_grid, r, c):
                samples.append((x, y))

    pts = np.array(samples)
    nodes_explored = len(samples)

    # Build roadmap: connect k nearest neighbors with collision-free lines
    def line_free(x1, y1, x2, y2):
        dist = math.hypot(x2-x1, y2-y1)
        n = max(2, int(dist / (resolution * 0.5)))
        for i in range(n+1):
            t = i / n
            px, py = x1 + t*(x2-x1), y1 + t*(y2-y1)
            r, c = world_to_grid(px, py, origin_x, origin_y, resolution)
            if not is_free(g_grid, r, c):
                return False
        return True

    adj = {i: [] for i in range(len(samples))}
    for i in range(len(samples)):
        dists = np.sqrt((pts[:,0]-pts[i,0])**2 + (pts[:,1]-pts[i,1])**2)
        nearest = np.argsort(dists)[1:k_neighbors+1]
        for j in nearest:
            j = int(j)
            d = float(dists[j])
            if line_free(samples[i][0], samples[i][1], samples[j][0], samples[j][1]):
                adj[i].append((j, d))
                adj[j].append((i, d))

    # Dijkstra on roadmap
    open_set = [(0.0, 0)]  # start is index 0
    g_cost = {0: 0.0}
    came_from = {0: None}

    while open_set:
        cost, cur = heapq.heappop(open_set)
        if cur == 1:  # goal is index 1
            break
        for nxt, w in adj[cur]:
            ng = g_cost[cur] + w
            if ng < g_cost.get(nxt, float('inf')):
                g_cost[nxt] = ng
                came_from[nxt] = cur
                heapq.heappush(open_set, (ng, nxt))

    dt = (time.time()-t0)*1000

    if 1 not in came_from:
        return None, compute_metrics(None, dt, nodes_explored)

    path = []
    cur = 1
    while cur is not None:
        path.append(samples[cur])
        cur = came_from[cur]
    path.reverse()
    return path, compute_metrics(path, dt, nodes_explored)


# ─────────────────────────────────────────────────────────────────────────────
#  5. RRT PLANNER
# ─────────────────────────────────────────────────────────────────────────────

def plan_rrt(grid, resolution, origin_x, origin_y, start_xy, goal_xy, margin=0.25,
             max_iter=8000, step_size=0.4, goal_radius=0.5, goal_bias=0.10):
    t0 = time.time()
    if len(grid.shape) == 2:
        g_grid = grid
    else:
        g_grid = prepare_grid(grid, 0, 0, resolution, margin)

    height, width = g_grid.shape
    x_min = origin_x
    x_max = origin_x + width * resolution
    y_min = origin_y
    y_max = origin_y + height * resolution

    sx, sy = start_xy
    gx, gy = goal_xy

    def world_free(x, y):
        r, c = world_to_grid(x, y, origin_x, origin_y, resolution)
        return is_free(g_grid, r, c)

    def line_free(x1, y1, x2, y2):
        dist = math.hypot(x2-x1, y2-y1)
        n = max(2, int(dist / (resolution * 0.5)))
        for i in range(n+1):
            t = i / n
            if not world_free(x1 + t*(x2-x1), y1 + t*(y2-y1)):
                return False
        return True

    nodes = [(sx, sy, -1)]
    goal_node = None

    for iteration in range(max_iter):
        if random.random() < goal_bias:
            rx, ry = gx, gy
        else:
            rx, ry = random.uniform(x_min, x_max), random.uniform(y_min, y_max)

        # Nearest node
        pts = np.array([(n[0], n[1]) for n in nodes])
        dists = (pts[:,0]-rx)**2 + (pts[:,1]-ry)**2
        nearest_idx = int(np.argmin(dists))
        nx, ny = nodes[nearest_idx][0], nodes[nearest_idx][1]

        dist = math.hypot(rx-nx, ry-ny)
        if dist <= step_size:
            new_x, new_y = rx, ry
        else:
            ratio = step_size / dist
            new_x = nx + ratio * (rx - nx)
            new_y = ny + ratio * (ry - ny)

        if not line_free(nx, ny, new_x, new_y):
            continue

        new_idx = len(nodes)
        nodes.append((new_x, new_y, nearest_idx))

        if math.hypot(new_x-gx, new_y-gy) < goal_radius:
            if line_free(new_x, new_y, gx, gy):
                nodes.append((gx, gy, new_idx))
                goal_node = len(nodes) - 1
                break

    dt = (time.time()-t0)*1000
    nodes_explored = len(nodes)

    if goal_node is None:
        return None, compute_metrics(None, dt, nodes_explored)

    path = []
    idx = goal_node
    while idx != -1:
        path.append((nodes[idx][0], nodes[idx][1]))
        idx = nodes[idx][2]
    path.reverse()

    # Simplify RRT path with greedy shortcuts
    simplified = [path[0]]
    i = 0
    while i < len(path) - 1:
        farthest = i + 1
        for j in range(len(path)-1, i, -1):
            if line_free(path[i][0], path[i][1], path[j][0], path[j][1]):
                farthest = j
                break
        simplified.append(path[farthest])
        i = farthest
    path = simplified

    return path, compute_metrics(path, dt, nodes_explored)


# ─────────────────────────────────────────────────────────────────────────────
#  6. THETA* PLANNER
# ─────────────────────────────────────────────────────────────────────────────

def _bresenham_free(grid, r1, c1, r2, c2):
    """Bresenham line-of-sight check."""
    dr = abs(r2 - r1)
    dc = abs(c2 - c1)
    sr = 1 if r2 > r1 else -1
    sc = 1 if c2 > c1 else -1
    r, c = r1, c1
    err = dr - dc
    while True:
        if not is_free(grid, r, c):
            return False
        if r == r2 and c == c2:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return True


def plan_theta_star(grid, resolution, origin_x, origin_y, start_xy, goal_xy, margin=0.25):
    t0 = time.time()
    if len(grid.shape) == 2:
        g_grid = grid
    else:
        g_grid = prepare_grid(grid, 0, 0, resolution, margin)

    height, width = g_grid.shape
    start = world_to_grid(start_xy[0], start_xy[1], origin_x, origin_y, resolution)
    goal = world_to_grid(goal_xy[0], goal_xy[1], origin_x, origin_y, resolution)
    start, goal = validate_endpoints(g_grid, start, goal, height, width)

    if start is None or goal is None:
        return None, compute_metrics(None, (time.time()-t0)*1000, 0)

    open_set = []
    counter = 0
    heapq.heappush(open_set, (0.0, counter, start))
    came_from = {start: start}
    g = {start: 0.0}
    closed = set()
    nodes_explored = 0
    dirs = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    while open_set:
        _, _, cur = heapq.heappop(open_set)
        if cur == goal:
            break
        if cur in closed:
            continue
        closed.add(cur)
        nodes_explored += 1

        for dr, dc in dirs:
            nxt = (cur[0]+dr, cur[1]+dc)
            if not is_free(g_grid, *nxt) or nxt in closed:
                continue

            parent_cur = came_from[cur]
            if _bresenham_free(g_grid, parent_cur[0], parent_cur[1], nxt[0], nxt[1]):
                new_g = g[parent_cur] + math.hypot(nxt[0]-parent_cur[0], nxt[1]-parent_cur[1])
                new_parent = parent_cur
            else:
                step = 1.414 if (dr != 0 and dc != 0) else 1.0
                new_g = g[cur] + step
                new_parent = cur

            if new_g < g.get(nxt, float('inf')):
                g[nxt] = new_g
                came_from[nxt] = new_parent
                f = new_g + math.hypot(goal[0]-nxt[0], goal[1]-nxt[1])
                counter += 1
                heapq.heappush(open_set, (f, counter, nxt))

    dt = (time.time()-t0)*1000

    if goal not in came_from:
        return None, compute_metrics(None, dt, nodes_explored)

    path = []
    cur = goal
    while cur != came_from[cur]:
        path.append(grid_to_world(*cur, origin_x, origin_y, resolution))
        cur = came_from[cur]
    path.append(grid_to_world(*start, origin_x, origin_y, resolution))
    path.reverse()
    return path, compute_metrics(path, dt, nodes_explored)


# ─────────────────────────────────────────────────────────────────────────────
#  REGISTRY
# ─────────────────────────────────────────────────────────────────────────────
PLANNERS = {
    'A*':           plan_astar,
    'Dijkstra':     plan_dijkstra,
    'Bellman Ford': plan_bellman_ford,
    'PRM':          plan_prm,
    'RRT':          plan_rrt,
    'Theta*':       plan_theta_star,
}
