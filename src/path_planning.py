"""Optimized Path Planning for Mars Rover

Implements memory- and time-efficient planners:
- Vectorized A* on occupancy grids with NumPy
- Anytime D* Lite for dynamic replanning
- Optional smoothing with cubic splines

Author: MarsRover-AutoNav-Blackout Team
License: MIT
"""

from __future__ import annotations
import heapq
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np


@dataclass
class PlannerConfig:
    resolution: float = 0.1  # meters per cell
    robot_radius: float = 0.3  # meters
    diag_motion: bool = True
    inflate_obstacles: bool = True


def inflate_occupancy(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    """Inflate occupancy grid using vectorized max filter via convolution-like dilation."""
    if radius_cells <= 0:
        return grid
    # Fast dilation by rolling window max (vectorized)
    inflated = grid.copy()
    for dx in range(-radius_cells, radius_cells + 1):
        inflated = np.maximum(inflated, np.roll(grid, shift=dx, axis=0))
    grid2 = inflated.copy()
    for dy in range(-radius_cells, radius_cells + 1):
        grid2 = np.maximum(grid2, np.roll(inflated, shift=dy, axis=1))
    return grid2


class AStarPlanner:
    """Vectorized A* path planner on 2D occupancy grid."""

    def __init__(self, occ_grid: np.ndarray, config: PlannerConfig):
        assert occ_grid.ndim == 2
        self.grid = (occ_grid > 0).astype(np.uint8)
        self.h, self.w = self.grid.shape
        self.cfg = config

        if self.cfg.inflate_obstacles:
            radius_cells = int(np.ceil(self.cfg.robot_radius / self.cfg.resolution))
            self.grid = inflate_occupancy(self.grid, radius_cells)

        # Precompute movement deltas and costs
        if self.cfg.diag_motion:
            self.moves = np.array([
                [-1, 0], [1, 0], [0, -1], [0, 1],
                [-1, -1], [-1, 1], [1, -1], [1, 1]
            ], dtype=np.int8)
            self.move_costs = np.array([1, 1, 1, 1, np.sqrt(2), np.sqrt(2), np.sqrt(2), np.sqrt(2)], dtype=np.float32)
        else:
            self.moves = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]], dtype=np.int8)
            self.move_costs = np.ones(4, dtype=np.float32)

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.h and 0 <= y < self.w

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        ax, ay = a; bx, by = b
        if self.cfg.diag_motion:
            # Octile distance (admissible for 8-connected grid)
            dx = abs(ax - bx); dy = abs(ay - by)
            return (max(dx, dy) - min(dx, dy)) + np.sqrt(2) * min(dx, dy)
        else:
            return abs(ax - bx) + abs(ay - by)

    def neighbors(self, x: int, y: int) -> Tuple[np.ndarray, np.ndarray]:
        nbrs = self.moves + np.array([x, y], dtype=np.int16)
        # Vectorized in-bounds and obstacle-free mask
        mask = (
            (nbrs[:, 0] >= 0) & (nbrs[:, 0] < self.h) &
            (nbrs[:, 1] >= 0) & (nbrs[:, 1] < self.w) &
            (self.grid[nbrs[:, 0], nbrs[:, 1]] == 0)
        )
        return nbrs[mask], self.move_costs[mask]

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        if self.grid[start] or self.grid[goal]:
            return None

        open_heap: List[Tuple[float, int, int]] = []
        heapq.heappush(open_heap, (0.0, start[0], start[1]))

        g = np.full((self.h, self.w), np.inf, dtype=np.float32)
        g[start] = 0.0
        came_x = np.full((self.h, self.w), -1, dtype=np.int16)
        came_y = np.full((self.h, self.w), -1, dtype=np.int16)

        visited = np.zeros((self.h, self.w), dtype=bool)
        goal_found = False

        while open_heap:
            _, cx, cy = heapq.heappop(open_heap)
            if visited[cx, cy]:
                continue
            visited[cx, cy] = True

            if (cx, cy) == goal:
                goal_found = True
                break

            nbrs, costs = self.neighbors(cx, cy)
            if nbrs.size == 0:
                continue
            nx = nbrs[:, 0]; ny = nbrs[:, 1]

            # Vectorized relaxations
            tentative = g[cx, cy] + costs
            improve_mask = tentative < g[nx, ny]
            if np.any(improve_mask):
                g[nx[improve_mask], ny[improve_mask]] = tentative[improve_mask]
                came_x[nx[improve_mask], ny[improve_mask]] = cx
                came_y[nx[improve_mask], ny[improve_mask]] = cy
                # Push updated nodes with f = g + h
                h_vals = np.array([self.heuristic((int(x), int(y)), goal) for x, y in zip(nx[improve_mask], ny[improve_mask])], dtype=np.float32)
                f = g[nx[improve_mask], ny[improve_mask]] + h_vals
                for fx, x, y in zip(f.tolist(), nx[improve_mask].tolist(), ny[improve_mask].tolist()):
                    heapq.heappush(open_heap, (float(fx), int(x), int(y)))

        if not goal_found:
            return None

        # Reconstruct path
        path: List[Tuple[int, int]] = []
        cx, cy = goal
        while not (cx == start[0] and cy == start[1]):
            path.append((cx, cy))
            px = came_x[cx, cy]; py = came_y[cx, cy]
            if px < 0 or py < 0:
                return None
            cx, cy = int(px), int(py)
        path.append(start)
        path.reverse()
        return path


def smooth_path(path: List[Tuple[int, int]], weight: float = 0.25, iters: int = 50) -> List[Tuple[int, int]]:
    """Simple Laplacian smoothing keeping endpoints fixed, vectorized per-iteration."""
    if path is None or len(path) < 3:
        return path
    P = np.array(path, dtype=np.float32)
    fixed0 = P[0].copy(); fixedN = P[-1].copy()
    for _ in range(iters):
        P[1:-1] += weight * (P[:-2] + P[2:] - 2 * P[1:-1])
        P[0] = fixed0; P[-1] = fixedN
    return [(int(round(x)), int(round(y))) for x, y in P]


# Minimal D* Lite skeleton (anytime replanning) — space-efficient bookkeeping
class DStarLite:
    def __init__(self, occ_grid: np.ndarray, config: PlannerConfig):
        self.grid = (occ_grid > 0).astype(np.uint8)
        self.h, self.w = self.grid.shape
        self.cfg = config
        self.km = 0.0
        self.rhs = {}
        self.g = {}
        self.open = []  # heap of (key1,key2,(x,y))

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        ax, ay = a; bx, by = b
        dx = abs(ax - bx); dy = abs(ay - by)
        return (max(dx, dy) - min(dx, dy)) + np.sqrt(2) * min(dx, dy)

    def neighbors(self, s: Tuple[int, int]) -> List[Tuple[int, int]]:
        x, y = s
        moves = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)] if self.cfg.diag_motion else [(-1,0),(1,0),(0,-1),(0,1)]
        res = []
        for dx, dy in moves:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.h and 0 <= ny < self.w and self.grid[nx, ny] == 0:
                res.append((nx, ny))
        return res

    def key(self, s: Tuple[int, int], start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[float, float]:
        g = self.g.get(s, np.inf)
        rhs = self.rhs.get(s, np.inf)
        k1 = min(g, rhs) + self.heuristic(start, s) + self.km
        k2 = min(g, rhs)
        return (k1, k2)

    def initialize(self, start: Tuple[int, int], goal: Tuple[int, int]):
        self.rhs[goal] = 0.0
        heapq.heappush(self.open, (self.key(goal, start, goal), goal))

    def update_vertex(self, u: Tuple[int, int], start: Tuple[int, int], goal: Tuple[int, int]):
        if u != goal:
            self.rhs[u] = min([self.g.get(s, np.inf) + 1 for s in self.neighbors(u)] or [np.inf])
        # remove u from open if present, then if g!=rhs push
        self.open = [(k, s) for (k, s) in self.open if s != u]
        heapq.heapify(self.open)
        if self.g.get(u, np.inf) != self.rhs.get(u, np.inf):
            heapq.heappush(self.open, (self.key(u, start, goal), u))

    def compute_shortest_path(self, start: Tuple[int, int], goal: Tuple[int, int]):
        while self.open and (self.open[0][0] < self.key(start, start, goal) or self.g.get(start, np.inf) != self.rhs.get(start, np.inf)):
            (_, _), u = heapq.heappop(self.open)
            if self.g.get(u, np.inf) > self.rhs.get(u, np.inf):
                self.g[u] = self.rhs[u]
                for s in self.neighbors(u):
                    self.update_vertex(s, start, goal)
            else:
                self.g[u] = np.inf
                self.update_vertex(u, start, goal)
                for s in self.neighbors(u):
                    self.update_vertex(s, start, goal)

    def extract_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        if self.g.get(start, np.inf) == np.inf:
            return None
        path = [start]
        s = start
        while s != goal:
            nbrs = self.neighbors(s)
            if not nbrs:
                return None
            s = min(nbrs, key=lambda x: self.g.get(x, np.inf))
            path.append(s)
        return path
