from __future__ import annotations

import heapq
from typing import List

import numpy as np

from jetbot_grid_planner.algorithms.base import GridCoord


class DijkstraPlanner:
    """4-connected Dijkstra on a weighted grid (Nav2 / map_server occupancy)."""

    def plan(
        self,
        occ: np.ndarray,
        start: GridCoord,
        goal: GridCoord,
        *,
        lethal_cost: int = 99,
        allow_unknown: bool = True,
    ) -> List[GridCoord] | None:
        h, w = occ.shape
        sx, sy = start
        gx, gy = goal
        if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
            return None

        def cell_weight(ix: int, iy: int) -> float:
            v = int(occ[iy, ix])
            if v < 0:
                if not allow_unknown:
                    return float('inf')
                return 25.0
            if v >= lethal_cost:
                return float('inf')
            # Higher occupancy (inflation) → higher cost, free ~1
            return 1.0 + (v / max(lethal_cost - 1, 1)) * 15.0

        if cell_weight(sx, sy) == float('inf') or cell_weight(gx, gy) == float('inf'):
            return None

        inf = float('inf')
        dist = np.full((h, w), inf, dtype=np.float64)
        prev = np.empty((h, w, 2), dtype=np.int32)
        prev[..., :] = -1
        dist[sy, sx] = 0.0
        pq: list[tuple[float, int, int]] = [(0.0, sx, sy)]
        neigh = ((1, 0), (-1, 0), (0, 1), (0, -1))

        while pq:
            d, x, y = heapq.heappop(pq)
            if x == gx and y == gy:
                break
            if d > dist[y, x]:
                continue
            cw = cell_weight(x, y)
            if cw == inf:
                continue
            for dx, dy in neigh:
                nx_, ny_ = x + dx, y + dy
                if not (0 <= nx_ < w and 0 <= ny_ < h):
                    continue
                nw = cell_weight(nx_, ny_)
                if nw == inf:
                    continue
                nd = d + 0.5 * (cw + nw)
                if nd < dist[ny_, nx_]:
                    dist[ny_, nx_] = nd
                    prev[ny_, nx_] = (x, y)
                    heapq.heappush(pq, (nd, nx_, ny_))

        if dist[gy, gx] == inf:
            return None

        path: List[GridCoord] = []
        cx, cy = gx, gy
        while True:
            path.append((cx, cy))
            if cx == sx and cy == sy:
                break
            px, py = int(prev[cy, cx, 0]), int(prev[cy, cx, 1])
            if px < 0:
                return None
            cx, cy = px, py
        path.reverse()
        return path
