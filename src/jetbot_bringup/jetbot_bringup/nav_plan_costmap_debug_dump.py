#!/usr/bin/env python3
"""Subscribe to Nav2 global plan + global costmap; optionally dump PNG frames."""

from __future__ import annotations

import math
from pathlib import Path as FsPath
from typing import Iterable

import numpy as np
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as NavPath
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _manhattan_bridge_cells(x0: int, y0: int, x1: int, y1: int) -> Iterable[tuple[int, int]]:
    """Только 4-соседство: сначала по X, потом по Y (без диагональных рёбер)."""
    x, y = x0, y0
    yield (x, y)
    while x != x1:
        x += 1 if x1 > x else -1
        yield (x, y)
    while y != y1:
        y += 1 if y1 > y else -1
        yield (x, y)


def _simplify_snapped_l1(chain: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Убирает промежуточные клетки, лежащие на каком-либо кратчайшем L1-пути (уменьшает зигзаг → «ширину»)."""
    if len(chain) <= 2:
        return list(chain)
    out: list[tuple[int, int]] = [chain[0]]
    for i in range(1, len(chain) - 1):
        ax, ay = out[-1]
        bx, by = chain[i]
        cx, cy = chain[i + 1]
        d_direct = abs(cx - ax) + abs(cy - ay)
        d_via = abs(bx - ax) + abs(by - ay) + abs(cx - bx) + abs(cy - by)
        if d_via == d_direct:
            continue
        out.append((bx, by))
    out.append(chain[-1])
    return out


def _quat_yaw(q: Quaternion) -> float:
    """Yaw (rad) from planar quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class NavPlanCostmapDebugDump(Node):
    """When dump_directory is set, saves costmap / overlay PNGs on each global plan update."""

    def __init__(self) -> None:
        super().__init__('nav_plan_costmap_debug_dump')

        self.declare_parameter('dump_directory', '')
        self.declare_parameter('plan_topic', 'plan')
        self.declare_parameter('costmap_topic', 'global_costmap/costmap')

        dump_raw = self.get_parameter('dump_directory').value
        self.dump_dir = FsPath(str(dump_raw).strip()) if dump_raw else FsPath()
        self.enabled = bool(str(dump_raw).strip())

        self.plan_topic = self.get_parameter('plan_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value

        self._latest_costmap: OccupancyGrid | None = None
        self._latest_plan: NavPath | None = None
        self._plan_needs_dump = False
        self._seq = 0

        # Nav2 publishes plan / global_costmap with sensor-style QoS (often BEST_EFFORT).
        # A RELIABLE subscriber would receive nothing → empty dump dir was the symptom.
        qos = qos_profile_sensor_data

        if self.enabled:
            self.dump_dir.mkdir(parents=True, exist_ok=True)
            resolved = self.dump_dir.resolve()
            self.get_logger().info(
                f'Nav debug dump ENABLED -> {resolved} '
                f'(plan={self.plan_topic!r}, costmap={self.costmap_topic!r})'
            )
            self.create_subscription(NavPath, self.plan_topic, self._on_plan, qos)
            self.create_subscription(OccupancyGrid, self.costmap_topic, self._on_costmap, qos)
        else:
            self.get_logger().info(
                'Nav debug dump disabled (empty dump_directory); '
                'node idle — minimal overhead.'
            )

        self.get_logger().debug(f'dump_directory param raw={dump_raw!r}')

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._latest_costmap = msg
        self._attempt_dump()

    def _on_plan(self, msg: NavPath) -> None:
        if not self.enabled:
            return
        self._latest_plan = msg
        self._plan_needs_dump = True
        self._attempt_dump()

    def _attempt_dump(self) -> None:
        if not self.enabled or not self._plan_needs_dump:
            return
        plan = self._latest_plan
        costmap = self._latest_costmap
        if plan is None or costmap is None:
            return
        try:
            self._dump_pair(plan, costmap)
            self._plan_needs_dump = False
        except Exception as exc:  # noqa: BLE001 — debug tooling must not crash stack
            self.get_logger().error(f'Debug dump failed: {exc}')

    def _world_to_px(self, wx: float, wy: float, grid: OccupancyGrid) -> tuple[int, int]:
        """Grid pixel (column=x-index, row=y-index) from map-frame world coords."""
        info = grid.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        yaw = _quat_yaw(info.origin.orientation)
        dx = wx - ox
        dy = wy - oy
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        gx = dx * c - dy * s
        gy = dx * s + dy * c
        ix = int(gx / info.resolution)
        iy = int(gy / info.resolution)
        return ix, iy

    def _plan_cells_on_costmap_grid(
        self, plan: NavPath, grid: OccupancyGrid
    ) -> set[tuple[int, int]]:
        """
        Клетки траектории на сетке global costmap (аппроксимация по топику plan).

        Раньше между снапами шёл Бризенхем с диагональными шагами — отсюда пары клеток только
        по углу и местами визуальная «ширина» 2–3 px из‑за зигзага снапов. Сейчас: сжатие
        цепочки по L1 и соединение отрезков только ортогональными шагами (Манхэттен).
        """
        gw, gh = grid.info.width, grid.info.height
        snapped: list[tuple[int, int]] = []
        for ps in plan.poses:
            ix, iy = self._world_to_px(ps.pose.position.x, ps.pose.position.y, grid)
            if 0 <= ix < gw and 0 <= iy < gh:
                if not snapped or snapped[-1] != (ix, iy):
                    snapped.append((ix, iy))

        snapped = _simplify_snapped_l1(snapped)

        cells: set[tuple[int, int]] = set()
        for i in range(len(snapped) - 1):
            x0, y0 = snapped[i]
            x1, y1 = snapped[i + 1]
            for cx, cy in _manhattan_bridge_cells(x0, y0, x1, y1):
                if 0 <= cx < gw and 0 <= cy < gh:
                    cells.add((cx, cy))
        if len(snapped) == 1:
            cells.add(snapped[0])
        return cells

    def _occupancy_to_rgb(self, grid: OccupancyGrid) -> np.ndarray:
        """H×W×3 uint8 RGB: costmap в красных тонах (-1 unknown, 0–100 cost)."""
        w, h = grid.info.width, grid.info.height
        raw = np.asarray(grid.data, dtype=np.int16)
        if raw.size != w * h:
            raise ValueError(f'costmap data size {raw.size} != {w*h}')
        occ = raw.reshape((h, w))

        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        unk = occ < 0
        rgb[unk] = (200, 140, 140)

        known = ~unk
        free = known & (occ == 0)
        rgb[free] = (255, 235, 235)

        infl = known & (occ > 0) & (occ < 100)
        if np.any(infl):
            t = occ[infl].astype(np.float32) / 99.0
            r = (220 + 35 * t).astype(np.uint8)
            g = (180 * (1.0 - t)).astype(np.uint8)
            b = (180 * (1.0 - t)).astype(np.uint8)
            rgb[infl] = np.stack([r, g, b], axis=-1)

        lethal = known & (occ >= 100)
        rgb[lethal] = (110, 0, 0)

        # ROS OccupancyGrid row 0 is typically bottom edge — flip for image coords
        return np.flipud(rgb)

    def _dump_pair(self, plan: NavPath, grid: OccupancyGrid) -> None:
        seq = self._seq
        self._seq += 1
        prefix = f'nav_dbg_{seq:06d}'
        cmap_rgb = self._occupancy_to_rgb(grid)
        h, w = cmap_rgb.shape[:2]

        cost_path = self.dump_dir / f'{prefix}_global_costmap.png'
        Image.fromarray(cmap_rgb).save(cost_path)

        overlay = cmap_rgb.copy()
        path_cells = self._plan_cells_on_costmap_grid(plan, grid)
        path_rgb = np.array([40, 140, 255], dtype=np.uint8)
        gh = grid.info.height
        for ix, iy in path_cells:
            iy_img = (gh - 1) - iy
            if 0 <= ix < w and 0 <= iy_img < h:
                overlay[iy_img, ix] = path_rgb

        overlay_path = self.dump_dir / f'{prefix}_global_plan_overlay.png'
        Image.fromarray(overlay).save(overlay_path)

        self.get_logger().info(
            f'Saved {cost_path.name} + {overlay_path.name} '
            f'({len(plan.poses)} poses, {len(path_cells)} grid cells painted)'
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavPlanCostmapDebugDump()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
