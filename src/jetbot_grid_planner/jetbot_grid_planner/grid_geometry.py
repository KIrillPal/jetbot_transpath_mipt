"""World ↔ grid helpers for OccupancyGrid / map_server."""
from __future__ import annotations

import math
from typing import Tuple

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid


def quat_yaw(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def world_to_cell(wx: float, wy: float, grid: OccupancyGrid) -> Tuple[int, int]:
    info = grid.info
    ox, oy = info.origin.position.x, info.origin.position.y
    yaw = quat_yaw(info.origin.orientation)
    dx, dy = wx - ox, wy - oy
    c, s = math.cos(-yaw), math.sin(-yaw)
    gx = dx * c - dy * s
    gy = dx * s + dy * c
    ix = int(math.floor(gx / info.resolution))
    iy = int(math.floor(gy / info.resolution))
    return ix, iy


def cell_center_to_world(ix: int, iy: int, grid: OccupancyGrid) -> Tuple[float, float]:
    info = grid.info
    lx = (ix + 0.5) * info.resolution
    ly = (iy + 0.5) * info.resolution
    yaw = quat_yaw(info.origin.orientation)
    c, s = math.cos(yaw), math.sin(yaw)
    wx = info.origin.position.x + c * lx - s * ly
    wy = info.origin.position.y + s * lx + c * ly
    return wx, wy


def yaw_to_quat(yaw: float) -> Quaternion:
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))
