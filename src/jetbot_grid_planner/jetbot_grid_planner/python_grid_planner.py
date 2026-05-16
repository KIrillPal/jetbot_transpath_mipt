#!/usr/bin/env python3
"""
Nav2-compatible global planning action servers (Python grid planners).

Интеграция без C++ pluginlib: Nav2 BT вызывает ``compute_path_to_pose`` и
``compute_path_through_poses``. В Humble узел ``bt_navigator`` всегда активирует
оба навигатора, поэтому оба action-сервера обязательны. Стандартный
``planner_server`` в launch отключите, иначе конфликт имён. План по топику
``map`` (OccupancyGrid от map_server).

Параметр ``algorithm`` выбирает реализацию (сейчас: dijkstra); позже добавьте
новый класс и ветку в ``algorithms.make_planner``.
"""

from __future__ import annotations

import time
from typing import List

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

from jetbot_grid_planner.algorithms import make_planner
from jetbot_grid_planner.grid_geometry import cell_center_to_world, world_to_cell, yaw_to_quat


class PythonGridPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('python_grid_planner')

        self.declare_parameter('grid_topic', 'map')
        self.declare_parameter('algorithm', 'dijkstra')
        self.declare_parameter('lethal_cost', 100)
        self.declare_parameter('allow_unknown', True)

        ns = self.get_namespace().strip('/')
        default_base = f'{ns}/base_footprint' if ns else 'base_footprint'
        self.declare_parameter('robot_base_frame', default_base)

        self.grid_topic = self.get_parameter('grid_topic').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        alg_name = self.get_parameter('algorithm').value
        self.lethal_cost = int(self.get_parameter('lethal_cost').value)
        self.allow_unknown = bool(self.get_parameter('allow_unknown').value)

        self.planner_impl = make_planner(str(alg_name))

        self._map: OccupancyGrid | None = None
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, self.grid_topic, self._on_map, qos_map)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._cb_group = ReentrantCallbackGroup()
        self._action_to_pose = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self._action_through_poses = ActionServer(
            self,
            ComputePathThroughPoses,
            'compute_path_through_poses',
            execute_callback=self._execute_through_poses,
            goal_callback=self._through_goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f'Python grid planner: actions compute_path_to_pose + compute_path_through_poses, '
            f'topic={self.grid_topic!r} algorithm={alg_name!r}'
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _goal_cb(self, goal_request: ComputePathToPose.Goal):
        return GoalResponse.ACCEPT

    def _through_goal_cb(self, goal_request: ComputePathThroughPoses.Goal):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, cancel_request):
        return CancelResponse.ACCEPT

    def _plan_cell_path(
        self,
        start_pose: PoseStamped,
        goal_pose: PoseStamped,
        occ: np.ndarray,
        grid: OccupancyGrid,
    ) -> List[tuple[int, int]] | None:
        sx, sy = world_to_cell(
            start_pose.pose.position.x, start_pose.pose.position.y, grid
        )
        gx, gy = world_to_cell(
            goal_pose.pose.position.x, goal_pose.pose.position.y, grid
        )
        return self.planner_impl.plan(
            occ,
            (sx, sy),
            (gx, gy),
            lethal_cost=self.lethal_cost,
            allow_unknown=self.allow_unknown,
        )

    def _execute(self, goal_handle):
        request = goal_handle.request
        t0 = time.monotonic()
        grid = self._map
        if grid is None:
            self.get_logger().error('No map/grid received yet.')
            goal_handle.abort()
            res = ComputePathToPose.Result()
            res.path = Path()
            res.planning_time = Duration()
            return res

        frame_id = grid.header.frame_id
        if request.use_start:
            start_pose = request.start
        else:
            start_pose = self._lookup_robot_pose(frame_id)
            if start_pose is None:
                self.get_logger().error('Could not resolve robot pose in map frame.')
                goal_handle.abort()
                res = ComputePathToPose.Result()
                res.path = Path()
                res.planning_time = Duration()
                return res

        goal_pose = request.goal
        w, h = grid.info.width, grid.info.height
        occ = np.asarray(grid.data, dtype=np.int16).reshape((h, w))

        path_cells = self._plan_cell_path(start_pose, goal_pose, occ, grid)
        dt = time.monotonic() - t0
        if path_cells is None:
            self.get_logger().warn('No path found (Dijkstra unreachable).')
            goal_handle.abort()
            res = ComputePathToPose.Result()
            res.path = Path()
            res.planning_time = self._sec_to_duration(dt)
            return res

        nav_path = self._cells_to_path(path_cells, grid, frame_id)
        res = ComputePathToPose.Result()
        res.path = nav_path
        res.planning_time = self._sec_to_duration(dt)
        goal_handle.succeed()
        self.get_logger().info(
            f'Path OK: {len(nav_path.poses)} poses in {dt:.3f}s ({len(path_cells)} cells)'
        )
        return res

    def _execute_through_poses(self, goal_handle):
        request = goal_handle.request
        t0 = time.monotonic()
        grid = self._map
        empty = ComputePathThroughPoses.Result()
        empty.path = Path()
        empty.planning_time = Duration()

        if grid is None:
            self.get_logger().error('No map/grid received yet.')
            goal_handle.abort()
            return empty

        if not request.goals:
            self.get_logger().error('Through-poses: empty goals list.')
            goal_handle.abort()
            return empty

        frame_id = grid.header.frame_id
        if request.use_start:
            current = request.start
        else:
            current = self._lookup_robot_pose(frame_id)
            if current is None:
                self.get_logger().error('Could not resolve robot pose for through-poses.')
                goal_handle.abort()
                return empty

        w, h = grid.info.width, grid.info.height
        occ = np.asarray(grid.data, dtype=np.int16).reshape((h, w))
        all_cells: List[tuple[int, int]] = []

        for wp in request.goals:
            segment = self._plan_cell_path(current, wp, occ, grid)
            if segment is None:
                self.get_logger().warn('Through-poses: unreachable segment.')
                goal_handle.abort()
                res = ComputePathThroughPoses.Result()
                res.path = Path()
                res.planning_time = self._sec_to_duration(time.monotonic() - t0)
                return res
            if all_cells and segment and segment[0] == all_cells[-1]:
                segment = segment[1:]
            all_cells.extend(segment)
            current = wp

        dt = time.monotonic() - t0
        nav_path = self._cells_to_path(all_cells, grid, frame_id)
        res = ComputePathThroughPoses.Result()
        res.path = nav_path
        res.planning_time = self._sec_to_duration(dt)
        goal_handle.succeed()
        self.get_logger().info(
            f'Through-poses path OK: {len(nav_path.poses)} poses, '
            f'{len(request.goals)} waypoints in {dt:.3f}s'
        )
        return res

    def _lookup_robot_pose(self, map_frame: str) -> PoseStamped | None:
        try:
            t = self._tf_buffer.lookup_transform(
                map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'TF {map_frame} <- {self.robot_base_frame}: {exc}')
            return None
        p = PoseStamped()
        p.header.frame_id = self.robot_base_frame
        p.pose.orientation.w = 1.0
        return do_transform_pose(p, t)

    @staticmethod
    def _sec_to_duration(sec: float) -> Duration:
        d = Duration()
        d.sec = int(sec)
        d.nanosec = int((sec - d.sec) * 1e9) % 1_000_000_000
        return d

    def _cells_to_path(
        self,
        cells: List[tuple[int, int]],
        grid: OccupancyGrid,
        frame_id: str,
    ) -> Path:
        out = Path()
        out.header.frame_id = frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        if not cells:
            return out

        poses: List[PoseStamped] = []
        for i, (ix, iy) in enumerate(cells):
            wx, wy = cell_center_to_world(ix, iy, grid)
            yaw = 0.0
            if i < len(cells) - 1:
                wx2, wy2 = cell_center_to_world(cells[i + 1][0], cells[i + 1][1], grid)
                yaw = float(np.arctan2(wy2 - wy, wx2 - wx))
            elif len(cells) >= 2:
                wx0, wy0 = cell_center_to_world(cells[i - 1][0], cells[i - 1][1], grid)
                yaw = float(np.arctan2(wy - wy0, wx - wx0))

            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = out.header.stamp
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quat(yaw)
            poses.append(ps)
        out.poses = poses
        return out


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PythonGridPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
