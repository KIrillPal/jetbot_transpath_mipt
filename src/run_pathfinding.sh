#!/usr/bin/env bash
# Launch сразу; initialpose и goal — через 20 с после старта (все процессы параллельно).
set -euo pipefail

ros2 launch jetbot_bringup all_robot_launches_localization.launch.py \
  robot_namespace:=robot_2 \
  map:=/home/app/ros2_ws/src/jetbot_bringup/maps/map_labirint_v3.yaml \
  params_file:=/home/app/ros2_ws/src/jetbot_bringup/config/nav2_default_localization_ignore_dyn_obst.yaml \
  odometry_source:=encoders \
  nav_debug_dump_dir:="/home/app/ros2_ws/src/jetbot_bringup/tmp/nav_debug_dump3"

sleep 20; \
ros2 topic pub --once /robot_2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'robot_2/map'}, pose: {pose: {position: {x: 0.019, y: 0.011, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}"
ros2 topic pub --once /robot_2/nav/goal geometry_msgs/msg/Pose \
"{position: {x: 1.539, y: 0.011, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

wait