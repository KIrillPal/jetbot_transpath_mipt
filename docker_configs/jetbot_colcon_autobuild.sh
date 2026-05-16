#!/usr/bin/env bash
# При каждом старте контейнера: colcon jetbot_grid_planner + jetbot_bringup.
# Пропуск: SKIP_COLCON_ON_START=1 в environment docker-compose.
set -euo pipefail
if [[ "${SKIP_COLCON_ON_START:-0}" == "1" ]]; then
  echo "[jetbot_colcon_autobuild] SKIP_COLCON_ON_START=1 — пропуск сборки."
  exit 0
fi
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/install/setup.bash}"
WS_DIR="${WS_DIR:-/home/app/ros2_ws}"
if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$ROS_SETUP"
fi
cd "$WS_DIR"
export COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS:-2}"
colcon build --symlink-install \
  --packages-select jetbot_grid_planner jetbot_bringup \
  --event-handlers console_direct+
# shellcheck source=/dev/null
source "$WS_DIR/install/setup.bash"
