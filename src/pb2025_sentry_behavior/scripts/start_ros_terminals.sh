#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/home/simon/nav"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_DIR}/install/setup.bash"

CMD1="source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=test18 slam:=False use_robot_state_pub:=True"
CMD2="source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py"
CMD3="source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 launch rm_serial_driver serial_driver.launch.py"

launch_one_terminal() {
  local title="$1"
  local command="$2"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="${title}" -- bash -lc "${command}; exec bash" &
    return
  fi

  if command -v konsole >/dev/null 2>&1; then
    konsole --new-tab -p tabtitle="${title}" -e bash -lc "${command}; exec bash" &
    return
  fi

  if command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --title="${title}" -e "bash -lc \"${command}; exec bash\"" &
    return
  fi

  if command -v xterm >/dev/null 2>&1; then
    xterm -T "${title}" -e bash -lc "${command}; exec bash" &
    return
  fi

  echo "未找到可用终端模拟器（gnome-terminal/konsole/xfce4-terminal/xterm）。" >&2
  exit 1
}

launch_one_terminal "PB Nav Bringup" "${CMD1}"
sleep 1
launch_one_terminal "PB Sentry Behavior" "${CMD2}"
sleep 1
launch_one_terminal "RM Serial Driver" "${CMD3}"
