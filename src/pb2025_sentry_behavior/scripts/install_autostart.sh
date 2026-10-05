#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
START_SCRIPT="${SCRIPT_DIR}/start_ros_terminals.sh"
AUTOSTART_DIR="${HOME}/.config/autostart"
DESKTOP_FILE="${AUTOSTART_DIR}/pb2025_ros_startup.desktop"

if [[ ! -f "${START_SCRIPT}" ]]; then
  echo "未找到启动脚本: ${START_SCRIPT}" >&2
  exit 1
fi

mkdir -p "${AUTOSTART_DIR}"
chmod +x "${START_SCRIPT}"

cat > "${DESKTOP_FILE}" <<EOF
[Desktop Entry]
Type=Application
Name=PB2025 ROS Startup
Comment=Auto start ROS launch terminals after login
Exec=${START_SCRIPT}
X-GNOME-Autostart-enabled=true
Terminal=false
EOF

echo "已创建自启动项: ${DESKTOP_FILE}"
echo "下次图形登录后将自动打开三个终端并启动对应 ROS 节点。"
