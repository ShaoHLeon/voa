#!/usr/bin/env bash
set -euo pipefail

# 定义项目路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO_NAME="${ROS_DISTRO:-noetic}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

# 设置工作空间路径
VOA_WS="${VOA_WS:-${SCRIPT_DIR}}"
REALSENSE_WS="${REALSENSE_WS:-${COG_ROOT}/drivers/realsense_ws}"
TRANSFER_WS="${TRANSFER_WS:-${COG_ROOT}/transfer}"

# 设置环境变量
ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
ENABLE_URDF="${ENABLE_URDF:-true}"
ENABLE_SAFETY="${ENABLE_SAFETY:-true}"

# 检查文件是否存在，如果不存在则退出
require_file() {
  local file_path="$1"
  local label="$2"

  if [[ ! -f "${file_path}" ]]; then
    echo "[voa-lite] Missing ${label}: ${file_path}" >&2
    exit 1
  fi
}

# 关闭系统自启动的标准通信服务。
# 标准 transfer 会让 ros2qnx 直接订阅 /cmd_vel，可能绕过 safety_node。
stop_system_transfer_service() {
  local service_name="$1"

  # 检查当前系统是否有 systemctl 命令
  if ! command -v systemctl >/dev/null 2>&1; then
    return
  fi

  # 检查当前系统是否安装了指定的服务
  if ! systemctl list-unit-files "${service_name}" --no-legend 2>/dev/null | grep -q .; then
    return
  fi

  # 如果服务正在运行，则停止服务
  if systemctl is-active --quiet "${service_name}"; then
    echo "[voa-lite] ${service_name} is active. Stopping it before VOA starts..."
    sudo systemctl stop "${service_name}"
  fi

  # 如果服务仍然在运行，则退出脚本
  if systemctl is-active --quiet "${service_name}"; then
    echo "[voa-lite] Failed to stop ${service_name}. Stop it manually before continuing:" >&2
    echo "  sudo systemctl stop ${service_name}" >&2
    exit 1
  fi
}

# 打开新的 gnome-terminal 窗口，并运行指定的命令
launch_terminal() {
  local title="$1"
  local command="$2"

  gnome-terminal --title="${title}" -x bash -c "${command}; echo; echo '[voa-lite] ${title} exited. Press Ctrl+D or close this terminal.'; exec bash"
}

# 检查是否安装了 gnome-terminal
if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "[voa-lite] gnome-terminal not found. Run this script on the robot desktop environment." >&2
  exit 1
fi

# 1. 检查文件是否存在
require_file "${ROS_SETUP}" "ROS setup"
require_file "${REALSENSE_WS}/devel/setup.bash" "RealSense workspace setup"
require_file "${TRANSFER_WS}/devel/setup.bash" "transfer workspace setup"
require_file "${VOA_WS}/devel/setup.bash" "voa-lite workspace setup"

# 2. 关闭系统自启动的标准通信服务。
stop_system_transfer_service "transfer.service"
stop_system_transfer_service "message_transformer.service"

echo "[voa-lite] ROS distro: ${ROS_DISTRO_NAME}"
echo "[voa-lite] RealSense workspace: ${REALSENSE_WS}"
echo "[voa-lite] transfer workspace: ${TRANSFER_WS}"
echo "[voa-lite] voa-lite workspace: ${VOA_WS}"
echo "[voa-lite] Opening RealSense, transfer, and VOA terminals..."

# 3. 启动 RealSense 相机
launch_terminal "voa-lite camera" "\
source '${ROS_SETUP}' && \
source '${REALSENSE_WS}/devel/setup.bash' && \
cd '${REALSENSE_WS}' && \
roslaunch realsense2_camera dr_camera.launch enable_pointcloud:=true"

sleep 2

# 4. 启动 transfer
launch_terminal "voa-lite transfer" "\
source '${ROS_SETUP}' && \
source '${TRANSFER_WS}/devel/setup.bash' && \
source '${VOA_WS}/devel/setup.bash' && \
cd '${TRANSFER_WS}' && \
roslaunch deeprobotics_local_height_map message_transformer_voa_safe.launch"

sleep 2

# 5. 启动 voa-lite 避障
launch_terminal "voa-lite voa" "\
source '${ROS_SETUP}' && \
source '${REALSENSE_WS}/devel/setup.bash' && \
source '${TRANSFER_WS}/devel/setup.bash' && \
source '${VOA_WS}/devel/setup.bash' && \
cd '${VOA_WS}' && \
roslaunch deeprobotics_local_height_map height_map_lite.launch \
enable_transformer:=false \
enable_safety:='${ENABLE_SAFETY}' \
enable_urdf:='${ENABLE_URDF}' \
enable_rviz:='${ENABLE_RVIZ}'"

echo "[voa-lite] Started terminals:"
echo "  1. camera: realsense2_camera/dr_camera.launch"
echo "  2. transfer: deeprobotics_local_height_map/message_transformer_voa_safe.launch"
echo "  3. voa: deeprobotics_local_height_map/height_map_lite.launch"
