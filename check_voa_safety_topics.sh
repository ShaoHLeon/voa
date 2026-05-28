#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO_NAME="${ROS_DISTRO:-noetic}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
VOA_WS="${VOA_WS:-${SCRIPT_DIR}}"
TRANSFER_WS="${TRANSFER_WS:-${COG_ROOT}/transfer}"

# 如果某个 setup.bash 文件存在，就 source 它
source_if_exists() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    # shellcheck source=/dev/null
    source "${setup_file}"
  fi
}

print_section() {
  echo
  echo "========== $1 =========="
}

source_if_exists "${ROS_SETUP}"
source_if_exists "${TRANSFER_WS}/devel/setup.bash"
source_if_exists "${VOA_WS}/devel/setup.bash"

# 检查 VOA 相关的 systemd 服务状态
print_section "systemd transfer services"
for service in transfer.service message_transformer.service; do
  if systemctl list-unit-files "${service}" >/dev/null 2>&1; then
    printf "%-32s %s\n" "${service}" "$(systemctl is-active "${service}" 2>/dev/null || true)"
  fi
done

print_section "ROS nodes"
# 显示包含'safety|ros2qnx|qnx2ros|message|transformer'的节点
rosnode list | grep -E 'safety|ros2qnx|qnx2ros|message|transformer' || true

print_section "/cmd_vel"
rostopic info /cmd_vel || true

print_section "/cmd_vel_corrected"
rostopic info /cmd_vel_corrected || true

print_section "/cmd_vel_disabled_by_voa"
rostopic info /cmd_vel_disabled_by_voa || true

print_section "VOA safety status sample"
timeout 3s rostopic echo -n 1 /voa/safety_status || true

print_section "topic rates"
timeout 6s rostopic hz /camera/depth/color/points /leg_odom /elevation_mapping/elevation_map /cmd_vel_corrected || true
