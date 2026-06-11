#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ROS_DOMAIN_ID=9
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

set +u
source /opt/ros/humble/setup.bash
set -u

# 최초 실행 시 colcon 빌드
if [ ! -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "=== kistar_hand_bridge 첫 빌드 중... ==="
    (cd "${SCRIPT_DIR}" && colcon build --packages-select kistar_hand_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release)
    echo "=== 빌드 완료 ==="
fi

set +u
source "${SCRIPT_DIR}/install/setup.bash"
set -u

exec ros2 launch kistar_hand_bridge hand_bridge.launch.py
