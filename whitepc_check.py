"""(WHITE PC 측) 옆 PC 의 ROS2 + KISTAR bridge 셋업 검증 스크립트.

목적: 우리 PC 의 grasp pipeline 이 옆 PC 의 KISTAR 컨트롤러에 도달 가능한지
       옆 PC 측에서 한 번 실행해 점검. 실로봇에 명령 안 보냄 (read-only).

옆 PC 에서 실행:
  source /opt/ros/humble/setup.bash
  source <kistar_ws>/install/setup.bash    # /shm_ros2_bridge 가 떠있어야 함
  python3 whitepc_check.py --side right

판정 기준:
  [OK] /shm_ros2_bridge 노드 살아있음
  [OK] /franka/arm_state/{side} publish 되고 있음 (10 Hz 이상)
  [OK] /franka/arm_target/{side} subscriber 1개 이상 (= /shm_ros2_bridge)
  [OK] /hand/state/{side} publish 되고 있음
  [OK] /hand/target/{side} subscriber 1개 이상

이 5개 모두 OK 면 우리 PC 에서 traj bridge + grasp pipeline 보내도 됨.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right"], default="right")
    ap.add_argument("--watch-secs", type=float, default=3.0,
                    help="seconds to watch for state messages")
    args = ap.parse_args()
    side = args.side

    rclpy.init()
    n = Node("whitepc_check")

    try:
        from kistar_hand_ros2.msg import (FrankaArmState, FrankaArmTarget,
                                          HandState, HandTarget)
    except ImportError as e:
        print(f"FAIL: kistar_hand_ros2 messages not available — {e}")
        print("ensure dex_ros kistar_ws is sourced.")
        sys.exit(1)

    print(f"=== white PC check (side={side}) ===")

    # 1) /shm_ros2_bridge node alive?
    nodes = n.get_node_names()
    bridge_alive = any("shm_ros2_bridge" in nm for nm in nodes)
    print(f"[{'OK ' if bridge_alive else 'NG '}] /shm_ros2_bridge node alive: {bridge_alive}")

    # 2) /franka/arm_state/{side} publishing?
    arm_state_count = 0
    def _arm_state_cb(msg):
        nonlocal arm_state_count
        arm_state_count += 1
    sub_arm = n.create_subscription(FrankaArmState, f"/franka/arm_state/{side}", _arm_state_cb, 10)

    # 3) /hand/state/{side} publishing?
    hand_state_count = 0
    last_hand_state = [None]
    def _hand_state_cb(msg):
        nonlocal hand_state_count
        hand_state_count += 1
        last_hand_state[0] = msg
    sub_hand = n.create_subscription(HandState, f"/hand/state/{side}", _hand_state_cb, 10)

    deadline = time.time() + args.watch_secs
    while time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.1)

    rate_arm = arm_state_count / args.watch_secs
    rate_hand = hand_state_count / args.watch_secs
    print(f"[{'OK ' if rate_arm > 1 else 'NG '}] /franka/arm_state/{side} rate ≈ {rate_arm:.1f} Hz")
    print(f"[{'OK ' if rate_hand > 1 else 'NG '}] /hand/state/{side} rate ≈ {rate_hand:.1f} Hz")

    # 4) /franka/arm_target/{side} has subscriber?
    arm_target_pub_info = n.get_subscriptions_info_by_topic(f"/franka/arm_target/{side}")
    n_arm_target_subs = len(arm_target_pub_info)
    print(f"[{'OK ' if n_arm_target_subs >= 1 else 'NG '}] "
          f"/franka/arm_target/{side} subscriber count = {n_arm_target_subs} "
          f"(expect ≥1 = /shm_ros2_bridge)")

    # 5) /hand/target/{side} has subscriber?
    hand_target_pub_info = n.get_subscriptions_info_by_topic(f"/hand/target/{side}")
    n_hand_target_subs = len(hand_target_pub_info)
    print(f"[{'OK ' if n_hand_target_subs >= 1 else 'NG '}] "
          f"/hand/target/{side} subscriber count = {n_hand_target_subs}")

    # bonus: print one current arm/hand state
    if last_hand_state[0] is not None:
        s = last_hand_state[0]
        print(f"  current hand joint_positions[0:4] = {list(s.joint_positions[:4])}")
        print(f"  tactile max = {max(s.tactile_sensors)}, kin max = {max(s.kinesthetic_sensors)}")

    n.destroy_node()
    rclpy.shutdown()

    ok = bridge_alive and rate_arm > 1 and rate_hand > 1 and n_arm_target_subs >= 1 and n_hand_target_subs >= 1
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
