#!/usr/bin/env python3
"""White PC 로컬 테스트: Planning 과 동일하게 `FollowJointTrajectory` goal 만 전송.

  사용법:
    터미널 A: ./scripts/planning_orchestrator.py --backend whitepc --side right
    터미널 B: ./scripts/whitepc_send_test_follow_goal.py --side right

  동작:
    1) /franka/arm_state/{side} 에서 현재 관절 7개를 한 번 읽음 (kistar_hand_ros2)
    2) 같은 관절에서 joint 하나만 아주 조금 움직이는 2-point trajectory 구성
    3) /fr3_arm_controller/follow_joint_trajectory 로 goal 전송 후 결과 대기

  Franka 가 여전히 안 움직이면: shm_ros2_bridge, 실시간 메인, 도메인, max_vel 등은 orchestrator 쪽과 동일하게 점검.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# planning_orchestrator 기본과 동일 (joint_names 검증 통과용)
DEFAULT_JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]
PANDA_JOINT_NAMES = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]


def _dur(sec: float) -> Duration:
    d = Duration()
    s = int(sec)
    d.sec = s
    d.nanosec = int(round((sec - s) * 1e9))
    return d


class Sender(Node):
    def __init__(self, side: str, joint_preset: str):
        super().__init__("whitepc_send_test_follow_goal")
        from kistar_hand_ros2.msg import FrankaArmState

        self._side = side
        self._joint_names = (
            PANDA_JOINT_NAMES if joint_preset == "panda" else DEFAULT_JOINT_NAMES
        )
        self._q: Optional[List[float]] = None
        self._evt = threading.Event()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            FrankaArmState,
            f"/franka/arm_state/{side}",
            self._on_state,
            qos,
        )
        self._client = ActionClient(self, FollowJointTrajectory, "/fr3_arm_controller/follow_joint_trajectory")

    def _on_state(self, msg):
        if self._q is not None:
            return
        if not hasattr(msg, "joint_positions") or len(msg.joint_positions) < 7:
            return
        self._q = [float(msg.joint_positions[i]) for i in range(7)]
        self._evt.set()

    def wait_state(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._evt.is_set():
                return True
        return False

    def send_motion(self, delta_rad: float, joint_index: int, duration_sec: float) -> int:
        if self._q is None:
            print("FAIL: arm state 없음", file=sys.stderr)
            return 2
        if not self._client.wait_for_server(timeout_sec=15.0):
            print("FAIL: action server 없음 (/fr3_arm_controller/follow_joint_trajectory)", file=sys.stderr)
            return 3

        q0 = list(self._q)
        q1 = list(q0)
        ji = max(0, min(6, joint_index))
        q1[ji] = float(q1[ji]) + float(delta_rad)

        p0 = JointTrajectoryPoint()
        p0.positions = q0
        p0.time_from_start = _dur(0.0)
        p1 = JointTrajectoryPoint()
        p1.positions = q1
        p1.time_from_start = _dur(duration_sec)

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        traj.points = [p0, p1]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        print(f"goal 전송: joint[{ji}] += {delta_rad} rad, T={duration_sec}s, names={self._joint_names[0]}…")
        fut = self._client.send_goal_async(goal)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        if not gh.accepted:
            print("FAIL: goal rejected", file=sys.stderr)
            return 4
        rfut = gh.get_result_async()
        while rclpy.ok() and not rfut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        res = rfut.result().result
        print(f"result error_code={res.error_code} msg={res.error_string!r}")
        return 0 if res.error_code == 0 else 5


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", choices=["left", "right"], default="right")
    ap.add_argument("--joint-preset", choices=["fr3", "panda"], default="fr3")
    ap.add_argument("--wait-state", type=float, default=8.0, help="arm_state 수신 대기(초)")
    ap.add_argument("--delta-rad", type=float, default=0.06, help="테스트로 더할 관절각(rad)")
    ap.add_argument("--joint-index", type=int, default=3, help="0..6")
    ap.add_argument("--duration-sec", type=float, default=5.0, help="궤적 마지막 point 시간")
    args = ap.parse_args()

    try:
        from kistar_hand_ros2.msg import FrankaArmState  # noqa: F401
    except ImportError as e:
        print(f"FAIL: kistar_hand_ros2 필요 — {e}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = Sender(args.side, args.joint_preset)
    try:
        print("arm_state 대기 중…")
        if not node.wait_state(args.wait_state):
            print("FAIL: /franka/arm_state timeout", file=sys.stderr)
            sys.exit(2)
        code = node.send_motion(args.delta_rad, args.joint_index, args.duration_sec)
        sys.exit(code)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
