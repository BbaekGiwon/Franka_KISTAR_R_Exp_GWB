#!/usr/bin/env python3
"""White PC: Planning PC 가 보내는 FollowJointTrajectory 액션을 서버로 받아 시퀀스 실행.

  기본: control_msgs/FollowJointTrajectory 액션 서버 (WHITEPC_SETUP 과 동일 계열 이름 가능)
       예) /fr3_arm_controller/follow_joint_trajectory

  Goal 수신 후:
    기본: 바로 시퀀스 실행 (Planning `trajectory_to_whitepc` / forwarder 호환).
    --require-terminal-s: 터미널 **S** 후에만 실행 (goal timeout 주의).

  시퀀스:
    1) Franka 안전 → 2) 핸드 안전(모드2) → 3) grasp 직전(모드2) → 4) 궤적(Goal 내 trajectory)
       → 5) 저장 포인트(모드1) → 6) 대기+스냅샷+500 → 7) Franka 안전
  t  : Franka 즉시 유지; 진행 중 액션은 취소/중단 처리.

  옵션: --traj-topic 이 있으면 JointTrajectory 토픽도 구독(디버그·이중 입력 시 최신만 사용).

백엔드 (--backend):
  whitepc   : kistar_hand_ros2 — /franka/arm_target/{side}, /hand/target/{side},
              상태는 /franka/arm_state/{side}, /hand/state/{side} (whitepc_check.py 와 동일 계열)
  shm_bridge: 이 저장소 ROS2_USAGE.md 의 std_msgs 토픽 (/franka/target_joint 등)

kistar_hand_ros2 메시지 필드명이 다르면 자동 추론에 실패할 수 있음. 그때:
  python3 -c "from kistar_hand_ros2.msg import FrankaArmTarget; print(FrankaArmTarget.get_fields_and_field_types())"
  결과를 보고 scripts/planning_orchestrator.py 상단의 FIELD_HINTS 를 조정.
"""

from __future__ import annotations

import argparse
import select
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from control_msgs.action import FollowJointTrajectory

# --- 핸드 모드 (사용자 확인됨) ---
HAND_MODE_POSITION = 1
HAND_MODE_CIRCULAR = 2

HAND_MOTOR_EXCLUDE_FOR_OFFSET = frozenset({0, 1, 4, 8, 12})
HAND_CLOSE_OFFSET = 500  # 제외 축 제외 +500 (int16)

ARM_DOF = 7
HAND_DOF = 16

# Planning / trajectory_to_whitepc 가 보내는 joint_names 기대 순서 (WHITEPC_SETUP 트러블슈팅 참고)
# 다르면: --joint-names 로 7개 이름을 planning 과 동일 순서로 지정
DEFAULT_JOINT_NAMES_FR3: Tuple[str, ...] = (
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
)
# 일부 파이프라인은 panda_* 이름 사용
ALTERNATE_JOINT_NAMES_PANDA: Tuple[str, ...] = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)

# --- 기본 안전/핸드 포즈 (현장 값, CLI 로 override 가능) ---
FRANKA_SAFE_POSITION: List[float] = [
    0.5084, -0.0591, -0.2496, -2.2724, 0.6068, 2.2322, -0.9324,
]
HAND_SAFE_POSITION: List[int] = [
    4096, -4096, -1000, 1000, -1500, 1000, 1000, 1000, 0, 1000, 1000, 1000, 1500, 1000, 1000, 1000,
]
# grasp 직전 / 저장 포인트(동일 엔코더, 단계별 모드만 다름)
HAND_SAVE_POINT: List[int] = [
    4096, -4096, -1000, 3000, -1500, 1000, 2000, 3000, 0, 1000, 2000, 3000, 1500, 1000, 2000, 3000,
]
HAND_MOVE_DURATION: float = 2.0
HAND_POST_SAVE_SETTLE_SEC: float = 1.5  # 저장 포즈 후 스냅샷 전 대기
HAND_SAFE_SERVO_ON: int = 1


def _ros_ft(cls) -> Dict[str, str]:
    return dict(cls.get_fields_and_field_types())


def _pick_franka_joint_field(cls, prefer: Tuple[str, ...]) -> str:
    ft = _ros_ft(cls)
    for name in prefer:
        if name in ft:
            return name
    for name, typ in ft.items():
        if "sequence<double>" in typ or "double[7]" in typ or typ.startswith("double["):
            return name
    raise RuntimeError(f"{cls.__name__}: 관절 벡터 필드를 찾을 수 없음: {ft}")


def _pick_franka_speed_field(cls) -> Optional[str]:
    ft = _ros_ft(cls)
    for name in ("max_velocity", "speed_factor", "velocity_scale", "max_vel", "speed"):
        if name in ft:
            t = ft[name]
            if "sequence" not in t and "[" not in t:
                return name
    for name, typ in ft.items():
        if typ in ("double", "float64") and any(
            x in name.lower() for x in ("speed", "vel", "factor", "max")
        ):
            return name
    return None


def _pick_hand_joint_field(cls) -> str:
    ft = _ros_ft(cls)
    for name in ("joint_positions", "joint_targets", "positions", "target_positions"):
        if name in ft:
            return name
    for name, typ in ft.items():
        if "int16" in typ or "short" in typ or "sequence<int16>" in typ:
            return name
    raise RuntimeError(f"{cls.__name__}: 핸드 관절 배열 필드를 찾을 수 없음: {ft}")


def _interp_traj(traj: JointTrajectory, t: float) -> List[float]:
    if not traj.points:
        raise ValueError("JointTrajectory.points 비어 있음")
    pts = traj.points

    def pt_time(i: int) -> float:
        p = pts[i]
        return float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9

    if t <= pt_time(0):
        q = pts[0].positions
        if len(q) < ARM_DOF:
            raise ValueError(f"positions 길이 {len(q)} < {ARM_DOF}")
        return [float(q[i]) for i in range(ARM_DOF)]

    for i in range(len(pts) - 1):
        t0, t1 = pt_time(i), pt_time(i + 1)
        if t0 <= t <= t1:
            alpha = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            a, b = pts[i].positions, pts[i + 1].positions
            if len(a) < ARM_DOF or len(b) < ARM_DOF:
                raise ValueError("positions 길이 부족")
            return [float(a[j]) + alpha * (float(b[j]) - float(a[j])) for j in range(ARM_DOF)]

    q = pts[-1].positions
    if len(q) < ARM_DOF:
        raise ValueError(f"positions 길이 {len(q)} < {ARM_DOF}")
    return [float(q[i]) for i in range(ARM_DOF)]


def _traj_duration(traj: JointTrajectory) -> float:
    if not traj.points:
        return 0.0
    p = traj.points[-1]
    return float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9


def _normalize_trajectory_for_arm(traj: JointTrajectory, expected: Sequence[str]) -> JointTrajectory:
    """joint_names 가 있으면 expected 순서로 positions 재배열. expected 가 비면 이름 검사 생략."""
    if not traj.points:
        return traj
    if not expected or len(expected) == 0:
        names = list(traj.joint_names)
        if len(names) not in (0, ARM_DOF):
            raise ValueError(f"joint_names len={len(names)} (0 또는 {ARM_DOF} 만 허용)")
        for i, pt in enumerate(traj.points):
            if len(pt.positions) < ARM_DOF:
                raise ValueError(f"joint_names 비어/검사 생략 모드인데 point[{i}] positions 부족")
        return traj
    names = list(traj.joint_names)
    if len(names) == 0:
        for i, pt in enumerate(traj.points):
            if len(pt.positions) < ARM_DOF:
                raise ValueError(f"joint_names 비어 있고 point[{i}] positions 부족")
        return traj

    if len(names) != ARM_DOF:
        raise ValueError(f"joint_names len={len(names)} (need {ARM_DOF})")

    idx_map: List[int] = []
    for exp in expected:
        if exp not in names:
            raise ValueError(f"joint_names 에 '{exp}' 없음. 실제: {names}")
        idx_map.append(names.index(exp))

    out = JointTrajectory()
    out.header = traj.header
    out.joint_names = list(expected)
    for pt in traj.points:
        new_pt = JointTrajectoryPoint()
        new_pt.time_from_start = pt.time_from_start
        if len(pt.positions) < ARM_DOF:
            raise ValueError("trajectory point positions 부족")
        new_pt.positions = [float(pt.positions[j]) for j in idx_map]
        if len(pt.velocities) >= ARM_DOF:
            new_pt.velocities = [float(pt.velocities[j]) for j in idx_map]
        if len(pt.accelerations) >= ARM_DOF:
            new_pt.accelerations = [float(pt.accelerations[j]) for j in idx_map]
        if len(pt.effort) >= ARM_DOF:
            new_pt.effort = [float(pt.effort[j]) for j in idx_map]
        out.points.append(new_pt)
    return out


class _ShmBridgeBackend:
    """ROS2_USAGE.md kistar_hand_bridge std_msgs 경로."""

    def __init__(self, node: Node):
        from std_msgs.msg import Float32MultiArray, Float64, Float64MultiArray, Int16MultiArray, Int32

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._node = node
        self._q_lock = threading.Lock()
        self._franka_q: List[float] = [0.0] * ARM_DOF
        self._hand_q: List[int] = [0] * HAND_DOF

        self._pub_franka = node.create_publisher(Float64MultiArray, "/franka/target_joint", 10)
        self._pub_speed = node.create_publisher(Float64, "/franka/target_speed_factor", 10)
        self._pub_hand = node.create_publisher(Int16MultiArray, "/hand/target_joint", 10)
        self._pub_mode = node.create_publisher(Int32, "/hand/target_mode", 10)
        self._pub_servo = node.create_publisher(Int32, "/hand/target_servo_on", 10)

        node.create_subscription(
            Float64MultiArray, "/franka/joint_position", self._on_franka_q, qos
        )
        node.create_subscription(
            Float32MultiArray, "/hand/joint_position", self._on_hand_q, qos
        )

    def _on_franka_q(self, msg):
        if len(msg.data) < ARM_DOF:
            return
        with self._q_lock:
            self._franka_q = [float(msg.data[i]) for i in range(ARM_DOF)]

    def _on_hand_q(self, msg):
        if len(msg.data) < HAND_DOF:
            return
        with self._q_lock:
            self._hand_q = [int(round(float(msg.data[i]))) for i in range(HAND_DOF)]

    def hold_franka(self):
        with self._q_lock:
            q = list(self._franka_q)
        m = __import__("std_msgs.msg", fromlist=["Float64MultiArray"]).Float64MultiArray()
        m.data = q
        self._pub_franka.publish(m)

    def publish_franka(self, q: Sequence[float], speed: float):
        from std_msgs.msg import Float64, Float64MultiArray

        sp = Float64()
        sp.data = float(max(1e-3, min(1.0, speed)))
        self._pub_speed.publish(sp)
        m = Float64MultiArray()
        m.data = [float(x) for x in q]
        self._pub_franka.publish(m)

    def publish_hand(self, h: Sequence[int]):
        from std_msgs.msg import Int16MultiArray

        m = Int16MultiArray()
        m.data = [int(max(-32768, min(32767, h[i]))) for i in range(HAND_DOF)]
        self._pub_hand.publish(m)

    def set_hand_mode(self, mode: int):
        from std_msgs.msg import Int32

        self._pub_mode.publish(Int32(data=int(mode)))

    def set_servo(self, on: bool):
        from std_msgs.msg import Int32

        self._pub_servo.publish(Int32(data=1 if on else 0))

    def snapshot_hand(self) -> List[int]:
        with self._q_lock:
            return list(self._hand_q)


class _WhitePcBackend:
    """kistar_hand_ros2 + /franka/arm_target/{side} (WHITEPC_SETUP.md)."""

    def __init__(self, node: Node, side: str):
        try:
            from kistar_hand_ros2.msg import FrankaArmState, FrankaArmTarget, HandState, HandTarget
        except ImportError as e:
            raise RuntimeError(
                "kistar_hand_ros2 가 없습니다. kistar_ws source 후 실행하거나 --backend shm_bridge 사용."
            ) from e

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._node = node
        self._FrankaArmTarget = FrankaArmTarget
        self._FrankaArmState = FrankaArmState
        self._HandTarget = HandTarget
        self._HandState = HandState

        self._fq = _pick_franka_joint_field(FrankaArmState, ("joint_positions", "q", "positions"))
        self._tj = _pick_franka_joint_field(FrankaArmTarget, ("joint_positions", "joint_targets", "positions"))
        self._tv = _pick_franka_speed_field(FrankaArmTarget)
        self._hj = _pick_hand_joint_field(HandState)
        self._htj = _pick_hand_joint_field(HandTarget)

        ft_ht = _ros_ft(HandTarget)
        self._htm = None
        for cand in ("hand_mode", "mode", "control_mode"):
            if cand in ft_ht:
                self._htm = cand
                break

        self._q_lock = threading.Lock()
        self._franka_q: List[float] = [0.0] * ARM_DOF
        self._hand_q: List[int] = [0] * HAND_DOF
        self._last_speed = 0.15
        self._last_hand: List[int] = [0] * HAND_DOF
        self._last_hand_mode: int = HAND_MODE_POSITION

        self._pub_franka = node.create_publisher(FrankaArmTarget, f"/franka/arm_target/{side}", 10)
        self._pub_hand = node.create_publisher(HandTarget, f"/hand/target/{side}", 10)

        node.create_subscription(FrankaArmState, f"/franka/arm_state/{side}", self._on_arm_state, qos)
        node.create_subscription(HandState, f"/hand/state/{side}", self._on_hand_state, qos)

        node.get_logger().info(
            f"whitepc 필드: FrankaArmState.{self._fq}, FrankaArmTarget.{self._tj}"
            + (f", speed={self._tv}" if self._tv else "")
            + f", HandState.{self._hj}, HandTarget.{self._htj}"
            + (f", mode={self._htm}" if self._htm else " (HandTarget 에 모드필드 없음 — 별도 토픽 필요할 수 있음)")
        )

    def _read_float7(self, msg, field: str) -> List[float]:
        v = list(getattr(msg, field))
        if len(v) < ARM_DOF:
            raise ValueError(f"{field} len={len(v)}")
        return [float(v[i]) for i in range(ARM_DOF)]

    def _read_hand16(self, msg, field: str) -> List[int]:
        v = list(getattr(msg, field))
        out = []
        for i in range(HAND_DOF):
            if i < len(v):
                out.append(int(v[i]))
            else:
                out.append(0)
        return out

    def _on_arm_state(self, msg):
        try:
            q = self._read_float7(msg, self._fq)
        except ValueError:
            return
        with self._q_lock:
            self._franka_q = q

    def _on_hand_state(self, msg):
        try:
            h = self._read_hand16(msg, self._hj)
        except ValueError:
            return
        with self._q_lock:
            self._hand_q = h

    def hold_franka(self):
        with self._q_lock:
            q = list(self._franka_q)
        self.publish_franka(q, self._last_speed)

    def publish_franka(self, q: Sequence[float], speed: float):
        self._last_speed = float(max(1e-3, min(1.0, speed)))
        m = self._FrankaArmTarget()
        setattr(m, self._tj, [float(x) for x in q])
        if self._tv:
            setattr(m, self._tv, float(self._last_speed))
        self._pub_franka.publish(m)

    def publish_hand(self, h: Sequence[int]):
        self._last_hand = [int(max(-32768, min(32767, h[i]))) for i in range(HAND_DOF)]
        m = self._HandTarget()
        setattr(m, self._htj, list(self._last_hand))
        if self._htm:
            setattr(m, self._htm, int(self._last_hand_mode))
        self._pub_hand.publish(m)

    def set_hand_mode(self, mode: int):
        self._last_hand_mode = int(mode)
        m = self._HandTarget()
        setattr(m, self._htj, list(self._last_hand))
        if self._htm:
            setattr(m, self._htm, int(mode))
            self._pub_hand.publish(m)
        else:
            self._node.get_logger().warn(
                "HandTarget 에 hand_mode 필드가 없어 set_hand_mode 가 noop 일 수 있습니다. "
                "별도 /hand/target_mode 가 있다면 알려주세요."
            )

    def set_servo(self, on: bool):
        ft = _ros_ft(self._HandTarget)
        for cand in ("servo_on", "target_servo_on", "servo"):
            if cand in ft:
                m = self._HandTarget()
                setattr(m, self._htj, list(self._last_hand))
                if self._htm:
                    setattr(m, self._htm, int(self._last_hand_mode))
                typ = ft[cand].lower()
                setattr(m, cand, bool(on) if "bool" in typ else (1 if on else 0))
                self._pub_hand.publish(m)
                return
        self._node.get_logger().warn("HandTarget 에 servo 필드 없음 — set_servo noop.")

    def snapshot_hand(self) -> List[int]:
        with self._q_lock:
            return list(self._hand_q)


class OrchestratorNode(Node):
    def __init__(
        self,
        backend: str,
        side: str,
        traj_topic: Optional[str],
        action_name: Optional[str],
        abort_event: threading.Event,
        start_seq_event: threading.Event,
        require_terminal_s: bool,
    ):
        super().__init__("planning_orchestrator_whitepc" if backend == "whitepc" else "planning_orchestrator_bridge")
        self._traj_lock = threading.Lock()
        self._latest_traj: Optional[JointTrajectory] = None
        self._traj_ready_announced = False
        self._cb_group = ReentrantCallbackGroup()
        self._abort_event = abort_event
        self._start_seq_event = start_seq_event
        self._require_terminal_s = require_terminal_s
        self._active_goal_handle: Optional[ServerGoalHandle] = None
        self._gh_lock = threading.Lock()

        if backend == "whitepc":
            self.io: object = _WhitePcBackend(self, side)
        else:
            self.io = _ShmBridgeBackend(self)

        if traj_topic:
            self.create_subscription(
                JointTrajectory,
                traj_topic,
                self._on_traj,
                10,
                callback_group=self._cb_group,
            )
            self.get_logger().info(f"JointTrajectory 토픽(옵션): {traj_topic}")

        self._action_server = None
        if action_name:
            self._action_server = ActionServer(
                self,
                FollowJointTrajectory,
                action_name,
                self._execute_follow_joint,
                callback_group=self._cb_group,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            )
            self.get_logger().info(f"FollowJointTrajectory 액션 서버: {action_name}")

        self._franka_safe: List[float] = []
        self._hand_safe: List[int] = []
        self._hand_save: List[int] = []
        self._hand_dur = 2.0
        self._post_wait = 1.5
        self._args_ns: Optional[argparse.Namespace] = None
        self._expected_joint_names: Tuple[str, ...] = ()
        self._exec_busy = threading.Lock()
        self._goal_busy = False

    def attach_sequence_params(
        self,
        franka_safe: Sequence[float],
        hand_safe: Sequence[int],
        hand_save: Sequence[int],
        hand_dur: float,
        post_wait: float,
        args_ns: argparse.Namespace,
        expected_joint_names: Tuple[str, ...],
    ) -> None:
        self._franka_safe = list(franka_safe)
        self._hand_safe = list(hand_safe)
        self._hand_save = list(hand_save)
        self._hand_dur = float(hand_dur)
        self._post_wait = float(post_wait)
        self._args_ns = args_ns
        self._expected_joint_names = tuple(expected_joint_names)

    def _on_traj(self, msg: JointTrajectory):
        with self._traj_lock:
            self._latest_traj = msg
        if not self._traj_ready_announced:
            self._traj_ready_announced = True
            print("\n>>> [ROS2] JointTrajectory 토픽 수신(옵션 구독).\n", flush=True)

    def get_traj_copy(self) -> Optional[JointTrajectory]:
        with self._traj_lock:
            if self._latest_traj is None:
                return None
            return self._latest_traj

    def goal_abort_from_user(self) -> None:
        self._abort_event.set()
        self.io.hold_franka()
        with self._gh_lock:
            gh = self._active_goal_handle
        if gh is not None and gh.is_active:
            try:
                gh.abort()
            except Exception:
                pass

    def _goal_callback(self, goal_request) -> GoalResponse:
        ok, err = _validate_follow_joint_goal(goal_request.trajectory, self._expected_joint_names)
        if not ok:
            self.get_logger().warn(f"Goal reject: {err}")
            return GoalResponse.REJECT
        with self._exec_busy:
            if self._goal_busy:
                self.get_logger().warn("Goal reject: 이전 시퀀스 실행 중")
                return GoalResponse.REJECT
            self._goal_busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self._abort_event.set()
        self.io.hold_franka()
        return CancelResponse.ACCEPT

    def _execute_follow_joint(self, goal_handle: ServerGoalHandle) -> FollowJointTrajectory.Result:
        self._abort_event.clear()
        request = goal_handle.request
        with self._gh_lock:
            self._active_goal_handle = goal_handle

        try:
            if self._args_ns is None:
                goal_handle.abort()
                return _fj_result_reject("internal: attach_sequence_params not called")

            traj_raw = request.trajectory
            traj = _normalize_trajectory_for_arm(traj_raw, self._expected_joint_names)
            if self._require_terminal_s:
                self._start_seq_event.clear()
                print(
                    "\n>>> [Action] FollowJointTrajectory 수신. 터미널에서 **S** 입력 시 시퀀스 시작.\n",
                    flush=True,
                )
                while rclpy.ok():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return _fj_result_abort("canceled")
                    if self._abort_event.is_set():
                        goal_handle.abort()
                        return _fj_result_abort("user abort (t)")
                    if self._start_seq_event.wait(timeout=0.1):
                        self._start_seq_event.clear()
                        break

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return _fj_result_abort("canceled")
            else:
                print("\n>>> [Action] FollowJointTrajectory 수신 → 시퀀스 즉시 시작.\n", flush=True)

            self._abort_event.clear()
            ok = _run_full_sequence(
                self.io,
                traj,
                self._franka_safe,
                self._hand_safe,
                self._hand_save,
                self._hand_dur,
                self._post_wait,
                self._args_ns,
                self._abort_event,
            )
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return _fj_result_abort("canceled during execute")

            if ok:
                goal_handle.succeed()
                return _fj_result_ok()
            goal_handle.abort()
            return _fj_result_abort("sequence failed or aborted")
        finally:
            with self._exec_busy:
                self._goal_busy = False
            with self._gh_lock:
                self._active_goal_handle = None


def _spin_bg(executor):
    executor.spin()


def _stdin_watcher(stop: threading.Event, on_s, on_t):
    while not stop.is_set():
        if sys.stdin.isatty():
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.2)
            except (ValueError, OSError):
                time.sleep(0.2)
                continue
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch.lower() == "s":
                on_s()
            elif ch.lower() == "t":
                on_t()
        else:
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.2)
                continue
            for ch in line.strip().lower():
                if ch == "s":
                    on_s()
                elif ch == "t":
                    on_t()


def _ramp_hand(io, h0: Sequence[int], h1: Sequence[int], duration_s: float, hz: float, abort: threading.Event):
    steps = max(1, int(duration_s * hz))
    dt = duration_s / steps
    for k in range(steps + 1):
        if abort.is_set():
            return
        a = k / steps
        h = [int(round(h0[i] + a * (h1[i] - h0[i]))) for i in range(HAND_DOF)]
        io.publish_hand(h)
        time.sleep(dt)


def _wait_q_close(io, target: Sequence[float], tol: float, timeout_s: float, abort: threading.Event) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if abort.is_set():
            return False
        with io._q_lock:
            qc = list(io._franka_q)
        if all(abs(qc[i] - target[i]) < tol for i in range(ARM_DOF)):
            return True
        io.publish_franka(target, 0.15)
        time.sleep(0.05)
    return False


def _run_trajectory(io, traj: JointTrajectory, speed_scale: float, hz: float, abort: threading.Event):
    T = _traj_duration(traj)
    n = max(2, int(T * hz))
    for i in range(n + 1):
        if abort.is_set():
            io.hold_franka()
            return
        t = T * (i / n)
        q = _interp_traj(traj, t)
        io.publish_franka(q, speed_scale)
        time.sleep(1.0 / hz)
    io.publish_franka(_interp_traj(traj, T), speed_scale)


def _apply_hand_close_offset(h: Sequence[int], delta: int, exclude: frozenset) -> List[int]:
    out = list(h)
    for i in range(HAND_DOF):
        if i in exclude:
            continue
        out[i] = int(out[i]) + delta
        out[i] = max(-32768, min(32767, out[i]))
    return out


def _validate_follow_joint_goal(traj: JointTrajectory, expected: Tuple[str, ...]) -> Tuple[bool, str]:
    if not traj.points:
        return False, "empty trajectory points"
    for i, pt in enumerate(traj.points):
        if len(pt.positions) < ARM_DOF:
            return False, f"point[{i}] positions len={len(pt.positions)} (need {ARM_DOF})"
    try:
        _normalize_trajectory_for_arm(traj, expected)
    except ValueError as e:
        return False, str(e)
    return True, ""


def _run_full_sequence(
    io: object,
    traj: JointTrajectory,
    franka_safe: Sequence[float],
    hand_safe: Sequence[int],
    hand_save: Sequence[int],
    hand_dur: float,
    post_wait: float,
    args: argparse.Namespace,
    abort: threading.Event,
) -> bool:
    print("--- 1) Franka 안전 관절 ---", flush=True)
    io.publish_franka(franka_safe, args.move_speed)
    if not _wait_q_close(io, franka_safe, args.joint_tol, args.pose_wait_timeout, abort):
        print(">>> Franka 안전 자세 중단/타임아웃.\n", flush=True)
        return False

    print("--- 2) 핸드 안전 위치 (모드2, 램프) ---", flush=True)
    io.set_servo(bool(HAND_SAFE_SERVO_ON))
    time.sleep(0.05)
    io.set_hand_mode(HAND_MODE_CIRCULAR)
    time.sleep(0.15)
    if abort.is_set():
        return False
    h_start = io.snapshot_hand()
    _ramp_hand(io, h_start, hand_safe, hand_dur, 40.0, abort)
    if abort.is_set():
        return False

    print("--- 3) 핸드 grasp 직전 (모드2, 램프) ---", flush=True)
    h_at_safe = io.snapshot_hand()
    _ramp_hand(io, h_at_safe, hand_save, hand_dur, 40.0, abort)
    if abort.is_set():
        return False

    print("--- 4) Franka 궤적 ---", flush=True)
    _run_trajectory(io, traj, float(args.max_vel), float(args.traj_hz), abort)
    if abort.is_set():
        return False

    print("--- 5) 핸드 저장 포인트 (모드1, 램프) ---", flush=True)
    io.set_hand_mode(HAND_MODE_POSITION)
    time.sleep(0.15)
    h_after_traj = io.snapshot_hand()
    _ramp_hand(io, h_after_traj, hand_save, hand_dur, 40.0, abort)
    if abort.is_set():
        return False

    print(f"--- 6) {post_wait}s 대기 → 스냅샷 + (제외 축 제외) +{args.hand_close_delta} ---", flush=True)
    time.sleep(post_wait)
    if abort.is_set():
        return False
    snap = io.snapshot_hand()
    boosted = _apply_hand_close_offset(snap, args.hand_close_delta, HAND_MOTOR_EXCLUDE_FOR_OFFSET)
    io.publish_hand(boosted)
    time.sleep(0.05)
    if abort.is_set():
        return False

    print("--- 7) Franka 안전 관절 ---", flush=True)
    io.publish_franka(franka_safe, args.move_speed)
    if not _wait_q_close(io, franka_safe, args.joint_tol, args.pose_wait_timeout, abort):
        print(">>> 최종 Franka 안전 자세 중단/타임아웃.\n", flush=True)
        return False

    print(">>> 시퀀스 완료.\n", flush=True)
    return True


def _fj_result_ok() -> FollowJointTrajectory.Result:
    r = FollowJointTrajectory.Result()
    r.error_code = 0
    r.error_string = "SUCCESSFUL"
    return r


def _fj_result_reject(msg: str) -> FollowJointTrajectory.Result:
    r = FollowJointTrajectory.Result()
    r.error_code = -1
    r.error_string = msg[:200]
    return r


def _fj_result_abort(msg: str) -> FollowJointTrajectory.Result:
    r = FollowJointTrajectory.Result()
    r.error_code = -4
    r.error_string = msg[:200]
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=["whitepc", "shm_bridge"], default="whitepc")
    ap.add_argument("--side", choices=["left", "right"], default="right")
    ap.add_argument(
        "--follow-joint-action",
        default="/fr3_arm_controller/follow_joint_trajectory",
        help="FollowJointTrajectory 액션 서버 이름 (절대 경로)",
    )
    ap.add_argument(
        "--no-follow-joint-action",
        action="store_true",
        help="액션 서버 비활성화 (--traj-topic 으로 토픽만 받을 때 필수)",
    )
    ap.add_argument(
        "--traj-topic",
        default=None,
        help="옵션: JointTrajectory 토픽 구독 (디버그/레거시)",
    )
    ap.add_argument(
        "--require-terminal-s",
        action="store_true",
        help="Goal 수신 후 터미널 S 가 있어야 시퀀스 시작 (Planning goal timeout 을 길게)",
    )
    ap.add_argument(
        "--joint-preset",
        choices=["fr3", "panda", "passthrough"],
        default="fr3",
        help="Planning joint_names 검증/재정렬: fr3_joint* / panda_joint* / passthrough(이름 미검사)",
    )
    ap.add_argument(
        "--joint-names",
        nargs=7,
        default=None,
        metavar="J",
        help="기대 관절 이름 7개 (지정 시 --joint-preset 보다 우선)",
    )
    ap.add_argument(
        "--max-vel",
        type=float,
        default=0.1,
        help="궤적 구간 Franka 속도 계수 (WHITEPC_SETUP 권장 첫값 0.1)",
    )
    ap.add_argument(
        "--traj-hz",
        type=float,
        default=100.0,
        help="궤적 보간 publish Hz (bridge 와 유사하게 100)",
    )
    ap.add_argument(
        "--franka-safe",
        type=float,
        nargs=7,
        default=None,
        metavar=("q1", "q2", "q3", "q4", "q5", "q6", "q7"),
        help="Franka 안전 관절(rad). 생략 시 FRANKA_SAFE_POSITION",
    )
    ap.add_argument(
        "--hand-safe",
        type=int,
        nargs=16,
        default=None,
        help="핸드 안전 엔코더 16. 생략 시 HAND_SAFE_POSITION",
    )
    ap.add_argument(
        "--hand-save",
        type=int,
        nargs=16,
        default=None,
        help="grasp 직전/저장 포인트 엔코더 16. 생략 시 HAND_SAVE_POINT",
    )
    ap.add_argument("--hand-move-duration", type=float, default=None, help="핸드 램프 시간(초). 기본 HAND_MOVE_DURATION")
    ap.add_argument("--post-save-wait", type=float, default=None, help="모드1 저장 포즈 후 스냅샷 전 대기(초). 기본 1.5")
    ap.add_argument("--move-speed", type=float, default=0.10)
    ap.add_argument("--hand-close-delta", type=int, default=HAND_CLOSE_OFFSET)
    ap.add_argument("--joint-tol", type=float, default=0.08)
    ap.add_argument("--pose-wait-timeout", type=float, default=120.0)
    args = ap.parse_args()

    action_name = None if args.no_follow_joint_action else str(args.follow_joint_action)
    traj_topic = args.traj_topic
    if action_name is None and traj_topic is None:
        ap.error("--no-follow-joint-action 이면 --traj-topic 이 필요합니다.")

    franka_safe = list(args.franka_safe) if args.franka_safe is not None else list(FRANKA_SAFE_POSITION)
    hand_safe = list(args.hand_safe) if args.hand_safe is not None else list(HAND_SAFE_POSITION)
    hand_save = list(args.hand_save) if args.hand_save is not None else list(HAND_SAVE_POINT)
    hand_dur = float(args.hand_move_duration) if args.hand_move_duration is not None else float(HAND_MOVE_DURATION)
    post_wait = float(args.post_save_wait) if args.post_save_wait is not None else float(HAND_POST_SAVE_SETTLE_SEC)

    if len(hand_safe) != HAND_DOF or len(hand_save) != HAND_DOF:
        ap.error("hand-safe / hand-save 는 각각 16개 정수여야 합니다.")

    if args.joint_names:
        expected_joint: Tuple[str, ...] = tuple(str(x) for x in args.joint_names)
    elif args.joint_preset == "panda":
        expected_joint = ALTERNATE_JOINT_NAMES_PANDA
    elif args.joint_preset == "passthrough":
        expected_joint = ()
    else:
        expected_joint = DEFAULT_JOINT_NAMES_FR3

    rclpy.init()
    abort = threading.Event()
    start_seq = threading.Event()
    node = OrchestratorNode(
        args.backend,
        args.side,
        traj_topic,
        action_name,
        abort,
        start_seq,
        bool(args.require_terminal_s),
    )
    node.attach_sequence_params(franka_safe, hand_safe, hand_save, hand_dur, post_wait, args, expected_joint)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_th = threading.Thread(target=_spin_bg, args=(executor,), daemon=True)
    spin_th.start()
    time.sleep(0.5)

    parts = []
    if action_name:
        parts.append(f"액션 서버: {action_name}")
    if traj_topic:
        parts.append(f"토픽: {traj_topic}")
    print(">>> [ROS2] " + (" | ".join(parts) if parts else "(액션/토픽 없음)") + "\n", flush=True)

    stop_stdin = threading.Event()

    def on_t():
        node.goal_abort_from_user()
        print("\n>>> [t] Franka 정지 / goal abort\n", flush=True)

    def on_s():
        start_seq.set()
        print("\n>>> [S] 대기 중인 FollowJointTrajectory 실행 허가\n", flush=True)

    threading.Thread(target=_stdin_watcher, args=(stop_stdin, on_s, on_t), daemon=True).start()
    if args.require_terminal_s:
        print(
            "키: **S**=액션 goal 대기 해제, **t**=Franka 정지·goal abort. /shm_ros2_bridge 필요.\n",
            flush=True,
        )
    else:
        print(
            "키: Planning goal 은 **즉시** 시퀀스 실행. **t**=Franka 정지·goal abort. /shm_ros2_bridge 필요.\n",
            flush=True,
        )

    try:
        if action_name is None and traj_topic is not None:
            io = node.io
            while rclpy.ok():
                start_seq.clear()
                abort.clear()
                while rclpy.ok() and not start_seq.is_set():
                    time.sleep(0.05)
                if not rclpy.ok():
                    break
                traj_raw = node.get_traj_copy()
                if traj_raw is None:
                    print(">>> JointTrajectory 없음. publish 후 다시 S.\n", flush=True)
                    continue
                try:
                    traj_use = _normalize_trajectory_for_arm(traj_raw, expected_joint)
                except ValueError as e:
                    print(f">>> 궤적 거부: {e}\n", flush=True)
                    continue
                if not _run_full_sequence(
                    io, traj_use, franka_safe, hand_safe, hand_save, hand_dur, post_wait, args, abort
                ):
                    print(">>> 중단.\n", flush=True)
                print(">>> 토픽 모드 한 사이클 끝. 다시 S 가능.\n", flush=True)
        else:
            while rclpy.ok():
                time.sleep(0.5)
    finally:
        stop_stdin.set()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
