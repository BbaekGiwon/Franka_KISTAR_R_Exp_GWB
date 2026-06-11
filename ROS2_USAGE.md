# Franka_KISTAR_R_Exp_GWB_V1.0 — ROS2 사용법

EXP_PC에서 FR3 + KISTAR Hand를 구동하는 운영 문서입니다.
C++ 실시간 프로세스가 EtherCAT(Hand) + Franka를 제어하고, ROS2 SHM 브리지가
공유메모리를 ROS2 토픽으로 노출합니다. 외부(예: Topdown_Grasp PC)에서 토픽으로
**waypoint trajectory** 또는 단일 관절 타겟을 보내면 FR3가 실행합니다.

> 경로는 EXP_PC 기준 `/home/prime/...` 입니다. 환경에 맞게 수정하세요.

---

## 실행 — 두 개 터미널

### 터미널 1 — C++ 실시간 프로세스

```bash
cd /home/prime/Franka_KISTAR_R_Exp_GWB_V1.0

# 빌드 (처음 또는 코드 수정 후)
mkdir -p build && cd build
cmake .. && make -j$(nproc)
cd ..

# 실행 (sudo 필요: EtherCAT raw socket)
sudo ./build/test/Franka_KISTAR_R_Exp_V1
```

하는 일:
- EtherCAT으로 KISTAR Hand 제어
- Franka 로봇 연결 (`172.16.0.1`)
- SHM에 상태 쓰기 / 타겟·trajectory 읽기 후 1 kHz joint-position 루프로 실행

### 터미널 2 — ROS2 SHM 브리지

```bash
cd /home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/ros2

# 실행 (첫 실행 시 자동으로 colcon 빌드)
./run_hand_bridge.sh
```

---

## 토픽 인터페이스

브리지 노드(`kistar_hand_bridge_node`)가 노출하는 토픽 (코드 기준).

### 상태 읽기 — SHM → ROS2 (브리지가 publish)

| 토픽 | 타입 | 비고 |
|------|------|------|
| `/franka/joint_position` | `Float64MultiArray` | 현재 관절 각도 (7) |
| `/franka/joint_target`   | `Float64MultiArray` | 현재 관절 타겟 (7) |
| `/franka/joint_velocity` | `Float64MultiArray` | (7) |
| `/franka/joint_torque`   | `Float64MultiArray` | (7) |
| `/franka/speed_factor`   | `Float64`           | 현재 속도 계수 |
| `/hand/joint_position`   | `Float32MultiArray` | 현재 손 위치 (16) |
| `/hand/joint_kinesthetic`| `Float32MultiArray` | |
| `/hand/joint_tactile`    | `Float32MultiArray` | |
| `/hand/joint_target`     | `Int16MultiArray`   | 현재 손 타겟 (16) |
| `/hand/hand_mode`        | `Int32`             | |
| `/hand/servo_on`         | `Int32`             | |

### 타겟 쓰기 — ROS2 → SHM (브리지가 subscribe)

| 토픽 | 타입 | 비고 |
|------|------|------|
| `/franka/target_trajectory`   | `trajectory_msgs/JointTrajectory` | **waypoint trajectory** (1~1000 pts) |
| `/franka/target_joint`        | `Float64MultiArray` | 단일 관절 타겟 (7, PtoP) |
| `/franka/target_speed_factor` | `Float64`           | 속도 계수 (0.001 ~ 1.0) |
| `/hand/target_joint`          | `Int16MultiArray`   | 손 타겟 (16) |
| `/hand/target_mode`           | `Int32`             | 손 모드 |
| `/hand/target_servo_on`       | `Int32`             | 손 서보 ON/OFF |

---

## 수동 조작 (터미널 3)

### 환경 세팅 (이 터미널에서 한 번만)

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=9
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

### 상태 읽기 (echo)

```bash
# Franka 현재 관절 각도 / 타겟 (7축, double)
ros2 topic echo /franka/joint_position
ros2 topic echo /franka/joint_target

# Franka 속도/토크
ros2 topic echo /franka/joint_velocity
ros2 topic echo /franka/joint_torque

# Hand 현재 위치 (16축, int16)
ros2 topic echo /hand/joint_position

# Hand servo_on / hand_mode 확인
ros2 topic echo /hand/servo_on --once
ros2 topic echo /hand/hand_mode --once
```

### 타겟 쓰기 (pub)

```bash
# 1) Hand 서보 ON
ros2 topic pub /hand/target_servo_on std_msgs/msg/Int32 "{data: 1}" --once

# 2) Hand 모드 설정
ros2 topic pub /hand/target_mode std_msgs/msg/Int32 "{data: 1}" --once

# 3) Hand 타겟 보내기 (16축)
ros2 topic pub /hand/target_joint std_msgs/msg/Int16MultiArray \
  "{data: [100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0]}" --once

# 4) Franka 속도 설정 (0.001 ~ 1.0)
ros2 topic pub /franka/target_speed_factor std_msgs/msg/Float64 \
  "{data: 0.1}" --once

# 5) Franka 단일 관절 타겟 (7축, radian) — PtoP
ros2 topic pub /franka/target_joint std_msgs/msg/Float64MultiArray \
  "{data: [0.9, 0.05, -0.7, -1.9, -0.4, 1.9, -0.7]}" --once
```

---

## Waypoint trajectory 보내기

`/franka/target_trajectory` 로 `trajectory_msgs/JointTrajectory` 를 발행합니다.

- `points` 는 1~1000개. 각 point는 `positions`(7축, radian) + `time_from_start`(trajectory 시작 기준 경과 시간).
- `joint_names` 가 `fr3_joint1..fr3_joint7` 이면 이름으로 매핑하고, 비어있거나 매칭이 안 되면 positional order(0..6)로 간주합니다.
- 브리지가 SHM에 기록 후 상태를 `PENDING`으로 바꾸면, C++ 프로세스가 waypoint 사이를 선형 보간하며 1 kHz로 실행합니다.

수동 발행 예시 (2 waypoint):

```bash
ros2 topic pub --once /franka/target_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
  points: [
    {positions: [0.0, -0.7,  0.0, -2.3, 0.0, 1.5, 0.7], time_from_start: {sec: 2, nanosec: 0}},
    {positions: [0.3, -0.5,  0.0, -2.0, 0.0, 1.5, 0.7], time_from_start: {sec: 4, nanosec: 0}}
  ]
}'
```

> 실제 실험에서는 Topdown_Grasp PC의 플래너가 IK로 경로를 계산해
> 이 토픽(또는 PtoP `/franka/target_joint`)을 발행합니다. 플래너 쪽 실행법은
> [Topdown_Grasp](https://github.com/KIST-HARILAB/Topdown_Grasp) 저장소를 참고하세요.

---

## 연결 / 문제 해결

```bash
# 양쪽 PC 공통 환경변수
export ROS_DOMAIN_ID=9
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# 로봇 상태 수신 확인
ros2 topic echo /franka/joint_position --once

# 타겟 토픽 흐름 확인
ros2 topic hz /franka/target_trajectory
```

| 증상 | 해결 |
|------|------|
| `/franka/joint_position` 수신 안 됨 | 터미널 1·2 실행 여부 확인 |
| trajectory를 보냈는데 안 움직임 | `--speed_factor`(속도 계수) 0.1 이상인지, waypoint 수가 1~1000인지, 브리지 로그에서 `PENDING` 기록 확인 |
| waypoint 매핑이 이상함 | `joint_names`가 `fr3_joint1..7`인지, 아니면 positional order로 들어가는지 확인 |
| DDS 연결 안 됨 | 방화벽 `sudo ufw allow 7400:7500/udp`, `ROS_DOMAIN_ID` 일치 확인 |
