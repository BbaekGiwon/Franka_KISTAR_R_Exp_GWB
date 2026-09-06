# Franka_KISTAR_R_Exp_GWB

Real-time control repository for the **Franka Research 3 (FR3)** arm with a **KISTAR Hand**.
It receives **joint-space waypoint trajectories** over ROS2 and executes them on the FR3 in real time, while driving the KISTAR Hand over EtherCAT.

Designed to work together with [Topdown_Grasp](https://github.com/BbaekGiwon/Topdown_Grasp), which runs the perception / motion-planning side and publishes the trajectory targets.

- Original code by **[Jaesung Lee](https://github.com/JayLee00)** (PRIME LAB) — [Franka_KISTAR_R_Exp_V1.2_PtoP](https://github.com/JayLee00/Franka_KISTAR_R_Exp_V1.2_PtoP)
- Modified by **[Giwon Baek](https://github.com/BbaekGiwon)** (HARI LAB), 2026.06 — Replaced PtoP control with waypoint-trajectory execution.

---

## Architecture

This repository runs on the **experiment PC (EXP_PC)** that is physically wired to the robot.
It exposes the robot to ROS2 through a shared-memory (SHM) bridge:

```
Topdown_Grasp PC (perception / planning)        EXP_PC (this repo)
──────────────────────────────────────          ──────────────────────────────
  MoveIt (IK + path planning)                    [1] C++ real-time process
  robot_executor.py                                  - FR3 control (172.16.0.1)
    └─ PUB /franka/target_joint  ───────────►        - KISTAR Hand over EtherCAT
       PUB /hand/target_joint                        - SHM read/write
                                                  [2] ROS2 SHM bridge
                                                     - SHM <-> ROS2 topics
```

The C++ process pulls waypoint trajectories from SHM and runs them under FR3's
1 kHz joint-position control loop (linear interpolation between waypoints).

---

## Prerequisites

- Ubuntu 22.04 + ROS2 Humble
- FR3 reachable on the robot network (default `172.16.0.1`)
- KISTAR Hand on the EtherCAT bus (raw socket → requires `sudo`)
- Build toolchain: `cmake`, `make`, a C++17 compiler
- Bundled dependencies (no separate install needed): `libfranka`, `soem_lib` (SOEM EtherCAT master)

> Paths in the docs assume the EXP_PC home directory `/home/prime`. Adjust to your environment.

---

## Build

```bash
cd /home/prime/Franka_KISTAR_R_Exp_GWB_V1.0
mkdir -p build && cd build
cmake .. && make -j$(nproc)
```

Produces the executable `build/test/Franka_KISTAR_R_Exp_V1`.

---

## Usage

Two terminals on the EXP_PC:

```bash
# Terminal 1 — C++ real-time process (sudo: EtherCAT raw socket)
cd /home/prime/Franka_KISTAR_R_Exp_GWB_V1.0
sudo ./build/test/Franka_KISTAR_R_Exp_V1

# Terminal 2 — ROS2 SHM bridge (auto-builds with colcon on first run)
cd /home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/ros2
./run_hand_bridge.sh
```

For the full topic interface, waypoint-trajectory format, manual commands, and
troubleshooting, see **[ROS2_USAGE.md](ROS2_USAGE.md)**.

---

## License

`libfranka/` is distributed under the Apache License 2.0 (see [libfranka/LICENSE](libfranka/LICENSE)).
See the respective subdirectories for third-party license terms.
