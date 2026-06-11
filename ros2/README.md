# ROS2 Hand Bridge

This workspace publishes hand shared-memory data to ROS2 topics.

## Topics

| Topic | Message Type | Rate | Raw Shape |
|---|---|---|---|
| `/hand/joint_position` | `std_msgs/msg/Float32MultiArray` | 1 kHz | `[16]` |
| `/hand/joint_kinesthetic` | `std_msgs/msg/Float32MultiArray` | 1 kHz | `[12]` (`4 x 3` flatten) |
| `/hand/joint_tactile` | `std_msgs/msg/Float32MultiArray` | 1 kHz | `[60]` |

## Build

```bash
cd ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select kistar_hand_bridge
```

## Run

```bash
cd ros2
./run_hand_bridge.sh
```

Or:

```bash
cd ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kistar_hand_bridge hand_bridge.launch.py
```

If `ros2 launch kistar_hand_bridge hand_bridge.launch.py` prints `Package 'kistar_hand_bridge' not found`, your shell is likely using another workspace's `install/setup.bash`.

Use the setup file from this workspace explicitly:

```bash
cd /home/psw/KISTAR_Hand_RTOS/Franka_KISTAR_R_Exp_V1.2_PtoP/ros2
source /opt/ros/humble/setup.bash
source /home/psw/KISTAR_Hand_RTOS/Franka_KISTAR_R_Exp_V1.2_PtoP/ros2/install/setup.bash
ros2 launch kistar_hand_bridge hand_bridge.launch.py
```

The most reliable method is:

```bash
cd /home/psw/KISTAR_Hand_RTOS/Franka_KISTAR_R_Exp_V1.2_PtoP/ros2
./run_hand_bridge.sh
```
