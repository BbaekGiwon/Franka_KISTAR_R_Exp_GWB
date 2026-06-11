from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "9"),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
            DeclareLaunchArgument("shm_key", default_value="14641"),
            DeclareLaunchArgument("traj_shm_key", default_value="14642"),
            DeclareLaunchArgument("publish_rate_hz", default_value="1000.0"),
            Node(
                package="kistar_hand_bridge",
                executable="kistar_hand_bridge_node",
                name="kistar_hand_bridge_node",
                output="screen",
                parameters=[
                    {
                        "shm_key": LaunchConfiguration("shm_key"),
                        "traj_shm_key": LaunchConfiguration("traj_shm_key"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                    }
                ],
            ),
        ]
    )
