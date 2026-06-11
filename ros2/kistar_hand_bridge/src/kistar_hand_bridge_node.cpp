#include "shm.h"

#include <sys/shm.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace {

using namespace std::chrono_literals;

std_msgs::msg::Float32MultiArray make_message_with_layout(
    const std::vector<std::pair<std::string, std::size_t>>& dims) {
  std_msgs::msg::Float32MultiArray msg;
  msg.layout.dim.reserve(dims.size());

  std::size_t stride = 1;
  for (auto it = dims.rbegin(); it != dims.rend(); ++it) {
    stride *= it->second;
  }

  std::size_t running_stride = stride;
  for (const auto& dim : dims) {
    std_msgs::msg::MultiArrayDimension dimension;
    dimension.label = dim.first;
    dimension.size = static_cast<uint32_t>(dim.second);
    dimension.stride = static_cast<uint32_t>(running_stride);
    running_stride /= dim.second;
    msg.layout.dim.push_back(dimension);
  }

  msg.layout.data_offset = 0;
  msg.data.resize(stride, 0.0f);
  return msg;
}

std_msgs::msg::Float64MultiArray make_message_with_layout_64(
    const std::vector<std::pair<std::string, std::size_t>>& dims) {
  std_msgs::msg::Float64MultiArray msg;
  msg.layout.dim.reserve(dims.size());

  std::size_t stride = 1;
  for (auto it = dims.rbegin(); it != dims.rend(); ++it) {
    stride *= it->second;
  }

  std::size_t running_stride = stride;
  for (const auto& dim : dims) {
    std_msgs::msg::MultiArrayDimension dimension;
    dimension.label = dim.first;
    dimension.size = static_cast<uint32_t>(dim.second);
    dimension.stride = static_cast<uint32_t>(running_stride);
    running_stride /= dim.second;
    msg.layout.dim.push_back(dimension);
  }

  msg.layout.data_offset = 0;
  msg.data.resize(stride, 0.0);
  return msg;
}

}  // namespace

class KistarHandBridgeNode : public rclcpp::Node {
 public:
  KistarHandBridgeNode()
      : Node("kistar_hand_bridge_node"),
        shm_id_(-1),
        shm_ref_(nullptr),
        traj_shm_id_(-1),
        traj_shm_ref_(nullptr),
        joint_position_msg_(make_message_with_layout({{"joint", Hand_DOF}})),
        joint_kinesthetic_msg_(
            make_message_with_layout({{"finger", Kinesthetic_Sensor_Num}, {"axis", Kinesthetic_Sensor_DOF}})),
        joint_tactile_msg_(make_message_with_layout({{"taxel", Tactile_Sensor_Num}})),
        hand_target_msg_(),
        franka_joint_position_msg_(make_message_with_layout_64({{"joint", Arm_DOF}})),
        franka_joint_target_msg_(make_message_with_layout_64({{"joint", Arm_DOF}})),
        franka_joint_velocity_msg_(make_message_with_layout_64({{"joint", Arm_DOF}})),
        franka_joint_torque_msg_(make_message_with_layout_64({{"joint", Arm_DOF}})) {
    const auto shm_key = declare_parameter<int>("shm_key", static_cast<int>(shm_msg_key));
    const auto traj_shm_key =
        declare_parameter<int>("traj_shm_key", static_cast<int>(shm_arm_traj_key));
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 1000.0);

    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be greater than zero");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();

    joint_position_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>("/hand/joint_position", qos);
    joint_kinesthetic_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>("/hand/joint_kinesthetic", qos);
    joint_tactile_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>("/hand/joint_tactile", qos);
    hand_mode_pub_ = create_publisher<std_msgs::msg::Int32>("/hand/hand_mode", qos);
    hand_servo_on_pub_ = create_publisher<std_msgs::msg::Int32>("/hand/servo_on", qos);
    franka_joint_position_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/franka/joint_position", qos);
    franka_joint_target_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/franka/joint_target", qos);
    franka_joint_velocity_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/franka/joint_velocity", qos);
    franka_joint_torque_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("/franka/joint_torque", qos);
    franka_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/franka/speed_factor", qos);
    hand_target_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("/hand/joint_target", qos);

    franka_target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/franka/target_joint",
        qos,
        std::bind(&KistarHandBridgeNode::on_franka_target, this, std::placeholders::_1));
    franka_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/franka/target_speed_factor",
        qos,
        std::bind(&KistarHandBridgeNode::on_franka_speed, this, std::placeholders::_1));
    hand_target_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        "/hand/target_joint",
        qos,
        std::bind(&KistarHandBridgeNode::on_hand_target, this, std::placeholders::_1));
    hand_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/hand/target_mode",
        qos,
        std::bind(&KistarHandBridgeNode::on_hand_mode, this, std::placeholders::_1));
    hand_servo_on_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/hand/target_servo_on",
        qos,
        std::bind(&KistarHandBridgeNode::on_hand_servo_on, this, std::placeholders::_1));
    franka_traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/franka/target_trajectory",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&KistarHandBridgeNode::on_franka_trajectory, this, std::placeholders::_1));

    init_shm(shm_key, shm_id_, &shm_ref_);
    if (shm_ref_ == nullptr) {
      throw std::runtime_error("shared memory attach returned null");
    }
    init_arm_traj_shm(traj_shm_key, traj_shm_id_, &traj_shm_ref_);
    if (traj_shm_ref_ == nullptr) {
      throw std::runtime_error("trajectory shared memory attach returned null");
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&KistarHandBridgeNode::publish_from_shared_memory, this));

    RCLCPP_INFO(get_logger(),
                "Publishing shared memory data at %.1f Hz using shm_key=%d",
                publish_rate_hz,
                static_cast<int>(shm_key));
    RCLCPP_INFO(get_logger(),
                "Using trajectory shm_key=%d",
                static_cast<int>(traj_shm_key));
    RCLCPP_INFO(get_logger(),
                "SHM<->ROS2 bridge enabled for hand/franka states and targets");
  }

  ~KistarHandBridgeNode() override {
    if (traj_shm_ref_ != nullptr) {
      deleteArmTrajSharedMemory(traj_shm_id_, traj_shm_ref_);
      shmdt(static_cast<void*>(traj_shm_ref_));
      traj_shm_ref_ = nullptr;
    }
    if (shm_ref_ != nullptr) {
      deleteSharedMemory(shm_id_, shm_ref_);
      shmdt(static_cast<void*>(shm_ref_));
      shm_ref_ = nullptr;
    }
  }

 private:
  void publish_from_shared_memory() {
    if (shm_ref_ == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(shm_mutex_);

    hand_mode_msg_.data = static_cast<int32_t>(shm_ref_->hand_mode[0]);
    hand_servo_on_msg_.data = static_cast<int32_t>(shm_ref_->servo_on[0]);
    hand_target_msg_.data.resize(Hand_DOF);
    franka_speed_msg_.data = shm_ref_->Arm_Speed_Factor[0];

    for (std::size_t i = 0; i < joint_position_msg_.data.size(); ++i) {
      joint_position_msg_.data[i] = static_cast<float>(shm_ref_->j_pos[0][i]);
      hand_target_msg_.data[i] = shm_ref_->j_tar[0][i];
    }

    std::size_t kin_index = 0;
    for (int finger = 0; finger < Kinesthetic_Sensor_Num; ++finger) {
      for (int axis = 0; axis < Kinesthetic_Sensor_DOF; ++axis) {
        joint_kinesthetic_msg_.data[kin_index++] =
            static_cast<float>(shm_ref_->j_kin[0][finger][axis]);
      }
    }

    for (std::size_t i = 0; i < joint_tactile_msg_.data.size(); ++i) {
      joint_tactile_msg_.data[i] = static_cast<float>(shm_ref_->j_tac[0][i]);
    }
    for (std::size_t i = 0; i < franka_joint_position_msg_.data.size(); ++i) {
      franka_joint_position_msg_.data[i] = shm_ref_->Arm_j_pos[0][i];
      franka_joint_target_msg_.data[i] = shm_ref_->Arm_j_tar[0][i];
      franka_joint_velocity_msg_.data[i] = shm_ref_->Arm_j_vel[0][i];
      franka_joint_torque_msg_.data[i] = shm_ref_->Arm_j_tq[0][i];
    }

    joint_position_pub_->publish(joint_position_msg_);
    joint_kinesthetic_pub_->publish(joint_kinesthetic_msg_);
    joint_tactile_pub_->publish(joint_tactile_msg_);
    hand_mode_pub_->publish(hand_mode_msg_);
    hand_servo_on_pub_->publish(hand_servo_on_msg_);
    hand_target_pub_->publish(hand_target_msg_);
    franka_joint_position_pub_->publish(franka_joint_position_msg_);
    franka_joint_target_pub_->publish(franka_joint_target_msg_);
    franka_joint_velocity_pub_->publish(franka_joint_velocity_msg_);
    franka_joint_torque_pub_->publish(franka_joint_torque_msg_);
    franka_speed_pub_->publish(franka_speed_msg_);
  }

  void on_franka_target(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (shm_ref_ == nullptr) {
      return;
    }
    if (msg->data.size() != static_cast<std::size_t>(Arm_DOF)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "/franka/target_joint size=%zu (expected %d)",
          msg->data.size(), Arm_DOF);
      return;
    }
    std::lock_guard<std::mutex> lock(shm_mutex_);
    for (int i = 0; i < Arm_DOF; ++i) {
      shm_ref_->Arm_j_tar[0][i] = msg->data[i];
    }
  }

  void on_franka_speed(const std_msgs::msg::Float64::SharedPtr msg) {
    if (shm_ref_ == nullptr) {
      return;
    }
    double speed = msg->data;
    if (speed < 1e-3) {
      speed = 1e-3;
    } else if (speed > 1.0) {
      speed = 1.0;
    }
    std::lock_guard<std::mutex> lock(shm_mutex_);
    shm_ref_->Arm_Speed_Factor[0] = speed;
  }

  void on_hand_target(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (shm_ref_ == nullptr) {
      return;
    }
    if (msg->data.size() != static_cast<std::size_t>(Hand_DOF)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "/hand/target_joint size=%zu (expected %d)",
          msg->data.size(), Hand_DOF);
      return;
    }
    std::lock_guard<std::mutex> lock(shm_mutex_);
    for (int i = 0; i < Hand_DOF; ++i) {
      shm_ref_->j_tar[0][i] = msg->data[i];
    }
  }

  void on_hand_mode(const std_msgs::msg::Int32::SharedPtr msg) {
    if (shm_ref_ == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(shm_mutex_);
    shm_ref_->hand_mode[0] = static_cast<uint8_t>(msg->data);
  }

  void on_hand_servo_on(const std_msgs::msg::Int32::SharedPtr msg) {
    if (shm_ref_ == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(shm_mutex_);
    shm_ref_->servo_on[0] = static_cast<uint8_t>(msg->data);
  }

  void on_franka_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    if (traj_shm_ref_ == nullptr) {
      return;
    }
    const int n = static_cast<int>(msg->points.size());
    if (n <= 0 || n > ARM_TRAJ_MAX_PTS) {
      RCLCPP_WARN(get_logger(),
                  "/franka/target_trajectory: %d waypoints (expected 1..%d)",
                  n, ARM_TRAJ_MAX_PTS);
      return;
    }

    // Identify joint index order matching fr3_joint1..7
    // msg->joint_names may differ in order from SHM layout; resolve by name.
    const std::vector<std::string> kJointOrder = {
        "fr3_joint1","fr3_joint2","fr3_joint3","fr3_joint4",
        "fr3_joint5","fr3_joint6","fr3_joint7"};
    std::vector<int> idx(Arm_DOF, -1);
    for (int j = 0; j < Arm_DOF; ++j) {
      for (int k = 0; k < static_cast<int>(msg->joint_names.size()); ++k) {
        if (msg->joint_names[k] == kJointOrder[j]) { idx[j] = k; break; }
      }
    }
    // If joint_names is empty or doesn't match, assume positional order 0..6
    bool use_positional = false;
    for (int j = 0; j < Arm_DOF; ++j) {
      if (idx[j] < 0) { use_positional = true; break; }
    }
    if (use_positional) {
      for (int j = 0; j < Arm_DOF; ++j) idx[j] = j;
    }

    std::lock_guard<std::mutex> lock(shm_mutex_);
    // Write waypoints then set state=PENDING (sequential write ordering)
    traj_shm_ref_->Arm_traj[0].len = n;
    for (int i = 0; i < n; ++i) {
      const auto& pt = msg->points[i];
      for (int j = 0; j < Arm_DOF; ++j) {
        int src = idx[j];
        traj_shm_ref_->Arm_traj[0].q[i][j] =
            (src < static_cast<int>(pt.positions.size())) ? pt.positions[src] : 0.0;
      }
      traj_shm_ref_->Arm_traj[0].t[i] =
          pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
    }
    traj_shm_ref_->Arm_traj[0].state = ARM_TRAJ_PENDING;

    RCLCPP_INFO(get_logger(),
                "/franka/target_trajectory: %d waypoints, duration=%.2f s",
                n, traj_shm_ref_->Arm_traj[0].t[n - 1]);
  }

  int shm_id_;
  SHMmsgs* shm_ref_;
  int traj_shm_id_;
  ArmTrajShm* traj_shm_ref_;
  std::mutex shm_mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_kinesthetic_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_tactile_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hand_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hand_servo_on_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr hand_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr franka_joint_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr franka_joint_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr franka_joint_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr franka_joint_torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr franka_speed_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr franka_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr franka_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr hand_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hand_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hand_servo_on_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr franka_traj_sub_;
  std_msgs::msg::Float32MultiArray joint_position_msg_;
  std_msgs::msg::Float32MultiArray joint_kinesthetic_msg_;
  std_msgs::msg::Float32MultiArray joint_tactile_msg_;
  std_msgs::msg::Int32 hand_mode_msg_;
  std_msgs::msg::Int32 hand_servo_on_msg_;
  std_msgs::msg::Int16MultiArray hand_target_msg_;
  std_msgs::msg::Float64MultiArray franka_joint_position_msg_;
  std_msgs::msg::Float64MultiArray franka_joint_target_msg_;
  std_msgs::msg::Float64MultiArray franka_joint_velocity_msg_;
  std_msgs::msg::Float64MultiArray franka_joint_torque_msg_;
  std_msgs::msg::Float64 franka_speed_msg_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<KistarHandBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("kistar_hand_bridge"),
                 "Failed to start node: %s",
                 exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
