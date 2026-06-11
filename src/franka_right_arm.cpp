
#include "franka_right_arm.h"
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "shm.h"

namespace {

constexpr int kRightArmIndex = 0;
constexpr int kDefaultControlPeriodUs = 1000;
constexpr int kDefaultPrintIntervalUs = 100000;
constexpr double kTargetEps = 1e-4;
constexpr double kDefaultSpeedFactor = 0.1;
constexpr double kMinSpeedFactor = 1e-3;
constexpr double kMaxSpeedFactor = 1.0;
constexpr std::array<double, 7> kFr3LowerJointLimits{
    {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159}};
constexpr std::array<double, 7> kFr3UpperJointLimits{
    {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159}};

std::mutex g_shm_mutex;
SHMmsgs* g_shm_msgs = nullptr;
int g_shm_id = -1;
ArmTrajShm* g_traj_shm = nullptr;
int g_traj_shm_id = -1;

void ensureShmAttached() {
  std::lock_guard<std::mutex> lock(g_shm_mutex);
  if (g_shm_msgs == nullptr) {
    init_shm(shm_msg_key, g_shm_id, &g_shm_msgs);
  }
  if (g_traj_shm == nullptr) {
    init_arm_traj_shm(shm_arm_traj_key, g_traj_shm_id, &g_traj_shm);
  }
}

void updateRightArmStateToShm(const franka::RobotState& state) {
  std::lock_guard<std::mutex> lock(g_shm_mutex);
  if (g_shm_msgs == nullptr) return;
  for (int i = 0; i < Arm_DOF; ++i) {
    g_shm_msgs->Arm_j_pos[kRightArmIndex][i] = state.q[i];
    g_shm_msgs->Arm_j_vel[kRightArmIndex][i] = state.dq[i];
    g_shm_msgs->Arm_j_tq[kRightArmIndex][i]  = state.tau_J[i];
  }
  std::memcpy(g_shm_msgs->Arm_C_Pos[kRightArmIndex], state.O_T_EE.data(), 16 * sizeof(double));
}

std::array<double, 7> readRightArmTargetFromShm(const std::array<double, 7>& fallback) {
  std::array<double, 7> target = fallback;
  std::lock_guard<std::mutex> lock(g_shm_mutex);
  if (g_shm_msgs == nullptr) return target;
  for (int i = 0; i < Arm_DOF; ++i)
    target[i] = g_shm_msgs->Arm_j_tar[kRightArmIndex][i];
  return target;
}

double readRightArmSpeedFromShm(double fallback) {
  std::lock_guard<std::mutex> lock(g_shm_mutex);
  if (g_shm_msgs == nullptr) return fallback;
  double speed = g_shm_msgs->Arm_Speed_Factor[kRightArmIndex];
  return speed;
}

double sanitizeSpeedFactor(double speed, double fallback) {
  if (!std::isfinite(speed)) return fallback;
  if (speed < kMinSpeedFactor) return kMinSpeedFactor;
  if (speed > kMaxSpeedFactor) return kMaxSpeedFactor;
  return speed;
}

bool isTargetChanged(const std::array<double, 7>& a, const std::array<double, 7>& b) {
  for (size_t i = 0; i < a.size(); ++i)
    if (std::abs(a[i] - b[i]) > kTargetEps) return true;
  return false;
}

bool validateWaypoint(const std::array<double, 7>& q, int index) {
  for (int j = 0; j < Arm_DOF; ++j) {
    if (!std::isfinite(q[j])) {
      std::cerr << "[Traj] invalid waypoint " << index << ": joint" << (j + 1)
                << " is not finite (" << q[j] << ")\n";
      return false;
    }
    if (q[j] < kFr3LowerJointLimits[j] || q[j] > kFr3UpperJointLimits[j]) {
      std::cerr << "[Traj] invalid waypoint " << index << ": joint" << (j + 1)
                << "=" << q[j] << " outside ["
                << kFr3LowerJointLimits[j] << ", " << kFr3UpperJointLimits[j] << "]\n";
      return false;
    }
  }
  return true;
}

void markTrajectoryDone() {
  std::lock_guard<std::mutex> lock(g_shm_mutex);
  if (g_traj_shm != nullptr) {
    g_traj_shm->Arm_traj[kRightArmIndex].state = ARM_TRAJ_DONE;
  }
}

// Linear interpolation helper
std::array<double, 7> lerp7(const std::array<double, 7>& a,
                             const std::array<double, 7>& b, double alpha) {
  std::array<double, 7> out;
  for (int i = 0; i < 7; ++i)
    out[i] = a[i] + alpha * (b[i] - a[i]);
  return out;
}

// Execute one trajectory from SHM buffer using joint position control.
// Returns the final joint positions.
std::array<double, 7> executeTrajFromShm(franka::Robot& robot, SHMmsgs* shm, ArmTrajShm* traj_shm) {
  const int arm = kRightArmIndex;

  // Copy trajectory locally under lock (bridge may update SHM during copy)
  int len = 0;
  std::vector<std::array<double, 7>> waypoints;
  std::vector<double> times;
  {
    std::lock_guard<std::mutex> lock(g_shm_mutex);
    if (shm == nullptr || traj_shm == nullptr) {
      return {};
    }
    traj_shm->Arm_traj[arm].state = ARM_TRAJ_EXECUTING;
    len = traj_shm->Arm_traj[arm].len;
    if (len <= 0 || len > ARM_TRAJ_MAX_PTS) {
      traj_shm->Arm_traj[arm].state = ARM_TRAJ_DONE;
      return {};
    }
    waypoints.resize(len);
    times.resize(len);
    for (int i = 0; i < len; ++i) {
      for (int j = 0; j < Arm_DOF; ++j)
        waypoints[i][j] = traj_shm->Arm_traj[arm].q[i][j];
      times[i] = traj_shm->Arm_traj[arm].t[i];
      if (!validateWaypoint(waypoints[i], i)) {
        traj_shm->Arm_traj[arm].state = ARM_TRAJ_DONE;
        return {};
      }
      if (!std::isfinite(times[i]) || times[i] < 0.0 ||
          (i > 0 && times[i] <= times[i - 1])) {
        std::cerr << "[Traj] invalid time at waypoint " << i << ": " << times[i] << "\n";
        traj_shm->Arm_traj[arm].state = ARM_TRAJ_DONE;
        return {};
      }
    }
  }

  const double total_dur = times[len - 1];
  double elapsed = 0.0;
  std::array<double, 7> q_last = waypoints[len - 1];

  std::cerr << "[Traj] executing " << len << " waypoints, duration=" << total_dur << "s\n";

  try {
    robot.control(
    [&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
      elapsed += period.toSec();

      // Update SHM state (no lock: hot path, double writes are harmless)
      for (int i = 0; i < Arm_DOF; ++i) {
        shm->Arm_j_pos[arm][i] = state.q[i];
        shm->Arm_j_vel[arm][i] = state.dq[i];
        shm->Arm_j_tq[arm][i]  = state.tau_J[i];
      }

      // Find segment by linear search (len <= 1000, 1kHz → negligible cost)
      std::array<double, 7> q_des;
      if (elapsed >= total_dur) {
        q_des = waypoints[len - 1];
      } else {
        int seg = 0;
        for (int i = 0; i < len - 1; ++i) {
          if (elapsed >= times[i] && elapsed < times[i + 1]) { seg = i; break; }
        }
        double dt = times[seg + 1] - times[seg];
        double alpha = (dt > 1e-9) ? (elapsed - times[seg]) / dt : 0.0;
        q_des = lerp7(waypoints[seg], waypoints[seg + 1], alpha);
      }

      // Finish 50 ms after the end of the trajectory
      if (elapsed >= total_dur + 0.05) {
        return franka::MotionFinished(franka::JointPositions(waypoints[len - 1]));
      }
      return franka::JointPositions(q_des);
    },
    true
    );
  } catch (...) {
    markTrajectoryDone();
    throw;
  }

  {
    std::lock_guard<std::mutex> lock(g_shm_mutex);
    traj_shm->Arm_traj[arm].state = ARM_TRAJ_DONE;
    // Sync Arm_j_tar so PtoP mode does not immediately re-trigger
    for (int i = 0; i < Arm_DOF; ++i)
      shm->Arm_j_tar[arm][i] = waypoints[len - 1][i];
  }

  std::cerr << "[Traj] done\n";
  return q_last;
}

}  // namespace

void* franka_control_thread_R(void* ptr) {
  FrankaRightArmConfig default_cfg{"172.16.0.1", kDefaultSpeedFactor,
                                   kDefaultControlPeriodUs, kDefaultPrintIntervalUs};
  const FrankaRightArmConfig* cfg = static_cast<const FrankaRightArmConfig*>(ptr);
  const FrankaRightArmConfig& c   = (cfg == nullptr) ? default_cfg : *cfg;
  const int period_us = (c.control_period_us > 0) ? c.control_period_us : kDefaultControlPeriodUs;

  while (true) {
  try {
    ensureShmAttached();
    std::cerr << "Connecting Franka right arm: " << c.robot_ip << std::endl;

    franka::Robot robot(c.robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();

    robot.setCollisionBehavior(
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    franka::RobotState robot_state = robot.readOnce();
    updateRightArmStateToShm(robot_state);

    std::array<double, 7> last_target = robot_state.q;
    {
      std::lock_guard<std::mutex> lock(g_shm_mutex);
      if (g_shm_msgs != nullptr) {
        for (int i = 0; i < Arm_DOF; ++i)
          g_shm_msgs->Arm_j_tar[kRightArmIndex][i] = robot_state.q[i];
        g_shm_msgs->Arm_Speed_Factor[kRightArmIndex] = kDefaultSpeedFactor;
      }
      if (g_traj_shm != nullptr) {
        const int state = g_traj_shm->Arm_traj[kRightArmIndex].state;
        if (state != ARM_TRAJ_PENDING && state != ARM_TRAJ_EXECUTING) {
          g_traj_shm->Arm_traj[kRightArmIndex].state = ARM_TRAJ_IDLE;
          g_traj_shm->Arm_traj[kRightArmIndex].len = 0;
        }
      }
    }

    while (true) {
      robot_state = robot.readOnce();
      updateRightArmStateToShm(robot_state);

      // ── Trajectory mode: takes priority over PtoP ──────────────────────
      int traj_state;
      {
        std::lock_guard<std::mutex> lock(g_shm_mutex);
        traj_state = (g_traj_shm != nullptr)
                     ? g_traj_shm->Arm_traj[kRightArmIndex].state
                     : ARM_TRAJ_IDLE;
      }
      if (traj_state == ARM_TRAJ_PENDING) {
        std::cerr << "[Traj] pending trajectory detected\n";
        last_target = executeTrajFromShm(robot, g_shm_msgs, g_traj_shm);
        // After trajectory, re-read state for next iteration
        robot_state = robot.readOnce();
        updateRightArmStateToShm(robot_state);
        continue;
      }

      // ── PtoP mode (legacy single-target) ──────────────────────────────
      const std::array<double, 7> target = readRightArmTargetFromShm(last_target);
      if (isTargetChanged(target, last_target)) {
        const double spd = sanitizeSpeedFactor(
            readRightArmSpeedFromShm(c.speed_factor), c.speed_factor);
        MotionGenerator motion_generator(spd, target);
        robot.control(motion_generator);
        last_target = target;
        robot_state = robot.readOnce();
        updateRightArmStateToShm(robot_state);
      }

      std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    }
  } catch (const franka::Exception& e) {
    markTrajectoryDone();
    std::cerr << "Franka right arm: " << e.what() << std::endl;
  } catch (const std::exception& e) {
    markTrajectoryDone();
    std::cerr << "Franka right arm error: " << e.what() << std::endl;
  }
    std::cerr << "Franka right arm thread will retry in 1 s\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return nullptr;
}

void* data_print_thread(void* ptr) {
  int interval_us = kDefaultPrintIntervalUs;
  if (ptr != nullptr) interval_us = *static_cast<int*>(ptr);
  if (interval_us <= 0) interval_us = kDefaultPrintIntervalUs;

  ensureShmAttached();

  while (true) {
    if (g_shm_msgs != nullptr) {
      std::lock_guard<std::mutex> lock(g_shm_mutex);
      std::cout << "\rR_arm q[0..2]=["
                << g_shm_msgs->Arm_j_pos[kRightArmIndex][0] << ", "
                << g_shm_msgs->Arm_j_pos[kRightArmIndex][1] << ", "
                << g_shm_msgs->Arm_j_pos[kRightArmIndex][2] << "] "
                << "tau[0..2]=["
                << g_shm_msgs->Arm_j_tq[kRightArmIndex][0] << ", "
                << g_shm_msgs->Arm_j_tq[kRightArmIndex][1] << ", "
                << g_shm_msgs->Arm_j_tq[kRightArmIndex][2] << "] "
                << "Hand_j0=" << g_shm_msgs->j_pos[0][0]
                << "   ";
      std::cout.flush();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(interval_us));
  }

  return nullptr;
}
