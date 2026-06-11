#include "shm_print.h"
#include "shm.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace {

constexpr int kArmIndex = 0;
constexpr int kHandIndex = 0;
constexpr int kDefaultIntervalMs = 100;
constexpr int kPrintLines = 5;  // Arm q, Arm tar, Hand j_pos, Hand j_tar, servo/mode

}  // namespace

extern "C" void* shm_print_thread(void* ptr) {
  int interval_ms = kDefaultIntervalMs;
  if (ptr != nullptr) {
    interval_ms = *static_cast<int*>(ptr);
  }
  if (interval_ms <= 0) {
    interval_ms = kDefaultIntervalMs;
  }

  int shm_id = -1;
  SHMmsgs* shm_msgs = nullptr;
  init_shm(shm_msg_key, shm_id, &shm_msgs);
  if (shm_msgs == nullptr) {
    std::fprintf(stderr, "[shm_print] init_shm failed\n");
    return nullptr;
  }

  // 출력 루프 시작 전 300ms 대기
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  bool first = true;
  while (true) {
    if (!first) {
      std::fprintf(stderr, "\033[%dA", kPrintLines);  // cursor up
    }
    first = false;

    // Arm 현재값 (한 줄)
    std::fprintf(stderr, "[SHM] Arm q   =[");
    for (int i = 0; i < Arm_DOF; ++i) {
      std::fprintf(stderr, "%.4f", shm_msgs->Arm_j_pos[kArmIndex][i]);
      if (i < Arm_DOF - 1) std::fprintf(stderr, ", ");
    }
    std::fprintf(stderr, "]\n");

    // Arm 타깃 (한 줄 띄움)
    std::fprintf(stderr, "[SHM] Arm tar =[");
    for (int i = 0; i < Arm_DOF; ++i) {
      std::fprintf(stderr, "%.4f", shm_msgs->Arm_j_tar[kArmIndex][i]);
      if (i < Arm_DOF - 1) std::fprintf(stderr, ", ");
    }
    std::fprintf(stderr, "]\n");

    // Hand j_pos
    std::fprintf(stderr, "[SHM] Hand j_pos=[");
    for (int i = 0; i < Hand_DOF; ++i) {
      std::fprintf(stderr, "%d", static_cast<int>(shm_msgs->j_pos[kHandIndex][i]));
      if (i < Hand_DOF - 1) std::fprintf(stderr, ", ");
    }
    std::fprintf(stderr, "]\n");

    // Hand j_tar
    std::fprintf(stderr, "[SHM] Hand j_tar=[");
    for (int i = 0; i < Hand_DOF; ++i) {
      std::fprintf(stderr, "%d", static_cast<int>(shm_msgs->j_tar[kHandIndex][i]));
      if (i < Hand_DOF - 1) std::fprintf(stderr, ", ");
    }
    std::fprintf(stderr, "]\n");

    // servo_on, mode
    std::fprintf(stderr, "[SHM] servo_on=%u  hand_mode=%u\n",
                 static_cast<unsigned>(shm_msgs->servo_on[kHandIndex]),
                 static_cast<unsigned>(shm_msgs->hand_mode[kHandIndex]));
    std::fflush(stderr);

    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
  }

  return nullptr;
}
