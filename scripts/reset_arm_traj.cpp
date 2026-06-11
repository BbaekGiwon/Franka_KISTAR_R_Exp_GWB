#include "shm.h"

#include <cstdio>
#include <cstring>
#include <sys/shm.h>

int main() {
  const int shm_id = shmget(shm_arm_traj_key, sizeof(ArmTrajShm), 0666);
  if (shm_id == -1) {
    std::perror("shmget arm trajectory");
    return 1;
  }

  void* raw = shmat(shm_id, nullptr, 0);
  if (raw == reinterpret_cast<void*>(-1)) {
    std::perror("shmat arm trajectory");
    return 1;
  }

  auto* shm = static_cast<ArmTrajShm*>(raw);
  for (int arm = 0; arm < Arm_Num; ++arm) {
    shm->Arm_traj[arm].state = ARM_TRAJ_IDLE;
    shm->Arm_traj[arm].len = 0;
    std::memset(shm->Arm_traj[arm].q, 0, sizeof(shm->Arm_traj[arm].q));
    std::memset(shm->Arm_traj[arm].t, 0, sizeof(shm->Arm_traj[arm].t));
  }

  std::printf("reset arm trajectory shm key=0x%x shmid=%d\n",
              static_cast<unsigned>(shm_arm_traj_key),
              shm_id);
  shmdt(raw);
  return 0;
}
