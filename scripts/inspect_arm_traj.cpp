#include "shm.h"

#include <cstdio>
#include <sys/shm.h>

int main() {
  const int shm_id = shmget(shm_arm_traj_key, sizeof(ArmTrajShm), 0666);
  if (shm_id == -1) {
    std::perror("shmget arm trajectory");
    return 1;
  }

  void* raw = shmat(shm_id, nullptr, SHM_RDONLY);
  if (raw == reinterpret_cast<void*>(-1)) {
    std::perror("shmat arm trajectory");
    return 1;
  }

  const auto* shm = static_cast<const ArmTrajShm*>(raw);
  const auto& traj = shm->Arm_traj[0];
  std::printf("key=0x%x shmid=%d process_num=%d state=%d len=%d\n",
              static_cast<unsigned>(shm_arm_traj_key),
              shm_id,
              shm->process_num.load(),
              traj.state,
              traj.len);
  if (traj.len > 0 && traj.len <= ARM_TRAJ_MAX_PTS) {
    std::printf("first_t=%.6f last_t=%.6f\n", traj.t[0], traj.t[traj.len - 1]);
    std::printf("first_q=[");
    for (int i = 0; i < Arm_DOF; ++i) {
      std::printf("%s%.6f", (i == 0 ? "" : ", "), traj.q[0][i]);
    }
    std::printf("]\nlast_q=[");
    for (int i = 0; i < Arm_DOF; ++i) {
      std::printf("%s%.6f", (i == 0 ? "" : ", "), traj.q[traj.len - 1][i]);
    }
    std::printf("]\n");
  }

  shmdt(raw);
  return 0;
}
