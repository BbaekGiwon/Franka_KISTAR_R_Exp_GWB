#include "shm.h"

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <sys/shm.h>

const key_t shm_msg_key = 0x3931;
const key_t shm_rd_key = 10334;
const key_t shm_arm_traj_key = 0x3932;

void init_shm(int shm_key, int &shm_id, SHMmsgs **shm_ref)
{
    if ((shm_id = shmget(shm_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
    {
        std::printf("shm mtx failed\n");
        std::exit(0);
    }

    if ((*shm_ref = static_cast<SHMmsgs *>(shmat(shm_id, nullptr, 0))) == reinterpret_cast<SHMmsgs *>(-1))
    {
        std::printf("shmat failed\n");
        std::exit(0);
    }

    if ((*shm_ref)->process_num == 0)
    {
        std::printf("Process num 0 ! Clean Start!\n");
        std::memset(*shm_ref, 0, sizeof(SHMmsgs));
        for (int arm = 0; arm < Arm_Num; ++arm)
        {
            (*shm_ref)->Arm_Speed_Factor[arm] = 0.1;
        }
    }

    (*shm_ref)->process_num++;
}

void deleteSharedMemory(int shm_id, SHMmsgs *shm_ref)
{
    if (shm_ref == nullptr)
    {
        return;
    }

    shm_ref->process_num--;
    if (shm_ref->process_num == 0)
    {
        std::printf("process num 0. removing shared memory\n");

        if (shmctl(shm_id, IPC_RMID, nullptr) == -1)
        {
            std::printf("shared memory failed to remove.\n");
        }
        else
        {
            std::printf("Shared memory successfully removed\n");
        }
    }
}

void init_arm_traj_shm(int shm_key, int &shm_id, ArmTrajShm **shm_ref)
{
    if ((shm_id = shmget(shm_key, sizeof(ArmTrajShm), IPC_CREAT | 0666)) == -1)
    {
        std::printf("arm traj shm failed\n");
        std::exit(0);
    }

    if ((*shm_ref = static_cast<ArmTrajShm *>(shmat(shm_id, nullptr, 0))) == reinterpret_cast<ArmTrajShm *>(-1))
    {
        std::printf("arm traj shmat failed\n");
        std::exit(0);
    }

    if ((*shm_ref)->process_num == 0)
    {
        std::printf("Arm trajectory SHM clean start\n");
        std::memset(*shm_ref, 0, sizeof(ArmTrajShm));
        for (int arm = 0; arm < Arm_Num; ++arm)
        {
            (*shm_ref)->Arm_traj[arm].state = ARM_TRAJ_IDLE;
            (*shm_ref)->Arm_traj[arm].len = 0;
        }
    }

    (*shm_ref)->process_num++;
}

void deleteArmTrajSharedMemory(int shm_id, ArmTrajShm *shm_ref)
{
    if (shm_ref == nullptr)
    {
        return;
    }

    shm_ref->process_num--;
    if (shm_ref->process_num == 0)
    {
        std::printf("arm traj process num 0. removing shared memory\n");

        if (shmctl(shm_id, IPC_RMID, nullptr) == -1)
        {
            std::printf("arm trajectory shared memory failed to remove.\n");
        }
        else
        {
            std::printf("Arm trajectory shared memory successfully removed\n");
        }
    }
}
