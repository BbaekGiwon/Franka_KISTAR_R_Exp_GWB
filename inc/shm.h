#pragma once

#include <atomic>
#include <cstdint>
#include <sys/ipc.h>

constexpr int Hand_Num = 1;
constexpr int Hand_DOF = 16;
constexpr int Kinesthetic_Sensor_Num = 4;
constexpr int Kinesthetic_Sensor_DOF = 3;
constexpr int Tactile_Sensor_Num = 60;
constexpr int Finger_Num = 4;
constexpr int Cartesian_DOF = 3;
constexpr int Quaternion_DOF = 4;

constexpr int  Arm_Num = 1;
constexpr int  Arm_DOF  = 7;

constexpr int  Glove_Num = 1;
constexpr int  Glove_DOF = 16;
constexpr int  Glove_Tactile_Sensor_Num = 16;

// Trajectory buffer: up to 1000 waypoints per arm
constexpr int ARM_TRAJ_MAX_PTS = 1000;

// state values
constexpr int ARM_TRAJ_IDLE      = 0;  // no trajectory
constexpr int ARM_TRAJ_PENDING   = 1;  // written by bridge, waiting for C++ to pick up
constexpr int ARM_TRAJ_EXECUTING = 2;  // C++ is following the trajectory
constexpr int ARM_TRAJ_DONE      = 3;  // C++ finished

struct ArmTrajBuf {
    double q[ARM_TRAJ_MAX_PTS][Arm_DOF];  // joint positions [rad]
    double t[ARM_TRAJ_MAX_PTS];            // time_from_start [sec]
    int    len;                            // number of valid waypoints
    // state: producer writes data, then sets state=PENDING (sequential).
    // C++ controller reads state, sets EXECUTING, then DONE.
    volatile int state;
};

struct ArmTrajShm {
    ArmTrajBuf Arm_traj[Arm_Num];
    std::atomic<int> process_num;
};

struct SHMmsgs {
    // Hand Information
    uint8_t hand_mode[Hand_Num];
    uint8_t servo_on[Hand_Num];

    int16_t j_pos[Hand_Num][Hand_DOF];
    int16_t j_tar[Hand_Num][Hand_DOF];

    int16_t j_cur[Hand_Num][Hand_DOF];
    int16_t j_kin[Hand_Num][Kinesthetic_Sensor_Num][Kinesthetic_Sensor_DOF];
    int16_t j_tac[Hand_Num][Tactile_Sensor_Num];

    float tip_pos[Hand_Num][Finger_Num][Cartesian_DOF];
    float tip_quat[Hand_Num][Finger_Num][Quaternion_DOF];

    // Franka Arm Information
    double Arm_j_pos[Arm_Num][Arm_DOF];
    double Arm_j_tar[Arm_Num][Arm_DOF];   // single-point PtoP target (legacy)

    double Arm_j_vel[Arm_Num][Arm_DOF];
    double Arm_C_Pos[Arm_Num][16];
    double Arm_j_tq[Arm_Num][Arm_DOF];

    double Arm_Speed_Factor[Arm_Num];

    // Glove Information
    int16_t g_pos[Glove_Num];
    int16_t g_tac[Glove_Num][Glove_Tactile_Sensor_Num];

    std::atomic<int> process_num;
};

static_assert(sizeof(SHMmsgs) == 760, "SHMmsgs must stay compatible with key 0x3931");

extern const key_t shm_msg_key;
extern const key_t shm_rd_key;
extern const key_t shm_arm_traj_key;

void init_shm(int shm_key, int &shm_id, SHMmsgs **shm_ref);
void deleteSharedMemory(int shm_id, SHMmsgs *shm_ref);
void init_arm_traj_shm(int shm_key, int &shm_id, ArmTrajShm **shm_ref);
void deleteArmTrajSharedMemory(int shm_id, ArmTrajShm *shm_ref);
