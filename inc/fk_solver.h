#pragma once

#include "shm.h"

void compute_hand_fk_from_joint_pos(const int16_t joint_pos[Hand_DOF], float tip_pos[4][3], float tip_quat_wxyz[4][4]);
