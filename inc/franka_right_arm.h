#pragma once

#include <pthread.h>

struct SHMmsgs;

struct FrankaRightArmConfig {
  const char* robot_ip;
  double speed_factor;
  int control_period_us;
  int print_interval_us;
};

void* franka_control_thread_R(void* ptr);
void* data_print_thread(void* ptr);
