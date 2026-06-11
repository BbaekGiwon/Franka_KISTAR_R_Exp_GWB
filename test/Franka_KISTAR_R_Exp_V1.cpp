#include <csignal>
#include <cstdio>
#include <pthread.h>

#include "Ethercat_setting.h"
#include "franka_right_arm.h"
#include "shm_print.h"

int main(int argc, char *argv[])
{
    
    std::printf("Franka right arm and KISTAR hand\n");

    std::signal(SIGINT, signal_callback_handler);

    const char *if_name = (argc > 1) ? argv[1] : "enp1s0f0";
    std::printf("Using network interface: %s\n", if_name);

 
    int ctime = 1000;
    pthread_create(&thread1, nullptr, &ecatthread, static_cast<void *>(&ctime));
    pthread_create(&thread2, nullptr, &ecatcheck, nullptr);
    
    pthread_t franka_thread;
    pthread_t shm_print_t;
    FrankaRightArmConfig cfg{"172.16.0.1", 0.1, 1000, 100000};
    pthread_create(&franka_thread, nullptr, &franka_control_thread_R, &cfg);

    int shm_print_interval_ms = 100;
    pthread_create(&shm_print_t, nullptr, &shm_print_thread, &shm_print_interval_ms);

    ethercat_run(if_name);
    return 0;
}
