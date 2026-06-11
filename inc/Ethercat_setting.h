#pragma once

#include <pthread.h>

extern pthread_t thread1;
extern pthread_t thread2;

void signal_callback_handler(int signum);
void *ecatthread(void *ptr);
void *ecatcheck(void *ptr);
void ethercat_run(const char *if_name);
