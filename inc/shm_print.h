#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** SHM 값을 터미널에 주기적으로 출력하는 스레드 진입점.
 *  ptr: nullptr 또는 (int*) 출력 간격(ms). 0 이하면 100ms 사용. */
void* shm_print_thread(void* ptr);

#ifdef __cplusplus
}
#endif
