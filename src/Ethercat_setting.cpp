#include "Ethercat_setting.h"

#include <sys/time.h>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <exception>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <pthread.h>
#include <unistd.h>

#include <ethercattype.h>
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#include "fk_solver.h"
#include "hdf5_logger.h"
#include "shm.h"

namespace
{
constexpr int64_t NSEC_PER_SEC = 1000000000;
constexpr int EC_TIMEOUTMON = 500;
constexpr int kHandNum = Hand_Num;

constexpr int Hand_Data_Read_Byte_Num = 256;
constexpr int Hand_Data_Write_Byte_Num = 64;

struct sched_param schedp;
char IOmap[4096];
struct timeval tv, t1, t2;
int deltat;
int tmax = 0;
int64 toff;
int64 gl_delta;
int DCdiff;
int os;
int dorun;
uint8 ob;
uint16 ob2;
uint8 *digout = nullptr;
int expectedWKC;
boolean needlf;
volatile int wkc;
uint8 currentgroup = 0;
bool bool_ethercat_loop = true;
volatile std::sig_atomic_t g_request_shutdown = 0;  // 시그널에서만 1로 설정 (async-signal-safe)
int pose_print_divider = 100;

typedef union
{
    uint8_t Byte[256];
    struct
    {
        uint16_t Hand_status1;
        uint16_t Hand_status2;

        int16_t Position[16];
        int16_t Current[16];
        int16_t Kinesthetic[12];
        int16_t Tactile[60];
        int16_t ADD_INFO[22];

    } OUT;
} Robot_Hand_Data;

typedef union
{
    uint8_t Byte[64];
    struct
    {
        uint16_t Hand_status1;
        uint16_t Hand_status2;

        int16_t JOINT_TARGET[16];
        int16_t ADD_INFO[14];
    } IN;
} Robot_Hand_Command;

SHMmsgs *shm_msgs = nullptr;
int shm_id = -1;

Robot_Hand_Data kistar_rx[kHandNum];
Robot_Hand_Command kistar_tx[kHandNum];
std::unique_ptr<Hdf5HandLogger> g_hdf5_logger;
int g_active_hand_num = kHandNum;

void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec;
    int64 nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta = (reftime - 50000) % cycletime;

    if (delta > (cycletime / 2))
    {
        delta -= cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }

    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

void print_fingertip_pose(const SHMmsgs *shm)
{
    if (shm == nullptr)
    {
        return;
    }

    static const char *kFingerName[Finger_Num] = {"THUMB", "INDEX", "MIDDLE", "RING"};
    std::printf("\n=== Fingertip Pose (mount frame) ===\n");
    for (int hand = 0; hand < kHandNum; ++hand)
    {
        std::printf("[Hand %d]\n", hand);
        for (int i = 0; i < Finger_Num; ++i)
        {
            std::printf("%s p=[%.4f %.4f %.4f] q(wxyz)=[%.4f %.4f %.4f %.4f]\n",
                        kFingerName[i],
                        shm->tip_pos[hand][i][0],
                        shm->tip_pos[hand][i][1],
                        shm->tip_pos[hand][i][2],
                        shm->tip_quat[hand][i][0],
                        shm->tip_quat[hand][i][1],
                        shm->tip_quat[hand][i][2],
                        shm->tip_quat[hand][i][3]);
        }
    }
    std::fflush(stdout);
}

void print_tx_status_and_target(int hand_idx, const Robot_Hand_Command &tx)
{
    std::printf("\r[H%d] status1=0x%04X status2=0x%04X joint_target=[",
                hand_idx,
                tx.IN.Hand_status1,
                tx.IN.Hand_status2);
    for (int i = 0; i < Hand_DOF; ++i)
    {
        std::printf("%d", tx.IN.JOINT_TARGET[i]);
        if (i < Hand_DOF - 1)
        {
            std::printf(",");
        }
    }
    std::printf("]    ");
    std::fflush(stdout);
}
} // namespace

pthread_t thread1;
pthread_t thread2;

void signal_callback_handler(int signum)
{
    (void)signum;
    g_request_shutdown = 1;  // 메인 루프에서만 정리 (시그널 핸들러에서는 플래그만 설정)
}

void *ecatthread(void *ptr)
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000;
    cycletime = *static_cast<int *>(ptr) * 1000;
    toff = 0;
    dorun = 0;

    init_shm(shm_msg_key, shm_id, &shm_msgs);
    /*try
    {
        g_hdf5_logger = std::make_unique<Hdf5HandLogger>();
        std::printf("\nHDF5 logger started: %s\n", g_hdf5_logger->file_path().c_str());
    }
    catch (const std::exception &e)
    {
        std::printf("\nHDF5 logger init failed: %s\n", e.what());
        g_hdf5_logger.reset();
    }*/

    ec_send_processdata();
    while (1)
    {
        add_timespec(&ts, cycletime + toff);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        if (dorun > 0)
        {
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            dorun++;
            for (int hand = 0; hand < g_active_hand_num; ++hand)
            {
                if (shm_msgs->servo_on[hand] == 1)
                {
                    kistar_tx[hand].IN.Hand_status1 = 0xFFFF;
                }
                else
                {
                    kistar_tx[hand].IN.Hand_status1 = 0x0000;
                }
                kistar_tx[hand].IN.Hand_status2 = shm_msgs->hand_mode[hand];

                for (int i = 0; i < Hand_DOF; i++)
                {
                    kistar_tx[hand].IN.JOINT_TARGET[i] = shm_msgs->j_tar[hand][i];
                }

                const int tx_offset = hand * Hand_Data_Write_Byte_Num;
                for (int i = 0; i < Hand_Data_Write_Byte_Num; i++)
                {
                    ec_slave[0].outputs[tx_offset + i] = kistar_tx[hand].Byte[i];
                }

                const int rx_offset = hand * Hand_Data_Read_Byte_Num;
                for (int i = 0; i < Hand_Data_Read_Byte_Num; i++)
                {
                    kistar_rx[hand].Byte[i] = ec_slave[0].inputs[rx_offset + i];
                }

                for (int i = 0; i < Hand_DOF; i++)
                {
                    shm_msgs->j_pos[hand][i] = kistar_rx[hand].OUT.Position[i];
                    shm_msgs->j_cur[hand][i] = kistar_rx[hand].OUT.Current[i];
                }
                for (int i = 0; i < Kinesthetic_Sensor_Num; i++)
                {
                    for (int j = 0; j < Kinesthetic_Sensor_DOF; j++)
                    {
                        shm_msgs->j_kin[hand][i][j] = kistar_rx[hand].OUT.Kinesthetic[3 * i + j];
                    }
                }
                for (int i = 0; i < Tactile_Sensor_Num; i++)
                {
                    shm_msgs->j_tac[hand][i] = kistar_rx[hand].OUT.Tactile[i];
                }

                compute_hand_fk_from_joint_pos(
                    shm_msgs->j_pos[hand], shm_msgs->tip_pos[hand], shm_msgs->tip_quat[hand]);

                /*if (hand == 0) {
                    print_tx_status_and_target(hand, kistar_tx[hand]);
                }*/
            }
            /*if (g_hdf5_logger)
            {
                g_hdf5_logger->append(*shm_msgs);
            }
            if ((dorun % pose_print_divider) == 0)
            {
                print_fingertip_pose(shm_msgs);
            }*/

            if (digout)
            {
                *digout = static_cast<uint8>((dorun / 16) & 0xff);
            }

            if (ec_slave[0].hasdc)
            {
                ec_sync(ec_DCtime, cycletime, &toff);
            }
            ec_send_processdata();
        }
    }
}

void *ecatcheck(void *ptr)
{
    (void)ptr;

    while (1)
    {
        if ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)
        {
            if (needlf)
            {
                needlf = FALSE;
                std::printf("\n");
            }

            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();

            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) &&
                    (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        std::printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        std::printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            std::printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            std::printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }

                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            std::printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        std::printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }

            if (!ec_group[currentgroup].docheckstate)
            {
                std::printf("OK : all slaves resumed OPERATIONAL.\n");
            }
        }
        osal_usleep(10000);
    }
}

void ethercat_run(const char *if_name)
{
    if (if_name == nullptr || if_name[0] == '\0')
    {
        if_name = "enp5s0f0";
    }

    if (!ec_init(if_name))
    {
        std::printf("No socket connection on %s\nExecute as root\n", if_name);
        return;
    }

    std::printf("ec_init on %s succeeded.\n", if_name);

    if (ec_config(FALSE, &IOmap) <= 0)
    {
        std::printf("No slaves found!\n");
        ec_close();
        return;
    }

    ec_configdc();
    std::printf("%d slaves found and configured.\n", ec_slavecount);

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    ec_readstate();

    for (int cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        std::printf(
            "Slave:%d Name:%s Output size:%3d bits Input size:%3d bits State:%2d delay:%d hasdc:%d\r\n",
            cnt,
            ec_slave[cnt].name,
            ec_slave[cnt].Obits,
            ec_slave[cnt].Ibits,
            ec_slave[cnt].state,
            static_cast<int>(ec_slave[cnt].pdelay),
            ec_slave[cnt].hasdc);

        if (!digout && ((ec_slave[cnt].eep_id == 0x0af83052) || (ec_slave[cnt].eep_id == 0x07d83052)))
        {
            digout = ec_slave[cnt].outputs;
        }
    }

    const int max_hands_by_output =
        (Hand_Data_Write_Byte_Num > 0) ? (ec_slave[0].Obytes / Hand_Data_Write_Byte_Num) : 0;
    const int max_hands_by_input =
        (Hand_Data_Read_Byte_Num > 0) ? (ec_slave[0].Ibytes / Hand_Data_Read_Byte_Num) : 0;
    const int max_hands_by_pdo = std::min(max_hands_by_output, max_hands_by_input);
    g_active_hand_num = std::max(1, std::min(kHandNum, max_hands_by_pdo));
    if (g_active_hand_num != kHandNum)
    {
        std::printf("WARNING : Hand_Num=%d but PDO layout supports %d hand(s). Running with %d.\n",
                    kHandNum, max_hands_by_pdo, g_active_hand_num);
    }

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    if (expectedWKC != 3 * g_active_hand_num)
    {
        std::printf("WARNING : Calculated Workcounter insufficient!\n");
    }
    else
    {
        std::printf("Calculated workcounter %d\n", expectedWKC);
    }

    std::printf("Request operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);
    dorun = 1;

    ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);
    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        std::printf("Operational state reached for all slaves.\n");
        std::printf("Operation Start (Ctrl+C to stop)\n");
        std::fflush(stdout);
        osal_usleep(20000);

        while (!g_request_shutdown)
        {
        }

        if (g_request_shutdown)
            std::printf("\nExiting: interrupt received (Ctrl+C).\n");
    }
    else
    {
        std::printf("Not all slaves reached operational state.\n");
        ec_readstate();
        for (int i = 1; i <= ec_slavecount; i++)
        {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL)
            {
                std::printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i,
                            ec_slave[i].state,
                            ec_slave[i].ALstatuscode,
                            ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    }

    g_hdf5_logger.reset();
    deleteSharedMemory(shm_id, shm_msgs);
    std::printf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    ec_close();
}
