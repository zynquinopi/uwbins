#include <nuttx/config.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <asmp/mpshm.h>
#include <arpa/inet.h>
#include <sys/socket.h>

//  #ifndef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
//  #  include <arch/board/board.h>
//  #endif

#include "include/imu_sensor.h"
#include "include/uwb_sensor.h"
//  #include "gnss_sensor.h"
#include "sensing/sensor_api.h"
#include "sensing/logical_sensor/step_counter.h"
#include "memutils/message/Message.h"
#include "memutils/memory_manager/MemHandle.h"
#include "include/mem_conf/pool_layout.h"
#include "include/mem_conf/msgq_pool.h"
#include "include/mem_conf/fixed_fence.h"
#include <include/MadgwickAHRS.h>
#include "include/types.h"


#define UDP_IP "192.168.11.11"
#define UDP_PORT 50000

#define CM_2_M 0.01f
#define SENSOR_SECTION SECTION_NO0

using namespace MemMgrLite;

#define EXIT_REQUEST_KEY 0x20 /* space key */

#define message(format, ...) printf(format, ##__VA_ARGS__)
#define err(format, ...) fprintf(stderr, format, ##__VA_ARGS__)

//  #ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
//  static FAR GnssSensor *sp_gnss_sensor = NULL;
//  #endif
// static FAR StepCounterClass *sp_step_counter_ins = NULL;
static mpshm_t s_shm;
static Madgwick madgwick_filter;
static float last_time = 0.0f;

static int udp_sock;
static struct sockaddr_in udp_addr;


static bool sensor_init_libraries(void) {
    uint32_t addr = SHM_SRAM_ADDR;

    /* Initialize shared memory.*/
    int ret = mpshm_init(&s_shm, 1, SHM_SRAM_SIZE);
    if (ret < 0) {
        err("Error: mpshm_init() failure. %d\n", ret);
        return false;
    }
    ret = mpshm_remap(&s_shm, (void *)addr);
    if (ret < 0) {
        err("Error: mpshm_remap() failure. %d\n", ret);
        return false;
    }

    /* Initalize MessageLib. */
    err_t err = MsgLib::initFirst(NUM_MSGQ_POOLS, MSGQ_TOP_DRM);
    if (err != ERR_OK) {
        err("Error: MsgLib::initFirst() failure. 0x%x\n", err);
        return false;
    }
    err = MsgLib::initPerCpu();
    if (err != ERR_OK) {
        err("Error: MsgLib::initPerCpu() failure. 0x%x\n", err);
        return false;
    }

    void *mml_data_area = translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
    err = Manager::initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
    if (err != ERR_OK) {
        err("Error: Manager::initFirst() failure. 0x%x\n", err);
        return false;
    }
    err = Manager::initPerCpu(mml_data_area, static_pools, pool_num, layout_no);
    if (err != ERR_OK) {
        err("Error: Manager::initPerCpu() failure. 0x%x\n", err);
        return false;
    }

    /* Create static memory pool.*/
    const uint8_t sec_no = SECTION_NO0;
    const NumLayout layout_no = 0;
    void *work_va = translatePoolAddrToVa(S0_MEMMGR_WORK_AREA_ADDR);
    const PoolSectionAttr *ptr = &MemoryPoolLayouts[sec_no][layout_no][0];
    err = Manager::createStaticPools(sec_no,
                                     layout_no,
                                     work_va,
                                     S0_MEMMGR_WORK_AREA_SIZE,
                                     ptr);
    if (err != ERR_OK) {
        err("Error: Manager::createStaticPools() failure. %x\n", err);
        return false;
    }

    return true;
}


static bool sensor_finalize_libraries(void) {
    MsgLib::finalize();

    MemMgrLite::Manager::destroyStaticPools(SENSOR_SECTION);

    MemMgrLite::Manager::finalize();

    int ret = mpshm_detach(&s_shm);
    if (ret < 0) {
        err("Error: mpshm_detach() failure. %d\n", ret);
        return false;
    }

    ret = mpshm_destroy(&s_shm);
    if (ret < 0) {
        err("Error: mpshm_destroy() failure. %d\n", ret);
        return false;
    }

    return true;
}


static int imu_read_callback(uint32_t ev_type,
                               uint32_t timestamp,
                               char* data) {
    // sensor_command_data_mh_t packet;
    // packet.header.size = 0;
    // packet.header.code = SendData;
    // packet.self = accelID;
    // packet.time = timestamp;
    // packet.fs = IMU_SAMPLING_FREQUENCY;
    // packet.size = IMU_NUM_FIFO;
    // packet.mh = mh;
    // SS_SendSensorDataMH(&packet);


    // FAR cxd5602pwbimu_data_t *data = NULL;
    // data = static_cast<cxd5602pwbimu_data_t *>(mh.getVa());
    FAR imu_data_t* imu_data = reinterpret_cast<FAR imu_data_t*>(data);
    for (int i = 0; i < IMU_NUM_FIFO; i++) {
        // printf("[ imu]ts=%f, t=%2.2f, ax=%6.2f, ay=%6.2f, az=%6.2f, gx=%8.4f, gy=%8.4f, gz=%8.4f\n",
        //        imu_data[i].data.timestamp / 19200000.0f, imu_data[i].data.temp,
        //        imu_data[i].data.ax, imu_data[i].data.ay, imu_data[i].data.az,
        //        imu_data[i].data.gx, imu_data[i].data.gy, imu_data[i].data.gz);
        printf("imu");
        sendto(udp_sock, &imu_data[i], sizeof(imu_data_t), 0, (struct sockaddr*)&udp_addr, sizeof(udp_addr));
        // last_time = imu_data[i].data.timestamp / 19200000.0f;
        // madgwick_filter.updateIMU(imu_data[i].data.gx * 180 / 3.14, imu_data[i].data.gy * 180 / 3.14, imu_data[i].data.gz * 180 / 3.14,
        //                           imu_data[i].data.ax / 9.81, imu_data[i].data.ay / 9.81, imu_data[i].data.az / 9.81);
        // float q[4];
        // madgwick_filter.getQuaternion(q);
        // printf("[pose]ts=%f, x=0.0,y=0.0, z=0.0, qx=%f, qy=%f, qz=%f, qw=%f\n",
        //        imu_data[i].data.timestamp / 19200000.0f, q[0], q[1], q[2], q[3]);

        // pose_data_t pose;
        // pose.type = DATA_TYPE_POSE;
        // pose.timestamp = imu_data[i].data.timestamp / 19200000.0f;
        // pose.x = 0.0f; pose.y = 0.0f; pose.z = 0.0f;
        // pose.qx = q[0]; pose.qy = q[1]; pose.qz = q[2]; pose.qw = q[3];
        // sendto(udp_sock, &pose, sizeof(pose_data_t), 0, (struct sockaddr*)&udp_addr, sizeof(udp_addr)); 
    }

    return 0;
}


static int uwb_read_callback(uint32_t ev_type,
                             uint32_t timestamp,
                             char* data) {
    // sensor_command_data_mh_t packet;
    // packet.header.size = 0;
    // packet.header.code = SendData;
    // packet.self = accelID; //TODO
    // packet.time = timestamp;
    // packet.fs = UWB_SAMPLING_FREQUENCY;
    // packet.size = UWB_NUM_ANCHOR;
    // packet.mh = mh;
    // SS_SendSensorDataMH(&packet);

    // FAR type2bp_data_t *data = NULL;
    // data = static_cast<type2bp_data_t *>(mh.getVa());
    FAR type2bp_data_t* uwb_data = reinterpret_cast<FAR type2bp_data_t*>(data);
    // sendto(udp_sock, data, sizeof(type2bp_data_t) * UWB_NUM_ANCHOR, 0, (struct sockaddr*)&udp_addr, sizeof(udp_addr));
    // for (int i = 0; i < UWB_NUM_ANCHOR; i++) {
    //     if (uwb_data[i].anchor_id < 0) {
    //         continue;
    //     }
    //     printf("[ uwb]ts=%lu, i=%hhd, n=%hhu, d=%6.2f, a=%6.2f, e=%6.2f\n",
    //            uwb_data[i].timestamp, uwb_data[i].anchor_id,
    //            uwb_data[i].nlos, uwb_data[i].distance * CM_2_M,
    //            uwb_data[i].azimuth, uwb_data[i].elevation);
    // }
    return 0;
}


//  #ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
//  static int gnss_read_callback(uint32_t context, FAR GnssSampleData *pos)
//  {
//    MemMgrLite::MemHandle mh;
//    if (ERR_OK != mh.allocSeg(GNSS_DATA_BUF_POOL, sizeof(GnssSampleData)))
//      {
//        ASSERT(0);
//      }

//    FAR GnssSampleData *data_top = NULL;
//    data_top = static_cast<GnssSampleData*>(mh.getVa());
//    *data_top = *pos;

//    sensor_command_data_mh_t packet;
//    packet.header.size = 0;
//    packet.header.code = SendData;
//    packet.self        = gnssID;
//    packet.time        = 0;
//    packet.fs          = 0;
//    packet.size        = 1;
//    packet.mh          = mh;

//    SS_SendSensorDataMH(&packet);

//    return 0;
//  }
//  #endif


//  #ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
//  static int gnss_sensor_entry(int argc,  const char* argv[])
//  {
//    GnssSensorStartSensing(sp_gnss_sensor);
//    GnssSensorDestroy(sp_gnss_sensor);
//    sp_gnss_sensor = NULL;
//    return 0;
//  }
//  #endif


// bool step_counter_receive_data(sensor_command_data_mh_t &data) {
//     StepCounterWrite(sp_step_counter_ins, &data);
//     return true;
// }


// bool step_counter_recieve_result(sensor_command_data_mh_t &data) {
//     bool ret = true;
//     FAR SensorResultStepCounter *result =
//         reinterpret_cast<SensorResultStepCounter *>(data.mh.getVa());
//     if (SensorOK == result->exec_result) {
// #ifndef CONFIG_CPUFREQ_RELEASE_LOCK
//         float tempo = 0;

//         switch (result->steps.movement_type) {
//         case STEP_COUNTER_MOVEMENT_TYPE_WALK:
//         case STEP_COUNTER_MOVEMENT_TYPE_RUN:
//             /* Tempo values are valid for walking and running only. */
//             tempo = result->steps.tempo;
//             break;
//         default:
//             break;
//         }

//         message("%11.5f,%11.2f,%11.5f,%11.2f,%11lld,",
//                 tempo,
//                 result->steps.stride,
//                 result->steps.speed,
//                 result->steps.distance,
//                 result->steps.time_stamp);
// #endif
//         message("   %8ld,", result->steps.step);
//         switch (result->steps.movement_type) {
//         case STEP_COUNTER_MOVEMENT_TYPE_STILL:
//             message("   stopping\r");
//             break;
//         case STEP_COUNTER_MOVEMENT_TYPE_WALK:
//             message("   walking \r");
//             break;
//         case STEP_COUNTER_MOVEMENT_TYPE_RUN:
//             message("   running \r");
//             break;
//         default:
//             message("   UNKNOWN \r");
//             break;
//         }
//     }
//     else {
//         ret = false;
//     }

//     return ret;
// }


static void sensor_manager_api_response(unsigned int code,
                                        unsigned int ercd,
                                        unsigned int self) {
    if (ercd != SS_ECODE_OK) {
        err("Error: get api response. code %d, ercd %d, self %d\n",
            code, ercd, self);
    }
    return;
}


extern "C" int uwbins_main(int argc, FAR char *argv[]) {

    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        err("Error: socket creation failed");
        return EXIT_FAILURE;
    }

    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_port = htons(UDP_PORT);
    inet_pton(AF_INET, UDP_IP, &udp_addr.sin_addr);

    sensor_init_libraries();

    madgwick_filter.begin(IMU_SAMPLING_FREQUENCY);

    // if (!SS_ActivateSensorSubSystem(MSGQ_SEN_MGR, sensor_manager_api_response)) {
    //     err("Error: SS_ActivateSensorSubSystem() failure.\n");
    //     return EXIT_FAILURE;
    // }

    FAR physical_sensor_t *imu = ImuSensorCreate(imu_read_callback);
    if (imu == NULL) {
        err("Error: ImuSensorCreate() failure.\n");
        return EXIT_FAILURE;
    }

    // FAR physical_sensor_t *uwb = UwbSensorCreate(uwb_read_callback);
    // if (uwb == NULL) {
    //     err("Error: UwbSensorCreate() failure.\n");
    //     return EXIT_FAILURE;
    // }

    /* Setup GNSS sensor. */
    //  #ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
    //    ret = GnssSensorCreate(&sp_gnss_sensor);
    //    if (ret < 0)
    //      {
    //        err("Error: GnssSensorCreate() failure.\n");
    //        return EXIT_FAILURE;
    //      }

    //    uint32_t context = 0;
    //    ret = GnssSensorRegisterHandler(sp_gnss_sensor,
    //                                    gnss_read_callback,
    //                                    context);
    //    if (ret < 0)
    //      {
    //        err("Error: GnssSensorRegisterHandler() failure.\n");
    //        return EXIT_FAILURE;
    //      }
    //  #endif

    /* Resister sensor clients. */
    // sensor_command_register_t reg;

    // reg.header.size = 0;
    // reg.header.code = ResisterClient;
    // reg.self = accelID;
    // reg.subscriptions = 0;
    // reg.callback = NULL;
    // reg.callback_mh = NULL;
    // SS_SendSensorResister(&reg);

    // reg.header.size = 0;
    // reg.header.code = ResisterClient;
    // reg.self = gnssID;
    // reg.subscriptions = 0;
    // reg.callback = NULL;
    // reg.callback_mh = NULL;
    // SS_SendSensorResister(&reg);

    // reg.header.size = 0;
    // reg.header.code = ResisterClient;
    // reg.self = stepcounterID;
    // reg.subscriptions = (0x01 << accelID) | (0x01 << gnssID);
    // reg.callback = NULL;
    // reg.callback_mh = &step_counter_receive_data;
    // SS_SendSensorResister(&reg);

    // reg.header.size = 0;
    // reg.header.code = ResisterClient;
    // reg.self = app0ID;
    // reg.subscriptions = (0x01 << stepcounterID);
    // reg.callback = NULL;
    // reg.callback_mh = &step_counter_recieve_result;
    // SS_SendSensorResister(&reg);

    /* Setup logical sensor. */
    // sp_step_counter_ins = StepCounterCreate(S0_SENSOR_DSP_CMD_BUF_POOL);
    // if (NULL == sp_step_counter_ins) {
    //     err("Error: StepCounterCreate() failure.\n");
    //     return EXIT_FAILURE;
    // }

    // ret = StepCounterOpen(sp_step_counter_ins);
    // if (ret != SS_ECODE_OK) {
    //     err("Error: StepCounterOpen() failure. error = %d\n", ret);
    //     return EXIT_FAILURE;
    // }

    /* Setup Stride setting.
     * The range of configurable stride lenght is 1 - 249[cm].
     * For the mode, set STEP_COUNTER_MODE_FIXED_LENGTH fixed.
     */
    // StepCounterSetting set;
    // set.walking.step_length = CONFIG_EXAMPLES_STEP_COUNTER_WALKING_STRIDE;
    // set.walking.step_mode = STEP_COUNTER_MODE_FIXED_LENGTH;
    // set.running.step_length = CONFIG_EXAMPLES_STEP_COUNTER_RUNNING_STRIDE;
    // set.running.step_mode = STEP_COUNTER_MODE_FIXED_LENGTH;
    // ret = StepCounterSet(sp_step_counter_ins, &set);
    // if (ret != SS_ECODE_OK) {
    //     err("Error: StepCounterSet() failure. error = %d\n", ret);
    //     return EXIT_FAILURE;
    // }

// #if defined(CONFIG_CPUFREQ_RELEASE_LOCK) && !defined(CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS)
//     /* After here,
//      * because this example program doesn't access to flash
//      * and doesn't use TCXO,
//      * it turn the flash and TCXO power off to reduce power consumption.
//      */
//     board_xtal_power_control(false);
//     board_flash_power_control(false);
// #endif

    message("start sensoring...\n");

    // /* Start physical sensor process. */
    if (ImuSensorOpen(imu) < 0) {
        err("Error: ImuSensorOpen() failure.\n");
        return EXIT_FAILURE;
    }
    // if (UwbSensorOpen(uwb) < 0) {
    //     err("Error: UwbSensorOpen() failure.\n");
    //     return EXIT_FAILURE;
    // }
    if (ImuSensorStart(imu) < 0) {
        err("Error: ImuSensorStart() failure.\n");
        return EXIT_FAILURE;
    }
    // if (UwbSensorStart(uwb) < 0) {
    //     err("Error: UwbSensorStart() failure.\n");
    //     return EXIT_FAILURE;
    // }

    //  #ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
    //    task_create("gnss_sensoring",
    //                110,
    //                2048,
    //                (main_t)gnss_sensor_entry,
    //                (FAR char * const *)NULL);
    //  #endif

    while (1) {
        /* Wait exit request. */
        if (fgetc(stdin) == EXIT_REQUEST_KEY) {
            break;
        }
        usleep(200 * 1000);
        break;
    }

    /* Stop physical sensor. */
    if (ImuSensorStop(imu) < 0) {
        err("Error: ImuSensorStop() failure.\n");
        return EXIT_FAILURE;
    }
    if (ImuSensorClose(imu) < 0) {
        err("Error: ImuSensorClose() failure.\n");
        return EXIT_FAILURE;
    }

    // if (UwbSensorStop(uwb) < 0) {
    //     err("Error: UwbSensorStop() failure.\n");
    //     return EXIT_FAILURE;
    // }
    // if (UwbSensorClose(uwb) < 0) {
    //     err("Error: UwbSensorClose() failure.\n");
    //     return EXIT_FAILURE;
    // }

#ifdef CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS
    GnssSensorStopSensing(sp_gnss_sensor);
#endif

    /* Release sensor clients. */
    // sensor_command_release_t rel;

    // rel.header.size = 0;
    // rel.header.code = ReleaseClient;
    // rel.self = app0ID;
    // SS_SendSensorRelease(&rel);

    // rel.header.size = 0;
    // rel.header.code = ReleaseClient;
    // rel.self = stepcounterID;
    // SS_SendSensorRelease(&rel);

    // rel.header.size = 0;
    // rel.header.code = ReleaseClient;
    // rel.self = accelID;
    // SS_SendSensorRelease(&rel);

    // rel.header.size = 0;
    // rel.header.code = ReleaseClient;
    // rel.self = gnssID;
    // SS_SendSensorRelease(&rel);

#if defined(CONFIG_CPUFREQ_RELEASE_LOCK) && !defined(CONFIG_EXAMPLES_STEP_COUNTER_ENABLE_GNSS)
    /* Turn flash and TCXO power on */
    board_xtal_power_control(true);
    board_flash_power_control(true);
#endif

    // /* Close logical sensor. */
    // if (SS_ECODE_OK != StepCounterClose(sp_step_counter_ins)) {
    //     err("Error: StepCounterOpen() failure.\n");
    //     return EXIT_FAILURE;
    // }

    // if (!SS_DeactivateSensorSubSystem()) {
    //     err("Error: SS_DeactivateSensorSubSystem() failure.\n");
    //     return EXIT_FAILURE;
    // }

    ImuSensorDestroy(imu);
    // UwbSensorDestroy(uwb);
    sensor_finalize_libraries();
    close(udp_sock);
    return EXIT_SUCCESS;
}