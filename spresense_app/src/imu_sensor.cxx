#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <sched.h>
#include <nuttx/sensors/cxd5602pwbimu.h>

#include "include/imu_sensor.h"


#define UWBINS_IMU_DEVNAME "/dev/imu0"

#define err(format, ...)    fprintf(stderr, format, ##__VA_ARGS__)


static FAR void *imu_sensor_entry(pthread_addr_t arg) {
    static int s_sensor_entry_result = PHYSICAL_SENSOR_ERR_CODE_OK;
    FAR physical_sensor_t *sensor = reinterpret_cast<FAR physical_sensor_t *>(arg);

    ImuSensorClass *instance = new ImuSensorClass(sensor);

    instance->run();

    delete (instance);

    return (void *)&s_sensor_entry_result;
}


FAR physical_sensor_t *ImuSensorCreate(pysical_event_handler_t handler) {
    return PhysicalSensorCreate(handler,
                                (void *)imu_sensor_entry,
                                "imu_sensor");
}


int ImuSensorOpen(FAR physical_sensor_t *sensor) {
    return PhysicalSensorOpen(sensor, NULL);
}


int ImuSensorStart(FAR physical_sensor_t *sensor) {
    return PhysicalSensorStart(sensor);
}


int ImuSensorStop(FAR physical_sensor_t *sensor) {
    return PhysicalSensorStop(sensor);
}


int ImuSensorClose(FAR physical_sensor_t *sensor) {
    return PhysicalSensorClose(sensor);
}


int ImuSensorDestroy(FAR physical_sensor_t *sensor) {
    return PhysicalSensorDestroy(sensor);
}


int ImuSensorClass::open_sensor() {
    m_fd = open(UWBINS_IMU_DEVNAME, O_RDONLY);
    if (m_fd <= 0) {
        return -1;
    }
    fds[0].fd = m_fd;
    fds[0].events = POLLIN;
    return 0;
}


int ImuSensorClass::close_sensor() {
    return close(m_fd);
}


int ImuSensorClass::start_sensor() {
    int ret = ioctl(m_fd, SNIOC_ENABLE, 1);
    if (ret) {
        err("ERROR: SNIOC_ENABLE failed. %d\n", ret);
        return -1;
    }
    ret = read_away_50ms_data();
    if (ret) {
        err("ERROR: read_away_50ms_data failed.\n");
        return -1;
    }
    return ret;
}


int ImuSensorClass::stop_sensor() {
    return ioctl(m_fd, SNIOC_ENABLE, 0);
}


int ImuSensorClass::setup_sensor(FAR void *param) {
    int ret;
    cxd5602pwbimu_range_t range = {IMU_ACCEL_DRANGE, IMU_GYRO_DRANGE};

    ret = ioctl(m_fd, SNIOC_SSAMPRATE, IMU_SAMPLING_FREQUENCY);
    if (ret) {
        err("ERROR: Set sampling rate failed. %d\n", ret);
        return 1;
    }

    ret = ioctl(m_fd, SNIOC_SDRANGE, &range);
    if (ret) {
        err("ERROR: Set dynamic range failed. %d\n", ret);
        return 1;
    }

    ret = ioctl(m_fd, SNIOC_SFIFOTHRESH, IMU_NUM_FIFO);
    if (ret) {
        err("ERROR: Set FIFO threshold failed. %d\n", ret);
        return 1;
    }

    return 0;
}


int ImuSensorClass::read_data() {
    MemMgrLite::MemHandle mh;
    FAR char *ptr;

    /* Get segment of memory handle. */
    // if (ERR_OK != mh.allocSeg(S0_IMU_DATA_BUF_POOL, sizeof(cxd5602pwbimu_data_t))) {
    //     err("Fail to allocate segment of memory handle.\n");
    //     ASSERT(0);
    // }
    // ptr = reinterpret_cast<char *>(mh.getPa());
    ptr = reinterpret_cast<char *>(malloc(sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO));
    
    if (!ptr) {
        err("Fail to allocate memory.\n");
        return -1;
    }
    
    /* Read imu data from driver. */
    int ret = poll(fds, 1, 1000);
    if (ret < 0) {
        if (errno != EINTR) {
            err("ERROR: poll failed. %d\n", errno);
        }
    }
    if (ret == 0) {
        err("ERROR: poll timeout.\n");
    }
    if (fds[0].revents & POLLIN) {
        ret = read(m_fd, ptr, sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO);
        if (ret != sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO) {
            err("ERROR: read failed. %d\n", errno);
        }
    }
    char* out;
    out = reinterpret_cast<char *>(malloc(sizeof(imu_data_t) * IMU_NUM_FIFO));
    imu_data_t* out_ = reinterpret_cast<imu_data_t *>(out);
    cxd5602pwbimu_data_t *in = reinterpret_cast<cxd5602pwbimu_data_t *>(ptr);
    for (int i = 0; i < IMU_NUM_FIFO; i++) {
        out_[i].type = DATA_TYPE_IMU;
        out_[i].data.timestamp = in[i].timestamp;
        out_[i].data.temp = in[i].temp;
        out_[i].data.ax = in[i].ax;
        out_[i].data.ay = in[i].ay;
        out_[i].data.az = in[i].az;
        out_[i].data.gx = in[i].gx;
        out_[i].data.gy = in[i].gy;
        out_[i].data.gz = in[i].gz;
    }


    // this->notify_data(mh);
    m_handler(0, get_timestamp(), out);

    // mh.freeSeg();
    free(ptr);
    return 0;
}


// int ImuSensorClass::notify_data(MemMgrLite::MemHandle &mh) {
//     uint32_t timestamp = get_timestamp();
//     return m_handler(0, timestamp, mh);
// };


int ImuSensorClass::read_away_50ms_data() {
    int cnt = IMU_SAMPLING_FREQUENCY / 20 * 3; /* data size of 50ms */
    cnt = ((cnt + IMU_NUM_FIFO - 1) / IMU_NUM_FIFO) * IMU_NUM_FIFO;
    if (cnt == 0) cnt = IMU_NUM_FIFO;

    cxd5602pwbimu_data_t tmp_data[IMU_NUM_FIFO];
    while (cnt) {
        read(m_fd, tmp_data, sizeof(tmp_data[0]) * IMU_NUM_FIFO);
        cnt -= IMU_NUM_FIFO;
    }

    return 0;
}