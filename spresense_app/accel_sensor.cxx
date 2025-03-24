#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/sensors/cxd5602pwbimu.h>

#include "accel_sensor.h"


#define STEP_COUNTER_ACCEL_DEVNAME "/dev/imu0"

#define err(format, ...)    fprintf(stderr, format, ##__VA_ARGS__)


static FAR void *accel_sensor_entry(pthread_addr_t arg) {
    static int s_sensor_entry_result = PHYSICAL_SENSOR_ERR_CODE_OK;
    FAR physical_sensor_t *sensor = reinterpret_cast<FAR physical_sensor_t *>(arg);

    AccelSensorClass *instance = new AccelSensorClass(sensor);

    instance->run();

    delete (instance);

    return (void *)&s_sensor_entry_result;
}


FAR physical_sensor_t *AccelSensorCreate(pysical_event_handler_t handler) {
    return PhysicalSensorCreate(handler,
                                (void *)accel_sensor_entry,
                                "accel_sensor");
}


int AccelSensorOpen(FAR physical_sensor_t *sensor) {
    return PhysicalSensorOpen(sensor, NULL);
}


int AccelSensorStart(FAR physical_sensor_t *sensor) {
    return PhysicalSensorStart(sensor);
}


int AccelSensorStop(FAR physical_sensor_t *sensor) {
    return PhysicalSensorStop(sensor);
}


int AccelSensorClose(FAR physical_sensor_t *sensor) {
    return PhysicalSensorClose(sensor);
}


int AccelSensorDestroy(FAR physical_sensor_t *sensor) {
    return PhysicalSensorDestroy(sensor);
}


int AccelSensorClass::open_sensor() {
    m_fd = open(STEP_COUNTER_ACCEL_DEVNAME, O_RDONLY);
    if (m_fd <= 0) {
        return -1;
    }
    fds[0].fd = m_fd;
    fds[0].events = POLLIN;
    return 0;
}


int AccelSensorClass::close_sensor() {
    return close(m_fd);
}


int AccelSensorClass::start_sensor() {
    return ioctl(m_fd, SNIOC_ENABLE, 1);
}


int AccelSensorClass::stop_sensor() {
    return ioctl(m_fd, SNIOC_ENABLE, 0);
}


int AccelSensorClass::setup_sensor(FAR void *param) {
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


int AccelSensorClass::read_data() {
    MemMgrLite::MemHandle mh_src;
    FAR char *p_src;

    /* Get segment of memory handle. */
    if (ERR_OK != mh_src.allocSeg(S0_IMU_DATA_BUF_POOL, sizeof(cxd5602pwbimu_data_t))) {
        err("Fail to allocate segment of memory handle.\n");
        ASSERT(0);
    }
    p_src = reinterpret_cast<char *>(mh_src.getPa());

    /* Read accelerometer data from driver. */
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
        ret = read(m_fd, p_src, sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO);
        if (ret != sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO) {
            err("ERROR: read failed. %d\n", errno);
        }
    }

    /* Notify accelerometer data to sensor manager. */
    this->notify_data(mh_src);

    mh_src.freeSeg();
    return 0;
}


int AccelSensorClass::notify_data(MemMgrLite::MemHandle &mh_dst) {
    uint32_t timestamp = get_timestamp();
    return m_handler(0, timestamp, mh_dst);
};
