#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <sched.h>
#include <memory>
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
    std::unique_ptr<Packet[]> out(new Packet[IMU_NUM_FIFO]);
    std::unique_ptr<char[]> ptr(new char[sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO]);
    
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
        ret = read(m_fd, ptr.get(), sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO);
        if (ret != sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO) {
            err("ERROR: read failed. %d\n", errno);
        }
    }
    cxd5602pwbimu_data_t* imu_raw = reinterpret_cast<cxd5602pwbimu_data_t *>(ptr.get());
    uint64_t base_timestamp = ((uint64_t)imu_raw[3].timestamp * 1000000ULL) / 19200000ULL;
    for (auto i = IMU_NUM_FIFO - 1; i >= 0; i--) {
        out[i].type = PacketType::IMU;
        uint64_t tmp_timestamp = ((uint64_t)imu_raw[i].timestamp * 1000000ULL) / 19200000ULL;
        out[i].timestamp = get_us_timestamp() - (base_timestamp - tmp_timestamp);
        memcpy(&out[i].imu, &imu_raw[i], sizeof(cxd5602pwbimu_data_t));
    }
    m_handler(0, get_timestamp(), reinterpret_cast<char *>(out.get()));
    return 0;
}


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