#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <termios.h>

#include "include/uwb_sensor.h"


#define UWBINS_UWB_DEVNAME "/dev/ttyS2"
#define UWB_BAUDRATE B115200

#define err(format, ...)    fprintf(stderr, format, ##__VA_ARGS__)


static FAR void *uwb_sensor_entry(pthread_addr_t arg) {
    static int s_sensor_entry_result = PHYSICAL_SENSOR_ERR_CODE_OK;
    FAR physical_sensor_t *sensor = reinterpret_cast<FAR physical_sensor_t *>(arg);

    UwbSensorClass *instance = new UwbSensorClass(sensor);

    instance->run();

    delete (instance);

    return (void *)&s_sensor_entry_result;
}


FAR physical_sensor_t *UwbSensorCreate(pysical_event_handler_t handler) {
    return PhysicalSensorCreate(handler,
                                (void *)uwb_sensor_entry,
                                "uwb_sensor");
}


int UwbSensorOpen(FAR physical_sensor_t *sensor) {
    return PhysicalSensorOpen(sensor, NULL);
}


int UwbSensorStart(FAR physical_sensor_t *sensor) {
    return PhysicalSensorStart(sensor);
}


int UwbSensorStop(FAR physical_sensor_t *sensor) {
    return PhysicalSensorStop(sensor);
}


int UwbSensorClose(FAR physical_sensor_t *sensor) {
    return PhysicalSensorClose(sensor);
}


int UwbSensorDestroy(FAR physical_sensor_t *sensor) {
    return PhysicalSensorDestroy(sensor);
}


int UwbSensorClass::open_sensor() {
    m_fd = open(UWBINS_UWB_DEVNAME, O_RDONLY | O_NONBLOCK);
    if (m_fd <= 0) {
        return -1;
    }
    return 0;
}


int UwbSensorClass::close_sensor() {
    return close(m_fd);
}


int UwbSensorClass::start_sensor() {
    struct termios tty;
    tcgetattr(m_fd, &tty);
    tty.c_cflag |= CREAD;
    tcsetattr(m_fd, TCSANOW, &tty);
    return 0;
}


int UwbSensorClass::stop_sensor() {
    struct termios tty;
    tcgetattr(m_fd, &tty);
    tty.c_cflag &= CREAD;
    tcsetattr(m_fd, TCSANOW, &tty);
    return 0;
}


int UwbSensorClass::setup_sensor(FAR void *param) {
    struct termios tty;
    tcgetattr(m_fd, &tty);
    // cfsetospeed(&tty, UWB_BAUDRATE);
    cfsetispeed(&tty, UWB_BAUDRATE);
    tty.c_cflag &= ~PARENB;  // parity : none
    tty.c_cflag &= ~CSTOPB;  // stop bit : 1
    tty.c_cflag &= ~CSIZE;   // clear data size
    tty.c_cflag |= CS8;      // data size : 8bit
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Canonical mode : off
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // software flow control : off
    // tty.c_oflag &= ~OPOST;                          // output processing : off
    tcsetattr(m_fd, TCSANOW, &tty);
    return 0;
}


int UwbSensorClass::read_data() {
    MemMgrLite::MemHandle mh;
    FAR char *ptr;

    /* Get segment of memory handle. */
    if (ERR_OK != mh.allocSeg(S0_UWB_DATA_BUF_POOL, sizeof(type2bp_data_t) * UWB_NUM_ANCHOR)) {
        err("Fail to allocate segment of memory handle.\n");
        ASSERT(0);
    }
    ptr = reinterpret_cast<char *>(mh.getPa());

    /* Read imu data from driver. */
    // int ret = poll(fds, 1, 1000);
    // if (ret < 0) {
    //     if (errno != EINTR) {
    //         err("ERROR: poll failed. %d\n", errno);
    //     }
    // }
    // if (ret == 0) {
    //     err("ERROR: poll timeout.\n");
    // }
    // if (fds[0].revents & POLLIN) {
    //     ret = read(m_fd, ptr, sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO);
    //     if (ret != sizeof(cxd5602pwbimu_data_t) * IMU_NUM_FIFO) {
    //         err("ERROR: read failed. %d\n", errno);
    //     }
    // }
    for (auto i = 0; i < UWB_NUM_ANCHOR; i++) {
        type2bp_data_t *data;
        data->timestamp = 0;
        data->anchor_id = i;
        data->nlos = 0;
        data->distance  = i * 1.0f;
        data->azimuth   = i * 1.0f;
        data->elevation = i * 1.0f;
        memcpy(ptr + i * sizeof(type2bp_data_t), data, sizeof(type2bp_data_t));
    }

    this->notify_data(mh);

    mh.freeSeg();
    usleep(1 / UWB_SAMPLING_FREQUENCY * 1000 * 1000);
    return 0;
}


int UwbSensorClass::notify_data(MemMgrLite::MemHandle &mh) {
    uint32_t timestamp = get_timestamp();
    return m_handler(0, timestamp, mh);
};
