#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <termios.h>
#include <memory>

#include "include/uwb_sensor.h"


#define UWBINS_UWB_DEVNAME "/dev/ttyS2"
#define UWB_BAUDRATE B2000000
#define CM_2_M 0.01f

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
    if (m_fd < 0) {
        err("ERROR: open %s failed. errno=%d\n", UWBINS_UWB_DEVNAME, errno);
        return -1;
    }
    fds[0].fd = m_fd;
    fds[0].events = POLLIN;
    return 0;
}


int UwbSensorClass::close_sensor() {
    if (m_fd >= 0) {
        int ret = close(m_fd);
        m_fd = -1;
    }
    fds[0].fd = -1;
    fds[0].events = 0;
    return 0;
}


int UwbSensorClass::start_sensor() {
    struct termios tty;
    if (tcgetattr(m_fd, &tty) != 0) {
        err("ERROR: tcgetattr failed. errno=%d\n", errno);
        return -1;
    }

    tty.c_cflag |= CREAD;
    
    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        err("ERROR: tcsetattr failed. errno=%d\n", errno);
        return -1;
    }
    return 0;
}


int UwbSensorClass::stop_sensor() {
    struct termios tty;
    if (tcgetattr(m_fd, &tty) != 0) {
        err("ERROR: tcgetattr failed. errno=%d\n", errno);
        return -1;
    }

    tty.c_cflag &= ~CREAD;

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        err("ERROR: tcsetattr failed. errno=%d\n", errno);
        return -1;
    }
    tcflush(m_fd, TCIFLUSH);
    return 0;
}


int UwbSensorClass::setup_sensor(FAR void *param) {
    struct termios tty;
    tcgetattr(m_fd, &tty);
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
    std::unique_ptr<char[]> buffer(new char[512]);
    size_t buffer_index = 0;

    tcflush(m_fd, TCIFLUSH);

    /* Read uwb data from driver. */
    while (1) {
        int ret = poll(fds, 1, 1000);
        if (ret < 0) {
            if (errno != EINTR) {
                err("ERROR: poll failed. errno=%d\n", errno);
                break;
            }
            continue;
        }
        if (ret == 0) {
            err("ERROR: poll timeout.\n");
            break;
        }
        if (fds[0].revents & POLLIN) {
            usleep(3 * 1000);
            int num_bufers;
            ioctl(m_fd, FIONREAD, &num_bufers);
            // uint8_t byte;
            ssize_t num_bytes = read(m_fd, buffer.get(), num_bufers);
            if (num_bytes > 0) {
                // buffer[buffer_index++] = byte;
                // if (byte == '\n') {
                    buffer[num_bufers] = '\0';
                    std::unique_ptr<Type2bp[]> data_array(new Type2bp[UWB_NUM_ANCHOR]);
                    parse_uwb_data(buffer.get(), data_array.get());
                    std::unique_ptr<Packet[]> out(new Packet[UWB_NUM_ANCHOR]);
                    for (int i = 0; i < UWB_NUM_ANCHOR; i++) {
                        out[i].type = PacketType::UWB;
                        out[i].timestamp = get_us_timestamp();
                        memcpy(&out[i].uwb, &data_array[i], sizeof(Type2bp));
                    }
                    m_handler(0, 0, reinterpret_cast<char *>(out.get()));
                    return 0;
                if (buffer_index >= sizeof(buffer)) {
                    err("Frame length exceeded maximum allowed size.\n");
                    buffer_index = 0;
                }
            } else if (num_bytes == 0) {
                continue;
            } else {
                if (errno != EAGAIN) {
                    err("Error reading from UART: %s\n", strerror(errno));
                    break;
                }
            }
        }
    }
    return -1;
}


void UwbSensorClass::parse_uwb_data(const char* src_data, Type2bp* data_array) {
    for (auto i = 0; i < UWB_NUM_ANCHOR; i++) {
        data_array[i].anchor_id = -1;
        data_array[i].nlos      = 0u;
        data_array[i].distance  = 0.0f;
        data_array[i].azimuth   = 0.0f;
        data_array[i].elevation = 0.0f;
    }

    while ((src_data = strstr(src_data, "i")) != NULL) {
        Type2bp data;
        if (sscanf(src_data, "i%hhd,n%hhu,d%f,a%f,e%f,",
                   &data.anchor_id, &data.nlos, &data.distance, &data.azimuth, &data.elevation) == 5){
            data.distance *= CM_2_M;
            if (data.anchor_id >= 0 && data.anchor_id < UWB_NUM_ANCHOR) {
                data_array[data.anchor_id] = data;
            }
        }
        src_data += 1;
    }
}


// int UwbSensorClass::notify_data(MemMgrLite::MemHandle &mh) {
//     uint32_t timestamp = get_timestamp();
//     return m_handler(0, timestamp, mh);
// };
