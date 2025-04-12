#include <nuttx/config.h>
#include <stdio.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <memory>
#include <iostream>

#include "include/imu_sensor.h"
#include "include/uwb_sensor.h"
#include "sensing/sensor_ecode.h"
#include "include/types.h"
#include "include/MadgwickAHRS.h"
#include "include/file_logger.h"


#define PIN_LED0 PIN_I2S1_BCK
#define PIN_LED1 PIN_I2S1_LRCK
#define PIN_LED2 PIN_I2S1_DATA_IN
#define PIN_LED3 PIN_I2S1_DATA_OUT
#define PIN_BUTTON PIN_SPI3_CS1_X

#define UDP_IP "192.168.11.11"
#define UDP_PORT 50000

#define EXIT_REQUEST_KEY 0x20 /* space key */

#define message(format, ...) printf(format, ##__VA_ARGS__)
#define err(format, ...) fprintf(stderr, format, ##__VA_ARGS__)

static Madgwick madgwick_filter;
static int udp_sock;
static struct sockaddr_in udp_addr;
volatile bool g_start_requested = false;

static int button_handler(int irq, void *context, void *arg) {
    g_start_requested = !g_start_requested;
    return OK;
}

static void init_leds(void) {
    board_gpio_write(PIN_LED0, -1);
    board_gpio_config(PIN_LED0, 0, false, true, PIN_FLOAT);
    board_gpio_write(PIN_LED1, -1);
    board_gpio_config(PIN_LED1, 0, false, true, PIN_FLOAT);
    board_gpio_write(PIN_LED2, -1);
    board_gpio_config(PIN_LED2, 0, false, true, PIN_FLOAT);
    board_gpio_write(PIN_LED3, -1);
    board_gpio_config(PIN_LED3, 0, false, true, PIN_FLOAT);
}

static void set_leds(int ptn){
    board_gpio_write(PIN_LED0, (ptn & 0x01) ? 1 : 0);
    board_gpio_write(PIN_LED1, (ptn & 0x02) ? 1 : 0);
    board_gpio_write(PIN_LED2, (ptn & 0x04) ? 1 : 0);
    board_gpio_write(PIN_LED3, (ptn & 0x08) ? 1 : 0);
}

static void init_gpio(void) {
    board_gpio_config(PIN_BUTTON, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(PIN_BUTTON, INT_FALLING_EDGE, true, button_handler);
    board_gpio_int(PIN_BUTTON, true); //enable interrupt
}

static int imu_read_callback(uint32_t ev_type,
                             uint32_t timestamp,
                             char* data) {
    Packet *imu_packet = reinterpret_cast<Packet*>(data);
    std::unique_ptr<Packet[]> pose_packet(new Packet[IMU_NUM_FIFO]);

    for (int i = 0; i < IMU_NUM_FIFO; i++) {
        // printf("[ imu]ts=%f, t=%2.2f, ax=%6.2f, ay=%6.2f, az=%6.2f, gx=%8.4f, gy=%8.4f, gz=%8.4f\n",
        //        imu_data[i].data.timestamp / 19200000.0f, imu_data[i].data.temp,
        //        imu_data[i].data.ax, imu_data[i].data.ay, imu_data[i].data.az,
        //        imu_data[i].data.gx, imu_data[i].data.gy, imu_data[i].data.gz);
        sendto(udp_sock, &imu_packet[i], sizeof(Packet), 0, (struct sockaddr*)&udp_addr, sizeof(udp_addr));
        imu_file << imu_packet[i].timestamp << ","
                 << imu_packet[i].imu.temp << ","
                 << imu_packet[i].imu.ax << ","
                 << imu_packet[i].imu.ay << ","
                 << imu_packet[i].imu.az << ","
                 << imu_packet[i].imu.gx << ","
                 << imu_packet[i].imu.gy << ","
                 << imu_packet[i].imu.gz << std::endl;

        madgwick_filter.updateIMU(imu_packet[i].imu.gx, imu_packet[i].imu.gy, imu_packet[i].imu.gz,
                                  imu_packet[i].imu.ax, imu_packet[i].imu.ay, imu_packet[i].imu.az);
        float q[4];
        madgwick_filter.getQuaternion(q);
        // printf("[pose]ts=%f, x=0.0,y=0.0, z=0.0, qx=%f, qy=%f, qz=%f, qw=%f\n",
        //        imu_data[i].timestamp / 19200000.0f, q[0], q[1], q[2], q[3]);

        pose_packet[i].type = PacketType::POSE;
        pose_packet[i].timestamp = imu_packet[i].timestamp;
        pose_packet[i].pose.x = 0.0f; pose_packet[i].pose.y = 0.0f; pose_packet[i].pose.z = 0.0f;
        pose_packet[i].pose.qx = q[1]; pose_packet[i].pose.qy = q[2]; pose_packet[i].pose.qz = q[3]; pose_packet[i].pose.qw = q[0];
        sendto(udp_sock, &pose_packet[i], sizeof(Packet), 0, (struct sockaddr*)&udp_addr, sizeof(udp_addr));
    }
    return 0;
}


static int uwb_read_callback(uint32_t ev_type,
                             uint32_t timestamp,
                             char* data) {
    Packet *packet = reinterpret_cast<Packet*>(data);
    for (int i = 0; i < UWB_NUM_ANCHOR; i++) {
        if (packet[i].uwb.anchor_id < 0) {
            continue;
        }
        sendto(udp_sock, &packet[i], sizeof(Packet), 0, (struct sockaddr*)&udp_addr, sizeof(udp_addr));
        uwb_file << packet[i].timestamp << ","
                 << static_cast<int>(packet[i].uwb.anchor_id) << ","
                 << static_cast<int>(packet[i].uwb.nlos) << ","
                 << packet[i].uwb.distance << ","
                 << packet[i].uwb.azimuth << ","
                 << packet[i].uwb.elevation << std::endl;
        // printf("[ uwb]ts=%f, i=%hhd, n=%hhu, d=%6.2f, a=%6.2f, e=%6.2f\n",
        //        packet[i].timestamp, packet[i].uwb.anchor_id,
        //        packet[i].uwb.nlos, packet[i].uwb.distance,
        //        packet[i].uwb.azimuth, packet[i].uwb.elevation);
    }
    return 0;
}


extern "C" int uwbins_main(int argc, FAR char *argv[]) {
    bool started = false;
    bool led_on = true;
    int led_blink_counter = 0;
    init_leds();
    init_gpio();

    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        err("Error: socket creation failed");
        return EXIT_FAILURE;
    }
    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_port = htons(UDP_PORT);
    inet_pton(AF_INET, UDP_IP, &udp_addr.sin_addr);

    madgwick_filter.begin(IMU_SAMPLING_FREQUENCY);


    physical_sensor_t *imu = ImuSensorCreate(imu_read_callback);
    if (imu == NULL) {
        err("Error: ImuSensorCreate() failure.\n");
        return EXIT_FAILURE;
    }
    physical_sensor_t *uwb = UwbSensorCreate(uwb_read_callback);
    if (uwb == NULL) {
        err("Error: UwbSensorCreate() failure.\n");
        return EXIT_FAILURE;
    }
    if (ImuSensorOpen(imu) < 0) {
        err("Error: ImuSensorOpen() failure.\n");
        return EXIT_FAILURE;
    }
    if (UwbSensorOpen(uwb) < 0) {
        err("Error: UwbSensorOpen() failure.\n");
        return EXIT_FAILURE;
    }
    message("Setup done.\n");

    while (1) {
        if (g_start_requested && !started) {
            if (!init_log_files()) {
                err("Error: open_sensor_logs() failure.\n");
                return EXIT_FAILURE;
            }
            usleep(1000 * 1000);
            if (ImuSensorStart(imu) < 0) {
                err("Error: ImuSensorStart() failure.\n");
                return EXIT_FAILURE;
            }
            if (UwbSensorStart(uwb) < 0) {
                err("Error: UwbSensorStart() failure.\n");
                return EXIT_FAILURE;
            }
            started = true;
            printf("Capturing...\n");
        } else if (!g_start_requested && started) {
            if (ImuSensorStop(imu) < 0) {
                err("Error: ImuSensorStop() failure.\n");
                return EXIT_FAILURE;
            }
            if (UwbSensorStop(uwb) < 0) {
                err("Error: UwbSensorStop() failure.\n");
                return EXIT_FAILURE;
            }
            close_log_files();
            printf("Capture stop.\n");
            started = false;
        }
        if (started) {
            if (++led_blink_counter >= 1) { // 100ms * 5 = 500ms
                led_on = !led_on;
                set_leds(led_on ? 0x0F : 0x00);
                led_blink_counter = 0;
            }
        } else {
            set_leds(0x0F);
        }
      usleep(100 * 1000); // 100ms
    }

    message("Finalizing...\n");
    if (ImuSensorClose(imu) < 0) {
        err("Error: ImuSensorClose() failure.\n");
        return EXIT_FAILURE;
    }
    if (UwbSensorClose(uwb) < 0) {
        err("Error: UwbSensorClose() failure.\n");
        return EXIT_FAILURE;
    }
    close_log_files();
    ImuSensorDestroy(imu);
    UwbSensorDestroy(uwb);
    close(udp_sock);
    message("Finalized.\n");
    return EXIT_SUCCESS;
}