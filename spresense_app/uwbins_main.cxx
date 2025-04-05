#include <nuttx/config.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <memory>

#include "include/imu_sensor.h"
#include "include/uwb_sensor.h"
#include "sensing/sensor_ecode.h"
#include "include/types.h"
#include "include/MadgwickAHRS.h"


#define UDP_IP "192.168.11.11"
#define UDP_PORT 50000

#define EXIT_REQUEST_KEY 0x20 /* space key */

#define message(format, ...) printf(format, ##__VA_ARGS__)
#define err(format, ...) fprintf(stderr, format, ##__VA_ARGS__)

static Madgwick madgwick_filter;
static int udp_sock;
static struct sockaddr_in udp_addr;


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
        // printf("[ uwb]ts=%f, i=%hhd, n=%hhu, d=%6.2f, a=%6.2f, e=%6.2f\n",
        //        packet[i].timestamp, packet[i].uwb.anchor_id,
        //        packet[i].uwb.nlos, packet[i].uwb.distance,
        //        packet[i].uwb.azimuth, packet[i].uwb.elevation);
    }
    return 0;
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

    madgwick_filter.begin(IMU_SAMPLING_FREQUENCY);


    physical_sensor_t *imu = ImuSensorCreate(imu_read_callback);
    if (imu == NULL) {
        err("Error: ImuSensorCreate() failure.\n");
        return EXIT_FAILURE;
    }
    FAR physical_sensor_t *uwb = UwbSensorCreate(uwb_read_callback);
    if (uwb == NULL) {
        err("Error: UwbSensorCreate() failure.\n");
        return EXIT_FAILURE;
    }

    message("start sensoring...\n");

    /* Start physical sensor process. */
    if (ImuSensorOpen(imu) < 0) {
        err("Error: ImuSensorOpen() failure.\n");
        return EXIT_FAILURE;
    }
    if (UwbSensorOpen(uwb) < 0) {
        err("Error: UwbSensorOpen() failure.\n");
        return EXIT_FAILURE;
    }
    if (ImuSensorStart(imu) < 0) {
        err("Error: ImuSensorStart() failure.\n");
        return EXIT_FAILURE;
    }
    if (UwbSensorStart(uwb) < 0) {
        err("Error: UwbSensorStart() failure.\n");
        return EXIT_FAILURE;
    }

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
    printf("ImuSensorStop() success.\n");
    if (ImuSensorClose(imu) < 0) {
        err("Error: ImuSensorClose() failure.\n");
        return EXIT_FAILURE;
    }
    // printf("ImuSensorClose() success.\n");
    if (UwbSensorStop(uwb) < 0) {
        err("Error: UwbSensorStop() failure.\n");
        return EXIT_FAILURE;
    }
    printf("UwbSensorStop() success.\n");
    if (UwbSensorClose(uwb) < 0) {
        err("Error: UwbSensorClose() failure.\n");
        return EXIT_FAILURE;
    }
    printf("UwbSensorClose() success.\n");

    ImuSensorDestroy(imu);
    printf("ImuSensorDestroy() success.\n");
    UwbSensorDestroy(uwb);
    printf("UwbSensorDestroy() success.\n");
    close(udp_sock);
    printf("close() success.\n");
    return EXIT_SUCCESS;
}