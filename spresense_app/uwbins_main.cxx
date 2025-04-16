#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>  // struct task_info_s
#include <arch/board/board.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <pthread.h>
#include <memory>
#include <iostream>
#include <fstream>
// #include <MadgwickAHRS.h>


#include "include/types.h"
#include "include/queue.h"
#include "include/gpio_led.h"
#include "include/file_logger.h"
#include "include/imu_sensor.h"
#include "include/uwb_sensor.h"
#include "include/MadgwickAHRS.h"


#define UDP_IP "192.168.11.11"
#define UDP_PORT 50000
#define message(format, ...) printf(format, ##__VA_ARGS__)
#define err(format, ...) fprintf(stderr, format, ##__VA_ARGS__)


static Madgwick madgwick_filter;
static int udp_sock;
static struct sockaddr_in udp_addr;

SafeQueue<Packet> fileQueue;
SafeQueue<Packet> udpQueue;
SafeQueue<Packet> slamQueue;

std::unique_ptr<Packet> pose_packet;


void* file_writer_thread(void*) {
    while (true) {
        printf("[file] backlog=%zu\n", fileQueue.size());
        Packet packet = fileQueue.wait_and_pop();
        if (packet.type == PacketType::POSE) {
            continue;
        } else if (packet.type == PacketType::IMU) {
            imu_file << packet.timestamp << ","
                     << packet.imu.temp << ","
                     << packet.imu.ax << ","
                     << packet.imu.ay << ","
                     << packet.imu.az << ","
                     << packet.imu.gx << ","
                     << packet.imu.gy << ","
                     << packet.imu.gz << std::endl;
        } else if (packet.type == PacketType::UWB) {
            uwb_file << packet.timestamp << ","
                     << static_cast<int>(packet.uwb.anchor_id) << ","
                     << static_cast<int>(packet.uwb.nlos) << ","
                     << packet.uwb.distance << ","
                     << packet.uwb.azimuth << ","
                     << packet.uwb.elevation << std::endl;
        }
    }
    return nullptr;
}

void* udp_sender_thread(void*) {
    while (true) {
        printf("[udp] backlog=%zu\n", udpQueue.size());
        Packet data = udpQueue.wait_and_pop();
        sendto(udp_sock, &data, sizeof(Packet), 0, (struct sockaddr*)&udp_addr, sizeof(udp_addr));
    }
    return nullptr;
}

void* slam_est_thread(void*) {
    while (true) {
        printf("[slam] backlog=%zu\n", slamQueue.size());
        Packet packet = slamQueue.wait_and_pop();
        if (packet.type == PacketType::IMU) {
            madgwick_filter.updateIMU(packet.imu.gx, packet.imu.gy, packet.imu.gz,
                                      packet.imu.ax, packet.imu.ay, packet.imu.az);
            float q[4];
            madgwick_filter.getQuaternion(q);
            // printf("[pose]ts=%f, x=0.0,y=0.0, z=0.0, qx=%f, qy=%f, qz=%f, qw=%f\n",
            //        packet.timestamp / 19200000.0f, q[0], q[1], q[2], q[3]);
            pose_packet->type = PacketType::POSE;
            pose_packet->timestamp = packet.timestamp;
            pose_packet->pose.x = 0.0f; pose_packet->pose.y = 0.0f; pose_packet->pose.z = 0.0f;
            pose_packet->pose.qx = q[1]; pose_packet->pose.qy = q[2]; pose_packet->pose.qz = q[3]; pose_packet->pose.qw = q[0];
            udpQueue.push(*pose_packet);
        }
    }
    return nullptr;
}

static int imu_read_callback(uint32_t ev_type,
                             uint32_t timestamp,
                             char* data) {
    Packet *imu_packet = reinterpret_cast<Packet*>(data);
    for (int i = 0; i < IMU_NUM_FIFO; i++) {
        // fileQueue.push(imu_packet[i]);
        udpQueue.push(imu_packet[i]);
        slamQueue.push(imu_packet[i]);
        // printf("[ imu]ts=%f, t=%2.2f, ax=%6.2f, ay=%6.2f, az=%6.2f, gx=%8.4f, gy=%8.4f, gz=%8.4f\n",
        //        imu_data[i].data.timestamp / 19200000.0f, imu_data[i].data.temp,
        //        imu_data[i].data.ax, imu_data[i].data.ay, imu_data[i].data.az,
        //        imu_data[i].data.gx, imu_data[i].data.gy, imu_data[i].data.gz);
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
        fileQueue.push(packet[i]);
        udpQueue.push(packet[i]);
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

    pthread_attr_t attr;
    struct sched_param sch_param;
    pthread_attr_init(&attr);
    sch_param.sched_priority = 110;
    attr.stacksize = 1024 * 2;
    pthread_attr_setschedparam(&attr, &sch_param);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_t log_thread, net_thread, slam_thread;
    pthread_create(&log_thread, &attr, file_writer_thread, NULL);
    pthread_create(&net_thread, &attr, udp_sender_thread, NULL);
    pthread_create(&slam_thread, &attr, slam_est_thread, NULL);

    pose_packet = std::make_unique<Packet>();


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
            // if (UwbSensorStart(uwb) < 0) {
            //     err("Error: UwbSensorStart() failure.\n");
            //     return EXIT_FAILURE;
            // }
            started = true;
            printf("Capturing...\n");
        } else if (!g_start_requested && started) {
            if (ImuSensorStop(imu) < 0) {
                err("Error: ImuSensorStop() failure.\n");
                return EXIT_FAILURE;
            }
            // if (UwbSensorStop(uwb) < 0) {
            //     err("Error: UwbSensorStop() failure.\n");
            //     return EXIT_FAILURE;
            // }
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