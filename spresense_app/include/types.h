#ifndef _UWBINS_TYPES_H
#define _UWBINS_TYPES_H


#include <nuttx/sensors/cxd5602pwbimu.h>


enum class PacketType {
    IMU  = 0,
    UWB  = 1,
    POSE = 2
};


struct Type2bp {
    int8_t anchor_id;
    uint8_t nlos;
    float distance;   // [m]
    float azimuth;    // [degree]
    float elevation;  // [degree]
};


struct Pose {
    float x, y, z;
    float qx, qy, qz, qw;
};


struct Packet {
    PacketType type;
    uint64_t timestamp;
    union {
        cxd5602pwbimu_data_t imu;
        Type2bp uwb;
        Pose pose;
    };
};

#endif /* _UWBINS_TYPES_H */