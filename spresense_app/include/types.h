#ifndef _UWBINS_TYPES_H
#define _UWBINS_TYPES_H

typedef enum {
    DATA_TYPE_IMU  = 0,
    DATA_TYPE_UWB  = 1,
    DATA_TYPE_POSE = 2
} data_type_t;

struct pose_data_s {
    data_type_t type;
    float timestamp;
    float x, y, z;
    float qx, qy, qz, qw;
};
typedef struct pose_data_s pose_data_t;

#endif /* _UWBINS_TYPES_H */