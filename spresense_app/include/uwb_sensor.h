#ifndef _UWBINS_UWB_SENSOR_H
#define _UWBINS_UWB_SENSOR_H


#include <poll.h>

#include "physical_sensor.h"


#define UWB_NUM_ANCHOR 4
#define UWB_SAMPLING_FREQUENCY 30 // [Hz]


#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
  FAR physical_sensor_t *UwbSensorCreate(pysical_event_handler_t handler);
  int UwbSensorOpen(FAR physical_sensor_t *sensor);
  int UwbSensorStart(FAR physical_sensor_t *sensor);
  int UwbSensorStop(FAR physical_sensor_t *sensor);
  int UwbSensorClose(FAR physical_sensor_t *sensor);
  int UwbSensorDestroy(FAR physical_sensor_t *sensor);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

struct type2bp_data_s {
    uint32_t timestamp;
    int32_t anchor_id;
    uint32_t nlos;
    float distance;   // [cm]
    float azimuth;    // [degree]
    float elevation;  // [degree]
};
typedef struct type2bp_data_s type2bp_data_t;


#ifdef __cplusplus
class UwbSensorClass : public PhysicalSensorClass {
public:
    UwbSensorClass(FAR physical_sensor_t *sensor) : PhysicalSensorClass(sensor) {
        create();
    };

    ~UwbSensorClass() {};

private:
    /* Override method */
    int open_sensor();
    int close_sensor();
    int start_sensor();
    int stop_sensor();
    int setup_sensor(FAR void *param);

    /* Local method */
    int read_data();
    void parse_uwb_data(const char* src_data, type2bp_data_t* data_array);
    int notify_data(MemMgrLite::MemHandle &mh_dst);

    /* Inline method */
    uint32_t get_timestamp(){ // TODO
        /* Get timestamp in millisecond. Tick in 32768 Hz  */
        // return 1000 * m_wm_ts.sec + ((1000 * m_wm_ts.tick) >> 15);
        return 0;
    }

    int m_fd;
    struct pollfd fds[1];
};

#endif /* __cplusplus */
#endif /* _EXAMPLES_UWBINS_UWB_SENSOR_H */
