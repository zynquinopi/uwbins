#ifndef _UWBINS_UWB_SENSOR_H
#define _UWBINS_UWB_SENSOR_H

#include <poll.h>

#include "include/types.h"
#include "physical_sensor.h"


#define UWB_NUM_ANCHOR 4


extern "C" {
  physical_sensor_t *UwbSensorCreate(pysical_event_handler_t handler);
  int UwbSensorOpen(physical_sensor_t *sensor);
  int UwbSensorStart(physical_sensor_t *sensor);
  int UwbSensorStop(physical_sensor_t *sensor);
  int UwbSensorClose(physical_sensor_t *sensor);
  int UwbSensorDestroy(physical_sensor_t *sensor);
} /* end of extern "C" */


class UwbSensorClass : public PhysicalSensorClass {
public:
    UwbSensorClass(physical_sensor_t *sensor) : PhysicalSensorClass(sensor) {
        create();
    };

    ~UwbSensorClass() {};

private:
    /* Override method */
    int open_sensor();
    int close_sensor();
    int start_sensor();
    int stop_sensor();
    int setup_sensor(void *param);

    /* Local method */
    int read_data();
    void parse_uwb_data(const char* src_data, Type2bp* data_array);

    int m_fd;
    struct pollfd fds[1];
};

#endif /* _EXAMPLES_UWBINS_UWB_SENSOR_H */
