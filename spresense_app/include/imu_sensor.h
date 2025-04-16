#ifndef _UWBINS_IMU_SENSOR_H
#define _UWBINS_IMU_SENSOR_H


#include <poll.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <memory>

#include "physical_sensor.h"
#include "include/types.h"


#define IMU_SAMPLING_FREQUENCY   120 //15, 30, 60, 120, 240, 480, 960, 1920 [Hz]
#define IMU_ACCEL_DRANGE          2 // 2, 4, 8, 16 [g]
#define IMU_GYRO_DRANGE         500 // 125, 250, 500, 1000, 2000, 4000 [dps]
#define IMU_NUM_FIFO              1 // 1, 2, 3, 4


extern "C" {
    physical_sensor_t *ImuSensorCreate(pysical_event_handler_t handler);
    int ImuSensorOpen(physical_sensor_t *sensor);
    int ImuSensorStart(physical_sensor_t *sensor);
    int ImuSensorStop(physical_sensor_t *sensor);
    int ImuSensorClose(physical_sensor_t *sensor);
    int ImuSensorDestroy(physical_sensor_t *sensor);

} /* end of extern "C" */


class ImuSensorClass : public PhysicalSensorClass {
public:
    ImuSensorClass(physical_sensor_t *sensor) : PhysicalSensorClass(sensor) {
        create();
    };

    ~ImuSensorClass() {};

private:
    /* Override method */
    int open_sensor();
    int close_sensor();
    int start_sensor();
    int stop_sensor();
    int setup_sensor(void *param);

    /* Local method */
    int read_data();
    int read_away_50ms_data();

    /* Inline method */
    uint32_t get_timestamp(){ // TODO
        /* Get timestamp in millisecond. Tick in 32768 Hz  */
        // return 1000 * m_wm_ts.sec + ((1000 * m_wm_ts.tick) >> 15);
        return 0;
    }

  int m_fd;
  struct pollfd fds[1];
  std::unique_ptr<Packet[]> out;
  std::unique_ptr<char[]> ptr;
};
#endif /* _EXAMPLES_UWBINS_IMU_SENSOR_H */
