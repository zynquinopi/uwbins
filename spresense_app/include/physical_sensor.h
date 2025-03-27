#ifndef _PHYSICAL_SENSOR_H
#define _PHYSICAL_SENSOR_H


#include <fcntl.h>
#include <errno.h>
#include <mqueue.h>

#include <nuttx/queue.h>

#include "memutils/memory_manager/MemHandle.h"
#include "include/mem_conf/mem_layout.h"
#include "include/mem_conf/msgq_id.h"


#define PHYSICAL_SENSOR_ERR_CODE_OK       0   /* Success.                    */
#define PHYSICAL_SENSOR_ERR_CODE_PARAM   -1   /* Parameter is invalid.       */
#define PHYSICAL_SENSOR_ERR_CODE_SEND    -2   /* Message send is failure.    */
#define PHYSICAL_SENSOR_ERR_CODE_RECEIVE -3   /* Message receive is failure. */
#define PHYSICAL_SENSOR_ERR_CODE_IOCTL   -4   /* I/O control is failure.     */
#define PHYSICAL_SENSOR_ERR_CODE_FATAL   -5   /* Fatal error is occured.     */

#define PHYSICAL_SENSOR_MAX_MQ_NAME 24


typedef int (*pysical_event_handler_t)(uint32_t event_id,
                                       uint32_t timestamp,
                                       MemMgrLite::MemHandle &mh);

struct physical_sensor_s {
    mqd_t mq_request_des;
    char mq_request_name[PHYSICAL_SENSOR_MAX_MQ_NAME];
    mqd_t mq_response_des;
    char mq_response_name[PHYSICAL_SENSOR_MAX_MQ_NAME];
    pysical_event_handler_t handler;
    pthread_t thread_id;
};

typedef struct physical_sensor_s physical_sensor_t;


#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
    FAR physical_sensor_t *PhysicalSensorCreate(
        pysical_event_handler_t handler,
        FAR void *entry_function,
        FAR const char *dev_name);
    int PhysicalSensorOpen(FAR physical_sensor_t *sensor, FAR void *param);
    int PhysicalSensorStart(FAR physical_sensor_t *sensor);
    int PhysicalSensorStop(FAR physical_sensor_t *sensor);
    int PhysicalSensorClose(FAR physical_sensor_t *sensor);
    int PhysicalSensorDestroy(FAR physical_sensor_t *sensor);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */


#ifdef __cplusplus
class PhysicalSensorClass {
public:
    enum PhysicalSensorMsgType_e {
        PHYSICAL_SENSOR_MSG_TYPE_CREATE = 0,
        PHYSICAL_SENSOR_MSG_TYPE_OPEN,
        PHYSICAL_SENSOR_MSG_TYPE_START,
        PHYSICAL_SENSOR_MSG_TYPE_STOP,
        PHYSICAL_SENSOR_MSG_TYPE_CLOSE,
        PHYSICAL_SENSOR_MSG_TYPE_DESTROY,
        PHYSICAL_SENSOR_MSG_TYPE_NUM
    };

    struct physical_sensor_request_s {
        PhysicalSensorMsgType_e msg_type;
        void *param;
    };

    typedef struct physical_sensor_request_s physical_sensor_request_t;
    struct physical_sensor_response_s {
        PhysicalSensorMsgType_e msg_type;
        int result;
    };
    typedef struct physical_sensor_response_s physical_sensor_response_t;

    PhysicalSensorClass(FAR physical_sensor_t *sensor) : m_status(PHYSICAL_SENSOR_STATE_END),
                                                         m_mq_request_des(sensor->mq_request_des),
                                                         m_mq_response_des(sensor->mq_response_des),
                                                         m_handler(sensor->handler) {
        /* Write self thread id for cancel process */
        sensor->thread_id = pthread_self();
    };
    virtual ~PhysicalSensorClass() {};
    void create();
    void run();

private:
    enum PhysicalSensorState_e {
        PHYSICAL_SENSOR_STATE_IDLE = 0,
        PHYSICAL_SENSOR_STATE_READY,
        PHYSICAL_SENSOR_STATE_RUNNING,
        PHYSICAL_SENSOR_STATE_END,
        PHYSICAL_SENSOR_STATE_NUM
    };

    void msg_destroy(physical_sensor_request_t &msg);
    void msg_open(physical_sensor_request_t &msg);
    void msg_close(physical_sensor_request_t &msg);
    void msg_start(physical_sensor_request_t &msg);
    void msg_stop(physical_sensor_request_t &msg);
    void msg_illegal(physical_sensor_request_t &msg);

    void parse(physical_sensor_request_t &msg);

    /* Virtual methods */
    virtual int open_sensor() = 0;
    virtual int close_sensor() = 0;
    virtual int start_sensor() = 0;
    virtual int stop_sensor() = 0;

    virtual int setup_sensor(FAR void *param) {
        /* Do nothing. */
        return 0;
    };

    virtual int read_data() {
        /* Do nothing. */
        return 0;
    };

    /* Inline methods */
    bool send(physical_sensor_response_t &msg) {
        if (mq_send(m_mq_response_des,
                    (FAR const char *)&msg,
                    sizeof(physical_sensor_response_t), 0) < 0) {
            return false;
        }
        return true;
    }

    bool receive(FAR physical_sensor_request_t &msg) {
        if (mq_receive(m_mq_request_des,
                       (FAR char *)&msg,
                       sizeof(physical_sensor_request_t), 0) < 0) {
            return false;
        }
        return true;
    }

    typedef void (PhysicalSensorClass::*msg_process)(physical_sensor_request_t &msg);
    static msg_process msg_process_table[PHYSICAL_SENSOR_MSG_TYPE_NUM][PHYSICAL_SENSOR_STATE_NUM];

    int m_status;
    mqd_t m_mq_request_des;
    mqd_t m_mq_response_des;

protected:
    pysical_event_handler_t m_handler;
};

#endif /* __cplusplus */
#endif /* _PHYSICAL_SENSOR_H */