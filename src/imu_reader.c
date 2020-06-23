#include <modules/driver_invensense/driver_invensense.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/pubsub/pubsub.h>

#include <imu_reader.h>

#include <math.h>
#include <hal.h>
#include <string.h>

#ifndef IMU_READER_WORKER_THREAD
#error Please define IMU_READER_WORKER_THREAD in framework_conf.h.
#endif

#define WT IMU_READER_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct invensense_instance_s invensense;

static struct worker_thread_timer_task_s invensense_read_task;
static void invensense_read_task_func(struct worker_thread_timer_task_s* task);
const int16_t ovf_val = 0x7000;
static int16_t prev_gyro[3];

PUBSUB_TOPIC_GROUP_CREATE(invensense_raw_sample_topic_group, 1024);
struct pubsub_topic_s invensense_raw_sample_topic;

RUN_ON(PUBSUB_TOPIC_INIT) {
    pubsub_init_topic(&invensense_raw_sample_topic, &invensense_raw_sample_topic_group);
}

RUN_AFTER(INIT_END) {
    invensense_init(&invensense, 3, BOARD_PAL_LINE_SPI_CS_ICM, INVENSENSE_IMU_TYPE_ICM20602);

    worker_thread_add_timer_task(&WT, &invensense_read_task, invensense_read_task_func, NULL, LL_US2ST(200), true);
}

static void __attribute__((optimize("O3"))) invensense_read_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    struct imu_sample_s data[72];
    
    size_t count = invensense_read_fifo(&invensense, data)/sizeof(struct imu_sample_s);
    
    for (size_t i=0; i<count; i++) {
        if(data[i].gyro_x > ovf_val || data[i].gyro_x < -ovf_val) {
            data[i].gyro_x = prev_gyro[0];
        } else {
            prev_gyro[0] = data[i].gyro_x;
        }

        if(data[i].gyro_y > ovf_val || data[i].gyro_y < -ovf_val) {
            data[i].gyro_y = prev_gyro[1];
        } else {
            prev_gyro[1] = data[i].gyro_y;
        }

        if(data[i].gyro_z > ovf_val || data[i].gyro_z < -ovf_val) {
            data[i].gyro_z = prev_gyro[2];
        } else {
            prev_gyro[2] = data[i].gyro_z;
        }

        pubsub_publish_message(&invensense_raw_sample_topic, sizeof(struct imu_sample_s), pubsub_copy_writer_func, &data[i]);
    }
}
