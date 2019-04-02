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

PUBSUB_TOPIC_GROUP_CREATE(invensense_raw_sample_topic_group, 1024);
struct pubsub_topic_s invensense_raw_sample_topic;

RUN_ON(PUBSUB_TOPIC_INIT) {
    pubsub_init_topic(&invensense_raw_sample_topic, &invensense_raw_sample_topic_group);
}

RUN_AFTER(INIT_END) {
    invensense_init(&invensense, 3, BOARD_PAL_LINE_SPI_CS_ICM, INVENSENSE_IMU_TYPE_ICM20602);

    worker_thread_add_timer_task(&WT, &invensense_read_task, invensense_read_task_func, NULL, LL_US2ST(200), true);
}

static void invensense_read_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    struct imu_sample_s data[72];
    
    size_t count = invensense_read_fifo(&invensense, data)/sizeof(struct imu_sample_s);
    
    for (size_t i=0; i<count; i++) {
        pubsub_publish_message(&invensense_raw_sample_topic, sizeof(struct imu_sample_s), pubsub_copy_writer_func, &data[0]);
    }
}
