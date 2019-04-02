#include <imu_reader.h>
#include <gyro_integrator.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/param/param.h>
#include <math.h>
#include <string.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#ifndef IMU_INTEGRATOR_WORKER_THREAD
#error Please define IMU_INTEGRATOR_WORKER_THREAD in framework_conf.h.
#endif

#define WT IMU_INTEGRATOR_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

struct pubsub_topic_s gyro_deltas_topic;

static systime_t last_publish;

static int32_t gyro_sum_int[3];


#define DELAY_BUF_DEPTH 80
static struct {
    int16_t gyro[3];
} delay_buf[DELAY_BUF_DEPTH];

static uint8_t delay_buf_head;

static bool first_run = true;
static uint32_t raw_meas_count;
static float dt = 1/8000.0;
static float dt_sum;

PARAM_DEFINE_FLOAT32_PARAM_STATIC(max_update_rate, "FLOW_RATE_HZ", 20.0f, 1.0f, 1000.0f)

static bool trigger;

static struct worker_thread_listener_task_s raw_imu_listener_task;
static void raw_imu_handler(size_t msg_size, const void* buf, void* ctx);

RUN_ON(PUBSUB_TOPIC_INIT) {
    pubsub_init_topic(&gyro_deltas_topic, NULL);
}

RUN_ON(INIT_END) {
    worker_thread_add_listener_task(&WT, &raw_imu_listener_task, &invensense_raw_sample_topic, raw_imu_handler, NULL);
}

void gyro_integrator_trigger(void) {
    trigger = true;
}

static void delta_publisher_func(size_t msg_size, void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    
    struct gyro_delta_s* delta = (struct gyro_delta_s*)buf;
    
    delta->dt = dt_sum;
    delta->delta_gyro[0] = (gyro_sum_int[0]*0.0010652969463144809 * dt_sum) / raw_meas_count;
    delta->delta_gyro[1] = (gyro_sum_int[1]*0.0010652969463144809 * dt_sum) / raw_meas_count;
    delta->delta_gyro[2] = (gyro_sum_int[2]*0.0010652969463144809 * dt_sum) / raw_meas_count;

    board_apply_imu_rotation(delta->delta_gyro);
}

static void raw_imu_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    
    const struct imu_sample_s* raw_sample = (const struct imu_sample_s*)buf;

    delay_buf_head = (delay_buf_head+1) % DELAY_BUF_DEPTH;

    gyro_sum_int[0] += delay_buf[delay_buf_head].gyro[0];
    gyro_sum_int[1] += delay_buf[delay_buf_head].gyro[1];
    gyro_sum_int[2] += delay_buf[delay_buf_head].gyro[2];

    delay_buf[delay_buf_head].gyro[0] = raw_sample->gyro_x;
    delay_buf[delay_buf_head].gyro[1] = raw_sample->gyro_y;
    delay_buf[delay_buf_head].gyro[2] = raw_sample->gyro_z;
    
    dt_sum += dt;
    raw_meas_count++;

    const float publish_interval = 1.0/max_update_rate;

    if (trigger && dt_sum >= publish_interval) {
        pubsub_publish_message(&gyro_deltas_topic, sizeof(struct gyro_delta_s), delta_publisher_func, NULL);
        memset(gyro_sum_int, 0, sizeof(gyro_sum_int));

        const systime_t tnow = chVTGetSystemTimeX();
        const float pub_dt_meas = (tnow-last_publish)/(float)CH_CFG_ST_FREQUENCY;
        const float dt_meas = pub_dt_meas/(float)raw_meas_count;

        const float alpha = pub_dt_meas/(pub_dt_meas+10.0);
        if (!first_run) {
            dt += (dt_meas-dt)*alpha;
        }
        first_run = false;
        dt_sum = 0;
        raw_meas_count = 0;
        last_publish = tnow;
    }
    trigger = false;
}
