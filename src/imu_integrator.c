#include <imu_reader.h>
#include <imu_integrator.h>
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

struct pubsub_topic_s imu_deltas_topic;

static float x[2][7];
static uint8_t x_idx;

PARAM_DEFINE_FLOAT32_PARAM_STATIC(update_rate, "IMU_RATE_HZ", 100.0f, 1.0f, 1000.0f)

static systime_t last_publish;
static uint32_t raw_meas_count;
static float dt = 1/8000.0;
static float dt_sum;
static bool first_run = true;

static struct worker_thread_listener_task_s raw_imu_listener_task;
static void raw_imu_handler(size_t msg_size, const void* buf, void* ctx);
static void integrate(float* x, float* omega, float* accel, float dt, float* x_ret);

RUN_ON(PUBSUB_TOPIC_INIT) {
    pubsub_init_topic(&imu_deltas_topic, NULL);
}

RUN_ON(INIT_END) {
    memset(x,0,sizeof(x));
    x[x_idx][0] = 1;
    worker_thread_add_listener_task(&WT, &raw_imu_listener_task, &invensense_raw_sample_topic, raw_imu_handler, NULL);
}

static void delta_publisher_func(size_t msg_size, void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    
    struct imu_delta_s* delta = (struct imu_delta_s*)buf;
    
    delta->dt = dt_sum;
    
    float l = sqrtf(((x[x_idx][1])*(x[x_idx][1])) + ((x[x_idx][2])*(x[x_idx][2])) + ((x[x_idx][3])*(x[x_idx][3])));

    if(l == 0) {
        delta->delta_ang[0] = 0;
        delta->delta_ang[1] = 0;
        delta->delta_ang[2] = 0;
    } else {
        delta->delta_ang[0] = 2.0*atan2f(l,x[x_idx][0])/l * x[x_idx][1];
        delta->delta_ang[1] = 2.0*atan2f(l,x[x_idx][0])/l * x[x_idx][2];
        delta->delta_ang[2] = 2.0*atan2f(l,x[x_idx][0])/l * x[x_idx][3];
    }

    delta->delta_vel[0] = x[x_idx][4];
    delta->delta_vel[1] = x[x_idx][5];
    delta->delta_vel[2] = x[x_idx][6];

    board_apply_imu_rotation(delta->delta_ang);
    board_apply_imu_rotation(delta->delta_vel);
    
    memset(x,0,sizeof(x));
    x[x_idx][0] = 1;
}

static void raw_imu_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    
    const struct imu_sample_s* raw_sample = (const struct imu_sample_s*)buf;
    
    uint8_t prev_x_idx = x_idx;
    x_idx = (x_idx+1)%2;
    
    float omega[] = { raw_sample->gyro_x*0.0010652969463144809, raw_sample->gyro_y*0.0010652969463144809, raw_sample->gyro_z*0.0010652969463144809 };
    float accel[] = { raw_sample->accel_x*0.004788500625629444, raw_sample->accel_y*0.004788500625629444, raw_sample->accel_z*0.004788500625629444 };
    
    integrate(x[prev_x_idx], omega, accel, dt, x[x_idx]);
    
    dt_sum += dt;
    raw_meas_count++;

    const float publish_interval = 1.0/update_rate;

    if (dt_sum >= publish_interval) {
        pubsub_publish_message(&imu_deltas_topic, sizeof(struct imu_delta_s), delta_publisher_func, NULL);

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
}

static void integrate(float* x, float* omega, float* accel, float dt, float* x_ret) {
    float X0 = (1.0f/2.0f)*omega[0];
    float X1 = (1.0f/2.0f)*omega[1];
    float X2 = (1.0f/2.0f)*omega[2];
    float X3 = dt*(-X0*x[1] - X1*x[2] - X2*x[3]) + x[0];
    float X4 = dt*(X0*x[0] - X1*x[3] + X2*x[2]) + x[1];
    float X5 = (1.0f/2.0f)*x[0];
    float X6 = dt*(X0*x[3] - X2*x[1] + X5*omega[1]) + x[2];
    float X7 = dt*(-X0*x[2] + X1*x[1] + X5*omega[2]) + x[3];
    float X8 = 1/(sqrtf(((fabsf(X3))*(fabsf(X3))) + ((fabsf(X4))*(fabsf(X4))) + ((fabsf(X6))*(fabsf(X6))) + ((fabsf(X7))*(fabsf(X7)))));
    float X9 = ((x[0])*(x[0]));
    float X10 = ((x[2])*(x[2]));
    float X11 = -X10;
    float X12 = ((x[3])*(x[3]));
    float X13 = -X12;
    float X14 = ((x[1])*(x[1]));
    float X15 = 2.0*x[1];
    float X16 = X15*x[2];
    float X17 = 2.0*x[0];
    float X18 = X17*x[3];
    float X19 = X17*x[2];
    float X20 = X15*x[3];
    float X21 = -X14 + X9;
    float X22 = 2.0*x[2]*x[3];
    float X23 = X15*x[0];

    x_ret[0] = X3*X8;
    x_ret[1] = X4*X8;
    x_ret[2] = X6*X8;
    x_ret[3] = X7*X8;
    x_ret[4] = dt*(accel[0]*(X11 + X13 + 1.0*X14 + X9) + accel[1]*(X16 - X18) + accel[2]*(X19 + X20)) + x[4];
    x_ret[5] = dt*(accel[0]*(X16 + X18) + accel[1]*(1.0*X10 + X13 + X21) + accel[2]*(X22 - X23)) + x[5];
    x_ret[6] = dt*(accel[0]*(-X19 + X20) + accel[1]*(X22 + X23) + accel[2]*(X11 + 1.0*X12 + X21)) + x[6];
}
