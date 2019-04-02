#include <modules/worker_thread/worker_thread.h>
#include <modules/param/param.h>
#include <hal.h>
#include <string.h>

#include <modules/uavcan_debug/uavcan_debug.h>

#include <modules/driver_pmw3901mb/driver_pmw3901mb.h>

#include <com.hex.equipment.flow.Measurement.h>
#include <uavcan.equipment.ahrs.RawIMU.h>
#include <uavcan.protocol.file.Read.h>

#include <imu_integrator.h>

WORKER_THREAD_DECLARE_EXTERN(spi3_thread)

static struct worker_thread_listener_task_s imu_delta_listener_task;
static void imu_deltas_handler(size_t msg_size, const void* buf, void* ctx);

PARAM_DEFINE_BOOL_PARAM_STATIC(publish_raw_imu, "PUBLISH_RAW_IMU", false)

RUN_AFTER(INIT_END) {
    worker_thread_add_listener_task(&spi3_thread, &imu_delta_listener_task, &imu_deltas_topic, imu_deltas_handler, NULL);
}

static void imu_deltas_handler(size_t msg_size, const void* buf, void* ctx) {
    const struct imu_delta_s* deltas = (const struct imu_delta_s*)buf;

    if (publish_raw_imu) {
        struct uavcan_equipment_ahrs_RawIMU_s msg;
        memset(&msg, 0, sizeof(msg));

        msg.integration_interval = deltas->dt;

        msg.rate_gyro_latest[0] = deltas->delta_ang[0]/deltas->dt;
        msg.rate_gyro_latest[1] = deltas->delta_ang[1]/deltas->dt;
        msg.rate_gyro_latest[2] = deltas->delta_ang[2]/deltas->dt;

        msg.accelerometer_latest[0] = deltas->delta_vel[0]/deltas->dt;
        msg.accelerometer_latest[1] = deltas->delta_vel[1]/deltas->dt;
        msg.accelerometer_latest[2] = deltas->delta_vel[2]/deltas->dt;

        msg.rate_gyro_integral[0] = deltas->delta_ang[0];
        msg.rate_gyro_integral[1] = deltas->delta_ang[1];
        msg.rate_gyro_integral[2] = deltas->delta_ang[2];

        msg.accelerometer_integral[0] = deltas->delta_vel[0];
        msg.accelerometer_integral[1] = deltas->delta_vel[1];
        msg.accelerometer_integral[2] = deltas->delta_vel[2];

        uavcan_broadcast(0, &uavcan_equipment_ahrs_RawIMU_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &msg);
    }
}
