#include <modules/worker_thread/worker_thread.h>
#include <modules/param/param.h>
#include <hal.h>

#include <modules/uavcan_debug/uavcan_debug.h>

#include <modules/driver_pmw3901mb/driver_pmw3901mb.h>

#include <com.hex.equipment.flow.Measurement.h>
#include <uavcan.equipment.ahrs.RawIMU.h>
#include <uavcan.protocol.file.Read.h>

#include <gyro_integrator.h>
#include <string.h>

WORKER_THREAD_DECLARE_EXTERN(spi3_thread)

static struct pmw3901mb_instance_s pmw3901mb;

static struct worker_thread_listener_task_s gyro_delta_listener_task;
static void gyro_deltas_handler(size_t msg_size, const void* buf, void* ctx);

static struct worker_thread_timer_task_s flow_task;
static void flow_task_func(struct worker_thread_timer_task_s* task);

static systime_t last_publish;

PARAM_DEFINE_BOOL_PARAM_STATIC(publish_flow, "PUBLISH_FLOW", true)

RUN_AFTER(INIT_END) {
    // initialize PMW3901MB optical flow
    pmw3901mb_init(&pmw3901mb, FLOW_SPI_BUS, BOARD_PAL_LINE_SPI_CS_FLOW, PMW3901MB_TYPE_V1);

    worker_thread_add_listener_task(&spi3_thread, &gyro_delta_listener_task, &gyro_deltas_topic, gyro_deltas_handler, NULL);
    worker_thread_add_timer_task(&spi3_thread, &flow_task, flow_task_func, NULL, LL_US2ST(500), true);
}

static void flow_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    if (publish_flow) {
        if (pmw3901mb_read(&pmw3901mb, 0x15) & 0x20) {
            pmw3901mb_write(&pmw3901mb, 0x15, 0);
            gyro_integrator_trigger();
        }
    }
}

static void gyro_deltas_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    const struct gyro_delta_s* deltas = (const struct gyro_delta_s*)buf;

    if (publish_flow) {
        struct pmw3901mb_motion_report_s motion_report;
        pmw3901mb_burst_read(&pmw3901mb, &motion_report);

        static struct com_hex_equipment_flow_Measurement_s msg;

        msg.quality = motion_report.squal;

        if (motion_report.shutter_upper == 0x1f && motion_report.squal < 0x19) {
            msg.quality = 0;
        }

        msg.integration_interval = deltas->dt;
        msg.rate_gyro_integral[0] = deltas->delta_gyro[0];
        msg.rate_gyro_integral[1] = deltas->delta_gyro[1];
        msg.flow_integral[0] = motion_report.delta_x*2.27e-3;
        msg.flow_integral[1] = motion_report.delta_y*2.27e-3;

        uavcan_broadcast(0, &com_hex_equipment_flow_Measurement_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg);
    }
}

#define RAW_IMAGE_FILE_NAME "raw_image"

static struct worker_thread_listener_task_s read_req_listener_task;
static void read_req_handler(size_t msg_size, const void* buf, void* ctx);

RUN_AFTER(UAVCAN_INIT) {
    struct pubsub_topic_s* read_req_topic = uavcan_get_message_topic(0, &uavcan_protocol_file_Read_req_descriptor);
    worker_thread_add_listener_task(&spi3_thread, &read_req_listener_task, read_req_topic, read_req_handler, NULL);
}

static void read_req_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_protocol_file_Read_req_s* req = (const struct uavcan_protocol_file_Read_req_s*)msg_wrapper->msg;

    if (req->path.path_len != strlen(RAW_IMAGE_FILE_NAME) || memcmp(req->path.path, RAW_IMAGE_FILE_NAME, strlen(RAW_IMAGE_FILE_NAME)) != 0) {
        return;
    }

    struct uavcan_protocol_file_Read_res_s res;

    if (req->offset == 0) {
        pmw3901mb_frame_capture_start(&pmw3901mb);
    }

    res.data_len = pmw3901mb_frame_capture_get_frame_chunk(&pmw3901mb, sizeof(res.data), res.data);
    res.error.value = 0;

    uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
}
