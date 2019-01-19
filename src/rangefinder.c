#include <modules/worker_thread/worker_thread.h>
#include <modules/driver_vl53l1x/vl53l1_api.h>
#include <uavcan.equipment.range_sensor.Measurement.h>
#include <modules/param/param.h>

PARAM_DEFINE_BOOL_PARAM_STATIC(publish_rangefinder, "PUBLISH_RANGEFINDER", true)

WORKER_THREAD_DECLARE_EXTERN(i2c2_thread)

static const I2CConfig i2cconfig = {
  STM32_TIMINGR_PRESC(11U) |
  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(3U)  | STM32_TIMINGR_SCLL(9U),
  0,
  0
};

static VL53L1_DistanceModes distance_mode = VL53L1_DISTANCEMODE_SHORT;

static VL53L1_Dev_t vl53l1x;

static struct worker_thread_timer_task_s range_task;
static void range_task_func(struct worker_thread_timer_task_s* task);

RUN_AFTER(INIT_END) {
    // initialize VL53L1 lidar
    i2cStart(&I2CD2, &i2cconfig);
    vl53l1x.bus = &I2CD2;
    vl53l1x.i2c_address = 0x29;

    VL53L1_Error status;
    status = VL53L1_WaitDeviceBooted(&vl53l1x);
    status = VL53L1_DataInit(&vl53l1x);
    status = VL53L1_StaticInit(&vl53l1x);
    status = VL53L1_SetDistanceMode(&vl53l1x, distance_mode);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&vl53l1x, 140000);
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(&vl53l1x, 145);
    status = VL53L1_StartMeasurement(&vl53l1x);

    worker_thread_add_timer_task(&i2c2_thread, &range_task, range_task_func, NULL, LL_US2ST(200), true);
}

static void range_task_func(struct worker_thread_timer_task_s* task) {
    // get VL53L1 range measurement
    VL53L1_Error status;
    uint8_t measurement_available = 0;
    status = VL53L1_GetMeasurementDataReady(&vl53l1x, &measurement_available);
    if (measurement_available) {
        VL53L1_RangingMeasurementData_t meas_data;
        status = VL53L1_GetRangingMeasurementData(&vl53l1x, &meas_data);

        if (status == VL53L1_ERROR_NONE) {
            static struct uavcan_equipment_range_sensor_Measurement_s msg;
            memset(&msg, 0, sizeof(msg));

            msg.sensor_id = 0;
            msg.beam_orientation_in_body_frame.orientation_defined = false;

            msg.field_of_view = 0.4;

            if (meas_data.RangeStatus == 0 || meas_data.RangeStatus == 7) {
                msg.range = meas_data.RangeMilliMeter*1e-3;
                msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
            } else {
                msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;

                // Try the other distance mode
                switch(distance_mode) {
                    case VL53L1_DISTANCEMODE_SHORT:
                        distance_mode = VL53L1_DISTANCEMODE_LONG;
                        break;
                    case VL53L1_DISTANCEMODE_LONG:
                        distance_mode = VL53L1_DISTANCEMODE_SHORT;
                        break;
                }
                status = VL53L1_SetDistanceMode(&vl53l1x, distance_mode);
            }
            uavcan_broadcast(0, &uavcan_equipment_range_sensor_Measurement_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM+1, &msg);
        }

        status = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x);
    }
}
