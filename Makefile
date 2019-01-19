CSRC = $(shell find src -name "*.c")
INCDIR = ./include
MODULE_SEARCH_DIRS = modules

USE_EXCEPTIONS_STACKSIZE = 256
USE_PROCESS_STACKSIZE = 1536

UDEFS += -DUSE_I2C_2V8

MODULES_ENABLED = \
chibios_sys_init \
chibios_hal_init \
app_descriptor \
boot_msg \
timing \
system \
pubsub \
worker_thread \
can_driver_stm32 \
can \
can_autobaud \
uavcan \
uavcan_nodestatus_publisher \
uavcan_getnodeinfo_server \
uavcan_beginfirmwareupdate_server \
uavcan_allocatee \
uavcan_restart \
freemem_check \
uavcan_debug \
flash \
param \
uavcan_param_interface \
driver_vl53l1x \
spi_device \
driver_pmw3901mb \
driver_invensense \
stack_measurement

MESSAGES_ENABLED = \
uavcan.protocol.debug.LogMessage \
com.hex.equipment.flow.Measurement \
uavcan.equipment.range_sensor.Measurement \
uavcan.equipment.ahrs.RawIMU \
uavcan.protocol.file.Read

VENDOR_DSDL_NAMESPACE_DIRS = ./dsdl/com

include framework/include.mk
