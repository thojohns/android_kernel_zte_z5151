#
# Makefile for the focaltech touchscreen drivers.
#

# Each configuration option enables a list of files.
ccflags-y += -I$(srctree)/drivers/input/touchscreen/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/focaltech_touch/firmware_config/$(CONFIG_TS_FIRMWARE)/
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)	+=  focaltech_core.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)	+=  focaltech_ex_fun.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)	+=  focaltech_ex_mode.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)	+=  focaltech_flash.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)	+=  focaltech_gesture.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)	+=  focaltech_esdcheck.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)   +=  focaltech_i2c.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)   +=  focaltech_sensor.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)   +=  focaltech_point_report_check.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)   +=  focaltech_flash/
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST)   +=  focaltech_test/

