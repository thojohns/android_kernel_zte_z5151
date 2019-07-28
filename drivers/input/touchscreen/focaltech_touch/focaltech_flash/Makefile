#
# Makefile for the focaltech touchscreen drivers.
#

# Each configuration option enables a list of files.

ccflags-y += -I$(srctree)/drivers/input/touchscreen/focaltech_touch/firmware_config/$(CONFIG_TS_FIRMWARE)/
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST) += focaltech_upgrade_ft5x46.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST) += focaltech_upgrade_ft6336gu.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST) += focaltech_upgrade_ft8006.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST) += focaltech_upgrade_idc.o
obj-$(CONFIG_TOUCHSCREEN_FTS_TP_TEST) += focaltech_upgrade_test.o