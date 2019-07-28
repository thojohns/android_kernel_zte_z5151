# Makefile for the Himax touchscreen drivers.

ccflags-y += -I$(srctree)/drivers/input/touchscreen/HiMax83102/$(CONFIG_HXTS_FIRMWARE_PATH)/
obj-$(CONFIG_TOUCHSCREEN_HIMAX_I2C_HX83102)   	+= himax_platform.o himax_ic.o himax_common.o himax_debug.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)   	+= 	himax_ito_test.o
