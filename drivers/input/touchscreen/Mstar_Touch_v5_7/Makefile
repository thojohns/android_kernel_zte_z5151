#
# Makefile for the Mstar MSG2xxx touchscreen driver.
#

# Each configuration option enables a list of files.
ccflags-y += -I$(srctree)/drivers/input/touchscreen/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/Mstar_Touch_v5_7/firmware_config/$(CONFIG_TS_FIRMWARE)/
obj-y	+=  mstar_drv_common.o
obj-y	+=  mstar_drv_fw_control.o

obj-y	+=  mstar_drv_ic_fw_porting_layer.o
obj-y	+=  mstar_drv_jni_interface.o
obj-y	+=  mstar_drv_main.o
obj-y	+= mstar_drv_qcom.o
obj-y	+=  mstar_drv_platform_interface.o
obj-y	+=  mstar_drv_platform_porting_layer.o
obj-y	+=  mstar_drv_utility_adaption.o
obj-y	+=  mp_test/
