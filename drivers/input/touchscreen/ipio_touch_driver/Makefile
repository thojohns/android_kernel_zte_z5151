# include header files for MTK
ccflags-y += -I$(srctree)/drivers/input/touchscreen/ipio_touch_driver/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/ipio_touch_driver/$(CONFIG_ILI_FIRMWARE_PATH)/

obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  platform.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  userspace.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  core/

