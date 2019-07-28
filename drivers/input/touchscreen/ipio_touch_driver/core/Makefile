# include header files
ccflags-y += -I$(srctree)/drivers/input/touchscreen/ipio_touch_driver/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/ipio_touch_driver/$(CONFIG_ILI_FIRMWARE_PATH)/

obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  config.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  i2c.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  firmware.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  finger_report.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  mp_test.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  flash.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  protocol.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  parser.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  gesture.o
obj-$(CONFIG_TOUCHSCREEN_ILITEK)	+=  spi.o
