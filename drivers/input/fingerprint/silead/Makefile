#
# by warren zhao
#

-include $(srctree)/drivers/misc/mediatek/Makefile.custom
SPI_DRV_MODE_SEL := PT
KBUILD_CFLAGS += -DGSL_DEV_SEL_device=spi_device -DGSL_DEV_SEL_driver=spi_driver -DGSL_DEV_SEL_dev_put=spi_dev_put -DGSL_DEV_SEL_set_drvdata=spi_set_drvdata -DGSL_DEV_SEL_get_drvdata=spi_get_drvdata -DGSL_DEV_SEL_register_driver=spi_register_driver -DGSL_DEV_SEL_unregister_driver=spi_unregister_driver

$(info thomas SPI_DRV_MODE_SEL=$(SPI_DRV_MODE_SEL) ...)

ifeq ($(SPI_DRV_MODE_SEL),PT)
KBUILD_CFLAGS += -DSLPT
KBUILD_CFLAGS += -UGSL_DEV_SEL_device -UGSL_DEV_SEL_driver -UGSL_DEV_SEL_dev_put -UGSL_DEV_SEL_set_drvdata -UGSL_DEV_SEL_get_drvdata -UGSL_DEV_SEL_driver -UGSL_DEV_SEL_register_driver -UGSL_DEV_SEL_unregister_driver -DGSL_DEV_SEL_device=platform_device -DGSL_DEV_SEL_driver=platform_driver -DGSL_DEV_SEL_dev_put=platform_dev_put -DGSL_DEV_SEL_set_drvdata=platform_set_drvdata -DGSL_DEV_SEL_get_drvdata=platform_get_drvdata -DGSL_DEV_SEL_register_driver=platform_register_driver -DGSL_DEV_SEL_unregister_driver=platform_unregister_driver -Dplatform_register_driver=platform_driver_register -Dplatform_unregister_driver=platform_driver_unregister -Dplatform_dev_put=platform_device_put
endif
ifeq ($(SL_PROJECT_NAME),android60B64TZ)
KBUILD_CFLAGS += -DMTK60B64
endif
ifeq ($(SL_PROJECT_NAME),android60B64TZWATCH)
KBUILD_CFLAGS += -DMTK60B64TZWATCH
endif
ifeq ($(SL_PROJECT_NAME),android-5.1)
KBUILD_CFLAGS += -DSPRD51B64
endif
ifeq ($(SL_PROJECT_NAME),android50B64)
KBUILD_CFLAGS += -DMTK50B64
endif
ifeq ($(SL_PROJECT_NAME),android50B64TZ)
KBUILD_CFLAGS += -DMTK50B64TZ
endif
ifeq ($(SL_PROJECT_NAME),android60B64MTK96board)
KBUILD_CFLAGS += -DMTK60X20
endif
ifeq ($(SL_PROJECT_NAME),android60B64TZWATCH-sprd)
KBUILD_CFLAGS += -DSPRD60B64TZWATCH
endif
ifeq ($(SL_PROJECT_NAME),android60B64TZ_BEANPOD)
KBUILD_CFLAGS += -DMTKBEANPOD60B64TZ
endif
ifeq ($(SL_PROJECT_NAME),android60B64_MT6797)
KBUILD_CFLAGS += -DMTK60B64MT6797
endif
ifeq ($(SL_PROJECT_NAME),android70B64TZ)
KBUILD_CFLAGS += -DMTK70B64TZ
endif

# flag for implement for kernel send fingerprint vkey code
KBUILD_CFLAGS += -DSL_KERNEL_KEYTYPE

ifneq ($(filter android60B64TZ android60B64TZWATCH android50B64 android50B64TZ,$(SL_PROJECT_NAME)),)
LINUXINCLUDE    += -I$(srctree)/drivers/misc/mediatek/mach/mt6795/include/mach -I$(srctree)/include/mach
else ifneq ($(filter android60B64TZ_BEANPOD android60B64_MT6797,$(SL_PROJECT_NAME)),)
LINUXINCLUDE    += -I$(srctree)/drivers/spi/mediatek/mt6797 -I$(srctree)/drivers/misc/mediatek/include -I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt6797/include
else ifneq ($(filter android70B64TZ,$(SL_PROJECT_NAME)),)
LINUXINCLUDE    += -I$(srctree)/drivers/spi/mediatek/mt6757 -I$(srctree)/drivers/misc/mediatek/include/mt-plat -I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt6757/include/mach
endif
obj-$(CONFIG_FINGERPRINT_SILEAD_6153E) += slspi.o slspi_board.o
obj-$(CONFIG_FINGERPRINT_SILEAD_6153E) += sl_proc.o
