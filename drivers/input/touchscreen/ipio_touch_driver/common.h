/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __COMMON_H
#define __COMMON_H

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/ctype.h>

#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/socket.h>
#include <net/sock.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>

#include <linux/namei.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>

#include <linux/gpio.h>
#include <linux/spi/spi.h>

#ifdef CONFIG_OF
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#else
#include <linux/earlysuspend.h>
#endif
#include "ipio_firmware_config.h"
/*
 * Relative Driver with Touch IC
 */

#define CHIP_ID_ERR	(-100)

/* A platform currently supported by driver */
#define PT_QCOM	1
#define PT_MTK	2
#define PT_SPRD	3
#define TP_PLATFORM PT_QCOM

/* A interface currently supported by driver */
#define I2C_INTERFACE 1
#define SPI_INTERFACE 2
#define INTERFACE I2C_INTERFACE

/* Driver version */
#define DRIVER_VERSION	"1.0.3.6"

/* Driver core type */
#define CORE_TYPE_B		0x00
#define CORE_TYPE_E		0x03

/* Protocol version */
#define PROTOCOL_MAJOR		0x5
#define PROTOCOL_MID		0x4
#define PROTOCOL_MINOR		0x0

/*  Debug messages */
#ifdef BIT
#undef BIT
#endif
#define BIT(x)	(1 << (x))

enum {
	DEBUG_NONE = 0,
	DEBUG_IRQ = BIT(0),
	DEBUG_FINGER_REPORT = BIT(1),
	DEBUG_FIRMWARE = BIT(2),
	DEBUG_CONFIG = BIT(3),
	DEBUG_I2C = BIT(4),
	DEBUG_BATTERY = BIT(5),
	DEBUG_MP_TEST = BIT(6),
	DEBUG_IOCTL = BIT(7),
	DEBUG_NETLINK = BIT(8),
	DEBUG_PARSER = BIT(9),
	DEBUG_GESTURE = BIT(10),
	DEBUG_ESD = BIT(11),
	DEBUG_ALL = ~0,
};

#define ipio_info(fmt, arg...)	\
	pr_info("ILITEK: (%s, %d): " fmt, __func__, __LINE__, ##arg);

#define ipio_err(fmt, arg...)	\
	pr_err("ILITEK: (%s, %d): " fmt, __func__, __LINE__, ##arg);

#define ipio_debug(level, fmt, arg...) \
	do { \
		if (level & ipio_debug_level) \
		pr_info("ILITEK: (%s, %d): " fmt, __func__, __LINE__, ##arg);	\
	} while (0)

/* Distributed to all core functions */
extern uint32_t ipio_debug_level;
extern uint32_t ipio_chip_list[2];

/* Macros */
#define CHECK_EQUAL(X, Y) ((X == Y) ? 0 : -1)
#define ERR_ALLOC_MEM(X)	((IS_ERR(X) || X == NULL) ? 1 : 0)
#define MIN(a, b) (((a) < (b))?(a):(b))
#define MAX(a, b) (((a) > (b))?(a):(b))
#define USEC	1
#define MSEC	(USEC * 1000)

/* The size of firmware upgrade */
#define MAX_HEX_FILE_SIZE			(160*1024)
#define MAX_FLASH_FIRMWARE_SIZE		(256*1024)
#define MAX_IRAM_FIRMWARE_SIZE		(60*1024)
#define ILI_FILE_HEADER				64
#define MAX_AP_FIRMWARE_SIZE		(64*1024)
#define MAX_DLM_FIRMWARE_SIZE		(8*1024)
#define MAX_MP_FIRMWARE_SIZE		(64*1024)
#define MAX_GESTURE_FIRMWARE_SIZE	(8*1024)
#define DLM_START_ADDRESS           0x20610
#define DLM_HEX_ADDRESS             0x10000
#define MP_HEX_ADDRESS              0x13000
#define SPI_UPGRADE_LEN		        2048
#define UPDATE_RETRY_COUNT          3

/* ILI9881 Series */
enum ili9881_types {
	ILI9881_TYPE_F = 0x0F,
	ILI9881_TYPE_H = 0x11
};

/*Error code*/
#define SUCCESS					0
#define UPDATE_FAIL				-1
#define CHECK_RECOVER			-2
#define I2C_ESD_CHECK			-5

#define ILI9881_SLAVE_ADDR		0x41
#define ILI9881_ICE_MODE_ADDR	0x181062
#define ILI9881_PID_ADDR		0x4009C
#define ILI9881_WDT_ADDR		0x5100C

/*
 * Other settings
 */
#define INI_FILE_PATH		"/sdcard/"
#define CSV_FILE_PATH		"/sdcard/"
#define INI_NAME_PATH		"mp.ini"
#define UPDATE_FW_PATH		"/mnt/sdcard/ILITEK_FW"
#define POWER_STATUS_PATH	"/sys/class/power_supply/battery/status"
#define PROC_DIR_NAME	"ilitek"
#define PROC_DIR_NAME_CUSTOM	"touchscreen"
#define CHECK_BATTERY_TIME  2000
#define CHECK_ESD_TIME		4000
#define VDD_VOLTAGE			2800000
#define VDD_I2C_VOLTAGE		1800000

/* It's only for spi interface used to download data to iram */
#if (INTERFACE == SPI_INTERFACE)
#define HOST_DOWNLOAD
#endif

/* Linux multiple touch protocol, either B type or A type. */
#define MT_B_TYPE

/* Enable the support of regulator power. */
#define REGULATOR_POWER_ON

/* Either an interrupt event handled by kthread or work queue. */
/* #define USE_KTHREAD */

/* Enable DMA with I2C. */
/* #define I2C_DMA */

/* Split the length written to or read from IC via I2C. */
/* #define I2C_SEGMENT */

/* debug buffer size */
#define ILITEK_DEBUG_BUFF_ROW_SIZE 1024
#define ILITEK_DEBUG_BUFF_RANK_SIZE 2048

/* rawdata error */
#define TEST_BEYOND_MAX_LIMIT		0x0001
#define TEST_BEYOND_MIN_LIMIT		0x0002
#define TEST_BEYOND_ACCORD_LIMIT	0x0004
#define TEST_BEYOND_OFFSET_LIMIT	0x0008
#define TEST_BEYOND_JITTER_LIMIT	0x0010
#define TEST_KEY_BEYOND_MAX_LIMIT	0x0020
#define TEST_KEY_BEYOND_MIN_LIMIT	0x0040
#define TEST_MODULE_TYPE_ERR		0x0080
#define TEST_VERSION_ERR			0x0100
#define TEST_GT_OPEN				0x0200
#define TEST_GT_SHORT				0x0400
#define TEST_BEYOND_UNIFORMITY_LIMIT 0x0800
#define TEST_BEYOND_FLASH_LAYER_LIMIT  0x1000
#define TEST_BETWEEN_ACCORD_AND_LINE  0x2000

static inline void ipio_kfree(void **mem)
{
	if (*mem != NULL) {
		kfree(*mem);
		*mem = NULL;
	}
}

static inline void ipio_msleep(unsigned int ms)
{
	unsigned long us = 0;

	if (ms >= 20) {
		msleep(ms);
	} else {
		us = 1000 * (unsigned long)ms;
		usleep_range(us, us+10);
	}
}

extern int ilitek_vendor_id;
extern int katoi(char *string);
extern int str2hex(char *str);
#endif /* __COMMON_H */
