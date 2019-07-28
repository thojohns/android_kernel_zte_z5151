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
#include "common.h"
#include "core/config.h"
#include "core/i2c.h"
#include "core/spi.h"
#include "core/firmware.h"
#include "core/finger_report.h"
#include "core/flash.h"
#include "core/protocol.h"
#include "platform.h"
#include "core/mp_test.h"
#include "core/gesture.h"

#include "../tpd_sys.h"
#ifdef BATTERY_CHECK
#include <linux/power_supply.h>
#endif

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"

#if (TP_PLATFORM == PT_MTK)
#define DTS_OF_NAME		"mediatek,cap_touch"
#include "tpd.h"
extern struct tpd_device *tpd;
#define MTK_RST_GPIO GTP_RST_PORT
#define MTK_INT_GPIO GTP_INT_PORT
#else
#define DTS_OF_NAME		"tchip,ilitek"
#endif /* PT_MTK */

#define DEVICE_ID	"ILITEK_TDDI"

#ifdef USE_KTHREAD
static DECLARE_WAIT_QUEUE_HEAD(waiter);
#endif

/* Debug level */
uint32_t ipio_debug_level = DEBUG_NONE;
EXPORT_SYMBOL(ipio_debug_level);

struct ilitek_platform_data *ipd = NULL;

extern int mp_path_init(void);

void ilitek_platform_disable_irq(void)
{
	unsigned long nIrqFlag;

	ipio_debug(DEBUG_IRQ, "IRQ = %d\n", ipd->isEnableIRQ);

	spin_lock_irqsave(&ipd->plat_spinlock, nIrqFlag);

	if (ipd->isEnableIRQ) {
		if (ipd->isr_gpio) {
			disable_irq_nosync(ipd->isr_gpio);
			ipd->isEnableIRQ = false;
			ipio_debug(DEBUG_IRQ, "Disable IRQ: %d\n", ipd->isEnableIRQ);
		} else
			ipio_err("The number of gpio to irq is incorrect\n");
	} else
		ipio_debug(DEBUG_IRQ, "IRQ was already disabled\n");

	spin_unlock_irqrestore(&ipd->plat_spinlock, nIrqFlag);
}
EXPORT_SYMBOL(ilitek_platform_disable_irq);

void ilitek_platform_enable_irq(void)
{
	unsigned long nIrqFlag;

	ipio_debug(DEBUG_IRQ, "IRQ = %d\n", ipd->isEnableIRQ);

	spin_lock_irqsave(&ipd->plat_spinlock, nIrqFlag);

	if (!ipd->isEnableIRQ) {
		if (ipd->isr_gpio) {
			enable_irq(ipd->isr_gpio);
			ipd->isEnableIRQ = true;
			ipio_debug(DEBUG_IRQ, "Enable IRQ: %d\n", ipd->isEnableIRQ);
		} else
			ipio_err("The number of gpio to irq is incorrect\n");
	} else
		ipio_debug(DEBUG_IRQ, "IRQ was already enabled\n");

	spin_unlock_irqrestore(&ipd->plat_spinlock, nIrqFlag);
}
EXPORT_SYMBOL(ilitek_platform_enable_irq);

int ilitek_platform_tp_hw_reset(bool isEnable)
{
	int ret = 0;
	ipio_info("HW Reset: %d\n", isEnable);

	ilitek_platform_disable_irq();

	if (isEnable) {
#if (TP_PLATFORM == PT_MTK)
		tpd_gpio_output(ipd->reset_gpio, 1);
		ipio_msleep(ipd->delay_time_high);
		tpd_gpio_output(ipd->reset_gpio, 0);
		ipio_msleep(ipd->delay_time_low);
		tpd_gpio_output(ipd->reset_gpio, 1);
		ipio_msleep(ipd->edge_delay);
#else
		gpio_direction_output(ipd->reset_gpio, 1);
		ipio_msleep(ipd->delay_time_high);
		gpio_set_value(ipd->reset_gpio, 0);
		ipio_msleep(ipd->delay_time_low);
		gpio_set_value(ipd->reset_gpio, 1);
		ipio_msleep(ipd->edge_delay);
#endif /* PT_MTK */
	} else {
#if (TP_PLATFORM == PT_MTK)
		tpd_gpio_output(ipd->reset_gpio, 0);
#else
		gpio_set_value(ipd->reset_gpio, 0);
#endif /* PT_MTK */
	}

#ifdef HOST_DOWNLOAD
	core_config_ice_mode_enable();
	ret = core_firmware_upgrade(UPDATE_FW_PATH, true);
	if (ret < 0)
		ipio_err("host download failed!\n");
#endif
	ipio_msleep(10);
	ilitek_platform_enable_irq();
	return ret;
}
EXPORT_SYMBOL(ilitek_platform_tp_hw_reset);

#ifdef REGULATOR_POWER_ON
void ilitek_regulator_power_on(bool status)
{
	int res = 0;

	ipio_info("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (ipd->vdd) {
			res = regulator_enable(ipd->vdd);
			if (res < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			res = regulator_enable(ipd->vdd_i2c);
			if (res < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	} else {
		if (ipd->vdd) {
			res = regulator_disable(ipd->vdd);
			if (res < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			res = regulator_disable(ipd->vdd_i2c);
			if (res < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	}

	/* NULL POINT in this */
	/* core_config->icemodeenable = false; */
	ipio_msleep(5);
}
EXPORT_SYMBOL(ilitek_regulator_power_on);

static void ilitek_regulator_release(void)
{
	if (ipd == NULL)
		return;

	if (ipd->vdd) {
		regulator_put(ipd->vdd);
	}

	if (ipd->vdd_i2c) {
		regulator_put(ipd->vdd_i2c);
	}

	return;
}
#endif /* REGULATOR_POWER_ON */

#ifdef BATTERY_CHECK
uint32_t ilitek_get_charger_status(void)
{
	union power_supply_propval ret = {0, };
	static struct power_supply *batt_psy;

	if (batt_psy == NULL) {
		batt_psy = power_supply_get_by_name("battery");
	}
	if (batt_psy) {
		/* if battery has been registered, use the status property */
		batt_psy->get_property(batt_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
		return ret.intval;
	}

	/* Default to false if the battery power supply is not registered. */
	ipio_debug(DEBUG_BATTERY, "battery power supply is not registered\n");
	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static void ilitek_platform_vpower_notify(struct work_struct *pWork)
{
	uint32_t charger_status = POWER_SUPPLY_STATUS_UNKNOWN;
	static int charge_mode = 0;

	ipio_debug(DEBUG_BATTERY, "isEnableCheckPower = %d\n", ipd->isEnablePollCheckPower);
	charger_status = ilitek_get_charger_status();
	ipio_debug(DEBUG_BATTERY, "Batter Status: %d\n", charger_status);

	if (charger_status == POWER_SUPPLY_STATUS_CHARGING
		|| charger_status == POWER_SUPPLY_STATUS_FULL) {
		if (charge_mode != 1) {
			ipio_debug(DEBUG_BATTERY, "Charging mode\n");
			core_config_plug_ctrl(false);
			charge_mode = 1;
		}
	} else {
		if (charge_mode != 2) {
			ipio_debug(DEBUG_BATTERY, "Not charging mode\n");
			core_config_plug_ctrl(true);
			charge_mode = 2;
		}
	}

	if (ipd->isEnablePollCheckPower)
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
}
#endif /* BATTERY_CHECK */

#ifdef ESD_CHECK
static void ilitek_platform_esd_recovery(struct work_struct *work)
{
	int ret = 0;

	mutex_lock(&ipd->plat_mutex);
	ret = ilitek_platform_tp_hw_reset(true);
	if (ret < 0)
		ipio_err("host download failed!\n");
	mutex_unlock(&ipd->plat_mutex);
}

static void ilitek_platform_esd_check(struct work_struct *pWork)
{
#if (INTERFACE == SPI_INTERFACE)
	uint8_t tx_data = 0x82, rx_data = 0;

	ipio_debug(DEBUG_ESD, "isEnablePollCheckEsd = %d\n", ipd->isEnablePollCheckEsd);
	if (spi_write_then_read(core_spi->spi, &tx_data, 1, &rx_data, 1) < 0) {
		ipio_err("spi Write Error\n");
	}

	if (rx_data == 0x82) {
		ipio_info("Doing ESD recovery (0x%x)\n", rx_data);
		schedule_work(&ipd->esd_recovery);
	} else {
		if (ipd->isEnablePollCheckEsd)
			queue_delayed_work(ipd->check_esd_status_queue,
				&ipd->check_esd_status_work, ipd->esd_check_time);
	}
#else  /* IIC_INTERFACE */
	uint8_t read_retry_cnt = 0;

	ipio_debug(DEBUG_ESD, "IIC isEnablePollCheckEsd = %d\n", ipd->isEnablePollCheckEsd);
	while (read_retry_cnt < 3) {
		if (core_config_get_fw_ver() < 0) {
			read_retry_cnt++;
		       ipio_err("Failed to get firmware version cnt %d\n", read_retry_cnt);
	       }
		else {
			goto ilitek_esd_check_out;
		}
	}

	ipio_err("Doing ESD recovery\n");
	schedule_work(&ipd->esd_recovery);

ilitek_esd_check_out:
	if (ipd->isEnablePollCheckEsd)
		queue_delayed_work(ipd->check_esd_status_queue,
			&ipd->check_esd_status_work, ipd->esd_check_time);

	ipio_debug(DEBUG_ESD, "ilitek esd %s: out.......\n", __func__);
#endif
}
#endif /* ESD_CHECK */

#if (TP_PLATFORM == PT_MTK)
static void tpd_resume(struct device *h)
{
	ipio_info("TP Resume\n");

	if (!core_firmware->isUpgrading) {
		core_config_ic_resume();
	}
}

static void tpd_suspend(struct device *h)
{
	ipio_info("TP Suspend\n");

	if (!core_firmware->isUpgrading) {
		core_config_ic_suspend();
	}
}
#elif defined CONFIG_FB
static int ilitek_platform_notifier_fb(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
	struct fb_event *evdata = data;

	ipio_info("Notifier's event = %ld\n", event);

	/*
	 *  FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
	 *  FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change occurred.
	 */
	if (evdata && evdata->data && (event == FB_EVENT_BLANK)) {
		blank = evdata->data;

#if (TP_PLATFORM == PT_SPRD)
		if (*blank == DRM_MODE_DPMS_OFF)
#else
		if (*blank == FB_BLANK_POWERDOWN)
#endif /* PT_SPRD */
		{
			ipio_info("TP Suspend\n");

			if (!core_firmware->isUpgrading) {
				core_config_ic_suspend();
				core_config->isSuspend = true;
			}
			ipio_info("TP Suspend done\n");
		}
#if (TP_PLATFORM == PT_SPRD)
		else if (*blank == DRM_MODE_DPMS_ON)
#else
		else if (*blank == FB_BLANK_UNBLANK || *blank == FB_BLANK_NORMAL)
#endif /* PT_SPRD */
		{
			ipio_info("TP Resume\n");

			if (!core_firmware->isUpgrading) {
				core_config_ic_resume();
				core_config->isSuspend = false;
			}
			ipio_info("TP Resume done\n");
		}
	}

	return NOTIFY_OK;
}
#else /* CONFIG_HAS_EARLYSUSPEND */
static void ilitek_platform_early_suspend(struct early_suspend *h)
{
	ipio_info("TP Suspend\n");

	/* TODO: there is doing nothing if an upgrade firmware's processing. */

	core_fr_touch_release(0, 0, 0);

	input_sync(core_fr->input_device);

	core_fr->isEnableFR = false;

	core_config_ic_suspend();
}

static void ilitek_platform_late_resume(struct early_suspend *h)
{
	ipio_info("TP Resume\n");

	core_fr->isEnableFR = true;
	core_config_ic_resume();
}
#endif /* PT_MTK */

/**
 * reg_power_check - register a thread to inquery status at certain time.
 */
static int ilitek_platform_reg_power_check(void)
{
	int res = 0;

#ifdef BATTERY_CHECK
	INIT_DELAYED_WORK(&ipd->check_power_status_work, ilitek_platform_vpower_notify);
	ipd->check_power_status_queue = create_workqueue("ili_power_check");
	ipd->work_delay = msecs_to_jiffies(CHECK_BATTERY_TIME);
	ipd->isEnablePollCheckPower = true;
	if (!ipd->check_power_status_queue) {
		ipio_err("Failed to create a work thread to check power status\n");
		ipd->vpower_reg_nb = false;
		res = -1;
	} else {
		ipio_info("Created a work thread to check power status at every %u jiffies\n",
			 (unsigned)ipd->work_delay);

		if (ipd->isEnablePollCheckPower) {
			queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work,
					   ipd->work_delay);
			ipd->vpower_reg_nb = true;
		}
	}
#endif /* BATTERY_CHECK */

	return res;
}

static int ilitek_platform_reg_esd_check(void)
{
	int res = 0;

#ifdef ESD_CHECK
	INIT_DELAYED_WORK(&ipd->check_esd_status_work, ilitek_platform_esd_check);
	ipd->check_esd_status_queue = create_workqueue("ili_esd_check");
	ipd->esd_check_time = msecs_to_jiffies(CHECK_ESD_TIME);
	ipd->isEnablePollCheckEsd = true;
	if (!ipd->check_esd_status_queue) {
		ipio_err("Failed to create a work thread to check power status\n");
		ipd->vesd_reg_nb = false;
		res = -1;
	} else {
		ipio_info("Created a work thread to check power status at every %u jiffies\n",
			 (unsigned)ipd->esd_check_time);

		INIT_WORK(&ipd->esd_recovery, ilitek_platform_esd_recovery);

		if (ipd->isEnablePollCheckEsd) {
			queue_delayed_work(ipd->check_esd_status_queue, &ipd->check_esd_status_work,
					   ipd->esd_check_time);
			ipd->vesd_reg_nb = true;
		}
	}
#endif /* ESD_CHECK */

	return res;
}
/**
 * Register a callback function when the event of suspend and resume occurs.
 *
 * The default used to wake up the cb function comes from notifier block mechnaism.
 * If you'd rather liek to use early suspend, CONFIG_HAS_EARLYSUSPEND in kernel config
 * must be enabled.
 */
static int ilitek_platform_reg_suspend(void)
{
	int res = 0;

#if (TP_PLATFORM == PT_MTK)
	ipio_info("It does nothing if platform is MTK\n");
#else
	ipio_info("Register suspend/resume callback function\n");
#ifdef CONFIG_FB
	ipd->notifier_fb.notifier_call = ilitek_platform_notifier_fb;
#if (TP_PLATFORM == PT_SPRD)
	res = adf_register_client(&ipd->notifier_fb);
#else
	res = fb_register_client(&ipd->notifier_fb);
#endif /* PT_SPRD */
#else
	ipd->early_suspend->suspend = ilitek_platform_early_suspend;
	ipd->early_suspend->esume = ilitek_platform_late_resume;
	ipd->early_suspend->level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	res = register_early_suspend(ipd->early_suspend);
#endif /* CONFIG_FB */
#endif /* PT_MTK */

	return res;
}

#ifndef USE_KTHREAD
static void ilitek_platform_work_queue(struct work_struct *work)
{
	ipio_debug(DEBUG_IRQ, "work_queue: IRQ = %d\n", ipd->isEnableIRQ);

	core_fr_handler();

	if (!ipd->isEnableIRQ)
		ilitek_platform_enable_irq();
}
#endif /* USE_KTHREAD */

static irqreturn_t ilitek_platform_irq_handler(int irq, void *dev_id)
{
	ipio_debug(DEBUG_IRQ, "IRQ = %d\n", ipd->isEnableIRQ);

	if (ipd->isEnableIRQ) {
		ilitek_platform_disable_irq();
#ifdef USE_KTHREAD
		ipd->irq_trigger = true;
		wake_up_interruptible(&waiter);
#else
		schedule_work(&ipd->report_work_queue);
#endif /* USE_KTHREAD */
	}

	return IRQ_HANDLED;
}

static int ilitek_platform_input_init(void)
{
	int res = 0;

#if (TP_PLATFORM == PT_MTK)
	int i;

	ipd->input_device = tpd->dev;

	if (tpd_dts_data.use_tpd_button) {
		for (i = 0; i < tpd_dts_data.tpd_key_num; i++) {
			input_set_capability(ipd->input_device, EV_KEY, tpd_dts_data.tpd_key_local[i]);
		}
	}
	core_fr_input_set_param(ipd->input_device);
	return res;
#else
	ipd->input_device = input_allocate_device();
	if (ERR_ALLOC_MEM(ipd->input_device)) {
		ipio_err("Failed to allocate touch input device\n");
		res = -ENOMEM;
		goto fail_alloc;
	}

#if (INTERFACE == I2C_INTERFACE)
	ipd->input_device->name = ipd->client->name;
	ipd->input_device->phys = "I2C";
	ipd->input_device->dev.parent = &ipd->client->dev;
	ipd->input_device->id.bustype = BUS_I2C;
#else
	ipd->input_device->name = DEVICE_ID;
	ipd->input_device->phys = "SPI";
	ipd->input_device->dev.parent = &ipd->spi->dev;
	ipd->input_device->id.bustype = BUS_SPI;
#endif

	core_fr_input_set_param(ipd->input_device);

	/* register the input device to input sub-system */
	res = input_register_device(ipd->input_device);
	if (res < 0) {
		ipio_err("Failed to register touch input device, res = %d\n", res);
		goto err_register_input;
	}

	return res;

err_register_input:
	input_free_device(ipd->input_device);
fail_alloc:
	return res;
#endif /* PT_MTK */
}

#if defined(BOOT_FW_UPGRADE) || defined(USE_KTHREAD)
static int kthread_handler(void *arg)
{
	int res = 0;
	char *str = (char *)arg;

	if (strcmp(str, "boot_fw") == 0) {
		/* FW Upgrade event */
		core_firmware->isboot = true;

		ilitek_platform_disable_irq();

#ifdef HOST_DOWNLOAD
		res = core_firmware_boot_host_download();
#else
#ifdef BOOT_FW_UPGRADE
		res = core_firmware_boot_upgrade();
#endif /* BOOT_FW_UPGRADE */
#endif /* HOST_DOWNLOAD */
		if (res < 0)
			ipio_err("Failed to upgrade FW at boot stage\n");

		ilitek_platform_enable_irq();

		core_firmware->isboot = false;
	}
#ifdef USE_KTHREAD
	else if (strcmp(str, "irq") == 0) {
		/* IRQ event */
		struct sched_param param = {.sched_priority = 4 };

		sched_setscheduler(current, SCHED_RR, &param);

		while (!kthread_should_stop() && !ipd->free_irq_thread) {
			ipio_debug(DEBUG_IRQ, "kthread: before->irq_trigger = %d\n", ipd->irq_trigger);
			set_current_state(TASK_INTERRUPTIBLE);
			wait_event_interruptible(waiter, ipd->irq_trigger);
			ipd->irq_trigger = false;
			set_current_state(TASK_RUNNING);
			ipio_debug(DEBUG_IRQ, "kthread: after->irq_trigger = %d\n", ipd->irq_trigger);
			core_fr_handler();
			ilitek_platform_enable_irq();
		}
	}
#endif
	else {
		ipio_err("Unknown EVENT\n");
	}

	return res;
}
#endif

static int ilitek_platform_isr_register(void)
{
	int res = 0;
#if (TP_PLATFORM == PT_MTK)
	struct device_node *node;
#endif /* PT_MTK */

#ifdef USE_KTHREAD
	ipd->irq_thread = kthread_run(kthread_handler, "irq", "ili_irq_thread");
	if (IS_ERR(ipd->irq_thread)) {
		ipd->irq_thread = NULL;
		ipio_err("Failed to create kthread\n");
		res = -ENOMEM;
		goto out;
	}
	ipd->irq_trigger = false;
	ipd->free_irq_thread = false;
#else
	INIT_WORK(&ipd->report_work_queue, ilitek_platform_work_queue);
#endif /* USE_KTHREAD */

#if (TP_PLATFORM == PT_MTK)
	node = of_find_matching_node(NULL, touch_of_match);
	if (node) {
		ipd->isr_gpio = irq_of_parse_and_map(node, 0);
	}
#else
	ipd->isr_gpio = gpio_to_irq(ipd->int_gpio);
#endif /* PT_MTK */

	ipio_info("ipd->isr_gpio = %d\n", ipd->isr_gpio);

	res = request_threaded_irq(ipd->isr_gpio,
				   NULL,
				   ilitek_platform_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ilitek", NULL);
	if (res != 0) {
		ipio_err("Failed to register irq handler, irq = %d, res = %d\n", ipd->isr_gpio, res);
		goto out;
	}

	ipd->isEnableIRQ = true;

out:
	return res;
}

static int ilitek_platform_gpio(void)
{
	int res = 0;
#if (TP_PLATFORM == PT_MTK)
	ipd->int_gpio = MTK_INT_GPIO;
	ipd->reset_gpio = MTK_RST_GPIO;
#else
#ifdef CONFIG_OF
#if (INTERFACE == I2C_INTERFACE)
	struct device_node *dev_node = ipd->client->dev.of_node;
#else
	struct device_node *dev_node = ipd->spi->dev.of_node;
#endif
	uint32_t flag;

	ipd->int_gpio = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	ipd->reset_gpio = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);
#endif /* CONFIG_OF */
#endif /* PT_MTK */

	ipio_info("GPIO INT: %d\n", ipd->int_gpio);
	ipio_info("GPIO RESET: %d\n", ipd->reset_gpio);

	if (!gpio_is_valid(ipd->int_gpio)) {
		ipio_err("Invalid INT gpio: %d\n", ipd->int_gpio);
		return -EBADR;
	}

	if (!gpio_is_valid(ipd->reset_gpio)) {
		ipio_err("Invalid RESET gpio: %d\n", ipd->reset_gpio);
		return -EBADR;
	}

	res = gpio_request(ipd->int_gpio, "ILITEK_TP_IRQ");
	if (res < 0) {
		ipio_err("Request IRQ GPIO failed, res = %d\n", res);
		goto int_gpio_request_failed;
	}

	res = gpio_request(ipd->reset_gpio, "ILITEK_TP_RESET");
	if (res < 0) {
		ipio_err("Request RESET GPIO failed, res = %d\n", res);
		goto reset_gpio_request_failed;
	}

	gpio_direction_input(ipd->int_gpio);

	return res;

reset_gpio_request_failed:
	if (gpio_is_valid(ipd->int_gpio)) {
		gpio_free(ipd->int_gpio);
	}
int_gpio_request_failed:
	return res;
}

static int ilitek_platform_set_vendor_id_gpio(void)
{
	struct device_node *dev_node = ipd->client->dev.of_node;
	int ret = 0;

	ipd->gpio_id0 = of_get_named_gpio(dev_node, "touch,id0-gpio", 0);
	if (!gpio_is_valid(ipd->gpio_id0)) {
		ipio_err("gpio_id0 value is not valid\n");
		goto id_gpio_get_failed;
	}
	ipd->gpio_id1 = of_get_named_gpio(dev_node, "touch,id1-gpio", 0);
	if (!gpio_is_valid(ipd->gpio_id1)) {
		ipio_err("gpio_id1 value is not valid\n");
		goto id_gpio_get_failed;
	}
	ipio_info("id0=%d, id1=%d", ipd->gpio_id0, ipd->gpio_id1);

	/* configure touchscreen id0 input gpio */
	ret = gpio_request(ipd->gpio_id0, "ilitek_id0");
	if (ret) {
		ipio_err("unable to request gpio [%d]\n", ipd->gpio_id0);
		goto err_request_id0;
	}

	ret = gpio_direction_input(ipd->gpio_id0);
	if (ret) {
		ipio_err("unable to set direction for gpio [%d]\n", ipd->gpio_id0);
		goto err_set_direction_id0;
	}

	/* configure touchscreen id1 input gpio */
	ret = gpio_request(ipd->gpio_id1, "ilitek_id1");
	if (ret) {
		ipio_err("unable to request gpio [%d]\n", ipd->gpio_id1);
		goto err_request_id1;
	}

	ret = gpio_direction_input(ipd->gpio_id1);
	if (ret) {
		ipio_err("unable to set direction for gpio [%d]\n", ipd->gpio_id1);
		goto err_set_direction_id1;
	}

	return ret;

err_set_direction_id1:
	gpio_free(ipd->gpio_id1);
err_request_id1:
err_set_direction_id0:
	gpio_free(ipd->gpio_id0);
err_request_id0:
id_gpio_get_failed:
	return ret;
}

int ilitek_vendor_id = 0;
static int ilitek_platform_get_vendor_id(void)
{
	int tp_id0 = 0;
	int tp_id1 = 0;
	int id = 0;

	/* identify vendor id */
	if (gpio_is_valid(ipd->gpio_id0) && gpio_is_valid(ipd->gpio_id1)) {
		tp_id0 = gpio_get_value(ipd->gpio_id0);
		tp_id1 = gpio_get_value(ipd->gpio_id1);
		ipio_info("id0=%d, id1=%d\n", tp_id0, tp_id1);
	}

	if ((tp_id0 == 0) && (tp_id1 == 0)) {
		id = ILITEK_VENDOR_ID_0;
	} else if ((tp_id0 == 1) && (tp_id1 == 0)) {
		id = ILITEK_VENDOR_ID_1;
	} else if ((tp_id0 == 0) && (tp_id1 == 1)) {
		id = ILITEK_VENDOR_ID_2;
	} else  if ((tp_id0 == 1) && (tp_id1 == 1)) {
		id = ILITEK_VENDOR_ID_3;
	} else {
		id = ILITEK_VENDOR_ID_0;
	}

	return id;
}

static int ilitek_platform_read_tp_info(void)
{
	if (core_config_get_chip_id() < 0) {
		ipio_err("Failed to get chip id\n");
		return CHIP_ID_ERR;
	}

	if (core_config_get_protocol_ver() < 0) {
		ipio_err("Failed to get protocol version\n");
		return -EPERM;
	}

	if (core_config_get_fw_ver() < 0) {
		ipio_err("Failed to get firmware version\n");
		return -EPERM;
	}

	if (core_config_get_core_ver() < 0) {
		ipio_err("Failed to get core version\n");
		return -EPERM;
	}

	if (core_config_get_tp_info() < 0) {
		ipio_err("Failed to get TP information\n");
		return -EPERM;
	}

	if (core_config_get_key_info() < 0) {
		ipio_err("Failed to get key information\n");
		return -EPERM;
	}

	return 0;
}

/**
 * Remove Core APIs memeory being allocated.
 */
static void ilitek_platform_core_remove(void)
{
	ipio_info("Remove all core's compoenets\n");
	ilitek_proc_remove();
	core_flash_remove();
	core_firmware_remove();
	core_fr_remove();
	core_config_remove();
	core_i2c_remove();
	core_protocol_remove();
	core_gesture_remove();
}

/**
 * The function is to initialise all necessary structurs in those core APIs,
 * they must be called before the i2c dev probes up successfully.
 */
static int ilitek_platform_core_init(void)
{
	ipio_info("Initialise core's components\n");

	if (core_config_init() < 0 || core_protocol_init() < 0 ||
		core_firmware_init() < 0 || core_fr_init() < 0 ||
		core_gesture_init() < 0) {
		ipio_err("Failed to initialise core components\n");
		return -EINVAL;
	}

#if (INTERFACE == I2C_INTERFACE)
	if (core_i2c_init(ipd->client) < 0)
#else
	if (core_spi_init(ipd->spi) < 0)
#endif
	{
		ipio_err("Failed to initialise interface\n");
		return -EINVAL;
	}
	return 0;
}

#define ILITEK_PINCTRL_INIT_STATE "pmx_ts_init"
static int ilitek_platform_pinctrl_init(struct ilitek_platform_data *ipd)
{
	int ret;

	/* Get pinctrl if target uses pinctrl */
	ipd->ts_pinctrl = devm_pinctrl_get(&(ipd->client->dev));
	if (IS_ERR_OR_NULL(ipd->ts_pinctrl)) {
		ret = PTR_ERR(ipd->ts_pinctrl);
		ipio_err("Target does not use pinctrl %d\n", ret);
		goto err_pinctrl_get;
	}

	ipd->pinctrl_state_init
	    = pinctrl_lookup_state(ipd->ts_pinctrl, ILITEK_PINCTRL_INIT_STATE);
	if (IS_ERR_OR_NULL(ipd->pinctrl_state_init)) {
		ret = PTR_ERR(ipd->pinctrl_state_init);
		ipio_err("Can not lookup %s pinstate %d\n", ILITEK_PINCTRL_INIT_STATE, ret);
		goto err_pinctrl_lookup;
	}

	ret = pinctrl_select_state(ipd->ts_pinctrl, ipd->pinctrl_state_init);
	if (ret < 0) {
		ipio_err("failed to select pin to active state");
		goto err_select_init_state;
	}

	return 0;

err_select_init_state:
err_pinctrl_lookup:
	devm_pinctrl_put(ipd->ts_pinctrl);
err_pinctrl_get:
	ipd->ts_pinctrl = NULL;
	return ret;
}

#if (INTERFACE == I2C_INTERFACE)
static int ilitek_platform_remove(struct i2c_client *client)
#else
static int ilitek_platform_remove(struct spi_device *spi)
#endif
{
	ipio_info("Remove platform components\n");

#ifdef CONFIG_FB
	fb_unregister_client(&ipd->notifier_fb);
#else
	unregister_early_suspend(&ipd->early_suspend);
#endif /* CONFIG_FB */

#ifdef USE_KTHREAD
	if (ipd->irq_thread != NULL) {
		ipd->irq_trigger = true;
		ipd->free_irq_thread = true;
		wake_up_interruptible(&waiter);
		kthread_stop(ipd->irq_thread);
		ipd->irq_thread = NULL;
	}
#endif /* USE_KTHREAD */

	if (ipd->vpower_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		destroy_workqueue(ipd->check_power_status_queue);
	}

	if (ipd->vesd_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_esd_status_work);
		destroy_workqueue(ipd->check_esd_status_queue);
	}

	if (ipd->isEnableIRQ) {
		disable_irq_nosync(ipd->isr_gpio);
		free_irq(ipd->isr_gpio, (void *)ipd->i2c_id);
	}

	if (ipd->input_device != NULL) {
		input_unregister_device(ipd->input_device);
		input_free_device(ipd->input_device);
	}

	if (gpio_is_valid(ipd->int_gpio)) {
		gpio_free(ipd->int_gpio);
	}

	if (gpio_is_valid(ipd->reset_gpio)) {
		gpio_free(ipd->reset_gpio);
	}

	if (gpio_is_valid(ipd->gpio_id0)) {
		gpio_free(ipd->gpio_id0);
	}

	if (gpio_is_valid(ipd->gpio_id1)) {
		gpio_free(ipd->gpio_id1);
	}

	if (ipd->ts_pinctrl) {
		devm_pinctrl_put(ipd->ts_pinctrl);
		ipd->ts_pinctrl = NULL;
	}

#ifdef REGULATOR_POWER_ON
	ilitek_regulator_power_on(false);
	ilitek_regulator_release();
#endif

	ipio_kfree((void **)&ipd);
	ilitek_platform_core_remove();

	mutex_destroy(&ipd->ilitek_debug_read_mutex);
	mutex_destroy(&ipd->ilitek_debug_mutex);
	mutex_destroy(&ipd->plat_mutex);

	return 0;
}

/**
 * The probe func would be called after an i2c device was detected by kernel.
 *
 * It will still return zero even if it couldn't get a touch ic info.
 * The reason for why we allow it passing the process is because users/developers
 * might want to have access to ICE mode to upgrade a firwmare forcelly.
 */
#if (INTERFACE == I2C_INTERFACE)
static int ilitek_platform_probe(struct i2c_client *client, const struct i2c_device_id *id)
#else
static int ilitek_platform_probe(struct spi_device *spi)
#endif
{
	int ret;

#ifdef REGULATOR_POWER_ON
#if (TP_PLATFORM == PT_MTK)
	const char *vdd_name = "vtouch";
#else
	const char *vdd_name = "vdd";
#endif /* PT_MTK */
	const char *vcc_i2c_name = "vcc_i2c";
#endif /* REGULATOR_POWER_ON */

	if (tpd_fw_cdev.TP_have_registered) {
		ipio_info("TP have registered by other TP\n");
		return -EPERM;
	}

	/* initialise the struct of touch ic memebers. */
	ipd = kzalloc(sizeof(*ipd), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ipd)) {
		ipio_err("Failed to allocate ipd memory, %ld\n", PTR_ERR(ipd));
		ret = -ENOMEM;
		goto KZALLOC_FAILED;
	}

#if (INTERFACE == I2C_INTERFACE)
	/* Set i2c slave addr if it's not configured */
	ipio_info("I2C Slave address = 0x%x\n", client->addr);

	ipd->client = client;
	ipd->i2c_id = id;
#else
	ipd->spi = spi;
#endif
	ipd->chip_id = TP_TOUCH_IC;
	ipd->isEnableIRQ = false;
	ipd->isEnablePollCheckPower = false;
	ipd->vpower_reg_nb = false;
	ipd->isEnablePollCheckEsd = false;
	ipd->vesd_reg_nb = false;

	ipio_info("Driver Version : %s\n", DRIVER_VERSION);
	ipio_info("Driver for Touch IC :  %x\n", TP_TOUCH_IC);
	ipio_info("Driver on platform :  %x\n", TP_PLATFORM);
	ipio_info("Driver interface :  %s\n", (INTERFACE == I2C_INTERFACE) ? "I2C" : "SPI");

	/*
	 * Different ICs may require different delay time for the reset.
	 * They may also depend on what your platform need to.
	 */
	 if (ipd->chip_id == CHIP_TYPE_ILI9881) {
		 ipd->delay_time_high = 10;
		 ipd->delay_time_low = 5;
#if (INTERFACE == I2C_INTERFACE)
		 ipd->edge_delay = 100;
#else
		 ipd->edge_delay = 1;
#endif
	}

	mutex_init(&ipd->plat_mutex);
	spin_lock_init(&ipd->plat_spinlock);

	/* Init members for debug */
	mutex_init(&ipd->ilitek_debug_mutex);
	mutex_init(&ipd->ilitek_debug_read_mutex);
	init_waitqueue_head(&(ipd->inq));
	ipd->debug_data_frame = 0;
	ipd->debug_node_open = false;

#ifdef REGULATOR_POWER_ON
#if (TP_PLATFORM == PT_MTK)
	ipd->vdd = regulator_get(tpd->tpd_dev, vdd_name);
	tpd->reg = ipd->vdd;
#else
	ipd->vdd = regulator_get(&ipd->client->dev, vdd_name);
#endif /* PT_MTK */
	if (ERR_ALLOC_MEM(ipd->vdd)) {
		ipio_err("regulator_get vdd fail\n");
		ipd->vdd = NULL;
		ret = PTR_ERR(ipd->vdd);
		goto REULATOR_GET_FAILED;
	} else {
		if (regulator_set_voltage(ipd->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
			ipio_err("Failed to set vdd %d.\n", VDD_VOLTAGE);
	}

	ipd->vdd_i2c = regulator_get(&ipd->client->dev, vcc_i2c_name);
	if (ERR_ALLOC_MEM(ipd->vdd_i2c)) {
		ipio_err("regulator_get vdd_i2c fail.\n");
		ipd->vdd_i2c = NULL;
		ret = PTR_ERR(ipd->vdd_i2c);
		goto REULATOR_GET_FAILED;
	} else {
		if (regulator_set_voltage(ipd->vdd_i2c, VDD_I2C_VOLTAGE, VDD_I2C_VOLTAGE) < 0)
			ipio_err("Failed to set vdd_i2c %d\n", VDD_I2C_VOLTAGE);
	}
	ilitek_regulator_power_on(true);
#endif /* REGULATOR_POWER_ON */

	ret = ilitek_platform_pinctrl_init(ipd);
	if (ret < 0) {
		ipio_err("Failed to init pinctrl\n ");
	}

	ret = ilitek_platform_gpio();
	if (ret < 0) {
		ipio_err("Failed to request gpios\n ");
		goto GPIO_GET_FAILED;
	}

	ret = ilitek_platform_set_vendor_id_gpio();
	if (ret < 0) {
		ipio_err("Failed to request vendor id gpios");
		goto VENDORID_GPIO_GET_FAILED;
	}

	/* If kernel failes to allocate memory to the core components, driver will be unloaded. */
	if (ilitek_platform_core_init() < 0) {
		ipio_err("Failed to allocate cores' mem\n");
		ret = -ENOMEM;
		goto PLATFORM_CORE_INIT_FAILED;
	}

#ifdef HOST_DOWNLOAD
	core_firmware_boot_host_download();
#else
	ilitek_platform_tp_hw_reset(true);
#endif

	/* get our tp ic information
	    when fw in chip is cleared, info-read must be failed.
	    so don't return error in this
	*/
	ilitek_platform_read_tp_info();

	ilitek_vendor_id = ilitek_platform_get_vendor_id();

	/* If it defines boot upgrade, input register will be done inside boot function. */
	ret = ilitek_platform_input_init();
	if (ret < 0) {
		ipio_err("Failed to init input device in kernel\n");
		goto INPUT_INIT_FAILED;
	}

	ret = ilitek_platform_isr_register();
	if (ret < 0) {
		ipio_err("Failed to register ISR\n");
		goto REGISTER_ISR_FAILED;
	}

	ret = ilitek_platform_reg_suspend();
	if (ret < 0) {
		ipio_err("Failed to register suspend/resume function\n");
		goto FB_REGISTER_FAILED;
	}
	ret = ilitek_platform_reg_power_check();
	if (ret < 0) {
		ipio_err("Failed to register power check function\n");
		goto BATTERY_REG_FAILED;
	}

	ret = ilitek_platform_reg_esd_check();
	if (ret < 0) {
		ipio_err("Failed to register esd check function\n");
		goto ESD_REG_FAILED;
	}

	/* Create nodes for users */
	ret = ilitek_proc_init();
	if (ret < 0) {
		ipio_err("Failed to create proc\n");
		goto PROC_INIT_FAILED;
	}
#if (TP_PLATFORM == PT_MTK)
	tpd_load_status = 1;
#endif /* PT_MTK */

	/* init rawdata path */
	mp_path_init();

#ifdef BOOT_FW_UPGRADE
	ipd->update_thread = kthread_run(kthread_handler, "boot_fw", "ili_fw_boot");
	if (IS_ERR(ipd->update_thread)) {
		ipd->update_thread = NULL;
		ipio_err("Failed to create fw upgrade thread ret=%ld\n", PTR_ERR(ipd->update_thread));
	}
#endif /* BOOT_FW_UPGRADE */

	tpd_fw_cdev.TP_have_registered = true;
	return 0;

PROC_INIT_FAILED:
	ilitek_proc_remove();
	if (ipd->vesd_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_esd_status_work);
		destroy_workqueue(ipd->check_esd_status_queue);
	}
ESD_REG_FAILED:
	if (ipd->vpower_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		destroy_workqueue(ipd->check_power_status_queue);
	}
BATTERY_REG_FAILED:
#if (TP_PLATFORM != PT_MTK)
	#ifdef CONFIG_FB
	fb_unregister_client(&ipd->notifier_fb);
	#else
	unregister_early_suspend(&ipd->early_suspend);
	#endif /* CONFIG_FB */
#endif
FB_REGISTER_FAILED:
REGISTER_ISR_FAILED:
	if (ipd->isEnableIRQ) {
		disable_irq_nosync(ipd->isr_gpio);
		free_irq(ipd->isr_gpio, (void *)ipd->i2c_id);
	}
#ifdef USE_KTHREAD
	if (ipd->irq_thread != NULL) {
		ipd->irq_trigger = true;
		ipd->free_irq_thread = true;
		wake_up_interruptible(&waiter);
		kthread_stop(ipd->irq_thread);
		ipd->irq_thread = NULL;
	}
#endif /* USE_KTHREAD */

	if (ipd->input_device != NULL) {
		input_unregister_device(ipd->input_device);
		input_free_device(ipd->input_device);
	}
INPUT_INIT_FAILED:
PLATFORM_CORE_INIT_FAILED:
	core_fr_remove();
	core_firmware_remove();
	core_i2c_remove();
	core_protocol_remove();
	core_config_remove();

	if (gpio_is_valid(ipd->gpio_id1)) {
		gpio_free(ipd->gpio_id1);
	}
	if (gpio_is_valid(ipd->gpio_id0)) {
		gpio_free(ipd->gpio_id0);
	}

VENDORID_GPIO_GET_FAILED:
	if (gpio_is_valid(ipd->int_gpio)) {
		gpio_free(ipd->int_gpio);
	}
	if (gpio_is_valid(ipd->reset_gpio)) {
		gpio_free(ipd->reset_gpio);
	}
GPIO_GET_FAILED:
	if (ipd->ts_pinctrl) {
		devm_pinctrl_put(ipd->ts_pinctrl);
		ipd->ts_pinctrl = NULL;
	}

#ifdef REGULATOR_POWER_ON
	ilitek_regulator_power_on(false);
REULATOR_GET_FAILED:
	ilitek_regulator_release();
#endif

	mutex_destroy(&ipd->ilitek_debug_read_mutex);
	mutex_destroy(&ipd->ilitek_debug_mutex);
	mutex_destroy(&ipd->plat_mutex);
	kfree(ipd);
KZALLOC_FAILED:

	return ret;
}

static const struct i2c_device_id tp_device_id[] = {
	{DEVICE_ID, 0},
	{},			/* should not omitted */
};

MODULE_DEVICE_TABLE(i2c, tp_device_id);

/*
 * The name in the table must match the definiation
 * in a dts file.
 *
 */
static struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

#if (TP_PLATFORM == PT_MTK)
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	ipio_info("TPD detect i2c device\n");
	strlcpy(info->type, TPD_DEVICE, sizeof(info->type));
	return 0;
}
#endif /* PT_MTK */

#if (INTERFACE == I2C_INTERFACE)
static struct i2c_driver tp_i2c_driver = {
	.driver = {
		   .name = DEVICE_ID,
		   .owner = THIS_MODULE,
		   .of_match_table = tp_match_table,
		   },
	.probe = ilitek_platform_probe,
	.remove = ilitek_platform_remove,
	.id_table = tp_device_id,
#if (TP_PLATFORM == PT_MTK)
	.detect = tpd_detect,
#endif /* PT_MTK */
};
#else
static struct spi_driver tp_spi_driver = {
	.driver = {
		.name	= DEVICE_ID,
		.owner = THIS_MODULE,
		.of_match_table = tp_match_table,
	},
	.probe = ilitek_platform_probe,
	.remove = ilitek_platform_remove,
};
#endif

#if (TP_PLATFORM == PT_MTK)
static int tpd_local_init(void)
{
	ipio_info("TPD init device driver\n");

	if (i2c_add_driver(&tp_i2c_driver) != 0) {
		ipio_err("Unable to add i2c driver\n");
		return -EPERM;
	}
	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");

		i2c_del_driver(&tp_i2c_driver);
		return -EPERM;
	}

	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
				   tpd_dts_data.tpd_key_dim_local);
	}

	tpd_type_cap = 1;

	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = DEVICE_ID,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};
#endif /* PT_MTK */

static int __init ilitek_platform_init(void)
{
	int res = 0;

	ipio_info("TP driver init\n");

#if (TP_PLATFORM == PT_MTK)
	tpd_get_dts_info();
	res = tpd_driver_add(&tpd_device_driver);
	if (res < 0) {
		ipio_err("TPD add TP driver failed\n");
		tpd_driver_remove(&tpd_device_driver);
		return -ENODEV;
	}
#else
#if (INTERFACE == I2C_INTERFACE)
	ipio_info("TP driver add i2c interface\n");
	res = i2c_add_driver(&tp_i2c_driver);
	if (res < 0) {
		ipio_err("Failed to add i2c driver\n");
		i2c_del_driver(&tp_i2c_driver);
		return -ENODEV;
	}
#else
	ipio_info("TP driver add spi interface\n");
	res = spi_register_driver(&tp_spi_driver);
	if (res < 0) {
		ipio_err("Failed to add ilitek driver\n");
		spi_unregister_driver(&tp_spi_driver);
		return -ENODEV;
	}
#endif
#endif /* PT_MTK */

	ipio_info("Succeed to add ilitek driver\n");
	return res;
}

static void __exit ilitek_platform_exit(void)
{
	ipio_info("I2C driver has been removed\n");

#if (TP_PLATFORM == PT_MTK)
	tpd_driver_remove(&tpd_device_driver);
#else
#if (INTERFACE == I2C_INTERFACE)
	i2c_del_driver(&tp_i2c_driver);
#else
	spi_unregister_driver(&tp_spi_driver);
#endif
#endif /* PT_MTK */
}

module_init(ilitek_platform_init);
module_exit(ilitek_platform_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
