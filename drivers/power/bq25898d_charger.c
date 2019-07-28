/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * This package demos the architecture of bq2589x driver used for linux
 * based system. It also contains the code to tune the output of an adjustable
 * adjust high voltage adapter(AHVDCP) dynamically to achieve better charging efficiency.
 *									[DISCLAIMER]
 * The code is designed for DEMO purpose only, and may be subject to change
 * for any bug fix or improvement without proir notice.
 * TI could offer help to port and debug customer code derived from the demo code,
 * but it is customer's responsibility to assure the code quality and reliability
 * to meet their application requirement.
 */

#define pr_fmt(fmt)	"bq2589x: %s: " fmt, __func__
#define DEBUG
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>
#include "bq25898d_reg.h"

#define DBG_FS

enum bq2589x_vbus_type {
	BQ2589X_VBUS_NONE,
	BQ2589X_VBUS_USB_SDP,
	BQ2589X_VBUS_USB_CDP,
	BQ2589X_VBUS_USB_DCP,
	BQ2589X_VBUS_MAXC,
	BQ2589X_VBUS_UNKNOWN,
	BQ2589X_VBUS_NONSTAND,
	BQ2589X_VBUS_OTG,
	BQ2589X_VBUS_TYPE_NUM,
};

enum bq2589x_device_type {
	BQ25898 = 0x00,
	BQ25898D = 0x02,
};

enum bq2589x_dp_volt {
	BQ2589X_DP_HIZ,
	BQ2589X_DP_0MV,
	BQ2589X_DP_600MV,
	BQ2589X_DP_1200MV,
	BQ2589X_DP_2000MV,
	BQ2589X_DP_2700MV,
	BQ2589X_DP_3300MV,
	BQ2589X_DP_SHORT
};

enum bq2589x_dm_volt {
	BQ2589X_DM_HIZ,
	BQ2589X_DM_0MV,
	BQ2589X_DM_600MV,
	BQ2589X_DM_1200MV,
	BQ2589X_DM_2000MV,
	BQ2589X_DM_2700MV,
	BQ2589X_DM_3300MV,
	BQ2589X_DM_3300MV2,
};

enum {
	USER		= BIT(0),
	JEITA		= BIT(1),
	BATT_FC		= BIT(2),
	BATT_PRES	= BIT(3),
	BATT_TUNE	= BIT(4),
};

enum wakeup_src {
	WAKEUP_SRC_MONITOR = 0,
	WAKEUP_SRC_JEITA,
	WAKEUP_SRC_MAX,
};

enum bq2589x_charge_state {
	CHARGE_STATE_IDLE = BQ2589X_CHRG_STAT_IDLE,
	CHARGE_STATE_PRECHG = BQ2589X_CHRG_STAT_PRECHG,
	CHARGE_STATE_FASTCHG = BQ2589X_CHRG_STAT_FASTCHG,
	CHARGE_STATE_CHGDONE = BQ2589X_CHRG_STAT_CHGDONE,
};

#define WAKEUP_SRC_MASK (~(~0 << WAKEUP_SRC_MAX))
struct bq2589x_wakeup_source {
	struct wakeup_source source;
	unsigned long enabled_bitmap;
	spinlock_t ws_lock;
};

struct bq2589x_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq2589x_config {
	bool enable_auto_dpdm;
	bool enable_12v;
	bool enable_hvdcp;
	bool enable_maxc;

	int	charge_voltage;
	int	charge_current;
	int boost_voltage;
	int boost_current;

	bool enable_term;
	int	term_current;

	bool enable_ico;
	bool use_absolute_vindpm;

	int dpdm_sw_gpio;
	int usb_id_gpio;
};


struct bq2589x_chip {
	struct device *dev;
	struct i2c_client *client;

	enum bq2589x_device_type device_type;
	int revision;

	struct bq2589x_config cfg;


	struct mutex i2c_rw_lock;
	struct mutex charging_disable_lock;
	struct mutex profile_change_lock;
	struct mutex data_lock;
	struct mutex irq_complete;

	struct bq2589x_wakeup_source bq2589x_ws;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool usb_present;
	bool charge_enabled;
	bool otg_enabled;

	bool power_good;
	bool vbus_good;

	bool batt_full;
	bool batt_present;

	int	vbus_type;
	int charge_state;
	int charging_disabled_status;

	enum power_supply_type usb_type;

	int chg_ma;
	int chg_mv;
	int icl_ma;
	int ivl_mv;

/* if use software jeita in case of NTC is connected to gauge */
	bool software_jeita_supported;
	bool jeita_active;

	bool batt_hot;
	bool batt_cold;
	bool batt_warm;
	bool batt_cool;

	int batt_hot_degc;
	int batt_warm_degc;
	int batt_cool_degc;
	int batt_cold_degc;
	int hot_temp_hysteresis;
	int cold_temp_hysteresis;

	int batt_cool_ma;
	int batt_warm_ma;
	int batt_cool_mv;
	int batt_warm_mv;

	int batt_temp;

	int jeita_ma;
	int jeita_mv;

	int vbus_volt;
	int vbat_volt;
	int vsys_volt;
	int chg_curr;

	int fault_status;

	int skip_writes;
	int skip_reads;

	struct delayed_work ico_work;

	struct delayed_work discharge_jeita_work;
	struct delayed_work charge_jeita_work;
	struct delayed_work update_heartbeat_work;

	struct alarm jeita_alarm;

	struct dentry *debug_root;

	struct bq2589x_otg_regulator otg_vreg;

	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply batt_psy;

};


static int bq2589x_update_charging_profile(struct bq2589x_chip *bq);
static void bq2589x_dpdm_to_bq(struct bq2589x_chip *bq);
static void bq2589x_dpdm_to_ap(struct bq2589x_chip *bq);

static int __bq2589x_read_byte(struct bq2589x_chip *bq, u8 reg, u8 *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8)ret;

	return 0;

}

static int __bq2589x_write_byte(struct bq2589x_chip *bq, u8 reg, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				data, reg, ret);
		return ret;
	}
	return 0;

}

static int bq2589x_read_byte(struct bq2589x_chip *bq, u8 *data, u8 reg)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2589x_read_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2589x_write_byte(struct bq2589x_chip *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2589x_write_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq2589x_update_bits(struct bq2589x_chip *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);

	ret = __bq2589x_read_byte(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, reg=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;
	pr_info("tmp:%x,data:%x\n", tmp, data);
	ret = __bq2589x_write_byte(bq, reg, tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, reg=%d\n", reg, ret);
	}

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static void bq2589x_stay_awake(struct bq2589x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);

	if (!__test_and_set_bit(wk_src, &source->enabled_bitmap)) {
		__pm_stay_awake(&source->source);
		pr_debug("enabled source %s, wakeup_src %d\n",
			source->source.name, wk_src);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);
}

static void bq2589x_relax(struct bq2589x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);
	if (__test_and_clear_bit(wk_src, &source->enabled_bitmap) &&
		!(source->enabled_bitmap & WAKEUP_SRC_MASK)) {
		__pm_relax(&source->source);
		pr_debug("disabled source %s\n", source->source.name);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);

	pr_debug("relax source %s, wakeup_src %d\n",
		source->source.name, wk_src);
}

static void bq2589x_wakeup_src_init(struct bq2589x_chip *bq)
{
	spin_lock_init(&bq->bq2589x_ws.ws_lock);
	wakeup_source_init(&bq->bq2589x_ws.source, "bq2589x_chip");
}

#if 0
static enum bq2589x_vbus_type bq2589x_get_vbus_type(struct bq2589x_chip *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0)
		return 0;
	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	return val;
}
#endif

static int bq2589x_enable_otg(struct bq2589x_chip *bq)
{
	u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
							   BQ2589X_OTG_CONFIG_MASK, val);

}

static int bq2589x_disable_otg(struct bq2589x_chip *bq)
{
	u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
							   BQ2589X_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_disable_otg);

static int bq2589x_set_otg_volt(struct bq2589x_chip *bq, int volt)
{
	u8 val = 0;

	if (volt < BQ2589X_BOOSTV_BASE)
		volt = BQ2589X_BOOSTV_BASE;
	if (volt > BQ2589X_BOOSTV_BASE
			+ (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT)
			* BQ2589X_BOOSTV_LSB)
		volt = BQ2589X_BOOSTV_BASE
			+ (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT)
			* BQ2589X_BOOSTV_LSB;

	val = ((volt - BQ2589X_BOOSTV_BASE) / BQ2589X_BOOSTV_LSB)
			<< BQ2589X_BOOSTV_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A,
							BQ2589X_BOOSTV_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_volt);

static int bq2589x_set_otg_current(struct bq2589x_chip *bq, int curr)
{
	u8 temp;

	if (curr == 500)
		temp = BQ2589X_BOOST_LIM_500MA;
	else if (curr == 700)
		temp = BQ2589X_BOOST_LIM_700MA;
	else if (curr == 1100)
		temp = BQ2589X_BOOST_LIM_1100MA;
	else if (curr == 1600)
		temp = BQ2589X_BOOST_LIM_1600MA;
	else if (curr == 1800)
		temp = BQ2589X_BOOST_LIM_1800MA;
	else if (curr == 2100)
		temp = BQ2589X_BOOST_LIM_2100MA;
	else if (curr == 2400)
		temp = BQ2589X_BOOST_LIM_2400MA;
	else
		temp = BQ2589X_BOOST_LIM_1300MA;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A,
							BQ2589X_BOOST_LIM_MASK,
							temp << BQ2589X_BOOST_LIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_current);


static int bq2589x_enable_hvdcp(struct bq2589x_chip *bq)
{
	int ret;
	u8 val = BQ2589X_HVDCP_ENABLE << BQ2589X_HVDCPEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_HVDCPEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_hvdcp);

static int bq2589x_disable_hvdcp(struct bq2589x_chip *bq)
{
	int ret;
	u8 val = BQ2589X_HVDCP_DISABLE << BQ2589X_HVDCPEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_HVDCPEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_hvdcp);

static int bq2589x_enable_maxc(struct bq2589x_chip *bq)
{
	int ret;
	u8 val = BQ2589X_MAXC_ENABLE << BQ2589X_MAXCEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_MAXCEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_maxc);

static int bq2589x_disable_maxc(struct bq2589x_chip *bq)
{
	int ret;
	u8 val = BQ2589X_MAXC_DISABLE << BQ2589X_MAXCEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_MAXCEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_maxc);


static int bq2589x_enable_charger(struct bq2589x_chip *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
						BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}

static int bq2589x_disable_charger(struct bq2589x_chip *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
						BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_charger);


int bq2589x_adc_start(struct bq2589x_chip *bq, bool oneshot)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_02);
	if (ret < 0) {
		dev_err(bq->dev, "%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot)
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_START_MASK,
							BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
	else
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,
							BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_start);

int bq2589x_adc_stop(struct bq2589x_chip *bq)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,
							BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_adc_stop);


int bq2589x_adc_read_battery_volt(struct bq2589x_chip *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0E);
	if (ret < 0) {
		dev_err(bq->dev, "read battery voltage failed :%d\n", ret);
		return ret;
	}
	volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB;
	return volt;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);


int bq2589x_adc_read_sys_volt(struct bq2589x_chip *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0F);
	if (ret < 0) {
		dev_err(bq->dev, "read system voltage failed :%d\n", ret);
		return ret;
	}
	volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB;
	return volt;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);

int bq2589x_adc_read_vbus_volt(struct bq2589x_chip *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_11);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	}
	volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB;
	return volt;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);

int bq2589x_adc_read_temperature(struct bq2589x_chip *bq)
{
	uint8_t val;
	int temp;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_10);
	if (ret < 0) {
		dev_err(bq->dev, "read temperature failed :%d\n", ret);
		return ret;
	}
	temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB;
	return temp;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_temperature);

int bq2589x_adc_read_charge_current(struct bq2589x_chip *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_12);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	}
	volt = (int)(BQ2589X_ICHGR_BASE
			+ ((val & BQ2589X_ICHGR_MASK)
			>> BQ2589X_ICHGR_SHIFT)
			* BQ2589X_ICHGR_LSB);
	return volt;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);

int bq2589x_set_chargecurrent(struct bq2589x_chip *bq, int curr)
{
	u8 ichg;

	ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_04,
						BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_chargecurrent);

int bq2589x_set_term_current(struct bq2589x_chip *bq, int curr)
{
	u8 iterm;

	iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05,
						BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_term_current);


int bq2589x_set_prechg_current(struct bq2589x_chip *bq, int curr)
{
	u8 iprechg;

	iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05,
						BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_prechg_current);

int bq2589x_set_chargevoltage(struct bq2589x_chip *bq, int volt)
{
	u8 val;

	val = (volt - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06,
						BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_chargevoltage);


int bq2589x_set_input_volt_limit(struct bq2589x_chip *bq, int volt)
{
	u8 val;

	val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_0D,
						BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_volt_limit);

int bq2589x_set_input_current_limit(struct bq2589x_chip *bq, int curr)
{
	u8 val;
	int ret;

	pr_info("curr:%d\n", curr);
	val = curr / BQ2589X_IINLIM_LSB;
	pr_info("val:%x\n", val);
	ret = bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_IINLIM_MASK,
						val << BQ2589X_IINLIM_SHIFT);
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	pr_info("reg00:%x\n", val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_current_limit);


int bq2589x_set_vindpm_offset(struct bq2589x_chip *bq, int offset)
{
	u8 val;

	if (offset == 400)
		val = BQ2589X_VINDPMOS_400MV;
	else
		val = BQ2589X_VINDPMOS_600MV;

	return bq2589x_update_bits(bq, BQ2589X_REG_01, BQ2589X_VINDPMOS_MASK,
						val << BQ2589X_VINDPMOS_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_vindpm_offset);

int bq2589x_get_charging_status(struct bq2589x_chip *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		pr_err("Failed to read register 0x0b:%d\n", ret);
		return ret;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	return val;
}
EXPORT_SYMBOL_GPL(bq2589x_get_charging_status);

void bq2589x_set_otg(struct bq2589x_chip *bq, bool enable)
{
	int ret;

	if (enable)
		ret = bq2589x_enable_otg(bq);
	else
		ret = bq2589x_disable_otg(bq);

	if (!ret)
		bq->otg_enabled = enable;
	else
		dev_err(bq->dev, "%s:Failed to %s otg:%d\n", __func__,
						enable ? "enable" : "disable", ret);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg);

int bq2589x_set_watchdog_timer(struct bq2589x_chip *bq, u8 timeout)
{
	u8 val;

	val = (timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB;
	val <<= BQ2589X_WDT_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_07,
						BQ2589X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_set_watchdog_timer);

int bq2589x_disable_watchdog_timer(struct bq2589x_chip *bq)
{
	u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_07,
						BQ2589X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_disable_watchdog_timer);

int bq2589x_reset_watchdog_timer(struct bq2589x_chip *bq)
{
	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
						BQ2589X_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_reset_watchdog_timer);

int bq2589x_force_dpdm(struct bq2589x_chip *bq)
{
	int ret;
	u8 val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_FORCE_DPDM_MASK, val);

	pr_info("Force DPDM %s\n", !ret ? "successfully" : "failed");

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_force_dpdm);

int bq2589x_reset_chip(struct bq2589x_chip *bq)
{
	int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_14,
						BQ2589X_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

int bq2589x_enter_ship_mode(struct bq2589x_chip *bq)
{
	int ret;
	u8 val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_BATFET_DIS_MASK, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enter_ship_mode);

int bq2589x_enter_hiz_mode(struct bq2589x_chip *bq)
{
	u8 val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00,
						BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_enter_hiz_mode);

int bq2589x_exit_hiz_mode(struct bq2589x_chip *bq)
{

	u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00,
						BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_exit_hiz_mode);

int bq2589x_get_hiz_mode(struct bq2589x_chip *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret)
		return ret;
	*state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);

int bq2589x_pumpx_enable(struct bq2589x_chip *bq, int enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
	else
		val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_04,
						BQ2589X_EN_PUMPX_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_enable);

int bq2589x_pumpx_increase_volt(struct bq2589x_chip *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_PUMPX_UP_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt);

int bq2589x_pumpx_increase_volt_done(struct bq2589x_chip *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_UP_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx up finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt_done);

int bq2589x_pumpx_decrease_volt(struct bq2589x_chip *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_PUMPX_DOWN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt);

int bq2589x_pumpx_decrease_volt_done(struct bq2589x_chip *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_DOWN_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx down finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt_done);

static int bq2589x_force_ico(struct bq2589x_chip *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_FORCE_ICO_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_force_ico);

static int bq2589x_check_ico_done(struct bq2589x_chip *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_14);
	if (ret)
		return ret;

	if (val & BQ2589X_ICO_OPTIMIZED_MASK)
		return 1;  /*finished*/
	else
		return 0;   /* in progress*/
}
EXPORT_SYMBOL_GPL(bq2589x_check_ico_done);

static int bq2589x_enable_term(struct bq2589x_chip *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	else
		val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_07,
						BQ2589X_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_term);

static int bq2589x_enable_12v_handshake(struct bq2589x_chip *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_ENABLE_12V << BQ2589X_EN12V_SHIFT;
	else
		val = BQ2589X_DISABLE_12V << BQ2589X_EN12V_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_01,
						BQ2589X_EN12V_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_12v_handshake);

static int bq2589x_enable_auto_dpdm(struct bq2589x_chip *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_AUTO_DPDM_EN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_auto_dpdm);

static int bq2589x_use_absolute_vindpm(struct bq2589x_chip *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
	else
		val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_use_absolute_vindpm);

static int bq2589x_enable_ico(struct bq2589x_chip *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	else
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_ico);

static int bq2589x_read_idpm_limit(struct bq2589x_chip *bq)
{
	uint8_t val;
	int curr;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_13);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	}
	curr = ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB;
	return curr;
}
EXPORT_SYMBOL_GPL(bq2589x_read_idpm_limit);

#if 0
static bool bq2589x_is_charge_done(struct bq2589x_chip *bq)
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s:read REG0B failed :%d\n", __func__, ret);
		return false;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;

	return (val == BQ2589X_CHRG_STAT_CHGDONE);
}
EXPORT_SYMBOL_GPL(bq2589x_is_charge_done);
#endif

static int bq2589x_init_device(struct bq2589x_chip *bq)
{
	int ret;

	bq2589x_disable_watchdog_timer(bq);

	if (bq->cfg.enable_hvdcp)
		bq2589x_enable_hvdcp(bq);
	else
		bq2589x_disable_hvdcp(bq);

	if (bq->cfg.enable_maxc)
		bq2589x_enable_maxc(bq);
	else
		bq2589x_disable_maxc(bq);

	bq2589x_enable_12v_handshake(bq, bq->cfg.enable_12v);
	bq2589x_enable_auto_dpdm(bq, bq->cfg.enable_auto_dpdm);
	bq2589x_enable_term(bq, bq->cfg.enable_term);
	bq2589x_enable_ico(bq, bq->cfg.enable_ico);

	/*force use absolute vindpm if auto_dpdm not enabled*/
	if (!bq->cfg.enable_auto_dpdm)
		bq->cfg.use_absolute_vindpm = true;
	bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);

	bq->chg_ma = bq->cfg.charge_current;
	bq->chg_mv = bq->cfg.charge_voltage;

	ret = bq2589x_set_vindpm_offset(bq, 600);
	if (ret < 0) {
		pr_err("Failed to set vindpm offset:%d\n",  ret);
	}

	ret = bq2589x_set_term_current(bq, bq->cfg.term_current);
	if (ret < 0) {
		pr_err("Failed to set termination current:%d\n",  ret);
	}

	ret = bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage);
	if (ret < 0) {
		pr_err("Failed to set charge voltage:%d\n",  ret);
	}

	ret = bq2589x_set_chargecurrent(bq, bq->cfg.charge_current);
	if (ret < 0) {
		pr_err("Failed to set charge current:%d\n",  ret);
	}

	ret = bq2589x_set_otg_volt(bq, bq->cfg.boost_voltage);
	if (ret < 0) {
		pr_err("Failed to set boost voltage:%d\n", ret);
	}

	ret = bq2589x_set_otg_current(bq, bq->cfg.boost_current);
	if (ret < 0) {
		pr_err("Failed to set boost current:%d\n", ret);
	}

	ret = bq2589x_enable_charger(bq);
	if (ret < 0) {
		pr_err("Failed to enable charger:%d\n",  ret);
		return ret;
	}
	bq->charge_enabled = true;
	return 0;
}

#if 0
static int bq2589x_charge_status(struct bq2589x_chip *bq)
{
	u8 val = 0;

	bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	switch (val) {
	case BQ2589X_CHRG_STAT_FASTCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2589X_CHRG_STAT_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BQ2589X_CHRG_STAT_CHGDONE:
	case BQ2589X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}
#endif
static int bq2589x_charging_disable(struct bq2589x_chip *bq, int reason,
								int disable)
{

	int ret = 0;
	int disabled;

	mutex_lock(&bq->charging_disable_lock);

	disabled = bq->charging_disabled_status;

	pr_info("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled && bq->charge_enabled)
		ret = bq2589x_disable_charger(bq);
	else if (!disabled && !bq->charge_enabled)
		ret = bq2589x_enable_charger(bq);

	if (ret) {
		pr_err("Couldn't disable/enable charging for reason=%d ret=%d\n",
							ret, reason);
	} else {
		bq->charging_disabled_status = disabled;
		mutex_lock(&bq->data_lock);
		bq->charge_enabled = !disabled;
		mutex_unlock(&bq->data_lock);
	}
	mutex_unlock(&bq->charging_disable_lock);

	return ret;
}


static struct power_supply *get_bms_psy(struct bq2589x_chip *bq)
{
	if (bq->bms_psy)
		return bq->bms_psy;
	bq->bms_psy = power_supply_get_by_name("ti_bms");
	if (!bq->bms_psy)
		pr_err("battery power supply not found\n");

	return bq->bms_psy;
}

static int bq2589x_get_batt_property(struct bq2589x_chip *bq,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct power_supply *bms_psy = get_bms_psy(bq);

	int ret;

	if (!bms_psy)
		return -EINVAL;

	ret = bms_psy->get_property(bq->bms_psy, psp, val);

	return ret;
}

static int bq2589x_get_prop_batt_present(struct bq2589x_chip *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2589x_get_batt_property(bq,
					POWER_SUPPLY_PROP_PRESENT, &batt_prop);
	if (!ret)
		bq->batt_present = batt_prop.intval;

	return ret;

}

#define DEFAULT_BATT_VOLTAGE_NOW	0
static int bq2589x_get_prop_batt_voltage(struct bq2589x_chip *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret, voltage;

	ret = bq2589x_get_batt_property(bq,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &batt_prop);
	if (!ret) {
		voltage = batt_prop.intval;
	} else {
		voltage = DEFAULT_BATT_VOLTAGE_NOW;
	}

	return voltage;
}

#define DEFAULT_BATT_CURRENT_NOW	0
static int bq2589x_get_prop_current_now(struct bq2589x_chip *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret, current_now;

	ret = bq2589x_get_batt_property(bq,
					POWER_SUPPLY_PROP_CURRENT_NOW, &batt_prop);
	if (!ret) {
		current_now = batt_prop.intval;
	} else {
		current_now = DEFAULT_BATT_CURRENT_NOW;
	}

	return current_now;
}

#define DEFAULT_BATT_CAPACITY	50
static int bq2589x_get_prop_capacity(struct bq2589x_chip *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret, capacity;

	ret = bq2589x_get_batt_property(bq,
					POWER_SUPPLY_PROP_CAPACITY, &batt_prop);
	if (!ret) {
		capacity = batt_prop.intval;
	} else {
		capacity = DEFAULT_BATT_CAPACITY;
	}

	return capacity;
}

static int bq2589x_get_prop_batt_full(struct bq2589x_chip *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq2589x_get_batt_property(bq,
					POWER_SUPPLY_PROP_STATUS, &batt_prop);
	pr_info("batt_prop.intval:%d\n", batt_prop.intval);
	if (!ret)
		bq->batt_full = (batt_prop.intval == POWER_SUPPLY_STATUS_FULL);
	pr_info("bq->batt_full:%d\n", bq->batt_full);

	return ret;
}


static void bq2589x_check_batt_pres(struct bq2589x_chip *bq)
{
	int ret = 0;
	bool last_batt_pres = bq->batt_present;

	ret = bq2589x_get_prop_batt_present(bq);
	if (!ret) {
		if (last_batt_pres != bq->batt_present) {
			ret = bq2589x_charging_disable(bq, BATT_PRES, !bq->batt_present);
			if (ret) {
				pr_err("failed to %s charging, ret = %d\n",
						bq->batt_full ? "disable" : "enable",
						ret);
			}
			power_supply_changed(&bq->batt_psy);
			power_supply_changed(bq->usb_psy);
		}
	}

}

static void bq2589x_check_batt_full(struct bq2589x_chip *bq)
{
	int ret = 0;
	bool last_batt_full = bq->batt_full;

	ret = bq2589x_get_prop_batt_full(bq);
	if (!ret) {
		if (last_batt_full != bq->batt_full) {
			ret = bq2589x_charging_disable(bq, BATT_FC, bq->batt_full);
			if (ret) {
				pr_err("failed to %s charging, ret = %d\n",
						bq->batt_full ? "disable" : "enable",
						ret);
			}
			power_supply_changed(&bq->batt_psy);
			power_supply_changed(bq->usb_psy);
		}
	}
}

static void bq2589x_check_jeita(struct bq2589x_chip *bq)
{

	int ret;
	bool last_hot, last_warm, last_cool, last_cold;
	union power_supply_propval batt_prop = {0,};

	ret = bq2589x_get_batt_property(bq,
					POWER_SUPPLY_PROP_TEMP, &batt_prop);
	if (!ret)
		bq->batt_temp = batt_prop.intval;

	if (bq->batt_temp == -EINVAL)
		return;

	last_hot = bq->batt_hot;
	last_warm = bq->batt_warm;
	last_cool = bq->batt_cool;
	last_cold = bq->batt_cold;
	pr_info("bq->batt_temp:%d,bq->batt_hot_degc:%d\n", bq->batt_temp, bq->batt_hot_degc);
	if (bq->batt_temp >= bq->batt_hot_degc) {/* HOT */
		if (!bq->batt_hot) {
			bq->batt_hot  = true;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp >= bq->batt_warm_degc) {/* WARM */
		if (!bq->batt_hot ||
				(bq->batt_temp < bq->batt_hot_degc - bq->hot_temp_hysteresis)) {
			bq->batt_hot  = false;
			bq->batt_warm = true;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_mv = bq->batt_warm_mv;
			bq->jeita_ma = bq->batt_warm_ma;
		}
	} else if (bq->batt_temp < bq->batt_cold_degc) {/* COLD */
		if (!bq->batt_cold) {
			bq->batt_hot  = false;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = true;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp < bq->batt_cool_degc) {/* COOL */
		if (!bq->batt_cold ||
				(bq->batt_temp > bq->batt_cold_degc + bq->cold_temp_hysteresis)) {
			bq->batt_hot  = false;
			bq->batt_warm = false;
			bq->batt_cool = true;
			bq->batt_cold = false;
			bq->jeita_mv = bq->batt_cool_mv;
			bq->jeita_ma = bq->batt_cool_ma;
		}
	} else {/* NORMAL */
		bq->batt_hot  = false;
		bq->batt_warm = false;
		bq->batt_cool = false;
		bq->batt_cold = false;
	}

	bq->jeita_active = bq->batt_cool || bq->batt_hot ||
					   bq->batt_cold || bq->batt_warm;

	if ((last_cold != bq->batt_cold) || (last_warm != bq->batt_warm) ||
		(last_cool != bq->batt_cool) || (last_hot != bq->batt_hot)) {
		bq2589x_update_charging_profile(bq);
		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
	} else if (bq->batt_hot || bq->batt_cold) {
		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
	}

}

static int calculate_jeita_poll_interval(struct bq2589x_chip *bq)
{
	int interval;

	if (bq->batt_hot || bq->batt_cold)
		interval = 5;
	else if (bq->batt_warm || bq->batt_cool)
		interval = 10;
	else
		interval = 15;
	return interval;
}

#define	FG_LOG_INTERVAL		60
static void bq2589x_dump_fg_reg(struct bq2589x_chip *bq)
{
	union power_supply_propval val = {0,};
	static int dump_cnt;

	if (++dump_cnt >= (FG_LOG_INTERVAL / calculate_jeita_poll_interval(bq))) {
		dump_cnt = 0;
		val.intval = 0;
		bq->bms_psy->set_property(bq->bms_psy,
				POWER_SUPPLY_PROP_UPDATE_NOW, &val);
	}
}

static enum alarmtimer_restart bq2589x_jeita_alarm_cb(struct alarm *alarm, ktime_t now)
{
	struct bq2589x_chip *bq = container_of(alarm, struct bq2589x_chip, jeita_alarm);
	unsigned long ns;

	bq2589x_stay_awake(&bq->bq2589x_ws, WAKEUP_SRC_JEITA);
	schedule_delayed_work(&bq->charge_jeita_work, HZ/2);

	ns = calculate_jeita_poll_interval(bq) * 1000000000LL;
	alarm_forward_now(alarm, ns_to_ktime(ns));
	return ALARMTIMER_RESTART;
}

static void bq2589x_dump_status(struct bq2589x_chip *bq);
static void bq2589x_charge_jeita_workfunc(struct work_struct *work)
{
	struct bq2589x_chip *bq = container_of(work,
							struct bq2589x_chip, charge_jeita_work.work);

	pr_info("enter");

	bq2589x_reset_watchdog_timer(bq);

	bq2589x_check_batt_pres(bq);
	bq2589x_check_batt_full(bq);
	bq2589x_dump_fg_reg(bq);

	bq2589x_check_jeita(bq);
	bq2589x_dump_status(bq);
	bq2589x_relax(&bq->bq2589x_ws, WAKEUP_SRC_JEITA);
}

static void bq2589x_discharge_jeita_workfunc(struct work_struct *work)
{
	struct bq2589x_chip *bq = container_of(work,
							struct bq2589x_chip, discharge_jeita_work.work);

	bq2589x_check_batt_pres(bq);
	bq2589x_check_batt_full(bq);
	bq2589x_dump_fg_reg(bq);

	bq2589x_check_jeita(bq);
	schedule_delayed_work(&bq->discharge_jeita_work,
							calculate_jeita_poll_interval(bq) * HZ);
}

static inline bool is_device_suspended(struct bq2589x_chip *bq);
static int bq2589x_get_prop_charge_type(struct bq2589x_chip *bq)
{
	u8 val = 0;

	if (is_device_suspended(bq))
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	switch (val) {
	case CHARGE_STATE_FASTCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHARGE_STATE_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHARGE_STATE_CHGDONE:
	case CHARGE_STATE_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int bq2589x_get_prop_charge_status(struct bq2589x_chip *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;
	u8 status;

	ret = bq2589x_get_batt_property(bq,
					POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret && batt_prop.intval == POWER_SUPPLY_STATUS_FULL)
		return POWER_SUPPLY_STATUS_FULL;

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) {
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	mutex_lock(&bq->data_lock);
	bq->charge_state =
		(status & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	mutex_unlock(&bq->data_lock);

	switch (bq->charge_state) {
	case CHARGE_STATE_FASTCHG:
	case CHARGE_STATE_PRECHG:
		return POWER_SUPPLY_STATUS_CHARGING;
	case CHARGE_STATE_CHGDONE:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case CHARGE_STATE_IDLE:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

}

static int bq2589x_get_prop_health(struct bq2589x_chip *bq)
{
	int ret;
	union power_supply_propval batt_prop = {0,};

	if (bq->software_jeita_supported) {
		if (bq->jeita_active) {
			if (bq->batt_hot)
				ret = POWER_SUPPLY_HEALTH_OVERHEAT;
			else if (bq->batt_warm)
				ret = POWER_SUPPLY_HEALTH_WARM;
			else if (bq->batt_cool)
				ret = POWER_SUPPLY_HEALTH_COOL;
			else if (bq->batt_cold)
				ret = POWER_SUPPLY_HEALTH_COLD;
		} else {
			ret = POWER_SUPPLY_HEALTH_GOOD;
		}
	} else {/* get health status from gauge */
		ret = bq2589x_get_batt_property(bq,
					POWER_SUPPLY_PROP_HEALTH, &batt_prop);
		if (!ret)
			ret = batt_prop.intval;
		else
			ret = POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	return ret;
}

static enum power_supply_property bq2589x_charger_props[] = {
		POWER_SUPPLY_PROP_CHARGE_TYPE,
		POWER_SUPPLY_PROP_PRESENT,
		POWER_SUPPLY_PROP_CHARGING_ENABLED,
		POWER_SUPPLY_PROP_HEALTH,
		POWER_SUPPLY_PROP_CAPACITY,
		POWER_SUPPLY_PROP_VOLTAGE_NOW,
		POWER_SUPPLY_PROP_CURRENT_NOW,
		POWER_SUPPLY_PROP_STATUS,
		POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,

		POWER_SUPPLY_PROP_CAPACITY_LEVEL,
		POWER_SUPPLY_PROP_TEMP,
		POWER_SUPPLY_PROP_CHARGE_FULL,
};


static int bq2589x_charger_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{

	struct bq2589x_chip *bq = container_of(psy, struct bq2589x_chip, batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_get_prop_charge_type(bq);
		pr_debug("POWER_SUPPLY_PROP_CHARGE_TYPE:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 3080;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2589x_get_prop_charge_status(bq);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq2589x_get_prop_health(bq);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq2589x_get_prop_capacity(bq);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->batt_present;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq2589x_get_prop_batt_voltage(bq);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq2589x_get_prop_current_now(bq);
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq->batt_temp;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
			return bq2589x_get_batt_property(bq, psp, val);
	default:
		return -EINVAL;

	}
	return 0;
}

static int bq2589x_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct bq2589x_chip *bq = container_of(psy, struct bq2589x_chip, batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq2589x_charging_disable(bq, USER, !val->intval);
		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int bq2589x_charger_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static int bq2589x_psy_register(struct bq2589x_chip *bq)
{
	int ret;

	bq->batt_psy.name = "battery";
	bq->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->batt_psy.properties = bq2589x_charger_props;
	bq->batt_psy.num_properties = ARRAY_SIZE(bq2589x_charger_props);
	bq->batt_psy.get_property = bq2589x_charger_get_property;
	bq->batt_psy.set_property = bq2589x_charger_set_property;
	bq->batt_psy.external_power_changed = NULL;
	bq->batt_psy.property_is_writeable = bq2589x_charger_is_writeable;
	ret = power_supply_register(bq->dev, &bq->batt_psy);
	if (ret < 0) {
		pr_err("failed to register batt_psy:%d\n", ret);
		return ret;
	}

	return 0;
}

static void bq2589x_psy_unregister(struct bq2589x_chip *bq)
{
	power_supply_unregister(&bq->batt_psy);
}

static int bq2589x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;

	struct bq2589x_chip *bq = rdev_get_drvdata(rdev);

	ret = bq2589x_enable_otg(bq);
	if (ret) {
		pr_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq->otg_enabled = true;
		pr_info("bq2589x_chip OTG mode Enabled!\n");
	}

	return ret;
}


static int bq2589x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2589x_chip *bq = rdev_get_drvdata(rdev);

	ret = bq2589x_disable_otg(bq);
	if (ret) {
		pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		bq->otg_enabled = false;
		pr_info("bq2589x_chip OTG mode Disabled\n");
	}

	return ret;
}


static int bq2589x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret;
	u8 status;
	u8 enabled;

	struct bq2589x_chip *bq = rdev_get_drvdata(rdev);

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_03);
	if (ret)
		return ret;
	enabled = ((status & BQ2589X_OTG_CONFIG_MASK) >> BQ2589X_OTG_CONFIG_SHIFT);

	return (enabled == BQ2589X_OTG_ENABLE) ? 1 : 0;

}


struct regulator_ops bq2589x_otg_reg_ops = {
	.enable		= bq2589x_otg_regulator_enable,
	.disable	= bq2589x_otg_regulator_disable,
	.is_enabled = bq2589x_otg_regulator_is_enable,
};

static int bq2589x_regulator_init(struct bq2589x_chip *bq)
{
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(bq->dev, bq->dev->of_node);
	if (!init_data) {
		dev_err(bq->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		bq->otg_vreg.rdesc.owner = THIS_MODULE;
		bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		bq->otg_vreg.rdesc.ops = &bq2589x_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		pr_info("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = bq->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		bq->otg_vreg.rdev = regulator_register(
					&bq->otg_vreg.rdesc, &cfg);
		if (IS_ERR(bq->otg_vreg.rdev)) {
			ret = PTR_ERR(bq->otg_vreg.rdev);
			bq->otg_vreg.rdev = NULL;
			if (ret != -EPROBE_DEFER)
				dev_err(bq->dev,
					"OTG reg failed, rc=%d\n", ret);
		}
	}

	return ret;
}

static ssize_t bq2589x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2589x_chip *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25898d");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
						"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2589x_store_register(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2589x_chip *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x14) {
		bq2589x_write_byte(bq, (unsigned char)reg, (unsigned char)val);
	}

	return count;
}

static DEVICE_ATTR(registers, 0660, bq2589x_show_registers, bq2589x_store_register);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};


static int bq2589x_parse_dt(struct device *dev, struct bq2589x_chip *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "ti,bq2589x,dpdm-switch-gpio",
			&bq->cfg.dpdm_sw_gpio);
	if (ret)
		pr_err("Failed to read node of ti, bq2589x,dpdm-switch-gpio\n");

	bq->cfg.usb_id_gpio = of_get_named_gpio(np, "ti,usbid-gpio", 0);
	if (bq->cfg.usb_id_gpio < 0) {
		pr_err("usb_id_gpio is not available\n");
	}
	pr_info("bq->cfg.dpdm_sw_gpio:%d\n", bq->cfg.dpdm_sw_gpio);
	pr_info("bq->cfg.usb_id_gpio:%d\n", bq->cfg.usb_id_gpio);

	bq->cfg.enable_auto_dpdm = of_property_read_bool(np,
			"ti,bq2589x,enable-auto-dpdm");
	bq->cfg.enable_term = of_property_read_bool(np,
			"ti,bq2589x,enable-termination");
	bq->cfg.enable_ico = of_property_read_bool(np,
			"ti,bq2589x,enable-ico");
	bq->cfg.use_absolute_vindpm = of_property_read_bool(np,
			"ti,bq2589x,use-absolute-vindpm");
	bq->cfg.enable_12v = of_property_read_bool(np,
			"ti,bq2589x,enable-12v");
	bq->cfg.enable_hvdcp = of_property_read_bool(np,
			"ti,bq2589x,enable-hvdcp");
	bq->cfg.enable_maxc = of_property_read_bool(np,
			"ti,bq2589x,enable-maxc");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-voltage",
			&bq->cfg.charge_voltage);
	if (ret)
		pr_err("Failed to read node of ti,bq2589x,charge-voltage\n");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current",
								&bq->cfg.charge_current);
	if (ret)
		pr_err("Failed to read node of ti,bq2589x,charge-current\n");

	ret = of_property_read_u32(np, "ti,bq2589x,termination-current",
			&bq->cfg.term_current);
	if (ret)
		pr_err("Failed to read node of ti,bq2589x, termination-current\n");

	pr_info("bq->cfg.enable_auto_dpdm:%d\n", bq->cfg.enable_auto_dpdm);
	pr_info("bq->cfg.enable_term:%d\n", bq->cfg.enable_term);
	pr_info("bq->cfg.enable_ico:%d\n", bq->cfg.enable_ico);
	pr_info("bq->cfg.use_absolute_vindpm:%d\n", bq->cfg.use_absolute_vindpm);
	pr_info("bq->cfg.enable_12v:%d\n", bq->cfg.enable_12v);
	pr_info("bq->cfg.enable_hvdcp:%d\n", bq->cfg.enable_hvdcp);
	pr_info("bq->cfg.enable_maxc:%d\n", bq->cfg.enable_maxc);
	pr_info("bq->cfg.charge_voltage:%d\n", bq->cfg.charge_voltage);
	pr_info("bq->cfg.charge_current:%d\n", bq->cfg.charge_current);
	pr_info("bq->cfg.term_current:%d\n", bq->cfg.term_current);

	return 0;
}


static int bq2589x_parse_jeita_dt(struct device *dev, struct bq2589x_chip *bq)
{
	struct device_node *np = dev->of_node;
	int ret;

	ret = of_property_read_u32(np, "ti,bq2589x,jeita-hot-degc",
			&bq->batt_hot_degc);
	if (ret) {
		pr_err("Failed to read ti,bq2589x, jeita-hot-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2589x,jeita-warm-degc",
			&bq->batt_warm_degc);
	if (ret) {
		pr_err("Failed to read ti,bq2589x, jeita-warm-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2589x,jeita-cool-degc",
			&bq->batt_cool_degc);
	if (ret) {
		pr_err("Failed to read ti,bq2589x, jeita-cool-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2589x,jeita-cold-degc",
			&bq->batt_cold_degc);
	if (ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cold-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti, bq2589x,jeita-hot-hysteresis",
			&bq->hot_temp_hysteresis);
	if (ret) {
		pr_err("Failed to read ti, bq2589x,jeita-hot-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2589x,jeita-cold-hysteresis",
			&bq->cold_temp_hysteresis);
	if (ret) {
		pr_err("Failed to read ti, bq2589x,jeita-cold-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2589x,jeita-cool-ma",
			&bq->batt_cool_ma);
	if (ret) {
		pr_err("Failed to read ti, bq2589x,jeita-cool-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2589x,jeita-cool-mv",
			&bq->batt_cool_mv);
	if (ret) {
		pr_err("Failed to read ti,bq2589x, jeita-cool-mv\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2589x,jeita-warm-ma",
			&bq->batt_warm_ma);
	if (ret) {
		pr_err("Failed to read ti, bq2589x,jeita-warm-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti, bq2589x,jeita-warm-mv",
			&bq->batt_warm_mv);
	if (ret) {
		pr_err("Failed to read ti, bq2589x,jeita-warm-mv\n");
		return ret;
	}

	bq->software_jeita_supported =
		of_property_read_bool(np, "ti,bq2589x,software-jeita-supported");

	return 0;
}

static void bq2589x_init_jeita(struct bq2589x_chip *bq)
{

	bq->batt_temp = -EINVAL;

	/* set default value in case of dts read fail */
	bq->batt_hot_degc = 600;
	bq->batt_warm_degc = 450;
	bq->batt_cool_degc = 100;
	bq->batt_cold_degc = 0;

	bq->hot_temp_hysteresis = 50;
	bq->cold_temp_hysteresis = 50;

	bq->batt_cool_ma = 400;
	bq->batt_cool_mv = 4100;
	bq->batt_warm_ma = 400;
	bq->batt_warm_mv = 4100;

	bq->software_jeita_supported = true;

	/* DTS setting will overwrite above default value */

	bq2589x_parse_jeita_dt(&bq->client->dev, bq);
}

static int bq2589x_detect_device(struct bq2589x_chip *bq)
{
	int ret;
	u8 data;

	ret = bq2589x_read_byte(bq, &data, BQ2589X_REG_14);
	if (ret == 0) {
		bq->device_type = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
		bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	}

	return ret;
}
#if 0
static void bq2589x_adjust_absolute_vindpm(struct bq2589x_chip *bq)
{
	u16 vbus_volt;
	int ret;

	/* wait for new adc data */
	msleep(1000);
	vbus_volt = bq2589x_adc_read_vbus_volt(bq);

	if (vbus_volt < 6000)
		bq->ivl_mv = vbus_volt - 600;
	else
		bq->ivl_mv = vbus_volt - 1200;
	ret = bq2589x_set_input_volt_limit(bq, bq->ivl_mv);

	pr_err("Set absolute vindpm threshold %s\n",
			!ret ? "Successfully" : "Failed");

}
#endif

static int bq2589x_update_charging_profile(struct bq2589x_chip *bq)
{
	int ret;
	union power_supply_propval prop = {0,};
	int chg_ma;
	int chg_mv;
	int icl;

	pr_info("chg_mv1:%d,chg_ma:%d,icl:%d,ivl:%d\n",
						bq->chg_mv, bq->chg_ma, bq->icl_ma, bq->ivl_mv);

	if (!bq->usb_present)
		return 0;

	pr_info("chg_mv:%d,chg_ma:%d,icl:%d,ivl:%d\n",
					bq->chg_mv, bq->chg_ma, bq->icl_ma, bq->ivl_mv);


	mutex_lock(&bq->profile_change_lock);

	if (bq->jeita_active) {
		chg_ma = bq->jeita_ma;
		chg_mv = bq->jeita_mv;
	} else {
		chg_ma = bq->chg_ma;
		chg_mv = bq->chg_mv;
	}

	icl = bq->icl_ma;


	pr_info("chg_mv2:%d,chg_ma:%d,icl:%d,ivl:%d\n",
							bq->chg_mv, bq->chg_ma, bq->icl_ma, bq->ivl_mv);
	/*TODO: add therm_lvl_sel*/

	pr_info("chg_mv:%d, chg_ma:%d, icl:%d, ivl:%d\n",
				chg_mv, chg_ma, icl, bq->ivl_mv);
	ret = bq2589x_set_input_current_limit(bq, bq->icl_ma);
	if (ret < 0)
		pr_err("failed to set input current limit:%d\n", ret);
	else
		pr_info("Set input current limit succefully:%d\n", ret);

	ret = bq2589x_set_chargevoltage(bq, bq->chg_mv);
	if (ret < 0)
		pr_err("Failed to set charge voltage:%d\n", ret);
	else
		pr_info("Set charge voltage succefully:%d\n", ret);

	ret = bq2589x_set_chargecurrent(bq, bq->chg_ma);
	if (ret < 0)
		pr_err("Failed to set charge current:%d\n", ret);
	else
		pr_info("Set charge current succefully:%d\n", ret);

	ret = bq2589x_set_input_volt_limit(bq, bq->ivl_mv);
	if (ret < 0)
		pr_err("failed to set input volt limit:%d\n", ret);
	else
		pr_info("Set input volt limit succefully:%d\n", ret);


	mutex_unlock(&bq->profile_change_lock);

	ret = bq->usb_psy->get_property(bq->usb_psy,
								POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		pr_err("could not read USB ONLINE property, ret=%d\n", ret);
	else
		pr_info("usb online status =%d\n", prop.intval);


	/* handle usb online update */
	pr_info("handle usb online update\n");
	ret = bq->usb_psy->get_property(bq->usb_psy,
								POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		pr_err("could not read USB ONLINE property, ret=%d\n", ret);
	else
		pr_info("usb online status =%d\n", prop.intval);

	ret = 0;
	pr_info("bq->usb_present:%d,bq->charging_disabled_status:%d,prop.intval:%d\n",
		bq->usb_present, bq->charging_disabled_status, prop.intval);
	if (bq->usb_present && !bq->charging_disabled_status
			/* && bq->usb_psy_ma != 0 */) {
		if (prop.intval == 0) {
			pr_info("set usb online\n");
			ret = power_supply_set_online(bq->usb_psy, true);
		}
	} else {
		if (prop.intval == 1) {
			pr_info("set usb offline\n");
			ret = power_supply_set_online(bq->usb_psy, false);
		}
	}

	if (ret < 0)
		pr_info("could not set usb online state, ret=%d\n", ret);

	return 0;
}

static void bq2589x_convert_vbus_type(struct bq2589x_chip *bq)
{
	switch (bq->vbus_type) {
	case BQ2589X_VBUS_USB_SDP:
		bq->usb_type = POWER_SUPPLY_TYPE_USB;
		break;
	case BQ2589X_VBUS_USB_CDP:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case BQ2589X_VBUS_USB_DCP:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case BQ2589X_VBUS_MAXC:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_HVDCP;
		break;
	case BQ2589X_VBUS_NONSTAND:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case BQ2589X_VBUS_UNKNOWN:
		bq->usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	default:
		bq->usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}
}

static void bq2589x_get_hvdcp_profile(struct bq2589x_chip *bq)
{
	u16 vbus_volt;

	vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	if (vbus_volt <= 5300) {
		bq->icl_ma = 500;
		bq->ivl_mv = 4600;
		bq->chg_ma = 500;
	} else if (vbus_volt <= 9500) {
		bq->icl_ma = 2000;
		bq->ivl_mv = vbus_volt - 1200;
		bq->chg_ma = 3000;
	} else {
		bq->icl_ma = 1500;
		bq->ivl_mv = vbus_volt - 1200;
		bq->chg_ma = 3000;
	}
}

static void bq2589x_adapter_in_handler(struct bq2589x_chip *bq)
{
	int ret;
	bool update_profile = true;

	bq2589x_convert_vbus_type(bq);
	pr_info("vbus_type:%d, usb_type:%d\n", bq->vbus_type, bq->usb_type);
	bq2589x_adc_start(bq, false);
	switch (bq->usb_type) {
	case POWER_SUPPLY_TYPE_USB:
		bq2589x_dpdm_to_ap(bq);
		bq->icl_ma = 500;
		bq->ivl_mv = 4600;
		bq->chg_ma = 500;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		bq2589x_dpdm_to_ap(bq);
		bq->icl_ma = 1500;
		bq->ivl_mv = 4600;
		bq->chg_ma = 1500;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		bq->icl_ma = 2000;
		bq->ivl_mv = 4600;
		bq->chg_ma = 2000;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		bq2589x_get_hvdcp_profile(bq);
		break;
	case POWER_SUPPLY_TYPE_USB_ACA:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_ACA;
		bq->icl_ma = 1000;
		bq->ivl_mv = 4600;
		bq->chg_ma = 1000;
		break;
	case POWER_SUPPLY_TYPE_UNKNOWN:
		bq->icl_ma = 500;
		bq->ivl_mv = 4600;
		bq->chg_ma = 500;
		break;
	default:
		break;
	}

	ret = power_supply_set_supply_type(bq->usb_psy, bq->usb_type);
	pr_info("set supply type:%d %s\n", bq->usb_type,
			!ret ? "successfully" : "failed");

	ret = power_supply_set_present(bq->usb_psy, bq->usb_present);
	pr_info("set usb present:%d %s\n", bq->usb_present,
			!ret ? "successfully" : "failed");

	pr_info("chg_mv1:%d,chg_ma:%d,icl:%d,ivl:%d,update_profile:%d\n",
						bq->chg_mv, bq->chg_ma, bq->icl_ma, bq->ivl_mv, update_profile);
	if (update_profile)
		bq2589x_update_charging_profile(bq);

	ret = bq2589x_force_ico(bq);
	if (!ret) {
		schedule_delayed_work(&bq->ico_work, 2 * HZ);
		pr_info("Force ICO successfully\n");
	} else {
		pr_err("Force ICO failed\n");
	}

	cancel_delayed_work(&bq->discharge_jeita_work);

	if (bq->software_jeita_supported) {
		ret = alarm_start_relative(&bq->jeita_alarm,
				ns_to_ktime(calculate_jeita_poll_interval(bq) * 1000000000LL));
		if (ret)
			pr_err("start alarm for JEITA detection failed, ret=%d\n", ret);
	}

	bq2589x_set_watchdog_timer(bq, 80);
}

static void bq2589x_adapter_out_handler(struct bq2589x_chip *bq)
{
	power_supply_set_supply_type(bq->usb_psy, POWER_SUPPLY_TYPE_UNKNOWN);
	power_supply_set_present(bq->usb_psy, bq->usb_present);
	power_supply_set_online(bq->usb_psy, false);
	bq2589x_adc_stop(bq);

	bq2589x_dpdm_to_bq(bq);

	if (bq->software_jeita_supported) {
		alarm_try_to_cancel(&bq->jeita_alarm);
	}

	bq2589x_disable_watchdog_timer(bq);
	schedule_delayed_work(&bq->discharge_jeita_work,
			calculate_jeita_poll_interval(bq) * HZ);

	pr_info("usb removed, set usb present = %d\n", bq->usb_present);
}

static const unsigned char *charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};

static void bq2589x_dump_reg(struct bq2589x_chip *bq)
{
	int ret;
	int addr;
	u8 val;

	for (addr = 0x0; addr <= 0x14; addr++) {
		msleep(20);
		ret = bq2589x_read_byte(bq, &val, addr);
		if (ret == 0)
			pr_err("Reg[%02X] = 0x%02X\n", addr, val);
		pr_info("Reg[%02X] = 0x%02X\n", addr, val);
	}

}

static void bq2589x_dump_status(struct bq2589x_chip *bq)
{
	int ret;
	u8 status, fault;
	int chg_current;

	bq->vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	bq->vbat_volt = bq2589x_adc_read_battery_volt(bq);
	chg_current = bq2589x_adc_read_charge_current(bq);

	pr_info("vbus:%d,vbat:%d,ibat:%d\n", bq->vbus_volt,
					bq->vbat_volt, chg_current);

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
	if (!ret) {
		if (status & BQ2589X_VDPM_STAT_MASK)
			dev_info(bq->dev, "%s:VINDPM occurred\n", __func__);
		if (status & BQ2589X_IDPM_STAT_MASK)
			dev_info(bq->dev, "%s:IINDPM occurred\n", __func__);
	}


	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (!ret) {
		mutex_lock(&bq->data_lock);
		bq->fault_status = fault;
		mutex_unlock(&bq->data_lock);
	}

	if (bq->fault_status & BQ2589X_FAULT_WDT_MASK)
		pr_err("Watchdog timer expired!\n");
	if (bq->fault_status & BQ2589X_FAULT_BOOST_MASK)
		pr_err("Boost fault occurred!\n");

	status = (bq->fault_status & BQ2589X_FAULT_CHRG_MASK) >> BQ2589X_FAULT_CHRG_SHIFT;
	if (status == BQ2589X_FAULT_CHRG_INPUT)
		pr_err("input fault!\n");
	else if (status == BQ2589X_FAULT_CHRG_THERMAL)
		pr_err("charge thermal shutdown fault!\n");
	else if (status == BQ2589X_FAULT_CHRG_TIMER)
		pr_err("charge timer expired fault!\n");

	if (bq->fault_status & BQ2589X_FAULT_BAT_MASK)
		pr_err("battery ovp fault!\n");

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (!ret) {
		bq->charge_state = status & BQ2589X_CHRG_STAT_MASK;
		bq->charge_state >>= BQ2589X_CHRG_STAT_SHIFT;
		pr_err("%s\n", charge_stat_str[bq->charge_state]);
	}

	bq2589x_dump_reg(bq);
}

static void bq2589x_ico_workfunc(struct work_struct *work)
{
	struct bq2589x_chip *bq = container_of(work, struct bq2589x_chip, ico_work.work);
	int ret;

	ret = bq2589x_check_ico_done(bq);

	if (ret == 1) {
		ret = bq2589x_read_idpm_limit(bq);
		if (ret < 0)
			pr_err("ICO done, but failed to read idmp limit:%d\n", ret);
		else
			pr_info("ICO done, idpm limit = %dmA\n", ret);
	} else {
		schedule_delayed_work(&bq->ico_work, 2 * HZ);
	}

}

static void bq2589x_dpdm_to_ap(struct bq2589x_chip *bq)
{
	gpio_direction_output(bq->cfg.dpdm_sw_gpio, 0);
	pr_info("dpdm switch to AP\n");
}

static void bq2589x_dpdm_to_bq(struct bq2589x_chip *bq)
{
	gpio_direction_output(bq->cfg.dpdm_sw_gpio, 1);
	pr_info("dpdm switch to charger\n");

}

static irqreturn_t bq2589x_usb_id_interrupt(int irq, void *data)
{
	int usb_id_status;
	struct bq2589x_chip *bq = data;

	msleep(100);
	usb_id_status = gpio_get_value(bq->cfg.usb_id_gpio);

	power_supply_set_usb_otg(bq->usb_psy, !usb_id_status);

	pr_info("OTG device %s\n",
				!usb_id_status ? "inserted" : "removed");

	return IRQ_HANDLED;
}

static irqreturn_t bq2589x_charger_interrupt(int irq, void *data)
{
	int ret;
	struct bq2589x_chip *bq = data;
	u8 status = 0;
	u8 fault = 0;

	pr_info("enter  interrupt\n");

	msleep(20);

	bq2589x_dump_reg(bq);

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		dev_dbg(bq->dev, "IRQ triggered before device-resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;


	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) {
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret) {
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}

	mutex_lock(&bq->data_lock);
	bq->fault_status = fault;
	mutex_unlock(&bq->data_lock);

	/*bq->vbus_type = (status & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;

	if (((bq->vbus_type == BQ2589X_VBUS_NONE) || (bq->vbus_type == BQ2589X_VBUS_OTG))
			&& bq->usb_present) {
		pr_err("adapter removed\n");
		bq->usb_present = false;
		bq2589x_adapter_out_handler(bq);
	} else if (((bq->vbus_type != BQ2589X_VBUS_NONE) && (bq->vbus_type != BQ2589X_VBUS_OTG))
			&& !bq->usb_present) {
		pr_err("adapter plugged in\n");
		bq->usb_present = true;
		bq2589x_adapter_in_handler(bq);
	}*/

	if (((status & BQ2589X_PG_STAT_MASK) >> BQ2589X_PG_STAT_SHIFT)) {
		pr_info("adapter plugged in\n");
		bq->usb_present = true;
		bq2589x_adapter_in_handler(bq);
	} else {
		pr_info("adapter removed\n");
		bq->usb_present = false;
		bq2589x_adapter_out_handler(bq);
	}

	mutex_unlock(&bq->irq_complete);

	power_supply_changed(&bq->batt_psy);

	pr_info("exit interrupt\n");

	return IRQ_HANDLED;
}

static void determine_initial_status(struct bq2589x_chip *bq)
{
	int ret;
	u8 status;

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	pr_info("determine_initial_status:%d\n", status);
	if (!ret && (status & BQ2589X_PG_STAT_MASK)) {
		bq2589x_dpdm_to_bq(bq);
		msleep(20);
		bq2589x_force_dpdm(bq);
	}
}

#ifdef DBG_FS
static int show_registers(struct seq_file *m, void *data)
{
	struct bq2589x_chip *bq = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, &val, addr);
		if (!ret)
			seq_printf(m, "Reg[0x%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2589x_chip *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}


static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct bq2589x_chip *bq)
{
	bq->debug_root = debugfs_create_dir("bq2589x_chip", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
						bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("charging_disable_status", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charging_disabled_status));

		debugfs_create_x32("vbus_type", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->vbus_type));

		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charge_state));

		debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_writes));
	}
}
#endif

static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq2589x_chip *bq = container_of(dwork, struct bq2589x_chip, update_heartbeat_work);

	int temperature, voltage, capacity, status;
	int charge_type, battery_present, chg_current, batt_health;

	temperature = bq->batt_temp;
	voltage = bq2589x_get_prop_batt_voltage(bq);
	capacity = bq2589x_get_prop_capacity(bq);
	status = bq2589x_get_prop_charge_status(bq);
	charge_type = bq2589x_get_prop_charge_type(bq);
	battery_present = bq->batt_present;
	chg_current = bq2589x_get_prop_current_now(bq);
	batt_health = bq2589x_get_prop_health(bq);

	pr_info("batt_health=%d(1 good,2 overheat,9 warm,6 cold,10 cool,11 hot).\n", batt_health);
	pr_info("batt_status=%d(1->chg, 2->dischg, 4->full)", status);
	pr_info("chg_state=%d(0->unknown, 1->none, 2-> trickle, 3->fast, 4->taper)\n", charge_type);
	pr_info("batt present=%d\n", battery_present);
	pr_info("capacity=%d voltage=%d temperature=%d chg_current=%d\n",
			capacity, voltage, temperature, chg_current);

	schedule_delayed_work(&bq->update_heartbeat_work, msecs_to_jiffies(30000));
}

static int bq2589x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x_chip *bq;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	int irq_usb_id = 0;
	int ret;

	pr_info("bq2589x_charger_probe enter.\n");
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("ti_bms");
	if (!bms_psy) {
		dev_dbg(&client->dev, "bms supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x_chip), GFP_KERNEL);
	if (!bq) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	bq->usb_psy = usb_psy;
	bq->bms_psy = bms_psy;

	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	ret = bq2589x_detect_device(bq);
	if (!ret && bq->device_type == BQ25898D) {
		pr_info("charger device bq25898D detected, revision:%d\n",
									bq->revision);
	} else {
		pr_info("no bq25898D charger device found:%d\n", ret);
		devm_kfree(&client->dev, bq);
		return -ENODEV;
	}

	if (client->dev.of_node) {
		pr_info("zty_debug parse dt_version_0");
		bq2589x_parse_dt(&client->dev, bq);
	}

	ret = bq2589x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_0;
	}

	ret = bq2589x_psy_register(bq);
	if (ret)
		goto err_0;

	/*Enable the bq25898d power ldo,but we do not need do it*/
	ret = bq2589x_regulator_init(bq);
	if (ret) {
		pr_err("Couldn't initialize bq2589x_chip regulator ret=%d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&bq->charge_jeita_work, bq2589x_charge_jeita_workfunc);
	INIT_DELAYED_WORK(&bq->discharge_jeita_work, bq2589x_discharge_jeita_workfunc);
	INIT_DELAYED_WORK(&bq->ico_work, bq2589x_ico_workfunc);
	INIT_DELAYED_WORK(&bq->update_heartbeat_work, update_heartbeat);

	bq2589x_init_jeita(bq);

	alarm_init(&bq->jeita_alarm, ALARM_BOOTTIME, bq2589x_jeita_alarm_cb);
#if 0 /*Make the DPDM by PMIC,but we do not need now,maybe use it in the future*/
	if (gpio_is_valid(bq->cfg.dpdm_sw_gpio)) {
		ret = devm_gpio_request(bq->dev, bq->cfg.dpdm_sw_gpio,
										"bq2589x_dpdm_switch");
		if (ret) {
			pr_err("Failed to request dpdm switch gpio:%d\n", ret);
		}
	}
#endif
	/*USB id gpio,but we do not need now,maybe use it in the future
	what is USB id gpio*/

	if (gpio_is_valid(bq->cfg.usb_id_gpio)) {
		ret = devm_gpio_request(bq->dev, bq->cfg.usb_id_gpio, "usb_id_gpio");
		if (ret) {
			pr_err("failed to request usbid gpio\n");
			goto err_irq;
		} else
			irq_usb_id = gpio_to_irq(bq->cfg.usb_id_gpio);
	}

	if (irq_usb_id) {
		ret = devm_request_threaded_irq(bq->dev, irq_usb_id, NULL,
				bq2589x_usb_id_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"bq2589x_usb_id_irq", bq);
		if (ret) {
			pr_err("Request IRQ %d failed:%d\n", irq_usb_id, ret);
			goto err_irq;
		} else {
			pr_info("usb id irq = %d\n", irq_usb_id);
		}
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, bq2589x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2589x_charger1_irq", bq);
		if (ret) {
			pr_err("Request IRQ %d failed: %d\n", client->irq, ret);
			goto err_irq;
		} else {
			pr_info("irq = %d\n", client->irq);
		}
		enable_irq_wake(client->irq);
	}

	bq2589x_wakeup_src_init(bq);

	device_init_wakeup(bq->dev, true);

	bq2589x_dump_reg(bq);

#ifdef DBG_FS
	create_debugfs_entry(bq);
#endif
	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		pr_err("failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}

	determine_initial_status(bq);
	schedule_delayed_work(&bq->update_heartbeat_work, msecs_to_jiffies(5000));
	pr_err("bq2589x_chip probe successfully\n");

	return 0;

err_irq:

err_0:
	bq2589x_psy_unregister(bq);
	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->irq_complete);

	return ret;
}

static inline bool is_device_suspended(struct bq2589x_chip *bq)
{
	return !bq->resume_completed;
}

static int bq2589x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x_chip *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	return 0;
}

static int bq2589x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x_chip *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq2589x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x_chip *bq = i2c_get_clientdata(client);


	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		bq2589x_charger_interrupt(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(&bq->batt_psy);

	return 0;
}

static int bq2589x_charger_remove(struct i2c_client *client)
{
	struct bq2589x_chip *bq = i2c_get_clientdata(client);

	alarm_try_to_cancel(&bq->jeita_alarm);

	cancel_delayed_work_sync(&bq->charge_jeita_work);
	cancel_delayed_work_sync(&bq->discharge_jeita_work);
	cancel_delayed_work_sync(&bq->update_heartbeat_work);

	regulator_unregister(bq->otg_vreg.rdev);

	bq2589x_psy_unregister(bq);

	cancel_delayed_work(&bq->ico_work);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->irq_complete);

#ifdef DBG_FS
	debugfs_remove_recursive(bq->debug_root);
#endif

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);

	return 0;
}

static void bq2589x_charger_shutdown(struct i2c_client *client)
{
	pr_info("shutdown..");
}

static struct of_device_id bq2589x_charger_match_table[] = {
	{.compatible = "ti,bq2589x",},
	{},
};


static const struct i2c_device_id bq2589x_charger_id[] = {
	{ "bq2589x", BQ25898 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2589x_charger_id);

const struct dev_pm_ops bq2589x_pm = {
	.suspend = bq2589x_suspend,
	.resume = bq2589x_resume,
};

static const struct dev_pm_ops bq2589x_pm_ops = {
	.resume		= bq2589x_resume,
	.suspend_noirq = bq2589x_suspend_noirq,
	.suspend	= bq2589x_suspend,
};

static struct i2c_driver bq2589x_charger_driver = {
	.driver		= {
		.name	= "bq25898d",
		.of_match_table = bq2589x_charger_match_table,
		.pm	= &bq2589x_pm_ops,
	},
	.id_table	= bq2589x_charger_id,

	.probe		= bq2589x_charger_probe,
	.shutdown	= bq2589x_charger_shutdown,
	.remove		= bq2589x_charger_remove,
};

static int __init bq2589x_init(void)
{
	int ret;

	pr_info("%s()\n", __func__);
	ret = i2c_add_driver(&bq2589x_charger_driver);

	if (ret)
		pr_info("Unable to register ti2419x_init i2c driver\n");

	return ret;
}

static void __exit bq2589x_exit(void)
{
	pr_info("%s()\n", __func__);
	i2c_del_driver(&bq2589x_charger_driver);
}
late_initcall(bq2589x_init);


MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
