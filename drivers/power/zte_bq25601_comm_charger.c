/* Copyright (c) 2013-2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "[CHG] %s(%d): " fmt, __func__, __LINE__
#define DEBUG
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/wakelock.h>
#include <linux/reboot.h>	/*zte add:For kernel_power_off() */
#include <soc/qcom/socinfo.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/alarmtimer.h>
#include "zte_misc.h"
#include <soc/qcom/socinfo.h>

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
int syna_ts_notifier_call_chain(unsigned long val);
#endif

/*zte feature definition*/
#ifndef FEATURE_SOFT_CC
#define FEATURE_SOFT_CC 0
#define SOFT_CC_DEBUG 0
#endif
#define PRINT_TI24296_REG 1
#define _TIC_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define TIC_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_TIC_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

/* Charger Registers */
#define INPUT_SOURCE_CTRL_REG			0x00
#define IINLIMIT_MASK					TIC_MASK(2, 0)
#define IINLIMIT_SHIFT					0

/*for TI25601 IC begin*/
#define TI25601_IINLIMIT_MASK			TIC_MASK(4, 0)
#define TI25601_IINLIMIT_SHIFT			0
/*for TI25601 IC end*/

#define VINLIMIT_MASK					TIC_MASK(6, 3)
#define VINLIMIT_SHIFT					3
#define EN_HIZ_MASK						TIC_MASK(7, 7)
#define EN_HIZ_SHIFT					7

#define PON_CFG_REG						0x01
/* bit 0:  2419x: 0-500mA, 1-1.3A; 2x29x: 0-1A, 1-1.5A */
#define BOOST_LIM_MASK					TIC_MASK(0, 0)
#define BOOST_LIM_SHIFT					0
#define V_SYS_MIN_MASK					TIC_MASK(3, 1)
#define V_SYS_MIN_SHIFT					1
#define CHG_CFG_MASK					TIC_MASK(5, 4)
#define CHG_CFG_SHIFT					4
#define I2C_WDOG_RESET_MASK				TIC_MASK(6, 6)
#define I2C_WDOG_RESET_SHIFT			6
#define REGISTER_RESET_MASK				TIC_MASK(7, 7)
#define REGISTER_RESET_SHIFT			7

#define CHG_I_CTRL_REG					0x02
/* different between 2419x and 2429x, bit 1 */
#define FORCE_20PCT_MASK				TIC_MASK(0, 0)
#define FORCE_20PCT_SHIFT				0
#define IFAST_CHG_MASK					TIC_MASK(7, 2)
#define IFAST_CHG_SHIFT					2

/*for TI25601 IC begin*/
#define TI25601_IFAST_CHG_MASK			TIC_MASK(5, 0)
#define TI25601_IFAST_CHG_SHIFT			0
/*for TI25601 IC end*/

/* for 2429x */
#define TI2429X_BOOST_COLD_MASK			TIC_MASK(1, 1)
#define TI2429X_BOOST_COLD_SHIFT		1

#define IPRECHG_ITERM_CTRL_REG			0x03
#define ITERM_MASK						TIC_MASK(3, 0)
#define ITERM_SHIFT						0
#define IPRECHG_MASK					TIC_MASK(7, 4)
#define IPRECHG_SHIFT					4

#define VCHG_CTRL_REG					0x04
#define VRECHG_MASK						TIC_MASK(0, 0)
#define VRECHG_SHIFT					0
#define V_BAT_LOW_THRE_MASK				TIC_MASK(1, 1)
#define V_BAT_LOW_THRE_SHIFT			1
#define VCHG_MASK						TIC_MASK(7, 2)
#define VCHG_SHIFT						2

/*for TI25601 IC begin*/
#define TI25601_VCHG_MASK				TIC_MASK(7, 3)
#define TI25601_VCHG_SHIFT				3
/*for TI25601 IC end*/

#define CHG_TERM_TIMER_CTRL_REG			0x05
/* different between 2419x and 2429x, bit 0, bit 6 */
#define JEITA_ISET_MASK					TIC_MASK(0, 0)
#define JEITA_ISET_SHIFT				0
#define FAST_CHG_TIMER_MASK				TIC_MASK(2, 1)
#define FAST_CHG_TIMER_SHIFT			1

/*for TI25601 IC begin*/
#define TI25601_FAST_CHG_TIMER_MASK		TIC_MASK(2, 2)
#define TI25601_FAST_CHG_TIMER_SHIFT	2
/*for TI25601 IC end*/

#define SAFE_TIMER_EN_MASK				TIC_MASK(3, 3)
#define SAFE_TIMER_EN_SHIFT				3
#define I2C_WDOG_TIMER_MASK				TIC_MASK(5, 4)
#define I2C_WDOG_TIMER_SHIFT			4
#define TERM_STAT_MASK					TIC_MASK(6, 6)
#define TERM_STAT_SHIFT					6
#define EN_TERM_MASK					TIC_MASK(7, 7)
#define EN_TERM_SHIFT					7

#define IR_THERM_REG					0x06
/* different between 2419x and 2429x, bit 2~7 */
#define THERM_THRE_MASK					TIC_MASK(1, 0)
#define THERM_THRE_SHIFT				0
#define TI2419X_IR_COMP_V_MASK			TIC_MASK(4, 2)
#define TI2419X_IR_COMP_V_SHIFT			2
#define TI2419X_IR_COMP_R_MASK			TIC_MASK(7, 5)
#define TI2419X_IR_COMP_R_SHIFT			5
/* for 2429x */
#define TI2429X_BOOST_HOT_MASK			TIC_MASK(3, 2)
#define TI2429X_BOOST_HOT_SHIFT			2
#define TI2429X_BOOST_V_MASK			TIC_MASK(7, 4)
#define TI2429X_BOOST_V_SHIFT			4

/*for TI25601 IC begin*/
#define TI25601_VINLIMIT_MASK			TIC_MASK(3, 0)
#define TI25601_VINLIMIT_SHIFT			0
/*for TI25601 IC end*/

#define MISC_OPERA_CTRL_REG				0x07
/* different between 2419x and 2429x, bit 4 */
#define INT_MASK						TIC_MASK(1, 0)
#define INT_SHIFT						0
#define JEITA_VSET_MASK					TIC_MASK(4, 4)
#define JEITA_VSET_SHIFT				4
#define BATFET_DISABLE_MASK				TIC_MASK(5, 5)
#define BATFET_DISABLE_SHIFT			5
#define TMR2X_EN_MASK					TIC_MASK(6, 6)
#define TMR2X_EN_SHIFT					6
#define DPDM_EN_MASK					TIC_MASK(7, 7)
#define DPDM_EN_SHIFT					7

#define SYS_STAT_REG					0x08
#define VSYS_STAT_MASK					TIC_MASK(0, 0)
#define VSYS_STAT_SHIFT					0
#define THERM_STAT_MASK					TIC_MASK(1, 1)
#define THERM_STAT_SHIFT				1
#define PG_STAT_MASK					TIC_MASK(2, 2)
#define PG_STAT_SHIFT					2
#define DPM_STAT_MASK					TIC_MASK(3, 3)
#define DPM_STAT_SHIFT					3
#define CHG_STAT_MASK					TIC_MASK(5, 4)
#define CHG_STAT_SHIFT					4
#define VBUS_STAT_MASK					TIC_MASK(7, 6)
#define VBUS_STAT_SHIFT					6

/*for TI25601 IC begin*/
#define TI25601_CHG_STAT_MASK			TIC_MASK(4, 3)
#define TI25601_CHG_STAT_SHIFT			3
/*for TI25601 IC end*/

#define FAULT_REG						0x09
/* different between 2419x and 2429x, bit 0~2, bit 6 */
#define TI2419X_NTC_FAULT_MASK			TIC_MASK(2, 0)
#define TI2419X_NTC_FAULT_SHIFT			0
#define BAT_FAULT_MASK					TIC_MASK(3, 3)
#define BAT_FAULT_SHIFT					3
#define CHG_FAULT_MASK					TIC_MASK(5, 4)
#define CHG_FAULT_SHIFT					4
#define TI2419X_BOOST_FAULT_MASK		TIC_MASK(6, 6)
#define TI2419X_BOOST_FAULT_SHIFT		6
#define WDOG_FAULT_MASK					TIC_MASK(7, 7)
#define WDOG_FAULT_SHIFT				7
/* for 2429x */
#define TI2429X_NTC_FAULT_MASK			TIC_MASK(1, 0)
#define TI2429X_NTC_FAULT_SHIFT			0
#define TI2429X_OTG_FAULT_MASK			TIC_MASK(6, 6)
#define TI2429X_OTG_FAULT_SHIFT			6

#define VENDOR_REG						0x0A

/*for TI25601 IC begin*/
#define TI25601_VENDOR_REG				0x0B
/*for TI25601 IC end*/

/* for 2419x */
#define TI2419X_DEV_REG_MASK			TIC_MASK(1, 0)
#define TI2419X_TS_PROFILE_MASK			TIC_MASK(2, 2)
#define TI2419X_PN_MASK					TIC_MASK(5, 3)
/* for 2429x */
#define TI2429X_REVISION_MASK			TIC_MASK(2, 0)
#define TI2429X_PN_MASK					TIC_MASK(7, 5)

/*for TI25601 IC begin*/
#define TI25601_PN_MASK					TIC_MASK(6, 3)
#define TI25601PN_SHFIT					3
/*for TI25601 IC end*/

#define CHARGER_IC_2419X				0
#define CHARGER_IC_2429X				1
#define CHARGER_IC_25601				2

#if defined(CONFIG_BOARD_GEMI)
#define FUELGUAGE_MAXIM_17055			1
#endif

#if defined(CONFIG_BOARD_HELEN)
#define LED_GPIO_CONTROL				1
#endif
#define CFG_MIN_VOLTAGE_UV				3400000
#define TI_CHARGER_VENDOR					0x2
#define TI2419X_VENDOR_SHIFT			4
#define TI25601_VENDOR_SHIFT		3

#define BATT_NOT_CHG_VAL				0x0
#define BATT_PRE_CHG_VAL				0x1
#define BATT_FAST_CHG_VAL				0x2
#define BATT_CHG_DONE					0x3

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	TEMP = BIT(3),			/* temperature */
	SOC = BIT(3),
	POLICY = BIT(4),
};

/* ti chip charging status */
enum {
	NOT_CHARGING,
	PRE_CHARGE,
	FAST_CHARGE,
	CHARGE_TERM_DONE,
};

enum {
	TI_TEMP_COLD_STATE,
	TI_TEMP_COOL_STATE,
	TI_TEMP_NORMAL_STATE,
	TI_TEMP_WARM_STATE,
	TI_TEMP_HOT_STATE,
};

enum {
	TI_24X9X_CHARGER_IC,
	TI_25601_CHARGER_IC,
};

static int ic_type = -1;

/* for usb-otg */
struct ti2419x_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};
static char *pm_batt_supplied_to[] = {
	"bms",
};

struct ti2419x_chip {
	struct i2c_client			   *client;
	struct device				   *dev;
	unsigned short					default_i2c_addr;

	/* configuration data - charger */
	int								fake_battery_soc;	/* soc set from user */
	bool							charging_disabled;
	bool							hw_iterm_disabled;
	int								iterm_ma;
	int								vfloat_mv;
	int								safety_time;
	int								resume_delta_mv;
	int								warm_resume_delta_mv;
	int								vbatdet_max_err_mv;
	unsigned int					thermal_levels;
	unsigned int					therm_lvl_sel;
	unsigned int					*thermal_mitigation;

	/* status tracking */
	bool							usb_present;
	bool							batt_present;	/* how to check battery present */
	bool							bat_is_cool;	/* zte */
	bool							bat_is_cold;	/* zte */
	bool							bat_is_warm;
	bool							bat_is_hot;	/* zte */
	unsigned int					hot_batt_p;	/* ZTE, for PMIC batt therm correcting */
	unsigned int					cold_batt_p;	/* ZTE, for PMIC batt therm correcting */
	int								tm_state;
	bool							batt_full;
	bool							batt_warm_full;
	bool							batt_real_full;
	bool							chg_done;
	bool							resume_completed;
	bool							irq_waiting;
	u8								irq_cfg_mask[3];
	int								usb_psy_ma;
	int								charging_disabled_status;
	int								max_iusb;
	int								max_ibat;
	int								current_ibat;
	int								max_input_voltage;

	int								skip_writes;
	int								skip_reads;
	u8								reg_addr;
	struct dentry					*debug_root;
	struct qpnp_vadc_chip			*vadc_dev;
	struct power_supply				*usb_psy;
	struct power_supply				batt_psy;
	struct power_supply				*bms_psy;
	struct ti2419x_otg_regulator	otg_vreg;	/* for usb-otg */
	struct mutex					irq_complete;
	struct mutex					charging_disable_lock;
	struct mutex					current_change_lock;
	struct mutex					read_write_lock;
	struct mutex					jeita_configure_lock;
	struct mutex					chg_enable_lock;

	struct delayed_work				update_heartbeat_work;
	struct delayed_work				charger_eoc_work;
	struct delayed_work				temp_control_work;
#if FEATURE_SOFT_CC
	struct delayed_work				soft_cc_monitor_work;	/* zte add */
#endif
	struct work_struct				poweroff_work;	/* ZTE */
	struct wake_lock				charger_wake_lock;	/* zte add */
	struct wake_lock				charger_valid_lock;	/* zte add */
	int								chargeIC_type;

	unsigned int					warm_bat_mv;
	unsigned int					cool_bat_mv;
	unsigned int					warm_bat_chg_ma;
	unsigned int					cool_bat_chg_ma;
	unsigned int					cooler_bat_chg_ma;
	int								warm_bat_decidegc;
	int								cool_bat_decidegc;
	int								cooler_bat_decidegc;
	int								hot_bat_decidegc;
	int								cold_bat_decidegc;
	int								low_temp_threshold;
	int								high_temp_threshold;
	int								health;
	int								soc;
	bool							on_bms;
	bool							mx_bms;
	bool							ti_bms;
	bool							vm_bms;
	unsigned int						cfg_max_voltage_mv;
	struct alarm				charging_expired_alarm;
	struct charging_policy_ops		battery_charging_policy_ops;
	struct delayed_work			charging_policy_work;
	struct delayed_work			usb_type_detection_work;
	int					policy_usb_psy_ma;
};

struct ti2419x_chip *the_ti2419x_chip;

static int chg_time[] = {
	5,	/* hours */
	8,
	12,
	20,
};

static int input_current_limit[] = {
	100, 150, 500, 900, 1000, 1500, 2000, 3000,
};
static int ichg_current_limit[] = {
	512, 512, 512, 1152, 1152, 1664, 2048, 3008,
};

static int ti25601_chg_time[] = {
	5,	/* hours */
	10,
};

static int ti25601_input_current_limit[] = {
	100, 200, 300, 400, 500, 600, 700, 800, 900, 1000,
	1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000,
	2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000,
	3100, 3200,
};
static int ti25601_ichg_current_limit[] = {
	120, 240, 360, 420, 540, 660, 720, 840, 960, 1020,
	1140, 1260, 1320, 1440, 1560, 1620, 1740, 1860, 1920, 2040,
	2160, 2220, 2340, 2460, 2520, 2640, 2760, 2820, 2940, 3000,
	3000, 3000,
};

#define LOW_SOC_HEARTBEAT_MS  	20000
#define HEARTBEAT_MS		  	60000

#ifdef ZTE_CHARGER_TYPE_OEM
#define HEARTBEAT_CHARGER_TYPE_OEM_MS 2000
#define DETECT_CHARGER_TYPE_OME_COUNTER 5
extern enum charger_types_oem charge_type_oem;
int g_update_period_ms = HEARTBEAT_MS;
int g_low_soc_update_period_ms = LOW_SOC_HEARTBEAT_MS;
int g_charger_detect_counter = 0;
#endif

static void
offcharge_poweroff_work(struct work_struct *work)
{
	pr_info("%s,ZTE shutdown for charger remove at offcharging mode\n", __func__);

	kernel_power_off();   /* ZTE */
	/* kernel_restart(NULL); */
}


static int __ti2419x_read(struct ti2419x_chip *chip, int reg,
				u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	}
	*val = ret;
	return 0;
}

static int __ti2419x_write(struct ti2419x_chip *chip, int reg,
						u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	pr_debug("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}

static int ti2419x_read(struct ti2419x_chip *chip, int reg,
				u8 *val)
{
	int rc;

	if (chip->skip_reads) {
		*val = 0;
		return 0;
	}
	mutex_lock(&chip->read_write_lock);
	rc = __ti2419x_read(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int ti2419x_masked_write(struct ti2419x_chip *chip, int reg,
						u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	if (chip->skip_writes || chip->skip_reads)
		return 0;

	mutex_lock(&chip->read_write_lock);
	rc = __ti2419x_read(chip, reg, &temp);
	if (rc < 0) {
		dev_err(chip->dev, "read failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __ti2419x_write(chip, reg, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"write failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int
ti2419x_is_chg_plugged_in(struct ti2419x_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = ti2419x_read(chip, SYS_STAT_REG, &reg);
	if (rc) {
		pr_err("Couldn't read SYS_STAT_REG rc=%d\n", rc);
		return 0;
	}

	pr_debug("chgr usb sts %d\n", (reg & PG_STAT_MASK) ? 1 : 0);

	return (reg & PG_STAT_MASK) ? 1 : 0;
}

#define MIN_FLOAT_MV		3504
#define MAX_FLOAT_MV		4400
#define VFLOAT_STEP_MV		16

#define TI25601_MIN_FLOAT_MV		3856
#define TI25601_MAX_FLOAT_MV		4400
#define TI25601_VFLOAT_STEP_MV		32
static int ti2419x_float_voltage_set(struct ti2419x_chip *chip, int vfloat_mv)
{
	u8 temp;

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
			dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
			return -EINVAL;
		}

		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

		return ti2419x_masked_write(chip, VCHG_CTRL_REG,
				VCHG_MASK, temp << VCHG_SHIFT);
	case TI_25601_CHARGER_IC:
		if ((vfloat_mv < TI25601_MIN_FLOAT_MV) || (vfloat_mv > TI25601_MAX_FLOAT_MV)) {
			dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
			return -EINVAL;
		}

		temp = (vfloat_mv - TI25601_MIN_FLOAT_MV) / TI25601_VFLOAT_STEP_MV;

		return ti2419x_masked_write(chip, VCHG_CTRL_REG,
				TI25601_VCHG_MASK, temp << TI25601_VCHG_SHIFT);
	}
	return -EINVAL;
}

static int ti2419x_float_voltage_get(struct ti2419x_chip *chip, int *vfloat_mv)
{
	int rc;
	u8 reg = 0;
	int vol = 0;

	rc = ti2419x_read(chip, VCHG_CTRL_REG, &reg);
	if (rc) {
		pr_err("Couldn't read SYS_STAT_REG rc=%d\n", rc);
		return -EINVAL;
	}

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		reg &= VCHG_MASK;
		reg >>= VCHG_SHIFT;

		vol = reg * VFLOAT_STEP_MV + MIN_FLOAT_MV;
		if ((vol < MIN_FLOAT_MV) || (vol > MAX_FLOAT_MV)) {
			dev_err(chip->dev, "bad float voltage mv =%d\n", vol);
			return -EINVAL;
		}
		break;
	case TI_25601_CHARGER_IC:
		reg &= TI25601_VCHG_MASK;
		reg >>= TI25601_VCHG_SHIFT;

		vol = reg * TI25601_VFLOAT_STEP_MV + TI25601_MIN_FLOAT_MV;
		if ((vol < TI25601_MIN_FLOAT_MV) || (vol > TI25601_MAX_FLOAT_MV)) {
			dev_err(chip->dev, "bad float voltage mv =%d\n", vol);
			return -EINVAL;
		}
		break;
	}

	*vfloat_mv = vol;

	return 0;
}

#define MIN_RECHG_MV		50
#define MAX_RECHG_MV		300
static int ti2419x_recharge_threshold_set(struct ti2419x_chip *chip,
							int resume_mv)
{
	u8 temp;

	if ((resume_mv < MIN_RECHG_MV) || (resume_mv > MAX_RECHG_MV)) {
		dev_err(chip->dev, "bad rechg_thrsh =%d asked to set\n",
							resume_mv);
		return -EINVAL;
	}

	temp = resume_mv / MAX_RECHG_MV;

	return ti2419x_masked_write(chip, VCHG_CTRL_REG,
		VRECHG_MASK, temp << VRECHG_SHIFT);
}

static int __ti2419x_charging_disable(struct ti2419x_chip *chip, bool disable)
{
	int rc;

	rc = ti2419x_masked_write(chip, PON_CFG_REG,
			CHG_CFG_MASK, disable ? 0 : 1 << CHG_CFG_SHIFT);
	if (rc < 0)
		pr_err("Couldn't set CHG_CFG disable=%d rc = %d\n",
							disable, rc);
	else
		pr_debug("CHG_CFG status=%d\n", !disable);

	return rc;
}

static int ti2419x_charging_disable(struct ti2419x_chip *chip, int reason,
								int disable)
{
	int rc = 0;
	int disabled;

	mutex_lock(&chip->charging_disable_lock);

	disabled = chip->charging_disabled_status;

	pr_info("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled)
		rc = __ti2419x_charging_disable(chip, true);
	else
		rc = __ti2419x_charging_disable(chip, false);

	if (rc)
		pr_err("Couldn't disable charging for reason=%d rc=%d\n",
							rc, reason);
	else
		chip->charging_disabled_status = disabled;

	mutex_unlock(&chip->charging_disable_lock);

	return rc;
}

static void ti2419x_set_appropriate_float_voltage(struct ti2419x_chip *chip)
{
	pr_info("vfloat_mv=%dmv warm_resume_delta_mv=%dmv resume_delta_mv=%dmv\n",
			chip->vfloat_mv,
			chip->warm_resume_delta_mv,
			chip->resume_delta_mv
			);

	pr_info("is_cold=%d is_cool=%d is_warm=%d cool_bat_mv=%dmv warm_bat_mv=%dmv\n",
			chip->bat_is_cold,
			chip->bat_is_cool,
			chip->bat_is_warm,
			chip->cool_bat_mv,
			chip->warm_bat_mv
			);
	if (chip->bat_is_cool) {
		ti2419x_float_voltage_set(chip, chip->cool_bat_mv);
		ti2419x_recharge_threshold_set(chip, chip->warm_resume_delta_mv);
	} else if (chip->bat_is_warm) {
		ti2419x_float_voltage_set(chip, chip->warm_bat_mv);
		ti2419x_recharge_threshold_set(chip, chip->warm_resume_delta_mv);
	} else {
		ti2419x_float_voltage_set(chip, chip->vfloat_mv);
		ti2419x_recharge_threshold_set(chip, chip->resume_delta_mv);
	}
}

static enum power_supply_property ti2419x_battery_properties[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	/* POWER_SUPPLY_PROP_RESISTANCE, */
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_SHIPMODE,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,

};

extern int hw_version;
static int ti2419x_get_prop_batt_present(struct ti2419x_chip *chip)
{
	#ifdef CONFIG_BOARD_ABBY
	/* Note(JZN)20151012:
	 * return TRUE for hw board error in hw version 0
	 */
	 /* pr_debug("P890A57 hw_version:0x%02x\n",hw_version); */
	if (hw_version == 0) {
		chip->batt_present = 1;
		return chip->batt_present;
	 }
	#endif
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_PRESENT, &ret);
		chip->batt_present = ret.intval;
	}

	return	chip->batt_present;
}

/*
  * return -1: error
  * return 0: not charge
  * return 1: pre charge
  * return 2: fast charge
  * return 3: charge term
  */
static int ti2419x_get_charging_status(struct ti2419x_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = ti2419x_read(chip, SYS_STAT_REG, &reg);
	if (rc) {
		pr_err("Couldn't read SYS_STAT_REG rc=%d\n", rc);
		return -EINVAL;
	}

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		reg &= CHG_STAT_MASK;
		reg >>= CHG_STAT_SHIFT;
		break;
	case TI_25601_CHARGER_IC:
		reg &= TI25601_CHG_STAT_MASK;
		reg >>= TI25601_CHG_STAT_SHIFT;
		break;
	}

	return reg;
}

static int ti2419x_get_prop_batt_capacity(struct ti2419x_chip *chip);
#define DEFAULT_STATUS 0
static int ti2419x_get_prop_batt_bms_status(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->ti_bms) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BMS_STATUS, &ret);
			return ret.intval;
		}
	}
	return DEFAULT_STATUS;
}

static int ti2419x_get_prop_batt_bms_flags(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->ti_bms) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BMS_FLAGS, &ret);
			return ret.intval;
		}
	}
	return DEFAULT_STATUS;
}
static int ti2419x_get_prop_batt_bms_remainingcapacity(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->ti_bms) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BMS_RC, &ret);
			return ret.intval;
		}
	}
	return DEFAULT_STATUS;
}

static int ti2419x_get_prop_batt_bms_qmax(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->ti_bms) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_QMAX, &ret);
			return ret.intval;
		}
	}
	return DEFAULT_STATUS;
}




#define DEFAULT_FCC 3000000
static int ti2419x_get_prop_batt_bms_fcc(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->vm_bms) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL, &ret);

			return ret.intval;
		}
	}
	return DEFAULT_FCC;
}


static int ti2419x_get_prop_batt_status(struct ti2419x_chip *chip)
{
	int is_chg_in, stat;

	is_chg_in = ti2419x_is_chg_plugged_in(chip);
	stat	  = ti2419x_get_charging_status(chip);
	if (stat < 0) {
		pr_err("Couldn't read SYS_STAT_REG.\n");
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
	if (chip->vm_bms) {
		if (is_chg_in && chip->batt_full)
			return POWER_SUPPLY_STATUS_FULL;
	} else {
		if (is_chg_in && (chip->soc == 100 || chip->batt_full))
			return POWER_SUPPLY_STATUS_FULL;
	}

	if (((stat == 0x1) || (stat == 0x2)) && is_chg_in)
		return POWER_SUPPLY_STATUS_CHARGING;

	if (stat == 0 || stat == 0x3)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int ti2419x_is_charging_enabled(struct ti2419x_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = ti2419x_read(chip, PON_CFG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read PON_CFG_REG rc=%d\n", rc);
		return 0;
	}

	reg &= CHG_CFG_MASK;
	reg >>= CHG_CFG_SHIFT;
	return (reg == 0x1) ? 1 : 0;
}

static int ti2419x_get_prop_charge_type(struct ti2419x_chip *chip)
{
	int rc;
	u8 reg = 0;
	u8 chg_type;

	rc = ti2419x_read(chip, SYS_STAT_REG, &reg);
	if (rc) {
		pr_err("Couldn't read SYS_STAT_REG rc=%d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		chg_type = (reg & CHG_STAT_MASK) >> CHG_STAT_SHIFT;
		break;
	case TI_25601_CHARGER_IC:
		chg_type = (reg & TI25601_CHG_STAT_MASK) >> TI25601_CHG_STAT_SHIFT;
		break;
	}

	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if ((chg_type == BATT_FAST_CHG_VAL) ||
			(chg_type == BATT_CHG_DONE))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

/* updated by temp control work */
static int ti2419x_get_prop_batt_health(struct ti2419x_chip *chip)
{
	if (chip->bat_is_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->bat_is_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	else if (chip->bat_is_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->bat_is_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static int zero_reported = false;
/* int enable_to_shutdown=1;*/
extern int enable_to_shutdown;
#define SHUTDOWN_VOLTAGE 3400000
#define LOW_TEMP_SHUTDOWN_VOLTAGE 3200000
int ti2419x_get_prop_voltage_now(struct ti2419x_chip *chip);
#if FEATURE_SOFT_CC
static int soft_cc_adjust_soc(struct ti2419x_chip *chip);
#endif
#define ZERO_REPORT_DELAY_DELTA (HZ*40*1) /* 40s */

static bool zero_report_check(void)
{
	static unsigned long  report_zero_jiffies = 0;
	bool temp;

	if (report_zero_jiffies == 0) {
		report_zero_jiffies = jiffies;
		pr_info("start check at %ld\n", report_zero_jiffies);
		temp = false;
	} else {
		pr_info("capacity zero\n");
		if (time_after(jiffies, report_zero_jiffies+ZERO_REPORT_DELAY_DELTA))
			temp = true;
		else
			temp = false;
	}
	return temp;
}

int smooth_capacity(struct ti2419x_chip *chip, int capacity)
{
	static bool turnon_flags = true;
	int cap = capacity;
	int batt_vol, is_chg_in;
	static int count = 0;

	if ((turnon_flags) && (capacity > 0))
		turnon_flags = false;

	pr_debug("CHG:before smooth capacity= %d\n", capacity);

	if (zero_reported) {
		pr_info("ZERO reported already, just return 0 directly\n");
		return 0;
	}
	if (chip->batt_full == true) {
		pr_info("[CHG]: batt_full=1; before smooth capacity= %d\n", capacity);
		return 100;
	}

	/* Note by zte JZN 20160301:
	* 1)if battery voltage continuously lower than SHUTDOWN_VOLTAGE for 5 times,
	*	 force capacity=0,to avoid over discharging of the battery
	* 2) it's especially useful in low temperature situation
	*/
	batt_vol   = ti2419x_get_prop_voltage_now(chip);
	is_chg_in = ti2419x_is_chg_plugged_in(chip);
	if ((batt_vol <=  LOW_TEMP_SHUTDOWN_VOLTAGE) &&  (is_chg_in == 0)) {
		count++;
		pr_info("[CHG]: batt voltage=%d,count=%d\n", batt_vol, count);
		if (count == 5) {
			pr_info("[CHG]: batt voltage is critically low,force soc=0\n");
			capacity = 0;
			count = 0;
		}

	} else
		count = 0;

	if (capacity == 0) {
		if (!enable_to_shutdown) {
			pr_debug("CHG: enable_to_shutdown=0,return soc=1\n");
			return 1;
		}

		if (batt_vol > SHUTDOWN_VOLTAGE) {
			pr_debug("CHG: vol is higher than shutdown vol,return soc=1\n");
			return 1;
		}

#if FEATURE_SOFT_CC
		/*
		* ZTE:when charger in and the soc=0,we need to do a 60s check for this situation.
		* If after 60s monitor, the soft_cc_toal is still positive, that means the charger
		* can't change in anymore, report zero to uplayer.
		*/
		if (ti2419x_is_chg_plugged_in(chip)) {
			/* return 0 or 1 */
			cap = soft_cc_adjust_soc(chip);
		}
		pr_debug("CHG:after soft cc capacity= %d\n", cap);
#endif
		if (capacity == 0) {
			if (!turnon_flags)
				zero_reported = true;
			else if (zero_report_check()) {
				zero_reported = true;
				turnon_flags = false;
			} else
				cap = 1;
		}
	}

	pr_debug("CHG:after smooth capacity= %d\n", cap);
	return cap;
}
/* zte jiangfeng add, end */

/* ZTE add for CTS*/
#define DEFAULT_BATT_CHARGE_COUNTER 1
static int ti2419x_get_prop_charge_counter(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CYCLE_COUNT, &ret);
	}

	if (ret.intval <= 0)
		ret.intval = DEFAULT_BATT_CHARGE_COUNTER;
	return ret.intval;
}

/* static int enable_to_shutdown=1;    //ZTE add */
#define DEFAULT_CAPACITY	50
/* static bool report_zero = false;   //ZTE */
static int ti2419x_get_prop_batt_capacity_real(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		return ret.intval;
	}

	return DEFAULT_CAPACITY;
}

/* need get from bms */
static int ti2419x_get_prop_batt_capacity(struct ti2419x_chip *chip)
{
	chip->soc = ti2419x_get_prop_batt_capacity_real(chip);
	return smooth_capacity(chip, chip->soc);
}

/* ITE(INDICATOR_TO_EMPTY):this parameter is only for ON BMS */
static int ti2419x_get_prop_batt_ite(struct ti2419x_chip *chip)
{

	union power_supply_propval ret = {0,};

	if (chip->on_bms) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CAPACITY_LEVEL, &ret);
			return	ret.intval;
		}
	}
	return 0;
}


static int ti2419x_get_prop_chg_full_design(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if ((chip->on_bms) || (chip->ti_bms)) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &ret);
			return ret.intval;
		}
	}
	return DEFAULT_FCC;
}

#define DEFAULT_TEMP		250
static int
ti2419x_get_prop_batt_temp(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_TEMP, &ret);
		return ret.intval;
	}
			return DEFAULT_TEMP;
}


#define DEFAULT_VOLTAGE		3700000
int ti2419x_get_prop_voltage_now(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
		return ret.intval;
	}

	return DEFAULT_VOLTAGE;
}

static int ti2419x_get_prop_batt_voltage_avg(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->mx_bms) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_VOLTAGE_AVG, &ret);
			return ret.intval/1000;
		}
	}
	return DEFAULT_VOLTAGE;
}

/* return uA */
static int ti2419x_get_prop_current_now(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	}

	return 0;
}
int set_ibat(struct ti2419x_chip *chip, int ma);

static int ti2419x_enable_ship_mode(struct ti2419x_chip *chip, int enable)
{
	int rc = -1;

	/* NOTE:
	*  this function is for TI BQ24298,to enable ship mode:
	*  step 1:disable watchdog timer(REG05[5:4]=00)
	*  step 2:disabling BATFET (REG07[5] bit=1)
	*  by doing this,BATFET will be off after 7.5s
	*/
	if (enable == 0x0) {
		rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
				I2C_WDOG_TIMER_MASK, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable WDOG timer rc=%d\n", rc);
			return rc;
		}

		rc = ti2419x_masked_write(chip, MISC_OPERA_CTRL_REG,
				BATFET_DISABLE_MASK, 1<<BATFET_DISABLE_SHIFT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't turn OFF BATTFET rc=%d\n", rc);
			return rc;
		}
	} else {
		pr_info("write ship_mode enable=%d\n", enable);
		return rc;
	}

	return 0;
}

static int ti2419x_get_ship_mode(struct ti2419x_chip *chip)
{
	int rc;
	u8 reg;

	rc = ti2419x_read(chip, MISC_OPERA_CTRL_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't turn OFF BATTFET rc=%d\n", rc);
		return rc;
	}
	reg = (reg & 0x20) >> BATFET_DISABLE_SHIFT;

	return !reg;
}

static int ti2419x_set_appropriate_usb_current(struct ti2419x_chip *chip)
{
	int rc = 0, i, therm_ma, current_ma;
	int path_current = chip->usb_psy_ma;

	/*
	 * If battery is absent do not modify the current at all, these
	 * would be some appropriate values set by the bootloader or default
	 * configuration and since it is the only source of power we should
	 * not change it
	 */
	if (!chip->batt_present) {
		pr_debug("ignoring current request since battery is absent\n");
		return 0;
	}

	if (chip->therm_lvl_sel > 0
			&& chip->therm_lvl_sel < (chip->thermal_levels - 1))
		/*
		 * consider thermal limit only when it is active and not at
		 * the highest level
		 */
		therm_ma = chip->thermal_mitigation[chip->therm_lvl_sel];
	else
		therm_ma = path_current;

	current_ma = min(therm_ma, path_current);

	if (chip->max_iusb > 0)
		current_ma = min(current_ma, chip->max_iusb);

	if (current_ma <= 2) {
		/* when usb report 2MA, must set to the smallest current */
		current_ma = 100;
	}

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		for (i = ARRAY_SIZE(input_current_limit) - 1; i >= 0; i--) {
			if (input_current_limit[i] <= current_ma)
				break;
		}

		if (i < 0) {
			pr_debug("Couldn't find ICL mA rc=%d\n", rc);
			i = 0;
		}

		/* set input current limit */
		rc = ti2419x_masked_write(chip, INPUT_SOURCE_CTRL_REG,
				IINLIMIT_MASK, i);

		if (rc)
			pr_err("Couldn't set ICL mA rc=%d\n", rc);

		pr_info("input current set to = %d,set ichg_current to %d\n",
				input_current_limit[i], ichg_current_limit[i]);
		break;
	case TI_25601_CHARGER_IC:
		for (i = ARRAY_SIZE(ti25601_input_current_limit) - 1; i >= 0; i--) {
			if (ti25601_input_current_limit[i] <= current_ma)
				break;
		}

		if (i < 0) {
			pr_debug("Couldn't find ICL mA rc=%d\n", rc);
			i = 0;
		}

		/* set input current limit */
		rc = ti2419x_masked_write(chip, INPUT_SOURCE_CTRL_REG,
				TI25601_IINLIMIT_MASK, i);

		if (rc)
			pr_err("Couldn't set ICL mA rc=%d\n", rc);

		pr_info("input current set to = %d,set ichg_current to %d\n",
				ti25601_input_current_limit[i], ti25601_ichg_current_limit[i]);
		break;
	}

	return rc;
}

static int ti2419x_system_temp_level_set(struct ti2419x_chip *chip,
							int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;

	if (!chip->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->current_change_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		/* Disable charging if highest value selected */
		rc = ti2419x_charging_disable(chip, THERMAL, true);
		if (rc < 0) {
			pr_err("Couldn't disable charging rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	ti2419x_set_appropriate_usb_current(chip);

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Hence enable charging.
		 */
		rc = ti2419x_charging_disable(chip, THERMAL, false);
		if (rc < 0) {
			pr_err("Couldn't enable charging rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->current_change_lock);
	return rc;
}

static int ti2419x_battery_set_property(struct power_supply *psy,
					   enum power_supply_property prop,
					   const union power_supply_propval *val)
{
	struct ti2419x_chip *chip = container_of(psy,
				struct ti2419x_chip, batt_psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		pr_info("val->intval:%d\n", val->intval);
		mutex_lock(&chip->chg_enable_lock);
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_FULL:
			rc = ti2419x_charging_disable(chip, SOC, true);
			if (rc)
				pr_err("Failed to disable charging rc=%d\n", rc);
			else if (ti2419x_is_chg_plugged_in(chip))
				chip->batt_full = true;
			else
				pr_err("usb not present,do not set the battery full flags\n");
			power_supply_changed(&chip->batt_psy);
			break;
		case POWER_SUPPLY_STATUS_CHARGING:
			pr_debug("resuming charging by bms\n");
			chip->batt_full = false;
			rc = ti2419x_charging_disable(chip, SOC, false);
			if (rc)
				pr_err("Failed to enable charging rc=%d\n", rc);
			power_supply_changed(&chip->batt_psy);
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			pr_debug("status = DISCHARGING chg_done = %d\n",
					chip->batt_full);
			break;
		default:
			break;
		}
		mutex_unlock(&chip->chg_enable_lock);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ti2419x_charging_disable(chip, USER, !val->intval);
		if (val->intval == 0) {
			wake_unlock(&chip->charger_valid_lock);
			/* ti2419x_masked_write(chip, INPUT_SOURCE_CTRL_REG,
				EN_HIZ_MASK, 1 << EN_HIZ_SHIFT); */
		} else {
			 /* ti2419x_masked_write(chip, INPUT_SOURCE_CTRL_REG,
				EN_HIZ_MASK, 0 << EN_HIZ_SHIFT); */
			wake_lock(&chip->charger_valid_lock);
		}
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		pr_info("fake_soc set to %d\n", chip->fake_battery_soc);
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		ti2419x_system_temp_level_set(chip, val->intval);
		break;
	case  POWER_SUPPLY_PROP_PRESENT:
		/* need get from bms */
		break;
	case POWER_SUPPLY_PROP_SHIPMODE:
		ti2419x_enable_ship_mode(chip, val->intval);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int ti2419x_battery_is_writeable(struct power_supply *psy,
					   enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_SHIPMODE:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int ti2419x_battery_get_property(struct power_supply *psy,
					   enum power_supply_property prop,
					   union power_supply_propval *val)
{
	struct ti2419x_chip *chip = container_of(psy,
				struct ti2419x_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = ti2419x_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = ti2419x_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = ti2419x_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = ti2419x_is_charging_enabled(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = ti2419x_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = ti2419x_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = ti2419x_get_prop_chg_full_design(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = ti2419x_get_prop_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = ti2419x_get_prop_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = ti2419x_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->usb_present;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = ti2419x_get_prop_batt_bms_fcc(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->cfg_max_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = CFG_MIN_VOLTAGE_UV;
		break;
	case POWER_SUPPLY_PROP_SHIPMODE:
		val->intval = ti2419x_get_ship_mode(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = ti2419x_get_prop_charge_counter(chip);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
void set_charger_state(struct ti2419x_chip *chip, bool disable)
{
	pr_info("disable=%d chip->batt_full=%d\n", disable, chip->batt_full);
	if (disable) {
		chip->batt_full = true;
		ti2419x_charging_disable(chip, CURRENT, true);
		power_supply_changed(&chip->batt_psy);
		pr_info("charger_disable1=%d chip->batt_full1=%d\n", disable, chip->batt_full);
	} else {
		ti2419x_charging_disable(chip, CURRENT, false);
		power_supply_changed(&chip->batt_psy);
		pr_info("charger_disable2=%d chip->batt_full2=%d\n", disable, chip->batt_full);
	}
}

static void ti2419x_external_power_changed(struct power_supply *psy)
{
	struct ti2419x_chip *chip = container_of(psy,
				struct ti2419x_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0;

	pr_info("external power changed\n");
	if (!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("ti_bms");

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;

	pr_info("current_limit = %d chip->usb_psy_ma = %d\n", current_limit, chip->usb_psy_ma);

	if (chip->usb_psy_ma != current_limit) {
		mutex_lock(&chip->current_change_lock);
		chip->usb_psy_ma = current_limit;
		rc = ti2419x_set_appropriate_usb_current(chip);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		mutex_unlock(&chip->current_change_lock);
		pr_info("usb_psy_ma: %d\n", chip->usb_psy_ma);
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (rc < 0)
		pr_err("could not read USB ONLINE property, rc=%d\n", rc);

	/* update online property */
	rc = 0;
	if (chip->usb_present && chip->usb_psy_ma != 0) {
		if (prop.intval == 0)
			rc = power_supply_set_online(chip->usb_psy, true);
	} else {
		if (prop.intval == 1)
			rc = power_supply_set_online(chip->usb_psy, false);
	}
	if (rc < 0)
		pr_err("could not set usb online, rc=%d\n", rc);
}

#define CHARGER_POLICY_MS  30000
static int ti2419x_battery_discharging(struct ti2419x_chip *bq)
{
	/* if (bq->battery_charging_policy_ops.battery_status != BATTERY_DISCHARGING) { */
	if (bq->policy_usb_psy_ma == 0)
		bq->policy_usb_psy_ma = bq->usb_psy_ma;
	bq->usb_psy_ma = 2;
	pr_info("policy_usb_psy_ma=%d usb_psy_ma=%d\n", bq->policy_usb_psy_ma, bq->usb_psy_ma);
	ti2419x_set_appropriate_usb_current(bq);
	ti2419x_charging_disable(bq, POLICY, true);
	bq->battery_charging_policy_ops.battery_status = BATTERY_DISCHARGING;

	return 0;
}

static int ti2419x_battery_not_charging(struct ti2419x_chip *bq)
{
	/* if (bq->battery_charging_policy_ops.battery_status != BATTERY_NOT_CHARGING) { */
	if (bq->policy_usb_psy_ma != 0) {
		bq->usb_psy_ma = bq->policy_usb_psy_ma;
		pr_info("policy_usb_psy_ma=%d usb_psy_ma=%d\n", bq->policy_usb_psy_ma, bq->usb_psy_ma);
	}
	ti2419x_set_appropriate_usb_current(bq);
	if (bq->battery_charging_policy_ops.battery_status == BATTERY_DISCHARGING)
		ti2419x_charging_disable(bq, POLICY, true);
	bq->battery_charging_policy_ops.battery_status = BATTERY_NOT_CHARGING;

	return 0;
}

static int ti2419x_battery_charging(struct ti2419x_chip *bq, bool force)
{
	pr_info("force=%d\n", force);

	if (force) {
		if ((!bq->bat_is_hot) && (!bq->bat_is_cold)) {
			if (bq->policy_usb_psy_ma != 0) {
				bq->usb_psy_ma = bq->policy_usb_psy_ma;
				pr_info("policy_usb_psy_ma=%d, usb_psy_ma=%d\n", bq->policy_usb_psy_ma, bq->usb_psy_ma);
			}
			ti2419x_set_appropriate_usb_current(bq);
			ti2419x_charging_disable(bq, POLICY, false);
		}
		bq->battery_charging_policy_ops.battery_status = BATTERY_CHARGING;
		bq->battery_charging_policy_ops.charging_policy_status = DEFAULT_CHARGING_POLICY;

		return 0;
	}

	/* if (bq->battery_charging_policy_ops.battery_status !=  BATTERY_CHARGING) { */
	if ((!bq->bat_is_hot) && (!bq->bat_is_cold)) {
		if (bq->policy_usb_psy_ma != 0) {
			pr_info("policy_usb_psy_ma=%d, usb_psy_ma=%d\n", bq->policy_usb_psy_ma, bq->usb_psy_ma);
			bq->usb_psy_ma = bq->policy_usb_psy_ma;
		}
		ti2419x_set_appropriate_usb_current(bq);
		ti2419x_charging_disable(bq, POLICY, false);
	}
	bq->battery_charging_policy_ops.battery_status = BATTERY_CHARGING;

	return 0;
}

static int ti2419x_battery_clean_charging_expired(struct ti2419x_chip *bq)
{
	bq->battery_charging_policy_ops.charging_policy_status &= ~EXPIRED_CHARGING_POLICY;

	return 0;
}

static int ti2419x_battery_charging_policy_check(struct ti2419x_chip *bq)
{
	int soc = ti2419x_get_prop_batt_capacity(bq);

	pr_info("charging_policy_status = 0x%x(1:normal 2:demo 4:expired) soc = %d\n",
		bq->battery_charging_policy_ops.charging_policy_status, soc);
	pr_info("battery_status = %d(0:charging 1:discharging 2:not_charging)\n",
		bq->battery_charging_policy_ops.battery_status);

	if ((bq->battery_charging_policy_ops.charging_policy_status & DEMO_CHARGING_POLICY)
		|| (bq->battery_charging_policy_ops.charging_policy_status & EXPIRED_CHARGING_POLICY)) {
		if (soc >= MAX_BATTERY_PROTECTED_PERCENT) {
			ti2419x_battery_discharging(bq);
		} else if (soc > MIN_BATTERY_PROTECTED_PERCENT && soc < MAX_BATTERY_PROTECTED_PERCENT) {
			ti2419x_battery_not_charging(bq);
		} else if (soc <= MIN_BATTERY_PROTECTED_PERCENT) {
			ti2419x_battery_charging(bq, false);
		}
	} else {
		ti2419x_battery_charging(bq, true);
	}

	return 0;
}

static void ti2419x_charging_policy_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *bq = container_of(dwork, struct ti2419x_chip, charging_policy_work);

	ti2419x_battery_charging_policy_check(bq);
	power_supply_changed(&bq->batt_psy);
	schedule_delayed_work(&bq->charging_policy_work,
		round_jiffies_relative(msecs_to_jiffies(CHARGER_POLICY_MS)));
}

static enum alarmtimer_restart ti2419x_charging_expired_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	#if (EXPIRED_CHARGING_POLICY_ENABLE == 1)
	struct ti2419x_chip *bq = container_of(alarm, struct ti2419x_chip,
					charging_expired_alarm);

	pr_info("\n");

	bq->battery_charging_policy_ops.charging_policy_status &= ~NORMAL_CHARGING_POLICY;
	bq->battery_charging_policy_ops.charging_policy_status |= EXPIRED_CHARGING_POLICY;
	cancel_delayed_work(&bq->charging_policy_work);
	schedule_delayed_work(&bq->charging_policy_work,
		round_jiffies_relative(msecs_to_jiffies(1000)));
	#endif

	return ALARMTIMER_NORESTART;
}

static void ti2419x_usb_type_detection_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *chip = container_of(dwork, struct ti2419x_chip, usb_type_detection_work);
	int ret;

	alarm_cancel(&chip->charging_expired_alarm);
	chip->policy_usb_psy_ma = 0;
	if (!chip->usb_present) {
		if (!(chip->battery_charging_policy_ops.charging_policy_status & DEMO_CHARGING_POLICY)) {
			pr_info("policy resume charging\n");
			ti2419x_battery_charging(chip, true);
			ti2419x_battery_clean_charging_expired(chip);
			cancel_delayed_work_sync(&chip->charging_policy_work);
		}
		pr_info("usb removed\n");
	} else {
		ret = alarm_start_relative(&chip->charging_expired_alarm,
			ns_to_ktime(CHARGING_EXPIRATION_TIME_NS));
		if (ret)
			pr_err("Failed to start alarm: %d\n", ret);
		pr_info("usb inserted\n");
	}
}

static int bc_policy_demo_sts_set(struct charging_policy_ops *charging_policy, bool enable)
{
	struct ti2419x_chip *bq = container_of(charging_policy, struct ti2419x_chip,
					battery_charging_policy_ops);

	pr_info("charging_policy_status = 0x%x, enable = %d\n",
		bq->battery_charging_policy_ops.charging_policy_status, enable);

	if (enable) {
		bq->battery_charging_policy_ops.charging_policy_status &= ~NORMAL_CHARGING_POLICY;
		bq->battery_charging_policy_ops.charging_policy_status |= DEMO_CHARGING_POLICY;
	} else {
		bq->battery_charging_policy_ops.charging_policy_status &= ~DEMO_CHARGING_POLICY;
		if ((bq->battery_charging_policy_ops.charging_policy_status & EXPIRED_CHARGING_POLICY)
			!= EXPIRED_CHARGING_POLICY)
			bq->battery_charging_policy_ops.charging_policy_status |= NORMAL_CHARGING_POLICY;
	}

	pr_info("charging_policy_status = 0x%x\n", bq->battery_charging_policy_ops.charging_policy_status);
	cancel_delayed_work_sync(&bq->charging_policy_work);
	schedule_delayed_work(&bq->charging_policy_work,
		round_jiffies_relative(msecs_to_jiffies(1000)));

	return 0;
}

static int bc_policy_demo_sts_get(struct charging_policy_ops *charging_policy)
{
	struct ti2419x_chip *bq = container_of(charging_policy, struct ti2419x_chip,
					battery_charging_policy_ops);

	pr_info("charging_policy_status = 0x%x\n", bq->battery_charging_policy_ops.charging_policy_status);

	if ((bq->battery_charging_policy_ops.charging_policy_status & DEMO_CHARGING_POLICY) == DEMO_CHARGING_POLICY)
		return 1;
	else
		return 0;
}

static int bc_policy_expired_sts_get(struct charging_policy_ops *charging_policy)
{
	struct ti2419x_chip *bq = container_of(charging_policy, struct ti2419x_chip,
					battery_charging_policy_ops);

	pr_info("charging_policy_status = 0x%x\n", bq->battery_charging_policy_ops.charging_policy_status);

	if ((bq->battery_charging_policy_ops.charging_policy_status & EXPIRED_CHARGING_POLICY)
		== EXPIRED_CHARGING_POLICY)
		return 1;
	else
		return 0;
}

static int bc_policy_expired_sec_set(struct charging_policy_ops *charging_policy, int sec)
{
	pr_info("\n");

	return 0;
}

static int bc_policy_expired_sec_get(struct charging_policy_ops *charging_policy)
{
	pr_info("\n");

	return 0;
}

static int charging_policy_init(struct ti2419x_chip *bq)
{
	bq->battery_charging_policy_ops.battery_status = BATTERY_CHARGING;
	bq->battery_charging_policy_ops.charging_policy_status = DEFAULT_CHARGING_POLICY;
	bq->battery_charging_policy_ops.charging_policy_demo_sts_set = bc_policy_demo_sts_set;
	bq->battery_charging_policy_ops.charging_policy_demo_sts_get = bc_policy_demo_sts_get;
	bq->battery_charging_policy_ops.charging_policy_expired_sts_get = bc_policy_expired_sts_get;
	bq->battery_charging_policy_ops.charging_policy_expired_sec_set = bc_policy_expired_sec_set;
	bq->battery_charging_policy_ops.charging_policy_expired_sec_get = bc_policy_expired_sec_get;

	return 0;
}


static int sys_ov_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int therm_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static void smb23x_charge_current_limit(struct ti2419x_chip *chip, int temp);
static int power_good_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	bool usb_present = rt_stat;

	pr_info("chip->usb_present = %d usb_present = %d\n",
				chip->usb_present, usb_present);

	/* in poweroff charging,once the charger is absent,run poweroff work*/
	if (socinfo_get_charger_flag() && !usb_present) {
		pr_info("usb removed!!!poweroff\n");
		schedule_work(&chip->poweroff_work);
	}

	if (chip->usb_present ^ usb_present)
		wake_lock_timeout(&chip->charger_wake_lock, 5 * HZ);

	if (chip->usb_present && !usb_present) {
		/* USB removed */
		chip->usb_present = usb_present;
		#ifdef ZTE_CHARGER_TYPE_OEM
		charge_type_oem = CHARGER_TYPE_DEFAULT;
		g_update_period_ms = HEARTBEAT_MS;
		g_low_soc_update_period_ms = LOW_SOC_HEARTBEAT_MS;
		g_charger_detect_counter = 0;
		#endif
		power_supply_set_present(chip->usb_psy, usb_present);
		#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
		syna_ts_notifier_call_chain(0);
		#endif
		wake_unlock(&chip->charger_valid_lock);
		chip->batt_full = false;
		chip->batt_warm_full = false;
		chip->batt_real_full = false;
		ti2419x_charging_disable(chip, CURRENT, false);
		ti2419x_charging_disable(chip, SOC, false);
		ti2419x_charging_disable(chip, TEMP, false);
	}

	if (!chip->usb_present && usb_present) {
		/* USB inserted */
			wake_lock(&chip->charger_valid_lock);
			#ifdef ZTE_CHARGER_TYPE_OEM
			charge_type_oem = CHARGER_TYPE_DEFAULT;
			g_update_period_ms = HEARTBEAT_CHARGER_TYPE_OEM_MS;
			g_low_soc_update_period_ms = HEARTBEAT_CHARGER_TYPE_OEM_MS;
			g_charger_detect_counter = 0;
			#endif
			chip->usb_present = usb_present;
			power_supply_set_present(chip->usb_psy, usb_present);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
			syna_ts_notifier_call_chain(1);
#endif
			cancel_delayed_work(&chip->charger_eoc_work);	 /* stop charger_eoc_work. */
			schedule_delayed_work(&chip->charger_eoc_work, round_jiffies_relative(msecs_to_jiffies(2000)));

	}
	cancel_delayed_work(&chip->usb_type_detection_work);
	schedule_delayed_work(&chip->usb_type_detection_work, round_jiffies_relative(msecs_to_jiffies(50)));
	return 0;
}

static int dpm_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__, rt_stat);
	return 0;
}

static int chg_stat_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	int rc;
	u8 reg = 0;

	rc = ti2419x_read(chip, SYS_STAT_REG, &reg);

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		reg &= CHG_STAT_MASK;
		reg >>= CHG_STAT_SHIFT;
		break;
	case TI_25601_CHARGER_IC:
		reg &= TI25601_CHG_STAT_MASK;
		reg >>= TI25601_CHG_STAT_SHIFT;
		break;
	}

	pr_info("%s, charge status = 0x%02x\n", __func__, reg);

	return 0;
}

static int vbus_stat_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__, rt_stat);
	return 0;
}

static int hot_cold_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, not support stop charge automatically when hot or cold.\n", __func__);
	return 0;
}

static int bat_fault_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	/* over charge */
	pr_info("%s, rt_stat = 0x%02x\n", __func__, rt_stat);
	return 0;
}

static int chg_fault_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__, rt_stat);
	return 0;
}

static int boost_fault_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__, rt_stat);
	return 0;
}

static int wdog_fault_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__, rt_stat);
	return 0;
}


struct ti2419x_irq_info {
	const char		*name;
	int (*ti2419x_irq)(struct ti2419x_chip *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct ti2419x_irq_info	irq_info[8];
};

static struct irq_handler_info handlers[] = {
	{SYS_STAT_REG, 0, 0,
		{
			{
				.name		= "sys_voltage_status",
				.ti2419x_irq	= sys_ov_handler,
			},
			{
				.name		= "THERM_status",
				.ti2419x_irq	= therm_handler,
			},
			{
				.name		= "PG_status",
				.ti2419x_irq	= power_good_handler,
			},
			{
				.name		= "DPM_status",
				.ti2419x_irq	= dpm_handler,
			},
			{
				.name		= "chg_status0",
				.ti2419x_irq	= chg_stat_handler,
			},
			{
				.name		= "chg_status1",
				.ti2419x_irq	= chg_stat_handler,
			},
			{
				.name		= "VBUS_status0",
				.ti2419x_irq	= vbus_stat_handler,
			},
			{
				.name		= "VBUS_status1",
				.ti2419x_irq	= vbus_stat_handler,
			},
		},
	},
	{FAULT_REG, 0, 0,
		{
			{
				.name		= "NTC_fault0",
				.ti2419x_irq	= hot_cold_handler,
			},
			{
				.name		= "NTC_fault1",
				.ti2419x_irq	= hot_cold_handler,
			},
			{
				.name		= "NTC_fault2",
				.ti2419x_irq	= hot_cold_handler,
			},
			{
				.name		= "BAT_fault",
				.ti2419x_irq	= bat_fault_handler,
			},
			{
				.name		= "CHG_fault0",
				.ti2419x_irq	= chg_fault_handler,
			},
			{
				.name		= "CHG_fault1",
				.ti2419x_irq	= chg_fault_handler,
			},
			{
				.name		= "BOOST_fault",
				.ti2419x_irq	= boost_fault_handler,
			},
			{
				.name		= "WDOG_fault",
				.ti2419x_irq	= wdog_fault_handler,
			},
		},
	},
};

#define IRQ_STATUS_MASK		0x01
static irqreturn_t ti2419x_stat_handler(int irq, void *dev_id)
{
	struct ti2419x_chip *chip = dev_id;
	int i, j;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	/* pr_info("enter..\n"); */
	mutex_lock(&chip->irq_complete);
	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = ti2419x_read(chip, handlers[i].stat_reg,
					&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}
		pr_info("[%d]reg=0x%x val=0x%x prev_val=0x%x\n",
				i, handlers[i].stat_reg, handlers[i].val, handlers[i].prev_val);

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << j);
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << j);
			changed = prev_rt_stat ^ rt_stat;

			if (changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if (changed	&& handlers[i].irq_info[j].ti2419x_irq != NULL) {
				handler_count++;
				pr_info("call %pf, handler_count=%d\n",
					handlers[i].irq_info[j].ti2419x_irq, handler_count);
				rc = handlers[i].irq_info[j].ti2419x_irq(chip,
								rt_stat);
				if (rc < 0)
					dev_err(chip->dev,
						"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_debug("handler count = %d\n", handler_count);
	if (handler_count) {
		cancel_delayed_work(&chip->update_heartbeat_work);
		schedule_delayed_work(&chip->update_heartbeat_work,
			round_jiffies_relative(msecs_to_jiffies(500)));

		cancel_delayed_work(&chip->charger_eoc_work);
		schedule_delayed_work(&chip->charger_eoc_work, 0);
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 8; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct ti2419x_chip *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define LAST_CNFG_REG	0xA

static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct ti2419x_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = ti2419x_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct ti2419x_chip *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* not support now, need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
/* okay, it is time to have a change. */
static int ti2419x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct ti2419x_chip *chip = rdev_get_drvdata(rdev);

	pr_info("%s\n", __func__);
	rc = ti2419x_masked_write(chip, PON_CFG_REG, CHG_CFG_MASK,
			(2 << CHG_CFG_SHIFT));
	if (rc < 0)
		pr_err("Couldn't enable  OTG boost power, rc=%d\n", rc);

	return rc;
}

static int ti2419x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct ti2419x_chip *chip = rdev_get_drvdata(rdev);

	pr_info("%s\n", __func__);
	rc = ti2419x_masked_write(chip, PON_CFG_REG, CHG_CFG_MASK,
			(1 << CHG_CFG_SHIFT));
	if (rc < 0)
		pr_err("Couldn't disable OTG boost power, rc=%d\n", rc);

	return rc;
}

static int ti2419x_otg_regulator_is_enabled(struct regulator_dev *rdev)
{
	u8 reg = 0;
	int rc = 0;
	struct ti2419x_chip *chip = rdev_get_drvdata(rdev);

	pr_info("%s\n", __func__);
	rc = ti2419x_read(chip, PON_CFG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read OTG boost power state, rc=%d\n", rc);
		return rc;
	}

	return	((reg & CHG_CFG_MASK) & (2 << CHG_CFG_SHIFT)) ? 1 : 0;
}

struct regulator_ops ti2419x_otg_reg_ops = {
	.enable		= ti2419x_otg_regulator_enable,
	.disable	= ti2419x_otg_regulator_disable,
	.is_enabled	= ti2419x_otg_regulator_is_enabled,
};

static int ti2419x_otg_is_enabled(struct ti2419x_chip *chip)
{
	u8 reg = 0;
	int rc = 0;

	rc = ti2419x_read(chip, PON_CFG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read OTG boost power state, rc=%d\n", rc);
		return rc;
	}

	return	((reg & CHG_CFG_MASK) & (2 << CHG_CFG_SHIFT)) ? 1 : 0;
}

static int ti2419x_regulator_init(struct ti2419x_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &ti2419x_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}
/* end, not support now, change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
/* changed */

static int determine_initial_status(struct ti2419x_chip *chip)
{
	/* set usb_present to false when initialize */
	chip->usb_present = false;
	chip->bat_is_cold = false;
	chip->bat_is_hot = false;
	chip->bat_is_cool = false;
	chip->bat_is_warm = false;
	chip->high_temp_threshold = chip->warm_bat_decidegc;
	chip->low_temp_threshold = chip->cool_bat_decidegc;
	chip->soc = -1;
	chip->tm_state = TI_TEMP_NORMAL_STATE;

	return 0;
}


#define MIN_ITERM_MA		60
#define MAX_ITERM_MA		960
static void set_iterm(struct ti2419x_chip *chip, int ma)
{
	u8 temp;
	int rc;

	if ((ma < MIN_ITERM_MA) || (ma > MAX_ITERM_MA)) {
		dev_err(chip->dev, "bad terminate current mv =%d asked to set\n",
					ma);
		return;
	}

	temp = (ma - MIN_ITERM_MA) / MIN_ITERM_MA;

	rc = ti2419x_masked_write(chip, IPRECHG_ITERM_CTRL_REG,
			ITERM_MASK, temp << ITERM_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set ITERM rc=%d\n", rc);
		return;
	}

	rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
			EN_TERM_MASK | TERM_STAT_MASK, 0x2 << TERM_STAT_SHIFT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable EN_TERM rc=%d\n", rc);
}

#define TI2419X_WDOG_DISABLE	0
#define TI2419X_WDOG_40S		1
#define TI2419X_WDOG_80S		2
#define TI2419X_WDOG_160S		3
int set_charge_wdog(struct ti2419x_chip *chip, int time)
{
	int rc;

	if (time < TI2419X_WDOG_DISABLE || time > TI2419X_WDOG_160S) {
		dev_err(chip->dev, "invalid charge watch dog setting\n");
		return -EINVAL;
	}

	rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
			I2C_WDOG_TIMER_MASK, time << I2C_WDOG_TIMER_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set watch dog timer rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#define MIN_IBAT_MA		512
#define MAX_IBAT_MA		3008
#define MAX_IBAT_STEP	64

#define TI25601_MIN_IBAT_MA		0
#define TI25601_MAX_IBAT_MA		3000
#define TI25601_MAX_IBAT_STEP	60
int set_ibat(struct ti2419x_chip *chip, int ma)
{
	int rc;
	u8 reg;

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		if (ma < MIN_IBAT_MA) {
			ma = MIN_IBAT_MA;
			dev_err(chip->dev, "bad battery charge current ma =%d asked to set\n",
					ma);
		} else if (ma > MAX_IBAT_MA) {
			ma = MAX_IBAT_MA;
			dev_err(chip->dev, "bad battery charge current ma =%d asked to set\n",
					ma);
		}

		reg = (ma - MIN_IBAT_MA)/MAX_IBAT_STEP;

		rc = ti2419x_masked_write(chip, CHG_I_CTRL_REG,
				IFAST_CHG_MASK, reg << IFAST_CHG_SHIFT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set battery charge current rc=%d\n", rc);
			return rc;
		}
		pr_info("ibat current set to = %d\n", (reg*MAX_IBAT_STEP+512));
		break;
	case TI_25601_CHARGER_IC:
		if (ma < TI25601_MIN_IBAT_MA) {
			ma = TI25601_MIN_IBAT_MA;
			dev_err(chip->dev, "bad battery charge current ma =%d asked to set\n",
					ma);
		} else if (ma > TI25601_MAX_IBAT_MA) {
			ma = TI25601_MAX_IBAT_MA;
			dev_err(chip->dev, "bad battery charge current ma =%d asked to set\n",
					ma);
		}

		reg = (ma - TI25601_MIN_IBAT_MA)/TI25601_MAX_IBAT_STEP;

		rc = ti2419x_masked_write(chip, CHG_I_CTRL_REG,
				TI25601_IFAST_CHG_MASK, reg << TI25601_IFAST_CHG_SHIFT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set battery charge current rc=%d\n", rc);
			return rc;
		}
		pr_info("ibat current set to = %d\n", (reg*TI25601_MAX_IBAT_STEP));
		break;
	}

	return 0;
}

void ti2419x_set_appropriate_ibat(struct ti2419x_chip *chip, int temp)
{
	unsigned int chg_current = chip->max_ibat;

	if (chip->bat_is_cool) {
		if (temp < chip->cooler_bat_decidegc)
			chg_current = min(chg_current, chip->cooler_bat_chg_ma);
		else
			chg_current = min(chg_current, chip->cool_bat_chg_ma);
	}
	if (chip->bat_is_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_ma);

	if (chip->current_ibat != chg_current) {
		pr_debug("setting %d mA\n", chg_current);
		set_ibat(chip, chg_current);
		chip->current_ibat = chg_current;
	}
}

#define MIN_INPUT_VOLTAGE_MV	3880
#define MAX_INPUT_VOLTAGE_MV	5080
#define MAX_INPUT_VOLTAGE_STEP	80

#define TI25601_MIN_INPUT_VOLTAGE_MV	3900
#define TI25601_MAX_INPUT_VOLTAGE_MV	5400
#define TI25601_MAX_INPUT_VOLTAGE_STEP	100
int set_input_voltage(struct ti2419x_chip *chip, int mv)
{
	int rc;
	u8 reg;

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		if ((mv < MIN_INPUT_VOLTAGE_MV) || (mv > MAX_INPUT_VOLTAGE_MV)) {
			dev_err(chip->dev, "bad input voltage mv =%d asked to set\n",
					mv);
			return -EINVAL;
		}

		reg = (mv - MIN_INPUT_VOLTAGE_MV)/MAX_INPUT_VOLTAGE_STEP;

		rc = ti2419x_masked_write(chip, INPUT_SOURCE_CTRL_REG,
				VINLIMIT_MASK, reg << VINLIMIT_SHIFT);
		break;
	case TI_25601_CHARGER_IC:
		if ((mv < TI25601_MIN_INPUT_VOLTAGE_MV) || (mv > TI25601_MAX_INPUT_VOLTAGE_MV)) {
			dev_err(chip->dev, "bad input voltage mv =%d asked to set\n",
					mv);
			return -EINVAL;
		}

		reg = (mv - TI25601_MIN_INPUT_VOLTAGE_MV)/TI25601_MAX_INPUT_VOLTAGE_STEP;

		rc = ti2419x_masked_write(chip, IR_THERM_REG,
				TI25601_VINLIMIT_MASK, reg << TI25601_VINLIMIT_SHIFT);
		break;
	}
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set input voltage rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int get_charge_IC_type(struct ti2419x_chip *chip)
{
	int rc;
	u8	reg;

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		rc = ti2419x_read(chip, VENDOR_REG, &reg);
		if (rc) {
			pr_err("Couldn't read VENDOR_REG rc=%d\n", rc);
			return -EINVAL;
		}

		reg &= TI2419X_DEV_REG_MASK;
		return (reg) ? CHARGER_IC_2419X : CHARGER_IC_2429X;
	case TI_25601_CHARGER_IC:
		rc = ti2419x_read(chip, TI25601_VENDOR_REG, &reg);
		if (rc) {
			pr_err("Couldn't read TI25601_VENDOR_REG rc=%d\n", rc);
			return -EINVAL;
		}

		reg &= TI25601_PN_MASK;
		reg >>= TI25601_VENDOR_SHIFT;
		if (reg == CHARGER_IC_25601)
			return 1;
		else
			return 0;
	}
	return -EINVAL;
}

static int ti2419x_hw_init(struct ti2419x_chip *chip)
{
	int rc;
	int i;
	u8 reg;

	/* wdog timer, 80s */
	set_charge_wdog(chip, TI2419X_WDOG_160S);

	/* charge IC type */
	chip->chargeIC_type = get_charge_IC_type(chip);

	/* EN_HIZ */
	rc = ti2419x_masked_write(chip, INPUT_SOURCE_CTRL_REG,
			EN_HIZ_MASK, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set INPUT_SOURCE_CTRL_REG rc=%d\n", rc);
		return rc;
	}

	if (chip->max_ibat > 0)
		set_ibat(chip, chip->max_ibat);

	/* sare time, not 2X */
	rc = ti2419x_masked_write(chip, MISC_OPERA_CTRL_REG,
					TMR2X_EN_MASK, 0);
	if (rc)
		pr_err("Couldn't set TMR2X_EN rc=%d\n", rc);

	/* input voltage */
	set_input_voltage(chip, chip->max_input_voltage);

	/* charge config */
	rc = ti2419x_masked_write(chip, PON_CFG_REG,
					CHG_CFG_MASK, 0x1 << CHG_CFG_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set PON_CFG_REG rc=%d\n",
				rc);
		return rc;
	}


	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = ti2419x_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	/* set iterm */
	if (chip->iterm_ma == -EINVAL) {
		pr_err("error: iterm_ma invalid, need set in dts");
	} else	if (chip->hw_iterm_disabled) {
		pr_info("using sw iterm function");
		ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
				EN_TERM_MASK, 0);
		if (rc) {
			dev_err(chip->dev, "Couldn't disable hw iterm rc = %d\n",
								rc);
			return rc;
		}
	} else {
		pr_info("using hw iterm function,iterm_ma=%d\n", chip->iterm_ma);
		set_iterm(chip, chip->iterm_ma);
	}

	/* set the safety time voltage */
	if (chip->safety_time != -EINVAL) {
		if (chip->safety_time == 0) {
			/* safety timer disabled */
			rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
						SAFE_TIMER_EN_MASK, 0);
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't disable safety timer rc = %d\n",
								rc);
				return rc;
			}
		} else {
			switch (ic_type) {
			case TI_24X9X_CHARGER_IC:
				for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
					if (chip->safety_time <= chg_time[i]) {
						reg = i << FAST_CHG_TIMER_SHIFT;
						break;
					}
				}
				rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
						FAST_CHG_TIMER_MASK, reg);
				break;
			case TI_25601_CHARGER_IC:
				for (i = 0; i < ARRAY_SIZE(ti25601_chg_time); i++) {
					if (chip->safety_time <= ti25601_chg_time[i]) {
						reg = i << TI25601_FAST_CHG_TIMER_SHIFT;
						break;
					}
				}
				rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
						TI25601_FAST_CHG_TIMER_MASK, reg);
				break;
			}
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set safety timer rc = %d\n",
									rc);
				return rc;
			}

			rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
						SAFE_TIMER_EN_MASK, 1 << SAFE_TIMER_EN_SHIFT);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't enable safety timer rc = %d\n",
									rc);
				return rc;
			}
		}
	}

	/* configure resume threshold, auto recharge*/
	if (chip->resume_delta_mv != -EINVAL) {
		rc = ti2419x_recharge_threshold_set(chip,
						chip->resume_delta_mv);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set rechg thresh rc = %d\n",
								rc);
			return rc;
		}
	}

	if (ic_type == TI_24X9X_CHARGER_IC) {
		/* interrupt enabling - active low */
		if (chip->client->irq) {
			rc = ti2419x_masked_write(chip, MISC_OPERA_CTRL_REG, INT_MASK, INT_MASK);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set irq config rc = %d\n",
						rc);
				return rc;
			}
		}
	}

	rc = ti2419x_charging_disable(chip, USER, !!chip->charging_disabled);
	if (rc)
		dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);

	return rc;
}

#define	ZTE_HOT_TEMP_DEFAULT		500
#define	ZTE_COLD_TEMP_DEFAULT		0
#define	ZTE_WARM_TEMP_DEFAULT		450
#define	ZTE_COOL_TEMP_DEFAULT		100
#define	HYSTERISIS_DECIDEGC		20
#define	MAX_TEMP		800
#define	MIN_TEMP		-300
enum {
	TI_TM_HIGHER_STATE,
	TI_TM_LOWER_STATE,
	TI_TM_NORMAL_STATE,
};

#if defined(LED_GPIO_CONTROL)
extern void zte_misc_red_led_control(bool value);
extern void zte_misc_green_led_control(bool value);
static void ti2419x_led_disable(void)
{
	pr_info("Turn off green&red led in hot/cold mode.");
	zte_misc_red_led_control(false);
	zte_misc_green_led_control(false);
}
#endif
/*
 * smb23x_charge_current_limit
 * set appropriate voltages and currents.
 *
 * (TODO)Note that when the battery is hot or cold, the charger
 * driver will not resume with SoC. Only vbatdet is used to
 * determine resume of charging.
 */

/* NOTE(by ZTE JZN):
*  if vbatt>warm_bat_mv and charging is enabled, battery will discharging and iusb=0 ;
*  if vbatt>warm_bat_mv and charging is disabled, iusb will be the currnt source and ibat=0 .
*  so, there is a bug in the below codes:if vbatt>warm_bat_mv,battery will discharging and iusb=0
*/
static void smb23x_charge_current_limit(struct ti2419x_chip *chip, int temp)
{
	ti2419x_set_appropriate_usb_current(chip);
	ti2419x_set_appropriate_ibat(chip, temp);
	ti2419x_set_appropriate_float_voltage(chip);
	ti2419x_charging_disable(chip, TEMP, false);
	ti2419x_charging_disable(chip, CURRENT, false);
	power_supply_changed(&chip->batt_psy);
}

static void ti2419x_update_temp_state(struct ti2419x_chip *chip)
{
	chip->bat_is_hot = (chip->tm_state == TI_TEMP_HOT_STATE) ? true : false;
	chip->bat_is_warm = (chip->tm_state == TI_TEMP_WARM_STATE) ? true : false;
	chip->bat_is_cool = (chip->tm_state == TI_TEMP_COOL_STATE) ? true : false;
	chip->bat_is_cold = (chip->tm_state == TI_TEMP_COLD_STATE) ? true : false;
}

static void  ti2419x_temp_state_changed(struct ti2419x_chip *chip, int temp)
{
	ti2419x_update_temp_state(chip);

	switch (chip->tm_state) {
	case(TI_TEMP_COLD_STATE):
		chip->low_temp_threshold = MIN_TEMP;
		chip->high_temp_threshold = chip->cold_bat_decidegc + HYSTERISIS_DECIDEGC;
		ti2419x_charging_disable(chip, TEMP, true);
		power_supply_changed(&chip->batt_psy);
#if defined(LED_GPIO_CONTROL)
		ti2419x_led_disable();
#endif
		break;
	case(TI_TEMP_COOL_STATE):
		chip->low_temp_threshold = chip->cold_bat_decidegc;
		chip->high_temp_threshold = chip->cool_bat_decidegc + HYSTERISIS_DECIDEGC;
		smb23x_charge_current_limit(chip, temp);
		break;
	case(TI_TEMP_NORMAL_STATE):
		chip->low_temp_threshold = chip->cool_bat_decidegc;
		chip->high_temp_threshold = chip->warm_bat_decidegc;
		smb23x_charge_current_limit(chip, temp);
		break;
	case(TI_TEMP_WARM_STATE):
		chip->low_temp_threshold = chip->warm_bat_decidegc - HYSTERISIS_DECIDEGC;
		chip->high_temp_threshold = chip->hot_bat_decidegc;
		smb23x_charge_current_limit(chip, temp);
		break;
	case(TI_TEMP_HOT_STATE):
		chip->low_temp_threshold = chip->hot_bat_decidegc - HYSTERISIS_DECIDEGC;
		chip->high_temp_threshold = MAX_TEMP;
		ti2419x_charging_disable(chip, TEMP, true);
		power_supply_changed(&chip->batt_psy);
#if defined(LED_GPIO_CONTROL)
		ti2419x_led_disable();
#endif
		break;
	default:
		break;
	}
}

#define TEMP_DETECT_WORK_DELAY_2S 2000
#define TEMP_DETECT_WORK_DELAY_30S 30000

static void ti_temp_control_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *chip = container_of(dwork, struct ti2419x_chip, temp_control_work);
	int state = TI_TM_NORMAL_STATE;
	int temp = 0;
	int period = TEMP_DETECT_WORK_DELAY_2S;

	if (!chip->resume_completed) {
		pr_info("ti_temp_control_func launched before device-resume, schedule to 2s later\n");
		schedule_delayed_work(&chip->temp_control_work, round_jiffies_relative(msecs_to_jiffies(2000)));
		return;
	}
	if (ti2419x_otg_is_enabled(chip) == 1) {
		pr_info("ti2419x_otg_is_enabled, schedule to 1 min later\n");
		schedule_delayed_work(&chip->temp_control_work, round_jiffies_relative(msecs_to_jiffies(60000)));
		return;
	}

	temp = ti2419x_get_prop_batt_temp(chip);

	if (temp > MAX_TEMP)
		temp = MAX_TEMP;
	if (temp < MIN_TEMP)
		temp = MIN_TEMP;

	if (temp > chip->high_temp_threshold)
		state = TI_TM_HIGHER_STATE;
	else if (temp < chip->low_temp_threshold)
		state = TI_TM_LOWER_STATE;
	else
		state = TI_TM_NORMAL_STATE;

	mutex_lock(&chip->jeita_configure_lock);

	if (state == TI_TM_HIGHER_STATE) {
		switch (chip->tm_state) {
		case(TI_TEMP_COLD_STATE):
			chip->tm_state = TI_TEMP_COOL_STATE;
			break;
		case(TI_TEMP_COOL_STATE):
			chip->tm_state = TI_TEMP_NORMAL_STATE;
			break;
		case(TI_TEMP_NORMAL_STATE):
			chip->tm_state = TI_TEMP_WARM_STATE;
			break;
		case(TI_TEMP_WARM_STATE):
			chip->tm_state = TI_TEMP_HOT_STATE;
			break;
		}
		ti2419x_temp_state_changed(chip, temp);
	} else if (state == TI_TM_LOWER_STATE) {
		switch (chip->tm_state) {
		case(TI_TEMP_COOL_STATE):
			chip->tm_state = TI_TEMP_COLD_STATE;
			break;
		case(TI_TEMP_NORMAL_STATE):
			chip->tm_state = TI_TEMP_COOL_STATE;
			break;
		case(TI_TEMP_WARM_STATE):
			chip->tm_state = TI_TEMP_NORMAL_STATE;
			break;
		case(TI_TEMP_HOT_STATE):
			chip->tm_state = TI_TEMP_WARM_STATE;
			break;
		}
		ti2419x_temp_state_changed(chip, temp);
	} else {
		pr_debug("TI_TM_NORMAL_STATE\n");
		if (chip->bat_is_cool)
			ti2419x_set_appropriate_ibat(chip, temp);
		goto check_again;
	}

check_again:
	mutex_unlock(&chip->jeita_configure_lock);

	if ((state == TI_TM_HIGHER_STATE) || (state == TI_TM_LOWER_STATE)) {
		pr_info("temp=%d threshold=(%d-%d) health=(%d-%d-%d-%d)\n",
				temp, chip->low_temp_threshold, chip->high_temp_threshold,
				chip->bat_is_cold, chip->bat_is_cool,
				chip->bat_is_warm, chip->bat_is_hot);
}

	if (chip->tm_state == TI_TEMP_NORMAL_STATE)
		period = TEMP_DETECT_WORK_DELAY_30S;
	else
		period = TEMP_DETECT_WORK_DELAY_2S;

	schedule_delayed_work(&chip->temp_control_work, round_jiffies_relative(msecs_to_jiffies(period)));
}


static int heartbeat_ms = 0;
static int set_heartbeat_ms(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (the_ti2419x_chip) {
		pr_info("set_heartbeat_ms to %d\n", heartbeat_ms);
		cancel_delayed_work_sync(&the_ti2419x_chip->update_heartbeat_work);
		schedule_delayed_work(&the_ti2419x_chip->update_heartbeat_work,
					  round_jiffies_relative(msecs_to_jiffies
								 (heartbeat_ms)));
		return 0;
	}
	return -EINVAL;
}
module_param_call(heartbeat_ms, set_heartbeat_ms, param_get_uint,
					&heartbeat_ms, 0644);

void check_fullcharged_state(struct ti2419x_chip *chip)
{
	int charger_status = 0;
	int is_chg_in, is_batt_present, capatity, bms_flags, voltage_now;
	static int is_first_time_check = 1;
	bool fc_bit;

	if (chip->ti_bms) {
		is_chg_in = ti2419x_is_chg_plugged_in(chip);
		bms_flags = ti2419x_get_prop_batt_bms_flags(chip);
		fc_bit = (bms_flags&0x0200)?1:0;
		pr_info("chip->batt_full:%d,fc_bit:%d,is_chg_in:%d\n",
			chip->batt_full, fc_bit, is_chg_in);
		if ((chip->batt_full != fc_bit) && is_chg_in)
			set_charger_state(chip, fc_bit);
	} else {
		is_chg_in		   = ti2419x_is_chg_plugged_in(chip);
		is_batt_present = ti2419x_get_prop_batt_present(chip);
		capatity			 = ti2419x_get_prop_batt_capacity_real(chip);
		charger_status	= ti2419x_get_charging_status(chip);
		voltage_now = ti2419x_get_prop_voltage_now(chip) / 1000;

		if (charger_status == CHARGE_TERM_DONE) {
			pr_info("charger_status %d DONE,old batt_full=%d\n", charger_status, chip->batt_full);
			if (is_chg_in && is_batt_present && (capatity <= 99) && is_first_time_check) {
				__ti2419x_charging_disable(chip, true);
				__ti2419x_charging_disable(chip, false);/* re-charging if soc not 100 */
				pr_info("[CHG] recharge when charger reported DONE and soc =%d\n",
					capatity);
				return;
			}
			chip->batt_full = true;
			power_supply_changed(&chip->batt_psy);
			pr_info("[CHG]set batt_full=1 and soc =%d\n", capatity);
			/* ti2419x_charging_disable(chip, CURRENT, true); */
			/* set_charger_state(chip,1); */
		} else {
			pr_info("charger_status %d\n", charger_status);
			/* set_charger_state(chip,0); */
		}
		is_first_time_check = 0;
	}
}

void print_ti2419x_regs(struct ti2419x_chip *chip)
{
	int i;
	u8	reg[13];

	for (i = 0; i <= 11; i++)
		ti2419x_read(chip, i, &reg[i]);
	pr_info("TI_24296_REG:[0]=%x,[1]=%x,[2]=%x,[3]=%x,[4]=%x,[5]=%x,[6]=%x,[7]=%x,[8]=%x,[9]=%x,[A]=%x,[B]=%x\n",
		reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], reg[6], reg[7], reg[8], reg[9], reg[10], reg[11]);
}

/*print the BMS regs add by zte ssj*/
void print_bms_regs(struct ti2419x_chip *chip)
{
	int bms_capacity, bms_current, bms_voltage, bms_status;
	int bms_fcc, bms_flags, bms_rc, bms_qmax,  temp;

	bms_voltage = ti2419x_get_prop_voltage_now(chip)/1000;
	bms_capacity = ti2419x_get_prop_batt_capacity_real(chip);
	bms_current = ti2419x_get_prop_current_now(chip)/1000;
	bms_status = ti2419x_get_prop_batt_bms_status(chip);
	bms_fcc = ti2419x_get_prop_batt_bms_fcc(chip);
	bms_flags = ti2419x_get_prop_batt_bms_flags(chip);
	bms_rc	  = ti2419x_get_prop_batt_bms_remainingcapacity(chip);
	bms_qmax = ti2419x_get_prop_batt_bms_qmax(chip);
	temp = ti2419x_get_prop_batt_temp(chip);
	pr_info("bms_voltage:%d,bms_capacity:%d,bms_current:%d,bms_status:0x%x\n",
			bms_voltage, bms_capacity, bms_current, bms_status);
	pr_info("bms_fcc:%d,bms_flags:0x%x,bms_rc:%d,bms_qmax:%d,temp:%d\n",
			bms_fcc, bms_flags, bms_rc, bms_qmax, temp);
	pr_info("ti_vol=%d, ti_cap=%d\n,",
		bms_voltage, bms_capacity);
}
/*print the BMS regs add by zte ssj*/


static unsigned int poweroff_enable = 1;

module_param(poweroff_enable, uint, 0644);
MODULE_PARM_DESC(poweroff_enable, "poweroff enable flag bit - 0 disables poweroff");

#define OFFCHG_FORCE_POWEROFF_DELTA (HZ*60*10) /* 10mins */
#define NORMAL_FORCE_POWEROFF_DELTA (HZ*60*1) /* 1mins */
static void force_power_off_check(int capacity)
{
	static unsigned long  report_zero_jiffies = 0;

	/*
	  *report zero, but the uplayer is not shutdown in 60s,
	  *kernel should power off directly.
	  */
	if (capacity == 0) {
		if (report_zero_jiffies == 0) {
			report_zero_jiffies = jiffies;
			pr_info("start check at %ld\n", report_zero_jiffies);
		} else {
			pr_info("offcharging_flag=%d %ld,%ld\n",
			socinfo_get_charger_flag(),  jiffies, report_zero_jiffies);
			if ((socinfo_get_charger_flag()
				&& time_after(jiffies, report_zero_jiffies+OFFCHG_FORCE_POWEROFF_DELTA))
				|| (!socinfo_get_charger_flag()
				&& time_after(jiffies, report_zero_jiffies+NORMAL_FORCE_POWEROFF_DELTA)))
				kernel_power_off();
		}
	} else
		report_zero_jiffies = 0;
}

#ifdef ZTE_CHARGER_TYPE_OEM
#define BATTERY_CURRENT 600
#define ZTE_CONSECUTIVE_COUNT 1
#define NBC1P2_IDEV_CHG_MID 1001
#define SLOW_CHG_CAP_THRES 75

void reset_period(int capacity, int *period)
{
	if (capacity <= 20)
		*period = LOW_SOC_HEARTBEAT_MS;
	else
		*period = HEARTBEAT_MS;
}

void ti22419x_detect_dcp_charger_oem_type(struct ti2419x_chip *chip,
	int battery_current, int capacity, int *period, int battery_status)
{
	static int count = 0;

	if ((!chip->bat_is_cold && !chip->bat_is_cool
		&& !chip->bat_is_warm && !chip->bat_is_hot
		&& chip->therm_lvl_sel == 0 && battery_status == POWER_SUPPLY_STATUS_CHARGING)
		&& charge_type_oem != CHARGER_TYPE_DCP_SLOW) {
		*period = HEARTBEAT_CHARGER_TYPE_OEM_MS;
		if (count == ZTE_CONSECUTIVE_COUNT) {
			if ((-1*battery_current) < BATTERY_CURRENT && capacity <= SLOW_CHG_CAP_THRES) {
				charge_type_oem = CHARGER_TYPE_DCP_SLOW;
				count = 0;
				reset_period(capacity, period);
				power_supply_changed(&chip->batt_psy);
				pr_info("dcp slow success charge_type_oem=%d\n", charge_type_oem);
			} else {
				if (g_charger_detect_counter > DETECT_CHARGER_TYPE_OME_COUNTER) {
					count = 0;
					reset_period(capacity, period);
				}
			}
		} else {
			if ((-1*battery_current) < BATTERY_CURRENT && capacity <= SLOW_CHG_CAP_THRES) {
				count++;
				pr_info("dcp slow start\n");
			} else {
				if (g_charger_detect_counter > DETECT_CHARGER_TYPE_OME_COUNTER) {
					count = 0;
					reset_period(capacity, period);
				}
			}
		}
	} else {
		if (g_charger_detect_counter > DETECT_CHARGER_TYPE_OME_COUNTER) {
			count = 0;
			reset_period(capacity, period);
		}
	}
}

void ti22419x_detect_sdp_charger_oem_type(struct ti2419x_chip *chip,
	int battery_current, int capacity, int *period, int battery_status)
{
	static int count = 0;

	if (chip->usb_psy_ma == NBC1P2_IDEV_CHG_MID) {
		if (charge_type_oem != CHARGER_TYPE_SDP_NBC1P2) {
			if (battery_status == POWER_SUPPLY_STATUS_CHARGING
				|| battery_status == POWER_SUPPLY_STATUS_FULL)
				charge_type_oem = CHARGER_TYPE_SDP_NBC1P2;
			else {
				charge_type_oem = CHARGER_TYPE_SDP_NBC1P2_CHARGR_ERR;
				reset_period(capacity, period);
			}
			pr_info("check charge_type_oem=%d to no bc1.2\n", charge_type_oem);
		} else if (charge_type_oem == CHARGER_TYPE_SDP_NBC1P2
			&& !chip->bat_is_cold && !chip->bat_is_cool
			&& !chip->bat_is_warm && !chip->bat_is_hot
			&& chip->therm_lvl_sel == 0) {
			*period = HEARTBEAT_CHARGER_TYPE_OEM_MS;
			if (count == ZTE_CONSECUTIVE_COUNT) {
				if ((-1*battery_current) < BATTERY_CURRENT && capacity <= SLOW_CHG_CAP_THRES) {
					charge_type_oem = CHARGER_TYPE_SDP_NBC1P2_SLOW;
					count = 0;
					reset_period(capacity, period);
					power_supply_changed(&chip->batt_psy);
					pr_info("check charge_type_oem no bc1.2 & slow success\n");
				} else {
					if (g_charger_detect_counter > DETECT_CHARGER_TYPE_OME_COUNTER) {
						count = 0;
						reset_period(capacity, period);
					}
				}
			} else {
				if ((-1*battery_current) < BATTERY_CURRENT && capacity <= SLOW_CHG_CAP_THRES) {
					count++;
				} else {
					if (g_charger_detect_counter > DETECT_CHARGER_TYPE_OME_COUNTER) {
						count = 0;
						reset_period(capacity, period);
					}
				}
			}
		} else {
			count = 0;
			reset_period(capacity, period);
		}
		pr_info("charge_type_oem=%d\n", charge_type_oem);
	 } else {
		if (g_charger_detect_counter > DETECT_CHARGER_TYPE_OME_COUNTER) {
			count = 0;
			reset_period(capacity, period);
		}
	}
}

void ti22419x_detect_charger_oem_type(struct ti2419x_chip *chip,
	int battery_current, int capacity, int *period, int battery_status)
{
	union power_supply_propval prop = {0,};
	int rc;

	if (chip->usb_present) {
		g_charger_detect_counter++;
		rc = chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_TYPE, &prop);
		pr_info("prop type=%d g_charger_detect_counter=%d\n", prop.intval, g_charger_detect_counter);
		if (prop.intval == POWER_SUPPLY_TYPE_USB)
			ti22419x_detect_sdp_charger_oem_type(chip,
				battery_current, capacity, period, battery_status);
		else if (prop.intval == POWER_SUPPLY_TYPE_USB_DCP)
			ti22419x_detect_dcp_charger_oem_type(chip,
				battery_current, capacity, period, battery_status);
	}
}
#endif

static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *chip = container_of(dwork, struct ti2419x_chip, update_heartbeat_work);
	int rc;
	int period = 0;

	int temp, voltage, cap, status, charge_type, present;
	int chg_current, usb_present, health, ite;

	static int old_temp = 0;
	static int old_cap = 0;
	static int old_status = 0;
	static int old_present = 0;
	static int old_usb_present = 0;
	static int old_health = 0;
	static int count = 0;

	/* zte add start by ssj */
	if (!chip->resume_completed) {
		pr_info("update_heartbeat launched before device-resume, schedule to 5s later\n");
		schedule_delayed_work(&chip->update_heartbeat_work,
			  round_jiffies_relative(msecs_to_jiffies(5000)));
		return;
	}
	/* zte add end by ssj */
	/* kick watch dog */
	rc = ti2419x_masked_write(chip, PON_CFG_REG,
			I2C_WDOG_RESET_MASK, I2C_WDOG_RESET_MASK);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't reset watch dog rc=%d\n", rc);
		schedule_delayed_work(&chip->update_heartbeat_work,
			  round_jiffies_relative(msecs_to_jiffies(5000)));/* add by ssj */
		return;
	}

	/* update state */
	if (chip == NULL) {
		pr_err("pmic fatal error:the_chip=null\n!!");
		return;
	}

	if (!chip->bms_psy) {
		chip->bms_psy = power_supply_get_by_name("on_bms");
		if (chip->bms_psy)
			pr_info("bms psy successful\n");
	}

#if PRINT_TI24296_REG
	 print_ti2419x_regs(chip);
	 print_bms_regs(chip);

#endif

	temp = ti2419x_get_prop_batt_temp(chip)/10;
	voltage = ti2419x_get_prop_voltage_now(chip)/1000;
	cap = ti2419x_get_prop_batt_capacity(chip);
	status = ti2419x_get_prop_batt_status(chip);
	charge_type = ti2419x_get_prop_charge_type(chip);
	present = ti2419x_get_prop_batt_present(chip);
	chg_current = ti2419x_get_prop_current_now(chip)/1000;
	health = ti2419x_get_prop_batt_health(chip);
	usb_present = chip->usb_present;
	ite = ti2419x_get_prop_batt_ite(chip); /* for ON BMS only */

#if defined(LED_GPIO_CONTROL)
	if (status == 1) {
		zte_misc_red_led_control(true);
		zte_misc_green_led_control(false);
	} else if (status == 4) {
		if (usb_present == 1) {
			zte_misc_red_led_control(false);
			zte_misc_green_led_control(true);
		} else {
			zte_misc_red_led_control(false);
			zte_misc_green_led_control(false);
		}
	} else {
		pr_info("The status = %d\n", status);
		zte_misc_red_led_control(false);
		zte_misc_green_led_control(false);
	}
#endif

	/*if heatbeat_ms is bigger than 500ms, it means users need this information, must output the logs directly.*/
	if ((heartbeat_ms >= 500) || (abs(temp-old_temp) >= 1)
		|| (old_cap != cap) || (old_status != status)
		|| (old_present != present) || (old_usb_present != usb_present)
		|| (old_health != health) || (count%5 == 0)) {
		pr_info("***temp=%d,vol=%d,cap=%d,ite=%d,status=%d,chg_state=%d,current=%d\n",
			temp, voltage, cap, ite, status, charge_type, chg_current);
		pr_info("***batt_present=%d,usb_present=%d,chg_en=%d(%d)\n",
			present, usb_present,
			chip->charging_disabled_status, ti2419x_is_charging_enabled(chip));
		pr_info("chip->batt_full=%d,chip->batt_warm_full=%d,chip->batt_real_full=%d\n",
			chip->batt_full, chip->batt_warm_full, chip->batt_real_full);
		old_temp = temp;
		old_cap = cap;
		old_status = status;
		old_present = present;
		old_usb_present = usb_present;
		old_health = health;
		count = 0;
	}

	count++;

	/*
	  *report zero, but the uplayer is not shutdown in 60s,
	  *kernel should power off directly.
	  */
	if (poweroff_enable == 0)
		force_power_off_check(0);
	else
		force_power_off_check(cap);

	power_supply_changed(&chip->batt_psy);
	#ifdef ZTE_CHARGER_TYPE_OEM
	if (heartbeat_ms >= 500) {
		period = heartbeat_ms;
	} else {
		if (cap <= 20)
			period = g_low_soc_update_period_ms;
		else
			period = g_update_period_ms;
	}
	ti22419x_detect_charger_oem_type(chip, chg_current, cap, &period, status);
	#else
	if (heartbeat_ms >= 500) {
		period = heartbeat_ms;
	} else {
		if (cap <= 20)
			period = LOW_SOC_HEARTBEAT_MS;
		else
			period = HEARTBEAT_MS;
	}
	#endif
	schedule_delayed_work(&chip->update_heartbeat_work,
					  round_jiffies_relative(msecs_to_jiffies
								 (period)));
}

/* Move from 8974-UFI */
#define CONSECUTIVE_COUNT	5
#define EOC_CHECK_PERIOD_MS	10000

static void ti_set_charger_state(struct ti2419x_chip *chip, bool disable)
{
	pr_info("disable=%d chip->batt_full=%d\n", disable, chip->batt_full);
	if (disable) {
		ti2419x_charging_disable(chip, CURRENT, true);
		power_supply_changed(&chip->batt_psy);
		pr_info("charger_disable1=%d chip->batt_real_full1=%d\n", disable, chip->batt_real_full);
	} else {
		ti2419x_charging_disable(chip, CURRENT, false);
		power_supply_changed(&chip->batt_psy);
		pr_info("charger_disable2=%d chip->batt_real_full2=%d\n", disable, chip->batt_real_full);
	}
}
#define DEFAULT_FULLCAP	3318
static int max17055_get_fullcap(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->mx_bms) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL, &ret);
			return ret.intval;
		}
	}
	return DEFAULT_FULLCAP;
}

#define DEFAULT_REPCAP	1650
static int max17055_get_repcap(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->mx_bms) {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CHARGE_NOW, &ret);
			return ret.intval;
		}
	}
	return DEFAULT_REPCAP;
}

#define FULL_CAPACITY 100
#define RECHARGE_CAPACITY 99

static bool max17055_recharge_now(struct ti2419x_chip *chip)
{
	int  capacity, fullcap, repcap;

	fullcap = max17055_get_fullcap(chip);
	repcap = max17055_get_repcap(chip);
	capacity = ti2419x_get_prop_batt_capacity_real(chip);
	pr_info("fullcap:%d, repcap:%d, capacity:%d\n", fullcap, repcap, capacity);
	if ((capacity < RECHARGE_CAPACITY) || ((fullcap - repcap) * 1000 > (fullcap * 15))) {
		ti_set_charger_state(chip, false);
		chip->batt_real_full = false;
		return true;
	} else
		return false;
}
static bool max17055_check_charge_term(struct ti2419x_chip *chip, int current_now)
{
	int  capacity, fullcap, repcap;

	fullcap = max17055_get_fullcap(chip);
	repcap = max17055_get_repcap(chip);
	capacity = ti2419x_get_prop_batt_capacity_real(chip);
	pr_info("fullcap:%d, repcap:%d, capacity:%d\n", fullcap, repcap, capacity);
	if ((current_now <= 0) && (abs(current_now) < (chip->iterm_ma)))
		if ((capacity >= RECHARGE_CAPACITY) && (fullcap == repcap))
			return true;
	return false;
}

static void max17055_check_fullcharged_state(struct ti2419x_chip *chip)
{
	int  current_now, capacity;
	static int count = 0;

	capacity = ti2419x_get_prop_batt_capacity_real(chip);
	current_now = ti2419x_get_prop_current_now(chip) / 1000;

	pr_debug("soc =%d,batt_full:%d,current_now:%d,batt_real_full:%d\n",
		capacity, chip->batt_full, current_now, chip->batt_real_full);
	if (chip->batt_real_full) {
		if (!max17055_recharge_now(chip)) {
			pr_info("do not need charge now\n");
			return;
		}
		pr_info("recharge now\n");
	} else if (current_now > 0) {
		pr_info("Charging but system demand increased\n");
		count = 0;
	} else if (!max17055_check_charge_term(chip, current_now)) {
		pr_info("Not at EOC, current_now=%d,capacity:%d\n", current_now, capacity);
		count = 0;
	} else {
		if (count == CONSECUTIVE_COUNT) {
			chip->batt_full = true;
			chip->batt_real_full = true;
			ti_set_charger_state(chip, true);
		} else {
			count += 1;
			pr_info("EOC count = %d\n", count);
		}
	}
}

static bool max17055_warm_cool_recharge_now(struct ti2419x_chip *chip)
{
	int voltage_avg;

	voltage_avg = ti2419x_get_prop_batt_voltage_avg(chip);

	if (chip->tm_state == TI_TEMP_WARM_STATE) {
		if (voltage_avg < (chip->warm_bat_mv - chip->warm_resume_delta_mv))
			return true;
		else
			return false;
	}

	if (chip->tm_state == TI_TEMP_COOL_STATE) {
		if (voltage_avg < (chip->cool_bat_mv - chip->warm_resume_delta_mv))
			return true;
		else
			return false;
	}

	return false;

}

static void max17055_warm_cool_check_fullcharged_state(struct ti2419x_chip *chip)
{
	int  current_now;
	bool recharge_now;
	static int count = 0;

	current_now = ti2419x_get_prop_current_now(chip);
	recharge_now = max17055_warm_cool_recharge_now(chip);

	pr_debug("chip->batt_warm_full:%d,current_avg:%d,recharge_now:%d\n",
		 chip->batt_warm_full, current_now, recharge_now);
	if (chip->batt_warm_full) {
		if (recharge_now) {
			ti_set_charger_state(chip, false);
			chip->batt_warm_full = false;
		} else {
			pr_info("do not need charge now\n");
			return;
		}
	} else if (current_now > 0) {
		pr_info("Charging but system demand increased\n");
		count = 0;
	} else if ((current_now * -1) > (chip->iterm_ma)) {
		pr_info("Not at EOC, ibat_ma=%d\n", current_now);
		count = 0;
	} else {
		if (count == CONSECUTIVE_COUNT) {
			chip->batt_warm_full = true;
			ti_set_charger_state(chip, true);
		} else {
			count += 1;
			pr_info("EOC count = %d\n", count);
		}
	}
}

static void max17055_charger_eoc(struct ti2419x_chip *chip)
{
	if (!chip->resume_completed) {
		pr_info("charger_eoc launched before device-resume, schedule to 2s later\n");
		return;
	}

	if (chip->usb_present == false) {
		pr_info("no chg connected, go through directly\n");
		if (chip->batt_warm_full || chip->batt_full) {
			ti_set_charger_state(chip, false);
			chip->batt_warm_full = false;
			chip->batt_full = false;
		}
		return;
	}

	if (chip->batt_present == false) {
		pr_info("no battery, go through directly\n");
		return;
	}

	if (chip->tm_state == TI_TEMP_NORMAL_STATE) {
		pr_debug("normal temp\n");
		if (chip->batt_warm_full) {
			ti_set_charger_state(chip, false);
			chip->batt_warm_full = false;
		}
		max17055_check_fullcharged_state(chip);
	}

	if (chip->tm_state == TI_TEMP_COOL_STATE || chip->tm_state == TI_TEMP_WARM_STATE) {
		pr_debug("warm or cool temp\n");
		max17055_warm_cool_check_fullcharged_state(chip);
	}
}

static void charger_eoc(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *chip = container_of(dwork, struct ti2419x_chip, charger_eoc_work);
	static int count = 0;
	static int vbat_low_count = 0;
	int vbat_mv, capacity_soc;
	int charger_status = 0;
	int delay_timer;
	int is_chg_in = 0;
	int rc = 0;
	int max_float_voltage = 0;

	if (!chip->resume_completed) {
		pr_info("charger_eoc launched before device-resume, schedule to 2s later\n");
		schedule_delayed_work(&chip->charger_eoc_work, round_jiffies_relative(msecs_to_jiffies(5000)));
		return;
	}

	capacity_soc = ti2419x_get_prop_batt_capacity(chip);
	is_chg_in	 = ti2419x_is_chg_plugged_in(chip);
	if (chip->mx_bms) {
		max17055_charger_eoc(chip);
		goto check_again_later;
	}

	if (is_chg_in == 0) {
		pr_info("no chg connected, go through directly\n");
		count = 0;
		vbat_low_count = 0;
		goto check_again_later;
	}

	if (!chip->bat_is_warm && !chip->bat_is_cool) {
		pr_debug("temp not in  warm/cool mode\n");
		check_fullcharged_state(chip);
		goto check_again_later;
	}

	pr_debug("temp in warm mode\n");

	/*check for recharging*/
	rc = ti2419x_float_voltage_get(chip, &max_float_voltage);
	if (rc < 0) {
		pr_err("error when get float voltage from register\n");
		goto check_again_later;
	}

	vbat_mv = ti2419x_get_prop_voltage_now(chip)/1000;
	if (chip->batt_warm_full) {
		if (vbat_mv <= (max_float_voltage - chip->warm_resume_delta_mv - chip->vbatdet_max_err_mv)) {
			pr_info("need recharging now\n");
			ti2419x_charging_disable(chip, CURRENT, false);
			chip->batt_warm_full = false;
		} else
			pr_debug("No need recharging, check it again\n");

	} else {
		/*check for full charging*/
		charger_status = ti2419x_get_charging_status(chip);
		if (charger_status == FAST_CHARGE) {
			if ((vbat_mv <	(max_float_voltage - chip->warm_resume_delta_mv  - chip->vbatdet_max_err_mv))
				&& (chip->bat_is_warm || chip->bat_is_cool)) {
				pr_info("woke up too early vbat_mv = %d, max_mv = %d, resume_mv = %d\n",
						vbat_mv, max_float_voltage,
						chip->resume_delta_mv
						);
				pr_info("tolerance_mv = %d low_count = %d soc=%d\n",
						chip->vbatdet_max_err_mv,
						vbat_low_count,
						capacity_soc);
				count = 0;
				goto check_again_later;
			}
			if (chip->on_bms) {
				/*check for warm full by soc */
				/*SOC 75% ~ 4.05v  */
				if (capacity_soc > 75) {
					if (count == CONSECUTIVE_COUNT) {
						chip->batt_warm_full = true;
						ti2419x_charging_disable(chip, CURRENT, true);

						pr_info("End of warm/cool Charging.\n");
						pr_info("chip->batt_warm_full:%d\n", chip->batt_warm_full);
						power_supply_changed(&chip->batt_psy);
					} else {
						count += 1;
						pr_info("EOC count = %d\n", count);
					}
				 }
			} else {
				/*check for warm full by ibat current */
				int ibat_ma;

				ibat_ma	= ti2419x_get_prop_current_now(chip) / 1000;
				if ((ibat_ma * -1) > chip->iterm_ma) {
					pr_info("Not at EOC, battery current too high, ibat_ma=%d\n", ibat_ma);
					count = 0;
				} else if (ibat_ma > 0) {
					pr_info("Charging but system demand increased\n");
					count = 0;
				} else {
					if (count == CONSECUTIVE_COUNT) {
						chip->batt_warm_full = true;
						ti2419x_charging_disable(chip, CURRENT, true);

						pr_info("End of Charging.\n");
						pr_info("chip->batt_warm_full:%d\n", chip->batt_warm_full);
						power_supply_changed(&chip->batt_psy);
						/* goto stop_eoc; */
					} else {
						count += 1;
						pr_info("EOC count = %d\n", count);
					}
				}
			}
		} else
			pr_debug("not in fast charging\n");
	}

check_again_later:
	if ((capacity_soc <= 60) || (is_chg_in == 0))
		delay_timer = EOC_CHECK_PERIOD_MS * 10;
	else
		delay_timer = EOC_CHECK_PERIOD_MS;

	schedule_delayed_work(&chip->charger_eoc_work, round_jiffies_relative(msecs_to_jiffies(delay_timer)));
}

#if FEATURE_SOFT_CC
/*ZTE:	Soft CC*/
#define SOFT_CC_NUM 60
struct qpnp_soft_cc {
	int data[SOFT_CC_NUM];
	int pos;
	int total;
	bool monitor_launched;
	bool total_valid;
};

static struct qpnp_soft_cc soft_cc;
#if SOFT_CC_DEBUG
static char debug_buff[1024] = {0};
#endif

static void soft_cc_append_data(int data)
{
	struct qpnp_soft_cc *sc = &soft_cc;

	int pos = (sc->pos + 1) % SOFT_CC_NUM;
	int old_data = sc->data[pos];

	if (sc->pos >= SOFT_CC_NUM-1)
		sc->total_valid = true;

	sc->data[pos] = data;
	sc->pos = pos;
	sc->total = sc->total - old_data + data;
#if SOFT_CC_DEBUG
	{
	int i = 0, cnt = 0;

	memset(debug_buff, 0, sizeof(debug_buff));
	for (i = 0; i < SOFT_CC_NUM; i++) {
		if (i == sc->pos)
			cnt += snprintf(debug_buff + cnt, sizeof(sc->data[i]), "%5d*", sc->data[i]);
		else
			cnt += snprintf(debug_buff + cnt, sizeof(sc->data[i]), "%5d ", sc->data[i]);
	}
	pr_info("==soft cc dump==\n");
	pr_info("%s\n", debug_buff);
	pr_info("\ttotal=%d\n", sc->total);
	pr_info("\tpos=%d\n", sc->pos);
	pr_info("\ttotal_valid=%d\n", sc->total_valid);
	pr_info("\tmonitor_launched=%d\n", sc->monitor_launched);
	}
#endif
}

static void soft_cc_reset(void)
{
	struct qpnp_soft_cc *sc = &soft_cc;
	int i = 0;

	for (i = 0; i < SOFT_CC_NUM; i++)
		sc->data[i] = 0;
	sc->pos  = -1;
	sc->total = 0;
	sc->total_valid = false;
	sc->monitor_launched = false;
}

/* this function called when soc=0 and charger in */
static int soft_cc_adjust_soc(struct ti2419x_chip *chip)
{
	struct qpnp_soft_cc *sc = &soft_cc;

#if SOFT_CC_DEBUG
	pr_info("sc->total_valid=%d sc->total=%d\n", sc->total_valid, sc->total);
#endif
	if (sc->total_valid && (sc->total > 0)) {
		pr_info("soc=0 and charger in, soft_cc_total>0, report 0\n");
		return 0;
	if (!sc->monitor_launched) {
		soft_cc_reset();
		sc->monitor_launched = true;
		pr_info("soc=0 and charger is in, launching soft_cc_monitor_work\n");
		schedule_delayed_work(&chip->soft_cc_monitor_work,
			round_jiffies_relative(msecs_to_jiffies(1000)));
	}
	return 1;
}

static void soft_cc_monitor(struct work_struct *work)
{
	int chg_current = 0;
	int usb_present = 0;
	int cap = 0;
	int vol = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *chip = container_of(dwork,
				struct ti2419x_chip, soft_cc_monitor_work);

	if (chip == NULL) {
		pr_err("pmic fatal error:the_chip=null\n!!");
		return;
	}
	if (!chip->resume_completed) {
		pr_info("soft_cc_monitor launched before device-resume, schedule to 2s later\n");
		schedule_delayed_work(&chip->soft_cc_monitor_work,
					  round_jiffies_relative(msecs_to_jiffies
								 (1000)));
		return;
	}
	chg_current = ti2419x_get_prop_current_now(chip)/1000;/* mA */
	usb_present = ti2419x_is_chg_plugged_in(chip);
	cap = ti2419x_get_prop_batt_capacity(chip);
	vol = ti2419x_get_prop_voltage_now(chip);
	if ((vol <= 3600000) && (cap <= 2) &&  usb_present) {
		soft_cc_append_data(chg_current);
		if (cap != 0)
			schedule_delayed_work(&chip->soft_cc_monitor_work,
					  round_jiffies_relative(msecs_to_jiffies
								 (1000)));
	} else {
		soft_cc_reset();
		pr_info("soft cc reset , cap=%d usb_present=%d\n", cap, usb_present);
	}
}
#endif
/* FEATURE_SOFT_CC */

void bq27x00_notify(void)
{
	pr_info("\n");
	if (the_ti2419x_chip) {
		cancel_delayed_work(&the_ti2419x_chip->update_heartbeat_work);
		schedule_delayed_work(&the_ti2419x_chip->update_heartbeat_work, 0);
	}
}


static int ti2419x_parse_dt(struct ti2419x_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "zte,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0)
		chip->vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "zte,charging-timeout",
						&chip->safety_time);
	if (rc < 0)
		chip->safety_time = -EINVAL;

	switch (ic_type) {
	case TI_24X9X_CHARGER_IC:
		if (!rc && (chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
			dev_err(chip->dev, "Bad charging-timeout %d\n",
					chip->safety_time);
			return -EINVAL;
		}
		break;
	case TI_25601_CHARGER_IC:
		if (!rc && (chip->safety_time > ti25601_chg_time[ARRAY_SIZE(ti25601_chg_time) - 1])) {
			dev_err(chip->dev, "Bad charging-timeout %d\n",
					chip->safety_time);
			return -EINVAL;
		}
		break;
	}

	rc = of_property_read_u32(node, "zte,recharge-thresh-mv",
						&chip->resume_delta_mv);
	if (rc < 0)
		chip->resume_delta_mv = -EINVAL;
	rc = of_property_read_u32(node, "zte,warm_recharge-thresh-mv",
						&chip->warm_resume_delta_mv);
	if (rc < 0)
		chip->resume_delta_mv = -EINVAL;


	rc = of_property_read_u32(node, "zte,vbatdet-max-err-mv",
						&chip->vbatdet_max_err_mv);
	if (rc < 0)
		chip->vbatdet_max_err_mv = -EINVAL;

	rc = of_property_read_u32(node, "zte,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	chip->hw_iterm_disabled = of_property_read_bool(node,
						"zte,hw-iterm-disabled");

	chip->charging_disabled = of_property_read_bool(node,
						"zte,charging-disabled");
	pr_info("hw-iterm-disabled: %d charging-disabled: %d\n",
		chip->hw_iterm_disabled,
		chip->charging_disabled);

	/* iusb */
	rc = of_property_read_u32(node,
						"zte,max_usb_current", &chip->max_iusb);
	if (rc < 0)
		chip->max_iusb = -EINVAL;
	pr_info("zte,max usb input current: %d\n", chip->max_iusb);

	/* ibat */
	rc = of_property_read_u32(node,
						"zte,max_battery_current", &chip->max_ibat);
	if (rc < 0)
		chip->max_ibat = -EINVAL;
	pr_info("zte,max battery charge current: %d\n", chip->max_ibat);

	/* input voltage */
	rc = of_property_read_u32(node,
						"zte,input_voltage_mv", &chip->max_input_voltage);
	if (rc < 0)
		chip->max_input_voltage = -EINVAL;
	pr_info("zte,input voltage: %d\n", chip->max_input_voltage);

	rc = of_property_read_u32(node, "zte,warm_bat_mv", &chip->warm_bat_mv);
	if (rc < 0)
		chip->warm_bat_mv = -EINVAL;
	pr_info("warm_bat_mv: %d\n", chip->warm_bat_mv);

	rc = of_property_read_u32(node, "zte,cool_bat_mv", &chip->cool_bat_mv);
	if (rc < 0)
		chip->cool_bat_mv = -EINVAL;
	pr_info("cool_bat_mv: %d\n", chip->cool_bat_mv);

	/* move from qpnp-charger.c */
	rc = of_property_read_u32(node, "zte,warm-bat-decidegc", &chip->warm_bat_decidegc);
	if (rc < 0)
		chip->warm_bat_decidegc = ZTE_WARM_TEMP_DEFAULT; /* default 45 deg C */
	pr_info("warm_bat_decidegc: %d\n", chip->warm_bat_decidegc);

	rc = of_property_read_u32(node, "zte,cool-bat-decidegc", &chip->cool_bat_decidegc);
	if (rc < 0)
		chip->cool_bat_decidegc = ZTE_COOL_TEMP_DEFAULT; /* default 10 deg C */
	pr_info("cool_bat_decidegc: %d\n", chip->cool_bat_decidegc);

	rc = of_property_read_u32(node, "zte,cooler-bat-decidegc", &chip->cooler_bat_decidegc);
	if (rc < 0)
		chip->cooler_bat_decidegc = ZTE_COOL_TEMP_DEFAULT; /* default 10 deg C */
	pr_info("cooler_bat_decidegc: %d\n", chip->cooler_bat_decidegc);

	rc = of_property_read_u32(node, "zte,hot-bat-decidegc", &chip->hot_bat_decidegc);
	if (rc < 0)
		chip->hot_bat_decidegc = ZTE_HOT_TEMP_DEFAULT;	/* default 50 deg C */
	pr_info("hot_bat_decidegc: %d\n", chip->hot_bat_decidegc);

	rc = of_property_read_u32(node, "zte,cold-bat-decidegc", &chip->cold_bat_decidegc);
	if (rc < 0)
		chip->cold_bat_decidegc = ZTE_COLD_TEMP_DEFAULT;  /* default 0 deg C */
	pr_info("cold_bat_decidegc: %d\n", chip->cold_bat_decidegc);

	rc = of_property_read_u32(node, "zte,batt-hot-percentage", &chip->hot_batt_p);
	if (rc < 0)
		chip->hot_batt_p = -EINVAL;
	pr_info("hot_batt_p: %d\n", chip->hot_batt_p);

	rc = of_property_read_u32(node, "zte,batt-cold-percentage", &chip->cold_batt_p);
	if (rc < 0)
		chip->cold_batt_p = -EINVAL;
	pr_info("cold_batt_p: %d\n", chip->cold_batt_p);

	/* end move from qpnp-charger.c */

	rc = of_property_read_u32(node, "zte,warm_bat_chg_ma", &chip->warm_bat_chg_ma);
	if (rc < 0)
		chip->warm_bat_chg_ma = -EINVAL;
	pr_info("warm_bat_chg_ma: %d\n", chip->warm_bat_chg_ma);

	rc = of_property_read_u32(node, "zte,cool_bat_chg_ma", &chip->cool_bat_chg_ma);
	if (rc < 0)
		chip->cool_bat_chg_ma = -EINVAL;
	pr_info("cool_bat_chg_ma: %d\n", chip->cool_bat_chg_ma);

	rc = of_property_read_u32(node, "zte,cooler_bat_chg_ma", &chip->cooler_bat_chg_ma);
	if (rc < 0)
		chip->cooler_bat_chg_ma = chip->cool_bat_chg_ma;
	pr_info("cooler_bat_chg_ma: %d\n", chip->cooler_bat_chg_ma);

	rc = of_property_read_u32(node, "zte,float-voltage-mv", &chip->cfg_max_voltage_mv);
	if (rc < 0)
		chip->cfg_max_voltage_mv = -EINVAL;
	pr_info("cfg_max_voltage_mv: %d\n", chip->cfg_max_voltage_mv);

	if (of_find_property(node, "zte,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
					chip->thermal_levels,
						GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"zte,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			pr_err("Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	/* fg params */
	return 0;
}

static int ti2419x_debug_data_set(void *data, u64 val)
{
	struct ti2419x_chip *chip = data;

	pr_info("reg=%d val=%llu\n", chip->reg_addr, val);
	__ti2419x_write(chip, chip->reg_addr, val);
	return 0;
}

static int ti2419x_debug_data_get(void *data, u64 *val)
{
	struct ti2419x_chip *chip = data;
	u8 temp = 0;

	ti2419x_read(chip, chip->reg_addr, &temp);
	*val = (u64) temp;
	pr_info("reg=%d val=%llu\n", chip->reg_addr, *val);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ti2419x_debug_data_fops, ti2419x_debug_data_get,
			ti2419x_debug_data_set, "%llu\n");


static struct power_supply *ti2419x_get_bms_psy(struct ti2419x_chip *chip)
{
	struct power_supply *bms_psy;

	bms_psy = power_supply_get_by_name("max17055_bms");
	if (!bms_psy) {
		pr_info("ti2419x_get_bms_psy4\n");
		pr_err("MAX17055 supply not found!\n");
	} else {
		pr_info("MAX17055 supply found\n");
		chip->on_bms = false;
		chip->mx_bms = true;
		chip->ti_bms = false;
		chip->vm_bms = false;
		goto out;
	}

	bms_psy = power_supply_get_by_name("on_bms");
	if (!bms_psy) {
		pr_err("ON BMS supply not found!\n");
	} else {
		pr_info("ON BMS supply found\n");
		chip->on_bms = true;
		chip->mx_bms = false;
		chip->ti_bms = false;
		chip->vm_bms = false;
		goto out;
	}

	bms_psy = power_supply_get_by_name("ti_bms");
	if (!bms_psy) {
		pr_err("TI BMS supply not found!\n");
	} else {
		pr_info("TI BMS supply found\n");
		chip->on_bms = false;
		chip->mx_bms = false;
		chip->ti_bms = true;
		chip->vm_bms = false;
		goto out;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		pr_err("VM BMS supply not found!\n");
	} else {
		pr_info("VM BMS supply found\n");
		chip->on_bms = false;
		chip->mx_bms = false;
		chip->ti_bms = false;
		chip->vm_bms = true;
		goto out;
	}
out:
	return bms_psy;
}

static int ti2419x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	u8 reg_2419x, reg_25601;
	int rc;
	struct ti2419x_chip *chip;
	struct power_supply *usb_psy, *bms_psy;

	pr_info("Charger IC enter\n");

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	pr_info("ti2419x_get_bms_psy\n");
	bms_psy = ti2419x_get_bms_psy(chip);
	pr_info("ti2419x_get_bms_psy\n");
	if (!bms_psy) {
		dev_dbg(&client->dev, "BMS supply not found; defer probe\n");
		devm_kfree(&client->dev, chip);
		return -EPROBE_DEFER;
	}
	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->bms_psy = bms_psy;
	chip->fake_battery_soc = -EINVAL;
	mutex_init(&chip->read_write_lock);

	/* probe the device to check if its actually connected */
	rc = ti2419x_read(chip, VENDOR_REG, &reg_2419x);
	pr_err("zty_kernel the reg is 0x%x\n", reg_2419x);
	reg_2419x >>= TI2419X_VENDOR_SHIFT;
	pr_err("zty_kernel the reg_2419x is 0x%x\n", reg_2419x);
	if (rc) {
		pr_err("Failed to detect TI 2419x, device may be absent\n");
		return -ENODEV;
	}

	rc = ti2419x_read(chip, TI25601_VENDOR_REG, &reg_25601);
	pr_err("zty_kernel the reg is 0x%x\n", reg_25601);
	reg_25601 >>= TI25601_VENDOR_SHIFT;
	pr_err("zty_kernel the reg_256013 is 0x%x\n", reg_25601);
	if (rc) {
		pr_err("Failed to detect TI 25601, device may be absent\n");
		return -ENODEV;
	}

	if ((reg_2419x != TI_CHARGER_VENDOR) && (reg_25601 == TI_CHARGER_VENDOR))
		ic_type = TI_25601_CHARGER_IC;
	else if ((reg_2419x == TI_CHARGER_VENDOR) && (reg_25601 != TI_CHARGER_VENDOR))
		ic_type = TI_24X9X_CHARGER_IC;

	rc = ti2419x_parse_dt(chip);
	if (rc < 0) {
		dev_err(&client->dev, "Unable to parse DT nodes\n");
		return rc;
	}

	device_init_wakeup(chip->dev, 1);
	i2c_set_clientdata(client, chip);
	chip->resume_completed = true;
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->charging_disable_lock);
	mutex_init(&chip->current_change_lock);
	mutex_init(&chip->jeita_configure_lock);
	mutex_init(&chip->chg_enable_lock);
	chip->default_i2c_addr = client->addr;

	pr_debug("default_i2c_addr=%x\n", chip->default_i2c_addr);
	alarm_init(&chip->charging_expired_alarm, ALARM_BOOTTIME, ti2419x_charging_expired_alarm_cb);
	INIT_WORK(&chip->poweroff_work, offcharge_poweroff_work);
	wake_lock_init(&chip->charger_wake_lock, WAKE_LOCK_SUSPEND, "zte_chg_event");
	wake_lock_init(&chip->charger_valid_lock, WAKE_LOCK_SUSPEND, "zte_chg_valid");
	INIT_DELAYED_WORK(&chip->update_heartbeat_work, update_heartbeat);
	INIT_DELAYED_WORK(&chip->charger_eoc_work, charger_eoc);
	INIT_DELAYED_WORK(&chip->temp_control_work, ti_temp_control_func);
	INIT_DELAYED_WORK(&chip->charging_policy_work, ti2419x_charging_policy_work);
	INIT_DELAYED_WORK(&chip->usb_type_detection_work, ti2419x_usb_type_detection_work);
#if FEATURE_SOFT_CC
	INIT_DELAYED_WORK(&chip->soft_cc_monitor_work, soft_cc_monitor);/* zte */
	soft_cc_reset();/* ZTE:initialize soft_cc parameters */
#endif

	/* for usb-otg */
	rc = ti2419x_regulator_init(chip);
	if	(rc) {
		dev_err(&client->dev,
			"Couldn't initialize ti2419x ragulator rc=%d\n", rc);
		return rc;
	}

	rc = ti2419x_hw_init(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to initialize hardware rc = %d\n", rc);
		goto fail_hw_init;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto fail_hw_init;
	}

	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= ti2419x_battery_get_property;
	chip->batt_psy.set_property	= ti2419x_battery_set_property;
	chip->batt_psy.properties	= ti2419x_battery_properties;
	chip->batt_psy.num_properties  = ARRAY_SIZE(ti2419x_battery_properties);
	chip->batt_psy.external_power_changed = ti2419x_external_power_changed;
	chip->batt_psy.property_is_writeable = ti2419x_battery_is_writeable;
	chip->batt_psy.supplied_to = pm_batt_supplied_to;
	chip->batt_psy.num_supplicants =
		ARRAY_SIZE(pm_batt_supplied_to);

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto fail_hw_init;
	}

	/* NOTE(by zte jzn 20150714):
	*  Here set_batt_hot_cold_threshold() is used for correcting different batteries' NTC therm;
	*  it is defined in qpnp-adc-common.c
	*/
	pr_info("hot threshold: %d, cold threshold: %d\n",
		chip->hot_batt_p, chip->cold_batt_p);

	chip->vadc_dev = qpnp_get_vadc(chip->dev, "tichg");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("%s ,vadc property missing\n", __func__);
		else
			pr_err("%s ,vadc property fail\n", __func__);
		pr_info("ZTE !!!!error rc=%d\n", rc);
		/* goto fail_chg_enable; */
	}
	/* STAT irq configuration */
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				ti2419x_stat_handler, IRQF_ONESHOT,
				"ti2419x_stat_irq", chip);
		if (rc < 0) {
			dev_err(&client->dev,
				"request_irq for irq=%d  failed rc = %d\n",
				client->irq, rc);
			goto unregister_batt_psy;
		}
		enable_irq_wake(client->irq);
	}

	chip->debug_root = debugfs_create_dir("ti2419x", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cnfg debug file\n");
		ent = debugfs_create_u8("address", S_IRUSR | S_IWUSR,
					chip->debug_root,
					&chip->reg_addr);
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create address debug file\n");
		}

		ent = debugfs_create_file("data",  S_IRUSR | S_IWUSR,
					chip->debug_root, chip,
					&ti2419x_debug_data_fops);
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create data debug file\n");
		}

		ent = debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_writes));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file\n");

		ent = debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_reads));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file\n");

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create count debug file\n");
	}

	if (client->irq) {
		pr_info("call ti2419x_stat_handler when probe finish\n");
		ti2419x_stat_handler(client->irq, chip);
	}

	/*if in offcharging and usb is absent,run power off*/
	if (socinfo_get_charger_flag() && !chip->usb_present) {
		pr_info("in offcharging and usb removed.poweroff!!!\n");
		/* kernel_power_off(); */
		schedule_work(&chip->poweroff_work);
	}

	schedule_delayed_work(&chip->update_heartbeat_work, 0);
	schedule_delayed_work(&chip->charger_eoc_work, 0);
	schedule_delayed_work(&chip->temp_control_work, 0);
	the_ti2419x_chip	=	chip;
	charging_policy_init(chip);
	zte_misc_register_charging_policy_ops(&chip->battery_charging_policy_ops);

	pr_info("probe success! batt=%d usb=%d soc=%d\n",
			ti2419x_get_prop_batt_present(chip),
			chip->usb_present,
			ti2419x_get_prop_batt_capacity(chip));
	return 0;

unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
fail_hw_init:
	regulator_unregister(chip->otg_vreg.rdev); /* for usb-otg */
	return rc;
}

static int ti2419x_remove(struct i2c_client *client)
{
	struct ti2419x_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->charger_eoc_work);	 /* stop charger_eoc_work. */
	cancel_delayed_work_sync(&chip->temp_control_work);   /* stop temp_control_work. */
	cancel_delayed_work_sync(&chip->update_heartbeat_work);
	cancel_delayed_work_sync(&chip->charging_policy_work);
	cancel_delayed_work_sync(&chip->usb_type_detection_work);
	cancel_work_sync(&chip->poweroff_work);
	regulator_unregister(chip->otg_vreg.rdev); /* for usb-otg */
	power_supply_unregister(&chip->batt_psy);
	mutex_destroy(&chip->charging_disable_lock);
	mutex_destroy(&chip->current_change_lock);
	mutex_destroy(&chip->read_write_lock);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->jeita_configure_lock);
	debugfs_remove_recursive(chip->debug_root);

	pr_info("ti chg driver exit...\n");
	return 0;
}

static int ti2419x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ti2419x_chip *chip = i2c_get_clientdata(client);

	pr_info("enter ti2419x_suspend\n");
	set_charge_wdog(chip, TI2419X_WDOG_DISABLE);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);
	cancel_delayed_work_sync(&chip->charger_eoc_work);	 /* stop charger_eoc_work. */
	cancel_delayed_work_sync(&chip->temp_control_work);   /* stop temp_control_work. */
	cancel_delayed_work_sync(&chip->update_heartbeat_work);
#if FEATURE_SOFT_CC
	cancel_delayed_work_sync(&chip->soft_cc_monitor_work);
#endif

	return 0;
}

static int ti2419x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ti2419x_chip *chip = i2c_get_clientdata(client);

	pr_info("enter ti2419x_suspend_noirq\n");
	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int ti2419x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ti2419x_chip *chip = i2c_get_clientdata(client);

	pr_info("enter ti2419x_resume\n");
	chip->fake_battery_soc = -EINVAL;
	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	if (chip->irq_waiting) {
		mutex_unlock(&chip->irq_complete);
		ti2419x_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	} else {
		mutex_unlock(&chip->irq_complete);
	}
	/* kick watch dog zte ssj */
	if (ti2419x_masked_write(chip, PON_CFG_REG, I2C_WDOG_RESET_MASK, I2C_WDOG_RESET_MASK) < 0)
		dev_err(chip->dev, "Couldn't reset watch dog\n");
	set_charge_wdog(chip, TI2419X_WDOG_160S);

	/* NOTE(by zte JZN)--20150720
	* 1)delay updateheart workqueue to 5s later.
	* 2)If the system be awake less than 5s, it's not need to run update_heartbeat_work.
	* 3)In ti2419x_suspend, the below workqueues will be canceled
	*/
	schedule_delayed_work(&chip->update_heartbeat_work, round_jiffies_relative(msecs_to_jiffies(5000)));
	schedule_delayed_work(&chip->charger_eoc_work, 0);
	schedule_delayed_work(&chip->temp_control_work, 0);

	power_supply_changed(&chip->batt_psy);
	return 0;
}

static const struct dev_pm_ops ti2419x_pm_ops = {
	.resume		= ti2419x_resume,
	.suspend_noirq	= ti2419x_suspend_noirq,
	.suspend	= ti2419x_suspend,
};

static const struct of_device_id ti2419x_match_table[] = {
	{ .compatible = "zte,ti2419x-chg",},
	{ },
};

static const struct i2c_device_id ti2419x_id[] = {
	{"ti2419x-chg", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ti2419x_id);

static struct i2c_driver ti2419x_driver = {
	.driver		= {
		.name		= "ti2419x-chg",
		.owner		= THIS_MODULE,
		.of_match_table	= ti2419x_match_table,
		.pm		= &ti2419x_pm_ops,			/* //irq can't be disabled individually */
	},
	.probe		= ti2419x_probe,
	.remove		= ti2419x_remove,
	.id_table	= ti2419x_id,
};

static int __init ti2419x_init(void)
{
	int ret;

	pr_info("%s()\n", __func__);
	ret = i2c_add_driver(&ti2419x_driver);

	if (ret)
		pr_info("Unable to register ti2419x_init i2c driver\n");

	return ret;
}

static void __exit ti2419x_exit(void)
{
	pr_info("%s()\n", __func__);
	i2c_del_driver(&ti2419x_driver);
}
/* module_init(ti2419x_init); */
late_initcall(ti2419x_init);
/* module_exit(lc709203f_exit); */


MODULE_DESCRIPTION("TI 25601 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:ti2419x-chg");

