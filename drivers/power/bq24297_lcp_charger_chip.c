/*
 * BQ2560x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#define DEBUG

#define pr_fmt(fmt)	"%s " fmt, __func__

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
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>

#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>

#include <linux/pinctrl/consumer.h>

#include <linux/pmic-voter.h>
#include <linux/alarmtimer.h>
#include "bq2429x_reg.h"
#include "bq2429x.h"
#include "zte_misc.h"

#define CHG_SMOOTH_BATTERY_PROP

#ifdef CHG_SMOOTH_BATTERY_PROP
#define CALC_AVG_NUMS 5

#define BATTERY_COLD 0
#define BATTERY_COOL 1
#define BATTERY_NORMAL 2
#define BATTERY_WARM 3
#define BATTERY_HOT 4

struct battery_smooth_prop {
	int pre_soc;
	int current_soc;
	int fg_soc;
	int ui_soc;
	int soc_rate;
	int soc_direction;

	int battery_therm_status;
	int battery_status;
	int chg_present;

	int chg_unplugged_time;
	bool pre_chg_present;

	int temps[CALC_AVG_NUMS];
	int voltages[CALC_AVG_NUMS];
	int avg_volt;
	int avg_temp;

	bool overdischarge;

	s64 current_boot_sec;
	s64 battinfo_print_sec;

	struct delayed_work battery_smooth_up;
	struct delayed_work battery_smooth_down;
	struct mutex mutex_smoothing_battery;
};

static struct battery_smooth_prop g_smooth_prop;

#endif
enum bq2429x_vbus_type {
	BQ2429X_VBUS_NONE = REG08_VBUS_TYPE_NONE,
	BQ2429X_VBUS_USB = REG08_VBUS_TYPE_USB,
	BQ2429X_VBUS_ADAPTER = REG08_VBUS_TYPE_ADAPTER,
	BQ2429X_VBUS_OTG = REG08_VBUS_TYPE_OTG,
};



enum bq2429x_part_no {
	BQ24295 = 0x06,
	BQ24296 = 0x01,
	BQ24297 = 0x03,
};

enum bq2429x_charge_state {
	CHARGE_STATE_IDLE = REG08_CHRG_STAT_IDLE,
	CHARGE_STATE_PRECHG = REG08_CHRG_STAT_PRECHG,
	CHARGE_STATE_FASTCHG = REG08_CHRG_STAT_FASTCHG,
	CHARGE_STATE_CHGDONE = REG08_CHRG_STAT_CHGDONE,
};

#define USER_FCC_VOTER "USER_FCC_VOTER"
#define CHARGER_TYPE_FCC_VOTER "CHARGER_TYPE_FCC_VOTER"
#define BATT_TEMP_FCC_VOTER "BATT_TEMP_FCC_VOTER"
#define THERMAL_FCC_VOTER "THERMAL_FCC_VOTER"
#define SUSPEND_FCC_VOTER "SUSPEND_FCC_VOTER"
#define CHG_TERM_SOC_FCC_VOTER "CHG_TERM_SOC_FCC_VOTER"
#define CHARGING_POLICY_FCC_VOTER "CHARGING_POLICY_FCC_VOTER"
#define NUM_FCC_VOTER 7

#define CHARGER_TYPE_ICL_VOTER "CHARGER_TYPE_ICL_VOTER"
#define PSY_ICL_VOTER "PSY_ICL_VOTER"
#define THERMAL_ICL_VOTER "THERMAL_ICL_VOTER"
#define USER_ICL_VOTER "USER_ICL_VOTER"
#define SUSPEND_ICL_VOTER "SUSPEND_ICL_VOTER"
#define PA_THERMAL_ICL_VOTER "PA_THERMAL_ICL_VOTER"
#define CHARGING_POLICY_ICL_VOTER "CHARGING_POLICY_ICL_VOTER"
#define NUM_ICL_VOTER 7

#define BQ2429X_STATUS_PLUGIN	0x00000001

struct bq2429x_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};


struct bq2429x {
	struct device *dev;
	struct i2c_client *client;

	enum bq2429x_part_no part_no;
	int	revision;

	int chip;
	int vbus_type;
	int chg_type;

	int status;
	int fake_rsoc;

	bool chg_term_full_charge_detected;
	bool chg_term_by_soc;
	int chg_term_battery_status;

	struct mutex lock;
	struct mutex current_change_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;
	bool dpdm_enabled;

	bool chg_present;
	bool prop_set_charge_enabled;
	bool charge_enabled;
	bool otg_enabled;
	bool batfet_enabled;

	bool dpm_triggered;

	bool in_therm_regulation;
	bool in_vsys_regulation;
	bool resume_completed;
	bool power_good;

	bool monitor_work_waiting;
	bool irq_waiting;

	int	usb_psy_ma;

	int charge_state;

	int fault_status;

	struct bq2429x_platform_data *platform_data;

	struct delayed_work monitor_work;
	/*struct delayed_work jeita_work;*/
	struct delayed_work otg_boot_work;
	struct delayed_work usb_type_detection_work;
	struct delayed_work dcp_icl_recheck_work;
	struct alarm charging_expired_alarm;

	struct dentry *debug_root;
	unsigned int			peek_poke_address;
	struct bq2429x_otg_regulator otg_vreg;

	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply batt_psy;

	const char		*bms_psy_name;

	struct pinctrl  *pinctrl;
	struct pinctrl_state  *enpin_output0;

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

	int usb_dp_dm_status;

	int batt_temp;

	int jeita_ma;
	int jeita_mv;

	unsigned int    thermal_levels;
	unsigned int    therm_lvl_sel;
	unsigned int    *thermal_mitigation;
	int             shipping_mode_enabled;
	struct wake_lock battery_suspend_lock;
	struct wake_lock smb_monitor_wake_lock;
	struct wake_lock usb_plug_in_out_wake_lock;
	/* voters */
	struct votable    *fcc_votable;
	struct votable    *usb_icl_votable;

	int psy_health_sts;
	bool input_unspec;
	bool need_recheck_usb_type;

	bool test_disable_feed_wdt;
	bool use_test_temp;
	int test_temp;
	bool use_test_batt_volt;
	int test_batt_volt;

	struct charging_policy_ops battery_charging_policy_ops;
	struct delayed_work charging_policy_work;
};

static u64 g_charging_expiration_ns = 0;
static bool charger_boot = false;
static struct bq2429x *g_bq = NULL;

int bq2429x_set_input_current_limit(struct bq2429x *bq, int curr);
int bq2429x_set_input_volt_limit(struct bq2429x *bq, int volt);
int bq2429x_set_chargevoltage(struct bq2429x *bq, int volt);
int bq2429x_set_chargecurrent(struct bq2429x *bq, int curr);
static int bq2429x_set_appropriate_current(struct bq2429x *bq);
static void bq2429x_monitor_workfunc(struct work_struct *work);
static int bq2429x_get_prop_batt_capacity(struct bq2429x *bq);

static DEFINE_MUTEX(bq2429x_i2c_lock);

static int bq2429x_read_byte(struct bq2429x *bq, u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&bq2429x_i2c_lock);
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("failed to read 0x%.2x\n", reg);
		mutex_unlock(&bq2429x_i2c_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&bq2429x_i2c_lock);

	return 0;
}

static int bq2429x_write_byte(struct bq2429x *bq, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&bq2429x_i2c_lock);
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	mutex_unlock(&bq2429x_i2c_lock);
	return ret;
}

static int bq2429x_update_bits(struct bq2429x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bq2429x_read_byte(bq, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2429x_write_byte(bq, reg, tmp);
}

#if 0
static enum bq2429x_vbus_type bq2429x_get_vbus_type(struct bq2429x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2429x_read_byte(bq, &val, BQ2429X_REG_08);
	if (ret < 0)
		return 0;
	val &= REG08_VBUS_STAT_MASK;
	val >>= REG08_VBUS_STAT_SHIFT;

	return val;
}
#endif

static int bq2429x_enable_otg(struct bq2429x *bq)
{
	u8 val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;

	pr_info("%s", __func__);

	return bq2429x_update_bits(bq, BQ2429X_REG_01,
							   REG01_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_enable_otg);

static int bq2429x_disable_otg(struct bq2429x *bq)
{
	u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;

	pr_info("%s", __func__);

	return bq2429x_update_bits(bq, BQ2429X_REG_01,
							   REG01_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_disable_otg);


static int bq2429x_enable_charger(struct bq2429x *bq)
{
	int ret;
	u8 val = REG01_CHG_ENABLE << REG01_CHG_CONFIG_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_CHG_CONFIG_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_enable_charger);

static int bq2429x_disable_charger(struct bq2429x *bq)
{
	int ret;
	u8 val = REG01_CHG_DISABLE << REG01_CHG_CONFIG_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_CHG_CONFIG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_disable_charger);

int bq2429x_set_chargecurrent(struct bq2429x *bq, int curr)
{
	u8 ichg;

	ichg = (curr - REG02_ICHG_BASE)/REG02_ICHG_LSB;
	return bq2429x_update_bits(bq, BQ2429X_REG_02, REG02_ICHG_MASK, ichg << REG02_ICHG_SHIFT);

}

int bq2429x_set_term_current(struct bq2429x *bq, int curr)
{
	u8 iterm;

	iterm = (curr - REG03_ITERM_BASE) / REG03_ITERM_LSB;

	return bq2429x_update_bits(bq, BQ2429X_REG_03, REG03_ITERM_MASK, iterm << REG03_ITERM_SHIFT);
}


int bq2429x_set_prechg_current(struct bq2429x *bq, int curr)
{
	u8 iprechg;

	iprechg = (curr - REG03_IPRECHG_BASE) / REG03_IPRECHG_LSB;

	return bq2429x_update_bits(bq, BQ2429X_REG_03, REG03_IPRECHG_MASK, iprechg << REG03_IPRECHG_SHIFT);
}

int bq2429x_set_chargevoltage(struct bq2429x *bq, int volt)
{
	u8 val;

	val = (volt - REG04_VREG_BASE)/REG04_VREG_LSB;
	return bq2429x_update_bits(bq, BQ2429X_REG_04, REG04_VREG_MASK, val << REG04_VREG_SHIFT);
}

int bq2429x_set_rechargevoltage(struct bq2429x *bq, int volt)
{
	u8 val = 0;

	if (volt > 100) {
		val = 1;
	}

	return bq2429x_update_bits(bq, BQ2429X_REG_04, 0x1, val);
}

int bq2429x_set_input_volt_limit(struct bq2429x *bq, int volt)
{
	u8 val;

	val = (volt - REG00_VINDPM_BASE) / REG00_VINDPM_LSB;
	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_VINDPM_MASK, val << REG00_VINDPM_SHIFT);
}

int bq2429x_set_input_current_limit(struct bq2429x *bq, int curr)
{
	u8 val;

	switch (curr) {
	case BQ2429X_ILIM_100mA:
		val = REG00_IINLIM_100MA;
		break;
	case BQ2429X_ILIM_150mA:
		val = REG00_IINLIM_150MA;
		break;
	case BQ2429X_ILIM_500mA:
	case BQ2429X_ILIM_400mA:
	case BQ2429X_ILIM_300mA:
	case BQ2429X_ILIM_200mA:
		val = REG00_IINLIM_500MA;
		break;
	case BQ2429X_ILIM_900mA:
	case BQ2429X_ILIM_800mA:
	case BQ2429X_ILIM_700mA:
	case BQ2429X_ILIM_600mA:
		val = REG00_IINLIM_900MA;
		break;
	case BQ2429X_ILIM_1000mA:
		val = REG00_IINLIM_1000MA;
		break;
	case BQ2429X_ILIM_1500mA:
	case BQ2429X_ILIM_1400mA:
	case BQ2429X_ILIM_1300mA:
	case BQ2429X_ILIM_1200mA:
	case BQ2429X_ILIM_1100mA:
		val = REG00_IINLIM_1500MA;
		break;
	case BQ2429X_ILIM_2000mA:
	case BQ2429X_ILIM_1800mA:
		val = REG00_IINLIM_2000MA;
		break;
	case BQ2429X_ILIM_3000mA:
		val = REG00_IINLIM_3000MA;
		break;
	default:
		val = REG00_IINLIM_500MA;
	}

	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_IINLIM_MASK, val << REG00_IINLIM_SHIFT);
}

int bq2429x_set_watchdog_timer(struct bq2429x *bq, u8 timeout)
{
	return bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_WDT_MASK,
				(u8)((timeout - REG05_WDT_BASE) / REG05_WDT_LSB) << REG05_WDT_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2429x_set_watchdog_timer);

int bq2429x_disable_watchdog_timer(struct bq2429x *bq)
{
	u8 val = REG05_WDT_DISABLE << REG05_WDT_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2429x_disable_watchdog_timer);

int bq2429x_reset_watchdog_timer(struct bq2429x *bq)
{
	u8 val = REG01_WDT_RESET << REG01_WDT_RESET_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2429x_reset_watchdog_timer);

/*merge from svn*/
/*0 -- normal ,1 -- expired*/
static int bq2429x_get_watchdog_timer_state(struct bq2429x *bq)
{
	u8 val = 0;

	bq2429x_read_byte(bq, &val, BQ2429X_REG_09);
	val &= REG09_FAULT_WDT_MASK;
	val >>= REG09_FAULT_WDT_SHIFT;

	if (val == 1)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(bq2429x_get_watchdog_timer_state);
/*merge from svn*/

static int bq2429x_enable_safety_timer(struct bq2429x *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG05_CHG_TIMER_ENABLE << REG05_EN_TIMER_SHIFT;
	else
		val = REG05_CHG_TIMER_DISABLE << REG05_EN_TIMER_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_EN_TIMER_MASK, val);

	return ret;
}
/*merge from svn*/
EXPORT_SYMBOL_GPL(bq2429x_enable_safety_timer);

static int bq2429x_get_safety_timer_state(struct bq2429x *bq)
{
	u8 val = 0;

	bq2429x_read_byte(bq, &val, BQ2429X_REG_05);
	val &= REG05_EN_TIMER_MASK;
	val >>= REG05_EN_TIMER_SHIFT;

	if (val == 1)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(bq2429x_get_safety_timer_state);
/*merge from svn*/

int bq2429x_reset_chip(struct bq2429x *bq)
{
	int ret;

	u8 val = REG01_REG_RESET << REG01_REG_RESET_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_REG_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_reset_chip);

int bq2429x_enter_hiz_mode(struct bq2429x *bq)
{
	u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_enter_hiz_mode);

int bq2429x_exit_hiz_mode(struct bq2429x *bq)
{

	u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_exit_hiz_mode);

int bq2429x_get_hiz_mode(struct bq2429x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2429x_read_byte(bq, &val, BQ2429X_REG_00);
	if (ret)
		return ret;
	*state = (val & REG00_ENHIZ_MASK) >> REG00_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2429x_get_hiz_mode);


static int bq2429x_enable_term(struct bq2429x *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG05_TERM_ENABLE << REG05_EN_TERM_SHIFT;
	else
		val = REG05_TERM_DISABLE << REG05_EN_TERM_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_enable_term);

int bq2429x_set_boost_volt(struct bq2429x *bq, int volt)
{
	u8 val;

	val = (volt - REG06_BOOSTV_BASE) / REG06_BOOSTV_LSB;

	return bq2429x_update_bits(bq, BQ2429X_REG_06, REG06_BOOSTV_MASK, val << REG06_BOOSTV_SHIFT);
}

int bq2429x_set_boost_current(struct bq2429x *bq, int curr)
{
	u8 val;

	val = REG01_BOOST_LIM_1A;
	if (curr == BOOSTI_1500) {
		val = REG01_BOOST_LIM_1P5A;
	}

	return bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_BOOST_LIM_MASK, val << REG01_BOOST_LIM_SHIFT);
}

int bq2429x_set_batfet_disable(struct bq2429x *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG07_BATFET_OFF << REG07_BATFET_DIS_SHIFT;
	else
		val = REG07_BATFET_ON << REG07_BATFET_DIS_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_07, REG07_BATFET_DIS_MASK, val);

	return ret;
}

static void bq2429x_set_shipping_mode_enable(struct bq2429x *bq, bool enable)
{
	bq2429x_exit_hiz_mode(bq);	/*disable hiz mode */

	if (enable == 1) {
		bq2429x_disable_watchdog_timer(bq);		/*disable watchdog time.*/
		bq2429x_set_batfet_disable(bq, enable);	/*disable BATFET*/
		bq->shipping_mode_enabled  = 1;
	} else {
		bq2429x_set_watchdog_timer(bq, 40);		/*enable watchdog time.*/
		bq2429x_set_batfet_disable(bq, enable);	/*enable BATFET*/
		bq->shipping_mode_enabled  = 0;
	}
}

static int bq2429x_set_dp_dm_detection_enable(struct bq2429x *bq)
{
	int ret;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_07, REG07_FORCE_DPDM_MASK,
								REG07_FORCE_DPDM << REG07_FORCE_DPDM_SHIFT);

	return ret;
}

static int bq2429x_battery_discharging(struct bq2429x *bq)
{
	int ret;

	/*if (bq->battery_charging_policy_ops.battery_status != BATTERY_DISCHARGING) {*/
		ret = vote(bq->fcc_votable, CHARGING_POLICY_FCC_VOTER, true, 0);
		if (ret < 0)
			pr_err("%s,charging current limit: CHARGER_TYPE_FCC_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		/* ensure the vote takes  effect, so do twice different votes */
		ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, true, BQ2429X_ILIM_150mA);
		ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, true, BQ2429X_ILIM_100mA);
		if (ret < 0)
			pr_err("%s,input current limit: CHARGER_TYPE_ICL_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		bq->battery_charging_policy_ops.battery_status = BATTERY_DISCHARGING;
	/*}*/
	return 0;
}

static int bq2429x_battery_not_charging(struct bq2429x *bq)
{
	int ret;

	if (bq->battery_charging_policy_ops.battery_status != BATTERY_NOT_CHARGING) {
		ret = vote(bq->fcc_votable, CHARGING_POLICY_FCC_VOTER, true, 0);
		if (ret < 0)
			pr_err("%s,charging current limit: CHARGER_TYPE_FCC_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, false, 0);
		if (ret < 0)
			pr_err("%s,input current limit: CHARGER_TYPE_ICL_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		bq->battery_charging_policy_ops.battery_status = BATTERY_NOT_CHARGING;
	}
	return 0;
}

static int bq2429x_battery_charging(struct bq2429x *bq, bool force)
{
	int ret;

	if (bq->battery_charging_policy_ops.battery_status !=  BATTERY_CHARGING) {
		if ((!bq->batt_hot) && (!bq->batt_cold)) {
			ret = vote(bq->fcc_votable, CHARGING_POLICY_FCC_VOTER, false, 0);
			if (ret < 0)
				pr_err("%s,charging current limit: CHARGER_TYPE_FCC_VOTER couldn't vote, ret=%d\n",
					__func__, ret);

			ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, false, 0);
			if (ret < 0)
				pr_err("%s,input current limit: CHARGER_TYPE_ICL_VOTER couldn't vote, ret=%d\n",
					__func__, ret);
		}
		bq->battery_charging_policy_ops.battery_status = BATTERY_CHARGING;
	}

	if (force) {
		if ((!bq->batt_hot) && (!bq->batt_cold)) {
			ret = vote(bq->fcc_votable, CHARGING_POLICY_FCC_VOTER, false, 0);
			if (ret < 0)
				pr_err("%s,charging current limit: CHARGER_TYPE_FCC_VOTER couldn't vote, ret=%d\n",
					__func__, ret);

			ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, false, 0);
			if (ret < 0)
				pr_err("%s,input current limit: CHARGER_TYPE_ICL_VOTER couldn't vote, ret=%d\n",
					__func__, ret);
		}
		bq->battery_charging_policy_ops.battery_status = BATTERY_CHARGING;
		bq->battery_charging_policy_ops.charging_policy_status = DEFAULT_CHARGING_POLICY;
	}
	return 0;
}

static int bq2429x_battery_clean_charging_expired(struct bq2429x *bq)
{
	bq->battery_charging_policy_ops.charging_policy_status &= ~EXPIRED_CHARGING_POLICY;

	return 0;
}

static int bq2429x_battery_charging_policy_check(struct bq2429x *bq)
{
	int soc = bq2429x_get_prop_batt_capacity(bq);

	pr_info("charging_policy_status = 0x%x, battery_status = %d, soc = %d\n",
		bq->battery_charging_policy_ops.charging_policy_status,
		bq->battery_charging_policy_ops.battery_status, soc);
	if ((bq->battery_charging_policy_ops.charging_policy_status & DEMO_CHARGING_POLICY)
		|| (bq->battery_charging_policy_ops.charging_policy_status & EXPIRED_CHARGING_POLICY)) {
		if (soc > MAX_BATTERY_PROTECTED_PERCENT) {
			bq2429x_battery_discharging(bq);
		} else if (soc == MAX_BATTERY_PROTECTED_PERCENT) {
			bq2429x_battery_not_charging(bq);
		} else if (soc < MAX_BATTERY_PROTECTED_PERCENT) {
			bq2429x_battery_charging(bq, false);
		}
	} else {
		if (bq->prop_set_charge_enabled)
			bq2429x_battery_charging(bq, true);
	}

	return 0;
}

static void bq2429x_charging_policy_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq2429x *bq = container_of(dwork, struct bq2429x, charging_policy_work);

	pr_info("\n");
	bq2429x_battery_charging_policy_check(bq);
	power_supply_changed(&bq->batt_psy);
}

static enum alarmtimer_restart bq2429x_charging_expired_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
#ifdef CONFIG_LONG_CHARGING_PROTECT_POLICY
	struct bq2429x *bq = container_of(alarm, struct bq2429x,
					charging_expired_alarm);

	pr_info("\n");

	bq->battery_charging_policy_ops.charging_policy_status &= ~NORMAL_CHARGING_POLICY;
	bq->battery_charging_policy_ops.charging_policy_status |= EXPIRED_CHARGING_POLICY;
	cancel_delayed_work_sync(&bq->charging_policy_work);
	schedule_delayed_work(&bq->charging_policy_work, round_jiffies_relative(msecs_to_jiffies(1000)));
#endif

	return ALARMTIMER_NORESTART;
}

static int bc_policy_demo_sts_set(struct charging_policy_ops *charging_policy, bool enable)
{
	struct bq2429x *bq = container_of(charging_policy, struct bq2429x,
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
	schedule_delayed_work(&bq->charging_policy_work, round_jiffies_relative(msecs_to_jiffies(1000)));

	return 0;
}

static int bc_policy_demo_sts_get(struct charging_policy_ops *charging_policy)
{
	struct bq2429x *bq = container_of(charging_policy, struct bq2429x,
					battery_charging_policy_ops);

	pr_info("charging_policy_status = 0x%x\n", bq->battery_charging_policy_ops.charging_policy_status);

	if ((bq->battery_charging_policy_ops.charging_policy_status & DEMO_CHARGING_POLICY) == DEMO_CHARGING_POLICY)
		return 1;
	else
		return 0;
}

static int bc_policy_expired_sts_get(struct charging_policy_ops *charging_policy)
{
	struct bq2429x *bq = container_of(charging_policy, struct bq2429x,
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
	pr_info(" %d sec\n", sec);

	g_charging_expiration_ns = (u64)sec * 100000000;

	return 0;
}

static int bc_policy_expired_sec_get(struct charging_policy_ops *charging_policy)
{
	pr_info("\n");

	return 0;
}

static int charging_policy_init(struct bq2429x *bq)
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

static int bq2429x_get_prop_charge_type(struct bq2429x *bq)
{
	u8 val = 0;

	bq2429x_read_byte(bq, &val, BQ2429X_REG_08);
	val &= REG08_CHRG_STAT_MASK;
	val >>= REG08_CHRG_STAT_SHIFT;

	switch (val) {
	case CHARGE_STATE_FASTCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHARGE_STATE_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHARGE_STATE_CHGDONE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	case CHARGE_STATE_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}
}

static struct power_supply *get_bms_psy(struct bq2429x *bq)
{
	if (bq->bms_psy)
		return bq->bms_psy;
	bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->bms_psy)
		pr_debug("bms power supply not found\n");

	return bq->bms_psy;
}

static int bq2429x_get_batt_property(struct bq2429x *bq,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct power_supply *bms_psy = get_bms_psy(bq);

	int ret;

	if (!bms_psy)
		return -EINVAL;

	ret = bms_psy->get_property(bms_psy, psp, val);

	return ret;
}

static int bq2429x_get_prop_charge_status(struct bq2429x *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;
	u8 status;

	ret = bq2429x_get_batt_property(bq, POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (bq->chg_term_by_soc && ((bq->chg_term_battery_status == POWER_SUPPLY_STATUS_FULL)
		|| (!ret && batt_prop.intval == POWER_SUPPLY_STATUS_FULL))) {
		return POWER_SUPPLY_STATUS_FULL;
	}

	ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_08);
	if (ret) {
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	mutex_lock(&bq->lock);
	bq->charge_state = (status & REG08_CHRG_STAT_MASK) >> REG08_CHRG_STAT_SHIFT;
	mutex_unlock(&bq->lock);

	switch (bq->charge_state) {
	case CHARGE_STATE_FASTCHG:
	case CHARGE_STATE_PRECHG:
		return POWER_SUPPLY_STATUS_CHARGING;
	case CHARGE_STATE_CHGDONE:
		return POWER_SUPPLY_STATUS_FULL;
	case CHARGE_STATE_IDLE:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

static int bq2429x_get_prop_charger_health(struct bq2429x *bq)
{
#if 0
	u8 ntc_fault;
	int ret;

	mutex_lock(&bq->lock);
	ntc_fault =
		(bq->fault_status & REG09_FAULT_NTC_MASK) >> REG09_FAULT_NTC_SHIFT;
	mutex_unlock(&bq->lock);
	if (ntc_fault == REG09_FAULT_NTC_HOT)
		ret = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (ntc_fault == REG09_FAULT_NTC_COLD)
		ret = POWER_SUPPLY_HEALTH_COLD;
	else
		ret = POWER_SUPPLY_HEALTH_GOOD;
	return ret;
#else
	union power_supply_propval ret = {0, };

	if (bq->batt_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (bq->batt_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if (bq->batt_warm)
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	else if (bq->batt_cool)
		ret.intval = POWER_SUPPLY_HEALTH_COOL;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

#ifdef CONFIG_BQ2XXXX_OVER_VOLTAGE_REPORT
	if (bq->psy_health_sts == POWER_SUPPLY_HEALTH_OVERVOLTAGE)
		ret.intval = bq->psy_health_sts;

#endif

	return ret.intval;
#endif
}

static int get_property_from_fg(struct bq2429x *bq,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!bq->bms_psy && bq->bms_psy_name)
		bq->bms_psy =
			power_supply_get_by_name((char *)bq->bms_psy_name);
	if (!bq->bms_psy) {
		pr_info("no bms psy found\n");
		return -EINVAL;
	}

	rc = bq->bms_psy->get_property(bq->bms_psy, prop, &ret);
	if (rc) {
		pr_info("bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#define DEFAULT_BATT_CHARGE_COUNTER 1
static int bq2429x_get_prop_batt_charge_counter(struct bq2429x *bq)
{
	int counter, rc;

	rc = get_property_from_fg(bq, POWER_SUPPLY_PROP_CHARGE_COUNTER, &counter);
	if (rc || counter == 0) {
		pr_err("Couldn't get charge counter rc = %d\n", rc);
		counter = DEFAULT_BATT_CHARGE_COUNTER;
	}
	return counter;
}

#define DEFAULT_BATT_CURRENT_NOW	0
#define DEFAULT_BATT_VOLTAGE_NOW	0
static int bq2429x_get_prop_batt_current_now(struct bq2429x *bq)
{
	int ua, rc;

	rc = get_property_from_fg(bq, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_err("Couldn't get current rc = %d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}
	return ua;
}

#define DEFAULT_TEMP 250
static int bq2429x_get_prop_batt_temp(struct bq2429x *bq)
{
	int temp, rc;

	rc = get_property_from_fg(bq, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_err("Couldn't get battery temp rc = %d\n", rc);
		temp = DEFAULT_TEMP;
	}
	if (bq->use_test_temp) {
		dev_notice(bq->dev, "%s use  temprature: %d\n", __func__, bq->test_temp);
		temp = bq->test_temp;
	}
	return temp;
}
#define DEFAULT_CAPACITY 50
static int bq2429x_get_prop_batt_capacity(struct bq2429x *bq)
{
	int cap, rc;

	rc = get_property_from_fg(bq, POWER_SUPPLY_PROP_CAPACITY, &cap);
	if (rc) {
		pr_err("Couldn't get battery capacity rc = %d\n", rc);
		cap = DEFAULT_CAPACITY;
	}
	return cap;
}

static int bq2429x_get_prop_battery_voltage_now(struct bq2429x *bq)
{
	int uv, rc;

	rc = get_property_from_fg(bq, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_err("Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}
	if (bq->use_test_batt_volt) {
		dev_notice(bq->dev, "%s use  temprature: %d\n", __func__, bq->test_batt_volt);
		uv = bq->test_batt_volt;
	}

	return uv;
}

static int bq2429x_get_prop_batt_status(struct bq2429x *bq)
{
	u8 val = 0;

	bq2429x_read_byte(bq, &val, BQ2429X_REG_08);
	val &= REG08_CHRG_STAT_MASK;
	val >>= REG08_CHRG_STAT_SHIFT;

	switch (val) {
	case CHARGE_STATE_FASTCHG:
	case CHARGE_STATE_PRECHG:
		return POWER_SUPPLY_STATUS_CHARGING;
	case CHARGE_STATE_CHGDONE:
		if (bq2429x_get_prop_charger_health(bq) == POWER_SUPPLY_HEALTH_GOOD)
			return POWER_SUPPLY_STATUS_FULL;
		else
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case CHARGE_STATE_IDLE:
		if (!bq->chg_present)
			return POWER_SUPPLY_STATUS_DISCHARGING;
		else
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static enum power_supply_property bq2429x_charger_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE, /* Charger status output */
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY, /* for debug */
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_SHIPPING_MODE,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};
static int bq2429x_system_temp_level_set(struct bq2429x *bq,
								int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;

	if (!bq->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}
	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= bq->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n",
			lvl_sel,	bq->thermal_levels - 1);
		lvl_sel = bq->thermal_levels - 1;
	}

	if (lvl_sel == bq->therm_lvl_sel)
		return 0;

	mutex_lock(&bq->current_change_lock);
	prev_therm_lvl = bq->therm_lvl_sel;
	bq->therm_lvl_sel = lvl_sel;
	mutex_unlock(&bq->current_change_lock);

	pr_notice("system temp level:%d set current:%d ma.\n",
		lvl_sel, bq->thermal_mitigation[bq->therm_lvl_sel]);

	rc = vote(bq->usb_icl_votable, PA_THERMAL_ICL_VOTER, true, bq->thermal_mitigation[bq->therm_lvl_sel]);
	if (rc < 0)
		pr_err("%s,input current limit:PA_THERMAL_ICL_VOTER couldn't vote, ret=%d\n", __func__, rc);

	return rc;
}

static int bq2429x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{

	struct bq2429x *bq = container_of(psy, struct bq2429x, batt_psy);
	int type;

	mutex_lock(&bq->lock);
	type = bq->vbus_type;
	mutex_unlock(&bq->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2429x_get_prop_batt_status(bq);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (type == BQ2429X_VBUS_ADAPTER || type == BQ2429X_VBUS_USB)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2429x_get_prop_charge_type(bq);
		pr_debug("POWER_SUPPLY_PROP_CHARGE_TYPE:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq2429x_get_prop_charger_health(bq);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		#ifdef CHG_SMOOTH_BATTERY_PROP
				val->intval = g_smooth_prop.ui_soc;
		#else
				val->intval = bq2429x_get_prop_batt_capacity(bq);
		#endif
		pr_debug("POWER_SUPPLY_PROP_CAPACITY:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		#ifdef CHG_SMOOTH_BATTERY_PROP
			val->intval = g_smooth_prop.avg_temp;
		#else
			val->intval = bq2429x_get_prop_batt_temp(bq);
		#endif
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "BQ2429x";
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CHG_SMOOTH_BATTERY_PROP
		val->intval = g_smooth_prop.avg_volt;
#else
		val->intval = bq2429x_get_prop_battery_voltage_now(bq);
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq2429x_get_prop_batt_current_now(bq);
		break;
	case POWER_SUPPLY_PROP_SHIPPING_MODE:
		val->intval = bq->shipping_mode_enabled;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = bq->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->prop_set_charge_enabled;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = bq->usb_dp_dm_status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = bq2429x_get_prop_batt_charge_counter(bq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int bq2429x_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq2429x *bq = container_of(psy,
				struct bq2429x, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if (val->intval)
			bq2429x_enable_charger(bq);
		else
			bq2429x_disable_charger(bq);
		mutex_lock(&bq->lock);
		bq->prop_set_charge_enabled = !!val->intval;
		mutex_unlock(&bq->lock);
		if (val->intval == 0) {
			if (bq->battery_suspend_lock.ws.active == true) {
				wake_unlock(&bq->battery_suspend_lock);
			}
			bq2429x_disable_watchdog_timer(bq);
		}
		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		bq->usb_dp_dm_status = val->intval;
		switch (val->intval) {
		case POWER_SUPPLY_DP_DM_DPF_DMF:
			power_supply_set_online(bq->usb_psy, 1);
			power_supply_set_present(bq->usb_psy, 1);
			break;
		case POWER_SUPPLY_DP_DM_UNKNOWN:
		case POWER_SUPPLY_DP_DM_PREPARE:
		case POWER_SUPPLY_DP_DM_DPR_DMR:
			pr_notice("bq2429x %s usb offline\n", __func__);
			if (bq->battery_suspend_lock.ws.active == true) {
				wake_unlock(&bq->battery_suspend_lock);
			}
			power_supply_set_online(bq->usb_psy, 0);
			power_supply_set_present(bq->usb_psy, 0);
			break;
		default:
			break;
		}
	case POWER_SUPPLY_PROP_CAPACITY:
		bq->fake_rsoc = val->intval;
		pr_info("fake_soc set to %d\n", bq->fake_rsoc);
		power_supply_changed(&bq->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SHIPPING_MODE:
		bq2429x_set_shipping_mode_enable(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		bq2429x_system_temp_level_set(bq, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int bq2429x_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SHIPPING_MODE:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_DP_DM:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int bq2429x_set_appropriate_current(struct bq2429x *bq)
{
	int ret = 0;
	union power_supply_propval prop = {0,};

	ret = bq->usb_psy->get_property(bq->usb_psy, POWER_SUPPLY_PROP_TYPE, &prop);

	if (ret < 0)
		pr_err("couldn't read USB TYPE property, ret=%d\n", ret);
	else
		pr_info("USB TYPE detected is:%d\n", prop.intval);

	bq->chg_type = prop.intval;

	switch (prop.intval) {
	case POWER_SUPPLY_TYPE_USB_DCP:
		ret = vote(bq->fcc_votable, CHARGER_TYPE_FCC_VOTER, true, bq->platform_data->ta.ichg);
		if (ret < 0)
			pr_err("%s,charging current limit: CHARGER_TYPE_FCC_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		ret = vote(bq->usb_icl_votable, CHARGER_TYPE_ICL_VOTER, true, bq->platform_data->ta.ilim);
		if (ret < 0)
			pr_err("%s,input current limit: CHARGER_TYPE_ICL_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		/* wa: bq24297 will auto set icl as 3A if charger type is dcp */
		cancel_delayed_work_sync(&bq->dcp_icl_recheck_work);
		schedule_delayed_work(&bq->dcp_icl_recheck_work, msecs_to_jiffies(5000));

		ret = bq2429x_set_input_volt_limit(bq, bq->platform_data->ta.vlim);
		if (ret < 0)
			pr_err("couldn't set input voltage limit, ret=%d\n", ret);

		break;
	case POWER_SUPPLY_TYPE_USB:
	default:
		ret = bq2429x_set_input_volt_limit(bq, bq->platform_data->usb.vlim);
		if (ret < 0)
			pr_err("couldn't set input voltage limit, ret=%d\n", ret);

		ret = vote(bq->fcc_votable, CHARGER_TYPE_FCC_VOTER, true, bq->platform_data->usb.ichg);
		if (ret < 0)
			pr_err("%s,charging current limit:CHARGER_TYPE_FCC_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		ret = vote(bq->usb_icl_votable, CHARGER_TYPE_ICL_VOTER, true, bq->platform_data->usb.ilim);
		if (ret < 0)
			pr_err("%s,input current limit:CHARGER_TYPE_ICL_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		break;
	}
	return ret;
}


static void bq2429x_external_power_changed(struct power_supply *psy)
{
	struct bq2429x *bq = container_of(psy, struct bq2429x, batt_psy);

	union power_supply_propval prop = {0,};
	int ret, current_limit = 0;

	cancel_delayed_work_sync(&bq->monitor_work);
	bq2429x_monitor_workfunc(&bq->monitor_work.work);

	ret = bq->usb_psy->get_property(bq->usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
		pr_err("could not read USB current_max property, ret=%d\n", ret);
	else
		current_limit = prop.intval / 1000;

	pr_info("current_limit = %d\n", current_limit);

	if (bq->need_recheck_usb_type && current_limit != 0 && current_limit != 501) {
		cancel_delayed_work_sync(&bq->usb_type_detection_work);
		bq->need_recheck_usb_type = false;
		pr_info("cancel usb_type_detection_work.\n");
	}

	if (current_limit == 501) {
		cancel_delayed_work_sync(&bq->usb_type_detection_work);
		schedule_delayed_work(&bq->usb_type_detection_work, msecs_to_jiffies(3000));
		bq->need_recheck_usb_type = true;
		pr_info("usb_type_detection_work in 3s.\n");
	}

	if (bq->usb_psy_ma != current_limit) {
		mutex_lock(&bq->current_change_lock);
		bq->usb_psy_ma = current_limit;
		ret = bq2429x_set_appropriate_current(bq);
		if (ret < 0)
			pr_err("couldn't set usb current, ret=%d\n", ret);
		mutex_unlock(&bq->current_change_lock);
	}

	ret = bq->usb_psy->get_property(bq->usb_psy, POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		pr_err("could not read USB ONLINE property, ret=%d]n", ret);

	ret = 0;
	bq2429x_get_prop_charge_status(bq);
	if (bq->chg_present) {
		if (prop.intval == 0) {
			pr_err("set usb online\n");
			ret = power_supply_set_online(bq->usb_psy, true);
		}
	} else {
		if (prop.intval == 1) {
			pr_err("set usb offline\n");
			ret = power_supply_set_online(bq->usb_psy, false);
		}
	}

	if (ret < 0)
		pr_info("could not set usb online state, ret=%d\n", ret);

}

static int bq2429x_psy_register(struct bq2429x *bq)
{
	int ret;

	bq->batt_psy.name = "battery";
	bq->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->batt_psy.properties = bq2429x_charger_props;
	bq->batt_psy.num_properties = ARRAY_SIZE(bq2429x_charger_props);
	bq->batt_psy.get_property = bq2429x_charger_get_property;
	bq->batt_psy.set_property = bq2429x_charger_set_property;
	bq->batt_psy.external_power_changed = bq2429x_external_power_changed;
	bq->batt_psy.property_is_writeable = bq2429x_charger_is_writeable;

	ret = power_supply_register(bq->dev, &bq->batt_psy);
	if (ret < 0) {
		pr_err("failed to register batt_psy:%d\n", ret);
		return ret;
	}

	return 0;

}

static void bq2429x_psy_unregister(struct bq2429x *bq)
{
	power_supply_unregister(&bq->batt_psy);
}


static int bq2429x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2429x *bq = rdev_get_drvdata(rdev);

	ret = bq2429x_enable_otg(bq);
	if (ret) {
		pr_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq->otg_enabled = true;
		pr_info("bq2429x OTG mode Enabled!\n");
	}

	return ret;
}

static int bq2429x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2429x *bq = rdev_get_drvdata(rdev);

	ret = bq2429x_disable_otg(bq);
	if (ret) {
		pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		bq->otg_enabled = false;
		pr_info("bq2429x OTG mode Disabled\n");
	}

	return ret;
}


static int bq2429x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret;
	u8 status;
	u8 enabled;
	struct bq2429x *bq = rdev_get_drvdata(rdev);

	ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_01);
	if (ret)
		return ret;
	enabled = ((status & REG01_OTG_CONFIG_MASK) >> REG01_OTG_CONFIG_SHIFT);

	return (enabled == REG01_OTG_ENABLE) ? 1 : 0;

}


struct regulator_ops bq2429x_otg_reg_ops = {
	.enable		= bq2429x_otg_regulator_enable,
	.disable	= bq2429x_otg_regulator_disable,
	.is_enabled = bq2429x_otg_regulator_is_enable,
};

static int bq2429x_regulator_init_chip(struct bq2429x *bq)
{
	int rc = 0;
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *regulator_node;

	pr_info("begin.\n");

	regulator_node = of_find_node_by_name(NULL,
			"qcom,ti-boost-otg");

	init_data = of_get_regulator_init_data(bq->dev, regulator_node);
	if (!init_data) {
		dev_err(bq->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		bq->otg_vreg.rdesc.owner = THIS_MODULE;
		bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		bq->otg_vreg.rdesc.ops = &bq2429x_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		pr_info("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = regulator_node; /*bq->dev->of_node;*/

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

	pr_info("return :%d.\n", rc);
	return rc;
}

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *smb_chg_proc_entry;

static ssize_t smb_chg_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	struct bq2429x *bq = PDE_DATA(file_inode(filp));
	char message[256];
	int reg, val;
	int copy_len = len;

	if (copy_len > 256)
		copy_len = 255;

	if (copy_from_user(message, buff, copy_len))
		return -EFAULT;
	message[copy_len] = '\0';

	pr_notice("bq24298 %s\n", message);
	if (strnstr(message, "write", sizeof("write"))) {
		if (sscanf(&message[6], "%x %x", &reg, &val) != 1) {
			pr_info("sscanf() != 1\n");
		}
		bq2429x_write_byte(bq, reg, val);
		return len;
	} else if (strnstr(message, "temp", sizeof("temp"))) {
		if (kstrtoint(&message[5], 10, &val) < 0) {
			pr_info("kstrtoint failed!\n");
		}
		pr_notice("temp = %d\n", val);
		if (val < 990) {
			bq->use_test_temp = true;
			bq->test_temp = val;
		} else {
			bq->use_test_temp = false;
		}
		return len;
	} else if (strnstr(message, "volt", sizeof("volt"))) {
		if (kstrtoint(&message[5], 10, &val) < 0) {
			pr_info("kstrtoint failed!\n");
		}
		pr_notice("battery volt = %d\n", val);
		if (val < 4900000) {
			bq->use_test_batt_volt = true;
			bq->test_batt_volt = val;
		} else {
			bq->use_test_batt_volt = false;
		}
		return len;
	} else if (strnstr(message, "disable feed wdt", sizeof("disable feed wdt"))) {
		bq->test_disable_feed_wdt = true;
		return len;
	} else if (strnstr(message, "enable feed wdt", sizeof("enable feed wdt"))) {
		bq->test_disable_feed_wdt = false;
		return len;
	} else if (strnstr(message, "health", sizeof("health"))) {
		if (kstrtoint(&message[7], 10, &val) < 0) {
			pr_info("kstrtoint failed!\n");
		}
		pr_notice("health = %d\n", val);
		bq->psy_health_sts = val;
		return len;
	}

	return len;
}

static int smb_chg_proc_show(struct seq_file *seq, void *v)
{
	struct bq2429x *bq = seq->private;
	u8 status[11];
	int i = 0;
	int ret = -1;

	pr_notice("bq24297 %s\n", __func__);

	for (i = 0; i <= 0xA; i++) {
		bq2429x_read_byte(bq, &status[i], i);
	}

	ret = seq_puts(seq, "ti chg_20180305\n");
	ret |= seq_printf(seq, "[bq24298]\n0x0=0x%02x\n0x1=0x%02x\n0x2=0x%02x\n0x3=0x%02x\n",
					status[0], status[1], status[2], status[3]);
	ret |= seq_printf(seq, "0x4=0x%02x\n0x5=0x%02x\n0x6=0x%02x\n0x7=0x%02x\n",
					status[4], status[5], status[6], status[7]);
	ret |= seq_printf(seq, "0x8=0x%02x\n 0x9=0x%02x\n0xA=0x%02x\n",
					status[8], status[9], status[10]);

	if (ret < 0) {
		pr_notice("bq2429x %s seq write error.\n", __func__);
	}
	return 0;
}

static int smb_chg_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb_chg_proc_show, PDE_DATA(inode));
}

static const struct file_operations smb_chg_proc_ops = {
	.owner		= THIS_MODULE,
	.open		= smb_chg_seq_open,
	.read		= seq_read,
	.write		= smb_chg_proc_write,
	.llseek		= seq_lseek,
	/*.release	= seq_release_ops,*/
};

static void create_smb_chg_proc_entry(struct bq2429x *data)
{
	smb_chg_proc_entry = proc_create_data("driver/ti_regs", 0644, NULL, &smb_chg_proc_ops, data);
	if (smb_chg_proc_entry) {
		pr_info("create proc file success!\n");
	} else
		pr_info("create proc file failed!\n");
}

/*
static void remove_tpd_debug_proc_entry(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(TOUCH_PROC_FILE, &proc_root);
}
*/
#endif


static int bq2429x_regulator_init(struct bq2429x *bq)
{
	return bq2429x_regulator_init_chip(bq);
#if 0
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
		bq->otg_vreg.rdesc.ops = &bq2429x_otg_reg_ops;
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
#endif
}

static int bq2429x_parse_jeita_dt(struct device *dev, struct bq2429x *bq)
{
	struct device_node *np = dev->of_node;
	int ret;

	ret = of_property_read_u32(np, "ti,bq2429x,jeita-hot-degc", &bq->batt_hot_degc);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-hot-degc\n");
		bq->batt_hot_degc = -EINVAL;
	}

	ret = of_property_read_u32(np, "ti,bq2429x,jeita-warm-degc", &bq->batt_warm_degc);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-warm-degc\n");
		bq->batt_warm_degc = -EINVAL;
	}

	ret = of_property_read_u32(np, "ti,bq2429x,jeita-cool-degc", &bq->batt_cool_degc);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-cool-degc\n");
		bq->batt_cool_degc = -EINVAL;
	}
	ret = of_property_read_u32(np, "ti,bq2429x,jeita-cold-degc", &bq->batt_cold_degc);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-cold-degc\n");
		bq->batt_cold_degc = -EINVAL;
	}

	ret = of_property_read_u32(np, "ti,bq2429x,jeita-hot-hysteresis", &bq->hot_temp_hysteresis);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-hot-hysteresis\n");
		bq->hot_temp_hysteresis = -EINVAL;
	}

	ret = of_property_read_u32(np, "ti,bq2429x,jeita-cold-hysteresis", &bq->cold_temp_hysteresis);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-cold-hysteresis\n");
		bq->cold_temp_hysteresis = -EINVAL;
	}

	ret = of_property_read_u32(np, "ti,bq2429x,jeita-cool-ma", &bq->batt_cool_ma);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-cool-ma\n");
		bq->batt_cool_ma = -EINVAL;
	}

	ret = of_property_read_u32(np, "ti,bq2429x,jeita-cool-mv", &bq->batt_cool_mv);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-cool-mv\n");
		bq->batt_cool_mv = -EINVAL;
	}

	ret = of_property_read_u32(np, "ti,bq2429x,jeita-warm-ma", &bq->batt_warm_ma);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-warm-ma\n");
		bq->batt_warm_ma = -EINVAL;
	}

	ret = of_property_read_u32(np, "ti,bq2429x,jeita-warm-mv", &bq->batt_warm_mv);
	if (ret) {
		pr_err("Failed to read ti,bq2429x,jeita-warm-mv\n");
		bq->batt_warm_mv = -EINVAL;
	}

	bq->software_jeita_supported = of_property_read_bool(np, "ti,bq2429x,software-jeita-supported");

	return 0;
}

static struct bq2429x_platform_data *bq2429x_parse_dt(struct device *dev, struct bq2429x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	struct bq2429x_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct bq2429x_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_err("Out of memory\n");
		return NULL;
	}

	ret = of_property_read_u32(np, "ti,bq2429x,usb-vlim", &pdata->usb.vlim);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,usb-vlim\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,usb-ilim", &pdata->usb.ilim);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,usb-ilim\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,usb-float-voltage-mv", &pdata->usb.float_voltage_mv);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,usb-float-voltage-mv\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,usb-ichg", &pdata->usb.ichg);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,usb-ichg\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,ta-vlim", &pdata->ta.vlim);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,ta-vlim\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,ta-ilim", &pdata->ta.ilim);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,ta-ilim\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,ta-float-voltage-mv", &pdata->ta.float_voltage_mv);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,ta-float-voltage-mv\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,ta-ichg", &pdata->ta.ichg);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,ta-ichg\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,precharge-current", &pdata->iprechg);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,precharge-current\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,termination-current", &pdata->iterm);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,termination-current\n");
	}

	ret = of_property_read_u32(np, "ti,recharge-mv", &pdata->recharge_voltage_mv);
	if (ret) {
		pr_err("Failed to read node of ti,recharge-mv\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,boost-volt", &pdata->boostv);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,boost-volt\n");
	}

	ret = of_property_read_u32(np, "ti,bq2429x,boost-current", &pdata->boosti);
	if (ret) {
		pr_err("Failed to read node of ti,bq2429x,boost-current\n");
	}

	if (of_find_property(np, "ti,bq2429x,thermal-mitigation", &bq->thermal_levels)) {
		bq->thermal_mitigation = devm_kzalloc(bq->dev,
			bq->thermal_levels,
			GFP_KERNEL);
		if (bq->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			devm_kfree(dev, pdata);
			return NULL;
		}
		bq->thermal_levels /= sizeof(int);
		ret = of_property_read_u32_array(np,
				"ti,bq2429x,thermal-mitigation",
				bq->thermal_mitigation, bq->thermal_levels);
		if (ret) {
			pr_err("Couldn't read threm limits ret = %d\n", ret);
			devm_kfree(dev, pdata);
			devm_kfree(dev, bq->thermal_mitigation);
			return NULL;
		}
	}
	pdata->enable_term = of_property_read_bool(np, "ti,bq2429x,enable_term");
	if (pdata->enable_term == false) {
		pr_notice("charge tem by soc:%d\n", bq->chg_term_by_soc);
		bq->chg_term_by_soc = true;
	}

	/*pdata->otg_enable_gpio = of_get_named_gpio_flags(np, "ti,otg-enable-gpio", 0, NULL);*/
	pdata->otg_irq_gpio  = of_get_named_gpio_flags(np, "ti,irq-gpio", 0, NULL);

	pr_notice("irq gpio:%d\n", pdata->otg_irq_gpio);

	/*pr_notice("otg_enable_gpio:%d\n", pdata->otg_enable_gpio);*/
	return pdata;
}


static void bq2429x_init_jeita(struct bq2429x *bq)
{

	bq->batt_temp = -EINVAL;

	/* set default value in case of dts read fail */
	bq->batt_hot_degc = 600;
	bq->batt_warm_degc = 450;
	bq->batt_cool_degc = 150;
	bq->batt_cold_degc = 0;

	bq->hot_temp_hysteresis = 20;
	bq->cold_temp_hysteresis = 20;

	bq->batt_cool_ma = 900;
	bq->batt_cool_mv = 4400;
	bq->batt_warm_ma = 900;
	bq->batt_warm_mv = 4100;

	bq->software_jeita_supported = true;

	/* DTS setting will overwrite above default value */

	bq2429x_parse_jeita_dt(&bq->client->dev, bq);
}


static int bq2429x_init_device(struct bq2429x *bq)
{
	int ret;

	pr_notice("begin.\n");

	if (bq->chg_present == false) {
		ret = bq2429x_disable_watchdog_timer(bq);
		if (ret)
			pr_err("Failed to disable wdt, ret = %d\n", ret);
	} else {
		ret = bq2429x_set_watchdog_timer(bq, 80);
		if (ret)
			pr_err("Failed to set wdt, ret = %d\n", ret);
	}

	ret = bq2429x_enable_safety_timer(bq, false);
	if (ret)
		pr_err("Failed to set safety timer, ret = %d\n", ret);

	ret = bq2429x_set_chargevoltage(bq, bq->platform_data->usb.float_voltage_mv);
	if (ret)
		pr_err("Failed to set charge voltage, ret = %d\n", ret);

	ret = bq2429x_set_rechargevoltage(bq, bq->platform_data->recharge_voltage_mv);
	if (ret)
		pr_err("Failed to set prechg current, ret = %d\n", ret);

	ret = bq2429x_set_prechg_current(bq, bq->platform_data->iprechg);
	if (ret)
		pr_err("Failed to set prechg current, ret = %d\n", ret);

	ret = bq2429x_set_term_current(bq, bq->platform_data->iterm);
	if (ret)
		pr_err("Failed to set termination current, ret = %d\n", ret);

	ret = bq2429x_set_boost_volt(bq, bq->platform_data->boostv);
	if (ret)
		pr_err("Failed to set boost voltage, ret = %d\n", ret);

	ret = bq2429x_set_boost_current(bq, bq->platform_data->boosti);
	if (ret)
		pr_err("Failed to set boost current, ret = %d\n", ret);

	ret = bq2429x_set_input_volt_limit(bq, bq->platform_data->usb.vlim);
	if (ret < 0)
		pr_err("couldn't set input voltage limit, ret=%d\n", ret);

	bq2429x_enable_term(bq, bq->platform_data->enable_term);

	bq2429x_update_bits(bq, BQ2429X_REG_07, 0x3, 0x3);

	ret = bq2429x_enable_charger(bq);
	if (ret) {
		pr_err("Failed to enable charger, ret = %d\n", ret);
	} else {
		bq->charge_enabled = true;
		pr_info("Charger Enabled Successfully!");
	}

	bq2429x_set_dp_dm_detection_enable(bq);

	return 0;
}

static int bq2429x_detect_device(struct bq2429x *bq)
{
	int ret;
	u8 data;
	int part_no = 0;
	int revision = 0;
	int try_times = 0;

	for (try_times = 0; try_times < 2; try_times++) {
		ret = bq2429x_read_byte(bq, &data, BQ2429X_REG_0A);
		if (ret == 0) {
			part_no = (data & REG0A_PN_MASK) >> REG0A_PN_SHIFT;
			revision = (data & REG0A_DEV_REV_MASK) >> REG0A_DEV_REV_SHIFT;
		}
		pr_notice("read %d times: data = 0x%x, part_no = %d, revision =%d\n",
					try_times, data, part_no, revision);
		if (part_no != 1) {
			pr_err(" failed part_no = %d\n!", part_no);
			ret = -ENODEV;
			goto out;
		} else {
			msleep(20);
		}
	}

	/*ret = bq2429x_read_byte(bq, &data, BQ2560X_REG_0B);
	if (ret == 0) {
		part_no = (data & REG0B_PN_MASK) >> REG0B_PN_SHIFT;
		revision = (data & REG0B_DEV_REV_MASK) >> REG0B_DEV_REV_SHIFT;
	}
	if (part_no == 2) {
		pr_err(" failed part_no = %d\n!", part_no);
		ret = -ENODEV;
		goto out;
	}*/

out:
	pr_notice(" ret = %d, part_no = %d, revision =%d\n", ret, part_no, revision);
	bq->part_no = part_no;
	bq->revision = revision;

	return 0; /*ret;*/
}

static void bq2429x_dump_regs(struct bq2429x *bq)
{
	u8 status[11];
	int i = 0;

	for (i = 0; i <= 0xA; i++) {
		if (i == 0x9)
			continue;
		bq2429x_read_byte(bq, &status[i], i);
	}
	pr_info("0x0=0x%02x,0x1=0x%02x,0x2=0x%02x,0x3=0x%02x,"
			"0x4=0x%02x,0x5=0x%02x,0x6=0x%02x,0x7=0x%02x,"
			"0x8=0x%02x,0x9=0x%02x,0xA=0x%02x\n",
			status[0], status[1], status[2], status[3],
			status[4], status[5], status[6], status[7],
			status[8], status[9], status[10]);
}

/*static void bq2429x_jeita_workfunc(struct work_struct *work)*/
static void bq2429x_battery_temp_work(struct bq2429x *bq, int temp)
{
/*
	struct bq2429x *bq = container_of(work,
						struct bq2429x, jeita_work.work);
*/
	int ret = 0;

	bq->batt_temp = temp;   /*bq2429x_get_prop_batt_temp(bq);*/
	if (bq->batt_temp == -EINVAL) {
		/*schedule_delayed_work(&bq->jeita_work, 2 * HZ);*/
		return;
	}
	dev_dbg(bq->dev, "batt_temp= %d\n", bq->batt_temp);
	dev_dbg(bq->dev, "batt_hot_degc =%d,batt_warm_degc=%d,batt_cool_degc=%d,batt_cold_degc=%d\n",
		bq->batt_hot_degc, bq->batt_warm_degc, bq->batt_cool_degc, bq->batt_cold_degc);

	dev_dbg(bq->dev, "batt_cool_mv=%d,batt_warm_mv=%d,batt_cool_ma=%d,batt_warm_ma=%d\n",
			bq->batt_cool_mv, bq->batt_warm_mv, bq->batt_cool_ma, bq->batt_warm_ma);
	if (bq->batt_temp >= bq->batt_hot_degc) {/* HOT:batt_temp> 60 */
		if (!bq->batt_hot) {
			bq->batt_hot  = true;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp >= bq->batt_warm_degc) {/* WARM :batt_temp>=45*/
		if (!bq->batt_hot ||
				(bq->batt_temp < bq->batt_hot_degc - bq->hot_temp_hysteresis)) {
			bq->batt_hot  = false;
			bq->batt_warm = true;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_mv = bq->batt_warm_mv;
			bq->jeita_ma = bq->batt_warm_ma;
		}
	} else if (bq->batt_temp < bq->batt_cold_degc) {/* COLD: batt_temp < 0 */
		if (!bq->batt_cold) {
			bq->batt_hot  = false;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = true;
			bq->jeita_ma = 0;
			bq->jeita_mv = 0;
		}
	} else if (bq->batt_temp < bq->batt_cool_degc) {/* COOL:batt_temp < 15 */
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
		/*bq2429x_set_appropriate_current(bq);*/
	}

	bq->jeita_active = bq->batt_cool || bq->batt_hot ||
					   bq->batt_cold || bq->batt_warm;

	if (bq->jeita_active) {
		pr_notice("jeita_active= %d,jeita_mv =%d,jeita_ma=%d\n", bq->jeita_active, bq->jeita_mv, bq->jeita_ma);
		bq2429x_set_chargevoltage(bq, bq->jeita_mv);

		/*bq2429x_set_appropriate_battery_currentt(bq, bq->jeita_ma);*/
		vote(bq->fcc_votable, BATT_TEMP_FCC_VOTER, true, bq->jeita_ma);
		if (ret < 0)
			pr_err("%s,charging current limit: BATT_TEMP_FCC_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

	} else {
		vote(bq->fcc_votable, BATT_TEMP_FCC_VOTER, false, bq->jeita_ma);
		if (ret < 0)
			pr_err("%s,charging current limit: BATT_TEMP_FCC_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		bq2429x_set_chargevoltage(bq, bq->platform_data->usb.float_voltage_mv);
	}

	/*schedule_delayed_work(&bq->jeita_work, 10 * HZ);*/
}

static const unsigned char *charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};

static void bq2429x_update_status(struct bq2429x *bq)
{
	u8 status;
	int ret;

	ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_01);
	if (ret)
		return;
	mutex_lock(&bq->lock);
	bq->charge_enabled = ((status & REG01_CHG_CONFIG_MASK) >> REG01_CHG_CONFIG_SHIFT) == REG01_CHG_ENABLE;
	bq->otg_enabled = ((status & REG01_OTG_CONFIG_MASK) >> REG01_OTG_CONFIG_SHIFT) == REG01_OTG_ENABLE;
	mutex_unlock(&bq->lock);

	ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_08);
	if (ret)
		return;

	mutex_lock(&bq->lock);
	bq->charge_state = (status & REG08_CHRG_STAT_MASK) >> REG08_CHRG_STAT_SHIFT;
	bq->power_good = !!(status & REG08_PG_STAT_MASK);
	bq->in_therm_regulation = !!(status & REG08_THERM_STAT_MASK);
	bq->in_vsys_regulation = !!(status & REG08_VSYS_STAT_MASK);
	bq->dpm_triggered = !!(status & REG08_INDPM_STAT_MASK);
	mutex_unlock(&bq->lock);

	ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_09);
	if (ret)
		return;

	mutex_lock(&bq->lock);
	bq->fault_status = status;
	mutex_unlock(&bq->lock);


	if (!bq->power_good)
		pr_info("Power Poor\n");
	if (bq->in_therm_regulation)
		pr_info("In thermal regulation!\n");
	if (bq->in_vsys_regulation)
		pr_info("In VSYS regulation!\n");
	if (bq->dpm_triggered)
		pr_info("VINDPM or IINDPM triggered\n");

	if (bq->fault_status & REG09_FAULT_WDT_MASK) {
		int fcc_ma = 0, icl_ma = 0;

		bq2429x_init_device(bq);

		fcc_ma = get_effective_result(bq->fcc_votable);
		if (fcc_ma > 0)
			bq2429x_set_chargecurrent(bq, fcc_ma);

		icl_ma = get_effective_result(bq->usb_icl_votable);
		bq2429x_set_input_current_limit(bq, icl_ma);

		pr_notice("Watchdog timer expired! reset fcc_ma=%d,icl_ma=%d", fcc_ma, icl_ma);
	}
	if (bq->fault_status & REG09_FAULT_BOOST_MASK)
		pr_info("Boost fault occurred!\n");

	status = (bq->fault_status & REG09_FAULT_CHRG_MASK) >> REG09_FAULT_CHRG_SHIFT;
	if (status == REG09_FAULT_CHRG_INPUT) {
#ifdef CONFIG_BQ2XXXX_OVER_VOLTAGE_REPORT
		if (bq->input_unspec && (bq->psy_health_sts != POWER_SUPPLY_HEALTH_OVERVOLTAGE)) {
			bq->psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			power_supply_set_health_state(bq->usb_psy, bq->psy_health_sts);
			power_supply_changed(bq->usb_psy);
			pr_info("input overvoltage!\n");
		}
#endif
		pr_info("input fault!\n");
	} else if (status == REG09_FAULT_CHRG_THERMAL)
		pr_info("charge thermal shutdown fault!\n");
	else if (status == REG09_FAULT_CHRG_TIMER)
		pr_info("charge timer expired fault!\n");

#ifdef CONFIG_BQ2XXXX_OVER_VOLTAGE_REPORT
	if ((bq->psy_health_sts == POWER_SUPPLY_HEALTH_OVERVOLTAGE) && bq->vbus_type && bq->power_good) {
		bq->psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
		power_supply_set_health_state(bq->usb_psy, bq->psy_health_sts);
		power_supply_changed(bq->usb_psy);
		pr_info("input overvoltage to good\n");
	} else if ((bq->psy_health_sts == POWER_SUPPLY_HEALTH_OVERVOLTAGE) && (!bq->vbus_type)) {
		bq->psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
		power_supply_set_health_state(bq->usb_psy, bq->psy_health_sts);
		power_supply_changed(bq->usb_psy);
		pr_info("input overvoltage to unknown\n");
	}
#endif

	if (bq->fault_status & REG09_FAULT_BAT_MASK)
		pr_info("battery ovp fault!\n");

	status = (bq->fault_status & REG09_FAULT_NTC_MASK) >> REG09_FAULT_NTC_SHIFT;

	if (status == REG09_FAULT_NTC_HOT)
		pr_info("NTC HOT\n");
	else if (status == REG09_FAULT_NTC_COLD)
		pr_info("NTC COLD\n");
	else
		pr_info("NTC Normal\n");

	pr_info("%s fault status:0x%x\n", charge_stat_str[bq->charge_state], bq->fault_status);

	bq2429x_dump_regs(bq);
}

#ifdef CHG_SMOOTH_BATTERY_PROP
static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(100, soc);
	return soc;
}

static int update_soc(struct battery_smooth_prop *smooth, int soc)
{
	soc = bound_soc(soc);

	if (smooth->fg_soc <= 0 && soc <= 0) {
		if (smooth->avg_volt > 3400000) {
			soc = 1;
		} else {
			soc = 0;
		}
	}

	if (soc != smooth->ui_soc) {
		smooth->ui_soc = soc;
		power_supply_changed(&g_bq->batt_psy);
	}
	return 0;
}

static void battery_smooth_up_work(struct work_struct *work)
{
	int ui_soc = 0;
	struct battery_smooth_prop *smooth = container_of(work,
				struct battery_smooth_prop,
				battery_smooth_up.work);

	pr_notice("bq2429x %s\n", __func__);
	mutex_lock(&smooth->mutex_smoothing_battery);

	ui_soc = smooth->ui_soc;
	if (ui_soc < 100) {
		ui_soc++;
	}
	update_soc(smooth, ui_soc);

	mutex_unlock(&smooth->mutex_smoothing_battery);
}

static void battery_smooth_down_work(struct work_struct *work)
{
	int ui_soc = 0;
	struct battery_smooth_prop *smooth = container_of(work,
				struct battery_smooth_prop,
				battery_smooth_down.work);

	pr_notice("bq2429x %s\n", __func__);

	mutex_lock(&smooth->mutex_smoothing_battery);

	ui_soc = smooth->ui_soc;
	if (ui_soc > smooth->current_soc) {
		ui_soc--;
	}
	update_soc(smooth, ui_soc);

	mutex_unlock(&smooth->mutex_smoothing_battery);
}

static int smbchg_smoothing_battery_prop(void)
{
	int ui_soc = 0, current_soc = 0;
	struct battery_smooth_prop *smooth = &g_smooth_prop;

	ui_soc = smooth->ui_soc;
	current_soc = smooth->current_soc;

	mutex_lock(&smooth->mutex_smoothing_battery);

	/*
	pr_notice("charger present:%d, therm_status:%d, battery_status:%d soc=%d ui_soc=%d\n",
		g_smooth_prop.chg_present, g_smooth_prop.battery_therm_status,
		g_smooth_prop.battery_status, current_soc, ui_soc);
	*/
	pr_notice("ui_soc = %d\n", ui_soc);
	if (smooth->chg_present == 1) {
		if (smooth->battery_status == POWER_SUPPLY_STATUS_FULL) {
			cancel_delayed_work(&smooth->battery_smooth_down);
			if (ui_soc < 100) {
				smooth->soc_direction = 1;
				schedule_delayed_work(&smooth->battery_smooth_up, msecs_to_jiffies(60000));
			}
		} else if (smooth->battery_status == POWER_SUPPLY_STATUS_CHARGING) {
			cancel_delayed_work(&smooth->battery_smooth_down);
			if ((ui_soc - current_soc) > 2) {
				/*
				g_smooth_prop.soc_direction = 1;
				schedule_delayed_work(&g_smooth_prop.battery_smooth_up, msecs_to_jiffies(60000));
				*/
			/*merge from svn*/
			} else if (ui_soc < (current_soc - 1)) {
				smooth->soc_direction = 1;
				schedule_delayed_work(&smooth->battery_smooth_up, msecs_to_jiffies(100000));
				pr_notice("bq2429x charging, ui_soc < soc ,ui_soc increase 1 per 100s!\n");
			/*merge from svn*/
			} else {
				ui_soc = current_soc;
			}
		} else {
			/*POWER_SUPPLY_STATUS_DISCHARGING ||
			POWER_SUPPLY_STATUS_NOT_CHARGING ||
			POWER_SUPPLY_STATUS_UNKNOWN*/
			cancel_delayed_work(&smooth->battery_smooth_up);
			if ((ui_soc == 100) &&
				(current_soc > 97) &&
				((smooth->current_boot_sec - smooth->chg_unplugged_time) < 180)) {
				pr_notice("bq2429x decrease soc after 3 min\n");
			} else if ((ui_soc - current_soc) > 2) {
				smooth->soc_direction = 0;
				schedule_delayed_work(&smooth->battery_smooth_down, msecs_to_jiffies(60000));
			} else {
				ui_soc = current_soc;
			}
		}
	} else {
		cancel_delayed_work(&smooth->battery_smooth_up);
		 if ((ui_soc == 100) &&
			(current_soc > 97) &&
			((smooth->current_boot_sec - smooth->chg_unplugged_time) < 180)) {
			pr_notice("bq2429x decrease soc after 3 min\n");
		} else if ((ui_soc - current_soc) > 2) {
			smooth->soc_direction = 0;
			if (current_soc > 90)
				schedule_delayed_work(&smooth->battery_smooth_down, msecs_to_jiffies(60000));
			if (current_soc > 30)
				schedule_delayed_work(&smooth->battery_smooth_down, msecs_to_jiffies(45000));
			else if (current_soc > 15)
				schedule_delayed_work(&smooth->battery_smooth_down, msecs_to_jiffies(30000));
			else
				schedule_delayed_work(&smooth->battery_smooth_down, msecs_to_jiffies(10000));
		/*merge from svn*/
		} else if (ui_soc < current_soc) {
			pr_notice("bq2429x discharging, ui_soc < soc ,const ui_soc!\n");
		/*merge from svn*/
		} else {
			ui_soc = current_soc;
		}
	}
	update_soc(smooth, ui_soc);

	mutex_unlock(&smooth->mutex_smoothing_battery);
	return 0;
}

static int smbchg_smoothing_init(struct bq2429x *bq)
{
	int status = 0, health = 0, soc = 0, current_now = 0, voltage_now = 0, temp = 0;

	status = bq2429x_get_prop_batt_status(bq);
	health = bq2429x_get_prop_charger_health(bq);
	soc = bq2429x_get_prop_batt_capacity(bq);
	voltage_now = bq2429x_get_prop_battery_voltage_now(bq);
	current_now = bq2429x_get_prop_batt_current_now(bq);
	temp = bq2429x_get_prop_batt_temp(bq);

	g_smooth_prop.avg_volt = voltage_now;
	g_smooth_prop.avg_temp = temp;
	g_smooth_prop.pre_soc = g_smooth_prop.current_soc = g_smooth_prop.fg_soc = g_smooth_prop.ui_soc = soc;
	g_smooth_prop.chg_present = bq->chg_present;
	g_smooth_prop.battery_status = status;
	g_smooth_prop.battery_therm_status = BATTERY_NORMAL;
	g_smooth_prop.battinfo_print_sec = 0;
	g_smooth_prop.overdischarge = false;

	INIT_DELAYED_WORK(&g_smooth_prop.battery_smooth_up, battery_smooth_up_work);
	INIT_DELAYED_WORK(&g_smooth_prop.battery_smooth_down, battery_smooth_down_work);
	return 0;
}

static int update_average_voltage(int volt)
{
	int i = 0;
	unsigned int amount = 0;
	static int index = 0;
	static int number = 0;

	g_smooth_prop.voltages[index] = volt;

	number++;
	index++;

	if (index >= CALC_AVG_NUMS) {
		index = 0;
	}
	if (number >= CALC_AVG_NUMS)
		number = CALC_AVG_NUMS;

	amount = 0;
	for (i = 0; i < number; i++) {
		amount += g_smooth_prop.voltages[i];
	}
	/*pr_notice("avgvbat index:%d, number:%d, amount:%u, average:%d.\n", index, number, amount, amount / number);*/

	g_smooth_prop.avg_volt = amount / number;
	return g_smooth_prop.avg_volt;
}
static int update_average_temp(int temp)
{
	int i = 0;
	int amount = 0;
	static int index = 0;
	static int number = 0;

	g_smooth_prop.temps[index] = temp;

	number++;
	index++;

	if (index >= CALC_AVG_NUMS) {
		index = 0;
	}
	if (number >= CALC_AVG_NUMS) {
		number = CALC_AVG_NUMS;
	}

	amount = 0;
	for (i = 0; i < number; i++) {
		amount += g_smooth_prop.temps[i];
	}
	/*pr_notice("avgvbat temp index:%d, number:%d, amount:%u, average:%d.\n",
	index, number, amount, amount / number);*/
	g_smooth_prop.avg_temp = amount / number;
	return g_smooth_prop.avg_temp;
}
static int is_voltage_invalid(int volt)
{
	if ((volt > 6000000) || (volt < 100000))
		return true;

	return false;
}
static int is_temp_invalid(int temp)
{
	if ((temp > 2000) || (temp < -1000))
		return true;

	return false;
}
#endif

/*merge from svn*/
void bq2429x_charging_init(struct bq2429x *bq)
{
	int fcc_ma = 0, icl_ma = 0;

	bq2429x_exit_hiz_mode(bq);
	bq2429x_init_device(bq);
	bq2429x_set_appropriate_current(bq);
	bq2429x_reset_watchdog_timer(bq);
	bq2429x_enable_safety_timer(bq, 0);
	/* bq24296_set_int_mask(0x0); Disable fault interrupt */

	fcc_ma = get_effective_result(bq->fcc_votable);
	if (fcc_ma > 0)
		bq2429x_set_chargecurrent(bq, fcc_ma);

	icl_ma = get_effective_result(bq->usb_icl_votable);
	bq2429x_set_input_current_limit(bq, icl_ma);
	pr_notice("bq2429x_charging_init:set fcc_ma=%d,icl_ma=%d\n", fcc_ma, icl_ma);
}
/*merge from svn*/

int bq27x00_battery_get_fg_regs(struct power_supply *psy, int *regs);
static int bq2429x_monitor(struct bq2429x *bq)
{
	/*struct bq2429x *bq = container_of(work, struct bq2429x, monitor_work.work);*/
	int status = 0, health = 0, soc = 0, current_now = 0, voltage_now = 0, temp = 0, chg_type = 0;
	ktime_t current_ktime;
	s64 current_boot_sec = 0;
	static unsigned int nums = 0;
	int ret = 0;

	current_ktime = ktime_get_boottime();
	current_boot_sec = div_s64(ktime_to_ns(current_ktime), 1000000000);

	/*merge from svn*/
	if (bq->chg_present == true) {
		if ((bq2429x_get_safety_timer_state(bq) == 1) || (bq2429x_get_watchdog_timer_state(bq) == 1)) {
			bq2429x_charging_init(bq);
			pr_err("bq2429x watchdog expired,charging re-init!\n");
		}
	}
	/*merge from svn*/

	status = bq2429x_get_prop_batt_status(bq);
	health = bq2429x_get_prop_charger_health(bq);
	soc = bq2429x_get_prop_batt_capacity(bq);
	voltage_now = bq2429x_get_prop_battery_voltage_now(bq);
	current_now = bq2429x_get_prop_batt_current_now(bq);
	temp = bq2429x_get_prop_batt_temp(bq);
	chg_type = bq->chg_type;

	if (!bq->test_disable_feed_wdt)
		bq2429x_reset_watchdog_timer(bq);
	else {
		pr_notice("temp disable feed whatchdog\n");
		/*bq->test_disable_feed_wdt = false;*/
	}

#ifdef CHG_SMOOTH_BATTERY_PROP
	if (is_temp_invalid(temp)) {
		pr_err("bq2429x temp invalid:%d\n", temp);
		if (soc == 0) {
			pr_err("bq2429x temp && soc invalid skip soc cal\n");
			goto out;
		}
	} else {
		update_average_temp(temp);
		bq2429x_battery_temp_work(bq, temp);
	}
	if (is_voltage_invalid(voltage_now)) {
		pr_err("bq2429x vol invalid:%d\n", voltage_now);
		if (soc == 0) {
			pr_err("bq2429x vol && soc invalid skip soc cal\n");
			goto out;
		}
	} else {
		update_average_voltage(voltage_now);
	}

	/*avoid battery overdischarge.*/
	if (g_smooth_prop.overdischarge && bq->chg_present) {
		g_smooth_prop.overdischarge = false;
	}
	if ((g_smooth_prop.avg_volt < 3320000) && (!bq->chg_present) && (soc < 10) && (!g_smooth_prop.overdischarge)) {
		pr_err("battery voltage is low, soc is %d > 0, then set current soc = 0.\n", soc);
		g_smooth_prop.current_soc = 0;
		g_smooth_prop.overdischarge = true;
	} else if (g_smooth_prop.overdischarge) {
		pr_err("battery voltage is low, soc is %d > 0, then set current soc = 0.\n", soc);
		g_smooth_prop.current_soc = 0;
		g_smooth_prop.overdischarge = true;
	} else {
		g_smooth_prop.current_soc = soc;
	}

	g_smooth_prop.fg_soc = soc;
	g_smooth_prop.chg_present = bq->chg_present;
	g_smooth_prop.battery_status = status;
	g_smooth_prop.battery_therm_status = BATTERY_NORMAL;

	if (bq->batt_hot) {
		g_smooth_prop.battery_therm_status = BATTERY_HOT;
	} else if (bq->batt_warm) {
		g_smooth_prop.battery_therm_status = BATTERY_WARM;
	} else if (bq->batt_cool) {
		g_smooth_prop.battery_therm_status = BATTERY_COOL;
	} else if (bq->batt_cold) {
		g_smooth_prop.battery_therm_status = BATTERY_COLD;
	} else {
		g_smooth_prop.battery_therm_status = BATTERY_NORMAL;
	}

	if (bq->chg_term_by_soc) {
		int avg_voltage = g_smooth_prop.avg_volt / 1000;

		if (bq->chg_present) {
			if (bq->chg_term_battery_status != POWER_SUPPLY_STATUS_FULL) {
				/*pr_notice("bq2429x soc=%d avg_voltage= %d jeita_mv=%d fv=%d\n",
				soc, avg_voltage, bq->jeita_mv, bq->platform_data->usb.float_voltage_mv);*/
				if ((soc == 100) ||
					(bq->jeita_active && (bq->jeita_mv < bq->platform_data->usb.float_voltage_mv) &&
					(avg_voltage > (bq->jeita_mv - 12)))) {
					if (soc == 100)
						bq->chg_term_full_charge_detected = true;
					if (bq->batt_cold || bq->batt_hot) {
						bq->chg_term_battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
					} else {
						bq->chg_term_battery_status = POWER_SUPPLY_STATUS_FULL;

						vote(bq->fcc_votable, CHG_TERM_SOC_FCC_VOTER, true, 0);
						if (ret < 0)
							pr_err("%s,charging current limit: CHG_TERM_SOC_FCC_VOTER couldn't vote, ret=%d\n",
								__func__, ret);
					}
				}
			} else {
				if ((!bq->jeita_active && !bq->chg_term_full_charge_detected) ||
					(!bq->jeita_active && (avg_voltage <
					(bq->platform_data->usb.float_voltage_mv - 300))) ||
					(bq->jeita_active &&  (avg_voltage < (bq->jeita_mv - 300))) ||
					(bq->batt_cold || bq->batt_hot)) {
					bq->chg_term_full_charge_detected = false;
					bq->chg_term_battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
					vote(bq->fcc_votable, CHG_TERM_SOC_FCC_VOTER, false, 0);
					if (ret < 0)
						pr_err("%s,charging current limit: CHG_TERM_SOC_FCC_VOTER couldn't vote, ret=%d\n",
							__func__, ret);
				}
			}
		} else {
			bq->chg_term_full_charge_detected = false;
			bq->chg_term_battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
			vote(bq->fcc_votable, CHG_TERM_SOC_FCC_VOTER, false, 0);
			if (ret < 0)
				pr_err("%s,charging current limit: CHG_TERM_SOC_FCC_VOTER couldn't vote, ret=%d\n",
					__func__, ret);
		}
	}

	g_smooth_prop.current_boot_sec = current_boot_sec;
	if ((!g_smooth_prop.chg_present) && g_smooth_prop.pre_chg_present) {
		g_smooth_prop.chg_unplugged_time = current_boot_sec;
	}
	g_smooth_prop.pre_chg_present = g_smooth_prop.chg_present;

	smbchg_smoothing_battery_prop();
out:
	if (current_boot_sec - g_smooth_prop.battinfo_print_sec > 16) {
		pr_notice("time=,%lld, avgvbat=,%d, V=,%d, I=,%d, "
			"avgt=,%d, T=,%d, presoc=,%d, soc=,%d, "
			"uisoc=,%d, chg=,%d, Pret=,%d, Therm=,%d, BatSta=,%d\n",
			g_smooth_prop.current_boot_sec, g_smooth_prop.avg_volt / 1000,
			voltage_now / 1000, current_now / 1000,
			g_smooth_prop.avg_temp, temp, g_smooth_prop.pre_soc, soc,
			g_smooth_prop.ui_soc, chg_type, g_smooth_prop.chg_present,
			g_smooth_prop.battery_therm_status, g_smooth_prop.battery_status);

		g_smooth_prop.battinfo_print_sec = current_boot_sec;
	}
	/*
	pr_notice("Time=,%lld, AvgVbat=,%d,Vbat=,%d,AvgIchr=,%d,Ichr=,%d,Ibat=,%d,VChr=,%d,AvgT=,%d,
			T=,%d,pre_SOC=,%d,SOC=,%d,UI_SOC=,%d,ZCV=,%d,CHR_Type=,%d, Pret:%d, Therm:%d, BatSta:%d",
			current_boot_sec, voltage_now, voltage_now, current_now, current_now, current_now,
			voltage_now, temp, temp, g_smooth_prop.pre_soc, soc, g_smooth_prop.ui_soc, voltage_now,
			chg_type, g_smooth_prop.chg_present,
			g_smooth_prop.battery_therm_status, g_smooth_prop.battery_status);
	*/
	g_smooth_prop.pre_soc = g_smooth_prop.current_soc;
#else
	bq2429x_battery_temp_work(bq, temp);
out:
	pr_info("bat_vol=,%d,Ibat=,%d,T=,%d,SOC=,%d,CHR_Type=,%d,\n",
		voltage_now/1000, current_now/1000, temp, soc, chg_type);
#endif
	/*schedule_delayed_work(&bq->monitor_work,10*HZ);*/
	if ((nums % 20) == 0) {
		bq2429x_update_status(bq);
	}
	nums++;

	cancel_delayed_work_sync(&bq->charging_policy_work);
	schedule_delayed_work(&bq->charging_policy_work, round_jiffies_relative(msecs_to_jiffies(3000)));

	/* wa: bq24297 will auto set icl as 3A if charger type is dcp */
	cancel_delayed_work_sync(&bq->dcp_icl_recheck_work);
	schedule_delayed_work(&bq->dcp_icl_recheck_work, msecs_to_jiffies(100));

	return 0;
}
static void bq2429x_monitor_workfunc(struct work_struct *work)
{
	struct bq2429x *bq = container_of(work, struct bq2429x, monitor_work.work);

	mutex_lock(&bq->irq_complete);
	bq->monitor_work_waiting = false;
	wake_lock_timeout(&bq->smb_monitor_wake_lock, 1 * HZ);    /*added for suspend20170321*/
	if (bq->resume_completed == false) {
		bq->monitor_work_waiting = true;
		mutex_unlock(&bq->irq_complete);
		goto out;
	}
	mutex_unlock(&bq->irq_complete);

	bq2429x_monitor(bq);

out:
	schedule_delayed_work(&bq->monitor_work, msecs_to_jiffies(15000));
}

static void bq2429x_otg_boot_workfunc(struct work_struct *work)
{
	struct bq2429x *bq = container_of(work, struct bq2429x, otg_boot_work.work);

	if (!!gpio_get_value(bq->platform_data->otg_irq_gpio) == 0) {
		power_supply_set_usb_otg(bq->usb_psy, 1);
		pr_info("bq2429x_otg_boot_workfunc: otg enable\n");
	}
}
static irqreturn_t bq2429x_otg_handler(int irq, void *dev_id)
{
	struct bq2429x *bq = dev_id;
	int irq_value = 0;
	int otg_enabled = 0;
	/*int rc = -1;*/

	irq_value = gpio_get_value(bq->platform_data->otg_irq_gpio);
	pr_info(" irq value:%d", irq_value);

	otg_enabled = !irq_value;
	/*
	if (otg_enabled) {
		otg_enabled = 0;
		bq2429x_disable_otg(g_bq);
	} else {
		otg_enabled = 1;
		bq2429x_enable_otg(g_bq);
	}
	*/

	dev_notice(bq->dev, "%s status = 0x%02x\n", __func__, otg_enabled);
	power_supply_set_usb_otg(bq->usb_psy, otg_enabled ? 1 : 0);

	return IRQ_HANDLED;
}

static void bq2429x_dcp_icl_recheck_work(struct work_struct *work)
{
	struct bq2429x *bq = container_of(work, struct bq2429x, dcp_icl_recheck_work.work);
	u8 status;
	int icl = 0;
	int ret;

	ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_00);
	if (ret) {
		pr_err("fail to read BQ2429X_REG_00.\n");
		return;
	}
	pr_info("BQ2429X_REG_00 = 0x%x before icl recheck\n", status);
	if ((status & 0x7) == REG00_IINLIM_3000MA)
		icl = BQ2429X_ILIM_3000mA;
	else if ((status & 0x7) == REG00_IINLIM_2000MA)
		icl = BQ2429X_ILIM_2000mA;
	else if ((status & 0x7) == REG00_IINLIM_1500MA)
		icl = BQ2429X_ILIM_1500mA;
	else if ((status & 0x7) == REG00_IINLIM_1000MA)
		icl = BQ2429X_ILIM_1000mA;
	else if ((status & 0x7) == REG00_IINLIM_900MA)
		icl = BQ2429X_ILIM_900mA;
	else if ((status & 0x7) == REG00_IINLIM_500MA)
		icl = BQ2429X_ILIM_500mA;
	else if ((status & 0x7) == REG00_IINLIM_150MA)
		icl = BQ2429X_ILIM_150mA;
	else if ((status & 0x7) == REG00_IINLIM_100MA)
		icl = BQ2429X_ILIM_100mA;

	if (icl > bq->platform_data->ta.ilim) {
		bq2429x_set_input_current_limit(bq, bq->platform_data->ta.ilim);
		ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_00);
		if (ret) {
			pr_err("fail to read BQ2429X_REG_00.\n");
			return;
		}
		pr_info("BQ2429X_REG_00 = 0x%x after icl recheck\n", status);
	}
}

static void bq2429x_usb_type_detection_work(struct work_struct *work)
{
	struct bq2429x *bq = container_of(work, struct bq2429x, usb_type_detection_work.work);
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;
	u8 status;
	int ret, retry = 0, retry_max = 100;
	static bool float_chg_det_done = false;

	if (bq->need_recheck_usb_type && float_chg_det_done == false) {
		power_supply_set_supply_type(bq->usb_psy, POWER_SUPPLY_TYPE_UNKNOWN);
		power_supply_set_present(bq->usb_psy, false);
		msleep(20);
		bq->chg_present = false;
		pr_info("force dpdm detection\n");
		bq2429x_set_dp_dm_detection_enable(bq);
		while (retry < retry_max) {
			ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_07);
			if (ret) {
				pr_err("fail to read BQ2429X_REG_07.\n");
				return;
			}
			if ((status & REG07_FORCE_DPDM_MASK) == REG07_FORCE_DPDM_MASK) {
				msleep(20);
				retry++;
			} else {
				pr_info("dpdm detection completed\n");
				break;
			}
		}
		if (retry == retry_max)
			pr_err("dpdm detection timeout\n");
		bq->need_recheck_usb_type = false;
		float_chg_det_done = true;
		return;
	}

	ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_08);
	if (ret) {
		pr_err("fail to read BQ2429X_REG_08.\n");
		return;
	}
	bq->vbus_type = (status & REG08_VBUS_STAT_MASK) >> REG08_VBUS_STAT_SHIFT;
	bq->power_good = !!(status & REG08_PG_STAT_MASK);
	pr_info("reg08:0x%x, vbus_type:%d, power_good:%d\n", status, bq->vbus_type, bq->power_good);

	if (bq->vbus_type == BQ2429X_VBUS_USB) {
		type = POWER_SUPPLY_TYPE_USB;
	} else if (bq->vbus_type == BQ2429X_VBUS_ADAPTER) {
		type = POWER_SUPPLY_TYPE_USB_DCP;
	}

	if (!bq->power_good && bq->chg_present) {
		bq->chg_present = false;
		power_supply_set_present(bq->usb_psy, bq->chg_present);
		pr_info("usb plugged out, set usb present = %d\n", bq->chg_present);
		bq2429x_disable_watchdog_timer(bq);
		if (bq->battery_suspend_lock.ws.active == true) {
			wake_unlock(&bq->battery_suspend_lock);
		}
		alarm_cancel(&bq->charging_expired_alarm);
		bq2429x_battery_clean_charging_expired(bq);
		float_chg_det_done = false;
	} else if (bq->power_good && !bq->chg_present) {
		bq->chg_present = true;
		if (bq->battery_suspend_lock.ws.active == false) {
			wake_lock(&bq->battery_suspend_lock);
		}
		if (bq->software_jeita_supported)
			bq2429x_battery_temp_work(bq, bq->batt_temp);
		power_supply_set_supply_type(bq->usb_psy, type);
		power_supply_set_present(bq->usb_psy, bq->chg_present);
		bq2429x_set_watchdog_timer(bq, 80);
		pr_info("usb plugged in, set usb present = %d\n", bq->chg_present);
		alarm_cancel(&bq->charging_expired_alarm);
		msleep(20);
		ret = alarm_start_relative(&bq->charging_expired_alarm,
			ns_to_ktime(g_charging_expiration_ns));
		if (ret) {
			pr_err("Failed to start alarm: %d\n", ret);
		}
		cancel_delayed_work_sync(&bq->charging_policy_work);
		schedule_delayed_work(&bq->charging_policy_work, round_jiffies_relative(msecs_to_jiffies(3000)));
	}

	bq2429x_update_status(bq);
}

static irqreturn_t bq2429x_charger_interrupt(int irq, void *dev_id)
{
	struct bq2429x *bq = dev_id;
	static bool charger_int_init = false;  /*merge from svn*/
	u8 status;
	int ret;

	pr_info("begin.\n");

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = false;
	wake_lock_timeout(&bq->usb_plug_in_out_wake_lock, 1 * HZ);
	if (bq->resume_completed == false) {
		bq->irq_waiting = true;
		mutex_unlock(&bq->irq_complete);
		goto out;
	}
	mutex_unlock(&bq->irq_complete);

	/*  for power monitor fake battery unable mount usb on pv version
	cancel_delayed_work_sync(&bq->usb_type_detection_work); */
	schedule_delayed_work(&bq->usb_type_detection_work, msecs_to_jiffies(300));
	pr_info("usb_type_detection_work in 300ms.\n");

	ret = bq2429x_read_byte(bq, &status, BQ2429X_REG_08);
	if (ret)
		return IRQ_HANDLED;
	pr_info("reg08 : 0x%x\n", status);

	mutex_lock(&bq->lock);
	bq->vbus_type = (status & REG08_VBUS_STAT_MASK) >> REG08_VBUS_STAT_SHIFT;
	bq->power_good = !!(status & REG08_PG_STAT_MASK);
	mutex_unlock(&bq->lock);

	/*merge from svn*/
	if (charger_int_init == false) {
		if (!bq->power_good) {
			bq->chg_present = false;
			bq2429x_disable_watchdog_timer(bq);
			/*cancel_delayed_work_sync(&bq->monitor_work);*/
			power_supply_set_present(bq->usb_psy, 0);
			pr_info("boot usb is not exist, set usb present = %d\n", bq->chg_present);

			if (bq->battery_suspend_lock.ws.active == true) {
				wake_unlock(&bq->battery_suspend_lock);
			}
		}
		charger_int_init = true;
	}
	/*merge from svn*/

#ifdef CONFIG_BQ2XXXX_OVER_VOLTAGE_REPORT
	if (bq->vbus_type && !bq->power_good && bq->chg_present) {
		msleep(20);

		status = 0;
		bq2429x_read_byte(bq, &status, BQ2429X_REG_08);
		bq->vbus_type = (status & REG08_VBUS_STAT_MASK) >> REG08_VBUS_STAT_SHIFT;
		bq->power_good = !!(status & REG08_PG_STAT_MASK);
		pr_info("usb present, power poor, check again staus= 0x%x, vbus_type:%d, power_good:%d\n",
			status, bq->vbus_type, bq->power_good);

		if (bq->vbus_type && !bq->power_good) {
			bq->input_unspec = true;
			bq2429x_update_status(bq);
			return IRQ_HANDLED;
		}
	}
	bq->input_unspec = false;
#endif
#if 0
	if (!bq->power_good && bq->chg_present) {
		bq->chg_present = false;
		/*cancel_delayed_work_sync(&bq->monitor_work);*/
		power_supply_set_present(bq->usb_psy, 0);
		pr_info("usb removed, set usb present = %d\n", bq->chg_present);

		bq2429x_disable_watchdog_timer(bq);

		if (bq->battery_suspend_lock.ws.active == true) {
			wake_unlock(&bq->battery_suspend_lock);
		}
	} else if (bq->power_good && !bq->chg_present) {
		bq->chg_present = true;
		if (bq->battery_suspend_lock.ws.active == false) {
			wake_lock(&bq->battery_suspend_lock);
		}
		/*schedule_delayed_work(&bq->monitor_work, HZ);*/
		if (bq->software_jeita_supported)
			/*schedule_delayed_work(&bq->jeita_work, 0);*/
			bq2429x_battery_temp_work(bq, bq->batt_temp);
		power_supply_set_present(bq->usb_psy, bq->chg_present);

		bq2429x_set_watchdog_timer(bq, 80);

		pr_info("usb plugged in, set usb present = %d\n", bq->chg_present);
	}

	bq2429x_update_status(bq);
#endif
out:
	return IRQ_HANDLED;
}

static int get_reg(void *data, u64 *val)
{
	struct bq2429x *bq = data;
	int rc;
	u8 temp;

	rc = bq2429x_read_byte(bq, &temp, bq->peek_poke_address);
	if (rc < 0) {
		dev_err(bq->dev,
			"Couldn't read reg %x rc = %d\n",
			bq->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct bq2429x *bq = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = bq2429x_write_byte(bq, bq->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(bq->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			bq->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int show_registers(struct seq_file *m, void *data)
{
	struct bq2429x *bq = m->private;
	int addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x0A; addr++) {
		ret = bq2429x_read_byte(bq, &val, addr);
		if (!ret)
			seq_printf(m, "Reg[0x%.2x] = 0x%.2x\n", addr, val);
	}
	return 0;
}


static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2429x *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}

static int set_fastchg_current_vote_cb(struct votable *votable, void *dev,
						int fcc_ma,
						const char *client)
{
	struct bq2429x *bq = dev_get_drvdata(dev);
	int rc = -1;

	pr_info("%s set %d mA\n", __func__, fcc_ma);

	if (fcc_ma > 0) {
		rc = bq2429x_enable_charger(bq);
		rc = bq2429x_set_chargecurrent(bq, fcc_ma);
		rc = bq2429x_set_watchdog_timer(bq, 80);
		pr_err("bq2429x_enable_charger!\n");
	} else {
		rc = bq2429x_disable_charger(bq);
		rc = bq2429x_disable_watchdog_timer(bq);
		pr_err("bq2429x_disable_charger!\n");
	}

	if (rc) {
		dev_err(bq->dev, "Can't set FCC fcc_ma=%d rc=%d\n", fcc_ma, rc);
		return rc;
	}

	return 0;
}
/*
 * set the usb charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int set_usb_current_limit_vote_cb(struct votable *votable, void *dev,
						int icl_ma,
						const char *client)
{
	struct bq2429x *bq = dev_get_drvdata(dev);
	int rc = -1;

	pr_info("%s set %d mA\n", __func__, icl_ma);

	rc = bq2429x_set_input_current_limit(bq, icl_ma);

	if (rc) {
		dev_err(bq->dev, "Can't set FCC fcc_ma=%d rc=%d\n", icl_ma, rc);
		return rc;
	}

	return 0;
}

static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if 0
static void create_debugfs_entry(struct bq2429x *bq)
{
	bq->debug_root = debugfs_create_dir("bq2429x", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
						bq->debug_root, bq, &reg_debugfs_ops);
		debugfs_create_x32("fault_status", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->fault_status));

		debugfs_create_x32("vbus_type", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->vbus_type));

		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charge_state));
	}
}
#endif
static void create_debugfs_entry(struct bq2429x *bq)
{
	int rc;

	bq->debug_root = debugfs_create_dir("bq2429x", NULL);
	if (!bq->debug_root)
		dev_err(bq->dev, "Couldn't create debug dir\n");

	if (bq->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("registers", S_IFREG | S_IRUGO,
					  bq->debug_root, bq,
					  &reg_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(bq->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->peek_poke_address));
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(bq->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root, bq,
					  &poke_poke_debug_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(bq->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);
		}

		debugfs_create_x32("fault_status", S_IFREG | S_IRUGO, bq->debug_root, &(bq->fault_status));
		debugfs_create_x32("vbus_type", S_IFREG | S_IRUGO, bq->debug_root, &(bq->vbus_type));
		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO, bq->debug_root, &(bq->charge_state));
	}
}

static int bq2429x_pinctrl_init(struct bq2429x *bq)
{
	struct pinctrl_state *set_state;

	bq->pinctrl = devm_pinctrl_get(bq->dev);
	if (IS_ERR_OR_NULL(bq->pinctrl)) {
		pr_err("pinctrl not defined.\n");
		return PTR_ERR(bq->pinctrl);
	}


	set_state = pinctrl_lookup_state(bq->pinctrl, "pmx_otg_active");
	if (IS_ERR_OR_NULL(set_state)) {
		pr_err("pmx_otg_active lookup failed.\n");
		bq->enpin_output0 = NULL;
		return PTR_ERR(set_state);
	}
	bq->enpin_output0 = set_state;

	pinctrl_select_state(bq->pinctrl, bq->enpin_output0);
	/* get more pins conig here, if need */
	return 0;
}

static int bq2429x_charger_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct bq2429x *bq;
	struct power_supply *usb_psy;
	struct pinctrl *pinctrl;
	int ret;

	const char *str = (const char *)saved_command_line;
	const char *ret_boot = strnstr(str, "androidboot.mode=charger", sizeof("androidboot.mode=charger"));

	if (ret_boot != NULL) {
		charger_boot = true;
		pr_info("%s CHARGER BOOT!\n", __func__);
	}

	pr_info("func in\n");

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2429x), GFP_KERNEL);
	if (!bq) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->usb_psy = usb_psy;
	bq->bms_psy = NULL;
	bq->bms_psy_name = "bms";
	bq->client = client;
	i2c_set_clientdata(client, bq);
	bq->chip = BQ24297; /*id->driver_data;*/

	if (bq->chip == BQ24297)
		bq->dpdm_enabled = true;

	mutex_init(&bq2429x_i2c_lock);
	mutex_init(&bq->lock);
	mutex_init(&bq->current_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->irq_complete);

	bq->chg_present = false;
	bq->chg_term_by_soc = false;
	bq->chg_term_full_charge_detected = false;
	bq->test_disable_feed_wdt = false;
	bq->use_test_temp = false;
	bq->test_temp = 250;
	bq->use_test_batt_volt = false;
	bq->test_batt_volt = 3800000;
	bq->input_unspec = false;
	bq->psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
	bq->prop_set_charge_enabled = true;
	bq->need_recheck_usb_type = false;
	g_charging_expiration_ns = CHARGING_EXPIRATION_TIME_NS;

	g_bq = bq;

	ret = bq2429x_detect_device(bq);
	if (ret) {
		pr_err("No bq2429x device found!");
		devm_kfree(&client->dev, bq);
		return -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(&client->dev);
	if (IS_ERR(pinctrl)) {
		dev_warn(&client->dev, "pins are not configured from the driver\n");
		/*return -EINVAL;*/
	}

	wake_lock_init(&bq->battery_suspend_lock, WAKE_LOCK_SUSPEND, "battery suspend wakelock");
	wake_lock_init(&bq->smb_monitor_wake_lock, WAKE_LOCK_SUSPEND, "smb_monitor_wake");
	wake_lock_init(&bq->usb_plug_in_out_wake_lock, WAKE_LOCK_SUSPEND, "usb_plug_in_out_wake_lock");

	bq2429x_init_jeita(bq);

	if (client->dev.of_node)
		bq->platform_data = bq2429x_parse_dt(&client->dev, bq);
	else
		bq->platform_data = client->dev.platform_data;

	if (!bq->platform_data) {
		pr_err("No platform data provided.\n");
		devm_kfree(&client->dev, bq);
		return -EINVAL;
	}

	bq->fcc_votable = create_votable("bq2429x: fcc", VOTE_MIN, set_fastchg_current_vote_cb, &client->dev);
	if (IS_ERR(bq->fcc_votable)) {
		devm_kfree(&client->dev, bq);
		return PTR_ERR(bq->fcc_votable);
	}
	ret = vote(bq->fcc_votable, CHARGER_TYPE_FCC_VOTER, true, bq->platform_data->ta.ichg);
	if (ret < 0)
		pr_err("charging current limit: CHARGER_TYPE_FCC_VOTER couldn't vote, ret=%d\n",
			ret);

	bq->usb_icl_votable = create_votable("bq2429x: usb_icl", VOTE_MIN, set_usb_current_limit_vote_cb, &client->dev);
	if (IS_ERR(bq->usb_icl_votable)) {
		devm_kfree(&client->dev, bq);
		return PTR_ERR(bq->usb_icl_votable);
	}
	ret = vote(bq->usb_icl_votable, CHARGER_TYPE_ICL_VOTER, true, bq->platform_data->ta.ilim);
	if (ret < 0)
		pr_err("input current limit: CHARGER_TYPE_ICL_VOTER couldn't vote, ret=%d\n",
			ret);

	INIT_DELAYED_WORK(&bq->monitor_work, bq2429x_monitor_workfunc);
	/*INIT_DELAYED_WORK(&bq->jeita_work, bq2429x_jeita_workfunc);*/
	bq->resume_completed = true;

	ret = bq2429x_init_device(bq);
	if (ret) {
		pr_err("Failed to init device");
		devm_kfree(&client->dev, bq);
		return ret;
	}

#ifdef CHG_SMOOTH_BATTERY_PROP
	smbchg_smoothing_init(bq);
	mutex_init(&g_smooth_prop.mutex_smoothing_battery);
#endif

	ret = bq2429x_psy_register(bq);
	if (ret) {
		devm_kfree(&client->dev, bq);
		return ret;
	}

	ret = bq2429x_regulator_init(bq);
	if (ret) {
		pr_err("Couldn't initialize bq2429x regulator ret=%d\n", ret);
		devm_kfree(&client->dev, bq);
		return ret;
	}
	bq2429x_dump_regs(bq);

	schedule_delayed_work(&bq->monitor_work, HZ);
	power_supply_set_usb_otg(bq->usb_psy, 0);

	bq2429x_pinctrl_init(bq);

	INIT_DELAYED_WORK(&bq->usb_type_detection_work, bq2429x_usb_type_detection_work);
	INIT_DELAYED_WORK(&bq->dcp_icl_recheck_work, bq2429x_dcp_icl_recheck_work);
	INIT_DELAYED_WORK(&bq->charging_policy_work, bq2429x_charging_policy_work);
	alarm_init(&bq->charging_expired_alarm, ALARM_BOOTTIME, bq2429x_charging_expired_alarm_cb);

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				bq2429x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2429x charger irq", bq);
		if (ret < 0) {
			pr_err("request irq for irq=%d failed, ret =%d\n", client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
		pr_info("interrupt register successfully!");
	}

	create_debugfs_entry(bq);

	pr_info("bq2429x probe successfully, Part Num:%d, Revision:%d\n!",
				bq->part_no, bq->revision);

	/*call explicitly to check adapter state, */
	bq2429x_charger_interrupt(client->irq, bq);

	create_smb_chg_proc_entry(bq);
	if (gpio_is_valid(bq->platform_data->otg_irq_gpio)) {
		int irq = 0;
		int rc = 0;

		rc = gpio_request(bq->platform_data->otg_irq_gpio, "ba2429x_otg_irq");
		if (rc) {
			dev_err(&client->dev,
					"irq gpio request failed, rc=%d", rc);
		} else {
			pr_info("gpio_request succes");
			rc = gpio_direction_input(bq->platform_data->otg_irq_gpio);

			irq = gpio_to_irq(bq->platform_data->otg_irq_gpio);
			if (irq < 0) {
				dev_err(&client->dev,
					"Invalid irq_gpio irq = %d\n", irq);
			} else {
				pr_info("gpio_to_irq succes");
				rc = devm_request_threaded_irq(&client->dev, irq, NULL,
						bq2429x_otg_handler,
						IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						"bq2429x_otg_handler", bq);
				if (rc) {
					dev_err(&client->dev,
						"Failed STAT irq=%d request rc = %d\n",
						irq, rc);
				} else {
					pr_info("devm_request_threaded_irq otg irq succes");
				}
			}
		}
		enable_irq_wake(irq);
	}

	if (charger_boot != true) {
		INIT_DELAYED_WORK(&bq->otg_boot_work, bq2429x_otg_boot_workfunc);
		schedule_delayed_work(&bq->otg_boot_work, msecs_to_jiffies(18000));
		pr_info("BOOT with OTG,OTG detect after 18s!\n");
	}

	charging_policy_init(bq);
	zte_misc_register_charging_policy_ops(&bq->battery_charging_policy_ops);
	schedule_delayed_work(&bq->charging_policy_work, 0);

	return 0;

err_1:
	bq2429x_psy_unregister(bq);
	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->current_change_lock);
	mutex_destroy(&bq->lock);
	devm_kfree(&client->dev, bq);

	return ret;
}

static int bq2429x_charger_remove(struct i2c_client *client)
{
	struct bq2429x *bq = i2c_get_clientdata(client);

	regulator_unregister(bq->otg_vreg.rdev);

	bq2429x_psy_unregister(bq);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->usb_type_detection_work);
	cancel_delayed_work_sync(&bq->dcp_icl_recheck_work);
	cancel_delayed_work_sync(&bq->charging_policy_work);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->current_change_lock);
	mutex_destroy(&bq->lock);
	mutex_destroy(&bq->irq_complete);
#ifdef CHG_SMOOTH_BATTERY_PROP
	mutex_destroy(&g_smooth_prop.mutex_smoothing_battery);
#endif
	debugfs_remove_recursive(bq->debug_root);

	return 0;
}


static void bq2429x_charger_shutdown(struct i2c_client *client)
{
	struct bq2429x *bq = i2c_get_clientdata(client);

	bq2429x_battery_charging(bq, true);
	power_supply_set_usb_otg(bq->usb_psy, 0);
	bq2429x_disable_otg(bq);
	msleep(100);

	pr_info("shutdown\n");
}

static int bq2429x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2429x *bq = i2c_get_clientdata(client);

	pr_debug("%s\n", __func__);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	return 0;
}

static int bq2429x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2429x *bq = i2c_get_clientdata(client);

	pr_debug("%s\n", __func__);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	mutex_unlock(&bq->irq_complete);

	if (bq->irq_waiting) {
		bq2429x_charger_interrupt(client->irq, bq);
	}
	if (bq->monitor_work_waiting) {
		pr_notice("%s monitor work in suspend, need perform\n", __func__);
		bq2429x_monitor(bq);
	}
	return 0;
}
static const struct dev_pm_ops bq2429x_pm_ops = {
	.suspend	= bq2429x_suspend,
	.resume		= bq2429x_resume,
};
static struct of_device_id bq2429x_charger_match_table[] = {
	{.compatible = "ti,charger,bq24295",},
	{.compatible = "ti,charger,bq24296",},
	{.compatible = "ti,charger,bq24297",},
	{},
};
MODULE_DEVICE_TABLE(of, bq2429x_charger_match_table);

static const struct i2c_device_id bq2429x_charger_id[] = {
	{ "bq24295-charger", BQ24295 },
	{ "bq24296-charger", BQ24296 },
	{ "bq24297-charger", BQ24297 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2429x_charger_id);

static struct i2c_driver bq2429x_charger_driver = {
	.driver		= {
		.name	= "bq2429x-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq2429x_charger_match_table,
		.pm		= &bq2429x_pm_ops,
	},
	.id_table	= bq2429x_charger_id,

	.probe		= bq2429x_charger_probe,
	.remove		= bq2429x_charger_remove,
	.shutdown	= bq2429x_charger_shutdown,
};

module_i2c_driver(bq2429x_charger_driver);

MODULE_DESCRIPTION("TI BQ2429x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");

