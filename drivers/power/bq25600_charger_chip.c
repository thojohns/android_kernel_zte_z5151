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
#define pr_fmt(fmt)	"BQ25600 %s: " fmt, __func__

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
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>

#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>

#include <linux/pinctrl/consumer.h>

#include <linux/pmic-voter.h>
#include <linux/alarmtimer.h>
#include "bq25600_reg.h"
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
	int usb_present;

	int chg_unplugged_time;
	bool pre_usb_present;

	int temps[CALC_AVG_NUMS];
	int voltages[CALC_AVG_NUMS];
	int volt_now;
	int current_now;
	int avg_volt;
	int avg_temp;

	bool overdischarge;

	s64 current_boot_sec;
	s64 pre_print_battinfo_sec;
	s64 pre_calc_avg_sec;

	int battery_smooth_dst_soc;

	struct delayed_work battery_smooth_up;
	struct delayed_work battery_smooth_down;
	struct mutex mutex_smoothing_battery;
	struct mutex mutex_smooth_up_down;
};

static struct battery_smooth_prop g_smooth_prop;
#endif

enum bq25600_vbus_type {
	BQ25600_VBUS_NONE = REG08_VBUS_TYPE_NONE,
	BQ25600_VBUS_USB = REG08_VBUS_TYPE_USB,
	BQ25600_VBUS_CDP = REG08_VBUS_TYPE_CDP,
	BQ25600_VBUS_DCP = REG08_VBUS_TYPE_DCP,
	BQ25600_VBUS_UNKNOWN = REG08_VBUS_TYPE_UNKNOWN,
	BQ25600_VBUS_NON_STANDARD = REG08_VBUS_TYPE_NON_STANDARD,
	BQ25600_VBUS_OTG = REG08_VBUS_TYPE_OTG,
};

enum bq25600_part_no {
	BQ25600 = 0x0000,
	BQ25600D = 0x0001,
};

enum {
	USER		= BIT(0),
	JEITA		= BIT(1),
	BATT_FC		= BIT(2),
	BATT_PRES	= BIT(3),
};

enum wakeup_src {
	WAKEUP_SRC_MONITOR = 0,
	WAKEUP_SRC_JEITA,
	WAKEUP_SRC_MAX,
};

#define WAKEUP_SRC_MASK (~(~0 << WAKEUP_SRC_MAX))
struct bq25600_wakeup_source {
	struct wakeup_source source;
	unsigned long enabled_bitmap;
	spinlock_t ws_lock;
};

enum bq25600_charge_state {
	CHARGE_STATE_IDLE = REG08_CHRG_STAT_IDLE,
	CHARGE_STATE_PRECHG = REG08_CHRG_STAT_PRECHG,
	CHARGE_STATE_FASTCHG = REG08_CHRG_STAT_FASTCHG,
	CHARGE_STATE_CHGDONE = REG08_CHRG_STAT_CHGDONE,
};

#define BATT_NORMAL_CHG_VOLT_VOTER "BATT_NORMAL_CHG_VOLT_VOTER"
#define BATT_JEITA_CHG_VOLT_VOTER "BATT_JEITA_CHG_VOLT_VOTER"
#define NUM_CHG_VOLT_VOTER 2

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

enum stat_ctrl {
	STAT_CTRL_STAT,
	STAT_CTRL_ICHG,
	STAT_CTRL_INDPM,
	STAT_CTRL_DISABLE,
};

enum vboost {
	BOOSTV_4850 = 4850,
	BOOSTV_5000 = 5000,
	BOOSTV_5150 = 5150,
	BOOSTV_5300	= 5300,
};

enum iboost {
	BOOSTI_500 = 500,
	BOOSTI_1200 = 1200,
};

enum vac_ovp {
	VAC_OVP_5500 = 5500,
	VAC_OVP_6500 = 6500,
	VAC_OVP_10500 = 10500,
	VAC_OVP_14000 = 14000,
};

struct bq25600_charge_param {
	int vlim;
	int ilim;
	int ichg;
	int vreg;
	int float_voltage_mv;
};

struct bq25600_platform_data {
	struct bq25600_charge_param usb;
	struct bq25600_charge_param dcp;
	struct bq25600_charge_param cdp;
	int iprechg;
	int iterm;
	int recharge_voltage_mv;

	enum stat_ctrl statctrl;
	enum vboost boostv;
	enum iboost boosti;
	enum vac_ovp vac_ovp;

	bool enable_term;
	int otg_enable_gpio;
	int otg_irq_gpio;
};

struct bq25600_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq25600 {
	struct device *dev;
	struct i2c_client *client;

	enum bq25600_part_no part_no;
	int	revision;

	int gpio_ce;

	int	vbus_type;
	int chg_type;
	int status;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex profile_change_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;
	struct mutex current_change_lock;

	struct bq25600_wakeup_source bq25600_ws;

	bool monitor_work_waiting;
	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool usb_present;

	bool batt_full;

	bool prop_set_charge_enabled;
	bool charge_enabled;/* Register bit status */
	bool otg_enabled;
	bool batfet_enabled;
	bool in_hiz;

	bool vindpm_triggered;
	bool iindpm_triggered;

	bool in_therm_regulation;
	bool in_vsys_regulation;

	bool power_good;
	bool vbus_good;

	bool topoff_active;
	bool acov_triggered;

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
	int batt_cool_degc_ext;
	int batt_cold_degc;
	int hot_temp_hysteresis;
	int cold_temp_hysteresis;

	int batt_cool_ma;
	int batt_cool_ma_ext;
	int batt_warm_ma;
	int batt_cool_mv;
	int batt_warm_mv;

	int batt_temp;

	int jeita_ma;
	int jeita_mv;

	int	usb_psy_ma;
	int charge_state;
	int charging_disabled_status;

	int fault_status;

	int skip_writes;
	int skip_reads;

	struct bq25600_platform_data *platform_data;

	struct delayed_work monitor_work;
	struct delayed_work otg_boot_work;
	struct delayed_work usb_type_detection_work;
	struct delayed_work dcp_icl_recheck_work;
	struct wake_lock usb_plug_in_out_wake_lock;
	struct alarm charging_expired_alarm;

	struct dentry *debug_root;
	unsigned int			peek_poke_address;
	struct bq25600_otg_regulator otg_vreg;
	struct regulator *vdd;
	int vdd_levels[3]; /* none, low, high */
	bool vdd_enabled;

	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply batt_psy;
	int	usb_dp_dm_status;
	unsigned int	thermal_levels;
	unsigned int	therm_lvl_sel;
	unsigned int	*thermal_mitigation;
	int	shipmode_enabled;
	struct wake_lock battery_suspend_lock;
	struct wake_lock smb_monitor_wake_lock;

	/* voters */
	struct votable *float_volt_votable;
	struct votable *fcc_votable;
	struct votable *usb_icl_votable;

	int psy_health_sts;
	bool input_unspec;
	bool need_recheck_usb_type;

	bool test_disable_feed_wdt;
	bool use_test_temp;
	int test_temp;

	bool chg_term_full_charge_detected;
	bool chg_term_by_soc;
	int chg_term_battery_status;
	struct charging_policy_ops battery_charging_policy_ops;
	struct delayed_work charging_policy_work;
};

static int BatteryTestStatus_enable = 0;
static bool charger_boot = false;
static struct bq25600 *g_bq = NULL;

static void bq25600_monitor_workfunc(struct work_struct *work);
static int bq25600_get_prop_battery_capacity(struct bq25600 *bq);

static int __bq25600_read_reg(struct bq25600 *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8)ret;

	return 0;
}

static int __bq25600_write_reg(struct bq25600 *bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq25600_read_byte(struct bq25600 *bq, u8 *data, u8 reg)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq25600_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}


static int bq25600_write_byte(struct bq25600 *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes) {
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq25600_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
	}

	return ret;
}


static int bq25600_update_bits(struct bq25600 *bq, u8 reg,
									u8 mask, u8 data)
{
	int ret;
	u8 tmp;


	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq25600_read_reg(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq25600_write_reg(bq, reg, tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
	}

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq25600_enable_otg(struct bq25600 *bq)
{
	u8 val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;

	return bq25600_update_bits(bq, BQ25600_REG_01,
							   REG01_OTG_CONFIG_MASK, val);

}

static int bq25600_disable_otg(struct bq25600 *bq)
{
	u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;

	return bq25600_update_bits(bq, BQ25600_REG_01,
							   REG01_OTG_CONFIG_MASK, val);
}

static int bq25600_enable_charger(struct bq25600 *bq)
{
	int ret;
	u8 val = REG01_CHG_ENABLE << REG01_CHG_CONFIG_SHIFT;

	ret = bq25600_update_bits(bq, BQ25600_REG_01, REG01_CHG_CONFIG_MASK, val);

	return ret;
}

static int bq25600_disable_charger(struct bq25600 *bq)
{
	int ret;
	u8 val = REG01_CHG_DISABLE << REG01_CHG_CONFIG_SHIFT;

	ret = bq25600_update_bits(bq, BQ25600_REG_01, REG01_CHG_CONFIG_MASK, val);
	return ret;
}

int bq25600_set_chargecurrent(struct bq25600 *bq, int curr)
{
	u8 ichg;

	ichg = (curr - REG02_ICHG_BASE)/REG02_ICHG_LSB;
	return bq25600_update_bits(bq, BQ25600_REG_02, REG02_ICHG_MASK,
							ichg << REG02_ICHG_SHIFT);

}

int bq25600_set_term_current(struct bq25600 *bq, int curr)
{
	u8 iterm;

	iterm = (curr - REG03_ITERM_BASE) / REG03_ITERM_LSB;

	return bq25600_update_bits(bq, BQ25600_REG_03, REG03_ITERM_MASK,
							iterm << REG03_ITERM_SHIFT);
}


int bq25600_set_prechg_current(struct bq25600 *bq, int curr)
{
	u8 iprechg;

	iprechg = (curr - REG03_IPRECHG_BASE) / REG03_IPRECHG_LSB;

	return bq25600_update_bits(bq, BQ25600_REG_03, REG03_IPRECHG_MASK,
							iprechg << REG03_IPRECHG_SHIFT);
}

int bq25600_set_chargevolt(struct bq25600 *bq, int volt)
{
	u8 val;

	val = (volt - REG04_VREG_BASE)/REG04_VREG_LSB;
	return bq25600_update_bits(bq, BQ25600_REG_04, REG04_VREG_MASK,
							val << REG04_VREG_SHIFT);
}

int bq25600_set_rechargevoltage(struct bq25600 *bq, int volt)
{
	u8 val = 0;

	if (volt > 100) {
		val = 1;
	}

	return bq25600_update_bits(bq, BQ25600_REG_04, REG04_VRECHG_MASK, val << REG04_VRECHG_SHIFT);
}

int bq25600_set_input_volt_limit(struct bq25600 *bq, int volt)
{
	u8 val;

	val = (volt - REG06_VINDPM_BASE) / REG06_VINDPM_LSB;
	return bq25600_update_bits(bq, BQ25600_REG_06, REG06_VINDPM_MASK,
							val << REG06_VINDPM_SHIFT);
}

int bq25600_set_input_current_limit(struct bq25600 *bq, int curr)
{
	u8 val;

	val = (curr - REG00_IINLIM_BASE) / REG00_IINLIM_LSB;
	return bq25600_update_bits(bq, BQ25600_REG_00, REG00_IINLIM_MASK,
							val << REG00_IINLIM_SHIFT);
}

int bq25600_set_watchdog_timer(struct bq25600 *bq, u8 timeout)
{
	u8 temp;

	temp = (u8)(((timeout - REG05_WDT_BASE) / REG05_WDT_LSB) << REG05_WDT_SHIFT);

	return bq25600_update_bits(bq, BQ25600_REG_05, REG05_WDT_MASK, temp);
}
EXPORT_SYMBOL_GPL(bq25600_set_watchdog_timer);

int bq25600_disable_watchdog_timer(struct bq25600 *bq)
{
	u8 val = REG05_WDT_DISABLE << REG05_WDT_SHIFT;

	return bq25600_update_bits(bq, BQ25600_REG_05, REG05_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq25600_disable_watchdog_timer);

static int bq25600_enable_safety_timer(struct bq25600 *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG05_CHG_TIMER_ENABLE << REG05_EN_TIMER_SHIFT;
	else
		val = REG05_CHG_TIMER_DISABLE << REG05_EN_TIMER_SHIFT;

	ret = bq25600_update_bits(bq, BQ25600_REG_05, REG05_EN_TIMER_MASK, val);

	return ret;
}

static int bq25600_get_safety_timer_state(struct bq25600 *bq)
{
	u8 val = 0;
	int ret = 0;

	ret = bq25600_read_byte(bq, &val, BQ25600_REG_05);
	if (ret)
		return ret;
	ret = (val & REG05_EN_TIMER_MASK) >> REG05_EN_TIMER_SHIFT;

	return ret;
}

int bq25600_get_watchdog_timer_state(struct bq25600 *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq25600_read_byte(bq, &val, BQ25600_REG_09);
	if (ret)
		return ret;
	*state = (val & REG09_FAULT_WDT_MASK) >> REG09_FAULT_WDT_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq25600_get_watchdog_timer_state);

int bq25600_get_chg_timer(struct bq25600 *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq25600_read_byte(bq, &val, BQ25600_REG_05);
	if (ret)
		return ret;
	*state = (val & REG05_CHG_TIMER_MASK) >> REG05_CHG_TIMER_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq25600_get_chg_timer);


int bq25600_reset_watchdog_timer(struct bq25600 *bq)
{
	u8 val = REG01_WDT_RESET << REG01_WDT_RESET_SHIFT;

	return bq25600_update_bits(bq, BQ25600_REG_01, REG01_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq25600_reset_watchdog_timer);

int bq25600_reset_chip(struct bq25600 *bq)
{
	int ret;
	u8 val = REG0B_REG_RESET << REG0B_REG_RESET_SHIFT;

	ret = bq25600_update_bits(bq, BQ25600_REG_0B, REG0B_REG_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq25600_reset_chip);

int bq25600_enter_hiz_mode(struct bq25600 *bq)
{
	u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

	return bq25600_update_bits(bq, BQ25600_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq25600_enter_hiz_mode);

int bq25600_exit_hiz_mode(struct bq25600 *bq)
{

	u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

	return bq25600_update_bits(bq, BQ25600_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq25600_exit_hiz_mode);

int bq25600_get_hiz_mode(struct bq25600 *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq25600_read_byte(bq, &val, BQ25600_REG_00);
	if (ret)
		return ret;
	*state = (val & REG00_ENHIZ_MASK) >> REG00_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq25600_get_hiz_mode);


static int bq25600_enable_term(struct bq25600 *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG05_TERM_ENABLE << REG05_EN_TERM_SHIFT;
	else
		val = REG05_TERM_DISABLE << REG05_EN_TERM_SHIFT;

	ret = bq25600_update_bits(bq, BQ25600_REG_05, REG05_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq25600_enable_term);

int bq25600_set_boost_current(struct bq25600 *bq, int curr)
{
	u8 val;

	val = REG02_BOOST_LIM_0P5A;
	if (curr == BOOSTI_1200)
		val = REG02_BOOST_LIM_1P2A;

	return bq25600_update_bits(bq, BQ25600_REG_02, REG02_BOOST_LIM_MASK,
							val << REG02_BOOST_LIM_SHIFT);
}

int bq25600_set_boost_voltage(struct bq25600 *bq, int volt)
{
	u8 val;

	if (volt == BOOSTV_4850)
		val = REG06_BOOSTV_4P85V;
	else if (volt == BOOSTV_5000)
		val = REG06_BOOSTV_5V;
	else if (volt == BOOSTV_5150)
		val = REG06_BOOSTV_5P15V;
	else if (volt == BOOSTV_5300)
		val = REG06_BOOSTV_5P3V;

	return bq25600_update_bits(bq, BQ25600_REG_06, REG06_BOOSTV_MASK,
							val << REG06_BOOSTV_SHIFT);
}

static int bq25600_set_acovp_threshold(struct bq25600 *bq, int volt)
{
	u8 val;

	if (volt >= VAC_OVP_14000)
		val = REG06_OVP_14V;
	else if (volt >= VAC_OVP_10500)
		val = REG06_OVP_10P5V;
	else if (volt >= VAC_OVP_6500)
		val = REG06_OVP_6P5V;
	else
		val = REG06_OVP_5P5V;

	return bq25600_update_bits(bq, BQ25600_REG_06, REG06_OVP_MASK,
							val << REG06_OVP_SHIFT);
}


static int bq25600_set_stat_ctrl(struct bq25600 *bq, int ctrl)
{
	u8 val;

	val = ctrl;

	return bq25600_update_bits(bq, BQ25600_REG_00, REG00_STAT_CTRL_MASK,
							val << REG00_STAT_CTRL_SHIFT);
}


static int bq25600_set_int_mask(struct bq25600 *bq, int mask)
{
	u8 val;

	val = mask;

	return bq25600_update_bits(bq, BQ25600_REG_0A, REG0A_INT_MASK_MASK,
							val << REG0A_INT_MASK_SHIFT);
}


static int bq25600_enable_batfet(struct bq25600 *bq)
{
	const u8 val = REG07_BATFET_ON << REG07_BATFET_DIS_SHIFT;

	return bq25600_update_bits(bq, BQ25600_REG_07, REG07_BATFET_DIS_MASK,
								val);
}
EXPORT_SYMBOL_GPL(bq25600_enable_batfet);


static int bq25600_disable_batfet(struct bq25600 *bq)
{
	const u8 val = REG07_BATFET_OFF << REG07_BATFET_DIS_SHIFT;

	return bq25600_update_bits(bq, BQ25600_REG_07, REG07_BATFET_DIS_MASK,
								val);
}
EXPORT_SYMBOL_GPL(bq25600_disable_batfet);

static int bq25600_set_batfet_delay(struct bq25600 *bq, uint8_t delay)
{
	u8 val;

	if (delay == 0)
		val = REG07_BATFET_DLY_0S;
	else
		val = REG07_BATFET_DLY_10S;

	val <<= REG07_BATFET_DLY_SHIFT;

	return bq25600_update_bits(bq, BQ25600_REG_07, REG07_BATFET_DLY_MASK,
								val);
}
EXPORT_SYMBOL_GPL(bq25600_set_batfet_delay);

static int bq25600_charging_disable(struct bq25600 *bq, int reason,
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
		ret = bq25600_disable_charger(bq);
	else if (!disabled && !bq->charge_enabled)
		ret = bq25600_enable_charger(bq);

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


static struct power_supply *get_bms_psy(struct bq25600 *bq)
{
	if (bq->bms_psy)
		return bq->bms_psy;
	bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->bms_psy)
		pr_debug("bms power supply not found\n");

	return bq->bms_psy;
}

static int bq25600_get_batt_property(struct bq25600 *bq,
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

static void bq25600_set_shipmode_enable(struct bq25600 *bq, bool enable)
{
	bq25600_exit_hiz_mode(bq);	/*disable hiz mode */

	pr_info("enable = %d\n", enable);

	if (enable == 0) {
		bq25600_disable_watchdog_timer(bq);		/*disable watchdog time.*/
		bq25600_disable_batfet(bq);	/*disable BATFET*/
		bq->shipmode_enabled  = 1;
	} else {
		bq25600_set_watchdog_timer(bq, 80);		/*enable watchdog time.*/
		bq25600_enable_batfet(bq);	/*enable BATFET*/
		bq->shipmode_enabled  = 0;
	}
}

static int bq25600_set_dp_dm_detection_enable(struct bq25600 *bq)
{
	int ret;

	ret = bq25600_update_bits(bq, BQ25600_REG_07, REG07_FORCE_DPDM_MASK,
								REG07_FORCE_DPDM << REG07_FORCE_DPDM_SHIFT);

	return ret;
}

static int bq25600_set_batfet_reset_enable(struct bq25600 *bq, bool enable)
{
	int ret;
	int val;

	if (enable)
		val = REG07_BATFET_RST_ENABLE;
	else
		val = REG07_BATFET_RST_DISABLE;

	ret = bq25600_update_bits(bq, BQ25600_REG_07, REG07_BATFET_RST_EN_MASK,
								val << REG07_BATFET_RST_EN_SHIFT);

	return ret;
}

static int bq25600_battery_discharging(struct bq25600 *bq)
{
	int ret;

	/*if (bq->battery_charging_policy_ops.battery_status != BATTERY_DISCHARGING) {*/
		ret = vote(bq->fcc_votable, CHARGING_POLICY_FCC_VOTER, true, 0);
		if (ret < 0)
			pr_err("%s,charging current limit: CHARGING_POLICY_FCC_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		/* ensure the vote takes  effect, so do twice different votes */
		ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, true, REG00_IINLIM_BASE + REG00_IINLIM_LSB);
		ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, true, REG00_IINLIM_BASE);
		if (ret < 0)
			pr_err("%s,input current limit: CHARGING_POLICY_ICL_VOTER couldn't vote, ret=%d\n",
				__func__, ret);
		bq->battery_charging_policy_ops.battery_status = BATTERY_DISCHARGING;
	/*}*/
	return 0;
}

static int bq25600_battery_not_charging(struct bq25600 *bq)
{
	int ret;

	if (bq->battery_charging_policy_ops.battery_status != BATTERY_NOT_CHARGING) {
		ret = vote(bq->fcc_votable, CHARGING_POLICY_FCC_VOTER, true, 0);
		if (ret < 0)
			pr_err("%s,charging current limit: CHARGING_POLICY_FCC_VOTER couldn't vote, ret=%d\n",
				__func__, ret);

		ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, false, 0);
		if (ret < 0)
			pr_err("%s,input current limit: CHARGING_POLICY_ICL_VOTER couldn't vote, ret=%d\n",
				__func__, ret);
		bq->battery_charging_policy_ops.battery_status = BATTERY_NOT_CHARGING;
	}
	return 0;
}

static int bq25600_battery_charging(struct bq25600 *bq, bool force)
{
	int ret;

	if (bq->battery_charging_policy_ops.battery_status != BATTERY_CHARGING) {
		if ((!bq->batt_hot) && (!bq->batt_cold)) {
			ret = vote(bq->fcc_votable, CHARGING_POLICY_FCC_VOTER, false, 0);
			if (ret < 0)
				pr_err("%s,charging current limit: CHARGING_POLICY_FCC_VOTER couldn't vote, ret=%d\n",
					__func__, ret);

			ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, false, 0);
			if (ret < 0)
				pr_err("%s,input current limit: CHARGING_POLICY_ICL_VOTER couldn't vote, ret=%d\n",
					__func__, ret);
		}
		bq->battery_charging_policy_ops.battery_status = BATTERY_CHARGING;
	}

	if (force) {
		if ((!bq->batt_hot) && (!bq->batt_cold)) {
			ret = vote(bq->fcc_votable, CHARGING_POLICY_FCC_VOTER, false, 0);
			if (ret < 0)
				pr_err("%s,charging current limit: CHARGING_POLICY_FCC_VOTER couldn't vote, ret=%d\n",
					__func__, ret);

			ret = vote(bq->usb_icl_votable, CHARGING_POLICY_ICL_VOTER, false, 0);
			if (ret < 0)
				pr_err("%s,input current limit: CHARGING_POLICY_ICL_VOTER couldn't vote, ret=%d\n",
					__func__, ret);
		}
		bq->battery_charging_policy_ops.battery_status = BATTERY_CHARGING;
		bq->battery_charging_policy_ops.charging_policy_status = DEFAULT_CHARGING_POLICY;
	}
	return 0;
}

static int bq25600_battery_clean_charging_expired(struct bq25600 *bq)
{
	bq->battery_charging_policy_ops.charging_policy_status &= ~EXPIRED_CHARGING_POLICY;

	return 0;
}

static int bq25600_battery_charging_policy_check(struct bq25600 *bq)
{
	int soc = bq25600_get_prop_battery_capacity(bq);

	pr_info("charging_policy_status = 0x%x, battery_status = %d, soc = %d\n",
		bq->battery_charging_policy_ops.charging_policy_status,
		bq->battery_charging_policy_ops.battery_status, soc);
	if ((bq->battery_charging_policy_ops.charging_policy_status & DEMO_CHARGING_POLICY)
		|| (bq->battery_charging_policy_ops.charging_policy_status & EXPIRED_CHARGING_POLICY)) {
		if (soc > MAX_BATTERY_PROTECTED_PERCENT) {
			bq25600_battery_discharging(bq);
		} else if (soc == MAX_BATTERY_PROTECTED_PERCENT) {
			bq25600_battery_not_charging(bq);
		} else if (soc < MAX_BATTERY_PROTECTED_PERCENT) {
			bq25600_battery_charging(bq, false);
		}
	} else {
		if (bq->prop_set_charge_enabled)
			bq25600_battery_charging(bq, true);
	}

	return 0;
}

static void bq25600_charging_policy_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq25600 *bq = container_of(dwork, struct bq25600, charging_policy_work);

	pr_info("\n");
	bq25600_battery_charging_policy_check(bq);
	power_supply_changed(&bq->batt_psy);
}

static enum alarmtimer_restart bq25600_charging_expired_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	#if (EXPIRED_CHARGING_POLICY_ENABLE == 1)
	struct bq25600 *bq = container_of(alarm, struct bq25600,
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
	struct bq25600 *bq = container_of(charging_policy, struct bq25600,
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
	struct bq25600 *bq = container_of(charging_policy, struct bq25600,
					battery_charging_policy_ops);

	pr_info("charging_policy_status = 0x%x\n", bq->battery_charging_policy_ops.charging_policy_status);

	if ((bq->battery_charging_policy_ops.charging_policy_status & DEMO_CHARGING_POLICY)
		== DEMO_CHARGING_POLICY)
		return 1;
	else
		return 0;
}

static int bc_policy_expired_sts_get(struct charging_policy_ops *charging_policy)
{
	struct bq25600 *bq = container_of(charging_policy, struct bq25600,
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

static int charging_policy_init(struct bq25600 *bq)
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

static int bq25600_usb_phy_init(struct device *dev, struct bq25600 *bq)
{
	int rc;

	rc = of_property_read_u32_array(dev->of_node, "qcom,vdd-voltage-level",
					 (u32 *) bq->vdd_levels,
					 ARRAY_SIZE(bq->vdd_levels));
	if (rc) {
		pr_err("error reading qcom,vdd-voltage-level property\n");
	}

	bq->vdd = regulator_get(&bq->client->dev, "vdd");
	if (IS_ERR(bq->vdd)) {
		pr_err("Regulator get failed vdd rc=%d", rc);
	}

	bq->vdd_enabled = false;

	if (!bq->vdd_enabled) {
		rc = regulator_set_voltage(bq->vdd, bq->vdd_levels[1], bq->vdd_levels[2]);
		if (rc) {
			pr_err("Regulator set_vtg failed vdd rc=%d", rc);
		}
		rc = regulator_enable(bq->vdd);
		if (rc) {
			pr_err("Regulator vdd enable failed rc=%d", rc);
		} else {
			bq->vdd_enabled = true;
		}
	}

	return 0;
}

static int bq25600_usb_phy_ctrl(struct bq25600 *bq, bool enable)
{
	int rc;

	pr_info("enable : %s, enabled : %s now\n",
		enable ? "yes" : "no", bq->vdd_enabled ? "yes" : "no");
	if (enable && !bq->vdd_enabled) {
		rc = regulator_set_voltage(bq->vdd, bq->vdd_levels[1], bq->vdd_levels[2]);
		if (rc) {
			pr_err("Regulator set_vtg failed vdd rc=%d", rc);
		}
		rc = regulator_enable(bq->vdd);
		if (rc) {
			pr_err("Regulator vdd enable failed rc=%d", rc);
		}
		bq->vdd_enabled = true;
	} else if (!enable && bq->vdd_enabled) {
		rc = regulator_disable(bq->vdd);
		if (rc) {
			pr_err("Regulator vdd disable failed rc=%d", rc);
		}
		rc = regulator_set_voltage(bq->vdd, bq->vdd_levels[0], bq->vdd_levels[2]);
		if (rc) {
			pr_err("Regulator set_vtg failed vdd rc=%d", rc);
		}
		bq->vdd_enabled = false;
	}

	return 0;
}

static inline bool is_device_suspended(struct bq25600 *bq);
static int bq25600_get_prop_charge_type(struct bq25600 *bq)
{
	u8 val = 0;

	if (is_device_suspended(bq))
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	bq25600_read_byte(bq, &val, BQ25600_REG_08);
	val &= REG08_CHRG_STAT_MASK;
	val >>= REG08_CHRG_STAT_SHIFT;
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

#define DEFAULT_BATT_CHARGE_COUNTER 1
static int bq25600_get_prop_battery_charge_counter(struct bq25600 *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq25600_get_batt_property(bq, POWER_SUPPLY_PROP_CHARGE_COUNTER, &batt_prop);
	if (ret || batt_prop.intval == 0) {
		pr_err("Couldn't get charge counter rc = %d\n", batt_prop.intval);
		batt_prop.intval = DEFAULT_BATT_CHARGE_COUNTER;
	}
	return batt_prop.intval;
}

static int bq25600_get_prop_batt_present(struct bq25600 *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq25600_get_batt_property(bq, POWER_SUPPLY_PROP_PRESENT, &batt_prop);
	if (!ret)
		bq->batt_present = batt_prop.intval;

	return ret;
}

static int bq25600_get_prop_batt_full(struct bq25600 *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq25600_get_batt_property(bq, POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (!ret)
		bq->batt_full = (batt_prop.intval == POWER_SUPPLY_STATUS_FULL);

	return ret;
}

#define DEFAULT_BATT_VOLTAGE_NOW 4000
static int bq25600_get_prop_battery_voltage_now(struct bq25600 *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq25600_get_batt_property(bq, POWER_SUPPLY_PROP_VOLTAGE_NOW, &batt_prop);
	if (ret) {
		pr_err("Couldn't get voltage rc = %d\n", batt_prop.intval);
		batt_prop.intval = DEFAULT_BATT_VOLTAGE_NOW;
	}
	return batt_prop.intval;
}

#define DEFAULT_BATT_CURRENT_NOW 0
static int bq25600_get_prop_battery_current_now(struct bq25600 *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq25600_get_batt_property(bq, POWER_SUPPLY_PROP_CURRENT_NOW, &batt_prop);
	if (ret) {
		pr_err("Couldn't get voltage rc = %d\n", batt_prop.intval);
		batt_prop.intval = DEFAULT_BATT_CURRENT_NOW;
	}
	return batt_prop.intval;
}

#define DEFAULT_BATT_CAPACITY 50
static int bq25600_get_prop_battery_capacity(struct bq25600 *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq25600_get_batt_property(bq, POWER_SUPPLY_PROP_CAPACITY, &batt_prop);
	if (ret) {
		pr_err("Couldn't get voltage rc = %d\n", batt_prop.intval);
		batt_prop.intval = DEFAULT_BATT_CAPACITY;
	}
	return batt_prop.intval;
}

#define DEFAULT_BATT_TEMP 250
static int bq25600_get_prop_battery_temp(struct bq25600 *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;

	ret = bq25600_get_batt_property(bq, POWER_SUPPLY_PROP_TEMP, &batt_prop);
	if (ret) {
		pr_err("Couldn't get voltage rc = %d\n", batt_prop.intval);
		batt_prop.intval = DEFAULT_BATT_TEMP;
	}
	if (bq->use_test_temp) {
		dev_notice(bq->dev, "use temprature: %d\n", bq->test_temp);
		batt_prop.intval = bq->test_temp;
	}
	return batt_prop.intval;
}
#define DEFAULT_CHG_TYPE 0
static int bq25600_get_chg_type(struct bq25600 *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret = 0;

	ret = bq->usb_psy->get_property(bq->usb_psy, POWER_SUPPLY_PROP_TYPE, &batt_prop);

	if (ret) {
		pr_err("Couldn't get USB TYPE rc = %d\n", ret);
		batt_prop.intval = DEFAULT_CHG_TYPE;
	}
	return batt_prop.intval;
}

static int bq25600_get_prop_charge_status(struct bq25600 *bq)
{
	union power_supply_propval batt_prop = {0,};
	int ret;
	u8 status;

	ret = bq25600_get_batt_property(bq, POWER_SUPPLY_PROP_STATUS, &batt_prop);
	if (bq->chg_term_by_soc && ((bq->chg_term_battery_status == POWER_SUPPLY_STATUS_FULL)
		|| (!ret && batt_prop.intval == POWER_SUPPLY_STATUS_FULL))) {
		return POWER_SUPPLY_STATUS_FULL;
	}

	ret = bq25600_read_byte(bq, &status, BQ25600_REG_08);
	if (ret) {
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	mutex_lock(&bq->data_lock);
	bq->charge_state = (status & REG08_CHRG_STAT_MASK) >> REG08_CHRG_STAT_SHIFT;
	mutex_unlock(&bq->data_lock);

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

static int bq25600_get_prop_health(struct bq25600 *bq)
{
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

	return ret.intval;
}

static int bq25600_get_prop_batt_status(struct bq25600 *bq)
{
	u8 val = 0;

	bq25600_read_byte(bq, &val, BQ25600_REG_08);
	val &= REG08_CHRG_STAT_MASK;
	val >>= REG08_CHRG_STAT_SHIFT;

	switch (val) {
	case CHARGE_STATE_FASTCHG:
	case CHARGE_STATE_PRECHG:
		return POWER_SUPPLY_STATUS_CHARGING;
	case CHARGE_STATE_CHGDONE:
		if (bq25600_get_prop_health(bq) == POWER_SUPPLY_HEALTH_GOOD)
			return POWER_SUPPLY_STATUS_FULL;
		else
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case CHARGE_STATE_IDLE:
		if (!bq->usb_present)
			return POWER_SUPPLY_STATUS_DISCHARGING;
		else
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static enum power_supply_property bq25600_charger_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	/* POWER_SUPPLY_PROP_TIME_TO_EMPTY, */
	POWER_SUPPLY_PROP_CHARGE_FULL,
	/* POWER_SUPPLY_PROP_CYCLE_COUNT, */
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_SHIPMODE,
};

static int bq25600_update_charging_profile(struct bq25600 *bq);
static int bq25600_system_temp_level_set(struct bq25600 *bq, int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;
	int therm_ma;

	if (!bq->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}
	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}
	if (lvl_sel >= bq->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel, bq->thermal_levels - 1);
		lvl_sel = bq->thermal_levels - 1;
	}
	if (lvl_sel == bq->therm_lvl_sel)
		return 0;
	mutex_lock(&bq->current_change_lock);
	prev_therm_lvl = bq->therm_lvl_sel;
	bq->therm_lvl_sel = lvl_sel;
	mutex_unlock(&bq->current_change_lock);

	pr_notice("system temp level:%d set current:%d ma.\n", lvl_sel, bq->thermal_mitigation[bq->therm_lvl_sel]);

	if (bq->therm_lvl_sel >= 0 && bq->therm_lvl_sel <= (bq->thermal_levels - 1)) {
		therm_ma = bq->thermal_mitigation[bq->therm_lvl_sel];

		rc = vote(bq->usb_icl_votable, PA_THERMAL_ICL_VOTER, true, therm_ma);
		if (rc < 0)
			pr_err("input current limit:PA_THERMAL_ICL_VOTER couldn't vote, ret=%d\n", rc);
	}

	return rc;
}

void static runin_work(struct bq25600 *bq, int batt_capacity)
{
	int rc;

	pr_debug("BatteryTestStatus_enable = %d bq->usb_present = %d\n",
			BatteryTestStatus_enable, bq->usb_present);

	if (/*!bq->usb_present || */!BatteryTestStatus_enable) {
		if (bq->in_hiz) {
			rc = bq25600_exit_hiz_mode(bq);
			if (rc) {
				dev_err(bq->dev, "Couldn't enable charge rc=%d\n", rc);
			} else {
				pr_info("Exit Hiz Successfully\n");
				bq->in_hiz = false;
			}
		}
		return;
	}

	if (batt_capacity >= 80) {
		pr_debug("soc >= 80\n");
		/*rc = bq25600_charging_disable(bq, USER, true);*/
		if (!bq->in_hiz) {
			rc = bq25600_enter_hiz_mode(bq);
			if (rc) {
				dev_err(bq->dev, "Couldn't disenable charge rc=%d\n", rc);
			} else {
				pr_info("Enter Hiz Successfully\n");
				bq->in_hiz = true;
			}
		}
	} else if (batt_capacity < 60) {
		pr_debug("bq25600_get_prop_batt_capacity < 60\n");
		/*rc = bq25600_charging_disable(bq, USER, false);*/
		if (bq->in_hiz) {
			rc = bq25600_exit_hiz_mode(bq);
			if (rc) {
				dev_err(bq->dev, "Couldn't enable charge rc=%d\n", rc);
			} else {
				pr_info("Exit Hiz Successfully\n");
				bq->in_hiz = false;
			}
		}
	}
}

static int bq25600_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{

	struct bq25600 *bq = container_of(psy, struct bq25600, batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (bq->vbus_type != BQ25600_VBUS_NONE && bq->vbus_type != BQ25600_VBUS_OTG)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq25600_get_prop_charge_type(bq);
		pr_debug("POWER_SUPPLY_PROP_CHARGE_TYPE:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		bq25600_get_batt_property(bq, psp, val);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq25600_get_prop_batt_status(bq);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq25600_get_prop_health(bq);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CHG_SMOOTH_BATTERY_PROP
		val->intval = g_smooth_prop.ui_soc;
#else
		val->intval = bq25600_get_prop_batt_capacity(bq);
#endif
		runin_work(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CHG_SMOOTH_BATTERY_PROP
		val->intval = g_smooth_prop.avg_volt;
#else
		val->intval = bq25600_get_prop_battery_voltage_now(bq);
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq25600_get_prop_battery_current_now(bq);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		return bq25600_get_batt_property(bq, psp, val);
	case POWER_SUPPLY_PROP_TEMP:
#ifdef CHG_SMOOTH_BATTERY_PROP
		val->intval = g_smooth_prop.avg_temp;
#else
		val->intval = bq25600_get_prop_battery_temp(bq);
#endif
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		return bq25600_get_batt_property(bq, psp, val);
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = bq->prop_set_charge_enabled;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = bq->usb_dp_dm_status;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = bq->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "BQ25600";
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = bq25600_get_prop_battery_charge_counter(bq);
		break;
	case POWER_SUPPLY_PROP_SHIPMODE:
		val->intval = bq->shipmode_enabled;
		break;
	default:
		return -EINVAL;

	}

	return 0;
}

static int bq25600_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq25600 *bq = container_of(psy, struct bq25600, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq25600_charging_disable(bq, USER, !val->intval);
		bq->prop_set_charge_enabled = !!val->intval;
		if (val->intval == 0) {
			if (bq->battery_suspend_lock.ws.active == true) {
				wake_unlock(&bq->battery_suspend_lock);
			}
			bq25600_disable_watchdog_timer(bq);
			bq25600_usb_phy_ctrl(bq, false);
		} else {
			bq25600_usb_phy_ctrl(bq, true);
		}
		power_supply_changed(&bq->batt_psy);
		power_supply_changed(bq->usb_psy);
		pr_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
						val->intval ? "enable" : "disable");
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
			pr_notice("usb offline\n");
			if (bq->battery_suspend_lock.ws.active == true) {
				wake_unlock(&bq->battery_suspend_lock);
			}
			power_supply_set_online(bq->usb_psy, 0);
			power_supply_set_present(bq->usb_psy, 0);
			break;
		default:
			break;
		}
	case POWER_SUPPLY_PROP_SHIPMODE:
		bq25600_set_shipmode_enable(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		bq25600_system_temp_level_set(bq, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25600_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_SHIPMODE:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_DP_DM:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int bq25600_update_charging_profile(struct bq25600 *bq)
{
	int ret;
	int chg_ma = 0;
	int chg_mv = 0;
	int icl_ma, icl_mv;
	union power_supply_propval prop = {0,};

	ret = bq->usb_psy->get_property(bq->usb_psy, POWER_SUPPLY_PROP_TYPE, &prop);
	if (ret < 0) {
		pr_err("couldn't read USB TYPE property, ret=%d\n", ret);
		return ret;
	}
	bq->chg_type = prop.intval;

	mutex_lock(&bq->profile_change_lock);

	if (prop.intval == POWER_SUPPLY_TYPE_USB_DCP) {
		chg_ma = bq->platform_data->dcp.ichg;
		chg_mv = bq->platform_data->dcp.vreg;
		icl_ma = bq->platform_data->dcp.ilim;
		icl_mv = bq->platform_data->dcp.vlim;
		cancel_delayed_work_sync(&bq->dcp_icl_recheck_work);
		schedule_delayed_work(&bq->dcp_icl_recheck_work, msecs_to_jiffies(5000));
	} else if (prop.intval == POWER_SUPPLY_TYPE_USB_CDP) {
		chg_ma = bq->platform_data->cdp.ichg;
		chg_mv = bq->platform_data->cdp.vreg;
		icl_ma = bq->platform_data->cdp.ilim;
		icl_mv = bq->platform_data->cdp.vlim;
	} else {
		chg_ma = bq->platform_data->usb.ichg;
		chg_mv = bq->platform_data->usb.vreg;
		icl_ma = bq->platform_data->usb.ilim;
		icl_mv = bq->platform_data->usb.vlim;
	}

	ret = vote(bq->fcc_votable, CHARGER_TYPE_FCC_VOTER, true, chg_ma);
	if (ret < 0)
		pr_err("charging current limit:CHARGER_TYPE_FCC_VOTER couldn't vote\n");

	ret = vote(bq->usb_icl_votable, CHARGER_TYPE_ICL_VOTER, true, icl_ma);
	if (ret < 0)
		pr_err("input current limit:CHARGER_TYPE_ICL_VOTER couldn't vote\n");

	mutex_unlock(&bq->profile_change_lock);

	return 0;
}

static void bq25600_external_power_changed(struct power_supply *psy)
{
	struct bq25600 *bq = container_of(psy, struct bq25600, batt_psy);

	union power_supply_propval prop = {0,};
	int ret, current_limit = 0;

	cancel_delayed_work_sync(&bq->monitor_work);
	bq25600_monitor_workfunc(&bq->monitor_work.work);

	ret = bq->usb_psy->get_property(bq->usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
		pr_err("could not read USB current_max property, ret=%d\n", ret);
	else
		current_limit = prop.intval / 1000;

	pr_info("current_limit = %d, bq->usb_psy_ma = %d\n", current_limit, bq->usb_psy_ma);

	if (bq->need_recheck_usb_type && current_limit != 0 && current_limit != 501) {
		cancel_delayed_work_sync(&bq->usb_type_detection_work);
		bq->need_recheck_usb_type = false;
		pr_info("cancel usb_type_detection_work.\n");
	}

	if (current_limit == 501) {
		cancel_delayed_work_sync(&bq->usb_type_detection_work);
		schedule_delayed_work(&bq->usb_type_detection_work, msecs_to_jiffies(3000));
		bq->need_recheck_usb_type = true;
		pr_info("usb_type_detection_work in 3.\n");
	}

	if (bq->usb_psy_ma != current_limit) {
		mutex_lock(&bq->current_change_lock);
		bq->usb_psy_ma = current_limit;
		bq25600_update_charging_profile(bq);
		mutex_unlock(&bq->current_change_lock);
	}

	ret = bq->usb_psy->get_property(bq->usb_psy, POWER_SUPPLY_PROP_ONLINE, &prop);
	if (ret < 0)
		pr_err("could not read USB ONLINE property, ret=%d\n", ret);

	ret = 0;
	bq25600_get_prop_charge_status(bq);
	if (bq->usb_present) {
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

}

static int bq25600_psy_register(struct bq25600 *bq)
{
	int ret;

	bq->batt_psy.name = "battery";
	bq->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->batt_psy.properties = bq25600_charger_props;
	bq->batt_psy.num_properties = ARRAY_SIZE(bq25600_charger_props);
	bq->batt_psy.get_property = bq25600_charger_get_property;
	bq->batt_psy.set_property = bq25600_charger_set_property;
	bq->batt_psy.external_power_changed = bq25600_external_power_changed;
	bq->batt_psy.property_is_writeable = bq25600_charger_is_writeable;

	ret = power_supply_register(bq->dev, &bq->batt_psy);
	if (ret < 0) {
		pr_err("failed to register batt_psy:%d\n", ret);
		return ret;
	}

	return 0;
}

static void bq25600_psy_unregister(struct bq25600 *bq)
{
	power_supply_unregister(&bq->batt_psy);
}

static int bq25600_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq25600 *bq = rdev_get_drvdata(rdev);

	ret = bq25600_enable_otg(bq);
	if (ret) {
		pr_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq->otg_enabled = true;
		pr_info("OTG mode Enabled!\n");
	}

	return ret;
}

static int bq25600_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq25600 *bq = rdev_get_drvdata(rdev);

	ret = bq25600_disable_otg(bq);
	if (ret) {
		pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		bq->otg_enabled = false;
		pr_info("OTG mode Disabled\n");
	}

	return ret;
}

static int bq25600_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret;
	u8 status;
	u8 enabled;
	struct bq25600 *bq = rdev_get_drvdata(rdev);

	ret = bq25600_read_byte(bq, &status, BQ25600_REG_01);
	if (ret)
		return ret;
	enabled = ((status & REG01_OTG_CONFIG_MASK) >> REG01_OTG_CONFIG_SHIFT);

	return (enabled == REG01_OTG_ENABLE) ? 1 : 0;

}

struct regulator_ops bq25600_otg_reg_ops = {
	.enable		= bq25600_otg_regulator_enable,
	.disable	= bq25600_otg_regulator_disable,
	.is_enabled = bq25600_otg_regulator_is_enable,
};

static int bq25600_regulator_init_chip(struct bq25600 *bq)
{
	int rc = 0;
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *regulator_node;

	pr_debug("\n");

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
		bq->otg_vreg.rdesc.ops = &bq25600_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		pr_info("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = regulator_node;

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

	pr_debug("return :%d.\n", rc);
	return rc;
}

static int bq25600_regulator_init(struct bq25600 *bq)
{
	return bq25600_regulator_init_chip(bq);
}

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *smb_chg_proc_entry;

static ssize_t smb_chg_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	struct bq25600 *bq = PDE_DATA(file_inode(filp));
	char message[256];
	int reg, val;
	int copy_len;

	if (len > 256)
		copy_len = 255;

	if (copy_from_user(message, buff, copy_len))
		return -EFAULT;
	message[copy_len] = '\0';

	pr_notice("%s\n", message);

	if (strnstr(message, "write", sizeof("write"))) {
		if (sscanf(&message[6], "%x %x", &reg, &val) != 1) {
			pr_info("sscanf() != 1\n");
		}
		bq25600_write_byte(bq, reg, val);
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
	struct bq25600 *bq = seq->private;
	u8 status[12];
	int i = 0;
	int ret = -1;

	pr_notice("\n");

	for (i = 0; i <= 0xB; i++) {
		bq25600_read_byte(bq, &status[i], i);
	}

	ret = seq_printf(seq, "[bq2560X]\n0x0=0x%02x\n0x1=0x%02x\n0x2=0x%02x\n0x3=0x%02x\n"
								"0x4=0x%02x\n0x5=0x%02x\n0x6=0x%02x\n0x7=0x%02x\n"
								"0x8=0x%02x\n0x9=0x%02x\n0xA=0x%02x\n0xB=0x%02x\n",
			status[0], status[1], status[2], status[3], status[4], status[5],
			status[6], status[7], status[8], status[9], status[10], status[11]);
	if (ret < 0) {
		pr_notice("seq write error.\n");
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

static void create_smb_chg_proc_entry(struct bq25600 *data)
{
	smb_chg_proc_entry = proc_create_data("driver/ti_regs", 0644, NULL, &smb_chg_proc_ops, data);
	if (smb_chg_proc_entry) {
		pr_info("create proc file success!\n");
	} else
		pr_info("create proc file failed!\n");
}
#endif

static int bq25600_parse_jeita_dt(struct device *dev, struct bq25600 *bq)
{
	struct device_node *np = dev->of_node;
	int ret;

	ret = of_property_read_u32(np, "ti,bq25600,jeita-hot-degc", &bq->batt_hot_degc);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-hot-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-warm-degc", &bq->batt_warm_degc);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-warm-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-cool-degc", &bq->batt_cool_degc);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-cool-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-cool-degc-ext", &bq->batt_cool_degc_ext);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-cool-degc-ext\n");
		bq->batt_cool_degc_ext = -EINVAL; /* batt_cool_degc_ext is not supported */
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-cold-degc", &bq->batt_cold_degc);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-cold-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-hot-hysteresis", &bq->hot_temp_hysteresis);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-hot-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-cold-hysteresis", &bq->cold_temp_hysteresis);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-cold-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-cool-ma", &bq->batt_cool_ma);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-cool-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-cool-ma-ext", &bq->batt_cool_ma_ext);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-cool-ma-ext\n");
		bq->batt_cool_ma_ext = -EINVAL; /* batt_cool_ma_ext is not supported */
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-cool-mv", &bq->batt_cool_mv);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-cool-mv\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-warm-ma", &bq->batt_warm_ma);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-warm-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq25600,jeita-warm-mv", &bq->batt_warm_mv);
	if (ret) {
		pr_err("Failed to read ti,bq25600,jeita-warm-mv\n");
		return ret;
	}

	bq->software_jeita_supported = of_property_read_bool(np, "ti,bq25600,software-jeita-supported");

	return 0;
}

static struct bq25600_platform_data *bq25600_parse_dt(struct device *dev, struct bq25600 *bq)
{
	int ret;
	struct device_node *np = dev->of_node;
	struct bq25600_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct bq25600_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_err("Out of memory\n");
		return NULL;
	}

	ret = of_property_read_u32(np, "ti,bq25600,usb-vlim", &pdata->usb.vlim);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,usb-vlim\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,usb-ilim", &pdata->usb.ilim);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,usb-ilim\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,usb-vreg", &pdata->usb.vreg);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,usb-vreg\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,usb-ichg", &pdata->usb.ichg);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,usb-ichg\n");
	}
	pdata->usb.float_voltage_mv = pdata->usb.vreg;

	ret = of_property_read_u32(np, "ti,bq25600,dcp-vlim", &pdata->dcp.vlim);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,dcp-vlim\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,dcp-ilim", &pdata->dcp.ilim);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,dcp-ilim\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,dcp-vreg", &pdata->dcp.vreg);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,dcp-vreg\n");
	}
	pdata->dcp.float_voltage_mv = pdata->dcp.vreg;

	ret = of_property_read_u32(np, "ti,bq25600,dcp-ichg", &pdata->dcp.ichg);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,dcp-ichg\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,cdp-vlim", &pdata->cdp.vlim);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,cdp-vlim\n");
		pdata->cdp.vlim = pdata->dcp.vlim;
	}

	ret = of_property_read_u32(np, "ti,bq25600,cdp-ilim", &pdata->cdp.ilim);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,cdp-ilim\n");
		pdata->cdp.ilim = pdata->dcp.ilim;
	}

	ret = of_property_read_u32(np, "ti,bq25600,cdp-vreg", &pdata->cdp.vreg);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,cdp-vreg\n");
		pdata->cdp.vreg = pdata->dcp.vreg;
	}
	pdata->cdp.float_voltage_mv = pdata->cdp.vreg;

	ret = of_property_read_u32(np, "ti,bq25600,cdp-ichg", &pdata->cdp.ichg);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,cdp-ichg\n");
		pdata->cdp.ichg = pdata->dcp.ichg;
	}

	if (of_find_property(np, "ti,bq25600,thermal-mitigation", &bq->thermal_levels)) {
		bq->thermal_mitigation = devm_kzalloc(bq->dev, bq->thermal_levels, GFP_KERNEL);
		if (bq->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			devm_kfree(dev, pdata);
			return NULL;
		}
		bq->thermal_levels /= sizeof(int);
		ret = of_property_read_u32_array(np, "ti,bq25600,thermal-mitigation",
			bq->thermal_mitigation, bq->thermal_levels);
		if (ret) {
			pr_err("Couldn't read threm limits ret = %d\n", ret);
			devm_kfree(dev, pdata);
			devm_kfree(dev, bq->thermal_mitigation);
			return NULL;
		}
	}

	ret = of_property_read_u32(np, "ti,bq25600,stat-pin-ctrl", &pdata->statctrl);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,stat-pin-ctrl\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,precharge-current", &pdata->iprechg);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,precharge-current\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,termination-current", &pdata->iterm);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,termination-current\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,recharge-mv", &pdata->recharge_voltage_mv);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,recharge-mv\n");
		pdata->recharge_voltage_mv = 200;
	}

	ret = of_property_read_u32(np, "ti,bq25600,boost-voltage", &pdata->boostv);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,boost-voltage\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,boost-current", &pdata->boosti);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,boost-current\n");
	}

	ret = of_property_read_u32(np, "ti,bq25600,vac-ovp-threshold", &pdata->vac_ovp);
	if (ret) {
		pr_err("Failed to read node of ti,bq25600,vac-ovp-threshold\n");
	}

	pdata->otg_enable_gpio = of_get_named_gpio_flags(np, "ti,otg-enable-gpio", 0, NULL);
	pdata->otg_irq_gpio  = of_get_named_gpio_flags(np, "ti,irq-gpio", 0, NULL);
	pdata->enable_term = of_property_read_bool(np, "ti,bq25600,enable_term");

	bq->chg_term_by_soc = of_property_read_bool(np, "ti,bq25600,enable_soc_term_charge");
	bq->gpio_ce = of_get_named_gpio_flags(np, "ti,charge_enable-gpio", 0, NULL);

	pr_notice("irq gpio:%d, otg irq:%d, otg en:%d, charge enable term:%d, "
		"charger term by soc:%d, float volt:%d, recharge %d mv\n",
		bq->gpio_ce, pdata->otg_irq_gpio, pdata->otg_enable_gpio, pdata->enable_term,
		bq->chg_term_by_soc, pdata->usb.float_voltage_mv, pdata->recharge_voltage_mv);

	return pdata;
}

static void bq25600_init_jeita(struct bq25600 *bq)
{
	bq->batt_temp = -EINVAL;

	/* set default value in case of dts read fail */
	bq->batt_hot_degc = 600;
	bq->batt_warm_degc = 450;
	bq->batt_cool_degc = 100;
	bq->batt_cold_degc = 0;

	bq->hot_temp_hysteresis = 20;
	bq->cold_temp_hysteresis = 20;

	bq->batt_cool_ma = 900;
	bq->batt_cool_mv = 4400;
	bq->batt_warm_ma = 1080;
	bq->batt_warm_mv = 4080;

	bq->software_jeita_supported = true;

	/* DTS setting will overwrite above default value */
	bq25600_parse_jeita_dt(&bq->client->dev, bq);
}

static int bq25600_init_device(struct bq25600 *bq)
{
	int ret;

	if (false == bq->usb_present) {
		ret = bq25600_disable_watchdog_timer(bq);
		if (ret)
			pr_err("Failed to disable wdt\n");
	} else {
		ret = bq25600_set_watchdog_timer(bq, 80);
		if (ret)
			pr_err("Failed to set wdt\n");
	}

	bq25600_enable_batfet(bq);

	ret = bq25600_set_stat_ctrl(bq, bq->platform_data->statctrl);
	if (ret)
		pr_err("Failed to set stat pin control mode\n");

	ret = bq25600_enable_safety_timer(bq, false);
	if (ret)
		pr_err("Failed to set safety timer\n");

	ret = vote(bq->float_volt_votable, BATT_NORMAL_CHG_VOLT_VOTER, true, bq->platform_data->usb.vreg);
	if (ret < 0)
		pr_err("float voltage:BATT_NORMAL_CHG_VOLT_VOTER couldn't vote\n");

	ret = bq25600_set_rechargevoltage(bq, bq->platform_data->recharge_voltage_mv);
	if (ret)
		pr_err("Failed to set prechg current\n");

	ret = bq25600_set_prechg_current(bq, bq->platform_data->iprechg);
	if (ret)
		pr_err("Failed to set prechg current\n");

	ret = bq25600_set_term_current(bq, bq->platform_data->iterm);
	if (ret)
		pr_err("Failed to set termination current\n");

	ret = bq25600_set_boost_voltage(bq, bq->platform_data->boostv);
	if (ret)
		pr_err("Failed to set boost voltage\n");

	ret = bq25600_set_boost_current(bq, bq->platform_data->boosti);
	if (ret)
		pr_err("Failed to set boost current\n");

	ret = bq25600_set_acovp_threshold(bq, bq->platform_data->vac_ovp);
	if (ret)
		pr_err("Failed to set acovp threshold\n");

	ret = bq25600_set_input_volt_limit(bq, bq->platform_data->usb.vlim);
	if (ret < 0)
		pr_err("couldn't set input voltage limit\n");

	bq25600_enable_term(bq, bq->platform_data->enable_term);

	ret = bq25600_set_int_mask(bq, REG0A_IINDPM_INT_MASK | REG0A_VINDPM_INT_MASK);
	if (ret)
		pr_err("Failed to set vindpm and iindpm int mask\n");

	ret = bq25600_enable_charger(bq);
	if (ret) {
		pr_err("Failed to enable charger\n");
	} else {
		bq->charge_enabled = true;
		pr_info("Charger Enabled Successfully!\n");
	}

	bq25600_set_dp_dm_detection_enable(bq);

	bq25600_set_batfet_reset_enable(bq, false);

	return 0;
}

static int bq25600_detect_device(struct bq25600 *bq)
{
	int ret;
	u8 data;

	ret = bq25600_read_byte(bq, &data, BQ25600_REG_0B);
	if (ret == 0) {
		bq->part_no = (data & REG0B_PN_MASK) >> REG0B_PN_SHIFT;
		bq->revision = (data & REG0B_DEV_REV_MASK) >> REG0B_DEV_REV_SHIFT;
	}

	pr_info("ret = %d,bq->part_no = %d,bq->revision =%d\n",
		ret, bq->part_no, bq->revision);
	return ret;
}

/*check bat temp,set voltage and current*/
static void bq25600_check_jeita(struct bq25600 *bq, int temp)
{
	int ret;

	bq->batt_temp = temp;

	if (bq->batt_temp == -EINVAL)
		return;

	if (bq->batt_temp >= bq->batt_hot_degc) {/* HOT */
		if (!bq->batt_hot) {
			bq->batt_hot  = true;
			bq->batt_warm = false;
			bq->batt_cool = false;
			bq->batt_cold = false;
			bq->jeita_ma = 0;
			bq->jeita_mv = bq->batt_warm_mv;
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
			bq->jeita_mv = bq->batt_cool_mv;
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
			if (bq->batt_cool_degc_ext != -EINVAL && bq->batt_cool_ma_ext != -EINVAL
				&& bq->batt_temp <= bq->batt_cool_degc_ext) {
				bq->jeita_ma = bq->batt_cool_ma_ext;
			}
		}
	}  else {/* NORMAL */
		bq->batt_hot  = false;
		bq->batt_warm = false;
		bq->batt_cool = false;
		bq->batt_cold = false;
	}

	bq->jeita_active = bq->batt_cool || bq->batt_hot ||
					   bq->batt_cold || bq->batt_warm;

	if (bq->jeita_active == true) {
		if (bq->batt_hot || bq->batt_cold)
			ret = vote(bq->fcc_votable, BATT_TEMP_FCC_VOTER, true, -1);
		ret = vote(bq->fcc_votable, BATT_TEMP_FCC_VOTER, true, bq->jeita_ma);
		if (ret < 0)
			pr_err("charging current limit:THERMAL_FCC_VOTER couldn't vote, ret=%d\n", ret);

		ret = vote(bq->float_volt_votable, BATT_JEITA_CHG_VOLT_VOTER, true, bq->jeita_mv);
		if (ret < 0)
			pr_err("float voltage:BATT_JEITA_CHG_VOLT_VOTER couldn't vote, ret=%d\n", ret);
	} else {
		ret = vote(bq->fcc_votable, BATT_TEMP_FCC_VOTER, false, bq->jeita_ma);
		if (ret < 0)
			pr_err("charging current limit:THERMAL_FCC_VOTER couldn't vote, ret=%d\n", ret);

		ret = vote(bq->float_volt_votable, BATT_JEITA_CHG_VOLT_VOTER, false, bq->jeita_mv);
		if (ret < 0)
			pr_err("float voltage:BATT_JEITA_CHG_VOLT_VOTER couldn't vote, ret=%d\n", ret);
	}
}

/*check battery present,if it is changed ,present--charging,Non-existent--not charging*/
static void bq25600_check_batt_present(struct bq25600 *bq)
{
	int ret = 0;
	bool last_batt_pres = bq->batt_present;

	ret = bq25600_get_prop_batt_present(bq);
	if (!ret) {
		if (last_batt_pres != bq->batt_present) {
			ret = bq25600_charging_disable(bq, BATT_PRES, !bq->batt_present);
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

/*check battery full(from battery meter),if it is changed ,not full--charging,full--not charging*/
static void bq25600_check_batt_full(struct bq25600 *bq)
{
	int ret = 0;
	bool last_batt_full = bq->batt_full;

	ret = bq25600_get_prop_batt_full(bq);
	if (!ret) {
		if (last_batt_full != bq->batt_full) {
			ret = bq25600_charging_disable(bq, BATT_FC, bq->batt_full);
			if (ret) {
				pr_err("failed to %s charging, ret = %d\n",
						bq->batt_full ? "disable" : "enable",
						ret);
			}
			/*power_supply_changed(&bq->batt_psy);*/
			/*power_supply_changed(bq->usb_psy);*/
		}
	}
}

static const unsigned char *charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};

static void bq25600_dump_regs(struct bq25600 *bq)
{
	u8 status[12];
	int i = 0;

	for (i = 0; i <= 0xB; i++) {
		if (i == 0x9)
			continue;
		bq25600_read_byte(bq, &status[i], i);
	}
	pr_info("0x0=0x%02x,0x1=0x%02x,0x2=0x%02x,0x3=0x%02x,"
			"0x4=0x%02x,0x5=0x%02x,0x6=0x%02x,0x7=0x%02x,"
			"0x8=0x%02x,0x9=0x%02x,0xA=0x%02x,0xB=0x%02x\n",
			status[0], status[1], status[2], status[3],
			status[4], status[5], status[6], status[7],
			status[8], status[9], status[10], status[11]);
}

static void bq25600_device_reinit(struct bq25600 *bq)
	{
	int fcc_ma = 0, icl_ma = 0, float_mv = 0;

	bq25600_init_device(bq);

	float_mv = get_effective_result(bq->float_volt_votable);
	if (float_mv > 4000)
		bq25600_set_chargevolt(bq, float_mv);

	fcc_ma = get_effective_result(bq->fcc_votable);
	if (fcc_ma > 0)
		bq25600_set_chargecurrent(bq, fcc_ma);

	icl_ma = get_effective_result(bq->usb_icl_votable);
	bq25600_set_input_current_limit(bq, icl_ma);

	pr_info("device reinit fcc_ma=%d,icl_ma=%d, float %d mv ", fcc_ma, icl_ma, float_mv);
}

static void bq25600_dump_status(struct bq25600 *bq)
{
	u8 status;
	int ret = 0;
	union power_supply_propval batt_prop = {0,};

	ret = bq25600_get_batt_property(bq,
					POWER_SUPPLY_PROP_CURRENT_NOW, &batt_prop);
	if (!ret)
		pr_info("FG current:%d ", batt_prop.intval);

	if (!bq->power_good)
		pr_info("Power Poor\n");
	if (!bq->vbus_good)
		pr_info("Vbus voltage not good!\n");
	if (bq->vindpm_triggered)
		pr_info("VINDPM triggered\n");
	if (bq->iindpm_triggered)
		pr_info("IINDPM triggered\n");
	if (bq->acov_triggered)
		pr_info("ACOV triggered\n");

	if (bq->fault_status & REG09_FAULT_WDT_MASK) {
		bq25600_device_reinit(bq);
		pr_info(" Watchdog timer expired!\n");
	}

	if (bq->fault_status & REG09_FAULT_BOOST_MASK)
		pr_info("Boost fault occurred!\n");

	status = (bq->fault_status & REG09_FAULT_CHRG_MASK) >> REG09_FAULT_CHRG_SHIFT;
	if (status == REG09_FAULT_CHRG_INPUT)
		pr_info("input fault!\n");
	else if (status == REG09_FAULT_CHRG_THERMAL)
		pr_info("charge thermal shutdown fault!\n");
	else if (status == REG09_FAULT_CHRG_TIMER)
		pr_info("charge timer expired fault!\n");

	if (bq->fault_status & REG09_FAULT_BAT_MASK)
		pr_info("battery ovp fault!\n");

	if (!bq->software_jeita_supported) {
		status = (bq->fault_status & REG09_FAULT_NTC_MASK) >> REG09_FAULT_NTC_SHIFT;

		if (status == REG09_FAULT_NTC_WARM)
			pr_debug("JEITA ACTIVE: WARM\n");
		else if (status == REG09_FAULT_NTC_COOL)
			pr_debug("JEITA ACTIVE: COOL\n");
		else if (status == REG09_FAULT_NTC_COLD)
			pr_debug("JEITA ACTIVE: COLD\n");
		else if (status == REG09_FAULT_NTC_HOT)
			pr_debug("JEITA ACTIVE: HOT!\n");
	} else if (bq->jeita_active) {
		if (bq->batt_hot)
			pr_info("JEITA ACTIVE: HOT\n");
		else if (bq->batt_warm)
			pr_info("JEITA ACTIVE: WARM\n");
		else if (bq->batt_cool)
			pr_info("JEITA ACTIVE: COOL\n");
		else if (bq->batt_cold)
			pr_info("JEITA ACTIVE: COLD\n");
	}

	pr_info("%s fault status:0x%x\n", charge_stat_str[bq->charge_state], bq->fault_status);

	bq25600_dump_regs(bq);
}

static void bq25600_update_status(struct bq25600 *bq)
{
	u8 status;
	int ret;

	ret = bq25600_read_byte(bq, &status, BQ25600_REG_0A);
	if (ret) {
		pr_err("failed to read reg0a\n");
		return;
	}

	mutex_lock(&bq->data_lock);
	bq->vbus_good = !!(status & REG0A_VBUS_GD_MASK);
	bq->vindpm_triggered = !!(status & REG0A_VINDPM_STAT_MASK);
	bq->iindpm_triggered = !!(status & REG0A_IINDPM_STAT_MASK);
	bq->topoff_active = !!(status & REG0A_TOPOFF_ACTIVE_MASK);
	bq->acov_triggered = !!(status & REG0A_ACOV_STAT_MASK);
	mutex_unlock(&bq->data_lock);

	/* Read twice to get present status */
	ret = bq25600_read_byte(bq, &status, BQ25600_REG_09);
	ret = bq25600_read_byte(bq, &status, BQ25600_REG_09);
	if (ret)
		return;

	mutex_lock(&bq->data_lock);
	bq->fault_status = status;
	mutex_unlock(&bq->data_lock);

	bq25600_dump_status(bq);
}

static void bq25600_otg_boot_workfunc(struct work_struct *work)
{
	struct bq25600 *bq = container_of(work, struct bq25600, otg_boot_work.work);

	if (gpio_is_valid(bq->platform_data->otg_irq_gpio)) {
		if (0 == !!gpio_get_value(bq->platform_data->otg_irq_gpio)) {
			power_supply_set_usb_otg(bq->usb_psy, 1);
			pr_info("otg enable\n");
		}
	}
}

static irqreturn_t bq25600_otg_handler(int irq, void *dev_id)
{
	struct bq25600 *bq = dev_id;
	int irq_value = 0;
	int otg_enabled = 0;

	irq_value = gpio_get_value(bq->platform_data->otg_irq_gpio);
	pr_info("irq value:%d", irq_value);
	otg_enabled = !irq_value;
	dev_notice(bq->dev, "status = 0x%02x\n", otg_enabled);
	power_supply_set_usb_otg(bq->usb_psy, otg_enabled ? 1 : 0);

	return IRQ_HANDLED;
}

static void bq25600_dcp_icl_recheck_work(struct work_struct *work)
{
	struct bq25600 *bq = container_of(work, struct bq25600, dcp_icl_recheck_work.work);
	u8 status;
	int ret;

	ret = bq25600_read_byte(bq, &status, BQ25600_REG_00);
	if (ret) {
		pr_err("fail to read BQ25600_REG_00.\n");
		return;
	}
	pr_info("BQ25600_REG_00 = 0x%x before icl recheck\n", status);

	if ((status & 0x1F) > 0xF) {
		bq25600_set_input_current_limit(bq, bq->platform_data->dcp.ilim);
		ret = bq25600_read_byte(bq, &status, BQ25600_REG_00);
		if (ret) {
			pr_err("fail to read BQ25600_REG_00.\n");
			return;
		}
		pr_info("BQ25600_REG_00 = 0x%x after icl recheck\n", status);
	}
}

static void bq25600_usb_type_detection_work(struct work_struct *work)
{
	struct bq25600 *bq = container_of(work, struct bq25600, usb_type_detection_work.work);
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;
	u8 status;
	int ret, retry = 0, retry_max = 100;
	static bool is_first = true;
	static bool float_chg_det_done = false;

	if (bq->need_recheck_usb_type && float_chg_det_done == false) {
		power_supply_set_supply_type(bq->usb_psy, POWER_SUPPLY_TYPE_UNKNOWN);
		power_supply_set_present(bq->usb_psy, false);
		msleep(20);
		bq->usb_present = false;
		pr_info("force dpdm detection\n");
		bq25600_set_dp_dm_detection_enable(bq);
		while (retry < retry_max) {
			ret = bq25600_read_byte(bq, &status, BQ25600_REG_07);
			if (ret) {
				pr_err("fail to read BQ25600_REG_07.\n");
				return;
			}
			if ((status >> REG07_FORCE_DPDM_SHIFT) == REG07_FORCE_DPDM) {
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

	ret = bq25600_read_byte(bq, &status, BQ25600_REG_08);
	if (ret) {
		pr_err("fail to read BQ25600_REG_08.\n");
		return;
	}

	bq->vbus_type = (status & REG08_VBUS_STAT_MASK) >> REG08_VBUS_STAT_SHIFT;
	bq->power_good = !!(status & REG08_PG_STAT_MASK);
	pr_info("reg08:0x%x, vbus_type:%d, power_good:%d, bq->usb_present = %d\n",
		status, bq->vbus_type, bq->power_good, bq->usb_present);

	if (bq->vbus_type == BQ25600_VBUS_USB) {
		type = POWER_SUPPLY_TYPE_USB;
	} else if (bq->vbus_type == BQ25600_VBUS_CDP) {
		type = POWER_SUPPLY_TYPE_USB_CDP;
	} else if (bq->vbus_type == BQ25600_VBUS_DCP) {
		type = POWER_SUPPLY_TYPE_USB_DCP;
	} else if (bq->vbus_type == BQ25600_VBUS_NON_STANDARD) {
		type = POWER_SUPPLY_TYPE_USB_DCP;/* show non stardard charger as ac only */
	}

	if (bq->power_good)
		power_supply_set_supply_type(bq->usb_psy, type);

	if (!bq->power_good && bq->usb_present) {
		bq->usb_present = false;
		power_supply_set_present(bq->usb_psy, bq->usb_present);
		pr_notice("usb plugged out, set usb present = %d\n", bq->usb_present);
		if (bq->battery_suspend_lock.ws.active == true) {
			wake_unlock(&bq->battery_suspend_lock);
		}
		bq25600_disable_watchdog_timer(bq);
		alarm_cancel(&bq->charging_expired_alarm);
		bq25600_battery_clean_charging_expired(bq);
		float_chg_det_done = false;
	} else if (bq->power_good && !bq->usb_present) {
		bq->usb_present = true;
		/* if port is non standard, call api power_supply_set_online avoiding active usb work flow */
		if (bq->vbus_type == BQ25600_VBUS_NON_STANDARD) {
			power_supply_set_online(bq->usb_psy, bq->usb_present);
		} else {
			power_supply_set_present(bq->usb_psy, bq->usb_present);
		}
		pr_notice("usb plugged in, set usb present = %d\n", bq->usb_present);
		if (bq->battery_suspend_lock.ws.active == false) {
			wake_lock(&bq->battery_suspend_lock);
		}
		bq25600_set_watchdog_timer(bq, 80);
		alarm_cancel(&bq->charging_expired_alarm);
		msleep(20);
		ret = alarm_start_relative(&bq->charging_expired_alarm,
			ns_to_ktime(CHARGING_EXPIRATION_TIME_NS));
		if (ret) {
			pr_err("Failed to start alarm: %d\n", ret);
		}
		cancel_delayed_work_sync(&bq->charging_policy_work);
		schedule_delayed_work(&bq->charging_policy_work, round_jiffies_relative(msecs_to_jiffies(3000)));
	}

	if (is_first) {
		if (bq->vbus_type == BQ25600_VBUS_NONE && bq->power_good == 0) {
			bq->usb_present = false;
			power_supply_set_present(bq->usb_psy, bq->usb_present);
			bq25600_disable_watchdog_timer(bq);
			if (bq->battery_suspend_lock.ws.active == true) {
				wake_unlock(&bq->battery_suspend_lock);
			}
		}
		is_first = false;
	}

	bq25600_update_status(bq);

	power_supply_changed(&bq->batt_psy);
	if (bq->power_good == 1 && bq->usb_psy->type == 0 &&
		bq->vbus_type != BQ25600_VBUS_NONE && bq->vbus_type != BQ25600_VBUS_OTG) {
		pr_info("here is wa for no charging icon: power_good: %d, usb_psy->type=%d\n",
			bq->power_good, bq->usb_psy->type);
		bq->usb_psy->type = POWER_SUPPLY_TYPE_USB;
	}
	power_supply_changed(bq->usb_psy);
}

static irqreturn_t bq25600_charger_interrupt(int irq, void *dev_id)
{
	struct bq25600 *bq = dev_id;

	u8 status;
	int ret;

	pr_notice("\n");

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	wake_lock_timeout(&bq->smb_monitor_wake_lock, 5 * HZ);
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

	cancel_delayed_work_sync(&bq->usb_type_detection_work);
	schedule_delayed_work(&bq->usb_type_detection_work, msecs_to_jiffies(300));
	pr_info("usb_type_detection_work in 300ms.\n");

	ret = bq25600_read_byte(bq, &status, BQ25600_REG_08);
	if (ret) {
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}

	mutex_lock(&bq->data_lock);
	bq->power_good = !!(status & REG08_PG_STAT_MASK);
	mutex_unlock(&bq->data_lock);
#if 0
	if (!bq->power_good && bq->usb_present) {
		bq->usb_present = false;
		power_supply_set_present(bq->usb_psy, bq->usb_present);

		bq25600_disable_watchdog_timer(bq);

		if (bq->battery_suspend_lock.ws.active == true) {
			wake_unlock(&bq->battery_suspend_lock);
		}
		pr_notice("usb removed, set usb present = %d\n", bq->usb_present);
	} else if (bq->power_good && !bq->usb_present) {
		bq->usb_present = true;
		power_supply_set_present(bq->usb_psy, bq->usb_present);
		if (bq->battery_suspend_lock.ws.active == false) {
			wake_lock(&bq->battery_suspend_lock);
		}

		bq25600_set_watchdog_timer(bq, 80);

		pr_notice("usb plugged in, set usb present = %d\n", bq->usb_present);
	}

	bq25600_update_status(bq);
#endif
	mutex_unlock(&bq->irq_complete);

	return IRQ_HANDLED;
}

static void determine_initial_status(struct bq25600 *bq)
{
	int ret;
	u8 status = 0;

	ret = bq25600_get_hiz_mode(bq, &status);
	if (!ret)
		bq->in_hiz = !!status;

	bq25600_charger_interrupt(bq->client->irq, bq);
}

static ssize_t bq25600_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq25600 *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25600 Reg");
	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = bq25600_read_byte(bq, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx, "Reg[0x%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq25600_store_registers(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq25600 *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg < 0x0B) {
		bq25600_write_byte(bq, (unsigned char)reg, (unsigned char)val);
	}

	return count;
}

static ssize_t bq25600_battery_test_status_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 12, "%d\n", BatteryTestStatus_enable);
}
static ssize_t bq25600_battery_test_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;

	if (kstrtoint(buf, 10, &input) != 0)
		retval = -EINVAL;
	else
		BatteryTestStatus_enable = input;

	pr_info("BatteryTestStatus_enable = %d\n", BatteryTestStatus_enable);

	return retval;
}
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR,
	bq25600_show_registers, bq25600_store_registers);
static DEVICE_ATTR(BatteryTestStatus, S_IRUGO | S_IWUSR,
	bq25600_battery_test_status_show, bq25600_battery_test_status_store);

static struct attribute *bq25600_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_BatteryTestStatus.attr,
	NULL,
};

static const struct attribute_group bq25600_attr_group = {
	.attrs = bq25600_attributes,
};

static int get_reg(void *data, u64 *val)
{
	struct bq25600 *bq = data;
	int rc;
	u8 temp;

	rc = bq25600_read_byte(bq, &temp, bq->peek_poke_address);
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
	struct bq25600 *bq = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = bq25600_write_byte(bq, bq->peek_poke_address, temp);
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
	struct bq25600 *bq = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = bq25600_read_byte(bq, &val, addr);
		if (!ret)
			seq_printf(m, "Reg[0x%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq25600 *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}

static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct bq25600 *bq)
{
	int rc;

	bq->debug_root = debugfs_create_dir("bq25600", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("registers", S_IFREG | S_IRUGO,
				bq->debug_root, bq, &reg_debugfs_ops);
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

		debugfs_create_x32("charging_disable_status", S_IFREG | S_IRUGO,
					bq->debug_root, &(bq->charging_disabled_status));
		debugfs_create_x32("fault_status", S_IFREG | S_IRUGO, bq->debug_root, &(bq->fault_status));
		debugfs_create_x32("vbus_type", S_IFREG | S_IRUGO, bq->debug_root, &(bq->vbus_type));
		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO, bq->debug_root, &(bq->charge_state));
		debugfs_create_x32("skip_reads", S_IFREG | S_IWUSR | S_IRUGO, bq->debug_root, &(bq->skip_reads));
		debugfs_create_x32("skip_writes", S_IFREG | S_IWUSR | S_IRUGO, bq->debug_root, &(bq->skip_writes));
	}
}

static int set_charge_voltage_vote_cb(struct votable *votable, void *dev,
										int volt_mv, const char *client)
{
	struct bq25600 *bq = dev_get_drvdata(dev);
	int rc = -1;

	pr_notice("set %dmv\n", volt_mv);

	rc = bq25600_set_chargevolt(bq, volt_mv);
	if (rc) {
		dev_err(bq->dev, "Can't set FCC fcc_ma=%d rc=%d\n", volt_mv, rc);
		return rc;
	}

	return 0;
}

static int set_fastchg_current_vote_cb(struct votable *votable, void *dev,
										int fcc_ma, const char *client)
{
	struct bq25600 *bq = dev_get_drvdata(dev);
	int rc = -1;

	pr_notice("set %dmA, %s\n", fcc_ma, client);

	if (fcc_ma > 0) {
		rc = bq25600_enable_charger(bq);
		rc = bq25600_set_chargecurrent(bq, fcc_ma);
		rc = bq25600_set_watchdog_timer(bq, 80);
		pr_info("enable_charger!\n");
	} else {
		if (!strncmp(client, CHARGING_POLICY_ICL_VOTER, strlen(client))) {
			rc = bq25600_disable_charger(bq);
			pr_info("disable_charger!\n");
		}
		rc = bq25600_set_chargecurrent(bq, fcc_ma);
		rc = bq25600_disable_watchdog_timer(bq);
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
										int icl_ma, const char *client)
{
	struct bq25600 *bq = dev_get_drvdata(dev);
	int rc = -1;

	pr_notice("set %dmA\n", icl_ma);

	rc = bq25600_set_input_current_limit(bq, icl_ma);
	if (rc) {
		dev_err(bq->dev, "Can't set FCC fcc_ma=%d rc=%d\n", icl_ma, rc);
		return rc;
	}

	return 0;
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
		if (smooth->avg_volt > 3400000 && smooth->avg_temp > 50) {
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

	pr_notice("\n");
	mutex_lock(&smooth->mutex_smoothing_battery);

	ui_soc = smooth->ui_soc;
	if (ui_soc < smooth->battery_smooth_dst_soc) {
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

	pr_notice("\n");
	mutex_lock(&smooth->mutex_smoothing_battery);

	ui_soc = smooth->ui_soc;
	if (ui_soc > smooth->battery_smooth_dst_soc) {
		ui_soc--;
	}
	update_soc(smooth, ui_soc);

	mutex_unlock(&smooth->mutex_smoothing_battery);
}

static int smbchg_smoothing_soc(struct battery_smooth_prop *smooth, int src_soc, int dst_soc, int rate_msec)
{
	smooth->battery_smooth_dst_soc = dst_soc;

	if (src_soc > dst_soc) {	/* ui soc smooth down */
		smooth->soc_direction = 0;
		schedule_delayed_work(&smooth->battery_smooth_down, msecs_to_jiffies(rate_msec));
	} else {		/* ui soc smooth up */
		smooth->soc_direction = 1;
		schedule_delayed_work(&smooth->battery_smooth_up, msecs_to_jiffies(rate_msec));
	}

	return 0;
}

static int smbchg_smoothing_battery_prop(void)
{
	int ui_soc = 0, current_soc = 0;

	mutex_lock(&g_smooth_prop.mutex_smoothing_battery);

	ui_soc = g_smooth_prop.ui_soc;
	current_soc = g_smooth_prop.current_soc;

	if (g_smooth_prop.usb_present == 1) {
		if (g_smooth_prop.battery_status == POWER_SUPPLY_STATUS_FULL) {	/* soc keep or up */
			cancel_delayed_work(&g_smooth_prop.battery_smooth_down);
			if (ui_soc < 100) {
				smbchg_smoothing_soc(&g_smooth_prop, ui_soc, 100, 60000);
			}
		} else if (g_smooth_prop.battery_status == POWER_SUPPLY_STATUS_CHARGING) {	/* soc keep or up */
			cancel_delayed_work(&g_smooth_prop.battery_smooth_down);
			if ((ui_soc - current_soc) >= 2) {
				;
			} else if ((current_soc - ui_soc) >= 2) {
				smbchg_smoothing_soc(&g_smooth_prop, ui_soc, current_soc, 30000);
			} else {
				ui_soc = current_soc;
			}
		} else {	/* POWER_SUPPLY_STATUS_DISCHARGING || POWER_SUPPLY_STATUS_NOT_CHARGING
		||POWER_SUPPLY_STATUS_UNKNOWN    //soc keep or down */
			cancel_delayed_work(&g_smooth_prop.battery_smooth_up);
			if ((ui_soc == 100) && (current_soc > 97)
				&& ((g_smooth_prop.current_boot_sec - g_smooth_prop.chg_unplugged_time) < 180)) {
				pr_notice("decrease soc after 3 min\n");
			} else if ((ui_soc - current_soc) >= 2) {
				smbchg_smoothing_soc(&g_smooth_prop, ui_soc, current_soc, 60000);
			} else if ((current_soc - ui_soc) >= 2) {
				;
			} else {
				ui_soc = current_soc;
			}
		}
	} else {
		cancel_delayed_work(&g_smooth_prop.battery_smooth_up);
		if ((ui_soc == 100) && (current_soc > 97) && ((g_smooth_prop.current_boot_sec
		- g_smooth_prop.chg_unplugged_time) < 180)) {	/* soc keep or down */
			pr_notice("decrease soc after 3 min\n");
		} else if ((ui_soc - current_soc) >= 2) {	/* soc keep or down */
			g_smooth_prop.soc_direction = 0;
			if (current_soc > 90)
				smbchg_smoothing_soc(&g_smooth_prop, ui_soc, current_soc, 60000);
			if (current_soc > 30)
				smbchg_smoothing_soc(&g_smooth_prop, ui_soc, current_soc, 45000);
			else if (current_soc > 15)
				smbchg_smoothing_soc(&g_smooth_prop, ui_soc, current_soc, 30000);
			else
				smbchg_smoothing_soc(&g_smooth_prop, ui_soc, current_soc, 10000);
		} else {
			ui_soc = current_soc;
		}
	}

	if (g_smooth_prop.current_boot_sec < 30) {
		pr_notice("soc use fg report directly.");
		ui_soc = current_soc;

		if (current_soc <= 0) {
			if (g_smooth_prop.volt_now > 3400000 && g_smooth_prop.avg_temp > 50)
				ui_soc = 1;
		}
	}

	update_soc(&g_smooth_prop, ui_soc);

	mutex_unlock(&g_smooth_prop.mutex_smoothing_battery);

	return 0;
}

static int smbchg_smoothing_init(struct bq25600 *bq)
{
	int status = 0, health = 0, soc = 0, current_now = 0, voltage_now = 0, temp = 0;

	status = bq25600_get_prop_batt_status(bq);
	health = bq25600_get_prop_health(bq);
	soc = bq25600_get_prop_battery_capacity(bq);
	voltage_now = bq25600_get_prop_battery_voltage_now(bq);
	current_now = bq25600_get_prop_battery_current_now(bq);
	temp = bq25600_get_prop_battery_temp(bq);

	g_smooth_prop.avg_volt = voltage_now;
	g_smooth_prop.avg_temp = temp;
	g_smooth_prop.pre_soc = g_smooth_prop.current_soc = g_smooth_prop.fg_soc = g_smooth_prop.ui_soc = soc;
	g_smooth_prop.usb_present = bq->usb_present;
	g_smooth_prop.battery_status = status;
	g_smooth_prop.battery_therm_status = BATTERY_NORMAL;
	g_smooth_prop.pre_print_battinfo_sec = 0;
	g_smooth_prop.pre_calc_avg_sec = 0;
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

	if (index >= CALC_AVG_NUMS)
		index = 0;
	if (number >= CALC_AVG_NUMS)
		number = CALC_AVG_NUMS;

	amount = 0;
	for (i = 0; i < number; i++) {
		amount += g_smooth_prop.voltages[i];
	}

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
	g_smooth_prop.avg_temp = amount / number;
	return g_smooth_prop.avg_temp;
}
#endif

static int bq25600_monitor(struct bq25600 *bq)
{
	int status = 0, health = 0, soc = 0, current_now = 0, voltage_now = 0, temp = 0, chg_type = 0;
	ktime_t current_ktime;
	s64 current_boot_sec = 0;
	int ret = 0;
	static unsigned int nums = 0;

	current_ktime = ktime_get_boottime();
	current_boot_sec = div_s64(ktime_to_ns(current_ktime), 1000000000);

	if (bq->usb_present == true) {
		u8 state = 0;

		bq25600_get_watchdog_timer_state(bq, &state);
		if (bq25600_get_safety_timer_state(bq) || state) {
			pr_notice("maybe wdt timer expired!\n");
			bq25600_device_reinit(bq);
		}
		if (!bq->test_disable_feed_wdt)
			bq25600_reset_watchdog_timer(bq);
		else {
			pr_notice("temp disable feed whatchdog\n");
		}
	}

	bq25600_check_batt_present(bq);
	if (0) {
		bq25600_check_batt_full(bq);
	}

	status = bq25600_get_prop_batt_status(bq);
	health = bq25600_get_prop_health(bq);
	soc = bq25600_get_prop_battery_capacity(bq);
	voltage_now = bq25600_get_prop_battery_voltage_now(bq);
	current_now = bq25600_get_prop_battery_current_now(bq);
	temp = bq25600_get_prop_battery_temp(bq);
	chg_type = bq->chg_type = bq25600_get_chg_type(bq);

#ifdef CHG_SMOOTH_BATTERY_PROP
	if (((current_boot_sec - g_smooth_prop.pre_calc_avg_sec) > 10)
		|| (g_smooth_prop.pre_calc_avg_sec == 0)) {
		g_smooth_prop.pre_calc_avg_sec = current_boot_sec;

		if (is_temp_invalid(temp)) {
			pr_err("temp invalid:%d\n", temp);
			if (soc == 0) {
				pr_err("temp && soc invalid skip soc cal\n");
				goto out;
			}
		} else {
			update_average_temp(temp);
			/* bq25600_check_jeita(bq, temp); */
			bq25600_check_jeita(bq, g_smooth_prop.avg_temp);
		}
		if (is_voltage_invalid(voltage_now)) {
			pr_info("vol invalid:%d\n", voltage_now);
			if (soc == 0) {
				pr_err("bq25600 vol && soc invalid skip soc cal\n");
				goto out;
			}
		} else {
			update_average_voltage(voltage_now);
		}
	}

	g_smooth_prop.volt_now = voltage_now;

	if (g_smooth_prop.overdischarge && bq->usb_present) {
		g_smooth_prop.overdischarge = false;
	}
	if ((g_smooth_prop.avg_volt < 3350000) && (!bq->usb_present)
	&& (soc < 15) && (!g_smooth_prop.overdischarge)) {
		pr_info("battery voltage is low, soc is %d > 0, then set current soc = 0.\n", soc);
		g_smooth_prop.current_soc = 0;
		g_smooth_prop.overdischarge = true;
	}  else if (g_smooth_prop.overdischarge) {
		pr_info("battery voltage is low, soc is %d > 0, then set current soc = 0.\n", soc);
		g_smooth_prop.current_soc = 0;
		g_smooth_prop.overdischarge = true;
	} else {
		g_smooth_prop.current_soc = soc;
	}

	g_smooth_prop.fg_soc = soc;
	g_smooth_prop.usb_present = bq->usb_present;
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

		if (bq->usb_present) {
			if (bq->chg_term_battery_status != POWER_SUPPLY_STATUS_FULL) {
				/*pr_notice("bq25600 soc=%d avg_voltage= %d jeita_mv=%d fv=%d\n",
				soc, avg_voltage, bq->jeita_mv, bq->platform_data->usb.float_voltage_mv); */
				if ((soc == 100)
				    || (bq->jeita_active && (bq->jeita_mv < bq->platform_data->usb.float_voltage_mv)
						&& (avg_voltage > (bq->jeita_mv - 12)))) {
					if (soc == 100)
						bq->chg_term_full_charge_detected = true;
					if (bq->batt_cold || bq->batt_hot) {
						bq->chg_term_battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
					} else {
						bq->chg_term_battery_status = POWER_SUPPLY_STATUS_FULL;

						vote(bq->fcc_votable, CHG_TERM_SOC_FCC_VOTER, true, 0);
						if (ret < 0)
							pr_err("current limit:CHG_TERM_SOC_FCC_VOTER couldn't vote\n");

						power_supply_changed(&bq->batt_psy);
					}
				}
			} else {
				int recharge_mv = bq->platform_data->recharge_voltage_mv;

				if ((!bq->jeita_active && !bq->chg_term_full_charge_detected)
					|| (!bq->jeita_active
						&& (avg_voltage <
							(bq->platform_data->usb.float_voltage_mv - recharge_mv)))
					|| (bq->jeita_active &&  (avg_voltage < (bq->jeita_mv - recharge_mv)))
					|| (bq->batt_cold || bq->batt_hot)) {
					bq->chg_term_full_charge_detected = false;
					bq->chg_term_battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
					vote(bq->fcc_votable, CHG_TERM_SOC_FCC_VOTER, false, 0);
					if (ret < 0)
						pr_err("current limit:CHG_TERM_SOC_FCC_VOTER couldn't vote\n");

					power_supply_changed(&bq->batt_psy);
				}
			}
		} else {
			bq->chg_term_full_charge_detected = false;
			bq->chg_term_battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
			vote(bq->fcc_votable, CHG_TERM_SOC_FCC_VOTER, false, 0);
			if (ret < 0)
				pr_err("current limit:CHG_TERM_SOC_FCC_VOTER couldn't vote\n");
		}
	}

	g_smooth_prop.current_boot_sec = current_boot_sec;
	if ((!g_smooth_prop.usb_present) && g_smooth_prop.pre_usb_present) {
		g_smooth_prop.chg_unplugged_time = current_boot_sec;
	}
	g_smooth_prop.pre_usb_present = g_smooth_prop.usb_present;

	smbchg_smoothing_battery_prop();

out:
	if ((current_boot_sec - g_smooth_prop.pre_print_battinfo_sec > 16)
	    || (g_smooth_prop.pre_print_battinfo_sec == 0)) {
		pr_notice("time=%lld,avgvbat=%d,V=%d,I=%d,avgt=%d,T=%d,presoc=%d\n",
			 g_smooth_prop.current_boot_sec, g_smooth_prop.avg_volt / 1000,
			 voltage_now / 1000, current_now / 1000, g_smooth_prop.avg_temp,
			 temp, g_smooth_prop.pre_soc);
		pr_notice("soc=%d,uisoc=%d,chg=%d,Pret:%d,Therm:%d,BatSta:%d\n",
			soc, g_smooth_prop.ui_soc, chg_type, g_smooth_prop.usb_present,
			g_smooth_prop.battery_therm_status,  g_smooth_prop.battery_status);

		g_smooth_prop.pre_print_battinfo_sec = current_boot_sec;
	}
	g_smooth_prop.pre_soc = g_smooth_prop.current_soc;
#else
	if (is_voltage_invalid(voltage_now) || is_temp_invalid(temp)) {
		pr_err("charger vol or temp invalid skip this smooth, vol:%d, temp:%d\n", voltage_now, temp);
		goto out;
	}
	bq25600_check_jeita(bq, temp);
out:
	pr_notice("time=\t%lld\t,V=\t%d\t,I=\t%d\t,T=\t%d\t,soc=\t%d\t,uisoc=\t%d\t,chg=\t%d\t\n",
		current_boot_sec, voltage_now / 1000, current_now / 1000, temp, soc, soc, chg_type);
#endif
	if ((nums % 30) == 0) {
		bq25600_update_status(bq);
	}
	nums++;

	cancel_delayed_work_sync(&bq->charging_policy_work);
	schedule_delayed_work(&bq->charging_policy_work, round_jiffies_relative(msecs_to_jiffies(3000)));

	cancel_delayed_work_sync(&bq->dcp_icl_recheck_work);
	schedule_delayed_work(&bq->dcp_icl_recheck_work, msecs_to_jiffies(100));

	return 0;
}

static void bq25600_monitor_workfunc(struct work_struct *work)
{
	struct bq25600 *bq = container_of(work, struct bq25600, monitor_work.work);

	mutex_lock(&bq->irq_complete);
	bq->monitor_work_waiting = false;
	wake_lock_timeout(&bq->smb_monitor_wake_lock, 1 * HZ);
	if (false == bq->resume_completed) {
		bq->monitor_work_waiting = true;
		mutex_unlock(&bq->irq_complete);
		goto out;
	}
	mutex_unlock(&bq->irq_complete);

	bq25600_monitor(bq);

out:
	schedule_delayed_work(&bq->monitor_work, msecs_to_jiffies(15000));
}

static int bq25600_charger_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct bq25600 *bq;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct pinctrl *pinctrl;
	int ret;
	const char *str = (const char *)saved_command_line;
	const char *ret_boot = strnstr(str, "androidboot.mode=charger", sizeof("androidboot.mode=charger"));

	if (ret_boot != NULL) {
		charger_boot = true;
		pr_debug("CHARGER BOOT!\n");
	}

	pr_notice("\n");

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		dev_dbg(&client->dev, "bms supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	bq = devm_kzalloc(&client->dev, sizeof(struct bq25600), GFP_KERNEL);
	if (!bq) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->usb_psy = usb_psy;
	bq->bms_psy = bms_psy;

	bq->client = client;

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->irq_complete);
	mutex_init(&bq->current_change_lock);

	bq->resume_completed = true;
	bq->irq_waiting = false;
	bq->usb_present = false;
	bq->prop_set_charge_enabled = true;
	bq->need_recheck_usb_type = false;

	g_bq = bq;

	ret = bq25600_detect_device(bq);
	if (ret) {
		pr_err("No bq25600 device found!\n");
		devm_kfree(&client->dev, bq);
		return -ENODEV;
	}

	if (bq->part_no != BQ25600D) {
		pr_err("[bq25600_charger.c]bq->part_no != BQ25600D!\n");
		devm_kfree(&client->dev, bq);
		return -ENODEV;
	}

	wake_lock_init(&bq->battery_suspend_lock, WAKE_LOCK_SUSPEND, "battery suspend wakelock");
	wake_lock_init(&bq->smb_monitor_wake_lock, WAKE_LOCK_SUSPEND, "smb_monitor_wake");

	INIT_DELAYED_WORK(&bq->monitor_work, bq25600_monitor_workfunc);

	pinctrl = devm_pinctrl_get_select_default(&client->dev);
	if (IS_ERR(pinctrl)) {
		dev_warn(&client->dev, "pins are not configured from the driver\n");
		devm_kfree(&client->dev, bq);
		return -EINVAL;
	}

	bq25600_init_jeita(bq);
	if (client->dev.of_node)
		bq->platform_data = bq25600_parse_dt(&client->dev, bq);
	else
		bq->platform_data = client->dev.platform_data;

	if (!bq->platform_data) {
		pr_err("No platform data provided.\n");
		devm_kfree(&client->dev, bq);
		return -EINVAL;
	}

	if (gpio_is_valid(bq->gpio_ce)) {
		ret = devm_gpio_request(&client->dev, bq->gpio_ce, "bq25600_ce");
		if (ret) {
			pr_err("Failed to request chip enable gpio %d:, err: %d\n", bq->gpio_ce, ret);
			devm_kfree(&client->dev, bq);
			return ret;
		}
		gpio_direction_output(bq->gpio_ce, 0);
	}

	bq->float_volt_votable = create_votable("float voltage", VOTE_MIN,
							set_charge_voltage_vote_cb, &client->dev);
	if (IS_ERR(bq->float_volt_votable)) {
		devm_kfree(&client->dev, bq);
		return PTR_ERR(bq->float_volt_votable);
	}
	ret = vote(bq->float_volt_votable, BATT_NORMAL_CHG_VOLT_VOTER, true, bq->platform_data->dcp.vreg);

	bq->fcc_votable = create_votable("charge current", VOTE_MIN,
							set_fastchg_current_vote_cb, &client->dev);
	if (IS_ERR(bq->fcc_votable)) {
		devm_kfree(&client->dev, bq);
		return PTR_ERR(bq->fcc_votable);
	}
	ret = vote(bq->fcc_votable, CHARGER_TYPE_FCC_VOTER, true, bq->platform_data->dcp.ichg);

	bq->usb_icl_votable = create_votable("usb icl", VOTE_MIN,
							set_usb_current_limit_vote_cb, &client->dev);
	if (IS_ERR(bq->usb_icl_votable)) {
		devm_kfree(&client->dev, bq);
		return PTR_ERR(bq->usb_icl_votable);
	}
	ret = vote(bq->usb_icl_votable, CHARGER_TYPE_ICL_VOTER, true, bq->platform_data->dcp.ilim);

	ret = bq25600_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		devm_kfree(&client->dev, bq);
		return ret;
	}

#ifdef CHG_SMOOTH_BATTERY_PROP
	smbchg_smoothing_init(bq);
	mutex_init(&g_smooth_prop.mutex_smoothing_battery);
	mutex_init(&g_smooth_prop.mutex_smooth_up_down);
#endif

	ret = bq25600_psy_register(bq);
	if (ret) {
		devm_kfree(&client->dev, bq);
		return ret;
	}

	ret = bq25600_regulator_init(bq);
	if (ret) {
		pr_err("Couldn't initialize bq25600 regulator ret=%d\n", ret);
		devm_kfree(&client->dev, bq);
		return ret;
	}

	bq25600_usb_phy_init(&client->dev, bq);

	power_supply_set_usb_otg(bq->usb_psy, 0);
	schedule_delayed_work(&bq->monitor_work, msecs_to_jiffies(1000));

	INIT_DELAYED_WORK(&bq->usb_type_detection_work, bq25600_usb_type_detection_work);
	INIT_DELAYED_WORK(&bq->dcp_icl_recheck_work, bq25600_dcp_icl_recheck_work);
	INIT_DELAYED_WORK(&bq->charging_policy_work, bq25600_charging_policy_work);
	alarm_init(&bq->charging_expired_alarm, ALARM_BOOTTIME, bq25600_charging_expired_alarm_cb);

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				bq25600_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq25600 charger irq", bq);
		if (ret < 0) {
			pr_err("request irq for irq=%d failed, ret =%d\n", client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
		pr_info("request irq for irq=%d, ret =%d\n", client->irq, ret);
	}

	create_debugfs_entry(bq);

	ret = sysfs_create_group(&bq->dev->kobj, &bq25600_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
	}

	determine_initial_status(bq);

	pr_info("probe successfully, Part Num:%d, Revision:%d\n!",  bq->part_no, bq->revision);

	if (gpio_is_valid(bq->platform_data->otg_irq_gpio)) {
		int irq = 0;
		int rc = 0;

		rc = gpio_request(bq->platform_data->otg_irq_gpio, "ba2560x_otg_irq");
		if (rc) {
			dev_err(&client->dev, "irq gpio request failed, rc=%d", rc);
		} else {
			pr_info("gpio_request succes");
			rc = gpio_direction_input(bq->platform_data->otg_irq_gpio);

			irq = gpio_to_irq(bq->platform_data->otg_irq_gpio);
			if (irq < 0) {
				dev_err(&client->dev,
				"Invalid irq_gpio irq = %d\n", irq);
			} else {
				pr_info("gpio_to_irq succes");
				rc = devm_request_threaded_irq(&client->dev, irq, NULL, bq25600_otg_handler,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"bq25600_otg_handler", bq);
				if (rc) {
					dev_err(&client->dev, "Failed STAT irq=%d request rc = %d\n", irq, rc);
				} else {
					pr_info("devm_request_threaded_irq otg irq succes");
				}
			}
		}
		enable_irq_wake(irq);
	}

	pr_info("end!\n");

	if (charger_boot != true) {
		INIT_DELAYED_WORK(&bq->otg_boot_work, bq25600_otg_boot_workfunc);
		schedule_delayed_work(&bq->otg_boot_work, msecs_to_jiffies(18000));
		pr_info("BOOT with OTG,OTG detect after 18s!\n");
	}

#ifdef CONFIG_PROC_FS
	create_smb_chg_proc_entry(bq);
#endif

	charging_policy_init(bq);
	zte_misc_register_charging_policy_ops(&bq->battery_charging_policy_ops);
	schedule_delayed_work(&bq->charging_policy_work, 0);

	return 0;

err_1:
	mutex_destroy(&bq->current_change_lock);
	bq25600_psy_unregister(bq);
	pr_err("end bq25600_psy_unregister!\n");
	return ret;
}

static inline bool is_device_suspended(struct bq25600 *bq)
{
	return !bq->resume_completed;
}

static int bq25600_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq25600 *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	bq25600_usb_phy_ctrl(bq, false);
	mutex_unlock(&bq->irq_complete);
	pr_info("Suspend successfully!");

	return 0;
}

static int bq25600_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq25600 *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq25600_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq25600 *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq25600_usb_phy_ctrl(bq, true);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		bq25600_charger_interrupt(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}
	if (bq->monitor_work_waiting) {
		pr_notice("monitor work in suspend, need perform\n");
		bq25600_monitor(bq);
	}

	pr_info("Resume successfully!");

	return 0;
}

static int bq25600_charger_remove(struct i2c_client *client)
{
	struct bq25600 *bq = i2c_get_clientdata(client);

	regulator_unregister(bq->otg_vreg.rdev);

	bq25600_psy_unregister(bq);

	cancel_delayed_work_sync(&bq->usb_type_detection_work);
	cancel_delayed_work_sync(&bq->dcp_icl_recheck_work);
	cancel_delayed_work_sync(&bq->charging_policy_work);
	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);
#ifdef CHG_SMOOTH_BATTERY_PROP
	mutex_destroy(&g_smooth_prop.mutex_smoothing_battery);
	mutex_destroy(&g_smooth_prop.mutex_smooth_up_down);
#endif

	debugfs_remove_recursive(bq->debug_root);
	sysfs_remove_group(&bq->dev->kobj, &bq25600_attr_group);

	return 0;
}

static void bq25600_charger_shutdown(struct i2c_client *client)
{
	struct bq25600 *bq = i2c_get_clientdata(client);

	bq25600_battery_charging(bq, true);
	power_supply_set_usb_otg(bq->usb_psy, 0);
	bq25600_disable_otg(bq);
	msleep(100);

	pr_info("Shutdown Successfully\n");
}

static struct of_device_id bq25600_charger_match_table[] = {
	{.compatible = "ti,charger,bq25600",},
	{.compatible = "ti,charger,bq25600d",},
	{},
};
MODULE_DEVICE_TABLE(of, bq25600_charger_match_table);

static const struct i2c_device_id bq25600_charger_id[] = {
	{ "bq25600-charger", BQ25600 },
	{ "bq25600d-charger", BQ25600D },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25600_charger_id);

static const struct dev_pm_ops bq25600_pm_ops = {
	.resume		= bq25600_resume,
	.suspend_noirq = bq25600_suspend_noirq,
	.suspend	= bq25600_suspend,
};

static struct i2c_driver bq25600_charger_driver = {
	.driver = {
		.name = "bq25600-charger",
		.owner = THIS_MODULE,
		.of_match_table = bq25600_charger_match_table,
		.pm = &bq25600_pm_ops,
	},
	.id_table	= bq25600_charger_id,

	.probe = bq25600_charger_probe,
	.remove = bq25600_charger_remove,
	.shutdown = bq25600_charger_shutdown,

};

module_i2c_driver(bq25600_charger_driver);

MODULE_DESCRIPTION("TI BQ2560x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
