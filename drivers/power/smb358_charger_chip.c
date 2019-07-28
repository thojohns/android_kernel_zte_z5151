/* Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
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


#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>

#include <linux/reboot.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/wakelock.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include "pmic-voter.h"

#define CHG_SMOOTH_BATTERY_PROP
#define CONFIG_WAKE_LOCK_ON_CHARGING

#ifdef CHG_SMOOTH_BATTERY_PROP
#define CALC_AVG_NUMS 3

#define BATTERY_COLD 0
#define BATTERY_COOL 1
#define BATTERY_NORMAL 2
#define BATTERY_WARM 3
#define BATTERY_HOT 4

struct battery_smooth_prop {
	int pre_soc;
	int current_soc;
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

	s64 current_boot_sec;
	s64 battinfo_print_sec;

	struct delayed_work battery_smooth_up;
	struct delayed_work battery_smooth_down;
};

static struct battery_smooth_prop g_smooth_prop;

#endif

#define _SMB358_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB358_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB358_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
			(RIGHT_BIT_POS))

/* Config/Control registers */
#define CHG_CURRENT_CTRL_REG		0x0
#define CHG_OTH_CURRENT_CTRL_REG	0x1
#define VARIOUS_FUNC_REG		0x2
#define VFLOAT_REG			0x3
#define CHG_CTRL_REG			0x4
#define STAT_AND_TIMER_CTRL_REG		0x5
#define CHG_PIN_EN_CTRL_REG		0x6
#define THERM_A_CTRL_REG		0x7
#define SYSOK_AND_USB3_REG		0x8
#define OTHER_CTRL_REG			0x9
#define OTG_TLIM_CTRL_REG		0xA
#define HARD_SOFT_TEMP_REG	0xB
#define FAULT_INT_REG			0xC
#define STATUS_INT_REG			0xD

/* Command registers */
#define CMD_A_REG			0x30
#define CMD_B_REG			0x31

/* IRQ status registers */
#define IRQ_A_REG			0x35
#define IRQ_B_REG			0x36
#define IRQ_C_REG			0x37
#define IRQ_D_REG			0x38
#define IRQ_E_REG			0x39
#define IRQ_F_REG			0x3A

/* Status registers */
#define STATUS_C_REG			0x3D
#define STATUS_D_REG			0x3E
#define STATUS_E_REG			0x3F

#define CHG_BATTERY_OV_CONFIG_MASK 0x02
#define CHG_BATTERY_OV_END_CHARGE_CYCLE 0x02
#define CHG_BATTERY_OV_NOT_END_CHARGE_CYCLE 0x00

/* Config bits */
#define CHG_INHI_EN_MASK			BIT(1)
#define CHG_INHI_EN_BIT				BIT(1)
#define CMD_A_CHG_ENABLE_BIT			BIT(1)
#define CMD_A_VOLATILE_W_PERM_BIT		BIT(7)
#define CMD_A_CHG_SUSP_EN_BIT			BIT(2)
#define CMD_A_CHG_SUSP_EN_MASK			BIT(2)
#define CMD_A_OTG_ENABLE_BIT			BIT(4)
#define CMD_A_OTG_ENABLE_MASK			BIT(4)
#define CMD_B_CHG_HC_ENABLE_BIT			BIT(0)
#define USB3_ENABLE_BIT				BIT(5)
#define USB3_ENABLE_MASK			BIT(5)
#define CMD_B_CHG_USB_500_900_ENABLE_BIT	BIT(1)
#define CHG_CTRL_AUTO_RECHARGE_ENABLE_BIT	0x0
#define CHG_CTRL_CURR_TERM_END_CHG_BIT		0x0
#define CHG_CTRL_BATT_MISSING_DET_THERM_IO	SMB358_MASK(5, 4)
#define CHG_CTRL_AUTO_RECHARGE_MASK		BIT(7)
#define CHG_AUTO_RECHARGE_DIS_BIT		BIT(7)
#define CHG_CTRL_CURR_TERM_END_MASK		BIT(6)
#define CHG_CTRL_BATT_MISSING_DET_MASK		SMB358_MASK(5, 4)
#define CHG_CTRL_APSD_EN_BIT			BIT(2)
#define CHG_CTRL_APSD_EN_MASK			BIT(2)
#define CHG_ITERM_MASK				0x07
#define CHG_PIN_CTRL_USBCS_REG_BIT		0x0
/* This is to select if use external pin EN to control CHG */
#define CHG_PIN_CTRL_CHG_EN_LOW_PIN_BIT		SMB358_MASK(6, 5)
#define CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_MASK		SMB358_MASK(6, 5)

#define CHG_LOW_BATT_THRESHOLD \
				SMB358_MASK(3, 0)

#define CHG_OTG_EN_MASK  SMB358_MASK(7, 6)
#define CHG_OTG_CURRENT_LIMIT_MASK  SMB358_MASK(3, 2)

#define CHG_PIN_CTRL_USBCS_REG_MASK		BIT(4)
#define CHG_PIN_CTRL_APSD_IRQ_BIT		BIT(1)
#define CHG_PIN_CTRL_APSD_IRQ_MASK		BIT(1)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_BIT		BIT(2)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_MASK		BIT(2)
#define VARIOUS_FUNC_USB_SUSP_EN_REG_BIT	BIT(6)
#define VARIOUS_FUNC_USB_SUSP_MASK		BIT(6)
#define VARIOUS_FUNC_AICL_THRESHOLD_4250 0x0
#define VARIOUS_FUNC_AICL_THRESHOLD_4500 BIT(3)
#define VARIOUS_FUNC_AICL_THRESHOLD_MASK BIT(3)
#define FAULT_INT_HOT_COLD_HARD_BIT		BIT(7)
#define FAULT_INT_HOT_COLD_SOFT_BIT		BIT(6)
#define FAULT_INT_INPUT_OV_BIT			BIT(3)
#define FAULT_INT_INPUT_UV_BIT			BIT(2)
#define FAULT_INT_AICL_COMPLETE_BIT		BIT(1)
#define STATUS_INT_CHG_TIMEOUT_BIT		BIT(7)
#define STATUS_INT_OTG_DETECT_BIT		BIT(6)
#define STATUS_INT_BATT_OV_BIT			BIT(5)
#define STATUS_INT_CHGING_BIT			BIT(4)
#define STATUS_INT_CHG_INHI_BIT			BIT(3)
#define STATUS_INT_INOK_BIT			BIT(2)
#define STATUS_INT_MISSING_BATT_BIT		BIT(1)
#define STATUS_INT_LOW_BATT_BIT			BIT(0)
#define THERM_A_THERM_MONITOR_EN_BIT		0x0
#define THERM_A_THERM_MONITOR_EN_MASK		BIT(4)
#define VFLOAT_MASK				0x3F

/* IRQ status bits */
#define IRQ_A_HOT_HARD_BIT			BIT(6)
#define IRQ_A_COLD_HARD_BIT			BIT(4)
#define IRQ_A_HOT_SOFT_BIT			BIT(2)
#define IRQ_A_COLD_SOFT_BIT			BIT(0)
#define IRQ_B_BATT_MISSING_BIT			BIT(4)
#define IRQ_B_BATT_LOW_BIT			BIT(2)
#define IRQ_B_BATT_OV_BIT			BIT(6)
#define IRQ_B_PRE_FAST_CHG_BIT			BIT(0)
#define IRQ_C_TAPER_CHG_BIT			BIT(2)
#define IRQ_C_TERM_BIT				BIT(0)
#define IRQ_C_INT_OVER_TEMP_BIT			BIT(6)
#define IRQ_D_CHG_TIMEOUT_BIT			(BIT(0) | BIT(2))
#define IRQ_D_AICL_DONE_BIT			BIT(4)
#define IRQ_D_APSD_COMPLETE			BIT(6)
#define IRQ_E_INPUT_UV_BIT			BIT(0)
#define IRQ_E_INPUT_OV_BIT			BIT(2)
#define IRQ_E_AFVC_ACTIVE                       BIT(4)
#define IRQ_F_OTG_VALID_BIT			BIT(2)
#define IRQ_F_OTG_BATT_FAIL_BIT			BIT(4)
#define IRQ_F_OTG_OC_BIT			BIT(6)
#define IRQ_F_POWER_OK				BIT(0)

/* Status  bits */
#define STATUS_C_CHARGING_MASK			SMB358_MASK(2, 1)
#define STATUS_C_FAST_CHARGING			BIT(2)
#define STATUS_C_PRE_CHARGING			BIT(1)
#define STATUS_C_TAPER_CHARGING			SMB358_MASK(2, 1)
#define STATUS_C_CHG_ERR_STATUS_BIT		BIT(6)
#define STATUS_C_CHG_ENABLE_STATUS_BIT		BIT(0)
#define STATUS_C_CHG_HOLD_OFF_BIT		BIT(3)
#define STATUS_D_CHARGING_PORT_MASK \
				SMB358_MASK(3, 0)
#define STATUS_D_PORT_ACA_DOCK			BIT(3)
#define STATUS_D_PORT_SDP			BIT(2)
#define STATUS_D_PORT_DCP			BIT(1)
#define STATUS_D_PORT_CDP			BIT(0)
#define STATUS_D_PORT_OTHER			SMB358_MASK(1, 0)
#define STATUS_D_PORT_ACA_A			(BIT(2) | BIT(0))
#define STATUS_D_PORT_ACA_B			SMB358_MASK(2, 1)
#define STATUS_D_PORT_ACA_C			SMB358_MASK(2, 0)

/* constants */
#define USB2_MIN_CURRENT_MA		100
#define USB2_MAX_CURRENT_MA		500
#define USB3_MIN_CURRENT_MA		150
#define USB3_MAX_CURRENT_MA		900
#define AC_CHG_CURRENT_MASK		0x70
#define AC_CHG_CURRENT_SHIFT		4
#define SMB358_IRQ_REG_COUNT		6
#define SMB358_FAST_CHG_MIN_MA		200
#define SMB358_FAST_CHG_MAX_MA		2000
#define SMB358_FAST_CHG_SHIFT		5
#define SMB_FAST_CHG_CURRENT_MASK	0xE0
#define SMB358_DEFAULT_BATT_CAPACITY	50
#define SMB358_BATT_GOOD_THRE_2P5	0x1

enum {
	USER		= BIT(0),
	THERMAL		= BIT(1),
	CURRENT		= BIT(2),
	SOC		= BIT(3),
	FAKE_BATTERY	= BIT(4),
};

struct smb358_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct smb358_charger {
	struct i2c_client	*client;
	struct device		*dev;

	bool			inhibit_disabled;
	bool			recharge_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			vfloat_mv;
	int			chg_valid_gpio;
	int			chg_valid_act_low;
	int			chg_present;
	int			fake_battery_soc;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			using_pmic_therm;
	bool			pmic_vbat_sns;
	bool			battery_missing;
	const char		*bms_psy_name;
	bool			resume_completed;
	bool			early_resume_completed;
	bool			monitor_work_waiting;
	bool			irq_waiting;
	bool			bms_controlled_charging;
	bool			skip_usb_suspend_for_fake_battery;
	struct mutex		read_write_lock;
	struct mutex		path_suspend_lock;
	struct mutex		irq_complete;
	u8			irq_cfg_mask[2];
	int			irq_gpio;
	int			charging_disabled;
	int			fastchg_current_max_ma;
	unsigned int		cool_bat_ma;
	unsigned int		warm_bat_ma;
	unsigned int		cool_bat_mv;
	unsigned int		warm_bat_mv;
	unsigned int		connected_rid;

	enum power_supply_type supply_type;

	/* debugfs related */
#if defined(CONFIG_DEBUG_FS)
	struct dentry		*debug_root;
	u32			peek_poke_address;
#endif
	/* status tracking */
	bool			batt_full;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;
	bool			jeita_supported;
	int			charging_disabled_status;
	int			usb_suspended;
	int			usb_dp_dm_status;

	int			psy_health_sts;

	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;
	struct mutex			therm_lvl_lock;

	/* power supply */
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	batt_psy;

	/* otg 5V regulator */
	struct smb358_regulator	otg_vreg;

	/* adc_tm parameters */
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;
	int			cold_bat_decidegc;
	int			hot_bat_decidegc;
	int			cool_bat_decidegc;
	int			warm_bat_decidegc;
	int			bat_present_decidegc;
	/* i2c pull up regulator */
	struct regulator	*vcc_i2c;

	struct delayed_work battery_monitor_work;
	struct wake_lock smb_monitor_wake_lock;

#ifdef CONFIG_WAKE_LOCK_ON_CHARGING
	struct wake_lock smb_charge_wake_lock;
	struct mutex charge_mutex;
	int charge_wakelocked;
#endif

	int fastchg_current_ma;
	int input_current_ma;

#if defined(CONFIG_FB)
	struct work_struct fb_notify_work;
	struct notifier_block fb_notif;
#endif
	bool change_icl_in_suspend;
	bool aicl_completed;

	/* voters */
	struct votable			*fcc_votable;
	struct votable			*usb_icl_votable;

	bool dump_irq_status;

	bool battery_ov;
	bool floating_volt_down;
};

struct smb_irq_info {
	const char		*name;
	int			(*smb_irq)(struct smb358_charger *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

enum fcc_voters {
	USER_FCC_VOTER,
	CHARGER_TYPE_FCC_VOTER,
	BATT_TEMP_FCC_VOTER,
	SUSPEND_FCC_VOTER,
	NUM_FCC_VOTER,
};

enum icl_voters {
	PSY_ICL_VOTER,
	THERMAL_ICL_VOTER,
	USER_ICL_VOTER,
	SUSPEND_ICL_VOTER,
	NUM_ICL_VOTER,
};

static void smbchg_battery_monitor_work(struct work_struct *work);

static int chg_current[] = {
	300, 500, 700, 1000, 1200, 1500, 1800, 2000,
};

static int fast_chg_current[] = {
	200, 450, 600, 900, 1300, 1500, 1800, 2000,
};

static int aicl_current[8] = {
	300, 500, 700, 1000, 1200, 1500, 1800, 2000,
};

static bool charger_boot = false;
static bool is_init = false;
static bool is_user_temp = false;
static int user_temp = 0;

static int __smb358_read_reg(struct smb358_charger *chip, u8 reg, u8 *val)
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

static int __smb358_write_reg(struct smb358_charger *chip, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb358_read_reg(struct smb358_charger *chip, int reg,
						u8 *val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb358_write_reg(struct smb358_charger *chip, int reg,
						u8 val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_write_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb358_masked_write(struct smb358_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, &temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_read_reg Failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __smb358_write_reg(chip, reg, temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_write Failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int smb358_enable_volatile_writes(struct smb358_charger *chip)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_PERM_BIT,
						CMD_A_VOLATILE_W_PERM_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n",
				rc);

	return rc;
}

static int smb358_fastchg_current_set(struct smb358_charger *chip,
					unsigned int fastchg_current)
{
	int i;

	if ((fastchg_current < SMB358_FAST_CHG_MIN_MA) ||
		(fastchg_current >  SMB358_FAST_CHG_MAX_MA)) {
		dev_dbg(chip->dev, "bad fastchg current mA=%d asked to set\n",
						fastchg_current);
		return -EINVAL;
	}

	for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
		if (fast_chg_current[i] <= fastchg_current)
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Invalid current setting %dmA\n",
						fastchg_current);
		i = 0;
	}

	i = i << SMB358_FAST_CHG_SHIFT;
	dev_dbg(chip->dev, "fastchg limit=%d setting %02x\n",
					fastchg_current, i);

	return smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
				SMB_FAST_CHG_CURRENT_MASK, i);
}

#define MIN_FLOAT_MV		3500
#define MAX_FLOAT_MV		4500
#define VFLOAT_STEP_MV		20
#define VFLOAT_4350MV		4350
static int smb358_float_voltage_set(struct smb358_charger *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (vfloat_mv == VFLOAT_4350MV)
		temp = 0x2B;
	else if (vfloat_mv > VFLOAT_4350MV)
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV + 1;
	else
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb358_masked_write(chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

#define CHG_ITERM_30MA			0x00
#define CHG_ITERM_40MA			0x01
#define CHG_ITERM_60MA			0x02
#define CHG_ITERM_80MA			0x03
#define CHG_ITERM_100MA			0x04
#define CHG_ITERM_125MA			0x05
#define CHG_ITERM_150MA			0x06
#define CHG_ITERM_200MA			0x07
static int smb358_term_current_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled)
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");

		if (chip->iterm_ma <= 30)
			reg = CHG_ITERM_30MA;
		else if (chip->iterm_ma <= 40)
			reg = CHG_ITERM_40MA;
		else if (chip->iterm_ma <= 60)
			reg = CHG_ITERM_60MA;
		else if (chip->iterm_ma <= 80)
			reg = CHG_ITERM_80MA;
		else if (chip->iterm_ma <= 100)
			reg = CHG_ITERM_100MA;
		else if (chip->iterm_ma <= 125)
			reg = CHG_ITERM_125MA;
		else if (chip->iterm_ma <= 150)
			reg = CHG_ITERM_150MA;
		else
			reg = CHG_ITERM_200MA;

		rc = smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
							CHG_ITERM_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->iterm_disabled) {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK,
					CHG_CTRL_CURR_TERM_END_MASK);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n",
								rc);
			return rc;
		}
	} else {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK, 0);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't enable iterm rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define VFLT_300MV			0x0C
#define VFLT_200MV			0x08
#define VFLT_100MV			0x04
#define VFLT_50MV			0x00
#define VFLT_MASK			0x0C
static int smb358_recharge_and_inhibit_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->recharge_disabled)
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
		CHG_CTRL_AUTO_RECHARGE_MASK, CHG_AUTO_RECHARGE_DIS_BIT);
	else
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
			CHG_CTRL_AUTO_RECHARGE_MASK, 0x0);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set auto recharge en reg rc = %d\n", rc);
	}

	if (chip->inhibit_disabled)
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, 0x0);
	else
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, CHG_INHI_EN_BIT);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set inhibit en reg rc = %d\n", rc);
	}

	if (chip->recharge_mv != -EINVAL) {
		if (chip->recharge_mv <= 50)
			reg = VFLT_50MV;
		else if (chip->recharge_mv <= 100)
			reg = VFLT_100MV;
		else if (chip->recharge_mv <= 200)
			reg = VFLT_200MV;
		else
			reg = VFLT_300MV;

		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						VFLT_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set inhibit threshold rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

static int set_fastchg_current_vote_cb(struct device *dev, int fcc_ma, int client, int last_fcc_ma, int last_client)
{
	struct smb358_charger *chip = dev_get_drvdata(dev);
	int rc = -1;

	pr_notice("smb358 %s set %d mA, last %d mA, client %d last client %d\n",
		__func__, fcc_ma, last_fcc_ma, client, last_client);

	rc = smb358_fastchg_current_set(chip, fcc_ma);
	if (rc) {
		dev_err(chip->dev, "Can't set FCC fcc_ma=%d rc=%d\n", fcc_ma, rc);
		return rc;
	}

	return 0;
}

/*
 * set the usb charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int set_usb_current_limit_vote_cb(struct device *dev,
			int icl_ma, int client, int last_icl_ma, int last_client)
{
	struct smb358_charger *chip = dev_get_drvdata(dev);
	int i = 0, rc = -1;

	pr_notice("smb358 %s set %d mA, last %d mA, client %d last client %d\n",
		__func__, icl_ma, last_icl_ma, client, last_client);

	for (i = ARRAY_SIZE(chg_current) - 1; i >= 0; i--) {
		if (chg_current[i] <= icl_ma)
			break;
	}
	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dmA\n", icl_ma);
		i = 0;
		return rc;
	}
	i = i << AC_CHG_CURRENT_SHIFT;

	rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG, AC_CHG_CURRENT_MASK, i);
	if (rc) {
		dev_err(chip->dev, "Can't set FCC fcc_ma=%d rc=%d\n", icl_ma, rc);
		return rc;
	}

	return 0;
}

static int smb358_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	dev_notice(chip->dev,  "%s\n", __func__);

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT,
							CMD_A_OTG_ENABLE_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);
	return rc;
}

static int smb358_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	dev_notice(chip->dev,  "%s\n", __func__);

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
	if (rc)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);
	return rc;
}

static int smb358_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_read_reg(chip, CMD_A_REG, &reg);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't read OTG enable bit rc=%d, reg=%2x\n",
							rc, CMD_A_REG);
		return rc;
	}

	return  (reg & CMD_A_OTG_ENABLE_BIT) ? 1 : 0;
}

struct regulator_ops smb358_chg_otg_reg_ops = {
	.enable		= smb358_chg_otg_regulator_enable,
	.disable	= smb358_chg_otg_regulator_disable,
	.is_enabled	= smb358_chg_otg_regulator_is_enable,
};

static int smb358_regulator_init_chip(struct smb358_charger *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *regulator_node;

	pr_notice("%s\n", __func__);

	regulator_node = of_find_node_by_name(NULL,
			"qcom,smb358-boost-otg");

	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb358_chg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = devm_regulator_register(chip->dev,
						&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}
	pr_notice("%s return :%d.\n", __func__, rc);
	return rc;
}

static int smb358_regulator_init(struct smb358_charger *chip)
{
	return smb358_regulator_init_chip(chip);
#if 0
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Allocate memory failed\n");
		return -ENOMEM;
	}

	/* Give the name, then will register */
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb358_chg_otg_reg_ops;
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
#endif
}
#ifdef CONFIG_WAKE_LOCK_ON_CHARGING
/*added for charge wakelock 20170425*/
static void smb358_charge_wake_lock(struct smb358_charger *chip)
{
	pr_notice("%s\n", __func__);
	mutex_lock(&chip->charge_mutex);

	if (!chip->charge_wakelocked) {
		wake_lock(&chip->smb_charge_wake_lock);
		chip->charge_wakelocked = 1;
	}

	mutex_unlock(&chip->charge_mutex);
}

static void smb358_charge_wake_unlock(struct smb358_charger *chip)
{
	pr_notice("%s\n", __func__);
	mutex_lock(&chip->charge_mutex);

	if (chip->charge_wakelocked) {
		wake_unlock(&chip->smb_charge_wake_lock);
		chip->charge_wakelocked = 0;
	}

	mutex_unlock(&chip->charge_mutex);
}
#endif
static int __smb358_path_suspend(struct smb358_charger *chip, bool suspend)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK,
					suspend ? CMD_A_CHG_SUSP_EN_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set CMD_A reg, rc = %d\n", rc);

	return rc;
}

static int smb358_path_suspend(struct smb358_charger *chip, int reason,
								bool suspend)
{
	int rc = 0;
	int suspended;

	mutex_lock(&chip->path_suspend_lock);
	suspended = chip->usb_suspended;

	if (suspend == false)
		suspended &= ~reason;
	else
		suspended |= reason;

	if (!chip->usb_suspended && suspended) {
		rc = __smb358_path_suspend(chip, true);
		chip->usb_suspended = suspended;
		power_supply_set_online(chip->usb_psy, !chip->usb_suspended);
		power_supply_changed(chip->usb_psy);
	} else if (chip->usb_suspended && !suspended) {
		rc = __smb358_path_suspend(chip, false);
		chip->usb_suspended = suspended;
		power_supply_set_online(chip->usb_psy, !chip->usb_suspended);
		power_supply_changed(chip->usb_psy);
	}

	if (rc)
		dev_err(chip->dev, "Couldn't set/unset suspend rc = %d\n", rc);

	mutex_unlock(&chip->path_suspend_lock);

	return rc;
}


static int __smb358_charging_disable(struct smb358_charger *chip, bool disable)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT,
			disable ? 0 : CMD_A_CHG_ENABLE_BIT);
	if (rc < 0)
		pr_err("Couldn't set CHG_ENABLE_BIT disable = %d, rc = %d\n",
				disable, rc);
	return rc;
}

static int smb358_charging_disable(struct smb358_charger *chip,
						int reason, int disable)
{
	int rc = 0;
	int disabled;

	disabled = chip->charging_disabled_status;

	pr_debug("reason = %d requested_disable = %d disabled_status = %d\n",
						reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (!!disabled == !!chip->charging_disabled_status)
		goto skip;

	rc = __smb358_charging_disable(chip, !!disabled);
	if (rc) {
		pr_err("Failed to disable charging rc = %d\n", rc);
		return rc;
	}
	/* will not modify online status in this condition */
	power_supply_changed(&chip->batt_psy);

skip:
	chip->charging_disabled_status = disabled;
	return rc;
}

#define MAX_INV_BATT_ID		7700
#define MIN_INV_BATT_ID		7300

static int smb358_hw_init(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0, mask = 0;

	/*
	 * If the charger is pre-configured for autonomous operation,
	 * do not apply additonal settings
	 */
	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "Charger configured for autonomous mode\n");
		return 0;
	}

	rc = smb358_enable_volatile_writes(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure volatile writes rc=%d\n",
				rc);
		return rc;
	}

	/* setup defaults for CHG_CNTRL_REG */
#if 0
	reg = CHG_CTRL_BATT_MISSING_DET_THERM_IO;
#else
	reg = 0x20;
#endif
	mask = CHG_CTRL_BATT_MISSING_DET_MASK;
	rc = smb358_masked_write(chip, CHG_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	/* setup defaults for PIN_CTRL_REG */
	reg = CHG_PIN_CTRL_USBCS_REG_BIT | CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT |
		CHG_PIN_CTRL_APSD_IRQ_BIT | CHG_PIN_CTRL_CHG_ERR_IRQ_BIT;
	mask = CHG_PIN_CTRL_CHG_EN_MASK | CHG_PIN_CTRL_USBCS_REG_MASK |
		CHG_PIN_CTRL_APSD_IRQ_MASK | CHG_PIN_CTRL_CHG_ERR_IRQ_MASK;
	rc = smb358_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* setup USB suspend and APSD  */
	rc = smb358_masked_write(chip, VARIOUS_FUNC_REG,
		VARIOUS_FUNC_USB_SUSP_MASK | VARIOUS_FUNC_AICL_THRESHOLD_MASK,
		VARIOUS_FUNC_USB_SUSP_EN_REG_BIT | VARIOUS_FUNC_AICL_THRESHOLD_4500);
	if (rc) {
		dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n",
				rc);
		return rc;
	}

	if (!chip->disable_apsd)
		reg = CHG_CTRL_APSD_EN_BIT;
	else
		reg = 0;

	rc = smb358_masked_write(chip, CHG_CTRL_REG,
				CHG_CTRL_APSD_EN_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	rc = smb358_masked_write(chip, STAT_AND_TIMER_CTRL_REG,
				0x0C, 0x0C);
	if (rc) {
		dev_err(chip->dev, "Couldn't set STAT_AND_TIMER_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* Fault and Status IRQ configuration */
	reg = FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_HOT_COLD_SOFT_BIT
		| FAULT_INT_INPUT_UV_BIT | FAULT_INT_AICL_COMPLETE_BIT
		| FAULT_INT_INPUT_OV_BIT;
	rc = smb358_write_reg(chip, FAULT_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set FAULT_INT_REG rc=%d\n", rc);
		return rc;
	}
	reg = STATUS_INT_CHG_TIMEOUT_BIT | STATUS_INT_OTG_DETECT_BIT |
		STATUS_INT_BATT_OV_BIT | STATUS_INT_CHGING_BIT |
		STATUS_INT_CHG_INHI_BIT | STATUS_INT_INOK_BIT |
		STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT;
	rc = smb358_write_reg(chip, STATUS_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set STATUS_INT_REG rc=%d\n", rc);
		return rc;
	}
#if 0
	/* setup THERM Monitor */
	rc = smb358_masked_write(chip, THERM_A_CTRL_REG,
		THERM_A_THERM_MONITOR_EN_MASK, THERM_A_THERM_MONITOR_EN_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
#endif

	if (charger_boot) {
		rc = vote(chip->usb_icl_votable, SUSPEND_ICL_VOTER, true, 1200);
		if (rc)
			dev_err(chip->dev, "Couldn't set input current rc = %d\n", rc);
		rc = vote(chip->fcc_votable, CHARGER_TYPE_FCC_VOTER, true, 1300);
	} else {
		rc = vote(chip->fcc_votable, CHARGER_TYPE_FCC_VOTER, true, chip->fastchg_current_max_ma);
	}
	if (rc) {
		dev_err(chip->dev, "Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	/* set the float voltage */
	rc = smb358_float_voltage_set(chip, chip->vfloat_mv);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}

	/* set iterm */
	rc = smb358_term_current_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set term current rc=%d\n", rc);

	/* set recharge */
	rc = smb358_recharge_and_inhibit_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set recharge para rc=%d\n", rc);

	/* suspend USB path for fake battery */
	if (!chip->skip_usb_suspend_for_fake_battery) {
		if ((chip->connected_rid >= MIN_INV_BATT_ID) &&
				(chip->connected_rid <= MAX_INV_BATT_ID)) {
			rc = smb358_path_suspend(chip, FAKE_BATTERY, true);
			if (!rc)
				dev_info(chip->dev,
					"Suspended USB path reason FAKE_BATTERY\n");
		}
	}

	/* enable/disable charging */
	if (chip->charging_disabled) {
		rc = smb358_charging_disable(chip, USER, 1);
		if (rc)
			dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);
	} else {
		/*
		 * Enable charging explicitly,
		 * because not sure the default behavior.
		 */
		rc = __smb358_charging_disable(chip, 0);
		if (rc)
			dev_err(chip->dev, "Couldn't enable charging\n");
	}

	/*
	* Workaround for recharge frequent issue: When battery is
	* greater than 4.2v, and charging is disabled, charger
	* stops switching. In such a case, system load is provided
	* by battery rather than input, even though input is still
	* there. Make reg09[0:3] to be a non-zero value which can
	* keep the switcher active
	*/
	rc = smb358_masked_write(chip, OTHER_CTRL_REG, CHG_LOW_BATT_THRESHOLD | CHG_OTG_EN_MASK,
						SMB358_BATT_GOOD_THRE_2P5 | 0x80);
	if (rc)
		dev_err(chip->dev, "Couldn't write OTHER_CTRL_REG, rc = %d\n",
								rc);

	/*otg current limit at usbin 0x00:250 0x04:500 0x08:750 0x0C:900*/
	rc = smb358_masked_write(chip, OTG_TLIM_CTRL_REG, CHG_OTG_CURRENT_LIMIT_MASK, 0x0C);
	if (rc)
		dev_err(chip->dev, "Couldn't write OTG_TLIM_CTRL_REG, rc = %d\n",
								rc);

	return rc;
}

static enum power_supply_property smb358_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_DP_DM,
};

static int get_property_from_fg(struct smb358_charger *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_debug("no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_debug("bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#define DEFAULT_BATT_CURRENT_NOW	0
#define DEFAULT_BATT_VOLTAGE_NOW	0
static int smb358_get_prop_batt_current_now(struct smb358_charger *chip)
{
	int ua, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_debug("Couldn't get current rc = %d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}
	return ua;
}

static int smb358_get_prop_battery_voltage_now(struct smb358_charger *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_debug("Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}
	return uv;
}
static int smb358_get_prop_batt_status(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	if (chip->batt_full && chip->chg_present)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);

	if (reg & STATUS_C_CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if ((reg & STATUS_C_CHARGING_MASK) &&
			!(reg & STATUS_C_CHG_ERR_STATUS_BIT))
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int smb358_get_prop_batt_present(struct smb358_charger *chip)
{
	return !chip->battery_missing;
}

static int smb358_get_prop_batt_capacity(struct smb358_charger *chip)
{
#if 0 /* need_delete*/
	int uv = 0, soc = 1;
	union power_supply_propval ret = {0, };

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		soc = ret.intval;
	}


	return soc;
#else
	union power_supply_propval ret = {0, };

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		return ret.intval;
	}

	dev_dbg(chip->dev,
		"Couldn't get bms_psy, return default capacity\n");
	return SMB358_DEFAULT_BATT_CAPACITY;
#endif
}

static int smb358_get_prop_charge_type(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);

	reg &= STATUS_C_CHARGING_MASK;

	if (reg == STATUS_C_FAST_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_C_TAPER_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (reg == STATUS_C_PRE_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb358_get_prop_batt_health(struct smb358_charger *chip)
{
#if 0
	int health, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_HEALTH, &health);
	if (rc) {
		pr_debug("Couldn't get voltage rc = %d\n", rc);
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
	}
	return health;
#else
	union power_supply_propval ret = {0, };

	if (chip->batt_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		ret.intval = POWER_SUPPLY_HEALTH_COOL;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	if (chip->psy_health_sts == POWER_SUPPLY_HEALTH_OVERVOLTAGE)
		ret.intval = chip->psy_health_sts;

	return ret.intval;
#endif
}

#define DEFAULT_TEMP 250
static int smb358_get_prop_batt_temp(struct smb358_charger *chip)
{
	int temp, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_debug("Couldn't get voltage rc = %d\n", rc);
		temp = DEFAULT_TEMP;
	}
	if (is_user_temp) {
		dev_notice(chip->dev, "%s use  temprature: %d\n", __func__, user_temp);
		temp = user_temp;
	}

	return temp;
}
#if 0
static int smb358_get_prop_batt_temp(struct smb358_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (!smb358_get_prop_batt_present(chip)
			|| !chip->vadc_dev
			|| !chip->using_pmic_therm)
		return DEFAULT_TEMP;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp %d, %lld\n",
		results.adc_code, results.physical);

	return (int)results.physical;
}

static int
smb358_get_prop_battery_voltage_now(struct smb358_charger *chip)
{

	int rc = 0;
	struct qpnp_vadc_result results;

	if (!chip->vadc_dev || !chip->pmic_vbat_sns)
		return 0;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}
	return results.physical;
}
#endif
static int smb358_set_usb_chg_current(struct smb358_charger *chip,
		int current_ma)
{
	int rc = 0;
	u8 reg1 = 0, reg2 = 0, mask = 0;

	pr_notice("smb358 set current: %d\n", current_ma);

	if (chip->chg_autonomous_mode) {
		dev_notice(chip->dev, "%s: Charger in autonmous mode\n", __func__);
		return 0;
	}

	if (current_ma < USB3_MIN_CURRENT_MA && current_ma != 2)
		current_ma = USB2_MIN_CURRENT_MA;

	if (current_ma == USB2_MIN_CURRENT_MA) {
		/* USB 2.0 - 100mA */
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 &= ~CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB2_MAX_CURRENT_MA) {
		/* USB 2.0 - 500mA */
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB3_MAX_CURRENT_MA) {
		/* USB 3.0 - 900mA */
		reg1 |= USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma > USB2_MAX_CURRENT_MA) {
		/* HC mode  - if none of the above */
		reg2 |= CMD_B_CHG_HC_ENABLE_BIT;

		/*for (i = ARRAY_SIZE(chg_current) - 1; i >= 0; i--) {
			if (chg_current[i] <= current_ma)
				break;
		}
		if (i < 0) {
			dev_err(chip->dev, "Cannot find %dmA\n", current_ma);
			i = 0;
		}

		i = i << AC_CHG_CURRENT_SHIFT;
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						AC_CHG_CURRENT_MASK, i);*/

		rc = vote(chip->usb_icl_votable, PSY_ICL_VOTER, true, current_ma);
		if (rc)
			dev_err(chip->dev, "Couldn't set input mA rc=%d\n", rc);
	}

	mask = CMD_B_CHG_HC_ENABLE_BIT | CMD_B_CHG_USB_500_900_ENABLE_BIT;
	rc = smb358_masked_write(chip, CMD_B_REG, mask, reg2);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set charging mode rc = %d\n", rc);

	mask = USB3_ENABLE_MASK;
	rc = smb358_masked_write(chip, SYSOK_AND_USB3_REG, mask, reg1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set USB3 mode rc = %d\n", rc);

	/* Only set suspend bit when chg present and current_ma = 2 */
	if (current_ma == 2 && chip->chg_present) {
		rc = smb358_path_suspend(chip, CURRENT, true);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't suspend rc = %d\n", rc);
	} else {
		rc = smb358_path_suspend(chip, CURRENT, false);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set susp rc = %d\n", rc);
	}

	return rc;
}

static int
smb358_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_DP_DM:
		return 1;
	default:
		break;
	}

	return 0;
}

static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(soc, 100);
	return soc;
}

static int smb358_system_temp_level_set(struct smb358_charger *chip, int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;
	int thermal_icl_ma;

	if (!chip->thermal_mitigation) {
		dev_err(chip->dev, "Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		dev_err(chip->dev, "Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		dev_err(chip->dev, "Unsupported level selected %d forcing %d\n",
				lvl_sel, chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->therm_lvl_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;

	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		pr_notice("%s suspend.\n", __func__);
		rc = smb358_path_suspend(chip, THERMAL, true);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	if (chip->therm_lvl_sel == 0) {
		rc = vote(chip->usb_icl_votable, THERMAL_ICL_VOTER, false, 0);
		pr_notice("%s vote 1\n", __func__);
		if (rc < 0)
			pr_err("Couldn't disable USB thermal ICL vote rc=%d\n",	rc);
	} else {
		thermal_icl_ma =
			(int)chip->thermal_mitigation[chip->therm_lvl_sel];
		rc = vote(chip->usb_icl_votable, THERMAL_ICL_VOTER, true, thermal_icl_ma);
		pr_notice("%s vote 2\n", __func__);
		if (rc < 0)
			pr_err("Couldn't vote for USB thermal ICL rc=%d\n", rc);
	}

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Enable charging by taking the USB path
		 * out of suspend.
		 */
		pr_notice("%s disable suspend.\n", __func__);
		rc = smb358_path_suspend(chip, THERMAL, false);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
		goto out;
	}

out:
	mutex_unlock(&chip->therm_lvl_lock);
	return rc;
}

static int smb358_battery_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	int rc;
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!chip->bms_controlled_charging)
			return -EINVAL;
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_FULL:
			rc = smb358_charging_disable(chip, SOC, true);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set charging disable rc = %d\n",
					rc);
			} else {
				chip->batt_full = true;
				dev_dbg(chip->dev, "status = FULL, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			chip->batt_full = false;
			power_supply_changed(&chip->batt_psy);
			dev_dbg(chip->dev, "status = DISCHARGING, batt_full = %d\n",
							chip->batt_full);
			break;
		case POWER_SUPPLY_STATUS_CHARGING:
			rc = smb358_charging_disable(chip, SOC, false);
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set charging disable rc = %d\n",
								rc);
			} else {
				chip->batt_full = false;
				dev_dbg(chip->dev, "status = CHARGING, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smb358_charging_disable(chip, USER, !val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		smb358_path_suspend(chip, USER, val->intval);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		chip->usb_dp_dm_status = val->intval;
		switch (val->intval) {
		case POWER_SUPPLY_DP_DM_DPF_DMF:
			power_supply_set_online(chip->usb_psy, 1);
			power_supply_set_present(chip->usb_psy, 1);
			break;
		case POWER_SUPPLY_DP_DM_UNKNOWN:
		case POWER_SUPPLY_DP_DM_PREPARE:
		case POWER_SUPPLY_DP_DM_DPR_DMR:
			pr_notice("smb358 %s usb offline\n", __func__);
#ifdef CONFIG_WAKE_LOCK_ON_CHARGING
			smb358_charge_wake_unlock(chip);
#endif
			power_supply_set_online(chip->usb_psy, 0);
			power_supply_set_present(chip->usb_psy, 0);
			break;
		default:
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = bound_soc(val->intval);
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smb358_system_temp_level_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		chip->fastchg_current_ma = val->intval / 1000;
		rc = vote(chip->fcc_votable, USER_FCC_VOTER, true, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		chip->input_current_ma = val->intval / 1000;
		rc = vote(chip->usb_icl_votable, USER_ICL_VOTER, true, val->intval / 1000);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb358_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb358_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb358_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CHG_SMOOTH_BATTERY_PROP
		val->intval = g_smooth_prop.ui_soc;
#else
		val->intval = smb358_get_prop_batt_capacity(chip);
#endif
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !(chip->charging_disabled_status & USER);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		val->intval = chip->usb_suspended;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = chip->usb_dp_dm_status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb358_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb358_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SMB358";
		break;
	case POWER_SUPPLY_PROP_TEMP:
#ifdef CHG_SMOOTH_BATTERY_PROP
		val->intval = g_smooth_prop.avg_temp;
#else
		val->intval = smb358_get_prop_batt_temp(chip);
#endif
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CHG_SMOOTH_BATTERY_PROP
		val->intval = g_smooth_prop.avg_volt;
#else
		val->intval = smb358_get_prop_battery_voltage_now(chip);
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = smb358_get_prop_batt_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->fastchg_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->input_current_ma * 1000;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int aicl_complete(struct smb358_charger *chip, u8 status)
{
	int rc;
	u8 reg = 0;

	status = (status & 0x10) >> 4;

	pr_notice("smb358 %s in status:0x%x.\n", __func__, status);

	if (status) {
		chip->aicl_completed = true;
		rc = smb358_read_reg(chip, STATUS_E_REG, &reg);
		if (rc) {
			dev_err(chip->dev, "Couldn't read AICL D rc = %d\n", rc);
			return rc;
		}
	} else {
		chip->aicl_completed = false;
	}

	reg = reg & 0x07;
	if (reg < 8)
		pr_notice("smb358 %s in aicl current:%dmA.\n", __func__, aicl_current[reg]);
	else
		pr_notice("smb358 %s in reg:0x%x.\n", __func__, reg);

	return 0;
}

static int apsd_complete(struct smb358_charger *chip, u8 status)
{
	int rc;
	u8 reg = 0;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

	dev_notice(chip->dev, "smbchg %s in status:%d.\n", __func__, status);

	/*
	 * If apsd is disabled, charger detection is done by
	 * DCIN UV irq.
	 * status = ZERO - indicates charger removed, handled
	 * by DCIN UV irq
	 */
	if (chip->disable_apsd || status == 0) {
		dev_warn(chip->dev, "APSD %s, status = %d\n",
			chip->disable_apsd ? "disabled" : "enabled", !!status);
		return 0;
	}

	rc = smb358_read_reg(chip, STATUS_D_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STATUS D rc = %d\n", rc);
		return rc;
	}

	dev_notice(chip->dev, "%s: STATUS_D_REG=%x\n", __func__, reg);

	switch (reg & STATUS_D_CHARGING_PORT_MASK) {
	case STATUS_D_PORT_ACA_DOCK:
	case STATUS_D_PORT_ACA_C:
	case STATUS_D_PORT_ACA_B:
	case STATUS_D_PORT_ACA_A:
		type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case STATUS_D_PORT_CDP:
		type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case STATUS_D_PORT_DCP:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case STATUS_D_PORT_SDP:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	case STATUS_D_PORT_OTHER:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	}

	chip->chg_present = !!status;
	chip->supply_type = type;

	dev_notice(chip->dev, "APSD complete. USB type detected=%d chg_present=%d",
						chip->supply_type, chip->chg_present);

	power_supply_set_dp_dm(chip->usb_psy, POWER_SUPPLY_DP_DM_DPF_DMF);

	power_supply_set_supply_type(chip->usb_psy, chip->supply_type);

	 /* SMB is now done sampling the D+/D- lines, indicate USB driver */
	dev_notice(chip->dev, "%s updating usb_psy present=%d", __func__,
			chip->chg_present);
	power_supply_set_present(chip->usb_psy, chip->chg_present);
	power_supply_set_health_state(chip->usb_psy, POWER_SUPPLY_HEALTH_GOOD);
	power_supply_set_online(chip->usb_psy, chip->chg_present);

#ifdef CONFIG_WAKE_LOCK_ON_CHARGING
	smb358_charge_wake_lock(chip);   /*added for charge wakelock 20170425*/
#endif
	return 0;
}

static int chg_uv(struct smb358_charger *chip, u8 status)
{
	int rc;

	dev_notice(chip->dev, "smbchg %s chip->disable_apsd :%d in\n", __func__, chip->disable_apsd);

	/* use this to detect USB insertion only if !apsd */
	if (chip->disable_apsd && status == 0) {
		chip->chg_present = true;
		dev_warn(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_supply_type(chip->usb_psy,
						POWER_SUPPLY_TYPE_USB);
		power_supply_set_present(chip->usb_psy, chip->chg_present);

#ifdef CONFIG_WAKE_LOCK_ON_CHARGING
		smb358_charge_wake_lock(chip);   /*added for charge wakelock 20170425*/
#endif

		if (chip->bms_controlled_charging) {
			/*
			* Disable SOC based USB suspend to enable charging on
			* USB insertion.
			*/
			rc = smb358_charging_disable(chip, SOC, false);
			if (rc < 0)
				dev_err(chip->dev,
				"Couldn't disable usb suspend rc = %d\n",
								rc);
		}
	}

	if (status != 0) {
		chip->chg_present = false;
		dev_warn(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
	/* we can't set usb_psy as UNKNOWN here, will lead USERSPACE issue */
		chip->supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
		if (chip->disable_apsd) {
			power_supply_set_present(chip->usb_psy, chip->chg_present);
		} else {
			power_supply_set_online(chip->usb_psy, chip->chg_present);
			power_supply_set_present(chip->usb_psy, chip->chg_present);
			power_supply_set_health_state(chip->usb_psy, POWER_SUPPLY_HEALTH_UNKNOWN);
			power_supply_set_dp_dm(chip->usb_psy, POWER_SUPPLY_DP_DM_DPR_DMR);
		}

#ifdef CONFIG_WAKE_LOCK_ON_CHARGING
		smb358_charge_wake_unlock(chip);   /*added for charge wakelock 20170425*/
#endif
		if (charger_boot && !is_init) {
			dev_notice(chip->dev, "charger boot and charger gone, do machine_power_off\n");
		}
	}

	power_supply_changed(chip->usb_psy);
	dev_dbg(chip->dev, "chip->chg_present = %d\n", chip->chg_present);

	return 0;
}

static int chg_ov(struct smb358_charger *chip, u8 status)
{
	u8 psy_health_sts;

	dev_notice(chip->dev, "smbchg %s in\n", __func__);

	if (status)
		psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else
		psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;

	chip->psy_health_sts = psy_health_sts;
	power_supply_set_health_state(
				chip->usb_psy, psy_health_sts);
	power_supply_changed(chip->usb_psy);

	return 0;
}

#define STATUS_FAST_CHARGING BIT(6)
static int fast_chg(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev,  "%s status:%d\n", __func__, status);

	if (status & STATUS_FAST_CHARGING)
		chip->batt_full = false;
	return 0;
}

static int chg_term(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev,  "%s status:%d\n", __func__, status);

	if (!chip->iterm_disabled)
		chip->batt_full = !!status;
	return 0;
}

static int taper_chg(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev,  "%s status:%d\n", __func__, status);
	return 0;
}

static int chg_recharge(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev,  "%s, status = %d\n", __func__, !!status);
	/* to check the status mean */
	chip->batt_full = !status;
	return 0;
}

static void smb358_chg_set_appropriate_battery_current(
				struct smb358_charger *chip)
{
	int rc;
	unsigned int current_max = chip->fastchg_current_max_ma;

	if (chip->batt_cool)
		current_max =
			min(current_max, chip->cool_bat_ma);
	if (chip->batt_warm)
		current_max =
			min(current_max, chip->warm_bat_ma);
	dev_dbg(chip->dev, "setting %dmA", current_max);

	rc = vote(chip->fcc_votable, BATT_TEMP_FCC_VOTER, true, current_max);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);
}

static void smb358_chg_set_appropriate_vddmax(
				struct smb358_charger *chip)
{
	int rc;
	unsigned int vddmax = chip->vfloat_mv;

	if (chip->batt_cool)
		vddmax = min(vddmax, chip->cool_bat_mv);
	if (chip->batt_warm)
		vddmax = min(vddmax, chip->warm_bat_mv);

	if (vddmax < 4200) {
		if (false == chip->floating_volt_down) {
			pr_notice("smb358 disable battery ov end charge cycle.\n");
			rc = smb358_masked_write(chip, VARIOUS_FUNC_REG,
				CHG_BATTERY_OV_CONFIG_MASK, CHG_BATTERY_OV_NOT_END_CHARGE_CYCLE);
			if (rc) {
				dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n", rc);
			}
			msleep(300);
		}
		chip->floating_volt_down = true;
	}
	dev_notice(chip->dev, "setting %dmV\n", vddmax);
	rc = smb358_float_voltage_set(chip, vddmax);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set float voltage rc = %d\n", rc);

	if (vddmax > 4200) {
		if (true == chip->floating_volt_down) {
			pr_notice("smb358 enable battery ov end charge cycle.\n");
			rc = smb358_masked_write(chip, VARIOUS_FUNC_REG,
				CHG_BATTERY_OV_CONFIG_MASK, CHG_BATTERY_OV_END_CHARGE_CYCLE);
			if (rc) {
				dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n", rc);
			}
		}
		chip->floating_volt_down = false;
	}
}

#define HYSTERESIS_DECIDEGC 20
static void smb_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct smb358_charger *chip = ctx;
	bool bat_hot = 0, bat_cold = 0, bat_present = 0, bat_warm = 0,
							bat_cool = 0;
	int temp;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invallid state parameter %d\n", state);
		return;
	}

	temp = smb358_get_prop_batt_temp(chip);

	dev_notice(chip->dev, "temp = %d state = %s\n", temp,
				state == ADC_TM_WARM_STATE ? "hot" : "cold");

	if (state == ADC_TM_WARM_STATE) {
		if (temp >= chip->hot_bat_decidegc) {
			bat_hot = true;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->hot_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >=
			chip->warm_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->warm_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc;
		} else if (temp >=
			chip->cool_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cool_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc;
		} else if (temp >=
			chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cold_bat_decidegc - HYSTERESIS_DECIDEGC;
			if (chip->jeita_supported)
				chip->adc_param.high_temp =
						chip->cool_bat_decidegc;
			else
				chip->adc_param.high_temp =
						chip->hot_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp >= chip->bat_present_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.high_temp = chip->cold_bat_decidegc;
			chip->adc_param.low_temp = chip->bat_present_decidegc
							- HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp <= chip->bat_present_decidegc) {
			bat_cold = true;
			bat_cool = false;
			bat_hot = false;
			bat_warm = false;
			bat_present = false;
			chip->adc_param.high_temp = chip->bat_present_decidegc
							+ HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_WARM_THR_ENABLE;
		} else if (temp <= chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cold_bat_decidegc + HYSTERESIS_DECIDEGC;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp =
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->cool_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cool_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cold_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->warm_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cool_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->hot_bat_decidegc) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			if (chip->jeita_supported)
				chip->adc_param.low_temp =
					chip->warm_bat_decidegc;
			else
				chip->adc_param.low_temp =
					chip->cold_bat_decidegc;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (bat_present)
		chip->battery_missing = false;
	else
		chip->battery_missing = true;

	if (bat_hot ^ chip->batt_hot || bat_cold ^ chip->batt_cold) {
		chip->batt_hot = bat_hot;
		chip->batt_cold = bat_cold;
		/* stop charging explicitly since we use PMIC thermal pin*/
		if (bat_hot || bat_cold || chip->battery_missing)
			smb358_charging_disable(chip, THERMAL, 1);
		else
			smb358_charging_disable(chip, THERMAL, 0);
	}

	if ((chip->batt_warm ^ bat_warm || chip->batt_cool ^ bat_cool)
						&& chip->jeita_supported) {
		chip->batt_warm = bat_warm;
		chip->batt_cool = bat_cool;
		smb358_chg_set_appropriate_battery_current(chip);
		smb358_chg_set_appropriate_vddmax(chip);
	}

	pr_debug("hot %d, cold %d, warm %d, cool %d, jeita supported %d, missing %d, low = %d deciDegC, high = %d deciDegC\n",
		chip->batt_hot, chip->batt_cold, chip->batt_warm,
		chip->batt_cool, chip->jeita_supported, chip->battery_missing,
		chip->adc_param.low_temp, chip->adc_param.high_temp);
	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}

/* only for SMB thermal */
static int hot_hard_handler(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev, "%s status = 0x%02x\n", __func__, status);
	chip->batt_hot = !!status;
	return 0;
}
static int cold_hard_handler(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev, "%s status = 0x%02x\n", __func__, status);
	chip->batt_cold = !!status;
	return 0;
}
static int hot_soft_handler(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev, "%s status = 0x%02x\n", __func__, status);
	chip->batt_warm = !!status;
	return 0;
}
static int cold_soft_handler(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev, "%s status = 0x%02x\n", __func__, status);
	chip->batt_cool = !!status;
	return 0;
}

static int battery_missing(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev, "%s status = 0x%02x\n", __func__, status);

	chip->battery_missing = !!status;
	return 0;
}

static int battery_overvoltage(struct smb358_charger *chip, u8 status)
{
	chip->battery_ov = !!status;

	dev_notice(chip->dev, "%s status = 0x%02x, battery ov= %d\n", __func__, status, chip->battery_ov);

	return 0;
}

static int otg_det_handler(struct smb358_charger *chip, u8 status)
{
	dev_notice(chip->dev, "%s status = 0x%02x\n", __func__, status);

	power_supply_set_usb_otg(chip->usb_psy, status ? 1 : 0);
	return 0;
}

static struct irq_handler_info handlers[] = {
	[0] = {
		.stat_reg	= IRQ_A_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "cold_soft",
				.smb_irq	= cold_soft_handler,
			},
			{
				.name		= "hot_soft",
				.smb_irq	= hot_soft_handler,
			},
			{
				.name		= "cold_hard",
				.smb_irq	= cold_hard_handler,
			},
			{
				.name		= "hot_hard",
				.smb_irq	= hot_hard_handler,
			},
		},
	},
	[1] = {
		.stat_reg	= IRQ_B_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_hot",
			},
			{
				.name		= "vbat_low",
			},
			{
				.name		= "battery_missing",
				.smb_irq	= battery_missing
			},
			{
				.name		= "battery_ov",
				.smb_irq	= battery_overvoltage,
			},
		},
	},
	[2] = {
		.stat_reg	= IRQ_C_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_term",
				.smb_irq	= chg_term,
			},
			{
				.name		= "taper",
				.smb_irq	= taper_chg,
			},
			{
				.name		= "recharge",
				.smb_irq	= chg_recharge,
			},
			{
				.name		= "fast_chg",
				.smb_irq	= fast_chg,
			},
		},
	},
	[3] = {
		.stat_reg	= IRQ_D_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "prechg_timeout",
			},
			{
				.name		= "safety_timeout",
			},
			{
				.name		= "aicl_complete",
				.smb_irq	= aicl_complete,
			},
			{
				.name		= "src_detect",
				.smb_irq	= apsd_complete,
			},
		},
	},
	[4] = {
		.stat_reg	= IRQ_E_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "usbin_uv",
				.smb_irq        = chg_uv,
			},
			{
				.name		= "usbin_ov",
				.smb_irq	= chg_ov,
			},
			{
				.name		= "unknown",
			},
			{
				.name		= "unknown",
			},
		},
	},
	[5] = {
		.stat_reg	= IRQ_F_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "power_ok",
			},
			{
				.name		= "otg_det",
				.smb_irq		= otg_det_handler,
			},
			{
				.name		= "otg_batt_uv",
			},
			{
				.name		= "otg_oc",
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static irqreturn_t smb358_chg_stat_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int i, j;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

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
		rc = smb358_read_reg(chip, handlers[i].stat_reg,
						&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			triggered = handlers[i].val
			       & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
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
		pr_debug("batt psy changed\n");
		power_supply_changed(&chip->batt_psy);
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static irqreturn_t smb358_chg_valid_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int present;

	present = gpio_get_value_cansleep(chip->chg_valid_gpio);
	if (present < 0) {
		dev_err(chip->dev, "Couldn't read chg_valid gpio=%d\n",
						chip->chg_valid_gpio);
		return IRQ_HANDLED;
	}
	present ^= chip->chg_valid_act_low;

	dev_notice(chip->dev, "%s: chg_present = %d\n", __func__, present);

	if (present != chip->chg_present) {
		chip->chg_present = present;
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

	return IRQ_HANDLED;
}

static void smb358_external_power_changed(struct power_supply *psy)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0;

	cancel_delayed_work_sync(&chip->battery_monitor_work);
	smbchg_battery_monitor_work(&chip->battery_monitor_work.work);

	if (chip->bms_psy_name)
		chip->bms_psy = power_supply_get_by_name((char *)chip->bms_psy_name);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc)
		dev_err(chip->dev,
			"Couldn't read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;

	rc = chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_TYPE, &prop);
	if (rc)
		dev_err(chip->dev, "Couldn't get charge type from USB property, rc=%d\n", rc);
	else {
		if ((prop.intval != chip->supply_type) && chip->chg_present)
			pr_notice("%s: chip APSD type:%d, from USB type:%d\n",
				__func__, chip->supply_type, prop.intval);
	}

	if ((chip->supply_type == POWER_SUPPLY_TYPE_USB_DCP)
		|| (chip->supply_type == POWER_SUPPLY_TYPE_USB_CDP)
		|| (chip->supply_type == POWER_SUPPLY_TYPE_USB_ACA)) {
		current_limit = 1500;
	}

	smb358_enable_volatile_writes(chip);
	smb358_set_usb_chg_current(chip, current_limit);

	dev_dbg(chip->dev, "%s set current_limit = %d\n", __func__, current_limit);
}

#if defined(CONFIG_DEBUG_FS)
#define LAST_CNFG_REG	0x13
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x30
#define LAST_CMD_REG	0x33
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x35
#define LAST_STATUS_REG		0x3F
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
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
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	rc = smb358_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb358_write_reg(chip, chip->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int force_irq_set(void *data, u64 val)
{
	struct smb358_charger *chip = data;

	smb358_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");

static int battery_temp_set(void *data, u64 val)
{
	if (val < 990) {
		is_user_temp = true;
		user_temp = val;
	} else {
		is_user_temp = false;
		user_temp = val;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(battery_temp_ops, NULL, battery_temp_set, "%02lld\n");
#endif

static int reset_input_path(struct smb358_charger *chip)
{
	pr_notice("smb358 %s\n", __func__);

	smb358_path_suspend(chip, USER, true);
	msleep(1000);
	smb358_path_suspend(chip, USER, false);

	return 0;
}

#ifdef CONFIG_PROC_FS
#define SMB_REGS_LENGTH 2048
static struct proc_dir_entry *smb_proc_entry;
static char g_smb_regs_buffer[SMB_REGS_LENGTH];
static int g_ilength = 0;

static void dump_regs(struct smb358_charger *chip);

static ssize_t smb_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	struct smb358_charger *chip = PDE_DATA(file_inode(filp));
	char message[256];
	int reg, val;
	int copy_len;

	if (len > 256)
		copy_len = 255;

	if (copy_from_user(message, buff, copy_len))
		return -EFAULT;
	message[copy_len] = '\0';

	pr_notice("smb358 %s\n", message);

	if (strnstr(message, "write", strlen(message)) {
		if (sscanf(&message[6], "%x %x", &reg, &val) < 0)
			pr_notice("sscanf error.\n");
		smb358_write_reg(chip, reg, val);
		return len;
	} else if (strnstr(message, "reset input", strlen(message)) {
		reset_input_path(chip);
	}

	return len;
}

static int smb_proc_show(struct seq_file *seq, void *v)
{
	struct smb358_charger *chip = seq->private;
	int ret = -1;

	pr_notice("smb358 %s", __func__);

	g_ilength = 0;

	chip->dump_irq_status = true;
	dump_regs(chip);

	g_ilength += snprintf(g_smb_regs_buffer + g_ilength, SMB_REGS_LENGTH - g_ilength, "smb358_20170701\n");

	ret = seq_printf(seq, "%s", g_smb_regs_buffer);
	if (ret < 0) {
		pr_notice("smb358 %s seq write error.\n", __func__);
	}
	return 0;
}

static int smb_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb_proc_show, PDE_DATA(inode));
}

static const struct file_operations smb_proc_ops = {
	.owner		= THIS_MODULE,
	.open		= smb_seq_open,
	.read		= seq_read,
	.write		= smb_proc_write,
	.llseek		= seq_lseek,
};

static void create_smb_proc_entry(struct smb358_charger *data)
{
	smb_proc_entry = proc_create_data("driver/smb358_regs", 0644, NULL, &smb_proc_ops, data);
	if (smb_proc_entry) {
		pr_notice("create proc file success!\n");
	} else
		pr_notice("create proc file failed!\n");
}
#endif

#define DEBUG

#ifdef DEBUG
static void dump_regs(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;
	u8 addr;

	g_ilength = 0;

	pr_notice("smb358 dump regs.\n");

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			dev_dbg(chip->dev, "0x%02x = 0x%02x\n", addr, reg);
		g_ilength += snprintf(g_smb_regs_buffer + g_ilength,
			SMB_REGS_LENGTH - g_ilength, "0x%02x = 0x%02x\n", addr, reg);
		pr_notice("0x%02x = 0x%02x\n", addr, reg);
	}
	if (true == chip->dump_irq_status) {
		for (addr = 0x35; addr <= 0x3A; addr++) {
			rc = smb358_read_reg(chip, addr, &reg);
			if (rc)
				dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
						addr, rc);
			else
				dev_dbg(chip->dev, "0x%02x = 0x%02x\n", addr, reg);
			g_ilength += snprintf(g_smb_regs_buffer + g_ilength,
				SMB_REGS_LENGTH - g_ilength, "0x%02x = 0x%02x\n", addr, reg);
			pr_notice("0x%02x = 0x%02x\n", addr, reg);
		}
		chip->dump_irq_status = false;
	}

	for (addr = 0x3B; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			dev_dbg(chip->dev, "0x%02x = 0x%02x\n", addr, reg);
		g_ilength += snprintf(g_smb_regs_buffer + g_ilength,
			SMB_REGS_LENGTH - g_ilength, "0x%02x = 0x%02x\n", addr, reg);
		pr_notice("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			dev_dbg(chip->dev, "0x%02x = 0x%02x\n", addr, reg);
		g_ilength += snprintf(g_smb_regs_buffer + g_ilength,
			SMB_REGS_LENGTH - g_ilength, "0x%02x = 0x%02x\n", addr, reg);
		pr_notice("0x%02x = 0x%02x\n", addr, reg);
	}
	addr = 0x49;
	smb358_read_reg(chip, addr, &reg);
	g_ilength += snprintf(g_smb_regs_buffer + g_ilength,
		SMB_REGS_LENGTH - g_ilength, "0x%02x = 0x%02x\n", addr, reg);
	pr_notice("0x%02x = 0x%02x\n", addr, reg);
	addr = 0x15;
	smb358_read_reg(chip, addr, &reg);
	g_ilength += snprintf(g_smb_regs_buffer + g_ilength,
		SMB_REGS_LENGTH - g_ilength, "0x%02x = 0x%02x\n", addr, reg);
	pr_notice("0x%02x = 0x%02x\n", addr, reg);
}
#else
static void dump_regs(struct smb358_charger *chip)
{
}
#endif

#ifdef CHG_SMOOTH_BATTERY_PROP
static void battery_smooth_up_work(struct work_struct *work)
{
	int ui_soc = 0;
	struct battery_smooth_prop *smooth = container_of(work,
				struct battery_smooth_prop,
				battery_smooth_up.work);

	pr_notice("smb358 %s\n", __func__);

	ui_soc = smooth->ui_soc;
	if (ui_soc < 100) {
		ui_soc++;
	}
	smooth->ui_soc = bound_soc(ui_soc);
}

static void battery_smooth_down_work(struct work_struct *work)
{
	int ui_soc = 0;
	struct battery_smooth_prop *smooth = container_of(work,
				struct battery_smooth_prop,
				battery_smooth_down.work);

	pr_notice("smb358 %s\n", __func__);

	ui_soc = smooth->ui_soc;
	if (ui_soc > smooth->current_soc) {
		ui_soc--;
	}
	smooth->ui_soc = bound_soc(ui_soc);
}

static int smbchg_smoothing_battery_prop(void)
{
	if (g_smooth_prop.chg_present == 1) {
		if (g_smooth_prop.battery_status == POWER_SUPPLY_STATUS_FULL) {
			cancel_delayed_work(&g_smooth_prop.battery_smooth_down);
			if (g_smooth_prop.ui_soc < 100) {
				g_smooth_prop.soc_direction = 1;
				schedule_delayed_work(&g_smooth_prop.battery_smooth_up, msecs_to_jiffies(60000));
			}
		} else if (g_smooth_prop.battery_status == POWER_SUPPLY_STATUS_CHARGING) {
			cancel_delayed_work(&g_smooth_prop.battery_smooth_down);
			if ((g_smooth_prop.ui_soc - g_smooth_prop.current_soc) > 2) {
				;
			} else {
				g_smooth_prop.ui_soc = g_smooth_prop.current_soc;
			}
		} else {
			cancel_delayed_work(&g_smooth_prop.battery_smooth_up);
			if ((g_smooth_prop.ui_soc - g_smooth_prop.current_soc) > 2) {
				g_smooth_prop.soc_direction = 0;
				schedule_delayed_work(&g_smooth_prop.battery_smooth_down, msecs_to_jiffies(60000));
			} else if ((g_smooth_prop.ui_soc == 100)
				&& (g_smooth_prop.current_soc > 97)
				&& ((g_smooth_prop.current_boot_sec - g_smooth_prop.chg_unplugged_time) < 180)) {
				pr_notice("smb358 decrease soc after 1 min\n");
			} else {
				g_smooth_prop.ui_soc = g_smooth_prop.current_soc;
			}
		}
	} else {
		cancel_delayed_work(&g_smooth_prop.battery_smooth_up);
		if (g_smooth_prop.ui_soc < 100 && (g_smooth_prop.ui_soc - g_smooth_prop.current_soc) > 2) {
			g_smooth_prop.soc_direction = 0;
			if (g_smooth_prop.current_soc > 30)
				schedule_delayed_work(&g_smooth_prop.battery_smooth_down, msecs_to_jiffies(45000));
			else if (g_smooth_prop.current_soc > 15)
				schedule_delayed_work(&g_smooth_prop.battery_smooth_down, msecs_to_jiffies(30000));
			else
				schedule_delayed_work(&g_smooth_prop.battery_smooth_down, msecs_to_jiffies(10000));
		} else if ((g_smooth_prop.ui_soc == 100)
			&& (g_smooth_prop.current_soc > 97)
			&& ((g_smooth_prop.current_boot_sec - g_smooth_prop.chg_unplugged_time) < 180)) {
			pr_notice("smb358 decrease soc after 1 min\n");
		} else {
			g_smooth_prop.ui_soc = g_smooth_prop.current_soc;
		}
	}

	g_smooth_prop.ui_soc = bound_soc(g_smooth_prop.ui_soc);

	if (g_smooth_prop.current_soc <= 0) {
		if (g_smooth_prop.avg_volt > 3410000) {
			g_smooth_prop.ui_soc = 1;
		} else {
			g_smooth_prop.ui_soc = 0;
		}
	}

	return 0;
}

static int smbchg_smoothing_init(struct smb358_charger *chip)
{
	int status = 0, health = 0, soc = 0, current_now = 0, voltage_now = 0, temp = 0;

	status = smb358_get_prop_batt_status(chip);
	health = smb358_get_prop_batt_health(chip);
	soc = smb358_get_prop_batt_capacity(chip);
	voltage_now = smb358_get_prop_battery_voltage_now(chip);
	current_now = smb358_get_prop_batt_current_now(chip);
	temp = smb358_get_prop_batt_temp(chip);

	g_smooth_prop.avg_volt = voltage_now;
	g_smooth_prop.avg_temp = temp;
	g_smooth_prop.pre_soc = g_smooth_prop.current_soc = g_smooth_prop.ui_soc = soc;
	g_smooth_prop.chg_present = chip->chg_present;
	g_smooth_prop.battery_status = status;
	g_smooth_prop.battery_therm_status = BATTERY_NORMAL;

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

	g_smooth_prop.avg_volt = amount / number;
	return g_smooth_prop.avg_volt;
}
static int update_average_temp(int temp)
{
	int i = 0;
	s64 amount = 0;
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

static void smb358_battery_temp_work(struct smb358_charger *chip, int temp)
{
	int batt_temp;
	bool bat_hot = 0, bat_cold = 0, bat_present = 0,		bat_warm = 0, bat_cool = 0;

	dev_dbg(chip->dev,  "%s\n", __func__);

	batt_temp = temp;

	dev_dbg(chip->dev,  "batt_temp= %d\n", batt_temp);
	dev_dbg(chip->dev,  "hot_bat_decidegc =%d,warm_bat_decidegc=%d,cool_bat_decidegc=%d,cold_bat_decidegc=%d\n",
		chip->hot_bat_decidegc, chip->warm_bat_decidegc, chip->cool_bat_decidegc, chip->cold_bat_decidegc);

	dev_dbg(chip->dev,  "cool_bat_mv=%d, warm_bat_mv=%d, cool_bat_mA=%d, warm_bat_mA=%d\n",
		chip->cool_bat_mv, chip->warm_bat_mv, chip->cool_bat_ma, chip->warm_bat_ma);
	if (batt_temp >= chip->hot_bat_decidegc) {
			bat_hot = true;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
		} else if (batt_temp >=
			chip->warm_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
		} else if (batt_temp >=
			chip->cool_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
		} else if (batt_temp >=
			chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;

		} else if (batt_temp >= chip->bat_present_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;
		}

	if (bat_hot ^ chip->batt_hot || bat_cold ^ chip->batt_cold) {
			chip->batt_hot = bat_hot;
			chip->batt_cold = bat_cold;
			if (bat_hot || bat_cold || chip->battery_missing)
				smb358_charging_disable(chip, THERMAL, 1);
			else
				smb358_charging_disable(chip, THERMAL, 0);
		}

	if ((chip->batt_warm ^ bat_warm || chip->batt_cool ^ bat_cool)
						&& chip->jeita_supported) {
		chip->batt_warm = bat_warm;
		chip->batt_cool = bat_cool;
		smb358_chg_set_appropriate_battery_current(chip);
		smb358_chg_set_appropriate_vddmax(chip);
	}
	dev_dbg(chip->dev,  "hot %d, cold %d, warm %d, cool %d, jeita supported %d, missing %d\n",
		chip->batt_hot, chip->batt_cold, chip->batt_warm,
		chip->batt_cool, chip->jeita_supported, chip->battery_missing);


}

static int smbchg_battery_monitor(struct smb358_charger *chip)
{
	int status = 0, health = 0, soc = 0, current_now = 0, voltage_now = 0, temp = 0, chg_type = 0;
	ktime_t current_ktime;
	s64 current_boot_sec = 0;
	static unsigned int nums = 0;

	current_ktime = ktime_get_boottime();
	current_boot_sec = div_s64(ktime_to_ns(current_ktime), 1000000000);


	status = smb358_get_prop_batt_status(chip);
	health = smb358_get_prop_batt_health(chip);
	soc = smb358_get_prop_batt_capacity(chip);
	voltage_now = smb358_get_prop_battery_voltage_now(chip);
	current_now = smb358_get_prop_batt_current_now(chip);
	temp = smb358_get_prop_batt_temp(chip);
	chg_type = chip->supply_type;

#ifdef CHG_SMOOTH_BATTERY_PROP
	if (is_temp_invalid(temp)) {
		pr_err("smb358 temp invalid:%d\n", temp);
		if (soc == 0) {
			pr_err("smb358 temp && soc invalid skip soc cal\n");
			goto out;
		}
	} else {
		update_average_temp(temp);
		smb358_battery_temp_work(chip, temp);
	}
	if (is_voltage_invalid(voltage_now)) {
		pr_err("smb358 vol invalid:%d\n", voltage_now);
		if (soc == 0) {
			pr_err("smb358 vol && soc invalid skip soc cal\n");
			goto out;
		}
	} else {
		update_average_voltage(voltage_now);
	}

	g_smooth_prop.current_soc = soc;
	g_smooth_prop.chg_present = chip->chg_present;
	g_smooth_prop.battery_status = status;
	g_smooth_prop.battery_therm_status = BATTERY_NORMAL;

	if (chip->batt_hot) {
		g_smooth_prop.battery_therm_status = BATTERY_HOT;
	} else if (chip->batt_warm) {
		g_smooth_prop.battery_therm_status = BATTERY_WARM;
	} else if (chip->batt_cool) {
		g_smooth_prop.battery_therm_status = BATTERY_COOL;
	} else if (chip->batt_cold) {
		g_smooth_prop.battery_therm_status = BATTERY_COLD;
	} else {
		g_smooth_prop.battery_therm_status = BATTERY_NORMAL;
	}
	g_smooth_prop.current_boot_sec = current_boot_sec;

	if ((!g_smooth_prop.chg_present) && g_smooth_prop.pre_chg_present) {
		g_smooth_prop.chg_unplugged_time = current_boot_sec;
	}
	g_smooth_prop.pre_chg_present = g_smooth_prop.chg_present;

	smbchg_smoothing_battery_prop();

	if (g_smooth_prop.current_boot_sec < 30) {
		pr_notice("smb358 soc use fg report directly.");
		g_smooth_prop.pre_soc = g_smooth_prop.current_soc = g_smooth_prop.ui_soc = soc;

		if (g_smooth_prop.current_soc <= 0) {
			if (g_smooth_prop.avg_volt > 3410000) {
				g_smooth_prop.ui_soc = 1;
			} else {
				g_smooth_prop.ui_soc = 0;
			}
		}
	}
out:
	if (current_boot_sec - g_smooth_prop.battinfo_print_sec > 15) {
		pr_notice("t,%lld,avgvbat,%d,V,%d,I,%d,avgt,%d,T,%d,pc,%d,c,%d,uc,%d,TP,%d,Pret:%d,Therm:%d,BatSta:%d",
			g_smooth_prop.current_boot_sec, g_smooth_prop.avg_volt / 1000, voltage_now / 1000,
			current_now / 1000, g_smooth_prop.avg_temp, temp,
			g_smooth_prop.pre_soc, soc, g_smooth_prop.ui_soc, chg_type,
			g_smooth_prop.chg_present, g_smooth_prop.battery_therm_status,  g_smooth_prop.battery_status);
		g_smooth_prop.battinfo_print_sec = current_boot_sec;
	}
	g_smooth_prop.pre_soc = g_smooth_prop.current_soc;
#else
	smb358_battery_temp_work(chip, temp);
out:
	pr_notice("smb358 AvgVbat=,%d,bat_vol=,%d,AvgIchr=,%d,
		Ichr=,%d,Ibat=,%d,VChr=,%d,AvgT=,%d,T=,%d,
		pre_SOC=,%d,SOC=,%d,UI_SOC=,%d,ZCV=,%d,CHR_Type=,%d",
		voltage_now, voltage_now, current_now, current_now,
		current_now, voltage_now, temp, temp, soc, soc, soc, voltage_now, chg_type);
#endif

	if ((nums % 35) == 0) {
		if ((g_smooth_prop.battery_status == POWER_SUPPLY_STATUS_FULL)
			&& ((g_smooth_prop.avg_volt < 4100000) || (!g_smooth_prop.chg_present))) {
			if (g_smooth_prop.chg_present) {
				pr_notice("smb358 status full, vol below 4.1v.");
				nums++;
				reset_input_path(chip);
			}
			chip->dump_irq_status = true;
			dump_regs(chip);
		} else if ((true == chip->battery_ov)
			&& (g_smooth_prop.avg_volt < 3900000)
			&& (g_smooth_prop.chg_present)) {
			pr_notice("smb358 status battery overvoltage, volt below 3.9v.");
			nums++;
			reset_input_path(chip);
		} else {
			dump_regs(chip);
		}
	}
	nums++;

	return 0;
}

static void smbchg_battery_monitor_work(struct work_struct *work)
{
	struct smb358_charger *chip = container_of(work,
				struct smb358_charger,
				battery_monitor_work.work);

	mutex_lock(&chip->irq_complete);
	chip->monitor_work_waiting = false;
	wake_lock_timeout(&chip->smb_monitor_wake_lock, 1 * HZ);    /*added for suspend20170321*/
	if (false == chip->resume_completed) {
		chip->monitor_work_waiting = true;
		mutex_unlock(&chip->irq_complete);
		goto out;
	}
	mutex_unlock(&chip->irq_complete);

	smbchg_battery_monitor(chip);

out:
	schedule_delayed_work(&chip->battery_monitor_work, msecs_to_jiffies(30000));
}

static int smb_parse_batt_id(struct smb358_charger *chip)
{
	int rc = 0, rpull = 0, vref = 0;
	int64_t denom, batt_id_uv, numerator;
	struct device_node *node = chip->dev->of_node;
	struct qpnp_vadc_result result;

	rc = of_property_read_u32(node, "qcom,batt-id-vref-uv", &vref);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read batt-id-vref-uv rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,batt-id-rpullup-kohm", &rpull);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read batt-id-rpullup-kohm rc=%d\n", rc);
		return rc;
	}

	/* read battery ID */
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't read batt id channel=%d, rc=%d\n",
			LR_MUX2_BAT_ID, rc);
		return rc;
	}
	batt_id_uv = result.physical;

	if (batt_id_uv == 0) {
		/*vadc not correct or batt id line grounded, report 0 kohms */
		dev_warn(chip->dev, "batt_id_uv=0, batt-id grounded\n");
		return 0;
	}

	numerator = batt_id_uv * rpull * 1000;
	denom = vref  - batt_id_uv;

	/* batt id connector might be open, return 0 kohms */
	if (denom == 0)
		return 0;

	chip->connected_rid = div64_s64(numerator, denom);

	dev_dbg(chip->dev,
		"batt_id_voltage=%lld numerator=%lld denom=%lld connected_rid=%d\n",
		batt_id_uv, numerator, denom, chip->connected_rid);

	return 0;
}

static int smb_parse_dt(struct smb358_charger *chip)
{
	int rc;
	enum of_gpio_flags gpio_flags;
	struct device_node *node = chip->dev->of_node;
	int batt_present_degree_negative;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	chip->charging_disabled = of_property_read_bool(node,
					"qcom,charger-disabled");

	chip->inhibit_disabled = of_property_read_bool(node,
					"qcom,chg-inhibit-disabled");
	chip->chg_autonomous_mode = of_property_read_bool(node,
					"qcom,chg-autonomous-mode");

	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");

	chip->using_pmic_therm = of_property_read_bool(node,
						"qcom,using-pmic-therm");
	chip->pmic_vbat_sns = of_property_read_bool(node,
					"qcom,using-vbat-sns");
	chip->bms_controlled_charging = of_property_read_bool(node,
						"qcom,bms-controlled-charging");

	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	chip->chg_valid_gpio = of_get_named_gpio_flags(node,
				"qcom,chg-valid-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(chip->chg_valid_gpio))
		dev_dbg(chip->dev, "Invalid chg-valid-gpio");
	else
		chip->chg_valid_act_low = gpio_flags & OF_GPIO_ACTIVE_LOW;

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
						&chip->fastchg_current_max_ma);
	if (rc)
		chip->fastchg_current_max_ma = SMB358_FAST_CHG_MAX_MA;

	chip->iterm_disabled = of_property_read_bool(node,
					"qcom,iterm-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0) {
		chip->vfloat_mv = -EINVAL;
		pr_err("float-voltage-mv property missing, exit\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,recharge-mv",
						&chip->recharge_mv);
	if (rc < 0)
		chip->recharge_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
					"qcom,recharge-disabled");

	rc = of_property_read_u32(node, "qcom,cold-bat-decidegc",
						&chip->cold_bat_decidegc);
	if (rc < 0)
		chip->cold_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,hot-bat-decidegc",
						&chip->hot_bat_decidegc);
	if (rc < 0)
		chip->hot_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,warm-bat-decidegc",
						&chip->warm_bat_decidegc);

	rc |= of_property_read_u32(node, "qcom,cool-bat-decidegc",
						&chip->cool_bat_decidegc);

	if (!rc) {
		rc = of_property_read_u32(node, "qcom,cool-bat-mv",
						&chip->cool_bat_mv);

		rc |= of_property_read_u32(node, "qcom,warm-bat-mv",
						&chip->warm_bat_mv);

		rc |= of_property_read_u32(node, "qcom,cool-bat-ma",
						&chip->cool_bat_ma);

		rc |= of_property_read_u32(node, "qcom,warm-bat-ma",
						&chip->warm_bat_ma);
		if (rc)
			chip->jeita_supported = false;
		else
			chip->jeita_supported = true;
	}

	pr_debug("jeita_supported = %d", chip->jeita_supported);

	rc = of_property_read_u32(node, "qcom,bat-present-decidegc",
						&batt_present_degree_negative);
	if (rc < 0)
		chip->bat_present_decidegc = -EINVAL;
	else
		chip->bat_present_decidegc = -batt_present_degree_negative;

	if (of_get_property(node, "qcom,vcc-i2c-supply", NULL)) {
		chip->vcc_i2c = devm_regulator_get(chip->dev, "vcc-i2c");
		if (IS_ERR(chip->vcc_i2c)) {
			dev_err(chip->dev,
				"%s: Failed to get vcc_i2c regulator\n",
								__func__);
			return PTR_ERR(chip->vcc_i2c);
		}
	}

	chip->skip_usb_suspend_for_fake_battery = of_property_read_bool(node,
				"qcom,skip-usb-suspend-for-fake-battery");
	if (!chip->skip_usb_suspend_for_fake_battery) {
		if (!chip->vadc_dev) {
			dev_err(chip->dev,
				"VADC device not present with usb suspend on fake battery\n");
			return -EINVAL;
		}

		rc = smb_parse_batt_id(chip);
		if (rc) {
			dev_err(chip->dev,
				"failed to read batt-id rc=%d\n", rc);
			return rc;
		}
	}

	if (of_find_property(node, "qcom,thermal-mitigation",
				&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			dev_err(chip->dev, "thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	pr_notice("smb358 inhibit-disabled = %d, recharge-disabled = %d, recharge-mv = %d,\n",
		chip->inhibit_disabled, chip->recharge_disabled,
						chip->recharge_mv);
	pr_notice("smb358 vfloat-mv = %d, iterm-disabled = %d, iterm_ma = %d\n",
			chip->vfloat_mv, chip->iterm_disabled, chip->iterm_ma);
	pr_notice("smb358 fastchg-current = %d, charging-disabled = %d,\n",
			chip->fastchg_current_max_ma,
					chip->charging_disabled);
	pr_notice("smb358 disable-apsd = %d bms = %s cold-bat-degree = %d,\n",
		chip->disable_apsd, chip->bms_psy_name,
					chip->cold_bat_decidegc);
	pr_notice("smb358 hot-bat-degree = %d, bat-present-decidegc = %d\n",
		chip->hot_bat_decidegc, chip->bat_present_decidegc);
	return 0;
}

static int determine_initial_state(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, IRQ_B_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_B rc = %d\n", rc);
		goto fail_init_status;
	}

	rc = smb358_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_C rc = %d\n", rc);
		goto fail_init_status;
	}
	chip->batt_full = (reg & IRQ_C_TERM_BIT) ? true : false;

	rc = smb358_read_reg(chip, IRQ_A_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq A rc = %d\n", rc);
		return rc;
	}

	/* For current design, can ignore this */
	if (reg & IRQ_A_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_A_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_A_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_A_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = smb358_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_E_INPUT_UV_BIT) {
		chg_uv(chip, 1);
	} else {
		chg_uv(chip, 0);
		apsd_complete(chip, 1);
	}

	rc = smb358_read_reg(chip, IRQ_F_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
		goto fail_init_status;
	}
	if (reg & 0x04) {
		otg_det_handler(chip, 1);
	} else {
		otg_det_handler(chip, 0);
	}

	return 0;

fail_init_status:
	dev_err(chip->dev, "Couldn't determine initial status\n");
	return rc;
}

#if defined(CONFIG_DEBUG_FS)
static void smb358_debugfs_init(struct smb358_charger *chip)
{
	int rc;

	chip->debug_root = debugfs_create_dir("smb358", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &status_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create status debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cmd_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cmd debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("force_irq",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &force_irq_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create force_irq debug file rc =%d\n",
				rc);
		}

		ent = debugfs_create_file("battery_temp",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &battery_temp_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create battery_temp debug file rc =%d\n",
				rc);
		}

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cnfg irq_count file rc = %d\n",
				rc);
		}
	}
}
#else
static void smb358_debugfs_init(struct smb358_charger *chip)
{
}
#endif

#define SMB_I2C_VTG_MIN_UV 1800000
#define SMB_I2C_VTG_MAX_UV 1800000

#if defined(CONFIG_FB)
static int smb358_late_resume(struct smb358_charger *chip)
{
	int rc = -1;

	pr_notice("%s chg type %d\n", __func__, chip->supply_type);
	if ((chip->supply_type != POWER_SUPPLY_TYPE_USB_DCP && !chip->change_icl_in_suspend) || charger_boot)
		goto out;

	rc = vote(chip->fcc_votable, CHARGER_TYPE_FCC_VOTER, true, chip->fastchg_current_max_ma);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);

	rc = vote(chip->fcc_votable, SUSPEND_FCC_VOTER, false, 1300);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);

	rc = vote(chip->usb_icl_votable, SUSPEND_ICL_VOTER, false, 1200);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);

	chip->change_icl_in_suspend = false;

out:
	chip->early_resume_completed = true;
	return 0;
}

static int smb358_early_suspend(struct smb358_charger *chip)
{
	int rc = -1;

	pr_notice("%s chg type %d\n", __func__, chip->supply_type);
	if ((chip->supply_type != POWER_SUPPLY_TYPE_USB_DCP) || charger_boot)
		goto out;

	rc = vote(chip->fcc_votable, SUSPEND_FCC_VOTER, true, 1300);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);

	rc = vote(chip->fcc_votable, CHARGER_TYPE_FCC_VOTER, false, chip->fastchg_current_max_ma);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);

	rc = vote(chip->usb_icl_votable, SUSPEND_ICL_VOTER, true, 1200);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);

	chip->change_icl_in_suspend = true;

out:
	chip->early_resume_completed = false;
	return 0;
}

static void fb_notify_smb_resume_work(struct work_struct *work)
{
	struct smb358_charger *chip = container_of(work, struct smb358_charger, fb_notify_work);

	pr_notice("%s\n", __func__);

	smb358_late_resume(chip);
}
/*****************************************************************************
*  Name: fb_notifier_smb_callback
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fb_notifier_smb_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct smb358_charger *chip = container_of(self, struct smb358_charger, fb_notif);

	if (evdata && evdata->data && chip && chip->client) {
		blank = evdata->data;
		if (event == FB_EARLY_EVENT_BLANK &&	*blank == FB_BLANK_UNBLANK)
			schedule_work(&chip->fb_notify_work);
		else if (event == FB_EVENT_BLANK && *blank == FB_BLANK_POWERDOWN) {
			flush_work(&chip->fb_notify_work);
			smb358_early_suspend(chip);
		}
	}

	return 0;
}
#endif

static int smb358_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc, irq;
	struct smb358_charger *chip;
	struct power_supply *usb_psy;
	u8 reg = 0;
	const char *str = (const char *)saved_command_line;
	const char *ret = strnstr(str, "androidboot.mode=charger", strlen(str));

	if (ret != NULL) {
		charger_boot = true;
		pr_debug("%s CHARGER BOOT!\n", __func__);
	}

	pr_debug("%s\n", __func__);
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;
	i2c_set_clientdata(client, chip);

	if (of_find_property(chip->dev->of_node, "qcom,chg-vadc", NULL)) {
		/* early for VADC get, defer probe if needed */
		chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
		if (IS_ERR(chip->vadc_dev)) {
			rc = PTR_ERR(chip->vadc_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("vadc property configured incorrectly\n");
			return rc;
		}
	}

	rc = smb_parse_dt(chip);
	if (rc) {
		dev_err(&client->dev, "Couldn't parse DT nodes rc=%d\n", rc);
		return rc;
	}
	/* i2c pull up regulator configuration */
	if (chip->vcc_i2c) {
		if (regulator_count_voltages(chip->vcc_i2c) > 0) {
			rc = regulator_set_voltage(chip->vcc_i2c,
				SMB_I2C_VTG_MIN_UV, SMB_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&client->dev,
				"regulator vcc_i2c set failed, rc = %d\n",
								rc);
				return rc;
			}
		}

		rc = regulator_enable(chip->vcc_i2c);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_i2c enable failed rc = %d\n",
									rc);
			goto err_set_vtg_i2c;
		}
	}

	chip->fcc_votable = create_votable(&client->dev,
		"SMBCHG: fcc", VOTE_MIN, NUM_FCC_VOTER, 2000, set_fastchg_current_vote_cb);
	if (IS_ERR(chip->fcc_votable))
		return PTR_ERR(chip->fcc_votable);

	chip->usb_icl_votable = create_votable(&client->dev,
		"SMBCHG: usb_icl", VOTE_MIN, NUM_ICL_VOTER, 1800, set_usb_current_limit_vote_cb);
	if (IS_ERR(chip->usb_icl_votable))
		return PTR_ERR(chip->usb_icl_votable);

	mutex_init(&chip->therm_lvl_lock);
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->read_write_lock);
	mutex_init(&chip->path_suspend_lock);

	/* probe the device to check if its actually connected */
	rc = smb358_read_reg(chip, CHG_OTH_CURRENT_CTRL_REG, &reg);
	if (rc) {
		pr_err("Failed to detect SMB358, device absent, rc = %d\n", rc);
		goto err_set_vtg_i2c;
	}

	wake_lock_init(&chip->smb_monitor_wake_lock, WAKE_LOCK_SUSPEND, "smb_monitor_wake");
#ifdef CONFIG_WAKE_LOCK_ON_CHARGING
	wake_lock_init(&chip->smb_charge_wake_lock, WAKE_LOCK_SUSPEND, "smb_charge_wake");
	mutex_init(&chip->charge_mutex);
	chip->charge_wakelocked = 0;
#endif

	/* using adc_tm for implementing pmic therm */
	if (chip->using_pmic_therm) {
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("adc_tm property missing\n");
			return rc;
		}
	}

	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb358_battery_get_property;
	chip->batt_psy.set_property	= smb358_battery_set_property;
	chip->batt_psy.property_is_writeable = smb358_batt_property_is_writeable;
	chip->batt_psy.properties	= smb358_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smb358_battery_properties);
	chip->batt_psy.external_power_changed = smb358_external_power_changed;

	chip->resume_completed = true;
	chip->early_resume_completed = true;

	chip->battery_ov = false;
	chip->floating_volt_down = false;

	chip->supply_type = POWER_SUPPLY_TYPE_UNKNOWN;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev, "Couldn't register batt psy rc = %d\n",
				rc);
		goto err_set_vtg_i2c;
	}

	chip->dump_irq_status = true;
	dump_regs(chip);
	chip->dump_irq_status = false;

	smbchg_smoothing_init(chip);

	INIT_DELAYED_WORK(&chip->battery_monitor_work, smbchg_battery_monitor_work);    /*added by zte20170118*/
	schedule_delayed_work(&chip->battery_monitor_work, msecs_to_jiffies(5000));
	chip->aicl_completed = false;

	rc = smb358_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize smb358 ragulator rc=%d\n", rc);
		goto fail_regulator_register;
	}

	rc = smb358_hw_init(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't initialize hardware rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}

	rc = determine_initial_state(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't determine initial state rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}

	/* We will not use it by default */
	if (gpio_is_valid(chip->chg_valid_gpio)) {
		rc = gpio_request(chip->chg_valid_gpio, "smb358_chg_valid");
		if (rc) {
			dev_err(&client->dev,
				"gpio_request for %d failed rc=%d\n",
				chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		irq = gpio_to_irq(chip->chg_valid_gpio);
		if (irq < 0) {
			dev_err(&client->dev,
				"Invalid chg_valid irq = %d\n", irq);
			goto fail_chg_valid_irq;
		}
		rc = devm_request_threaded_irq(&client->dev, irq,
				NULL, smb358_chg_valid_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"smb358_chg_valid_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed request_irq irq=%d, gpio=%d rc=%d\n",
				irq, chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		smb358_chg_valid_handler(irq, chip);
		enable_irq_wake(irq);
	}

	chip->irq_gpio = of_get_named_gpio_flags(chip->dev->of_node,
				"qcom,irq-gpio", 0, NULL);
	pr_debug("smbchg %s irq gpio:%d.\n", __func__, chip->irq_gpio);
	/* STAT irq configuration */
	if (gpio_is_valid(chip->irq_gpio)) {
		rc = gpio_request(chip->irq_gpio, "smb358_irq");
		if (rc) {
			dev_err(&client->dev,
					"irq gpio request failed, rc=%d", rc);
			goto fail_smb358_hw_init;
		}
		rc = gpio_direction_input(chip->irq_gpio);
		if (rc) {
			dev_err(&client->dev,
					"set_direction for irq gpio failed\n");
			goto fail_irq_gpio;
		}

		irq = gpio_to_irq(chip->irq_gpio);
		if (irq < 0) {
			dev_err(&client->dev,
				"Invalid irq_gpio irq = %d\n", irq);
			goto fail_irq_gpio;
		}
		rc = devm_request_threaded_irq(&client->dev, irq, NULL,
				smb358_chg_stat_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"smb358_chg_stat_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed STAT irq=%d request rc = %d\n",
				irq, rc);
			goto fail_irq_gpio;
		}
		enable_irq_wake(irq);
	} else {
		goto fail_irq_gpio;
	}

	if (chip->using_pmic_therm) {
		if (!chip->jeita_supported) {
			/* add hot/cold temperature monitor */
			chip->adc_param.low_temp = chip->cold_bat_decidegc;
			chip->adc_param.high_temp = chip->hot_bat_decidegc;
		} else {
			chip->adc_param.low_temp = chip->cool_bat_decidegc;
			chip->adc_param.high_temp = chip->warm_bat_decidegc;
		}
		chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->adc_param.btm_ctx = chip;
		chip->adc_param.threshold_notification =
				smb_chg_adc_notification;
		chip->adc_param.channel = LR_MUX1_BATT_THERM;

		/* update battery missing info in tm_channel_measure*/
		rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
							&chip->adc_param);
		if (rc)
			pr_err("requesting ADC error %d\n", rc);
	}

	smb358_debugfs_init(chip);

#ifdef CONFIG_PROC_FS
	create_smb_proc_entry(chip);
#endif

#if defined(CONFIG_FB)
	INIT_WORK(&chip->fb_notify_work, fb_notify_smb_resume_work);
	chip->fb_notif.notifier_call = fb_notifier_smb_callback;
	rc = fb_register_client(&chip->fb_notif);
	if (rc)
		pr_err("[FB]Unable to register fb_notifier: %d", rc);
#endif

	dev_info(chip->dev, "SMB358 successfully probed. charger=%d, batt=%d\n",
			chip->chg_present, smb358_get_prop_batt_present(chip));
	return 0;

fail_chg_valid_irq:
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);
fail_irq_gpio:
	if (gpio_is_valid(chip->irq_gpio))
		gpio_free(chip->irq_gpio);
fail_smb358_hw_init:
	regulator_unregister(chip->otg_vreg.rdev);
fail_regulator_register:
	power_supply_unregister(&chip->batt_psy);
err_set_vtg_i2c:
	if (chip->vcc_i2c)
		if (regulator_count_voltages(chip->vcc_i2c) > 0)
			regulator_set_voltage(chip->vcc_i2c, 0,
						SMB_I2C_VTG_MAX_UV);
	return rc;
}

static int smb358_charger_remove(struct i2c_client *client)
{
	struct smb358_charger *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->batt_psy);
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);

	if (chip->vcc_i2c)
		regulator_disable(chip->vcc_i2c);

	mutex_destroy(&chip->irq_complete);
	debugfs_remove_recursive(chip->debug_root);
	return 0;
}

static void smb358_charger_shutdown(struct i2c_client *client)
{
	pr_notice("%s\n", __func__);
}

static int smb358_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	pr_debug("%s\n", __func__);

	for (i = 0; i < 2; i++) {
		rc = smb358_read_reg(chip, FAULT_INT_REG + i,
					&chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't save irq cfg regs rc = %d\n", rc);
	}

	/* enable wake up IRQs */
	rc = smb358_write_reg(chip, FAULT_INT_REG,
			FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_INPUT_UV_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set fault_irq_cfg rc = %d\n", rc);

	rc = smb358_write_reg(chip, STATUS_INT_REG,
			STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT |
			STATUS_INT_CHGING_BIT | STATUS_INT_INOK_BIT |
			STATUS_INT_OTG_DETECT_BIT | STATUS_INT_CHG_INHI_BIT);
	if (rc < 0)
		dev_err(chip->dev,
			"Couldn't set status_irq_cfg rc = %d\n", rc);

	mutex_lock(&chip->irq_complete);
	if (chip->vcc_i2c) {
		rc = regulator_disable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c disable failed rc=%d\n", rc);
			mutex_unlock(&chip->irq_complete);
			return rc;
		}
	}

	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);
	return 0;
}

static int smb358_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);

	pr_debug("%s\n", __func__);

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int smb358_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	pr_debug("%s\n", __func__);

	if (chip->vcc_i2c) {
		rc = regulator_enable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}
	}
	/* Restore IRQ config */
	for (i = 0; i < 2; i++) {
		rc = smb358_write_reg(chip, FAULT_INT_REG + i,
					chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't restore irq cfg regs rc=%d\n", rc);
	}

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	mutex_unlock(&chip->irq_complete);
	if (chip->irq_waiting) {
		smb358_chg_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	}
	if (chip->monitor_work_waiting) {
		pr_notice("%s monitor work in suspend, need perform\n", __func__);
		smbchg_battery_monitor(chip);
	}

	return 0;
}

static const struct dev_pm_ops smb358_pm_ops = {
	.suspend	= smb358_suspend,
	.suspend_noirq	= smb358_suspend_noirq,
	.resume		= smb358_resume,
};

static struct of_device_id smb358_match_table[] = {
	{ .compatible = "qcom,smb358-charger",},
	{ },
};

static const struct i2c_device_id smb358_charger_id[] = {
	{"smb358-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb358_charger_id);

static struct i2c_driver smb358_charger_driver = {
	.driver		= {
		.name		= "smb358-charger",
		.owner		= THIS_MODULE,
		.of_match_table = smb358_match_table,
		.pm		= &smb358_pm_ops,
	},
	.probe		= smb358_charger_probe,
	.remove		= smb358_charger_remove,
	.shutdown	= smb358_charger_shutdown,
	.id_table	= smb358_charger_id,
};

module_i2c_driver(smb358_charger_driver);

MODULE_DESCRIPTION("SMB358 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb358-charger");
