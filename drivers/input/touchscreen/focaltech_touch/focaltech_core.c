/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
*
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1	/* Early-suspend level */
#endif
#include "tpd_sys.h"
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define INTERVAL_READ_REG                   20	/* interval time per read reg unit:ms */
#define TIMEOUT_READ_REG                    300	/* timeout of read reg unit:ms */
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV                      2600000
#define FTS_VTG_MAX_UV                      3300000
#define FTS_I2C_VTG_MIN_UV                  1800000
#define FTS_I2C_VTG_MAX_UV                  1800000
#endif
#define FTS_READ_TOUCH_BUFFER_DIVIDED       0
/*****************************************************************************
* Global variable or extern global variabls/functions
******************************************************************************/
struct i2c_client *fts_i2c_client;
struct fts_ts_data *fts_wq_data;
struct input_dev *fts_input_dev;

#if FTS_DEBUG_EN
int g_show_log = 1;
#else
int g_show_log = 0;
#endif

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
char g_sz_debug[1024] = { 0 };
#endif

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static void fts_release_all_finger(void);
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief:   Read chip id until TP FW become valid, need call when reset/power on/resume...
*           1. Read Chip ID per INTERVAL_READ_REG(20ms)
*           2. Timeout: TIMEOUT_READ_REG(300ms)
*  Input:
*  Output:
*  Return: 0 - Get correct Device ID
*****************************************************************************/
int fts_wait_tp_to_valid(struct i2c_client *client)
{
	int ret = 0;
	int cnt = 0;
	u8 reg_value = 0;

	do {
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &reg_value);
		if ((ret < 0) || (reg_value != chip_types.chip_idh)) {
			FTS_INFO("TP Not Ready, ReadData = 0x%x", reg_value);
		} else if (reg_value == chip_types.chip_idh) {
			FTS_INFO("TP Ready, Device ID = 0x%x", reg_value);
			return 0;
		}
		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	/* error: not get correct reg data */
	return -EPERM;
}

/*****************************************************************************
*  Name: fts_recover_state
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_tp_state_recovery(struct i2c_client *client)
{
	/* wait tp stable */
	fts_wait_tp_to_valid(client);
	/* recover TP charger state 0x8B */
	/* recover TP glove state 0xC0 */
	/* recover TP cover state 0xC1 */
	fts_ex_mode_recovery(client);
	/* recover TP gesture state 0xD0 */
#if FTS_GESTURE_EN
	fts_gesture_recovery(client);
#endif
}

/*****************************************************************************
*  Name: fts_reset_proc
*  Brief: Execute reset operation
*  Input: hdelayms - delay time unit:ms
*  Output:
*  Return:
*****************************************************************************/
int fts_reset_proc(int hdelayms)
{
	gpio_direction_output(fts_wq_data->pdata->reset_gpio, 0);
	msleep(20);
	gpio_direction_output(fts_wq_data->pdata->reset_gpio, 1);
	msleep(hdelayms);

	return 0;
}

/*****************************************************************************
*  Name: fts_irq_disable
*  Brief: disable irq
*  Input:
*   sync:
*  Output:
*  Return:
*****************************************************************************/
void fts_irq_disable(void)
{
	unsigned long irqflags;

	spin_lock_irqsave(&fts_wq_data->irq_lock, irqflags);

	if (!fts_wq_data->irq_disable) {
		disable_irq_nosync(fts_wq_data->client->irq);
		fts_wq_data->irq_disable = 1;
	}

	spin_unlock_irqrestore(&fts_wq_data->irq_lock, irqflags);
}

/*****************************************************************************
*  Name: fts_irq_enable
*  Brief: enable irq
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_irq_enable(void)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&fts_wq_data->irq_lock, irqflags);

	if (fts_wq_data->irq_disable) {
		enable_irq(fts_wq_data->client->irq);
		fts_wq_data->irq_disable = 0;
	}

	spin_unlock_irqrestore(&fts_wq_data->irq_lock, irqflags);
}

/*****************************************************************************
*  Name: fts_input_dev_init
*  Brief: input dev init
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_input_dev_init(struct i2c_client *client,
			      struct fts_ts_data *data,
			      struct input_dev *input_dev,
			      struct fts_ts_platform_data *pdata)
{
	int err, len;

	FTS_FUNC_ENTER();

	/* Init and register Input device */
	input_dev->name = FTS_DRIVER_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	if (data->pdata->have_key) {
		FTS_DEBUG("set key capabilities");
		for (len = 0; len < data->pdata->key_number; len++) {
			input_set_capability(input_dev, EV_KEY,  data->pdata->keys[len]);
		}
	}
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if FTS_MT_PROTOCOL_B_EN
	input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0f, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,  pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,  pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

	err = input_register_device(input_dev);
	if (err) {
		FTS_ERROR("Input device registration failed");
		goto free_inputdev;
	}

	FTS_FUNC_EXIT();

	return 0;

free_inputdev:
	input_free_device(input_dev);
	FTS_FUNC_EXIT();
	return err;

}

/*****************************************************************************
* Power Control
*****************************************************************************/
#if FTS_POWER_SOURCE_CUST_EN
static int fts_power_source_init(struct fts_ts_data *data)
{
	int rc;

	FTS_FUNC_ENTER();

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		FTS_ERROR("Regulator get failed vdd rc=%d", rc);
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FTS_VTG_MIN_UV, FTS_VTG_MAX_UV);
		if (rc) {
			FTS_ERROR("Regulator set_vtg failed vdd rc=%d", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		FTS_ERROR("Regulator get failed vcc_i2c rc=%d", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FTS_I2C_VTG_MIN_UV,  FTS_I2C_VTG_MAX_UV);
		if (rc) {
			FTS_ERROR("Regulator set_vtg failed vcc_i2c rc=%d", rc);
			goto reg_vcc_i2c_put;
		}
	}

	FTS_FUNC_EXIT();
	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FTS_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	FTS_FUNC_EXIT();
	return rc;
}

static int fts_power_source_ctrl(struct fts_ts_data *data, int enable)
{
	int rc;

	FTS_FUNC_ENTER();
	if (enable) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			FTS_ERROR("Regulator vdd enable failed rc=%d", rc);
		}

		rc = regulator_enable(data->vcc_i2c);
		if (rc) {
			FTS_ERROR("Regulator vcc_i2c enable failed rc=%d", rc);
		}
	} else {
		rc = regulator_disable(data->vdd);
		if (rc) {
			FTS_ERROR("Regulator vdd disable failed rc=%d", rc);
		}
		rc = regulator_disable(data->vcc_i2c);
		if (rc) {
			FTS_ERROR("Regulator vcc_i2c disable failed rc=%d", rc);
		}
	}
	FTS_FUNC_EXIT();
	return 0;
}

#endif

/*****************************************************************************
*  Reprot related
*****************************************************************************/
/*****************************************************************************
*  Name: fts_release_all_finger
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_release_all_finger(void)
{
#if FTS_MT_PROTOCOL_B_EN
	unsigned int finger_count = 0;
#endif

	mutex_lock(&fts_wq_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
	for (finger_count = 0;
	     finger_count < fts_wq_data->pdata->max_touch_number; finger_count++) {
		input_mt_slot(fts_input_dev, finger_count);
		input_mt_report_slot_state(fts_input_dev, MT_TOOL_FINGER, false);
	}
#else
	input_mt_sync(fts_input_dev);
#endif
	input_report_key(fts_input_dev, BTN_TOUCH, 0);
	input_sync(fts_input_dev);
	mutex_unlock(&fts_wq_data->report_mutex);
}

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
static void fts_show_touch_buffer(u8 *buf, int point_num)
{
	int len = point_num * FTS_ONE_TCH_LEN;
	int count = 0;
	int i;

	memset(g_sz_debug, 0, 1024);
	if (len > (POINT_READ_BUF - 3)) {
		len = POINT_READ_BUF - 3;
	} else if (len == 0) {
		len += FTS_ONE_TCH_LEN;
	}
	count += snprintf(g_sz_debug, sizeof(g_sz_debug), "%02X,%02X,%02X", buf[0], buf[1], buf[2]);
	for (i = 0; i < len; i++) {
		count += snprintf(g_sz_debug + count, sizeof(g_sz_debug), ",%02X", buf[i + 3]);
	}
	FTS_DEBUG("buffer: %s", g_sz_debug);
}
#endif

static int fts_input_dev_report_key_event(struct ts_event *event,
					  struct fts_ts_data *data)
{
	int i;

	if (data->pdata->have_key) {
		if ((event->touch_point == 1 || event->point_num == 1) &&
		    (event->au16_y[0] == data->pdata->key_y_coord)) {

			if (event->point_num == 0) {
				FTS_DEBUG("Keys All Up!");
				for (i = 0; i < data->pdata->key_number; i++) {
					input_report_key(data->input_dev, data->pdata->keys[i], 0);
				}
			} else {
				for (i = 0; i < data->pdata->key_number; i++) {
					if (event->au16_x[0] > (data->pdata->key_x_coords[i] - FTS_KEY_WIDTH)
					    && event->au16_x[0] < (data->pdata->key_x_coords[i] + FTS_KEY_WIDTH)) {

						if (event->au8_touch_event[i] == 0 || event->au8_touch_event[i] == 2) {
							input_report_key(data->input_dev, data->pdata->keys[i], 1);
							FTS_DEBUG("Key%d(%d, %d) DOWN!", i,
								event->au16_x[0], event->au16_y[0]);
						} else {
							input_report_key(data->input_dev, data->pdata->keys[i], 0);
							FTS_DEBUG("Key%d(%d, %d) Up!", i,
								event->au16_x[0], event->au16_y[0]);
						}
						break;
					}
				}
			}
			input_sync(data->input_dev);
			return 0;
		}
	}

	return -EPERM;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_dev_report_b(struct ts_event *event,
				  struct fts_ts_data *data)
{
	int i = 0;
	int uppoint = 0;
	int touchs = 0;
	/* print tp point */
	static unsigned char finger_presence = 0;
	static unsigned short log_x = 0;
	static unsigned short log_y = 0;

	for (i = 0; i < event->touch_point; i++) {
		if (event->au8_finger_id[i] >= data->pdata->max_touch_number) {
			break;
		}
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i] == FTS_TOUCH_DOWN || event->au8_touch_event[i] == FTS_TOUCH_CONTACT) {
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if FTS_REPORT_PRESSURE_EN
#if FTS_FORCE_TOUCH_EN
			if (event->pressure[i] <= 0) {
				FTS_ERROR("[B]Illegal pressure: %d", event->pressure[i]);
				event->pressure[i] = 1;
			}
#else
			event->pressure[i] = 0x3f;
#endif
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
#endif

			if (event->area[i] <= 0) {
				FTS_ERROR("[B]Illegal touch-major: %d", event->area[i]);
				event->area[i] = 1;
			}
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);

			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			touchs |= BIT(event->au8_finger_id[i]);
			data->touchs |= BIT(event->au8_finger_id[i]);
#if 0 /* zhangjian delete */
#if FTS_REPORT_PRESSURE_EN
			FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
				  event->au8_finger_id[i], event->au16_x[i],
				  event->au16_y[i], event->pressure[i],
				  event->area[i]);
#else
			FTS_DEBUG("[B]P%d(%d, %d)[tm:%d] DOWN!",
				  event->au8_finger_id[i], event->au16_x[i],
				  event->au16_y[i], event->area[i]);
#endif
#endif
			/* print tp point */
			if (!finger_presence) {
				FTS_INFO("touch down\n");
				log_x = event->au16_x[i];
				log_y = event->au16_y[i];
				finger_presence = 1;
			}
		} else {
			uppoint++;
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#if FTS_REPORT_PRESSURE_EN
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0);
#endif
			data->touchs &= ~BIT(event->au8_finger_id[i]);
		}
	}

	if (unlikely(data->touchs ^ touchs)) {
		for (i = 0; i < data->pdata->max_touch_number; i++) {
			if (BIT(i) & (data->touchs ^ touchs)) {
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#if FTS_REPORT_PRESSURE_EN
				input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0);
#endif
			}
		}
	}
	data->touchs = touchs;
	if (event->touch_point == uppoint) {
		input_report_key(data->input_dev, BTN_TOUCH, 0);

		/* print tp point */
		if (finger_presence == 1) {
			FTS_INFO("touch up nums: %d, coord [%d:%d]\n", event->touch_point, log_x, log_y);
			log_x = 0;
			log_y = 0;
			finger_presence = 0;
		}
	} else {
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	}
	input_sync(data->input_dev);
	return 0;
}

#else
static int fts_input_dev_report_a(struct ts_event *event,
				  struct fts_ts_data *data)
{
	int i = 0;
	int uppoint = 0;
	int touchs = 0;

	for (i = 0; i < event->touch_point; i++) {

		if (event->au8_touch_event[i] == FTS_TOUCH_DOWN
		    || event->au8_touch_event[i] == FTS_TOUCH_CONTACT) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);
#if FTS_REPORT_PRESSURE_EN
#if FTS_FORCE_TOUCH_EN
			if (event->pressure[i] <= 0) {
				FTS_ERROR("[B]Illegal pressure: %d", event->pressure[i]);
				event->pressure[i] = 1;
			}
#else
			event->pressure[i] = 0x3f;
#endif
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
#endif

			if (event->area[i] <= 0) {
				FTS_ERROR("[B]Illegal touch-major: %d", event->area[i]);
				event->area[i] = 1;
			}
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);

			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);

			input_mt_sync(data->input_dev);
#if 0	/* zhangjian delete */
#if FTS_REPORT_PRESSURE_EN
			FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
				  event->au8_finger_id[i], event->au16_x[i],
				  event->au16_y[i], event->pressure[i],
				  event->area[i]);
#else
			FTS_DEBUG("[B]P%d(%d, %d)[tm:%d] DOWN!",
				  event->au8_finger_id[i], event->au16_x[i],
				  event->au16_y[i], event->area[i]);
#endif
#endif
		} else {
			uppoint++;
		}
	}

	data->touchs = touchs;
	if (event->touch_point == uppoint) {
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		input_mt_sync(data->input_dev);
	} else {
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	}

	input_sync(data->input_dev);

	return 0;
}
#endif

/*****************************************************************************
*  Name: fts_read_touchdata
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_read_touchdata(struct fts_ts_data *data)
{
	u8 buf[POINT_READ_BUF] = { 0 };
	u8 pointid = FTS_MAX_ID;
	int ret = -1;
	int i;

	struct ts_event *event = &(data->event);

#if FTS_GESTURE_EN
	{
		u8 state;

		if (data->suspended && data->wakeup_gesture_enter) {
			fts_i2c_read_reg(data->client, FTS_REG_GESTURE_EN, &state);
			if (state == 1) {
				fts_gesture_readdata(data->client);
				ret = 1;
				return ret;
			}
		}
	}
#endif

#if FTS_PSENSOR_EN
	if ((fts_sensor_read_data(data) != 0) && (data->suspended == 1)) {
		ret = 1;
		return ret;
	}
#endif

#if FTS_READ_TOUCH_BUFFER_DIVIDED
	memset(buf, 0xFF, POINT_READ_BUF);
	memset(event, 0, sizeof(struct ts_event));

	buf[0] = 0x00;
	ret = fts_i2c_read(data->client, buf, 1, buf, (3 + FTS_ONE_TCH_LEN));
	if (ret < 0) {
		FTS_ERROR("%s read touchdata failed.", __func__);
		return ret;
	}
	event->touch_point = 0;
	event->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
	if (event->point_num > data->pdata->max_touch_number)
		event->point_num = data->pdata->max_touch_number;

	if (event->point_num > 1) {
		buf[9] = 0x09;
		fts_i2c_read(data->client, buf + 9, 1, buf + 9, (event->point_num - 1) * FTS_ONE_TCH_LEN);
	}
#else
	ret = fts_i2c_read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		FTS_ERROR("[B]Read touchdata failed, ret: %d", ret);
		return ret;
	}
#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_queue_work();
#endif

	memset(event, 0, sizeof(struct ts_event));
	event->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
	if (event->point_num > data->pdata->max_touch_number)
		event->point_num = data->pdata->max_touch_number;
	event->touch_point = 0;
#endif

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
	fts_show_touch_buffer(buf, event->point_num);
#endif

	for (i = 0; i < data->pdata->max_touch_number; i++) {
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			goto OVER_FTS_MAX_ID;
		else
			event->touch_point++;

		event->au16_x[i] =
		    (s16) (buf[FTS_TOUCH_X_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F)
		    << 8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_ONE_TCH_LEN * i];
		event->au16_y[i] =
		    (s16) (buf[FTS_TOUCH_Y_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F)
		    << 8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_ONE_TCH_LEN * i];
		event->au8_touch_event[i] =
		    buf[FTS_TOUCH_EVENT_POS + FTS_ONE_TCH_LEN * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FTS_TOUCH_ID_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		event->area[i] =
		    (buf[FTS_TOUCH_AREA_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		event->pressure[i] =
		    (s16) buf[FTS_TOUCH_PRE_POS + FTS_ONE_TCH_LEN * i];

		if (event->area[i] == 0)
			event->area[i] = 0x09;

		if (event->pressure[i] == 0)
			event->pressure[i] = 0x3f;

		if ((event->au8_touch_event[i] == 0 || event->au8_touch_event[i] == 2)
		    && (event->point_num == 0)) {
			FTS_DEBUG("abnormal touch data from fw");
			ret = -1;
			return ret;
		}
	}
OVER_FTS_MAX_ID:
	if (event->touch_point == 0) {
		ret = -1;
		return ret;
	}
	return 0;
}

/*****************************************************************************
*  Name: fts_report_value
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_report_value(struct fts_ts_data *data)
{
	struct ts_event *event = &data->event;

	/*  FTS_DEBUG("point number: %d, touch point: %d", event->point_num,
		 event->touch_point); */

	if (fts_input_dev_report_key_event(event, data) == 0) {
		return;
	}
#if FTS_MT_PROTOCOL_B_EN
	fts_input_dev_report_b(event, data);
#else
	fts_input_dev_report_a(event, data);
#endif

	return;

}

/*****************************************************************************
*  Name: fts_ts_interrupt
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static irqreturn_t fts_ts_interrupt(int irq, void *dev_id)
{
	struct fts_ts_data *fts_ts = dev_id;
	int ret = -1;

	if (!fts_ts) {
		FTS_ERROR("[INTR]: Invalid fts_ts");
		return IRQ_HANDLED;
	}
#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(1);
#endif

	ret = fts_read_touchdata(fts_wq_data);

	if (ret == 0) {
		mutex_lock(&fts_wq_data->report_mutex);
		fts_report_value(fts_wq_data);
		mutex_unlock(&fts_wq_data->report_mutex);
	}
#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(0);
#endif

	return IRQ_HANDLED;
}

/*****************************************************************************
*  Name: fts_gpio_configure
*  Brief: Configure IRQ&RESET GPIO
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_gpio_configure(struct fts_ts_data *data)
{
	int err = 0;

	FTS_FUNC_ENTER();
	/* request irq gpio */
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		err = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
		if (err) {
			FTS_ERROR("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		err = gpio_direction_input(data->pdata->irq_gpio);
		if (err) {
			FTS_ERROR("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}
	/* request reset gpio */
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		err = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
		if (err) {
			FTS_ERROR("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		err = gpio_direction_output(data->pdata->reset_gpio, 1);
		if (err) {
			FTS_ERROR("[GPIO]set_direction for reset gpio failed");
			goto err_reset_gpio_dir;
		}
	}

	FTS_FUNC_EXIT();
	return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	FTS_FUNC_EXIT();
	return err;
}

/* zhangjian add */
#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"
static int fts_ts_pinctrl_init(struct fts_ts_data *fts_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	fts_data->ts_pinctrl = devm_pinctrl_get(&(fts_data->client->dev));
	if (IS_ERR_OR_NULL(fts_data->ts_pinctrl)) {
		retval = PTR_ERR(fts_data->ts_pinctrl);
		FTS_ERROR("Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	fts_data->pinctrl_state_active
	    = pinctrl_lookup_state(fts_data->ts_pinctrl, PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(fts_data->pinctrl_state_active)) {
		retval = PTR_ERR(fts_data->pinctrl_state_active);
		FTS_ERROR("Can not lookup %s pinstate %d\n", PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	fts_data->pinctrl_state_suspend
	    = pinctrl_lookup_state(fts_data->ts_pinctrl, PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(fts_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(fts_data->pinctrl_state_suspend);
		FTS_ERROR("Can not lookup %s pinstate %d\n", PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	fts_data->pinctrl_state_release
	    = pinctrl_lookup_state(fts_data->ts_pinctrl, PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(fts_data->pinctrl_state_release)) {
		retval = PTR_ERR(fts_data->pinctrl_state_release);
		FTS_ERROR("Can not lookup %s pinstate %d\n",
			  PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(fts_data->ts_pinctrl);
err_pinctrl_get:
	fts_data->ts_pinctrl = NULL;
	return retval;
}

/* add end */

/*****************************************************************************
*  Name: fts_get_dt_coords
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_get_dt_coords(struct device *dev, char *name,
			     struct fts_ts_platform_data *pdata)
{
	u32 coords[FTS_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FTS_COORDS_ARR_SIZE) {
		FTS_ERROR("invalid %s", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		FTS_ERROR("Unable to read %s", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		FTS_ERROR("unsupported property %s", name);
		return -EINVAL;
	}

	return 0;
}

/*****************************************************************************
*  Name: fts_parse_dt
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	FTS_FUNC_ENTER();

	rc = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		FTS_ERROR("Unable to get display-coords");

	/* key */
	pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
	if (pdata->have_key) {
		rc = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
		if (rc) {
			FTS_ERROR("Key number undefined!");
		}
		rc = of_property_read_u32_array(np, "focaltech,keys", pdata->keys, pdata->key_number);
		if (rc) {
			FTS_ERROR("Keys undefined!");
		}
		rc = of_property_read_u32(np, "focaltech,key-y-coord", &pdata->key_y_coord);
		if (rc) {
			FTS_ERROR("Key Y Coord undefined!");
		}
		rc = of_property_read_u32_array(np, "focaltech,key-x-coords", pdata->key_x_coords, pdata->key_number);
		if (rc) {
			FTS_ERROR("Key X Coords undefined!");
		}
		FTS_DEBUG("%d: (%d, %d, %d), [%d, %d, %d][%d]",
			  pdata->key_number, pdata->keys[0], pdata->keys[1],
			  pdata->keys[2], pdata->key_x_coords[0],
			  pdata->key_x_coords[1], pdata->key_x_coords[2],
			  pdata->key_y_coord);
	}

	/* reset, irq gpio info */
	pdata->reset_gpio =
	    of_get_named_gpio_flags(np, "focaltech,reset-gpio", 0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0) {
		FTS_ERROR("Unable to get reset_gpio");
	}

	pdata->irq_gpio =
	    of_get_named_gpio_flags(np, "focaltech,irq-gpio", 0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0) {
		FTS_ERROR("Unable to get irq_gpio");
	}

	rc = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
	if (!rc) {
		pdata->max_touch_number = temp_val;
		FTS_DEBUG("max_touch_number=%d", pdata->max_touch_number);
	} else {
		FTS_ERROR("Unable to get max-touch-number");
		pdata->max_touch_number = FTS_MAX_POINTS;
	}

	FTS_FUNC_EXIT();
	return 0;
}

#if defined(CONFIG_FB)
static void fts_tpd_resume_work_callback(struct work_struct *work)
{
	struct fts_ts_data *fts_data =
	    container_of(work, struct fts_ts_data, fb_tpd_resume_work);

	FTS_FUNC_ENTER();
	fts_ts_resume(&fts_data->client->dev);
	fts_data->fb_tpd_suspend_flag = 0;
}

/*****************************************************************************
*  Name: fb_notifier_callback
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	int err = 0;
	struct fts_ts_data *fts_data =
	    container_of(self, struct fts_ts_data, fb_notif);

#if 0/* zhangjian modify */
	if (evdata && evdata->data && event == FB_EVENT_BLANK && fts_data && fts_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			fts_ts_resume(&fts_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			fts_ts_suspend(&fts_data->client->dev);
	}
#else
	if (evdata && evdata->data && event == FB_EVENT_BLANK && fts_data && fts_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			if (fts_data->fb_tpd_suspend_flag) {
				err = queue_work(fts_data->fb_tpd_resume_wq, &fts_data->fb_tpd_resume_work);
				if (!err) {
					FTS_DEBUG("start fb_tpd_resume_wq failed\n");
					return err;
				}
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
			FTS_DEBUG("lcd off notifier.\n");
			err = cancel_work_sync(&fts_data->fb_tpd_resume_work);
			if (!err)
				FTS_DEBUG("cancel fb_tpd_resume_wq err = %d\n", err);
			fts_ts_suspend(&fts_data->client->dev);
			fts_data->fb_tpd_suspend_flag = 1;
		}
	}
#endif
	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*****************************************************************************
*  Name: fts_ts_early_suspend
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_ts_early_suspend(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler, struct fts_ts_data, early_suspend);

	fts_ts_suspend(&data->client->dev);
}

/*****************************************************************************
*  Name: fts_ts_late_resume
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_ts_late_resume(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler, struct fts_ts_data, early_suspend);

	fts_ts_resume(&data->client->dev);
}
#endif
/* zhangjian add */
#define FTS_REG_ID		0xA3
#define FTS_REG_FW_VER		0xA6
#define FTS_REG_FW_VENDOR_ID	0xA8
static int tpd_init_tpinfo(struct tpd_classdev_t *cdev)
{
	u8 fwver_in_chip = 0;
	u8 vendorid_in_chip = 0;
	u8 chipid_in_chip = 0;
	u8 retry = 0;

	while (retry++ < 5) {
		fts_i2c_read_reg(fts_i2c_client, FTS_REG_ID, &chipid_in_chip);
		fts_i2c_read_reg(fts_i2c_client, FTS_REG_FW_VENDOR_ID, &vendorid_in_chip);
		fts_i2c_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver_in_chip);
		if ((chipid_in_chip != 0) && (vendorid_in_chip != 0) && (fwver_in_chip != 0)) {
			FTS_DEBUG("chip_id = %x,vendor_id =%x,fw_version=%x .\n",
			     chipid_in_chip, vendorid_in_chip, fwver_in_chip);
			break;
		}
		FTS_DEBUG("chip_id = %x,vendor_id =%x,fw_version=%x .\n",
		      chipid_in_chip, vendorid_in_chip, fwver_in_chip);
		msleep(20);
	}

	strlcpy(cdev->ic_tpinfo.tp_name, "Focal", 10);
	cdev->ic_tpinfo.chip_model_id = TS_CHIP_FOCAL;

	cdev->ic_tpinfo.chip_part_id = chipid_in_chip;
	cdev->ic_tpinfo.module_id = vendorid_in_chip;
	cdev->ic_tpinfo.chip_ver = 0;
	cdev->ic_tpinfo.firmware_ver = fwver_in_chip;
	cdev->ic_tpinfo.i2c_type = 0;
	cdev->ic_tpinfo.i2c_addr = 0x38;

	return 0;
}

static int tpd_get_wakegesture(struct tpd_classdev_t *cdev)
{
	struct fts_ts_data *data = (struct fts_ts_data *)cdev->private;

	FTS_INFO("%s wakeup_gesture_enable val is:%d.\n", __func__, data->wakeup_gesture_enable);
	cdev->b_gesture_enable = data->wakeup_gesture_enable;
	return 0;
}

static int tpd_enable_wakegesture(struct tpd_classdev_t *cdev, int enable)
{
	struct fts_ts_data *data = (struct fts_ts_data *)cdev->private;

	FTS_INFO("%s previous val is:%d, current val is:%d.\n", __func__, data->wakeup_gesture_enable, enable);

	data->wakeup_gesture_enable = enable;

	return enable;
}

static int tpd_register_fw_class(struct fts_ts_data *data)
{
	tpd_fw_cdev.private = (void *)data;
	tpd_fw_cdev.get_tpinfo = tpd_init_tpinfo;
	tpd_fw_cdev.get_gesture = tpd_get_wakegesture;
	tpd_fw_cdev.wake_gesture = tpd_enable_wakegesture;
	return 0;
}

/* add end */
/*****************************************************************************
*  Name: fts_ts_probe
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct fts_ts_platform_data *pdata;
	struct fts_ts_data *data;
	struct input_dev *input_dev;
	int err;
	u8 reg_value = 0;

	FTS_FUNC_ENTER();
	/* zhangjian add for TP compat */
	if (tpd_fw_cdev.TP_have_registered) {
		FTS_DEBUG("TP have registered by other TP.\n");
		return -EPERM;
	}
	/* 1. Get Platform data */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(struct fts_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			FTS_ERROR("[MEMORY]Failed to allocate memory");
			FTS_FUNC_EXIT();
			return -ENOMEM;
		}
		err = fts_parse_dt(&client->dev, pdata);
		if (err) {
			FTS_ERROR("[DTS]DT parsing failed");
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		FTS_ERROR("Invalid pdata");
		FTS_FUNC_EXIT();
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FTS_ERROR("I2C not supported");
		FTS_FUNC_EXIT();
		devm_kfree(&client->dev, pdata);
		return -ENODEV;
	}

	data =
	    devm_kzalloc(&client->dev, sizeof(struct fts_ts_data), GFP_KERNEL);
	if (!data) {
		FTS_ERROR("[MEMORY]Failed to allocate memory");
		FTS_FUNC_EXIT();
		devm_kfree(&client->dev, pdata);
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		FTS_ERROR("[INPUT]Failed to allocate input device");
		FTS_FUNC_EXIT();
		devm_kfree(&client->dev, data);
		devm_kfree(&client->dev, pdata);
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	fts_wq_data = data;
	fts_i2c_client = client;
	fts_input_dev = input_dev;

	spin_lock_init(&fts_wq_data->irq_lock);
	mutex_init(&fts_wq_data->report_mutex);

	fts_input_dev_init(client, data, input_dev, pdata);

#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_init(data);
	fts_power_source_ctrl(data, 1);
#endif
/* zhangjian add  */
	data->wakeup_gesture_enable = false;
	data->wakeup_gesture_enter = false;
	data->fw_loading = false;
	err = fts_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		err = pinctrl_select_state(data->ts_pinctrl, data->pinctrl_state_active);
		if (err < 0) {
			FTS_ERROR("failed to select pin to active state");
		}
	}
/* add end */
	err = fts_gpio_configure(data);
	if (err < 0) {
		FTS_ERROR("[GPIO]Failed to configure the gpios");
		goto free_gpio;
	}

	fts_reset_proc(200);
	fts_ctpm_get_upgrade_array();
	err = fts_wait_tp_to_valid(client);
	err = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &reg_value);
	if (err < 0) {		/*  zhangjian add */
		FTS_ERROR("fts i2c test read failed");
		goto free_gpio;
	}
	err = request_threaded_irq(client->irq, NULL, fts_ts_interrupt,
				   pdata->irq_gpio_flags | IRQF_ONESHOT |
				   IRQF_TRIGGER_FALLING,
				   client->dev.driver->name, data);
	if (err) {
		FTS_ERROR("Request irq failed!");
		goto free_gpio;
	}

	fts_irq_disable();

#if FTS_PSENSOR_EN
	if (fts_sensor_init(data) != 0) {
		FTS_ERROR("fts_sensor_init failed!");
		err = -ENOMEM;
		goto free_gpio;
	}
#endif

#if FTS_APK_NODE_EN
	fts_create_apk_debug_channel(client);
#endif

#if FTS_SYSFS_NODE_EN
	fts_create_sysfs(client);
#endif

#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_init();
#endif

	fts_ex_mode_init(client);

#if FTS_GESTURE_EN
	fts_gesture_init(input_dev, client);
#endif

#if FTS_ESDCHECK_EN
	fts_esdcheck_init();
#endif

	fts_irq_enable();

#if FTS_AUTO_UPGRADE_EN
	fts_ctpm_upgrade_init();
#endif
	tpd_register_fw_class(data);	/* zhangjian add */
#if FTS_TEST_EN
	fts_test_init(client);
#endif

#if defined(CONFIG_FB)

	data->fb_tpd_suspend_flag = 0;
	data->fb_tpd_resume_wq = create_singlethread_workqueue("Fts_touch_resume");
	INIT_WORK(&data->fb_tpd_resume_work, fts_tpd_resume_work_callback);
	data->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&data->fb_notif);
	if (err)
		FTS_ERROR("[FB]Unable to register fb_notifier: %d", err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
	data->early_suspend.suspend = fts_ts_early_suspend;
	data->early_suspend.resume = fts_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
	tpd_fw_cdev.TP_have_registered = true;	/* zhangjian add for TP compat */
	FTS_FUNC_EXIT();
	return 0;

free_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
/* zhangjian add */
	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(data->ts_pinctrl, data->pinctrl_state_release);
			if (err)
				FTS_ERROR
				    ("failed to select relase pinctrl state\n");
		}
	}
#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_ctrl(data, 0);
#endif
	input_unregister_device(input_dev);
	input_dev = NULL;
	devm_kfree(&client->dev, data);
	devm_kfree(&client->dev, pdata);
/* add end */
	return err;

}

/*****************************************************************************
*  Name: fts_ts_remove
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *data = i2c_get_clientdata(client);

	FTS_FUNC_ENTER();
	cancel_work_sync(&data->touch_event_work);

#if FTS_PSENSOR_EN
	fts_sensor_remove(data);
#endif

#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_exit();
#endif

#if FTS_APK_NODE_EN
	fts_release_apk_debug_channel();
#endif

#if FTS_SYSFS_NODE_EN
	fts_remove_sysfs(client);
#endif

	fts_ex_mode_exit(client);

#if FTS_AUTO_UPGRADE_EN
	cancel_work_sync(&fw_update_work);
#endif

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		FTS_ERROR("Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	input_unregister_device(data->input_dev);

#if FTS_TEST_EN
	fts_test_exit(client);
#endif

#if FTS_ESDCHECK_EN
	fts_esdcheck_exit();
#endif

	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
*  Name: fts_ts_suspend
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_ts_suspend(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	int retval = 0;
	int err;

	FTS_FUNC_ENTER();
	if (data->suspended) {
		FTS_INFO("Already in suspend state");
		FTS_FUNC_EXIT();
		return 0;
	}
	if (data->fw_loading) {
		FTS_INFO("fw upgrade in process, can't suspend");
		return 0;
	}
#if FTS_ESDCHECK_EN
	fts_esdcheck_suspend();
#endif

#if FTS_GESTURE_EN
	/* zhangjian add for tp wakeup gesture */
	if (data->wakeup_gesture_enable) {
		data->wakeup_gesture_enter = true;
		retval = fts_gesture_suspend(data->client);
		if (retval == 0) {
			/* Enter into gesture mode(suspend) */
			retval = enable_irq_wake(fts_wq_data->client->irq);
			if (retval)
				FTS_ERROR("%s: set_irq_wake failed", __func__);
			data->suspended = true;
			FTS_FUNC_EXIT();
			return 0;
		}
	}
#endif
	/* zhangjian add */
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl, data->pinctrl_state_suspend);
		if (err < 0)
			FTS_ERROR("Cannot get suspend pinctrl state\n");
	}
	/* add end */
#if FTS_PSENSOR_EN
	if (fts_sensor_suspend(data) != 0) {
		enable_irq_wake(data->client->irq);
		data->suspended = true;
		return 0;
	}
#endif

	fts_irq_disable();

	/* TP enter sleep mode */
	retval =
	    fts_i2c_write_reg(data->client, FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
	if (retval < 0) {
		FTS_ERROR("Set TP to sleep mode fail, ret=%d!", retval);
	}
	data->suspended = true;
#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_ctrl(data, 0);
#endif
	FTS_FUNC_EXIT();

	return 0;
}

/*****************************************************************************
*  Name: fts_ts_resume
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fts_ts_resume(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	int err;

	FTS_FUNC_ENTER();
	if (!data->suspended) {
		FTS_DEBUG("Already in awake state");
		FTS_FUNC_EXIT();
		return 0;
	}

	fts_release_all_finger();
#if FTS_POWER_SOURCE_CUST_EN
	if (!data->wakeup_gesture_enter) {
		fts_power_source_ctrl(data, 1);
	}
#endif
	/* zhangjian add */
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl, data->pinctrl_state_active);
		if (err < 0)
			FTS_ERROR("Cannot get active pinctrl state\n");
	}
	/* add end */
#if (!FTS_CHIP_IDC)
	fts_reset_proc(200);
#endif

	fts_tp_state_recovery(data->client);

#if FTS_ESDCHECK_EN
	fts_esdcheck_resume();
#endif

#if FTS_GESTURE_EN
	if (data->wakeup_gesture_enter) {
		data->wakeup_gesture_enter = false;
		if (fts_gesture_resume(data->client) == 0) {
			int err;

			err = disable_irq_wake(data->client->irq);
			if (err)
				FTS_ERROR("%s: disable_irq_wake failed", __func__);
			data->suspended = false;
			FTS_FUNC_EXIT();
			return 0;
		}
	}
#endif

#if FTS_PSENSOR_EN
	if (fts_sensor_resume(data) != 0) {
		disable_irq_wake(data->client->irq);
		data->suspended = false;
		FTS_FUNC_EXIT();
		return 0;
	}
#endif

	data->suspended = false;

	fts_irq_enable();

	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
* I2C Driver
*****************************************************************************/
static const struct i2c_device_id fts_ts_id[] = {
	{FTS_DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, fts_ts_id);

static struct of_device_id fts_match_table[] = {
	{.compatible = "focaltech,fts_ts",},
	{},
};

static struct i2c_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		   .name = FTS_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = fts_match_table,
		   },
	.id_table = fts_ts_id,
};

/*****************************************************************************
*  Name: fts_ts_init
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int __init fts_ts_init(void)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ret = i2c_add_driver(&fts_ts_driver);
	if (ret != 0) {
		FTS_ERROR("Focaltech touch screen driver init failed!");
	}
	FTS_FUNC_EXIT();
	return ret;
}

/*****************************************************************************
*  Name: fts_ts_exit
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
