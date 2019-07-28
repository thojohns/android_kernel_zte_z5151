/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * Revision: 24538
 * Date: 2018-03-15 18:59:07 +0800 (2018)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */


#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include "nt36xxx.h"

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"
#define NVT_CHARGER "tp_charger_switch"

#define I2C_TANSFER_LENGTH  64

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

#define XDATA_SECTOR_SIZE   256

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};
static int32_t xdata_i[2048] = {0};
static int32_t xdata_q[2048] = {0};

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;
static struct proc_dir_entry *NVT_proc_information_entry;
#if WAKEUP_GESTURE
static struct proc_dir_entry *NVT_proc_wake_gesture_entry;
#endif
#if NVT_CHARGER_SWITCH
static struct proc_dir_entry *NVT_proc_charger_entry;
#endif

extern int g_nvt_vendor_id;
/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode(uint8_t mode)
{
	uint8_t buf[8] = {0};

	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
		msleep(20);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe(void)
{
	uint8_t buf[8] = {0};

	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[I2C_TANSFER_LENGTH + 1] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = ts->x_num * ts->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		buf[0] = 0xFF;
		buf[1] = ((head_addr + XDATA_SECTOR_SIZE * i) >> 16) & 0xFF;
		buf[2] = ((head_addr + XDATA_SECTOR_SIZE * i) >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		for (j = 0; j < (XDATA_SECTOR_SIZE / I2C_TANSFER_LENGTH); j++) {
			buf[0] = I2C_TANSFER_LENGTH * j;
			CTP_I2C_READ(ts->client, I2C_FW_Address, buf, I2C_TANSFER_LENGTH + 1);

			for (k = 0; k < I2C_TANSFER_LENGTH; k++)
				xdata_tmp[XDATA_SECTOR_SIZE * i + I2C_TANSFER_LENGTH * j + k] = buf[k + 1];
		}
	}

	if (residual_len != 0) {
		buf[0] = 0xFF;
		buf[1] = ((xdata_addr + data_len - residual_len) >> 16) & 0xFF;
		buf[2] = ((xdata_addr + data_len - residual_len) >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		for (j = 0; j < (residual_len / I2C_TANSFER_LENGTH + 1); j++) {
			buf[0] = I2C_TANSFER_LENGTH * j;
			CTP_I2C_READ(ts->client, I2C_FW_Address, buf, I2C_TANSFER_LENGTH + 1);

			for (k = 0; k < I2C_TANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) +
					I2C_TANSFER_LENGTH * j + k] = buf[k + 1];
			}
		}
	}

	for (i = 0; i < (data_len / 2); i++)
		xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] +
			256 * xdata_tmp[dummy_len + i * 2 + 1]);

#if TOUCH_KEY_NUM > 0
	buf[0] = 0xFF;
	buf[1] = (xdata_btn_addr >> 16) & 0xFF;
	buf[2] = ((xdata_btn_addr >> 8) & 0xFF);
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, (TOUCH_KEY_NUM * 2 + 1));

	/* 2bytes-to-1data */
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts->x_num * ts->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data from IQ to rss function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata_rss(uint32_t xdata_i_addr, uint32_t xdata_q_addr,
	uint32_t xdata_btn_i_addr, uint32_t xdata_btn_q_addr)
{
	int i = 0;

	nvt_read_mdata(xdata_i_addr, xdata_btn_i_addr);
	memcpy(xdata_i, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	nvt_read_mdata(xdata_q_addr, xdata_btn_q_addr);
	memcpy(xdata_q, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	for (i = 0; i < (ts->x_num * ts->y_num + TOUCH_KEY_NUM); i++)
		xdata[i] = (int32_t)int_sqrt((unsigned long)(xdata_i[i] * xdata_i[i]) +
			(unsigned long)(xdata_q[i] * xdata_q[i]));
}

/*******************************************************
Description:
	Novatek touchscreen get meta data function.

return:
	n.a.
*******************************************************/
void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
	*m_x_num = ts->x_num;
	*m_y_num = ts->y_num;
	memcpy(buf, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n",
		ts->fw_ver, ts->x_num, ts->y_num, ts->max_button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts->y_num; i++) {
		for (j = 0; j < ts->x_num; j++) {
			seq_printf(m, "%5d, ", xdata[i * ts->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%5d, ", xdata[ts->x_num * ts->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_puts(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
}

const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int32_t c_information_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Manufacturer: Novatek\n");

	seq_printf(m, "chip type: NT%X\n", ts->chip_id);

	switch (g_nvt_vendor_id) {
	case NVT_VENDOR_ID_0:
		seq_printf(m, "sensor partner: %s\n", NVT_VENDOR_0_NAME);
		break;
	case NVT_VENDOR_ID_1:
		seq_printf(m, "sensor partner: %s\n", NVT_VENDOR_1_NAME);
		break;
	case NVT_VENDOR_ID_2:
		seq_printf(m, "sensor partner: %s\n", NVT_VENDOR_2_NAME);
		break;
	case NVT_VENDOR_ID_3:
		seq_printf(m, "sensor partner: %s\n", NVT_VENDOR_3_NAME);
		break;
	default:
		seq_printf(m, "sensor partner: %s\n", "UNKNOWN");
		break;
	}

	seq_printf(m, "FW Revision: %02d\n", ts->fw_ver);

	return 0;
}


const struct seq_operations nvt_information_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_information_show
};

static int32_t nvt_information_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock))
		return -ERESTARTSYS;

	NVT_LOG("++\n");

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_information_seq_ops);
}

static const struct file_operations nvt_information_ops = {
	.owner = THIS_MODULE,
	.open = nvt_information_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};


/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		nvt_read_mdata_rss(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_Q_ADDR,
				ts->mmap->BASELINE_BTN_ADDR, ts->mmap->BASELINE_BTN_Q_ADDR);
	} else {
		nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata_rss(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_PIPE0_Q_ADDR,
				ts->mmap->RAW_BTN_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_rss(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_PIPE1_Q_ADDR,
				ts->mmap->RAW_BTN_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (ts->carrier_system) {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata_rss(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_PIPE0_Q_ADDR,
				ts->mmap->DIFF_BTN_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_rss(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_PIPE1_Q_ADDR,
				ts->mmap->DIFF_BTN_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);
	}

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

#if NVT_CHARGER_SWITCH
#define CMD_CHARGER_ON	(0x53)
#define CMD_CHARGER_OFF (0x51)

int32_t nvt_set_charger(uint8_t charger_on_off)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;

	NVT_LOG("++\n");
	NVT_LOG("set charger: %d\n", charger_on_off);

	msleep(35);
	/* set xdata index to EVENT BUF ADDR */
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		goto nvt_set_charger_out;
	}

	if (charger_on_off == CHARGER_ON) {
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = CMD_CHARGER_ON;
		ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Write set charger command fail!\n");
			goto nvt_set_charger_out;
		}
	} else if (charger_on_off == CHARGER_OFF) {
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = CMD_CHARGER_OFF;
		ret = CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Write set charger command fail!\n");
			goto nvt_set_charger_out;
		}
	} else {
		NVT_ERR("Invalid charger parameter!\n");
		ret = -1;
	}

nvt_set_charger_out:
	NVT_LOG("--\n");
	return ret;
}

/* 0: no charger; 1: charger; 2: status is no chage */
NVT_CHARGER_STATE NVT_CHARGE_SWITCH_FLAG = NO_CHARGER_CHANGE_SWITCH;
extern uint8_t bTouchIsAwake;
static ssize_t nvt_charger_store(struct file *file, const char *buffer, size_t count, loff_t *pos)
{
	int ret;
	unsigned int input;
	char data_buf[512] = {0};

	ret = copy_from_user(data_buf, buffer, count);
	if (ret)
		return -EINVAL;
	ret = kstrtouint(data_buf, 0, &input);
	if (ret)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (bTouchIsAwake == 0) {
		NVT_CHARGE_SWITCH_FLAG = input;
		NVT_LOG("input: %d when tp is suspend\n", NVT_CHARGE_SWITCH_FLAG);
		return count;
	}

	if (input == 1) {
		if (mutex_lock_interruptible(&ts->lock)) {
			return -ERESTARTSYS;
		}
		NVT_LOG("input: %d\n", input);
		nvt_set_charger(CHARGER_ON);
		mutex_unlock(&ts->lock);
	} else if (input == 0) {
		if (mutex_lock_interruptible(&ts->lock)) {
			return -ERESTARTSYS;
		}
		NVT_LOG("input: %d\n", input);
		nvt_set_charger(CHARGER_OFF);
		mutex_unlock(&ts->lock);
	} else {
		NVT_ERR("Invalid input: %d\n", input);
		return -EINVAL;
	}

	return count;
}

static int nvt_charger_show(struct seq_file *m, void *v)
{
	return 0;
}

static int32_t nvt_charger_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvt_charger_show, NULL);
}

static const struct file_operations nvt_charger_fops = {
	.owner = THIS_MODULE,
	.open = nvt_charger_open,
	.read = seq_read,
	.write = nvt_charger_store,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

#if WAKEUP_GESTURE
bool nvt_gesture_enable;
static ssize_t tp_wake_gesture_read(struct file *file,
					 char __user *buffer, size_t count, loff_t *offset)
{
	ssize_t len = 0;
	uint8_t data_buf[10] = {0};

	pr_notice("tpd: %s val:%d.\n", __func__, nvt_gesture_enable);

	len = snprintf(data_buf, sizeof(data_buf), "%u\n", nvt_gesture_enable);
	return simple_read_from_buffer(buffer, count, offset, data_buf, len);
}

static ssize_t tp_wake_gesture_write(struct file *file,
				const char __user *buffer, size_t len, loff_t *off)
{
	int ret;
	unsigned int input;
	char data_buf[10] = {0};

	ret = copy_from_user(data_buf, buffer, len);
	if (ret)
		return -EINVAL;
	ret = kstrtouint(data_buf, 0, &input);
	if (ret)
		return -EINVAL;
	input = input > 0 ? 1 : 0;
	pr_notice("tpd: %s val %d.\n", __func__, input);
	nvt_gesture_enable = input;
	return len;
}

static const struct file_operations nvt_wake_gesture_fops = {
	.owner = THIS_MODULE,
	.read = tp_wake_gesture_read,
	.write = tp_wake_gesture_write,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
int32_t nvt_extra_proc_init(void)
{
	struct proc_dir_entry *dir;

	dir = proc_mkdir("touchscreen", NULL);
	NVT_proc_information_entry = proc_create("ts_information", 0664, dir, &nvt_information_ops);
	if (NVT_proc_information_entry == NULL) {
		NVT_ERR("create proc/touchscreen/ts_information failed!\n");
		return -ENOMEM;
	}

	NVT_LOG("create proc/touchscreen/ts_information Succeeded!\n");

#if WAKEUP_GESTURE
	NVT_proc_wake_gesture_entry = proc_create("wake_gesture", 0444, dir, &nvt_wake_gesture_fops);
	if (NVT_proc_wake_gesture_entry == NULL) {
		NVT_ERR("create proc/touchscreen/wake_gesture Failed!\n");
		return -ENOMEM;
	}

	NVT_LOG("create proc/touchscreen/wake_gesture Succeeded!\n");
#endif

	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, dir, &nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/touchscreen/nvt_fw_version Failed!\n");
		return -ENOMEM;
	}

	NVT_LOG("create proc/touchscreen/nvt_fw_version Succeeded!\n");

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, dir, &nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/touchscreen/nvt_baseline Failed!\n");
		return -ENOMEM;
	}

	NVT_LOG("create proc/touchscreen/nvt_baseline Succeeded!\n");

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, dir, &nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/touchscreen/nvt_raw Failed!\n");
		return -ENOMEM;
	}

	NVT_LOG("create proc/touchscreen/nvt_raw Succeeded!\n");

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, dir, &nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/touchscreen/nvt_diff Failed!\n");
		return -ENOMEM;
	}

	NVT_LOG("create proc/touchscreen/nvt_diff Succeeded!\n");

#if NVT_CHARGER_SWITCH
	NVT_proc_charger_entry = proc_create(NVT_CHARGER, 0220, dir, &nvt_charger_fops);
	if (NVT_proc_charger_entry == NULL) {
		NVT_ERR("create proc/nvt_charger Failed!\n");
		return -ENOMEM;
	}
#endif

	NVT_LOG("create proc/nvt_charger Succeeded!\n");

	return 0;
}
#endif
