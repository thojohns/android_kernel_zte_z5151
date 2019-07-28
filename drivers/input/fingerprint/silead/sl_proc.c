/*
 * =====================================================================================
 *
 *		 Filename:	sl_proc.c
 *
 *		Description:
 *
 *			Version:	1.0
 *			Created:	2015/03/10
 *		 Revision:	none
 *		 Compiler:	gcc
 *
 *			 Author:	YOUR NAME (),
 *			Company:
 *
 * =====================================================================================
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/compat.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>

#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include "sl_proc.h"
/*
 *proc sl_fp command
 * start : start cali mode ,disable capture
 * stop : stop cali mode ,start capture
 * write addr data:  write data to reg, Ex. write FF08000C 0000009F >sl_fp
 * read reg reg_val: read reg val; Ex. read reg FF08000C >sl_fp
 * read frame:	read frame from silead device:
 *
 *
 * */
#define SL_FP_PROC_FILE "sl_fp"
#ifdef CONFIG_PROC_FS
#undef CONFIG_PROC_FS
#endif
#define CONFIG_PROC_FS
static struct proc_dir_entry *sl_proc = NULL;
static struct spidev_data *fp_spidev = NULL;
#ifndef SLPT
static struct spi_transfer t[SL_ONE_FRAME_PAGES];
#endif

static void sl_proc_cmd_read(char *cmd);
#ifndef SLPT
static void sl_proc_cmd_reg(char *cmd);
static void sl_proc_cmd_frame(char *cmd);
#endif
typedef enum {
	SL_READ_FRAME_STAT = 1,
	SL_WRTE_REG_STAT = 2,
	SL_READ_REG_STAT = 3,
	SL_UNKNOWN_STAT = 0xFFFFFFFF,
} current_seq_stat_t;
static current_seq_stat_t g_current_stat = SL_UNKNOWN_STAT;
static inline char *paser_sub_cmd(char *cmd)
{
	if (cmd == NULL)
		return cmd;
	if (!strlen(cmd))
		return cmd;
	while (cmd) {
		if (isalnum(*cmd) != 0) {
			break;
		}
		cmd++;
	}
	return cmd;
}

static inline char *trim_sub_cmd(char *cmd)
{
	if (cmd == NULL)
		return cmd;
	if (!strlen(cmd))
		return cmd;
	while (cmd) {
		if (isalnum(*cmd) == 0) {
			break;
		}
		cmd++;
	}
	return cmd;
}

static void *sl_seq_start(struct seq_file *f, loff_t *pos)
{
	if (g_current_stat == SL_UNKNOWN_STAT) {
		dev_info(&fp_spidev->spi->dev, "SL proc Unknown state");
		return NULL;
	}
	if (*pos == 0) {
		if (g_current_stat == SL_READ_REG_STAT) {
			return (void *)fp_spidev->mmap_buf;
		}
		return (void *)fp_spidev->mmap_buf + SL_HEAD_SIZE;
	}
	if (*pos < SL_ONE_FRAME_PAGES) {
		return (void *)fp_spidev->mmap_buf + SL_HEAD_SIZE +
			(*pos) * SL_PAGE_SIZE;
	}
	return NULL;
}

static void *sl_seq_next(struct seq_file *f, void *v, loff_t *pos)
{
	dev_dbg(&fp_spidev->spi->dev, "%s:f=%p, *v=%p, *pos=%lld:%d\n", __func__,
		f, v, *pos, SL_ONE_FRAME_PAGES);
	if (g_current_stat == SL_READ_REG_STAT) {
		dev_dbg(&fp_spidev->spi->dev, "%s:SL proc read reg state",
			__func__);
		return NULL;
	}

	if ((*pos) < SL_ONE_FRAME_PAGES) {
		(*pos) = (*pos) + 1;
		return (void *)((char *)v + SL_PAGE_SIZE);
	}
	return NULL;
}

static void sl_seq_stop(struct seq_file *f, void *v)
{
	SL_LOGD("%s:f=%p, *v=%p\n", __func__, f, v);
}

static int sl_seq_show(struct seq_file *m, void *v)
{
	int i = 0;
	char *data = (char *)v;

	SL_LOGD("%s:f=%p, *v=%p\n", __func__, m, v);
	if (g_current_stat == SL_READ_REG_STAT) {
		while (*data) {
			seq_printf(m, "%c", *data);
			data++;
		}
	}
	if (g_current_stat == SL_READ_FRAME_STAT) {
		for (i = 0; i < SL_PAGE_SIZE; i++) {
			seq_printf(m, "%02x", *data);
			data++;
		}
	}
	return 0;
}

const struct seq_operations sl_seq_ops = {
	.start = sl_seq_start,
	.next = sl_seq_next,
	.stop = sl_seq_stop,
	.show = sl_seq_show,
};

static int sl_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &sl_seq_ops);
}

static void sl_proc_cmd_start(char *cmd)
{
	SL_LOGD("%s\n", __func__);
	atomic_set(&fp_spidev->is_cal_mode, 1);
}

static void sl_proc_cmd_stop(char *cmd)
{
	atomic_set(&fp_spidev->is_cal_mode, 0);
	SL_LOGD("%s\n", __func__);
}

static void sl_proc_cmd_discal(char *cmd)
{
	SL_LOGD("%s\n", __func__);
	atomic_set(&fp_spidev->is_cal_mode, 0);
}

static void sl_proc_cmd_encal(char *cmd)
{
	SL_LOGD("%s\n", __func__);
	atomic_set(&fp_spidev->is_cal_mode, 1);
}

#ifndef SLPT
static void sl_proc_cmd_write(char *cmd)
{
	unsigned int addr;
	unsigned int val32;
	unsigned long res;
	char *val_cmd = NULL;

	if (cmd == NULL)
		return;
	if (!strlen(cmd))
		return;
	kstrtol(cmd, 16, &res);
	addr = (int)res;
	val_cmd = trim_sub_cmd(cmd);
	if (!val_cmd) {
		SL_LOGE("%s:trim sub cmd failed\n", __func__);
		return;
	}
	val_cmd = paser_sub_cmd(val_cmd);
	if (!val_cmd) {
		SL_LOGE("%s:paser sub cmd failed\n", __func__);
		return;
	}
	kstrtol(val_cmd, 16, &res);
	val32 = (int)res;

	if (addr & (~0xFF)) {
		spidev_write_reg(fp_spidev, (addr >> 7), 0xF0);
		spidev_write_reg(fp_spidev, val32, addr % 0x80);
	} else {
		spidev_write_reg(fp_spidev, val32, addr);
	}
}
#endif

static char *event[] = { "silead_fp", NULL };
static void sl_proc_cmd_uevent(char *cmd)
{
	if (fp_spidev) {
		kobject_uevent_env(&fp_spidev->spi->dev.kobj, KOBJ_CHANGE,
					 event);
	}
}

typedef struct sl_proc_cmd {
	char *cmd;
	void (*call)(char *);
} sl_proc_cmd_t;

struct sl_proc_cmd sl_cmd[] = {
	{"start", sl_proc_cmd_start},
	{"stop", sl_proc_cmd_stop},
	{"discal", sl_proc_cmd_discal},
	{"encal", sl_proc_cmd_encal},
#ifndef SLPT
	{"write", sl_proc_cmd_write},
#endif
	{"read", sl_proc_cmd_read},
	{"uevent", sl_proc_cmd_uevent},
};

struct sl_proc_cmd sl_sub_cmd[] = {
#ifndef SLPT
	{"reg", sl_proc_cmd_reg},
	{"frame", sl_proc_cmd_frame},
#endif
};

static void sl_proc_cmd_read(char *cmd)
{
	int i;

	for (i = 0; i < sizeof(sl_sub_cmd) / sizeof(struct sl_proc_cmd); i++) {
		if ((strncmp(sl_sub_cmd[i].cmd, cmd, strlen(sl_sub_cmd[i].cmd))
			 == 0)
			&& (sl_sub_cmd[i].call != NULL)) {
			sl_sub_cmd[i].call(paser_sub_cmd
						 (cmd + strlen(sl_sub_cmd[i].cmd)));
			break;
		}
	}
}

#ifndef SLPT
static void sl_proc_cmd_reg(char *cmd)
{
	unsigned int addr;
	unsigned long res;

	if (cmd == NULL)
		return;
	if (!strlen(cmd))
		return;

	kstrtol(cmd, 16, &res);
	addr = (int)res;
	/*addr = simple_strtol(cmd, NULL, 16);*/

	if (addr & (~0xFF)) {
		spidev_write_reg(fp_spidev, (addr >> 7), 0xF0);
		snprintf(fp_spidev->mmap_buf, 32, "0x%08x=0x%08x", addr,
				spidev_read_reg(fp_spidev, addr % 0x80));
	} else {
		snprintf(fp_spidev->mmap_buf, 32, "0x%08x=0x%08x", addr,
				spidev_read_reg(fp_spidev, addr));
	}
	g_current_stat = SL_READ_REG_STAT;
}

static void sl_proc_cmd_frame(char *cmd)
{
	struct spidev_data *spidev = fp_spidev;
	struct spi_message m;
	int i;

	t[0].rx_buf = spidev->mmap_buf;
	t[0].tx_buf = spidev->tx_mmap_buf;
	t[0].len = SL_HEAD_SIZE + SL_PAGE_SIZE;
	t[0].bits_per_word = SPI_BITS;
	t[0].delay_usecs = SPI_DELAY;
	t[0].speed_hz = SPI_SPEED;
	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	for (i = 1; i < SL_ONE_FRAME_PAGES; ++i) {
		t[i].rx_buf =
			spidev->mmap_buf + i * SL_PAGE_SIZE + SL_HEAD_SIZE;
		t[i].tx_buf =
			spidev->tx_mmap_buf + i * SL_PAGE_SIZE + SL_HEAD_SIZE;
		t[i].len = SL_PAGE_SIZE;
		t[i].bits_per_word = SPI_BITS;
		t[i].delay_usecs = SPI_DELAY;
		t[i].speed_hz = SPI_SPEED;
		spi_message_add_tail(&t[i], &m);
	}
	init_frame(spidev);
	spidev_sync(spidev, &m);
	g_current_stat = SL_READ_FRAME_STAT;
}
#endif

static ssize_t
sl_proc_write(struct file *fp, const char __user *buffer, size_t count,
			loff_t *f_pos)
{
	int i;
	char *cmd;

	cmd = kzalloc(count + 1, GFP_KERNEL);
	if (cmd == NULL) {
		return 0;
	}
	memset(cmd, 0, count);
	if (copy_from_user(cmd, buffer, count)) {
		SL_LOGE("copy from user fail\n");
		kfree(cmd);
		return -EIO;
	}

	for (i = 0; i < sizeof(sl_cmd) / sizeof(struct sl_proc_cmd); i++) {
		if ((strncmp(sl_cmd[i].cmd, cmd, strlen(sl_cmd[i].cmd)) == 0) &&
			(sl_cmd[i].call != NULL)) {
			sl_cmd[i].
				call(paser_sub_cmd(cmd + strlen(sl_cmd[i].cmd)));
			break;
		}
	}
	kfree(cmd);
	return count;
}

static const struct file_operations sl_proc_fops = {
	.open = sl_proc_open,
	.read = seq_read,
	.write = sl_proc_write,
	.llseek = seq_lseek,
	.release = seq_release,
};

int sl_proc_init(struct spidev_data *spidev)
{
#ifdef CONFIG_PROC_FS
	char errnum = -1;

	sl_proc = proc_create(SL_FP_PROC_FILE, 0444, NULL, &sl_proc_fops);
	SL_LOGD("[%s] proc file create pass !\n", __func__);
	if (sl_proc == NULL) {
		SL_LOGE("create_proc_entry %s failed\n", SL_FP_PROC_FILE);
		return errnum;
	}
	fp_spidev = spidev;
#endif
	return 0;
}
