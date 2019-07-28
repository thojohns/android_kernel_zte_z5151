/*********************************************************************************************
##	File Name:	SPI device driver for SILEAD INC.
##	Description:	This file aims to provide the implementation of low-level SPI transfer for fingerprint devices.
##	Author:	SILEAD INC.
##	Change History:
1.unify to apply DTS format for various platforms(MTK,QCOM,SPRD,etc.),2017/1/3 thomas
2.unify silead spi driver & platform driver,2017/5/26 thomas
**********************************************************************************************/
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
#include <linux/of_irq.h>
#include <linux/input.h>

/*#define sileadDBG*/
/*#define SPI_DRV_COMPATIBLE*/
/*add by thomas 20160427 for time consumption debug only begin*/
#ifdef sileadDBG
#include <linux/timex.h>
#include <linux/timer.h>
#endif
/*add by thomas 20160427 for time consumption debug only end*/
/*add by bacon for fp_wake_lock*/
#include <linux/kobject.h>
#include <linux/debugfs.h>
#include <../kernel/power/power.h>
/*add by bacon for fp_wake_lock*/
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/cdev.h>
#include <linux/cpumask.h>
#include <linux/kernel_stat.h>

#ifdef MTK50B64TZ
#include <mt_clkmgr.h>
#endif

#include "slspi.h"

#if !defined(SPRD51B64) && !defined(SPRD60B64TZWATCH)
#if !defined(MTK70B64TZ)
/*#include <mt_spi.h>*/
#else
#include <mtk_spi.h>
#endif
#endif

#include "sl_proc.h"

#define VERBOSE  0
#define SL_MAX_FRAME_NUM 2
#define N_SPI_MINORS			32

/*add by thomas for compatabling sprd driver 20161014 begin*/
#ifndef VM_RESERVED				/*for kernel up to 3.7.0 version */
#define  VM_RESERVED   (VM_DONTEXPAND | VM_DONTDUMP)
#endif
/*add by thomas for compatabling sprd driver 20161014 end*/
#define SILEAD_PROC

static DECLARE_BITMAP(minors, N_SPI_MINORS);

#ifdef SPI_DRV_COMPATIBLE
static void intToStr(unsigned int chipID, char *buf);
#endif

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DECLARE_WAIT_QUEUE_HEAD(sileadfp_pollerr_waitq);
static int g_poll_event_mask = 0;

static struct spidev_data	*fp_spidev = NULL;
static unsigned int spidev_major = 0;
static struct cdev spicdev;
/*static unsigned int irq_counter=1;*/
static unsigned int g_irq_counter = 1;

static int g_release_esd_reset_irq = 0;
static int g_irq_svc_debouce = 0;
static int chip_power_off = 0;

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*add by bacon for fp_wake_lock*/
static ssize_t fp_wake_lock_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n);
static ssize_t fp_wake_unlock_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n);
static ssize_t fp_wake_lock_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t fp_wake_unlock_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t fp_ext_timer_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n);
static ssize_t fp_ext_timer_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
/*add by bacon for fp_wake_lock*/
/*add by ben for script*/
extern int gsl_spi_gpio_set(void);
extern int gsl_spi_speed_set(struct GSL_DEV_SEL_device *spi);
extern int gsl_spi_open_clock_set(struct GSL_DEV_SEL_device *spi);
extern int gsl_spi_close_clock_set(struct GSL_DEV_SEL_device *spi);
extern int gsl_spidev_set_before_rd_chipid(struct GSL_DEV_SEL_device *spi);
extern int gsl_spidev_set_after_rd_chipid(struct GSL_DEV_SEL_device *spi);
extern long gsl_fp_pinctrl_init(struct spidev_data *spidev);
extern int spidev_reset_hw(struct spidev_data *spidev);
extern int spidev_shutdown_hw(struct spidev_data *spidev);
extern int silead_init_eint(struct spidev_data *spidev);
extern int silead_fp_parse_reset_and_int_gpios(struct spidev_data *spidev);
extern int silead_fp_init_gpio_states(struct spidev_data *spidev);
extern void silead_register_board_info(void);
/*add by ben for script*/
extern int sl_proc_init(struct spidev_data *spidev);

/*add detect silead fp */
extern int is_silead_fp(void);

/*add by bacon for fp_wake_lock*/
#define silead_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0666,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

silead_attr(fp_wake_lock);
silead_attr(fp_wake_unlock);
silead_attr(fp_ext_timer);

static struct attribute *g[] = {
	&fp_wake_lock_attr.attr,
	&fp_wake_unlock_attr.attr,
	&fp_ext_timer_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

struct wakelock {
	char			*name;
	struct wakeup_source	ws;
};
static struct wakelock *g_wakelock_list[10] = {0};

static DEFINE_MUTEX(wakelocks_lock);
static ssize_t fp_wake_lock_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	/*return pm_show_wakelocks(buf, true);*/

	int i;
	char *str = buf;
	char *end = buf + PAGE_SIZE;

	mutex_lock(&wakelocks_lock);
	for (i = 0; i < 10; i++) {
		if (g_wakelock_list[i] != NULL) {
			str += scnprintf(str, end - str, "%s ", g_wakelock_list[i]->name);
		}
	}
	if (str > buf)
		str--;

	str += scnprintf(str, end - str, "\n");

	mutex_unlock(&wakelocks_lock);
	return (str - buf);
}

static ssize_t fp_wake_lock_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
/*	int error = pm_wake_lock(buf);*/
/*	return error ? error : n;*/

	int i, j;
	int ret = -1;
	char *wl_name;
	struct wakelock *wl;

	wl_name = kstrndup(buf, n, GFP_KERNEL);
	if (!wl_name) {
		return -ENOMEM;
	}

	mutex_lock(&wakelocks_lock);
	for (j = 0; j < 10; j++) {
		if (g_wakelock_list[j] != NULL) {
			if (strcmp(g_wakelock_list[j]->name, buf) == 0) {
				wl = g_wakelock_list[j];
				ret = n;
				break;
			}
		}
	}

	if (j == 10) {
		wl = kzalloc(sizeof(*wl), GFP_KERNEL);
		if (!wl)
			return -ENOMEM;

		wl->name = wl_name;
		wl->ws.name = wl_name;
		wakeup_source_add(&wl->ws);

		for (i = 0; i < 10; i++) {
			if (g_wakelock_list[i] == NULL) {
				g_wakelock_list[i] = wl;
				ret = n;
				break;
			}
		}
	}

	__pm_stay_awake(&wl->ws);
	mutex_unlock(&wakelocks_lock);

	SL_LOGD("fp_wake_lock_store ret = %d\n", ret);
	return ret;
}

static ssize_t fp_wake_unlock_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	/*return pm_show_wakelocks(buf, fasle);*/
	int i;
	char *str = buf;
	char *end = buf + PAGE_SIZE;

	mutex_lock(&wakelocks_lock);
	for (i = 0; i < 10; i++) {
		if (g_wakelock_list[i] != NULL) {
			str += scnprintf(str, end - str, "%s ", g_wakelock_list[i]->name);
		}
	}
	if (str > buf)
		str--;

	str += scnprintf(str, end - str, "\n");

	mutex_unlock(&wakelocks_lock);
	return (str - buf);
}

static ssize_t fp_wake_unlock_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
/*	int error = pm_wake_unlock(buf);*/
/*	return error ? error : n;*/

	struct wakelock *wl;
	int ret = -1;
	int i;

	mutex_lock(&wakelocks_lock);
	for (i = 0; i < 10; i++) {
		if (g_wakelock_list[i] != NULL) {
			if (strcmp(g_wakelock_list[i]->name, buf) == 0) {
				wl = g_wakelock_list[i];
				__pm_relax(&wl->ws);
				wakeup_source_remove(&wl->ws);
				kfree(wl->name);
				kfree(wl);
				g_wakelock_list[i] = NULL;
				ret = n;
				break;
			}
		}
	}

	mutex_unlock(&wakelocks_lock);
	SL_LOGD("fp_wake_unlock_store ret = %d\n", ret);
	return ret;
}
/*add by bacon for fp_wake_unlock*/

/*add by matthew*/
static char g_ext_timer_stamp[64+1] = "";
static ssize_t fp_ext_timer_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int n = strlen(g_ext_timer_stamp);
	char *str = buf;

	scnprintf(str, PAGE_SIZE - 2, "%s", g_ext_timer_stamp);
	str[PAGE_SIZE - 2] = '\n';
	str[PAGE_SIZE - 1] = '\n';

	return n;
}

static ssize_t fp_ext_timer_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	struct spidev_data *spidev = fp_spidev;

	int ret = -1;
	int t_n = n;

	if (t_n > 64) {
		t_n = 64;
	}
	scnprintf(g_ext_timer_stamp, t_n, "%s", buf);
	g_ext_timer_stamp[t_n] = 0;

	if (strncmp("deadic_chktimer", buf, 15) == 0) {
		char *env_ext_forged[2] = {"SILEAD_FP_EVENT=IRQ_FORGED", NULL};

		kobject_uevent_env(&spidev->spi->dev.kobj, KOBJ_CHANGE, env_ext_forged);
		ret = n;
		/*for user space's poll/select,*/
		/*g_poll_event_mask |= POLLERR;*/
		/*wake_up_interruptible(&sileadfp_pollerr_waitq);*/
	}

	SL_LOGD("fp_ext_timer_store ret = %d\n", ret);
	return ret;
}

int slfp_ext_timer_fire(const char *cmdstr)
{
	return fp_ext_timer_store(NULL, NULL, cmdstr, strlen(cmdstr));
}
#define FP_VDD_MIN_UV	2850000
#define FP_VDD_MAX_UV	3300000
static int AVDD_FP_power_init(struct spidev_data *spidev, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(spidev->avdd) > 0)
			regulator_set_voltage(spidev->avdd, 0, FP_VDD_MAX_UV);
		regulator_put(spidev->avdd);
	} else {
		pr_info("%s: AVDD_FP_power_init %d ", __func__, __LINE__);

		spidev->avdd = regulator_get(&spidev->spi->dev, "avdd");
		if (IS_ERR(spidev->avdd)) {
			rc = PTR_ERR(spidev->avdd);
			dev_err(&spidev->spi->dev, "Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(spidev->avdd) > 0) {
			rc = regulator_set_voltage(spidev->avdd, 2850000,
						   FP_VDD_MAX_UV);
			if (rc) {
				dev_err(&spidev->spi->dev, "Regulator set failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}
	}
	pr_info("%s: AVDD_FP_power_init %d  end", __func__, __LINE__);

	return 0;
reg_vdd_put:
	regulator_put(spidev->avdd);
	return rc;
}
static int AVDD_FP_power_on(struct spidev_data *spidev, bool on)
{

	int rc = 0;

	if (!on)
		goto power_off;
	rc = regulator_enable(spidev->avdd);
	if (rc) {
			dev_err(&spidev->spi->dev, "Regulator vdd enable failed rc=%d\n", rc);
			goto fp_power_done;
		}
fp_power_done:
	return rc;
power_off:
	rc = regulator_disable(spidev->avdd);
	if (rc) {
			dev_err(&spidev->spi->dev, "Regulator vdd disable failed rc=%d\n", rc);
			goto fp_power_done;
		}
	goto fp_power_done;
}
/*add by matthew*/

#ifdef GSL_FP_POWER_CTRL
static int GSL_FP_power_init(struct spidev_data *spidev, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(spidev->vdd) > 0)
			regulator_set_voltage(spidev->vdd, 0, GSL_FP_VDD_MAX_UV);
		regulator_put(spidev->vdd);
		if (regulator_count_voltages(spidev->vio) > 0)
			regulator_set_voltage(spidev->vio, 0, GSL_FP_VIO_MAX_UV);
		regulator_put(spidev->vio);
	} else {
		spidev->vdd = regulator_get(&spidev->spi->dev, "vdd");
		if (IS_ERR(spidev->vdd)) {
			rc = PTR_ERR(spidev->vdd);
			dev_err(&spidev->spi->dev, "Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(spidev->vdd) > 0) {
			rc = regulator_set_voltage(spidev->vdd, GSL_FP_VDD_MIN_UV,
						   GSL_FP_VDD_MAX_UV);
			if (rc) {
				dev_err(&spidev->spi->dev, "Regulator set failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}
		spidev->vio = regulator_get(&spidev->spi->dev, "vio");
		if (IS_ERR(spidev->vio)) {
			rc = PTR_ERR(spidev->vio);
			dev_err(&spidev->spi->dev, "Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}
		if (regulator_count_voltages(spidev->vio) > 0) {
			rc = regulator_set_voltage(spidev->vio, GSL_FP_VIO_MIN_UV,
						   GSL_FP_VIO_MAX_UV);
			if (rc) {
				dev_err(&spidev->spi->dev, "Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}
	return 0;
reg_vio_put:
	regulator_put(spidev->vio);
reg_vdd_set:
	if (regulator_count_voltages(spidev->vdd) > 0)
		regulator_set_voltage(spidev->vdd, 0, GSL_FP_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(spidev->vdd);
	return rc;
}

static int GSL_FP_power_on(struct spidev_data *spidev, bool on)
{

	int rc = 0;

	if (!on)
		goto power_off;
	rc = regulator_enable(spidev->vdd);
	if (rc) {
			dev_err(&spidev->spi->dev, "Regulator vdd enable failed rc=%d\n", rc);
			goto fp_power_done;
		}

	rc = regulator_enable(spidev->vio);
	if (rc) {
			dev_err(&spidev->spi->dev, "Regulator vio enable failed rc=%d\n", rc);
			rc = regulator_disable(spidev->vdd);
			if (rc)
				dev_err(&spidev->spi->dev, "Regulator vdd disable failed rc=%d\n", rc);
			goto fp_power_done;
		}
fp_power_done:
	return rc;
power_off:
	rc = regulator_disable(spidev->vdd);
	if (rc) {
			dev_err(&spidev->spi->dev, "Regulator vdd disable failed rc=%d\n", rc);
			goto fp_power_done;
		}

	rc = regulator_disable(spidev->vio);
	if (rc) {
			dev_err(&spidev->spi->dev, "Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(spidev->vdd);
			if (rc)
				dev_err(&spidev->spi->dev, "Regulator vdd enable failed rc=%d\n", rc);
		}
	goto fp_power_done;
}
#endif

#ifndef SLPT
static int gsl_fp_rdinit(struct spidev_data *spidev, unsigned char reg)
{
	uint8_t tx[] = {
		reg, SL_READ,
	};
	unsigned rx[ARRAY_SIZE(tx)] = {0};
	struct spi_message	m;
	struct spi_transfer	t = {
		.rx_buf		= rx,
		.tx_buf		= tx,
		.len		= ARRAY_SIZE(tx),
		.bits_per_word = SPI_BITS,
		.delay_usecs = SPI_DELAY,
		.speed_hz = SPI_SPEED,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}
#endif

#ifdef LSB_TO_MSB
static const unsigned char reverse_table256[] = {
	0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
	0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
	0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
	0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
	0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
	0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
	0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
	0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
	0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
	0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
	0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
	0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
	0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
	0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
	0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
	0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

static void Modifybuf(u8 *buf, size_t len)
{
	int i;

	for (i = 0; i < len; i++) {
		buf[i] = reverse_table256[buf[i]];
	}
}
#endif

#ifndef SLPT
static void spidev_complete(void *arg)
{
	complete(arg);
}
#endif

static void spidev_irq_enable_control(struct spidev_data *in_spi_dev, int enableStatus)
{
	if (in_spi_dev == NULL) {
		SL_LOGE("input struct spidev_data is null!\n");
		return;
	}

	if (g_irq_counter == 1) {
		if (enableStatus == 0) {
			disable_irq_nosync(in_spi_dev->irq);
			g_irq_counter--;
		} else {
			return;
		}
	} else if (g_irq_counter == 0) {
		if (enableStatus == 1) {
			enable_irq(in_spi_dev->irq);
			g_irq_counter++;
		} else {
			return;
		}
	} else	{

	}
}

static void spidev_irq_work(struct work_struct *work)
{
	struct spidev_data *spidev = container_of(work, struct spidev_data, irq_work);
	char *env_ext[2] = {"SILEAD_FP_EVENT=IRQ", NULL};
/*#ifndef REDUCE_REPEAT_IRQ*/
	char *env_ext_forged[2] = {"SILEAD_FP_EVENT=IRQ_FORGED", NULL};
/*#endif*/
	SL_LOGD("irq bottom half spidev_irq_work enter\n");
/*#ifndef REDUCE_REPEAT_IRQ*/
	if (g_irq_svc_debouce) {
		/*for huaqi ali case, open this.*/
		kobject_uevent_env(&spidev->spi->dev.kobj, KOBJ_CHANGE, env_ext_forged);
		return;
	}
	g_irq_svc_debouce = 1;
/*	#endif*/

	kobject_uevent_env(&spidev->spi->dev.kobj, KOBJ_CHANGE, env_ext);
}

static irqreturn_t spidev_irq_routing(int irq, void *dev)
{
	struct spidev_data *spidev = dev;
	SL_LOGI("irq_routing entry");
	if (g_release_esd_reset_irq == 1) {
		SL_LOGE("irq top half release esd reset irq\n");
		return IRQ_HANDLED;
	}

	if (chip_power_off) {
		return IRQ_HANDLED;
	}
#ifdef REDUCE_REPEAT_IRQ
	spidev_shutdown_hw(spidev);
#endif
#ifdef sileadDBG
	struct timex txc;

	do_gettimeofday(&(txc.time));
	SL_LOGD("txc.time.tv_sec=%ld,txc.time.tv_usec=%ld\n", txc.time.tv_sec, txc.time.tv_usec);
#endif
	SL_LOGD("irq top half spidev_irq_routing enter\n");
	wake_lock_timeout(&spidev->wake_lock, 1*HZ);
/*	disable_irq_nosync(spidev->irq);*/
/*	irq_counter--;	*/
	spidev_irq_enable_control(spidev, 0);

	if (spidev->wqueue) {
#ifdef sileadDBG
		SL_LOGD("now spidev->wqueue is not NULL!\n");
#endif
		queue_work(spidev->wqueue, &spidev->irq_work);
	} else {
#ifdef sileadDBG
		SL_LOGD("now spidev->wqueue is NULL!\n");
#endif
		schedule_work(&spidev->irq_work);
	}
	return IRQ_HANDLED;
}

#ifndef SLPT
ssize_t spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

#ifdef LSB_TO_MSB
	struct list_head *p;
	struct spi_transfer *t;

	list_for_each(p, &message->transfers) {
		/*pr_err("%s:%d\n", __func__, __LINE__);*/
		t = list_entry(p, struct spi_transfer, transfer_list);
		if (t->tx_buf) {
			Modifybuf((u8 *)t->tx_buf, t->len);
		}
	}
#endif

	message->complete = spidev_complete;
	message->context = &done;

	if (spidev->spi == NULL) {
		status = -ESHUTDOWN;
	} else {
		spin_lock_irq(&spidev->spi_lock);
		status = spi_async(spidev->spi, message);
		spin_unlock_irq(&spidev->spi_lock);
	}

	if (status == 0) {
		/*wait_for_completion(&done);------> deadlock*/
		/*unsigned long ret = */
		wait_for_completion_timeout(&done, msecs_to_jiffies(3000));
		status = message->status;
		if (status == 0)
			status = message->actual_length;

#ifdef LSB_TO_MSB
		list_for_each(p, &message->transfers) {
			/*pr_err("%s:%d\n", __func__, __LINE__);*/
			t = list_entry(p, struct spi_transfer, transfer_list);
			if (t->rx_buf) {
				Modifybuf((u8 *)t->rx_buf, t->len);
			}
		}
#endif
	}
	return status;
}
#endif


static unsigned int spidev_poll(struct file *file, struct poll_table_struct *wait)
{

	unsigned int mask = g_poll_event_mask;

	g_poll_event_mask = 0;
	poll_wait(file, &sileadfp_pollerr_waitq, wait);
	return mask;
}

#ifndef SLPT
static union {
	unsigned char temp_char[4];
	unsigned int get_reg_data;
} temp_data;

unsigned int spidev_read_reg(struct spidev_data *spidev, unsigned char reg)
{
	struct spi_message	m;
	unsigned char rx[6];
	unsigned char tx[] = {
		reg, SL_READ, 0x00, 0x00, 0x00, 0x00
	};
	struct spi_transfer	t = {
		.rx_buf		= rx,
		.tx_buf		= tx,
		.len		= ARRAY_SIZE(tx),
		.bits_per_word = SPI_BITS,
		.delay_usecs = SPI_DELAY,
		.speed_hz = SPI_SPEED,
	};
	if (!(reg > 0x80 && reg < 0x100)) {
		gsl_fp_rdinit(spidev, reg);
	}
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	spidev_sync(spidev, &m);
	memcpy(temp_data.temp_char, (rx+2), 4);
	return temp_data.get_reg_data;
}

int spidev_write_reg(struct spidev_data *spidev, unsigned int data, unsigned char reg)
{
	struct spi_message	m;
	uint8_t rx[6] = {0};
	uint8_t tx[] = {
		reg, SL_WRITE,
		(data >> 0) & 0xFF,
		(data >> 8) & 0xFF,
		(data >> 16) & 0xFF,
		(data >> 24) & 0xFF,
	};
	struct spi_transfer	t = {
		.rx_buf		= rx,
		.tx_buf		= tx,
		.len		= ARRAY_SIZE(tx),
		.bits_per_word = SPI_BITS,
		.delay_usecs = SPI_DELAY,
		.speed_hz = SPI_SPEED,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	struct spi_message	m;
	struct spi_transfer t;
	ssize_t	status = 0;

	spidev = filp->private_data;
	t.rx_buf = spidev->buffer;
	t.len = count;
	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;
	mutex_lock(&spidev->buf_lock);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spidev_sync(spidev, &m);

	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, spidev->buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spidev->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t status = 0;
	unsigned long missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	missing = copy_from_user(spidev->buffer, buf, count);
	if (missing == 0) {
	struct spi_transfer t = {
		.tx_buf			= spidev->buffer,
		.len			= count,
	};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spidev_sync(spidev, &m);

	} else
		status = -EFAULT;
	mutex_unlock(&spidev->buf_lock);

	return status;
}

static int spidev_message(struct spidev_data *spidev, struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned n, total;
	u8 *buf;
	int status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	buf = spidev->buffer;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers; n; n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;
		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}
		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
							(uintptr_t) u_tmp->rx_buf,
							u_tmp->len))
			goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)(uintptr_t)u_tmp->tx_buf, u_tmp->len))
			goto done;
		}
		buf += k_tmp->len;
		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#if 0
		SL_LOGD("  xfer len %d %s%s%s%dbits %u usec %uHz\n",
				u_tmp->len,
				u_tmp->rx_buf ? "rx " : "",
				u_tmp->tx_buf ? "tx " : "",
				u_tmp->cs_change ? "cs " : "",
				u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
				u_tmp->delay_usecs,
				u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}
	status = spidev_sync(spidev, &msg);
	if (status < 0) {
		dev_err(&spidev->spi->dev, "spidev sync failed %d\n", status);
		goto done;
	}
/* copy any rx data out of bounce buffer */
	buf = spidev->buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)(uintptr_t) u_tmp->rx_buf, buf, u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;
done:
	kfree(k_xfers);
	return status;
}

void init_frame(struct spidev_data *spidev)
{
	unsigned int ret = 0;
	unsigned long timeout;
/*init page point*/
	spidev_write_reg(spidev, 0x00, 0xBF);
/* start scanning*/
	spidev_write_reg(spidev, (0xFF080024>>7), 0xF0);
	spidev_write_reg(spidev, 0x2007FFFF, (0xFF080024%0x80));
/* Wait  2 seconds for scanning done */
	timeout = jiffies + 2*HZ;
	while (time_before(jiffies, timeout)) {
		ret = spidev_read_reg(spidev, 0xBF);
		dev_dbg(&spidev->spi->dev, "0xBF=0x%02x\n", ret);
		if (ret != 0) {
			break;
		}
		udelay(100);
	}
	dev_dbg(&spidev->spi->dev, "last ret 0xBF=0x%02x\n", ret);
	spidev_write_reg(spidev, 0x00, 0xF0);
	gsl_fp_rdinit(spidev, 0);
}
#endif

static int spidev_mmap(struct file *filep, struct vm_area_struct *vma)
{
	struct spidev_data	*spidev = filep->private_data;

	vma->vm_flags |= VM_RESERVED;
	vma->vm_flags |= VM_LOCKED;
	if (spidev->mmap_buf == NULL) {
		dev_err(&spidev->spi->dev, "frame buffer is not alloc\n");
		return -ENOMEM;
	}
	return remap_pfn_range(vma, vma->vm_start,
							virt_to_phys((void *)((unsigned long)spidev->mmap_buf))
			  >>PAGE_SHIFT, vma->vm_end - vma->vm_start, PAGE_SHARED);
}

static long spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	struct spidev_data *spidev;
	struct GSL_DEV_SEL_device *spi;
#ifndef SLPT
	u32 tmp;
	unsigned n_ioc;
	struct spi_ioc_transfer	*ioc;
#endif
	unsigned int int_count_ptr[nr_cpu_ids];
	int j;

	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
#ifndef SLPT
	spi = spi_dev_get(spidev->spi);
#else
	spi = (spidev->spi && get_device(&spidev->spi->dev)) ? spidev->spi : NULL;
#endif
	spin_unlock_irq(&spidev->spi_lock);
	if (atomic_read(&spidev->is_cal_mode)) {
		dev_dbg(&spidev->spi->dev, "Current stat is cal mode\n");
		return -EBUSY;
	}
	if (atomic_read(&spidev->is_suspend)) {
		dev_dbg(&spidev->spi->dev, "device is suspend\n");
		return -EBUSY;
	}
	if (spi == NULL)
		return -ESHUTDOWN;
	mutex_lock(&spidev->buf_lock);
	switch (cmd) {
	case SPI_HW_RESET:
		chip_power_off = 1;
		spidev_reset_hw(spidev);
		msleep(1);
		chip_power_off = 0;
		break;
	case SPI_HW_SHUTDOWN:
		chip_power_off = 1;
		spidev_shutdown_hw(spidev);
		/*if (!irq_counter)  { */
		/*	enable_irq(spidev->irq);*/
		/*	irq_counter++;*/
		/* }*/
		SL_LOGD("matthew-SPI_HW_SHUTDOWN\n");
		spidev_irq_enable_control(spidev, 1);
		break;
	case SPI_HW_POWEROFF:
		SL_LOGD("SPI_HW_POWEROFF is OK\n");
		break;
	case SPI_HW_POWERON:
		SL_LOGD("SPI_HW_POWERON is OK\n");
		break;
	case SPI_OPEN_CLOCK:
		gsl_spi_open_clock_set(spidev->spi);
		break;
	case SPI_CLOSE_CLOCK:
		gsl_spi_close_clock_set(spidev->spi);
		break;
#ifndef SLPT
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}
			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0) {
				spi->mode = save;
			} else {
					dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
				}
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			gsl_spi_speed_set(spi);
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else {
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
			}
		}
		break;
#endif
	case SPI_HW_IRQ_ENBALE:
/*		if(arg) */
/*		{	*/
/*			SL_LOGD("int mode, irq_counter is %d\n",irq_counter); */
/*			if (!irq_counter)  */
/*			{*/
/*				enable_irq(spidev->irq);*/
/*				  irq_counter++;*/
/*			}*/
/*#ifndef REDUCE_REPEAT_IRQ*/
/*			g_irq_svc_debouce = 1;*/
/*			mdelay(5);*/
/*			g_irq_svc_debouce = 0;*/
/*#endif*/
/*		}*/
/*		else*/
/*		{*/
/*			SL_LOGD("polling mode");*/
/*		}*/
		if (arg == 0) {
			spidev_irq_enable_control(spidev, 0);
		} else if (arg == 1) {
			spidev_irq_enable_control(spidev, 1);
			g_irq_svc_debouce = 1;
			mdelay(5);
			g_irq_svc_debouce = 0;
		} else if (arg == 2) {
			g_release_esd_reset_irq = 1;
			spidev_irq_enable_control(spidev, 1);
			mdelay(5);
			spidev_irq_enable_control(spidev, 0);
			g_release_esd_reset_irq = 0;
		} else {
			SL_LOGE("SPI_HW_IRQ_ENBALE error arg\n");
		}
		break;
	case SPI_HW_IRQ_REGISTER:
		retval = 0;
		break;
	case SPI_HW_FINGER_STATE_INFO:
		if (arg) {
			SL_LOGD("finger on");
		} else {
			SL_LOGD("finger off");
		}
		retval = 0;
		break;
	case SPI_HW_VIRTUAL_KEY_INFO:
		if (arg) {
			switch (arg) {
			case 1:
					SL_LOGD("single click.\n");
#ifdef SL_KERNEL_KEYTYPE
					input_report_key(spidev->input,SL_INPUT_SCLICK_KEY,1);
					input_sync(spidev->input);
					input_report_key(spidev->input,SL_INPUT_SCLICK_KEY,0);
					input_sync(spidev->input);
#endif
					break;
			case 2:
					SL_LOGD("long press.\n");
#ifdef SL_KERNEL_KEYTYPE
					input_report_key(spidev->input,SL_INPUT_LPRESS_KEY,1);
					input_sync(spidev->input);
					input_report_key(spidev->input,SL_INPUT_LPRESS_KEY,0);
					input_sync(spidev->input);

#endif
					break;
			case 3:
					SL_LOGD("double click.\n");
#ifdef SL_KERNEL_KEYTYPE
					input_report_key(spidev->input,SL_INPUT_DCLICK_KEY,1);
					input_sync(spidev->input);
					input_report_key(spidev->input,SL_INPUT_DCLICK_KEY,0);
					input_sync(spidev->input);
#endif
					break;
			case 4:
					SL_LOGD("Slide up.\n");
#ifdef SL_KERNEL_KEYTYPE
					input_report_key(spidev->input, SL_INPUT_UP_KEY, 1);
					input_sync(spidev->input);
					input_report_key(spidev->input, SL_INPUT_UP_KEY, 0);
					input_sync(spidev->input);
#endif
					break;
			case 5:
					SL_LOGD("slide down.\n");
#ifdef SL_KERNEL_KEYTYPE
					input_report_key(spidev->input, SL_INPUT_DOWN_KEY, 1);
					input_sync(spidev->input);
					input_report_key(spidev->input, SL_INPUT_DOWN_KEY, 0);
					input_sync(spidev->input);
#endif
					break;
			case 6:
					SL_LOGD("slide left.\n");
#ifdef SL_KERNEL_KEYTYPE
#ifdef CONFIG_FINGERPRINT_SILEAD_NAVI
					SL_LOGD("slide kitty up.\n");
					input_report_key(spidev->input, SL_INPUT_UP_KEY, 1);
					input_sync(spidev->input);
					input_report_key(spidev->input, SL_INPUT_UP_KEY, 0);
					input_sync(spidev->input);
#else
					input_report_key(spidev->input, SL_INPUT_LEFT_KEY, 1);
					input_sync(spidev->input);
					input_report_key(spidev->input, SL_INPUT_LEFT_KEY, 0);
					input_sync(spidev->input);
#endif
#endif
					break;
			case 7:
					SL_LOGD("slide right.\n");
#ifdef SL_KERNEL_KEYTYPE
#ifdef CONFIG_FINGERPRINT_SILEAD_NAVI
					SL_LOGD("slide kitty down.\n");
					input_report_key(spidev->input, SL_INPUT_DOWN_KEY, 1);
					input_sync(spidev->input);
					input_report_key(spidev->input, SL_INPUT_DOWN_KEY, 0);
					input_sync(spidev->input);
#else
					input_report_key(spidev->input, SL_INPUT_RIGHT_KEY, 1);
					input_sync(spidev->input);
					input_report_key(spidev->input, SL_INPUT_RIGHT_KEY, 0);
					input_sync(spidev->input);
#endif
#endif
					break;
			case 8:
					SL_LOGD("IRQ WAKEUP");
					break;
			case 9:
					SL_LOGD("verify success,wake up");
					break;
			case 10:
					SL_LOGD("wakeup canceled");
					break;
			}
		} else {
			SL_LOGD("no vkey result.\n");
		}
		break;
/*add by thomas for int begin*/
	case SPI_GET_KERNEL_INT_INFO:
		SL_LOGD("nr_cpu_ids = %d", nr_cpu_ids);
		for (j = 0; j < nr_cpu_ids; j++)
			int_count_ptr[j] = kstat_irqs_cpu(spidev->irq, j);
		if (copy_to_user((unsigned int __user *)arg, int_count_ptr, nr_cpu_ids)) {
			SL_LOGE("copy_to_user int_count_ptr failed");
			return -EFAULT;
		}
		break;
/*add by thomas for int end*/
	default:
#ifndef SLPT
/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}
		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;
/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}
/* translate to spi_message, execute */
		retval = spidev_message(spidev, ioc, n_ioc);
		kfree(ioc);
#endif
		break;
	}
	mutex_unlock(&spidev->buf_lock);
	GSL_DEV_SEL_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int	status = -ENXIO;

	if (atomic_read(&fp_spidev->is_cal_mode)) {
		dev_dbg(&fp_spidev->spi->dev, "Current stat is cal mode\n");
		return -EACCES;
	}
	mutex_lock(&device_list_lock);
	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (!spidev->buffer) {
			spidev->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spidev->buffer) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			spidev->users++;
			filp->private_data = spidev;
			nonseekable_open(inode, filp);
		}
	} else {
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int status = 0;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;
	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int dofree;

		kfree(spidev->buffer);
		spidev->buffer = NULL;
/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);
		if (dofree) {
			kfree(spidev);
			mutex_unlock(&device_list_lock);
			return status;
		}
	}
	cancel_work_sync(&spidev->work);
	mutex_unlock(&device_list_lock);
	return status;
}

static const struct file_operations spidev_fops = {
	.owner = THIS_MODULE,
#ifndef SLPT
	.write = spidev_write,
	.read =	spidev_read,
#endif
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open = spidev_open,
	.release = spidev_release,
	.llseek = no_llseek,
	.mmap = spidev_mmap,
	.poll = spidev_poll,
};

static struct class *spidev_class;

#ifdef SPI_DRV_COMPATIBLE
static void intToStr(unsigned int chipID, char *buf)
{
	int i;

	for (i = 0; i < 8; i++) {
		if (((chipID >> 4*(7 - i)) & 0xf) >= 0 && ((chipID >> 4*(7 - i)) & 0xf) <= 9) {
			buf[i] = ((chipID >> 4*(7 - i)) & 0xf) + '0';
		} else {
			buf[i] = (((chipID >> 4*(7 - i)) & 0xf) - 10) + 'a';
		}
	}
	buf[8] = '\0';
}
#endif

static int silead_fp_request_gpios(struct spidev_data *spidev)
{
	int status = 0;

	if (gpio_is_valid(spidev->hw_reset_gpio)) {
		status = devm_gpio_request(&spidev->spi->dev, spidev->hw_reset_gpio, "silead_reset_gpio");
		if (status) {
			SL_LOGE("devm_gpio_request reset_gpio failed! status = %d", status);
			return -EFAULT;
		}
	SL_LOGD("devm_gpio_request reset_gpio success!");
	}
	if (gpio_is_valid(spidev->hw_int_gpio)) {
		status = devm_gpio_request_one(&spidev->spi->dev, spidev->hw_int_gpio,
						(GPIOF_DIR_IN | GPIOF_INIT_LOW), "silead_int_gpio");
		if (status) {
			SL_LOGE("devm_gpio_request_one int_gpio failed! status = %d", status);
			return -EFAULT;
		}
		SL_LOGD("devm_gpio_request int_gpio success!");
	}
	return status;
}

static int silead_fp_free_gpios(struct spidev_data *spidev)
{
	int status = 0;

	if (gpio_is_valid(spidev->hw_reset_gpio)) {
		devm_gpio_free(&spidev->spi->dev, spidev->hw_reset_gpio);
		SL_LOGD("devm_gpio_free reset_gpio success!");
	}
	if (gpio_is_valid(spidev->hw_int_gpio)) {
		devm_gpio_free(&spidev->spi->dev, spidev->hw_int_gpio);
		SL_LOGD("devm_gpio_free int_gpio success!");
	}
	return status;
}

#ifdef SL_KERNEL_KEYTYPE
static int sl_reg_key_kernel(struct spidev_data *spidev)
{
	spidev->input = input_allocate_device();
	if (IS_ERR(spidev->input)) {
		SL_LOGE("Failed to allocate input device.");
		return PTR_ERR(spidev->input);
	}
	__set_bit(EV_KEY, spidev->input->evbit);
	__set_bit(SL_INPUT_SCLICK_KEY, spidev->input->keybit);
	__set_bit(SL_INPUT_LPRESS_KEY, spidev->input->keybit);
	__set_bit(SL_INPUT_DCLICK_KEY, spidev->input->keybit);
	__set_bit(SL_INPUT_UP_KEY, spidev->input->keybit);
	__set_bit(SL_INPUT_DOWN_KEY, spidev->input->keybit);
	__set_bit(SL_INPUT_LEFT_KEY, spidev->input->keybit);
	__set_bit(SL_INPUT_RIGHT_KEY, spidev->input->keybit);
	spidev->input->name = "silead-key";
	if (input_register_device(spidev->input)) {
		SL_LOGE("Failed to register input device.");
	}
	return 0;
}
#endif

static int	spidev_probe(struct GSL_DEV_SEL_device *spi)
{
	struct spidev_data	*spidev;
	int	status;
	long ret;
	unsigned irq_flags;
#ifndef SLPT
#ifdef SPI_DRV_COMPATIBLE
	unsigned int val;
	char tmp[9];
#endif
#endif
	unsigned long minor, page;

	SL_LOGI("has entered!");
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev) {
		SL_LOGE("no enough mem for spidev!");
		return -ENOMEM;
	}
	spidev->spi = spi;
	status = gsl_spidev_set_before_rd_chipid(spi);
	if (status < 0)
		dev_err(&spi->dev, "gsl_spidev_set_before_rd_chipid failed! status=%d\n", status);

	AVDD_FP_power_init(spidev, true);
	AVDD_FP_power_on(spidev, true);

#ifdef GSL_FP_POWER_CTRL
	status = GSL_FP_power_init(spidev, true);
	if (status < 0) {
		dev_err(&spi->dev, "power init failed! err=%d", status);
		kfree(spidev);
		return status;
	}
	status = GSL_FP_power_on(spidev, true);
	if (status < 0) {
		dev_err(&spi->dev, "power on failed! err=%d\n", status);
		return status;
	}
#endif
	ret = gsl_fp_pinctrl_init(spidev);
	if (ret) {
		SL_LOGE("gsl_fp_pinctrl_init failed!");
		kfree(spidev);
		return -EFAULT;
	}
	silead_fp_parse_reset_and_int_gpios(spidev);
	status = silead_fp_request_gpios(spidev);
	if (status) {
		SL_LOGE("silead_fp_request_gpios failed!");
		kfree(spidev);
		return -EFAULT;
	}
	status = silead_fp_init_gpio_states(spidev);
	if (status) {
		SL_LOGE("silead_fp_init_gpio_states failed!");
		kfree(spidev);
		return -EFAULT;
	}
#ifndef SLPT
#ifdef SPI_DRV_COMPATIBLE
	val = spidev_read_reg(spidev, 0xfc);
	intToStr(val, tmp);
	tmp[8] = '\0';
	SL_LOGD("[%s] the chip id = %s\n", __func__, tmp);
	if (strncmp(tmp, "61", 2)) {
		SL_LOGE("the chip is not for silead\n");
		silead_fp_free_gpios(spidev);
		kfree(spidev);
		spidev = NULL;
		return -EINVAL;
	}
#endif
#endif
	status = gsl_spidev_set_after_rd_chipid(spi);
	if (status < 0)
		dev_err(&spi->dev, "gsl_spidev_set_after_rd_chipid failed! status=%d\n", status);
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);
	INIT_LIST_HEAD(&spidev->device_entry);
	INIT_WORK(&spidev->irq_work, spidev_irq_work);
	wake_lock_init(&spidev->wake_lock, WAKE_LOCK_SUSPEND, "silead_wake_lock");
	spidev->wqueue = create_singlethread_workqueue("silead_wq");
	spidev->max_frame_num = SL_MAX_FRAME_NUM;
	spidev->max_buf_size = ((sizeof(struct sl_frame)*spidev->max_frame_num+PAGE_SIZE)/PAGE_SIZE)*PAGE_SIZE;
	spidev->mmap_buf = kmalloc(spidev->max_buf_size, GFP_KERNEL);
	if (!spidev->mmap_buf) {
		dev_err(&spi->dev, "alloc mmap_buf failed\n");
		return -ENOMEM;
	}
	spidev->tx_mmap_buf = kmalloc(spidev->max_buf_size, GFP_KERNEL);
	if (!spidev->tx_mmap_buf) {
		/*dev_err(&spi->dev, "alloc tx_mmap_buf failed\n");*/
		kfree(spidev->mmap_buf);
		return -ENOMEM;
	}
	memset(spidev->mmap_buf, 0, spidev->max_buf_size);
	memset(spidev->tx_mmap_buf, 0, spidev->max_buf_size);
	for (page = (unsigned long)spidev->mmap_buf;
			page < (unsigned long)spidev->mmap_buf+spidev->max_buf_size; page += PAGE_SIZE) {
			SetPageReserved(virt_to_page(page));
	}
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	SL_LOGD("minor = %ld\n", minor);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(spidev_major, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
				spidev, "silead_fp_dev");
				status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	SL_LOGD("status = %d\n", status);
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
#ifdef SL_KERNEL_KEYTYPE
	sl_reg_key_kernel(spidev);
#endif
	if (status == 0) {
		GSL_DEV_SEL_set_drvdata(spi, spidev);
		fp_spidev = spidev;
		silead_init_eint(spidev);
		irq_flags = IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND;
		ret = request_irq(spidev->irq, spidev_irq_routing, irq_flags, SL_INT_NAME, spidev);
		if (ret < 0) {
			kfree(spidev->mmap_buf);
			kfree(spidev->tx_mmap_buf);
			SL_LOGD("request_irq ret= %ld\n", ret);
			return -EFAULT;
		}
		fp_spidev->irq = spidev->irq;
#ifdef SILEAD_PROC
		sl_proc_init(spidev);
#endif
		atomic_set(&spidev->is_cal_mode, 0);/*default is enroll mode*/
		atomic_set(&spidev->is_suspend, 0);/*default is enroll mode*/
/*disable_irq_nosync(spidev->irq);*/
/*irq_counter--;*/
/*mdelay(1);*/
/*enable_irq(spidev->irq);*/
/*irq_counter++;*/
		spidev_irq_enable_control(spidev, 0);
		msleep(1);
		spidev_irq_enable_control(spidev, 1);
/*reset device, begin*/
		gpio_direction_output(spidev->hw_reset_gpio, 0);
/*reset device, end*/

	} else {
		kfree(spidev);
	}
	return status;
}

static int	spidev_remove(struct GSL_DEV_SEL_device *spi)
{
	struct spidev_data	*spidev = GSL_DEV_SEL_get_drvdata(spi);
	unsigned long page;
#ifdef GSL_FP_POWER_CTRL
	int status = 0;
#endif
	dev_err(&spi->dev, "silead %s has entered!\n", __func__);
	SL_LOGD("silead %s has entered!\n", __func__);
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	GSL_DEV_SEL_set_drvdata(spi, NULL);
	spin_unlock_irq(&spidev->spi_lock);
#ifdef GSL_FP_POWER_CTRL
	status = GSL_FP_power_on(spidev, false);
	if (status < 0)
		dev_err(&spi->dev, "power off failed! err=%d\n", status);
	status = GSL_FP_power_init(spidev, false);
	if (status < 0)
		dev_err(&spi->dev, "power deinit failed! err=%d", status);
#endif
	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0) {
		if (spidev->mmap_buf) {
			for (page = (unsigned long)spidev->mmap_buf;
				page < (unsigned long)spidev->mmap_buf+spidev->max_buf_size; page += PAGE_SIZE) {
				ClearPageReserved(virt_to_page(page));
			}
			kfree(spidev->mmap_buf);
		}
		kfree(spidev->tx_mmap_buf);
		wake_lock_destroy(&spidev->wake_lock);
		kfree(spidev);
	}
	mutex_unlock(&device_list_lock);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int spidev_suspend(struct device *spi)
{
	/*if platform is GSL8937 ,please open enable_irq_wake*/
	struct spidev_data *spidev = dev_get_drvdata(spi);

	enable_irq_wake(spidev->irq);
	SL_LOGD(" entering ");
	return 0;
}

static int spidev_resume(struct device *spi)
{
	/*if platform is GSL8937 ,please open enable_irq_wake*/
	struct spidev_data  *spidev = dev_get_drvdata(spi);

	disable_irq_wake(spidev->irq);
	SL_LOGD(" entering ");
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(silead_fp_pm_ops, spidev_suspend, spidev_resume);

static const struct of_device_id silead_of_match[] = {
	{.compatible = "silead,silead_fp",},
};
MODULE_DEVICE_TABLE(of, silead_of_match);

static struct GSL_DEV_SEL_driver silead_fp_driver = {
	.driver = {
		.name =	"silead_fp",
		.owner = THIS_MODULE,
	.pm = &silead_fp_pm_ops,
	.of_match_table = silead_of_match,
	},

	.probe = spidev_probe,
	.remove = spidev_remove,
};

static int __init spidev_init(void)
{
	int status, error;
	dev_t devno;
	struct kobject *power_kobj;

	SL_LOGI("has entered!");

#ifndef CONFIG_FINGERPRINT_SILEAD_NO_FINGERID
	if (is_silead_fp()) {
		SL_LOGI("<HW> Silead fingerprint hw detected\n");
	} else {
		SL_LOGE("<HW> not Silead fingerprint hw, skip\n");
		return -EFAULT;
	}
#endif

	silead_register_board_info();
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = alloc_chrdev_region(&devno, 0, 255, "sileadfp");
	if (status < 0)
		return status;
	spidev_major = MAJOR(devno);
	cdev_init(&spicdev, &spidev_fops);
	spicdev.owner = THIS_MODULE;
	status = cdev_add(&spicdev, MKDEV(spidev_major, 0), N_SPI_MINORS);
	if (status) {
		unregister_chrdev_region(devno, 255);
		return status;
	}
	spidev_class = class_create(THIS_MODULE, "spidev1");
	if (IS_ERR(spidev_class)) {
		unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
		return PTR_ERR(spidev_class);
	}
	SL_LOGD("GSL_DEV_SEL_register_driver start!");
	status = GSL_DEV_SEL_register_driver(&silead_fp_driver);
	if (status < 0) {
		SL_LOGE("GSL_DEV_SEL_register_driver failed!");
		class_destroy(spidev_class);
		unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
		return status;
	}
/*add by bacon for fp_wake_lock*/
	power_kobj = kobject_create_and_add("silead", NULL);
	if (!power_kobj) {
		class_destroy(spidev_class);
		unregister_chrdev_region(devno, 255);
		unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
		return -ENOMEM;
	}
	error = sysfs_create_group(power_kobj, &attr_group);
	if (error) {
		class_destroy(spidev_class);
		unregister_chrdev_region(devno, 255);
		unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
		return error;
	}
/*add by bacon for fp_wake_lock*/
	return status;
}

static void __exit spidev_exit(void)
{
	silead_fp_free_gpios(fp_spidev);
	free_irq(fp_spidev->irq, fp_spidev);
	device_destroy(spidev_class, fp_spidev->devt);
	class_destroy(spidev_class);
	kfree(fp_spidev);
	fp_spidev = NULL;
	cdev_del(&spicdev);
	GSL_DEV_SEL_unregister_driver(&silead_fp_driver);
	unregister_chrdev(spidev_major, silead_fp_driver.driver.name);
}

module_init(spidev_init);
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
