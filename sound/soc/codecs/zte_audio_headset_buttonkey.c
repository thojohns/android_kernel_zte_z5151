#ifdef CONFIG_ZTE_HEADSET_BUTTONKEY_CAL
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/sysfs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/kthread.h>

#include "msm8x16-wcd.h"
#include "wcd-mbhc-v2.h"
#include "msm8916-wcd-irq.h"
#include "msm8x16_wcd_registers.h"

static struct proc_dir_entry *headsetkey_para_proc_entry = NULL;
static struct proc_dir_entry *headsetkey_check_proc_entry = NULL;
struct task_struct *headsetkey_thrd = NULL;
extern struct wcd_mbhc *mbhc_extern;
extern struct headsetkey_head headset_button;

static int cal_fail;
static u16 btn_key_value; /*headset btn key min cal value 25*/

extern void headsetkey_set_btn_calpara(struct wcd_mbhc *mbhc, u16 btn_calkey_val);

ssize_t headsetkey_write_allpara(struct file *filp, const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char lbuf[64];
	int i = 0;
	int rc = 0;
	char *opt;
	char *pbuf;
	unsigned long value = 0;

	pr_info("%s:\n", __func__);
	if (!filp || !ppos || !ubuf)
		return -EINVAL;

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';
	pbuf = lbuf;

	pr_info("%s:%s\n", __func__, lbuf);

	if (!strncmp(lbuf, "set_original_para", sizeof("set_original_para")-1)) {
		headset_button.use_keycalpara = false;
		cal_fail = 0;
	} else if (!strncmp(lbuf, "set_calkey_para", sizeof("set_calkey_para")-1)) {
		headset_button.use_keycalpara = true;
		cal_fail = 0;
	}
	while ((opt = strsep(&pbuf, ",")) != NULL) {
		if (!*opt)
			continue;

		while (*opt == ' ') {
			opt++;
			if (*opt == '\0')
				break;
		}

		rc = kstrtoul(opt, 10, &value);
		if (rc < 0) {
			pr_err("%s: Failed to convert \"%s\" to hex number\n", __func__, opt);
			continue;
		}
		pr_info("%s:val[%d]=string:%s convert to %ld\n", __func__, i, opt, value);

		if ((value > 0) && (value <= MAX_KEYVAL)) {
			if (i < 5)
				headset_button.btn_callow[i] = (u16)value;
			else
				headset_button.btn_calhigh[i-5] = (u16)value;
		} else {
			pr_err("%s:value error=%ld\n", __func__, value);
			return -EINVAL;
		}
		i++;
		if (i == 10)/*all key para count is 10*/
			break;
	}
	headsetkey_set_btn_calpara(mbhc_extern, 0xffff);/*btn_calkey_val ==0xffff force update para*/
	return cnt;
}

ssize_t headsetkey_read_allpara(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[64] = {0};
	int blen = 0;

	pr_info("%s:%d\n", __func__, (int)*ppos);
	if (*ppos)
		return 0;

	blen = snprintf(buffer, sizeof(buffer), "headsetkey_para,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\n",
		headset_button.btn_callow[0], headset_button.btn_callow[1], headset_button.btn_callow[2],
		headset_button.btn_callow[3], headset_button.btn_callow[4], headset_button.btn_calhigh[0],
		headset_button.btn_calhigh[1], headset_button.btn_calhigh[2], headset_button.btn_calhigh[3],
		headset_button.btn_calhigh[4]);
	if (blen < 0)
		return 0;

	if (copy_to_user(buf, buffer, blen))
		return -EFAULT;

	pr_info("%s: %s read count %ld,%d\n", __func__, buffer, count, blen);

	*ppos += blen;
	return blen;
}

int headsetkey_cycle_checking(void *data)
{
	u16 times_counts = 0;
	u16 btn_key_value_old = 0;
	int i;
	int btn;

	pr_info("%s:\n", __func__);
	cal_fail = 0;
	headset_button.cal_begin = true;
	headset_button.cal_presskeyok = false;
	headset_button.cal_singlekey_ok = false;
	headset_button.cal_press_key[HEADSETKEY_MD] = false;
	headset_button.cal_press_key[HEADSETKEY_UP] = false;
	headset_button.cal_press_key[HEADSETKEY_DN] = false;
	btn_key_value_old = btn_key_value;
	headsetkey_set_btn_calpara(mbhc_extern, btn_key_value);
	msleep(100);

	while (headset_button.cal_begin == true) {
		if (headset_button.cal_presskeyok == true) {
			if (headset_button.cal_press_key[headset_button.keynumber] == true) {
				pr_info("%s:cal keynumber=%d ok\n", __func__, headset_button.keynumber);
				break;
			}
			btn_key_value += CALKEY_STEP;
			pr_info("%s:cal btn keyvalue=%d\n", __func__, btn_key_value);
			if (btn_key_value >=  MAX_KEYVAL - CALKEY_STEP*2) {
				pr_err("%s:cal fail keyvalue beyond max\n", __func__);
				cal_fail++;
				headset_button.cal_begin = false;
				btn_key_value = btn_key_value_old;
				return cal_fail;
			}
			headsetkey_set_btn_calpara(mbhc_extern, btn_key_value);
			msleep(100);/*wait btn release then press message, so codec will update btn value*/

			btn = mbhc_extern->mbhc_cb->map_btn_code_to_num(mbhc_extern->codec);
			pr_info("%s:cal msleep ms,btn=%d,keynumber=%d\n", __func__, btn, headset_button.keynumber);
			if (headset_button.keynumber == btn)
				headset_button.cal_press_key[headset_button.keynumber] = true;
		} else {
			pr_info("%s:cal nopress times_counts=%d\n", __func__, times_counts);
			times_counts++;
			if (times_counts > 20) {
				pr_err("%s:cal fail timeout\n", __func__);
				cal_fail++;
				headset_button.cal_begin = false;
				return btn_key_value_old;
			}
			msleep(500);
		}
	}

	pr_info("%s:cal old=%d,temp=%d\n", __func__, btn_key_value_old, btn_key_value);
	if (btn_key_value_old >= btn_key_value) {
		pr_err("%s:cal btn_key_value wrong", __func__);
		cal_fail++;
		headset_button.cal_begin = false;
		btn_key_value = btn_key_value_old;
		return cal_fail;
	}

	if (!headset_button.micbias2)/*if micbias, interval 50*/
		headset_button.btn_callow[headset_button.keynumber] =
			btn_key_value + headset_button.keynumber * CALKEY_STEP*2;
	else
		headset_button.btn_calhigh[headset_button.keynumber] =
			btn_key_value + headset_button.keynumber * CALKEY_STEP*2;

	pr_info("%s:cal keynumber=%d ok, btn_key_value=%d\n", __func__, headset_button.keynumber, btn_key_value);
	headset_button.cal_begin = false;
	headset_button.cal_singlekey_ok = true;

	if (headset_button.keynumber == HEADSETKEY_DN) {
		pr_info("%s:cal three key button finished\n", __func__);
		if (!headset_button.micbias2) {
			headset_button.btn_callow[3] = MAX_KEYVAL-CALKEY_STEP;
			headset_button.btn_callow[4] = MAX_KEYVAL;
			for (i = 0; i < 3; i++) {
				headset_button.btn_callow[i] =
					(headset_button.btn_callow[i] + headset_button.btn_callow[i+1])/2;
			}
		} else {
			headset_button.btn_calhigh[3] = MAX_KEYVAL-CALKEY_STEP;
			headset_button.btn_calhigh[4] = MAX_KEYVAL;
			for (i = 0; i < 3; i++) {
				headset_button.btn_calhigh[i] =
					(headset_button.btn_calhigh[i] + headset_button.btn_calhigh[i+1])/2;
			}
		}
	}

	return cal_fail;
}

static int headsetkey_thread_start(void)
{
	int ret = 0;

	pr_info("%s:\n", __func__);
	headsetkey_thrd = kthread_run(headsetkey_cycle_checking, (void *)NULL, "headsetkey_check");
	if (IS_ERR(headsetkey_thrd)) {
		ret = PTR_ERR(headsetkey_thrd);
		headsetkey_thrd = NULL;
	}
	return ret;
}

ssize_t headsetkey_write_dochecking(struct file *filp, const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char lbuf[64];
	int rc;

	pr_info("%s:\n", __func__);
	if (!filp || !ppos || !ubuf)
		return -EINVAL;

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';

	if (mbhc_extern->mbhc_cb->micbias_enable_status) {
		headset_button.micbias2 = mbhc_extern->mbhc_cb->micbias_enable_status(mbhc_extern, MIC_BIAS_2);
	}
	pr_info("%s: micbias2=%d,cal_fail=%d\n", __func__, headset_button.micbias2, cal_fail);
	if (!strncmp(lbuf, "startcal_low", sizeof("startcal_low")-1) && !headset_button.micbias2) {
		if (!strncmp(lbuf, "startcal_low_md", sizeof("startcal_low_md")-1)) {
			headset_button.keynumber = HEADSETKEY_MD;
			btn_key_value = 0;
		} else if (!strncmp(lbuf, "startcal_low_up", sizeof("startcal_low_up")-1)) {
			headset_button.keynumber = HEADSETKEY_UP;
			/*btn_key_value = 0;*/
		} else if (!strncmp(lbuf, "startcal_low_dn", sizeof("startcal_low_dn")-1)) {
			headset_button.keynumber = HEADSETKEY_DN;
			/*btn_key_value = 0;*/
		}
	} else if (!strncmp(lbuf, "startcal_high", sizeof("startcal_high")-1) && headset_button.micbias2) {
		if (!strncmp(lbuf, "startcal_high_md", sizeof("startcal_high_md")-1)) {
			headset_button.keynumber = HEADSETKEY_MD;
			btn_key_value = 0;
		} else if (!strncmp(lbuf, "startcal_high_up", sizeof("startcal_high_up")-1)) {
			headset_button.keynumber = HEADSETKEY_UP;
			/*btn_key_value = 0;*/
		} else if (!strncmp(lbuf, "startcal_high_dn", sizeof("startcal_high_dn")-1)) {
			headset_button.keynumber = HEADSETKEY_DN;
			/*btn_key_value = 0;*/
		}
	} else {
		pr_err("%s:wrong cmd\n", __func__);
		cal_fail++;
		return cnt;
	}

	headsetkey_thread_start();

	pr_info("%s:btn_key_value = %d\n", __func__, btn_key_value);
	return cnt;
}
ssize_t headsetkey_read_checkresult(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[64] = {0};
	int blen = 0;

	pr_info("%s:%d\n", __func__, (int)*ppos);
	if (*ppos)
		return 0;

	if (headset_button.cal_begin) {
		blen = snprintf(buffer, sizeof(buffer), "headsetkey_calpara unfinished\n");
	} else {
		if (cal_fail) {
			blen = snprintf(buffer, sizeof(buffer), "headsetkey_calpara_ng\n");
		} else {
			blen = snprintf(buffer, sizeof(buffer), "headsetkey_calpara_ok\n");
		}
	}
	if (blen < 0)
		return 0;

	if (copy_to_user(buf, buffer, blen))
		return -EFAULT;

	pr_info("%s: %s read count %ld,%d\n", __func__, buffer, count, blen);

	*ppos += blen;

	return blen;
}
static const struct file_operations headsetkey_para_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = headsetkey_read_allpara,
	.write = headsetkey_write_allpara,
};
static const struct file_operations headsetkey_check_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = headsetkey_read_checkresult,
	.write = headsetkey_write_dochecking,
};
int headsetkey_init_info_node(void)
{
	headsetkey_para_proc_entry = proc_create_data("driver/headsetkey_para",
		0644, NULL, &headsetkey_para_fops, NULL);
	if (headsetkey_para_proc_entry == NULL) {
		pr_err("%s: Create proc entry driver/headsetkey_para FAILED!\n", __func__);
		return -EFAULT;
	}

	headsetkey_check_proc_entry = proc_create_data("driver/headsetkey_check",
		0644, NULL, &headsetkey_check_fops, NULL);
	if (headsetkey_check_proc_entry == NULL) {
		pr_err("%s: Create proc entry driver/headsetkey_check FAILED!\n", __func__);
		return -EFAULT;
	}
	pr_info("%s: Create proc entry SUCCESS!\n", __func__);
	return 0;
}
#endif /*#ifdef CONFIG_ZTE_HEADSET_BUTTONKEY_CAL*/

