/*
 *This program is used for recode the ap and modem's sleep and wake time.
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/smp.h>
#include <linux/tick.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/msm-bus.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <soc/qcom/spm.h>
#include <soc/qcom/pm.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/scm-boot.h>
#include <asm/suspend.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/system_misc.h>

#include <soc/qcom/socinfo.h>
#include "../clk/msm/clock.h"
#include <linux/seq_file.h>

#include <linux/fb.h>

static int msm_pm_debug_mask = 1;
module_param_named(
	debug_mask, msm_pm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

/*ZTE_PM ++++ Interface For Ril Open F3 Log*/
static int apSleep_modemAwake_timeThreshold = 10;
static int apSleep_modemAwake_precent = 900;
static int apSleep_modemAwake_count = 5;
module_param_named(zte_amss_time_threshold, apSleep_modemAwake_timeThreshold, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(zte_amss_awake_precent, apSleep_modemAwake_precent, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(zte_amss_awake_acount, apSleep_modemAwake_count, int, S_IRUGO | S_IWUSR | S_IWGRP);
static int zte_amss_acount;
static struct device  *msm_cpu_pm_dev;
/*ZTE_PM ++++ GPIO*/
#ifndef ZTE_GPIO_DEBUG
#define ZTE_GPIO_DEBUG
#endif
/*ZTE_PM ++++ GPIO*/

/*ZTE ++++ */
#ifndef LCD_ON_TIME_ZTE
#define LCD_ON_TIME_ZTE
#endif

/*ZTE ---- */

enum {
#ifdef ZTE_GPIO_DEBUG
	MSM_PM_DEBUG_ZTE_LOGS = BIT(9),
#endif
	MSM_PM_DEBUG_ZTE_IDLE_CLOCK = BIT(10),/* LOG default not open*/
};

typedef struct {
	uint32_t app_suspend_state;
	uint32_t modemsleeptime;
	uint32_t modemawaketime;
	uint32_t modemsleep_or_awake;/*1 sleep,2 awake,0 never enter sleep*/
	uint32_t physlinktime;
	uint32_t modemawake_timeout_crash;
} pm_count_time;

pm_count_time *zte_imem_ptr = NULL;

static int kernel_sleep_count;

/*ZTE_PM ++++ GPIO*/
#ifdef ZTE_GPIO_DEBUG

extern int msm_dump_gpios(struct seq_file *s, int curr_len, char *gpio_buffer);
extern int pmic_dump_pins(struct seq_file *s, int curr_len, char *gpio_buffer);
static char *gpio_sleep_status_info;

int print_gpio_buffer(struct seq_file *s)
{
	if (gpio_sleep_status_info)
		seq_printf(s, gpio_sleep_status_info);
	else
		seq_puts(s, "Device haven't suspended yet!\n");

	return 0;
}
EXPORT_SYMBOL(print_gpio_buffer);

int free_gpio_buffer(void)
{
	kfree(gpio_sleep_status_info);
	gpio_sleep_status_info = NULL;

	return 0;
}
EXPORT_SYMBOL(free_gpio_buffer);

#endif
/*ZTE_PM ---- GPIO*/

/*ZTE LCD ++++ */
#ifdef LCD_ON_TIME_ZTE
long mTimeFromLateresumeToEarlysuspend = 0;
void zte_update_lateresume_to_earlysuspend_time(bool resume_or_earlysuspend)
{
	pr_info("[PM] turn LCD %s\n", resume_or_earlysuspend ? "ON" : "OFF");

	if (resume_or_earlysuspend)
		mTimeFromLateresumeToEarlysuspend = current_kernel_time().tv_sec;
	else
		mTimeFromLateresumeToEarlysuspend = current_kernel_time().tv_sec - mTimeFromLateresumeToEarlysuspend;
}

static int lcd_fb_callback(struct notifier_block *nfb,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		/*pr_info("[PM] %s enter , blank=%d\n", __func__, *blank);*/

		if (*blank == FB_BLANK_UNBLANK) {
			/*notes:update resume time,
			indicate the LCD will turn on.*/
			zte_update_lateresume_to_earlysuspend_time(true);
		} else if ((*blank == FB_BLANK_POWERDOWN) || (*blank == FB_BLANK_NORMAL)) {
			/*notes:update suspend time,
			indicate the LCD will turn off.*/
			zte_update_lateresume_to_earlysuspend_time(false);
		}
	}
	return 0;
}

static struct notifier_block __refdata lcd_fb_notifier = {
	.notifier_call = lcd_fb_callback,
};
#endif
/*ZTE LCD ---- */

/*ZTE_PM ++++ Ap sleep and aweak*/
#ifndef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
/*#define ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED*/
#endif

#ifdef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
static zte_smem_global *zte_global;
#endif

#ifndef RECORD_APP_AWAKE_SUSPEND_TIME_ZTE
#define RECORD_APP_AWAKE_SUSPEND_TIME_ZTE
#endif

#ifdef RECORD_APP_AWAKE_SUSPEND_TIME_ZTE

#define MSM_PM_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & msm_pm_debug_mask) \
			printk(level message, ## __VA_ARGS__); \
	} while (0)

unsigned pm_modem_sleep_time_get(void)
{
	if (!zte_imem_ptr) {
		pr_err("zte_imem_ptr is null,pm_modem_sleep_time_get return\n");
		return 0;
	}
	pr_info("[PM] get modemsleeptime %d\n", zte_imem_ptr->modemsleeptime);
	return zte_imem_ptr->modemsleeptime;
}

unsigned pm_modem_phys_link_time_get(void)
{
	if (!zte_imem_ptr) {
		pr_err("zte_imem_ptr is null,pm_modem_phys_link_time_get return\n");
		return 0;
	}
	pr_info("[PM] get physlinktime %d\n", zte_imem_ptr->physlinktime);
	return zte_imem_ptr->physlinktime;
}

unsigned pm_modem_awake_time_get(int *current_sleep)
{
	if (!zte_imem_ptr) {
		pr_err("zte_imem_ptr is null,pm_modem_awake_time_get return\n");
		return 0;
	}
	*current_sleep =  zte_imem_ptr->modemsleep_or_awake;
	pr_info("[PM] get modemawaketime %d,current_sleep=%d\n", zte_imem_ptr->modemawaketime, *current_sleep);
	return zte_imem_ptr->modemawaketime;
}

/*Interface For Ril Open F3 Log*/
static int zte_amss_invalid_parameter(void)
{
	if (apSleep_modemAwake_count <= 0)
		return 0;
	else if (apSleep_modemAwake_precent < 500 || apSleep_modemAwake_precent > 1000)
		return 0;
	else if (apSleep_modemAwake_timeThreshold <= 0)
		return 0;
	else
		return 1;
}

static  void  zte_amss_updateEvent(int modemState)
{
	char *event = NULL;
	char *envp[2];
	const char *name;

	name = modemState?"OPEN":"CLOSE";
	event = kasprintf(GFP_KERNEL, "AMSS_PM_STATE=%s", name);
	envp[0] = event;
	envp[1] = NULL;

	if (msm_cpu_pm_dev == NULL) {
		pr_info("amss, msm_cpu_pm_dev is NULL");
	} else {
		kobject_uevent_env(&msm_cpu_pm_dev->kobj, KOBJ_CHANGE, envp);
	}
}

static int  zte_amss_needF3log(int apSleep_time_s, int modemAwake_percent)
{
	int invalidParameter = 0;

	invalidParameter = zte_amss_invalid_parameter();

	if (apSleep_time_s < 0 || (modemAwake_percent < 900 || modemAwake_percent > 1000))
		return 0;
	if (invalidParameter == 0)
		return 0;

	if (zte_amss_acount > apSleep_modemAwake_count)
		zte_amss_acount = 0;

	zte_amss_acount = zte_amss_acount + 1;
	if ((zte_amss_acount == apSleep_modemAwake_count) && invalidParameter == 1)
		return 1;
	else
		return 0;
}

#define AMSS_NEVER_ENTER_SLEEP 0x4
#define AMSS_NOW_SLEEP 0x0
#define AMSS_NOW_AWAKE 0x1
#define THRESOLD_FOR_OFFLINE_AWAKE_TIME 100 /*ms*/
#define THRESOLD_FOR_OFFLINE_TIME 5000 /*s*/

static int mEnableRrecordFlag_ZTE;
module_param_named(zte_enableRecord,
	mEnableRrecordFlag_ZTE, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define RECORED_TOTAL_TIME
struct timespec time_updated_when_sleep_awake;
void record_sleep_awake_time(bool record_sleep_awake)
{
	struct timespec ts;
	unsigned time_updated_when_sleep_awake_s;
#ifdef RECORED_TOTAL_TIME
	static bool record_firsttime = true;
	static bool record_firsttime_modem = true;
	static	unsigned time_modem_firsttime_awake_s;
	static	unsigned time_modem_firsttime_sleep_s;
	static	unsigned time_app_total_awake_s;
	static	unsigned time_app_total_sleep_s;
	static unsigned time_lcdon_total_s;
#endif
	unsigned time_updated_when_sleep_awake_ms;
	unsigned time_updated_when_sleep_awake_ms_temp;
	static unsigned amss_sleep_time_ms = 0;
	static unsigned amss_physlink_current_total_time_s;
	static unsigned amss_physlink_last_total_time_s;
	unsigned amss_sleep_time_ms_temp = 0;
	unsigned deta_sleep_ms = 0;
	unsigned deta_awake_ms = 0;
	unsigned deta_physlink_s = 0;
	unsigned amss_awake_last = 0;
	int result_state = 0;

	unsigned amss_current_sleep_or_awake = 0;/*1 never enter sleep,2 sleep,3 awake*/
	static unsigned  amss_current_sleep_or_awake_previous;

	static unsigned amss_awake_time_ms;
	unsigned amss_awake_time_ms_temp = 0;
	bool get_amss_awake_ok = false;

	unsigned percentage_amss_not_sleep_while_app_suspend = 0;
	static bool sleep_success_flag;

	if (mEnableRrecordFlag_ZTE == 1) {
		pr_info("[PM]: not enable to record when app enter to suspend or resume yet!\n");
		return;
	}
	ts = current_kernel_time();

	time_updated_when_sleep_awake_ms_temp =	(unsigned) ((ts.tv_sec - time_updated_when_sleep_awake.tv_sec) *
		MSEC_PER_SEC + ((ts.tv_nsec / NSEC_PER_MSEC) -
		(time_updated_when_sleep_awake.tv_nsec / NSEC_PER_MSEC)));
	time_updated_when_sleep_awake_s = (time_updated_when_sleep_awake_ms_temp/MSEC_PER_SEC);
	time_updated_when_sleep_awake_ms = (time_updated_when_sleep_awake_ms_temp -
		time_updated_when_sleep_awake_s * MSEC_PER_SEC);

	/*ZTE:record app awake time*/
	if (record_sleep_awake) {
		sleep_success_flag = true;
		amss_sleep_time_ms_temp = amss_sleep_time_ms;
		amss_sleep_time_ms = pm_modem_sleep_time_get();
		deta_sleep_ms = amss_sleep_time_ms - amss_sleep_time_ms_temp;

		amss_awake_time_ms_temp = amss_awake_time_ms;
		amss_awake_time_ms  = pm_modem_awake_time_get(&amss_current_sleep_or_awake);
		deta_awake_ms = amss_awake_time_ms - amss_awake_time_ms_temp;

		amss_physlink_current_total_time_s = pm_modem_phys_link_time_get();
		deta_physlink_s = amss_physlink_current_total_time_s - amss_physlink_last_total_time_s;
		amss_physlink_last_total_time_s = amss_physlink_current_total_time_s;

		/*
		amss_current_sleep_or_awake_previous  amss_current_sleep_or_awake
		X 4 ---modem not enter sleep yet
		0 0 ---previous is sleep,curret is sleep,
				modem awake time is updated,get awake deta directly.
		otherwise get modem sleep time.
		if modem is set to offline,print offline in the log
		*/

		if ((amss_current_sleep_or_awake_previous == AMSS_NOW_SLEEP) &&
				(amss_current_sleep_or_awake == AMSS_NOW_SLEEP)) {
			/*ZTE:if sleep time is 0 and awake is 0,offline mode*/
			if (deta_awake_ms < THRESOLD_FOR_OFFLINE_AWAKE_TIME) {
				if (time_updated_when_sleep_awake_ms_temp > THRESOLD_FOR_OFFLINE_TIME)
					pr_info("[PM] offline mode\n");
			}
			get_amss_awake_ok = true;
			amss_awake_last = deta_awake_ms;
		} else if (amss_current_sleep_or_awake == AMSS_NEVER_ENTER_SLEEP) {
			pr_info("[PM] modem not enter sleep yet\n");
		}

		if (!get_amss_awake_ok) {
			amss_awake_last = time_updated_when_sleep_awake_ms_temp - deta_sleep_ms;
		}
		percentage_amss_not_sleep_while_app_suspend =
				(amss_awake_last * 1000/(time_updated_when_sleep_awake_ms_temp + 1));

#ifdef RECORED_TOTAL_TIME
		if (!record_firsttime) {
			time_app_total_awake_s += time_updated_when_sleep_awake_s;
			time_lcdon_total_s += mTimeFromLateresumeToEarlysuspend;
		}
		record_firsttime = false;
#endif
		pr_info("[PM] APP wake for %6d.%03d s, lcd on for %5d s %3d %%\n",
			time_updated_when_sleep_awake_s, time_updated_when_sleep_awake_ms,
			(int) mTimeFromLateresumeToEarlysuspend,
			(int)(mTimeFromLateresumeToEarlysuspend * 100/(time_updated_when_sleep_awake_s + 1)));
		pr_info("[PM] modem wake for %10d ms(%s) %4d %%o,modem sleep for %10d --%d%d\n",
			amss_awake_last, get_amss_awake_ok ? "get_directly " : "from sleep_time",
			percentage_amss_not_sleep_while_app_suspend,
			deta_sleep_ms, amss_current_sleep_or_awake_previous,
			amss_current_sleep_or_awake);/*in case Division by zero, +1*/

		pr_info("[PM] modem_phys_link_total_time %4d min %4d s\n",
			amss_physlink_current_total_time_s/60,
			amss_physlink_current_total_time_s%60);
		pr_info("[PM] deta_physlink_s %4d min %4d s during app wake\n",
			deta_physlink_s/60,	deta_physlink_s%60);

		time_updated_when_sleep_awake = ts;
		mTimeFromLateresumeToEarlysuspend = 0;/*ZTE:clear how long the lcd keeps on*/
	} else {
		/*ZTE:record app sleep time*/
		if (!sleep_success_flag) {
			pr_info("[PM] app resume due to fail to suspend\n");
			return;
		}
		sleep_success_flag = false;
		amss_sleep_time_ms_temp = amss_sleep_time_ms;
		amss_sleep_time_ms  = pm_modem_sleep_time_get();
		deta_sleep_ms = amss_sleep_time_ms - amss_sleep_time_ms_temp;
		amss_awake_time_ms_temp = amss_awake_time_ms;
		amss_awake_time_ms  = pm_modem_awake_time_get(&amss_current_sleep_or_awake);
		deta_awake_ms = amss_awake_time_ms - amss_awake_time_ms_temp;

		amss_physlink_current_total_time_s = pm_modem_phys_link_time_get();
		deta_physlink_s = amss_physlink_current_total_time_s - amss_physlink_last_total_time_s;
		amss_physlink_last_total_time_s = amss_physlink_current_total_time_s;

		/*ZTE:00,get modem awake time*/
		if ((amss_current_sleep_or_awake_previous == AMSS_NOW_SLEEP) &&
			(amss_current_sleep_or_awake == AMSS_NOW_SLEEP)) {
			/*ZTE:if sleep time is 0 and awake is 0,offline mode*/
			if ((deta_awake_ms < THRESOLD_FOR_OFFLINE_AWAKE_TIME)
				&& (time_updated_when_sleep_awake_ms_temp > THRESOLD_FOR_OFFLINE_TIME)) {
				pr_info("[PM] offline mode\n");
			}
			get_amss_awake_ok = true;
			amss_awake_last = deta_awake_ms;
		} else if (amss_current_sleep_or_awake == AMSS_NEVER_ENTER_SLEEP) {
			pr_info("[PM] modem not enter sleep yet\n");
		}

		if (!get_amss_awake_ok)
			amss_awake_last = time_updated_when_sleep_awake_ms_temp - deta_sleep_ms;

#ifdef RECORED_TOTAL_TIME
		time_app_total_sleep_s += time_updated_when_sleep_awake_s;
		if (record_firsttime_modem) {
			time_modem_firsttime_awake_s = amss_awake_last/1000;
			time_modem_firsttime_sleep_s = amss_sleep_time_ms/1000;
			record_firsttime_modem = false;
		}
		pr_info("[PM] modem total sleep: %d s,modem total awake %d s\n",
			(amss_sleep_time_ms/1000 - time_modem_firsttime_sleep_s),
			(amss_awake_time_ms/1000 - time_modem_firsttime_awake_s));

		pr_info("[PM] app total sleep: %d s,app total awake: %d s,lcd on total: %d s\n",
			time_app_total_sleep_s, time_app_total_awake_s, time_lcdon_total_s);
#endif

		if (kernel_sleep_count > 10000) {
			kernel_sleep_count = 1;
			pr_info("[PM] init again, kernel_sleep_count=%d\n", kernel_sleep_count);
		} else {
			kernel_sleep_count = kernel_sleep_count+1;
			if (kernel_sleep_count%5 == 0)
				pr_info("[PM] kernel_sleep_count=%d\n", kernel_sleep_count);
		}

		percentage_amss_not_sleep_while_app_suspend =
				(amss_awake_last * 1000/(time_updated_when_sleep_awake_ms_temp + 1));

		pr_info("[PM] APP sleep for %3d.%03d s, modem wake %6d ms,(%s),%3d %%o\n",
			time_updated_when_sleep_awake_s,
			time_updated_when_sleep_awake_ms, amss_awake_last,
			get_amss_awake_ok ? "get_directly " : "from sleep_time",
			percentage_amss_not_sleep_while_app_suspend);
		pr_info("[PM] modem_sleep for %3d ---%d%d\n",
			deta_sleep_ms, amss_current_sleep_or_awake_previous,
			amss_current_sleep_or_awake);

		pr_info("[PM] PhysLinkTotalTime %4d min %4d, DetaPhyslink %4d min %4d in this time\n",
			amss_physlink_current_total_time_s/60,
			amss_physlink_current_total_time_s%60,
			deta_physlink_s/60,	deta_physlink_s%60);

		time_updated_when_sleep_awake = ts;

		/*Interface For Ril Open F3 Log*/
		result_state = zte_amss_needF3log(time_updated_when_sleep_awake_s,
								percentage_amss_not_sleep_while_app_suspend);
		zte_amss_updateEvent(result_state);
	}

	amss_current_sleep_or_awake_previous = amss_current_sleep_or_awake;
}
#endif

void zte_pm_before_powercollapse(void)
{
#ifdef ZTE_GPIO_DEBUG
	int curr_len = 0;/*Default close*/

	do {
		if (MSM_PM_DEBUG_ZTE_LOGS & msm_pm_debug_mask) {
			if (gpio_sleep_status_info) {
				memset(gpio_sleep_status_info, 0, sizeof(*gpio_sleep_status_info));
			} else {
				gpio_sleep_status_info = kmalloc(25000, GFP_KERNEL);
				if (!gpio_sleep_status_info) {
					pr_err("[PM] kmalloc memory failed in %s\n", __func__);
					break;
				}
			}

			/*ZTE_PM ++++ GPIO*/
			/*ZTE_PM: 1> echo 512 > sys/module/msm_pm/parameter/debug_mask
						2> let device sleep
						3> cat dump_sleep_gpios*/
			curr_len = msm_dump_gpios(NULL, curr_len, gpio_sleep_status_info);
			curr_len = pmic_dump_pins(NULL, curr_len, gpio_sleep_status_info);
			/*ZTE_PM ---- GPIO*/
		}
	} while (0);
#endif

#ifdef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
		if (zte_global == NULL) {
			zte_global = ioremap(ZTE_SMEM_LOG_GLOBAL_BASE, sizeof(zte_smem_global));
		}
		if (zte_global) {
			zte_global->app_suspend_state = 0xAA;
		}
#endif

#ifdef RECORD_APP_AWAKE_SUSPEND_TIME_ZTE
		record_sleep_awake_time(true);
#endif
}

/*ZTE_PM: called after exit PowerCollapse from suspend,
which will inform modem app has exit suspend.*/
void zte_pm_after_powercollapse(void)
{
#ifdef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
	if (zte_global)
		zte_global->app_suspend_state = 0;
#endif
}

static int zte_pm_debug_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("%s +++++\n", __func__);
	/*zte_pm Interface for Ril F3 LOG*/
	msm_cpu_pm_dev = &pdev->dev;

#ifdef ZTE_PM_NOTIFY_MODEM_APP_SUSPENDED
	zte_global = ioremap(ZTE_SMEM_LOG_GLOBAL_BASE, sizeof(zte_smem_global));
	if (zte_global)
		zte_global->app_suspend_state = 0;
#endif

#if 0
	zte_imem_ptr->modemsleeptime = 1000;
	zte_imem_ptr->modemawaketime = 2000;
	zte_imem_ptr->modemsleep_or_awake = 5;
#endif

#ifdef CONFIG_ZTE_BOOT_MODE
/*ZTE:Support for FTM & RECOVERY Mode,0: Normal mode,1: FTM mode*/
		if (socinfo_get_ftm_flag() == 1) {
			mEnableRrecordFlag_ZTE = 1;
			pr_info("[PM] set mEnableRrecordFlag_ZTE to true in FTM mode");
		}
#endif

	return ret;
}

static int zte_pm_debug_suspend(struct device *dev)
{
	zte_pm_before_powercollapse();
	return 0;
}

static int zte_pm_debug_resume(struct device *dev)
{
	pr_info("Resume DONE\n");
	record_sleep_awake_time(false);
	return 0;
}

static int  zte_pm_debug_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id zte_pm_debug_table[] = {
	{.compatible = "zte_pm_debug"},
	{},
};

static const struct dev_pm_ops zte_pm_debug_ops = {
	.suspend	= zte_pm_debug_suspend,
	.resume		= zte_pm_debug_resume,
};

static struct platform_driver zte_pm_debug_driver = {
	.probe = zte_pm_debug_probe,
	.remove	= zte_pm_debug_remove,
	.driver = {
		.name = "zte_pm_debug",
		.owner = THIS_MODULE,
		.pm	= &zte_pm_debug_ops,
		.of_match_table = zte_pm_debug_table,
	},
};

int __init zte_pm_debug_init(void)
{
	static bool registered;
	struct device_node *np; /*ZTE_PM*/

	if (registered)
		return 0;
	registered = true;

#ifdef LCD_ON_TIME_ZTE
	fb_register_client(&lcd_fb_notifier);
#endif

	pr_info("%s: e\n", __func__);
	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-pm-count-time");
	if (!np) {
		pr_err("unable to find DT imem msm-imem-pm-count-time node\n");
	} else {
		zte_imem_ptr = (pm_count_time  *)of_iomap(np, 0);
		if (!zte_imem_ptr)
			pr_err("unable to map imem golden copyoffset\n");
	}

	return platform_driver_register(&zte_pm_debug_driver);
}
late_initcall(zte_pm_debug_init);

static void __exit zte_pm_debug_exit(void)
{
	platform_driver_unregister(&zte_pm_debug_driver);
}
module_exit(zte_pm_debug_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("pm sleep wake time for zte");
MODULE_ALIAS("platform:zte_pm_debug");
