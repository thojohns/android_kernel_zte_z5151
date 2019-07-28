/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include "../../staging/android/timed_output.h"
/*add by heweiran*/
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#define QPNP_VIB_VTG_CTL(base)		(base + 0x41)
#define QPNP_VIB_EN_CTL(base)		(base + 0x46)

#define QPNP_VIB_MAX_LEVEL		31
#define QPNP_VIB_MIN_LEVEL		12

#define QPNP_VIB_DEFAULT_TIMEOUT	15000
#define QPNP_VIB_DEFAULT_VTG_LVL	3100

struct qpnp_vib {
	struct platform_device *pdev;
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;
	signed motor_en_gpio;
	u16 base;
	int state;
	int vtg_level;
	int timeout;
	struct mutex lock;
};

static int qpnp_vib_set(struct qpnp_vib *vib, int on)
{
	dev_info(&vib->pdev->dev, "qpnp_vib_set on is %d\n", on);
	if (on) {
		/*added by heweran for pwr, begin*/
		if (gpio_is_valid(vib->motor_en_gpio))
			gpio_set_value(vib->motor_en_gpio, 1);
	} else {
		if (gpio_is_valid(vib->motor_en_gpio))
			gpio_set_value(vib->motor_en_gpio, 0);
	}
	return 0;
}

static void qpnp_vib_enable(struct timed_output_dev *dev, int value)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib, timed_dev);

	mutex_lock(&vib->lock);
	hrtimer_cancel(&vib->vib_timer);
	dev_info(&vib->pdev->dev, "qpnp_vib_enable vaule is %d\n", value);
	if (value == 0) {
		vib->state = 0;
	} else {
		value = (value > vib->timeout ? vib->timeout : value);
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
					ktime_set(value / 1000, (value % 1000) * 1000000),
					HRTIMER_MODE_REL);
		pr_info("qpnp_vib_enable\n");
	}
	mutex_unlock(&vib->lock);
	schedule_work(&vib->work);
}

static void qpnp_vib_update(struct work_struct *work)
{
	struct qpnp_vib *vib = container_of(work, struct qpnp_vib, work);

	qpnp_vib_set(vib, vib->state);
}

static int qpnp_vib_get_time(struct timed_output_dev *dev)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib, timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);

		pr_info("qpnp_vib_get_time\n");
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart qpnp_vib_timer_func(struct hrtimer *timer)
{
	struct qpnp_vib *vib = container_of(timer, struct qpnp_vib, vib_timer);

	vib->state = 0;
	schedule_work(&vib->work);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_PM
static int qpnp_vibrator_suspend(struct device *dev)
{
	struct qpnp_vib *vib = dev_get_drvdata(dev);

	pr_info("qpnp_vibrator_suspend\n");

	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	/* turn-off vibrator */
	qpnp_vib_set(vib, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(qpnp_vibrator_pm_ops, qpnp_vibrator_suspend, NULL);

static int qpnp_vib_parse_dt(struct qpnp_vib *vib)
{
	struct platform_device *pdev = vib->pdev;
	int rc;

	vib->timeout = QPNP_VIB_DEFAULT_TIMEOUT;
	vib->vtg_level = QPNP_VIB_DEFAULT_VTG_LVL;
	vib->vtg_level /= 100;
	if (vib->vtg_level < QPNP_VIB_MIN_LEVEL)
		vib->vtg_level = QPNP_VIB_MIN_LEVEL;
	else if (vib->vtg_level > QPNP_VIB_MAX_LEVEL)
		vib->vtg_level = QPNP_VIB_MAX_LEVEL;

	/*get motor_en_gpio*/
	vib->motor_en_gpio = of_get_named_gpio(pdev->dev.of_node, "zte,motor_en_gpio", 0);
	if (!gpio_is_valid(vib->motor_en_gpio)) {
		pr_info("motor_en_gpio is invalid.\n");
		return -EINVAL;
	}
	rc = gpio_request(vib->motor_en_gpio, "vibrator_motor_en_gpio");
	if (rc) {
		dev_err(&pdev->dev, "Failed to request motor_en_gpio.\n");
		return -EINVAL;
	}
	gpio_direction_output(vib->motor_en_gpio, 0);

	return 0;
}

static int qpnp_vibrator_probe(struct platform_device *pdev)
{
	struct qpnp_vib *vib;
	int rc;

	pr_info("ztevibrator probe1 ");

	vib = devm_kzalloc(&pdev->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->pdev = pdev;
	vib->motor_en_gpio = -EINVAL;

	if (pdev->dev.of_node == NULL) {
		dev_info(&pdev->dev, "can not find device tree node\n");
		devm_kfree(&pdev->dev, vib);
		return -ENODEV;
	}

	rc = qpnp_vib_parse_dt(vib);
	if (rc) {
		dev_err(&pdev->dev, "DT parsing failed\n");
		devm_kfree(&pdev->dev, vib);
		return rc;
	}
	mutex_init(&vib->lock);
	INIT_WORK(&vib->work, qpnp_vib_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = qpnp_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = qpnp_vib_get_time;
	vib->timed_dev.enable = qpnp_vib_enable;

	dev_set_drvdata(&pdev->dev, vib);
	pr_err("ztevibrator probe");

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0) {
		devm_kfree(&pdev->dev, vib);
		return rc;
	}

	pr_info("ztevibrator probe2");

	return rc;
}

static int qpnp_vibrator_remove(struct platform_device *pdev)
{
	struct qpnp_vib *vib = dev_get_drvdata(&pdev->dev);

	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);
	mutex_destroy(&vib->lock);

	return 0;
}

static struct of_device_id vibrator_match_table[] = {
	{	.compatible = "zte_vibrator",
	},
	{}
};

static struct platform_driver qpnp_vibrator_driver = {
	.driver		= {
		.name	= "qcom,qpnp-vibrator",
		.of_match_table = vibrator_match_table,
		.pm	= &qpnp_vibrator_pm_ops,
	},
	.probe		= qpnp_vibrator_probe,
	.remove		= qpnp_vibrator_remove,
};

static int __init qpnp_vibrator_init(void)
{
	return platform_driver_register(&qpnp_vibrator_driver);
}
module_init(qpnp_vibrator_init);

static void __exit qpnp_vibrator_exit(void)
{
	return platform_driver_unregister(&qpnp_vibrator_driver);
}
module_exit(qpnp_vibrator_exit);

MODULE_DESCRIPTION("qpnp vibrator driver");
MODULE_LICENSE("GPL v2");
