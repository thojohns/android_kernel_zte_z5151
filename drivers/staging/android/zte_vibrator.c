/* drivers/misc/timed_gpio.c
 *
 * opyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "timed_output.h"
#include "timed_gpio.h"

#define ZTE_VIBRATOR_GPIO 25
#define ZTE_VIBRATOR_TIMEOUT_MS_MAX 15000

#define MAX_SUPPORT_GPIOS 2
#define ZTE_PM_GPIO_NUM 1

struct vibrator_gpio_info {
	int sys_num;
	const char *name;
};

struct zte_timed_gpio_data {
	struct timed_output_dev dev;
	struct hrtimer timer;
	spinlock_t lock;    /* spinlock for vibrator */
	unsigned gpio;
	int max_timeout;
	u8 active_low;
};

static struct vibrator_gpio_info zte_gpios[MAX_SUPPORT_GPIOS];
static int vibrator_pm_gpio = -1;
extern void zte_vibrator_spmi_write(int gpio);

static enum hrtimer_restart gpio_timer_func(struct hrtimer *timer)
{
	struct zte_timed_gpio_data *data =
		container_of(timer, struct zte_timed_gpio_data, timer);

	gpio_direction_output(data->gpio, data->active_low ? 1 : 0);
	gpio_direction_output(vibrator_pm_gpio, 0);
	return HRTIMER_NORESTART;
}

static int gpio_get_time(struct timed_output_dev *dev)
{
	struct zte_timed_gpio_data *data;
	struct timeval t;

	data = container_of(dev, struct zte_timed_gpio_data, dev);

	if (!hrtimer_active(&data->timer))
		return 0;

	t = ktime_to_timeval(hrtimer_get_remaining(&data->timer));

	return t.tv_sec * 1000 + t.tv_usec / 1000;
}

static void gpio_enable(struct timed_output_dev *dev, int value)
{
	struct zte_timed_gpio_data	*data =
		container_of(dev, struct zte_timed_gpio_data, dev);
	unsigned long	flags;

	spin_lock_irqsave(&data->lock, flags);

	/* cancel previous timer and set GPIO according to value */
	hrtimer_cancel(&data->timer);
	gpio_direction_output(vibrator_pm_gpio, 1);
	gpio_direction_output(data->gpio, data->active_low ? !value : !!value);
	zte_vibrator_spmi_write(ZTE_PM_GPIO_NUM);

	if (value > 0) {
		if (value > data->max_timeout)
			value = data->max_timeout;

		hrtimer_start(&data->timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&data->lock, flags);
}

static int get_sysnumber_byname(char *name)
{
	int i;

	for (i = 0; i < MAX_SUPPORT_GPIOS; i++) {
		if (zte_gpios[i].name) {
			if (!strcmp(zte_gpios[i].name, name))
				return zte_gpios[i].sys_num;
		}
	}
	return 0;
}

static int get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	int count = -1;

	pr_info("zte_vibrator: translate hardware pin to system pin\n");
	node = dev->of_node;
	if (!node)
		return -ENODEV;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (!of_find_property(pp, "label", NULL)) {
			dev_warn(dev, "Found without labels\n");
			continue;
		}
		count++;
		zte_gpios[count].name = kstrdup(of_get_property(pp, "label", NULL),
								GFP_KERNEL);
		zte_gpios[count].sys_num = of_get_named_gpio(pp, zte_gpios[count].name, 0);
		pr_info("zte_vibrator: sys_number=%d name=%s\n", zte_gpios[count].sys_num, zte_gpios[count].name);
	}
	return 0;
}

static int zte_vibrator_probe(struct platform_device *pdev)
{
	struct timed_gpio *cur_gpio;
	struct zte_timed_gpio_data *gpio_data, *gpio_dat;
	int i, ret;
	struct timed_gpio gpio = {
		.name = "vibrator",
		.gpio = ZTE_VIBRATOR_GPIO,
		.max_timeout = ZTE_VIBRATOR_TIMEOUT_MS_MAX,
		.active_low = 0,
	};
	struct timed_gpio_platform_data pdata = {
		.num_gpios = 1,
		.gpios = &gpio,
	};
	struct device *dev = &pdev->dev;

	pr_info("zte_vibrator_probe\n");
	ret = get_devtree_pdata(dev);
	if (ret)
		return ret;

	vibrator_pm_gpio = get_sysnumber_byname("vibrator_pm_gpio");

	gpio_data = devm_kzalloc(&pdev->dev,
			sizeof(struct zte_timed_gpio_data) * pdata.num_gpios,
			GFP_KERNEL);
	if (!gpio_data)
		return -ENOMEM;

	for (i = 0; i < pdata.num_gpios; i++) {
		cur_gpio = &pdata.gpios[i];
		gpio_dat = &gpio_data[i];

		hrtimer_init(&gpio_dat->timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
		gpio_dat->timer.function = gpio_timer_func;
		spin_lock_init(&gpio_dat->lock);

		gpio_dat->dev.name = cur_gpio->name;
		gpio_dat->dev.get_time = gpio_get_time;
		gpio_dat->dev.enable = gpio_enable;
		cur_gpio->gpio = vibrator_pm_gpio;
		ret = gpio_request(cur_gpio->gpio, cur_gpio->name);
		if (ret < 0)
			goto err_out;
		ret = timed_output_dev_register(&gpio_dat->dev);
		if (ret < 0) {
			gpio_free(cur_gpio->gpio);
			goto err_out;
		}

		gpio_dat->gpio = cur_gpio->gpio;
		gpio_dat->max_timeout = cur_gpio->max_timeout;
		gpio_dat->active_low = cur_gpio->active_low;
		gpio_direction_output(gpio_dat->gpio, gpio_dat->active_low);
	}

	platform_set_drvdata(pdev, gpio_data);
	pr_info("zte_vibrator_probe success\n");

	return 0;

err_out:
	while (--i >= 0) {
		timed_output_dev_unregister(&gpio_data[i].dev);
		gpio_free(gpio_data[i].gpio);
	}

	devm_kfree(&pdev->dev, gpio_data);
	return ret;
}

static int zte_vibrator_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id zte_vibrator_of_match[] = {
	{ .compatible = "zte-vibrator", },
	{ },
};

static struct platform_driver zte_vibrator_device_driver = {
	.probe		= zte_vibrator_probe,
	.remove		= zte_vibrator_remove,
	.driver		= {
		.name	= "zte-vibrator",
		.owner	= THIS_MODULE,
		.of_match_table = zte_vibrator_of_match,
	}
};

int __init zte_vibrator_init(void)
{
	return platform_driver_register(&zte_vibrator_device_driver);
}

static void __exit zte_vibrator_exit(void)
{
	platform_driver_unregister(&zte_vibrator_device_driver);
}

fs_initcall(zte_vibrator_init);
module_exit(zte_vibrator_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("driver for zte vibator");
MODULE_ALIAS("platform:zte-vibrator");
