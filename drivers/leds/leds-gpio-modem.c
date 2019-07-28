/*
 * leds-leds-gpio-modem.c - MSM qmi LEDs driver.
 *
 * Copyright (c) 2009, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is disributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * Added by zte_sw for modem control led
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <soc/qcom/sysmon.h>
#include <linux/pinctrl/consumer.h>

struct BLINK_LED_data {
	int blink_flag;
	int blink_on_time;
	int blink_off_time;
	struct led_classdev led;
};

struct STATUS_LED_data {
	struct mutex data_lock;
	struct BLINK_LED_data blink_led[3];  /* red, green, trickle*/
	int red_led;
	int green_led;
	int trickle_red_led;
	int gpio_base;
	int led_suspend_flag;
};

static struct of_device_id gpio_led_of_match[] = {
	{.compatible = "gpio-modem-leds",},
	{},
};

struct STATUS_LED_data *STATUS_LED = NULL;

static int send_info = 0;

static void gpio_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	char buffer[15];
	if (!strcmp(led_cdev->name, "red")) {
		if (value == LED_OFF) {
			gpio_direction_output(STATUS_LED->red_led, 0);
		} else {
			gpio_direction_output(STATUS_LED->red_led, 1);
		}
	} else if (!strcmp(led_cdev->name, "green")) {
		if (value == LED_OFF) {
			gpio_direction_output(STATUS_LED->green_led, 0);
		} else {
			gpio_direction_output(STATUS_LED->green_led, 1);
		}
	}

	if (!send_info) {
		int rc = 0;
		snprintf(buffer, sizeof(buffer), "%d,%d", STATUS_LED->red_led - STATUS_LED->gpio_base,
		STATUS_LED->green_led - STATUS_LED->gpio_base);
		rc |= sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_GPIO, buffer);
		snprintf(buffer, sizeof(buffer), "%d,%d,%d", STATUS_LED->blink_led[0].blink_flag,
		STATUS_LED->blink_led[0].blink_on_time, STATUS_LED->blink_led[0].blink_off_time);
		rc |= sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_RED_BLINK, buffer);
		snprintf(buffer, sizeof(buffer), "%d,%d,%d", STATUS_LED->blink_led[1].blink_flag,
		STATUS_LED->blink_led[1].blink_on_time, STATUS_LED->blink_led[1].blink_off_time);
		rc |= sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_GREEN_BLINK, buffer);
		if (!rc)
			send_info = 1;
		else {
			dev_err(led_cdev->dev, "led send info error in %s, %s\n", __func__, led_cdev->name);
			return;
		}
	}

	snprintf(buffer, sizeof(buffer), "%d", value);
	if (!strcmp(led_cdev->name, "red")) {
		if (value == LED_OFF) {
			gpio_direction_output(STATUS_LED->red_led, 0);
		} else {
			gpio_direction_output(STATUS_LED->red_led, 1);
		}
		sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_RED_BRIGHTNESS, buffer);
	} else if (!strcmp(led_cdev->name, "green")) {
		if (value == LED_OFF) {
			gpio_direction_output(STATUS_LED->green_led, 0);
		} else {
			gpio_direction_output(STATUS_LED->green_led, 1);
		}
		sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_GREEN_BRIGHTNESS, buffer);
	} else if (gpio_is_valid(STATUS_LED->trickle_red_led)) {
		if (value == LED_OFF) {
			gpio_direction_output(STATUS_LED->trickle_red_led, 0);
		} else {
			gpio_direction_output(STATUS_LED->trickle_red_led, 1);
		}
	}
}
static ssize_t led_blink_solid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = 0;

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "green"))
		idx = 1;
	else
		idx = 2;

	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);

	/* no lock needed for this */
	snprintf(buf, 16, "%d\n", STATUS_LED->blink_led[idx].blink_flag);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t led_blink_solid_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	unsigned long blinking;
	unsigned long state;
	ssize_t ret = -EINVAL;
	char buffer[15];

	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "green"))
		idx = 1;
	else
		idx = 2;

	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);

	if (!send_info) {
		int rc = 0;

		snprintf(buffer, sizeof(buffer), "%d,%d", STATUS_LED->red_led - STATUS_LED->gpio_base,
		STATUS_LED->green_led - STATUS_LED->gpio_base);
		rc |= sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_GPIO, buffer);

		snprintf(buffer, sizeof(buffer), "%d,%d,%d", STATUS_LED->blink_led[0].blink_flag,
		STATUS_LED->blink_led[0].blink_on_time, STATUS_LED->blink_led[0].blink_off_time);
		rc |= sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_RED_BLINK, buffer);

		snprintf(buffer, sizeof(buffer), "%d,%d,%d", STATUS_LED->blink_led[1].blink_flag,
		STATUS_LED->blink_led[1].blink_on_time, STATUS_LED->blink_led[1].blink_off_time);
		rc |= sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_GREEN_BLINK, buffer);

		if (!rc)
			send_info = 1;
		else {
			dev_info(dev, "led send info error in %s\n", __func__);
			return ret;
		}
	}
	state = kstrtoul(buf, 10, &blinking);
	if (state)
		return state;
	mutex_lock(&STATUS_LED->data_lock);
	if (blinking == 0) {
		STATUS_LED->blink_led[idx].blink_flag = 0;
		snprintf(buffer, sizeof(buffer), "%d,%d,%d", STATUS_LED->blink_led[idx].blink_flag,
		STATUS_LED->blink_led[idx].blink_on_time, STATUS_LED->blink_led[idx].blink_off_time);
		if (idx == 0)
			sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_RED_BLINK, buffer);
		else if (idx == 1)
			sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_GREEN_BLINK, buffer);
		else
			pr_debug("trickle led not blink 1\n");
	} else {
		STATUS_LED->blink_led[idx].blink_flag = 1;
		snprintf(buffer, sizeof(buffer), "%d,%d,%d", STATUS_LED->blink_led[idx].blink_flag,
		STATUS_LED->blink_led[idx].blink_on_time, STATUS_LED->blink_led[idx].blink_off_time);
		if (idx == 0)
			sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_RED_BLINK, buffer);
		else if (idx == 1)
			sysmon_send_led_cmd(SSCTL_SSR_EVENT_LEDS_SET_GREEN_BLINK, buffer);
		else
			pr_debug("trickle led not blink 2\n");
	}
	mutex_unlock(&STATUS_LED->data_lock);
	return ret;
}

static DEVICE_ATTR(blink, 0644, led_blink_solid_show, led_blink_solid_store);


static ssize_t led_blink_on_solid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = 0;

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "green"))
		idx = 1;
	else
		idx = 2;

	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);

	/* no lock needed for this */
	snprintf(buf, 16, "%d\n", STATUS_LED->blink_led[idx].blink_on_time);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t led_blink_on_solid_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	unsigned long state;
	ssize_t ret = -EINVAL;
	unsigned long blinking;

	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "green"))
		idx = 1;
	else
		idx = 2;
	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);
	state = kstrtoul(buf, 10, &blinking);
	if (state)
		return state;
	mutex_lock(&STATUS_LED->data_lock);
	STATUS_LED->blink_led[idx].blink_on_time = blinking;
	mutex_unlock(&STATUS_LED->data_lock);

	return ret;
}

static DEVICE_ATTR(blink_on, 0644, led_blink_on_solid_show, led_blink_on_solid_store);

static ssize_t led_blink_off_solid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = 0;

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "green"))
		idx = 1;
	else
		idx = 2;

	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);

	/* no lock needed for this */
	snprintf(buf, 16, "%d\n", STATUS_LED->blink_led[idx].blink_off_time);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t led_blink_off_solid_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	unsigned long state;
	ssize_t ret = -EINVAL;
	unsigned long blinking;

	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "green"))
		idx = 1;
	else
		idx = 2;

	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);

	state = kstrtoul(buf, 10, &blinking);
	if (state)
		return state;
	mutex_lock(&STATUS_LED->data_lock);
	STATUS_LED->blink_led[idx].blink_off_time = blinking;
	mutex_unlock(&STATUS_LED->data_lock);

	return ret;
}
static DEVICE_ATTR(blink_off, 0644, led_blink_off_solid_show, led_blink_off_solid_store);

static int led_parse_dt(struct platform_device *pdev)
{
	u32 temp_val;
	int rc;
	char *key;

	key = "qcom,red-led";
	rc = of_get_named_gpio(pdev->dev.of_node, key, 0);
	if (rc < 0)
		goto parse_error;
	STATUS_LED->red_led = rc;

	key = "qcom,green-led";
	rc = of_get_named_gpio(pdev->dev.of_node, key, 0);
	if (rc < 0)
		goto parse_error;
	STATUS_LED->green_led = rc;

	key = "qcom,trickle_red-led";
	rc = of_get_named_gpio(pdev->dev.of_node, key, 0);
	if (rc < 0)
		STATUS_LED->trickle_red_led = -1;
	else
		STATUS_LED->trickle_red_led = rc;

	key = "qcom,blink-on-time";
	rc = of_property_read_u32(pdev->dev.of_node, key, &temp_val);
	if (rc && (rc != -EINVAL))
		goto parse_error;
	else if (rc) {
		dev_warn(&pdev->dev, "led have no property %s, use default\n", key);
		STATUS_LED->blink_led[0].blink_on_time = 500;
		STATUS_LED->blink_led[1].blink_on_time = 500;
		STATUS_LED->blink_led[2].blink_on_time = 500;
	} else {
		STATUS_LED->blink_led[0].blink_on_time = temp_val;
		STATUS_LED->blink_led[1].blink_on_time = temp_val;
		STATUS_LED->blink_led[2].blink_on_time = temp_val;
	}

	key = "qcom,blink-off-time";
	rc = of_property_read_u32(pdev->dev.of_node, key, &temp_val);
	if (rc && (rc != -EINVAL))
		goto parse_error;
	else if (rc) {
		dev_warn(&pdev->dev, "led have no property %s, use default\n", key);
		STATUS_LED->blink_led[0].blink_off_time = 500;
		STATUS_LED->blink_led[1].blink_off_time = 500;
		STATUS_LED->blink_led[2].blink_off_time = 500;
	} else {
		STATUS_LED->blink_led[0].blink_off_time = temp_val;
		STATUS_LED->blink_led[1].blink_off_time = temp_val;
		STATUS_LED->blink_led[2].blink_off_time = temp_val;
	}

	return 0;
parse_error:
	dev_err(&pdev->dev, "parse property %s failed, rc = %d\n", key, rc);
	return rc;
}

static int led_gpio_config(struct platform_device *pdev)
{
	int rc;

	rc = gpio_request(STATUS_LED->red_led, "red_led");
	if (rc) {
		dev_err(&pdev->dev, "Failed to request gpio %d,rc = %d\n", STATUS_LED->red_led, rc);
		return rc;
	}

	rc = gpio_request(STATUS_LED->green_led, "green_led");
	if (rc) {
		dev_err(&pdev->dev, "Failed to request gpio %d,rc = %d\n", STATUS_LED->green_led, rc);
		gpio_free(STATUS_LED->red_led);
		return rc;
	}

	if (gpio_is_valid(STATUS_LED->trickle_red_led)) {
		rc = gpio_request(STATUS_LED->trickle_red_led, "trickle_red_led");
		if (rc) {
			dev_err(&pdev->dev, "Failed to request gpio %d,rc = %d\n", STATUS_LED->green_led, rc);
			gpio_free(STATUS_LED->red_led);
			return rc;
		}
		gpio_direction_output(STATUS_LED->trickle_red_led, 1);
	}
	return 0;
}

static void led_gpio_unconfig(void)
{
	if (gpio_is_valid(STATUS_LED->trickle_red_led))
		gpio_free(STATUS_LED->trickle_red_led);
	gpio_free(STATUS_LED->green_led);
	gpio_free(STATUS_LED->red_led);
}

static int gpio_led_probe(struct platform_device *pdev)
{
	int rc = 0;
	int i, j;
	struct pinctrl *pinctrl;

	if (pdev->dev.of_node == NULL) {
		dev_info(&pdev->dev, "can not find device tree node\n");
		return -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_warn(&pdev->dev, "pins are not configured from the driver\n");
		return -EINVAL;
	}

	STATUS_LED = kzalloc(sizeof(struct STATUS_LED_data), GFP_KERNEL);
	if (STATUS_LED == NULL) {
		dev_err(&pdev->dev, "no memory for device\n");
		return -ENOMEM;
	}
	pr_debug("entry gpio_led_probe\n");
	rc = led_parse_dt(pdev);
	if (rc) {
		goto err_alloc_failed;
	}

	rc = led_gpio_config(pdev);
	if (rc) {
		dev_warn(&pdev->dev, "config led gpio failed\n");
	}
	STATUS_LED->gpio_base = gpio_to_chip(STATUS_LED->red_led)->base;

	STATUS_LED->blink_led[0].led.name = "red";
	STATUS_LED->blink_led[0].led.brightness_set = gpio_led_set;
	STATUS_LED->blink_led[0].led.brightness = LED_OFF;
	STATUS_LED->blink_led[0].blink_flag = 0;

	STATUS_LED->blink_led[1].led.name = "green";
	STATUS_LED->blink_led[1].led.brightness_set = gpio_led_set;
	STATUS_LED->blink_led[1].led.brightness = LED_OFF;
	STATUS_LED->blink_led[1].blink_flag = 0;

	STATUS_LED->blink_led[2].led.name = "trickle_red";
	STATUS_LED->blink_led[2].led.brightness_set = gpio_led_set;
	STATUS_LED->blink_led[2].led.brightness = LED_OFF;
	STATUS_LED->blink_led[2].blink_flag = 0;

	mutex_init(&STATUS_LED->data_lock);

	for (i = 0; i < 3; i++) {
		if ((!strcmp(STATUS_LED->blink_led[i].led.name, "trickle_red")) &&
		(!gpio_is_valid(STATUS_LED->trickle_red_led)))
			continue;
		rc = led_classdev_register(&pdev->dev, &STATUS_LED->blink_led[i].led);
		if (rc) {
			dev_err(&pdev->dev, "STATUS_LED: led_classdev_register failed\n");
			goto err_led_classdev_register_failed;
		}
	}
	for (i = 0; i < 3; i++) {
		if ((!strcmp(STATUS_LED->blink_led[i].led.name, "trickle_red")) &&
		(!gpio_is_valid(STATUS_LED->trickle_red_led)))
			continue;
		rc = device_create_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink);
		if (rc) {
			dev_err(&pdev->dev, "STATUS_LED: create dev_attr_blink failed\n");
			goto err_out_attr_blink;
		}
		rc = device_create_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink_on);
		if (rc) {
			dev_err(&pdev->dev, "STATUS_LED: create dev_attr_blink failed\n");
			goto err_out_attr_blink;
		}
		rc = device_create_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink_off);
		if (rc) {
			dev_err(&pdev->dev, "STATUS_LED: create dev_attr_blink failed\n");
			goto err_out_attr_blink;
		}
	}

	dev_set_drvdata(&pdev->dev, STATUS_LED);

	pr_debug("PM_DEBUG_MXP:Exit gpio_led_probe.\r\n");
	return 0;

err_out_attr_blink:
	for (j = 0; j < i; j++)
		device_remove_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink);
	i = 3;
err_led_classdev_register_failed:
	for (j = 0; j < i; j++)
		led_classdev_unregister(&STATUS_LED->blink_led[i].led);
	led_gpio_unconfig();
err_alloc_failed:
	kfree(STATUS_LED);

	return rc;

}

static int gpio_led_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 3; i++) {
		device_remove_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink);
		led_classdev_unregister(&STATUS_LED->blink_led[i].led);
	}
	led_gpio_unconfig();
	kfree(STATUS_LED);
	return 0;
}

static int gpio_led_suspend(struct device *dev)
{
	STATUS_LED->led_suspend_flag = 1;
	return 0;
}

static int gpio_led_resume(struct device *dev)
{
	STATUS_LED->led_suspend_flag = 0;
	return 0;
}

static SIMPLE_DEV_PM_OPS(led_pm_ops, gpio_led_suspend, gpio_led_resume);

static struct platform_driver gpio_led_driver = {
	.probe		= gpio_led_probe,
	.remove		= gpio_led_remove,
	.driver		= {
		.name	= "zte,leds",
		.owner	= THIS_MODULE,
		.of_match_table = gpio_led_of_match,
		.pm = &led_pm_ops,
	},
};

static int __init gpio_led_init(void)
{
	return platform_driver_register(&gpio_led_driver);
}

static void __exit gpio_led_exit(void)
{
	platform_driver_unregister(&gpio_led_driver);
}

late_initcall(gpio_led_init);
module_exit(gpio_led_exit);
