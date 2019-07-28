/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#include <linux/spmi.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define PM8937_GPIO_BASE         (0xC000)
#define LED_GPIO_MODE_CTRL(base, id)	(base + 0x100 * (id - 1) + 0x40)
#define LED_GPIO_EN_CTRL(base, id)		(base + 0x100 * (id - 1) + 0x46)

#define PWM_ENABLE_ADDRESS		0xBC46
#define PWM_SYNC_ADDRESS			0xBC47
#define PWM_SEC_ADDRESS			0xBCD0
#define PWM_TO_GPIO_ADDRESS		0xBCE2
#define PWM_TO_GPIO_ADDRESS_DTEST2		0xBCE3
struct spmi_lite {
	struct spmi_device *spmi;
	u32 led_red_gpio;
	u32 led_green_gpio;
	bool led_is_dtest2;
};

struct spmi_lite *chip;

void zte_led_spmi_write(bool red_on, bool green_on, int blink)
{
	u8 value = 0;

	pr_info("zte_led_spmi_write red_on=%d, green_on=%d, blink=%d\n", red_on, green_on, blink);
	if (chip->spmi) {
		if (blink) {
			if (chip->led_is_dtest2)
				value = 0x1a;    /* enable pm gpio led to DTEST2 */
			else
				value = 0x18;    /* enable pm gpio led to DTEST1 */
			if (red_on)
				spmi_ext_register_writel(chip->spmi->ctrl, 0,
				LED_GPIO_MODE_CTRL(PM8937_GPIO_BASE, chip->led_red_gpio), &value, 1);
			if (green_on)
				spmi_ext_register_writel(chip->spmi->ctrl, 0,
				LED_GPIO_MODE_CTRL(PM8937_GPIO_BASE, chip->led_green_gpio), &value, 1);

			if (chip->led_is_dtest2) {
				value = 0xA5;   /* enable write register */
				spmi_ext_register_writel(chip->spmi->ctrl, 1, PWM_SEC_ADDRESS, &value, 1);
				value = 0x2;    /* PM gpio relation to PWM DTEST2 is 2*/
				spmi_ext_register_writel(chip->spmi->ctrl, 1, PWM_TO_GPIO_ADDRESS_DTEST2, &value, 1);
			} else {
				value = 0xA5;   /* enable write register */
				spmi_ext_register_writel(chip->spmi->ctrl, 1, PWM_SEC_ADDRESS, &value, 1);
				value = 0x1;    /* PM gpio relation to PWM*/
				spmi_ext_register_writel(chip->spmi->ctrl, 1, PWM_TO_GPIO_ADDRESS, &value, 1);
			}
			/* This is the sync register to update the PWM register */
			value = 0x1;
			spmi_ext_register_writel(chip->spmi->ctrl, 1, PWM_SYNC_ADDRESS, &value, 1);
			value = 0x80;
			spmi_ext_register_writel(chip->spmi->ctrl, 1, PWM_ENABLE_ADDRESS, &value, 1);

			if (chip->led_is_dtest2) {
				value = 0x0;
				spmi_ext_register_writel(chip->spmi->ctrl, 0, 0x7846, &value, 1);
			}
		} else {
			value = 0x10; /* set register to low */
			if (!red_on)
				spmi_ext_register_writel(chip->spmi->ctrl, 0,
				LED_GPIO_MODE_CTRL(PM8937_GPIO_BASE, chip->led_red_gpio), &value, 1); /* set low */
			if (!green_on)
				spmi_ext_register_writel(chip->spmi->ctrl, 0,
				LED_GPIO_MODE_CTRL(PM8937_GPIO_BASE, chip->led_green_gpio), &value, 1); /* set low */
			if (!red_on && !green_on)
				spmi_ext_register_writel(chip->spmi->ctrl, 1,
				PWM_ENABLE_ADDRESS, &value, 1); /*disable PWM*/
		}
		/* enable pm gpio led, if enable in dts, the led will light link in system power on */
		value = 0x80;
		spmi_ext_register_writel(chip->spmi->ctrl, 0,
			LED_GPIO_EN_CTRL(PM8937_GPIO_BASE, chip->led_red_gpio), &value, 1);
		spmi_ext_register_writel(chip->spmi->ctrl, 0,
			LED_GPIO_EN_CTRL(PM8937_GPIO_BASE, chip->led_green_gpio), &value, 1);
		pr_info("zte_led_spmi_write success\n");
	}
}


void zte_vibrator_spmi_write(int gpio)
{
	/* enable pm gpio vibrator, if enable in dts, the vibrator will on in system power on */
	u8 value = 0x80;

	if (chip->spmi) {
		spmi_ext_register_writel(chip->spmi->ctrl, 0,
		LED_GPIO_EN_CTRL(PM8937_GPIO_BASE, gpio), &value, 1);
	}
}

static int spmi_lite_probe(struct spmi_device *spmi)
{
	int rc;
	struct device_node *np;
	u32 val;

	pr_info("%s enter\n", __func__);

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->spmi = spmi;
	np = chip->spmi->dev.of_node;
	rc = of_property_read_u32(np, "zte,led_red_gpio", &val);
	if (!rc) {
		chip->led_red_gpio = val;
	} else {
		return -EINVAL;
	}
	rc = of_property_read_u32(np, "zte,led_green_gpio", &val);
	if (!rc) {
		chip->led_green_gpio = val;
	} else {
		return -EINVAL;
	}

	chip->led_is_dtest2 = of_property_read_bool(np, "zte,led_dtest2");

	pr_info("led_red_gpio:%d, led_green_gpio:%d, led_is_dtest2=%d\n",
		chip->led_red_gpio, chip->led_green_gpio, chip->led_is_dtest2);

	pr_info("%s success\n", __func__);
	return 0;
}

static int spmi_lite_remove(struct spmi_device *spmi)
{
	return 0;
}

static const struct of_device_id spmi_match_table[] = {
	{	.compatible = "zte,spmi-lite",
	},
	{}
};

static struct spmi_driver spmi_lite_driver = {
	.driver		= {
		.name	= "zte,spmi-lite",
		.of_match_table = spmi_match_table,
	},
	.probe		= spmi_lite_probe,
	.remove		= spmi_lite_remove,
};

static int __init spmi_lite_init(void)
{
	return spmi_driver_register(&spmi_lite_driver);
}
fs_initcall(spmi_lite_init);

static void __exit spmi_lite_exit(void)
{
	return spmi_driver_unregister(&spmi_lite_driver);
}
module_exit(spmi_lite_exit);

MODULE_DESCRIPTION("zte spmi lite driver");
MODULE_LICENSE("GPL v2");
