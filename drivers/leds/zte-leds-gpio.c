/*
 * LEDs driver for GPIOs
 *
 * Copyright (C) 2007 8D Technologies inc.
 * Raphael Assenat <raph@8d.com>
 * Copyright (C) 2008 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/qpnp/pwm.h>



#define LED_GPIO_VIN_CTRL_DEFAULT	0
#define LED_GPIO_SOURCE_SEL_DEFAULT	0x1

#define LED_GPIO_MODE_SINK		(0x06 << 4)
#define PWM_MODE 0

struct zte_pwm_config_data {
	struct pwm_device	*pwm_dev;
	u32			pwm_period_us;
	struct pwm_duty_cycles	*duty_cycles;
	int	*old_duty_pcts;
	u8	mode;
	u8	default_mode;
	bool	pwm_enabled;
	bool use_blink;
	bool blinking;
};

struct zte_gpio_config_data {
	struct zte_pwm_config_data	*pwm_cfg;
	u8	current_setting;
	u8	source_sel;
	u8	mode_ctrl;
	u8	vin_ctrl;
	u8	min_brightness;
	u8	pwm_mode;
	bool	enable;
};

struct zte_gpio_led_data {
	struct led_classdev cdev;
	struct zte_gpio_config_data *gpio_cfg;
	unsigned gpio;
	struct work_struct work;
	struct mutex lock;
	u8 new_level;
	u8 can_sleep;
	u8 active_low;
	u8 blinking;
	int (*platform_gpio_blink_set)(unsigned gpio, int state,
			unsigned long *delay_on, unsigned long *delay_off);
	int			blinkoff_delayms_cust;
	int			blinkon_delayms_cust;
	bool			is_zte_cust;
	bool	is_operator_sprint;
};

extern void zte_led_spmi_write(bool red_on, bool green_on, int blink);

static bool red_led_on	= false;
static bool green_led_on = false;

static void update_led_state(struct zte_gpio_led_data *led)
{
	if (!strcmp(led->cdev.name, "red")) {
		if (led->cdev.brightness)
			red_led_on = true;
		else
			red_led_on = false;
		return;
	}

	if (!strcmp(led->cdev.name, "green")) {
		if (led->cdev.brightness)
			green_led_on = true;
		else
			green_led_on = false;
	}
	pr_info("LedState:red=%d green=%d\n", red_led_on, green_led_on);
}

static void gpio_led_work(struct work_struct *work)
{
	struct zte_gpio_led_data *led_dat =
		container_of(work, struct zte_gpio_led_data, work);

	if (led_dat->blinking) {
		led_dat->platform_gpio_blink_set(led_dat->gpio,
						 led_dat->new_level,
						 NULL, NULL);
		led_dat->blinking = 0;
	} else
		gpio_set_value_cansleep(led_dat->gpio, led_dat->new_level);
}

static void gpio_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct zte_gpio_led_data *led_dat =
		container_of(led_cdev, struct zte_gpio_led_data, cdev);
	int level;
	int rc;

	pr_info("gpio_led_set name = %s,value = %d can_sleep =%d\n",
		led_cdev->name, value, led_dat->can_sleep);
	if (value == LED_OFF)
		level = 0;
	else
		level = 1;

	led_dat->cdev.brightness = value;
	update_led_state(led_dat);

	if (led_dat->active_low)
		level = !level;

	/* Setting GPIOs with I2C/etc requires a task context, and we don't
	 * seem to have a reliable way to know if we're already in one; so
	 * let's just assume the worst.
	 */
	if (led_dat->can_sleep) {
		led_dat->new_level = level;
		schedule_work(&led_dat->work);
	} else {
		if (led_dat->blinking) {
			led_dat->platform_gpio_blink_set(led_dat->gpio, level,
							 NULL, NULL);
			led_dat->blinking = 0;
		} else {
			gpio_set_value(led_dat->gpio, level);
			if (led_dat->gpio_cfg && led_dat->gpio_cfg->pwm_cfg) {
				if (led_dat->gpio_cfg->pwm_cfg->blinking) {
				if (led_dat->is_zte_cust) {
					if (led_dat->gpio_cfg->pwm_cfg->pwm_dev) {
					/* for zte cust blink on and off us*/
					rc = pwm_config_us(led_dat->gpio_cfg->pwm_cfg->pwm_dev,
					(int)((led_dat->blinkon_delayms_cust/10)*100000),
					(int)((led_dat->blinkoff_delayms_cust/10)*1000000));
					pr_err("LEDLOG:gpio_led_set blink  is_zte_cust on = %d,off =%d\n",
					(int)((led_dat->blinkon_delayms_cust/10)*100000),
					(int)((led_dat->blinkoff_delayms_cust/10)*1000000));
					}
				} else {
					if (led_dat->is_operator_sprint) {
							/* for SPRINT,blink 0.5s per 7.5s */
							if (led_dat->gpio_cfg->pwm_cfg->pwm_dev)
								rc = pwm_config_us(led_dat->gpio_cfg->pwm_cfg->pwm_dev,
									0.5*1000000, 7.5*1000000);

					} else {
							if (led_dat->gpio_cfg->pwm_cfg->pwm_dev)
							/* for others,blink 0.5s per 3s */
								rc = pwm_config_us(led_dat->gpio_cfg->pwm_cfg->pwm_dev,
									0.5*1000000, 3*1000000);
					}
				}
					if (rc < 0)
						pr_info("Failed to configure pwm for new values\n");
					pr_info("LEDLOG: set %s, br=%d, blink=%d\n", led_dat->cdev.name,
						led_dat->cdev.brightness, led_dat->gpio_cfg->pwm_cfg->blinking);
				}
				zte_led_spmi_write(red_led_on, green_led_on, led_dat->gpio_cfg->pwm_cfg->blinking);
			}
		}
	}
}

static int gpio_blink_set(struct led_classdev *led_cdev,
	unsigned long *delay_on, unsigned long *delay_off)
{
	struct zte_gpio_led_data *led_dat =
		container_of(led_cdev, struct zte_gpio_led_data, cdev);

	led_dat->blinking = 1;
	return led_dat->platform_gpio_blink_set(led_dat->gpio, GPIO_LED_BLINK,
						delay_on, delay_off);
}

static int create_gpio_led(const struct gpio_led *template,
	struct zte_gpio_led_data *led_dat, struct device *parent,
	int (*blink_set)(unsigned, int, unsigned long *, unsigned long *))
{
	int ret, state;

	led_dat->gpio = -1;

	gpio_free(template->gpio);
	ret = gpio_request(template->gpio, template->name);
	pr_info("create_gpio_led ret=%d\n", ret);
	if (ret < 0)
		return ret;

	led_dat->cdev.name = template->name;
	led_dat->cdev.default_trigger = template->default_trigger;
	led_dat->gpio = template->gpio;
	led_dat->can_sleep = gpio_cansleep(template->gpio);
	led_dat->active_low = template->active_low;
	led_dat->blinking = 0;
	if (blink_set) {
		led_dat->platform_gpio_blink_set = blink_set;
		led_dat->cdev.blink_set = gpio_blink_set;
	}
	led_dat->cdev.brightness_set = gpio_led_set;
	if (template->default_state == LEDS_GPIO_DEFSTATE_KEEP)
		state = !!gpio_get_value_cansleep(led_dat->gpio) ^ led_dat->active_low;
	else
		state = (template->default_state == LEDS_GPIO_DEFSTATE_ON);
	led_dat->cdev.brightness = state ? LED_FULL : LED_OFF;
	led_dat->cdev.max_brightness = LED_FULL;
	if (!template->retain_state_suspended)
		led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;

	ret = gpio_direction_output(led_dat->gpio, led_dat->active_low ^ state);
	if (ret < 0)
		return ret;

	INIT_WORK(&led_dat->work, gpio_led_work);

	ret = led_classdev_register(parent, &led_dat->cdev);
	if (ret < 0)
		return ret;

	return 0;
}

static void delete_gpio_led(struct zte_gpio_led_data *led)
{
	if (!gpio_is_valid(led->gpio))
		return;
	led_classdev_unregister(&led->cdev);
	cancel_work_sync(&led->work);
}


static void led_blink(struct zte_gpio_led_data *led,
			struct zte_pwm_config_data *pwm_cfg)
{
	flush_work(&led->work);
	mutex_lock(&led->lock);
	if (pwm_cfg->use_blink) {
		if (led->cdev.brightness)
			pwm_cfg->blinking = true;
		else
			pwm_cfg->blinking = false;
		/*zte led add only for blink_show() selftest*/
		led->cdev.blink_value = pwm_cfg->blinking;
		pr_info("LEDLOG: blink set, %s, br=%d  blink=%d\n",
		led->cdev.name, led->cdev.brightness, pwm_cfg->blinking);	/*ZTE LOG*/
		if (pwm_cfg->pwm_dev)
			pwm_free(pwm_cfg->pwm_dev);
		/*to test led blink: write blink first and then write brightness*/
		led->cdev.brightness = 0;

	}
	mutex_unlock(&led->lock);
}

static ssize_t blink_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct zte_gpio_led_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	led = container_of(led_cdev, struct zte_gpio_led_data, cdev);
	return snprintf(buf, 10, "%d\n", led->cdev.blink_value);
}

static ssize_t blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct zte_gpio_led_data *led;
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct zte_gpio_led_data, cdev);
	led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;
	if (led->gpio_cfg && led->gpio_cfg->pwm_cfg)
		led_blink(led, led->gpio_cfg->pwm_cfg);
	pr_info("LEDLOG: blink_store:%s br=%d blink=%d\n",
	led->cdev.name, led->cdev.brightness, led->cdev.blink_value);

	return count;
}

static DEVICE_ATTR(blink, 0664, blink_show, blink_store); /*enable read blink*/

static struct attribute *blink_attrs[] = {
	&dev_attr_blink.attr,
	NULL
};

static const struct attribute_group blink_attr_group = {
	.attrs = blink_attrs,
};

static int qpnp_get_config_pwm(struct zte_pwm_config_data *pwm_cfg,
				struct platform_device *pdev,
				struct device_node *node)
{
	int rc;
	u32 val;

	pwm_cfg->pwm_dev = of_pwm_get(node, NULL);
	if (IS_ERR(pwm_cfg->pwm_dev)) {
		rc = PTR_ERR(pwm_cfg->pwm_dev);
		dev_err(&pdev->dev, "Cannot get PWM device rc:(%d)\n", rc);
		pwm_cfg->pwm_dev = NULL;
	}

	rc = of_property_read_u32(node, "qcom,pwm-us", &val);
	if (!rc)
		pwm_cfg->pwm_period_us = val;
	else
		return rc;

	pwm_cfg->use_blink =
		of_property_read_bool(node, "qcom,use-blink");

	pr_info("qpnp_get_config_pwm pwm_period_us=%d, use_blink=%d\n",
		pwm_cfg->pwm_period_us,	 pwm_cfg->use_blink);
	return 0;
}

static int qpnp_get_config_gpio(struct zte_gpio_led_data *led,
	struct platform_device *pdev, struct device_node *node)
{
	int rc;
	u32 val;
	u8 led_mode;
	const char *mode;
	const char *temp_string;

	pr_info("qpnp_get_config_gpio++++++\n");
	led->gpio_cfg = devm_kzalloc(&pdev->dev,
			sizeof(struct zte_gpio_config_data), GFP_KERNEL);
	if (!led->gpio_cfg) {
		dev_err(&pdev->dev, "Unable to allocate memory gpio struct\n");
		return -ENOMEM;
	}

	led->gpio_cfg->source_sel = LED_GPIO_SOURCE_SEL_DEFAULT;
	rc = of_property_read_u32(node, "qcom,source-sel", &val);
	if (!rc)
		led->gpio_cfg->source_sel = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_gpio;

	led->gpio_cfg->mode_ctrl = LED_GPIO_MODE_SINK;
	rc = of_property_read_u32(node, "qcom,mode-ctrl", &val);
	if (!rc)
		led->gpio_cfg->mode_ctrl = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_gpio;

	led->gpio_cfg->vin_ctrl = LED_GPIO_VIN_CTRL_DEFAULT;
	rc = of_property_read_u32(node, "qcom,vin-ctrl", &val);
	if (!rc)
		led->gpio_cfg->vin_ctrl = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_gpio;

	led->gpio_cfg->min_brightness = 0;
	rc = of_property_read_u32(node, "qcom,min-brightness", &val);
	if (!rc)
		led->gpio_cfg->min_brightness = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_gpio;

	rc = of_property_read_string(node, "qcom,mode", &mode);
	if (!rc) {
		led_mode = PWM_MODE;
		led->gpio_cfg->pwm_mode = led_mode;
		led->gpio_cfg->pwm_cfg = devm_kzalloc(&pdev->dev,
					sizeof(struct zte_pwm_config_data),
					GFP_KERNEL);
		if (!led->gpio_cfg->pwm_cfg) {
			dev_err(&pdev->dev,
				"Unable to allocate memory\n");
			rc = -ENOMEM;
			goto err_config_gpio;
		}
		led->gpio_cfg->pwm_cfg->mode = led_mode;
		led->gpio_cfg->pwm_cfg->default_mode = led_mode;
	} else
		goto err_config_gpio;

	led->is_operator_sprint = false;
	rc = of_property_read_string(node, "zte,is-operator-sprint",
		&temp_string);
	if (!rc) {
		if (strncmp(temp_string, "yes", sizeof("yes")) == 0)
			led->is_operator_sprint = true;
	} else if (rc != -EINVAL)
		goto err_conifg_gpio_pwm;

	led->blinkoff_delayms_cust = 0;
	rc = of_property_read_u32(node, "zte,blinkoff_delayms_cust", &val);
	if (!rc)
		led->blinkoff_delayms_cust = val;
	else if (rc != -EINVAL)
		return rc;
	led->blinkon_delayms_cust = 0;
	rc = of_property_read_u32(node, "zte,blinkon_delayms_cust", &val);
	if (!rc)
		led->blinkon_delayms_cust = val;
	else if (rc != -EINVAL)
		return rc;
	led->is_zte_cust = false;
	rc = of_property_read_string(node, "zte,is_zte_cust",
		&temp_string);
	if (!rc) {
		if (strncmp(temp_string, "yes", sizeof("yes")) == 0)
			led->is_zte_cust = true;
	} else if (rc != -EINVAL)
		return rc;

	rc = qpnp_get_config_pwm(led->gpio_cfg->pwm_cfg, pdev, node);
	if (rc < 0)
		goto err_conifg_gpio_pwm;

	if (led->gpio_cfg->pwm_cfg->use_blink) {
		rc = sysfs_create_group(&led->cdev.dev->kobj,
			&blink_attr_group);
		if (rc)
			dev_err(&pdev->dev, "Cannot register blink device rc:(%d)\n", rc);

	}
	pr_info("qpnp_get_config_gpio source_sel = %d, mode_ctrl=%d, vin_ctrl=%d\n",
		led->gpio_cfg->source_sel,	led->gpio_cfg->mode_ctrl, led->gpio_cfg->vin_ctrl);
	return 0;

err_conifg_gpio_pwm:
	devm_kfree(&pdev->dev, led->gpio_cfg->pwm_cfg);
err_config_gpio:
	devm_kfree(&pdev->dev, led->gpio_cfg);
	return rc;
}

struct zte_gpio_leds_priv {
	int num_leds;
	struct zte_gpio_led_data leds[];
};

static inline int sizeof_zte_gpio_leds_priv(int num_leds)
{
	return sizeof(struct zte_gpio_leds_priv) +
		(sizeof(struct zte_gpio_led_data) * num_leds);
}

/* Code to create from OpenFirmware platform devices */
#ifdef CONFIG_OF_GPIO
static struct zte_gpio_leds_priv *gpio_leds_create_of(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *child;
	struct zte_gpio_leds_priv *priv;
	int count, ret;

	/* count LEDs in this device, so we know how much to allocate */
	count = of_get_available_child_count(np);
	if (!count)
		return ERR_PTR(-ENODEV);

	for_each_available_child_of_node(np, child)
		if (of_get_gpio(child, 0) == -EPROBE_DEFER)
			return ERR_PTR(-EPROBE_DEFER);

	priv = devm_kzalloc(&pdev->dev, sizeof_zte_gpio_leds_priv(count),
			GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);
	priv->num_leds = 0;

	for_each_available_child_of_node(np, child) {
		struct gpio_led led = {};
		enum of_gpio_flags flags;
		const char *state;

		led.active_low = flags & 0;
		led.name = of_get_property(child, "label", NULL) ? : child->name;
		led.gpio = of_get_named_gpio(child, led.name, 0);
		pr_info("led.gpio=%d led.name=%s\n", led.gpio, led.name);
		led.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
		state = of_get_property(child, "default-state", NULL);
		if (state) {
			if (!strcmp(state, "keep"))
				led.default_state = LEDS_GPIO_DEFSTATE_KEEP;
			else if (!strcmp(state, "on"))
				led.default_state = LEDS_GPIO_DEFSTATE_ON;
			else
				led.default_state = LEDS_GPIO_DEFSTATE_OFF;
		}

		if (of_get_property(child, "retain-state-suspended", NULL))
			led.retain_state_suspended = 1;

		ret = create_gpio_led(&led, &priv->leds[priv->num_leds],
				      &pdev->dev, NULL);
		if (ret < 0) {
			of_node_put(child);
			goto err;
		}
		ret = qpnp_get_config_gpio(&priv->leds[priv->num_leds], pdev, child);
		if (ret < 0) {
			goto err;
		}
		mutex_init(&priv->leds[priv->num_leds].lock);
		priv->num_leds++;
	}

	return priv;

err:
	pr_info("priv->num_leds=%d\n", priv->num_leds);
	for (count = priv->num_leds; count >= 0; count--)
		delete_gpio_led(&priv->leds[count]);
	devm_kfree(&pdev->dev, priv);
	return ERR_PTR(-ENODEV);
}

static const struct of_device_id of_gpio_leds_match[] = {
	{ .compatible = "zte-gpio-leds", },
	{},
};

MODULE_DEVICE_TABLE(of, of_gpio_leds_match);
#else /* CONFIG_OF_GPIO */
static struct zte_gpio_leds_priv *gpio_leds_create_of(struct platform_device *pdev)
{
	return ERR_PTR(-ENODEV);
}
#endif /* CONFIG_OF_GPIO */

static int gpio_led_probe(struct platform_device *pdev)
{
	struct gpio_led_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct zte_gpio_leds_priv *priv;
	int i, ret = 0;

	if (pdata && pdata->num_leds) {
		priv = devm_kzalloc(&pdev->dev,
				sizeof_zte_gpio_leds_priv(pdata->num_leds),
					GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		priv->num_leds = pdata->num_leds;
		for (i = 0; i < priv->num_leds; i++) {
			ret = create_gpio_led(&pdata->leds[i],
					      &priv->leds[i],
					      &pdev->dev, pdata->gpio_blink_set);
			if (ret < 0) {
				/* On failure: unwind the led creations */
				for (i = i - 1; i >= 0; i--)
					delete_gpio_led(&priv->leds[i]);
				devm_kfree(&pdev->dev, priv);
				return ret;
			}
		}
	} else {
		priv = gpio_leds_create_of(pdev);
		if (IS_ERR(priv))
			return PTR_ERR(priv);
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int gpio_led_remove(struct platform_device *pdev)
{
	struct zte_gpio_leds_priv *priv = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < priv->num_leds; i++)
		delete_gpio_led(&priv->leds[i]);

	return 0;
}

static struct platform_driver gpio_led_driver = {
	.probe		= gpio_led_probe,
	.remove		= gpio_led_remove,
	.driver		= {
		.name	= "zte-leds-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_gpio_leds_match),
	},
};

module_platform_driver(gpio_led_driver);

MODULE_AUTHOR("Raphael Assenat <raph@8d.com>, Trent Piepho <tpiepho@freescale.com>");
MODULE_DESCRIPTION("ZTE GPIO LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-gpio");
