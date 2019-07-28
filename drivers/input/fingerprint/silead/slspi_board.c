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
/*#define SPI_DRV_COMPATIBLE*/
#define sileadDBG
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#define VERBOSE  0
#include <asm/uaccess.h>
#define SL_MAX_FRAME_NUM 2

#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/cdev.h>

#include "slspi.h"

int gsl_spi_open_clock_set(struct GSL_DEV_SEL_device *spi)
{
	return 0;
}

int gsl_spi_close_clock_set(struct GSL_DEV_SEL_device *spi)
{
	return 0;
}

int gsl_spi_speed_set(struct GSL_DEV_SEL_device *spi)
{
	return 0;
}

int gsl_spidev_set_before_rd_chipid(struct GSL_DEV_SEL_device *spi)
{
	int rv = 0;
#ifndef SLPT
/*reset spi mode */
	if (spi->max_speed_hz > SPI_SPEED)
		spi->max_speed_hz = SPI_SPEED;
	spi->bits_per_word = SPI_BITS;
	spi->mode = SPI_MODE_0;
	rv = spi_setup(spi);
	if (rv < 0) {
		dev_info(&spi->dev, "spi master set mode failed");
		return rv;
	}
#endif
	return rv;
}

int gsl_spidev_set_after_rd_chipid(struct GSL_DEV_SEL_device *spi)
{
	int rv = 0;
	return rv;
}

long gsl_fp_pinctrl_init(struct spidev_data *spidev)
{
	long ret = 0;

	spidev->fp_pinctrl = devm_pinctrl_get(&spidev->spi->dev);
	if (IS_ERR(spidev->fp_pinctrl)) {
		SL_LOGE("devm_pinctrl_get failed!");
		ret = PTR_ERR(spidev->fp_pinctrl);
		goto pinctrl_get_fail;
	}
	spidev->pinctrl_state_pwractive = pinctrl_lookup_state(spidev->fp_pinctrl, GSL_FP_PWR_ACTIVE);
	if (IS_ERR(spidev->pinctrl_state_pwractive)) {
		SL_LOGE("pinctrl_lookup_state reset_active failed!");
		ret = PTR_ERR(spidev->pinctrl_state_pwractive);
		goto lookup_state_fail;
	}
	spidev->pinctrl_state_active = pinctrl_lookup_state(spidev->fp_pinctrl, GSL_FP_RESET_ACTIVE);
	if (IS_ERR(spidev->pinctrl_state_active)) {
		SL_LOGE("pinctrl_lookup_state reset_active failed!");
		ret = PTR_ERR(spidev->pinctrl_state_active);
		goto lookup_state_fail;
	}
	spidev->pinctrl_state_suspend = pinctrl_lookup_state(spidev->fp_pinctrl, GSL_FP_RESET_DEACTIVE);
	if (IS_ERR(spidev->pinctrl_state_suspend)) {
		SL_LOGE("pinctrl_lookup_state reset_deactive failed!");
		ret = PTR_ERR(spidev->pinctrl_state_suspend);
		goto lookup_state_fail;
	}
#if defined(ZTE_CFG_IRQ_PIN_NP)
	spidev->pinctrl_state_interrupt = pinctrl_lookup_state(spidev->fp_pinctrl, GSL_FP_IRQ_NP);
	if (IS_ERR(spidev->pinctrl_state_interrupt)) {
		SL_LOGE("pinctrl_lookup_state GSL_FP_IRQ_NP failed!");
		ret = PTR_ERR(spidev->pinctrl_state_interrupt);
		goto lookup_state_fail;
	}
#else
	spidev->pinctrl_state_interrupt = pinctrl_lookup_state(spidev->fp_pinctrl, GSL_FP_IRQ_ACTIVE);
	if (IS_ERR(spidev->pinctrl_state_interrupt)) {
		SL_LOGE("pinctrl_lookup_state irq_active failed!");
		ret = PTR_ERR(spidev->pinctrl_state_interrupt);
		goto lookup_state_fail;
	}
#endif
	spidev->pinctrl_state_finger_id = pinctrl_lookup_state(spidev->fp_pinctrl, GSL_FP_ID_NP);
	if (!IS_ERR(spidev->pinctrl_state_finger_id)) {
		SL_LOGD("pinctrl_lookup_state GSL_FP_ID_NP SUCCESS!");
		ret = pinctrl_select_state(spidev->fp_pinctrl, spidev->pinctrl_state_finger_id);
		if (ret) {
			dev_err(&spidev->spi->dev, "[silead]set fingerid np failed!!\n");
			goto lookup_state_fail;
		}
	}
	ret = pinctrl_select_state(spidev->fp_pinctrl, spidev->pinctrl_state_pwractive);
	if (ret) {
		dev_err(&spidev->spi->dev, "[silead]power on failed!! pinctrl_state_pwractive state\n");
		goto lookup_state_fail;
	}
	return 0;
lookup_state_fail:
	devm_pinctrl_put(spidev->fp_pinctrl);
pinctrl_get_fail:
	spidev->fp_pinctrl = NULL;
	return ret;
}

int spidev_reset_hw(struct spidev_data *spidev)
{
	int rc = 0;
#ifdef sileadDBG
	int value = 2;
#endif
	if (spidev->fp_pinctrl) {
		rc = pinctrl_select_state(spidev->fp_pinctrl, spidev->pinctrl_state_suspend);
		if (rc)
			dev_err(&spidev->spi->dev, "[silead]cannot get suspend pinctrl state\n");
	}
	msleep(1);
#ifdef sileadDBG
	value = gpio_get_value_cansleep(spidev->hw_reset_gpio);
	SL_LOGD("[%s] GPIO %d state is %d\n", __func__, spidev->hw_reset_gpio, value);
#endif
	if (spidev->fp_pinctrl) {
		rc = pinctrl_select_state(spidev->fp_pinctrl, spidev->pinctrl_state_active);
		if (rc)
			dev_err(&spidev->spi->dev, "[silead]cannot get active pinctrl state\n");
	}
#ifdef sileadDBG
	value = gpio_get_value_cansleep(spidev->hw_reset_gpio);
	SL_LOGD("[%s] GPIO %d state is %d\n", __func__, spidev->hw_reset_gpio, value);
#endif
	return rc;
}

int spidev_shutdown_hw(struct spidev_data *spidev)
{
	int rc = 0;
#ifdef sileadDBG
	int value = 2;
#endif
	if (spidev->fp_pinctrl) {
		rc = pinctrl_select_state(spidev->fp_pinctrl, spidev->pinctrl_state_suspend);
		if (rc)
			dev_err(&spidev->spi->dev, "[silead]cannot get suspend pinctrl state\n");
	}
#ifdef sileadDBG
	value = gpio_get_value_cansleep(spidev->hw_reset_gpio);
	SL_LOGD("[%s] GPIO %d state is %d\n", __func__, spidev->hw_reset_gpio, value);
#endif
	/* 20150817 silead thomas end */
	return rc;
}

int silead_init_eint(struct spidev_data *spidev)
{
	int irq, debounce = 0;

	irq = gpio_to_irq(spidev->hw_int_gpio);
	gpio_set_debounce(spidev->hw_int_gpio, debounce);
	spidev->irq = irq;
	return 0;
}

int silead_fp_parse_reset_and_int_gpios(struct spidev_data *spidev)
{
	spidev->hw_reset_gpio = of_get_named_gpio(spidev->spi->dev.of_node, "silead,gpio_reset", 0);
	SL_LOGD("spidev->hw_reset_gpio= %d", spidev->hw_reset_gpio);
	spidev->hw_int_gpio = of_get_named_gpio(spidev->spi->dev.of_node, "silead,gpio_irq", 0);
	SL_LOGD("spidev->hw_int_gpio= %d", spidev->hw_int_gpio);
	return 0;
}

int silead_fp_init_gpio_states(struct spidev_data *spidev)
{
	int status = 0;

	spidev_reset_hw(spidev);
	SL_LOGD("reset gpio successful!");
	status = pinctrl_select_state(spidev->fp_pinctrl, spidev->pinctrl_state_interrupt);
	if (status) {
		SL_LOGE("pinctrl_select_state pinctrl_state_interrupt failed!");
		return status;
	}
	SL_LOGD("int gpio select status successful!");
	return status;
}

void silead_register_board_info(void)
{
}
