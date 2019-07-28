/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#define FP_VDD_MIN_UV	2850000
#define FP_VDD_MAX_UV	3300000
static int AVDD_FP_power_init(struct gf_dev *gf_dev)
{
	int rc;

	pr_info("%s, %d ", __func__, __LINE__);

	gf_dev->avdd = regulator_get(&gf_dev->spi->dev, "avdd");
	if (IS_ERR(gf_dev->avdd)) {
		rc = PTR_ERR(gf_dev->avdd);
		dev_err(&gf_dev->spi->dev, "Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}
	if (regulator_count_voltages(gf_dev->avdd) > 0) {
		rc = regulator_set_voltage(gf_dev->avdd, FP_VDD_MIN_UV,
						   FP_VDD_MAX_UV);
		if (rc) {
			dev_err(&gf_dev->spi->dev, "Regulator set failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	pr_info("%s, %d  end", __func__, __LINE__);

	return 0;
reg_vdd_put:
	regulator_put(gf_dev->avdd);
	return rc;
}

static int AVDD_FP_power_on(struct gf_dev *gf_dev, bool on)
{
	int rc = 0;

	pr_info("%s, %d, on==%d ", __func__, __LINE__, on);
	if (!on)
		goto power_off;
	rc = regulator_enable(gf_dev->avdd);
	if (rc) {
			dev_err(&gf_dev->spi->dev, "Regulator vdd enable failed rc=%d\n", rc);
			goto fp_power_done;
		}
fp_power_done:
	return rc;
power_off:
	rc = regulator_disable(gf_dev->avdd);
	if (rc) {
			dev_err(&gf_dev->spi->dev, "Regulator vdd disable failed rc=%d\n", rc);
			goto fp_power_done;
		}
	goto fp_power_done;
}

int get_pwr_and_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

	gf_dev->pwr_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_pwr", 0);
	pr_err("gf::pwr_gpio:%d\n", gf_dev->pwr_gpio);
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		pr_err("PWR GPIO is valid.\n");
		rc = gpio_request(gf_dev->pwr_gpio, "goodix_pwr");
		if (rc) {
			dev_err(&gf_dev->spi->dev, "Failed to request PWR GPIO. rc = %d\n", rc);
			return rc;
		}
		gpio_direction_output(gf_dev->pwr_gpio, 1);
		return rc;
	}
	pr_err("gf: config regulator\n");
	rc = AVDD_FP_power_init(gf_dev);
	rc = AVDD_FP_power_on(gf_dev, true);
	return rc;

}

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	int err_rc = -1;
#if defined(ZTE_CFG_IRQ_PIN_NP)
	struct pinctrl_state *irq_pd_state = NULL;
	struct pinctrl_state *irq_np_state = NULL;
#endif

	/*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "fp-gpio-reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_info("RESET GPIO is invalid.\n");
		return err_rc;
	}

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request RESET GPIO. rc = %d\n", rc);
		return err_rc;
	}

	gpio_direction_output(gf_dev->reset_gpio, 1);

	/*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "fp-gpio-irq", 0);
	pr_info("gf::irq_gpio:%d\n", gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_info("IRQ GPIO is invalid.\n");
		return err_rc;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", rc);
		return err_rc;
	}
	gpio_direction_input(gf_dev->irq_gpio);
#if defined(ZTE_CFG_IRQ_PIN_NP)
	/* set goodix_irq status pull down */
	gf_dev->goodix_pinctrl = devm_pinctrl_get(&(gf_dev->spi->dev));
	if (IS_ERR(gf_dev->goodix_pinctrl)) {
		if (PTR_ERR(gf_dev->goodix_pinctrl) == -EPROBE_DEFER) {
			dev_err(&gf_dev->spi->dev, "pinctrl not ready\n");
			return -EPROBE_DEFER;
		}
		dev_err(&gf_dev->spi->dev, "Target does not use pinctrl\n");
		gf_dev->goodix_pinctrl = NULL;
		return -EINVAL;
	}
	irq_pd_state = pinctrl_lookup_state(gf_dev->goodix_pinctrl, "goodix_irq_pd");
	if (IS_ERR(irq_pd_state)) {
		dev_err(&gf_dev->spi->dev, "cannot find goodix_irq_pd\n");
		return -EINVAL;
	}
	rc = pinctrl_select_state(gf_dev->goodix_pinctrl, irq_pd_state);
	if (rc)
		dev_err(&gf_dev->spi->dev, "cannot select goodix_irq_pd\n");
	else
		dev_info(&gf_dev->spi->dev, "select goodix_irq_pd success\n");
	/* set goodix_irq status end */
#endif

/*get pwr resourece*/
	get_pwr_and_power_on(gf_dev);
	/*
	gf_dev->pwr_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_pwr", 0);
	pr_info("gf::pwr_gpio:%d\n", gf_dev->pwr_gpio);
	if (!gpio_is_valid(gf_dev->pwr_gpio)) {
		pr_err("PWR GPIO is invalid.\n");
		return err_rc;
	}

	rc = gpio_request(gf_dev->pwr_gpio, "goodix_pwr");
	if (rc) {
		dev_err(&gf_dev->spi->dev, "Failed to request PWR GPIO. rc = %d\n", rc);
		return err_rc;
	}

	gpio_direction_output(gf_dev->pwr_gpio, 1);
	*//*power on*/
#if defined(ZTE_CFG_IRQ_PIN_NP)
	/* set goodix_irq status no pull */
	irq_np_state = pinctrl_lookup_state(gf_dev->goodix_pinctrl, "goodix_irq_np");
	if (IS_ERR(irq_np_state)) {
		dev_err(&gf_dev->spi->dev, "cannot find goodix_irq_np\n");
		return -EINVAL;
	}
	rc = pinctrl_select_state(gf_dev->goodix_pinctrl, irq_np_state);
	if (rc) {
		dev_err(&gf_dev->spi->dev, "cannot select goodix_irq_np\n");
		return -EINVAL;
	}
	dev_info(&gf_dev->spi->dev, "goodix_irq_np set success\n");
	/* set goodix_irq status end */
#endif

	return 0;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_free(gf_dev->pwr_gpio);
		pr_info("remove pwr_gpio success\n");
	} else {
		regulator_put(gf_dev->avdd);
		pr_info("regulator_put success\n");
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_set_value(gf_dev->pwr_gpio, 1);
	} else {
		rc = AVDD_FP_power_on(gf_dev, true);
	}

	msleep(20);
	pr_info("---- power on ok ----\n");

	return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_set_value(gf_dev->pwr_gpio, 0);
	} else {
		rc = AVDD_FP_power_on(gf_dev, false);
	}
	pr_info("---- power off ----\n");
	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	int err_rc = -1;

	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return err_rc;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	int err_rc = -1;

	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return err_rc;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

