
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "zte_eeprom.h"

static struct zte_eeprom_fn_t common_eeprom_func_tbl = {
	.eeprom_parse_map = common_eeprom_parse_memory_map,
	.kernel_read_eeprom_memory = common_kernel_read_eeprom_memory,
	.user_read_eeprom_memory = common_user_read_eeprom_memory,
	.eeprom_match_crc = msm_eeprom_match_crc,
	.eeprom_checksum = NULL,
	.validflag_check_eeprom = NULL,
	.parse_module_name = NULL,
};

static const struct of_device_id common_eeprom_dt_match[] = {
	{ .compatible = "zte,common-eeprom", .data = (void *)(&common_eeprom_func_tbl)},
	{}
};
MODULE_DEVICE_TABLE(of, common_eeprom_dt_match);

static int common_eeprom_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(common_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_probe_user(pdev, match->data);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);

	return rc;
}

static int common_eeprom_platform_remove(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int32_t rc = 0;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(common_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_remove(pdev);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static struct platform_driver common_eeprom_platform_driver = {
	.driver = {
		.name = "zte,common-eeprom",
		.owner = THIS_MODULE,
		.of_match_table = common_eeprom_dt_match,
	},
	.probe = common_eeprom_platform_probe,
	.remove = common_eeprom_platform_remove,
};

static int __init common_eeprom_init_module(void)
{
	int rc = 0;

	pr_info("%s : %dE\n", __func__, __LINE__);
	rc = platform_driver_register(&common_eeprom_platform_driver);
	pr_info("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static void __exit common_eeprom_exit_module(void)
{
	platform_driver_unregister(&common_eeprom_platform_driver);

}

module_init(common_eeprom_init_module);
module_exit(common_eeprom_exit_module);
MODULE_DESCRIPTION("ZTE EEPROM driver");
MODULE_LICENSE("GPL v2");

