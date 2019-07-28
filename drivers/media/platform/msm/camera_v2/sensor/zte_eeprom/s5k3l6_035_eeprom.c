

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "zte_eeprom.h"

#define S5K3L6_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define S5K3L6_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define S5K3L6_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define S5K3L6_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define S5K3L6_SENSOR_INFO_MODULE_ID_DARLING	0x05
#define S5K3L6_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define S5K3L6_SENSOR_INFO_MODULE_ID_OFLIM		0x07
#define S5K3L6_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define S5K3L6_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define S5K3L6_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define S5K3L6_SENSOR_INFO_MODULE_ID_ABICO		0x14
#define S5K3L6_SENSOR_INFO_MODULE_ID_LITE_ON	0x15
#define S5K3L6_SENSOR_INFO_MODULE_ID_CHICONY	0x16
#define S5K3L6_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define S5K3L6_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define S5K3L6_SENSOR_INFO_MODULE_ID_MCNEX		0x31
#define S5K3L6_SENSOR_INFO_MODULE_ID_HOLITECH	0x42
#define S5K3L6_SENSOR_INFO_MODULE_ID_GOERTEK	0x54
#define S5K3L6_SENSOR_INFO_MODULE_ID_SHINETECH	0x55
#define S5K3L6_SENSOR_INFO_MODULE_ID_SUNWIN	0x56
#define S5K3L6_SENSOR_INFO_MODULE_ID_JSL			0x57
#define S5K3L6_SENSOR_INFO_MODULE_ID_UNION		0x58
#define S5K3L6_SENSOR_INFO_MODULE_ID_SEASONS	0x59



MODULE_Map_Table S5K3L6_MODULE_MAP[] = {
	{ S5K3L6_SENSOR_INFO_MODULE_ID_SUNNY,
		"sunny_s5k3l6", "sunny_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_TRULY,
		"truly_s5k3l6", "truly_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_A_KERR,
		"a_kerr_s5k3l6", "a_kerr_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_LITEARRAY,
		"litearray_s5k3l6", "litearray_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_DARLING,
		"darling_s5k3l6", "darling_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_QTECH,
		"qtech_s5k3l6", "qtech_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_OFLIM,
		"oflim_s5k3l6", "oflim_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_FOXCONN,
		"foxconn_s5k3l6", "foxconn_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_IMPORTEK,
		"importek_s5k3l6", "importek_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_ALTEK,
		"altek_s5k3l6", "altek_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_ABICO,
		"abico_s5k3l6", "abico_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_LITE_ON,
		"lite_on_s5k3l6", "lite_on_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_CHICONY,
		"chicony_s5k3l6", "chicony_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_PRIMAX,
		"primax_s5k3l6", "primax_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_SHARP,
		"sharp_s5k3l6", "sharp_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_MCNEX,
		"mcnex_s5k3l6", "mcnex_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_HOLITECH,
		"holitech_s5k3l6", "holitech_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_GOERTEK,
		"goertek_s5k3l6", "goertek_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_SHINETECH,
		"shinetech_s5k3l6", "shinetech_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_SUNWIN,
		"sunwin_s5k3l6", "sunwin_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_JSL,
		"jsl_s5k3l6", "jsl_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_UNION,
		"union_s5k3l6", "union_s5k3l6", NULL},
	{ S5K3L6_SENSOR_INFO_MODULE_ID_SEASONS,
		"seasons_s5k3l6", "seasons_s5k3l6", NULL},
};



#define S5K3L6_ID_ADDR 0x1

#define FLAG_MODULE_INFO_ADDR 0x0
#define FLAG_AF_ADDR 0xC
#define FLAG_AWB_ADDR 0x1A
#define FLAG_LSC_ADDR 0x28
#define FLAG_PDAF_ADDR 0x712
#define FLAG_PCC_ADDR 0xD36

#define FLAG_VALID_VALUE 0x01

/*CS:checksum*/
#define CS_MODULE_INFO_S_ADDR 0x01
#define CS_MODULE_INFO_E_ADDR 0x08
#define CS_MODULE_INFO_ADDR 0x09

#define CS_AF_S_ADDR 0x0D
#define CS_AF_E_ADDR 0x12
#define CS_AF_ADDR 0x13

#define CS_AWB_S_ADDR 0x1B
#define CS_AWB_E_ADDR 0x26
#define CS_AWB_ADDR 0x27

#define CS_LSC_S_ADDR 0x29
#define CS_LSC_E_ADDR 0x710
#define CS_LSC_ADDR 0x711

#define CS_PDAF_S_ADDR 0x713
#define CS_PDAF_E_ADDR 0xA8C
#define CS_PDAF_ADDR 0xA8D

#define CS_PCC_S_ADDR 0xD37
#define CS_PCC_E_ADDR 0xD9C
#define CS_PCC_ADDR 0xD9D





void s5k3l6_parse_module_name(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t sensor_module_id = e_ctrl->cal_data.mapdata[S5K3L6_ID_ADDR];

	parse_module_name(&(e_ctrl->module_info[0]), S5K3L6_MODULE_MAP,
		sizeof(S5K3L6_MODULE_MAP) / sizeof(MODULE_Map_Table),
		sensor_module_id);

}

int s5k3l6_validflag_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  flag = 0;

	if (e_ctrl->cal_data.mapdata[FLAG_MODULE_INFO_ADDR] != FLAG_VALID_VALUE) {
		pr_err("%s :%d: module info flag invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[FLAG_MODULE_INFO_ADDR]);
		flag |= 0x1;
	}

	if (e_ctrl->cal_data.mapdata[FLAG_AF_ADDR] != FLAG_VALID_VALUE) {
		pr_err("%s :%d: AF flag invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[FLAG_AF_ADDR]);
		flag |= 0x2;
	}

	if (e_ctrl->cal_data.mapdata[FLAG_AWB_ADDR] != FLAG_VALID_VALUE) {
		pr_err("%s :%d: AWB flag invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[FLAG_AWB_ADDR]);
		flag |= 0x4;
	}

	if (e_ctrl->cal_data.mapdata[FLAG_LSC_ADDR] != FLAG_VALID_VALUE) {
		pr_err("%s :%d: LSC flag invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[FLAG_LSC_ADDR]);
		flag |= 0x8;
	}

	if (e_ctrl->cal_data.mapdata[FLAG_PDAF_ADDR] != FLAG_VALID_VALUE) {
		pr_err("%s :%d: PDAF flag invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[FLAG_PDAF_ADDR]);
		flag |= 0x10;
	}

	if (e_ctrl->cal_data.mapdata[FLAG_PCC_ADDR] != FLAG_VALID_VALUE) {
		pr_err("%s :%d: DCC flag invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[FLAG_PCC_ADDR]);
		flag |= 0x20;
	}
	pr_info("%s :%d: valid info flag = 0x%x  %s\n",
		__func__, __LINE__, flag, (flag == 0) ? "true" : "false");

	return flag;

}

int s5k3l6_checksum_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  checksum = 0;
	int j;
	int rc = 0;

	pr_info("%s :%d: E\n", __func__, __LINE__);

	for (j = CS_MODULE_INFO_S_ADDR; j <= CS_MODULE_INFO_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum % 256) != e_ctrl->cal_data.mapdata[CS_MODULE_INFO_ADDR]) {
		pr_err("%s :%d: module info checksum fail\n", __func__, __LINE__);
		rc  |= 0x1;
	}
	checksum = 0;

	for (j = CS_AF_S_ADDR; j <= CS_AF_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum % 256) != e_ctrl->cal_data.mapdata[CS_AF_ADDR]) {
		pr_err("%s :%d: af info  checksum fail\n", __func__, __LINE__);
		rc  |= 0x2;
	}
	checksum = 0;

	for (j = CS_AWB_S_ADDR; j <= CS_AWB_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum % 256) != e_ctrl->cal_data.mapdata[CS_AWB_ADDR]) {
		pr_err("%s :%d: awb info checksum fail\n", __func__, __LINE__);
		rc  |= 0x4;
	}
	checksum = 0;

	for (j = CS_LSC_S_ADDR; j <= CS_LSC_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum % 256) != e_ctrl->cal_data.mapdata[CS_LSC_ADDR]) {
		pr_err("%s :%d: lsc info checksum fail\n", __func__, __LINE__);
		rc  |= 0x8;
	}
	checksum = 0;

	for (j = CS_PDAF_S_ADDR; j <= CS_PDAF_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum % 256) != e_ctrl->cal_data.mapdata[CS_PDAF_ADDR]) {
		pr_err("%s :%d: ois info checksum fail\n", __func__, __LINE__);
		rc  |= 0x10;
	}
	checksum = 0;

	for (j = CS_PCC_S_ADDR; j <= CS_PCC_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum % 256) != e_ctrl->cal_data.mapdata[CS_PCC_ADDR]) {
		pr_err("%s :%d: pdaf info checksum fail\n", __func__, __LINE__);
		rc  |= 0x20;
		}
	pr_info("%s :%d: cal info checksum rc = 0x%x %s\n",
			__func__, __LINE__, rc, (rc == 0) ? "true" : "false");
	return rc;
}

static struct zte_eeprom_fn_t s5k3l6_eeprom_func_tbl = {
	.eeprom_parse_map = zte_kernel_eeprom_parse_memory_map,
	.kernel_read_eeprom_memory = zte_kernel_read_eeprom_memory,
	.user_read_eeprom_memory = common_user_read_eeprom_memory,
	.eeprom_match_crc = NULL,
	.eeprom_checksum = s5k3l6_checksum_eeprom,
	.validflag_check_eeprom = s5k3l6_validflag_check_eeprom,
	.parse_module_name = s5k3l6_parse_module_name,
};

static const struct of_device_id s5k3l6_eeprom_dt_match[] = {
	{ .compatible = "zte,s5k3l6_035-eeprom", .data = &s5k3l6_eeprom_func_tbl},
};
MODULE_DEVICE_TABLE(of, common_eeprom_dt_match);

static int s5k3l6_eeprom_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(s5k3l6_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_probe_user(pdev, match);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static int s5k3l6_eeprom_platform_remove(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int32_t rc = 0;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(s5k3l6_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_remove(pdev);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static struct platform_driver s5k3l6_eeprom_platform_driver = {
	.driver = {
		.name = "zte,s5k3l6_035-eeprom",
		.owner = THIS_MODULE,
		.of_match_table = s5k3l6_eeprom_dt_match,
	},
	.probe = s5k3l6_eeprom_platform_probe,
	.remove = s5k3l6_eeprom_platform_remove,
};

static int __init s5k3l6_eeprom_init_module(void)
{
	int rc = 0;

	pr_info("%s E\n", __func__);
	rc = platform_driver_register(&s5k3l6_eeprom_platform_driver);
	pr_info("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static void __exit s5k3l6_eeprom_exit_module(void)
{
	platform_driver_unregister(&s5k3l6_eeprom_platform_driver);

}

module_init(s5k3l6_eeprom_init_module);
module_exit(s5k3l6_eeprom_exit_module);
MODULE_DESCRIPTION("ZTE EEPROM driver");
MODULE_LICENSE("GPL v2");

