

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "zte_eeprom.h"

#define OV16B10_031_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define OV16B10_031_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define OV16B10_031_SENSOR_INFO_MODULE_ID_A_KERR	0x03
#define OV16B10_031_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define OV16B10_031_SENSOR_INFO_MODULE_ID_DARLING	0x05
#define OV16B10_031_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define OV16B10_031_SENSOR_INFO_MODULE_ID_OFLIM		0x07
#define OV16B10_031_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define OV16B10_031_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define OV16B10_031_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define OV16B10_031_SENSOR_INFO_MODULE_ID_ABICO		0x14
#define OV16B10_031_SENSOR_INFO_MODULE_ID_LITE_ON	0x15
#define OV16B10_031_SENSOR_INFO_MODULE_ID_CHICONY	0x16
#define OV16B10_031_SENSOR_INFO_MODULE_ID_PRIMAX	0x17
#define OV16B10_031_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define OV16B10_031_SENSOR_INFO_MODULE_ID_LITE_ON_N	0x22
#define OV16B10_031_SENSOR_INFO_MODULE_ID_MCNEX		0x31
#define OV16B10_031_SENSOR_INFO_MODULE_ID_MCNEX_LSC	0xA0

MODULE_Map_Table OV16B10_031_MODULE_MAP[] = {
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_SUNNY,
		"sunny_ov16b10_031", "sunny_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_TRULY,
		"truly_ov16b10_031", "truly_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_A_KERR,
		"a_kerr_ov16b10_031", "a_kerr_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_LITEARRAY,
		"litearray_ov16b10_031", "litearray_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_DARLING,
		"darling_ov16b10_031", "darling_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_QTECH,
		"qtech_ov16b10_031", "qtech_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_OFLIM,
		"oflim_ov16b10_031", "oflim_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_FOXCONN,
		"foxconn_ov16b10_031", "foxconn_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_IMPORTEK,
		"importek_ov16b10_031", "importek_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_ALTEK,
		"altek_ov16b10_031", "altek_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_ABICO,
		"abico_ov16b10_031", "abico_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_LITE_ON,
		"lite_on_ov16b10_031", "lite_on_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_CHICONY,
		"chicony_ov16b10_031", "chicony_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_PRIMAX,
		"primax_ov16b10_031", "primax_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_SHARP,
		"sharp_ov16b10_031", "sharp_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_LITE_ON_N,
		"lite_on_new_ov16b10_031", "lite_on_new_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_MCNEX,
		"mcnex_ov16b10_031", "mcnex_ov16b10_031", NULL},
	{ OV16B10_031_SENSOR_INFO_MODULE_ID_MCNEX_LSC,
		"mcnex_lsc_ov16b10_031", "mcnex_lsc_ov16b10_031", NULL},
};

#define HI556_031_SENSOR_INFO_MODULE_ID_OFLIM       0x7
#define HI556_031_SENSOR_INFO_MODULE_ID_SUNWIN		0xA1
#define HI556_031_SENSOR_INFO_MODULE_ID_SHINE_TECH	0xA2


MODULE_Map_Table HI556_031_MODULE_MAP[] = {
	{ HI556_031_SENSOR_INFO_MODULE_ID_OFLIM,
		"oflim_hi556_031", "oflim_hi556_031", NULL},
	{ HI556_031_SENSOR_INFO_MODULE_ID_SUNWIN,
		"sunwin_hi556_031", "sunwin_hi556_031", NULL},
	{ HI556_031_SENSOR_INFO_MODULE_ID_SHINE_TECH,
		"shine_tech_hi556_031", "shine_tech_hi556_031", NULL},
};


#define OV16B10_031_ID_H_ADDR 0x1
#define OV16B10_031_ID_L_ADDR (OV16B10_031_ID_H_ADDR + 1)

#define FLAG_MODULE_INFO_ADDR 0x0
#define FLAG_AF_ADDR 0xC
#define FLAG_AWB_ADDR 0x1A
#define FLAG_LSC_ADDR 0x28
#define FLAG_PDAF_ADDR 0x712
#define FLAG_DCC_ADDR 0xB56

#define FLAG_VALID_VALUE 0x01


#define CS_MODULE_INFO_S_ADDR 0x01
#define CS_MODULE_INFO_E_ADDR 0x07
#define CS_MODULE_INFO_ADDR 0x09

#define CS_AF_S_ADDR 0x0D
#define CS_AF_E_ADDR 0x12
#define CS_AF_ADDR (CS_AF_E_ADDR+1)

#define CS_AWB_S_ADDR 0x1B
#define CS_AWB_E_ADDR 0x26
#define CS_AWB_ADDR (CS_AWB_E_ADDR+1)

#define CS_LSC_S_ADDR 0x29
#define CS_LSC_E_ADDR 0x710
#define CS_LSC_ADDR (CS_LSC_E_ADDR+1)

#define CS_PDAF_S_ADDR 0x713
#define CS_PDAF_E_ADDR 0xA8C
#define CS_PDAF_ADDR (CS_PDAF_E_ADDR+1)

#define CS_DCC_S_ADDR 0xB57
#define CS_DCC_E_ADDR 0xBBC
#define CS_DCC_ADDR (CS_DCC_E_ADDR+1)

#define CS_TOTAL_S_ADDR 0x0
#define CS_TOTAL_E_ADDR 0x1FFD
#define CS_TOTAL_H_ADDR (CS_TOTAL_E_ADDR+1)
#define CS_TOTAL_L_ADDR (CS_TOTAL_H_ADDR+1)

#define CS_DUAL_IQ_MA_S_ADDR 0x0C42
#define CS_DUAL_IQ_MA_E_ADDR 0x0C56
#define CS_DUAL_IQ_MA_ADDR (CS_DUAL_IQ_MA_E_ADDR+1)

#define HI556_031_START_ADDR 0x0C5B

#define CS_SLAVE_AWB_S_ADDR HI556_031_START_ADDR
#define CS_SLAVE_AWB_E_ADDR 0x0C68
#define CS_SLAVE_AWB_ADDR (CS_SLAVE_AWB_E_ADDR+1)

#define CS_SLAVE_LSC_S_ADDR 0x0C6B
#define CS_SLAVE_LSC_E_ADDR 0x1352
#define CS_SLAVE_LSC_ADDR (CS_SLAVE_LSC_E_ADDR + 1)


#define HI556_031_ID_H_ADDR HI556_031_START_ADDR
#define HI556_031_ID_L_ADDR (HI556_031_START_ADDR + 1)

void ov16b10_031_parse_module_name(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t sensor_module_id = (e_ctrl->cal_data.mapdata[OV16B10_031_ID_H_ADDR]);
	parse_module_name(&(e_ctrl->module_info[0]), OV16B10_031_MODULE_MAP,
		sizeof(OV16B10_031_MODULE_MAP) / sizeof(MODULE_Map_Table),
		sensor_module_id);

	sensor_module_id = (e_ctrl->cal_data.mapdata[HI556_031_ID_H_ADDR]);
	parse_module_name(&(e_ctrl->module_info[1]), HI556_031_MODULE_MAP,
		sizeof(HI556_031_MODULE_MAP) / sizeof(MODULE_Map_Table),
		sensor_module_id);
}

int ov16b10_031_validflag_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
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


	if (e_ctrl->cal_data.mapdata[FLAG_DCC_ADDR] != FLAG_VALID_VALUE) {
		pr_err("%s :%d: DCC flag invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[FLAG_DCC_ADDR]);
		flag |= 0x20;
	}

	pr_info("%s :%d: valid info flag = 0x%x  %s\n",
		__func__, __LINE__, flag, (flag == 0) ? "true" : "false");

	return flag;

}

int ov16b10_031_checksum_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  checksum = 0;
	int j;
	int rc = 0;

	pr_info("%s :%d: E", __func__, __LINE__);

	for (j = CS_MODULE_INFO_S_ADDR; j <= CS_MODULE_INFO_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_MODULE_INFO_ADDR]) {
		pr_err("%s :%d: module info checksum fail\n", __func__, __LINE__);
		rc  |= 0x1;
	}
	checksum = 0;

	for (j = CS_AF_S_ADDR; j <= CS_AF_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_AF_ADDR]) {
		pr_err("%s :%d: af info  checksum fail\n", __func__, __LINE__);
		rc  |= 0x2;
	}
	checksum = 0;

	for (j = CS_AWB_S_ADDR; j <= CS_AWB_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_AWB_ADDR]) {
		pr_err("%s :%d: awb info checksum fail\n", __func__, __LINE__);
		rc  |= 0x4;
	}
	checksum = 0;

	for (j = CS_LSC_S_ADDR; j <= CS_LSC_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_LSC_ADDR]) {
		pr_err("%s :%d: lsc info checksum fail\n", __func__, __LINE__);
		rc  |= 0x8;
	}
	checksum = 0;

	for (j = CS_PDAF_S_ADDR; j <= CS_PDAF_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_PDAF_ADDR]) {
		pr_err("%s :%d: pdaf info checksum fail\n", __func__, __LINE__);
		rc  |= 0x8;
	}
	checksum = 0;

	for (j = CS_DCC_S_ADDR; j <= CS_DCC_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_DCC_ADDR]) {
		pr_err("%s :%d: pdaf info checksum fail\n", __func__, __LINE__);
		rc  |= 0x20;
	}
	checksum = 0;

	checksum = 0;
	for (j = CS_TOTAL_S_ADDR; j <= CS_TOTAL_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((((checksum >> 8) & 0xff) != e_ctrl->cal_data.mapdata[CS_TOTAL_H_ADDR])
		|| ((checksum  & 0xff) != e_ctrl->cal_data.mapdata[CS_TOTAL_L_ADDR])) {
		pr_err("%s :%d: total info checksum fail\n", __func__, __LINE__);
		rc  |= 0x80;
	}


	checksum = 0;
	for (j = CS_SLAVE_AWB_S_ADDR; j <= CS_SLAVE_AWB_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_SLAVE_AWB_ADDR]) {
		pr_err("%s :%d: slave awb info checksum fail\n", __func__, __LINE__);
		rc  |= 0x100;
	}

	checksum = 0;
	for (j = CS_SLAVE_LSC_S_ADDR; j <= CS_SLAVE_LSC_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_SLAVE_LSC_ADDR]) {
		pr_err("%s :%d: slav lsc info checksum fail\n", __func__, __LINE__);
		rc  |= 0x200;
	}

	checksum = 0;
	for (j = CS_DUAL_IQ_MA_S_ADDR; j <= CS_DUAL_IQ_MA_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff)  != e_ctrl->cal_data.mapdata[CS_DUAL_IQ_MA_ADDR]) {
		pr_err("%s :%d: dual master info checksum fail\n", __func__, __LINE__);
		rc  |= 0x400;
	}

	pr_info("%s :%d: cal info checksum rc = 0x%x %s\n",
			__func__, __LINE__, rc, (rc == 0) ? "true" : "false");
	return rc;
}

static struct zte_eeprom_fn_t ov16b10_031_eeprom_func_tbl = {
	.eeprom_parse_map = zte_kernel_eeprom_parse_memory_map,
	.kernel_read_eeprom_memory = zte_kernel_read_eeprom_memory,
	.user_read_eeprom_memory = common_user_read_eeprom_memory,
	.eeprom_match_crc = NULL,
	.eeprom_checksum = ov16b10_031_checksum_eeprom,
	.validflag_check_eeprom = ov16b10_031_validflag_check_eeprom,
	.parse_module_name = ov16b10_031_parse_module_name,
};

static const struct of_device_id ov16b10_031_eeprom_dt_match[] = {
	{ .compatible = "zte,ov16b10_031-eeprom", .data = &ov16b10_031_eeprom_func_tbl},
};
MODULE_DEVICE_TABLE(of, common_eeprom_dt_match);

static int ov16b10_031_eeprom_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(ov16b10_031_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_probe_user(pdev, match);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static int ov16b10_031_eeprom_platform_remove(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int32_t rc = 0;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(ov16b10_031_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_remove(pdev);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static struct platform_driver ov16b10_031_eeprom_platform_driver = {
	.driver = {
		.name = "zte,ov16b10_031-eeprom",
		.owner = THIS_MODULE,
		.of_match_table = ov16b10_031_eeprom_dt_match,
	},
	.probe = ov16b10_031_eeprom_platform_probe,
	.remove = ov16b10_031_eeprom_platform_remove,
};

static int __init ov16b10_031_eeprom_init_module(void)
{
	int rc = 0;

	pr_info("%s E\n", __func__);
	rc = platform_driver_register(&ov16b10_031_eeprom_platform_driver);
	pr_info("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static void __exit ov16b10_031_eeprom_exit_module(void)
{
	platform_driver_unregister(&ov16b10_031_eeprom_platform_driver);

}

module_init(ov16b10_031_eeprom_init_module);
module_exit(ov16b10_031_eeprom_exit_module);
MODULE_DESCRIPTION("ZTE EEPROM driver");
MODULE_LICENSE("GPL v2");

