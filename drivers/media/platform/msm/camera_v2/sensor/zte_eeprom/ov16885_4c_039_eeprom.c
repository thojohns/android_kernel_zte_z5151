

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "zte_eeprom.h"

#define OV16885_4C_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define OV16885_4C_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define OV16885_4C_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define OV16885_4C_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define OV16885_4C_SENSOR_INFO_MODULE_ID_DARLING	0x05
#define OV16885_4C_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define OV16885_4C_SENSOR_INFO_MODULE_ID_OFLIM		0x07
#define OV16885_4C_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define OV16885_4C_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define OV16885_4C_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define OV16885_4C_SENSOR_INFO_MODULE_ID_ABICO		0x14
#define OV16885_4C_SENSOR_INFO_MODULE_ID_LITE_ON	0x15
#define OV16885_4C_SENSOR_INFO_MODULE_ID_CHICONY	0x16
#define OV16885_4C_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define OV16885_4C_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define OV16885_4C_SENSOR_INFO_MODULE_ID_LITE_ON_N	0x22
#define OV16885_4C_SENSOR_INFO_MODULE_ID_MCNEX		0x31
#define OV16885_4C_SENSOR_INFO_MODULE_ID_MCNEX_LSC	0xA0

MODULE_Map_Table OV16885_4C_MODULE_MAP[] = {
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_SUNNY,
		"sunny_ov16885_4c", "sunny_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_TRULY,
		"truly_ov16885_4c", "truly_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_A_KERR,
		"a_kerr_ov16885_4c", "a_kerr_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_LITEARRAY,
		"litearray_ov16885_4c", "litearray_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_DARLING,
		"darling_ov16885_4c", "darling_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_QTECH,
		"qtech_ov16885_4c", "qtech_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_OFLIM,
		"oflim_ov16885_4c", "oflim_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_FOXCONN,
		"foxconn_ov16885_4c", "foxconn_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_IMPORTEK,
		"importek_ov16885_4c", "importek_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_ALTEK,
		"altek_ov16885_4c", "altek_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_ABICO,
		"abico_ov16885_4c", "abico_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_LITE_ON,
		"lite_on_ov16885_4c", "lite_on_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_CHICONY,
		"chicony_ov16885_4c", "chicony_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_PRIMAX,
		"primax_ov16885_4c", "primax_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_SHARP,
		"sharp_ov16885_4c", "sharp_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_LITE_ON_N,
		"lite_on_new_ov16885_4c", "lite_on_new_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_MCNEX,
		"mcnex_ov16885_4c", "mcnex_ov16885_4c", NULL},
	{ OV16885_4C_SENSOR_INFO_MODULE_ID_MCNEX_LSC,
		"mcnex_lsc_ov16885_4c", "mcnex_lsc_ov16885_4c", NULL},
};



#define OV16885_4C_ID_H_ADDR 0x1
#define OV16885_4C_ID_L_ADDR (OV16885_4C_ID_H_ADDR + 1)

#define FLAG_MODULE_INFO_ADDR 0x0

#define FLAG_AWB_ADDR 0x1A
#define FLAG_LSC_ADDR 0x28

#define FLAG_VALID_VALUE 0x01


#define CS_MODULE_INFO_S_ADDR 0x01
#define CS_MODULE_INFO_E_ADDR 0x08
#define CS_MODULE_INFO_ADDR 0x09

#define CS_AWB_S_ADDR 0x1B
#define CS_AWB_E_ADDR 0x26
#define CS_AWB_ADDR (CS_AWB_E_ADDR+1)

#define CS_LSC_S_ADDR 0x29
#define CS_LSC_E_ADDR 0x710
#define CS_LSC_ADDR (CS_LSC_E_ADDR+1)

void ov16885_4c_parse_module_name(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t sensor_module_id = (e_ctrl->cal_data.mapdata[OV16885_4C_ID_H_ADDR] << 8)
								| e_ctrl->cal_data.mapdata[OV16885_4C_ID_L_ADDR];
	parse_module_name(&(e_ctrl->module_info[0]), OV16885_4C_MODULE_MAP,
		sizeof(OV16885_4C_MODULE_MAP) / sizeof(MODULE_Map_Table),
		sensor_module_id);
}

int ov16885_4c_validflag_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  flag = 0;

	if (e_ctrl->cal_data.mapdata[FLAG_MODULE_INFO_ADDR] != FLAG_VALID_VALUE) {
		pr_err("%s :%d: module info flag invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[FLAG_MODULE_INFO_ADDR]);
		flag |= 0x1;
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

	pr_info("%s :%d: valid info flag = 0x%x  %s\n",
		__func__, __LINE__, flag, (flag == 0) ? "true" : "false");

	return flag;

}

int ov16885_4c_checksum_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
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

	for (j = CS_AWB_S_ADDR; j <= CS_AWB_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_AWB_ADDR]) {
		pr_err("%s :%d: awb info checksum fail\n", __func__, __LINE__);
		rc  |= 0x2;
	}
	checksum = 0;

	for (j = CS_LSC_S_ADDR; j <= CS_LSC_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_LSC_ADDR]) {
		pr_err("%s :%d: lsc info checksum fail\n", __func__, __LINE__);
		rc  |= 0x6;
	}
	checksum = 0;

	pr_info("%s :%d: cal info checksum rc = 0x%x %s\n",
			__func__, __LINE__, rc, (rc == 0) ? "true" : "false");
	return rc;
}

static struct zte_eeprom_fn_t ov16885_4c_eeprom_func_tbl = {
	.eeprom_parse_map = zte_kernel_eeprom_parse_memory_map,
	.kernel_read_eeprom_memory = zte_kernel_read_eeprom_memory,
	.user_read_eeprom_memory = common_user_read_eeprom_memory,
	.eeprom_match_crc = NULL,
	.eeprom_checksum = ov16885_4c_checksum_eeprom,
	.validflag_check_eeprom = ov16885_4c_validflag_check_eeprom,
	.parse_module_name = ov16885_4c_parse_module_name,
};

static const struct of_device_id ov16885_4c_eeprom_dt_match[] = {
	{ .compatible = "zte,ov16885_4c_039-eeprom", .data = &ov16885_4c_eeprom_func_tbl},
};
MODULE_DEVICE_TABLE(of, common_eeprom_dt_match);

static int ov16885_4c_eeprom_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(ov16885_4c_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_probe_user(pdev, match);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static int ov16885_4c_eeprom_platform_remove(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int32_t rc = 0;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(ov16885_4c_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_remove(pdev);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static struct platform_driver ov16885_4c_eeprom_platform_driver = {
	.driver = {
		.name = "zte,ov16885_4c_039-eeprom",
		.owner = THIS_MODULE,
		.of_match_table = ov16885_4c_eeprom_dt_match,
	},
	.probe = ov16885_4c_eeprom_platform_probe,
	.remove = ov16885_4c_eeprom_platform_remove,
};

static int __init ov16885_4c_eeprom_init_module(void)
{
	int rc = 0;

	pr_info("%s E\n", __func__);
	rc = platform_driver_register(&ov16885_4c_eeprom_platform_driver);
	pr_info("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static void __exit ov16885_4c_eeprom_exit_module(void)
{
	platform_driver_unregister(&ov16885_4c_eeprom_platform_driver);

}

module_init(ov16885_4c_eeprom_init_module);
module_exit(ov16885_4c_eeprom_exit_module);
MODULE_DESCRIPTION("ZTE EEPROM driver");
MODULE_LICENSE("GPL v2");

