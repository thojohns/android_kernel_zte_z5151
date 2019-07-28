

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "zte_eeprom.h"

#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_DARLING	0x05
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_OFLIM		0x07
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_ABICO		0x14
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_LITE_ON	0x15
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_CHICONY	0x16
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_LITE_ON_N	0x22
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_MCNEX		0x31
#define SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_MCNEX_LSC	0xA0

MODULE_Map_Table SP2509V_Z6450T_090_MODULE_MAP[] = {
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_SUNNY,
		"sunny_sp2509v_z6450t_090", "sunny_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_TRULY,
		"truly_sp2509v_z6450t_090", "truly_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_A_KERR,
		"a_kerr_sp2509v_z6450t_090", "a_kerr_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_LITEARRAY,
		"litearray_sp2509v_z6450t_090", "litearray_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_DARLING,
		"darling_sp2509v_z6450t_090", "darling_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_QTECH,
		"qtech_sp2509v_z6450t_090", "qtech_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_OFLIM,
		"oflim_sp2509v_z6450t_090", "oflim_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_FOXCONN,
		"foxconn_sp2509v_z6450t_090", "foxconn_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_IMPORTEK,
		"importek_sp2509v_z6450t_090", "importek_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_ALTEK,
		"altek_sp2509v_z6450t_090", "altek_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_ABICO,
		"abico_sp2509v_z6450t_090", "abico_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_LITE_ON,
		"lite_on_sp2509v_z6450t_090", "lite_on_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_CHICONY,
		"chicony_sp2509v_z6450t_090", "chicony_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_PRIMAX,
		"primax_sp2509v_z6450t_090", "primax_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_SHARP,
		"sharp_sp2509v_z6450t_090", "sharp_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_LITE_ON_N,
		"lite_on_new_sp2509v_z6450t_090", "lite_on_new_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_MCNEX,
		"mcnex_sp2509v_z6450t_090", "mcnex_sp2509v_z6450t_090", NULL},
	{ SP2509V_Z6450T_090_SENSOR_INFO_MODULE_ID_MCNEX_LSC,
		"mcnex_lsc_sp2509v_z6450t_090", "mcnex_lsc_sp2509v_z6450t_090", NULL},
};


#define SP2509V_Z6450T_090_ID_ADDR 0x1

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

void sp2509v_z6450t_090_parse_module_name(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t sensor_module_id = e_ctrl->cal_data.mapdata[SP2509V_Z6450T_090_ID_ADDR];

	parse_module_name(&(e_ctrl->module_info[0]), SP2509V_Z6450T_090_MODULE_MAP,
		sizeof(SP2509V_Z6450T_090_MODULE_MAP) / sizeof(MODULE_Map_Table),
		sensor_module_id);
}

int sp2509v_z6450t_090_validflag_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
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

int sp2509v_z6450t_090_checksum_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
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

static struct zte_eeprom_fn_t sp2509v_z6450t_090_eeprom_func_tbl = {
	.eeprom_parse_map = zte_kernel_eeprom_parse_memory_map,
	.kernel_read_eeprom_memory = zte_kernel_read_eeprom_memory,
	.user_read_eeprom_memory = common_user_read_eeprom_memory,
	.eeprom_match_crc = NULL,
	.eeprom_checksum = sp2509v_z6450t_090_checksum_eeprom,
	.validflag_check_eeprom = sp2509v_z6450t_090_validflag_check_eeprom,
	.parse_module_name = sp2509v_z6450t_090_parse_module_name,
};

static const struct of_device_id sp2509v_z6450t_090_eeprom_dt_match[] = {
	{ .compatible = "zte,sp2509v_z6450t_090-eeprom", .data = &sp2509v_z6450t_090_eeprom_func_tbl},
};
MODULE_DEVICE_TABLE(of, common_eeprom_dt_match);

static int sp2509v_z6450t_090_eeprom_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(sp2509v_z6450t_090_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_probe_user(pdev, match);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static int sp2509v_z6450t_090_eeprom_platform_remove(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int32_t rc = 0;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(sp2509v_z6450t_090_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_remove(pdev);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static struct platform_driver sp2509v_z6450t_090_eeprom_platform_driver = {
	.driver = {
		.name = "zte,sp2509v_z6450t_090-eeprom",
		.owner = THIS_MODULE,
		.of_match_table = sp2509v_z6450t_090_eeprom_dt_match,
	},
	.probe = sp2509v_z6450t_090_eeprom_platform_probe,
	.remove = sp2509v_z6450t_090_eeprom_platform_remove,
};

static int __init sp2509v_z6450t_090_eeprom_init_module(void)
{
	int rc = 0;

	pr_info("%s E\n", __func__);
	rc = platform_driver_register(&sp2509v_z6450t_090_eeprom_platform_driver);
	pr_info("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static void __exit sp2509v_z6450t_090_eeprom_exit_module(void)
{
	platform_driver_unregister(&sp2509v_z6450t_090_eeprom_platform_driver);

}

module_init(sp2509v_z6450t_090_eeprom_init_module);
module_exit(sp2509v_z6450t_090_eeprom_exit_module);
MODULE_DESCRIPTION("ZTE EEPROM driver");
MODULE_LICENSE("GPL v2");

