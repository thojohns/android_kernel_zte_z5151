

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "zte_eeprom.h"

#define OV8856_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define OV8856_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define OV8856_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define OV8856_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define OV8856_SENSOR_INFO_MODULE_ID_DARLING	0x05
#define OV8856_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define OV8856_SENSOR_INFO_MODULE_ID_OFLIM		0x07
#define OV8856_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define OV8856_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define OV8856_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define OV8856_SENSOR_INFO_MODULE_ID_ABICO		0x14
#define OV8856_SENSOR_INFO_MODULE_ID_LITE_ON	0x15
#define OV8856_SENSOR_INFO_MODULE_ID_CHICONY	0x16
#define OV8856_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define OV8856_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define OV8856_SENSOR_INFO_MODULE_ID_MCNEX		0x31
#define OV8856_SENSOR_INFO_MODULE_ID_HOLITECH	0x42
#define OV8856_SENSOR_INFO_MODULE_ID_GOERTEK	0x54
#define OV8856_SENSOR_INFO_MODULE_ID_SHINETECH	0x55
#define OV8856_SENSOR_INFO_MODULE_ID_SUNWIN	0x56
#define OV8856_SENSOR_INFO_MODULE_ID_JSL			0x57
#define OV8856_SENSOR_INFO_MODULE_ID_UNION		0x58
#define OV8856_SENSOR_INFO_MODULE_ID_SEASIONS	0x59

MODULE_Map_Table OV8856_MODULE_MAP[] = {
	{ OV8856_SENSOR_INFO_MODULE_ID_SUNNY,
		"sunny_ov8856", "sunny_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_TRULY,
		"truly_ov8856", "truly_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_A_KERR,
		"a_kerr_ov8856", "a_kerr_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_LITEARRAY,
		"litearray_ov8856", "litearray_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_DARLING,
		"darling_ov8856", "darling_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_QTECH,
		"qtech_ov8856", "qtech_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_OFLIM,
		"oflim_ov8856", "oflim_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_FOXCONN,
		"foxconn_ov8856", "foxconn_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_IMPORTEK,
		"importek_ov8856", "importek_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_ALTEK,
		"altek_ov8856", "altek_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_ABICO,
		"abico_ov8856", "abico_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_LITE_ON,
		"lite_on_ov8856", "lite_on_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_CHICONY,
		"chicony_ov8856", "chicony_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_PRIMAX,
		"primax_ov8856", "primax_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_SHARP,
		"sharp_ov8856", "sharp_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_MCNEX,
		"mcnex_ov8856", "mcnex_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_HOLITECH,
		"holitech_ov8856", "holitech_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_GOERTEK,
		"goertek_ov8856", "goertek_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_SHINETECH,
		"shinetech_ov8856", "shinetech_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_SUNWIN,
		"sunwin_ov8856", "sunwin_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_JSL,
		"jsl_ov8856", "jsl_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_UNION,
		"union_ov8856", "union_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_SEASIONS,
		"seasons_ov8856", "seasons_ov8856", NULL},
};

#define OV8856_GROUP_ONE_MASK		0xC0
#define OV8856_GROUP_ONE_VALUE		0x40
#define OV8856_GROUP_TWO_MASK		0x30
#define OV8856_GROUP_TWO_VALUE		0x10
#define OV8856_GROUP_INVLID_VALUE	-1

#define OV8856_ID_ABS_ADDR 0x7011

#define OV8856_MODULE_INFO_GOUP_VALID_ABS_ADDR 0x7010
#define OV8856_MODULE_INFO_GOUP_OFF_SIZE 0x8

#define MI_S_ABS_ADDR 0x7011
#define MI_E_ABS_ADDR 0x7018
#define OV8856_MODULE_INFO_SIZE (MI_E_ABS_ADDR - MI_S_ABS_ADDR + 1)


#define OV8856_LSC_GOUP_VALID_ABS_ADDR 0x7028
#define OV8856_LSC_GOUP_OFF_SIZE 0xF1

#define CS_LSC_S_ABS_ADDR 0x7029
#define CS_LSC_E_ABS_ADDR 0x7118
#define OV8856_LSC_SIZE (CS_LSC_E_ABS_ADDR - CS_LSC_S_ABS_ADDR + 1)


#define MI_GROUP_ADDR 0x0
#define LSC_GROUP_ADDR (MI_GROUP_ADDR + 1 + OV8856_MODULE_INFO_SIZE)

#define CS_LSC_S_ADDR (LSC_GROUP_ADDR + 1)
#define CS_LSC_E_ADDR (CS_LSC_S_ADDR + OV8856_LSC_SIZE - 1)
#define CS_LSC_ADDR (CS_LSC_E_ADDR + 1)


#define OV8856_ID_ADDR (MI_GROUP_ADDR + 1)

void ov8856_parse_module_name(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t sensor_module_id = e_ctrl->cal_data.mapdata[OV8856_ID_ADDR];

	parse_module_name(&(e_ctrl->module_info[0]), OV8856_MODULE_MAP,
		sizeof(OV8856_MODULE_MAP) / sizeof(MODULE_Map_Table),
		sensor_module_id);
}


static int  ov8856_get_group_id(uint8_t data)
{
	if ((data & OV8856_GROUP_ONE_MASK) == OV8856_GROUP_ONE_VALUE)
		return Group_One;
	else if ((data & OV8856_GROUP_TWO_MASK) == OV8856_GROUP_TWO_VALUE)
		return Group_Two;

	pr_err("%s :%d: group id  = 0x%x invlid", __func__,
				__LINE__, data);
	return Invlid_Group;
}

static int ov8856_validgroup_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  rc, flag = 0;

	rc = ov8856_get_group_id(e_ctrl->cal_data.mapdata[MI_GROUP_ADDR]);
	if (rc == Invlid_Group) {
		pr_err("%s :%d: module info group invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[MI_GROUP_ADDR]);
		flag |= 0x1;
	}

	rc = ov8856_get_group_id(e_ctrl->cal_data.mapdata[LSC_GROUP_ADDR]);
	if (rc == Invlid_Group) {
		pr_err("%s :%d: lsc group invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[LSC_GROUP_ADDR]);
		flag |= 0x2;
	}

	pr_info("%s :%d: group id valid = 0x%x  %s\n",
		__func__, __LINE__, flag, (flag == 0) ? "true" : "false");

	return flag;

}

static int ov8856_checksum_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  checksum = 0;
	int j;
	int rc = 0;

	for (j = CS_LSC_S_ADDR; j <= CS_LSC_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];

	if ((checksum & 0xff) != e_ctrl->cal_data.mapdata[CS_LSC_ADDR]) {
		pr_err("%s :%d: lsc info checksum %d: fail", __func__, __LINE__, checksum);
		rc  |= 0x1;
	}

	pr_info("%s :%d: cal info checksum rc = 0x%x %s\n",
			__func__, __LINE__, rc, (rc == 0) ? "true" : "false");
	return rc;
}

int ov8856_read_eeprom_init(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc =  0;
	uint16_t temp;

	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),
			0x5000, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: read failed\n", __func__);
		return rc;
	}
	pr_err("%s:0x5000 temp=0x%X", __func__, temp);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),
		0x5001, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s:0x5001 temp=0x%X", __func__, temp);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x0100, 0x01, MSM_CAMERA_I2C_BYTE_DATA);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x5001, (0x00 & 0x08) | (temp & (~0x08)),
		MSM_CAMERA_I2C_BYTE_DATA);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d84, 0xc0, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),
		0x3d84, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d88, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d89, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d8a, 0x72, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d8b, 0x0a, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	udelay(5);
	return rc;
}

void ov8856_read_eeprom_end(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;
	int rc =  0;

	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),
		0x5001, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: read failed\n", __func__);
		return;
	}
	pr_err("%s:0x5001 temp=0x%X", __func__, temp);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x5001, (0x08 & 0x08) | (temp & (~0x08)),
		MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
}

int ov8856_user_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_array *eeprom_map_array)
{
	int rc =  0, i, j, group_off;
	uint8_t *memptr = e_ctrl->cal_data.mapdata;
	struct msm_eeprom_mem_map_t *eeprom_map;

	for (j = 0; j < eeprom_map_array->msm_size_of_max_mappings; j++) {
		eeprom_map = &(eeprom_map_array->memory_map[j]);
		if (e_ctrl->i2c_client.cci_client) {
			e_ctrl->i2c_client.cci_client->sid =
				eeprom_map->slave_addr >> 1;
		} else if (e_ctrl->i2c_client.client) {
			e_ctrl->i2c_client.client->addr =
				eeprom_map->slave_addr >> 1;
		}
		pr_info("Slave Addr: 0x%X\n", eeprom_map->slave_addr);
		pr_info("Memory map Size: %d",
			eeprom_map->memory_map_size);
		rc = ov8856_read_eeprom_init(e_ctrl);
		if (rc < 0) {
			pr_err("%s: read failed\n", __func__);
			return rc;
		}
		for (i = 0; i < eeprom_map->memory_map_size; i++) {
			switch (eeprom_map->mem_settings[i].i2c_operation) {
			case MSM_CAM_WRITE: {
				e_ctrl->i2c_client.addr_type =
					eeprom_map->mem_settings[i].addr_type;
				rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(e_ctrl->i2c_client),
					eeprom_map->mem_settings[i].reg_addr,
					eeprom_map->mem_settings[i].reg_data,
					eeprom_map->mem_settings[i].data_type);
				msleep(eeprom_map->mem_settings[i].delay);
				if (rc < 0) {
					pr_err("%s: page write failed\n",
						__func__);
					return rc;
				}
			}
			break;
			case MSM_CAM_POLL: {
				e_ctrl->i2c_client.addr_type =
					eeprom_map->mem_settings[i].addr_type;
				rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_poll(
					&(e_ctrl->i2c_client),
					eeprom_map->mem_settings[i].reg_addr,
					eeprom_map->mem_settings[i].reg_data,
					eeprom_map->mem_settings[i].data_type,
					eeprom_map->mem_settings[i].delay);
				if (rc < 0) {
					pr_err("%s: poll failed\n",
						__func__);
					return rc;
				}
			}
			break;
			case MSM_CAM_READ: {
				e_ctrl->i2c_client.addr_type =
					eeprom_map->mem_settings[i].addr_type;
				rc = e_ctrl->i2c_client.i2c_func_tbl->
					i2c_read_seq(&(e_ctrl->i2c_client),
					eeprom_map->mem_settings[i].reg_addr,
					memptr,
					eeprom_map->mem_settings[i].reg_data);
				msleep(eeprom_map->mem_settings[i].delay);
				if (rc < 0) {
					pr_err("%s: read failed\n",
						__func__);
					return rc;
				}
				if ((eeprom_map->mem_settings[i].reg_addr
					== OV8856_MODULE_INFO_GOUP_VALID_ABS_ADDR)
					&& ((i + 1) <= eeprom_map->memory_map_size)) {
					group_off = ov8856_get_group_id(*memptr);
					if (group_off == Invlid_Group) {
						pr_err("%s: 0x%x: 0x%x group invlid\n", __func__,
							eeprom_map->mem_settings[i].reg_addr, *memptr);
					} else {
						pr_info("%s: 0x%x: 0x%x group id = %d\n", __func__,
							eeprom_map->mem_settings[i].reg_addr, *memptr, group_off);
						group_off -= Group_One;
						eeprom_map->mem_settings[i + 1].reg_addr
							+= OV8856_MODULE_INFO_GOUP_OFF_SIZE * group_off;
					}
				}
				if ((eeprom_map->mem_settings[i].reg_addr
					== OV8856_LSC_GOUP_VALID_ABS_ADDR)
					&& ((i + 1) <= eeprom_map->memory_map_size)) {
					group_off = ov8856_get_group_id(*memptr);
					if (group_off == Invlid_Group) {
						pr_err("%s: 0x%x: 0x%x group invlid\n", __func__,
							eeprom_map->mem_settings[i].reg_addr, *memptr);
					} else {
						pr_info("%s: 0x%x: 0x%x group id = %d\n", __func__,
							eeprom_map->mem_settings[i].reg_addr, *memptr, group_off);
						group_off -= Group_One;
						eeprom_map->mem_settings[i + 1].reg_addr
							+= OV8856_LSC_GOUP_OFF_SIZE * group_off;
					}
				}
				memptr += eeprom_map->mem_settings[i].reg_data;
			}
			break;
			default:
				pr_err("%s: %d Invalid i2c operation LC:%d\n",
					__func__, __LINE__, i);
				return rc;
			}
		}
	}
	ov8856_read_eeprom_end(e_ctrl);
	return rc;
}

static struct zte_eeprom_fn_t ov8856_eeprom_func_tbl = {
	.eeprom_parse_map = NULL,
	.kernel_read_eeprom_memory = NULL,
	.user_read_eeprom_memory = ov8856_user_read_eeprom_memory,
	.eeprom_match_crc = NULL,
	.eeprom_checksum = ov8856_checksum_eeprom,
	.validflag_check_eeprom = ov8856_validgroup_check_eeprom,
	.parse_module_name = ov8856_parse_module_name,
};

static const struct of_device_id ov8856_eeprom_dt_match[] = {
	{ .compatible = "zte,ov8856-eeprom", .data = &ov8856_eeprom_func_tbl},
};
MODULE_DEVICE_TABLE(of, common_eeprom_dt_match);

static int ov8856_eeprom_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(ov8856_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_probe_user(pdev, match);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static int ov8856_eeprom_platform_remove(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int32_t rc = 0;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(ov8856_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_remove(pdev);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static struct platform_driver ov8856_eeprom_platform_driver = {
	.driver = {
		.name = "zte,ov8856-eeprom",
		.owner = THIS_MODULE,
		.of_match_table = ov8856_eeprom_dt_match,
	},
	.probe = ov8856_eeprom_platform_probe,
	.remove = ov8856_eeprom_platform_remove,
};

static int __init ov8856_eeprom_init_module(void)
{
	int rc = 0;

	pr_info("%s E\n", __func__);
	rc = platform_driver_register(&ov8856_eeprom_platform_driver);
	pr_info("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static void __exit ov8856_eeprom_exit_module(void)
{
	platform_driver_unregister(&ov8856_eeprom_platform_driver);

}

module_init(ov8856_eeprom_init_module);
module_exit(ov8856_eeprom_exit_module);
MODULE_DESCRIPTION("ZTE EEPROM driver");
MODULE_LICENSE("GPL v2");

