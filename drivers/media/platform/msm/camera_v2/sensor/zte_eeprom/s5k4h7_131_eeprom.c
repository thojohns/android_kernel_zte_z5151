

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "zte_eeprom.h"

#define S5K4H7_131_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_DARLING	0x05
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_OFLIM		0x07
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_ABICO		0x14
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_LITE_ON	0x15
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_CHICONY	0x16
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_MCNEX		0x31
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_HOLITECH	0x42
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_GOERTEK	0x54
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_SHINETECH	0x55
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_SUNWIN	0x56
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_JSL			0x57
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_UNION		0x58
#define S5K4H7_131_SENSOR_INFO_MODULE_ID_SEASIONS	0x59

MODULE_Map_Table S5K4H7_131_MODULE_MAP[] = {
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_SUNNY,
		"sunny_s5k4h7_131", "sunny_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_TRULY,
		"truly_s5k4h7_131", "truly_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_A_KERR,
		"a_kerr_s5k4h7_131", "a_kerr_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_LITEARRAY,
		"litearray_s5k4h7_131", "litearray_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_DARLING,
		"darling_s5k4h7_131", "darling_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_QTECH,
		"qtech_s5k4h7_131", "qtech_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_OFLIM,
		"oflim_s5k4h7_131", "oflim_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_FOXCONN,
		"foxconn_s5k4h7_131", "foxconn_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_IMPORTEK,
		"importek_s5k4h7_131", "importek_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_ALTEK,
		"altek_s5k4h7_131", "altek_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_ABICO,
		"abico_s5k4h7_131", "abico_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_LITE_ON,
		"lite_on_s5k4h7_131", "lite_on_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_CHICONY,
		"chicony_s5k4h7_131", "chicony_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_PRIMAX,
		"primax_s5k4h7_131", "primax_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_SHARP,
		"sharp_s5k4h7_131", "sharp_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_MCNEX,
		"mcnex_s5k4h7_131", "mcnex_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_HOLITECH,
		"holitech_s5k4h7_131", "holitech_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_GOERTEK,
		"goertek_s5k4h7_131", "goertek_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_SHINETECH,
		"shinetech_s5k4h7_131", "shinetech_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_SUNWIN,
		"sunwin_s5k4h7_131", "sunwin_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_JSL,
		"jsl_s5k4h7_131", "jsl_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_UNION,
		"union_s5k4h7_131", "union_s5k4h7_131", NULL},
	{ S5K4H7_131_SENSOR_INFO_MODULE_ID_SEASIONS,
		"seasons_s5k4h7_131", "seasons_s5k4h7_131", NULL},
};

#define S5K4H7_131_GROUP_ONE_MASK		0xC0
#define S5K4H7_131_GROUP_ONE_VALUE		0x40
#define S5K4H7_131_GROUP_TWO_MASK		0x30
#define S5K4H7_131_GROUP_TWO_VALUE		0x10
#define S5K4H7_131_GROUP_INVLID_VALUE	-1

#define S5K4H7_131_ID_ABS_ADDR 0x7011

#define S5K4H7_131_MODULE_INFO_GOUP_VALID_ABS_ADDR 0x7010
#define S5K4H7_131_MODULE_INFO_GOUP_OFF_SIZE 0x8

#define MI_S_ABS_ADDR 0x7011
#define MI_E_ABS_ADDR 0x7018
#define S5K4H7_131_MODULE_INFO_SIZE (MI_E_ABS_ADDR - MI_S_ABS_ADDR + 1)


#define S5K4H7_131_LSC_GOUP_VALID_ABS_ADDR 0x7028
#define S5K4H7_131_LSC_GOUP_OFF_SIZE 0xF1

#define CS_LSC_S_ABS_ADDR 0x7029
#define CS_LSC_E_ABS_ADDR 0x7118
#define S5K4H7_131_LSC_SIZE (CS_LSC_E_ABS_ADDR - CS_LSC_S_ABS_ADDR + 1)


#define MI_GROUP_ADDR 0x0
#define LSC_GROUP_ADDR (MI_GROUP_ADDR + 1 + S5K4H7_131_MODULE_INFO_SIZE)

#define CS_LSC_S_ADDR (LSC_GROUP_ADDR + 1)
#define CS_LSC_E_ADDR (CS_LSC_S_ADDR + S5K4H7_131_LSC_SIZE - 1)
#define CS_LSC_ADDR (CS_LSC_E_ADDR + 1)


#define S5K4H7_131_MODULEINFO_VALID1 (0)
#define S5K4H7_131_MODULEINFO_VALID2 (0xA)
#define S5K4H7_131_MODULEINFO_VALID3 (0x14)

#define VALID   0x01
#define INVALID 0x03


void s5k4h7_131_parse_module_name(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t sensor_module_id = 0;

	if (e_ctrl->cal_data.mapdata[S5K4H7_131_MODULEINFO_VALID1] == VALID) {
		sensor_module_id = e_ctrl->cal_data.mapdata[S5K4H7_131_MODULEINFO_VALID1+1];
	} else if (e_ctrl->cal_data.mapdata[S5K4H7_131_MODULEINFO_VALID2] == VALID) {
	    sensor_module_id = e_ctrl->cal_data.mapdata[S5K4H7_131_MODULEINFO_VALID2+1];
	} else if (e_ctrl->cal_data.mapdata[S5K4H7_131_MODULEINFO_VALID3] == VALID) {
	    sensor_module_id = e_ctrl->cal_data.mapdata[S5K4H7_131_MODULEINFO_VALID3+1];
	}

	parse_module_name(&(e_ctrl->module_info[0]), S5K4H7_131_MODULE_MAP,
		sizeof(S5K4H7_131_MODULE_MAP) / sizeof(MODULE_Map_Table),
		sensor_module_id);
}


static int s5k4h7_131_checksum_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc = 0;

	return rc;
}

int s5k4h7_131_read_eeprom_init(struct msm_eeprom_ctrl_t *e_ctrl, uint16_t groupid)
{
	int rc =  0;

	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		 0x0100, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(50);

	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		 0x0A02, (groupid+21), MSM_CAMERA_I2C_BYTE_DATA);

	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		  0x0A00, 0x01, MSM_CAMERA_I2C_BYTE_DATA);

	msleep(55);
	return rc;
}

void s5k4h7_131_read_eeprom_end(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc =  0;

	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x0A00, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
}

int s5k4h7_131_user_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_array *eeprom_map_array)
{
	int rc =  0, i, j;
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
		for (i = 0; i < eeprom_map->memory_map_size; i++) {
		   rc = s5k4h7_131_read_eeprom_init(e_ctrl, i);
		   if (rc < 0) {
			  pr_err("%s:%d read failed\n", __func__, __LINE__);
			  return rc;
		    }
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
					pr_err("%s:%d read failed\n", __func__, __LINE__);
					return rc;
				}
				memptr += eeprom_map->mem_settings[i].reg_data;
			}
			break;
			default:
				pr_err("%s: %d Invalid i2c operation LC:%d\n",
					__func__, __LINE__, i);
				return rc;
			}
		    s5k4h7_131_read_eeprom_end(e_ctrl);
		}
	}

	return rc;
}

static struct zte_eeprom_fn_t s5k4h7_131_eeprom_func_tbl = {
	.eeprom_parse_map = NULL,
	.kernel_read_eeprom_memory = NULL,
	.user_read_eeprom_memory = s5k4h7_131_user_read_eeprom_memory,
	.eeprom_match_crc = NULL,
	.eeprom_checksum = s5k4h7_131_checksum_eeprom,
	.validflag_check_eeprom = NULL,
	.parse_module_name = s5k4h7_131_parse_module_name,
};

static const struct of_device_id s5k4h7_131_eeprom_dt_match[] = {
	{ .compatible = "zte,s5k4h7_131-eeprom", .data = &s5k4h7_131_eeprom_func_tbl},
};
MODULE_DEVICE_TABLE(of, common_eeprom_dt_match);

static int s5k4h7_131_eeprom_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(s5k4h7_131_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_probe_user(pdev, match);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static int s5k4h7_131_eeprom_platform_remove(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int32_t rc = 0;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(s5k4h7_131_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_remove(pdev);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static struct platform_driver s5k4h7_131_eeprom_platform_driver = {
	.driver = {
		.name = "zte,s5k4h7_131-eeprom",
		.owner = THIS_MODULE,
		.of_match_table = s5k4h7_131_eeprom_dt_match,
	},
	.probe = s5k4h7_131_eeprom_platform_probe,
	.remove = s5k4h7_131_eeprom_platform_remove,
};

static int __init s5k4h7_131_eeprom_init_module(void)
{
	int rc = 0;

	pr_info("%s E\n", __func__);
	rc = platform_driver_register(&s5k4h7_131_eeprom_platform_driver);
	pr_info("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static void __exit s5k4h7_131_eeprom_exit_module(void)
{
	platform_driver_unregister(&s5k4h7_131_eeprom_platform_driver);

}

module_init(s5k4h7_131_eeprom_init_module);
module_exit(s5k4h7_131_eeprom_exit_module);
MODULE_DESCRIPTION("ZTE EEPROM driver");
MODULE_LICENSE("GPL v2");

