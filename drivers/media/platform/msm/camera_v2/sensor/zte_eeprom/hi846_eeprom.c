

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "zte_eeprom.h"

/*#define HI846_EEPROM_DEBUG */
#undef CDBG
#ifdef HI846_EEPROM_DEBUG
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define HI846_SENSOR_INFO_MODULE_ID_SUNNY		0x01
#define HI846_SENSOR_INFO_MODULE_ID_TRULY		0x02
#define HI846_SENSOR_INFO_MODULE_ID_A_KERR		0x03
#define HI846_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define HI846_SENSOR_INFO_MODULE_ID_DARLING		0x05
#define HI846_SENSOR_INFO_MODULE_ID_QTECH		0x06
#define HI846_SENSOR_INFO_MODULE_ID_OFLIM		0x07
#define HI846_SENSOR_INFO_MODULE_ID_FOXCONN		0x11
#define HI846_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define HI846_SENSOR_INFO_MODULE_ID_ALTEK		0x13
#define HI846_SENSOR_INFO_MODULE_ID_ABICO		0x14
#define HI846_SENSOR_INFO_MODULE_ID_LITE_ON		0x15
#define HI846_SENSOR_INFO_MODULE_ID_CHICONY		0x16
#define HI846_SENSOR_INFO_MODULE_ID_PRIMAX		0x17
#define HI846_SENSOR_INFO_MODULE_ID_SHARP		0x21
#define HI846_SENSOR_INFO_MODULE_ID_MCNEX		0x31
#define HI846_SENSOR_INFO_MODULE_ID_HOLITECH	0x42
#define HI846_SENSOR_INFO_MODULE_ID_GOERTEK		0x54
#define HI846_SENSOR_INFO_MODULE_ID_SHINETECH	0x55
#define HI846_SENSOR_INFO_MODULE_ID_SUNWIN		0x56
#define HI846_SENSOR_INFO_MODULE_ID_JSL			0x57
#define HI846_SENSOR_INFO_MODULE_ID_UNION		0x58
#define HI846_SENSOR_INFO_MODULE_ID_SEASIONS	0x59

MODULE_Map_Table HI846_MODULE_MAP[] = {
	{ HI846_SENSOR_INFO_MODULE_ID_SUNNY,
		"sunny_hi846", "sunny_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_TRULY,
		"truly_hi846", "truly_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_A_KERR,
		"a_kerr_hi846", "a_kerr_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_LITEARRAY,
		"litearray_hi846", "litearray_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_DARLING,
		"darling_hi846", "darling_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_QTECH,
		"qtech_hi846", "qtech_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_OFLIM,
		"oflim_hi846", "oflim_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_FOXCONN,
		"foxconn_hi846", "foxconn_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_IMPORTEK,
		"importek_hi846", "importek_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_ALTEK,
		"altek_hi846", "altek_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_ABICO,
		"abico_hi846", "abico_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_LITE_ON,
		"lite_on_hi846", "lite_on_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_CHICONY,
		"chicony_hi846", "chicony_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_PRIMAX,
		"primax_hi846", "primax_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_SHARP,
		"sharp_hi846", "sharp_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_MCNEX,
		"mcnex_hi846", "mcnex_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_HOLITECH,
		"holitech_hi846", "holitech_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_GOERTEK,
		"goertek_hi846", "goertek_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_SHINETECH,
		"shinetech_hi846", "shinetech_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_SUNWIN,
		"sunwin_hi846", "sunwin_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_JSL,
		"jsl_hi846", "jsl_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_UNION,
		"union_hi846", "union_hi846", NULL},
	{ HI846_SENSOR_INFO_MODULE_ID_SEASIONS,
		"seasons_hi846", "seasons_hi846", NULL},
};

#define HI846_GROUP_ONE_VALUE 0x01
#define HI846_GROUP_TWO_VALUE 0x13
#define HI846_GROUP_THRESS_VALUE 0x37


#define HI846_START_ADDR 0x201
#define HI846_MODULE_INFO_GOUP_VALID_ABS_ADDR 0x201
#define HI846_MODULE_INFO_GOUP_OFF_SIZE 0x11


#define CS_MI_S_ABS_ADDR 0x0202
#define CS_MI_E_ABS_ADDR 0x0208

#define HI846_AWB_GOUP_VALID_ABS_ADDR 0xC5F
#define HI846_AWB_GOUP_OFF_SIZE 0x1E

#define CS_AWB_S_ABS_ADDR 0xC60
#define CS_AWB_E_ABS_ADDR 0xC6B


#define CS_MI_S_ADDR (CS_MI_S_ABS_ADDR - HI846_MODULE_INFO_GOUP_VALID_ABS_ADDR)
#define CS_MI_E_ADDR (CS_MI_S_ADDR + (CS_MI_E_ABS_ADDR - CS_MI_S_ABS_ADDR))
#define CS_MI_CHECK_ADDR (CS_MI_E_ADDR + 1)

#define CS_AWB_S_ADDR (CS_MI_CHECK_ADDR + 2)
#define CS_AWB_E_ADDR (CS_AWB_S_ADDR + (CS_AWB_E_ABS_ADDR - CS_AWB_S_ABS_ADDR))
#define CS_AWB_CHECK_ADDR (CS_AWB_E_ADDR + 1)


#define HI846_ID_ADDR CS_MI_S_ADDR


void hi846_parse_module_name(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t sensor_module_id = e_ctrl->cal_data.mapdata[HI846_ID_ADDR];

	parse_module_name(&(e_ctrl->module_info[0]), HI846_MODULE_MAP,
		sizeof(HI846_MODULE_MAP) / sizeof(MODULE_Map_Table),
		sensor_module_id);
}

static int  hi846_get_group_id(uint8_t data)
{
	int rc = Invlid_Group;

	switch (data) {
	case HI846_GROUP_ONE_VALUE:
		rc = Group_One;
		break;
	case HI846_GROUP_TWO_VALUE:
		rc = Group_Two;
		break;
	case HI846_GROUP_THRESS_VALUE:
		rc = Group_Three;
		break;
	default:
		pr_info("%s :%d: group id  = 0x%x invlid", __func__,
			__LINE__, data);
		break;
	}
	return rc;
}

static int  hi846_validgroup_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  rc, flag = 0;

	rc = hi846_get_group_id(e_ctrl->cal_data.mapdata[CS_MI_S_ADDR - 1]);
	if (rc == Invlid_Group) {
		pr_err("%s :%d: Module info group invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[CS_MI_S_ADDR - 1]);
		flag |= 0x1;
	}

	rc = hi846_get_group_id(e_ctrl->cal_data.mapdata[CS_AWB_S_ADDR - 1]);
	if (rc == Invlid_Group) {
		pr_err("%s :%d: AWB group invalid 0x%x\n",
			__func__, __LINE__, e_ctrl->cal_data.mapdata[CS_AWB_S_ADDR - 1]);
		flag |= 0x2;
	}

	pr_info("%s :%d: group id valid = 0x%x  %s\n",
		__func__, __LINE__, flag, (flag == 0) ? "true" : "false");

	return flag;
}

static int hi846_checksum_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int  checksum = 0;
	int j;
	int rc = 0;

	for (j = CS_MI_S_ADDR; j <= CS_MI_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];
	if ((checksum % 0xff)+1 != e_ctrl->cal_data.mapdata[CS_MI_CHECK_ADDR]) {
		pr_err("%s :%d: module info checksum fail", __func__, __LINE__);
		rc  |= 0x1;
	}
	checksum = 0;

	for (j = CS_AWB_S_ADDR; j <= CS_AWB_E_ADDR; j++)
		checksum += e_ctrl->cal_data.mapdata[j];
	if ((checksum % 0xff)+1 != e_ctrl->cal_data.mapdata[CS_AWB_CHECK_ADDR]) {
		pr_err("%s :%d: awb info checksum fail", __func__, __LINE__);
		rc  |= 0x2;
	}

	pr_info("%s :%d: cal info checksum rc = 0x%x %s\n",
			__func__, __LINE__, rc, (rc == 0) ? "true" : "false");
	return rc;
}

struct msm_camera_i2c_reg_array hi846_readotp_init_regval[] = {
	{0x2000, 0x100A, 0x00},
	{0x2002, 0x00FF, 0x00},
	{0x2004, 0x0007, 0x00},
	{0x2006, 0x3FFF, 0x00},
	{0x2008, 0x3FFF, 0x00},
	{0x200A, 0xC216, 0x00},
	{0x200C, 0x1292, 0x00},
	{0x200E, 0xC01A, 0x00},
	{0x2010, 0x403D, 0x00},
	{0x2012, 0x000E, 0x00},
	{0x2014, 0x403E, 0x00},
	{0x2016, 0x0B80, 0x00},
	{0x2018, 0x403F, 0x00},
	{0x201A, 0x82AE, 0x00},
	{0x201C, 0x1292, 0x00},
	{0x201E, 0xC00C, 0x00},
	{0x2020, 0x4130, 0x00},
	{0x2022, 0x43E2, 0x00},
	{0x2024, 0x0180, 0x00},
	{0x2026, 0x4130, 0x00},
	{0x2028, 0x7400, 0x00},
	{0x202A, 0x5000, 0x00},
	{0x202C, 0x0253, 0x00},
	{0x202E, 0x0AD1, 0x00},
	{0x2030, 0x2360, 0x00},
	{0x2032, 0x0009, 0x00},
	{0x2034, 0x5020, 0x00},
	{0x2036, 0x000B, 0x00},
	{0x2038, 0x0002, 0x00},
	{0x203A, 0x0044, 0x00},
	{0x203C, 0x0016, 0x00},
	{0x203E, 0x1792, 0x00},
	{0x2040, 0x7002, 0x00},
	{0x2042, 0x154F, 0x00},
	{0x2044, 0x00D5, 0x00},
	{0x2046, 0x000B, 0x00},
	{0x2048, 0x0019, 0x00},
	{0x204A, 0x1698, 0x00},
	{0x204C, 0x000E, 0x00},
	{0x204E, 0x099A, 0x00},
	{0x2050, 0x0058, 0x00},
	{0x2052, 0x7000, 0x00},
	{0x2054, 0x1799, 0x00},
	{0x2056, 0x0310, 0x00},
	{0x2058, 0x03C3, 0x00},
	{0x205A, 0x004C, 0x00},
	{0x205C, 0x064A, 0x00},
	{0x205E, 0x0001, 0x00},
	{0x2060, 0x0007, 0x00},
	{0x2062, 0x0BC7, 0x00},
	{0x2064, 0x0055, 0x00},
	{0x2066, 0x7000, 0x00},
	{0x2068, 0x1550, 0x00},
	{0x206A, 0x158A, 0x00},
	{0x206C, 0x0004, 0x00},
	{0x206E, 0x1488, 0x00},
	{0x2070, 0x7010, 0x00},
	{0x2072, 0x1508, 0x00},
	{0x2074, 0x0004, 0x00},
	{0x2076, 0x0016, 0x00},
	{0x2078, 0x03D5, 0x00},
	{0x207A, 0x0055, 0x00},
	{0x207C, 0x08CA, 0x00},
	{0x207E, 0x2019, 0x00},
	{0x2080, 0x0007, 0x00},
	{0x2082, 0x7057, 0x00},
	{0x2084, 0x0FC7, 0x00},
	{0x2086, 0x5041, 0x00},
	{0x2088, 0x12C8, 0x00},
	{0x208A, 0x5060, 0x00},
	{0x208C, 0x5080, 0x00},
	{0x208E, 0x2084, 0x00},
	{0x2090, 0x12C8, 0x00},
	{0x2092, 0x7800, 0x00},
	{0x2094, 0x0802, 0x00},
	{0x2096, 0x040F, 0x00},
	{0x2098, 0x1007, 0x00},
	{0x209A, 0x0803, 0x00},
	{0x209C, 0x080B, 0x00},
	{0x209E, 0x3803, 0x00},
	{0x20A0, 0x0807, 0x00},
	{0x20A2, 0x0404, 0x00},
	{0x20A4, 0x0400, 0x00},
	{0x20A6, 0xFFFF, 0x00},
	{0x20A8, 0xF0B2, 0x00},
	{0x20AA, 0xFFEF, 0x00},
	{0x20AC, 0x0A84, 0x00},
	{0x20AE, 0x1292, 0x00},
	{0x20B0, 0xC02E, 0x00},
	{0x20B2, 0x4130, 0x00},
	{0x23FE, 0xC056, 0x00},
	{0x3232, 0xFC0C, 0x00},
	{0x3236, 0xFC22, 0x00},
	{0x3248, 0xFCA8, 0x00},
	{0x326A, 0x8302, 0x00},
	{0x326C, 0x830A, 0x00},
	{0x326E, 0x0000, 0x00},
	{0x32CA, 0xFC28, 0x00},
	{0x32CC, 0xC3BC, 0x00},
	{0x32CE, 0xC34C, 0x00},
	{0x32D0, 0xC35A, 0x00},
	{0x32D2, 0xC368, 0x00},
	{0x32D4, 0xC376, 0x00},
	{0x32D6, 0xC3C2, 0x00},
	{0x32D8, 0xC3E6, 0x00},
	{0x32DA, 0x0003, 0x00},
	{0x32DC, 0x0003, 0x00},
	{0x32DE, 0x00C7, 0x00},
	{0x32E0, 0x0031, 0x00},
	{0x32E2, 0x0031, 0x00},
	{0x32E4, 0x0031, 0x00},
	{0x32E6, 0xFC28, 0x00},
	{0x32E8, 0xC3BC, 0x00},
	{0x32EA, 0xC384, 0x00},
	{0x32EC, 0xC392, 0x00},
	{0x32EE, 0xC3A0, 0x00},
	{0x32F0, 0xC3AE, 0x00},
	{0x32F2, 0xC3C4, 0x00},
	{0x32F4, 0xC3E6, 0x00},
	{0x32F6, 0x0003, 0x00},
	{0x32F8, 0x0003, 0x00},
	{0x32FA, 0x00C7, 0x00},
	{0x32FC, 0x0031, 0x00},
	{0x32FE, 0x0031, 0x00},
	{0x3300, 0x0031, 0x00},
	{0x3302, 0x82CA, 0x00},
	{0x3304, 0xC164, 0x00},
	{0x3306, 0x82E6, 0x00},
	{0x3308, 0xC19C, 0x00},
	{0x330A, 0x001F, 0x00},
	{0x330C, 0x001A, 0x00},
	{0x330E, 0x0034, 0x00},
	{0x3310, 0x0000, 0x00},
	{0x3312, 0x0000, 0x00},
	{0x3314, 0xFC94, 0x00},
	{0x3316, 0xC3D8, 0x00},
	{0x0A00, 0x0000, 0x00},
	{0x0E04, 0x0012, 0x00},
	{0x002E, 0x1111, 0x00},
	{0x0032, 0x1111, 0x00},
	{0x0022, 0x0008, 0x00},
	{0x0026, 0x0040, 0x00},
	{0x0028, 0x0017, 0x00},
	{0x002C, 0x09CF, 0x00},
	{0x005C, 0x2101, 0x00},
	{0x0006, 0x09DE, 0x00},
	{0x0008, 0x0ED8, 0x00},
	{0x000E, 0x0100, 0x00},
	{0x000C, 0x0022, 0x00},
	{0x0A22, 0x0000, 0x00},
	{0x0A24, 0x0000, 0x00},
	{0x0804, 0x0000, 0x00},
	{0x0A12, 0x0CC0, 0x00},
	{0x0A14, 0x0990, 0x00},
	{0x0074, 0x09D8, 0x00},
	{0x0076, 0x0000, 0x00},
	{0x051E, 0x0000, 0x00},
	{0x0200, 0x0400, 0x00},
	{0x0A1A, 0x0C00, 0x00},
	{0x0A0C, 0x0010, 0x00},
	{0x0A1E, 0x0CCF, 0x00},
	{0x0402, 0x0110, 0x00},
	{0x0404, 0x00F4, 0x00},
	{0x0408, 0x0000, 0x00},
	{0x0410, 0x008D, 0x00},
	{0x0412, 0x011A, 0x00},
	{0x0414, 0x864C, 0x00},
	{0x021C, 0x0001, 0x00},
	{0x0C00, 0x9150, 0x00},
	{0x0C06, 0x0021, 0x00},
	{0x0C10, 0x0040, 0x00},
	{0x0C12, 0x0040, 0x00},
	{0x0C14, 0x0040, 0x00},
	{0x0C16, 0x0040, 0x00},
	{0x0A02, 0x0100, 0x00},
	{0x0A04, 0x014A, 0x00},
	{0x0418, 0x0000, 0x00},
	{0x012A, 0x03B4, 0x00},
	{0x0120, 0x0046, 0x00},
	{0x0122, 0x0376, 0x00},
	{0x0B02, 0xE04D, 0x00},
	{0x0B10, 0x6821, 0x00},
	{0x0B12, 0x0120, 0x00},
	{0x0B14, 0x0001, 0x00},
	{0x2008, 0x38FD, 0x00},
	{0x326E, 0x0000, 0x00},
	{0x0900, 0x0300, 0x00},
	{0x0902, 0xC319, 0x00},
	{0x0914, 0xC109, 0x00},
	{0x0916, 0x061A, 0x00},
	{0x0918, 0x0407, 0x00},
	{0x091A, 0x0A0B, 0x00},
	{0x091C, 0x0E08, 0x00},
	{0x091E, 0x0A00, 0x00},
	{0x090C, 0x0427, 0x00},
	{0x090E, 0x0069, 0x00},
	{0x0954, 0x0089, 0x00},
	{0x0956, 0x0000, 0x00},
	{0x0958, 0xCA80, 0x00},
	{0x095A, 0x9240, 0x00},
	{0x0F08, 0x2F04, 0x00},
	{0x0F30, 0x001F, 0x00},
	{0x0F36, 0x001F, 0x00},
	{0x0F04, 0x3A00, 0x00},
	{0x0F32, 0x025A, 0x00},
	{0x0F38, 0x025A, 0x00},
	{0x0F2A, 0x4124, 0x00},
	{0x006A, 0x0100, 0x00},
	{0x004C, 0x0100, 0x00},
};

struct msm_camera_i2c_reg_setting hi846_otp_read_init_setting = {
	.reg_setting = hi846_readotp_init_regval,
	.size = ARRAY_SIZE(hi846_readotp_init_regval),
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	.data_type = MSM_CAMERA_I2C_WORD_DATA,
	.delay = 2,
};

struct msm_camera_i2c_reg_array hi846_readotp_init_regval_start[] = {
	{0x0a02, 0x01, 0x0000},
	{0x0a00, 0x00, 0x000a},
	{0x0f02, 0x00, 0x0000},
	{0x071a, 0x01, 0x0000},
	{0x071b, 0x09, 0x0000},
	{0x0d04, 0x01, 0x0000},
	{0x0d00, 0x07, 0x0000},
	{0x003e, 0x10, 0x0000},
	{0x070f, 0x05, 0x0000},
	{0x0a00, 0x01, 0x0000},

	{0x70a, 0x2, 0},
	{0x70b, 0x1, 0},
	{0x702, 0x1, 0},
};
struct msm_camera_i2c_reg_setting hi846_otp_read_init_setting_start = {
	.reg_setting = hi846_readotp_init_regval_start,
	.size = ARRAY_SIZE(hi846_readotp_init_regval_start),
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 2,
};

int hi846_read_eeprom_init(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc =  0;
	return rc;
}

int hi846_user_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_array *eeprom_map_array)
{
	int rc =  0, i, j, g, group_off;
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
		pr_info("Memory map Size: %d", eeprom_map->memory_map_size);

		e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&(e_ctrl->i2c_client),
			&hi846_otp_read_init_setting);
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&(e_ctrl->i2c_client),
			&hi846_otp_read_init_setting_start);
		rc = hi846_read_eeprom_init(e_ctrl);
		if (rc < 0) {
			pr_info("%s: read failed\n", __func__);
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
				e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client), 0x070a,
					((eeprom_map->mem_settings[i].reg_addr>>8)&0xff), MSM_CAMERA_I2C_BYTE_DATA);
				e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client), 0x070b,
					((eeprom_map->mem_settings[i].reg_addr)&0xff), MSM_CAMERA_I2C_BYTE_DATA);
				e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client), 0x0702,
					0x01, MSM_CAMERA_I2C_BYTE_DATA);
				for (g = 0; g < eeprom_map->mem_settings[i].reg_data; g++) {
					e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(&(e_ctrl->i2c_client), 0x0708,
						memptr, eeprom_map->mem_settings[i].reg_data);
					CDBG("%s: goup info 0x%x\n", __func__, *memptr);
					memptr++;
				}
				memptr -= eeprom_map->mem_settings[i].reg_data;
				if (rc < 0) {
					pr_err("%s: read failed\n",
						__func__);
					return rc;
				}
				if ((eeprom_map->mem_settings[i].reg_addr
					== HI846_MODULE_INFO_GOUP_VALID_ABS_ADDR)
					&& ((i + 2) <= eeprom_map->memory_map_size)) {
					group_off = hi846_get_group_id(*memptr);
					if (group_off == Invlid_Group) {
						pr_err("%s: 0x%x: 0x%x group invlid\n", __func__,
							eeprom_map->mem_settings[i].reg_addr, *memptr);
					} else {
						CDBG("%s: 0x%x: 0x%x group %d\n", __func__,
							eeprom_map->mem_settings[i].reg_addr, *memptr, group_off);
						group_off -= Group_One;
						eeprom_map->mem_settings[i + 1].reg_addr
							+= HI846_MODULE_INFO_GOUP_OFF_SIZE * group_off;
						CDBG("%s: 0x%x: 0x%x group %d\n", __func__,
							eeprom_map->mem_settings[i + 1].reg_addr, *memptr, group_off);
						eeprom_map->mem_settings[i + 2].reg_addr
							+= HI846_MODULE_INFO_GOUP_OFF_SIZE * group_off;
					}
				}
				if ((eeprom_map->mem_settings[i].reg_addr
					== HI846_AWB_GOUP_VALID_ABS_ADDR)
					&& ((i + 2) <= eeprom_map->memory_map_size)) {
					group_off = hi846_get_group_id(*memptr);
					if (group_off == Invlid_Group) {
						pr_err("%s: 0x%x: 0x%x group invlid\n", __func__,
							eeprom_map->mem_settings[i].reg_addr, *memptr);
					} else {
						CDBG("%s: 0x%x: 0x%x group %d\n", __func__,
							eeprom_map->mem_settings[i].reg_addr, *memptr, group_off);
						group_off -= Group_One;
						eeprom_map->mem_settings[i + 1].reg_addr
							+= HI846_AWB_GOUP_OFF_SIZE * group_off;
						CDBG("%s: 0x%x: 0x%x group %d\n", __func__,
							eeprom_map->mem_settings[i + 1].reg_addr, *memptr, group_off);
						eeprom_map->mem_settings[i + 2].reg_addr
							+= HI846_AWB_GOUP_OFF_SIZE * group_off;
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
	return rc;
}

static struct zte_eeprom_fn_t hi846_eeprom_func_tbl = {
	.eeprom_parse_map = NULL,
	.kernel_read_eeprom_memory = NULL,
	.user_read_eeprom_memory = hi846_user_read_eeprom_memory,
	.eeprom_match_crc = NULL,
	.eeprom_checksum = hi846_checksum_eeprom,
	.validflag_check_eeprom = hi846_validgroup_check_eeprom,
	.parse_module_name = hi846_parse_module_name,
};

static const struct of_device_id hi846_eeprom_dt_match[] = {
	{ .compatible = "zte,hi846-eeprom", .data = &hi846_eeprom_func_tbl},
};
MODULE_DEVICE_TABLE(of, common_eeprom_dt_match);

static int hi846_eeprom_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(hi846_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_probe_user(pdev, match);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static int hi846_eeprom_platform_remove(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int32_t rc = 0;

	pr_info("%s:%d E", __func__, __LINE__);
	match = of_match_device(hi846_eeprom_dt_match, &pdev->dev);
	if (match)
		rc = zte_eeprom_platform_remove(pdev);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_info("%s:%d X", __func__, __LINE__);
	return rc;
}

static struct platform_driver hi846_eeprom_platform_driver = {
	.driver = {
		.name = "zte,hi846-eeprom",
		.owner = THIS_MODULE,
		.of_match_table = hi846_eeprom_dt_match,
	},
	.probe = hi846_eeprom_platform_probe,
	.remove = hi846_eeprom_platform_remove,
};

static int __init hi846_eeprom_init_module(void)
{
	int rc = 0;

	pr_info("%s E\n", __func__);
	rc = platform_driver_register(&hi846_eeprom_platform_driver);
	pr_info("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static void __exit hi846_eeprom_exit_module(void)
{
	platform_driver_unregister(&hi846_eeprom_platform_driver);

}

module_init(hi846_eeprom_init_module);
module_exit(hi846_eeprom_exit_module);
MODULE_DESCRIPTION("ZTE EEPROM driver");
MODULE_LICENSE("GPL v2");

