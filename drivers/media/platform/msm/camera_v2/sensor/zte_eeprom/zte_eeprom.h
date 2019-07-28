/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_EEPROM_H
#define MSM_EEPROM_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_spi.h"
#include "msm_camera_io_util.h"
#include "msm_camera_dt_util.h"

struct msm_eeprom_ctrl_t;

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define PROPERTY_MAXSIZE 32

typedef struct {
	uint16_t id;
	const char *sensor_module_name;
	const char *chromtix_lib_name;
	const char *default_chromtix_lib_name;
} MODULE_Map_Table;

typedef enum {
	Invlid_Group,
	Group_One,
	Group_Two,
	Group_Three,
} Group_t;

struct zte_eeprom_fn_t {
	int (*eeprom_parse_map)(struct device_node *of,
				struct msm_eeprom_memory_block_t *data);
	int (*kernel_read_eeprom_memory)(struct msm_eeprom_ctrl_t *e_ctrl,
				struct msm_eeprom_memory_block_t *block);
	int (*user_read_eeprom_memory)(struct msm_eeprom_ctrl_t *e_ctrl,
				struct msm_eeprom_memory_map_array *eeprom_map_array);
	uint32_t (*eeprom_match_crc)(struct msm_eeprom_memory_block_t *data);
	int (*eeprom_checksum)(struct msm_eeprom_ctrl_t *e_ctrl);
	int (*validflag_check_eeprom)(struct msm_eeprom_ctrl_t *e_ctrl);
	void (*parse_module_name)(struct msm_eeprom_ctrl_t *e_ctrl);
};

typedef struct {
	const char *sensor_module_name;
	const char *chromtix_lib_name;
	const char *default_chromtix_lib_name;
} zte_eeprom_module_info_t;

struct msm_eeprom_ctrl_t {
	struct platform_device *pdev;
	struct mutex *eeprom_mutex;

	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *eeprom_v4l2_subdev_ops;
	enum msm_camera_device_type_t eeprom_device_type;
	struct msm_sd_subdev msm_sd;
	enum cci_i2c_master_t cci_master;
	enum i2c_freq_mode_t i2c_freq_mode;

	struct msm_camera_i2c_client i2c_client;
	struct msm_eeprom_board_info *eboard_info;
	uint32_t subdev_id;
	int32_t userspace_probe;
	struct msm_eeprom_memory_block_t cal_data;
	uint8_t is_supported;

	struct zte_eeprom_fn_t *eeprom_fun_p;
	zte_eeprom_module_info_t module_info[2];
	uint32_t checksum;
	uint32_t valid_flag;
	uint32_t share_eeprom;
};

uint32_t msm_eeprom_match_crc(struct msm_eeprom_memory_block_t *data);
int user_eeprom_parse_memory_map(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_array *eeprom_map_array);
void parse_module_name(zte_eeprom_module_info_t *module_info,
	MODULE_Map_Table *map, uint16_t len, uint16_t  sensor_module_id);
int zte_kernel_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_block_t *block);
int zte_kernel_eeprom_parse_memory_map(struct device_node *of,
				struct msm_eeprom_memory_block_t *data);
int common_kernel_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_block_t *block);
int common_user_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_memory_map_array *eeprom_map_array);
int common_eeprom_parse_memory_map(struct device_node *of,
	struct msm_eeprom_memory_block_t *data);
int zte_eeprom_platform_probe_user(struct platform_device *pdev,
	const struct of_device_id *match);
int zte_eeprom_platform_probe_kernel(struct platform_device *pdev,
	const struct of_device_id *match);
int zte_eeprom_platform_remove(struct platform_device *pdev);

#endif
