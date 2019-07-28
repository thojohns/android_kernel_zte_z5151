#ifndef __BQFS_CONFIG_H__
#define __BQFS_CONFIG_H__

#include "bqfs_cmd_type.h"

#ifdef CONFIG_BATTERY_NJ_2500_ID1M_TMB903_366071
#include "bqfs_file/batt_nj_2500_id1m_tmb903_366071.h"
#endif
#ifdef CONFIG_BATTERY_4870_SCUD_066_486488
#include "bqfs_file/batt_4870_scud_066_486488.h"
#endif
#ifdef CONFIG_BATTERY_Li3931T44P8h806139_386279_IBTB_3100MAH_4400MV
#include "bqfs_file/battery_data_Li3931T44P8h806139_386279_IBTB_3100mAh_4400mV.h"
#endif
#ifdef CONFIG_BATTERY_Li3931T44P8h806139_386279_COS_3100MAH_4400MV
#include "bqfs_file/battery_data_Li3931T44P8h806139_386279_COS_3100mAh_4400mV.h"
#endif
#ifdef CONFIG_BATTERY_Li3930T44P8h826636_346679_COS_3010MAH_4400MV
#include "bqfs_file/battery_data_Li3930T44P8h826636_346679_COS_3010mAh_4400mV.h"
#endif
#ifdef CONFIG_BATTERY_450_18_3900_ID10K_JD_ATL436383
#include "bqfs_file/batt_450_18_3900_id10k_jd_atl436383.h"
#endif
#ifdef CONFIG_BATTERY_450_18_3900_ID20K_GY_GY436483
#include "bqfs_file/batt_450_18_3900_id20k_gy_gy436483.h"
#endif
#ifdef CONFIG_BATTERY_450_18_3900_ID390K_TMB1458_TMB436484
#include "bqfs_file/batt_450_18_3900_id390k_tmb1458_tmb436484.h"
#endif

struct bqfs_array {
	bqfs_cmd_t *bqfs_image;
	int array_size;
	int battery_id;
	int battery_full_design;
};

static struct bqfs_array g_bqfs_array[] = {
#ifdef CONFIG_BATTERY_NJ_2500_ID1M_TMB903_366071
	{.bqfs_image = (bqfs_cmd_t *)&batt_nj_2500_id1m_tmb903_366071_image[0],
	.array_size = ARRAY_SIZE(batt_nj_2500_id1m_tmb903_366071_image),
	.battery_id = 1000000,
	.battery_full_design = 2500000 },
#endif
#ifdef CONFIG_BATTERY_4870_SCUD_066_486488
	{.bqfs_image = (bqfs_cmd_t *)&batt_4870_scud_066_486488_image[0],
	.array_size = ARRAY_SIZE(batt_4870_scud_066_486488_image),
	.battery_id = 1650000,
	.battery_full_design = 4870000 },
#endif
#ifdef CONFIG_BATTERY_Li3931T44P8h806139_386279_IBTB_3100MAH_4400MV
	{.bqfs_image = (bqfs_cmd_t *)&battery_data_Li3931T44P8h806139_386279_IBTB_3100mAh_4400mV_image[0],
	.array_size = ARRAY_SIZE(battery_data_Li3931T44P8h806139_386279_IBTB_3100mAh_4400mV_image),
	.battery_id = 390000, /* Rbatt_id = 390k, Rpullup = 100k */
	.battery_full_design = 3100000 },
#endif
#ifdef CONFIG_BATTERY_Li3931T44P8h806139_386279_COS_3100MAH_4400MV
	{.bqfs_image = (bqfs_cmd_t *)&battery_data_Li3931T44P8h806139_386279_COS_3100mAh_4400mV_image[0],
	.array_size = ARRAY_SIZE(battery_data_Li3931T44P8h806139_386279_COS_3100mAh_4400mV_image),
	.battery_id = 20000, /* Rbatt_id = 20k, Rpullup = 100k */
	.battery_full_design = 3100000 },
#endif
#ifdef CONFIG_BATTERY_Li3930T44P8h826636_346679_COS_3010MAH_4400MV
	{.bqfs_image = (bqfs_cmd_t *)&battery_data_Li3930T44P8h826636_346679_COS_3010mAh_4400mV_image[0],
	.array_size = ARRAY_SIZE(battery_data_Li3930T44P8h826636_346679_COS_3010mAh_4400mV_image),
	.battery_id = 20000, /* Rbatt_id = 20k, Rpullup = 100k */
	.battery_full_design = 3010000 },
#endif
#ifdef CONFIG_BATTERY_450_18_3900_ID10K_JD_ATL436383
	{.bqfs_image = (bqfs_cmd_t *)&batt_450_18_3900_id10k_jd_atl436383[0],
	.array_size = ARRAY_SIZE(batt_450_18_3900_id10k_jd_atl436383),
	.battery_id = 10000, /* Rbatt_id = 10k, Rpullup = 100k */
	.battery_full_design = 4000000 },
#endif
#ifdef CONFIG_BATTERY_450_18_3900_ID20K_GY_GY436483
	{.bqfs_image = (bqfs_cmd_t *)&batt_450_18_3900_id20k_gy_gy436483[0],
	.array_size = ARRAY_SIZE(batt_450_18_3900_id20k_gy_gy436483),
	.battery_id = 20000, /* Rbatt_id = 20k, Rpullup = 100k */
	.battery_full_design = 4000000 },
#endif
#ifdef CONFIG_BATTERY_450_18_3900_ID390K_TMB1458_TMB436484
	{.bqfs_image = (bqfs_cmd_t *)&batt_450_18_3900_id390k_tmb1458_tmb436484[0],
	.array_size = ARRAY_SIZE(batt_450_18_3900_id390k_tmb1458_tmb436484),
	.battery_id = 390000, /* Rbatt_id = 390k, Rpullup = 100k */
	.battery_full_design = 4000000 },
#endif
};

#endif
