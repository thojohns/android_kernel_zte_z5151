
/************************************************************************
*
* File Name: focaltech_firmware_config.h
*
*  *   Version: v1.0
*
************************************************************************/
#ifndef _FOCLATECH_FIRMWARE_CONFIG_H_
#define _FOCLATECH_FIRMWARE_CONFIG_H_

/********************** Upgrade ***************************

  auto upgrade, please keep enable
*********************************************************/
#define FTS_AUTO_UPGRADE_EN                     1
/*
 * Check vendor_id number
 * 0:No check vendor_id (default)
 * 1/2/3: Check vendor_id for vendor compatibility
 */
#define FTS_GET_VENDOR_ID_NUM                   0

/*
 * vendor_id(s) for vendor(s) to be compatible with.
 * a confirmation of vendor_id(s) is recommended.
 * FTS_GET_VENDOR_ID_NUM == 0, no check vendor id, you may ignore them
 * FTS_GET_VENDOR_ID_NUM >= 1, compatible with FTS_VENDOR_1_ID
 * FTS_GET_VENDOR_ID_NUM >= 2, compatible with FTS_VENDOR_2_ID
 * FTS_GET_VENDOR_ID_NUM == 3, compatible with FTS_VENDOR_3_ID
 */
#define FTS_VENDOR_1_ID                         0x00
#define FTS_VENDOR_2_ID                         0x00
#define FTS_VENDOR_3_ID                         0x00

/*
 * FW_APP.i file for auto upgrade, you must replace it with your own
 * define your own fw_app, the sample one to be replaced is invalid
 * NOTE: if FTS_GET_VENDOR_ID_NUM >= 1, it's the fw corresponding with FTS_VENDOR_1_ID
 */
#define FTS_UPGRADE_FW_APP                      "include/firmware/FT8716_app_sample.i"

/*
 * if FTS_GET_VENDOR_ID_NUM >= 2, fw corrsponding with FTS_VENDOR_2_ID
 * define your own fw_app, the sample one is invalid
 */
#define FTS_UPGRADE_FW2_APP                     "include/firmware/FT8716_app_sample.i"

/*
 * if FTS_GET_VENDOR_ID_NUM == 3, fw corrsponding with FTS_VENDOR_3_ID
 * define your own fw_app, the sample one is invalid
 */
#define FTS_UPGRADE_FW3_APP                     "include/firmware/FT8716_app_sample.i"

/*
 * lcd_cfg.i file for lcd cfg upgrade
 * define your own lcd_cfg.i, the sample one is invalid
 */
#define FTS_UPGRADE_LCD_CFG                     "include/firmware/lcd_cfg.i"

#endif
