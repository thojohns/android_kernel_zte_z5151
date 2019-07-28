
/************************************************************************
*
* File Name: himax_firmware_config.h
*
*  *   Version: v1.0
*
************************************************************************/
#ifndef _HIMAX_FIRMWARE_CONFIG_H_
#define _HIMAX_FIRMWARE_CONFIG_H_

/********************** Upgrade ***************************

  auto upgrade, please keep enable
*********************************************************/
#define HX_AUTO_UPDATE_FW
#define FIX_HX_MAX_PT                                10
/*
 * Check vendor_id number
 * 0:No check vendor_id (default)
 * 1/2/3: Check vendor_id for vendor compatibility
 */
#define HX_GET_VENDOR_ID_NUM                   1

#define HX_VENDOR_ID_0 0
#define HX_VENDOR_ID_1 1
#define HX_VENDOR_ID_2 2

/*
 * vendor_id(s) for vendor(s) to be compatible with.
 * a confirmation of vendor_id(s) is recommended.
 * HX_GET_VENDOR_ID_NUM == 0, no check vendor id, you may ignore them
 * HX_GET_VENDOR_ID_NUM >= 1, compatible with HX_VENDOR_1_ID
 * HX_GET_VENDOR_ID_NUM >= 2, compatible with HX_VENDOR_2_ID
 * HX_GET_VENDOR_ID_NUM == 3, compatible with HX_VENDOR_3_ID
 */
#define HXTS_VENDOR_0_NAME                         "Truly"
#define HXTS_VENDOR_1_NAME                         "Lead"
#define HXTS_VENDOR_2_NAME                         "unknown"

/*
 * FW_APP.i file for auto upgrade, you must replace it with your own
 * define your own fw_app, the sample one to be replaced is invalid
 * NOTE: if FTS_GET_VENDOR_ID_NUM >= 1, it's the fw corresponding with FTS_VENDOR_1_ID
 */
#define HXTS_UPGRADE_FW0_APP                      "Z5270T_HiMax83102_Truly.i"

/*
 * if HXTS_GET_VENDOR_ID_NUM >= 2, fw corrsponding with HXTS_VENDOR_2_ID
 * define your own fw_app, the sample one is invalid
 */
#define HXTS_UPGRADE_FW1_APP                     "firmware_default/default_HiMax83102.i"

/*
 * if HXTS_GET_VENDOR_ID_NUM == 3, fw corrsponding with HXTS_VENDOR_3_ID
 * define your own fw_app, the sample one is invalid
 */
#define HXTS_UPGRADE_FW2_APP                     "firmware_default/default_HiMax83102.i"

#endif
