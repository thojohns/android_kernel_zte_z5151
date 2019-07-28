
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
#define HX_AUTO_UPDATE_FW 1
/*#define TOUCH_MAX_FINGER_NUM 10*/
/*#define TOUCH_KEY_NUM 0*/
#define HX_SMART_WAKEUP 1
/*#define CONFIG_TOUCHSCREEN_SPECIAL_INTERFACE*/
#define HX_USB_DETECT_GLOBAL

#define HX_FIX_RX_NUM_0  36    /*unknown*/
#define HX_FIX_RX_NUM_1  34    /*tianma*/
#define HX_FIX_RX_NUM_2  36    /*unknown */
#define HX_FIX_RX_NUM_3  36    /*unknown*/
/*
 * Check vendor_id number
 * 0:No check vendor_id (default)
 * 1/2/3: Check vendor_id for vendor compatibility
 */
#define HX_GET_VENDOR_ID_NUM                   2

/* id[1:0]: 0x00 0x01 0x10 0x11 */
#define HX_VENDOR_ID_0 0
#define HX_VENDOR_ID_1 1
#define HX_VENDOR_ID_2 2
#define HX_VENDOR_ID_3 3
/*
 * vendor_id(s) for vendor(s) to be compatible with.
 * a confirmation of vendor_id(s) is recommended.
 * HX_GET_VENDOR_ID_NUM == 0, no check vendor id, you may ignore them
 * HX_GET_VENDOR_ID_NUM >= 1, compatible with NVT_VENDOR_1_ID
 * HX_GET_VENDOR_ID_NUM >= 2, compatible with NVT_VENDOR_2_ID
 * HX_GET_VENDOR_ID_NUM == 3, compatible with NVT_VENDOR_3_ID
 */
#define HX_VENDOR_0_NAME                         "dijing"
#define HX_VENDOR_1_NAME                         "tianma"
#define HX_VENDOR_2_NAME                         "unknown"
#define HX_VENDOR_3_NAME                         "unknown"
#define HX_VENDOR_DEFAULT_NAME                   "unknown"
/*
 * FW_APP.i file for auto upgrade, you must replace it with your own
 * define your own fw_app, the sample one to be replaced is invalid
 * NOTE: if HX_GET_VENDOR_ID_NUM >= 1, it's the fw corresponding with NVT_VENDOR_ID_0
 */
#define HX_UPGRADE_FW0_APP                      "HX83102A_FM_Version_Dijing.i"

/*
 * if HX_GET_VENDOR_ID_NUM >= 2, fw corrsponding with NVT_VENDOR_ID_1
 * define your own fw_app, the sample one is invalid
 */
#define HX_UPGRADE_FW1_APP                     "HX83102A_FM_Version_Tianma.i"

/*
 * if HX_GET_VENDOR_ID_NUM == 3, fw corrsponding with NVT_VENDOR_ID_2
 * define your own fw_app, the sample one is invalid
 */
#define HX_UPGRADE_FW2_APP                     "HX83102A_FM_Version_Xinli.i"

/*
 * if HX_GET_VENDOR_ID_NUM == 3, fw corrsponding with NVT_VENDOR_ID_3
 * define your own fw_app, the sample one is invalid
 */
#define HX_UPGRADE_FW3_APP                     "HX83102A_FM_Version_Dijing.i"

#endif
