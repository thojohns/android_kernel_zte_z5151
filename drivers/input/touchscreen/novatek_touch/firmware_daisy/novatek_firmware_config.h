
/************************************************************************
*
* File Name: novatek_firmware_config.h
*
*  *   Version: v1.0
*
************************************************************************/
#ifndef _NOVATEK_FIRMWARE_CONFIG_H_
#define _NOVATEK_FIRMWARE_CONFIG_H_

/********************** Upgrade ***************************

  auto upgrade, please keep enable
*********************************************************/
#define BOOT_UPDATE_FIRMWARE 1
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#define WAKEUP_GESTURE 0

/* ESD Protect */
#define NVT_TOUCH_ESD_PROTECT 1
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500 /* ms */

#define NVT_CHARGER_SWITCH 1

#define TOUCH_DEFAULT_MAX_WIDTH 1080
#define TOUCH_DEFAULT_MAX_HEIGHT 1920
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0

/*
 * Check vendor_id number
 * 0:No check vendor_id (default)
 * 1/2/3: Check vendor_id for vendor compatibility
 */
#define NVT_GET_VENDOR_ID_NUM                   4

/* id[1:0]: 0x00 0x01 0x10 0x11 */
#define NVT_VENDOR_ID_0 0
#define NVT_VENDOR_ID_1 1
#define NVT_VENDOR_ID_2 2
#define NVT_VENDOR_ID_3 3
/*
 * vendor_id(s) for vendor(s) to be compatible with.
 * a confirmation of vendor_id(s) is recommended.
 * NVT_GET_VENDOR_ID_NUM == 0, no check vendor id, you may ignore them
 * NVT_GET_VENDOR_ID_NUM >= 1, compatible with NVT_VENDOR_1_ID
 * NVT_GET_VENDOR_ID_NUM >= 2, compatible with NVT_VENDOR_2_ID
 * NVT_GET_VENDOR_ID_NUM == 3, compatible with NVT_VENDOR_3_ID
 */
#define NVT_VENDOR_0_NAME                         "Truly"
#define NVT_VENDOR_1_NAME                         "unknown"
#define NVT_VENDOR_2_NAME                         "unknown"
#define NVT_VENDOR_3_NAME                         "unknown"
#define NVT_VENDOR_DEFAULT_NAME                   "unknown"
/*
 * FW_APP.i file for auto upgrade, you must replace it with your own
 * define your own fw_app, the sample one to be replaced is invalid
 * NOTE: if NVT_GET_VENDOR_ID_NUM >= 1, it's the fw corresponding with NVT_VENDOR_ID_0
 */
#define NVT_UPGRADE_FW0_APP                      "ZTE_Z610DL_TRULY_5P99_1080_2160_NT36672_62323037.bin"

/*
 * if NVT_GET_VENDOR_ID_NUM >= 2, fw corrsponding with NVT_VENDOR_ID_1
 * define your own fw_app, the sample one is invalid
 */
#define NVT_UPGRADE_FW1_APP                     "nvt_empty.bin"

/*
 * if NVT_GET_VENDOR_ID_NUM == 3, fw corrsponding with NVT_VENDOR_ID_2
 * define your own fw_app, the sample one is invalid
 */
#define NVT_UPGRADE_FW2_APP                     "nvt_empty.bin"

/*
 * if NVT_GET_VENDOR_ID_NUM == 3, fw corrsponding with NVT_VENDOR_ID_3
 * define your own fw_app, the sample one is invalid
 */
#define NVT_UPGRADE_FW3_APP                     "nvt_empty.bin"

#endif
