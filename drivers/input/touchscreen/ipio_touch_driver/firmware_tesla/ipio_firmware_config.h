
/************************************************************************
*
* File Name: ipio_firmware_config.h
*
* Version: v1.0
*
************************************************************************/
#ifndef _IPIO_FIRMWARE_CONFIG_H_
#define _IPIO_FIRMWARE_CONFIG_H_

/* An Touch IC currently supported by driver */
#define CHIP_TYPE_ILI7807	0x7807
#define CHIP_TYPE_ILI9881	0x9881
#define TP_TOUCH_IC		CHIP_TYPE_ILI9881

/* define the width and heigth of a screen. */
#define TOUCH_SCREEN_X_MIN 0
#define TOUCH_SCREEN_Y_MIN 0
#define TOUCH_SCREEN_X_MAX 720
#define TOUCH_SCREEN_Y_MAX 1280

/* define the range on panel */
#define TPD_HEIGHT 2048
#define TPD_WIDTH 2048

/* How many numbers of touch are supported by IC. */
#define MAX_TOUCH_NUM	10

/********************** Upgrade ***************************
  auto upgrade, please keep enable
*********************************************************/
#define BOOT_FW_UPGRADE

/* Check battery's status in order to avoid some effects from charge. */
/* #define BATTERY_CHECK */

/* Check whether the IC is damaged by ESD */
#define ESD_CHECK

/*
 * Check vendor_id number
 * 0:No check vendor_id (default)
 * 1/2/3: Check vendor_id for vendor compatibility
 */
#define ILITEK_GET_VENDOR_ID_NUM                   4

#define ILITEK_VENDOR_ID_0 0
#define ILITEK_VENDOR_ID_1 1
#define ILITEK_VENDOR_ID_2 2
#define ILITEK_VENDOR_ID_3 3

/*
 * vendor_id(s) for vendor(s) to be compatible with.
 * a confirmation of vendor_id(s) is recommended.
 * ILITEK_GET_VENDOR_ID_NUM == 0, no check vendor id, you may ignore them
 * ILITEK_GET_VENDOR_ID_NUM >= 1, compatible with ILITEK_VENDOR_1_ID
 * ILITEK_GET_VENDOR_ID_NUM >= 2, compatible with ILITEK_VENDOR_2_ID
 * ILITEK_GET_VENDOR_ID_NUM == 3, compatible with ILITEK_VENDOR_3_ID
 */
#define ILITEK_VENDOR_0_NAME                         "unknown"
#define ILITEK_VENDOR_1_NAME                         "unknown"
#define ILITEK_VENDOR_2_NAME                         "Dijing"
#define ILITEK_VENDOR_3_NAME                         "unknown"

/*
 * FW_APP.i file for auto upgrade, you must replace it with your own
 * define your own fw_app, the sample one to be replaced is invalid
 * NOTE: if FTS_GET_VENDOR_ID_NUM >= 1, it's the fw corresponding with ILITEK_VENDOR_0_ID
 */
#define ILITEK_UPGRADE_FW0_APP                      "firmware_default/default_ilitek.ili"

/*
 * if ILITEK_GET_VENDOR_ID_NUM >= 2, fw corrsponding with ILITEK_VENDOR_1_ID
 * define your own fw_app, the sample one is invalid
 */
#define ILITEK_UPGRADE_FW1_APP                     "firmware_default/default_ilitek.ili"

/*
 * if ILITEK_GET_VENDOR_ID_NUM >= 3, fw corrsponding with ILITEK_VENDOR_2_ID
 * define your own fw_app, the sample one is invalid
 */
#define ILITEK_UPGRADE_FW2_APP                     "Z578DL_ILI9881F_DJN.ili"

/*
 * if ILITEK_GET_VENDOR_ID_NUM == 4, fw corrsponding with ILITEK_VENDOR_3_ID
 * define your own fw_app, the sample one is invalid
 */
#define ILITEK_UPGRADE_FW3_APP                     "firmware_default/default_ilitek.ili"

#endif
