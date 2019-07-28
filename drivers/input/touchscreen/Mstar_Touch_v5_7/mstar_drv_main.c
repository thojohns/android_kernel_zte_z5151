/******************************************************************************

Copyright (c) 2006-2014 MStar Semiconductor, Inc.
All rights reserved.

Unless otherwise stipulated in writing, any and all information contained
herein regardless in any format shall remain the sole proprietary of
MStar Semiconductor Inc. and be kept in strict confidence
(??MStar Confidential Information??) by the recipient.
Any unauthorized act including without limitation unauthorized disclosure,
copying, use, reproduction, sale, distribution, modification, disassembling,
reverse engineering and compiling of the contents of MStar Confidential
Information is unlawful and strictly prohibited. MStar hereby reserves the
rights to any and all damages, losses, costs and expenses resulting therefrom.

*******************************************************************************/

/**
 *
 *@file    mstar_drv_main.c
 *
 *@brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
/*INCLUDE FILE*/
/*=============================================================*/

#include "mstar_drv_main.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_ic_fw_porting_layer.h"

#ifdef CONFIG_ENABLE_HOTKNOT
#include "mstar_drv_hotknot.h"
#include "mstar_drv_hotknot_queue.h"
#endif /*CONFIG_ENABLE_HOTKNOT */

#ifdef CONFIG_ENABLE_JNI_INTERFACE
#include "mstar_drv_jni_interface.h"
#endif /*CONFIG_ENABLE_JNI_INTERFACE */

/*=============================================================*/
/*CONSTANT VALUE DEFINITION*/
/*=============================================================*/
#define MSTAR_INI_FILE_PATH "/mnt/sdcard/"

/*=============================================================*/
/*EXTERN VARIABLE DECLARATION*/
/*=============================================================*/

extern MutualFirmwareInfo_t g_MutualFirmwareInfo;
extern SelfFirmwareInfo_t g_SelfFirmwareInfo;
/*for MSG21xxA/MSG22xx : DEMO_MODE_PACKET_LENGTH = SELF_DEMO_MODE_PACKET_LENGTH,
for MSG26xxM/MSG28xx : DEMO_MODE_PACKET_LENGTH = MUTUAL_DEMO_MODE_PACKET_LENGTH */
extern u8 g_DemoModePacket[MUTUAL_DEMO_MODE_PACKET_LENGTH];
/*for MSG21xxA/MSG22xx : DEBUG_MODE_PACKET_LENGTH = SELF_DEBUG_MODE_PACKET_LENGTH,
for MSG26xxM/MSG28xx : DEBUG_MODE_PACKET_LENGTH = MUTUAL_DEBUG_MODE_PACKET_LENGTH */
extern u8 g_LogModePacket[MUTUAL_DEBUG_MODE_PACKET_LENGTH];
extern u16 g_FirmwareMode;

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u32 g_GestureWakeupMode[2];

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
extern u8 g_LogGestureDebug[128];
extern u8 g_GestureDebugFlag;
extern u8 g_GestureDebugMode;

extern struct input_dev *g_InputDevice;
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
extern u32 g_LogGestureInfor[GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH];
#endif /*CONFIG_ENABLE_GESTURE_INFORMATION_MODE */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

extern struct i2c_client *g_I2cClient;
extern struct mutex g_Mutex;

extern u8 g_ChipType;
extern u8 g_OriginalChipType;
extern u8 g_Msg22xxChipRevision;

#ifdef CONFIG_ENABLE_ITO_MP_TEST
#if defined(CONFIG_ENABLE_CHIP_TYPE_MSG26XXM) || defined(CONFIG_ENABLE_CHIP_TYPE_MSG28XX)
extern TestScopeInfo_t g_TestScopeInfo;
#endif /*CONFIG_ENABLE_CHIP_TYPE_MSG26XXM || CONFIG_ENABLE_CHIP_TYPE_MSG28XX */
#endif /*CONFIG_ENABLE_ITO_MP_TEST */

extern u8 g_IsEnableGloveMode;
extern u8 g_IsEnableLeatherSheathMode;

/*=============================================================*/
/*LOCAL VARIABLE DEFINITION*/
/*=============================================================*/

static u16 _gDebugReg[MAX_DEBUG_REGISTER_NUM] = { 0 };
static u16 _gDebugRegValue[MAX_DEBUG_REGISTER_NUM] = { 0 };

static u32 _gDebugRegCount = 0;

static u8 _gDebugCmdArgu[MAX_DEBUG_COMMAND_ARGUMENT_NUM] = { 0 };

static u16 _gDebugCmdArguCount = 0;
static u32 _gDebugReadDataSize = 0;

static char _gDebugBuf[1024] = { 0 };

u8 *_gPlatformFwVersion = NULL;	/*internal use firmware version for MStar */

#ifdef CONFIG_ENABLE_ITO_MP_TEST
static ItoTestMode_e _gItoTestMode = 0;
#endif /*CONFIG_ENABLE_ITO_MP_TEST */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
static u32 _gLogGestureCount = 0;
static u8 _gLogGestureInforType = 0;
#endif /*CONFIG_ENABLE_GESTURE_INFORMATION_MODE */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

static u32 _gIsUpdateComplete = 0;

u8 *_gFwVersion = NULL;		/*customer firmware version */

static u8 _gnDebugLogTimesStamp = 0;

static u32 _gFeatureSupportStatus = 0;

static struct proc_dir_entry *_gProcClassEntry = NULL;
static struct proc_dir_entry *_gProcMsTouchScreenMsg20xxEntry = NULL;
static struct proc_dir_entry *_gProcDeviceEntry = NULL;
static struct proc_dir_entry *_gProcChipTypeEntry = NULL;
static struct proc_dir_entry *_gProcFirmwareDataEntry = NULL;
static struct proc_dir_entry *_gProcApkFirmwareUpdateEntry = NULL;
static struct proc_dir_entry *_gProcCustomerFirmwareVersionEntry = NULL;
static struct proc_dir_entry *_gProcPlatformFirmwareVersionEntry = NULL;
static struct proc_dir_entry *_gProcDeviceDriverVersionEntry = NULL;
static struct proc_dir_entry *_gProcSdCardFirmwareUpdateEntry = NULL;
static struct proc_dir_entry *_gProcFirmwareDebugEntry = NULL;
static struct proc_dir_entry *_gProcFirmwareSetDebugValueEntry = NULL;
static struct proc_dir_entry *_gProcFirmwareSmBusDebugEntry = NULL;
static struct proc_dir_entry *_gProcFirmwareSetDQMemValueEntry = NULL;
#ifdef CONFIG_ENABLE_ITO_MP_TEST
static struct proc_dir_entry *_gProcMpTestEntry = NULL;
static struct proc_dir_entry *_gProcMpTestLogEntry = NULL;
static struct proc_dir_entry *_gProcMpTestFailChannelEntry = NULL;
static struct proc_dir_entry *_gProcMpTestScopeEntry = NULL;
#ifdef CONFIG_ENABLE_CHIP_TYPE_MSG28XX
static struct proc_dir_entry *_gProcMpTestLogALLEntry = NULL;
#endif /*CONFIG_ENABLE_CHIP_TYPE_MSG28XX */
#endif /*CONFIG_ENABLE_ITO_MP_TEST */
static struct proc_dir_entry *_gProcFirmwareModeEntry = NULL;
static struct proc_dir_entry *_gProcFirmwareSensorEntry = NULL;
static struct proc_dir_entry *_gProcFirmwarePacketHeaderEntry = NULL;
static struct proc_dir_entry *_gProcQueryFeatureSupportStatusEntry = NULL;
static struct proc_dir_entry *_gProcChangeFeatureSupportStatusEntry = NULL;
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
static struct proc_dir_entry *_gProcGestureWakeupModeEntry = NULL;
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
static struct proc_dir_entry *_gProcGestureDebugModeEntry = NULL;
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */
#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
static struct proc_dir_entry *_gProcGestureInforModeEntry = NULL;
#endif /*CONFIG_ENABLE_GESTURE_INFORMATION_MODE */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */
#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
static struct proc_dir_entry *_gProcReportRateEntry = NULL;
#endif /*CONFIG_ENABLE_COUNT_REPORT_RATE */
static struct proc_dir_entry *_gProcGloveModeEntry = NULL;
static struct proc_dir_entry *_gProcOpenGloveModeEntry = NULL;
static struct proc_dir_entry *_gProcCloseGloveModeEntry = NULL;
static struct proc_dir_entry *_gProcLeatherSheathModeEntry = NULL;
#ifdef CONFIG_ENABLE_JNI_INTERFACE
static struct proc_dir_entry *_gProcJniMethodEntry = NULL;
#endif /*CONFIG_ENABLE_JNI_INTERFACE */
static struct proc_dir_entry *_gProcSeLinuxLimitFirmwareUpdateEntry = NULL;
static struct proc_dir_entry *_gProcForceFirmwareUpdateEntry = NULL;
static struct proc_dir_entry *_gProcMpTestCustomisedEntry = NULL;

static const struct file_operations _gProcMpTestCustomised = {
	.read = DrvMainProcfsMpTestCustomisedRead,
	.write = DrvMainProcfsMpTestCustomisedWrite,
};

static const struct file_operations _gProcChipType = {
	.read = DrvMainProcfsChipTypeRead,
	.write = DrvMainProcfsChipTypeWrite,
};

static const struct file_operations _gProcFirmwareData = {
	.read = DrvMainProcfsFirmwareDataRead,
	.write = DrvMainProcfsFirmwareDataWrite,
};

static const struct file_operations _gProcApkFirmwareUpdate = {
	.read = DrvMainProcfsFirmwareUpdateRead,
	.write = DrvMainProcfsFirmwareUpdateWrite,
};

static const struct file_operations _gProcCustomerFirmwareVersion = {
	.read = DrvMainProcfsCustomerFirmwareVersionRead,
	.write = DrvMainProcfsCustomerFirmwareVersionWrite,
};

static const struct file_operations _gProcPlatformFirmwareVersion = {
	.read = DrvMainProcfsPlatformFirmwareVersionRead,
	.write = DrvMainProcfsPlatformFirmwareVersionWrite,
};

static const struct file_operations _gProcDeviceDriverVersion = {
	.read = DrvMainProcfsDeviceDriverVersionRead,
	.write = DrvMainProcfsDeviceDriverVersionWrite,
};

static const struct file_operations _gProcSdCardFirmwareUpdate = {
	.read = DrvMainProcfsSdCardFirmwareUpdateRead,
	.write = DrvMainProcfsSdCardFirmwareUpdateWrite,
};

static const struct file_operations _gProcFirmwareDebug = {
	.read = DrvMainProcfsFirmwareDebugRead,
	.write = DrvMainProcfsFirmwareDebugWrite,
};

static const struct file_operations _gProcFirmwareSetDebugValue = {
	.read = DrvMainProcfsFirmwareSetDebugValueRead,
	.write = DrvMainProcfsFirmwareSetDebugValueWrite,
};

static const struct file_operations _gProcFirmwareSmBusDebug = {
	.read = DrvMainProcfsFirmwareSmBusDebugRead,
	.write = DrvMainProcfsFirmwareSmBusDebugWrite,
};

static const struct file_operations _gProcFirmwareSetDQMemValue = {
	.read = DrvMainProcfsFirmwareSetDQMemValueRead,
	.write = DrvMainProcfsFirmwareSetDQMemValueWrite,
};

#ifdef CONFIG_ENABLE_ITO_MP_TEST
static const struct file_operations _gProcMpTest = {
	.read = DrvMainProcfsMpTestRead,
	.write = DrvMainProcfsMpTestWrite,
};

static const struct file_operations _gProcMpTestLog = {
	.read = DrvMainProcfsMpTestLogRead,
	.write = DrvMainProcfsMpTestLogWrite,
};

static const struct file_operations _gProcMpTestFailChannel = {
	.read = DrvMainProcfsMpTestFailChannelRead,
	.write = DrvMainProcfsMpTestFailChannelWrite,
};

static const struct file_operations _gProcMpTestScope = {
	.read = DrvMainProcfsMpTestScopeRead,
	.write = DrvMainProcfsMpTestScopeWrite,
};

#ifdef CONFIG_ENABLE_CHIP_TYPE_MSG28XX
static const struct file_operations _gProcMpTestLogAll = {
	.read = DrvMainProcfsMpTestLogAllRead,
	.write = DrvMainProcfsMpTestLogAllWrite,
};
#endif /*CONFIG_ENABLE_CHIP_TYPE_MSG28XX */
#endif /*CONFIG_ENABLE_ITO_MP_TEST */

static const struct file_operations _gProcFirmwareMode = {
	.read = DrvMainProcfsFirmwareModeRead,
	.write = DrvMainProcfsFirmwareModeWrite,
};

static const struct file_operations _gProcFirmwareSensor = {
	.read = DrvMainProcfsFirmwareSensorRead,
	.write = DrvMainProcfsFirmwareSensorWrite,
};

static const struct file_operations _gProcFirmwarePacketHeader = {
	.read = DrvMainProcfsFirmwarePacketHeaderRead,
	.write = DrvMainProcfsFirmwarePacketHeaderWrite,
};

static const struct file_operations _gProcQueryFeatureSupportStatus = {
	.read = DrvMainProcfsQueryFeatureSupportStatusRead,
	.write = DrvMainProcfsQueryFeatureSupportStatusWrite,
};

static const struct file_operations _gProcChangeFeatureSupportStatus = {
	.read = DrvMainProcfsChangeFeatureSupportStatusRead,
	.write = DrvMainProcfsChangeFeatureSupportStatusWrite,
};

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
static const struct file_operations _gProcGestureWakeupMode = {
	.read = DrvMainProcfsGestureWakeupModeRead,
	.write = DrvMainProcfsGestureWakeupModeWrite,
};

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
static const struct file_operations _gProcGestureDebugMode = {
	.read = DrvMainProcfsGestureDebugModeRead,
	.write = DrvMainProcfsGestureDebugModeWrite,
};
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */
#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
static const struct file_operations _gProcGestureInforMode = {
	.read = DrvMainProcfsGestureInforModeRead,
	.write = DrvMainProcfsGestureInforModeWrite,
};
#endif /*CONFIG_ENABLE_GESTURE_INFORMATION_MODE */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
static const struct file_operations _gProcReportRate = {
	.read = DrvMainProcfsReportRateRead,
	.write = DrvMainProcfsReportRateWrite,
};
#endif /*CONFIG_ENABLE_COUNT_REPORT_RATE */

static const struct file_operations _gProcGloveMode = {
	.read = DrvMainProcfsGloveModeRead,
	.write = DrvMainProcfsGloveModeWrite,
};

static const struct file_operations _gProcOpenGloveMode = {
	.read = DrvMainProcfsOpenGloveModeRead,
};

static const struct file_operations _gProcCloseGloveMode = {
	.read = DrvMainProcfsCloseGloveModeRead,
};

static const struct file_operations _gProcLeatherSheathMode = {
	.read = DrvMainProcfsLeatherSheathModeRead,
	.write = DrvMainProcfsLeatherSheathModeWrite,
};

#ifdef CONFIG_ENABLE_JNI_INTERFACE
static const struct file_operations _gProcJniMethod = {
	.read = MsgToolRead,
	.write = MsgToolWrite,
	.unlocked_ioctl = MsgToolIoctl,
	.compat_ioctl = MsgToolIoctl,
};
#endif /*CONFIG_ENABLE_JNI_INTERFACE */

static const struct file_operations _gProcSeLinuxLimitFirmwareUpdate = {
	.read = DrvMainProcfsSeLinuxLimitFirmwareUpdateRead,
};

static const struct file_operations _gProcForceFirmwareUpdate = {
	.read = DrvMainProcfsForceFirmwareUpdateRead,
};

#ifdef CONFIG_ENABLE_HOTKNOT
struct mutex g_HKMutex;
extern struct mutex g_QMutex;
#endif /*CONFIG_ENABLE_HOTKNOT */

/*=============================================================*/
/*GLOBAL VARIABLE DEFINITION*/
/*=============================================================*/

u32 SLAVE_I2C_ID_DBBUS = (0xC4 >> 1);	/*0x62 // for MSG21XX/MSG21XXA/MSG26XXM/MSG28XX */
/*u32 SLAVE_I2C_ID_DBBUS = (0xB2>>1); //0x59 // for MSG22XX*/
u32 SLAVE_I2C_ID_DWI2C = (0x4C >> 1);	/*0x26 */

u16 FIRMWARE_MODE_UNKNOWN_MODE = 0xFFFF;
u16 FIRMWARE_MODE_DEMO_MODE = 0xFFFF;
u16 FIRMWARE_MODE_DEBUG_MODE = 0xFFFF;
u16 FIRMWARE_MODE_RAW_DATA_MODE = 0xFFFF;
/*If project use MSG26xxM/MSG28xx, set MUTUAL_DEMO_MODE_PACKET_LENGTH as default.
If project use MSG21xxA/MSG22xx, set SELF_DEMO_MODE_PACKET_LENGTH as default. */
u16 DEMO_MODE_PACKET_LENGTH = 0;
/*If project use MSG26xxM/MSG28xx, set MUTUAL_DEBUG_MODE_PACKET_LENGTH as default.
If project use MSG21xxA/MSG22xx, set SELF_DEBUG_MODE_PACKET_LENGTH as default. */
u16 DEBUG_MODE_PACKET_LENGTH = 0;
/*If project use MSG26xxM/MSG28xx, set MUTUAL_MAX_TOUCH_NUM as default.
If project use MSG21xxA/MSG22xx, set SELF_MAX_TOUCH_NUM as default. */
u16 MAX_TOUCH_NUM = 0;

struct kset *g_TouchKSet = NULL;
struct kobject *g_TouchKObj = NULL;
u8 g_IsSwitchModeByAPK = 0;

u8 IS_GESTURE_WAKEUP_ENABLED = 0;
u8 IS_GESTURE_DEBUG_MODE_ENABLED = 0;
u8 IS_GESTURE_INFORMATION_MODE_ENABLED = 0;
u8 IS_GESTURE_WAKEUP_MODE_SUPPORT_64_TYPES_ENABLED = 0;

u8 TOUCH_DRIVER_DEBUG_LOG_LEVEL = CONFIG_TOUCH_DRIVER_DEBUG_LOG_LEVEL;
u8 IS_FIRMWARE_DATA_LOG_ENABLED = CONFIG_ENABLE_FIRMWARE_DATA_LOG;
u8 IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED = 0;
u8 IS_DISABLE_ESD_PROTECTION_CHECK = 0;

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
struct kset *g_GestureKSet = NULL;
struct kobject *g_GestureKObj = NULL;
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA
u16 g_FwPacketDataAddress = 0;
u16 g_FwPacketFlagAddress = 0;
u8 g_FwSupportSegment = 0;
#endif /*CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA */

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
u32 g_IsEnableReportRate = 0;
u32 g_InterruptCount = 0;
u32 g_ValidTouchCount = 0;
u32 g_InterruptReportRate = 0;
u32 g_ValidTouchReportRate = 0;

struct timeval g_StartTime;
#endif /*CONFIG_ENABLE_COUNT_REPORT_RATE */

u8 g_FwData[MAX_UPDATE_FIRMWARE_BUFFER_SIZE][1024];
u32 g_FwDataCount = 0;

u8 g_IsHotknotEnabled = 0;
u8 g_IsBypassHotknot = 0;

#ifdef CONFIG_TPD_MSTAR_TEST
unsigned char mstar_str_save_file_path[256];
static unsigned char g_str_ini_file_path[256];
static unsigned char g_str_ini_filename[128];
struct mstar_test_buffer g_mstar_test_buffer;
int g_mstar_tptest_result = 0;

/*static int g_node_data_type = -1;*/
#endif
/*=============================================================*/
/*EXTERN FUNCTION DECLARATION*/
/*=============================================================*/

/*=============================================================*/
/*LOCAL FUNCTION DEFINITION*/
/*=============================================================*/

static s32 _DrvMainCreateProcfsDirEntry(void);

#ifdef CONFIG_ENABLE_HOTKNOT
static s32 _DrvMainHotknotRegistry(void);
#endif /*CONFIG_ENABLE_HOTKNOT */

/*=============================================================*/
/*GLOBAL FUNCTION DEFINITION*/
/*=============================================================*/

/*------------------------------------------------------------------------------//*/
ssize_t DrvMainProcfsMpTestCustomisedRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%d", g_ChipType);

	DBG(&g_I2cClient->dev, "g_ChipType = 0x%x, g_OriginalChipType = 0x%x\n",
		g_ChipType, g_OriginalChipType);

	if (g_ChipType == CHIP_TYPE_MSG22XX) {	/*(0x7A) */
		DBG(&g_I2cClient->dev, "g_Msg22xxChipRevision = 0x%x\n", g_Msg22xxChipRevision);
	}

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsMpTestCustomisedWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					   loff_t *pPos)
{
	char *pValid = NULL;
	char *pTmpFilePath = NULL;
	char szFilePath[100] = { 0 };
	u16 nMajor = 0, nMinor = 0;
	int res = 0;

	if (pBuffer != NULL) {
		pValid = strnstr(pBuffer, ".ini", 50);

		if (pValid) {
			pTmpFilePath = strsep((char **)&pBuffer, ".");

			strlcat(szFilePath, pTmpFilePath, sizeof(szFilePath));
			strlcat(szFilePath, ".ini", sizeof(szFilePath));

			DBG(&g_I2cClient->dev, "*** %s(): File Path =  %s ***\n", __func__, szFilePath);

			DrvIcFwLyrGetCustomerFirmwareVersion(&nMajor, &nMinor, &_gFwVersion);

			DBG(&g_I2cClient->dev, "*** %s() Major = %x , Minor = %x ***\n", __func__, nMajor, nMinor);
			DBG(&g_I2cClient->dev, "*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

			DrvIcFwLyrGetPlatformFirmwareVersion(&_gPlatformFwVersion);

			DBG(&g_I2cClient->dev, "*** %s() _gPlatformFwVersion = %s ***\n", __func__,
			    _gPlatformFwVersion);

			getFirmwareVersionOnFlash();

			DBG(&g_I2cClient->dev, "*** %s() Driver version = %s ***\n", __func__,
			    DEVICE_DRIVER_RELEASE_VERSION);

			/*doing init_Mp_test function */
			res = startMPTest(CHIP_TYPE_MSG28XX, szFilePath);
			if (res < 0) {
				pr_err("*** %s: MP Test is finised but failed ****\n", __func__);
				goto out;
			} else {
				pr_notice("*** %s: MP Test is finised, now saving data into csv file ****\n", __func__);
				save_test_data();
			}
		} else {
			pr_err("*** %s(): The format of file is invaild\n", __func__);
		}
	} else {
		pr_err("*** %s(): The path is invaild\n", __func__);
	}

out:
	Msg28xxEndMPTest();
	return nCount;
}

ssize_t DrvMainProcfsChipTypeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%d", g_ChipType);

	DBG(&g_I2cClient->dev, "g_ChipType = 0x%x, g_OriginalChipType = 0x%x\n", g_ChipType, g_OriginalChipType);

	if (g_ChipType == CHIP_TYPE_MSG22XX) {	/*(0x7A) */
		DBG(&g_I2cClient->dev, "g_Msg22xxChipRevision = 0x%x\n", g_Msg22xxChipRevision);
	}

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsChipTypeWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

/*g_ChipType = DrvIcFwLyrGetChipType();*/

	return nCount;
}

ssize_t DrvMainProcfsFirmwareDataRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	DBG(&g_I2cClient->dev, "*** %s() g_FwDataCount = %d ***\n", __func__, g_FwDataCount);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	*pPos += g_FwDataCount;

	return g_FwDataCount;
}

ssize_t DrvMainProcfsFirmwareDataWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nNum = nCount / 1024;
	u32 nRemainder = nCount % 1024;
	u32 i;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (nNum > 0) {		/*nCount >= 1024 */
		for (i = 0; i < nNum; i++) {
			memcpy(g_FwData[g_FwDataCount], pBuffer + (i * 1024), 1024);

			g_FwDataCount++;
		}

		if (nRemainder > 0) {	/*Handle special firmware size like MSG22XX(48.5KB) */
			DBG(&g_I2cClient->dev, "nRemainder = %d\n", nRemainder);

			memcpy(g_FwData[g_FwDataCount], pBuffer + (i * 1024), nRemainder);

			g_FwDataCount++;
		}
	} else {		/*nCount < 1024 */

		if (nCount > 0) {
			memcpy(g_FwData[g_FwDataCount], pBuffer, nCount);

			g_FwDataCount++;
		}
	}

	DBG(&g_I2cClient->dev, "*** g_FwDataCount = %d ***\n", g_FwDataCount);

	if (pBuffer != NULL) {
		DBG(&g_I2cClient->dev, "*** buf[0] = %c ***\n", pBuffer[0]);
	}

	return nCount;
}

ssize_t DrvMainProcfsFirmwareUpdateRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%u", _gIsUpdateComplete);

	DBG(&g_I2cClient->dev, "*** _gIsUpdateComplete = %d ***\n", _gIsUpdateComplete);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsFirmwareUpdateWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	DrvPlatformLyrDisableFingerTouchReport();

	DBG(&g_I2cClient->dev, "*** %s() g_FwDataCount = %d ***\n", __func__, g_FwDataCount);

	if (DrvIcFwLyrUpdateFirmware(g_FwData, EMEM_ALL) != 0) {
		_gIsUpdateComplete = 0;
		DBG(&g_I2cClient->dev, "Update FAILED\n");
	} else {
		_gIsUpdateComplete = 1;
		DBG(&g_I2cClient->dev, "Update SUCCESS\n");
	}

	DrvPlatformLyrEnableFingerTouchReport();

	return nCount;
}

ssize_t DrvMainProcfsCustomerFirmwareVersionRead(struct file *pFile, char __user *pBuffer, size_t nCount,
						 loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%s\n", _gFwVersion);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsCustomerFirmwareVersionWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
						  loff_t *pPos)
{
	u16 nMajor = 0, nMinor = 0;

	DrvIcFwLyrGetCustomerFirmwareVersion(&nMajor, &nMinor, &_gFwVersion);

	DBG(&g_I2cClient->dev, "*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

	return nCount;
}

ssize_t DrvMainProcfsPlatformFirmwareVersionRead(struct file *pFile, char __user *pBuffer, size_t nCount,
						 loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() _gPlatformFwVersion = %s ***\n", __func__, _gPlatformFwVersion);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%s\n", _gPlatformFwVersion);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsPlatformFirmwareVersionWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
						  loff_t *pPos)
{
	DrvIcFwLyrGetPlatformFirmwareVersion(&_gPlatformFwVersion);

	DBG(&g_I2cClient->dev, "*** %s() _gPlatformFwVersion = %s ***\n", __func__, _gPlatformFwVersion);

	return nCount;
}

ssize_t DrvMainProcfsDeviceDriverVersionRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%s", DEVICE_DRIVER_RELEASE_VERSION);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsDeviceDriverVersionWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					      loff_t *pPos)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	return nCount;
}

ssize_t DrvMainProcfsSdCardFirmwareUpdateRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u16 nMajor = 0, nMinor = 0;
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read and
	indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	DrvIcFwLyrGetCustomerFirmwareVersion(&nMajor, &nMinor, &_gFwVersion);

	DBG(&g_I2cClient->dev, "*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

	nLength = snprintf(pBuffer, PAGE_SIZE, "%s\n", _gFwVersion);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsSdCardFirmwareUpdateWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					       loff_t *pPos)
{
	char *pValid = NULL;
	char *pTmpFilePath = NULL;
	char szFilePath[100] = { 0 };

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);
	DBG(&g_I2cClient->dev, "pBuffer = %s\n", pBuffer);

	if (pBuffer != NULL) {
		pValid = strnstr(pBuffer, ".bin", 50);

		if (pValid) {
			pTmpFilePath = strsep((char **)&pBuffer, ".");

			DBG(&g_I2cClient->dev, "pTmpFilePath = %s\n", pTmpFilePath);
			strlcat(szFilePath, pTmpFilePath, sizeof(szFilePath));
			strlcat(szFilePath, ".bin", sizeof(szFilePath) - sizeof(pTmpFilePath));

			DBG(&g_I2cClient->dev, "szFilePath = %s\n", szFilePath);

			if (DrvIcFwLyrUpdateFirmwareBySdCard(szFilePath) != 0) {
				DBG(&g_I2cClient->dev, "Update FAILED\n");
			} else {
				DBG(&g_I2cClient->dev, "Update SUCCESS\n");
			}
		} else {
			DBG(&g_I2cClient->dev, "The file type of the update firmware bin file is not a .bin file.\n");
		}
	} else {
		DBG(&g_I2cClient->dev, "The file path of the update firmware bin file is NULL.\n");
	}

	return nCount;
}

ssize_t DrvMainProcfsSeLinuxLimitFirmwareUpdateRead(struct file *pFile, char __user *pBuffer, size_t nCount,
						    loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read and
	indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	DBG(&g_I2cClient->dev, "FIRMWARE_FILE_PATH_ON_SD_CARD = %s\n", FIRMWARE_FILE_PATH_ON_SD_CARD);

	if (DrvIcFwLyrUpdateFirmwareBySdCard(FIRMWARE_FILE_PATH_ON_SD_CARD) != 0) {
		_gIsUpdateComplete = 0;
		DBG(&g_I2cClient->dev, "Update FAILED\n");
	} else {
		_gIsUpdateComplete = 1;
		DBG(&g_I2cClient->dev, "Update SUCCESS\n");
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%u", _gIsUpdateComplete);

	DBG(&g_I2cClient->dev, "*** _gIsUpdateComplete = %d ***\n", _gIsUpdateComplete);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsForceFirmwareUpdateRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	DBG(&g_I2cClient->dev, "*** IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED = %d ***\n",
	    IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED);

	IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED = 1;	/*Enable force firmware update */
	_gFeatureSupportStatus = IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED;

	nLength = snprintf(pBuffer, PAGE_SIZE, "%u", _gFeatureSupportStatus);

	DBG(&g_I2cClient->dev, "*** _gFeatureSupportStatus = %d ***\n", _gFeatureSupportStatus);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsFirmwareDebugRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 i, nLength = 0;
	u8 nBank, nAddr;
	u16 szRegData[MAX_DEBUG_REGISTER_NUM] = { 0 };
	u8 szOut[MAX_DEBUG_REGISTER_NUM * 25] = { 0 }, szValue[10] = {0};

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	DbBusEnterSerialDebugMode();
	DbBusStopMCU();
	DbBusIICUseBus();
	DbBusIICReshape();

	for (i = 0; i < _gDebugRegCount; i++) {
		szRegData[i] = RegGet16BitValue(_gDebugReg[i]);
	}

	DbBusIICNotUseBus();
	DbBusNotStopMCU();
	DbBusExitSerialDebugMode();

	for (i = 0; i < _gDebugRegCount; i++) {
		nBank = (_gDebugReg[i] >> 8) & 0xFF;
		nAddr = _gDebugReg[i] & 0xFF;

		DBG(&g_I2cClient->dev, "reg(0x%02X,0x%02X)=0x%04X\n", nBank, nAddr, szRegData[i]);

		strlcat(szOut, "reg(", sizeof(szOut));
		snprintf(szValue, sizeof(szValue), "0x%02X", nBank);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, ",", sizeof(szOut));
		snprintf(szValue, sizeof(szValue), "0x%02X", nAddr);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, ")=", sizeof(szOut));
		snprintf(szValue, sizeof(szValue), "0x%04X", szRegData[i]);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, "\n", sizeof(szOut));
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%s\n", szOut);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsFirmwareDebugWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 i;
	char *pCh = NULL;
	char *pStr = NULL;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		DBG(&g_I2cClient->dev, "*** pBuffer[0] = %c ***\n", pBuffer[0]);
		DBG(&g_I2cClient->dev, "*** pBuffer[1] = %c ***\n", pBuffer[1]);
		DBG(&g_I2cClient->dev, "*** pBuffer[2] = %c ***\n", pBuffer[2]);
		DBG(&g_I2cClient->dev, "*** pBuffer[3] = %c ***\n", pBuffer[3]);
		DBG(&g_I2cClient->dev, "*** pBuffer[4] = %c ***\n", pBuffer[4]);
		DBG(&g_I2cClient->dev, "*** pBuffer[5] = %c ***\n", pBuffer[5]);

		DBG(&g_I2cClient->dev, "nCount = %d\n", (int)nCount);

		memset(_gDebugBuf, 0, 1024);

		if (copy_from_user(_gDebugBuf, pBuffer, nCount)) {
			DBG(&g_I2cClient->dev, "copy_from_user() failed\n");

			return -EFAULT;
		}

		_gDebugBuf[nCount] = '\0';
		pStr = _gDebugBuf;

		i = 0;

		while ((pCh = strsep((char **)&pStr, " ,")) && (i < MAX_DEBUG_REGISTER_NUM)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			_gDebugReg[i] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));

			DBG(&g_I2cClient->dev, "_gDebugReg[%d] = 0x%04X\n", i, _gDebugReg[i]);
			i++;
		}
		_gDebugRegCount = i;

		DBG(&g_I2cClient->dev, "_gDebugRegCount = %d\n", _gDebugRegCount);
	}

	return nCount;
}

ssize_t DrvMainProcfsFirmwareSetDebugValueRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 i, nLength = 0;
	u8 nBank, nAddr;
	u16 szRegData[MAX_DEBUG_REGISTER_NUM] = { 0 };
	u8 szOut[MAX_DEBUG_REGISTER_NUM * 25] = { 0 }, szValue[10] = {0};

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	DbBusEnterSerialDebugMode();
	DbBusStopMCU();
	DbBusIICUseBus();
	DbBusIICReshape();

	for (i = 0; i < _gDebugRegCount; i++) {
		szRegData[i] = RegGet16BitValue(_gDebugReg[i]);
	}

	DbBusIICNotUseBus();
	DbBusNotStopMCU();
	DbBusExitSerialDebugMode();

	for (i = 0; i < _gDebugRegCount; i++) {
		nBank = (_gDebugReg[i] >> 8) & 0xFF;
		nAddr = _gDebugReg[i] & 0xFF;

		DBG(&g_I2cClient->dev, "reg(0x%02X,0x%02X)=0x%04X\n", nBank, nAddr, szRegData[i]);

		strlcat(szOut, "reg(", sizeof(szOut));
		snprintf(szValue, sizeof(szValue), "0x%02X", nBank);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, ",", sizeof(szOut));
		snprintf(szValue, sizeof(szValue), "0x%02X", nAddr);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, ")=", sizeof(szOut));
		snprintf(szValue, sizeof(szValue), "0x%04X", szRegData[i]);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, "\n", sizeof(szOut));
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%s\n", szOut);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsFirmwareSetDebugValueWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
						loff_t *pPos)
{
	u32 i, j, k;
	char *pCh = NULL;
	char *pStr = NULL;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		DBG(&g_I2cClient->dev, "*** pBuffer[0] = %c ***\n", pBuffer[0]);
		DBG(&g_I2cClient->dev, "*** pBuffer[1] = %c ***\n", pBuffer[1]);

		DBG(&g_I2cClient->dev, "nCount = %d\n", (int)nCount);

		memset(_gDebugBuf, 0, 1024);

		if (copy_from_user(_gDebugBuf, pBuffer, nCount)) {
			DBG(&g_I2cClient->dev, "copy_from_user() failed\n");

			return -EFAULT;
		}

		_gDebugBuf[nCount] = '\0';
		pStr = _gDebugBuf;

		i = 0;
		j = 0;
		k = 0;

		while ((pCh = strsep((char **)&pStr, " ,")) && (i < 2)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			if ((i % 2) == 0) {
				_gDebugReg[j] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));
				DBG(&g_I2cClient->dev, "_gDebugReg[%d] = 0x%04X\n", j, _gDebugReg[j]);
				j++;
			} else {	/*(i%2) == 1 */

				_gDebugRegValue[k] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));
				DBG(&g_I2cClient->dev, "_gDebugRegValue[%d] = 0x%04X\n", k, _gDebugRegValue[k]);
				k++;
			}

			i++;
		}
		_gDebugRegCount = j;

		DBG(&g_I2cClient->dev, "_gDebugRegCount = %d\n", _gDebugRegCount);

		DbBusEnterSerialDebugMode();
		DbBusStopMCU();
		DbBusIICUseBus();
		DbBusIICReshape();

		for (i = 0; i < _gDebugRegCount; i++) {
			RegSet16BitValue(_gDebugReg[i], _gDebugRegValue[i]);
			DBG(&g_I2cClient->dev, "_gDebugReg[%d] = 0x%04X, _gDebugRegValue[%d] = 0x%04X\n",
				i, _gDebugReg[i], i, _gDebugRegValue[i]);
		}

		DbBusIICNotUseBus();
		DbBusNotStopMCU();
		DbBusExitSerialDebugMode();
	}

	return nCount;
}

ssize_t DrvMainProcfsFirmwareSmBusDebugRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 i = 0, nLength = 0;
	u8 szSmBusRxData[MAX_I2C_TRANSACTION_LENGTH_LIMIT] = { 0 };
	u8 szOut[MAX_I2C_TRANSACTION_LENGTH_LIMIT * 2] = { 0 };
	u8 szValue[10] = { 0 };
	s32 rc = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}
#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	DmaReset();
#endif /*CONFIG_ENABLE_DMA_IIC */
#endif /*CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	mutex_lock(&g_Mutex);
	DBG(&g_I2cClient->dev, "*** %s() *** mutex_lock(&g_Mutex)\n", __func__);	/*add for debug */

	while (i < 5) {
		if (_gDebugCmdArguCount > 0) {	/*Send write command */
			DBG(&g_I2cClient->dev, "Execute I2C SMBUS write command\n");

			mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
			rc = IicWriteData(SLAVE_I2C_ID_DWI2C, &_gDebugCmdArgu[0], _gDebugCmdArguCount);
			if (rc > 0) {
				DBG(&g_I2cClient->dev, "IicWriteData(0x%X, 0x%X, %d) success\n", SLAVE_I2C_ID_DWI2C,
				    _gDebugCmdArgu[0], _gDebugCmdArguCount);

				if (_gDebugReadDataSize == 0) {
					break;	/*No need to execute I2C SMBUS read command later. So, break here. */
				}
			}
		}

		if (_gDebugReadDataSize > 0) {	/*Send read command */
			DBG(&g_I2cClient->dev, "Execute I2C SMBUS read command\n");

			mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
			rc = IicReadData(SLAVE_I2C_ID_DWI2C, &szSmBusRxData[0], _gDebugReadDataSize);
			if (rc > 0) {
				DBG(&g_I2cClient->dev, "IicReadData(0x%X, 0x%X, %d) success\n", SLAVE_I2C_ID_DWI2C,
				    szSmBusRxData[0], _gDebugReadDataSize);
				break;
			}
		}

		i++;
	}
	if (i == 5) {
		DBG(&g_I2cClient->dev, "IicWriteData() & IicReadData() failed, rc = %d\n", rc);
	}

	for (i = 0; i < _gDebugReadDataSize; i++) {	/*Output format 2. */
		DBG(&g_I2cClient->dev, "szSmBusRxData[%d] = 0x%x\n", i, szSmBusRxData[i]);

		snprintf(szValue, sizeof(szValue), "%02x", szSmBusRxData[i]);
		strlcat(szOut, szValue, sizeof(szOut));

		if (i < (_gDebugReadDataSize - 1)) {
			strlcat(szOut, ",", sizeof(szOut));
		}
	}

	DBG(&g_I2cClient->dev, "*** %s() *** mutex_unlock(&g_Mutex)\n", __func__);	/*add for debug */
	mutex_unlock(&g_Mutex);

	nLength = snprintf(pBuffer, PAGE_SIZE, "%s\n", szOut);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsFirmwareSmBusDebugWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					     loff_t *pPos)
{
	u32 i, j;
	char szCmdType[5] = { 0 };
	char *pCh = NULL;
	char *pStr = NULL;
	int	ret;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		DBG(&g_I2cClient->dev, "*** pBuffer[0] = %c ***\n", pBuffer[0]);
		DBG(&g_I2cClient->dev, "*** pBuffer[1] = %c ***\n", pBuffer[1]);
		DBG(&g_I2cClient->dev, "*** pBuffer[2] = %c ***\n", pBuffer[2]);
		DBG(&g_I2cClient->dev, "*** pBuffer[3] = %c ***\n", pBuffer[3]);
		DBG(&g_I2cClient->dev, "*** pBuffer[4] = %c ***\n", pBuffer[4]);
		DBG(&g_I2cClient->dev, "*** pBuffer[5] = %c ***\n", pBuffer[5]);

		DBG(&g_I2cClient->dev, "nCount = %d\n", (int)nCount);

		memset(_gDebugBuf, 0, 1024);

		if (copy_from_user(_gDebugBuf, pBuffer, nCount)) {
			DBG(&g_I2cClient->dev, "copy_from_user() failed\n");

			return -EFAULT;
		}

		/*Reset to 0 before parsing the adb command */
		_gDebugCmdArguCount = 0;
		_gDebugReadDataSize = 0;

		_gDebugBuf[nCount] = '\0';
		pStr = _gDebugBuf;

		i = 0;
		j = 0;

		while ((pCh = strsep((char **)&pStr, " ,")) && (j < MAX_DEBUG_COMMAND_ARGUMENT_NUM)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			if (strcmp(pCh, "w") == 0 || strcmp(pCh, "r") == 0) {
				memcpy(szCmdType, pCh, strlen(pCh));
			} else if (strcmp(szCmdType, "w") == 0) {
				_gDebugCmdArgu[j] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));
				DBG(&g_I2cClient->dev, "_gDebugCmdArgu[%d] = 0x%02X\n", j, _gDebugCmdArgu[j]);

				j++;

				_gDebugCmdArguCount = j;
				DBG(&g_I2cClient->dev, "_gDebugCmdArguCount = %d\n", _gDebugCmdArguCount);
			} else if (strcmp(szCmdType, "r") == 0) {
				ret = kstrtoint(pCh, 10, &_gDebugReadDataSize);
				if (ret) {
					DBG(&g_I2cClient->dev, "invalid param.\n");
					return ret;
				}
				DBG(&g_I2cClient->dev, "_gDebugReadDataSize = %d\n", _gDebugReadDataSize);
			} else {
				DBG(&g_I2cClient->dev, "Un-supported adb command format!\n");
			}

			i++;
		}
	}

	return nCount;
}

ssize_t DrvMainProcfsFirmwareSetDQMemValueRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 i, nLength = 0;
	u8 nBank, nAddr;
	u32 szRegData[MAX_DEBUG_REGISTER_NUM] = { 0 };
	u8 szOut[MAX_DEBUG_REGISTER_NUM * 25] = { 0 }, szValue[10] = {0};

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	for (i = 0; i < _gDebugRegCount; i++) {
		szRegData[i] = DrvIcFwLyrReadDQMemValue(_gDebugReg[i]);
	}

	for (i = 0; i < _gDebugRegCount; i++) {
		nBank = (_gDebugReg[i] >> 8) & 0xFF;
		nAddr = _gDebugReg[i] & 0xFF;

		DBG(&g_I2cClient->dev, "reg(0x%02X,0x%02X)=0x%08X\n", nBank, nAddr, szRegData[i]);

		strlcat(szOut, "reg(", sizeof(szOut));
		snprintf(szValue, sizeof(szValue), "0x%02X", nBank);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, ",", sizeof(szOut));
		snprintf(szValue, sizeof(szValue), "0x%02X", nAddr);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, ")=", sizeof(szOut));
		snprintf(szValue, sizeof(szValue), "0x%04X", szRegData[i]);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, "\n", sizeof(szOut));
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%s\n", szOut);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsFirmwareSetDQMemValueWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
						loff_t *pPos)
{
	u32 i, j, k;
	char *pCh = NULL;
	char *pStr = NULL;
	u16 nRealDQMemAddr = 0;
	u32 nRealDQMemValue = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		DBG(&g_I2cClient->dev, "*** pBuffer[0] = %c ***\n", pBuffer[0]);
		DBG(&g_I2cClient->dev, "*** pBuffer[1] = %c ***\n", pBuffer[1]);

		DBG(&g_I2cClient->dev, "nCount = %d\n", (int)nCount);

		memset(_gDebugBuf, 0, 1024);

		if (copy_from_user(_gDebugBuf, pBuffer, nCount)) {
			DBG(&g_I2cClient->dev, "copy_from_user() failed\n");

			return -EFAULT;
		}

		_gDebugBuf[nCount] = '\0';
		pStr = _gDebugBuf;

		i = 0;
		j = 0;
		k = 0;

		while ((pCh = strsep((char **)&pStr, " ,")) && (i < 2)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			if ((i % 2) == 0) {
				_gDebugReg[j] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));
				DBG(&g_I2cClient->dev, "_gDebugReg[%d] = 0x%04X\n", j, _gDebugReg[j]);
				j++;
			} else {	/*(i%2) == 1 */

				_gDebugRegValue[k] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));
				DBG(&g_I2cClient->dev, "_gDebugRegValue[%d] = 0x%04X\n", k, _gDebugRegValue[k]);
				k++;
			}

			i++;
		}
		_gDebugRegCount = j;

		DBG(&g_I2cClient->dev, "_gDebugRegCount = %d\n", _gDebugRegCount);

		if ((_gDebugReg[0] % 4) == 0) {
			nRealDQMemAddr = _gDebugReg[0];
			nRealDQMemValue = DrvIcFwLyrReadDQMemValue(nRealDQMemAddr);
			_gDebugReg[0] = nRealDQMemAddr;
			DBG(&g_I2cClient->dev, "nRealDQMemValue Raw = %X\n", nRealDQMemValue);
			nRealDQMemValue &= 0xFFFF0000;
			nRealDQMemValue |= _gDebugRegValue[0];
			DBG(&g_I2cClient->dev, "nRealDQMemValue Modify = %X\n", nRealDQMemValue);
			DrvIcFwLyrWriteDQMemValue(nRealDQMemAddr, nRealDQMemValue);
		} else if ((_gDebugReg[0] % 4) == 2) {
			nRealDQMemAddr = _gDebugReg[0] - 2;
			nRealDQMemValue = DrvIcFwLyrReadDQMemValue(nRealDQMemAddr);
			_gDebugReg[0] = nRealDQMemAddr;
			DBG(&g_I2cClient->dev, "nRealDQMemValue Raw = %X\n", nRealDQMemValue);

			nRealDQMemValue &= 0x0000FFFF;
			nRealDQMemValue |= (_gDebugRegValue[0] << 16);
			DBG(&g_I2cClient->dev, "nRealDQMemValue Modify = %X\n", nRealDQMemValue);
			DrvIcFwLyrWriteDQMemValue(nRealDQMemAddr, nRealDQMemValue);
		}
	}

	return nCount;
}

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_ITO_MP_TEST

ssize_t DrvMainProcfsMpTestRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	DBG(&g_I2cClient->dev, "*** ctp mp test status = %d ***\n", DrvIcFwLyrGetMpTestResult());

	nLength = snprintf(pBuffer, PAGE_SIZE, "%d", DrvIcFwLyrGetMpTestResult());

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsMpTestWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nMode = 0;
	u32 i = 0;
	char *pCh = NULL;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		i = 0;
		while ((pCh = strsep((char **)&pBuffer, ",")) && (i < 1)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			nMode = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));

			i++;
		}

		DBG(&g_I2cClient->dev, "Mp Test Mode = 0x%x\n", nMode);

		if (nMode == ITO_TEST_MODE_OPEN_TEST) {	/*open test */
			_gItoTestMode = ITO_TEST_MODE_OPEN_TEST;
			DrvIcFwLyrScheduleMpTestWork(ITO_TEST_MODE_OPEN_TEST);
		} else if (nMode == ITO_TEST_MODE_SHORT_TEST) {	/*short test */
			_gItoTestMode = ITO_TEST_MODE_SHORT_TEST;
			DrvIcFwLyrScheduleMpTestWork(ITO_TEST_MODE_SHORT_TEST);
		} else if (nMode == ITO_TEST_MODE_WATERPROOF_TEST) {	/*waterproof test */
			_gItoTestMode = ITO_TEST_MODE_WATERPROOF_TEST;
			DrvIcFwLyrScheduleMpTestWork(ITO_TEST_MODE_WATERPROOF_TEST);
		} else {
			DBG(&g_I2cClient->dev, "*** Undefined MP Test Mode ***\n");
		}
	}

	return nCount;
}

ssize_t DrvMainProcfsMpTestLogRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	DrvIcFwLyrGetMpTestDataLog(_gItoTestMode, pBuffer, &nLength);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsMpTestLogWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	return nCount;
}

ssize_t DrvMainProcfsMpTestFailChannelRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	DrvIcFwLyrGetMpTestFailChannel(_gItoTestMode, pBuffer, &nLength);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsMpTestFailChannelWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					    loff_t *pPos)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	return nCount;
}

ssize_t DrvMainProcfsMpTestScopeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}
#if defined(CONFIG_ENABLE_CHIP_TYPE_MSG26XXM) || defined(CONFIG_ENABLE_CHIP_TYPE_MSG28XX)
	if (g_ChipType == CHIP_TYPE_MSG26XXM || g_ChipType == CHIP_TYPE_MSG28XX || g_ChipType == CHIP_TYPE_MSG58XXA) {
		DrvIcFwLyrGetMpTestScope(&g_TestScopeInfo);

		nLength =
		    snprintf(pBuffer, PAGE_SIZE, "%d,%d,%d",
		    g_TestScopeInfo.nMx, g_TestScopeInfo.nMy, g_TestScopeInfo.nKeyNum);
	}
#endif /*CONFIG_ENABLE_CHIP_TYPE_MSG26XXM || CONFIG_ENABLE_CHIP_TYPE_MSG28XX */

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsMpTestScopeWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	return nCount;
}

#if defined(CONFIG_ENABLE_CHIP_TYPE_MSG28XX)
ssize_t DrvMainProcfsMpTestLogAllRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	if (g_ChipType == CHIP_TYPE_MSG28XX || g_ChipType == CHIP_TYPE_MSG58XXA) {
		DrvIcFwLyrGetMpTestLogAll(pBuffer, &nLength);
	}

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsMpTestLogAllWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	return nCount;
}
#endif /*CONFIG_ENABLE_CHIP_TYPE_MSG28XX */
#endif /*CONFIG_ENABLE_ITO_MP_TEST */

/*--------------------------------------------------------------------------*/

ssize_t DrvMainProcfsFirmwareModeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	if (g_ChipType == CHIP_TYPE_MSG26XXM) {
		g_FirmwareMode = DrvIcFwLyrGetFirmwareMode();

		DBG(&g_I2cClient->dev, "%s() firmware mode = 0x%x\n", __func__, g_FirmwareMode);

		nLength = snprintf(pBuffer, PAGE_SIZE, "%x", g_FirmwareMode);
	} else if (g_ChipType == CHIP_TYPE_MSG28XX || g_ChipType == CHIP_TYPE_MSG58XXA) {
		DrvIcFwLyrMutualGetFirmwareInfo(&g_MutualFirmwareInfo);
		g_FirmwareMode = g_MutualFirmwareInfo.nFirmwareMode;

		DBG(&g_I2cClient->dev, "%s() firmware mode = 0x%x\n", __func__, g_MutualFirmwareInfo.nFirmwareMode);

		nLength = snprintf(pBuffer, PAGE_SIZE, "%x", g_MutualFirmwareInfo.nFirmwareMode);
	} else if (g_ChipType == CHIP_TYPE_MSG21XXA || g_ChipType == CHIP_TYPE_MSG22XX) {
		DrvIcFwLyrSelfGetFirmwareInfo(&g_SelfFirmwareInfo);
		g_FirmwareMode = g_SelfFirmwareInfo.nFirmwareMode;

		DBG(&g_I2cClient->dev, "%s() firmware mode = 0x%x, can change firmware mode = %d\n", __func__,
		    g_SelfFirmwareInfo.nFirmwareMode, g_SelfFirmwareInfo.nIsCanChangeFirmwareMode);

		nLength =
		    snprintf(pBuffer, PAGE_SIZE, "%x,%d", g_SelfFirmwareInfo.nFirmwareMode,
			    g_SelfFirmwareInfo.nIsCanChangeFirmwareMode);
	}

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsFirmwareModeWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nMode;
	int ret;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		ret = kstrtoint(pBuffer, 16, &nMode);
		if (ret) {
			DBG(&g_I2cClient->dev, "invalid param.\n");
			return ret;
		}
		DBG(&g_I2cClient->dev, "firmware mode = 0x%x\n", nMode);

		g_IsSwitchModeByAPK = 0;

		if (nMode == FIRMWARE_MODE_DEMO_MODE) {	/*demo mode */
			g_FirmwareMode = DrvIcFwLyrChangeFirmwareMode(FIRMWARE_MODE_DEMO_MODE);
		} else if (nMode == FIRMWARE_MODE_DEBUG_MODE) {	/*debug mode */
			g_FirmwareMode = DrvIcFwLyrChangeFirmwareMode(FIRMWARE_MODE_DEBUG_MODE);
			g_IsSwitchModeByAPK = 1;
			/*Set _gnDebugLogTimesStamp for filter duplicate packet on MTPTool APK */
			_gnDebugLogTimesStamp = 0;
		} else if (nMode == FIRMWARE_MODE_RAW_DATA_MODE) {	/*raw data mode */
			if (g_ChipType == CHIP_TYPE_MSG21XXA || g_ChipType == CHIP_TYPE_MSG22XX) {
				g_FirmwareMode = DrvIcFwLyrChangeFirmwareMode(FIRMWARE_MODE_RAW_DATA_MODE);
				g_IsSwitchModeByAPK = 1;
			}
		} else {
			DBG(&g_I2cClient->dev, "*** Undefined Firmware Mode ***\n");
		}
	}

	DBG(&g_I2cClient->dev, "*** g_FirmwareMode = 0x%x ***\n", g_FirmwareMode);

	return nCount;
}

ssize_t DrvMainProcfsFirmwareSensorRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	if (g_ChipType == CHIP_TYPE_MSG26XXM || g_ChipType == CHIP_TYPE_MSG28XX || g_ChipType == CHIP_TYPE_MSG58XXA) {
		if (g_MutualFirmwareInfo.nLogModePacketHeader == 0xA5
		    || g_MutualFirmwareInfo.nLogModePacketHeader == 0xAB) {
			nLength =
				snprintf(pBuffer, PAGE_SIZE, "%d,%d",
				g_MutualFirmwareInfo.nMx, g_MutualFirmwareInfo.nMy);
		} else if (g_MutualFirmwareInfo.nLogModePacketHeader == 0xA7) {
			nLength =
			    snprintf(pBuffer, PAGE_SIZE, "%d,%d,%d,%d", g_MutualFirmwareInfo.nMx,
			    g_MutualFirmwareInfo.nMy, g_MutualFirmwareInfo.nSs, g_MutualFirmwareInfo.nSd);
		} else {
			DBG(&g_I2cClient->dev, "Undefined debug mode packet format : 0x%x\n",
			    g_MutualFirmwareInfo.nLogModePacketHeader);
			nLength = 0;
		}
	} else if (g_ChipType == CHIP_TYPE_MSG21XXA || g_ChipType == CHIP_TYPE_MSG22XX) {
		nLength = snprintf(pBuffer, PAGE_SIZE, "%d", g_SelfFirmwareInfo.nLogModePacketLength);
	}

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsFirmwareSensorWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	return nCount;
}

ssize_t DrvMainProcfsFirmwarePacketHeaderRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	if (g_ChipType == CHIP_TYPE_MSG26XXM || g_ChipType == CHIP_TYPE_MSG28XX || g_ChipType == CHIP_TYPE_MSG58XXA) {
		nLength = snprintf(pBuffer, PAGE_SIZE, "%d", g_MutualFirmwareInfo.nLogModePacketHeader);
	} else if (g_ChipType == CHIP_TYPE_MSG21XXA || g_ChipType == CHIP_TYPE_MSG22XX) {
		nLength = snprintf(pBuffer, PAGE_SIZE, "%d", g_SelfFirmwareInfo.nLogModePacketHeader);
	}

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsFirmwarePacketHeaderWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					       loff_t *pPos)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	return nCount;
}

ssize_t DrvMainKObjectPacketShow(struct kobject *pKObj, struct kobj_attribute *pAttr, char *pBuf)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (strcmp(pAttr->attr.name, "packet") == 0) {
		if (g_FirmwareMode == FIRMWARE_MODE_DEMO_MODE) {
			if (g_DemoModePacket != NULL) {
				DBG(&g_I2cClient->dev,
				    "g_FirmwareMode=%x, g_DemoModePacket[0]=%x, g_DemoModePacket[1]=%x\n",
				    g_FirmwareMode, g_DemoModePacket[0], g_DemoModePacket[1]);
				DBG(&g_I2cClient->dev, "g_DemoModePacket[2]=%x, g_DemoModePacket[3]=%x\n",
				    g_DemoModePacket[2], g_DemoModePacket[3]);
				DBG(&g_I2cClient->dev, "g_DemoModePacket[4]=%x, g_DemoModePacket[5]=%x\n",
				    g_DemoModePacket[4], g_DemoModePacket[5]);

				memcpy(pBuf, g_DemoModePacket, DEMO_MODE_PACKET_LENGTH);

				nLength = DEMO_MODE_PACKET_LENGTH;
				DBG(&g_I2cClient->dev, "nLength = %d\n", nLength);
			} else {
				DBG(&g_I2cClient->dev, "g_DemoModePacket is NULL\n");
			}
		} else {
			/*g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE ||
			g_FirmwareMode == FIRMWARE_MODE_RAW_DATA_MODE */
			if (g_LogModePacket != NULL) {
				DBG(&g_I2cClient->dev,
				    "g_FirmwareMode=%x, g_LogModePacket[0]=%x, g_LogModePacket[1]=%x\n", g_FirmwareMode,
				    g_LogModePacket[0], g_LogModePacket[1]);
				DBG(&g_I2cClient->dev, "g_LogModePacket[2]=%x, g_LogModePacket[3]=%x\n",
				    g_LogModePacket[2], g_LogModePacket[3]);
				DBG(&g_I2cClient->dev, "g_LogModePacket[4]=%x, g_LogModePacket[5]=%x\n",
				    g_LogModePacket[4], g_LogModePacket[5]);

				if ((g_ChipType == CHIP_TYPE_MSG26XXM || g_ChipType == CHIP_TYPE_MSG28XX
				     || g_ChipType == CHIP_TYPE_MSG58XXA)
				    && (g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE) && (g_LogModePacket[0] == 0xA5
											|| g_LogModePacket[0] == 0xAB
											|| g_LogModePacket[0] ==
											0xA7)) {
					memcpy(pBuf, g_LogModePacket, g_MutualFirmwareInfo.nLogModePacketLength);

					if (_gnDebugLogTimesStamp >= 255) {
						_gnDebugLogTimesStamp = 0;
					} else {
						_gnDebugLogTimesStamp++;
					}

					pBuf[g_MutualFirmwareInfo.nLogModePacketLength] = _gnDebugLogTimesStamp;
					DBG(&g_I2cClient->dev, "_gnDebugLogTimesStamp=%d\n",
						pBuf[g_MutualFirmwareInfo.nLogModePacketLength]);

					nLength = g_MutualFirmwareInfo.nLogModePacketLength + 1;
					DBG(&g_I2cClient->dev, "nLength = %d\n", nLength);
				} else if ((g_ChipType == CHIP_TYPE_MSG21XXA || g_ChipType == CHIP_TYPE_MSG22XX)
					   && (g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE
					       || g_FirmwareMode == FIRMWARE_MODE_RAW_DATA_MODE)
					   && (g_LogModePacket[0] == 0x62)) {
					memcpy(pBuf, g_LogModePacket, g_SelfFirmwareInfo.nLogModePacketLength);

					nLength = g_SelfFirmwareInfo.nLogModePacketLength;
					DBG(&g_I2cClient->dev, "nLength = %d\n", nLength);
				} else {
					DBG(&g_I2cClient->dev,
					    "CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
				}
			} else {
				DBG(&g_I2cClient->dev, "g_LogModePacket is NULL\n");
			}
		}
	} else {
		DBG(&g_I2cClient->dev, "pAttr->attr.name = %s\n", pAttr->attr.name);
	}

	return nLength;
}

ssize_t DrvMainKObjectPacketStore(struct kobject *pKObj, struct kobj_attribute *pAttr, const char *pBuf,
				  size_t nCount)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);
/*
    if (strcmp(pAttr->attr.name, "packet") == 0)
    {

    }
*/
	return nCount;
}

static struct kobj_attribute packet_attr = __ATTR(packet, 0664, DrvMainKObjectPacketShow, DrvMainKObjectPacketStore);

/* Create a group of attributes so that we can create and destroy them all at once. */
static struct attribute *attrs[] = {
	&packet_attr.attr,
	NULL,			/* need to NULL terminate the list of attributes */
};

/*
 *An unnamed attribute group will put all of the attributes directly in
 *the kobject directory. If we specify a name, a subdirectory will be
 *created for the attributes with the directory being the name of the
 *attribute group.
 */
static struct attribute_group attr_group = {
	.attrs = attrs,
};

/*------------------------------------------------------------------------------//*/

ssize_t DrvMainProcfsQueryFeatureSupportStatusRead(struct file *pFile, char __user *pBuffer, size_t nCount,
						   loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%u", _gFeatureSupportStatus);

	DBG(&g_I2cClient->dev, "*** _gFeatureSupportStatus = %d ***\n", _gFeatureSupportStatus);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsQueryFeatureSupportStatusWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
						    loff_t *pPos)
{
	u32 nFeature;
	int ret;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		ret = kstrtoint(pBuffer, 16, &nFeature);
		if (ret) {
			DBG(&g_I2cClient->dev, "invalid param.\n");
			return ret;
		}
		DBG(&g_I2cClient->dev, "nFeature = 0x%x\n", nFeature);
/*Not support dynamically on/off gesture wakeup mode by adb command yet. */
		if (nFeature == FEATURE_GESTURE_WAKEUP_MODE) {
			_gFeatureSupportStatus = IS_GESTURE_WAKEUP_ENABLED;
/*Not support dynamically on/off gesture debug mode by adb command yet. */
		} else if (nFeature == FEATURE_GESTURE_DEBUG_MODE) {
			_gFeatureSupportStatus = IS_GESTURE_DEBUG_MODE_ENABLED;
/*Not support dynamically on/off gesture information mode by adb command yet. */
		} else if (nFeature == FEATURE_GESTURE_INFORMATION_MODE) {
			_gFeatureSupportStatus = IS_GESTURE_INFORMATION_MODE_ENABLED;
		} else if (nFeature == FEATURE_TOUCH_DRIVER_DEBUG_LOG) {
			_gFeatureSupportStatus = TOUCH_DRIVER_DEBUG_LOG_LEVEL;
		} else if (nFeature == FEATURE_FIRMWARE_DATA_LOG) {
			_gFeatureSupportStatus = IS_FIRMWARE_DATA_LOG_ENABLED;

#ifdef CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA
			if (g_ChipType == CHIP_TYPE_MSG26XXM || g_ChipType == CHIP_TYPE_MSG28XX
			    || g_ChipType == CHIP_TYPE_MSG58XXA) {
				/*If the debug mode data log function is supported, then get packet address
				and flag address for segment read finger touch data. */
				if (_gFeatureSupportStatus == 1) {
					DrvIcFwLyrGetTouchPacketAddress(&g_FwPacketDataAddress, &g_FwPacketFlagAddress);
				}
			}
#endif /*CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA */
		} else if (nFeature == FEATURE_FORCE_TO_UPDATE_FIRMWARE) {
			_gFeatureSupportStatus = IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED;
		} else if (nFeature == FEATURE_DISABLE_ESD_PROTECTION_CHECK) {
			_gFeatureSupportStatus = IS_DISABLE_ESD_PROTECTION_CHECK;
		} else {
			DBG(&g_I2cClient->dev, "*** Undefined Feature ***\n");
		}
	}

	DBG(&g_I2cClient->dev, "*** _gFeatureSupportStatus = %d ***\n", _gFeatureSupportStatus);

	return nCount;
}

ssize_t DrvMainProcfsChangeFeatureSupportStatusRead(struct file *pFile, char __user *pBuffer, size_t nCount,
						    loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%u", _gFeatureSupportStatus);

	DBG(&g_I2cClient->dev, "*** _gFeatureSupportStatus = %d ***\n", _gFeatureSupportStatus);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsChangeFeatureSupportStatusWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
						     loff_t *pPos)
{
	u32 i;
	u32 nFeature = 0, nNewValue = 0;
	char *pCh = NULL;
	char *pStr = NULL;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		DBG(&g_I2cClient->dev, "nCount = %d\n", (int)nCount);

		memset(_gDebugBuf, 0, 1024);

		if (copy_from_user(_gDebugBuf, pBuffer, nCount)) {
			DBG(&g_I2cClient->dev, "copy_from_user() failed\n");

			return -EFAULT;
		}

		_gDebugBuf[nCount] = '\0';
		pStr = _gDebugBuf;

		i = 0;

		while ((pCh = strsep((char **)&pStr, " ,")) && (i < 3)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			if (i == 0) {
				nFeature = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));
				DBG(&g_I2cClient->dev, "nFeature = 0x%04X\n", nFeature);
			} else if (i == 1) {
				nNewValue = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));
				DBG(&g_I2cClient->dev, "nNewValue = %d\n", nNewValue);
			} else {
				DBG(&g_I2cClient->dev, "End of parsing adb command.\n");
			}

			i++;
		}
/*Not support dynamically on/off gesture wakeup mode by adb command yet. */
		if (nFeature == FEATURE_GESTURE_WAKEUP_MODE) {
			IS_GESTURE_WAKEUP_ENABLED = nNewValue;
			_gFeatureSupportStatus = IS_GESTURE_WAKEUP_ENABLED;
/*Not support dynamically on/off gesture debug mode by adb command yet. */
		} else if (nFeature == FEATURE_GESTURE_DEBUG_MODE) {
			IS_GESTURE_DEBUG_MODE_ENABLED = nNewValue;
			_gFeatureSupportStatus = IS_GESTURE_DEBUG_MODE_ENABLED;
/*Not support dynamically on/off gesture information mode by adb command yet. */
		} else if (nFeature == FEATURE_GESTURE_INFORMATION_MODE) {
			IS_GESTURE_INFORMATION_MODE_ENABLED = nNewValue;
			_gFeatureSupportStatus = IS_GESTURE_INFORMATION_MODE_ENABLED;
		} else if (nFeature == FEATURE_TOUCH_DRIVER_DEBUG_LOG) {
			TOUCH_DRIVER_DEBUG_LOG_LEVEL = nNewValue;
			_gFeatureSupportStatus = TOUCH_DRIVER_DEBUG_LOG_LEVEL;
		} else if (nFeature == FEATURE_FIRMWARE_DATA_LOG) {
			IS_FIRMWARE_DATA_LOG_ENABLED = nNewValue;
			_gFeatureSupportStatus = IS_FIRMWARE_DATA_LOG_ENABLED;
		} else if (nFeature == FEATURE_FORCE_TO_UPDATE_FIRMWARE) {
			IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED = nNewValue;
			_gFeatureSupportStatus = IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED;
		} else if (nFeature == FEATURE_DISABLE_ESD_PROTECTION_CHECK) {
			IS_DISABLE_ESD_PROTECTION_CHECK = nNewValue;
			_gFeatureSupportStatus = IS_DISABLE_ESD_PROTECTION_CHECK;
		} else {
			DBG(&g_I2cClient->dev, "*** Undefined Feature ***\n");
		}

		DBG(&g_I2cClient->dev, "*** _gFeatureSupportStatus = %d ***\n", _gFeatureSupportStatus);
	}

	return nCount;
}

/*------------------------------------------------------------------------------//*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

ssize_t DrvMainProcfsGestureWakeupModeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}
#ifdef CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE
	DBG(&g_I2cClient->dev, "g_GestureWakeupMode = 0x%x, 0x%x\n", g_GestureWakeupMode[0], g_GestureWakeupMode[1]);

	nLength = snprintf(pBuffer, PAGE_SIZE, "%x,%x", g_GestureWakeupMode[0], g_GestureWakeupMode[1]);
#else
	DBG(&g_I2cClient->dev, "g_GestureWakeupMode = 0x%x\n", g_GestureWakeupMode[0]);

	nLength = snprintf(pBuffer, PAGE_SIZE, "%x", g_GestureWakeupMode[0]);
#endif /*CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE */

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsGestureWakeupModeWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					    loff_t *pPos)
{
	u32 nLength;
	u32 nWakeupMode[2] = { 0 };
	int ret;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
#ifdef CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE
		u32 i;
		char *pCh;

		i = 0;
		while ((pCh = strsep((char **)&pBuffer, " ,")) && (i < 2)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			nWakeupMode[i] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));

			DBG(&g_I2cClient->dev, "nWakeupMode[%d] = 0x%04X\n", i, nWakeupMode[i]);
			i++;
		}
#else
		ret = kstrtoint(pBuffer, 16, &nWakeupMode[0]);
		if (ret) {
			DBG(&g_I2cClient->dev, "invalid param.\n");
			return ret;
		}
		DBG(&g_I2cClient->dev, "nWakeupMode = 0x%x\n", nWakeupMode[0]);
#endif /*CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE */

		nLength = nCount;
		DBG(&g_I2cClient->dev, "nLength = %d\n", nLength);

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) == GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE1_FLAG) == GESTURE_WAKEUP_MODE_RESERVE1_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE1_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE1_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE2_FLAG) == GESTURE_WAKEUP_MODE_RESERVE2_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE2_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE2_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE3_FLAG) == GESTURE_WAKEUP_MODE_RESERVE3_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE3_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE3_FLAG);
		}

#ifdef CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE
		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE4_FLAG) == GESTURE_WAKEUP_MODE_RESERVE4_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE4_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE4_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE5_FLAG) == GESTURE_WAKEUP_MODE_RESERVE5_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE5_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE5_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE6_FLAG) == GESTURE_WAKEUP_MODE_RESERVE6_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE6_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE6_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE7_FLAG) == GESTURE_WAKEUP_MODE_RESERVE7_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE7_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE7_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE8_FLAG) == GESTURE_WAKEUP_MODE_RESERVE8_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE8_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE8_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE9_FLAG) == GESTURE_WAKEUP_MODE_RESERVE9_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE9_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE9_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE10_FLAG) == GESTURE_WAKEUP_MODE_RESERVE10_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE10_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE10_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE11_FLAG) == GESTURE_WAKEUP_MODE_RESERVE11_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE11_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE11_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE12_FLAG) == GESTURE_WAKEUP_MODE_RESERVE12_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE12_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE12_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE13_FLAG) == GESTURE_WAKEUP_MODE_RESERVE13_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE13_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE13_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE14_FLAG) == GESTURE_WAKEUP_MODE_RESERVE14_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE14_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE14_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE15_FLAG) == GESTURE_WAKEUP_MODE_RESERVE15_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE15_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE15_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE16_FLAG) == GESTURE_WAKEUP_MODE_RESERVE16_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE16_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE16_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE17_FLAG) == GESTURE_WAKEUP_MODE_RESERVE17_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE17_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE17_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE18_FLAG) == GESTURE_WAKEUP_MODE_RESERVE18_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE18_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE18_FLAG);
		}

		if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE19_FLAG) == GESTURE_WAKEUP_MODE_RESERVE19_FLAG) {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] | GESTURE_WAKEUP_MODE_RESERVE19_FLAG;
		} else {
			g_GestureWakeupMode[0] = g_GestureWakeupMode[0] & (~GESTURE_WAKEUP_MODE_RESERVE19_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE20_FLAG) == GESTURE_WAKEUP_MODE_RESERVE20_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE20_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE20_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE21_FLAG) == GESTURE_WAKEUP_MODE_RESERVE21_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE21_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE21_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE22_FLAG) == GESTURE_WAKEUP_MODE_RESERVE22_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE22_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE22_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE23_FLAG) == GESTURE_WAKEUP_MODE_RESERVE23_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE23_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE23_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE24_FLAG) == GESTURE_WAKEUP_MODE_RESERVE24_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE24_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE24_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE25_FLAG) == GESTURE_WAKEUP_MODE_RESERVE25_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE25_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE25_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE26_FLAG) == GESTURE_WAKEUP_MODE_RESERVE26_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE26_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE26_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE27_FLAG) == GESTURE_WAKEUP_MODE_RESERVE27_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE27_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE27_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE28_FLAG) == GESTURE_WAKEUP_MODE_RESERVE28_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE28_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE28_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE29_FLAG) == GESTURE_WAKEUP_MODE_RESERVE29_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE29_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE29_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE30_FLAG) == GESTURE_WAKEUP_MODE_RESERVE30_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE30_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE30_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE31_FLAG) == GESTURE_WAKEUP_MODE_RESERVE31_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE31_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE31_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE32_FLAG) == GESTURE_WAKEUP_MODE_RESERVE32_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE32_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE32_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE33_FLAG) == GESTURE_WAKEUP_MODE_RESERVE33_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE33_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE33_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE34_FLAG) == GESTURE_WAKEUP_MODE_RESERVE34_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE34_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE34_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE35_FLAG) == GESTURE_WAKEUP_MODE_RESERVE35_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE35_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE35_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE36_FLAG) == GESTURE_WAKEUP_MODE_RESERVE36_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE36_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE36_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE37_FLAG) == GESTURE_WAKEUP_MODE_RESERVE37_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE37_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE37_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE38_FLAG) == GESTURE_WAKEUP_MODE_RESERVE38_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE38_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE38_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE39_FLAG) == GESTURE_WAKEUP_MODE_RESERVE39_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE39_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE39_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE40_FLAG) == GESTURE_WAKEUP_MODE_RESERVE40_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE40_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE40_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE41_FLAG) == GESTURE_WAKEUP_MODE_RESERVE41_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE41_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE41_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE42_FLAG) == GESTURE_WAKEUP_MODE_RESERVE42_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE42_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE42_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE43_FLAG) == GESTURE_WAKEUP_MODE_RESERVE43_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE43_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE43_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE44_FLAG) == GESTURE_WAKEUP_MODE_RESERVE44_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE44_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE44_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE45_FLAG) == GESTURE_WAKEUP_MODE_RESERVE45_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE45_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE45_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE46_FLAG) == GESTURE_WAKEUP_MODE_RESERVE46_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE46_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE46_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE47_FLAG) == GESTURE_WAKEUP_MODE_RESERVE47_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE47_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE47_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE48_FLAG) == GESTURE_WAKEUP_MODE_RESERVE48_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE48_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE48_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE49_FLAG) == GESTURE_WAKEUP_MODE_RESERVE49_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE49_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE49_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE50_FLAG) == GESTURE_WAKEUP_MODE_RESERVE50_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE50_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE50_FLAG);
		}

		if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE51_FLAG) == GESTURE_WAKEUP_MODE_RESERVE51_FLAG) {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] | GESTURE_WAKEUP_MODE_RESERVE51_FLAG;
		} else {
			g_GestureWakeupMode[1] = g_GestureWakeupMode[1] & (~GESTURE_WAKEUP_MODE_RESERVE51_FLAG);
		}
#endif /*CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE */

		DBG(&g_I2cClient->dev, "g_GestureWakeupMode = 0x%x,  0x%x\n", g_GestureWakeupMode[0],
		    g_GestureWakeupMode[1]);
	}

	return nCount;
}

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE

ssize_t DrvMainProcfsGestureDebugModeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	DBG(&g_I2cClient->dev, "g_GestureDebugMode = 0x%x\n", g_GestureDebugMode);

	nLength = snprintf(pBuffer, PAGE_SIZE, "%d", g_GestureDebugMode);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsGestureDebugModeWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					   loff_t *pPos)
{
	u8 ucGestureMode[2];
	u8 i;
	char *pCh;

	if (pBuffer != NULL) {
		i = 0;
		while ((pCh = strsep((char **)&pBuffer, " ,")) && (i < 2)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			ucGestureMode[i] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));

			DBG(&g_I2cClient->dev, "ucGestureMode[%d] = 0x%04X\n", i, ucGestureMode[i]);
			i++;
		}

		g_GestureDebugMode = ucGestureMode[0];
		g_GestureDebugFlag = ucGestureMode[1];

		DBG(&g_I2cClient->dev, "Gesture flag = 0x%x\n", g_GestureDebugFlag);

		if (g_GestureDebugMode == 0x01) {
			DrvIcFwLyrOpenGestureDebugMode(g_GestureDebugFlag);

/*input_report_key(g_InputDevice, RESERVER42, 1);*/
			input_report_key(g_InputDevice, KEY_POWER, 1);
			input_sync(g_InputDevice);
/*input_report_key(g_InputDevice, RESERVER42, 0);*/
			input_report_key(g_InputDevice, KEY_POWER, 0);
			input_sync(g_InputDevice);
		} else if (g_GestureDebugMode == 0x00) {
			DrvIcFwLyrCloseGestureDebugMode();
		} else {
			DBG(&g_I2cClient->dev, "*** Undefined Gesture Debug Mode ***\n");
		}
	}

	return nCount;
}

ssize_t DrvMainKObjectGestureDebugShow(struct kobject *pKObj, struct kobj_attribute *pAttr, char *pBuf)
{
	u32 i = 0;
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (strcmp(pAttr->attr.name, "gesture_debug") == 0) {
		if (g_LogGestureDebug != NULL) {
			DBG(&g_I2cClient->dev, "g_LogGestureDebug[0]=%x, g_LogGestureDebug[1]=%x\n",
			    g_LogGestureDebug[0], g_LogGestureDebug[1]);
			DBG(&g_I2cClient->dev, "g_LogGestureDebug[2]=%x, g_LogGestureDebug[3]=%x\n",
			    g_LogGestureDebug[2], g_LogGestureDebug[3]);
			DBG(&g_I2cClient->dev, "g_LogGestureDebug[4]=%x, g_LogGestureDebug[5]=%x\n",
			    g_LogGestureDebug[4], g_LogGestureDebug[5]);

			if (g_LogGestureDebug[0] == 0xA7 && g_LogGestureDebug[3] == 0x51) {
				for (i = 0; i < 0x80; i++) {
					pBuf[i] = g_LogGestureDebug[i];
				}

				nLength = 0x80;
				DBG(&g_I2cClient->dev, "nLength = %d\n", nLength);
			} else {
				DBG(&g_I2cClient->dev,
				    "CURRENT MODE IS NOT GESTURE DEBUG MODE/WRONG GESTURE DEBUG MODE HEADER\n");
			}
		} else {
			DBG(&g_I2cClient->dev, "g_LogGestureDebug is NULL\n");
		}
	} else {
		DBG(&g_I2cClient->dev, "pAttr->attr.name = %s\n", pAttr->attr.name);
	}

	return nLength;
}

ssize_t DrvMainKObjectGestureDebugStore(struct kobject *pKObj, struct kobj_attribute *pAttr, const char *pBuf,
					size_t nCount)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);
/*
    if (strcmp(pAttr->attr.name, "gesture_debug") == 0)
    {

    }
*/
	return nCount;
}

static struct kobj_attribute gesture_attr =
__ATTR(gesture_debug, 0664, DrvMainKObjectGestureDebugShow, DrvMainKObjectGestureDebugStore);

/* Create a group of attributes so that we can create and destroy them all at once. */
static struct attribute *gestureattrs[] = {
	&gesture_attr.attr,
	NULL,			/* need to NULL terminate the list of attributes */
};

/*
 *An unnamed attribute group will put all of the attributes directly in
 *the kobject directory. If we specify a name, a subdirectory will be
 *created for the attributes with the directory being the name of the
 *attribute group.
 */
struct attribute_group gestureattr_group = {
	.attrs = gestureattrs,
};

#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE

ssize_t DrvMainProcfsGestureInforModeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u8 szOut[GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH * 5] = { 0 }, szValue[10] = {
	0};
	u32 szLogGestureInfo[GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH] = { 0 };
	u32 i = 0;
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	_gLogGestureCount = 0;
	/*FIRMWARE_GESTURE_INFORMATION_MODE_A */
	if (_gLogGestureInforType == FIRMWARE_GESTURE_INFORMATION_MODE_A) {
		for (i = 0; i < 2; i++) {	/*0 EventFlag; 1 RecordNum */
			szLogGestureInfo[_gLogGestureCount] = g_LogGestureInfor[4 + i];
			_gLogGestureCount++;
		}

		for (i = 2; i < 8; i++) {	/*2~3 Xst Yst; 4~5 Xend Yend; 6~7 char_width char_height */
			szLogGestureInfo[_gLogGestureCount] = g_LogGestureInfor[4 + i];
			_gLogGestureCount++;
		}
	/*FIRMWARE_GESTURE_INFORMATION_MODE_B*/
	} else if (_gLogGestureInforType == FIRMWARE_GESTURE_INFORMATION_MODE_B) {
		for (i = 0; i < 2; i++) {	/*0 EventFlag; 1 RecordNum */
			szLogGestureInfo[_gLogGestureCount] = g_LogGestureInfor[4 + i];
			_gLogGestureCount++;
		}

		for (i = 0; i < g_LogGestureInfor[5] * 2; i++) {	/*(X and Y)*RecordNum */
			szLogGestureInfo[_gLogGestureCount] = g_LogGestureInfor[12 + i];
			_gLogGestureCount++;
		}
	} else if (_gLogGestureInforType == FIRMWARE_GESTURE_INFORMATION_MODE_C) {
		/*FIRMWARE_GESTURE_INFORMATION_MODE_C */
		for (i = 0; i < 6; i++) {	/*header */
			szLogGestureInfo[_gLogGestureCount] = g_LogGestureInfor[i];
			_gLogGestureCount++;
		}

		for (i = 6; i < 86; i++) {
			szLogGestureInfo[_gLogGestureCount] = g_LogGestureInfor[i];
			_gLogGestureCount++;
		}

		szLogGestureInfo[_gLogGestureCount] = g_LogGestureInfor[86];	/*dummy */
		_gLogGestureCount++;
		szLogGestureInfo[_gLogGestureCount] = g_LogGestureInfor[87];	/*checksum */
		_gLogGestureCount++;
	} else {
		DBG(&g_I2cClient->dev, "*** Undefined GESTURE INFORMATION MODE ***\n");
	}

	for (i = 0; i < _gLogGestureCount; i++) {
		snprintf(szValue, sizeof(szValue), "%u", szLogGestureInfo[i]);
		strlcat(szOut, szValue, sizeof(szOut));
		strlcat(szOut, ",", sizeof(szOut));
	}

	nLength = snprintf(pBuffer, PAGE_SIZE, "%s\n", szOut);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsGestureInforModeWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					   loff_t *pPos)
{
	u32 nMode;
	int	ret;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		ret = kstrtoint(pBuffer, 10, &nMode);
		if (ret) {
			DBG(&g_I2cClient->dev, "invalid param.\n");
			return ret;
		}
		_gLogGestureInforType = nMode;
	}

	DBG(&g_I2cClient->dev, "*** _gLogGestureInforType type = 0x%x ***\n", _gLogGestureInforType);

	return nCount;
}

#endif /*CONFIG_ENABLE_GESTURE_INFORMATION_MODE */

#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
ssize_t DrvMainProcfsReportRateRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	struct timeval tEndTime;
	suseconds_t nStartTime, nEndTime, nElapsedTime;
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	DBG(&g_I2cClient->dev, "g_InterruptCount = %d, g_ValidTouchCount = %d\n", g_InterruptCount, g_ValidTouchCount);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	/*Get end time */
	do_gettimeofday(&tEndTime);

	nStartTime = g_StartTime.tv_sec + g_StartTime.tv_usec / 1000000;
	nEndTime = tEndTime.tv_sec + tEndTime.tv_usec / 1000000;

	nElapsedTime = nEndTime - nStartTime;

	DBG(&g_I2cClient->dev, "Start time : %lu sec, %lu msec\n", g_StartTime.tv_sec, g_StartTime.tv_usec);
	DBG(&g_I2cClient->dev, "End time : %lu sec, %lu msec\n", tEndTime.tv_sec, tEndTime.tv_usec);

	DBG(&g_I2cClient->dev, "Elapsed time : %lu sec\n", nElapsedTime);

	/*Calculate report rate */
	if (nElapsedTime != 0) {
		g_InterruptReportRate = g_InterruptCount / nElapsedTime;
		g_ValidTouchReportRate = g_ValidTouchCount / nElapsedTime;
	} else {
		g_InterruptReportRate = 0;
		g_ValidTouchReportRate = 0;
	}

	DBG(&g_I2cClient->dev, "g_InterruptReportRate = %d, g_ValidTouchReportRate = %d\n", g_InterruptReportRate,
	    g_ValidTouchReportRate);

	g_InterruptCount = 0;	/*Reset count */
	g_ValidTouchCount = 0;

	nLength = snprintf(pBuffer, PAGE_SIZE, "%u,%u", g_InterruptReportRate, g_ValidTouchReportRate);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsReportRateWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	int ret;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		ret = kstrtoint(pBuffer, 10, &g_IsEnableReportRate);
		if (ret) {
			DBG(&g_I2cClient->dev, "invalid param.\n");
			return ret;
		}
/*1 : enable report rate calculation, 0 : disable report rate calculation, 2 : reset count */
		DBG(&g_I2cClient->dev, "g_IsEnableReportRate = %d\n", g_IsEnableReportRate);

		g_InterruptCount = 0;	/*Reset count */
		g_ValidTouchCount = 0;
	}

	return nCount;
}
#endif /*CONFIG_ENABLE_COUNT_REPORT_RATE */

/*--------------------------------------------------------------------------*/

ssize_t DrvMainProcfsGloveModeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;
	u8 nGloveMode = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	if (g_ChipType == CHIP_TYPE_MSG28XX || g_ChipType == CHIP_TYPE_MSG58XXA) {
		DrvPlatformLyrDisableFingerTouchReport();

		DrvIcFwLyrGetGloveInfo(&nGloveMode);

		DrvPlatformLyrEnableFingerTouchReport();

		DBG(&g_I2cClient->dev, "Glove Mode = 0x%x\n", nGloveMode);

		nLength = snprintf(pBuffer, PAGE_SIZE, "%x", nGloveMode);
	}

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsGloveModeWrite(struct file *pFile, const char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nGloveMode = 0;
	u32 i = 0;
	char *pCh = NULL;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		i = 0;
		while ((pCh = strsep((char **)&pBuffer, ",")) && (i < 1)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			nGloveMode = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));

			i++;
		}

		DBG(&g_I2cClient->dev, "Glove Mode = 0x%x\n", nGloveMode);

		DrvPlatformLyrDisableFingerTouchReport();

		if (nGloveMode == 0x01) {	/*open glove mode */
			DrvIcFwLyrOpenGloveMode();
		} else if (nGloveMode == 0x00) {	/*close glove mode */
			DrvIcFwLyrCloseGloveMode();
		} else {
			DBG(&g_I2cClient->dev, "*** Undefined Glove Mode ***\n");
		}
		DBG(&g_I2cClient->dev, "g_IsEnableGloveMode = 0x%x\n", g_IsEnableGloveMode);

		DrvPlatformLyrEnableFingerTouchReport();
	}

	return nCount;
}

ssize_t DrvMainProcfsOpenGloveModeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	if (g_ChipType == CHIP_TYPE_MSG28XX || g_ChipType == CHIP_TYPE_MSG58XXA) {
		DrvPlatformLyrDisableFingerTouchReport();

		DrvIcFwLyrOpenGloveMode();

		DrvPlatformLyrEnableFingerTouchReport();
	}
	DBG(&g_I2cClient->dev, "g_IsEnableGloveMode = 0x%x\n", g_IsEnableGloveMode);

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsCloseGloveModeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	if (g_ChipType == CHIP_TYPE_MSG28XX || g_ChipType == CHIP_TYPE_MSG58XXA) {
		DrvPlatformLyrDisableFingerTouchReport();

		DrvIcFwLyrCloseGloveMode();

		DrvPlatformLyrEnableFingerTouchReport();
	}
	DBG(&g_I2cClient->dev, "g_IsEnableGloveMode = 0x%x\n", g_IsEnableGloveMode);

	*pPos += nLength;

	return nLength;
}

/*--------------------------------------------------------------------------*/

ssize_t DrvMainProcfsLeatherSheathModeRead(struct file *pFile, char __user *pBuffer, size_t nCount, loff_t *pPos)
{
	u32 nLength = 0;
	u8 nLeatherSheathMode = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	/*If file position is non-zero, then assume the string has been read
	and indicate there is no more data to be read. */
	if (*pPos != 0) {
		return 0;
	}

	if (g_ChipType == CHIP_TYPE_MSG28XX || g_ChipType == CHIP_TYPE_MSG58XXA) {
		DrvPlatformLyrDisableFingerTouchReport();

		DrvIcFwLyrGetLeatherSheathInfo(&nLeatherSheathMode);

		DrvPlatformLyrEnableFingerTouchReport();

		DBG(&g_I2cClient->dev, "Leather Sheath Mode = 0x%x\n", nLeatherSheathMode);

		nLength = snprintf(pBuffer, PAGE_SIZE, "%x", nLeatherSheathMode);
	}

	*pPos += nLength;

	return nLength;
}

ssize_t DrvMainProcfsLeatherSheathModeWrite(struct file *pFile, const char __user *pBuffer, size_t nCount,
					    loff_t *pPos)
{
	u32 nLeatherSheathMode = 0;
	u32 i = 0;
	char *pCh = NULL;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (pBuffer != NULL) {
		i = 0;
		while ((pCh = strsep((char **)&pBuffer, ",")) && (i < 1)) {
			DBG(&g_I2cClient->dev, "pCh = %s\n", pCh);

			nLeatherSheathMode = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));

			i++;
		}

		DBG(&g_I2cClient->dev, "Leather Sheath Mode = 0x%x\n", nLeatherSheathMode);

		DrvPlatformLyrDisableFingerTouchReport();

		if (nLeatherSheathMode == 0x01) {	/*open leather sheath mode */
			DrvIcFwLyrOpenLeatherSheathMode();
		} else if (nLeatherSheathMode == 0x00) {	/*close leather sheath mode */
			DrvIcFwLyrCloseLeatherSheathMode();
		} else {
			DBG(&g_I2cClient->dev, "*** Undefined Leather Sheath Mode ***\n");
		}
		DBG(&g_I2cClient->dev, "g_IsEnableLeatherSheathMode = 0x%x\n", g_IsEnableLeatherSheathMode);

		DrvPlatformLyrEnableFingerTouchReport();
	}

	return nCount;
}

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_HOTKNOT
static long hotknot_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long nRet = 0;

	DBG(&g_I2cClient->dev, "*** %s ***\n", __func__);
	mutex_lock(&g_HKMutex);
	nRet = HotKnotIoctl(file, cmd, arg);
	mutex_unlock(&g_HKMutex);

	return nRet;
}

static int hotknot_value;

static ssize_t hotknot_value_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",  hotknot_value);
}

static ssize_t hotknot_value_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = kstrtoint(buf, 10, &hotknot_value);
	if (ret) {
		DBG(&g_I2cClient->dev, "invalid param.\n");
		return ret;
	}
	return count;
}

static struct kobj_attribute hotknot_value_attr = __ATTR(hotknot_value, 0666, hotknot_value_show, hotknot_value_store);
static struct attribute *hotknot_attrs[] = {
	&hotknot_value_attr.attr, 0
};

static struct attribute_group hotknot_attr_group = {
	.attrs = hotknot_attrs
};

static const struct file_operations hotknot_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hotknot_ioctl
};

/*static struct miscdevice hotknot_miscdevice =*/
struct miscdevice hotknot_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hotknot",
	.nodename = "hotknot",
	.mode = 0666,
	.fops = &hotknot_fops
};
#endif /*CONFIG_ENABLE_HOTKNOT */

/*------------------------------------------------------------------------------//*/

s32 DrvMainTouchDeviceInitialize(void)
{
	s32 nRetVal = 0;
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
	int nErr;
	struct hwmsen_object tObjPs;
#endif /*CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */
#endif /*CONFIG_ENABLE_PROXIMITY_DETECTION */

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	_DrvMainCreateProcfsDirEntry();	/*Create procfs directory entry */
#ifdef CONFIG_ENABLE_HOTKNOT
	_DrvMainHotknotRegistry();	/*register hotknot here...         */
#endif /*CONFIG_ENABLE_HOTKNOT */

#ifdef CONFIG_ENABLE_JNI_INTERFACE
	CreateMsgToolMem();
#endif /*CONFIG_ENABLE_JNI_INTERFACE */

#ifdef CONFIG_ENABLE_ITO_MP_TEST
	DrvIcFwLyrCreateMpTestWorkQueue();
#endif /*CONFIG_ENABLE_ITO_MP_TEST */

	g_ChipType = DrvIcFwLyrGetChipType();	/*Try to get chip type by SLAVE_I2C_ID_DBBUS(0x62) firstly. */

	if (g_ChipType == 0) {	/*If failed, try to get chip type by SLAVE_I2C_ID_DBBUS(0x59) again. */
		SLAVE_I2C_ID_DBBUS = (0xB2 >> 1);	/*0x59 */

		g_ChipType = DrvIcFwLyrGetChipType();
	}

	DrvPlatformLyrTouchDeviceResetHw();

	if (g_ChipType != 0) {	/*To make sure TP is attached on cell phone. */
		if (g_ChipType == CHIP_TYPE_MSG26XXM || g_ChipType == CHIP_TYPE_MSG28XX
		    || g_ChipType == CHIP_TYPE_MSG58XXA) {
			memset(&g_MutualFirmwareInfo, 0x0, sizeof(MutualFirmwareInfo_t));
		} else if (g_ChipType == CHIP_TYPE_MSG21XXA || g_ChipType == CHIP_TYPE_MSG22XX) {
			memset(&g_SelfFirmwareInfo, 0x0, sizeof(SelfFirmwareInfo_t));
		}

		DrvIcFwLyrVariableInitialize();

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
		{
			u8 szChargerStatus[20] = { 0 };

			DrvCommonReadFile(POWER_SUPPLY_BATTERY_STATUS_PATCH, szChargerStatus, 20);

			DBG(&g_I2cClient->dev, "*** Battery Status : %s ***\n", szChargerStatus);

			if (strnstr(szChargerStatus, "Charging", sizeof(szChargerStatus)) != NULL
				|| strnstr(szChargerStatus, "Full", sizeof(szChargerStatus)) != NULL
				|| strnstr(szChargerStatus, "Fully charged", sizeof(szChargerStatus)) != NULL) {
				DrvFwCtrlChargerDetection(1);	/*charger plug-in */
			} else {	/*Not charging */

				DrvFwCtrlChargerDetection(0);	/*charger plug-out */
			}
		}
#endif /*CONFIG_ENABLE_CHARGER_DETECTION */

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
/*tsps_assist_register_callback("msg2xxx", &DrvPlatformLyrTpPsEnable, &DrvPlatformLyrGetTpPsData);*/
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
		tObjPs.polling = 0;	/*interrupt mode */
		tObjPs.sensor_operate = DrvPlatformLyrTpPsOperate;
		nErr = hwmsen_attach(ID_PROXIMITY, &tObjPs)
		if (nErr) {
			DBG(&g_I2cClient->dev, "call hwmsen_attach() failed = %d\n", nErr);
		}
#endif
#endif /*CONFIG_ENABLE_PROXIMITY_DETECTION */
	} else {
		nRetVal = -ENODEV;
	}

	return nRetVal;
}

void DrvMainRemoveProcfsDirEntry(void)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (_gProcGloveModeEntry != NULL) {
		remove_proc_entry(PROC_NODE_GLOVE_MODE, _gProcDeviceEntry);
		_gProcGloveModeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_GLOVE_MODE);
	}

	if (_gProcOpenGloveModeEntry != NULL) {
		remove_proc_entry(PROC_NODE_OPEN_GLOVE_MODE, _gProcDeviceEntry);
		_gProcOpenGloveModeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_OPEN_GLOVE_MODE);
	}

	if (_gProcCloseGloveModeEntry != NULL) {
		remove_proc_entry(PROC_NODE_CLOSE_GLOVE_MODE, _gProcDeviceEntry);
		_gProcCloseGloveModeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_CLOSE_GLOVE_MODE);
	}

	if (_gProcLeatherSheathModeEntry != NULL) {
		remove_proc_entry(PROC_NODE_LEATHER_SHEATH_MODE, _gProcDeviceEntry);
		_gProcLeatherSheathModeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_LEATHER_SHEATH_MODE);
	}
#ifdef CONFIG_ENABLE_JNI_INTERFACE
	if (_gProcJniMethodEntry != NULL) {
		remove_proc_entry(PROC_NODE_JNI_NODE, _gProcDeviceEntry);
		_gProcJniMethodEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_JNI_NODE);
	}
#endif /*CONFIG_ENABLE_JNI_INTERFACE    */

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
	if (_gProcReportRateEntry != NULL) {
		remove_proc_entry(PROC_NODE_REPORT_RATE, _gProcDeviceEntry);
		_gProcReportRateEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_REPORT_RATE);
	}
#endif /*CONFIG_ENABLE_COUNT_REPORT_RATE */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	if (_gProcGestureWakeupModeEntry != NULL) {
		remove_proc_entry(PROC_NODE_GESTURE_WAKEUP_MODE, _gProcDeviceEntry);
		_gProcGestureWakeupModeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_GESTURE_WAKEUP_MODE);
	}
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
	if (_gProcGestureDebugModeEntry != NULL) {
		remove_proc_entry(PROC_NODE_GESTURE_DEBUG_MODE, _gProcDeviceEntry);
		_gProcGestureDebugModeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_GESTURE_DEBUG_MODE);
	}
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */
#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
	if (_gProcGestureInforModeEntry != NULL) {
		remove_proc_entry(PROC_NODE_GESTURE_INFORMATION_MODE, _gProcDeviceEntry);
		_gProcGestureInforModeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_GESTURE_INFORMATION_MODE);
	}
#endif /*CONFIG_ENABLE_GESTURE_INFORMATION_MODE */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

	if (_gProcFirmwareModeEntry != NULL) {
		remove_proc_entry(PROC_NODE_FIRMWARE_MODE, _gProcDeviceEntry);
		_gProcFirmwareModeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_MODE);
	}

	if (_gProcFirmwareSensorEntry != NULL) {
		remove_proc_entry(PROC_NODE_FIRMWARE_SENSOR, _gProcDeviceEntry);
		_gProcFirmwareSensorEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_SENSOR);
	}

	if (_gProcFirmwarePacketHeaderEntry != NULL) {
		remove_proc_entry(PROC_NODE_FIRMWARE_PACKET_HEADER, _gProcDeviceEntry);
		_gProcFirmwarePacketHeaderEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_PACKET_HEADER);
	}

	if (_gProcQueryFeatureSupportStatusEntry != NULL) {
		remove_proc_entry(PROC_NODE_QUERY_FEATURE_SUPPORT_STATUS, _gProcDeviceEntry);
		_gProcQueryFeatureSupportStatusEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_QUERY_FEATURE_SUPPORT_STATUS);
	}

	if (_gProcChangeFeatureSupportStatusEntry != NULL) {
		remove_proc_entry(PROC_NODE_CHANGE_FEATURE_SUPPORT_STATUS, _gProcDeviceEntry);
		_gProcChangeFeatureSupportStatusEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_CHANGE_FEATURE_SUPPORT_STATUS);
	}
#ifdef CONFIG_ENABLE_ITO_MP_TEST
	if (_gProcMpTestEntry != NULL) {
		remove_proc_entry(PROC_NODE_MP_TEST, _gProcDeviceEntry);
		_gProcMpTestEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_MP_TEST);
	}

	if (_gProcMpTestLogEntry != NULL) {
		remove_proc_entry(PROC_NODE_MP_TEST_LOG, _gProcDeviceEntry);
		_gProcMpTestLogEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_LOG);
	}

	if (_gProcMpTestFailChannelEntry != NULL) {
		remove_proc_entry(PROC_NODE_MP_TEST_FAIL_CHANNEL, _gProcDeviceEntry);
		_gProcMpTestFailChannelEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_FAIL_CHANNEL);
	}

	if (_gProcMpTestScopeEntry != NULL) {
		remove_proc_entry(PROC_NODE_MP_TEST_SCOPE, _gProcDeviceEntry);
		_gProcMpTestScopeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_SCOPE);
	}
#ifdef CONFIG_ENABLE_CHIP_TYPE_MSG28XX
	if (_gProcMpTestLogALLEntry != NULL) {
		remove_proc_entry(PROC_NODE_MP_TEST_LOG_ALL, _gProcDeviceEntry);
		_gProcMpTestLogALLEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_LOG_ALL);
	}
#endif /*CONFIG_ENABLE_CHIP_TYPE_MSG28XX */
#endif /*CONFIG_ENABLE_ITO_MP_TEST */

	if (_gProcFirmwareSetDQMemValueEntry != NULL) {
		remove_proc_entry(PROC_NODE_FIRMWARE_SET_DQMEM_VALUE, _gProcDeviceEntry);
		_gProcFirmwareSetDQMemValueEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_SET_DQMEM_VALUE);
	}

	if (_gProcFirmwareSmBusDebugEntry != NULL) {
		remove_proc_entry(PROC_NODE_FIRMWARE_SMBUS_DEBUG, _gProcDeviceEntry);
		_gProcFirmwareSmBusDebugEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_SMBUS_DEBUG);
	}

	if (_gProcFirmwareSetDebugValueEntry != NULL) {
		remove_proc_entry(PROC_NODE_FIRMWARE_SET_DEBUG_VALUE, _gProcDeviceEntry);
		_gProcFirmwareSetDebugValueEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_SET_DEBUG_VALUE);
	}

	if (_gProcFirmwareDebugEntry != NULL) {
		remove_proc_entry(PROC_NODE_FIRMWARE_DEBUG, _gProcDeviceEntry);
		_gProcFirmwareDebugEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_DEBUG);
	}

	if (_gProcSdCardFirmwareUpdateEntry != NULL) {
		remove_proc_entry(PROC_NODE_SD_CARD_FIRMWARE_UPDATE, _gProcDeviceEntry);
		_gProcSdCardFirmwareUpdateEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_SD_CARD_FIRMWARE_UPDATE);
	}

	if (_gProcSeLinuxLimitFirmwareUpdateEntry != NULL) {
		remove_proc_entry(PROC_NODE_SELINUX_LIMIT_FIRMWARE_UPDATE, _gProcDeviceEntry);
		_gProcSeLinuxLimitFirmwareUpdateEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_SELINUX_LIMIT_FIRMWARE_UPDATE);
	}

	if (_gProcForceFirmwareUpdateEntry != NULL) {
		remove_proc_entry(PROC_NODE_FORCE_FIRMWARE_UPDATE, _gProcDeviceEntry);
		_gProcForceFirmwareUpdateEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FORCE_FIRMWARE_UPDATE);
	}

	if (_gProcDeviceDriverVersionEntry != NULL) {
		remove_proc_entry(PROC_NODE_DEVICE_DRIVER_VERSION, _gProcDeviceEntry);
		_gProcDeviceDriverVersionEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_DEVICE_DRIVER_VERSION);
	}

	if (_gProcPlatformFirmwareVersionEntry != NULL) {
		remove_proc_entry(PROC_NODE_PLATFORM_FIRMWARE_VERSION, _gProcDeviceEntry);
		_gProcPlatformFirmwareVersionEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_PLATFORM_FIRMWARE_VERSION);
	}

	if (_gProcCustomerFirmwareVersionEntry != NULL) {
		remove_proc_entry(PROC_NODE_CUSTOMER_FIRMWARE_VERSION, _gProcDeviceEntry);
		_gProcCustomerFirmwareVersionEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_CUSTOMER_FIRMWARE_VERSION);
	}

	if (_gProcApkFirmwareUpdateEntry != NULL) {
		remove_proc_entry(PROC_NODE_FIRMWARE_UPDATE, _gProcDeviceEntry);
		_gProcApkFirmwareUpdateEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_UPDATE);
	}

	if (_gProcFirmwareDataEntry != NULL) {
		remove_proc_entry(PROC_NODE_FIRMWARE_DATA, _gProcDeviceEntry);
		_gProcFirmwareDataEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_DATA);
	}

	if (_gProcMpTestCustomisedEntry != NULL) {
		remove_proc_entry(PROC_NODE_MP_TEST_CUSTOMISED, _gProcDeviceEntry);
		_gProcMpTestCustomisedEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_CUSTOMISED);
	}

	if (_gProcChipTypeEntry != NULL) {
		remove_proc_entry(PROC_NODE_CHIP_TYPE, _gProcDeviceEntry);
		_gProcChipTypeEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_CHIP_TYPE);
	}

	if (_gProcDeviceEntry != NULL) {
		remove_proc_entry(PROC_NODE_DEVICE, _gProcMsTouchScreenMsg20xxEntry);
		_gProcDeviceEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_DEVICE);
	}

	if (_gProcMsTouchScreenMsg20xxEntry != NULL) {
		remove_proc_entry(PROC_NODE_MS_TOUCHSCREEN_MSG20XX, _gProcClassEntry);
		_gProcMsTouchScreenMsg20xxEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_MS_TOUCHSCREEN_MSG20XX);
	}

	if (_gProcClassEntry != NULL) {
		remove_proc_entry(PROC_NODE_CLASS, NULL);
		_gProcClassEntry = NULL;
		DBG(&g_I2cClient->dev, "Remove procfs file node(%s) OK!\n", PROC_NODE_CLASS);
	}
}

/*=============================================================*/
/*LOCAL FUNCTION DEFINITION*/
/*=============================================================*/
#ifdef CONFIG_TPD_MSTAR_TEST
int Mstar_save_failed_node_to_buffer(struct mstar_test_buffer *stp_test, char *tmp_buffer, int length)
{

	if (stp_test->node_failed_buffer == NULL ||
		(stp_test->node_failed_buffer_length + length) > TEST_RESULT_LENGTH) {
		pr_err("warning:buffer is null or buffer overflow, return");
		return -EPERM;
	}

	memcpy(stp_test->node_failed_buffer + stp_test->node_failed_buffer_length, tmp_buffer, length);
	stp_test->node_failed_buffer_length += length;
	stp_test->node_failed_count++;

	return 0;
}

int Mstar_save_failed_node(int iRow, int iCol)
{
	int i_len = 0;

	i_len = snprintf(g_mstar_test_buffer.temp_buffer, TEST_TEMP_LENGTH, ",%d,%d", iRow, iCol);
	Mstar_save_failed_node_to_buffer(&g_mstar_test_buffer, g_mstar_test_buffer.temp_buffer, i_len);
	return 0;
}

extern int _gSenseLineNum;
extern int _gDriveLineNum;
static int tpd_test_cmd_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	struct mstar_test_buffer *stp_test = &g_mstar_test_buffer;
	int buffer_length = 0;
	int i_len = 0;

	mutex_lock(&g_Mutex);

	i_len =
	    snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d", g_mstar_tptest_result, _gDriveLineNum, _gSenseLineNum,
		    stp_test->node_failed_count);

	buffer_length =
	    (stp_test->node_failed_buffer_length + 1) >
	    (PAGE_SIZE - i_len) ? (PAGE_SIZE - i_len - 1) : stp_test->node_failed_buffer_length;

	if (stp_test->node_failed_buffer != NULL && buffer_length > 0) {
		memcpy(buf + i_len, stp_test->node_failed_buffer, buffer_length);
		buf[buffer_length + i_len] = '\0';
	}

	MSTAR_TEST_DBG(" %s test cmd show:%s.", __func__, buf);

	num_read_chars = buffer_length + i_len;
	mutex_unlock(&g_Mutex);

	return num_read_chars;
}

static int tpd_test_cmd_store(struct tpd_classdev_t *cdev, const char *buf)
{
	unsigned long command = 0;
	int retval = -1;
	static int node_opened = 0;
	struct mstar_test_buffer *stp_test = &g_mstar_test_buffer;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
	snprintf(filepath, sizeof(filepath), "%s%s", g_str_ini_file_path, g_str_ini_filename);
	retval = kstrtoul(buf, 10, &command);
	if (retval) {
		MSTAR_TEST_DBG("invalid param:%s", buf);
		return 0;
	}
	MSTAR_TEST_DBG(" %s [func] command:%ld, ini filename:%s.\n", __func__, command, g_str_ini_filename);

	/*mutex_lock(&g_Mutex); */

	/*command 1:open node; 2:start test; 3:close node. */
	if (command == 1) {	/*open test node, alloc space etc...       */
		if (stp_test->node_failed_buffer == NULL) {
			stp_test->node_failed_buffer = kmalloc(TEST_RESULT_LENGTH, GFP_ATOMIC);
			stp_test->node_failed_buffer_length = 0;
			stp_test->node_failed_count = 0;
		}
		if (stp_test->temp_buffer == NULL) {
			stp_test->temp_buffer = kmalloc(TEST_TEMP_LENGTH, GFP_ATOMIC);
		}
		node_opened = 1;
	} else if (command == 2) {
		if (node_opened == 1) {	/*start test                */
			stp_test->node_failed_buffer_length = 0;
			stp_test->node_failed_count = 0;
			stp_test->test_result = g_mstar_tptest_result = 0;

			retval = startMPTest(CHIP_TYPE_MSG28XX, filepath);
			if (retval < 0)
				Msg28xxEndMPTest();
			else {
				save_test_data();
				Msg28xxEndMPTest();
			}
		} else {
			stp_test->test_result = 0;
			MSTAR_TEST_DBG("command:%ld,  open node before start test.", command);
		}
	} else if (command == 3) {	/*close test node, free space etc... */
		node_opened = 0;
		if (stp_test->node_failed_buffer != NULL) {
			kfree(stp_test->node_failed_buffer);
			stp_test->node_failed_buffer = NULL;
			stp_test->node_failed_buffer_length = 0;
			stp_test->node_failed_count = 0;
		}
		if (stp_test->temp_buffer != NULL) {
			kfree(stp_test->temp_buffer);
			stp_test->temp_buffer = NULL;
		}

	} else {
		MSTAR_TEST_DBG("invalid command %ld", command);
	}
	/*mutex_unlock(&g_Mutex);    */

	return 0;
}

static char *string_trim_tail(unsigned char *str_buf)
{
	int length = 0;
	int i = 0;

	length = strlen(str_buf);

	for (i = length - 1; i >= 0; i--) {
		if ((!isprint(str_buf[i])) || (' ' == str_buf[i])) {
			str_buf[i] = '\0';
		} else {
			break;
		}
	}

	return str_buf;
}

static char *dir_path_add_slash(unsigned char *str_buf)
{
	int length = 0;

	string_trim_tail(str_buf);

	length = strlen(str_buf);

	if ('/' != str_buf[length - 1]) {
		strlcat(str_buf, "/", PAGE_SIZE);
	}

	return str_buf;
}

static int tpd_test_save_file_path_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&g_Mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", mstar_str_save_file_path);

	mutex_unlock(&g_Mutex);

	return num_read_chars;
}

static int tpd_test_save_file_path_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(mstar_str_save_file_path, 0, sizeof(mstar_str_save_file_path));
	snprintf(mstar_str_save_file_path, 256, "%s", buf);

	dir_path_add_slash(mstar_str_save_file_path);

	MSTAR_TEST_DBG("save file path:%s.", mstar_str_save_file_path);

	return 0;
}

static int tpd_test_ini_file_path_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&g_Mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_file_path);

	mutex_unlock(&g_Mutex);

	return num_read_chars;
}

static int tpd_test_ini_file_path_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_str_ini_file_path, 0, sizeof(g_str_ini_file_path));
	snprintf(g_str_ini_file_path, 256, "%s", buf);

	dir_path_add_slash(g_str_ini_file_path);

	MSTAR_TEST_DBG("ini file path:%s.", g_str_ini_file_path);

	return 0;
}

static int tpd_test_filename_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&g_Mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_filename);

	mutex_unlock(&g_Mutex);

	return num_read_chars;
}

static int tpd_test_filename_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_str_ini_filename, 0, sizeof(g_str_ini_filename));
	snprintf(g_str_ini_filename, 128, "%s", buf);

	string_trim_tail(g_str_ini_filename);

	MSTAR_TEST_DBG("ini file name:%s.", g_str_ini_filename);

	return 0;
}

/*
static int tpd_test_node_data_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	int iLen = 0;

	mutex_lock(&g_Mutex);
	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);
	fts_test_funcs();

	iLen = tpd_test_get_tp_node_data(g_node_data_type, buf, 4096);

	num_read_chars = iLen;
	mutex_unlock(&g_Mutex);

	return num_read_chars;
}

static int tpd_test_node_data_store(struct tpd_classdev_t *cdev, const char *buf)
{
	int data_type = 0;
	int ret ;

	ret = kstrtoint(buf, 10, &data_type);
	if (ret) {
		DBG(&g_I2cClient->dev, "invalid param.\n");
		return ret;
	}
	MSTAR_TEST_DBG("%s type:%d .", __func__, data_type);

	mutex_lock(&g_Mutex);

	g_node_data_type = data_type;

	mutex_unlock(&g_Mutex);

	return 0;
}*/
extern int ms_getInidata(char *section, char *ItemName, char *returnValue);
extern int my_parser(char *path);
static int tpd_test_channel_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	int buffer[5] = { 0, 0, 0, 0, 0 };

	g_mstar_test_buffer.i_txNum = buffer[0] = 12;
	g_mstar_test_buffer.i_rxNum = buffer[1] = 24;
	num_read_chars =
	    snprintf(buf, PAGE_SIZE, "%d %d %d %d %d", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);

	return num_read_chars;
}

static int tpd_test_result_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "0x%x\n", g_mstar_tptest_result);

	return num_read_chars;
}

int _mstar_test_init(void)
{

	strlcpy(mstar_str_save_file_path, MSTAR_INI_FILE_PATH, 256);
	strlcpy(g_str_ini_file_path, MSTAR_INI_FILE_PATH, 256);
	strlcpy(g_str_ini_filename, "test.ini", 128);

	tpd_fw_cdev.tpd_test_set_save_filepath = tpd_test_save_file_path_store;
	tpd_fw_cdev.tpd_test_get_save_filepath = tpd_test_save_file_path_show;
	tpd_fw_cdev.tpd_test_set_ini_filepath = tpd_test_ini_file_path_store;
	tpd_fw_cdev.tpd_test_get_ini_filepath = tpd_test_ini_file_path_show;
	tpd_fw_cdev.tpd_test_set_filename = tpd_test_filename_store;
	tpd_fw_cdev.tpd_test_get_filename = tpd_test_filename_show;
	tpd_fw_cdev.tpd_test_set_cmd = tpd_test_cmd_store;
	tpd_fw_cdev.tpd_test_get_cmd = tpd_test_cmd_show;
/*tpd_fw_cdev.tpd_test_set_node_data_type = tpd_test_node_data_store;*/
/*tpd_fw_cdev.tpd_test_get_node_data = tpd_test_node_data_show;*/
	tpd_fw_cdev.tpd_test_get_channel_info = tpd_test_channel_show;
	tpd_fw_cdev.tpd_test_get_result = tpd_test_result_show;

	return 1;

}

#endif

static s32 _DrvMainCreateProcfsDirEntry(void)
{
	s32 nRetVal = 0;
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
	u8 *pGesturePath = NULL;
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */
	u8 *pDevicePath = NULL;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	_gProcClassEntry = proc_mkdir(PROC_NODE_CLASS, NULL);

	_gProcMsTouchScreenMsg20xxEntry = proc_mkdir(PROC_NODE_MS_TOUCHSCREEN_MSG20XX, _gProcClassEntry);

	_gProcDeviceEntry = proc_mkdir(PROC_NODE_DEVICE, _gProcMsTouchScreenMsg20xxEntry);

	_gProcMpTestCustomisedEntry =
	    proc_create(PROC_NODE_MP_TEST_CUSTOMISED, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcMpTestCustomised);
	if (_gProcMpTestCustomisedEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_MP_TEST_CUSTOMISED);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_CUSTOMISED);
	}

	_gProcChipTypeEntry = proc_create(PROC_NODE_CHIP_TYPE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcChipType);
	if (_gProcChipTypeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_CHIP_TYPE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_CHIP_TYPE);
	}

	_gProcFirmwareDataEntry =
	    proc_create(PROC_NODE_FIRMWARE_DATA, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcFirmwareData);
	if (_gProcFirmwareDataEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FIRMWARE_DATA);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_DATA);
	}

	_gProcApkFirmwareUpdateEntry =
	    proc_create(PROC_NODE_FIRMWARE_UPDATE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcApkFirmwareUpdate);
	if (_gProcApkFirmwareUpdateEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FIRMWARE_UPDATE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_UPDATE);
	}

	_gProcCustomerFirmwareVersionEntry =
	    proc_create(PROC_NODE_CUSTOMER_FIRMWARE_VERSION, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcCustomerFirmwareVersion);
	if (_gProcCustomerFirmwareVersionEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_CUSTOMER_FIRMWARE_VERSION);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_CUSTOMER_FIRMWARE_VERSION);
	}

	_gProcPlatformFirmwareVersionEntry =
	    proc_create(PROC_NODE_PLATFORM_FIRMWARE_VERSION, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcPlatformFirmwareVersion);
	if (_gProcPlatformFirmwareVersionEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_PLATFORM_FIRMWARE_VERSION);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_PLATFORM_FIRMWARE_VERSION);
	}

	_gProcDeviceDriverVersionEntry =
	    proc_create(PROC_NODE_DEVICE_DRIVER_VERSION, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcDeviceDriverVersion);
	if (_gProcDeviceDriverVersionEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_DEVICE_DRIVER_VERSION);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_DEVICE_DRIVER_VERSION);
	}

	_gProcSdCardFirmwareUpdateEntry =
	    proc_create(PROC_NODE_SD_CARD_FIRMWARE_UPDATE, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcSdCardFirmwareUpdate);
	if (_gProcSdCardFirmwareUpdateEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_SD_CARD_FIRMWARE_UPDATE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_SD_CARD_FIRMWARE_UPDATE);
	}

	_gProcFirmwareDebugEntry =
	    proc_create(PROC_NODE_FIRMWARE_DEBUG, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcFirmwareDebug);
	if (_gProcFirmwareDebugEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FIRMWARE_DEBUG);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_DEBUG);
	}

	_gProcFirmwareSetDebugValueEntry =
	    proc_create(PROC_NODE_FIRMWARE_SET_DEBUG_VALUE, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcFirmwareSetDebugValue);
	if (_gProcFirmwareSetDebugValueEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FIRMWARE_SET_DEBUG_VALUE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_SET_DEBUG_VALUE);
	}

	_gProcFirmwareSmBusDebugEntry =
	    proc_create(PROC_NODE_FIRMWARE_SMBUS_DEBUG, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcFirmwareSmBusDebug);
	if (_gProcFirmwareSmBusDebugEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FIRMWARE_SMBUS_DEBUG);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_SMBUS_DEBUG);
	}

	_gProcFirmwareSetDQMemValueEntry =
	    proc_create(PROC_NODE_FIRMWARE_SET_DQMEM_VALUE, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcFirmwareSetDQMemValue);
	if (_gProcFirmwareSetDQMemValueEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FIRMWARE_SET_DQMEM_VALUE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_SET_DQMEM_VALUE);
	}

#ifdef CONFIG_ENABLE_ITO_MP_TEST
	_gProcMpTestEntry = proc_create(PROC_NODE_MP_TEST, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcMpTest);
	if (_gProcMpTestEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_MP_TEST);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_MP_TEST);
	}

	_gProcMpTestLogEntry =
	    proc_create(PROC_NODE_MP_TEST_LOG, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcMpTestLog);
	if (_gProcMpTestLogEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_MP_TEST_LOG);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_LOG);
	}

	_gProcMpTestFailChannelEntry =
	    proc_create(PROC_NODE_MP_TEST_FAIL_CHANNEL, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcMpTestFailChannel);
	if (_gProcMpTestFailChannelEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_MP_TEST_FAIL_CHANNEL);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_FAIL_CHANNEL);
	}

	_gProcMpTestScopeEntry =
	    proc_create(PROC_NODE_MP_TEST_SCOPE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcMpTestScope);
	if (_gProcMpTestScopeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_MP_TEST_SCOPE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_SCOPE);
	}

#ifdef CONFIG_ENABLE_CHIP_TYPE_MSG28XX
	_gProcMpTestLogALLEntry =
	    proc_create(PROC_NODE_MP_TEST_LOG_ALL, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcMpTestLogAll);
	if (_gProcMpTestLogALLEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_MP_TEST_LOG_ALL);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_MP_TEST_LOG_ALL);
	}
#endif /*CONFIG_ENABLE_CHIP_TYPE_MSG28XX */
#endif /*CONFIG_ENABLE_ITO_MP_TEST */

	_gProcFirmwareModeEntry =
	    proc_create(PROC_NODE_FIRMWARE_MODE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcFirmwareMode);
	if (_gProcFirmwareModeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FIRMWARE_MODE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_MODE);
	}

	_gProcFirmwareSensorEntry =
	    proc_create(PROC_NODE_FIRMWARE_SENSOR, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcFirmwareSensor);
	if (_gProcFirmwareSensorEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FIRMWARE_SENSOR);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_SENSOR);
	}

	_gProcFirmwarePacketHeaderEntry =
	    proc_create(PROC_NODE_FIRMWARE_PACKET_HEADER, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcFirmwarePacketHeader);
	if (_gProcFirmwarePacketHeaderEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FIRMWARE_PACKET_HEADER);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FIRMWARE_PACKET_HEADER);
	}

	/* create a kset with the name of "kset_example" which is located under /sys/kernel/ */
	g_TouchKSet = kset_create_and_add("kset_example", NULL, kernel_kobj);
	if (!g_TouchKSet) {
		DBG(&g_I2cClient->dev, "*** kset_create_and_add() failed, nRetVal = %d ***\n", nRetVal);

		nRetVal = -ENOMEM;
	}

	g_TouchKObj = kobject_create();
	if (!g_TouchKObj) {
		DBG(&g_I2cClient->dev, "*** kobject_create() failed, nRetVal = %d ***\n", nRetVal);

		nRetVal = -ENOMEM;
		kset_unregister(g_TouchKSet);
		g_TouchKSet = NULL;
	}

	g_TouchKObj->kset = g_TouchKSet;

	nRetVal = kobject_add(g_TouchKObj, NULL, "%s", "kobject_example");
	if (nRetVal != 0) {
		DBG(&g_I2cClient->dev, "*** kobject_add() failed, nRetVal = %d ***\n", nRetVal);

		kobject_put(g_TouchKObj);
		g_TouchKObj = NULL;
		kset_unregister(g_TouchKSet);
		g_TouchKSet = NULL;
	}

	/* create the files associated with this kobject */
	nRetVal = sysfs_create_group(g_TouchKObj, &attr_group);
	if (nRetVal != 0) {
		DBG(&g_I2cClient->dev, "*** sysfs_create_file() failed, nRetVal = %d ***\n", nRetVal);

		kobject_put(g_TouchKObj);
		g_TouchKObj = NULL;
		kset_unregister(g_TouchKSet);
		g_TouchKSet = NULL;
	}

	pDevicePath = kobject_get_path(g_TouchKObj, GFP_KERNEL);
	DBG(&g_I2cClient->dev, "DEVPATH = %s\n", pDevicePath);

	_gProcQueryFeatureSupportStatusEntry =
	    proc_create(PROC_NODE_QUERY_FEATURE_SUPPORT_STATUS, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcQueryFeatureSupportStatus);
	if (_gProcQueryFeatureSupportStatusEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n",
		    PROC_NODE_QUERY_FEATURE_SUPPORT_STATUS);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_QUERY_FEATURE_SUPPORT_STATUS);
	}

	_gProcChangeFeatureSupportStatusEntry =
	    proc_create(PROC_NODE_CHANGE_FEATURE_SUPPORT_STATUS, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcChangeFeatureSupportStatus);
	if (_gProcChangeFeatureSupportStatusEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n",
		    PROC_NODE_CHANGE_FEATURE_SUPPORT_STATUS);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_CHANGE_FEATURE_SUPPORT_STATUS);
	}

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	_gProcGestureWakeupModeEntry =
	    proc_create(PROC_NODE_GESTURE_WAKEUP_MODE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcGestureWakeupMode);
	if (_gProcGestureWakeupModeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_GESTURE_WAKEUP_MODE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_GESTURE_WAKEUP_MODE);
	}

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
	_gProcGestureDebugModeEntry =
	    proc_create(PROC_NODE_GESTURE_DEBUG_MODE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcGestureDebugMode);
	if (_gProcGestureDebugModeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_GESTURE_DEBUG_MODE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_GESTURE_DEBUG_MODE);
	}

	/* create a kset with the name of "kset_gesture" which is located under /sys/kernel/ */
	g_GestureKSet = kset_create_and_add("kset_gesture", NULL, kernel_kobj);
	if (!g_GestureKSet) {
		DBG(&g_I2cClient->dev, "*** kset_create_and_add() failed, nRetVal = %d ***\n", nRetVal);
		nRetVal = -ENOMEM;
	}

	g_GestureKObj = kobject_create();
	if (!g_GestureKObj) {
		DBG(&g_I2cClient->dev, "*** kobject_create() failed, nRetVal = %d ***\n", nRetVal);

		nRetVal = -ENOMEM;
		kset_unregister(g_GestureKSet);
		g_GestureKSet = NULL;
	}

	g_GestureKObj->kset = g_GestureKSet;

	nRetVal = kobject_add(g_GestureKObj, NULL, "%s", "kobject_gesture");
	if (nRetVal != 0) {
		DBG(&g_I2cClient->dev, "*** kobject_add() failed, nRetVal = %d ***\n", nRetVal);

		kobject_put(g_GestureKObj);
		g_GestureKObj = NULL;
		kset_unregister(g_GestureKSet);
		g_GestureKSet = NULL;
	}

	/* create the files associated with this g_GestureKObj */
	nRetVal = sysfs_create_group(g_GestureKObj, &gestureattr_group);
	if (nRetVal != 0) {
		DBG(&g_I2cClient->dev, "*** sysfs_create_file() failed, nRetVal = %d ***\n", nRetVal);

		kobject_put(g_GestureKObj);
		g_GestureKObj = NULL;
		kset_unregister(g_GestureKSet);
		g_GestureKSet = NULL;
	}

	pGesturePath = kobject_get_path(g_GestureKObj, GFP_KERNEL);
	DBG(&g_I2cClient->dev, "DEVPATH = %s\n", pGesturePath);
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
	_gProcGestureInforModeEntry =
	    proc_create(PROC_NODE_GESTURE_INFORMATION_MODE, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcGestureInforMode);
	if (_gProcGestureInforModeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_GESTURE_INFORMATION_MODE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_GESTURE_INFORMATION_MODE);
	}
#endif /*CONFIG_ENABLE_GESTURE_INFORMATION_MODE */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
	_gProcReportRateEntry =
	    proc_create(PROC_NODE_REPORT_RATE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcReportRate);
	if (_gProcReportRateEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_REPORT_RATE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_REPORT_RATE);
	}
#endif /*CONFIG_ENABLE_COUNT_REPORT_RATE */

	_gProcGloveModeEntry = proc_create(PROC_NODE_GLOVE_MODE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcGloveMode);
	if (_gProcGloveModeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_GLOVE_MODE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_GLOVE_MODE);
	}

	_gProcOpenGloveModeEntry =
	    proc_create(PROC_NODE_OPEN_GLOVE_MODE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcOpenGloveMode);
	if (_gProcOpenGloveModeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_OPEN_GLOVE_MODE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_OPEN_GLOVE_MODE);
	}

	_gProcCloseGloveModeEntry =
	    proc_create(PROC_NODE_CLOSE_GLOVE_MODE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcCloseGloveMode);
	if (_gProcCloseGloveModeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_CLOSE_GLOVE_MODE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_CLOSE_GLOVE_MODE);
	}

	_gProcLeatherSheathModeEntry =
	    proc_create(PROC_NODE_LEATHER_SHEATH_MODE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcLeatherSheathMode);
	if (_gProcLeatherSheathModeEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_LEATHER_SHEATH_MODE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_LEATHER_SHEATH_MODE);
	}

#ifdef CONFIG_ENABLE_JNI_INTERFACE
	_gProcJniMethodEntry = proc_create(PROC_NODE_JNI_NODE, PROCFS_AUTHORITY, _gProcDeviceEntry, &_gProcJniMethod);
	if (_gProcJniMethodEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_JNI_NODE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_JNI_NODE);
	}
#endif /*CONFIG_ENABLE_JNI_INTERFACE */

	_gProcSeLinuxLimitFirmwareUpdateEntry =
	    proc_create(PROC_NODE_SELINUX_LIMIT_FIRMWARE_UPDATE, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcSeLinuxLimitFirmwareUpdate);
	if (_gProcSeLinuxLimitFirmwareUpdateEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n",
		    PROC_NODE_SELINUX_LIMIT_FIRMWARE_UPDATE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_SELINUX_LIMIT_FIRMWARE_UPDATE);
	}

	_gProcForceFirmwareUpdateEntry =
	    proc_create(PROC_NODE_FORCE_FIRMWARE_UPDATE, PROCFS_AUTHORITY, _gProcDeviceEntry,
			&_gProcForceFirmwareUpdate);
	if (_gProcForceFirmwareUpdateEntry == NULL) {
		DBG(&g_I2cClient->dev, "Failed to create procfs file node(%s)!\n", PROC_NODE_FORCE_FIRMWARE_UPDATE);
	} else {
		DBG(&g_I2cClient->dev, "Create procfs file node(%s) OK!\n", PROC_NODE_FORCE_FIRMWARE_UPDATE);
	}

	return nRetVal;
}

#ifdef CONFIG_ENABLE_HOTKNOT
/*register hotknot ioctl handler*/
static s32 _DrvMainHotknotRegistry(void)
{
	s32 nRetVal = 0;

	nRetVal = misc_register(&hotknot_miscdevice);
	if (nRetVal < 0) {
		DBG(&g_I2cClient->dev, "Failed to register misc device. Err:%d\n", nRetVal);
	}
	DBG(&g_I2cClient->dev, "*** Misc device registered ***\n");

	nRetVal = sysfs_create_group(&hotknot_miscdevice.this_device->kobj, &hotknot_attr_group);
	if (nRetVal < 0) {
		DBG(&g_I2cClient->dev, "Failed to create attribute group. Err:%d\n", nRetVal);
		/*misc_deregister( &hotknot_miscdevice ); */
	}
	DBG(&g_I2cClient->dev, "*** Attribute group created ***\n");

	mutex_init(&g_HKMutex);
	mutex_init(&g_QMutex);
	CreateQueue();
	CreateHotKnotMem();

	return nRetVal;
}
#endif /*CONFIG_ENABLE_HOTKNOT */
