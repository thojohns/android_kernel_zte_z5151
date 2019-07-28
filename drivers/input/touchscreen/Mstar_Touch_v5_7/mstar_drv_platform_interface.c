/*******************************************************************************

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
 *@file    mstar_drv_platform_interface.c
 *
 *@brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
/*INCLUDE FILE*/
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_utility_adaption.h"

#ifdef CONFIG_ENABLE_HOTKNOT
#include "mstar_drv_hotknot.h"
#endif /*CONFIG_ENABLE_HOTKNOT */
#include "tpd_sys.h"
/*=============================================================*/
/*EXTERN VARIABLE DECLARATION*/
/*=============================================================*/
#ifdef CONFIG_TPD_MSTAR_TEST
extern int _mstar_test_init(void);
#endif
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u32 g_GestureWakeupMode[2];
extern u8 g_GestureWakeupFlag;

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
extern u8 g_GestureDebugFlag;
extern u8 g_GestureDebugMode;
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern u8 g_EnableTpProximity;
#endif /*CONFIG_ENABLE_PROXIMITY_DETECTION */

extern u8 g_IsEnableGloveMode;
extern u8 g_IsEnableLeatherSheathMode;

extern u8 g_IsUpdateFirmware;

extern struct input_dev *g_InputDevice;
extern struct i2c_client *g_I2cClient;

#ifdef CONFIG_ENABLE_HOTKNOT
extern u8 g_HotKnotState;
extern u32 SLAVE_I2C_ID_DWI2C;
#endif /*CONFIG_ENABLE_HOTKNOT */

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
extern u8 g_ForceUpdate;
#endif /*CONFIG_ENABLE_CHARGER_DETECTION */

#ifdef CONFIG_ENABLE_ESD_PROTECTION
extern int g_IsEnableEsdCheck;
extern struct delayed_work g_EsdCheckWork;
extern struct workqueue_struct *g_EsdCheckWorkqueue;
#endif /*CONFIG_ENABLE_ESD_PROTECTION */

extern u8 IS_FIRMWARE_DATA_LOG_ENABLED;
/*zhangjian add*/
extern void DrvIcFwLyrGetCustomerFirmwareVersion(u16 *pMajor, u16 *pMinor, u8 **ppVersion);
extern u8 DrvIcFwLyrGetChipType(void);
extern u8 g_ChipType;
int Mstar_wakeup_gesture_enable = 0;
extern void BG_enable_irq_wake(bool enable);
struct work_struct mstar_tpd_resume_work;
struct workqueue_struct *mstar_tpd_resume_wq;
static bool mstar_tpd_suspend_flag = false;
/*=============================================================*/
/*GLOBAL VARIABLE DEFINITION*/
/*=============================================================*/

/*=============================================================*/
/*LOCAL VARIABLE DEFINITION*/
/*=============================================================*/

#ifdef CONFIG_ENABLE_HOTKNOT
static u8 _gAMStartCmd[4] = { HOTKNOT_SEND, ADAPTIVEMOD_BEGIN, 0, 0 };
#endif /*CONFIG_ENABLE_HOTKNOT */
#ifdef CONFIG_ENABLE_NOTIFIER_FB
void mstar_tp_resume(void)
{
	DBG(&g_I2cClient->dev, "*** %s() TP Resume ***\n", __func__);

	if (g_IsUpdateFirmware != 0) {/*Check whether update frimware is finished */
		DBG(&g_I2cClient->dev, "Not allow to power on/off touch ic while update firmware.\n");
		return;
	}
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	if (g_EnableTpProximity == 1) {
		DBG(&g_I2cClient->dev, "g_EnableTpProximity = %d\n", g_EnableTpProximity);
		return;
	}
#endif /*CONFIG_ENABLE_PROXIMITY_DETECTION */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE
	    && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /*CONFIG_ENABLE_HOTKNOT */
	{
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
		if (g_GestureDebugMode == 1) {
			DrvIcFwLyrCloseGestureDebugMode();
		}
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */

		if (g_GestureWakeupFlag == 1) {
			BG_enable_irq_wake(false);
			DrvIcFwLyrCloseGestureWakeup();
		} else {
			DrvPlatformLyrEnableFingerTouchReport();
		}
	}
#ifdef CONFIG_ENABLE_HOTKNOT
	else {	/*Enable touch in hotknot transfer mode */

		DrvPlatformLyrEnableFingerTouchReport();
	}
#endif /*CONFIG_ENABLE_HOTKNOT */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE
	    && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /*CONFIG_ENABLE_HOTKNOT        */
	{
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
		DrvPlatformLyrTouchDeviceRegulatorPowerOn(true);
#endif /*CONFIG_ENABLE_REGULATOR_POWER_ON               */
		DrvPlatformLyrTouchDevicePowerOn();
	}
#ifdef CONFIG_ENABLE_CHARGER_DETECTION
	{
		u8 szChargerStatus[20] = { 0 };

		DrvCommonReadFile(POWER_SUPPLY_BATTERY_STATUS_PATCH, szChargerStatus, 20);

		DBG(&g_I2cClient->dev, "*** Battery Status : %s ***\n", szChargerStatus);

		g_ForceUpdate = 1;	/*Set flag to force update charger status */

		if (strnstr(szChargerStatus, "Charging", sizeof(szChargerStatus)) != NULL
			|| strnstr(szChargerStatus, "Full", sizeof(szChargerStatus)) != NULL
			|| strnstr(szChargerStatus, "Fully charged", sizeof(szChargerStatus)) != NULL) {
			DrvFwCtrlChargerDetection(1);	/*charger plug-in */
		} else {	/*Not charging */

			DrvFwCtrlChargerDetection(0);	/*charger plug-out */
		}

		g_ForceUpdate = 0;	/*Clear flag after force update charger status */
	}
#endif /*CONFIG_ENABLE_CHARGER_DETECTION */

	if (g_IsEnableGloveMode == 1) {
		DrvIcFwLyrOpenGloveMode();
	}

	if (g_IsEnableLeatherSheathMode == 1) {
		DrvIcFwLyrOpenLeatherSheathMode();
	}

	if (IS_FIRMWARE_DATA_LOG_ENABLED) {
/*Mark this function call for avoiding device driver may spend longer time to resume from suspend state. */
		DrvIcFwLyrRestoreFirmwareModeToLogDataMode();
	}
	/*IS_FIRMWARE_DATA_LOG_ENABLED */
#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
	DrvPlatformLyrEnableFingerTouchReport();
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_ESD_PROTECTION
	g_IsEnableEsdCheck = 1;
	queue_delayed_work(g_EsdCheckWorkqueue, &g_EsdCheckWork, ESD_PROTECT_CHECK_PERIOD);
#endif /*CONFIG_ENABLE_ESD_PROTECTION */
}
void mstar_tp_suspend(void)
{
	DBG(&g_I2cClient->dev, "*** %s() TP Suspend ***\n", __func__);

#ifdef CONFIG_ENABLE_ESD_PROTECTION
	g_IsEnableEsdCheck = 0;
	cancel_delayed_work_sync(&g_EsdCheckWork);
#endif /*CONFIG_ENABLE_ESD_PROTECTION */

	if (g_IsUpdateFirmware != 0) {	/*Check whether update frimware is finished */
		DBG(&g_I2cClient->dev, "Not allow to power on/off touch ic while update firmware.\n");
		return;
	}
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	if (g_EnableTpProximity == 1) {
		DBG(&g_I2cClient->dev, "g_EnableTpProximity = %d\n", g_EnableTpProximity);
		return;
	}
#endif /*CONFIG_ENABLE_PROXIMITY_DETECTION */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE
	    && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /*CONFIG_ENABLE_HOTKNOT */
	{
		if (Mstar_wakeup_gesture_enable) {
			if (g_GestureWakeupMode[0] != 0x00000000
			    || g_GestureWakeupMode[1] != 0x00000000) {
				BG_enable_irq_wake(true);
				DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
				return;
			}
		}
	}
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState == HOTKNOT_BEFORE_TRANS_STATE || g_HotKnotState == HOTKNOT_TRANS_STATE
	    || g_HotKnotState == HOTKNOT_AFTER_TRANS_STATE) {
		IicWriteData(SLAVE_I2C_ID_DWI2C, &_gAMStartCmd[0], 4);
	}
#endif /*CONFIG_ENABLE_HOTKNOT */

	DrvPlatformLyrFingerTouchReleased(0, 0, 0);	/*Send touch end for clearing point touch */
	input_sync(g_InputDevice);

	DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE
	    && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /*CONFIG_ENABLE_HOTKNOT        */
	{
		DrvPlatformLyrTouchDevicePowerOff();
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
		DrvPlatformLyrTouchDeviceRegulatorPowerOn(false);
#endif /*CONFIG_ENABLE_REGULATOR_POWER_ON */
	}
}
static void mstar_tpd_resume_work_callback(struct work_struct *work)
{
	pr_notice("%s func in\n", __func__);
	mstar_tp_resume();
}
/*=============================================================*/
/*GLOBAL FUNCTION DEFINITION*/
/*=============================================================*/

int MsDrvInterfaceTouchDeviceFbNotifierCallback(struct notifier_block *pSelf, unsigned long nEvent, void *pData)
{
	struct fb_event *pEventData = pData;
	int *pBlank;
	int err = 0;

	if (pEventData && pEventData->data && nEvent == FB_EVENT_BLANK) {
		pBlank = pEventData->data;

		if (*pBlank == FB_BLANK_UNBLANK) {
			if (mstar_tpd_suspend_flag) {
				err = queue_work(mstar_tpd_resume_wq, &mstar_tpd_resume_work);
				if (!err) {
					pr_notice("start fb_tpd_resume_wq failed\n");
					return err;
				}
			}
		} else if (*pBlank == FB_BLANK_POWERDOWN) {
			cancel_work_sync(&mstar_tpd_resume_work);
			mstar_tp_suspend();
			mstar_tpd_suspend_flag = true;
		}
	}

	return 0;
}

#else

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
void MsDrvInterfaceTouchDeviceSuspend(struct device *pDevice)
#else
void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend)
#endif				/*CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD */
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

#ifdef CONFIG_ENABLE_ESD_PROTECTION
	g_IsEnableEsdCheck = 0;
	cancel_delayed_work_sync(&g_EsdCheckWork);
#endif /*CONFIG_ENABLE_ESD_PROTECTION */

	if (g_IsUpdateFirmware != 0) {	/*Check whether update frimware is finished */
		DBG(&g_I2cClient->dev, "Not allow to power on/off touch ic while update firmware.\n");
		return;
	}
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	if (g_EnableTpProximity == 1) {
		DBG(&g_I2cClient->dev, "g_EnableTpProximity = %d\n", g_EnableTpProximity);
		return;
	}
#endif /*CONFIG_ENABLE_PROXIMITY_DETECTION */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE
	    && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /*CONFIG_ENABLE_HOTKNOT */
	{
		if (Mstar_wakeup_gesture_enable) {	/*zhangjian add */
			if (g_GestureWakeupMode[0] != 0x00000000 || g_GestureWakeupMode[1] != 0x00000000) {
				BG_enable_irq_wake(true);	/*zhangjian add */
				DrvIcFwLyrOpenGestureWakeup(&g_GestureWakeupMode[0]);
				return;
			}
		}
	}
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState == HOTKNOT_BEFORE_TRANS_STATE || g_HotKnotState == HOTKNOT_TRANS_STATE
	    || g_HotKnotState == HOTKNOT_AFTER_TRANS_STATE) {
		IicWriteData(SLAVE_I2C_ID_DWI2C, &_gAMStartCmd[0], 4);
	}
#endif /*CONFIG_ENABLE_HOTKNOT  */

	DrvPlatformLyrFingerTouchReleased(0, 0, 0);	/*Send touch end for clearing point touch */
	input_sync(g_InputDevice);

	DrvPlatformLyrDisableFingerTouchReport();

#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE
	    && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /*CONFIG_ENABLE_HOTKNOT        */
	{
		DrvPlatformLyrTouchDevicePowerOff();
#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
		DrvPlatformLyrTouchDeviceRegulatorPowerOn(false);
#endif /*CONFIG_ENABLE_REGULATOR_POWER_ON               */
#endif /*CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD */
	}
}

#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
void MsDrvInterfaceTouchDeviceResume(struct device *pDevice)
#else
void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend)
#endif				/*CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD */
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	if (g_IsUpdateFirmware != 0) {	/*Check whether update frimware is finished */
		DBG(&g_I2cClient->dev, "Not allow to power on/off touch ic while update firmware.\n");
		return;
	}
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
	if (g_EnableTpProximity == 1) {
		DBG(&g_I2cClient->dev, "g_EnableTpProximity = %d\n", g_EnableTpProximity);
		return;
	}
#endif /*CONFIG_ENABLE_PROXIMITY_DETECTION */

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE
	    && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /*CONFIG_ENABLE_HOTKNOT */
	{
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
		if (g_GestureDebugMode == 1) {
			DrvIcFwLyrCloseGestureDebugMode();
		}
#endif /*CONFIG_ENABLE_GESTURE_DEBUG_MODE */

		if (g_GestureWakeupFlag == 1) {
			BG_enable_irq_wake(false);	/*zhangjian ad */
			DrvIcFwLyrCloseGestureWakeup();
		} else {
			DrvPlatformLyrEnableFingerTouchReport();
		}
	}
#ifdef CONFIG_ENABLE_HOTKNOT
	else {			/*Enable touch in hotknot transfer mode */

		DrvPlatformLyrEnableFingerTouchReport();
	}
#endif /*CONFIG_ENABLE_HOTKNOT */
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_HOTKNOT
	if (g_HotKnotState != HOTKNOT_BEFORE_TRANS_STATE && g_HotKnotState != HOTKNOT_TRANS_STATE
	    && g_HotKnotState != HOTKNOT_AFTER_TRANS_STATE)
#endif /*CONFIG_ENABLE_HOTKNOT        */
	{
#ifdef CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
		DrvPlatformLyrTouchDeviceRegulatorPowerOn(true);
#endif /*CONFIG_ENABLE_REGULATOR_POWER_ON               */
#endif /*CONFIG_PLATFORM_USE_ANDROID_SDK_6_UPWARD */
		DrvPlatformLyrTouchDevicePowerOn();
	}
#ifdef CONFIG_ENABLE_CHARGER_DETECTION
	{
		u8 szChargerStatus[20] = { 0 };

		DrvCommonReadFile(POWER_SUPPLY_BATTERY_STATUS_PATCH, szChargerStatus, 20);

		DBG(&g_I2cClient->dev, "*** Battery Status : %s ***\n", szChargerStatus);

		g_ForceUpdate = 1;	/*Set flag to force update charger status */

		if (strnstr(szChargerStatus, "Charging", sizeof(szChargerStatus)) != NULL
			|| strnstr(szChargerStatus, "Full", sizeof(szChargerStatus)) != NULL
			|| strnstr(szChargerStatus, "Fully charged", sizeof(szChargerStatus)) != NULL) {
			DrvFwCtrlChargerDetection(1);	/*charger plug-in */
		} else {	/*Not charging */

			DrvFwCtrlChargerDetection(0);	/*charger plug-out */
		}

		g_ForceUpdate = 0;	/*Clear flag after force update charger status */
	}
#endif /*CONFIG_ENABLE_CHARGER_DETECTION */

	if (g_IsEnableGloveMode == 1) {
		DrvIcFwLyrOpenGloveMode();
	}

	if (g_IsEnableLeatherSheathMode == 1) {
		DrvIcFwLyrOpenLeatherSheathMode();
	}

	if (IS_FIRMWARE_DATA_LOG_ENABLED) {
		/*Mark this function call for avoiding device driver may spend longer time
		to resume from suspend state. */
		DrvIcFwLyrRestoreFirmwareModeToLogDataMode();
	}
	/*IS_FIRMWARE_DATA_LOG_ENABLED */
#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
	DrvPlatformLyrEnableFingerTouchReport();
#endif /*CONFIG_ENABLE_GESTURE_WAKEUP */

#ifdef CONFIG_ENABLE_ESD_PROTECTION
	g_IsEnableEsdCheck = 1;
	queue_delayed_work(g_EsdCheckWorkqueue, &g_EsdCheckWork, ESD_PROTECT_CHECK_PERIOD);
#endif /*CONFIG_ENABLE_ESD_PROTECTION */
}

#endif /*CONFIG_ENABLE_NOTIFIER_FB */

/*zhangjian add for TP info read*/
static int tpd_init_tpinfo(struct tpd_classdev_t *cdev)
{
	u16 FW_Major = 0, FW_Minor = 0;
	u8 Chiptype = 0;
	u8 *FW_Version = NULL;

	DrvIcFwLyrGetCustomerFirmwareVersion(&FW_Major, &FW_Minor, &FW_Version);
	Chiptype = g_ChipType;	/*DrvIcFwLyrGetChipType(); */
	strlcpy(cdev->ic_tpinfo.tp_name, "Mstar", sizeof(cdev->ic_tpinfo.tp_name));
	cdev->ic_tpinfo.chip_model_id = 7;
	cdev->ic_tpinfo.chip_part_id = Chiptype;
	cdev->ic_tpinfo.module_id = FW_Major;
	cdev->ic_tpinfo.chip_ver = 0;
	cdev->ic_tpinfo.firmware_ver = FW_Minor;
	cdev->ic_tpinfo.i2c_type = 0;
	cdev->ic_tpinfo.i2c_addr = 0x26;
	return 0;
}

static int tpd_get_wakegesture(struct tpd_classdev_t *cdev)
{
	cdev->b_gesture_enable = Mstar_wakeup_gesture_enable;
	return 0;
}

static int tpd_enable_wakegesture(struct tpd_classdev_t *cdev, int enable)
{
	Mstar_wakeup_gesture_enable = enable;
	return enable;
}

static int tpd_register_fw_class(void)
{
	tpd_fw_cdev.get_tpinfo = tpd_init_tpinfo;
	tpd_fw_cdev.get_gesture = tpd_get_wakegesture;
	tpd_fw_cdev.wake_gesture = tpd_enable_wakegesture;
	return 0;
}

/*add end*/
/*add end*/
/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
	s32 nRetVal = 0;

	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	DrvPlatformLyrVariableInitialize();

	DrvPlatformLyrTouchDeviceRequestGPIO(pClient);

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
	DrvPlatformLyrTouchDeviceRegulatorPowerOn(true);
#endif /*CONFIG_ENABLE_REGULATOR_POWER_ON */

	DrvPlatformLyrTouchDevicePowerOn();

	nRetVal = DrvMainTouchDeviceInitialize();
	if (nRetVal == -ENODEV) {
		DrvPlatformLyrTouchDeviceRemove(pClient);

		return nRetVal;
	}

	DrvPlatformLyrInputDeviceInitialize(pClient);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
	DmaAlloc();		/*DmaAlloc() shall be called after DrvPlatformLyrInputDeviceInitialize() */
#endif /*CONFIG_ENABLE_DMA_IIC */
#endif /*CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM */

	DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler();

	DrvPlatformLyrTouchDeviceRegisterEarlySuspend();

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
	DrvIcFwLyrCheckFirmwareUpdateBySwId();
#endif /*CONFIG_UPDATE_FIRMWARE_BY_SW_ID */

#ifdef CONFIG_ENABLE_ESD_PROTECTION
	INIT_DELAYED_WORK(&g_EsdCheckWork, DrvPlatformLyrEsdCheck);
	g_EsdCheckWorkqueue = create_workqueue("esd_check");
	queue_delayed_work(g_EsdCheckWorkqueue, &g_EsdCheckWork, ESD_PROTECT_CHECK_PERIOD);
#endif /*CONFIG_ENABLE_ESD_PROTECTION */
	mstar_tpd_resume_wq = create_singlethread_workqueue("mstar_touch_resume");
	INIT_WORK(&mstar_tpd_resume_work, mstar_tpd_resume_work_callback);
	mstar_tpd_suspend_flag = 0;
	tpd_register_fw_class();	/*zhangjian add */
#ifdef CONFIG_TPD_MSTAR_TEST
	_mstar_test_init();
#endif
	tpd_fw_cdev.TP_have_registered = true;	/*zhangjian add for TP compat */
	DBG(&g_I2cClient->dev, "*** MStar touch driver registered ***\n");

	return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	return DrvPlatformLyrTouchDeviceRemove(pClient);
}

void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
	DBG(&g_I2cClient->dev, "*** %s() ***\n", __func__);

	DrvPlatformLyrSetIicDataRate(pClient, nIicDataRate);
}
