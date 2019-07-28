#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/ctype.h>

#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "../mstar_drv_common.h"
#include "../mstar_drv_platform_porting_layer.h"
#include "../mstar_drv_utility_adaption.h"

#define CHIP_TYPE_MSG28XX   (0x85)	/*EX. MSG2856 */
#define CHIP_TYPE_MSG28XXA   (0xBF)	/*EX. MSG2856 */

#define EMEM_SIZE_MSG28XX (1024*130)
#define EMEM_SIZE_MSG22XX ((1024*48) + 512)
#define EMEM_TYPE_ALL_BLOCKS	0x00
#define EMEM_TYPE_MAIN_BLOCK	0x01
#define EMEM_TYPE_INFO_BLOCK	0x02

#define MAX_CHANNEL_DRV  30
#define MAX_CHANNEL_SEN  20
#define	MAX_CHANNEL_NUM_28XX	60
#define MAX_CHANNEL_NUM_30XX    49
#define MAX_AFE_NUM_30XX    14
#define MAX_AFE_NUM_28XX    13
#define	TEST_ITEM_NUM 8
/*Camaro : 14AFE * 17DRI * 8SF, Cayenne: 13AFE * 15DRI * 6SF */
#define MAX_MUTUAL_NUM  1904

#define KEY_SEPARATE	0x5566
#define	KEY_COMBINE		0x7788

/*MPTest Result Items*/
#define MPTEST_RESULT                   0
#define MPTEST_SCOPE                    1
#define OPEN_TEST_DATA                  2
#define OPEN_TEST_FAIL_CHANNEL          3
#define SHORT_TEST_DATA                 4
#define SHORT_TEST_FAIL_CHANNEL         5
#define WATERPROOF_TEST_DATA            6
#define WATERPROOF_TEST_FAIL_CHANNEL    7

/*MPTest Result*/
#define ITO_NO_TEST                   0
#define ITO_TEST_OK                   1
#define ITO_TEST_FAIL                 2
#define ITO_TEST_GET_TP_TYPE_ERROR    3
#define ITO_TEST_UNDEFINED_ERROR      4

/*int[] location in Report(MPTEST_RESULT)*/
#define OPEN_TEST_RESULT          0
#define SHORT_TEST_RESULT         1
#define WATERPROOF_TEST_RESULT    2

#define MIN(a, b) (((a) < (b))?(a):(b))
#define	MAX(a, b) (((a) > (b))?(a):(b))

#define NULL_DATA -3240
#define PIN_NO_ERROR 0xFFFF
#define UN_USE_SENSOR 0x5AA5

#define IIR_MAX 32600
#define PIN_UN_USE 0xABCD

#define DISABLE_DOUBLE  1

#define PARSER_MAX_CFG_BUF          512
#define PARSER_MAX_KEY_NUM	        600
#define PARSER_MAX_KEY_NAME_LEN	    100
#define PARSER_MAX_KEY_VALUE_LEN	2000

#define TEST_PASS			0
#define TEST_BEYOND_MAX_LIMIT		0x0001
#define TEST_BEYOND_MIN_LIMIT		0x0002
#define TEST_BEYOND_ACCORD_LIMIT	0x0004
#define TEST_BEYOND_OFFSET_LIMIT	0x0008
#define TEST_BEYOND_JITTER_LIMIT	0x0010
#define TEST_KEY_BEYOND_MAX_LIMIT	0x0020
#define TEST_KEY_BEYOND_MIN_LIMIT	0x0040
#define TEST_MODULE_TYPE_ERR		0x0080
#define TEST_VERSION_ERR			0x0100
#define TEST_GT_OPEN				0x0200
#define TEST_GT_SHORT				0x0400
#define TEST_BEYOND_UNIFORMITY_LIMIT 0x0800
#define TEST_BEYOND_FLASH_LAYER_LIMIT  0x1000
#define TEST_BETWEEN_ACCORD_AND_LINE  0x2000

typedef struct _ST_INI_FILE_DATA {
	char pSectionName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyValue[PARSER_MAX_KEY_VALUE_LEN];
	int iSectionNameLen;
	int iKeyNameLen;
	int iKeyValueLen;
} ST_INI_FILE_DATA;

struct _TP_INFO {
	char DriverVersion[32];
	char FwVersion[32];
	char PlatformVersion[32];
	char MainBlockFWVersion[32];
	char InfoBlockFWVersion[32];
	char ChipType;
};

typedef struct {
	u16 X;
	u16 Y;
} MutualMapping_t;

typedef struct {
	char *sSupportIC;
	int bmDKVerify;
	int bCurrentTest;
	int bChipVerify;
	int bFWUpdate;
	int bFWTest;
	int bOpenTest;
	int bShortTest;
	int bWpTest;
	int bFunctionTest;
	int bAutoStart;
	int bAutoMation;
	int bTriggerMode;
	int bTSMode;
	int bTSEnable;
	int bPhaseKTest;
} MutualUIConfig_t;

typedef struct {
	u16 persentDC_VA_Range;
	u16 persentDC_VA_Ratio;

	u16 persentDC_Border_Ratio;
	u16 persentDC_VA_Range_up;
	u16 persentDC_VA_Ratio_up;

	u16 persentDG_Range;
	u16 persentDG_Ratio;
	u16 persentDG_Range_up;
	u16 persentDG_Ratio_up;
	u16 persentWater_DG_Range;
} MutualToast_t;

typedef struct {
	u16 numKey;
	u16 numKeyLine;
	u16 numDummy;
	u16 numTriangle;
	u16 KeyDrv;
	u16 KEY_CH;
	u16 KeyDrv_o;
	int thrsShort;
	int thrsICpin;
	int thrsOpen;
	int thrsWater;

	u16 numSen;
	u16 numDrv;
	u16 numGr;
	MutualMapping_t *mapping;
} MutualSensor_t;

typedef struct {
	MutualUIConfig_t UIConfig;
	int logResult;
	int logFWResult;

	int Enable;

	char *ana_version;
	char *project_name;
	char *binname;
	char *versionFW;
	u16 slaveI2cID;
	char *stationNow;
	char *inipassword;
	u16 Mutual_Key;
	u16 Pattern_type;
	u16 Pattern_model;

	int Crc_check;

	MutualSensor_t sensorInfo;
	MutualToast_t ToastInfo;
	int FPC_threshold;
	int KeyDC_threshold;
	int KEY_Timer;
	int Open_test_csub;
	int Open_test_cfb;
	int Open_mode;
	int Open_fixed_carrier;
	int Open_fixed_carrier1;
	int Open_test_chargepump;
	int inverter_mode;
#ifndef DISABLE_DOUBLE
	double Current_threshold;
	double CurrentThreshold_Powerdown;
#else
	int Current_threshold;
	int CurrentThreshold_Powerdown;
#endif

	int OPEN_Charge;
	int OPEN_Dump;
	int SHORT_Charge;
	int SHORT_Dump1;
	int SHORT_Dump2;
	int Water_Charge;
	int Water_Dump;

	int *KeySen;
#ifndef DISABLE_DOUBLE
	double *Goldensample_CH_0_Max_Avg;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	double *Goldensample_CH_0_Max;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	double *Goldensample_CH_0;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	double *Goldensample_CH_0_Min;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	double *Goldensample_CH_0_Min_Avg;	/*[ana26xxM.MAX_MUTUAL_NUM]; */

	double *PhaseGolden_Max;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	double *PhaseGolden;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	double *PhaseGolden_Min;	/*[ana26xxM.MAX_MUTUAL_NUM]; */

	double *PhaseWaterGolden_Max;
	double *PhaseWaterGolden;
	double *PhaseWaterGolden_Min;
#else
	int *Goldensample_CH_0_Max_Avg;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	int *Goldensample_CH_0_Max;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	int *Goldensample_CH_0;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	int *Goldensample_CH_0_Min;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	int *Goldensample_CH_0_Min_Avg;	/*[ana26xxM.MAX_MUTUAL_NUM]; */

	int *PhaseGolden_Max;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	int *PhaseGolden;	/*[ana26xxM.MAX_MUTUAL_NUM]; */
	int *PhaseGolden_Min;	/*[ana26xxM.MAX_MUTUAL_NUM]; */

	int *PhaseWaterGolden_Max;
	int *PhaseWaterGolden;
	int *PhaseWaterGolden_Min;
#endif

	u16 *PAD2Sense;
	u16 *PAD2Drive;
	u16 *PAD2GR;

	u16 *phase_freq;
	u16 freq_num;
	u16 phase_time;
	u16 band_num;
	u16 *pgd;
	u16 *water_pgd;
	u16 *water_sense;
	u16 *water_drive;
	u16 charge_pump;
	u16 raw_type;
	u16 noise_thd;
	u16 sub_frame;
	u16 afe_num;
	u16 phase_sen_num;
	u16 *phase_sense;
	u16 water_sen_num;
	u16 water_drv_num;
	int update_bin;
	int force_phaseK;
	int update_info;
	int log_phasek;
	int border_drive_phase;
	int sw_calibration;
	u8 phase_version;
	u8 mapping_version;
	int deep_standby;
	int deep_standby_timeout;
	int Open_KeySettingByFW;
} MutualMpTest_t;

typedef struct {
	int nOpenResult;
	int nShortResult;
	int nWaterProofResult;
	int nRetry;

	int nRatioAvg_max;
#ifndef DISABLE_DOUBLE
	double nRatioAvg_min;
	double nBorder_RatioAvg_max;
	double nBorder_RatioAvg_min;
#else
	int nRatioAvg_min;
	int nBorder_RatioAvg_max;
	int nBorder_RatioAvg_min;
#endif

	int *pOpenResultData;
	int *pOpenFailChannel;
	int *pOpenRatioFailChannel;
	int *pShortResultData;
#ifndef DISABLE_DOUBLE
	double *pShortRData;
#else
	int *pShortRData;
#endif
	int *pICPinShortResultData;
#ifndef DISABLE_DOUBLE
	double *pICPinShortRData;
#else
	int *pICPinShortRData;
#endif
	int *pICPinChannel;
	int *pShortFailChannel;
	int *pICPinShortFailChannel;
	int *pWaterProofResultData;
	int *pWaterProofFailChannel;

	int *pCheck_Fail;
#ifndef DISABLE_DOUBLE
	double *pResult_DeltaC;
	double *pGolden_CH_Max_Avg;
	double *pGolden_CH_Min_Avg;
	double *pGolden_CH_Max;
	double *pGolden_CH_Min;
	double *pGolden_CH;
#else
	int *pResult_DeltaC;
	int *pGolden_CH_Max_Avg;
	int *pGolden_CH_Min_Avg;
	int *pGolden_CH_Max;
	int *pGolden_CH_Min;
	int *pGolden_CH;
#endif
	char *mapTbl_sec;
} MutualMpTestResult_t;

extern MutualMpTest_t *ptMutualMpTest;
extern MutualMpTestResult_t *ptMutualMpTestResult;

/*declaired at my_parser.c*/
extern void my_parser_exit(void);
extern int my_parser(char *path);
extern int ms_getInidata(char *section, char *ItemName, char *returnValue);
extern int ms_atoi(char *nptr);
extern int ms_ini_split_u8_array(char *key, u8 *pBuf);
extern int ms_ini_split_u16_array(char *key, u16 *pBuf);
extern int ms_ini_split_int_array(char *key, int *pBuf);
extern int ms_ini_2d_array(const char *pFile, char *pSection, u16 pArray[][2]);
extern int ms_ini_split_golden(int *pBuf);

/*declaired at IniLookup.c*/
extern int IniSecKeysTo2DArray2(const char *pFile, char *pSection, u16 pArray[][2]);
extern int IniGetU16Array(const char *section, const char *key, u16 *pBuf);
#ifndef DISABLE_DOUBLE
extern int IniGetDoubleArray(char *section, char *key, double *pBuf);
#else
extern int IniGetDoubleArray(char *section, char *key, int *pBuf);
#endif
extern int IniGetIntArray(char *section, char *key, int *pBuf);
extern int IniSplitU8Array(char *section, char *key, u8 *pBuf);
extern char *IniGetString(char *section, char *key);
extern int IniGetInt(char *section, char *key);
extern int IniLoad(char *pFile);
extern void IniFree(void);

/*declaired at MpTest.c*/
#ifndef DISABLE_DOUBLE
extern void calGoldenRange(double *goldenTbl, u16 weight, u16 weight_up, double *maxTbl, double *minTbl,
			   int length);
#else
extern void calGoldenRange(int *goldenTbl, u16 weight, u16 weight_up, int *maxTbl, int *minTbl, int length);
#endif
extern int Msg28xxLoadIni(char *pFilePath);
extern int Msg28xxMPTestResult(int nResultType, int *pResult);
extern int Msg28xxMPTestResultD(int nResultType, double *pResult);
extern void Msg28xxEndMPTest(void);
extern u16 getFirmwareOfficialVersionOnFlash(void);
extern int Msg28xxMPTestResult(int nResultType, int *pResult);
extern int Msg28xxStartMPTest(void);
extern void Msg28xxEndMPTest(void);

/*declaired at MsgControl.c*/
extern void EnterDBBus(void);
extern void ExitDBBus(void);
extern void StartMCU(void);
extern void StopMCU(void);
extern s32 EnterMpMode(void);
extern s32 Msg28xxCheckSwitchStatus(void);
extern s32 Msg28xxSwitchFwMode(u16 *nFMode, u16 *deep_standby);
extern int ReadFlash(u8 nChipType, u32 nAddr, int nBlockType, int nLength, u8 *pFlashData);

/*declaired at MsgLib.c*/
extern int getFirmwareVersionOnFlash(void);
extern int startMPTest(int nChipType, char *FilePath);

/*declaired at MsgUtility.c*/
extern void RegGet16BitByteValueBuf(u16 nAddr, u8 *pBuf, u16 nLen);
extern void DebugShowArray2(void *pBuf, u16 nLen, int nDataType, int nCarry, int nChangeLine);

/*declaired at OpenTest.c*/
extern int Msg28xxOpenTest(u16 fw_ver);
extern s32 Msg28xxopen_latter_FW_v1007(u16 nFMode);

/*declaired at ShortTest.c*/
extern s32 Msg28xxGetValueR(s32 *pTarget);
extern int Msg28xxShortTest(void);

/*declaired at WaterProofTest.c*/
extern int Msg28xxWaterProofTest(void);

/*declaired at TestCommon.c*/
extern s32 Msg28xxtGetWaterProofOneShotRawIIR(s16 *pRawDataWP, int *nSize);
extern s32 Msg28xxGetMutualOneShotRawIIR(s16 *nResultData, u16 *pSenNum, u16 *pDrvNum);
extern void Msg28xxEnableAdcOneShot(void);
extern void Msg28xxDBBusReEnter(void);
extern void Msg28xxDBBusReadDQMemEnd(void);
extern void Msg28xxDBBusReadDQMemStart(void);
extern void Msg28xxAnaChangeCDtime(u16 T1, u16 T2);
extern u8 CheckValueInRange(s32 nValue, s32 nMax, s32 nMin);
#ifndef DISABLE_DOUBLE
extern u8 checkDoubleValueInRange(double nValue, double nMax, double nMin);
#else
extern u8 checkDoubleValueInRange(s32 nValue, s32 nMax, s32 nMin);
#endif
extern void Msg28xxAnaSwReset(void);
extern s32 Msg28xxGetMutualOneShotRawIIR(s16 *nResultData, u16 *pSenNum, u16 *pDrvNum);

/*declaired at CreateCSV.c*/
extern void save_test_data(void);

#endif
