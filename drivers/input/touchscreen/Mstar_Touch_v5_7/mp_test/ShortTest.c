#include "global.h"

extern int _gSenseLineNum;
extern int _gDriveLineNum;
extern int _gDeltaC[MAX_MUTUAL_NUM];
int _gSenseNum = 0;
int _gDriveNum = 0;
int _gGRNum = 0;
int _gGRSize = 0;
int _gGRtestPins[13] = { 0 };

u16 GRPins[13];
/*6:max subframe    13:max afe */
u16 testPin_data[TEST_ITEM_NUM][MAX_MUTUAL_NUM];

u16 _gMuxMem_20_3E_0_Settings[16] = { 0 };
u16 _gMuxMem_20_3E_1_Settings[16] = { 0 };
u16 _gMuxMem_20_3E_2_Settings[16] = { 0 };
u16 _gMuxMem_20_3E_3_Settings[16] = { 0 };
u16 _gMuxMem_20_3E_4_Settings[16] = { 0 };
u16 _gMuxMem_20_3E_5_Settings[16] = { 0 };
u16 _gMuxMem_20_3E_6_Settings[16] = { 0 };

u16 MUX_MEM_2833_1_Settings[16] = {
	0x0001, 0x0000, 0x0000, 0x2000, 0x0000, 0x0003, 0x0000, 0x0400,
	0x0000, 0x0500, 0x0000, 0x0006, 0x0000, 0x0000, 0x7000, 0x0000
};
u16 MUX_MEM_2833_2_Settings[16] = {
	0x0000, 0x0000, 0x1000, 0x0000, 0x0002, 0x0000, 0x0300, 0x0000,
	0x0004, 0x0000, 0x0005, 0x0060, 0x0000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2833_3_Settings[16] = {
	0x0000, 0x0000, 0x0000, 0x0001, 0x0020, 0x0000, 0x3000, 0x0000,
	0x0040, 0x0000, 0x0050, 0x0600, 0x0000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2833_4_Settings[16] = {
	0x0000, 0x0000, 0x0000, 0x0010, 0x0200, 0x0000, 0x0000, 0x0003,
	0x0000, 0x0004, 0x0500, 0x6000, 0x0000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2833_5_Settings[16] = {
	0x0000, 0x0000, 0x0000, 0x0100, 0x2000, 0x0000, 0x0000, 0x0030,
	0x0000, 0x0040, 0x5000, 0x0000, 0x0000, 0x0000, 0x0600, 0x0000
};

u16 MUX_MEM_2835_1_Settings[16] = {
	0x0001, 0x0000, 0x0000, 0x0020, 0x0300, 0x0000, 0x0000, 0x0004,
	0x0000, 0x0005, 0x0600, 0x7000, 0x0000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2835_2_Settings[16] = {
	0x0010, 0x0000, 0x0000, 0x0200, 0x3000, 0x0000, 0x0000, 0x0040,
	0x0000, 0x0050, 0x6000, 0x0000, 0x0000, 0x0000, 0x0700, 0x0000
};
u16 MUX_MEM_2835_3_Settings[16] = {
	0x0100, 0x0000, 0x0000, 0x2000, 0x0000, 0x0003, 0x0000, 0x0400,
	0x0000, 0x0500, 0x0000, 0x0006, 0x0000, 0x0000, 0x7000, 0x0000
};
u16 MUX_MEM_2835_4_Settings[16] = {
	0x0000, 0x0000, 0x1000, 0x0000, 0x0002, 0x0000, 0x0300, 0x0000,
	0x0004, 0x0000, 0x0005, 0x0060, 0x0000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2835_5_Settings[16] = {
	0x0000, 0x0000, 0x0000, 0x0001, 0x0020, 0x0000, 0x3000, 0x0000,
	0x0040, 0x0000, 0x0050, 0x0600, 0x0000, 0x0000, 0x0000, 0x0000
};

u16 MUX_MEM_2836_1_Settings[16] = {
	0x0001, 0x0000, 0x0200, 0x3000, 0x0000, 0x0004, 0x0000, 0x0005,
	0x0600, 0x0000, 0x0070, 0x0800, 0x0000, 0x0000, 0x9000, 0x0000
};
u16 MUX_MEM_2836_2_Settings[16] = {
	0x0010, 0x0000, 0x2000, 0x0000, 0x0003, 0x0040, 0x0000, 0x0050,
	0x0000, 0x0006, 0x0700, 0x8000, 0x0000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2836_3_Settings[16] = {
	0x0100, 0x0000, 0x0000, 0x0002, 0x0030, 0x0000, 0x0040, 0x0500,
	0x0000, 0x0060, 0x7000, 0x0000, 0x0008, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2836_4_Settings[16] = {
	0x1000, 0x0000, 0x0000, 0x0020, 0x0300, 0x0000, 0x0400, 0x0000,
	0x0005, 0x0600, 0x0000, 0x0007, 0x0080, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2836_5_Settings[16] = {
	0x0000, 0x0001, 0x0000, 0x0200, 0x3000, 0x0000, 0x4000, 0x0000,
	0x0050, 0x0000, 0x0006, 0x0070, 0x0000, 0x0000, 0x0800, 0x0000
};

u16 MUX_MEM_5846_1_Settings[16] = {
	0x0001, 0x0000, 0x0200, 0x3000, 0x0000, 0x0004, 0x0000, 0x0005,
	0x0600, 0x0000, 0x0070, 0x0800, 0x0000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_5846_2_Settings[16] = {
	0x0010, 0x0000, 0x2000, 0x0000, 0x0003, 0x0040, 0x0000, 0x0050,
	0x0000, 0x0006, 0x0700, 0x8000, 0x0000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_5846_3_Settings[16] = {
	0x0100, 0x0000, 0x0000, 0x0002, 0x0030, 0x0000, 0x0040, 0x0500,
	0x0000, 0x0060, 0x7000, 0x0000, 0x0008, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_5846_4_Settings[16] = {
	0x1000, 0x0000, 0x0000, 0x0020, 0x0300, 0x0000, 0x0400, 0x0000,
	0x0005, 0x0600, 0x0000, 0x0007, 0x0080, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_5846_5_Settings[16] = {
	0x0000, 0x0001, 0x0000, 0x0200, 0x3000, 0x0000, 0x4000, 0x0000,
	0x0050, 0x0000, 0x0006, 0x0070, 0x0000, 0x0000, 0x0800, 0x0000
};

u16 MUX_MEM_2838_1_Settings[16] = {
	0x0001, 0x0000, 0x0000, 0x0002, 0x0030, 0x0000, 0x0040, 0x0500,
	0x0000, 0x0060, 0x7000, 0x0000, 0x0008, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2838_2_Settings[16] = {
	0x0100, 0x0000, 0x0000, 0x0020, 0x0300, 0x0000, 0x0400, 0x0000,
	0x0005, 0x0600, 0x0000, 0x0007, 0x0080, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2838_3_Settings[16] = {
	0x1000, 0x0000, 0x0000, 0x0200, 0x3000, 0x0000, 0x4000, 0x0000,
	0x0050, 0x0000, 0x0006, 0x0070, 0x0000, 0x0000, 0x0800, 0x0000
};
u16 MUX_MEM_2838_4_Settings[16] = {
	0x0000, 0x0000, 0x0100, 0x2000, 0x0000, 0x0003, 0x0000, 0x0004,
	0x0500, 0x0000, 0x0060, 0x0700, 0x0000, 0x0000, 0x8000, 0x0000
};
u16 MUX_MEM_2838_5_Settings[16] = {
	0x0000, 0x0000, 0x1000, 0x0000, 0x0002, 0x0030, 0x0000, 0x0040,
	0x0000, 0x0005, 0x0600, 0x7000, 0x0000, 0x0000, 0x0000, 0x0000
};

u16 MUX_MEM_2856_1_Settings[16] = {
	0x0001, 0x0000, 0x0020, 0x0030, 0x0400, 0x0000, 0x0500, 0x0000,
	0x0006, 0x0070, 0x8000, 0x0000, 0x0009, 0x0000, 0xA000, 0x0000
};
u16 MUX_MEM_2856_2_Settings[16] = {
	0x0010, 0x0000, 0x0200, 0x0300, 0x4000, 0x0000, 0x5000, 0x0000,
	0x0060, 0x0700, 0x0000, 0x0008, 0x0090, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2856_3_Settings[16] = {
	0x0100, 0x0000, 0x2000, 0x3000, 0x0000, 0x0040, 0x0000, 0x0005,
	0x0600, 0x0000, 0x0007, 0x0080, 0x0900, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2856_4_Settings[16] = {
	0x1000, 0x0000, 0x0000, 0x0002, 0x0030, 0x0000, 0x0004, 0x0050,
	0x6000, 0x0000, 0x0070, 0x0800, 0x9000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2856_5_Settings[16] = {
	0x0000, 0x0000, 0x0001, 0x0000, 0x0002, 0x0003, 0x0040, 0x0500,
	0x0000, 0x0006, 0x0700, 0x8000, 0x0000, 0x0000, 0x0900, 0x0000
};

u16 MUX_MEM_2856s_1_Settings[16] = {
	0x0001, 0x0000, 0x0000, 0x0002, 0x0030, 0x0000, 0x0004, 0x0050,
	0x6000, 0x0000, 0x0070, 0x0800, 0x9000, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2856s_2_Settings[16] = {
	0x0000, 0x0000, 0x0001, 0x0020, 0x0300, 0x0000, 0x0040, 0x0500,
	0x0000, 0x0060, 0x0700, 0x8000, 0x0000, 0x0000, 0x0900, 0x0000
};
u16 MUX_MEM_2856s_3_Settings[16] = {
	0x0000, 0x0000, 0x0010, 0x0200, 0x3000, 0x0000, 0x0400, 0x0000,
	0x0005, 0x0600, 0x7000, 0x0000, 0x0008, 0x0000, 0x9000, 0x0000
};
u16 MUX_MEM_2856s_4_Settings[16] = {
	0x0000, 0x0000, 0x0100, 0x2000, 0x0000, 0x0003, 0x4000, 0x0000,
	0x0050, 0x0006, 0x0000, 0x0007, 0x0080, 0x0000, 0x0000, 0x0000
};
u16 MUX_MEM_2856s_5_Settings[16] = {
	0x0000, 0x0000, 0x1000, 0x0000, 0x0002, 0x0030, 0x0000, 0x0004,
	0x0500, 0x0000, 0x0006, 0x0070, 0x0800, 0x0000, 0x0000, 0x0000
};

u16 sensepad_pin_mapping_2833[60] = {
	10, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 0,  0,  0,  0,  0, 21, 22, 23, 24,  25,  0, 26, 27,  0,  0, 28, 29, 30, 0,
	31, 32, 33, 34, 35, 36, 37, 38,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 39, 40
};
u16 sensepad_pin_mapping_2835[60] = {
	8,  9, 10,  0,  0,  0,  0,  0,  0,  0,  0, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 0,  0,  0,  0,  0, 21, 22, 23, 24,	25,  0, 26, 27,  0,  0, 28, 29, 30, 0,
	31, 32, 33, 34, 35, 36, 37, 38,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 39, 40
};
u16 sensepad_pin_mapping_2836[60] = {
	8,  9, 10, 11, 12,  0,  0,  0,  0,  0, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
	23, 24,  0,  0,  0, 25, 26, 27, 28, 29, 30,  0, 31, 32, 33,  0, 34, 35, 36, 0,
	37, 38, 39, 40, 41, 42, 43, 44, 45, 46,  0,  0,  0,  0,  0,  0,  0,  0, 47, 48
};
u16 sensepad_pin_mapping_2838[60] = {
	10,  0, 11, 12,  0,  0,  0,  0,  0,  0, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
	23, 24,  0,  0,  0, 25, 26, 27, 28, 29, 30,  0, 31, 32, 33,  0, 34, 35, 36,  0,
	37, 38, 39, 40, 41, 42, 43, 44, 45, 46,  0,  0,  0,  0,  0,  0,  0,  0, 47, 48
};
u16 sensepad_pin_mapping_2856[60] = {
	11, 12, 13, 14,  0,  0,  0,  0, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
	27, 28,  0,  0, 29, 30, 31, 32, 33, 34, 35,  0, 36, 37, 38, 39, 40, 41, 42,  0,
	43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54,  0,  0,  0,  0,  0,  0, 55, 56
};

u16 sensepad_pin_mapping_2856s[60] = {
	14,  0,  0,  0,  0,  0,  0,  0, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
	27, 28,  0,  0, 29, 30, 31, 32, 33, 34, 35,  0, 36, 37, 38, 39, 40, 41, 42, 0,
	43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54,  0,  0,  0,  0,  0,  0, 55, 56
};
u16 sensepad_pin_mapping_5846[60] = {
	8,  9, 10, 11, 12,  0,  0,  0,  0,  0, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
	23, 24,  0,  0,  0, 25, 26, 27, 28, 29, 30,  0, 31, 32, 33,  0, 34, 35, 36,  0,
	37, 38, 39, 40, 41, 42, 43, 44, 45, 46,  0,  0,  0,  0,  0,  0,  0,  0, 47, 0
};

s32 Msg28xxGetValueR(s32 *pTarget)
{
	s16 *pRawData = NULL;
	u16 nSenNumBak = 0;
	u16 nDrvNumBak = 0;
	u16 nShift = 0;
	s16 i, j;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	pRawData = kcalloc(MAX_MUTUAL_NUM, sizeof(s16), GFP_KERNEL);

	if (Msg28xxGetMutualOneShotRawIIR(pRawData, &nSenNumBak, &nDrvNumBak) < 0) {
		pr_err("*** Msg28xx Short Test# GetMutualOneShotRawIIR failed! ***\n");
		kfree(pRawData);
		return -EPERM;
	}

	for (i = 0; i < 5; i++) {
		for (j = 0; j < 13; j++) {
			nShift = (u16) (j + 13 * i);
			pTarget[nShift] = pRawData[j + 13 * i];
		}
	}

	kfree(pRawData);

	return 0;
}

void readSettings(void)
{
	int i, nSize;
	char str[512] = { 0};

	TEST_DBG(0, "*** %s() ***\n", __func__);

	for (i = 1; i < 6; i++) {
		nSize = 0;
		if (i == 1) {
			/*nSize = IniGetU16Array("[SHORT_TEST_N1]","MUX_MEM_20_3E", _gMuxMem_20_3E_1_Settings); */
			ms_getInidata("SHORT_TEST_N1", "MUX_MEM_20_3E", str);
			nSize = ms_ini_split_u16_array(str, _gMuxMem_20_3E_1_Settings);
			TEST_DBG(0, "SHORT_TEST_N1\n");
			DebugShowArray2(_gMuxMem_20_3E_1_Settings, 16, 16, 16, 16);
		} else if (i == 2) {
			/*nSize = IniGetU16Array("[SHORT_TEST_N2]","MUX_MEM_20_3E", _gMuxMem_20_3E_2_Settings); */
			ms_getInidata("SHORT_TEST_N2", "MUX_MEM_20_3E", str);
			nSize = ms_ini_split_u16_array(str, _gMuxMem_20_3E_2_Settings);
			TEST_DBG(0, "SHORT_TEST_N2\n");
			DebugShowArray2(_gMuxMem_20_3E_2_Settings, 16, 16, 16, 16);
		} else if (i == 3) {
			/*nSize = IniGetU16Array("[SHORT_TEST_S1]","MUX_MEM_20_3E", _gMuxMem_20_3E_3_Settings); */
			ms_getInidata("SHORT_TEST_S1", "MUX_MEM_20_3E", str);
			nSize = ms_ini_split_u16_array(str, _gMuxMem_20_3E_3_Settings);
			TEST_DBG(0, "SHORT_TEST_S1\n");
			DebugShowArray2(_gMuxMem_20_3E_3_Settings, 16, 16, 16, 16);
		} else if (i == 4) {
			/*nSize = IniGetU16Array("[SHORT_TEST_S2]","MUX_MEM_20_3E", _gMuxMem_20_3E_4_Settings); */
			ms_getInidata("SHORT_TEST_S2", "MUX_MEM_20_3E", str);
			nSize = ms_ini_split_u16_array(str, _gMuxMem_20_3E_4_Settings);
			TEST_DBG(0, "SHORT_TEST_S2\n");
			DebugShowArray2(_gMuxMem_20_3E_4_Settings, 16, 16, 16, 16);
		} else if (i == 5) {
			/*if (IniGetU16Array("[SHORT_TEST_N3]","MUX_MEM_20_3E", _gMuxMem_20_3E_5_Settings)) {
			pr_notice("SHORT_TEST_N3\n");
			DebugShowArray2(_gMuxMem_20_3E_5_Settings, 16, 16, 16, 16);
			return;
			}
			if (IniGetU16Array("[SHORT_TEST_S3]","MUX_MEM_20_3E", _gMuxMem_20_3E_5_Settings)) {
			pr_notice("SHORT_TEST_S3\n");
			DebugShowArray2(_gMuxMem_20_3E_5_Settings, 16, 16, 16, 16);
			return;
			}
			if (IniGetU16Array("[SHORT_TEST_GR]","MUX_MEM_20_3E", _gMuxMem_20_3E_5_Settings)) {
			pr_notice("SHORT_TEST_GR\n");
			DebugShowArray2(_gMuxMem_20_3E_5_Settings, 16, 16, 16, 16);
			return;
			} */
			if (ms_getInidata("SHORT_TEST_N3", "MUX_MEM_20_3E", str) != 0) {
				if (ms_ini_split_u16_array(str, _gMuxMem_20_3E_5_Settings)) {
					TEST_DBG(0, "SHORT_TEST_N3\n");
					DebugShowArray2(_gMuxMem_20_3E_5_Settings, 16, 16, 16, 16);
					return;
				}
			}

			if (ms_getInidata("SHORT_TEST_S3", "MUX_MEM_20_3E", str) != 0) {
				if (ms_ini_split_u16_array(str, _gMuxMem_20_3E_5_Settings)) {
					TEST_DBG(0, "SHORT_TEST_S3\n");
					DebugShowArray2(_gMuxMem_20_3E_5_Settings, 16, 16, 16, 16);
					return;
				}
			}

			if (ms_getInidata("SHORT_TEST_GR", "MUX_MEM_20_3E", str) != 0) {
				if (ms_ini_split_u16_array(str, _gMuxMem_20_3E_5_Settings)) {
					TEST_DBG(0, "SHORT_TEST_GR\n");
					DebugShowArray2(_gMuxMem_20_3E_5_Settings, 16, 16, 16, 16);
					TEST_DBG(0, "SHORT_TEST_GR end\n");
					return;
				}
			}
		}
	}
}

#ifndef DISABLE_DOUBLE
double Msg28xxCovertRValue(s32 deltaR)
#else
int Msg28xxCovertRValue(s32 deltaR)
#endif
{
#ifndef DISABLE_DOUBLE
	if (deltaR >= IIR_MAX)
		return 0;
	if (deltaR == 0)
		deltaR = 1;
	return ((3.72 - 1.3) * 2.15 * 32768 * 1.1 / (50 * (((double)deltaR - 0))));
#else
	if (deltaR >= IIR_MAX)
		return 0;
	if (deltaR == 0)
		deltaR = 1;
	return (187541 / (50 * (((s32) deltaR - 0))));
#endif

}

void Msg28xxChangeANASetting_for_ICpin_short_test(char *sSupportIC)
{
	int i, nMappingItem;
	u8 nChipVer;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	/*Stop mcu */
	/*RegSet16BitValue(0x0FE6, 0x0001); //bank:mheg5, addr:h0073 */
	StopMCU();

	nChipVer = RegGetLByteValue(0x1ECE);

	if (nChipVer != 0)
		RegSetLByteValue(0x131E, 0x01);

	if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2833")) {
		memcpy(_gMuxMem_20_3E_1_Settings, MUX_MEM_2833_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(_gMuxMem_20_3E_2_Settings, MUX_MEM_2833_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(_gMuxMem_20_3E_3_Settings, MUX_MEM_2833_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(_gMuxMem_20_3E_4_Settings, MUX_MEM_2833_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(_gMuxMem_20_3E_5_Settings, MUX_MEM_2833_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2835")) {
		memcpy(_gMuxMem_20_3E_1_Settings, MUX_MEM_2835_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(_gMuxMem_20_3E_2_Settings, MUX_MEM_2835_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(_gMuxMem_20_3E_3_Settings, MUX_MEM_2835_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(_gMuxMem_20_3E_4_Settings, MUX_MEM_2835_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(_gMuxMem_20_3E_5_Settings, MUX_MEM_2835_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else
	    if ((!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2836"))
		|| (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2840"))) {
		memcpy(_gMuxMem_20_3E_1_Settings, MUX_MEM_2836_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(_gMuxMem_20_3E_2_Settings, MUX_MEM_2836_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(_gMuxMem_20_3E_3_Settings, MUX_MEM_2836_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(_gMuxMem_20_3E_4_Settings, MUX_MEM_2836_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(_gMuxMem_20_3E_5_Settings, MUX_MEM_2836_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2838")) {
		memcpy(_gMuxMem_20_3E_1_Settings, MUX_MEM_2838_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(_gMuxMem_20_3E_2_Settings, MUX_MEM_2838_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(_gMuxMem_20_3E_3_Settings, MUX_MEM_2838_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(_gMuxMem_20_3E_4_Settings, MUX_MEM_2838_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(_gMuxMem_20_3E_5_Settings, MUX_MEM_2838_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2856")) {
		memcpy(_gMuxMem_20_3E_1_Settings, MUX_MEM_2856_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(_gMuxMem_20_3E_2_Settings, MUX_MEM_2856_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(_gMuxMem_20_3E_3_Settings, MUX_MEM_2856_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(_gMuxMem_20_3E_4_Settings, MUX_MEM_2856_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(_gMuxMem_20_3E_5_Settings, MUX_MEM_2856_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else
	    if ((!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2856s"))
		|| (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2856S"))) {
		memcpy(_gMuxMem_20_3E_1_Settings, MUX_MEM_2856s_1_Settings,
		       sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(_gMuxMem_20_3E_2_Settings, MUX_MEM_2856s_2_Settings,
		       sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(_gMuxMem_20_3E_3_Settings, MUX_MEM_2856s_3_Settings,
		       sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(_gMuxMem_20_3E_4_Settings, MUX_MEM_2856s_4_Settings,
		       sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(_gMuxMem_20_3E_5_Settings, MUX_MEM_2856s_5_Settings,
		       sizeof(_gMuxMem_20_3E_5_Settings));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "5846")) {
		memcpy(_gMuxMem_20_3E_1_Settings, MUX_MEM_5846_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(_gMuxMem_20_3E_2_Settings, MUX_MEM_5846_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(_gMuxMem_20_3E_3_Settings, MUX_MEM_5846_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(_gMuxMem_20_3E_4_Settings, MUX_MEM_5846_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(_gMuxMem_20_3E_5_Settings, MUX_MEM_5846_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	}

	for (nMappingItem = 0; nMappingItem < 6; nMappingItem++) {
		/*sensor mux sram read/write base address / write length */
		RegSetLByteValue(0x2192, 0x00);
		RegSetLByteValue(0x2102, 0x01);
		RegSetLByteValue(0x2102, 0x00);
		RegSetLByteValue(0x2182, 0x08);
		RegSetLByteValue(0x2180, 0x08 * nMappingItem);
		RegSetLByteValue(0x2188, 0x01);

		for (i = 0; i < 8; i++) {
			if (nMappingItem == 0 && nChipVer == 0x0) {
				memset(_gMuxMem_20_3E_0_Settings, 0, sizeof(_gMuxMem_20_3E_0_Settings));
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_0_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_0_Settings[2 * i + 1]);
			}
			if ((nMappingItem == 1 && nChipVer == 0x0) || (nMappingItem == 0 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_1_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_1_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_1_Settings[%d] = %x, _gMuxMem_20_3E_1_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_1_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_1_Settings[2 * i + 1]);
			}
			if ((nMappingItem == 2 && nChipVer == 0x0) || (nMappingItem == 1 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_2_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_2_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_2_Settings[%d] = %x, _gMuxMem_20_3E_2_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_2_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_2_Settings[2 * i + 1]);
			}
			if ((nMappingItem == 3 && nChipVer == 0x0) || (nMappingItem == 2 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_3_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_3_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_3_Settings[%d] = %x, _gMuxMem_20_3E_3_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_3_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_3_Settings[2 * i + 1]);
			}
			if ((nMappingItem == 4 && nChipVer == 0x0) || (nMappingItem == 3 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_4_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_4_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_4_Settings[%d] = %x, _gMuxMem_20_3E_4_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_4_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_4_Settings[2 * i + 1]);
			}
			if ((nMappingItem == 5 && nChipVer == 0x0) || (nMappingItem == 4 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_5_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_5_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_5_Settings[%d] = %x, _gMuxMem_20_3E_5_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_5_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_5_Settings[2 * i + 1]);
			}
			if (nMappingItem == 5 && nChipVer != 0x0) {
				memset(_gMuxMem_20_3E_6_Settings, 0, sizeof(_gMuxMem_20_3E_6_Settings));
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_6_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_6_Settings[2 * i + 1]);
			}
		}
	}
}

void Msg28xxSetNoiseSensorMode(u8 nEnable)
{
	s16 j;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	if (nEnable) {
		RegSet16BitValueOn(0x1546, BIT4);
		for (j = 0; j < 10; j++) {
			RegSet16BitValue(0x2148 + 2 * j, 0x0000);
		}
		RegSet16BitValue(0x215C, 0x1FFF);
	}
}

void Msg28xxAndChangeCDtime(u16 nTime1, u16 nTime2)
{
	TEST_DBG(0, "*** %s() ***\n", __func__);

	RegSet16BitValue(0x1026, nTime1);
	RegSet16BitValue(0x1030, nTime2);
}

void Msg28xxAnaFixPrs(u16 option)
{
	u16 regData = 0;

	TEST_DBG(0, "*** %s() ***\n", __func__);
	/*RegSet16BitValue(0x0FE6, BIT0);   // Stop mcu */
	StopMCU();

	regData = RegGet16BitValue(0x1008) & (BIT0 | BIT4 | BIT5 | BIT6 | BIT7);
	regData |= (u16) ((option << 1) & (BIT1 | BIT2 | BIT3));
	RegSet16BitValue(0x1008, regData);
}

#ifndef DISABLE_DOUBLE
s32 Msg28xxICpin_short_test_result_prepare(double thrs, double *senseR)
#else
s32 Msg28xxICpin_short_test_result_prepare(int thrs, int *senseR)
#endif
{
	int count = 0, i, nRet = 0;
/*int test = 0;*/

	u16 pinTBL[MAX_CHANNEL_NUM_28XX];

	TEST_DBG(0, "*** %s() ***\n", __func__);
#ifndef DISABLE_DOUBLE
	if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2833")) {
		memcpy(pinTBL, sensepad_pin_mapping_2833, sizeof(pinTBL));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2835")) {
		memcpy(pinTBL, sensepad_pin_mapping_2835, sizeof(pinTBL));
	} else
	    if ((!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2836"))
		|| (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2840"))) {
		memcpy(pinTBL, sensepad_pin_mapping_2836, sizeof(pinTBL));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2838")) {
		memcpy(pinTBL, sensepad_pin_mapping_2838, sizeof(pinTBL));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2856")) {
		memcpy(pinTBL, sensepad_pin_mapping_2856, sizeof(pinTBL));
	} else
	    if ((!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2856s"))
		|| (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2856S"))) {
		memcpy(pinTBL, sensepad_pin_mapping_2856s, sizeof(pinTBL));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "5846")) {
		memcpy(pinTBL, sensepad_pin_mapping_5846, sizeof(pinTBL));
	}

	for (i = 0; i < MAX_CHANNEL_NUM_28XX; i++) {
		ptMutualMpTestResult->pICPinChannel[i] = 0;
		ptMutualMpTestResult->pICPinShortFailChannel[i] = 0;
		if (pinTBL[i] != 0) {
			ptMutualMpTestResult->pICPinChannel[i] = (int)pinTBL[i];
			ptMutualMpTestResult->pICPinShortRData[i] = senseR[i];
			TEST_DBG(0, "pICPinChannel[%d] = %d, IC Pin P[%d] = %.2fM, deltaC = %d\n", i,
				 ptMutualMpTestResult->pICPinChannel[i], pinTBL[i], senseR[i],
				 ptMutualMpTestResult->pICPinShortResultData[i]);
			if (senseR[i] < thrs) {
				ptMutualMpTestResult->pICPinShortFailChannel[i] = (int)pinTBL[i];
				TEST_DBG(0, "IC Pin Fail P%d = %.2fM\n", pinTBL[i], senseR[i]);
				count++;
				nRet = -1;
			}
		}
	}			/*for (int i = 0; i < senseR.Length; i++) */
#else
	if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2833")) {
		memcpy(pinTBL, sensepad_pin_mapping_2833, sizeof(pinTBL));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2835")) {
		memcpy(pinTBL, sensepad_pin_mapping_2835, sizeof(pinTBL));
	} else
	    if ((!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2836"))
		|| (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2840"))) {
		memcpy(pinTBL, sensepad_pin_mapping_2836, sizeof(pinTBL));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2838")) {
		memcpy(pinTBL, sensepad_pin_mapping_2838, sizeof(pinTBL));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2856")) {
		memcpy(pinTBL, sensepad_pin_mapping_2856, sizeof(pinTBL));
	} else
	    if ((!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2856s"))
		|| (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "2856S"))) {
		memcpy(pinTBL, sensepad_pin_mapping_2856s, sizeof(pinTBL));
	} else if (!strcmp(ptMutualMpTest->UIConfig.sSupportIC, "5846")) {
		memcpy(pinTBL, sensepad_pin_mapping_5846, sizeof(pinTBL));
	}

	for (i = 0; i < MAX_CHANNEL_NUM_28XX; i++) {
		ptMutualMpTestResult->pICPinChannel[i] = 0;
		ptMutualMpTestResult->pICPinShortFailChannel[i] = 0;
		if (pinTBL[i] != 0) {
			ptMutualMpTestResult->pICPinChannel[i] = (int)pinTBL[i];
			ptMutualMpTestResult->pICPinShortRData[i] = senseR[i];
			TEST_DBG(0, "pICPinChannel[%d] = %d, IC Pin P[%d] = %.2dM, deltaC = %d\n", i,
				 ptMutualMpTestResult->pICPinChannel[i], pinTBL[i], senseR[i],
				 ptMutualMpTestResult->pICPinShortResultData[i]);
			if (senseR[i] < thrs) {
				ptMutualMpTestResult->pICPinShortFailChannel[i] = (int)pinTBL[i];
				TEST_DBG(0, "IC Pin Fail P%d = %.2dM\n", pinTBL[i], senseR[i]);
				count++;
				nRet = -1;
			}
		}
	}			/*for (int i = 0; i < senseR.Length; i++) */
#endif
	return nRet;
}

void Msg28xxPrepareAna_for_short_test(void)
{
	TEST_DBG(0, "*** %s() ***\n", __func__);

	/*Stop mcu */
	/*RegSet16BitValue(0x0FE6, 0x0001); //bank:mheg5, addr:h0073 */
	StopMCU();

	/*set Subframe = 6 ; Sensor = 13 */
	RegSetLByteValue(0x130A, 0x6D);
	RegSetLByteValue(0x1103, 0x06);
	RegSetLByteValue(0x1016, 0x0C);

	RegSetLByteValue(0x1104, 0x0C);
	RegSetLByteValue(0x100C, 0x0C);
	RegSetLByteValue(0x1B10, 0x0C);

	/*adc analog+digital pipe delay, 60= 13 AFE. */
	RegSetLByteValue(0x102F, 0x60);

	/*trim: Fout 52M &  1.2V */
	RegSet16BitValue(0x1420, 0xA55A);	/*password */
	RegSet16BitValue(0x1428, 0xA55A);	/*password */
	RegSet16BitValue(0x1422, 0xFC4C);	/*go */

	Msg28xxSetNoiseSensorMode(1);
	Msg28xxAnaFixPrs(3);
	Msg28xxAndChangeCDtime(0x007E, 0x0006);
	/*Msg28xxAndChangeCDtime(0x007E, 0x001F); */

	/*all AFE Cfb use defalt (50p) */
	RegSet16BitValue(0x1508, 0x1FFF);	/*all AFE Cfb: SW control */
	RegSet16BitValue(0x1550, 0x0000);	/*all AFE Cfb use defalt (50p) */

	/*reg_afe_icmp disenable */
	RegSet16BitValue(0x1552, 0x0000);

	/*reg_hvbuf_sel_gain */
	RegSet16BitValue(0x1564, 0x0077);

	/*ADC: AFE Gain bypass */
	RegSet16BitValue(0x1260, 0x1FFF);
	/*reg_sel_ros disenable */
	RegSet16BitValue(0x156A, 0x0000);

	/*reg_adc_desp_invert disenable */
	RegSetLByteValue(0x1221, 0x00);

	/*AFE gain = 1X */
	RegSet16BitValue(0x1318, 0x4440);
	RegSet16BitValue(0x131A, 0x4444);

	RegSet16BitValueByAddressMode(0x1012, 0x0680, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1022, 0x0000, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x110A, 0x0104, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1310, 0x04F1, ADDRESS_MODE_16BIT);

	RegSet16BitValueByAddressMode(0x1317, 0x04F1, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1432, 0x0000, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1435, 0x0C00, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1538, 0x0400, ADDRESS_MODE_16BIT);

	RegSet16BitValueByAddressMode(0x1540, 0x0012, ADDRESS_MODE_16BIT);
	/**/ RegSet16BitValueByAddressMode(0x1530, 0x0133, ADDRESS_MODE_16BIT);	/*HI v buf enable */
	/*RegSet16BitValueByAddressMode(0x1533, 0x0522, ADDRESS_MODE_16BIT);//low v buf gain */
	RegSet16BitValueByAddressMode(0x1533, 0x0000, ADDRESS_MODE_16BIT);	/*low v buf gain */
	RegSet16BitValueByAddressMode(0x1E11, 0x8000, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x2003, 0x007E, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x2006, 0x137F, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x213E, 0x1FFF, ADDRESS_MODE_16BIT);

	/*re-set sample and coefficient */
	RegSet16BitValueByAddressMode(0x100D, 0x0020, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1103, 0x0020, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1104, 0x0020, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1302, 0x0020, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1B30, 0x0020, ADDRESS_MODE_16BIT);
	/*coefficient */
	RegSet16BitValueByAddressMode(0x136B, 0x10000 / 0x0020, ADDRESS_MODE_16BIT);	/*65536/ sample */
}

s32 Msg28xxRead_mapping_to_ICpin_for_short_test(void)
{
	int i, j, nMappingItem, nRet = 0;
	char *sSupportIC = ptMutualMpTest->UIConfig.sSupportIC;
	u16 MUX_MEM_1_Settings[16] = { 0 };
	u16 MUX_MEM_2_Settings[16] = { 0 };
	u16 MUX_MEM_3_Settings[16] = { 0 };
	u16 MUX_MEM_4_Settings[16] = { 0 };
	u16 MUX_MEM_5_Settings[16] = { 0 };

	TEST_DBG(0, "*** %s() ***\n", __func__);

	if (!strcmp(sSupportIC, "2833")) {
		memcpy(MUX_MEM_1_Settings, MUX_MEM_2833_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(MUX_MEM_2_Settings, MUX_MEM_2833_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(MUX_MEM_3_Settings, MUX_MEM_2833_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(MUX_MEM_4_Settings, MUX_MEM_2833_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(MUX_MEM_5_Settings, MUX_MEM_2833_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else if (!strcmp(sSupportIC, "2835")) {
		memcpy(MUX_MEM_1_Settings, MUX_MEM_2835_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(MUX_MEM_2_Settings, MUX_MEM_2835_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(MUX_MEM_3_Settings, MUX_MEM_2835_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(MUX_MEM_4_Settings, MUX_MEM_2835_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(MUX_MEM_5_Settings, MUX_MEM_2835_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else if ((!strcmp(sSupportIC, "2836")) || (!strcmp(sSupportIC, "2840"))) {
		memcpy(MUX_MEM_1_Settings, MUX_MEM_2836_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(MUX_MEM_2_Settings, MUX_MEM_2836_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(MUX_MEM_3_Settings, MUX_MEM_2836_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(MUX_MEM_4_Settings, MUX_MEM_2836_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(MUX_MEM_5_Settings, MUX_MEM_2836_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else if (!strcmp(sSupportIC, "2838")) {
		memcpy(MUX_MEM_1_Settings, MUX_MEM_2838_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(MUX_MEM_2_Settings, MUX_MEM_2838_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(MUX_MEM_3_Settings, MUX_MEM_2838_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(MUX_MEM_4_Settings, MUX_MEM_2838_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(MUX_MEM_5_Settings, MUX_MEM_2838_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else if (!strcmp(sSupportIC, "2856")) {
		memcpy(MUX_MEM_1_Settings, MUX_MEM_2856_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(MUX_MEM_2_Settings, MUX_MEM_2856_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(MUX_MEM_3_Settings, MUX_MEM_2856_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(MUX_MEM_4_Settings, MUX_MEM_2856_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(MUX_MEM_5_Settings, MUX_MEM_2856_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else if ((!strcmp(sSupportIC, "2856s")) || (!strcmp(sSupportIC, "2856S"))) {
		memcpy(MUX_MEM_1_Settings, MUX_MEM_2856s_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(MUX_MEM_2_Settings, MUX_MEM_2856s_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(MUX_MEM_3_Settings, MUX_MEM_2856s_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(MUX_MEM_4_Settings, MUX_MEM_2856s_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(MUX_MEM_5_Settings, MUX_MEM_2856s_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else if (!strcmp(sSupportIC, "5846")) {
		memcpy(MUX_MEM_1_Settings, MUX_MEM_5846_1_Settings, sizeof(_gMuxMem_20_3E_1_Settings));
		memcpy(MUX_MEM_2_Settings, MUX_MEM_5846_2_Settings, sizeof(_gMuxMem_20_3E_2_Settings));
		memcpy(MUX_MEM_3_Settings, MUX_MEM_5846_3_Settings, sizeof(_gMuxMem_20_3E_3_Settings));
		memcpy(MUX_MEM_4_Settings, MUX_MEM_5846_4_Settings, sizeof(_gMuxMem_20_3E_4_Settings));
		memcpy(MUX_MEM_5_Settings, MUX_MEM_5846_5_Settings, sizeof(_gMuxMem_20_3E_5_Settings));
	} else
		return nRet;

	for (nMappingItem = 1; nMappingItem < 6; nMappingItem++) {
		u16 testpin = 1;
		u16 index = 0;

		for (i = 0; i < sizeof(MUX_MEM_1_Settings) / sizeof(u16); i++) {
			for (j = 0; j < 4; j++) {
				if (nMappingItem == 1) {
					if (((MUX_MEM_1_Settings[i] >> (4 * j)) & 0x0F) != 0) {
						testPin_data[nMappingItem][index] = testpin;
						index++;
					}
				} else if (nMappingItem == 2) {
					if (((MUX_MEM_2_Settings[i] >> (4 * j)) & 0x0F) != 0) {
						testPin_data[nMappingItem][index] = testpin;
						index++;
					}
				} else if (nMappingItem == 3) {
					if (((MUX_MEM_3_Settings[i] >> (4 * j)) & 0x0F) != 0) {
						testPin_data[nMappingItem][index] = testpin;
						index++;
					}
				} else if (nMappingItem == 4) {
					if (((MUX_MEM_4_Settings[i] >> (4 * j)) & 0x0F) != 0) {
						testPin_data[nMappingItem][index] = testpin;
						index++;
					}
				} else if (nMappingItem == 5) {
					if (((MUX_MEM_5_Settings[i] >> (4 * j)) & 0x0F) != 0) {
						testPin_data[nMappingItem][index] = testpin;
						index++;
					}
				}
				testpin++;
			}	/*for (j = 0; j < 4; j++) */
		}		/*for (i = 0; i < MUX_MEM_1_Settings.Length; i++) */
	}			/*for (nMappingItem = 1; nMappingItem < 6; nMappingItem++) */
	return 1;
}

void Msg28xxAnaLoadSetting_for_ICpin_short_test(char *SupportIC)
{
	/*Stop mcu */
	/*RegSet16BitValue(0x0FE6, 0x0001); //bank:mheg5, addr:h0073 */
	StopMCU();
	Msg28xxChangeANASetting_for_ICpin_short_test(SupportIC);
}

void SetSensorPADState(u16 state)
{
	u16 value = 0, i;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	for (i = 0; i < 8; i++)
		value |= (u16) (state << (i * 2));
	for (i = 0; i < 8; i++)
		RegSet16BitValueByAddressMode(0x1514 + i, value, ADDRESS_MODE_16BIT);
	for (i = 0; i < 4; i++)
		RegSet16BitValueByAddressMode(0x1510 + i, 0xFFFF, ADDRESS_MODE_16BIT);
}

void patchFWAnaSetting_for_shorttest(void)
{
	int i = 0;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	/*overwrite sensor PAD , restore to default state */
	for (i = 0; i < 8; i++)
		RegSet16BitValueByAddressMode(0x1e33 + i, 0x0000, ADDRESS_MODE_16BIT);
	/*overwrite PAD gpio , restore to default state */
	RegSet16BitValueByAddressMode(0x1e30, 0x000f, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1e31, 0x0000, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1e32, 0xffff, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1e3b, 0xffff, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1e3c, 0xffff, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x1e3d, 0x003f, ADDRESS_MODE_16BIT);
	for (i = 0; i < 20; i++)
		RegSet16BitValueByAddressMode(0x2110 + i, 0x0000, ADDRESS_MODE_16BIT);
	for (i = 0; i < 16; i++)
		RegSet16BitValueByAddressMode(0x2160 + i, 0x0000, ADDRESS_MODE_16BIT);
	/*post idle for */
	RegSet16BitValueByAddressMode(0x101a, 0x0028, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x101b, 0x0028, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x101c, 0x0028, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x101d, 0x0028, ADDRESS_MODE_16BIT);
	RegSet16BitValueByAddressMode(0x101e, 0x0028, ADDRESS_MODE_16BIT);
}

/*int deltaC[MAX_MUTUAL_NUM] = {0};*/
s32 Msg28xxICPinShort(void)
{
	int *deltaC = NULL;
	s16 i;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	deltaC = kmalloc(sizeof(int) * 1904, GFP_KERNEL);

	Msg28xxPrepareAna_for_short_test();
	SetSensorPADState(POS_PULSE);
	/*Msg28xxUpdateSensorPADState(GRPins, GND, _gGRSize); */
	/*DAC overwrite */
	RegSet16BitValue(0x150C, 0x8040);	/*bit15 //AFE:1.3v for test */
	Msg28xxAnaLoadSetting_for_ICpin_short_test(ptMutualMpTest->UIConfig.sSupportIC);
	Msg28xxAnaSwReset();
	patchFWAnaSetting_for_shorttest();
	memset(_gDeltaC, 0, sizeof(_gDeltaC));
	if (Msg28xxGetValueR(_gDeltaC) < 0) {
		pr_err("*** Msg28xx IC Pin Short Test# GetValueR failed! ***\n");
		kfree(deltaC);
		return -EPERM;
	}

	TEST_DBG(0, "*** Msg28xx IC Pin Short Test# GetValueR 1.3v! ***\n");
	DebugShowArray2(_gDeltaC, 65, -32, 10, 13);

	/*memset(deltaC, 0, sizeof(deltaC)); */
	memset(deltaC, 0, sizeof(*deltaC));
	/*DAC overwrite */
	RegSet16BitValue(0x150C, 0x8083);	/*bit15 //AFE:3.72v for test */
	if (Msg28xxGetValueR(deltaC) < 0) {
		pr_err("*** Msg28xx Short Test# GetValueR failed! ***\n");
		kfree(deltaC);
		return -EPERM;
	}

	TEST_DBG(0, "*** Msg28xx IC Pin Short Test# GetValueR 3.72v! ***\n");
	DebugShowArray2(deltaC, 65, -32, 10, 13);
	for (i = 0; i < 65; i++) {	/*13 AFE * 5 subframe */
		if ((abs(deltaC[i]) < IIR_MAX) && (abs(_gDeltaC[i]) < IIR_MAX))
			_gDeltaC[i] = deltaC[i] - _gDeltaC[i];
		else {
			if (abs(deltaC[i]) > abs(_gDeltaC[i]))
				_gDeltaC[i] = deltaC[i];
		}

		if (abs(_gDeltaC[i]) >= (IIR_MAX))
			_gDeltaC[i] = 0x7FFF;
		else
			_gDeltaC[i] = abs(_gDeltaC[i]);
	}

	TEST_DBG(0, "*** Msg28xx IC Pin Short Test# GetValueR 3.72v - 1.3v ! ***\n");
	DebugShowArray2(_gDeltaC, 65, -32, 10, 13);
	kfree(deltaC);
	return 0;
}

s32 Msg28xxShortTestJudge_ICpin(u16 nItemID, u16 *deltaC_result, s8 *TestFail, int nGRSize)
{
	int nRet = 1, i, count_test_pin = 0, j;
	u16 GR_Id[13] = { 0 };

	int found_gr = 0, count = 0, BypassGR = 0;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	if (!Msg28xxRead_mapping_to_ICpin_for_short_test())
		return 0;

	if (ptMutualMpTest->sensorInfo.numGr == 0)
		BypassGR = 1;

	if (nGRSize) {
		for (j = 0; j < sizeof(GRPins) / sizeof(u16); j++) {
			if (GRPins[j] == 0xFFFF)
				continue;
			for (i = 0; i < TEST_ITEM_NUM; i++) {
				if ((testPin_data[nItemID][i] != PIN_UN_USE) && (testPin_data[nItemID][i] != 0)) {
					if (testPin_data[nItemID][i] == (GRPins[j] + 1)) {
						TEST_DBG(0, "testPin_data[%d][%d] = %d\n", nItemID, i,
							 testPin_data[nItemID][i]);
						GR_Id[count] = i;
						found_gr = 1;
						if (count < sizeof(GR_Id) / sizeof(u16))
							count++;
					}
				} else
					break;
			}
		}
	}

	count = 0;
	for (i = (nItemID - 1) * 13; i < (13 * nItemID); i++) {
		if ((found_gr == 1) && (i == GR_Id[count] + ((nItemID - 1) * 13)) && (BypassGR == 1)) {
			_gDeltaC[i] = 1;
			if (count < sizeof(GR_Id) / sizeof(u16))
				count++;
		}
	}

	for (i = 0; i < TEST_ITEM_NUM; i++) {
		if (testPin_data[nItemID][i] != PIN_NO_ERROR)
			count_test_pin++;
	}

	for (i = 0; i < count_test_pin; i++) {
		if (0 ==
		    CheckValueInRange(_gDeltaC[i + (nItemID - 1) * 13], ptMutualMpTest->sensorInfo.thrsICpin,
				      -ptMutualMpTest->sensorInfo.thrsICpin)) {
			TestFail[nItemID] = 1;
			nRet = 0;
		}
	}

	return nRet;
}

void Msg28xxChangeANASetting(void)
{
	int i, nMappingItem;
	u8 nChipVer;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	/*Stop mcu */
	/*RegSet16BitValue(0x0FE6, 0x0001); //bank:mheg5, addr:h0073 */
	StopMCU();

	nChipVer = RegGetLByteValue(0x1ECE);

	if (nChipVer != 0)
		RegSetLByteValue(0x131E, 0x01);

	for (nMappingItem = 0; nMappingItem < 6; nMappingItem++) {
		/*sensor mux sram read/write base address / write length */
		RegSetLByteValue(0x2192, 0x00);
		RegSetLByteValue(0x2102, 0x01);
		RegSetLByteValue(0x2102, 0x00);
		RegSetLByteValue(0x2182, 0x08);
		RegSetLByteValue(0x2180, 0x08 * nMappingItem);
		RegSetLByteValue(0x2188, 0x01);

		for (i = 0; i < 8; i++) {
			if (nMappingItem == 0 && nChipVer == 0x0) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_0_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_0_Settings[2 * i + 1]);
				TEST_DBG(0, "_gMuxMem_20_3E_0_Settings\n");
			}
			if ((nMappingItem == 1 && nChipVer == 0x0) || (nMappingItem == 0 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_1_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_1_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_1_Settings[%d] = %x, _gMuxMem_20_3E_1_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_1_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_1_Settings[2 * i + 1]);
			}
			if ((nMappingItem == 2 && nChipVer == 0x0) || (nMappingItem == 1 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_2_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_2_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_2_Settings[%d] = %x, _gMuxMem_20_3E_2_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_2_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_2_Settings[2 * i + 1]);
			}
			if ((nMappingItem == 3 && nChipVer == 0x0) || (nMappingItem == 2 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_3_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_3_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_3_Settings[%d] = %x, _gMuxMem_20_3E_3_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_3_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_3_Settings[2 * i + 1]);
			}
			if ((nMappingItem == 4 && nChipVer == 0x0) || (nMappingItem == 3 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_4_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_4_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_4_Settings[%d] = %x, _gMuxMem_20_3E_4_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_4_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_4_Settings[2 * i + 1]);
			}
			if ((nMappingItem == 5 && nChipVer == 0x0) || (nMappingItem == 4 && nChipVer != 0x0)) {
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_5_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_5_Settings[2 * i + 1]);
				TEST_DBG(0,
					 "_gMuxMem_20_3E_5_Settings[%d] = %x, _gMuxMem_20_3E_5_Settings[%d] = %x\n",
					 2 * i, _gMuxMem_20_3E_5_Settings[2 * i], 2 * i + 1,
					 _gMuxMem_20_3E_5_Settings[2 * i + 1]);
			}

			if (nMappingItem == 5 && nChipVer != 0x0) {	/*useless entry */
				RegSet16BitValue(0x218A, _gMuxMem_20_3E_6_Settings[2 * i]);
				RegSet16BitValue(0x218C, _gMuxMem_20_3E_6_Settings[2 * i + 1]);
			}

		}
	}
}

void Msg28xxAnaLoadSetting(void)
{
	/*Stop mcu */
	/*RegSet16BitValue(0x0FE6, 0x0001); //bank:mheg5, addr:h0073 */
	StopMCU();
	Msg28xxChangeANASetting();
}

/*int deltaC_tmp[MAX_MUTUAL_NUM] = {0};*/

s32 Msg28xxItoShort(void)
{
	/*int deltaC[MAX_MUTUAL_NUM] = {0}; */
	int *deltaC = NULL;
	s16 i;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	deltaC = kmalloc(sizeof(int) * 1904, GFP_KERNEL);

	Msg28xxPrepareAna_for_short_test();
	SetSensorPADState(POS_PULSE);
	/*Msg28xxUpdateSensorPADState(GRPins, GND, _gGRSize); */
	/*DAC overwrite */
	RegSet16BitValue(0x150C, 0x8040);	/*bit15 //AFE:1.3v for test */
	Msg28xxAnaLoadSetting();
	Msg28xxAnaSwReset();
	patchFWAnaSetting_for_shorttest();
	memset(_gDeltaC, 0, sizeof(_gDeltaC));
	if (Msg28xxGetValueR(_gDeltaC) < 0) {
		pr_err("*** Msg28xx Ito Short Test# GetValueR failed! ***\n");
		kfree(deltaC);
		return -EPERM;
	}
	TEST_DBG(0, "*** Msg28xx Ito Short Test# GetValueR 1.3v! ***\n");
	DebugShowArray2(_gDeltaC, 65, -32, 10, 13);

	memset(deltaC, 0, sizeof(*deltaC));
	/*DAC overwrite */
	RegSet16BitValue(0x150C, 0x8083);	/*bit15 //AFE:3.72v for test */
	if (Msg28xxGetValueR(deltaC) < 0) {
		pr_err("*** Msg28xx Short Test# GetValueR failed! ***\n");
		kfree(deltaC);
		return -EPERM;
	}
	TEST_DBG(0, "*** Msg28xx Ito Short Test# GetValueR 3.72v! ***\n");
	DebugShowArray2(deltaC, 65, -32, 10, 13);
	for (i = 0; i < 65; i++) {	/*13 AFE * 5 subframe */
		if ((abs(deltaC[i]) < IIR_MAX) && (abs(_gDeltaC[i]) < IIR_MAX))
			_gDeltaC[i] = deltaC[i] - _gDeltaC[i];
		else {
			if (abs(deltaC[i]) > abs(_gDeltaC[i]))
				_gDeltaC[i] = deltaC[i];
		}

		if (abs(_gDeltaC[i]) >= (IIR_MAX))
			_gDeltaC[i] = 0x7FFF;
		else
			_gDeltaC[i] = abs(_gDeltaC[i]);
	}

	TEST_DBG(0, "*** Msg28xx Ito Short Test# GetValueR 3.72v - 1.3v ! ***\n");
	DebugShowArray2(_gDeltaC, 65, -32, 10, 13);
	kfree(deltaC);

	return 0;
}

s32 Msg28xxICPinShortTest(void)
{
	s16 i = 0, j = 0;
	int nGRSize = 0;
#ifndef DISABLE_DOUBLE
	double *senseR = malloc(MAX_CHANNEL_NUM_28XX * sizeof(double));
	double thrs = 0.0f;
#else
	int *senseR = kmalloc((MAX_CHANNEL_NUM_28XX * sizeof(int)), GFP_KERNEL);
	int thrs = 0;
#endif

	u16 fmode;

	s8 normalTestFail[TEST_ITEM_NUM] = { 0 };	/*0:golden    1:ratio */
	int count_test_pin = 0;

	u16 nTestItemLoop = 6;
	u16 nTestItem = 0;
/*u16 nTestPinMap[6][13] = {{0}};        //6:max subframe    13:max afe*/
	s32 nRetVal = 0;

	u8 time = 0;
	u16 deep_standby = ptMutualMpTest->deep_standby;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	/*DisableFingerTouch(); */
	DrvPlatformLyrDisableFingerTouchReport();

_retry_icpin_short:

	/*TouchDeviceResetHw(); */
	DrvPlatformLyrTouchDeviceResetHw();

	EnterDBBus();
	/*usleep(100000); */
	mdelay(100);

	/*Start mcu */
	StartMCU();
	fmode = MUTUAL_SINGLE_DRIVE;
	if (Msg28xxSwitchFwMode(&fmode, &deep_standby) < 0) {
		pr_err("*** Msg28xx Open Test# SwitchFwMode failed! ***\n");
		time++;
		if (time < 10)
			goto _retry_icpin_short;
		else {
			ptMutualMpTestResult->pCheck_Fail[2] = 1;	/*ic pin short fail */
			nRetVal = -1;
			goto ITO_TEST_END;
		}
	}

	/*Stop mcu */
	StopMCU();

	for (i = 0; i < MAX_CHANNEL_NUM_28XX; i++) {
		senseR[i] = PIN_NO_ERROR;
	}
	thrs = Msg28xxCovertRValue(ptMutualMpTest->sensorInfo.thrsICpin);

	for (i = 1; i < 6; i++) {	/*max 6 subframe */
		for (j = 0; j < 13; j++) {	/*max 13 AFE */
			if (((i - 1) * 13 + j) < MAX_CHANNEL_NUM_28XX)	/*prevent heap corruption detected */
				ptMutualMpTestResult->pICPinShortFailChannel[(i - 1) * 13 + j] =
				    (int)PIN_UN_USE;
			testPin_data[i][j] = PIN_UN_USE;
		}
	}

	/*N1_ShortTest */
	if (Msg28xxICPinShort() < 0) {
		pr_err("*** Msg28xx Short Test# Get DeltaC failed! ***\n");
		ptMutualMpTestResult->pCheck_Fail[2] = 1;	/*ic pin short fail */
		nRetVal = -1;
		goto ITO_TEST_END;
	}
	nGRSize = _gGRSize;	/*Msg28xxreadGR_ICpin(); */
	for (nTestItem = 1; nTestItem < nTestItemLoop; nTestItem++) {
		if (!Msg28xxShortTestJudge_ICpin(nTestItem, (u16 *) _gDeltaC, normalTestFail, nGRSize)) {
			ptMutualMpTestResult->pCheck_Fail[2] = 1;	/*ic pin short fail */
			nRetVal = -1;
			/*goto ITO_TEST_END; */
		}

		count_test_pin = 0;

		for (i = 0; i < 13; i++) {
			if (testPin_data[nTestItem][i] != PIN_UN_USE) {
				/*DEBUG("normalTestFail_check[%d, %d] = %d",
				nTestItem, i, testPin_data[nTestItem][i]); */
				count_test_pin++;
			}
		}
		if ((nTestItem > 0) && (nTestItem < 6)) {
			for (i = 0; i < count_test_pin; i++) {
				if ((testPin_data[nTestItem][i] > 0)
				    && (testPin_data[nTestItem][i] <= MAX_CHANNEL_NUM_28XX)) {
					senseR[testPin_data[nTestItem][i] - 1] =
					    Msg28xxCovertRValue(_gDeltaC[i + (nTestItem - 1) * 13]);
					ptMutualMpTestResult->
					    pICPinShortResultData[testPin_data[nTestItem][i] - 1] =
					    _gDeltaC[i + (nTestItem - 1) * 13];
					/*pr_notice("senseR[%d] ={%.2f}, _gDeltaC[{%d}] ={%d}",
					testPin_data[nTestItem][i] - 1, senseR[testPin_data[nTestItem][i] - 1],
					i + (nTestItem - 1) * 13, _gDeltaC[i + (nTestItem - 1) * 13]); */
					TEST_DBG(0, "senseR[%d] ={%.2d}, _gDeltaC[{%d}] ={%d}\n",
						 testPin_data[nTestItem][i] - 1,
						 senseR[testPin_data[nTestItem][i] - 1],
						 i + (nTestItem - 1) * 13,
						 _gDeltaC[i + (nTestItem - 1) * 13]);
				} else {
					nRetVal = -1;
					/*ic pin short fail */
					ptMutualMpTestResult->pCheck_Fail[2] = 1;
					/*goto ITO_TEST_END; */
				}
			}
		}		/*if ((nTestItem > 0) && (nTestItem < 6)) */
	}
#ifndef DISABLE_DOUBLE
	if (Msg28xxICpin_short_test_result_prepare(thrs, (double *)senseR) < 0) {
#else
	if (Msg28xxICpin_short_test_result_prepare(thrs, (int *)senseR) < 0) {
#endif
		ptMutualMpTestResult->pCheck_Fail[2] = 1;	/*ic pin short fail */
		nRetVal = -1;
	}

	ExitDBBus();

ITO_TEST_END:

	DrvPlatformLyrTouchDeviceResetHw();
	/*TouchDeviceResetHw(); */
	/*usleep(300000); */
	mdelay(300);
	/*EnableFingerTouch(); */
	DrvPlatformLyrEnableFingerTouchReport();
	kfree(senseR);
	return nRetVal;
}

int Msg28xxreadGR_ICpin(void)
{
	int nSize = 0, i;
	char str[512] = { 0 };
/*int testPins[13] = {0};*/

	for (i = 0; i < sizeof(GRPins) / sizeof(u16); i++) {
		GRPins[i] = 0xFFFF;
	}

	TEST_DBG(0, " *** %s ***\n", __func__);

	/*nSize = IniGetIntArray("[SHORT_TEST_GR]", "TEST_PIN", _gGRtestPins); */
	ms_getInidata("SHORT_TEST_GR", "TEST_PIN", str);
	nSize = ms_ini_split_int_array(str, _gGRtestPins);
	for (i = 0; i < 13; i++)
		TEST_DBG(0, "_gGRtestPins[%d] = %d\n", i, _gGRtestPins[i]);
	TEST_DBG(0, "nSize = %d\n", nSize);
	for (i = 0; i < nSize; i++) {
		GRPins[i] = _gGRtestPins[i];
		TEST_DBG(0, "GRPins[%d] = %d\n", i, GRPins[i]);
	}
	TEST_DBG(0, " *** %s end ***\n", __func__);
	return nSize;
}

u16 Msg28xxreadTestPins_28XX(u16 itemID, int *testPins)
{
	int nSize = 0, i;
	char str[512] = { 0 };

	TEST_DBG(0, "*** %s() ***\n", __func__);

	switch (itemID) {
	case 1:
	case 11:
		/*nSize = IniGetIntArray("[SHORT_TEST_N1]", "TEST_PIN", testPins); */
		ms_getInidata("SHORT_TEST_N1", "TEST_PIN", str);
		nSize = ms_ini_split_int_array(str, testPins);
		TEST_DBG(0, "SHORT_TEST_N1 nSize = %d\n", nSize);
		break;
	case 2:
	case 12:
		/*nSize = IniGetIntArray("[SHORT_TEST_N2]", "TEST_PIN", testPins); */
		ms_getInidata("SHORT_TEST_N2", "TEST_PIN", str);
		nSize = ms_ini_split_int_array(str, testPins);
		TEST_DBG(0, "SHORT_TEST_N2 nSize = %d\n", nSize);
		break;
	case 3:
	case 13:
		/*nSize = IniGetIntArray("[SHORT_TEST_S1]", "TEST_PIN", testPins); */
		ms_getInidata("SHORT_TEST_S1", "TEST_PIN", str);
		nSize = ms_ini_split_int_array(str, testPins);
		TEST_DBG(0, "SHORT_TEST_S1 nSize = %d\n", nSize);
		break;
	case 4:
	case 14:
		/*nSize = IniGetIntArray("[SHORT_TEST_S2]", "TEST_PIN", testPins); */
		ms_getInidata("SHORT_TEST_S2", "TEST_PIN", str);
		nSize = ms_ini_split_int_array(str, testPins);
		TEST_DBG(0, "SHORT_TEST_S2 nSize = %d\n", nSize);
		break;

	case 5:
	case 15:
		/*if (IniGetString("[SHORT_TEST_N3]", "MUX_MEM_20_3E") != 0) {
		nSize = IniGetIntArray("[SHORT_TEST_N3]", "TEST_PIN", testPins);
		pr_notice("SHORT_TEST_N3 nSize = %d\n", nSize);
		}
		else if (IniGetString("[SHORT_TEST_S3]", "MUX_MEM_20_3E") != 0) {
		nSize = IniGetIntArray("[SHORT_TEST_S3]", "TEST_PIN", testPins);
		pr_notice("SHORT_TEST_S3 nSize = %d\n", nSize);
		}
		else if (IniGetString("[SHORT_TEST_GR]", "MUX_MEM_20_3E") != 0) {
		if (ptMutualMpTest->sensorInfo.numGr == 0)
		nSize = 0;
		else
		nSize = _gGRSize;

		for (i = 0; i < sizeof(_gGRtestPins) / sizeof(_gGRtestPins[0]); i++)
		testPins[i] = _gGRtestPins[i];
		pr_notice("SHORT_TEST_GR nSize = %d\n", nSize);
		} */

		if (ms_getInidata("SHORT_TEST_N3", "MUX_MEM_20_3E", str) != 0) {
			ms_getInidata("SHORT_TEST_N3", "TEST_PIN", str);
			nSize = ms_ini_split_int_array(str, testPins);
			TEST_DBG(0, "SHORT_TEST_N3 nSize = %d\n", nSize);
		} else if (ms_getInidata("SHORT_TEST_S3", "MUX_MEM_20_3E", str) != 0) {
			ms_getInidata("SHORT_TEST_S3", "TEST_PIN", str);
			nSize = ms_ini_split_int_array(str, testPins);
			TEST_DBG(0, "SHORT_TEST_S3 nSize = %d\n", nSize);
		} else if (ms_getInidata("SHORT_TEST_GR", "MUX_MEM_20_3E", str) != 0) {
			if (ptMutualMpTest->sensorInfo.numGr == 0)
				nSize = 0;
			else
				nSize = _gGRSize;

			for (i = 0; i < sizeof(_gGRtestPins) / sizeof(int); i++)
				testPins[i] = _gGRtestPins[i];
			TEST_DBG(0, "SHORT_TEST_GR nSize = %d\n", nSize);
		}

		break;

	case 0:
	default:
		return 0;
	}

	for (i = nSize; i < MAX_CHANNEL_NUM_28XX; i++) {
		testPins[i] = PIN_NO_ERROR;
	}

	return nSize;
}

/*s32 Msg28xxShortTestJudge(u8 nItemID, s8 *TestFail, u16 *TestFail_check)*/
s32 Msg28xxShortTestJudge(u8 nItemID, s8 *TestFail, u16 TestFail_check[][MAX_MUTUAL_NUM])
{
	int nRet = 1, i, count_test_pin = 0;
	int testPins[MAX_CHANNEL_NUM_28XX] = { 0 };

	TEST_DBG(0, "*** %s() ***\n", __func__);

	Msg28xxreadTestPins_28XX(nItemID, testPins);

	for (i = 0; i < sizeof(testPins) / sizeof(int); i++) {
		if (testPins[i] != PIN_NO_ERROR)
			count_test_pin++;
	}

	for (i = 0; i < count_test_pin; i++) {
		TestFail_check[nItemID][i] = testPins[i];
		/**(TestFail_check+nItemID*TEST_ITEM_NUM+i) = testPins[i];*/
		TEST_DBG(0, "testPins[%d] =%d\n", i, testPins[i]);
		/*if (0 == CheckValueInRange(_gDeltaC[i + (nItemID - 1) * 13],
		ptMutualMpTest->sensorInfo.thrsShort, -1000)) */
		if (0 ==
		    CheckValueInRange(_gDeltaC[i + (nItemID - 1) * 13], ptMutualMpTest->sensorInfo.thrsShort,
				      -ptMutualMpTest->sensorInfo.thrsShort)) {
			TestFail[nItemID] = 1;
			nRet = 0;
		}
	}

	return nRet;
}

s8 normalTestFail[TEST_ITEM_NUM] = { 0 };	/*0:golden    1:ratio */

u16 normalTestFail_check[TEST_ITEM_NUM][MAX_MUTUAL_NUM];/*6:max subframe    13:max afe */

s32 Msg28xxItoShortTest(void)
{
	s16 i = 0, j = 0;
#ifndef DISABLE_DOUBLE
	double *senseR = malloc(_gSenseNum * sizeof(double));
	double *driveR = malloc(_gDriveNum * sizeof(double));
	double *GRR = malloc(_gGRNum * sizeof(double));
	double thrs = 0.0f;
#else
	int *senseR = kmalloc((ptMutualMpTest->sensorInfo.numSen * sizeof(int)), GFP_KERNEL);
	int *driveR = kmalloc((ptMutualMpTest->sensorInfo.numDrv * sizeof(int)), GFP_KERNEL);
	int *GRR = kmalloc((ptMutualMpTest->sensorInfo.numGr * sizeof(int)), GFP_KERNEL);
	int thrs = 0;
#endif

	u16 Temp_20_3E_Settings[16] = { 0 };

	u16 nTestItemLoop = 6;
	u16 nTestItem = 0;
/*u16 nTestPinMap[6][13] = {{0}};//6:max subframe    13:max afe*/
	u32 nRetVal = 0;
	u16 fmode;

	u8 time = 0;
	u16 deep_standby = ptMutualMpTest->deep_standby;
	int count_test_pin = 0;
	char str[512] = { 0 };

	TEST_DBG(0, "*** %s() ***\n", __func__);

	_gSenseNum = ptMutualMpTest->sensorInfo.numSen;
	_gDriveNum = ptMutualMpTest->sensorInfo.numDrv;
	_gGRNum = ptMutualMpTest->sensorInfo.numGr;
	/*DisableFingerTouch(); */
	DrvPlatformLyrDisableFingerTouchReport();

_retry_short:

	/*TouchDeviceResetHw(); */
	DrvPlatformLyrTouchDeviceResetHw();

	EnterDBBus();
	/*usleep(100000); */
	mdelay(100);

	/*Start mcu */
	StartMCU();
	fmode = MUTUAL_SINGLE_DRIVE;
	if (Msg28xxSwitchFwMode(&fmode, &deep_standby) < 0) {
		pr_err("*** Msg28xx Ito Short Test# SwitchFwMode failed! ***\n");
		time++;
		if (time < 10)
			goto _retry_short;
		else {
			nRetVal = -1;
			goto ITO_TEST_END;
		}
	}

	/*Stop mcu */
	StopMCU();

	thrs = Msg28xxCovertRValue(ptMutualMpTest->sensorInfo.thrsShort);

	for (i = 0; i < _gSenseNum; i++)
		senseR[i] = thrs;
	for (i = 0; i < _gDriveNum; i++)
		driveR[i] = thrs;
	for (i = 0; i < _gGRNum; i++)
		GRR[i] = thrs;

	for (i = 1; i < 6; i++) {	/*max 6 subframe */
		for (j = 0; j < 13; j++) {	/*max 13 AFE */
			if (((i - 1) * 13 + j) < MAX_CHANNEL_NUM_28XX)
				ptMutualMpTestResult->pShortFailChannel[(i - 1) * 13 + j] = (u32) PIN_UN_USE;
			/**(normalTestFail_check+i*TEST_ITEM_NUM+j)=PIN_UN_USE;*/
			normalTestFail_check[i][j] = PIN_UN_USE;
		}
	}

	memset(normalTestFail, 0, TEST_ITEM_NUM * sizeof(s8));

	/*DEBUG("*** Initialize ShortFailChannel ***"); */
	/*DebugShowArray2(ptMutualMpTestResult->pShortFailChannel, 65, 32, 16, 13); */
	/*DEBUG("*** Pad2SENSE ***"); */
	/*DebugShowArray2(ptMutualMpTest->PAD2Sense, 65, 16, 16, 13); */
	/*N1_ShortTest */
	if (Msg28xxItoShort() < 0) {
		pr_err("*** Msg28xx Ito Short Test# Get DeltaC failed! ***\n");
		nRetVal = -1;
		goto ITO_TEST_END;
	}

	for (nTestItem = 1; nTestItem < nTestItemLoop; nTestItem++) {
		if (!Msg28xxShortTestJudge(nTestItem, normalTestFail, normalTestFail_check)) {
			pr_err("*** Msg28xx Ito Short Test# ShortTestJudge failed! ***\n");
			nRetVal = -1;
			/*goto ITO_TEST_END; */
		}

		count_test_pin = 0;

		for (i = 0; i < 13; i++) {
			/*DEBUG("normalTestFail_check[%d][%d] = %x",
				nTestItem, i, normalTestFail_check[nTestItem][i]); */
			if (normalTestFail_check[nTestItem][i] != PIN_UN_USE)
			/*if (*(normalTestFail_check+nTestItem*TEST_ITEM_NUM+i) != PIN_UN_USE) */
			{
				count_test_pin++;
			}
		}

		TEST_DBG(0, "nTestItem = %d, count_test_pin = %d , _gSenseNum = %d\n", nTestItem,
			 count_test_pin, _gSenseNum);

		memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
		ms_getInidata("SHORT_TEST_N3", "MUX_MEM_20_3E", str);
		if (str != NULL) {
			if (nTestItem == 1 || nTestItem == 2
			    || (nTestItem == 5 && (ms_ini_split_u16_array(str, Temp_20_3E_Settings)))) {
				TEST_DBG(0, "SHORT_TEST_N3\n");
				for (i = 0; i < count_test_pin; i++) {
					for (j = 0; j < _gSenseNum; j++) {
						if (normalTestFail_check[nTestItem][i] ==
						    ptMutualMpTest->PAD2Sense[j]) {
							senseR[j] =
							    Msg28xxCovertRValue(_gDeltaC[i + (nTestItem - 1) * 13]);
							TEST_DBG(0, "senseR[%d] = %.2d, _gDeltaC[%d] = %d\n",
								 j, senseR[j], i + (nTestItem - 1) * 13,
								 _gDeltaC[i + (nTestItem - 1) * 13]);
							ptMutualMpTestResult->pShortRData[j] = senseR[j];
							ptMutualMpTestResult->pShortResultData[j] =
							    _gDeltaC[i + (nTestItem - 1) * 13];
							if (senseR[j] >= 10)
								ptMutualMpTestResult->pShortRData[j] = 10.0;
							if (0 ==
							    CheckValueInRange(_gDeltaC[i + (nTestItem - 1) * 13],
									      ptMutualMpTest->sensorInfo.thrsShort,
									      -ptMutualMpTest->sensorInfo.thrsShort)) {
								ptMutualMpTestResult->pShortFailChannel[j] =
								    (u32) normalTestFail_check[nTestItem][i];
							}
						} /* if (normalTestFail_check[nTestItem, i].... */
					} /* for (j = 0; j < sizeof(sens...*/
				} /* for (i = 0; i < count_test_pin; i++) */
			} /* if (nTestItem == 1 || nTe....*/
		}
		memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
		ms_getInidata("SHORT_TEST_S3", "MUX_MEM_20_3E", str);
		if (str != NULL) {
			if (nTestItem == 3 || nTestItem == 4
			    || (nTestItem == 5 && (ms_ini_split_u16_array(str, Temp_20_3E_Settings)))) {
				TEST_DBG(0, "SHORT_TEST_S3\n");
				for (i = 0; i < count_test_pin; i++) {
					for (j = 0; j < _gDriveNum; j++) {
						if (normalTestFail_check[nTestItem][i] ==
						    ptMutualMpTest->PAD2Drive[j]) {
							driveR[j] =
							    Msg28xxCovertRValue(_gDeltaC[i + (nTestItem - 1) * 13]);
							TEST_DBG(0, "driveR[%d] = %.2d, _gDeltaC[%d] = %d\n",
								 j, driveR[j], i + (nTestItem - 1) * 13,
								 _gDeltaC[i + (nTestItem - 1) * 13]);
							ptMutualMpTestResult->pShortRData[_gSenseNum + j]
								= driveR[j];
							ptMutualMpTestResult->pShortResultData[_gSenseNum + j]
								= _gDeltaC[i + (nTestItem - 1) * 13];
							if (driveR[j] >= 10)
								ptMutualMpTestResult->pShortRData
								[_gSenseNum + j] = 10.0;
							if (0 ==
							    CheckValueInRange(_gDeltaC[i + (nTestItem - 1) * 13],
									      ptMutualMpTest->sensorInfo.thrsShort,
									      -ptMutualMpTest->sensorInfo.thrsShort)) {
							ptMutualMpTestResult->pShortFailChannel[_gSenseNum + j] =
								(u32) normalTestFail_check[nTestItem][i];
							}
						}	/*if (normalTestFail_check[nTestItem, i] .... */
					}	/*for (j = 0; j < sizeof(driveR.... */
				}	/*for (i = 0; i < count_test_pin; i++) */
			}	/*if (nTestItem == 3 || nTe.... */
		}

		memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
		ms_getInidata("SHORT_TEST_GR", "MUX_MEM_20_3E", str);
		if (str != NULL) {
			if (nTestItem == 5 && (ms_ini_split_u16_array(str, Temp_20_3E_Settings))) {
				for (i = 0; i < count_test_pin; i++) {
					for (j = 0; j < _gGRNum; j++) {
						TEST_DBG(0,
							 "normalTestFail_check[nTestItem = %d][i = %d] = %d, ptMutualMpTest->PAD2GR[j = %d] = %d\n",
							 nTestItem, i, normalTestFail_check[nTestItem][i], j,
							 ptMutualMpTest->PAD2GR[j]);
						if (normalTestFail_check[nTestItem][i] ==
						    ptMutualMpTest->PAD2GR[j]) {
							GRR[j] =
								Msg28xxCovertRValue(_gDeltaC[i + (nTestItem - 1) * 13]);
							ptMutualMpTestResult->pShortRData[_gSenseNum + _gDriveNum + j]
								= GRR[j];
							ptMutualMpTestResult->pShortResultData
								[_gSenseNum + _gDriveNum + j]
								=  _gDeltaC[i + (nTestItem - 1) * 13];
							if (GRR[j] >= 10)
								ptMutualMpTestResult->pShortRData
								[_gSenseNum + _gDriveNum + j] = 10.0;
							if (0 ==
							    CheckValueInRange(_gDeltaC[i + (nTestItem - 1) * 13],
									      ptMutualMpTest->sensorInfo.thrsShort,
									      -ptMutualMpTest->sensorInfo.thrsShort)) {
								ptMutualMpTestResult->pShortFailChannel
									[_gSenseNum + _gDriveNum + j]
									= (u32) normalTestFail_check[nTestItem][i];
							}
						}	/*if (normalTestFail_check[nTestItem, i] */
					}	/*for (j = 0; j < sizeof(GRR) */
				}	/*for (i = 0; i < count_test_pin; i++) */
			}	/*if (nTestItem == 5 && */
		}

		if (normalTestFail[nTestItem]) {
			/*ito short fail */
			ptMutualMpTestResult->pCheck_Fail[3] = normalTestFail[nTestItem];
			nRetVal = -1;
		}
	}

	ExitDBBus();

ITO_TEST_END:

	DrvPlatformLyrTouchDeviceResetHw();
	mdelay(300);
	DrvPlatformLyrEnableFingerTouchReport();

	kfree(senseR);
	kfree(driveR);
	kfree(GRR);
	return nRetVal;
}

s32 Msg28xxShortTestEntry(void)
{
	u32 nRetVal1 = 0, nRetVal2 = 0;

	TEST_DBG(0, "*** %s() ***\n", __func__);
	readSettings();
	_gSenseNum = ptMutualMpTest->sensorInfo.numSen;
	_gDriveNum = ptMutualMpTest->sensorInfo.numDrv;
	_gGRNum = ptMutualMpTest->sensorInfo.numGr;
	/*must execute once only , or it will return invalid buffer data
		(IniGetIntArray("SHORT_TEST_GR", "TEST_PIN", testPins)) */
	_gGRSize = Msg28xxreadGR_ICpin();
	nRetVal1 = Msg28xxItoShortTest();
	nRetVal2 = Msg28xxICPinShortTest();

	TEST_DBG(0, "nRetVal1 = %d, nRetVal2 = %d\n", nRetVal1, nRetVal2);

	if ((nRetVal1 == -1) || (nRetVal2 == -1))
		return -EPERM;
	else
		return 0;
}

int Msg28xxShortTest(void)
{
	int nRetVal = 0;
	int nRet = 0;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	nRetVal = Msg28xxShortTestEntry();
	if (nRetVal == 0) {
		nRet = ITO_TEST_OK;
		/*PASS*/ pr_notice("Msg28xx Short Test# MP test success\n");
	} else {
		if (nRetVal == -1) {
			pr_notice("Msg28xx Short Test# MP test fail\n");
			nRet = ITO_TEST_FAIL;
		} else if (nRetVal == -2) {
			nRet = ITO_TEST_GET_TP_TYPE_ERROR;
		} else {
			nRet = ITO_TEST_UNDEFINED_ERROR;
		}

		pr_err("Msg28xx Short# MP test failed\n");
	}

	return nRet;
}
