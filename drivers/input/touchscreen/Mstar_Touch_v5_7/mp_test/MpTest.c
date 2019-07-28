#include "global.h"
#include "IniParser/convert_file_op.h"

#define FILENAME_MAX 1024

MutualMpTest_t *ptMutualMpTest = NULL;
MutualMpTestResult_t *ptMutualMpTestResult = NULL;
int nCount = 0;
u8 nAna_version[100] = { 0 };

char *iniFilePath = NULL;

extern struct i2c_client *g_I2cClient;

int ascii_to_hex(char ch)
{
	char ch_tmp;
	int hex_val = -1;

	ch_tmp = tolower(ch);

	if ((ch_tmp >= '0') && (ch_tmp <= '9')) {
		hex_val = ch_tmp - '0';
	} else if ((ch_tmp >= 'a') && (ch_tmp <= 'f')) {
		hex_val = ch_tmp - 'a' + 10;
	}

	return hex_val;
}

int str_to_hex(char *hex_str)
{
	int i, len;
	int hex_tmp, hex_val;

	len = strlen(hex_str);
	hex_val = 0;
	for (i = 0; i < len; i++) {
		hex_tmp = ascii_to_hex(hex_str[i]);

		if (hex_tmp == -1) {
			return -EPERM;
		}

		hex_val = (hex_val) * 16 + hex_tmp;
	}
	return hex_val;
}

int Msg28xxLoadIni(char *pFilePath)
{
/*int nOpenTest;*/
/*int nShortTest;*/
/*int nMapSize = 0;*/
	int nSize;
/*int i;*/
	char str[512] = { 0 };

	TEST_DBG(&g_I2cClient->dev, "*** %s : iniPath = %s ***\n", __func__, pFilePath);

	iniFilePath = kmalloc(100, GFP_KERNEL);
	strlcpy(iniFilePath, pFilePath, 100);

	if (my_parser(iniFilePath) < 0) {
		pr_err("*** %s : failed to parse file = %s ***\n", __func__, pFilePath);
		return -EPERM;
	}

	pr_notice("Open %s success!\n", iniFilePath);

	ptMutualMpTest = kcalloc(1, sizeof(MutualMpTest_t), GFP_KERNEL);
	ptMutualMpTestResult = kcalloc(1, sizeof(MutualMpTestResult_t), GFP_KERNEL);

	ms_getInidata("UI_CONFIG", "OpenTest", str);
	ptMutualMpTest->UIConfig.bOpenTest = ms_atoi(str);
	ms_getInidata("UI_CONFIG", "ShortTest", str);
	ptMutualMpTest->UIConfig.bShortTest = ms_atoi(str);
	ms_getInidata("UI_CONFIG", "WpTest", str);
	ptMutualMpTest->UIConfig.bWpTest = ms_atoi(str);
	ms_getInidata("UI_CONFIG", "ANAGEN_VER", str);
	ptMutualMpTest->ana_version = kmalloc((FILENAME_MAX * sizeof(char)), GFP_KERNEL);
	strlcpy(ptMutualMpTest->ana_version, str, FILENAME_MAX * sizeof(char));
	nCount = ms_ini_split_u8_array(ptMutualMpTest->ana_version, nAna_version);

	TEST_DBG(0, "%s: nCount = %d\n", __func__, nCount);

	ms_getInidata("SENSOR", "DrvNum", str);
	ptMutualMpTest->sensorInfo.numDrv = ms_atoi(str);
	ms_getInidata("SENSOR", "SenNum", str);
	ptMutualMpTest->sensorInfo.numSen = ms_atoi(str);
	ms_getInidata("SENSOR", "KeyNum", str);
	ptMutualMpTest->sensorInfo.numKey = ms_atoi(str);
	ms_getInidata("SENSOR", "KeyLine", str);
	ptMutualMpTest->sensorInfo.numKeyLine = ms_atoi(str);
	ms_getInidata("SENSOR", "GrNum", str);
	ptMutualMpTest->sensorInfo.numGr = ms_atoi(str);

	ms_getInidata("OPEN_TEST_N", "CSUB_REF", str);
	ptMutualMpTest->Open_test_csub = ms_atoi(str);
	ms_getInidata("OPEN_TEST_N", "CFB_REF", str);
	ptMutualMpTest->Open_test_cfb = ms_atoi(str);
	ms_getInidata("OPEN_TEST_N", "OPEN_MODE", str);
	ptMutualMpTest->Open_mode = ms_atoi(str);
	ms_getInidata("OPEN_TEST_N", "FIXED_CARRIER", str);
	ptMutualMpTest->Open_fixed_carrier = ms_atoi(str);
	ms_getInidata("OPEN_TEST_N", "CHARGE_PUMP", str);
	ptMutualMpTest->Open_test_chargepump = ms_atoi(str);

	ms_getInidata("INFORMATION", "MutualKey", str);
	ptMutualMpTest->Mutual_Key = ms_atoi(str);
	ms_getInidata("INFORMATION", "Pattern_type", str);
	ptMutualMpTest->Pattern_type = ms_atoi(str);
	ms_getInidata("INFORMATION", "1T2R_MODEL", str);
	ptMutualMpTest->Pattern_model = ms_atoi(str);

	ms_getInidata("INFORMATION", "RETRY", str);
	ptMutualMpTestResult->nRetry = ms_atoi(str);

	if (ptMutualMpTestResult->nRetry <= 0)
		ptMutualMpTestResult->nRetry = 3;

	TEST_DBG(0, "ptMutualMpTestResult->nRetry  = %d\n", ptMutualMpTestResult->nRetry);

	ms_getInidata("RULES", "DC_Range", str);
	ptMutualMpTest->ToastInfo.persentDC_VA_Range = ms_atoi(str);
	ms_getInidata("RULES", "DC_Range_up", str);
	ptMutualMpTest->ToastInfo.persentDC_VA_Range_up = ms_atoi(str);
	ms_getInidata("RULES", "DC_Ratio", str);
	ptMutualMpTest->ToastInfo.persentDC_VA_Ratio = ms_atoi(str);
	ms_getInidata("RULES", "DC_Ratio_up", str);
	ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up = ms_atoi(str);
	ms_getInidata("RULES", "DC_Border_Ratio", str);
	ptMutualMpTest->ToastInfo.persentDC_Border_Ratio = ms_atoi(str);

	ms_getInidata("BASIC", "DEEP_STANDBY", str);
	ptMutualMpTest->deep_standby = ms_atoi(str);
	TEST_DBG(0, "DEEP_STANDBY = %d\n", ptMutualMpTest->deep_standby);

	ms_getInidata("BASIC", "DEEP_STANDBY_TIMEOUT", str);
	ptMutualMpTest->deep_standby_timeout = ms_atoi(str);

	if ((ptMutualMpTest->Mutual_Key == 1) || (ptMutualMpTest->Mutual_Key == 2)) {
		ms_getInidata("SENSOR", "KEY_CH", str);
		ptMutualMpTest->sensorInfo.KEY_CH = ms_atoi(str);
	}

	ms_getInidata("PHASE_CALIBRATION", "SUB_FRAME", str);
	ptMutualMpTest->sub_frame = ms_atoi(str);

	/*ptMutualMpTest->UIConfig.bOpenTest = IniGetInt("[UI_CONFIG]", "OpenTest"); */
	/*ptMutualMpTest->UIConfig.bShortTest = IniGetInt("[UI_CONFIG]", "ShortTest"); */
	/*ptMutualMpTest->UIConfig.bWpTest = IniGetInt("[UI_CONFIG]", "WpTest"); */
	/*ptMutualMpTest->ana_version = (char *)kmalloc((FILENAME_MAX * sizeof(char)), GFP_KERNEL); */
	/*strlcpy(ptMutualMpTest->ana_version, IniGetString("[UI_CONFIG]",
		"ANAGEN_VER"), FILENAME_MAX * sizeof(char)); */
	/*nCount = IniSplitU8Array("[UI_CONFIG]", "ANAGEN_VER", nAna_version); */

	/*ptMutualMpTest->sensorInfo.numDrv = (u16)IniGetInt("[SENSOR]", "DrvNum"); */
	/*ptMutualMpTest->sensorInfo.numSen = (u16)IniGetInt("[SENSOR]", "SenNum"); */
	/*ptMutualMpTest->sensorInfo.numKey = (u16)IniGetInt("[SENSOR]", "KeyNum"); */
	/*ptMutualMpTest->sensorInfo.numKeyLine = (u16)IniGetInt("[SENSOR]", "KeyLine"); */
	/*ptMutualMpTest->sensorInfo.numGr = (u16)IniGetInt("[SENSOR]", "GrNum"); */
	/*ptMutualMpTest->Open_test_csub = IniGetInt("[OPEN_TEST_N]", "CSUB_REF"); */
	/*ptMutualMpTest->Open_test_cfb = IniGetInt("[OPEN_TEST_N]", "CFB_REF"); */
	/*ptMutualMpTest->Open_mode = IniGetInt("[OPEN_TEST_N]", "OPEN_MODE"); */
	/*ptMutualMpTest->Open_fixed_carrier = IniGetInt("[OPEN_TEST_N]", "FIXED_CARRIER"); */
	/*ptMutualMpTest->Open_test_chargepump = IniGetInt("[OPEN_TEST_N]", "CHARGE_PUMP"); */
	/*ptMutualMpTest->Mutual_Key = IniGetInt("[INFORMATION]", "MutualKey"); */
	/*ptMutualMpTest->Pattern_type = IniGetInt("[INFORMATION]", "Pattern_type"); */
	/*ptMutualMpTest->Pattern_model = IniGetInt("[INFORMATION]", "1T2R_MODEL"); */
	/*ptMutualMpTest->ToastInfo.persentDC_VA_Range = (u16)IniGetInt("[RULES]", "DC_Range"); */
	/*ptMutualMpTest->ToastInfo.persentDC_VA_Range_up = (u16)IniGetInt("[RULES]", "DC_Range_up"); */
	/*ptMutualMpTest->ToastInfo.persentDC_VA_Ratio = (u16)IniGetInt("[RULES]", "DC_Ratio"); */
	/*ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up = (u16)IniGetInt("[RULES]", "DC_Ratio_up"); */
	/*ptMutualMpTest->ToastInfo.persentDC_Border_Ratio = (u16)IniGetInt("[RULES[", "DC_Border_Ratio"); */
	/*ptMutualMpTest->deep_standby = (u16)IniGetInt("[BASIC]", "DEEP_STANDBY"); */
	/*pr_notice("DEEP_STANDBY = %d\n", ptMutualMpTest->deep_standby); */
	/*ptMutualMpTest->deep_standby_timeout = (u16)IniGetInt("[BASIC]", "DEEP_STANDBY_TIMEOUT"); */
	/*if ((ptMutualMpTest->Mutual_Key == 1) && (ptMutualMpTest->Mutual_Key == 2)) */
	/*ptMutualMpTest->sensorInfo.KEY_CH = (u16)IniGetInt("[SENSOR]", "KEY_CH"); */

	/*ptMutualMpTest->sub_frame = (u16)IniGetInt("[PHASE_CALIBRATION]", "SUB_FRAME"); */

	pr_notice("ANAGEN_VER:    [%s]\n", ptMutualMpTest->ana_version);
	pr_notice("OpenTest:      [%d]\n", ptMutualMpTest->UIConfig.bOpenTest);
	pr_notice("ShortTest:      [%d]\n", ptMutualMpTest->UIConfig.bShortTest);
	pr_notice("WPTest:      [%d]\n", ptMutualMpTest->UIConfig.bWpTest);
	pr_notice("DrvNum:      [%d]\n", ptMutualMpTest->sensorInfo.numDrv);
	pr_notice("SenNum:      [%d]\n", ptMutualMpTest->sensorInfo.numSen);
	pr_notice("KeyNum:      [%d]\n", ptMutualMpTest->sensorInfo.numKey);
	pr_notice("KeyLine:      [%d]\n", ptMutualMpTest->sensorInfo.numKeyLine);
	pr_notice("GrNum:      [%d]\n", ptMutualMpTest->sensorInfo.numGr);
	pr_notice("CSUB_REF:      [%d]\n", ptMutualMpTest->Open_test_csub);
	pr_notice("CFB_REF:      [%d]\n", ptMutualMpTest->Open_test_cfb);
	pr_notice("OPEN_MODE:      [%d]\n", ptMutualMpTest->Open_mode);
	pr_notice("FIXED_CARRIER:      [%d]\n", ptMutualMpTest->Open_fixed_carrier);
	pr_notice("CHARGE_PUMP:      [%d]\n", ptMutualMpTest->Open_test_chargepump);
	pr_notice("MutualKey:      [%d]\n", ptMutualMpTest->Mutual_Key);
	pr_notice("KEY_CH:      [%d]\n", ptMutualMpTest->sensorInfo.KEY_CH);
	pr_notice("Pattern_type:      [%d]\n", ptMutualMpTest->Pattern_type);
	pr_notice("Pattern_model:      [%d]\n", ptMutualMpTest->Pattern_model);
	pr_notice("DC_Range:      [%d]\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range);
	pr_notice("DC_Ratio:      [%d]\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio);
	pr_notice("DC_Range_up:      [%d]\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range_up);
	pr_notice("DC_Ratio_up:      [%d]\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up);
	pr_notice("SUB_FRAME:      [%d]\n", ptMutualMpTest->sub_frame);

	if (ptMutualMpTest->sensorInfo.numKey > 0) {
		ptMutualMpTest->KeySen =
		   kcalloc(ptMutualMpTest->sensorInfo.numKey, sizeof(int), GFP_KERNEL);
		ms_getInidata("SENSOR", "KEYSEN", str);
		strlcpy((char *)ptMutualMpTest->KeySen, str, ptMutualMpTest->sensorInfo.numKey * sizeof(int));
		ptMutualMpTest->sensorInfo.numKey = ms_ini_split_int_array(str, ptMutualMpTest->KeySen);

		ms_getInidata("SENSOR", "KEyDrv_o", str);
		ptMutualMpTest->sensorInfo.KeyDrv_o = ms_atoi(str);
	}

	/*if ((nCount) && (nAna_version[2] <= 15)) {
	nMapSize = IniSecKeysTo2DArray2(iniFilePath, "MAPPING_TABLE_NORMAL", szMapTable);
	ptMutualMpTest->sensorInfo.mapping = (MutualMapping_t *)kcalloc(nMapSize, sizeof(MutualMapping_t), GFP_KERNEL);
	pr_notice("mapping number = %d\n", nMapSize);

	for(i=0; i<nMapSize; i++){
	ptMutualMpTest->sensorInfo.mapping[i].X = szMapTable[i][0];
	ptMutualMpTest->sensorInfo.mapping[i].Y = szMapTable[i][1];
	}
	} */

	ptMutualMpTest->UIConfig.sSupportIC = kmalloc((FILENAME_MAX * sizeof(char)), GFP_KERNEL);
	memset(ptMutualMpTest->UIConfig.sSupportIC, 0, sizeof(*ptMutualMpTest->UIConfig.sSupportIC));
	if (ms_getInidata("UI_CONFIG", "SupportIC", str) != 0)
		strlcpy(ptMutualMpTest->UIConfig.sSupportIC, str, FILENAME_MAX * sizeof(char));

	TEST_DBG(0, "SupportIC:      [%s]\n", ptMutualMpTest->UIConfig.sSupportIC);

	ptMutualMpTest->project_name = kmalloc((FILENAME_MAX * sizeof(char)), GFP_KERNEL);
	memset(ptMutualMpTest->project_name, 0, sizeof(*ptMutualMpTest->project_name));
	if (ms_getInidata("INFORMATION", "PROJECT", str) != 0)
		strlcpy(ptMutualMpTest->project_name, str, FILENAME_MAX * sizeof(char));

	TEST_DBG(0, "PROJECT:      [%s]\n", ptMutualMpTest->project_name);

#ifndef DISABLE_DOUBLE
	ptMutualMpTest->Goldensample_CH_0 = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
	nSize = 0;
	nSize = IniGetDoubleArray("[RULES]", "Golden_CH_0", ptMutualMpTest->Goldensample_CH_0);
#else
	ptMutualMpTest->Goldensample_CH_0 = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	/*nSize = IniGetDoubleArray("[RULES]","Golden_CH_0",ptMutualMpTest->Goldensample_CH_0);
	ms_getInidata("RULES","Golden_CH_0",str);
	pr_notice("%s: s = %s\n", __func__,str); */
	nSize = ms_ini_split_golden(ptMutualMpTest->Goldensample_CH_0);
	TEST_DBG(0, "%s: nSize = %d\n", __func__, nSize);
#endif

#ifndef DISABLE_DOUBLE
	ptMutualMpTest->Goldensample_CH_0_Max = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
	ptMutualMpTest->Goldensample_CH_0_Max_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
	ptMutualMpTest->Goldensample_CH_0_Min = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
	ptMutualMpTest->Goldensample_CH_0_Min_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
#else
	ptMutualMpTest->Goldensample_CH_0_Max = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	ptMutualMpTest->Goldensample_CH_0_Max_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	ptMutualMpTest->Goldensample_CH_0_Min = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	ptMutualMpTest->Goldensample_CH_0_Min_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
#endif

	if (ptMutualMpTest->sensorInfo.numDrv && ptMutualMpTest->sensorInfo.numSen) {
		ptMutualMpTest->PAD2Drive =
		    kmalloc(ptMutualMpTest->sensorInfo.numDrv * sizeof(u16), GFP_KERNEL);
		ms_getInidata("PAD_TABLE", "DRIVE", str);
		TEST_DBG(0, "PAD_TABLE(DRIVE):      [%s]\n", str);
		nSize = 0;
		nSize = ms_ini_split_u16_array(str, ptMutualMpTest->PAD2Drive);

		ptMutualMpTest->PAD2Sense =
		   kmalloc(ptMutualMpTest->sensorInfo.numSen * sizeof(u16), GFP_KERNEL);
		ms_getInidata("PAD_TABLE", "SENSE", str);
		TEST_DBG(0, "PAD_TABLE(SENSE):      [%s]\n", str);
		nSize = 0;
		nSize = ms_ini_split_u16_array(str, ptMutualMpTest->PAD2Sense);

		TEST_DBG(0, "PAD2Sense\n\n");
		DebugShowArray2(ptMutualMpTest->PAD2Sense, ptMutualMpTest->sensorInfo.numSen, 16, 16,
				ptMutualMpTest->sensorInfo.numSen);
	}

	if (ptMutualMpTest->sensorInfo.numGr) {
		ptMutualMpTest->PAD2GR =
		    kmalloc(ptMutualMpTest->sensorInfo.numGr * sizeof(u16), GFP_KERNEL);
		ms_getInidata("PAD_TABLE", "GR", str);
		TEST_DBG(0, "PAD_TABLE(GR):      [%s]\n", str);
		nSize = 0;
		nSize = ms_ini_split_u16_array(str, ptMutualMpTest->PAD2GR);
	}

	ms_getInidata("RULES", "SHORTVALUE", str);
	ptMutualMpTest->sensorInfo.thrsShort = ms_atoi(str);
	TEST_DBG(0, "SHORTVALUE:      [%d]\n", ptMutualMpTest->sensorInfo.thrsShort);

	ms_getInidata("RULES", "ICPINSHORT", str);
	ptMutualMpTest->sensorInfo.thrsICpin = ms_atoi(str);
	TEST_DBG(0, "ICPINSHORT:      [%d]\n", ptMutualMpTest->sensorInfo.thrsICpin);

	return 0;
}

#ifndef DISABLE_DOUBLE
void calGoldenRange(double *goldenTbl, u16 weight, u16 weight_up, double *maxTbl, double *minTbl, int length)
#else
extern void calGoldenRange(int *goldenTbl, u16 weight, u16 weight_up, int *maxTbl, int *minTbl, int length)
#endif
{
#ifndef DISABLE_DOUBLE
	double value = 0, value_up = 0;
#else
	int value = 0, value_up = 0;
#endif
	int i;

	TEST_DBG(0, " %s\n", __func__);

	for (i = 0; i < length; i++) {
#ifndef DISABLE_DOUBLE
		value = (double)weight * abs(goldenTbl[i]) / 100;
		value_up = (double)weight_up * abs(goldenTbl[i]) / 100;
#else
		value = (int)weight * abs(goldenTbl[i]) / 100;
		value_up = (int)weight_up * abs(goldenTbl[i]) / 100;
#endif
		maxTbl[i] = goldenTbl[i] + value + value_up;
		minTbl[i] = goldenTbl[i] - value;
	}
}

int Msg28xxMPTestResult(int nResultType, int *pResult)
{
	int nResultSize = 0;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	switch (nResultType) {

	case MPTEST_RESULT:
		pResult[OPEN_TEST_RESULT] = ptMutualMpTestResult->nOpenResult;
		pResult[SHORT_TEST_RESULT] = ptMutualMpTestResult->nShortResult;
		pResult[WATERPROOF_TEST_RESULT] = ptMutualMpTestResult->nWaterProofResult;
		nResultSize = 3;
		break;

	case MPTEST_SCOPE:
		pResult[0] = ptMutualMpTest->sensorInfo.numSen;
		pResult[1] = ptMutualMpTest->sensorInfo.numDrv;
		pResult[2] = ptMutualMpTest->sensorInfo.numKey;
		nResultSize = 3;
		break;

	case OPEN_TEST_DATA:
		pResult[0] = ptMutualMpTest->sensorInfo.numDrv * ptMutualMpTest->sensorInfo.numSen;
		memcpy(&pResult[1], ptMutualMpTestResult->pOpenResultData, sizeof(int) * pResult[0]);
		nResultSize = pResult[0] + 1;
		break;

	case OPEN_TEST_FAIL_CHANNEL:
		pResult[0] = MAX_MUTUAL_NUM;
		memcpy(&pResult[1], ptMutualMpTestResult->pOpenFailChannel, sizeof(int) * pResult[0]);
		nResultSize = pResult[0] + 1;
		break;

	case SHORT_TEST_DATA:
		break;

	case SHORT_TEST_FAIL_CHANNEL:
		break;

	case WATERPROOF_TEST_DATA:
		break;

	case WATERPROOF_TEST_FAIL_CHANNEL:
		break;

	default:
		break;
	}

	return nResultSize;
}

int Msg28xxMPTestResultD(int nResultType, double *pResult)
{
	int nResultSize = 0;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	switch (nResultType) {
	case SHORT_TEST_DATA:
		break;

	default:
		break;
	}

	return nResultSize;
}

void Msg28xxEndMPTest(void)
{
	pr_notice("*** %s() ***\n", __func__);

	if (ptMutualMpTest->KeySen != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->KeySen\n***");
		kfree(ptMutualMpTest->KeySen);
		ptMutualMpTest->KeySen = NULL;
	}

	if (ptMutualMpTest->sensorInfo.mapping != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->sensorInfo.mapping\n***");
		kfree(ptMutualMpTest->sensorInfo.mapping);
		ptMutualMpTest->sensorInfo.mapping = NULL;
	}

	if (ptMutualMpTest->ana_version != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->ana_version\n***");
		kfree(ptMutualMpTest->ana_version);
		ptMutualMpTest->ana_version = NULL;
	}

	if (ptMutualMpTest->project_name != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->project_name\n***");
		kfree(ptMutualMpTest->project_name);
		ptMutualMpTest->project_name = NULL;
	}

	if (ptMutualMpTest->Goldensample_CH_0 != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->Goldensample_CH_0\n***");
		kfree(ptMutualMpTest->Goldensample_CH_0);
		ptMutualMpTest->Goldensample_CH_0 = NULL;
	}

	if (ptMutualMpTest->Goldensample_CH_0_Max != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->Goldensample_CH_0_Max\n***");
		kfree(ptMutualMpTest->Goldensample_CH_0_Max);
		ptMutualMpTest->Goldensample_CH_0_Max = NULL;
	}

	if (ptMutualMpTest->Goldensample_CH_0_Max_Avg != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->Goldensample_CH_0_Max_Avg\n***");
		kfree(ptMutualMpTest->Goldensample_CH_0_Max_Avg);
		ptMutualMpTest->Goldensample_CH_0_Max_Avg = NULL;
	}

	if (ptMutualMpTest->Goldensample_CH_0_Min != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->Goldensample_CH_0_Min\n***");
		kfree(ptMutualMpTest->Goldensample_CH_0_Min);
		ptMutualMpTest->Goldensample_CH_0_Min = NULL;
	}

	if (ptMutualMpTest->Goldensample_CH_0_Min_Avg != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->Goldensample_CH_0_Min_Avg\n***");
		kfree(ptMutualMpTest->Goldensample_CH_0_Min_Avg);
		ptMutualMpTest->Goldensample_CH_0_Min_Avg = NULL;
	}

	if (ptMutualMpTest->PAD2Drive != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->PAD2Drive\n***");
		kfree(ptMutualMpTest->PAD2Drive);
		ptMutualMpTest->PAD2Drive = NULL;
	}

	if (ptMutualMpTest->PAD2Sense != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->PAD2Sense\n***");
		kfree(ptMutualMpTest->PAD2Sense);
		ptMutualMpTest->PAD2Sense = NULL;
	}

	if (ptMutualMpTest->PAD2GR != NULL) {
		TEST_DBG(0, "** free ptMutualMpTest->PAD2GR\n***");
		kfree(ptMutualMpTest->PAD2GR);
		ptMutualMpTest->PAD2GR = NULL;
	}

	if (ptMutualMpTestResult->pCheck_Fail != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pCheck_Fail\n***");
		kfree(ptMutualMpTestResult->pCheck_Fail);
		ptMutualMpTestResult->pCheck_Fail = NULL;
	}

	if (ptMutualMpTestResult->pOpenResultData != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pOpenResultData\n***");
		kfree(ptMutualMpTestResult->pOpenResultData);
		ptMutualMpTestResult->pOpenResultData = NULL;
	}

	if (ptMutualMpTestResult->pOpenFailChannel != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pOpenFailChannel\n***");
		kfree(ptMutualMpTestResult->pOpenFailChannel);
		ptMutualMpTestResult->pOpenFailChannel = NULL;
	}

	if (ptMutualMpTestResult->pOpenRatioFailChannel != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pOpenRatioFailChannel\n***");
		kfree(ptMutualMpTestResult->pOpenRatioFailChannel);
		ptMutualMpTestResult->pOpenRatioFailChannel = NULL;
	}

	if (ptMutualMpTestResult->pGolden_CH != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pGolden_CH\n***");
		kfree(ptMutualMpTestResult->pGolden_CH);
		ptMutualMpTestResult->pGolden_CH = NULL;
	}

	if (ptMutualMpTestResult->pGolden_CH_Max != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pGolden_CH_Max\n***");
		kfree(ptMutualMpTestResult->pGolden_CH_Max);
		ptMutualMpTestResult->pGolden_CH_Max = NULL;
	}

	if (ptMutualMpTestResult->pGolden_CH_Max_Avg != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pGolden_CH_Max_Avg\n***");
		kfree(ptMutualMpTestResult->pGolden_CH_Max_Avg);
		ptMutualMpTestResult->pGolden_CH_Max_Avg = NULL;
	}

	if (ptMutualMpTestResult->pGolden_CH_Min != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pGolden_CH_Min\n***");
		kfree(ptMutualMpTestResult->pGolden_CH_Min);
		ptMutualMpTestResult->pGolden_CH_Min = NULL;
	}

	if (ptMutualMpTestResult->pGolden_CH_Min_Avg != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pGolden_CH_Min_Avg\n***");
		kfree(ptMutualMpTestResult->pGolden_CH_Min_Avg);
		ptMutualMpTestResult->pGolden_CH_Min_Avg = NULL;
	}
	if (ptMutualMpTestResult->pICPinChannel != NULL) {

		TEST_DBG(0, "** free ptMutualMpTestResult->pICPinChannel\n***");
		kfree(ptMutualMpTestResult->pICPinChannel);
		ptMutualMpTestResult->pICPinChannel = NULL;
	}

	if (ptMutualMpTestResult->pShortFailChannel != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pShortFailChannel\n***");
		kfree(ptMutualMpTestResult->pShortFailChannel);
		ptMutualMpTestResult->pShortFailChannel = NULL;
	}

	if (ptMutualMpTestResult->pICPinShortFailChannel != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pICPinShortFailChannel\n***");
		kfree(ptMutualMpTestResult->pICPinShortFailChannel);
		ptMutualMpTestResult->pICPinShortFailChannel = NULL;
	}

	if (ptMutualMpTestResult->pShortResultData != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pShortResultData\n***");
		kfree(ptMutualMpTestResult->pShortResultData);
		ptMutualMpTestResult->pShortResultData = NULL;
	}

	if (ptMutualMpTestResult->pShortRData) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pShortRData\n***");
		kfree(ptMutualMpTestResult->pShortRData);
		ptMutualMpTestResult->pShortRData = NULL;
	}

	if (ptMutualMpTestResult->pICPinShortResultData != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pICPinShortResultData\n***");
		kfree(ptMutualMpTestResult->pICPinShortResultData);
		ptMutualMpTestResult->pICPinShortResultData = NULL;
	}

	if (ptMutualMpTestResult->pICPinShortRData != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pICPinShortRData\n***");
		kfree(ptMutualMpTestResult->pICPinShortRData);
		ptMutualMpTestResult->pICPinShortRData = NULL;
	}

	if (ptMutualMpTestResult->pWaterProofResultData != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->pWaterProofResultData\n***");
		kfree(ptMutualMpTestResult->pWaterProofResultData);
		ptMutualMpTestResult->pWaterProofResultData = NULL;
	}

	if (ptMutualMpTestResult->mapTbl_sec != NULL) {
		TEST_DBG(0, "** free ptMutualMpTestResult->mapTbl_sec\n***");
		kfree(ptMutualMpTestResult->mapTbl_sec);
		ptMutualMpTestResult->mapTbl_sec = NULL;
	}

	if (ptMutualMpTest != NULL) {
		kfree(ptMutualMpTest);
		ptMutualMpTest = NULL;
	}

	if (ptMutualMpTestResult != NULL) {
		kfree(ptMutualMpTestResult);
		ptMutualMpTestResult = NULL;
	}
	/*IniFree(); */
	my_parser_exit();
}

u16 getFirmwareOfficialVersionOnFlash(void)
{
	u8 *info_fw_ver;
	int official_ver = 0;
	u8 nChipType = 0;

	TEST_DBG(0, "*** %s() ***\n", __func__);

	info_fw_ver = kcalloc(1, 4, GFP_KERNEL);

	EnterDBBus();
	StopMCU();

	nChipType = RegGet16BitValue(0x1ECC) & 0xFF;
	ReadFlash(nChipType, 0x7f8, EMEM_TYPE_INFO_BLOCK, 4, info_fw_ver);
	TEST_DBG(0, "info_fw_ver[0] = %x, info_fw_ver[1] = %x, info_fw_ver[2] = %x, info_fw_ver[3] = %x\n",
		 info_fw_ver[0], info_fw_ver[1], info_fw_ver[2], info_fw_ver[3]);
	info_fw_ver[1] = 0;

	official_ver = str_to_hex((char *)info_fw_ver);
	kfree(info_fw_ver);
	StartMCU();
	return (u16) official_ver;
}

u16 szMapTable[1000][2] = { {0} };

int Msg28xxInitVariables(void)
{
	int i, nMapSize = 0;
	u16 regData = 0;

	if ((nCount) && (nAna_version[2] > 15)) {
		/*DisableFingerTouch(); */
		DrvPlatformLyrDisableFingerTouchReport();
		/*TouchDeviceResetHw(); */
		DrvPlatformLyrTouchDeviceResetHw();
		EnterDBBus();
		if (!EnterMpMode()) {	/*to get key combine or key separate scan mode */
			/*udelay(100000); */
			mdelay(100);
			StopMCU();
			regData = RegGet16BitValueByAddressMode(0x136E, ADDRESS_MODE_16BIT);

			ptMutualMpTestResult->mapTbl_sec =
			    kmalloc((FILENAME_MAX * sizeof(char)), GFP_KERNEL);

			if (regData == KEY_COMBINE)
				snprintf(ptMutualMpTestResult->mapTbl_sec, FILENAME_MAX * sizeof(char),
				"MAPPING_TABLE_BARKER%d_KEY", RegGetLByteValue(0x1312));
			else if (regData == KEY_SEPARATE)
				snprintf(ptMutualMpTestResult->mapTbl_sec, FILENAME_MAX * sizeof(char),
				"MAPPING_TABLE_BARKER%d", RegGetLByteValue(0x1312));
			else
				snprintf(ptMutualMpTestResult->mapTbl_sec, FILENAME_MAX * sizeof(char),
				"MAPPING_TABLE_BARKER%d", RegGetLByteValue(0x1312));

			TEST_DBG(0, "mapping table name = %s, code length = %d, scan mode = 0x%x\n",
				 ptMutualMpTestResult->mapTbl_sec, RegGetLByteValue(0x1312), regData);

			nMapSize = ms_ini_2d_array(iniFilePath, "MAPPING_TABLE_BARKER", szMapTable);
			/*nMapSize = ms_ini_2d_array(iniFilePath, mapTbl_sec, szMapTable); */

			TEST_DBG(0, "mapping number = %d\n", nMapSize);
			ptMutualMpTest->sensorInfo.mapping =
			    kcalloc(nMapSize, sizeof(MutualMapping_t), GFP_KERNEL);

			if (nMapSize == 0)
				goto out;
			else {
				for (i = 0; i < nMapSize; i++) {
					ptMutualMpTest->sensorInfo.mapping[i].X = szMapTable[i][0];
					ptMutualMpTest->sensorInfo.mapping[i].Y = szMapTable[i][1];
				}
			}
		}
	}

	calGoldenRange(ptMutualMpTest->Goldensample_CH_0,
		       ptMutualMpTest->ToastInfo.persentDC_VA_Range,
		       ptMutualMpTest->ToastInfo.persentDC_VA_Range_up, ptMutualMpTest->Goldensample_CH_0_Max,
		       ptMutualMpTest->Goldensample_CH_0_Min, MAX_MUTUAL_NUM);

	ptMutualMpTestResult->pCheck_Fail = kmalloc(sizeof(int) * TEST_ITEM_NUM, GFP_KERNEL);
	ptMutualMpTestResult->pOpenResultData = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pOpenFailChannel = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pOpenRatioFailChannel = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);

#ifndef DISABLE_DOUBLE

	ptMutualMpTestResult->pGolden_CH = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
	ptMutualMpTestResult->pGolden_CH_Max = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
	ptMutualMpTestResult->pGolden_CH_Max_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
	ptMutualMpTestResult->pGolden_CH_Min = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
	ptMutualMpTestResult->pGolden_CH_Min_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(double), GFP_KERNEL);
	ptMutualMpTestResult->nRatioAvg_max =
	    (double)(100 + ptMutualMpTest->ToastInfo.persentDC_VA_Ratio +
		     ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up) / 100;
	ptMutualMpTestResult->nRatioAvg_min =
	    (double)(100 - ptMutualMpTest->ToastInfo.persentDC_VA_Ratio) / 100;
	ptMutualMpTestResult->nBorder_RatioAvg_max =
	    (double)(100 + ptMutualMpTest->ToastInfo.persentDC_Border_Ratio +
		     ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up) / 100;
	ptMutualMpTestResult->nBorder_RatioAvg_min =
	    (double)(100 - ptMutualMpTest->ToastInfo.persentDC_Border_Ratio) / 100;
#else
	ptMutualMpTestResult->pGolden_CH = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pGolden_CH_Max = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pGolden_CH_Max_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pGolden_CH_Min = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pGolden_CH_Min_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);

	ptMutualMpTestResult->nRatioAvg_max =
	    (int)(100 + ptMutualMpTest->ToastInfo.persentDC_VA_Ratio +
		  ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up) / 100;
	ptMutualMpTestResult->nRatioAvg_min = (int)(100 - ptMutualMpTest->ToastInfo.persentDC_VA_Ratio) / 100;
	ptMutualMpTestResult->nBorder_RatioAvg_max =
	    (int)(100 + ptMutualMpTest->ToastInfo.persentDC_Border_Ratio +
		  ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up) / 100;
	ptMutualMpTestResult->nBorder_RatioAvg_min =
	    (int)(100 - ptMutualMpTest->ToastInfo.persentDC_Border_Ratio) / 100;
#endif

	ptMutualMpTestResult->pShortFailChannel = kcalloc(MAX_CHANNEL_NUM_28XX, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pShortResultData = kcalloc(MAX_CHANNEL_NUM_28XX, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pICPinChannel = kcalloc(MAX_CHANNEL_NUM_28XX, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pICPinShortFailChannel = kcalloc(MAX_CHANNEL_NUM_28XX, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pICPinShortResultData = kcalloc(MAX_CHANNEL_NUM_28XX, sizeof(int), GFP_KERNEL);
#ifndef DISABLE_DOUBLE
	ptMutualMpTestResult->pICPinShortRData = kcalloc(MAX_CHANNEL_NUM_28XX, sizeof(double), GFP_KERNEL);
	ptMutualMpTestResult->pShortRData = kcalloc(MAX_CHANNEL_NUM_28XX, sizeof(double), GFP_KERNEL);
#else
	ptMutualMpTestResult->pICPinShortRData = kcalloc(MAX_CHANNEL_NUM_28XX, sizeof(int), GFP_KERNEL);
	ptMutualMpTestResult->pShortRData = kcalloc(MAX_CHANNEL_NUM_28XX, sizeof(int), GFP_KERNEL);
#endif

	return 0;

out:
	return -EPERM;
}

extern int g_mstar_tptest_result;
int Msg28xxStartMPTest(void)
{
	int res = 0, test_count = 0;
	int i;
	u16 fw_ver = 0xFF;

	pr_notice("*** %s() ***\n", __func__);

	fw_ver = getFirmwareOfficialVersionOnFlash();
	if (fw_ver == 0xFF)
		fw_ver = 0;
	TEST_DBG(0, "info fw_ver = %x\n", fw_ver);

	if (Msg28xxInitVariables() < 0) {
		res = -1;
		goto out;
	}

	if (ptMutualMpTest->UIConfig.bOpenTest == 1) {

		ptMutualMpTestResult->nOpenResult = Msg28xxOpenTest(fw_ver);

		if (ptMutualMpTestResult->nOpenResult == ITO_TEST_FAIL) {
			g_mstar_tptest_result =
			    g_mstar_tptest_result | TEST_BEYOND_MAX_LIMIT | TEST_BEYOND_MIN_LIMIT |
			    TEST_GT_OPEN;
		}

		for (i = 0; i < MAX_MUTUAL_NUM; i++) {
			ptMutualMpTestResult->pGolden_CH[i] = ptMutualMpTest->Goldensample_CH_0[i];
			ptMutualMpTestResult->pGolden_CH_Max[i] = ptMutualMpTest->Goldensample_CH_0_Max[i];
			ptMutualMpTestResult->pGolden_CH_Min[i] = ptMutualMpTest->Goldensample_CH_0_Min[i];
		}
	}

	else {
		ptMutualMpTestResult->nOpenResult = ITO_NO_TEST;
	}

	if (ptMutualMpTest->UIConfig.bShortTest == 1) {

		ptMutualMpTestResult->nShortResult = Msg28xxShortTest();
		if (ptMutualMpTestResult->nShortResult == ITO_TEST_FAIL) {
			g_mstar_tptest_result = g_mstar_tptest_result | TEST_GT_SHORT;
		}
	} else {
		ptMutualMpTestResult->nShortResult = ITO_NO_TEST;
	}

	if (ptMutualMpTest->UIConfig.bWpTest == 1) {
		ptMutualMpTestResult->pWaterProofResultData = kmalloc(sizeof(int) * 8, GFP_KERNEL);
		ptMutualMpTestResult->nWaterProofResult = Msg28xxWaterProofTest();
	} else {
		ptMutualMpTestResult->nWaterProofResult = ITO_NO_TEST;
	}

	/* Retry if one of them is failed once */
	if (ptMutualMpTestResult->nOpenResult != ITO_NO_TEST &&
				ptMutualMpTestResult->nOpenResult == ITO_TEST_FAIL) {
		while (test_count <  ptMutualMpTestResult->nRetry) {
			pr_info(" ****  Retry Open Test : %d  ****\n", test_count+1);

			ptMutualMpTestResult->nOpenResult = Msg28xxOpenTest(fw_ver);
			if (ptMutualMpTestResult->nOpenResult == ITO_TEST_OK)
				break;

			test_count++;
		}
	}

	test_count = 0;

	if (ptMutualMpTestResult->nShortResult != ITO_NO_TEST &&
				ptMutualMpTestResult->nShortResult == ITO_TEST_FAIL) {
		while (test_count <  ptMutualMpTestResult->nRetry) {
			pr_info(" ****  Retry Short Test : %d  ****\n", test_count+1);
			ptMutualMpTestResult->nShortResult = Msg28xxShortTest();
			if (ptMutualMpTestResult->nShortResult == ITO_TEST_OK)
				break;

			test_count++;
		}
	}

	pr_info(" **** Result: Open = %d, Short = %d  ****\n", ptMutualMpTestResult->nOpenResult,
				ptMutualMpTestResult->nShortResult);

out:
	kfree(iniFilePath);

	return 0;
}
