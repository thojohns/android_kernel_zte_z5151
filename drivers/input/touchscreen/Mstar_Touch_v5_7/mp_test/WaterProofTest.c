#include "global.h"

s32 Msg28xxgetDeltaC_WP(s32 *pDeltaC, int *nSize)
{
	int i = 0;
	s16 pRawData[8] = { 0 };

	pr_notice("*** %s() ***\n", __func__);

	if (Msg28xxtGetWaterProofOneShotRawIIR(pRawData, nSize) == -1)
		return 0;

	for (i = 0; i < 8; i++)
		pDeltaC[i] = (s32) pRawData[i];

	return 1;
}

s32 Msg28xxWaterProof(s32 *pDeltaC, int *nSize)
{
	Msg28xxAnaSwReset();

	pr_notice("*** %s() ***\n", __func__);

	if (!Msg28xxgetDeltaC_WP(pDeltaC, nSize))
		return 0;

	return 1;
}

s32 Msg28xxWaterProof_test_judge(s32 *pDeltaC_water, int *nSize)
{
	int i, nRet = 1;

	pr_notice("*** %s() ***\n", __func__);

	for (i = 0; i < *nSize; i++) {
		if (abs(pDeltaC_water[i]) > ptMutualMpTest->sensorInfo.thrsWater)
			nRet = 0;

		ptMutualMpTestResult->pWaterProofResultData = (int *)abs(pDeltaC_water[i]);
	}
	return nRet;
}

s32 Msg28xxWaterProof_test(void)
{
	s32 nRetVal = 0;
	u8 time = 0;
	u16 deep_standby = ptMutualMpTest->deep_standby;
	u32 wp_support = 0;
	/*s32 thrsWP = ptMutualMpTest->sensorInfo.thrsWater; */
	u16 fmode;

	int deltaC_water_result[8] = { 0 }, nSize = 0;

	pr_notice("*** %s() ***\n", __func__);
	/*DisableFingerTouch() */
	DrvPlatformLyrDisableFingerTouchReport();

_retry_waterproof:

	/*TouchDeviceResetHw(); */
	DrvPlatformLyrTouchDeviceResetHw();

	EnterDBBus();
	/*usleep(100000); */
	mdelay(100);

	/*Start mcu */
	StartMCU();

	fmode = WATERPROOF;
	if (Msg28xxSwitchFwMode(&fmode, &deep_standby) < 0) {
		pr_err("*** Msg28xx Water Proof# SwitchFwMode failed! ***\n");
		nRetVal = -1;
		time++;
		if (time < 10)
			goto _retry_waterproof;
		else
			goto ITO_TEST_END;
	}

	StopMCU();

	wp_support = RegGet16BitValue(0x1402);
	/*if polling 0X8BBD, means the FW is NOT supporting WP. */
	if (wp_support == 0x00008bbd) {
		pr_err("Water Proof Test Fail : No supporting the function.");
		nRetVal = -1;
		goto ITO_TEST_END;
	}

	if (!Msg28xxWaterProof(deltaC_water_result, &nSize)) {
		pr_err("Water Proof Test Fail : (Get Data)");
		nRetVal = -1;
		goto ITO_TEST_END;
	}

	if (!Msg28xxWaterProof_test_judge(deltaC_water_result, &nSize))
		nRetVal = -1;

ITO_TEST_END:

	DrvPlatformLyrTouchDeviceResetHw();
	/*TouchDeviceResetHw(); */
	/*usleep(300000); */
	mdelay(300);
	/*EnableFingerTouch(); */
	DrvPlatformLyrEnableFingerTouchReport();

	return nRetVal;

}

s32 Msg28xxWaterProofTestEntry(void)
{
	u32 nRetVal = 0;

	pr_notice("*** %s() ***\n", __func__);

	nRetVal = Msg28xxWaterProof_test();

	if (nRetVal == -1)
		return -EPERM;
	else
		return 0;
}

int Msg28xxWaterProofTest(void)
{
	int nRetVal = 0;
	int nRet = 0;

	pr_notice("*** %s() ***\n", __func__);

	nRetVal = Msg28xxWaterProofTestEntry();
	if (nRetVal == 0) {
		nRet = ITO_TEST_OK;
		/*PASS*/ pr_notice("Msg28xx Water Proof Test# MP test success\n");
	} else {
		if (nRetVal == -1) {
			pr_err("Msg28xx Water Proof Test# MP test fail\n");
			nRet = ITO_TEST_FAIL;
		} else if (nRetVal == -2) {
			nRet = ITO_TEST_GET_TP_TYPE_ERROR;
		} else {
			nRet = ITO_TEST_UNDEFINED_ERROR;
		}
		pr_err("Msg28xx Water Proof # MP test failed\n");
	}

	return nRet;
}
