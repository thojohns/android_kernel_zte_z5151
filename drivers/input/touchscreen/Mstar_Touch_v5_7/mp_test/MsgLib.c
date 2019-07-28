#include "global.h"

extern struct i2c_client *g_I2cClient;
/*internal use firmware version for MStar */
extern u8 *_gPlatformFwVersion;
extern u8 *_gFwVersion;		/*customer firmware version */
extern struct mutex g_Mutex;
u32 g_IsInMpTest = 0;

struct _TP_INFO tpinfo;

int getFirmwareVersionOnFlash(void)
{
	uint8_t nChipType = 0;
/*uint8_t *pVer;*/
	/*uint8_t *unknown_version="unknown"; */
	uint8_t *main_fw_ver = kmalloc(4, GFP_KERNEL);
	uint8_t *info_fw_ver = kmalloc(4, GFP_KERNEL);
	uint8_t *ret = kmalloc(21, GFP_KERNEL);	/*0001.0001 / 0001.0001 */
	uint16_t main_major, main_minor;
	uint16_t info_major, info_minor;
#ifdef BUILD_FUNCTION_APK
	jbyteArray result;
#else
	uint8_t result = 0;
#endif

	memset(main_fw_ver, 0, 4);
	memset(info_fw_ver, 0, 4);
	memset(ret, 0, 21);

	TEST_DBG(&g_I2cClient->dev, "*** %s ***\n", __func__);

	EnterDBBus();
	StopMCU();

	nChipType = RegGet16BitValue(0x1ECC) & 0xFF;

	if (nChipType != CHIP_TYPE_MSG28XX &&	/*(0x85) */
	    nChipType != CHIP_TYPE_MSG28XXA) {	/*(0xBF) */
		nChipType = 0;
	}

	switch (nChipType) {

	case CHIP_TYPE_MSG28XX:
	case CHIP_TYPE_MSG28XXA:
		ReadFlash(nChipType, 0x1fff4, EMEM_TYPE_MAIN_BLOCK, 4, main_fw_ver);

		/*DEBUG("*** %s() main_fw_ver[0]:0x%x main_fw_ver[1]:
		0x%x main_fw_ver[2]:0x%x main_fw_ver[3]:0x%x***\n", */
		    /*__func__,main_fw_ver[0],main_fw_ver[1],main_fw_ver[2],main_fw_ver[3]);*/

		ReadFlash(nChipType, 0x07ec, EMEM_TYPE_INFO_BLOCK, 4, info_fw_ver);
		/*DEBUG("*** %s() info_fw_ver[0]:0x%x info_fw_ver[1]:0x%x info_fw_ver[2]:
		0x%x info_fw_ver[3]:0x%x***\n", */
		    /*__func__,info_fw_ver[0],info_fw_ver[1],info_fw_ver[2],info_fw_ver[3]);*/

		/*ret[0]='5'; */
		/*ret[1]='6'; */
		break;

	default:
		ret[0] = '5';
		ret[1] = 'A';
		ret[2] = 'A';
		ret[3] = '5';
		break;
	}

	StartMCU();
	ExitDBBus();

	main_major = (main_fw_ver[0] | main_fw_ver[1] << 8);
	main_minor = (main_fw_ver[2] | main_fw_ver[3] << 8);

	info_major = (info_fw_ver[0] | info_fw_ver[1] << 8);
	info_minor = (info_fw_ver[2] | info_fw_ver[3] << 8);

	TEST_DBG(0, "%d.%03d/%d.%03d", main_major, main_minor, info_major, info_minor);

#ifdef BUILD_FUNCTION_APK
	result = (*env)->NewByteArray(env, 21);
	(*env)->SetByteArrayRegion(env, result, 0, 21, ret);
#else
	tpinfo.ChipType = nChipType;
	snprintf(tpinfo.MainBlockFWVersion, sizeof(tpinfo.MainBlockFWVersion), "%d.%03d", main_major, main_minor);
	snprintf(tpinfo.InfoBlockFWVersion, sizeof(tpinfo.InfoBlockFWVersion), "%d.%03d", info_major, info_minor);
	snprintf(tpinfo.PlatformVersion, sizeof(tpinfo.PlatformVersion), "%s", _gPlatformFwVersion);
	snprintf(tpinfo.FwVersion, sizeof(tpinfo.FwVersion), "%s", _gFwVersion);

#endif

	kfree(main_fw_ver);
	kfree(info_fw_ver);
	kfree(ret);
return result;
}
int startMPTest(int nChipType, char *pFilePath)
{
	/*char *pFile; */
	/*size_t str_length = strlen(pFilePath); */
	/*pFile = kmalloc(str_length, GFP_KERNEL); */
	/*memcpy((char*)pFile, (char *)pFilePath, str_length); */

	TEST_DBG(&g_I2cClient->dev, "*** %s : Start running MP Test ***\n", __func__);
	TEST_DBG(&g_I2cClient->dev, "*** %s : iniPath = %s ***\n", __func__, pFilePath);

	if (nChipType == CHIP_TYPE_MSG28XX) {
		if (Msg28xxLoadIni(pFilePath) < 0) {
		TEST_DBG(&g_I2cClient->dev, "*** %s : MSG28xx failed to load ini ***\n", __func__);
		/*kfree(pFile); */
		return -EPERM;
		}
		/*SaveChipType(nChipType); */
		mutex_lock(&g_Mutex);
		g_IsInMpTest = 1;
		mutex_unlock(&g_Mutex);
		if (Msg28xxStartMPTest() < 0) {
			TEST_DBG(&g_I2cClient->dev, "*** %s : MSG28xx MP Test failed ***\n", __func__);
			mutex_lock(&g_Mutex);
			g_IsInMpTest = 0;
			mutex_unlock(&g_Mutex);
			return -EPERM;
		}
		mutex_lock(&g_Mutex);
		g_IsInMpTest = 0;
		mutex_unlock(&g_Mutex);
	}
	/*kfree(pFile); */
	return 0;
}
