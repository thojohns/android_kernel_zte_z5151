#include "global.h"

extern struct i2c_client *g_I2cClient;
/*0x62  for MSG21XX/MSG21XXA/MSG26XXM/MSG28XX */
extern u8 SLAVE_I2C_ID_DBBUS;
/*u8 SLAVE_I2C_ID_DBBUS = (0xB2>>1); //0x59 // for MSG22XX*/
extern u8 SLAVE_I2C_ID_DWI2C;	/*0x26 */

static void ReadFlashFinale28XX(void)
{
	TEST_DBG(&g_I2cClient->dev, "*** %s ***\n", __func__);

	/*set read done */
	RegSet16BitValue(0x1606, 0x02);

	/*unset info flag */
	RegSetLByteValue(0x1607, 0x00);

	/*clear addr */
	RegSet16BitValue(0x1600, 0x00);
}

static void ReadFlashInit28XX(u16 cayenne_address, int nBlockType)
{
	TEST_DBG(&g_I2cClient->dev, "*** %s ***\n", __func__);

	/*wriu 0x1608 0x20 */
	RegSet16BitValue(0x1608, 0x20);
	/*wriu 0x1606 0x20 */
	RegSet16BitValue(0x1606, 0x20);

	/*wriu read done */
	RegSet16BitValue(0x1606, 0x02);

	/*set address */
	RegSet16BitValue(0x1600, cayenne_address);

	if (nBlockType == EMEM_TYPE_INFO_BLOCK) {
		/*set Info Block */
		RegSetLByteValue((uint) 0x1607, (uint) 0x08);

		/*set Info Double Buffer */
		RegSetLByteValue((uint) 0x1604, (uint) 0x01);
	} else {
		/*set Main Block */
		RegSetLByteValue((uint) 0x1607, (uint) 0x00);

		/*set Main Double Buffer */
		RegSetLByteValue((uint) 0x1604, (uint) 0x01);
	}
	/*set FPGA flag */
	RegSetLByteValue(0x1610, 0x01);

	/*set mode trigger */
	if (nBlockType == EMEM_TYPE_INFO_BLOCK) {
		/*set eflash mode to read mode */
		/*set info flag */
		RegSet16BitValue(0x1606, 0x0801);
	} else {
		/*set eflash mode to read mode */
		RegSet16BitValue(0x1606, 0x0001);
	}
}

static int ReadFlashRIU28XX(u32 nAddr, int nBlockType, int nLength, u8 *pFlashData)
{
	uint read_16_addr_a = 0, read_16_addr_c = 0;

	TEST_DBG(&g_I2cClient->dev, "*** %s() nAddr:0x%x***\n", __func__, nAddr);

	/*set read address */
	RegSet16BitValue(0x1600, nAddr);

	/*read 16+16 bits */
	read_16_addr_a = RegGet16BitValue(0x160a);
	read_16_addr_c = RegGet16BitValue(0x160c);

	/*DEBUG("*** %s() read_16_addr_a:%x read_16_addr_c:%x***\n",__func__,read_16_addr_a,read_16_addr_c); */

	pFlashData[0] = (u8) (read_16_addr_a & 0xff);
	pFlashData[1] = (u8) ((read_16_addr_a >> 8) & 0xff);
	pFlashData[2] = (u8) (read_16_addr_c & 0xff);
	pFlashData[3] = (u8) ((read_16_addr_c >> 8) & 0xff);
	/*DEBUG("*** %s() pFlashData[0]:0x%x pFlashData[1]:0x%x pFlashData[2]:0x%x pFlashData[3]:0x%x***\n", */
    /*__func__,pFlashData[0],pFlashData[1],pFlashData[2],pFlashData[3]);*/
	return 0;
}

static int ReadFlash28XX(u32 nAddr, int nBlockType, int nLength, u8 *pFlashData)
{
	u16 _28xx_addr = nAddr / 4;
	u32 addr_star, addr_end, addr_step;
	u32 read_byte = 0;

	addr_star = nAddr;
	addr_end = nAddr + nLength;

	if ((addr_star >= EMEM_SIZE_MSG28XX) || (addr_end > EMEM_SIZE_MSG28XX)) {
		TEST_DBG(&g_I2cClient->dev, "*** %s : addr_start = 0x%x , addr_end = 0x%x ***\n", __func__,
			 addr_star, addr_end);
		return -EPERM;
	}

	addr_step = 4;

	ReadFlashInit28XX(_28xx_addr, nBlockType);

	for (addr_star = nAddr; addr_star < addr_end; addr_star += addr_step) {
		_28xx_addr = addr_star / 4;

		TEST_DBG(&g_I2cClient->dev,
			 "*** %s() _28xx_addr:0x%x addr_star:0x%x addr_end:%x nLength:%d pFlashData:%p***\n",
			 __func__, _28xx_addr, addr_star, addr_end, nLength, pFlashData);
		ReadFlashRIU28XX(_28xx_addr, nBlockType, nLength, (pFlashData + read_byte));
		TEST_DBG(&g_I2cClient->dev, "*** %s() pFlashData[%x]: %02x %02x %02x %02x read_byte:%d\n",
			 __func__, addr_star, pFlashData[read_byte], pFlashData[read_byte + 1],
			 pFlashData[read_byte + 2], pFlashData[read_byte + 3], read_byte);
		/*pFlashData+=addr_step; */
		read_byte += 4;
	}

	ReadFlashFinale28XX();

	return 0;
}

/*Start mcu*/
void StartMCU(void)
{
	TEST_DBG(&g_I2cClient->dev, "*** %s ***\n", __func__);
	/*Start mcu */
	RegSetLByteValue(0x0FE6, 0x00);	/*bank:mheg5, addr:h0073 */
}

/*Stop mcu*/
void StopMCU(void)
{
	TEST_DBG(&g_I2cClient->dev, "*** %s ***\n", __func__);
	RegSetLByteValue(0x0FE6, 0x01);	/*bank:mheg5, addr:h0073 */
}

void ExitDBBus(void)
{
	TEST_DBG(&g_I2cClient->dev, "*** %s ***\n", __func__);
	DbBusIICNotUseBus();
	DbBusNotStopMCU();
	DbBusExitSerialDebugMode();
}

void EnterDBBus(void)
{
	TEST_DBG(&g_I2cClient->dev, "*** %s ***\n", __func__);
	DbBusEnterSerialDebugMode();
	DbBusStopMCU();
	DbBusIICUseBus();
	DbBusIICReshape();
}

s32 Msg28xxCheckSwitchStatus(void)
{
	u32 nRegData = 0;
	int nTimeOut = 280;
	int nT = 0;

	TEST_DBG(0, "*** %s() ***\n", __func__);
	do {
		nRegData = RegGet16BitValue(0x1402);
		/*udelay(20000); */
		mdelay(20);
		nT++;
		if (nT > nTimeOut) {
			return -EPERM;
		}
		/*DEBUG("nRegData = %x", nRegData); */

	} while (nRegData != 0x7447);

	return 0;
}

s32 EnterMpMode(void)
{
	TEST_DBG(0, "*** %s() ***\n", __func__);
	StopMCU();
	/*udelay(100000); */
	mdelay(100);
	RegSet16BitValue(0X3C60, 0xAA55);	/*disable watch dog */
	RegSet16BitValue(0X3D08, 0xFFFF);	/*clear interrupt status */
	RegSet16BitValue(0X3D18, 0xFFFF);	/*clear interrupt status */

	RegSet16BitValue(0x1402, 0x7474);	/*enter mp mode */

	RegSet16BitValue(0x1E06, 0x0000);
	RegSet16BitValue(0x1E06, 0x0001);
	StartMCU();
	/*udelay(300000); */
	mdelay(300);

	if (Msg28xxCheckSwitchStatus() < 0) {
		pr_err("*** Msg28xx MP Test# CheckSwitchStatus failed! ***\n");
		return -EPERM;
	}
	return 0;
}

s32 Msg28xxSwitchFwMode(u16 *nFMode, u16 *deep_standby)
{
	u8 nFreq = 0;
	u8 cmd[3] = { 0x0B, 0x01, nFreq };

	TEST_DBG(0, "*** %s() ***\n", __func__);

	/*RegSet16BitValue(0x0FE6, 0x0001);    // MCU_stop */
	StopMCU();
	/*usleep(100000); */
	mdelay(100);
	RegSet16BitValue(0X3C60, 0xAA55);	/*disable watch dog */

	RegSet16BitValue(0X3D08, 0xFFFF);	/*clear interrupt status */
	RegSet16BitValue(0X3D18, 0xFFFF);	/*clear interrupt status */

	RegSet16BitValue(0x1402, 0x7474);	/*enter mp mode */

	RegSet16BitValue(0x1E06, 0x0000);
	RegSet16BitValue(0x1E06, 0x0001);
	/*RegSet16BitValue(0x0FE6, 0x0000);   // MCU start */
	StartMCU();
	/*usleep(300000); */
	mdelay(300);

	if (Msg28xxCheckSwitchStatus() < 0) {
		pr_err("*** Msg28xx MP Test# CheckSwitchStatus failed! ***\n");
		return -EPERM;
	}

	/*deep standby mode */
	if (*deep_standby == 1) {
		TEST_DBG(0, "*** Msg28xx MP Test# enter deep standby! ***\n");
		RegSet16BitValue(0x1402, 0x6179);
		/*usleep(ptMutualMpTest->deep_standby_timeout * 1000); // depend on ini, default = 450ms */
		mdelay(ptMutualMpTest->deep_standby_timeout);

		EnterDBBus();

		if (Msg28xxCheckSwitchStatus() < 0) {
			*deep_standby = 0;
			pr_err("*** Msg28xx MP Test# enter deep standby FAILED! ***\n");
			return -EPERM;
		}
	}
	TEST_DBG(0, "nFMode = %x\n", *nFMode);
	nFreq = (u8) ptMutualMpTest->Open_fixed_carrier;
	if (*nFMode == MUTUAL_SINE) {
		nFreq = MAX(nFreq, 30);
		nFreq = MIN(nFreq, 140);
		cmd[2] = nFreq;

		IicWriteData(SLAVE_I2C_ID_DWI2C, &cmd[0], sizeof(cmd) / sizeof(u8));
	}
	switch (*nFMode) {
	case MUTUAL_MODE:
		RegSet16BitValue(0x1402, 0x5705);
		break;

	case MUTUAL_SINE:
		RegSet16BitValue(0x1402, 0x5706);
		break;

	case SELF:
		RegSet16BitValue(0x1402, 0x6278);
		break;

	case WATERPROOF:
		RegSet16BitValue(0x1402, 0x7992);
		break;

	case MUTUAL_SINGLE_DRIVE:
		RegSet16BitValue(0x1402, 0x0158);
		break;

	case SINE_PHASE:
		RegSet16BitValue(0x1402, 0xECFA);
		break;

	default:
		return -EPERM;
	}
	if (Msg28xxCheckSwitchStatus() < 0) {
		pr_err("*** Msg28xx MP Test# CheckSwitchStatus failed! ***\n");
		return -EPERM;
	}

	if (*nFMode == MUTUAL_SINE) {
		u16 regData = RegGet16BitValueByAddressMode(0x2003, ADDRESS_MODE_16BIT);
		int freq = (int)(regData * (13000000 / 16384) / 1000);	/*khz */

		if (abs(freq - ptMutualMpTest->Open_fixed_carrier) >= 2) {
			pr_err
			    ("Fixed carrier failed, current frequency = %d khz, need fixed frequency = %d khz",
			     freq, ptMutualMpTest->Open_fixed_carrier);
			return -EPERM;
		}
	}

	/*RegSet16BitValue(0x0FE6, 0x0001);// stop mcu */
	StopMCU();
	RegSet16BitValue(0x3D08, 0xFEFF);	/*open timer */

	return 0;
}

int ReadFlash(u8 nChipType, u32 nAddr, int nBlockType, int nLength, u8 *pFlashData)
{
	int ret = 0;

	TEST_DBG(&g_I2cClient->dev, "*** %s()  nChipType 0x%x***\n", __func__, nChipType);

	switch (nChipType) {

	case CHIP_TYPE_MSG28XX:
	case CHIP_TYPE_MSG28XXA:
		ret = ReadFlash28XX(nAddr, nBlockType, nLength, pFlashData);
		break;

	default:
		break;
	}

	return ret;
}
