#include "global.h"
/*0x62 / for MSG21XX/MSG21XXA/MSG26XXM/MSG28XX */
extern u8 SLAVE_I2C_ID_DBBUS;
/*u8 SLAVE_I2C_ID_DBBUS = (0xB2>>1); /0x59 / for MSG22XX*/
extern u8 SLAVE_I2C_ID_DWI2C;	/*0x26 */

void RegGet16BitByteValueBuf(u16 nAddr, u8 *pBuf, u16 nLen)
{
	u16 i;
	u8 tx_data[3] = { 0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF };
	u8 *rx_data = kcalloc(nLen, sizeof(u8), GFP_KERNEL);

	IicWriteData(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
	IicReadData(SLAVE_I2C_ID_DBBUS, rx_data, nLen);

	for (i = 0; i < nLen; i++)
		pBuf[i] = rx_data[i];
	kfree(rx_data);
}

char szStrBuf[1024];
char szStrTmp[10];

void DebugShowArray2(void *pBuf, u16 nLen, int nDataType, int nCarry, int nChangeLine)
{
	u8 *pU8Buf = NULL;
	s8 *pS8Buf = NULL;
	u16 *pU16Buf = NULL;
	s16 *pS16Buf = NULL;
	u32 *pU32Buf = NULL;
	s32 *pS32Buf = NULL;

	int i;

	TEST_DBG(0, " %s\n", __func__);

	if (nDataType == 8)
		pU8Buf = (u8 *) pBuf;
	else if (nDataType == -8)
		pS8Buf = (s8 *) pBuf;
	else if (nDataType == 16)
		pU16Buf = (u16 *) pBuf;
	else if (nDataType == -16)
		pS16Buf = (s16 *) pBuf;
	else if (nDataType == 32)
		pU32Buf = (u32 *) pBuf;
	else if (nDataType == -32)
		pS32Buf = (s32 *) pBuf;

	for (i = 0; i < nLen; i++) {
		if (nCarry == 16) {
			if (nDataType == 8)
				snprintf(szStrTmp, sizeof(szStrTmp), "%02X ", pU8Buf[i]);
			else if (nDataType == -8)
				snprintf(szStrTmp, sizeof(szStrTmp), "%02X ", pS8Buf[i]);
			else if (nDataType == 16)
				snprintf(szStrTmp, sizeof(szStrTmp), "%04X ", pU16Buf[i]);
			else if (nDataType == -16)
				snprintf(szStrTmp, sizeof(szStrTmp), "%04X ", pS16Buf[i]);
			else if (nDataType == 32)
				snprintf(szStrTmp, sizeof(szStrTmp), "%08X ", pU32Buf[i]);
			else if (nDataType == -32)
				snprintf(szStrTmp, sizeof(szStrTmp), "%08X ", pS32Buf[i]);
		} else if (nCarry == 10) {
			if (nDataType == 8)
				snprintf(szStrTmp, sizeof(szStrTmp), "%6u ", pU8Buf[i]);
			else if (nDataType == -8)
				snprintf(szStrTmp, sizeof(szStrTmp), "%6d ", pS8Buf[i]);
			else if (nDataType == 16)
				snprintf(szStrTmp, sizeof(szStrTmp), "%6u ", pU16Buf[i]);
			else if (nDataType == -16)
				snprintf(szStrTmp, sizeof(szStrTmp), "%6d ", pS16Buf[i]);
			else if (nDataType == 32)
				snprintf(szStrTmp, sizeof(szStrTmp), "%6u ", pU32Buf[i]);
			else if (nDataType == -32)
				snprintf(szStrTmp, sizeof(szStrTmp), "%6d ", pS32Buf[i]);
		}

		strlcat(szStrBuf, szStrTmp, sizeof(szStrBuf));
		memset(szStrTmp, 0, 10);
		if (i % nChangeLine == nChangeLine - 1) {
			TEST_DBG(0, "%s\n", szStrBuf);
			memset(szStrBuf, 0, 1024);
		}
	}
	TEST_DBG(0, "\n");
}
