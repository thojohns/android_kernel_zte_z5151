#include "global.h"
#include "IniParser/convert_file_op.h"

int ini_items = 0;
char *_gData = NULL;
char M_CFG_SSL = '[';
char M_CFG_SSR = ']';
char M_CFG_NIS = ':';
char M_CFG_NTS = '#';
char M_CFG_EQS = '=';

ST_INI_FILE_DATA ms_ini_file_data[PARSER_MAX_KEY_NUM];

int isdigit_t(int x)
{
	if (x <= '9' && x >= '0')
		return 1;
	else
		return 0;
}

int isspace_t(int x)
{
	if (x == ' ' || x == '\t' || x == '\n' || x == '\f' || x == '\b' || x == '\r')
		return 1;
	else
		return 0;
}

long atol_t(char *nptr)
{
	int c;			/* current char */
	long total;		/* current total */
	int sign;		/* if ''-'', then negative, otherwise positive */
	/* skip whitespace */
	while (isspace_t((int)(unsigned char)*nptr))
		++nptr;
	c = (int)(unsigned char)*nptr++;
	sign = c;		/* save sign indication */
	if (c == '-' || c == '+')
		c = (int)(unsigned char)*nptr++;	/* skip sign */
	total = 0;
	while (isdigit_t(c)) {
		total = 10 * total + (c - '0');	/* accumulate digit */
		c = (int)(unsigned char)*nptr++;	/* get next char */
	}
	if (sign == '-')
		return -total;
	else
		return total;	/* return result, negated if necessary */
}

int ms_atoi(char *nptr)
{
	return (int)atol_t(nptr);
}

int ms_ini_2d_array(const char *pFile, char *pSection, u16 pArray[][2])
{
	struct file *f;
	char *s;
	char szLine[100], szSection[100];
	char *pToken;
	int nInSection = 0;
	int nCount = 0;
	int nKeyNum = 0;
	char EOF[10];

	strlcpy(EOF, "&^*EOF", sizeof(EOF));

	f = file_open(pFile, O_RDONLY, 0, NULL);
	if (f == NULL) {
		pr_err("Cannot open file: %s\n", pFile);
		return -EPERM;
	}

	snprintf(szSection, sizeof(szSection), "%s", pSection);
	while (kgets(szLine, 100, f) != NULL) {
		if (strncmp(szLine, EOF, 5) == 0)
			break;

		if (strnstr(szLine, szSection, sizeof(szSection)) != NULL) {
			nInSection = 1;
			continue;
		}

		if (nInSection == 1) {
			if (szLine != NULL) {
				if (strnstr(szLine, "[", sizeof(szLine)) != NULL) {
					pr_notice("szLine = %s\n", szLine);
					break;
				}
				s = kstrdup(szLine, GFP_KERNEL);
				while ((pToken = strsep(&s, "=,")) != NULL) {
					if (nCount == 1) {
						pArray[nKeyNum][0] = katoi(pToken);
					} else if (nCount == 2) {
						pArray[nKeyNum][1] = katoi(pToken);
					}
					nCount++;
				}
				TEST_DBG(0, "%s: %d,%d\n", pSection, pArray[nKeyNum][0], pArray[nKeyNum][1]);
				s = NULL;
				nCount = 0;
				nKeyNum++;
			} else {
				nInSection = 0;
				/*pr_notice("nKeyNum : %d", nKeyNum); */
				break;
			}
		}
	}

	/*pr_notice("%s: line num = %d\n",__func__,line_num); */
	file_close(f);
	return nKeyNum;
}

int ms_ini_split_u8_array(char *key, u8 *pBuf)
{
	char *s = key;
	char *pToken;
	int nCount = 0;
	int res;
	long s_to_long = 0;

	/*s = kmalloc(512, GFP_KERNEL); */
	/*memset(s, 0, sizeof(*s)); */

	if (isspace_t((int)(unsigned char)*s) == 0) {
		while ((pToken = strsep(&s, ".")) != NULL) {
			res = kstrtol(pToken, 0, &s_to_long);
			if (res == 0)
				pBuf[nCount] = s_to_long;
			else
				pr_err("%s: convert string to long error %d\n", __func__, res);
			nCount++;
		}
	}
	/*kfree(s); */
	return nCount;
}

int ms_ini_split_u16_array(char *key, u16 *pBuf)
{
	char *s = key;
	char *pToken;
	int nCount = 0;
	int res;
	long s_to_long = 0;

	/*s = kmalloc(512, GFP_KERNEL); */
	/*memset(s, 0, sizeof(*s)); */
	/*pr_notice("%s: s = %p, key = %p\n",__func__,s,key); */
	/*pr_notice("%s: s = %s\n",__func__,s); */
	if (isspace_t((int)(unsigned char)*s) == 0) {
		while ((pToken = strsep(&s, ",")) != NULL) {
			/*pr_notice("%s: pToken = %s\n",__func__,pToken); */
			res = kstrtol(pToken, 0, &s_to_long);
			if (res == 0)
				pBuf[nCount] = s_to_long;
			else
				pr_notice("%s: convert string to long error %d\n", __func__, res);
			nCount++;
		}
	}
	/*kfree(s); */
	return nCount;
}

int ms_ini_split_golden(int *pBuf)
{
	char *pToken;
	int nCount = 0;
	int res;
	int num_count = 0;
	long s_to_long = 0;
	char szSection[100] = { 0 };
	char str[100] = { 0 };
	char *s = NULL;

	/*pr_notice("*** %s ****\n",__func__); */

	while (num_count < 24) {
		snprintf(szSection, sizeof(szSection), "Golden_CH_%d", num_count);
		TEST_DBG(0, "%s: szSection = %s\n", __func__, szSection);
		ms_getInidata("RULES", szSection, str);
		/*pr_notice("%s: str = %s\n",__func__,str); */
		s = str;
		while ((pToken = strsep(&s, ",")) != NULL) {
			/*pr_notice("%s: pToken = %s\n",__func__,pToken); */
			res = kstrtol(pToken, 0, &s_to_long);
			if (res == 0)
				pBuf[nCount] = s_to_long;
			else
				pr_err("%s: convert string to long error %d\n", __func__, res);
			nCount++;
		}
		num_count++;
	}

	return nCount;
}

int ms_ini_split_int_array(char *key, int *pBuf)
{
	char *s = key;
	char *pToken;
	int nCount = 0;
	int res;
	long s_to_long = 0;

	/*s = kmalloc(512, GFP_KERNEL); */
	/*memset(s, 0, sizeof(*s)); */

	/*pr_notice("%s: s = %p, key = %p\n",__func__,s,key); */
	/*pr_notice("%s: s = %s\n",__func__,s); */
	if (isspace_t((int)(unsigned char)*s) == 0) {
		while ((pToken = strsep(&s, ",")) != NULL) {
			/*pr_notice("%s: pToken = %s\n",__func__,pToken); */
			res = kstrtol(pToken, 0, &s_to_long);
			if (res == 0)
				pBuf[nCount] = s_to_long;
			else
				pr_err("%s: convert string to long error %d\n", __func__, res);
			nCount++;
		}
	}
	/*kfree(s); */
	return nCount;
}

char *ms_ini_str_trim_r(char *buf)
{
	int len, i;
	char tmp[512];

	memset(tmp, 0, sizeof(tmp));
	len = strlen(buf);
/*tmp = (char *)malloc(len);*/

	memset(tmp, 0x00, len);
	for (i = 0; i < len; i++) {
		if (buf[i] != ' ')
			break;
	}
	if (i < len) {
		strlcpy(tmp, (buf + i), (len - i + 1));
	}
	strlcpy(buf, tmp, (len + 1));
/*free(tmp);*/
	return buf;
}

char *ms_ini_str_trim_l(char *buf)
{
	int len, i;
	char tmp[512];

	memset(tmp, 0, sizeof(tmp));
	len = strlen(buf);
	/*tmp = (char *)malloc(len); */

	memset(tmp, 0x00, len);

	for (i = 0; i < len; i++) {
		if (buf[len - i - 1] != ' ')
			break;
	}
	if (i < len) {
		strlcpy(tmp, buf, len - i + 1);
	}
	strlcpy(buf, tmp, (len + 1));
	/*free(tmp); */
	return buf;
}

int ms_ini_get_key(char *section, char *key, char *value)
{
	int i = 0;
	int ret = -2;
	int len = 0;

	/*pr_notice("%s: section = %s , key = %s, ini_items = %d\n", __func__, section, key, ini_items); */

	len = strlen(key);
	/*pr_notice("%s: ***** len = %d *****\n",__func__,len); */

	for (i = 0; i < ini_items; i++) {
		if (strncmp(section, ms_ini_file_data[i].pSectionName,
			    ms_ini_file_data[i].iSectionNameLen) != 0)
			continue;

		/*pr_notice("%s: Section Name:%s, Len:%d\n\n", __func__,
		ms_ini_file_data[i].pSectionName, ms_ini_file_data[i].iSectionNameLen); */

		/*pr_notice("%s: key:%s , pKeyName: %s iKeyNameLen =%d\n\n",
		__func__,key, ms_ini_file_data[i].pKeyName, ms_ini_file_data[i].iKeyNameLen); */

		/*if(strncmp(key, ms_ini_file_data[i].pKeyName, */
		/*ms_ini_file_data[i].iKeyNameLen) == 0) */
		if (strncmp(key, ms_ini_file_data[i].pKeyName, len) == 0) {
			memcpy(value, ms_ini_file_data[i].pKeyValue, ms_ini_file_data[i].iKeyValueLen);
			/*pr_notice("%s: value:%s , pKeyValue: %s\n",__func__,value, ms_ini_file_data[i].pKeyValue); */
			ret = 0;
			break;
		}
	}

	return ret;
}

int ms_getInidata(char *section, char *ItemName, char *returnValue)
{
	char value[512] = { 0 };
	int len = 0;

	if (returnValue == NULL) {
		pr_err(" returnValue as NULL in function %s.\n", __func__);
		return 0;
	}

	/*pr_notice("%s: ItemName = %s\n",__func__,ItemName); */

	if (ms_ini_get_key(section, ItemName, value) < 0) {
		snprintf(returnValue, 100, "%s", value);
		goto out;
	} else {
		len = snprintf(returnValue, 100, "%s", value);
	}

	/*pr_notice("%s: return = %s\n", __func__, returnValue); */
	/*pr_notice("%s: len = %d\n\n", __func__, len); */

	return len;

out:
	return 0;
}

int ms_ini_file_get_line(char *filedata, char *buffer, int maxlen)
{
	int i = 0;
	int j = 0;
	int iRetNum = -1;
	char ch1 = '\0';

	for (i = 0, j = 0; i < maxlen; j++) {
		ch1 = filedata[j];
		iRetNum = j + 1;
		if (ch1 == '\n' || ch1 == '\r') {	/*line end */
			ch1 = filedata[j + 1];
			if (ch1 == '\n' || ch1 == '\r') {
				iRetNum++;
			}

			goto out;
		} else if (ch1 == 0x00) {
			iRetNum = -1;
			goto out;
		} else {
			buffer[i++] = ch1;
		}
	}
out:
	buffer[i] = '\0';

	return iRetNum;
}

int ms_init_key_data(void)
{
	int i = 0;

	TEST_DBG(0, "%s\n", __func__);

	ini_items = 0;

	for (i = 0; i < PARSER_MAX_KEY_NUM; i++) {
		memset(ms_ini_file_data[i].pSectionName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ms_ini_file_data[i].pKeyName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ms_ini_file_data[i].pKeyValue, 0, PARSER_MAX_KEY_VALUE_LEN);
		ms_ini_file_data[i].iSectionNameLen = 0;
		ms_ini_file_data[i].iKeyNameLen = 0;
		ms_ini_file_data[i].iKeyValueLen = 0;
	}

	return 1;
}

char ini_buf[PARSER_MAX_CFG_BUF + 1] = {0};
char tmpSectionName[PARSER_MAX_CFG_BUF + 1] = {0};

void ms_ini_get_key_data(char *filedata)
{
	int n = 0;
	/*int ret=0; */
	int dataoff = 0;
	int iEqualSign = 0;
	int i = 0;

	ms_init_key_data();

	ini_items = 0;
	memset(ini_buf, 0, sizeof(ini_buf));
	memset(tmpSectionName, 0, sizeof(tmpSectionName));
	while (1) {
		if (ini_items > PARSER_MAX_KEY_NUM) {
			/*ret = CFG_ERR_TOO_MANY_KEY_NUM; */
			pr_notice("MAX_KEY_NUM: Out Of Length\n\n");
			return;
		}

		n = ms_ini_file_get_line(filedata + dataoff, ini_buf, PARSER_MAX_CFG_BUF);

		if (n < -1) {
			pr_err("%s: n error = %d\n", __func__, n);
			return;
		}
		if (n < 0) {
			pr_err("%s: n error = %d\n", __func__, n);
			break;
		}

		dataoff += n;

		n = strlen(ms_ini_str_trim_l(ms_ini_str_trim_r(ini_buf)));
		if (n == 0 || ini_buf[0] == M_CFG_NTS)
			continue;

		/*get section name */
		if (n > 2 && ((ini_buf[0] == M_CFG_SSL && ini_buf[n - 1] != M_CFG_SSR))) {
			pr_err("Bad Section:%s\n\n", ini_buf);
			return;
		}

		if (ini_buf[0] == M_CFG_SSL) {
			ms_ini_file_data[ini_items].iSectionNameLen = n - 2;
			if (ms_ini_file_data[ini_items].iSectionNameLen > PARSER_MAX_KEY_NAME_LEN) {
				/*ret = CFG_ERR_OUT_OF_LEN; */
				pr_notice("MAX_KEY_NAME_LEN: Out Of Length\n\n");
				/*goto cfg_scts_end; */
				return;
			}

			ini_buf[n - 1] = 0x00;
			strlcpy((char *)tmpSectionName, ini_buf + 1, sizeof(tmpSectionName));
			pr_notice("Section Name:%s, Len:%d\n\n", tmpSectionName, n - 2);
			continue;
		}
		/*get section name end */

		strlcpy(ms_ini_file_data[ini_items].pSectionName, tmpSectionName,
			sizeof(ms_ini_file_data[ini_items].pSectionName));
		ms_ini_file_data[ini_items].iSectionNameLen = strlen(tmpSectionName);

		iEqualSign = 0;
		for (i = 0; i < n; i++) {
			if (ini_buf[i] == M_CFG_EQS) {
				iEqualSign = i;
				break;
			}
		}

		if (iEqualSign == 0)
			continue;

		/*pr_notice("%s: ini items = %d\n", __func__,ini_items); */
		ms_ini_file_data[ini_items].iKeyNameLen = iEqualSign;
		/*pr_notice("%s: Key Name Len = %d\n", __func__, ms_ini_file_data[ini_items].iKeyNameLen); */

		if (ms_ini_file_data[ini_items].iKeyNameLen > PARSER_MAX_KEY_NAME_LEN) {
			/*ret = CFG_ERR_OUT_OF_LEN; */
			pr_err("MAX_KEY_NAME_LEN: Out Of Length\n\n");
			return;
		}

		memcpy(ms_ini_file_data[ini_items].pKeyName,
		       ini_buf, ms_ini_file_data[ini_items].iKeyNameLen);

		/*pr_notice("%s: Key Name = %s\n", __func__, ms_ini_file_data[ini_items].pKeyName); */

		ms_ini_file_data[ini_items].iKeyValueLen = n - iEqualSign - 1;
		/*pr_notice("%s: Key value Len = %d\n", __func__, ms_ini_file_data[ini_items].iKeyValueLen); */
		if (ms_ini_file_data[ini_items].iKeyValueLen > PARSER_MAX_KEY_VALUE_LEN) {
			/*ret = CFG_ERR_OUT_OF_LEN; */
			pr_err("MAX_KEY_VALUE_LEN: Out Of Length\n\n");
			return;
		}

		memcpy(ms_ini_file_data[ini_items].pKeyValue,
		       ini_buf + iEqualSign + 1, ms_ini_file_data[ini_items].iKeyValueLen);

		/*pr_notice("%s: key value = %s\n\n", __func__, ms_ini_file_data[ini_items].pKeyValue); */

		ini_items++;
	}

}

int my_parser(char *path)
{
	int res = 0;
	struct file *f = NULL;
	int fsize = 0;
	/*char filepath[128]; */

	/*memset(filepath, 0, sizeof(filepath)); */

	pr_notice("%s: path = %s\n", __func__, path);

	f = file_open(path, O_RDONLY, 0, &fsize);

	pr_notice("size = %d\n", fsize);

	if (fsize <= 0) {
		res = -1;
		goto out;
	}

	_gData = kmalloc(fsize + 1, GFP_KERNEL);
	memset(_gData, 0, sizeof(char) * fsize);

	res = file_read(f, 0, _gData, fsize);

	ms_ini_get_key_data(_gData);

out:
	file_close(f);
	return res;
}

void my_parser_exit(void)
{
	pr_notice("*** %s ***\n", __func__);
	if (_gData != NULL) {
		kfree(_gData);
		_gData = NULL;
	}
}
