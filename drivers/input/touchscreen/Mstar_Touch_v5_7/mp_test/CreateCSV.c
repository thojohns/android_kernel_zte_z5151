#include "global.h"

#ifdef CONFIG_TPD_MSTAR_TEST
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/time.h>
#include <linux/rtc.h>
#endif

extern MutualMpTest_t *ptMutualMpTest;
extern MutualMpTestResult_t *ptMutualMpTestResult;
extern struct _TP_INFO tpinfo;
#define SETCSVDATA_LEN  (1024 * 200)
#ifdef CONFIG_TPD_MSTAR_TEST
extern unsigned char mstar_str_save_file_path[256];
static int mstar_test_get_save_filename(char *filename, int len)
{
	char *board_idFile = { "/persist/factoryinfo/board_id" };
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	/*unsigned long magic; */
	off_t fsize = 0;
	char filepath[128];
	loff_t pos = 0;
	mm_segment_t old_fs;
	char board_id[20];
	struct timespec ts;
	struct rtc_time tm;

	/*get rtc time */
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	memset(filepath, 0, sizeof(filepath));
	snprintf(filepath, sizeof(filepath), "%s", board_idFile);

	if (pfile == NULL) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}

	if (IS_ERR(pfile)) {
		MSTAR_TEST_DBG("error occurred while opening file %s.", filepath);
		snprintf(filename, len, "test_data%04d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
		return 0;
	}

	inode = pfile->f_dentry->d_inode;
	/*magic = inode->i_sb->s_magic; */
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, board_id, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	/*snprintf(filename, len, "TP_test_data%s_%02d%02d%02d", board_id, tm.tm_hour, tm.tm_min, tm.tm_sec); */
	snprintf(filename, len, "TP_test_data%s", board_id);
	return 0;
}
#endif

void save_test_data(void)
{
	int i = 0, j = 0, max_channel, test_Interval = 3, testCount = 0, failCount = 0;
/*char *pbuf = NULL;*/
	char sResult[60];
	char *SetCsvData = NULL, CsvPATHName[256] = "";
	struct file *f = NULL;
	mm_segment_t fs;

#ifdef CONFIG_TPD_MSTAR_TEST
	char filename[64];
	char procfilename[64];
#endif

	fs = get_fs();
	set_fs(KERNEL_DS);

	SetCsvData = kmalloc(SETCSVDATA_LEN, GFP_KERNEL);
	if (ptMutualMpTestResult->nShortResult == ITO_TEST_OK && ptMutualMpTestResult->nOpenResult == ITO_TEST_OK) {
		strlcpy(sResult, "PASS", sizeof(sResult));

	} else {
		if (testCount >= test_Interval) {
			failCount++;
			if (failCount == test_Interval)
				strlcpy(sResult, "FAIL", sizeof(sResult));
			else
				strlcpy(sResult, "FAIL", sizeof(sResult));
		}
		strlcpy(sResult, "FAIL", sizeof(sResult));
	}

#ifdef CONFIG_TPD_MSTAR_TEST

	mstar_test_get_save_filename(filename, 56);
	strlcpy(procfilename, filename, 56);
	strlcpy(sResult, filename, 56);

#endif

	snprintf(CsvPATHName, sizeof(CsvPATHName), "%s%s.csv", mstar_str_save_file_path, sResult);
	pr_notice("CSV:%s\n", CsvPATHName);
	if (f == NULL)
		f = filp_open(CsvPATHName, O_CREAT | O_RDWR, 0644);

	if (IS_ERR(f)) {
		pr_err("Failed to open csv file %s\n", CsvPATHName);
		goto fail_open;
	}

	/*snprintf(SetCsvData, 1024, "Golden 0 Max,,"); */
	strlcpy(SetCsvData, "Golden 0 Max,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < MAX_MUTUAL_NUM; i++) {
		/*snprintf(SetCsvData, 1024, "%1f,", ptMutualMpTestResult->pGolden_CH_Max[i]); */
		snprintf(SetCsvData, 1024, "%d,", ptMutualMpTestResult->pGolden_CH_Max[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	/*snprintf(SetCsvData, 1024, "Golden 0,,"); */
	strlcpy(SetCsvData, "Golden 0,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < MAX_MUTUAL_NUM; i++) {
		/*snprintf(SetCsvData, 1024, "%1f,", ptMutualMpTestResult->pGolden_CH[i]); */
		snprintf(SetCsvData, 1024, "%d,", ptMutualMpTestResult->pGolden_CH[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	/*snprintf(SetCsvData, 1024, "Golden 0 Min,,"); */
	strlcpy(SetCsvData, "Golden 0 Min,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < MAX_MUTUAL_NUM; i++) {
		/*snprintf(SetCsvData, 1024, "%1f,", ptMutualMpTestResult->pGolden_CH_Min[i]); */
		snprintf(SetCsvData, 1024, "%1d,", ptMutualMpTestResult->pGolden_CH_Min[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);

	/*snprintf(SetCsvData, 1024, "test_0_deltaC,,"); */
	strlcpy(SetCsvData, "test_0_deltaC,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < MAX_MUTUAL_NUM; i++) {
		snprintf(SetCsvData, 1024, "%d,", ptMutualMpTestResult->pOpenResultData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);

	/*snprintf(SetCsvData, 1024, "test_0_ratio,[%1$.2f~%2$.2f],",
	ptMutualMpTestResult->nRatioAvg_max, ptMutualMpTestResult->nRatioAvg_min); */
	snprintf(SetCsvData, 1024, "test_0_ratio,[%1d~%2d],", ptMutualMpTestResult->nRatioAvg_max,
		ptMutualMpTestResult->nRatioAvg_min);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < MAX_MUTUAL_NUM; i++) {
		/*snprintf(SetCsvData, 1024, "%1f,", ptMutualMpTestResult->pGolden_CH_Max_Avg[i]); */
		snprintf(SetCsvData, 1024, "%1d,", ptMutualMpTestResult->pGolden_CH_Max_Avg[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);

	/*snprintf(SetCsvData, 1024, "test_border_ratio,[%1$.2f~%2$.2f] ",
	ptMutualMpTestResult->nBorder_RatioAvg_max, ptMutualMpTestResult->nBorder_RatioAvg_min); */
	snprintf(SetCsvData, 1024, "test_border_ratio,[%1d~%2d] ", ptMutualMpTestResult->nBorder_RatioAvg_max,
		ptMutualMpTestResult->nBorder_RatioAvg_min);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < MAX_MUTUAL_NUM; i++) {
		snprintf(SetCsvData, 1024, "%s", ",");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "Platform Version :%s\n", tpinfo.PlatformVersion);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	snprintf(SetCsvData, 1024, "Device Driver Version : %s\n", DEVICE_DRIVER_RELEASE_VERSION);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	snprintf(SetCsvData, 1024, "FW Version : %s\n", tpinfo.FwVersion);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	snprintf(SetCsvData, 1024, "Main Block FW Version : %s\n", tpinfo.MainBlockFWVersion);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	snprintf(SetCsvData, 1024, "Info Block FW Version : %s\n", tpinfo.InfoBlockFWVersion);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	snprintf(SetCsvData, 1024, "ANA_Version : %s\n", ptMutualMpTest->ana_version);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	snprintf(SetCsvData, 1024, "SupportIC : %s\n", ptMutualMpTest->UIConfig.sSupportIC);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	snprintf(SetCsvData, 1024, "Project name : %s\n", ptMutualMpTest->project_name);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	snprintf(SetCsvData, 1024, "Mapping table name : %s\n", ptMutualMpTestResult->mapTbl_sec);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	/*snprintf(SetCsvData, 1024, "DC_Range=%u\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range); */
	snprintf(SetCsvData, 1024, "DC_Range=%d\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	/*snprintf(SetCsvData, 1024, "DC_Range_up=%hu\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range_up); */
	snprintf(SetCsvData, 1024, "DC_Range_up=%d\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range_up);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	/*snprintf(SetCsvData, 1024, "DC_Ratio=%u\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio); */
	snprintf(SetCsvData, 1024, "DC_Ratio=%d\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	/*snprintf(SetCsvData, 1024, "DC_Border_Ratio=%u\n", ptMutualMpTest->ToastInfo.persentDC_Border_Ratio); */
	snprintf(SetCsvData, 1024, "DC_Border_Ratio=%d\n", ptMutualMpTest->ToastInfo.persentDC_Border_Ratio);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	/*snprintf(SetCsvData, 1024, "DC_Ratio_up=%hu\n\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up); */
	snprintf(SetCsvData, 1024, "DC_Ratio_up=%d\n\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);

	/*snprintf(SetCsvData, 1024, "Golden,,"); */
	strlcpy(SetCsvData, "Golden,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
		snprintf(SetCsvData, 1024, "D%d,", i + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++) {
		snprintf(SetCsvData, 1024, ",S%d,", j + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
			if (ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i] == NULL_DATA) {
				/*for mutual key */
				snprintf(SetCsvData, 1024, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			} else {
				/*snprintf(SetCsvData, 1024, "%1$.2f,",
				ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i]); */
				snprintf(SetCsvData, 1024, "%.2d,",
					ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
		}
		snprintf(SetCsvData, 1024, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);

	/*snprintf(SetCsvData, 1024, "Golden_Max,,"); */
	strlcpy(SetCsvData, "Golden_Max,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
		snprintf(SetCsvData, 1024, "D%d,", i + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++) {
		snprintf(SetCsvData, 1024, ",S%d,", j + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
			if (ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i] == NULL_DATA) {
				/*for mutual key */
				snprintf(SetCsvData, 1024, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			} else {
				/*snprintf(SetCsvData, 1024, "%1$.2f,",
				ptMutualMpTestResult->pGolden_CH_Max[j * ptMutualMpTest->sensorInfo.numDrv + i]); */
				snprintf(SetCsvData, 1024, "%.2d,",
					ptMutualMpTestResult->pGolden_CH_Max[j * ptMutualMpTest->sensorInfo.numDrv +
									     i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
		}
		snprintf(SetCsvData, 1024, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);

	/*snprintf(SetCsvData, 1024, "Golden_Min,,"); */
	strlcpy(SetCsvData, "Golden_Min,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
		snprintf(SetCsvData, 1024, "D%d,", i + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++) {
		snprintf(SetCsvData, 1024, ",S%d,", j + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
			if (ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i] == NULL_DATA) {
				/*for mutual key */
				snprintf(SetCsvData, 1024, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			} else {
				/*snprintf(SetCsvData, 1024, "%1$.2f,",
				ptMutualMpTestResult->pGolden_CH_Min[j * ptMutualMpTest->sensorInfo.numDrv + i]); */
				snprintf(SetCsvData, 1024, "%.2d,",
					ptMutualMpTestResult->pGolden_CH_Min[j * ptMutualMpTest->sensorInfo.numDrv +
									     i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
		}
		snprintf(SetCsvData, 1024, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);

	/*snprintf(SetCsvData, 1024, "DeltaC,,"); */
	strlcpy(SetCsvData, "DeltaC,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
		snprintf(SetCsvData, 1024, "D%d,", i + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++) {
		snprintf(SetCsvData, 1024, ",S%d,", j + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
			if (ptMutualMpTestResult->pOpenResultData[j * ptMutualMpTest->sensorInfo.numDrv + i]
				== NULL_DATA) {
				/*for mutual key */
				snprintf(SetCsvData, 1024, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			} else {
				/*snprintf(SetCsvData, 1024, "%1$d,",
				ptMutualMpTestResult->pOpenResultData[j * ptMutualMpTest->sensorInfo.numDrv + i]); */
				snprintf(SetCsvData, 1024, "%1d,",
					ptMutualMpTestResult->pOpenResultData[j * ptMutualMpTest->sensorInfo.numDrv +
									      i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
		}
		snprintf(SetCsvData, 1024, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	if (ptMutualMpTestResult->nOpenResult == 1) {
		snprintf(SetCsvData, 1024, "DeltaC_Result:PASS\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	} else {
		if (ptMutualMpTestResult->pCheck_Fail[0] == 1) {
			snprintf(SetCsvData, 1024, "DeltaC_Result:FAIL\nFail Channel:");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			for (i = 0; i < MAX_MUTUAL_NUM; i++) {
				if (ptMutualMpTestResult->pOpenFailChannel[i] == PIN_NO_ERROR)
					continue;
				/*snprintf(SetCsvData, 1024,"D%1$d.S%2$d",
				ptMutualMpTestResult->pOpenFailChannel[i] % 100,
				ptMutualMpTestResult->pOpenFailChannel[i] / 100); */
				snprintf(SetCsvData, 1024, "D%1d.S%2d", ptMutualMpTestResult->pOpenFailChannel[i] % 100,
					ptMutualMpTestResult->pOpenFailChannel[i] / 100);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
			snprintf(SetCsvData, 1024, "\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		} else {
			snprintf(SetCsvData, 1024, "DeltaC_Result:PASS\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
	}
	/*snprintf(SetCsvData, 1024, "\nRatio,,"); */
	strlcpy(SetCsvData, "\nRatio,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
		snprintf(SetCsvData, 1024, "D%d,", i + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++) {
		snprintf(SetCsvData, 1024, ",S%d,", j + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		for (i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++) {
			if (ptMutualMpTestResult->pOpenResultData[j * ptMutualMpTest->sensorInfo.numDrv + i]
				== NULL_DATA) {
				/*for mutual key */
				snprintf(SetCsvData, 1024, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			} else {
				/*snprintf(SetCsvData, 1024, "%1$f,",
				ptMutualMpTestResult->pGolden_CH_Max_Avg[j * ptMutualMpTest->sensorInfo.numDrv + i]); */
				snprintf(SetCsvData, 1024, "%1d,",
					ptMutualMpTestResult->pGolden_CH_Max_Avg[j * ptMutualMpTest->sensorInfo.numDrv +
										 i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
		}
		snprintf(SetCsvData, 1024, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	if (ptMutualMpTestResult->nOpenResult == 1) {
		snprintf(SetCsvData, 1024, "Ratio_Result:PASS\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	} else {
		if (ptMutualMpTestResult->pCheck_Fail[1] == 1) {
			snprintf(SetCsvData, 1024, "Ratio_Result:FAIL\nFail Channel:");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			for (i = 0; i < MAX_MUTUAL_NUM; i++) {
				if (ptMutualMpTestResult->pOpenFailChannel[i] == PIN_NO_ERROR)
					continue;
				/*snprintf(SetCsvData, 1024,"D%1$d.S%2$d",
				ptMutualMpTestResult->pOpenRatioFailChannel[i] % 100,
				ptMutualMpTestResult->pOpenRatioFailChannel[i] / 100); */
				snprintf(SetCsvData, 1024, "D%1d.S%2d",
				ptMutualMpTestResult->pOpenRatioFailChannel[i] % 100,
				ptMutualMpTestResult->pOpenRatioFailChannel[i] / 100);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
			snprintf(SetCsvData, 1024, "\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		} else {
			snprintf(SetCsvData, 1024, "Ratio_Result:PASS\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
	}
	snprintf(SetCsvData, 1024, "\n\nShortValue=%d\n\n", ptMutualMpTest->sensorInfo.thrsShort);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);

	snprintf(SetCsvData, 1024, "ICPinShort=%d\n\n", ptMutualMpTest->sensorInfo.thrsICpin);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);

	/*snprintf(SetCsvData, 1024, "Pin Number,,"); */
	strlcpy(SetCsvData, "Pin Number,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	max_channel = MAX_CHANNEL_NUM_28XX;
	if (tpinfo.ChipType == CHIP_TYPE_MSG28XXA)
		max_channel = MAX_CHANNEL_NUM_30XX;

	for (i = 0; i < max_channel; i++) {
		if (ptMutualMpTestResult->pICPinChannel[i] == 0) {
			continue;
		}
		/*logHeader[i+1-j] = "P" + Integer.toString(ptMutualMpTestResult->pICPinChannel[i]); */
		snprintf(SetCsvData, 1024, "P%d,", ptMutualMpTestResult->pICPinChannel[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	/*snprintf(SetCsvData, 1024, "\ndeltaR,,"); */
	strlcpy(SetCsvData, "\ndeltaR,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < max_channel; i++) {
		if (ptMutualMpTestResult->pICPinChannel[i] == 0) {
			continue;
		}
		/*snprintf(SetCsvData, 1024, "%1$.1fM,",ptMutualMpTestResult->pICPinShortRData[i]); */
		snprintf(SetCsvData, 1024, "%1dM,", ptMutualMpTestResult->pICPinShortRData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}

	/*snprintf(SetCsvData, 1024, "\nresultData,,"); */
	strlcpy(SetCsvData, "\nresultData,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < max_channel; i++) {
		if (ptMutualMpTestResult->pICPinChannel[i] == 0) {
			continue;
		}
		/*snprintf(SetCsvData, 1024, "%1$d,",ptMutualMpTestResult->pICPinShortResultData[i]); */
		snprintf(SetCsvData, 1024, "%d,", ptMutualMpTestResult->pICPinShortResultData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}

	if (ptMutualMpTestResult->nShortResult == ITO_TEST_OK) {
		snprintf(SetCsvData, 1024, "\nICPin Short Test:PASS\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	} else {
		if (ptMutualMpTestResult->pCheck_Fail[2] == 1) {
			snprintf(SetCsvData, 1024, "\nICPin Short Test:FAIL\nFail Channel:,,");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			for (i = 0; i < max_channel; i++) {
				if (ptMutualMpTestResult->pICPinShortFailChannel[i] == 0) {
					continue;
				}
				/*snprintf(SetCsvData, 1024, "P%1$d,",
				ptMutualMpTestResult->pICPinShortFailChannel[i]); */
				snprintf(SetCsvData, 1024, "P%d,", ptMutualMpTestResult->pICPinShortFailChannel[i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
		} else {

			snprintf(SetCsvData, 1024, "\nICPin Short Test:PASS\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
	}

	/*snprintf(SetCsvData, 1024, "\ndeltaR,,"); */
	strlcpy(SetCsvData, "\ndeltaR,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < 10; i++) {
		snprintf(SetCsvData, 1024, "%d,", i + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	for (i = 0; i < (ptMutualMpTest->sensorInfo.numSen); i++) {
		if ((i % 10) == 0) {
			snprintf(SetCsvData, 1024, "\n,S%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
		/*snprintf(SetCsvData, 1024, "%1$.1fM,", ptMutualMpTestResult->pShortRData[i]); */
		snprintf(SetCsvData, 1024, "%dM,", ptMutualMpTestResult->pShortRData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < (ptMutualMpTest->sensorInfo.numDrv); i++) {
		if ((i % 10) == 0) {
			snprintf(SetCsvData, 1024, "\n,D%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
		/*snprintf(SetCsvData, 1024, "%1$.1fM,",
		ptMutualMpTestResult->pShortRData[i + ptMutualMpTest->sensorInfo.numSen]); */
		TEST_DBG(0, " pshortdata = %d\n",
			 ptMutualMpTestResult->pShortRData[i + ptMutualMpTest->sensorInfo.numSen]);
		snprintf(SetCsvData, 1024, "%1dM,",
			ptMutualMpTestResult->pShortRData[i + ptMutualMpTest->sensorInfo.numSen]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}

	snprintf(SetCsvData, 1024, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < ptMutualMpTest->sensorInfo.numGr; i++) {
		if ((i % 10) == 0) {
			snprintf(SetCsvData, 1024, "\n,GR%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
		/*snprintf(SetCsvData, 1024, "%1$d,",
		ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen +
		ptMutualMpTest->sensorInfo.numDrv]); */
		snprintf(SetCsvData, 1024, "%d,",
			ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen +
							       ptMutualMpTest->sensorInfo.numDrv]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}

	if (ptMutualMpTestResult->nShortResult == ITO_TEST_OK) {
		/*snprintf(SetCsvData, 1024, "\nITO Short Test:PASS,"); */
		strlcpy(SetCsvData, "\nITO Short Test:PASS,", SETCSVDATA_LEN);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	} else {
		if (ptMutualMpTestResult->pCheck_Fail[3] == 1) {
			if (testCount >= test_Interval) {
				if (failCount == test_Interval) {
					/*snprintf(SetCsvData, 1024, "\nITO Short Test:FAIL\nFail Channel:,,"); */
					strlcpy(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,",
						SETCSVDATA_LEN);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
					j = 0;
					for (i = 0; i < ptMutualMpTest->sensorInfo.numSen; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						/*snprintf(SetCsvData, 1024, "S%1$d,", i + 1); */
						snprintf(SetCsvData, 1024, "S%d,", i + 1);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char),
							       &f->f_pos);
					}
					for (;
					     i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv;
					     i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						/*snprintf(SetCsvData, 1024, "D%1$d,",
							i + 1 - ptMutualMpTest->sensorInfo.numSen); */
						snprintf(SetCsvData, 1024, "D%d,",
							i + 1 - ptMutualMpTest->sensorInfo.numSen);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char),
							       &f->f_pos);
					}
					for (;
					     i <
					     ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv +
					     ptMutualMpTest->sensorInfo.numGr; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						/*snprintf(SetCsvData, 1024, "GR%1$d",
						i + 1 - ptMutualMpTest->sensorInfo.numSen
						- ptMutualMpTest->sensorInfo.numDrv); */
						snprintf(SetCsvData, 1024, "GR%d",
							i + 1 - ptMutualMpTest->sensorInfo.numSen -
							ptMutualMpTest->sensorInfo.numDrv);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char),
							       &f->f_pos);
					}
				} else {
					/*snprintf(SetCsvData, 1024, "\nITO Short Test:PASS,"); */
					strlcpy(SetCsvData, "\nITO Short Test:PASS,", SETCSVDATA_LEN);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				}
			} else {
				/*snprintf(SetCsvData, 1024, "\nITO Short Test:FAIL\nFail Channel:,,"); */
				strlcpy(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,", SETCSVDATA_LEN);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				j = 0;
				for (i = 0; i < ptMutualMpTest->sensorInfo.numSen; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					/*snprintf(SetCsvData, 1024, "S%1$d,", i + 1); */
					snprintf(SetCsvData, 1024, "S%d,", i + 1);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				}
				for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					/*snprintf(SetCsvData, 1024, "D%1$d,",
					i + 1 - ptMutualMpTest->sensorInfo.numSen); */
					snprintf(SetCsvData, 1024, "D%d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				}
				for (;
				     i <
				     ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv +
				     ptMutualMpTest->sensorInfo.numGr; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					/*snprintf(SetCsvData, 1024, "GR%1$d",
					i + 1 - ptMutualMpTest->sensorInfo.numSen -
					ptMutualMpTest->sensorInfo.numDrv); */
					snprintf(SetCsvData, 1024, "GR%d",
						i + 1 - ptMutualMpTest->sensorInfo.numSen -
						ptMutualMpTest->sensorInfo.numDrv);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				}
			}
		} else {
			/*snprintf(SetCsvData, 1024, "\nITO Short Test:PASS,"); */
			strlcpy(SetCsvData, "\nITO Short Test:PASS,", SETCSVDATA_LEN);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
	}

	/*snprintf(SetCsvData, 1024, "\nresultData,,"); */
	strlcpy(SetCsvData, "\nresultData,,", SETCSVDATA_LEN);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for (i = 0; i < 10; i++) {
		snprintf(SetCsvData, 1024, "%d,", i + 1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	for (i = 0; i < (ptMutualMpTest->sensorInfo.numSen); i++) {
		if ((i % 10) == 0) {
			snprintf(SetCsvData, 1024, "\n,S%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
		/*snprintf(SetCsvData, 1024, "%1$d,",  ptMutualMpTestResult->pShortResultData[i]); */
		snprintf(SetCsvData, 1024, "%d,", ptMutualMpTestResult->pShortResultData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}

	for (i = 0; i < (ptMutualMpTest->sensorInfo.numDrv); i++) {
		if ((i % 10) == 0) {
			snprintf(SetCsvData, 1024, "\n,D%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
		/*snprintf(SetCsvData, 1024, "%1$d,",
		ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen]); */
		snprintf(SetCsvData, 1024, "%d,",
			ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}

	for (i = 0; i < (ptMutualMpTest->sensorInfo.numGr); i++) {
		if ((i % 10) == 0) {
			snprintf(SetCsvData, 1024, "\n,GR%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
		/*snprintf(SetCsvData, 1024, "%1$d,",
		ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen +
		ptMutualMpTest->sensorInfo.numDrv]); */
		snprintf(SetCsvData, 1024, "%d,",
			ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen +
							       ptMutualMpTest->sensorInfo.numDrv]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}

	if (ptMutualMpTestResult->nShortResult == ITO_TEST_OK) {
		/*snprintf(SetCsvData, 1024, "\nITO Short Test:PASS\n"); */
		strlcpy(SetCsvData, "\nITO Short Test:PASS\n", SETCSVDATA_LEN);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	} else {
		if (ptMutualMpTestResult->pCheck_Fail[3] == 1) {
			if (testCount >= test_Interval) {
				if (failCount == test_Interval) {
					/*snprintf(SetCsvData, 1024, "\nITO Short Test:FAIL\nFail Channel:,,"); */
					strlcpy(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,",
						SETCSVDATA_LEN);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
					j = 0;
					for (i = 0; i < ptMutualMpTest->sensorInfo.numSen; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						/*snprintf(SetCsvData, 1024, "S%1$d,", i + 1); */
						snprintf(SetCsvData, 1024, "S%d,", i + 1);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char),
							       &f->f_pos);
					}
					for (;
					     i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv;
					     i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						/*snprintf(SetCsvData, 1024, "D%1$d,",
						i + 1 - ptMutualMpTest->sensorInfo.numSen); */
						snprintf(SetCsvData, 1024, "D%d,",
							i + 1 - ptMutualMpTest->sensorInfo.numSen);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char),
							       &f->f_pos);
					}
					for (;
					     i <
					     ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv +
					     ptMutualMpTest->sensorInfo.numGr; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						/*snprintf(SetCsvData, 1024, "GR%1$d",
						i + 1 - ptMutualMpTest->sensorInfo.numSen -
						ptMutualMpTest->sensorInfo.numDrv); */
						snprintf(SetCsvData, 1024, "GR%d",
							i + 1 - ptMutualMpTest->sensorInfo.numSen -
							ptMutualMpTest->sensorInfo.numDrv);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char),
							       &f->f_pos);
					}
				} else {
					/*snprintf(SetCsvData, 1024, "\nITO Short Test:PASS\n"); */
					strlcpy(SetCsvData, "\nITO Short Test:PASS\n", SETCSVDATA_LEN);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				}
			} else {
				/*snprintf(SetCsvData, 1024, "\nITO Short Test:FAIL\nFail Channel:,,"); */
				strlcpy(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,", SETCSVDATA_LEN);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				j = 0;
				for (i = 0; i < ptMutualMpTest->sensorInfo.numSen; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					/*snprintf(SetCsvData, 1024, "S%1$d,", i + 1); */
					snprintf(SetCsvData, 1024, "S%d,", i + 1);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				}
				for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					/*snprintf(SetCsvData, 1024, "D%1$d,",
					i + 1 - ptMutualMpTest->sensorInfo.numSen); */
					snprintf(SetCsvData, 1024, "D%d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				}
				for (;
				     i <
				     ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv +
				     ptMutualMpTest->sensorInfo.numGr; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					/*snprintf(SetCsvData, 1024, "GR%1$d",
					i + 1 - ptMutualMpTest->sensorInfo.numSen -
					ptMutualMpTest->sensorInfo.numDrv); */
					snprintf(SetCsvData, 1024, "GR%d",
						i + 1 - ptMutualMpTest->sensorInfo.numSen -
						ptMutualMpTest->sensorInfo.numDrv);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
				}
			}
		} else {
			/*snprintf(SetCsvData, 1024, "\nITO Short Test:PASS,"); */
			strlcpy(SetCsvData, "\nITO Short Test:PASS,", SETCSVDATA_LEN);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
		}
	}

	filp_close(f, NULL);
fail_open:
	set_fs(fs);
	kfree(SetCsvData);
}
