/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/************************************************************************
*
* File Name: focaltech_test.c
*
* Author:     Software Department, FocalTech
*
* Created: 2016-08-01
*
* Modify:
*
* Abstract: create char device and proc node for  the comm between APK and TP
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>		/*iic*/
#include <linux/delay.h>	/*msleep*/
#include <linux/time.h>
#include <linux/rtc.h>

#include "../../focaltech_core.h"
#include "../include/focaltech_test_main.h"
#include "../include/focaltech_test_ini.h"
#include "tpd_sys.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define IC_TEST_VERSION  "Test version:  V4.0.0.0 ---- 2016-07-18"

/*Define the configuration file storage directory*/
#define FTS_INI_FILE_PATH "/mnt/sdcard/"

#define FTS_TEST_BUFFER_SIZE        (80 * 1024)
#define FTS_TEST_PRINT_SIZE     128
/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/
static unsigned char g_str_save_file_path[256];
static unsigned char g_str_ini_file_path[256];
static unsigned char g_str_ini_filename[128];

static int g_node_data_type = -1;	/*tp test*/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
int g_int_tptest_result = 0;
struct fts_test_st fts_test;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_test_get_ini_size(char *config_name);
static int fts_test_read_ini_data(char *config_name, char *config_buf);
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen);
static int fts_test_get_testparam_from_ini(char *config_name);
static int fts_test_entry(char *ini_file_name);

static int fts_test_i2c_read(unsigned char *writebuf, int writelen,
			     unsigned char *readbuf, int readlen);
static int fts_test_i2c_write(unsigned char *writebuf, int writelen);

/*****************************************************************************
* functions body
*****************************************************************************/
#if 0
/*old fts_i2c_read/write function. need to set fts_i2c_client.*/
extern struct i2c_client *fts_i2c_client;
extern int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen,
			char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf,
			 int writelen);
#endif
static int fts_test_i2c_read(unsigned char *writebuf, int writelen,
			     unsigned char *readbuf, int readlen)
{
	int iret = -1;

	/*old fts_i2c_read function. need to set fts_i2c_client.*/
	/*Modify the i2c_read function that is used in this project*/
	iret = fts_i2c_read(fts_i2c_client, writebuf, writelen, readbuf, readlen);

	return iret;

}

static int fts_test_i2c_write(unsigned char *writebuf, int writelen)
{
	int iret = -1;

	/*old fts_i2c_write function.  need to set fts_i2c_client.*/
	/*Modify the i2c_read function that is used in this project*/
	iret = fts_i2c_write(fts_i2c_client, writebuf, writelen);

	return iret;
}

static int fts_test_get_save_filename(char *filename, int len)
{
	char *board_idFile = { "/persist/factoryinfo/board_id" };
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	/*unsigned long magic;*/
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
		FTS_TEST_DBG("error occurred while opening file %s.", filepath);
		snprintf(filename, len, "test_data%04d%02d%02d.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	/*magic = inode->i_sb->s_magic;*/
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, board_id, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	snprintf(filename, len, "TP_test_data%s.csv", board_id);
	return 0;

}

/*Gets the configuration file size for allocating memory to read configuration*/
static int fts_test_get_ini_size(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	/*unsigned long magic;*/
	off_t fsize = 0;
	char filepath[128];

	FTS_TEST_FUNC_ENTER();

	memset(filepath, 0, sizeof(filepath));

	snprintf(filepath, sizeof(filepath), "%s%s", g_str_ini_file_path, config_name);
	/*snprintf(filepath, sizeof(filepath), "%s%s", FTS_INI_FILE_PATH, config_name);*/

	if (pfile == NULL)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		FTS_TEST_ERROR("error occurred while opening file %s.", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	/*magic = inode->i_sb->s_magic;*/
	fsize = inode->i_size;
	filp_close(pfile, NULL);

	FTS_TEST_FUNC_ENTER();

	return fsize;
}

/*Read configuration to memory*/
static int fts_test_read_ini_data(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	/*unsigned long magic;*/
	off_t fsize = 0;
	char filepath[128];
	loff_t pos = 0;
	mm_segment_t old_fs;

	FTS_TEST_FUNC_ENTER();

	memset(filepath, 0, sizeof(filepath));
	snprintf(filepath, sizeof(filepath), "%s%s", g_str_ini_file_path, config_name);
	/*snprintf(filepath, sizeof(filepath), "%s%s", FTS_INI_FILE_PATH, config_name);*/
	if (pfile == NULL) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		FTS_TEST_ERROR("error occurred while opening file %s.", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	/*magic = inode->i_sb->s_magic;*/
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	FTS_TEST_FUNC_EXIT();
	return 0;
}

/*Save test data to SD card etc.*/
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen)
{
	struct file *pfile = NULL;

	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	FTS_TEST_FUNC_ENTER();

	memset(filepath, 0, sizeof(filepath));
	snprintf(filepath, sizeof(filepath), "%s%s", g_str_save_file_path, file_name);
	/*snprintf(filepath, sizeof(filepath), "%s%s", FTS_INI_FILE_PATH, file_name);*/
	if (pfile == NULL) {

		pfile = filp_open(filepath, O_TRUNC | O_CREAT | O_RDWR, 0);
	}
	if (IS_ERR(pfile)) {
		FTS_TEST_ERROR("error occurred while opening file %s.", filepath);
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(pfile, data_buf, iLen, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	FTS_TEST_FUNC_EXIT();
	return 0;
}

/*Read, parse the configuration file, initialize the test variable*/
static int fts_test_get_testparam_from_ini(char *config_name)
{
	char *pcfiledata = NULL;
	int ret = 0;
	int inisize = 0;

	FTS_TEST_FUNC_ENTER();

	inisize = fts_test_get_ini_size(config_name);
	FTS_TEST_DBG("ini_size = %d ", inisize);
	if (inisize <= 0) {
		FTS_TEST_ERROR("%s ERROR:Get firmware size failed", __func__);
		return -EIO;
	}

	pcfiledata = fts_malloc(inisize + 1);
	if (pcfiledata == NULL) {
		FTS_TEST_ERROR("fts_malloc failed in function:%s", __func__);
		return -EPERM;
	}

	memset(pcfiledata, 0, inisize + 1);

	if (fts_test_read_ini_data(config_name, pcfiledata)) {
		FTS_TEST_ERROR(" - ERROR: fts_test_read_ini_data failed");
		fts_free(pcfiledata);
		pcfiledata = NULL;

		goto READ_INI_ERR;
	} else {
		FTS_TEST_DBG("fts_test_read_ini_data successful");
	}

	ret = set_param_data(pcfiledata);

	fts_free(pcfiledata);	/*lifengshi add. 20160608*/
	pcfiledata = NULL;

	FTS_TEST_FUNC_EXIT();

	if (ret < 0)
		return ret;

	return 0;
READ_INI_ERR:
	return -EIO;
}


/*Test library call entry*/

static int fts_test_entry(char *ini_file_name)
{
	/* place holder for future use */
	char cfgname[128];
	char *testdata = NULL;
	char *printdata = NULL;
	/*The actual length of the test data in the library is used to save the data to the file.*/
	int iTestDataLen = 0;
	int ret = 0;
	int icycle = 0, i = 0;
	int print_index = 0;

	FTS_TEST_FUNC_ENTER();
	FTS_TEST_DBG("ini_file_name:%s.", ini_file_name);
	/*Used to obtain the test data stored in the library, pay attention to the size of the distribution space. */
	FTS_TEST_DBG("Allocate memory, size: %d", FTS_TEST_BUFFER_SIZE);
	testdata = fts_malloc(FTS_TEST_BUFFER_SIZE);
	if (testdata == NULL) {
		FTS_TEST_ERROR("fts_malloc failed in function:%s", __func__);
		return -EPERM;
	}
	printdata = fts_malloc(FTS_TEST_PRINT_SIZE);
	if (printdata == NULL) {
		FTS_TEST_ERROR("fts_malloc failed in function:%s", __func__);
		fts_free(testdata);
		return -EPERM;
	}
	/*Initialize the platform related I2C read and write functions */

#if 0
	init_i2c_write_func(fts_i2c_write);
	init_i2c_read_func(fts_i2c_read);
#else
	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);
#endif

	/*Initialize pointer memory */
	ret = focaltech_test_main_init();
	if (ret < 0) {
		FTS_TEST_ERROR("focaltech_test_main_init() error.");
		goto TEST_ERR;
	}

	/*Read parse configuration file */
	memset(cfgname, 0, sizeof(cfgname));
	snprintf(cfgname, sizeof(cfgname), "%s", ini_file_name);
	FTS_TEST_DBG("ini_file_name = %s", cfgname);

	fts_test_funcs();

	if (fts_test_get_testparam_from_ini(cfgname) < 0) {
		FTS_TEST_ERROR("get testparam from ini failure");
		goto TEST_ERR;
	}

	if ((g_ScreenSetParam.iSelectedIC >> 4 != FTS_CHIP_TEST_TYPE >> 4)) {
		FTS_TEST_ERROR
		    ("Select IC and Read IC from INI does not match ");
		goto TEST_ERR;
	}

	/*Start testing according to the test configuration */
	if (true == start_test_tp()) {
		TestResultLen += snprintf(TestResult + TestResultLen,
			BUFF_LEN_TMP_BUFFER - TestResultLen, "Tp test pass.\n\n");
		FTS_TEST_INFO("tp test pass");
	}

	else {
		TestResultLen += snprintf(TestResult + TestResultLen,
			BUFF_LEN_TMP_BUFFER - TestResultLen, "Tp test failure.\n\n");
		FTS_TEST_INFO("tp test failure");
	}

	/*Gets the number of tests in the test library and saves it */
	iTestDataLen = get_test_data(testdata);
	/*FTS_TEST_DBG("\n%s", testdata);*/

	icycle = 0;
	FTS_TEST_DBG("print test data:\n");
	for (i = 0; i < iTestDataLen; i++) {
		if ((testdata[i] == '\0')	/*Meet the end*/
		    || (icycle == FTS_TEST_PRINT_SIZE - 2)	/*Meet the print string length requirements*/
		    || (i == iTestDataLen - 1)	/*The last character*/
		    ) {
			if (icycle == 0) {
				print_index++;
			} else {
				memcpy(printdata, testdata + print_index, icycle);
				printdata[FTS_TEST_PRINT_SIZE - 1] = '\0';
				print_index += icycle;
				icycle = 0;
			}
		} else {
			icycle++;
		}
	}
	FTS_TEST_DBG("\n");
	{
		char filename[64];

		fts_test_get_save_filename(filename, 64);
		fts_test_save_test_data(filename, testdata, iTestDataLen);
	}
	/*fts_test_save_test_data("testdata.csv", testdata, iTestDataLen);*/
	fts_test_save_test_data("testresult.txt", TestResult, TestResultLen);

	/*Release memory */
	focaltech_test_main_exit();

	/*mutex_unlock(&g_device_mutex);*/
	if (testdata != NULL)
		fts_free(testdata);
	if (printdata != NULL)
		fts_free(printdata);

	FTS_TEST_FUNC_EXIT();

	return 0;

TEST_ERR:
	/*Release memory */
	focaltech_test_main_exit();
	if (testdata != NULL)
		fts_free(testdata);
	if (printdata != NULL)
		fts_free(printdata);

	FTS_TEST_FUNC_EXIT();

	return -EPERM;
}

/************************************************************************
* Name: fts_test_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_test_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return -EPERM;
}

/************************************************************************
* Name: fts_test_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_test_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	char fwname[128] = { 0 };
	struct i2c_client *client = fts_i2c_client;

	FTS_TEST_FUNC_ENTER();

	memset(fwname, 0, sizeof(fwname));
	snprintf(fwname, sizeof(fwname), "%s", buf);
	fwname[count - 1] = '\0';
	FTS_TEST_DBG("fwname:%s.", fwname);

	mutex_lock(&fts_input_dev->mutex);

	disable_irq(client->irq);

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
	fts_esdcheck_switch(DISABLE);
#endif
	fts_test_entry(fwname);

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
	fts_esdcheck_switch(ENABLE);
#endif
	enable_irq(client->irq);

	mutex_unlock(&fts_input_dev->mutex);

	FTS_TEST_FUNC_EXIT();

	return count;
}

/*  upgrade from app.bin
*    example:echo "***.ini" > fts_test
*/
/*tp test*/
extern int tpd_test_get_channel_setting(int *buffer);
extern int tpd_test_get_tp_node_data(int type, char *buffer, int length);

int fts_save_failed_node_to_buffer(char *tmp_buffer, int length)
{

	if (fts_test.special_buffer == NULL || (fts_test.special_buffer_length + length) > TEST_RESULT_LENGTH) {
		FTS_TEST_INFO("warning:buffer is null or buffer overflow, return");
		return -EPERM;
	}

	memcpy(fts_test.special_buffer + fts_test.special_buffer_length, tmp_buffer, length);
	fts_test.special_buffer_length += length;
	fts_test.rawdata_failed_count++;

	return 0;
}

int fts_save_failed_node(int iRow, int iCol)
{
	int i_len = 0;

	if (fts_test.temp_buffer != NULL) {
		i_len = snprintf(fts_test.temp_buffer, TEST_TEMP_LENGTH, ",%d,%d", iRow, iCol);
		fts_save_failed_node_to_buffer(fts_test.temp_buffer, i_len);
		return 0;
	} else
		return -EPERM;
}

static int tpd_test_save_file_path_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&fts_input_dev->mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_save_file_path);

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_save_file_path_store(struct tpd_classdev_t *cdev,
					 const char *buf)
{
	memset(g_str_save_file_path, 0, sizeof(g_str_save_file_path));
	snprintf(g_str_save_file_path, 256, "%s", buf);
	/*g_str_save_file_path[count] = '\0';*/

	FTS_TEST_DBG("save file path:%s.", g_str_save_file_path);

	return 0;
}

static int tpd_test_ini_file_path_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&fts_input_dev->mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_file_path);

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_ini_file_path_store(struct tpd_classdev_t *cdev,
					const char *buf)
{
	memset(g_str_ini_file_path, 0, sizeof(g_str_ini_file_path));
	snprintf(g_str_ini_file_path, 256, "%s", buf);
	/*g_str_ini_file_path[count] = '\0';*/

	FTS_TEST_DBG("ini file path:%s.", g_str_ini_file_path);

	return 0;
}

static int tpd_test_filename_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&fts_input_dev->mutex);

	num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_filename);

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_filename_store(struct tpd_classdev_t *cdev, const char *buf)
{
	memset(g_str_ini_filename, 0, sizeof(g_str_ini_filename));
	snprintf(g_str_ini_filename, 128, "%s", buf);
	/*g_str_ini_filename[count] = '\0';*/

	FTS_TEST_DBG("fwname:%s.", g_str_ini_filename);

	return 0;
}

static int tpd_test_cmd_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	struct i2c_client *client = fts_i2c_client;
	int buffer_length = 0;
	int i_len = 0;

	mutex_lock(&fts_input_dev->mutex);
	disable_irq(client->irq);
	i_len = snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d", g_int_tptest_result,
			g_ScreenSetParam.iTxNum, g_ScreenSetParam.iRxNum,  fts_test.rawdata_failed_count);
	FTS_TEST_INFO("tpd test resutl:0x%x && rawdata node failed count:0x%x.\n",
			g_int_tptest_result, fts_test.rawdata_failed_count);
	FTS_TEST_INFO("tpd resutl && failed cout string length:0x%x.\n", i_len);

	buffer_length = (fts_test.special_buffer_length + 1) >
	    (PAGE_SIZE - i_len) ? (PAGE_SIZE - i_len - 1) : fts_test.special_buffer_length;
	FTS_TEST_INFO("tpd failed node string length:0x%x, buffer_length:0x%x.\n",
	     fts_test.special_buffer_length, buffer_length);

	if (fts_test.special_buffer != NULL && buffer_length > 0) {
		memcpy(buf + i_len, fts_test.special_buffer, buffer_length);
		buf[buffer_length + i_len] = '\0';
	}

	FTS_TEST_INFO("tpd  test:%s.\n", buf);

	num_read_chars = buffer_length + i_len;

	enable_irq(client->irq);
	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_cmd_store(struct tpd_classdev_t *cdev, const char *buf)
{
	struct i2c_client *client = fts_i2c_client;
	unsigned long command = 0;
	int retval = -1;

	FTS_TEST_DBG("%s fwname:%s.", __func__, g_str_ini_filename);
	retval = kstrtoul(buf, 10, &command);
	if (retval) {
		FTS_TEST_DBG("invalid param:%s", buf);
		return 0;
	}
	mutex_lock(&fts_input_dev->mutex);
	if (command == 1) {
		fts_test.special_buffer = NULL;

		if (fts_test.special_buffer  == NULL) {
			fts_test.special_buffer = (char *)fts_malloc(TEST_RESULT_LENGTH);
			fts_test.special_buffer_length = 0;
			fts_test.rawdata_failed_count = 0;
		}
		fts_test.temp_buffer = NULL;
		if (fts_test.temp_buffer == NULL) {
			fts_test.temp_buffer = (char *)fts_malloc(TEST_TEMP_LENGTH);
		}
		fts_test.node_opened = 1;
	} else if (command == 2) {
		if (fts_test.node_opened == 1) {
			disable_irq(client->irq);
			g_int_tptest_result = 0;
			fts_test_entry(g_str_ini_filename);
			enable_irq(client->irq);
		} else {
			FTS_TEST_DBG("command:%ld, but node not opened.", command);
		}
	} else if (command == 3) {
		fts_test.node_opened = 0;
		if (fts_test.special_buffer != NULL) {
			fts_free(fts_test.special_buffer);
			fts_test.special_buffer = NULL;
			fts_test.special_buffer_length = 0;
			fts_test.rawdata_failed_count = 0;
		}
		if (fts_test.temp_buffer != NULL) {
			fts_free(fts_test.temp_buffer);
			fts_test.temp_buffer = NULL;
		}
	} else {
		FTS_TEST_DBG("invalid command %ld", command);
	}

	mutex_unlock(&fts_input_dev->mutex);
	return 0;
}

static int tpd_test_node_data_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	int iLen = 0;

	mutex_lock(&fts_input_dev->mutex);
	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);
	fts_test_funcs();

	iLen = tpd_test_get_tp_node_data(g_node_data_type, buf, 4096);

	num_read_chars = iLen;
	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static int tpd_test_node_data_store(struct tpd_classdev_t *cdev,
				    const char *buf)
{
	int data_type = 0;
	int ret = 0;

	ret =  kstrtoint(buf, 10, &data_type);

	FTS_TEST_DBG("%s type:%d , ret is %d.", __func__, data_type, ret);

	mutex_lock(&fts_input_dev->mutex);

	g_node_data_type = data_type;

	mutex_unlock(&fts_input_dev->mutex);

	return 0;
}

static int tpd_test_channel_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;
	int buffer[5] = { 0, 0, 0, 0, 0 };

	init_i2c_write_func(fts_test_i2c_write);
	init_i2c_read_func(fts_test_i2c_read);
	fts_test_funcs();

	tpd_test_get_channel_setting(&buffer[0]);

	num_read_chars =
	    snprintf(buf, PAGE_SIZE, "%d %d %d %d %d", buffer[0], buffer[1],
		     buffer[2], buffer[3], buffer[4]);

	return num_read_chars;
}

static int tpd_test_result_show(struct tpd_classdev_t *cdev, char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "0x%x", g_int_tptest_result);

	return num_read_chars;
}

static DEVICE_ATTR(fts_test, S_IRUGO | S_IWUSR, fts_test_show, fts_test_store);

/* add your attr in here*/
static struct attribute *fts_test_attributes[] = {
	&dev_attr_fts_test.attr,

	NULL
};

static struct attribute_group fts_test_attribute_group = {
	.attrs = fts_test_attributes
};

int fts_test_init(struct i2c_client *client)
{
	int err = 0;

	FTS_TEST_FUNC_ENTER();

	FTS_TEST_INFO("[focal] %s ", IC_TEST_VERSION);	/*show version*/

	strlcpy(g_str_save_file_path, FTS_INI_FILE_PATH, 256);
	strlcpy(g_str_ini_file_path, FTS_INI_FILE_PATH, 256);
	strlcpy(g_str_ini_filename, "test.ini", 128);
	err = sysfs_create_group(&client->dev.kobj, &fts_test_attribute_group);
	if (err != 0) {
		FTS_TEST_ERROR("[focal] %s() - ERROR: sysfs_create_group() failed.",   __func__);
		goto CREAT_GROUP_ERR;
	} else {
		FTS_TEST_DBG("[focal] %s() - sysfs_create_group() succeeded.",  __func__);
	}
	/* zhangjian add tp test */
	tpd_fw_cdev.tpd_test_set_save_filepath = tpd_test_save_file_path_store;
	tpd_fw_cdev.tpd_test_get_save_filepath = tpd_test_save_file_path_show;
	tpd_fw_cdev.tpd_test_set_ini_filepath = tpd_test_ini_file_path_store;
	tpd_fw_cdev.tpd_test_get_ini_filepath = tpd_test_ini_file_path_show;
	tpd_fw_cdev.tpd_test_set_filename = tpd_test_filename_store;
	tpd_fw_cdev.tpd_test_get_filename = tpd_test_filename_show;
	tpd_fw_cdev.tpd_test_set_cmd = tpd_test_cmd_store;
	tpd_fw_cdev.tpd_test_get_cmd = tpd_test_cmd_show;
	tpd_fw_cdev.tpd_test_set_node_data_type = tpd_test_node_data_store;
	tpd_fw_cdev.tpd_test_get_node_data = tpd_test_node_data_show;
	tpd_fw_cdev.tpd_test_get_channel_info = tpd_test_channel_show;
	tpd_fw_cdev.tpd_test_get_result = tpd_test_result_show;
	/*  add end */
	FTS_TEST_FUNC_EXIT();
	return err;
CREAT_GROUP_ERR:
	return -EIO;
}

int fts_test_exit(struct i2c_client *client)
{
	FTS_TEST_FUNC_ENTER();
	sysfs_remove_group(&client->dev.kobj, &fts_test_attribute_group);

	FTS_TEST_FUNC_EXIT();
	return 0;
}
