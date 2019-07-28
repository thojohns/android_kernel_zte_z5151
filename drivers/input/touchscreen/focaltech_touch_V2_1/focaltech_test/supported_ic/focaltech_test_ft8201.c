/************************************************************************
* Copyright (C) 2012-2017, Focaltech Systems (R),All Rights Reserved.
*
* File Name: focaltech_test_ft8201.c
*
* Author: LiuWeiGuang
*
* Created: 2017-08-08
*
* Abstract: test item for ft8201
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include "../focaltech_test.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/* Reg ft8201 */
#define DEVIDE_MODE_ADDR                0x00
#define REG_LINE_NUM                    0x01
#define REG_TX_NUM                      0x02
#define REG_RX_NUM                      0x03
#define FT8201_LEFT_KEY_REG             0X1E
#define FT8201_RIGHT_KEY_REG            0X1F
#define REG_CbAddrH                     0x18
#define REG_CbAddrL                     0x19
#define REG_OrderAddrH                  0x1A
#define REG_OrderAddrL                  0x1B
#define REG_RawBuf0                     0x6A
#define REG_RawBuf1                     0x6B
#define REG_OrderBuf0                   0x6C
#define REG_CbBuf0                      0x6E
#define REG_K1Delay                     0x31
#define REG_K2Delay                     0x32
#define REG_SCChannelCf                 0x34
#define REG_CLB                          0x04
#define DOUBLE_TX_NUM_MAX                160
#define DOUBLE_RX_NUM_MAX   160
#define REG_8201_LCD_NOISE_FRAME         0X12
#define REG_8201_LCD_NOISE_START         0X11
#define REG_8201_LCD_NOISE_NUMBER        0X13

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
enum CS_TYPE {
	CS_TWO_SINGLE_MASTER = 1,	/* double chip master */
	CS_TWO_SINGLE_SLAVE = 0,	/* double chip slave */
	CS_SINGLE_CHIP = 3,	/* signel chip */
};
enum CS_DIRECTION {
	CS_LEFT_RIGHT = 0x00,	/* left right direction */
	CS_UP_DOWN = 0x40,	/* up down direction */
};
enum CD_S0_ROLE {
	CS_S0_AS_MASTER = 0x00,	/* S0 as master */
	CS_S0_AS_SLAVE = 0x80,	/* S0 as slave */
};
struct cs_info_packet {
	enum CS_TYPE cs_type;	/* chip type */
	enum CS_DIRECTION cs_direction;	/* chip direction */
	enum CD_S0_ROLE cs_s0_role;	/* S0 is master or slave */
	u8 cs_master_addr;	/* Master IIC Address */
	u8 cs_slave_addr;	/* Slave IIC Address */
	u8 cs_master_tx;	/* Master Tx */
	u8 cs_master_rx;	/* Master Rx */
	u8 cs_slave_tx;		/* Slave Tx */
	u8 cs_slave_rx;		/* Slave Rx */
};
struct ft8201 {
	struct cs_info_packet *cs_info;
	unsigned char current_slave_addr;

};
struct cs_chip_addr_mgr {
	struct ft8201 *m_parent;
	u8 slave_addr;
};
struct ft8201_test_item {
	bool rawdata_test;
	bool cb_test;
	bool short_test;
	bool lcd_noise_test;
	bool open_test;
};
struct ft8201_basic_threshold {
	int rawdata_test_min;
	int rawdata_test_max;
	bool cb_test_va_check;
	int cb_test_min;
	int cb_test_max;
	bool cb_test_vk_check;
	int cb_test_min_vk;
	int cb_test_max_vk;
	int short_res_min;
	int short_res_vk_min;
	int lcd_noise_test_frame;
	int lcd_noise_test_max_screen;
	int lcd_noise_test_max_frame;
	int lcd_noise_coefficient;
	int lcd_noise_coefficient_key;
	int open_test_min;
};
enum test_item_ft8201 {
	CODE_FT8201_ENTER_FACTORY_MODE = 0,	/* All IC are required to test items */
	CODE_FT8201_RAWDATA_TEST = 7,
	CODE_FT8201_CHANNEL_NUM_TEST = 8,
	CODE_FT8201_CB_TEST = 12,
	CODE_FT8201_SHORT_CIRCUIT_TEST = 14,
	CODE_FT8201_LCD_NOISE_TEST = 15,
	CODE_FT8201_OPEN_TEST = 25,
};
/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
struct ft8201_test_item ft8201_item;
struct ft8201_basic_threshold ft8201_basic_thr;

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static unsigned char read_raw_data(unsigned char freq, unsigned char line_num, int byte_num, int *rev_buffer);
static unsigned char ft8201_get_raw_data(struct ft8201 *g_ft8201, int *data);
void cat_single_to_one_screen(struct ft8201 *g_ft8201, int *buffer_master, int *buffer_slave,
			      int *buffer_cated);
void release_cs_info(struct ft8201 *g_ft8201);
unsigned char ft8201_enter_factory(struct ft8201 *g_ft8201);
unsigned char ft8201_chip_clb(struct ft8201 *g_ft8201);
static unsigned char ft8201_get_tx_rx_cb(struct ft8201 *g_ft8201, unsigned short start_node,
					 unsigned short read_num, int *read_buffer);
unsigned char ft8201_weakshort_get_adcdata(struct ft8201 *g_ft8201, int all_adc_data_len, int *rev_buffer);
unsigned char ft8201_enter_work(struct ft8201 *g_ft8201);
int ft8201_set_slave_addr(struct ft8201 *g_ft8201, u8 slave_addr);
unsigned char ft8201_write_reg(struct ft8201 *g_ft8201, unsigned char reg_addr, unsigned char reg_data);
bool chip_has_two_heart(struct ft8201 *g_ft8201);
unsigned char ft8201_start_scan(struct ft8201 *g_ft8201);
unsigned char ft8201_read_raw_data(struct ft8201 *g_ft8201, unsigned char freq, unsigned char line_num,
				   int byte_num, int *rev_buffer);
unsigned char weakshort_start_adc_scan(void);
unsigned char weakshort_get_scan_result(void);
unsigned char weakshort_get_adc_result(int all_adc_data_len, int *rev_buffer);
void init_current_address(struct ft8201 *g_ft8201);
void cs_info_packet(struct cs_info_packet *cspt);
static int ft8201_get_key_num(void);
static int ft8201_enter_factory_mode(struct ft8201 *g_ft8201);
static int ft8201_rawdata_test(struct ft8201 *g_ft8201, bool *test_result);
static int ft8201_cb_test(struct ft8201 *g_ft8201, bool *test_result);
static int ft8201_short_test(struct ft8201 *g_ft8201, bool *test_result);
static int ft8201_lcdnoise_test(struct ft8201 *g_ft8201, bool *test_result);
static int ft8201_open_test(struct ft8201 *g_ft8201, bool *test_result);

/************************************************************************
* Name: start_test_ft8201
* Brief:  Test entry. Determine which test item to test
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/

bool start_test_ft8201(void)
{
	bool test_result = true, temp_result = 1;
	int ret;
	int item_count = 0;
	struct ft8201 g_ft8201;
	struct cs_info_packet cspt;

	g_ft8201.cs_info = fts_malloc(sizeof(struct cs_info_packet));
	cs_info_packet(&cspt);
	init_current_address(&g_ft8201);
	g_ft8201.cs_info = &cspt;

	/* --------------2. test item */
	if (test_data.test_num == 0)
		test_result = false;

	/* Testing process, the order of the test_data.test_item structure of the test items */
	for (item_count = 0; item_count < test_data.test_num; item_count++) {
		test_data.test_item_code = test_data.test_item[item_count].itemcode;

		/* FT8201_ENTER_FACTORY_MODE */
		if (test_data.test_item[item_count].itemcode == CODE_FT8201_ENTER_FACTORY_MODE) {
			ret = ft8201_enter_factory_mode(&g_ft8201);
			if (ret != ERROR_CODE_OK || (!temp_result)) {
				test_result = false;
				test_data.test_item[item_count].testresult = RESULT_NG;
				goto ITEM_FAIL;	/* if this item FAIL, no longer test. */
			} else
				test_data.test_item[item_count].testresult = RESULT_PASS;
		}
		/* FT8201_RAWDATA_TEST */
		if (test_data.test_item[item_count].itemcode == CODE_FT8201_RAWDATA_TEST) {
			ret = ft8201_rawdata_test(&g_ft8201, &temp_result);
			if (ret != ERROR_CODE_OK || (!temp_result)) {
				test_result = false;
				test_data.test_item[item_count].testresult = RESULT_NG;
				g_int_tptest_result = g_int_tptest_result | TEST_BEYOND_MAX_LIMIT
				    | TEST_BEYOND_MIN_LIMIT;
			} else
				test_data.test_item[item_count].testresult = RESULT_PASS;
		}
		/* FT8201_CB_TEST */
		if (test_data.test_item[item_count].itemcode == CODE_FT8201_CB_TEST) {
			ret = ft8201_cb_test(&g_ft8201, &temp_result);
			if (ret != ERROR_CODE_OK || (!temp_result)) {
				test_result = false;
				test_data.test_item[item_count].testresult = RESULT_NG;
				g_int_tptest_result = g_int_tptest_result | TEST_BEYOND_MAX_LIMIT
				    | TEST_BEYOND_MIN_LIMIT;
			} else
				test_data.test_item[item_count].testresult = RESULT_PASS;
		}
		/* FT8201_SHORT_CIRCUIT_TEST */
		if (test_data.test_item[item_count].itemcode == CODE_FT8201_SHORT_CIRCUIT_TEST) {
			ret = ft8201_short_test(&g_ft8201, &temp_result);
			if (ret != ERROR_CODE_OK || (!temp_result)) {
				test_result = false;
				test_data.test_item[item_count].testresult = RESULT_NG;
				g_int_tptest_result = g_int_tptest_result | TEST_GT_SHORT;
			} else
				test_data.test_item[item_count].testresult = RESULT_PASS;
		}
		/* FT8201_LCD_NOISE_TEST */
		if (test_data.test_item[item_count].itemcode == CODE_FT8201_LCD_NOISE_TEST) {
			ret = ft8201_lcdnoise_test(&g_ft8201, &temp_result);
			if (ret != ERROR_CODE_OK || (!temp_result)) {
				test_result = false;
				test_data.test_item[item_count].testresult = RESULT_NG;
				g_int_tptest_result = g_int_tptest_result | TEST_BEYOND_MAX_LIMIT
				    | TEST_BEYOND_MIN_LIMIT;
			} else
				test_data.test_item[item_count].testresult = RESULT_PASS;
		}
		/* FT8201_OPEN_TEST */
		if (test_data.test_item[item_count].itemcode == CODE_FT8201_OPEN_TEST) {
			ret = ft8201_open_test(&g_ft8201, &temp_result);
			if (ret != ERROR_CODE_OK || (!temp_result)) {
				test_result = false;
				test_data.test_item[item_count].testresult = RESULT_NG;
				g_int_tptest_result = g_int_tptest_result | TEST_GT_OPEN;
			} else
				test_data.test_item[item_count].testresult = RESULT_PASS;
		}
	}
ITEM_FAIL:
	if (g_ft8201.cs_info != NULL) {
		fts_free(g_ft8201.cs_info);
	}
	/* --------------4. return result */
	return test_result;

}

/************************************************************************
* Name:
* Brief:
* Input:
* Output:
* Return:
***********************************************************************/
void init_current_address(struct ft8201 *g_ft8201)
{
	g_ft8201->current_slave_addr = 0x70;
}

struct cs_info_packet *get_cs_info(struct ft8201 *g_ft8201)
{
	return g_ft8201->cs_info;
}

unsigned char get_current_addr(struct ft8201 *g_ft8201)
{
	return g_ft8201->current_slave_addr;
}

int get_cs_type(struct cs_info_packet *cspt)
{
	return cspt->cs_type;
}

int get_cs_direction(struct cs_info_packet *cspt)
{
	return cspt->cs_direction;
}

int get_s0_role(struct cs_info_packet *cspt)
{
	return cspt->cs_s0_role;
}

void set_master_addr(struct cs_info_packet *cspt, u8 master)
{
	cspt->cs_master_addr = master;
}

u8 get_master_addr(struct cs_info_packet *cspt)
{
	return cspt->cs_master_addr;
}

void set_slave_addr(struct cs_info_packet *cspt, u8 slave)
{
	cspt->cs_slave_addr = slave;
}

u8 get_slave_addr(struct cs_info_packet *cspt)
{
	return cspt->cs_slave_addr;
}

void set_master_tx(struct cs_info_packet *cspt, u8 tx)
{
	cspt->cs_master_tx = tx;
}

u8 get_master_tx(struct cs_info_packet *cspt)
{
	return cspt->cs_master_tx;
}

void set_master_rx(struct cs_info_packet *cspt, u8 rx)
{
	cspt->cs_master_rx = rx;
}

u8 get_master_rx(struct cs_info_packet *cspt)
{
	return cspt->cs_master_rx;
}

void set_slave_tx(struct cs_info_packet *cspt, u8 tx)
{
	cspt->cs_slave_tx = tx;
}

u8 get_slave_tx(struct cs_info_packet *cspt)
{
	return cspt->cs_slave_tx;
}

void set_slave_rx(struct cs_info_packet *cspt, u8 rx)
{
	cspt->cs_slave_rx = rx;
}

u8 get_slave_rx(struct cs_info_packet *cspt)
{
	return cspt->cs_slave_rx;
}

void initialize(struct cs_info_packet *cspt, u8 reg_cfg)
{
	cspt->cs_type = (enum CS_TYPE)(reg_cfg & 0x3F);
	cspt->cs_direction = (enum CS_DIRECTION)(reg_cfg & 0x40);
	cspt->cs_s0_role = (enum CD_S0_ROLE)(reg_cfg & 0x80);
	cspt->cs_master_addr = 0x70;
	cspt->cs_slave_addr = 0x72;
	cspt->cs_master_tx = 0;
	cspt->cs_master_rx = 0;
	cspt->cs_slave_tx = 0;
	cspt->cs_slave_rx = 0;
}

void cs_info_packet(struct cs_info_packet *cspt)
{
	cspt->cs_type = CS_SINGLE_CHIP;
	cspt->cs_direction = CS_UP_DOWN;
	cspt->cs_s0_role = CS_S0_AS_MASTER;
	cspt->cs_master_addr = 0x72;
	cspt->cs_slave_addr = 0x70;
	cspt->cs_master_tx = 0;
	cspt->cs_master_rx = 0;
	cspt->cs_slave_tx = 0;
	cspt->cs_slave_rx = 0;
}

void release_cs_info(struct ft8201 *g_ft8201)
{
	memset(g_ft8201->cs_info, 0, sizeof(struct cs_info_packet));
}

void update_cs_info(struct ft8201 *g_ft8201)
{
	int ret = ERROR_CODE_OK;
	u8 RegValue = 0;

	ret = read_reg(0x26, &RegValue);
	initialize(g_ft8201->cs_info, RegValue);

	ret = read_reg(0x50, &RegValue);
	set_master_tx(g_ft8201->cs_info, RegValue);

	ret = read_reg(0x51, &RegValue);
	set_master_rx(g_ft8201->cs_info, RegValue);

	ret = read_reg(0x52, &RegValue);
	set_slave_tx(g_ft8201->cs_info, RegValue);

	ret = read_reg(0x53, &RegValue);
	set_slave_rx(g_ft8201->cs_info, RegValue);

	ret = write_reg(0x17, 0);
	ret = read_reg(0x81, &RegValue);
	set_master_addr(g_ft8201->cs_info, RegValue);

	ret = write_reg(0x17, 12);
	ret = read_reg(0x81, &RegValue);
	set_slave_addr(g_ft8201->cs_info, RegValue);

	FTS_TEST_DBG("cs_type = %d, cs_direction = %d, cs_s0_role = %d, cs_master_addr = 0x%x\n",
		     g_ft8201->cs_info->cs_type, g_ft8201->cs_info->cs_direction,
		     g_ft8201->cs_info->cs_s0_role, g_ft8201->cs_info->cs_master_addr);

	FTS_TEST_DBG("cs_slave_addr =  0x%x, cs_master_tx = %d, cs_master_rx = %d\n",
		     g_ft8201->cs_info->cs_slave_addr, g_ft8201->cs_info->cs_master_tx,
		     g_ft8201->cs_info->cs_master_rx);
	FTS_TEST_DBG("cs_slave_tx = %d, cs_slave_rx = %d.\n",
		     g_ft8201->cs_info->cs_slave_tx,  g_ft8201->cs_info->cs_slave_rx);

}

bool chip_has_two_heart(struct ft8201 *g_ft8201)
{
	if (get_cs_type(g_ft8201->cs_info) == CS_SINGLE_CHIP)
		return false;

	return true;
}

void cs_chip_addr_mgr_init(struct cs_chip_addr_mgr *mgr, struct ft8201 *parent)
{

	mgr->m_parent = parent;
	mgr->slave_addr = get_current_addr(parent);

}

void cs_chip_addr_mgr_exit(struct cs_chip_addr_mgr *mgr)
{
	if (mgr->slave_addr != get_current_addr(mgr->m_parent)) {

		ft8201_set_slave_addr(mgr->m_parent, mgr->slave_addr);
	}
}

int ft8201_set_slave_addr(struct ft8201 *g_ft8201, u8 slave_addr)
{
	unsigned char value = 0;
	int ret = ERROR_CODE_OK;
	int tmp = 0;

	g_ft8201->current_slave_addr = slave_addr;
	tmp = fts_data->client->addr;

	FTS_TEST_INFO("Original i2c addr 0x%x ", fts_data->client->addr);
	FTS_TEST_INFO("CurrentAddr 0x%x ", (slave_addr >> 1));

	if (fts_data->client->addr != (slave_addr >> 1)) {
		fts_data->client->addr = (slave_addr >> 1);
		FTS_TEST_INFO("Change i2c addr 0x%x to 0x%x", tmp, fts_data->client->addr);
	}
	/* debug start */
	ret = read_reg(0x20, &value);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_INFO("[focal] read_reg Error, code: %d ", ret);
	} else {
		FTS_TEST_INFO("[focal] read_reg successed, Addr: 0x20, value: 0x%02x ", value);
	}
	/* debug end */

	return 0;
}

void work_as_master(struct cs_chip_addr_mgr *mgr)
{
	struct cs_info_packet *pcs = get_cs_info(mgr->m_parent);

	if (get_master_addr(pcs) != get_current_addr(mgr->m_parent)) {
		ft8201_set_slave_addr(mgr->m_parent, get_master_addr(pcs));
	}

}

void work_as_slave(struct cs_chip_addr_mgr *mgr)
{
	struct cs_info_packet *pcs = get_cs_info(mgr->m_parent);

	if (get_slave_addr(pcs) != get_current_addr(mgr->m_parent)) {
		ft8201_set_slave_addr(mgr->m_parent, get_slave_addr(pcs));
	}
}

void cat_single_to_one_screen(struct ft8201 *g_ft8201, int *buffer_master, int *buffer_slave,
			      int *buffer_cated)
{
	int left_channel_num = 0, right_channel_num = 0;
	int up_channel_num = 0, down_channel_num = 0;
	unsigned char split_rx = 0;
	unsigned char split_tx = 0;
	int row = 0, col = 0;
	int relative_rx = 0;
	int relative_tx = 0;
	int total_rx = 0;
	int total_tx = 0;
	int *buffer_left = NULL;
	int *buffer_right = NULL;
	int *buffer_up = NULL;
	int *buffer_down = NULL;

	FTS_TEST_FUNC_ENTER();

	/* make sure chip is double chip or not */
	if (get_cs_type(g_ft8201->cs_info) == CS_SINGLE_CHIP)
		return;

	/* left right direction */
	if (get_cs_direction(g_ft8201->cs_info) == CS_LEFT_RIGHT) {

		if (get_s0_role(g_ft8201->cs_info) == CS_S0_AS_MASTER) {
			split_rx = get_master_rx(g_ft8201->cs_info);
			buffer_left = buffer_master;
			buffer_right = buffer_slave;
			left_channel_num =
			get_master_tx(g_ft8201->cs_info) * get_master_rx(g_ft8201->cs_info);
			right_channel_num = get_slave_tx(g_ft8201->cs_info) * get_slave_rx(g_ft8201->cs_info);
		} else {
			split_rx = get_slave_rx(g_ft8201->cs_info);
			buffer_left = buffer_slave;
			buffer_right = buffer_master;
			left_channel_num = get_slave_tx(g_ft8201->cs_info) * get_slave_rx(g_ft8201->cs_info);
			right_channel_num =
			get_master_tx(g_ft8201->cs_info) * get_master_rx(g_ft8201->cs_info);
		}

		for (row = 0; row < get_master_tx(g_ft8201->cs_info); ++row) {
			for (col = 0;
			     col < get_master_rx(g_ft8201->cs_info) + get_slave_rx(g_ft8201->cs_info);
			     ++col) {
				relative_rx = col - split_rx;
				total_rx = get_master_rx(g_ft8201->cs_info) + get_slave_rx(g_ft8201->cs_info);
				if (col >= split_rx) {
					/* right data */
					buffer_cated[row * total_rx + col] =
					    buffer_right[row * (total_rx - split_rx) + relative_rx];
				} else {
					/* left data */
					buffer_cated[row * total_rx + col] =
					    buffer_left[row * split_rx + col];
				}
			}
		}

		memcpy(buffer_cated +
		       get_master_tx(g_ft8201->cs_info) * (get_master_rx(g_ft8201->cs_info) +
							   get_slave_rx(g_ft8201->cs_info)),
		       buffer_left + left_channel_num, 6);
		memcpy(buffer_cated +
		       get_master_tx(g_ft8201->cs_info) * (get_master_rx(g_ft8201->cs_info) +
							   get_slave_rx(g_ft8201->cs_info)) + 6,
		       buffer_right + right_channel_num, 6);
	}

	else if (get_cs_direction(g_ft8201->cs_info) == CS_UP_DOWN) {
		if (get_s0_role(g_ft8201->cs_info) == CS_S0_AS_MASTER) {
			split_tx = get_master_tx(g_ft8201->cs_info);
			buffer_up = buffer_master;
			buffer_down = buffer_slave;
			up_channel_num = get_master_tx(g_ft8201->cs_info) * get_master_rx(g_ft8201->cs_info);
			down_channel_num = get_slave_tx(g_ft8201->cs_info) * get_slave_rx(g_ft8201->cs_info);
		} else {
			split_tx = get_slave_tx(g_ft8201->cs_info);
			buffer_up = buffer_slave;
			buffer_down = buffer_master;
			up_channel_num = get_slave_tx(g_ft8201->cs_info) * get_slave_rx(g_ft8201->cs_info);
			down_channel_num =
			    get_master_tx(g_ft8201->cs_info) * get_master_rx(g_ft8201->cs_info);
		}
		for (row = 0; row < get_master_tx(g_ft8201->cs_info) + get_slave_tx(g_ft8201->cs_info); ++row) {
			for (col = 0; col < get_master_rx(g_ft8201->cs_info); ++col) {
				relative_tx = row - split_tx;
				total_tx = get_master_tx(g_ft8201->cs_info) + get_slave_tx(g_ft8201->cs_info);
				if (row >= split_tx) {
					/* down data */
					buffer_cated[row * get_master_rx(g_ft8201->cs_info) + col] =
					    buffer_down[relative_tx * get_master_rx(g_ft8201->cs_info) + col];
				} else {
					/* up data */
					buffer_cated[row * get_master_rx(g_ft8201->cs_info) + col] =
					    buffer_up[row * get_master_rx(g_ft8201->cs_info) + col];
				}
			}
		}

		FTS_TEST_INFO("===liu=== 592");
		memcpy(buffer_cated +
		       (get_master_tx(g_ft8201->cs_info) +
			get_slave_tx(g_ft8201->cs_info)) * get_master_rx(g_ft8201->cs_info),
		       buffer_up + up_channel_num, 6);
		memcpy(buffer_cated +
		       (get_master_tx(g_ft8201->cs_info) +
			get_slave_tx(g_ft8201->cs_info)) * get_master_rx(g_ft8201->cs_info) + 6,
		       buffer_down + down_channel_num, 6);
	}

	FTS_TEST_FUNC_EXIT();

}

/************************************************************************
* Name: ft8201_enter_factory_mode
* Brief:  Check whether TP can enter Factory Mode, and do some thing
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static int ft8201_enter_factory_mode(struct ft8201 *g_ft8201)
{

	int ret = 0;

	ret = ft8201_enter_factory(g_ft8201);
	if (ret < 0) {
		FTS_TEST_SAVE_INFO("enter factory mode fail, can't get tx/rx num");
		return ret;
	}
	ret = ft8201_get_key_num();
	if (ret < 0) {
		FTS_TEST_SAVE_INFO("get key num fail");
		return ret;
	}

	return ret;

}

/************************************************************************
* Name: ft8201_rawdata_test
* Brief:  TestItem: RawDataTest. Check if MCAP RawData is within the range.
* Input: test_result
* Output: test_result, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static int ft8201_rawdata_test(struct ft8201 *g_ft8201, bool *test_result)
{
	int ret;
	bool tmp_result = true;
	int i = 0;
	int *rawdata = NULL;

	FTS_TEST_SAVE_INFO("\n\n==============================Test Item: -------- Raw Data Test\n");

	memset(test_data.buffer, 0,
	       ((test_data.screen_param.tx_num + 1) * test_data.screen_param.rx_num) * sizeof(int));
	rawdata = test_data.buffer;

	ret = ft8201_enter_factory(g_ft8201);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("\r\n\r\n Failed to Enter factory mode. Error Code: %d", ret);
		return ret;
	}
	/* ----------------------------------------------------------Read RawData */
	for (i = 0; i < 3; i++) {	/* Lost 3 Frames, In order to obtain stable data */
		ret = ft8201_get_raw_data(g_ft8201, rawdata);
	}

	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("Failed to get Raw Data!! Error Code: %d", ret);
		return ret;
	}
	/* ----------------------------------------------------------Show RawData */
	show_data_incell(rawdata, true);

	/* ----------------------------------------------------------To Determine RawData if in Range or not */
	tmp_result =
	    compare_detailthreshold_data_incell(rawdata, test_data.incell_detail_thr.rawdata_test_min,
						test_data.incell_detail_thr.rawdata_test_max, true);

	/* Save Test Data */
	save_testdata_incell(rawdata, "RawData Test", 0, test_data.screen_param.tx_num + 1,
			     test_data.screen_param.rx_num, 1);
	/* ----------------------------------------------------------Return Result */
	if (tmp_result) {
		*test_result = true;
		FTS_TEST_SAVE_INFO("\n\nRawData Test is OK!\r\n");
	} else {
		*test_result = false;
		FTS_TEST_SAVE_INFO("\n\nRawData Test is NG!\r\n");
	}
	return ret;
}

/************************************************************************
* Name: ft8201_cb_test
* Brief:  TestItem: Cb Test. Check if Cb is within the range.
* Input: none
* Output: test_result, PASS or FAIL
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static int ft8201_cb_test(struct ft8201 *g_ft8201, bool *test_result)
{
	bool tmp_result = true;
	int ret = ERROR_CODE_OK;
	int i = 0;
	bool include_key = false;
	int *cbdata = NULL;

	include_key = ft8201_basic_thr.cb_test_vk_check;
	FTS_TEST_SAVE_INFO("\n\n==============================Test Item: --------  CB Test\n");

	memset(test_data.buffer, 0,
	       ((test_data.screen_param.tx_num + 1) * test_data.screen_param.rx_num) * sizeof(int));
	cbdata = test_data.buffer;

	ret = ft8201_enter_factory(g_ft8201);
	if (ret != ERROR_CODE_OK) {
		tmp_result = false;
		FTS_TEST_SAVE_INFO("\r\n\r\n Failed to Enter factory mode. Error Code: %d", ret);
		goto TEST_ERR;
	}

	for (i = 0; i < 10; i++) {
		ret = ft8201_chip_clb(g_ft8201);
		sys_delay(50);
		if (ret == ERROR_CODE_OK) {
			break;
		}
	}
	if (i == 10) {
		FTS_TEST_SAVE_INFO("\r\nReCalib Failed\r\n");
		tmp_result = false;
	}

	ret =
	    ft8201_get_tx_rx_cb(g_ft8201, 0,
				(short)(test_data.screen_param.tx_num * test_data.screen_param.rx_num +
					test_data.screen_param.key_num_total), cbdata);
	if (ret != ERROR_CODE_OK) {
		tmp_result = false;
		FTS_TEST_SAVE_INFO("\nFailed to get CB value...\n");
		goto TEST_ERR;
	}
	/* ------------------------------------------------Show CbData */
	show_data_incell(cbdata, include_key);

	/* ----------------------------------------------------------To Determine RawData if in Range or not */
	tmp_result =
	    compare_detailthreshold_data_incell(cbdata, test_data.incell_detail_thr.cb_test_min,
						test_data.incell_detail_thr.cb_test_max, include_key);

	/* Save Test Data */
	save_testdata_incell(cbdata, "CB Test", 0, test_data.screen_param.tx_num + 1,
			     test_data.screen_param.rx_num, 1);
	if (tmp_result) {
		*test_result = true;
		FTS_TEST_SAVE_INFO("\n\nCB Test is OK!\r\n");
	} else {
		*test_result = false;
		FTS_TEST_SAVE_INFO("\n\nCB Test is NG!\r\n");
	}

	return ret;

TEST_ERR:

	*test_result = false;
	FTS_TEST_SAVE_INFO("\n\nCB Test is NG!\n\n");
	return ret;
}

static int ft8201_short_test(struct ft8201 *g_ft8201, bool *test_result)
{
	int ret = ERROR_CODE_OK;
	bool temp_result = true;
	int res_min = 0;
	int vk_res_min = 0;
	int all_adc_data_num = 0;
	unsigned char tx_num = 0, rx_num = 0, channel_num = 0;
	int row = 0, col = 0;
	int tmp_adc = 0;
	int value_min = 0;
	int vk_value_min = 0;
	int value_max = 0;
	int i = 0;
	int *adcdata = NULL;

	FTS_TEST_SAVE_INFO
	    ("\r\n\r\n==============================Test Item: -------- Short Circuit Test \r\n");

	memset(test_data.buffer, 0,
	       ((test_data.screen_param.tx_num + 1) * test_data.screen_param.rx_num) * sizeof(int));
	adcdata = test_data.buffer;

	ret = ft8201_enter_factory(g_ft8201);
	if (ret != ERROR_CODE_OK) {
		temp_result = false;
		FTS_TEST_SAVE_INFO("\r\n\r\n Failed to Enter factory mode. Error Code: %d", ret);
		goto TEST_END;
	}

	res_min = ft8201_basic_thr.short_res_min;
	vk_res_min = ft8201_basic_thr.short_res_vk_min;
	ret = read_reg(0x02, &tx_num);
	ret = read_reg(0x03, &rx_num);
	if (ret != ERROR_CODE_OK) {
		temp_result = false;
		FTS_TEST_SAVE_INFO("\r\n\r\n Failed to read reg. Error Code: %d", ret);
		goto TEST_END;
	}

	channel_num = tx_num + rx_num;
	all_adc_data_num = tx_num * rx_num + test_data.screen_param.key_num_total;

	for (i = 0; i < 1; i++) {
		ret = ft8201_weakshort_get_adcdata(g_ft8201, all_adc_data_num * 2, adcdata);
		sys_delay(50);
		if (ret != ERROR_CODE_OK) {
			temp_result = false;
			FTS_TEST_SAVE_INFO("\r\n\r\n Failed to get AdcData. Error Code: %d", ret);
			goto TEST_END;
		}
	}

	/* show ADCData */
#if 0
	FTS_TEST_SAVE_INFO("ADCData:\n");
	for (i = 0; i < all_adc_data_num; i++) {
		FTS_TEST_SAVE_INFO("%-4d  ", adcdata[i]);
		if (0 == (i + 1) % rx_num) {
			FTS_TEST_SAVE_INFO("\n");
		}
	}
	FTS_TEST_SAVE_INFO("\n");
#endif

	for (row = 0; row < test_data.screen_param.tx_num + 1; ++row) {
		for (col = 0; col < test_data.screen_param.rx_num; ++col) {
			tmp_adc = adcdata[row * rx_num + col];
			if (tmp_adc > 4050)
				tmp_adc = 4050;
			adcdata[row * rx_num + col] = (tmp_adc * 100) / (4095 - tmp_adc);
		}
	}

	show_data_incell(adcdata, true);

	/* analyze */
	value_min = res_min;
	vk_value_min = vk_res_min;
	value_max = 100000000;
	FTS_TEST_SAVE_INFO("\nShort Circuit test , VA_Set_Range=(%d, %d), VK_Set_Range=(%d, %d)\n",
			   value_min, value_max, vk_value_min, value_max);

	temp_result = compare_data_incell(adcdata, value_min, value_max, vk_value_min, value_max, true);

	save_testdata_incell(adcdata, "Short Circuit Test", 0, test_data.screen_param.tx_num + 1,
			     test_data.screen_param.rx_num, 1);

TEST_END:

	if (temp_result) {
		FTS_TEST_SAVE_INFO("\r\n\r\nShort Circuit Test is OK!\r\n");
		*test_result = true;
	} else {
		FTS_TEST_SAVE_INFO("\r\n\r\nShort Circuit Test is NG!\r\n");
		*test_result = false;
	}
	return ret;
}

static int ft8201_lcdnoise_test(struct ft8201 *g_ft8201, bool *test_result)
{
	int ret = ERROR_CODE_OK;
	bool result_flag = true;
	unsigned char old_mode = 0, reg_value = 0;
	int retry = 0;
	unsigned char status = 0xff;
	int row = 0, col = 0;
	int value_min = 0;
	int value_max = 0;
	int vk_value_max = 0;
	int *lcdnoise = NULL;

	FTS_TEST_SAVE_INFO("\r\n\r\n==============================Test Item: -------- LCD Noise Test \r\n");

	memset(test_data.buffer, 0,
	       ((test_data.screen_param.tx_num + 1) * test_data.screen_param.rx_num) * sizeof(int));
	lcdnoise = test_data.buffer;

	ret = ft8201_enter_factory(g_ft8201);
	if (ret != ERROR_CODE_OK) {
		result_flag = false;
		FTS_TEST_SAVE_INFO("\r\n\r\n Failed to Enter factory mode. Error Code: %d", ret);
		goto TEST_END;
	}
	/* is differ mode */
	read_reg(0x06, &old_mode);
	/* Set the upper limit of CA filter */
	read_reg(0x5E, &reg_value);
	ft8201_write_reg(g_ft8201, 0x5E, 0x64);
	ft8201_write_reg(g_ft8201, 0x06, 1);

	/* set scan number */
	ret =
	    ft8201_write_reg(g_ft8201, REG_8201_LCD_NOISE_FRAME,
			     ft8201_basic_thr.lcd_noise_test_frame & 0xff);
	ret =
	    ft8201_write_reg(g_ft8201, REG_8201_LCD_NOISE_FRAME + 1,
			     (ft8201_basic_thr.lcd_noise_test_frame >> 8) & 0xff);

	/* set point */
	ret = ft8201_write_reg(g_ft8201, 0x01, 0xAD);

	/* start lcd noise test */
	ret = ft8201_write_reg(g_ft8201, REG_8201_LCD_NOISE_START, 0x01);

	/* check status */
	for (retry = 0; retry < 50; ++retry) {

		ret = read_reg(REG_8201_LCD_NOISE_START, &status);
		if (status == 0x00)
			break;
		sys_delay(500);
	}

	if (retry == 50) {
		result_flag = false;
		FTS_TEST_SAVE_INFO("\r\nScan Noise Time Out!");
		goto TEST_END;
	}

	ret =
	    read_mass_data(0x6A,
			   test_data.screen_param.tx_num * test_data.screen_param.rx_num * 2 +
			   test_data.screen_param.key_num_total * 2, lcdnoise);
	if (ret) {
		FTS_TEST_SAVE_INFO("Failed to Read Data.\n");
		goto TEST_END;
	}

	for (row = 0; row < test_data.screen_param.tx_num + 1; ++row) {
		for (col = 0; col < test_data.screen_param.rx_num; ++col) {
			lcdnoise[row * test_data.screen_param.rx_num + col] =
			    focal_abs(lcdnoise[row * test_data.screen_param.rx_num + col]);
		}
	}

	/* show lcd noise data */
	show_data_incell(lcdnoise, true);

	/* compare */
	value_min = 0;
	value_max = ft8201_basic_thr.lcd_noise_coefficient * test_data.va_touch_thr * 32 / 100;
	vk_value_max = ft8201_basic_thr.lcd_noise_coefficient_key * test_data.key_touch_thr * 32 / 100;
	result_flag = compare_data_incell(lcdnoise, value_min, value_max, value_min, vk_value_max, true);

	/* save data */
	save_testdata_incell(lcdnoise, "LCD Noise Test", 0, test_data.screen_param.tx_num + 1,
			     test_data.screen_param.rx_num, 1);

TEST_END:

	ft8201_write_reg(g_ft8201, 0x06, old_mode);
	ft8201_write_reg(g_ft8201, 0x5E, reg_value);
	sys_delay(20);
	ft8201_write_reg(g_ft8201, REG_8201_LCD_NOISE_START, 0x00);
	if (result_flag) {
		FTS_TEST_SAVE_INFO("\r\n\r\nLCD Noise Test is OK!\r\n");
		*test_result = true;
	} else {
		FTS_TEST_SAVE_INFO("\r\n\r\nLCD Noise Test is NG!\r\n");
		*test_result = false;
	}
	return ret;
}

static int ft8201_open_test(struct ft8201 *g_ft8201, bool *test_result)
{
	int ret = ERROR_CODE_OK;
	bool tmp_result = true;
	unsigned char ch_value = 0xff;
	unsigned char reg_data = 0xff;
	int max_value = 0;
	int min_value = 0;
	int *opendata = NULL;

	FTS_TEST_SAVE_INFO("\r\n\r\n==============================Test Item: --------  Open Test\n");

	memset(test_data.buffer, 0,
	       ((test_data.screen_param.tx_num + 1) * test_data.screen_param.rx_num) * sizeof(int));
	opendata = test_data.buffer;

	ret = ft8201_enter_factory(g_ft8201);
	sys_delay(50);
	if (ret != ERROR_CODE_OK) {
		tmp_result = false;
		FTS_TEST_SAVE_INFO("\r\n=========  Enter Factory Failed!");
		goto TEST_END;
	}

	ret = read_reg(0x20, &ch_value);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("\r\nFailed to Read Reg!\r\n ");
		tmp_result = false;
		goto TEST_END;
	}

	ret = read_reg(0x86, &reg_data);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("\r\nFailed to Read Reg!\r\n ");
		tmp_result = false;
		goto TEST_END;
	}
	/* 0x86 register write 0x01, the VREF_TP is set to 2V. */
	ret = ft8201_write_reg(g_ft8201, 0x86, 0x01);
	sys_delay(50);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("\r\nFailed to Read or Write Reg!\r\n ");
		tmp_result = false;
		goto TEST_END;
	}
	/* The 0x20 register Bit4~Bit5 is set to 2b'10 (Source to GND), the value of other bit remains unchanged */
	ret = ft8201_write_reg(g_ft8201, 0x20, ((ch_value & 0xCF) + 0x20));
	sys_delay(50);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("\r\nFailed to Read or Write Reg!\r\n ");
		tmp_result = false;
		goto TEST_END;
	}

	ret = ft8201_chip_clb(g_ft8201);
	if (ret != ERROR_CODE_OK) {
		tmp_result = false;
		FTS_TEST_SAVE_INFO("\r\n========= auto clb Failed!");
		goto TEST_END;
	}
	/* get cb data */
	ret =
	    ft8201_get_tx_rx_cb(g_ft8201, 0,
				(short)(test_data.screen_param.tx_num * test_data.screen_param.rx_num +
					test_data.screen_param.key_num_total), opendata);
	if (ret != ERROR_CODE_OK) {
		tmp_result = false;
		FTS_TEST_SAVE_INFO("\r\n\r\n=========get CB Failed!");
		goto TEST_END;
	}
	/* show open data */
	show_data_incell(opendata, false);

	/* compare */
	min_value = ft8201_basic_thr.open_test_min;
	max_value = 255;
	tmp_result = compare_data_incell(opendata, min_value, max_value, 0, 0, false);

	/* save data */
	save_testdata_incell(opendata, "Open Test", 0, test_data.screen_param.tx_num,
			     test_data.screen_param.rx_num, 1);

	ret = ft8201_write_reg(g_ft8201, 0x20, ch_value);
	sys_delay(50);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("\r\nFailed to Read or Write Reg!\r\n");
		tmp_result = false;
		goto TEST_END;
	}

	ret = ft8201_write_reg(g_ft8201, 0x86, reg_data);
	sys_delay(50);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("\r\nFailed to Read or Write Reg!\r\n");
		tmp_result = false;
		goto TEST_END;
	}

	ret = ft8201_chip_clb(g_ft8201);
	if (ret != ERROR_CODE_OK) {
		tmp_result = false;
		FTS_TEST_SAVE_INFO("\r\n========= auto clb Failed!");
		goto TEST_END;
	}

TEST_END:
	if (tmp_result) {
		*test_result = true;
		FTS_TEST_SAVE_INFO("\r\n\r\nOpen Test is OK!\r\n");
	} else {
		*test_result = false;
		FTS_TEST_SAVE_INFO("\r\n\r\nOpen Test is NG!\r\n");
	}

	return ret;

}

static int ft8201_get_key_num(void)
{
	int ret = 0;
	int i = 0;
	u8 keyval = 0;

	test_data.screen_param.key_num = 0;
	for (i = 0; i < 3; i++) {
		ret = read_reg(FT8201_LEFT_KEY_REG, &keyval);
		if (ret != 0) {
			sys_delay(150);
			continue;
		} else {
			if (((keyval >> 0) & 0x01)) {
				test_data.screen_param.left_key1 = true;
				++test_data.screen_param.key_num;
			}
			if (((keyval >> 1) & 0x01)) {
				test_data.screen_param.left_key2 = true;
				++test_data.screen_param.key_num;
			}
			if (((keyval >> 2) & 0x01)) {
				test_data.screen_param.left_key3 = true;
				++test_data.screen_param.key_num;
			}
			break;
		}
	}

	if (i >= 3) {
		FTS_TEST_SAVE_INFO("can't get left key num");
		return ret;
	}

	for (i = 0; i < 3; i++) {
		ret = read_reg(FT8201_RIGHT_KEY_REG, &keyval);
		if (ret != 0) {
			sys_delay(150);
			continue;
		} else {
			if (((keyval >> 0) & 0x01)) {
				test_data.screen_param.right_key1 = true;
				++test_data.screen_param.key_num;
			}
			if (((keyval >> 1) & 0x01)) {
				test_data.screen_param.right_key2 = true;
				++test_data.screen_param.key_num;
			}
			if (((keyval >> 2) & 0x01)) {
				test_data.screen_param.right_key3 = true;
				++test_data.screen_param.key_num;
			}
			break;
		}
	}

	if (i >= 3) {
		FTS_TEST_SAVE_INFO("can't get right key num");
		return ret;
	}

	return 0;
}

/************************************************************************
* Name: read_raw_data(Same function name as FT_MultipleTest)
* Brief:  read Raw Data
* Input: freq(No longer used, reserved), line_num, byte_num
* Output: rev_buffer
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char read_raw_data(unsigned char freq, unsigned char line_num, int byte_num, int *rev_buffer)
{
	int ret = ERROR_CODE_COMM_ERROR;
	unsigned char i2c_w_buffer[3] = { 0 };
	u8 *read_data = NULL;
	int i, read_num;
	unsigned short bytes_per_time = 0;

	read_num = byte_num / BYTES_PER_TIME;

	if ((byte_num % BYTES_PER_TIME) != 0)
		read_num++;

	if (byte_num <= BYTES_PER_TIME) {
		bytes_per_time = byte_num;
	} else {
		bytes_per_time = BYTES_PER_TIME;
	}

	ret = write_reg(REG_LINE_NUM, line_num);	/* Set row addr; */

	read_data = fts_malloc(byte_num * sizeof(u8));
	if (read_data == NULL) {
		return -ENOMEM;
	}
	/* ***********************************************************Read raw data in test mode1 */
	i2c_w_buffer[0] = REG_RawBuf0;	/* set begin address */
	if (ret == ERROR_CODE_OK) {
		sys_delay(10);
		ret = fts_i2c_read_write(i2c_w_buffer, 1, read_data, bytes_per_time);
	}

	for (i = 1; i < read_num; i++) {
		if (ret != ERROR_CODE_OK)
			break;

		if (i == read_num - 1) {	/* last packet */
			sys_delay(10);
			ret =
			    fts_i2c_read_write(NULL, 0, read_data + BYTES_PER_TIME * i,
					       byte_num - BYTES_PER_TIME * i);
		} else {
			sys_delay(10);
			ret = fts_i2c_read_write(NULL, 0, read_data + BYTES_PER_TIME * i, BYTES_PER_TIME);
		}

	}

	if (ret == ERROR_CODE_OK) {
		for (i = 0; i < (byte_num >> 1); i++) {
			rev_buffer[i] = (read_data[i << 1] << 8) + read_data[(i << 1) + 1];
		}
	}

	fts_free(read_data);

	return ret;

}

static unsigned char ft8201_get_tx_rx_cb(struct ft8201 *g_ft8201, unsigned short start_node,
					 unsigned short read_num, int *read_buffer)
{
	int ret = ERROR_CODE_OK;
	struct cs_chip_addr_mgr mgr;
	int *buffer_master = NULL;
	int *buffer_slave = NULL;
	int master_tx = get_master_tx(g_ft8201->cs_info);
	int master_rx = get_master_rx(g_ft8201->cs_info);
	int slave_tx = get_slave_tx(g_ft8201->cs_info);
	int slave_rx = get_slave_rx(g_ft8201->cs_info);

	FTS_TEST_FUNC_ENTER();

	if (chip_has_two_heart(g_ft8201)) {

		buffer_master = fts_malloc((master_tx + 1) * master_rx * sizeof(int));
		buffer_slave = fts_malloc((slave_tx + 1) * slave_rx * sizeof(int));

		cs_chip_addr_mgr_init(&mgr, g_ft8201);

		work_as_master(&mgr);
		ret = get_cb_incell(0, master_tx * master_rx + 6, buffer_master);

		work_as_slave(&mgr);
		ret = get_cb_incell(0, slave_tx * slave_rx + 6, buffer_slave);

		cat_single_to_one_screen(g_ft8201, buffer_master, buffer_slave, read_buffer);

		cs_chip_addr_mgr_exit(&mgr);

		fts_free(buffer_master);
		fts_free(buffer_slave);
	}

	FTS_TEST_FUNC_EXIT();

	return ret;
}

/************************************************************************
* Name: ft8201_get_raw_data
* Brief:  Get Raw Data of VA area and Key area
* Input: none
* Output: none
* Return: Comm Code. Code = 0x00 is OK, else fail.
***********************************************************************/
static unsigned char ft8201_get_raw_data(struct ft8201 *g_ft8201, int *data)
{
	int ret = ERROR_CODE_OK;

	FTS_TEST_FUNC_ENTER();

	/* --------------------------------------------Start Scanning */
	ret = ft8201_start_scan(g_ft8201);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("Failed to Scan ...");
		return ret;
	}
	/* --------------------------------------------Read RawData for Channel Area */
	ret =
	    ft8201_read_raw_data(g_ft8201, 0, 0xAD,
				 test_data.screen_param.tx_num * test_data.screen_param.rx_num * 2, data);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("Failed to Get RawData");
		return ret;
	}
	/* --------------------------------------------Read RawData for Key Area */
	ret =
	    ft8201_read_raw_data(g_ft8201, 0, 0xAE, test_data.screen_param.key_num_total * 2,
				 data + test_data.screen_param.tx_num * test_data.screen_param.rx_num);
	if (ret != ERROR_CODE_OK) {
		FTS_TEST_SAVE_INFO("Failed to Get RawData");
		return ret;
	}

	FTS_TEST_FUNC_EXIT();

	return ret;

}

unsigned char ft8201_read_raw_data(struct ft8201 *g_ft8201, unsigned char freq, unsigned char line_num,
				   int byte_num, int *rev_buffer)
{
	int ret = ERROR_CODE_OK;

	FTS_TEST_FUNC_ENTER();

	if (chip_has_two_heart(g_ft8201)) {
		/* Read rawdata hand over to the master  to complete it. */
		struct cs_chip_addr_mgr mgr;

		cs_chip_addr_mgr_init(&mgr, g_ft8201);

		work_as_master(&mgr);
		ret = read_raw_data(freq, line_num, byte_num, rev_buffer);

		cs_chip_addr_mgr_exit(&mgr);
	}

	FTS_TEST_FUNC_EXIT();

	return ret;
}

unsigned char ft8201_enter_factory(struct ft8201 *g_ft8201)
{
	int ret = ERROR_CODE_COMM_ERROR;

	/* Every time  enter the factory mode, the previous information is deleted. */
	release_cs_info(g_ft8201);

	ret = enter_factory_mode();

	/* Every time  update the information after enter  the factory mode */
	update_cs_info(g_ft8201);

	return ret;
}

unsigned char ft8201_enter_work(struct ft8201 *g_ft8201)
{
	int ret = ERROR_CODE_OK;

	if (chip_has_two_heart(g_ft8201)) {
		/* double chip deal */
		struct cs_chip_addr_mgr mgr;

		cs_chip_addr_mgr_init(&mgr, g_ft8201);

		work_as_master(&mgr);
		ret = enter_work_mode();
		cs_chip_addr_mgr_exit(&mgr);

	}

	return ret;
}

unsigned char ft8201_chip_clb(struct ft8201 *g_ft8201)
{
	int ret = ERROR_CODE_OK;

	FTS_TEST_FUNC_ENTER();

	if (chip_has_two_heart(g_ft8201)) {
		/* double chip deal */
		struct cs_chip_addr_mgr mgr;

		cs_chip_addr_mgr_init(&mgr, g_ft8201);
		work_as_master(&mgr);
		ret = (ret == ERROR_CODE_OK) ? chip_clb_incell() : ret;

		work_as_slave(&mgr);
		ret = (ret == ERROR_CODE_OK) ? chip_clb_incell() : ret;

		cs_chip_addr_mgr_exit(&mgr);
	}

	FTS_TEST_FUNC_EXIT();

	return ret;
}

unsigned char ft8201_weakshort_get_adcdata(struct ft8201 *g_ft8201, int all_adc_data_len, int *rev_buffer)
{
	int ret = ERROR_CODE_OK;
	struct cs_chip_addr_mgr mgr;
	int master_adc_num = 0;
	int slave_adc_num = 0;
	int *buffer_master = NULL;
	int *buffer_slave = NULL;
	int master_tx = get_master_tx(g_ft8201->cs_info);
	int master_rx = get_master_rx(g_ft8201->cs_info);
	int slave_tx = get_slave_tx(g_ft8201->cs_info);
	int slave_rx = get_slave_rx(g_ft8201->cs_info);

	FTS_TEST_FUNC_ENTER();

	if (chip_has_two_heart(g_ft8201)) {

		buffer_master = fts_malloc((master_tx + 1) * master_rx * sizeof(int));
		buffer_slave = fts_malloc((slave_tx + 1) * slave_rx * sizeof(int));

		cs_chip_addr_mgr_init(&mgr, g_ft8201);

		work_as_master(&mgr);
		ret = weakshort_start_adc_scan();

		work_as_master(&mgr);
		master_adc_num = (master_tx * master_rx + 6) << 1;
		ret = (ret == ERROR_CODE_OK) ? weakshort_get_scan_result() : ret;
		ret = (ret == ERROR_CODE_OK) ? weakshort_get_adc_result(master_adc_num, buffer_master) : ret;

		work_as_slave(&mgr);
		slave_adc_num = (slave_tx * slave_rx + 6) << 1;
		ret = (ret == ERROR_CODE_OK) ? weakshort_get_scan_result() : ret;
		ret = (ret == ERROR_CODE_OK) ? weakshort_get_adc_result(slave_adc_num, buffer_slave) : ret;

		cat_single_to_one_screen(g_ft8201, buffer_master, buffer_slave, rev_buffer);

		cs_chip_addr_mgr_exit(&mgr);

		fts_free(buffer_master);
		fts_free(buffer_slave);

	}

	FTS_TEST_FUNC_EXIT();

	return ret;

}

unsigned char weakshort_start_adc_scan(void)
{
	return write_reg(0x0F, 1);	/* Start ADC sample */
}

unsigned char ft8201_write_reg(struct ft8201 *g_ft8201, unsigned char reg_addr, unsigned char reg_data)
{
	int ret = ERROR_CODE_OK;

	if (chip_has_two_heart(g_ft8201)) {
		/* double chip deal */
		struct cs_chip_addr_mgr mgr;

		cs_chip_addr_mgr_init(&mgr, g_ft8201);

		work_as_master(&mgr);
		ret = (ret == ERROR_CODE_OK) ? write_reg(reg_addr, reg_data) : ret;

		work_as_slave(&mgr);
		ret = (ret == ERROR_CODE_OK) ? write_reg(reg_addr, reg_data) : ret;

		cs_chip_addr_mgr_exit(&mgr);
	}

	return ret;
}

unsigned char weakshort_get_scan_result(void)
{
	unsigned char reg_mark = 0;
	int ret = ERROR_CODE_OK;
	int index = 0;

	for (index = 0; index < 50; ++index) {
		sys_delay(50);
		ret = read_reg(0x10, &reg_mark);	/* Polling sampling end mark */
		if (ERROR_CODE_OK == ret && 0 == reg_mark)
			break;
	}

	return ret;
}

unsigned char weakshort_get_adc_result(int all_adc_data_len, int *rev_buffer)
{
	int ret = ERROR_CODE_OK;
	int index = 0;
	int i = 0;
	int return_num = 0;
	unsigned char w_buffer[2] = { 0 };
	int read_num = 0;
	u8 *read_buffer = NULL;

	FTS_TEST_FUNC_ENTER();

	read_num = all_adc_data_len / BYTES_PER_TIME;
	if ((all_adc_data_len % BYTES_PER_TIME) > 0)
		++read_num;

	memset(w_buffer, 0, sizeof(w_buffer));
	w_buffer[0] = 0x89;

	read_buffer = fts_malloc(all_adc_data_len * sizeof(u8));
	if (read_buffer == NULL) {
		return -ENOMEM;
	}

	return_num = BYTES_PER_TIME;
	if (ret == ERROR_CODE_OK) {
		ret = fts_i2c_read_write(w_buffer, 1, read_buffer, return_num);
	}

	for (i = 1; i < read_num; i++) {
		if (ret != ERROR_CODE_OK) {
			FTS_TEST_SAVE_INFO("fts_i2c_read_write  error.   !!!");
			break;
		}

		if (i == read_num - 1) {	/* last packet */
			return_num = all_adc_data_len - BYTES_PER_TIME * i;
			ret = fts_i2c_read_write(NULL, 0, read_buffer + BYTES_PER_TIME * i, return_num);
		} else {
			return_num = BYTES_PER_TIME;
			ret = fts_i2c_read_write(NULL, 0, read_buffer + BYTES_PER_TIME * i, return_num);
		}
	}

	for (index = 0; index < all_adc_data_len / 2; ++index) {
		rev_buffer[index] = (read_buffer[index * 2] << 8) + read_buffer[index * 2 + 1];
	}

	fts_free(read_buffer);

	FTS_TEST_FUNC_EXIT();

	return ret;
}

unsigned char ft8201_start_scan(struct ft8201 *g_ft8201)
{
	int ret = ERROR_CODE_OK;

	FTS_TEST_FUNC_ENTER();

	if (chip_has_two_heart(g_ft8201)) {
		/* double chip deal */
		struct cs_chip_addr_mgr mgr;

		cs_chip_addr_mgr_init(&mgr, g_ft8201);

		work_as_master(&mgr);
		ret = start_scan_incell();
		cs_chip_addr_mgr_exit(&mgr);
	}

	FTS_TEST_FUNC_EXIT();

	return ret;
}

void init_testitem_ft8201(char *strIniFile)
{
	char str[512];

	FTS_TEST_FUNC_ENTER();

	/* RawData Test */
	GetPrivateProfileString("TestItem", "RAWDATA_TEST", "1", str, strIniFile);
	ft8201_item.rawdata_test = fts_atoi(str);

	/* CB_TEST */
	GetPrivateProfileString("TestItem", "CB_TEST", "1", str, strIniFile);
	ft8201_item.cb_test = fts_atoi(str);

	/* SHORT_CIRCUIT_TEST */
	GetPrivateProfileString("TestItem", "SHORT_CIRCUIT_TEST", "1", str, strIniFile);
	ft8201_item.short_test = fts_atoi(str);

	/* LCD_NOISE_TEST */
	GetPrivateProfileString("TestItem", "LCD_NOISE_TEST", "0", str, strIniFile);
	ft8201_item.lcd_noise_test = fts_atoi(str);

	/* OPEN_TEST */
	GetPrivateProfileString("TestItem", "OPEN_TEST", "0", str, strIniFile);
	ft8201_item.open_test = fts_atoi(str);

	FTS_TEST_FUNC_EXIT();
}

void init_basicthreshold_ft8201(char *strIniFile)
{
	char str[512];

	FTS_TEST_FUNC_ENTER();

	/* RawData Test */
	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Min", "5000", str, strIniFile);
	ft8201_basic_thr.rawdata_test_min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Max", "11000", str, strIniFile);
	ft8201_basic_thr.rawdata_test_max = fts_atoi(str);

	/* CB Test */
	GetPrivateProfileString("Basic_Threshold", "CBTest_VA_Check", "1", str, strIniFile);
	ft8201_basic_thr.cb_test_va_check = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "CBTest_Min", "3", str, strIniFile);
	ft8201_basic_thr.cb_test_min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "CBTest_Max", "100", str, strIniFile);
	ft8201_basic_thr.cb_test_max = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "CBTest_VKey_Check", "1", str, strIniFile);
	ft8201_basic_thr.cb_test_vk_check = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "CBTest_Min_Vkey", "3", str, strIniFile);
	ft8201_basic_thr.cb_test_min_vk = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "CBTest_Max_Vkey", "100", str, strIniFile);
	ft8201_basic_thr.cb_test_max_vk = fts_atoi(str);

	/* Short Circuit Test */
	GetPrivateProfileString("Basic_Threshold", "ShortCircuit_ResMin", "1000", str, strIniFile);
	ft8201_basic_thr.short_res_min = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "ShortCircuit_VkResMin", "500", str, strIniFile);
	ft8201_basic_thr.short_res_vk_min = fts_atoi(str);

	/* Lcd Noise Test */
	GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Frame", "50", str, strIniFile);
	ft8201_basic_thr.lcd_noise_test_frame = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Max_Screen", "32", str, strIniFile);
	ft8201_basic_thr.lcd_noise_test_max_screen = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Max_Frame", "32", str, strIniFile);
	ft8201_basic_thr.lcd_noise_test_max_frame = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Coefficient", "50", str, strIniFile);
	ft8201_basic_thr.lcd_noise_coefficient = fts_atoi(str);
	GetPrivateProfileString("Basic_Threshold", "LCD_NoiseTest_Coefficient_key", "50", str, strIniFile);
	ft8201_basic_thr.lcd_noise_coefficient_key = fts_atoi(str);

	/* Open Test */
	GetPrivateProfileString("Basic_Threshold", "OpenTest_CBMin", "0", str, strIniFile);
	ft8201_basic_thr.open_test_min = fts_atoi(str);

	FTS_TEST_FUNC_EXIT();

}

void init_detailthreshold_ft8201(char *ini)
{
	FTS_TEST_FUNC_ENTER();

	OnInit_InvalidNode(ini);
	OnInit_DThreshold_RawDataTest(ini);
	OnInit_DThreshold_CBTest(ini);

	FTS_TEST_FUNC_EXIT();
}

void set_testitem_sequence_ft8201(void)
{
	test_data.test_num = 0;

	FTS_TEST_FUNC_ENTER();

	/* Enter Factory Mode */
	fts_set_testitem(CODE_FT8201_ENTER_FACTORY_MODE);

	/* OPEN_TEST */
	if (ft8201_item.open_test == 1) {
		fts_set_testitem(CODE_FT8201_OPEN_TEST);
	}
	/* SHORT_CIRCUIT_TEST */
	if (ft8201_item.short_test == 1) {
		fts_set_testitem(CODE_FT8201_SHORT_CIRCUIT_TEST);
	}
	/* CB_TEST */
	if (ft8201_item.cb_test == 1) {
		fts_set_testitem(CODE_FT8201_CB_TEST);
	}
	/* RawData Test */
	if (ft8201_item.rawdata_test == 1) {
		fts_set_testitem(CODE_FT8201_RAWDATA_TEST);
	}
	/* LCD_NOISE_TEST */
	if (ft8201_item.lcd_noise_test == 1) {
		fts_set_testitem(CODE_FT8201_LCD_NOISE_TEST);
	}

	FTS_TEST_FUNC_EXIT();

}

struct test_funcs test_func_ft8201 = {
	.ic_series = TEST_ICSERIES(IC_FT8201),
	.init_testitem = init_testitem_ft8201,
	.init_basicthreshold = init_basicthreshold_ft8201,
	.init_detailthreshold = init_detailthreshold_ft8201,
	.set_testitem_sequence = set_testitem_sequence_ft8201,
	.start_test = start_test_ft8201,
};
