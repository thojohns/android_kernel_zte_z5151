/*****************************************************************************
* Copyright 2010 Texas Instruments Corporation, All Rights Reserved.
* TI makes NO WARRANTY as to software products, which are supplied "AS-IS"
*****************************************************************************/

/*****************************************************************************
*     This code is automatically generated from bqfs/dffs file.              *
*          DO NOT MODIFY THIS FILE DIRECTLY                                  *
*****************************************************************************/

/*this file is generated from NTC Learned.gm.fs at Thu Jan 11 14:12:19 2018*/

#ifndef __BATT_450_18_3900_ID390K_TMB1458_TMB436484__
#define __BATT_450_18_3900_ID390K_TMB1458_TMB436484__

#include "bqfs_cmd_type.h"

const bqfs_cmd_t batt_450_18_3900_id390k_tmb1458_tmb436484[] = {
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x00,
		.data		= {.bytes = {0x01, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x00,
		.data		= {.bytes = {0x26, 0x04} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x00,
		.data		= {.bytes = {0x02, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x00,
		.data		= {.bytes = {0x02, 0x02} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x02, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x02, 0x26, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xA5} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x02, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xA5} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x24, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x00, 0x19, 0x28, 0x63, 0x5F, 0xFF, 0x62, 0x00, 0x32, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x69} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x24, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x69} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x31, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x0A, 0x0F, 0x02, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xDF} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x31, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xDF} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x40, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x64, 0x79, 0x0F, 0x86, 0x23, 0x0A, 0x00, 0x14, 0x04, 0x00,
			0x09, 0x04, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x15} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x40, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x15} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x44, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x00, 0x32, 0x01, 0xC2, 0x30, 0x00, 0x03, 0x08, 0x98, 0x01,
			0x00, 0x3C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xF9} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x44, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xF9} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x50, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x01, 0xF4, 0x00, 0x1E, 0xC8, 0x14, 0x08, 0x00, 0x3C, 0x0E,
			0x10, 0x00, 0x0A, 0x46, 0x05, 0x14, 0x05, 0x0F, 0x03, 0x20,
			0x7F, 0xFF, 0x00, 0xF0, 0x46, 0x50, 0x18, 0x01, 0x90, 0x00,
			0x64, 0x19} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xE4} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x50, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xE4} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x50, 0x01} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0xDC, 0x5C, 0x60, 0x00, 0x7D, 0x00, 0x04, 0x03, 0x19, 0x25,
			0x0F, 0x14, 0x0A, 0x78, 0x60, 0x28, 0x01, 0xF4, 0x00, 0x00,
			0x00, 0x00, 0x43, 0x80, 0x04, 0x01, 0x14, 0x00, 0x08, 0x0B,
			0xB8, 0x01} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xDB} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x50, 0x01} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xDB} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x50, 0x02} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x2C, 0x0A, 0x01, 0x0A, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x64,
			0x02, 0x00, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x01, 0x03, 0x5A,
			0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x47} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x50, 0x02} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x47} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x51, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x01, 0x40, 0x01, 0x90, 0x07, 0xD0, 0x00, 0x3C, 0x3C, 0x01,
			0xB3, 0xB3, 0x01, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xE6} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x51, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xE6} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x52, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x40, 0x4D, 0x00, 0x00, 0x00, 0x81, 0x0C, 0x80, 0x30, 0x0C,
			0x0C, 0xE4, 0x00, 0xC8, 0x00, 0x32, 0x00, 0x14, 0x03, 0xE8,
			0x01, 0x00, 0xB1, 0x00, 0x0A, 0xFF, 0xCE, 0xFF, 0xCB, 0x00,
			0x02, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xEB} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x52, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xEB} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x53, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x18, 0x05, 0x11, 0x0F, 0xDF, 0xE1, 0xE4, 0xDF, 0xE5, 0xE1,
			0xE6, 0xE2, 0xE7, 0xE6, 0xE5, 0xE1, 0xE7, 0xEC, 0xED, 0xE9,
			0xE3, 0xE7, 0xEE, 0xF0, 0xF3, 0xF4, 0xF7, 0xF9, 0xFB, 0xFA,
			0xFC, 0xFF} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x06} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x53, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x06} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x53, 0x01} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0xF6, 0xF7, 0xF5, 0xF3, 0xEE, 0xEB, 0xFB, 0xF8, 0xFF, 0xD6,
			0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x09} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x53, 0x01} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x09} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x54, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x07, 0x1B, 0x04, 0xFF, 0xEF, 0xED, 0x15, 0xED, 0x0D, 0xEA,
			0x0A, 0xEC, 0xF4, 0x11, 0x31, 0x0D, 0xE4, 0xFB, 0x18, 0xF3,
			0xE0, 0x00, 0xEE, 0x08, 0xFD, 0xFB, 0xE9, 0xEC, 0xF2, 0xEB,
			0xD0, 0xE4} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xAE} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x54, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xAE} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x54, 0x01} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0xE5, 0x0C, 0x15, 0x0C, 0x04, 0xF5, 0x23, 0xEC, 0x56, 0x81,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x0E} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x54, 0x01} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x0E} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x55, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0xFE, 0xD2, 0x01, 0x29, 0xF7, 0x0F, 0x01, 0x0A, 0xFF, 0xFB,
			0xF9, 0x15, 0xB6, 0x40, 0xE1, 0x17, 0xF7, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x07} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x55, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x07} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x6C, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0xFE, 0x49, 0x01, 0x09, 0x00, 0x18, 0xFA, 0xEC, 0x07, 0xE5,
			0xFA, 0x1E, 0xF1, 0xCB, 0x01, 0x2B, 0xA6, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x1E} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x6C, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x1E} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x59, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x00, 0x44, 0x00, 0x44, 0x00, 0x40, 0x00, 0x4C, 0x00, 0x44,
			0x00, 0x37, 0x00, 0x40, 0x00, 0x3D, 0x00, 0x45, 0x00, 0x43,
			0x00, 0x6A, 0x00, 0x4E, 0x00, 0x73, 0x00, 0xBE, 0x00, 0x8F,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xF3} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x59, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xF3} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x6D, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x0C, 0x38, 0x0E, 0xE6, 0x0E, 0xB1, 0x11, 0x22, 0x10, 0xDD,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xE8} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x6D, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xE8} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x68, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x00, 0x00, 0x00, 0x00, 0xB7, 0x15, 0xF8, 0x80, 0xCB, 0x4F,
			0x7E, 0x8D, 0x02, 0x1B, 0xC8, 0xCE, 0x7E, 0x48, 0x99, 0xB3,
			0x28, 0x8C, 0x2D, 0xB7, 0x00, 0x00, 0x00, 0x00, 0xCB, 0xD4,
			0x1A, 0x05} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x7B} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x68, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x7B} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x68, 0x01} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x1C, 0x98, 0x02, 0xD3, 0xFF, 0xB9, 0x30, 0xEF, 0x05, 0x11,
			0x05, 0x01, 0x00, 0x00, 0x10, 0x01, 0x00, 0x3C, 0x00, 0x50,
			0x3C, 0x00, 0x64, 0x3C, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xEA} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x68, 0x01} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xEA} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x69, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x00, 0x00, 0x0B, 0xD6, 0x7E, 0x73, 0xB6, 0x45, 0x93, 0x0A,
			0xA5, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xCE} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x69, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xCE} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x6B, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0xEF, 0x05, 0x11, 0x05, 0x01, 0x00, 0x00, 0x10, 0x01, 0x00,
			0x3C, 0x00, 0x50, 0x3C, 0x00, 0x64, 0x3C, 0x00, 0x20, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x5B} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x6B, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0x5B} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x70, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x40,
		.data		= {.bytes = {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00} },
		.data_len	= 32,
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xFF} },
		.data_len	= 1,
	},
	{
		.cmd_type	= CMD_X,
		.data		= {.delay = 10},
	},
	{
		.cmd_type	= CMD_W,
		.addr		= 0xAA,
		.reg		= 0x3E,
		.data		= {.bytes = {0x70, 0x00} },
		.data_len	= 2,
	},
	{
		.cmd_type	= CMD_C,
		.addr		= 0xAA,
		.reg		= 0x60,
		.data		= {.bytes = {0xFF} },
		.data_len	= 1,
	},
};
/*end of const bqfs_cmd_t bqfs_image[]*/
#endif
