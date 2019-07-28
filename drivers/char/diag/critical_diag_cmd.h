#ifndef CRITICAL_DIAG_CMD_H
#define CRITICAL_DIAG_CMD_H
/*==========================================================================
General Description
	Critial Diag Commands.
	These Diag Commands are considered as harmful items to handset security.

Edit History

when			who		what, where, why
--------		---		---------------------------------------------------
2017/09/29		wzy		Add critical diag commands


===========================================================================*/

typedef struct {
	unsigned char packet_check_length;
	unsigned char cmd_code;
	unsigned char subsys_id;
	unsigned short subsys_cmd_code;
}
diag_packet_type;

static diag_packet_type critical_diag_packets[] = {

	/***************************
	0x3A: reboot to dl (+ AP only)
	****************************/
	{ 1, 0x3a, 0, 0, },

	/**************************************************************
	(2017-09-29 added)
	0x4b 0x65 0x01 0x00 ... // Reboot edl
	***************************************************************/
	{ 4, 0x4B, 0x65, 0x0001, },

	/* add more ciritical diag commands here if need */

};

#endif /* CRITICAL_DIAG_CMD_H */

