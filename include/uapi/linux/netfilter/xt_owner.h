#ifndef _XT_OWNER_MATCH_H
#define _XT_OWNER_MATCH_H

#include <linux/types.h>

#ifdef ZTE_MULTIPLE_UID_LIMIT_NET_RATE
#define ZTE_MAX_UID_ENTRY 10
#endif

enum {
	XT_OWNER_UID    = 1 << 0,
	XT_OWNER_GID    = 1 << 1,
	XT_OWNER_SOCKET = 1 << 2,
};

struct xt_owner_match_info {
	__u32 uid_min, uid_max;
	__u32 gid_min, gid_max;
	__u8 match, invert;
#ifdef ZTE_MULTIPLE_UID_LIMIT_NET_RATE
	__u8 uid_array_pos;
	__u32 uid_array[ZTE_MAX_UID_ENTRY];
#endif
};

#endif /* _XT_OWNER_MATCH_H */
