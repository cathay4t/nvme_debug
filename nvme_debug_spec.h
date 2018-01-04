/*
 * Structs and constants defined by NVMe SPEC used by nvme_debug.
 * Copyright (c) 2017, Red Hat Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Author: Gris Ge <fge@redhat.com>
 */

#ifndef _NVME_DEBUG_SPEC_H
#define _NVME_DEBUG_SPEC_H

#include <linux/types.h>

/*BUG(Gris Ge): Handle bit field in big endian system */

#pragma pack(push, 1)
/* NVMe v1.3, 3.1 Register Definition */
struct nvme_debug_reg_cap {
	u16 mqes;
	u8 cqr			: 1;
	u8 ams			: 2;
	u8 reserved_0		: 5;
	u8 to;
	u8 dstrd		: 4;
	u8 nssrs		: 1;
	u8 css;
	u8 bps			: 1;
	u8 reserved_1		: 2;
	u8 mpsmin		: 4;
	u8 mpsmax		: 4;
	u8 reserved_2;
};

struct nvme_debug_reg_vs {
	u8 ter;
	u8 mnr;
	u8 mjr;
};

struct nvme_debug_reg_intms {
	u32 ivms;
};

struct nvme_debug_reg_intmc {
	u32 ivmc;
};

struct nvme_debug_reg_cc {
	u8 en			: 1;
	u8 reserved_0		: 3;
	u8 css			: 3;
	u8 mps			: 4;
	u8 ams			: 3;
	u8 shn			: 2;
	u8 iosqes		: 4;
	u8 iocqes		: 4;
	u8 reserved_1;
};

struct nvme_debug_reg_csts {
	u8 rdy			: 1;
	u8 cfs			: 1;
	u8 shst			: 2;
	u8 nssro		: 1;
	u8 pp			: 1;
	u32 reserved_0		: 26;
};

struct nvme_debug_reg_nssr {
	u32 nssrc;
};

struct nvme_debug_reg_aqa {
	u16 asqs		: 12;
	u8  reserved_0		: 4;
	u16 acqs		: 12;
	u8  reserved_1		: 4;
};

struct nvme_debug_reg_asq {
	u16 reserved_0		: 12;
	u64 asqb		: 52;
};

struct nvme_debug_reg_acq {
	u16 reserved_0		: 12;
	u64 acqb		: 52;
};

struct nvme_debug_reg_cmbloc {
	u8 bir			: 3;
	u16 reserved_0		: 9;
	u32 ofst		: 20;
};

struct nvme_debug_reg_cmbsz {
	u8 sqs			: 1;
	u8 cqs			: 1;
	u8 lists		: 1;
	u8 rds			: 1;
	u8 wds			: 1;
	u8 reserved_0		: 3;
	u8 szu			: 4;
	u32 sz			: 20;
};

/* NVMe v1.3, 3.1 Register Definition */
struct nvme_debug_reg_data {
	struct nvme_debug_reg_cap cap;
	struct nvme_debug_reg_vs vs;
	struct nvme_debug_reg_intms intms;
	struct nvme_debug_reg_intmc intmc;
	struct nvme_debug_reg_cc cc;
	u8 reserved_0[4];
	struct nvme_debug_reg_csts csts;
	struct nvme_debug_reg_nssr nssr;
	struct nvme_debug_reg_aqa aqa;
	struct nvme_debug_reg_asq asq;
	struct nvme_debug_reg_acq acq;
	struct nvme_debug_reg_cmbloc cmbloc;
	struct nvme_debug_reg_cmbsz cmbsz;
	u8 reserved_1[3776];
	u8 reserved_2[255];
	/* Rest(start from 0x1004) are doorbells */
};
#pragma pack(pop)

#endif /* _NVME_DEBUG_SPEC_H */
