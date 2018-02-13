/*
 * Structs and constants defined by NVMe SPEC used by nvme_debug.
 * Copyright (c) 2017,2018 Red Hat Corporation.
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

#include "nvme_debug_misc.h"

/* NVMe v1.3, 3.1.1 Offset 00h: CAP - Controller Capabilities */
#define NVME_REG_CAP_MQES_SET(c, v) _set_bits_u16_le((u8 *) c, 15, 0, v)
#define NVME_REG_CAP_CQR(c, v) _set_bits_u8((u8 *) c, 16, 16, v)
#define NVME_REG_CAP_AMS(c, v) _set_bits_u8((u8 *) c, 18, 17, v)
#define NVME_REG_CAP_TO(c, v) _set_bits_u8((u8 *) c, 31, 24, v)
#define NVME_REG_CAP_DSTRD(c, v) _set_bits_u8((u8 *) c, 35, 32, v)
#define NVME_REG_CAP_NSSRS(c, v) _set_bits_u8((u8 *) c, 36, 36, v)
#define NVME_REG_CAP_CSS(c, v) _set_bits_u8((u8 *) c, 44, 37, v)
#define NVME_REG_CAP_BPS(c, v) _set_bits_u8((u8 *) c, 45, 45, v)
#define NVME_REG_CAP_MPSMIN(c, v) _set_bits_u8((u8 *) c, 51, 48, v)
#define NVME_REG_CAP_MPSMAX(c, v) _set_bits_u8((u8 *) c, 55, 52, v)

/* NVMe v1.3, 3.1.2 Offset 08h: VS - Version */
#define NVME_REG_CS_MJR_SET(c, v) _set_bits_u16_le((u8 *) c, 31, 16, v)
#define NVME_REG_CS_MNR_SET(c, v) _set_bits_u8((u8 *) c, 15, 8, v)
#define NVME_REG_CS_TER_SET(c, v) _set_bits_u8((u8 *) c, 7, 0, v)

/* NVMe v1.3, 3.1.5 Offset 14h: CC - Controller Configuration */
#define NVME_REG_CC_EN_GET(c) _get_bits_u8((u8 *) c, 0, 0)
#define NVME_REG_CC_EN_SET(c, v) _set_bits_u8((u8 *) c, 0, 0, v)
#define NVME_REG_CC_CSS_SET(c, v) _set_bits_u8((u8 *) c, 6, 4, v)
#define NVME_REG_CC_MPS_SET(c, v) _set_bits_u8((u8 *) c, 10, 7, v)
#define NVME_REG_CC_AMS_SET(c, v) _set_bits_u8((u8 *) c, 13, 11, v)
#define NVME_REG_CC_SHN_SET(c, v) _set_bits_u8((u8 *) c, 15, 14, v)
#define NVME_REG_CC_IOSQES_SET(c, v) _set_bits_u8((u8 *) c, 19, 16, v)
#define NVME_REG_CC_IOCQES_SET(c, v) _set_bits_u8((u8 *) c, 23, 20, v)

/* NVMe v1.3, 3.1.6 Offset 1Ch: CSTS - Controller Status */
#define NVME_REG_CSTS_RDY_SET(c, v) _set_bits_u8((u8 *) c, 0, 0, v)
#define NVME_REG_CSTS_CFS_SET(c, v) _set_bits_u8((u8 *) c, 1, 1, v)
#define NVME_REG_CSTS_SHST_SET(c, v) _set_bits_u8((u8 *) c, 3, 2, v)
#define NVME_REG_CSTS_NSSRO_SET(c, v) _set_bits_u8((u8 *) c, 4, 4, v)
#define NVME_REG_CSTS_PP_SET(c, v) _set_bits_u8((u8 *) c, 5, 5, v)

/* NVMe v1.3, 3.1.8 Offset 24h: AQA - Admin Queue Attributes */
#define NVME_REG_AQA_ASQS(c, v) _set_bits_u16_le((u8 *) c, 11, 0, v)
#define NVME_REG_AQA_ACQS(c, v) _set_bits_u16_le((u8 *) c, 27, 16, v)

/* NVMe v1.3, 3.1.9 Offset 28h: ASQ - Admin Submission Queue Base Address */
#define NVME_REG_ASQ_ASQB(c, v) _set_bits_u64_le((u8 *) c, 63, 12, v)

/* NVMe v1.3, 3.1.10 Offset 30h: ACQ - Admin Completion Queue Base Address */
#define NVME_REG_ACQ_ACQB(c, v) _set_bits_u64_le((u8 *) c, 63, 12, v)

#pragma pack(push, 1)
/* NVMe v1.3, 3.1 Register Definition */
struct nvme_debug_reg_data {
	u64 cap;
	u32 vs;
	u32 intms;
	u32 intmc;
	u32 cc;
	u32 reserved_0;
	u32 csts;
	u32 nssr;
	u32 aqa;
	u32 asq;
	u32 acq;
	u32 cmbloc;
	u32 cmbsz;
	u32 bpinfo;
	u32 bprsel;
	u64 bpmbl;
	u8 reserved_1[3760];
	u8 cs_specific[256]; /* Command Set Specific */
	/* Rest(start from 0x1000) are doorbells */
};
#pragma pack(pop)

#endif /* _NVME_DEBUG_SPEC_H */
