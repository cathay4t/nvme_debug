/*
 * NVM debug adapter driver
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

#include <linux/types.h>

#include "nvme.h"
#include "nvme_debug.h"
#include "nvme_debug_spec.h"
#include "nvme_debug_reg_data.h"

static void nvme_debug_reg_data_cap_init(u64 *cap);
static void nvme_debug_reg_data_vs_init(u32 *vs);
static void nvme_debug_reg_data_cc_init(u32 *cc);
static void nvme_debug_reg_data_csts_init(u32 *csts);
static void nvme_debug_reg_data_aqa_init(u32 *aqa);

static void nvme_debug_reg_data_cap_init(u64 *cap)
{
	NVME_REG_CAP_MQES_SET(cap, _NVME_DEBUG_QUEUE_SIZE);
	/* ^ Max queue size of I/O submit and complete queue */
	NVME_REG_CAP_CQR(cap, 0);
	/* ^ No need to be physically contiguous */
	NVME_REG_CAP_AMS(cap, 0);
	/* ^ No special arbitration mechanism */
	NVME_REG_CAP_TO(cap, 1);
	/* ^ the worst case time that host software shall wait for CSTS.RDY .
	 *   This field is in 500 millisecond units.
	 */
	NVME_REG_CAP_DSTRD(cap, 1);
	/* ^ The stride of Submission Queue and Completion Queue doorbell
	 *   register: (2 ^ (2 + DSTRD))
	 *   1 here means this stride is 8 bytes(64 bits).
	 *   TODO(Gris Ge): Need to check section 8.6
	 *                  "Doorbell Stride for Software Emulation"
	 */
	NVME_REG_CAP_NSSRS(cap, 0);
	/* ^ Not support NVMe reset yet */

	NVME_REG_CAP_CSS(cap, 1);
	/* ^ Support NVMe command set */

	NVME_REG_CAP_BPS(cap, 0);
	/* ^ No boot partition support. TODO(Gris Ge): this might be required
	 *   for firmware upgrade.
	 */
	NVME_REG_CAP_MPSMIN(cap, 0);
	/* ^ Minimum memory page size is 4 KiB. (2 ** (12 + 0) bytes) */
	NVME_REG_CAP_MPSMAX(cap, 15);
	/* ^ Maximum memory page size is 128 MiB. (2 ** (12 + 15) bytes) */
}

static void nvme_debug_reg_data_vs_init(u32 *vs)
{
	/* Set to 1.3.0 version */
	NVME_REG_CS_MJR_SET(vs, 1);
	NVME_REG_CS_MNR_SET(vs, 3);
	NVME_REG_CS_TER_SET(vs, 0);
}

static void nvme_debug_reg_data_cc_init(u32 *cc)
{
	NVME_REG_CC_EN_SET(cc, 1);
	/* ^ Enabled */
	NVME_REG_CC_CSS_SET(cc, 0);
	/* ^ NVMe command set */
	NVME_REG_CC_MPS_SET(cc, 12);
	/* ^ Memory page size: 16 MiB. (2 ** (12 + 5) bytes) */
	NVME_REG_CC_AMS_SET(cc, 0);
	/* ^ Use round robin arbitration mechanism */
	NVME_REG_CC_SHN_SET(cc, 0);
	/* ^ No shutdown notification yet */
	NVME_REG_CC_IOSQES_SET(cc, 6);
	/* ^ 64 bytes for I/O submission queue entry size */
	NVME_REG_CC_IOCQES_SET(cc, 4);
	/* ^ 16 bytes for I/o completion queue entry size */
}

static void nvme_debug_reg_data_csts_init(u32 *csts)
{
	NVME_REG_CSTS_RDY_SET(csts, 1);
	/* ^ Ready for submission queue */
	NVME_REG_CSTS_CFS_SET(csts, 0);
	/* ^ No fatal error */
	NVME_REG_CSTS_SHST_SET(csts, 0);
	/* ^ No shutdown has been requested */
	NVME_REG_CSTS_NSSRO_SET(csts, 0);
	/* ^ No reset has been requested */
	NVME_REG_CSTS_PP_SET(csts, 0);
	/* ^ No paused for processing commands */
}

static void nvme_debug_reg_data_aqa_init(u32 *aqa)
{
	NVME_REG_AQA_ASQS(aqa, _NVME_DEBUG_QUEUE_SIZE);
	NVME_REG_AQA_ACQS(aqa, _NVME_DEBUG_QUEUE_SIZE);
}

void nvme_debug_reg_data_init(struct nvme_debug_reg_data *reg_data)
{
	nvme_debug_reg_data_cap_init(&reg_data->cap);
	nvme_debug_reg_data_vs_init(&reg_data->vs);
	reg_data->intms = 0;
	reg_data->intmc = 0;
	/* ^ TODO(Gris Ge): No idea what Interrupt Mask means */
	nvme_debug_reg_data_cc_init(&reg_data->cc);
	nvme_debug_reg_data_csts_init(&reg_data->csts);
	reg_data->nssr = 0;
	/* ^ No reset has been requested */
	nvme_debug_reg_data_aqa_init(&reg_data->aqa);
	NVME_REG_ASQ_ASQB(&reg_data->asq, 0);
	/* ^ BUG TODO(Gris Ge): Request an memory address for admin submission
	 *                      queue. Should be aligned to cc->mps.
	 */
	NVME_REG_ACQ_ACQB(&reg_data->acq, 0);
	/* ^ BUG TODO(Gris Ge): Request an memory address for admin completion
	 *                      queue. Should be aligned to cc->mps.
	 */
}

int nvme_debug_reg_read32(struct nvme_ctrl *ctrl, u32 off, u32 *val)
{
	if (off >= sizeof(struct nvme_debug_reg_data) - sizeof(u32))
		return -EINVAL;
	*val = *( (u32 *) (((u8 *) &((_ctrl_to_ndc(ctrl))->reg_data)) + off));
	_ND_DEBUG("read32 offset 0x%x: 0x%x", off, *val);
	return 0;
}

int nvme_debug_reg_write32(struct nvme_ctrl *ctrl, u32 off, u32 val)
{
	struct nvme_debug_reg_data *reg_data = NULL;

	if (off >= sizeof(struct nvme_debug_reg_data) - sizeof(u32))
		return -EINVAL;

	reg_data = &((_ctrl_to_ndc(ctrl))->reg_data);
	_ND_DEBUG("write32 offset 0x%x: 0x%x", off, val);

	if (off == NVME_REG_CC) {
		if (NVME_REG_CC_EN_GET(&val) == 0) {
			if (NVME_REG_CC_EN_GET(&reg_data->cc) != 0) {
				_ND_DEBUG("disabling controller");
				// Disabling controller.
				NVME_REG_CSTS_RDY_SET(&reg_data->csts, 0);
			} else
				_ND_DEBUG("controller already disabled");
		} else {
			if (NVME_REG_CC_EN_GET(&reg_data->cc) == 0) {
				// Enabling controller.
				_ND_DEBUG("enabling controller");
				NVME_REG_CSTS_RDY_SET(&reg_data->csts, 1);
			} else
				_ND_DEBUG("controller already enabled");
		}
	}
	*( (u32 *) (((u8 *) reg_data) + off)) = val;

	return 0;
}

int nvme_debug_reg_read64(struct nvme_ctrl *ctrl, u32 off, u64 *val)
{
	if (off >= sizeof(struct nvme_debug_reg_data) - sizeof(u64))
		return -EINVAL;

	*val = *( (u64 *) (((u8 *) &((_ctrl_to_ndc(ctrl))->reg_data)) + off));
	_ND_DEBUG("read64 offset 0x%x: 0x%llx", off, *val);
	return 0;
}
