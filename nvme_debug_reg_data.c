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

static void nvme_debug_reg_data_cap_init(struct nvme_debug_reg_cap *cap);
static void nvme_debug_reg_data_vs_init(struct nvme_debug_reg_vs *vs);
static void nvme_debug_reg_data_cc_init(struct nvme_debug_reg_cc *cc);
static void nvme_debug_reg_data_csts_init(struct nvme_debug_reg_csts *csts);
static void nvme_debug_reg_data_aqa_init(struct nvme_debug_reg_aqa *aqa);
static void nvme_debug_reg_data_cmbloc_init
	(struct nvme_debug_reg_cmbloc *cmbloc);

static void nvme_debug_reg_data_cap_init(struct nvme_debug_reg_cap *cap)
{
	cap->mqes = _NVME_DEBUG_QUEUE_SIZE;
	/* ^ Max queue size of I/O submit and complete queue */
	cap->cqr = 0;
	/* ^ No need to be physically contiguous */
	cap->ams = 0;
	/* ^ No special arbitration mechanism */
	cap->to = 1;
	/* ^ the worst case time that host software shall wait for CSTS.RDY .
	 *   This field is in 500 millisecond units.
	 */
	cap->dstrd = 1;
	/* ^ The stride of Submission Queue and Completion Queue doorbell
	 *   register: (2 ^ (2 + DSTRD))
	 *   1 here means this stride is 8 bytes(64 bits).
	 *   TODO(Gris Ge): Need to check section 8.6
	 *                  "Doorbell Stride for Software Emulation"
	 */
	cap->nssrs = 0;
	/* ^ Not support NVMe reset yet */

	cap->css = 1;
	/* ^ Support NVMe command set */

	cap->bps = 0;
	/* ^ No boot partition support. TODO(Gris Ge): this might be required
	 *   for firmware upgrade.
	 */
	cap->mpsmin = 0;
	/* ^ Minimum memory page size is 4 KiB. (2 ** (12 + 0) bytes) */
	cap->mpsmax = 15;
	/* ^ Maximum memory page size is 128 MiB. (2 ** (12 + 15) bytes) */
}

static void nvme_debug_reg_data_vs_init(struct nvme_debug_reg_vs *vs)
{
	/* Set to 1.3.0 version */
	vs->mjr = 1;
	vs->mnr = 3;
	vs->ter = 0;
}

static void nvme_debug_reg_data_cc_init(struct nvme_debug_reg_cc *cc)
{
	cc->en = 1;
	/* ^ Enabled */
	cc->css = 0;
	/* ^ NVMe command set */
	cc->mps = 12;
	/* ^ Memory page size: 16 MiB. (2 ** (12 + 5) bytes) */
	cc->ams = 0;
	/* ^ Use round robin arbitration mechanism */
	cc->shn = 0;
	/* ^ No shutdown notification yet */
	cc->iosqes = 6;
	/* ^ 64 bytes for I/O submission queue entry size */
	cc->iocqes = 4;
	/* ^ 16 bytes for I/o completion queue entry size */
}

static void nvme_debug_reg_data_csts_init(struct nvme_debug_reg_csts *csts)
{
	csts->rdy = 1;
	/* ^ Ready for submission queue */
	csts->cfs = 0;
	/* ^ No fatal error */
	csts->shst = 0;
	/* ^ No shutdown has been requested */
	csts->nssro = 0;
	/* ^ No reset has been requested */
	csts->pp = 0;
	/* ^ No paused for processing commands */
}

static void nvme_debug_reg_data_aqa_init(struct nvme_debug_reg_aqa *aqa)
{
	aqa->asqs = _NVME_DEBUG_QUEUE_SIZE;
	aqa->acqs = _NVME_DEBUG_QUEUE_SIZE;
}

static void nvme_debug_reg_data_cmbloc_init
	(struct nvme_debug_reg_cmbloc *cmbloc)
{
	cmbloc->bir = 0;
	/* ^ Disable controller memory buffer */
	cmbloc->ofst = 0;
	/* ^ Disable controller memory buffer */

	/* TODO(Gris Ge): There seems is no way to simulate a BAR for
	 *                "Controller Memory Buffer"
	 */
}

void nvme_debug_reg_data_init(struct nvme_debug_reg_data *reg_data)
{
	nvme_debug_reg_data_cap_init(&reg_data->cap);
	nvme_debug_reg_data_vs_init(&reg_data->vs);
	reg_data->intms.ivms = 0;
	reg_data->intmc.ivmc = 0;
	/* ^ TODO(Gris Ge): No idea what Interrupt Mask means */
	nvme_debug_reg_data_cc_init(&reg_data->cc);
	nvme_debug_reg_data_csts_init(&reg_data->csts);
	reg_data->nssr.nssrc = 0;
	/* ^ No reset has been requested */
	nvme_debug_reg_data_aqa_init(&reg_data->aqa);
	reg_data->asq.asqb = 0;
	/* ^ BUG TODO(Gris Ge): Request an memory address for admin submission
	 *                      queue. Should be aligned to cc->mps.
	 */
	reg_data->acq.acqb = 0;
	/* ^ BUG TODO(Gris Ge): Request an memory address for admin completion
	 *                      queue. Should be aligned to cc->mps.
	 */
	nvme_debug_reg_data_cmbloc_init(&reg_data->cmbloc);
}

int nvme_debug_reg_read32(struct nvme_ctrl *ctrl, u32 off, u32 *val)
{
	*val = *(((u32 *) &((_ctrl_to_ndc(ctrl))->reg_data)) + off);
	return 0;
}

int nvme_debug_reg_write32(struct nvme_ctrl *ctrl, u32 off, u32 val)
{
	/* TODO(Gris Ge): Handle impact like controller reset and etc */
	((u32 *) &(_ctrl_to_ndc(ctrl)->reg_data))[off] = val;
	return 0;
}

int nvme_debug_reg_read64(struct nvme_ctrl *ctrl, u32 off, u64 *val)
{
	*val = ((u64 *) &(_ctrl_to_ndc(ctrl)->reg_data))[off];
	return 0;
}
