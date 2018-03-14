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
#include "nvme_debug_blk_mq.h"

struct nvme_debug_cmd {
	struct list_head list;
	struct llist_node ll_list;
	call_single_data_t csd;
	struct request *rq;
	struct bio *bio;
	unsigned int tag;
	struct nvme_debug_queue *nq;
	struct hrtimer timer;
	blk_status_t error;
};

static blk_status_t nvme_debug_handle_cmd(struct nvme_debug_cmd *cmd)
{
	struct nvme_debug_ctrl *ndc = cmd->nq->ndc;
	struct nvme_debug *nvme_debug = ndc->nvme_debug;
	int err = 0;

	// Coding
	if (nvme_debug->dev->badblocks.shift != -1) {
		int bad_sectors;
		sector_t sector, size, first_bad;
		bool is_flush = true;

		if (dev->queue_mode == NULL_Q_BIO &&
				bio_op(cmd->bio) != REQ_OP_FLUSH) {
			is_flush = false;
			sector = cmd->bio->bi_iter.bi_sector;
			size = bio_sectors(cmd->bio);
		}
		if (dev->queue_mode != NULL_Q_BIO &&
				req_op(cmd->rq) != REQ_OP_FLUSH) {
			is_flush = false;
			sector = blk_rq_pos(cmd->rq);
			size = blk_rq_sectors(cmd->rq);
		}
		if (!is_flush && badblocks_check(&nvme_debug->dev->badblocks, sector,
				size, &first_bad, &bad_sectors)) {
			cmd->error = BLK_STS_IOERR;
			goto out;
		}
	}

	if (dev->memory_backed) {
		if (dev->queue_mode == NULL_Q_BIO) {
			if (bio_op(cmd->bio) == REQ_OP_FLUSH)
				err = nvme_debug_handle_flush(nvme_debug);
			else
				err = nvme_debug_handle_bio(cmd);
		} else {
			if (req_op(cmd->rq) == REQ_OP_FLUSH)
				err = nvme_debug_handle_flush(nvme_debug);
			else
				err = nvme_debug_handle_rq(cmd);
		}
	}
	cmd->error = errno_to_blk_status(err);
out:
	blk_mq_complete_request(cmd->rq);
	return BLK_STS_OK;
}

blk_status_t nvme_debug_blk_mq_queue_rq
	(struct blk_mq_hw_ctx *hctx, const struct blk_mq_queue_data *bd)
{
	struct nvme_debug_cmd *cmd = blk_mq_rq_to_pdu(bd->rq);
	struct nvme_debug_queue *nq = hctx->driver_data;

	might_sleep_if(hctx->flags & BLK_MQ_F_BLOCKING);

	cmd->rq = bd->rq;
	cmd->nq = nq;

	blk_mq_start_request(bd->rq);

	return _handle_cmd(cmd);
}

void nvme_debug_blk_mq_complete(struct request *rq)
{
	struct nvme_debug_cmd *cmd = blk_mq_rq_to_pdu(rq);

	blk_mq_end_request(cmd->rq, cmd->error);
}
