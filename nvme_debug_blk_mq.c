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

static void _handle_cmd(struct nvme_debug_queue *queue, struct nvme_command *cmd);
static int _init_request(struct nvme_debug_ctrl *ndc, struct request *rq,
			  unsigned int queue_idx);

static void _handle_cmd(struct nvme_debug_queue *queue, struct nvme_command *cmd)
{
	_DEBUG_PRINT_FUNC_NAME();
}

static int _init_request(struct nvme_debug_ctrl *ndc, struct request *rq,
			  unsigned int queue_idx)
{
	struct nvme_debug_request *ndr = blk_mq_rq_to_pdu(rq);
	struct nvme_debug_queue *queue = ndc->queues[queue_idx];

	_DEBUG_PRINT_FUNC_NAME();

	BUG_ON(queue == NULL);

	ndr->queue = queue;
	return 0;
}
blk_status_t nvme_debug_blk_mq_queue_rq(struct blk_mq_hw_ctx *hctx,
					const struct blk_mq_queue_data *bd)
{
	struct nvme_debug_queue *queue = hctx->driver_data;
	struct nvme_ns *ns = hctx->queue->queuedata;
	struct request *rq = bd->rq;
	struct nvme_command cmd;
	int ret = 0 ;

	_DEBUG_PRINT_FUNC_NAME();

	if (ns == NULL) {
		/* for admin queue */
	} else {
		/* for submission queue */
		return BLK_STS_IOERR;
	}

	ret = nvme_setup_cmd(ns, rq, &cmd);
	if (ret != 0)
		goto out;

	blk_mq_start_request(rq);

	_handle_cmd(queue, &cmd);

	return BLK_STS_OK;

out:
	return (ret == -ENOMEM || ret == -EAGAIN) ?
		BLK_STS_RESOURCE : BLK_STS_IOERR;

}

enum blk_eh_timer_return nvme_debug_blk_mq_timeout(struct request *rq,
						   bool reserved)
{

	_DEBUG_PRINT_FUNC_NAME();
	return BLK_EH_NOT_HANDLED;
}

void nvme_debug_blk_mq_complete(struct request *rq)
{

	_DEBUG_PRINT_FUNC_NAME();
	blk_mq_complete_request(rq);

}

int nvme_debug_blk_mq_init_admin_hctx(struct blk_mq_hw_ctx *hctx, void *data,
				      unsigned int hctx_idx)
{
	struct nvme_debug_ctrl *ndc = data;
	struct nvme_debug_queue *queue = ndc->queues[_NVME_DEBUG_ADMIN_QUEUE_IDX];

	_DEBUG_PRINT_FUNC_NAME();

	WARN_ON(hctx_idx != _NVME_DEBUG_ADMIN_QUEUE_IDX);
	WARN_ON(ndc->admin_tagset.tags[_NVME_DEBUG_ADMIN_QUEUE_IDX]
		!= hctx->tags);
	WARN_ON(queue->tags);

	hctx->driver_data = queue;
	queue->tags = &ndc->admin_tagset.tags[_NVME_DEBUG_ADMIN_QUEUE_IDX];
	return 0;
}

int nvme_debug_blk_mq_init_admin_request(struct blk_mq_tag_set *set,
					 struct request *rq,
					 unsigned int hctx_idx,
					 unsigned int numa_node)
{
//	_DEBUG_PRINT_FUNC_NAME();

	WARN_ON(hctx_idx != _NVME_DEBUG_ADMIN_QUEUE_IDX);

	return _init_request(set->driver_data, rq, _NVME_DEBUG_ADMIN_QUEUE_IDX);
}

void nvme_debug_blk_mq_admin_exit_hctx(struct blk_mq_hw_ctx *hctx,
				       unsigned int hctx_idx)
{
	_DEBUG_PRINT_FUNC_NAME();
}

int nvme_debug_blk_mq_admin_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
				      unsigned int hctx_idx)
{
	_DEBUG_PRINT_FUNC_NAME();
	return 0;
}

int nvme_debug_blk_mq_admin_init_request(struct blk_mq_tag_set *set,
					 struct request *rq,
					 unsigned int hctx_idx,
					 unsigned int numa_node)
{
	_DEBUG_PRINT_FUNC_NAME();
	return 0;
}


int nvme_debug_blk_mq_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
				unsigned int hctx_idx)
{
	_DEBUG_PRINT_FUNC_NAME();

	return 0;
}

int nvme_debug_blk_mq_init_request(struct blk_mq_tag_set *set,
					 struct request *rq,
					 unsigned int hctx_idx,
					 unsigned int numa_node)
{

	_DEBUG_PRINT_FUNC_NAME();
	return 0;
}

int nvme_debug_blk_mq_poll(struct blk_mq_hw_ctx *hctx, unsigned int tag)
{
	_DEBUG_PRINT_FUNC_NAME();
	return 0;
}

int nvme_debug_blk_mq_map_queues(struct blk_mq_tag_set *set)
{
	_DEBUG_PRINT_FUNC_NAME();
	return 0;
}
