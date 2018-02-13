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
 */

#ifndef _NVME_DEBUG_BLK_MQ_H
#define _NVME_DEBUG_BLK_MQ_H

#include <linux/blk-mq.h>

blk_status_t nvme_debug_blk_mq_queue_rq(struct blk_mq_hw_ctx *hctx,
					const struct blk_mq_queue_data *bd);
enum blk_eh_timer_return nvme_debug_blk_mq_timeout(struct request *rq,
						   bool reserved);
void nvme_debug_blk_mq_complete(struct request *rq);
int nvme_debug_blk_mq_admin_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
				      unsigned int hctx_idx);
int nvme_debug_blk_mq_admin_init_request(struct blk_mq_tag_set *set,
					 struct request *rq,
					 unsigned int hctx_idx,
					 unsigned int numa_node);
void nvme_debug_blk_mq_admin_exit_hctx(struct blk_mq_hw_ctx *hctx,
				       unsigned int hctx_idx);
int nvme_debug_blk_mq_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
				unsigned int hctx_idx);
int nvme_debug_blk_mq_init_request(struct blk_mq_tag_set *set,
					 struct request *rq,
					 unsigned int hctx_idx,
					 unsigned int numa_node);
int nvme_debug_blk_mq_poll(struct blk_mq_hw_ctx *hctx, unsigned int tag);
int nvme_debug_blk_mq_map_queues(struct blk_mq_tag_set *set);



#endif /* _NVME_DEBUG_BLK_MQ_H */
