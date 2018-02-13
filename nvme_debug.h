/*
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

#ifndef _NVME_DEBUG_H
#define _NVME_DEBUG_H

#include <linux/list.h>
#include <linux/init.h>

#include "nvme.h"
#include "nvme_debug_spec.h"

struct nvme_debug_ctrl;
struct nvme_debug_ns;
struct nvme_debug_queue;
struct nvme_debug_request;

struct nvme_debug_ctrl {
	struct list_head		ndc_list;
	struct nvme_ctrl		ctrl;
	struct device			dev;
	struct list_head		ndn_list;
	struct nvme_debug_reg_data	reg_data;
	struct blk_mq_tag_set		tagset;
	struct blk_mq_tag_set		admin_tagset;
	struct nvme_debug_queue		**queues;
	u32				queue_count;
	u64				cap;
	struct mutex			shutdown_lock;
	unsigned int			online_queues;
};

struct nvme_debug_ns {
	struct list_head		ndn_list;
	struct nvme_ns			ns;
	struct nvme_debug_ctrl		*ndc;
};


struct nvme_debug_queue {
	struct nvme_debug_ctrl		*ndc;
	spinlock_t			q_lock;
	struct blk_mq_tags		**tags;
	struct nvme_completion		*cqes;
	struct nvme_command		*sq_cmds;
	u16				sq_tail;
	u16				cq_head;
	u8				cq_phase;
	u16				q_depth;
	u16				qid;
	s16				cq_vector;
	void *				asqb;
	void *				acqb;
};

struct nvme_debug_request {
	struct nvme_request req;
	struct nvme_debug_queue *queue;
};

#define _dev_to_ndc(__d)	\
	container_of(__d, struct nvme_debug_ctrl, dev)

#define _ctrl_to_ndc(__c)	\
	container_of(__c, struct nvme_debug_ctrl, ctrl)

#define _reset_work_to_ndc(__c)	\
	container_of(__c, struct nvme_debug_ctrl, reset_work)

#define __FILE_NAME__ kbasename(__FILE__)

#define _ND_DEBUG(fmt, ...) \
	do { \
		pr_debug(fmt " %s:%d\n", ##__VA_ARGS__, \
			 __FILE_NAME__, __LINE__); \
	} while(0)


#define _DEBUG_PRINT_FUNC_NAME() \
	do { \
		pr_debug("%-40s %s:%d\n", __FUNCTION__, \
			 __FILE_NAME__, __LINE__); \
	} while(0)

#define _NVME_DEBUG_QUEUE_SIZE		256
#define _NVME_DEBUG_QUEUE_COUNT		256
#define _NVME_DEBUG_ADMIN_QUEUE_IDX	0

#endif /* _NVME_DEBUG_H */
