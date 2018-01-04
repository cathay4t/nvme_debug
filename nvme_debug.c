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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uuid.h>
#include <linux/string.h>
#include <linux/blk-mq.h>

#include "nvme.h"
#include "nvme_debug.h"
#include "nvme_debug_spec.h"
#include "nvme_debug_reg_data.h"
#include "nvme_debug_blk_mq.h"

/* NVMe SPEC is using little endian format */

#define _MY_NAME			"nvme_debug"
#define _DEFAULT_CTRL_NUM		1

#define _NVME_MAX_CTRL_ID		0xffff

#define _NVME_DEBUG_ADMIN_QUEUE_IDX	0
#define _NVME_DEBUG_AQ_BLKMQ_DEPTH	31

#define _mem_alloc_check(__ptr_to_check, rc, goto_out) \
	do { \
		if (__ptr_to_check == NULL) { \
			pr_err("Out of memory at func %s, file %s, line %d\n", \
			       __FUNCTION__, __FILE_NAME__,  __LINE__); \
			rc = -ENOMEM; \
			goto goto_out; \
		} \
	} while(0) \

#define _call_and_check_return(__ret_val, __ret) \
	do { \
		__ret = __ret_val; \
		if (__ret != 0) { \
			pr_debug("%s ret %d, %s %d \n", \
				 __FUNCTION__, __ret, __FILE_NAME__, __LINE__); \
			return ret; \
		} \
	} while(0) \

/*
 * TODO(Gris Ge): Just copy from pci.c. Need investigation.
 * We handle AEN commands ourselves and don't even let the
 * block layer know about them.
 */
#define NVME_AQ_BLKMQ_DEPTH	(NVME_AQ_DEPTH - NVME_NR_AERS)

/*
 * Copy from pci.c
 */
#define SQ_SIZE(depth)		(depth * sizeof(struct nvme_command))
#define CQ_SIZE(depth)		(depth * sizeof(struct nvme_completion))

static int nvme_debug_bus_register(void);
static int nvme_debug_lld_bus_match(struct device *dev,
				    struct device_driver *dev_driver);
static int nvme_debug_driver_probe(struct device *dev);
static int nvme_debug_driver_remove(struct device *dev);
static ssize_t nvme_debug_ctrl_num_show(struct device_driver *ddp, char *buf);
static ssize_t nvme_debug_ctrl_num_store(struct device_driver *ddp,
					 const char *buf, size_t count);
static int nvme_debug_ctrl_num_param_set(const char *val,
					 const struct kernel_param *kp);
static DRIVER_ATTR(ctrl_num, S_IWUSR| S_IRUGO, nvme_debug_ctrl_num_show,
		   nvme_debug_ctrl_num_store);
static int nvme_debug_ctrl_num_change(void);
static int nvme_debug_add_ctrl(void);
static void nvme_debug_disable_io_queues(struct nvme_debug_ctrl *ndc,
					 int queues);
static void nvme_debug_disable_admin_queue(struct nvme_debug_ctrl *ndc,
					   bool shutdown);
static int nvme_debug_alloc_admin_tags(struct nvme_debug_ctrl *ndc);
static int nvme_debug_setup_io_queues(struct nvme_debug_ctrl *ndc);
static int nvme_debug_blk_mq_dev_add(struct nvme_debug_ctrl *ndc);

static void nvme_debug_remove_all_ctrls(void);
static void nvme_debug_release_ctrl(struct device *dev);
static struct nvme_debug_ns *nvme_debug_create_ns
	(struct nvme_debug_ctrl *ndc, gfp_t flags, u32 nsid);

int nvme_debug_reg_read32(struct nvme_ctrl *ctrl, u32 off, u32 *val);
int nvme_debug_reg_write32(struct nvme_ctrl *ctrl, u32 off, u32 val);
int nvme_debug_reg_read64(struct nvme_ctrl *ctrl, u32 off, u64 *val);
static void nvme_debug_free_ctrl(struct nvme_ctrl *ctrl);
static int nvme_debug_reset_ctrl(struct nvme_ctrl *ctrl);
static void nvme_debug_submit_async_event(struct nvme_ctrl *ctrl, int aer_idx);
static int nvme_debug_reset(struct nvme_debug_ctrl *ndc);
static void nvme_debug_reset_work(struct work_struct *work);
static int nvme_debug_configure_admin_queue(struct nvme_debug_ctrl *ndc);
static void nvme_debug_ctrl_disable(struct nvme_debug_ctrl *ndc, bool shutdown);
static int nvme_debug_ctrl_enable(struct nvme_dev *dev);
static struct nvme_debug_queue *nvme_debug_alloc_queue
	(struct nvme_debug_ctrl *ndc, int qid, int depth, int node);
static int nvme_debug_alloc_sq_cmds(struct nvme_debug_queue *nvmeq,
				    int qid, int depth, int node);
static void nvme_debug_init_queue(struct nvme_debug_queue *nvmeq, u16 qid);


static char nd_proc_name[] = _MY_NAME;

static LIST_HEAD(ndc_list);
static DEFINE_SPINLOCK(ndc_list_lock);
static unsigned int nd_ctrl_num = _DEFAULT_CTRL_NUM ; /* number of control */

/*
 * Note: The following array creates attribute files in the
 * /sys/bus/pseudo_nvme_debug/drivers/nvme_debug directory. The advantage of
 * these files (over those found in the /sys/module/nvme_debug/parameters
 * directory) is that auxiliary actions can be triggered when an attribute is
 * changed. For example see: nvme_debug_ctrl_count_store() above.
 */
static struct attribute *nd_drv_attrs[] = {
	&driver_attr_ctrl_num.attr,
	NULL,
};
ATTRIBUTE_GROUPS(nd_drv);

static struct device *nd_pseudo_primary = NULL;

static struct bus_type nd_pseudo_lld_bus = {
	.name = "pseudo_nvme_debug",
	.match = nvme_debug_lld_bus_match,
	.probe = nvme_debug_driver_probe,
	.remove = nvme_debug_driver_remove,
	.drv_groups = nd_drv_groups,
};

static struct device_driver nd_driverfs_driver = {
	.name 		= nd_proc_name,
	.bus		= &nd_pseudo_lld_bus,
};

static struct kernel_param_ops nd_param_ops_ctrl_num =
{
	.set = nvme_debug_ctrl_num_param_set,
	.get = param_get_uint,
};

static const struct nvme_ctrl_ops nvme_debug_ctrl_ops = {
	.name			= "debug",
	.module			= THIS_MODULE,
	.reg_read32		= nvme_debug_reg_read32,
	.reg_write32		= nvme_debug_reg_write32,
	.reg_read64		= nvme_debug_reg_read64,
	.reset_ctrl		= nvme_debug_reset_ctrl,
	.free_ctrl		= nvme_debug_free_ctrl,
	.submit_async_event	= nvme_debug_submit_async_event,
};

struct blk_mq_ops nvme_debug_mq_admin_ops = {
	.queue_rq	= nvme_debug_blk_mq_queue_rq,
	.timeout	= nvme_debug_blk_mq_timeout,
	.complete	= nvme_debug_blk_mq_complete,
	.init_hctx	= nvme_debug_blk_mq_init_admin_hctx,
	.init_request	= nvme_debug_blk_mq_init_admin_request,
	.exit_request	= nvme_debug_blk_mq_exit_admin_request,
	.reinit_request	= nvme_debug_blk_mq_reinit_request,
};

struct blk_mq_ops nvme_debug_blk_mq_mq_ops = {
	.queue_rq	= nvme_debug_blk_mq_queue_rq,
	.timeout	= nvme_debug_blk_mq_timeout,
	.poll		= nvme_debug_blk_mq_poll,
	.complete	= nvme_debug_blk_mq_complete,
	.init_hctx	= nvme_debug_blk_mq_init_hctx,
	.init_request	= nvme_debug_blk_mq_init_request,
	.exit_request	= nvme_debug_blk_mq_exit_request,
	.reinit_request	= nvme_debug_blk_mq_reinit_request,
};

// Function code begin

static int nvme_debug_lld_bus_match(struct device *dev,
				    struct device_driver *dev_driver)
{
	_DEBUG_PRINT_FUNC_NAME();
	return 1;
	/* Always indicate current driver can support device on this bus */
}

static int nvme_debug_driver_probe(struct device *dev)
{
	int ret = 0;
	struct nvme_debug_ctrl *ndc = NULL;

	_DEBUG_PRINT_FUNC_NAME();

	ndc = _dev_to_ndc(dev);
	if (WARN_ON(ndc == NULL))
		return 0;

	node = dev_to_node(&dev);
	if (node == NUMA_NO_NODE)
		set_dev_node(&dev, first_memory_node);

	nvme_debug_reg_data_init(&(ndc->reg_data));

	INIT_LIST_HEAD(&ndc->ndn_list);

	ndc->queue_count = _NVME_DEBUG_QUEUE_COUNT + 1 /* 1 for admin queue */;
	ndc->queues = kzalloc(ndc->queue_count *
			      sizeof(struct nvme_debug_queue *),
			      GFP_KERNEL);
	if (ndc->queues == NULL)
		goto out;

	INIT_WORK(&ndc->reset_work, nvme_debug_reset_work);

	ret = nvme_init_ctrl(&ndc->ctrl, dev, &nvme_debug_ctrl_ops,
			       0 /* No quirk */);
	if (ret != 0)
		goto out;

	nvme_change_ctrl_state(&ndc->ctrl, NVME_CTRL_RESETTING);
	dev_info(ndc->ctrl.device, "scsi_debug %s\n", dev_name(&dev));

	queue_work(nvme_workq, &ndc->reset_work);

out:
	if (ret != 0) {
		pr_debug("nvme_debug_driver_probe failed %d\n", ret);
		nvme_uninit_ctrl(&ndc->ctrl);
		nvme_put_ctrl(&ndc->ctrl);
	}
	return ret;
}

static int nvme_debug_driver_remove(struct device *dev)
{
	_DEBUG_PRINT_FUNC_NAME();
	return 0;
}

static ssize_t nvme_debug_ctrl_num_show(struct device_driver *ddp, char *buf)
{
	_DEBUG_PRINT_FUNC_NAME();
	return scnprintf(buf, PAGE_SIZE, "%d\n", nd_ctrl_num);
}

static ssize_t nvme_debug_ctrl_num_store(struct device_driver *ddp,
					 const char *buf, size_t count)
{
	int ret = 0;

	_DEBUG_PRINT_FUNC_NAME();

	_call_and_check_return(kstrtouint(buf, 0 /* base -- auto detect */,
					  &nd_ctrl_num), ret);
	_call_and_check_return(nvme_debug_ctrl_num_change(), ret);

	return count;
}

static int nvme_debug_bus_register(void)
{
	int ret = 0;

	_DEBUG_PRINT_FUNC_NAME();

	nd_pseudo_primary = root_device_register("pseudo_0_nvme_debug");
	if (IS_ERR(nd_pseudo_primary)) {
		pr_warn("root_device_register() error\n");
		ret = PTR_ERR(nd_pseudo_primary);
		return ret;
	}
	ret = bus_register(&nd_pseudo_lld_bus);
	if (ret < 0) {
		pr_warn("bus_register error: %d\n", ret);
		goto dev_unreg;
	}
	ret = driver_register(&nd_driverfs_driver);
	if (ret < 0) {
		pr_warn("driver_register error: %d\n", ret);
		goto bus_unreg;
	}
	return ret;

bus_unreg:
	bus_unregister(&nd_pseudo_lld_bus);

dev_unreg:
	root_device_unregister(nd_pseudo_primary);

	nd_pseudo_primary = NULL;

	return ret;
}

static int nvme_debug_ctrl_num_change(void)
{
	_DEBUG_PRINT_FUNC_NAME();
	// TODO

	return 0;
}

int __init nvme_debug_init(void)
{
	int ret = 0;
	unsigned int nd_ctrl_to_add = nd_ctrl_num;
	unsigned int i = 0;

	nd_ctrl_num = 0;

	_DEBUG_PRINT_FUNC_NAME();

	_call_and_check_return(nvme_debug_bus_register(), ret);

	for (i = 0; i < nd_ctrl_to_add; ++i) {
		if (nvme_debug_add_ctrl() != 0) {
			pr_err("nvme_debug_add_ctrl failed i=%d\n", i);
			break;
		}
	}

	return ret;
}

void __exit nvme_debug_exit(void)
{
	int i = nd_ctrl_num;

	_DEBUG_PRINT_FUNC_NAME();

	nvme_debug_remove_all_ctrls();

	driver_unregister(&nd_driverfs_driver);
	bus_unregister(&nd_pseudo_lld_bus);
	if (nd_pseudo_primary != NULL)
		root_device_unregister(nd_pseudo_primary);
}

static int nvme_debug_ctrl_num_param_set(const char *val,
					 const struct kernel_param *kp)
{
	int ret = 0;

	_DEBUG_PRINT_FUNC_NAME();

	ret = param_set_uint(val, kp);
	if (ret)
		return ret;

	return nvme_debug_ctrl_num_change();
}

static int nvme_debug_add_ctrl(void)
{
	int ret = -ENOMEM;
	struct nvme_debug_ctrl *ndc = NULL;

	_DEBUG_PRINT_FUNC_NAME();

	ndc = kzalloc(sizeof(struct nvme_debug_ctrl), GFP_KERNEL);
	_mem_alloc_check(ndc, ret, out);

	ndc->dev.bus = &nd_pseudo_lld_bus;
	ndc->dev.parent = nd_pseudo_primary;
	ndc->dev.release = &nvme_debug_release_ctrl;
	dev_set_name(&ndc->dev, "controller_%d", nd_ctrl_num);

	spin_lock(&ndc_list_lock);
	list_add_tail(&nc->ndc_list, &ndc_list);
	spin_unlock(&ndc_list_lock);

	ret = device_register(&ndc->dev);
	if (ret != 0)
		goto out;

	++nd_ctrl_num;
	ret = 0;

out:
	if (ret != 0) {
#if 0
		list_for_each_entry_safe(ndn, tmp_ndn,
					 &ndc->ndn_list,
					 ndn_list) {
			list_del(&ndn->ndn_list);
			kfree(ndn);
		}
#endif
		if (ndc->queues != NULL)
			kfree(ndc->queues);
		kfree(ndc);
	}
	return ret;
}

static void nvme_debug_release_ctrl(struct device *dev)
{
	struct nvme_debug_ctrl *ndc = NULL;

	_DEBUG_PRINT_FUNC_NAME();

	ndc = _dev_to_ndc(dev);
	if (WARN_ON(ndc == NULL))
		return;

	kfree(ndc);
}

static void nvme_debug_remove_all_ctrls(void)
{
	struct nvme_debug_ctrl *ndc = top_ndc;
	_DEBUG_PRINT_FUNC_NAME();

	/* BUG(Gris Ge): Assuming only have 1 controller */

	if (ndc == NULL)
		return;
	nvme_change_ctrl_state(&ndc->ctrl, NVME_CTRL_DELETING);

	cancel_work_sync(&ndc->reset_work);
	flush_work(&ndc->reset_work);
	nvme_uninit_ctrl(&ndc->ctrl);
	nvme_debug_ctrl_disable(ndc, true);
//	nvme_dev_remove_admin(dev);
//	nvme_free_queues(ndc, 0);
//	nvme_release_prp_pools(dev);
//	nvme_dev_unmap(dev);
	nvme_put_ctrl(&ndc->ctrl);

	device_unregister(&ndc->dev);
}

static struct nvme_debug_ns *nvme_debug_create_ns
	(struct nvme_debug_ctrl *ndc, gfp_t flags, unsigned int nsid)
{
	struct nvme_debug_ns *ndn = NULL;

	_DEBUG_PRINT_FUNC_NAME();
	ndn = kzalloc(sizeof(struct nvme_debug_ns), flags);
	if (ndn == NULL)
		return NULL;

	uuid_le_gen((uuid_le *) &ndn->ns.uuid);
	ndn->ns.ns_id = nsid;
	ndn->ndc = ndc;
	list_add_tail(&ndn->ndn_list, &ndc->ndn_list);
	return ndn;
};

static void nvme_debug_free_ctrl(struct nvme_ctrl *ctrl)
{
	struct nvme_debug_ctrl *ndc = _ctrl_to_ndc(ctrl);
	u32 i = 0;

	_DEBUG_PRINT_FUNC_NAME();
	if (ndc->queues != NULL) {
		for (; i < ndc->queue_count; ++i) {
			kfree(ndc->queues[i]);
		}
		kfree(ndc->queues);
	}
	put_device(&ndc->dev);
	kfree(ndc);
}

static int nvme_debug_reset_ctrl(struct nvme_ctrl *ctrl)
{
	struct nvme_debug_ctrl *ndc = _ctrl_to_ndc(ctrl);
	int ret = nvme_debug_reset(ndc);

	_DEBUG_PRINT_FUNC_NAME();
	if (!ret)
		flush_work(&ndc->reset_work);
	return ret;
}

static int nvme_debug_reset(struct nvme_debug_ctrl *ndc)
{
	_DEBUG_PRINT_FUNC_NAME();
	if (!ndc->ctrl.admin_q || blk_queue_dying(ndc->ctrl.admin_q))
		return -ENODEV;
	if (work_busy(&ndc->reset_work))
		return -ENODEV;
	if (!queue_work(nvme_workq, &ndc->reset_work))
		return -EBUSY;
	return 0;
}

static void nvme_debug_submit_async_event(struct nvme_ctrl *ctrl, int aer_idx)
{
	_DEBUG_PRINT_FUNC_NAME();
}

static void nvme_debug_reset_work(struct work_struct *work)
{
	struct nvme_debug_ctrl *ndc = NULL;
	int ret = 0;

	_DEBUG_PRINT_FUNC_NAME();

	ndc = _reset_work_to_ndc(work);
	if (WARN_ON(ndc->ctrl.state == NVME_CTRL_RESETTING))
		goto out;

	/*
	 * If we're called to reset a live controller first shut it down before
	 * moving on.
	 */
	if (dev->ctrl.ctrl_config & NVME_CC_ENABLE)
		nvme_debug_ctrl_disable(ndc, false);

	/* configure admin queue */
	ret = nvme_debug_configure_admin_queue(ndc);
	if (ret != 0)
		goto out;

	ret = nvme_debug_alloc_admin_tags(ndc);
	if (ret)
		goto out;

	ret = nvme_init_identify(&ndc->ctrl);
	if (ret != 0)
		goto out;

	/* TODO(Gris Ge): OPAL and host buffer feature here. */

	ret = nvme_debug_setup_io_queues(ndc);
	if (ret != 0)
		goto out;

	if (ndc->online_queues < 2) {
		dev_warn(ndc->ctrl.device, "IO queues not created\n");
		nvme_kill_queues(&ndc->ctrl);
		nvme_remove_namespaces(&ndc->ctrl);
	} else {
		nvme_start_queues(&ndc->ctrl);
		nvme_wait_freeze(&ndc->ctrl);
		nvme_debug_blk_mq_dev_add(ndc);
		nvme_unfreeze(&ndc->ctrl);
	}

	if (!nvme_change_ctrl_state(&ndc->ctrl, NVME_CTRL_LIVE)) {
		dev_warn(ndc->ctrl.device, "failed to mark controller live\n");
		goto out;
	}

	nvme_start_ctrl(&ndc->ctrl);

out:
	/* We don't have dead controller in nvme_debug */
	if (ret != 0)
		dev_warn(ndc->ctrl.device, "nvme_debug_reset_work() failed %d",
			 ret);
	return;
}

static int nvme_debug_configure_admin_queue(struct nvme_debug_ctrl *ndc)
{
	int result = 0;
	u32 aqa = 0; // Admin Queue attributes
	struct nvme_queue *nvmeq = NULL;

	_DEBUG_PRINT_FUNC_NAME();

	result = nvme_disable_ctrl(&ndc->ctrl, ndc->ctrl.cap);
	if (result < 0)
		return result;

	nvmeq = ndc->queues[0];

	if (nvmeq == NULL) {
		nvmeq = nvme_debug_alloc_queue(ndc, 0, NVME_AQ_DEPTH,
					       dev_to_node(ndc->dev));
		if (!nvmeq)
			return -ENOMEM;
	}

	aqa = nvmeq->q_depth - 1;
	aqa |= aqa << 16; // submission and completion queue are the same size

	nvme_debug_reg_write32(ndc->ctrl, NVME_REG_AQA, aqa);
	nvme_debug_reg_write32(ndc->ctrl, NVME_REG_ASQ, nvmeq->asqb);
	nvme_debug_reg_write32(ndc->ctrl, NVME_REG_ASQ + 4, nvmeq->asqb >> 32);
	nvme_debug_reg_write32(ndc->ctrl, NVME_REG_ACQ, nvmeq->acqb);
	nvme_debug_reg_write32(ndc->ctrl, NVME_REG_ACQ + 4, nvmeq->acqb >> 32);

	result = nvme_enable_ctrl(&dev->ctrl, dev->ctrl.cap);
	if (result)
		return result;

	nvmeq->cq_vector = 0;
	nvme_debug_init_queue(nvmeq, 0);
	// TODO(Gris Ge): Do we have to do IRQ on completion queue?

	return result;
}

static void nvme_debug_ctrl_disable(struct nvme_debug_ctrl *ndc, bool shutdown)
{
	_DEBUG_PRINT_FUNC_NAME();

	mutex_lock(&dev->shutdown_lock);

	nvme_start_freeze(&ndc->ctrl);

	if (shutdown)
		nvme_wait_freeze_timeout(&ndc->ctrl, NVME_IO_TIMEOUT);

	nvme_stop_queues(&ndc->ctrl);
	queues = dev->online_queues - 1;
	for (i = dev->ctrl.queue_count - 1; i > 0; i--)
		nvme_suspend_queue(dev->queues[i]);

	nvme_debug_disable_io_queues(ndc, queues);
	nvme_debug_disable_admin_queue(ndc, shutdown);

	// TODO(Gris Ge): Should set host memory buffer to 0 if supported.

	blk_mq_tagset_busy_iter(&ndc->tag_set, nvme_cancel_request, &dev->ctrl);
	blk_mq_tagset_busy_iter(&ndc->admin_tag_set, nvme_cancel_request,
				&ndc->ctrl);

	if (shutdown)
		nvme_start_queues(&ndc->ctrl);

	/* Re-enable the queue to prevent dead-lock */
	mutex_unlock(&dev->shutdown_lock);
}

static void nvme_debug_disable_io_queues(struct nvme_debug_ctrl *ndc,
					 int queues)
{
	_DEBUG_PRINT_FUNC_NAME();
}

static void nvme_debug_disable_admin_queue(struct nvme_debug_ctrl *ndc,
					   bool shutdown)
{
	_DEBUG_PRINT_FUNC_NAME();
}

static int nvme_debug_alloc_admin_tags(struct nvme_debug_ctrl *ndc)
{
	_DEBUG_PRINT_FUNC_NAME();

	if (!ndc->ctrl.admin_q) {
		ndc->admin_tagset.ops = &nvme_debug_mq_admin_ops;
		ndc->admin_tagset.nr_hw_queues = 1;

		/*
		 * Subtract one to leave an empty queue entry for 'Full Queue'
		 * condition. See NVM-Express 1.2 specification, section 4.1.2.
		 */
		ndc->admin_tagset.queue_depth = NVME_AQ_BLKMQ_DEPTH - 1;
		ndc->admin_tagset.timeout = ADMIN_TIMEOUT;
		ndc->admin_tagset.numa_node = dev_to_node(ndc->dev);
		ndc->admin_tagset.cmd_size = nvme_debug_cmd_size(ndc); //CODING
		ndc->admin_tagset.flags = BLK_MQ_F_NO_SCHED;
		ndc->admin_tagset.driver_data = ndc;

		if (blk_mq_alloc_tag_set(&ndc->admin_tagset))
			return -ENOMEM;
		ndc->ctrl.admin_tagset = &ndc->admin_tagset;

		ndc->ctrl.admin_q = blk_mq_init_queue(&ndc->admin_tagset);
		if (IS_ERR(ndc->ctrl.admin_q)) {
			blk_mq_free_tag_set(&ndc->admin_tagset);
			return -ENOMEM;
		}
		if (!blk_get_queue(ndc->ctrl.admin_q)) {
			nvme_dev_remove_admin(dev);
			ndc->ctrl.admin_q = NULL;
			return -ENODEV;
		}
	} else
		blk_mq_unquiesce_queue(ndc->ctrl.admin_q);

	return 0;
}

static int nvme_debug_setup_io_queues(struct nvme_debug_ctrl *ndc)
{
	_DEBUG_PRINT_FUNC_NAME();
	return 0;
}

static int nvme_debug_blk_mq_dev_add(struct nvme_debug_ctrl *ndc)
{
	_DEBUG_PRINT_FUNC_NAME();
	return 0;
}

static struct nvme_debug_queue *nvme_debug_alloc_queue
	(struct nvme_debug_ctrl *ndc, int qid, int depth, int node)
{
	struct nvme_debug_queue *nvmeq = NULL;

	_DEBUG_PRINT_FUNC_NAME();

	nvmeq = kzalloc_node(sizeof(*nvmeq), GFP_KERNEL, node);
	if (!nvmeq)
		return NULL;

	nvmeq->cqes = kzalloc_node(CQ_SIZE(depth), GFP_KERNEL, node);
	if (!nvmeq->cqes)
		goto free_nvmeq;

	if (nvme_debug_alloc_sq_cmds(nvmeq, qid, depth, node))
		goto free_cqes;

	nvmeq->ndc = dev;
	spin_lock_init(&nvmeq->q_lock);
	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	nvmeq->q_depth = depth;
	nvmeq->qid = qid;
	nvmeq->cq_vector = -1;
	nvmeq->asqb = nvmeq->cqes;
	nvmeq->acqb = nvmeq->sq_cmds;
	ndc->queues[qid] = nvmeq;
	ndc->ctrl.queue_count++;

	return nvmeq;

free_cqes:
	kfree(nvmeq->cqes);

free_nvmeq:
	kfree(nvmeq);
	return NULL;
}

static int nvme_debug_alloc_sq_cmds(struct nvme_debug_queue *nvmeq,
				    int qid, int depth, int node)
{
	_DEBUG_PRINT_FUNC_NAME();

	// TODO(Gris Ge): Support controller memory buffer here for command
	//		  queue.
	nvmeq->sq_cmds = kzalloc_node(SQ_SIZE(depth), GFP_KERNEL, node);
	if (!nvmeq->sq_cmds)
		return -ENOMEM;

	return 0;
}

static void nvme_init_queue(struct nvme_queue *nvmeq, u16 qid)
{
	_DEBUG_PRINT_FUNC_NAME();
	struct nvme_debug_ctrl *ndc = nvmeq->ndc;

	spin_lock_irq(&nvmeq->q_lock);
	nvmeq->sq_tail = 0;
	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	memset((void *)nvmeq->cqes, 0, CQ_SIZE(nvmeq->q_depth));
	ndc->online_queues++;
	spin_unlock_irq(&nvmeq->q_lock);
}

/*
 * Note: The following macros create attribute files in the
 * /sys/module/scsi_debug/parameters directory. Unfortunately this
 * driver is unaware of a change and cannot trigger auxiliary actions
 * as it can when the corresponding attribute in the
 * /sys/bus/pseudo/drivers/scsi_debug directory is changed.
 */
module_param_cb(ctrl_num, &nd_param_ops_ctrl_num, &nd_ctrl_num,
		S_IRUGO | S_IWUSR);
module_init(nvme_debug_init);
module_exit(nvme_debug_exit);

MODULE_PARM_DESC(ctrl_num, "number of NVMe controller (def=1)");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Gris Ge <fge@redhat.com>");
