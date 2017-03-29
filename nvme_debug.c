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
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include "nvme.h"

#define _MY_NAME			"nvme_debug"
#define _DEFAULT_CTRL_NUM		1

#define _NVME_MAX_CTRL_ID		0xffff

#define _mem_check_return(__ptr_to_check) \
	do { \
		if (__ptr_to_check == NULL) { \
			pr_err("Out of memory at file %s, func %s, line %d\n", \
			       __FILE__, __FUNCTION__, __LINE__); \
			return -ENOMEM; \
		} \
	} while(0) \

#define _call_and_check_return(__ret_val, __ret) \
	do { \
		__ret = __ret_val; \
		if (__ret == 0) \
			return ret; \
	} while(0) \

#define _DEBUG_PRINT_FUNC_NAME() \
	do { \
		pr_debug("%s()\n", __FUNCTION__); \
	} while(0)

#define _to_cl_info(d)	\
	container_of(d, struct nvme_debug_ctrl_info, dev)

struct nvme_debug_ctrl_info {
	struct list_head cl_info_list;
	struct nvme_ctrl cl;
	struct device dev;
	struct list_head ns_info_list;
};

struct nvme_debug_ns_info {
	struct list_head ns_info_list;
	struct nvme_ns ns;
	u32 ns_id;
	struct nvme_debug_ctrl_info *cl_info;
};

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
static void nvme_debug_release_ctrl(struct device *dev);
static struct nvme_debug_ns_info *nvme_debug_create_ns
	(struct nvme_debug_ctrl_info *cl_info, gfp_t flags);

static char nd_proc_name[] = _MY_NAME;
static unsigned int nd_ctrl_num = _DEFAULT_CTRL_NUM ; /* number of control */
static LIST_HEAD(nd_cl_info_list);
static DEFINE_SPINLOCK(nd_cl_info_list_lock);

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
	_DEBUG_PRINT_FUNC_NAME();

//	device_register
//	nvme_init_ctrl
	return 0;
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

void nvme_debug_exit(void)
{
	_DEBUG_PRINT_FUNC_NAME();

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
	int error = 0;
	struct nvme_debug_ctrl_info *cl_info = NULL;
	struct nvme_debug_ns_info *ns_info = NULL;
	struct nvme_debug_ns_info *tmp_ns_info = NULL;

	cl_info = kzalloc(sizeof(struct nvme_debug_ctrl_info), GFP_KERNEL);
	_mem_check_return(cl_info);

	INIT_LIST_HEAD(&cl_info->ns_info_list);

	/* TODO: Only create 1 namespace here, need to expand it to configurable
	 */

	ns_info = nvme_debug_create_ns(cl_info, GFP_KERNEL);
	_mem_check_return(ns_info);

	spin_lock(&nd_cl_info_list_lock);
	list_add_tail(&cl_info->cl_info_list, &nd_cl_info_list);
	spin_unlock(&nd_cl_info_list_lock);

	cl_info->dev.bus = &nd_pseudo_lld_bus;
	cl_info->dev.parent = nd_pseudo_primary;
	cl_info->dev.release = &nvme_debug_release_ctrl;
	dev_set_name(&cl_info->dev, "controller_%d", nd_ctrl_num);

	error = device_register(&cl_info->dev);

	if (error)
		goto clean;

	++nd_ctrl_num;
	return error;

clean:
	list_for_each_entry_safe(ns_info, tmp_ns_info, &cl_info->ns_info_list,
				 ns_info_list) {
		list_del(&ns_info->ns_info_list);
		kfree(ns_info);
	}

	kfree(cl_info);
	return error;
}


static void nvme_debug_release_ctrl(struct device *dev)
{
	struct nvme_debug_ctrl_info *cl_info;

	cl_info = _to_cl_info(dev);
	kfree(cl_info);
}

static struct nvme_debug_ns_info *nvme_debug_create_ns
	(struct nvme_debug_ctrl_info *cl_info, gfp_t flags)
{
	/* TODO: Need to handle kernel parameter for namespace count. */
	return NULL;
};

ATTRIBUTE_GROUPS(nd_drv);
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
