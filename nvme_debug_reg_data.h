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

#ifndef _NVME_DEBUG_REG_DATA_H
#define _NVME_DEBUG_REG_DATA_H

struct nvme_debug_reg_data;

void nvme_debug_reg_data_init(struct nvme_debug_reg_data *reg_data);

#endif /* _NVME_DEBUG_REG_DATA_H */
