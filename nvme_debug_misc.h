/*
 * Structs and constants defined by NVMe MISC used by nvme_debug.
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

#ifndef _NVME_DEBUG_MISC_H
#define _NVME_DEBUG_MISC_H

#include <linux/types.h>

void _set_bits_u8(u8 *data, u32 end, u32 start, u8 value);
void _set_bits_u16_le(u8 *data, u32 end, u32 start, u16 value);
void _set_bits_u32_le(u8 *data, u32 end, u32 start, u32 value);
void _set_bits_u64_le(u8 *data, u32 end, u32 start, u64 value);
u8 _get_bits_u8(u8 *data, u32 end, u32 start);

#endif /* _NVME_DEBUG_MISC_H */
