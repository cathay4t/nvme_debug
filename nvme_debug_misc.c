/*
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

#include <linux/types.h>
#include <linux/bug.h>
#include <linux/kernel.h>

#include "nvme_debug_spec.h"

void _set_bits_u8(u8 *data, u32 end, u32 start, u8 value)
{
	u32 start_bit = start % 8;
	u32 start_byte = start / 8;
	u32 end_byte = end / 8;
	u8 max_value = 0;

	WARN_ON(start > end);
	WARN_ON(end >= start + 8);

	if (end_byte == start_byte) {
		max_value = ((1 << (end - start + 1)) - 1) << start_bit;
		value = (value << start_bit ) & max_value;
		// clear those bits
		data[start_byte] &= ~max_value;
		// set those bits
		data[start_byte] |= value;
	} else {
		_set_bits_u8(data, end_byte * 8 - 1, start, value);
		_set_bits_u8(data, end, end_byte * 8, value >> (8 - start_bit));
	}
}

void _set_bits_u16_le(u8 *data, u32 end, u32 start, u16 value)
{
	if ((end - start) <= 8)
		return _set_bits_u8(data, end, start, value & U8_MAX);

	_set_bits_u8(data, start + 7, start, value & U8_MAX);
	_set_bits_u8(data, end, start + 8, value >> 8);
}

void _set_bits_u32_le(u8 *data, u32 end, u32 start, u32 value)
{
	if ((end - start) <= 16)
		return _set_bits_u16_le(data, end, start, value & U16_MAX);

	_set_bits_u16_le(data, start + 15, start, value & U16_MAX);
	_set_bits_u16_le(data, end, start + 16, value >> 16);
}

void _set_bits_u64_le(u8 *data, u32 end, u32 start, u64 value)
{
	if ((end - start) <= 32)
		return _set_bits_u32_le(data, end, start, value & U32_MAX);

	_set_bits_u32_le(data, start + 31, start, value & U32_MAX);
	_set_bits_u32_le(data, end, start + 32, value >> 32);
}

u8 _get_bits_u8(u8 *data, u32 end, u32 start)
{
	u32 start_bit = start % 8;
	u32 start_byte = start / 8;
	u32 end_byte = end / 8;
	u8 max_value = 0;
	u8 ret = 0;

	if (end >= start + 8)
		return 0;

	max_value = (1 << (end - start + 1)) - 1;
	if (end_byte == start_byte)
		ret = (data[start_byte] >> start_bit) & max_value;
	else
		ret =_get_bits_u8(data, end_byte * 8 - 1, start) +
			((_get_bits_u8(data, end, end_byte * 8)
			  << (8 - start_bit)));

	return ret;
}
