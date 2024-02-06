#ifndef STM32_COMMON_CRC_DATA_H
#define STM32_COMMON_CRC_DATA_H

#include "stm32_shared.h"

enum regIndex
{
	RI_DR, // 0x00
	RI_IDR,// 0x04
	RI_CR, // 0x08
	RI_RESERVED,
	RI_INIT = (0x10/4),// 0x10
	RI_POL,	// 0x14
	RI_END,
};

static const stm32_reginfo_t stm32f030_crc_reginfo[RI_END] =
{
	[RI_DR] = {.mask = UINT32_MAX, .reset_val = UINT32_MAX},
	[RI_IDR] = {.mask = 0xFF},
	[RI_CR] = {.mask = 0xE1, .unimp_mask = 0xE0},
	[RI_RESERVED] = {.is_reserved = true},
	[RI_INIT] = { .mask = UINT32_MAX, .reset_val = UINT32_MAX},
	[RI_POL] = { .is_reserved = true }
};

static const stm32_reginfo_t stm32g070_crc_reginfo[RI_END] =
{
	[RI_DR] = {.mask = UINT32_MAX, .reset_val = UINT32_MAX},
	[RI_IDR] = {.mask = 0xFF},
	[RI_CR] = {.mask = 0xE1, .unimp_mask = 0xE0},
	[RI_RESERVED] = {.is_reserved = true},
	[RI_INIT] = { .mask = UINT32_MAX, .reset_val = UINT32_MAX},
	[RI_POL] = { .mask = UINT32_MAX, .reset_val = 0x04C11DB7, .unimp_mask = UINT32_MAX }
};

static const stm32_reginfo_t stm32f2xx_crc_reginfo[RI_END] =
{
	[RI_DR] = {.mask = UINT32_MAX, .reset_val = UINT32_MAX},
	[RI_IDR] = {.mask = 0xFF},
	[RI_CR] = {.mask = 0x01},
	[RI_RESERVED] = {.is_reserved = true},
	[RI_INIT] = {.is_reserved = true, .reset_val = UINT32_MAX},
	[RI_POL] = {.is_reserved = true}
};

static const stm32_reginfo_t stm32f4xx_crc_reginfo[RI_END] =
{
	[RI_DR] = {.mask = UINT32_MAX, .reset_val = UINT32_MAX},
	[RI_IDR] = {.mask = 0xFF},
	[RI_CR] = {.mask = 0x01},
	[RI_RESERVED] = {.is_reserved = true},
	[RI_INIT] = {.is_reserved = true, .reset_val = UINT32_MAX},
	[RI_POL] = {.is_reserved = true}
};

#endif
