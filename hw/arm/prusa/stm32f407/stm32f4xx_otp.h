/*
    stm32f4xx_otp.h - OTP block for STM32F4xx

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STM32F4XX_OTP_H
#define STM32F4XX_OTP_H
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "exec/memory.h"
#include "sysemu/block-backend.h"


#define OTP_SIZE 0x220/4

#define TYPE_STM32F4XX_OTP "stm32f4xx-otp"
OBJECT_DECLARE_SIMPLE_TYPE(Stm32f4xx_OTP, STM32F4XX_OTP);

typedef struct Stm32f4xx_OTP {
    SysBusDevice  busdev;
    MemoryRegion  iomem;

    BlockBackend *blk;

    uint32_t data[OTP_SIZE];

    bool first_read;

} Stm32f4xx_OTP;

#endif //STM32F4XX_OTP_H
