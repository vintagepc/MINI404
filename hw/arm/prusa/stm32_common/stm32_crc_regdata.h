#ifndef STM32_COMMON_CRC_DATA_H
#define STM32_COMMON_CRC_DATA_H

#include "stm32_shared.h"

enum regIndex
{
	RI_DR                   = (0x00 /4U), /* CRC Data register, */
	RI_IDR                  = (0x04 /4U), /* CRC Independent data register, */
	RI_CR                   = (0x08 /4U), /* CRC Control register, */
	RI_INIT                 = (0x10 /4U), /* Initial CRC value register, */
	RI_POL                  = (0x14 /4U), /* CRC polynomial register, */
	RI_END
};

#include "../stm32_registers/generated/stm32f030/CRC_reginfo.h"

#include "../stm32_registers/generated/stm32g070/CRC_reginfo.h"

#include "../stm32_registers/generated/stm32h503/CRC_reginfo.h"

#include "../stm32_registers/generated/stm32f427/CRC_reginfo.h"

#endif
