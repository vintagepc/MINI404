#ifndef STM32_SHARED_H
#define STM32_SHARED_H

#include "qemu/osdep.h"

/* Used for uniquely identifying a peripheral */
typedef uint32_t stm32_periph_t;
#define DEFINE_PROP_PERIPH_T DEFINE_PROP_UINT32
#define QDEV_PROP_SET_PERIPH_T qdev_prop_set_uint32

// Struct for managing info on register differences between
// chips that have the same layout but different features.
typedef struct stm32_reginfo_t
{
	const uint32_t mask; // Read/write mask to prevent modifying reserved bits.
	const bool is_reserved; // Is this register usable in this flavour?
	const uint32_t reset_val; // Reset value of register.
	const uint32_t unimp_mask; // Mask for unimplemented bits (convenience func on register write)

} stm32_reginfo_t;

enum {
    STM32_P_UNDEFINED = -1,
    STM32_P_RCC = 0,
    STM32_P_GPIOA,
    STM32_P_GPIOB,
    STM32_P_GPIOC,
    STM32_P_GPIOD,
    STM32_P_GPIOE,
    STM32_P_GPIOF,
    STM32_P_GPIOG,
    STM32_P_GPIOH,
    STM32_P_GPIOI,
    STM32_P_GPIOJ,
    STM32_P_GPIOK,
    STM32_P_SYSCFG,
    STM32_P_AFIO,
    STM32_P_UART1,
    STM32_P_UART2,
    STM32_P_UART3,
    STM32_P_UART4,
    STM32_P_UART5,
    STM32_P_UART6,
    STM32_P_UART7,
    STM32_P_UART8,
    STM32_P_ADC_ALL, // special common ADC for shared reset.
    STM32_P_ADC1,
    STM32_P_ADC2,
    STM32_P_ADC3,
    STM32_P_ADCC,
    STM32_P_DAC,
    STM32_P_TIM1,
    STM32_P_TIM2,
    STM32_P_TIM3,
    STM32_P_TIM4,
    STM32_P_TIM5,
    STM32_P_TIM6,
    STM32_P_TIM7,
    STM32_P_TIM8,
    STM32_P_TIM9,
    STM32_P_TIM10,
    STM32_P_TIM11,
    STM32_P_TIM12,
    STM32_P_TIM13,
    STM32_P_TIM14,
    STM32_P_TIM15,
    STM32_P_TIM16,
    STM32_P_TIM17,
    STM32_P_BKP,
    STM32_P_PWR,
    STM32_P_I2C1,
    STM32_P_I2C2,
    STM32_P_I2C3,
    STM32_P_I2C4,
    STM32_P_I2S1,
    STM32_P_I2S2,
    STM32_P_I2S3,
    STM32_P_IWDG,
    STM32_P_WWDG,
    STM32_P_CAN1,
    STM32_P_CAN2,
    STM32_P_CAN,
    STM32_P_USB,
    STM32_P_USB2,
    STM32_P_SPI1,
    STM32_P_SPI2,
    STM32_P_SPI3,
    STM32_P_SPI4,
    STM32_P_SPI5,
    STM32_P_SPI6,
    STM32_P_EXTI,
    STM32_P_SDIO,
    STM32_P_FSMC,
    STM32_P_RTC,
    STM32_P_CRC,
	STM32_P_DMAMUX,
    STM32_P_DMA1,
    STM32_P_DMA2,
    STM32_P_DCMI,
    STM32_P_CRYP,
    STM32_P_HASH,
    STM32_P_RNG,
    STM32_P_QSPI,
    STM32_P_LPTIM1,
    STM32_P_COUNT,
};

QEMU_BUILD_BUG_MSG(STM32_P_COUNT>=256,"Err - peripheral reset arrays not meant to handle >255 peripherals!");

#define _CHECK_BOUNDS(index, val, max, reginfo, descr, type) \
    if (index >= max || reginfo[index].is_reserved) { \
        qemu_log_mask(LOG_GUEST_ERROR, "Register "type" 0x%x (val 0x%"PRIx64") invalid on "descr"\n", \
          (unsigned int)index << 2, val);

// Checks whether a register access is in-bounds and the register is not fully reserved.
// Issues a GUEST_ERROR if it is.
#define CHECK_BOUNDS_W(index, val, max, reginfo, descr) \
    _CHECK_BOUNDS(index, val, max, reginfo, descr, "write") \
		return; \
	}

// Checks whether a register access is in-bounds and the register is not fully reserved.
// Issues a GUEST_ERROR if it is.
#define CHECK_BOUNDS_R(index, max, reginfo, descr) \
 	_CHECK_BOUNDS(index, 0xBADULL, max, reginfo, descr, "read") \
	 	return 0; \
	}

// Enforces the reserved mask for a register
#define ENFORCE_RESERVED(val, reginfo, index) \
	if (val & (~reginfo[index].mask)) { \
		qemu_log_mask(LOG_GUEST_ERROR, "Attempted to alter a reserved bit in "#index"!\n"); \
	} \
	val = val & reginfo[index].mask;

// Checks if a write attempts to set any unimplemented bits and issues a LOG_UNIMP.
#define CHECK_UNIMP(val, reginfo, index) \
if (val & (reginfo[index].unimp_mask)) { \
	qemu_log_mask(LOG_UNIMP, "Modified unimplemented field (mask: 0x%"PRIx32" value: 0x%"PRIx64") "#index"!\n", reginfo[index].unimp_mask, val); \
}

// Enforces reserved bits and then checks for unimplemented ones
#define CHECK_UNIMP_RESVD(val, reginfo, index) \
	ENFORCE_RESERVED(val, reginfo, index); \
	CHECK_UNIMP(val, reginfo, index);

// Adjusts for read offsets and sizes given old and new register values.
// Allowed is a bitmask of sizes, e.g:
// 7 = 4,2, and 1 byte is allowed
// 6 = 4 and 2 bytes only, etc.
// disallowed writes are not blocked but log a guest error.
#define ADJUST_FOR_OFFSET_AND_SIZE_W(old, new, size, off, allowed) \
	if (!(size & allowed)) {\
		qemu_log_mask(LOG_GUEST_ERROR, "Disallowed register write of size %d in %s!\n", size, __FILE__); \
	} \
    switch (size) { \
        case 1: \
			new = (old & ~(0xff << (off * 8))) | new << (off * 8); \
            break; \
        case 2: \
            new = (old & ~(0xffff << (off * 8))) | new << (off * 8); \
            break; \
        case 4: \
            break; \
        default: \
            abort(); \
    }

// Adjusts for read offsets and sizes given old and new register values.
// Allowed is a bitmask of sizes, e.g:
// 7 = 4,2, and 1 byte is allowed
// 6 = 4 and 2 bytes only, etc.
// disallowed writes are not blocked but log a guest error.
#define ADJUST_FOR_OFFSET_AND_SIZE_R(data, size, off, allowed) \
	if (!(size & allowed)) {\
		qemu_log_mask(LOG_GUEST_ERROR, "Disallowed register read of size %d in %s!\n", size, __func__); \
	} \
    switch (size) { \
        case 1: \
			data = (data>>(off*8U)&0xFFU); \
            break; \
        case 2: \
            data = (data>>(off*8U)&0xFFFFU); \
            break; \
        case 4: \
            break; \
        default: \
            abort(); \
    }

#endif //STM32_SHARED_H
