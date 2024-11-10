#ifndef STM32_SHARED_H
#define STM32_SHARED_H

#include "qemu/osdep.h"
#include "qemu/log.h"

/* Used for uniquely identifying a peripheral */
typedef uint32_t stm32_periph_t;
#define DEFINE_PROP_PERIPH_T DEFINE_PROP_UINT32
#define QDEV_PROP_SET_PERIPH_T qdev_prop_set_uint32

// Struct for managing info on register differences between
// chips that have the same layout but different features.
typedef struct stm32_reginfo_t
{
	const uint32_t mask; // Read/write mask to prevent modifying reserved bits.
	const bool not_reserved; // Is this register reserved? (negative so zero-initialized is treated as reserved)
	const bool is_reserved; // Is this register usable in this flavour?
	const uint32_t reset_val; // Reset value of register.
	const uint32_t unimp_mask; // Mask for unimplemented bits (convenience func on register write)
} stm32_reginfo_t;

// NOTE: it's an implicit thing that RCC comes first so that when iterating
// it is ready to be attached to all other peripherals to do their RESET and clock handling.
#define PERIPHS \
    _P(RCC), \
    _P(GPIOA), \
    _P(GPIOB), \
    _P(GPIOC), \
    _P(GPIOD), \
    _P(GPIOE), \
    _P(GPIOF), \
    _P(GPIOG), \
    _P(GPIOH), \
    _P(GPIOI), \
    _P(GPIOJ), \
    _P(GPIOK), /* NOTE: update the convenience index below if you add more GPIOS */ \
    _P(SYSCFG), \
    _P(AFIO), \
    _P(UART1), \
    _P(UART2), \
    _P(UART3), \
    _P(UART4), \
    _P(UART5), \
    _P(UART6), \
    _P(UART7), \
    _P(UART8), /* NOTE: update the convenience index below if you add more UARTS */ \
    _P(ADC_ALL), /* special common ADC for shared reset.*/ \
    _P(ADCC), \
    _P(ADC1), \
    _P(ADC2), \
    _P(ADC3), /* NOTE: update the convenience index below if you add more ADCs */ \
    _P(DAC), \
    _P(TIM1), \
    _P(TIM2), \
    _P(TIM3), \
    _P(TIM4), \
    _P(TIM5), \
    _P(TIM6), \
    _P(TIM7), \
    _P(TIM8), \
    _P(TIM9), \
    _P(TIM10), \
    _P(TIM11), \
    _P(TIM12), \
    _P(TIM13), \
    _P(TIM14), \
    _P(TIM15), \
    _P(TIM16), \
    _P(TIM17), \
    _P(BKP), \
    _P(PWR), \
    _P(I2C1), \
    _P(I2C2), \
    _P(I2C3), \
    _P(I2C4), \
    _P(I2S1), \
    _P(I2S2), \
    _P(I2S3), \
	_P(I3C1), \
	_P(I3C2), \
    _P(IWDG), \
    _P(WWDG), \
    _P(CAN1), \
    _P(CAN2), \
    _P(CAN), \
    _P(USBFS), /*DANGER WILL ROBINSON - FS must come first in init order otherwise the USB drive gets attached to the wrong port!*/ \
    _P(USBHS), \
    _P(SPI1), \
    _P(SPI2), \
    _P(SPI3), \
    _P(SPI4), \
    _P(SPI5), \
    _P(SPI6), \
    _P(EXTI), \
    _P(SDIO), \
    _P(FSMC), \
    _P(FINT), \
    _P(RTC), \
    _P(CRC), \
	_P(DMAMUX), \
    _P(DMA1), \
    _P(DMA2), /* NOTE: update the convenience index below if you add more DMAs */ \
    _P(DCMI), \
    _P(CRYP), \
    _P(HASH), \
    _P(RNG), \
    _P(QSPI), \
    _P(LPTIM1), \
	_P(ETH), \
	_P(OTP), \
	_P(DWT), \
	_P(ITM), \
	_P(DBG), \
	_P(ICACHE),

#define _P(x) STM32_P_##x

enum STM32_PERIPHS {
	_P(UNDEFINED) = -1,
    PERIPHS
    _P(COUNT),
	STM32_P_DMA_BEGIN = STM32_P_DMA1,
	STM32_P_DMA_END = STM32_P_DMA2,
	STM32_P_GPIO_BEGIN = STM32_P_GPIOA,
	STM32_P_GPIO_END = STM32_P_GPIOK + 1U,
	STM32F030_GPIO_END = STM32_P_GPIOF + 1U,
	STM32G070_GPIO_END = STM32_P_GPIOF + 1U,
	STM32H503_GPIO_END = STM32_P_GPIOH + 1U,
	STM32_P_ADC_BEGIN = STM32_P_ADC1,
	STM32_P_ADC_END = STM32_P_ADC3 + 1U,
    STM32_P_USART_BEGIN = STM32_P_UART1,
	STM32F030_USART_END = STM32_P_UART2 + 1U,
	STM32H503_USART_END = STM32_P_UART3 + 1U,
	STM32G070_USART_END = STM32_P_UART6 + 1U,
    STM32_P_USART_END = STM32_P_UART8 + 1U,
};
#undef _P

#define _P(x) #x
static const char *_PERIPHNAMES[STM32_P_COUNT]  __attribute__((unused)) = {
    PERIPHS
} ;
#undef _P

#undef PERIPHS // Clear the IRQ pairs for subsequent classes.

// Globals are ugly, but this is the only way I see to easily get a value
// available during the xxx_init routine for the purposes of a pretty set of MR names.
extern stm32_periph_t g_stm32_periph_init;

QEMU_BUILD_BUG_MSG(STM32_P_COUNT>=256,"Err - peripheral reset arrays not meant to handle >255 peripherals!");

#define _CHECK_BOUNDS(index, val, max, reginfo, descr, type) \
    if (index >= max || reginfo[index].is_reserved) { \
        qemu_log_mask(LOG_GUEST_ERROR, "Register "type" 0x%x (val 0x%x) invalid on "descr"\n", \
          (unsigned int)index << 2, (unsigned int)val);

// Checks whether a register access is in-bounds and the register is not fully reserved.
// Issues a GUEST_ERROR if it is.
#define CHECK_BOUNDS_W(index, val, max, reginfo, descr) \
    _CHECK_BOUNDS(index, val, max, reginfo, descr, "write") \
		return; \
	}

// Checks whether a register access is in-bounds and the register is not fully reserved.
// Issues a GUEST_ERROR if it is.
#define CHECK_BOUNDS_R(index, max, reginfo, descr) \
 	_CHECK_BOUNDS(index, 0xBADUL, max, reginfo, descr, "read") \
	 	return 0; \
	}

#define _CHECK_BOUNDS_V2(index, val, max, type) \
    if (index >= max) { \
        qemu_log_mask(LOG_GUEST_ERROR, "Register "type" 0x%x (val 0x%x) invalid on %s:%s\n", \
          (unsigned int)index << 2, (unsigned int)val, __FILE__, __func__);

// Checks whether a register access is in-bounds and the register is not fully reserved.
// Issues a GUEST_ERROR if it is.
#define CHECK_BOUNDS_W_V2(index, val, max) \
    _CHECK_BOUNDS_V2(index, val, max, "write") \
		return; \
	}

// Checks whether a register access is in-bounds and the register is not fully reserved.
// Issues a GUEST_ERROR if it is.
#define CHECK_BOUNDS_R_V2(index, max) \
 	_CHECK_BOUNDS_V2(index, 0xBADUL, max, "read") \
	 	return 0; \
	}


// Enforces reserved bits in a register, new version. Meant to be used with the generated structs that set not_reserved too.
#define ENFORCE_RESERVED_V2(val, curval, reginfo, index) \
	stm32_enforce_reserved_v2(&val, curval, reginfo, index, __func__, #index)

G_GNUC_UNUSED static void stm32_enforce_reserved_v2(uint64_t* val, uint64_t curval, const stm32_reginfo_t* reginfo, int index, const char* name, const char* regname)
{
	// There are two tiers of checking, the register may be entirely unspecified (entry uninitialized) or it may have reserved bits.
	if (reginfo[index].not_reserved==false) {
		qemu_log_mask(LOG_GUEST_ERROR, "%s: Attempted write reserved register '%s'@%03x!\n", name, regname, index*4U);
		*val = 0;
	}
	else
	{
		uint32_t old_reserved = curval & ~reginfo[index].mask;
		uint32_t new_reserved = *val & ~reginfo[index].mask;
		if (old_reserved != new_reserved) {
			qemu_log_mask(LOG_GUEST_ERROR, "%s: Attempted to alter a reserved bit in '%s'@0x%03x!\n", name, regname, index*4U);
			*val = *val & reginfo[index].mask;
		}
	}
}

#define WARN_UNIMPLEMENTED_REG(offset, type) \
        stm32_unimp("%s: unimplemented register "#type": 0x%x", __func__, (int)offset)

// Checks if a write attempts to set any unimplemented bits and issues a LOG_UNIMP.
// Does NOT check against already set bits, so may give false positives. Use CHECK_UNIMP_V2 for that.
#define CHECK_UNIMP(val, reginfo, index) \
	stm32_check_unimp(val, 0, reginfo, index, __func__, #index)

// Checks if a write attempts to set any unimplemented bits and issues a LOG_UNIMP.
// V2 actually checks for changes before printing the warning.
#define CHECK_UNIMP_V2(val, curval, reginfo, index) \
	stm32_check_unimp(val, curval, reginfo, index, __func__, #index)

G_GNUC_UNUSED static void stm32_check_unimp(uint64_t val, uint64_t curval, const stm32_reginfo_t *reginfo, int index, const char* name, const char* regname)
{
	uint32_t old_unimp = curval & reginfo[index].unimp_mask;
	uint32_t new_unimp = val & reginfo[index].unimp_mask;
	if (old_unimp != new_unimp) {
		qemu_log_mask(LOG_UNIMP, "%s: Modified unimplemented field (mask: 0x%"PRIx32" value: 0x%"PRIx64" => 0x%"PRIx64") '%s'!\n", name, reginfo[index].unimp_mask, curval, val, regname);
	}
}

// Enforces the reserved mask for a register
#define ENFORCE_RESERVED(val, reginfo, index) \
	stm32_enforce_reserved(&val, reginfo, index, __func__, #index)

G_GNUC_UNUSED static void stm32_enforce_reserved(uint64_t* val, const stm32_reginfo_t* reginfo, int index, const char* name, const char* regname)
{
	if (*val & (~reginfo[index].mask)) {
		qemu_log_mask(LOG_GUEST_ERROR, "%s: Attempted to alter a reserved bit in '%s'!\n", name, regname);
		*val = *val & reginfo[index].mask;
	}
}

// Enforces reserved bits and then checks for unimplemented ones
#define CHECK_UNIMP_RESVD(val, reginfo, index) \
	ENFORCE_RESERVED(val, reginfo, index); \
	CHECK_UNIMP(val, reginfo, index);

#define CHECK_UNIMP_RESVD_V2(val, curval, reginfo, index) \
	ENFORCE_RESERVED_V2(val, curval, reginfo, index); \
	CHECK_UNIMP_V2(val, curval, reginfo, index);


// Adjusts for read offsets and sizes given old and new register values.
// Allowed is a bitmask of sizes, e.g:
// 7 = 4,2, and 1 byte is allowed
// 6 = 4 and 2 bytes only, etc.
// disallowed writes are not blocked but log a guest error.
#define ADJUST_FOR_OFFSET_AND_SIZE_W(old, new, size, off, allowed) \
	stm32_adjust_off_size_w(old, &new, size, off, allowed, __func__);

static inline void stm32_adjust_off_size_w(uint32_t old, uint64_t* new, unsigned int size, int off, uint8_t allowed, const char* name)
{
	if (!(size & allowed)) {
		qemu_log_mask(LOG_GUEST_ERROR, "Disallowed register write of size %d in %s!\n", size,  name);
	}
    switch (size) {
        case 1:
			*new = (old & ~(0xff << (off * 8))) | *new << (off * 8);
            break;
        case 2:
            *new = (old & ~(0xffff << (off * 8))) | *new << (off * 8);
            break;
        case 4:
            break;
        default:	// LCOV_EXCL_LINE
            abort(); // LCOV_EXCL_LINE
    }
}

// Adjusts for read offsets and sizes given old and new register values.
// Allowed is a bitmask of sizes, e.g:
// 7 = 4,2, and 1 byte is allowed
// 6 = 4 and 2 bytes only, etc.
// disallowed writes are not blocked but log a guest error.
#define ADJUST_FOR_OFFSET_AND_SIZE_R(data, size, off, allowed) \
	stm32_adjust_off_size_r(&data, size, off, allowed,  __func__);

static inline void stm32_adjust_off_size_r(uint32_t* data, unsigned int size, int off, uint8_t allowed, const char* name)
{
	if (!(size & allowed)) {
		qemu_log_mask(LOG_GUEST_ERROR, "Disallowed register read of size %d in %s!\n", size, name);
	}
    switch (size) {
        case 1:
			*data = (*data>>(off*8U)&0xFFU);
            break;
        case 2:
            *data = (*data>>(off*8U)&0xFFFFU);
            break;
        case 4:
            break;
        default: 	// LCOV_EXCL_LINE
            abort(); // LCOV_EXCL_LINE
    }
}

#define RI_TO_ADDRESS(ri) (ri << 2U)
#define STM32_RI_ADDRESS(base, ri) (base + ((ri) << 2U))

#endif //STM32_SHARED_H
