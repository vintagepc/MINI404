/*
 * STM32 Microcontroller
 *
 * Copyright (C) 2010 Andre Beckus
 * Adapted for QEMU 5.2 in 2020 by VintagePC <http://github.com/vintagepc>
 *
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STM32_H
#define STM32_H

#include "qemu/timer.h"
#include "hw/arm/armv7m.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "../stm32_common/stm32_shared.h"
// #include "sysemu/char.h"


#define ENUM_STRING(x) [x] = #x
#define ARRAY_LENGTH(array) (sizeof((array))/sizeof((array)[0]))

/* COMMON */
#define BYTE_ACCESS_SIZE 1
#define HALFWORD_ACCESS_SIZE 2
#define WORD_ACCESS_SIZE 4

/* VALUE_BETWEEN is inclusive */
#define VALUE_BETWEEN(value, start, end) ((value >= start) && (value <= end))

#define GET_BIT_MASK(position, value) ((value ? 1 : 0) << position)
#define GET_BIT_MASK_ONE(position) (1UL << position)
#define GET_BIT_MASK_ZERO(position) (~(1 << position))
#define GET_BIT_VALUE(value, position) \
                ((value & GET_BIT_MASK_ONE(position)) >> position)
#define IS_BIT_SET(value, position) ((value & GET_BIT_MASK_ONE(position)) != 0)
#define IS_BIT_RESET(value, position) ((value & GET_BIT_MASK_ONE(position)) ==0)
#define SET_BIT(var, position)   var |= GET_BIT_MASK_ONE(position)
#define RESET_BIT(var, position) var &= GET_BIT_MASK_ZERO(position)

/* Can be true, false, 0, or 1 */
#define CHANGE_BIT(var, position, new_value) \
            var = new_value ? \
                    (var | GET_BIT_MASK_ONE(position)) : \
                    (var & GET_BIT_MASK_ZERO(position))
#define CHANGE_BITS(var, start, mask, new_value) \
            var = (var & ~mask) | ((new_value << start) & mask)

// void stm32_hw_warn(const char *fmt, ...)
//    __attribute__ ((__format__ (__printf__, 1, 2)));

#define stm32_unimp(x...) qemu_log_mask(LOG_UNIMP, x)



const char *stm32_periph_name(stm32_periph_t periph);

/* REGISTER HELPERS */
/* Macros used for converting a half-word into a word.
 * Assume that memory alignment can be determined by looking at offset
 * i.e. the base address should always be 4 byte aligned.
 * Also assume that odd offsets will never occur
 * i.e. all offsets must be 2 byte aligned.
 */
#define STM32_REG_READH_VALUE(offset, value32) \
          ((offset & 3) ? \
            (value32 & 0xffff0000) >> 16 : \
            value32 & 0x0000ffff)
#define STM32_REG_WRITEH_VALUE(offset, old_value32, new_value32) \
          ((offset & 3) ? \
            (old_value32 & 0x0000ffff) | ((new_value32 & 0x0000ffff) << 16) : \
            (old_value32 & 0xffff0000) | (new_value32 & 0x0000ffff) )

/* Error handlers */
# define STM32_BAD_REG(offset, size)       \
        printf("%s: Bad register 0x%x - size %u\n", __FUNCTION__, (int)offset, size)
# define STM32_WARN_RO_REG(offset)        \
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Read-only register 0x%x\n", \
                      __FUNCTION__, (int)offset)
# define STM32_WARN_WO_REG(offset)        \
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Write-only register 0x%x\n", \
                      __FUNCTION__, (int)offset)
# define STM32_NOT_IMPL_REG(offset, size)      \
        printf("%s: Not implemented yet 0x%x - size %u\n", __FUNCTION__, (int)offset, size)



/* EXTI */
typedef struct Stm32Exti Stm32Exti;

#define TYPE_STM32_EXTI "stm32-exti"
#define STM32_EXTI(obj) OBJECT_CHECK(Stm32Exti, (obj), TYPE_STM32_EXTI)

/* Assigns the specified EXTI line to the specified GPIO. */
void stm32_exti_set_gpio(Stm32Exti *s, unsigned exti_line, const uint8_t gpio_index);

/* Unassigns the specified EXTI line from the specified GPIO. */
void stm32_exti_reset_gpio(Stm32Exti *s, unsigned exti_line, const uint8_t gpio_index);



/* GPIO */
typedef struct Stm32Gpio Stm32Gpio;

#define TYPE_STM32_GPIO "stm32-gpio"
#define STM32_GPIO(obj) OBJECT_CHECK(Stm32Gpio, (obj), TYPE_STM32_GPIO)

#define STM32_GPIO_COUNT (STM32_GPIOG - STM32_GPIOA + 1)
#define STM32_GPIO_PIN_COUNT 16


/* Sets the EXTI IRQ for the specified pin.  When a change occurs
 * on this pin, and interrupt will be generated on this IRQ.
 */
void stm32_gpio_set_exti_irq(Stm32Gpio *s, unsigned pin, qemu_irq in_irq);

/* GPIO pin mode */
#define STM32_GPIO_MODE_IN 0
#define STM32_GPIO_MODE_OUT_10MHZ 1
#define STM32_GPIO_MODE_OUT_2MHZ 2
#define STM32_GPIO_MODE_OUT_50MHZ 3
uint8_t stm32_gpio_get_mode_bits(Stm32Gpio *s, unsigned pin);

/* GPIO pin config */
#define STM32_GPIO_IN_ANALOG 0
#define STM32_GPIO_IN_FLOAT 1
#define STM32_GPIO_IN_PULLUPDOWN 2
#define STM32_GPIO_OUT_PUSHPULL 0
#define STM32_GPIO_OUT_OPENDRAIN 1
#define STM32_GPIO_OUT_ALT_PUSHPULL 2
#define STM32_GPIO_OUT_ALT_OPEN 3
uint8_t stm32_gpio_get_config_bits(Stm32Gpio *s, unsigned pin);


#define TYPE_STM32_RCC "stm32-rcc"
#define STM32_RCC(obj) OBJECT_CHECK(Stm32Rcc, (obj), TYPE_STM32_RCC)


/* TIM */
typedef struct Stm32Timer Stm32Timer;
#define STM32_TIM_COUNT   14


/* LPTIM */
typedef struct Stm32F7xxLPTimer Stm32F7xxLPTimer;


/* UART */
#define STM32_UART_COUNT 5

typedef struct Stm32Uart Stm32Uart;

#define TYPE_STM32_UART "stm32-uart"
#define STM32_UART(obj) OBJECT_CHECK(Stm32Uart, (obj), TYPE_STM32_UART)

/* Connects the character driver to the specified UART.  The
 * board's pin mapping should be passed in.  This will be used to
 * verify the correct mapping is configured by the software.
 */
//void stm32_uart_connect(Stm32Uart *s, CharDriverState *chr,
//                        uint32_t afio_board_map);

/* Low level methods that let you connect a UART device to any other instance
 * that has read/write handlers. These can be used in place of stm32_uart_connect
 * if not connecting to a CharDriverState instance. */
// void stm32_uart_set_write_handler(Stm32Uart *s, void *obj,
//         int (*chr_write_handler)(void *chr_write_obj, const uint8_t *buf, int len));
// void stm32_uart_get_rcv_handlers(Stm32Uart *s, IOCanReadHandler **can_read,
//                                  IOReadHandler **read, IOEventHandler **event);

void stm32_create_uart_dev(
        Object *stm32_container,
        stm32_periph_t periph,
        int uart_num,
        DeviceState *rcc_dev,
        DeviceState **gpio_dev,
        DeviceState *afio_dev,
        hwaddr addr,
        qemu_irq irq);


/* STM32F7xx UART */
#define STM32F7XX_UART_COUNT 8

typedef struct Stm32F7xxUart Stm32F7xxUart;

#define TYPE_STM32F7XX_UART "stm32f7xx-uart"
#define STM32F7XX_UART(obj) OBJECT_CHECK(Stm32F7xxUart, (obj), TYPE_STM32F7XX_UART)

/* Connects the character driver to the specified UART.  The
 * board's pin mapping should be passed in.  This will be used to
 * verify the correct mapping is configured by the software.
 */
//void stm32f7xx_uart_connect(Stm32F7xxUart *s, CharDriverState *chr,
  //                      uint32_t afio_board_map);

/* Low level methods that let you connect a UART device to any other instance
 * that has read/write handlers. These can be used in place of stm32_uart_connect
 * if not connecting to a CharDriverState instance. */
// void stm32f7xx_uart_set_write_handler(Stm32F7xxUart *s, void *obj,
//         int (*chr_write_handler)(void *chr_write_obj, const uint8_t *buf, int len));
// void stm32f7xx_uart_get_rcv_handlers(Stm32F7xxUart *s, IOCanReadHandler **can_read,
//                                  IOReadHandler **read, IOEventHandler **event);

void stm32f7xx_create_uart_dev(
        Object *stm32_container,
        stm32_periph_t periph,
        int uart_num,
        DeviceState *rcc_dev,
        DeviceState **gpio_dev,
        DeviceState *afio_dev,
        hwaddr addr,
        qemu_irq irq);


/* AFIO */
#define TYPE_STM32_AFIO "stm32-afio"
#define STM32_AFIO(obj) OBJECT_CHECK(Stm32Afio, (obj), TYPE_STM32_AFIO)

typedef struct Stm32Afio Stm32Afio;

/* AFIO Peripheral Mapping */
#define STM32_USART1_NO_REMAP 0
#define STM32_USART1_REMAP 1

#define STM32_USART2_NO_REMAP 0
#define STM32_USART2_REMAP 1

#define STM32_USART3_NO_REMAP 0
#define STM32_USART3_PARTIAL_REMAP 1
#define STM32_USART3_FULL_REMAP 3

/* Gets the pin mapping for the specified peripheral.  Will return one
 * of the mapping values defined above. */
uint32_t stm32_afio_get_periph_map(Stm32Afio *s, int32_t periph_num);

void stm32_afio_uart_check_tx_pin_callback(Stm32Uart *s);




/* STM32 PERIPHERALS - GENERAL */
DeviceState *stm32_init_periph(DeviceState *dev, stm32_periph_t periph,
                               hwaddr addr, qemu_irq irq);


/* STM32 MICROCONTROLLER - GENERAL */
typedef struct Stm32 Stm32;

/* Initialize the STM32 microcontroller.  Returns arrays
 * of GPIOs and UARTs so that connections can be made. */
void stm32f1xx_init(
            ram_addr_t flash_size,
            ram_addr_t ram_size,
            const char *kernel_filename,
            Stm32Gpio **stm32_gpio,
            Stm32Uart **stm32_uart,
            uint32_t osc_freq,
            uint32_t osc32_freq);

struct stm32f2xx;
void stm32f2xx_init(
                    ram_addr_t flash_size,
                    ram_addr_t ram_size,
                    const char *kernel_filename,
                    Stm32Gpio **stm32_gpio,
                    Stm32Uart **stm32_uart,
                    Stm32Timer **stm32_timer,
                    DeviceState **stm32_rtc,
                    uint32_t osc_freq,
                    uint32_t osc32_freq,
                    struct stm32f2xx *stm,
                    ARMCPU **cpu);

struct stm32f4xx;
void stm32f4xx_init(
                    ram_addr_t flash_size,
                    ram_addr_t ram_size,
                    const char *kernel_filename,
                    Stm32Gpio **stm32_gpio,
                    const uint32_t *gpio_idr_masks,
                    Stm32Uart **stm32_uart,
                    Stm32Timer **stm32_timer,
                    DeviceState **stm32_rtc,
                    uint32_t osc_freq,
                    uint32_t osc32_freq,
                    struct stm32f4xx *stm,
                    ARMCPU **cpu);

struct stm32f7xx;
void stm32f7xx_init(
                    ram_addr_t flash_size,
                    ram_addr_t ram_size,
                    const char *kernel_filename,
                    Stm32Gpio **stm32_gpio,
                    const uint32_t *gpio_idr_masks,
                    Stm32F7xxUart **stm32_uart,
                    Stm32Timer **stm32_timer,
                    Stm32F7xxLPTimer **lptimer,
                    DeviceState **stm32_rtc,
                    uint32_t osc_freq,
                    uint32_t osc32_freq,
                    struct stm32f7xx *stm,
                    ARMCPU **cpu);

#endif /* STM32_H */
