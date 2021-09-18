/*
 * STM32F407 SoC
 *
 * Original F405 base (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Modified and adapted for Mini404/F407 2020 by VintagePC <http://github.com/vintagepc>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "stm32f407_soc.h"
#include "hw/misc/unimp.h"
#include "net/net.h"
#include "hw/i2c/smbus_eeprom.h"
#include "exec/ramblock.h"
#define SYSCFG_ADD                     0x40013800
static const uint32_t usart_addr[] = { 0x40011000, 0x40004400, 0x40004800,
                                       0x40004C00, 0x40005000, 0x40011400,
                                       0x40007800, 0x40007C00 };
static const uint32_t adc_addr[] = { 0x40012000, 0x40012100, 0x40012200,
                                     0x40012300, 0x40012400, 0x40012500 };
static const uint32_t spi_addr[] =   { 0x40013000, 0x40003800, 0x40003C00,
                                       0x40013400, 0x40015000, 0x40015400 };
#define EXTI_ADDR                      0x40013C00

static const uint32_t i2c_addr[] = { 0x40005400, 0x40005800, 0x40005C00};

static const uint32_t dma_addr[] = {0x40026000, 0x40026400 };

static const uint32_t gpio_addr[] = {0x40020000, 0x40020400, 0x40020800, 0x40020C00, 0x40021000, 0x40021400, 0x40021800, 0x40021C00, 0x40022000, 0x40022400, 0x40022800};
uint32_t gpio_idr_masks[STM_NUM_GPIOS];

     /* Timers */
    struct {
        uint8_t timer_num;
        uint32_t addr;
        uint8_t irq_idx;
    } const timer_desc[] = {
        {1, 0x40010000, 0}, // TIM1 FIXME: TIM1 is a complex timer w/ multiple IRQs
        {2, 0x40000000, STM32_TIM2_IRQ}, // TIM2
        {3, 0x40000400, STM32_TIM3_IRQ}, // TIM3
        {4, 0x40000800, STM32_TIM4_IRQ}, // TIM4
        {5, 0x40000C00, STM32_TIM5_IRQ}, // TIM5
        {6, 0x40001000, STM32_TIM6_IRQ}, // TIM6
        {7, 0x40001400, STM32_TIM7_IRQ}, // TIM7
        {8, 0x40010400, 0}, // TIM8 FIXME: TIM8 is a complex timer w/ multiple IRQs
        {9, 0x40014000, STM32_TIM1_BRK_TIM9_IRQ}, // TIM9
        {10, 0x40014400, STM32_TIM1_UP_TIM10_IRQ}, // TIM10
        {11, 0x40014800, STM32_TIM1_TRG_COM_TIM11_IRQ}, // TIM11
        {12, 0x40001800, STM32_TIM8_BRK_TIM12_IRQ}, // TIM12
        {13, 0x40001C00, STM32_TIM8_UP_TIM13_IRQ}, // TIM13
        {14, 0x40002000, STM32_TIM8_TRG_COMM_TIM14_IRQ}, // TIM14
    };

#define SYSCFG_IRQ               71
static const int usart_irq[] = { 37, 38, 39, 52, 53, 71, 82, 83 };
// static const int timer_irq[] = { 28, 29, 30, 50 };
#define ADC_IRQ 18
static const int spi_irq[] =   { 35, 36, 51, 0, 0, 0 };
static const int exti_irq[] =  { 6, 7, 8, 9, 10, 23, 23, 23, 23, 23, 40,
                                 40, 40, 40, 40, 40} ;
static const int i2c_ev_irq[] = { 31, 33, 72, 95};
static const int i2c_er_irq[] = { 32, 34, 73, 96};

static const int dma1_irq[] = { 11,12,13,14,15,16,17, 47 };
static const int dma2_irq[] = { 56, 57, 58, 59, 60, 68, 69, 70 };

static void stm32f407_soc_initfn(Object *obj)
{
    STM32F407State *s = STM32F407_SOC(obj);
    int i;

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

    object_initialize_child(obj, "syscfg", &s->syscfg, TYPE_STM32F4XX_SYSCFG);

    for (i = 0; i < STM_NUM_USARTS; i++) {
        // if (i==1) continue;
        object_initialize_child(obj, "usart[*]", &s->usart[i],
                                TYPE_STM32_UART);
    }


    // object_initialize_child(obj, "uart2", &s->uart2, TYPE_STM32_UART);

    object_initialize_child(obj, "adcc", &s->adc_common, TYPE_STM32F4XX_ADCC);

    for (i = 0; i < STM_NUM_ADCS; i++) {
        object_initialize_child(obj, "adc[*]", &s->adc[i], TYPE_STM32F4XX_ADC);
    }

    for (i = 0; i < STM_NUM_SPIS; i++) {
        object_initialize_child(obj, "spi[*]", &s->spi[i], TYPE_STM32F4XX_SPI);
    }

    for (i = 0; i < STM_NUM_I2CS; i++) {
        object_initialize_child(obj, "i2c[*]", &s->i2c[i], TYPE_STM32F2XX_I2C);
    }

    for (i=0; i < STM_NUM_GPIOS; i++) {
        object_initialize_child(obj, "gpio[*]", &s->gpio[i],TYPE_STM32F2XX_GPIO);
    }

    object_initialize_child(obj, "rcc", &s->rcc, TYPE_STM32F2XX_RCC);

    object_initialize_child(obj, "exti", &s->exti, TYPE_STM32F4XX_EXTI);

    object_initialize_child(obj, "rtc", &s->rtc, TYPE_STM32F2XX_RTC);

    object_initialize_child(obj, "flashIF", &s->flashIF, TYPE_STM32F2XX_FINT);

    object_initialize_child(obj, "pwr", &s->pwr, TYPE_STM32F2XX_PWR);

    for (i=0; i < STM_NUM_DMAS; i++) {
        object_initialize_child(obj,"dma[*]", &s->dma[i], TYPE_STM32F2XX_DMA);
    }

    for (i=0; i < STM_NUM_TIMERS; i++)
    {
        s->timers[i].id = i+1;
        object_initialize_child(obj, "timers[*]", &s->timers[i], TYPE_STM32F4XX_TIMER);
    }

    object_initialize_child(obj, "itm", &s->itm, TYPE_STM32F4XX_ITM);

    object_initialize_child(obj,"crc",&s->crc, TYPE_STM32F2XX_CRC);

    object_initialize_child(obj, "iwdg",&s->iwdg, TYPE_STM32F4XX_IWDG);

    object_initialize_child(obj, "otg_fs", &s->otg_fs, TYPE_STM32F4xx_USB);
    // // object_property_add_const_link(OBJECT(&s->otg_fs), "dma-mr",
    // //                             OBJECT(&s->temp_usb));
    object_initialize_child(obj, "otg_hs", &s->otg_hs, TYPE_STM32F4xx_USB);
    // object_property_add_const_link(OBJECT(&s->otg_hs), "dma-mr",
    //                             OBJECT(&s->temp_usb));

    object_initialize_child(obj, "otp", &s->otp, TYPE_STM32F4XX_OTP);

}


static void stm32f407_soc_finalize(Object *obj)
{
    printf("soc finalize;");
}

static void stm32f407_soc_realize(DeviceState *dev_soc, Error **errp)
{
    STM32F407State *s = STM32F407_SOC(dev_soc);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *dev, *armv7m, *rcc;
    SysBusDevice *busdev;
    Error *err = NULL;
    int i;

    memory_region_init_rom(&s->flash, OBJECT(dev_soc), "STM32F407.flash",
                           FLASH_SIZE, &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                             "STM32F407.flash.alias", &s->flash, 0,
                             FLASH_SIZE);

    // Kinda sketchy but needed to bypass the FW check on the Mini...
     s->flash.ram_block->host[FLASH_SIZE-1] = 0xFF;

    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);

    memory_region_init_ram(&s->sram, NULL, "STM32F407.sram", SRAM_SIZE,
                           &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, &s->sram);

    memory_region_init_ram(&s->ccmsram, OBJECT(dev_soc), "STM32F407.ccmsram", 64* KiB,&err);

    memory_region_add_subregion(system_memory, 0x10000000, &s->ccmsram);

    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", 96);
    qdev_prop_set_string(armv7m, "cpu-type", s->cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }
    /* System configuration controller */
    dev = DEVICE(&s->syscfg);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->syscfg), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, SYSCFG_ADD);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, SYSCFG_IRQ));

    // RCC
    uint32_t osc_freq = 12000000; /*osc_freq*/
                   //osc32_freq = 32768; /*osc2_freq*/

    rcc = DEVICE(&(s->rcc));
    qdev_prop_set_uint32(rcc, "osc_freq", osc_freq);
    //qdev_prop_set_uint32(dev, "osc32_freq", osc32_freq);
    
    if (!sysbus_realize(SYS_BUS_DEVICE(rcc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(rcc);
    sysbus_mmio_map(busdev, 0, 0x40023800);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, STM32_RCC_IRQ));

    for (i=0; i<STM_NUM_GPIOS; i++) {
        dev = DEVICE(&(s->gpio[i]));
        qdev_prop_set_uint32(dev,"periph",i);
        qdev_prop_set_uint32(dev,"idr-mask",gpio_idr_masks[i]);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio[i]),errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, gpio_addr[i]);
        qdev_connect_gpio_out_named(rcc,"reset",STM32_GPIOA+i,qdev_get_gpio_in_named(DEVICE(&s->gpio[i]),"rcc-reset",0));
    }

    // TODO - Connect EXTI and WAKEUP to the GPIOs.

    /* Attach UART (uses USART registers) and USART controllers */
    struct Chardev;

    Chardev* chrdev_assigns[STM_NUM_USARTS];

    char name[]="stm32uart0";

    // First assign by ID
    for (i = 0; i< STM_NUM_USARTS; i++) {
        name[9] = '0' + i;
        chrdev_assigns[i] = qemu_chr_find(name);
        if (chrdev_assigns[i]!=NULL) {
            printf("Found ID %s - assigned to UART %d\n",name, i+1);
        }
    }
    for (i = 0; i < STM_NUM_USARTS; i++) {
        // if (i==1) continue;
        dev = DEVICE(&(s->usart[i]));
        s->usart[i].periph = STM32_UART1+i;
        s->usart[i].stm32_rcc = (Stm32Rcc*)&s->rcc;
        qdev_prop_set_chr(dev, "chardev", chrdev_assigns[i]);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->usart[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, usart_irq[i]));
    }

    for (i = 0; i < STM_NUM_TIMERS; ++i) {
        //const stm32_periph_t periph = STM32_TIM1 + timer_desc[i].timer_num - 1;
        dev = DEVICE(&s->timers[i]);
        s->timers[i].id = i+1;
        s->timers[i].periph = STM32_TIM1+i;
        //if (i!=5 && i!=13 && i!=2) 
        s->timers[i].rcc = (Stm32Rcc*)&s->rcc;
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->timers[i]),errp))
            return;
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, timer_desc[i].addr);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m,timer_desc[i].irq_idx));
        qdev_connect_gpio_out_named(rcc,"reset",STM32_TIM1+i,qdev_get_gpio_in_named(DEVICE(&s->timers[i]),"rcc-reset",0));

         // timer->id = stm32f4xx_periph_name_arr[periph];
        // stm32_init_periph(timer, periph, timer_desc[i].addr,
        //                   qdev_get_gpio_in(nvic, timer_desc[i].irq_idx));
    }
    /* ADC device, the IRQs are ORed together */
    if (!object_initialize_child_with_props(OBJECT(s), "adc-orirq",
                                            &s->adc_irqs, sizeof(s->adc_irqs),
                                            TYPE_OR_IRQ, errp, NULL)) {
        return;
    }
    object_property_set_int(OBJECT(&s->adc_irqs), "num-lines", STM_NUM_ADCS,
                            &error_abort);
    if (!qdev_realize(DEVICE(&s->adc_irqs), NULL, errp)) {
        return;
    }
    qdev_connect_gpio_out(DEVICE(&s->adc_irqs), 0,
                          qdev_get_gpio_in(armv7m, ADC_IRQ));


    DeviceState* adcc_dev = DEVICE(&s->adc_common);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->adc_common), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(adcc_dev);
    sysbus_mmio_map(busdev, 0, 0x40012300);


    DeviceState* adc_reset = qdev_new("split-irq");
    qdev_prop_set_uint16(adc_reset, "num-lines", STM_NUM_ADCS);
    qdev_realize(adc_reset, NULL, errp);
    qdev_connect_gpio_out_named(rcc,"reset", STM32_ADC1, qdev_get_gpio_in(adc_reset,0));
    for (i = 0; i < STM_NUM_ADCS; i++) {
        dev = DEVICE(&(s->adc[i]));
        s->adc[i].id = STM32_ADC1 + i;
        s->adc[i].rcc = (Stm32Rcc*)&s->rcc;
        s->adc[i].common = &s->adc_common;
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->adc[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, adc_addr[i]);
        sysbus_connect_irq(busdev, 0,
                           qdev_get_gpio_in(DEVICE(&s->adc_irqs), i));
        // ADCs have common reset.
        qdev_connect_gpio_out(adc_reset,i,qdev_get_gpio_in_named(DEVICE(&s->adc[i]),"rcc-reset",0));

    }

    /* SPI devices */
    for (i = 0; i < STM_NUM_SPIS; i++) {
        dev = DEVICE(&(s->spi[i]));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, spi_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, spi_irq[i]));
        qdev_connect_gpio_out_named(rcc,"reset",STM32_SPI1+i,qdev_get_gpio_in_named(DEVICE(&s->spi[i]),"rcc-reset",0));
    }

    /* I2C */
    for (i = 0; i < STM_NUM_I2CS; i++) {
        dev = DEVICE(&(s->i2c[i]));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->i2c[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, i2c_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, i2c_ev_irq[i]));
        sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(armv7m, i2c_er_irq[i]));
        qdev_connect_gpio_out_named(rcc,"reset",STM32_I2C1+i,qdev_get_gpio_in_named(DEVICE(&s->i2c[i]),"rcc-reset",0));
    }

    for (i = 0; i < STM_NUM_DMAS; i++) {
        dev = DEVICE(&(s->dma[i]));
        s->dma[i].id = i+1;
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->dma[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, dma_addr[i]);
        for (int j=0; j<8; j++) {
            if (i==0)
                sysbus_connect_irq(busdev, j, qdev_get_gpio_in(armv7m, dma1_irq[j]));
            else if (i==1)
                sysbus_connect_irq(busdev, j, qdev_get_gpio_in(armv7m, dma2_irq[j]));
        }
        qdev_connect_gpio_out_named(rcc,"reset",STM32_DMA1+i,qdev_get_gpio_in_named(DEVICE(&s->dma[i]),"rcc-reset",0));
    }
    for (int j=0; j<STM_NUM_USARTS; j++) // Attach the USART dmar IRQs
    {
        qemu_irq split_usart =qemu_irq_split(
                                qdev_get_gpio_in_named(DEVICE(&s->dma[0]), "dmar",STM32_UART1+j),
                                qdev_get_gpio_in_named(DEVICE(&s->dma[1]), "dmar",STM32_UART1+j));
        qdev_connect_gpio_out_named(DEVICE(&s->usart[j]), "uart-dmar",0, split_usart);
        qdev_connect_gpio_out_named(rcc,"reset",STM32_UART1+i,qdev_get_gpio_in_named(DEVICE(&s->usart[i]),"rcc-reset",0));
    }
    for (int j=0; j<STM_NUM_ADCS; j++) // Attach the ADC dmar IRQs
    {
        qemu_irq split_dmar =qemu_irq_split(
                                qdev_get_gpio_in_named(DEVICE(&s->dma[0]), "dmar",STM32_ADC1+j),
                                qdev_get_gpio_in_named(DEVICE(&s->dma[1]), "dmar",STM32_ADC1+j));
        qdev_connect_gpio_out_named(DEVICE(&s->adc[j]), "dmar",0, split_dmar);
    }


    /* EXTI device */
    dev = DEVICE(&s->exti);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->exti), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, EXTI_ADDR);
    for (i = 0; i < 16; i++) {
        sysbus_connect_irq(busdev, i, qdev_get_gpio_in(armv7m, exti_irq[i]));
    }
    for (i = 0; i < 16; i++) {
        qdev_connect_gpio_out(DEVICE(&s->syscfg), i, qdev_get_gpio_in(dev, i));
    }

    dev = DEVICE(&s->rtc);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rtc),errp))
        return;
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40002800);

    dev = DEVICE(&s->crc);
    s->crc.rcc = (Stm32Rcc*)&s->rcc;
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->crc),errp))
        return;
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40023000);
    // TODO - RTC wakeup exti
    // sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 0, qdev_get_gpio_in(exti_dev, 17));
    // // Alarm B
    // sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 1, qdev_get_gpio_in(exti_dev, 17));
    // // Wake up timer
    // sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 2, qdev_get_gpio_in(exti_dev, 22));

    dev = DEVICE(&s->flashIF);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->flashIF),errp))
        return;
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40023C00);

    dev = DEVICE(&s->pwr);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->pwr),errp))
        return;
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40007000);

    s->iwdg.rcc = (Stm32Rcc*)&s->rcc;
    dev = DEVICE(&s->iwdg);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->iwdg),errp))
        return;
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x40003000);


    dev = DEVICE(&s->otp);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->otp),errp))
        return;
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0x1FFF7800);

    // ITM@ (0xE0000000UL) 
    dev = DEVICE(&s->itm);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->itm),errp))
        return;
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, 0xE0000000UL);

    dev = DEVICE(&s->otg_fs);
    //s->otg_fs.debug = true;
    qdev_prop_set_chr(dev, "chardev", qemu_chr_find("stm32usbfscdc"));

    // IRQs: FS wakeup: 42 FS Global: 67
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->otg_fs),errp))
    {
        return;
    }    
    memory_region_add_subregion(system_memory, 0x50000000UL,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->otg_fs), 0));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->otg_fs), 0, qdev_get_gpio_in(armv7m, 67));
   // qdev_connect_gpio_out_named(rcc,"reset",STM32_USB,qdev_get_gpio_in_named(DEVICE(&s->otg_hs),"rcc-reset",0));

    // USB IRQs:
    // Global HS: 77. WKUP: 76, EP1 in/out = 75/74.
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->otg_hs),errp))
    {
        return;
    }    
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->otg_hs),0,0x40040000UL);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->otg_hs), 0, qdev_get_gpio_in(armv7m, 77));
    qdev_connect_gpio_out_named(rcc,"reset",STM32_USB,qdev_get_gpio_in_named(DEVICE(&s->otg_hs),"rcc-reset",0));


    qemu_check_nic_model(&nd_table[0], "stm32f4xx-ethernet");
    dev = qdev_new("stm32f4xx-ethernet");
    qdev_set_nic_properties(dev, &nd_table[0]);
    if (qemu_find_netdev("mini-eth")!=NULL){
        qdev_prop_set_string(dev,"netdev","mini-eth");
        qdev_prop_set_bit(dev, "connected", true);
    } else {
        printf("Ethernet disconnected. use -netdev id=mini-eth,[opts...] to connect it.\n");
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x40028000);
    // ETH GI = 61, Wakeup Exti = 62
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, qdev_get_gpio_in(armv7m, 61));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1, qdev_get_gpio_in(armv7m, 62));
    // memory_region_add_subregion(system_memory, 0x40040000UL,
    //     sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->otg_hs), 0));

    // create_unimplemented_device("timer[7]",    0x40001400, 0x400);
    // create_unimplemented_device("timer[12]",   0x40001800, 0x400);
    // create_unimplemented_device("timer[6]",    0x40001000, 0x400);
    // create_unimplemented_device("timer[13]",   0x40001C00, 0x400);
    // create_unimplemented_device("timer[14]",   0x40002000, 0x400);
    //create_unimplemented_device("RTC and BKP", 0x40002800, 0x400);
    create_unimplemented_device("WWDG",        0x40002C00, 0x400);
    //create_unimplemented_device("IWDG",        0x40003000, 0x400);
    create_unimplemented_device("I2S2ext",     0x40003000, 0x400);
    create_unimplemented_device("I2S3ext",     0x40004000, 0x400);
    create_unimplemented_device("CAN1",        0x40006400, 0x400);
    create_unimplemented_device("CAN2",        0x40006800, 0x400);
    // create_unimplemented_device("PWR",         0x40007000, 0x400);
    create_unimplemented_device("DAC",         0x40007400, 0x400);
    // create_unimplemented_device("timer[1]",    0x40010000, 0x400);
    // create_unimplemented_device("timer[8]",    0x40010400, 0x400);
    create_unimplemented_device("SDIO",        0x40012C00, 0x400);
    // create_unimplemented_device("timer[9]",    0x40014000, 0x400);
    // create_unimplemented_device("timer[10]",   0x40014400, 0x400);
    // create_unimplemented_device("timer[11]",   0x40014800, 0x400);

    // create_unimplemented_device("CRC",         0x40023000, 0x400);
    //create_unimplemented_device("Flash Int",   0x40023C00, 0x400);
    create_unimplemented_device("BKPSRAM",     0x40024000, 0x400);
    // create_unimplemented_device("DMA1",        0x40026000, 0x400);
    // create_unimplemented_device("DMA2",        0x40026400, 0x400);
    create_unimplemented_device("Ethernet",    0x40028000, 0x1400);
//    create_unimplemented_device("USB OTG HS",  0x40040000, 0x30000);
    create_unimplemented_device("USB OTG FS",  0x50000000, 0x31000); // Note - FS is the serial port/micro-usb connector
    create_unimplemented_device("DCMI",        0x50050000, 0x400);
    create_unimplemented_device("RNG",         0x50060800, 0x400);
    create_unimplemented_device("SYSRAM/RSVD",         0x1FFF0000, 0x8000);
    // create_unimplemented_device("OTP",         0x1FFF7800, 0x21F);
  //  create_unimplemented_device("EXTERNAL",    0xA0000000, 0x3FFFFFFF);

    
}

static Property stm32f407_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type", STM32F407State, cpu_type),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32f407_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f407_soc_realize;
    device_class_set_props(dc, stm32f407_soc_properties);
    /* No vmstate or reset required: device has no internal state */
}

static const TypeInfo stm32f407_soc_info = {
    .name          = TYPE_STM32F407_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F407State),
    .instance_init = stm32f407_soc_initfn,
    .class_init    = stm32f407_soc_class_init,
    .instance_finalize = stm32f407_soc_finalize
};

static void stm32f407_soc_types(void)
{
    type_register_static(&stm32f407_soc_info);
}

type_init(stm32f407_soc_types)
