/*
 * STM32F4XX ADC Common register
 *
 * Copyright (c) 2021 by VintagePC <http://github.com/vintagepc> for Mini404
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
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/typedefs.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "stm32f4xx_adcc.h"
#include "../utility/macros.h"

#ifndef STM_ADCC_ERR_DEBUG
#define STM_ADCC_ERR_DEBUG 0
#endif

#define R_CSR       (0x00/4)
#define R_CCR       (0x04/4)
#define R_CDR       (0x08/4)

static void stm32f4xx_adcc_reset(DeviceState *dev)
{
    STM32F4XXADCCState *s = STM32F4XX_ADCC(dev);

    memset(&s->regs,0,sizeof(s->regs));

}

uint8_t stm32f4xx_adcc_get_adcpre(STM32F4XXADCCState *s)
{
    return (s->defs.CCR.ADCPRE<<1) + 2U;
}

// ADC data in from peripherals
static void stm32f4xx_adcc_sr_in(void *opaque, int n, int level){

    STM32F4XXADCCState *s = STM32F4XX_ADCC(opaque);
    uint32_t mask = 0xFF << (n*2U); 
    s->regs[R_CSR] &= ~mask;
    s->regs[R_CSR] |= (level& 0xFF) << (n*2U);
}

static uint64_t stm32f4xx_adcc_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    STM32F4XXADCCState *s = opaque;

   // DB_PRINT("Address: 0x%" HWADDR_PRIx "\n", addr);
    if (size!=4) {
        printf("FIXME: handle non-word ADC reads!\n");
    }
    addr>>=2;

    if (addr >= R_ADCC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR,
                    "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }

    switch (addr) {
        case R_CSR: 
            printf("FIXME: ADCC R_CSR read\n");
            /* FALLTHRU */
        case R_CCR:
            return s->regs[addr];
        case R_CDR:
            printf("FIXME: ADCC R_CDR\n");
            /* FALLTHRU */
        default:
            return 0;
    }
}

static void stm32f4xx_adcc_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    STM32F4XXADCCState *s = opaque;

    // printf("ADCC_write: 0x%" HWADDR_PRIx ", Value: 0x%x\n",
    //          addr, (uint32_t)value);

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: ADC write with size != word (32 bits)!\n", __func__);
    }

    addr>>=2; // Get index in array. 

    if (addr >= 0x100) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: ADC Common Register Unsupported\n", __func__);
    }

    switch (addr) {
        case R_CCR:
            s->regs[addr] = value;          
            break;
        case R_CSR:
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: Bad write to 0x%" HWADDR_PRIx "\n", __func__, addr);
    }
}

static const MemoryRegionOps stm32f4xx_adcc_ops = {
    .read = stm32f4xx_adcc_read,
    .write = stm32f4xx_adcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};


static const VMStateDescription vmstate_stm32f4xx_adcc = {
    .name = TYPE_STM32F4XX_ADCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, STM32F4XXADCCState, R_ADCC_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32f4xx_adcc_init(Object *obj)
{
    STM32F4XXADCCState *s = STM32F4XX_ADCC(obj);

    // Check the register union definitions... This thows compile errors if they are misaligned, so it's ok in regards to not throwing exceptions
    // during object init in QEMU.
    CHECK_ALIGN(sizeof(s->defs),sizeof(uint32_t)*R_ADCC_MAX, "defs union");
    CHECK_ALIGN(sizeof(s->defs),sizeof(s->regs), "Raw array");
    CHECK_REG_u32(s->defs.CSR);
    CHECK_REG_u32(s->defs.CCR);
    CHECK_REG_u32(s->defs.CDR);

    memory_region_init_io(&s->mmio, obj, &stm32f4xx_adcc_ops, s,
                          TYPE_STM32F4XX_ADCC, 0x0C);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    qdev_init_gpio_in_named(DEVICE(obj),stm32f4xx_adcc_sr_in,"sr-in",3);


}

static void stm32f4xx_adcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32f4xx_adcc_reset;
    dc->vmsd = &vmstate_stm32f4xx_adcc;
}

static const TypeInfo stm32f4xx_adcc_info = {
    .name          = TYPE_STM32F4XX_ADCC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F4XXADCCState),
    .instance_init = stm32f4xx_adcc_init,
    .class_init    = stm32f4xx_adcc_class_init,
};

static void stm32f4xx_adcc_register_types(void)
{
    type_register_static(&stm32f4xx_adcc_info);
}

type_init(stm32f4xx_adcc_register_types)
