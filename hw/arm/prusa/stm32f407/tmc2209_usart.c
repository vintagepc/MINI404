/*
 * TMC2209 USART mux - based on stm32f2xx_usart
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
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
#include "tmc2209_usart.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"

#ifndef STM_USART_ERR_DEBUG
#define STM_USART_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_USART_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

// static int tmc2209_usart_can_receive(void *opaque)
// {
//     TMC2209UsartState *s = opaque;

//     if (!(s->usart_sr & USART_SR_RXNE)) {
//         return 1;
//     }

//     return 0;
// }

// static void tmc2209_usart_receive(void *opaque, const uint8_t *buf, int size)
// {
//     TMC2209UsartState *s = opaque;

//     if (!(s->usart_cr1 & USART_CR1_UE && s->usart_cr1 & USART_CR1_RE)) {
//         /* USART not enabled - drop the chars */
//         DB_PRINT("Dropping the chars\n");
//         return;
//     }

//     s->usart_dr = *buf;
//     s->usart_sr |= USART_SR_RXNE;

//     if (s->usart_cr1 & USART_CR1_RXNEIE) {
//         qemu_set_irq(s->irq, 1);
//     }

//     DB_PRINT("Receiving: %c\n", s->usart_dr);
// }

static void tmc2209_usart_reset(DeviceState *dev)
{
    TMC2209UsartState *s = TMC2209_USART(dev);

    s->usart_sr = USART_SR_RESET;
    s->usart_dr = 0x00000000;
    s->usart_brr = 0x00000000;
    s->usart_cr1 = 0x00000000;
    s->usart_cr2 = 0x00000000;
    s->usart_cr3 = 0x00000000;
    s->usart_gtpr = 0x00000000;
    s->cmdLen = 4;
    s->cmdTxIdx =0;

    qemu_set_irq(s->irq, 0);
}

static uint64_t tmc2209_usart_read(void *opaque, hwaddr addr,
                                       unsigned int size)
{
    TMC2209UsartState *s = opaque;
    uint64_t retvalue;

    DB_PRINT("Read 0x%"HWADDR_PRIx"\n", addr);

    switch (addr) {
    case USART_SR:
        retvalue = s->usart_sr;
        return retvalue;
    case USART_DR:
        DB_PRINT("Value: 0x%" PRIx32 ", %c\n", s->usart_dr, (char) s->usart_dr);
        retvalue = s->cmdRx.bytes[7-s->cmdRxLen++];
        //return s->usart_dr & 0x3FF;
        if (s->cmdRxLen==8)
            s->usart_sr &= ~USART_SR_RXNE; // clear, reply was sent. 
      //  printf("Byte return: %02x\n",retvalue);
        return retvalue;
    case USART_BRR:
        return s->usart_brr;
    case USART_CR1:
        return s->usart_cr1;
    case USART_CR2:
        return s->usart_cr2;
    case USART_CR3:
        return s->usart_cr3;
    case USART_GTPR:
        return s->usart_gtpr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return 0;
    }

    return 0;
}

static void tmc2209_usart_ssi_tx(TMC2209UsartState *s)
{
    // This is a little weird because the clients will reply after the command has completed, 
    // so we clock extra bytes in order to get the reply this cycle (as opposed to) 
    // traditional SPI where the reply comes with the next data packet.
    // Honestly an I2CSlave is probably better suited but since SPI seems to be the other TMC option 
    // it'll hopefully make for slightley easier sharing of code. 
            qemu_set_irq(s->irqCS[s->cmdTx.bytes[6]],1);
            ssi_transfer(s->spi,s->cmdTx.dwords[1]); // Do the addr dword first so the driver knows if it's a read/write ASAP
            ssi_transfer(s->spi,s->cmdTx.dwords[0]);
            // TODO - get reply. 
            qemu_set_irq(s->irqCS[s->cmdTx.bytes[6]],0);

}


static void tmc2209_usart_write(void *opaque, hwaddr addr,
                                  uint64_t val64, unsigned int size)
{
    TMC2209UsartState *s = opaque;
    uint32_t value = val64;
    DB_PRINT("Write 0x%" PRIx32 ", 0x%"HWADDR_PRIx"\n", value, addr);

    switch (addr) {
    case USART_SR:
        if (value <= 0x3FF) {
            /* I/O being synchronous, TXE is always set. In addition, it may
               only be set by hardware, so keep it set here. */
            s->usart_sr = value | USART_SR_TXE;
        } else {
            s->usart_sr &= value;
        }
        if (!(s->usart_sr & USART_SR_RXNE)) {
            qemu_set_irq(s->irq, 0);
        }
        return;
    case USART_DR:
        if (s->cmdTxIdx==0 && value==0 && size==1) 
            return;
     //   printf("byte: %x (%d)\n",val64,size);
        s->cmdTx.bytes[7-s->cmdTxIdx++]=value;
        if (s->cmdTxIdx==3)
        {
            s->cmdLen = (value & 0x80) ? 8 : 4;
        }
        if (s->cmdTxIdx==s->cmdLen)
        {
            // printf("%s complete, %8lx\n",s->cmdLen==8? "Write":"Read",s->cmdTx.raw);
            s->cmdTxIdx=0;

            tmc2209_usart_ssi_tx(s);

            s->cmdRx.raw = 0x50FF010000000000;
            s->cmdRxLen = 4;
            s->usart_sr |= USART_SR_RXNE;
            if (s->usart_cr1 & USART_CR1_RXNEIE &&
                s->usart_sr & USART_SR_RXNE) {
                qemu_set_irq(s->irq, 1);
            }
        }
        /* XXX I/O are currently synchronous, making it impossible for
            software to observe transient states where TXE or TC aren't
            set. Unlike TXE however, which is read-only, software may
            clear TC by writing 0 to the SR register, so set it again
            on each write. */
        s->usart_sr |= USART_SR_TC;
        return;
    case USART_BRR:
        s->usart_brr = value;
        return;
    case USART_CR1:
        s->usart_cr1 = value;
            if (s->usart_cr1 & USART_CR1_RXNEIE &&
                s->usart_sr & USART_SR_RXNE) {
                qemu_set_irq(s->irq, 1);
            }
        return;
    case USART_CR2:
        s->usart_cr2 = value;
        return;
    case USART_CR3:
        s->usart_cr3 = value;
        return;
    case USART_GTPR:
        s->usart_gtpr = value;
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }
}

static const MemoryRegionOps tmc2209_usart_ops = {
    .read = tmc2209_usart_read,
    .write = tmc2209_usart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void tmc2209_usart_init(Object *obj)
{
    TMC2209UsartState *s = TMC2209_USART(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->mmio, obj, &tmc2209_usart_ops, s,
                          TYPE_TMC2209_USART, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    for (int i=0; i<4; i++)
        qdev_init_gpio_out_named(DEVICE(obj), &s->irqCS[i], "tmc2209_usart_cs", 1);

    s->spi = ssi_create_bus(DEVICE(obj), "spi");
}

static void tmc2209_usart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = tmc2209_usart_reset;
}

static const TypeInfo tmc2209_usart_info = {
    .name          = TYPE_TMC2209_USART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TMC2209UsartState),
    .instance_init = tmc2209_usart_init,
    .class_init    = tmc2209_usart_class_init,
};

static void tmc2209_usart_register_types(void)
{
    type_register_static(&tmc2209_usart_info);
}

type_init(tmc2209_usart_register_types)
