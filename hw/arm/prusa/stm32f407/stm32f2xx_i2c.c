/*
 * STM32F2XX I2C
 *
 * Copyright (c) 2013 <https://github.com/pebble/qemu>
 * Adapted for Mini404 in 2020 VintagePC <github.com/vinagepc>
 * (Significant portions rewritten and updated)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "stm32f2xx_i2c.h"
#include "../utility/macros.h"

#define	R_CR1      (0x00 / 4)
#define	R_CR2      (0x04 / 4)
#define R_OAR1     (0x08 / 4)
#define R_OAR2     (0x0c / 4)
#define R_DR       (0x10 / 4)
#define R_SR1      (0x14 / 4)
#define R_SR2      (0x18 / 4)
#define R_CCR      (0x1c / 4)
#define R_TRISE    (0x20 / 4)
#define R_MAX      (0x24 / 4)

//#define DEBUG_STM32STM32F2XXI2CState
#ifdef DEBUG_STM32STM32F2XXI2CState
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32STM32F2XXI2CState: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)


    static const char *stm32f2xxi2c_reg_name_arr[] = {
        "CR1",
        "CR2",
        "OAR1",
        "OAR2",
        "DR",
        "SR1",
        "SR2",
        "CCR",
        "TRISE"
    };

#else
#define DPRINTF(fmt, ...)
#endif


static const char *_SR1_names[] = {
    "SB",
    "ADDR",
    "BTF",
    "ADD10",
    "STOPF",
    "",
    "RxNE",
    "TxE",
    "BERR",
    "ARLO",
    "AF",
    "OVR",
    "PECERR",
    "",
    "TIMEOUT",
    "SMBALERT",
};

static const char *_CR2_names[] = {
    "ITERREN",
    "ITEVTEN",
    "ITBUFEN",
    "DMAEN",
    "LAST",
};

static const char *_CR1_names[] = {
    "PE",
    "SMBUS",
    "",
    "SMBTYPE",
    "ENARB",
    "ENPEC",
    "ENGC",
    "NOSTRETCH",
    "START",
    "STOP",
    "ACK",
    "POS",
    "PEC",
    "ALERT",
    "",
    "SWRTS",
};

static void  dumpReg(const char* prefix, uint16_t value, const char ** names)
{
    return;
    int i =0;
    printf("%s (%04x):\t", prefix,value);
    while (value>0)
    {
        if (value &1)
        {
            printf("%s ",names[i]);
        }
        i++;
        value >>=1;
    }
    printf("\n");
}


/* Routine which updates the I2C's IRQs.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32f2xxi2c_update_irq(STM32F2XXI2CState *s) {
    int new_err_irq_level = 0;
    if (s->defs.CR2.ITERREN) {
        new_err_irq_level =  (s->defs.SR1.BERR)
                           | (s->defs.SR1.ARLO)
                           | (s->defs.SR1.AF)
                           | (s->defs.SR1.OVR)
                           | (s->defs.SR1.PECERR)
                           | (s->defs.SR1.TIMEOUT)
                           | (s->defs.SR1.SMBALERT);
    }

    int new_evt_irq_level = 0;
    if (s->defs.CR2.ITEVTEN) {
        new_evt_irq_level =  (s->defs.SR1.SB)
                           | (s->defs.SR1.ADDR)
                           | (s->defs.SR1.ADD10)
                           | (s->defs.SR1.STOPF)
                           | (s->defs.SR1.BTF);

        if (s->defs.CR2.ITBUFEN) {
            new_evt_irq_level |= (s->defs.SR1.TxE)
                               | (s->defs.SR1.RxNE);
        }
    }

    DPRINTF("%s %s: setting evt_irq to %d\n", __func__, s->busdev.parent_obj.id,
              !!new_evt_irq_level);
    qemu_set_irq(s->evt_irq, !!new_evt_irq_level);

    DPRINTF("%s %s: setting err_irq to %d\n", __func__, s->busdev.parent_obj.id,
              !!new_err_irq_level);
    qemu_set_irq(s->err_irq, !!new_err_irq_level);
}



static uint64_t
stm32f2xxi2c_read(void *arg, hwaddr offset, unsigned size)
{
    STM32F2XXI2CState *s = arg;
    uint16_t r = UINT16_MAX;

    if (!(size == 2 || size == 4 || (offset & 0x3) != 0)) {
		qemu_log_mask(LOG_GUEST_ERROR, "Bad I2C accessoffset 0x%x\n",
          (unsigned)offset);
    }
    offset >>= 2;
    if (offset < R_MAX) {
        switch(offset) {
            case R_DR:
                if (s->defs.SR1.RxNE && i2c_bus_busy(s->bus)) {
                     if (s->dr_unread) {
                        s->dr_unread = false;
                    } else {
                        s->defs.DR = s->shiftreg;
                        s->shiftreg = i2c_recv(s->bus);
                        s->defs.SR1.BTF = s->shift_full = true;
                    }
                } else if (s->defs.SR1.RxNE) {
                    if (s->dr_unread) {
                        // Special case for 2 bytes left, don't clobber the wanted value currently in DR.
                    } else if (s->shift_full) {
                        s->defs.DR = s->shiftreg;
                        s->shift_full = false;
                        s->defs.SR1.RxNE = false;
                        s->defs.SR1.BTF = false;
                    } else {
                        s->defs.DR = 0;
                        printf("FIXME: READ with no data!\n");
                    }
                }
                s->dr_unread = false;
                // printf("DR Val = %02ulx\n", s->defs.DR);
                break;
            case R_SR2:
                if (s->last_read == R_SR1) {
                    s->defs.SR1.ADDR = false; // Clear ADDR flag.
                }
                break;
        }
        s->last_read = offset;
        r = s->regs[offset];
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "Out of range I2C write, offset 0x%x\n",
          (unsigned)offset << 2);
    }

    //printf("%s %s:  register %s, result: 0x%x\n", __func__, "s->bus.parent_obj.id",
    //         reg_name, r);
    return r;
}


static void
stm32f2xxi2c_write(void *arg, hwaddr offset, uint64_t data, unsigned size)
{
    struct STM32F2XXI2CState *s = (struct STM32F2XXI2CState *)arg;

    if (size != 2 && size != 4) {
		qemu_log_mask(LOG_GUEST_ERROR, "Bad I2C access size 0x%x\n",
          (unsigned)size);
    }
    /* I2C registers are all at most 16 bits wide */
    data &= 0xFFFFF;
    offset >>= 2;
    s->regs[offset] = data;
    switch (offset) {
        case R_CR1:
            dumpReg("Wri CR1",data,_CR1_names);
            if (s->defs.CR1.START) {
                s->defs.SR1.SB = true;
                s->defs.CR1.START = false; // and clear START.
            }
            if (s->defs.CR1.POS && !s->defs.SR1.BTF) { // 2 byte mode
                s->dr_unread = true;
                s->defs.DR = s->shiftreg;
                s->shiftreg = i2c_recv(s->bus);
                s->defs.SR1.BTF = true;
            }
            if (s->defs.CR1.STOP){
                if (s->is_read) {
                    if (!s->dr_unread) {
                        s->defs.DR = s->shiftreg;
                        s->shiftreg = i2c_recv(s->bus);
                        s->shift_full = true;
                        s->dr_unread = true;
                    } else {
						// I'm not sure if this is a problem but it's pretty spammy.
						// Things seem to work fine so it's not like data is being lost somehow.
                        //printf("FIXME - stop with DR unread!\n");
                    }
                }
                i2c_end_transfer(s->bus);
                s->defs.SR1.BTF = false;
                s->defs.CR1.STOP = false;
                s->defs.SR1.TxE = false;
            }
            dumpReg("New SR1",s->regs[R_SR1],_SR1_names);
            break;
        case R_DR:
            // printf("WR DR %02x\n", (uint8_t)(data&0xFF));
            if (s->defs.SR1.SB) { // This is the address byte (we don't support 10bit addresses in this implementation.)
                s->defs.SR1.ADDR = true;
                s->defs.SR1.SB = false;
                s->slave_address = data;
                if (i2c_start_transfer(s->bus, data>>1, data &1)){
                    s->defs.SR1.AF = true; // no device, AF flag.
                } else if (data&1){
                    s->is_read = true;
                    s->shiftreg = i2c_recv(s->bus);
                    s->shift_full = true;
                    s->defs.SR1.RxNE = true;
                } else {
                    s->is_read = false;
                    s->defs.SR1.TxE = true; // Ready for data
                }
            } else if (!s->is_read && s->defs.SR1.TxE) { // Continuing to transmit.
                i2c_send(s->bus, data& 0xFF); // Send the byte.
                s->defs.SR1.BTF = true; // Pretend we put said byte in the shift reg.
            }
            break;
        case R_CR2:
            dumpReg("WR CR2", s->regs[R_CR2],_CR2_names);
            break;
    default:
        if (offset < ARRAY_SIZE(s->regs)) {
            s->regs[offset] = data;
        } else {
            printf("ERR: F2xx I2c reg out of bounds\n");
        }
    }
    stm32f2xxi2c_update_irq(s);
}

static const MemoryRegionOps stm32f2xxi2c_ops = {
    .read = stm32f2xxi2c_read,
    .write = stm32f2xxi2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void
stm32f2xxi2c_reset(DeviceState *dev)
{
    STM32F2XXI2CState *s = STM32F2XX_I2C(dev);
    s->defs.TRISE = 2;
}

static void stm32f2xx_i2c_rcc_reset(void *opaque, int n, int level) {
    if (!level) {
        stm32f2xxi2c_reset(DEVICE(opaque));
    }
}
OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(STM32F2XXI2CState, stm32f2xxi2c, STM32F2XX_I2C, STM32_PERIPHERAL, {NULL});

static void
stm32f2xxi2c_finalize(Object *obj)
{
}

static void
stm32f2xxi2c_init(Object *obj)
{
    STM32F2XXI2CState *s = STM32F2XX_I2C(obj);
    DeviceState *dev = DEVICE(obj);

    // If you hit this it means you've messed up the register packing/bitfields!
    assert(sizeof(s->defs) == sizeof(s->regs));

    memory_region_init_io(&s->mmio, obj, &stm32f2xxi2c_ops, s, TYPE_STM32F2XX_I2C, 0x3ff);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->evt_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->err_irq);
    s->bus = i2c_init_bus(dev, "i2c");

    qdev_init_gpio_in_named(dev,stm32f2xx_i2c_rcc_reset,"rcc-reset",1);

}

static const VMStateDescription vmstate_stm32f2xx_i2c = {
    .name = TYPE_STM32F2XX_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs,STM32F2XXI2CState,R_I2C_COUNT),
        VMSTATE_UINT8(last_read,STM32F2XXI2CState),
        VMSTATE_UINT8(slave_address,STM32F2XXI2CState),
        VMSTATE_UINT8(shiftreg,STM32F2XXI2CState),
        VMSTATE_INT32(rx,STM32F2XXI2CState),
        VMSTATE_BOOL(shift_full,STM32F2XXI2CState),
        VMSTATE_BOOL(is_read,STM32F2XXI2CState),
        VMSTATE_BOOL(dr_unread,STM32F2XXI2CState),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32f2xxi2c_class_init(ObjectClass *c, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(c);
    dc->vmsd = &vmstate_stm32f2xx_i2c;
    dc->reset = stm32f2xxi2c_reset;
}
