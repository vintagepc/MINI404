/*-
 * Copyright (c) 2013
 * Adapted for qemu 5.x 2020 VintagePC <github.com/vinagepc>
 * Based on pebble qemu i2c implementation.
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
/*
 * QEMU model of the stm32f2xx I2C controller.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "stm32_util.h"
#include "stm32f2xx_i2c.h"

#define	STM_I2C_CR1      (0x00 / 4)
#define	STM_I2C_CR2      (0x04 / 4)
#define STM_I2C_OAR1     (0x08 / 4)
#define STM_I2C_OAR2     (0x0c / 4)
#define STM_I2C_DR       (0x10 / 4)
#define STM_I2C_SR1      (0x14 / 4)
#define STM_I2C_SR2      (0x18 / 4)
#define STM_I2C_CCR      (0x1c / 4)
#define STM_I2C_TRISE    (0x20 / 4)
#define STM_I2C_MAX      (0x24 / 4)

//#define DEBUG_STM32STM32F2XXI2CState
#ifdef DEBUG_STM32STM32F2XXI2CState
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32STM32F2XXI2CState: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

static const char *STM32F2XXI2CState_reg_name_arr[] = {
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

/* Routine which updates the I2C's IRQs.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void STM32F2XXI2CState_update_irq(STM32F2XXI2CState *s) {
    int new_err_irq_level = 0;
    if (s->i2c_cr2 & STM_I2C_CR2_ITERREN_BIT) {
        new_err_irq_level =  (s->i2c_sr1 & STM_I2C_SR1_BERR_BIT)
                           | (s->i2c_sr1 & STM_I2C_SR1_ARLO_BIT)
                           | (s->i2c_sr1 & STM_I2C_SR1_AF_BIT)
                           | (s->i2c_sr1 & STM_I2C_SR1_OVR_BIT)
                           | (s->i2c_sr1 & STM_I2C_SR1_PECERR_BIT)
                           | (s->i2c_sr1 & STM_I2C_SR1_TIMEOUT_BIT)
                           | (s->i2c_sr1 & STM_I2C_SR1_SMBALERT_BIT);
    }

    int new_evt_irq_level = 0;
    if (s->i2c_cr2 & STM_I2C_CR2_ITEVTEN_BIT) {
        new_evt_irq_level =  (s->i2c_sr1  & STM_I2C_SR1_SB_BIT)
                           | (s->i2c_sr1  & STM_I2C_SR1_ADDR_BIT)
                           | (s->i2c_sr1  & STM_I2C_SR1_ADD10_BIT)
                           | (s->i2c_sr1  & STM_I2C_SR1_STOPF_BIT)
                           | (s->i2c_sr1  & STM_I2C_SR1_BTF_BIT);

        if (s->i2c_cr2 & STM_I2C_CR2_ITBUFEN_BIT) {
            new_evt_irq_level |= (s->i2c_sr1  & STM_I2C_SR1_TxE_BIT)
                               | (s->i2c_sr1  & STM_I2C_SR1_RxNE_BIT);
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
STM32F2XXI2CState_read(void *arg, hwaddr offset, unsigned size)
{
    STM32F2XXI2CState *s = arg;
    uint16_t r = UINT16_MAX;
    const char *reg_name = "UNKNOWN";

    if (!(size == 2 || size == 4 || (offset & 0x3) != 0)) {
        STM32_BAD_REG(offset, size);
    }
    offset >>= 2;
    if (offset < STM_I2C_MAX) {
        r = s->regs[offset];
        reg_name = STM32F2XXI2CState_reg_name_arr[offset];
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "Out of range I2C write, offset 0x%x\n",
          (unsigned)offset << 2);
    }

   // printf("%s %s:  register %s, result: 0x%x\n", __func__, "s->bus.parent_obj.id",
     //         reg_name, r);
    return r;
}


static void
STM32F2XXI2CState_write(void *arg, hwaddr offset, uint64_t data, unsigned size)
{
    const char *reg_name = "UNKNOWN";
    struct STM32F2XXI2CState *s = (struct STM32F2XXI2CState *)arg;

    if (size != 2 && size != 4) {
        STM32_BAD_REG(offset, size);
    }
    /* I2C registers are all at most 16 bits wide */
    data &= 0xFFFFF;
    offset >>= 2;

    if (offset < STM_I2C_MAX) {
        reg_name = STM32F2XXI2CState_reg_name_arr[offset];
    }
    printf("%s %s: register %s, data: 0x%llx, size:%d\n", __func__, "s->busdev.parent_obj.id",
            reg_name, data, size);


    switch (offset) {
    case STM_I2C_CR1:
        s->regs[offset] = data;
        if (data & STM_I2C_CR1_START_BIT) {
            // For now, abort all attempted master transfers with a bus error
            s->regs[STM_I2C_SR1] |= STM_I2C_SR1_SB_BIT; //STM_I2C_SR1_BERR_BIT;
        }
        if ((data & STM_I2C_CR1_PE_BIT) == 0) {
            s->regs[STM_I2C_SR1] = 0;
        }
        break;

    case STM_I2C_DR:
        i2c_send(s->bus, (uint8_t)data);
        break;

    default:
        if (offset < ARRAY_SIZE(s->regs)) {
            s->regs[offset] = data;
        } else {
            STM32_BAD_REG(offset, WORD_ACCESS_SIZE);
        }
    }
    STM32F2XXI2CState_update_irq(s);
}

static const MemoryRegionOps STM32F2XXI2CState_ops = {
    .read = STM32F2XXI2CState_read,
    .write = STM32F2XXI2CState_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void
STM32F2XXI2CState_reset(DeviceState *dev)
{
    //struct STM32F2XXI2CState *s = FROM_SYSBUS(struct STM32F2XXI2CState, SYS_BUS_DEVICE(dev));

//    s->regs[R_SR] = R_SR_RESET;
}

static void
STM32F2XXI2CState_init(Object *obj)
{
    STM32F2XXI2CState *s = STM32F2XX_I2C(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &STM32F2XXI2CState_ops, s, TYPE_STM32F2XX_I2C, 0x3ff);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->evt_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->err_irq);
    s->bus = i2c_init_bus(dev, "i2c");
}

static void
STM32F2XXI2CState_class_init(ObjectClass *c, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(c);
    dc->reset = STM32F2XXI2CState_reset;
}

static const TypeInfo STM32F2XXI2CState_info = {
    .name = TYPE_STM32F2XX_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F2XXI2CState),
    .instance_init = STM32F2XXI2CState_init,
    .class_init = STM32F2XXI2CState_class_init
};

static void
STM32F2XXI2CState_register_types(void)
{
    type_register_static(&STM32F2XXI2CState_info);
}

type_init(STM32F2XXI2CState_register_types)
