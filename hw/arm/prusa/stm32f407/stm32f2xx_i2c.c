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

#define	R_I2C_CR1      (0x00 / 4)
#define	R_I2C_CR2      (0x04 / 4)
#define R_I2C_OAR1     (0x08 / 4)
#define R_I2C_OAR2     (0x0c / 4)
#define R_I2C_DR       (0x10 / 4)
#define R_I2C_SR1      (0x14 / 4)
#define R_I2C_SR2      (0x18 / 4)
#define R_I2C_CCR      (0x1c / 4)
#define R_I2C_TRISE    (0x20 / 4)
#define R_I2C_MAX      (0x24 / 4)

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
static void STM32F2XXI2CState_update_irq(STM32F2XXI2CState *s) {
    int new_err_irq_level = 0;
    if (s->i2c_cr2 & R_I2C_CR2_ITERREN_BIT) {
        new_err_irq_level =  (s->i2c_sr1 & R_I2C_SR1_BERR_BIT)
                           | (s->i2c_sr1 & R_I2C_SR1_ARLO_BIT)
                           | (s->i2c_sr1 & R_I2C_SR1_AF_BIT)
                           | (s->i2c_sr1 & R_I2C_SR1_OVR_BIT)
                           | (s->i2c_sr1 & R_I2C_SR1_PECERR_BIT)
                           | (s->i2c_sr1 & R_I2C_SR1_TIMEOUT_BIT)
                           | (s->i2c_sr1 & R_I2C_SR1_SMBALERT_BIT);
    }

    int new_evt_irq_level = 0;
    if (s->i2c_cr2 & R_I2C_CR2_ITEVTEN_BIT) {
        new_evt_irq_level =  (s->i2c_sr1  & R_I2C_SR1_SB_BIT)
                           | (s->i2c_sr1  & R_I2C_SR1_ADDR_BIT)
                           | (s->i2c_sr1  & R_I2C_SR1_ADD10_BIT)
                           | (s->i2c_sr1  & R_I2C_SR1_STOPF_BIT)
                           | (s->i2c_sr1  & R_I2C_SR1_BTF_BIT);

        if (s->i2c_cr2 & R_I2C_CR2_ITBUFEN_BIT) {
            new_evt_irq_level |= (s->i2c_sr1  & R_I2C_SR1_TxE_BIT)
                               | (s->i2c_sr1  & R_I2C_SR1_RxNE_BIT);
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
    if (offset < R_I2C_MAX) {

        reg_name = STM32F2XXI2CState_reg_name_arr[offset];
        switch(offset) {
            //case R_I2C_SR1: 
            //     if ((s->slave_address&1) && (s->regs[R_I2C_SR1]&R_I2C_SR1_BTF_BIT)) { // This is likely polling for a read...
            //         if (!(s->regs[R_I2C_SR1]&R_I2C_SR1_RxNE_BIT))
            //             s->regs[R_I2C_DR] = i2c_recv(s->bus); // Actually get the value, if prev value has already been read.
            //         s->regs[R_I2C_SR1] |= R_I2C_SR1_RxNE_BIT;
            //     }
            // case R_I2C_DR:
            // {
            //     s->regs[R_I2C_SR1] &= ~R_I2C_SR1_RxNE_BIT;

            // }
            case R_I2C_DR:
               // printf("Read DR\n");
                //s->i2c_sr1 &= ~R_I2C_SR1_RxNE_BIT; // Clear RxNE (EV7)
                dumpReg("NDR SR1",s->i2c_sr1, _SR1_names);
                break;
            case R_I2C_SR2:
              //  printf("Read SR2\n");
                if (s->last_read == R_I2C_SR1){
                    s->i2c_sr1 &= ~R_I2C_SR1_ADDR_BIT; // Clear addr (EV6, 7-bit mode)
                    if (!(s->slave_address & 1U))
                        s->i2c_sr1 |= R_I2C_SR1_TxE_BIT; // Ready to send (EV8_1);
                    else
                    {
                        if (i2c_bus_busy(s->bus) ) // in a transfer and reading
                        s->i2c_dr = i2c_recv(s->bus);
                        s->i2c_sr1 |= R_I2C_SR1_RxNE_BIT | R_I2C_SR1_BTF_BIT; // set up for read. 
                        // if (s->i2c_cr1 & R_I2C_CR1_ACK_BIT)
                        // {
                        //     s->i2c_sr1 |= R_I2C_SR1_BTF_BIT; // set RxNE only if we are acking and expecting more.
                        // }
                    }
                    
                }
                break;
            case R_I2C_CR1:
                dumpReg("Read CR1",s->i2c_cr1, _CR1_names);
                break;
            case R_I2C_SR1:
                dumpReg("Read SR1",s->i2c_sr1, _SR1_names);
                break;
            case R_I2C_CR2:    
                dumpReg("Read CR2",s->i2c_cr2, _CR2_names);
                break;
        }
        r = s->regs[offset];
        s->last_read = offset;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "Out of range I2C write, offset 0x%x\n",
          (unsigned)offset << 2);
    }

    //printf("%s %s:  register %s, result: 0x%x\n", __func__, "s->bus.parent_obj.id",
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

    if (offset < R_I2C_MAX) {
        reg_name = STM32F2XXI2CState_reg_name_arr[offset];
    }
    // printf("%s %s: register %s, data: 0x%llx, size:%d\n", __func__, "s->busdev.parent_obj.id",
    //         reg_name, data, size);
    //printf("Write: %s: %04x\n",reg_name, data);
    int result = 0;
    switch (offset) {
        case R_I2C_CR1:
            dumpReg("Wri CR1",data,_CR1_names);
            //if (s->last_read == R_I2C_SR1)
           // {
            //    s->i2c_sr1 &= ~R_I2C_SR1_STOPF_BIT; // Per STOPF bit description for sw clear. 
           // }
            if (data & R_I2C_CR1_PE_BIT)
            {
                s->i2c_cr1 |= R_I2C_CR1_PE_BIT; // Enable I2C. 
            } else {
                s->i2c_cr1 = 0; // Periph disabled.
            }
            if (data & R_I2C_CR1_START_BIT) {
                s->i2c_sr1 |= R_I2C_SR1_SB_BIT;
            }
            if (data & R_I2C_CR1_STOP_BIT)
            {
              //  s->i2c_sr1 |= R_I2C_SR1_STOPF_BIT;
                s->i2c_sr1 &= ~(R_I2C_SR1_BTF_BIT | R_I2C_SR1_TxE_BIT); // Clear TxE/BTF on stop (EV8_2)
                i2c_end_transfer(s->bus);
            }
            dumpReg("New SR1",s->i2c_sr1,_SR1_names);
            break;
        case R_I2C_DR:
            if (s->i2c_sr1 & R_I2C_SR1_SB_BIT){ // starting a transfer...
                if (s->last_read == R_I2C_SR1)
                    s->i2c_sr1 &= ~R_I2C_SR1_SB_BIT; // Clear SB, (EV5)
                if (i2c_start_transfer(s->bus, data>>1, data &1)){
                    s->i2c_sr1 |= R_I2C_SR1_AF_BIT;
                } else {
                    s->i2c_sr1 |= R_I2C_SR1_ADDR_BIT; // TODO... 10 bit mode. 
                    s->slave_address = data;
                }
            }
            if (s->i2c_sr1 & R_I2C_SR1_TxE_BIT) {
                if (i2c_send(s->bus, (uint8_t)data)) {
                    s->i2c_sr1 |= R_I2C_SR1_AF_BIT;
                } else {
                    s->i2c_sr1 |= R_I2C_SR1_BTF_BIT;
                }
            } // Technically this should clear TxE but just leave it 1 for additional writes.
              dumpReg("W RD new SR1",s->i2c_sr1,_SR1_names);
            break;
            
    // case R_I2C_CR1:
    //     s->regs[offset] = data;
    //     s->regs[R_I2C_SR1] &= ~(R_I2C_SR1_STOPF_BIT|R_I2C_SR1_SB_BIT);
    //     if (data & R_I2C_CR1_START_BIT) {
    //         // For now, abort all attempted master transfers with a bus error
    //         s->regs[R_I2C_SR1] |= R_I2C_SR1_SB_BIT; //R_I2C_SR1_BERR_BIT;
    //     }
    //     if (data & R_I2C_CR1_STOP_BIT) {
    //         i2c_end_transfer(s->bus);
    //         s->slave_address = 0;
    //         s->regs[R_I2C_SR1] |= R_I2C_SR1_STOPF_BIT;
    //         s->regs[R_I2C_CR1] &= ~R_I2C_CR1_STOP_BIT;
    //     }
    //     if ((data & R_I2C_CR1_PE_BIT) == 0) {
    //         s->regs[R_I2C_SR1] = 0;
    //     }
    //     break;

    // case R_I2C_DR:
    // {
    //     if (s->regs[R_I2C_SR1] & R_I2C_SR1_SB_BIT){ // TODO: 7-bit only atm. 
    //         result = i2c_start_transfer(s->bus, data>>1, data&1);
    //         s->slave_address = data;
    //         s->regs[R_I2C_SR1] &= ~R_I2C_SR1_SB_BIT;
    //         s->regs[R_I2C_CR1] &= ~R_I2C_CR1_START_BIT;
    //         s->regs[R_I2C_SR1] |= (result == 0) ? R_I2C_SR1_ADDR_BIT : R_I2C_SR1_AF_BIT;
    //         if (data &1)
    //             s->regs[R_I2C_SR1] |= R_I2C_SR1_BTF_BIT; // set in prep for read. 
    //     } else { // Data byte
    //         result = i2c_send(s->bus, (uint8_t)data);
    //         s->regs[R_I2C_SR1] |= (result == 0) ? R_I2C_SR1_BTF_BIT : R_I2C_SR1_AF_BIT;
    //     }
    //     if (result == 0)
    //         s->regs[R_I2C_SR1] |= R_I2C_SR1_TxE_BIT; // Set TxE if success, we can receive next bit.
    //     break;
    // }
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
