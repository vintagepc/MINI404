/*-
 * Copyright (c) 2013
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
/*
 * QEMU stm32f2xx TIM emulation
 */
#include "stm32f2xx_tim.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include <assert.h>

#define R_TIM_CR1    (0x00 / 4) //p
#define R_TIM_CR2    (0x04 / 4)
#define R_TIM_SMCR   (0x08 / 4)
#define R_TIM_DIER   (0x0c / 4) //p
#define R_TIM_SR     (0x10 / 4) //p
#define R_TIM_EGR    (0x14 / 4) //p
#define R_TIM_CCMR1  (0x18 / 4)
#define R_TIM_CCMR2  (0x1c / 4)
#define R_TIM_CCER   (0x20 / 4)
#define R_TIM_CNT    (0x24 / 4)
#define R_TIM_PSC    (0x28 / 4) //p
#define R_TIM_ARR    (0x2c / 4) //p
#define R_TIM_CCR1   (0x34 / 4)
#define R_TIM_CCR2   (0x38 / 4)
#define R_TIM_CCR3   (0x3c / 4)
#define R_TIM_CCR4   (0x40 / 4)
#define R_TIM_DCR    (0x48 / 4)
#define R_TIM_DMAR   (0x4c / 4)
#define R_TIM_OR     (0x50 / 4)

#define R_TIM_DIER_UIE 0x1

//#define DEBUG_STM32F2XX_TIM
#ifdef  DEBUG_STM32F2XX_TIM
static const char *f2xx_tim_reg_names[] = {
    ENUM_STRING(R_TIM_CR1),
    ENUM_STRING(R_TIM_CR2),
    ENUM_STRING(R_TIM_SMCR),
    ENUM_STRING(R_TIM_DIER),
    ENUM_STRING(R_TIM_SR),
    ENUM_STRING(R_TIM_EGR),
    ENUM_STRING(R_TIM_CCMR1),
    ENUM_STRING(R_TIM_CCMR2),
    ENUM_STRING(R_TIM_CCER),
    ENUM_STRING(R_TIM_CNT),
    ENUM_STRING(R_TIM_PSC),
    ENUM_STRING(R_TIM_ARR),
    ENUM_STRING(R_TIM_CCR1),
    ENUM_STRING(R_TIM_CCR2),
    ENUM_STRING(R_TIM_CCR3),
    ENUM_STRING(R_TIM_CCR4),
    ENUM_STRING(R_TIM_DCR),
    ENUM_STRING(R_TIM_CCMR1),
    ENUM_STRING(R_TIM_DMAR),
    ENUM_STRING(R_TIM_OR)
};
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_STM32F2XX_TIM: " fmt , ## __VA_ARGS__); \
         usleep(100); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

static uint32_t
f2xx_tim_period(f2xx_tim *s)
{
    
    uint64_t clock_freq = 84000000UL; // APB2 @ 84 Mhz
    if (s->rcc!=NULL) {
        clock_freq = stm32_rcc_get_periph_freq(s->rcc, s->periph);

    }
    clock_freq/= (s->defs.PSC+1);
    uint32_t interval = 1000000000UL/clock_freq;
    return interval;

}

static int64_t
f2xx_tim_next_transition(f2xx_tim *s, int64_t current_time)
{
    if (s->defs.CR1.CMS || s->defs.CR1.DIR) {
        qemu_log_mask(LOG_UNIMP, "f2xx tim, only upedge-aligned mode supported\n");
       // return -1;
    }
    // Note - counter counts from 0...ARR, so there are actually ARR+1 "ticks" to account for.
    return current_time + f2xx_tim_period(s) * (s->defs.ARR+1);
}

static void
f2xx_tim_timer(void *arg)
{
    f2xx_tim *s = arg;

    if (s->defs.CR1.CEN) {
        timer_mod(s->timer, f2xx_tim_next_transition(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
    }
    if (!s->defs.SR.UIF) {
        //printf("f2xx tim timer expired, setting int\n");
        s->defs.SR.UIF = 1;
    }
    // Set IRQ if UIE is enabled. 
   if (s->defs.DIER.UIE)
   {
        qemu_set_irq(s->irq, 1);
   }
}

static uint64_t
f2xx_tim_read(void *arg, hwaddr addr, unsigned int size)
{
    f2xx_tim *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_TIM_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx tim invalid read register 0x%x\n",
          (unsigned int)addr << 2);
        return 0;
    }
    r = (s->regs[addr] >> offset * 8) & ((1ull << (8 * size)) - 1);
    switch (addr) {
    case R_TIM_CR1:
    case R_TIM_DIER:
    case R_TIM_SR:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f2xx tim unimplemented read 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)r);
    }

    DPRINTF("%s %s: reg: %s, size: %u, value: 0x%x\n", s->busdev.parent_obj.id,
                  __func__, f2xx_tim_reg_names[addr], size, r);
    return r;
}

static int f2xx_tim_calc_pwm_ratio(f2xx_tim *s, uint32_t CCR, uint8_t mode, bool active_low)
{
    bool is_inverted = mode &0x1;
    uint32_t ratio = 0;
    switch (mode)
    {
        case 0x00: // frozen
            return -1;
    //    case 0x4: // Force inactive/active:
    //    case 0x5:
            return 255*is_inverted;
        case 0x6:
      //  case 0x7:
            ratio = (CCR*255)/s->defs.ARR;
            if (is_inverted^active_low) ratio = 255-ratio;
            return ratio;
        default:
            printf("FIXME: unimplemented OCxM!\n");
            return -1;
    }
}

// static void f2xx_tim_update_pwm_pin_state(f2xx_tim *s, uint32_t CCR, uint8_t mode, bool active_low)
// {
//     bool is_inverted = 0; mode &0x1;
//     uint32_t ratio = 0;
//     bool is_on = false;
//     switch (mode)
//     {
//         case 0x00: // frozen; do nothing.
//             return;
   
//         case 0x6:
//       //  case 0x7:
//             if (s->defs.CR1.DIR) // downcount. 
//             {
//                 is_on = 
//             }
//             ratio = (CCR*255)/s->defs.ARR;
//             if (is_inverted^active_low) ratio = 255-ratio;
//             return ratio;
//         default:
//             printf("FIXME: unimplemented OCxM!\n");
//             return;
//     }
// }

// static int 

static void
f2xx_tim_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    f2xx_tim *s = arg;
    int offset = addr & 0x3;
    uint32_t changed = 0;
    addr >>= 2;
    if (addr >= R_TIM_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "f2xx tim invalid write register 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    switch(size) {
    case 1:
        data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
        break;
    case 2:
        data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
        break;
    case 4:
        break;
    default:
        abort();
    }
    changed = s->regs[addr]^data;
    if (changed==0)
        return;

    // if (s->id==3) printf("Timer%d: Wrote %08x to %02x (change mask %08x)\n", s->id, data, addr, changed);
    switch(addr) {
    case R_TIM_CR1:
        if (data & ~1) {
            qemu_log_mask(LOG_UNIMP, "f2xx tim non-zero CR1 unimplemented\n");
        }
        if ((s->regs[addr] & 1) == 0 && data & 1) {
            printf("f2xx tim started: %d\n", s->id);
            timer_mod(s->timer, f2xx_tim_next_transition(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
            qemu_set_irq(s->pwm_enable[0], 1);
            printf("pwm en\n");
        } else if (s->regs[addr] & 1 && (data & 1) == 0) {
            timer_del(s->timer);
            qemu_set_irq(s->pwm_enable[0], 0);
            printf("pwm dis\n");

        }
        if (data & 1<<3)
        {
            printf("%d OPM!\n", s->id);
        }
        s->regs[addr] = data;
        break;
    case R_TIM_SR:
        if (s->regs[addr] & 1 && (data & 1) == 0) {
            //printf("f2xx tim clearing int\n");
            qemu_set_irq(s->irq, 0);
        }
        s->regs[addr] &= data;
        break;
    case R_TIM_EGR:
        qemu_log_mask(LOG_UNIMP, "f2xx tim unimplemented write EGR+%u size %u val 0x%x\n",
          offset, size, (unsigned int)data);
        s->regs[addr] = data;
        break;
    case R_TIM_CCER:
        // Capture/Compare Enable register
       // printf("CCER: %04x\n",data);
        s->regs[addr] = data;

        // If we are enabling PWM mode in output 1, notify the callback if we have one registered
        if ( s->defs.CCER.CC1E // Capture Compare 1 output enable
             && s->defs.CCMR1.OC1M == 6
             && s->defs.CCMR1.CC1S == 0) {

            uint32_t ratio = (s->defs.CCR1 * 255) / s->defs.ARR;
            DPRINTF("Setting PWM ratio to %u\n", ratio);
            qemu_set_irq(s->pwm_ratio_changed[0], ratio);
        }

        // If we are enabling PWM mode in output 1, notify the callback if we have one registered
        if ( s->defs.CCER.CC2E // Capture Compare 2 output enable
            && s->defs.CCMR1.OC2M == 6
            && s->defs.CCMR1.CC2S == 0) {

            uint32_t ratio = (s->defs.CCR2 * 255) / s->defs.ARR;
            DPRINTF("Setting PWM ratio to %u\n", ratio);
            qemu_set_irq(s->pwm_ratio_changed[1], ratio);
        } 
        if (changed & 0xF00) {
            if ( s->defs.CCER.CC3E // Capture Compare 2 output enable
                   && s->defs.CCMR2.CC3S == 0) {
                int ratio = f2xx_tim_calc_pwm_ratio(s, s->defs.CCR3, s->defs.CCMR2.OC3M, s->defs.CCER.CC3P);
                if (ratio>=0)
                    qemu_set_irq(s->pwm_ratio_changed[2], ratio);
            } 
            
        }
        if (changed & 0x3000){
            if (s->defs.CCER.CC4E // Capture Compare 4 output enable
                && s->defs.CCMR2.CC4S == 0) {
                int ratio = f2xx_tim_calc_pwm_ratio(s, s->defs.CCR4, s->defs.CCMR2.OC4M, s->defs.CCER.CC4P);
                DPRINTF("Setting PWM ratio to %d\n", ratio);
                if (ratio>=0)
                    qemu_set_irq(s->pwm_ratio_changed[3], ratio);
            }
        }
        break;
    case R_TIM_CCMR1:
    case R_TIM_CCMR2:
    case R_TIM_DIER:
    case R_TIM_PSC:
    case R_TIM_ARR:
        s->regs[addr] = data;
        break;
    case R_TIM_CCR1:
    case R_TIM_CCR2:
    case R_TIM_CCR3:
    case R_TIM_CCR4:
        s->regs[addr] = data;
        if (addr == R_TIM_CCR4)
        {
            // printf("CCR4: %02x\n",data);
        }
        break;
    default:
        s->regs[addr] = data;
        // printf("f2xx tim unimplemented write 0x%x+%u size %u val 0x%x\n",
        //   (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }

}

static const MemoryRegionOps f2xx_tim_ops = {
    .read = f2xx_tim_read,
    .write = f2xx_tim_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void f2xx_tim_reset(DeviceState *dev)
{
    f2xx_tim *s = STM32F4XX_TIMER(dev);
    timer_del(s->timer);
    memset(&s->regs, 0, sizeof(s->regs));
}

#define CHECK_ALIGN(x,y, name) static_assert(x == y, "ERROR - TIMER " name " register definition misaligned!")
#define CHECK_REG_u32(reg) CHECK_ALIGN(sizeof(reg),sizeof(uint32_t),#reg)

static void
f2xx_tim_init(Object *obj)
{
    f2xx_tim *s = STM32F4XX_TIMER(obj);
    // Check the register union definitions... This thows compile errors if they are misaligned, so it's ok in regards to not throwing exceptions
    // during object init in QEMU.
    CHECK_ALIGN(sizeof(s->defs),sizeof(uint32_t)*R_TIM_MAX, "defs union");
    CHECK_REG_u32(s->defs.CR1);
    CHECK_REG_u32(s->defs.CR2);
    CHECK_REG_u32(s->defs.SMCR);
    CHECK_REG_u32(s->defs.DIER);
    CHECK_REG_u32(s->defs.SR);
    CHECK_REG_u32(s->defs.EGR);
    CHECK_REG_u32(s->defs.CCMR1);
    CHECK_REG_u32(s->defs.CCMR2);
    CHECK_REG_u32(s->defs.CCER);
    CHECK_REG_u32(s->defs.RCR);
    CHECK_REG_u32(s->defs.BDTR);
    CHECK_REG_u32(s->defs.DCR);


    // End size check.
    memory_region_init_io(&s->iomem, obj, &f2xx_tim_ops, s, "tim", 0xa0);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    //s->regs[R_RTC_ISR] = R_RTC_ISR_RESET;
    ////s->regs[R_RTC_PRER] = R_RTC_PRER_RESET
    //s->regs[R_RTC_WUTR] = R_RTC_WUTR_RESET;
    DeviceState *dev = DEVICE(obj);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, f2xx_tim_timer, s);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    qdev_init_gpio_out_named(DEVICE(dev), s->pwm_ratio_changed, "pwm_ratio_changed", 4); // OCx1..4
    qdev_init_gpio_out_named(DEVICE(dev), s->pwm_enable, "pwm_enable", 4);
}

// static Property f2xx_tim_properties[] = {
//     DEFINE_PROP_END_OF_LIST(),
// };

static void
f2xx_tim_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    //TODO: fix this: dc->no_user = 1;
    dc->reset = f2xx_tim_reset;

}

static const TypeInfo
f2xx_tim_info = {
    .name          = TYPE_STM32F4XX_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(f2xx_tim),
    .instance_init = f2xx_tim_init,
    .class_init    = f2xx_tim_class_init,
};

static void
f2xx_tim_register_types(void)
{
    type_register_static(&f2xx_tim_info);
}

type_init(f2xx_tim_register_types)
