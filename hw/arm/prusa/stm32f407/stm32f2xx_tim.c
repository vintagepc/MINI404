/*-
 * Copyright (c) 2013 https://github.com/pebble/qemu/
 * Adapted for QEMU 5.2 in 2020 by VintagePC <http://github.com/vintagepc>
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
#include "migration/vmstate.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/qdev-properties.h"
#include "../utility/macros.h"
#include "../stm32_common/stm32_rcc_if.h"

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

enum CCxS_val
{
	CCxS_OUTPUT,
	CCxS_INPUT_TI1,
	CCxS_INPUT_TI2,
	CCxS_INPUT_TRC
};

enum OCxM_val
{
	OCxM_Frozen,
	OCxM_ACT_M,
	OCxM_INACT_M,
	OCxM_TOGGLE_M,
	OCxM_F_INACT,
	OCxM_F_ACT,
	OCxM_PWM1,
	OCxM_PWM2,
};

static uint32_t
f2xx_tim_period(f2xx_tim *s)
{

    uint64_t clock_freq = stm32_rcc_if_get_periph_freq(&s->parent);

    clock_freq/= (s->defs.PSC+1);
    uint32_t interval = 1000000000UL/clock_freq;
    return interval;

}

static inline int64_t f2xx_tim_ns_to_ticks(f2xx_tim *s, int64_t t)
{
    uint64_t clock_freq = stm32_rcc_if_get_periph_freq(&s->parent);
	if (clock_freq == 0)
		return 0;
	else
	    return muldiv64(t, clock_freq, 1000000000ULL) / (s->defs.PSC + 1);
}


static int64_t
f2xx_tim_next_transition(f2xx_tim *s, int64_t current_time)
{
    if (s->defs.CR1.CMS || s->defs.CR1.DIR) {
        qemu_log_mask(LOG_UNIMP, "f2xx tim, only upedge-aligned mode supported\n");
       // return -1;
    }
    // We also need to update the timebase used for determining the count value each rollover.
    s->count_timebase = f2xx_tim_ns_to_ticks(s,current_time);
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
    if (!s->defs.SR.UIF && s->defs.DIER.UIE) {
        //printf("f2xx tim timer expired, setting int\n");
        s->defs.SR.UIF = 1;
    }
    // Set IRQ if UIE is enabled.
   if (s->defs.DIER.UIE)
   {
        qemu_set_irq(s->irq, 1);
   }
}

static void
f2xx_tim_ccmr1(void *arg)
{
    f2xx_tim *s = arg;
	if (!s->defs.CR1.CEN)
		return;
	s->defs.SR.CC1IF |= 1;
   if (s->defs.DIER.CC1IE)
   {
        qemu_set_irq(s->irq, 1);
   }
}

static void f2xx_tim_update_ccr_timer(f2xx_tim *s, int n, uint32_t when)
{
	// Calculate timer tick for CCR event:
	int64_t current_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    // When will we match CCR?
	timer_mod(s->ccrtimer[n-1],  current_time + f2xx_tim_period(s) *(when));
}

// Called if there's an update to ARR without buffering (ARPE=0)
static void
f2xx_arr_update(f2xx_tim *s) {
    if (s->defs.CR1.CEN) {
        if (s->defs.CR1.CMS || s->defs.CR1.DIR) {
            printf("FIXME - non-upcounting ARR updates!\n");
        }
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t current_count = f2xx_tim_ns_to_ticks(s, now) - s->count_timebase;
        int64_t ns_remaining = ((s->defs.ARR+1) - current_count) * f2xx_tim_period(s);
        if (ns_remaining<=0) {
            //printf("ERR: ARR update would have caused timer %u to fire %ld in the past!\n",s->id, ((s->defs.ARR+1) - current_count));
            // Fire at the next tick
            //f2xx_tim_timer(s);
            timer_mod(s->timer, now + f2xx_tim_period(s));
        } else {
            timer_mod(s->timer, now + ns_remaining);
        }
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
    case R_TIM_CNT:
        r = f2xx_tim_ns_to_ticks(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)) - s->count_timebase;
        // printf("Attempted to read count on timer %u (val %u)\n", s->id,r);
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
        case 0x7:
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

static void f2xx_tim_update_pwm(f2xx_tim *s, int n)
{
	if (!s->defs.CR1.CEN)
		return;
	int ratio = -1;
	switch (n)
	{
		case 1:
			if (s->defs.CCER.CC1E && s->defs.CCMR1.CC1S == CCxS_OUTPUT)
				ratio = f2xx_tim_calc_pwm_ratio(s, s->defs.CCR1, s->defs.CCMR1.OC1M, s->defs.CCER.CC1P);
			if (s->defs.CCMR1.OC1M == OCxM_Frozen && s->defs.DIER.CC1IE)
				f2xx_tim_update_ccr_timer(s, 1, s->defs.CCR1);
			break;
		case 2:
			if (s->defs.CCER.CC2E  && s->defs.CCMR1.CC2S == CCxS_OUTPUT)
				ratio = f2xx_tim_calc_pwm_ratio(s, s->defs.CCR2, s->defs.CCMR1.OC2M, s->defs.CCER.CC2P);
			break;
		case 3:
			if (s->defs.CCER.CC3E  && s->defs.CCMR2.CC3S == CCxS_OUTPUT)
				ratio = f2xx_tim_calc_pwm_ratio(s, s->defs.CCR3, s->defs.CCMR2.OC3M, s->defs.CCER.CC3P);
			break;
		case 4:
			if (s->defs.CCER.CC4E && s->defs.CCMR2.CC4S == CCxS_OUTPUT)
				ratio = f2xx_tim_calc_pwm_ratio(s, s->defs.CCR4, s->defs.CCMR2.OC4M, s->defs.CCER.CC4P);
			break;
	}
	if (ratio>=0)
	{
		qemu_set_irq(s->pwm_ratio_changed[n-1], ratio);
	}
}

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

    //if (s->id==5 && addr!=4) printf("Timer%d: Wrote %08lx to %02lx (change mask %08x)\n", s->id, data, addr, changed);

    bool update_required = false;

    switch(addr) {
    case R_TIM_CR1:
        if (data & ~1) {
            qemu_log_mask(LOG_UNIMP, "f2xx tim non-zero CR1 unimplemented\n");
        }
        if ((s->regs[addr] & 1) == 0 && data & 1) {
            printf("f2xx tim started: %u\n", 1U + s->parent.periph - STM32_P_TIM1);
            timer_mod(s->timer, f2xx_tim_next_transition(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
            qemu_set_irq(s->pwm_enable[0], 1);
			s->defs.CR1.CEN = 1;
			f2xx_tim_update_pwm(s, 1);
            printf("pwm en\n");
        } else if (s->regs[addr] & 1 && (data & 1) == 0) {
            timer_del(s->timer);
            qemu_set_irq(s->pwm_enable[0], 0);
            printf("pwm dis\n");

        }
        if (data & 1<<3)
        {
            printf("%u OPM!\n", 1U + s->parent.periph - STM32_P_TIM1);
        }
        s->regs[addr] = data;
        break;
    case R_TIM_SR:
        if ((s->regs[addr] & data) == 0) {
            //printf("f2xx tim clearing int\n");
            qemu_set_irq(s->irq, 0);
        }
        s->regs[addr] &= data;
        break;
	case R_TIM_DIER:
    case R_TIM_EGR:
        qemu_log_mask(LOG_UNIMP, "f2xx tim unimplemented write EGR+%u size %u val 0x%x\n",
          offset, size, (unsigned int)data);
        s->regs[addr] = data;
        break;
    case R_TIM_CCER:
        // Capture/Compare Enable register
       // printf("CCER: %04x\n",data);
        s->regs[addr] = data;

        if (changed & 0x01) {
			f2xx_tim_update_pwm(s, 1);
        }
        if (changed & 0x10) {
			f2xx_tim_update_pwm(s, 2);
        }
        if (changed & 0x100) {
			f2xx_tim_update_pwm(s, 3);
        }
        if (changed & 0x1000){
			f2xx_tim_update_pwm(s, 4);
        }
        break;
    // case R_TIM_CCMR1:
    // case R_TIM_CCMR2:
    // case R_TIM_DIER:
    // case R_TIM_PSC:
    case R_TIM_CCR1:
    case R_TIM_CCR2:
	case R_TIM_CCR3:
    case R_TIM_CCR4:
	{
		// Only timer 5 and 2 are the full 32-bit
        if (s->parent.periph !=STM32_P_TIM5 && s->parent.periph != STM32_P_TIM2) {
            data &= 0xFFFFU;
        }
        s->regs[addr] = data;
		f2xx_tim_update_pwm(s, (addr - R_TIM_CCR1)+1 );
	}
	break;
    case R_TIM_ARR:
        // Check for ARR buffering setting
        update_required = s->defs.CR1.ARPE == 0;
        /* FALLTHRU */
    case R_TIM_CNT:
    default:
        s->regs[addr] = data;
        // printf("f2xx tim unimplemented write 0x%x+%u size %u val 0x%x\n",
        //   (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }
    if (update_required && s->defs.ARR!=0) {
        f2xx_arr_update(s);
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
    s->count_timebase = f2xx_tim_ns_to_ticks(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void f2xx_tim_realize(DeviceState* dev, Error **errp)
{
}


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
	for (int i=0; i<4; i++)
	{
		s->ccrtimer[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL, f2xx_tim_ccmr1, s);
	}
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    qdev_init_gpio_out_named(DEVICE(dev), s->pwm_ratio_changed, "pwm_ratio_changed", 4); // OCx1..4
    qdev_init_gpio_out_named(DEVICE(dev), s->pwm_enable, "pwm_enable", 4);
}

static const VMStateDescription vmstate_stm32f2xx_tim = {
    .name = TYPE_STM32F4XX_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_TIMER_PTR(timer,f2xx_tim),
        VMSTATE_UINT32_ARRAY(regs,f2xx_tim,R_TIM_MAX),
        VMSTATE_INT64(count_timebase,f2xx_tim),
        VMSTATE_END_OF_LIST()
    }
};

static void
f2xx_tim_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32f2xx_tim;
    dc->reset = f2xx_tim_reset;
	dc->realize = f2xx_tim_realize;
}

static const TypeInfo
f2xx_tim_info = {
    .name          = TYPE_STM32F4XX_TIMER,
    .parent        = TYPE_STM32_PERIPHERAL,
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
