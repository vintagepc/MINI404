/*
    stm32f4xx_iwdg.c - IWDG for STM32F4xx

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32f4xx_iwdg.h"
#include "stm32_rcc.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "sysemu/watchdog.h"
#include "sysemu/runstate.h"
#include "qapi/qapi-commands-run-state.h"
#include "qapi/qapi-events-run-state.h"


//#define DEBUG_STM32F4XX_IWDG
#ifdef DEBUG_STM32F4XX_IWDG
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_STM32F4XX_IWDG: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


#define R_KR 0x00
#define R_PR 0x04/4
#define R_RLR 0x08/4
#define R_SR 0x0c/4

static uint64_t
stm32f4xx_iwdg_read(void *arg, hwaddr addr, unsigned int size)
{
    stm32f4xx_iwdg *s = arg;
    uint32_t r;
    int offset = addr & 0x3;

    addr >>= 2;
    if (addr >= R_IWDG_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid read f4xx_iwdg register 0x%x\n",
          (unsigned int)addr << 2);
        DPRINTF("  %s: result: 0\n", __func__);
        return 0;
    }

    uint32_t value = s->regs.all[addr];

    r = (value >> offset * 8) & ((1ull << (8 * size)) - 1);

    if (addr == R_KR)
    {
        r = 0; // KR is write only. 
    }

    DPRINTF("%s: addr: 0x%lx, size: %u, value: 0x%x\n", __func__, addr, size, r);
    return r;
}

static void stm32f4xx_iwdg_fire(void *opaque) {
    printf("Watchdog fired without guest update, resetting!\n");
    qapi_event_send_watchdog(WATCHDOG_ACTION_RESET);
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    
}

static void stm32f4xx_iwdg_update(stm32f4xx_iwdg *s){
    // Calculate how long this will take...
    if (!s->started){
        return;
    }
    uint32_t clkrate = 32000;
    if (s->rcc != NULL){
        clkrate = stm32_rcc_get_periph_freq(s->rcc, STM32_IWDG);
        if (clkrate == 0)
        {
            qemu_log_mask(LOG_GUEST_ERROR,"ERR: Attempted to enable IWDG with LSI clock disabled!\n");
            return;
        }
    }
    uint32_t prescale = 4<<s->regs.defs.PR.PR;
    uint32_t tickrate = clkrate/prescale; // ticks per second.
    uint64_t delay_us = (1000000U* s->regs.defs.RLR.RL)/tickrate;
    if (s->time_changed) {
        printf("Watchdog configured with timeout of %"PRIu64" ms\n", delay_us/1000U);
        s->time_changed = false;
    }
    timer_mod(s->timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (delay_us*1000));
}

static void
stm32f4xx_iwdg_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    stm32f4xx_iwdg *s = arg;
    int offset = addr & 0x3;

    DPRINTF("%s: addr: 0x%lx, data: 0x%lx, size: %u\n", __func__, addr, data, size);

    addr >>= 2;
    if (addr >= R_IWDG_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write f4xx_iwdg register 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    switch(size) {
    case 2:
        data = (s->regs.all[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
        break;
    case 4:
        break;
    default:
        abort();
    }

    switch(addr) {
    case R_KR:
        s->regs.all[addr] = data;
        switch (s->regs.defs.KR.KEY){
            case 0xCCCC:
                s->started = true;
                /* FALLTHRU */
            case 0xAAAA:
                stm32f4xx_iwdg_update(s);
                break;
            default:
                break;
        }
        break;
    case R_PR:
    case R_RLR:
        if (s->regs.defs.KR.KEY != 0x5555){
            qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write IWDG_PR or RLR while it is write protected!\n");
            return;
        }
        s->regs.all[addr] = data;
        s->time_changed = true;
        stm32f4xx_iwdg_update(s);
        break;
    case R_SR:
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write read-only IWDG_SR!\n");
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f4xx iwdg unimplemented write 0x%x+%u size %u val 0x%x\n",
        (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }
}

static const MemoryRegionOps stm32f4xx_iwdg_ops = {
    .read = stm32f4xx_iwdg_read,
    .write = stm32f4xx_iwdg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 4,
    }
};


static void stm32f4xx_iwdg_reset(DeviceState *dev)
{
    stm32f4xx_iwdg *s = STM32F4XX_IWDG(dev);
    if (s->timer) {
        timer_del(s->timer);
        s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32f4xx_iwdg_fire, s);
    }
    memset(&s->regs, 0, sizeof(s->regs));
    s->regs.defs.RLR.RL = 0xFFF;   
    s->time_changed = true;
    s->started = false;
}


static void
stm32f4xx_iwdg_init(Object *obj)
{
    stm32f4xx_iwdg *s = STM32F4XX_IWDG(obj);

    assert(sizeof(s->regs)==sizeof(s->regs.all)); // Make sure packing is correct.
    CHECK_REG_u32(s->regs.defs.KR);
    CHECK_REG_u32(s->regs.defs.SR);
    CHECK_REG_u32(s->regs.defs.RLR);
    CHECK_REG_u32(s->regs.defs.SR);

    memory_region_init_io(&s->iomem, obj, &stm32f4xx_iwdg_ops, s, "iwdg", 0x0c);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32f4xx_iwdg_fire, s);
}

static const VMStateDescription vmstate_stm32f4xx_iwdg = {
    .name = TYPE_STM32F4XX_IWDG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.all,stm32f4xx_iwdg, R_IWDG_MAX),
        VMSTATE_TIMER_PTR(timer, stm32f4xx_iwdg),
        VMSTATE_BOOL(time_changed,stm32f4xx_iwdg),
        VMSTATE_BOOL(started,stm32f4xx_iwdg),     
        VMSTATE_END_OF_LIST()
    }
};


static void
stm32f4xx_iwdg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32f4xx_iwdg;
    dc->reset = stm32f4xx_iwdg_reset;
}

static const TypeInfo
stm32f4xx_iwdg_info = {
    .name          = TYPE_STM32F4XX_IWDG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(stm32f4xx_iwdg),
    .instance_init = stm32f4xx_iwdg_init,
    .class_init    = stm32f4xx_iwdg_class_init,
};

static void
stm32f4xx_iwdg_register_types(void)
{
    type_register_static(&stm32f4xx_iwdg_info);
}

type_init(stm32f4xx_iwdg_register_types)
