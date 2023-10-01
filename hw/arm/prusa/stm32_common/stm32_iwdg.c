/*
    stm32_iwdg.c - IWDG for STM32.
	Currently supports the F4xx, F030, and G070 layouts.

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

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "exec/memory.h"
#include "stm32_common.h"
#include "stm32_shared.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "sysemu/watchdog.h"
#include "sysemu/runstate.h"
#include "qapi/qapi-commands-run-state.h"
#include "qapi/qapi-events-run-state.h"
#include "hw/qdev-properties.h"
#include "stm32_rcc_if.h"
#include "stm32_iwdg_regdata.h"

OBJECT_DECLARE_TYPE(COM_STRUCT_NAME(Iwdg), COM_CLASS_NAME(Iwdg), STM32COM_IWDG);

typedef struct COM_STRUCT_NAME(Iwdg) {
    STM32Peripheral  parent;
    MemoryRegion  iomem;

    union {
        struct {
            REG_S32(KEY, 16) KR;
			REG_S32(PR, 3) PR;
			REG_S32(RL, 12) RLR;
            struct {
				REG_B32(PVU);
                REG_B32(RVU);
                REG_B32(WVU);
                uint32_t :29;
            } QEMU_PACKED SR;
			REG_S32(WIN, 12);
        } QEMU_PACKED defs;
        uint32_t raw[RI_END];
    } QEMU_PACKED regs;

    QEMUTimer *timer;

	const stm32_reginfo_t* reginfo;

    bool time_changed, started;
} COM_STRUCT_NAME(Iwdg);

typedef struct COM_CLASS_NAME(Iwdg) {
	STM32PeripheralClass parent_class;
    stm32_reginfo_t var_reginfo[RI_END];
} COM_CLASS_NAME(Iwdg);


static const stm32_reginfo_t stm32f030_iwdg_reginfo[RI_END] =
{
	[RI_KR] = {.mask = UINT16_MAX },
	[RI_PR] = {.mask = 0b111},
	[RI_RLR] = {.mask = 0xFFFU, .reset_val = 0xFFFU },
	[RI_SR] = {.mask = 0b111},
	[RI_WINR] = { .mask = 0xFFFU, .reset_val = 0xFFFU, .unimp_mask = 0xFFFU},
};

static const stm32_reginfo_t stm32g070_iwdg_reginfo[RI_END] =
{
	[RI_KR] = {.mask = UINT16_MAX },
	[RI_PR] = {.mask = 0b111},
	[RI_RLR] = {.mask = 0xFFFU, .reset_val = 0xFFFU },
	[RI_SR] = {.mask = 0b111},
	[RI_WINR] = { .mask = 0xFFFU, .reset_val = 0xFFFU, .unimp_mask = 0xFFFU},
};

static const stm32_reginfo_t stm32f4xx_iwdg_reginfo[RI_END] =
{
	[RI_KR] = {.mask = UINT16_MAX },
	[RI_PR] = {.mask = 0b111},
	[RI_RLR] = {.mask = 0xFFFU, .reset_val = 0xFFFU },
	[RI_SR] = {.mask = 0b11},
	[RI_WINR] = { .is_reserved = true},
};

static const stm32_periph_variant_t stm32_iwdg_variants[3] = {
	{TYPE_STM32F030_IWDG, stm32f030_iwdg_reginfo},
	{TYPE_STM32G070_IWDG, stm32g070_iwdg_reginfo},
	{TYPE_STM32F4xx_IWDG, stm32f4xx_iwdg_reginfo}
};

static uint64_t
stm32_common_iwdg_read(void *opaque, hwaddr addr, unsigned int size)
{
	COM_STRUCT_NAME(Iwdg) *s = STM32COM_IWDG(opaque);
    int offset = addr & 0x3;

    addr >>= 2;
	CHECK_BOUNDS_R(addr, RI_END, s->reginfo, "STM32 IWDG"); // LCOV_EXCL_LINE

    uint32_t value = s->regs.raw[addr];

	ADJUST_FOR_OFFSET_AND_SIZE_R(value, size, offset, 0b110);

    if (addr == RI_KR)
    {
        value = 0; // KR is write only.
    }
    return value;
}

static void stm32_common_iwdg_fire(void *opaque) {
    printf("# Watchdog fired without guest update, resetting!\n");
    qapi_event_send_watchdog(WATCHDOG_ACTION_RESET);
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);

}

static void stm32_common_iwdg_update(COM_STRUCT_NAME(Iwdg) *s){
    // Calculate how long this will take...
    if (!s->started){
        return;
    }
    uint32_t clkrate = stm32_rcc_if_get_periph_freq(&s->parent);
	if (clkrate == 0)
	{
		qemu_log_mask(LOG_GUEST_ERROR,"ERR: Attempted to enable IWDG with LSI clock disabled!\n");
		return;
	}

    uint32_t prescale = IWDG_PRESCALES[s->regs.defs.PR.PR];
    uint32_t tickrate = clkrate/prescale; // ticks per second.
    uint64_t delay_us = (1000000U * s->regs.defs.RLR.RL)/tickrate;
    if (s->time_changed) {
		uint64_t time = delay_us;
		if (time > 1000)
		{
			time /= 1000U;
		}
        printf("# Watchdog configured with timeout of %"PRIu64" %s\n", time, delay_us>1000? "ms": "us");
        s->time_changed = false;
    }
    timer_mod(s->timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (delay_us*1000));
}

static void
stm32_common_iwdg_write(void *opaque, hwaddr addr, uint64_t data, unsigned int size)
{
	COM_STRUCT_NAME(Iwdg) *s = STM32COM_IWDG(opaque);

    int offset = addr & 0x3;

    addr >>= 2;
	CHECK_BOUNDS_W(addr, data, RI_END, s->reginfo, "STM32 IWDG"); // LCOV_EXCL_LINE

	ADJUST_FOR_OFFSET_AND_SIZE_W(s->regs.raw[addr], data, size, offset, 0b110);

    switch(addr) {
    case RI_KR:
		ENFORCE_RESERVED(data, s->reginfo, RI_KR);
        s->regs.raw[addr] = data;
        switch (s->regs.defs.KR.KEY){
            case 0xCCCC:
                s->started = true;
                /* FALLTHRU */
            case 0xAAAA:
                stm32_common_iwdg_update(s);
                break;
            default:
                break;
        }
        break;
    case RI_PR:
    case RI_RLR:
		ENFORCE_RESERVED(data, s->reginfo, addr);
        if (s->regs.defs.KR.KEY != 0x5555){
            qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write IWDG_PR or RLR while it is write protected!\n");
            return;
        }
        s->regs.raw[addr] = data;
        s->time_changed = true;
        stm32_common_iwdg_update(s);
        break;
    case RI_SR:
        qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write read-only IWDG_SR!\n");
        break;
	case RI_WINR:
		CHECK_UNIMP_RESVD(data, s->reginfo, RI_WINR);
		if (s->regs.defs.KR.KEY != 0x5555){
        	qemu_log_mask(LOG_GUEST_ERROR,"Attempted to write IWDG_WINR while it is write protected!\n");
            return;
        }
		s->regs.raw[addr] = data;
		break;
    default:
        qemu_log_mask(LOG_UNIMP, "STM32 iwdg unimplemented write 0x%x+%u size %u val 0x%x\n",
        (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }
}

static const MemoryRegionOps stm32_common_iwdg_ops = {
    .read = stm32_common_iwdg_read,
    .write = stm32_common_iwdg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 4,
    }
};


static void stm32_common_iwdg_reset(DeviceState *dev)
{
	COM_STRUCT_NAME(Iwdg) *s = STM32COM_IWDG(dev);
    if (s->timer) {
        timer_del(s->timer);
        g_free(s->timer);
        s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32_common_iwdg_fire, s);
    }
	for (int i=0;i<RI_END; i++)
	{
		s->regs.raw[i] = s->reginfo[i].reset_val;
	}
    s->time_changed = true;
    s->started = false;
}

static void
stm32_common_iwdg_init(Object *obj)
{
	COM_STRUCT_NAME(Iwdg) *s = STM32COM_IWDG(obj);


    assert(sizeof(s->regs)==sizeof(s->regs.raw)); // Make sure packing is correct.
    CHECK_REG_u32(s->regs.defs.KR);
    CHECK_REG_u32(s->regs.defs.SR);
    CHECK_REG_u32(s->regs.defs.RLR);
    CHECK_REG_u32(s->regs.defs.SR);

    STM32_MR_IO_INIT(&s->iomem, obj, &stm32_common_iwdg_ops, s, 1U*KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, stm32_common_iwdg_fire, s);

	COM_CLASS_NAME(Iwdg) *k = STM32COM_IWDG_GET_CLASS(obj);

	s->reginfo = k->var_reginfo;

}

static const VMStateDescription vmstate_stm32_common_iwdg = {
    .name = TYPE_STM32COM_IWDG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,COM_STRUCT_NAME(Iwdg), RI_END),
        VMSTATE_TIMER_PTR(timer, COM_STRUCT_NAME(Iwdg)),
        VMSTATE_BOOL(time_changed,COM_STRUCT_NAME(Iwdg)),
        VMSTATE_BOOL(started,COM_STRUCT_NAME(Iwdg)),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32_common_iwdg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_stm32_common_iwdg;
    dc->reset = stm32_common_iwdg_reset;

	COM_CLASS_NAME(Iwdg) *k = STM32COM_IWDG_CLASS(klass);
	memcpy(k->var_reginfo, data, sizeof(k->var_reginfo));
	QEMU_BUILD_BUG_MSG(sizeof(k->var_reginfo) != sizeof(stm32_reginfo_t[RI_END]), "Reginfo not sized correctly!");
}

static const TypeInfo
stm32_common_iwdg_info = {
    .name          = TYPE_STM32COM_IWDG,
    .parent        = TYPE_STM32_PERIPHERAL,
    .instance_size = sizeof(COM_STRUCT_NAME(Iwdg)),
	.class_size    = sizeof(COM_CLASS_NAME(Iwdg)),
	.abstract	   = true,
};

static void
stm32_common_iwdg_register_types(void)
{
    type_register_static(&stm32_common_iwdg_info);
	for (int i = 0; i < ARRAY_SIZE(stm32_iwdg_variants); ++i) {
        TypeInfo ti = {
            .name       = stm32_iwdg_variants[i].variant_name,
            .parent     = TYPE_STM32COM_IWDG,
			.instance_init = stm32_common_iwdg_init,
    		.class_init    = stm32_common_iwdg_class_init,
            .class_data = (void *)stm32_iwdg_variants[i].variant_regs,
        };
        type_register(&ti);
    }

}

type_init(stm32_common_iwdg_register_types)
