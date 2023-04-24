/*
    stm32f4xx_dwt.h - dwt Debug channel for STM32

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
#include "hw/sysbus.h"
#include "../stm32_common/stm32_types.h"
#include "../stm32_common/stm32_shared.h"
#include <assert.h>
#include "../utility/macros.h"
#include "sysemu/cpu-timers.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"

enum RegIndex {
	RI_CTRL,				/*!< Offset: 0x000 (R/W)  Control Register */
	RI_CYCCNT,				/*!< Offset: 0x004 (R/W)  Cycle Count Register */
	RI_CPICNT,				/*!< Offset: 0x008 (R/W)  CPI Count Register */
	RI_EXCCNT,				/*!< Offset: 0x00C (R/W)  Exception Overhead Count Register */
	RI_SLEEPCNT,			/*!< Offset: 0x010 (R/W)  Sleep Count Register */
	RI_LSUCNT,				/*!< Offset: 0x014 (R/W)  LSU Count Register */
	RI_FOLDCNT,				/*!< Offset: 0x018 (R/W)  Folded-instruction Count Register */
	RI_PCSR,				/*!< Offset: 0x01C (R/ )  Program Counter Sample Register */
	RI_COMP0,				/*!< Offset: 0x020 (R/W)  Comparator Register 0 */
	RI_MASK0,				/*!< Offset: 0x024 (R/W)  Mask Register 0 */
	RI_FUNCTION0,			/*!< Offset: 0x028 (R/W)  Function Register 0 */
	RI_COMP1 = 0x30/4,		/*!< Offset: 0x030 (R/W)  Comparator Register 1 */
	RI_MASK1,				/*!< Offset: 0x034 (R/W)  Mask Register 1 */
	RI_FUNCTION1,			/*!< Offset: 0x038 (R/W)  Function Register 1 */
	RI_COMP2 = 0x40/4,		/*!< Offset: 0x040 (R/W)  Comparator Register 2 */
	RI_MASK2,				/*!< Offset: 0x044 (R/W)  Mask Register 2 */
	RI_FUNCTION2,			/*!< Offset: 0x048 (R/W)  Function Register 2 */
	RI_COMP3 = 0x50/4,		/*!< Offset: 0x050 (R/W)  Comparator Register 3 */
	RI_MASK3,				/*!< Offset: 0x054 (R/W)  Mask Register 3 */
	RI_FUNCTION3,			/*!< Offset: 0x058 (R/W)  Function Register 3 */
	RI_END
};

OBJECT_DECLARE_SIMPLE_TYPE(STM32F4xxDWTState, STM32F4xx_DWT)

struct STM32F4xxDWTState {
    SysBusDevice busdev;
    MemoryRegion iomem;

    union{
        struct {
            struct {
                uint32_t CYCCNT_ENA :1;
                REG_R(31);
            } QEMU_PACKED CTRL;
            uint32_t CYCCNT;
            uint32_t CPICNT;
            uint32_t EXCCNT;
            uint32_t SLEEPCNT;
            uint32_t LSUCNT;
            uint32_t FOLDCNT;
            uint32_t PCSR;
            uint32_t COMP0;
            uint32_t MASK0;
            uint32_t FUNCTION0;
            REGDEF_R(0x2C);
            uint32_t COMP1;
            uint32_t MASK1;
            uint32_t FUNCTION1;
            REGDEF_R(0x3C);
            uint32_t COMP2;
            uint32_t MASK2;
            uint32_t FUNCTION2;
            REGDEF_R(0x4C);
            uint32_t COMP3;
            uint32_t MASK3;
            uint32_t FUNCTION3;
        } defs;
        uint32_t raw[RI_END];
    } regs;


    uint64_t cyccnt_start;
};

static uint64_t
stm32f4xx_dwt_read(void *arg, hwaddr offset, unsigned int size)
{
    STM32F4xxDWTState *s = STM32F4xx_DWT(arg);
    if (size !=4) printf("FIXME: DWT READS != 4 bytes\n");
    uint32_t r = 0;

    offset >>= 2;
    switch(offset) {
        case RI_CYCCNT:
			if (s->regs.defs.CTRL.CYCCNT_ENA)
			{
            	r = icount_get_raw() - s->cyccnt_start;
			}
        break;
        default:
            r = s->regs.raw[offset];
            qemu_log_mask(LOG_UNIMP, "STM32F4xxDWT READ from 0x%02x with data %08x not handled.\n", (uint8_t)offset, r);
    }

    return r;
}

static void
stm32f4xx_dwt_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    STM32F4xxDWTState *s = STM32F4xx_DWT(arg);
    int offset = addr % 3;

    addr >>= 2;
    if (addr > RI_END) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid dwt write reg 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }
    switch(size) {
    case 1:
       data = (s->regs.raw[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
       break;
    case 2:
        qemu_log_mask(LOG_UNIMP, "f4xx-dwt - writes !=32bits are not implemented.\n");
        return;
        break;
    case 4:
        break;
    default:
        abort();
    }
    uint32_t changed = s->regs.raw[addr] ^ data;
    uint32_t old = s->regs.raw[addr];
    s->regs.raw[addr] = data;
    (void)old;

    switch (addr) {
        case RI_CTRL:
            if ((changed & 0x1) && s->regs.defs.CTRL.CYCCNT_ENA) {
				if (icount_get_raw() == 0)
				{
					printf("WARNING: -icount (instruction counting) is not enabled. DWT CYCCNT register will not function reliably!\n");
				}
				else
				{
					printf("CYCCNT: 1 instruction is %ld ns\n", icount_to_ns(1));
				}
                s->cyccnt_start = icount_get_raw();
            }
        break;
        case RI_CYCCNT:
            if (data) {
                printf("FIXME: Nonzero Cyccnt set not supported!\n");
            } else {
                s->cyccnt_start = icount_get_raw();
            }
        break;
        default:
            s->regs.raw[addr] = data;
            printf("FIXME: DWT write to 0x%02x with data %08x\n", (uint8_t)addr, (uint32_t)data);
            qemu_log_mask(LOG_UNIMP, "f2xx dwt reg 0x%x:%d write (0x%x) unimplemented\n",
            (int)addr << 2, offset, (int)data);
            break;
    }
}

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(STM32F4xxDWTState, stm32f4xx_dwt, STM32F4xx_DWT, SYS_BUS_DEVICE, {NULL});

static const MemoryRegionOps stm32f4xx_dwt_ops = {
    .read = stm32f4xx_dwt_read,
    .write = stm32f4xx_dwt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4
    }
};

static void
stm32f4xx_dwt_reset(DeviceState *dev)
{
    // stm32f4xx_dwt *s = STM32F4XX_dwt(dev);
}

static void stm32f4xx_dwt_finalize(Object *obj) {

}

static void
stm32f4xx_dwt_init(Object *obj)
{
    STM32F4xxDWTState *s = STM32F4xx_DWT(obj);
    STM32_MR_IO_INIT(&s->iomem, obj, &stm32f4xx_dwt_ops, s, RI_END);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    CHECK_ALIGN(sizeof(s->regs.defs), sizeof(s->regs.raw), "DWT");
    CHECK_ALIGN(sizeof(uint32_t)*RI_END, sizeof(s->regs), "DWT");
}

static const VMStateDescription vmstate_stm32f4xx_dwt = {
    .name = TYPE_STM32F4xx_DWT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs.raw,STM32F4xxDWTState, RI_END),
        // VMSTATE_UINT32_ARRAY(port_buffer,stm32f4xx_dwt, 101),
        // VMSTATE_UINT8(buffer_pos, stm32f4xx_dwt),
        // VMSTATE_BOOL(unlocked, stm32f4xx_dwt),
        VMSTATE_END_OF_LIST()
    }
};

static void
stm32f4xx_dwt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = stm32f4xx_dwt_reset;
    dc->vmsd = &vmstate_stm32f4xx_dwt;
}
