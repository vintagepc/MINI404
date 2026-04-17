#pragma once


#include "hw/core/registerfields.h"
#include "hw/core/sysbus.h"
#include "system/block-backend.h"


#define TYPE_ESP32_CROSSCORE_INT "misc.esp32.crosscoreint"
#define ESP32_CROSSCORE_INT(obj) OBJECT_CHECK(Esp32CrosscoreInt, (obj), TYPE_ESP32_CROSSCORE_INT)

typedef struct Esp32CrosscoreInt {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    int n_irqs;
    qemu_irq *irqs;
} Esp32CrosscoreInt;
