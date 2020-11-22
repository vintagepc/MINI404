#ifndef HW_THERMISTOR_H
#define HW_THERMISTOR_H

#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_THERMISTOR "thermistor"
OBJECT_DECLARE_SIMPLE_TYPE(ThermistorState, THERMISTOR)

struct ThermistorState {
    SysBusDevice parent;

    qemu_irq irq_value;

    int16_t *table;    
    int table_length;
    float temperature;
    int8_t oversampling;
    uint16_t temp;
};

#endif /* HW_THERMISTOR_H */
