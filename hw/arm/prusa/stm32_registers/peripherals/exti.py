from stm32_fixupspec import PeripheralFixup, RegisterFixup

# G070 and C092 share the same EXTI IMR1/EMR1 behaviour.
EXTI_TYPE_A = PeripheralFixup(registers={
    "IMR1": RegisterFixup(reset_value=0xFFF80000, unimplemented=True),
    "EMR1": RegisterFixup(unimplemented=True),
})
