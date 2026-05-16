from stm32_fixupspec import PeripheralFixup, RegisterFixup

# ISR is absent on F4xx (which uses SR1/SR2 instead); apply_peripheral_fixup
# silently skips missing registers so this spec is safe to apply to all chips.
I2C_TYPE_A = PeripheralFixup(registers={
    "ISR":      RegisterFixup(reset_value=0x00000001),
    "OAR1":     RegisterFixup(unimplemented=True),
    "OAR2":     RegisterFixup(unimplemented=True),
    "TIMINGR":  RegisterFixup(unimplemented=True),
    "PECR":     RegisterFixup(unimplemented=True),
    "TIMEOUTR": RegisterFixup(unimplemented=True),
})
