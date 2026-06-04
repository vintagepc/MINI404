from stm32_fixupspec import PeripheralFixup, RegisterFixup

# Applied globally to all chips that expose a TIM peripheral.
TIM_TYPE_A = PeripheralFixup(registers={
    # DMA burst transfer registers: the common timer driver accepts these
    # addresses but does not implement burst DMA (logs an error on access).
    "DCR":   RegisterFixup(unimplemented=True),
    "DMAR":  RegisterFixup(unimplemented=True),
    # Option register: chip-specific alternative-function bits; not used by
    # the common driver.  Present on F427/F030/G070 (absent on C092).
    "OR":    RegisterFixup(unimplemented=True),
    # Extended 6-channel registers present only on G070/C092 advanced timers.
    # The shared driver does not implement these; mask them as unimplemented
    # until the driver is bifurcated into per-variant implementations.
    "CCMR3": RegisterFixup(unimplemented=True),
    "CCR5":  RegisterFixup(unimplemented=True),
    "CCR6":  RegisterFixup(unimplemented=True),
    "AF1":   RegisterFixup(unimplemented=True),
    "AF2":   RegisterFixup(unimplemented=True),
    "TISEL": RegisterFixup(unimplemented=True),
})
