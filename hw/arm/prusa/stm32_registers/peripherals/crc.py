from stm32_fixupspec import PeripheralFixup, RegisterFixup

CRC_TYPE_A = PeripheralFixup(registers={
    "DR":   RegisterFixup(reset_value=0xFFFFFFFF),
    "INIT": RegisterFixup(reset_value=0xFFFFFFFF),
    "POL":  RegisterFixup(reset_value=0x04C11DB7),
    "CR":   RegisterFixup(fields_unimplemented_except=["RESET"]),
})

# H503 uses the same CRC core but adds IP-versioning registers not present
# on other devices; strip them so they don't appear in the generated output.
CRC_H503 = CRC_TYPE_A.extend(
    HWCFGR=RegisterFixup(remove=True),
    VERR=RegisterFixup(remove=True),
    PIDR=RegisterFixup(remove=True),
    SIDR=RegisterFixup(remove=True),
)
