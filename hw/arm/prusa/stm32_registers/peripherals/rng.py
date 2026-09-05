from stm32_fixupspec import PeripheralFixup, RegisterFixup

# DR is a single full-width field; ST doesn't define Pos/Msk for it.
# HTCR is only present on some chips (e.g. H503); missing registers are silently skipped.
RNG_TYPE_A = PeripheralFixup(registers={
    "DR":   RegisterFixup(full_width=True),
    "HTCR": RegisterFixup(unimplemented=True),
    "CR":   RegisterFixup(fields_unimplemented_except=["RNGEN", "IE"]),
    "SR":   RegisterFixup(fields_unimplemented=["SECS", "SEIS"]),
})

# H503-specific reset values; apply before RNG_TYPE_A so field marking runs after.
RNG_H503 = PeripheralFixup(registers={
    "CR":   RegisterFixup(reset_value=0x0080D00),
    "HTCR": RegisterFixup(reset_value=0x000072AC),
})
