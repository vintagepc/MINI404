from stm32_fixupspec import PeripheralFixup, RegisterFixup
from stm32_regtypes import Register


def inject_adc_ccr(periph_map: dict):
    """Add a CCR register to ADC (not defined in the HAL typedef)."""
    if "ADC" not in periph_map:
        return
    periph_map["ADC"]["CCR"] = Register(
        name="CCR", desc="ADC common control register",
        hex_addr="0x0", int_addr=0x0, fields={}, access=None, reset_value=0)


def split_adcc(periph_map: dict):
    """Move ADC.CCR into a separate ADCC peripheral."""
    if "ADC" not in periph_map or "CCR" not in periph_map["ADC"]:
        return
    periph_map.setdefault("ADCC", {})
    periph_map["ADCC"]["CCR"] = periph_map["ADC"].pop("CCR")


# CCR fields active in QEMU: PRESC (clock prescaler) and VREFEN (Vref enable).
ADCC_TYPE_A = PeripheralFixup(registers={
    "CCR": RegisterFixup(fields_unimplemented_except=["PRESC", "VREFEN"]),
})
