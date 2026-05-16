from stm32_regtypes import *
from stm32_fixupspec import apply_peripheral_fixup
from peripherals.rng import RNG_TYPE_A

class stm32f427xx(STM32Fixups):
    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        # IDR is a uint8_t in the header; inject manually so it is captured.
        chip.periph_map["CRC"]["IDR"] = Register(name="IDR", desc="CRC independent data register", hex_addr="0x04", int_addr=0x04, fields={}, access=None, reset_value=0)

    @staticmethod
    def supplemental_data(chip: STM32Chip):
        chip.periph_map["CRC"]["DR"].reset_value = 0xFFFFFFFF
        apply_peripheral_fixup(chip.periph_map, "RNG", RNG_TYPE_A)
