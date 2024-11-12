from stm32_regtypes import *

class stm32f030xx(STM32Fixups):
    @staticmethod
    def post_register_fixups(chip: STM32Chip):
        # For some reason this is a uint8_t in the header and it fails the regex match.
        field = {}
        field["IDR"]=RegisterBitField(name="IDR", desc="Independent data register", shift=0, width=8, permissions=None, unimplemented=False)
        chip.periph_map["CRC"]["IDR"] = Register(name="IDR", desc="CRC independent data register", hex_addr="0x04", int_addr=0x04, fields=field, access=None, reset_value=0)

    @staticmethod
    def supplemental_data(chip: STM32Chip):
        chip.periph_map["CRC"]["DR"].reset_value = 0xFFFFFFFF
        chip.periph_map["CRC"]["INIT"].reset_value = 0xFFFFFFFF
        for field in chip.periph_map["CRC"]["CR"].fields.values():
            if field.name not in ["RESET"]:
                field.unimplemented = True
